/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <array>
#include <cstring>
#include <rda5807m.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fm_tuner, LOG_LEVEL_INF);

#define BT_UUID_FM_SERVICE_VAL BT_UUID_128_ENCODE(0x12345678, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_FREQ_VAL    BT_UUID_128_ENCODE(0x12345678, 0x0001, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_VOL_VAL     BT_UUID_128_ENCODE(0x12345678, 0x0002, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_SEEK_VAL    BT_UUID_128_ENCODE(0x12345678, 0x0003, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_MUTE_VAL    BT_UUID_128_ENCODE(0x12345678, 0x0004, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_STATUS_VAL  BT_UUID_128_ENCODE(0x12345678, 0x0005, 0x1000, 0x8000, 0x00805F9B34FB)

#define RDA5807M_SHADOW_COUNT    6
#define RDA5807M_TUNE_TIMEOUT_MS 1000
#define RDA5807M_TUNE_POLL_MS    10
#define RDA5807M_SEEK_TIMEOUT_MS 5000
#define RDA5807M_RESET_DELAY_MS  150
#define RDA5807M_I2C_ADDR        0x10

#define SHADOW_CONFIG  0
#define SHADOW_CHANNEL 1
#define SHADOW_GPIO    2
#define SHADOW_VOLUME  3
#define SHADOW_OPEN    4
#define SHADOW_BLEND   5

static const struct bt_uuid_128 uuid_fm_service = BT_UUID_INIT_128(BT_UUID_FM_SERVICE_VAL);
static const struct bt_uuid_128 uuid_fm_freq = BT_UUID_INIT_128(BT_UUID_FM_FREQ_VAL);
static const struct bt_uuid_128 uuid_fm_vol = BT_UUID_INIT_128(BT_UUID_FM_VOL_VAL);
static const struct bt_uuid_128 uuid_fm_seek = BT_UUID_INIT_128(BT_UUID_FM_SEEK_VAL);
static const struct bt_uuid_128 uuid_fm_mute = BT_UUID_INIT_128(BT_UUID_FM_MUTE_VAL);
static const struct bt_uuid_128 uuid_fm_status = BT_UUID_INIT_128(BT_UUID_FM_STATUS_VAL);

static const struct i2c_dt_spec radio_i2c = I2C_DT_SPEC_GET(DT_ALIAS(radio0));

struct rda5807m_radio {
    struct i2c_dt_spec i2c;
    uint16_t shadow[RDA5807M_SHADOW_COUNT];
    uint32_t frequency_khz;
    struct k_mutex lock;
};

static struct rda5807m_radio radio = {
    .i2c = radio_i2c,
    .shadow = {},
    .frequency_khz = 101000U,
};

static uint8_t current_vol = 8U;
static bool current_mute = false;
static uint32_t current_freq_khz = 101000U;

static struct bt_conn *active_conn = nullptr;
static bool status_notify_enabled = false;
static const struct bt_gatt_attr *status_attr = nullptr;
static const struct bt_gatt_attr *freq_attr = nullptr;

static struct k_work_q radio_workq;
static K_THREAD_STACK_DEFINE(radio_workq_stack, 2048);
static struct k_work seek_work;
static bool seek_direction = true;

static int rda5807m_seek(struct rda5807m_radio *dev, bool up);
static int rda5807m_get_status(struct rda5807m_radio *dev, struct rda5807m_status *status);

static void seek_work_handler(struct k_work *work) {
    int ret = rda5807m_seek(&radio, seek_direction);
    if (ret == 0) {
        struct rda5807m_status status = {};
        if (rda5807m_get_status(&radio, &status) == 0) {
            current_freq_khz = status.frequency_khz;
        }
    } else if (ret == -EIO) {
        LOG_WRN("Seek: no station found");
    } else {
        LOG_ERR("seek failed: %d", ret);
    }

    if (freq_attr) {
        uint32_t freq_be = __builtin_bswap32(radio.frequency_khz);
        bt_gatt_notify(nullptr, freq_attr, &freq_be, sizeof(freq_be));
    }
}

static int rda5807m_write_regs(struct rda5807m_radio *dev, uint8_t count) {
    if (count > RDA5807M_SHADOW_COUNT) {
        return -EINVAL;
    }

    uint8_t buff[RDA5807M_SHADOW_COUNT * 2] = {0};

    for (int i = 0; i < count; i++) {
        buff[i * 2] = (dev->shadow[i] >> 8) & 0xFF;
        buff[i * 2 + 1] = dev->shadow[i] & 0xFF;
    }

    return i2c_write_dt(&dev->i2c, buff, count * 2);
}

static int rda5807m_read_status_raw(struct rda5807m_radio *dev, uint16_t *status_a, uint16_t *status_b) {
    uint8_t buff[4] = {0};

    int ret = i2c_read_dt(&dev->i2c, buff, sizeof(buff));
    if (ret) {
        LOG_ERR("Status read failed: %d", ret);
        return ret;
    }

    *status_a = ((uint16_t) buff[0] << 8) | buff[1];
    *status_b = ((uint16_t) buff[2] << 8) | buff[3];
    return 0;
}

static int rda5807m_wait_stc(struct rda5807m_radio *dev, uint32_t timeout_ms, uint16_t *status_a_out,
                             uint16_t *status_b_out) {
    uint16_t status_a;
    uint16_t status_b;
    uint32_t elapsed = 0;

    while (elapsed < timeout_ms) {
        k_msleep(RDA5807M_TUNE_POLL_MS);
        elapsed += RDA5807M_TUNE_POLL_MS;

        int ret = rda5807m_read_status_raw(dev, &status_a, &status_b);
        if (ret) {
            return ret;
        }

        if (elapsed <= 50) {
            LOG_INF("Poll %u ms: 0x0A=0x%04X 0x0B=0x%04X STC=%d SF=%d", elapsed, status_a, status_b,
                    (status_a & RDA5807M_ST_STC) != 0, (status_a & RDA5807M_ST_SF) != 0);
        }

        if (status_a & RDA5807M_ST_STC) {
            if (status_a_out) *status_a_out = status_a;
            if (status_b_out) *status_b_out = status_b;
            return 0;
        }
    }

    LOG_WRN("STC timeout after %u ms — last: 0x0A=0x%04X 0x0B=0x%04X", timeout_ms, status_a, status_b);
    return -ETIMEDOUT;
}

static int rda5807m_set_frequency(struct rda5807m_radio *dev, uint32_t freq_khz) {
    int ret = 0;

    if (freq_khz < RDA5807M_FREQ_MIN_KHZ || freq_khz > RDA5807M_FREQ_MAX_KHZ) {
        LOG_ERR("Frequency %u kHz out of range", freq_khz);
        return -EINVAL;
    }

    freq_khz = (freq_khz / RDA5807M_FREQ_STEP_KHZ) * RDA5807M_FREQ_STEP_KHZ;
    uint16_t channel = (freq_khz - RDA5807M_FREQ_BASE_KHZ) / RDA5807M_FREQ_STEP_KHZ;

    k_mutex_lock(&dev->lock, K_FOREVER);

    dev->shadow[SHADOW_CHANNEL] =
        (channel << RDA5807M_CHAN_SHIFT) | RDA5807M_CHAN_TUNE | RDA5807M_CHAN_BAND_87108 | RDA5807M_CHAN_SPACE_100K;

    ret = rda5807m_write_regs(dev, 2);
    if (ret) {
        LOG_ERR("Frequency write failed: %d", ret);
        goto cleanup;
    }

    ret = rda5807m_wait_stc(dev, RDA5807M_TUNE_TIMEOUT_MS, nullptr, nullptr);
    if (ret == -ETIMEDOUT) {
        LOG_WRN("Tune timed out, resetting chip");
        dev->shadow[SHADOW_CONFIG] = RDA5807M_CFG_SOFT_RESET | RDA5807M_CFG_ENABLE | RDA5807M_CFG_NEW_METHOD;
        rda5807m_write_regs(dev, 1);
        k_msleep(50);
        dev->shadow[SHADOW_CONFIG] =
            RDA5807M_CFG_DHIZ | RDA5807M_CFG_DMUTE | RDA5807M_CFG_NEW_METHOD | RDA5807M_CFG_ENABLE;
        dev->shadow[SHADOW_CHANNEL] =
            (channel << RDA5807M_CHAN_SHIFT) | RDA5807M_CHAN_TUNE | RDA5807M_CHAN_BAND_87108 | RDA5807M_CHAN_SPACE_100K;
        rda5807m_write_regs(dev, 4);
        k_msleep(50);
        dev->shadow[SHADOW_CHANNEL] &= ~RDA5807M_CHAN_TUNE;
        rda5807m_write_regs(dev, 2);

        dev->frequency_khz = freq_khz;
        goto cleanup;
    } else if (ret) {
        goto cleanup;
    }

    dev->frequency_khz = freq_khz;
    LOG_INF("Tuned to %u kHz", freq_khz);

cleanup:
    dev->shadow[SHADOW_CHANNEL] &= ~RDA5807M_CHAN_TUNE;
    int write_ret = rda5807m_write_regs(dev, 2);
    if (write_ret) {
        LOG_ERR("TUNE bit clear failed: %d", write_ret);
        ret = write_ret;
    }

    k_mutex_unlock(&dev->lock);
    return ret;
}

static int rda5807m_set_volume(struct rda5807m_radio *dev, uint8_t vol) {
    if (vol > RDA5807M_VOL_MAX) {
        LOG_ERR("Volume %u out of range (max %u)", vol, RDA5807M_VOL_MAX);
        return -EINVAL;
    }

    k_mutex_lock(&dev->lock, K_FOREVER);
    dev->shadow[SHADOW_VOLUME] = (dev->shadow[SHADOW_VOLUME] & ~RDA5807M_VOL_MASK) | vol;
    int ret = rda5807m_write_regs(dev, 4);
    if (ret) {
        LOG_ERR("Volume write failed: %d", ret);
    }
    k_mutex_unlock(&dev->lock);
    return ret;
}

static int rda5807m_set_mute(struct rda5807m_radio *dev, bool mute) {
    k_mutex_lock(&dev->lock, K_FOREVER);

    if (mute) {
        dev->shadow[SHADOW_CONFIG] &= ~RDA5807M_CFG_DMUTE;
    } else {
        dev->shadow[SHADOW_CONFIG] |= RDA5807M_CFG_DMUTE;
    }

    int ret = rda5807m_write_regs(dev, 1);
    if (ret) {
        LOG_ERR("Mute write failed: %d", ret);
    }

    k_mutex_unlock(&dev->lock);
    return ret;
}

static int rda5807m_seek(struct rda5807m_radio *dev, bool up) {
    uint16_t status_a = 0;
    uint16_t status_b = 0;
    int ret = 0;

    k_mutex_lock(&dev->lock, K_FOREVER);

    dev->shadow[SHADOW_CONFIG] |= RDA5807M_CFG_SEEK;
    if (up) {
        dev->shadow[SHADOW_CONFIG] |= RDA5807M_CFG_SEEKUP;
    } else {
        dev->shadow[SHADOW_CONFIG] &= ~RDA5807M_CFG_SEEKUP;
    }

    ret = rda5807m_write_regs(dev, 1);
    if (ret) {
        LOG_ERR("Seek write failed: %d", ret);
        goto cleanup;
    }

    ret = rda5807m_wait_stc(dev, RDA5807M_SEEK_TIMEOUT_MS, &status_a, &status_b);
    if (ret == -ETIMEDOUT) {
        LOG_WRN("Seek timed out, resetting chip");
        dev->shadow[SHADOW_CONFIG] = RDA5807M_CFG_SOFT_RESET | RDA5807M_CFG_ENABLE | RDA5807M_CFG_NEW_METHOD;
        rda5807m_write_regs(dev, 1);
        k_msleep(50);
        dev->shadow[SHADOW_CONFIG] =
            RDA5807M_CFG_DHIZ | RDA5807M_CFG_DMUTE | RDA5807M_CFG_NEW_METHOD | RDA5807M_CFG_ENABLE;
        rda5807m_write_regs(dev, 4);

        uint16_t ch = (dev->frequency_khz - RDA5807M_FREQ_BASE_KHZ) / RDA5807M_FREQ_STEP_KHZ;
        dev->shadow[SHADOW_CHANNEL] =
            (ch << RDA5807M_CHAN_SHIFT) | RDA5807M_CHAN_TUNE | RDA5807M_CHAN_BAND_87108 | RDA5807M_CHAN_SPACE_100K;
        rda5807m_write_regs(dev, 2);
        k_msleep(50);
        dev->shadow[SHADOW_CHANNEL] &= ~RDA5807M_CHAN_TUNE;
        rda5807m_write_regs(dev, 2);

        goto cleanup;
    } else if (ret) {
        goto cleanup;
    }

    if (status_a & RDA5807M_ST_SF) {
        LOG_WRN("Seek failed - no station found");
        ret = -EIO;
    } else {
        uint16_t channel = (status_a & RDA5807M_ST_READCHAN_MASK) >> RDA5807M_ST_READCHAN_SHIFT;
        dev->frequency_khz = ((uint32_t) channel * RDA5807M_FREQ_STEP_KHZ) + RDA5807M_FREQ_BASE_KHZ;
        LOG_INF("Seek landed at %u kHz", dev->frequency_khz);
    }

cleanup:
    dev->shadow[SHADOW_CONFIG] &= ~RDA5807M_CFG_SEEK;
    int write_ret = rda5807m_write_regs(dev, 1);
    if (write_ret) {
        LOG_ERR("SEEK bit clear failed: %d", write_ret);
        ret = write_ret;
    }

    k_mutex_unlock(&dev->lock);
    return ret;
}

static int rda5807m_get_status(struct rda5807m_radio *dev, struct rda5807m_status *status) {
    uint16_t sa, sb;

    if (!status) {
        return -EINVAL;
    }

    int ret = rda5807m_read_status_raw(dev, &sa, &sb);
    if (ret) {
        return ret;
    }

    status->frequency_khz = dev->frequency_khz;
    status->rssi = (sb & RDA5807M_ST_RSSI_MASK) >> RDA5807M_ST_RSSI_SHIFT;
    status->stereo = (sa & RDA5807M_ST_STEREO) != 0;
    status->station = (sb & RDA5807M_ST_FM_TRUE) != 0;
    status->seek_fail = (sa & RDA5807M_ST_SF) != 0;

    return 0;
}

static int rda5807m_init(struct rda5807m_radio *dev) {
    int ret = 0;

    k_mutex_init(&dev->lock);

    if (!i2c_is_ready_dt(&dev->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    dev->shadow[SHADOW_CONFIG] = RDA5807M_CFG_SOFT_RESET | RDA5807M_CFG_ENABLE | RDA5807M_CFG_NEW_METHOD;
    ret = rda5807m_write_regs(dev, 1);
    if (ret) {
        LOG_ERR("Reset write failed: %d", ret);
        return ret;
    }
    k_msleep(RDA5807M_RESET_DELAY_MS);

    dev->shadow[SHADOW_CONFIG] = RDA5807M_CFG_DHIZ | RDA5807M_CFG_DMUTE | RDA5807M_CFG_NEW_METHOD | RDA5807M_CFG_ENABLE;
    dev->shadow[SHADOW_CHANNEL] = RDA5807M_CHAN_BAND_87108 | RDA5807M_CHAN_SPACE_100K;
    dev->shadow[SHADOW_VOLUME] = (RDA5807M_SEEKTH_DEFAULT << RDA5807M_SEEKTH_SHIFT) | 8U;

    ret = rda5807m_write_regs(dev, 4);
    if (ret) {
        LOG_ERR("Init write failed: %d", ret);
        return ret;
    }

    dev->frequency_khz = RDA5807M_FREQ_MIN_KHZ;
    LOG_INF("RDA5807M initialized");
    return 0;
}

static ssize_t read_freq(struct bt_conn *, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    uint32_t freq_be = __builtin_bswap32(radio.frequency_khz);
    return bt_gatt_attr_read(nullptr, attr, buf, len, offset, &freq_be, sizeof(freq_be));
}

static ssize_t write_freq(struct bt_conn *, const struct bt_gatt_attr *, const void *buf, uint16_t len, uint16_t,
                          uint8_t) {
    if (len != sizeof(uint32_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint32_t freq_be;
    memcpy(&freq_be, buf, sizeof(freq_be));
    uint32_t freq_khz = __builtin_bswap32(freq_be);

    if (int ret = rda5807m_set_frequency(&radio, freq_khz); ret) {
        LOG_ERR("set_frequency(%u) failed: %d", freq_khz, ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    current_freq_khz = freq_khz;
    LOG_INF("Tuned to %u kHz", freq_khz);
    return len;
}

static ssize_t read_vol(struct bt_conn *, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(nullptr, attr, buf, len, offset, &current_vol, sizeof(current_vol));
}

static ssize_t write_vol(struct bt_conn *, const struct bt_gatt_attr *, const void *buf, uint16_t len, uint16_t,
                         uint8_t) {
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *bytes = static_cast<const uint8_t *>(buf);
    uint8_t vol = bytes[0];
    if (int ret = rda5807m_set_volume(&radio, vol); ret) {
        LOG_ERR("set_volume(%u) failed: %d", vol, ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    current_vol = vol;
    return len;
}

static ssize_t write_seek(struct bt_conn *, const struct bt_gatt_attr *, const void *buf, uint16_t len, uint16_t,
                          uint8_t) {
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *bytes = static_cast<const uint8_t *>(buf);
    uint8_t dir = bytes[0];
    if (dir != 0x01 && dir != 0x02) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    seek_direction = (dir == 0x01);
    k_work_submit_to_queue(&radio_workq, &seek_work);
    return len;
}

static ssize_t read_mute(struct bt_conn *, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    uint8_t mute_byte = current_mute ? 1U : 0U;
    return bt_gatt_attr_read(nullptr, attr, buf, len, offset, &mute_byte, sizeof(mute_byte));
}

static ssize_t write_mute(struct bt_conn *, const struct bt_gatt_attr *, const void *buf, uint16_t len, uint16_t,
                          uint8_t) {
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *bytes = static_cast<const uint8_t *>(buf);
    uint8_t val = bytes[0];
    bool new_mute = (val == 0xFF) ? !current_mute : (val != 0);

    if (int ret = rda5807m_set_mute(&radio, new_mute); ret) {
        LOG_ERR("set_mute(%d) failed: %d", new_mute, ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    current_mute = new_mute;
    LOG_INF("Mute: %s", current_mute ? "on" : "off");
    return len;
}

static void status_ccc_changed(const struct bt_gatt_attr *, uint16_t value) {
    status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Status notifications %s", status_notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(
    fm_radio_svc, BT_GATT_PRIMARY_SERVICE(&uuid_fm_service),
    BT_GATT_CHARACTERISTIC(&uuid_fm_freq.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_freq, write_freq, nullptr),
    BT_GATT_CCC(nullptr, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&uuid_fm_vol.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_vol, write_vol, nullptr),
    BT_GATT_CHARACTERISTIC(&uuid_fm_seek.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, nullptr, write_seek, nullptr),
    BT_GATT_CHARACTERISTIC(&uuid_fm_mute.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_mute, write_mute, nullptr),
    BT_GATT_CHARACTERISTIC(&uuid_fm_status.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, nullptr, nullptr, nullptr),
    BT_GATT_CCC(status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

static const std::array<bt_data, 2> ad = {{
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_FM_SERVICE_VAL),
}};

static const std::array<bt_data, 1> sd = {{
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
}};

static int start_advertising() {
    int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad.data(), ad.size(), sd.data(), sd.size());
    if (ret) {
        LOG_ERR("Advertising start failed: %d", ret);
    }
    return ret;
}

static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed: 0x%02x", err);
        return;
    }
    active_conn = bt_conn_ref(conn);
    LOG_INF("BLE connected");
}

static void on_disconnected(struct bt_conn *, uint8_t reason) {
    LOG_INF("BLE disconnected, reason 0x%02x", reason);
    if (active_conn) {
        bt_conn_unref(active_conn);
        active_conn = nullptr;
    }
    status_notify_enabled = false;
    start_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

constexpr int STATUS_NOTIFY_STACK = 1024;
constexpr int STATUS_NOTIFY_PRIO = K_PRIO_PREEMPT(7);
constexpr k_timeout_t STATUS_NOTIFY_PERIOD = K_MSEC(2000);

static void status_notify_thread_fn(void *, void *, void *) {
    while (true) {
        k_sleep(STATUS_NOTIFY_PERIOD);

        if (!active_conn || !status_notify_enabled || !status_attr) {
            continue;
        }

        struct rda5807m_status status = {};
        if (rda5807m_get_status(&radio, &status) != 0) {
            continue;
        }

        struct __packed {
            uint8_t rssi;
            uint8_t stereo;
        } payload = {
            .rssi = status.rssi,
            .stereo = static_cast<uint8_t>(status.stereo ? 1U : 0U),
        };

        int ret = bt_gatt_notify(nullptr, status_attr, &payload, sizeof(payload));
        if (ret && ret != -ENOTCONN) {
            LOG_WRN("Notify failed: %d", ret);
        }
    }
}

K_THREAD_DEFINE(status_notify_tid, STATUS_NOTIFY_STACK, status_notify_thread_fn, nullptr, nullptr, nullptr,
                STATUS_NOTIFY_PRIO, 0, 0);

int main() {
    if (!device_is_ready(radio.i2c.bus)) {
        LOG_ERR("RDA5807M I2C bus not ready");
        return -ENODEV;
    }

    current_freq_khz = radio.frequency_khz;

    int ret = rda5807m_init(&radio);
    if (ret) {
        LOG_ERR("RDA5807M init failed: %d", ret);
        return ret;
    }

    LOG_INF("RDA5807M ready");

    k_work_queue_init(&radio_workq);
    k_work_queue_start(&radio_workq, radio_workq_stack, K_THREAD_STACK_SIZEOF(radio_workq_stack), K_PRIO_PREEMPT(5),
                       nullptr);
    k_work_init(&seek_work, seek_work_handler);

    ret = bt_enable(nullptr);
    if (ret) {
        LOG_ERR("BT enable failed: %d", ret);
        return ret;
    }

    LOG_INF("Bluetooth ready");

    status_attr = bt_gatt_find_by_uuid(fm_radio_svc.attrs, fm_radio_svc.attr_count, &uuid_fm_status.uuid);
    if (!status_attr) {
        LOG_ERR("Status attr not found — check service table");
        return -ENOENT;
    }

    freq_attr = bt_gatt_find_by_uuid(fm_radio_svc.attrs, fm_radio_svc.attr_count, &uuid_fm_freq.uuid);
    if (!freq_attr) {
        LOG_ERR("Freq attr not found — check service table");
        return -ENOENT;
    }

    ret = start_advertising();
    if (ret) {
        return ret;
    }

    LOG_INF("Advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
    return 0;
}