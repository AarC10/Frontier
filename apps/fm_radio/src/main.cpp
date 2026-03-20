/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rda5807m.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fm_tuner, LOG_LEVEL_INF);

#define BT_UUID_FM_SERVICE_VAL BT_UUID_128_ENCODE(0x12345678, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_FREQ_VAL    BT_UUID_128_ENCODE(0x12345678, 0x0001, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_VOL_VAL     BT_UUID_128_ENCODE(0x12345678, 0x0002, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_SEEK_VAL    BT_UUID_128_ENCODE(0x12345678, 0x0003, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_MUTE_VAL    BT_UUID_128_ENCODE(0x12345678, 0x0004, 0x1000, 0x8000, 0x00805F9B34FB)
#define BT_UUID_FM_STATUS_VAL  BT_UUID_128_ENCODE(0x12345678, 0x0005, 0x1000, 0x8000, 0x00805F9B34FB)

static struct bt_uuid_128 uuid_fm_service = BT_UUID_INIT_128(BT_UUID_FM_SERVICE_VAL);
static struct bt_uuid_128 uuid_fm_freq = BT_UUID_INIT_128(BT_UUID_FM_FREQ_VAL);
static struct bt_uuid_128 uuid_fm_vol = BT_UUID_INIT_128(BT_UUID_FM_VOL_VAL);
static struct bt_uuid_128 uuid_fm_seek = BT_UUID_INIT_128(BT_UUID_FM_SEEK_VAL);
static struct bt_uuid_128 uuid_fm_mute = BT_UUID_INIT_128(BT_UUID_FM_MUTE_VAL);
static struct bt_uuid_128 uuid_fm_status = BT_UUID_INIT_128(BT_UUID_FM_STATUS_VAL);

static const struct device *radio = DEVICE_DT_GET(DT_NODELABEL(rda5807m));

static uint32_t current_freq_khz = 101000U;
static uint8_t current_vol = 8U;
static bool current_mute = false;

static struct bt_conn *active_conn = nullptr;
static bool status_notify_enabled = false;
static const struct bt_gatt_attr *status_attr = nullptr;

struct __attribute__((packed)) fm_status_t {
    uint8_t rssi;
    uint8_t stereo;
};

static ssize_t read_freq(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                         uint16_t offset) {
    uint32_t freq_be = __builtin_bswap32(current_freq_khz);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &freq_be, sizeof(freq_be));
}

static ssize_t write_freq(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags) {
    if (len != sizeof(uint32_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint32_t freq_be;
    memcpy(&freq_be, buf, sizeof(freq_be));
    uint32_t freq_khz = __builtin_bswap32(freq_be);

    int ret = rda5807m_set_frequency(radio, freq_khz);
    if (ret) {
        LOG_ERR("set_frequency(%u) failed: %d", freq_khz, ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    current_freq_khz = freq_khz;
    LOG_INF("Tuned to %u kHz", freq_khz);
    return len;
}

static ssize_t read_vol(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                        uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_vol, sizeof(current_vol));
}

static ssize_t write_vol(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags) {
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t vol = *static_cast<const uint8_t *>(buf);
    int ret = rda5807m_set_volume(radio, vol);
    if (ret) {
        LOG_ERR("set_volume(%u) failed: %d", vol, ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    current_vol = vol;
    return len;
}

static ssize_t write_seek(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags) {
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t dir = *static_cast<const uint8_t *>(buf);
    if (dir != 0x01 && dir != 0x02) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    int ret = rda5807m_seek(radio, dir == 0x01);
    if (ret == -EIO) {
        LOG_WRN("Seek: no station found");
        return len; /* Not fatal — client can read status to confirm */
    } else if (ret) {
        LOG_ERR("seek failed: %d", ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    /* Sync cached freq from driver after seek lands */
    struct rda5807m_status status;
    if (rda5807m_get_status(radio, &status) == 0) {
        current_freq_khz = status.frequency_khz;
    }

    return len;
}

static ssize_t read_mute(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                         uint16_t offset) {
    uint8_t mute_byte = current_mute ? 1U : 0U;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &mute_byte, sizeof(mute_byte));
}

static ssize_t write_mute(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags) {
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t val = *static_cast<const uint8_t *>(buf);

    bool new_mute;
    if (val == 0xFF) {
        new_mute = !current_mute; /* Toggle */
    } else {
        new_mute = (val != 0);
    }

    int ret = rda5807m_set_mute(radio, new_mute);
    if (ret) {
        LOG_ERR("set_mute(%d) failed: %d", new_mute, ret);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    current_mute = new_mute;
    LOG_INF("Mute: %s", current_mute ? "on" : "off");
    return len;
}

static void status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Status notifications %s", status_notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(fm_radio_svc, BT_GATT_PRIMARY_SERVICE(&uuid_fm_service),

                       BT_GATT_CHARACTERISTIC(&uuid_fm_freq.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_freq, write_freq, nullptr),

                       BT_GATT_CHARACTERISTIC(&uuid_fm_vol.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_vol, write_vol, nullptr),

                       BT_GATT_CHARACTERISTIC(&uuid_fm_seek.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, nullptr,
                                              write_seek, nullptr),

                       BT_GATT_CHARACTERISTIC(&uuid_fm_mute.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_mute, write_mute, nullptr),

                       BT_GATT_CHARACTERISTIC(&uuid_fm_status.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, nullptr,
                                              nullptr, nullptr),
                       BT_GATT_CCC(status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_FM_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int start_advertising() {
    int ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
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

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
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

#define STATUS_NOTIFY_STACK  1024
#define STATUS_NOTIFY_PRIO   K_PRIO_PREEMPT(7)
#define STATUS_NOTIFY_PERIOD K_MSEC(2000)

static void status_notify_thread_fn(void *, void *, void *) {
    while (true) {
        k_sleep(STATUS_NOTIFY_PERIOD);

        if (!active_conn || !status_notify_enabled || !status_attr) {
            continue;
        }

        struct rda5807m_status status;
        if (rda5807m_get_status(radio, &status) != 0) {
            continue;
        }

        struct status_payload payload = {
            .rssi = status.rssi,
            .stereo = status.stereo ? 1U : 0U,
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
    if (!device_is_ready(radio)) {
        LOG_ERR("RDA5807M not ready");
        return -ENODEV;
    }

    LOG_INF("RDA5807M ready");

    int ret = rda5807m_set_frequency(radio, current_freq_khz);
    if (ret) {
        LOG_WRN("Initial tune failed: %d", ret);
    }

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

    ret = start_advertising();
    if (ret) {
        return ret;
    }

    LOG_INF("Advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
    return 0;
}