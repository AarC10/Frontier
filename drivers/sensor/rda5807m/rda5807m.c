/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT rda_rda5807m

#include "rda5807m.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rda5807m, CONFIG_RDA5807M_LOG_LEVEL);

#define RDA5807M_SHADOW_COUNT       6   /* Regs 0x02–0x07 */
#define RDA5807M_TUNE_TIMEOUT_MS    200
#define RDA5807M_TUNE_POLL_MS       10
#define RDA5807M_SEEK_TIMEOUT_MS    5000
#define RDA5807M_RESET_DELAY_MS     50

#define SHADOW_CONFIG   0
#define SHADOW_CHANNEL  1
#define SHADOW_GPIO     2
#define SHADOW_VOLUME   3
#define SHADOW_OPEN     4
#define SHADOW_BLEND    5

struct rda5807m_config {
    struct i2c_dt_spec i2c;
    uint8_t  vol_default;
    uint8_t  band;      /* 0=87-108, 1=76-91, 2=76-108 */
};

struct rda5807m_data {
    uint16_t shadow[RDA5807M_SHADOW_COUNT]; /* Big-endian shadow of regs 0x02–0x07 */
    uint16_t frequency_khz;
    struct k_mutex lock;
};

static int rda5807m_write_regs(const struct device *dev, uint8_t count) {
    const struct rda5807m_config *cfg = dev->config;
    struct rda5807m_data *data = dev->data;
    uint8_t buff[RDA5807M_SHADOW_COUNT * 2] = {0};

    if (count > RDA5807M_SHADOW_COUNT) {
        return -EINVAL;
    }

    for (int i = 0; i < count; i++) {
        buff[i * 2] = (data->shadow[i] >> 8) & 0xFF;
        buff[i * 2 + 1] =  data->shadow[i]       & 0xFF;
    }

    return i2c_write_dt(&cfg->i2c, buff, count * 2);
}

static int rda5807m_read_status_raw(const struct device *dev,
                                    uint16_t *status_a, uint16_t *status_b) {
    const struct rda5807m_config *cfg = dev->config;
    uint8_t buff[4] = {0};

    int ret = i2c_read_dt(&cfg->i2c, buff, sizeof(buff));
    if (ret) {
        LOG_ERR("Status read failed: %d", ret);
        return ret;
    }

    *status_a = ((uint16_t)buff[0] << 8) | buff[1];
    *status_b = ((uint16_t)buff[2] << 8) | buff[3];
    return 0;
}

static int rda5807m_wait_stc(const struct device *dev, uint32_t timeout_ms,
                              uint16_t *status_a_out, uint16_t *status_b_out) {
    LOG_WRN("STC timeout after %u ms", timeout_ms);
    return -ETIMEDOUT;
}

int rda5807m_set_frequency(const struct device *dev, uint16_t freq_khz) {
    return 0;
}

int rda5807m_set_volume(const struct device *dev, uint8_t vol) {
    return 0;
}

int rda5807m_set_mute(const struct device *dev, bool mute) {
    return 0;
}

int rda5807m_seek(const struct device *dev, bool up) {
    return 0;
}

int rda5807m_get_status(const struct device *dev, struct rda5807m_status *status) {
    return 0;
}

static int rda5807m_init(const struct device *dev) {
    const struct rda5807m_config *cfg = dev->config;
    struct rda5807m_data *data = dev->data;
    int ret = 0;

    k_mutex_init(&data->lock);

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    data->shadow[SHADOW_CONFIG] = RDA5807M_CFG_SOFT_RESET |
                                   RDA5807M_CFG_ENABLE     |
                                   RDA5807M_CFG_NEW_METHOD;
    ret = rda5807m_write_regs(dev, 1);
    if (ret) {
        LOG_ERR("Reset write failed: %d", ret);
        return ret;
    }
    k_msleep(RDA5807M_RESET_DELAY_MS);

    data->shadow[SHADOW_CONFIG] = RDA5807M_CFG_DHIZ       |
                                   RDA5807M_CFG_DMUTE      |
                                   RDA5807M_CFG_NEW_METHOD |
                                   RDA5807M_CFG_ENABLE;

    data->shadow[SHADOW_CHANNEL] = RDA5807M_CHAN_BAND_87108 | RDA5807M_CHAN_SPACE_100K;

    data->shadow[SHADOW_VOLUME] = (RDA5807M_SEEKTH_DEFAULT << RDA5807M_SEEKTH_SHIFT) |
                                   cfg->vol_default;

    ret = rda5807m_write_regs(dev, 4);
    if (ret) {
        LOG_ERR("Init write failed: %d", ret);
        return ret;
    }

    data->frequency_khz = RDA5807M_FREQ_MIN_KHZ;

    LOG_INF("RDA5807M initialized, vol=%u", cfg->vol_default);
    return 0;
}

#define RDA5807M_DEFINE(inst)                                               \
    static struct rda5807m_data rda5807m_data_##inst;                      \
                                                                            \
    static const struct rda5807m_config rda5807m_config_##inst = {         \
        .i2c         = I2C_DT_SPEC_INST_GET(inst),                         \
        .vol_default = DT_INST_PROP(inst, vol_default),                    \
        .band        = 0,                                                   \
    };                                                                      \
                                                                            \
    DEVICE_DT_INST_DEFINE(inst,                                            \
                          rda5807m_init,                                    \
                          NULL,                                             \
                          &rda5807m_data_##inst,                           \
                          &rda5807m_config_##inst,                         \
                          POST_KERNEL,                                      \
                          CONFIG_I2C_INIT_PRIORITY,                        \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(RDA5807M_DEFINE)