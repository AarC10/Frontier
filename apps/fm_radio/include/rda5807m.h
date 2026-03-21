/*
 * Copyright (c) 2026 Aaron Chan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RADIO_RDA5807M_H__
#define __RADIO_RDA5807M_H__

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register Addresses */

/* Sequential write mode (I2C addr 0x10) starts at 0x02 */
#define RDA5807M_REG_CONFIG     0x02
#define RDA5807M_REG_CHANNEL    0x03
#define RDA5807M_REG_GPIO       0x04
#define RDA5807M_REG_VOLUME     0x05
#define RDA5807M_REG_OPEN       0x06
#define RDA5807M_REG_BLEND      0x07

/* Sequential read mode (I2C addr 0x10) starts at 0x0A */
#define RDA5807M_REG_STATUS_A   0x0A
#define RDA5807M_REG_STATUS_B   0x0B
#define RDA5807M_REG_STATUS_C   0x0C
#define RDA5807M_REG_STATUS_D   0x0D

/* REG 0x02 — Config */

#define RDA5807M_CFG_ENABLE         BIT(0)
#define RDA5807M_CFG_SOFT_RESET     BIT(1)
#define RDA5807M_CFG_NEW_METHOD     BIT(2)
#define RDA5807M_CFG_RDS_EN         BIT(3)
#define RDA5807M_CFG_CLK_MODE_SHIFT 4
#define RDA5807M_CFG_CLK_MODE_MASK  (0x7 << 4)
#define RDA5807M_CFG_SKMODE         BIT(7)
#define RDA5807M_CFG_SEEK           BIT(8)
#define RDA5807M_CFG_SEEKUP         BIT(9)
#define RDA5807M_CFG_RCLK_DIRECT    BIT(10)
#define RDA5807M_CFG_RCLK_NON_CAL   BIT(11)
#define RDA5807M_CFG_BASS           BIT(12)
#define RDA5807M_CFG_MONO           BIT(13)
#define RDA5807M_CFG_DMUTE          BIT(14)
#define RDA5807M_CFG_DHIZ           BIT(15)

/* REG 0x03 — Channel */
#define RDA5807M_CHAN_SPACE_100K    (0x0 << 0)
#define RDA5807M_CHAN_SPACE_200K    (0x1 << 0)
#define RDA5807M_CHAN_SPACE_50K     (0x2 << 0)
#define RDA5807M_CHAN_SPACE_25K     (0x3 << 0)
#define RDA5807M_CHAN_BAND_87108    (0x0 << 2)
#define RDA5807M_CHAN_BAND_7691     (0x1 << 2)
#define RDA5807M_CHAN_BAND_76108    (0x2 << 2)
#define RDA5807M_CHAN_TUNE          BIT(4)
#define RDA5807M_CHAN_DIRECT        BIT(5)  /* Direct mode */
#define RDA5807M_CHAN_SHIFT         6       /* Channel bits [15:6] */
#define RDA5807M_CHAN_MASK          (0x3FF << RDA5807M_CHAN_SHIFT)

/* REG 0x05 — Volume */

#define RDA5807M_VOL_MASK           0x000F
#define RDA5807M_VOL_MAX            15
#define RDA5807M_SEEKTH_SHIFT       8
#define RDA5807M_SEEKTH_MASK        (0x0F << RDA5807M_SEEKTH_SHIFT)
#define RDA5807M_SEEKTH_DEFAULT     2

/* REG 0x0A — Read Status A */

#define RDA5807M_ST_RDSR            BIT(15) /* RDS ready */
#define RDA5807M_ST_STC             BIT(14) /* Seek/tune complete */
#define RDA5807M_ST_SF              BIT(13) /* Seek fail */
#define RDA5807M_ST_RDSS            BIT(12) /* RDS sync */
#define RDA5807M_ST_BLKE            BIT(11) /* RDS block E */
#define RDA5807M_ST_STEREO          BIT(10) /* Stereo indicator */
#define RDA5807M_ST_READCHAN_SHIFT  0
#define RDA5807M_ST_READCHAN_MASK   0x03FF  /* Currently tuned channel */

/* REG 0x0B — Read Status B */
#define RDA5807M_ST_RSSI_SHIFT      9
#define RDA5807M_ST_RSSI_MASK       (0x7F << RDA5807M_ST_RSSI_SHIFT)
#define RDA5807M_ST_FM_TRUE         BIT(8)  /* 1=station, 0=noise */
#define RDA5807M_ST_FM_READY        BIT(7)

/* Frequency limits (kHz) */

#define RDA5807M_FREQ_MIN_KHZ       87000U
#define RDA5807M_FREQ_MAX_KHZ       108000U
#define RDA5807M_FREQ_STEP_KHZ      100U
#define RDA5807M_FREQ_BASE_KHZ      87000U  /* Band base for channel calc */


struct rda5807m_status {
    uint16_t frequency_khz;
    uint8_t  rssi;          /* 0–127 */
    bool     stereo;
    bool     station;       /* FM_TRUE bit — tuned to actual station */
    bool     seek_fail;
};

/**
 * @brief Tune to a specific frequency.
 *
 * @param dev     RDA5807M device
 * @param freq_khz Frequency in kHz (87000–108000, 100kHz steps)
 * @return 0 on success, -EINVAL for out-of-range, -ETIMEDOUT if STC never asserts
 */
int rda5807m_set_frequency(const struct device *dev, uint32_t freq_khz);

/**
 * @brief Set output volume.
 *
 * @param dev  RDA5807M device
 * @param vol  Volume level 0–15 (0 = minimum, 15 = maximum)
 * @return 0 on success, -EINVAL if out of range
 */
int rda5807m_set_volume(const struct device *dev, uint8_t vol);

/**
 * @brief Mute or unmute audio output.
 *
 * @param dev   RDA5807M device
 * @param mute  true to mute, false to unmute
 * @return 0 on success
 */
int rda5807m_set_mute(const struct device *dev, bool mute);

/**
 * @brief Seek to the next valid station.
 *
 * Blocking — returns when seek completes or fails.
 *
 * @param dev   RDA5807M device
 * @param up    true to seek upward in frequency, false downward
 * @return 0 on success, -EIO on seek failure (no station found)
 */
int rda5807m_seek(const struct device *dev, bool up);

/**
 * @brief Read current tuner status.
 *
 * @param dev     RDA5807M device
 * @param status  Pointer to status struct to populate
 * @return 0 on success
 */
int rda5807m_get_status(const struct device *dev, struct rda5807m_status *status);

#ifdef __cplusplus
}
#endif

#endif /* __RADIO_RDA5807M_H__ */