/*
 * Copyright (c) 2026 Aaron Chan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RADIO_RDA5807M_H__
#define __RADIO_RDA5807M_H__

#define RDA5807M_REG_CONFIG     0x02
#define RDA5807M_REG_CHANNEL    0x03
#define RDA5807M_REG_GPIO       0x04
#define RDA5807M_REG_VOLUME     0x05

#define RDA5807M_REG_STATUS_A   0x0A
#define RDA5807M_REG_STATUS_B   0x0B

/* REG 0x02 */
#define RDA_CFG_ENABLE      BIT(0)
#define RDA_CFG_SOFT_RESET  BIT(1)
#define RDA_CFG_NEW_METHOD  BIT(4)  /* Should be set for reliable operation */
#define RDA_CFG_RDS_EN      BIT(3)
#define RDA_CFG_DMUTE       BIT(14) /* Set to 1 to be unmuted */
#define RDA_CFG_DHIZ        BIT(15) /* Set to 1 for audio output active */

/* REG 0x03 */
#define RDA_CHAN_SPACE_100K (0x0 << 0)  /* 100kHz steps — use this for US/EU */
#define RDA_CHAN_BAND_87108 (0x0 << 2)
#define RDA_CHAN_TUNE        BIT(4)
/* channel[9:0] at bits [15:6] */
/* channel = (freq_khz / 100) - 870 */

/* REG 0x05 */
#define RDA_VOL_MASK        0x000F
#define RDA_SEEKTH_MASK     0x0F00  /* Seek SNR threshold */

/* REG 0x0A (read status) */
#define RDA_ST_RDSR         BIT(15) /* RDS ready */
#define RDA_ST_STC          BIT(14) /* Seek/tune complete */
#define RDA_ST_SF           BIT(13) /* Seek fail */
#define RDA_ST_STEREO       BIT(10)
#define RDA_ST_RSSI_SHIFT   9
#define RDA_ST_RSSI_MASK    (0x7F << 9)


#endif /* __RADIO_RDA5807M_H__ */
