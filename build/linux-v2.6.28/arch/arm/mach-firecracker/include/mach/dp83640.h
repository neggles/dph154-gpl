/*
 * arch/arm/mach-firecracker/include/dp83640.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2009 picoChip Designs Ltd.
 *
 * Description:
 *
 * This module provides some definitions for the National Semiconductors
 * DP83640 Ethernet Phy.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __FIRECRACKER_DP83640_H
#define __FIRECRACKER_DP83640_H

/* PHY Register offsets
 * Note: These registers are access over the Phy management bus
 */

/* PHY Registers  */
#define DP83640_BASIC_MODE_CTRL_REG             (0x00)
#define DP83640_BASIC_MODE_STATUS_REG           (0x01)
#define DP83640_PHY_ID_1_REG                    (0x02)
#define DP83640_PHY_ID_2_REG                    (0x03)
#define DP83640_PHY_STATUS_REG                  (0x10)
#define DP83640_MII_INT_CTRL_REG                (0x11)
#define DP83640_MII_INT_STATUS_REG              (0x12)
#define DP83640_MII_PAGE_SELECT_REG             (0x13)

/* PTP 1588 Base Registers - Page 4 */
#define DP83640_PTP_CTRL_REG                    (0x14)
#define DP83640_PTP_TIME_DATA_REG               (0x15)
#define DP83640_PTP_STATUS_REG                  (0x16)
#define DP83640_PTP_TRIGGER_STATUS_REG          (0x17)
#define DP83640_PTP_RATE_LOW_REG                (0x18)
#define DP83640_PTP_RATE_HIGH_REG               (0x19)
#define DP83640_PTP_PAGE4_READ_CHKSUM_REG       (0x1A)
#define DP83640_PTP_PAGE4_WRITE_CHKSUM_REG      (0x1B)
#define DP83640_PTP_TX_TIMESTAMP_REG            (0x1C)
#define DP83640_PTP_RX_TIMESTAMP_REG            (0x1D)
#define DP83640_PTP_EVENT_STATUS_REG            (0x1E)
#define DP83640_PTP_EVENT_DATA_REG              (0x1F)

/* PTP 1588 Configuration Registers - Page 5 */
#define DP83640_PTP_TRIG_CONFIG_REG             (0x14)
#define DP83640_PTP_EVENT_CONFIG_REG            (0x15)
#define DP83640_PTP_TX_CONFIG_0_REG             (0x16)
#define DP83640_PTP_TX_CONFIG_1_REG             (0x17)
#define DP83640_PTP_STATUS_FRAMES_CONFIG_0_REG  (0x18)
#define DP83640_PTP_RX_CONFIG_0_REG             (0x19)
#define DP83640_PTP_RX_CONFIG_1_REG             (0x1A)
#define DP83640_PTP_RX_CONFIG_2_REG             (0x1B)
#define DP83640_PTP_RX_CONFIG_3_REG             (0x1C)
#define DP83640_PTP_RX_CONFIG_4_REG             (0x1D)
#define DP83640_PTP_TEMP_RATE_DUR_LOW_REG       (0x1E)
#define DP83640_PTP_TEMP_RATE_DUR_HIGH_REG      (0x1F)

/* PTP 1588 Configuration Registers - Page 6 */
#define DP83640_PTP_CLK_OP_CTRL_REG             (0x14)
#define DP83640_PTP_STATUS_FRAMES_CONFIG_1_REG  (0x15)
#define DP83640_PTP_STATUS_FRAMES_CONFIG_2_REG  (0x16)
#define DP83640_PTP_STATUS_FRAMES_CONFIG_3_REG  (0x17)
#define DP83640_PTP_STATUS_FRAMES_CONFIG_4_REG  (0x18)
#define DP83640_PTP_SFD_CONFIG_REG              (0x19)
#define DP83640_PTP_INT_CTRL_REG                (0x1A)
#define DP83640_PTP_CLK_SOURCE_REG              (0x1B)
#define DP83640_PTP_ETHERNET_TYPE_REG           (0x1C)
#define DP83640_PTP_OFFSET_REG                  (0x1D)
#define DP83640_PTP_GPIO_MONITOR_REG            (0x1E)
#define DP83640_PTP_RX_HASH_REG_REG             (0x1F)

/* Phy Register bit field definitions */

/* DP83640_MII_PAGE_SELECT_REG */
#define DP83640_PAGE_SEL_0                      (0)
#define DP83640_PAGE_SEL_4                      (4)
#define DP83640_PAGE_SEL_5                      (5)
#define DP83640_PAGE_SEL_6                      (6)
#define DP83640_PAGE_SEL_MASK                   (0x07)

/* DP83640_PTP_CTRL_REG */
#define DP83640_PTP_RESET                       (1 << 0)
#define DP83640_PTP_DISABLE                     (1 << 1)
#define DP83640_PTP_ENABLE                      (1 << 2)
#define DP83640_PTP_STEP_CLK                    (1 << 3)
#define DP83640_PTP_LOAD_CLK                    (1 << 4)
#define DP83640_PTP_READ_CLK                    (1 << 5)

#endif /* __FIRECRACKER_DP83640_H */
