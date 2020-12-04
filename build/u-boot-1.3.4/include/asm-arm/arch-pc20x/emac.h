/*
 *  include/asm/arch/svb20x/ethernet.h
 *
 *  BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * Description:
 *
 * This module provides the network interface to the firecracker EMAC 
 * hardware.
 *
 * References:
 *
 * Synopsys DesignWare Ethernet Universal Databook Version 3.2 
 *  November 2005.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * v0.01 09nov06    - Initial version
 */

#ifndef __FIRECRACKER_PC20X_ETHERNET_H
#define __FIRECRACKER_PC20X_ETHERNET_H

/* Register offsets: */
#define EMAC_MAC_CONFIG_REG_OFFSET          (0x0000)
#define EMAC_MAC_FRAME_FLT_REG_OFFSET       (0x0004)
#define EMAC_MAC_HASH_HIGH_REG_OFFSET       (0x0008)
#define EMAC_MAC_HASH_LOW_REG_OFFSET        (0x000c)
#define EMAC_MAC_GMII_ADDR_REG_OFFSET       (0x0010)
#define EMAC_MAC_GMII_DATA_REG_OFFSET       (0x0014)
#define EMAC_MAC_FLOW_CONTROL_REG_OFFSET    (0x0018)
#define EMAC_MAC_VLAN_TAG_REG_OFFSET        (0x001c)
#define EMAC_MAC_VERSION_REG_OFFSET         (0x0020)
#define EMAC_MAC_WAKE_FILT_REG_OFFSET       (0x0028)
#define EMAC_MAC_PMT_REG_OFFSET             (0x002c)
#define EMAC_MAC_ADDR_N_HIGH_REG_OFFSET(__N)(0x0040 + (0x0008 * __N))
#define EMAC_MAC_ADDR_N_LOW_REG_OFFSET(__N) (0x0044 + (0x0008 * __N))

#define EMAC_DMA_BUS_MODE_REG_OFFSET        (0x1000)
#define EMAC_DMA_TX_DEMAND_REG_OFFSET       (0x1004)
#define EMAC_DMA_RX_DEMAND_REG_OFFSET       (0x1008)
#define EMAC_DMA_RX_LIST_REG_OFFSET         (0x100c)
#define EMAC_DMA_TX_LIST_REG_OFFSET         (0x1010)
#define EMAC_DMA_STATUS_REG_OFFSET          (0x1014)
#define EMAC_DMA_MODE_REG_OFFSET            (0x1018)
#define EMAC_DMA_IE_REG_OFFSET              (0x101c)
#define EMAC_DMA_STATS_REG_OFFSET           (0x1020)
#define EMAC_DMA_CURR_TX_DESC_REG_OFFSET    (0x1048)
#define EMAC_DMA_CURR_RX_DESC_REG_OFFSET    (0x104c)
#define EMAC_DMA_CURR_TX_BUF_REG_OFFSET     (0x1050)
#define EMAC_DMA_CURR_RX_BUF_REG_OFFSET     (0x1054)


/* Register bit definitions: */

/* EMAC_MAC_CONFIG_REG_OFFSET bits: */
#define EMAC_WATCHDOG_DISABLE               (1 << 23)
#define EMAC_JABBER_DISABLE                 (1 << 22)
#define EMAC_FRAME_BURST_ENABLE             (1 << 21)
#define EMAC_JUMBO_FRAME_ENABLE             (1 << 20)
#define EMAC_INTER_FRAME_GAP_SHIFT          (17)
#define EMAC_INTER_FRAME_GAP_MASK           (0x7 << EMAC_INTER_FRAME_GAP_SHIFT)
#define EMAC_PORT_SELECT                    (1 << 15)
#define EMAC_SPEED_100                      (1 << 14)
#define EMAC_DIABLE_RECEIVE_OWN             (1 << 13)
#define EMAC_LOOP_BACK_MODE                 (1 << 12)
#define EMAC_DUPLEX_MODE                    (1 << 11)
#define EMAC_CHECKSUM_OFFLOAD               (1 << 10)
#define EMAC_DISABLE_RETRY                  (1 << 9)
#define EMAC_LINK_UP                        (1 << 8)
#define EMAC_CRC_STRIPPING                  (1 << 7)
#define EMAC_BACK_OFF_SHIFT                 (5)
#define EMAC_BACK_OFF_MASK                  (0x3 << EMAC_BACK_OFF_SHIFT)
#define EMAC_DEFERRAL_CHECK                 (1 << 4)
#define EMAC_TX_ENABLE                      (1 << 3)
#define EMAC_RX_ENABLE                      (1 << 2)

/* EMAC_MAC_FRAME_FLT_REG_OFFSET bits: */
#define EMAC_RECEIVE_ALL                    (1 << 31)
#define EMAC_SOURCE_FILTER_ENABLE           (1 << 9)
#define EMAC_SOURCE_INVERSE_FILTER          (1 << 8)
#define EMAC_PASS_CONTROL_FRAMES_SHIFT      (6)
#define EMAC_PASS_CONTROL_FRAMES_MASK       (0x3 << EMAC_PASS_CONTROL_FRAMES_SHIFT)
#define EMAC_DISABLE_BROADCAST              (1 << 5)
#define EMAC_PASS_ALL_MULTICAST             (1 << 4)
#define EMAC_INVERSE_FILTER                 (1 << 3)
#define EMAC_HASH_MULTICAST                 (1 << 2)
#define EMAC_HASH_UNICAST                   (1 << 1)
#define EMAC_PROMISCUOUS_MODE               (1 << 0)


/* EMAC_MAC_GMII_ADDR_REG_OFFSET bits: */
#define EMAC_GMII_ADDRESS_SHIFT             (11)
#define EMAC_GMII_ADDRESS_MASK              (0x1f << EMAC_GMII_ADDRESS_SHIFT)
#define EMAC_GMII_GMII_REG_SHIFT            (6)
#define EMAC_GMII_GMII_REG_MASK             (0x1f << EMAC_GMII_GMII_REG_SHIFT)
#define EMAC_GMII_CSR_CLOCK_SHIFT           (2)
#define EMAC_GMII_CSR_CLOCK_MASK            (0x7 << EMAC_GMII_CSR_CLOCK_SHIFT)
#define EMAC_GMII_CSR_RANGE_60_100          (0x0 << EMAC_GMII_CSR_CLOCK_SHIFT)
#define EMAC_GMII_CSR_RANGE_100_150         (0x1 << EMAC_GMII_CSR_CLOCK_SHIFT)
#define EMAC_GMII_CSR_RANGE_20_35           (0x2 << EMAC_GMII_CSR_CLOCK_SHIFT)
#define EMAC_GMII_CSR_RANGE_35_60           (0x3 << EMAC_GMII_CSR_CLOCK_SHIFT)
#define EMAC_GMII_CSR_RANGE_150_250         (0x4 << EMAC_GMII_CSR_CLOCK_SHIFT)
#define EMAC_GMII_CSR_RANGE_250_300         (0x5 << EMAC_GMII_CSR_CLOCK_SHIFT)
#define EMAC_GMII_WRITE                     (1 << 1)
#define EMAC_GMII_BUSY                      (1 << 0)


/* EMAC_MAC_FLOW_CONTROL_REG_OFFSET bits: */
#define EMAC_PAUSE_TIME_SHIFT               (16)
#define EMAC_PAUSE_TIME_MASK                (0xffff << EMAC_PAUSE_TIME_SHIFT)
#define EMAC_PAUSE_LOW_THRESHOLD_SHIFT      (4)
#define EMAC_PAUSE_LOW_THRESHOLD_MASK       (0x3 << EMAC_PAUSE_LOW_THRESHOLD_SHIFT)
#define EMAC_UNI_PAUSE_DETECT               (1 << 3)
#define EMAC_RX_FLOW_ENABLE                 (1 << 2)
#define EMAC_TX_FLOW_ENABLE                 (1 << 1)
#define EMAC_FLOW_BUSY                      (1 << 0)


/* EMAC_MAC_ADDR_N_HIGH_REG_OFFSET bits: */
#define EMAC_MAC_ADDR_ENABLE                (1 << 31)
#define EMAC_MAC_ADDR_SOURCE                (1 << 30)
#define EMAC_MAC_ADDR_MASK_CNTL_SHIFT       (24)
#define EMAC_MAC_ADDR_MASK_CNTL_MASK        (0x3f << EMAC_MAC_ADDR_MASK_CNTL_SHIFT)


/* EMAC_DMA_BUS_MODE_REG_OFFSET bits: */
#define EMAC_FIXED_BURST                    (1 << 16)
#define EMAC_PRIORITY_RATIO_SHIFT           (14)
#define EMAC_PRIORITY_RATIO_MASK            (0x3 << EMAC_PRIORITY_RATIO_SHIFT)
#define EMAC_BURST_LENGTH_SHIFT             (8)
#define EMAC_BURST_LENGTH_MASK              (0x3f << EMAC_BURST_LENGTH_SHIFT)
#define EMAC_BURST_LENGTH_1                 (0x1 << EMAC_BURST_LENGTH_SHIFT)
#define EMAC_BURST_LENGTH_2                 (0x2 << EMAC_BURST_LENGTH_SHIFT)
#define EMAC_BURST_LENGTH_4                 (0x4 << EMAC_BURST_LENGTH_SHIFT)
#define EMAC_BURST_LENGTH_8                 (0x8 << EMAC_BURST_LENGTH_SHIFT)
#define EMAC_BURST_LENGTH_16                (0x10 << EMAC_BURST_LENGTH_SHIFT)
#define EMAC_BURST_LENGTH_32                (0x20 << EMAC_BURST_LENGTH_SHIFT)
#define EMAC_DMA_SKIP_SHIFT                 (2)
#define EMAC_DMA_SKIP_MASK                  (0x0000007c)
#define EMAC_DMA_ARBITRATION                (1 << 1)
#define EMAC_SOFT_RESET                     (1 << 0)

/* EMAC_DMA_STATUS_REG_OFFSET bits: */
#define EMAC_GMAC_PMT_INT                   (1 << 28)
#define EMAC_GMAC_MMC_INT                   (1 << 27)
#define EMAC_GMAC_LINE_IF_INT               (1 << 26)
#define EMAC_BUS_ERROR_SHIFT                (23)
#define EMAC_BUS_ERROR_MASK                 (0x7 << EMAC_BUS_ERROR_SHIFT)
#define EMAC_DMA_TX_STATE_SHIFT             (20)
#define EMAC_DMA_TX_STATE_MASK              (0x7 << EMAC_DMA_TX_STATE_SHIFT)
#define EMAC_DMA_TX_STOPPED                 (0x0 << EMAC_DMA_TX_STATE_SHIFT)
#define EMAC_DMA_TX_FETCHING                (0x1 << EMAC_DMA_TX_STATE_SHIFT)
#define EMAC_DMA_TX_WAITING                 (0x2 << EMAC_DMA_TX_STATE_SHIFT)
#define EMAC_DMA_TX_READING                 (0x3 << EMAC_DMA_TX_STATE_SHIFT)
#define EMAC_DMA_TX_SUSPENDED               (0x6 << EMAC_DMA_TX_STATE_SHIFT)
#define EMAC_DMA_TX_RUNNING                 (0x7 << EMAC_DMA_TX_STATE_SHIFT)
#define EMAC_DMA_RX_STATE_SHIFT             (17)
#define EMAC_DMA_RX_STATE_MASK              (0x7 << EMAC_DMA_RX_STATE_SHIFT)
#define EMAC_DMA_RX_STOPPED                 (0x0 << EMAC_DMA_RX_STATE_SHIFT)
#define EMAC_DMA_RX_FETCHING                (0x1 << EMAC_DMA_RX_STATE_SHIFT)
#define EMAC_DMA_RX_WAITING                 (0x3 << EMAC_DMA_RX_STATE_SHIFT)
#define EMAC_DMA_RX_SUSPENDED               (0x4 << EMAC_DMA_RX_STATE_SHIFT)
#define EMAC_DMA_RX_CLOSING                 (0x5 << EMAC_DMA_RX_STATE_SHIFT)
#define EMAC_DMA_RX_TRANSFERRING            (0x7 << EMAC_DMA_RX_STATE_SHIFT)
#define EMAC_NORMAL_SUMM_INT                (1 << 16)
#define EMAC_ABNORMAL_SUMM_INT              (1 << 15)
#define EMAC_EARLY_RX_INT                   (1 << 14)
#define EMAC_FATAL_BUS_ERROR_INT            (1 << 13)
#define EMAC_EARLY_TX_INT                   (1 << 10)
#define EMAC_RX_TIMEOUT_INT                 (1 << 9)
#define EMAC_RX_STOPPED_INT                 (1 << 8)
#define EMAC_RX_UNAVAILABLE_INT             (1 << 7)
#define EMAC_RX_INT                         (1 << 6)
#define EMAC_TX_UNDERFLOW_INT               (1 << 5)
#define EMAC_RX_OVERFLOW_INT                (1 << 4)
#define EMAC_TX_JABBER_TIMEOUT_INT          (1 << 3)
#define EMAC_TX_UNAVAILABLE_INT             (1 << 2)
#define EMAC_TX_STOPPED_INT                 (1 << 1)
#define EMAC_TX_INT                         (1 << 0)


/* EMAC_DMA_MODE_REG_OFFSET bits: */
#define EMAC_STORE_FORWARD                  (1 << 21)
#define EMAC_FLUSH_TX_FIFO                  (1 << 20)
#define EMAC_TX_THRESHOLD_SHIFT             (14)
#define EMAC_TX_THRESHOLD_MASK              (0x7 << EMAC_TX_THRESHOLD_SHIFT)
#define EMAC_START_TX                       (1 << 13)
#define EMAC_FC_DEACTIVE_SHIFT              (11)
#define EMAC_FC_DEACTIVE_MASK               (0x3 << EMAC_FC_DEACTIVE_SHIFT)
#define EMAC_FC_ACTIVE_SHIFT                (9)
#define EMAC_FC_ACTIVE_MASK                 (0x3 << EMAC_FC_ACTIVE_SHIFT)
#define EMAC_ENABLE_FLOW_CONTROL            (1 << 8)
#define EMAC_FORWARD_ERROR_FRAMES           (1 << 7)
#define EMAC_FORWARD_UNDERSIZE_FRAMES       (1 << 6)
#define EMAC_RX_THRESHOLD_CONTROL_SHIFT     (1 << 3)
#define EMAC_RX_THRESHOLD_CONTROL_MASK      (0x3 << EMAC_RX_THRESHOLD_CONTROL_SHIFT)
#define EMAC_OPERATE_ON_SECOND_FRAME        (1 << 2)
#define EMAC_START_RX                       (1 << 1)


/* EMAC_DMA_IE_REG_OFFSET bits: */
#define EMAC_NORMAL_SUMM_IE                 (1 << 16)
#define EMAC_ABNORMAL_SUMM_IE               (1 << 15)
#define EMAC_EARLY_RX_IE                    (1 << 14)
#define EMAC_FATAL_BUS_ERROR_IE             (1 << 13)
#define EMAC_EARLY_TX_IE                    (1 << 10)
#define EMAC_RX_TIMEOUT_IE                  (1 << 9)
#define EMAC_RX_STOPPED_IE                  (1 << 8)
#define EMAC_RX_UNAVAILABLE_IE              (1 << 7)
#define EMAC_RX_IE                          (1 << 6)
#define EMAC_TX_UNDERFLOW_IE                (1 << 5)
#define EMAC_RX_OVERFLOW_IE                 (1 << 4)
#define EMAC_TX_JABBER_IE                   (1 << 3)
#define EMAC_TX_UNAVAILABLE_IE              (1 << 2)
#define EMAC_TX_STOPPED_IE                  (1 << 1)
#define EMAC_TX_IE                          (1 << 0)


/* EMAC_DMA_STATS_REG_OFFSET bits: */
#define EMAC_OVERFLOW_FIFO_OVERFLOW         (1 << 28)
#define EMAC_APP_MISSED_SHIFT               (17)
#define EMAC_APP_MISSED_MASK                (0x7ff << EMAC_MISSED_FRAMES_SHIFT)
#define EMAC_OVERFLOW_MISSED_FRAMES         (1 << 16)
#define EMAC_CNTR_MISSED_SHIFT              (0)
#define EMAC_CNTR_MISSED_MASK               (0xffff << EMAC_CNTR_MISSED_SHIFT)



/* Descriptor fields: */

/* Transmit/Receive Status: */
#define EMAC_DESC_STATUS_OWNER              (0x80000000)
#define EMAC_DESC_STATUS_ERR_SUM            (0x00008000)

/* Transmit Status word: */
#define EMAC_TX_DESC_STATUS_JABBER_TO       (0x00004000)
#define EMAC_TX_DESC_STATUS_FLUSHED         (0x00002000)
#define EMAC_TX_DESC_STATUS_LOSS_CARRIER    (0x00000800)
#define EMAC_TX_DESC_STATUS_NC              (0x00000400)
#define EMAC_TX_DESC_STATUS_LC              (0x00000200)
#define EMAC_TX_DESC_STATUS_EC              (0x00000100)
#define EMAC_TX_DESC_STATUS_VLAN            (0x00000080)
#define EMAC_TX_DESC_STATUS_CC_SHIFT        (3)
#define EMAC_TX_DESC_STATUS_CC_MASK         (0x00000078)
#define EMAC_TX_DESC_STATUS_ED              (0x00000004)
#define EMAC_TX_DESC_STATUS_UF              (0x00000002)
#define EMAC_TX_DESC_STATUS_DB              (0x00000001)

/* Receive Status word: */
#define EMAC_RX_DESC_STATUS_FAIL_DST_ADDR   (0x40000000)
#define EMAC_RX_DESC_STATUS_FLEN_SHIFT      (16)
#define EMAC_RX_DESC_STATUS_FLEN_MASK       (0x3fff0000)
#define EMAC_RX_DESC_STATUS_DESC_ERR        (0x00004000)
#define EMAC_RX_DESC_STATUS_FAIL_SRC_ADDR   (0x00002000)
#define EMAC_RX_DESC_STATUS_LEN_ERR         (0x00001000)
#define EMAC_RX_DESC_STATUS_OV_ERR          (0x00000800)
#define EMAC_RX_DESC_STATUS_VLAN            (0x00000400)
#define EMAC_RX_DESC_STATUS_FIRST_DESC      (0x00000200)
#define EMAC_RX_DESC_STATUS_LAST_DESC       (0x00000100)
#define EMAC_RX_DESC_STATUS_IPC_ERR         (0x00000080)
#define EMAC_RX_DESC_STATUS_LATE_COL        (0x00000040)
#define EMAC_RX_DESC_STATUS_FRAME_TYPE      (0x00000020)
#define EMAC_RX_DESC_STATUS_WDT_TIMEOUT     (0x00000010)
#define EMAC_RX_DESC_STATUS_RX_ERR          (0x00000008)
#define EMAC_RX_DESC_STATUS_DRIBBLE_ERR     (0x00000004)
#define EMAC_RX_DESC_STATUS_CRC_ERR         (0x00000002)
#define EMAC_RX_DESC_STATUS_DST_MATCH_MAC0  (0x00000001)

/* Transmit/Receive Control word: */
#define EMAC_RX_DESC_CNTL_DISABLE_INT_COM   (0x80000000)    /* Rx only */
#define EMAC_TX_DESC_CNTL_ENABLE_INT_COM    (0x80000000)    /* Tx only */
#define EMAC_TX_DESC_CNTL_LAST_SEG          (0x40000000)    /* Tx only */
#define EMAC_TX_DESC_CNTL_FIRST_SEG         (0x20000000)    /* Tx only */
#define EMAC_TX_DESC_CNTL_DISABLE_CRC       (0x04000000)    /* Tx only */
#define EMAC_DESC_CNTL_END_RING             (0x02000000)
#define EMAC_DESC_CNTL_CHAINED              (0x01000000)
#define EMAC_TX_DESC_CNTL_DISABLE_PAD       (0x00800000)    /* Tx only */
#define EMAC_DESC_CNTL_TBS2_SHIFT           (11)
#define EMAC_DESC_CNTL_TBS2_MASK            (0x003ff800)
#define EMAC_DESC_CNTL_TBS1_SHIFT           (0)
#define EMAC_DESC_CNTL_TBS1_MASK            (0x000007ff)

/* EMAC_MAX_MAC_ADDRS defines the number of perfect match addresses
 * available.
 */
#define EMAC_MAX_MAC_ADDRS                      (16)


#endif /* __FIRECRACKER_PC20X_ETHERNET_INCLUDED */

