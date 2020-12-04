/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file
* \brief pc302_emac.h
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_EMAC_H
#define PC302_EMAC_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/*!
 * Register Offset Addresses
 */
#define EMAC_NETWORK_CTRL_REG_OFFSET                                (0x0000)
#define EMAC_NETWORK_CFG_REG_OFFSET                                 (0x0004)
#define EMAC_NETWORK_STATUS_REG_OFFSET                              (0x0008)
#define EMAC_USER_IO_REG_OFFSET                                     (0x000C)
#define EMAC_DMA_CFG_REG_OFFSET                                     (0x0010)
#define EMAC_TX_STATUS_REG_OFFSET                                   (0x0014)
#define EMAC_RX_BUFF_Q_BASE_ADDR_REG_OFFSET                         (0x0018)
#define EMAC_TX_BUFF_Q_BASE_ADDR_REG_OFFSET                         (0x001C)
#define EMAC_RX_STATUS_REG_OFFSET                                   (0x0020)
#define EMAC_INT_STATUS_REG_OFFSET                                  (0x0024)
#define EMAC_INT_ENABLE_REG_OFFSET                                  (0x0028)
#define EMAC_INT_DISABLE_REG_OFFSET                                 (0x002C)
#define EMAC_INT_MASK_REG_OFFSET                                    (0x0030)
#define EMAC_PHY_MAINTAIN_REG_OFFSET                                (0x0034)
#define EMAC_RX_PAUSE_QUANTUM_REG_OFFSET                            (0x0038)
#define EMAC_TX_PAUSE_QUATNUM_REG_OFFSET                            (0x003C)
#define EMAC_HASH_BOT_31_0_REG_OFFSET                               (0x0080)
#define EMAC_HASH_TOP_63_32_REG_OFFSET                              (0x0084)
#define EMAC_SPEC_ADDR_1_BOT_31_0_REG_OFFSET                        (0x0088)
#define EMAC_SPEC_ADDR_1_TOP_47_32_REG_OFFSET                       (0x008C)
#define EMAC_SPEC_ADDR_2_BOT_31_0_REG_OFFSET                        (0x0090)
#define EMAC_SPEC_ADDR_2_TOP_47_32_REG_OFFSET                       (0x0094)
#define EMAC_SPEC_ADDR_3_BOT_31_0_REG_OFFSET                        (0x0098)
#define EMAC_SPEC_ADDR_3_TOP_47_32_REG_OFFSET                       (0x009C)
#define EMAC_SPEC_ADDR_4_BOT_31_0_REG_OFFSET                        (0x00A0)
#define EMAC_SPEC_ADDR_4_TOP_47_32_REG_OFFSET                       (0x00A4)
#define EMAC_TYPE_ID_MATCH_1_REG_OFFSET                             (0x00A8)
#define EMAC_TYPE_ID_MATCH_2_REG_OFFSET                             (0x00AC)
#define EMAC_TYPE_ID_MATCH_3_REG_OFFSET                             (0x00B0)
#define EMAC_TYPE_ID_MATCH_4_REG_OFFSET                             (0x00B4)
#define EMAC_WOL_REG_OFFSET                                         (0x00B8)
#define EMAC_IPG_STRETCH_REG_OFFSET                                 (0x00BC)
#define EMAC_STACKED_VLAN_REG_OFFSET                                (0x00C0)

#define EMAC_MODULE_ID_REG_OFFSET                                   (0x00FC)

#define EMAC_OCTETS_TX_31_0_REG_OFFSET                              (0x0100)
#define EMAC_OCTETS_TX_47_32_REG_OFFSET                             (0x0104)
#define EMAC_FRAMES_TX_NO_ERROR_REG_OFFSET                          (0x0108)
#define EMAC_BROADCAST_FRAMES_TX_NO_ERROR_REG_OFFSET                (0x010C)
#define EMAC_MULTICAST_FRAMES_TX_NO_ERROR_REG_OFFSET                (0x0110)
#define EMAC_TX_PAUSE_FRAMES_REG_OFFSET                             (0x0114)
#define EMAC_64_BYTE_FRAMES_TX_NO_ERROR_REG_OFFSET                  (0x0118)
#define EMAC_65_127_BYTE_FRAMES_TX_NO_ERROR_REG_OFFSET              (0x011C)
#define EMAC_128_255_BYTE_FRAMES_TX_NO_ERROR_REG_OFFSET             (0x0120)
#define EMAC_256_511_BYTE_FRAMES_TX_NO_ERROR_REG_OFFSET             (0x0124)
#define EMAC_512_1023_BYTE_FRAMES_TX_NO_ERROR_REG_OFFSET            (0x0128)
#define EMAC_1024_1518_BYTE_FRAMES_TX_NO_ERROR_REG_OFFSET           (0x012C)
#define EMAC_GREATER_THAN_1518_BYTE_FRAMES_TX_NO_ERROR_REG_OFFSET   (0x0130)
#define EMAC_TX_UNDER_RUN_ERROR_REG_OFFSET                          (0x0134)
#define EMAC_SINGLE_COLLISION_FRAMES_REG_OFFSET                     (0x0138)
#define EMAC_MULTIPLE_COLLISION_FRAMES_REG_OFFSET                   (0x013C)
#define EMAC_EXCESSIVE_COLLISIONS_REG_OFFSET                        (0x0140)
#define EMAC_LATE_COLLISIONS_REG_OFFSET                             (0x0144)
#define EMAC_DEFFERED_TX_FRAMES_REG_OFFSET                          (0x0148)
#define EMAC_CARRIER_SENSE_ERRORS_REG_OFFSET                        (0x014C)
#define EMAC_OCTETS_RX_31_0_NO_ERROR_REG_OFFSET                     (0x0150)
#define EMAC_OCTETS_RX_47_32_NO_ERROR_REG_OFFSET                    (0x0154)
#define EMAC_FRAMES_RX_NO_ERROR_REG_OFFSET                          (0x0158)
#define EMAC_BROADCAST_FRAMES_RX_NO_ERROR_REG_OFFSET                (0x015C)
#define EMAC_MULTICAST_FRAMES_RX_NO_ERROR_REG_OFFSET                (0x0160)
#define EMAC_PAUSE_FRAME_RX_REG_OFFSET                              (0x0164)
#define EMAC_64_BYTE_FRAMES_RX_NO_ERROR_REG_OFFSET                  (0x0168)
#define EMAC_65_127_BYTE_FRAMES_RX_NO_ERROR_REG_OFFSET              (0x016C)
#define EMAC_128_255_BYTE_FRAMES_RX_NO_ERROR_REG_OFFSET             (0x0170)
#define EMAC_256_511_BYTE_FRAMES_RX_NO_ERROR_REG_OFFSET             (0x0174)
#define EMAC_512_1023_BYTE_FRAMES_RX_NO_ERROR_REG_OFFSET            (0x0178)
#define EMAC_1024_1518_BYTE_FRAMES_RX_NO_ERROR_REG_OFFSET           (0x017C)
#define EMAC_1519_TO_MAX_FRAMES_RX_NO_ERROR_REG_OFFSET              (0x0180)
#define EMAC_UNDERSIZE_FRAMES_RX_REG_OFFSET                         (0x0184)
#define EMAC_OVERSIZE_FRAMES_RX_REG_OFFSET                          (0x0188)
#define EMAC_JABBER_FRAMES_RX_REG_OFFSET                            (0x018C)
#define EMAC_FCS_ERRORS_REG_OFFSET                                  (0x0190)
#define EMAC_LENGTH_FIELD_FRAME_ERRORS_REG_OFFSET                   (0x0194)
#define EMAC_RX_SYMBOL_ERRORS_REG_OFFSET                            (0x0198)
#define EMAC_ALLIGNMENT_ERRORS_REG_OFFSET                           (0x019C)
#define EMAC_RX_RESOURCE_ERRORS_REG_OFFSET                          (0x01A0)
#define EMAC_RX_OVERRUN_ERRORS_REG_OFFSET                           (0x01A4)
#define EMAC_IP_HDR_CHECKSUM_ERRORS_REG_OFFSET                      (0x01A8)
#define EMAC_TCP_CHECKSUM_ERRORS_REG_OFFSET                         (0x01AC)
#define EMAC_UDP_CHECKSUM_ERRORS_REG_OFFSET                         (0x01B0)

#define EMAC_1588_TIMER_SECONDS_REG_OFFSET                          (0x01D0)
#define EMAC_1588_TIMER_NANO_SECONDS_REG_OFFSET                     (0x01D4)
#define EMAC_1588_TIMER_ADJUST_REG_OFFSET                           (0x01D8)
#define EMAC_1588_TIMER_INCREMENT_REG_OFFSET                        (0x01DC)

#define EMAC_PTP_EVENT_FRAME_TX_SECONDS_REG_OFFSET                  (0x01E0)
#define EMAC_PTP_EVENT_FRAME_TX_NANO_SECONDS_REG_OFFSET             (0x01E4)
#define EMAC_PTP_EVENT_FRAME_RX_SECONDS_REG_OFFSET                  (0x01E8)
#define EMAC_PTP_EVENT_FRAME_RX_NANO_SECONDS_REG_OFFSET             (0x01EC)
#define EMAC_PTP_PEER_EVENT_FRAME_TX_SECONDS_REG_OFFSET             (0x01F0)
#define EMAC_PTP_PEER_EVENT_FRAME_TX_NANO_SECONDS_REG_OFFSET        (0x01F4)
#define EMAC_PTP_PEER_EVENT_FRAME_RX_SECONDS_REG_OFFSET             (0x01F8)
#define EMAC_PTP_PEER_EVENT_FRAME_RX_NANO_SECONDS_REG_OFFSET        (0x01FC)

#define EMAC_PCS_CTRL_REG_OFFSET                                    (0x0200)
#define EMAC_PCS_STATUS_REG_OFFSET                                  (0x0204)
#define EMAC_PCS_UPPER_PHY_ID_REG_OFFSET                            (0x0208)
#define EMAC_PCS_LOWER_PHY_ID_REG_OFFSET                            (0x020C)
#define EMAC_PCS_AUTO_NEG_ADVERT_REG_OFFSET                         (0x0210)
#define EMAC_PCS_AUTO_NEG_LINK_PARTNER_AB_REG_OFFSET                (0x0214)
#define EMAC_PCS_AUTO_NEG_EXPANSION_REG_OFFSET                      (0x0218)
#define EMAC_PCS_AUTO_NEG_NEXT_PAGE_REG_OFFSET                      (0x021C)
#define EMAC_PCS_AUTO_NEG_LINK_PARTNER_NEXT_PAGE_REG_OFFSET         (0x0220)
#define EMAC_PCS_EXTENDED_STATUS_REG_OFFSET                         (0x023C)

/*!
 * EMAC_NETWORK_CTRL_REG_OFFSET bits
 */
#define EMAC_STORE_TX_TIMESTAMP_IN_CRC                              (1 << 15)
#define EMAC_TRANSMIT_HALT                                          (1 << 10)
#define EMAC_START_TX                                               (1 << 9)
#define EMAC_CLEAR_STATS_REGISTERS                                  (1 << 5)
#define EMAC_MDIO_ENABLE                                            (1 << 4)
#define EMAC_TX_ENABLE                                              (1 << 3)
#define EMAC_RX_ENABLE                                              (1 << 2)

/*!
 * EMAC_NETWORK_CFG_REG_OFFSET bits
 */
#define EMAC_RX_CHKSUM_OFFLOAD_ENABLE                               (1 << 24)
#define EMAC_64_BIT_AMBA_DATA_BUS_WITDH                             (1 << 21)
#define EMAC_MDC_CLOCK_DIV_MASK                                     (0x7 <<18)
#define EMAC_MDC_CLOCK_DIV_96                                       (0x5 <<18)
#define EMAC_LENGTH_FIELD_ERROR_FRAME_DISCARD                       (1 << 16)
#define EMAC_PAUSE_ENABLE                                           (1 << 13)
#define EMAC_RX_1536_BYTE_FRAMES                                    (1 << 8)
#define EMAC_MULTICAST_HASH_ENABLE                                  (1 << 6)
#define EMAC_NO_BROADCAST_FRAMES                                    (1 << 5)
#define EMAC_COPY_ALL_FRAMES                                        (1 << 4)
#define EMAC_FULL_DUPLEX                                            (1 << 1)
#define EMAC_SPEED_100_MBPS                                         (1 << 0)

/*!
 * EMAC_NETWORK_STATUS_REG_OFFSET bits
 */
#define EMAC_PHY_MANAGEMENT_IDLE                                    (1 << 2)

/*!
 * EMAC_DMA_CFG_REG_OFFSET bits
 */
#define EMAC_DMA_RX_BUFFER_SIZE_IDX                                 (16)
#define EMAC_DMA_RX_BUFFER_SIZE_MASK                                (0xFF << EMAC_DMA_RX_BUFFER_SIZE_IDX)
#define EMAC_DMA_RX_BUFFER_SIZE                                     (0x18 << EMAC_DMA_RX_BUFFER_SIZE_IDX)

/*!
 * EMAC_PHY_MAINTAIN_REG_OFFSET bits
 */
#define EMAC_PHY_ID_MASK                                            (0x1F)
#define EMAC_PHY_ID_SHIFT                                           (23)
#define EMAC_PHY_REG_MASK                                           (0x1F)
#define EMAC_PHY_REG_SHIFT                                          (18)
#define EMAC_PHY_DATA_MASK                                          (0xFFFF)
#define EMAC_PHY_READ                                               (0x60020000)
#define EMAC_PHY_WRITE                                              (0x50020000)

/*!
 * EMAC_TX_STATUS_REG_OFFSET bits
 */
#define EMAC_TRANSMIT_HRESP_NOT_OK                                  (1 << 8)
#define EMAC_TRANSMIT_UNDERRUN                                      (1 << 6)
#define EMAC_TRANSMIT_COMPLETE                                      (1 << 5)
#define EMAC_TRANSMIT_CORRUPTION_AHB_ERROR                          (1 << 4)
#define EMAC_TRANSMIT_GO                                            (1 << 3)
#define EMAC_TRANSMIT_RETRY_LIMIT_EXCEEDED                          (1 << 2)
#define EMAC_COLLISION_OCCURRED                                     (1 << 1)
#define EMAC_USED_BIT_READ                                          (1 << 0)

/*!
 * EMAC_RX_STATUS_REG_OFFSET bits
 */
#define EMAC_RECEIVE_HRESP_NOT_OK                                   (1 << 3)
#define EMAC_RECEIVE_OVERRUN                                        (1 << 2)
#define EMAC_FRAME_RECEIVED                                         (1 << 1)
#define EMAC_RX_BUFFER_NOT_AVAIL                                    (1 << 0)

/*!
 * EMAC_INT_STATUS_REG_OFFSET bits
 * EMAC_INT_ENABLE_REG_OFFSET bits
 * EMAC_INT_DISABLE_REG_OFFSET bits
 * EMAC_INT_MASK_REG_OFFSET bits
 */
/* Bits 31-27 reserved */
#define EMAC_ENABLE_TSU_SECONDS_REG_INCREMENT                       (1 << 26)
#define EMAC_ENABLE_PTP_PDELAY_RESP_FRAME_TX                        (1 << 25)
#define EMAC_ENABLE_PTP_PDELAY_REQ_FRAME_TX                         (1 << 24)
#define EMAC_ENABLE_PTP_PDELAY_RESP_FRAME_RX                        (1 << 23)
#define EMAC_ENABLE_PTP_PDELAY_REQ_FRAME_RX                         (1 << 22)
#define EMAC_ENABLE_PTP_SYNC_FRAME_TX                               (1 << 21)
#define EMAC_ENABLE_PTP_DELAY_REQ_FRAME_TX                          (1 << 20)
#define EMAC_ENABLE_PTP_SYNC_FRAME_RX                               (1 << 19)
#define EMAC_ENABLE_PTP_DELAY_REQ_FRAME_RX                          (1 << 18)
#define EMAC_ENABLE_PCS_LINK_PARTNER_PAGE_RX                        (1 << 17)
#define EMAC_ENABLE_PCS_AUTONEG_COMPLETE                            (1 << 16)
#define EMAC_ENABLE_EXTERNAL_INTERRUPT                              (1 << 15)
#define EMAC_ENABLE_PAUSE_FRAME_TX                                  (1 << 14)
#define EMAC_ENABLE_PAUSE_FRAME_TIME_ZERO                           (1 << 13)
#define EMAC_ENABLE_PAUSE_FRAME_NON_ZERO_PAUSE                      (1 << 12)
#define EMAC_ENABLE_HRESP_NOT_OK                                    (1 << 11)
#define EMAC_ENABLE_RX_OVERRUN                                      (1 << 10)
#define EMAC_ENABLE_LINK_CHANGE                                     (1 << 9)
/* Bit 8 not used */
#define EMAC_ENABLE_TX_COMPLETE                                     (1 << 7)
#define EMAC_ENABLE_TRANSMIT_CORRUPTION_AHB_ERROR                   (1 << 6)
#define EMAC_ENABLE_RETRY_LIMIT_EXCEEDED                            (1 << 5)
#define EMAC_ENABLE_TX_BUFF_UNDERRUN                                (1 << 4)
#define EMAC_ENABLE_TX_USED_BIT_READ                                (1 << 3)
#define EMAC_ENABLE_RX_USED_BIT_READ                                (1 << 2)
#define EMAC_ENABLE_RX_COMPLETE                                     (1 << 1)
#define EMAC_ENABLE_MANAGEMENT_DONE                                 (1 << 0)

/*!
 * Rx Descriptor Bits
 */
#define EMAC_RX_DESC_WRAP                                           (1 << 1)
#define EMAC_RX_DESC_HOST_OWN                                       (1 << 0)

#define EMAC_RX_DESC_CSUM_OFFLOAD_IDX                               (22)
#define EMAC_RX_DESC_CSUM_OFFLOAD_MASK                              (0x3 << EMAC_RX_DESC_CSUM_OFFLOAD_IDX)
#define EMAC_RX_DESC_VLAN_TAG                                       (1 << 21)
#define EMAC_RX_DESC_END_OF_FRAME                                   (1 << 15)
#define EMAC_RX_DESC_START_OF_FRAME                                 (1 << 14)
#define EMAC_RX_DESC_LENGTH_MASK                                    (0x1FFF)

/*!
 * Tx Descriptor Bits
 */
#define EMAC_TX_DESC_HOST_OWN                                       (1 << 31)
#define EMAC_TX_DESC_WRAP                                           (1 << 30)
#define EMAC_TX_RETRY_ERROR                                         (1 << 29)
#define EMAC_TX_UNDERRUN_ERROR                                      (1 << 28)
#define EMAC_TX_FRAME_CORRUPTION_ERROR                              (1 << 27)
#define EMAC_TX_LATE_COLLISION_ERROR                                (1 << 26)
#define EMAC_TX_NO_CRC_APPEND                                       (1 << 16)
#define EMAC_TX_LAST_BUFFER                                         (1 << 15)
#define EMAC_TX_BUFFER_LENGTH_MASK                                  (0x3FFF)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_EMAC_H */
