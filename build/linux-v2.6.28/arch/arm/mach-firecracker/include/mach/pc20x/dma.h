/*
 * Copyright(C) 2006 picoChip(R) Designs Ltd.
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*****************************************************************************
 *
 * Description: picoChip PC20x ARM Subsystem
 *
 *              DMA Register Definitions
 *
 *****************************************************************************/
#ifndef __FIRECRACKER_PC20X_DMA_H
#define __FIRECRACKER_PC20X_DMA_H


/* Channel register offsets from DMA base */
#define DMA_N_SRC_ADDRESS_REG_OFFSET(__N)           (0x000 + (0x058 * (__N)))
#define DMA_N_DST_ADDRESS_REG_OFFSET(__N)           (0x008 + (0x058 * (__N)))
#define DMA_N_LINKED_LIST_POINTER_REG_OFFSET(__N)   (0x010 + (0x058 * (__N)))
#define DMA_N_CONTROL_REG_OFFSET(__N)               (0x018 + (0x058 * (__N)))
#define DMA_N_BLOCK_SIZE_REG_OFFSET(__N)            (0x01c + (0x058 * (__N)))
#define DMA_N_LOW_CONFIGURATION_REG_OFFSET(__N)     (0x040 + (0x058 * (__N)))
#define DMA_N_HIGH_CONFIGURATION_REG_OFFSET(__N)    (0x044 + (0x058 * (__N)))
#define DMA_N_SRC_GATHER_REG_OFFSET(__N)            (0x048 + (0x058 * (__N)))
#define DMA_N_DST_SCATTER_REG_OFFSET(__N)           (0x050 + (0x058 * (__N)))

/* Control register bits */
#define DMA_INT_EN                                  (1 << 0)
#define DMA_DST_TR_WIDTH_SHIFT                      (1)
#define DMA_DST_TR_WIDTH_MASK                       (0x7 << DMA_DST_TR_WIDTH_SHIFT)
#define DMA_SRC_TR_WIDTH_SHIFT                      (4)
#define DMA_SRC_TR_WIDTH_MASK                       (0x7 << DMA_SRC_TR_WIDTH_SHIFT)
#define DMA_DINC_SHIFT                              (7)
#define DMA_DINC_MASK                               (0x3 << DMA_DINC_SHIFT)
#define DMA_SINC_SHIFT                              (9)
#define DMA_SINC_MASK                               (0x3 << DMA_SINC_SHIFT)
#define DMA_DST_MSIZE_SHIFT                         (11)
#define DMA_DST_MSIZE_MASK                          (0x7 << DMA_DST_MSIZE_SHIFT)
#define DMA_SRC_MSIZE_SHIFT                         (14)
#define DMA_SRC_MSIZE_MASK                          (0x7 << DMA_SRC_MSIZE_SHIFT)
#define DMA_SRC_GATHER_EN                           (1 << 17)
#define DMA_DST_SCATTER_EN                          (1 << 18)
#define DMA_TT_FC_SHIFT                             (20)
#define DMA_TT_FC_MASK                              (0x7 << DMA_TT_FC_SHIFT)
#define DMA_DMS_SHIFT                               (23)
#define DMA_DMS_MASK                                (0x3 << DMA_DMS_SHIFT)
#define DMA_SMS_SHIFT                               (25)
#define DMA_SMS_MASK                                (0x3 << DMA_SMS_SHIFT)
#define DMA_LLP_DST_EN                              (1 << 27)
#define DMA_LLP_SRC_EN                              (1 << 28)

/* Block size (control upper dword) bits */
#define DMA_BLOCK_TS_SHIFT                          (0)
#define DMA_BLOCK_TS_MASK                           (0xfff << DMA_BLOCK_TS_SHIFT)
#define DMA_DONE                                    (1 << 12)

/* Low Configuration register bits */
#define DMA_CH_PRIOR_SHIFT                          (5)
#define DMA_CH_PRIOR_MASK                           (0x7 << DMA_CH_PRIOR_SHIFT)
#define DMA_CH_SUSP                                 (1 << 8)
#define DMA_FIFO_EMPTY                              (1 << 9)
#define DMA_HS_SEL_DST                              (1 << 10)
#define DMA_HS_SEL_SRC                              (1 << 11)
#define DMA_DST_HS_POL                              (1 << 18)
#define DMA_SRC_HS_POL                              (1 << 19)
#define DMA_MAX_ABRST_SHIFT                         (20)
#define DMA_MAX_ABRST_MASK                          (0x3ff << DMA_MAX_ABRST_SHIFT)
#define DMA_RELOAD_SRC                              (1 << 30)
#define DMA_RELOAD_DST                              (1 << 31)

/* High Configuration register bits */
#define DMA_FCMODE                                  (1 << 0)
#define DMA_FIFO_MODE                               (1 << 1)
#define DMA_PROTCTL_SHIFT                           (2)
#define DMA_PROTCTL_MASK                            (0x7 << DMA_PROTCTL_SHIFT)
#define DMA_SRC_PER_SHIFT                           (7)
#define DMA_SRC_PER_MASK                            (0xf << DMA_SRC_PER_SHIFT)
#define DMA_DST_PER_SHIFT                           (11)
#define DMA_DST_PER_MASK                            (0xf << DMA_DST_PER_SHIFT)

/* Scatter and gather registers bits */
#define DMA_SG_INTERVAL_SHIFT                       (0)
#define DMA_SG_INTERVAL_MASK                        (0xfffff << DMA_SG_INTERVAL_SHIFT)
#define DMA_SG_COUNT_SHIFT                          (20)
#define DMA_SG_COUNT_MASK                           (0xfff << DMA_SG_COUNT_SHIFT)



/* Interrupt register types - used in the following interrupt register macros
 * eg Clear the block complete interrupt for channel 3
 * write(DMA_IRQ_CHANNEL(3), DMA_BLOCK_COMPLETE(CLEAR))
 */
#define RAW                                         (0x028 * 0)
#define STATUS                                      (0x028 * 1)
#define MASK                                        (0x028 * 2)
#define CLEAR                                       (0x028 * 3)

/* Interrupt registers */
#define DMA_TRANSFER_COMPLETE_REG_OFFSET(__T)       (0x2c0 + (__T))
#define DMA_BLOCK_COMPLETE_REG_OFFSET(__T)          (0x2c8 + (__T))
#define DMA_SRC_TRX_COMPLETE_REG_OFFSET(__T)        (0x2d0 + (__T))
#define DMA_DST_TRX_COMPLETE_REG_OFFSET(__T)        (0x2d8 + (__T))
#define DMA_ERROR_REG_OFFSET(__T)                   (0x2e0 + (__T))
#define DMA_IRQ_STATUS_REG_OFFSET                   (0x360)

/* Interrupt channel bits */
#define DMA_IRQ_CHANNEL(__N)                        (1 << (__N))

/* Interrupt mask channel bits */
#define DMA_IRQ_ENABLE_CHANNEL(__N)                 (0x0101 << (__N))
#define DMA_IRQ_DISABLE_CHANNEL(__N)                (0x0100 << (__N))

/* Interrupt status bits */
#define DMA_STATUS_TFR                              (1 << 0)
#define DMA_STATUS_BLOCK                            (1 << 1)
#define DMA_STATUS_SRCT                             (1 << 2)
#define DMA_STATUS_DSTT                             (1 << 3)
#define DMA_STATUS_ERR                              (1 << 4)
#define DMA_STATUS_IRQ_TYPES                        (5)


/* Software handshaking register types - used with the following handshaking
 * register macros.
 * For example, request a single transaction on the destination of channel 2
 * write(DMA_REQ_CHANNEL(2), DMA_DST_TRANSACTION_REQUEST(SINGLE))
 */
#define BURST                                       (0x010 * 0)
#define SINGLE                                      (0x010 * 1)
#define LAST                                        (0x010 * 2)

/* Software handshaking registers */
#define DMA_SRC_TRX_REQUEST_REG_OFFSET(__S)         (0x368 + (__S))
#define DMA_DST_TRX_REQUEST_REG_OFFSET(__S)         (0x370 + (__S))

/* Transaction request channel bits */
#define DMA_REQ_CHANNEL(__N)                        (0x0101 << (__N))


/* Miscellaneous registers */
#define DMA_CONFIGURATION_REG_OFFSET                (0x398)
#define DMA_CHANNEL_ENABLE_REG_OFFSET               (0x3a0)
#define DMA_ID_REG_OFFSET                           (0x3a8)
#define DMA_TEST_REG_OFFSET                         (0x3b0)

/* DMA configuration register bits */
#define DMA_ENABLE                                  (0x01)

/* Channel enable/disable bits */
#define DMA_ENABLE_CHANNEL(__N)                     (0x0101 << (__N))
#define DMA_DISABLE_CHANNEL(__N)                    (0x0100 << (__N))

/* Four channels per controller */
#define DMA_CHANNELS        4
#define DMA_CHANNEL(__N)                            (0x01 << (__N))

/* Four handshaking interfaces per controller */
#define DMA_HANDSHAKING_IFS 4

#endif /* __FIRECRACKER_PC20X_DMA_H */



