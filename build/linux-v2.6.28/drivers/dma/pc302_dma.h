/*******************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 ******************************************************************************/
/*!
 * \file pc302_dma.h
 * \brief PC302 DMA Register Definitions and register manipulation macros
 *
 * Copyright (c) 2008 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All inquiries to support@picochip.com
 */

#ifndef __PC302_DMA_H__
#define __PC302_DMA_H__

/* Channel register offsets from DMA base */
#define PC302_DMA_N_SRC_ADDR_REG_OFFSET(__N)     (0x000 + (0x058 * (__N)))
#define PC302_DMA_N_DST_ADDR_REG_OFFSET(__N)     (0x008 + (0x058 * (__N)))
#define PC302_DMA_N_LLP_REG_OFFSET(__N)          (0x010 + (0x058 * (__N)))
#define PC302_DMA_N_CTRL_REG_OFFSET(__N)         (0x018 + (0x058 * (__N)))
#define PC302_DMA_N_BLOCK_SIZE_REG_OFFSET(__N)   (0x01c + (0x058 * (__N)))
#define PC302_DMA_N_SRC_STATUS_REG_OFFSET(__N)   (0x020 + (0x058 * (__N)))
#define PC302_DMA_N_DST_STATUS_REG_OFFSET(__N)   (0x028 + (0x058 * (__N)))
#define PC302_DMA_N_SRC_STATUS_ADDR_REG_OFF(__N) (0x030 + (0x058 * (__N)))
#define PC302_DMA_N_DST_STATUS_ADDR_REG_OFF(__N) (0x038 + (0x058 * (__N)))
#define PC302_DMA_N_LOW_CONFIG_REG_OFFSET(__N)   (0x040 + (0x058 * (__N)))
#define PC302_DMA_N_HIGH_CONFIG_REG_OFFSET(__N)  (0x044 + (0x058 * (__N)))
#define PC302_DMA_N_SRC_GATHER_REG_OFFSET(__N)   (0x048 + (0x058 * (__N)))
#define PC302_DMA_N_DST_SCATTER_REG_OFFSET(__N)  (0x050 + (0x058 * (__N)))

/* Control register (CTL.x) 32 LSBs */
#define PC302_DMA_INT_EN                (1 << 0)
#define PC302_DMA_DST_TR_WIDTH_SHIFT    (1)
#define PC302_DMA_DST_TR_WIDTH_MASK     (0x7 << PC302_DMA_DST_TR_WIDTH_SHIFT)
#define PC302_DMA_SRC_TR_WIDTH_SHIFT    (4)
#define PC302_DMA_SRC_TR_WIDTH_MASK     (0x7 << PC302_DMA_SRC_TR_WIDTH_SHIFT)
#define PC302_DMA_DINC_SHIFT            (7)
#define PC302_DMA_DINC_MASK             (0x3 << PC302_DMA_DINC_SHIFT)
#define PC302_DMA_SINC_SHIFT            (9)
#define PC302_DMA_SINC_MASK             (0x3 << PC302_DMA_SINC_SHIFT)
#define PC302_DMA_DST_MSIZE_SHIFT       (11)
#define PC302_DMA_DST_MSIZE_MASK        (0x7 << PC302_DMA_DST_MSIZE_SHIFT)
#define PC302_DMA_SRC_MSIZE_SHIFT       (14)
#define PC302_DMA_SRC_MSIZE_MASK        (0x7 << PC302_DMA_SRC_MSIZE_SHIFT)
#define PC302_DMA_SRC_GATHER_EN         (1 << 17)
#define PC302_DMA_DST_SCATTER_EN        (1 << 18)
/* Bit 19 is undefined */
#define PC302_DMA_TT_FC_SHIFT           (20)
#define PC302_DMA_TT_FC_MASK            (0x7 << PC302_DMA_TT_FC_SHIFT)
#define PC302_DMA_DMS_SHIFT             (23)
#define PC302_DMA_DMS_MASK              (0x3 << PC302_DMA_DMS_SHIFT)
#define PC302_DMA_SMS_SHIFT             (25)
#define PC302_DMA_SMS_MASK              (0x3 << PC302_DMA_SMS_SHIFT)
#define PC302_DMA_LLP_DST_EN            (1 << 27)
#define PC302_DMA_LLP_SRC_EN            (1 << 28)
/* Bit 31 is undefined */

/* Block size register (CTL.x) 32 MSBs */
#define PC302_DMA_BLOCK_TS_SHIFT        (0)
/* Max channel block size = 4095. Number of bits = log2(blk_size+1)= 12 */
#define PC302_DMA_BLOCK_TS_MASK         (0xFFF << PC302_DMA_BLOCK_TS_SHIFT)
#define PC302_DMA_DONE                  (1 << 12)

/* Configuration register (CFG.x) LSBs */
/* bits 4:0 are reserved */
#define PC302_DMA_CH_PRIOR_SHIFT        (5)
#define PC302_DMA_CH_PRIOR_MASK         (0xF << PC302_DMA_CH_PRIOR_SHIFT)
#define PC302_DMA_CH_SUSP               (1 << 8)
#define PC302_DMA_FIFO_EMPTY            (1 << 9)
#define PC302_DMA_HS_SEL_DST            (1 << 10)
#define PC302_DMA_HS_SEL_SRC            (1 << 11)
#define PC302_DMA_CHAN_LOCK_LEVEL_SHIFT (1 << 12)
#define PC302_DMA_CHAN_LOCK_LEVEL_MASK  (0x3 << PC302_DMA_CHAN_LOCK_LEVEL_MASK)
#define PC302_DMA_BUS_LOCK_LEVEL_SHIFT  (1 << 14)
#define PC302_DMA_BUS_LOCK_LEVEL_MASK   (0x3 << PC302_DMA_BUS_LOCK_LEVEL_SHIFT)
#define PC302_DMA_DST_HS_POL            (1 << 18)
#define PC302_DMA_SRC_HS_POL            (1 << 19)
#define PC302_DMA_MAX_ABRST_SHIFT       (20)
#define PC302_DMA_MAX_ABRST_MASK        (0x3FF << PC302_DMA_MAX_ABRST_SHIFT)
#define PC302_DMA_RELOAD_SRC            (1 << 30)
#define PC302_DMA_RELOAD_DST            (1 << 31)

/* Configuration register (CFG.x) MSBs */
#define PC302_DMA_FC_MODE               (1 << 0)
#define PC302_DMA_FIFO_MODE             (1 << 1)
#define PC302_DMA_PROTCTL_SHIFT         (2)
#define PC302_DMA_PROTCTL_MASK          (0x7 << PC302_DMA_PROTCTL_SHIFT)
#define PC302_DMA_DST_UPD_EN            (1 << 5)
#define PC302_DMA_SRC_UPD_EN            (1 << 6)
#define PC302_DMA_SRC_PER_SHIFT         (7)
#define PC302_DMA_SRC_PER_MASK          (0xF << PC302_DMA_SRC_PER_SHIFT)
#define PC302_DMA_DST_PER_SHIFT         (11)
#define PC302_DMA_DST_PER_MASK          (0xF << PC302_DMA_DST_PER_SHIFT)
/* bits 15:31 unused */

/* Scatter and gather (SGR.x) registers LSBs */
#define PC302_DMA_SG_INTERVAL_SHIFT     (0) /* 20 bit register */
#define PC302_DMA_SG_INTERVAL_MASK      (0xFFFFF << PC302_DMA_SG_INTERVAL_SHIFT)
#define PC302_DMA_SG_COUNT_SHIFT        (20)/* 12 bit register */
#define PC302_DMA_SG_COUNT_MASK         (0xFFF << PC302_DMA_SG_COUNT_SHIFT)

/* Interrupt registers.
 * There are 5 types of interrupt which have their own register families:
 *      Transfer complete,
 *      Block complete,
 *      Source transfer complete,
 *      Destination transfer complete, and
 *      Error 
 *
 * An update is made by writing/reading a bit in the appropriate register:
 *      Raw Interrupt Status registers for reading the status of each bit
 *      Processed Interrupt Status registers
 *      Interrupt Mask registers
 *      Interrupt Clear registers
 *
 * The following tables gives the address of each register:
 *  -----------------------------------------------------------------------
 * | Operation          |                Interrupt Type                    |
 * |                    | Transfer |  Block  | SRC T/F | DST T/F |  Error  |
 * |                    |----------|---------|---------|---------|---------|
 * |Raw Status (R)      |  0x2c0   |  0x2c8  |  0x2d0  |  0x2d8  |  0x2e0  |
 * |Processed Status (R)|  0x2e8   |  0x2f0  |  0x2f8  |  0x300  |  0x308  |
 * |Mask (R/W)          |  0x310   |  0x318  |  0x320  |  0x328  |  0x330  |
 * |Clear (W)           |  0x338   |  0x340  |  0x348  |  0x350  |  0x358  |
 *  -----------------------------------------------------------------------
 *
 * The following 5 macros can be used to access all 20 registers. O is one
 * of the operation macros defined below.
 */ 
#define PC302_DMA_TXFER_COMPLETE_REG_OFFSET(__O)     (0x2c0 + (__O))
#define PC302_DMA_BLOCK_COMPLETE_REG_OFFSET(__O)     (0x2c8 + (__O))
#define PC302_DMA_SRC_TRX_COMPLETE_REG_OFFSET(__O)   (0x2d0 + (__O))
#define PC302_DMA_DST_TRX_COMPLETE_REG_OFFSET(__O)   (0x2d8 + (__O))
#define PC302_DMA_ERROR_REG_OFFSET(__O)              (0x2e0 + (__O))

/* Interrupt operation macros */
#define PC302_DMA_INTERRUPT_RAW                      (0x028 * 0)
#define PC302_DMA_INTERRUPT_STATUS                   (0x028 * 1)
#define PC302_DMA_INTERRUPT_MASK                     (0x028 * 2)
#define PC302_DMA_INTERRUPT_CLEAR                    (0x028 * 3)

/* Interrupt mask channel macros
 * Bits 0:PC302_DMA_CHANNELS-1  = enable / disable
 * Bits PC302_DMA_CHANNELS:7    = reserved
 * Bits 8:7+PC302_DMA_CHANNELS  = Corresponding write enable bit
 * Bits 8+PC302_DMA_CHANNELS:63 = unused
 */
#define PC302_DMA_IRQ_ENABLE_CHANNEL(__N)            (0x0101 << (__N))
#define PC302_DMA_IRQ_DISABLE_CHANNEL(__N)           (0x0100 << (__N))

/* Interrupt clear macro */
#define PC302_DMA_IRQ_CHANNEL(__N)                   (0x1 << (__N))

/* Interrupt status register */
#define PC302_DMA_IRQ_STATUS_REG_OFFSET              (0x360)

/* Interrupt status bits */
#define PC302_DMA_STATUS_TFR                         (1 << 0)
#define PC302_DMA_STATUS_BLOCK                       (1 << 1)
#define PC302_DMA_STATUS_SRCT                        (1 << 2)
#define PC302_DMA_STATUS_DSTT                        (1 << 3)
#define PC302_DMA_STATUS_ERR                         (1 << 4)
#define PC302_DMA_STATUS_IRQ_TYPES                   (5)
/* Bits 5 to 63 are unused */

/*
 * Software handshaking registers.
 * Three types of S/W handshaking is permitted, controlled by either the
 * source or destination; burst (software), single, or last. This results in
 * 6 registers:
 *     Source Software Transaction Request           (0x368)
 *     Destination Software Transaction Request      (0x370)
 *     Single Source Transaction Request             (0x378)
 *     Single Destination Transacion Request         (0x380) 
 *     Last Source Transaction Request               (0x388)
 *     Last Destination Transaction Request          (0x390)
 *
 * The following macros allows the transfer controller to be addressed.
 * S is the software control type specified by the macros listed below.
 */
#define PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(__S)    (0x368 + (__S))
#define PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(__S)    (0x370 + (__S))

/* Software flow control macros */
#define PC302_DMA_BURST_TRANSFER                     (0x010 * 0)
#define PC302_DMA_SINGLE_TRANSFER                    (0x010 * 1)
#define PC302_DMA_LAST_TRANSFER                      (0x010 * 2)

/*
 * Software flow Transaction request channel bits 
 * Bits 0:PC302_DMA_CHANNELS-1  = enable
 * Bits PC302_DMA_CHANNELS:7    = reserved
 * Bits 8:7+PC302_DMA_CHANNELS  = Corresponding write enable bit
 * Bits 8+PC302_DMA_CHANNELS:63 = unused
 */
#define PC302_DMA_REQ_CHANNEL(__N)                   (0x0101 << (__N))

/* Miscellaneous registers */
#define PC302_DMA_CONFIGURATION_REG_OFFSET           (0x398)
#define PC302_DMA_CHANNEL_ENABLE_REG_OFFSET          (0x3a0)
#define PC302_DMA_ID_REG_OFFSET                      (0x3a8)
#define PC302_DMA_TEST_REG_OFFSET                    (0x3b0)

/* DMA configuration (DmaCfgReg) register bits */
#define PC302_DMA_ENABLE                             (0x01)
/* bits 1:63 are unused */

/* Channel (ChEnReg) enable/disable bits
 * Bits 0:PC302_DMA_CHANNELS-1  = enable/disable
 * Bits PC302_DMA_CHANNELS:7    = reserved
 * Bits 8:7+PC302_DMA_CHANNELS  = Corresponding write enable bit
 * Bits 8+PC302_DMA_CHANNELS:63 = unused
 */
#define PC302_DMA_ENABLE_CHANNEL(__N)                (0x0101 << (__N))
#define PC302_DMA_DISABLE_CHANNEL(__N)               (0x0100 << (__N))
#define PC302_DMA_CHANNEL(__N)                       (0x01 << (__N))

/* Channels per controller */
#define PC302_DMA_CHANNELS                           (8)

/* Hardware handshaking interfaces per controller */
#define PC302_DMA_HANDSHAKING_IFS                    (8)

/* Four DMA masters per controller */
#define PC302_DMA_MASTERS                            (4)

/* Two DMAs in the system */
#define PC302_NUM_OF_DMAS                            (2)

#endif /* __PC302_DMA_H__ */
