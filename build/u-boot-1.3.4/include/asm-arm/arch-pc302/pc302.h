/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file pc302.h
* \brief Definitions for the PC302 ARM sub-system.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_H
#define PC302_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/*****************************************************************************/
/* Internal Boot ROM                                                         */
/*****************************************************************************/
#define PC302_BOOT_ROM_BASE         (0xFFFF0000)

/*****************************************************************************/
/* AXI2PICO Buffers                                                          */
/*****************************************************************************/
#define PC302_AXI2PICO_BUFFERS_BASE (0xC0000000)

/*****************************************************************************/
/* Peripheral Bus                                                            */
/*****************************************************************************/
#define PC302_MEMIF_BASE            (0x80000000)
#define PC302_EBI_BASE              (0x80010000)
#define PC302_EMAC_BASE             (0x80030000)
#define PC302_DMAC1_BASE            (0x80040000)
#define PC302_DMAC2_BASE            (0x80050000)
#define PC302_VIC0_BASE             (0x80060000)
#define PC302_VIC1_BASE             (0x80064000)
#define PC302_TZIC_BASE             (0x80068000)
#define PC302_TZPC_BASE             (0x80070000)
#define PC302_FUSE_BASE             (0x80080000)
#define PC302_SSI_BASE              (0x80090000)
#define PC302_AXI2CFG_BASE          (0x800A0000)
#define PC302_IPSEC_BASE            (0x80100000)
#define PC302_SRTP_BASE             (0x80140000)
#define PC302_CIPHER_BASE           (0x80180000)
#define PC302_RTCLK_BASE            (0x80200000)
#define PC302_TIMER_BASE            (0x80210000)
#define PC302_GPIO_BASE             (0x80220000)
#define PC302_UART1_BASE            (0x80230000)
#define PC302_UART2_BASE            (0x80240000)
#define PC302_WDOG_BASE             (0x80250000)

/*****************************************************************************/
/* External Memory                                                           */
/*****************************************************************************/
#define PC302_DDRBANK_BASE          (0x00000000)

#define PC302_EBI_CS0_BASE          (0x40000000)
#define PC302_EBI_CS1_BASE          (0x48000000)
#define PC302_EBI_CS2_BASE          (0x50000000)
#define PC302_EBI_CS3_BASE          (0x58000000)

#define PC302_FLASH_BASE            (PC302_EBI_CS0_BASE)
#define PC302_FLASH_START           (PC302_EBI_CS0_BASE)
#define PC302_FLASH_SIZE            (0x08000000)

/*****************************************************************************/
/* Internal SRAM Memory                                                      */
/*****************************************************************************/
#define PC302_SRAM_BASE             (0x20000000)
#define PC302_SRAM_START            (0x20000000)
#define PC302_SRAM_SIZE             (0x00020000)

/*****************************************************************************/
/* Silicon Revision                                                          */
/*****************************************************************************/
#define PC302_REV_A                 (0)
#define PC302_REV_D                 (1)

/*****************************************************************************/
/* Device Ids                                                                */
/*****************************************************************************/
#define PC302_DEVICE_ID             (3)
#define PC312_DEVICE_ID             (7)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_H */
