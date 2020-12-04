/* Copyright (c) 2008 picoChip Designs Ltd.
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
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
 */

#ifndef __PC302_H__
#define __PC302_H__

#ifndef __ASSEMBLY__

/**
 * Read the system configuration register.
 *
 * \return Returns the value of the system configuration register.
 */
u32 syscfg_read(void);

/**
 * Update the system configuration register.
 *
 * \param mask Mask of the bits to update.
 * \param val The value to write.
 */
void syscfg_update(u32 mask, u32 val);

#endif /* __ASSEMBLY__ */

/*****************************************************************************/
/* Internal Boot ROM                                                         */
/*****************************************************************************/
#define BOOT_ROM_BASE           0xFFFF0000
#define BOOT_ROM_SIZE           0x400
/*****************************************************************************/
/* AXI2PICO Buffers                                                          */
/*****************************************************************************/
#define AXI2PICO_BUFFERS_BASE   0xC0000000
#define AXI2PICO_BUFFERS_SIZE   0x00010000

/*****************************************************************************/
/* Peripheral Bus                                                            */
/*****************************************************************************/
#define PC302_PERIPH_BASE             0x80000000
#define PC302_PERIPH_LENGTH           0x00400000
#define PC302_MEMIF_BASE              0x80000000
#define PC302_EBI_BASE                0x80010000
#define PC302_EMAC_BASE               0x80030000
#define PC302_DMAC1_BASE              0x80040000
#define PC302_DMAC2_BASE              0x80050000
#define PC302_VIC0_BASE               0x80060000
#define PC302_VIC1_BASE               0x80064000
#define PC302_TZIC_BASE               0x80068000
#define PC302_TZPC_BASE               0x80070000
#define PC302_FUSE_BASE               0x80080000
#define PC302_SSI_BASE                0x80090000
#define PC302_AXI2CFG_BASE            0x800A0000
#define PC302_IPSEC_BASE              0x80100000
#define PC302_SRTP_BASE               0x80140000
#define PC302_CIPHER_BASE             0x80180000
#define PC302_RTCLK_BASE              0x80200000
#define PC302_TIMER_BASE              0x80210000
#define PC302_GPIO_BASE               0x80220000
#define PC302_UART1_BASE              0x80230000
#define PC302_UART2_BASE              0x80240000
#define PC302_WDOG_BASE               0x80250000

/*****************************************************************************/
/* External Memory                                                           */
/*****************************************************************************/
#define DDRBANK0_BASE           0x00000000
#define DDRBANK1_BASE           0x04000000
#define DDRBANK2_BASE           0x08000000
#define DDRBANK3_BASE           0x0C000000

#define EBI_CS0_BASE            0x40000000
#define EBI_CS1_BASE            0x48000000
#define EBI_CS2_BASE            0x50000000
#define EBI_CS3_BASE            0x58000000

#define FLASH_BASE              EBI_CS0_BASE
#define FLASH_START             EBI_CS0_BASE
#define FLASH_SIZE              0x08000000

/*****************************************************************************/
/* Internal SRAM Memory                                                      */
/*****************************************************************************/
#define SRAM_BASE               0x20000000
#define SRAM_START              0x20000000
#define SRAM_SIZE               0x00020000

#endif /* __PC302_H__ */
