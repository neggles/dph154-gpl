/*
 * linux/include/asm-arm/arch-firecracker/platform.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
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

#ifndef __ARCH_FIRECRACKER_PLATFORM_H
#define __ARCH_FIRECRACKER_PLATFORM_H

#include <asm/sizes.h>

/* Flash size and position */
#define FIRECRACKER_FLASH_BASE          0x20000000
#define FIRECRACKER_FLASH_SIZE          SZ_128M

#if defined(CONFIG_IP202FF_XC)
#define FIRECRACKER_FLASH_WIDTH         2
#else
#define FIRECRACKER_FLASH_WIDTH         4
#endif

/* SDRAM size and position - the ram in the firecracker is split into 4 banks.
 * 64M spread over 4 banks, starting at zero with 64M between each bank.
 * NOTE: The actual ram available is 128M on the SVB but half of it is reserved
 * for the picoArray.
 * NOTE: The size here is the default, set if the pc20x_mem= parameter is not
 * used.
 */
#define FIRECRACKER_RAM_START           0x00000000
#define FIRECRACKER_RAM_SIZE            SZ_64M      /* Default */

#if defined(CONFIG_IP202FF_XC)
#define FIRECRACKER_RAM_BANKS           2
#else
#define FIRECRACKER_RAM_BANKS           4
#endif

#define FIRECRACKER_RAM_BANK_STRIDE     SZ_64M

/* The clock frequency for the UARTs */
#define FIRECRACKER_BASE_BAUD           3686400     /* 3.6864MHz */

/* The clock frequency for the timers on the various boards */
#define SVB_TIMER_FREQ                  140000000   /* 140MHz */
#define PC72052_I10_REVB_TIMER_FREQ     140000000   /* 140MHz */
#define PC72052_I10_REVA_TIMER_FREQ     80000000    /* 80MHz */

#endif /* __ARCH_FIRECRACKER_PLATFORM_H */

