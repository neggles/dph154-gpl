/*
 * platform.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2009 picoChip Designs Ltd.
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

#ifndef __ARCH_PC302_PLATFORM_H
#define __ARCH_PC302_PLATFORM_H

#include <asm/sizes.h>

/* Flash size and position */
/* Physical address of the Flash in the ARM sub-system memory map */
#define PC302_FLASH_BASE        0x40000000

/* Size of the Flash (in bytes */
#define PC302_FLASH_SIZE        SZ_128M

/* Data bus width of the Flash (in bytes */
#define PC302_FLASH_WIDTH       1

/* SDRAM size and position - the ram in the PC302 is split into 4 banks.
 * 64M spread over 4 banks, starting at zero with 64M between each bank.
 * NOTE: The actual ram available is 128M on the SVB but half of it is reserved
 * for the picoArray.
 * NOTE: The size here is the default, set if the pc302_mem= parameter is not
 * used.
 */
#define PC302_RAM_START         0x00000000
#define PC302_RAM_SIZE          SZ_64M      /* Default */
#define PC302_RAM_BANKS         4
#define PC302_RAM_BANK_STRIDE   SZ_64M

/* The clock frequency for the UARTs */
#define PC302_BASE_BAUD         3686400     /* 3.6864 MHz */

/* The clock frequency for the timers on the various boards */
#define PC302_TIMER_FREQ        200000000   /* 200 MHz */

#endif /* __ARCH_PC302_PLATFORM_H */
