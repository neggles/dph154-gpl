/*
 *  linux/include/asm-arm/arch-firecracker/hardware.h
 *
 *  BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 *  This file contains the hardware definitions of the Firecracker boards.
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <mach/platform.h>

/* Include all the firecracker IO definitions copied from the bootloader here */
#include <mach/pc20x/pc20x.h>
#include <mach/pc20x/gpio.h>
#include <mach/pc20x/memif.h>
#include <mach/pc20x/picoarray.h>
#include <mach/pc20x/rap.h>
#include <mach/pc20x/rtc.h>
#include <mach/pc20x/timer.h>
#include <mach/pc20x/uart.h>
#include <mach/pc20x/wdt.h>
#include <mach/pc20x/vic.h>
#include <mach/pc20x/dma.h>
#include <mach/pc20x/ethernet.h>

/* macro to get at IO space when running virtually */

/* The Virtual IO mapping for firecracker simply maps 0xffxx,xxxx to 
 * 0xfexx,xxxx.
 */
#define IO_ADDRESS(x) ((x) & 0xfeffffff)

#endif
