/*
 *  hardware.h
 *
 *  BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2008 picoChip Designs Ltd.
 *
 *  This file contains the hardware definitions of the PC302 boards.
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

#include <mach/pc302/axi2cfg.h>
#include <mach/pc302/axi2pico.h>
#include <mach/pc302/ebi.h>
#include <mach/pc302/fuse.h>
#include <mach/pc302/gpio.h>
#include <mach/pc302/mem_arm.h>
#include <mach/pc302/mem.h>
#include <mach/pc302/mem_shd.h>
#include <mach/pc302/pa.h>
#include <mach/pc302/pc302.h>
#include <mach/pc302/rtc.h>
#include <mach/pc302/spa.h>
#include <mach/pc302/ssi.h>
#include <mach/pc302/timer.h>
#include <mach/pc302/tzic.h>
#include <mach/pc302/tzpc.h>
#include <mach/pc302/uart.h>
#include <mach/pc302/vic.h>
#include <mach/pc302/wdog.h>

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x) (((x) & 0x00ffffff) | 0xfe000000)

#endif
