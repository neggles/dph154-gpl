/*
 *  BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 *  PC302 architecture timex specifications
 *
 * Copyright (c) 2008 picoChip Designs Ltd.
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
/* This is the rate at which the timer that controls the linux tick runs.
 * It is used to calculate the actual tick frequency that is achieved as
 * it is not always possible to divide this frequency down to the configured
 * HZ value.
 * See jiffies.h.
 */

#ifndef __TIMEX_H__
#define __TIMEX_H__

#include <mach/platform.h>

/* The frequency at which the timer gets clocked is dependent on the
 * machine type we are running on.
 * Unfortunately the machine we are running on is not known until run time
 * when the bootloader tells us. Therefore we do not have a known clock
 * tick rate. 
 * *** Fake the tick rate! ***
 * By setting the CLOCK_TICK_RATE to HZ, the kernel is built assuming that
 * the clock is incremented once every 'tick'. We will have to configure the
 * hardware (at run time) to make this true.
 */
#define CLOCK_TICK_RATE		(HZ)

#endif /* __TIMEX_H__ */

