/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright(C) 2006 picoChip(R) Designs Ltd.
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
 *              Timer Register Definitions
 *
 *****************************************************************************/ 
#ifndef __PC20X_TIMER_H
#define __PC20X_TIMER_H

/* The number of timers in the hardware, numbered 0 to N-1 */
#define TimerNumberOfTimers                         4

/* Register definitions for the timers */
#define TimerNLoadCountRegOffset(__N)               (0x0000 + (0x14 * (__N)))
#define TimerNCurrentValueRegOffset(__N)            (0x0004 + (0x14 * (__N)))
#define TimerNControlRegOffset(__N)                 (0x0008 + (0x14 * (__N)))
#define TimerNEOIRegOffset(__N)                     (0x000c + (0x14 * (__N)))
#define TimerNInterruptStatusRegOffset(__N)         (0x0010 + (0x14 * (__N)))

/* Timer N control register bit definitions */
#define TimerEnable                                 (0x00000001)
#define TimerMode                                   (0x00000002)
#define TimerInterruptMask                          (0x00000004)


/* Register definitions for global timer registers */
#define TimersInterruptStatusRegOffset              (0x00a0)
#define TimersEOIRegOffset                          (0x00a4)
#define TimersRawInterruptStatusRegOffset           (0x00a8)

/* Global Timer Registers bit definitions */
#define Timer(__N)                                  (0x00000001 << (__N))


#endif /* __PC20X_TIMER_H */



