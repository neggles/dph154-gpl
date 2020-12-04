
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
 *              Timer Register Definitions
 *
 *****************************************************************************/ 
#ifndef __FIRECRACKER_PC20X_TIMER_H
#define __FIRECRACKER_PC20X_TIMER_H

/* The number of timers in the hardware, numbered 0 to N-1 */
#define TIMER_NUMBER_OF_TIMERS                       4

/* Register definitions for the timers */
#define TIMER_N_LOAD_COUNT_REG_OFFSET(__N)          (0x0000 + (0x14 * (__N)))
#define TIMER_N_CURRENT_VALUE_REG_OFFSET(__N)       (0x0004 + (0x14 * (__N)))
#define TIMER_N_CONTROL_REG_OFFSET(__N)             (0x0008 + (0x14 * (__N)))
#define TIMER_N_EOI_REG_OFFSET(__N)                 (0x000c + (0x14 * (__N)))
#define TIMER_N_INTERRUPT_STATUS_REG_OFFSET(__N)    (0x0010 + (0x14 * (__N)))

/* Timer N control register bit definitions */
#define TIMER_ENABLE                                (0x00000001)
#define TIMER_MODE                                  (0x00000002)
#define TIMER_INTERRUPT_MASK                        (0x00000004)


/* Register definitions for global timer registers */
#define TIMERS_INTERRUPT_STATUS_REG_OFFSET          (0x00a0)
#define TIMERS_EOI_REG_OFFSET                       (0x00a4)
#define TIMERS_RAW_INTERRUPT_STATUS_REG_OFFSET      (0x00a8)

/* Global Timer Registers bit definitions */
#define TIMER(__N)                                  (0x00000001 << (__N))


#endif /* __FIRECRACKER_PC20X_TIMER_H */



