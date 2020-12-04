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
 *              Real Time Clock Register Definitions
 *
 *****************************************************************************/
#ifndef __FIRECRACKER_PC20X_RTC_H
#define __FIRECRACKER_PC20X_RTC_H

/* Register address offsets from RTC Base */
#define RTC_CURRECT_COUNTER_VALUE_REG_OFFSET     (0x00)
#define RTC_COUNTER_MATCH_REG_OFFSET             (0x04)
#define RTC_COUNTER_LOAD_REG_OFFSET              (0x08)
#define RTC_COUNTER_CONTROL_REG_OFFSET           (0x0c)
#define RTC_INTERRUPT_STATUS_REG_OFFSET          (0x10)
#define RTC_RAW_INTERRUPT_STATUS_REG_OFFSET      (0x14)
#define RTC_INTERRUPT_EOI_REG_OFFSET             (0x18)
#define RTC_COMPONENT_VERSION_REG_OFFSET         (0x1c)

/* RtcCounterControlRegOffset bit definitions */
#define INTERRUPT_ENABLE                         (0x01)
#define INTERRUPT_MASK                           (0x02)

/* Interrupt bits */
#define RTC_INTERRUPT                            (0x01)


#endif /* __FIRECRACKER_PC20X_RTC_H */



