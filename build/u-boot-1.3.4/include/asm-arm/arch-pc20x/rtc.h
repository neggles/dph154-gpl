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
 *              Real Time Clock Register Definitions
 *
 *****************************************************************************/
#ifndef __PC20X_RTC_H
#define __PC20X_RTC_H

/* Register address offsets from RTC Base */
#define RtcCurrectCounterValueRegOffset         (0x00)
#define RtcCounterMatchRegOffset                (0x04)
#define RtcCounterLoadRegOffset                 (0x08)
#define RtcCounterControlRegOffset              (0x0c)
#define RtcInterruptStatusRegOffset             (0x10)
#define RtcRawInterruptStatusRegOffset          (0x14)
#define RtcInterruptEOIRegOffset                (0x18)
#define RtcComponentVersionRegOffset            (0x1c)

/* RtcCounterControlRegOffset bit definitions */
#define InterruptEnable                         (0x01)
#define InterruptMask                           (0x02)

/* Interrupt bits */
#define RtcInterrupt                            (0x01)


#endif /* __PC20X_RTC_H */



