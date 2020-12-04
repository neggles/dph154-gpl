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
 *              Watchdog Timer Register Definitions
 *
 *****************************************************************************/
#ifndef __PC20X_WDT_H
#define __PC20X_WDT_H

/* Register address offsets from WDT Base */
#define WdtControlRegOffset                     (0x00)
#define WdtTimeoutRangeRegOffset                (0x04)
#define WdtCurrectCounterValueRegOffset         (0x08)
#define WdtCounterRestartRegOffset              (0x0c)
#define WdtInterruptStatusRegOffset             (0x10)
#define WdtInterruptEOIRegOffset                (0x14)


/* WdtControlRegOffset register bit definitions */
#define EnableWdt                               (0x01)
#define ResponseMode                            (0x02)
#define ResetPulseLength2pclk                   (0x00 << 2)
#define ResetPulseLength4pclk                   (0x01 << 2)
#define ResetPulseLength8pclk                   (0x02 << 2)
#define ResetPulseLength16pclk                  (0x03 << 2)
#define ResetPulseLength32pclk                  (0x04 << 2)
#define ResetPulseLength64pclk                  (0x05 << 2)
#define ResetPulseLength128pclk                 (0x06 << 2)
#define ResetPulseLength256pclk                 (0x07 << 2)

/* WdtTimeoutRangeRegOffset register bit definitions */
#define TimeoutPeriod64K                        (0x00 << 0)
#define TimeoutPeriod128K                       (0x01 << 0)
#define TimeoutPeriod256K                       (0x02 << 0)
#define TimeoutPeriod512K                       (0x03 << 0)
#define TimeoutPeriod1M                         (0x04 << 0)
#define TimeoutPeriod2M                         (0x05 << 0)
#define TimeoutPeriod4M                         (0x06 << 0)
#define TimeoutPeriod8M                         (0x07 << 0)
#define TimeoutPeriod16M                        (0x08 << 0)
#define TimeoutPeriod32M                        (0x09 << 0)
#define TimeoutPeriod64M                        (0x0a << 0)
#define TimeoutPeriod128M                       (0x0b << 0)
#define TimeoutPeriod256M                       (0x0c << 0)
#define TimeoutPeriod512M                       (0x0d << 0)
#define TimeoutPeriod1G                         (0x0e << 0)
#define TimeoutPeriod2G                         (0x0f << 0)
#define FirstTimeoutPeriod64K                   (0x00 << 4)
#define FirstTimeoutPeriod128K                  (0x01 << 4)
#define FirstTimeoutPeriod256K                  (0x02 << 4)
#define FirstTimeoutPeriod512K                  (0x03 << 4)
#define FirstTimeoutPeriod1M                    (0x04 << 4)
#define FirstTimeoutPeriod2M                    (0x05 << 4)
#define FirstTimeoutPeriod4M                    (0x06 << 4)
#define FirstTimeoutPeriod8M                    (0x07 << 4)
#define FirstTimeoutPeriod16M                   (0x08 << 4)
#define FirstTimeoutPeriod32M                   (0x09 << 4)
#define FirstTimeoutPeriod64M                   (0x0a << 4)
#define FirstTimeoutPeriod128M                  (0x0b << 4)
#define FirstTimeoutPeriod256M                  (0x0c << 4)
#define FirstTimeoutPeriod512M                  (0x0d << 4)
#define FirstTimeoutPeriod1G                    (0x0e << 4)
#define FirstTimeoutPeriod2G                    (0x0f << 4)

/* WdtCounterRestartRegOffset register timer restart code.
 * To restart the WDT, set this value.
 */
#define KickWdt                                 (0x76)

/* WdtInterruptStatusRegOffset register bits */
#define WdtInterrupt                            (0x01)



#endif /* __PC20X_WDT_H */



