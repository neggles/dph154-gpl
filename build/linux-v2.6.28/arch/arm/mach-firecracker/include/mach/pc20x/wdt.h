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
 *              Watchdog Timer Register Definitions
 *
 *****************************************************************************/
#ifndef __FIRECRACKER_PC20X_WDT_H
#define __FIRECRACKER_PC20X_WDT_H

/* Register address offsets from WDT Base */
#define WDT_CONTROL_REG_OFFSET                      (0x00)
#define WDT_TIMEOUT_RANGE_REG_OFFSET                (0x04)
#define WDT_CURRECT_COUNTER_VALUE_REG_OFFSET        (0x08)
#define WDT_COUNTER_RESTART_REG_OFFSET              (0x0c)
#define WDT_INTERRUPT_STATUS_REG_OFFSET             (0x10)
#define WDT_INTERRUPT_EOI_REG_OFFSET                (0x14)


/* WdtControlRegOffset register bit definitions */
#define ENABLE_WDT                                  (0x01)
#define RESPONSE_MODE                               (0x02)
#define RESET_PULSE_LENGTH_2_PCLK                   (0x00 << 2)
#define RESET_PULSE_LENGTH_4_PCLK                   (0x01 << 2)
#define RESET_PULSE_LENGTH_8_PCLK                   (0x02 << 2)
#define RESET_PULSE_LENGTH_16_PCLK                  (0x03 << 2)
#define RESET_PULSE_LENGTH_32_PCLK                  (0x04 << 2)
#define RESET_PULSE_LENGTH_64_PCLK                  (0x05 << 2)
#define RESET_PULSE_LENGTH_128_PCLK                 (0x06 << 2)
#define RESET_PULSE_LENGTH_256_PCLK                 (0x07 << 2)

/* WdtTimeoutRangeRegOffset register bit definitions */
#define TIMEOUT_PERIOD_64K                          (0x00 << 0)
#define TIMEOUT_PERIOD_128K                         (0x01 << 0)
#define TIMEOUT_PERIOD_256K                         (0x02 << 0)
#define TIMEOUT_PERIOD_512K                         (0x03 << 0)
#define TIMEOUT_PERIOD_1M                           (0x04 << 0)
#define TIMEOUT_PERIOD_2M                           (0x05 << 0)
#define TIMEOUT_PERIOD_4M                           (0x06 << 0)
#define TIMEOUT_PERIOD_8M                           (0x07 << 0)
#define TIMEOUT_PERIOD_16M                          (0x08 << 0)
#define TIMEOUT_PERIOD_32M                          (0x09 << 0)
#define TIMEOUT_PERIOD_64M                          (0x0a << 0)
#define TIMEOUT_PERIOD_128M                         (0x0b << 0)
#define TIMEOUT_PERIOD_256M                         (0x0c << 0)
#define TIMEOUT_PERIOD_512M                         (0x0d << 0)
#define TIMEOUT_PERIOD_1G                           (0x0e << 0)
#define TIMEOUT_PERIOD_2G                           (0x0f << 0)
#define FIRST_TIMEOUT_PERIOD_64K                    (0x00 << 4)
#define FIRST_TIMEOUT_PERIOD_128K                   (0x01 << 4)
#define FIRST_TIMEOUT_PERIOD_256K                   (0x02 << 4)
#define FIRST_TIMEOUT_PERIOD_512K                   (0x03 << 4)
#define FIRST_TIMEOUT_PERIOD_1M                     (0x04 << 4)
#define FIRST_TIMEOUT_PERIOD_2M                     (0x05 << 4)
#define FIRST_TIMEOUT_PERIOD_4M                     (0x06 << 4)
#define FIRST_TIMEOUT_PERIOD_8M                     (0x07 << 4)
#define FIRST_TIMEOUT_PERIOD_16M                    (0x08 << 4)
#define FIRST_TIMEOUT_PERIOD_32M                    (0x09 << 4)
#define FIRST_TIMEOUT_PERIOD_64M                    (0x0a << 4)
#define FIRST_TIMEOUT_PERIOD_128M                   (0x0b << 4)
#define FIRST_TIMEOUT_PERIOD_256M                   (0x0c << 4)
#define FIRST_TIMEOUT_PERIOD_512M                   (0x0d << 4)
#define FIRST_TIMEOUT_PERIOD_1G                     (0x0e << 4)
#define FIRST_TIMEOUT_PERIOD_2G                     (0x0f << 4)

/* WdtCounterRestartRegOffset register timer restart code.
 * To restart the WDT, set this value.
 */
#define KICK_WDT                                    (0x76)

/* WdtInterruptStatusRegOffset register bits */
#define WDT_INTERRUPT                               (0x01)



#endif /* __FIRECRACKER_PC20X_WDT_H */



