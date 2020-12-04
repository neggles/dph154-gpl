/* Copyright (c) 2008 picoChip Designs Ltd.
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
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

#ifndef PC302_WDOG_H
#define PC302_WDOG_H

/* Constants -------------------------------------------------------------- */

#define WDOG_CONTROL_REG_OFFSET             0x00
#define WDOG_TIMEOUT_RANGE_REG_OFFSET       0x04
#define WDOG_CURRENT_COUNT_REG_OFFSET       0x08
#define WDOG_COUNTER_RESTART_REG_OFFSET     0x0c
#define WDOG_INT_STATUS_REG_OFFSET          0x10
#define WDOG_CLEAR_REG_OFFSET               0x14

#define WDOG_PARAMS_5_REG_OFFSET            0xe4
#define WDOG_PARAMS_4_REG_OFFSET            0xe8
#define WDOG_PARAMS_3_REG_OFFSET            0xec
#define WDOG_PARAMS_2_REG_OFFSET            0xf0
#define WDOG_PARAMS_1_REG_OFFSET            0xf4

#define WDOG_COMP_VERSION_REG_OFFSET        0xf8
#define WDOG_COMP_TYPE_REG_OFFSET           0xfc

#define WDOG_CONTROL_REG_RESET              0x00000016
#define WDOG_TIMEOUT_RANGE_REG_RESET        0x0000000c
#define WDOG_CURRENT_COUNT_REG_RESET        0x0fffffff
#define WDOG_COUNTER_RESTART_REG_RESET      0x00000000
#define WDOG_INT_STATUS_REG_RESET           0x00000000
#define WDOG_CLEAR_REG_RESET                0x00000000

/* Calculated from spec document */
#define WDOG_PARAMS_5_REG_RESET             0x7fffffff
#define WDOG_PARAMS_4_REG_RESET             0x00000000
#define WDOG_PARAMS_3_REG_RESET             0x0000000c
#define WDOG_PARAMS_2_REG_RESET             0x0fffffff
#define WDOG_PARAMS_1_REG_RESET             0x100c1602

#define WDOG_COMP_VERSION_REG_RESET         0x3130332a
#define WDOG_COMP_TYPE_REG_RESET            0x44570120

/* Kick value */
#define WDOG_COUNTER_RESTART_KICK_VALUE	    0x76

/* Control bits */
#define WDOGCONTROLREGWDT_ENIDX		    0
#define WDOGCONTROLREGRMODIDX		    1
#define WDOGCONTROLREGRPLIDX		    2

/* Masks */
#define WDOGCONTROLREGWDT_ENMASK	    1 << WDOGCONTROLREGWDT_ENIDX
#define WDOGCONTROLREGRMODMASK		    1 << WDOGCONTROLREGRMODIDX
#define WDOGCONTROLREGRPLMASK		    0x7 << WDOGCONTROLREGRPLIDX

#endif /* PC302_WDOG_H */
