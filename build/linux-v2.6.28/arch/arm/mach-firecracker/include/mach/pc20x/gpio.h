/* Copyright (c) 2009 picoChip Designs Ltd.
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

#ifndef PC202_GPIO_H
#define PC202_GPIO_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define GPIO_SW_PORT_A_DR_REG_OFFSET        0x00    /*; Port A Data Register */
#define GPIO_SW_PORT_A_DDR_REG_OFFSET	    0x04    /*; Port A Data Direction Register */
#define GPIO_SW_PORT_A_CTL_REG_OFFSET       0x08    /*; Port A Data Source */

/* global GPIO registers */
#define GPIO_INT_EN_REG_OFFSET              0x30    /*; Interrupt enable */
#define GPIO_INT_MASK_REG_OFFSET            0x34    /*; Interrupt mask */
#define GPIO_INT_TYPE_LEVEL_REG_OFFSET 	    0x38    /*; Interrupt level */
#define GPIO_INT_POLARITY_REG_OFFSET 	    0x3c    /*; Interrupt polarity */

#define GPIO_INT_STATUS_REG_OFFSET	    0x40    /*; Interrupt status */
#define GPIO_RAW_INT_STATUS_REG_OFFSET	    0x44    /*; Raw interrupt status */

/* no debounce */
#define GPIO_PORT_A_EOI_REG_OFFSET          0x4c    /*; Clear interrupt */
#define GPIO_EXT_PORT_A_REG_OFFSET          0x50    /*; Value of External pins Port A */

#define GPIO_LS_SYNC_REG_OFFSET		    0x60    /*; Synchronization level */
#define GPIO_ID_CODE_REG_OFFSET		    0x64    /*; GPIO ID code */
#define GPIO_RESERVED_REG_OFFSET            0x68    /*; reserved */
#define GPIO_COMP_VERSION_REG_OFFSET	    0x6c    /*; GPIO Component Version */

/* Bit definitions */
#define GPIO(__N)                           (0x01 << (__N))

#endif /* PC202_GPIO_H */
