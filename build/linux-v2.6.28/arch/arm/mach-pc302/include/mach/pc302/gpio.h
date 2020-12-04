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

#ifndef PC302_GPIO_H
#define PC302_GPIO_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define GPIO_SW_PORT_A_DR_REG_OFFSET        0x00    /*; Port A Data Register */
#define GPIO_SW_PORT_A_DDR_REG_OFFSET	    0x04    /*; Port A Data Direction Register */
#define GPIO_SW_PORT_A_CTL_REG_OFFSET       0x08    /*; Port A Data Source */
#define GPIO_SW_PORT_B_DR_REG_OFFSET        0x0C    /*; Port B Data Register */
#define GPIO_SW_PORT_B_DDR_REG_OFFSET	    0x10    /*; Port B Data Direction Register */
#define GPIO_SW_PORT_B_CTL_REG_OFFSET       0x14    /*; Port B Data Source */
#define GPIO_SW_PORT_C_DR_REG_OFFSET        0x18    /*; Port C Data Register */
#define GPIO_SW_PORT_C_DDR_REG_OFFSET	    0x1C    /*; Port C Data Direction Register */
#define GPIO_SW_PORT_C_CTL_REG_OFFSET       0x20    /*; Port C Data Source */

/* no port {D} */

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
#define GPIO_EXT_PORT_B_REG_OFFSET          0x54    /*; Value of External pins Port B */
#define GPIO_EXT_PORT_C_REG_OFFSET          0x58    /*; Value of External pins Port C */

/* no port {D} */

#define GPIO_LS_SYNC_REG_OFFSET		    0x60    /*; Synchronization level */
#define GPIO_ID_CODE_REG_OFFSET		    0x64    /*; GPIO ID code */
#define GPIO_RESERVED_REG_OFFSET            0x68    /*; reserved */
#define GPIO_COMP_VERSION_REG_OFFSET	    0x6c    /*; GPIO Component Version */

/*****************************************************************************/
/* Register Reset Values                                                     */
/*****************************************************************************/

/*Address offset from GPIO_BASE */
#define GPIO_SW_PORT_A_DR_REG_RESET         0x00000000  /*; Port A Data Register */
#define GPIO_SW_PORT_A_DDR_REG_RESET        0x00000000  /*; Port A Data Direction Register */
#define GPIO_SW_PORT_A_CTL_REG_RESET        0x00000000  /*; Port A A Data Source */
#define GPIO_SW_PORT_B_DR_REG_RESET         0x00000000  /*; Port B Data Register */
#define GPIO_SW_PORT_B_DDR_REG_RESET        0x00000000  /*; Port B Data Direction Register */
#define GPIO_SW_PORT_B_CTL_REG_RESET        0x00000000  /*; Port B Data Source */
#define GPIO_SW_PORT_C_DR_REG_RESET         0x00000000  /*; Port C Data Register */
#define GPIO_SW_PORT_C_DDR_REG_RESET        0x00000000  /*; Port C Data Direction Register */
#define GPIO_SW_PORT_C_CTL_REG_RESET        0x00000000  /*; Port C Data Source */

/* no port {D} */

/* global GPIO registers */
#define GPIO_INT_EN_REG_RESET               0x00000000	/*; Interrupt enable */
#define GPIO_INT_MASK_REG_RESET	 	    0x00000000	/*; Interrupt mask */
#define GPIO_INT_TYPE_LEVEL_REG_RESET 	    0x00000000	/*; Interrupt level */
#define GPIO_INT_POLARITY_REG_RESET 	    0x00000000	/*; Interrupt polarity */

#define GPIO_INT_STATUS_REG_RESET           0x00000000  /*; Interrupt status */
#define GPIO_RAW_INT_STATUS_REG_RESET       0x00000000  /*; Raw interrupt status */

/* no debounce */

#define GPIO_PORT_A_EOI_REG_RESET           0x00000000	/*; Clear interrupt */
#define GPIO_EXT_PORT_A_REG_RESET           0x00000000	/*; Value of External pins Port A */
#define GPIO_EXT_PORT_B_REG_RESET           0x00000000	/*; Value of External pins Port B */
#define GPIO_EXT_PORT_C_REG_RESET           0x00000000	/*; Value of External pins Port C */

/* no port {D} */

#define GPIO_LS_SYNC_REG_RESET		    0x00000000  /*; Synchronization level */
#define GPIO_ID_CODE_REG_RESET		    0x00000000  /*; GPIO ID code */
#define GPIO_COMP_VERSION_REG_RESET         0x3230362A  /* ; 2.06b => '2.06*' */

/* Bit definitions */

#define GPIO_BIT_7 0x80
#define GPIO_BIT_6 0x40
#define GPIO_BIT_5 0x20
#define GPIO_BIT_4 0x10
#define GPIO_BIT_3 0x08
#define GPIO_BIT_2 0x04
#define GPIO_BIT_1 0x02
#define GPIO_BIT_0 0x01

#endif /* PC302_GPIO_H */
