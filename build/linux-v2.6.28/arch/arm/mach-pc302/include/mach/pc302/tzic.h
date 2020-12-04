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

#ifndef PC302_TZIC_H
#define PC302_TZIC_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

/* Functional Registers */

#define TZIC_FIQ_STATUS_REG_OFFSET              0x0000
#define TZIC_RAW_INT_STATUS_REG_OFFSET          0x0004
#define TZIC_INT_SELECT_REG_OFFSET              0x0008
#define TZIC_FIQ_ENABLE_REG_OFFSET              0x000C
#define TZIC_FIQ_ENABLE_CLEAR_REG_OFFSET        0x0010
#define TZIC_FIQ_BYPASS_REG_OFFSET              0x0014
#define TZIC_PROTECTION_REG_OFFSET              0x0018
#define TZIC_LOCK_REG_OFFSET                    0x001C
#define TZIC_LOCK_STATUS_REG_OFFSET             0x0020

/* Identification Registers */

#define TZIC_PERIPHERAL_ID_0_REG_OFFSET         0x0FE0
#define TZIC_PERIPHERAL_ID_1_REG_OFFSET         0x0FE4
#define TZIC_PERIPHERAL_ID_2_REG_OFFSET         0x0FE8
#define TZIC_PERIPHERAL_ID_3_REG_OFFSET         0x0FEC
#define TZIC_PRIMECELL_ID_0_REG_OFFSET          0x0FF0
#define TZIC_PRIMECELL_ID_1_REG_OFFSET          0x0FF4
#define TZIC_PRIMECELL_ID_2_REG_OFFSET          0x0FF8
#define TZIC_PRIMECELL_ID_3_REG_OFFSET          0x0FFC


/*****************************************************************************/
/* Register Reset Values                                                     */
/*****************************************************************************/

/* Functional Registers */

#define TZIC_FIQ_STATUS_REG_RESET               0x00000000
/* TZIC_RAW_INT_STATUS_REG_RESET  Unknown reset value */
#define TZIC_INT_SELECT_REG_RESET               0x00000000
#define TZIC_FIQ_ENABLE_REG_RESET               0x00000000
/* TZIC_FIQ_ENABLE_CLEAR_REG_RESET  Unknown reset value */
#define TZIC_FIQ_BYPASS_REG_RESET               0x00000000
#define TZIC_PROTECTION_REG_RESET               0x00000000
/* TZIC_LOCK_REG_RESET  Unknown reset value */
#define TZIC_LOCK_STATUS_REG_RESET              0x00000001

/* Identification Registers */

#define TZIC_PERIPHERAL_ID_0_REG_RESET          0x00000090
#define TZIC_PERIPHERAL_ID_1_REG_RESET          0x00000018
#define TZIC_PERIPHERAL_ID_2_REG_RESET          0x00000004
#define TZIC_PERIPHERAL_ID_3_REG_RESET          0x00000000
#define TZIC_PRIMECELL_ID_0_REG_RESET           0x0000000D
#define TZIC_PRIMECELL_ID_1_REG_RESET           0x000000F0
#define TZIC_PRIMECELL_ID_2_REG_RESET           0x00000005
#define TZIC_PRIMECELL_ID_3_REG_RESET           0x000000B1

/*****************************************************************************/
/* Register Bit Field Manipulation                                           */
/*****************************************************************************/

#define TZIC_FIQ_BYPASS_REG_MASK                0x00000001
#define TZIC_PROTECTION_REG_MASK                0x00000001
#define TZIC_LOCK_STATUS_REG_MASK               0x00000001

#define TZIC_PERIPHERAL_ID_MASK                 0x000000FF
#define TZIC_PRIMECELL_ID_MASK                  0x000000FF  

#define TZIC_UNLOCK_ACCESS_CODE                 0x0ACCE550

#endif /* PC302_TZIC_H */
