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

#ifndef PC302_TZPC_H
#define PC302_TZPC_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

/* Functional Registers */

#define TZPC_R0_SIZE_REG_OFFSET                 0x0000
#define TZPC_DEC_PROT_0_STAT_REG_OFFSET         0x0800
#define TZPC_DEC_PROT_0_SET_REG_OFFSET          0x0804
#define TZPC_DEC_PROT_0_CLE_REG_OFFSET          0x0808
#define TZPC_DEC_PROT_1_STAT_REG_OFFSET         0x080c
#define TZPC_DEC_PROT_1_SET_REG_OFFSET          0x0810
#define TZPC_DEC_PROT_1_CLE_REG_OFFSET          0x0814
#define TZPC_DEC_PROT_2_STAT_REG_OFFSET         0x0818
#define TZPC_DEC_PROT_2_SET_REG_OFFSET          0x081c
#define TZPC_DEC_PROT_2_CLE_REG_OFFSET          0x0820

/* Identification Registers */

#define TZPC_PERIPHERAL_ID_0_REG_OFFSET         0x0FE0
#define TZPC_PERIPHERAL_ID_1_REG_OFFSET         0x0FE4
#define TZPC_PERIPHERAL_ID_2_REG_OFFSET         0x0FE8
#define TZPC_PERIPHERAL_ID_3_REG_OFFSET         0x0FEC
#define TZPC_PRIMECELL_ID_0_REG_OFFSET          0x0FF0
#define TZPC_PRIMECELL_ID_1_REG_OFFSET          0x0FF4
#define TZPC_PRIMECELL_ID_2_REG_OFFSET          0x0FF8
#define TZPC_PRIMECELL_ID_3_REG_OFFSET          0x0FFC


/*****************************************************************************/
/* Register Reset Values                                                     */
/*****************************************************************************/

/* Functional Registers */

#define TZPC_R0_SIZE_REG_RESET                  0x00000200
#define TZPC_DEC_PROT_0_STAT_REG_RESET          0x00000000
/* TZPC_DEC_PROT_0_SET_REG_RESET         EQU Unknown reset value */
/* TZPC_DEC_PROT_0_CLE_REG_RESET         EQU Unknown reset value */
#define TZPC_DEC_PROT_1_STAT_REG_RESET          0x00000000
/* TZPC_DEC_PROT_1_SET_REG_RESET         EQU Unknown reset value */
/* TZPC_DEC_PROT_1_CLE_REG_RESET         EQU Unknown reset value */
#define TZPC_DEC_PROT_2_STAT_REG_RESET          0x00000000
/* TZPC_DEC_PROT_2_SET_REG_RESET         EQU Unknown reset value */
/* TZPC_DEC_PROT_2_CLE_REG_RESET         EQU Unknown reset value */

/* Identification Registers */

#define TZPC_PERIPHERAL_ID_0_REG_RESET          0x00000070
#define TZPC_PERIPHERAL_ID_1_REG_RESET          0x00000018
#define TZPC_PERIPHERAL_ID_2_REG_RESET          0x00000004
#define TZPC_PERIPHERAL_ID_3_REG_RESET          0x00000000
#define TZPC_PRIMECELL_ID_0_REG_RESET           0x0000000D
#define TZPC_PRIMECELL_ID_1_REG_RESET           0x000000F0
#define TZPC_PRIMECELL_ID_2_REG_RESET           0x00000005
#define TZPC_PRIMECELL_ID_3_REG_RESET           0x000000B1

#endif /* PC302_TZPC_H */
