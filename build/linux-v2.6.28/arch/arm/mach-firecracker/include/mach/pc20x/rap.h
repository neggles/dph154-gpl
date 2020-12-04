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
 *              Remap Block Register Definitions
 *
 *****************************************************************************/
#ifndef __FIRECRACKER_PC20X_RAP_H
#define __FIRECRACKER_PC20X_RAP_H


#define RAP_PAUSE_MODE_REG_OFFSET        0x00
#define RAP_ID_CODE_REG_OFFSET           0x04
#define RAP_REMAP_MODE_REG_OFFSET        0x08
#define RAP_RESET_STATUS_REG_OFFSET      0x0c
#define RAP_RESET_STATUS_CLR_OFFSET      0x10
#define RAP_COMP_VERSION_OFFSET          0x14

#define RAP_ID_CODE_VALUE                0x00000000

#define RAP_RESET_STATUS_SOFT_IDX        0
#define RAP_RESET_STATUS_WDOG_IDX        1
#define RAP_RESET_STATUS_GLBL_IDX        2

#define RAP_RESET_STATUS_SOFT_MASK      (1 << RAP_RESET_STATUS_SOFT_IDX)
#define RAP_RESET_STATUS_WDOG_MASK      (1 << RAP_RESET_STATUS_WDOG_IDX)
#define RAP_RESET_STATUS_GLBL_MASK      (1 << RAP_RESET_STATUS_GLBL_IDX)

#define RAP_PAUSE_MODE_REG_RESET         0x00000000
#define RAP_ID_CODE_REG_RESET            RAP_ID_CODE_VALUE
#define RAP_REMAP_MODE_REG_RESET         0x00000000
#define RAP_RESET_STATUS_REG_RESET       0x00000000
#define RAP_RESET_STATUS_CLR_RESET       0x00000000
#define RAP_COMP_VERSION_RESET           0x3230322b

#define REMAP_NORMAL_MODE                0x00000001

#endif /* __FIRECRACKER_PC20X_RAP_H */
