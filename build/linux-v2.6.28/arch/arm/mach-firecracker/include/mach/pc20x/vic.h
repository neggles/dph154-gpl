
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
 *              VIC Register Definitions
 *
 *****************************************************************************/ 
#ifndef __FIRECRACKER_PC20X_VIC_H
#define __FIRECRACKER_PC20X_VIC_H

/* VIC registers */
#define VIC_ENABLE_REG_OFFSET                           (0x00)
#define VIC_MASK_REG_OFFSET                             (0x08)
#define VIC_FORCE_REG_OFFSET                            (0x10)
#define VIC_RAW_STATUS_REG_OFFSET                       (0x18)
#define VIC_STATUS_REG_OFFSET                           (0x20)
#define VIC_MASK_STATUS_REG_OFFSET                      (0x28)
#define VIC_FINAL_STATUS_REG_OFFSET                     (0x30)
#define VIC_VECTOR_REG_OFFSET                           (0x38)
#define VIC_VECTOR_PRIORITY_N_REG_OFFSET(__N)           (0x40 + 0x08 * (__N))
#define VIC_PRIORITY_LEVEL_REG_OFFSET                   (0xd8)
#define VIC_IRQ_N_PRIORITY_LEVEL_REG_OFFSET(__N)        (0xe8 + 0x04 * (__N))

/* Bit definitions */

#define IRQ_BIT(__N)                                    (0x00000001 << (__N))

/* The active IRQs bitmask */
#define VIC_USED_IRQ_MASK                               (0x3fffffff)

#endif /* __FIRECRACKER_PC20X_VIC_H */


