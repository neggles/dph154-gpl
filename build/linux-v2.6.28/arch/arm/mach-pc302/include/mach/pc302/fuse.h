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

#ifndef PC302_FUSE_H
#define PC302_FUSE_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define FUSE_MAP_0_REG_OFFSET           0x00
#define FUSE_MAP_1_REG_OFFSET           0x04
#define FUSE_MAP_2_REG_OFFSET           0x08
#define FUSE_MAP_3_REG_OFFSET           0x0C
#define FUSE_MAP_4_REG_OFFSET           0x10
#define FUSE_MAP_5_REG_OFFSET           0x14
#define FUSE_MAP_6_REG_OFFSET           0x18
#define FUSE_MAP_7_REG_OFFSET           0x1C
#define FUSE_MAP_8_REG_OFFSET           0x20
#define FUSE_MAP_9_REG_OFFSET           0x24
#define FUSE_MAP_10_REG_OFFSET          0x28
#define FUSE_MAP_11_REG_OFFSET          0x2C
#define FUSE_MAP_12_REG_OFFSET          0x30
#define FUSE_MAP_13_REG_OFFSET          0x34
#define FUSE_MAP_14_REG_OFFSET          0x38
#define FUSE_MAP_15_REG_OFFSET          0x3C
#define FUSE_MAP_16_REG_OFFSET          0x40
#define FUSE_MAP_17_REG_OFFSET          0x44
#define FUSE_MAP_18_REG_OFFSET          0x48
#define FUSE_MAP_19_REG_OFFSET          0x4C
#define FUSE_MAP_20_REG_OFFSET          0x50
#define FUSE_MAP_21_REG_OFFSET          0x54
#define FUSE_MAP_22_REG_OFFSET          0x58
#define FUSE_MAP_23_REG_OFFSET          0x5C
#define FUSE_MAP_24_REG_OFFSET          0x60
#define FUSE_MAP_25_REG_OFFSET          0x64
#define FUSE_MAP_26_REG_OFFSET          0x68
#define FUSE_MAP_27_REG_OFFSET          0x6C
#define FUSE_MAP_28_REG_OFFSET          0x70
#define FUSE_MAP_29_REG_OFFSET          0x74
#define FUSE_MAP_30_REG_OFFSET          0x78
#define FUSE_MAP_31_REG_OFFSET          0x7C
          
#define PC302_FUSE_CTRL_REG ( 0x200 )
#define PC302_FUSE_WR_BIT_ADDRESS_REG   ( 0x204 )
#define PC302_FUSE_WR_PERFORM_REG   ( 0x208 )
#define PC302_FUSE_WRITE_PAD_EN_REG ( 0x20C )
#define PC302_FUSE_WRITE_PAD_REG    ( 0x210 )

#define PC302_FUSE_CTRL_WRITE_BUSY  ( 1 << 0 )
#define PC302_FUSE_CTRL_VDDQ_OE     ( 1 << 1 )
#define PC302_FUSE_CTRL_VDDQ        ( 1 << 2 )

#define PC302_FUSE_WR_PERFORM_VALUE     ( 0x66757365 ) /* "fuse" */
#define PC302_FUSE_WRITE_PAD_EN_VALUE   ( 0x656E626C ) /* "enbl" */
#define PC302_FUSE_WRITE_PAD_VALUE      ( 0x56444451 ) /* "VDDQ" */

#endif /* PC302_FUSE_H */
