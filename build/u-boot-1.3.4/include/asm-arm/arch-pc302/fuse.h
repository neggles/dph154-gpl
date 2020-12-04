/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file fuse.h
* \brief Definitions for the PC302 Fuse Block.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/
#ifndef PC302_FUSE_H
#define PC302_FUSE_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define FUSE_MAP_0_REG_OFFSET           (0x00)
#define FUSE_MAP_1_REG_OFFSET           (0x04)
#define FUSE_MAP_2_REG_OFFSET           (0x08)
#define FUSE_MAP_3_REG_OFFSET           (0x0C)

#define FUSE_MAP_4_REG_OFFSET           (0x10)
#define FUSE_MAP_5_REG_OFFSET           (0x14)
#define FUSE_MAP_6_REG_OFFSET           (0x18)
#define FUSE_MAP_7_REG_OFFSET           (0x1C)

#define FUSE_MAP_8_REG_OFFSET           (0x20)
#define FUSE_MAP_9_REG_OFFSET           (0x24)
#define FUSE_MAP_10_REG_OFFSET          (0x28)
#define FUSE_MAP_11_REG_OFFSET          (0x2C)

#define FUSE_MAP_12_REG_OFFSET          (0x30)
#define FUSE_MAP_13_REG_OFFSET          (0x34)
#define FUSE_MAP_14_REG_OFFSET          (0x38)
#define FUSE_MAP_15_REG_OFFSET          (0x3C)

#define FUSE_MAP_16_REG_OFFSET          (0x40)
#define FUSE_MAP_17_REG_OFFSET          (0x44)
#define FUSE_MAP_18_REG_OFFSET          (0x48)
#define FUSE_MAP_19_REG_OFFSET          (0x4C)

#define FUSE_MAP_20_REG_OFFSET          (0x50)
#define FUSE_MAP_21_REG_OFFSET          (0x54)
#define FUSE_MAP_22_REG_OFFSET          (0x58)
#define FUSE_MAP_23_REG_OFFSET          (0x5C)

#define FUSE_MAP_24_REG_OFFSET          (0x60)
#define FUSE_MAP_25_REG_OFFSET          (0x64)
#define FUSE_MAP_26_REG_OFFSET          (0x68)
#define FUSE_MAP_27_REG_OFFSET          (0x6C)

#define FUSE_MAP_28_REG_OFFSET          (0x70)
#define FUSE_MAP_29_REG_OFFSET          (0x74)
#define FUSE_MAP_30_REG_OFFSET          (0x78)
#define FUSE_MAP_31_REG_OFFSET          (0x7C)

#define FUSE_CONTROL_REG_OFFSET         (0x80)
#define FUSE_WRITE_BIT_ADDR_REG_OFFSET  (0x84)
#define FUSE_WRITE_PERFORM              (0x88)

/*****************************************************************************/
/* Register Bit Field Manipulation                                           */
/*****************************************************************************/

#define FUSE_WRITE_FUSE_ENABLE          (0x66757365)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_FUSE_H */
