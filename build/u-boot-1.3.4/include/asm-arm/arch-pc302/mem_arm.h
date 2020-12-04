/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file mem_arm.h
* \brief Definitions for the PC302 Memif-ARM Block.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_MEM_ARM_H
#define PC302_MEM_ARM_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define MEMIF_ARM_INDIRECT_RW_CMD_OFFSET    (0x00 * 4)
#define MEMIF_ARM_RSVD_0_OFFSET             (0x01 * 4)
#define MEMIF_ARM_HPR_OFFSET                (0x02 * 4)
#define MEMIF_ARM_LPR_OFFSET                (0x03 * 4)
#define MEMIF_ARM_WR_OFFSET                 (0x04 * 4)
#define MEMIF_ARM_DRAM_PARAM_0_OFFSET       (0x05 * 4)
#define MEMIF_ARM_DRAM_PARAM_1_OFFSET       (0x06 * 4)
#define MEMIF_ARM_DRAM_PARAM_2_OFFSET       (0x07 * 4)
#define MEMIF_ARM_DRAM_PARAM_3_OFFSET       (0x08 * 4)
#define MEMIF_ARM_DRAM_PARAM_4_OFFSET       (0x09 * 4)
#define MEMIF_ARM_DRAM_INIT_PARAM_OFFSET    (0x0A * 4)
#define MEMIF_ARM_DRAM_EMR2_EMR3_OFFSET     (0x0B * 4)
#define MEMIF_ARM_DRAM_EMR_MR_OFFSET        (0x0C * 4)
#define MEMIF_ARM_DRAM_BL_OFFSET            (0x0D * 4)
#define MEMIF_ARM_DRAM_FORCE_LPR_OFFSET     (0x0E * 4)
#define MEMIF_ARM_ADDR_MAP_0_OFFSET         (0x0F * 4)
#define MEMIF_ARM_ADDR_MAP_1_OFFSET         (0x10 * 4)
#define MEMIF_ARM_ADDR_MAP_2_OFFSET         (0x11 * 4)
#define MEMIF_ARM_DRAM_ODT_OFFSET           (0x12 * 4)
#define MEMIF_ARM_PHY_DEBUG_0_OFFSET        (0x13 * 4)
#define MEMIF_ARM_PHY_CMD_RDC_OFFSET        (0x14 * 4)
#define MEMIF_ARM_CTRL_MODE_OFFSET          (0x15 * 4)
#define MEMIF_ARM_DLL_CALIB_OFFSET          (0x16 * 4)
#define MEMIF_ARM_ODT_CTRL_OFFSET           (0x17 * 4)
#define MEMIF_ARM_DDRC_CTRL_0_OFFSET        (0x18 * 4)
#define MEMIF_ARM_DDRC_CTRL_1_OFFSET        (0x19 * 4)
#define MEMIF_ARM_PHY_SLV_DLL_OFFSET        (0x1A * 4)
#define MEMIF_ARM_PHY_DEBUG_1_OFFSET        (0x1B * 4)
#define MEMIF_ARM_PHY_DEBUG_2_OFFSET        (0x1C * 4)
#define MEMIF_ARM_PHY_DEBUG_3_OFFSET        (0x1D * 4)
#define MEMIF_ARM_PHY_DEBUG_4_OFFSET        (0x1E * 4)
#define MEMIF_ARM_PHY_LOCAL_ODT_OFFSET      (0x1F * 4)
#define MEMIF_ARM_GP0_OFFSET                (0x20 * 4)
#define MEMIF_ARM_GP1_OFFSET                (0x21 * 4)
#define MEMIF_ARM_GP2_OFFSET                (0x22 * 4)
#define MEMIF_ARM_PHY_LOCAL_DRV_STRENGTH_OFFSET \
        (MEMIF_ARM_GP2_OFFSET)
#define MEMIF_ARM_GP3_OFFSET                (0x23 * 4)
#define MEMIF_ARM_GP4_OFFSET                (0x24 * 4)

#define MEMIF_ARM_AXI_HP_MSTR_0_OFFSET      (0x3A * 4)
#define MEMIF_ARM_AXI_HP_MSTR_1_OFFSET      (0x3B * 4)
#define MEMIF_ARM_AXI_HP_MSTR_2_OFFSET      (0x3C * 4)
#define MEMIF_ARM_AXI_HP_MSTR_3_OFFSET      (0x3D * 4)
#define MEMIF_ARM_AXI_START_ADDR_OFFSET     (0x3E * 4)
#define MEMIF_ARM_AXI_END_ADDR_OFFSET       (0x3F * 4)

/*****************************************************************************/
/* Register Values                                                           */
/*****************************************************************************/

/* Set up values for a single 512Mbx16 sdram device */
#define MEMIF_ARM_ADDR_MAP_0_VAL_512Mbx16   (0x00000F77)  /* bank */
#define MEMIF_ARM_ADDR_MAP_1_VAL_512Mbx16   (0xFFF00000)  /* col */
#define MEMIF_ARM_ADDR_MAP_2_VAL_512Mbx16   (0x0FF22222)  /* row */

/* Set up values for a single 1Gbx16 sdram device */
#define MEMIF_ARM_ADDR_MAP_0_VAL_1Gbx16     (0x00000777)  /* bank */
#define MEMIF_ARM_ADDR_MAP_1_VAL_1Gbx16     (0xFFF00000)  /* col */
#define MEMIF_ARM_ADDR_MAP_2_VAL_1Gbx16     (0x0FF33333)  /* row */

#define MEMIF_ARM_DRAM_EMR_MR_VAL           (0x00000A63)

/* final wait after ddr initialisation sequence is 8 cycles */
#define MEMIF_ARM_DRAM_INIT_PARAM_FWAIT_IDX     (0)
#define MEMIF_ARM_DRAM_INIT_PARAM_FWAIT_MSK     (0x3F)
#define MEMIF_ARM_DRAM_INIT_PARAM_FWAIT_VAL     (0x8)

/* pre cke assertion delay must be 200us */
#define MEMIF_ARM_DRAM_INIT_PARAM_PRE_CKE_IDX   (14)
#define MEMIF_ARM_DRAM_INIT_PARAM_PRE_CKE_MSK   (0xFF)
#define MEMIF_ARM_DRAM_INIT_PARAM_PRE_CKE_VAL   (0x50)

/* post cke assertion is 400ns */
#define MEMIF_ARM_DRAM_INIT_PARAM_PST_CKE_IDX   (22)
#define MEMIF_ARM_DRAM_INIT_PARAM_PST_CKE_MSK   (0xFF)
#define MEMIF_ARM_DRAM_INIT_PARAM_PST_CKE_VAL   (0x2)

/*****************************************************************************/
/* Base Addresses                                                            */
/*****************************************************************************/

/* col  [9:0]  = axaddr[10:1] */
/* bank [1:0]  = axaddr[12:11] */
/* row  [12:0] = axaddr[25:13] */
#define BANK0_BASE_512Mbx16                 (0x00000000)
#define BANK1_BASE_512Mbx16                 (0x00000800)
#define BANK2_BASE_512Mbx16                 (0x00001000)
#define BANK3_BASE_512Mbx16                 (0x00001800)

/* col  [9:0]  = axaddr[10:1] */
/* bank [2:0]  = axaddr[13:11] */
/* row  [13:0] = axaddr[27:14] */
#define BANK0_BASE_1Gbx8                    (0x00000000)
#define BANK1_BASE_1Gbx8                    (0x00000800)
#define BANK2_BASE_1Gbx8                    (0x00001000)
#define BANK3_BASE_1Gbx8                    (0x00001800)
#define BANK4_BASE_1Gbx8                    (0x00002000)
#define BANK5_BASE_1Gbx8                    (0x00002800)
#define BANK6_BASE_1Gbx8                    (0x00003000)
#define BANK7_BASE_1Gbx8                    (0x00003800)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_MEM_ARM_H */
