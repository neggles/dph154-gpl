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

#ifndef PC302_MEM_ARM_H
#define PC302_MEM_ARM_H

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
#define MEMIF_ARM_AXI_GP0_OFFSET            (0x20 * 4)
#define MEMIF_ARM_AXI_GP1_OFFSET            (0x21 * 4)
#define MEMIF_ARM_AXI_GP2_OFFSET            (0x22 * 4)
#define MEMIF_ARM_AXI_GP3_OFFSET            (0x23 * 4)
#define MEMIF_ARM_AXI_GP4_OFFSET            (0x24 * 4)
#define MEMIF_ARM_AXI_GPSTATUS0_OFFSET      (0x2A * 4)
#define MEMIF_ARM_AXI_GPSTATUS1_OFFSET      (0x2B * 4)
#define MEMIF_ARM_AXI_GPSTATUS2_OFFSET      (0x2C * 4)
#define MEMIF_ARM_AXI_GPSTATUS3_OFFSET      (0x2D * 4)
#define MEMIF_ARM_AXI_GPSTATUS4_OFFSET      (0x2E * 4)

/* 31 - 57 UNUSED */

#define MEMIF_ARM_AXI_HP_MSTR_0_OFFSET      (0x3A * 4)      
#define MEMIF_ARM_AXI_HP_MSTR_1_OFFSET      (0x3B * 4)      
#define MEMIF_ARM_AXI_HP_MSTR_2_OFFSET      (0x3C * 4)      
#define MEMIF_ARM_AXI_HP_MSTR_3_OFFSET      (0x3D * 4)      
#define MEMIF_ARM_AXI_START_ADDR_OFFSET     (0x3E * 4)
#define MEMIF_ARM_AXI_END_ADDR_OFFSET       (0x3F * 4)
                
/*****************************************************************************/
/* Register Values                                                           */
/*****************************************************************************/

/* MEMIF_ARM_DRAM_EMR_MR */        

#define MEMIF_ARM_DRAM_EMR_MR_VAL                 0x00000A63        

/* MEMIF_ARM_ADDR_MAP0-2 */        

#define MEMIF_ARM_ADDR_MAP_0_BANK_BIT0_IDX        0
#define MEMIF_ARM_ADDR_MAP_0_BANK_BIT1_IDX        4
#define MEMIF_ARM_ADDR_MAP_0_BANK_BIT2_IDX        8

#define MEMIF_ARM_ADDR_MAP_1_COL_BIT2_IDX         0
#define MEMIF_ARM_ADDR_MAP_1_COL_BIT3_IDX         4
#define MEMIF_ARM_ADDR_MAP_1_COL_BIT4_6_IDX       8
#define MEMIF_ARM_ADDR_MAP_1_COL_BIT7_IDX         12
#define MEMIF_ARM_ADDR_MAP_1_COL_BIT8_IDX         16
#define MEMIF_ARM_ADDR_MAP_1_COL_BIT9_IDX         20
#define MEMIF_ARM_ADDR_MAP_1_COL_BIT10_IDX        24
#define MEMIF_ARM_ADDR_MAP_1_COL_BIT11_IDX        28

#define MEMIF_ARM_ADDR_MAP_2_ROW_BIT0_IDX         0
#define MEMIF_ARM_ADDR_MAP_2_ROW_BIT1_IDX         4
#define MEMIF_ARM_ADDR_MAP_2_ROW_BIT2_11_IDX      8   
#define MEMIF_ARM_ADDR_MAP_2_ROW_BIT12_IDX        12
#define MEMIF_ARM_ADDR_MAP_2_ROW_BIT13_IDX        16
#define MEMIF_ARM_ADDR_MAP_2_ROW_BIT14_IDX        20
#define MEMIF_ARM_ADDR_MAP_2_ROW_BIT15_IDX        24

#define MEMIF_ARM_ADDR_MAP_0_VAL_512Mbx16         0x00000F77  /* bank */
#define MEMIF_ARM_ADDR_MAP_1_VAL_512Mbx16         0xFFF00000  /* col */
#define MEMIF_ARM_ADDR_MAP_2_VAL_512Mbx16         0x0FF22222  /* row */

/* MEMIF_ARM_PHY_CMD_RDC */        

#define MEMIF_ARM_PHY_CMD_RDC_USE_FIXED_RE_IDX    16
#define MEMIF_ARM_PHY_CMD_RDC_WE_TO_RE_DEL_IDX    8
#define MEMIF_ARM_PHY_CMD_RDC_WR_CMD_TO_DATA_IDX  4
#define MEMIF_ARM_PHY_CMD_RDC_RD_CMD_TO_DATA_IDX  0

#define MEMIF_ARM_PHY_CMD_RDC_USE_FIXED_RE_MSK    0x1
#define MEMIF_ARM_PHY_CMD_RDC_WE_TO_RE_DEL_MSK    0x3
#define MEMIF_ARM_PHY_CMD_RDC_WR_CMD_TO_DATA_MSK  0xF
#define MEMIF_ARM_PHY_CMD_RDC_RD_CMD_TO_DATA_MSK  0xF
    
/* MEMIF_ARM_PHY_SLV_DLL */

/* note Hoyle_DDRC spec v0.12 is wrong, this is the correct bit mapping */
#define MEMIF_ARM_PHY_SLV_DLL_RD_RATIO0_IDX       16
#define MEMIF_ARM_PHY_SLV_DLL_RD_RATIO1_IDX       24
#define MEMIF_ARM_PHY_SLV_DLL_WR_RATIO_IDX        0

#define MEMIF_ARM_PHY_SLV_DLL_RD_RATIO0_MSK       0xff
#define MEMIF_ARM_PHY_SLV_DLL_RD_RATIO1_MSK       0xff
#define MEMIF_ARM_PHY_SLV_DLL_WR_RATIO_MSK        0xff

/* MEMIF_ARM_PHY_DEBUG_1 */
    
#define MEMIF_ARM_PHY_DEBUG_1_RC_DLL_SVAL0_IDX    0
#define MEMIF_ARM_PHY_DEBUG_1_RC_DLL_SVAL1_IDX    9
#define MEMIF_ARM_PHY_DEBUG_1_WR_DLL_SVAL_IDX     18

#define MEMIF_ARM_PHY_DEBUG_1_RC_DLL_SVAL0_MSK    0x1ff
#define MEMIF_ARM_PHY_DEBUG_1_RC_DLL_SVAL1_MSK    0x1ff
#define MEMIF_ARM_PHY_DEBUG_1_WR_DLL_SVAL_MSK     0x1ff

/* MEMIF_ARM_PHY_DEBUG_2 */
    
#define MEMIF_ARM_PHY_DEBUG_2_MSTR_DLL_LOCK0_IDX  0
#define MEMIF_ARM_PHY_DEBUG_2_MSTR_DLL_SVAL0_IDX  2

#define MEMIF_ARM_PHY_DEBUG_2_MSTR_DLL_LOCK0_MSK  0x1
#define MEMIF_ARM_PHY_DEBUG_2_MSTR_DLL_SVAL0_MSK  0x1ff

/* MEMIF_ARM_PHY_DEBUG_3 */
    
#define MEMIF_ARM_PHY_DEBUG_3_IN_LOCK_IDX         0
#define MEMIF_ARM_PHY_DEBUG_3_IN_DELAY_IDX        2
#define MEMIF_ARM_PHY_DEBUG_3_OUT_DELAY_IDX       11

#define MEMIF_ARM_PHY_DEBUG_3_IN_LOCK_MSK         0x3
#define MEMIF_ARM_PHY_DEBUG_3_IN_DELAY_MSK        0x1ff
#define MEMIF_ARM_PHY_DEBUG_3_OUT_DELAY_MSK       0x1ff

/* MEMIF_ARM_AXI_GP0 */

#define MEMIF_ARM_AXI_GP0_LOOPBACK_MODE_IDX       0
#define MEMIF_ARM_AXI_GP0_LOOPBACK_ERR_CLR_IDX    1

/* MEMIF_ARM_AXI_GP1 */

#define MEMIF_ARM_AXI_GP1_WL_DLL_FORCE_IDX        0
#define MEMIF_ARM_AXI_GP1_RC_DLL_FORCE0_IDX       1
#define MEMIF_ARM_AXI_GP1_RC_DLL_FORCE1_IDX       2
#define MEMIF_ARM_AXI_GP1_WL_DLL_VALUE_IDX        3
#define MEMIF_ARM_AXI_GP1_RC_DLL_VALUE0_IDX       12
#define MEMIF_ARM_AXI_GP1_RC_DLL_VALUE1_IDX       21

/* MEMIF_ARM_AXI_GPSTATUS0 */

#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_DONE_IDX  0
#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_FAIL_IDX  1
#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_ERR0_IDX  2
#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_ERR1_IDX  3
#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_ERR2_IDX  4
#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_ERR3_IDX  5
#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_ERR4_IDX  6

#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_DONE_MSK 0x1
#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_FAIL_MSK 0x1
#define MEMIF_ARM_AXI_GPSTATUS0_LOOPBACK_ERR_MSK  0x1f


/*****************************************************************************/
/* Base Addresses                                                            */
/*****************************************************************************/
        
/* col  [9:0]  = axaddr[10:1] */
/* bank [1:0]  = axaddr[12:11] */
/* row  [12:0] = axaddr[25:13] */      
#define BANK0_BASE_512Mbx16                 0x00000000 
#define BANK1_BASE_512Mbx16                 0x00000800
#define BANK2_BASE_512Mbx16                 0x00001000
#define BANK3_BASE_512Mbx16                 0x00001800

#endif /* PC302_MEM_ARM_H */
