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

#ifndef PC302_MEMIF_H
#define PC302_MEMIF_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define MEMIF_SDRAM_SETUP_REG_OFFSET            0x00A0
#define MEMIF_SDRAM_REFRESH_REG_OFFSET          0x00A2
#define MEMIF_SDRAM_MRS_REG_OFFSET              0x00A4
#define MEMIF_SDRAM_EMRS_REG_OFFSET             0x00A6
#define MEMIF_SDRAM_CFG_DONE_REG_OFFSET         0x00A8
#define MEMIF_SRAM_CFG_DONE_REG_OFFSET          0x00AA
#define MEMIF_SDRAM_ARB_G0_S01_REG_OFFSET       0x0040
#define MEMIF_SDRAM_ARB_G0_S23_REG_OFFSET       0x0042
#define MEMIF_SDRAM_ARB_G1_S01_REG_OFFSET       0x0044
#define MEMIF_SDRAM_ARB_G1_S23_REG_OFFSET       0x0046
#define MEMIF_SDRAM_ARB_G2_S01_REG_OFFSET       0x0048
#define MEMIF_SDRAM_ARB_G2_S23_REG_OFFSET       0x004A
#define MEMIF_SDRAM_ARB_G3_S01_REG_OFFSET       0x004C
#define MEMIF_SDRAM_ARB_G3_S23_REG_OFFSET       0x004E
#define MEMIF_SDRAM_ARB_G4_S01_REG_OFFSET       0x0050
#define MEMIF_SDRAM_ARB_G4_S23_REG_OFFSET       0x0052
#define MEMIF_SDRAM_ARB_G5_S01_REG_OFFSET       0x0054
#define MEMIF_SDRAM_ARB_G5_S23_REG_OFFSET       0x0056
#define MEMIF_SDRAM_ARB_G6_S01_REG_OFFSET       0x0058
#define MEMIF_SDRAM_ARB_G6_S23_REG_OFFSET       0x005A
#define MEMIF_SDRAM_ARB_G7_S01_REG_OFFSET       0x005C
#define MEMIF_SDRAM_ARB_G7_S23_REG_OFFSET       0x005E
#define MEMIF_SDRAM_ARB_VALID_REG_OFFSET        0x0060
#define MEMIF_SRAM_ARB_S01_REG_OFFSET           0x0080
#define MEMIF_SRAM_ARB_S23_REG_OFFSET           0x0082
#define MEMIF_SRAM_ARB_S45_REG_OFFSET           0x0084
#define MEMIF_SRAM_ARB_VALID_REG_OFFSET         0x0086
#define MEMIF_DLL0_MASTER_REG_OFFSET            0x0120
#define MEMIF_DLL0_SLAVE_REG_OFFSET             0x0122
#define MEMIF_DLL0_TEST_REG_OFFSET              0x0124
#define MEMIF_DLL1_MASTER_REG_OFFSET            0x0126
#define MEMIF_DLL1_SLAVE_REG_OFFSET             0x0128
#define MEMIF_DLL1_TEST_REG_OFFSET              0x012A
#define MEMIF_DLL2_MASTER_REG_OFFSET            0x012C
#define MEMIF_DLL2_SLAVE_REG_OFFSET             0x012E
#define MEMIF_DLL2_TEST_REG_OFFSET              0x0130
#define MEMIF_DLL3_MASTER_REG_OFFSET            0x0132
#define MEMIF_DLL3_SLAVE_REG_OFFSET             0x0134
#define MEMIF_DLL3_TEST_REG_OFFSET              0x0136
#define MEMIF_DLL_UPDATE_REG_OFFSET             0x0138

/* configuration register values */
#define MEMIF_SDRAM_SETUP_REG_SIZE_SHIFT        0
#define MEMIF_SDRAM_SETUP_REG_16B_SHIFT         2
#define MEMIF_SDRAM_SETUP_REG_RW_SHIFT          3
#define MEMIF_SDRAM_SETUP_REG_WR_SHIFT          7
#define MEMIF_SDRAM_SETUP_REG_ODT_SHIFT         11
#define MEMIF_SDRAM_SETUP_REG_CD_SHIFT          13
#define MEMIF_SDRAM_SETUP_REG_SIZE_0            (0 << MEMIF_SDRAM_SETUP_REG_SIZE_SHIFT)
#define MEMIF_SDRAM_SETUP_REG_SIZE_1            (1 << MEMIF_SDRAM_SETUP_REG_SIZE_SHIFT)
#define MEMIF_SDRAM_SETUP_REG_SIZE_2            (2 << MEMIF_SDRAM_SETUP_REG_SIZE_SHIFT)
#define MEMIF_SDRAM_SETUP_REG_32BIT_MODE        (0 << MEMIF_SDRAM_SETUP_REG_16B_SHIFT)       
#define MEMIF_SDRAM_SETUP_REG_16BIT_MODE        (1 << MEMIF_SDRAM_SETUP_REG_16B_SHIFT)
#define MEMIF_SDRAM_SETUP_REG_RW_GAP_4          (4 << MEMIF_SDRAM_SETUP_REG_RW_SHIFT)
#define MEMIF_SDRAM_SETUP_REG_WR_GAP_7          (7 << MEMIF_SDRAM_SETUP_REG_WR_SHIFT)
#define MEMIF_SDRAM_SETUP_ODT_OFF               (0 << MEMIF_SDRAM_SETUP_REG_ODT_SHIFT)
#define MEMIF_SDRAM_SETUP_ODT_75_OHM            (1 << MEMIF_SDRAM_SETUP_REG_ODT_SHIFT)
#define MEMIF_SDRAM_SETUP_ODT_150_OHM           (2 << MEMIF_SDRAM_SETUP_REG_ODT_SHIFT)
#define MEMIF_SDRAM_SETUP_ODT_50_OHM            (3 << MEMIF_SDRAM_SETUP_REG_ODT_SHIFT)
#define MEMIF_SDRAM_SETUP_CAPT_SIM              (1 << MEMIF_SDRAM_SETUP_REG_CD_SHIFT)
#define MEMIF_SDRAM_SETUP_CAPT_HW               (2 << MEMIF_SDRAM_SETUP_REG_CD_SHIFT)
#define MEMIF_SDRAM_REFRESH                     0x05DC
#define MEMIF_SDRAM_MRS                         0x0442
#define MEMIF_SDRAM_EMRS_ODT_OFF                0x0000
#define MEMIF_SDRAM_EMRS_ODT_75_OHM             0x0004
#define MEMIF_SDRAM_EMRS_ODT_150_OHM            0x0040
#define MEMIF_SDRAM_EMRS_ODT_50_OHM             0x0044
#define MEMIF_CFG_DONE                          0x0001
#define MEMIF_SDRAM_ARB_PA_AHB_LWR_SHIFT        0
#define MEMIF_SDRAM_ARB_OP_DET_LWR_SHIFT        1
#define MEMIF_SDRAM_ARB_WR_RD_LWR_SHIFT         2
#define MEMIF_SDRAM_ARB_CSP_LWR_SHIFT           3
#define MEMIF_SDRAM_ARB_PA_AHB_UPR_SHIFT        8
#define MEMIF_SDRAM_ARB_OP_DET_UPR_SHIFT        9
#define MEMIF_SDRAM_ARB_WR_RD_UPR_SHIFT         10
#define MEMIF_SDRAM_ARB_CSP_UPR_SHIFT           11
#define MEMIF_SDRAM_ARB_LWR_PA                  (1 << MEMIF_SDRAM_ARB_PA_AHB_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_AHB                 (0 << MEMIF_SDRAM_ARB_PA_AHB_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_OP                  (1 << MEMIF_SDRAM_ARB_OP_DET_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_DET                 (0 << MEMIF_SDRAM_ARB_OP_DET_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_WR                  (1 << MEMIF_SDRAM_ARB_WR_RD_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_RD                  (0 << MEMIF_SDRAM_ARB_WR_RD_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_CSP_0               (0 << MEMIF_SDRAM_ARB_CSP_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_CSP_1               (1 << MEMIF_SDRAM_ARB_CSP_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_CSP_2               (2 << MEMIF_SDRAM_ARB_CSP_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_LWR_CSP_3               (3 << MEMIF_SDRAM_ARB_CSP_LWR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_PA                  (1 << MEMIF_SDRAM_ARB_PA_AHB_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_AHB                 (0 << MEMIF_SDRAM_ARB_PA_AHB_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_OP                  (1 << MEMIF_SDRAM_ARB_OP_DET_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_DET                 (0 << MEMIF_SDRAM_ARB_OP_DET_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_WR                  (1 << MEMIF_SDRAM_ARB_WR_RD_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_RD                  (0 << MEMIF_SDRAM_ARB_WR_RD_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_CSP_0               (0 << MEMIF_SDRAM_ARB_CSP_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_CSP_1               (1 << MEMIF_SDRAM_ARB_CSP_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_CSP_2               (2 << MEMIF_SDRAM_ARB_CSP_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_UPR_CSP_3               (3 << MEMIF_SDRAM_ARB_CSP_UPR_SHIFT)
#define MEMIF_SDRAM_ARB_1_VALID_GRP             0x0001
#define MEMIF_SDRAM_ARB_2_VALID_GRP             0x0003
#define MEMIF_SDRAM_ARB_3_VALID_GRP             0x0007
#define MEMIF_SDRAM_ARB_4_VALID_GRP             0x000F
#define MEMIF_SDRAM_ARB_5_VALID_GRP             0x001F
#define MEMIF_SDRAM_ARB_6_VALID_GRP             0x003F
#define MEMIF_SDRAM_ARB_7_VALID_GRP             0x007F
#define MEMIF_SDRAM_ARB_8_VALID_GRP             0x00FF
#define MEMIF_DLL0_SLAVE_DELAY                  0x4803
#define MEMIF_DLL1_SLAVE_DELAY                  0x4A03
#define MEMIF_DLL2_SLAVE_DELAY                  0x4A03
#define MEMIF_DLL3_SLAVE_DELAY                  0x4A03
#define MEMIF_DLL_UPDATE                        0x0001

#ifdef HARDWARE_BUILD
#define MEMIF_SDRAM_SETUP_CAPT_DELAY  MEMIF_SDRAM_SETUP_CAPT_HW
#else
#define MEMIF_SDRAM_SETUP_CAPT_DELAY  MEMIF_SDRAM_SETUP_CAPT_SIM
#endif

#endif /* PC302__MEMIF_H */
