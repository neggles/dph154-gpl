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

#ifndef PC302_MEM_SHD_H
#define PC302_MEM_SHD_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Includes ---------------------------------------------------------------- */
#include "pa.h"

/* Constants --------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define ADDR_PA_BUF0_SETUP                  (0x0000)
#define ADDR_PA_BUF0_ADDR                   (0x0001)
#define ADDR_PA_BUF1_SETUP                  (0x0002)
#define ADDR_PA_BUF1_ADDR                   (0x0003)
#define ADDR_PA_BUF2_SETUP                  (0x0004)
#define ADDR_PA_BUF2_ADDR                   (0x0005)
#define ADDR_PA_BUF3_SETUP                  (0x0006)
#define ADDR_PA_BUF3_ADDR                   (0x0007)
#define ADDR_PA_BUF4_SETUP                  (0x0008)
#define ADDR_PA_BUF4_ADDR                   (0x0009)
#define ADDR_PA_BUF5_SETUP                  (0x000A)
#define ADDR_PA_BUF5_ADDR                   (0x000B)
#define ADDR_PA_BUF6_SETUP                  (0x000C)
#define ADDR_PA_BUF6_ADDR                   (0x000D)
#define ADDR_PA_BUF7_SETUP                  (0x000E)
#define ADDR_PA_BUF7_ADDR                   (0x000F)
#define ADDR_PA_BUF8_SETUP                  (0x0010)
#define ADDR_PA_BUF8_ADDR                   (0x0011)
#define ADDR_PA_BUF9_SETUP                  (0x0012)
#define ADDR_PA_BUF9_ADDR                   (0x0013)
#define ADDR_SDRAM_ARB_G0_S0_S1             (0x0020)
#define ADDR_SDRAM_ARB_G0_S2_S3             (0x0021)
#define ADDR_SDRAM_ARB_G1_S0_S1             (0x0022)
#define ADDR_SDRAM_ARB_G1_S2_S3             (0x0023)
#define ADDR_SDRAM_ARB_G2_S0_S1             (0x0024)
#define ADDR_SDRAM_ARB_G2_S2_S3             (0x0025)
#define ADDR_SDRAM_ARB_G3_S0_S1             (0x0026)
#define ADDR_SDRAM_ARB_G3_S2_S3             (0x0027)
#define ADDR_SDRAM_ARB_G4_S0_S1             (0x0028)
#define ADDR_SDRAM_ARB_G4_S2_S3             (0x0029)
#define ADDR_SDRAM_ARB_G5_S0_S1             (0x002A)
#define ADDR_SDRAM_ARB_G5_S2_S3             (0x002B)
#define ADDR_SDRAM_ARB_G6_S0_S1             (0x002C)
#define ADDR_SDRAM_ARB_G6_S2_S3             (0x002D)
#define ADDR_SDRAM_ARB_G7_S0_S1             (0x002E)
#define ADDR_SDRAM_ARB_G7_S2_S3             (0x002F)
#define ADDR_SDRAM_VALID_GROUPS             (0x0030)
#define ADDR_SRAM_ARB_S0_S1                 (0x0040)
#define ADDR_SRAM_ARB_S2_S3                 (0x0041)
#define ADDR_SRAM_ARB_S4_S5                 (0x0042)
#define ADDR_SRAM_VALID_SLOTS               (0x0043)
#define ADDR_ARB_UPDATE                     (0x004F)
#define ADDR_SDRAM_SETUP                    (0x0050)
#define ADDR_SDRAM_REFRESH                  (0x0051)
#define ADDR_SDRAM_MRS                      (0x0052)
#define ADDR_SDRAM_EMRS                     (0x0053)
#define ADDR_SDRAM_EMRS2                    (0x0054)
#define ADDR_SDRAM_EMRS3                    (0x0055)
#define ADDR_SDRAM_ODT_SETUP                (0x0056)
#define ADDR_SDRAM_CFG_DONE                 (0x0057)
#define ADDR_SDRAM_AXI_CONFIG               (0x0058)
#define ADDR_SDRAM_DEBUG                    (0x0059)
#define ADDR_SDRAM_STATUS                   (0x005A)
#define ADDR_PHY_TEST                       (0x0060)
#define ADDR_PHY_CONFIG                     (0x0061)
#define ADDR_PHY_LOCAL_ODT_CONFIG           (0x0062)
#define ADDR_PHY_RDC_FIFO_RST_ERR_CNT       (0x0063)
#define ADDR_PHY_WR_SLAVE                   (0x0064)
#define ADDR_PHY_RD_SLAVE                   (0x0065)
#define ADDR_PHY_DEBUG_WR_DLL0              (0x0066)
#define ADDR_PHY_DEBUG_RC_DLL0              (0x0068)
#define ADDR_PHY_DEBUG_RC_DLL1              (0x0069)
#define ADDR_PHY_IO_CELL_CONFIG             (0x006A)
#define ADDR_PHY_DEBUG_STATUS_BC            (0x0070)
#define ADDR_PHY_DEBUG_STATUS_RC_DLL_0      (0x0071)
#define ADDR_PHY_DEBUG_STATUS_RC_DLL_1      (0x0072)
#define ADDR_PHY_DEBUG_STATUS_MASTER_DLL_0  (0x0073)
#define ADDR_PHY_DEBUG_STATUS_OF_IN_DELAY_0 (0x0075)
#define ADDR_PHY_DEBUG_STATUS_OF_OUT_DELAY_0    (0x0076)
#define ADDR_PA_FIFO_STATUS_LWR             (0x0080)
#define ADDR_PA_FIFO_STATUS_UPR             (0x0081)
#define ADDR_PA_ADDR_ERR_LWR                (0x0082)
#define ADDR_PA_ADDR_ERR_UPR                (0x0083)
#define ADDR_PA_ADDR_ERR_MASK_LWR           (0x0084)
#define ADDR_PA_ADDR_ERR_MASK_UPR           (0x0085)

#define CBFM_SLEEPREG_ADDR                  (0xA060)
#define CBFM_RUNREG_ADDR                    (0xA061)
#define CBFM_ERRREG_ADDR                    (0xA062)
#define CBFM_RESETREG_ADDR                  (0xA063)
#define CBFM_IDREG_ADDR                     (0xA064)
#define CBFM_RSTRUNMASK_ADDR                (0xA065)
#define CBFM_BISTEN_ADDR                    (0xA066)
#define CBFM_BISTDATA_ADDR                  (0xA067)
#define CBFM_TM_ADDR                        (0xA068)

/*****************************************************************************/
/* Register Values                                                           */
/*****************************************************************************/

#define DDR2_ROW_13_COL_9               (0x2)
#define DDR2_ROW_13_COL_10              (0x1)
#define DDR2_ROW_14_COL_10              (0x0)
#define DDR2_8_BANKS                    (0x1)
#define DDR2_4_BANKS                    (0x0)
#define DDR2_BRC_ADDR                   (0x0)
#define DDR2_RBC_ADDR                   (0x1)
#define DDR2_RW_GAP                     (0x3)
#define DDR2_WR_GAP                     (0xa)
#define DDR2_CAS_4                      (0x4)
#define DDR2_CAS_5                      (0x5)
#define DDR2_CAS_6                      (0x6)
#define DDR2_CAS_7                      (0x7)
#define DDR2_REFRESH_TIME               (0x07d0)
#define DDR2_BURST_SEQUENTIAL           (0x0000)
#define DDR2_WR                         (0x05)
#define DDR2_DQSN_DISABLE               (0x1)

#define SDRAM_ARB_AXI_SLOT              (0)
#define SDRAM_ARB_PA_SLOT               (1)
#define SDRAM_ARB_DET_SLOT              (0)
#define SDRAM_ARB_OPP_SLOT              (1)
#define SDRAM_ARB_RD_SLOT               (0)
#define SDRAM_ARB_WR_SLOT               (1)
#define SDRAM_ARB_CSP_1_SLOT            (0)
#define SDRAM_ARB_CSP_2_SLOT            (1)
#define SDRAM_ARB_CSP_4_SLOT            (2)
#define SDRAM_ARB_CSP_8_SLOT            (3)

#define SDRAM_ARB_1_VALID_SLOT          (1)
#define SDRAM_ARB_2_VALID_SLOT          (3)
#define SDRAM_ARB_3_VALID_SLOT          (7)
#define SDRAM_ARB_4_VALID_SLOT          (15)
#define SDRAM_ARB_5_VALID_SLOT          (31)
#define SDRAM_ARB_6_VALID_SLOT          (63)
#define SDRAM_ARB_7_VALID_SLOT          (127)
#define SDRAM_ARB_8_VALID_SLOT          (255)

#define PHY_CONFIG_BL_4                 (2)
#define PHY_CONFIG_BL_8                 (3)
#define PHY_CONFIG_RDC_WE_TO_RE_2       (2)
#define PHY_CONFIG_FIXED_RE             (1)

#define ADDR_SDRAM_STATUS_INIT_DONE_BIT     (0)
#define ADDR_SDRAM_STATUS_IN_RESET_BIT      (1)
#define ADDR_SDRAM_STATUS_IN_STB_CLK_BIT    (2)
#define ADDR_SDRAM_STATUS_IN_INIT_BIT       (3)
#define ADDR_SDRAM_STATUS_IN_ARB_BIT        (4)
#define ADDR_SDRAM_STATUS_IN_REF_BIT        (5)
#define ADDR_SDRAM_STATUS_IN_UPDATE_BIT     (6)
#define ADDR_SDRAM_STATUS_IN_RW_GAP_BIT     (7)
#define ADDR_SDRAM_STATUS_IN_WR_GAP_BIT     (8)

#define SDRAM_SETUP_SIZE_IDX            (0)
#define SDRAM_SETUP_BANK_IDX            (2)
#define SDRAM_SETUP_RW_GAP_IDX          (3)
#define SDRAM_SETUP_WR_GAP_IDX          (7)
#define SDRAM_SETUP_CAS_IDX             (11)

#define SDRAM_AXI_SETUP_RBC_IDX         (0)
#define SDRAM_AXI_SETUP_SIZE_IDX        (1)
#define SDRAM_AXI_SETUP_BANK_IDX        (3)

#define SDRAM_ARB_S0_AP_IDX             (0)
#define SDRAM_ARB_S0_APOD_IDX           (1)
#define SDRAM_ARB_S0_RW_IDX             (2)
#define SDRAM_ARB_S0_RWOD_IDX           (3)
#define SDRAM_ARB_S0_CSP_IDX            (4)

#define SDRAM_ARB_S1_AP_IDX             (8)
#define SDRAM_ARB_S1_APOD_IDX           (9)
#define SDRAM_ARB_S1_RW_IDX             (10)
#define SDRAM_ARB_S1_RWOD_IDX           (11)
#define SDRAM_ARB_S1_CSP_IDX            (12)

#define SDRAM_ARB_S2_AP_IDX             (0)
#define SDRAM_ARB_S2_APOD_IDX           (1)
#define SDRAM_ARB_S2_RW_IDX             (2)
#define SDRAM_ARB_S2_RWOD_IDX           (3)
#define SDRAM_ARB_S2_CSP_IDX            (4)

#define SDRAM_ARB_S3_AP_IDX             (8)
#define SDRAM_ARB_S3_APOD_IDX           (9)
#define SDRAM_ARB_S3_RW_IDX             (10)
#define SDRAM_ARB_S3_RWOD_IDX           (11)
#define SDRAM_ARB_S3_CSP_IDX            (12)

#define PHY_CONFIG_FIRST_RD_IDX         (0)
#define PHY_CONFIG_FIRST_WR_IDX         (4)
#define PHY_CONFIG_BL_IDX               (8)
#define PHY_CONFIG_RDC_WE_TO_RE_IDX     (11)
#define PHY_CONFIG_FIXED_RE_IDX         (13)

#define MRS_PHY_CONFIG_BL_IDX           (0)
#define MRS_DDR2_BURST_SEQUENTIAL_IDX   (3)
#define MRS_DDR2_CAS_IDX                (4)
#define MRS_DDR2_WR_IDX                 (9)

#define EMRS_DRIVE_STRENGTH_IDX         (1)
#define EMRS_ODT_LOW_BIT_IDX            (2)
#define EMRS_ODT_HIGH_BIT_IDX           (6)
#define EMRS_DQSN_DISABLE_IDX           (10)

/*****************************************************************************/
/* Used to create a 'load file' for the pA                                   */
/*****************************************************************************/

#define LF_PA_AEID_MEMIF                (PA_AEID_MEMIF | PA_CONFIG_AEID)

#define LF_CBFM_SLEEPREG_ADDR           (CBFM_SLEEPREG_ADDR | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_ARB_G0_S0_S1      (ADDR_SDRAM_ARB_G0_S0_S1 | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_ARB_G0_S2_S3      (ADDR_SDRAM_ARB_G0_S2_S3 | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_ARB_G1_S0_S1      (ADDR_SDRAM_ARB_G1_S0_S1 | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_ARB_G1_S2_S3      (ADDR_SDRAM_ARB_G1_S2_S3 | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_VALID_GROUPS      (ADDR_SDRAM_VALID_GROUPS | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_SETUP             (ADDR_SDRAM_SETUP | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_REFRESH           (ADDR_SDRAM_REFRESH | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_ODT_SETUP         (ADDR_SDRAM_ODT_SETUP | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_AXI_CONFIG        (ADDR_SDRAM_AXI_CONFIG | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_MRS               (ADDR_SDRAM_MRS | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_EMRS              (ADDR_SDRAM_EMRS | PA_CONFIG_ADDR)
#define LF_ADDR_PHY_CONFIG              (ADDR_PHY_CONFIG | PA_CONFIG_ADDR)
#define LF_ADDR_PHY_LOCAL_ODT_CONFIG    (ADDR_PHY_LOCAL_ODT_CONFIG | PA_CONFIG_ADDR)
#define LF_ADDR_PHY_RD_SLAVE            (ADDR_PHY_RD_SLAVE | PA_CONFIG_ADDR)
#define LF_ADDR_PHY_WR_SLAVE            (ADDR_PHY_WR_SLAVE | PA_CONFIG_ADDR)
#define LF_ADDR_PHY_IO_CELL_CONFIG      (ADDR_PHY_IO_CELL_CONFIG | PA_CONFIG_ADDR)
#define LF_ADDR_SDRAM_CFG_DONE          (ADDR_SDRAM_CFG_DONE | PA_CONFIG_ADDR)

#define ADDR_SDRAM_ARB_G0_S0_S1_DATA    (0x00000000 | PA_CONFIG_WRITE | \
                                        (SDRAM_ARB_AXI_SLOT     << SDRAM_ARB_S0_AP_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S0_APOD_IDX) | \
                                        (SDRAM_ARB_RD_SLOT      << SDRAM_ARB_S0_RW_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S0_RWOD_IDX) | \
                                        (SDRAM_ARB_CSP_2_SLOT   << SDRAM_ARB_S0_CSP_IDX)  | \
                                        (SDRAM_ARB_AXI_SLOT     << SDRAM_ARB_S1_AP_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S1_APOD_IDX) | \
                                        (SDRAM_ARB_RD_SLOT      << SDRAM_ARB_S1_RW_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S1_RWOD_IDX) | \
                                        (SDRAM_ARB_CSP_2_SLOT   << SDRAM_ARB_S1_CSP_IDX))

#define ADDR_SDRAM_ARB_G0_S2_S3_DATA    (0x00000000 | PA_CONFIG_WRITE | \
                                        (SDRAM_ARB_AXI_SLOT     << SDRAM_ARB_S2_AP_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S2_APOD_IDX) | \
                                        (SDRAM_ARB_RD_SLOT      << SDRAM_ARB_S2_RW_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S2_RWOD_IDX) | \
                                        (SDRAM_ARB_CSP_2_SLOT   << SDRAM_ARB_S2_CSP_IDX)  | \
                                        (SDRAM_ARB_AXI_SLOT     << SDRAM_ARB_S3_AP_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S3_APOD_IDX) | \
                                        (SDRAM_ARB_RD_SLOT      << SDRAM_ARB_S3_RW_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S3_RWOD_IDX) | \
                                        (SDRAM_ARB_CSP_2_SLOT   << SDRAM_ARB_S3_CSP_IDX))

#define ADDR_SDRAM_ARB_G1_S0_S1_DATA    (0x00000000 | PA_CONFIG_WRITE | \
                                        (SDRAM_ARB_AXI_SLOT     << SDRAM_ARB_S0_AP_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S0_APOD_IDX) | \
                                        (SDRAM_ARB_RD_SLOT      << SDRAM_ARB_S0_RW_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S0_RWOD_IDX) | \
                                        (SDRAM_ARB_CSP_2_SLOT   << SDRAM_ARB_S0_CSP_IDX)  | \
                                        (SDRAM_ARB_AXI_SLOT     << SDRAM_ARB_S1_AP_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S1_APOD_IDX) | \
                                        (SDRAM_ARB_RD_SLOT      << SDRAM_ARB_S1_RW_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S1_RWOD_IDX) | \
                                        (SDRAM_ARB_CSP_2_SLOT   << SDRAM_ARB_S1_CSP_IDX))

#define ADDR_SDRAM_ARB_G1_S2_S3_DATA    (0x00000000 | PA_CONFIG_WRITE | \
                                        (SDRAM_ARB_AXI_SLOT     << SDRAM_ARB_S2_AP_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S2_APOD_IDX) | \
                                        (SDRAM_ARB_RD_SLOT      << SDRAM_ARB_S2_RW_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S2_RWOD_IDX) | \
                                        (SDRAM_ARB_CSP_2_SLOT   << SDRAM_ARB_S2_CSP_IDX)  | \
                                        (SDRAM_ARB_AXI_SLOT     << SDRAM_ARB_S3_AP_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S3_APOD_IDX) | \
                                        (SDRAM_ARB_RD_SLOT      << SDRAM_ARB_S3_RW_IDX)   | \
                                        (SDRAM_ARB_OPP_SLOT     << SDRAM_ARB_S3_RWOD_IDX) | \
                                        (SDRAM_ARB_CSP_2_SLOT   << SDRAM_ARB_S3_CSP_IDX))

#define ADDR_SDRAM_VALID_GROUPS_DATA    (0x00000000 | PA_CONFIG_WRITE | SDRAM_ARB_2_VALID_SLOT)

#define ADDR_SDRAM_SETUP_DATA           (0x00000000 | PA_CONFIG_WRITE | \
                                        (DDR2_ROW_13_COL_10 << SDRAM_SETUP_SIZE_IDX)     | \
                                        (DDR2_8_BANKS       << SDRAM_SETUP_BANK_IDX)     | \
                                        (DDR2_RW_GAP        << SDRAM_SETUP_RW_GAP_IDX)   | \
                                        (DDR2_WR_GAP        << SDRAM_SETUP_WR_GAP_IDX)   | \
                                        (DDR2_CAS_6         << SDRAM_SETUP_CAS_IDX))

#define ADDR_SDRAM_REFRESH_DATA         (0x00000000 | PA_CONFIG_WRITE | DDR2_REFRESH_TIME)

#define ADDR_SDRAM_AXI_CONFIG_DATA      (0x00000000 | PA_CONFIG_WRITE | \
                                        (DDR2_BRC_ADDR      << SDRAM_AXI_SETUP_RBC_IDX)   | \
                                        (DDR2_ROW_13_COL_10 << SDRAM_AXI_SETUP_SIZE_IDX)  | \
                                        (DDR2_8_BANKS       << SDRAM_AXI_SETUP_BANK_IDX))

#define ADDR_SDRAM_MRS_DATA             (0x00000000 | PA_CONFIG_WRITE | \
                                        (PHY_CONFIG_BL_4        << MRS_PHY_CONFIG_BL_IDX) | \
                                        (DDR2_BURST_SEQUENTIAL  << MRS_DDR2_BURST_SEQUENTIAL_IDX) | \
                                        (DDR2_CAS_6             << MRS_DDR2_CAS_IDX) | \
                                        (DDR2_WR                << MRS_DDR2_WR_IDX))

#define SDRAM_ODT_75_OHM_LO             (0x1)
#define SDRAM_ODT_75_OHM_HI             (0x0)
#define SDRAM_REDUCED_DRIVE             (0x1)
#define SDRAM_FULL_DRIVE                (0x0)

#define ADDR_SDRAM_EMRS_DATA            (0x00000000 | PA_CONFIG_WRITE | \
                                        (SDRAM_ODT_75_OHM_LO        << EMRS_ODT_LOW_BIT_IDX) | \
                                        (SDRAM_ODT_75_OHM_HI        << EMRS_ODT_HIGH_BIT_IDX) | \
                                        (SDRAM_REDUCED_DRIVE        << EMRS_DRIVE_STRENGTH_IDX) | \
                                        (DDR2_DQSN_DISABLE << EMRS_DQSN_DISABLE_IDX))

#define ADDR_PHY_CONFIG_DATA            (0x00000000 | PA_CONFIG_WRITE | \
                                        (DDR2_CAS_6                 << PHY_CONFIG_FIRST_RD_IDX) | \
                                        ((DDR2_CAS_6 - 1)           << PHY_CONFIG_FIRST_WR_IDX) | \
                                        (PHY_CONFIG_BL_4            << PHY_CONFIG_BL_IDX)       | \
                                        (PHY_CONFIG_RDC_WE_TO_RE_2  << PHY_CONFIG_RDC_WE_TO_RE_IDX) | \
                                        (PHY_CONFIG_FIXED_RE        << PHY_CONFIG_FIXED_RE_IDX))

#define SDRAM_ODT_ENABLE_IDX            (0)
#define SDRAM_ODT_ON_DELAY_IDX          (1)
#define SDRAM_ODT_ON_DURN_IDX           (4)
#define SDRAM_ODT_ENABLE                (0x1)
#define SDRAM_ODT_ON_DURN               (0x2)

#define ADDR_SDRAM_ODT_DATA             (0x00000000 | PA_CONFIG_WRITE | \
                                        (SDRAM_ODT_ENABLE   << SDRAM_ODT_ENABLE_IDX) | \
                                        ((DDR2_CAS_6 - 4)   << SDRAM_ODT_ON_DELAY_IDX) | \
                                        (SDRAM_ODT_ON_DURN  << SDRAM_ODT_ON_DURN_IDX))

#define PHY_LOCAL_ODT_75_OHM            (0x1)
#define PHY_LOCAL_ODT_OFF               (0x0)
#define PHY_LOCAL_ODT_READ_IDX          (0)
#define PHY_LOCAL_ODT_WRITE_IDX         (2)
#define PHY_LOCAL_ODT_IDLE_IDX          (4)

#define ADDR_PHY_LOCAL_ODT_CONFIG_DATA  (0x00000000 | PA_CONFIG_WRITE | \
                                        (PHY_LOCAL_ODT_75_OHM   << PHY_LOCAL_ODT_READ_IDX) | \
                                        (PHY_LOCAL_ODT_OFF      << PHY_LOCAL_ODT_WRITE_IDX) | \
                                        (PHY_LOCAL_ODT_OFF      << PHY_LOCAL_ODT_IDLE_IDX))

#define PHY_RD0_DLL_SLAVE_DELAY         (0x30)
#define PHY_RD1_DLL_SLAVE_DELAY         (0x30)
#define PHY_WR_DLL_SLAVE_DELAY          (0x40)

#define PHY_RD0_DLL_SLAVE_IDX           (0)
#define PHY_RD1_DLL_SLAVE_IDX           (8)
#define PHY_WR_DLL_SLAVE_IDX            (0)


#define ADDR_PHY_RD_SLAVE_DATA          (0x00000000 | PA_CONFIG_WRITE | \
                                        (PHY_RD0_DLL_SLAVE_DELAY << PHY_RD0_DLL_SLAVE_IDX) | \
                                        (PHY_RD1_DLL_SLAVE_DELAY << PHY_RD1_DLL_SLAVE_IDX))

#define ADDR_PHY_WR_SLAVE_DATA          (0x00000000 | PA_CONFIG_WRITE | \
                                        (PHY_WR_DLL_SLAVE_DELAY << PHY_WR_DLL_SLAVE_IDX))


#define PHY_IO_CELL_CONFIG_FIFO_WE_IN_ODT_IDX   (4)

#define PHY_CTRL_DRV_STRENGTH_HALF              (1)
#define PHY_DATA_DRV_STRENGTH_HALF              (1)
#define PHY_CLK_DRV_STRENGTH_HALF               (1)
#define PHY_FIFO_WE_DRV_STRENGTH_HALF           (1)

#define PHY_CTRL_DRV_STRENGTH_HALF_IDX          (0)
#define PHY_DATA_DRV_STRENGTH_HALF_IDX          (1)
#define PHY_CLK_DRV_STRENGTH_HALF_IDX           (2)
#define PHY_FIFO_WE_DRV_STRENGTH_HALF_IDX       (3)


#define ADDR_PHY_IO_CELL_CONFIG_DATA    (0x00000000 | PA_CONFIG_WRITE | \
                                        (PHY_LOCAL_ODT_75_OHM           << PHY_IO_CELL_CONFIG_FIFO_WE_IN_ODT_IDX) | \
                                        (PHY_FIFO_WE_DRV_STRENGTH_HALF  << PHY_FIFO_WE_DRV_STRENGTH_HALF_IDX) | \
                                        (PHY_CLK_DRV_STRENGTH_HALF      << PHY_CLK_DRV_STRENGTH_HALF_IDX) | \
                                        (PHY_DATA_DRV_STRENGTH_HALF     << PHY_DATA_DRV_STRENGTH_HALF_IDX) | \
                                        (PHY_CTRL_DRV_STRENGTH_HALF     << PHY_CTRL_DRV_STRENGTH_HALF_IDX))

#define ADDR_SDRAM_CFG_DONE_DATA        (0x0001 | PA_CONFIG_WRITE)

#define PA_WRITE_ZERO_DATA              (PA_CONFIG_WRITE)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_MEM_SHD_H */
