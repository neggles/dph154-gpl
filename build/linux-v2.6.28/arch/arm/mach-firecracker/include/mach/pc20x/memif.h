
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
 *              Memory Interface Register Definitions
 *
 *****************************************************************************/ 
#ifndef __FIRECRACKER_PC20X_MEMIF_H
#define __FIRECRACKER_PC20X_MEMIF_H

#define MEMIF_PASD_BUFFER0_SETUP_REG_OFFSET                (0x0000 * 2)
#define MEMIF_PASD_BUFFER0_ADDRESS_SETUP_REG_OFFSET        (0x0001 * 2)
#define MEMIF_PASD_BUFFER0_FIFO_SETUP_REG_OFFSET           (0x0002 * 2)
#define MEMIF_PASD_BUFFER1_SETUP_REG_OFFSET                (0x0004 * 2)
#define MEMIF_PASD_BUFFER1_ADDRESS_SETUP_REG_OFFSET        (0x0005 * 2)
#define MEMIF_PASD_BUFFER1_FIFO_SETUP_REG_OFFSET           (0x0006 * 2)
#define MEMIF_PASD_BUFFER2_SETUP_REG_OFFSET                (0x0008 * 2)
#define MEMIF_PASD_BUFFER2_ADDRESS_SETUP_REG_OFFSET        (0x0009 * 2)
#define MEMIF_PASD_BUFFER2_FIFO_SETUP_REG_OFFSET           (0x000A * 2)
#define MEMIF_PASD_BUFFER3_SETUP_REG_OFFSET                (0x000C * 2)
#define MEMIF_PASD_BUFFER3_ADDRESS_SETUP_REG_OFFSET        (0x000D * 2)
#define MEMIF_PASD_BUFFER3_FIFO_SETUP_REG_OFFSET           (0x000E * 2)
#define MEMIF_PAS_BUFFER0_SETUP_REG_OFFSET                 (0x0010 * 2)
#define MEMIF_PAS_BUFFER0_ADDRESS_SETUP_REG_OFFSET         (0x0011 * 2)
#define MEMIF_PAS_BUFFER0_FIFO_SETUP_REG_OFFSET            (0x0012 * 2)
#define MEMIF_PAS_BUFFER1_SETUP_REG_OFFSET                 (0x0014 * 2)
#define MEMIF_PAS_BUFFER1_ADDRESS_SETUP_REG_OFFSET         (0x0015 * 2)
#define MEMIF_PAS_BUFFER1_FIFO_SETUP_REG_OFFSET            (0x0016 * 2)
#define MEMIF_SDRAM_ARB_GROUP0_SLOTA_CONFIG_REG_OFFSET     (0x0020 * 2)
#define MEMIF_SDRAM_ARB_GROUP0_SLOTB_CONFIG_REG_OFFSET     (0x0021 * 2)
#define MEMIF_SDRAM_ARB_GROUP1_SLOTA_CONFIG_REG_OFFSET     (0x0022 * 2)
#define MEMIF_SDRAM_ARB_GROUP1_SLOTB_CONFIG_REG_OFFSET     (0x0023 * 2)
#define MEMIF_SDRAM_ARB_GROUP2_SLOTA_CONFIG_REG_OFFSET     (0x0024 * 2)
#define MEMIF_SDRAM_ARB_GROUP2_SLOTB_CONFIG_REG_OFFSET     (0x0025 * 2)
#define MEMIF_SDRAM_ARB_GROUP3_SLOTA_CONFIG_REG_OFFSET     (0x0026 * 2)
#define MEMIF_SDRAM_ARB_GROUP3_SLOTB_CONFIG_REG_OFFSET     (0x0027 * 2)
#define MEMIF_SDRAM_ARB_GROUP4_SLOTA_CONFIG_REG_OFFSET     (0x0028 * 2)
#define MEMIF_SDRAM_ARB_GROUP4_SLOTB_CONFIG_REG_OFFSET     (0x0029 * 2)
#define MEMIF_SDRAM_ARB_GROUP5_SLOTA_CONFIG_REG_OFFSET     (0x002A * 2)
#define MEMIF_SDRAM_ARB_GROUP5_SLOTB_CONFIG_REG_OFFSET     (0x002B * 2)
#define MEMIF_SDRAM_ARB_GROUP6_SLOTA_CONFIG_REG_OFFSET     (0x002C * 2)
#define MEMIF_SDRAM_ARB_GROUP6_SLOTB_CONFIG_REG_OFFSET     (0x002D * 2)
#define MEMIF_SDRAM_ARB_GROUP7_SLOTA_CONFIG_REG_OFFSET     (0x002E * 2)
#define MEMIF_SDRAM_ARB_GROUP7_SLOTB_CONFIG_REG_OFFSET     (0x002F * 2)
#define MEMIF_SDRAM_ARB_VALID_GROUPS_CONFIG_REG_OFFSET     (0x0030 * 2)
#define MEMIF_SRAM_ARB_SLOTA_CONFIG_REG_OFFSET             (0x0040 * 2)
#define MEMIF_SRAM_ARB_SLOTB_CONFIG_REG_OFFSET             (0x0041 * 2)
#define MEMIF_SRAM_ARB_SLOTC_CONFIG_REG_OFFSET             (0x0042 * 2)
#define MEMIF_SRAM_ARB_VALID_SLOTS_CONFIG_REG_OFFSET       (0x0043 * 2)
#define MEMIF_ARB_UPDATE_REG_OFFSET                        (0x004F * 2)
#define MEMIF_SDRAM_SETUP_REG_OFFSET                       (0x0050 * 2)
#define MEMIF_SDRAM_REFRESH_RATE_REG_OFFSET                (0x0051 * 2)
#define MEMIF_SDRAM_MRS_REG_OFFSET                         (0x0052 * 2)
#define MEMIF_SDRAM_ERS_REG_OFFSET                         (0x0053 * 2)
#define MEMIF_SDRAM_SETUP_COMPLETE_REG                     (0x0054 * 2)
#define MEMIF_SRAM_SETUP_COMPLETE_REG                      (0x0055 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT0_MASTERS03_REG_OFFSET  (0x0060 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT0_MASTERS46_REG_OFFSET  (0x0061 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT1_MASTERS03_REG_OFFSET  (0x0062 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT1_MASTERS46_REG_OFFSET  (0x0063 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT2_MASTERS03_REG_OFFSET  (0x0064 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT2_MASTERS46_REG_OFFSET  (0x0065 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT3_MASTERS03_REG_OFFSET  (0x0066 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT3_MASTERS46_REG_OFFSET  (0x0067 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT4_MASTERS03_REG_OFFSET  (0x0068 * 2)
#define MEMIF_AHB_ARB_PRIORITY_PORT4_MASTERS46_REG_OFFSET  (0x0069 * 2)
#define MEMIF_AHB_ARBITRATION_SCHEME_REG_OFFSET            (0x006A * 2)
#define MEMIF_TEST_MODE_ENABLE_REG_OFFSET                  (0x0070 * 2)
#define MEMIF_TEST_MODE_WRITE_DATA_REG_OFFSET              (0x0071 * 2)
#define MEMIF_TEST_MODE_ADDRESS_REG_OFFSET                 (0x0072 * 2)
#define MEMIF_TEST_MODE_ADDRESS2_REG_OFFSET                (0x0073 * 2)
#define MEMIF_TEST_MODE_CONTROL_REG_OFFSET                 (0x0074 * 2)
#define MEMIF_TEST_MODE_READ_DATA_REG_OFFSET               (0x0075 * 2)
#define MEMIF_PA_BUFFER_STATUS_REG_OFFSET                  (0x0080 * 2)

#define SDRAM_REFRESH_COUNT                                 0x0600
#define SDRAM_WRGAP7                                       (0x7 << 7)
#define SDRAM_RWGAP4                                       (0x4 << 3)
#define SDRAM_WIDTH32                                       0x0000
#define SDRAM_WIDTH16                                      (0x1 << 2)
#define SDRAM_SIZE14_R10C                                   0x0000
#define SDRAM_SIZE13_R10C                                   0x0001
#define SDRAM_SIZE13_R9C                                    0x0010
#define SDRAM_CAPTURE_DELAY	                                0x2000

#endif /* __FIRECRACKER_PC20X_MEMIF_H */
