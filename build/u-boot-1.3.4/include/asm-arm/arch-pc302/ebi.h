/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file ebi.h
* \brief Definitions for the PC302 EBI Block.
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_EBI_H
#define PC302_EBI_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define SDRAM_CON_REG_OFFSET                    (0x00)
#define SDRAM_TIM0_REG_OFFSET                   (0x04)
#define SDRAM_TIM1_REG_OFFSET                   (0x08)
#define SDRAM_CTL_REG_OFFSET                    (0x0C)
#define SDRAM_REFRESH_REG_OFFSET                (0x10)
#define SCHIP_SEL_REGION0_LOW_REG_OFFSET        (0x14)
#define SCHIP_SEL_REGION1_LOW_REG_OFFSET        (0x18)
#define SCHIP_SEL_REGION2_LOW_REG_OFFSET        (0x1C)
#define SCHIP_SEL_REGION3_LOW_REG_OFFSET        (0x20)
#define SCHIP_SEL_REGION4_LOW_REG_OFFSET        (0x24)
#define SCHIP_SEL_REGION5_LOW_REG_OFFSET        (0x28)
#define SCHIP_SEL_REGION6_LOW_REG_OFFSET        (0x2C)
#define SCHIP_SEL_REGION7_LOW_REG_OFFSET        (0x30)
#define SCHIP_SEL_REGION0_HGH_REG_OFFSET        (0x34)
#define SCHIP_SEL_REGION1_HGH_REG_OFFSET        (0x38)
#define SCHIP_SEL_REGION2_HGH_REG_OFFSET        (0x3C)
#define SCHIP_SEL_REGION3_HGH_REG_OFFSET        (0x40)
#define SCHIP_SEL_REGION4_HGH_REG_OFFSET        (0x44)
#define SCHIP_SEL_REGION5_HGH_REG_OFFSET        (0x48)
#define SCHIP_SEL_REGION6_HGH_REG_OFFSET        (0x4C)
#define SCHIP_SEL_REGION7_HGH_REG_OFFSET        (0x50)
#define SMASK0_REG_OFFSET                       (0x54)
#define SMASK1_REG_OFFSET                       (0x58)
#define SMASK2_REG_OFFSET                       (0x5C)
#define SMASK3_REG_OFFSET                       (0x60)
#define SMASK4_REG_OFFSET                       (0x64)
#define SMASK5_REG_OFFSET                       (0x68)
#define SMASK6_REG_OFFSET                       (0x6C)
#define SMASK7_REG_OFFSET                       (0x70)
#define CHIP_SEL_ALIAS0_LOW_REG_OFFSET          (0x74)
#define CHIP_SEL_ALIAS1_LOW_REG_OFFSET          (0x78)
#define CHIP_SEL_ALIAS0_HGH_REG_OFFSET          (0x7C)
#define CHIP_SEL_ALIAS1_HGH_REG_OFFSET          (0x80)
#define CHIP_SEL_REMAP0_LOW_REG_OFFSET          (0x84)
#define CHIP_SEL_REMAP1_LOW_REG_OFFSET          (0x88)
#define CHIP_SEL_REMAP0_HGH_REG_OFFSET          (0x8C)
#define CHIP_SEL_REMAP1_HGH_REG_OFFSET          (0x90)
#define STATIC_MEM_TIMSET0_REG_OFFSET           (0x94)
#define STATIC_MEM_TIMSET1_REG_OFFSET           (0x98)
#define STATIC_MEM_TIMSET2_REG_OFFSET           (0x9C)
#define FLASH_TRPDR_REG_OFFSET                  (0xA0)
#define STATICM_EMCONTROL_REG_OFFSET            (0xA4)
#define SYNC_FLASH_OPCODE_REG_OFFSET            (0xA8)
#define EXTEND_MODE_REG_OFFSET                  (0xAC)
#define SYNC_FLASH_CONFIG_REG_OFFSET            (0xB0)
#define SYNC_FLASH_CONTROL_REG_OFFSET           (0xB4)
#define SYNC_FLASH_TIM_REG_OFFSET               (0xB8)

/* Macros ------------------------------------------------------------------ */

#define SETLOWFREQDEV                           (1<<27)
#define RESETLOWFREQDEV                         (0<<27)
#define SETREADYMODE                            (1<<26)
#define RESETREADYMODE                          (0<<26)
#define SETPAGEMODE                             (1<<26)
#define RESETPAGEMODE                           (0<<26)
#define PAGESIZE4                               (0<<24)
#define PAGESIZE8                               (1<<24)
#define PAGESIZE16                              (2<<24)
#define PAGESIZE32                              (3<<24)
#define T_PRC_0                                 (4)
#define T_BTA_0                                 (7)
#define T_WP_0                                  (20<<10)
#define T_WR_0                                  (3)
#define T_AS_0                                  (1)
#define T_RC_0                                  (32)
#define T_PRC_1                                 (1)
#define T_BTA_1                                 (1)
#define T_WP_1                                  (2)
#define T_WR_1                                  (1)
#define T_AS_1                                  (1)
#define T_RC_1                                  (4)
#define T_PRC_2                                 (1)
#define T_BTA_2                                 (1)
#define T_WP_2                                  (2)
#define T_WR_2                                  (1)
#define T_AS_2                                  (1)
#define T_RC_2                                  (4)

#define EBI_DECODE_0                            (0)
#define EBI_DECODE_1                            (1)
#define EBI_DECODE_2                            (2)
#define EBI_DECODE_3                            (3)

#define EBI_SMSKR_REG_SELECT_SHIFT              (8)
#define EBI_SMSKR_REG_SELECT_MASK               (3)
#define EBI_REG_SELECT_TIMING_SET_0             (0)
#define EBI_REG_SELECT_TIMING_SET_1             (1)
#define EBI_REG_SELECT_TIMING_SET_2             (2)

#define EBI_SMSKR_MEM_TYPE_SHIFT                (5)
#define EBI_SMSKR_MEM_TYPE_MASK                 (3)
#define EBI_MEM_TYPE_SDRAM                      (0)
#define EBI_MEM_TYPE_SRAM                       (1)
#define EBI_MEM_TYPE_FLASH                      (2)

#define EBI_SMSKR_MEM_SIZE_SHIFT                (0)
#define EBI_SMSKR_MEM_SIZE_MASK                 (0x1F)
#define EBI_MEM_SIZE_NO_MEMORY                  (0x00)
#define EBI_MEM_SIZE_64KB                       (0x01)
#define EBI_MEM_SIZE_128KB                      (0x02)
#define EBI_MEM_SIZE_256KB                      (0x03)
#define EBI_MEM_SIZE_512KB                      (0x04)
#define EBI_MEM_SIZE_1MB                        (0x05)
#define EBI_MEM_SIZE_2MB                        (0x06)
#define EBI_MEM_SIZE_4MB                        (0x07)
#define EBI_MEM_SIZE_8MB                        (0x08)
#define EBI_MEM_SIZE_16MB                       (0x09)
#define EBI_MEM_SIZE_32MB                       (0x0A)
#define EBI_MEM_SIZE_64MB                       (0x0B)
#define EBI_MEM_SIZE_128MB                      (0x0C)
#define EBI_MEM_SIZE_256MB                      (0x0D)
#define EBI_MEM_SIZE_512MB                      (0x0E)
#define EBI_MEM_SIZE_1GB                        (0x0F)
#define EBI_MEM_SIZE_2GB                        (0x10)
#define EBI_MEM_SIZE_4GB                        (0x11)

#define EBI_TIMING_SET_0                        (0)
#define EBI_TIMING_SET_1                        (1)
#define EBI_TIMING_SET_2                        (2)

#define EBI_SMTMGR_REG_READ_PIPE_SHIFT          (28)
#define EBI_SMTMGR_REG_READ_PIPE_MASK           (3)

#define EBI_SMTMGR_REG_CLK_SYNC_SHIFT           (27)
#define EBI_SMTMGR_REG_CLK_SYNC_MASK            (1)

#define EBI_SMTMGR_REG_READY_MODE_SHIFT         (26)
#define EBI_SMTMGR_REG_READY_MODE_MASK          (1)

#define EBI_SMTMGR_REG_PAGE_SIZE_SHIFT          (24)
#define EBI_SMTMGR_REG_PAGE_SIZE_MASK           (3)
#define EBI_PAGE_SIZE_4                         (0)
#define EBI_PAGE_SIZE_8                         (1)
#define EBI_PAGE_SIZE_16                        (2)
#define EBI_PAGE_SIZE_32                        (3)

#define EBI_SMTMGR_REG_PAGE_MODE_SHIFT          (23)
#define EBI_SMTMGR_REG_PAGE_MODE_MASK           (1)

#define EBI_SMTMGR_REG_PAGE_READ_CYCLE_SHIFT    (19)
#define EBI_SMTMGR_REG_PAGE_READ_CYCLE_MASK     (0xF)

#define EBI_SMTMGR_REG_BUS_TURN_AROUND_SHIFT    (16)
#define EBI_SMTMGR_REG_BUS_TURN_AROUND_MASK     (0x7)

#define EBI_SMTMGR_REG_WRITE_PULSE_SHIFT        (10)
#define EBI_SMTMGR_REG_WRITE_PULSE_MASK         (0x3F)

#define EBI_SMTMGR_REG_ADDR_HOLD_SHIFT          (8)
#define EBI_SMTMGR_REG_ADDR_HOLD_MASK           (3)

#define EBI_SMTMGR_REG_ADDR_SETUP_SHIFT         (6)
#define EBI_SMTMGR_REG_ADDR_SETUP_MASK          (3)

#define EBI_SMTMGR_REG_TIMING_READ_CYCLE_MASK   (0x1F)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_EBI_H */
