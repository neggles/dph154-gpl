/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright(C) 2006 picoChip(R) Designs Ltd.
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
 *              External Bus Interface Register Definitions
 *
 *****************************************************************************/ 
#ifndef __PC20X_EBI_H
#define __PC20X_EBI_H

#define SdramConRegOffset               0x00
#define SdramTim0RegOffset              0x04
#define SdramTim1RegOffset              0x08
#define SdramCtlRegOffset               0x0C
#define SdramRefreshRegOffset           0x10
#define SChipSelRegion0LowRegOffset     0x14
#define SChipSelRegion1LowRegOffset     0x18
#define SChipSelRegion2LowRegOffset     0x1C
#define SChipSelRegion3LowRegOffset     0x20
#define SChipSelRegion4LowRegOffset     0x24
#define SChipSelRegion5LowRegOffset     0x28
#define SChipSelRegion6LowRegOffset     0x2C
#define SChipSelRegion7LowRegOffset     0x30
#define SChipSelRegion0HghRegOffset     0x34
#define SChipSelRegion1HghRegOffset     0x38
#define SChipSelRegion2HghRegOffset     0x3C
#define SChipSelRegion3HghRegOffset     0x40
#define SChipSelRegion4HghRegOffset     0x44
#define SChipSelRegion5HghRegOffset     0x48
#define SChipSelRegion6HghRegOffset     0x4C
#define SChipSelRegion7HghRegOffset     0x50
#define SMask0RegOffset                 0x54
#define SMask1RegOffset                 0x58
#define SMask2RegOffset                 0x5C
#define SMask3RegOffset                 0x60
#define SMask4RegOffset                 0x64
#define SMask5RegOffset                 0x68
#define SMask6RegOffset                 0x6C
#define SMask7RegOffset                 0x70
#define ChipSelAlias0LowRegOffset       0x74
#define ChipSelAlias1LowRegOffset       0x78
#define ChipSelAlias0HghRegOffset       0x7C
#define ChipSelAlias1HghRegOffset       0x80
#define ChipSelRemap0LowRegOffset       0x84
#define ChipSelRemap1LowRegOffset       0x88
#define ChipSelRemap0HghRegOffset       0x8C
#define ChipSelRemap1HghRegOffset       0x90
#define StaticMemTimSet0RegOffset       0x94
#define StaticMemTimSet1RegOffset       0x98
#define StaticMemTimSet2RegOffset       0x9C
#define FlashTrpdrRegOffset             0xA0
#define StaticMemControlRegOffset       0xA4
#define SyncFlashOpcodeRegOffset        0xA8
#define ExtendModeRegOffset             0xAC
#define SyncFlashConfigRegOffset        0xB0
#define SyncFlashControlRegOffset       0xB4
#define SyncFlashTimRegOffset           0xB8

/* EBI bus speeds for Flash access - depends on clock speed of board variant */
#if defined(IPA_BOARD_TYPE_XB) || defined(IPA_BOARD_TYPE_XCMINUS)
#   define StaticMemTimSet0Value           0x10C7174F 

#elif defined(IPA_BOARD_TYPE_XBPLUS) || defined(IPA_BOARD_TYPE_XC)
#   define StaticMemTimSet0Value           0x10F7279C 

#else
#   error IPA_BOARD_TYPE not defined so EBI bus config not known!
#   define StaticMemTimSet0Value           0x10C7174F 
#endif


#define WriteProtectAllDisable          0x0000000E


#endif /* __PC20X_EBI_H */
