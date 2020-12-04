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
 *              Memory Interface Register Definitions
 *
 *****************************************************************************/ 
#ifndef __PC20X_MEMIF_H
#define __PC20X_MEMIF_H

#define MemifpASdBuffer0SetupRegOffset              (0x0000 * 2)
#define MemifpASdBuffer0AddressSetupRegOffset       (0x0001 * 2)
#define MemifpASdBuffer0FIFOSetupRegOffset          (0x0002 * 2)
#define MemifpASdBuffer1SetupRegOffset              (0x0004 * 2)
#define MemifpASdBuffer1AddressSetupRegOffset       (0x0005 * 2)
#define MemifpASdBuffer1FIFOSetupRegOffset          (0x0006 * 2)
#define MemifpASdBuffer2SetupRegOffset              (0x0008 * 2)
#define MemifpASdBuffer2AddressSetupRegOffset       (0x0009 * 2)
#define MemifpASdBuffer2FIFOSetupRegOffset          (0x000A * 2)
#define MemifpASdBuffer3SetupRegOffset              (0x000C * 2)
#define MemifpASdBuffer3AddressSetupRegOffset       (0x000D * 2)
#define MemifpASdBuffer3FIFOSetupRegOffset          (0x000E * 2)
#define MemifpASBuffer0SetupRegOffset               (0x0010 * 2)
#define MemifpASBuffer0AddressSetupRegOffset        (0x0011 * 2)
#define MemifpASBuffer0FIFOSetupRegOffset           (0x0012 * 2)
#define MemifpASBuffer1SetupRegOffset               (0x0014 * 2)
#define MemifpASBuffer1AddressSetupRegOffset        (0x0015 * 2)
#define MemifpASBuffer1FIFOSetupRegOffset           (0x0016 * 2)
#define MemifSdramArbGroup0SlotAConfigRegOffset     (0x0020 * 2)
#define MemifSdramArbGroup0SlotBConfigRegOffset     (0x0021 * 2)
#define MemifSdramArbGroup1SlotAConfigRegOffset     (0x0022 * 2)
#define MemifSdramArbGroup1SlotBConfigRegOffset     (0x0023 * 2)
#define MemifSdramArbGroup2SlotAConfigRegOffset     (0x0024 * 2)
#define MemifSdramArbGroup2SlotBConfigRegOffset     (0x0025 * 2)
#define MemifSdramArbGroup3SlotAConfigRegOffset     (0x0026 * 2)
#define MemifSdramArbGroup3SlotBConfigRegOffset     (0x0027 * 2)
#define MemifSdramArbGroup4SlotAConfigRegOffset     (0x0028 * 2)
#define MemifSdramArbGroup4SlotBConfigRegOffset     (0x0029 * 2)
#define MemifSdramArbGroup5SlotAConfigRegOffset     (0x002A * 2)
#define MemifSdramArbGroup5SlotBConfigRegOffset     (0x002B * 2)
#define MemifSdramArbGroup6SlotAConfigRegOffset     (0x002C * 2)
#define MemifSdramArbGroup6SlotBConfigRegOffset     (0x002D * 2)
#define MemifSdramArbGroup7SlotAConfigRegOffset     (0x002E * 2)
#define MemifSdramArbGroup7SlotBConfigRegOffset     (0x002F * 2)
#define MemifSdramArbValidGroupsConfigRegOffset     (0x0030 * 2)
#define MemifSramArbSlotAConfigRegOffset            (0x0040 * 2)
#define MemifSramArbSlotBConfigRegOffset            (0x0041 * 2)
#define MemifSramArbSlotCConfigRegOffset            (0x0042 * 2)
#define MemifSramArbValidSlotsConfigRegOffset       (0x0043 * 2)
#define MemifArbUpdateRegOffset                     (0x004F * 2)
#define MemifSdramSetupRegOffset                    (0x0050 * 2)
#define MemifSdramRefreshRateRegOffset              (0x0051 * 2)
#define MemifSdramMrsRegOffset                      (0x0052 * 2)
#define MemifSdramErsRegOffset                      (0x0053 * 2)
#define MemifSdramSetupCompleteReg                  (0x0054 * 2)
#define MemifSramSetupCompleteReg                   (0x0055 * 2)
#define MemifAhbArbPriorityPort0Masters03RegOffset  (0x0060 * 2)
#define MemifAhbArbPriorityPort0Masters46RegOffset  (0x0061 * 2)
#define MemifAhbArbPriorityPort1Masters03RegOffset  (0x0062 * 2)
#define MemifAhbArbPriorityPort1Masters46RegOffset  (0x0063 * 2)
#define MemifAhbArbPriorityPort2Masters03RegOffset  (0x0064 * 2)
#define MemifAhbArbPriorityPort2Masters46RegOffset  (0x0065 * 2)
#define MemifAhbArbPriorityPort3Masters03RegOffset  (0x0066 * 2)
#define MemifAhbArbPriorityPort3Masters46RegOffset  (0x0067 * 2)
#define MemifAhbArbPriorityPort4Masters03RegOffset  (0x0068 * 2)
#define MemifAhbArbPriorityPort4Masters46RegOffset  (0x0069 * 2)
#define MemifAhbArbitrationSchemeRegOffset          (0x006A * 2)
#define MemifTestModeEnableRegOffset                (0x0070 * 2)
#define MemifTestModeWriteDataRegOffset             (0x0071 * 2)
#define MemifTestModeAddressRegOffset               (0x0072 * 2)
#define MemifTestModeAddress2RegOffset              (0x0073 * 2)
#define MemifTestModeControlRegOffset               (0x0074 * 2)
#define MemifTestModeReadDataRegOffset              (0x0075 * 2)
#define MemifpABufferStatusRegOffset                (0x0080 * 2)
                     
/* DLL register offsets */
#define DLL0SlaveAdjustRegOffset                     0x0122
#define DLL1SlaveAdjustRegOffset                     0x0128
#define DLL2SlaveAdjustRegOffset                     0x012E
#define DLL3SlaveAdjustRegOffset                     0x0134
#define DLLConfigUpdateRegOffset                     0x0138

/* Memory Interface constants */

/* _SDRAM_Setup Register */
#define SdramSize14r10c                             0x00
#define SdramSize13r10c                             0x01
#define SdramSize13r9c                              0x10
#define SdramWidth32                                (0x0 << 2)
#define SdramWidth16                                (0x1 << 2)
#define SdramRWGap4                                 (0x4 << 3)
#define SdramWRGap7                                 (0x7 << 7)
#define SdramOdtDisabled                            (0x00 << 11)
#define SdramOdt75Ohm                               (0x01 << 11)
#define SdramOdt150Ohm                              (0x02 << 11)
#define SdramCaptureDelay0Mclk	                    (0x00 << 13)
#define SdramCaptureDelay0_5Mclk	            (0x01 << 13)
#define SdramCaptureDelay1Mclk	                    (0x02 << 13)
#define SdramCaptureDelay1_5Mclk	            (0x03 << 13)

/* _SDRAM_Refresh_Rate */
#define SdramRefreshCount                           0x05DC

/* _SRAM_Config_Done */
#define SramSetupComplete                           0x0001

/* _SDRAM_Config_Done */
#define SdramSetupComplete                          0x0001

/* _SDRAM_ERS */
#define SdramEmrsSetup                              0x0004

/* DLL Registers */
#define DLL0SlaveAdjustValue                        0x4803
#define DLL1SlaveAdjustValue                        0x4A03
#define DLL2SlaveAdjustValue                        0x4A03
#define DLL3SlaveAdjustValue                        0x4A03
#define DLLConfigUpdateInProgress                   0x0001
#define DLLConfigUpdate                             0x0001

#endif /* __PC20X_MEMIF_H */
