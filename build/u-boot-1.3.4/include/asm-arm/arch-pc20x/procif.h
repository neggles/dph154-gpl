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
 * Description: picoChip PC20x Procif AHB Slave Register Definitions
 *
 *****************************************************************************/
#ifndef __PC20X_PROCIF_H
#define __PC20X_PROCIF_H
 
/* Slave AHB Interface Registers */

#define PC20X_PROCIF_BASE           0xFFFF0000	/* Base Address */
#define ProcifExtTabEnt0RegOffset   0x0020      /* Extended Table Entry #0 */
#define ProcifExtTabEnt1RegOffset   0x0024      /* Extended Table Entry #1 */
#define ProcifExtTabEnt2RegOffset   0x0028      /* Extended Table Entry #2 */
#define ProcifExtTabEnt3RegOffset   0x002C      /* Extended Table Entry #3 */
#define ProcifExtTabEnt4RegOffset   0x0030      /* Extended Table Entry #4 */
#define ProcifExtTabEnt5RegOffset   0x0034      /* Extended Table Entry #5 */
#define ProcifExtTabEnt6RegOffset   0x0038      /* Extended Table Entry #6 */
#define ProcifExtTabEnt7RegOffset   0x003C      /* Extended Table Entry #7 */

#define ProcifIntGpioRegOffset      0x03FC      /* Internal GPIO register offset */
	
#define ProcifGpioARMSofResetIdx    31          /* ARM Sub-System Soft Reset */                                               
#define ProcifGpioARMSofResetMask   (1<<ProcifGpioARMSofResetIdx)

#define ProcifGpioPicoSofResetIdx   30          /* picoArray Soft Reset */ 
#define ProcifGpioPicoSofResetMask  (1<<ProcifGpioPicoSofResetIdx)

#define ProcifGpioMemifSofResetIdx  29          /* MEM-IF Soft Reset */
#define ProcifGpioMemifSofResetMask (1<<ProcifGpioMemifSofResetIdx)

#define ProcifGpioSlaveAHBAccessIdx     28      /* Slave AHB Access Control */ 
#define ProcifGpioSlaveAHBAccessMask    (1<<ProcifGpioSlaveAHBAccessIdx)

#define ProcifGpioPicoAccessIdx     27          /* picoArray Access Control */ 
#define ProcifGpioPicoAccessMask    (1<<ProcifGpioPicoAccessIdx)

#define ProcifIRQRequestIdx         6           /* IRQ Request, used to assert an ARM926EJ IRQ via the VIC */
#define ProcifIRQRequestMask        (1<<ProcifIRQRequestIdx)

#define ProcifWdogPauseIdx          5           /* Watchdog Pause, when asserted pauses the Watchdog Timer's countdown */ 
#define ProcifWdogPauseMask         (1<<ProcifWdogPauseIdx)

#define ProcifDmac1DreqMuxIdx       4           /* DMAC1 DREQ Mux, allows DMAC1's DREQ[1] & [3] source to be selected in master mode */
#define ProcifDmac1DreqMuxMask      (1<<ProcifDmac1DreqMuxIdx)

#define ProcifDmac2DreqMuxIdx       3           /* DMAC2 DREQ Mux, allows DMAC2's DREQ[1] & [3] source to be selected in master mode */
#define ProcifDmac2DreqMuxMask      (1<<ProcifDmac2DreqMuxIdx)

#define ProcifReverseMIIEnablePinIdx    2       /* Reverse MII Enable Pin N/A Returns the value of the Reverse MII Block enable pin */
#define ProcifReverseMIIEnablePinMask   (1<<ProcifReverseMIIEnablePinIdx)

#define ProcifEthernetBootModePinIdx    1 	/* Ethernet Boot Mode Pin N/A Returns the value of the Ethernet Boot mode input pin */
#define ProcifEthernetBootModePinMask   (1<<ProcifEthernetBootModePinIdx)

#define ProcifSlaveMasterModePinIdx     0       /* Slave/Master Mode Pin N/A Returns the value of the Slave/Master mode input pin */
#define ProcifSlaveMasterModePinMask    (1<<ProcifSlaveMasterModePinIdx)

#endif /* __PC20X_PROCIF_H */

