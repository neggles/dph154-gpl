/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file lowlevel_init.S
* \brief Various low level setup functions.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#include <config.h>
#include <asm/arch/pc20x.h>
#include <asm/arch/memif.h>
#include <asm/arch/ebi.h>
#include <asm/arch/gpio.h>
#include <asm/arch/rap.h>
#include <asm/arch/procif.h>

/*****************************************************************************
 *
 * lowlevel_init
 *
 * Description: Perform some low level PC20x initialisation
 *
 *****************************************************************************/

 .global lowlevel_init

 lowlevel_init:

        /* Preserve link register across call */
        mov r11,lr

#ifdef CONFIG_PC20X_SIMULATION
        bl      register_initialisation
#endif

        /* Extinguish LEDS controlled by the ARM gpio */
        bl      extinguishArmLeds

        /* Setup the ebi memory interface (Flash Memory) */
        bl      ebi_init

        /* Setup the memory interface (DDR2 Memory) */
        bl      memif_init

        /* Restore link register */
        mov     lr, r11

        /* Return to my caller */
        mov	pc, lr

/*****************************************************************************
 *
 * ebi_init
 *
 * Description: Initialise the External Bus Interface
 *
 *****************************************************************************/

ebi_init:
        ldr     r2, =PC20X_EBI_BASE
        ldr     r3, =StaticMemTimSet0Value
        str     r3, [r2, #StaticMemTimSet0RegOffset]

        /* Disable all three 'Write Protect' outputs from the EBI */
        ldr     r3, [r2, #StaticMemControlRegOffset]
        orr     r3, r3, #WriteProtectAllDisable
        str     r3, [r2, #StaticMemControlRegOffset]

        mov	pc, lr		/* Return to my caller */

/*****************************************************************************
 *
 * memif_init
 *
 * Description: Initialise the Memory Controller
 *
 * Note:
 * Before using SDRAM, we must wait 200us for it to initialise, accesses
 * will be held off until the initialisation process is complete.
 *
 *****************************************************************************/

memif_init:

/*****************************************************************************
 Reset the MEMIF before continuing
*****************************************************************************/
        ldr     r0, =PC20X_PROCIF_BASE
        ldr     r1, =ProcifIntGpioRegOffset
        ldr     r2, =ProcifGpioMemifSofResetMask
        orr     r0, r0, r1

        ldr     r1, [r0]
        orr     r1, r1, r2
        str     r1, [r0]

waitForMemIfResetToClear:
        ldr     r1, [r0]
        ands    r1, r1, r2
        bne     waitForMemIfResetToClear

/*****************************************************************************
 Setup MEMIF base address
*****************************************************************************/

        ldr     r2, =PC20X_MEM_IF_BASE

/*****************************************************************************
 DLL Configuration
*****************************************************************************/

/* DLL #0 */
        ldr     r3, =DLL0SlaveAdjustValue
        ldr     r4, =DLL0SlaveAdjustRegOffset
        orr     r4, r4, r2
        strh    r3, [r4]

/* DLL #1 */
        ldr     r3, =DLL1SlaveAdjustValue
        ldr     r4, =DLL1SlaveAdjustRegOffset
        orr     r4, r4, r2
        strh    r3, [r4]

/* DLL #2 */
        ldr     r3, =DLL2SlaveAdjustValue
        ldr     r4, =DLL2SlaveAdjustRegOffset
        orr     r4, r4, r2
        strh    r3, [r4]

/* DLL #3 */
        ldr     r3, =DLL3SlaveAdjustValue
        ldr     r4, =DLL3SlaveAdjustRegOffset
        orr     r4, r4, r2
        strh    r3, [r4]

/* Initiate a DLL update */
        ldr     r3, =DLLConfigUpdate
        ldr     r4, =DLLConfigUpdateRegOffset
        orr     r4, r4, r2
        strh    r3, [r4]

/* Wait until the DLL update has completed */

waitForDLLUpdate:
        ldrh    r3, [r4]
        tst     r3, #DLLConfigUpdateInProgress
        bne     waitForDLLUpdate

/*****************************************************************************
 SDRAM Arbitration
*****************************************************************************/

#ifndef CONFIG_PC20X_2_DDR_RAM_BANKS
/* We want a 4 DDR Bank setup then */

        mov     r3, #0x0000
        strh    r3, [r2, #MemifSdramArbGroup0SlotAConfigRegOffset]
        strh    r3, [r2, #MemifSdramArbGroup0SlotBConfigRegOffset]

        mov     r3, #0x0400
        orr     r3, r3, #0x0004
        strh    r3, [r2, #MemifSdramArbGroup1SlotAConfigRegOffset]
        strh    r3, [r2, #MemifSdramArbGroup1SlotBConfigRegOffset]

        mov     r3, #0x0003
        strh    r3, [r2, #MemifSdramArbValidGroupsConfigRegOffset]

#else   /* We want a 2 DDR Bank setup then */

        /* Group 0, Banks 0 & 1 -> ARM, Rd, Deterministic */
        mov     r3, #0x0000
        strh    r3, [r2, #MemifSdramArbGroup0SlotAConfigRegOffset]

        /* Group 0, Banks 2 & 3 -> pA, Rd, Deterministic */
        mov     r3, #0x0100
        orr     r3, r3, #0x0001
        strh    r3, [r2, #MemifSdramArbGroup0SlotBConfigRegOffset]

        /* Group 1, Banks 0 & 1 -> ARM, Wr, Deterministic */
        mov     r3, #0x0400
        orr     r3, r3, #0x0004
        strh    r3, [r2, #MemifSdramArbGroup1SlotAConfigRegOffset]

        /* Group 1, Banks 2 & 3 -> pA, Wr Deterministic */
        mov     r3, #0x0500
        orr     r3, r3, #0x0005
        strh    r3, [r2, #MemifSdramArbGroup1SlotBConfigRegOffset]

        /* Valid Groups, 2 */
        mov     r3, #0x0003
        strh    r3, [r2, #MemifSdramArbValidGroupsConfigRegOffset]

#endif

/*****************************************************************************
 SDRAM Setup
*****************************************************************************/

/* Set the SDRAM size and width */
        mov     r3, #SdramSize13r10c
        orr     r3, r3, #SdramWidth32
        orr     r3, r3, #SdramRWGap4
        orr     r3, r3, #SdramWRGap7
        orr     r3, r3, #SdramOdt75Ohm
        orr     r3, r3, #SdramCaptureDelay1Mclk
        strh    r3, [r2,#MemifSdramSetupRegOffset]

/* Set the refresh rate */
        ldr     r3, =SdramRefreshCount
        strh    r3, [r2, #MemifSdramRefreshRateRegOffset]

/* Set the device mode register */
        mov     r3, #0x0400
        orr     r3, r3, #0x0042
        strh    r3, [r2, #MemifSdramMrsRegOffset]

/* Set the extended mode register */
        ldr     r3, =SdramEmrsSetup
        strh    r3, [r2, #MemifSdramErsRegOffset]

/*****************************************************************************
 Configuration complete, start the mem-if
*****************************************************************************/
        mov     r3, #SdramSetupComplete
        strh    r3, [r2, #MemifSdramSetupCompleteReg]
        mov     r3, #SramSetupComplete
        strh    r3, [r2, #MemifSramSetupCompleteReg]

        mov	pc, lr		/* Return to my caller */

/*****************************************************************************
 *
 * ddr2_dummy_access
 *
 * Description: Perform a 'dummy' write to and read back from the DDR2 SDRAM
 *
 *****************************************************************************/
.global ddr2_dummy_access

 ddr2_dummy_access:
        ldr     r0, =PHYS_SDRAM_1
        mov     r1, #0x00000000
        str     r1, [r0]    /* perform a 'dummy' write */

        ldr     r2, [r0]    /* perform a 'dummy' read */
        cmp     r1, r2      /* ignore the result */

        mov	pc, lr      /* Return to my caller */

/*****************************************************************************
 *
 * register_initialisation
 *
 * Description: Initialise some CPU registers
 *
 * Note:
 * Used to prevent X propagation if running this code in a simulation
 *
 *****************************************************************************/

register_initialisation:

        mov     r0, #0x00000000
        mov     r1, #0x00000000
        mov     r2, #0x00000000
        mov     r3, #0x00000000
        mov     r4, #0x00000000
        mov     r5, #0x00000000
        mov     r6, #0x00000000
        mov     r7, #0x00000000
        mov     r8, #0x00000000
        mov     r9, #0x00000000
        mov     r10,#0x00000000

        /* Don't noodle with r11 (fp)
                             r12 (ip)
                             r13 (sp)
                             r14 (lr)
                             r15 (pc)
           as this would cause big probs ! */

        /* Return to my caller */
        mov	pc, lr

/*****************************************************************************
 *
 * assertBootError
 *
 * Illuminate LED15 on the picoChip CPE20x hardware platform
 *
 *****************************************************************************/
 .global assertBootError

 assertBootError:

         /* Get base address of the gpio block */
        ldr r0,=PC20X_GPIO_BASE

        /* Set up ARM gpio bit #5 as an o/p */
        ldr r1,[r0, #GpioPortADataDirectionRegOffset]
        orr r1,r1,#Gpio(5)
        str r1,[r0, #GpioPortADataDirectionRegOffset]

        /* Switch LED to RED on the ip202ff Platform */
        ldr r1,[r0, #GpioPortAOutputDataRegOffset]
        bic r1,r1,#Gpio(5)
        str r1,[r0, #GpioPortAOutputDataRegOffset]

        /* Return to my caller */
        mov	pc, lr

/*****************************************************************************
 *
 * extinguishArmLeds
 *
 * Extinguish both ARM controlled LEDS on the CPE20x Platform
 *
 *****************************************************************************/

.global extinguishArmLeds

extinguishArmLeds:

        /* Get base address of the gpio block */
        ldr r0,=PC20X_GPIO_BASE

        /* Setup ARM gpio bit #5 as an output */
        ldr r1,[r0, #GpioPortADataDirectionRegOffset]
        orr r1,r1,#Gpio(5)
        str r1,[r0, #GpioPortADataDirectionRegOffset]

        /* Switch LED to GREEN on the ip202ff Platform */
        ldr r1,[r0, #GpioPortAOutputDataRegOffset]
        orr r1,r1,#Gpio(5)
        str r1,[r0, #GpioPortAOutputDataRegOffset]

        /* Return to my caller */
        mov     pc, lr

       .end
