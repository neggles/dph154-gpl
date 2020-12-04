/*
 * entry-macro.S
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2008 picoChip Designs Ltd.
 *
 * Low-level IRQ helper macros for PC302 platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/hardware.h>
#include <mach/pc302/pc302.h>

    .macro  disable_fiq
    .endm

    .macro  get_irqnr_preamble, base, tmp
    .endm

    .macro  arch_ret_to_user, tmp1, tmp2
    .endm
            /*                   r0       r6     r5   lr */
    .macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
    /* Set the base address of the VIC. */
    ldr     \base,      =IO_ADDRESS(PC302_VIC1_BASE)

    /* Read the status register. */
    ldr     \irqstat,  [\base, #VIC_IRQ_STATUS_REG_OFFSET]
    mov     \irqnr,     #0

    /* Check for no interrupt. */
    teq     \irqstat,   #0
    beq     1003f               /*< There is no interrupt. */

    /* Find the number of leading 0's and subtract from 31 to get the
     * interrupt number. */
    clz     \irqnr,     \irqstat
    rsb     \irqnr,     \irqnr,     #31
    b       1005f

    /* Check the next VIC. This is exactly the same as above but with VIC0
     * rather than VIC1. */
1003:
    ldr     \base,      =IO_ADDRESS(PC302_VIC0_BASE)
    ldr     \irqstat,    [\base,     #VIC_IRQ_STATUS_REG_OFFSET]
    mov     \irqnr,     #0
    teq     \irqstat,   #0
    beq     1005f   /* There is no interrupt. */
    clz     \irqnr,     \irqstat
    rsb     \irqnr,     \irqnr,     #31
    add     \irqnr,     \irqnr,     #32
1005:
    .endm

