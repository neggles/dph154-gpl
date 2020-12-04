/*
 * include/asm-arm/arch-firecracker/entry-macro.S
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * Low-level IRQ helper macros for Firecracker platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/hardware.h>

                .macro  disable_fiq
                .endm

                .macro  get_irqnr_preamble, base, tmp
                .endm

                .macro  arch_ret_to_user, tmp1, tmp2
                .endm

                .macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
                ldr     \base, =IO_ADDRESS(PC20X_VIC_BASE)              @ get virtual base
                ldr     \irqstat, [\base, #VIC_STATUS_REG_OFFSET]       @ get masked status
                mov     \irqnr, #0                                      @ Init irq number to zero
                teq     \irqstat, #0                                    @ Test the irq status read from the VIC               
                beq     1003f                                           @  and if zero, branch (and set the EQ flag)

                clz     \irqnr, \irqstat                                @ Count leading zeros to get the irq number
                rsb     \irqnr, \irqnr, #31                             @ irqnr = 31 - irqnr so that bit 0 is irq 0 etc.
1003:          /* EQ will be set if we reach MAXIRQNUM */
                .endm

