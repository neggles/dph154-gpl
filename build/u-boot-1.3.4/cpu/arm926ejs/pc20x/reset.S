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
#include <config.h>
#include <asm/arch/pc20x.h>
#include <asm/arch/procif.h>

	.align	5

 /*!
 *
 * Perform a software reset of the PC20x device.
 *
 * Note: We will never return from this function
 *
 */
.globl reset_cpu
reset_cpu:

        /* Set up the new reset exception vector in the procif */
        ldr     r2, = PC20X_PROCIF_BASE                 
        mov     r3, #PC20X_BOOTABLE_FLASH_BASE
        str     r3, [r2,#ProcifExtTabEnt0RegOffset]
        
        /* Replace the default reset exception instruction at addr 0xFFFF0000 */
        ldr     r3, ldr_instruction
        str     r3, [r2]                                
        
        /* Perform a software reboot */
        mov     pc, r2                                  

reset_loop_forever:

        /* We will never return from this function */
        b       reset_loop_forever                      

ldr_instruction:

        /* ldr pc,[0xffff0020]... */
       .word 0xe59ff018                                 
