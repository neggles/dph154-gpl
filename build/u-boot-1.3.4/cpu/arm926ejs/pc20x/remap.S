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
#include <asm/arch/rap.h>
#include <asm/arch/procif.h>

	.align	5

/*!
 *
 * Remap the bootable flash back to its 'correct' location in the
 * pc20x memory map.
 * Once this function returns we will be able to access
 * sdram at addr 0x00000000.
 *                
 */
 .global remap_flash
 
 remap_flash:      
 
        /* Check the remap in the rap block and return if we are already remapped */
        ldr     r2, = PC20X_REMAP_BASE
        ldr     r3, [r2,#RapRemapModeRegOffset]
        ands    r3, r3, #REMAP_NORMAL_MODE
        
        /* If we are, don't remap */
        bne     remap_flash_return                      
 	
        /* Preserve link register across call */
        mov r11,lr                                       

        
        /* Check to see if we are doing a software reboot */
        ldr     r2, = PC20X_PROCIF_BASE                 
        ldr     r3, [r2,#ProcifExtTabEnt0RegOffset]   
        cmp     r3, #PC20X_BOOTABLE_FLASH_BASE

        /* If we are, don't remap */
        beq     remap_flash_return                      
        
        bl  noodle_program_counter
        
        
        /* We are now executing code from the 'correct' Bootable Flash memory address */
        
        /* Perform the remap in the rap block */
        ldr     r2, = PC20X_REMAP_BASE
        mov     r3, #REMAP_NORMAL_MODE
        str     r3, [r2,#RapRemapModeRegOffset]
 	
	/* Allow things to settle down... */
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop
        nop

        /* Restore link register */
        mov     lr, r11                                 

        /* on return, execute code from Bootable Flash memory */
        orr     lr, lr, #PC20X_BOOTABLE_FLASH_BASE	

remap_flash_return:

        /* Return to my caller */
        mov	pc, lr		                        
        
 
 /*!
 *
 * Set the pc so we are executing code from Bootable Flash memory
 * at the 'correct' address in the PC20x memory map.
 *                
 */
 noodle_program_counter:
 
        /* On return, execute code from Bootable Flash memory */
        orr     lr, lr, #PC20X_BOOTABLE_FLASH_BASE	

        /* Return to my caller */
        mov     pc, lr		                        
