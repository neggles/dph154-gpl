/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/
 
/*!
* \file
* \brief remap.S
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/ 

#include <config.h>
#include <asm/arch/pc302.h>
#include <asm/arch/axi2cfg.h>

	.align	5
 
/*!
 *
 * Remap the EBI connected parallel flash memory from its aliased
 * location to its 'correct' location in the pc302 memory map.
 *              
 * Note: Once this function returns we will be able to access
 *       DDR SDRAM at address 0x00000000 (assuming DDR SDRAM
 *       has been initialised).
 *                
 */
 .global remap_flash
 
 remap_flash:     
 	    
        /* Preserve link register across call */
        mov r11,lr                                      
        
        bl  noodle_program_counter
        
        /* We are now executing code from the 'correct' 
           Bootable Flash memory address */
        
        /* Perform the 'remap' in the axi2cfg block */
        ldr     r0, =PC302_AXI2CFG_BASE
        ldr     r1, [r0, #AXI2CFG_SYS_CONFIG_REG_OFFSET]
        bic     r1, #AXI2CFG_SYS_CONFIG_RW_REMAP_MASK
        str     r1, [r0, #AXI2CFG_SYS_CONFIG_REG_OFFSET]
 	
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
        orr     lr, lr, #PC302_BOOTABLE_FLASH_BASE	

remap_flash_return:
        
        /* Return to my caller */
        mov	pc, lr		                        
       
/*!
 *
 * Set the pc so we are executing code from Bootable
 * at the 'correct' address in the PC302 memory map.
 *                
 */
 noodle_program_counter:
 
        /* On return, execute code from Bootable Flash memory */
        orr     lr, lr, #PC302_BOOTABLE_FLASH_BASE	

        /* Return to my caller */
        mov     pc, lr		                        
