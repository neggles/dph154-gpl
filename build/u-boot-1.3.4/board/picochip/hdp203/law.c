/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright 2008 Freescale Semiconductor, Inc.
 *
 * (C) Copyright 2000
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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

#include <common.h>
#include <asm/fsl_law.h>
#include <asm/mmu.h>

/*
 * LAW(Local Access Window) configuration:
 *
 * BAR0    - DDR @ 0x00000000, 256M
 * BAR1    - PCI @ 0x40000000, 1G
 * BAR2    - Local Bus (Daughtercard) @ 0xB0000000, 256M
 * BAR3    - Local Bus (Pico + CPLD + Flash) @ 0xC0000000, 1G
 *
 * Notes:
 *    CCSRBAR and L2-as-SRAM don't need a configured Local Access Window.
 *    If flash is 8M at default position (last 8M), no LAW needed.
 */

/* IMPORTANT: LAWBAR0 is reserved (for things we don't do on this target), 
 * and the loader knows this.  The first loaded register is LAWBAR1.
 */

/* DDR mapping.
 *
 * We must not enable the DDR LAW until the DDR controller has been 
 * initialised (otherwise the debugger falls over...)
 * The enable is set in spd_sdram or fixed_sdram depending on config
 */

struct law_entry law_table[] = {
#ifndef CONFIG_SPD_EEPROM
	/* DDR mapping.
         *
         * We must not enable the DDR LAW until the DDR controller has been 
         * initialised (otherwise the debugger falls over...)
         * The enable is set in spd_sdram or fixed_sdram depending on config
         */
        SET_LAW_ENTRY(1, CFG_DDR_SDRAM_BASE, LAW_SIZE_256M, LAW_TRGT_IF_DDR),
#endif
	/* PCI mapping - for Crypto chip */
        SET_LAW_ENTRY(2, CFG_PCI1_MEM_BASE, LAW_SIZE_256M, LAW_TRGT_IF_PCI),
	SET_LAW_ENTRY(3, CFG_PCI1_IO_BASE, LAW_SIZE_256M, LAW_TRGT_IF_PCI),

	/* Local bus mapping for ATM daughtercard */
        SET_LAW_ENTRY(4, CFG_DCARD_MEM_BASE, LAW_SIZE_256M, LAW_TRGT_IF_LBC),
	
        /* Main IO mapping, including flash */
        SET_LAW_ENTRY(5, CFG_PFIFO_MEM_BASE, LAWAR_SIZE_1G, LAW_TRGT_IF_LBC),
};

int num_law_entries = ARRAY_SIZE(law_table);
