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
#include <asm/mmu.h>

struct fsl_e_tlb_entry tlb_table[] = {
	/*
	 * TLB0		16K	Cacheable, non-guarded
	 * 			Based at CFG_INIT_RAM_ADDR
	 *
	 * Use four 4K TLB0 entries.  These entries must be cacheable
	 * as they provide the bootstrap memory before the memory
	 * controler and real memory have been configured.
	 */
	SET_TLB_ENTRY(0, CFG_INIT_RAM_ADDR, CFG_INIT_RAM_ADDR,
		      MAS3_SX|MAS3_SW|MAS3_SR, 0,
		      0, 0, BOOKE_PAGESZ_4K, 0),
	SET_TLB_ENTRY(0, CFG_INIT_RAM_ADDR + 4 * 1024 , CFG_INIT_RAM_ADDR + 4 * 1024,
		      MAS3_SX|MAS3_SW|MAS3_SR, 0,
		      0, 0, BOOKE_PAGESZ_4K, 0),
	SET_TLB_ENTRY(0, CFG_INIT_RAM_ADDR + 8 * 1024 , CFG_INIT_RAM_ADDR + 8 * 1024,
		      MAS3_SX|MAS3_SW|MAS3_SR, 0,
		      0, 0, BOOKE_PAGESZ_4K, 0),
	SET_TLB_ENTRY(0, CFG_INIT_RAM_ADDR + 12 * 1024 , CFG_INIT_RAM_ADDR + 12 * 1024,
		      MAS3_SX|MAS3_SW|MAS3_SR, 0,
		      0, 0, BOOKE_PAGESZ_4K, 0),

	/*
	 * TLB1,0:	256M	Non-cacheable, guarded
	 * 0xF0000000	256M	FLASH (We only have 32M..128M, but map more)
	 * Out of reset this entry is only 4K.
	 */
	SET_TLB_ENTRY(1, (CFG_FLASH_BASE & 0xF0000000), (CFG_FLASH_BASE & 0xF0000000),
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 0, BOOKE_PAGESZ_256M, 1),

	/*
	 * TLB1,1:	256M	Non-cacheable, guarded
	 * 0xE0000000	256M	CPLD
	 */
	SET_TLB_ENTRY(1, CFG_CPLD_MEM_BASE, CFG_CPLD_MEM_BASE,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 1, BOOKE_PAGESZ_256M, 1),

	/*
	 * TLB1,2:	256M	Non-cacheable, guarded
	 * 0xD0000000	256M	PicoChip Registers
	 */
	SET_TLB_ENTRY(1, CFG_PREG_MEM_BASE, CFG_PREG_MEM_BASE,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 2, BOOKE_PAGESZ_256M, 1),

	/*
	 * TLB1,3:	256M	Non-cacheable, guarded
	 * 0xC0000000	256M	PicoChip FIFOs
	 */
	SET_TLB_ENTRY(1, CFG_PFIFO_MEM_BASE, CFG_PFIFO_MEM_BASE,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 3, BOOKE_PAGESZ_256M, 1),

	/*
	 * TLB1,4:	256M	Non-cacheable, guarded
	 * 0xB0000000	256M	Daughtercard (ATM)
	 */
	SET_TLB_ENTRY(1, CFG_DCARD_MEM_BASE, CFG_DCARD_MEM_BASE,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 4, BOOKE_PAGESZ_256M, 1),

	/*
	 * TLB1,5:	64M	Non-cacheable, guarded
	 * 0x8000_0000  1M      CCSRBAR
	 */
	SET_TLB_ENTRY(1, CFG_CCSRBAR, CFG_CCSRBAR,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 5, BOOKE_PAGESZ_64M, 1),

        /*
         * TLB1,6:      256M    Non-cacheable, guarded
         * 0x40000000   256M    PCI (Crypto processor)
         */
	SET_TLB_ENTRY(1, CFG_PCI1_MEM_BASE, CFG_PCI1_MEM_BASE,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 6, BOOKE_PAGESZ_256M, 1),
        
        /*
         * TLB1,7:      256M    Non-cacheable, guarded
         * 0x40000000   256M    PCI (Crypto processor) - IO area
         */
	SET_TLB_ENTRY(1, CFG_PCI1_IO_BASE, CFG_PCI1_IO_BASE,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 6, BOOKE_PAGESZ_256M, 1),                      

#if !defined(CONFIG_SPD_EEPROM)
	/*
	 * TLB1,8:	256M	DDR
	 * 0x00000000	256M	DDR System memory
	 * Without SPD EEPROM configured DDR, this must be setup manually.
	 * Make sure the TLB count at the top of this table is correct.
	 * Likely it needs to be increased by two for these entries.
	 */

	SET_TLB_ENTRY(1, CFG_DDR_SDRAM_BASE, CFG_DDR_SDRAM_BASE,
		      MAS3_SX|MAS3_SW|MAS3_SR, 0,
		      0, 8, BOOKE_PAGESZ_256M, 1),
#endif
};

int num_tlb_entries = ARRAY_SIZE(tlb_table);
