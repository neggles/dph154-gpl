/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Board support for the picoChip HDP203 platform (http://www.picochip.com/)
 *
 * Copyright 2005 Zetetica Ltd.  <david.warman@zetetica.co.uk>
 *     originally based on MPC8560ADS support.
 *
 * Copyright 2004 Freescale Semiconductor.
 * (C) Copyright 2003,Motorola Inc.  Xianghua Xiao, (X.Xiao@motorola.com)
 * (C) Copyright 2002 Scott McNutt <smcnutt@artesyncp.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Revision April 13, 2007 - stuartr
 * 	Arrays of constants for UPM updated : Dummy write address 
 * 	changed from 0xC000_0000 to 0xD000_0000, three wait states added
 * 	to pico_single_read{ } upm_data table 
 *
 */


#include <common.h>
#include <pci.h>
#include <asm/processor.h>
#include <asm/immap_85xx.h>
#include <asm/mmu.h>
#include <ioports.h>
#include <spd.h>
#include <miiphy.h>

#if defined(CONFIG_DDR_ECC)
extern void ddr_enable_ecc(unsigned int dram_size);
#endif

extern long int spd_sdram(void);

void local_bus_init(void);
long int fixed_sdram(void);
void pico_init(void);

/*
 * I/O Port configuration table
 *
 * if conf is 1, then that port pin will be configured at boot time
 * according to the five values podr/pdir/ppar/psor/pdat for that entry
 */

/* Names in the rightmost column match the netlist names on the schematic */

const iop_conf_t iop_conf_tab[4][32] = {
    /* For the moment, leave PORTA tristate (default) in the bootstrap */
    /* Port A configuration */
    {   /*            conf ppar psor pdir podr pdat */
	/* PA31 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXEN */
	/* PA30 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXCLAV */
	/* PA29 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXSOC */
	/* PA28 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXEN */
	/* PA27 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXSOC */
	/* PA26 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXCLAV */
	/* PA25 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXD0 */
	/* PA24 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXD1 */
	/* PA23 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXD2 */
	/* PA22 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXD3 */
	/* PA21 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXD4 */
	/* PA20 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXD5 */
	/* PA19 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXD6 */
	/* PA18 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXD7 */
	/* PA17 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXD7 */
	/* PA16 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXD6 */
	/* PA15 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXD5 */
	/* PA14 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXD4 */
	/* PA13 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXD3 */
	/* PA12 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXD2 */
	/* PA11 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXD1 */
	/* PA10 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXD0 */
	/* PA9  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PA8  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SP7 */
	/* PA7  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SP6 */
	/* PA6  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SP5 */
	/* PA5  */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXPAR */
	/* PA4  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SP4 */
	/* PA3  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SP3 */
	/* PA2  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SP2 */
	/* PA1  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SP1 */
	/* PA0  */ {   0,   0,   0,   0,   0,   0   }  /* CPM_SP0 */
    },

    /* Port B configuration */
    {   /*            conf ppar psor pdir podr pdat */
	/* PB31 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO15 */
	/* PB30 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO14 */
	/* PB29 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO13 */
	/* PB28 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO12 */
	/* PB27 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO11 */
	/* PB26 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO10 */
	/* PB25 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO9 */
	/* PB24 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO8 */
	/* PB23 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO7 */
	/* PB22 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO6 */
	/* PB21 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO5 */
	/* PB20 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO4 */
	/* PB19 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO3 */
	/* PB18 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO2 */
	/* PB17 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO1 */
	/* PB16 */ {   0,   0,   0,   0,   0,   0   }, /* CPM_GPIO0 */
	/* PB15 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB14 */ {   1,   1,   0,   0,   0,   0   }, /* RS232_RX0 (SCC3) */
	/* PB13 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB12 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB11 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB10 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB9  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB8  */ {   1,   1,   1,   1,   0,   0   }, /* RS232_TX0 (SCC3) */
	/* PB7  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB6  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB5  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB4  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PB3  */ {   0,   0,   0,   0,   0,   0   }, /* pin doesn't exist */
	/* PB2  */ {   0,   0,   0,   0,   0,   0   }, /* pin doesn't exist */
	/* PB1  */ {   0,   0,   0,   0,   0,   0   }, /* pin doesn't exist */
	/* PB0  */ {   0,   0,   0,   0,   0,   0   }  /* pin doesn't exist */
    },

    /* Port C */
    {   /*            conf ppar psor pdir podr pdat */
	/* PC31 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC30 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC29 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC28 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC27 */ {   1,   1,   0,   0,   0,   0   }, /* CPM_CLK5 */
	/* PC26 */ {   1,   1,   0,   0,   0,   0   }, /* CPM_CLK6 */
	/* PC25 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC24 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC23 */ {   1,   1,   0,   0,   0,   0   }, /* CPM_CLK9 */
	/* PC22 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXPAR */
	/* PC21 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC20 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC19 */ {   1,   1,   1,   0,   0,   0   }, /* CPM_SPI_CLK */
	/* PC18 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC17 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC16 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC15 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXA0 */
	/* PC14 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXA0 */
	/* PC13 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXA1 */
	/* PC12 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXA1 */
	/* PC11 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC10 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC9  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC8  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PC7  */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXA2 */
	/* PC6  */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXA2 */
	/* PC5  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SPI_nCS5 */
	/* PC4  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SPI_nCS4 */
	/* PC3  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SPI_nCS3 */
	/* PC2  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SPI_nCS2 */
	/* PC1  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SPI_nCS1 */
	/* PC0  */ {   0,   0,   0,   0,   0,   0   }, /* CPM_SPI_nCS0 */
    },

    /* Port D */
    {   /*            conf ppar psor pdir podr pdat */
	/* PD31 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD30 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD29 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_RXA3 */
	/* PD28 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD27 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD26 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD25 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD24 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD23 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD22 */ {   1,   1,   0,   0,   0,   0   }, /* RS232_RX1 (SCC4) */
	/* PD21 */ {   1,   1,   0,   1,   0,   0   }, /* RS232_TX1 (SCC4) */
	/* PD20 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD19 */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXA4 */
	/* PD18 */ {   0,   0,   0,   0,   0,   0   }, /* ATX_RXA4 */
	/* PD17 */ {   1,   1,   1,   0,   0,   0   }, /* CPM_SPI_MOSI */
	/* PD16 */ {   1,   1,   1,   0,   0,   0   }, /* CPM_SPI_MISO */
	/* PD15 */ {   1,   1,   1,   0,   1,   0   }, /* CPM_I2C_DA */
	/* PD14 */ {   1,   1,   1,   0,   1,   0   }, /* CPM_I2C_CLK */
	/* PD13 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD12 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD11 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD10 */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD9  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD8  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD7  */ {   0,   0,   0,   0,   0,   0   }, /* ATM_TXA3 */
	/* PD6  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD5  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD4  */ {   0,   0,   0,   0,   0,   0   }, /* - (NC) */
	/* PD3  */ {   0,   0,   0,   0,   0,   0   }, /* pin doesn't exist */
	/* PD2  */ {   0,   0,   0,   0,   0,   0   }, /* pin doesn't exist */
	/* PD1  */ {   0,   0,   0,   0,   0,   0   }, /* pin doesn't exist */
	/* PD0  */ {   0,   0,   0,   0,   0,   0   }  /* pin doesn't exist */
    }
};

typedef struct upm_data
{
	uint mode;
	uint dummy;
	uint entries;
	uint data[];
} upm_data_t;

/* UPM configuration for access to picoChip DMAs */

const upm_data_t pico_single_read =
{ 	0x10000000, 	/* offset 0x00 */
	0xD0000000,
	6,
	{ 0x0FFFFC00, 
	  0x0FFCFC00,
	  0x0FFCFC00,  /* 1st wait state */
	  0x0FFCFC00,  /* 2nd wait state */
	  0x0FFCFC00,  /* 3rd wait state */
	  0x0FFCFC05
	}
};

const upm_data_t pico_burst_read =    /* 3 wait states need to be added in here if bursting is to be used. To be done */
{ 	0x10000008, 	/* offset 0x08 */
	0xD0000000,
	10,
	{ 0x0FFEFC00,
	  0x0FFCFC04,
	  0x0FFCFC04,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0D
	}
};

const upm_data_t pico_single_write =
{ 	0x10000018, 	/* offset 0x18 */
	0xD0000000,
	3,
	{ 0x0FFFFC00,
	  0x0FFFFC00,
	  0x00FFFC05
	}
};

const upm_data_t pico_burst_write =
{ 	0x10000020, 	/* offset 0x20 */
	0xD0000000,
	9,
	{ 0x0FFFFC00,
	  0x00FFFC04,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0C,
	  0x0FFCFC0D
	}
};

int board_early_init_f (void)
{
	return 0;
}

void reset_phy (void)
{
	/* PHY reset is connected to hardware reset on this target */
}


int checkboard (void)
{
	puts("Board: picoChip HDP203\n");

#ifdef CONFIG_PCI
	printf("PCI1:  32 bit, %d MHz (compiled)\n",
	       CONFIG_SYS_CLK_FREQ / 1000000);
#else
	printf("PCI1:  disabled\n");
#endif

	/*
	 * Initialize local bus.
	 */
	local_bus_init();

	/* Check picoChip status 
	 */
	pico_init();

	return 0;
}


phys_size_t
initdram(int board_type)
{
	long dram_size = 0;
	extern long spd_sdram (void);

	puts("Initializing...\n");

#if defined(CONFIG_DDR_DLL)
	{
	    volatile ccsr_gur_t *gur= (void *)(CFG_MPC85xx_GUTS_ADDR);
	    uint temp_ddrdll = 0;

	    /*
	     * Work around to stabilize DDR DLL
	     */
	    temp_ddrdll = gur->ddrdllcr;
	    gur->ddrdllcr = ((temp_ddrdll & 0xff) << 16) | 0x80000000;
	    asm("sync;isync;msync");
	}
#endif

#if defined(CONFIG_SPD_EEPROM)
        {
           dram_size = spd_sdram ();
        }
#else
	{
           dram_size = fixed_sdram ();
        } 
#endif

#if defined(CONFIG_DDR_ECC)
	/*
	 * Initialize and enable DDR ECC.
	 */
	ddr_enable_ecc(dram_size);
#endif

	puts("DDR:   ");
	return dram_size;
}

void program_upm_a(const upm_data_t const* upm)
{
	volatile ccsr_lbc_t *lbc = (void *)(CFG_MPC85xx_LBC_ADDR);

	uint i;

	/* Set base of UPM region */
	lbc->mamr = upm->mode;
	for(i = 0; i < upm->entries; i++) {

		/* Data to write to UPM */
		lbc->mdr = upm->data[i];

		/* Dummy write to region commits data */
		*(volatile uint*)(upm->dummy) = 0x0;

		/* Force sync between writes */
		asm("sync;isync;msync");
	}

	/* Configure for normal operation */


	/* Old : lbc->mamr = upm->mode & ~0x30000000; */

	lbc->mamr = 0x08000000; 	/* Setting mamr to 0x8000000 does the biz in Lauterbach scripts */ 

}

/*
 * Initialize Local Bus
 */
void
local_bus_init(void)
{
	volatile ccsr_gur_t *gur = (void *)(CFG_MPC85xx_GUTS_ADDR);
	volatile ccsr_lbc_t *lbc = (void *)(CFG_MPC85xx_LBC_ADDR);

	uint clkdiv;
	uint lbc_hz;
	sys_info_t sysinfo;

	/*
	 * Errata LBC11.
	 * Fix Local Bus clock glitch when DLL is enabled.
	 *
	 * If localbus freq is < 66Mhz, DLL bypass mode must be used.
	 * If localbus freq is > 133Mhz, DLL can be safely enabled.
	 * Between 66 and 133, the DLL is enabled with an override workaround.
	 */

	get_sys_info(&sysinfo);
	clkdiv = lbc->lcrr & 0x0f;
	lbc_hz = sysinfo.freqSystemBus / 1000000 / clkdiv;

	if (lbc_hz < 66) {
		lbc->lcrr = CFG_LBC_LCRR | 0x80000000;	/* DLL Bypass */

	} else if (lbc_hz >= 133) {
		lbc->lcrr = CFG_LBC_LCRR & (~0x80000000); /* DLL Enabled */

	} else {
		/*
		 * On REV1 boards, need to change CLKDIV before enable DLL.
		 * Default CLKDIV is 8, change it to 4 temporarily.
		 */
		uint pvr = get_pvr();
		uint temp_lbcdll = 0;

		if (pvr == PVR_85xx_REV1) {
			/* FIXME: Justify the high bit here. */
			lbc->lcrr = 0x10000004;
		}

		lbc->lcrr = CFG_LBC_LCRR & (~0x80000000);/* DLL Enabled */
		udelay(200);

		/*
		 * Sample LBC DLL ctrl reg, upshift it to set the
		 * override bits.
		 */
		temp_lbcdll = gur->lbcdllcr;
		gur->lbcdllcr = (((temp_lbcdll & 0xff) << 16) | 0x80000000);
		asm("sync;isync;msync");
	}

	lbc->lbcr = CFG_LBC_LBCR;
	asm("msync");

	lbc->lsrt = CFG_LBC_LSRT;
	lbc->mrtpr = CFG_LBC_MRTPR;
	asm("sync");

	/* Most of the local bus control registers are set by the CPU
	 * section, but for some reason OR2 is not programmed for
	 * an MPC85xx - probably something to do with the ADS.
	 * We do it here to minimise patches.
	 */

	lbc->br2 = CFG_BR2_PRELIM;
	lbc->or2 = CFG_OR2_PRELIM;

	/* Now set up the UPM for picoChip */

	program_upm_a(&pico_single_read);
	program_upm_a(&pico_burst_read);
	program_upm_a(&pico_single_write);
	program_upm_a(&pico_burst_write);

}


#if defined(CFG_DRAM_TEST)
int testdram (void)
{
	uint *pstart = (uint *) CFG_MEMTEST_START;
	uint *pend = (uint *) CFG_MEMTEST_END;
	uint *p;

	printf("SDRAM test phase 1:\n");
	for (p = pstart; p < pend; p++)
		*p = 0xaaaaaaaa;

	for (p = pstart; p < pend; p++) {
		if (*p != 0xaaaaaaaa) {
			printf ("SDRAM test fails at: %08x\n", (uint) p);
			return 1;
		}
	}

	printf("SDRAM test phase 2:\n");
	for (p = pstart; p < pend; p++)
		*p = 0x55555555;

	for (p = pstart; p < pend; p++) {
		if (*p != 0x55555555) {
			printf ("SDRAM test fails at: %08x\n", (uint) p);
			return 1;
		}
	}

	printf("SDRAM test passed.\n");
	return 0;
}
#endif


#if !defined(CONFIG_SPD_EEPROM)
/*************************************************************************
 *  fixed sdram init -- doesn't use serial presence detect.
 ************************************************************************/
long int fixed_sdram (void)
{
	volatile ccsr_ddr_t *ddr= (void *)(CFG_MPC85xx_DDR_ADDR);
	volatile ccsr_local_ecm_t *ecm = (void *)(CFG_MPC85xx_ECM_ADDR);

    #ifndef CFG_RAMBOOT
	ddr->cs0_bnds = CFG_DDR_CS0_BNDS;
	ddr->cs0_config = CFG_DDR_CS0_CONFIG;
	ddr->timing_cfg_1 = CFG_DDR_TIMING_1;
	ddr->timing_cfg_2 = CFG_DDR_TIMING_2;
	ddr->sdram_mode = CFG_DDR_MODE;
	ddr->sdram_interval = CFG_DDR_INTERVAL;
    #if defined (CONFIG_DDR_ECC)
	ddr->err_disable = 0x0000000D;
	ddr->err_sbe = 0x00ff0000;
    #endif
	asm("sync;isync;msync");
	udelay(500);
    #if defined (CONFIG_DDR_ECC)
	/* Enable ECC checking */
	ddr->sdram_cfg = (CFG_DDR_CONTROL | 0x20000000);
    #else
	/* Set the "start" bits last */
	ddr->sdram_cfg = CFG_DDR_CONTROL & ~0xC0000000;
	asm("sync; isync; msync");
	ddr->sdram_cfg = CFG_DDR_CONTROL;
    #endif
	asm("sync; isync; msync");
	udelay(500);
  #endif /* CFG_RAMBOOT */

	/* The SPD mode automatically enables the LAWBAR register
	 * associated with the DDR bank in LAWBAR1 according to the
	 * detected type.  We must do this explicitly here.
	 */
	ecm->lawar1 |= LAWAR_EN;

	return CFG_SDRAM_SIZE * 1024 * 1024;
}
#endif	/* !defined(CONFIG_SPD_EEPROM) */

#if defined(CONFIG_PCI)
/*
 * Initialize PCI Devices, report devices found.
 */

#ifndef CONFIG_PCI_PNP
static struct pci_config_table pci_picohdp_config_table[] = {
    { PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID,
      PCI_IDSEL_NUMBER, PCI_ANY_ID,
      pci_cfgfunc_config_device, { PCI_ENET0_IOADDR,
				   PCI_ENET0_MEMADDR,
				   PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER
      } },
    { }
};
#endif


static struct pci_controller hose = {
#ifndef CONFIG_PCI_PNP
	config_table: pci_picohdp_config_table,
#endif
};

#endif	/* CONFIG_PCI */


void
pci_init_board(void)
{
#ifdef CONFIG_PCI
	extern void pci_mpc85xx_init(struct pci_controller *hose);

	pci_mpc85xx_init(&hose);
#endif /* CONFIG_PCI */
}

uint pico_read_ident(int id)
{
	uchar* picobase = (uchar*)(CFG_BR2_PRELIM & 0xFFFF0000);
	volatile uint* picoget;
	volatile uint* picoset;
	uint data, timeout;

	switch(id)
	{
		case 0:
			picoget = (volatile uint*)(picobase + 0x0078);
			picoset = (volatile uint*)(picobase + 0x007C);
			break;

		case 1: 
			picoget = (volatile uint*)(picobase + 0x4078);
			picoset = (volatile uint*)(picobase + 0x407C);
			break;
                
                case 2:                
			picoget = (volatile uint*)(picobase + 0x8078);
			picoset = (volatile uint*)(picobase + 0x807C);
			break;
                
                case 3: 
			picoget = (volatile uint*)(picobase + 0xC078);
			picoset = (volatile uint*)(picobase + 0xC07C);
			break;
                                        
		default:
			printf("Pico: No such device\n");
			return 0;
	}

	/* Make sure there is no old read data in the pipeline */
	data = *picoget;	
 	__asm__ __volatile__ ("eieio");
	data = *picoget;	
 	__asm__ __volatile__ ("eieio");

	*picoset = 0x00080048;
 	__asm__ __volatile__ ("eieio");
	*picoset = 0x00040030;
 	__asm__ __volatile__ ("eieio");
	*picoset = 0x00020000;
 	__asm__ __volatile__ ("eieio");


	data = *picoget;	
	__asm__ __volatile__ ("eieio");
	timeout = 10;
	while ((timeout > 0) && (data & 0x00010000) == 0x00000000) {
		timeout--;
		udelay(1);
		data = *picoget;	
		__asm__ __volatile__ ("eieio");
	}
	

	return data;
}

/* This resets both picoArray devices via the CPLD */
void pico_reset(void)
{
	uchar* cpld_reset = (uchar*)(CFG_BR1_PRELIM & 0xFFFF0000) + 0x06;
                             
        *(volatile unsigned short*)cpld_reset = 0x001F;	/* assert picoArray reset signals */
                
        udelay (1000);  /* wait a bit... */
        
        *(volatile unsigned short*)cpld_reset = 0xF01F; /* negate picoArray reset signals */
}


/* This function reports the cpld version */
void pico_report_cpld_version(void)
{
	uchar* cpld_version = (uchar*)(CFG_BR1_PRELIM & 0xFFFF0000) + 0x00;

        unsigned short cpldVersion;
        uchar cpldVersionTens, cpldVersionUnits;
                       
        /* Read the cpld version */
        cpldVersion = *(volatile unsigned short*)cpld_version;

        cpldVersionTens = (cpldVersion & 0x00F0) >> 4;
        cpldVersionUnits = (cpldVersion & 0x000F);
        
        printf("CPLD:  Version %01X.%01X\n", cpldVersionTens, cpldVersionUnits);
}

void pico_init(void)
{
	uint deva, devb, devc, devd;

        /* Report the CPLD version during u-boot bootup */
        pico_report_cpld_version();	
        
        /* Ensure the picoArrays are reset */
        pico_reset();

	/* Wait after reset */
	udelay(1000);

	/* Read the device idents */
	deva = pico_read_ident(0);
	devb = pico_read_ident(1);
        devc = pico_read_ident(2);
        devd = pico_read_ident(3);

	printf("picoChip: PC203 ");

	if (deva & 0x00010000) {
		printf("Device A Id %04X ", deva & 0x0000FFFF);
	}

	if (devb & 0x00010000) {
		printf("Device B Id %04X ", devb & 0x0000FFFF);
	}

        if (devc & 0x00010000) {
		printf("Device C Id %04X ", devb & 0x0000FFFF);
	}
        
        if (devd & 0x00010000) {
		printf("Device D Id %04X ", devb & 0x0000FFFF);
	}
         
	if ( !(deva & 0x00010000) && !(devb & 0x00010000)  && !(devc & 0x00010000)  && !(devd & 0x00010000) )
	{
		printf("No devices found!");
	}

	printf("\n");
}
