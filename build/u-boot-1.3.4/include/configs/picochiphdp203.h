/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Configuration for picoChip HDP203 target (www.picochip.com)
 *     by david.warman@zetetica.co.uk
 * 
 * Copyright 2005 Zetetica Ltd.
 * 
 * parts derived from MPC8560 ADS configuration,
 *     Copyright 2004 Freescale Semiconductor.
 *     (C) Copyright 2002,2003 Motorola,Inc.
 *     Xianghua Xiao <X.Xiao@motorola.com>
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
 */

/*
 * picoChip HDP203 board.  If you have one of these, you'll know what it is.
 *
 * Make sure you change the MAC address and other network params first,
 * search for CONFIG_ETHADDR, CONFIG_SERVERIP, etc in this file.
 *
 * Revision April 13, 2007 - stuartr
 *  	CFG_BR2_PRELIM, CFG_OR2_PRELIM, CFG_BR3_PRELIM and CFG_OR3_PRELIM
 *	register settings revised to use UPMA instead of GPCM local bus accesses 
 * 
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*-----------------------------------------------------------------------------
 * Platform Identification Stuff
 */
#define PICOCHIP "picochip"

/* Which hardware platform I am destined for */
#define PICOCHIP_PLATFORM "hdp203"

/* Specific version of this build */
#ifndef PICOCHIP_PLATFORM_VERSION
#define PICOCHIP_PLATFORM_VERSION "3.2.4"
#endif /* PICOCHIP_PLATFORM_VERSION */

#define CONFIG_IDENT_STRING " "PICOCHIP"-"PICOCHIP_PLATFORM_VERSION \
                            "-"PICOCHIP_PLATFORM

/*-----------------------------------------------------------------------------
 * High Level Configuration Options
 */
#define CONFIG_BOOKE		1	/* BOOKE */
#define CONFIG_E500		1	/* BOOKE e500 family */
#define CONFIG_MPC85xx		1	/* MPC8540/MPC8560 */
#define CONFIG_CPM2		1	/* has CPM2 */
#define CONFIG_PICOHDP	    	1	/* Board specific symbol */
#define CONFIG_PCI                      /* Include PCI support */
#define CONFIG_TSEC_ENET 		/* tsec ethernet support */
#undef  CONFIG_ETHER_ON_FCC             /* cpm FCC ethernet support */

#define CONFIG_SPD_EEPROM		/* Use SPD EEPROM for DDR setup*/
#define CONFIG_DDR_DLL			/* possible DLL fix needed */
#define CONFIG_DDR_2T_TIMING		/* Sets the 2T timing bit */
//#define CONFIG_DDR_ECC			/* only for ECC DDR module */
#define CONFIG_MEM_INIT_VALUE		0xDeadBeef

#define CONFIG_FSL_LAW          1       /* Use common FSL init code */

/* sysclk for MPC85xx -- 66.67MHz on this target */
#ifndef CONFIG_SYS_CLK_FREQ
#define CONFIG_SYS_CLK_FREQ	66670000
#endif

/* These can be toggled for performance analysis, otherwise use default */
#define CONFIG_L2_CACHE			/* toggle L2 cache */
#define CONFIG_BTB			/* toggle branch predition */
#define CONFIG_ADDR_STREAMING		/* toggle addr streaming */

/* Not used at present, but may need it later */
/* Used to call the platform specific board_early_init_f() */
#define CONFIG_BOARD_EARLY_INIT_F	1	

#define CFG_INIT_DBCR DBCR_IDM		/* Enable Debug Exceptions */

#undef	CFG_DRAM_TEST			/* memory test, takes time */
#define CFG_MEMTEST_START	0x00200000	/* memtest region */
#define CFG_MEMTEST_END		0x00400000

/* Base addresses -- Note these are effective addresses where the
   actual resources get mapped (not physical addresses) */
#define CFG_CCSRBAR_DEFAULT 	0xFF700000	/* CCSRBAR Default */
#define CFG_CCSRBAR		0x80000000	/* relocated CCSRBAR */
#define CFG_CCSRBAR_PHYS	CFG_CCSRBAR	/* physical addr of CCSRBAR */ 
#define CFG_IMMR		CFG_CCSRBAR	/* PQII uses CFG_IMMR */

#define CFG_CPLD_MEM_BASE	0xE0000000
#define CFG_PREG_MEM_BASE	0xD0000000
#define CFG_PFIFO_MEM_BASE	0xC0000000
#define CFG_DCARD_MEM_BASE	0xB0000000

/* DDR Setup */
#define CFG_DDR_SDRAM_BASE	0x00000000	/* DDR is system memory*/
#define CFG_SDRAM_BASE		CFG_DDR_SDRAM_BASE

#if defined(CONFIG_SPD_EEPROM)
    /* Determine DDR configuration from I2C interface. */
    #define SPD_EEPROM_ADDRESS	0x50		/* DDR DIMM */
#else
    /* Manually set up DDR parameters */
    #define CFG_SDRAM_SIZE	256		/* DDR is 256MBytes */
    #define CFG_DDR_CS0_BNDS	0x0000000F	/* 0-256MB */
    #define CFG_DDR_CS0_CONFIG	0x80000102
    #define CFG_DDR_TIMING_1	0x36332321	/* See notes */
    #define CFG_DDR_TIMING_2	0x00000C00	/* See notes */
    #define CFG_DDR_CONTROL	0xC2008000	/* unbuffered,no DYN_PWR */
    #define CFG_DDR_MODE	0x00000022	/* SDMODE=63  */
    #define CFG_DDR_INTERVAL	0x04110100	/* See notes */
#endif /* CONFIG_SPD_EEPROM */

#undef CONFIG_WATCHDOG			        /* watchdog disabled */

/*-----------------------------------------------------------------------------
 * Flash Memory Stuff
 */
#ifdef CFG_PICOHDP_FLASH32M
#define CFG_BR0_PRELIM		0xFE001801	/* port size 32bit */
#define CFG_OR0_PRELIM		0xFE000E62	/* 32MB Flash */
#define CFG_FLASH_BASE		0xFE000000	/* start of FLASH 32M */
#else
/* Note: in 128M mode we rely on the upper part of the flash not
 * moving when we change the mapping to allow us access to the entire
 * device.  
 */
#define CFG_BR0_PRELIM		0xF8001801	/* port size 32bit */
#define CFG_OR0_PRELIM		0xF8000E62	/* 128MB Flash */
#define CFG_FLASH_BASE		0xF8000000	/* start of FLASH 128M */
#endif /* CFG_PICOHDP_FLASH32M */

/* Maximum number of memory banks */
#define CFG_MAX_FLASH_BANKS	1

/* Maximum number of sectors per flash device */
#define CFG_MAX_FLASH_SECT	512
#undef	CFG_FLASH_CHECKSUM
#define CFG_FLASH_ERASE_TOUT	40000	/* Flash Erase Timeout (ms) */
#define CFG_FLASH_WRITE_TOUT	500	/* Flash Write Timeout (ms) */

#define CFG_MONITOR_BASE    	TEXT_BASE	/* start of monitor */
#define CFG_FLASH_KERNEL_BASE	0xFFE00000	/* start of kernel */

#if (CFG_MONITOR_BASE < CFG_FLASH_BASE)
#define CFG_RAMBOOT
#else
#undef  CFG_RAMBOOT
/* If not doing a ram boot, set up for Flash */
#define CFG_FLASH_CFI
#define CFG_FLASH_CFI_DRIVER
#define CFG_FLASH_EMPTY_INFO
#endif

/* Provide a much improved performance when writing to the Flash */
#define CFG_FLASH_USE_BUFFER_WRITE

/*-----------------------------------------------------------------------------
 * Local Bus Definitions
 */
/*
 * CS0 - Flash (above)
 * CS1 - CPLD (LEDs, etc)
 * CS2 - picoArray GPR regs
 * CS3 - picoArray FIFOs
 * CS4 - picoArray AHB
 *
 * SCS1 - SDRAM Bank 1
 * SCS0 - SDRAM Bank 0
 */

/* These are actually _final_ mappings, but it avoids a patch to keep the
 * symbol names the same.
 */

/* CPLD mapping on CS1 */
#define CFG_BR1_PRELIM	0xE0001001
#define CFG_OR1_PRELIM	0xFFFF8010

/* PicoChip Registers */
#define CFG_BR2_PRELIM	0xD0001881         /* Use UPMA for GPR register accesses, base addr 0xD0000000, 32bit port, 
       					      error check disabled, r/w accesses, not atomic, bank is valid */
#define CFG_OR2_PRELIM	0xFFFF0010         /* 64k allocated to pA registers, BCTLD=0, BI=1 bursts inhibited  							   
					      TRLX=0 (normal timing), EHTR=0 (no extended hold time)
					      EAD=0 (no additional external ALE cycle) */
					   /* NOTE - Burst accesses inhibited (as per HDP102) */

/* PicoChip FIFOs */
#define CFG_BR3_PRELIM	0xC0001881         /* Use UPMA for FIFO accesses, base addr 0xC0000000 */
#define CFG_OR3_PRELIM	0xFFF00010         /* 1MB allocated to pA Fifos (256k per pA). BCTLD=0, BI=1 bursts inhibited    
 					      TRLX=0 (normal timing), EHTR=0 (no extended hold time)
					      EAD=0 (no additional external ALE cycle) */
					   /* NOTE - Burst accesses inhibited (as per HDP102) */

/* PC203 AHB */
#define CFG_BR4_PRELIM	0xB0001801         /* These settings not verified, AHB accesses untested on HDP203 */
#define CFG_OR4_PRELIM	0xFFFF8010         /* Ditto */

/*-----------------------------------------------------------------------------
 * Local Bus Configuration
 */
#define CFG_LBC_LCRR		0x00030004    /* LB clock ratio reg */
#define CFG_LBC_LBCR		0x00000000    /* LB config reg */
#define CFG_LBC_LSRT		0x20000000    /* LB sdram refresh timer */
#define CFG_LBC_MRTPR		0x20000000    /* LB refresh timer prescal*/

#define CONFIG_L1_INIT_RAM
#define CFG_INIT_RAM_LOCK 	1
#define CFG_INIT_RAM_ADDR	0xA0000000	/* Initial RAM address */
#define CFG_INIT_RAM_END    	0x4000	    	/* End of used area in RAM */

#define CFG_GBL_DATA_SIZE  	128		/* num bytes initial data */
#define CFG_GBL_DATA_OFFSET	(CFG_INIT_RAM_END - CFG_GBL_DATA_SIZE)
#define CFG_INIT_SP_OFFSET	CFG_GBL_DATA_OFFSET

#define CFG_MONITOR_LEN	    	(384 * 1024)    /* Reserve 384 kB for Mon */
#define CFG_MALLOC_LEN	    	(128 * 1024)    /* Reserved for malloc */

/*-----------------------------------------------------------------------------
 * Serial Port
 */
#define CONFIG_CONS_ON_SCC	/* define if console on SCC */
#undef  CONFIG_CONS_NONE	/* define if console on something else */
#define CONFIG_CONS_INDEX       3  /* which serial channel for console */

#define CONFIG_BAUDRATE	 	115200

#define CFG_BAUDRATE_TABLE  \
	{300, 600, 1200, 2400, 4800, 9600, 19200, 38400,115200}

/*-----------------------------------------------------------------------------
 * I2C
 */ 
#define CONFIG_HARD_I2C                 /* I2C with hardware support*/
#define CONFIG_I2C_ON_CPM	        /* Main I2C is on CPM interface */
#undef  CONFIG_SOFT_I2C                 /* I2C bit-banged */
#define CFG_I2C_SPEED           400000	/* I2C speed (Hz) */
#define CFG_I2C_SLAVE           0x7F    /* I2C Slave address */
#define CFG_I2C_NOPROBES        {0x50}	/* Don't probe these addrs */
#define CFG_I2C_OFFSET          0x3000

/*-----------------------------------------------------------------------------
 * General PCI
 * Addresses are mapped 1-1
 */  
#define CFG_PCI1_MEM_BASE	0x40000000
#define CFG_PCI1_MEM_PHYS	CFG_PCI1_MEM_BASE
#define CFG_PCI1_MEM_SIZE	0x10000000	/* 256M */

#define CFG_PCI1_IO_BASE	0x48000000
#define CFG_PCI1_IO_PHYS	CFG_PCI1_IO_BASE
#define CFG_PCI1_IO_SIZE	0x10000000	/* 256M */

#if defined(CONFIG_PCI)

#define CONFIG_NET_MULTI
#define CONFIG_PCI_PNP	               	/* do pci plug-and-play */

#undef CONFIG_EEPRO100
#undef CONFIG_TULIP

#undef CONFIG_PCI_SCAN_SHOW		/* show pci devices on startup */
#define CFG_PCI_SUBSYS_VENDORID 0x1057  /* Motorola */

#endif	/* CONFIG_PCI */

/*-----------------------------------------------------------------------------
 * Ethernet
 */
#if defined(CONFIG_TSEC_ENET)

#ifndef CONFIG_NET_MULTI
#define CONFIG_NET_MULTI 	1
#endif

#define CONFIG_MII		1	/* MII PHY management */
#define CONFIG_TSEC1	        1
#define CONFIG_TSEC1_NAME	"TSEC0"
#define CONFIG_TSEC2	        1
#define CONFIG_TSEC2_NAME	"TSEC1"
#undef CONFIG_MPC85XX_FEC
#define TSEC1_PHY_ADDR		0
#define TSEC2_PHY_ADDR		1
#define TSEC1_PHYIDX		0
#define TSEC2_PHYIDX		0
#define TSEC1_FLAGS             TSEC_GIGABIT
#define TSEC2_FLAGS             TSEC_GIGABIT

/* Options are: TSEC[0-1] */
#define CONFIG_ETHPRIME		"TSEC0"

#elif defined(CONFIG_ETHER_ON_FCC)	/* CPM FCC Ethernet */

#define CONFIG_ETHER_ON_FCC	        /* define if ether on FCC   */
#undef  CONFIG_ETHER_NONE	        /* define if ether on something else */
#define CONFIG_ETHER_INDEX      2       /* which channel for ether */

#if (CONFIG_ETHER_INDEX == 2)
  /*
   * - Rx-CLK is CLK13
   * - Tx-CLK is CLK14
   * - Select bus for bd/buffers
   * - Full duplex
   */
  #define CFG_CMXFCR_MASK       (CMXFCR_FC2 | CMXFCR_RF2CS_MSK | CMXFCR_TF2CS_MSK)
  #define CFG_CMXFCR_VALUE      (CMXFCR_RF2CS_CLK13 | CMXFCR_TF2CS_CLK14)
  #define CFG_CPMFCR_RAMTYPE    0
  #define CFG_FCC_PSMR          (FCC_PSMR_FDE)
  #define FETH2_RST		0x01
#elif (CONFIG_ETHER_INDEX == 3)
  /* need more definitions here for FE3 */
  #define FETH3_RST		0x80
#endif  				/* CONFIG_ETHER_INDEX */

#define CONFIG_MII			/* MII PHY management */
#define CONFIG_BITBANGMII		/* bit-bang MII PHY management */

/*
 * GPIO pins used for bit-banged MII communications (not used at present)
 */
#define MDIO_PORT	2		/* Port C */
#define MDIO_ACTIVE	(iop->pdir |=  0x00400000)
#define MDIO_TRISTATE	(iop->pdir &= ~0x00400000)
#define MDIO_READ	((iop->pdat &  0x00400000) != 0)

#define MDIO(bit)	if(bit) iop->pdat |=  0x00400000; \
			else	iop->pdat &= ~0x00400000

#define MDC(bit)	if(bit) iop->pdat |=  0x00200000; \
			else	iop->pdat &= ~0x00200000

#define MIIDELAY	udelay(1)

#endif

/*-----------------------------------------------------------------------
 * Environment Configuration
 */
 
/* Turn off wite protection for vendor parameters */
#define CONFIG_ENV_OVERWRITE   
 
#ifndef CFG_RAMBOOT
  #define CFG_ENV_IS_IN_FLASH	1
  #define CFG_ENV_ADDR		0xFFF80000  /* First 256K of boot space is env */
  #define CFG_ENV_SECT_SIZE	0x40000	    /* 256K(one sector) for env */
  #define CFG_ENV_SIZE		0x10000     /* But 64K is sufficient */
  /* TEXT_BASE is at 0xFFFC0000 = ENV_ADDR + ENV_SECT_SIZE */
#else
  #define CFG_NO_FLASH		1	    /* Flash is not usable now */
  #define CFG_ENV_IS_NOWHERE	1	    /* Store ENV in memory only */
  #define CFG_ENV_ADDR		(CFG_MONITOR_BASE - 0x2000)
  #define CFG_ENV_SIZE		0x2000
#endif /* CFG_RAMBOOT */

#define CONFIG_LOADS_ECHO	1	/* echo on for serial download */
#define CFG_LOADS_BAUD_CHANGE	1	/* allow baudrate change */

/*-----------------------------------------------------------------------------
 * U-Boot Supported Commands
 */
#include "config_cmd_default.h"

#define CONFIG_CMD_PING
#undef CONFIG_CMD_AUTOSCRIPT
#undef CONFIG_CMD_BOOTD
#undef CONFIG_CMD_CONSOLE
#undef CONFIG_CMD_ECHO
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_LOADB
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_MISC
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_XIMG

#ifdef CFG_NO_FLASH
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_ENV
#endif /* CFG_NO_FLASH */

#define CONFIG_CMD_PCI
#define CONFIG_CMD_I2C
#define CONFIG_CMD_MII
#define CONFIG_CMD_DHCP

/* Use the HUSH parser */
#define CFG_HUSH_PARSER

#ifdef  CFG_HUSH_PARSER
#define CFG_PROMPT_HUSH_PS2 "> "
#endif

/*-----------------------------------------------------------------------------
 * Miscellaneous Configurable Options...
 */
 
/* Use 'long' help messages */
#define CFG_LONGHELP

/* default load address */
#define CFG_LOAD_ADDR	0x1000000

/* Monitor Command Prompt */
#define CFG_PROMPT	"=> "

/* Console I/O Buffer Size*/
#define CFG_CBSIZE	1024

/* Print buffer size */
#define CFG_PBSIZE (CFG_CBSIZE+sizeof(CFG_PROMPT)+16)

/* Maximum number of command args */
#define CFG_MAXARGS	16

/* Boot Argument Buffer Size*/
#define CFG_BARGSIZE	CFG_CBSIZE

/* decrementer freq: 1ms ticks */
#define CFG_HZ		1000

/* clocks NOT passsed to Linux in MHz */
#undef CONFIG_CLOCKS_IN_MHZ

/*
 * For booting Linux, the board info and command line data
 * have to be in the first 8 MB of memory, since this is
 * the maximum mapped by the Linux kernel during initialization.
 */
#define CFG_BOOTMAPSZ	(8 << 20)	/* Initial Memory map for Linux*/

/* Cache Configuration */
#define CFG_DCACHE_SIZE		32768
#define CFG_CACHELINE_SIZE	32
#if (CONFIG_COMMANDS & CFG_CMD_KGDB)
#define CFG_CACHELINE_SHIFT	5	/*log base 2 of the above value*/
#endif

/*
 * Internal Definitions
 *
 * Boot Flags
 */
#define BOOTFLAG_COLD	0x01		/* Normal Power-On: Boot from FLASH */
#define BOOTFLAG_WARM	0x02		/* Software reboot */

/*
 * Environment Configuration
 */

/* Preliminary settings - may need changing, particularly for different
 * NFS arrangements...
 */

/* These MAC addresses are based on the real picoChip OUI, they 
 * will need to be modified (as per the MAC address creation spec) for
 * each and every hdp board.
 */

#if defined(CONFIG_TSEC_ENET) || defined(CONFIG_ETHER_ON_FCC)

#define CONFIG_ETHADDR   00:15:E1:00:00:00
#define CONFIG_HAS_ETH1
#define CONFIG_ETH1ADDR  00:15:E1:00:00:01
/* There are only ever two Ethernets on this target */

#define CONFIG_IPADDR    172.17.10.248

#define CONFIG_HOSTNAME	 picohdp
#define CONFIG_ROOTPATH	 /var/nfshdp203
#define CONFIG_BOOTFILE	 uImage-hdp203

#define CONFIG_SERVERIP  172.17.7.100
#define CONFIG_GATEWAYIP 172.17.0.1
#define CONFIG_NETMASK   255.255.0.0

#endif

#define CONFIG_LOADADDR  200000	/* default location for tftp and bootm */

#define CONFIG_BOOTDELAY 5	/* -1 disables auto-boot */
#undef  CONFIG_BOOTARGS		/* the boot command will set bootargs */

#define CONFIG_BAUDRATE	115200

#define	CONFIG_EXTRA_ENV_SETTINGS				        \
   "netdev=eth0\0"                                                      \
   "consoledev=ttyCPM0\0"                                               \
   "kernel_flash_addr=" MK_STR(CFG_FLASH_KERNEL_BASE) "\0"		\
   "flash_jffs2=run jffs2_args; bootm $kernel_flash_addr\0"		\
   "fixed_nfs=run nfs_args; tftp; bootm\0"				\
   "jffs2_args=setenv bootargs root=/dev/mtdblock1 rw rootfstype=jffs2 " \
   "ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:any "    \
   "console=$consoledev,$baudrate $othbootargs;\0"                      \
   "nfs_args=setenv bootargs root=/dev/nfs rw nfsroot=$serverip:$rootpath " \
   "ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:any "    \
   "console=$consoledev,$baudrate $othbootargs;"                        \

#define CONFIG_NFSBOOTCOMMAND	                                        \
   "setenv bootargs root=/dev/nfs rw "                                  \
   "nfsroot=$serverip:$rootpath "                                       \
   "ip=$ipaddr:$serverip:$gatewayip:$netmask:$hostname:$netdev:any "    \
   "console=$consoledev,$baudrate $othbootargs;"                        \
   "bootm $kernel_flash_addr"

#define CONFIG_RAMBOOTCOMMAND \
   "setenv bootargs root=/dev/ram rw "                                  \
   "console=$consoledev,$baudrate $othbootargs;"                        \
   "tftp $ramdiskaddr $ramdiskfile;"                                    \
   "tftp $loadaddr $bootfile;"                                          \
   "bootm $loadaddr $ramdiskaddr"

/* Define as "run flash_jffs2" for on-board boot, or "run fixed_nfs" for
 * standard NFS with fixed IP.  Or use NFSBOOTCOMMAND etc as above.
 */
#define CONFIG_BOOTCOMMAND  "run flash_jffs2"

#endif	/* __CONFIG_H */
