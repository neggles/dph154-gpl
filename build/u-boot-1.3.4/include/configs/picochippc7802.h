/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file picochippc7802.h
* \brief Configuration file for U-Boot on the PC7802 platform.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/pc20x.h>
#include <asm/arch/sizes.h>
#include <asm/arch/uart.h>

/*-----------------------------------------------------------------------------
 * Platform Identification Stuff
 */
#define PICOCHIP "picochip"

/* Which hardware platform I am destined for */
#define PICOCHIP_PLATFORM "pc7802"

/* Specific version of this build */
#ifndef PICOCHIP_PLATFORM_VERSION
#define PICOCHIP_PLATFORM_VERSION "3.2.4"
#endif /* PICOCHIP_PLATFORM_VERSION */

#define CONFIG_IDENT_STRING " "PICOCHIP"-"PICOCHIP_PLATFORM_VERSION \
                            "-"PICOCHIP_PLATFORM

/*-----------------------------------------------------------------------------
 * High Level Configuration Options
 */
/* Running on picoChip PC20x Si */
#define CONFIG_PICOCHIP_PC20X

/* Running on a picoChip PC7802 platform */
#define CONFIG_PICOCHIP_PC7802

/* Define this if you want to run with 2 Banks DDR Ram */
#define CONFIG_PC20X_2_DDR_RAM_BANKS

/* Define this if running code in PC20X rtl simulation land */
#undef CONFIG_PC20X_SIMULATION

/* Bootable Flash memory has to live here (/ebi_decode0) */
#define PC20X_BOOTABLE_FLASH_BASE   (0x20000000)

/* Base address of the onchip SRAM */
#define PC20X_ONCHIP_SRAM_BASE      (0x10000000)
#define PC20X_ONCHIP_SRAM_SIZE      (SZ_128K)

/* ARM Sub-system peripherals are clocked at 140 MHz Rev B Si */
#define PC20X_AHB_CLOCK_FREQ        (140000000)

/* Don't use Interrupts */
#undef CONFIG_USE_IRQ

/* Onchip timer runs at this frequency */
#define CFG_HZ			    (PC20X_AHB_CLOCK_FREQ)
#define CONFIG_SYS_HZ               (CFG_HZ)

/* Display board info */
#define CONFIG_DISPLAY_BOARDINFO    (1)

/* Are we are going to be running from RAM ? */
#ifdef CONFIG_RUN_FROM_RAM
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_SKIP_RELOCATE_UBOOT
#endif /* CONFIG_RUN_FROM_RAM */

/*-----------------------------------------------------------------------
 * Stack Sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(SZ_256K)	    /* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(SZ_1K)	            /* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(SZ_1K)	            /* FIQ stack */
#endif /* CONFIG_USE_IRQ */

/*-----------------------------------------------------------------------------
 * Size of malloc() pool
 */
#define CFG_MALLOC_LEN		(SZ_128K)

 /* Size in bytes reserved for initial data */
#define CFG_GBL_DATA_SIZE	(SZ_256)

/*-----------------------------------------------------------------------------
 * Linux Kernel Stuff
 */
/* Allow passing of command line args (bootargs) to the linux kernel*/
#define CONFIG_CMDLINE_TAG

/*-----------------------------------------------------------------------------
 * DDR2 RAM Memory Map
 */
#ifndef CONFIG_PC20X_2_DDR_RAM_BANKS
/* We want a 4 DDR Bank setup then */

#define CONFIG_NR_DRAM_BANKS    4               /* We have 4 RAM banks */
#define PHYS_SDRAM_1		0x00000000
#define PHYS_SDRAM_1_SIZE	SZ_32M          /* 32 Mbytes */
#define PHYS_SDRAM_2		0x04000000
#define PHYS_SDRAM_2_SIZE	SZ_32M          /* 32 Mbytes */
#define PHYS_SDRAM_3		0x08000000
#define PHYS_SDRAM_3_SIZE	SZ_32M          /* 32 Mbytes */
#define PHYS_SDRAM_4		0x0C000000
#define PHYS_SDRAM_4_SIZE	SZ_32M          /* 32 Mbytes */

#else   /* We want a 2 DDR Bank setup then */

#define CONFIG_NR_DRAM_BANKS    2               /* We have 2 RAM banks */
#define PHYS_SDRAM_1		0x00000000
#define PHYS_SDRAM_1_SIZE	SZ_32M          /* 32 Mbytes */
#define PHYS_SDRAM_2		0x04000000
#define PHYS_SDRAM_2_SIZE	SZ_32M          /* 32 Mbytes */

#endif /* CONFIG_PC20X_2_DDR_RAM_BANKS */

/*-----------------------------------------------------------------------------
 * Flash Memory Stuff
 */
#ifndef CONFIG_RUN_FROM_RAM

/* Define Flash memory sector size
  256 kbytes (2 off Flash devices with 128K sector size) */
#define FLASH_SECTOR_SIZE	(SZ_256K)

#define CFG_FLASH_CFI
#define CFG_FLASH_CFI_DRIVER
#define CFG_FLASH_EMPTY_INFO

/* Provide a much improved performance when writing to the Flash */
#define CFG_FLASH_USE_BUFFER_WRITE

#define CFG_FLASH_BASE		(PC20X_BOOTABLE_FLASH_BASE)

/* Maximum number of memory banks */
#define CFG_MAX_FLASH_BANKS	(1)

/* Maximum number of sectors per flash device */
#define CFG_MAX_FLASH_SECT	(512)

/* Timeouts for Flash Erasing and writing */
#define CFG_FLASH_ERASE_TOUT	(2*CFG_HZ)
#define CFG_FLASH_WRITE_TOUT	(2*CFG_HZ)

/* U-Boot has its own Flash sector */
#define CFG_MONITOR_LEN		(FLASH_SECTOR_SIZE)

/* U-Boot lives in the bottom of the Flash memory */
#define CFG_MONITOR_BASE        (CFG_FLASH_BASE)

#else

/* No flash memory in the system */
#define CFG_NO_FLASH

#endif /* CONFIG_RUN_FROM_RAM */

/*-----------------------------------------------------------------------------
 * U-Boot Environment Stuff
 */
#ifndef CONFIG_RUN_FROM_RAM

/* Environment variables stored in Flash memory */
#define CFG_ENV_IS_IN_FLASH     (1)
#define CFG_ENV_ADDR            (PC20X_BOOTABLE_FLASH_BASE + FLASH_SECTOR_SIZE)

/* One flash sector for environment info */
#define CFG_ENV_SECT_SIZE       (FLASH_SECTOR_SIZE)

/* Turn off wite protection for vendor parameters */
#define CONFIG_ENV_OVERWRITE

#else

/* Just use the default environment in the image */
#define CFG_ENV_IS_NOWHERE

#endif /* CONFIG_RUN_FROM_RAM */

/* Where ever the environment is, 64Kbytes should be sufficient */
#define CFG_ENV_SIZE		(SZ_64K)

/*-----------------------------------------------------------------------------
 * Timer Stuff
 */
#define CFG_TIMERBASE           (PC20X_TIMER_BASE)

/*-----------------------------------------------------------------------------
 * Ethernet Stuff
 */
#define CFG_DW_EMAC
#define CONFIG_PHY_ADDR         (1)
#define CONFIG_NET_MULTI

/*-----------------------------------------------------------------------------
 * Serial Port Stuff
 */
#define CFG_DW_APB_UART

/* Baud rate generators clock with a 3.6864 MHz clock */
#define CONFIG_DW_APB_UART_CLOCK    (3686400)

/* Console on Uart #0 */
#define CONFIG_CONS_INDEX	    (1)
#define CFG_BAUDRATE_TABLE	    { 9600, 19200, 38400, 57600, 115200, 230400 }

/*-----------------------------------------------------------------------------
 * U-Boot Memory Test (mtest command) Stuff
 */
/* Default start address for memory test */
#define CFG_MEMTEST_START	(PC20X_ONCHIP_SRAM_BASE)

/* Default end address for memory test */
#define CFG_MEMTEST_END		(CFG_MEMTEST_START + PC20X_ONCHIP_SRAM_SIZE - 1)

/* Define this to use the super duper memory test */
#define CFG_ALT_MEMTEST

/* Use uart #1 scratch pad reg */
#define CFG_MEMTEST_SCRATCH     (PC20X_UART1_BASE + UartScratchRegOffset)

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

/* Use the HUSH parser */
#define CFG_HUSH_PARSER

#ifdef  CFG_HUSH_PARSER
/* This defines the secondary prompt string */
#define CFG_PROMPT_HUSH_PS2 "> "
#endif /* CFG_HUSH_PARSER */

/* Enable command line editing and history */
#define CONFIG_CMDLINE_EDITING

/*-----------------------------------------------------------------------------
 * Miscellaneous Configurable Options...
 */
/* Use 'long' help messages */
#define CFG_LONGHELP

/* Monitor Command Prompt */
#define CFG_PROMPT	"=> "

/* Console I/O Buffer Size*/
#define CFG_CBSIZE	(SZ_1K)

/* Print buffer size */
#define CFG_PBSIZE	(CFG_CBSIZE+sizeof(CFG_PROMPT)+16)

/* Maximum number of command args */
#define CFG_MAXARGS	(16)

/* Boot Argument Buffer Size*/
#define CFG_BARGSIZE	(CFG_CBSIZE)

/* everything, incl board info, in Hz */
#undef	CFG_CLKS_IN_HZ

/* Default load address for bootm and friends */
#define CFG_LOAD_ADDR	(0x20080000)

/*-----------------------------------------------------------------------
 * Environment Configuration
 */

/* Note: This MAC address is based on the real picoChip OUI, it
 * will need to be modified (as per the picoChip MAC address creation spec)
 * for each and every board.
 */

#if defined(CFG_DW_EMAC)

/* picoChip OUI, will need noodling by users */
#define CONFIG_ETHADDR          00:15:E1:00:00:00

/* picoChip default for testing, will need noodling by users */
#define CONFIG_IPADDR           172.17.14.248

#define CONFIG_HOSTNAME	        picopc7802
#define CONFIG_ROOTPATH	        /var/nfspc7802
#define CONFIG_BOOTFILE	        uImage-pc7802

#define CONFIG_SERVERIP         172.17.7.100
#define CONFIG_GATEWAYIP        172.17.0.1
#define CONFIG_NETMASK          255.255.0.0

#endif

/* Second flash sector... */
#define CFG_FLASH_KERNEL_BASE   0x20080000

/* Default location for tftp and bootm */
#define CONFIG_LOADADDR         0x00200000

/* Time in seconds before autoboot, -1 disables auto-boot */
#define CONFIG_BOOTDELAY        5

/* The boot command will set bootargs */
#undef  CONFIG_BOOTARGS

/* Default console baud rate */
#define CONFIG_BAUDRATE	        115200

#ifndef CONFIG_PC20X_2_DDR_RAM_BANKS
/* We want a 4 DDR Bank setup then */

/* Unless specified here we'll just rely on the kernel default */
#define OTHERBOOTARGS

#else   /* We want a 2 DDR Bank setup then */

/* We need to 'over ride' the kernel default */
#define OTHERBOOTARGS pc20x_mem=32M@PHYS_SDRAM_1 pc20x_mem=32M@PHYS_SDRAM_2

#endif /* CONFIG_PC20X_2_DDR_RAM_BANKS */

#define	CONFIG_EXTRA_ENV_SETTINGS				        \
   "othbootargs=" MK_STR (OTHERBOOTARGS) "\0"                           \
   "netdev=eth0\0"                                                      \
   "consoledev=ttyS0\0"                                                 \
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

#define CONFIG_RAMBOOTCOMMAND                                           \
   "setenv bootargs root=/dev/ram rw "                                  \
   "console=$consoledev,$baudrate $othbootargs;"                        \
   "tftp $ramdiskaddr $ramdiskfile;"                                    \
   "tftp $loadaddr $bootfile;"                                          \
   "bootm $loadaddr $ramdiskaddr"

/* Define as "run flash_jffs2" for on-board boot, or "run fixed_nfs" for
 * standard NFS with fixed IP.  Or use NFSBOOTCOMMAND etc as above.
 */
#define CONFIG_BOOTCOMMAND  "run flash_jffs2"

#endif /* __CONFIG_H */
