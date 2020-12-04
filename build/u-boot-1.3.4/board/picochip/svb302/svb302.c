/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file svb302.c
* \brief Various useful functions for use on an SVB302 Platform.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

/* Includes ---------------------------------------------------------------- */
#include <common.h>
#include <asm/arch/pc302.h>
#include <asm/arch/timer.h>
#include <asm/arch/axi2cfg.h>

/* Macros ------------------------------------------------------------------ */
#define SIXTEEN_BIT_MASK    0xFFFF

/* Constants --------------------------------------------------------------- */
DECLARE_GLOBAL_DATA_PTR;

enum pc302DeviceType { PC302_FAT_DEVICE = 3,
                       PC302_NORMAL_DEVICE
                     };

/* Prototypes--------------------------------------------------------------- */
/*!
 *
 * Start timer #0 in free running mode
 *
 */
static void pc302Timer0Start(void);

/*!
 *
 * Read the device identification code from the axi2cfg block
 *
 * \return The value read
 */
static unsigned int readDeviceId (void);

/*!
 *
 * Read the device revision code from the axi2cfg block
 *
 * \return The value read
 */
static unsigned int readDeviceRevision (void);

/*!
 *
 * Read an integer value from a register.
 *
 * \param The adrress to read from
 * \return The value read
 */
static __inline unsigned int readFromRegister (const unsigned int address);

/* Functions --------------------------------------------------------------- */

/*****************************************************************************
 *
 * show_boot_progress()
 *
 * Purpose: Indicate booting progress
 *
 * Note: see U-Boot README for a list of 'progress' values.
 *
 *****************************************************************************/
#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
	printf("Boot reached stage %d\n", progress);
}
#endif

/*****************************************************************************
 *
 * board_init()
 *
 * Purpose: Hardware platform initialisation functions
 *
 * Returns: 0 - Success
 *
 *****************************************************************************/
int board_init (void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = 0x00000100;
        gd->bd->bi_arch_number = MACH_TYPE_PC7302;
	gd->flags = 0;

	/* Enable the Instruction Cache */
        icache_enable ();

        /* Start a timer */
        pc302Timer0Start();

	return 0;
}

/*****************************************************************************
 *
 * checkboard()
 *
 * Purpose: Display some useful hardware platform information.
 *
 * Returns: 0 - Success
 *
 *****************************************************************************/
int checkboard (void)
{
    unsigned int deviceType, revision;

    puts("Build: picoChip "PICOCHIP_PLATFORM" \n");

    /* What device are we running on ? */
    puts("Device: ");

    deviceType = readDeviceId();        /* Read the device Id */
    revision = readDeviceRevision();    /* Read the revision code */

    switch (deviceType)
    {
        case PC302_FAT_DEVICE:
        {
            printf("PC302 (Fat) Rev %04d\n", revision);
            break;
        }
        case PC302_NORMAL_DEVICE:
        {
            printf("PC302 Rev %04d\n", revision);
            break;
        }
        default:
        {
            printf("Unknown !\n");
        }
    }

    return 0;
}

/*****************************************************************************
 *
 * misc_init_r()
 *
 * Purpose: Miscellaneous platform dependent initialisations
 *
 * Returns: 0 - Success
 *
 *****************************************************************************/
int misc_init_r (void)
{
    /* Not used right now, function template left here as a place holder */
    return 0;
}

/*****************************************************************************
 *
 * dram_init()
 *
 * Purpose: Initialize the DDR SDRAM info in the board data structure
 *
 * Returns: 0 - Success
 *
 *****************************************************************************/
int dram_init (void)
{

    gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
    gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

    return 0;
}

static void pc302Timer0Start(void)
{
    *(volatile unsigned int *)(CFG_TIMERBASE + TIMERNCONTROLREGOFFSET(0)) =
                              (TIMERINTERRUPTMASK | TIMERENABLE);
}

static unsigned int readDeviceId (void)
{
    unsigned int deviceId;

    deviceId = readFromRegister (PC302_AXI2CFG_BASE +
                                 AXI2CFG_DEVICE_ID_REG_OFFSET);
    deviceId &= SIXTEEN_BIT_MASK;

    return deviceId;
}

static unsigned int readDeviceRevision (void)
{
    unsigned int revisionCode;

    revisionCode = readFromRegister (PC302_AXI2CFG_BASE +
                                     AXI2CFG_REVISION_ID_REG_OFFSET);
    revisionCode &= SIXTEEN_BIT_MASK;

    return revisionCode;
}

static __inline unsigned int readFromRegister (const unsigned int address)
{
    /* Read an integer value from a register */

    return(*(volatile unsigned int *)address);
}
