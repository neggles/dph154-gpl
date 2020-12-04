/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file pc7302.c
* \brief Various useful functions for use on a PC7302 Platform.
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
#include <asm/arch/timer.h>
#include <asm/arch/utilities.h>

/* Macros ------------------------------------------------------------------ */
/*!
 * Flag used to control whether or not the Die ID is displayed at boot time
 */
//#define DISPLAY_DIE_ID          (1)

/* Constants --------------------------------------------------------------- */
DECLARE_GLOBAL_DATA_PTR;


/* Prototypes--------------------------------------------------------------- */
/*!
 *
 * Start timer #0 in free running mode
 *
 */
static void pc302_timer_0_start(void);

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
    pc302_timer_0_start();

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
    unsigned int device_type, revision;
    unsigned int die_id[4];

    puts("Build: picoChip "PICOCHIP_PLATFORM" \n");

    /* What device are we running on ? */
    puts("Device: ");

    device_type = pc302_read_device_id();       /* Read the device Id */
    revision = pc302_read_device_revision();    /* Read the revision code */
    pc302_read_die_id_number(&die_id[0]);       /* Read the die id */

    switch (device_type)
    {
        case PC302_DEVICE_ID:
        {
            printf("PC302 Rev %04d\n", revision);
            break;
        }
        case PC312_DEVICE_ID:
        {
            printf("PC312 Rev %04d\n", revision);
            break;
        }
        default:
        {
            printf("Unknown !\n");
        }
    }

#if defined (DISPLAY_DIE_ID)
    /* Ouput the die_id */
    printf("Die Id: ");
    printf("0x%08x\n", die_id[0]);
    printf("        0x%08x\n", die_id[1]);
    printf("        0x%08x\n", die_id[2]);
    printf("        0x%08x\n", die_id[3]);
#endif

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

static void pc302_timer_0_start(void)
{
    /* Make sure timer #0 is disabled */
    pc302_write_to_register((CFG_TIMERBASE + TIMERNCONTROLREGOFFSET(0)), 0);

    /* Initialise the timer #0 to all 1's.  We do this  because we want to run
       the timer in free running mode. */
    pc302_write_to_register((CFG_TIMERBASE + TIMERNLOADCOUNTREGOFFSET(0)),
                            0xFFFFFFFF);

    /* Start timer #0 in free running mode */
    pc302_write_to_register((CFG_TIMERBASE + TIMERNCONTROLREGOFFSET(0)),
                            (TIMERINTERRUPTMASK | TIMERENABLE));
}
