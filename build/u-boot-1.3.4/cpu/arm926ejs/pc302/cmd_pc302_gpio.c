/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file cmd_pc302_gpio.c
* \brief This file implements some test commands for the gpio pins on
*        the PC302 device.
*
* Copyright (c) 2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

/* Includes ---------------------------------------------------------------- */
#include <common.h>

#ifdef CONFIG_CMD_PC302_GPIO

#include <command.h>
#include <asm/arch/utilities.h>
#include <asm/arch/pc302_gpio.h>

int do_gpioset (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{

    int pin, value, ret;

    /* We need three arguments */
    if (argc < 3)
    {
        goto usage;
    }

    pin = simple_strtoul(argv[1], NULL, 0);
    value = simple_strtoul(argv[2], NULL, 0);

    /* Check the range on the 'pin' value */
    if ( (pin < PC302_GPIO_PIN_ARM_0) ||
         (pin > PC302_GPIO_PIN_SDGPIO_15) )
    {
        /* Oops, we have an out of range pin specified */
        printf ("pin out of range !\n");
        return 1;
    }

    /* Check on the 'value' specified */
    if ( (value < 0) ||
         (value > 1) )
    {
        /* Oops, we have an out of range value specified */
        printf ("value out of range !\n");
        return 1;
    }

    /* Initialise the gpio library */
    ret = pc302_gpio_init();
    if (ret != 0)
    {
        /* We have a problem initialising the gpio library */
        printf ("Can't initialise the gpio library !\n");
        return 1;
    }

    /* Request the gpio pin */
    ret = pc302_gpio_request (pin, "pin");
    if (ret != 0)
    {
        /* We have a problem requesting the gpio pin */
        printf ("Can't request gpio pin %d !\n", pin);
        return 1;
    }

    /* Set the gpio pin to be an output */
    ret = pc302_gpio_direction_output (pin, 0);
    if (ret != 0)
    {
        /* We have a problem setting the gpio pin as an output */
        printf ("Can't set gpio pin %d as an output !\n", pin);
        pc302_gpio_free (pin);
        return 1;
    }

    /* Set the value of the gpio pin */
    ret = pc302_gpio_set_value( pin, value);
    if (ret != 0)
    {
        /* We have a problem setting the value of the gpio pin */
        printf ("Can't set value of gpio pin %d !\n", pin);
        pc302_gpio_free (pin);
        return 1;
    }

    /* Free the pin */
    pc302_gpio_free (pin);

    return 0;

usage:
    printf("Usage:\n%s\n", cmdtp->usage);
    return 1;
}

/***************************************************/

U_BOOT_CMD(
	gpioset, 3, 0, do_gpioset,
	"gpioset - set a gpio pin high or low\n",
	"pin value - 'pin' is the gpio pin to set\n"
        "                    'value' is 1 or 0\n"
);

#endif /* CONFIG_CMD_PC302_GPIO */
