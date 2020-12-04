/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file reset.S
* \brief Function used to reset the PC302 device.
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
#include <config.h>
#include <asm/arch/pc302.h>
#include <asm/arch/wdog.h>
#include <asm/arch/gpio.h>

/* Macros ------------------------------------------------------------------ */
#define ASSERT_RESET_PIN            0x80    /* Using ARM GPIO 7 */
#define RESET_PIN_IS_OUTPUT         0x80

/* Functions --------------------------------------------------------------- */

	.align	5

/*!
 *
 * Perform a software reset of the PC302 device.
 *
 * Note: The method used to provide the reset functionailty is
 *       hardware platform dependant.
 * Note: We will never return from this function.
 *
 */
.globl reset_cpu

reset_cpu:

#if defined(CONFIG_PICOCHIP_PC7302) && defined(CONFIG_USE_GPIO_RESET_METHOD)

        /* We are running on a picoChip PC7302 platform
           and have been asked to reset using the ARM GPIO method */

        /* Get base address of the arm gpio block */
        ldr r0,=PC302_GPIO_BASE

        /* Assert the reset pin (in the data register) */
        ldr r1,[r0, #GPIO_SW_PORT_A_DR_REG_OFFSET]
        orr r1,r1,#ASSERT_RESET_PIN
        str r1,[r0, #GPIO_SW_PORT_A_DR_REG_OFFSET]

        /* Set up the reset pin to be an output */
        ldr r1,[r0, #GPIO_SW_PORT_A_DDR_REG_OFFSET]
        orr r1,r1,#RESET_PIN_IS_OUTPUT
        str r1,[r0, #GPIO_SW_PORT_A_DDR_REG_OFFSET]

        /* The reset input to the device should now be active */

#else
        /* Use the 'fallback' watchdog method for reseting */

        ldr     r0, =PC302_WDOG_BASE

        /* Read the control register */
        ldr     r1, [r0, #WDOG_CONTROL_REG_OFFSET]

        /* Clear the 'Response mode' bit */
        bic     r1, #WDOGCONTROLREGRMODMASK

        /* Set the 'Watchdog Enable' bit */
        orr     r1, #WDOGCONTROLREGWDT_ENMASK

        /* Write to the control register */
        str     r1, [r0, #WDOG_CONTROL_REG_OFFSET]

        /* The Watchdog is hardwired for a 1 second timeout */

        /* 'Kick' the Watchdog into life */
        mov     r1, #WDOG_COUNTER_RESTART_KICK_VALUE
        str     r1, [r0, #WDOG_CONTROL_REG_OFFSET]

#endif

reset_loop_forever:

        /* We will never return from this function */
        b       reset_loop_forever
