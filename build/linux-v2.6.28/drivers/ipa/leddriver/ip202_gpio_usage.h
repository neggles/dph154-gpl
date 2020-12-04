/*
 * FILE NAME ip202_gpio_usage.h
 *
 * Copyright (c) 2007 ip.access Ltd.
 *
 * BRIEF MODULE DESCRIPTION
 *  Header file which determines which GPIO line is used for which purpose
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
 
#if !defined (INCLUDED_IP202_GPIO_USAGE_H)
#define  INCLUDED_IP202_GPIO_USAGE_H

/* FixMe: Add beyond XB card support! */
#define ARM_GPIO_IN_SWITCH              (PC202_GPIO_PIN_ARM_0)
#define ARM_GPIO_OUT_NRESET             (PC202_GPIO_PIN_ARM_1)
#define ARM_GPIO_OUT_SERVICE_LED        (PC202_GPIO_PIN_ARM_2)
#define ARM_GPIO_IN_ETHPHY              (PC202_GPIO_PIN_ARM_3)
#define ARM_GPIO_IN_TEMPWARN            (PC202_GPIO_PIN_ARM_4)
#define ARM_GPIO_OUT_SYS_LED_RED        (PC202_GPIO_PIN_ARM_5)

/* These lines are only used on customer HW */
#define ARM_GPIO_OUT_ADD_LED_RED        (PC202_GPIO_PIN_ARM_6)
#define ARM_GPIO_OUT_ADD_LED_GPS        (PC202_GPIO_PIN_ARM_7)

#define LED_DRIVER_ALLOWED_GPIOS        (( ARM_GPIO_OUT_SERVICE_LED ) | \
                                        ( ARM_GPIO_OUT_SYS_LED_RED ) | \
                                        ( ARM_GPIO_OUT_ADD_LED_RED ) | \
                                        ( ARM_GPIO_OUT_ADD_LED_GPS ))

#endif /*INCLUDED_IP202_GPIO_USAGE_H*/
