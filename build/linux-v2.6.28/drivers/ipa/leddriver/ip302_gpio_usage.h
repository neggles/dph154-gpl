/*
 * FILE NAME ip302_gpio_usage.h
 *
 * Copyright (c) 2009 ip.access Ltd.
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
 
#if !defined (INCLUDED_IP302_GPIO_USAGE_H)
#define  INCLUDED_IP302_GPIO_USAGE_H

#define ARM_GPIO_IN_SWITCH              (PC302_GPIO_PIN_ARM_4)
#define ARM_GPIO_OUT_SERVICE_LED        (PC302_GPIO_PIN_ARM_5)
#define ARM_GPIO_OUT_SYS_LED_RED        (PC302_GPIO_PIN_ARM_3)
#define ARM_GPIO_OUT_NRESET             (PC302_GPIO_PIN_ARM_1)

/* These lines are only used on customer HW */
#define ARM_GPIO_OUT_ADD_LED_RED        (PC302_GPIO_PIN_ARM_6)
#define ARM_GPIO_OUT_ADD_LED_GPS        (PC302_GPIO_PIN_ARM_7)

#endif /*INCLUDED_IP202_GPIO_USAGE_H*/
