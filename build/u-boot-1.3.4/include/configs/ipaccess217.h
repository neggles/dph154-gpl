/*!
* \file ipaccess217.h
* \brief Configuration file for U-Boot on the Nano-8 platform.
*
* Copyright (c) 2010 ip.access Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@ipaccess.com
*/

#ifndef __CONFIG_H

/* Include common ip302ff config */
#include <configs/ipaccessip302ff.h>


/*-----------------------------------------------------------------------
 * I2C configuration
 */
#define	CONFIG_SOFT_I2C			/* Software I2C support enabled	*/

/* Dummy values required for generic code */
#define CFG_I2C_SPEED		50000
#define CFG_I2C_SLAVE		0x7F

/*
 * Software (bit-bang) I2C driver configuration
 */
#define I2C_ACTIVE	pc302_gpio_set_direction(PC302_GPIO_PIN_ARM_2, 0)
#define I2C_TRISTATE	pc302_gpio_set_direction(PC302_GPIO_PIN_ARM_2, 1)
#define I2C_READ	pc302_gpio_get_value(PC302_GPIO_PIN_ARM_2)
#define I2C_SDA(bit)	pc302_gpio_set_value(PC302_GPIO_PIN_ARM_2, bit ? 1 : 0)
#define I2C_SCL(bit)	pc302_gpio_set_value(PC302_GPIO_PIN_ARM_0, bit ? 1 : 0)
#define I2C_DELAY	udelay(5)	/* fairly conservative */

#define CONFIG_CMD_I2C

#define CONFIG_MICREL_SWITCH
#define CFG_MICREL_SWITCH_ADDR	0x5F


#endif /* __CONFIG_H */

