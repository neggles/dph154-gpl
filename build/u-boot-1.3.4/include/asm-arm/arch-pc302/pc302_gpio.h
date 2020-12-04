/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
 * \file pc302_gpio.h
 * \brief Definitions use with the PC302 gpio library.
 *
 * Copyright (c) 2009 picoChip Designs Ltd
 * Copyright (c) 2010 ip.access Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

#ifndef PC302_PC302_GPIO_H
#define PC302_PC302_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/*!
 * \brief GPIO and SDGPIO identifiers for PC302 devices.
 *
 * This enum defines a list of (SD)GPIO sources that are available for
 * use in PC302 devices.
 */
enum picoifGpioPinNum_PC302
{
    PC302_GPIO_PIN_INVAL = -1,  /*< Invalid pin configuration. */
    PC302_GPIO_PIN_ARM_0 =  0,  /* ARM GPIO pin identifiers. */
    PC302_GPIO_PIN_ARM_1,
    PC302_GPIO_PIN_ARM_2,
    PC302_GPIO_PIN_ARM_3,
    PC302_GPIO_PIN_ARM_4,
    PC302_GPIO_PIN_ARM_5,
    PC302_GPIO_PIN_ARM_6,
    PC302_GPIO_PIN_ARM_7,
    PC302_GPIO_PIN_SDGPIO_0,    /* SDGPIO pin identifiers. */
    PC302_GPIO_PIN_SDGPIO_1,
    PC302_GPIO_PIN_SDGPIO_2,
    PC302_GPIO_PIN_SDGPIO_3,
    PC302_GPIO_PIN_SDGPIO_4,
    PC302_GPIO_PIN_SDGPIO_5,
    PC302_GPIO_PIN_SDGPIO_6,
    PC302_GPIO_PIN_SDGPIO_7,
    PC302_GPIO_PIN_ARM_8,      /* ARM shared pins. */
    PC302_GPIO_PIN_ARM_9,
    PC302_GPIO_PIN_ARM_10,
    PC302_GPIO_PIN_ARM_11,
    PC302_GPIO_PIN_ARM_12,
    PC302_GPIO_PIN_ARM_13,
    PC302_GPIO_PIN_ARM_14,
    PC302_GPIO_PIN_ARM_15,
    PC302_GPIO_PIN_SDGPIO_8,  /* SDGPIO shared pins. */
    PC302_GPIO_PIN_SDGPIO_9,
    PC302_GPIO_PIN_SDGPIO_10,
    PC302_GPIO_PIN_SDGPIO_11,
    PC302_GPIO_PIN_SDGPIO_12,
    PC302_GPIO_PIN_SDGPIO_13,
    PC302_GPIO_PIN_SDGPIO_14,
    PC302_GPIO_PIN_SDGPIO_15,
    PICO_NUM_GPIOS,
};

/*!
 * \brief Type for identifying (SD)GPIO pins in a picoArray.
 *
 * GPIO identifier. This is used to define an GPIO resource which
 * should be used rather than absolute GPIO numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifGpioPinNum_PC302 picoifGpioPinNum_t;

/* Prototypes--------------------------------------------------------------- */
/**
 * Initialise the gpio library for use
 *
 * \return Returns zero on success, non-zero on failure.
 */
int
pc302_gpio_init( void );

/**
 * Request a new GPIO pin. This implements part of the Linux GPIO guidelines.
 *
 * \param gpio The pin to request.
 * \param label The name of the pin - this only serves as a tag for debugging
 * so can be anything.
 * \return Returns zero on success, non-zero on failure.
 */
int
pc302_gpio_request( unsigned gpio,
                    const char *label );

/**
 * Free a GPIO pin previously requested with gpio_request().
 *
 * \param gpio The GPIO pin to free.
 */
void
pc302_gpio_free( unsigned gpio );

/**
 * Set the direction of a GPIO pin requested with gpio_request() to be an
 * input.
 *
 * \param gpio The GPIO pin to configure.
 * \return Returns zero on success, non-zero on failure.
 */
int
pc302_gpio_direction_input( unsigned gpio );

/**
 * Set the direction of a GPIO pin requested with gpio_request() to be an
 * output.
 *
 * \param gpio The GPIO pin to configure.
 * \param value The initial output value for the gpio pin.
 * \return Returns zero on success, non-zero on failure.
 */
int
pc302_gpio_direction_output( unsigned gpio,
                             int value );

/**
 * Set the direction of a GPIO pin.
 *
 * \param gpio The number of the pin to set the value of.
 * \param value 0 for output, 1 for input
 */
int
pc302_gpio_set_direction( unsigned gpio,
                          int dir );

/**
 * Set the value of a GPIO pin.
 *
 * \param gpio The number of the pin to set the value of.
 * \param value The value to set the pin to.
 */
int
pc302_gpio_set_value( unsigned gpio,
                      int value );

/**
 * Get the value of a GPIO pin.
 *
 * \param gpio The number of the pin to get the value of.
 * \return Returns the value of the pin on success,
 * negative on failure.
 */
int
pc302_gpio_get_value( unsigned gpio );

int
pc302_gpio_configure_dac( unsigned gpio,
                          u8 converter_size,
                          u16 analogue_rate );

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* !PC302_PC302_GPIO_H */
