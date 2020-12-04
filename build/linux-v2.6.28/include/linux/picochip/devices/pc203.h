/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file pc203.h
 * \brief PC203 resource definition file.
 *
 * This file defines a list of resources including DMA channels, GPRs and
 * interrupt sources in the PC203 device.
 *
 */

#ifndef __PICOIF_PC203_H__
#define __PICOIF_PC203_H__

/*!
 * \brief DMA channel identifiers for PC203 devices.
 *
 * This enum defines a list of DMA channels that are available for data
 * transport in PC203 devices.
 */
enum picoifDMAId_PC203
{
    PC203_DMA_PROCIF_0 = 0x0000,
    PC203_DMA_PROCIF_1,
    PC203_DMA_PROCIF_2,
    PC203_DMA_PROCIF_3,
    PICO_NUM_DMA_CHANNELS,
};

/*!
 * \brief GPR identifiers for PC203 devices.
 *
 * This enum defines a list of general purpose registers (GPRs) that are
 * available for reading/writing and transport use in PC203 devices.
 */
enum picoifGPRId_PC203
{
    PC203_GPR_PROCIF_0 = 0x1000,
    PC203_GPR_PROCIF_1,
    PC203_GPR_PROCIF_2,
    PC203_GPR_PROCIF_3,
    PC203_GPR_PROCIF_4,
    PC203_GPR_PROCIF_5,
    PC203_GPR_PROCIF_6,
    PC203_GPR_PROCIF_7,
    PC203_GPR_PROCIF_8,
    PC203_GPR_PROCIF_9,
    PC203_GPR_PROCIF_10,
    PC203_GPR_PROCIF_11,
    PC203_GPR_PROCIF_12,
    PC203_GPR_PROCIF_13,
    PC203_GPR_PROCIF_14,
    PC203_GPR_PROCIF_15,
    PC203_GPR_PROCIF_16,
    PC203_GPR_PROCIF_17,
    PC203_GPR_PROCIF_18,
    PC203_GPR_PROCIF_19,
    PC203_GPR_PROCIF_20,
    PC203_GPR_PROCIF_21,
    PC203_GPR_PROCIF_22,
    PC203_GPR_PROCIF_23,
    PC203_GPR_ITM,
    PC203_GPR_ITS,
    PICO_NUM_GPRS,
};

/*!
 * \brief IRQ identifiers for PC203 devices.
 *
 * This enum defines a list of interrupt sources that are available for
 * transport use in PC203 devices.
 */
enum picoifIRQId_PC203
{
    PC203_IRQ_PROCIF = 0x2000,
    PC203_IRQ_AHB2PICO_0,
    PC203_IRQ_AHB2PICO_1,
    PC203_IRQ_AHB2PICO_2,
    PC203_IRQ_AHB2PICO_3,
    PC203_IRQ_AHB2PICO_4,
    PC203_IRQ_AHB2PICO_5,
    PC203_IRQ_AHB2PICO_6,
    PC203_IRQ_AHB2PICO_7,
    PC203_IRQ_AHB2PICO_8,
    PC203_IRQ_AHB2PICO_9,
    PC203_IRQ_AHB2PICO_10,
    PC203_IRQ_AHB2PICO_11,
    PC203_IRQ_AHB2PICO_12,
    PC203_IRQ_AHB2PICO_13,
    PC203_IRQ_AHB2PICO_14,
    PC203_IRQ_AHB2PICO_15,
    PC203_IRQ_AHB2PICO_16,
    PC203_IRQ_AHB2PICO_17,
    PC203_IRQ_AHB2PICO_18,
    PC203_IRQ_AHB2PICO_19,
    PC203_IRQ_AHB2PICO_20,
    PC203_IRQ_AHB2PICO_21,
    PC203_IRQ_AHB2PICO_22,
    PC203_IRQ_AHB2PICO_23,
    PICO_NUM_IRQS,
};

/*! 
 * \brief GPIO and SDGPIO identifiers for PC203 devices.
 *  
 * This enum defines a list of (SD)GPIO sources that are available for
 * use in PC203 devices.
 */ 
enum picoifGpioPinNum_PC203
{
    PC203_GPIO_PIN_INVAL = -1,  /*< Invalid pin configuration. */
    PC203_GPIO_PIN_SDGPIO_0,    /* SDGPIO pin identifiers. */
    PC203_GPIO_PIN_SDGPIO_1,
    PC203_GPIO_PIN_SDGPIO_2,
    PC203_GPIO_PIN_SDGPIO_3,
    PC203_GPIO_PIN_SDGPIO_4,
    PC203_GPIO_PIN_SDGPIO_5,
    PC203_GPIO_PIN_SDGPIO_6,
    PC203_GPIO_PIN_SDGPIO_7,
    PC203_GPIO_PIN_SDGPIO_8,
    PC203_GPIO_PIN_SDGPIO_9,
    PC203_GPIO_PIN_SDGPIO_10,
    PC203_GPIO_PIN_SDGPIO_11,
    PC203_GPIO_PIN_SDGPIO_12,
    PC203_GPIO_PIN_SDGPIO_13,
    PC203_GPIO_PIN_SDGPIO_14,
    PC203_GPIO_PIN_SDGPIO_15,
    PC203_GPIO_PIN_SDGPIO_16,
    PC203_GPIO_PIN_SDGPIO_17,
    PC203_GPIO_PIN_SDGPIO_18,
    PC203_GPIO_PIN_SDGPIO_19,
    PC203_GPIO_PIN_SDGPIO_20,
    PC203_GPIO_PIN_SDGPIO_21,
    PC203_GPIO_PIN_SDGPIO_22,
    PC203_GPIO_PIN_SDGPIO_23,
    PC203_GPIO_PIN_SDGPIO_24,
    PC203_GPIO_PIN_SDGPIO_25,
    PC203_GPIO_PIN_SDGPIO_26,
    PC203_GPIO_PIN_SDGPIO_27,
    PC203_GPIO_PIN_SDGPIO_28,
    PC203_GPIO_PIN_SDGPIO_29,
    PC203_GPIO_PIN_SDGPIO_30,
    PC203_GPIO_PIN_SDGPIO_31,
    PICO_NUM_GPIOS,

};

/*!
 * \brief Type for identifying registers in a picoArray.
 *
 * GPR identifier. This is used to define a GPR resource for transports and
 * for GPR accesses - these should be used rather than absolute GPR numbers.
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifGPRId_PC203 picoifGPRId_t;

/*!
 * \brief Type for identifying DMA channels in a picoArray.
 *
 * DMA identifier. This is used to define a DMA resource for transports these
 * should be used rather than absolute DMA channel numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifDMAId_PC203 picoifDMAId_t;

/*!
 * \brief Type for identifying (SD)GPIO pins in a picoArray.
 *
 * GPIO identifier. This is used to define an GPIO resource which
 * should be used rather than absolute GPIO numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifGpioPinNum_PC203 picoifGpioPinNum_t;

#endif /* !__PICOIF_PC203_H__ */
