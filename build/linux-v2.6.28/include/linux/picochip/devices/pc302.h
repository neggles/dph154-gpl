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
 * \file pc302.h
 * \brief PC302 resource definition file.
 *
 * This file defines a list of resources including DMA channels, GPRs and
 * interrupt sources in the PC302 device.
 */

#ifndef __PICOIF_PC302_H__
#define __PICOIF_PC302_H__

/*!
 * \brief DMA channel identifiers for PC302 devices.
 *
 * This enum defines a list of DMA channels that are available for data
 * transport in PC302 devices.
 */
enum picoifDMAId_PC302
{
    PC302_DMA_AXI2PICO_0 = 0x0000,
    PC302_DMA_AXI2PICO_1,
    PC302_DMA_AXI2PICO_2,
    PC302_DMA_AXI2PICO_3,
    PC302_DMA_AXI2PICO_4,
    PC302_DMA_AXI2PICO_5,
    PC302_DMA_AXI2PICO_6,
    PC302_DMA_AXI2PICO_7,
    PICO_NUM_DMA_CHANNELS,

};

/*!
 * \brief GPR identifiers for PC302 devices.
 *
 * This enum defines a list of general purpose registers (GPRs) that are
 * available for reading/writing and transport use in PC302 devices.
 */
enum picoifGPRId_PC302
{
    PC302_GPR_AXI2PICO_0 = 0x1000,
    PC302_GPR_AXI2PICO_1,
    PC302_GPR_AXI2PICO_2,
    PC302_GPR_AXI2PICO_3,
    PC302_GPR_AXI2PICO_4,
    PC302_GPR_AXI2PICO_5,
    PC302_GPR_AXI2PICO_6,
    PC302_GPR_AXI2PICO_7,
    PC302_GPR_AXI2PICO_8,
    PC302_GPR_AXI2PICO_9,
    PC302_GPR_AXI2PICO_10,
    PC302_GPR_AXI2PICO_11,
    PC302_GPR_AXI2PICO_12,
    PC302_GPR_AXI2PICO_13,
    PC302_GPR_AXI2PICO_14,
    PC302_GPR_AXI2PICO_15,
    PC302_GPR_AXI2PICO_16,
    PC302_GPR_AXI2PICO_17,
    PC302_GPR_AXI2PICO_18,
    PC302_GPR_AXI2PICO_19,
    PC302_GPR_AXI2PICO_20,
    PC302_GPR_AXI2PICO_21,
    PC302_GPR_AXI2PICO_22,
    PC302_GPR_AXI2PICO_23,
    PICO_NUM_GPRS,

};

/*!
 * \brief IRQ identifiers for PC302 devices.
 *
 * This enum defines a list of interrupt sources that are available for
 * transport use in PC302 devices.
 */
enum picoifIRQId_PC302
{
    PC302_IRQ_AXI2PICO_0 = 0x2000,
    PC302_IRQ_AXI2PICO_1,
    PC302_IRQ_AXI2PICO_2,
    PC302_IRQ_AXI2PICO_3,
    PC302_IRQ_AXI2PICO_4,
    PC302_IRQ_AXI2PICO_5,
    PC302_IRQ_AXI2PICO_6,
    PC302_IRQ_AXI2PICO_7,
    PC302_IRQ_AXI2PICO_8,
    PC302_IRQ_AXI2PICO_9,
    PC302_IRQ_AXI2PICO_10,
    PC302_IRQ_AXI2PICO_11,
    PC302_IRQ_AXI2PICO_12,
    PC302_IRQ_AXI2PICO_13,
    PC302_IRQ_AXI2PICO_14,
    PC302_IRQ_AXI2PICO_15,
    PC302_IRQ_AXI2PICO_16,
    PC302_IRQ_AXI2PICO_17,
    PC302_IRQ_AXI2PICO_18,
    PC302_IRQ_AXI2PICO_19,
    PC302_IRQ_AXI2PICO_20,
    PC302_IRQ_AXI2PICO_21,
    PC302_IRQ_AXI2PICO_22,
    PC302_IRQ_AXI2PICO_23,
    PICO_NUM_IRQS,

};

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
 * \brief Type for identifying registers in a picoArray.
 *
 * GPR identifier. This is used to define a GPR resource for transports and
 * for GPR accesses - these should be used rather than absolute GPR numbers.
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifGPRId_PC302 picoifGPRId_t;

/*!
 * \brief Type for identifying DMA channels in a picoArray.
 *
 * DMA identifier. This is used to define a DMA resource for transports these
 * should be used rather than absolute DMA channel numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifDMAId_PC302 picoifDMAId_t;

/*!
 * \brief Type for identifying IRQ numbers in a picoArray.
 *
 * IRQ identifier. This is used to define an IRQ resource for transports these
 * should be used rather than absolute IRQ numbers.
 *
 * Each device description file will populate this enum with device specific
 * values.
 */
typedef enum picoifIRQId_PC302 picoifIRQId_t;

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

#endif /* !__PICOIF_PC302_H__ */
