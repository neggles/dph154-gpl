/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file pc302_gpio.c
* \brief This file implements a driver for the GPIO pins on the picoChip PC302
*        device. This includes both the ARM and SD GPIO.
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
#include <malloc.h>
#include <asm/errno.h>
#include <asm/arch/axi2cfg.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pc302_gpio.h>
#include <asm/arch/utilities.h>

/* Include for BUG, BUG_ON */
#include <linux/mtd/compat.h>

/* Macros ------------------------------------------------------------------ */
/** The number of pads available for GPIO. */
#define PC302_GPIO_NUM_PADS ( 24 )

/** The maximum length of a pin name. */
#define PC302_GPIO_PIN_NAME_MAX ( 32 )

/** The width of the arm gpio ports */
#define PC302_ARM_GPIO_PORT_WIDTH   ( 8 )

/** The AXI2PICO wake up command */
#define PC302_AXI2PICO_WAKE_UP  ( 0 )

/** The base address of SD-GPIO config registers in the AXI2Pico. */
#define PC302_GPIO_SD_PIN_CONFIG_BASE   0x9800

/** The base address of SD-GPIO analogue value registers in the AXI2Pico. */
#define PC302_GPIO_SD_PIN_ANALOGUE_VALUE_BASE   0x9801

/** The base address of SD-GPIO analogue rate registers in the AXI2Pico. */
#define PC302_GPIO_SD_PIN_ANALOGUE_RATE_BASE    0x9802

/** The address of the control value register in the AXI2Pico. */
#define PC302_GPIO_SD_CONTROL_VAL_REG   0x9882

/** The address of the output value register in the AXI2Pico. */
#define PC302_GPIO_SD_OUTPUT_VAL_REG    0x9884

/** The address of the input value register in the AXI2Pico. */
#define PC302_GPIO_SD_INPUT_VAL_REG     0x9880

/** The address of the sleep register in the AXI2Pico. */
#define PC302_AXI2PICO_SLEEP_REG        0xA060

/** The spacing between SD-GPIO config registers. */
#define PC302_GPIO_SD_PIN_CONFIG_SPACING    4

/**
 * Macro to get the address of a config register for a SD-GPIO pin.
 *
 * \param _n The SD-GPIO pin number.
 * \return Returns the base address of the register.
 */
#define PC302_GPIO_SD_PIN_CONFIG( _n ) \
    PC302_GPIO_SD_PIN_CONFIG_BASE + ( _n * PC302_GPIO_SD_PIN_CONFIG_SPACING )

/**
 * Macro to get the address of a analogue rate register for a SD-GPIO pin.
 *
 * \param _n The SD-GPIO pin number.
 * \return Returns the base address of the register.
 */
#define PC302_GPIO_SD_PIN_ANALOGUE_RATE( _n ) \
    PC302_GPIO_SD_PIN_ANALOGUE_RATE_BASE + \
        ( _n * PC302_GPIO_SD_PIN_CONFIG_SPACING )

/**
 * Macro to get the address of a analogue value register for a SD-GPIO pin.
 *
 * \param _n The SD-GPIO pin number.
 * \return Returns the base address of the register.
 */
#define PC302_GPIO_SD_PIN_ANALOGUE_VAL( _n ) \
    PC302_GPIO_SD_PIN_ANALOGUE_VALUE_BASE + \
        ( _n * PC302_GPIO_SD_PIN_CONFIG_SPACING )

/** Control source bit. */
#define PC302_GPIO_SD_CONFIG_CS_MASK ~( 1 << 15 )

/** Analogue not digital bit. */
#define PC302_GPIO_SD_CONFIG_AND ( 1 << 14 )

/** The mask for analogue converter size in the config register. */
#define PC302_GPIO_SD_CONV_SZ_MASK ( 0xF )

/** Soft reset lock bit. */
#define PC302_GPIO_SD_CONFIG_SR_LOCK ( 1 << 13 )

/** PC302 AXI2Pico CAEID. */
#define PC302_AXI2PICO_CAEID    ( 0x00A8 )

/** PC302 PAI CAEID. */
#define PC302_PAI_CAEID         ( 0x0578 )

/** The address of the sleep register in the PAI. */
#define PC302_PAI_SLEEP_REG     ( 0xA060 )

/** The PAI wake up command */
#define PC302_PAI_WAKE_UP       ( 0 )

/** Mask for valid bits in the PAI pai_io_ctrl register. */
#define PC302_PAI_IO_CTRL_REG_MASK  ( 0xF )

/** The address of the pai_io_ctrl register in the PAI. */
#define PC302_PAI_IO_CTRL_REG   ( 0x0009 )

/* Constants --------------------------------------------------------------- */
/** Define some bit values for ARM_GPIO Muxing control */
typedef enum
{
    PAI_GPIO_PIN_ARM_4 = 0xB,
    PAI_GPIO_PIN_ARM_5 = 0xA,
    PAI_GPIO_PIN_ARM_6 = 0x9,
    PAI_GPIO_PIN_ARM_7 = 0x8,

} pc302_pai_arm;

/** Define some bit values for SDGPIO_GPIO Muxing control */
typedef enum
{
    PAI_SDGPIO_PIN_ARM_4 = 0x7,
    PAI_SDGPIO_PIN_ARM_5 = 0x6,
    PAI_SDGPIO_PIN_ARM_6 = 0x5,
    PAI_SDGPIO_PIN_ARM_7 = 0x4,

} pc302_pai_sdgpio;

/** GPIO pin type. */
typedef enum
{
    PC302_GPIO_ARM,
    PC302_GPIO_SD,

} pc302gpio_pin_type;

/** Private representation to store state of a GPIO pin. */
struct pc302gpio_pin_allocation
{
    /** The pin number (not pad number). */
    int pin_num;

    /** Boolean flag for an input GPIO. */
    int is_input;

    /** Boolean flag for whether the pin is active. */
    int enabled;

    /** The pad number. */
    int pad;

    /** The name of the pin - filled in when the pin is requested. */
    char name[ PC302_GPIO_PIN_NAME_MAX ];

    /** The cached value of the pin. Used for output GPIO pins to remember the
     * last value set. */
    int value;

    /** Analogue not digital flag. 0 == digital, non-zero == analogue. */
    int a_not_d;
};

/**
 * The current boot mode.
 */
static u32 boot_mode = 1;

static struct pc302gpio_pin_allocation *pc302gpio_pads[ PC302_GPIO_NUM_PADS ];

/* Prototypes--------------------------------------------------------------- */
/**
 * Get the pad number for the requested GPIO pin. This is needed as some pins
 * are multiplexed and we need to check whether the pad is already in use
 * either by the other shared pin or the EBI bus.
 *
 * \param pin The pin number being requested.
 * \return Returns the pad number in the range 0 ... PC302_GPIO_NUM_PADS on
 * success or negative on failure (invalid pin number, pad already in use
 * etc).
 */
static int
pc302gpio_get_pad( picoifGpioPinNum_t pin );

/**
 * Set the pin multiplexing for shared GPIO pins.
 *
 * \param gpio The pin to set the multiplex value of.
 * \return Returns zero on success or if the multiplexing does not need to be
 * set, non-zero on failure.
 */
static int
set_pin_mux( unsigned gpio );

/**
 * Allocate a GPIO pin state structure.
 *
 * \param name The name of the pin (this can be anything - it can be the
 * function of the pin or the application using it for example).
 * \return Returns a pointer to the pin on success, NULL on failure.
 */
static struct pc302gpio_pin_allocation *
pc302gpio_alloc_pin( const char *name );

/**
 * Set the direction of a GPIO pin.
 *
 * \param pin The pin to set the direction of.
 * \param input Flag for input GPIO pins.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_set_direction( struct pc302gpio_pin_allocation *pin,
                         int input );

/**
 * Get the pin type.
 *
 * \param pin The pin to query.
 * \return Returns the type of GPIO pin.
 */
static pc302gpio_pin_type
pc302gpio_get_pin_type( struct pc302gpio_pin_allocation *pin );

/**
 * Given a GPIO pin number, find the PC302 representation of it.
 *
 * \param gpio The GPIO pin number.
 * \return Returns a pointer to the internal representation.
 */
static struct pc302gpio_pin_allocation *
pc302gpio_find_pin( unsigned gpio );

/**
 * Given a GPIO pin number, find the number of the pin in the block. The
 * global enumeration of pin numbers include both types and are non
 * contiguous. This function takes the pin number and turns it into a number
 * in the range 0->15 for the block that controls it.
 *
 * \param pin The pin to query.
 * \return Returns the block pin number on success, non-zero on failure.
 */
static int
pc302gpio_pin_to_block_pin( struct pc302gpio_pin_allocation *pin );

/**
 * Set the value of an output SDGPIO pin.
 *
 * \param pin The pin to set the value of.
 * \param value The value to set the pin to. Intepreted as non-zero == 1.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_sd_set_value( struct pc302gpio_pin_allocation *pin,
                        int value );

/**
 * Get the value of an input SDGPIO pin.
 *
 * \param pin The pin to get the value of.
 * \return Returns the value read on success, negative on failure.
 */
static int
pc302gpio_sd_get_value( struct pc302gpio_pin_allocation *pin );

/**
 * Get the value of an input ARM GPIO pin.
 *
 * \param pin The pin to get the value of.
 * \return Returns the value read on success, negative on failure.
 */
static int
pc302gpio_arm_get_value( struct pc302gpio_pin_allocation *pin );

/**
 * Set the direction of an SD-GPIO pin.
 *
 * \param pin The pin to set the direction of.
 * \param input Boolean flag to set the pin to input.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_sd_set_direction( struct pc302gpio_pin_allocation *pin,
                            int input );

/**
 * Set the value of an output ARM GPIO pin.
 *
 * \param pin The pin to set the value of.
 * \param value The value to set the pin to. Intepreted as non-zero == 1.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_arm_set_value( struct pc302gpio_pin_allocation *pin,
                         int value );

/**
 * Set the direction of an ARM GPIO pin.
 *
 * \param pin The pin to set the direction of.
 * \param input Boolean flag to set the pin to input.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_arm_set_direction( struct pc302gpio_pin_allocation *pin,
                             int input );

/**
 * Set the value of a GPIO pin.
 *
 * \param pin The pin to set the value of.
 * \param value The value to set.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_set_value( struct pc302gpio_pin_allocation *pin,
                     int value );

/**
 * Get the value of a GPIO pin.
 *
 * \param pin The pin to get the value of.
 * \return Returns the value of the pin on success,
 * negative on failure.
 */
static int
pc302gpio_get_value( struct pc302gpio_pin_allocation *pin );

/**
 * Given a GPIO pin number, setup the pai gpio muxing.
 *
 * \param gpio The GPIO pin number.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_pai_muxing( picoifGpioPinNum_t pin );

/**
 * Read the System Configuration Register
 *
 * \return The value read.
 */
static u32
syscfg_read( void );

/**
 * Update the System Configuration Register
 *
 * \param mask Which bits we want to update
 * \param val The new value for the bits we want to update.
 */
static void
syscfg_update( u32 mask,
               u32 val );

/* Functions --------------------------------------------------------------- */
static int
pc302gpio_get_pad( picoifGpioPinNum_t pin )
{
    int ret = -EINVAL;

    if ( pin >= PC302_GPIO_PIN_ARM_0 && pin <= PC302_GPIO_PIN_ARM_7 )
    {
        /* ARM GPIO */

        /* If we are in parallel boot mode, then to use the bottom 4
         * bits of ARM GPIO we need to noodle with the PAI block
         * gpio mux control.*/
        ret = -EIO;
        if ( 0 == boot_mode && pin >= PC302_GPIO_PIN_ARM_4 )
        {
            ret = pc302gpio_pai_muxing( pin );
            if ( 0 != ret )
                goto out;
        }

        ret = pin - PC302_GPIO_PIN_ARM_0;
    }
    else if ( pin >= PC302_GPIO_PIN_SDGPIO_0 && pin <= PC302_GPIO_PIN_SDGPIO_7 )
    {
        /* SD-GPIO */

        /* If we are in parallel boot mode, then to use the bottom 4
         * bits of SDGPIO GPIO we need to noodle with the PAI block
         * gpio mux control. */
        ret = -EIO;
        if ( 0 == boot_mode && pin >= PC302_GPIO_PIN_SDGPIO_4 )
        {
            ret = pc302gpio_pai_muxing( pin );
            if ( 0 != ret )
                goto out;
        }

        ret = pin - PC302_GPIO_PIN_ARM_0;
    }
    else if ( pin >= PC302_GPIO_PIN_ARM_8 &&
              pin <= PC302_GPIO_PIN_ARM_15 )
    {
        /* Shared ARM GPIO */
        ret = pin - PC302_GPIO_PIN_ARM_0;

    }
    else if ( pin >= PC302_GPIO_PIN_SDGPIO_8 &&
              pin <= PC302_GPIO_PIN_SDGPIO_15 )
    {
        /* Shared SD-GPIO */
        ret = pin - PC302_GPIO_PIN_ARM_0 - 8;
    }

out:
    return ret;
}

static int
set_pin_mux( unsigned gpio )
{
    int is_arm = 0;
    u32 val;
    unsigned shared_pin_num;

    if ( ( gpio >= PC302_GPIO_PIN_ARM_0 && gpio <= PC302_GPIO_PIN_SDGPIO_7 ) )
    {
        if ( gpio == PC302_GPIO_PIN_SDGPIO_0 )
        {
            /* Need to set SDGPIO Freq_Synth Mux bit to 0 */
            syscfg_update( AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_MASK, (u32)0 );
        }
        return 0;
    }

    if ( ( gpio >= PC302_GPIO_PIN_ARM_8 && gpio <= PC302_GPIO_PIN_ARM_15 ) )
        is_arm = 1;

    switch ( gpio )
    {
        case PC302_GPIO_PIN_ARM_8 ... PC302_GPIO_PIN_ARM_15:
            shared_pin_num = gpio - PC302_GPIO_PIN_ARM_8;
            break;

        case PC302_GPIO_PIN_SDGPIO_8 ... PC302_GPIO_PIN_SDGPIO_15:
            shared_pin_num = gpio - PC302_GPIO_PIN_SDGPIO_8;
            break;

        default:
            return -EINVAL;
            BUG();
    }

    val = syscfg_read();
    val &=
        ~( 1 << ( shared_pin_num + AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_LO ) );
    if ( is_arm )
        val |= ( 1 << ( shared_pin_num +
                        AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_LO ) );

    syscfg_update( AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_MASK, val );

    return 0;
}

static struct pc302gpio_pin_allocation *
pc302gpio_alloc_pin( const char *name )
{
    struct pc302gpio_pin_allocation *p = malloc( sizeof( *p ) );

    if ( p )
    {
        sprintf( p->name, "%s", name );
        p->pin_num = -1;
        p->pad = -1;
        p->enabled = 0;
        p->is_input = 0;
        p->a_not_d = 0;
    }

    return p;
}

static pc302gpio_pin_type
pc302gpio_get_pin_type( struct pc302gpio_pin_allocation *pin )
{
    int ret;
    BUG_ON( !pin );

    switch ( pin->pin_num )
    {
        case PC302_GPIO_PIN_ARM_0 ... PC302_GPIO_PIN_ARM_7:
        case PC302_GPIO_PIN_ARM_8 ... PC302_GPIO_PIN_ARM_15:
            ret = PC302_GPIO_ARM;
            break;

        case PC302_GPIO_PIN_SDGPIO_0 ... PC302_GPIO_PIN_SDGPIO_7:
        case PC302_GPIO_PIN_SDGPIO_8 ... PC302_GPIO_PIN_SDGPIO_15:
            ret = PC302_GPIO_SD;
            break;

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static int
pc302gpio_pin_to_block_pin( struct pc302gpio_pin_allocation *pin )
{
    unsigned ret;

    BUG_ON( NULL == pin );

    switch ( pin->pin_num )
    {
        case PC302_GPIO_PIN_ARM_0 ... PC302_GPIO_PIN_ARM_7:
            ret =  pin->pin_num - PC302_GPIO_PIN_ARM_0;
            break;

        case PC302_GPIO_PIN_ARM_8 ... PC302_GPIO_PIN_ARM_15:
            ret =  ( pin->pin_num - PC302_GPIO_PIN_ARM_8 ) + 8;
            break;

        case PC302_GPIO_PIN_SDGPIO_0 ... PC302_GPIO_PIN_SDGPIO_7:
            ret =  pin->pin_num - PC302_GPIO_PIN_SDGPIO_0;
            break;

        case PC302_GPIO_PIN_SDGPIO_8 ... PC302_GPIO_PIN_SDGPIO_15:
            ret =  ( pin->pin_num - PC302_GPIO_PIN_SDGPIO_8 ) + 8;
            break;

        default:
            ret = -1;
            BUG();
    }

    return ret;
}

static int
pc302gpio_sd_set_value( struct pc302gpio_pin_allocation *pin,
                        int value )
{
    unsigned block_pin;
    int ret;
    u16 data;
    BUG_ON( NULL == pin );

    if ( pin->is_input )
        return -EINVAL;

    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( !pin->a_not_d )
    {
        /* Digital mode */
        ret = pc302_config_read( PC302_AXI2PICO_CAEID,
                                 PC302_GPIO_SD_OUTPUT_VAL_REG, 1, &data );
        if ( 1 != ret )
        {
            printf ("%s : ret = %d\n",  __FUNCTION__, ret);
            printf( "%s : failed to read SDGPIO output value reg\n", __FUNCTION__ );
            return -EIO;
        }

        data &= ~( 1 << block_pin );
        data |= ( !!value ) << block_pin;

        ret = pc302_config_write( PC302_AXI2PICO_CAEID,
                                  PC302_GPIO_SD_OUTPUT_VAL_REG,
                                  1, &data );
        if ( 1 != ret )
        {
            printf( "%s : failed to output control register for SDGPIO"
                    "pin %u\n", __FUNCTION__, block_pin );
            return -EIO;
        }
    }
    else
    {
        /* Analogue mode */
        data = (u16)value;
        ret = pc302_config_write( PC302_AXI2PICO_CAEID,
                                  PC302_GPIO_SD_PIN_ANALOGUE_VAL( block_pin ),
                                  1, &data );
        if ( 1 != ret )
        {
            printf( "%s : failed to write analogue value register for "
                    "SDGPIO pin %u\n", __FUNCTION__, block_pin );
            return -EIO;
        }
    }

    return 0;
}

static int
pc302gpio_sd_get_value( struct pc302gpio_pin_allocation *pin )
{
    unsigned block_pin;
    int ret;
    u16 data;
    BUG_ON( NULL == pin );

    if ( !pin->is_input && !pin->a_not_d )
        return -EINVAL;

    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( !pin->a_not_d )
    {
        /* Digital mode */
        ret = pc302_config_read( PC302_AXI2PICO_CAEID,
                                 PC302_GPIO_SD_INPUT_VAL_REG, 1, &data );
        if ( 1 != ret )
        {
            printf( "%s : failed to read SDGPIO input value reg\n",
                    __FUNCTION__ );
            return -EIO;
        }

        return !!( data & ( 1 << block_pin ) );
    }
    else
    {
        /* Analogue mode */
        ret = pc302_config_read( PC302_AXI2PICO_CAEID,
                                 PC302_GPIO_SD_PIN_ANALOGUE_VAL( block_pin ),
                                 1, &data );
        if ( 1 != ret )
        {
            printf( "%s : failed to read the analogue value register "
                    "for SDGPIO pin %u\n", __FUNCTION__, block_pin );
            return -EIO;
        }

        return (int)data;
    }
}

static int
pc302gpio_arm_get_value( struct pc302gpio_pin_allocation *pin )
{
    unsigned block_pin;
    unsigned int port_ds;
    u32 pin_offset;
    u32 val;
    BUG_ON( NULL == pin );

    if ( !pin->is_input )
        return -EINVAL;

    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( block_pin >= PC302_GPIO_PIN_ARM_0 &&
         block_pin <= PC302_GPIO_PIN_ARM_7 )
    {
        port_ds = PC302_GPIO_BASE + GPIO_EXT_PORT_A_REG_OFFSET;
        pin_offset = block_pin;
    }
    else if ( block_pin >= ( PC302_GPIO_PIN_ARM_8 -
                             PC302_ARM_GPIO_PORT_WIDTH ) &&
              block_pin <= ( PC302_GPIO_PIN_ARM_15 -
                             PC302_ARM_GPIO_PORT_WIDTH ) )
    {
        port_ds = PC302_GPIO_BASE + GPIO_EXT_PORT_B_REG_OFFSET;
        pin_offset = block_pin - PC302_ARM_GPIO_PORT_WIDTH;
    }
    else
    {
        printf( "%s : cannot set value of ARM GPIO pin (%d)\n",
                __FUNCTION__, block_pin );
        return -ENXIO;
    }

    val = pc302_read_from_register( port_ds );

    return !!( val & ( 1 << pin_offset ) );
}

static int
pc302gpio_sd_set_direction( struct pc302gpio_pin_allocation *pin,
                            int input )
{
    int ret;
    u16 data;
    unsigned block_pin;

    /* Set the pin to be controlled by the configuration bus. */
    block_pin = pc302gpio_pin_to_block_pin( pin );

    ret = pc302_config_read( PC302_AXI2PICO_CAEID,
                             PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                             1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to read config register for SDGPIO \
                pin %u\n", __FUNCTION__, block_pin );
        return -EIO;
    }

    data &= PC302_GPIO_SD_CONFIG_CS_MASK;
    ret = pc302_config_write( PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to write config register for SDGPIO \
                pin %u\n", __FUNCTION__, block_pin );
        return -EIO;
    }

    /* Configure the pin to drive or not drive the output as appropriate.
    */
    ret = pc302_config_read( PC302_AXI2PICO_CAEID,
                             PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to read SDGPIO control value register\n",
                __FUNCTION__ );
        return -EIO;
    }

    if ( input )
        data &= ~( 1 << block_pin );
    else
        data |= ( 1 << block_pin );

    ret = pc302_config_write( PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to write control value register for SDGPIO \
                pin %u\n", __FUNCTION__, block_pin );
        return -EIO;
    }

    return 0;
}

static int
pc302gpio_arm_set_direction( struct pc302gpio_pin_allocation *pin,
                             int input )
{
    unsigned block_pin;
    unsigned int port_ddr;
    unsigned int port_cr;
    u32 val;
    u32 pin_offset;

    /* Set the pin to be controlled by the configuration bus. */
    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( block_pin >= PC302_GPIO_PIN_ARM_0 &&
         block_pin <= PC302_GPIO_PIN_ARM_7 )
    {
        port_ddr = PC302_GPIO_BASE + GPIO_SW_PORT_A_DDR_REG_OFFSET;
        port_cr = PC302_GPIO_BASE + GPIO_SW_PORT_A_CTL_REG_OFFSET;
        pin_offset = block_pin;
    }
    else if ( block_pin >= ( PC302_GPIO_PIN_ARM_8 -
                             PC302_ARM_GPIO_PORT_WIDTH ) &&
              block_pin <= ( PC302_GPIO_PIN_ARM_15 -
                             PC302_ARM_GPIO_PORT_WIDTH ) )
    {
        port_ddr = PC302_GPIO_BASE + GPIO_SW_PORT_B_DDR_REG_OFFSET;
        port_cr = PC302_GPIO_BASE + GPIO_SW_PORT_B_CTL_REG_OFFSET;
        pin_offset = block_pin - PC302_ARM_GPIO_PORT_WIDTH;
    }
    else
    {
        printf( "%s : cannot set direction of ARM GPIO pin (%d)\n",
                __FUNCTION__, block_pin );
        return -ENXIO;
    }

    /* Set the direction register (a bit set indicates output). */
    val = pc302_read_from_register( port_ddr );
    if ( input )
        val &= ~( 1 << pin_offset );
    else
        val |= ( 1 << pin_offset );
    pc302_write_to_register( port_ddr, val );

    /* Set the control register for the pin to be software controlled. */
    val = pc302_read_from_register( port_cr );
    val &= ~( 1 << pin_offset );
    pc302_write_to_register( port_cr, val );

    return 0;
}

static int
pc302gpio_arm_set_value( struct pc302gpio_pin_allocation *pin,
                         int value )
{
    unsigned block_pin;
    u32 val;
    u32 pin_offset;
    unsigned int port_dr;
    BUG_ON( NULL == pin );

    if ( pin->is_input )
        return -EINVAL;

    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( block_pin >= PC302_GPIO_PIN_ARM_0 &&
         block_pin <= PC302_GPIO_PIN_ARM_7 )
    {
        port_dr = PC302_GPIO_BASE + GPIO_SW_PORT_A_DR_REG_OFFSET;
        pin_offset = block_pin;
    }
    else if ( block_pin >= ( PC302_GPIO_PIN_ARM_8 -
                             PC302_ARM_GPIO_PORT_WIDTH ) &&
              block_pin <= ( PC302_GPIO_PIN_ARM_15 -
                             PC302_ARM_GPIO_PORT_WIDTH ) )
    {
        port_dr = PC302_GPIO_BASE + GPIO_SW_PORT_B_DR_REG_OFFSET;
        pin_offset = block_pin - PC302_ARM_GPIO_PORT_WIDTH;
    }
    else
    {
        printf( "%s : cannot set value of ARM GPIO pin (%d)\n",
                __FUNCTION__, block_pin );
        return -ENXIO;
    }

    val = pc302_read_from_register( port_dr );
    val &= ~( 1 << pin_offset );
    val |= ( !!value << pin_offset );

    pc302_write_to_register( port_dr, val );

    return 0;
}

static int
pc302gpio_set_direction( struct pc302gpio_pin_allocation *pin,
                         int input )
{
    int ret;
    pc302gpio_pin_type ptype;
    BUG_ON( NULL == pin );

    ptype = pc302gpio_get_pin_type( pin );

    if ( PC302_GPIO_ARM == ptype )
        ret = pc302gpio_arm_set_direction( pin, input );
    else if ( PC302_GPIO_SD == ptype )
        ret = pc302gpio_sd_set_direction( pin, input );
    else
        ret = -EINVAL;

    if ( !ret )
        pin->is_input = !!input;

    return ret;
}

static int
pc302gpio_set_value( struct pc302gpio_pin_allocation *pin,
                     int value )
{
    int ret;
    pc302gpio_pin_type ptype;
    BUG_ON( NULL == pin );

    ptype = pc302gpio_get_pin_type( pin );

    if ( PC302_GPIO_ARM == ptype )
        ret = pc302gpio_arm_set_value( pin, value );
    else if ( PC302_GPIO_SD == ptype )
        ret = pc302gpio_sd_set_value( pin, value );
    else
        ret = -EINVAL;

    if ( !ret && !pin->a_not_d )
    {
        /* Digital mode */
        pin->value = !!value;
    }
    else
    {
        /* Analogue mode */
        pin->value = value;
    }
    return ret;
}

static int
pc302gpio_get_value( struct pc302gpio_pin_allocation *pin )
{
    int ret;
    pc302gpio_pin_type ptype;
    BUG_ON( NULL == pin );

    ptype = pc302gpio_get_pin_type( pin );

    if ( PC302_GPIO_ARM == ptype )
        ret = pc302gpio_arm_get_value( pin );
    else if ( PC302_GPIO_SD == ptype )
        ret = pc302gpio_sd_get_value( pin );
    else
        ret = -EINVAL;

    if ( ret >= 0 && !pin->a_not_d )
    {
        /* Digital mode */
        pin->value = !!ret;
    }
    else
    {
        /* Analogue mode */
        pin->value = ret;
    }
    return ret;
}

static struct pc302gpio_pin_allocation *
pc302gpio_find_pin( unsigned gpio )
{
    int i;

    for ( i = 0 ; i < PC302_GPIO_NUM_PADS; ++i )
    {
        if ( pc302gpio_pads[ i ] && pc302gpio_pads[ i ]->pin_num == gpio )
            return pc302gpio_pads[ i ];
    }

    return NULL;
}

static int
pc302gpio_pai_muxing( picoifGpioPinNum_t pin )
{
    int ret= -EINVAL;

    u16 data = PC302_PAI_WAKE_UP;

    /* We are about to setup a gpio mux in the PAI block,
     * therefore before we do anything else we need to wake up
     * the PAI block. */
    ret = pc302_config_write( PC302_PAI_CAEID,
                              PC302_PAI_SLEEP_REG, 1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to wake up the PAI block.\n", __FUNCTION__ );
        return ret;
    }

    /* Read back the current value of the pai_io_ctrl register */
    ret = pc302_config_read( PC302_PAI_CAEID,
                             PC302_PAI_IO_CTRL_REG, 1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to read from the PAI io_ctrl register.\n",
                __FUNCTION__ );
        return -EIO;
    }

    /* Make sure we have valid data */
    data &= PC302_PAI_IO_CTRL_REG_MASK;

    if ( ( pin >= PC302_GPIO_PIN_ARM_4 ) &&
         ( pin <= PC302_GPIO_PIN_ARM_7 ) )
    {
        /* We are muxing an ARM gpio pin */
        switch ( pin )
        {
            case PC302_GPIO_PIN_ARM_4:
                data |= PAI_GPIO_PIN_ARM_4;
                break;

            case PC302_GPIO_PIN_ARM_5:
                data |= PAI_GPIO_PIN_ARM_5;
                break;

            case PC302_GPIO_PIN_ARM_6:
                data |= PAI_GPIO_PIN_ARM_6;
                break;

            case PC302_GPIO_PIN_ARM_7:
                data |= PAI_GPIO_PIN_ARM_7;
                break;

            default:
                break;
        }
        /* Write the updated value to the pai_io_ctrl register */
        ret = pc302_config_write( PC302_PAI_CAEID,
                                  PC302_PAI_IO_CTRL_REG, 1, &data );
        if ( 1 != ret )
        {
            printf( "%s : failed to write to the PAI "
                    "io_ctrl register.\n", __FUNCTION__ );
            return ret;
        }
    }
    else if ( ( pin >= PC302_GPIO_PIN_SDGPIO_4 ) &&
              ( pin <= PC302_GPIO_PIN_SDGPIO_7 ) )
    {
        /* We are muxing an sd gpio pin */
        switch ( pin )
        {
            case PC302_GPIO_PIN_SDGPIO_4:
                data |= PAI_SDGPIO_PIN_ARM_4;
                break;

            case PC302_GPIO_PIN_SDGPIO_5:
                data |= PAI_SDGPIO_PIN_ARM_5;
                break;

            case PC302_GPIO_PIN_SDGPIO_6:
                data |= PAI_SDGPIO_PIN_ARM_6;
                break;

            case PC302_GPIO_PIN_SDGPIO_7:
                data |= PAI_SDGPIO_PIN_ARM_7;
                break;

            default:
                break;
        }
        /* Write the updated value to the pai_io_ctrl register */
        ret = pc302_config_write( PC302_PAI_CAEID,
                                  PC302_PAI_IO_CTRL_REG, 1, &data );
        if ( 1 != ret )
        {
            printf( "%s : failed to write to the PAI "
                    "io_ctrl register.\n", __FUNCTION__ );
            return ret;
        }
    }
    else
    {
        /* We have been called with an out of range pin specified */
        printf( "%s : pc302gpio: out of range gpio for "
                "pai muxing.\n", __FUNCTION__ );
        return  -EINVAL;
    }

    return 0;
}

static u32
syscfg_read(void)
{
    return (u32)pc302_read_from_register( PC302_AXI2CFG_BASE +
                                          AXI2CFG_SYS_CONFIG_REG_OFFSET );
}

static void
syscfg_update(u32 mask,
              u32 val)
{
    u32 tmp = syscfg_read();
    tmp &= ~mask;
    tmp |= (val & mask);
    pc302_write_to_register( ( PC302_AXI2CFG_BASE +
                               AXI2CFG_SYS_CONFIG_REG_OFFSET ),
                              (unsigned int)tmp );
}

/* Public Functions -------------------------------------------------------- */
int
pc302_gpio_request( unsigned gpio,
                    const char *label )
{
    int pad_num;
    int ret = 0;
    struct pc302gpio_pin_allocation *pin;
    pc302gpio_pin_type ptype;

    pad_num = pc302gpio_get_pad( gpio );

    ret = -EINVAL;
    if ( pad_num < 0 )
        goto out;

    /* If the pad is already in use then fail. */
    ret = -EBUSY;
    if ( pc302gpio_pads[ pad_num ] )
        goto out;

    ret = -EIO;
    if ( set_pin_mux( gpio ) )
        goto out;

    pc302gpio_pads[ pad_num ] = pc302gpio_alloc_pin( label );
    pc302gpio_pads[ pad_num ]->pin_num = gpio;
    pc302gpio_pads[ pad_num ]->pad = pad_num;

    pin = pc302gpio_find_pin( gpio );
    ptype = pc302gpio_get_pin_type( pin );

    ret = 0;

out:

    return ret;
}

void
pc302_gpio_free( unsigned gpio )
{
    unsigned i;
    unsigned found = 0;
    struct pc302gpio_pin_allocation *pin;
    pc302gpio_pin_type ptype;


    pin = pc302gpio_find_pin( gpio );
    if (!pin)
        return;
    ptype = pc302gpio_get_pin_type( pin );

    /* Find the pin. */
    for ( i = 0; i < PC302_GPIO_NUM_PADS; ++i )
    {
        if ( pc302gpio_pads[ i ] && pc302gpio_pads[ i ]->pin_num == gpio )
        {
            found = 1;
            break;
        }
    }

    if ( !found )
        return;

    free( pc302gpio_pads[ i ] );
    pc302gpio_pads[ i ] = NULL;
}

int
pc302_gpio_direction_input( unsigned gpio )
{
    int pad_num;
    struct pc302gpio_pin_allocation *pin;
    int ret;

    pad_num = pc302gpio_get_pad( gpio );
    if ( pad_num < 0 )
    {
        ret = pad_num;
        goto out;
    }

    pin = pc302gpio_pads[ pad_num ];

    ret = -ENXIO;
    if ( !pin )
        goto out;

    pin->is_input = 1;
    ret = pc302gpio_set_direction( pin, 1 );

out:
    return ret;
}

int
pc302_gpio_direction_output( unsigned gpio,
                             int value )
{
    int pad_num;
    struct pc302gpio_pin_allocation *pin;
    int ret;

    pad_num = pc302gpio_get_pad( gpio );
    if ( pad_num < 0 )
    {
        ret = pad_num;
        goto out;
    }

    pin = pc302gpio_pads[ pad_num ];

    ret = -ENXIO;
    if ( !pin )
        goto out;

    pin->is_input = 0;
    ret = pc302gpio_set_direction( pin, 0 );
    if ( 0 != ret )
    {
        goto out;
    }
    else
    {
        ret = pc302gpio_set_value( pin, value );
    }

out:
    return ret;
}

int
pc302_gpio_set_direction( unsigned gpio,
                          int dir )
{
    struct pc302gpio_pin_allocation *pin;
    int ret;

    pin = pc302gpio_find_pin( gpio );

    ret = -ENXIO;
    if ( !pin )
        goto out;

    pin->is_input = dir;
    ret = pc302gpio_set_direction( pin, dir );

out:
    return ret;
}

int
pc302_gpio_set_value( unsigned gpio,
                      int value )
{
    struct pc302gpio_pin_allocation *pin;
    int ret;

    pin = pc302gpio_find_pin( gpio );

    ret = -EINVAL;
    if ( !pin )
        goto out;

    ret = pc302gpio_set_value( pin, value );

out:
    return ret;
}

int
pc302_gpio_get_value( unsigned gpio )
{
    struct pc302gpio_pin_allocation *pin;
    int ret;

    pin = pc302gpio_find_pin( gpio );

    ret = -EINVAL;
    if ( !pin )
        goto out;

    ret = pc302gpio_get_value( pin );
out:
    return ret;
}

int
pc302_gpio_configure_dac( unsigned gpio,
                          u8 converter_size,
                          u16 analogue_rate )
{
    int ret;
    u16 data;
    unsigned block_pin;
    struct pc302gpio_pin_allocation *pin = pc302gpio_find_pin( gpio );

    if ( !pin )
        return -EINVAL;

    /* Set the pin to be controlled by the configuration bus. */
    block_pin = pc302gpio_pin_to_block_pin( pin );

    ret = pc302_config_read( PC302_AXI2PICO_CAEID,
                             PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                             1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to read config register for SDGPIO \
                pin %u\n", __FUNCTION__, block_pin );
        return -EIO;
    }

    data &= PC302_GPIO_SD_CONFIG_CS_MASK;
    data &= ~PC302_GPIO_SD_CONV_SZ_MASK;
    data |= ( PC302_GPIO_SD_CONFIG_AND |
            ( converter_size & PC302_GPIO_SD_CONV_SZ_MASK ) );

    ret = pc302_config_write( PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to write config register for SDGPIO \
                pin %u\n", __FUNCTION__, block_pin );
        return -EIO;
    }

    /* Configure the pin to drive the output. */
    ret = pc302_config_read( PC302_AXI2PICO_CAEID,
                             PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to read SDGPIO control value register\n",
                __FUNCTION__ );
        return -EIO;
    }

    data |= ( 1 << block_pin );

    ret = pc302_config_write( PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to write control value register for SDGPIO \
                pin %u\n", __FUNCTION__, block_pin );
        return -EIO;
    }

    /* Write the Analogue rate register */
    data = analogue_rate;
    ret = pc302_config_write( PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_ANALOGUE_RATE( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printf( "%s : failed to write analogue rate register for SDGPIO \
                pin %u\n", __FUNCTION__, block_pin );
        return -EIO;
    }

    /* Set the a_not_d flag in the pin allocation structure */
    pin->a_not_d = 1;

    return 0;
}

int
pc302_gpio_init( void )
{
    int ret;

    u16 data = PC302_AXI2PICO_WAKE_UP;

    /* Get the current boot mode. */
    u32 syscfg = syscfg_read();
    boot_mode = ( syscfg & AXI2CFG_SYS_CONFIG_BOOT_MODE_MASK ) >>
                AXI2CFG_SYS_CONFIG_BOOT_MODE_LO;

    /* This driver can use the sdgpio, therefore before we do anything
     * else we need to wake up the AXI2PICO block. */
    ret = pc302_config_write( PC302_AXI2PICO_CAEID,
                              PC302_AXI2PICO_SLEEP_REG, 1, &data );
    if ( 1 != ret )
    {
        /* Config write has failed */
        printf( "%s : failed to wake up the AXI2PICO block.\n", __FUNCTION__ );
        return -EIO;
    }

    return 0;
}
