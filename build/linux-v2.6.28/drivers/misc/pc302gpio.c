/**
 * \file pc302gpio.c
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright 2008 picoChip Designs LTD, All Rights Reserved.
 * http://www.picochip.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * This file implements a driver for the GPIO pins on the picoChip PC302
 * device. This includes both the ARM and SD GPIO. We provide 3 APIs for
 * accessing the GPIO pins:
 *   - kernel space with the GPIO guidelines (Documentation/gpio.txt)
 *   - userspace through configfs
 *     (Documentation/filesystems/configfs/configfs.txt)
 *   - userspace through an ioctl() interface
 */

#include <linux/module.h>
#include <linux/configfs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/picochip/picoif.h>
#include <linux/picochip/gpio.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/picochip/devices/pc302.h>
#include <linux/picochip/gpio_ioctl.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <mach/irqs.h>
#include <mach/pc302/gpio.h>
#include <mach/pc302/axi2cfg.h>
#include <mach/pc302/pc302.h>
#include <mach/pc302/vic.h>
#include <mach/hardware.h>
#include <mach/reset.h>
 
#define  PC302_RESET_OUTPUT_PIN  PC302_GPIO_PIN_ARM_1

#define CARDNAME "pc302gpio"
#define PC302_GPIO_MINOR    ( 243 )

/** A name for this module */
#define TITLE "PC302 GPIO Driver"

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
#define PC302_GPIO_SD_CONFIG_CS       ( 1 << 15 )
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

static int pc302gpio_open( struct inode *inode,
                           struct file *filp );

static int pc302gpio_release( struct inode *inode,
                              struct file *filp );

static int pc302gpio_ioctl( struct inode *inode,
                            struct file *filp,
                            unsigned int cmd,
                            unsigned long arg );

/*****************************************************************************
 * Data types.                                                               *
 *****************************************************************************/

/**
 * The file operations for the GPIO driver.
 */
static const struct file_operations pc302gpio_fops = {
    .open       = pc302gpio_open,
    .release    = pc302gpio_release,
    .ioctl      = pc302gpio_ioctl,
};

/** Private data storage for this module. */
static struct {

#ifdef CONFIG_DEBUG_FS
    /**
     * The directory entry for debugfs if we are using it.
     */
    struct dentry *debugfs_dir;

    /**
     * The list of all active GPIOs if we are using debugfs.
     */
    struct dentry *debugfs_file;
#endif /* CONFIG_DEBUG_FS */

    struct platform_device *pdev;
    struct platform_driver *pdrv;
    struct miscdevice dev;
    void __iomem *mem_region;
    spinlock_t lock;

} pc302gpio_priv = {
    .dev    = {
        .minor  = PC302_GPIO_MINOR,
        .name   = "gpio",
        .fops   = &pc302gpio_fops,
    },
    .lock       = __SPIN_LOCK_UNLOCKED( pc302gpio_priv_lock ),
};

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

    /** Boolean flag for a dynamic direction GPIO.  Previously the
     * default state of a GPIO when it was requested, as recorded in
     * the is_input flag, was always an output, even if the HW was
     * actually set to be an input.  That made it possible for user
     * code to request the pin, set an initial output value and then
     * switch the direction to output.  When we add code to set the
     * value of is_input from the actual HW state, it breaks code that
     * uses that idiom, since writing a value to a GPIO with is_input
     * set is EINVAL.  To work around this we introduce this flag that
     * allows output values to be set between requesting a GPIO and the
     * operation that sets the desired direction.
     */
    int direction_unset;

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

#ifdef CONFIG_CONFIGFS_FS
/** Structure to store the state of a pin in configfs. */
struct pc302gpio_cfs_pin
{
    /** The configfs entry. */
    struct config_item item;

    /** The pin allocation. */
    struct pc302gpio_pin_allocation *phys_pin;
};

/** Structure for setting and getting attributes through configfs. */
struct pc302gpio_pin_attr
{
    /** The attribute itself. */
    struct configfs_attribute attr;

    ssize_t ( *show )( struct pc302gpio_cfs_pin *pin, char *buf );
    ssize_t ( *store )( struct pc302gpio_cfs_pin *pin, const char *buf,
                        size_t count );
};
#endif /* CONFIG_CONFIGFS_FS */

/**
 * The current boot mode.
 */
static u32 boot_mode = 1;

static struct pc302gpio_pin_allocation *pc302gpio_pads[ PC302_GPIO_NUM_PADS ];

/*****************************************************************************
 * Private function prototypes.                                              *
 *****************************************************************************/

static int
pc302gpio_get_pad( picoifGpioPinNum_t pin );

static struct pc302gpio_pin_allocation *
pc302gpio_alloc_pin( const char *name );

static void
pc302gpio_free_pin( struct pc302gpio_pin_allocation *p );

static int
pc302gpio_set_direction( struct pc302gpio_pin_allocation *pin,
                         int input );

static pc302gpio_pin_type
pc302gpio_get_pin_type( struct pc302gpio_pin_allocation *pin );

static struct pc302gpio_pin_allocation *
pc302gpio_find_pin( unsigned gpio );

static int
pc302gpio_pin_to_block_pin( struct pc302gpio_pin_allocation *pin );

static int
pc302gpio_set_value( struct pc302gpio_pin_allocation *pin,
                     int value );

static int
pc302gpio_get_value( struct pc302gpio_pin_allocation *pin );

static int pc302gpio_configure_dac( unsigned gpio,
                                    u8 converter_size,
                                    u16 analogue_rate );

static int pc302gpio_pai_muxing( picoifGpioPinNum_t pin );

static int
pc302gpio_arm_get_direction( struct pc302gpio_pin_allocation *pin );

static int
pc302gpio_sd_get_direction( struct pc302gpio_pin_allocation *pin );


#ifdef CONFIG_CONFIGFS_FS
ssize_t	pc302gpio_cfs_show_attribute( struct config_item *,
                                      struct configfs_attribute *,
                                      char * );

ssize_t	pc302gpio_cfs_store_attribute( struct config_item *,
                                       struct configfs_attribute *,
                                       const char *,
                                       size_t );
#endif /* CONFIG_CONFIGFS_FS */

/*****************************************************************************
 * Implementation.                                                           *
 *****************************************************************************/

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

/**
 * Check if a GPIO pin number is a valid GPIO pin in this system.
 *
 * \param gpio The GPIO pin number to query.
 * \return Returns 1 if gpio is a valid pin, 0 otherwise.
 */
int
gpio_is_valid( int gpio )
{
    return ( gpio >= PC302_GPIO_PIN_ARM_0 && gpio <= PC302_GPIO_PIN_SDGPIO_15 );
}
EXPORT_SYMBOL( gpio_is_valid );

/**
 * Set the pin multiplexing for shared GPIO pins.
 *
 * \param gpio The pin to set the multiplex value of.
 * \return Returns zero on success or if the multiplexing does not need to be
 * set, non-zero on failure.
 */
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

    spin_lock( &pc302gpio_priv.lock );
    val = syscfg_read();
    val &=
        ~( 1 << ( shared_pin_num + AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_LO ) );
    if ( is_arm )
        val |= ( 1 << ( shared_pin_num +
                        AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_SEL_LO ) );
    syscfg_update( AXI2CFG_SYS_CONFIG_SD_ARM_GPIO_MASK, val );
    spin_unlock( &pc302gpio_priv.lock );
    return 0;
}

/**
 * Set the value of the "Soft Reset Lock" bit of an SDGPIO. If this is set, the
 * SDGPIO will not be reset along with the rest of the picoArray.
 *
 * \param pin The pin to configure
 * \param value one to set the bit, zero to clear it
 */
static int
pc302gpio_sd_reset_conifg( struct pc302gpio_pin_allocation *pin, int value )
{
    int ret, block_pin;
    u16 data;

    block_pin = pc302gpio_pin_to_block_pin( pin );
    ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read config register for SDGPIO \
                pin %u\n", block_pin );
        return -1;
    }

    if (value)
        data |= PC302_GPIO_SD_CONFIG_SR_LOCK;
    else
        data &= ~PC302_GPIO_SD_CONFIG_SR_LOCK;
    ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                               PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                               1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to write config register for SDGPIO \
                pin %u\n", block_pin );
        return -1;
    }

    return 0;
}

/**
 * Request a new GPIO pin. This implements part of the Linux GPIO guidelines.
 *
 * \param gpio The pin to request.
 * \param label The name of the pin - this only serves as a tag for debugging
 * so can be anything.
 * \return Returns zero on success, non-zero on failure.
 */
int
gpio_request( unsigned gpio,
              const char *label )
{
    int pad_num;
    int ret = 0;
    struct pc302gpio_pin_allocation *pin;
    pc302gpio_pin_type ptype;

    spin_lock( &pc302gpio_priv.lock );
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
    pc302gpio_pads[ pad_num ]->direction_unset = 1;

    pin = pc302gpio_find_pin( gpio );
    ptype = pc302gpio_get_pin_type( pin );
    if ( PC302_GPIO_SD == ptype )
    {
        /* Set the reset lock bit on SDGPIOs
         * Note: Once set these bits never get reset by this
         *       driver.
         */
        ret = pc302gpio_sd_reset_conifg( pin, 1 );
        if ( ret )
        {
            ret = -EIO;
            goto out;
        }
        
        /* Read the initial direction from the HW */
        ret = pc302gpio_sd_get_direction( pin );
        if (ret < 0)
        {
            ret = -EIO;
            goto out;
        }

        pc302gpio_pads[ pad_num ]->is_input = !!ret;
    }
    else if ( PC302_GPIO_ARM == ptype )
    {
        /* Read the initial direction from the HW */
        ret = pc302gpio_arm_get_direction( pin );
        if (ret < 0)
        {
            ret = -EIO;
            goto out;
        }
        
        pc302gpio_pads[ pad_num ]->is_input = !!ret;
    }

    ret = 0;

out:
    spin_unlock( &pc302gpio_priv.lock );

    return ret;
}
EXPORT_SYMBOL( gpio_request );

/**
 * Free a GPIO pin previously requested with gpio_request().
 *
 * \param gpio The GPIO pin to free.
 */
void
gpio_free( unsigned gpio )
{
    unsigned i;
    unsigned found = 0;
    struct pc302gpio_pin_allocation *pin;
    pc302gpio_pin_type ptype;

    spin_lock( &pc302gpio_priv.lock );

    pin = pc302gpio_find_pin( gpio );
    if (!pin)
	    goto out;
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
        goto out;

    kfree( pc302gpio_pads[ i ] );
    pc302gpio_pads[ i ] = NULL;

out:
    spin_unlock( &pc302gpio_priv.lock );
}
EXPORT_SYMBOL( gpio_free );

/**
 * Set the direction of a GPIO pin requested with gpio_request() to be an
 * input.
 *
 * \param gpio The GPIO pin to configure.
 * \return Returns zero on success, non-zero on failure.
 */
int
gpio_direction_input( unsigned gpio )
{
    int pad_num;
    struct pc302gpio_pin_allocation *pin;
    int ret;

    spin_lock( &pc302gpio_priv.lock );
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
    pin->direction_unset = 0;
    ret = pc302gpio_set_direction( pin, 1 );

out:
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}
EXPORT_SYMBOL( gpio_direction_input );

/**
 * Set the direction of a GPIO pin requested with gpio_request() to be an
 * output.
 *
 * \param gpio The GPIO pin to configure.
 * \param value The initial output value for the gpio pin.
 * \return Returns zero on success, non-zero on failure.
 */
int
gpio_direction_output( unsigned gpio,
                       int value )
{
    int pad_num;
    struct pc302gpio_pin_allocation *pin;
    int ret;

    spin_lock( &pc302gpio_priv.lock );
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
    pin->direction_unset = 0;
    ret = pc302gpio_set_direction( pin, 0 );
    if ( 0 != ret )
    {
        goto out;
    }
    else if (value < 0)
    {
        /* Leave the value of the pin alone. */
        ret = 0;
    }
    else
    {
        ret = pc302gpio_set_value( pin, value );
    }

out:
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}
EXPORT_SYMBOL( gpio_direction_output );

/**
 * Check if a GPIO pin can sleep.
 *
 * \return Returns zero on success, non-zero on failure.
 */
int
gpio_cansleep( unsigned gpio )
{
    /* None of our GPIO pin accesses can sleep so always return 0. */
    return 0;
}
EXPORT_SYMBOL( gpio_cansleep );

/* We don't use gpiolib as it doesn't handle multiplexing of pins well so we
 * don't get the sysfs export helpers. Rather than leave the functions
 * undefined, always return an error. */
int
gpio_export( unsigned gpio,
             bool direction_may_change )
{
    return -ENODEV;
}
EXPORT_SYMBOL( gpio_export );

int
gpio_unexport( unsigned gpio )
{
    return -ENODEV;
}
EXPORT_SYMBOL( gpio_unexport );

/**
 * Given a GPIO pin number, find the IRQ that it can generate. This is only
 * applicable to ARM GPIO pins 0-7.
 *
 * \param gpio The GPIO pin to get the IRQ line for.
 * \return Returns the IRQ number on success, negative on failure.
 */
int
gpio_to_irq( unsigned gpio )
{
    /* Only ARM GPIO pins 0..7 can generate interrupts. */
    if ( !( gpio >= PC302_GPIO_PIN_ARM_0 && gpio <= PC302_GPIO_PIN_ARM_7 ) )
        return -EINVAL;

    return ( gpio - PC302_GPIO_PIN_ARM_0 ) + IRQ_GPIO0;
}
EXPORT_SYMBOL( gpio_to_irq );

/**
 * Given a IRQ line number, find the GPIO pin can generate it. This is only
 * applicable to ARM GPIO pins 0-7.
 *
 * \param irq The IRQ line that can be generated by the GPIO.
 * \return Returns the GPIO pin number on success, negative on failure.
 */
int
irq_to_gpio( unsigned irq )
{
    if ( !( irq >= IRQ_GPIO0 && irq <= IRQ_GPIO7 ) )
        return -EINVAL;

    return ( irq - IRQ_GPIO0 ) + PC302_GPIO_PIN_ARM_0;
}

/* None of our GPIOs can sleep and we could just implement these functions as
 * wrappers around the spinlock safe functions, things could change in the
 * future and its better to be explicit. Using these _cansleep() variants will
 * always return -EIO. */
int
gpio_get_value_cansleep( unsigned gpio )
{
    return -EIO;
}
EXPORT_SYMBOL( gpio_get_value_cansleep );

int
gpio_set_value_cansleep( unsigned gpio,
                         int value )
{
    return -EIO;
}
EXPORT_SYMBOL( gpio_set_value_cansleep );

/**
 * Set the value of a GPIO pin.
 *
 * \param gpio The number of the pin to set the value of.
 * \param value The value to set the pin to.
 */
int
gpio_set_value( unsigned gpio,
                int value )
{
    struct pc302gpio_pin_allocation *pin;
    int ret;

    spin_lock( &pc302gpio_priv.lock );
    pin = pc302gpio_find_pin( gpio );

    ret = -EINVAL;
    if ( !pin )
        goto out;

    ret = pc302gpio_set_value( pin, value );

out:
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}
EXPORT_SYMBOL( gpio_set_value );

/**
 * Get the value of a GPIO pin.
 *
 * \param gpio The number of the pin to get the value of.
 * \return Returns the value of the pin on success,
 * negative on failure.
 */
int
gpio_get_value( unsigned gpio )
{
    struct pc302gpio_pin_allocation *pin;
    int ret;

    spin_lock( &pc302gpio_priv.lock );
    pin = pc302gpio_find_pin( gpio );

    ret = -EINVAL;
    if ( !pin )
        goto out;

    ret = pc302gpio_get_value( pin );
out:
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}
EXPORT_SYMBOL( gpio_get_value );

/**
 * Free a GPIO pin that has previously been allocated.
 *
 * \param p The pin to free.
 */
static void
pc302gpio_free_pin( struct pc302gpio_pin_allocation *p )
{
    kfree( p );
}

/**
 * Allocate a GPIO pin state structure.
 *
 * \param name The name of the pin (this can be anything - it can be the
 * function of the pin or the application using it for example).
 * \return Returns a pointer to the pin on success, NULL on failure.
 */
static struct pc302gpio_pin_allocation *
pc302gpio_alloc_pin( const char *name )
{
    struct pc302gpio_pin_allocation *p =
        kzalloc( sizeof( *p ), GFP_KERNEL );

    if ( p )
    {
        snprintf( p->name, PC302_GPIO_PIN_NAME_MAX, "%s", name );
        p->pin_num = -1;
        p->pad = -1;
        p->enabled = 0;
        p->is_input = 0;
        p->direction_unset = 1;
        p->a_not_d = 0;
    }

    return p;
}

/**
 * Get the pin type.
 *
 * \param pin The pin to query.
 * \return Returns the type of GPIO pin.
 */
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

/**
 * Set the value of an output SDGPIO pin.
 *
 * \param pin The pin to set the value of.
 * \param value The value to set the pin to. Intepreted as non-zero == 1.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_sd_set_value( struct pc302gpio_pin_allocation *pin,
                        int value )
{
    unsigned block_pin;
    int ret;
    u16 data;
    BUG_ON( NULL == pin );

    /* Allow client to set value before setting direction for
     * the first time.  Client code does this to try to avoid
     * glitches when the output is enabled in HW.
     */
    if ( pin->is_input && !pin->direction_unset )
        return -EINVAL;

    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( !pin->a_not_d )
    {
        /* Digital mode */
        ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                                  PC302_GPIO_SD_OUTPUT_VAL_REG, 1, &data );
        if ( 1 != ret )
        {
            printk( KERN_ALERT "failed to read SDGPIO output value reg\n" );
            return -EIO;
        }

        data &= ~( 1 << block_pin );
        data |= ( !!value ) << block_pin;

        ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                PC302_GPIO_SD_OUTPUT_VAL_REG, 1, &data );
        if ( 1 != ret )
        {
            printk( KERN_ALERT "failed to output control register for SDGPIO"
                    "pin %u\n", block_pin );
            return -EIO;
        }
    }
    else
    {
        /* Analogue mode */
        data = (u16)value;
        ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                                   PC302_GPIO_SD_PIN_ANALOGUE_VAL( block_pin ),
                                   1, &data );
        if ( 1 != ret )
        {
            printk( KERN_ALERT "failed to write analogue value register for "
                    "SDGPIO pin %u\n", block_pin );
            return -EIO;
        }
    }

    return 0;
}

/**
 * Get the value of an input SDGPIO pin.
 *
 * \param pin The pin to get the value of.
 * \return Returns the value read on success, negative on failure.
 */
static int
pc302gpio_sd_get_value( struct pc302gpio_pin_allocation *pin )
{
    unsigned block_pin;
    int ret;
    u16 data;
    BUG_ON( NULL == pin );

    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( !pin->is_input && !pin->a_not_d )
    {
        /* Digital mode output.  Return the programmed output value.
         * This is needed to allow an app to read the state of an
         * output that's been set by another app.
         */
        ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                                  PC302_GPIO_SD_OUTPUT_VAL_REG, 1, &data );
        if ( 1 != ret )
        {
            printk( KERN_ALERT "failed to read SDGPIO output value reg\n" );
            return -EIO;
        }

        return !!( data & ( 1 << block_pin ) );
    }
    else if ( !pin->a_not_d )
    {
        /* Digital mode */
        ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                                  PC302_GPIO_SD_INPUT_VAL_REG, 1, &data );
        if ( 1 != ret )
        {
            printk( KERN_ALERT "failed to read SDGPIO input value reg\n" );
            return -EIO;
        }

        return !!( data & ( 1 << block_pin ) );
    }
    else
    {
        /* Analogue mode */
        ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                                  PC302_GPIO_SD_PIN_ANALOGUE_VAL( block_pin ),
                                  1, &data );
        if ( 1 != ret )
        {
            printk( KERN_ALERT "failed to read the analogue value register "
                    "for SDGPIO pin %u\n", block_pin );
            return -EIO;
        }

        return (int)data;
    }
}

/**
 * Get the value of an input ARM GPIO pin.
 *
 * \param pin The pin to get the value of.
 * \return Returns the value read on success, negative on failure.
 */
static int
pc302gpio_arm_get_value( struct pc302gpio_pin_allocation *pin )
{
    unsigned block_pin;
    void __iomem *port_ds;
    u32 pin_offset;
    u32 val;
    BUG_ON( NULL == pin );

    /* If the pin is an output then return the current output value */

    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( block_pin >= PC302_GPIO_PIN_ARM_0 &&
         block_pin <= PC302_GPIO_PIN_ARM_7 )
    {
        port_ds = pc302gpio_priv.mem_region + GPIO_EXT_PORT_A_REG_OFFSET;
        pin_offset = block_pin;
    }
    else if ( block_pin >= ( PC302_GPIO_PIN_ARM_8 -
                             PC302_ARM_GPIO_PORT_WIDTH ) &&
              block_pin <= ( PC302_GPIO_PIN_ARM_15 -
                             PC302_ARM_GPIO_PORT_WIDTH ) )
    {
        port_ds = pc302gpio_priv.mem_region + GPIO_EXT_PORT_B_REG_OFFSET;
        pin_offset = block_pin - PC302_ARM_GPIO_PORT_WIDTH;
    }
    else
    {
        printk( KERN_INFO "cannot get value of ARM GPIO pin (%d)\n",
                block_pin );
        return -ENXIO;
    }

    val = ioread32( port_ds );

    return !!( val & ( 1 << pin_offset ) );
}

/**
 * Set the direction of an SD-GPIO pin.
 *
 * \param pin The pin to set the direction of.
 * \param input Boolean flag to set the pin to input.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_sd_set_direction( struct pc302gpio_pin_allocation *pin,
                            int input )
{
    int ret;
    u16 data;
    unsigned block_pin;

    /* Set the pin to be controlled by the configuration bus. */
    block_pin = pc302gpio_pin_to_block_pin( pin );

    ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read config register for SDGPIO \
                pin %u\n", block_pin );
        return -EIO;
    }

    data &= PC302_GPIO_SD_CONFIG_CS_MASK;
    ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                               PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                               1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to write config register for SDGPIO \
                pin %u\n", block_pin );
        return -EIO;
    }

    /* Configure the pin to drive or not drive the output as appropriate.
    */
    ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read SDGPIO control value register\n" );
        return -EIO;
    }

    if ( input )
        data &= ~( 1 << block_pin );
    else
        data |= ( 1 << block_pin );

    ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                               PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to write control value register for SDGPIO \
                pin %u\n", block_pin );
        return -EIO;
    }

    /* If this is an input then there's no guarantee that the control reg will
     * be updated before we try to read the input unless we perform a dummy data
     * write.  See the register description, p7 of PC302 Programmers Guide
     * Appendix S.  That involves reading the register and writing the same value
     * back to avoid changing any output values.  Without this, the input is dead
     * until someone writes to an output.
     */
    ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                                PC302_GPIO_SD_OUTPUT_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read SDGPIO output value reg\n" );
        return -EIO;
    }

    ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
            PC302_GPIO_SD_OUTPUT_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to write output register for SDGPIO"
                "pin %u\n", block_pin );
        return -EIO;
    }
    return 0;
}

/**
 * Get the direction of an SD-GPIO pin.
 *
 * \param pin The pin to get the direction of.
 * \return Returns 0 for output, 1 for input, less than zero on failure.
 */
static int
pc302gpio_sd_get_direction( struct pc302gpio_pin_allocation *pin )
{
    int ret;
    u16 data;
    unsigned block_pin;

    /* Check if the pin is controlled by the picoBus. */
    block_pin = pc302gpio_pin_to_block_pin( pin );

    ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read config register for SDGPIO \
                pin %u\n", block_pin );
        return -EIO;
    }

    if (PC302_GPIO_SD_CONFIG_CS & data)
    {
        /* If controlled by picoBus then essentially an input for us. */
        return 1;
    }

    /* Read the direction bit of the configuration bus programming. */
    ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read SDGPIO control value register\n" );
        return -EIO;
    }

    if ( data & ( 1 << block_pin ) )
        return 0;
    else
        return 1;
}

/**
 * Set the direction of an ARM GPIO pin.
 *
 * \param pin The pin to set the direction of.
 * \param input Boolean flag to set the pin to input.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_arm_set_direction( struct pc302gpio_pin_allocation *pin,
                             int input )
{
    unsigned block_pin;
    void __iomem *port_ddr;
    void __iomem *port_cr;
    u32 val;
    u32 pin_offset;

    /* Set the pin to be controlled by the configuration bus. */
    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( block_pin >= PC302_GPIO_PIN_ARM_0 &&
         block_pin <= PC302_GPIO_PIN_ARM_7 )
    {
        port_ddr = pc302gpio_priv.mem_region + GPIO_SW_PORT_A_DDR_REG_OFFSET;
        port_cr = pc302gpio_priv.mem_region + GPIO_SW_PORT_A_CTL_REG_OFFSET;
        pin_offset = block_pin;
    }
    else if ( block_pin >= ( PC302_GPIO_PIN_ARM_8 -
                             PC302_ARM_GPIO_PORT_WIDTH ) &&
              block_pin <= ( PC302_GPIO_PIN_ARM_15 -
                             PC302_ARM_GPIO_PORT_WIDTH ) )
    {
        port_ddr = pc302gpio_priv.mem_region + GPIO_SW_PORT_B_DDR_REG_OFFSET;
        port_cr = pc302gpio_priv.mem_region + GPIO_SW_PORT_B_CTL_REG_OFFSET;
        pin_offset = block_pin - PC302_ARM_GPIO_PORT_WIDTH;
    }
    else
    {
        printk( KERN_INFO "cannot set direction of ARM GPIO pin (%d)\n",
                block_pin );
        return -ENXIO;
    }

    /* Set the direction register (a bit set indicates output). */
    val = ioread32( port_ddr );
    if ( input )
        val &= ~( 1 << pin_offset );
    else
        val |= ( 1 << pin_offset );
    iowrite32( val, port_ddr );

    /* Set the control register for the pin to be software controlled. */
    val = ioread32( port_cr );
    val &= ~( 1 << pin_offset );
    iowrite32( val, port_cr );

    return 0;
}

/**
 * Get the direction of an ARM GPIO pin.
 *
 * \param pin The pin to get the direction of.
 * \return Returns 0 for output, 1 for input, less than zero on failure.
 */
static int
pc302gpio_arm_get_direction( struct pc302gpio_pin_allocation *pin )
{
    unsigned block_pin;
    void __iomem *port_ddr;
    void __iomem *port_cr;
    u32 val;
    u32 pin_offset;

    /* Get the pin number. */
    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( block_pin >= PC302_GPIO_PIN_ARM_0 &&
         block_pin <= PC302_GPIO_PIN_ARM_7 )
    {
        port_ddr = pc302gpio_priv.mem_region + GPIO_SW_PORT_A_DDR_REG_OFFSET;
        port_cr = pc302gpio_priv.mem_region + GPIO_SW_PORT_A_CTL_REG_OFFSET;
        pin_offset = block_pin;
    }
    else if ( block_pin >= ( PC302_GPIO_PIN_ARM_8 -
                             PC302_ARM_GPIO_PORT_WIDTH ) &&
              block_pin <= ( PC302_GPIO_PIN_ARM_15 -
                             PC302_ARM_GPIO_PORT_WIDTH ) )
    {
        port_ddr = pc302gpio_priv.mem_region + GPIO_SW_PORT_B_DDR_REG_OFFSET;
        port_cr = pc302gpio_priv.mem_region + GPIO_SW_PORT_B_CTL_REG_OFFSET;
        pin_offset = block_pin - PC302_ARM_GPIO_PORT_WIDTH;
    }
    else
    {
        printk( KERN_INFO "cannot get direction of ARM GPIO pin (%d)\n",
                block_pin );
        return -ENXIO;
    }

    /* Get the direction register (a bit set indicates output). */
    val = ioread32( port_ddr );
    if ( val & ( 1 << pin_offset ) )
        return 0;
    else
        return 1;
}

/**
 * Set the value of an output ARM GPIO pin.
 *
 * \param pin The pin to set the value of.
 * \param value The value to set the pin to. Intepreted as non-zero == 1.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_arm_set_value( struct pc302gpio_pin_allocation *pin,
                         int value )
{
    unsigned block_pin;
    u32 val;
    u32 pin_offset;
    void __iomem *port_dr;
    BUG_ON( NULL == pin );

    /* Allow client to set value before setting direction for
     * the first time.  Client code does this to try to avoid
     * glitches when the output is enabled in HW.
     */
    if ( pin->is_input && !pin->direction_unset )
        return -EINVAL;

    block_pin = pc302gpio_pin_to_block_pin( pin );

    if ( block_pin >= PC302_GPIO_PIN_ARM_0 &&
         block_pin <= PC302_GPIO_PIN_ARM_7 )
    {
        port_dr = pc302gpio_priv.mem_region + GPIO_SW_PORT_A_DR_REG_OFFSET;
        pin_offset = block_pin;
    }
    else if ( block_pin >= ( PC302_GPIO_PIN_ARM_8 -
                             PC302_ARM_GPIO_PORT_WIDTH ) &&
              block_pin <= ( PC302_GPIO_PIN_ARM_15 -
                             PC302_ARM_GPIO_PORT_WIDTH ) )
    {
        port_dr = pc302gpio_priv.mem_region + GPIO_SW_PORT_B_DR_REG_OFFSET;
        pin_offset = block_pin - PC302_ARM_GPIO_PORT_WIDTH;
    }
    else
    {
        printk( KERN_INFO "cannot set value of ARM GPIO pin (%d)\n",
                block_pin );
        return -ENXIO;
    }

    val = ioread32( port_dr );
    val &= ~( 1 << pin_offset );
    val |= ( !!value << pin_offset );

    iowrite32( val, port_dr );

    return 0;
}

/**
 * Set the direction of a GPIO pin.
 *
 * \param pin The pin to set the direction of.
 * \param input Flag for input GPIO pins.
 * \return Returns zero on success, non-zero on failure.
 */
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
    {
        pin->is_input = !!input;
        pin->direction_unset = 0;
    }

    return ret;
}

/**
 * Set the value of a GPIO pin.
 *
 * \param pin The pin to set the value of.
 * \param value The value to set.
 * \return Returns zero on success, non-zero on failure.
 */
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

/**
 * Get the value of a GPIO pin.
 *
 * \param pin The pin to get the value of.
 * \return Returns the value of the pin on success,
 * negative on failure.
 */
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

/**
 * Given a GPIO pin number, find the PC302 representation of it.
 *
 * \param gpio The GPIO pin number.
 * \return Returns a pointer to the internal representation.
 */
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
pc302gpio_configure_dac( unsigned gpio,
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

    ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read config register for SDGPIO \
                pin %u\n", block_pin );
        return -EIO;
    }

    data &= PC302_GPIO_SD_CONFIG_CS_MASK;
    data &= ~PC302_GPIO_SD_CONV_SZ_MASK;
    data |= ( PC302_GPIO_SD_CONFIG_AND |
            ( converter_size & PC302_GPIO_SD_CONV_SZ_MASK ) );

    ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_CONFIG( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to write config register for SDGPIO \
                pin %u\n", block_pin );
        return -EIO;
    }

    /* Configure the pin to drive the output. */
    ret = picoif_config_read( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read SDGPIO control value register\n" );
        return -EIO;
    }

    data |= ( 1 << block_pin );

    ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                               PC302_GPIO_SD_CONTROL_VAL_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to write control value register for SDGPIO \
                pin %u\n", block_pin );
        return -EIO;
    }

    /* Write the Analogue rate register */
    data = analogue_rate;
    ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                              PC302_GPIO_SD_PIN_ANALOGUE_RATE( block_pin ),
                              1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to write analogue rate register for SDGPIO \
                pin %u\n", block_pin );
        return -EIO;
    }

    /* Set the a_not_d flag in the pin allocation structure */
    pin->a_not_d = 1;

    return 0;
}

/**
 * Given a GPIO pin number, setup the pai gpio muxing.
 *
 * \param gpio The GPIO pin number.
 * \return Returns zero on success, non-zero on failure.
 */
static int
pc302gpio_pai_muxing( picoifGpioPinNum_t pin )
{
    int ret= -EINVAL;

    u16 data = PC302_PAI_WAKE_UP;

    /* We are about to setup a gpio mux in the PAI block,
     * therefore before we do anything else we need to wake up
     * the PAI block. */
    ret = picoif_config_write( 0, PC302_PAI_CAEID,
                               PC302_PAI_SLEEP_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to wake up the PAI block.\n");
        return ret;
    }

    /* Read back the current value of the pai_io_ctrl register */
    ret = picoif_config_read( 0, PC302_PAI_CAEID,
                              PC302_PAI_IO_CTRL_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to read from the PAI io_ctrl register.\n");
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
        ret = picoif_config_write( 0, PC302_PAI_CAEID,
                                   PC302_PAI_IO_CTRL_REG, 1, &data );
        if ( 1 != ret )
        {
            printk( KERN_ALERT "failed to write to the PAI "
                                "io_ctrl register.\n");
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
        ret = picoif_config_write( 0, PC302_PAI_CAEID,
                                   PC302_PAI_IO_CTRL_REG, 1, &data );
        if ( 1 != ret )
        {
            printk( KERN_ALERT "failed to write to the PAI "
                               "io_ctrl register.\n");
            return ret;
        }
    }
    else
    {
        /* We have been called with an out of range pin specified */
        printk( KERN_DEBUG "pc302gpio: out of range gpio for "
                            "pai muxing.\n");
        return  -EINVAL;
    }

    return 0;
}

#ifdef CONFIG_CONFIGFS_FS
/**
 * Show the pin number for the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to write the value in.
 *
 * \return Returns the number of bytes written.
 */
static ssize_t
pc302gpio_show_pin( struct pc302gpio_cfs_pin *pin,
                    char *buf )
{
    int ret;
    spin_lock( &pc302gpio_priv.lock );
    ret = snprintf( buf, PAGE_SIZE, "%d\n", pin->phys_pin->pin_num );
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}

/**
 * Store the pin number for the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to read the value from.
 * \param count The number of bytes available to read.
 *
 * \return Returns the length read on success, negative on failure.
 */
static ssize_t
pc302gpio_store_pin( struct pc302gpio_cfs_pin *pin,
                     const char *buf,
                     size_t count )
{
    unsigned pin_num;
    int pad_num;
    int ret;

    spin_lock( &pc302gpio_priv.lock );

    ret = -EINVAL;
    if ( pin->phys_pin->enabled )
        goto out;

    pin_num = simple_strtoul( buf, NULL, 10 );
    pad_num = pc302gpio_get_pad( pin_num );

    ret = -EINVAL;
    if ( pad_num < 0 || pc302gpio_pads[ pad_num ] )
        goto out;

    ret = -EIO;
    if ( set_pin_mux( pin_num ) )
        goto out;

    pin->phys_pin->pin_num = pin_num;
    pc302gpio_pads[ pad_num ] = pin->phys_pin;
    pin->phys_pin->pad = pad_num;

    ret = strlen( buf );
out:
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}

/**
 * Show the enabled state for the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to write the value in.
 *
 * \return Returns the number of bytes written.
 */
static ssize_t
pc302gpio_show_enabled( struct pc302gpio_cfs_pin *pin,
                        char *buf )
{
    int ret;
    spin_lock( &pc302gpio_priv.lock );
    ret = snprintf( buf, PAGE_SIZE, "%d\n", pin->phys_pin->enabled );
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}

/**
 * Store the enabled state for the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to read the value from.
 * \param count The number of bytes available to read.
 *
 * \return Returns the length read on success, negative on failure.
 */
static ssize_t
pc302gpio_store_enabled( struct pc302gpio_cfs_pin *pin,
                         const char *buf,
                         size_t count )
{
    int ret;

    spin_lock( &pc302gpio_priv.lock );
    ret = pc302gpio_set_direction( pin->phys_pin, pin->phys_pin->is_input );

    if ( !ret )
    {
        pin->phys_pin->enabled = !!simple_strtoul( buf, NULL, 10 );
        ret = strlen( buf );
    }
    else
        ret = -EINVAL;

    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}

/**
 * Show the direction for the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to write the value in.
 *
 * \return Returns the number of bytes written.
 */
static ssize_t
pc302gpio_show_is_input( struct pc302gpio_cfs_pin *pin,
                          char *buf )
{
    int ret;

    spin_lock( &pc302gpio_priv.lock );
    ret = snprintf( buf, PAGE_SIZE, "%d\n", pin->phys_pin->is_input );
    spin_unlock( &pc302gpio_priv.lock );

    return ret;
}

/**
 * Store the direction for the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to read the value from.
 * \param count The number of bytes available to read.
 *
 * \return Returns the length read on success, negative on failure.
 */
static ssize_t
pc302gpio_store_is_input( struct pc302gpio_cfs_pin *pin,
                           const char *buf,
                           size_t count )
{
    int ret;

    spin_lock( &pc302gpio_priv.lock );

    ret = -EINVAL;
    if ( pin->phys_pin->enabled )
        goto out;

    pin->phys_pin->is_input = simple_strtoul( buf, NULL, 10 );
    pin->phys_pin->direction_unset = 0;
    pc302gpio_set_direction( pin->phys_pin, pin->phys_pin->is_input );

    ret = strlen( buf );
out:
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}

/**
 * Show the name of the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to write the value in.
 *
 * \return Returns the number of bytes written.
 */
static ssize_t
pc302gpio_show_name( struct pc302gpio_cfs_pin *pin,
                     char *buf )
{
    int ret;

    spin_lock( &pc302gpio_priv.lock );
    ret = snprintf( buf, PAGE_SIZE, "%s\n", pin->phys_pin->name );
    spin_unlock( &pc302gpio_priv.lock );

    return ret;
}

/**
 * Show the pad number of the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to write the value in.
 *
 * \return Returns the number of bytes written.
 */
static ssize_t
pc302gpio_show_pad( struct pc302gpio_cfs_pin *pin,
                    char *buf )
{
    int ret;

    spin_lock( &pc302gpio_priv.lock );
    ret = snprintf( buf, PAGE_SIZE, "%d\n", pin->phys_pin->pad );
    spin_unlock( &pc302gpio_priv.lock );

    return ret;
}

/**
 * Show the value of the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to write the value in.
 *
 * \return Returns the number of bytes written.
 */
static ssize_t
pc302gpio_show_value( struct pc302gpio_cfs_pin *pin,
                      char *buf )
{
    int ret;

    spin_lock( &pc302gpio_priv.lock );
    if ( pin->phys_pin->is_input )
    {
        ret = pc302gpio_get_value( pin->phys_pin );
        if ( ret < 0 )
            goto out;
    }
    else
        ret = pin->phys_pin->value;

    ret = snprintf( buf, PAGE_SIZE, "%d\n", ret );
out:
    spin_unlock( &pc302gpio_priv.lock );
    return ret;
}

/**
 * Store the value for the given GPIO pin.
 *
 * \param pin The configfs representation of the pin being queried.
 * \param buf The buffer to read the value from.
 * \param count The number of bytes available to read.
 *
 * \return Returns the length read on success, negative on failure.
 */
static ssize_t
pc302gpio_store_value( struct pc302gpio_cfs_pin *pin,
                       const char *buf,
                       size_t count )
{
    int ret;
    spin_lock( &pc302gpio_priv.lock );

    if ( 1 != pin->phys_pin->enabled || pin->phys_pin->is_input )
    {
        spin_unlock( &pc302gpio_priv.lock );
        return -EINVAL;
    }

    pin->phys_pin->value = !!simple_strtoul( buf, NULL, 10 );
    ret = pc302gpio_set_value( pin->phys_pin, pin->phys_pin->value );

    spin_unlock( &pc302gpio_priv.lock );

    if ( ret < 0 )
        return -EIO;

    return strlen( buf );
}

/**
 * Create a read-only attribute in configfs.
 *
 * \param _name The name of the attribute.
 */
#define PC302GPIO_PIN_ATTR_RO( _name ) \
    static struct pc302gpio_pin_attr pc302gpio_pin_##_name = \
        __CONFIGFS_ATTR( _name, S_IRUGO, pc302gpio_show_##_name, NULL )

/**
 * Create a read/write attribute in configfs.
 *
 * \param _name The name of the attribute.
 */
#define PC302GPIO_PIN_ATTR_RW( _name ) \
    static struct pc302gpio_pin_attr pc302gpio_pin_##_name = \
        __CONFIGFS_ATTR( _name, S_IRUGO | S_IWUSR, pc302gpio_show_##_name, \
                        pc302gpio_store_##_name )

/** Pin number attribute. */
PC302GPIO_PIN_ATTR_RW( pin );
/** Direction attribute (1 == input, 0 == output). */
PC302GPIO_PIN_ATTR_RW( is_input );
/** The value of the pin (RO if input, RW if output). */
PC302GPIO_PIN_ATTR_RW( value );
/** Enabled attribute. */
PC302GPIO_PIN_ATTR_RW( enabled );
/** Name attribute */
PC302GPIO_PIN_ATTR_RO( name );
/** Pad number attribute */
PC302GPIO_PIN_ATTR_RO( pad );

/** Attributes to be shown for each configfs pin. */
static struct configfs_attribute *pc302gpio_pin_attrs[] = {
    &pc302gpio_pin_pin.attr,
    &pc302gpio_pin_is_input.attr,
    &pc302gpio_pin_value.attr,
    &pc302gpio_pin_enabled.attr,
    &pc302gpio_pin_name.attr,
    &pc302gpio_pin_pad.attr,
    NULL,
};

/**
 * From a configfs item, find the pin that it represents.
 *
 * \param item The configfs item.
 * \return Returns a pointer to the pin on success, NULL on failure.
 */
static struct pc302gpio_cfs_pin *
to_pin( struct config_item *item )
{
    return item ? container_of( item, struct pc302gpio_cfs_pin, item ) : NULL;
}

/**
 * Show an attribute of a configfs pin.
 *
 * \param item The item that we want to see the attribute of.
 * \param attr The attribute to show.
 * \param buf The buffer to store the value in.
 * \return Returns the number of bytes written to the buffer.
 */
static ssize_t
pc302gpio_pin_attr_show( struct config_item *item,
                            struct configfs_attribute *attr,
                            char *buf )
{
    ssize_t ret = -EINVAL;
    struct pc302gpio_cfs_pin *pin = to_pin( item );
    struct pc302gpio_pin_attr *attribute =
        container_of( attr, struct pc302gpio_pin_attr, attr );

    if ( attribute->show )
        ret = attribute->show( pin, buf );

    return ret;
}

/**
 * Store the value of an attribute in a configfs pin.
 *
 * \param item The item that we want to store the attribute value in.
 * \param attr The attribute that we want to set.
 * \param buf The value that we want to set.
 * \param count The number of bytes in the value buffer.
 * \return Returns the number of bytes read from the buffer.
 */
static ssize_t
pc302gpio_pin_attr_store( struct config_item *item,
                             struct configfs_attribute *attr,
                             const char *buf,
                             size_t count )
{
    ssize_t ret = -EINVAL;
    struct pc302gpio_cfs_pin *pin = to_pin( item );
    struct pc302gpio_pin_attr *attribute =
        container_of( attr, struct pc302gpio_pin_attr, attr );

    if ( attribute->show )
        ret = attribute->store( pin, buf, count );

    return ret;
}

/**
 * Release a configfs pin.
 *
 * \param item The item to release. This frees the GPIO pin up for reuse.
 */
static void
pc302gpio_pin_release( struct config_item *item )
{
    unsigned i;
    struct pc302gpio_cfs_pin *t = to_pin( item );

    for ( i = 0; i < PC302_GPIO_NUM_PADS; ++i )
        if ( pc302gpio_pads[ i ] == t->phys_pin )
        {
            pc302gpio_free_pin( pc302gpio_pads[ i ] );
            pc302gpio_pads[ i ] = NULL;
        }

    kfree( to_pin( item ) );
}

/** Operations that can happen on a configfs pin. */
static struct configfs_item_operations pc302gpio_pin_item_ops = {
    .release = pc302gpio_pin_release,
    .show_attribute = pc302gpio_pin_attr_show,
    .store_attribute = pc302gpio_pin_attr_store,
};

/** Description of a configfs GPIO pin and its associated attributes. */
static struct config_item_type pc302gpio_cfs_pin_type = {
    .ct_attrs = pc302gpio_pin_attrs,
    .ct_item_ops = &pc302gpio_pin_item_ops,
    .ct_owner = THIS_MODULE,
};


/**
 * Create a new configfs pin. This will be called when the user calls mkdir()
 * in /config/pc302gpio.
 *
 * \param group Not used.
 * \param name The name of the new entry.
 * \return Returns a pointer to the new entry on success, NULL on failure.
 */
static struct config_item *
make_pc302gpio_pin( struct config_group *group,
                    const char *name )
{
    struct pc302gpio_cfs_pin *t;
    char tmp_buf[ PC302_GPIO_PIN_NAME_MAX ];

    t = kzalloc( sizeof( *t ), GFP_KERNEL );
    if ( !t )
    {
        printk( KERN_ERR "pc302gpio: failed to allocate memory\n" );
        return NULL;
    }
    memset( t, sizeof( *t ), 0 );

    snprintf( tmp_buf, sizeof( tmp_buf ) - 1, "configfs/%s", name );
    t->phys_pin = pc302gpio_alloc_pin( tmp_buf );
    if ( !t->phys_pin )
    {
        printk( KERN_ERR "pc302gpio: failed to allocate memory\n" );
        kfree( t );
        return NULL;
    }

    config_item_init_type_name( &t->item, name, &pc302gpio_cfs_pin_type );

    return &t->item;
}

/**
 * Drop a configfs pin. If the reference count reaches zero, the pin will be
 * released.
 *
 * \param group Not used.
 * \param item The item being dropped.
 */
static void
drop_pc302gpio_pin( struct config_group *group,
                    struct config_item *item )
{
    struct pc302gpio_cfs_pin *t = to_pin( item );

    config_item_put( &t->item );
}

/** Operations table to describe how to create and drop pins through
 *  configfs. */
static struct configfs_group_operations pc302gpio_subsys_group_ops = {
    .make_item = make_pc302gpio_pin,
    .drop_item = drop_pc302gpio_pin,
};

/**
 * Description of the pc302gpio configfs subsystem type. */
static struct config_item_type pc302gpio_subsys_type = {
    .ct_group_ops = &pc302gpio_subsys_group_ops,
    .ct_owner = THIS_MODULE,
};

/** The configfs subsystem itself. */
static struct configfs_subsystem pc302gpio_subsys = {
    .su_group = {
        .cg_item = {
            .ci_namebuf = "pc302gpio",
            .ci_type = &pc302gpio_subsys_type,
        },
    },
};
#endif /* CONFIG_CONFIGFS_FS */

#ifdef CONFIG_DEBUG_FS
static void *
pc302gpio_seq_start( struct seq_file *f,
                     loff_t *pos )
{
    return ( *pos < PC302_GPIO_NUM_PADS ) ? pos : NULL;
}

static void *
pc302gpio_seq_next( struct seq_file *f,
                    void *v,
                    loff_t *pos )
{
    ( *pos )++;
    if ( *pos >= PC302_GPIO_NUM_PADS )
        return NULL;
    return pos;
}

static void
pc302gpio_seq_stop( struct seq_file *f,
                    void *v )
{
    /* This function is intentionally left blank. */
}

static int
pc302gpio_seq_show( struct seq_file *pi,
                    void *v )
{
    unsigned pad_num = *( unsigned * )v;

    if ( 0 == pad_num )
        seq_printf( pi, "%-3s  %-3s  %-16s  %-s\n", "pin", "pad", "name",
                    "is_input" );

    if ( pc302gpio_pads[ pad_num ] )
    {
        seq_printf( pi, "%-3d  %-3d  %-16s  %-d\n",
                    pc302gpio_pads[ pad_num]->pin_num, pad_num,
                    pc302gpio_pads[ pad_num ]->name,
                    pc302gpio_pads[ pad_num ]->is_input ? 1 : 0 );
    }

    return 0;
}

static struct seq_operations pc302gpio_seq_ops = {
    .start = pc302gpio_seq_start,
    .next  = pc302gpio_seq_next,
    .stop  = pc302gpio_seq_stop,
    .show  = pc302gpio_seq_show,
};

static int
pc302gpio_debugfs_open( struct inode *inode,
                       struct file *filp )
{
    return seq_open( filp, &pc302gpio_seq_ops );
}

static struct file_operations pc302gpio_debugfs_ops = {
    .open    = pc302gpio_debugfs_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = seq_release,
};

static int
pc302gpio_debugfs_init( void )
{
    pc302gpio_priv.debugfs_dir = debugfs_create_dir( "pc302gpio", NULL );
    if ( !pc302gpio_priv.debugfs_dir )
        return -ENOMEM;

    if ( !debugfs_create_file( "list", 0444, pc302gpio_priv.debugfs_dir, NULL,
                               &pc302gpio_debugfs_ops ) )
    {
        debugfs_remove( pc302gpio_priv.debugfs_dir );
        return -ENOMEM;
    }

    debugfs_create_u32( "boot_mode", 0555, pc302gpio_priv.debugfs_dir,
                        &boot_mode );

    return 0;
}

static void
pc302gpio_debugfs_exit( void )
{
    debugfs_remove_recursive( pc302gpio_priv.debugfs_dir );
}
#endif /* CONFIG_DEBUG_FS */

#ifndef MODULE
static void
pc302gpio_irq_enable(unsigned int irq)
{
    int gpio = (int)get_irq_data(irq);
    void *port_inten;
    u32 val;

    port_inten = (pc302gpio_priv.mem_region + GPIO_INT_EN_REG_OFFSET);

    val = ioread32(port_inten);
    val |= (1 << gpio);

    iowrite32(val, port_inten);

    val = ioread32(__io(IO_ADDRESS(PC302_VIC1_BASE
                                   + VIC_INT_ENABLE_REG_OFFSET)));
    val |= (1 << irq);
    iowrite32(val,
              __io(IO_ADDRESS(PC302_VIC1_BASE + VIC_INT_ENABLE_REG_OFFSET)));
}

static void
pc302gpio_irq_disable(unsigned int irq)
{
    int gpio = (int)get_irq_data(irq);
    void *port_inten;
    u32 val;

    port_inten = (pc302gpio_priv.mem_region + GPIO_INT_EN_REG_OFFSET);

    val = ioread32(port_inten);
    val &= ~(1 << gpio);

    iowrite32(val, port_inten);

    val = ioread32(__io(IO_ADDRESS(PC302_VIC1_BASE
                                   + VIC_INT_ENABLE_CLEAR_REG_OFFSET)));
    val |= (1 << irq);
    iowrite32(val,
              __io(IO_ADDRESS(PC302_VIC1_BASE
                              + VIC_INT_ENABLE_CLEAR_REG_OFFSET)));
}

static void
pc302gpio_irq_mask(unsigned int irq)
{
    int gpio = (int)get_irq_data(irq);
    void *port_intmask;
    u32 val;

    port_intmask = (pc302gpio_priv.mem_region + GPIO_INT_MASK_REG_OFFSET);

    val = ioread32(port_intmask);
    val |= (1 << gpio);

    iowrite32(val, port_intmask);

    val = ioread32(__io(IO_ADDRESS(PC302_VIC1_BASE
                                   + VIC_INT_ENABLE_CLEAR_REG_OFFSET)));
    val |= (1 << irq);
    iowrite32(val,
              __io(IO_ADDRESS(PC302_VIC1_BASE
                              + VIC_INT_ENABLE_CLEAR_REG_OFFSET)));
}

static void
pc302gpio_irq_ack(unsigned int irq)
{
    int gpio = (int)get_irq_data(irq);
    void *port_intmask, *port_eoi, *port_inttype_level;
    u32 val;

    port_intmask = (pc302gpio_priv.mem_region + GPIO_INT_MASK_REG_OFFSET);
    port_inttype_level = (pc302gpio_priv.mem_region
                          + GPIO_INT_TYPE_LEVEL_REG_OFFSET);
    port_eoi = (pc302gpio_priv.mem_region + GPIO_PORT_A_EOI_REG_OFFSET);

    val = ioread32(port_inttype_level);

    if (val & (1 << gpio))
    {
        /* Edge-sensitive */
        val = ioread32(port_eoi);
        val |= (1 << gpio);
        iowrite32(val, port_eoi);
    }
    else
    {
        /* Level-sensitive */
        val = ioread32(port_intmask);
        val |= (1 << gpio);
        iowrite32(val, port_intmask);
    }

}

static void
pc302gpio_irq_unmask(unsigned int irq)
{
    int gpio = (int)get_irq_data(irq);
    void *port_intmask;
    u32 val;

    port_intmask = (pc302gpio_priv.mem_region + GPIO_INT_MASK_REG_OFFSET);

    val = ioread32(port_intmask);
    val &= ~(1 << gpio);

    iowrite32(val, port_intmask);

    val = ioread32(__io(IO_ADDRESS(PC302_VIC1_BASE
                                   + VIC_INT_ENABLE_REG_OFFSET)));
    val |= (1 << irq);
    iowrite32(val,
              __io(IO_ADDRESS(PC302_VIC1_BASE + VIC_INT_ENABLE_REG_OFFSET)));
}

static int
pc302gpio_irq_set_type(unsigned int irq, unsigned int trigger)
{
    int gpio = (int)get_irq_data(irq);
    void *port_inttype_level;
    void *port_int_polarity;
    u32 level, polar;
    void *handler = handle_level_irq;

    if (trigger & ~(IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING
                    | IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW))
        return -EINVAL;

    port_inttype_level = (pc302gpio_priv.mem_region
                          + GPIO_INT_TYPE_LEVEL_REG_OFFSET);
    port_int_polarity = (pc302gpio_priv.mem_region
                         + GPIO_INT_POLARITY_REG_OFFSET);

    level = ioread32(port_inttype_level);
    polar = ioread32(port_int_polarity);

    if (trigger & IRQ_TYPE_EDGE_RISING)
    {
        level |= (1 << gpio);
        polar |= (1 << gpio);
    }
    else if (trigger & IRQ_TYPE_EDGE_FALLING)
    {
        level |= (1 << gpio);
        polar &= ~(1 << gpio);
    }
    else if (trigger & IRQ_TYPE_LEVEL_HIGH)
    {
        level &= ~(1 << gpio);
        polar |= (1 << gpio);
    }
    else if (trigger & IRQ_TYPE_LEVEL_LOW)
    {
        level &= ~(1 << gpio);
        polar &= ~(1 << gpio);
    }

    iowrite32(level, port_inttype_level);
    iowrite32(polar, port_int_polarity);

    if ((trigger & IRQ_TYPE_EDGE_RISING) || (trigger & IRQ_TYPE_EDGE_RISING))
        handler = handle_edge_irq;

    __set_irq_handler_unlocked(irq, handler);

    return 0;
}

static struct irq_chip pc302gpio_irqchip = {
    .name     = "pc302gpio",
    .ack      = pc302gpio_irq_ack,
    .mask     = pc302gpio_irq_mask,
    .unmask   = pc302gpio_irq_unmask,
    .enable   = pc302gpio_irq_enable,
    .disable  = pc302gpio_irq_disable,
    .set_type = pc302gpio_irq_set_type,
};

static int
pc302gpio_irq_setup(void)
{
    int irq;
    int gpio;

    for (irq = IRQ_GPIO0; irq <= IRQ_GPIO7; irq++)
    {
        gpio = (irq - IRQ_GPIO0) + PC302_GPIO_PIN_ARM_0;
        set_irq_chip(irq, &pc302gpio_irqchip);
        set_irq_data(irq, (void*)gpio);
        set_irq_handler(irq, handle_level_irq);
    }

    return 0;
}
#endif /* MODULE */

static int
pc302gpio_ioctl( struct inode *inode,
                 struct file *filp,
                 unsigned int cmd,
                 unsigned long arg )
{
    int ret;
    struct pc302gpio_pin_allocation *pin;
    picogpio_op_t op;
    picogpio_analogue_config_t dac_cfg;

    if ( _IOC_TYPE( cmd ) != PICOGPIO_IOCTL_BASE )
    {
        printk( KERN_DEBUG "pc302gpio: invalid command type\n" );
        return -ENOTTY;
    }

    if ( _IOC_NR( cmd ) >
            ( PICOGPIO_IOCTL_START + PICOGPIO_IOCTL_NUM_IOCTL ) ||
         _IOC_NR( cmd ) < ( PICOGPIO_IOCTL_START ) )
    {
        printk( KERN_DEBUG "pc302gpio: invalid command\n" );
        return -ENOTTY;
    }

    if ( cmd != PICOGPIO_ANALOGUE_CONFIG )
        ret = copy_from_user( &op, ( void __user * )arg, sizeof( op ) );
    else
        ret = copy_from_user( &dac_cfg, ( void __user * )arg,
                              sizeof( dac_cfg ) );

    if ( ret )
    {
        printk( KERN_DEBUG "pc302gpio: failed to copy structure\n" );
        return -EFAULT;
    }

    switch ( cmd )
    {
        case PICOGPIO_ACQUIRE:
            ret = gpio_request( op.pin, "ioctl" );
            break;

        case PICOGPIO_RELEASE:
            gpio_free( op.pin );
            ret = 0;
            break;

        case PICOGPIO_SET_DIRECTION:
            if ( PICOGPIO_INPUT == op.value )
                ret = gpio_direction_input( op.pin );
            else if ( PICOGPIO_OUTPUT == op.value )
                ret = gpio_direction_output( op.pin, -1 );
            else
                ret = -EINVAL;
            break;

        case PICOGPIO_GET_DIRECTION:
            pin = pc302gpio_find_pin( op.pin );
            if ( pin )
            {
                op.value = pin->is_input ? PICOGPIO_INPUT : PICOGPIO_OUTPUT;
                ret = copy_to_user( ( void __user * )arg, &op, sizeof( op ) );
            }
            break;

        case PICOGPIO_SET_VALUE:
            ret = gpio_set_value( op.pin, op.value );
            break;

        case PICOGPIO_GET_VALUE:
            ret = gpio_get_value( op.pin );
            if ( ret >= 0 )
            {
                op.value = ret;
                ret = copy_to_user( ( void __user * )arg, &op, sizeof( op ) );
            }
            break;

        case PICOGPIO_ANALOGUE_CONFIG:
            ret = pc302gpio_configure_dac( dac_cfg.pin,
                                           dac_cfg.converter_size,
                                           dac_cfg.analogue_rate );
            break;

        default:
            printk( KERN_DEBUG "pc302gpio: invalid ioctl(), cmd=%d\n", cmd );
            ret = -EINVAL;
            break;
    }

    return ret;
}

static int
pc302gpio_open( struct inode *inode,
                struct file *filp )
{
    return 0;
}

static int
pc302gpio_release( struct inode *inode,
                   struct file *filp )
{
    return 0;
}

static int pc302gpio_probe( struct platform_device *pdev );
static int pc302gpio_remove( struct platform_device *pdev );

static struct platform_driver pc302gpio_driver = {
    .probe = pc302gpio_probe,
    .remove = pc302gpio_remove,
    .driver = {
        .name = CARDNAME,
    },
};

static int
pc302gpio_probe( struct platform_device *pdev )
{
    int ret;
    struct resource *mem = platform_get_resource( pdev, IORESOURCE_MEM, 0 );

    if ( !mem )
        return -EINVAL;

    pc302gpio_priv.pdev = pdev;
    pc302gpio_priv.pdrv = &pc302gpio_driver;

    if ( !request_mem_region( mem->start, ( mem->end - mem->start ) + 1,
                              CARDNAME ) )
        return -EBUSY;

    pc302gpio_priv.mem_region =
        ioremap( mem->start, ( mem->end - mem->start ) + 1 );

    if ( !pc302gpio_priv.mem_region )
    {
        ret = -EBUSY;
        goto remap_failed;
    }

#ifndef MODULE
    pc302gpio_irq_setup();
#endif

    ret = misc_register( &pc302gpio_priv.dev );
    if (ret)
    {
        goto remap_failed;
    }

    return ret;

remap_failed:
    release_mem_region( mem->start, ( mem->end - mem->start ) + 1 );

    printk( KERN_ERR "PC302 GPIO driver registration failed\n" );

    return ret;
}

static int
pc302gpio_remove( struct platform_device *pdev )
{
    struct resource *mem = platform_get_resource( pdev, IORESOURCE_MEM, 0 );

    misc_deregister( &pc302gpio_priv.dev );
    iounmap( pc302gpio_priv.mem_region );
    release_mem_region( mem->start, ( mem->end - mem->start ) + 1 );

    return 0;
}

/* A function that we pass to the kernel so that it can do a hard reset */
static void pc302gpio_board_reset(char mode, void *cookie)
{
    /* RESET the board!! */
    if (gpio_request( PC302_RESET_OUTPUT_PIN, "reset") < 0)
    {
        return;
    }
    
    if (gpio_set_value( PC302_RESET_OUTPUT_PIN, 0 ) < 0)
    {
        return;
    }
    
    if (gpio_direction_output( PC302_RESET_OUTPUT_PIN, 0 ) < 0)
    {
        return;
    }
    
}

static int
pc302gpio_init( void )
{
    int ret = -EINVAL;

    u16 data = PC302_AXI2PICO_WAKE_UP;

    /* Get the current boot mode. */
    u32 syscfg = syscfg_read();
    boot_mode = ( syscfg & AXI2CFG_SYS_CONFIG_BOOT_MODE_MASK ) >>
        AXI2CFG_SYS_CONFIG_BOOT_MODE_LO;
#ifdef CONFIG_CONFIGFS_FS
    config_group_init( &pc302gpio_subsys.su_group );
    mutex_init( &pc302gpio_subsys.su_mutex );
    configfs_register_subsystem( &pc302gpio_subsys );
#endif /* CONFIG_CONFIGFS_FS */

#ifdef CONFIG_DEBUG_FS
    pc302gpio_debugfs_init();
#endif /* CONFIG_DEBUG_FS */

    /* This driver can use the sdgpio, therefore before we do anything
     * else we need to wake up the AXI2PICO block. */
    ret = picoif_config_write( 0, PC302_AXI2PICO_CAEID,
                               PC302_AXI2PICO_SLEEP_REG, 1, &data );
    if ( 1 != ret )
    {
        printk( KERN_ALERT "failed to wake up the AXI2PICO block.\n");
    }

    register_reset_handler(pc302gpio_board_reset, NULL);

    ret = platform_driver_register( &pc302gpio_driver );
    if ( 0 != ret )
    {
        printk(KERN_ERR "%s " CONFIG_LOCALVERSION " " __DATE__ " "  __TIME__
               " failed to load\n", TITLE);
        return ret;
    }

    printk(KERN_INFO "%s " CONFIG_LOCALVERSION " " __DATE__ " "  __TIME__
           " loaded\n", TITLE);

    return ret;
}

static void
pc302gpio_exit( void )
{
    deregister_reset_handler(pc302gpio_board_reset, NULL);
    
#ifdef CONFIG_CONFIGFS_FS
    configfs_unregister_subsystem( &pc302gpio_subsys );
#endif /* CONFIG_CONFIGFS_FS */

#ifdef CONFIG_DEBUG_FS
    pc302gpio_debugfs_exit();
#endif /* CONFIG_DEBUGFS */

    platform_driver_unregister( &pc302gpio_driver );

    printk(KERN_INFO "%s " CONFIG_LOCALVERSION " " __DATE__ " "  __TIME__
           " unloaded\n", TITLE);
}

module_init( pc302gpio_init );
module_exit( pc302gpio_exit );
MODULE_AUTHOR( "Jamie Iles" );
MODULE_LICENSE( "GPL" );
MODULE_DESCRIPTION( "picoChip PC302 GPIO driver" );
