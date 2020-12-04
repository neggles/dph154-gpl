/**
 *
 * \file pc302_wdt.c
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
 * This file implements a driver for the picoChip PC302 watchdog device in the
 * ARM subsystem.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/interrupt.h>
#include <mach/reset.h>

/**
 * The name used for the platform device and driver to allow Linux to match
 * them up.
 */
#define CARDNAME    "pc302wdt"

/**
 * The maximum TOP value that can be set in the watchdog.
 */
#define PC302_WDOG_MAX_TOP   ( 15 )

/**
 * The number of times the counter is decremented every second.
 */
#define PC302_WDOG_TICKS_PER_SECOND  ( 200000000 )

/**
 * The number of possible timeout values.
 */
#define PC302_WDT_NUM_TIMEOUTS  ( 16 )

/**
 * Open the watchdog driver device file.
 */
static int pc302wdt_open( struct inode *inode,
                    struct file *filp );

/**
 * Write to the watchdog device. Note that the values that are written are
 * discarded, writing simply pokes the watchdog.
 */
static ssize_t pc302wdt_write( struct file *filp,
                         const char __user *buf,
                         size_t len,
                         loff_t *offset );

/**
 * ioctl() call for the watchdog device driver. Used to set the timeout, poke
 * the hardware and to get the current timeout value.
 */
static int pc302wdt_ioctl( struct inode *inode,
                     struct file *filp,
                     unsigned int cmd,
                     unsigned long arg );

/**
 * Release method for the watchdog device driver. Removes all sysfs files and
 * deregisters the device.
 */
static int pc302wdt_release( struct inode *inodw,
                       struct file *filp );

/**
 * Set the timeout period for the watchdog.
 *
 * \param top_s The timeout period (in seconds).
 * \return Returns the value that was set on success, negative on failure.
 */
static int pc302wdt_set_top( unsigned top_s );

/**
 * Add the sysfs files. This should only be done once per module insertion.
 */
static void pc302wdt_sysfs_add( void );

/**
 * Poke the watchdog to prevent it from expiring.
 */
static void pc302wdt_keepalive( void );

/**
 * Remove the sysfs files.
 */
static void pc302wdt_sysfs_remove( void );

/**
 * Check if the watchdog is enabled. This uses the contents of the control
 * register in case the module is reloaded.
 *
 * \return Returns non-zero if enabled, zero if disabled.
 */
static int pc302wdt_is_enabled( void );

/**
 * Check if the pretimeout is enabled. This uses the contents of the control
 * register in case the module is reloaded.
 *
 * \return Returns non-zero if enabled, zero if disabled.
 */
static int pc302wdt_pretimeout_used( void );

/**
 * Set the pretimeout.
 *
 * \param timeout The timeout value to use (in seconds).
 * \return Returns the new pretimeout value.
 */
static int pc302wdt_set_pretimeout( int timeout );

/**
 * Get the time left until the WDT expires (in seconds).
 *
 * \return Returns the time left (in seconds).
 */
static u32 pc302wdt_time_left( void );

/**
 * ISR for the watchdog timer expiry. This routine will clear the interrupt
 * and the watchdog will begin decrementing again. When the pretimeout is
 * enabled, as long as this ISR returns, the system will not reset.
 */
static irqreturn_t pc302wdt_isr( int irq, void *dev );

/**
 * Probe function for the driver. When the module gets loaded, the kernel
 * should use the cardname to match this driver to the watchdog
 * platform_device.
 *
 * \param pdev The platform_device that this driver should drive.
 * \return Returns zero on success, non-zero on failure.
 */
static int pc302wdt_drv_probe( struct platform_device *pdev );

/**
 * Removal method for the driver. This will get called when the module is
 * unloaded and the driver is unregistered.
 *
 * \param pdev The platform_device that is being unloaded.
 * \return Returns zero on success, non-zero on failure.
 */
static int pc302wdt_drv_remove( struct platform_device *pdev );

/**
 * File operations for the watchdog device driver.
 */
static struct file_operations wdt_fops = {
    .open = pc302wdt_open,
    .write = pc302wdt_write,
    .ioctl = pc302wdt_ioctl,
    .release = pc302wdt_release
};

/**
 * Reset the platform by causing the watchdog to expire.
 *
 * \param mode Not used.
 * \param cookie Not used.
 */
static void pc302wdt_platform_reset( char mode,
                                     void *cookie );

static struct {

    /**
     * Internal data structures for the watchdog device driver.
     */
    struct miscdevice dev;

    /**
     * The platform device that the driver interfaces with. Used for the sysfs
     * entries.
     */
    struct platform_device *pdev;

    /**
     * The current timeout value (in seconds) of the watchdog.
     */
    unsigned int top;

    /**
     * Boolean flag for the enabled status of the watchdog.
     */
    int enabled;

    /**
     * Lock for the structure.
     */
    spinlock_t lock;

    /**
     * The memory region that maps the registers.
     */
    void __iomem *mem_region;

    /**
     * Boolean flag for using pretimeout. If enabled, the watchdog will
     * generate an interrupt and restart the counter before resetting.
     */
    int use_pretimeout;

    /**
     * Boolean flag for doing a system reset. If set, then kicking the
     * watchdog should have no effect.
     */
    int reset_pending;

} pc302wdt_int = {

    .dev = {
        .fops = &wdt_fops,
        .name = "watchdog",
        .minor = WATCHDOG_MINOR
    },

    .lock = __SPIN_LOCK_UNLOCKED( lock ),

    .top = 15,
    
    .mem_region = NULL,

    .use_pretimeout = 0,

    .reset_pending = 0,
};

/**
 * The PC302 watchdog supports 16 different timeout values, but 12 of these
 * values are all sub-1-second. As the watchdog API uses 1 second resolution,
 * these values aren't useful. These values are fixed in hardware and based on
 * the peripheral clock running at 200MHz.
 *
 * The timeout values in seconds. When selecting a timeout value, the driver
 * should find the timeout that is at least the requested value if possible.
 * If the requested value is larger than the maximum timeout, use the maximum.
 */
static const unsigned pc302wdt_timeouts[ 16 ] =
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 5, 10 };

/*
 * Show the current timeout value (in seconds) that the WDT is configured for.
 */
static ssize_t
pc302wdt_sysfs_timeout_show( struct device *dev,
                             struct device_attribute *attr,
                             char *buf )
{
    int ret;

    spin_lock( &pc302wdt_int.lock );
    ret = sprintf( buf, "%d\n", pc302wdt_int.enabled ?
                        pc302wdt_timeouts[ pc302wdt_int.top ] : -1 );
    spin_unlock( &pc302wdt_int.lock );

    return ret;
}

static DEVICE_ATTR( timeout, S_IRUGO, pc302wdt_sysfs_timeout_show, NULL );

/*
 * Show whether the WDT is currently enabled or not.
 */
static ssize_t
pc302wdt_sysfs_enabled_show( struct device *dev,
                             struct device_attribute *attr,
                             char *buf )
{
    int ret;

    spin_lock( &pc302wdt_int.lock );
    ret = sprintf( buf, "%c\n", pc302wdt_int.enabled ? '1' : '0' );
    spin_unlock( &pc302wdt_int.lock );

    return ret;
}

static DEVICE_ATTR( enabled, S_IRUGO, pc302wdt_sysfs_enabled_show, NULL );

/*
 * Show whether the pretimeout is currently enabled or not.
 */
static ssize_t
pc302wdt_sysfs_pretimeout_show( struct device *dev,
                                struct device_attribute *attr,
                                char *buf )
{
    int ret;

    spin_lock( &pc302wdt_int.lock );
    ret = sprintf( buf, "%d\n", pc302wdt_int.use_pretimeout ?
                        pc302wdt_timeouts[ pc302wdt_int.top ] : -1 );
    spin_unlock( &pc302wdt_int.lock );

    return ret;
}

static DEVICE_ATTR( pretimeout, S_IRUGO, pc302wdt_sysfs_pretimeout_show,
                    NULL );

/*
 * Show the current value of the counter in the WDT.
 */
static ssize_t
pc302wdt_sysfs_counter_show( struct device *dev,
                             struct device_attribute *attr,
                             char *buf )
{
    u32 ccv;

    spin_lock( &pc302wdt_int.lock );
    ccv = ioread32( pc302wdt_int.mem_region +
                    WDOG_CURRENT_COUNT_REG_OFFSET );
    spin_unlock( &pc302wdt_int.lock );

    return sprintf( buf, "%u\n", ccv );
}

static DEVICE_ATTR( counter, S_IRUGO, pc302wdt_sysfs_counter_show, NULL );

/*
 * Show the time left until the WDT fires.
 */
static ssize_t
pc302wdt_sysfs_time_left_show( struct device *dev,
                             struct device_attribute *attr,
                             char *buf )
{
    int ret;

    spin_lock( &pc302wdt_int.lock );
    ret = sprintf( buf, "%i\n",
                    pc302wdt_int.enabled ? pc302wdt_time_left() : -1 );
    spin_unlock( &pc302wdt_int.lock );

    return ret;
}

static DEVICE_ATTR( time_left, S_IRUGO, pc302wdt_sysfs_time_left_show, NULL );

/**
 * The group of attributes that should be added to the watchdog device. This
 * will most likely end up in /sys/devices/platform/pc302_wdt
 */
static struct attribute *pc302wdt_attrs[] = {
    &dev_attr_timeout.attr,
    &dev_attr_enabled.attr,
    &dev_attr_counter.attr,
    &dev_attr_pretimeout.attr,
    &dev_attr_time_left.attr,
    NULL
};

/**
 * Add the attributes as a group/
 */
static struct attribute_group pc302wdt_attr_group = {
    .attrs = pc302wdt_attrs
};

static void
pc302wdt_sysfs_remove( void )
{
    sysfs_remove_group( &pc302wdt_int.dev.this_device->kobj,
                        &pc302wdt_attr_group );
}

static void
pc302wdt_sysfs_add( void )
{
    int ret;

    ret = sysfs_create_group( &pc302wdt_int.pdev->dev.kobj,
                              &pc302wdt_attr_group );
    ret = sysfs_create_link( &pc302wdt_int.dev.this_device->kobj,
                             &pc302wdt_int.pdev->dev.kobj, "device" );
}

static int
pc302wdt_open( struct inode *inode,
                struct file *filp )
{
    u32 control_reg;

    if ( !pc302wdt_is_enabled() )
    {
        spin_lock( &pc302wdt_int.lock );

        /* The watchdog is not currently enabled. Set the timeout to the
         * maximum and then start it. */
        pc302wdt_set_top( PC302_WDOG_MAX_TOP );
        control_reg =
            ioread32( pc302wdt_int.mem_region + WDOG_CONTROL_REG_OFFSET );
        control_reg |= WDOGCONTROLREGWDT_ENMASK;
        iowrite32( control_reg,
                   pc302wdt_int.mem_region + WDOG_CONTROL_REG_OFFSET );
        pc302wdt_int.enabled = 1;

        spin_unlock( &pc302wdt_int.lock );
    }

    return 0;
}

ssize_t
pc302wdt_write( struct file *filp,
                const char __user *buf,
                size_t len,
                loff_t *offset )
{
    pc302wdt_keepalive();

    return len;
}

static u32
pc302wdt_time_left( void )
{
    u32 counter_val;
    spin_lock( &pc302wdt_int.lock );
    counter_val = ioread32( pc302wdt_int.mem_region +
                            WDOG_CURRENT_COUNT_REG_OFFSET );
    spin_unlock( &pc302wdt_int.lock );
    return counter_val / PC302_WDOG_TICKS_PER_SECOND;
}

static void
pc302wdt_keepalive( void )
{
    if ( !pc302wdt_int.reset_pending )
        iowrite32( WDOG_COUNTER_RESTART_KICK_VALUE,
                   pc302wdt_int.mem_region + WDOG_COUNTER_RESTART_REG_OFFSET );
}

static int
pc302wdt_ioctl( struct inode *inode,
                 struct file *filp,
                 unsigned int cmd,
                 unsigned long arg )
{
    int val;
    int ret;
    int err = 0;

    if ( _IOC_DIR( cmd ) & _IOC_READ )
        err = !access_ok( VERIFY_WRITE, ( void __user * )arg,
                          _IOC_SIZE( cmd ) );
    else if ( _IOC_DIR( cmd ) & _IOC_WRITE )
        err = !access_ok( VERIFY_READ, ( void __user * )arg,
                          _IOC_SIZE( cmd ) );

    if ( err )
        return -EFAULT;

    switch ( cmd )
    {
        /* Kick the watchdog to prevent it from expiring. */
        case WDIOC_KEEPALIVE:
            ret = 0;
            pc302wdt_keepalive();
            break;

        /* Set a new timeout value. */
        case WDIOC_SETTIMEOUT:
            ret = __get_user( val, ( int __user * )arg );
            if ( ret )
                break;
            val = pc302wdt_set_top( val );
            ret = __put_user( val, ( int __user * )arg );
            break;

        /* Get the time left until expiry. */
        case WDIOC_GETTIMELEFT:
            ret = __get_user( val, ( int __user * )arg );
            if ( ret )
                break;
            val = pc302wdt_time_left();
            ret = __put_user( val, ( int __user * )arg );
            break;

        /* Set the pretimeout. */
        case WDIOC_SETPRETIMEOUT:
            ret = __get_user( val, ( int __user * )arg );
            if ( ret )
                break;
            val = pc302wdt_set_pretimeout( val );
            ret = __put_user( val, ( int __user * )arg );
            break;

        default:
            printk( KERN_DEBUG "pc302wdt: invalid ioctl() cmd (%u)\n", cmd );
            ret = -EINVAL;
            break;
    }

    return ret;
}

static int
pc302wdt_set_top( unsigned top_s )
{
    unsigned i;
    int top_val = -1;
    unsigned long flags;

    /* Iterate over the timeout values until we find the closest match. We
     * always look for >=. */
    for ( i = 0; i < PC302_WDT_NUM_TIMEOUTS; ++i )
    {
        if ( pc302wdt_timeouts[ i ] >= top_s )
        {
            top_val = i;
            break;
        }
    }

    /* If we didn't find a suitable value, it must have been too large. Go
     * with the biggest that we can. */
    if ( top_val < 0 )
        top_val = PC302_WDT_NUM_TIMEOUTS - 1;

    /* Set the new value in the watchdog. */
    spin_lock_irqsave( &pc302wdt_int.lock, flags );
    iowrite32( top_val,
               pc302wdt_int.mem_region + WDOG_TIMEOUT_RANGE_REG_OFFSET );
    pc302wdt_int.top = top_val;
    spin_unlock_irqrestore( &pc302wdt_int.lock, flags );

    return pc302wdt_timeouts[ top_val ];
}

static int
pc302wdt_release( struct inode *inodw,
                  struct file *filp )
{
    return 0;
}

static int
pc302wdt_is_enabled( void )
{
    u32 control_reg;
    spin_lock( &pc302wdt_int.lock );
    control_reg =
        ioread32( pc302wdt_int.mem_region + WDOG_CONTROL_REG_OFFSET );
    spin_unlock( &pc302wdt_int.lock );

    return ( control_reg & WDOGCONTROLREGWDT_ENMASK );
}

static int
pc302wdt_set_pretimeout( int timeout )
{
    u32 control_reg;
    int ret;

    spin_lock( &pc302wdt_int.lock );

    control_reg =
        ioread32( pc302wdt_int.mem_region + WDOG_CONTROL_REG_OFFSET );

    /* We cannot choose an arbitrary pretimeout, so we interpret a timeout of
     * 0 as disable the pretimeout, and anything else as enable it. */
    if ( timeout )
        control_reg |= WDOGCONTROLREGRMODMASK;
    else
        control_reg &= ~( ( u32 )WDOGCONTROLREGRMODMASK );

    iowrite32( control_reg, pc302wdt_int.mem_region + WDOG_CONTROL_REG_OFFSET );

    /* The pretimeout will be the same as the timeout period so return this.
     */
    ret = ( control_reg & WDOGCONTROLREGRMODMASK ) ?
        pc302wdt_timeouts[ pc302wdt_int.top ] : 0;

    spin_unlock( &pc302wdt_int.lock );

    return ret;
}

static int
pc302wdt_pretimeout_used( void )
{
    u32 control_reg;
    spin_lock( &pc302wdt_int.lock );
    control_reg =
        ioread32( pc302wdt_int.mem_region + WDOG_CONTROL_REG_OFFSET );
    spin_unlock( &pc302wdt_int.lock );

    return ( control_reg & WDOGCONTROLREGRMODMASK );
}

static irqreturn_t
pc302wdt_isr( int irq,
              void *dev )
{
    /* Clear the interrupt in the device. Reading the register will clear the
     * interrupt but the value is of no interest so we can discard it. */
    ioread32( pc302wdt_int.mem_region + WDOG_CLEAR_REG_OFFSET );

    return IRQ_HANDLED;
}

static void
pc302wdt_platform_reset( char mode,
                         void *cookie )
{
    u32 control_reg =
            ioread32( pc302wdt_int.mem_region + WDOG_CONTROL_REG_OFFSET );
    control_reg |= WDOGCONTROLREGWDT_ENMASK;
    iowrite32( control_reg,
               pc302wdt_int.mem_region + WDOG_CONTROL_REG_OFFSET );
    wmb();

    /* Disable the pretimeout. */
    pc302wdt_set_pretimeout( 0 );

    /* Set the timeout to the minimum possible. */
    pc302wdt_set_top( 0 );

    pc302wdt_int.reset_pending = 1;
}

static int
pc302wdt_drv_probe( struct platform_device *pdev )
{
    int ret;
    struct resource *mem = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    struct resource *irq = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );

    if ( !mem || !irq )
        return -EINVAL;

    /* Map the memory. */
    if ( !request_mem_region( mem->start, ( mem->end - mem->start ) + 1,
                              CARDNAME ) )
        return -EBUSY;

    pc302wdt_int.mem_region =
        ioremap( mem->start, ( mem->end - mem->start ) + 1 );
    if ( !pc302wdt_int.mem_region )
    {
        ret = -EBUSY;
        goto remap_failed;
    }

    ret = request_irq( irq->start, pc302wdt_isr, IRQF_DISABLED, CARDNAME,
                       &pc302wdt_int.dev );
    if ( ret )
        goto irq_request_failed;

    pc302wdt_int.enabled = pc302wdt_is_enabled() ? 1 : 0;
    pc302wdt_int.use_pretimeout = pc302wdt_pretimeout_used() ? 1 : 0;
    pc302wdt_int.pdev = pdev;

    /* Register the device and configure the sysfs attributes. */
    ret = misc_register( &pc302wdt_int.dev );
    if ( ret )
        goto register_failed;
    pc302wdt_sysfs_add();

    register_reset_handler( pc302wdt_platform_reset, NULL );

    /* Disable the pretimeout. If we panic then we don't want a pretimeout
     * otherwise we'll never reset. */
    pc302wdt_set_pretimeout( 0 );

    return 0;

register_failed:
    free_irq( irq->start, &pc302wdt_int.dev );

irq_request_failed:
    iounmap( pc302wdt_int.mem_region );

remap_failed:
    release_mem_region( mem->start, ( mem->end - mem->start ) + 1 );
    pc302wdt_int.mem_region = NULL;

    printk( KERN_ERR "PC302 Watchdog registration failed\n" );

    return ret;
}

static int
pc302wdt_drv_remove( struct platform_device *pdev )
{
    struct resource *mem = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    struct resource *irq = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );

    if ( pc302wdt_int.mem_region )
    {
        iounmap( pc302wdt_int.mem_region );
        release_mem_region( mem->start, ( mem->end - mem->start ) + 1 );
    }

    free_irq( irq->start, &pc302wdt_int.dev );

    misc_deregister( &pc302wdt_int.dev );
    pc302wdt_sysfs_remove();

    deregister_reset_handler( pc302wdt_platform_reset, NULL );

    return 0;
}

static struct platform_driver pc302wdt_driver = {
    .probe = pc302wdt_drv_probe,
    .remove = pc302wdt_drv_remove,
    .driver = {
        .name = CARDNAME,
    }
};

static int
pc302wdt_watchdog_init( void )
{
    return platform_driver_register( &pc302wdt_driver );
}

static void
pc302wdt_watchdog_exit( void )
{
    platform_driver_unregister( &pc302wdt_driver );
}

module_init( pc302wdt_watchdog_init );
module_exit( pc302wdt_watchdog_exit );

MODULE_AUTHOR( "Jamie Iles" );
MODULE_DESCRIPTION( "picoChip PC302 Watchdog Driver" );
MODULE_LICENSE( "GPL");

