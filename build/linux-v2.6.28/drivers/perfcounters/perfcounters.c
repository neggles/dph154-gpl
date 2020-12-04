/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/perfcounters.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

#include <mach/irqs.h>

#define MAX_PERFCOUNTERS            ( 8 )

/*
 * Private data structure for the driver. This contains all of the registered
 * performance counters and any other associated data.
 */
static struct
{
    /* The first device in the chardev region. */
    dev_t                   first_dev;

    /* The counters that have been registered. */
    struct perfcounter      *counters[ MAX_PERFCOUNTERS ];

    /* The sysfs class for the performance counter sybsystem. */
    struct class            *sysfs_class;

    /* Serialisation lock. */
    spinlock_t              lock;

} perfcounter_internal;

/* Get a performance counter from the device minor number. Used to get the
 * counter from a device node so the application can read the count. */
static struct perfcounter *
minor_to_perfcounter( int minor )
{
    struct perfcounter *counter = NULL;
    int i;

    spin_lock( &perfcounter_internal.lock );

    for ( i = 0; i < MAX_PERFCOUNTERS; ++i )
        if ( perfcounter_internal.counters[ i ] &&
             MINOR( perfcounter_internal.counters[ i ]->cdev.dev ) == minor )
        {
            counter = perfcounter_internal.counters[ i ];
            break;
        }

    spin_unlock( &perfcounter_internal.lock );

    return counter;
}

/* Open the counter device node. Store a pointer to the counter in the
 * private_data field for fast access when reading. */
static int
perfcounter_open( struct inode *inode,
                  struct file *filp )
{
    struct perfcounter *counter = minor_to_perfcounter( iminor( inode ) );
    if ( !counter )
        return -ENODEV;

    filp->private_data = counter;

    if ( !counter->enabled )
        return counter->ops->enable( counter );

    return 0;
}

/* Release method for the counter. We could disable the counter here, but for
 * counters that share an enable method this could be bad! */
static int
perfcounter_release( struct inode *inode,
                     struct file *filp )
{
    return 0;
}

/* Read the value of a counter. */
static ssize_t
perfcounter_read( struct file *filp,
                  char __user *buf,
                  size_t len,
                  loff_t *offp )
{
    struct perfcounter *counter = filp->private_data;
    u64 val;
    int ret;

    if ( unlikely( !counter->ops->read ) )
        return -EOPNOTSUPP;

    if ( unlikely( len < sizeof( u64 ) ) )
        return -EMSGSIZE;

    ret = counter->ops->read( counter, &val );
    if ( !ret )
    {
        ret = copy_to_user( buf, &val, sizeof( val ) );
        if ( ret )
            ret = -EFAULT;
    }

    return ret ?: sizeof( val );
}

/* Write method for performance counters. This is used to clear a counter. */
static ssize_t
perfcounter_write( struct file *filp,
                   const char __user *buf,
                   size_t len,
                   loff_t *offp )
{
    struct perfcounter *counter = filp->private_data;
    int ret;

    /* Write is nothing intelligent. We just ignore the value the user gave us
     * and reset the counter. */
    if ( unlikely( !counter->ops->reset ) )
        return -EOPNOTSUPP;

    ret = counter->ops->reset( counter );

    return ret ?: len;
}

/* File operations for performance counters. */
static struct file_operations perfcounter_fops = {
    .owner      = THIS_MODULE,
    .open       = perfcounter_open,
    .release    = perfcounter_release,
    .read       = perfcounter_read,
    .write      = perfcounter_write,
};

/* Read the event that the counter is counting. */
static ssize_t
perfcounter_event_show( struct device *dev,
                        struct device_attribute *attr,
                        char *buf )
{
    struct perfcounter *counter = dev_get_drvdata( dev );
    return sprintf( buf, "0x%04x\n", counter->event );
}

/* Set the counter to count a new event type. */
static ssize_t
perfcounter_event_store( struct device *dev,
                         struct device_attribute *attr,
                         const char *buf,
                         size_t len )
{
    int event = simple_strtoul( buf, NULL, 0 );
    struct perfcounter *counter = dev_get_drvdata( dev );

    if ( unlikely( !counter->ops->set_event ) )
        return -EOPNOTSUPP;

    return counter->ops->set_event( counter, event ) ?: len;
}
DEVICE_ATTR( event, 0644, perfcounter_event_show,
             perfcounter_event_store );

/* Show the enabled status of a counter. */
static ssize_t
perfcounter_enabled_show( struct device *dev,
                          struct device_attribute *attr,
                          char *buf )
{
    struct perfcounter *counter = dev_get_drvdata( dev );
    return sprintf( buf, "%d\n", !!counter->enabled );
}

/* Enable or disable a counter. */
static ssize_t
perfcounter_enabled_store( struct device *dev,
                           struct device_attribute *attr,
                           const char *buf,
                           size_t len )
{
    int en = !!simple_strtoul( buf, NULL, 0 );
    struct perfcounter *counter = dev_get_drvdata( dev );
    int ret;

    if ( unlikely( !counter->ops->enable ) )
        return -EOPNOTSUPP;

    ret = en ? counter->ops->enable( counter ) :
               counter->ops->disable( counter );

    return ret ?: len;
}
DEVICE_ATTR( enabled, 0644, perfcounter_enabled_show,
             perfcounter_enabled_store );

/* Show the current value of a counter. */
static ssize_t
perfcounter_value_show( struct device *dev,
                        struct device_attribute *attr,
                        char *buf )
{
    u64 val;
    struct perfcounter *counter = dev_get_drvdata( dev );
    int ret;

    ret = -EOPNOTSUPP;
    if ( unlikely( !counter->ops->read ) )
        goto out;

    ret = counter->ops->read( counter, &val );

    if ( ret )
        goto out;

    ret = sprintf( buf, "%llu\n", val );
out:
    return ret;
}

/* Reset the counter back to 0. */
static ssize_t
perfcounter_value_store( struct device *dev,
                         struct device_attribute *attr,
                         const char *buf,
                         size_t len )
{
    struct perfcounter *counter = dev_get_drvdata( dev );
    int ret;

    ret = -EOPNOTSUPP;
    if ( unlikely( !counter->ops->reset ) )
        goto out;

    ret = counter->ops->reset( counter );

out:
    return ret ?: len;
}
DEVICE_ATTR( value, 0644, perfcounter_value_show, perfcounter_value_store );

/* Create the device attributes in sysfs for the counter. */
static int
perfcounter_create_attrs( struct perfcounter *counter )
{
    int ret = 0;

    ret = device_create_file( counter->dev, &dev_attr_value );
    if ( ret )
        goto out;

    ret = device_create_file( counter->dev, &dev_attr_enabled );
    if ( ret )
        goto enabled_fail;

    if ( counter->ops->set_event )
    {
        ret = device_create_file( counter->dev, &dev_attr_event );
        if ( ret )
            goto set_event_failed;
    }

    goto out;

set_event_failed:
    device_remove_file( counter->dev, &dev_attr_value );
enabled_fail:
    device_remove_file( counter->dev, &dev_attr_enabled );
out:
    return ret;
}

/* Remove the device attributes for a given counter. */
static void
perfcounter_remove_attrs( struct perfcounter *counter )
{
    if ( counter->ops->set_event )
        device_remove_file( counter->dev, &dev_attr_event );
    device_remove_file( counter->dev, &dev_attr_value );
    device_remove_file( counter->dev, &dev_attr_enabled );
}

/* Performance counter interrupt. Call the overflow method for the counter so
 * that it can maintain a 64 bit count from a 32 bit counter. */
static irqreturn_t
perfcounter_irq( int irq,
                 void *dev )
{
    struct perfcounter *counter = dev_get_drvdata( dev );

    return counter->ops->overflow( counter );
}

/* Register a new performance counter. This will register the counter with the
 * performance counter subsystem and leave it disabled at start. */
int
perfcounter_register( struct perfcounter *counter )
{
    int indx = -1;
    unsigned i;
    dev_t devno;
    int ret;

    spin_lock( &perfcounter_internal.lock );
    for ( i = 0; i < MAX_PERFCOUNTERS; ++i )
        if ( !perfcounter_internal.counters[ i ] )
        {
            indx = i;
            break;
        }

    ret = -ENOMEM;
    if ( indx < 0 )
        goto insert_failed;

    spin_unlock( &perfcounter_internal.lock );

    cdev_init( &counter->cdev, &perfcounter_fops );
    counter->cdev.owner = THIS_MODULE;
    counter->cdev.ops   = &perfcounter_fops;

    devno = MKDEV( MAJOR( perfcounter_internal.first_dev ),
                   MINOR( perfcounter_internal.first_dev ) + indx );

    ret = cdev_add( &counter->cdev, devno, 1 );
    if ( ret )
        goto out;

    counter->dev = device_create( perfcounter_internal.sysfs_class, NULL,
                                  counter->cdev.dev, NULL, counter->name );
    if ( IS_ERR( counter->dev ) )
    {
        ret = PTR_ERR( counter->dev );
        goto dev_fail;
    }
    dev_set_drvdata( counter->dev, counter );

    ret = perfcounter_create_attrs( counter );
    if ( ret )
        goto attr_fail;

    if ( counter->ops->overflow )
    {
        ret = request_irq( counter->irq, perfcounter_irq, counter->irq_flags,
                           counter->name, counter->dev );
        if ( ret )
            goto irq_failed;
    }

    /* Make sure the counter is disabled at start. */
    ret = counter->ops->disable( counter );
    if ( ret )
        goto disable_fail;

    spin_lock( &perfcounter_internal.lock );
    perfcounter_internal.counters[ indx ] = counter;
    spin_unlock( &perfcounter_internal.lock );

    goto out;

insert_failed:
    free_irq( counter->irq, counter->dev );
disable_fail:
    perfcounter_remove_attrs( counter );
irq_failed:
    if ( counter->ops->overflow )
        free_irq( counter->irq, counter->dev );
attr_fail:
    device_destroy( perfcounter_internal.sysfs_class, counter->cdev.dev );
dev_fail:
    cdev_del( &counter->cdev );
out:

    return ret;
}
EXPORT_SYMBOL( perfcounter_register );

/* Remove a counter from the performance counter subsystem. */
int
perfcounter_unregister( struct perfcounter *counter )
{
    unsigned i;
    int ret = -EINVAL;

    spin_lock( &perfcounter_internal.lock );

    if ( counter->ops->overflow )
        free_irq( counter->irq, counter->dev );

    for ( i = 0; i < MAX_PERFCOUNTERS; ++i )
        if ( perfcounter_internal.counters[ i ] == counter )
        {
            perfcounter_internal.counters[ i ] = NULL;
            ret = 0;
            break;
        }

    spin_unlock( &perfcounter_internal.lock );

    cdev_del( &counter->cdev );
    perfcounter_remove_attrs( counter );
    device_destroy( perfcounter_internal.sysfs_class, counter->cdev.dev );

    module_put( counter->owner );

    return ret;
}
EXPORT_SYMBOL( perfcounter_unregister );

static int __init
perfcounter_init( void )
{
    int ret = alloc_chrdev_region( &perfcounter_internal.first_dev,
                                   0, MAX_PERFCOUNTERS, "perfcounter" );

    if ( ret )
        goto out;

    spin_lock_init( &perfcounter_internal.lock );

    perfcounter_internal.sysfs_class = class_create( THIS_MODULE,
                                                     "perfcounter" );
    if ( IS_ERR( perfcounter_internal.sysfs_class ) )
    {
        ret = PTR_ERR( perfcounter_internal.sysfs_class );
        goto class_fail;
    }

    goto out;

class_fail:
    unregister_chrdev_region( perfcounter_internal.first_dev,
                              MAX_PERFCOUNTERS );
out:
    return ret;
}

static void __exit
perfcounter_exit( void )
{
    unsigned i;

    for ( i = 0; i < MAX_PERFCOUNTERS; ++i )
    {
        if ( perfcounter_internal.counters[ i ] )
            perfcounter_unregister( perfcounter_internal.counters[ i ] );
    }

    unregister_chrdev_region( perfcounter_internal.first_dev,
                              MAX_PERFCOUNTERS );

    class_destroy( perfcounter_internal.sysfs_class );
}

module_init( perfcounter_init );
module_exit( perfcounter_exit );

MODULE_AUTHOR( "Jamie Iles" );
MODULE_LICENSE( "GPL" );
MODULE_DESCRIPTION( "Performance counter subsystem" );
