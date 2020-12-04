/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006-2009 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

#ifndef __PERFCOUNTERS_H__
#define __PERFCOUNTERS_H__

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>

struct perfcounter;

#define PERFCOUNTER_MAX_NAME        ( 32 )

/*
 * Operations for performance counters. At a minimum, counters must implement
 * enable, disable and read methods.
 */
struct perfcounter_ops
{
    /* Enable the counter. */
    int ( *enable )( struct perfcounter *counter );

    /* Disable the counter. */
    int ( *disable )( struct perfcounter *counter );

    /* Read the value of the counter. */
    int ( *read )( struct perfcounter *counter,
                   u64 *val );

    /* Reset the counter back to 0. */
    int ( *reset )( struct perfcounter *counter );

    /* Set the event that the counter should monitor. */
    int ( *set_event)( struct perfcounter *counter,
                       int event );

    /* Interrupt for counter overflow. This is responsible for maintaining a
     * larger count. */
    irqreturn_t ( *overflow )( struct perfcounter *counter );
};

/*
 * Performance counter structure.
 */
struct perfcounter
{
    /* The unique name of the performance counter. This name is used for the
     * sysfs entry and the device node. These will be:
     *   /sys/class/perfcounter/_name_/
     *   /dev/_name_
     */
    const char              name[ PERFCOUNTER_MAX_NAME ];

    /* The operations for the counter. */
    struct perfcounter_ops  *ops;

    /*
     * The interrupt that the counter generates at overflow. This may be
     * omitted if not required. If set, the ops must have an overflow method.
     */
    unsigned int            irq;

    /* The IRQ flags to pass to request_irq(). */
    unsigned long           irq_flags;

    struct module           *owner;

    /*
     * Fields that are populated by the performance counter subsystem. These
     * fields should not be manipulated by users.
     */

    /* Boolean flag to indicate the enabled/disabled status of the counter. */
    int                     enabled;

    /* The base count of the counter. This should be incremented by 2^32 when
     * the counter overflows and added to the counter value when read. */
    u64                     base_count;

    /* The event that the counter is monitoring. This is ignored if the
     * counter is not configurable. */
    int                     event;

    /* The character device for the counter. */
    struct cdev             cdev;

    /* The actual sysfs device used for manipulating the counter. */
    struct device           *dev;
};

/* Register a new performance counter. This will register the counter with the
 * performance counter subsystem and leave it disabled at start. */
int
perfcounter_register( struct perfcounter *counter );

/* Remove a counter from the performance counter subsystem. */
int
perfcounter_unregister( struct perfcounter *counter );

#endif /* __PERFCOUNTERS_H__ */
