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

/*!
 * \file gpr_interrupt.c
 * \brief GPR interrupt transport module implementation.
 *
 * This file implements a GPR based transport mechanism that signals to the
 * application an interrupt has been generated via a GPR and optionally
 * records the values of the register at interrupt time.
 *
 * The two transport methods provided are:
 *  \li gpr_interrupt(with_values)
 *  \li gpr_interrupt(without_values)
 * The "with_values" variant samples the GPR at interrupt time and maintains a
 * queue of all of the values at interrupt time. When the transport is read,
 * an array of the values is returned to the user where each value is 32 bits
 * and the values are consumed. The "without_values" variant does not record
 * the value of the GPR at interrupt time (although it does read it to clear
 * the interrupt). Instead, this method maintains a count of the interrupts
 * raised and when read, returns the number of interrupts raised encoded in
 * the first 4 bytes of the buffer and the count is set to 0.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/picochip/transports/gpr_interrupt.h>
#include <linux/picochip/picoif_ioctl.h>

#include "debug.h"
#include "picoarray.h"
#include "picoif_internal.h"
#include "picoif_module.h"
#include "gpr_interrupt_internal.h"

/*!
 * \brief Stores the value of a GPR at interrupt time. This is dynamically
 * allocated so that we don't need static FIFOs.
 */
struct gpr_int_value
{
    u32                 value;  /*!< The value of the GPR when the interrupt
                                 *  was raised. */
    struct list_head    list;   /*!< The list storing the values. */
};

/*!
 * \brief GPR interrupt channel. Readable transport that returns the values of
 * the GPR at each interrupt.
 */
struct gpr_int_channel
{
    int                     record_values;  /*!< Boolean flag for whether GPR
                                             *   values should be recorded or
                                             *   just an interrupt count. */
    u32                     irq_count;      /*!< Number of interrupts raised. */
    struct pico_resource    *irq;           /*!< The IRQ used for the
                                             *   transport. */
    struct pico_resource    *gpr;           /*!< The GPR used for the
                                             *   transport. */
    struct picoarray        *pa;            /*!< The picoArray running the
                                             *   transport. */
    struct list_head        values;         /*!< The list storing the values. */
    spinlock_t              lock;           /*!< IRQ lock. */
    struct picoif_context   *reader;        /*!< The reader of the channel. */
};

/*!
 * \brief The gpr interrupt module.
 * \extends picoif_module
 */
struct gpr_int_module
{
    struct picoif_module   mod;            /*!< The generic module. */
    struct kmem_cache      *value_cache;   /*!< The kmem_cache for GPR values
                                            *   when IRQs are raised. */
};

static void
gpr_int_destructor( struct picoif_module *module );

static struct picoif_context *
gpr_int_create_trans_instance( struct picoif_module *module,
                               const char *description,
                               struct picoif_buf *params );

static void
gpr_int_close_trans_instance( struct picoif_module *module,
                              struct picoif_context *ctx );

static ssize_t
gpr_int_write( struct picoif_module *module,
               struct picoif_context *ctx,
               struct picoif_buf *buf,
               size_t len );

static ssize_t
gpr_int_read( struct picoif_module *module,
              struct picoif_context *ctx,
              struct picoif_buf *buf,
              size_t len );

static int
gpr_int_can_write( struct picoif_module *module,
                   struct picoif_context *ctx );

static int
gpr_int_can_read( struct picoif_module *module,
                  struct picoif_context *ctx );

/*! picoIf module operations for the GPR interrupt module. */
static struct picoif_module_ops gpr_int_ops = {
    .destructor             = gpr_int_destructor,
    .create_trans_instance  = gpr_int_create_trans_instance,
    .close_instance         = gpr_int_close_trans_instance,
    .write                  = gpr_int_write,
    .read                   = gpr_int_read,
    .can_read               = gpr_int_can_read,
    .can_write              = gpr_int_can_write,
};

/*! Transport methods for the GPR interrupt module. */
static const char *gpr_int_tmethods[] = {
    "with_values",
    "without_values",
};

/*! The GPR interrupt transport module. */
static struct gpr_int_module gpr_int_mod ={
    .mod = {
        .name       = "gpr_interrupt",
        .tmethods   = gpr_int_tmethods,
        .ops        = &gpr_int_ops,
    },
};

/*!
 * Write to the GPR interrupt transport. This is an invalid operation.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the write.
 * @param buf The data to be written.
 * @param len The length of data to be written in bytes.
 * @return Returns the number of bytes written on success, negative on
 * failure.
 */
static ssize_t
gpr_int_write( struct picoif_module *module,
               struct picoif_context *ctx,
               struct picoif_buf *buf,
               size_t len )
{
    return -EINVAL;
}

/*!
 * Check if the transport can be written to.
 *
 * @param module The module responsible for the context.
 * @param ctx The context of the transport.
 * @return GPR interrupt transports can not be written and will always return
 * 0.
 */
static int
gpr_int_can_write( struct picoif_module *module,
                   struct picoif_context *ctx )
{
    return 0;
}

/*!
 * Check if the transport can be read from.
 *
 * @param module The module responsible for the context.
 * @param ctx The context of the transport.
 * @return Returns 1 if the transport can be read, 0 otherwise.
 */
static int
gpr_int_can_read( struct picoif_module *module,
                  struct picoif_context *ctx )
{
    struct gpr_int_channel *channel = ctx->private_data;
    return ( channel->irq_count > 0 );
}

/*!
 * Read from the GPR interrupt transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the read.
 * @param buf The buffer to write the data into.
 * @param len The number of bytes requested to read.
 * @return Returns the number of bytes read on success, negative on failure.
 */
static ssize_t
gpr_int_read( struct picoif_module *module,
              struct picoif_context *ctx,
              struct picoif_buf *buf,
              size_t len )
{
    struct gpr_int_channel *channel = ctx->private_data;
    struct list_head *pos;
    struct list_head *tmp;
    const unsigned max_values =
        channel->record_values ? len / sizeof( u32 ) : 0xFFFFFFFF;
    unsigned i = 0;
    struct gpr_int_value *val;
    int ret = -EAGAIN;
    ssize_t nbytes = 0;
    unsigned long flags;
    struct list_head list;

    if ( len < 4 )
        goto out;

    INIT_LIST_HEAD( &list );

    PRINTD( COMPONENT_GPR_INT, DBG_TRACE, "read %u bytes", len );

    if ( channel->record_values )
    {
        spin_lock_irqsave( &channel->lock, flags );
        list_for_each_safe( pos, tmp, &channel->values )
        {
            if ( max_values == i++ )
                break;

            /* Move the list entry into the copy list. */
            list_del( pos );
            list_add_tail( pos, &list );
            --channel->irq_count;
        }
        spin_unlock_irqrestore( &channel->lock, flags );

        list_for_each_safe( pos, tmp, &list )
        {
            val = container_of( pos, struct gpr_int_value, list );
            ret = picoif_buf_copy_to( buf, &val->value, nbytes,
                                      sizeof( val->value ) );
            if ( ret )
                goto out;

            nbytes += sizeof( u32 );

            kmem_cache_free( gpr_int_mod.value_cache, val );
        }
    }
    else
    {
        u32 irq_count;
        spin_lock_irqsave( &channel->lock, flags );
        irq_count = channel->irq_count;
        channel->irq_count = 0;
        spin_unlock_irqrestore( &channel->lock, flags );

        ret = picoif_buf_copy_to( buf, &irq_count, 0, sizeof( irq_count ));
        if ( ret )
            goto out;

        if ( irq_count == 0 )
            nbytes = -EAGAIN;
        else
            nbytes = sizeof( irq_count );
    }

    ret = nbytes;

out:
    return ret;
}

/*!
 * Destructor for the gpr interrupt module.
 *
 * @param module The module being destroyed.
 */
static void
gpr_int_destructor( struct picoif_module *module )
{
    kmem_cache_destroy( gpr_int_mod.value_cache );
}

/*!
 * Handler function for interrupts raised by the GPRs in the picoArray. This
 * is passed to the add_irq_handler method of the picoArray and is called when
 * the interrupt is raised and must also clear the interrupt itself. This
 * function will acknowledge the interrupt, and if we are recording values,
 * will record the value to pass to the application when read.
 *
 * @param irq The irq resource that raised the interrupt.
 * @param cookie The channel associated with the interrupt source.
 * @return Returns zero on success, negative on failure.
 */
static int
gpr_int_handler( struct pico_resource *irq,
                 void *cookie )
{
    struct gpr_int_channel *channel = cookie;
    struct picoarray *pa = channel->pa;
    u32 val;

    /* Read the register value. This clears the interrupt at the same time. */
    int ret = pa->ops->register_read( pa, channel->gpr, &val );
    ++channel->irq_count;

    if ( channel->record_values )
    {
        struct gpr_int_value *gpr_val =
            kmem_cache_alloc( gpr_int_mod.value_cache, GFP_ATOMIC );
        ret = -ENOMEM;
        if ( !gpr_val )
            goto out;

        gpr_val->value = val;
        list_add_tail( &gpr_val->list, &channel->values );
        ret = 0;
    }

out:
    wake_up_interruptible( &channel->reader->readq );
    return ret;
}

/*!
 * Create a new GPR interrupt transport instance.
 *
 * @param module The module managing the transport.
 * @param description The type of transport to create.
 * @param params Extra parameters for the transport type.
 * @return Returns the transport context on success, or an ERR_PTR on failure.
 */
static struct picoif_context *
gpr_int_create_trans_instance( struct picoif_module *module,
                               const char *description,
                               struct picoif_buf *params )
{
    int ret = -ENOMEM;
    void *private_data = NULL;
    struct picoif_context *ctx = picoif_new_ctx( module, private_data );
    struct picoif_gpr_int_params gparams;
    struct picoarray *pa = NULL;
    struct pico_resource *irq = NULL;
    struct pico_resource *gpr = NULL;
    struct gpr_int_channel *channel;
    enum picoarray_device_type dev_type;

    ret = picoif_buf_copy_from( &gparams, params, 0,
                                sizeof( gparams ) );
    if ( ret )
        goto out;

    ret = -EINVAL;
    pa = picoif_get_device( gparams.dev_num );
    if ( !pa )
    {
        PRINTD( COMPONENT_GPR_INT, DBG_WARN, "invalid device number: %u",
                gparams.dev_num );
        goto out;
    }

    dev_type = pa->ops->get_device_type( pa );
    if ( !( PICOARRAY_PC202 == dev_type || PICOARRAY_PC302 == dev_type ) )
    {
        PRINTD( COMPONENT_GPR_INT, DBG_WARN,
                "device not supported for this transport" );
        goto out;
    }

    ret = -EBUSY;
    /* Request exclusive access to the IRQ resource. */
    irq = pa->ops->get_resource( pa, PICO_RES_IRQ, gparams.irq_num, 1 );
    if ( !irq )
    {
        PRINTD( COMPONENT_GPR_INT, DBG_WARN, "unabled to get irq: %u",
                gparams.irq_num );
        goto out;
    }

    /* Request exclusive access to the GPR resource. */
    gpr = pa->ops->get_resource( pa, PICO_RES_GPR, irq->metadata, 1 );
    if ( !gpr )
    {
        PRINTD( COMPONENT_GPR_INT, DBG_WARN, "unable to get gpr: %u",
                irq->metadata );
        goto out;
    }

    if ( !ctx )
        goto out;

    /* Allocate the new channel and initialise the data members. */
    ret = -ENOMEM;
    channel = kmalloc( sizeof( *channel ), GFP_KERNEL );
    channel->reader = ctx;
    ctx->private_data = channel;
    if ( !ctx->private_data )
        goto out;

    INIT_LIST_HEAD( &channel->values );

    /* Request the handler function for when the interrupt is raised. */
    channel->irq            = irq;
    channel->gpr            = gpr;
    channel->pa             = pa;
    channel->irq_count      = 0;
    if ( !strcmp( description, "gpr_interrupt(with_values)" ) )
        channel->record_values = 1;
    else if ( !strcmp( description, "gpr_interrupt(without_values)" ) )
        channel->record_values = 0;
    else
    {
        PRINTD( COMPONENT_GPR_INT, DBG_WARN, "invalid tmethod \"%s\"",
                description );
        ret = -EINVAL;
        goto bad_description;
    }

    ret = pa->ops->add_irq_handler( pa, channel->irq, gpr_int_handler,
                                    channel );
    if ( ret )
    {
        PRINTD( COMPONENT_GPR_INT, DBG_WARN,
                "failed to register interrupt handler" );
        goto handler_reg_failed;
    }

    ret = 0;
    goto out;

bad_description:
handler_reg_failed:
    kfree( channel );

out:
    if ( ret )
    {
        kfree( ctx );
        if ( irq )
            pa->ops->put_resource( pa, irq );
        if ( gpr )
            pa->ops->put_resource( pa, gpr );
    }

    return ret ? ERR_PTR( ret ) : ctx;
}

/*!
 * Close an existing transport instance. If this instance is the last one
 * using the channel, then destroy the channel to make sure we don't leak
 * memory.
 *
 * @param mod The module handling the transport.
 * @param ctx The context that is being closed.
 */
static void
gpr_int_close_trans_instance( struct picoif_module *mod,
                              struct picoif_context *ctx )
{
    struct gpr_int_channel *channel = ctx->private_data;
    struct picoarray *pa = channel->pa;

    pa->ops->remove_irq_handler( pa, channel->irq );
    pa->ops->put_resource( pa, channel->irq );
    pa->ops->put_resource( pa, channel->gpr );
    kfree( ctx );
}

struct picoif_context *
picoif_gpr_irq_ul( unsigned dev_num,
                   int irq_num )
{
    struct picoif_gpr_int_params gpr_params = {
        .dev_num    = dev_num,
        .irq_num    = irq_num,
    };
    struct picoif_buf buf = {
        .kbuf       = &gpr_params,
        .is_user    = 0,
    };
    struct picoif_context *ctx =
        gpr_int_create_trans_instance( &gpr_int_mod.mod,
                                       "gpr_interrupt(without_values)", &buf );

    return ctx;
}
EXPORT_SYMBOL( picoif_gpr_irq_ul );

struct picoif_context *
picoif_gpr_irq_ul_with_values( unsigned dev_num,
                               int irq_num )
{
    struct picoif_gpr_int_params gpr_params = {
        .dev_num    = dev_num,
        .irq_num    = irq_num,
    };
    struct picoif_buf buf = {
        .kbuf       = &gpr_params,
        .is_user    = 0,
    };
    struct picoif_context *ctx =
        gpr_int_create_trans_instance( &gpr_int_mod.mod,
                                       "gpr_interrupt(with_values)", &buf );

    return ctx;
}
EXPORT_SYMBOL( picoif_gpr_irq_ul_with_values );

int
gpr_interrupt_init( void )
{
    gpr_int_mod.value_cache =
        kmem_cache_create( "picoif_gpr_int",
                           sizeof( struct gpr_int_value ), 0, 0, NULL );
    if ( !gpr_int_mod.value_cache )
        return -ENOMEM;

    return picoif_register_module( &gpr_int_mod.mod );
}
