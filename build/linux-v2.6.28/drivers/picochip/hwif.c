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
 * \file hwif.c
 * \brief HwIF transport module implementation.
 *
 * This file implements the HwIF DMA transport.
 *
 * Two transport methods are provided:
 *   \li hwif(ul)
 *   \li hwif(int)
 *
 * hwif(ul) supports uplink DMA transfers, and hwif(int) supports the 
 * forwarding of interrupts from the picoArray to host that have no data
 * associated with them.
 *
 * This transport creates one context per device which will permit a maximum of
 * thirty one DMA channels or interrupts to be configured assuming that the
 * picoArray device can support this number. Each instance of the transport
 * requires a interrupt number between 0 and 31 to be assigned which must
 * match the interrupt bit set in the ITS register by the picoArray interface
 * code. One GPR is required per context for clearing all the interrupts.
 *
 * There is only limited checking that can be performed for consistency 
 * between the picoArray interface code and the application code.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include "debug.h"
#include "picoarray.h"
#include "picoif_internal.h"
#include "picoif_module.h"
#include "hwif_internal.h"
#include "dma_fifo_internal.h"
#include <linux/picochip/transports/hwif.h>
#include <linux/picochip/picoif_ioctl.h>

/*! The maximum number of interrupts that the transport supports. */
#define HWIF_MAX_INTERRUPT_NUM ( 31 )

/*! The maximum number of devices that the driver supports. */
#define HWIF_MAX_DEVICES       ( 8 )

/*!
 * \brief The context parameters used on each device
 */
struct hwif_ctx_params
{
    uint32_t interrupts_set;         /*!< The active interrupts on this device */
    int int_clear_gpr;               /*!< The interrupt clear GPR */
    struct pico_resource *int_clear; /*!< The int clear GPR resource */
    struct pico_resource *its;       /*!< The ITS resource */
    struct pico_resource *itm;       /*!< The ITM resource */
    struct pico_resource *irq;       /*!< The procif IRQ resource */
    struct picoarray *pa;            /*!< The picoArray running the context */
    struct dma_channel *channel[HWIF_MAX_INTERRUPT_NUM];
                                     /*!< DMA channels attached to this context */
    spinlock_t lock;                 /*!< Context parameter lock */
};

/*!
 * \brief The basic HwIF module.
 * \extends picoif_module
 */
struct hwif_module
{
    struct picoif_module mod;        /*!< The generic module. */
    struct hwif_ctx_params params[HWIF_MAX_DEVICES];
                                     /*!< Common parameters this transport */
};

/*!
 * \brief DMA channel parameters (one per DMA channel). 
 */
static struct dma_channel
{
    struct picoif_context *reader;    /*!< The reader of the transport. */
    unsigned activeChannel;           /*!< Boolean to determine if the DMA
                                       *   channel is active */
    unsigned isPureInterrupt;         /*!< Boolean to determine if the
                                       *   transport is a pure interrupt or
                                       *   carrying data */
    unsigned interruptNumber;         /*!< Interrupt number for the channel */
    struct pico_resource *chan;       /*!< Physical DMA channel */
    uint32_t irqCount;                /*!< Number of interrupts raised since
                                       *   last read (for pure interrupts only) */
    unsigned xferSize;                /*!< Size of transfer in bytes */
    unsigned bytesSent;               /*!< Bytes sent so far */
    unsigned totalBytes;              /*!< Total bytes */
    unsigned maxTransferSize;         /*!< Max transfer size in bytes */
    struct dma_fifo_t *fifo;          /*!< DMA FIFO */
    spinlock_t lock;                  /*!< DMA parameter lock */
    struct pico_resource *status;     /*!< The DMA status GPR resource */
    struct pico_resource *count;      /*!< The DMA count GPR resource */
    struct hwif_ctx_params *hwifCtx;  /*!< Pointer to the context parameters used
                                       *   in this channel */
}
dma_channel;

static void
hwif_destructor( struct picoif_module *module );

static struct picoif_context *
hwif_create_trans_instance( struct picoif_module *module,
                            const char *description,
                            struct picoif_buf *params );

static void
hwif_close_trans_instance( struct picoif_module *module,
                           struct picoif_context *ctx );

static ssize_t
hwif_write( struct picoif_module *module,
            struct picoif_context *ctx,
            struct picoif_buf *buf,
            size_t len );

static ssize_t
hwif_read( struct picoif_module *module,
           struct picoif_context *ctx,
           struct picoif_buf *buf,
           size_t len );

static int
hwif_can_write( struct picoif_module *module,
                struct picoif_context *ctx );

static int
hwif_can_read( struct picoif_module *module,
               struct picoif_context *ctx );

static int
hwif_queue_transfer( struct dma_channel *channel );

/*! picoIf module operations for the DMA module. */
static struct picoif_module_ops hwif_ops = {
    .destructor             = hwif_destructor,
    .create_trans_instance  = hwif_create_trans_instance,
    .close_instance         = hwif_close_trans_instance,
    .write                  = hwif_write,
    .read                   = hwif_read,
    .can_read               = hwif_can_read,
    .can_write              = hwif_can_write,
};

/*! Transport methods for the DMA transport module */
static const char *hwif_tmethods[] = {
    "ul",
};

/*! The HwIF transport */
static struct hwif_module hwif_mod ={
    .mod = {
        .name       = "hwif",
        .tmethods   = hwif_tmethods,
        .ops        = &hwif_ops,
    },
};

/*!
 * Check if the transport can be written to.
 *
 * @param module The module responsible for the context.
 * @param ctx The context of the transport.
 * @return HwiF2 transports cannot be written and will always return 0
 */
static int
hwif_can_write( struct picoif_module *module,
                struct picoif_context *ctx )
{
    return 0;
}

/*!
 * Check if the transport can be read from.
 *
 * @param module The module responsible for the context.
 * @param ctx The context of the transport.
 * @return Returns 1 if the transport can be read (and has data ready),
 * 0 otherwise.
 */
static int
hwif_can_read( struct picoif_module *module,
               struct picoif_context *ctx )
{
    int ret = 0;
    struct dma_channel *channel = ctx->private_data;

    if ( channel->isPureInterrupt )
        return ( channel->irqCount > 0 );
    else
    {
        unsigned long flags;
        unsigned bytes_to_read = dma_fifo_used( channel->fifo );

        if ( bytes_to_read )
            return 1;

        /* Start any queued transfers for this channel */
        spin_lock_irqsave( &channel->lock, flags );
        ret = hwif_queue_transfer(channel );
        spin_unlock_irqrestore( &channel->lock, flags );
        return 0;
    }
}

/*!
 * Handler function for interrupts generated by a completed HwIF transfer. 
 *
 * @param cookie The channel associated with the interrupt source.
 * @param errno The error number associated with the DMA transfer
 * @return Returns zero on success, negative on failure.
 */
__must_check static int
hwif_dma_handler( void *cookie,
                  int errno )
{
    struct dma_channel *channel = cookie;
    int ret = 0;
    unsigned long flags;

    if ( errno )
    {
        PRINTD( COMPONENT_HWIF, DBG_ERROR,"DMA transfer completed with "
            "error %d (cookie=0x%p)", errno, cookie );
        return errno;
    }
    else
    {
        PRINTD( COMPONENT_HWIF, DBG_TRACE,"tranfer of %u bytes completed",
            channel->xferSize );
    }

    spin_lock_irqsave( &channel->lock, flags );

    /* Add data transferred to FIFO control parameters */
    dma_fifo_add_transfer( channel->fifo, channel->xferSize );
    channel->bytesSent += channel->xferSize;
    channel->activeChannel = 0;

    if ( channel->bytesSent == channel->totalBytes )
    {
        wake_up_interruptible( &channel->reader->readq );
        channel->totalBytes = 0;
        channel->bytesSent = 0;
        channel->xferSize = 0;
    }

    ret = hwif_queue_transfer( channel );

    spin_unlock_irqrestore( &channel->lock, flags );

    return ret;
}

/*!
 * Write to the DMA transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the write.
 * @param buf The data to be written.
 * @param len The length of data to be written in bytes.
 * @return Returns EINVAL as a write is not permitted for this transport
 */
static ssize_t
hwif_write( struct picoif_module *module,
            struct picoif_context *ctx,
            struct picoif_buf *buf,
            size_t len )
{
    return -EINVAL;
}

/*!
 * Read from the DMA transport.
 *
 * @param module The module managing the transport.
 * @param ctx The context performing the read.
 * @param buf The buffer to write the data into.
 * @param len The number of bytes requested to read.
 * @return Returns the number of bytes read on success, negative on failure.
 */
static ssize_t
hwif_read( struct picoif_module *module,
           struct picoif_context *ctx,
           struct picoif_buf *buf,
           size_t len )
{
    struct dma_channel *channel = ctx->private_data;
    int ret = 0;

    if ( channel->isPureInterrupt )
    {
        unsigned long flags;

        spin_lock_irqsave( &channel->lock, flags );
        if ( ( len < sizeof( channel->irqCount ) ) ||
                                        ( !channel->irqCount ) )
        {
            ret = -EAGAIN;
        }
        else
        {
            PRINTD( COMPONENT_HWIF, DBG_TRACE, "read %u bytes,irq count=%d",
                 len, channel->irqCount );

            ret = picoif_buf_copy_to( buf, &channel->irqCount, 0,
                                  sizeof( channel->irqCount ) );
            channel->irqCount = 0;
            if ( !ret )
            {         
                ret = sizeof( channel->irqCount );
            }
        }

        spin_unlock_irqrestore( &channel->lock, flags );
        return ret;
    }
    else
    {
        unsigned used_bytes=0;
        unsigned long flags;

        /* Check that the length is invalid */
        if ( len & 0x3 )
            return -EINVAL;

        /* Determine how much data is already in the FIFO */
        used_bytes = dma_fifo_get( channel->fifo, buf, len, 0 );
        if ( used_bytes < len )
           used_bytes += dma_fifo_get( channel->fifo, buf, (len-used_bytes),
                                used_bytes ); 

        /* Start any queued transfers for this channel */
        spin_lock_irqsave( &channel->lock, flags );
        ret = hwif_queue_transfer(channel );
        spin_unlock_irqrestore( &channel->lock, flags );
        
        if ( !used_bytes )
            return -EAGAIN;

        PRINTD( COMPONENT_HWIF, DBG_TRACE,
                "read %u bytes (requested %u bytes)", used_bytes, len );

        if ( ret )
            return ret;
        else
            return used_bytes;
    }
}

/*!
 * Destructor for the DMA module.
 *
 * @param module The module being destroyed.
 */
static void
hwif_destructor( struct picoif_module *module )
{
}

/*!
 * Handler function for processing interrupts raised by the procIF. This function
 * is passed to the add_irq_handler() method of the picoArray and is called when the
 * prociIF IRQ is asserted. This function will acknowledge the interrupt, and start a
 * DMA transfer of the required size.
 *
 * @param irq The irq resource that raised the interrupt.
 * @param cookie The context associated with the the interrupt.
 * @return Returns zero on success, negative on failure.
 */
__must_check static int
hwif_int_handler( struct pico_resource *irq,
                    void *cookie )
{
    struct hwif_ctx_params *params = cookie;
    struct picoarray *pa = params->pa;
    int ret = 0;
    unsigned i=0;
    unsigned long flags;
    uint32_t its=0;

    /* Determine cause of interrupt. Make a local copy of ITS.
     */
    ret = pa->ops->register_read( pa, params->its, &its );
    if ( ret )
    {
        return ret;
    }

    if ( !its )
    {
        return 0;
    }

    /* Clear interrupt */
    ret = pa->ops->register_write( pa, params->its, 0 );
    if ( ret )
    {
        return ret;
    }

    /* Scan through pure interrupts */
    spin_lock_irqsave( &params->lock, flags );
    for( i=0; i < HWIF_MAX_INTERRUPT_NUM; i++ )
    {
        if ( its & ( 1 << i ) )
        {
            if ( params->interrupts_set & ( 1 << i ) )
            {
                if ( params->channel[i]->isPureInterrupt )
                {
                    /* Clear interrupt */
                    ret = pa->ops->register_write( pa, params->int_clear, ( 1 << i ) );

                    PRINTD( COMPONENT_HWIF, DBG_TRACE,
                       "pure int:%d ITS:0x%08x", i, its );

                    spin_lock_irqsave( &params->channel[i]->lock, flags );
                    ++params->channel[i]->irqCount;
                    wake_up_interruptible( &params->channel[i]->reader->readq );
                    spin_unlock_irqrestore( &params->channel[i]->lock, flags );

                }
                else
                {
                    /* Clear interrupt */
                    ret = pa->ops->register_write( pa, params->int_clear, ( 1 << i ) );

                    PRINTD( COMPONENT_HWIF, DBG_TRACE,
                       "data int:%d ITS:0x%08x", i, its );

                    spin_lock_irqsave( &params->channel[i]->lock, flags );
                    params->channel[i]->irqCount = 1;
                    ret = hwif_queue_transfer( params->channel[i] );
                    spin_unlock_irqrestore( &params->channel[i]->lock, flags );
          
                }
            }
            else
                PRINTD( COMPONENT_HWIF, DBG_WARN, "int:%d ITS:0x%08x no "
                        "handler for DMA channel", i, its );
        }
    }

    spin_unlock_irqrestore( &params->lock, flags );
    return 0;
}

/*!
 * Function that manages the queuing of packets for DMA transfer. This function
 * will add the data (if provided) to the tail of a queue, and if there is no
 * transfer in progress, extract the nexttransfer from the head of the queue.
 *
 * @param channel The DMA channel handle
 * @return Returns zero on success, non-zero on failure.
 */
__must_check static int
hwif_queue_transfer( struct dma_channel *channel )
{
    unsigned maxsize=0;
    uint32_t size=0;
    int ret = 0;
    struct picoarray *pa = channel->hwifCtx->pa;

    if ( channel->activeChannel )
    {
        return 0;
    }

    if ( !channel->totalBytes && channel->irqCount )
    {
        uint32_t old_size = 1; /* Number that should never occur */
        unsigned loop = 10000;
    
        /* Read the status GPR for 2 identical values */
        while(loop > 0)
        {
            ret = pa->ops->register_read( pa, channel->status, &size );
            if ( ret )
            {
                size = 0;
                break;
            }
            
            if ( size == old_size )
                break;

            old_size = size;
            loop--;
        }

        if ( !loop )
        {
            PRINTD( COMPONENT_HWIF, DBG_ERROR,
                "failed to read a consistent value for status register");
        }
        else
        {
            if ( size > channel->maxTransferSize )
            {
                PRINTD(COMPONENT_HWIF, DBG_WARN,
                    "Transfer size exceeds buffer size or max allowed by DMA. "
                    " Will truncate");
                size = channel->maxTransferSize;
            }
            else
            {
                PRINTD(COMPONENT_HWIF, DBG_TRACE,
                                     "GPR status read, value=0%u", size);
            }
            channel->totalBytes = size;
        }
    }

    size = channel->totalBytes-channel->bytesSent;
    maxsize = dma_fifo_space( channel->fifo );
    if ( size > maxsize )
       size = maxsize;

    if ( size > 0 )
    {
        struct scatterlist sgl;
        sgl.dma_address = dma_fifo_get_writeptr( channel->fifo );
        sgl.length = dma_fifo_space( channel->fifo ); /* Report the size of
                the buffer available from the write pointer */

        channel->xferSize = size;
        channel->activeChannel = 1;

        /* Report how much data we are going to accept */
        ret = pa->ops->register_write( pa, channel->count, size );
        if ( ret )
            PRINTD( COMPONENT_HWIF, DBG_ERROR,
                "Failure to request bytes to receive Sent=%u "
                "Transfered=%u Total=%u bytes",
                channel->bytesSent, size, channel->totalBytes ); 

        ret = pa->ops->dma_from_device( pa, channel->chan, &sgl, size,
                    hwif_dma_handler, channel );
        if ( ret )
        {
            PRINTD( COMPONENT_HWIF, DBG_ERROR,
                "Failure to begin transfer Sent=%u Transfered=%u Total=%u bytes",
                channel->bytesSent, size, channel->totalBytes );
        }
        else 
        {
            PRINTD( COMPONENT_HWIF, DBG_TRACE,
                "Sent=%u Transfered=%u Total=%u bytes", channel->bytesSent, 
            size, channel->totalBytes );
        }
    }
    else if ( !maxsize )
         PRINTD( COMPONENT_HWIF, DBG_WARN,
            "FIFO full on channel int number %",channel->interruptNumber );

    return ret;
}

/*!
 * Create a and start the DMA transport.
 *
 * @param module The module managing the transport.
 * @param description The type of transport to create.
 * @param params Extra parameters for the transport type.
 * @return Returns the transport context on success, or an ERR_PTR on failure.
 */
static struct picoif_context *
hwif_create_trans_instance( struct picoif_module *module,
                             const char *description,
                             struct picoif_buf *params )
{
    int ret = -ENOMEM;
    void *private_data = NULL;
    struct picoif_context *ctx = picoif_new_ctx( module, private_data );
    struct picoif_hwif_params gparams;
    struct picoarray *pa = NULL;
    struct dma_channel *channel;
    struct pico_resource *chan = NULL;
    enum picoarray_device_type dev_type;
    struct pico_resource *irq = NULL;
    struct pico_resource *status = NULL;
    struct pico_resource *count = NULL;
    unsigned long flags;

    ret = picoif_buf_copy_from( &gparams, params, 0,
                                sizeof( gparams ) );
    if ( ret )
        goto out;

    ret = -EINVAL;
    pa = picoif_get_device( gparams.dev_num );
    if ( !pa )
    {
        PRINTD( COMPONENT_HWIF, DBG_WARN, "invalid device number: %u",
                gparams.dev_num );
        goto out;
    }

    dev_type = pa->ops->get_device_type( pa );
    if ( !( PICOARRAY_PC202 == dev_type || PICOARRAY_PC203 == dev_type ) )
    {
        PRINTD( COMPONENT_HWIF, DBG_WARN,
                "device not supported for this transport" );
        goto out;
    }

    if ( !strcmp( description, "hwif(ul)" ) )
    {
        /* Check that the DMA channel is not already in use */
        ret = -EBUSY;
        chan = pa->ops->get_resource( pa, PICO_RES_DMA_CHANNEL,
                                             gparams.channel, 1 );
        if ( !chan )
        {
            PRINTD( COMPONENT_HWIF, DBG_ERROR, "invalid DMA channel:%u",
                gparams.channel );
            goto out;
        }

        /* Get the status and count GPR resources */
        status = pa->ops->get_resource( pa, PICO_RES_GPR, gparams.status_gpr, 1 );
        if ( !status )
        {
            PRINTD( COMPONENT_HWIF, DBG_WARN, "unable to get DMA status gpr: %u",
                gparams.status_gpr );
            goto out;
        }
 
        count = pa->ops->get_resource( pa, PICO_RES_GPR, gparams.count_gpr, 1 );
        if ( !count )
        {
            PRINTD( COMPONENT_HWIF, DBG_WARN, "unable to get DMA count gpr: %u",
                gparams.count_gpr );
            goto out;
        }
    }

    /* Check that the interrupt number is valid */
    if (gparams.dev_num >= HWIF_MAX_INTERRUPT_NUM)
        goto out;

    /* If this is the first transport on the device, set the context
       parameters */
    if ( !hwif_mod.params[gparams.dev_num].interrupts_set )
    {
        struct pico_resource *int_clear = pa->ops->get_resource( pa,
            PICO_RES_GPR, gparams.int_clear_gpr, 1 );
        struct pico_resource *its = NULL;
        struct pico_resource *itm = NULL;

        (void)pa->ops->get_procif_resource( pa, &its, &itm, &irq );

        ret = -EBUSY;
        if (( !int_clear ) || ( !its ) || ( !itm ) || ( !irq ))
        {
            if ( !int_clear)
                PRINTD( COMPONENT_HWIF, DBG_WARN, "unable to get int clear "
                    "gpr: %u", gparams.int_clear_gpr );
            else
                pa->ops->put_resource( pa, int_clear );

            if ( !its)
                PRINTD( COMPONENT_HWIF, DBG_WARN, "unable to get ITS gpr" );

            if ( !itm)
                PRINTD( COMPONENT_HWIF, DBG_WARN, "unable to get ITM gpr" );

            if ( !irq)
                PRINTD( COMPONENT_HWIF, DBG_WARN, "unable to procIF irq" );

            pa->ops->put_procif_resource( pa, its, itm, irq );

            goto out;
        }

        hwif_mod.params[gparams.dev_num].interrupts_set |= (1 << gparams.int_num);
        hwif_mod.params[gparams.dev_num].int_clear_gpr = gparams.int_clear_gpr;
        hwif_mod.params[gparams.dev_num].int_clear = int_clear;
        hwif_mod.params[gparams.dev_num].its = its;
        hwif_mod.params[gparams.dev_num].itm = itm;
        hwif_mod.params[gparams.dev_num].irq = irq;
        hwif_mod.params[gparams.dev_num].pa = pa;

        spin_lock_init( &hwif_mod.params[gparams.dev_num].lock );

    }
    else
    { /* Check that the common parameters are consistent */
        if (( gparams.int_clear_gpr !=
                          hwif_mod.params[gparams.dev_num].int_clear_gpr ) ||
            ( hwif_mod.params[gparams.dev_num].interrupts_set &
                                                        (1 << gparams.int_num) ))
            goto out;

        hwif_mod.params[gparams.dev_num].interrupts_set |= (1 << gparams.int_num); 
    }

    if ( !ctx )
        goto out;

    /* Allocate the new channel and initialise the data members. */
    ret = -ENOMEM;
    channel = kmalloc( sizeof( dma_channel ), GFP_KERNEL );
    if (!channel)
        goto out;
    ctx->private_data = channel;

    if ( !strcmp( description, "hwif(ul)" ) )
    { 
        ret = dma_fifo_create( &channel->fifo, gparams.buf_size );
        if (ret)
            goto bad_fifo_alloc;
    }

    channel->reader = ctx;
    channel->hwifCtx = &hwif_mod.params[gparams.dev_num]; 
    channel->interruptNumber = gparams.int_num;
    if ( pa->max_dma_sz < gparams.buf_size )
        channel->maxTransferSize = pa->max_dma_sz;
    else
        channel->maxTransferSize = gparams.buf_size;
    channel->chan = chan;
    channel->xferSize = 0;
    channel->bytesSent = 0;
    channel->totalBytes = 0;
    channel->activeChannel = 0;
    channel->status = status;
    channel->count = count;
    channel->irqCount = 0;
 
    hwif_mod.params[gparams.dev_num].channel[gparams.int_num] = channel;

    spin_lock_init( &channel->lock );

    if ( !strcmp( description, "hwif(ul)" ) )
    {
        channel->isPureInterrupt = 0;
        ret = pa->ops->dma_open( pa, chan, 0);
        if ( ret )
        {
            PRINTD( COMPONENT_HWIF, DBG_ERROR, "failed to open DMA channel" );
            goto bad_description;
        }
    }
    else if ( !strcmp( description, "hwif(int)" ) )
    {
        channel->fifo = NULL;
        channel->isPureInterrupt = 1;
    }
    else
    {
        PRINTD( COMPONENT_HWIF, DBG_ERROR, "invalid tmethod \"%s\"",
                description );
        ret = -EINVAL;
        goto bad_description;
    }

    if ( irq )
    {
        ret = pa->ops->add_irq_handler( pa, irq, hwif_int_handler,
                                        channel->hwifCtx );
        if ( ret )
        {
            PRINTD( COMPONENT_HWIF, DBG_ERROR,
                "failed to register interrupt handler" );
            goto handler_reg_failed;
        }
    }

    /* Enable interrupt */
    spin_lock_irqsave( &channel->hwifCtx->lock, flags );
    ret = pa->ops->register_write( pa, channel->hwifCtx->itm,
                     channel->hwifCtx->interrupts_set );
    spin_unlock_irqrestore( &channel->hwifCtx->lock, flags );

    ret = 0;
    goto out;

bad_description:
handler_reg_failed:
   if ( channel->fifo )
       dma_fifo_destroy( channel->fifo );

bad_fifo_alloc:
    kfree( channel );

out:
    if ( ret )
    {
        kfree( ctx );

        if ( chan )
            pa->ops->put_resource(  pa, chan );

        if ( status )
            pa->ops->put_resource(  pa, status );

        if ( count )
            pa->ops->put_resource(  pa, count );
    }

    return ret ? ERR_PTR( ret ) : ctx;
}

/*!
 * Close an existing transport instance.
 *
 * @param mod The module handling the transport.
 * @param ctx The context that is being closed.
 */
static void
hwif_close_trans_instance( struct picoif_module *mod,
                          struct picoif_context *ctx )
{
    struct dma_channel *channel = ctx->private_data;
    struct picoarray *pa = channel->hwifCtx->pa;
    unsigned long flags;
    unsigned interrupt_bit = ((uint32_t)1) << channel->interruptNumber;
    struct hwif_ctx_params *params = channel->hwifCtx;
    int ret = 0;

    if ( !channel->isPureInterrupt )
        ret = pa->ops->dma_close( pa, channel->chan );

    if ( channel->chan )
        pa->ops->put_resource( pa, channel->chan );

    if ( channel->fifo )
        dma_fifo_destroy( channel->fifo );

    params->interrupts_set &= ( ~interrupt_bit );

    if ( !params->interrupts_set )
    {
        pa->ops->put_resource( pa, params->int_clear );
        pa->ops->put_procif_resource( pa, params->its, params->itm,
                      params->irq );
    }

    if ( channel->status )
        pa->ops->put_resource( pa, channel->status );

    if ( channel->count )
        pa->ops->put_resource( pa, channel->count );

    /* Disable interrupt */
    spin_lock_irqsave( &params->lock, flags );
    ret = pa->ops->register_write( pa, params->itm, params->interrupts_set );
    spin_unlock_irqrestore( &params->lock, flags );
 
    params->channel[channel->interruptNumber] = NULL;
    if ( channel )
        kfree( channel );

    if ( ctx )
        kfree( ctx );
}

/* Kernel API and Public functions */
struct picoif_context *
picoif_hwif_dmaul_open( const picoif_hwif_t hwif_context,
                         unsigned interrupt_number,
                         int dma_channel,
                         int dma_status_gpr,
                         int dma_count_gpr,
                         size_t buffer_size )
{
    struct picoif_hwif_params hwif_params = {
        .dev_num        = hwif_context->dev_num,
        .int_num        = interrupt_number,
        .channel        = dma_channel,
        .status_gpr     = dma_status_gpr,
        .count_gpr      = dma_count_gpr,
        .buf_size       = buffer_size,
        .int_clear_gpr = hwif_context->int_clear_gpr,
    };
    struct picoif_buf buf = {
        .kbuf       = &hwif_params,
        .is_user    = 0,
    };
    struct picoif_context *ctx =
        hwif_create_trans_instance( &hwif_mod.mod,
                                       "hwif(ul)", &buf );
    return ctx;
}
EXPORT_SYMBOL( picoif_hwif_dmaul_open );

int
hwif_init( void )
{
    memset(hwif_mod.params, sizeof ( hwif_mod.params ), 0 );

    return picoif_register_module( &hwif_mod.mod );
}


