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
 * \file pc203.c
 * \brief PC203 device implementation.
 *
 * This file implements the PC203 support of picoIf. All implementation in
 * this file is private and should not be accessed directly by users and only
 * provides the necessary basic services with which transports can be built
 * upon and devices configured.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/dmaengine.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/cpld_hdp102.h>

#include <linux/picochip/devices/pc203.h>
#include "picoarray.h"
#include "resource.h"
#include "picoif_internal.h"
#include "procif.h"
#include "debug.h"
#include "utilities_internal.h"

#include "../dma/fsldma.h"

/*! The offset of the ITM register in the procif. */
#define PC203_PROCIF_ITM_OFFSET     ( 0x70 )
/*! The position of the memif reset bit in the chip control register. */
#define PC203_MEMIF_SOFT_RESET      ( 1 << 29 )
/*! The offset of the chip control register in the procif decode region 2. */
#define CHIP_CONTROL_OFFSET         ( 0x1F )

/*! The CAEID of the procif. */
#define PC203_PROCIF_CAEID          ( 0x48 )
/*! The offset in the procif for the operation request register. */
#define PC203_PROCIF_OP_REQ_OFFSET  ( 0x4018 )
/*! The offset in the procif for the operation status register. */
#define PC203_PROCIF_OP_STATUS_OFFSET  ( 0x401c )
/*! The offset of the start request bit in the operation request register. */
#define PC203_PROCIF_OP_REQ_START   ( 1 << 1 )
/*! The offset of the sync request bit in the operation request register. */
#define PC203_PROCIF_OP_REQ_SYNC    ( 1 << 0 )

/*! Downlinks transfer blocks of 128bytes. DMASrc generics must be
    lowWaterMark=>32, transferLen=>32, DREQMode=>3 */
#define PC203_DOWNLINK_SIZE         128
/*! Uplinks transfer blocks of 32bytes. DMASink generics must be
 *  lowWaterMark=>8, transferLen=>8, DREQMode=>3 */
#define PC203_UPLINK_SIZE           32

/*! The max transfer size that is permitted */
#define PICOARRAY_MAX_TRANSFER      ( 0xFFFFFFFC )

/*! The number of registered PC203s in the system. Until this reaches zero, we
 * can't unregister the platform driver. */
static unsigned num_pc203s;

static void
pc203_destroy( struct picoarray *pa );

/*!
 * \brief PC203 IRQ handler.
 *
 * This structure defines an IRQ handler for PC203 devices and is an internal
 * structure that could be reused for other device types.
 */
struct pc203_irq_handler
{
    /*! The position in the list of handlers for an interrupt source. */
    struct list_head        list;

    /*! The IRQ that this handles. */
    struct pico_resource    *irq;

    /*! A cookie to pass to the callback function when the interrupt is
     * raised. */
    void                    *cookie;

    /*! The callback function to call when the interrupt is raised. */
    int                     ( *callback )( struct pico_resource *irq,
                                           void *cookie );
};

/*!
 * DMA Engine can only pass one parameter to a callback function but the
 * callback function from dma.c needs two. Use this structure to translate
 * between the two callback definitions
 */
struct pc203_dma_cb_param {
    int ( *callback )( void *cookie,
                      int errno );
    void *cookie;
    int chanid;
};

/*!
 * \brief Private representation of a PC203 device.
 *
 * This describes all private data required in the PC203 implementation of
 * this driver.
 *
 * \extends picoarray
 */
struct pc203
{
    /*! The picoArray base class. */
    struct picoarray            pa;

    /*! The physical address of the procif registers. */
    dma_addr_t                  reg_base_phys;

    /*! The length of the procif registers in bytes. */
    size_t                      reg_base_len;

    /*! The virtually mapped address of the procif registers. */
    void __iomem                *reg_base;

    /*! The physical address of the chip control register. */
    dma_addr_t                  ccr_base_phys;

    /*! The length of the chip control register in bytes. */
    size_t                      ccr_base_len;

    /*! The physical address of the ProcIf DMA registers. */
    dma_addr_t                  dma_base_phys;

    /*! The length of the ProcIf DMA registers in bytes. */
    size_t                      dma_base_len;

    /*! The virtually mapped address of the chip control register. */
    void __iomem                *ccr_base;

    /*! The procif IRQ number. */
    unsigned                    procif_irq;

    /*! The handlers registered for the procif IRQ. */
    struct pc203_irq_handler    procif_irq_handlers;

    /*! The DMA engine channel */
    struct dma_chan             *dma_chan[PICO_NUM_DMA_CHANNELS];

    /*! The callback parameters, one per DMA channel */
    struct pc203_dma_cb_param   callback[PICO_NUM_DMA_CHANNELS];
};

/*!
 * Get the PC203 structure given a picoArray base class.
 *
 * @param pa The base class pointer.
 * @return Returns a pointer to the PC203 structure on success, NULL on
 * failure.
 */
static inline struct pc203 *
to_pc203( struct picoarray *pa )
{
    return pa ? container_of( pa, struct pc203, pa ) : NULL;
}

/*!
 * Read a number of 16 bit words from the PC203 procif.
 *
 * @param pa The device to read from.
 * @param caeid The CAEID of the AE to read from.
 * @param address The start address in the AE to begin reading from.
 * @param count The number of 16 bit words to read.
 * @param[out] data The buffer to store the data in.
 * @return Returns the number of words read on success, negative on failure.
 */
static int
pc203_config_read( struct picoarray *pa,
                   u16 caeid,
                   u16 address,
                   u16 count,
                   u16 *data )
{
    struct pc203 *dev = to_pc203( pa );

    PRINTD( COMPONENT_PC203, DBG_TRACE,
            "pa[%u]: config read %u words %04x@%04x", pa->dev_num,
            count, caeid, address );

    return procif_config_read( dev->reg_base, caeid, address, data, count );
}

/*!
 * Write a number of 16 bit words to the PC203 procif.
 *
 * @param pa The device to write to.
 * @param caeid The CAEID of the AE to write to.
 * @param address The start address in the AE to begin writing to.
 * @param count The number of 16 bit words to write.
 * @param[in] data The buffer to write from.
 * @return Returns the number of words written on success, negative on failure.
 */
static int
pc203_config_write( struct picoarray *pa,
                    u16 caeid,
                    u16 address,
                    u16 count,
                    u16 *data )
{
    struct pc203 *dev = to_pc203( pa );

    PRINTD( COMPONENT_PC203, DBG_TRACE,
            "pa[%u]: config write %u words %04x@%04x", pa->dev_num,
            count, caeid, address );

    return procif_config_write( dev->reg_base, caeid, address, data, count );
}

/*!
 * Sync the PC203 device.
 *
 * @param pa The device to sync.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_sync( struct picoarray *pa )
{
    u16 val;

    /* Read the current operation status register value. */
    int ret = pa->ops->config_read( pa, PC203_PROCIF_CAEID,
                                    PC203_PROCIF_OP_STATUS_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    /* If we are running, don't try and sync. */
    ret = 0;
    if ( val & PC203_PROCIF_OP_REQ_START )
        goto out;

    /* Read the current operation request register value. */
    ret = pa->ops->config_read( pa, PC203_PROCIF_CAEID,
                                    PC203_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    /* Set the sync request bit. */
    val |= PC203_PROCIF_OP_REQ_SYNC;
    ret = pa->ops->config_write( pa, PC203_PROCIF_CAEID,
                                 PC203_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    ret = 0;
out:
    return ret;
}

/*!
 * Start the PC203 device running.
 *
 * @param pa The device to start.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_start( struct picoarray *pa )
{
    u16 val;
    int ret = pa->ops->config_read( pa, PC203_PROCIF_CAEID,
                                    PC203_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;
    val |= PC203_PROCIF_OP_REQ_START;
    ret = pa->ops->config_write( pa, PC203_PROCIF_CAEID,
                                 PC203_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    ret = 0;
out:
    return ret;
}

/*!
 * Stop the PC203 device running.
 *
 * @param pa The device to stop.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_stop( struct picoarray *pa )
{
    u16 val;
    int ret = pa->ops->config_read( pa, PC203_PROCIF_CAEID,
                                    PC203_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;
    val &= ~PC203_PROCIF_OP_REQ_START;
    ret = pa->ops->config_write( pa, PC203_PROCIF_CAEID,
                                 PC203_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    ret = 0;
out:
    return ret;
}

/*!
 * Reset the PC203. This performs a soft reset that returns the picoArray to
 * be as close as possible to the hardware reset state without affecting the
 * ARM subsystem.
 *
 * @param pa The device being reset.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_reset( struct picoarray *pa )
{
    struct pc203 *dev = to_pc203( pa );
    unsigned long flags;

    spin_lock_irqsave( &pa->lock, flags );

    /* Assert the reset line. */
    cpld_pico_reset_assert( pa->dev_num );
    /* Wait for 2ms then deassert the reset line. */
    udelay( 2000 );
    cpld_pico_reset_deassert( pa->dev_num );

    /* Reset the memif. */
    picoif_out32( PC203_MEMIF_SOFT_RESET, dev->ccr_base + CHIP_CONTROL_OFFSET );

    spin_unlock_irqrestore( &pa->lock, flags );

    return 0;
}

/*!
 * Read a GPR (general purpose register) in the PC203.
 *
 * @param pa The device being read from.
 * @param reg The register to read.
 * @param[out] value The address to store the register value in.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_register_read( struct picoarray *pa,
                     struct pico_resource *reg,
                     u32 *value )
{
    struct pc203 *dev = to_pc203( pa );

    if ( !reg || PICO_RES_GPR != reg->type )
        return -EINVAL;

    if ( reg->value < PC203_GPR_PROCIF_0 ||
         reg->value >= PICO_NUM_GPRS )
        return -EINVAL;

    procif_reg_read( dev->reg_base, reg->offset, value );

    return 0;
}

/*!
 * Write a GPR (general purpose register) in the PC203.
 *
 * @param pa The device being written to.
 * @param reg The register to write.
 * @param value The value to write to the register.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_register_write( struct picoarray *pa,
                      struct pico_resource *reg,
                      u32 value )
{
    struct pc203 *dev = to_pc203( pa );

    if ( !reg || PICO_RES_GPR != reg->type )
        return -EINVAL;

    if ( reg->value < PC203_GPR_PROCIF_0 ||
         reg->value >= PICO_NUM_GPRS )
        return -EINVAL;

    procif_reg_write( dev->reg_base, reg->offset, value );

    return 0;
}

/*! The resources for a PC203 device.
 *
 * The pA ProcIf DMA channels are memory mapped into the 8560 through CS3:
 *          pA0: 0xc000_0000
 *          pA1: 0xc004_0000
 *          pA2: 0xc008_0000
 *          pA3: 0xc00c_0000
 * Each pA has four DMA channels in decode region 1 which are selected by
 * up_adhi[1:0]. These pins are connected to laddr[14:15] so the FIFOs are
 * mapped to offsets:
 *          DMA0: 0x0_0000
 *          DMA1: 0x1_0000
 *          DMA2: 0x2_0000
 *          DMA3: 0x3_0000
 * The read/write pointers are managed by the ProcIf so the 8560 DMA should
 * always read from the same device address (i.e. no incrementing)
 */
static struct pico_resource pc203_resources[] = {
    { .type = PICO_RES_DMA_CHANNEL, .value = PC203_DMA_PROCIF_0, .offset = 0x00000, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC203_DMA_PROCIF_1, .offset = 0x10000, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC203_DMA_PROCIF_2, .offset = 0x20000, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC203_DMA_PROCIF_3, .offset = 0x30000, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_0, .offset = 0x0, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_1, .offset = 0x4, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_2, .offset = 0x8, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_3, .offset = 0xc, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_4, .offset = 0x10, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_5, .offset = 0x14, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_6, .offset = 0x18, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_7, .offset = 0x1c, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_8, .offset = 0x20, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_9, .offset = 0x24, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_10, .offset = 0x28, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_11, .offset = 0x2c, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_12, .offset = 0x30, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_13, .offset = 0x34, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_14, .offset = 0x38, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_15, .offset = 0x3c, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_16, .offset = 0x40, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_17, .offset = 0x44, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_18, .offset = 0x48, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_19, .offset = 0x4c, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_20, .offset = 0x54, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_21, .offset = 0x5c, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_22, .offset = 0x64, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_PROCIF_23, .offset = 0x6c, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_ITM, .offset = 0x70, },
    { .type = PICO_RES_GPR, .value = PC203_GPR_ITS, .offset = 0x74, },
    { .type = 0, .value = 0 }, /* Sentinel value. Do not remove and keep this
                                * at the end. */
};

/*!
 * Add an IRQ handler to the PC203 for a given interrupt source.
 *
 * @param pa The device to register the interrupt handler with.
 * @param irq The IRQ to attach the handler to.
 * @param callback The callback function to call when the interrupt is
 * raised.
 * @param cookie The cookie to pass to the callback function. This may be NULL
 * if it is not required.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_add_irq_handler( struct picoarray *pa,
                       struct pico_resource *irq,
                       int ( *callback )( struct pico_resource *irq,
                                          void *cookie ),
                       void *cookie )
{
    struct pc203_irq_handler *handler;
    struct pc203 *dev = to_pc203( pa );
    int ret = -ENOMEM;
    unsigned long flags;

    if ( irq->type != PICO_RES_IRQ || !irq->exclusive || !callback )
        return -EINVAL;

    handler = kmalloc( sizeof( *handler ), GFP_KERNEL );
    if ( !handler )
        goto out;

    handler->callback   = callback;
    handler->cookie     = cookie;
    handler->irq        = irq;

    spin_lock_irqsave( &pa->lock, flags );
    list_add( &handler->list, &dev->procif_irq_handlers.list );
    spin_unlock_irqrestore( &pa->lock, flags );

    ret = 0;
out:
    return ret;
}

/*!
 * Remove an IRQ handler from a PC203 interrupt source.
 *
 * @param pa The PC203 to remove the handler from.
 * @param irq The IRQ to remove the handler for.
 */
static void
pc203_remove_irq_handler( struct picoarray *pa,
                          struct pico_resource *irq )
{
    struct list_head *pos;
    struct list_head *tmp;
    struct pc203 *dev = to_pc203( pa );
    int found = 0;
    struct pc203_irq_handler *handler;
    unsigned long flags;

    /* Protect against interrupts being raised whilst we remove the handler. */
    spin_lock_irqsave( &pa->lock, flags );

    list_for_each_safe( pos, tmp, &dev->procif_irq_handlers.list )
    {
        handler = container_of( pos, struct pc203_irq_handler, list );
        if ( handler->irq == irq )
        {
            list_del( pos );
            kfree( handler );
            found = 1;
            break;
        }
    }

    spin_unlock_irqrestore( &pa->lock, flags );
}

/*!
 * PC203 procif ISR. This ISR will be called when the procif generates an
 * interrupts and calls the appropriate handler if there is one registered. If
 * there is no interrupt handler then the ITM register is cleared to prevent
 * further interrupts from being raised.
 *
 * @param irq The irq that has been raised.
 * @param dev The PC203 device that has raised the interrupt.
 * @return Returns IRQ_HANDLED on success.
 */
static irqreturn_t
pc203_procif_irq( int irq,
                  void *dev )
{
    struct pc203 *pc203dev = dev;
    struct list_head *pos;
    struct pc203_irq_handler *handler;
    int ret;
    int handled = 0;

    list_for_each( pos, &pc203dev->procif_irq_handlers.list )
    {
        handler = container_of( pos, struct pc203_irq_handler, list );
        if ( handler->callback )
        {
            ret = handler->callback( handler->irq, handler->cookie );
            if ( ret )
                break;
            handled = 1;
        }
    }

    /* Mask out the interrupt to prevent it from occuring again. */
    if ( !handled )
        procif_reg_write( pc203dev->reg_base, PC203_PROCIF_ITM_OFFSET, 0 );

    return IRQ_HANDLED;
}

/*!
 * DMA callback, call the actuall callback passed to dma_to_device
 *
 * @param cookie A pointer to a pc203_dma_cp_param struct
 */
void pc203_dma_callback( void *cb_param )
{
    struct pc203_dma_cb_param *cookie = (struct pc203_dma_cb_param*)cb_param;
    cookie->callback( cookie->cookie, 0 );
}

/*!
 * DMA a scatter gather list of memory from a kernel mapped scatterlist
 * into a picoArray DMA channel. After the DMA transfer has completed, the
 * callback function will be called with the cookie as the parameter. The
 * caller of this function is responsible for mapping and unmapping the
 * buffers to be transferred into a DMA capable region.
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_chan The DMA channel to use as a destination.
 * @param sgl The scatter gather list of the source data.
 * @param nbytes The number of bytes in the scatter gather list.
 * @param callback The callback function to be called when the transfer
 * has completed. The parameter errno will be set to the status of the DMA
 * operation where 0 == success, negative == failure.
 * @param cookie The cookie to pass to the callback function.
 */
static int
pc203_dma_memcpy( struct picoarray *pa,
                  struct pico_resource *dma_channel,
                  dma_addr_t src,
                  dma_addr_t dst,
                  size_t nbytes,
                  int ( *callback )( void *cookie,
                                    int errno ),
                  void *cookie )
{
    struct pc203 *pc203dev = to_pc203( pa );
    struct dma_async_tx_descriptor *tx;
    struct dma_chan *chan = pc203dev->dma_chan[dma_channel->value];

    tx = chan->device->device_prep_dma_memcpy( chan, dst, src,
                                               nbytes, DMA_PREP_INTERRUPT);
    if (!tx)
        return -ENOMEM;

    pc203dev->callback[dma_channel->value].callback = callback;
    pc203dev->callback[dma_channel->value].cookie = cookie;
    pc203dev->callback[dma_channel->value].chanid = dma_channel->value;

    tx->callback = pc203_dma_callback;
    tx->callback_param = &pc203dev->callback[dma_channel->value];
    tx->parent = NULL;
    tx->tx_submit( tx );

    /* Send the DMA to the hardware */
    dma_async_issue_pending( chan );
    cpld_dma_reset(dma_channel->value);

    return 0;
}

/*!
 * Copy data from the scatter gather list into the pA. The callback function
 * will be called after the transfer is complete.
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_chan The DMA channel to use as a destination.
 * @param sgl The scatter gather list of the source data.
 * @param nbytes The number of bytes in the scatter gather list.
 * @param callback The callback function to be called when the transfer
 * has completed. The parameter errno will be set to the status of the DMA
 * operation where 0 == success, negative == failure.
 * @param cookie The cookie to pass to the callback function.
 */
static int
pc203_dma_to_device( struct picoarray *pa,
                     struct pico_resource *dma_chan,
                     struct scatterlist *sgl,
                     size_t nbytes,
                     int ( *callback )( void *cookie,
                                        int errno ),
                     void *cookie )
{
    struct pc203 *pc203dev = to_pc203( pa );

    dma_addr_t src = sg_dma_address( sgl );
    dma_addr_t dst = pc203dev->dma_base_phys + dma_chan->offset;

    return pc203_dma_memcpy( pa, dma_chan, src, dst, nbytes, callback, cookie );
}

/*!
 * Copy data from the pA into the scatter gather list. The callback function
 * will be called after the transfer is complete.
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_chan The DMA channel to use as a source.
 * @param sgl The scatter gather list of the destination buffer.
 * @param nbytes The number of bytes to transfer.
 * @param callback The callback function to be called when the transfer
 * has completed. The parameter errno will be set to the status of the DMA
 * operation where 0 == success, negative == failure.
 * @param cookie The cookie to pass to the callback function.
 */
static int
pc203_dma_from_device( struct picoarray *pa,
                         struct pico_resource *dma_chan,
                         struct scatterlist *sgl,
                         size_t nbytes,
                         int ( *callback )( void *cookie,
                                            int errno ),
                         void *cookie )
{
    struct pc203 *pc203dev = to_pc203( pa );

    dma_addr_t src = pc203dev->dma_base_phys + dma_chan->offset;
    dma_addr_t dst = sg_dma_address( sgl );

    return pc203_dma_memcpy( pa, dma_chan, src, dst, nbytes, callback, cookie );
}

/*!
 * Choose the specific DMA channel, this will get called once for each
 * DMA channel until we return true. see `dma_filter_fn` in dmaengine.h for
 * details.
 */
static bool filter(struct dma_chan *chan, void *param)
{
    struct pico_resource *dma_channel = (struct pico_resource*)param;

    /* Take the channel that was asked for by pc203_dma_open() */
    if ( chan->chan_id == dma_channel->value )
        return true;
    return false;
}

/*!
 * Open and enable the specified DMA channel
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to open
 * @param is_downlink 1 if the channel is a downlink, 0 for uplink
 * @return 0 on success, a negative number on failure
 */
__must_check static int
pc203_dma_open( struct picoarray *pa,
                struct pico_resource *dma_channel,
                int is_downlink )
{
    dma_cap_mask_t mask;
    struct pc203 *pc203dev = to_pc203( pa );
    struct fsl_dma_chan *fsl_chan;
    struct dma_chan *chan;

    dma_cap_zero( mask );
    dma_cap_set( DMA_MEMCPY, mask );
    dma_cap_set( DMA_PRIVATE, mask );
    chan = dma_request_channel( mask, filter, dma_channel );
    if ( !chan )
    {
        PRINTD( COMPONENT_PC203, DBG_ERROR,
                "Could not open DMA%d",
                dma_channel->value );
        return -ENODEV;
    }

    pc203dev->dma_chan[dma_channel->value] = chan;

    /* Setup the channel as either an uplink or a downlink, this is fsldma
       driver specific and can't be done through the DMA Engine API. */
    fsl_chan = to_fsl_chan( pc203dev->dma_chan[dma_channel->value] );
    fsl_chan->set_dest_loop_size( fsl_chan, 4 );
    fsl_chan->toggle_ext_start( fsl_chan, 1 );
    if ( is_downlink )
        fsl_chan->toggle_ext_pause( fsl_chan, PC203_DOWNLINK_SIZE );
    else
        fsl_chan->toggle_ext_pause( fsl_chan, PC203_UPLINK_SIZE );

    /* Setup the CPLD to fix the DMA lines */
    cpld_dma_enable( dma_channel->value );

    return 0;
}

/*!
 * Close and disable the specified DMA channel
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to close
 * @return 0 in all cases
 */
static int
pc203_dma_close( struct picoarray *pa,
                 struct pico_resource *dma_channel )
{
    struct pc203 *pc203dev = to_pc203( pa );

    cpld_dma_disable(dma_channel->value);

    dma_release_channel( pc203dev->dma_chan[dma_channel->value] );
    pc203dev->dma_chan[dma_channel->value] = NULL;

    return 0;
}

/*!
 * Get the device type of a PC203.
 *
 * @param pa The device to query.
 * @return Always returns PICOARRAY_PC203.
 */
static enum picoarray_device_type
pc203_get_device_type( struct picoarray *pa )
{
    return PICOARRAY_PC203;
}

/*!
 * Get the ITM, ITS and procIF IRQ resources.
 * This function will return the pointers to the resources requested on
 * success or NULL if the resource is already allocated.
 *
 * @param pa The device to take to DMA the data.
 * @param its The ITS resource or NULL on failure
 * @param itm The ITM resource or NULL on failure
 * @param procif_irq The IRQ resource or NULL on failure
 *
 * @return 0 on success, non-zero on failure
 */
static int
pc203_get_procif_resource( struct picoarray *pa,
                           struct pico_resource **its,
                           struct pico_resource **itm,
                           struct pico_resource **procif_irq )
{
    *its = generic_get_resource( pa, PICO_RES_GPR, PC203_GPR_ITS, 1 );
    *itm = generic_get_resource( pa, PICO_RES_GPR, PC203_GPR_ITM, 1 );
    *procif_irq = generic_get_resource( pa, PICO_RES_IRQ, PC203_IRQ_PROCIF, 1 );

    return ( (!(*its)) || (!(*itm)) || (!(*procif_irq)) );
}

/*!
 * Put the ITM, ITS and procIF IRQ resources.
 * This function will free any non NULL resources specified
 *
 * @param pa The device to take to DMA the data.
 * @param its The ITS resource
 * @param itm The ITM resource
 * @param procif_irq The IRQ resource
 */
static void
pc203_put_procif_resource( struct picoarray *pa,
                           struct pico_resource *its,
                           struct pico_resource *itm,
                           struct pico_resource *procif_irq )
{
    if ( its )
        generic_put_resource( pa, its );

    if ( itm )
        generic_put_resource( pa, itm );

    if ( procif_irq )
        generic_put_resource( pa, procif_irq );
}

/*!
 * Load the picoArray specified with the array of data
 *
 * @param pa The device being written to.
 * @param data The virtual address of the buffer to write
 * @param sgl The scatter gather list of the source data.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_pa_load( struct picoarray *pa,
               u32 *data,
               struct scatterlist *sgl )
{
    struct pc203 *dev = to_pc203( pa );
    unsigned i;
    int ret = 0;

    PRINTD( COMPONENT_PC203, DBG_TRACE, "pa[%u]: config reg "
            "write %u words", pa->dev_num, sgl->length );

    for( i=0; i < sgl->length ; i++)
    {
        ret = procif_reg_write( dev->reg_base,
                                PROCIF_REG_CFG_WR_OFFSET, data[i] );
        /* Do not need to check return value, it is always true */
    }

    return i;
}

/*! Operations for the PC203 devices. */
static struct picoarray_ops pc203_ops = {
    .sync                   = pc203_sync,
    .start                  = pc203_start,
    .stop                   = pc203_stop,
    .get_device_type        = pc203_get_device_type,
    .config_read            = pc203_config_read,
    .config_write           = pc203_config_write,
    .register_read          = pc203_register_read,
    .register_write         = pc203_register_write,
    .reset                  = pc203_reset,
    .get_resource           = generic_get_resource,
    .get_procif_resource    = pc203_get_procif_resource,
    .put_resource           = generic_put_resource,
    .put_procif_resource    = pc203_put_procif_resource,
    .destructor             = pc203_destroy,
    .add_irq_handler        = pc203_add_irq_handler,
    .remove_irq_handler     = pc203_remove_irq_handler,
    .dma_to_device          = pc203_dma_to_device,
    .dma_from_device        = pc203_dma_from_device,
    .dma_open               = pc203_dma_open,
    .dma_close              = pc203_dma_close,
    .pa_load                = pc203_pa_load,
};

static int pc203_probe( struct platform_device *pdev );
static int pc203_remove( struct platform_device *pdev );

/*! The PC203 platform driver.
 *  \todo Change the name to PC203 specific rather than generic picoArray.
 */
static struct platform_driver pc203_driver = {
    .probe      = pc203_probe,
    .remove     = pc203_remove,
    .driver     = {
        .name   = "picoArray",
    },
};

/*!
 * Probe method for the PC203 platform driver. This function creates a new
 * PC203 instance and is responsible for allocating all of the resources
 * required.
 *
 * @param pdev The platform device that has been probed.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_probe( struct platform_device *pdev )
{
    struct resource *res;
    int ret;
    struct pc203 *newdev = kmalloc( sizeof( *newdev ), GFP_KERNEL );

    if ( !newdev )
        return -ENOMEM;

    ret = -ENOMEM;
    newdev->pa.resources = kmalloc( sizeof( pc203_resources ), GFP_KERNEL );
    if ( !newdev->pa.resources )
        goto out;
    memcpy( newdev->pa.resources, pc203_resources, sizeof( pc203_resources ) );

    newdev->pa.dev_num = pdev->id;
    newdev->pa.ops = &pc203_ops;
    newdev->pa.features = PICOARRAY_HAS_DMA;
    newdev->pa.max_dma_sz = PICOARRAY_MAX_TRANSFER;
    spin_lock_init( &newdev->pa.lock );

    ret = -EINVAL;
    /* Get the procif IRQ. */
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "procif_irq" );
    if ( !res )
        goto out;
    newdev->procif_irq = res->start;

    /* Get the register base address for the procif. */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM, "procif" );
    if ( !res )
        goto out;
    newdev->reg_base_phys = res->start;
    newdev->reg_base_len = ( res->end - res->start ) + 1;

    /* Get the register base address for the chip control register. */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM,
                                        "ccr_base" );
    if ( !res )
        goto out;
    newdev->ccr_base_phys = res->start;
    newdev->ccr_base_len = ( res->end - res->start ) + 1;

    /* Get the base address of the memory-mapped pA DMA channel */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM, "dma_base" );
    if ( !res )
        goto out;
    newdev->dma_base_phys = res->start;
    newdev->dma_base_len = ( res->end - res->start ) + 1;

    /* Map the resources. */
    newdev->reg_base = request_and_map( "procif", newdev->reg_base_phys,
                                        newdev->reg_base_len );
    if ( !newdev->reg_base )
        goto out;

    newdev->ccr_base = request_and_map( "ccr", newdev->ccr_base_phys,
                                        newdev->ccr_base_len );
    if ( !newdev->ccr_base )
        goto ccr_map_failed;

    ret = request_irq( newdev->procif_irq, pc203_procif_irq, IRQF_DISABLED,
                       pdev->name, newdev );
    if ( ret )
        goto procif_irq_failed;

    /* Register our interest in DMA channels */
    dmaengine_get();

    /* Initialise the interrupt handler lists. */
    INIT_LIST_HEAD( &newdev->procif_irq_handlers.list );

    ret = picoif_register_dev( &newdev->pa );
    goto out;

procif_irq_failed:
    unmap_and_release( newdev->ccr_base_phys, newdev->ccr_base_len,
                       newdev->ccr_base );

ccr_map_failed:
    unmap_and_release( newdev->reg_base_phys, newdev->reg_base_len,
                       newdev->reg_base );
out:
    if ( ret && newdev->pa.resources )
        kfree( newdev->pa.resources );

    if ( ret )
    {
        platform_driver_unregister( &pc203_driver );
        kfree( newdev );
    }
    else
        ++num_pc203s;

    if ( !ret )
    {
        PRINTD( COMPONENT_PC203, DBG_TRACE, "PC203 device %d registered",
                newdev->pa.dev_num );
    }
    return ret;
}

/*!
 * Remove method for the PC203 platform driver. This method is called when the
 * platform driver is removed and must release all resources the driver has
 * been using.
 *
 * @param pdev The platform device being remove.
 * @return Returns zero on success, negative on failure.
 */
static int
pc203_remove( struct platform_device *pdev )
{
    struct picoarray *pa = picoif_get_device( pdev->id );
    struct pc203 *pc203dev = to_pc203( pa );
    int ret = 0;

    free_irq( pc203dev->procif_irq, pc203dev );
    unmap_and_release( pc203dev->ccr_base_phys, pc203dev->ccr_base_len,
                       pc203dev->ccr_base );
    unmap_and_release( pc203dev->reg_base_phys, pc203dev->reg_base_len,
                       pc203dev->reg_base );

    kfree( pc203dev->pa.resources );
    kfree( pc203dev );

    /* Release our interest in DMA channels */
    dmaengine_put();

    return ret;
}

int
pc203_init( void )
{
    return platform_driver_register( &pc203_driver );
}

/*!
 * Destructor to be called when a PC203 is removed from picoif. This
 * function must decrement the number of PC203s registered, and when this
 * reaches zero, remove the platform driver.
 *
 * @param pa The device being removed.
 */
static void
pc203_destroy( struct picoarray *pa )
{
    PRINTD( COMPONENT_PC203, DBG_TRACE, "pA[%u]: destructor called",
            pa->dev_num );

    /* If we have no more pc203s, then remove the driver. */
    if ( 0 == --num_pc203s )
        platform_driver_unregister( &pc203_driver );
}
