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
 * \file pc202.c
 * \brief PC202 device implementation.
 *
 * This file implements the PC202 support of picoIf. All implementation in
 * this file is private and should not be accessed directly by users and only
 * provides the necessary basic services with which transports can be built
 * upon and devices configured.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/dma.h>

#include <linux/picochip/devices/pc202.h>
#include "picoarray.h"
#include "resource.h"
#include "picoif_internal.h"
#include "procif.h"
#include "debug.h"
#include "soft_reset.h"
#include "utilities_internal.h"
#include "procif.h"
#include <mach/hardware.h>

/*! The offset of the data register for a virtual port in the AHB2Pico. */
#define AHB2PICO_VP_DATA            ( 0x08 )

/*! The address of the AHB2Pico interrupt status register. This is an offset
 * from the AHB2Pico base address. */
#define AHB2PICO_INT_STATUS_OFFSET  ( 0x01E0 )

/*! The number of virtual ports in the AHB2Pico. */
#define AHB2PICO_NUM_VPS           ( 32 )

/*! The CAEID of the procif. */
#define PC202_PROCIF_CAEID          ( 0x48 )
/*! The offset in the procif for the operation request register. */
#define PC202_PROCIF_OP_REQ_OFFSET  ( 0x4018 )
/*! The offset in the procif for the operation status register. */
#define PC202_PROCIF_OP_STATUS_OFFSET  ( 0x401c )
/*! The offset of the start request bit in the operation request register. */
#define PC202_PROCIF_OP_REQ_START   ( 1 << 1 )
/*! The offset of the sync request bit in the operation request register. */
#define PC202_PROCIF_OP_REQ_SYNC    ( 1 << 0 )

/*! The offset of a virtual port config register from the virtual port
 *  register base. */
#define AHB2PICO_VP_CONFIG_OFFSET   ( 0x00 )
/*! The offset of a virtual port status register from the virtual port
 *  register base. */
#define AHB2PICO_VP_STATUS_OFFSET   ( 0x04 )
/*! The offset of a virtual port data register from the virtual port
 *  register base. */
#define AHB2PICO_VP_DATA_OFFSET     ( 0x08 )
/*! The spacing of virtual ports in the AHB2Pico. */
#define AHB2PICO_VP_SPACING         ( 0x10 )

/*! The number of DMACs in the system */
#define PICO_NUM_DMACS              ( 0x02)

/*! The FIFO size for the DMA tasklet */
#define DMA_FIFO_SIZE               ( 9 )

/*! DMA burst size in words */
#define PC202_DMA_BURST_SIZE        ( 1 )

/*! The maximum number of transactions per transfer */
#define DMAH_CHX_MAX_BLK_SIZE       ( 4095 )

/*! Determine the msize and max transfer size that is permitted */
#if ((PC202_DMA_BURST_SIZE == 1) || (PC202_DMA_BURST_SIZE == 2))
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = MS_1_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 4)
#elif (PC202_DMA_BURST_SIZE == 4)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = MS_4_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 8)
#elif (PC202_DMA_BURST_SIZE == 8)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = MS_8_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 16)
#elif (PC202_DMA_BURST_SIZE == 16)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = MS_16_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 32)
#elif (PC202_DMA_BURST_SIZE == 32)   
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = MS_32_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 64)
#else
    #error
#endif

/*!
 * Determine if a GPR port is enabled
 *
 * @param _sr The value of the status register for the VP.
 * @return Returns non-zero if the VP is enabled, zero otherwise.
 */
#define IS_PORT_ENABLED( _sr ) ( _sr & 0x1 )

/*!
 *  Determine if a GPR port is configured as a receive port
 *
 *  @param _sr The value of the status register for the VP.
 *  @return Returns non-zero if the VP is configured as a receive port
 *  (pA->ARM), zero otherwise.
 */
#define IS_PORT_A_RECEIVE( _sr ) ( _sr & 0x2 )

/*!
 *  Determine if a GPR port is configured as a non blocking port
 *
 *  @param _sr The value of the status register for the VP.
 *  @return Returns non-zero if the VP is configured as non-blocking, zero
 *  otherwise.
 */
#define IS_PORT_NONBLOCKING( _sr ) ( _sr & 0x4 )

/*!
 *  Determine if a GPR port is ready to give or receive data
 *
 *  @param _sr The value of the status register for the VP.
 *  @return Returns non-zero if the VP can accept data, zero otherwise.
 */
#define IS_PORT_READY_FOR_DATA( _sr ) ( _sr & 0x8 )

/*! The number of registered PC202s in the system. Until this reaches zero, we
 * can't unregister the platform driver. */
static unsigned num_pc202s;

static void
pc202_destroy( struct picoarray *pa );

/*!
 * \brief PC202 IRQ handler.
 *
 * This structure defines an IRQ handler for PC202 devices and is an internal
 * structure that could be reused for other device types.
 */
struct pc202_irq_handler
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
 * \brief PC202 DMA tasklet FIFO parameters.
 *
 * This structure defines a small FIFO used by a tasklet for DMA transfers
 */
struct pc202_dma_fifo_params
{
    /*!< DMA channel */
    struct pc202_dma_channel *dma;

    /*!< Error number associated with the transfer */
    int errno;
};

/*!
 * \brief PC202 DMA tasklet structure.
 *
 * This structure defines a small FIFO used by a tasklet for DMA transfers
 */
struct pc202_dma_fifo
{
    /*!< FIFO read pointer */
    unsigned                readPtr;

    /*!< FIFO write pointer */
    unsigned                writePtr;

    /*!< FIFO parameters */
    struct pc202_dma_fifo_params fifo[DMA_FIFO_SIZE];
};

/*!
 * \brief PC202 DMA handler
 *
 * This structure defines a DMA handler for PC202 devices and is an internal
 * strucutre that might be reused for other device types that uses the Synopsys
 * DMAC. There will be one instance of this structure per DMA channel in the
 * system.
 */
struct pc202_dma_channel
{
    /*! Structure holding the current transfer details */
    firecracker_dma_xfr_t xfr;

    /*! Definition of the hardware handshaking interface set for this channel */
    firecracker_dma_handshake_t hs;

    /*! Identification of the DMAC connected to this channel */
    firecracker_dma_t dmac;

    /*! Scatter / gather list for this channel */
    firecracker_dma_list_t sgl;

    /*! picoArray DMA channel number */
    unsigned channel;

    /*! Channel status (1 for active and 0 for waiting */
    int stateActive;

    /*! Physical address for the picoArray DMA channel */
    dma_addr_t pico_addr;

    /*! Call back function to execute when the interrupt is raised */
    int ( *handler )( void *cookie,
                      int errno );

    /*! User data to be supplied to the callback function */
    void *cookie;

    /*! Pointer to parent structure */
    struct pc202 *dev;
};

/*!
 * \brief Private representation of a PC202 device.
 *
 * This describes all private data required in the PC202 implementation of
 * this driver.
 *
 * \extends picoarray
 */
struct pc202
{
    /*! The picoArray base class. */
    struct picoarray            pa;

    /*! The physical address of the procif registers. */
    dma_addr_t                  reg_base_phys;

    /*! The length of the procif registers in bytes. */
    size_t                      reg_base_len;

    /*! The virtually mapped address of the procif registers. */
    void __iomem                *reg_base;

    /*! The physical address of the AHB2Pico registers. */
    dma_addr_t                  ahb_base_phys;

    /*! The length of the AHB2Pico registers in bytes. */
    size_t                      ahb_base_len;

    /*! The virtually mappped address of the AHB2Pico registers. */
    void __iomem                *ahb_base;

    /*! The physical address of the CCR register. */
    dma_addr_t                  ccr_base_phys;

    /*! The length of the CCR register in bytes. */
    size_t                      ccr_base_len;

    /*! The virtually mappped address of the CCR register. */
    void __iomem                *ccr_base;

    /*! The procif IRQ number. */
    unsigned                    procif_irq;

    /*! The AHB2Pico IRQ number. The AHB2Pico supports a number of interrupt
     * sources, but these are all raised through a single IRQ line. */
    unsigned                    ahb2pico_irq;

    /*! The handlers registered for the procif IRQ. */
    struct pc202_irq_handler    procif_irq_handlers;

    /*! The handlers registered for the AHB2Pico IRQ. */
    struct pc202_irq_handler    ahb2pico_irq_handlers;

    /*! The DMAC 1 IRQ number. */
    unsigned                    dma1_irq;

    /*! The DMAC2 IRQ number. */
    unsigned                    dma2_irq;

    /*! DMA channel data */
    struct pc202_dma_channel    dma_channel[PICO_NUM_DMA_CHANNELS];

    /*! DMA tasklet FIFO */
    struct pc202_dma_fifo       dma_fifo;

    /*! DMA tasklet structure */
    struct tasklet_struct       dma_tasklet;
};

/*!
 * Get the PC202 structure given a picoArray base class.
 *
 * @param pa The base class pointer.
 * @return Returns a pointer to the PC202 structure on success, NULL on
 * failure.
 */
static inline struct pc202 *
to_pc202( struct picoarray *pa )
{
    return pa ? container_of( pa, struct pc202, pa ) : NULL;
}

/*!
 * Write the value of an AHB2Pico register.
 *
 * @param dev The PC202 owning the AHB2Pico.
 * @param offset The offset of the register in bytes.
 * @param value The value of the register to write.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_ahb2pico_reg_write( struct pc202 *dev,
                          unsigned offset,
                          u32 value )
{
    PRINTD( COMPONENT_PC202, DBG_TRACE, "pA[%u] ahb2pico, %04x:=%08x",
            dev->pa.dev_num, offset, value );
    picoif_out32( value, dev->ahb_base + offset );
    return 0;
}

/*!
 * Read the value of an AHB2Pico register.
 *
 * @param dev The PC202 owning the AHB2Pico.
 * @param offset The offset of the register in bytes.
 * @param[out] value Pointer to the address to store the register value in.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_ahb2pico_reg_read( struct pc202 *dev,
                         unsigned offset,
                         u32 *value )
{
    *value = picoif_in32( dev->ahb_base + offset );
    return 0;
}

/*!
 * Read a number of 16 bit words from the PC202 procif.
 *
 * @param pa The device to read from.
 * @param caeid The CAEID of the AE to read from.
 * @param address The start address in the AE to begin reading from.
 * @param count The number of 16 bit words to read.
 * @param[out] data The buffer to store the data in.
 * @return Returns the number of words read on success, negative on failure.
 */
static int
pc202_config_read( struct picoarray *pa,
                   u16 caeid,
                   u16 address,
                   u16 count,
                   u16 *data )
{
    struct pc202 *dev = to_pc202( pa );

    PRINTD( COMPONENT_PC202, DBG_TRACE,
            "pa[%u]: config read %u words %04x@%04x", pa->dev_num,
            count, caeid, address );

    return procif_config_read( dev->reg_base, caeid, address, data, count );
}

/*!
 * Write a number of 16 bit words to the PC202 procif.
 *
 * @param pa The device to write to.
 * @param caeid The CAEID of the AE to write to.
 * @param address The start address in the AE to begin writing to.
 * @param count The number of 16 bit words to write.
 * @param[in] data The buffer to write from.
 * @return Returns the number of words written on success, negative on failure.
 */
static int
pc202_config_write( struct picoarray *pa,
                    u16 caeid,
                    u16 address,
                    u16 count,
                    u16 *data )
{
    struct pc202 *dev = to_pc202( pa );

    PRINTD( COMPONENT_PC202, DBG_TRACE,
            "pa[%u]: config write %u words %04x@%04x", pa->dev_num,
            count, caeid, address );

    return procif_config_write( dev->reg_base, caeid, address, data, count );
}

/*!
 * Sync the PC202 device.
 *
 * @param pa The device to sync.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_sync( struct picoarray *pa )
{
    u16 val;

    /* Read the current operation status register value. */
    int ret = pa->ops->config_read( pa, PC202_PROCIF_CAEID,
                                    PC202_PROCIF_OP_STATUS_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    /* If we are running, don't try and sync. */
    ret = 0;
    if ( val & PC202_PROCIF_OP_REQ_START )
        goto out;

    /* Read the current operation request register value. */
    ret = pa->ops->config_read( pa, PC202_PROCIF_CAEID,
                                    PC202_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    /* Set the sync request bit. */
    val |= PC202_PROCIF_OP_REQ_SYNC;
    ret = pa->ops->config_write( pa, PC202_PROCIF_CAEID,
                                 PC202_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    ret = 0;
out:
    return ret;
}

/*!
 * Start the PC202 device running.
 *
 * @param pa The device to start.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_start( struct picoarray *pa )
{
    u16 val;
    int ret = pa->ops->config_read( pa, PC202_PROCIF_CAEID,
                                    PC202_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;
    val |= PC202_PROCIF_OP_REQ_START;
    ret = pa->ops->config_write( pa, PC202_PROCIF_CAEID,
                                 PC202_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    ret = 0;
out:
    return ret;
}

/*!
 * Stop the PC202 device running.
 *
 * @param pa The device to stop.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_stop( struct picoarray *pa )
{
    u16 val;
    int ret = pa->ops->config_read( pa, PC202_PROCIF_CAEID,
                                    PC202_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;
    val &= ~PC202_PROCIF_OP_REQ_START;
    ret = pa->ops->config_write( pa, PC202_PROCIF_CAEID,
                                 PC202_PROCIF_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    ret = 0;
out:
    return ret;
}

/*!
 * Reset the PC202. This performs a soft reset that returns the picoArray to
 * be as close as possible to the hardware reset state without affecting the
 * ARM subsystem.
 *
 * @param pa The device being reset.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_reset( struct picoarray *pa )
{
    struct pc202 *dev = to_pc202( pa );
    unsigned long flags;
    u32 mask;

    PRINTD( COMPONENT_PC202, DBG_TRACE, "pa[%u]: reset", pa->dev_num );

    /* Disable all interrupts apart from the tick timer by masking them.  In
     * particular, we are disabling all the interrupts from the picoArray and
     * the DMA engines to prevent us from re-entering this driver.
     * Since the timer tick is enabled we will prevent the kernel from using
     * that to preempt the soft reset, just in case that lets another task
     * access the picoArray. */
    preempt_disable();
    
    spin_lock_irqsave( &pa->lock, flags );
    mask = ioread32(__io(IO_ADDRESS(PC20X_VIC_BASE + VIC_ENABLE_REG_OFFSET)));
    iowrite32(mask & (1<<IRQ_TIMER_0), __io(IO_ADDRESS(PC20X_VIC_BASE + VIC_ENABLE_REG_OFFSET)));
    spin_unlock_irqrestore( &pa->lock, flags );

    picoArraySoftReset( dev->reg_base, dev->ahb_base );
    
    /* Re-enable previously active interrupts. */
    spin_lock_irqsave( &pa->lock, flags );
    iowrite32(mask, __io(IO_ADDRESS(PC20X_VIC_BASE + VIC_ENABLE_REG_OFFSET)));
    spin_unlock_irqrestore( &pa->lock, flags );

    preempt_enable();
    
    return 0;
}

/*!
 * Read a GPR (general purpose register) in the PC202.
 *
 * @param pa The device being read from.
 * @param reg The register to read.
 * @param[out] value The address to store the register value in.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_register_read( struct picoarray *pa,
                     struct pico_resource *reg,
                     u32 *value )
{
    struct pc202 *dev = to_pc202( pa );
    int ret = -EINVAL;

    if ( !reg || PICO_RES_GPR != reg->type )
        goto out;

    if ( reg->value < PC202_GPR_PROCIF_0 ||
         reg->value > PC202_GPR_AHB2PICO_23 )
        goto out;

    if ( reg->value >= PC202_GPR_PROCIF_0 &&
         reg->value <= PC202_GPR_ITS )
    {
        procif_reg_read( dev->reg_base, reg->offset, value );

        ret = 0;
    }
    else
    {
        u32 status;
        pc202_ahb2pico_reg_read( dev, reg->offset + AHB2PICO_VP_STATUS_OFFSET,
                                 &status );
        ret = -EAGAIN;
        /* For non-blocking GPR's, we only check if they are enabled and
         * receive GPR's. If they are blocking GPRs then we need to check that
         * there is some data to read. */
        if ( !IS_PORT_ENABLED( status ) ||
             !IS_PORT_A_RECEIVE( status ) ||
             ( !IS_PORT_NONBLOCKING( status ) &&
               !IS_PORT_READY_FOR_DATA( status ) ) )
            goto out;
        pc202_ahb2pico_reg_read( dev, reg->offset + AHB2PICO_VP_DATA,
                                 value );
        ret = 0;
    }

out:
    return ret;
}

/*!
 * Write a GPR (general purpose register) in the PC202.
 *
 * @param pa The device being written to.
 * @param reg The register to write.
 * @param value The value to write to the register.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_register_write( struct picoarray *pa,
                      struct pico_resource *reg,
                      u32 value )
{
    struct pc202 *dev = to_pc202( pa );
    int ret = -EINVAL;

    if ( !reg || PICO_RES_GPR != reg->type )
    {
        goto out;
    }

    if ( reg->value < PC202_GPR_PROCIF_0 ||
         reg->value > PC202_GPR_AHB2PICO_23 )
    {
        goto out;
    }

    if ( reg->value >= PC202_GPR_PROCIF_0 &&
         reg->value <= PC202_GPR_ITS )
    {
        procif_reg_write( dev->reg_base, reg->offset, value );
        ret = 0;
    }
    else
    {
        u32 status;
        pc202_ahb2pico_reg_read( dev, reg->offset + AHB2PICO_VP_STATUS_OFFSET,
                                 &status );
        ret = -EAGAIN;
        /* For non-blocking GPR's, we only check if they are enabled and
         * send GPR's. If they are blocking GPRs then we need to check that
         * there is some space to accept the value. */
        if ( !IS_PORT_ENABLED( status ) ||
             IS_PORT_A_RECEIVE( status ) ||
             ( !IS_PORT_NONBLOCKING( status ) &&
               !IS_PORT_READY_FOR_DATA( status ) ) )
            goto out;
        pc202_ahb2pico_reg_write( dev, reg->offset + AHB2PICO_VP_DATA,
                                  value );
        ret = 0;
    }

out:
    return ret;
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
pc202_pa_load( struct picoarray *pa,
               u32 *data,
               struct scatterlist *sgl )
{
    struct pc202 *dev = to_pc202( pa );
    unsigned i;
    int ret = 0;

    PRINTD( COMPONENT_PC202, DBG_TRACE, "pa[%u]: config reg "
            "write %u words", pa->dev_num, sgl->length );

    for( i=0; i < sgl->length ; i++)
    {
        ret = procif_reg_write( dev->reg_base,
                           PROCIF_REG_CFG_WR_OFFSET, data[i] );
        /* Do not need to check return value, it is always true */
    }
 
    return i;

}

/*! The resources for a PC202 device. */
static struct pico_resource pc202_resources[] = {
    { .type = PICO_RES_DMA_CHANNEL, .value = PC202_DMA_PROCIF_0, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC202_DMA_PROCIF_1, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC202_DMA_PROCIF_2, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC202_DMA_PROCIF_3, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC202_DMA_AHB2PICO_0, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC202_DMA_AHB2PICO_1, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC202_DMA_AHB2PICO_2, },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC202_DMA_AHB2PICO_3, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_0, .offset = 0x0, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_1, .offset = 0x4, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_2, .offset = 0x8, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_3, .offset = 0xc, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_4, .offset = 0x10, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_5, .offset = 0x14, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_6, .offset = 0x18, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_7, .offset = 0x1c, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_8, .offset = 0x20, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_9, .offset = 0x24, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_10, .offset = 0x28, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_11, .offset = 0x2c, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_12, .offset = 0x30, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_13, .offset = 0x34, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_14, .offset = 0x38, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_15, .offset = 0x3c, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_16, .offset = 0x40, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_17, .offset = 0x44, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_18, .offset = 0x48, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_19, .offset = 0x4c, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_20, .offset = 0x54, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_21, .offset = 0x5c, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_22, .offset = 0x64, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_PROCIF_23, .offset = 0x6c, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_ITM, .offset = 0x70, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_ITS, .offset = 0x74, },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_0, .metadata = PC202_IRQ_AHB2PICO_0, .offset = 0x40 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_1, .metadata = PC202_IRQ_AHB2PICO_1, .offset = 0x50 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_2, .metadata = PC202_IRQ_AHB2PICO_2, .offset = 0x60 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_3, .metadata = PC202_IRQ_AHB2PICO_3, .offset = 0x70 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_4, .metadata = PC202_IRQ_AHB2PICO_4, .offset = 0x80 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_5, .metadata = PC202_IRQ_AHB2PICO_5, .offset = 0x90 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_6, .metadata = PC202_IRQ_AHB2PICO_6, .offset = 0xa0 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_7, .metadata = PC202_IRQ_AHB2PICO_7, .offset = 0xb0 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_8, .metadata = PC202_IRQ_AHB2PICO_8, .offset = 0xc0 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_9, .metadata = PC202_IRQ_AHB2PICO_9, .offset = 0xd0 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_10, .metadata = PC202_IRQ_AHB2PICO_10, .offset = 0xe0 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_11, .metadata = PC202_IRQ_AHB2PICO_11, .offset = 0xf0 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_12, .metadata = PC202_IRQ_AHB2PICO_12, .offset = 0x100 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_13, .metadata = PC202_IRQ_AHB2PICO_13, .offset = 0x110 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_14, .metadata = PC202_IRQ_AHB2PICO_14, .offset = 0x120 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_15, .metadata = PC202_IRQ_AHB2PICO_15, .offset = 0x130 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_16, .metadata = PC202_IRQ_AHB2PICO_16, .offset = 0x140 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_17, .metadata = PC202_IRQ_AHB2PICO_17, .offset = 0x150 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_18, .metadata = PC202_IRQ_AHB2PICO_18, .offset = 0x160 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_19, .metadata = PC202_IRQ_AHB2PICO_19, .offset = 0x170 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_20, .metadata = PC202_IRQ_AHB2PICO_20, .offset = 0x180 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_21, .metadata = PC202_IRQ_AHB2PICO_21, .offset = 0x190 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_22, .metadata = PC202_IRQ_AHB2PICO_22, .offset = 0x1a0 },
    { .type = PICO_RES_GPR, .value = PC202_GPR_AHB2PICO_23, .metadata = PC202_IRQ_AHB2PICO_23, .offset = 0x1b0 },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_0, .metadata = PC202_GPR_AHB2PICO_0, .offset = 4, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_1, .metadata = PC202_GPR_AHB2PICO_1, .offset = 5, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_2, .metadata = PC202_GPR_AHB2PICO_2, .offset = 6, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_3, .metadata = PC202_GPR_AHB2PICO_3, .offset = 7, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_4, .metadata = PC202_GPR_AHB2PICO_4, .offset = 8, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_5, .metadata = PC202_GPR_AHB2PICO_5, .offset = 9, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_6, .metadata = PC202_GPR_AHB2PICO_6, .offset = 10, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_7, .metadata = PC202_GPR_AHB2PICO_7, .offset = 11, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_8, .metadata = PC202_GPR_AHB2PICO_8, .offset = 12, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_9, .metadata = PC202_GPR_AHB2PICO_9, .offset = 13, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_10, .metadata = PC202_GPR_AHB2PICO_10, .offset = 14, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_11, .metadata = PC202_GPR_AHB2PICO_11, .offset = 15, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_12, .metadata = PC202_GPR_AHB2PICO_12, .offset = 16, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_13, .metadata = PC202_GPR_AHB2PICO_13, .offset = 17, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_14, .metadata = PC202_GPR_AHB2PICO_14, .offset = 18, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_15, .metadata = PC202_GPR_AHB2PICO_15, .offset = 19, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_16, .metadata = PC202_GPR_AHB2PICO_16, .offset = 20, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_17, .metadata = PC202_GPR_AHB2PICO_17, .offset = 21, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_18, .metadata = PC202_GPR_AHB2PICO_18, .offset = 22, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_19, .metadata = PC202_GPR_AHB2PICO_19, .offset = 23, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_20, .metadata = PC202_GPR_AHB2PICO_20, .offset = 24, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_21, .metadata = PC202_GPR_AHB2PICO_21, .offset = 25, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_22, .metadata = PC202_GPR_AHB2PICO_22, .offset = 26, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_AHB2PICO_23, .metadata = PC202_GPR_AHB2PICO_23, .offset = 27, },
    { .type = PICO_RES_IRQ, .value = PC202_IRQ_PROCIF },
    { .type = 0, .value = 0 }, /* Sentinel value. Do not remove and keep this
                                * at the end. */
};

/*! The offset in the AHB2Pico for GPR virtual ports. */
#define AHB2PICO_GPR_VP_START       ( 0x40 )

/*! The position of the interrupt enable bit in a virtual port config
 *  register. */
#define AHB2PICO_INT_EN             ( 1 << 0 )

/*!
 * Add an IRQ handler to the PC202 for a given interrupt source.
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
pc202_add_irq_handler( struct picoarray *pa,
                       struct pico_resource *irq,
                       int ( *callback )( struct pico_resource *irq,
                                          void *cookie ),
                       void *cookie )
{
    struct pc202_irq_handler *handler = NULL;
    struct pc202 *dev = to_pc202( pa );
    int ret = -ENOMEM;
    unsigned long flags;

    if ( irq->type != PICO_RES_IRQ || !irq->exclusive || !callback )
        return -EINVAL;

    spin_lock_irqsave( &pa->lock, flags );

    handler = kmalloc( sizeof( *handler ), GFP_KERNEL );
    if ( !handler )
        goto out;

    handler->callback   = callback;
    handler->cookie     = cookie;
    handler->irq        = irq;

    /* If this is an AHB2Pico IRQ, then lets enable the interrupt. */
    if ( PC202_IRQ_PROCIF != irq->value )
    {
        unsigned gpr_num = irq->value - PC202_IRQ_AHB2PICO_0;
        unsigned offset =
            AHB2PICO_GPR_VP_START + ( gpr_num * AHB2PICO_VP_SPACING );
        u32 val;

        /* Read the status register. */
        pc202_ahb2pico_reg_read( dev, offset + AHB2PICO_VP_STATUS_OFFSET,
                                 &val );
        /* If the port is non-blocking then fail as we will receive constant
         * interrupts. We also only support receive GPRs for interrupt
         * generation. */
        if ( !IS_PORT_A_RECEIVE( val ) || IS_PORT_NONBLOCKING( val ) )
        {
            PRINTD( COMPONENT_PC202, DBG_WARN,
                    "cannot enable IRQ: AHB2Pico VP is not"
                    " configured as blocking receive" );
            ret = -EIO;
            goto out;
        }

        pc202_ahb2pico_reg_read( dev, offset, &val );
        val |= AHB2PICO_INT_EN;
        pc202_ahb2pico_reg_write( dev, offset, val );
    }

    list_add( &handler->list, irq->value == PC202_IRQ_PROCIF ?
              &dev->procif_irq_handlers.list :
              &dev->ahb2pico_irq_handlers.list );

    ret = 0;
out:
    if ( ret && handler )
        kfree( handler );

    spin_unlock_irqrestore( &pa->lock, flags );
    return ret;
}

/*!
 * Remove an IRQ handler from a PC202 interrupt source.
 *
 * @param pa The PC202 to remove the handler from.
 * @param irq The IRQ to remove the handler for.
 */
static void
pc202_remove_irq_handler( struct picoarray *pa,
                          struct pico_resource *irq )
{
    struct list_head *pos;
    struct list_head *tmp;
    struct pc202 *dev = to_pc202( pa );
    int found = 0;
    struct pc202_irq_handler *handler;
    unsigned long flags;

    /* Protect against interrupts being raised whilst we remove the handler.
     */
    spin_lock_irqsave( &pa->lock, flags );

    /* If this is an AHB2Pico IRQ, then lets disable the interrupt. */
    if ( PC202_IRQ_PROCIF != irq->value )
    {
        unsigned gpr_num = irq->value - PC202_IRQ_AHB2PICO_0;
        unsigned offset =
            AHB2PICO_GPR_VP_START + ( gpr_num * AHB2PICO_VP_SPACING );
        u32 val;
        pc202_ahb2pico_reg_read( dev, offset, &val );
        val &= ~AHB2PICO_INT_EN;
        pc202_ahb2pico_reg_write( dev, offset, val );
    }

    list_for_each_safe( pos, tmp, &dev->procif_irq_handlers.list )
    {
        handler = container_of( pos, struct pc202_irq_handler, list );
        if ( handler->irq == irq )
        {
            list_del( pos );
            kfree( handler );
            found = 1;
            break;
        }
    }

    if ( found )
        goto out;

    list_for_each_safe( pos, tmp, &dev->ahb2pico_irq_handlers.list )
    {
        handler = container_of( pos, struct pc202_irq_handler, list );
        if ( handler->irq == irq )
        {
            list_del( pos );
            kfree( handler );
            found = 1;
            break;
        }
    }
out:
    spin_unlock_irqrestore( &pa->lock, flags );
}

/*!
 * PC202 AHB2Pico ISR. This ISR will be called when the AHB2Pico generates an
 * interrupt and checks all of the AHB2Pico interrupt sources for raised
 * interrupts and calls the appropriate handler if there is one registered. If
 * there is no interrupt handler for the IRQ source then that source is
 * disabled.
 *
 * @param irq The irq that has been raised.
 * @param dev The PC202 device that has raised the interrupt.
 * @return Returns IRQ_HANDLED on success.
 */
static irqreturn_t
pc202_ahb2pico_irq( int irq,
                    void *dev )
{
    struct pc202 *pc202dev = dev;
    struct list_head *pos;
    struct pc202_irq_handler *handler;
    int ret;
    u32 int_status;
    unsigned i;
    unsigned handled;

    pc202_ahb2pico_reg_read( pc202dev, AHB2PICO_INT_STATUS_OFFSET,
                             &int_status );
    for ( i = 0; i < AHB2PICO_NUM_VPS; ++i )
    {
        if ( int_status & ( 1 << i ) )
        {
            handled = 0;
            list_for_each( pos, &pc202dev->ahb2pico_irq_handlers.list )
            {
                handler = container_of( pos, struct pc202_irq_handler, list );
                if ( ( handler->irq->offset ) == i  && handler->callback )
                {
                    ret = handler->callback( handler->irq, handler->cookie );
                    if ( !ret )
                    {
                        handled = 1;
                        break;
                    }
                }
            }
            if ( !handled )
            {
                PRINTD( COMPONENT_PC202, DBG_WARN,
                        "no interrupt handler for AHB2Pico VP %u", i );
                /* Disable the interrupt generation for this GPR - there
                 * is no handler installed. The configuration registers
                 * are 0x10 bytes apart starting from 0x40. */
                pc202_ahb2pico_reg_write( pc202dev, 
                        ( i * AHB2PICO_VP_SPACING ) + AHB2PICO_GPR_VP_START,
                        0 );
            }
        }
    }

    return IRQ_HANDLED;
}

/*!
 * PC202 procif ISR. This ISR will be called when the procif generates an
 * interrupts and calls the appropriate handler if there is one registered. If
 * there is no interrupt handler then the ITM register is cleared to prevent
 * further interrupts from being raised.
 *
 * @param irq The irq that has been raised.
 * @param dev The PC202 device that has raised the interrupt.
 * @return Returns IRQ_HANDLED on success.
 */
static irqreturn_t
pc202_procif_irq( int irq,
                  void *dev )
{
    struct pc202 *pc202dev = dev;
    struct list_head *pos;
    struct pc202_irq_handler *handler;
    int ret;
    int handled = 0;

    list_for_each( pos, &pc202dev->procif_irq_handlers.list )
    {
        handler = container_of( pos, struct pc202_irq_handler, list );
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
        procif_reg_write( pc202dev->reg_base, PC202_PROCIF_ITM_OFFSET,
                          0 );

    return IRQ_HANDLED;
}

/*!
 * PC202 DMA tasklet. This function will execute the appropriate DMA handler
 * at a time chosen by the host.  The DMA software will not permit more that one 
 * transfer to be queued for each DMA chhanel. Thus the FIFO which is common to
 * all DMA channels need not be large (one for each DMA channel plus 1 spare entry).
 * As a tasklet cannot be called when it is already running, this function must 
 * iterated around the FIFO until it is empty. 
 *
 * @param fifoP An unsigned long conmtaining the address of the DMA FIFO
 */
static void
pc202_dma_do_tasklet(unsigned long fifoP)
{
    struct pc202_dma_fifo *dma_fifo = (struct pc202_dma_fifo *)fifoP;

    while( dma_fifo->readPtr != dma_fifo->writePtr )
    {
        PRINTD( COMPONENT_PC202, DBG_TRACE,
                "calling handle 0x%p error=%d (Read ptr=%d, write ptr=%d)",
                dma_fifo->fifo[dma_fifo->readPtr].dma->handler,
                 dma_fifo->fifo[dma_fifo->readPtr].errno,
                dma_fifo->readPtr, dma_fifo->writePtr );

        dma_fifo->fifo[dma_fifo->readPtr].dma->handler(
              dma_fifo->fifo[dma_fifo->readPtr].dma->cookie,
              dma_fifo->fifo[dma_fifo->readPtr].errno );
        dma_fifo->readPtr++;
        dma_fifo->readPtr %= DMA_FIFO_SIZE;
    }
}

/*!
 * \brief PC202 DMA channel mappings
 *
 * This structure defines the DMAC and DMA channel numbers for each of the
 * picoArray DMA channels. This structure is internal and specific to the
 * PC202.
 */
struct pc202_dma_mapping
{
    unsigned dmac;
    unsigned chan;
};

/*!
 * \brief Initialise the DMAC and DMA channel mappings
 *
 * @param dma Pointer to the global DMA structure for the PC202 device
 * @return Returns 0 on succes, negatice number on error.
 *
 */
__must_check static int
pc202_dma_init( struct pc202 *dev )
{
    unsigned i;
    firecracker_dma_t dmac[PICO_NUM_DMACS];
    struct pc202_dma_mapping dma_mapping[PICO_NUM_DMA_CHANNELS] = {
        { 0, 1 },
        { 1, 1 },
        { 0, 3 },
        { 1, 3 },
        { 0, 0 },
        { 1, 0 },
        { 0, 2 },
        { 1, 2 },
    };
   
    for(i = 0; i < PICO_NUM_DMACS ; i++)
    {
       dmac[i] = firecracker_dma_get_dmac_handle( i );
       if (dmac[i] == NULL)
            return -ENOMEM; 
    }

    /* Assign the DMAC number and channel to the picoArray DMA channel */
    for (i = 0; i < PICO_NUM_DMA_CHANNELS; i++)
    {
        dev->dma_channel[i].hs.interface = dma_mapping[i].chan;
        dev->dma_channel[i].dmac = dmac[ dma_mapping[i].dmac ];
        dev->dma_channel[i].hs.active_low = 0; /* PC20x DREQs are active high */
        dev->dma_channel[i].stateActive = 0;
        dev->dma_channel[i].xfr = NULL;
        dev->dma_channel[i].dev = dev;
        dev->dma_channel[i].channel = i;
    }

    tasklet_init( &dev->dma_tasklet, pc202_dma_do_tasklet,
                                                  (unsigned long)&dev->dma_fifo);

    return 0;
}

/*!
 * PC202 DMA ISR. This ISR will be called when the DMAC generates an
 * interrupt and calls the appropriate handler if there is one registered. 
 *
 * @param irq The irq that has been raised.
 * @param dev The PC202 device that has raised the interrupt.
 * @return Returns IRQ_HANDLED on success.
 */
static irqreturn_t
pc202_dma_irq( int irq,
               void *dev )
{
    firecracker_dma_t dmac = dev;
    struct pc202_dma_channel *dma;
    firecracker_dma_xfr_t xfr;
    dma_int_type_t int_type = INT_ALL;

    /* retrieve the transfer handle and our private data (dma) */
    xfr = firecracker_dma_int_get_xfr(dmac, &int_type, (void **)&dma);

    if (xfr == NULL)
    {
        PRINTD( COMPONENT_PC202, DBG_ERROR, "dma %d: got NULL xfr",
                  dma->channel);
        return IRQ_NONE;
    }

    /* Reset the interrupt we are servicing */
    firecracker_dma_clear_int(xfr, int_type);

    if (dma == NULL)
    {
        PRINTD( COMPONENT_PC202, DBG_ERROR, "invalid DMA interrupt, irq = %d",
              irq);
        return IRQ_NONE;
    }

    /* call the handler function */
    dma->stateActive = 0;
    if ( dma->handler )
    {
       unsigned tmp = dma->dev->dma_fifo.writePtr +1;

       tmp %= DMA_FIFO_SIZE;
       if ( tmp == dma->dev->dma_fifo.readPtr )
           PRINTD( COMPONENT_PC202, DBG_ERROR, "dma tasklet full" );
       else
       {
           dma->dev->dma_fifo.fifo[dma->dev->dma_fifo.writePtr].dma = dma;

           if ( INT_ERROR & int_type )
               dma->dev->dma_fifo.fifo[dma->dev->dma_fifo.writePtr].errno = -EIO;
           else
               dma->dev->dma_fifo.fifo[dma->dev->dma_fifo.writePtr].errno = 0;

           PRINTD( COMPONENT_PC202, DBG_TRACE,
                   "queueing DMA handler call error=%d (cookie=0x%p) "
                   "(read ptr=%d, write ptr=%d)",
                   dma->dev->dma_fifo.fifo[dma->dev->dma_fifo.writePtr].errno,
                   dma->dev->dma_fifo.fifo[dma->dev->dma_fifo.writePtr].dma->cookie,
                   dma->dev->dma_fifo.readPtr, dma->dev->dma_fifo.writePtr );

           dma->dev->dma_fifo.writePtr = tmp;
           tasklet_schedule( &dma->dev->dma_tasklet );
       }
    }
    else
        PRINTD( COMPONENT_PC202, DBG_ERROR,
              "no interrupt handler for DMA channel %u", dma->channel );

    return IRQ_HANDLED;
}

/*!
 * DMA a scatter gather list of memory from a kernel mapped scatterlist
 * into a picoArray DMA channel. After the DMA transfer has completed, the
 * callback function will be called with the cookie as the parameter. The
 * caller of this function is responsible for mapping and unmapping the
 * buffers to be transferred into a DMA capable region.
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to use as a destination.
 * @param sgl The scatter gather list of the source data.
 * @param nbytes The number of bytes in the scatter gather list.
 * @param callback The callback function to be called when the transfer
 * has completed. The parameter errno will be set to the status of the DMA
 * operation where 0 == success, negative == failure.
 * @param cookie The cookie to pass to the callback function.
 */
static int
pc202_dma_to_device( struct picoarray *pa,
                     struct pico_resource *dma_channel,
                     struct scatterlist *sgl,
                     size_t nbytes,
                     int ( *callback )( void *cookie,
                                        int errno ),
                     void *cookie )
{
    int ret = 0;
    struct pc202 *dev = to_pc202( pa );
    struct pc202_dma_channel *dma = &dev->dma_channel[dma_channel->value];
    dma_addr_t src = sg_dma_address( sgl );
    firecracker_dma_endpoint_t src_ep;
    firecracker_dma_endpoint_t dst_ep;

    if ( dma->stateActive )
      return -EAGAIN;

    PRINTD( COMPONENT_PC202, DBG_TRACE, "DMA transfer Chan %d, Bytes %d",
          dma_channel->value, nbytes );

    /* make a single SGL entry */
    firecracker_dma_list_clear( dma->sgl );

    src_ep.dma_addr = src;
    src_ep.ahb_master_select = AHB_MASTER1;
    src_ep.periph_not_mem = 0;
    src_ep.flow_controller = 0;
    src_ep.enable_sg = 0;
    src_ep.addr_inc = INCREMENT;
    src_ep.tr_width = TR_WIDTH32;
    src_ep.msize = MS_AUTO;
    src_ep.auto_reload = 0;

    dst_ep.dma_addr = dma->pico_addr;
    if ( dma_channel->value >= PC202_DMA_AHB2PICO_0 )
        dst_ep.ahb_master_select = AHB_MASTER2;
    else
        dst_ep.ahb_master_select = AHB_MASTER4;

    dst_ep.periph_not_mem = 1;
    dst_ep.flow_controller = 0;
    dst_ep.enable_sg = 0;
    dst_ep.addr_inc = NO_CHANGE;
    dst_ep.tr_width = TR_WIDTH32;
    SET_PICOARRAY_MSIZE(dst_ep);
    dst_ep.auto_reload = 0;

    if ( dma->xfr != NULL )
        firecracker_dma_release( dma->xfr );

    dma->xfr = firecracker_dma_setup_direct_xfr( dma->dmac,
                &src_ep, &dst_ep, NULL, &(dma->hs), nbytes,
                PROTCTL_1, dma );
    if ( dma->xfr == NULL )
    {
        PRINTD( COMPONENT_PC202, DBG_ERROR, "error setting up list xfr "
               "for channel %d", dma_channel->value );
        return -EINVAL;
    }

    firecracker_dma_enable_int( dma->xfr, INT_ERROR | INT_TRANSFER );

    dma->handler = callback;
    dma->cookie = cookie;
    dma->stateActive = 1;
    ret = firecracker_dma_start( dma->xfr );
    if ( ret )
    {
        PRINTD( COMPONENT_PC202, DBG_ERROR, "error starting xfr, "
              "ret=%d", -ret );
        return -ret;
    }

    return ret;
}

/*!
 * DMA a scatter gather list of memory from a picoArray DMA channel. After
 * the DMA transfer has completed, the callback function will be called
 * with the cookie as the parameter. The caller of this function is
 * responsible for mapping and unmapping the buffers to be transferred
 * into a DMA capable region.
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to use as a source.
 * @param sgl The scatter gather list of the destination buffer.
 * @param nbytes The number of bytes to transfer.
 * @param callback The callback function to be called when the transfer
 * has completed. The parameter errno will be set to the status of the DMA
 * operation where 0 == success, negative == failure.
 * @param cookie The cookie to pass to the callback function.
 * @return 0 for success and non-zeo for error
 */
static int
pc202_dma_from_device( struct picoarray *pa,
                       struct pico_resource *dma_channel,
                       struct scatterlist *sgl,
                       size_t nbytes,
                       int ( *callback )( void *cookie,
                                          int errno ),
                       void *cookie )
{
    int ret = 0;
    struct pc202 *dev = to_pc202( pa );
    struct pc202_dma_channel *dma = &dev->dma_channel[dma_channel->value];
    dma_addr_t src = sg_dma_address( sgl );
    firecracker_dma_endpoint_t src_ep;
    firecracker_dma_endpoint_t dst_ep;

    if ( dma->stateActive )
        return -EAGAIN;

    PRINTD( COMPONENT_PC202, DBG_TRACE, "chan %d, bytes %u",
               dma_channel->value, nbytes );

    /* make a single SGL entry */
    firecracker_dma_list_clear(dma->sgl);

    src_ep.dma_addr = dma->pico_addr;
    if ( dma_channel->value >= PC202_DMA_AHB2PICO_0 )
        src_ep.ahb_master_select = AHB_MASTER2;
    else
        src_ep.ahb_master_select = AHB_MASTER4;
    src_ep.periph_not_mem = 1;
    src_ep.flow_controller = 0;
    src_ep.enable_sg = 0;
    src_ep.addr_inc = NO_CHANGE;
    src_ep.tr_width = TR_WIDTH32;
    SET_PICOARRAY_MSIZE(src_ep);
    src_ep.auto_reload = 0;

    dst_ep.dma_addr = src;
    dst_ep.ahb_master_select = AHB_MASTER1;
    dst_ep.periph_not_mem = 0;
    dst_ep.flow_controller = 0;
    dst_ep.enable_sg = 0;
    dst_ep.addr_inc = INCREMENT;
    dst_ep.tr_width = TR_WIDTH32;
    dst_ep.msize = MS_AUTO;
    dst_ep.auto_reload = 0;

    if ( dma->xfr != NULL )
        firecracker_dma_release( dma->xfr );

    dma->xfr = firecracker_dma_setup_direct_xfr( dma->dmac,
                &src_ep, &dst_ep, &(dma->hs), NULL, nbytes,
                PROTCTL_1, dma);
    if ( dma->xfr == NULL )
    {
        PRINTD( COMPONENT_PC202, DBG_ERROR, "error setting up list xfr");
        return -EINVAL;
    }

    firecracker_dma_enable_int( dma->xfr, INT_ERROR | INT_TRANSFER );

    dma->stateActive = 1;
    dma->handler = callback;
    dma->cookie = cookie;
    ret = firecracker_dma_start( dma->xfr );
    if ( ret )
    {
        PRINTD( COMPONENT_PC202, DBG_ERROR,"error starting xfr, ret=%d",
             -ret );
        return -ret;
    }

    return ret;
}

/*!
 * Open and enable the specified DMA channel
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to open
 * @param is_downlink 1 if the channel is a downlink, 0 for an uplink
 * @return 0 on success, a negative number on failure
 */
__must_check static int
pc202_dma_open( struct picoarray *pa,
                struct pico_resource *dma_channel,
                int is_downlink )
{
    struct pc202 *pc202dev = to_pc202( pa );
    struct pc202_dma_channel *dma = 
                                &pc202dev->dma_channel[dma_channel->value];

    if ( dma_channel->value >=  PC202_DMA_AHB2PICO_0 )
    {
        PRINTD( COMPONENT_PC202, DBG_TRACE,
              "AHB2PICO pa_dma %d  => cpu_dma %d",
              ( dma_channel->value-PC202_DMA_AHB2PICO_0 ), 
              dma_channel->value);

        dma->pico_addr = (dma_addr_t)pc202dev->ahb_base_phys 
            +((dma_channel->value-PC202_DMA_AHB2PICO_0)*AHB2PICO_VP_SPACING)
            +AHB2PICO_VP_DATA_OFFSET;

        /* Set the DMAC Enable bit (bit1) in the DMA config register */
        (void)pc202_ahb2pico_reg_write( pc202dev, 
                ((dma_channel->value-PC202_DMA_AHB2PICO_0)*AHB2PICO_VP_SPACING)
                +AHB2PICO_VP_CONFIG_OFFSET, (0x02 + (PC202_DMA_BURST_SIZE << 2)));
    }
    else
    {
        /* ProcIF interface - Set the channels phyiscal address and use the
           DMA shadow register. Register offset is calculated by counting
           the register order as follows:
              GPR0 -> GPR19, DMA0, GPR20, DMA1, GPR21, DMA2, GPR22, DMA3
           Each register occupies a long word, therefore the offsets to apply
           are:
              DMA Channel 0 -> 80
              DMA Channel 1 -> 88
              DMA Channel 2 -> 96
              DMA Channel 3 -> 108
           channel->value lies in the range PC202_DMA_PROCIF_0 to
               PC202_DMA_PROCIF_3 as an enumaration list */
        dma->pico_addr = (dma_addr_t)pc202dev->reg_base_phys +
          ( 2 * (10+(dma_channel->value-PC202_DMA_PROCIF_0)) * sizeof(uint32_t));
    }

    dma->sgl = firecracker_dma_list_create( dma->dmac, 1 );
    if ( dma->sgl == NULL )
    {
        PRINTD( COMPONENT_PC202, DBG_ERROR, "dma %d: cannot allocate "
           "scatter gather list", dma_channel->value);
        return -EINVAL;
    }

    /* Initialise parameters that need to be reset */
    dma->xfr = NULL;
    dma->stateActive=0;
    dma->handler=NULL;
    dma->cookie=NULL;

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
pc202_dma_close( struct picoarray *pa,
                 struct pico_resource *dma_channel )
{
    int ret = 0;
    struct pc202 *pc202dev = to_pc202( pa );
    struct pc202_dma_channel *dma =
                                &pc202dev->dma_channel[dma_channel->value];

    if ( dma->xfr )
    {
        if ( dma->stateActive )
        {
            ret = firecracker_dma_abort( dma->xfr );
            if ( ret )
                PRINTD( COMPONENT_PC202, DBG_WARN, "DMA abort failed, "
                "return code=%d", ret );

            PRINTD( COMPONENT_PC202, DBG_WARN, "dma %d: stopped while dma "
                "in progress", dma_channel->value);
        }

        /* Disable all interrupts for this channel */
        firecracker_dma_disable_int( dma->xfr, INT_ALL );

        /* Reset all interrupts for this channel */
        firecracker_dma_clear_int(dma->xfr, INT_ALL );

        firecracker_dma_release( dma->xfr );
    }

    if ( dma->sgl )
    {
        ret = firecracker_dma_list_destroy( dma->sgl );
        if (ret)
            PRINTD( COMPONENT_PC202, DBG_WARN, "DMA list_destroy failed "
                "return code = %d", ret );
    }

    dma->stateActive = 0;
    return 0;
}

/*!
 * Get the device type of a PC202.
 *
 * @param pa The device to query.
 * @return Always returns PICOARRAY_PC202.
 */
static enum picoarray_device_type
pc202_get_device_type( struct picoarray *pa )
{
    return PICOARRAY_PC202;
}

/*!
 * Get the ITM, ITS and procIF IRQ resources.
 * This function will return the pointers to the resources requested on success or NULL
 * if the resource is already allocated.
 *
 * @param pa The device to take to DMA the data.
 * @param its The ITS resource or NULL on failure
 * @param itm The ITM resource or NULL on failure
 * @param procif_irq The IRQ resource or NULL on failure
 * 
 * @return 0 on success, non-zero on failure
 */
static int
pc202_get_procif_resource( struct picoarray *pa,
                           struct pico_resource **its,
                           struct pico_resource **itm,
                           struct pico_resource **procif_irq )
{
    *its = generic_get_resource( pa, PICO_RES_GPR, PC202_GPR_ITS, 1 );
    *itm = generic_get_resource( pa, PICO_RES_GPR, PC202_GPR_ITM, 1 );
    *procif_irq = generic_get_resource( pa, PICO_RES_IRQ, PC202_IRQ_PROCIF, 1 );

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
pc202_put_procif_resource( struct picoarray *pa,
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

/*! Operations for the PC202 devices. */
static struct picoarray_ops pc202_ops = {
    .sync                = pc202_sync,
    .start               = pc202_start,
    .stop                = pc202_stop,
    .get_device_type     = pc202_get_device_type,
    .config_read         = pc202_config_read,
    .config_write        = pc202_config_write,
    .register_read       = pc202_register_read,
    .register_write      = pc202_register_write,
    .reset               = pc202_reset,
    .get_resource        = generic_get_resource,
    .get_procif_resource = pc202_get_procif_resource,
    .put_resource        = generic_put_resource,
    .put_procif_resource = pc202_put_procif_resource,
    .destructor          = pc202_destroy,
    .add_irq_handler     = pc202_add_irq_handler,
    .remove_irq_handler  = pc202_remove_irq_handler,
    .dma_to_device       = pc202_dma_to_device,
    .dma_from_device     = pc202_dma_from_device,
    .dma_open            = pc202_dma_open,
    .dma_close           = pc202_dma_close,
    .pa_load             = pc202_pa_load,
};

/*!
 * Probe method for the PC202 platform driver. This function creates a new
 * PC202 instance and is responsible for allocating all of the resources
 * required.
 *
 * @param pdev The platform device that has been probed.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_probe( struct platform_device *pdev )
{
    struct resource *res;
    int ret;
    uint32_t chip_control;
    struct pc202 *newdev = kmalloc( sizeof( *newdev ), GFP_KERNEL );

    if ( !newdev )
        return -ENOMEM;

    ret = -ENOMEM;
    newdev->pa.resources = kmalloc( sizeof( pc202_resources ), GFP_KERNEL );
    if ( !newdev->pa.resources )
        goto out;
    memcpy( newdev->pa.resources, pc202_resources, sizeof( pc202_resources ) );

    newdev->pa.dev_num = pdev->id;
    newdev->pa.ops = &pc202_ops;
    newdev->pa.features = PICOARRAY_HAS_DMA | PICOARRAY_HAS_DMA_LOAD;
    newdev->pa.max_dma_sz = PICOARRAY_MAX_TRANSFER;
    spin_lock_init( &newdev->pa.lock );

    ret = -EINVAL;
    /* Get the procif IRQ. */
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "procif_irq" );
    if ( !res )
        goto out;
    newdev->procif_irq = res->start;

    ret = -EINVAL;
    /* Get the DMA1 IRQ. */
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "dma1_irq" );
    if ( !res )
        goto out;
    newdev->dma1_irq = res->start;

    ret = -EINVAL;
    /* Get the DMA2 IRQ. */
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "dma2_irq" );
    if ( !res )
        goto out;
    newdev->dma2_irq = res->start;

    /* Get the IRQ for AHB2Pico GPRs. */
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "gpr_irq" );
    if ( !res )
        goto out;
    newdev->ahb2pico_irq = res->start;

    /* Get the register base address for the procif. */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM, "procif" );
    if ( !res )
        goto out;
    newdev->reg_base_phys = res->start;
    newdev->reg_base_len = ( res->end - res->start ) + 1;

    /* Get the register base address for the AHB2Pico. */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM,
                                        "ahb2pico_axi2pico" );
    if ( !res )
        goto out;
    newdev->ahb_base_phys = res->start;
    newdev->ahb_base_len = ( res->end - res->start ) + 1;

    /* Get the Chip Control Register */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM, "ccr_base");
    if ( !res )
        goto out;
    newdev->ccr_base_phys = res->start;
    newdev->ccr_base_len = ( res->end - res->start ) + 1;

    /* Map the resources. */
    newdev->reg_base = request_and_map( "procif", newdev->reg_base_phys,
                                        newdev->reg_base_len );
    if ( !newdev->reg_base )
        goto out;

    newdev->ahb_base = request_and_map( "ahb2pico", newdev->ahb_base_phys,
                                        newdev->ahb_base_len );
    if ( !newdev->ahb_base )
        goto ahb_map_failed;

    newdev->ccr_base = request_and_map( "ccr_base", newdev->ccr_base_phys,
                                        newdev->ccr_base_len );
    if ( !newdev->ccr_base )
        goto ccr_map_failed;

    /* Initialise the DMAC structures */
    ret = pc202_dma_init( newdev );
    if ( ret )
        goto dma_init_failed;

    /* Clear CCR bits 3 & 4 to set DREQ mux for procIf instead of EBI */
    chip_control = picoif_in32(newdev->ccr_base);
    chip_control &= ~(0x00000018);
    picoif_out32(chip_control,newdev->ccr_base);

    ret = request_irq( newdev->procif_irq, pc202_procif_irq, IRQF_DISABLED,
                       pdev->name, newdev );
    if ( ret )
        goto procif_irq_failed;

    ret = request_irq( newdev->ahb2pico_irq, pc202_ahb2pico_irq, IRQF_DISABLED,
                       pdev->name, newdev );
    if ( ret )
        goto ahb2pico_irq_failed;

    ret = request_irq( newdev->dma1_irq, pc202_dma_irq, IRQF_DISABLED,
                       pdev->name, firecracker_dma_get_dmac_handle( 0 ) );
    if ( ret )
        goto dma1_irq_failed;

    ret = request_irq( newdev->dma2_irq, pc202_dma_irq, IRQF_DISABLED,
                       pdev->name, firecracker_dma_get_dmac_handle( 1 ) );
    if ( ret )
        goto dma2_irq_failed;

    /* Initialise the interrupt handler lists. */
    INIT_LIST_HEAD( &newdev->procif_irq_handlers.list );
    INIT_LIST_HEAD( &newdev->ahb2pico_irq_handlers.list );

    newdev->dma_fifo.readPtr = newdev->dma_fifo.writePtr = 0;

    ret = picoif_register_dev( &newdev->pa );
    goto out;

dma2_irq_failed:
    free_irq( newdev->dma1_irq, firecracker_dma_get_dmac_handle( 0 ) );

dma1_irq_failed:
    free_irq( newdev->ahb2pico_irq, newdev );

ahb2pico_irq_failed:
    free_irq( newdev->procif_irq, newdev );

dma_init_failed:
procif_irq_failed:
    unmap_and_release( newdev->ccr_base_phys, newdev->ccr_base_len,
                       newdev->ccr_base );

ccr_map_failed:
    unmap_and_release( newdev->ahb_base_phys, newdev->ahb_base_len,
                       newdev->ahb_base );

ahb_map_failed:
    unmap_and_release( newdev->reg_base_phys, newdev->reg_base_len,
                       newdev->reg_base );
out:
    if ( ret && newdev->pa.resources )
        kfree( newdev->pa.resources );

    if ( ret )
        kfree( newdev );
    else
        ++num_pc202s;

    return ret;
}

/*!
 * Remove method for the PC202 platform driver. This method is called when the
 * platform driver is removed and must release all resources the driver has
 * been using.
 *
 * @param pdev The platform device being remove.
 * @return Returns zero on success, negative on failure.
 */
static int
pc202_remove( struct platform_device *pdev )
{
    struct picoarray *pa = picoif_get_device( pdev->id );
    struct pc202 *pc202dev = to_pc202( pa );
    int ret = 0;

    free_irq( pc202dev->dma1_irq, firecracker_dma_get_dmac_handle( 0 ) );
    free_irq( pc202dev->dma2_irq, firecracker_dma_get_dmac_handle( 1 ) );
    free_irq( pc202dev->procif_irq, pc202dev );
    free_irq( pc202dev->ahb2pico_irq, pc202dev );
    unmap_and_release( pc202dev->ahb_base_phys, pc202dev->ahb_base_len,
                       pc202dev->ahb_base );
    unmap_and_release( pc202dev->reg_base_phys, pc202dev->reg_base_len,
                       pc202dev->reg_base );
    unmap_and_release( pc202dev->ccr_base_phys, pc202dev->ccr_base_len,
                       pc202dev->ccr_base );

    kfree( pc202dev->pa.resources );
    kfree( pc202dev );

    return ret;
}

/*! The PC202 platform driver.
 *  \todo Change the name to PC202 specific rather than generic picoArray.
 */
static struct platform_driver pc202_driver = {
    .probe      = pc202_probe,
    .remove     = pc202_remove,
    .driver     = {
        .name   = "picoArray",
    },
};

int
pc202_init( void )
{
    return platform_driver_register( &pc202_driver );
}

/*!
 * Destructor to be called when a PC202 is removed from picoif. This
 * function must decrement the number of PC202s registered, and when this
 * reaches zero, remove the platform driver.
 *
 * @param pa The device being removed.
 */
static void
pc202_destroy( struct picoarray *pa )
{
    PRINTD( COMPONENT_PC202, DBG_TRACE, "pA[%u]: destructor called",
            pa->dev_num );

    /* If we have no more pc202s, then remove the driver. */
    if ( 0 == --num_pc202s )
        platform_driver_unregister( &pc202_driver );
}
