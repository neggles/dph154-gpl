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
 * \file pc302.c
 * \brief PC302 device implementation.
 *
 * This file implements the PC302 support of picoIf. All implementation in
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
#include <linux/jiffies.h>
#include <asm/io.h>
#include <mach/pc302/axi2cfg.h>
#include <mach/pc302/pc302.h>
#include <mach/pc302_dmac.h>

#include <linux/picochip/devices/pc302.h>

#include "picoarray.h"
#include "resource.h"
#include "picoif_internal.h"
#include "axi2cfg.h"
#include "debug.h"
#include "soft_reset.h"
#include "utilities_internal.h"
#include "axi2cfg.h"

/*! The address of the AXI2Pico interrupt status register. This is an offset
 * from the AXI2Pico base address. */
#define AXI2PICO_INT_STATUS_OFFSET  ( 0x0200 )

/*! The number of virtual ports in the AXI2Pico. */
#define AXI2PICO_NUM_VPS           ( 32 )

/*! The spacing of virtual ports in the AXI2Pico. */
#define AXI2PICO_VP_SPACING         ( 0x10 )

/*! The offset of a virtual port config register from the virtual port
 *  register base. */
#define AXI2PICO_VP_CONFIG_OFFSET   ( 0x00 )

/*! The offset of a virtual port status register from the virtual port
 *  register base. */
#define AXI2PICO_VP_STATUS_OFFSET   ( 0x04 )

/*! The offset of the data register for a virtual port in the AXI2Pico. */
#define AXI2PICO_VP_DATA_OFFSET     ( 0x08 )

/*! The CAEID of the procif. */
#define PC302_AXI2CFG_CAEID          ( 0x48 )

/*! The CAEID of the AXI2Pico. */
#define PC302_AXI2PICO_CAEID	     ( 0xa8 )

/*! The CAEID of the Frac-N. */
#define PC302_FRACN_CAEID            ( 0x0578 )

/*! The offset in the procif for the operation request register. */
#define PC302_AXI2CFG_OP_REQ_OFFSET  ( 0x4018 )
/*! The offset in the procif for the operation status register. */
#define PC302_PROCIF_OP_STATUS_OFFSET  ( 0x401c )
/*! The offset of the start request bit in the operation request register. */
#define PC302_AXI2CFG_OP_REQ_START   ( 1 << 1 )

/*! The offset of the start bit in the operation status register */
#define PC302_AXI2CFG_OP_STATUS_RUNNING  ( 1 << 1 )

/*! The position of the interrupt enable bit in a virtual port config
 *  register. */
#define AXI2PICO_INT_EN             ( 1 << 0 )

/*! The offset in an AE for the sleep register. */
#define SLEEP_REG_OFFSET	    ( 0xA060 )

/*! Timeout for DMA load of picoArray */
#define TIMEOUT_10MS                ( HZ / 100 )

/*! DMA burst size in words */
#define PC302_DMA_BURST_SIZE        ( 1 )

/*! The maximum number of transactions per transfer */
#define DMAH_CHX_MAX_BLK_SIZE       ( 4095 )

/*! Determine the msize and max transfer size that is permitted */
#if ((PC302_DMA_BURST_SIZE == 1) || (PC302_DMA_BURST_SIZE == 2))
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC302_DMA_MS_1_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 4)
#elif (PC302_DMA_BURST_SIZE == 4)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC302_DMA_MS_4_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 8)
#elif (PC302_DMA_BURST_SIZE == 8)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC302_DMA_MS_8_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 16)
#elif (PC302_DMA_BURST_SIZE == 16)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC302_DMA_MS_16_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 32)
#elif (PC302_DMA_BURST_SIZE == 32)
    #define SET_PICOARRAY_MSIZE(__e) ( __e.msize = PC302_DMA_MS_32_TRW)
    #define PICOARRAY_MAX_TRANSFER ((DMAH_CHX_MAX_BLK_SIZE/4) * 64)
#else
    #error
#endif

/*!
 * Determine if a GPR port is enabled
 */
#define IS_PORT_ENABLED(status_reg) (status_reg & 0x1)

/*!
 *  Determine if a GPR port is configured as a receive port
 */
#define IS_PORT_A_RECEIVE(status_reg) (status_reg & 0x2)

/*!
 *  Determine if a GPR port is configured as a non blocking port
 */
#define IS_PORT_NONBLOCKING(status_reg) (status_reg & 0x4)

/*!
 *  Determine if a GPR port is ready to give or receive data
 */
#define IS_PORT_READY_FOR_DATA(status_reg) (status_reg & 0x8)

/*! The number of registered PC302s in the system. Until this reaches zero, we
 * can't unregister the platform driver. */
static unsigned num_pc302s;

static void
pc302_destroy( struct picoarray *pa );

/*!
 * \brief PC302 IRQ handler.
 *
 * This structure defines an IRQ handler for PC302 devices and is an internal
 * structure that could be reused for other device types.
 */
struct pc302_irq_handler
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
 * \brief PC302 DMA handler
 *
 * This structure defines a DMA handler for PC302 devices and is an internal
 * strucutre that might be reused for other device types that uses the Synopsys
 * DMAC. There will be one instance of this structure per DMA channel in the
 * system.
 */
struct pc302_dma_channel
{
    /*! Structure holding the current transfer details */
    pc302_dma_xfr_t xfr;

    /*! Definition of the hardware handshaking interface set for this channel */
    pc302_dma_handshake_t hs;

    /*! Identification of the DMAC connected to this channel */
    pc302_dma_t dmac;

    /*! Scatter / gather list for this channel */
    pc302_dma_list_t sgl;

    /*! DMA source master */
    pc302_master_t srcMaster;

    /*! DMA destination master */
    pc302_master_t dstMaster;

    /*! picoArray DMA channel number */
    unsigned channel;

    /*! Channel status (1 for active and 0 for waiting) */
    int stateActive;

    /*! Physical address for the picoArray DMA channel */
    dma_addr_t pico_addr;

    /*! The callback function to call when the DMA transfer is completed. */
    int ( *callback )( void *cookie,
                       int errno );

    /*! User data */
    void *cookie;
};

/*!
 * \brief Private representation of a PC302 device.
 *
 * This describes all private data required in the PC302 implementation of
 * this driver.
 *
 * \extends picoarray
 */
struct pc302
{
    /*! The picoArray base class. */
    struct picoarray            pa;

    /*! The physical address of the lower axi2cfg registers. */
    dma_addr_t                  axi2cfg1_base_phys;

    /*! The length of the lowe axi2cfg registers in bytes. */
    size_t                      axi2cfg1_base_len;

    /*! The virtually mapped address of the lower axi2cfg registers. */
    void __iomem                *axi2cfg1_base;

    /*! The physical address of the upper axi2cfg registers. */
    dma_addr_t                  axi2cfg2_base_phys;

    /*! The length of the upper axi2cfg registers in bytes. */
    size_t                      axi2cfg2_base_len;

    /*! The virtually mapped address of the upper axi2cfg registers. */
    void __iomem                *axi2cfg2_base;

    /*! The physical address of the AXI2Pico registers. */
    dma_addr_t                  axi2pico_base_phys;

    /*! The length of the AXI2Pico registers in bytes. */
    size_t                      axi2pico_base_len;

    /*! The virtually mappped address of the AXI2Pico registers. */
    void __iomem                *axi2pico_base;

    /*! The AXI2Pico IRQ number. The AXI2Pico supports a number of interrupt
     * sources, but these are all raised through a single IRQ line. */
    unsigned                    gpr_irq;

    /*! The handlers registered for the AXI2Pico IRQ. */
    struct pc302_irq_handler    axi2pico_irq_handlers;

    /*! DMA channel data */
    struct pc302_dma_channel    dma_channel[PICO_NUM_DMA_CHANNELS];
};

/*!
 * Get the PC302 structure given a picoArray base class.
 *
 * @param pa The base class pointer.
 * @return Returns a pointer to the PC302 structure on success, NULL on
 * failure.
 */
static inline struct pc302 *
to_pc302( struct picoarray *pa )
{
    return pa ? container_of( pa, struct pc302, pa ) : NULL;
}

/*!
 * Write the value of an AXI2Pico register.
 *
 * @param dev The PC302 owning the AXI2Pico.
 * @param offset The offset of the register in bytes.
 * @param value The value of the register to write.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302_axi2pico_reg_write( struct pc302 *dev,
                          unsigned offset,
                          u32 value )
{
    PRINTD( COMPONENT_PC302, DBG_TRACE, "pA[%u] axi2pico, %04x:=%08x",
            dev->pa.dev_num, offset, value );
    picoif_out32( value, dev->axi2pico_base + offset );
    return 0;
}

/*!
 * Read the value of an AXI2Pico register.
 *
 * @param dev The PC302 owning the AXI2Pico.
 * @param offset The offset of the register in bytes.
 * @param[out] value Pointer to the address to store the register value in.
 */
static void
pc302_axi2pico_reg_read( struct pc302 *dev,
                         unsigned offset,
                         u32 *value )
{
    *value = picoif_in32( dev->axi2pico_base + offset );
}

/*!
 * Read a number of 16 bit words from the PC302 axi2cfg.
 *
 * @param pa The device to read from.
 * @param caeid The CAEID of the AE to read from.
 * @param address The start address in the AE to begin reading from.
 * @param count The number of 16 bit words to read.
 * @param[out] data The buffer to store the data in.
 * @return Returns the number of words read on success, negative on failure.
 */
static int
pc302_config_read( struct picoarray *pa,
                   u16 caeid,
                   u16 address,
                   u16 count,
                   u16 *data )
{
    struct pc302 *dev = to_pc302( pa );

    PRINTD( COMPONENT_PC302, DBG_TRACE,
            "pa[%u]: config read %u words %04x@%04x", pa->dev_num,
            count, caeid, address );

    return axi2cfg_config_read( dev->axi2cfg2_base, caeid, address, data, count );
}

/*!
 * Write a number of 16 bit words to the PC302 axi2cfg.
 *
 * @param pa The device to write to.
 * @param caeid The CAEID of the AE to write to.
 * @param address The start address in the AE to begin writing to.
 * @param count The number of 16 bit words to write.
 * @param[in] data The buffer to write from.
 * @return Returns the number of words written on success, negative on failure.
 */
static int
pc302_config_write( struct picoarray *pa,
                    u16 caeid,
                    u16 address,
                    u16 count,
                    u16 *data )
{
    struct pc302 *dev = to_pc302( pa );

    PRINTD( COMPONENT_PC302, DBG_TRACE,
            "pa[%u]: config write %u words %04x@%04x", pa->dev_num,
            count, caeid, address );

    return axi2cfg_config_write( dev->axi2cfg2_base, caeid, address, data, count );
}

/*!
 * Sync the PC302 device. PC302 requires no synchronisation as it is a single
 * device so do nothing.
 *
 * @param pa The device to sync.
 * @return Always returns zero.
 */
static int
pc302_sync( struct picoarray *pa )
{
    return 0;
}

static int
pc3xx_is_running( struct picoarray *pa )
{
    u16 val;

    if ( 1 != pa->ops->config_read( pa, PC302_AXI2CFG_CAEID,
                                   PC302_AXI2CFG_OP_REQ_OFFSET, 1, &val ) )
        return 0;

    return val & ( PC302_AXI2CFG_OP_STATUS_RUNNING );
}

/*!
 * Start the PC302 device running.
 *
 * @param pa The device to start.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302_start( struct picoarray *pa )
{
    u16 val;
    int ret = 0;

    if ( pc3xx_is_running( pa ) )
    {
        PRINTD( COMPONENT_PC302, DBG_WARN, "system already running");
        goto out;
    }

    val = PC302_AXI2CFG_OP_REQ_START;
    ret = pa->ops->config_write( pa, PC302_AXI2CFG_CAEID,
                                 PC302_AXI2CFG_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    ret = 0;
out:
    return ret;
}

/*!
 * Stop the PC302 device running.
 *
 * @param pa The device to stop.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302_stop( struct picoarray *pa )
{
    u16 val;
    int ret = 0;

    if ( !pc3xx_is_running( pa ) )
    {
        PRINTD( COMPONENT_PC302, DBG_WARN, "system is not running" );
        goto out;
    }

    val = 0;
    ret = pa->ops->config_write( pa, PC302_AXI2CFG_CAEID,
                                 PC302_AXI2CFG_OP_REQ_OFFSET, 1, &val );
    if ( 1 != ret )
        goto out;

    ret = 0;
out:
    return ret;
}

/*! The CAEID of the MemIf. */
#define PC302_MEMIF_CAEID           ( 0x88 )
/*! The address of the FIFO status lower register. */
#define PC302_MEMIF_FIFO_STAT_LOW   ( 0x0080 )

/*! The CAEID of the PAI */
#define PC302_PAI_CAEID             ( 0x0578 )
/*! The address of the pai_io_ctrl register. */
#define PC302_PAI_IO_CTRL_REG       ( 0x0009 )

/*!
 * Reset the PC302. This performs a soft reset that returns the picoArray to
 * be as close as possible to the hardware reset state without affecting the
 * ARM subsystem.
 *
 * @param pa The device being reset.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302_reset( struct picoarray *pa )
{
    int ret;
    unsigned long flags;
    u16 val;
    u16 io_ctrl_value;

    u16 buf_vals[ 0x14 ] = {
                            0xf0, 0, 0xf0, 0, 0xf0, 0, 0xf0, 0, 0xf0, 0,
                            0xf0, 0, 0xf0, 0, 0xf0, 0, 0xf0, 0, 0xf0, 0,
                           };
    u16 error_vals[ 4 ] = { 0, 0, 0, 0, };

    PRINTD( COMPONENT_PC302, DBG_TRACE, "pa[%u]: reset", pa->dev_num );

    spin_lock_irqsave( &pa->lock, flags );
    ret = pa->ops->config_write( pa, PC302_MEMIF_CAEID, 0, 0x14, buf_vals );
    if ( 0x14 != ret )
        goto out;

    ret = pa->ops->config_write( pa, PC302_MEMIF_CAEID,
                                 PC302_MEMIF_FIFO_STAT_LOW, 4, error_vals );
    if ( 4 != ret )
        goto out;

    /* Save the PAI io_ctrl register value */
    ret = pa->ops->config_read( pa, PC302_PAI_CAEID,
                                PC302_PAI_IO_CTRL_REG, 1,
                                &io_ctrl_value );
    if ( 1 != ret )
        goto out;

    syscfg_update( AXI2CFG_SYS_CONFIG_PA_RST_MASK,
                   AXI2CFG_SYS_CONFIG_PA_RST_MASK);

    /* Wait for the reset to clear. */
    while ( syscfg_read() & AXI2CFG_SYS_CONFIG_PA_RST_MASK )
        cpu_relax();

    /* Wake the AXI2Pico and the Frac-N back up. We need to do this so that the
     * GPIO driver and Frac-N drivers can keep running without losing access to
     * their blocks. */
    val = 0;
    ret =  pa->ops->config_write( pa, PC302_AXI2PICO_CAEID,
				  SLEEP_REG_OFFSET, 1, &val );
    if ( 1 != ret)
        goto out;

    ret = pa->ops->config_write( pa, PC302_FRACN_CAEID,
				 SLEEP_REG_OFFSET, 1, &val );
    if ( 1 != ret)
        goto out;

    /* Wake up the PAI up so we can restore the io_ctrl register
     * back to what it was prior to the reset happening. */
    ret = pa->ops->config_write( pa, PC302_PAI_CAEID,
                                 SLEEP_REG_OFFSET, 1, &val );
    if ( 1 != ret)
        goto out;

    /* Restore the PAI io_ctrl register value */
    ret = pa->ops->config_write( pa, PC302_PAI_CAEID,
                                 PC302_PAI_IO_CTRL_REG, 1,
                                 &io_ctrl_value );
    if ( 1 != ret)
        goto out;

    ret = 0;

out:
    spin_unlock_irqrestore( &pa->lock, flags );
    return ret;
}

/*!
 * Read a GPR (general purpose register) in the PC302.
 *
 * @param pa The device being read from.
 * @param reg The register to read.
 * @param[out] value The address to store the register value in.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302_register_read( struct picoarray *pa,
                     struct pico_resource *reg,
                     u32 *value )
{
    struct pc302 *dev = to_pc302( pa );
    u32 gprStatus = 0;

    if ( !reg || PICO_RES_GPR != reg->type )
        return -EINVAL;

    if ( reg->value < PC302_GPR_AXI2PICO_0 ||
         reg->value > PC302_GPR_AXI2PICO_23 )
        return -EINVAL;

    if ( !pc3xx_is_running( pa ) )
        return -EPERM;

    pc302_axi2pico_reg_read( dev, reg->offset + AXI2PICO_VP_STATUS_OFFSET,
                             &gprStatus );
    if ((!IS_PORT_ENABLED(gprStatus)) ||
        (!IS_PORT_A_RECEIVE(gprStatus)) ||
        (!IS_PORT_READY_FOR_DATA(gprStatus)))
      return -EINVAL;

    pc302_axi2pico_reg_read( dev, reg->offset + AXI2PICO_VP_DATA_OFFSET,
                             value );

    return 0;
}

/*!
 * Write a GPR (general purpose register) in the PC302.
 *
 * @param pa The device being written to.
 * @param reg The register to write.
 * @param value The value to write to the register.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302_register_write( struct picoarray *pa,
                      struct pico_resource *reg,
                      u32 value )
{
    struct pc302 *dev = to_pc302( pa );
    u32 gprStatus = 0;

    if ( !reg || PICO_RES_GPR != reg->type )
        return -EINVAL;

    if ( reg->value < PC302_GPR_AXI2PICO_0 ||
         reg->value > PC302_GPR_AXI2PICO_23 )
        return -EINVAL;

    if ( !pc3xx_is_running( pa ) )
        return -EPERM;

    pc302_axi2pico_reg_read( dev, reg->offset + AXI2PICO_VP_STATUS_OFFSET,
          &gprStatus );
    if ((!IS_PORT_ENABLED(gprStatus)) ||
        (IS_PORT_A_RECEIVE(gprStatus)) ||
        (!IS_PORT_READY_FOR_DATA(gprStatus)))
      return -EINVAL;

    pc302_axi2pico_reg_write( dev, reg->offset + AXI2PICO_VP_DATA_OFFSET,
                              value );

    return 0;
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
pc302_pa_load( struct picoarray *pa,
               u32 *data,
               struct scatterlist *sgl  )
{
    struct pc302 *dev = to_pc302( pa );
    int ret = 0;
    pc302_dma_endpoint_t src_ep;
    pc302_dma_endpoint_t dst_ep;
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t dma_xfr=NULL;
    unsigned long timeout_jiffies = 0;
    u32 tmp = 0;

    /* Purge the configuration read and write ports */
    ret = axi2cfg_reg_write( dev->axi2cfg1_base, AXI2CFG_REG_PURGE_CFG,
           ( AXI2CFG_PURGE_CFG_RD_PORT_IDX | AXI2CFG_PURGE_CFG_WR_PORT_IDX ));

    /* Poll purge configuration register to determine when the purge has
       completed */
    timeout_jiffies = jiffies + TIMEOUT_10MS;
    while (1)
    {
        ret = axi2cfg_reg_read( dev->axi2cfg1_base,
                AXI2CFG_REG_PURGE_CFG, &tmp );
        if (!(tmp & AXI2CFG_PURGE_CFG_WR_PRGSS_PORT_IDX))
            break;

        /* Check for time out */
        if (time_after(jiffies, timeout_jiffies))
        {
            PRINTD( COMPONENT_PC302, DBG_WARN,
                "Timed out waiting for DMA configuration purge to complete");
            ret = -EIO;
            goto out;
        }
        cpu_relax();
    }

    /* Enable DMAC interface */
    ret =  axi2cfg_reg_write( dev->axi2cfg1_base, AXI2CFG_REG_DMAC1_CFG,
        (AXI2CFG_DMAC1_CONFIG_DMA_WRITE_IDX | AXI2CFG_DMAC1_CONFIG_ENABLE_IDX));

    /* Get the DMAC handle for a DMAC 2 which is the only DMAC that connects
       to the AXI2CFG bus */
    dma = pc302_dma_get_dma_handle(1);
    if (dma == NULL)
    {
        ret = -ENOMEM;
        goto out;
    }

    PRINTD( COMPONENT_PC302, DBG_TRACE, "src=%08X, cnt=%d", sg_dma_address( sgl ),
          sgl->length );

    /* Set up the source endpoint, source can be master 1, 2 or 3, but pick
     * 1 as we shouldn't be too busy at this point. The AXI2Cfg is fixed to
     * master 4 so we have to use that. The AXI2Cfg doesn't require any
     * hardware flow control for writing so treat it like a non-incrementing
     * block of memory. */
    src_ep.dma_addr = sg_dma_address( sgl );
    src_ep.master = PC302_DMA_MASTER1;
    src_ep.periph_not_mem = 0;
    src_ep.flow_controller = 0;
    src_ep.enable_sg = 0;
    src_ep.addr_inc = PC302_DMA_ADDR_INCREMENT;
    if ( sgl->length % 2)
        src_ep.tr_width = PC302_DMA_TR_WIDTH32;
    else
        src_ep.tr_width = PC302_DMA_TR_WIDTH64;
    src_ep.auto_reload = 0;
    src_ep.msize = PC302_DMA_MS_AUTO;

    dst_ep.dma_addr = dev->axi2cfg2_base_phys + AXI2CFG_REG_CFG_WR;
    dst_ep.master = PC302_DMA_MASTER4;
    dst_ep.periph_not_mem = 0;
    dst_ep.flow_controller = 0;
    dst_ep.enable_sg = 0;
    dst_ep.addr_inc = PC302_DMA_ADDR_NO_CHANGE;
    dst_ep.tr_width = PC302_DMA_TR_WIDTH32;
    dst_ep.msize = PC302_DMA_MS_AUTO;
    dst_ep.auto_reload = 0;

    dma_xfr = pc302_dma_setup_direct_xfr( dma, &src_ep, &dst_ep, NULL, NULL,
                                          sgl->length * sizeof( u32 ),
                                          PC302_DMA_PROTCTL_1, NULL, NULL );
    if (dma_xfr == NULL)
    {
        PRINTD( COMPONENT_PC302, DBG_WARN, "NULL xfr" );
        return -EIO;
    }

    /* Clear interrupts for this channel */
    pc302_dma_clear_int(dma_xfr,PC302_DMA_INT_TRANSFER);

    ret = pc302_dma_start(dma_xfr);
    if (ret)
    {
        PRINTD( COMPONENT_PC302, DBG_WARN, "error starting xfr" );
        goto dma_start_failed;
    }

    /* Poll status register to determine when the transfer has completed */
    timeout_jiffies = jiffies + TIMEOUT_10MS;
    while (1)
    {
        if (pc302_dma_get_raw_status(dma_xfr,PC302_DMA_INT_TRANSFER))
            break;

        /* Check for time out */
        if (time_after(jiffies, timeout_jiffies))
        {
            PRINTD( COMPONENT_PC302, DBG_WARN,
                    "Timed out waiting for DMA Load to complete");
            ret = -EIO;
            break;
        }

        cpu_relax();
    }

dma_start_failed:
    ret = pc302_dma_release(dma_xfr);
    if ( ret )
        PRINTD( COMPONENT_PC302, DBG_WARN, "DMA release failed, "
            "return code = %d", ret );

out:
    /* Permit ARM access to configuration port, disable DMAC interface */
    ( void )axi2cfg_reg_write( dev->axi2cfg1_base, AXI2CFG_REG_DMAC1_CFG,
                               0x00000 );

    if ( !ret )
        return sgl->length;

    return ret;
}

/*! The resources for a PC302 device. */
static struct pico_resource pc302_resources[] = {
    { .type = PICO_RES_DMA_CHANNEL, .value = PC302_DMA_AXI2PICO_0, .offset = 0x00 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC302_DMA_AXI2PICO_1, .offset = 0x10 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC302_DMA_AXI2PICO_2, .offset = 0x20 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC302_DMA_AXI2PICO_3, .offset = 0x30 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC302_DMA_AXI2PICO_4, .offset = 0x40 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC302_DMA_AXI2PICO_5, .offset = 0x50 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC302_DMA_AXI2PICO_6, .offset = 0x60 },
    { .type = PICO_RES_DMA_CHANNEL, .value = PC302_DMA_AXI2PICO_7, .offset = 0x70 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_0, .metadata = PC302_IRQ_AXI2PICO_0, .offset = 0x80 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_1, .metadata = PC302_IRQ_AXI2PICO_1, .offset = 0x90 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_2, .metadata = PC302_IRQ_AXI2PICO_2, .offset = 0xa0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_3, .metadata = PC302_IRQ_AXI2PICO_3, .offset = 0xb0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_4, .metadata = PC302_IRQ_AXI2PICO_4, .offset = 0xc0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_5, .metadata = PC302_IRQ_AXI2PICO_5, .offset = 0xd0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_6, .metadata = PC302_IRQ_AXI2PICO_6, .offset = 0xe0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_7, .metadata = PC302_IRQ_AXI2PICO_7, .offset = 0xf0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_8, .metadata = PC302_IRQ_AXI2PICO_8, .offset = 0x100 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_9, .metadata = PC302_IRQ_AXI2PICO_9, .offset = 0x110 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_10, .metadata = PC302_IRQ_AXI2PICO_10, .offset = 0x120 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_11, .metadata = PC302_IRQ_AXI2PICO_11, .offset = 0x130 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_12, .metadata = PC302_IRQ_AXI2PICO_12, .offset = 0x140 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_13, .metadata = PC302_IRQ_AXI2PICO_13, .offset = 0x150 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_14, .metadata = PC302_IRQ_AXI2PICO_14, .offset = 0x160 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_15, .metadata = PC302_IRQ_AXI2PICO_15, .offset = 0x170 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_16, .metadata = PC302_IRQ_AXI2PICO_16, .offset = 0x180 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_17, .metadata = PC302_IRQ_AXI2PICO_17, .offset = 0x190 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_18, .metadata = PC302_IRQ_AXI2PICO_18, .offset = 0x1a0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_19, .metadata = PC302_IRQ_AXI2PICO_19, .offset = 0x1b0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_20, .metadata = PC302_IRQ_AXI2PICO_20, .offset = 0x1c0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_21, .metadata = PC302_IRQ_AXI2PICO_21, .offset = 0x1d0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_22, .metadata = PC302_IRQ_AXI2PICO_22, .offset = 0x1e0 },
    { .type = PICO_RES_GPR, .value = PC302_GPR_AXI2PICO_23, .metadata = PC302_IRQ_AXI2PICO_23, .offset = 0x1f0 },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_0, .metadata = PC302_GPR_AXI2PICO_0, .offset = 8, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_1, .metadata = PC302_GPR_AXI2PICO_1, .offset = 9, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_2, .metadata = PC302_GPR_AXI2PICO_2, .offset = 10, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_3, .metadata = PC302_GPR_AXI2PICO_3, .offset = 11, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_4, .metadata = PC302_GPR_AXI2PICO_4, .offset = 12, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_5, .metadata = PC302_GPR_AXI2PICO_5, .offset = 13, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_6, .metadata = PC302_GPR_AXI2PICO_6, .offset = 14, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_7, .metadata = PC302_GPR_AXI2PICO_7, .offset = 15, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_8, .metadata = PC302_GPR_AXI2PICO_8, .offset = 16, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_9, .metadata = PC302_GPR_AXI2PICO_9, .offset = 17, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_10, .metadata = PC302_GPR_AXI2PICO_10, .offset = 18, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_11, .metadata = PC302_GPR_AXI2PICO_11, .offset = 19, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_12, .metadata = PC302_GPR_AXI2PICO_12, .offset = 20, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_13, .metadata = PC302_GPR_AXI2PICO_13, .offset = 21, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_14, .metadata = PC302_GPR_AXI2PICO_14, .offset = 22, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_15, .metadata = PC302_GPR_AXI2PICO_15, .offset = 23, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_16, .metadata = PC302_GPR_AXI2PICO_16, .offset = 24, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_17, .metadata = PC302_GPR_AXI2PICO_17, .offset = 25, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_18, .metadata = PC302_GPR_AXI2PICO_18, .offset = 26, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_19, .metadata = PC302_GPR_AXI2PICO_19, .offset = 27, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_20, .metadata = PC302_GPR_AXI2PICO_20, .offset = 28, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_21, .metadata = PC302_GPR_AXI2PICO_21, .offset = 29, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_22, .metadata = PC302_GPR_AXI2PICO_22, .offset = 30, },
    { .type = PICO_RES_IRQ, .value = PC302_IRQ_AXI2PICO_23, .metadata = PC302_GPR_AXI2PICO_23, .offset = 31, },
    { .type = 0, .value = 0 }, /* Sentinel value. Do not remove and keep this
                                * at the end. */
};

/*! The offset in the AXI2Pico for GPR virtual ports. */
#define AXI2PICO_GPR_VP_START       ( 0x80 )

/*!
 * Add an IRQ handler to the PC302 for a given interrupt source.
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
pc302_add_irq_handler( struct picoarray *pa,
                       struct pico_resource *irq,
                       int ( *callback )( struct pico_resource *irq,
                                          void *cookie ),
                       void *cookie )
{
    struct pc302_irq_handler *handler;
    struct pc302 *dev = to_pc302( pa );
    int ret;
    unsigned gpr_num = irq->value - PC302_IRQ_AXI2PICO_0;
    unsigned offset =
            AXI2PICO_GPR_VP_START + ( gpr_num * AXI2PICO_VP_SPACING );
    u32 val = 0;
    unsigned long flags;

    if ( irq->type != PICO_RES_IRQ || !irq->exclusive || !callback )
        return -EINVAL;

    spin_lock_irqsave( &pa->lock, flags );

    /* Need to check that the port is bocking before enabling interrupts */
    pc302_axi2pico_reg_read( dev, offset + AXI2PICO_VP_STATUS_OFFSET,
                             &val);

    /* If the port is non-blocking then fail as we will receive constant
     * interrupts. We also only support receive GPRs for interrupt
     * generation. */
    if ( !IS_PORT_A_RECEIVE( val ) || IS_PORT_NONBLOCKING( val ) )
    {
        PRINTD( COMPONENT_PC302, DBG_WARN,
                "cannot enable IRQ: AXI2Pico VP is not"
                " configured as blocking receive" );
        PRINTD( COMPONENT_PC302, DBG_WARN,"offset=%d",offset);
        ret = -EIO;
        goto out;
    }

    ret = -ENOMEM;
    handler = kmalloc( sizeof( *handler ), GFP_KERNEL );
    if ( !handler )
        goto out;

    handler->callback   = callback;
    handler->cookie     = cookie;
    handler->irq        = irq;

    list_add( &handler->list, &dev->axi2pico_irq_handlers.list );

    /* Enable the interrupt. */
    pc302_axi2pico_reg_read( dev, offset + AXI2PICO_VP_CONFIG_OFFSET, &val );
    val |= AXI2PICO_INT_EN;
    pc302_axi2pico_reg_write( dev, offset + AXI2PICO_VP_CONFIG_OFFSET, val );

    ret = 0;
out:

    spin_unlock_irqrestore( &pa->lock, flags );
    return ret;
}


/*!
 * Remove an IRQ handler from a PC302 interrupt source.
 *
 * @param pa The PC302 to remove the handler from.
 * @param irq The IRQ to remove the handler for.
 */
static void
pc302_remove_irq_handler( struct picoarray *pa,
                          struct pico_resource *irq )
{
    struct list_head *pos;
    struct list_head *tmp;
    struct pc302 *dev = to_pc302( pa );
    struct pc302_irq_handler *handler;
    unsigned long flags;
    unsigned gpr_num = irq->value - PC302_IRQ_AXI2PICO_0;
    unsigned offset =
            AXI2PICO_GPR_VP_START + ( gpr_num * AXI2PICO_VP_SPACING );
    u32 val= 0;

    /* Protect against interrupts being raised whilst we remove the handler.
     */
    spin_lock_irqsave( &pa->lock, flags );

    /* Disable the interrupt. */
    pc302_axi2pico_reg_read( dev, offset + AXI2PICO_VP_CONFIG_OFFSET, &val );
    val &= ~AXI2PICO_INT_EN;
    pc302_axi2pico_reg_write( dev, offset + AXI2PICO_VP_CONFIG_OFFSET, val );

    list_for_each_safe( pos, tmp, &dev->axi2pico_irq_handlers.list )
    {
        handler = container_of( pos, struct pc302_irq_handler, list );
        if ( handler->irq == irq )
        {
            list_del( pos );
            kfree( handler );
            break;
        }
    }

    spin_unlock_irqrestore( &pa->lock, flags );
}

/*!
 * PC302 AXI2Pico ISR. This ISR will be called when the AXI2Pico generates an
 * interrupt and checks all of the AXI2Pico interrupt sources for raised
 * interrupts and calls the appropriate handler if there is one registered. If
 * there is no interrupt handler for the IRQ source then that source is
 * disabled.
 *
 * @param irq The irq that has been raised.
 * @param dev The PC302 device that has raised the interrupt.
 * @return Returns IRQ_HANDLED on success.
 */
static irqreturn_t
pc302_axi2pico_irq( int irq,
                    void *dev )
{
    struct pc302 *pc302dev = dev;
    struct list_head *pos;
    struct pc302_irq_handler *handler;
    int ret;
    u32 int_status;
    unsigned i;
    unsigned handled;

    pc302_axi2pico_reg_read( pc302dev, AXI2PICO_INT_STATUS_OFFSET,
                             &int_status );
    for ( i = 0; i < AXI2PICO_NUM_VPS; ++i )
    {
        if ( int_status & ( 1 << i ) )
        {
            handled = 0;
            list_for_each( pos, &pc302dev->axi2pico_irq_handlers.list )
            {
                handler = container_of( pos, struct pc302_irq_handler, list );
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
                PRINTD( COMPONENT_PC302, DBG_WARN,
                        "no interrupt handler for AXI2Pico VP %u", i );
                /* Disable the interrupt generation for this GPR - there
                 * is no handler installed. */
                pc302_axi2pico_reg_write( pc302dev,
                       ( i * AXI2PICO_VP_SPACING ) + AXI2PICO_GPR_VP_START,
                        0 );
            }
        }
    }

    return IRQ_HANDLED;
}

/*!
 * \brief PC302 DMA channel mappings
 *
 * This structure defines the DMAC and DMA channel numbers for each of the
 * picoArray DMA channels. This structure is internal and specific to the
 * PC302.
 */
struct pc302_dma_mapping
{
    pc302_master_t srcMaster;
    pc302_master_t dstMaster;
};

/*!
 * \brief Initialise the DMAC and DMA channel mappings
 *
 * @param dma Pointer to the global DMA structure for the PC302 device
 */
static void
pc302_dma_init( struct pc302_dma_channel *dma )
{
    unsigned i;
    pc302_dma_t dmac = pc302_dma_get_dma_handle( 0 );
    /* Make a slightly random mapping to attempt to distribute traffic over
     * a number of masters for efficiency. */
    struct pc302_dma_mapping dma_mapping[PICO_NUM_DMA_CHANNELS] = {
        { PC302_DMA_MASTER1, PC302_DMA_MASTER2},
        { PC302_DMA_MASTER3, PC302_DMA_MASTER4},
        { PC302_DMA_MASTER2, PC302_DMA_MASTER1},
        { PC302_DMA_MASTER4, PC302_DMA_MASTER3},
        { PC302_DMA_MASTER1, PC302_DMA_MASTER2},
        { PC302_DMA_MASTER3, PC302_DMA_MASTER4},
        { PC302_DMA_MASTER2, PC302_DMA_MASTER1},
        { PC302_DMA_MASTER4, PC302_DMA_MASTER3},
    };

    /* Assign the DMAC number and channel to the picoArray DMA channel */
    for (i = 0; i < PICO_NUM_DMA_CHANNELS; i++)
    {
        dma[i].srcMaster = dma_mapping[i].srcMaster;
        dma[i].dstMaster = dma_mapping[i].dstMaster;
        dma[i].hs.hwInterface = i;
        dma[i].dmac = dmac;
        dma[i].hs.active_low = 0; /* PC302 DREQs are active high */
        dma[i].stateActive = 0;
        dma[i].xfr = NULL;
    }
}

/*!
 * Open and enable the specified DMA channel
 *
 * @param pa The picoArray to DMA the data.
 * @param dma_channel The DMA channel to open
 * @param is_downlink 1 if the channel is an downlink, 0 for an uplink
 * @return 0 on success, a negative number on failure
 */
__must_check static int
pc302_dma_open( struct picoarray *pa,
                struct pico_resource *dma_channel,
                int is_downlink )
{
    struct pc302 *pc302dev = to_pc302( pa );
    struct pc302_dma_channel *dma =
        &pc302dev->dma_channel[dma_channel->value];
    u32 gprStatus = 0;

    PRINTD( COMPONENT_PC302, DBG_TRACE,
            "pa_dma %d  => cpu_dma %d",
            ( dma_channel->value-PC302_DMA_AXI2PICO_0 ),
            dma_channel->value);

    dma->pico_addr = (dma_addr_t)pc302dev->axi2pico_base_phys
        + ((dma_channel->value - PC302_DMA_AXI2PICO_0) * AXI2PICO_VP_SPACING)
        + AXI2PICO_VP_DATA_OFFSET;

    if ( ( dma_channel->value < PC302_DMA_AXI2PICO_0 ) ||
         ( dma_channel->value > PC302_DMA_AXI2PICO_7 ) )
        return -EINVAL;

    pc302_axi2pico_reg_read( pc302dev, dma_channel->offset
              + AXI2PICO_VP_STATUS_OFFSET, &gprStatus );

    if ( !IS_PORT_ENABLED(gprStatus) )
    {
        PRINTD( COMPONENT_PC302, DBG_ERROR, "dma %d: is not enabled",
           dma_channel->value);
        return -EINVAL;
    }

    /* Set the DMAC Enable bit (bit1) in the DMA config register */
    if ( IS_PORT_A_RECEIVE(gprStatus) )
       /* Enable the UL DMA channel by setting bit 1, and set the
           watermark to 1 x the burst size on bits 2 to 8.  */
       (void)pc302_axi2pico_reg_write( pc302dev,
           ((dma_channel->value - PC302_DMA_AXI2PICO_0) * AXI2PICO_VP_SPACING)
           + AXI2PICO_VP_CONFIG_OFFSET, 0x02 + (PC302_DMA_BURST_SIZE << 2));
    else
        /* Enable the DL DMA channel by setting bit 1, and set the watermark
           to 2 x the burst size on bits 2 to 8 */
        (void)pc302_axi2pico_reg_write( pc302dev,
         ((dma_channel->value-PC302_DMA_AXI2PICO_0)*AXI2PICO_VP_SPACING)
         +AXI2PICO_VP_CONFIG_OFFSET, 0x02 + ((2*PC302_DMA_BURST_SIZE) << 2));

    dma->sgl = pc302_dma_list_create( dma->dmac, 1 );
    if ( dma->sgl == NULL )
    {
        PRINTD( COMPONENT_PC302, DBG_ERROR, "dma %d: cannot allocate "
           "scatter gather list", dma_channel->value);
        return -EINVAL;
    }

    /* Initialise all variables that may become set */
    dma->xfr = NULL;
    dma->channel = dma_channel->value - PC302_DMA_AXI2PICO_0;
    dma->stateActive = 0;
    dma->callback = NULL;
    dma->cookie = NULL;

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
pc302_dma_close( struct picoarray *pa,
                 struct pico_resource *dma_channel )
{
    int ret = 0;
    struct pc302 *pc302dev = to_pc302( pa );
    struct pc302_dma_channel *dma =
        &pc302dev->dma_channel[dma_channel->value];

    if ( dma->xfr )
    {
        if ( dma->stateActive )
        {
            ret = pc302_dma_abort( dma->xfr );
            if ( ret )
                PRINTD( COMPONENT_PC302, DBG_WARN, "DMA abort failed, "
                "return code=%d", ret );

            PRINTD( COMPONENT_PC302, DBG_WARN, "dma %d: stopped while dma "
                "in progress", dma_channel->value);
        }

        /* Disable all interrupts for this channel */
        ret = pc302_dma_disable_int( dma->xfr, PC302_DMA_INT_ALL );
        if ( ret )
            PRINTD( COMPONENT_PC302, DBG_WARN, "disable interrupt failed "
                "return code=%d", ret );

        /* Reset all interrupts for this channel */
        pc302_dma_clear_int(dma->xfr, PC302_DMA_INT_ALL );

        ret = pc302_dma_release( dma->xfr );
        if (ret )
            PRINTD( COMPONENT_PC302, DBG_WARN, "DMA_release failed "
                "return code = %d", ret );
    }

    if ( dma->sgl )
    {
        ret = pc302_dma_list_destroy( dma->sgl );
        if (ret)
            PRINTD( COMPONENT_PC302, DBG_WARN, "DMA list_destroy failed "
                    "return code = %d", ret );
    }

    dma->stateActive = 0;
    return 0;
}

/*!
 * DMA handler function. This function simply resets the channel active flag
 * and passes the cookie onto the handler specified in the pc302_to_device
 * and pc302_from_device functions.
 *
 * @param cookie The cookie to pass to the callback function.
 * @param errno The status of the DMA operation
 * @return 0 on success, non-zero on failure
 */
static int
pc302_dma_handler( void *cookie,
                   int errno )
{
    struct pc302_dma_channel *dma = (struct pc302_dma_channel *)cookie;

    if ( errno & PC302_DMA_INT_ERROR)
        errno = -EIO;
    else
        errno = 0;

    dma->stateActive = 0;
    return dma->callback(dma->cookie, errno);
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
 * @return 0 on success, non-zero on failure
 */
static int
pc302_dma_to_device( struct picoarray *pa,
                     struct pico_resource *dma_channel,
                     struct scatterlist *sgl,
                     size_t nbytes,
                     int ( *callback )( void *cookie,
                                        int errno ),
                     void *cookie )
{
    int ret = 0;
    struct pc302 *dev = to_pc302( pa );
    struct pc302_dma_channel *dma = &dev->dma_channel[dma_channel->value];
    pc302_dma_endpoint_t src_ep;
    pc302_dma_endpoint_t dst_ep;

    if ( dma->stateActive )
      return -EAGAIN;

    PRINTD( COMPONENT_PC302, DBG_TRACE, "DMA transfer Chan %d, Bytes %d",
          dma_channel->value, nbytes );

    /* make a single SGL entry */
    ret = pc302_dma_list_clear( dma->sgl );

    src_ep.msize = PC302_DMA_MS_AUTO;
#if (PC302_DMA_BURST_SIZE == 1)
    src_ep.tr_width = PC302_DMA_TR_WIDTH32;
#else
    src_ep.tr_width = PC302_DMA_TR_WIDTH64;
#endif
    src_ep.dma_addr = sg_dma_address( sgl );
    src_ep.master = dma->srcMaster;
    src_ep.periph_not_mem = 0;
    src_ep.flow_controller = 0;
    src_ep.enable_sg = 0;
    src_ep.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src_ep.auto_reload = 0;

    SET_PICOARRAY_MSIZE(dst_ep);
    dst_ep.tr_width = PC302_DMA_TR_WIDTH32;
    dst_ep.dma_addr = dma->pico_addr;
    dst_ep.master = dma->dstMaster;
    dst_ep.periph_not_mem = 1;
    dst_ep.flow_controller = 0;
    dst_ep.enable_sg = 0;
    dst_ep.addr_inc = PC302_DMA_ADDR_NO_CHANGE;
    dst_ep.auto_reload = 0;

    if ( dma->xfr != NULL )
    {
        ret = pc302_dma_release(dma->xfr);
        if ( ret )
             PRINTD( COMPONENT_PC302, DBG_ERROR, "failed to release previous xfr, "
                     " ret = %d", ret );
    }

    dma->xfr = pc302_dma_setup_direct_xfr( dma->dmac, &src_ep,
            &dst_ep, NULL, &(dma->hs), nbytes, PC302_DMA_PROTCTL_1,
            pc302_dma_handler, dma );
    if ( dma->xfr == NULL )
    {
        PRINTD( COMPONENT_PC302, DBG_ERROR, "error setting up list xfr "
               "for channel %d", dma_channel->value );
        return -EINVAL;
    }

    ret = pc302_dma_enable_int(dma->xfr,
               PC302_DMA_INT_ERROR | PC302_DMA_INT_TRANSFER);
    if ( ret )
    {
        PRINTD( COMPONENT_PC302, DBG_ERROR, "failed to enable interrupts, "
            "ret=%d", ret );
    }

    dma->stateActive = 1;
    dma->callback = callback;
    dma->cookie = cookie;

    ret = pc302_dma_start( dma->xfr );
    if ( ret )
    {
        PRINTD( COMPONENT_PC302, DBG_ERROR, "error starting xfr, "
              "ret=%d", ret );
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
 * @return 0 on success, non-zero on failure
 */
static int
pc302_dma_from_device( struct picoarray *pa,
                         struct pico_resource *dma_channel,
                         struct scatterlist *sgl,
                         size_t nbytes,
                         int ( *callback )( void *cookie,
                                            int errno ),
                         void *cookie )
{
    int ret = 0;
    struct pc302 *dev = to_pc302( pa );
    struct pc302_dma_channel *dma = &dev->dma_channel[dma_channel->value];
    pc302_dma_endpoint_t src_ep;
    pc302_dma_endpoint_t dst_ep;

    if ( dma->stateActive )
        return -EAGAIN;

    PRINTD( COMPONENT_PC302, DBG_TRACE, "chan %d, bytes %d",
               dma_channel->value, nbytes );

    /* make a single SGL entry */
    ret = pc302_dma_list_clear(dma->sgl);

    SET_PICOARRAY_MSIZE(src_ep);
    src_ep.tr_width = PC302_DMA_TR_WIDTH32;
    src_ep.dma_addr = dma->pico_addr;
    src_ep.master = dma->srcMaster;
    src_ep.periph_not_mem = 1;
    src_ep.flow_controller = 0;
    src_ep.enable_sg = 0;
    src_ep.addr_inc = PC302_DMA_ADDR_NO_CHANGE;
    src_ep.auto_reload = 0;

    dst_ep.msize = PC302_DMA_MS_AUTO;
#if (PC302_DMA_BURST_SIZE == 1)
    dst_ep.tr_width = PC302_DMA_TR_WIDTH32;
#else
    dst_ep.tr_width = PC302_DMA_TR_WIDTH64;
#endif
    dst_ep.dma_addr = sg_dma_address( sgl );
    dst_ep.master = dma->dstMaster;
    dst_ep.periph_not_mem = 0;
    dst_ep.flow_controller = 0;
    dst_ep.enable_sg = 0;
    dst_ep.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst_ep.auto_reload = 0;

    if ( dma->xfr != NULL )
        ret = pc302_dma_release( dma->xfr );

    dma->xfr = pc302_dma_setup_direct_xfr( dma->dmac,
                &src_ep, &dst_ep, &(dma->hs), NULL, nbytes,
                PC302_DMA_PROTCTL_1, pc302_dma_handler, dma );
    if ( dma->xfr == NULL )
    {
        PRINTD( COMPONENT_PC302, DBG_ERROR, "error setting up list xfr");
        return -EINVAL;
    }

    ret = pc302_dma_enable_int( dma->xfr,
                              PC302_DMA_INT_ERROR | PC302_DMA_INT_TRANSFER );
    if ( ret )
    {
      PRINTD( COMPONENT_PC302, DBG_WARN, "DMA enable interrupts failed "
          "return code = %d", ret );
    }

    dma->stateActive = 1;
    dma->callback = callback;
    dma->cookie = cookie;
    ret = pc302_dma_start( dma->xfr );
    if ( ret )
    {
        PRINTD( COMPONENT_PC302, DBG_ERROR, "error starting xfr, ret=%d",
                ret );
    }

    return ret;
}

/*!
 * Get the device type of a PC302.
 *
 * @param pa The device to query.
 * @return Always returns PICOARRAY_PC302.
 */
static enum picoarray_device_type
pc302_get_device_type( struct picoarray *pa )
{
    return PICOARRAY_PC302;
}

/*!
 * Get the ITM, ITS and procIF IRQ resources.
 * This function will return an error as the PC302 has no procIF interface
 *
 * @param pa The device to take to DMA the data.
 * @param its The ITS resource or NULL on failure
 * @param itm The ITM resource or NULL on failure
 * @param procif_irq The IRQ resource or NULL on failure
 *
 * @return EINVAL in all situations
 */
static int
pc302_get_procif_resource( struct picoarray *pa,
                           struct pico_resource **its,
                           struct pico_resource **itm,
                           struct pico_resource **procif_irq )
{
    return -EINVAL;
}

/*!
 * Put the ITM, ITS and procIF IRQ resources.
 * This function is not relevant to the PC302 as it has no procIF interface.
 *
 * @param pa The device to take to DMA the data.
 * @param its The ITS resource
 * @param itm The ITM resource
 * @param procif_irq The IRQ resource
 */
static void
pc302_put_procif_resource( struct picoarray *pa,
                           struct pico_resource *its,
                           struct pico_resource *itm,
                           struct pico_resource *procif_irq )
{
}

/*! Operations for the PC302 devices. */
static struct picoarray_ops pc302_ops = {
    .sync                = pc302_sync,
    .start               = pc302_start,
    .stop                = pc302_stop,
    .get_device_type     = pc302_get_device_type,
    .config_read         = pc302_config_read,
    .config_write        = pc302_config_write,
    .register_read       = pc302_register_read,
    .register_write      = pc302_register_write,
    .reset               = pc302_reset,
    .get_resource        = generic_get_resource,
    .get_procif_resource = pc302_get_procif_resource,
    .put_resource        = generic_put_resource,
    .put_procif_resource = pc302_put_procif_resource,
    .destructor          = pc302_destroy,
    .add_irq_handler     = pc302_add_irq_handler,
    .remove_irq_handler  = pc302_remove_irq_handler,
    .dma_to_device       = pc302_dma_to_device,
    .dma_from_device     = pc302_dma_from_device,
    .dma_open            = pc302_dma_open,
    .dma_close           = pc302_dma_close,
    .pa_load             = pc302_pa_load,
};

/*!
 * Probe method for the PC302 platform driver. This function creates a new
 * PC302 instance and is responsible for allocating all of the resources
 * required.
 *
 * @param pdev The platform device that has been probed.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302_probe( struct platform_device *pdev )
{
    struct resource *res;
    int ret;
    struct pc302 *newdev = kmalloc( sizeof( *newdev ), GFP_KERNEL );

    if ( !newdev )
        return -ENOMEM;

    ret = -ENOMEM;
    newdev->pa.resources = kmalloc( sizeof( pc302_resources ), GFP_KERNEL );
    if ( !newdev->pa.resources )
        goto out;
    memcpy( newdev->pa.resources, pc302_resources, sizeof( pc302_resources ) );

    newdev->pa.dev_num = pdev->id;
    newdev->pa.ops = &pc302_ops;
    newdev->pa.features = PICOARRAY_HAS_DMA_LOAD | PICOARRAY_HAS_DMA;
    newdev->pa.max_dma_sz = PICOARRAY_MAX_TRANSFER;
    spin_lock_init( &newdev->pa.lock );

    ret = -EINVAL;

    /* Get the IRQ for AXI2Pico GPRs. */
    res = platform_get_resource_byname( pdev, IORESOURCE_IRQ, "gpr_irq" );
    if ( !res )
        goto out;
    newdev->gpr_irq = res->start;

    /* Get the register base address for the lower AXI2CFG registers */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM, "procif" );
    if ( !res )
        goto out;
    newdev->axi2cfg1_base_phys = res->start;
    newdev->axi2cfg1_base_len = ( res->end - res->start ) + 1;

    /* Get the register base address for the upper AXi2CFG registers */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM,
                                        "procif2" );
    if ( !res )
        goto out;
    newdev->axi2cfg2_base_phys = res->start;
    newdev->axi2cfg2_base_len = ( res->end - res->start ) + 1;

    /* Get the register base address for the AXI2Pico. */
    res = platform_get_resource_byname( pdev, IORESOURCE_MEM,
                                        "ahb2pico_axi2pico" );
    if ( !res )
        goto out;
    newdev->axi2pico_base_phys = res->start;
    newdev->axi2pico_base_len = ( res->end - res->start ) + 1;

    /* Map the resources. */
    newdev->axi2cfg1_base = request_and_map( "axi2cfg1", newdev->axi2cfg1_base_phys,
                                        newdev->axi2cfg1_base_len );
    if ( !newdev->axi2cfg1_base )
        goto out;

    newdev->axi2cfg2_base = request_and_map( "axi2cfg2", newdev->axi2cfg2_base_phys,
                                        newdev->axi2cfg2_base_len );
    if ( !newdev->axi2cfg2_base )
        goto axi2cfg2_map_failed;

    newdev->axi2pico_base = request_and_map( "axi2pico", newdev->axi2pico_base_phys,
                                        newdev->axi2pico_base_len );
    if ( !newdev->axi2pico_base )
        goto ahb2pico_map_failed;

    ret = request_irq( newdev->gpr_irq, pc302_axi2pico_irq, IRQF_DISABLED,
                       pdev->name, newdev );
    if ( ret )
        goto axi2pico_irq_failed;

    /* Initialise the interrupt handler lists. */
    INIT_LIST_HEAD( &newdev->axi2pico_irq_handlers.list );

    /* Initialise the DMA channel structure */
    pc302_dma_init( newdev->dma_channel );

    ret = picoif_register_dev( &newdev->pa );
    goto out;

axi2pico_irq_failed:
    unmap_and_release( newdev->axi2pico_base_phys, newdev->axi2pico_base_len,
                       newdev->axi2pico_base );

ahb2pico_map_failed:
    unmap_and_release( newdev->axi2cfg2_base_phys, newdev->axi2cfg2_base_len,
                       newdev->axi2cfg2_base );

axi2cfg2_map_failed:
    unmap_and_release( newdev->axi2cfg1_base_phys, newdev->axi2cfg1_base_len,
                       newdev->axi2cfg1_base );
out:
    if ( ret && newdev->pa.resources )
        kfree( newdev->pa.resources );

    if ( ret )
        kfree( newdev );
    else
        ++num_pc302s;

    return ret;
}

/*!
 * Remove method for the PC302 platform driver. This method is called when the
 * platform driver is removed and must release all resources the driver has
 * been using.
 *
 * @param pdev The platform device being remove.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302_remove( struct platform_device *pdev )
{
    struct picoarray *pa = picoif_get_device( pdev->id );
    struct pc302 *pc302dev = to_pc302( pa );
    int ret = 0;

    free_irq( pc302dev->gpr_irq, pc302dev );
    unmap_and_release( pc302dev->axi2cfg1_base_phys, pc302dev->axi2cfg1_base_len,
                       pc302dev->axi2cfg1_base );
    unmap_and_release( pc302dev->axi2cfg2_base_phys, pc302dev->axi2cfg2_base_len,
                       pc302dev->axi2cfg2_base );
    unmap_and_release( pc302dev->axi2pico_base_phys, pc302dev->axi2pico_base_len,
                       pc302dev->axi2pico_base );

    kfree( pc302dev );

    return ret;
}

/*! The PC302 platform driver.
 *  \todo Change the name to PC302 specific rather than generic picoArray.
 */
static struct platform_driver pc302_driver = {
    .probe      = pc302_probe,
    .remove     = pc302_remove,
    .driver     = {
        .name   = "picoArray",
    },
};

int
pc302_init( void )
{
    return platform_driver_register( &pc302_driver );
}

/*!
 * Destructor to be called when a PC302 is removed from picoif. This
 * function must decrement the number of PC302s registered, and when this
 * reaches zero, remove the platform driver.
 *
 * @param pa The device being removed.
 */
static void
pc302_destroy( struct picoarray *pa )
{
    PRINTD( COMPONENT_PC302, DBG_TRACE, "pA[%u]: destructor called",
            pa->dev_num );

    /* If we have no more pc302s, then remove the driver. */
    if ( 0 == --num_pc302s )
        platform_driver_unregister( &pc302_driver );
}
