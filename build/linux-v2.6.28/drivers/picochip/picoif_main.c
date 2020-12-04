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
 * \file picoif_main.c
 * \brief Main file for picoIf kernel driver.
 *
 * This file implements the core functionality of the picoIf kernel driver.
 * This file is responsible for registration of devices and transport modules
 * and handling all userspace/kernelspace interfaces such as file operations.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/poll.h>
#include <linux/aio.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <asm/errno.h>

#include "picoarray.h"
#include "picoif_internal.h"
#include <linux/picochip/picoif_ioctl.h>
#include "picoif_module.h"
#include "gpr_interrupt_internal.h"
#include "dma_internal.h"
#include "hwif_internal.h"
#include "hwif2_internal.h"
#include "debug.h"
#include <linux/picochip/picoif.h>

/*! The maximum number of devices the driver supports. */
#define PICOIF_MAX_DEVICES ( 8 )

/*! The maximum number of transport modules the driver supports. */
#define PICOIF_MAX_MODULES ( 8 )

/*! The maximum number of bytes to be transfered between user and
 *  kernel space in one operation. This number must take into account
 *  any DMA tranaction limits that exist if DMA is being used */
#define PICOIF_MAX_TRANSFER_SIZE ( 4096 )

#ifdef CONFIG_PICOIF_DMAPOOL
/*
 * We keep at most PICOIF_MAX_NUM_DMA_BUFFERS buffers in a pool ready to be
 * re-used for DMA coherent access.  The size of each of these buffers
 * will be exactly PICOIF_DMA_POOL_BUFFER_SIZE bytes.
 *
 * We have six transports, 2 each (uplink/downlink) for the Control and User
 * planes and two for the RSE.  64K is big enough for most situations.
 */
#define PICOIF_MAX_NUM_DMA_BUFFERS   6
#define PICOIF_DMA_POOL_BUFFER_SIZE  65536

/*!
 * \brief Structure to store information recycled DMA coherent buffers
 */
struct picoif_dmabuf_t {
    dma_addr_t  phys;
    void       *virt;
};
#endif /* CONFIG_PICOIF_DMAPOOL */

/*!
 * \brief Structure to store devices in the system and any other data private
 * to the driver.
 */
struct picoif_core_t
{
    /*! The devices that the driver interfaces to. */
    struct picoarray            *devices[ PICOIF_MAX_DEVICES ];

    /*! The miscdevice used for the Linux device registration. */
    struct miscdevice           miscdev;

    /*! The modules that have been registered with picoIf. */
    struct picoif_module        *modules[ PICOIF_MAX_MODULES ];

#ifdef CONFIG_DEBUG_FS
    /*! The entry in /debug for all debugfs files. */
    struct dentry               *debugfs_dir;

    /*! The entry in /debug/picoif for the debug log. */
    struct dentry               *debug_log;
#endif /* CONFIG_DEBUG_FS */

#ifdef CONFIG_PICOIF_DMAPOOL
    struct picoif_dmabuf_t      dma_buffers[PICOIF_MAX_NUM_DMA_BUFFERS];
    int                         active_dma_buffers;
    spinlock_t                  dma_buffers_lock;
#endif /* CONFIG_PICOIF_DMAPOOL */
};

static int
picoif_open( struct inode *inode,
             struct file *filp );

static ssize_t
picoif_write( struct file *filp,
              const char __user *buf,
              size_t len,
              loff_t *loff );

static ssize_t
picoif_read( struct file *filp,
             char __user *buf,
             size_t len,
             loff_t *loff );

static int
picoif_ioctl( struct inode *inode,
              struct file *filp,
              unsigned int cmd,
              unsigned long arg );

static int
picoif_release( struct inode *inode,
                struct file *filp );

static int
picoif_mmap( struct file *filp,
             struct vm_area_struct *vma );

static unsigned int
picoif_poll( struct file *filp,
             struct poll_table_struct *pollt );

static ssize_t
picoif_aio_write( struct kiocb *iocb,
                  const struct iovec *vecs,
                  unsigned long nr_segs,
                  loff_t offset );

static ssize_t
picoif_aio_read( struct kiocb *iocb,
                 const struct iovec *vecs,
                 unsigned long nr_segs,
                 loff_t offset );

/*!
 * File operations structure for the driver. All file operations for the
 * driver and transports will be in this structure and if the operation needs
 * to be rerouted to a transport module, this will happen in these functions.
 */
static struct file_operations picoif_fops = {
    .owner      = THIS_MODULE,
    .open       = picoif_open,
    .release    = picoif_release,
    .write      = picoif_write,
    .read       = picoif_read,
    .ioctl      = picoif_ioctl,
    .mmap       = picoif_mmap,
    .poll       = picoif_poll,
    .aio_read   = picoif_aio_read,
    .aio_write  = picoif_aio_write,
};

/*!
 * The miscdevice for the driver.
 */
static struct picoif_core_t picoif_core = {
    .miscdev    =  {
        .fops   = &picoif_fops,
        .name   = "picoif",
        .minor  = PICOIF_MINOR,  /* Our /dev is read only, so use a fixed value */
    },
};

int
picoif_buf_copy_from( void *dst,
                      struct picoif_buf *src,
                      unsigned offset,
                      size_t nbytes )
{
    if ( src->is_user )
        return copy_from_user( dst, src->ubuf + offset, nbytes );

    return memcpy( dst, src->kbuf + offset, nbytes ) ? 0 : 1;
}

int
picoif_buf_copy_to( struct picoif_buf *dst,
                    void *src,
                    unsigned offset,
                    size_t nbytes )
{
    if ( dst->is_user )
        return copy_to_user( dst->ubuf + offset, src, nbytes );

    return memcpy( dst->kbuf + offset, src, nbytes ) ? 0 : 1;
}

/*!
 * Open a new instance of the driver. At open, the file descriptor will be a
 * generic picoif instance for configuration and to use it as a transport
 * file descriptor, the NEW_TRANSPORT ioctl() call must be used to associate
 * it with a transport instance.
 *
 * @param inode The inode of the device file.
 * @param filp The context of the open instance. The private_data member can
 * be used to store driver information for this context. For non-transport
 * instances, private_data should be NULL and for transport instances it
 * should point to a struct picoif_context.
 * @return Returns zero on success, negative on failure.
 */
static int
picoif_open( struct inode *inode,
             struct file *filp )
{
    filp->private_data = NULL;
    return 0;
}

/*!
 * Close an instance of the driver. If this descriptor is associated with a
 * transport, close the transport instance.
 *
 * @param inode The inode of the device file.
 * @param filp The context of the open instance.
 * @return Returns zero on success, negative on failure.
 */
static int
picoif_release( struct inode *inode,
                struct file *filp )
{
    struct picoif_context *ctx = filp->private_data;

    /* If this is a transport instance, close it. */
    if ( ctx )
        ctx->module->ops->close_instance( ctx->module, ctx );

    return 0;
}

/*!
 * Asynchronous write operation for picoIf. We don't actually do any
 * asynchronous operations, but this is also used for the writev() system call
 * so we simply complete these operations synchronously.
 *
 * @param iocb The IO control buffer for the request.
 * @param vecs The vector of buffers to write.
 * @param nr_segs The number of entries in the vecs buffer to write.
 * @param offset Not used in this driver.
 * @return Returns the number of bytes written on success, negative on
 * failure.
 */
static ssize_t
picoif_aio_write( struct kiocb *iocb,
                  const struct iovec *vecs,
                  unsigned long nr_segs,
                  loff_t offset )
{
    ssize_t ret = 0;
    struct picoif_context *ctx = iocb->ki_filp->private_data;
    if ( !ctx )
        return -EBADF;

    if ( ctx->module->ops->writev )
        ret = ctx->module->ops->writev( ctx->module, ctx, vecs, nr_segs, 1 );
    else
    {
        /* If we don't have a writev method for the transport, then emulate it
         * with a series of normal writes. */
        unsigned seg;
        ssize_t total = 0;
        const struct iovec *vec;

        for ( seg = 0; seg < nr_segs; ++seg )
        {
            vec = &vecs[ seg ];
            ret = picoif_write( iocb->ki_filp, vec->iov_base, vec->iov_len,
                                &offset );
            total += ret;
            if ( ret < vec->iov_len )
                goto out;
        }

        ret = ret >= 0 ? total : ret;
    }
out:
    return ret;
}

/*!
 * Asynchronous read operation for picoIf. We don't actually do any
 * asynchronous operations, but this is also used for the readv() system call
 * so we simply complete these operations synchronously.
 *
 * @param iocb The IO control buffer for the request.
 * @param vecs The vector of buffers to read into.
 * @param nr_segs The number of entries in the vecs buffer to read.
 * @param offset Not used in this driver.
 * @return Returns the number of bytes read on success, negative on
 * failure.
 */
static ssize_t
picoif_aio_read( struct kiocb *iocb,
                 const struct iovec *vecs,
                 unsigned long nr_segs,
                 loff_t offset )
{
    ssize_t ret = 0;
    struct picoif_context *ctx = iocb->ki_filp->private_data;
    if ( !ctx )
        return -EBADF;

    if ( ctx->module->ops->readv )
        ret = ctx->module->ops->readv( ctx->module, ctx, vecs, nr_segs, 1 );
    else
    {
        unsigned seg;
        ssize_t total = 0;
        const struct iovec *vec;

        /* If we don't have a readv method for the transport, then emulate it
         * with a series of normal reads. */
        for ( seg = 0; seg < nr_segs; ++seg )
        {
            vec = &vecs[ seg ];
            ret = picoif_read( iocb->ki_filp, vec->iov_base, vec->iov_len,
                               &offset );
            total += ret;
            if ( ret < vec->iov_len )
                goto out;
        }

        ret = ret >= 0 ? total : ret;
    }
out:
    return ret;
}

/*!
 * Write to the open instance. For non-transport file descriptors this is an
 * invalid operation. For transport instances, use the context to find the
 * module that handles the instance and call its write() method.
 *
 * @param filp The context of the open instance.
 * @param buf The data to write into the transport instance.
 * @param len The number of bytes to write.
 * @param loff The offset to write into the file. This is ignored in this
 * driver.
 * @return Returns the number of bytes written on success, negative on
 * failure. It is possible that this may successfully return less than len
 * bytes.
 */
static ssize_t
picoif_write( struct file *filp,
              const char __user *buf,
              size_t len,
              loff_t *loff )
{
    struct picoif_context *ctx = filp->private_data;
    ssize_t ret;
    struct picoif_buf pbuf = {
        .ubuf       = ( char __user * )buf,
        .is_user    = 1,
    };

    if ( !ctx )
        return -EBADF;

again:
    if ( !( filp->f_flags & O_NONBLOCK ) &&
         wait_event_interruptible( ctx->writeq,
                                   ctx->module->ops->can_write( ctx->module,
                                   ctx ) ) )
    {
        ret = -ERESTARTSYS;
        goto out;
    }

    ret = ctx->module->ops->write( ctx->module, ctx, &pbuf, len );

    /* If the resource is temporarily unavailable and we are a blocking
     * transport, then do it again until it succeeds. */
    if ( -EAGAIN == ret && !( filp->f_flags & O_NONBLOCK ) )
        goto again;

    if ( !ret && ( filp->f_flags & O_NONBLOCK ) )
        ret = -EAGAIN;

out:
    return ret;
}

/*!
 * Read frm the open instance. For non-transport file descriptors this is an
 * invalid operation. For transport instances, use the context to find the
 * module that handles the instance and call its read() method.
 *
 * @param filp The context of the open instance.
 * @param buf The data to read from the transport instance.
 * @param len The number of bytes to read.
 * @param loff The offset to read from the file. This is ignored in this
 * driver.
 * @return Returns the number of bytes read on success, negative on
 * failure. It is possible that this may successfully return less than len
 * bytes.
 */
static ssize_t
picoif_read( struct file *filp,
             char __user *buf,
             size_t len,
             loff_t *loff )
{
    struct picoif_context *ctx = filp->private_data;
    ssize_t ret;
    struct picoif_buf pbuf = {
        .ubuf       = buf,
        .is_user    = 1,
    };

    if ( !ctx )
        return -EBADF;

again:
    if ( !( filp->f_flags & O_NONBLOCK ) &&
         wait_event_interruptible( ctx->readq,
                                   ctx->module->ops->can_read( ctx->module,
                                   ctx ) ) )
    {
        ret = -ERESTARTSYS;
        goto out;
    }

    ret = ctx->module->ops->read( ctx->module, ctx, &pbuf, len );

    /* If the resource is temporarily unavailable and we are a blocking
     * transport, then do it again until it succeeds. */
    if ( -EAGAIN == ret && !( filp->f_flags & O_NONBLOCK ) )
        goto again;

    if ( !ret && ( filp->f_flags & O_NONBLOCK ) )
        ret = -EAGAIN;

out:
    return ret;
}

/*!
 * Simple open function for the picoIf mmap() implementation. This is only
 * used for logging.
 *
 * @param vma The vma request is being mapped.
 */
static void
picoif_vma_open( struct vm_area_struct *vma )
{
    PRINTD( COMPONENT_PICOIF, DBG_TRACE, "mmap pa %lx to va %lx",
            vma->vm_pgoff << PAGE_SHIFT, vma->vm_start );
}

/*!
 * Simple close function for the picoIf mmap() implementation. This is only
 * used for logging.
 *
 * @param vma The vma request is being unmapped.
 */
static void
picoif_vma_close( struct vm_area_struct *vma )
{
    PRINTD( COMPONENT_PICOIF, DBG_TRACE, "mmap, close mapping of pa %lx",
            vma->vm_pgoff << PAGE_SHIFT );
}

/*!
 * VM operations for the picoIf mmap implementation. This functions do nothing
 * special and are only used for logging and debug.
 */
static struct vm_operations_struct picoif_vm_ops = {
    .open   = picoif_vma_open,
    .close  = picoif_vma_close,
};

/*!
 * Map a range of physical memory into the user process. This creates a
 * mapping from the beginning of the physical memory address (0) + the
 * requested offset. The user can effectively map the whole physical address
 * space with this call but we have no way of knowing what the physical memory
 * setup is at this point and certainly no way of knowing which memory is
 * shared between the host processor and the picoArray(s). It is in fact
 * possible that there is no shared memory and that this will only map host
 * memory, but there is no way for the driver to be able to detect this. For
 * example, for PC202, this driver may be running on the ARM or an external
 * host, so there may or may not shared memory.
 *
 * This function effectively reimplements a mapping of /dev/mem with the
 * exception that all memory accesses should be uncached. This is required to
 * avoid stalls and race conditions when communicating through shared memory.
 *
 * @param filp The context of the open instance.
 * @param vma The VMA for the requested operation.
 * @return Returns zero on success, negative on failure.
 */
static int
picoif_mmap( struct file *filp,
             struct vm_area_struct *vma )
{
    size_t size = vma->vm_end - vma->vm_start;
    int ret;

    /* Ensure that the mapping is non-cached and non-buffered. */
    vma->vm_page_prot = pgprot_noncached( vma->vm_page_prot );

    ret = -EAGAIN;
    if ( remap_pfn_range( vma, vma->vm_start, vma->vm_pgoff,
                          size, vma->vm_page_prot ) )
    {
        PRINTD( COMPONENT_PICOIF, DBG_WARN,
                "remap_pfn_range() failed (%u bytes @0x%08x)", size,
                vma->vm_start );
        goto out;
    }

    vma->vm_ops = &picoif_vm_ops;
    picoif_vma_open( vma );

    ret = 0;
out:
    return ret;
}

/*!
 * Poll a transport type. It is only valid to poll on a transport file
 * descriptor. Polling a non-transport file descriptor will return -EINVAL.
 *
 * @param filp The file descriptor being polled.
 * @param pollt The poll table for the request.
 * @return Returns a mask of the poll events on success, negative on failure.
 */
static unsigned int
picoif_poll( struct file *filp,
             struct poll_table_struct *pollt )
{
    struct picoif_context *ctx = filp->private_data;
    struct picoif_module *mod;
    unsigned int ret = 0;

    if ( !ctx )
        goto out;

    mod = ctx->module;

    poll_wait( filp, &ctx->writeq, pollt );
    poll_wait( filp, &ctx->readq, pollt );

    if ( mod->ops->can_write( mod, ctx ) )
        ret |= POLLOUT;
    if ( mod->ops->can_read( mod, ctx ) )
        ret |= POLLIN;

out:
    return ret;
}

int
picoif_reset( void )
{
    unsigned i;
    int ret = 0;
    struct picoarray *pa;

    PRINTD( COMPONENT_PICOIF, DBG_TRACE, "picoif: reset all devices" );

    /* Reset each device in sequence. */
    for ( i = 0; i < PICOIF_MAX_DEVICES; ++i )
    {
        pa = picoif_core.devices[ i ];
        if ( pa )
        {
            PRINTD( COMPONENT_PICOIF, DBG_TRACE, "resetting device %u",
                    i );
            ret = pa->ops->reset( pa );
            if ( ret )
                break;
        }
    }

    return ret;
}
EXPORT_SYMBOL( picoif_reset );

int
picoif_start_all( void )
{
    unsigned i;
    int ret = 0;
    struct picoarray *pa;

    PRINTD( COMPONENT_PICOIF, DBG_TRACE, "picoif: start all devices" );

    /* Sync the devices. */
    for ( i = 0; i < PICOIF_MAX_DEVICES; ++i )
    {
        pa = picoif_core.devices[ i ];
        if ( pa )
        {
            PRINTD( COMPONENT_PICOIF, DBG_TRACE, "syncing device %u", i );
            ret = pa->ops->sync( pa );
            if ( ret )
                break;
        }
    }

    /* Start each device in sequence. The master is the only device we need
     * to do this with but it is easier and quicker to do all devices rather
     * than find the master. */
    for ( i = 0; i < PICOIF_MAX_DEVICES; ++i )
    {
        pa = picoif_core.devices[ i ];
        if ( pa )
        {
            PRINTD( COMPONENT_PICOIF, DBG_TRACE, "starting device %u", i );
            ret = pa->ops->start( pa );
            if ( ret )
                break;
        }
    }

    return ret;
}
EXPORT_SYMBOL( picoif_start_all );

int
picoif_stop_all( void )
{
    unsigned i;
    int ret = 0;
    struct picoarray *pa;

    PRINTD( COMPONENT_PICOIF, DBG_TRACE, "picoif: stop all devices" );

    /* Start each device in sequence. The master is the only device we need
     * to do this with but it is easier and quicker to do all devices rather
     * than find the master. */
    for ( i = 0; i < PICOIF_MAX_DEVICES; ++i )
    {
        pa = picoif_core.devices[ i ];
        if ( pa )
        {
            PRINTD( COMPONENT_PICOIF, DBG_TRACE, "stopping device %u",
                    i );
            ret = pa->ops->stop( pa );
            if ( ret )
                break;
        }
    }

    return ret;
}
EXPORT_SYMBOL( picoif_stop_all );

struct picoarray *
picoif_get_device( unsigned dev_num )
{
    if ( dev_num >= PICOIF_MAX_DEVICES )
    {
        PRINTD( COMPONENT_PICOIF, DBG_WARN,
                "picoif: get_device, device out of range (%u)",
                dev_num );
        return NULL;
    }

    return picoif_core.devices[ dev_num ];
}

int
picoif_config_read( unsigned dev_num,
                    u16 caeid,
                    u16 address,
                    u16 count,
                    u16 *buf )
{
    struct picoarray *pa = picoif_get_device( dev_num );
    int ret = -EINVAL;

    if ( !pa )
        goto out;

    ret = pa->ops->config_read( pa, caeid, address, count, buf );

out:
    return ret;
}
EXPORT_SYMBOL( picoif_config_read );

int
picoif_config_write( unsigned dev_num,
                     u16 caeid,
                     u16 address,
                     u16 count,
                     u16 *buf )
{
    struct picoarray *pa = picoif_get_device( dev_num );
    int ret = -EINVAL;

    if ( !pa )
        goto out;

    ret = pa->ops->config_write( pa, caeid, address, count, buf );

out:
    return ret;
}
EXPORT_SYMBOL( picoif_config_write );

int
picoif_pa_load( unsigned dev_num,
                u32 *buf,
                struct scatterlist *sgl )
{
    struct picoarray *pa = picoif_get_device( dev_num );
    int ret = -EINVAL;

    if ( !pa )
        goto out;

    ret = pa->ops->pa_load( pa, buf, sgl );

out:
    return ret;
}
EXPORT_SYMBOL( picoif_pa_load );

int
picoif_register_write( unsigned dev_num,
                       unsigned reg_id,
                       u32 value )
{
    struct picoarray *pa = picoif_get_device( dev_num );
    struct pico_resource *reg;
    int ret = -EINVAL;

    if ( !pa )
        goto out;

    ret = -EBUSY;
    reg = pa->ops->get_resource( pa, PICO_RES_GPR, reg_id, 0 );
    if ( !reg )
        goto out;

    ret = pa->ops->register_write( pa, reg, value );

    pa->ops->put_resource( pa, reg );

out:
    return ret;
}
EXPORT_SYMBOL( picoif_register_write );

int
picoif_register_read( unsigned dev_num,
                      unsigned reg_id,
                      u32 *value )
{
    struct picoarray *pa = picoif_get_device( dev_num );
    struct pico_resource *reg;
    int ret = -EINVAL;

    if ( !pa )
        goto out;

    ret = -EBUSY;
    reg = pa->ops->get_resource( pa, PICO_RES_GPR, reg_id, 0 );
    if ( !reg )
        goto out;

    ret = pa->ops->register_read( pa, reg, value );

    pa->ops->put_resource( pa, reg );

out:
    return ret;
}
EXPORT_SYMBOL( picoif_register_read );

/*!
 * Handle a register read/write request ioctl. This is largely just a wrapper
 * around picoif_register_{read,write}() and doing the address space
 * copying.
 *
 * @param user_req The request structure containing the register ID, device
 * number and value.
 * @param write Boolean to indicate that the operation is a write.
 * @return Returns zero on success, negative on failure.
 */
static int
picoif_ioctl_reg_req( struct picoif_reg_req __user *user_req,
                      int write )
{
    struct picoif_reg_req reg_req;
    int ret;

    ret = copy_from_user( &reg_req, user_req, sizeof( reg_req ) );
    if ( ret )
        goto out;

    ret = write ? picoif_register_write( reg_req.dev, reg_req.reg_id,
                                              reg_req.value ) :
                  picoif_register_read( reg_req.dev, reg_req.reg_id,
                                             &reg_req.value );

    if ( !write && !ret )
    {
        ret = copy_to_user( user_req, &reg_req, sizeof( reg_req ) );
        if ( ret )
            goto out;
    }

out:
    return ret;
}

/*!
 * Handle a multiple configure register write request ioctl. This 
 * function will allocate cached/uncached memory depending on whether a DMA
 * is available, and provide both the virtual and phyiscal addresses to the
 * device layer so that both DMA and non DMA transfers are supported.
 *
 * @param user_req The request structure containing the device number,
 *  number of writes to make to the configuration write port and the
 *  values to write.
 * @param write Boolean to indicate that the operation is a write.
 * @return Returns zero on success, negative on failure.
 */
static int
picoif_ioctl_multi_reg_req( struct picoif_multi_reg_req __user *user_req,
                            int write )
{
    struct picoif_multi_reg_req multi_reg_req;
    int ret;
    int transfer_count = 0;
    u32 *bounce = NULL;
    struct scatterlist sgl;
    struct picoarray *pa;
    int do_dma;

    ret = copy_from_user( &multi_reg_req, user_req, sizeof( multi_reg_req ) );
    if ( ret )
        goto out;

    if ( !write )
    {
        ret = -EINVAL;
        goto out;
    }

    pa = picoif_get_device( multi_reg_req.dev );
    if ( !pa )
    {
        ret = -EINVAL;
        goto out;
    }

    do_dma = picoarray_has_feature( pa, PICOARRAY_HAS_DMA_LOAD );

    /* Allocate a buffer to perform the read/write to/from to avoid doing each
     * word with copy_{to,from}_user calls. */
    ret = -ENOMEM;
    if ( do_dma )
        bounce = dma_alloc_coherent( NULL, PICOIF_MAX_TRANSFER_SIZE,
               &sgl.dma_address, GFP_KERNEL );
    else
        bounce = kmalloc( PICOIF_MAX_TRANSFER_SIZE, GFP_KERNEL );
    if ( !bounce )
        goto out;

    /* Copy data from userspace */
    if ( multi_reg_req.count > 0 )
    {
        unsigned words_so_far=0;
        unsigned num_transfers = 
          ((multi_reg_req.count-1)/(PICOIF_MAX_TRANSFER_SIZE/sizeof( u32 )))+1;
        unsigned i=0;

        for(i = 0; i < num_transfers; i++)
        {
            sgl.length = multi_reg_req.count-words_so_far;
            if ( sgl.length >= (PICOIF_MAX_TRANSFER_SIZE / sizeof( u32 )))
                sgl.length = (PICOIF_MAX_TRANSFER_SIZE / sizeof( u32 ));

            ret = copy_from_user( bounce, &multi_reg_req.buf[words_so_far],
                          sgl.length * sizeof( u32 ));
            words_so_far += sgl.length;

            if ( ret )
                goto out;

            ret = picoif_pa_load( multi_reg_req.dev, bounce, &sgl );

            /* If successful, set the number of words read/written and copy
               the data back if it was a config read. */
            if ( ret > 0 )
            { 
                transfer_count += ret;
                ret = 0;
            }
        }
    }

    /* Copy the request back containing the number of words written/read. */
    if ( copy_to_user( user_req, &multi_reg_req, sizeof( multi_reg_req ) ) )
        ret = -EFAULT;

out:
    if ( !ret )
        ret = transfer_count;

    if ( bounce )
    {
        if ( do_dma )
            dma_free_coherent( NULL, PICOIF_MAX_TRANSFER_SIZE, bounce,
                 sgl.dma_address );
        else
            kfree( bounce );
    }

    return ret;
}

/*!
 * Handle a config bus read/write request ioctl. This is largely just a wrapper
 * around picoif_config_{read,write}() and doing the address space
 * copying.
 *
 * @param user_req The request structure containing the device number, CAEID,
 * AE address, count and pointer to data buffer.
 * @param write Boolean to indicate that the operation is a write.
 * @return Returns zero on success, negative on failure.
 */
static int
picoif_ioctl_cfg_req( struct picoif_cfg_req __user *user_req,
                      int write )
{
    struct picoif_cfg_req cfg_req;
    u16 *bounce = NULL;
    int ret;

    /* Copy the request from userspace. */
    ret = copy_from_user( &cfg_req, user_req, sizeof( cfg_req ) );
    if ( ret )
        goto out;

    ret = -EINVAL;
    if ( cfg_req.count > ( ( 1 << 16 ) - 1 ) )
    {
        PRINTD( COMPONENT_PICOIF, DBG_WARN,
                "config request count exceeds address space size" );
        goto out;
    }

    if ( cfg_req.count + cfg_req.ae_addr > ( ( 1 << 16 ) - 1 ) )
    {
        PRINTD( COMPONENT_PICOIF, DBG_WARN,
                "config request start + count exceeds address space size" );
        goto out;
    }

    /* Allocate a buffer to perform the read/write to/from to avoid doing each
     * word with copy_{to,from}_user calls. */
    ret = -ENOMEM;
    bounce = kmalloc( cfg_req.count * sizeof( u16 ), GFP_KERNEL );
    if ( !bounce )
        goto out;

    if ( write )
    {
        ret = copy_from_user( bounce, cfg_req.buf,
                              cfg_req.count * sizeof( u16 ) );
        if ( ret )
            goto out;
    }

    /* Do the actual read/write. */
    ret = write ? picoif_config_write( cfg_req.dev, cfg_req.caeid,
                                       cfg_req.ae_addr, cfg_req.count,
                                       bounce ) :
                  picoif_config_read( cfg_req.dev, cfg_req.caeid,
                                      cfg_req.ae_addr, cfg_req.count,
                                      bounce );

    /* If successful, set the number of words read/written and copy the data
     * back if it was a config read. */
    if ( ret >= 0 )
    {
        cfg_req.count = ret;
        if ( !write )
        {
            ret = copy_to_user( cfg_req.buf, bounce,
                                cfg_req.count * sizeof( u16 ) );
            if ( ret )
                goto out;
        }
        ret = 0;
    }

    /* Copy the request back containing the number of words written/read. */
    if ( copy_to_user( user_req, &cfg_req, sizeof( cfg_req ) ) )
        ret = -EFAULT;

out:
    if ( bounce )
        kfree( bounce );
    return ret;
}


unsigned
picoif_num_devices( void )
{
    unsigned i;
    unsigned num_devices = 0;

    for ( i = 0; i < PICOIF_MAX_DEVICES; ++i )
        if ( picoif_core.devices[ i ] )
            ++num_devices;

    return num_devices;
}
EXPORT_SYMBOL( picoif_num_devices );

ssize_t
picoif_transport_generic_read( struct picoif_context *ctx,
                               u8 *buf,
                               size_t len )
{
    struct picoif_buf pbuf = {
        .kbuf       = buf,
        .is_user    = 0,
    };
    return ctx->module->ops->read( ctx->module, ctx, &pbuf, len );
}
EXPORT_SYMBOL( picoif_transport_generic_read );

ssize_t
picoif_transport_generic_write( struct picoif_context *ctx,
                                const u8 *buf,
                                size_t len )
{
    struct picoif_buf pbuf = {
        .kbuf       = ( void * )buf,
        .is_user    = 0,
    };
    return ctx->module->ops->write( ctx->module, ctx, &pbuf, len );
}
EXPORT_SYMBOL( picoif_transport_generic_write );

void
picoif_transport_generic_close( struct picoif_context *ctx )
{
    ctx->module->ops->close_instance( ctx->module, ctx );
}
EXPORT_SYMBOL( picoif_transport_generic_close );

/*!
 * Handle an ioctl request to get the number of devices in the system. This is
 * just a wrapper around picoif_num_devices() with the address space
 * copying.
 *
 * @param res The destination to write the number of devices to.
 * @return Returns zero on success, negative on failure.
 */
static int
picoif_ioctl_num_devices( void __user *res )
{
    unsigned num_devices = picoif_num_devices();
    return copy_to_user( res, &num_devices, sizeof( num_devices ) );
}

/*!
 * Get a picoif module by the module name.
 *
 * @param name The name of the module.
 * @return Returns a pointer to the module on success, NULL on failure.
 */
static struct picoif_module *
picoif_get_module( const char *name )
{
    unsigned i;
    struct picoif_module *mod = NULL;

    for ( i = 0; i < PICOIF_MAX_MODULES; ++i )
        if ( picoif_core.modules[ i ] &&
             !strcmp( picoif_core.modules[ i ]->name, name ) )
        {
            mod = picoif_core.modules[ i ];
            break;
        }

    return mod;
}

/*!
 * Handle a new transport request. This attempts to open a new transport
 * described in req and associated it with the file descriptor filp.
 *
 * @param filp The file descriptor to be associated with the transport.
 * @param req The request for the new transport.
 * @return Returns zero on success, non-zero on failure.
 */
static int
picoif_new_trans( struct file *filp,
                  void __user *req )
{

    struct picoif_new_trans_req trans_req;
    int ret = copy_from_user( &trans_req, req, sizeof( trans_req ) );
    char module_name[ 32 ];
    char *p;
    struct picoif_module *mod;
    struct picoif_context *ctx;
    struct picoif_buf tbuf;

    if ( ret )
        goto out;

    PRINTD( COMPONENT_PICOIF, DBG_TRACE, "create new transport \"%s\"",
            trans_req.description );

    ret = -EINVAL;
    p = strchr( trans_req.description, '(' );
    if ( !p )
    {
        PRINTD( COMPONENT_PICOIF, DBG_WARN,
                "invalid transport description \"%s\"",
                trans_req.description );
        goto out;
    }

    /* Extract the module name. */
    strncpy( module_name, trans_req.description,
             ( p - trans_req.description ) );
    module_name[ p - trans_req.description ] = '\0';

    mod = picoif_get_module( module_name );
    if ( !mod )
    {
        PRINTD( COMPONENT_PICOIF, DBG_WARN,
                "module %s not registered", module_name );
        goto out;
    }

    tbuf.ubuf = trans_req.params;
    tbuf.is_user = 1;
    ctx = mod->ops->create_trans_instance( mod, trans_req.description,
                                           &tbuf );
    if ( IS_ERR( ctx ) )
    {
        PRINTD( COMPONENT_PICOIF, DBG_WARN,
                "failed to create transport \"%s\"", trans_req.description );
        ret = PTR_ERR( ctx );
        goto out;
    }

    filp->private_data = ctx;

    ret = 0;

out:
    return ret;
}

/*!
 * Handle an ioctl request for the driver. If this is a transport instance
 * then the ioctl request should be rerouted to the appropriate transport
 * module.
 *
 * @param inode The inode of the device file.
 * @param filp The file context for this instance.
 * @param cmd The ioctl type.
 * @param arg The argument for the ioctl. This may be casted to the
 * appropriate type depending on the ioctl type.
 */
static int
picoif_ioctl( struct inode *inode,
              struct file *filp,
              unsigned int cmd,
              unsigned long arg )
{
    int ret = -ENOTTY;

    switch ( cmd )
    {
        case PICOIF_IOC_RESET:
            ret = picoif_reset();
            break;

        case PICOIF_IOC_STARTALL:
            ret = picoif_start_all();
            break;

        case PICOIF_IOC_STOPALL:
            ret = picoif_stop_all();
            break;

        case PICOIF_IOC_CFG_READ:
            ret = picoif_ioctl_cfg_req( ( void __user * )arg, 0 );
            break;

        case PICOIF_IOC_CFG_WRITE:
            ret = picoif_ioctl_cfg_req( ( void __user * )arg, 1 );
            break;

        case PICOIF_IOC_REG_READ:
            ret = picoif_ioctl_reg_req( ( void __user * )arg, 0 );
            break;

        case PICOIF_IOC_REG_WRITE:
            ret = picoif_ioctl_reg_req( ( void __user * )arg, 1 );
            break;

        case PICOIF_IOC_MULTI_CFG_WRITE:
            ret = picoif_ioctl_multi_reg_req( ( void __user * )arg, 1 );
            break;

        case PICOIF_IOC_NUMDEV:
            ret = picoif_ioctl_num_devices( ( void __user * )arg );
            break;

        case PICOIF_IOC_NEW_TRANS:
            ret = picoif_new_trans( filp, ( void __user * )arg );
            break;

        default:
            PRINTD( COMPONENT_PICOIF, DBG_WARN,
                    "picoif: invalid ioctl cmd (%u)", cmd );
            break;
    }

    return ret;
}

int
picoif_register_dev( struct picoarray *pa )
{
    int ret = -EINVAL;
    if ( !pa )
        goto out;

    /* Check that we have the required methods filled in. */
    if ( !pa->ops ||
         !pa->resources ||
         !pa->ops->get_device_type ||
         !pa->ops->reset ||
         !pa->ops->start ||
         !pa->ops->stop ||
         !pa->ops->sync ||
         !pa->ops->dma_to_device ||
         !pa->ops->dma_from_device ||
         !pa->ops->config_read ||
         !pa->ops->config_write ||
         !pa->ops->register_read ||
         !pa->ops->register_write ||
         !pa->ops->reset ||
         !pa->ops->get_resource ||
         !pa->ops->get_procif_resource ||
         !pa->ops->put_resource ||
         !pa->ops->put_procif_resource ||
         !pa->ops->add_irq_handler ||
         !pa->ops->remove_irq_handler ||
         !pa->ops->destructor ||
         !pa->ops->pa_load )
    {
        PRINTD( COMPONENT_PICOIF, DBG_ERROR,
                "device %u does not have all required fields/methods",
                pa->dev_num );
        goto out;
    }

    ret = -ENOMEM;
    if ( pa->dev_num >= PICOIF_MAX_DEVICES )
    {
        PRINTD( COMPONENT_PICOIF, DBG_TRACE,
                "pA[ %u ]: device number out of range", pa->dev_num );
        goto out;
    }

    ret = 0;
    picoif_core.devices[ pa->dev_num ] = pa;

out:
    return ret;
}

void
picoif_unregister_dev( struct picoarray *pa )
{
    if ( !pa )
        return;

    if ( !picoif_core.devices[ pa->dev_num ] )
    {
        PRINTD( COMPONENT_PICOIF, DBG_WARN,
                "pA[ %u ]: device not registered", pa->dev_num );
        return;
    }

    picoif_core.devices[ pa->dev_num ] = NULL;
}

int
picoif_register_module( struct picoif_module *module )
{
    unsigned i;
    int ret = -EINVAL;

    if ( !module )
        goto out;

    if ( !module->name ||
         !module->ops ||
         !module->tmethods ||
         !module->ops->create_trans_instance ||
         !module->ops->close_instance ||
         !module->ops->destructor ||
         !module->ops->write ||
         !module->ops->read ||
         !module->ops->can_read ||
         !module->ops->can_write )
    {
        PRINTD( COMPONENT_PICOIF, DBG_ERROR,
                "transport module \"%s\" does not have all required "
                "fields/methods", module->name ?: "no name" );
        goto out;
    }

    ret = -ENOMEM;
    for ( i = 0; i < PICOIF_MAX_MODULES; ++i )
        if ( !picoif_core.modules[ i ] )
        {
            picoif_core.modules[ i ] = module;
            ret = 0;
            break;
        }

out:
    return ret;
}

void
picoif_unregister_module( struct picoif_module *module )
{
    unsigned i;

    if ( !module )
        goto out;

    for ( i = 0; i < PICOIF_MAX_MODULES; ++i )
        if ( picoif_core.modules[ i ] == module )
        {
            picoif_core.modules[ i ] = NULL;
            break;
        }

out:
    return;
}

/*!
 * Remove all registered devices from the core.
 */
static void
picoif_remove_devices( void )
{

    unsigned i;
    for ( i = 0; i < PICOIF_MAX_DEVICES; ++i )
    {
        struct picoarray *pa = picoif_core.devices[ i ];
        if ( pa )
            pa->ops->destructor( pa );
    }
}

/*!
 * Remove all registered modules from the core.
 */
static void
picoif_remove_modules( void )
{

    unsigned i;

    for ( i = 0; i < PICOIF_MAX_MODULES; ++i )
    {
        struct picoif_module *module = picoif_core.modules[ i ];
        if ( module )
            module->ops->destructor( module );
    }
}

#ifdef CONFIG_PICOIF_DMAPOOL

/*!
 * DMA coherent buffer control.  We'll keep at most PICOIF_MAX_NUM_DMA_BUFFERS
 * buffers in a pool ready to be re-used.  The size of each of these buffers
 * will be exactly PICOIF_DMA_POOL_BUFFER_SIZE bytes.  Initially we have no
 * buffers in the pool.  When the client asks for a buffer and we can't use a
 * buffer from the pool, then we allocate it using dma_alloc_coherent().  If
 * the buffer request asks for less than PICOIF_DMA_POOL_BUFFER_SIZE bytes,
 * then we round it up to that.  If the request is for more, then we allocate
 * exactly the size asked for.
 *
 *    Luckily, when the client returns the buffer it lets us know how big a buffer
 * it asked for.  Therefore, we know that if the size that was asked for was less
 * than PICOIF_DMA_POOL_BUFFER_SIZE we actually allocated a buffer that was exactly
 * PICOIF_DMA_POOL_BUFFER_SIZE bytes.  This is suitable for keeping in the pool.
 * Anything bigger than that should be freed immediately.
 *
 *    If the client asks for more than PICOIF_MAX_NUM_DMA_BUFFERS and returns them,
 * we only keep that number in the pool.  After the pool has that many buffers we
 * just free any more that are passed to us.
 */
void* picoif_alloc_coherent(size_t size, dma_addr_t *dma_handle, int flag)
{
    unsigned long  flags;
    void          *vaddr = NULL;

    /* Check first for a pool buffer that we have already allocated */
    spin_lock_irqsave(&(picoif_core.dma_buffers_lock), flags);

    if ((size <= PICOIF_DMA_POOL_BUFFER_SIZE) && (picoif_core.active_dma_buffers != 0))
    {
        picoif_core.active_dma_buffers--;

        *dma_handle = picoif_core.dma_buffers[picoif_core.active_dma_buffers].phys;
        vaddr       = picoif_core.dma_buffers[picoif_core.active_dma_buffers].virt;

        PRINTD( COMPONENT_PICOIF, DBG_TRACE, "load pool p=%p v=%p s=%d\n",
            *dma_handle, vaddr, size);
    }

    spin_unlock_irqrestore(&(picoif_core.dma_buffers_lock), flags);

    /* If no suitable pool buffer then ask the kernel for a new one */
    if (vaddr == NULL)
    {
        /* Always allocate at least the pool buffer size, that way any returned buffer
         * that's less than this can always be pooled safely.
         */
        if (size < PICOIF_DMA_POOL_BUFFER_SIZE)
        {
            size = PICOIF_DMA_POOL_BUFFER_SIZE;
        }

        vaddr = dma_alloc_coherent(NULL, size, dma_handle, flag);

        PRINTD( COMPONENT_PICOIF, DBG_TRACE, "alloc dma p=%p v=%p s=%d\n",
            *dma_handle, vaddr, size);
    }

    return vaddr;
}
EXPORT_SYMBOL(picoif_alloc_coherent);

/*!
 * picoif DMA memory pool de-allocation function   This function will
 * hold on to a DMA coherent buffer for later re-use.
 */
void picoif_free_coherent(size_t size, void *vaddr, dma_addr_t dma_handle)
{
    unsigned long flags;

    spin_lock_irqsave(&(picoif_core.dma_buffers_lock), flags);

    /* If we can return the buffer to the pool then do so */
    if ((size <= PICOIF_DMA_POOL_BUFFER_SIZE) && (picoif_core.active_dma_buffers < PICOIF_MAX_NUM_DMA_BUFFERS))
    {
        PRINTD( COMPONENT_PICOIF, DBG_TRACE, "save pool p=%p v=%p s=%d\n",
            dma_handle, vaddr, size);

        picoif_core.dma_buffers[picoif_core.active_dma_buffers].phys = dma_handle;
        picoif_core.dma_buffers[picoif_core.active_dma_buffers].virt = vaddr;

        picoif_core.active_dma_buffers++;

        vaddr = NULL;
    }

    spin_unlock_irqrestore(&(picoif_core.dma_buffers_lock), flags);

    /* If buffer was not returned to the pool then free it */
    if (vaddr != NULL)
    {
        if (size < PICOIF_DMA_POOL_BUFFER_SIZE)
        {
            size = PICOIF_DMA_POOL_BUFFER_SIZE;
        }

        PRINTD( COMPONENT_PICOIF, DBG_TRACE, "free dma p=%p v=%p s=%d\n",
            dma_handle, vaddr, size);

        dma_free_coherent(NULL, size, vaddr, dma_handle);
    }
}
EXPORT_SYMBOL(picoif_free_coherent);
#endif /* CONFIG_PICOIF_DMAPOOL */

/*!
 * picoif module initialisation. This function registers the misc device
 * and performs any initialisation necessary to allow users to use the
 * services provided.
 *
 * @return Returns zero on success, negative on failure.
 */
static int
picoif_init( void )
{
#ifdef CONFIG_PICOIF_DMAPOOL
    int i;
#endif
    int ret = misc_register( &picoif_core.miscdev );
    if ( ret )
    {
        printk( KERN_INFO "failed to register picoif miscdevice\n" );
        goto out;
    }

#ifdef CONFIG_DEBUG_FS
    ret = -ENOMEM;
    picoif_core.debugfs_dir = debugfs_create_dir( "picoif", NULL );
    if ( !picoif_core.debugfs_dir )
    {
        printk( KERN_INFO "failed to create debugfs entry\n" );
        goto debugfs_dir_failed;
    }

    ret = -ENOMEM;
    picoif_core.debug_log =
        pc_debug_create_log_file( picoif_core.debugfs_dir );
    if ( !picoif_core.debug_log )
    {
        printk( KERN_INFO "failed to create debugfs log file\n" );
        goto debugfs_log_failed;
    }
#endif /* CONFIG_DEBUG_FS */

#ifdef CONFIG_PICOIF_DMAPOOL
    spin_lock_init(&(picoif_core.dma_buffers_lock));
    picoif_core.active_dma_buffers = 0;
    
    /* Preallocate the buffers to ensure that apps that run before the
     * transport channels are opened can't fragment it before we get a
     * chance to allocate the DMA buffers
     */
    for (i=0; i<PICOIF_MAX_NUM_DMA_BUFFERS; i++)
    {
        dma_addr_t dma_handle;
        void       *vaddr;
        
        vaddr = dma_alloc_coherent(NULL, PICOIF_DMA_POOL_BUFFER_SIZE, &dma_handle, GFP_KERNEL);
        if (vaddr)
        {
            picoif_core.dma_buffers[picoif_core.active_dma_buffers].phys = dma_handle;
            picoif_core.dma_buffers[picoif_core.active_dma_buffers].virt = vaddr;
            picoif_core.active_dma_buffers++;
        }
    }
#endif /* CONFIG_PICOIF_DMAPOOL */

#ifdef CONFIG_PICOIF_PC202
    ret = pc202_init();
    if ( ret )
    {
        printk( KERN_INFO "pc202 registration failed\n" );
        goto internal_fail;
    }
#endif /* CONFIG_PICOIF_PC202 */

#ifdef CONFIG_PICOIF_PC203
    ret = pc203_init();
    if ( ret )
    {
        printk( KERN_INFO "pc203 registration failed\n" );
        goto internal_fail;
    }
#endif /* CONFIG_PICOIF_PC203 */

#ifdef CONFIG_PICOIF_PC302
    ret = pc302_init();
    if ( ret )
    {
        printk( KERN_INFO "pc302 registration failed\n" );
        goto internal_fail;
    }
#endif /* CONFIG_PICOIF_PC302 */

    ret = gpr_interrupt_init();
    if ( ret )
    {
        printk( KERN_INFO
                "gpr interrupt transport module registration failed\n" );
        goto internal_fail;
    }

    ret = dma_init();
    if ( ret )
    {
        printk( KERN_INFO
                "DMA transport module registration failed\n" );
        goto internal_fail;
    }

    ret = hwif_init();
    if ( ret )
    {
        printk( KERN_INFO
                "HwIF transport module registration failed\n" );
        goto internal_fail;
    }

    ret = hwif2_init();
    if ( ret )
    {
        printk( KERN_INFO
                "HwIF2 transport module registration failed\n" );
        goto internal_fail;
    }

    ret = 0;
    goto out;

internal_fail:
debugfs_log_failed:
    debugfs_remove_recursive( picoif_core.debugfs_dir );
debugfs_dir_failed:
    misc_deregister( &picoif_core.miscdev );
out:
    if ( ret )
    {
        picoif_remove_devices();
        picoif_remove_modules();
        pc_debug_close();
    }
    return ret;
}

/*!
 * Module exit function for picoif. This function releases any resources
 * and performs any cleanup necessary.
 */
static void
picoif_exit( void )
{
    picoif_remove_devices();
    picoif_remove_modules();

#ifdef CONFIG_PICOIF_DMAPOOL
    while (picoif_core.active_dma_buffers > 0)
    {
        size_t      size;
        void*       vaddr;
        dma_addr_t  dma_handle;

        picoif_core.active_dma_buffers--;

        size       = PICOIF_DMA_POOL_BUFFER_SIZE;
        vaddr      = picoif_core.dma_buffers[picoif_core.active_dma_buffers].virt;
        dma_handle = picoif_core.dma_buffers[picoif_core.active_dma_buffers].phys;

        /* The debug ring buffer is about to be discarded so PRINTD is pointless. */
        printk("cleanup dma p=%d v=%p s=%d\n", dma_handle, vaddr, size);

        dma_free_coherent(NULL, size, vaddr, dma_handle);
    }
#endif /* CONFIG_PICOIF_DMAPOOL */
    
    misc_deregister( &picoif_core.miscdev );

#ifdef CONFIG_DEBUG_FS
    debugfs_remove_recursive( picoif_core.debugfs_dir );
#endif /* CONFIG_DEBUG_FS */
    pc_debug_close();
}

module_init( picoif_init );
module_exit( picoif_exit );
MODULE_AUTHOR( "Jamie Iles" );
MODULE_LICENSE( "GPL" );
