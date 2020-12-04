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
 * \file picoif_module.h
 * \brief picoIf module and context definition.
 *
 * This file defines a base class for picoIf modules and should be inherited
 * from using the container_of() macro to implement new transport modules.
 *
 * \page moduleDocs Transport Module Overview
 *
 * \section introduction Introduction
 *
 * picoIf implements a modular transport system that allows transport modules
 * to be easily added and removed without breaking the ABI between userspace
 * and kernelspace. To achieve this, picoIf defines a picoif_module base class
 * from which transport modules may derive from and create a new transport
 * family. Transport modules support a transport family - there may be several
 * methods supported, but they have a common core. For example in some older
 * SRD's there is the HwIf transport family that uses an IrqMux entity in the
 * SRD to provide picoArray initiated uplink DMA and also simple interrupts.
 * Both of these methods would be supported in a single module as it would be
 * easier to manage the IrqMux in a single place.
 *
 * \section newtransports Opening new transports
 *
 * Transports are opened with the create_trans() method of the transport
 * module and this method takes a textual description of the transport and a
 * pointer to some transport specific parameters. The transport description is
 * encoded in the form:
 *
 * \code
 * module_name(method)
 * \endcode
 *
 * For example, the GPR interrupt transport module provides two transports:
 *  \li "gpr_interrupt(with_values)"
 *  \li "gpr_interrupt(without_values)"
 * Where the first method samples the GPR at interrupt time and the second
 * only records a count of the interrupts.
 *
 * The transport parameters should be stored in a structure that is available
 * to userspace and kernelspace and will be copied into the transport module
 * by the module itself and the values interpreted.
 *
 * \section readingAndWriting Reading and writing
 *
 * By default, all transports will be opened in blocking mode but may be put
 * into non-blocking mode at any time. However, the read and write methods for
 * the transport module must always be non-blocking and return -EAGAIN if the
 * read/write would block. This makes it easier for kernel API users to
 * multiplex multiple transport instances and allows the blocking code to be
 * factored out which is often common across multiple transports.
 *
 * The picoif_context structure contains wait queues for both reading and
 * writing. If the upper layers, such as the kernel API users or the
 * picoif_write() and picoif_read() functions need to block, then they can
 * wait on these queues. To simplify the logic for these functions, there are
 * also can_read() and can_write() methods that return non-zero if the
 * operation can take place without blocking and are therefore suitable for
 * use in the wait_event_interruptible macros(). For example, a blocking read
 * (minus error checking and return values) might look like:
 *
 * \code
 *  struct picoif_context *ctx;
 *  struct picoif_buf buffer;
 *  wait_event_interruptible( ctx->readq,
 *                            ctx->module->ops->can_read( ctx->module, ctx ) );
 *  ctx->module->ops->read( ctx->module, ctx, &buffer, transfer_len );
 * \endcode
 *
 * \section new_modules Adding new modules
 *
 * To add a new transport module, an implementation file for the transport
 * family should be created, for example dma.c. This file contains the
 * derived structure - dma, and has the picoif_module structure embedded in it.
 * The DMA transport methods are also defined, and these can get a dma pointer
 * from a pointer to the embedded picoif_module structure with the
 * container_of() macro:
 *
 * \code
 *  struct picoif_module *module;
 *  struct dma *dma_module =
 *      container_of( module, struct picoif_module, module );
 * \endcode
 *
 * When a new context is created with the create_trans() method, the transport
 * code should create any per-instance structure and store this in the
 * private_data field of the picoif_context structure for use in the read()
 * and write() methods etc.
 */
#ifndef __PICOIF_PICOIF_MODULE_H__
#define __PICOIF_PICOIF_MODULE_H__

#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include "picoif_internal.h"
#include <linux/picochip/picoif.h>

struct picoif_module;


/*!
 * Create a new picoIf context and initialise it. This should only be called
 * by the modules when creating new instances and not users.
 *
 * @param module The module owning the context.
 * @param private_data Private data for the module transport instance.
 * @return Returns a pointer to the new context on success, NULL on failure.
 */
static inline struct picoif_context *
picoif_new_ctx( struct picoif_module *module,
                void *private_data )
{
    struct picoif_context *ctx = kmalloc( sizeof( *ctx ), GFP_KERNEL );
    if ( !ctx )
        goto out;

    ctx->module = module;
    ctx->private_data = private_data;

    init_waitqueue_head( &ctx->readq );
    init_waitqueue_head( &ctx->writeq );

out:
    return ctx;
}

/*!
 * Free an existing context. This should only be called by modules when
 * releasing transport instances and not users.
 *
 * @param ctx The context to free.
 */
static inline void
picoif_free_ctx( struct picoif_context *ctx )
{
    kfree( ctx );
}

/*!
 * \brief Operations for the transport module.
 */
struct picoif_module_ops
{
    /*!
     * Create a new transport instance. This function should create a new
     * transport instance and on success, the file descriptor associated with
     * the this context will only be used for this transport.
     *
     * @param module The module to handle the transport.
     * @param description The name of the module and transport type in the
     * form "module_name(transport_type)".
     * @param params Any extra transport specific parameters should be encoded
     * in this parameter.
     * @return Returns a pointer to a new context on success, NULL on failure.
     */
    struct picoif_context *
        ( *create_trans_instance )( struct picoif_module *mod,
                                    const char *description,
                                    struct picoif_buf *params );

    /*!
     * Close an open transport instance.
     *
     * @param module The module handling the transport.
     * @param context The transport instance to close.
     */
    void ( *close_instance )( struct picoif_module *mod,
                              struct picoif_context *context );

    /*!
     * Transport module destructor. This will be called on termination of
     * picoif and should close all transport instances and free any
     * resources.
     *
     * @param module The module being destroyed.
     */
    void ( *destructor )( struct picoif_module *module );

    /*!
     * Write method for the transport. This is used for writing userspace data
     * into the transport. This function must be non-blocking and must return
     * -EAGAIN if there is no space to write any data. If a higher layer
     *  wishes to block then it can wait on the contexts write queue.
     *
     * @param module The module handling the transport.
     * @param ctx The context that is writing the data.
     * @param data The data to write.
     * @param len The length of the data to write in bytes.
     * @return Returns the number of bytes written on success, negative on
     * failure.
     */
    ssize_t ( *write )( struct picoif_module *module,
                        struct picoif_context *ctx,
                        struct picoif_buf *data,
                        size_t len );

    /*!
     * Read method for the transport. This is used for reading transport data
     * into userspace. This function must be non-blocking and must return
     * -EAGAIN if there is no data to be read. If a higher layer wishes to
     * block then it can wait on the contexts read queue.
     *
     * @param module The module handling the transport.
     * @param ctx The context that is reading the data.
     * @param data The buffer to store the data in.
     * @param len The length of the data to read in bytes.
     * @return Returns the number of bytes read on success, negative on
     * failure.
     */
    ssize_t ( *read )( struct picoif_module *module,
                       struct picoif_context *ctx,
                       struct picoif_buf *data,
                       size_t len );

    /*!
     * Vectored write method for the transport. This is used for writing
     * userspace data into the transport. This function must be non-blocking
     * and must return -EAGAIN if there is no space to write any data. If a
     * higher layer wishes to block then then can wait on the contexts write
     * queue.
     *
     * Transports do not need to implement writev if there is no significant
     * performance improvement by doing so. For example, a transport that
     * simply reads from a FIFO in memory would gain little speedup, but a
     * transport that writes into a FIFO before starting a DMA transfer could
     * reduce the number of DMA transfers by writing the segments in one go.
     *
     * @param module The module handling the transport.
     * @param ctx The context that is writing the data.
     * @param vecs The vectors of data to write into the transport.
     * @param nr_segs The number of entries in the IO vector.
     * @param from_user Boolean flag to indicate that the vectors point to
     * userspace buffers and must be copied across the address spaces.
     * @return Returns the number of bytes written into the transport on
     * success, negative on failure.
     */
    ssize_t ( *writev )( struct picoif_module *module,
                         struct picoif_context *ctx,
                         const struct iovec *vecs,
                         unsigned nr_segs,
                         int from_user );

    /*!
     * Vectored read method for the transport. This is used for reading data
     * from the transport into userspace. This function must be non-blocking
     * and must return -EAGAIN if there is no data to read. If a higher layer
     * wishes to block then then can wait on the contexts read queue.
     *
     * Transports do not need to implement readv if there is no significant
     * performance improvement by doing so. For example, a transport that
     * simply reads from a FIFO in memory would gain little speedup, but a
     * transport that DMAs into a FIFO on demand and then copies into the
     * buffer could perform the transfers in one go.
     *
     * @param module The module handling the transport.
     * @param ctx The context that is reading the data.
     * @param vecs The vectors of data to read the transport data into.
     * @param nr_segs The number of entries in the IO vector.
     * @param from_user Boolean flag to indicate that the vectors point to
     * userspace buffers and must be copied across the address spaces.
     * @return Returns the number of bytes read from the transport on
     * success, negative on failure.
     */
    ssize_t ( *readv )( struct picoif_module *module,
                        struct picoif_context *ctx,
                        const struct iovec *vecs,
                        unsigned nr_segs,
                        int to_user );

    /*!
     * Check if a write with the given context will not block.
     *
     * @param module The module the context belongs to.
     * @param ctx The context being queuried.
     * @return Returns 1 if the write will not block, 0 otherwise.
     */
    int ( *can_write )( struct picoif_module *module,
                        struct picoif_context *ctx );

    /*!
     * Check if a read with the given context will not block.
     *
     * @param module The module the context belongs to.
     * @param ctx The context being queuried.
     * @return Returns 1 if the read will not block, 0 otherwise.
     */
    int ( *can_read )( struct picoif_module *module,
                       struct picoif_context *ctx );
};

/*!
 * \brief picoif transport module.
 *
 * This defines a picoif module used for implementing transport methods.
 * A transport module may implement several transport methods using a common
 * base. For example a HwIf module may implement HwIf uplink DMA and IrqMux
 * interrupts. These should be combined in the same module as they share
 * resources and implementation.
 */
struct picoif_module
{
    /*! The name of the module. */
    const char                 *name;

    /*! The operations for the module. */
    struct picoif_module_ops   *ops;

    /*! An array of transport method names. */
    const char                 **tmethods;
};

#endif /* !__PICOIF_PICOIF_MODULE_H__ */
