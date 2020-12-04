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
 * \file picoif_internal.h
 * \brief Internal operations for picoIf driver implementation.
 *
 * This file defines functions in the picoIf core used for registering
 * transport modules, and accessing core module services.
 *
 * \mainpage
 *
 * This document describes the architecture and implementation of the picoIf
 * Linux kernel driver. This document also describes the kernel API available
 * to users.
 *
 * \section Overview
 *
 * picoIf is a library and driver to allow users to configure, control and
 * communicate with picoArray devices. This documentation describes the driver
 * implementation and kernel space API but the driver also provides a
 * userspace interface through POSIX system calls, and debugfs.
 *
 * For more details, see:
 * \li \ref moduleDocs
 * \li \ref paAbstraction
 * \li \ref kernelAPI
 * \li \ref debugging
 */
#ifndef __PICOIF_PICOIF_INTERNAL_H__
#define __PICOIF_PICOIF_INTERNAL_H__

#include <asm/io.h>
#include "picoarray.h"

struct picoif_module;

/*! Accessor for struct picoif_buf to get the user buffer address. */
#define ubuf    buf_u.u_buf
/*! Accessor for struct picoif_buf to get the kernel buffer address. */
#define kbuf    buf_u.k_buf

/*!
 * \brief Structure to abstract user/kernel buffer spaces.
 */
struct picoif_buf
{
    /*! The buffer pointers. u_buf if .is_user = 1, k_buf otherwise. */
    union
    {
        void __user *u_buf;
        void        *k_buf;
    } buf_u;

    /*! Boolean flag to indicate the buffer is a userspace buffer. */
    int             is_user;
};

/*!
 * Copy the contents of a struct picoif_buf into a kernel buffer. This is
 * used to allow functions to work for either userspace or kernel space
 * interfaces with a simple wrapper around them to create the struct
 * picoif_buf.
 *
 * @param dst The buffer to copy the data into.
 * @param src The buffer to copy the data from.
 * @param offset The byte offset in src to begin copying from.
 * @param nbytes The number of bytes to copy from src to dst.
 * @return Returns zero on success, negative on failure.
 */
int
picoif_buf_copy_from( void *dst,
                      struct picoif_buf *src,
                      unsigned offset,
                      size_t nbytes );

/*!
 * Copy the contents of a kernel buffer into a struct picoif_buf. This is
 * used to allow functions to work for either userspace or kernel space
 * interfaces with a simple wrapper around them to create the struct
 * picoif_buf.
 *
 * @param dst The buffer to copy the data into.
 * @param src The buffer to copy the data from.
 * @param offset The byte offset in dst to begin copying to.
 * @param nbytes The number of bytes to copy from src to dst.
 * @return Returns zero on success, negative on failure.
 */
int
picoif_buf_copy_to( struct picoif_buf *dst,
                    void *src,
                    unsigned offset,
                    size_t nbytes );

/*!
 * Register a new device with picoif.
 *
 * @param pa The new device to register with the module.
 * @return Returns zero on success, non-zero on failure.
 */
int
picoif_register_dev( struct picoarray *pa );

/*!
 * Unregister a device from the module.
 *
 * @param pa The device to unregister from the module.
 */
void
picoif_unregister_dev( struct picoarray *pa );

/*!
 * Register a new picoif module with the driver. This module should be
 * used to provide new transport methods. Modules are unregistered with
 * picoif_unregister_module().
 *
 * @param module The module to register with the driver.
 * @return Returns zero on success, negative on failure.
 */
int
picoif_register_module( struct picoif_module *module );

/*!
 * Unregister a transport module from the driver.
 *
 * @param module The module to unregister.
 */
void
picoif_unregister_module( struct picoif_module *module );

/*!
 * Get a device given its logical device number.
 *
 * @param dev_num The logical device number of the picoArray to get.
 * @return Returns a pointer to the device on success, NULL on failure.
 */
struct picoarray *
picoif_get_device( unsigned dev_num );

/*!
 * Perform a 32 bit write to the picoArray taking endianness into account.
 *
 * @param value The value to write.
 * @param addr The virtual IO address to write to.
 */
static inline void
picoif_out32( u32 value,
              void __iomem *addr )
{
#ifdef __LITTLE_ENDIAN
    iowrite32( value, addr );
#else /* __LITTLE_ENDIAN */
    out_be32( addr, value );
#endif /* __LITTLE_ENDIAN */
}

/*!
 * Perform a 32 bit read from the picoArray taking endianness into account.
 *
 * @param addr The virtual IO address to read from.
 * @return Returns the 32 bit value.
 */
static inline u32
picoif_in32( void __iomem *addr )
{
#ifdef __LITTLE_ENDIAN
    return ioread32( addr );
#else /* __LITTLE_ENDIAN */
    return in_be32( addr );
#endif /* __LITTLE_ENDIAN */
}

/*!
 * Ask picoif for a DMA coherent buffer.
 *
 * @param size The number of bytes to allocate.
 * @param dma_handle Pointer to location where the bus address will be saved
 * @param flag Flags to pass to dma_alloc_buffer if required
 * @return Returns the virtual address of the buffer or NULL.
 */
void* picoif_alloc_coherent(size_t size, dma_addr_t *dma_handle, int flag);

/*!
 * Pass a DMA coherent buffer back to picoif.
 *
 * @param size The number of bytes that were allocated for the buffer.
 * @param vaddr The virtual address of the buffer.
 * @param dma_handle The bus address of the buffer.
 */
void picoif_free_coherent(size_t size, void *vaddr, dma_addr_t dma_handle);

#endif /* !__PICOIF_PICOIF_INTERNAL_H__ */
