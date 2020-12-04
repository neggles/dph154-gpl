/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file dma.h
 *
 * \brief Public picoIf DMA transport method kernel API.
 *  \addtogroup kernelAPI
 *  @{
 *   \addtogroup DMA
 *    @{
 */

#ifndef __PICOIF_PUBLIC_DMA_H__
#define __PICOIF_PUBLIC_DMA_H__
#include <linux/types.h>
#include <linux/picochip/picoif.h>

struct picoif_context;

/*!
 * Open a new instance of the DMA DL transport.
 *
 * @param dev_num The logical device number of the picoArray to use.
 * @param dma_chan The identifier of the DMA channel to use. This should be
 * taken from the picoifDMAId_t enumeration for the appropriate device type.
 * @param buf_size The size of buffer in bytes to allocate for this channel
 * @return Returns a context on success, or an ERR_PTR encoded error on
 * failure.
 */
struct picoif_context *
picoif_dma_open_dl( unsigned dev_num,
                    int dma_chan,
                    size_t buf_size );

/*!
 * Open a new instance of the DMA UL transport.
 *
 * @param dev_num The logical device number of the picoArray to use.
 * @param dma_chan The identifier of the DMA channel to use. This should be
 * taken from the picoifDMAId_t enumeration for the appropriate device type.
 * @param buf_size The size of buffer in bytes to allocate for this channel
 * @return Returns a context on success, or an ERR_PTR encoded error on
 * failure.
 */
struct picoif_context *
picoif_dma_open_ul( unsigned dev_num,
                    int dma_chan,
                    size_t buf_size );

/*!
 * Read from the DMA channel up to the size of data specified
 *
 * @param ctx The open context of the transport.
 * @param buf The buffer to write the DMA'd data into.
 * @param len The length of the buffer in bytes.
 * @return Returns the number of bytes read into the buffer on success,
 * negative on failure. If no data is available to read, the -EAGAIN will be
 * returned.
 */
static inline ssize_t
picoif_dma_read( struct picoif_context *ctx,
                 u8 *buf,
                 size_t len )
{
    return picoif_transport_generic_read( ctx, buf, len );
}

/*!
 * Write to the data supplied in a kernel mapped buffer into a DMA transport.
 *
 * @param ctx The open context of the transport.
 * @param buf The buffer containing the data to transfer by DMA.
 * @param len The length of the buffer in bytes.
 * @return Returns the number of bytes written to the transport on success,
 * negative on failure. If there is no space in the transport to buffer the
 * data the -EAGAIN is returned.
 */
static inline ssize_t
picoif_dma_write( struct picoif_context *ctx,
                  u8 *buf,
                  size_t len )
{
    return picoif_transport_generic_write( ctx, buf, len );
}

/*!
 * Close an open instance of a DMA transport.
 *
 * @param ctx The context to close.
 */
static inline void
picoif_dma_close( struct picoif_context *ctx )
{
    picoif_transport_generic_close( ctx );
}

/*! @} */
/*! @} */

#endif /* !__PICOIF_PUBLIC_DMA_H__ */
