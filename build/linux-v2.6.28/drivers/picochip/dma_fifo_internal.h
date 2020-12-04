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
 * \file dma_fifo_internal.h
 * \brief Common DMA FIFO managing functions
 *
 * This file contains common functions for managing the DMA FIFOs.
 */

#ifndef __PICOIF_DMA_FIFO_INTERNAL__
#define __PICOIF_DMA_FIFO_INTERNAL__

struct dma_fifo_t;

/*!
 * Create and initialise the DMA FIFO.
 *
 * @param fifo The FIFO structure to initialise
 * @param size The  size (in bytes) of the FIFO to allocate
 * @return Returns 0 on success, or ENOMEM if the allocation fails
 */
int
dma_fifo_create( struct dma_fifo_t **fifo,
                 size_t size );

/*!
 * Free the DMA FIFO.
 *
 * @param fifo The FIFO structure to destroy
 */
void
dma_fifo_destroy( struct dma_fifo_t *fifo );

/*!
 * Determine the start address of the FIFO read pointer
 * 
 * @param fifo The FIFO structure
 * @return The physical address of the first byte to read
*/ 
dma_addr_t
dma_fifo_get_readptr( struct dma_fifo_t *fifo );

/*!
 * Determine the start address of the write pointer
 *
 * @param fifo The FIFO structure
 * @return The physical address of the first byte to write
*/
dma_addr_t
dma_fifo_get_writeptr( struct dma_fifo_t *fifo );

/*!
 * Determine the number of bytes for reading in the DMA FIFO without taking
 * wrapping into account.
 *
 * @param fifo The FIFO structure
 * @return The number filled bytes to the end of the buffer or the write
 *         pointer which ever occurs first
 */
unsigned
dma_fifo_used( struct dma_fifo_t *fifo );

/*!
 * Determine how many bytes are free for writing in the FIFO
 *
 * @param fifo The FIFO structure
 * @return The number free bytes to the end of the buffer or the read
 *         pointer which ever occurs first
 */
int
dma_fifo_space( struct dma_fifo_t *fifo );

/*
 * Read up to the specified number of bytes from the FIFO
 *
 * @param fifo The FIFO structure
 * @param buf The buffer to write the data into
 * @param size The number of bytes to write
 * @param offset The offset to apply to the buf
 * @return 0 on success, negative if the buffer copy fails
 */
unsigned
dma_fifo_get( struct dma_fifo_t *fifo,
              struct picoif_buf *buf,
              size_t size,
              unsigned offset );

/*!
 * Put the specified number of bytes into the FIFO
 *
 * @param fifo The FIFO structure
 * @param buf The buffer to write the data into
 * @param size The number of bytes to write
 * @param offset The offset to apply to the buf
 * @return 0 on success, negative if the buffer copy fails
 *
 */
unsigned
dma_fifo_put( struct dma_fifo_t *fifo,
              struct picoif_buf *buf,
              size_t size,
              unsigned offset );

/*!
 * Add the specified number of bytes into the FIFO transfered by the DMA
 * engine
 *
 * @param fifo The FIFO structure
 * @param size The number of bytes to add
 *
 */
void
dma_fifo_add_transfer( struct dma_fifo_t *fifo,
                       size_t size );

/*!
 * Remove the specified number of bytes from the FIFO transfered by the DMA
 * engine
 *
 * @param fifo The FIFO structure
 * @param size The number of bytes to remove
 *
 */
void
dma_fifo_remove_transfer( struct dma_fifo_t *fifo,
                          size_t size );

#endif /* !__PICOIF_DMA_FIFO_INTERNAL__ */
