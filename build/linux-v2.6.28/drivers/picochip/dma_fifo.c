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
 * \file dma_fifo.c
 * \brief Main file for the common DMA FIFO functions
 *
 * This file implements the DMA FIFO managing for all transports concerned with
 * the picoArray.
 */

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>

#include "picoif_module.h"
#include "dma_fifo_internal.h"
#include "picoif_internal.h"

/*! The maximum number of bytes that can be copied to user in a single transfer */
#define DMA_MAX_TRANSFER_SIZE ( 4096 )

/*!
 * \brief DMA FIFO parameters (one per DMA channel).
 */
struct dma_fifo_t
{
    dma_addr_t physaddr;          /*!< Physical base address of FIFO */
    unsigned char *buf;           /*!< Virtual base address  of FIFO */
    unsigned bufsize;             /*!< FIFO size in bytes */
    unsigned inptr;               /*!< FIFO input offset */
    unsigned outptr;              /*!< FIFO output offset */
    unsigned used;                /*!< FIFO bytes in use */
    spinlock_t lock;              /*!< FIFO access control */
};

__must_check int
dma_fifo_create( struct dma_fifo_t **fifo,
                 size_t size )
{
    *fifo  = kmalloc( sizeof( struct dma_fifo_t ), GFP_KERNEL );
    if ( !(*fifo) )
        return -ENOMEM;
 
#if defined(CONFIG_PICOIF_DMAPOOL)
    (*fifo)->buf = picoif_alloc_coherent( size, 
            &((*fifo)->physaddr), GFP_KERNEL );
#else /* !defined(CONFIG_PICOIF_DMAPOOL) */
    (*fifo)->buf = dma_alloc_coherent( NULL, size, 
            &((*fifo)->physaddr), GFP_KERNEL );
#endif /* CONFIG_PICOIF_DMAPOOL */

    if ( (*fifo)->buf == NULL )
    {
        kfree( *fifo );
        return -ENOMEM;
    }
 
    (*fifo)->bufsize = size;
    (*fifo)->inptr = 0;
    (*fifo)->outptr = 0;
    (*fifo)->used=0;
    
    spin_lock_init( &((*fifo)->lock) );

    return 0;
}

void
dma_fifo_destroy( struct dma_fifo_t *fifo )
{
    if ( fifo->buf != NULL )
#if defined(CONFIG_PICOIF_DMAPOOL)
        picoif_free_coherent( fifo->bufsize, fifo->buf,
               fifo->physaddr );
#else /* !defined(CONFIG_PICOIF_DMAPOOL) */
        dma_free_coherent( NULL, fifo->bufsize, fifo->buf,
               fifo->physaddr );
#endif /* CONFIG_PICOIF_DMAPOOL */

   if ( fifo )
       kfree ( fifo );
}

__must_check dma_addr_t
dma_fifo_get_readptr( struct dma_fifo_t *fifo )
{
    unsigned long flags;
    dma_addr_t sg_dma_address;

    spin_lock_irqsave( &fifo->lock, flags );
    sg_dma_address = fifo->physaddr +fifo->outptr;
    spin_unlock_irqrestore( &fifo->lock, flags );
    return sg_dma_address;
}

__must_check dma_addr_t
dma_fifo_get_writeptr( struct dma_fifo_t *fifo )
{
    unsigned long flags;
    dma_addr_t sg_dma_address;

    spin_lock_irqsave( &fifo->lock, flags );
    sg_dma_address = fifo->physaddr +fifo->inptr;
    spin_unlock_irqrestore( &fifo->lock, flags );
    return sg_dma_address;
}

__must_check unsigned
dma_fifo_used( struct dma_fifo_t *fifo )
{
    unsigned long flags;
    unsigned available=0;

    spin_lock_irqsave( &fifo->lock, flags );

    /* Determine how many bytes are available in the FIFO for retrieval */
    if ( fifo->inptr == fifo->outptr )
    {
        if ( fifo->used == 0 )
            available = 0; /* Nothing to receive */
        else
            available = fifo->bufsize - fifo->outptr;
    }
    else if ( fifo->inptr > fifo->outptr )
    {
        /* Write pointer has not wrapped */
        available = fifo->inptr -fifo->outptr;
    }
    else
    {
        /* Write pointer has wrapped */
        available = fifo->bufsize -fifo->outptr;
    }

    spin_unlock_irqrestore( &fifo->lock, flags );

    return available;
}

__must_check int
dma_fifo_space( struct dma_fifo_t *fifo )
{
    int available;
    unsigned long flags;

    spin_lock_irqsave( &fifo->lock, flags );

    if ( fifo->inptr == fifo->outptr )
    {
        if ( fifo->used == 0 )
            available = fifo->bufsize - fifo->inptr;
        else
            available = 0;
    }
    else if ( fifo->inptr > fifo->outptr )
        available = fifo->bufsize - fifo->inptr;
    else
        available = fifo->outptr - fifo->inptr;

    spin_unlock_irqrestore( &fifo->lock, flags );

    return available;
}

__must_check unsigned
dma_fifo_get( struct dma_fifo_t *fifo,
              struct picoif_buf *buf,
              size_t size,
              unsigned offset)
{
    unsigned ret=0;
    unsigned long flags;
    unsigned available=0;

    spin_lock_irqsave( &fifo->lock, flags );

    /* Determine how many bytes are available in the FIFO for retrieval */
    if ( fifo->inptr == fifo->outptr )
    {
        if ( fifo->used == 0 )
            available = 0; /* Nothing to receive */
        else
            available = fifo->bufsize - fifo->outptr;
    }
    else if ( fifo->inptr > fifo->outptr )
    {
        /* Write pointer has not wrapped */
        available = fifo->inptr -fifo->outptr;
    }
    else
    {
        /* Write pointer has wrapped */
        available = fifo->bufsize -fifo->outptr;
    }

    if ( size > available )
        ret = available;
    else
        ret = size;

    spin_unlock_irqrestore( &fifo->lock, flags );

    /* Copy data to userspace */
    if ( ret > 0 )
    {
        unsigned bytes_so_far=0;
        unsigned transferSize=0;
        unsigned i;

        for(i = 0; i < ((ret -1)/DMA_MAX_TRANSFER_SIZE)+1; i++)
        { 
            transferSize = ret-bytes_so_far;
            if ( transferSize >= DMA_MAX_TRANSFER_SIZE )
                transferSize = DMA_MAX_TRANSFER_SIZE;

            if (picoif_buf_copy_to( buf, 
                   (fifo->buf +fifo->outptr +bytes_so_far), 
                   offset, transferSize) < 0 )
                break;
            bytes_so_far += transferSize;
            offset += transferSize;
        }

        ret = bytes_so_far;
    }

    spin_lock_irqsave( &fifo->lock, flags );
    fifo->outptr += ret;
    fifo->outptr %= fifo->bufsize;
    fifo->used -= ret;
    spin_unlock_irqrestore( &fifo->lock, flags );

    return ret;
}

__must_check unsigned
dma_fifo_put( struct dma_fifo_t *fifo,
              struct picoif_buf *buf,
              size_t size,
              unsigned offset )
{
    unsigned ret=0;
    unsigned available=0;
    unsigned long flags;
    unsigned char *dest=NULL;

    spin_lock_irqsave( &fifo->lock, flags );

    /* Determine the number of free bytes */
    if ( fifo->inptr == fifo->outptr )
    {
        if (fifo->used == 0)
            available = fifo->bufsize -fifo->inptr;
        else
            available = 0;
    }
    else if (fifo->inptr > fifo->outptr)
        available = fifo->bufsize - fifo->inptr;
    else
        available = fifo->outptr -fifo->inptr;

    if ( size > available )
        ret = available;
    else
        ret = size;

    if ( ret == 0 )
    {
        spin_unlock_irqrestore( &fifo->lock, flags );
        return ret;
    }

    dest = fifo->buf + fifo->inptr;
    spin_unlock_irqrestore( &fifo->lock, flags );

    if ( ret > 0 )
        picoif_buf_copy_from( dest, buf, offset, ret );

    spin_lock_irqsave( &fifo->lock, flags );
    fifo->inptr += ret;
    fifo->inptr %= fifo->bufsize;
    fifo->used += ret;

    spin_unlock_irqrestore( &fifo->lock, flags );

    return ret;
}

void
dma_fifo_add_transfer( struct dma_fifo_t *fifo,
                       size_t size )
{
    unsigned long flags;

    spin_lock_irqsave( &fifo->lock, flags );
    fifo->used += size;
    fifo->inptr += size;
    fifo->inptr %= fifo->bufsize;
    spin_unlock_irqrestore( &fifo->lock, flags );
}

void
dma_fifo_remove_transfer( struct dma_fifo_t *fifo,
                          size_t size )
{
    unsigned long flags;

    spin_lock_irqsave( &fifo->lock, flags );
    fifo->used -= size;
    fifo->outptr += size;
    fifo->outptr %= fifo->bufsize;
    spin_unlock_irqrestore( &fifo->lock, flags );
}

