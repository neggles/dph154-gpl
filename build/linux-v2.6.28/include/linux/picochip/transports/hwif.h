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
 * \file hwif.h
 *
 * \brief Public picoIf HwIf DMA transport method kernel API.
 *  \addtogroup kernelAPI
 *  @{
 *   \addtogroup HwIf
 *    @{
 */

#ifndef __PICOIF_PUBLIC_HWIF_H__
#define __PICOIF_PUBLIC_HWIF_H__
#include <linux/types.h>
#include <linux/picochip/picoif.h>

struct picoif_context;

/*!
 * \brief Structure for the storage of the context in the HwIF transport
 */
struct picoif_hwif
{
    unsigned dev_num;       /*!< The device number to use in the transport. */
    int int_clear_gpr;      /*!< The interrupt clear GPR to use. */
};

/*! Opaque pointer for the picoIf HwIf context. */
typedef struct picoif_hwif * picoif_hwif_t;

/*!
 * Initialise a context (one per device) to support all HwIF UL transports.
 * Once finished with, the context should be destroyed with
 * picoifHwIF_DestroyContext().
 *
 * \param dev_num The logical device number to use for the transport.
 * \param int_clear_gpr The GPR for clearing HwIF UL interrupts. This value
 * should be taken from the picoIfGPRId_t enumeration.
 * \return Returns a HwIF context.
 */
picoif_hwif_t
picoif_hwif_init( unsigned dev_num,
                  int int_clear_gpr );


/*!
 * Destroy a HwIf context used for creating transports. This may be safely
 * destroyed once all transports using the context have been opened.
 *
 * \param context The context to destroy.
 */
void
picoif_hwif_destroy_context( picoif_hwif_t context );

/*!
 * Create a new instance of a HwIF uplink DMA transport. This transport will
 * allow DMA transfers from a picoArray DMA channel into a user buffer using
 * the HwIF (Traditional ISR) mechanism. On success, a file descriptor is
 * returned which may be read to transfer the data. When read(2) is called on
 * the file descriptor, a transfer of all available bytes up to the specified
 * number is made. By default this transport is configured in blocking mode
 * but can be converted to non-blocking using the fcntl(2) command.
 * 
 * This function requires an interrupt clear GPR to be specified
 * and this is done (first) by calling function picoifHwIf_Init().
 *
 * \param[in] hwif_context The HwIF context created by picoifHwIF_Init()
 * \param interrupt_number The interrupt number to initiate a new transfer.
 * \param dma_channel The DMA channel in the picoArray to use for the
 *      transport. This should be taken from the picoIfDMAId_t enumeration.
 * \param dma_status_gpr DMA status GPR. This value should be taken from the
 * picoIfGPRId_t enumeration.
 * \param dma_count_gpr DMA count GPR. This value should be taken from the
 * picoIfGPRId_t enumeration.
 * \param buffer_size The internal buffer size used for DMA transfers in bytes.
 *      This should be large enough such that it does not often fill up and
 *      reduce throughput.
 * @return Returns a context on success, or an ERR_PTR encoded error on
 * failure.
 */
struct picoif_context *
picoif_hwif_dmaul_open( const picoif_hwif_t hwif_context,
                        unsigned interrupt_number,
                        int dma_channel,
                        int dma_status_gpr,
                        int dma_count_gpr,
                        size_t buffer_size );

/*!
 * Read from the HwIf DMA channel up to the size of data specified
 *
 * @param ctx The open context of the transport.
 * @param buf The buffer to write the DMA'd data into.
 * @param len The length of the buffer in bytes.
 * @return Returns the number of bytes read into the buffer on success,
 * negative on failure. If no data is available to read, the -EAGAIN will be
 * returned.
 */
static inline ssize_t
picoif_hwif_dmaul_read( struct picoif_context *ctx,
                        u8 *buf,
                        size_t len )
{
    return picoif_transport_generic_read( ctx, buf, len );
}

/*!
 * Close an open instance of a HwIf DMA transport.
 *
 * @param ctx The context to close.
 */
static inline void
picoif_hwif_dmaul_close( struct picoif_context *ctx )
{
    picoif_transport_generic_close( ctx );
}

/*! @} */
/*! @} */

#endif /* !__PICOIF_PUBLIC_HWIF_H__ */
