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
 * \file gpr_interrupt.h
 *
 * \brief Public picoIf GPR interrupt transport method kernel API.
 *  \addtogroup kernelAPI
 *  @{
 *   \addtogroup GPRIrq
 *    @{
 */

#ifndef __PICOIF_PUBLIC_GPR_INTERRUPT_H__
#define __PICOIF_PUBLIC_GPR_INTERRUPT_H__
#include <linux/types.h>
#include <linux/picochip/picoif.h>

struct picoif_context;

/*!
 * Open a new instance of the GPR interrupt transport.
 *
 * @param dev_num The logical device number of the picoArray to use.
 * @param irq_num The identifier of the GPR number to use. This should be
 * taken from the picoifGPRId_t enumeration.
 * @return Returns a context on success, or an ERR_PTR encoded error on
 * failure.
 */
struct picoif_context *
picoif_gpr_irq_ul( unsigned dev_num,
                   int irq_num );

/*!
 * Open a new instance of the GPR interrupt transport. This is like
 * picoif_gpr_irq_ul() but rather than the read method returning the number of
 * interrupts raised, the buffer will contain the values of the GPR at the
 * time the interrupt was raised.
 *
 * @param dev_num The logical device number of the picoArray to use.
 * @param irq_num The identifier of the IRQ source to use. This should be
 * taken from the picoifIRQId_t enumeration.
 * @return Returns a context on success, or an ERR_PTR encoded error on
 * failure.
 */
struct picoif_context *
picoif_gpr_irq_ul_with_values( unsigned dev_num,
                               int irq_num );

/*!
 * Read from the GPR interrupt transport. If the values are not being
 * recorded, the number of interrupts raised since the transport has last read
 * will be encoded in host byte order in the first 4 bytes of the buffer so
 * buf must be at least 4 bytes long. Alternatively, if the values are being
 * recorded, then buf will contain the values of the GPR at interrupt time
 * encoded in host byte order and the number of interrupts raised will be the
 * return value divided by sizeof( u32 ).
 *
 * @param ctx The open context of the transport.
 * @param buf The buffer to write the transport data into.
 * @param len The length of the buffer in bytes.
 * @return Returns the number of bytes read into the buffer on success,
 * negative on failure. If no interrupts have been raised, then -EAGAIN will
 * be returned.
 */
static inline ssize_t
picoif_gpr_irq_ul_read( struct picoif_context *ctx,
                        u8 *buf,
                        size_t len )
{
    return picoif_transport_generic_read( ctx, buf, len );
}

/*!
 * Close an open instance of a GPR interrupt transport.
 *
 * @param ctx The context to close.
 */
static inline void
picoif_gpr_irq_ul_close( struct picoif_context *ctx )
{
    picoif_transport_generic_close( ctx );
}

/*! @} */
/*! @} */

#endif /* !__PICOIF_PUBLIC_GPR_INTERRUPT_H__ */
