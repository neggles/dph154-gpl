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
 * \file picoif.h
 * \brief Kernel API core functionality for picoIf.
 *
 * This file defines functions for configuration and control of picoArray devices.
 *
 */

/*! @defgroup kernelAPI */

/*!
 *  \addtogroup kernelAPI
 *  @{
 *
 *   \brief This group defines the picoIf kernel API.
 *
 *   \addtogroup kernelAPICore
 *    @{
 */

#ifndef __PICOIF_PICOIF_H__
#define __PICOIF_PICOIF_H__

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/scatterlist.h>

struct picoif_module;

/*!
 * \brief Context for a transport instance opened.
 *
 * This represents a transport instance and points to the module that should
 * handle the transport and the private data for that transport instance.
 */
struct picoif_context
{
    struct picoif_module    *module;        /*!< The module that implements
                                             *  the transport. */

    void                    *private_data;  /*!< Private data for the
                                             *  transport instance. */

    wait_queue_head_t       writeq;         /*!< Wait queue for blocking
                                             *   writes. */

    wait_queue_head_t       readq;          /*!< Wait queue for blocking
                                             *   reads. */
};

/*!
 * Perform a read from the configuration bus.
 *
 * @param dev_num The logical device number to read from.
 * @param caeid The CAEID of the AE to read from.
 * @param address The address inside the AE to begin reading from.
 * @param count The number of 16-bit words to read.
 * @param[out] buf Pointer to an array of u16s to store the data read.
 * @return Returns the number of 16-bit words read on success, negative on
 * failure.
 */
int
picoif_config_read( unsigned dev_num,
                    u16 caeid,
                    u16 address,
                    u16 count,
                    u16 *buf );

/*!
 * Perform a write to the configuration bus.
 *
 * @param dev_num The logical device number to write to.
 * @param caeid The CAEID of the AE to write to.
 * @param address The address inside the AE to begin writing to.
 * @param count The number of 16-bit words to write.
 * @param buf Pointer to an array of u16s to write.
 * @return Returns the number of 16-bit words written on success, negative on
 * failure.
 */
int
picoif_config_write( unsigned dev_num,
                     u16 caeid,
                     u16 address,
                     u16 count,
                     u16 *buf );

/*!
 * Write to a general purpose register (GPR) in a picoArray. It is possible
 * that the reg_id may be a valid register but is being used exclusively for a
 * transport. If this is the case then this function will return -EBUSY and
 * the register will not be written to.
 *
 * If the register does not have any space to write into, then -EAGAIN will
 * be returned.
 *
 * @param dev_num The logical device number of the device to use.
 * @param reg_id The ID of the register to write to.
 * @param value The value to write to the register.
 * @return Returns zero on success, negative on failure.
 */
int
picoif_register_write( unsigned dev_num,
                       unsigned reg_id,
                       u32 value );

/*!
 * Read from a general purpose register (GPR) in a picoArray. It is possible
 * that the reg_id may be a valid register but is being used exclusively for a
 * transport. If this is the case then this function will return -EBUSY and
 * the register will not be read from.
 *
 * If the register does not have any data available to read, then -EAGAIN will
 * be returned.
 *
 * @param dev_num The logical device number of the device to use.
 * @param reg_id The ID of the register to write to.
 * @param[out] value A pointer to the address where the register value should
 * be stored.
 * @return Returns zero on success, negative on failure.
 */
int
picoif_register_read( unsigned dev_num,
                      unsigned reg_id,
                      u32 *value );

/*!
 * Perform a picoArray load by multiple loads to the configuration bus write
 * port.
 *
 * @param dev_num The logical device number to write to.
 * @param buf Pointer to an array of u32s to write.
 * @return Returns the number of 32-bit words written on success, negative on
 * failure.
 */
int
picoif_pa_load( unsigned dev_num,
                u32 *buf,
                struct scatterlist *sgl );

/*!
 * Get the number of devices in the system.
 *
 * @return Returns the number of devices in the system.
 */
unsigned
picoif_num_devices( void );

/*!
 * Reset all of the devices in the system.
 *
 * @return Returns zero on success, non-zero on failure.
 */
int
picoif_reset( void );

/*!
 * Start all devices running.
 *
 * @return returns zero on success, non-zero on failure.
 */
int
picoif_start_all( void );

/*!
 * Stop all devices running.
 *
 * @return returns zero on success, non-zero on failure.
 */
int
picoif_stop_all( void );

/*! @} */

/*!
 * Generic read method for transport instances in kernelspace. This is used
 * to read from a transport instance when the transport type is unknown and
 * the transport specific read function is simply a wrapper around this.
 *
 * @param ctx The context to read from.
 * @param buf The buffer to store the data read into.
 * @param len The maximum number of bytes to read.
 * @return Returns the number of bytes read on success, negative on failure.
 */
ssize_t
picoif_transport_generic_read( struct picoif_context *ctx,
                               u8 *buf,
                               size_t len );

/*!
 * Generic write method for transport instances in kernelspace. This is used
 * to write to a transport instance when the transport type is unknown and
 * the transport specific write function is simply a wrapper around this.
 *
 * @param ctx The context to write to.
 * @param buf The buffer of data to write.
 * @param len The maximum number of bytes to write.
 * @return Returns the number of bytes written on success, negative on
 * failure.
 */
ssize_t
picoif_transport_generic_write( struct picoif_context *ctx,
                                const u8 *buf,
                                size_t len );

/*!
 * Generic close method for a transport instance in kernelspace. This is used
 * to close a transport instance when the transport type is unknown and the
 * transport specific close function is simply a wrapper around this.
 *
 * @param ctx The transport instance to close.
 */
void
picoif_transport_generic_close( struct picoif_context *ctx );

/*! @} */

#endif /* !__PICOIF_PICOIF_H__ */
