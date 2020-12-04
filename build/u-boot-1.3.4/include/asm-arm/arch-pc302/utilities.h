/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file utilities.h
* \brief Definitions for some useful PC302 functions.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/
#ifndef PC302_UTILITIES_H
#define PC302_UTILITIES_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */


/* Prototypes--------------------------------------------------------------- */
/*!
 *
 * Read an integer value from a register.
 *
 * \param The address to read from
 * \return The value read
 */
__inline unsigned int pc302_read_from_register (const unsigned int address);

/*!
 *
 * Write an integer value to a register
 *
 * \param The address to write to
 * \param The value to write.
 */
__inline void pc302_write_to_register (const unsigned int address,
                                       const unsigned int value);

/*!
 *
 * Read the device identification code from the axi2cfg block
 *
 * \return The value read
 */
unsigned int pc302_read_device_id (void);

/*!
 *
 * Read the device revision from the axi2cfg block
 *
 * \return The value read
 */
unsigned int pc302_read_device_revision (void);

/*!
 *
 * Read the die id number from the fuse block
 *
 * \param die_number Pointer to an array
 */
void pc302_read_die_id_number (unsigned int * die_number);

/*!
 *
 * Return the state of the Reduced MII (RMII) enabled bit in the
 * axi2cfg block, system config register.
 *
 * \return 0 - RMII not enabled
 *         1 - RMII enabled
 */
unsigned int pc302_get_rmii_enabled (void);

/*!
 * Read a number of 16 bit words from the PC302 axi2cfg.
 *
 * @param caeid The CAEID of the AE to read from.
 * @param address The start address in the AE to begin reading from.
 * @param count The number of 16 bit words to read.
 * @param[out] data The buffer to store the data in.
 * @return Returns the number of words read on success, negative on failure.
 */
int
pc302_config_read( u16 caeid,
                   u16 address,
                   u16 count,
                   u16 *data );

/*!
 * Write a number of 16 bit words to the PC302 axi2cfg.
 *
 * @param caeid The CAEID of the AE to write to.
 * @param address The start address in the AE to begin writing to.
 * @param count The number of 16 bit words to write.
 * @param[in] data The buffer to write from.
 * @return Returns the number of words written on success, negative on failure.
 */
int
pc302_config_write( u16 caeid,
                    u16 address,
                    u16 count,
                    u16 *data );

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_UTILITIES_H */
