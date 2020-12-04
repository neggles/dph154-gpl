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
 * \file procif.h
 * \brief ProcIf access functions.
 *
 * This file provides a set of functions for using the procif in picoArray
 * devices and should be used to implement the device files for devices that
 * contain a procif.
 *
 */

#ifndef __PICOIF_PROCIF_H__
#define __PICOIF_PROCIF_H__

/*! Register offset for the config bus read port. */
#define PROCIF_REG_CFG_RD_OFFSET   ( 0x78 )

/*! Register offset for the config bus write port. */
#define PROCIF_REG_CFG_WR_OFFSET   ( 0x7C )

/*! The offset of the ITM register in the procif. */
#define PC202_PROCIF_ITM_OFFSET    ( 0x70 )

/*!
 * Read a number of 16 bit words from a picoArray procif.
 *
 * @param procif_base The base address of the procif.
 * @param aeid The CAEID of the AE to read from.
 * @param ae_addr The address to begin reading from within the AE.
 * @param[out] buf The buffer to store the results in.
 * @param count The number of 16 bit words to read.
 * @return Returns the number of words read on success, negative on failure.
 */
int procif_config_read( void __iomem *procif_base,
                        u16 aeid,
                        u16 ae_addr,
                        u16 *buf,
                        u16 count );

/*!
 * Write a number of 16 bit words to a picoArray procif.
 *
 * @param procif_base The base address of the procif.
 * @param aeid The CAEID of the AE to write to.
 * @param ae_addr The address to begin writing to within the AE.
 * @param[in] buf The buffer to read the words from.
 * @param count The number of 16 bit words to write.
 * @return Returns the number of words written on success, negative on failure.
 */
int procif_config_write( void __iomem *procif_base,
                         u16 aeid,
                         u16 ae_addr,
                         u16 *buf,
                         u16 count );

/*!
 * Write a register in the procif.
 *
 * @param base The I/O memory base address of the procif.
 * @param offset The offset of the register in bytes.
 * @param value The value of the register to write.
 * @return Returns zero on success, negative on failure.
 */
int
procif_reg_write( void __iomem *base,
                  unsigned offset,
                  u32 value );

/*!
 * Read a register in the procif.
 *
 * @param base The I/O memory base address of the procif.
 * @param offset The offset of the register in bytes.
 * @param[out] value The address to store the register value in.
 * @return Returns zero on success, negative on failure.
 */
int
procif_reg_read( void __iomem *base,
                 unsigned offset,
                 u32 *value );

#endif /* __PICOIF_PROCIF_H__ */
