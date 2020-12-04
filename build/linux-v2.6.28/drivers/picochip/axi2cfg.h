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
 * \file axi2cfg.h
 * \brief axi2cfg access functions.
 *
 * This file provides a set of functions for using the axi2cfg in picoArray
 * devices and should be used to implement the device files for devices that
 * contain a axi2cfg.
 */

#ifndef __PICOIF_AXI2CFG__
#define __PICOIF_AXI2CFG__

/*! Register offset for the config bus read port (from the axi2cfg1 base
 *  address). */
#define AXI2CFG_REG_PURGE_CFG ( 0x00 )

/*! Register offset for the config bus write port (from the axi2cfg1 base
 *  address). */
#define AXI2CFG_REG_DMAC1_CFG ( 0x04 )

/*! Register offset for the config bus write port (from the axi2cfg2 base
 *  address). */
#define AXI2CFG_REG_CFG_WR ( 0x000 )

/*! Register offset for the config bus read port (from the axi2cfg2 base
 *  address). */
#define AXI2CFG_REG_CFG_RD ( 0x100 )

/*!
 * Read a number of 16 bit words from a picoArray axi2cfg.
 *
 * @param axi2cfg_base The base address of the upper axi2cfg.
 * @param aeid The CAEID of the AE to read from.
 * @param ae_addr The address to begin reading from within the AE.
 * @param[out] buf The buffer to store the results in.
 * @param count The number of 16 bit words to read.
 * @return Returns the number of words read on success, negative on failure.
 */
int axi2cfg_config_read( void __iomem *axi2cfg_base,
                         u16 aeid,
                         u16 ae_addr,
                         u16 *buf,
                         u16 count );

/*!
 * Write a number of 16 bit words to a picoArray axi2cfg.
 *
 * @param axi2cfg_base The base address of the upper axi2cfg.
 * @param aeid The CAEID of the AE to write to.
 * @param ae_addr The address to begin writing to within the AE.
 * @param[in] buf The buffer to read the words from.
 * @param count The number of 16 bit words to write.
 * @return Returns the number of words written on success, negative on failure.
 */
int axi2cfg_config_write( void __iomem *axi2cfg_base,
                          u16 aeid,
                          u16 ae_addr,
                          u16 *buf,
                          u16 count );


/*!
 * Write a register in the axi2cfg.
 *
 * @param address The base address of the axi2cfg (lower or upper region)
 * @param offset The offset of the register in bytes.
 * @param value The value of the register to write.
 * @return Returns zero on success, negative on failure.
 */
int
axi2cfg_reg_write( void __iomem *address,
                   unsigned offset,
                   u32 value );

/*!
 * Read a register in the axi2cfg.
 *
 * @param address The base address of the axi2cfg (lower or upper region)
 * @param offset The offset of the register in bytes.
 * @param[out] value The address to store the register value in.
 * @return Returns zero on success, negative on failure.
 */
int
axi2cfg_reg_read( void __iomem *address,
                  unsigned offset,
                  u32 *value );

#endif /* __PICOIF_AXI2CFG__ */
