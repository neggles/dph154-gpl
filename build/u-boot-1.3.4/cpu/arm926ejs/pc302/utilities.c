/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file utilities.c
* \brief Various useful functions for PC302
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

/* Includes ---------------------------------------------------------------- */
#include <common.h>
#include <asm/errno.h>
#include <asm/arch/pc302.h>
#include <asm/arch/axi2cfg.h>
#include <asm/arch/fuse.h>
#include <asm/arch/utilities.h>

/* Macros ------------------------------------------------------------------ */
/*!
 * Bit mask used to obtain the least significant 16 bits
 */
#define SIXTEEN_BIT_MASK        (0xFFFF)

/*!
 * Bit mask used to obtain the device ID
 */
#define DEVICE_ID_MASK          (0x000F)

/* Configuration port write bit positions. */
#define CAEID_BIT_MASK     ( 1 << 19 )    /*!< Bit 19 - AE ID signal. */
#define CADDR_BIT_MASK     ( 1 << 18 )    /*!< Bit 18 - AE ADDR signal. */
#define CREAD_BIT_MASK     ( 1 << 17 )    /*!< Bit 17 - READ data signal. */
#define CWRITE_BIT_MASK    ( 1 << 16 )    /*!< Bit 16 - WRITE data signal. */

#define RB_FAIL_MASK       ( 1 << 17 )    /*!< Bit 17 - readback failed. */
#define RB_VALID_MASK      ( 1 << 16 )    /*!< Bit 16 - readback valid. */

#define RETRIES ( 10 )                  /*!< The number of retries for an \
                                         *   AXI2Cfg config read. */

/*! Register offset for the config bus write port (from the axi2cfg2 base
 *  address). */
#define AXI2CFG_REG_CFG_WR ( 0x0100 )

/*! Register offset for the config bus read port (from the axi2cfg2 base
 *  address). */
#define AXI2CFG_REG_CFG_RD ( 0x0200 )

/* Constants --------------------------------------------------------------- */

/* Types ------------------------------------------------------------------- */

/* Prototypes--------------------------------------------------------------- */

/* Functions --------------------------------------------------------------- */
__inline unsigned int pc302_read_from_register (const unsigned int address)
{
    /* Read an integer (32 bit) value from a register */

    return(*(volatile unsigned int *)address);
}

__inline void pc302_write_to_register (const unsigned int address,
                                       const unsigned int value)
{
    /* Write an integer value to a register */

    *(volatile unsigned int *)address = value;
}

unsigned int pc302_read_device_id (void)
{
    unsigned int device_id;

    device_id = pc302_read_from_register (PC302_AXI2CFG_BASE +
                                          AXI2CFG_DEVICE_ID_REG_OFFSET);
    device_id &= DEVICE_ID_MASK;

    return device_id;
}

unsigned int pc302_read_device_revision (void)
{
    unsigned int revision_code;

    revision_code = pc302_read_from_register (PC302_AXI2CFG_BASE +
                                              AXI2CFG_REVISION_ID_REG_OFFSET);
    revision_code &= SIXTEEN_BIT_MASK;

    return revision_code;
}

void pc302_read_die_id_number (unsigned int * die_number)
{
    /* Read the 128 bit manufacturing id from the fuses and store
       in the provided array */

    unsigned int i;

    for (i = 0; i < 4; i++)
    {
        *die_number++ = pc302_read_from_register(PC302_FUSE_BASE +
                                                 FUSE_MAP_24_REG_OFFSET +
                                                 (i * sizeof (unsigned int)));
    }
}

unsigned int pc302_get_rmii_enabled (void)
{
    unsigned int rmii_enabled;

    rmii_enabled = pc302_read_from_register (PC302_AXI2CFG_BASE +
                                             AXI2CFG_SYS_CONFIG_REG_OFFSET);
    rmii_enabled &= AXI2CFG_RMII_EN;

    return !!rmii_enabled;
}


/*!
 * Read a number of 16 bit words from the PC302 axi2cfg.
 *
 * @param pa The device to read from.
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
                   u16 *data )
{
    u32 val;
    unsigned int write_p = PC302_AXI2CFG_BASE + AXI2CFG_REG_CFG_WR;
    unsigned int read_p = PC302_AXI2CFG_BASE + AXI2CFG_REG_CFG_RD;
    u16 to_read = count;
    u16 rc;
    unsigned i;
    unsigned retries;

    val = caeid | CAEID_BIT_MASK;
    pc302_write_to_register( write_p, val );

    while ( to_read )
    {
        /* Output the address to read from. */
        val = ( address + ( count - to_read ) ) | CADDR_BIT_MASK;
        pc302_write_to_register( write_p, val );

        /* Dispatch the read requests. */
        rc = ( to_read > 64 ) ? 64 : to_read;
        val = CREAD_BIT_MASK | rc;
        pc302_write_to_register( write_p, val );

        /* Now read the values. */
        for ( i = 0; i < rc; ++i )
        {
            retries = RETRIES;
            while ( retries )
            {
                val = pc302_read_from_register( read_p );
                if ( val & ( RB_VALID_MASK | RB_FAIL_MASK ) )
                    break;
                --retries;
            }

            if ( !retries || ( val & RB_FAIL_MASK ) )
            {
                break;
            }
            else
                data[ ( count - to_read ) + i ] = val & 0xFFFF;
        }

        if ( val & RB_FAIL_MASK )
            break;

        to_read -= rc;
    }

    return !( val & RB_FAIL_MASK ) ? count : -EIO;
}

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
                    u16 *data )
{
    u32 val;
    unsigned int write_p = PC302_AXI2CFG_BASE + AXI2CFG_REG_CFG_WR;
    unsigned i;

    val = caeid | CAEID_BIT_MASK;
    pc302_write_to_register( write_p, val );

    /* Output the address to write to */
    val = address | CADDR_BIT_MASK;
    pc302_write_to_register( write_p, val );

    /* Now write the values. */
    for ( i = 0; i < count; ++i )
    {
        val = data[ i ] | CWRITE_BIT_MASK;
        pc302_write_to_register( write_p, val );
    }

    return i;
}
