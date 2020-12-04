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
 * \file axi2cfg.c
 * \brief axi2cfg access functions.
 *
 * This file implements functions for using the axi2cfg to configure and debug
 * picoArray systems providing configuration bus access over the axi2cfg.
 */

#include <linux/types.h>
#include <asm/io.h>
#include <linux/pci.h>
#include "debug.h"
#include "axi2cfg.h"

/* Configuration port write bit positions. */
#define CAEID_BIT_MASK     ( 1 << 19 )    /*!< Bit 19 - AE ID signal. */
#define CADDR_BIT_MASK     ( 1 << 18 )    /*!< Bit 18 - AE ADDR signal. */
#define CREAD_BIT_MASK     ( 1 << 17 )    /*!< Bit 17 - READ data signal. */
#define CWRITE_BIT_MASK    ( 1 << 16 )    /*!< Bit 16 - WRITE data signal. */

#define RB_FAIL_MASK       ( 1 << 17 )    /*!< Bit 17 - readback failed. */
#define RB_VALID_MASK      ( 1 << 16 )    /*!< Bit 16 - readback valid. */

#define RETRIES ( 10 )                  /*!< The number of retries for an \
                                         *   AXI2Cfg config read. */

int
axi2cfg_config_read( void __iomem *axi2cfg_base,
                    u16 aeid,
                    u16 ae_addr,
                    u16 *buf,
                    u16 count )
{
    u32 val;
    void __iomem *write_p = axi2cfg_base + AXI2CFG_REG_CFG_WR;
    void __iomem *read_p = axi2cfg_base + AXI2CFG_REG_CFG_RD;
    u16 to_read = count;
    u16 rc;
    unsigned i;
    unsigned retries;

    PRINTD( COMPONENT_AXI2CFG, DBG_TRACE, "reading %u words from %04x@%04x",
            count, aeid, ae_addr );

    val = aeid | CAEID_BIT_MASK;
    iowrite32( val, write_p );
    wmb();

    while ( to_read )
    {
        /* Output the address to read from. */
        val = ( ae_addr + ( count - to_read ) ) | CADDR_BIT_MASK;
        iowrite32( val, write_p );
        wmb();

        /* Dispatch the read requests. */
        rc = ( to_read > 64 ) ? 64 : to_read;
        val = CREAD_BIT_MASK | rc;
        iowrite32( val, write_p );
        wmb();

        /* Now read the values. */
        for ( i = 0; i < rc; ++i )
        {
            retries = RETRIES;
            while ( retries )
            {
                val = ioread32( read_p );
                if ( val & ( RB_VALID_MASK | RB_FAIL_MASK ) )
                    break;
                --retries;
                rmb();
            }

            if ( !retries || ( val & RB_FAIL_MASK ) )
            {
                PRINTD( COMPONENT_AXI2CFG, DBG_ERROR,
                        "config read %04x@%04x failed", aeid,
                        ( ae_addr + ( count - to_read ) + i ) );
                break;
            }
            else
                buf[ ( count - to_read ) + i ] = val & 0xFFFF;
        }

        if ( val & RB_FAIL_MASK )
            break;

        to_read -= rc;
    }

    return !( val & RB_FAIL_MASK ) ? count : -EIO;
}

int
axi2cfg_config_write( void __iomem *axi2cfg_base,
                     u16 aeid,
                     u16 ae_addr,
                     u16 *buf,
                     u16 count )
{
    u32 val;
    void __iomem *write_p = axi2cfg_base + AXI2CFG_REG_CFG_WR;
    unsigned i;

    PRINTD( COMPONENT_AXI2CFG, DBG_TRACE, "writing %u words to %04x@%04x",
            count, aeid, ae_addr );

    val = aeid | CAEID_BIT_MASK;
    iowrite32( val, write_p );
    wmb();

    /* Output the address to read from. */
    val = ae_addr | CADDR_BIT_MASK;
    iowrite32( val, write_p );
    wmb();

    /* Now read the values. */
    for ( i = 0; i < count; ++i )
    {
        val = buf[ i ] | CWRITE_BIT_MASK;
        iowrite32( val, write_p );
        wmb();
    }

    return i;
}

int
axi2cfg_reg_write( void __iomem *axi2cfg_base,
                    unsigned offset,
                    u32 value )
{
    iowrite32( value, axi2cfg_base + offset );
    return 0;
}

int
axi2cfg_reg_read( void __iomem *axi2cfg_base,
                  unsigned offset,
                  u32 *value )
{
    *value = ioread32( axi2cfg_base + offset );
    return 0;
}


