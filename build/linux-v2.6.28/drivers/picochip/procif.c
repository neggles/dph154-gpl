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
 * \file procif.c
 * \brief ProcIf access functions.
 *
 * This file implements functions for using the procif to configure and debug
 * picoArray systems providing configuration bus access over the procif.
 */

#include <linux/types.h>
#include <asm/io.h>
#include <linux/spinlock.h>
#include "debug.h"
#include "procif.h"
#include "picoif_internal.h"

/* Configuration port write bit positions. */
#define CAEID_BIT_MASK    ( 1 << 19 )    /*!< Bit 19 - AE ID signal. */
#define CADDR_BIT_MASK    ( 1 << 18 )    /*!< Bit 18 - AE ADDR signal. */
#define CREAD_BIT_MASK    ( 1 << 17 )    /*!< Bit 17 - READ data signal. */
#define CWRITE_BIT_MASK   ( 1 << 16 )    /*!< Bit 16 - WRITE data signal. */

#define RB_FAIL_MASK      ( 1 << 17 )    /*!< Bit 17 - readback failed. */
#define RB_VALID_MASK     ( 1 << 16 )    /*!< Bit 16 - readback valid. */

#define RETRIES ( 10 )                   /*!< The number of retries for a \
                                          *   procif config read. */

/*! Spinlock for procif accesses. We could have a spinlock for each procif,
 *  but these operations are in the slow path and it is easier to have a
 *  single lock and serialise all config accesses. */
static spinlock_t procif_lock = SPIN_LOCK_UNLOCKED;

int
procif_config_read( void __iomem *procif_base,
                    u16 aeid,
                    u16 ae_addr,
                    u16 *buf,
                    u16 count )
{
    u32 val;
    void __iomem *write_p = procif_base + PROCIF_REG_CFG_WR_OFFSET;
    void __iomem *read_p = procif_base + PROCIF_REG_CFG_RD_OFFSET;
    u16 to_read = count;
    u16 rc;
    unsigned i;
    unsigned retries;
    unsigned long flags;

    PRINTD( COMPONENT_PROCIF, DBG_TRACE, "reading %u words from %04x@%04x",
            count, aeid, ae_addr );

    spin_lock_irqsave( &procif_lock, flags );

    val = aeid | CAEID_BIT_MASK;
    picoif_out32( val, write_p );
    wmb();

    while ( to_read )
    {
        /* Output the address to read from. */
        val = ( ae_addr + ( count - to_read ) ) | CADDR_BIT_MASK;
        picoif_out32( val, write_p );
        wmb();

        /* Dispatch the read requests. */
        rc = ( to_read > 64 ) ? 64 : to_read;
        val = CREAD_BIT_MASK;
        for ( i = 0; i < rc; ++i )
        {
            picoif_out32( val, write_p );
            wmb();
        }

        /* Now read the values. */
        for ( i = 0; i < rc; ++i )
        {
            retries = RETRIES;
            while ( retries )
            {
                val = picoif_in32( read_p );
                if ( val & ( RB_VALID_MASK | RB_FAIL_MASK ) )
                    break;
                --retries;
                rmb();
            }

            if ( !retries || ( val & RB_FAIL_MASK ) )
            {
                PRINTD( COMPONENT_PROCIF, DBG_ERROR,
                        "config read %04x@%04x failed", aeid,
                        ( ae_addr + ( count - to_read ) + i ) );
                if ( !retries )
                    PRINTD( COMPONENT_PROCIF, DBG_ERROR, "timed out" );

                break;
            }
            else
                buf[ ( count - to_read ) + i ] = val & 0xFFFF;
        }

        if ( val & RB_FAIL_MASK )
            break;

        to_read -= rc;
    }

    spin_unlock_irqrestore( &procif_lock, flags );

    return !( val & RB_FAIL_MASK ) ? count : -EIO;
}

int
procif_config_write( void __iomem *procif_base,
                     u16 aeid,
                     u16 ae_addr,
                     u16 *buf,
                     u16 count )
{
    u32 val;
    void __iomem *write_p = procif_base + PROCIF_REG_CFG_WR_OFFSET;
    unsigned i;
    unsigned long flags;

    PRINTD( COMPONENT_PROCIF, DBG_TRACE, "writing %u words to %04x@%04x",
            count, aeid, ae_addr );

    spin_lock_irqsave( &procif_lock, flags );

    val = aeid | CAEID_BIT_MASK;
    picoif_out32( val, write_p );
    /* Flush the AEID request to comply with the procif protocol. */
    wmb();

    /* Output the address to read from. */
    val = ae_addr | CADDR_BIT_MASK;
    picoif_out32( val, write_p );
    /* Flush the address request to comply with the procif protocol. */
    wmb();

    /* Now read the values. */
    for ( i = 0; i < count; ++i )
    {
        val = buf[ i ] | CWRITE_BIT_MASK;
        picoif_out32( val, write_p );
    }

    spin_unlock_irqrestore( &procif_lock, flags );

    return i;
}

int
procif_reg_write( void __iomem *base,
                  unsigned offset,
                  u32 value )
{
    picoif_out32( value, base + offset );
    return 0;
}

int
procif_reg_read( void __iomem *base,
                 unsigned offset,
                 u32 *value )
{
    *value = picoif_in32( base + offset );
    return 0;
}
