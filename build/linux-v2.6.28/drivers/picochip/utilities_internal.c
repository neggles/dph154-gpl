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
 * \file utilities_internal.c
 * \brief Implementation of utility functions for the picoIf driver.
 *
 * Implementation of utility functions for the picoIf driver.
 *
 */

#include <linux/io.h>
#include <linux/ioport.h>
#include "utilities_internal.h"

void __iomem *
request_and_map( const char *name,
                 unsigned long addr,
                 size_t len )
{
    void __iomem *ret = NULL;
    struct resource *r = request_mem_region( addr, len, name );
    if ( !r )
        goto out;

    ret = ioremap( addr, len );
    if ( !ret )
    {
        release_resource( r );
        goto out;
    }

out:
    return ret;
}

void
unmap_and_release( unsigned long addr,
                   size_t len,
                   void __iomem *vaddr )
{
    iounmap( vaddr );
    release_mem_region( addr, len );
}
