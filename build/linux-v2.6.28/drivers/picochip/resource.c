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
 * \file resource.c
 * \brief Generic resource acquisition/release functions.
 *
 * This file implements functions for requesting and putting picoArray
 * resources where no extra behaviour other than getting/releasing the
 * resource is required. This should be sufficient for almost all devices.
 */

#include "picoarray.h"
#include "resource.h"

/*!
 * Get a resource from a picoArray device. For most devices, this should be
 * sufficient, but if there are any corner cases for a device, this function
 * may be overridden.
 *
 * @param pa The device to acquire the resource from.
 * @param type The type of resource to acquire.
 * @param value The resource ID to acquire.
 * @param exclusive Boolean flag to indicate that exclusive access is required
 * to the resource.
 * @return Returns zero on success, non-zero on failure.
 */
struct pico_resource *
generic_get_resource( struct picoarray *pa,
                      enum pico_resource_type type,
                      unsigned value,
                      int exclusive )
{
    struct pico_resource *tmp = pa->resources;
    int ret = 1;
    struct pico_resource *resource = NULL;

    spin_lock( &pa->lock );

    /* The end of the array should be marked with all zero's. */
    while ( tmp->type || tmp->value || tmp->exclusive )
    {
        /* This is the resource. */
        if ( tmp->type == type && tmp->value == value )
        {
            if ( exclusive && !tmp->exclusive )
            {
                tmp->exclusive = 1;
                ret = 0;
            }
            if ( !exclusive && !tmp->exclusive )
                ret = 0;

            if ( !ret )
            {
                resource = tmp;
                atomic_inc( &tmp->ref_count );
                break;
            }
        }

        ++tmp;
    }

    spin_unlock( &pa->lock );

    return resource;
}

/*!
 * Decrement the reference count of a resource.
 *
 * @param dev The device the resource belongs to.
 * @param resource The resource to put.
 */
void
generic_put_resource( struct picoarray *dev,
                      struct pico_resource *resource )
{
    spin_lock( &dev->lock );

    if ( 0 == atomic_read( &resource->ref_count ) )
        goto out;

    if ( resource->exclusive )
        resource->exclusive = 0;

    atomic_dec( &resource->ref_count );

out:
    spin_unlock( &dev->lock );
}
