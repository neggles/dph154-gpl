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
 * \file resource.h
 * \brief Generic resource get and put functions.
 *
 * Generic device resource managing functions
 */

#ifndef __PICOIF_RESOURCE_H__
#define __PICOIF_RESOURCE_H__

#include "picoarray.h"

struct pico_resource *
generic_get_resource( struct picoarray *pa,
                      enum pico_resource_type type,
                      unsigned value,
                      int exclusive );

void
generic_put_resource( struct picoarray *dev,
                      struct pico_resource *resource );

#endif /* !__PICOIF_RESOURCE_H__ */
