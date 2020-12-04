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
 * \file hwif2_internal.h
 * \brief Initialisation function for HwIF2 transport.
 *
 * Internal HwIF2 transport function definitions
 */

#ifndef __PICOIF_HWIF2_INTERNAL_H__
#define __PICOIF_HWIF2_INTERNAL_H__

/*!
 * Initialise the HwIF2 transport.
 *
 * @return Returns zero on success, negative on failure.
 */
int
hwif2_init( void );

#endif /* !__PICOIF_HWIF2_INTERNAL_H__ */
