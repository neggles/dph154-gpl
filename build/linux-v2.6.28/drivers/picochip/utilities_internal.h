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
 * \file utilities_internal.h
 * \brief Definition of utility functions for the picoIf driver.
 */

#ifndef __PICOIF_UTILITIES_INTERNAL_H__
#define __PICOIF_UTILITIES_INTERNAL_H__

/*!
 * Generic request and map function. Maps a physical address into a kernel
 * virtual address.
 *
 * @param name The name of the memory region being mapped.
 * @param addr The physical address of the memory.
 * @param len The length of the memory region in bytes.
 * @return Returns a pointer to the virtual address on success, NULL on
 * failure.
 */
void __iomem *
request_and_map( const char *name,
                 unsigned long addr,
                 size_t len );

/*!
 * Generic unmap and release function. Unmaps a kernel virtual address and
 * releases the resource.
 *
 * @param addr The physical address of the memory.
 * @param len The length of the memory region in bytes.
 * @param vaddr The virtual address to unmap.
 */
void
unmap_and_release( unsigned long addr,
                   size_t len,
                   void __iomem *vaddr );

#endif /* !__PICOIF_UTILITIES_INTERNAL_H__ */
