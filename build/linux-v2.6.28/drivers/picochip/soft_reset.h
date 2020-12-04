/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2006 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file soft_reset.h
 * \brief Header for performing a software reset of the picoArray
 */

#ifndef __PICOIF_SOFT_RESET_H__
#define  __PICOIF_SOFT_RESET_H__

/*!
 * Perform a soft reset of the picoArray.
 *
 * @param procif_base The base address of the procif of the device to reset.
 * @param ahb_base The base address of the AHB2Pico of the device to reset.
 */
void picoArraySoftReset (void __iomem *procif_base, void __iomem *ahb_base );

#endif /* __PICOIF_SOFT_RESET_H__ */
