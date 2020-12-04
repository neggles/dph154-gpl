/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file pa.h
* \brief Definitions for the picoArray.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_PA_H
#define PC302_PA_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

#define PA_AEID_MEMIF       (0x0088)

#define PA_CONFIG_WRITE     (0x00010000)
#define PA_CONFIG_READ      (0x00020000)
#define PA_CONFIG_ADDR      (0x00040000)
#define PA_CONFIG_AEID      (0x00080000)
#define PA_CONFIG_VALID     (0x00010000)
#define PA_CONFIG_FAIL      (0x00020000)
#define PA_CONFIG_TIMEOUT   (0x00040000)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_PA_H */
