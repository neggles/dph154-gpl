/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file mii_phy.h
* \brief Definitions for the Ethernet Phy used on hardware platforms.
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_MII_PHY_H
#define PC302_MII_PHY_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constant-s -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/
#define PHY_BMCR                    (0x00)
#define PHY_BMSR		    (0x01)
#define PHY_PHYIDR1		    (0x02)
#define PHY_PHYIDR2		    (0x03)
#define PHY_ANAR		    (0x04)
#define PHY_ANLPAR		    (0x05)
#define PHY_ANER		    (0x06)
#define PHY_ANNPTR		    (0x07)

/* PHY_BMCR Bits */
#define PHY_BMCR_RESET		    (1<< 15)
#define PHY_BMCR_LOOP_BACK	    (1 << 14)
#define PHY_BMCR_100MB		    (1 << 13)
#define PHY_BMCR_AUTO_NEG_ENABLE    (1 << 12)
#define PHY_BMCR_POWER_DOWN         (1 << 11)
#define PHY_BMCR_ISOLATE	    (1 << 10)
#define PHY_BMCR_RESTART_NEG	    (1 << 9)
#define PHY_BMCR_FULL_DUPLEX	    (1 << 8)

/* PHY_BMSR Bits */
#define PHY_BMSR_100T4		    (1 << 15)
#define PHY_BMSR_100TXF		    (1 << 14)
#define PHY_BMSR_100TXH		    (1 << 13)
#define PHY_BMSR_10TF		    (1 << 12)
#define PHY_BMSR_10TH		    (1 << 11)
#define PHY_BMSR_AUTO_NEG_COMPLETE  (1 << 5)
#define PHY_BMSR_REMOTE_FAULT	    (1 << 4)
#define PHY_BMSR_AUTO_NEG_ABLE	    (1 << 3)
#define PHY_BMSR_LINK_UP	    (1 << 2)

/* PHY_ANLPAR Bits */
#define PHY_ANLPAR_100TXF	    (1 << 8)
#define PHY_ANLPAR_100TXH           (1 << 7)
#define PHY_ANLPAR_10TF             (1 << 6)
#define PHY_ANLPAR_10TH		    (1 << 5)

#endif /* PC302_MII_PHY_H */
