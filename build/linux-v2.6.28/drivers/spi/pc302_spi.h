/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file pc302_spi.h
* \brief Definitions for the PC302 SPI Block.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_SPI_H
#define PC302_SPI_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/* Types ------------------------------------------------------------------- */

/* Macros ------------------------------------------------------------------ */
#define PC302_MAX_NUMBER_SPI_CS     (4)
#define PC302_MAX_NUMBER_SPI_BUSSES (1)
#define PC302_MIN_SPI_CLK_DIVIDER   (2)
#define PC302_MAX_SPI_CLK_DIVIDER   (65534)

/* SSI_CTRL_REG_0_REG_OFFSET bites */
#define PC302_SPI_LOOPBACK_MODE     (1 << 11)
#define PC302_SPI_NORMAL_MODE       (0)
#define PC302_SPI_TMOD_TX_RX        (0x0)
#define PC302_SPI_TMOD_TX           (0x1 << 8)
#define PC302_SPI_TMOD_RX           (0x2 << 8)
#define PC302_SPI_TMOD_EEPROM_RX    (0x3 << 8)
#define PC302_SPI_SCPOL             (1 << 7)
#define PC302_SPI_SCPH              (1 << 6)
#define PC302_SPI_MOTO_FORMAT       (0x0)
#define PC302_SPI_DATA_FRM_8_BIT    (0x7)


/* SSI_ENABLE_REG_REG_OFFSET bits */
#define PC302_SPI_ENABLE            (1)
#define PC302_SPI_DISABLE           (0)

/* SSI_SLAVE_ENABLE_REG_OFFSET bits */
#define PC302_SPI_SLAVES_DISABLE    (0)

/* SSI_STATUS_REG_OFFSET bits */
#define PC302_SPI_STATUS_DCOL       (1 << 6)
#define PC302_SPI_STATUS_TXE        (1 << 5)
#define PC302_SPI_STATUS_RFF        (1 << 4)
#define PC302_SPI_STATUS_RFNE       (1 << 3)
#define PC302_SPI_STATUS_TFE        (1 << 2)
#define PC302_SPI_STATUS_TFNF       (1 << 1)
#define PC302_SPI_STATUS_BUSY       (1 << 0)

/* SSI_IMR_REG_RESET bits */
#define PC302_SPI_MASK_ALL_INTS     (0xFFFF)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_SPI_H */
