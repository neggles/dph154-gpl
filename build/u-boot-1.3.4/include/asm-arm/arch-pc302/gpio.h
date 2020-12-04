/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file gpio.h
* \brief Definitions for the PC302 GPIO Block.
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_GPIO_H
#define PC302_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define GPIO_SW_PORT_A_DR_REG_OFFSET        (0x00)
#define GPIO_SW_PORT_A_DDR_REG_OFFSET	    (0x04)
#define GPIO_SW_PORT_A_CTL_REG_OFFSET       (0x08)
#define GPIO_SW_PORT_B_DR_REG_OFFSET        (0x0C)
#define GPIO_SW_PORT_B_DDR_REG_OFFSET	    (0x10)
#define GPIO_SW_PORT_B_CTL_REG_OFFSET       (0x14)
#define GPIO_SW_PORT_C_DR_REG_OFFSET        (0x18)
#define GPIO_SW_PORT_C_DDR_REG_OFFSET	    (0x1C)
#define GPIO_SW_PORT_C_CTL_REG_OFFSET       (0x20)

/* no port {D} */

/* global GPIO registers */
#define GPIO_INT_EN_REG_OFFSET              (0x30)
#define GPIO_INT_MASK_REG_OFFSET            (0x34)
#define GPIO_INT_TYPE_LEVEL_REG_OFFSET 	    (0x38)
#define GPIO_INT_POLARITY_REG_OFFSET 	    (0x3c)

#define GPIO_INT_STATUS_REG_OFFSET	    (0x40)
#define GPIO_RAW_INT_STATUS_REG_OFFSET	    (0x44)

/* no debounce */
#define GPIO_PORT_A_EOI_REG_OFFSET          (0x4c)
#define GPIO_EXT_PORT_A_REG_OFFSET          (0x50)
#define GPIO_EXT_PORT_B_REG_OFFSET          (0x54)
#define GPIO_EXT_PORT_C_REG_OFFSET          (0x58)

/* no port {D} */

#define GPIO_LS_SYNC_REG_OFFSET		    (0x60)
#define GPIO_ID_CODE_REG_OFFSET		    (0x64)
#define GPIO_RESERVED_REG_OFFSET            (0x68)
#define GPIO_COMP_VERSION_REG_OFFSET	    (0x6c)


/* Macros ------------------------------------------------------------------ */

/* Useful bit definitions */
#define GPIO_BIT_7  (0x80)
#define GPIO_BIT_6  (0x40)
#define GPIO_BIT_5  (0x20)
#define GPIO_BIT_4  (0x10)
#define GPIO_BIT_3  (0x08)
#define GPIO_BIT_2  (0x04)
#define GPIO_BIT_1  (0x02)
#define GPIO_BIT_0  (0x01)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_GPIO_H */
