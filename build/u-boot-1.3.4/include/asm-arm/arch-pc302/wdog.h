/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file wdog.h
* \brief Definitions for the PC302 WDOG Block.
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_WDOG_H
#define PC302_WDOG_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define WDOG_CONTROL_REG_OFFSET             (0x00)
#define WDOG_TIMEOUT_RANGE_REG_OFFSET       (0x04)
#define WDOG_CURRENT_COUNT_REG_OFFSET       (0x08)
#define WDOG_COUNTER_RESTART_REG_OFFSET     (0x0c)
#define WDOG_INT_STATUS_REG_OFFSET          (0x10)
#define WDOG_CLEAR_REG_OFFSET               (0x14)

#define WDOG_PARAMS_5_REG_OFFSET            (0xe4)
#define WDOG_PARAMS_4_REG_OFFSET            (0xe8)
#define WDOG_PARAMS_3_REG_OFFSET            (0xec)
#define WDOG_PARAMS_2_REG_OFFSET            (0xf0)
#define WDOG_PARAMS_1_REG_OFFSET            (0xf4)

#define WDOG_COMP_VERSION_REG_OFFSET        (0xf8)
#define WDOG_COMP_TYPE_REG_OFFSET           (0xfc)

/*****************************************************************************/
/* Register Bit Field Manipulation                                           */
/*****************************************************************************/

/* Kick value */
#define WDOG_COUNTER_RESTART_KICK_VALUE	    (0x76)

/* Control bits */
#define WDOGCONTROLREGWDT_ENIDX		    (0)
#define WDOGCONTROLREGRMODIDX		    (1)
#define WDOGCONTROLREGRPLIDX		    (2)

/* Masks */
#define WDOGCONTROLREGWDT_ENMASK	    (1 << WDOGCONTROLREGWDT_ENIDX)
#define WDOGCONTROLREGRMODMASK		    (1 << WDOGCONTROLREGRMODIDX)
#define WDOGCONTROLREGRPLMASK		    (0x7 << WDOGCONTROLREGRPLIDX)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_WDOG_H */
