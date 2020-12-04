/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2007 picoChip Designs Ltd.
 *
 * * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file cpld_cpe20x.h
 * \brief Header file for CPE20x FPGA access
 *
 * This file defines the internal CPLD access functions.
 */

#ifndef _CPLD_CPE20X_H__
#define _CPLD_CPE20X_H__

#include <asm/types.h>

typedef struct cpld_reg_param
{
    __u16 device;
    __u16 reg;
    __u16 data;
} cpld_reg_param_t;

#define PICO_CPLD_IOC_MAGIC   'C'
#define PICO_CPLD_IOC_BASE     0
#define PICO_CPLD_IOCWRITEREG \
    _IOW(   PICO_CPLD_IOC_MAGIC, PICO_CPLD_IOC_BASE + 0, cpld_reg_param_t)
#define PICO_CPLD_IOCREADREG \
    _IOWR(  PICO_CPLD_IOC_MAGIC, PICO_CPLD_IOC_BASE + 1, cpld_reg_param_t)
#define PICO_CPLD_IOC_MAXNUM (PICO_CPLD_IOC_BASE + 1)

/* Platform stuff */
#define PICO_CPLD_MAX_NUM_DEV 2   /* Two programmable devices on the cpe20x
                                        platform */

/** Register definitions  **/
/*
 * Please maintain the comments on the ends of the lines - these are for
 * code-generattion in the cpld util.
 */

/* Bank #0 - Generic Registers */
#define PICO_CPLD_REG_BOARD_TYPE     (0x0)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_CODE_VERSION   (0x1)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_BOARD_MODE     (0x2)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_TEST_DEBUG     (0x3)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_DEBUG_COUNTER  (0x4)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IRQ_SELECT     (0x5)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IRQ_STATUS     (0x6)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IRQ_ENABLE     (0x7)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_DMA_SELECT     (0x8)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_PA_RESET       (0x9)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_PICE_CONTROL   (0xA)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_RESERVED_01    (0xB)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_RESERVED_02    (0xC)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_RESERVED_03    (0xD)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_RESERVED_04    (0xE)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_RESERVED_05    (0xF)  /* codegen_cpld_reg */

/* Bank #1 - Input registers */
#define PICO_CPLD_REG_IP_REG_00      (0x10)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_01      (0x11)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_02      (0x12)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_03      (0x13)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_04      (0x14)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_05      (0x15)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_06      (0x16)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_07      (0x17)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_08      (0x18)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_09      (0x19)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_10      (0x1A)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_11      (0x1B)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_12      (0x1C)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_13      (0x1D)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_14      (0x1E)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_IP_REG_15      (0x1F)  /* codegen_cpld_reg */

/* Bank #2 - Output registers */
#define PICO_CPLD_REG_OP_REG_00      (0x20)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_01      (0x21)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_02      (0x22)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_03      (0x23)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_04      (0x24)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_05      (0x25)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_06      (0x26)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_07      (0x27)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_08      (0x28)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_09      (0x29)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_10      (0x2A)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_11      (0x2B)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_12      (0x2C)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_13      (0x2D)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_14      (0x2E)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_OP_REG_15      (0x2F)  /* codegen_cpld_reg */

/* Bank #3 - Select registers */
#define PICO_CPLD_REG_SEL_REG_00     (0x30)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_01     (0x31)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_02     (0x32)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_03     (0x33)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_04     (0x34)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_05     (0x35)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_06     (0x36)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_07     (0x37)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_08     (0x38)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_09     (0x39)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_10     (0x3A)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_11     (0x3B)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_12     (0x3C)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_13     (0x3D)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_14     (0x3E)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_SEL_REG_15     (0x3F)  /* codegen_cpld_reg */

/* Bank #4 - Tri-State registers */
#define PICO_CPLD_REG_ENB_REG_00     (0x40)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_01     (0x41)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_02     (0x42)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_03     (0x43)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_04     (0x44)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_05     (0x45)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_06     (0x46)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_07     (0x47)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_08     (0x48)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_09     (0x49)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_10     (0x4A)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_11     (0x4B)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_12     (0x4C)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_13     (0x4D)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_14     (0x4E)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ENB_REG_15     (0x4F)  /* codegen_cpld_reg */

#define PICO_CPLD_REG_NREGS          (0x50)

#endif  /* _CPLD_CPE20X_H__ */

