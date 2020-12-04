/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
 * \file cpld_hdp102.h
 * \brief Header file for HDP102 CPLD access
 *
 * Copyright (c)(p) 1997-2004 Airspan Networks Inc
 * Copyright (c) 2007 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

#ifndef __CPLD_HDP102_H__
#define __CPLD_HDP102_H__

#include <asm/types.h>

/**
 This driver provides a common interface for other drivers that need 
 to access a shared register in the CPLD.  The API should be kept 
 consistent with drivers/asmax/cpld.h as far as possible.
**/

typedef struct cpld_reg_param {
    __u16 device;
    __u16 reg;
    __u16 data;
} cpld_reg_param_t;

#define PICO_CPLD_IOC_MAGIC   'C'
#define PICO_CPLD_IOC_BASE     0
#define PICO_CPLD_IOCWRITEREG  _IOW(   PICO_CPLD_IOC_MAGIC, PICO_CPLD_IOC_BASE + 0, cpld_reg_param_t)
#define PICO_CPLD_IOCREADREG   _IOWR(  PICO_CPLD_IOC_MAGIC, PICO_CPLD_IOC_BASE + 1, cpld_reg_param_t)
#define PICO_CPLD_IOC_MAXNUM (PICO_CPLD_IOC_BASE + 1)

/* Platform stuff */
#define PICO_CPLD_MAX_NUM_DEV 1              /* One programmable device on the hdp platforms */

/** Register definitions  **/
/*
 * Please maintain the comments on the ends of the lines - these are for
 * code-generattion in the cpld util.
 */
#define PICO_CPLD_REG_REV            (0x0)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_TEST           (0x1)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_DEBUG          (0x2)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_LEDRESET       (0x3)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_RFCTRL         (0x4)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_RFSPI          (0x5)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_RFIN           (0x6)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_ATMCTRL        (0x7)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_PC102JTAG      (0x8)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_DUMMY1         (0x9)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_DREQCTRL       (0xA)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_HDP_MODE       (0xB)  /* codegen_cpld_reg */
#define PICO_CPLD_REG_DMA_SELECT     (0xC)  /* codegen_cpld_reg */

#define PICO_CPLD_REG_NREGS          (0x10)

/*
 * Number of DMA channels under our control via 
 * PICO_CPLD_REG_DREQCTL register
 */
#define PICO_CPLD_DMA_NCHANS         (4)

/** PicoChip resets are in register 3, with the LED control and
 *  watchdog bits.
 *  Bits 4..0 are LEDS 4..0; resets are write 1 to reset, auto-clearing.
 */

#define PICO_CPLD_BIT_LEDRESET_PA0RESET      (0x8000)
#define PICO_CPLD_BIT_LEDRESET_PA1RESET      (0x4000)
#define PICO_CPLD_BIT_LEDRESET_PA2RESET      (0x2000)
#define PICO_CPLD_BIT_LEDRESET_PA3RESET      (0x1000)
#define PICO_CPLD_BIT_LEDRESET_WATCHDOG      (0x0080)
#define PICO_CPLD_BIT_LEDRESET_LEDMODE       (0x0040)
#define PICO_CPLD_BIT_LEDRESET_LEDMASK       (0x001F)

#define PICO_CPLD_REG_BLOCKSIZE	(0x10 * sizeof(u16))

#ifdef __KERNEL__

#include <asm/picochip/hdp102.h>

/* Reset Register */
/* These are dummy functions */
void cpld_fpga_reset(int fpga);
void cpld_fpga_reset_assert(int fpga);
void cpld_fpga_reset_deassert(int fpga);

/* Only reset_assert is used; our reset is auto-clearing */
void cpld_pico_reset_assert(int pico);
void cpld_pico_reset_deassert(int pico);

/* Host CTRL Register - not implemented on this target, so dummy functions */
void cpld_fan_on(int fan);
void cpld_fan_off(int fan);

/* functions for CPLD register read and write */
unsigned short cpld_reg_read(unsigned char regno);
void cpld_reg_write(unsigned char regno, unsigned short data);

/* functions for working with the CPLD DMA state machines */
void cpld_dma_enable(unsigned char chan);
void cpld_dma_disable(unsigned char chan);
void cpld_dma_reset(unsigned char chan);

/* CPLD Version Register - not used at present */
u8 cpld_version(void);
u8 cpld_diagnostics(void);

#endif	/* KERNEL */
#endif  /* __CPLD_HDP102__ */

