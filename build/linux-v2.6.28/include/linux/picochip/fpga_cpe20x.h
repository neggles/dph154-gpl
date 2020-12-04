/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 *
 */

/*!
 * \file fpga_cpe20x.h
 * \brief Firecracker - FPGA device driver header file
 *
 */

#ifndef __FPGA_CPE20X_H__
#define __FPGA_CPE20X_H__

/*!
 * Get the handle of an FPGA device for use with read/write functions
 */
void
*fpga_get_handle(unsigned int fpga_device_id);

/* Read/Write Calls may only be used from a context that may sleep */

/*!
 * Read one of the FPGA registers
 */
u16
fpga_read_reg(void *handle,
              unsigned int reg_addr);

/*!
 * Write one of the FPGA registers
 */
void
fpga_write_reg(void *handle,
               unsigned int reg_addr,
               u16 value);

/*!
 * Platform data structure associated with each fpga
 */
struct fpga_platform_data
{
    unsigned int fpga_device_id;
};


/*!
 * Device ID's for use with fpga_get_handle
 */
#define FPGA_RC_ID                      (1)
#define FPGA_AD_ID                      (2)

/* SPI protocol definitions */

/*!
 * First 16 bit word, address/command
 */
#define FPGA_SPI_ADDR_SHIFT             (8)
#define FPGA_SPI_ADDR_MASK              (0xff << FPGA_SPI_ADDR_SHIFT)
#define FPGA_SPI_CMD_R_NW               (1 << 7)
#define FPGA_SPI_CMD_RST                (1 << 6)


/*!
 * FPGA registers - BANK 0
 */
#define FPGA_TYPE_ID_REG                (0x00)
#define FPGA_VERSION_REG                (0x01)
#define FPGA_MODE_REG                   (0x02)
#define FPGA_TEST_REG                   (0x03)
#define FPGA_TEST_COUNTER_REG           (0x04)
#define FPGA_IRQ_SELECT_REG             (0x05)
#define FPGA_IRQ_STATUS_REG             (0x06)
#define FPGA_IRQ_ENABLE_REG             (0x07)
#define FPGA_IRQ_DMA_SELECT_REG         (0x08)
#define FPGA_PICOARRAY_RESET_REG        (0x09)
#define FPGA_PICOARRAY_CONTROL_REG      (0x0a)

/*!
 * FPGA bank select - BANK 1,2,3,4.
 * Note: Use these with the following macros. For example to select the
 * LED enable register (addr 0x4a) use:
 *  FPGA_LED_REG(FPGA_OE)
 */
#define FPGA_INPUT                      (1)
#define FPGA_OUTPUT                     (2)
#define FPGA_SELECT                     (3)
#define FPGA_OE                         (4)

/*!
 * Back registers
 */
#define FPGA_ADIB_CH0_RX_REG(__B)       (0x00 + ((__B) * 0x10)
#define FPGA_ADIB_CH1_TX_REG(__B)       (0x01 + ((__B) * 0x10)
#define FPGA_ADIC_CH0_RX_REG(__B)       (0x02 + ((__B) * 0x10)
#define FPGA_ADIC_CH1_TX_REG(__B)       (0x03 + ((__B) * 0x10)
#define FPGA_REG4(__B)                  (0x04 + ((__B) * 0x10)
#define FPGA_REG5(__B)                  (0x05 + ((__B) * 0x10)
#define FPGA_REG6(__B)                  (0x06 + ((__B) * 0x10)
#define FPGA_SD_GPIO_LOW_REG(__B)       (0x07 + ((__B) * 0x10)
#define FPGA_SD_GPIO_HIGH_REG(__B)      (0x08 + ((__B) * 0x10)
#define FPGA_ARM_GPIO_REG(__B)          (0x09 + ((__B) * 0x10)
#define FPGA_LED_REG(__B)               (0x0a + ((__B) * 0x10)
#define FPGA_REG11(__B)                 (0x0b + ((__B) * 0x10)
#define FPGA_REG12(__B)                 (0x0c + ((__B) * 0x10)
#define FPGA_REG13(__B)                 (0x0d + ((__B) * 0x10)
#define FPGA_REG14(__B)                 (0x0e + ((__B) * 0x10)
#define FPGA_REG15(__B)                 (0x0f + ((__B) * 0x10)


/* FPGA register bit definitions (incomplete) */

/*!
 * FPGA_TYPE_ID_REG bits
 */
#define FPGA_DEVICE_ID_SHIFT            (0)
#define FPGA_DEVICE_ID_MASK             (0x0f << FPGA_DEVICE_ID_SHIFT)

#endif /* __FPGA_CPE20X_H__ */

