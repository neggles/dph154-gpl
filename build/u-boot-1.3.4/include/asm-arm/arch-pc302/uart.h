/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file uart.h
* \brief Definitions for the PC302 UART Block.
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_UART_H
#define PC302_UART_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Constants --------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define UART_RX_BUFFER_REG_OFFSET               (0x00)
#define UART_TX_HOLDING_REG_OFFSET		(0x00)
#define UART_DIVISOR_LOW_REG_OFFSET		(0x00)
#define UART_DIVISOR_HIGH_REG_OFFSET		(0x04)
#define UART_INT_ENABLE_REG_OFFSET		(0x04)
#define UART_INT_IDENTITY_REG_OFFSET		(0x08)
#define UART_FIFO_CTRL_REG_OFFSET		(0x08)
#define UART_LINE_CTRL_REG_OFFSET		(0x0c)
#define UART_MODEM_CTRL_REG_OFFSET		(0x10)
#define UART_LINE_STATUS_REG_OFFSET		(0x14)
#define UART_MODEM_STATUS_REG_OFFSET		(0x18)
#define UART_SCRATCH_REG_OFFSET			(0x1c)

#define UART_UART_STATUS_REG_OFFSET             (0x7c)

#define UART_SHADOW_RTS_REG_OFFSET		(0x8c)
#define UART_SHADOW_BRK_CTRL_REG_OFFSET		(0x90)
#define UART_SHADOW_DMA_MODE_REG_OFFSET		(0x94)
#define UART_SHADOW_FIFO_ENABLE_REG_OFFSET	(0x98)
#define UART_SHADOW_RCVR_TRIGGER_REG_OFFSET	(0x9c)
#define UART_SHADOW_TX_EMPTY_TRIGGER_REG_OFFSET	(0xa0)
#define UART_HALT_TX_REG_OFFSET			(0xa4)
#define UART_DMA_SOFT_ACK_REG_OFFSET		(0xa8)
#define UART_COMP_PARAM_REG_OFFSET		(0xf4)
#define UART_UART_VERSION_REG_OFFSET		(0xf8)
#define UART_COMP_TYPE_REG_OFFSET		(0xfc)

/* Macros ------------------------------------------------------------------ */

/* DLL & DLH */
#define UART_DIVISOR_MASK               (0xFF)

/* IER */
#define UART_INT_ENABLE_PTIME_IDX	(0x7)	/* Programmable THRE Interrupt Mode Enable that can be */
						/*  written to only when THRE_MODE_USER == Enabled, */
						/*  always readable. This is used to enable/disable the */
						/*  generation of THRE Interrupt. */
#define UART_INT_ENABLE_EDSSI_IDX       (0x3)   /*  R/W Enable Modem Status Interrupt. */
#define UART_INT_ENABLE_ELSI_IDX        (0x2)   /*  R/W Enable Receiver Line Status Interrupt. */
#define UART_INT_ENABLE_ETBEI_IDX       (0x1)   /*  R/W Enable Transmit Holding Register Empty Interrupt. */
#define UART_INT_ENABLE_ERBFI_IDX       (0x0)   /*  R/W Enable Received Data Available Interrupt. */

#define UART_INT_ENABLE_PTIME_MASK          (1 << UART_INT_ENABLE_PTIME_IDX)
#define UART_INT_ENABLE_EDSSI_MASK          (1 << UART_INT_ENABLE_EDSSI_IDX)
#define UART_INT_ENABLE_ELSI_MASK           (1 << UART_INT_ENABLE_ELSI_IDX)
#define UART_INT_ENABLE_ETBEI_MASK          (1 << UART_INT_ENABLE_ETBEI_IDX)
#define UART_INT_ENABLE_ERBFI_MASK          (1 << UART_INT_ENABLE_ERBFI_IDX)

/* IIR */
#define UART_INT_IDENTITY_MASK	            (0x0F)
#define UART_INT_IDENTITY_MODEM_STATUS      (0x00)
#define UART_INT_IDENTITY_NONE	            (0x01)
#define UART_INT_IDENTITY_THR_EMPTY	    (0x02)
#define UART_INT_IDENTITY_RX_DATA	    (0x04)
#define UART_INT_IDENTITY_RX_LINE_STATUS    (0x06)
#define UART_INT_IDENTITY_BUSY_DETECT       (0x07)
#define UART_INT_IDENTITY_CHARACTER_TIMEOUT (0x0C)

/* FCR */
#define UART_FIFO_CTRL_RCVR_MASK	        (0xC0)
#define UART_FIFO_CTRL_RCVR1_CHAR	        (0x00)  /* 00 = 1 character in the FIFO */
#define UART_FIFO_CTRL_RCVR_QUARTER_FULL        (0x40)  /* 01 = FIFO ¼ full */
#define UART_FIFO_CTRL_RCVR_HALF_FULL           (0x80)  /* 10 = FIFO ½ full */
#define UART_FIFO_CTRL_RCVR_2_LESS_THAN_FULL    (0xC0)  /* 11 = FIFO 2 less than full */

#define UART_FIFO_CTRL_TX_EMPTY_MASK	        (0x30)
#define UART_FIFO_CTRL_TX_EMPTY_EMPTY           (0x00)  /* 00 = FIFO empty */
#define UART_FIFO_CTRL_TX_EMPTY_2_CHARS         (0x10)  /* 01 = 2 characters in the FIFO */
#define UART_FIFO_CTRL_TX_EMPTY_QUARTER_FULL    (0x20)  /* 10 = FIFO ¼ full */
#define UART_FIFO_CTRL_TX_EMPTY_HALF_FULL       (0x30)  /* 11 = FIFO ½ full */

#define UART_FIFO_CTRL_ENABLE		        (0x01)	/* fifo enable bit */


/* LCR indices */
#define UART_LINE_CTRL_DLAB_IDX                 (7)     /* Divisor latch access bit */
#define UART_LINE_CTRL_BRK_IDX                  (6)
#define UART_LINE_CTRL_EPS_IDX                  (4)
#define UART_LINE_CTRL_PEN_IDX                  (3)
#define UART_LINE_CTRL_STOP_IDX                 (2)
#define UART_LINE_CTRL_DLS_IDX                  (0)

/* LCR bit masks */
#define UART_LINE_CTRL_DLAB_MASK	        (1 << UART_LINE_CTRL_DLAB_IDX)
#define UART_LINE_CTRL_BRK_MASK	                (1 << UART_LINE_CTRL_BRK_IDX)
#define UART_LINE_CTRL_EPS_MASK	                (1 << UART_LINE_CTRL_EPS_IDX)
#define UART_LINE_CTRL_PEN_MASK	                (1 << UART_LINE_CTRL_PEN_IDX)
#define UART_LINE_CTRL_STOP_MASK                (1 << UART_LINE_CTRL_STOP_IDX)
#define UART_LINE_CTRL_DLS_MASK	                (3)
#define UART_LINE_CTRL_DLS_8BITS	        (3)     /* 8 bit data length */
#define UART_LINE_CTRL_DLS_7BITS	        (2)     /* 7 bit data length */
#define UART_LINE_CTRL_DLS_6BITS	        (1)     /* 6 bit data length */
#define UART_LINE_CTRL_DLS_5BITS	        (0)     /* 5 bit data length */

#define UART_LINE_CTRL_1STOP_BIT                (~(UART_LINE_CTRL_STOP_MASK))
#define UART_LINE_CTRL_PARITY_DISABLE           (~(UART_LINE_CTRL_PEN_MASK))

/* MCR indices */
#define UART_MODEM_CTRL_LOOPBACK_IDX            (4)
#define UART_MODEM_CTRL_OUT2_IDX		(3)
#define UART_MODEM_CTRL_OUT1_IDX		(2)
#define UART_MODEM_CTRL_CTS_IDX		        (1)
#define UART_MODEM_CTRL_DTR_IDX		        (0)

/* MCR bit masks */
#define UART_MODEM_CTRL_LOOPBACK_MASK	        (1 << UART_MODEM_CTRL_LOOPBACK_IDX)
#define UART_MODEM_CTRL_OUT2_MASK		(1 << UART_MODEM_CTRL_OUT2_IDX)
#define UART_MODEM_CTRL_OUT1_MASK		(1 << UART_MODEM_CTRL_OUT1_IDX)
#define UART_MODEM_CTRL_CTS_MASK		(1 << UART_MODEM_CTRL_CTS_IDX)
#define UART_MODEM_CTRL_DTR_MASK		(1 << UART_MODEM_CTRL_DTR_IDX)

#define UART_MODEM_STATUS_CTS_IDX               (4)     /* CTS input */
#define UART_MODEM_STATUS_DCTS_IDX              (0)     /* change in CTS input since last read */

#define UART_LINE_STATUS_THRE_IDX	        (5)
#define UART_LINE_STATUS_DATA_READY_IDX         (0)
#define UART_LINE_STATUS_THRE_MASK	        (1 << UART_LINE_STATUS_THRE_IDX)
#define UART_LINE_STATUS_DATA_READY_MASK        (1 << UART_LINE_STATUS_DATA_READY_IDX)

#define UART_UART_STATUS_RFF_IDX                (4)     /* RX FIFO full */
#define UART_UART_STATUS_RFNE_IDX	        (3)     /* RX FIFO not empty */
#define UART_UART_STATUS_TFE_IDX	        (2)
#define UART_UART_STATUS_TFNF_IDX	        (1)
#define UART_UART_STATUS_BUSY_IDX	        (0)

#define UART_UART_STATUS_RFF_MASK               (1 << UART_UART_STATUS_RFF_IDX)
#define UART_UART_STATUS_RFNE_MASK              (1 << UART_UART_STATUS_RFNE_IDX)
#define UART_UART_STATUS_TFE_MASK               (1 << UART_UART_STATUS_TFE_IDX)
#define UART_UART_STATUS_TFNF_MASK              (1 << UART_UART_STATUS_TFNF_IDX)
#define UART_UART_STATUS_BUSY_MASK              (1 << UART_UART_STATUS_BUSY_IDX)

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_UART_H */
