/* Copyright (c) 2008 picoChip Designs Ltd.
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef PC302_UART_H
#define PC302_UART_H

/* Constants ---------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define UART_RX_BUFFER_REG_OFFSET               0x00
#define UART_TX_HOLDING_REG_OFFSET		0x00
#define UART_DIVISOR_LOW_REG_OFFSET		0x00
#define UART_DIVISOR_HIGH_REG_OFFSET		0x04
#define UART_INT_ENABLE_REG_OFFSET		0x04
#define UART_INT_IDENTITY_REG_OFFSET		0x08
#define UART_FIFO_CTRL_REG_OFFSET		0x08
#define UART_LINE_CTRL_REG_OFFSET		0x0c
#define UART_MODEM_CTRL_REG_OFFSET		0x10
#define UART_LINE_STATUS_REG_OFFSET		0x14
#define UART_MODEM_STATUS_REG_OFFSET		0x18
#define UART_SCRATCH_REG_OFFSET			0x1c

#define UART_UART_STATUS_REG_OFFSET             0x7c

#define UART_SHADOW_RTS_REG_OFFSET		0x8c
#define UART_SHADOW_BRK_CTRL_REG_OFFSET		0x90
#define UART_SHADOW_DMA_MODE_REG_OFFSET		0x94
#define UART_SHADOW_FIFO_ENABLE_REG_OFFSET	0x98
#define UART_SHADOW_RCVR_TRIGGER_REG_OFFSET	0x9c
#define UART_SHADOW_TX_EMPTY_TRIGGER_REG_OFFSET	0xa0
#define UART_HALT_TX_REG_OFFSET			0xa4
#define UART_DMA_SOFT_ACK_REG_OFFSET		0xa8
#define UART_COMP_PARAM_REG_OFFSET		0xf4
#define UART_UART_VERSION_REG_OFFSET		0xf8
#define UART_COMP_TYPE_REG_OFFSET		0xfc

/*****************************************************************************/
/* Register Reset Values                                                     */
/*****************************************************************************/

#define UART_COMP_PARAM_REG_RESET               0x00021102 
#define UART_UART_VERSION_REG_RESET             0x3330372A 
#define UART_COMP_TYPE_REG_RESET                0x44570110 

/* DLL & DLH */
#define UARTDIVISORMASK                 0xFF

/* IER */ 
#define UARTINTENABLEPTIMEIDX		0x7	/* Programmable THRE Interrupt Mode Enable that can be */
						/*  written to only when THRE_MODE_USER == Enabled, */
						/*  always readable. This is used to enable/disable the */
						/*  generation of THRE Interrupt. */
#define UARTINTENABLEEDSSIIDX           0x3     /*  R/W Enable Modem Status Interrupt. */
#define UARTINTENABLEELSIIDX            0x2     /*  R/W Enable Receiver Line Status Interrupt. */
#define UARTINTENABLEETBEIIDX           0x1     /*  R/W Enable Transmit Holding Register Empty Interrupt. */
#define UARTINTENABLEERBFIIDX           0x0     /*  R/W Enable Received Data Available Interrupt. */

#define UARTINTENABLEPTIMEMASK          (1 << UARTINTENABLEPTIMEIDX)
#define UARTINTENABLEEDSSIMASK          (1 << UARTINTENABLEEDSSIIDX)            
#define UARTINTENABLEELSIMASK           (1 << UARTINTENABLEELSIIDX)             
#define UARTINTENABLEETBEIMASK          (1 << UARTINTENABLEETBEIIDX)            
#define UARTINTENABLEERBFIMASK          (1 << UARTINTENABLEERBFIIDX)            
	
/* IIR */
#define UARTINTIDENTITYMASK	        0x0F
#define UARTINTIDENTITYMODEMSTATUS      0x00
#define UARTINTIDENTITYNONE	        0x01
#define UARTINTIDENTITYTHREMPTY	        0x02
#define UARTINTIDENTITYRXDATA	        0x04
#define UARTINTIDENTITYRXLINESTATUS     0x06
#define UARTINTIDENTITYBUSYDETECT       0x07
#define UARTINTIDENTITYCHARACTERTIMEOUT 0x0C

/* FCR */
#define UARTFIFOCTRLRCVRMASK	        0xC0
#define UARTFIFOCTRLRCVR1CHAR	        0x00    /* 00 = 1 character in the FIFO */
#define UARTFIFOCTRLRCVRQUARTERFULL     0x40    /* 01 = FIFO ¼ full */
#define UARTFIFOCTRLRCVRHALFFULL        0x80    /* 10 = FIFO ½ full */
#define UARTFIFOCTRLRCVR2LESSTHANFULL   0xC0    /* 11 = FIFO 2 less than full */

#define UARTFIFOCTRLTXEMPTYMASK	        0x30
#define UARTFIFOCTRLTXEMPTYEMPTY        0x00    /* 00 = FIFO empty */
#define UARTFIFOCTRLTXEMPTY2CHARS       0x10    /* 01 = 2 characters in the FIFO */
#define UARTFIFOCTRLTXEMPTYQUARTERFULL  0x20    /* 10 = FIFO ¼ full */
#define UARTFIFOCTRLTXEMPTYHALFFULL     0x30    /* 11 = FIFO ½ full */

#define UartFIFOCtrlEnable		0x01	/* fifo enable bit */


/* LCR indices */
#define UARTLINECTRLDLABIDX             7       /* Divisor latch access bit */
#define UARTLINECTRLBRKIDX              6 
#define UARTLINECTRLEPSIDX              4 
#define UARTLINECTRLPENIDX              3 
#define UARTLINECTRLSTOPIDX             2 
#define UARTLINECTRLDLSIDX              0 

/* LCR bit masks */
#define UARTLINECTRLDLABMASK	        (1 << UARTLINECTRLDLABIDX)
#define UARTLINECTRLBRKMASK	        (1 << UARTLINECTRLBRKIDX)      
#define UARTLINECTRLEPSMASK	        (1 << UARTLINECTRLEPSIDX)      
#define UARTLINECTRLPENMASK	        (1 << UARTLINECTRLPENIDX)      
#define UARTLINECTRLSTOPMASK            (1 << UARTLINECTRLSTOPIDX)
#define UARTLINECTRLDLSMASK	        3 
#define UARTLINECTRLDLS8BITS	        3       /* 8 bit data length */
#define UARTLINECTRLDLS7BITS	        2       /* 7 bit data length */
#define UARTLINECTRLDLS6BITS	        1       /* 6 bit data length */
#define UARTLINECTRLDLS5BITS	        0       /* 5 bit data length */

#define UARTLINECTRL1STOPBIT            ~(UARTLINECTRLSTOPMASK)
#define UARTLINECTRLPARITYDISABLE       ~(UARTLINECTRLPENMASK)

/* MCR indices */
#define UARTMODEMCTRLLOOPBACKIDX	4
#define UARTMODEMCTRLOUT2IDX		3
#define UARTMODEMCTRLOUT1IDX		2
#define UARTMODEMCTRLCTSIDX		1
#define UARTMODEMCTRLDTRIDX		0

/* MCR bit masks */
#define UARTMODEMCTRLLOOPBACKMASK	(1 << UARTMODEMCTRLLOOPBACKIDX)
#define UARTMODEMCTRLOUT2MASK		(1 << UARTMODEMCTRLOUT2IDX)    
#define UARTMODEMCTRLOUT1MASK		(1 << UARTMODEMCTRLOUT1IDX)    
#define UARTMODEMCTRLCTSMASK		(1 << UARTMODEMCTRLCTSIDX)     
#define UARTMODEMCTRLDTRMASK		(1 << UARTMODEMCTRLDTRIDX)     

#define UARTMODEMSTATUSCTSIDX           4       /* CTS input */
#define UARTMODEMSTATUSDCTSIDX          0       /* change in CTS input since last read */

#define UARTLINESTATUSTHREIDX	        5
#define UARTLINESTATUSDATAREADYIDX      0
#define UARTLINESTATUSTHREMASK	        (1 << UARTLINESTATUSTHREIDX)
#define UARTLINESTATUSDATAREADYMASK     (1 << UARTLINESTATUSDATAREADYIDX)

#define UARTUARTSTATUSRFFIDX            4       /* RX FIFO full */
#define UARTUARTSTATUSRFNEIDX	        3       /* RX FIFO not empty */
#define UARTUARTSTATUSTFEIDX	        2 
#define UARTUARTSTATUSTFNFIDX	        1 
#define UARTUARTSTATUSBUSYIDX	        0 

#define UARTUARTSTATUSRFFMASK           (1 << UARTUARTSTATUSRFFIDX)    
#define UARTUARTSTATUSRFNEMASK          (1 << UARTUARTSTATUSRFNEIDX)
#define UARTUARTSTATUSTFEMASK           (1 << UARTUARTSTATUSTFEIDX)    
#define UARTUARTSTATUSTFNFMASK          (1 << UARTUARTSTATUSTFNFIDX)
#define UARTUARTSTATUSBUSYMASK          (1 << UARTUARTSTATUSBUSYIDX)

#endif /* PC302_UART_H */
