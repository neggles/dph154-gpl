/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright(C) 2006 picoChip(R) Designs Ltd.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*****************************************************************************
 *
 * Description: picoChip PC20x ARM Subsystem
 *
 *              UART Register Definitions
 *
 *****************************************************************************/
#ifndef __PC20X_UART_H
#define __PC20X_UART_H

/* Register address offsets from UART Base */
#define UartRXBufferRegOffset		0x00
#define UartTXHoldingRegOffset		0x00
#define UartDivisorLowRegOffset		0x00
#define UartDivisorHighRegOffset	0x04
#define UartIntEnableRegOffset		0x04
#define UartIntIdentityRegOffset	0x08
#define UartFIFOCtrlRegOffset		0x08
#define UartLineCtrlRegOffset		0x0c
#define UartModemCtrlRegOffset		0x10
#define UartLineStatusRegOffset		0x14
#define UartModemStatusRegOffset	0x18
#define UartScratchRegOffset		0x1c

#define UartShadowRXBufferRegOffset	0x30 /* not available in this implementation */
#define UartShadowTXHoldingRegOffset	0x30 /* not available in this implementation */

#define UartFIFOAccessRegOffset		0x70 /* not available in this implementation */
#define UartTXFIFOReadRegOffset		0x74 /* not available in this implementation */
#define UartRXFIFOWriteRegOffset	0x78 /* not available in this implementation */
#define UartUartStatusRegOffset		0x7c
#define UartTXFIFOLevelRegOffset	0x80 /* not available in this implementation */
#define UartRXFIFOLevelRegOffset	0x84 /* not available in this implementation */
#define UartSoftResetRegOffset		0x88 /* not available in this implementation */


#define UartShadowRTSRegOffset			0x8c
#define UartShadowBrkCtrlRegOffset		0x90
#define UartShadowDMAModeRegOffset		0x94
#define UartShadowFIFOEnableRegOffset		0x98
#define UartShadowRCVRTriggerRegOffset		0x9c
#define UartShadowTXEmptyTriggerRegOffset	0xa0
#define UartHaltTXRegOffset			0xa4
#define UartDMASoftAckRegOffset			0xa8
#define UartCompParamRegOffset			0xf4
#define UartUartVersionRegOffset		0xf8
#define UartCompTypeRegOffset			0xfc

/* DLL & DLH */
#define UartDivisorMask                 0xFF

/* IER */ 
#define UartIntEnablePTIMEIdx		0x7	/* Programmable THRE Interrupt Mode Enable that can be */
						/*  written to only when THRE_MODE_USER == Enabled, */
						/*  always readable. This is used to enable/disable the */
						/*  generation of THRE Interrupt. */
#define UartIntEnableEDSSIIdx		0x3	/*  R/W Enable Modem Status Interrupt. */
#define UartIntEnableELSIIdx		0x2	/*  R/W Enable Receiver Line Status Interrupt. */
#define UartIntEnableETBEIIdx		0x1	/*  R/W Enable Transmit Holding Register Empty Interrupt. */
#define UartIntEnableERBFIIdx		0x0	/*  R/W Enable Received Data Available Interrupt. */

#define UartIntEnablePTIMEMask		(1 << UartIntEnablePTIMEIdx)
#define UartIntEnableEDSSIMask          (1 << UartIntEnableEDSSIIdx)		
#define UartIntEnableELSIMask		(1 << UartIntEnableELSIIdx)    		
#define UartIntEnableETBEIMask		(1 << UartIntEnableETBEIIdx)		
#define UartIntEnableERBFIMask		(1 << UartIntEnableERBFIIdx)		
	
/* IIR */
#define UartIntIdentityMask		0x0F
#define UartIntIdentityModemStatus	0x00
#define UartIntIdentityNone		0x01
#define UartIntIdentityTHREmpty		0x02
#define UartIntIdentityRXData		0x04
#define UartIntIdentityRXLineStatus 	0x06
#define UartIntIdentityBusyDetect 	0x07
#define UartIntIdentityCharacterTimeout	0x0C

/* FCR */
#define UartFIFOCtrlRCVRMask		0xC0
#define UartFIFOCtrlRCVR1Char		0x00
#define UartFIFOCtrlRCVRQuarterFull	0x40
#define UartFIFOCtrlRCVRHalfFull	0x80
#define UartFIFOCtrlRCVR2LessThanFull	0xC0

#define UartFIFOCtrlTXEmptyMask		0x30
#define UartFIFOCtrlTXEmptyEmpty	0x00
#define UartFIFOCtrlTXEmpty2Chars	0x10
#define UartFIFOCtrlTXEmptyQuarterFull	0x20
#define UartFIFOCtrlTXEmptyHalfFull	0x30

#define UartFIFOCtrlEnable		0x01	/* fifo enable bit */


/* LCR indices */
#define UartLineCtrlDLABIdx		7	/* Divisor latch access bit */
#define UartLineCtrlBrkIdx		6 
#define UartLineCtrlEPSIdx		4 
#define UartLineCtrlPENIdx		3 
#define UartLineCtrlStopIdx		2 
#define UartLineCtrlDLSIdx		0 

/* LCR bit masks */
#define UartLineCtrlDLABMask		(1 << UartLineCtrlDLABIdx)
#define UartLineCtrlBrkMask		(1 << UartLineCtrlBrkIdx)      
#define UartLineCtrlEPSMask		(1 << UartLineCtrlEPSIdx)      
#define UartLineCtrlPENMask		(1 << UartLineCtrlPENIdx)      
#define UartLineCtrlStopMask            (1 << UartLineCtrlStopIdx)
#define UartLineCtrlDLSMask		3 
#define UartLineCtrlDLS8bits		3	/* 8 bit data length */
#define UartLineCtrlDLS7bits		2	/* 7 bit data length */
#define UartLineCtrlDLS6bits		1	/* 6 bit data length */
#define UartLineCtrlDLS5bits		0	/* 5 bit data length */

#define UartLineCtrl1StopBit            ~(UartLineCtrlStopMask)
#define UartLineCtrlParityDisable       ~(UartLineCtrlPENMask)


/* MCR indices */
#define UartModemCtrlLoopBackIdx	4
#define UartModemCtrlOut2Idx		3
#define UartModemCtrlOut1Idx		2
#define UartModemCtrlCTSIdx		1
#define UartModemCtrlDTRIdx		0

/* MCR bit masks */
#define UartModemCtrlLoopBackMask	(1 << UartModemCtrlLoopBackIdx)
#define UartModemCtrlOut2Mask		(1 << UartModemCtrlOut2Idx)    
#define UartModemCtrlOut1Mask		(1 << UartModemCtrlOut1Idx)    
#define UartModemCtrlCTSMask		(1 << UartModemCtrlCTSIdx)     
#define UartModemCtrlDTRMask		(1 << UartModemCtrlDTRIdx)     

#define UartModemStatusCTSIdx		4	/* CTS input */
#define UartModemStatusDCTSIdx		0	/* change in CTS input since last read */

#define UartLineStatusTHREIdx		5
#define UartLineStatusDataReadyIdx	0
#define UartLineStatusTHREMask		(1 << UartLineStatusTHREIdx)
#define UartLineStatusDataReadyMask	(1 << UartLineStatusDataReadyIdx)

#define UartUartStatusRFFIdx		4	/* RX FIFO full */
#define UartUartStatusRFNEIdx		3	/* RX FIFO not empty */
#define UartUartStatusTFEIdx		2 
#define UartUartStatusTFNFIdx		1 
#define UartUartStatusBusyIdx		0 

#define UartUartStatusRFFMask		(1 << UartUartStatusRFFIdx)    
#define UartUartStatusRFNEMask		(1 << UartUartStatusRFNEIdx)
#define UartUartStatusTFEMask		(1 << UartUartStatusTFEIdx)    
#define UartUartStatusTFNFMask		(1 << UartUartStatusTFNFIdx)
#define UartUartStatusBusyMask		(1 << UartUartStatusBusyIdx)

#endif /* __PC20X_UART_H */
