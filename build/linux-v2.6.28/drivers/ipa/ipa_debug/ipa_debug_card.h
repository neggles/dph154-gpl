/******************************************************************************
 *
 * Copyright (c) 2009 ip.access Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * 22 Apr 2009 Created by Simon D Hughes.
 *
 * Debug card driver API.  Communicates with FPGA on debug card via EBI.
 *****************************************************************************
 * Filename: 
 *****************************************************************************/

#ifndef IPA_DEBUG_CARD_H
#define IPA_DEBUG_CARD_H

/* Must be called before attempting to write debug messages,
 * only needs to be called once.
 * Returns 1 if it fails to open the device.
 */
int openIpaDebugCard(void);

/* May be called to close the device.
 */
void closeIpaDebugCard(void);

/* Pushes a message to the FPGA to be sent out of the USB tracing port.
 *   pid may be 0-15, is used to identify the process.
 *   message is an array of 16bit words to be sent as payload.
 *   length is the size of the message in 16bit words.
 */
int  writeIpaDebugMessage(unsigned char pid, unsigned short* message, unsigned short length);

#endif
