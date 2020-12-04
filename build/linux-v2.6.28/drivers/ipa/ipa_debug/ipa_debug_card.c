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

#include "ipa_debug_card.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static int ipaDebugDevice = -1;

#define MAX_MESSAGE_SIZE 1024

/* Must be called before attempting to write debug messages,
 * only needs to be called once.
 * Returns FALSE if it fails to open the device.
 */
int openIpaDebugCard(void)
{
    if (ipaDebugDevice == -1)
    {
        ipaDebugDevice = open("/dev/ipa_debug0", O_SYNC | O_WRONLY);
    }
    return (ipaDebugDevice == -1);
}

/* May be called to close the device.
 */
void closeIpaDebugCard(void)
{
    if (ipaDebugDevice != -1)
    {
        close(ipaDebugDevice);
    }
}

/* Pushes a message to the FPGA to be sent out of the USB tracing port.
 *   pid may be 0-15, is used to identify the process.
 *   message is an array of 16bit words to be sent as payload.
 *   length is the size of the message in 16bit words.
 */
int  writeIpaDebugMessage(unsigned char pid, unsigned short* message, unsigned short length)
{
    int wordsWritten = 0;

    if ((pid <= 15) &&
        (message != 0) && 
        (length > 0) && 
        ((length*2+2) < MAX_MESSAGE_SIZE))
    {
        unsigned char messageBuffer[MAX_MESSAGE_SIZE];
        int index = 0;
        unsigned short pidBit = 0;
        /* pack pid */
        pidBit = 1 << pid;
        messageBuffer[0] = (pidBit >> 8);
        messageBuffer[1] = (pidBit & 0x00ff);
        /* pack message words */
        for (index = 2; index <= (length*2); index += 2)
        {
            messageBuffer[index]   = (*message >> 8);
            messageBuffer[index+1] = (*message & 0x00ff);
            message++;
            wordsWritten++;
        }
        if (write(ipaDebugDevice, messageBuffer, (length*2+2)) != (length*2+2))
        {
            wordsWritten = 0;
        }
        fsync(ipaDebugDevice);
    }
    return wordsWritten;
}


