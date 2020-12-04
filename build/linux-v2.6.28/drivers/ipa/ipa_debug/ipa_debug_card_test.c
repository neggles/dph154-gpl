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

unsigned short testVector[] = 
{
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
//     0xBEEF, 0xC0DE, 0xBEEF, 0xC0DE,
//     0xBEEF, 0xC0DE, 0xBEEF, 0xC0DE,
//     0xBEEF, 0xC0DE, 0xBEEF, 0xC0DE,
//     0xBEEF, 0xC0DE, 0xBEEF, 0xC0DE,
//     0xABAB, 0xABAB, 0xABAB, 0xABAB,    
//     0xABAB, 0xABAB, 0xABAB, 0xABAB,    
//     0xABAB, 0xABAB, 0xABAB, 0xABAB,    
//     0xABAB, 0xABAB, 0xABAB, 0xABAB,    
    0x8000, 0x4000, 0x2000, 0x1000,
    0x0800, 0x0400, 0x0200, 0x0100,
    0x0080, 0x0040, 0x0020, 0x0010,
    0x0008, 0x0004, 0x0002, 0x0001,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x8000, 0x4000, 0x2000, 0x1000,
    0x0800, 0x0400, 0x0200, 0x0100,
    0x0080, 0x0040, 0x0020, 0x0010,
    0x0008, 0x0004, 0x0002, 0x0001,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0001, 0x0002, 0x0004, 0x0008,
//     0xBEEF, 0xC0DE, 0xBEEF, 0xC0DE,
//     0xBEEF, 0xC0DE, 0xBEEF, 0xC0DE,
//     0xBEEF, 0xC0DE, 0xBEEF, 0xC0DE,
//     0xBEEF, 0xC0DE, 0xBEEF, 0xC0DE,
//     0xABAB, 0xABAB, 0xABAB, 0xABAB,    
//     0xABAB, 0xABAB, 0xABAB, 0xABAB,    
//     0xABAB, 0xABAB, 0xABAB, 0xABAB,    
//     0xABAB, 0xABAB, 0xABAB, 0xABAB,    
    0x8000, 0x4000, 0x2000, 0x1000,
    0x0800, 0x0400, 0x0200, 0x0100,
    0x0080, 0x0040, 0x0020, 0x0010,
    0x0008, 0x0004, 0x0002, 0x0001,
};

int main(int argc, char* argv[])
{
    int maxPid = 1;
    int verbose = 0;

    if (argc > 1)
    {
        maxPid = atoi(argv[1]);
    }
    if (maxPid < 1 || maxPid > 16)
    {
        maxPid = 1;
    }
    if (argc > 2)
    {
        verbose = atoi(argv[2]);
    }

    if (openIpaDebugCard() == 0)
    {
        int pid = 0;
        for (pid = 0; pid < maxPid; pid++)
        {
            if (writeIpaDebugMessage(pid, testVector, sizeof(testVector)/sizeof(unsigned short)) > 0)
            {
                if (verbose)
                {
                    int index = 0;
                    printf("For PID %u, wrote: \n",pid);
                    for (index = 0; index < sizeof(testVector)/sizeof(unsigned short); index++)
                    {
                        printf(" 0x%04X", testVector[index]);
                        if (((index+1)%4) == 0)
                        {
                            printf("\n");
                        }
                    }
                    printf("\n");
                }
            }
        }
        closeIpaDebugCard();
    }
    return 0;
}
