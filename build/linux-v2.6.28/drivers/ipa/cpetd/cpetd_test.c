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
 * test application for cpetd
 *
 * This test application was run on x86 to verify the behaviour of the driver. These results
 * are recorded in cpetd_test_log_i386.txt for periods 5ms < period < 62ms.
 *****************************************************************************
 * Filename: 
 *****************************************************************************/


#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/ipa/cpetd.h>          /* driver interface header file */

static char* read_mode[] = { "non fast return", "fast return"};



int main(int argc, char* argv[])
{
    char*                       mode = NULL;
    int                         verbose = 0;
    int                         fd = -1;
    int                         ret = 0;
    int                         limit = 0;
    int                         i = 0;
    int                         version = 0;
    unsigned long               lastjiffies = 0;
    unsigned int                period;
    CpetdReadInfoV1             info;
    CpetdInfo                   ioctl_info;
    CpetdHiresInfo              ioctl_hires_info;

    /* debug & test parameters at end of structure */
    unsigned int                readEnd = 0;
    unsigned int                readEndPrev = 0;

    memset(&info, 0, sizeof(info));
    memset(&ioctl_info, 0, sizeof(ioctl_info));

    fd = open("/dev/cpetd0", O_SYNC | O_RDONLY);
    if(fd < 0) {
        printf("error: failed to open device.\n");
        goto out0;
    }

    ret = ioctl(fd, CPETD_IOCGETINFO, &ioctl_info);
    if(ret < 0) {
        printf("error: CPETD_IOCGETINFO ioctl failed.\n");
        goto out1;
    }
    version = (ioctl_info.flags & CPETD_INFO_F_VERSION_MASK) == CPETD_INFO_F_VERSION_1_0 ? 1: -1;
    printf("version=%d\n", version);

    ret = ioctl(fd, CPETD_IOCGETHIRESINFO, &ioctl_hires_info);
    if(ret < 0) {
        printf("error: CPETD_IOCGETHIRESINFO hires ioctl failed.\n");
        goto out1;
    }
    printf("hires resolution=%d ticks per sec, last beat @ 0x%08x\n",
           ioctl_hires_info.hiresTicksPerSecond,
           ioctl_hires_info.lastBeatHiresTime);

    for(period = CPETD_PERIOD; period <= CPETD_MAX_INTERVAL; period += CPETD_PERIOD)
    {
        /* set the period */
        ret = ioctl(fd, CPETD_IOCTPERIOD, period);
        if(ret < 0) {
            printf("error: CPETD_IOCTPERIOD ioctl failed.\n");
            goto out1;
        }

        /* step the offset for inducing wrap */
        memset(&ioctl_info, 0, sizeof(ioctl_info));
        ioctl_info.flags = CPETD_INFO_F_DEBUG_OFFSET_STEP;
        ret = ioctl(fd, CPETD_IOCSETINFO, &ioctl_info);
        if(ret < 0)
        {
            printf("error: CPETD_IOCSETINFO ioctl failed.\n");
            goto out1;
        }

        printf("=================================================================================\n");
        printf("Period=%d ms\n", period);
        printf("readEnd     jiffies     kDelay      missedBeat  userProc    flag        delta_jiff  readEnd dt  \n");
        printf("=======     =======     ======      ==========  ========    ====        ==========  ==========  \n");

        /* loop on timebase */
        for(i=0;i< (500/CPETD_PERIOD);i++)
        {
            /* test blocking mode */
            ret = read(fd, (void*) &info, sizeof(info));
            if(ret < 0)
            {
                printf("error: to read device.\n");
                goto out1;
            }
            if(ret != sizeof(info))
            {
                printf("error: to read correct number of bytes (%d).\n", sizeof(info));
                goto out1;
            }
            /* report once a second */
#ifdef CPETD_TEST
            /* debug & test parameters at end of structure */
            readEnd = info.actualBeat;
#endif /* CPETD_TEST */
            printf("0x%8x  0x%8x  0x%8x  0x%8x  0x%8x  0x%8x  0x%8x  0x%8x\n",
                readEnd, info.jiffies, info.kernelDelay, info.missedBeats, info.userProc,
                info.flags, info.jiffies - lastjiffies, readEnd-readEndPrev);
            lastjiffies = info.jiffies;

            readEndPrev = readEnd;
        }
    }
    /* now perform fast return test */
    period = 5;
    printf("=================================================================================\n");
    printf("==== Fast Return Test ===========================================================\n");
    printf("Period=%d ms\n", period);
    printf("jiffies     kDelay      missedBeat  userProc    flag        delta_jiff\n");
    printf("=======     ======      ==========  ========    ====        ==========\n");
    /* set the period */
    ret = ioctl(fd, CPETD_IOCTPERIOD, period);
    if(ret < 0) {
        printf("error: CPETD_IOCTPERIOD ioctl failed.\n");
        goto out1;
    }
    memset(&ioctl_info, 0, sizeof(ioctl_info));
    ioctl_info.flags = CPETD_INFO_F_FAST_RETURN;
    ret = ioctl(fd, CPETD_IOCSETINFO, &ioctl_info);
    if(ret < 0) {
        printf("error: CPETD_IOCSETINFO ioctl failed.\n");
        goto out1;
    }
    for(i=0; i < 100; i++)
    {
        /* test fast return mode mode */
        ret = read(fd, (void*) &info, sizeof(info));
        if(ret < 0)
        {
            printf("error: to read device.\n");
            goto out1;
        }
        if(ret != sizeof(info))
        {
            printf("error: to read correct number of bytes (%d).\n", sizeof(info));
            goto out1;
        }

        mode = info.userProc > period ? read_mode[1] : read_mode[0];
        /* report once a second */
        printf("0x%8x  0x%8x  0x%8x  0x%8x  0x%8x  0x%8x  %s\n",
            info.jiffies, info.kernelDelay, info.missedBeats, info.userProc,
            info.flags, info.jiffies - lastjiffies, mode);

        /* induce a sleep longer than the period */
        if(i % 2 == 0) usleep((period*1000) + (period*500));
    }

out1:
    if(fd) close(fd);
out0:
    return 0;
}
