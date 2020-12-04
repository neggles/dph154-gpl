/* -*- C -*-
 * cpetd.h -- definitions for the CPE Timebase Device driver module
 *
 * Copyright (C) 2007 ip.access Ltd
 *
 */

#include <linux/ioctl.h>


#define noCPETD_TEST              1       /* define if doing a build to work with cpetd_test */
#define CPETD_PERIOD            5       /* default device file timebase interval (ms)
                                         * must be multiple of CPETD_BASE_INTERVAL defined
                                         * in cpetd_main.c*/
#define CPETD_MAX_INTERVAL      63      /* maximum timebase interval (period) supported */
#define CPETD_MAX_NUM_DEVICES   4       /* maximum number of devices allowed */


/*
 * Ioctl definitions
 */

/* Use 'K' as magic number */
#define CPETD_IOC_MAGIC    'K'

#define CPETD_IOCRESET     _IO(CPEMSB_IOC_MAGIC, 0)

/*
 * S means "Set" through a ptr,
 * T means "Tell" directly
 * G means "Get" (to a pointed var)
 * Q means "Query", response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */
#define CPETD_IOCTPERIOD        _IO(CPETD_IOC_MAGIC,   1)
#define CPETD_IOCQPERIOD        _IO(CPETD_IOC_MAGIC,   2)
#define CPETD_IOCSETINFO        _IO(CPETD_IOC_MAGIC,   3)    /*sdh fix me: may need modifying */
#define CPETD_IOCGETINFO        _IO(CPETD_IOC_MAGIC,   4)
#define CPETD_IOCGETHIRESINFO   _IO(CPETD_IOC_MAGIC,   5)
#define CPETD_IOCGETPHYSREGINFO _IO(CPETD_IOC_MAGIC,   6)
#define CPETD_IOCPROCESSIDINFO  _IO(CPETD_IOC_MAGIC,   7)

#define CPETD_IOC_MAXNR    7
#define MAX_NUM_OF_PROCESSES  5
#define MAX_NUM_OF_THREADS_PER_PROCESS 30


/* STRUCTURE: ThreadInfoTag
 * data structure to store thread specific information
 */
typedef struct CpetdThreadInfoTag
{
	unsigned int tid;
	unsigned long incompleteTicks;
	long         state;
	int	preemptFlag;
	unsigned int preemptPid;
}CpetdThreadInfo;

/* STRUCTURE: ProcessInfoTag 
 *  data structure used to store process specific information
 * MEMBERS:
 * pid               process identifier
 * sleepTime         time in jiffies for which the process is in sleep mode
 * state             current state of the process
 */
typedef struct CpetdProcessInfoTag
{
        unsigned int numThreads;
	CpetdThreadInfo threadInfo[MAX_NUM_OF_THREADS_PER_PROCESS];
} CpetdProcessInfo;

/* STRUCTURE: CpetdReadInfoV1Tag
 *  data structure info returned by read(). Ordering of members is most important
 *  to least so that read can not incur the overhead of reading unwanted data
 * MEMBERS:
 * kernelDelay   ret time - expected hb (ms)
 * missedBeats   missed heartbeats (ms)
 * userProc      read time minus ret time (ms)
 * flags         e.g. indicates fast return
 * jiffies       jiffies count at ret of read
 * actualBeat    computed beat when the curernt read was supposed to finish.
 */
typedef struct CpetdReadInfoV1Tag
{
    unsigned char kernelDelay;
    unsigned char missedBeats;
    unsigned char userProc;
    unsigned char flags;
    unsigned int jiffies;
    CpetdProcessInfo processInfo[MAX_NUM_OF_PROCESSES];
    unsigned long numRunning;
    int 	  globPreemptFlag;

#ifdef CPETD_TEST
    /* debug & test parameters at end of structure */
    unsigned int actualBeat;
#endif /* CPETD_TEST */

} CpetdReadInfoV1;


/* bit flags for CpetdReadInfoV1Tag.flags */
#define CPETD_READINFO_F_FAST_RETURN     0x01


/* STRUCTURE: CpetdInfoTag
 * Data structure for CPETD_IOCSETINFO/CPETD_IOCGETINFO
 * MEMBERS:
 * flags;        e.g. indicates fast return
 */
typedef struct CpetdInfoTag
{
    unsigned int flags;
} CpetdInfo;

/* bit flags for CpetdInfoTag.flags */
#define CPETD_INFO_F_FAST_RETURN            CPETD_READINFO_F_FAST_RETURN
#define CPETD_INFO_F_VERSION_MASK           0xff000000
#define CPETD_INFO_F_VERSION_1_0            0x01000000
#define CPETD_INFO_F_DEBUG_MASK             0x00ff0000                      /* used for debug and test */
#define CPETD_INFO_F_DEBUG_OFFSET_STEP      0x00010000                      /* used for debug and test */
#define CPETD_INFO_F_DEBUG_OFFSET_RESET     0x00020000                      /* used for debug and test */


/* STRUCTURE: CpetdHiresInfoTag
 * Data structure for CPETD_IOCGETHIRESINFO
 * MEMBERS:
 * flags;        e.g. indicates fast return
 */
typedef struct CpetdHiresInfoTag
{
    unsigned int    lastBeatHiresTime;      /* hires timer value at last beat */
    unsigned int    hiresTicksPerSecond;    /* number of high resoltuion timer ticks in one second */
} CpetdHiresInfo;


/* STRUCTURE: CpetdPhysRegInfoTag
 * Data structure for CPETD_IOCGETPHYSREGINFO
 * MEMBERS:
 * timerRegisters;  The physical address of the Timer subsystem
 * timerCountRegOff;  The offset of the hires current count register
 */
typedef struct CpetdPhysRegInfoTag
{
    unsigned long   timerRegisters;      /* Physical address of the timer registers */
    unsigned long   timerCountRegOff;    /* Physical address of the hires timer */
} CpetdPhysRegInfo;



