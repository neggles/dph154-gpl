/* -*- C -*-
 * cpetd_main.c -- the CPE Timebase Device driver module
 *
 * Document refence list:
 * 1. 3GAP_CPE_HLD_0013_Timebase_Device_V00_03.doc
 *    the design document.
 *
 * Copyright (C) 2007 ip.access Ltd
 *
 * $Id: PC202/kernel/kernel-2.6.28/linux-v2.6.28-ipa-3.2.4-patch 1.4.2.18.1.2 2011/11/29 21:21:59GMT Mark Powell (mp3) Exp  $
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>   /* printk() */
#include <linux/slab.h>     /* kmalloc() */
#include <linux/fs.h>       /* everything... */
#include <linux/errno.h>    /* error codes */
#include <linux/types.h>    /* size_t */
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#if defined(CPETD_ON_ARM)
#include <asm/io.h>
#include <mach/hardware.h>

#if defined(CONFIG_ARCH_FIRECRACKER)
#  include <mach/pc20x/pc20x.h>
#  include <mach/pc20x/timer.h>
#elif defined(CONFIG_ARCH_PC302)
#  include <mach/pc302/pc302.h>
#  include <mach/pc302/timer.h>
#else  /* CONFIG_ARCH_... */
#  error "Unknown architecture"
#endif /* CONFIG_ARCH_... */

#include <mach/platform.h>
#endif  /* defined(CPETD_ON_ARM) */
#include <linux/ipa/cpetd.h>          /* driver interface header file */

#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
#include <mach/fpga_debug.h>
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

/*
 * Configuration definitions
 */
#define USE_KERNEL_TIMER        1
#define CPETD_USE_PROC          1


/*
 * Macros to help debugging
 */

#undef PDEBUG             /* undef it, just in case */
#if defined(CPETD_DEBUG)
#   define PDEBUG(fmt, args...) printk( KERN_DEBUG "cpetd: " fmt, ## args)
#else
#   define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif



#ifdef CONFIG_IPACCESS_CPETD_MODULE
#define CPETD_MAJOR             0           /* dynamic major by default */
#else
#define CPETD_MAJOR           240           /* Fixed major when working with RO filesys and driver in kernel */
#endif
#define CPETD_NUM_DEVS          1           /* only one device by default */

#define CPETD_BASE_INTERVAL     CPETD_PERIOD    /* timebase interrupt period (ms) */
#define CPETD_TIMEBASE_INIT     ((unsigned long)(unsigned int) (-250))  /* start value for the timebase tick
                                                                        * (check wrapping behaviour early!)
                                                                        * (-300*1000) == 5mins before wrap
                                                                        */


#if defined(CPETD_ON_ARM)
/* use PIT4 for free running timer */
#define TIMER_NUM       3

#if defined(CONFIG_ARCH_FIRECRACKER)

#  define TIMER_REG_BASE   PC20X_TIMERS_BASE
#  define TIMER_REG_OFFSET TIMER_N_CURRENT_VALUE_REG_OFFSET(TIMER_NUM)

#  define TIMER_VAL_REG (IO_ADDRESS(PC20X_TIMERS_BASE + TIMER_N_CURRENT_VALUE_REG_OFFSET(TIMER_NUM)))
#  define TIMER_CTL_REG (IO_ADDRESS(PC20X_TIMERS_BASE + TIMER_N_CONTROL_REG_OFFSET(TIMER_NUM)))
#  define TIMER_CNT_REG (IO_ADDRESS(PC20X_TIMERS_BASE + TIMER_N_LOAD_COUNT_REG_OFFSET(TIMER_NUM)))

#  define TIMER_CTL_REG_ENABLE_BIT (TIMER_ENABLE)

#elif defined(CONFIG_ARCH_PC302)

#  define TIMER_REG_BASE   PC302_TIMER_BASE
#  define TIMER_REG_OFFSET TIMERNCURRENTVALUEREGOFFSET(TIMER_NUM)

#  define TIMER_VAL_REG (IO_ADDRESS(PC302_TIMER_BASE + TIMERNCURRENTVALUEREGOFFSET(TIMER_NUM)))
#  define TIMER_CTL_REG (IO_ADDRESS(PC302_TIMER_BASE + TIMERNCONTROLREGOFFSET(TIMER_NUM)))
#  define TIMER_CNT_REG (IO_ADDRESS(PC302_TIMER_BASE + TIMERNLOADCOUNTREGOFFSET(TIMER_NUM)))

#  define TIMER_CTL_REG_ENABLE_BIT (TIMERENABLE)

#else  /* CONFIG_ARCH_... */

#  error "Unknown architecture"

#endif /* CONFIG_ARCH_... */

#endif  /* defined(CPETD_ON_ARM) */



/*---------------------------------------------------------------------------*/

/* STRUCTURE: CpetdDev
 *  Device structure
 * MEMBERS:
 * period
 *   timebase interval in jiffies (must be r/w as an atomic operation)
 * cdev
 *   char dev struct
 */
typedef struct CpetdDevTag
{
    unsigned short  period;
    struct cdev     cdev;
} CpetdDev;


/* STRUCTURE: CpetdFileCtxTag
 *  file structure
 * MEMBERS:
 * dev_p        pointer to associated device
 * info         read info
 * lastReadEnd  time of last return (jiffies). F in ref 1.
 * lastBeat     time (in jiffies) of last beat user woke up on
 * noReadYet    =1 initially, set to 0 after 1st user process read. Used to
 *              stop lastBeat being used until after the first read. This is
 *              done to allow for arbitrarily long delay between opening and reading.
 * offset       used for debug & test. offset is added to jiffies so that the
 *              test application can simulate jiffies wrapping.
 */
typedef struct CpetdFileCtxTag
{
    CpetdDev*           dev_p;
    CpetdReadInfoV1     info;
    unsigned int        lastReadEnd;
    unsigned int        lastBeat;
    unsigned int        flags;
    int                 noReadYet;
    /* debug and test members follow */
#ifdef CPETD_TEST
    unsigned int        offset;
#endif /* CPETD_TEST */
} CpetdFileCtx;

#define CPETD_FIILECTX_F_FAST_RETURN    (1<<0)

/* helper macro to see if wrapped*/
#define CPETD_FIILECTX_FAST_RETURN(ctx_p)   ((ctx_p)->flags & CPETD_FIILECTX_F_FAST_RETURN)
#define CPETD_INFO_FAST_RETURN(ctx_p)       ((ctx_p)->info.flags & CPETD_INFO_F_FAST_RETURN)

/*---------------------------------------------------------------------------*/

/*
 * The different configurable parameters (i.e. can be overridden when driver is installed)
 */
int     cpetdMajor              = CPETD_MAJOR;
int     cpetdNumDevs            = CPETD_NUM_DEVS;   /* number of cpetd devices */
short   cpetdPeriod             = CPETD_PERIOD;


/*
 * tick count used for timebase (1 tick = CPETD_BASE_INTERVAL ms)
 * this MUST be volatile to ensure atomic access.
 * NB timebase instants are now calculated usign jiffies rather than this value. It has been
 *    left in for clarity in debugging, but in theory could be removed.
 */
static volatile unsigned long   cpetdTimebaseTick = CPETD_TIMEBASE_INIT;

/*
 * Value of free-running PIT4 at the last (underlying) timebase instant
 */
static volatile int             cpetdHiresInstant = 0;

/* number of ticks of the high resolution timer per second - default to jiffies rate */
static int                      cpetdHiresResolution = HZ;

#if defined(USE_KERNEL_TIMER)
/*
 * kernel timer used to provide the timebase
 */
struct timer_list   cpetdTimebaseTimer;
#   if defined(CPETD_DEBUG)
unsigned long       cpetdTimerAdded;                /* jiffies value when timer was added */
unsigned long       cpetdTimerNumLate = 0;          /* # times kernel timer was late by >= 1 jiffies value (1 ms) */
unsigned long       cpetdTimerMaxLate = 0;          /* max lateness (in jiffies) observed */
unsigned long       cpetdTimerNumExpired = 0;       /* # times kernel timer has expired when we start it */
unsigned long       cpetdTimerMaxExpired = 0;       /* max time by which timer has already expired */
unsigned long       cpetdTimerNumExpired2 = 0;      /* # times kernel timer has expired when we start it (early calc) */
unsigned long       cpetdTimerMaxExpired2 = 0;      /* max time by which timer has already expired (early calc) */
unsigned long       cpetdTimerNumSlowExec = 0;      /* # times kernel expiry function ran for more than 2 ms */
unsigned long       cpetdTimerMaxExecTime = 0;      /* max time the timer expiry function runs for */
unsigned long       cpetdTimerMaxAddTime = 0;       /* max time the add_timer function runs for */
unsigned long       cpetdTimerMaxWakeupTime = 0;    /* max time the wake_up_interruptible function runs for */
#   endif  /* defined(CPETD_DEBUG) */

#else
#error No mechanismm for counting off the timebase!
#endif

/*
 * Context structures allocated per device (in cpetd_init())
 */
CpetdDev            *cpetdDevices_p = NULL;


/*
 * all processes will block on this queue when they read()
 */
DECLARE_WAIT_QUEUE_HEAD (cpetd_wait_q);


/*---------------------------------------------------------------------------*/

/*
 * export symbols to the kernel
 */
module_param (cpetdMajor, int, 0);
module_param (cpetdNumDevs, int, 0);
module_param (cpetdPeriod, short, 0);

MODULE_AUTHOR ("ip.access");
MODULE_LICENSE ("Dual BSD/GPL");



/*---------------------------------------------------------------------------*/
#ifdef CPETD_USE_PROC /* don't waste space if unused */
/*
 * The proc filesystem: function to read an entry
 */

int cpetd_read_procmem (char    *buf_p,
                        char    **start_p,
                        off_t   offset,
                        int     count,
                        int     *eof_p,
                        void    *data_p)
{
    int                 devNum;
    int                 len = 0;
    int                 limit = count - 80; /* Don't print more than this */
    unsigned short      period;
    CpetdDev            *dev_p;

    len += sprintf (buf_p + len,"\nVersion=0x%08x\n", CPETD_INFO_F_VERSION_1_0);
    len += sprintf (buf_p + len,"TimebaseNow=%ld\n", cpetdTimebaseTick);
    len += sprintf (buf_p + len,"HiresInstant=0x%08x\n", cpetdHiresInstant);
    len += sprintf (buf_p + len,"Hires ticks per second=%d\n", cpetdHiresResolution);
#if defined(USE_KERNEL_TIMER) && defined(CPETD_DEBUG)
    len += sprintf (buf_p + len,"  Num times late=%ld,  max lateness=%ld ms\n",
                    cpetdTimerNumLate, cpetdTimerMaxLate);
    len += sprintf (buf_p + len,"  Num times expired=%ld,  max expired time=%ld ms\n",
                    cpetdTimerNumExpired, cpetdTimerMaxExpired);
    len += sprintf (buf_p + len,"  Num times expired2=%ld,  max expired time2=%ld ms\n",
                    cpetdTimerNumExpired2, cpetdTimerMaxExpired2);
    len += sprintf (buf_p + len,"  Max expiry handler execution time=%ld ms   (Slow %ld times out of %ld)\n",
                    cpetdTimerMaxExecTime, cpetdTimerNumSlowExec, (cpetdTimebaseTick - CPETD_TIMEBASE_INIT));
    len += sprintf (buf_p + len,"  Max add_timer execution time=%ld ms\n",
                    cpetdTimerMaxAddTime);
    len += sprintf (buf_p + len,"  Max wake_up_interruptible execution time=%ld ms\n",
                    cpetdTimerMaxWakeupTime);
#endif  /* defined(USE_KERNEL_TIMER) && defined(CPETD_DEBUG) */

    dev_p = &cpetdDevices_p [0];
    for (devNum = 0; devNum < cpetdNumDevs; devNum++)
    {
        /* retrieve the period of each device and use remainder_table[] for atomicity*/
        period = dev_p->period;
        len += sprintf (buf_p + len,"\nDevice %i: Period %i ms\n", devNum, period);
        if (len > limit)
        {
            break;
        }

        dev_p++;
    }

    *eof_p = 1;
    return len;
}
#endif /* CPETD_USE_PROC */


/*---------------------------------------------------------------------------*/
#if defined(USE_KERNEL_TIMER)
void cpetdTimebaseTimerExpired (unsigned long data)
{
#   if defined(CPETD_DEBUG)
    unsigned long   t0 = jiffies;
    unsigned long   t1;
    unsigned long   t2;
    unsigned long   t3;
    unsigned long   expectedJiffies = cpetdTimerAdded + msecs_to_jiffies(CPETD_BASE_INTERVAL);

    if (time_after (expectedJiffies, t0))
    {
        cpetdTimerNumLate++;
        if (cpetdTimerMaxLate < (expectedJiffies - t0))
        {
            cpetdTimerMaxLate = expectedJiffies - t0;
        }
    }
    if (time_after (t0, cpetdTimebaseTimer.expires))
    {
        cpetdTimerNumExpired2++;
        if (cpetdTimerMaxExpired2 < (t0 - cpetdTimebaseTimer.expires))
        {
            cpetdTimerMaxExpired2 = t0 - cpetdTimebaseTimer.expires;
        }
    }
#   endif  /* defined(CPETD_DEBUG) */

#if defined(CPETD_ON_ARM)
    /* read free-running timer for high resolution value at the time of the instant */
    cpetdHiresInstant = ioread32(__io(TIMER_VAL_REG));
#else
    cpetdHiresInstant = jiffies;
#endif  /* defined(CPETD_ON_ARM) */

    /* this is the next timebase instant! */
    cpetdTimebaseTick++;

    /* restart the timer */
#   if defined(CPETD_DEBUG)
    cpetdTimerAdded = jiffies;                                        /* re-read in case chanegs as this function executes */
    t1 = jiffies;
#   endif   /* defined(CPETD_DEBUG) */
    cpetdTimebaseTimer.expires += msecs_to_jiffies(CPETD_BASE_INTERVAL);
    add_timer (&cpetdTimebaseTimer);

#   if defined(CPETD_DEBUG)
    t2 = jiffies;
    if (time_after (t2, cpetdTimebaseTimer.expires))
    {
        cpetdTimerNumExpired++;
        if (cpetdTimerMaxExpired < (t2 - cpetdTimebaseTimer.expires))
        {
            cpetdTimerMaxExpired = t2 - cpetdTimebaseTimer.expires;
        }
    }
#   endif  /* defined(CPETD_DEBUG) */

    /* kick off all the processes waiting on the timebase */
    wake_up_interruptible (&cpetd_wait_q);

#   if defined(CPETD_DEBUG)
    t3 = jiffies;
    if (cpetdTimerMaxAddTime < (t2 - t1))
    {
        cpetdTimerMaxAddTime = t2 - t1;
    }
    if (cpetdTimerMaxWakeupTime < (t3 - t2))
    {
        cpetdTimerMaxWakeupTime = t3 - t2;
    }
    if (cpetdTimerMaxExecTime < (t3 - t0))
    {
        cpetdTimerMaxExecTime = t3 - t0;
    }
    if (t3 - t0 > msecs_to_jiffies(CPETD_BASE_INTERVAL))
    {
        cpetdTimerNumSlowExec++;
    }
#   endif  /* defined(CPETD_DEBUG) */
}
#endif


/*---------------------------------------------------------------------------*/
/*
 * invoked when user process issues an open() call (or equivalent) on device file
 */
int cpetd_open (struct inode *inode, struct file *filp)
{
    int             result = 0;
    CpetdFileCtx*   ctx_p;      /* file private context */
    CpetdDev*       dev_p;      /* device information */

    /*  Find the device */
    dev_p = container_of (inode->i_cdev, CpetdDev, cdev);

    ctx_p = kmalloc(sizeof(CpetdFileCtx), GFP_KERNEL);
    if(!ctx_p)
    {
        result = -ENOMEM;
    }
    else
    {
        memset(ctx_p , 0, sizeof (CpetdFileCtx));
        ctx_p->lastBeat     = cpetdTimebaseTimer.expires - dev_p->period;
        ctx_p->noReadYet    = 1;
        ctx_p->lastReadEnd  = ctx_p->lastBeat;
        ctx_p->dev_p        = dev_p;
        filp->private_data  = ctx_p;
    }
    return result;
}


/*---------------------------------------------------------------------------*/
/*
 * invoked when user process issues a close() call (or equivalent) on device file
 */
int cpetd_release (struct inode *inode, struct file *filp)
{
    if (filp->private_data)
    {
        kfree (filp->private_data);
        filp->private_data = 0;
    }

    return 0;
}


/*---------------------------------------------------------------------------*/
/* offset used to induce wrap*/
static inline unsigned long get_jiffies(CpetdFileCtx* ctx_p)
{
#ifdef CPETD_TEST
    return jiffies + ctx_p->offset;
#else
    return jiffies;
#endif /* CPETD_TEST */

}


/*---------------------------------------------------------------------------*/
/*
 * invoked when user process issues a read() call (or equivalent) on device file
 */
ssize_t cpetd_read (struct file *filp,
                    char __user *buf_p,
                    size_t      count,
                    loff_t      *f_pos_p)
{
    CpetdFileCtx*       ctx_p = (CpetdFileCtx*) filp->private_data;
    CpetdDev            *dev_p = ctx_p->dev_p;
    unsigned long       readStartTime = get_jiffies(ctx_p);
    unsigned long       readEndTime;
    unsigned long       predictedNextBeat;          /* beat app expected to hit */
    unsigned long       actualNextBeat;             /* beat that app will actual be reactivated on */
    unsigned long       result;
    short               period = dev_p->period;     /* reading period ensure atomicity */
    int                 index, index1 = 0;
    unsigned long num_running;


    /* setup calculation of return data */
    ctx_p->info.flags &= ~CPETD_READINFO_F_FAST_RETURN;

    /* determine if beats have been missed*/
    predictedNextBeat = ctx_p->lastBeat + dev_p->period;
    if (   CPETD_FIILECTX_FAST_RETURN(ctx_p)
        && (ctx_p->noReadYet == 0)
        && time_before_eq (predictedNextBeat, readStartTime)
       )
    {   /* fast return enabled AND user is late for the next beat */
        PDEBUG("cpetd: fast return\n");
        ctx_p->info.flags |= CPETD_READINFO_F_FAST_RETURN;

        /* force user to resync though multiple fast erturns i.e. irrespective of the   */
        /* amount by which user is late, we only increment the beat by one period.      */
        /* The idea is that the user is strictly locked to the absolute timebase.       */
        actualNextBeat = predictedNextBeat;
/*SHH old code - skips beats, not good!        actualNextBeat = readStartTime - ((readStartTime - ctx_p->lastBeat) % period);*/
    }
    else
    {
        /* calculate next timebase instant (in jiffies) for the period we are using,    */
        /* and update timebaseAlignment if wrap                                         */
        actualNextBeat = readStartTime + (period - ((readStartTime - ctx_p->lastBeat) % period));

        /* sleep until the end time is reached */
        while (time_before (get_jiffies(ctx_p), actualNextBeat))
        {
            DEFINE_WAIT (wait);

            if (filp->f_flags & O_NONBLOCK)
            {
                return -EAGAIN;
            }

            prepare_to_wait (&cpetd_wait_q, &wait, TASK_INTERRUPTIBLE);
            if (time_before (get_jiffies(ctx_p), actualNextBeat))
            {   /* didn't reach timer while preparing for wait - so really go to sleep */
                schedule ();
            }
            finish_wait (&cpetd_wait_q, &wait);
            if (signal_pending (current))
            {
                return -ERESTARTSYS;
            }
        }
    }

    ctx_p->noReadYet = 0;

    /* set info.jiffies to now */
    readEndTime         = get_jiffies(ctx_p);
    ctx_p->info.jiffies = readEndTime;

    /* update userProc */
    result = jiffies_to_msecs(readStartTime - ctx_p->lastReadEnd);
    if (result > 255)
    {
        result = 255;
    }
    ctx_p->info.userProc = result;

    /* update kernelDelay */
    result = jiffies_to_msecs(readEndTime - actualNextBeat);
    if (result > 255)
    {
        result = 255;
    }
    ctx_p->info.kernelDelay = result;

    /* update missed beats */
    result = jiffies_to_msecs(actualNextBeat - predictedNextBeat);
    if (result > 255)
    {
        result = 255;
    }
    ctx_p->info.missedBeats = result;

#ifdef CPETD_TEST
    ctx_p->info.actualBeat = actualNextBeat;
#endif /* CPETD_TEST */

    ctx_p->lastReadEnd  = readEndTime;
    ctx_p->lastBeat     = actualNextBeat;
    num_running = ret_nr_running();
    ctx_p->info.numRunning = num_running;
    ctx_p->info.globPreemptFlag = 0;

    for(index = 0; index < MAX_NUM_OF_PROCESSES; ++index)
    {
		for(index1 = 0; index1 < ctx_p->info.processInfo[index].numThreads; ++index1)
		{
			struct task_struct *task_p = find_task_struct_by_pid(ctx_p->info.processInfo[index].threadInfo[index1].tid);
			if(task_p)
			{
				if (task_p->state == TASK_RUNNING)
				{
					/* The thread had more work to do the last time that it ran or it has just woken up.
					 * Update how long this thread has been waiting to run to completion. */
					task_p->incompleteTicks++;
				}
				ctx_p->info.processInfo[index].threadInfo[index1].incompleteTicks = task_p->incompleteTicks;
				ctx_p->info.processInfo[index].threadInfo[index1].state = task_p->state;
				ctx_p->info.processInfo[index].threadInfo[index1].preemptFlag = task_p->preempt_flag;
				ctx_p->info.processInfo[index].threadInfo[index1].preemptPid = task_p->preempt_pid;
				ctx_p->info.globPreemptFlag |= ctx_p->info.processInfo[index].threadInfo[index1].preemptFlag;
			}
		}
    }
    /* return data */
    result = min(count, sizeof(CpetdReadInfoV1));
    if (copy_to_user (buf_p, &ctx_p->info, result))
    {
        return -EFAULT;
    }
    return result;
}


/*---------------------------------------------------------------------------*/
/*
 * The ioctl() implementation
 */

static int cpetd_ioctl_setinfo(CpetdFileCtx* ctx_p, CpetdInfo* info_p)
{
    int ret = 0;

    /* process fast return bit*/
    if (info_p->flags & CPETD_INFO_F_FAST_RETURN)
    {
        /* set fast return mode*/
        ctx_p->flags |= CPETD_FIILECTX_F_FAST_RETURN;
    }
    else
    {
        /* clear fast return mode*/
        ctx_p->flags &= ~CPETD_FIILECTX_F_FAST_RETURN;
    }

#ifdef CPETD_TEST
    /* process debug offset stop*/
    if (info_p->flags & CPETD_INFO_F_DEBUG_OFFSET_STEP)
    {
        /* set offset to cause jiffies + offset to wrap in 10 seconds - ensure multiple of interval */
        ctx_p->offset = ((CPETD_TIMEBASE_INIT - jiffies)/CPETD_BASE_INTERVAL)*CPETD_BASE_INTERVAL;
    }
    /* process debug offset reset*/
    if (info_p->flags & CPETD_INFO_F_DEBUG_OFFSET_RESET)
    {
        /* set offset to cause jiffies + offset to wrap in 10 seconds */
        ctx_p->offset = 0;
    }
#endif /* CPETD_TEST */

    return ret;
}


int cpetd_ioctl (struct inode   *inode_p,
                 struct file    *filp,
                 unsigned int   cmd,
                 unsigned long  arg)
{
    int             ret = 0;
    CpetdDev*       dev_p;
    CpetdFileCtx*   ctx_p;
    int index, index1 = 0;
    int i, j = 0;

    /* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
    if (_IOC_TYPE(cmd) != CPETD_IOC_MAGIC) return -ENOTTY;
    if (_IOC_NR(cmd) > CPETD_IOC_MAXNR) return -ENOTTY;

    ctx_p = filp->private_data;
    dev_p = ctx_p->dev_p;

    switch (cmd)
    {
    case CPETD_IOCTPERIOD: /* Tell: arg is the value */
        if ((arg < CPETD_BASE_INTERVAL) || (arg > CPETD_MAX_INTERVAL))
        {
            ret = -EINVAL;
        }
        else
        {
            dev_p->period = msecs_to_jiffies((unsigned int) arg);
        }
        break;

    case CPETD_IOCQPERIOD: /* Query: return it (it's positive) */
        ret = jiffies_to_msecs(dev_p->period);
        break;

    case CPETD_IOCSETINFO:
    {
        CpetdInfo       info;

        if (copy_from_user(&info, (void*) arg, sizeof(info)))
        {
            ret = -EFAULT;
        }
        else
        {
            ret = cpetd_ioctl_setinfo(ctx_p, &info);
        }
        break;
    }

    case CPETD_IOCGETINFO:
    {
        CpetdInfo       info;

        memset(&info, 0, sizeof(info));

        if(ctx_p->flags & CPETD_FIILECTX_F_FAST_RETURN)
        {
            info.flags |= CPETD_INFO_F_FAST_RETURN;
        }
        else
        {
            info.flags &= ~CPETD_INFO_F_FAST_RETURN;
        }
        info.flags |= CPETD_INFO_F_VERSION_1_0;
        if(copy_to_user((void*) arg, &info, sizeof(info)))
        {
            ret = -EFAULT;
        }
        break;
    }

    case CPETD_IOCGETHIRESINFO:
    {
        CpetdHiresInfo  hiresInfo;

        memset(&hiresInfo, 0, sizeof(hiresInfo));

        hiresInfo.lastBeatHiresTime     = cpetdHiresInstant;
        hiresInfo.hiresTicksPerSecond   = cpetdHiresResolution;
        if(copy_to_user((void*) arg, &hiresInfo, sizeof(hiresInfo)))
        {
            ret = -EFAULT;
        }
        break;
    }

    case CPETD_IOCGETPHYSREGINFO:
    {
        CpetdPhysRegInfo  physRegInfo;

        memset(&physRegInfo, 0, sizeof(physRegInfo));

        physRegInfo.timerRegisters   = TIMER_REG_BASE;
        physRegInfo.timerCountRegOff = TIMER_REG_OFFSET;
        if(copy_to_user((void*) arg, &physRegInfo, sizeof(physRegInfo)))
        {
            ret = -EFAULT;
        }
        break;
    }
    case CPETD_IOCPROCESSIDINFO:
    {
	CpetdProcessInfo procInfo[MAX_NUM_OF_PROCESSES];

	memset(&procInfo, 0, (MAX_NUM_OF_PROCESSES * sizeof(CpetdProcessInfo)));

	if(copy_from_user(&procInfo, (void*) arg, (MAX_NUM_OF_PROCESSES * sizeof(CpetdProcessInfo))))
	{
            ret = -EFAULT;
	}

	for(index = 0; index < MAX_NUM_OF_PROCESSES; ++index)
	{
		ctx_p->info.processInfo[index].numThreads = procInfo[index].numThreads;

		for(index1 = 0; index1 < ctx_p->info.processInfo[index].numThreads; ++index1)
		{
			ctx_p->info.processInfo[index].threadInfo[index1].tid = procInfo[index].threadInfo[index1].tid;
		}
	}

	for(i=0; i < MAX_NUM_OF_PROCESSES; ++i)
        {
		printk(KERN_NOTICE "\nNumber of threads are: %d\n", ctx_p->info.processInfo[i].numThreads);
		for(j = 0; j < ctx_p->info.processInfo[i].numThreads; ++j)
		{
			struct task_struct *task_p = NULL;
			
			printk(KERN_NOTICE "\nThe tID is %d\n", ctx_p->info.processInfo[i].threadInfo[j].tid);
			
			task_p = find_task_struct_by_pid(ctx_p->info.processInfo[i].threadInfo[j].tid);
			if(task_p)
			{
				task_p->incompleteTicks = 0;
			}
		}
    	}
	
	break;
    }

    default:  /* redundant, as cmd was checked against MAXNR */
        ret = -ENOTTY;
    }

    return ret;
}


/*---------------------------------------------------------------------------*/
/*
 * The fops - this structure tells the kernel which functiosn to invoke to
 * handle user process access to the device file
 */
struct file_operations cpetdFops =
{
    .owner =     THIS_MODULE,
    .read =      cpetd_read,
    .ioctl =     cpetd_ioctl,
    .open =      cpetd_open,
    .release =   cpetd_release,
};


/*---------------------------------------------------------------------------*/
/*
 * Initialise a character device kernel structure
 */
static void cpetd_setup_cdev (CpetdDev *dev_p, int index)
{
    int     err;
    int     devno = MKDEV (cpetdMajor, index);

    cdev_init (&dev_p->cdev, &cpetdFops);
    dev_p->cdev.owner = THIS_MODULE;
    dev_p->cdev.ops = &cpetdFops;
    err = cdev_add (&dev_p->cdev, devno, 1);
    /* Fail gracefully if need be */
    if (err)
    {
        PDEBUG("Error %d adding cpetd%d", err, index);
    }
}

#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
static unsigned long cpetd_timestamp(void)
{
    return ioread32(__io(TIMER_VAL_REG));
}
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

/*---------------------------------------------------------------------------*/
/*
 * Invoked by the kernel when the device driver is loaded (insmod)
 */
int cpetd_init (void)
{
    int         result;
    int         devNum;
    dev_t       dev = MKDEV (cpetdMajor, 0);
    CpetdDev    *dev_p;


    PDEBUG("initialising\n");

    /*
     * check the parameters exported by the module for some sort of sanity
     */
    if ((cpetdPeriod < CPETD_BASE_INTERVAL) || (cpetdPeriod > CPETD_MAX_INTERVAL))
    {
        printk (KERN_NOTICE "cpetd: Period is out of range %d to %d (%d)",
                            CPETD_BASE_INTERVAL, CPETD_MAX_INTERVAL, cpetdPeriod);
        return -EINVAL;
    }
    /* check load module param cpetdPeriod is multiple of baseline period */
    if (cpetdPeriod % CPETD_BASE_INTERVAL != 0)
    {
        printk (KERN_NOTICE "cpetd: load module param for period (%d) is not multiple of baseline period (%d)",
                            cpetdPeriod, CPETD_BASE_INTERVAL);
        return -EINVAL;
    }

    if ((cpetdNumDevs < 1) || (cpetdNumDevs > CPETD_MAX_NUM_DEVICES))
    {
        printk (KERN_NOTICE "cpetd: Num devices is out of range 1 to %d (%d)",
                            CPETD_MAX_NUM_DEVICES, cpetdNumDevs);
        return -EINVAL;
    }

    /*
     * Register our major device number, and accept a dynamic number.
     */
    if (cpetdMajor)
    {
        result = register_chrdev_region (dev, cpetdNumDevs, "cpetd");
    }
    else
    {
        result = alloc_chrdev_region (&dev, 0, cpetdNumDevs, "cpetd");
        cpetdMajor = MAJOR (dev);
    }
    if (result < 0)
    {
        return result;
    }

    /*
     * allocate the devices -- we can't have them static, as the number
     * can be specified at load time
     */
    cpetdDevices_p = kmalloc (cpetdNumDevs*sizeof (CpetdDev), GFP_KERNEL);
    if (!cpetdDevices_p)
    {
        result = -ENOMEM;
        goto fail_malloc;
    }
    memset (cpetdDevices_p, 0, cpetdNumDevs*sizeof (CpetdDev));

    cpetdTimebaseTick = CPETD_TIMEBASE_INIT;
#if defined(USE_KERNEL_TIMER)
    init_timer (&cpetdTimebaseTimer);

#   if defined(CPETD_DEBUG)
    cpetdTimerAdded = jiffies;
#   endif   /* defined(CPETD_DEBUG) */
    cpetdTimebaseTimer.data = 0;
    cpetdTimebaseTimer.function = cpetdTimebaseTimerExpired;
    cpetdTimebaseTimer.expires = jiffies + msecs_to_jiffies(CPETD_BASE_INTERVAL);
    add_timer (&cpetdTimebaseTimer);
#endif

    dev_p = &cpetdDevices_p [0];
    for (devNum = 0; devNum < cpetdNumDevs; devNum++)
    {
        dev_p->period = msecs_to_jiffies((unsigned int) cpetdPeriod);

        cpetd_setup_cdev (dev_p, devNum);

        dev_p++;
    }

#if defined(CPETD_ON_ARM)
#if defined(CONFIG_MACH_PC72052_I10_REVA)
        cpetdHiresResolution = PC72052_I10_REVA_TIMER_FREQ;
#elif defined(CONFIG_MACH_PC72052_I10_REVB)
        cpetdHiresResolution = PC72052_I10_REVB_TIMER_FREQ;
#elif defined(CONFIG_MACH_PC7302)
        cpetdHiresResolution = PC302_TIMER_FREQ;
#else
#  error "Unknown architecture"
#endif

#else
        cpetdHiresResolution = HZ;
#endif  /* defined(CPETD_ON_ARM) */

#if defined(CPETD_ON_ARM)
    /*-----------------------------------------------------------------------*/
    /* stop the timer */
    iowrite32(0x0, __io(TIMER_CTL_REG));

    /* load the count */
    iowrite32(0xffffffff, __io(TIMER_CNT_REG));

    /* start the timer in free-running mode */
    iowrite32(TIMER_CTL_REG_ENABLE_BIT, __io(TIMER_CTL_REG));
    /*-----------------------------------------------------------------------*/
#endif  /* defined(CPETD_ON_ARM) */


#ifdef CPETD_USE_PROC /* only when available */
    create_proc_read_entry ("cpetdmem", 0, NULL, cpetd_read_procmem, NULL);
#endif

#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
    ipaHiresTimerPtr = cpetd_timestamp;
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

    return 0; /* succeed */

fail_malloc:
    unregister_chrdev_region (dev, cpetdNumDevs);
    return result;
}


/*---------------------------------------------------------------------------*/
/*
 * Invoked by the kernel when the device driver is unloaded (rmmod)
 */
void cpetd_cleanup (void)
{
    int     devNum;

#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
    ipaHiresTimerPtr = NULL;
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

#ifdef CPETD_USE_PROC
    remove_proc_entry ("cpetdmem", NULL);
#endif

    del_timer (&cpetdTimebaseTimer);

    for (devNum = 0; devNum < cpetdNumDevs; devNum++)
    {
        cdev_del (&cpetdDevices_p [devNum].cdev);
    }

    kfree (cpetdDevices_p);

    unregister_chrdev_region (MKDEV (cpetdMajor, 0), cpetdNumDevs);
}


/*---------------------------------------------------------------------------*/
/*
 * Export device driver main entry points.
 */
module_init (cpetd_init);
module_exit (cpetd_cleanup);
