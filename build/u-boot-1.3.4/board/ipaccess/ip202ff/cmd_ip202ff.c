/*
 * (C) Copyright 2007 ip.access Ltd
*/

/*
 * IP202FF platform timing/performance tests
 *
 * These tests are implemented as Board Specific U-boot commands.
 *
 */

#include <common.h>
#include <command.h>
#include <asm/arch/pc20x.h>
#include <asm/arch/gpio.h>
#include <asm/arch/timer.h>
#include <asm/arch/memif.h>


#if defined(CONFIG_CMD_BSP)

/* number of iteratios to run the read/write tests for */
#define NUM_TEST_ITERATIONS     100000


#define IOWRITE16(vALUE, aDDRESS)   *((volatile u16 *)(aDDRESS)) = (vALUE)
#define IOREAD16(aDDRESS)           (*((volatile u16 *)(aDDRESS)))
#define IOWRITE32(vALUE, aDDRESS)   *((volatile u32 *)(aDDRESS)) = (vALUE)
#define IOREAD32(aDDRESS)           (*((volatile u32 *)(aDDRESS)))


#define LED1_GPIO_NUM       2
#define LED1_DATA_REG       (PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset)
#define LED1_DIR_REG        (PC20X_GPIO_BASE + GpioPortADataDirectionRegOffset)

#define LED2_GPIO_NUM       5
#define LED2_DATA_REG       (PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset)
#define LED2_DIR_REG        (PC20X_GPIO_BASE + GpioPortADataDirectionRegOffset)

#define TIMER_NUM           1


typedef enum MemTypeTag
{
    MT_SDRAM = 0,
    MT_SRAM,
    MT_FLASH,

    NUM_MTS,

    MT_FULL_TEST
}
MemType;


/*#define SHH_TEST*/
#if defined(SHH_TEST)
static void cp_delay (void)
{
    volatile int i;

    /* copro seems to need some delay between reading and writing */
    for (i = 0; i < 100; i++);
}

/* read co-processor 15, register #1 (control register) */
static unsigned long read_p15_c1 (void)
{
    unsigned long value;

    __asm__ __volatile__(
        "mrc    p15, 0, %0, c1, c0, 0   @ read control reg\n"
        : "=r" (value)
        :
        : "memory");

    return value;
}

/* write to co-processor 15, register #1 (control register) */
static void write_p15_c1 (unsigned long value)
{
    __asm__ __volatile__(
        "mcr    p15, 0, %0, c1, c0, 0   @ write it back\n"
        :
        : "r" (value)
        : "memory");

    read_p15_c1 ();
}

/* See also ARM926EJ-S Technical Reference Manual */
#define C1_MMU      (1<<0)      /* mmu off/on */
#define C1_ALIGN    (1<<1)      /* alignment faults off/on */
#define C1_DC       (1<<2)      /* dcache off/on */

#define C1_BIG_ENDIAN   (1<<7)      /* big endian off/on */
#define C1_SYS_PROT (1<<8)      /* system protection */
#define C1_ROM_PROT (1<<9)      /* ROM protection */
#define C1_IC       (1<<12)     /* icache off/on */
#define C1_HIGH_VECTORS (1<<13)     /* location of vectors: low/high addresses */

void dcache_enable (void)
{
    ulong reg;

    reg = read_p15_c1 ();       /* get control reg. */
    cp_delay ();
    write_p15_c1 (reg | C1_DC);
}

#endif  /* defined(SHH_TEST) */



/*===========================================================================*/
/* run the timcal command.
 * This drives a LED low then high after a specified number of milliseconds.
 * The tester should hook up the LED to a scope (or other timing device to
 * time the pulse width.
 */
int do_timcal (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    int             guardCount;
    u32             gpioData;
    u32             millisecondsForTest = 20;

#if defined(SHH_TEST)
printf ("Calling dcache_enable...\n");   /* SHH delete me!!! */
dcache_enable ();   /* SHH delete me!!! */
#endif  /* defined(SHH_TEST) */

    switch (argc)
    {
    case 1:
    case 2:
        if (argc == 2)
        {
            int     dummy = simple_strtoul (argv[1], NULL, 10);
            if ((dummy > 0) && (dummy <= 1000))
            {
                millisecondsForTest = dummy;
            }
            else
            {
                printf ("Bad milliseconds for test parameter - using %d\n", millisecondsForTest);
            }
        }

        /* stop the timer */
        IOWRITE32(0x0, PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

        /* load the count */
        IOWRITE32((PC20X_AHB_CLOCK_FREQ/(1000/millisecondsForTest)),
                  PC20X_TIMER_BASE + TimerNLoadCountRegOffset(TIMER_NUM));

        /* set GPIO line to output for LED */
        gpioData = IOREAD32(LED1_DIR_REG);
        gpioData |= Gpio(LED1_GPIO_NUM);
        IOWRITE32(gpioData, LED1_DIR_REG);

        /* ensure LED is off */
        gpioData = IOREAD32(LED1_DATA_REG);
        gpioData &= ~Gpio(LED1_GPIO_NUM);
        IOWRITE32(gpioData, LED1_DATA_REG);

        guardCount = 1000000;
        /* turn LED on */
        gpioData |= Gpio(LED1_GPIO_NUM);
        IOWRITE32(gpioData, LED1_DATA_REG);

        /* start the timer in user defined count mode */
        IOWRITE32((TimerEnable|TimerMode), PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

        while (--guardCount > 0)
        {
            if (IOREAD32(PC20X_TIMER_BASE + TimerNInterruptStatusRegOffset(TIMER_NUM)) != 0)
            {
                IOREAD32(PC20X_TIMER_BASE + TimerNEOIRegOffset(TIMER_NUM));
                break;
            }
        }

        /* turn LED off */
        gpioData &= ~Gpio(LED1_GPIO_NUM);
        IOWRITE32(gpioData, LED1_DATA_REG);

        if (guardCount <= 0)
        {
            printf ("ERROR: Failed to detect end of timer count period within a reasonable length of time!\n");
        }
        else
        {
            printf ("LED should have been turned on for %dms (use scope to check!)\n", millisecondsForTest);
        }
        return 0;

    default:
        printf ("Usage:\n%s%s\n", cmdtp->usage, cmdtp->help);
        return 1;
    }

    return 0;
}


/*===========================================================================*/
#if defined(OLD_STYLE_TESTS)
/*---------------------------------------------------------------------------*/
static unsigned long mem_write_test (unsigned char *mem_p, int testDurationMs)
{
    unsigned long     numAccesses = 0;
    u32     count = (PC20X_AHB_CLOCK_FREQ/1000) * testDurationMs;
    int     guardCount = 100000000;
    volatile u32 *m_p = (volatile u32 *)mem_p;
    int     iStatus = icache_status ();

    if (! iStatus)
    {
        printf ("Enabling I-Cache\n");
        icache_enable ();
    }


    /* stop the timer */
    IOWRITE32(0x0, PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

    /* load the count */
    IOWRITE32(count, PC20X_TIMER_BASE + TimerNLoadCountRegOffset(TIMER_NUM));

    /* start the timer in user defined count mode */
    IOWRITE32((TimerEnable|TimerMode), PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

    while (--guardCount > 0)
    {
        if (IOREAD32(PC20X_TIMER_BASE + TimerNInterruptStatusRegOffset(TIMER_NUM)) != 0)
        {
            IOREAD32(PC20X_TIMER_BASE + TimerNEOIRegOffset(TIMER_NUM));
            break;
        }

        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;

        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;

        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;

        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;
        *m_p = 0x12345678;

        numAccesses += 64;
    }

    if (! iStatus)
    {
        printf ("Disabling I-Cache\n");
        icache_disable ();
    }

    if (guardCount <= 0)
    {
        printf ("ERROR: Failed to detect end of timer count period within a reasonable length of time!\n");
    }

    return numAccesses;
}

/*---------------------------------------------------------------------------*/
static unsigned long mem_read_test (unsigned char *mem_p, int testDurationMs)
{
    unsigned long     numAccesses = 0;
    u32     count = (PC20X_AHB_CLOCK_FREQ/1000) * testDurationMs;
    int     guardCount = 100000000;
    volatile u32 *m_p = (volatile u32 *)mem_p;
    int     iStatus = icache_status ();

    if (! iStatus)
    {
        printf ("Enabling I-Cache\n");
        icache_enable ();
    }

    /* stop the timer */
    IOWRITE32(0x0, PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

    /* load the count */
    IOWRITE32(count, PC20X_TIMER_BASE + TimerNLoadCountRegOffset(TIMER_NUM));

    /* start the timer in user defined count mode */
    IOWRITE32((TimerEnable|TimerMode), PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

    while (--guardCount > 0)
    {
        if (IOREAD32(PC20X_TIMER_BASE + TimerNInterruptStatusRegOffset(TIMER_NUM)) != 0)
        {
            IOREAD32(PC20X_TIMER_BASE + TimerNEOIRegOffset(TIMER_NUM));
            break;
        }

        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;

        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;

        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;

        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;
        *m_p;

        numAccesses += 64;
    }

    if (! iStatus)
    {
        printf ("Disabling I-Cache\n");
        icache_disable ();
    }

    if (guardCount <= 0)
    {
        printf ("ERROR: Failed to detect end of timer count period within a reasonable length of time!\n");
    }

    return numAccesses;
}

/*---------------------------------------------------------------------------*/
static void printTestResult (char *memType_p, unsigned long memAddr, int reading, int testDurationMs, unsigned long numAccesses)
{
    unsigned long   throughPut;
    unsigned long   integral, decimal;

    throughPut = ((((unsigned long)numAccesses)*4*8)/testDurationMs) + 5;

    /* convert to 2 decimal places */
    integral = throughPut/1000;
    decimal = (throughPut % 1000)/10;

    printf ("%-6s (0x%08lx): %10ld %s in %6d milliseconds. Throughput= %3ld.%02ld Mbps\n",
            memType_p,
            memAddr,
            numAccesses,
            (reading ? "Reads " : "Writes"),
            testDurationMs,
            integral,
            decimal);
}

/*---------------------------------------------------------------------------*/
static unsigned long sdram_test (int reading, int testDurationMs)
{
    unsigned long     numAccesses = 0;
    u32     *mem_p = (u32 *)0x04000000;     /* spare SDRAM bank */

    if (reading)
    {
        numAccesses = mem_read_test ((unsigned char *)mem_p, testDurationMs);
    }
    else
    {
        numAccesses = mem_write_test ((unsigned char *)mem_p, testDurationMs);
    }

    if (numAccesses <= 0)
    {
        printf ("SDRAM: No accesses - oops!\n");
    }
    else
    {
        printTestResult ("SDRAM", (unsigned long)mem_p, reading, testDurationMs, numAccesses);
    }

    return numAccesses;
}

/*---------------------------------------------------------------------------*/
static unsigned long sram_test (int reading, int testDurationMs)
{
    unsigned long     numAccesses = 0;

    if (reading)
    {
        numAccesses = mem_read_test ((unsigned char *)PC20X_ONCHIP_SRAM_BASE, testDurationMs);
    }
    else
    {
        numAccesses = mem_write_test ((unsigned char *)PC20X_ONCHIP_SRAM_BASE, testDurationMs);
    }

    if (numAccesses <= 0)
    {
        printf ("SRAM: No accesses - oops!\n");
    }
    else
    {
        printTestResult ("SRAM", PC20X_ONCHIP_SRAM_BASE, reading, testDurationMs, numAccesses);
    }

    return numAccesses;
}

/*---------------------------------------------------------------------------*/
static unsigned long flash_test (int reading, int testDurationMs)
{
    unsigned long     numAccesses = 0;

    if (reading)
    {
        numAccesses = mem_read_test ((unsigned char *)CFG_LOAD_ADDR, testDurationMs);
    }
    else
    {
        printf ("Flash WRITING not (yet) supported!\n");
    }

    if (numAccesses <= 0)
    {
        printf ("FLASH: No accesses - oops!\n");
    }
    else
    {
        printTestResult ("FLASH", CFG_LOAD_ADDR, reading, testDurationMs, numAccesses);
    }

    return numAccesses;
}

#else
/*---------------------------------------------------------------------------*/
static unsigned long mem2write_test (unsigned char *mem_p, int numIterations, int dummyRun)
{
    volatile u32    *m_p = (volatile u32 *)mem_p;
    int             iStatus = icache_status ();
    int             i;
    unsigned long   startCount = 0xffffffff;
    unsigned long   endCount;

    if (! iStatus)
    {
        printf ("Enabling I-Cache\n");
        icache_enable ();
    }


    /* stop the timer */
    IOWRITE32(0x0, PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

    /* load the count */
    IOWRITE32(startCount, PC20X_TIMER_BASE + TimerNLoadCountRegOffset(TIMER_NUM));

    i = numIterations;

    /* start the timer in user defined count mode */
    IOWRITE32((TimerEnable|TimerMode), PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

    while (--i > 0)
    {
        if (! dummyRun)
        {
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;

            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;

            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;

            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
            *m_p = 0x12345678;
        }
    }

    endCount = IOREAD32(PC20X_TIMER_BASE + TimerNCurrentValueRegOffset(TIMER_NUM));

    if (IOREAD32(PC20X_TIMER_BASE + TimerNInterruptStatusRegOffset(TIMER_NUM)) != 0)
    {
        printf ("WARNING: PIT looks likes it has wrapped!\n");
    }

    if (! iStatus)
    {
        printf ("Disabling I-Cache\n");
        icache_disable ();
    }

printf ("TickCount=0x%08lx\n", (startCount - endCount));
    return (startCount - endCount);
}

/*---------------------------------------------------------------------------*/
static unsigned long mem2read_test (unsigned char *mem_p, int numIterations, int dummyRun)
{
    volatile u32    *m_p = (volatile u32 *)mem_p;
    int             iStatus = icache_status ();
    int             i;
    unsigned long   startCount = 0xffffffff;
    unsigned long   endCount;

    if (! iStatus)
    {
        printf ("Enabling I-Cache\n");
        icache_enable ();
    }

    /* stop the timer */
    IOWRITE32(0x0, PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

    /* load the count */
    IOWRITE32(startCount, PC20X_TIMER_BASE + TimerNLoadCountRegOffset(TIMER_NUM));

    i = numIterations;

    /* start the timer in user defined count mode */
    IOWRITE32((TimerEnable|TimerMode), PC20X_TIMER_BASE + TimerNControlRegOffset(TIMER_NUM));

    while (--i > 0)
    {
        if (! dummyRun)
        {
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;

            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;

            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;

            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
            *m_p;
        }
    }

    endCount = IOREAD32(PC20X_TIMER_BASE + TimerNCurrentValueRegOffset(TIMER_NUM));

    if (IOREAD32(PC20X_TIMER_BASE + TimerNInterruptStatusRegOffset(TIMER_NUM)) != 0)
    {
        printf ("WARNING: PIT looks likes it has wrapped!\n");
    }

    if (! iStatus)
    {
        printf ("Disabling I-Cache\n");
        icache_disable ();
    }

printf ("TickCount=0x%08lx\n", (startCount - endCount));
    return (startCount - endCount);
}

/*---------------------------------------------------------------------------*/
static void printTest2Result (char *memType_p, unsigned long memAddr, int reading, int numIterations, unsigned long tickCount)
{
    unsigned long   throughPut;
    unsigned long   integral, decimal;

    throughPut = (((((unsigned long)numIterations)*64*8*4) / (tickCount/1000)) * (PC20X_AHB_CLOCK_FREQ/1000)) + 5;
    throughPut /= 1000;

    /* convert to 2 decimal places */
    integral = throughPut/1000;
    decimal = (throughPut % 1000)/10;

    printf ("%-6s (0x%08lx): %10ld %s in %6ld ms. Throughput= %3ld.%02ld Mbps\n",
            memType_p,
            memAddr,
            ((unsigned long)numIterations*64),
            (reading ? "Reads " : "Writes"),
            tickCount/(PC20X_AHB_CLOCK_FREQ/1000),
            integral,
            decimal);
}

/*---------------------------------------------------------------------------*/
static unsigned long sdram2test (int reading, int numIterations)
{
    unsigned long     tickCount = 0;
    unsigned long     tickCount2 = 0;
    u32     *mem_p = (u32 *)0x04000000;     /* spare SDRAM bank */

    if (reading)
    {
        tickCount = mem2read_test ((unsigned char *)mem_p, numIterations, 0);
        tickCount2 = mem2read_test ((unsigned char *)mem_p, numIterations, 1);
    }
    else
    {
        tickCount = mem2write_test ((unsigned char *)mem_p, numIterations, 0);
        tickCount2 = mem2write_test ((unsigned char *)mem_p, numIterations, 1);
    }

    if (tickCount <= 0)
    {
        printf ("SDRAM: No accesses - oops!\n");
    }
    else
    {
        printTest2Result ("SDRAM", (unsigned long)mem_p, reading, numIterations, (tickCount - tickCount2));
    }

    return tickCount;
}

/*---------------------------------------------------------------------------*/
static unsigned long sram2test (int reading, int numIterations)
{
    unsigned long     tickCount = 0;
    unsigned long     tickCount2 = 0;

    if (reading)
    {
        tickCount = mem2read_test ((unsigned char *)PC20X_ONCHIP_SRAM_BASE, numIterations, 0);
        tickCount2 = mem2read_test ((unsigned char *)PC20X_ONCHIP_SRAM_BASE, numIterations, 1);
    }
    else
    {
        tickCount = mem2write_test ((unsigned char *)PC20X_ONCHIP_SRAM_BASE, numIterations, 0);
        tickCount2 = mem2write_test ((unsigned char *)PC20X_ONCHIP_SRAM_BASE, numIterations, 1);
    }

    if (tickCount <= 0)
    {
        printf ("SRAM: No accesses - oops!\n");
    }
    else
    {
        printTest2Result ("SRAM", (unsigned long)PC20X_ONCHIP_SRAM_BASE, reading, numIterations, (tickCount - tickCount2));
    }

    return tickCount;
}

/*---------------------------------------------------------------------------*/
static unsigned long flash2test (int reading, int numIterations)
{
    unsigned long     tickCount = 0;
    unsigned long     tickCount2 = 0;

    if (reading)
    {
        tickCount = mem2read_test ((unsigned char *)CFG_LOAD_ADDR, numIterations, 0);
        tickCount2 = mem2read_test ((unsigned char *)CFG_LOAD_ADDR, numIterations, 1);
    }
    else
    {
        printf ("Flash WRITING not (yet) supported!\n");
    }

    if (tickCount <= 0)
    {
        printf ("FLASH: No accesses - oops!\n");
    }
    else
    {
        printTest2Result ("FLASH", (unsigned long)CFG_LOAD_ADDR, reading, numIterations, (tickCount - tickCount2));
    }

    return tickCount;
}

#endif  /* defined(OLD_STYLE_TESTS) */


/*---------------------------------------------------------------------------*/
/* run the memcal command.
 * This runs a number of read or write access as fast as it can in a set time period.
 */
int do_memcal (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
#if defined(OLD_STYLE_TESTS)
    u32             testDurationMs = 2000;
#endif  /* defined(OLD_STYLE_TESTS) */
    int             argsOK = 1;
    int             reading = 1;
    MemType         memType = MT_SDRAM;
    int             result = 0;

    switch (argc)
    {
    case 2:
        if (strcmp (argv[1], "full") == 0)
        {
            memType = MT_FULL_TEST;
        }
        else
        {
            argsOK = 0;
        }
        break;

    case 3:
        if (strcmp (argv[1], "sdram") == 0)
        {
            memType = MT_SDRAM;
        }
        else if (strcmp (argv[1], "sram") == 0)
        {
            memType = MT_SRAM;
        }
        else if (strcmp (argv[1], "flash") == 0)
        {
            memType = MT_FLASH;
        }
        else
        {
            printf ("Bad memory type\n");
            argsOK = 0;
        }

        if (strcmp (argv[2], "r") == 0)
        {
            reading = 1;
        }
        else if (strcmp (argv[2], "w") == 0)
        {
            reading = 0;
        }
        else
        {
            printf ("Bad access type (r or w)\n");
            argsOK = 0;
        }
        break;

    default:
        printf ("Bad number of arguments\n");
        argsOK = 0;
        break;
    }


    if (argsOK)
    {
        switch (memType)
        {
            case MT_SDRAM:
#if defined(OLD_STYLE_TESTS)
                sdram_test (reading, testDurationMs);
#else
                sdram2test (reading, NUM_TEST_ITERATIONS);
#endif  /* defined(OLD_STYLE_TESTS) */
                break;
            case MT_SRAM:
#if defined(OLD_STYLE_TESTS)
                sram_test (reading, testDurationMs);
#else
                sram2test (reading, NUM_TEST_ITERATIONS);
#endif  /* defined(OLD_STYLE_TESTS) */
                break;
            case MT_FLASH:
#if defined(OLD_STYLE_TESTS)
                flash_test (reading, testDurationMs);
#else
                flash2test (reading, NUM_TEST_ITERATIONS);
#endif  /* defined(OLD_STYLE_TESTS) */
                break;
            case MT_FULL_TEST:
                {
                    int     iStatus = icache_status ();

                    if (! iStatus)
                    {
                        printf ("Enabling I-Cache\n");
                        icache_enable ();
                    }

#if defined(OLD_STYLE_TESTS)
                    sdram_test (1, testDurationMs);
                    sdram_test (0, testDurationMs);
                    sram_test (1, testDurationMs);
                    sram_test (0, testDurationMs);
                    flash_test (1, testDurationMs);
#else
                    sdram2test (1, NUM_TEST_ITERATIONS);
                    sdram2test (0, NUM_TEST_ITERATIONS);
                    sram2test (1, NUM_TEST_ITERATIONS);
                    sram2test (0, NUM_TEST_ITERATIONS);
                    flash2test (1, NUM_TEST_ITERATIONS);
#endif  /* defined(OLD_STYLE_TESTS) */

                    if (! iStatus)
                    {
                        printf ("Disabling I-Cache\n");
                        icache_disable ();
                    }
                }
                break;
            default:
                printf ("Bad memory type - coding error!\n");
                break;
        }
    }
    else
    {
        result = 1;
        printf ("Usage:\n%s%s\n", cmdtp->usage, cmdtp->help);
    }


    return result;
}

/*===========================================================================*/
static void reportHwConfig (void)
{
    unsigned long i;
    unsigned long c1;
    int     armMode;

    asm ("mrs %0,cpsr":"=r" (i));
    armMode = i & 0x1f;
    printf ("ARM CPSR=0x%08lx (IRQ=%s, FIQ=%s, Thumb=%s, Mode=%s)\n",
            i,
            (i & 0x0080) ? "DISABLED" : "ENABLED",
            (i & 0x0040) ? "DISABLED" : "ENABLED",
            (i & 0x0020) ? "ENABLED"  : "DISABLED",
            (armMode == 0x13) ? "Supervisor"
                              : (armMode == 0x10) ? "User"
                                                  : (armMode == 0x1f) ? "System"
                                                                      : (armMode == 0x11) ? "FIQ"
                                                                                          : (armMode == 0x12) ? "IRQ"
                                                                                                              : "??????"
           );

    /* read c1 */
    asm ("mrc p15, 0, %0, c1, c0, 0":"=r" (c1));
    printf ("ARM CP15: Control (1)=0x%08lx  (I-Cache=%s, D-Cache=%s, MMU=%s)\n",
            c1,
            (c1 & 0x1000) ? "ON" : "OFF",
            (c1 & 0x0004) ? "ON" : "OFF",
            (c1 & 0x0001) ? "ON" : "OFF"
           );
}

/*---------------------------------------------------------------------------*/
static void reportMemArb (void)
{
    int                 i;
    int                 slotOff;
    unsigned short      value;
    unsigned short      valid;
    volatile unsigned short *armReg_p = (volatile unsigned short *)PC20X_MEM_IF_BASE;
    volatile unsigned short *addr_p;

    /* SDRAM */
    addr_p = armReg_p + (MemifSdramArbValidGroupsConfigRegOffset/2);
    valid = IOREAD16(addr_p);

    for (i=0; i<16; i++)
    {
        if (valid & (1 << (i/2)))
        {
            slotOff = ((i & 1) << 1);

            addr_p = armReg_p + (MemifSdramArbGroup0SlotAConfigRegOffset/2) + i;
            value = IOREAD16(addr_p);

            printf ("SDRAM_arb_group_%d_Slot_%d = %s,%s,%s,%dacc\n",
                                        (i/2), slotOff,
                                        (value & 0x0001) ? "pico" : "AHB ",
                                        (value & 0x0002) ? "Opp" : "Det",
                                        (value & 0x0004) ? "W" : "R",
                                        (((value & 0x0018) >> 3) + 1)*8
                   );
            printf ("SDRAM_arb_group_%d_Slot_%d = %s,%s,%s,%dacc\n",
                                        (i/2), slotOff + 1,
                                        (value & 0x0100) ? "pico" : "AHB ",
                                        (value & 0x0200) ? "Opp" : "Det",
                                        (value & 0x0400) ? "W" : "R",
                                        (((value & 0x1800) >> 11) + 1)*8
                   );
        }
    }
    printf ("\n");


    /* SRAM */
    addr_p = armReg_p + (MemifSramArbValidSlotsConfigRegOffset/2);
    valid = IOREAD16(addr_p);

    for (i=0; i<3; i++)
    {
        slotOff = i << 1;

        addr_p = armReg_p + (MemifSramArbSlotAConfigRegOffset/2) + i;
        value = IOREAD16(addr_p);

        if (valid & (1 << (2*i)))
        {
            printf ("SRAM_arb_Slot_%d = %s,%s,%s,%dacc,%s\n",
                                        slotOff,
                                        (value & 0x0001) ? "pico" : "AHB ",
                                        (value & 0x0002) ? "Opp" : "Det",
                                        (value & 0x0004) ? "W" : "R",
                                        (1 << ((value & 0x0018) >> 3)),
                                        ((value & 0x0001) ? ((value & 0x0020) ? "Buff1" : "Buff0")
                                                        : ((value & 0x0002) ? ((value & 0x0020) ? "Buff1" : "Buff0")
                                                                            : "-"
                                                            )
                                        )
                   );
        }

        if (valid & (1 << (2*i + 1)))
        {
            printf ("SRAM_arb_Slot_%d = %s,%s,%s,%dacc,%s\n",
                                        slotOff + 1,
                                        (value & 0x0100) ? "pico" : "AHB ",
                                        (value & 0x0200) ? "Opp" : "Det",
                                        (value & 0x0400) ? "W" : "R",
                                        (1 << ((value & 0x1800) >> 3)),
                                        ((value & 0x0100) ? ((value & 0x2000) ? "Buff1" : "Buff0")
                                                        : ((value & 0x0200) ? ((value & 0x2000) ? "Buff1" : "Buff0")
                                                                            : "-"
                                                            )
                                        )
                   );
        }
    }
    printf ("\n");
}


/*---------------------------------------------------------------------------*/
static void reportSdramCfg (void)
{
    volatile unsigned short *armReg_p = (volatile unsigned short *)PC20X_MEM_IF_BASE;
    volatile unsigned short *addr_p;

    /* SDRAM */
    addr_p = armReg_p + (DLL0SlaveAdjustRegOffset/2);
    printf ("SDRAM DLL Slave Adjusts: 0x%04x  0x%04x  0x%04x  0x%04x\n",
            IOREAD16(addr_p),
            IOREAD16((addr_p + 3)),
            IOREAD16((addr_p + 6)),
            IOREAD16((addr_p + 9)));

    addr_p = armReg_p + (MemifSdramSetupRegOffset/2);
    printf ("SDRAM Setup: 0x%04x\n", IOREAD16(addr_p));

    addr_p = armReg_p + (MemifSdramRefreshRateRegOffset/2);
    printf ("SDRAM Refresh Rate: 0x%04x\n", IOREAD16(addr_p));

    addr_p = armReg_p + (MemifSdramMrsRegOffset/2);
    printf ("SDRAM Mode: 0x%04x\n", IOREAD16(addr_p));

    addr_p = armReg_p + (MemifSdramErsRegOffset/2);
    printf ("SDRAM EMRS: 0x%04x\n", IOREAD16(addr_p));

    printf ("\n");
}

/*---------------------------------------------------------------------------*/
/* run the memcfg command.
 */
int do_memcfg (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    int     result = 0;
    switch (argc)
    {
    case 1:
/* for one-off test, changed CAS strobe - gave 1% increase in SDRAM reads (no affact on writes) */
/* *((volatile u16 *)(PC20X_MEM_IF_BASE + MemifSdramMrsRegOffset)) = 0x443;*/
        reportHwConfig ();
        reportMemArb ();
        reportSdramCfg ();
        break;

    default:
        printf ("Usage:\n%s%s\n", cmdtp->usage, cmdtp->help);
        result = 1;
        break;
    }

    return result;
}


/*===========================================================================*/
#define SDRAM_CFG_DATA_SIZE     17
typedef unsigned short SDRAM_CFG_DATA [SDRAM_CFG_DATA_SIZE];

#define SRAM_CFG_DATA_SIZE      4
typedef unsigned short SRAM_CFG_DATA [SRAM_CFG_DATA_SIZE];


#define MAX_SDRAM_CFGS          5
SDRAM_CFG_DATA  sdramCfgData [MAX_SDRAM_CFGS] =
{   /* just ARM access */
    {0x0000, 0x0000, 0x0404, 0x0404, 0x0000, 0x0000, 0x0000, 0x0000,
     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
     0x0003
    },

    /* ARM + pico equal access */
    {0x0404, 0x0404, 0x0505, 0x0505, 0x0000, 0x0000, 0x0101, 0x0101,
     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
     0x000f
    },

    /* ARM (det) + pico (opp) access */
    {0x0404, 0x0404, 0x0707, 0x0707, 0x0000, 0x0000, 0x0303, 0x0303,
     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
     0x000f
    },

    /* ARM (opp) + pico (opp) access */
    {0x0606, 0x0606, 0x0707, 0x0707, 0x0202, 0x0202, 0x0303, 0x0303,
     0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
     0x000f
    },

    /* just ARM - but using all groups */
    {0x0000, 0x0000, 0x0404, 0x0404, 0x0000, 0x0000, 0x0404, 0x0404,
     0x0000, 0x0000, 0x0404, 0x0404, 0x0000, 0x0000, 0x0404, 0x0404,
     0x00ff
    },
};

#define MAX_SRAM_CFGS           3
SRAM_CFG_DATA   sramCfgData [MAX_SRAM_CFGS] =
{   /* just ARM access */
    {0x0004, 0x0000, 0x0000,
     0x0003
    },

    /* ARM + pico equal access */
    {0x0504, 0x0025, 0x2101,
     0x003f
    },

    /* ARM (det) + pico (opp) access */
    {0x0704, 0x0027, 0x2303,
     0x003f
    }
};


static void sdramConfig (int sdramCfg)
{
    int                 i;
    volatile unsigned short *addr_p = ((volatile unsigned short *)PC20X_MEM_IF_BASE) + 0x20;
    SDRAM_CFG_DATA      *cfgData_p  = &sdramCfgData [sdramCfg];

    for (i=0; i<SDRAM_CFG_DATA_SIZE; i++)
    {
        IOWRITE16((*cfgData_p) [i], addr_p);

        addr_p++;
    }

    /* make it so */
    addr_p = ((volatile unsigned short *)PC20X_MEM_IF_BASE) + 0x4f;
    IOWRITE16(0x0001, addr_p);

    while ((IOREAD16(addr_p) & 0x0001) == 0x0001)
    {
    }
}

/*-----------------------------------------------------------------------*/
static void sramConfig (int sramCfg)
{
    int                 i;
    volatile unsigned short *addr_p = ((volatile unsigned short *)PC20X_MEM_IF_BASE) + 0x40;
    SRAM_CFG_DATA       *cfgData_p  = &sramCfgData [sramCfg];

    for (i=0; i<SRAM_CFG_DATA_SIZE; i++)
    {
        IOWRITE16((*cfgData_p) [i], addr_p);

        addr_p++;
    }

    /* make it so */
    addr_p = ((volatile unsigned short *)PC20X_MEM_IF_BASE) + 0x4f;
    IOWRITE16(0x0002, addr_p);

    while ((IOREAD16(addr_p) & 0x0002) == 0x0002)
    {
    }
}

/*---------------------------------------------------------------------------*/
/* run the memarb command.
 */
int do_memarb (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    int     result = 0;
    int     i;
    int     len;
    int     argsOK = 1;
    int     sdramCfg = -1;
    int     sramCfg = -1;

    switch (argc)
    {
    case 1:
        reportHwConfig ();
        reportMemArb ();
        reportSdramCfg ();
        break;

    default:
        /*-----------------------------------------------------------------------*/
        /* input parameter checking ... */
        for (i=1; i< argc; i++)
        {
            if (strncmp (argv[i], "sdram", 5) == 0)
            {
                len = strlen (argv[i]);
                if (len != 6)
                {
                    argsOK = 0;
                }
                else
                {
                    char    ch = argv[i][5];
                    if ((ch < '0') || (ch > '9'))
                    {
                        argsOK = 0;
                    }
                    else
                    {
                        sdramCfg = ch - '0';
                        if (sdramCfg >= MAX_SDRAM_CFGS)
                        {
                            argsOK = 0;
                        }
                    }
                }
            }
            else if (strncmp (argv[i], "sram", 4) == 0)
            {
                len = strlen (argv[i]);
                if (len != 5)
                {
                    argsOK = 0;
                }
                else
                {
                    char    ch = argv[i][4];
                    if ((ch < '0') || (ch > '9'))
                    {
                        argsOK = 0;
                    }
                    else
                    {
                        sramCfg = ch - '0';
                        if (sramCfg >= MAX_SRAM_CFGS)
                        {
                            argsOK = 0;
                        }
                    }
                }
            }
            else
            {
                argsOK = 0;
            }
        }

        if (argsOK)
        {
            if (sdramCfg != -1)
            {
                sdramConfig (sdramCfg);
            }

            if (sramCfg != -1)
            {
                sramConfig (sramCfg);
            }
        }
        else
        {
            printf ("Usage:\n%s%s\n", cmdtp->usage, cmdtp->help);
            result = 1;
        }
        break;
    }

    return result;
}


/*---------------------------------------------------------------------------*/
/* run the set_led command.
 */
int do_set_led (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    int     result = 1;

    if (argc == 2)
    {
        if (strcmp (argv[1], "green") == 0)
        {
            u32  gpioData;

            /* set GPIO line to output for LED */
            gpioData = IOREAD32(LED2_DIR_REG);
            gpioData |= Gpio(LED2_GPIO_NUM);
            IOWRITE32(gpioData, LED2_DIR_REG);

            /* ensure LED is on/green */
            gpioData = IOREAD32(LED2_DATA_REG);
            gpioData |= Gpio(LED2_GPIO_NUM);
            IOWRITE32(gpioData, LED2_DATA_REG);

            result = 0;
        }
        else if (strcmp (argv[1], "red") == 0)
        {
            u32  gpioData;

            /* set GPIO line to output for LED */
            gpioData = IOREAD32(LED2_DIR_REG);
            gpioData |= Gpio(LED2_GPIO_NUM);
            IOWRITE32(gpioData, LED2_DIR_REG);

            /* ensure LED is off/red */
            gpioData = IOREAD32(LED2_DATA_REG);
            gpioData &= ~Gpio(LED2_GPIO_NUM);
            IOWRITE32(gpioData, LED2_DATA_REG);

            result = 0;
        }
    }

    if (result)
    {
        printf ("Usage:\n%s%s\n", cmdtp->usage, cmdtp->help);
    }

    return result;
}


#ifdef CONFIG_BOOTCOUNT_LIMIT

#define SRAM_BOOT_VARS_ADDRESS 0x1001ffc0

void bootcount_init ()
{
	volatile ulong *save_addr =
		(volatile ulong *)(SRAM_BOOT_VARS_ADDRESS);
	int i;

	if (save_addr[15] != BOOTCOUNT_MAGIC)
	{
		for(i=0; i<15; i++)
			save_addr[i] = 0;
		save_addr[15] = BOOTCOUNT_MAGIC;
	}
}

void bootcount_store (ulong a)
{
	volatile ulong *save_addr =
		(volatile ulong *)(SRAM_BOOT_VARS_ADDRESS);

	bootcount_init();  // Initialise boot variables if no BOOTCOUNT_MAGIC
	save_addr[14] = a;
}

ulong bootcount_load (void)
{
	volatile ulong *save_addr =
		(volatile ulong *)(SRAM_BOOT_VARS_ADDRESS);

	if (save_addr[15] != BOOTCOUNT_MAGIC)
		return 0;
	else
		return save_addr[14];
}

#endif /* CONFIG_BOOTCOUNT_LIMIT */


/*===========================================================================*/

U_BOOT_CMD(
    timcal,   2,   1,     do_timcal,
    "timcal  - calibrate timing (via LED and using scope)\n",
    "  args: <N> - number of milliseconds to drive LED high (beware arithmetic rounding issues!)"
);


U_BOOT_CMD(
    memcal,   3,   1,     do_memcal,
    "memcal  - calibrate memory read or write timing (assumes timing calibrated successfully - see timcal)\n",
    "  args: [sdram | sram | flash] [r | w] - type of memory + read or write"
);


U_BOOT_CMD(
    memcfg,   1,   1,     do_memcfg,
    "memcfg  - report ARM/PC202 configuration affecting memory access speed\n",
    ""
);


U_BOOT_CMD(
    memarb,   3,   1,     do_memarb,
    "memarb  - report/change SDRAM/SRAM arbitration scheme\n",
    "  args: [sdram<n>] [sram<m>] where:-\n"
    "    sdram<n> configures SDRAM config number n\n"
    "          0 => just ARM access\n"
    "          1 => ARM + pico equal access\n"
    "          2 => ARM (deterministic) + pico (opportunistic) access\n"
    "          3 => ARM (opportunistic) + pico (opportunistic) access\n"
    "          4 => just ARM access - all 8 groups\n"
    "    sram<m> configures SRAM config number m\n"
    "          0 => just ARM access\n"
    "          1 => ARM + pico equal access\n"
    "          2 => ARM (deterministic) + pico (opportunistic) access\n"
);

U_BOOT_CMD(
    set_led,   2,   1,     do_set_led,
    "set_led - change colour of Sys/Pwr LED\n",
    "  args: [red | green]"
);

#endif	/* CFG_CMD_CACHE */
