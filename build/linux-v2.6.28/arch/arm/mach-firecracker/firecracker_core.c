/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 *  linux/arch/arm/mach-firecracker/firecracker_core.c
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <mach/dma.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/div64.h>
#include <linux/cnt32_to_63.h>
#include <linux/mm.h>
#include <linux/sched.h>	/* just for sched_clock() - funny that */
#include <linux/bug.h>

#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
#include <mach/fpga_debug.h>
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

#include "core.h"


#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
IPA_FPGA_DEBUG_0 ipaFpgaDebug0Ptr = NULL;
IPA_FPGA_DEBUG_1 ipaFpgaDebug1Ptr = NULL;
IPA_FPGA_DEBUG_2 ipaFpgaDebug2Ptr = NULL;
IPA_FPGA_DEBUG_3 ipaFpgaDebug3Ptr = NULL;
IPA_FPGA_DEBUG_4 ipaFpgaDebug4Ptr = NULL;
IPA_HIRES_TIMER  ipaHiresTimerPtr = NULL;
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

/* Converts a physical address number into a pointer to the virtual location */
#define _ioa(n)     __io(IO_ADDRESS(n))


/* INTERRUPTS:
 */
static void firecracker_mask_irq(u32 irq)
{
    u32 mask = ioread32(_ioa(PC20X_VIC_BASE + VIC_ENABLE_REG_OFFSET));

    mask &= ~(1 << irq);

    iowrite32(mask, _ioa(PC20X_VIC_BASE + VIC_ENABLE_REG_OFFSET));
}

static void firecracker_unmask_irq(u32 irq)
{
    u32 mask = ioread32(_ioa(PC20X_VIC_BASE + VIC_ENABLE_REG_OFFSET));

    mask |= (1 << irq);

    iowrite32(mask, _ioa(PC20X_VIC_BASE + VIC_ENABLE_REG_OFFSET));
}

static struct irq_chip firecracker_vic_chip = {
    .ack    = firecracker_mask_irq, /* Level triggering -> mask is ack */
    .mask   = firecracker_mask_irq,
    .unmask = firecracker_unmask_irq,
    /* NOTE - We have the opportunity to use the retrigger method by setting
     * a bit in the force register. No other platforms do this and I am not
     * sure of its value so I will leave it.
     */
};


void __init firecracker_init_irq(void)
{
    unsigned int irq;

    /* Disable all interrupts initially. */
    iowrite32(0, _ioa(PC20X_VIC_BASE + VIC_ENABLE_REG_OFFSET));

    /* Unmask all, and use the enable bits to control */
    iowrite32(0, _ioa(PC20X_VIC_BASE + VIC_MASK_REG_OFFSET));

    /*
     * Make sure we clear all existing interrupts...
     * As interrupts are asserted by peripherals, individual driver code
     * should ensure the interrupts are initially cleared.
     */

    /*
     * Set up the interrupt helper functions for the interrupts
     */
    for (irq = 0; irq < NR_IRQS; irq++) {

        set_irq_chip(irq, &firecracker_vic_chip);

        /* Skip interrupts that are unused */
        if (VIC_USED_IRQ_MASK & (1 << irq)) {
            set_irq_handler(irq, handle_level_irq);
            set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
        }
    }
}


/* IO MAPPING:
 */

static struct map_desc firecracker_io_desc[] __initdata = {

    /* For simplicity, create one big mapping that covers all the peripherals 
     * at PA range 0xffe00000 - 0xffffffff to VA range 0xfee00000 - 0xfeffffff 
     */
    {
        .virtual    =  IO_ADDRESS(PC20X_PERIPH_BASE),
        .pfn        = __phys_to_pfn(PC20X_PERIPH_BASE),
        .length     = PC20X_PERIPH_LENGTH,
        .type       = MT_DEVICE
    }
};

void __init firecracker_map_io(void)
{
    iotable_init(firecracker_io_desc, ARRAY_SIZE(firecracker_io_desc));
}


/* EMAC/PHY:
 */
static void emac_platform_release(struct device *dev)
{
    /* This function is intentionally left blank. */
}

static struct resource emac_resources[] =
{
    {
        .start  = PC20X_EMAC_BASE,
        .end    = PC20X_EMAC_BASE + 0x1FFF,
        .flags  = IORESOURCE_MEM,
    },
    {
        .start  = IRQ_EMAC,
        .end    = IRQ_EMAC,
        .flags  = IORESOURCE_IRQ,
    },
};

static struct platform_device emac_device = 
{
    .name = "pc20x-emac",
    .id = 0,
    .dev =
    {
        .coherent_dma_mask = 0xffffffff,
        .release = emac_platform_release,
    },
    .num_resources = ARRAY_SIZE(emac_resources),
    .resource = emac_resources,
};

static struct platform_device mii_device = {
    .name       = "pc20x-mii",
    .id         = 0,
};


/* UARTS:
 */
static struct plat_serial8250_port serial_platform_data[] = {
    {
        .membase    = (char*)IO_ADDRESS(PC20X_UART1_BASE),
        .mapbase    = (unsigned long)PC20X_UART1_BASE,
        .irq        = IRQ_UART_0,
        .flags      = UPF_BOOT_AUTOCONF,
        .iotype     = UPIO_MEM32,
        .regshift   = 2,
        .uartclk    = FIRECRACKER_BASE_BAUD,
    },
    {
        .membase    = (char*)IO_ADDRESS(PC20X_UART2_BASE),
        .mapbase    = (unsigned long)PC20X_UART2_BASE,
        .irq        = IRQ_UART_1,
        .flags      = UPF_BOOT_AUTOCONF,
        .iotype     = UPIO_MEM32,
        .regshift   = 2,
        .uartclk    = FIRECRACKER_BASE_BAUD,
    },
    { },
};

static struct platform_device serial_device = {
    .name           = "serial8250",
    .id             = 0,
    .dev            = {
        .platform_data = serial_platform_data,
    },
};


/* DMA:
 */

static struct plat_firecracker_dma dmac_platform_data[] = {
    {
        .membase    = (char*)IO_ADDRESS(PC20X_DMAC1_BASE),
    },
    {
        .membase    = (char*)IO_ADDRESS(PC20X_DMAC2_BASE),
    },
    { },
};


static struct platform_device dmac_device0 = {
    .name           = "pc20x-dmac",
    .id             = 0,
    .dev            = {
		.coherent_dma_mask = 0xffffffff,
        .platform_data = &dmac_platform_data[0],
    },
};

static struct platform_device dmac_device1 = {
    .name           = "pc20x-dmac",
    .id             = 1,
    .dev            = {
		.coherent_dma_mask = 0xffffffff,
        .platform_data = &dmac_platform_data[1],
    },
};


#ifdef CONFIG_FIRECRACKER_DMA_TEST
static struct platform_device dmac_test_device = {
    .name           = "pc20x-dmac-test",
    .id             = 0,
    .dev            = {
		.coherent_dma_mask = 0xffffffff,
    },
};
#endif

struct resource pa0_resources[] = {
    {
        .start = PC20X_PROCIF_BASEP,
        .end = PC20X_PROCIF_BASEP + 0x7f,
        .flags = IORESOURCE_MEM,
        .name = "procif",
    },
    {
        .start = 0xfff80000,
        .end = 0xfff80000 + 0x3ffff,
        .flags = IORESOURCE_MEM,
        .name = "dma_base",
    },
    {
        .start = PC20X_AHB_2_PICO_BASE,
        .end = PC20X_AHB_2_PICO_BASE + 0x1ff,
        .flags = IORESOURCE_MEM,
        .name = "ahb2pico_axi2pico",
    },
    {
        .start = PC02X_CHIP_CONTROL,
        .end = PC02X_CHIP_CONTROL + 0x3,
        .flags = IORESOURCE_MEM,
        .name = "ccr_base",
    },
    {
        .start = IRQ_PROCIF,
        .end = IRQ_PROCIF,
        .flags = IORESOURCE_IRQ,
        .name = "procif_irq",
    },
    {
        .start = IRQ_PICOARRAY_GPR,
        .end = IRQ_PICOARRAY_GPR,
        .flags = IORESOURCE_IRQ,
        .name = "gpr_irq",
    },
    {
        .start = IRQ_DMA_1,
        .end = IRQ_DMA_1,
        .flags = IORESOURCE_IRQ,
        .name = "dma1_irq",
    },
    {
        .start = IRQ_DMA_2,
        .end = IRQ_DMA_2,
        .flags = IORESOURCE_IRQ,
        .name = "dma2_irq",
    },
};

static struct platform_device pa0 = {
    .name           = "picoArray",
    .id             = 0,
    .dev            = {
                .coherent_dma_mask = 0xffffffff,
    },
    .resource       = pa0_resources,
    .num_resources  = ARRAY_SIZE(pa0_resources),
};

/* GPIO */
static void
pc202gpio_platform_release( struct device *dev )
{
    /* This function is intentionally left blank. */
}

static struct resource pc202gpio_resources[] = {
    {
        .start = PC20X_GPIO_BASE,
        .end   = PC20X_GPIO_BASE + 0xFFFF,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device pc202gpio_device = {
    .name = "pc202gpio",
    .id = -1,
    .dev = {
        .release = pc202gpio_platform_release,
    },
    .resource = pc202gpio_resources,
    .num_resources = ARRAY_SIZE( pc202gpio_resources ),
};

/*
 * Added by MontaVista for platform support of the PicoArray Crypto for IPsec.
 */
#if defined(CONFIG_IPACCESS_CRYPTO) || defined(CONFIG_IPACCESS_CRYPTO_MODULE)
static struct platform_device picoarray_ipsec_device = {
    .name       = "pico-crypto-ipsec",
    .id         = 0,
};
#endif

void __init firecracker_init(void)
{
    platform_device_register(&serial_device);
    platform_device_register(&mii_device);
    platform_device_register(&emac_device);
    platform_device_register(&dmac_device0);
    platform_device_register(&dmac_device1);
    platform_device_register(&pc202gpio_device);
    platform_device_register(&pa0);
    
#ifdef CONFIG_FIRECRACKER_DMA_TEST
    platform_device_register(&dmac_test_device);
#endif

#if defined(CONFIG_IPACCESS_CRYPTO) || defined(CONFIG_IPACCESS_CRYPTO_MODULE)
    platform_device_register(&picoarray_ipsec_device);
#endif
}

/* TIMERS: */

/* clock_tick_rate is the tick rate set at runtime. RT_LATCH is the derived timer reload
 * latch value.
 * 
 * PC202-PC205_ARM_Subsystem_Appendix_F-K_v1_3.pdf Section I.4 documents that the arm 
 * timers generate an interrupt when the counter (having reached 0) is decremented 
 * again. This operation automatically reloads the register with the TimerNLoadCount 
 * value. This being the case, the TimerNLoadCount should be 1 less than RT_LATCH as 
 * defined in the original 
 *  <kernel_root> /arch/arm/mach-firecracker/firecracker_core.c.
 * that is : #define RT_LATCH() ((clock_tick_rate + HZ/2) / HZ)
 * This error has been corrected below (sh4 20070903)
 */
static const unsigned long clock_tick_rate = PC72052_I10_REVB_TIMER_FREQ;
#define RT_LATCH() ( ((clock_tick_rate + HZ/2) / HZ) - 1)

/* Use timer 0 for the linux tick */
#define TICK_TIMER 0
#define TICK_TIMER_IRQ IRQ_TIMER_0

/* Default values suit the 140MHz RevB clock. These are adjusted by the initialisation 
 * code below.
 */
/* the <<1 gets rid of the cnt_32_to_63 top bit saving on a bic insn
 * in the sched_clock function
 */
static const unsigned long sched_clock_rtc_multiplier = 50<<1;
static const unsigned long sched_clock_rtc_divisor = 7<<1;

/*
 * This is the Firecracker sched_clock implementation.  
 *
 * RevB uses a 140MHz timer which gives us
 * a resolution of 7.1ns, and a maximum value of about  ?? (Many years!)
 *
 * The return value is guaranteed to be monotonic in that range as long as
 * there is always less than 15 seconds for RevB between successive calls to
 * this function.
 */
unsigned long long sched_clock(void)
{
	unsigned long long v = 
        cnt32_to_63(ioread32(_ioa(PC20X_RTC_CLK_BASE + 
                        RTC_CURRECT_COUNTER_VALUE_REG_OFFSET)));

	/* the <<1 gets rid of the cnt_32_to_63 top bit saving on a bic insn */
	v *= sched_clock_rtc_multiplier;
	do_div(v, sched_clock_rtc_divisor);

	return v;
}

/*
 * The system tick timer is critical since it is used to determine the clock
 * frequency for NTP and for scheduling periodic RT tasks.  Unfortunately we
 * could miss ticks if drivers disable interrupts for extended periods (e.g.,
 * picoif and vprintk).  We'll use the free running RTC counter to detect when
 * ticks were missed and we'll recover by calling timer_tick() multiple times.
 * 
 * When the timer first starts we'll record the current RTC count in the static
 * variable startOfTick.  This is nominally the value of the RTC when the
 * previous tick was raised.  When the tick handler is called we can use this
 * to calculate how many tick intervals have elapsed and then increment the
 * value of startOfTick by RTC_TICK_INCREMENT for each tick.  Initially, the
 * delay in calling the handler will mean that startOfTick is actually a bit
 * later than it should be.  As more interrupts arrive some may be slightly
 * earlier than expected if they weren't delayed by as much.  We have to
 * detect that and adjust the startOfTick to reflect the better information
 * we now have about when the ticks are due.  As a sanity check we don't allow
 * corrections if they are too big (>10%), if the total drift since start up
 * gets too big (more than a tick) or if we've just processed multiple ticks.
 * The last check is in case we detect multiple ticks and process them but
 * still manage to get an interrupt for the last one.
 *
 * Operating separately from the startOfTick is refTimerTick.  This is used by
 * the firecracker_gettimeoffset function to tell the timekeeping code how long
 * it's been since the last tick.  This is incremented by RTC_TICK_INCREMENT for
 * every tick.  It's not allowed to drift with the delay jitter, so the RTC count
 * always provides an absolute time reference.  However, the reference is set so
 * that it's nominally a couple of ticks in the past.  This is so that the value
 * returned by firecracker_gettimeoffset is always positive, even if the working
 * value of startOfTick does drift earlier than the initial snapshot.  To ensure
 * this, we don't allow the startOfTick to drift closer than RTC_TICK_INCREMENT
 * to refTimerTick.
 *
 */
#undef DEBUG_TIMER_ADJUSTMENT

/* How many RTC counts there are per ms tick interrupt */
#define RTC_TICK_INCREMENT     ((PC72052_I10_REVB_TIMER_FREQ + HZ/2)/HZ)
    
/* How many counts we allow the tick to drift back in one go.
 * Experiments showed that the initial delays at startup were fairly small
 * and any adjustments needed were also small.  10% should be enough.
 */
#define MAX_TICK_ADJUSTMENT     (RTC_TICK_INCREMENT/10)

/* Reference clock for firecracker_gettimeofffset.  Used to give sub tick
 * resolution for the current time, based on the RTC counter.  The combination
 * of this and the jiffies count should be locked to the CPU clock cycles.
 */
static u32 refTimerTick  = 0;

/*
 * Returns number of microseconds since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset().
 *
 */
static unsigned long firecracker_gettimeoffset(void)
{
    /* Convert from RTC counts to microseconds, i.e., divide by number of counts
     * per microsecond.  Division is expensive and the divisor is a constant, so
     * we can multiply by its reciprocal expressed as a 32.32 fixed point number.
     * Since the RTC is runnning faster than 1MHz this will be less than 1.0 and
     * we can discard the top 32 bits of the multiplier since they're 0.  The
     * operation then reduces to 32*32 => 64 and keep only the top 32, which gcc
     * optimises nicely.
     */
    static const u32 divisor    = (PC72052_I10_REVB_TIMER_FREQ + 500000) / 1000000;
    static const u32 multiplier = ( ((1ULL << 32) + (divisor/2)) / divisor );

    u32 cur_ticks = ipaHiresTimer() - refTimerTick;

    return ((unsigned long long)cur_ticks * multiplier) >> 32;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t firecracker_timer_interrupt(int irq, void *dev_id)
{
    static int       firstCall        = 1;

    /* The RTC count corresponding to the last time round this function */
    static u32       startOfTick      = 0;
    
    /* We have two choices.  We can clear the interrupt before reading the
     * RTC.  In between, the RTC might have crossed another tick boundary
     * and we might have another waiting interrupt.  We would process both
     * ticks here, and on completion we'd get another, apparently spurious,
     * interrupt right away.
     *
     * Alternatively, we can read the RTC and then clear the interrupt.  In
     * that case the RTC might be read just before a second, overflow, interrupt
     * is raised.  Both interrupts would be cleared and we would only process
     * one of them.  It would be on the next interrupt that we detected that
     * two ticks had actually passed.
     *
     * Neither is likely.  The first option should reduce the delays in
     * handling ticks.  We have to ensure that such a case can't be used
     * to adjust the startOfTick on a spurious interrupt.  We can do that
     * by preventing adjustment for a certain number of ticks after we
     * have processed a multi-tick interrupt.
     */
    static int       noAdjust         = 2;
  
    u32              now;

    // ...clear the interrupt
    ioread32(_ioa(PC20X_TIMERS_BASE + TIMER_N_EOI_REG_OFFSET(TICK_TIMER)));

    now = ipaHiresTimer();

    if (firstCall)
    {
        /* First interrupt */
        firstCall = 0;

        startOfTick = now;
        
        /* A baseline for firecracker_gettimeoffset.  This is in the past to
         * ensure that the value returned by that function will always be +ve,
         * even after we allow some correction to the startOfTick for delays.
         */
        refTimerTick  = now - 2*RTC_TICK_INCREMENT;
        
        /* Process tick */
        timer_tick();
    }
    else
    {
        /* How long since the previous tick? */
        s32 interval = (s32)(now - startOfTick);
        
        if (interval < RTC_TICK_INCREMENT)
        {
            /* This tick is early.  How much earlier than expected? */
            s32 earlierBy = RTC_TICK_INCREMENT - interval;
            
            /* If we were to adjust the startOfTick, how far ahead of
             * refTickTimer would we be?  We always want to stay at least
             * one tick ahead of the reference time.  (If we ever drifted
             * anywhere close to that much then something would be
             * seriously wrong.) */
            s32 headRoom = (s32)(startOfTick - refTimerTick) - earlierBy;
            
            if (noAdjust)
            {
                /* Adjustment is currently disabled */
#if defined (DEBUG_TIMER_ADJUSTMENT) && defined(CONFIG_IPACCESS_FPGA_DEBUG)
                ipaFpgaDebug0("Ignore tick (too soon after multi)");
#endif
            }
            else if (earlierBy > MAX_TICK_ADJUSTMENT)
            {
                /* Adjustment is more than we allow in a single correction */
#if defined (DEBUG_TIMER_ADJUSTMENT) && defined(CONFIG_IPACCESS_FPGA_DEBUG)
                ipaFpgaDebug1("Ignore tick (big adjust = %d)", earlierBy);
#endif
            }
            else if (headRoom < RTC_TICK_INCREMENT)
            {
                /* Adjustment would mean we'd compensated by too much in total */
#if defined (DEBUG_TIMER_ADJUSTMENT) && defined(CONFIG_IPACCESS_FPGA_DEBUG)
                ipaFpgaDebug1("Ignore tick (drifted too far %d)", headRoom);
#endif
            }
            else
            {
                /* This tick had less delay than previous ones.  Use this as
                 * the new baseline for startOfTick and handle the tick. */
#if defined (DEBUG_TIMER_ADJUSTMENT) && defined(CONFIG_IPACCESS_FPGA_DEBUG)
                ipaFpgaDebug1("Adjust tick %d", earlierBy);
#endif
                startOfTick -= earlierBy;
                interval     = RTC_TICK_INCREMENT;
            }
        }
        
        if (interval >= 2*RTC_TICK_INCREMENT)
        {
            /* We must have missed a tick interrupt.  We're playing
             * catch up.  Prevent adjustments on the next few interrupts
             * to be safe.
             */
#if defined (DEBUG_TIMER_ADJUSTMENT) && defined(CONFIG_IPACCESS_FPGA_DEBUG)
            struct pt_regs* const regs = get_irq_regs();
            ipaFpgaDebug2("Multi tick %d after instruction %p", interval, profile_pc(regs));
#endif
            noAdjust = 2;
        }
        else if (noAdjust)
        {
            noAdjust--;
        }
        
        /* Handle all the ticks we've had since the last one */
        while (interval >= RTC_TICK_INCREMENT)
        {
            startOfTick  += RTC_TICK_INCREMENT;
            refTimerTick += RTC_TICK_INCREMENT;
            interval     -= RTC_TICK_INCREMENT;
            
            timer_tick();
        }
    }

    return IRQ_HANDLED;
}

static struct irqaction firecracker_timer_irq = {
    .name       = "pc20x Timer Tick",
    .flags      = IRQF_DISABLED | IRQF_TIMER,
    .handler    = firecracker_timer_interrupt,
};

/*
 * Set up the timer and interrupt.
 */
static void __init firecracker_timer_init(void)
{
    /* Start with the timer disabled */
    iowrite32(0, 
            _ioa(PC20X_TIMERS_BASE + TIMER_N_CONTROL_REG_OFFSET(TICK_TIMER)));

    /* Set the reload count that gives us the desired interrupt frequency */
    iowrite32(RT_LATCH(), 
            _ioa(PC20X_TIMERS_BASE + TIMER_N_LOAD_COUNT_REG_OFFSET(TICK_TIMER)));
    
    /* Set user defined count mode, unmask interrupt and enable the timer */
    iowrite32((TIMER_ENABLE | TIMER_MODE), 
            _ioa(PC20X_TIMERS_BASE + TIMER_N_CONTROL_REG_OFFSET(TICK_TIMER)));

    /* Make irqs happen for the system timer */
    setup_irq(TICK_TIMER_IRQ, &firecracker_timer_irq);
}

struct sys_timer firecracker_timer = {
    .init       = firecracker_timer_init,
    .offset     = firecracker_gettimeoffset,
};


/* Pointer to the memory info held here so that we have access from the 
 * command line parsing. NOTE: This is horrid.
 */
static struct meminfo *meminfo __initdata;

/* Reset the memory information to the specified firecracker fragmented
 * memory map.  The memory map is specified as a size and a start address.
 *
 * The start address should be one of:
 *
 * 0x00000000 - DDR Bank 0
 * 0x04000000 - DDR Bank 1
 * 0x08000000 - DDR Bank 2
 * 0x0c000000 - DDR Bank 3 
 */
static void __init set_firecracker_frag_mem(u32 size, u32 start)
{
    meminfo->bank[meminfo->nr_banks].start = PAGE_ALIGN(start);
    meminfo->bank[meminfo->nr_banks].size = size & PAGE_MASK;
    meminfo->bank[meminfo->nr_banks].node = PHYS_TO_NID(start);
    meminfo->nr_banks++;               
}

/*
 * Pick out the memory size.  We look for pc20x_mem=size@startAddress,
 * where size is "size[KkMm]".
 */
static void __init early_firecracker_mem(char **p)
{
    static int usermem __initdata = 0;
        
    u32 size, start;
    
    /* Default value */
    start = FIRECRACKER_RAM_START;
	
    /*
     * If the user specifies memory size, we
     * blow away any automatically generated
     * size.
     */
    if (usermem == 0) {
        usermem = 1;
	meminfo->nr_banks = 0;
    }
        
    size  = memparse(*p, p);

    if (**p == '@')
        start = memparse(*p + 1, p);
       
    set_firecracker_frag_mem(size, start);
}
__early_param("pc20x_mem=", early_firecracker_mem);


/* Fixup function that is implemented so that we have a chance to pass the
 * sparse physical ram memory mapping to the kernel at bootup.
 */
void __init firecracker_fixup(
        struct machine_desc *md,
        struct tag *tag,
        char ** from,
        struct meminfo *mi)
{

    u32 startAddress, i;
    
    /* Record the pointer to the meminfo for the command line parsing later */
    meminfo = mi;
    
    /* At kernel boot up we initialise all four DDR RAM Banks.
     * If the user is utilising a 2 Bank DDR design or a design that requires different 
     * DDR Bank sizes for the kernel, then the memory requirements will need to be
     * specified on the command line at kernel boot time.
     *
     * Check out the pc20x_mem command line option for more information.
     */
       
    startAddress = FIRECRACKER_RAM_START;

    for (i = 0; i < FIRECRACKER_RAM_BANKS; i++)
    {
        set_firecracker_frag_mem(SZ_16M, startAddress); 
        startAddress += FIRECRACKER_RAM_BANK_STRIDE;
    }
                           
}

#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
void ipaFpgaDebug0(const char* text)
{
    if (ipaFpgaDebug0Ptr)
    {
        ipaFpgaDebug0Ptr(text);
    }
}

EXPORT_SYMBOL(ipaFpgaDebug0Ptr);
EXPORT_SYMBOL(ipaFpgaDebug0);

void ipaFpgaDebug1(const char* text, int num1)
{
    if (ipaFpgaDebug1Ptr)
    {
        ipaFpgaDebug1Ptr(text, num1);
    }
}

EXPORT_SYMBOL(ipaFpgaDebug1Ptr);
EXPORT_SYMBOL(ipaFpgaDebug1);

void ipaFpgaDebug2(const char* text, int num1, int num2)
{
    if (ipaFpgaDebug2Ptr)
    {
        ipaFpgaDebug2Ptr(text, num1, num2);
    }
}

EXPORT_SYMBOL(ipaFpgaDebug2Ptr);
EXPORT_SYMBOL(ipaFpgaDebug2);

void ipaFpgaDebug3(const char* text, int num1, int num2, int num3)
{
    if (ipaFpgaDebug3Ptr)
    {
        ipaFpgaDebug3Ptr(text, num1, num2, num3);
    }
}

EXPORT_SYMBOL(ipaFpgaDebug3Ptr);
EXPORT_SYMBOL(ipaFpgaDebug3);

void ipaFpgaDebug4(const char* text, int num1, int num2, int num3, int num4)
{
    if (ipaFpgaDebug4Ptr)
    {
        ipaFpgaDebug4Ptr(text, num1, num2, num3, num4);
    }
}

EXPORT_SYMBOL(ipaFpgaDebug4Ptr);
EXPORT_SYMBOL(ipaFpgaDebug4);

unsigned long ipaHiresTimer(void)
{
    /* We were using cpetd's hires timer here, but the RTC is always running so
     * we'll use that instead.
     */
    return ioread32(_ioa(PC20X_RTC_CLK_BASE + RTC_CURRECT_COUNTER_VALUE_REG_OFFSET));
}

EXPORT_SYMBOL(ipaHiresTimerPtr);
EXPORT_SYMBOL(ipaHiresTimer);
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */
