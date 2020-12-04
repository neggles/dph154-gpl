/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2008 picoChip Designs Ltd.
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
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <mach/pc302/vic.h>
#include <mach/pc302/axi2cfg.h>

#include "core.h"

/* Converts a physical address number into a pointer to the virtual location */
#define _ioa(n)     __io(IO_ADDRESS(n))

/** The system config register pointer (32 bit reg). */
static void __iomem *syscfg_reg;

/*!
 * Initialise the AXI bus error handling
 */
static void
pc302_axi_bus_error_init (void);

/* INTERRUPTS:
 */
static void pc302_mask_irq(u32 irq)
{
    u32 base = irq >= 32 ? PC302_VIC0_BASE : PC302_VIC1_BASE;
    u32 mask;

    if (irq >= 32)
        mask = (1 << (irq - 32));
    else
        mask = (1 << irq);

    iowrite32(mask, _ioa(base + VIC_INT_ENABLE_CLEAR_REG_OFFSET));
}

static void pc302_unmask_irq(u32 irq)
{
    u32 mask;
    u32 base = irq >= 32 ? PC302_VIC0_BASE : PC302_VIC1_BASE;

    mask = ioread32(_ioa(base + VIC_INT_ENABLE_REG_OFFSET));

    if (irq >= 32)
        mask |= (1 << (irq - 32));
    else
        mask |= (1 << irq);

    iowrite32(mask, _ioa(base + VIC_INT_ENABLE_REG_OFFSET));
}

static struct irq_chip pc302_vic0_chip = {
    .ack    = pc302_mask_irq, /* Level triggering -> mask is ack */
    .mask   = pc302_mask_irq,
    .unmask = pc302_unmask_irq,
};

static struct irq_chip pc302_vic1_chip = {
    .ack    = pc302_mask_irq, /* Level triggering -> mask is ack */
    .mask   = pc302_mask_irq,
    .unmask = pc302_unmask_irq,
};

void __init pc302_init_irq(void)
{
    unsigned int irq;

    /* Disable all interrupts initially. */
    iowrite32(0, _ioa(PC302_VIC0_BASE + VIC_INT_ENABLE_REG_OFFSET));
    iowrite32(0, _ioa(PC302_VIC1_BASE + VIC_INT_ENABLE_REG_OFFSET));

    /* Make sure that all interrupts are normal IRQs and not FIQs. */
    iowrite32(0, _ioa(PC302_VIC0_BASE + VIC_INT_ENABLE_REG_OFFSET));
    iowrite32(0, _ioa(PC302_VIC1_BASE + VIC_INT_ENABLE_REG_OFFSET));

    /*
     * Make sure we clear all existing interrupts...
     * As interrupts are asserted by peripherals, individual driver code
     * should ensure the interrupts are initially cleared.
     */

    /*
     * Set up the interrupt helper functions for the interrupts
     */
    for (irq = 0; irq < 32; irq++) {
        /* Skip interrupts that are unused */
        if (VIC1_IRQ_USED_MASK & (1 << irq)) {
            set_irq_chip(irq, &pc302_vic1_chip);
            set_irq_handler(irq, handle_level_irq);
            set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
        }
    }

    for (irq = 32; irq < 64; irq++) {
        /* Skip interrupts that are unused */
        if (VIC0_IRQ_USED_MASK & (u64)((u64)1LLU << irq)) {
            set_irq_chip(irq, &pc302_vic0_chip);
            set_irq_handler(irq, handle_level_irq);
            set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
        }
    }
}

/* IO MAPPING: we have most of the peripherals at 0x80000000, but the AXI2Pico
 * buffers live at 0xC0000000. */
static struct map_desc pc302_io_desc[] __initdata = {
    {
        .virtual    = IO_ADDRESS(AXI2PICO_BUFFERS_BASE),
        .pfn        = __phys_to_pfn(AXI2PICO_BUFFERS_BASE),
        .length     = AXI2PICO_BUFFERS_SIZE,
        .type       = MT_DEVICE,
    },
    {
        .virtual    =  IO_ADDRESS(PC302_PERIPH_BASE),
        .pfn        = __phys_to_pfn(PC302_PERIPH_BASE),
        .length     = PC302_PERIPH_LENGTH,
        .type       = MT_DEVICE,
    },
};

void __init pc302_map_io(void)
{
    iotable_init(pc302_io_desc, ARRAY_SIZE(pc302_io_desc));
}

/* UARTS */
static struct plat_serial8250_port serial_platform_data[] = {
    {
        .membase    = (char*)IO_ADDRESS(PC302_UART1_BASE),
        .mapbase    = (unsigned long)PC302_UART1_BASE,
        .irq        = IRQ_UART1,
        .flags      = UPF_BOOT_AUTOCONF,
        .iotype     = UPIO_MEM32,
        .regshift   = 2,
        .uartclk    = PC302_BASE_BAUD,
    },
    {
        .membase    = (char*)IO_ADDRESS(PC302_UART2_BASE),
        .mapbase    = (unsigned long)PC302_UART2_BASE,
        .irq        = IRQ_UART2,
        .flags      = UPF_BOOT_AUTOCONF,
        .iotype     = UPIO_MEM32,
        .regshift   = 2,
        .uartclk    = PC302_BASE_BAUD,
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

/* Watchdog */
static void
pc302wdt_platform_release( struct device *dev )
{
    /* This function is intentionally left blank. */
}

static struct resource pc302wdt_resources[] = {
    {
        .start = PC302_WDOG_BASE,
        .end   = PC302_WDOG_BASE + 0xffff,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = IRQ_WDG,
        .end   = IRQ_WDG,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device pc302wdt_device = {
    .name = "pc302wdt",
    .id = 0,
    .dev = {
        .coherent_dma_mask = 0xffffffff,
        .release = pc302wdt_platform_release,
    },
    .num_resources = ARRAY_SIZE( pc302wdt_resources ),
    .resource = pc302wdt_resources,
};

/* Fuses */
static void
pc302fuse_platform_release( struct device *dev )
{
    /* This function is intentionally left blank. */
}

static struct resource pc302fuse_resources[] = {
    {
        .start = PC302_FUSE_BASE,
        .end   = PC302_FUSE_BASE + 0xFFFF,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device pc302fuse_device = {
    .name = "pc302-fuse",
    .id = -1,
    .dev = {
        .release = pc302fuse_platform_release,
    },
    .resource = pc302fuse_resources,
    .num_resources = ARRAY_SIZE( pc302fuse_resources ),
};

/* EMAC */
static void
pc302emac_platform_release(struct device *dev)
{
    /* This function is intentionally left blank. */
}

static struct resource pc302emac_resources[] =
{
    {
        .start  = PC302_EMAC_BASE,
        .end    = PC302_EMAC_BASE + 0xFFFF,
        .flags  = IORESOURCE_MEM,
    },
    {
        .start  = IRQ_EMAC,
        .end    = IRQ_EMAC,
        .flags  = IORESOURCE_IRQ,
    },
};

static struct platform_device pc302emac_device =
{
    .name = "pc302-emac",
    .id = 0,
    .dev =
    {
        .coherent_dma_mask = 0xffffffff,
        .release = pc302emac_platform_release,
    },
    .num_resources = ARRAY_SIZE(pc302emac_resources),
    .resource = pc302emac_resources,
};

/* GPIO */
static void
pc302gpio_platform_release( struct device *dev )
{
    /* This function is intentionally left blank. */
}

static struct resource pc302gpio_resources[] = {
    {
        .start = PC302_GPIO_BASE,
        .end   = PC302_GPIO_BASE + 0xFFFF,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device pc302gpio_device = {
    .name = "pc302gpio",
    .id = -1,
    .dev = {
        .release = pc302gpio_platform_release,
    },
    .resource = pc302gpio_resources,
    .num_resources = ARRAY_SIZE( pc302gpio_resources ),
};

/* DMACs */
static void
pc302_dma_platform_release(struct device *dev)
{
    /* This function is intentionally left blank. */
}

static struct resource pc302dmac_resources0[] =
{
    {
        .start  = PC302_DMAC1_BASE,
        .end    = PC302_DMAC1_BASE + 0xFFFF,
        .flags  = IORESOURCE_MEM,
    },
    {
        .start  = IRQ_DMAC1,
        .end    = IRQ_DMAC1,
        .flags  = IORESOURCE_IRQ,
    },
};

static struct resource pc302dmac_resources1[] =
{
    {
        .start  = PC302_DMAC2_BASE,
        .end    = PC302_DMAC2_BASE + 0xFFFF,
        .flags  = IORESOURCE_MEM,
    },
    {
        .start  = IRQ_DMAC2,
        .end    = IRQ_DMAC2,
        .flags  = IORESOURCE_IRQ,
    },
};

static struct platform_device dmac_device0 =
{
    .name       = "pc302-dmac",
    .id         = 0,
    .dev        =
    {
        .coherent_dma_mask = 0xffffffff,
        .release = pc302_dma_platform_release,
    },
    .num_resources = ARRAY_SIZE(pc302dmac_resources0),
    .resource = pc302dmac_resources0,
};

static struct platform_device dmac_device1 =
{
    .name       = "pc302-dmac",
    .id         = 1,
    .dev        =
    {
        .coherent_dma_mask = 0xffffffff,
        .release = pc302_dma_platform_release,
    },
    .num_resources = ARRAY_SIZE(pc302dmac_resources1),
    .resource = pc302dmac_resources1,
};

/* picoArray driver */
struct resource pa0_resources[] = {
    {
        .start = AXI2PICO_BUFFERS_BASE,
        .end = AXI2PICO_BUFFERS_BASE + (AXI2PICO_BUFFERS_SIZE-1),
        .flags = IORESOURCE_MEM,
        .name = "ahb2pico_axi2pico",
    },
    {
        /* Take ownership of: Purge config port, DMAC config */
        .start = PC302_AXI2CFG_BASE + AXI2CFG_PURGE_CFG_PORT_REG_OFFSET,
        .end = PC302_AXI2CFG_BASE + AXI2CFG_DEVICE_ID_REG_OFFSET-1,
        .flags = IORESOURCE_MEM,
        .name = "procif",
    },
    {
        /* Take ownership of: Config write and read ports */
        .start = PC302_AXI2CFG_BASE +AXI2CFG_CONFIG_WRITE_REG_OFFSET,
        .end = PC302_AXI2CFG_BASE +AXI2CFG_DMAC1_CONFIG_REG_OFFSET-1,
        .flags = IORESOURCE_MEM,
        .name = "procif2",
    },
    {
        .start = IRQ_AXI2PICO8, /* Interrupt for GPRs */
        .end = IRQ_AXI2PICO8,
        .flags = IORESOURCE_IRQ,
        .name = "gpr_irq",
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

static struct resource ipsec_resources[] = {
    {
        .start  = PC302_IPSEC_BASE,
        .end    = PC302_IPSEC_BASE + 0xFFFF,
        .flags  = IORESOURCE_MEM,
        .name   = "ipsec_engine",
    },
    {
        .start  = IRQ_IPSEC,
        .end    = IRQ_IPSEC,
        .flags  = IORESOURCE_IRQ,
        .name   = "ipsec_engine",
    },
};

static struct platform_device ipsec_device = {
    .dev            = {
        .coherent_dma_mask = 0xFFFFFFFF,
    },
    .name           = "ipsec_engine",
    .id             = -1,
    .resource       = ipsec_resources,
    .num_resources  = ARRAY_SIZE( ipsec_resources ),
};

static struct resource l2_resources[] = {
    {
        .start  = PC302_CIPHER_BASE,
        .end    = PC302_CIPHER_BASE + 0xFFFF,
        .flags  = IORESOURCE_MEM,
        .name   = "l2_engine",
    },
    {
        .start  = IRQ_AES,
        .end    = IRQ_AES,
        .flags  = IORESOURCE_IRQ,
        .name   = "l2_engine",
    },
};

static struct platform_device l2_device = {
    .dev            = {
        .coherent_dma_mask = 0xFFFFFFFF,
    },
    .name           = "l2_engine",
    .id             = -1,
    .resource       = l2_resources,
    .num_resources  = ARRAY_SIZE( l2_resources ),
};

#if defined(CONFIG_SPI_PC302) || defined(CONFIG_SPI_PC302_MODULE)
/* SPI Master */
static struct resource spi_resources[] = {
    {
        .start  = PC302_SSI_BASE,
        .end    = PC302_SSI_BASE + 0xFFFF,
        .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device spi_device = {
    .dev            = {
        .coherent_dma_mask = 0xFFFFFFFF,
    },
    .name           = "pc302-spi",
    .id             = 0,
    .resource       = spi_resources,
    .num_resources  = ARRAY_SIZE( spi_resources ),
};
#endif

/* AXI2CFG System Configuration Register */
static void __init
map_syscfg(void)
{
    syscfg_reg = ioremap(PC302_AXI2CFG_BASE + AXI2CFG_SYS_CONFIG_REG_OFFSET,
                         sizeof(u32));
}

u32
syscfg_read(void)
{
    BUG_ON(!syscfg_reg);
    return ioread32(syscfg_reg);
}
EXPORT_SYMBOL(syscfg_read);

void
syscfg_update(u32 mask,
              u32 val)
{
    u32 tmp = syscfg_read();
    tmp &= ~mask;
    tmp |= (val & mask);
    iowrite32(tmp, syscfg_reg);
}
EXPORT_SYMBOL(syscfg_update);

void __init pc302_core_init(void)
{
    platform_device_register(&serial_device);
    platform_device_register(&pc302wdt_device);
    platform_device_register(&pc302fuse_device);
    platform_device_register(&pc302emac_device);
    platform_device_register(&pc302gpio_device);
    platform_device_register(&dmac_device0);
    platform_device_register(&dmac_device1);
    platform_device_register(&pa0);
    platform_device_register(&ipsec_device);
    platform_device_register(&l2_device);
#if defined(CONFIG_SPI_PC302) || defined(CONFIG_SPI_PC302_MODULE)
    platform_device_register(&spi_device);
#endif

    map_syscfg();
    pc302_axi_bus_error_init();
}

/* TIMERS */

/* clock_tick_rate is the tick rate set at runtime. RT_LATCH is the derived
 * timer reload latch value. */
static unsigned long clock_tick_rate = PC302_TIMER_FREQ;
#define RT_LATCH() (((clock_tick_rate + HZ/2) / HZ) - 1)

/* Use timer 0 for the linux tick */
#define TICK_TIMER 0
#define TICK_TIMER_IRQ IRQ_TIMER0

static void
timer_set_mode(enum clock_event_mode mode,
        struct clock_event_device *clk)
{
    switch (mode) {
        case CLOCK_EVT_MODE_PERIODIC:
            iowrite32(RT_LATCH(), _ioa(PC302_TIMER_BASE +
                        TIMERNLOADCOUNTREGOFFSET(0)));
            iowrite32(TIMERENABLE | TIMERMODE,
                    _ioa(PC302_TIMER_BASE + TIMERNCONTROLREGOFFSET(0)));
            break;
        case CLOCK_EVT_MODE_UNUSED:
        case CLOCK_EVT_MODE_SHUTDOWN:
        default:
            break;
            iowrite32(0,
                    _ioa(PC302_TIMER_BASE + TIMERNCONTROLREGOFFSET(0)));
    }
}

static int
timer_set_next_event(unsigned long evt,
		     struct clock_event_device *unused)
{
    /* Disable the timer. */
    iowrite32(0, _ioa(PC302_TIMER_BASE + TIMERNCONTROLREGOFFSET(0)));
    /* Write the new event. */
    iowrite32(evt, _ioa(PC302_TIMER_BASE + TIMERNLOADCOUNTREGOFFSET(0)));
    /* Re-enable the timer. */
    iowrite32(TIMERENABLE | TIMERMODE,
            _ioa(PC302_TIMER_BASE + TIMERNCONTROLREGOFFSET(0)));

    return 0;
}

static struct clock_event_device timer0_clockevent = {
    .name		= "timer0",
    .shift	    	= 20,
    .features   	= CLOCK_EVT_FEAT_PERIODIC,
    .set_mode   	= timer_set_mode,
    .set_next_event	= timer_set_next_event,
};

/* IRQ handler for the timer. */
static irqreturn_t
pc302_timer_interrupt(int irq,
                      void *dev_id)
{
    struct clock_event_device *evt = &timer0_clockevent;

    /* Clear the interrupt. */
    ioread32(_ioa(PC302_TIMER_BASE + TIMERNEOIREGOFFSET(TICK_TIMER)));

    evt->event_handler(evt);

    return IRQ_HANDLED;
}

static struct irqaction pc302_timer_irq = {
    .name       = "pc302 Timer Tick",
    .flags      = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
    .handler    = pc302_timer_interrupt,
};

static cycle_t
pc302_rtc_get_cycles(void)
{
    return ioread32(_ioa(PC302_RTCLK_BASE + RTCLK_CCV_REG_OFFSET));
}

static struct clocksource clocksource_pc302 = {
    .name       = "rtc",
    .rating     = 300,
    .read       = pc302_rtc_get_cycles,
    .mask       = CLOCKSOURCE_MASK(32),
    .shift      = 2,
    .flags      = CLOCK_SOURCE_IS_CONTINUOUS,
};

#define PC302_RTC_FREQ PC302_TIMER_FREQ

static void __init
pc302_clocksource_init(void)
{
    /* The RTC is always running. We don't need to do any initialization. */
    clocksource_pc302.mult =
        clocksource_hz2mult(PC302_RTC_FREQ, clocksource_pc302.shift);
    clocksource_register(&clocksource_pc302);
}

/* Set up the timer and interrupt.  */
static void __init
pc302_timer_init(void)
{
    iowrite32(0, _ioa(PC302_TIMER_BASE + TIMERNCONTROLREGOFFSET(0)));
    /* Make irqs happen for the system timer */
    setup_irq(TICK_TIMER_IRQ, &pc302_timer_irq);
    timer0_clockevent.mult =
        div_sc(clock_tick_rate, NSEC_PER_SEC, timer0_clockevent.shift);
    timer0_clockevent.max_delta_ns =
        clockevent_delta2ns(0xffffffff, &timer0_clockevent);
    timer0_clockevent.min_delta_ns =
        clockevent_delta2ns(0x1, &timer0_clockevent);
    timer0_clockevent.cpumask = cpumask_of_cpu(0);
    clockevents_register_device(&timer0_clockevent);
    pc302_clocksource_init();
}

struct sys_timer pc302_timer = {
    .init	= pc302_timer_init,
};

/* AXI Bus Read / Write Error Handling */

/* AXI Bus read errors */
static irqreturn_t pc302_axi_bus_read_error_interrupt(int irq, void *dev_id)
{
    /* If we ever get one of these interrupts
       then we are in big trouble, they should never happen.
       The error condition is non recoverable. */

    u32 axi_read_error =
    ioread32(_ioa(PC302_AXI2CFG_BASE + AXI2CFG_AXI_ERR_STATE_REG_OFFSET));

    axi_read_error &= AXI2CFG_AXI_RD_ERR_MASK;

    printk (KERN_ERR "AXI Bus Read Error 0x%08x has happened.\n", axi_read_error);

    /* There is no way back, therefore...*/
    BUG();

     /* Should never get here ! */
    return IRQ_HANDLED;
}

static struct irqaction pc302_axi_read_error_irq = {
    .name       = "pc302 Axi Bus Read Error",
    .flags      = IRQF_DISABLED,
    .handler    = pc302_axi_bus_read_error_interrupt,
};

/* AXI Bus write errors */
static irqreturn_t pc302_axi_bus_write_error_interrupt(int irq, void *dev_id)
{
    /* If we ever get one of these interrupts
       then we are in big trouble, they should never happen.
       The error condition is non recoverable. */

    u32 axi_write_error =
    ioread32(_ioa(PC302_AXI2CFG_BASE + AXI2CFG_AXI_ERR_STATE_REG_OFFSET));

    axi_write_error &= AXI2CFG_AXI_WR_ERR_MASK;

    printk (KERN_ERR "AXI Bus Write Error 0x%08x has happened.\n", axi_write_error);

    /* There is no way back, therefore...*/
    BUG();

    /* Should never get here ! */
    return IRQ_HANDLED;
}

static struct irqaction pc302_axi_write_error_irq = {
    .name       = "pc302 Axi Bus Write Error",
    .flags      = IRQF_DISABLED,
    .handler    = pc302_axi_bus_write_error_interrupt,
};

/* Initialise AXI Bus error handling */
static void
pc302_axi_bus_error_init (void)
{
    /* Setup the irq handler for AXI read errors */
    setup_irq(IRQ_AXI_RD_ERR, &pc302_axi_read_error_irq);

    /* Setup the irq handler for AXI write errors */
    setup_irq(IRQ_AXI_WR_ERR, &pc302_axi_write_error_irq);

    /* Make sure no AXI errors are masked */
    iowrite32(AXI2CFG_AXI_ERR_MASK_NONE,
            _ioa(PC302_AXI2CFG_BASE + AXI2CFG_AXI_ERR_MASK_REG_OFFSET));

    /* Enable interrupts for all AXI Read & Write errors */
    iowrite32(AXI2CFG_AXI_ERR_ENABLE_ALL,
            _ioa(PC302_AXI2CFG_BASE + AXI2CFG_AXI_ERR_ENABLE_REG_OFFSET));
}
