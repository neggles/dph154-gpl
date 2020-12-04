/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */
#include <linux/module.h>
#include <linux/perfcounters.h>
#include <mach/irqs.h>

/*
 * Spinlock for the ARM counters. We need a lock as the counters share a
 * configuration register and some bits inside it. In particular, there is a
 * global enable bit for all 3 counters - disabling one counter will also
 * disable the others.
 */
static DEFINE_SPINLOCK( arm_counter_lock );

/* Mask for the IRQ bits in the control register. */
#define ARM_COUNTERS_IRQS       ( 7 << 8 )

/* The cycle count IRQ status bit in the control register. */
#define CYCLE_COUNT_IRQ         ( 1 << 10 )

/* Handle a cycle count overflow. */
static irqreturn_t
arm_ccount_overflow( struct perfcounter *counter )
{
    u32 control;
    u32 irqs;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );

    /* We need to read the interrupts that have been read and then write back
     * the control register to clear the interrupt. Make sure that we don't
     * clear the interrupts for the other counters at the same time though. */
    irqs = control & ARM_COUNTERS_IRQS;
    control &= ~ARM_COUNTERS_IRQS;
    if ( irqs & CYCLE_COUNT_IRQ )
    {
        control |= CYCLE_COUNT_IRQ;
        asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );
        /* We have overflowed. Add a full 32-bit integer + 1 to the base
         * count. */
        counter->base_count += 0x100000000LLU;
        spin_unlock_irqrestore( &arm_counter_lock, flags );
        return IRQ_HANDLED;
    }
    else
    {
        spin_unlock_irqrestore( &arm_counter_lock, flags );
        return IRQ_NONE;
    }
}

/* Read the value of the cycle count performance monitor register. */
static int
arm_ccount_read( struct perfcounter *counter,
                 u64 *val )
{
    u32 ccount_low;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    asm( "mrc p15, 0, %0, c15, c12, 1" : "=r"( ccount_low ) );
    *val = ccount_low + counter->base_count;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/*
 * Global counter enable bit. Setting this will enable all 3 performance
 * counters.
 */
#define COUNTERS_ENABLE         ( 1 << 0 )

/*
 * Cycle count divider bit. If set, the counter will increment every cycle. If
 * unset it will increment every 64 cycles.
 */
#define CYCLE_COUNT_DIVIDER     ( 1 << 3 )

/* IRQ enable bit for the cycle count overflow. */
#define CYCLE_COUNT_IRQEN       ( 1 << 6 )

/* Enable the cycle count performance monitor. */
static int
arm_ccount_enable( struct perfcounter *counter )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    /* Configure the cycle count to tick every cycle. */
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control |= COUNTERS_ENABLE | CYCLE_COUNT_IRQEN;
    control &= ~CYCLE_COUNT_DIVIDER;
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->enabled = 1;

    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* Disable the cycle count performance monitor. */
static int
arm_ccount_disable( struct perfcounter *counter )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    /* Configure the cycle count to tick every cycle. */
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control &= ~( COUNTERS_ENABLE | CYCLE_COUNT_IRQEN );
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->enabled = 0;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/*
 * Global reset bit for the ARM performance counters. Setting this bit will
 * reset all 3 counters.
 */
#define CYCLE_COUNT_RESET   ( 1 << 2 )

/* Reset the cycle count register. */
static int
arm_ccount_reset( struct perfcounter *counter )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control |= CYCLE_COUNT_RESET;
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->base_count = 0;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* The counter operations for the cycle counter. */
struct perfcounter_ops ccount_ops = {
    .read           = arm_ccount_read,
    .enable         = arm_ccount_enable,
    .disable        = arm_ccount_disable,
    .overflow       = arm_ccount_overflow,
    .reset          = arm_ccount_reset,
};

/* The cycle count performance monitor. */
static struct perfcounter ccount = {
    .name           = "ccount",
    .ops            = &ccount_ops,
    .irq            = IRQ_NPMUIRQ,
    .irq_flags      = IRQF_DISABLED | IRQF_SHARED,
    .owner          = THIS_MODULE,
};

/* The count0 performance counter IRQ status bit. */
#define COUNT0_IRQ         ( 1 << 8 )

/* The overflow ISR for the count0 performance counter. */
static irqreturn_t
arm_count0_overflow( struct perfcounter *counter )
{
    u32 control;
    u32 irqs;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );

    /* We need to read the interrupts that have been read and then write back
     * the control register to clear the interrupt. Make sure that we don't
     * clear the interrupts for the other counters at the same time though. */
    irqs = control & ARM_COUNTERS_IRQS;
    control &= ~ARM_COUNTERS_IRQS;
    if ( irqs & COUNT0_IRQ )
    {
        control |= COUNT0_IRQ;
        asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );
        /* We have overflowed. Add a full 32-bit integer + 1 to the base
         * count. */
        counter->base_count += 0x100000000LLU;
        spin_unlock_irqrestore( &arm_counter_lock, flags );
        return IRQ_HANDLED;
    }
    else
    {
        spin_unlock_irqrestore( &arm_counter_lock, flags );
        return IRQ_NONE;
    }
}

/* Read the count0 performance counter. */
static int
arm_count0_read( struct perfcounter *counter,
                 u64 *val )
{
    u32 count0_low;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    asm( "mrc p15, 0, %0, c15, c12, 2" : "=r"( count0_low ) );
    *val = count0_low + counter->base_count;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* The count0 IRQ enable bit. */
#define COUNT0_IRQEN       ( 1 << 4 )

/* Enable the count0 performance counter. */
static int
arm_count0_enable( struct perfcounter *counter )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    /* Configure the cycle count to tick every cycle. */
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control |= COUNTERS_ENABLE | COUNT0_IRQEN;
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->enabled = 1;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* Enable the count0 performance monitor. */
static int
arm_count0_disable( struct perfcounter *counter )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    /* Configure the cycle count to tick every cycle. */
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control &= ~( COUNTERS_ENABLE | COUNT0_IRQEN );
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->enabled = 0;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/*
 * The offset of the event selection bits for the count0 performance monitor.
 */
#define EVENT_COUNT_0_OFFSET    ( 20 )

/* Mask for the event selection bits for the count0 performance monitor. */
#define EVENT_COUNT_0_MASK      ( 0xFF << EVENT_COUNT_0_OFFSET )

/* Set the event the count0 performance monitor will count. */
static int
arm_count0_set_event( struct perfcounter *counter,
                      int event )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    event &= 0xFF;
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control &= ~EVENT_COUNT_0_MASK;
    control |= ( event << EVENT_COUNT_0_OFFSET );
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->event = event;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* The offset of the count0/count1 reset bit. */
#define COUNT_N_RESET           ( 1 << 1 )

/* Reset ARM counters 0 and 1. */
static int
arm_countn_reset( struct perfcounter *counter )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control |= COUNT_N_RESET;
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->base_count = 0;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* The counter operations for count0. */
struct perfcounter_ops count0_ops = {
    .read           = arm_count0_read,
    .enable         = arm_count0_enable,
    .disable        = arm_count0_disable,
    .overflow       = arm_count0_overflow,
    .set_event      = arm_count0_set_event,
    .reset          = arm_countn_reset,
};

/* ARM counter count0. */
static struct perfcounter count0 = {
    .name           = "count0",
    .ops            = &count0_ops,
    .irq            = IRQ_NPMUIRQ,
    .irq_flags      = IRQF_DISABLED | IRQF_SHARED,
    .owner          = THIS_MODULE,
};

/* Counter count1 interrupt status bit. */
#define COUNT1_IRQ         ( 1 << 9 )

/* Overflow interrupt for the count1 counter. */
static irqreturn_t
arm_count1_overflow( struct perfcounter *counter )
{
    u32 control;
    u32 irqs;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );

    /* We need to read the interrupts that have been read and then write back
     * the control register to clear the interrupt. Make sure that we don't
     * clear the interrupts for the other counters at the same time though. */
    irqs = control & ARM_COUNTERS_IRQS;
    control &= ~ARM_COUNTERS_IRQS;
    if ( irqs & COUNT1_IRQ )
    {
        control |= COUNT1_IRQ;
        asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );
        /* We have overflowed. Add a full 32-bit integer + 1 to the base
         * count. */
        counter->base_count += 0x100000000LLU;
        spin_unlock_irqrestore( &arm_counter_lock, flags );
        return IRQ_HANDLED;
    }
    else
    {
        spin_unlock_irqrestore( &arm_counter_lock, flags );
        return IRQ_NONE;
    }
}

/* Read the value of the count1 counter. */
static int
arm_count1_read( struct perfcounter *counter,
                 u64 *val )
{
    u32 count1_low;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    asm( "mrc p15, 0, %0, c15, c12, 3" : "=r"( count1_low ) );
    *val = count1_low + counter->base_count;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* Interrupt enable bit for count1 overflow. */
#define COUNT1_IRQEN       ( 1 << 5 )

/* Enable the count1 counter. */
static int
arm_count1_enable( struct perfcounter *counter )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    /* Configure the cycle count to tick every cycle. */
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control |= COUNTERS_ENABLE | COUNT1_IRQEN;
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->enabled = 1;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* Disable the count1 counter. */
static int
arm_count1_disable( struct perfcounter *counter )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    /* Configure the cycle count to tick every cycle. */
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control &= ~( COUNTERS_ENABLE | COUNT1_IRQEN );
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->enabled = 0;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* Offset for the count1 event selection bits in the control register. */
#define EVENT_COUNT_1_OFFSET    ( 12 )

/* Mask for the count1 event selection bits in the control register. */
#define EVENT_COUNT_1_MASK      ( 0xFF << EVENT_COUNT_1_OFFSET )

/* Set the event that count1 counts. */
static int
arm_count1_set_event( struct perfcounter *counter,
                      int event )
{
    u32 control;
    unsigned long flags;

    spin_lock_irqsave( &arm_counter_lock, flags );
    event &= 0xFF;
    asm( "mrc p15, 0, %0, c15, c12, 0" : "=r"( control ) );
    control &= ~EVENT_COUNT_1_MASK;
    control |= ( event << EVENT_COUNT_1_OFFSET );
    asm( "mcr p15, 0, %0, c15, c12, 0" : : "r"( control ) );

    counter->event = event;
    spin_unlock_irqrestore( &arm_counter_lock, flags );

    return 0;
}

/* Counter count1 operations. */
struct perfcounter_ops count1_ops = {
    .read           = arm_count1_read,
    .enable         = arm_count1_enable,
    .disable        = arm_count1_disable,
    .overflow       = arm_count1_overflow,
    .set_event      = arm_count1_set_event,
    .reset          = arm_countn_reset,
};

/* ARM count1 counter. */
static struct perfcounter count1 = {
    .name           = "count1",
    .ops            = &count1_ops,
    .irq            = IRQ_NPMUIRQ,
    .irq_flags      = IRQF_DISABLED | IRQF_SHARED,
    .owner          = THIS_MODULE,
};

static int
arm_perfcounters_init( void )
{
    int ret = perfcounter_register( &ccount );
    if ( ret )
        goto out;

    ret = perfcounter_register( &count0 );
    if ( ret )
        goto count0_fail;

    ret = perfcounter_register( &count1 );
    if ( ret )
        goto count1_fail;

    goto out;

count1_fail:
    perfcounter_unregister( &count0 );
count0_fail:
    perfcounter_unregister( &ccount );
out:
    return ret;
}

static void
arm_perfcounters_exit( void )
{
    perfcounter_unregister( &count0 );
    perfcounter_unregister( &count1 );
    perfcounter_unregister( &ccount );
}

module_init( arm_perfcounters_init );
module_exit( arm_perfcounters_exit );

MODULE_AUTHOR( "Jamie Iles" );
MODULE_LICENSE( "GPL" );
MODULE_DESCRIPTION( "ARMv6 Performance Monitor Access" );
