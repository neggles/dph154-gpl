/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 *  linux/arch/arm/mach-firecracker/dma.c
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * Description:
 *
 * This module provides the interface to the firecracker low level DMA
 * functionality.
 *
 * References:
 *
 * Synopsys DesignWare DW_ahb_dmac Databook Version 2.07a December 14, 2005.
 * 
 * See linux/arch/arm/mach-firecracker/dma.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*****************************************************************************

                        Firecracker DMA API Specification
                             Ben Tucker 28sept2006
                                version 0.05

1. Introduction
===============

This is an informal description of the software interfaces exposed by the 
Firecracker DMA driver.
This API allows kernel modules to access the DMA capabilities of the
Firecracker. No user space interface to the DMA driver is provided.
This API is loosely based on the PC102 8560 DMA driver interfaces with the 
addition of the standard Linux ARM framework support.


1.1. Glossary
-------------

Transfer - A DMA transfer is the entire operation of moving data from
            one place to another, from start to finish. It is made up
            of 1 or more blocks, i.e. single vs multi-block.

Block    - The transfer of a number of data items. The size of the block
            can be controlled either by the sending or receiving end or
            by the DMA controller itself. Blocks can be scatter/gather.
            A DMA transfer can be multi-block if either a multi-block
            transfer list is used or if auto-reloading of one end of the
            transfer is employed.

Transaction - For efficiency, a number of data items can be moved at one
            time. This is also called Burst Transaction. A Block (or
            scatter/gather region) can be a whole number of transactions
            in size in which case it fits neatly into the number of
            burst transactions. Otherwise, smaller transactions are
            needed at the end of the block. These single transactions
            transfer a single data item at one time.

Transfer Width - The number of bytes in a data item.

Handshaking - In order to progress the movement of data, handshaking
            signals to the DMA hardware tell it to start a new 
            transaction. The handshaking signal can be generated in
            hardware (by a peripheral and hardware siganl), or by 
            software (by setting a bit in a register).

The 8560 DMA controller uses the tern 'Striding' where we refer to 
scatter/gather in this document. The 8560 term for 'Scatter/Gather' is similar
to the linked list multi-block transfers in this document. These differences
are inherited from the Synopsys documentation.


1.2. References
---------------

The information in this document needs to be read along side the 
DesignWare DW_ahb_dmac Databook Version 2.07a December 14, 2005 Synopsys 


2. High Level interfaces
========================

The driver software is layered as follows:
                ---------------------------     -----------------
                | ARM Linux DMA Framework |     | Pico.c driver |
                -------------------------------------------------
                 | Firecracker DMA API (loosely based on 8560) |
                 -----------------------------------------------
                           | Firecracker Hardware |
                           ------------------------


The Firecracker DMA API provides access to the capabilities of the Firecracker.
Thus mirrors loosely the 8560 API allowing software to be ported
to the new DMA hardware.

The ARM Linux DMA Framework is implemented using the Firecracker DMA API and 
provides the standard Linux DMA capabilities, allowing existing Linux drivers
to use the Firecracker DMA capabilities.
*****************************************************************************/

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/dma-mapping.h>

#include <mach/io.h>
#include <mach/hardware.h>
#include <mach/dma.h>

/* ENABLE_TR_WIDTH allows the setting of the transfer width to values other 
 * than 32 bits.
 */
#undef ENABLE_TR_WIDTH

#if CONFIG_FIRECRACKER_DMA_DEBUG > 0

/* Define ENABLE_DEBUGGING to include code for debugging via printk. 
 * The firecracker_dma_dump_regs will always be included.
 * The value of ENABLE_DEBUGGING specifies the default debug level.
 */
#define ENABLE_DEBUGGING CONFIG_FIRECRACKER_DMA_DEBUG

#else

#undef ENABLE_DEBUGGING

#endif

/* Internal state associated with a DMA transfer */
struct firecracker_dma_xfr_tag {
    void *cookie;                       /* Client context */
    firecracker_dma_list_t list;        /* Multi-block list 
                                           or NULL if direct xfr */
    firecracker_dma_t dma;              /* Link to the engine context */
    unsigned int channel;               /* The channel number used for this
                                           transfer */
    enum {
        STOPPED,
        RUNNING,
        STOPPING,
    } state;                            /* The current state of the transfer */
    u32 config_low;                     /* Mirror of low CFGx register */
    u32 config_high;                    /* Mirror of high CFGx register */
    firecracker_dma_endpoint_t src;     /* The source endpoint */
    firecracker_dma_endpoint_t dst;     /* The destination endpoint */
    unsigned int flags;                 /* The transfer flags */
    unsigned int count;                 /* bytes to transfer */
    firecracker_dma_handshake_t src_handshake;  /* Params for src */
    firecracker_dma_handshake_t dst_handshake;  /* Params for dst */
    int src_handshaking;                /* Is src hardware handshaking */
    int dst_handshaking;                /* Is dst hardware handshaking */
    firecracker_dma_sg_t gather;        /* Source gather parameters */
    firecracker_dma_sg_t scatter;       /* Destination scatter parameters */
};

/* Definition of the internal state (context, object whatever). One of these
 * exists for each DMA engine in the system
 */
struct firecracker_dma_tag {
    void __iomem	*membase;           /* Virtual base address of the DMA */
    struct channel_tag {
        int allocated;                  /* Channel allocated */
        struct firecracker_dma_xfr_tag transfer; /* Channel transfer */
    } channel[DMA_CHANNELS];            /* Channel information */
    spinlock_t spinlock;                /* Thread synchronisation */
    unsigned long spinlock_flags;
    unsigned int last_int_channel;      /* The last interrupt channel found */
    int handshaking_inuse[DMA_HANDSHAKING_IFS]; /* 1 if a handshaking 
                                                   interface is in use */
#ifdef ENABLE_DEBUGGING
    int lvl;                            /* The Debug level */
#endif
    struct device *device;              /* For dma_alloc_coherent */
};


/* Structure of internal DMA hardware list element */
typedef struct hw_block_descriptor_tag {
    u32 source_addr;                    /* LLI.SARx */
    u32 dest_addr;                      /* LLI.DARx */
    u32 link;                           /* LLI.LLPx */
    u32 control;                        /* LLI.CTLx[31:0] */
    u32 block_ts;                       /* LLI.CTLx[63:32] */
    u32 reserved1;
    u32 reserved2;
} hw_block_descriptor_t;


/* A descriptor holds the hardware descriptor at the start (the LLP will 
 * point to this) and additional information to manage the DMA.
 */
typedef struct descriptor_tag {
    hw_block_descriptor_t hw;       /* Hardware descriptor (must be the
                                       first element */
    struct {
        firecracker_dma_endpoint_t src; /* The source endpoint */
        firecracker_dma_endpoint_t dst; /* The destination endpoint */
        unsigned int count;             /* Bytes to transfer */
    } sw;                           /* Software descriptor */
} descriptor_t;

/* A type that has a size a multiple of 4 bytes. This is used to help us 
 * align the hardware block descriptors to a 32-bit boundary. The
 * size of this type is big enough to hold a struct descriptor_list_tag
 */
typedef struct aligned_descriptor_tag {
    u32 descriptor_data[(sizeof(struct descriptor_tag) / 4) + 1];
} aligned_descriptor_t;

/* Internal state associated with a DMA multi-block list */
struct firecracker_dma_list_tag {
    aligned_descriptor_t *descriptor_list;  /* Pointer the HW/SW list */
    dma_addr_t dma_descriptor_list;     /* DMA address of the list */
    unsigned int length;                /* Total list items */
    unsigned int items;                 /* Current used items */
    descriptor_t *last_desc;            /* Pointer to the last in the 
                                           list */
    firecracker_dma_t dma;              /* Link to the engine context */
    firecracker_dma_xfr_t dma_xfr;      /* Transfer associated with the list */
    int src_list;                       /* 1 if the source is a multi-block
                                           list type. 0 is multi-block
                                           auto-reload or continuous */
    int dst_list;                       /* 1 if the destination is a 
                                           multi-block list type.
                                           0 is multi-block auto-reload or
                                           continuous */
    descriptor_t *current_block;        /* When running, holds the currently
                                           transferring block */
    struct device *device;              /* For dma_alloc_coherent */
};


/* A name for this module */
#define TITLE "pc20x DMA Controller Driver"

/* A version for this module */
#define VERSION "v0.02"

/* The name of the driver used in the system */
#define CARDNAME "pc20x-dmac"

/* Macros for register read/write. These hide the virtual addressing. 
 * They must be used in the presence of a local 'dma' context
 */
#ifndef ENABLE_DEBUGGING

#define DMA_READ(__offset) ioread32(dma->membase + __offset)   

#define DMA_WRITE(__value, __offset)                                    \
    iowrite32(__value, dma->membase + __offset)

#define DB(__params)

#else /* ENABLE_DEBUGGING */

#define DMA_READ(__offset) debug_ioread32(dma->membase + __offset)

#define DMA_WRITE(__value, __offset)                                    \
    debug_iowrite32(__value, dma->membase + __offset)

#define DB(__params) dbug_print __params

/* Debugging level for this module */
static volatile int debug_lvl = ENABLE_DEBUGGING;

/* Debugging levels: */
#define LVL_FATAL       2   /* fatal error conditions         */
#define LVL_ERR         3   /* error conditions         */
#define LVL_WARNING     4   /* warning conditions           */
#define LVL_NOTICE      5   /* normal but significant condition */
#define LVL_INFO        6   /* informational            */
#define LVL_DEBUG       7   /* debug-level messages         */
#define LVL_TRACE       8   /* trace messages         */
#define LVL_TRACE_IO    9   /* Register IO trace messages         */


/* Name: dbug_print
 */
/**Purpose: Printk a debug message if the current debug level is high enough
 *
 * Pre-conditions: The debug level is set
 *
 * Post-conditions: Output via printk
 *
 * Notes: Only built with debug versions (ENABLE_DEBUGGING)
 *
 * @param this_lvl - the debug level of the message
 * @param fmt - va list printf style
 */
static void dbug_print(int lvl, const char *fmt, ...)
{
    static char lvl_ch[10] = {
        '0', '1', 'F', 'E', 'W', 'N', 'I', 'D', 'T', '9'
    };
#define MAX_FORMAT_SIZE 1024
    char buf[MAX_FORMAT_SIZE];
    va_list args;
    va_start(args, fmt);

    if (debug_lvl >= lvl) {

        if (printk_ratelimit() || lvl < LVL_TRACE_IO) {

            vsnprintf(buf, MAX_FORMAT_SIZE, fmt, args);
            printk("dmac <%c>: %s", lvl_ch[lvl], buf);
        }
        else {
            printk("%c", lvl_ch[lvl]);
        }
    }
}


/* Name: debug_ioread32
 */
/**Purpose: Do ioread32 and printk the results
 *
 * Pre-conditions: None.
 *
 * Post-conditions: None.
 *
 * Notes: Used for debugging
 * debug_lvl - The debugging level. Must be >3 to enable printk output.
 *
 * @param p - The address to read from
 *
 * @return the results of ioread32
 *
 * @see debug_iowrite32
 */
static u32 debug_ioread32(void __iomem *p)
{
    u32 v = ioread32(p);
    DB((LVL_TRACE_IO, "ioread32(%p) = 0x%08x\n", p, v));
    return v;
}

/* Name: debug_iowrite32
 */
/**Purpose: Do iowrite32 and printk the parameters
 *
 * Pre-conditions: None.
 *
 * Post-conditions: None.
 *
 * Notes: Used for debugging
 * debug_lvl - The debugging level. Must be >3 to enable printk output.
 *
 * @param v - Value to write
 * @param p - The address to read from
 *
 * @see debug_ioread32
 */
static void debug_iowrite32(u32 v, void __iomem *p)
{
    DB((LVL_TRACE_IO, "iowrite32(%p) = 0x%08x\n", p, v));
    iowrite32(v, p);
}

#endif /* ENABLE_DEBUGGING */



/* Macros for thread synchronisation via the spinlock in the engine 
 * context. Simple locking scheme is employed where the entire API
 * is locked down to one thread per DMA engine.
 * These need to be used in the presence of a local 'dma' context
 * which has to have had its spinlock initialised.
 */
#define GET_LOCK() \
    spin_lock_irqsave(&dma->spinlock, dma->spinlock_flags);

#define RELEASE_LOCK() \
    spin_unlock_irqrestore(&dma->spinlock, dma->spinlock_flags);


/* Time-out for waiting for the FIFO to empty when 
 * stopping a DMA (in jiffies).
 * Note: Make this small as it is used in a busy-waiting loop.
 */
#define ABORT_TIMEOUT       (HZ / 100)      /* 100th of a second */

/* TR_BYTES macro calculates the number of bytes per transaction, given
 * an endpoint.
 */
#define TR_BYTES(__E)       (1 << __E->tr_width)

/* BURST_BYTES macro calculates the number of bytes per burst transfer,
 * given an endpoint.
 * msize_lookup converts from the msize_t enum to transfer width multiplier
 */
static u32 msize_lookup[] = {1, 4, 8, 16, 32};
#define BURST_BYTES(__E)    (TR_BYTES(__E) * msize_lookup[__E->msize])

/* Transfer types used internally */
#define TRT_NONE    0
#define TRT_BURST   1
#define TRT_SINGLE  2
#define TRT_LAST    4


/* Local function prototypes: */
static void print_ctl_low(u32 reg);
static void print_ctl_high(u32 reg);
static void hw_initialise(firecracker_dma_t dma);
static void hw_shutdown(firecracker_dma_t dma);
static firecracker_dma_xfr_t alloc_xfr(firecracker_dma_t dma);
static void free_xfr(firecracker_dma_xfr_t dma_xfr);
static firecracker_dma_list_t alloc_list(
        firecracker_dma_t dma, unsigned int count);
static u32 lookup_tt_fc(int src_periph, int src_fc,
        int dst_periph, int dst_fc);
static u32 build_control_register(
        firecracker_dma_endpoint_t *src, firecracker_dma_endpoint_t *dst);
static u32 build_llp_control_register(firecracker_dma_list_t list);
static void update_xfr_registers(firecracker_dma_xfr_t dma_xfr);
static int get_xfr_state(firecracker_dma_xfr_t dma_xfr);
static void write_int_registers(firecracker_dma_t dma,
        dma_int_type_t int_types, unsigned int type, u32 value);
static int transfer_type(firecracker_dma_xfr_t dma_xfr,
        firecracker_dma_endpoint_t *endpoint, unsigned int *bytes_left);
static int get_transaction_state(
        firecracker_dma_xfr_t dma_xfr, int src_dst);
static void transfer_request(
        firecracker_dma_xfr_t dma_xfr, int src_dst, int type);
static void set_auto_burst_length(
        firecracker_dma_endpoint_t *endpoint, unsigned int count);
static int param_checks(firecracker_dma_endpoint_t *src,
        firecracker_dma_endpoint_t *dst, unsigned int count);
static void setup_handshaking(firecracker_dma_xfr_t dma_xfr,
        firecracker_dma_handshake_t *src_handshake,
        firecracker_dma_handshake_t *dst_handshake);
static int handshaking_check(firecracker_dma_t dma, 
        firecracker_dma_handshake_t *src_handshaking, 
        firecracker_dma_handshake_t *dst_handshaking);
#ifdef ENABLE_DEBUGGING
static u32 debug_ioread32(void __iomem *p);
static void debug_iowrite32(u32 v, void __iomem *p);
static void dbug_print(int this_lvl, const char *fmt, ...);
#endif /* ENABLE_DEBUGGING */


/* Name: firecracker_dma_init
 */
/**Purpose: Engine initialisation
 *
 * Pre-conditions: None.
 *
 * Post-conditions: DMA engine context initialised. Ready for API calls.
 * Hardware initialised, DMA engines enabled.
 *
 * @return 0 - success always 
 */
static struct firecracker_dma_tag *firecracker_dma_init(
        struct platform_device *pdev)
{
    struct plat_firecracker_dma *pdata = pdev->dev.platform_data;
    struct firecracker_dma_tag *dma;

    dma = kzalloc(sizeof(struct firecracker_dma_tag), GFP_ATOMIC);
    if (dma == NULL) {
        /* No memory, return null */
        DB((LVL_ERR, "Out of memory to allocate engine\n"));
        return NULL;
    }

    /* Set the base address */
    dma->membase = pdata->membase;

    /* Get a pointer to the underlying device for dma_alloc_coherent calls */
    dma->device = &pdev->dev;

#ifdef ENABLE_DEBUGGING
    /* Setup debug level */
    dma->lvl = ENABLE_DEBUGGING;
#endif

    /* Allocate a spinlock for each engine */
    spin_lock_init(&dma->spinlock);

    /* Initialise engine hardware */
    hw_initialise(dma);

    return dma;
}

/* Name: firecracker_dma_exit
 */
/**Purpose: Engine exit
 *
 * Pre-conditions: None.
 *
 * Post-conditions: Hardware shut-down, DMA engines disabled. Context freed.
 */
static void firecracker_dma_exit(struct firecracker_dma_tag *dma)
{
    if (dma != NULL) {
        /* Stop the hardware */
        hw_shutdown(dma);

        /* Free engine context */
        kfree(dma);
    }
}


/*****************************************************************************
3.1.5. firecracker_dma_setup_direct_xfr
---------------------------------------

firecracker_dma_xfr_t firecracker_dma_setup_direct_xfr(
        firecracker_dma_t dma,
        firecracker_dma_endpoint_t *src, firecracker_dma_endpoint_t *dst,
        firecracker_dma_handshake_t *src_handshaking,
        firecracker_dma_handshake_t *dst_handshaking,
        unsigned int count, unsigned int flags, void *cookie)


3.1.5.1. Description

Set up a DMA transfer between src and dst.
Note that 'count' is the number of bytes, not the number of words.

The DMA set up by this call will not start transferring data until
firecracker_dma_start is called.

The data structure pointed to by the src and dst parameters are copied
by the call so can be destroyed once the call returns.

The DMA channel set up by this call will have:
    - All interrupts disabled
    - Channel disabled but not suspended
    - Source and Destination address set to the parameters (see 3.2.2)
    - Single block transfer mode
    - Transfer size set to count
    - Source and Destination Master interface set to the parameters (see 3.2.2)
    - Transfer Type and Flow control set according to parameters (see 3.2.2)
    - No Scatter or Gather.
    - Source/Dest Burst Length set according to parameters (see 3.2.2)
    - Source/Dest address increment set according to parameters (see 3.2.2)
    - Source/Dest Transfer Width set according to parameters (see 3.2.2)
    - Protocol Control set according to flags
    - Transfer FIFO mode set according to flags
    - Data pre-fetching is set according to flags
    - Automatic source/destination reload set by the parameters (see 3.2.2)
    - Channel priority set according to flags
    - Source/Dest handshaking setup according to parameters.

If nether of the src or dst are auto-reload, the transfer setup by this call is
single-block only and will stop when the data has been transferred.
Free running transfers (multi-block) will wait between blocks if the block
interrupt is enabled. See 3.2.2 and 3.1.11.

The 'count' parameter is used in one of three ways.
(1) If one end of the transfer is the flow controller and has hardware handshaking,
    the count is ignored as the flow controller will control the amount of data in
    the block.
(2) If nether end of the transfer is a flow controller (the DMA engine is flow 
    controller), the count parameter is used to set the block size in the DMA
    control register.
(3) If one end of the transfer is a flow controller, but software handshaking is
    used, the count parameter is ignored. In this situation, client software may
    not know the size of the block at the point when the transfer is setup. The
    firecracker_dma_request_transaction function bytes_left parameter is used to 
    tell the driver the data left in the block when each transaction is requested.
    firecracker_dma_request_transaction can then decide the type of transaction
    to request.


3.1.5.2. Parameters
dma                 - The handle to the dma driver returned from 
                        firecracker_dma_get_dmac_handle.
src                 - Transfer source details (see 3.2.2)
dst                 - Transfer destination details (see 3.2.2)
src_handshake      - The handshaking parameters for transfer source or NULL
                     if hardware handshaking of source is not used (see 3.2.4)
dst_handshake      - The handshaking parameters for transfer destination or NULL
                     if hardware handshaking of destination is not used.
count               - The number of bytes to transfer. If either end of the
                        transfer is a hardware handshaking flow controller,
                        this is ignored.
flags | PROTCTL_0   - Sets the PROTCTL bits
flags | PROTCTL_1
flags | PROTCTL_2
flags | PROTCTL_3
flags | FIFO_MODE   - Sets the FIFO_MODE bit
flags | FCMODE      - Sets the FCMODE bit
flags | CH_PRIOR_0  \
flags | CH_PRIOR_1  - Sets the
flags | CH_PRIOR_2  -  channel priority
flags | CH_PRIOR_3  /
cookie              - Pointer returned by firecracker_dma_int_get_xfr

3.1.5.3. Returns
Handle to the new transfer created or NULL if something went wrong.
*****************************************************************************/
firecracker_dma_xfr_t firecracker_dma_setup_direct_xfr(
        firecracker_dma_t dma,
        firecracker_dma_endpoint_t *src, firecracker_dma_endpoint_t *dst,
        firecracker_dma_handshake_t *src_handshaking,
        firecracker_dma_handshake_t *dst_handshaking,
        unsigned int count, unsigned int flags, void *cookie)
{
    struct firecracker_dma_xfr_tag *dma_xfr = NULL;

    DB((LVL_INFO, "Enter firecracker_dma_setup_direct_xfr\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Check we have source and destination */
    if (src == NULL || dst == NULL) {
        DB((LVL_ERR, "src or dst NULL\n"));
        goto EXIT;
    }

    /* Do various parameter checks and exit if they fail */
    if (param_checks(src, dst, count) == 0) {
        goto EXIT;
    }

    /* Check the handshaking parameters */
    if (handshaking_check(dma, src_handshaking, dst_handshaking) == 0) {
        goto EXIT;
    }

    /* Allocate a transfer object including DMA channel */
    dma_xfr = alloc_xfr(dma);
    if (dma_xfr == NULL) {
        /* Out of resources */
        goto EXIT;
    }

    /* Set the transfer state */
    dma_xfr->state = STOPPED;

    /* Setup transfer structure */
    dma_xfr->src = *src;
    dma_xfr->dst = *dst;
    dma_xfr->count = count;
    dma_xfr->flags = flags;
    dma_xfr->list = NULL;
    dma_xfr->cookie = cookie;

    /* Deal with auto burst length */
    set_auto_burst_length(&dma_xfr->src, count);
    set_auto_burst_length(&dma_xfr->dst, count);

    /* Setup the handshaking */
    setup_handshaking(dma_xfr, src_handshaking, dst_handshaking);

    /* Disable all interrupts for the channel */
    write_int_registers(dma, INT_ALL, 
            MASK, DMA_IRQ_DISABLE_CHANNEL(dma_xfr->channel));

EXIT:
    RELEASE_LOCK();
    return dma_xfr;
}



/*****************************************************************************
3.1.1. firecracker_dma_start
----------------------------

int firecracker_dma_start(firecracker_dma_xfr_t dma_xfr)

3.1.1.1. Description
Start a DMA transfer on a channel previously set up with the 
firecracker_dma_setup_direct_xfr() or firecracker_dma_setup_list_xfr()
functions.
Once started, the transfer is locked and will not be modifiable. This
includes a DMA multi-block list that is associated with a transfer.
Transfers will continue to completion asynchronously. Once finished,
they can be restarted.

3.1.1.2. Parameters
dma_xfr             - The transfer handle.

3.1.1.3. Returns
EINVAL              - The transfer is already running
*****************************************************************************/
int firecracker_dma_start(firecracker_dma_xfr_t dma_xfr)
{
    firecracker_dma_t dma = dma_xfr->dma;
    firecracker_dma_list_t list = dma_xfr->list;
    unsigned int ch = dma_xfr->channel;
    int res = 0;

    DB((LVL_INFO, "Enter firecracker_dma_start\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Check and set the transfer state */
    if (get_xfr_state(dma_xfr) != STOPPED) {
        /* Already running error */
        DB((LVL_ERR, "Transfer running\n"));
        res = EINVAL;
        goto EXIT;
    }
    dma_xfr->state = RUNNING;

    /* Reset the current block if using multi-block list */
    if (list != NULL) {
        list->current_block = (descriptor_t *)list->descriptor_list;
    }

    /* Write the DMA hardware */
    update_xfr_registers(dma_xfr);

    /* Start the DMA channel */
    DMA_WRITE(DMA_ENABLE_CHANNEL(ch), DMA_CHANNEL_ENABLE_REG_OFFSET);

EXIT:
    RELEASE_LOCK();
    return res;
}


/*****************************************************************************
3.1.2. firecracker_dma_abort
----------------------------

int firecracker_dma_abort(firecracker_dma_xfr_t dma_xfr)

3.1.2.1. Description
Abort a DMA transfer.
Once aborted, the transfer can be restarted. Also the transfer 
can be modified once aborted. If the transfer is restarted, it
will continue from the start of the single transfer buffer or
multi-block list.

3.1.2.2. Parameters
dma_xfr             - The transfer handle.

3.1.2.3. Returns
EINVAL              - The transfer is not running
*****************************************************************************/
int firecracker_dma_abort(firecracker_dma_xfr_t dma_xfr)
{
    firecracker_dma_t dma = dma_xfr->dma;
    int res = 0;
    u32 status;
    unsigned long timeout_jiffies;

    DB((LVL_INFO, "Enter firecracker_dma_abort\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Check and set the transfer state */
    if (get_xfr_state(dma_xfr) != RUNNING) {
        /* Already stopped. It is not unusual for this to happen */
        DB((LVL_NOTICE, "Aborting a stopped transfer\n"));
        res = EINVAL;
        goto EXIT;
    }
    dma_xfr->state = STOPPING;

    /* Set the suspend bit, wait for the FIFO to clear before
     * disabling the channel
     */
    dma_xfr->config_low |= DMA_CH_SUSP;
    DMA_WRITE(dma_xfr->config_low,
            DMA_N_LOW_CONFIGURATION_REG_OFFSET(dma_xfr->channel));

    /* Release the API spinlock while waiting as interrupts may
     * need to be serviced to empty the FIFO and these will try
     * to grab the lock.
     * This is safe as we are not updating state while polling.
     */
    RELEASE_LOCK();

    /* Poll the FIFO_EMPTY bit */
    timeout_jiffies = jiffies + ABORT_TIMEOUT;
    while (1) {

        /* Check for time out */
        if (time_after(jiffies, timeout_jiffies)) {
            DB((LVL_ERR, "Timed out waiting for FIFO to empty\n"));
            res = EIO;
            break;
        }

        /* Check for the empty FIFO empty */
        status = DMA_READ(
            DMA_N_LOW_CONFIGURATION_REG_OFFSET(dma_xfr->channel));
        if (status & DMA_FIFO_EMPTY) {
            break;
        }
    };

    GET_LOCK();

    /* Disable the channel */
    DMA_WRITE(
            DMA_DISABLE_CHANNEL(dma_xfr->channel), 
            DMA_CHANNEL_ENABLE_REG_OFFSET);

    /* Now stopped, set state */
    dma_xfr->state = STOPPED;

EXIT:
    RELEASE_LOCK();
    return res;
}


/*****************************************************************************
3.1.3. firecracker_dma_release
------------------------------

firecracker_dma_release(firecracker_dma_xfr_t dma_xfr)

3.1.3.1. Description
Stop a DMA transfer and release resources.
Once released, the transfer can not be restarted and has to be setup
once more. The transfer handle will become invalid on exit of this
function.

3.1.3.2. Parameters
dma_xfr             - The transfer handle.
*****************************************************************************/
void firecracker_dma_release(firecracker_dma_xfr_t dma_xfr)
{
    firecracker_dma_t dma = dma_xfr->dma;
    firecracker_dma_list_t list = dma_xfr->list;

    DB((LVL_INFO, "Enter firecracker_dma_release\n"));

    /* Make sure the transfer has is stopped */
    firecracker_dma_abort(dma_xfr);     /* Ignoring the returned value */

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Reset any interrupts that have been left pending */
    write_int_registers(dma, INT_ALL, CLEAR, DMA_IRQ_CHANNEL(dma_xfr->channel));

    /* Disconnect from multi-block list */
    if (list != NULL) {
        list->dma_xfr = NULL;
    }

    /* Free the resources */
    free_xfr(dma_xfr);

    RELEASE_LOCK();
}


/*****************************************************************************
3.1.4. firecracker_dma_set_debug_level
--------------------------------------

firecracker_dma_set_debug_level(firecracker_dma_t dma, int lvl)

3.1.4.1. Description
Sets the debugging level for the DMA driver, 0 - no debug, 3 - all the debug.
Affects the amount of printk traffic.

3.1.4.2. Parameters
dma                 - The handle to the dma driver returned from 
                        firecracker_dma_get_dmac_handle.
lvl                 - Debug level (0-3).
*****************************************************************************/
void firecracker_dma_set_debug_level(firecracker_dma_t dma, int lvl)
{
#ifdef ENABLE_DEBUGGING
    debug_lvl = lvl;
#endif
}


/*****************************************************************************
3.1.6. firecracker_dma_list_create
----------------------------------

firecracker_dma_list_t firecracker_dma_list_create(
        firecracker_dma_t dma,
        unsigned int count)

3.1.6.1. Description
Creates a DMA buffer list that can be assigned to a multi-block transfer.
The list initially is empty. Entries can be added to the list using 
firecracker_dma_list_add.
A list can be assigned to one transfer only using firecracker_dma_setup_list_xfr.
The list can only be modified before any transfer is started.
This function allocates memory. Lists must be destroyed by calling 
firecracker_dma_list_destroy otherwise the driver will leak memory.

Circular multi-block lists have a link from the last element to the first. 
When applied to the DMA hardware, this means that the multi-block transfer
restarts once it gets to the end of the list.
The firecracker_dma_handle_block_int function should be used to set the
number of blocks of the multi-block transfer.

3.1.6.2. Parameters
dma                 - The handle to the dma driver returned from 
                        firecracker_dma_get_dmac_handle.
count               - The maximum number of elements in the list.

3.1.6.3. Returns
The address of the list created or NULL if the request could not be satisfied.
*****************************************************************************/
firecracker_dma_list_t firecracker_dma_list_create(
        firecracker_dma_t dma, unsigned int count)
{
    firecracker_dma_list_t list;

    DB((LVL_INFO, "Enter firecracker_dma_list_create\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Allocate the list */
    list = alloc_list(dma, count);
    if (list == NULL) {
        /* Not enough resources, exit */
        goto EXIT;
    }

    /* Setup the structure */
    list->dma = dma;
    list->dma_xfr = NULL;
    list->items = 0;
    list->last_desc = NULL;
    list->length = count;
    list->src_list = 1;
    list->dst_list = 1;

EXIT:
    RELEASE_LOCK();
    return list;
}


/*****************************************************************************
3.1.7. firecracker_dma_list_add
-------------------------------

firecracker_dma_list_add(firecracker_dma_list_t list,
                         firecracker_dma_endpoint_t *src, firecracker_dma_endpoint_t *dst,
                         unsigned int count)

3.1.7.1. Description
Add an entry to the end of a DMA multi-block list. The entry added to the list
specifies a simple block transfer.
The data pointed to by the src and dst parameters are copied by the call so
can be destroyed once the call returns.
This function will not modify a list that is currently in use with a DMA 
operation.

The first element of the list must have both src and dst endpoints. If either
endpoint (not both) is to be continuous, subsequent elements need to set 
the endpoint to NULL. If either endpoint (not both) is to be auto-reload,
the first element needs to set the auto_reload flag and subsequent elements
need to set the endpoint to NULL. See 3.2.2.

Notes:
1.  Each block of the list can set a transfer width. Transfer width affects the
    setting of scatter/gather which is set globally, for the transfer. Scatter/
    gather is setup in the hardware based on the transfer widths of the first
    element of the block list. It is recommended that subsequent blocks use
    the same transfer width (if SG is enabled), otherwise there will be
    undefined effects.

3.1.7.2. Parameters
list                - The DMA multi-block list to add a new block to
src                 - The source DMA endpoint parameters 
                        or NULL if auto-reload or continuous
dst                 - The destination endpoint parameters
                        or NULL if auto-reload or continuous
count               - The number of bytes to transfer. If either end of the
                        transfer is a hardware flow controller, this is ignored.
    
3.1.7.3. Returns
ENOMEM              - The list is full, no more items can be added
EBUSY               - If the list is in use with a DMA transfer
EINVAL              - The combination of source and destination endpoints is
                        invalid.
*****************************************************************************/
int firecracker_dma_list_add(
        firecracker_dma_list_t list,
        firecracker_dma_endpoint_t *src, firecracker_dma_endpoint_t *dst,
        unsigned int count)
{
    firecracker_dma_t dma = list->dma;
    int res = 0;
    descriptor_t *new_desc;
    dma_addr_t dma_new_desc;
    firecracker_dma_xfr_t dma_xfr = list->dma_xfr;
    descriptor_t *first_desc = (descriptor_t *)list->descriptor_list;

    DB((LVL_INFO, "Enter firecracker_dma_list_add\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Do various parameter checks and exit if they fail */
    if (param_checks(src, dst, count) == 0) {
        res = EINVAL;
        goto EXIT;
    }

    /* Check we have room */
    if (list->length == list->items) {
        DB((LVL_ERR, "List full\n"));
        res = ENOMEM;
        goto EXIT;
    }

    /* Check for transfers running the list */
    if (dma_xfr != NULL) {
        if (get_xfr_state(dma_xfr) != STOPPED) {
            /* Cannot modify a list that is active */
            DB((LVL_ERR, "Transfer running\n"));
            res = EBUSY;
            goto EXIT;
        }
    }

    /* If this is the first item, we must have both src and dst parameters */
    if (list->items == 0) {
        if (src == NULL || dst == NULL) {
            DB((LVL_ERR, "First list element must have both source and destination endpoints\n"));
            res = EINVAL;
            goto EXIT;
        }
    }
    /* If this is the second element and src or dst are NULL, 
     * we are not using a list for the endpoint
     */
    else if (list->items == 1) {
        if (src == NULL && dst == NULL) {
            /* Either src or dst must be set */
            DB((LVL_ERR, "List element must have either source or destination endpoints\n"));
            res = EINVAL;
            goto EXIT;
        }

        /* If the source is not set, we are not using a list for it */
        if (src == NULL) {
            list->src_list = 0;
        }

        /* If the destination is not set, we are not using a list for it */
        if (dst == NULL) {
            list->dst_list = 0;
        }
    }
    else {  /* Not the first or second item */

        /* Check that, once we have chosen a non list type multi-block transfer
         * (auto-reload or continuous), we must stick to it.
         */
        if (list->src_list == 1) {
            if (src == NULL) {
                res = EINVAL;
            }
        }
        else {
            if (src != NULL) {
                res = EINVAL;
            }
        }

        if (list->dst_list == 1) {
            if (dst == NULL) {
                res = EINVAL;
            }
        }
        else {
            if (dst != NULL) {
                res = EINVAL;
            }
        }

        if (res == EINVAL) {
            DB((LVL_ERR, "List element must be consistent\n"));
            goto EXIT;
        }
    }

    /* Get a pointer to the new descriptor */
    new_desc = (descriptor_t *)&(list->descriptor_list[list->items]);

    /* Get the physical address (DMA address) of the new descriptor */
    dma_new_desc = 
        list->dma_descriptor_list + 
        (sizeof(aligned_descriptor_t) * list->items);

    /* Allocate the new descriptor */
    list->items++;

    /* Record the first source/destination for the non-list type
     * transfers in the software list.
     */
    if (src != NULL) {
        new_desc->sw.src = *src;
        /* Deal with auto burst length */
        set_auto_burst_length(&new_desc->sw.src, count);
    }
    if (dst != NULL) {
        new_desc->sw.dst = *dst;
        /* Deal with auto burst length */
        set_auto_burst_length(&new_desc->sw.dst, count);
    }
    new_desc->sw.count = count;

    /* Set the link of the new last element to point to the first. 
     * This ensures that the hardware is ready to be restarted at the 
     * beginning of the list when it completes
     */
    new_desc->hw.link = list->dma_descriptor_list;

    /* Link the previous item to this one and enable the block chaining
     * for it.
     */
    if (list->last_desc != NULL) {
        list->last_desc->hw.link = dma_new_desc;
        list->last_desc->hw.control |= build_llp_control_register(list);
    }

    /* Where either the source or destination is not list type, and is null
     * at this point, use the endpoint descriptor at the head of the list
     */
    if (src == NULL) {
        src = &first_desc->sw.src;
    }
    if (dst == NULL) {
        dst = &first_desc->sw.dst;
    }

    /* Setup the new hardware block descriptor */
    new_desc->hw.source_addr = src->dma_addr;
    new_desc->hw.dest_addr = dst->dma_addr;
    new_desc->hw.control = build_control_register(src, dst);
    new_desc->hw.block_ts = count / TR_BYTES(src);

    /* The new last descriptor is now the one we just created */
    list->last_desc = new_desc;

EXIT:
    RELEASE_LOCK();
    return res;
}


/*****************************************************************************
3.1.8. firecracker_dma_list_clear
---------------------------------

firecracker_dma_list_clear(firecracker_dma_list_t list)


3.1.8.1. Description
This function resets a list to the state that it was in just after it was 
created when calling firecracker_dma_list_create. This function does
not free the memory associated by the list.
This function will not modify a list that is currently in use with a DMA 
operation.


3.1.8.2. Parameters
list                - The DMA list returned from firecracker_dma_list_create


3.1.8.3. Returns
EBUSY - If the list is in use with a DMA transfer
*****************************************************************************/
int firecracker_dma_list_clear(firecracker_dma_list_t list)
{
    firecracker_dma_t dma = list->dma;
    int res = 0;
    struct firecracker_dma_xfr_tag *dma_xfr = list->dma_xfr;

    DB((LVL_INFO, "Enter firecracker_dma_list_clear\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Check for transfers running the list */
    if (dma_xfr != NULL) {
        if (get_xfr_state(dma_xfr) != STOPPED) {
            /* Cannot modify a list that is active */
            DB((LVL_ERR, "Transfer running\n"));
            res = EBUSY;
            goto EXIT;
        }
    }

    /* Reset the list */
    list->items = 0;
    list->last_desc = NULL;
    list->src_list = 1;
    list->dst_list = 1;

EXIT:
    RELEASE_LOCK();
    return res;
}


/*****************************************************************************
3.1.9. firecracker_dma_list_destroy
-----------------------------------

firecracker_dma_list_destroy(firecracker_dma_list_t list)


3.1.9.1. Description
This function frees the resources allocated by a list. Lists need to
be freed after use to avoid a memory leak.
This function will not modify a list that is currently in use with a DMA 
operation.


3.1.9.2. Parameters
list                - The DMA list returned from firecracker_dma_list_create


3.1.9.3. Returns
EBUSY - If the list is in use with a DMA transfer
*****************************************************************************/
int firecracker_dma_list_destroy(firecracker_dma_list_t list)
{
    firecracker_dma_t dma = list->dma;
    int res = 0;
    struct firecracker_dma_xfr_tag *dma_xfr = list->dma_xfr;
    struct firecracker_dma_list_tag local_list;

    DB((LVL_INFO, "Enter firecracker_dma_list_destroy\n"));

    /* Stop compiler warnings */
    local_list.device = NULL;
    local_list.length = 0;
    local_list.descriptor_list = NULL;
    local_list.dma_descriptor_list = 0;

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Make sure there are no references to the list from transfers.
     * A transfers using the list needs to be destroyed before the
     * list can be destroyed.
     */
    if (dma_xfr != NULL) {
        /* Cannot destroy a list that is referenced */
        DB((LVL_ERR, "List in use\n"));
        res = EBUSY;
        goto EXIT;
    }

    /* Take a copy of the list before we free it so that we can free 
     * the DMA coherent area outside of the spinlock. This is done
     * because dma_free_coherent must not be called with interrupts
     * disabled.
     */
    local_list = *list;

    /* Free the object */
    kfree(list);

EXIT:
    RELEASE_LOCK();

    /* Free the descriptor list if were are not here because of an error */
    if (res == 0) {
        dma_free_coherent(local_list.device, 
                sizeof(aligned_descriptor_t) * local_list.length,
                local_list.descriptor_list, local_list.dma_descriptor_list);
    }

    return res;
}


/*****************************************************************************
3.1.10. firecracker_dma_setup_list_xfr
--------------------------------------

firecracker_dma_xfr_t firecracker_dma_setup_list_xfr(
        firecracker_dma_list_t list,
        firecracker_dma_handshake_t *src_handshaking,
        firecracker_dma_handshake_t *dst_handshaking,
        unsigned int flags, void *cookie)


3.1.10.1. Description
Set up a multi-block DMA transfer.
The transfer is controlled by the list provided. 
The DMA list can be modified once this function has been called but
when the DMA operations is started (by calling firecracker_dma_start) the
list will be locked and any attempt to modify it will not be allowed.
Data transfer is not started until the firecracker_dma_start is called.

The transfer setup by this call is multi-block only and will stop when the 
end of the multi-block list is reached. In addition, the transfer will wait
between blocks if the block interrupt is enabled. See 3.2.6.1.

Source/Dest handshaking setup according to parameters.

3.1.10.2. Parameters
list                - The DMA multi-block list containing the data blocks
                        to be transferred.
src_handshake      - The handshaking parameters for transfer source or NULL
                     if hardware handshaking of source is not used (see 3.2.4)
dst_handshake      - The handshaking parameters for transfer destination or NULL
                     if hardware handshaking of destination is not used.
flags | PROTCTL_0   - Sets the PROTCTL bits
flags | PROTCTL_1
flags | PROTCTL_2
flags | PROTCTL_3
flags | FIFO_MODE   - Sets the FIFO_MODE bit
flags | FCMODE      - Sets the FCMODE bit
flags | CH_PRIOR_0  \
flags | CH_PRIOR_1  - Sets the
flags | CH_PRIOR_2  -  channel priority
flags | CH_PRIOR_3  /
cookie              - Pointer returned by firecracker_dma_int_get_xfr

3.1.10.3. Returns
ENOMEM              - No enough resources to complete the call
EBUSY               - If the list is already in use with a DMA transfer
EINVAL              - Combination of parameters is illegal.
*****************************************************************************/
firecracker_dma_xfr_t firecracker_dma_setup_list_xfr(
        firecracker_dma_list_t list,
        firecracker_dma_handshake_t *src_handshaking,
        firecracker_dma_handshake_t *dst_handshaking,
        unsigned int flags, void *cookie)
{
    firecracker_dma_t dma = list->dma;
    struct firecracker_dma_xfr_tag *dma_xfr = NULL;

    DB((LVL_INFO, "Enter firecracker_dma_setup_list_xfr\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Check that the list is not in use with another transfer */
    if (list->dma_xfr != NULL) {
        DB((LVL_ERR, "List in use\n"));
        goto EXIT;
    }

    /* Check the handshaking parameters */
    if (handshaking_check(dma, src_handshaking, dst_handshaking) == 0) {
        goto EXIT;
    }

    /* Allocate a transfer object including DMA channel */
    dma_xfr = alloc_xfr(dma);
    if (dma_xfr == NULL) {
        /* Out of resources */
        goto EXIT;
    }

    /* Set the transfer state */
    dma_xfr->state = STOPPED;

    /* Setup transfer structure */
    dma_xfr->flags = flags;
    dma_xfr->list = list;
    dma_xfr->cookie = cookie;

    /* Pointer from list to the new transfer */
    list->dma_xfr = dma_xfr;

    /* Setup the handshaking */
    setup_handshaking(dma_xfr, src_handshaking, dst_handshaking);

    /* Disable all interrupts for the channel */
    write_int_registers(dma, INT_ALL, 
            MASK, DMA_IRQ_DISABLE_CHANNEL(dma_xfr->channel));

EXIT:
    RELEASE_LOCK();
    return dma_xfr;
}


/*****************************************************************************
3.1.11. firecracker_dma_enable_int
----------------------------------------

firecracker_dma_enable_int(firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types)

3.1.11.1. Description
Enable interrupt generation of a number of types of interrupt on a DMA transfer.

3.1.11.2. Parameters
dma_xfr             - The transfer handle.
int_types           - The types of interrupt (see 3.2.6).
*****************************************************************************/
void firecracker_dma_enable_int(
        firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types)
{
    firecracker_dma_t dma = dma_xfr->dma;

    DB((LVL_INFO, "Enter firecracker_dma_enable_int\n"));

    GET_LOCK();
    write_int_registers(dma, int_types, 
            MASK, DMA_IRQ_ENABLE_CHANNEL(dma_xfr->channel));
    RELEASE_LOCK();
}


/*****************************************************************************
3.1.12. firecracker_dma_disable_int
-----------------------------------------

firecracker_dma_disable_int(firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types)

3.1.12.1. Description
Disable interrupt generation of a number of types of interrupt on a DMA transfer.

3.1.12.2. Parameters
dma_xfr             - The transfer handle.
int_types           - The types of interrupt (see 3.2.6).
*****************************************************************************/
void firecracker_dma_disable_int(
        firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types)
{
    firecracker_dma_t dma = dma_xfr->dma;

    DB((LVL_INFO, "Enter firecracker_dma_disable_int\n"));

    GET_LOCK();
    write_int_registers(dma, int_types, 
            MASK, DMA_IRQ_DISABLE_CHANNEL(dma_xfr->channel));
    RELEASE_LOCK();
}


/*****************************************************************************
3.1.13. firecracker_dma_clear_int
-----------------------------------------

firecracker_dma_clear_int(firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types)

3.1.13.1. Description
Clears a number of interrupt status bits, ready for new interrupt generation.

3.1.13.2. Parameters
dma_xfr             - The transfer handle.
int_types           - The types of interrupt (see 3.2.6).
*****************************************************************************/
void firecracker_dma_clear_int(
        firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types)
{
    firecracker_dma_t dma = dma_xfr->dma;

    DB((LVL_INFO, "Enter firecracker_dma_clear_int\n"));

    GET_LOCK();
    write_int_registers(dma, int_types, 
            CLEAR, DMA_IRQ_CHANNEL(dma_xfr->channel));
    RELEASE_LOCK();
}


/*****************************************************************************
3.1.28. firecracker_dma_int_get_xfr
-----------------------------------

firecracker_dma_xfr_t firecracker_dma_int_get_xfr(
        firecracker_dma_t dma, dma_int_type_t *int_types, void **cookie)

3.1.28.1. Description
Returns the transfer handle that caused the last interrupt, so long as the interrupt 
is one of int_types.
This function also returns the interrupt type that caused the interrupt in the
location pointed to be int_types.

3.1.28.2. Parameters
dma                 - The engine handle.
int_types           - Pointer to types to search for and if one found, returns the 
                        type of the interrupt.
cookie              - Pointer to pointer set on exit to the cookie value set in
                        firecracker_dma_setup_direct_xfr or 
                        firecracker_dma_setup_list_xfr.

3.1.28.3. Returns
A transfer handle or NULL if no interrupt occurred.
*****************************************************************************/
firecracker_dma_xfr_t firecracker_dma_int_get_xfr(
        firecracker_dma_t dma, dma_int_type_t *interrupt_type,
        void **cookie)
{
    firecracker_dma_xfr_t dma_xfr = NULL;
    dma_int_type_t int_type;
    unsigned int int_channel = 0;
    u32 hw_int_status = 0;
 
    DB((LVL_INFO, "Enter firecracker_dma_int_get_xfr\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Search for an interrupt type that has been triggered */
    for (int_type = 1; int_type < INT_ALL; int_type <<= 1) {

        /* If the interrupt type is not one that has been requested, move
         * on to the next type.
         */
        if (((*interrupt_type) & int_type) == 0) {
            continue;
        }

        /* Read one of the interrupt status registers */
        /* Note: the order that the interrupt types appear in the following
         * switch is important as it defines the order in which interrupts 
         * are serviced.
         * The dma_int_type_t needs to be in the same order as this switch.
         */
        switch (int_type) {
            case INT_BLOCK:
                hw_int_status = 
                    DMA_READ(DMA_BLOCK_COMPLETE_REG_OFFSET(STATUS));
                break;

            case INT_DST_TRANSACTION:
                hw_int_status = 
                    DMA_READ(DMA_DST_TRX_COMPLETE_REG_OFFSET(STATUS));
                break;

            case INT_ERROR:
                hw_int_status = 
                    DMA_READ(DMA_ERROR_REG_OFFSET(STATUS));
                break;

            case INT_SRC_TRANSACTION:
                hw_int_status = 
                    DMA_READ(DMA_SRC_TRX_COMPLETE_REG_OFFSET(STATUS));
                break;

            case INT_TRANSFER:
                hw_int_status = 
                    DMA_READ(DMA_TRANSFER_COMPLETE_REG_OFFSET(STATUS));
                break;

            default:
                /* Execution should not get here */
                hw_int_status = 0;
        }

        /* Now find a channel number. Do a round
         * robin search to avoid starving any channels
         */
        int_channel = dma->last_int_channel;
        do {
            /* Move on to the next channel */
            int_channel++;
            if (int_channel >= DMA_CHANNELS) {
                int_channel = 0;
            }

            /* Break if the channel status is set and the channel
             * is enabled.
             */
            if ((hw_int_status & (1 << int_channel)) != 0 &&
                    dma->channel[int_channel].allocated == 1) {

                /* Channel found, set the transfer handle to be returned */
                dma_xfr = &dma->channel[int_channel].transfer;
                if (cookie != NULL) {
                    *cookie = dma_xfr->cookie;
                }
                *interrupt_type = int_type;
                break;
            }

            /* Repeat until we have got back to where we left off */
        } while (int_channel != dma->last_int_channel);
        
        if (dma_xfr != NULL) {
            /* Channel found, stop looking */
            break;
        }
    }

    if (dma_xfr == NULL) {
        DB((LVL_INFO, "firecracker_dma_int_get_xfr, no xfr found\n"));
    }

    /* Record the channel so that we do not service it next time
     * without looking for others first.
     */
    dma->last_int_channel = int_channel;

    RELEASE_LOCK();
    return dma_xfr;
}


/*****************************************************************************
3.1.32. firecracker_dma_handle_block_int
----------------------------------------

void firecracker_dma_handle_block_int(
        firecracker_dma_xfr_t dma_xfr, unsigned int *blocks_left)

3.1.32.1 Description
This function should be called by client code from the block complete interrupt
handler. This function handles proper termination of multi-block transfers by 
resetting the CFG.RELOAD_SRC, CFG.RELOAD_DST, CTL.LLP_SRC_EN and CTL.LLP_DST_EN bits
on the last but one block of the transfer.
The blocks_left parameter points to a count of the number of blocks in the transfer.
blocks_left is updated on exit of the function.

3.1.32.2. Parameters
dma_xfr             - The transfer handle.
blocks_left         - Pointer to the count of the blocks remaining in the transfer.
                        This is updated at the end of the transfer

3.1.32.3. Returns
EINVAL - The transfer is not running.
*****************************************************************************/
int firecracker_dma_handle_block_int(
        firecracker_dma_xfr_t dma_xfr, unsigned int *blocks_left)
{
    firecracker_dma_t dma = dma_xfr->dma;
    firecracker_dma_list_t list = dma_xfr->list;
    int res = 0;
    u32 curr_offset;

    DB((LVL_INFO, "Enter firecracker_dma_handle_block_int\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Check the transfer state */
    if (get_xfr_state(dma_xfr) != RUNNING) {
        /* Not running error */
        DB((LVL_ERR, "Transfer not running\n"));
        res = EINVAL;
        goto EXIT;
    }

    /* Update the number of blocks left */
    *blocks_left = *blocks_left - 1;

    /* Update the current_block pointer to the next in the list. This
     * value is currently in the LLP. The LLP will be dereferenced 
     * (by hardware) on exit of the block complete interrupt so it
     * should not change while we execute this code.
     */
    if (list != NULL) {

        /* Get the offset from the start of the list to the current block */
        curr_offset = 
            DMA_READ(DMA_N_LINKED_LIST_POINTER_REG_OFFSET(dma_xfr->channel)) -
            list->dma_descriptor_list;

        /* Get the current block as the offset from the first */
        list->current_block = 
            (descriptor_t *)(((char *)list->descriptor_list) + curr_offset);
    }

    /* Reset the auto-reload enable bits on the last but one block */
    if (*blocks_left == 1) {
        dma_xfr->config_low &= ~DMA_RELOAD_SRC;
        dma_xfr->config_low &= ~DMA_RELOAD_DST;

        /* Write the low configuration register */
        DMA_WRITE(dma_xfr->config_low,
                DMA_N_LOW_CONFIGURATION_REG_OFFSET(dma_xfr->channel));

        /* For list type transfers, set the last block LLI.CTLx.LLP_SRC_EN 
         * and LLI.CTLx.LLP_DST_EN to 0
         */
        if (list != NULL) {

            /* Disable the src and dst lists in the last (current) block */
            list->current_block->hw.control &= ~DMA_LLP_SRC_EN;
            list->current_block->hw.control &= ~DMA_LLP_DST_EN;
        }
    }

EXIT:
    RELEASE_LOCK();
    return res;
}



/*****************************************************************************
3.1.23. firecracker_dma_get_raw_status
--------------------------------------

dma_int_type_t firecracker_dma_get_raw_status(
    firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types)

3.1.23.1. Description
Get the raw status of a DMA transfer. The value returned is consistent with the 
dma_int_type_t type, see 3.2.6.
This function would typically be used to polling client code.

3.1.23.2. Parameters
dma_xfr             - The transfer handle.
int_types           - The status types to be retrieved.

3.1.23.3. Returns
A bitmask where a '1' bit indicates the status type is set.
*****************************************************************************/
dma_int_type_t firecracker_dma_get_raw_status(
    firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types)
{
    firecracker_dma_t dma = dma_xfr->dma;
    dma_int_type_t ret = 0;
    u32 hw_int_status;
    
    DB((LVL_INFO, "Enter firecracker_dma_get_raw_status\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* A macro that can be instantiated for each int_type */
#define GET_RAW_STATUS_TYPE(__type, __reg)                                  \
    if ((int_types & __type) != 0) {                                        \
        hw_int_status = DMA_READ(__reg(RAW));                               \
        if ((hw_int_status & DMA_IRQ_CHANNEL(dma_xfr->channel)) != 0) {       \
            ret |= __type;                                                  \
        }                                                                   \
    }

    GET_RAW_STATUS_TYPE(INT_BLOCK, DMA_BLOCK_COMPLETE_REG_OFFSET);
    GET_RAW_STATUS_TYPE(INT_DST_TRANSACTION, DMA_DST_TRX_COMPLETE_REG_OFFSET);
    GET_RAW_STATUS_TYPE(INT_ERROR, DMA_ERROR_REG_OFFSET);
    GET_RAW_STATUS_TYPE(INT_SRC_TRANSACTION, DMA_SRC_TRX_COMPLETE_REG_OFFSET);
    GET_RAW_STATUS_TYPE(INT_TRANSFER, DMA_TRANSFER_COMPLETE_REG_OFFSET);

    RELEASE_LOCK();
    return ret;
}



/*****************************************************************************
3.1.30. firecracker_dma_setup_sg
--------------------------------

int firecracker_dma_setup_sg(
    firecracker_dma_xfr_t dma_xfr, firecracker_dma_sg_t *sg, int src_dest)

3.1.30.1 Description
This function is used to setup the scatter or gather parameters of a DMA transfer.
If SG is not setup, the DMA transfer defaults to switching the SG capability off.

3.1.30.2. Parameters
dma_xfr             - The transfer handle.
sg                  - The SG parameters (see 3.2.3)
src_dest            - 1 for source and 0 for destination SG 


3.1.30.3. Returns
EBUSY - The DMA transfer is in progress and so cannot be modified at this time 
*****************************************************************************/
int firecracker_dma_setup_sg(
    firecracker_dma_xfr_t dma_xfr,
    firecracker_dma_sg_t *sg, int src_dst)
{
    firecracker_dma_t dma = dma_xfr->dma;
    int res = 0;

    DB((LVL_INFO, "Enter firecracker_dma_setup_sg\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();

    /* Check the transfer state */
    if (get_xfr_state(dma_xfr) != STOPPED) {
        /* Already running error */
        DB((LVL_ERR, "Transfer running\n"));
        res = EINVAL;
        goto EXIT;
    }

    /* Copy the parameters */
    if (src_dst == SRC) {
        dma_xfr->gather = *sg;
    }
    else {
        dma_xfr->scatter = *sg;
    }

EXIT:
    RELEASE_LOCK();
    return res;
}


/*****************************************************************************
3.1.31. firecracker_dma_request_transaction
-------------------------------------------

int firecracker_dma_request_transaction(
    firecracker_dma_xfr_t dma_xfr, int src_dest, unsigned int *bytes_left)

3.1.31.1 Description
This function is used to request a new transaction on the source or destination
of a transfer. The DMA endpoint (source or destination) needs to be configured
for software handshaking, otherwise this function has no effect.
For flow controlling endpoints, the type of transaction, burst, single or last is
calculated from the bytes_left parameter. For non-flow controlling endpoints, the
type of transfer is calculated by the hardware (the endpoint enters the single
transaction region automatically).

Notes:
1. When requesting transactions of a flow controlling endpoint, the bytes_left
   parameter can be set to a high value (say 1000) if it is not known how big
   the block is. This will cause burst endpoints.
2. Once the size of the block is known, the bytes_left should be set 
   appropriately. The bytes_left will be updated by the function and the new
   updated value should be used in the next call to this function.
3. Once the block size is known, client code should continue to call this 
   function with the value of bytes_left returned from the previous call and
   not revert to setting a high value (as in 1).
4. Client code should not call this function with zero bytes left if the last
   call specified many bytes left (as in 1). The reason is that the function
   needs to have prior knowledge of the end of the block so that it can set the
   last transaction flag on the last transaction.
5. The bytes_left must be divisible by the transfer width of the source and 
   destination endpoints.

Briefly (flow controlling endpoints):
Burst transactions are chosen up to the point where the last burst transaction
would spill over the end of the DMA buffer (size indicated by 'count').
Then single transactions are chosen for the remaining data.
Finally, a last transaction is chosen for the final transaction. 


3.1.31.2. Parameters
dma_xfr             - The transfer handle.
src_dest            - 1 for source and 0 for destination 
bytes_left          - The number of bytes left to transfer in the block. This is
                      used only for flow controlling endpoints and is updated
                      when the function returns. When this reached zero, no more
                      transactions should be requested for this endpoint.


3.1.31.3. Returns
EBUSY - The DMA transfer is not in progress and so transaction cannot be requested
        at this time.
EFAULT - Something went wrong and the driver did not keep track of the state of the
         hardware.
EINVAL - The transfer is not running.
*****************************************************************************/
int firecracker_dma_request_transaction(
        firecracker_dma_xfr_t dma_xfr, int src_dst,
        unsigned int *bytes_left)
{
    firecracker_dma_t dma = dma_xfr->dma;
    firecracker_dma_list_t list = dma_xfr->list;
    firecracker_dma_endpoint_t *endpoint;
    descriptor_t *curr_desc;
    int res = 0;
    int xfr_type;

    DB((LVL_INFO, "Enter firecracker_dma_request_transaction\n"));

    /* Simply lock the API against multiple threads */
    GET_LOCK();
    
    /* Check the transfer state */
    if (get_xfr_state(dma_xfr) != RUNNING) {
        /* Not running error */
        DB((LVL_NOTICE, "Transfer not running\n"));
        res = EINVAL;
        goto EXIT;
    }

    /* Check that a transfer is not already in progress */
    if (get_transaction_state(dma_xfr, src_dst) == 1) {
        /* Transaction in progress error */
        DB((LVL_NOTICE, "Transaction currently in progress\n"));
        res = EBUSY;
        goto EXIT;
    }

    /* If this is not a flow controlling endpoint, always do a 
     * burst transfer. The hardware will enter the single transaction
     * region automatically and interoperate our request appropriately
     */
    if ((src_dst == SRC && dma_xfr->src.flow_controller == 0) ||
            (src_dst == DST && dma_xfr->dst.flow_controller == 0)) {

        transfer_request(dma_xfr, src_dst, TRT_SINGLE | TRT_BURST);
    }
    else {  /* Endpoint is a flow controller */

        /* Handle non-list differently from list transfers */
        if (list == NULL) {

            /* Get the correct endpoint */
            if (src_dst == SRC) {
                endpoint = &dma_xfr->src;
            }
            else {
                endpoint = &dma_xfr->dst;
            }
        }
        else { /* Multi-block list type */

            /* Find the current running block */
            curr_desc = list->current_block;

            /* Get the correct endpoint */
            if (src_dst == SRC) {
                /* If auto-reload type, use the first element */
                if (list->src_list == 0) {
                    curr_desc = (descriptor_t *)&list->descriptor_list[0];
                }
                endpoint = &curr_desc->sw.src;
            }
            else {  /* Dst endpoint */
                /* If auto-reload type, use the first element */
                if (list->dst_list == 0) {
                    curr_desc = (descriptor_t *)&list->descriptor_list[0];
                }
                endpoint = &curr_desc->sw.dst;
            }
        }   /* multi-block list type transfer */

        /* Work out the transfer type based in the endpoint in use */
        xfr_type = transfer_type(dma_xfr, endpoint, bytes_left);

        /* do nothing and exit if we have nothing more to transfer */
        if (xfr_type != TRT_NONE) {
            /* Request the transfer */
            transfer_request(dma_xfr, src_dst, xfr_type);
        }
    }   /* endpoint is a flow controller */

EXIT:
    RELEASE_LOCK();
    return res;
}


/*****************************************************************************
3.1.24. firecracker_dma_get_dmac_handle
---------------------------------------

firecracker_dma_t firecracker_dma_get_dmac_handle(int controller)

3.1.24.1. Description
Get the handle of one of the Firecracker DMA controllers.

3.1.24.2. Parameters
controller          - The controller number, 0 or 1

3.1.24.3. Returns
A handle to the DMA driver that is to be used in other driver APIs.
*****************************************************************************/

static int find_dmac(struct device *device, void *data)
{
    struct platform_device *pdev = to_platform_device(device);
    if (!strcmp(pdev->name, CARDNAME) && (pdev->id == *(int *)data)) {
        return (int)pdev;
    }
    return 0;
}

firecracker_dma_t firecracker_dma_get_dmac_handle(int controller)
{
    struct platform_device *pdev =
        (struct platform_device *)bus_for_each_dev(
                &platform_bus_type, NULL, &controller, find_dmac);
    
    if (pdev != NULL) {
        return platform_get_drvdata(pdev);
    }
    
    return NULL;
}


/*****************************************************************************
3.1.25. firecracker_dma_dump_regs
---------------------------------

firecracker_dma_dump_regs(firecracker_dma_t dma)

3.1.25.1. Description
Use printk to output all DMA configuration registers for all channels,
including the information held in multi-block lists.

3.1.25.2. Parameters
dma_xfr             - The transfer handle.
*****************************************************************************/
void firecracker_dma_dump_regs(firecracker_dma_t dma)
{
    int i;
    u32 reg;
    u32 llp;

#define PRINT_REG(__name, __offset)                                     \
    reg = DMA_READ(__offset);                                           \
    printk("\t"__name":\t0x%08x\n", reg);

#define PRINT_BIT(__name, __mask)                                       \
    printk("\t\t"__name":\t%u\n", reg & __mask ?1:0);

#define PRINT_FIELD(__name, __mask, __shift)                            \
        printk("\t\t"__name":\t0x%08x\n", (reg & __mask) >> __shift)

    for (i = 0; i < 4; i++) {
        printk("CHANNEL %u:\n", i);

        PRINT_REG("SAR", DMA_N_SRC_ADDRESS_REG_OFFSET(i));

        PRINT_REG("DAR", DMA_N_DST_ADDRESS_REG_OFFSET(i));

        PRINT_REG("LLP", DMA_N_LINKED_LIST_POINTER_REG_OFFSET(i));
        llp = reg;
        
        PRINT_REG("CTLlow", DMA_N_CONTROL_REG_OFFSET(i));
        print_ctl_low(reg);
        
        PRINT_REG("CTLhigh", DMA_N_BLOCK_SIZE_REG_OFFSET(i));
        print_ctl_high(reg);
        
        PRINT_REG("CFGlow", DMA_N_LOW_CONFIGURATION_REG_OFFSET(i));
        PRINT_FIELD("CH_PRIOR", DMA_CH_PRIOR_MASK, DMA_CH_PRIOR_SHIFT);
        PRINT_BIT("CH_SUSP", DMA_CH_SUSP);
        PRINT_BIT("FIFO_EMPTY", DMA_FIFO_EMPTY);
        PRINT_BIT("HS_SEL_DST", DMA_HS_SEL_DST);
        PRINT_BIT("HS_SEL_SRC", DMA_HS_SEL_SRC);
        PRINT_BIT("DST_HS_POL", DMA_DST_HS_POL);
        PRINT_BIT("SRC_HS_POL", DMA_SRC_HS_POL);
        PRINT_FIELD("MAX_ABRST", DMA_MAX_ABRST_MASK, DMA_MAX_ABRST_SHIFT);
        PRINT_BIT("RELOAD_SRC", DMA_RELOAD_SRC);
        PRINT_BIT("RELOAD_DST", DMA_RELOAD_DST);
        
        PRINT_REG("CFGhigh", DMA_N_HIGH_CONFIGURATION_REG_OFFSET(i));
        PRINT_BIT("FCMODE", DMA_FCMODE);
        PRINT_BIT("FIFO_MODE", DMA_FIFO_MODE);
        PRINT_FIELD("PROTCTL", DMA_PROTCTL_MASK, DMA_PROTCTL_SHIFT);
        PRINT_FIELD("SRC_PER", DMA_SRC_PER_MASK, DMA_SRC_PER_SHIFT);
        PRINT_FIELD("DEST_PER", DMA_DST_PER_MASK, DMA_DST_PER_SHIFT);
        
        PRINT_REG("SGR", DMA_N_SRC_GATHER_REG_OFFSET(i));
        PRINT_FIELD("SGI", DMA_SG_INTERVAL_MASK, DMA_SG_INTERVAL_SHIFT);
        PRINT_FIELD("SGC", DMA_SG_COUNT_MASK, DMA_SG_COUNT_SHIFT);
        
        PRINT_REG("DSR", DMA_N_DST_SCATTER_REG_OFFSET(i));
        PRINT_FIELD("DSI", DMA_SG_INTERVAL_MASK, DMA_SG_INTERVAL_SHIFT);
        PRINT_FIELD("DSC", DMA_SG_COUNT_MASK, DMA_SG_COUNT_SHIFT);

#if 0   /* TODO - This code contains a bug, it uses the physical address
           from the llp where it should use a virtual address. */
           
#define PRINT_LLP_REG(__name, __field)                                  \
    reg = ((hw_block_descriptor_t *)llp)->__field;                      \
    printk("\tLLP."__name":\t0x%08x\n", reg);

        if (llp != 0) {
            printk("MULTI BLOCK LIST:\n");
            do {
                printk("\tBLOCK @ 0x%08x :\n", llp);

                PRINT_LLP_REG("SAR", source_addr);
                PRINT_LLP_REG("DAR", dest_addr);
                PRINT_LLP_REG("LLP", link);
                PRINT_LLP_REG("CTLlow", control);
                print_ctl_low(reg);
                PRINT_LLP_REG("CTLhigh", block_ts);
                print_ctl_high(reg);

                llp = ((hw_block_descriptor_t *)llp)->link;
            } while (llp != 0);
        }
#endif
    }

#define PRINT_CH_REG(__name, __offset)                                  \
    reg = DMA_READ(__offset);                                           \
    printk("\t"__name":\t ch0(%u) ch1(%u) ch2(%u) ch3(%u)\n", reg&DMA_CHANNEL(0)?1:0, reg&DMA_CHANNEL(1)?1:0, reg&DMA_CHANNEL(2)?1:0, reg&DMA_CHANNEL(3)?1:0);

    printk("INTERRUPTS:\n");

    PRINT_CH_REG("RawTfr", DMA_TRANSFER_COMPLETE_REG_OFFSET(RAW));
    PRINT_CH_REG("RawBlock", DMA_BLOCK_COMPLETE_REG_OFFSET(RAW));
    PRINT_CH_REG("RawSrcTran", DMA_SRC_TRX_COMPLETE_REG_OFFSET(RAW));
    PRINT_CH_REG("RawDstTran", DMA_DST_TRX_COMPLETE_REG_OFFSET(RAW));
    PRINT_CH_REG("RawErr", DMA_ERROR_REG_OFFSET(RAW));

    PRINT_CH_REG("StatusTfr", DMA_TRANSFER_COMPLETE_REG_OFFSET(STATUS));
    PRINT_CH_REG("StatusBlock", DMA_BLOCK_COMPLETE_REG_OFFSET(STATUS));
    PRINT_CH_REG("StatusSrcTran", DMA_SRC_TRX_COMPLETE_REG_OFFSET(STATUS));
    PRINT_CH_REG("StatusDstTran", DMA_DST_TRX_COMPLETE_REG_OFFSET(STATUS));
    PRINT_CH_REG("StatusErr", DMA_ERROR_REG_OFFSET(STATUS));

    PRINT_CH_REG("MaskTfr", DMA_TRANSFER_COMPLETE_REG_OFFSET(MASK));
    PRINT_CH_REG("MaskBlock", DMA_BLOCK_COMPLETE_REG_OFFSET(MASK));
    PRINT_CH_REG("MaskSrcTran", DMA_SRC_TRX_COMPLETE_REG_OFFSET(MASK));
    PRINT_CH_REG("MaskDstTran", DMA_DST_TRX_COMPLETE_REG_OFFSET(MASK));
    PRINT_CH_REG("MaskErr", DMA_ERROR_REG_OFFSET(MASK));

    PRINT_REG("StatusInt", DMA_IRQ_STATUS_REG_OFFSET);
    PRINT_BIT("TFR", DMA_STATUS_TFR);
    PRINT_BIT("BLOCK", DMA_STATUS_BLOCK);
    PRINT_BIT("SRCT", DMA_STATUS_SRCT);
    PRINT_BIT("DSTT", DMA_STATUS_DSTT);
    PRINT_BIT("ERR", DMA_STATUS_ERR);

    printk("HANDSHAKING:\n");
    PRINT_CH_REG("ReqSrcReg", DMA_SRC_TRX_REQUEST_REG_OFFSET(BURST));
    PRINT_CH_REG("ReqDstReg", DMA_DST_TRX_REQUEST_REG_OFFSET(BURST));
    PRINT_CH_REG("SglReqSrcReg", DMA_SRC_TRX_REQUEST_REG_OFFSET(SINGLE));
    PRINT_CH_REG("SglReqDstReg", DMA_DST_TRX_REQUEST_REG_OFFSET(SINGLE));
    PRINT_CH_REG("LstSrcReg", DMA_SRC_TRX_REQUEST_REG_OFFSET(LAST));
    PRINT_CH_REG("LstDstReg", DMA_DST_TRX_REQUEST_REG_OFFSET(LAST));

    printk("MISC:\n");
    PRINT_REG("DmaCfgReg", DMA_CONFIGURATION_REG_OFFSET);
    PRINT_CH_REG("ChEnReg", DMA_CHANNEL_ENABLE_REG_OFFSET);
    PRINT_REG("DmaIdReg", DMA_ID_REG_OFFSET);
    PRINT_REG("DmaTestReg", DMA_TEST_REG_OFFSET);
}


/* Name: print_ctl_low
 */
/**Purpose: Printk the bits of the low dword of the control register
 *
 * Pre-conditions: None.
 *
 * Post-conditions: Output to printk
 *
 * @param reg - The value of the low part of the control register 
 */
static void print_ctl_low(u32 reg)
{
    PRINT_BIT("INT_EN", DMA_INT_EN);
    PRINT_FIELD("DST_TR_WIDTH", DMA_DST_TR_WIDTH_MASK, DMA_DST_TR_WIDTH_SHIFT);
    PRINT_FIELD("SRC_TR_WIDTH", DMA_SRC_TR_WIDTH_MASK, DMA_SRC_TR_WIDTH_SHIFT);
    PRINT_FIELD("DINC", DMA_DINC_MASK, DMA_DINC_SHIFT);
    PRINT_FIELD("SINC", DMA_SINC_MASK, DMA_SINC_SHIFT);
    PRINT_FIELD("DST_MSIZE", DMA_DST_MSIZE_MASK, DMA_DST_MSIZE_SHIFT);
    PRINT_FIELD("SRC_MSIZE", DMA_SRC_MSIZE_MASK, DMA_SRC_MSIZE_SHIFT);
    PRINT_BIT("SRC_GATHER_EN", DMA_SRC_GATHER_EN);
    PRINT_BIT("DST_SCATTER_EN", DMA_DST_SCATTER_EN);
    PRINT_FIELD("TT_FC", DMA_TT_FC_MASK, DMA_TT_FC_SHIFT);
    PRINT_FIELD("DMS", DMA_DMS_MASK, DMA_DMS_SHIFT);
    PRINT_FIELD("SMS", DMA_SMS_MASK, DMA_SMS_SHIFT);
    PRINT_BIT("LLP_DST_EN", DMA_LLP_DST_EN);
    PRINT_BIT("LLP_SRC_EN", DMA_LLP_SRC_EN);
}


/* Name: print_ctl_high
 */
/**Purpose: Printk the bits of the high dword of the control register
 *
 * Pre-conditions: None.
 *
 * Post-conditions: Output to printk
 *
 * @param reg - The value of the low part of the control register 
 */
static void print_ctl_high(u32 reg)
{
    PRINT_FIELD("BLOCK_TS", DMA_BLOCK_TS_MASK, DMA_BLOCK_TS_SHIFT);
    PRINT_BIT("DONE", DMA_DONE);
}



/* Hardware initialisation */
/* Name: hw_initialise
 */
/**Purpose: Initialise the DMA engine and enable it.
 *
 * Pre-conditions: None.
 *
 * Post-conditions: DMA engine enabled, all channels disabled.
 *
 * @param dma - Engine context
 */
static void hw_initialise(firecracker_dma_t dma)
{
    int ch;

    /* Disable the hardware before we initialise it */
    hw_shutdown(dma);

    /* Switch all channels off and enable the DMA engine */
    for (ch = 0; ch < DMA_CHANNELS; ++ch) {
        DMA_WRITE(DMA_DISABLE_CHANNEL(ch), DMA_CHANNEL_ENABLE_REG_OFFSET);
    }

    /* Enable the engine */
    DMA_WRITE(DMA_ENABLE, DMA_CONFIGURATION_REG_OFFSET);
}


/* Name: hw_shutdown
 */
/**Purpose: Disable the DMA hardware
 *
 * Pre-conditions: None.
 *
 * Post-conditions: The DMA engine is disabled.
 *
 * @param dma - Engine context
 */
static void hw_shutdown(firecracker_dma_t dma)
{
    /* Disable the DMA */
    DMA_WRITE(~DMA_ENABLE, DMA_CONFIGURATION_REG_OFFSET);
}


/* Name: alloc_xfr
 */
/**Purpose: Allocate a transfer.
 *
 * Pre-conditions: The engine is initialised.
 *
 * Post-conditions: One of the DMA channels is marked as in use. The associated
 * transfer is initialised.
 *
 * Notes: There are 4 transfers available per engine.
 *
 * @param dma - Engine context
 *
 * @return handle to the new allocated transfer or NULL if none available. 
 *
 * @see free_xfr()
 */
static firecracker_dma_xfr_t alloc_xfr(firecracker_dma_t dma)
{
    firecracker_dma_xfr_t dma_xfr;
    int ch;

    /* Find a free channel */
    for (ch = 0; ch < DMA_CHANNELS; ++ch) {
        if (dma->channel[ch].allocated == 0) {
            break;
        }
    }

    if (ch == DMA_CHANNELS) {
        /* No channels available */
        DB((LVL_ERR, "All channels in use\n"));
        return NULL;
    }

    /* Pointer to the new transfer */
    dma_xfr = &dma->channel[ch].transfer;

    /* Reset the structure */
    memset(dma_xfr, 0, sizeof(struct firecracker_dma_xfr_tag));

    /* Set the link to the engine context */
    dma_xfr->dma = dma;

    /* Set the channel number and allocated flag */
    dma->channel[ch].allocated = 1;
    dma_xfr->channel = ch;

    return dma_xfr;
}

/* Name: free_xfr
 */
/**Purpose: Release a transfer and its DMA channel so that it 
 * becomes available to be allocated.
 *
 * Pre-conditions: The transfer is allocated from alloc_xfr.
 *
 * Post-conditions: The transfer is available for allocation. Any 
 * handshaking interfaces used by the transfer are made available.
 *
 * @param dma_xfr - The transfer handle returned from alloc_xfr 
 *
 * @see alloc_xfr()
 */
static void free_xfr(firecracker_dma_xfr_t dma_xfr)
{
    firecracker_dma_t dma = dma_xfr->dma;

    /* Reset the allocated flag */
    dma->channel[dma_xfr->channel].allocated = 0;

    /* Release any handshaking interfaces used in the transfer */
    if (dma_xfr->src_handshaking == 1) {
        dma->handshaking_inuse[dma_xfr->src_handshake.interface] = 0;
    }
    if (dma_xfr->dst_handshaking == 1) {
        dma->handshaking_inuse[dma_xfr->dst_handshake.interface] = 0;
    }
}


/* Name: alloc_list
 */
/**Purpose: Allocate the list to multi-block transfers
 *
 * Pre-conditions: None.
 *
 * Post-conditions: Heap memory allocated. The allocated list is initialised.
 *
 * Notes: The length of the list allocated is set at the calling time.
 * Memory should be allocated by free_list.
 *
 * @param count - the maximum number of elements to the list. 
 *
 * @return handle of the newly allocated list 
 *
 * @see free_list()
 */
static firecracker_dma_list_t alloc_list(
        firecracker_dma_t dma, unsigned int count)
{
    firecracker_dma_list_t list;

    /* Allocate the object */
    list = kzalloc(sizeof(struct firecracker_dma_list_tag), GFP_ATOMIC);
    if (list == NULL) {
        /* No memory, return null */
        DB((LVL_ERR, "Out of memory to allocate list\n"));
        return NULL;
    }

    /* Hold the device in the list for freeing */
    list->device = dma->device;

    /* Allocate the list. This must be on a 32-bit word boundary for the
     * hardware. We need a coherent buffer so to avoid the 
     * cache. This means that changes to the descriptors will be
     * seen immediately by the hardware.
     */
    list->descriptor_list = 
        (aligned_descriptor_t *)dma_alloc_coherent(
            list->device, sizeof(aligned_descriptor_t) * count,
            &list->dma_descriptor_list, GFP_ATOMIC);
    if (list->descriptor_list == NULL) {
        /* No memory, free the object and return null */
        DB((LVL_ERR, "Out of memory to allocate list\n"));
        kfree(list);
        return NULL;
    }

    /* Reset the list */
    memset(list->descriptor_list, 0, sizeof(aligned_descriptor_t) * count);

    return list;
}

/* Name: lookup_tt_fc
 */
/**Purpose: Given the periph_not_mem and flow controller flags for the source
 * and destination endpoints, work out the TT_FC field value.
 *
 * Pre-conditions: Memory endpoints must not be flow controllers.
 * Both source and destination cannot be flow controllers.
 *
 * Post-conditions: None.
 *
 * @param src_periph - 1 for source peripheral, 0 for memory 
 * @param src_fc - 1 for source flow controller 
 * @param dst_periph - 1 for destination peripheral, 0 for memory 
 * @param dst_fc - 1 for destination flow controller 
 *
 * @return <Comment about return var> 
 */


/* Lookup the value in the TT_FC field from the periph_not_mem and
 * flow_controller flags.
 */

/* Transfer Type/Flow Controller encoding table */
struct tt_fc_lookup_tag {
    int src_periph;      /* Source is peripheral not memory */
    int src_fc;          /* Source is a flow controller */
    int dst_periph;      /* Destination is peripheral not memory */
    int dst_fc;          /* Destination is a flow controller */
} tt_fc_lookup[] = {
    {0, 0, 0, 0},           /* M to M, DMA FC */
    {0, 0, 1, 0},           /* M to P, DMA FC */
    {1, 0, 0, 0},           /* P to M, DMA FC */
    {1, 0, 1, 0},           /* P to P, DMA FC */
    {1, 1, 0, 0},           /* P to M, SRC FC */
    {1, 1, 1, 0},           /* P to P, SRC FC */
    {0, 0, 1, 1},           /* M to P, DST FC */
    {1, 0, 1, 1}            /* P to P, DST FC */
};
#define TT_FC_LEN (sizeof(tt_fc_lookup) / sizeof(struct tt_fc_lookup_tag))
   

static u32 lookup_tt_fc(
    int src_periph,      /* Source is peripheral not memory */
    int src_fc,          /* Source is a flow controller */
    int dst_periph,      /* Destination is peripheral not memory */
    int dst_fc)          /* Destination is a flow controller */
{
    u32 tt_fc;
    struct tt_fc_lookup_tag match;

    match.src_periph = src_periph;
    match.src_fc = src_fc;
    match.dst_periph = dst_periph;
    match.dst_fc = dst_fc;

    for (tt_fc = 0; tt_fc < TT_FC_LEN; ++tt_fc)
    {
        if (memcmp(&tt_fc_lookup[tt_fc], &match, 
                    sizeof(struct tt_fc_lookup_tag)) == 0) {
            return tt_fc;
        }
    }

    return 0;
}


/* Name: build_control_register
 */
/**Purpose: Given a source and destination endpoint, build the contents of
 * the control register (CTL[0:32]).
 *
 * Pre-conditions: The source and destination endpoints are valid endpoints.
 *
 * Post-conditions: None.
 *
 * Notes: The DMA_LLP_SRC_EN and DMA_LLP_DST_EN bits are set by the 
 * build_llp_control_register function so that we do not set these bits
 * when used for the last element of a multi-block list.
 *
 * @param src - Source endpoint parameters
 * @param dst - Destination endpoint parameters
 *
 * @return The 32-bit value to be programmed into the control register.
 */
static u32 build_control_register(
        firecracker_dma_endpoint_t *src, firecracker_dma_endpoint_t *dst)
{
    u32 tt_fc;                      /* New TT_FC reg */

    /* Build up the control register */
    u32 control = 0;

    /* Enable the channel global interrupt */
    control |= DMA_INT_EN;

    /* Set the Source/Destination Transfer Width */
    control |= (((u32)src->tr_width << DMA_SRC_TR_WIDTH_SHIFT) 
            & DMA_SRC_TR_WIDTH_MASK);
    control |= (((u32)dst->tr_width << DMA_DST_TR_WIDTH_SHIFT) 
            & DMA_DST_TR_WIDTH_MASK);

    /* Set the Source/Destination address incrementing */
    control |= (((u32)src->addr_inc << DMA_SINC_SHIFT) & DMA_SINC_MASK);
    control |= (((u32)dst->addr_inc << DMA_DINC_SHIFT) & DMA_DINC_MASK);

    /* Set the Source/Destination transaction length */
    control |= ((src->msize << DMA_SRC_MSIZE_SHIFT) 
            & DMA_SRC_MSIZE_MASK);
    control |= ((dst->msize << DMA_DST_MSIZE_SHIFT) 
            & DMA_DST_MSIZE_MASK);

    /* Scatter gather enable */
    if (src->enable_sg == 1) {
        control |= DMA_SRC_GATHER_EN;
    }
    if (dst->enable_sg == 1) {
        control |= DMA_DST_SCATTER_EN;
    }

    /* Set the transfer type and flow control */
    tt_fc = lookup_tt_fc(
            src->periph_not_mem,
            src->flow_controller,
            dst->periph_not_mem,
            dst->flow_controller);
    control |= ((tt_fc << DMA_TT_FC_SHIFT) & DMA_TT_FC_MASK);

    /* Set the Source/Destination master select */
    control |= (((u32)src->ahb_master_select << DMA_SMS_SHIFT) & DMA_SMS_MASK);
    control |= (((u32)dst->ahb_master_select << DMA_DMS_SHIFT) & DMA_DMS_MASK);

    return control;
}



/* Name: build_llp_control_register
 */
/**Purpose: Given a linked list, build the contents of
 * the control register source/dest block chaining bits.
 *
 * Pre-conditions: The list ia a valid list.
 *
 * Post-conditions: None.
 *
 * @param list - Non-NULL indicates a list type transfer.
 *
 * @return The 32-bit value to be programmed into the control register.
 */
static u32 build_llp_control_register(firecracker_dma_list_t list)
{
    /* Build up the control register */
    u32 control = 0;

    /* Set the source/destination block chaining enable */
    if (list != NULL) {
        if (list->src_list == 1) {
            control |= DMA_LLP_SRC_EN;
        }
        if (list->dst_list == 1) {
            control |= DMA_LLP_DST_EN;
        }
    }

    return control;
}


/* Name: update_xfr_registers
 */
/**Purpose: DMA channel given a transfer.
 *
 * Pre-conditions: The transfer is not running. The transfer is valid.
 *
 * Post-conditions: DMA registers are written.
 *
 * Notes: For linked list transfers, the list is allocated and setup in the
 * transfer.
 * This function handles setting up all registers for the DMA transfer.
 *
 * @param dma_xfr - The transfer handle 
 */
static void update_xfr_registers(firecracker_dma_xfr_t dma_xfr)
{
    firecracker_dma_t dma = dma_xfr->dma;
    firecracker_dma_endpoint_t *src;
    firecracker_dma_endpoint_t *dst;
    firecracker_dma_list_t list = dma_xfr->list;
    unsigned int flags = dma_xfr->flags;
    unsigned int count = dma_xfr->count;
    unsigned int ch = dma_xfr->channel;
    u32 block_ts;                   /* New BLOCK_TS reg */
    u32 sg;                         /* New scatter/gather reg */
    descriptor_t *desc;

    /* If we are using a list, the src and dst endpoint information is
     * held in the list. The first element in the list sets the registers.
     * If we are not using a list, the src and dst 
     * endpoint information is held in the transfer itself.
     */
    /* Write the linked list pointer. This will be NULL if we have not
     * setup for linked multi-block.
     */
    if (list == NULL) {
        src = &dma_xfr->src;
        dst = &dma_xfr->dst;

        DMA_WRITE(0,
                DMA_N_LINKED_LIST_POINTER_REG_OFFSET(ch));
    }
    else {
        desc = (descriptor_t *)list->descriptor_list;

        src = &desc->sw.src;
        dst = &desc->sw.dst;

        /* Set the LLP register to the start of the block list */
        DMA_WRITE(list->dma_descriptor_list, 
                DMA_N_LINKED_LIST_POINTER_REG_OFFSET(ch));
    }

    /* Write the Source/Destination address registers */
    DMA_WRITE(src->dma_addr, DMA_N_SRC_ADDRESS_REG_OFFSET(ch));
    DMA_WRITE(dst->dma_addr, DMA_N_DST_ADDRESS_REG_OFFSET(ch));

    /* Write the control register */
    DMA_WRITE(
            build_control_register(src, dst) | build_llp_control_register(list),
            DMA_N_CONTROL_REG_OFFSET(ch));

    /* Write the block transfer size (top dword of control register) */
    block_ts = count / TR_BYTES(src);
    DMA_WRITE(block_ts, DMA_N_BLOCK_SIZE_REG_OFFSET(ch));

    /* Build the low configuration register */
    dma_xfr->config_low = 0;

    /* Set the channel priority */
    dma_xfr->config_low |= ((((u32)flags & CH_PRIOR_MASK) 
                << DMA_CH_PRIOR_SHIFT) & DMA_CH_PRIOR_MASK);

    /* Set software handshaking for source and destination */
    if (dma_xfr->src_handshaking == 1) {
        /* Is hardware handshaking, set polarity */
        if (dma_xfr->src_handshake.active_low == 1) {
            dma_xfr->config_low |= DMA_SRC_HS_POL;
        }
    }
    else {
        /* Set software handshaking */
        dma_xfr->config_low |= DMA_HS_SEL_SRC;
    }

    if (dma_xfr->dst_handshaking == 1) {
        /* Is hardware handshaking, set polarity */
        if (dma_xfr->dst_handshake.active_low == 1) {
            dma_xfr->config_low |= DMA_DST_HS_POL;
        }
    }
    else { 
        /* Set software handshaking */
        dma_xfr->config_low |= DMA_HS_SEL_DST;
    }

    /* Set the source and destination auto-reload */
    if (src->auto_reload == 1) {
        dma_xfr->config_low |= DMA_RELOAD_SRC;
    }
    if (dst->auto_reload == 1) {
        dma_xfr->config_low |= DMA_RELOAD_DST;
    }

    /* Write the low configuration register */
    DMA_WRITE(dma_xfr->config_low, DMA_N_LOW_CONFIGURATION_REG_OFFSET(ch));

    /* Build the high configuration register */
    dma_xfr->config_high = 0;

    /* Set the flow control mode */
    if (flags & FC_MODE) {
        dma_xfr->config_high |= DMA_FCMODE;
    }

    /* Set the FIFO mode */
    if (flags & FIFO_MODE) {
        dma_xfr->config_high |= DMA_FIFO_MODE;
    }

    /* Set the protection control */
    dma_xfr->config_high |= ((((u32)flags & PROTCTL_MASK) >> PROTCTL_SHIFT
                << DMA_PROTCTL_SHIFT) & DMA_PROTCTL_MASK);

    /* Set the src/dst handshaking peripheral */
    if (dma_xfr->src_handshaking == 1) {
        dma_xfr->config_high |= 
            (((u32)dma_xfr->src_handshake.interface << DMA_SRC_PER_SHIFT)
             & DMA_SRC_PER_MASK);
    }
    if (dma_xfr->dst_handshaking == 1) {
        dma_xfr->config_high |= 
            (((u32)dma_xfr->dst_handshake.interface << DMA_DST_PER_SHIFT)
             & DMA_DST_PER_MASK);
    }

    /* Write the high configuration register */
    DMA_WRITE(dma_xfr->config_high, DMA_N_HIGH_CONFIGURATION_REG_OFFSET(ch));

    /* Set the Scatter/Gather registers */
    sg = 0;
    sg |= ((dma_xfr->gather.count / TR_BYTES(src)
                << DMA_SG_COUNT_SHIFT) & DMA_SG_COUNT_MASK);
    sg |= ((dma_xfr->gather.interval / TR_BYTES(src)
                << DMA_SG_INTERVAL_SHIFT) & DMA_SG_INTERVAL_MASK);
    DMA_WRITE(sg, DMA_N_SRC_GATHER_REG_OFFSET(ch));

    sg = 0;
    sg |= ((dma_xfr->scatter.count / TR_BYTES(dst)
                << DMA_SG_COUNT_SHIFT) & DMA_SG_COUNT_MASK);
    sg |= ((dma_xfr->scatter.interval / TR_BYTES(dst)
                << DMA_SG_INTERVAL_SHIFT) & DMA_SG_INTERVAL_MASK);
    DMA_WRITE(sg, DMA_N_DST_SCATTER_REG_OFFSET(ch));
}


/* Name: get_xfr_state
 */
/**Purpose: Get/update the current state of a DMA transfer
 *
 * Pre-conditions: The transfer is allocated and initialised.
 *
 * Post-conditions: The transfer state is updated if the hardware indicates
 * that it is not running.
 *
 * Notes: Transfers are started in software and can stop in hardware. This
 * function gets the combined state.
 *
 * @param dma_xfr - The transfer handle 
 *
 * @return RUNNING, STOPPED or STOPPING. 
 */
static int get_xfr_state(firecracker_dma_xfr_t dma_xfr)
{
    firecracker_dma_t dma = dma_xfr->dma;
    u32 hw_state;

    /* Read the channel enables */
    hw_state = DMA_READ(DMA_CHANNEL_ENABLE_REG_OFFSET);

    /* If the channel is not enabled, it has stopped */
    if ((hw_state & DMA_CHANNEL(dma_xfr->channel)) == 0) {
        dma_xfr->state = STOPPED;
    }

    return dma_xfr->state;
}


/* Name: write_int_registers
 */
/**Purpose: Write many interrupt registers at once.
 *
 * Pre-conditions: The engine is initialised.
 *
 * Post-conditions: Several interrupt registers are updated
 *
 * @param dma - The engine context
 * @param int_types - A bit-field of the interrupt types to be affected. 
 *                      interrupt types include block, src/dst transaction,
 *                      error and transfer complete. See dma_int_type_t.
 * @param type - The property of the interrupt to update. Interrupt properties
 *                 include MASK or CLEAR.
 * @param value - The value to write to the interrupt registers.
 */
static void write_int_registers(
        firecracker_dma_t dma, dma_int_type_t int_types,
        unsigned int type, u32 value)
{
#define WRITE_INT_REG_TYPE(__type, __reg)                                   \
    if ((int_types & __type) != 0) {                                        \
        DMA_WRITE(value, __reg(type));                                      \
    }

    WRITE_INT_REG_TYPE(INT_BLOCK, DMA_BLOCK_COMPLETE_REG_OFFSET);
    WRITE_INT_REG_TYPE(INT_DST_TRANSACTION, DMA_DST_TRX_COMPLETE_REG_OFFSET);
    WRITE_INT_REG_TYPE(INT_ERROR, DMA_ERROR_REG_OFFSET);
    WRITE_INT_REG_TYPE(INT_SRC_TRANSACTION, DMA_SRC_TRX_COMPLETE_REG_OFFSET);
    WRITE_INT_REG_TYPE(INT_TRANSFER, DMA_TRANSFER_COMPLETE_REG_OFFSET);
}


/* Name: transfer_type
 */
/**Purpose: Return the transfer request signals (software handshaking) that
 * need to be asserted for the next transaction
 *
 * Pre-conditions: The transfer is running. The bytes_left is divisible by 
 * the transfer width. The endpoint uses software handshaking.
 *
 * Post-conditions: None.
 *
 * @param dma_xfr - The transfer handle 
 * @param endpoint - The endpoint for which the transfer request is to be made.
 * @param bytes_left - The number of bytes of the block left to transfer
 *
 * @return bit field with TRT_BURST, TRT_LAST, TRT_SINGLE bits set according
 * to the correct settings for the next transfer request.
 *
 * @see transfer_request()
 */
static int transfer_type(
        firecracker_dma_xfr_t dma_xfr,
        firecracker_dma_endpoint_t *endpoint,
        unsigned int *bytes_left)
{
    /* Burst transactions while there is enough data left for a whole
     * burst.
     */
    if (*bytes_left > BURST_BYTES(endpoint)) {
        /* Increment the transferred count by a burst */
        *bytes_left -= BURST_BYTES(endpoint);
        return TRT_BURST;
    }

    if (*bytes_left == BURST_BYTES(endpoint)) {
        /* Increment the transferred count by a burst */
        *bytes_left -= BURST_BYTES(endpoint);
        return TRT_BURST | TRT_LAST;
    }

    /* Single transactions up to the last */
    if (*bytes_left > TR_BYTES(endpoint)) {
        /* Increment the transferred counter by a single transaction */
        *bytes_left -= TR_BYTES(endpoint);
        /* For a single transfer, set the single and burst bits */
        return TRT_SINGLE | TRT_BURST;
    }
     
    /* Last transaction at the end. Note, it is impossible to 
     * transfer less than a transfer width.
     * This is defensive, parameter checking should ensure we always
     * have a block size that is a whole number of transfer widths.
     */
    if (*bytes_left == TR_BYTES(endpoint)) {
        /* Increment the transferred counter by a single transaction */
        *bytes_left -= TR_BYTES(endpoint);
        /* For a single transfer, set the single and burst bits */
        return TRT_SINGLE | TRT_BURST | TRT_LAST;
    }

    return TRT_NONE;
}


/* Name: get_transaction_state
 */
/**Purpose: Checks that a transfer request (software handshaking) is not in
 * progress.
 *
 * Pre-conditions: The transfer is running. 
 * The endpoint uses software handshaking.
 *
 * Post-conditions: None.
 *
 * @param dma_xfr - The transfer handle 
 * @param src_dst - The source or destination of the transfer.
 *
 * @return 1 if a transfer request is in progress and 0 otherwise.
 */
static int get_transaction_state(
        firecracker_dma_xfr_t dma_xfr, int src_dst)
{
    firecracker_dma_t dma = dma_xfr->dma;
    u32 status;

    /* Get the burst or single status bits, burst for flow controllers */
    if (src_dst == SRC) {
        if (dma_xfr->src.flow_controller == 0) {
            status = DMA_READ(DMA_SRC_TRX_REQUEST_REG_OFFSET(SINGLE));
        }
        else {
            status = DMA_READ(DMA_SRC_TRX_REQUEST_REG_OFFSET(BURST));
        }
    }
    else {
        if (dma_xfr->dst.flow_controller == 0) {
            status = DMA_READ(DMA_DST_TRX_REQUEST_REG_OFFSET(SINGLE));
        }
        else {
            status = DMA_READ(DMA_DST_TRX_REQUEST_REG_OFFSET(BURST));
        }
    }

    /* Return 1 if the burst bit is set for the channel */
    if ((status & DMA_CHANNEL(dma_xfr->channel)) != 0) {
        return 1;
    }

    return 0;
}


/* Name: transfer_request
 */
/**Purpose: Set the DMA transfer request registers to trigger a request.
 *
 * Pre-conditions: The transfer is running. The endpoint uses software 
 * handshaking. 
 *
 * Post-conditions: DMA transfer request registers updated.
 *
 * @param dma_xfr - The transfer handle 
 * @param src_dst - Selects the source or destination endpoint.
 * @param type - Bit field is the flags that need to be set (returned from 
 *               transfer_type.
 *
 * @see transfer_type()
 */
static void transfer_request(
        firecracker_dma_xfr_t dma_xfr, int src_dst, int type)
{
    firecracker_dma_t dma = dma_xfr->dma;

    /* A macro that sets the correct channel/request type bits */
#define MAKE_REQUEST(__type)                                                \
        if (src_dst == SRC) {                                              \
            DMA_WRITE(DMA_REQ_CHANNEL(dma_xfr->channel),                    \
                    DMA_SRC_TRX_REQUEST_REG_OFFSET(__type));                \
        }                                                                   \
        else {                                                              \
            DMA_WRITE(DMA_REQ_CHANNEL(dma_xfr->channel),                    \
                    DMA_DST_TRX_REQUEST_REG_OFFSET(__type));                \
        }

    if ((type & TRT_LAST) != 0) {
        MAKE_REQUEST(LAST);
    }

    if ((type & TRT_SINGLE) != 0) {
        MAKE_REQUEST(SINGLE);
    }

    if ((type & TRT_BURST) != 0) {
        MAKE_REQUEST(BURST);
    }
}


/* Name: set_auto_burst_length
 */
/**Purpose: This function deals with the 'auto' setting for the transaction
 * size. It chooses the best transaction size and updates the endpoint 
 * parameters.
 *
 * Pre-conditions: The count parameter is divisible by the transfer width.
 * The block size needs to be known and set by calling code.
 *
 * Post-conditions: The endpoint is updated with the chosen burst size.
 *
 * Notes: The transaction size that results in the fewest transactions 
 * (burst and single) to complete the block is chosen.
 *
 * @param endpoint - The endpoint parameters
 * @param count - The size of the block in bytes
 */
static void set_auto_burst_length(
        firecracker_dma_endpoint_t *endpoint, unsigned int count)
{
    int cost, cheepest_cost = 10000;    /* A big number! */
    msize_t cheepest = 0;

    /* Check that we has an auto type burst size */
    if (endpoint->msize != MS_AUTO) {
        return;
    }

    /* Try each burst size in turn and make costing. Cheapest wins. */
    for (endpoint->msize = 0; endpoint->msize < MS_AUTO; ++endpoint->msize) {

        /* If the count is smaller than the transaction size, skip 
         * the rest as only bigger ones follow
         */
        if (BURST_BYTES(endpoint) > count) {
            break;
        }

        /* The cost is the number of burst transactions plus the
         * number of single transactions needed to complete the
         * block.
         */
        cost = count / BURST_BYTES(endpoint);
        cost += (count % BURST_BYTES(endpoint)) / TR_BYTES(endpoint);

        /* If this is cheaper, remember it */
        if (cost <= cheepest_cost) {
            cheepest_cost = cost;
            cheepest = endpoint->msize;
        }
    }

    /* Set the cheapest in the endpoint */
    endpoint->msize = cheepest;
}


/* Name: param_checks
 */
/**Purpose: Do some parameter checks on endpoint parameters.
 *
 * Pre-conditions: None.
 *
 * Post-conditions: None.
 *
 * @param src - The source endpoint parameters
 * @param dst - The destination endpoint parameters
 * @param count - The block size
 *
 * @return 0 if parameter checking failed and 1 otherwise.
 */
static int param_checks(firecracker_dma_endpoint_t *src,
        firecracker_dma_endpoint_t *dst, unsigned int count)
{
    /* Check that the source transfer width is suitable for the number of
     * bytes to be transferred
     */
    if (src != NULL) {
        if ((count % TR_BYTES(src)) != 0) {
            DB((LVL_ERR, "Block size not divisible by transfer width\n"));
            return 0;
        }
    }

    /* Check that we are not setting memory as a flow controller */
    if (src != NULL) {
        if (src->periph_not_mem == 0 && src->flow_controller == 1) {
            DB((LVL_ERR, "Memory endpoint cannot be flow controller\n"));
            return 0;
        }
    }

    if (dst != NULL) {
        if (dst->periph_not_mem == 0 && dst->flow_controller == 1) {
            DB((LVL_ERR, "Memory endpoint cannot be flow controller\n"));
            return 0;
        }
    }

    /* Check that both the source and destination are not flow controllers */
    if (src != NULL && dst != NULL) {
        if (src->flow_controller == 1 && dst->flow_controller == 1) {
            DB((LVL_ERR, "Only one endpoint can be flow controller\n"));
            return 0;
        }
    }

#ifndef ENABLE_TR_WIDTH
    /* The DMA controller has fixed source and destination transfer widths.
     * Reject unsupported modes.
     */
    if (src != NULL) {
        if (src->tr_width != TR_WIDTH32) {
            DB((LVL_ERR, "DMA Controller only supports 32 bit transfer width\n"));
            return 0;
        }
    }
    if (dst != NULL) {
        if (dst->tr_width != TR_WIDTH32) {
            DB((LVL_ERR, "DMA Controller only supports 32 bit transfer width\n"));
            return 0;
        }
    }
#endif

    return 1;
}


/* Name: setup_handshaking
 */
/**Purpose: Set the handshaking parameter in the transfer endpoint
 *
 * Pre-conditions: The transfer in not running.
 *
 * Post-conditions: The transfer is updated. Hardware is not updated
 *
 * @param dma_xfr - The transfer handle 
 * @param src_handshake - The source handshaking parameters
 * @param dst_handshake - The destination handshaking parameters
 */
static void setup_handshaking(firecracker_dma_xfr_t dma_xfr,
        firecracker_dma_handshake_t *src_handshake,
        firecracker_dma_handshake_t *dst_handshake)
{
    firecracker_dma_t dma = dma_xfr->dma;

    /* Copy the parameters */
    if (src_handshake != NULL) {
        dma_xfr->src_handshake = *src_handshake;
        dma_xfr->src_handshaking = 1;

        /* Mark the interface as in use */
        dma->handshaking_inuse[src_handshake->interface] = 1;
    }

    if (dst_handshake != NULL) {
        dma_xfr->dst_handshake = *dst_handshake;
        dma_xfr->dst_handshaking = 1;

        /* Mark the interface as in use */
        dma->handshaking_inuse[dst_handshake->interface] = 1;
    }
}


/* Name: handshaking_check
 */
/**Purpose: Check the handshaking parameters. Check if any handshaking
 * interface is used on more than one endpoint.
 *
 * Pre-conditions: Engine is initialised. The handshaking interfaces in use
 * are maintained in the engine context.
 *
 * Post-conditions: None.
 *
 * @param dma - The engine context
 * @param src_handshaking - The handshaking parameters for the source or 
 *                          NULL if not using hardware handshaking 
 *                          parameters for the source.
 * @param dst_handshaking - As src_handshaking for the destination.
 *
 * @return 0 if the handshaking is in use and 1 otherwise
 */
static int handshaking_check(firecracker_dma_t dma, 
        firecracker_dma_handshake_t *src_handshaking, 
        firecracker_dma_handshake_t *dst_handshaking)
{
    if (src_handshaking != NULL) {
        if (dma->handshaking_inuse[src_handshaking->interface] == 1) {
            DB((LVL_ERR, "Source handshaking interface in use\n"));
            return 0;
        }
    }
    if (dst_handshaking != NULL) {
        if (dma->handshaking_inuse[dst_handshaking->interface] == 1) {
            DB((LVL_ERR, "Destination handshaking interface in use\n"));
            return 0;
        }
    }
    if (src_handshaking != NULL && dst_handshaking != NULL) {
        if (src_handshaking->interface == dst_handshaking->interface) {
            DB((LVL_ERR, "Same Source and Destination handshaking interface\n"));
            return 0;
        }
    }
    return 1;
}


static int dmac_drv_probe(struct platform_device *pdev)
{
    struct firecracker_dma_tag *dma;

    dma = firecracker_dma_init(pdev);
    if (!dma) {
        printk("%s: could not allocate device.\n", CARDNAME);
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, dma);

    return 0;
}


static int dmac_drv_remove(struct platform_device *pdev)
{
    struct firecracker_dma_tag *dma = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);

    firecracker_dma_exit(dma);

    return 0;
}


static struct platform_driver dmac_driver = {
    .probe      = dmac_drv_probe,
    .remove     = dmac_drv_remove,
    .driver     = {
        .name   = CARDNAME,
    }
};


static int dmac_init_module(void)
{
    int ret;

    ret = platform_driver_register(&dmac_driver);
    if (ret != 0) {
        DB((LVL_ERR, "Failed to register DMAC driver\n"));
        return ret;
    }

    printk(KERN_INFO "%s version %s loaded\n", TITLE, VERSION);

    return ret;
}


static void dmac_cleanup_module(void)
{
    platform_driver_unregister(&dmac_driver);
}


/* Module exports: */
module_init(dmac_init_module);
module_exit(dmac_cleanup_module);
EXPORT_SYMBOL(firecracker_dma_setup_direct_xfr);
EXPORT_SYMBOL(firecracker_dma_start);
EXPORT_SYMBOL(firecracker_dma_abort);
EXPORT_SYMBOL(firecracker_dma_release);
EXPORT_SYMBOL(firecracker_dma_set_debug_level);
EXPORT_SYMBOL(firecracker_dma_list_create);
EXPORT_SYMBOL(firecracker_dma_list_add);
EXPORT_SYMBOL(firecracker_dma_list_clear);
EXPORT_SYMBOL(firecracker_dma_list_destroy);
EXPORT_SYMBOL(firecracker_dma_setup_list_xfr);
EXPORT_SYMBOL(firecracker_dma_enable_int);
EXPORT_SYMBOL(firecracker_dma_disable_int);
EXPORT_SYMBOL(firecracker_dma_clear_int);
EXPORT_SYMBOL(firecracker_dma_int_get_xfr);
EXPORT_SYMBOL(firecracker_dma_handle_block_int);
EXPORT_SYMBOL(firecracker_dma_get_raw_status);
EXPORT_SYMBOL(firecracker_dma_setup_sg);
EXPORT_SYMBOL(firecracker_dma_request_transaction);
EXPORT_SYMBOL(firecracker_dma_get_dmac_handle);
EXPORT_SYMBOL(firecracker_dma_dump_regs);





