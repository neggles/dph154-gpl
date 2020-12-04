/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
 * \file pc302_dmac.c
 * \brief This module provides the interface to the low level DMA functionality.
 *
 * Copyright (c) 2009 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All inquiries to support@picochip.com
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/autoconf.h>

#include <mach/io.h>
#include <mach/hardware.h>

#include <mach/pc302_dmac.h>
#include "pc302_dma.h"

/*****************************************************************************
 * Magic numbers
 *****************************************************************************/

/*!
 * A name for this module
 */
#define TITLE "PC302 DMA Controller Driver"

/*!
 *  The name of the driver used in the system
 */
#define CARDNAME "pc302-dmac"

/*!
 * Defininition of sucess and fail
 */
#define SUCCESS               (0)
#define FAIL                  (1)

/*****************************************************************************
 * Macros
 *****************************************************************************/

/*!
 * The following macro must be set if paths not yet verified on hardware are
 * to be enabled
 */
#undef PC302_DMA_ENABLE_UNTESTED_PATHS

/*!
 * The following macro should be defined and set to the appropriate level
 * if debugging is required by default
 */
#define PC302_DMA_DEBUG_LEVEL (0)

/*!
 * Define the size of the tasklet FIFO
 */
#define PC302_DMA_FIFO_SIZE   (32)

/*!
 * Debugging macro - print warning messages
 */
#define PRINTD(dma, level, format, ...)  \
    do {\
        if (dma->lvl >= level)\
        {\
            printk("%s : " format, __FUNCTION__, ##__VA_ARGS__);\
        }\
    } while(0)

/*!
 * Print error message.
 */
#define PRINTE(format, ...) \
    do {\
        printk("%s %s %d:" format, __FILE__, __FUNCTION__, __LINE__, \
                 ##__VA_ARGS__);\
        WARN_ON(1); \
    } while(0)

/*!
 * Print error message.
 */
#define PRINTE_RATELIMIT(format, ...) \
    do {\
        if (printk_ratelimit()) \
        { \
            printk("%s %s %d:" format, __FILE__, __FUNCTION__, __LINE__, \
                 ##__VA_ARGS__);\
            WARN_ON(1); \
        } \
    } while (0)

/*!
 * Macro that can be used to determine if the DMA handle and engine are enabled.
 */
#define DMA_ENGINE_ENABLED(__dma) \
    ((__dma != NULL) && \
        ((pc302_dma_ioread32(__dma, PC302_DMA_CONFIGURATION_REG_OFFSET) & \
            PC302_DMA_ENABLE)))

/*!
 * Macro that can be used to check that a transfer handle is not NULL. 
 */
#define TRANSFER_HANDLE_ENABLED(__dma_xfr) (__dma_xfr != NULL)

/*!
 * Macro that can be used to check that a transfer list handle is not NULL.
 */
#define TRANSFER_LIST_HANDLE_ENABLED(__list) (__list != NULL)

/*!
 * Time-out for waiting for the FIFO to empty when stopping a DMA
 * (in jiffies).
 * Note: Make this small as it is used in a busy-waiting loop.
 */
#define ABORT_TIMEOUT       (HZ / 100)      /* Board timeout */

/*!
 * Calculate the number of bytes per transaction for a given endpoint.
 */
#define TR_BYTES(__E)       (1 << __E->tr_width)

/*!
 * BURST_BYTES macro calculates the number of bytes per burst transfer, 
 * given an endpoint.  msize_lookup converts from the pc302_dma_msize_t
 * enum to transfer width multiplier
 */
#define BURST_BYTES(__E) \
               (TR_BYTES(__E) * pc302_dma_msize_lookup[__E->msize])

/*!
 * Check that the number of bytes left is not divisible by the transfer width
 */
#define BURST_NOT_DIVISIBLE_BY_TRANSFER_WIDTH(__E, __COUNT) \
    ((((__COUNT)/TR_BYTES(__E)) * TR_BYTES(__E)) != __COUNT)

/*!
 * Read the specified hex digit and place the value in the 4 LSBs
 */
#define READ_HEX_DIGIT(__value, __digit) ((__value >> (4 * __digit)) & 0xF)

/*!
 * Print channel registers
 */
#define PRINT_CH_REG(__name, __offset) \
    ( { \
        reg = pc302_dma_ioread32(dma, __offset); \
        printk("    %16s: ch0(%u) ch1(%u) ch2(%u) ch3(%u)\n", __name, \
            reg&PC302_DMA_CHANNEL(0)?1:0, reg&PC302_DMA_CHANNEL(1)?1:0, \
            reg&PC302_DMA_CHANNEL(2)?1:0, reg&PC302_DMA_CHANNEL(3)?1:0); \
    } )

/*!
 * Print register
 */
#define PRINT_REG(__name, __offset) \
    ( { \
        reg = pc302_dma_ioread32(dma, __offset);  \
        printk("    %16s: 0x%08x\n", __name, reg); \
    } )

/*!
 * Print bit
 */
#define PRINT_BIT(__name, __mask) \
    ( printk("                %16s: %u\n", __name, reg & __mask ?1:0) )

/*!
 * Print field
 */
#define PRINT_FIELD(__name, __mask, __shift) \
    (printk("                %16s: 0x%08x\n", __name, \
        (reg & __mask) >> __shift))

/*!
 * Print a register from a link list descriptor
 */
#define PRINT_LLP_REG(__name, __field) \
    ( { \
        printk("                %16s: 0x%08x\n", __name, ioread32(__field)); \
    } )

/*!
 * Report an error through a U32
 */
#define ERROR_INDICATED         ((u32)(0xFFFFFFFF))

/*****************************************************************************
 * Private data structure & types
 *****************************************************************************/
/*!
 * Definition of the transfer type
 * This applies to the CTLx.TT_FC bits and is used to define the transfer type
 * and flow controller
 */
typedef enum
{
    /** Memory to memory transfer, DMA is the flow controller */
    PC302_DMA_MEM_TO_MEM_DMAC_CTRL = 0x0, 

    /** Memory to peripheral transfer, DMA is the flow controller */
    PC302_DMA_MEM_TO_PER_DMAC_CTRL = 0x1, 

    /** Peripheral to memory transfer, DMA is the flow controller */
    PC302_DMA_PER_TO_MEM_DMAC_CTRL = 0x2, 

    /** Peripheral to peripheral transfer, DMA is the flow controller */
    PC302_DMA_PER_TO_PER_DMAC_CTRL = 0x3, 

    /** Peripheral to memory transfer, Source is the flow controller */
    PC302_DMA_PER_TO_MEM_SRC_CTRL = 0x4, 

    /** Peripheral to peripheral transfer, Source is the flow controller */
    PC302_DMA_PER_TO_PER_SRC_CTRL = 0x5, 

    /** Memory to peripheral transfer, Destination is the flow controller */
    PC302_DMA_MEM_TO_PER_DST_CTRL = 0x6, 

    /** Peripheral to peripheral transfer, Destination is the flow
        controller */
    PC302_DMA_PER_TO_PER_DST_CTRL = 0x7, 

    /** Placeholder - used to determine the number of transfer types
        available */
    PC302_DMA_TRANSFER_TYPES
}
pc302_dma_tt_fc_t;
#define PC302_DMA_TRANSFER_TYPES_RANGE_FAIL (PC302_DMA_TRANSFER_TYPES)

/*!
 * Transaction type for end points
 */
typedef enum
{
    /** Undefined transaction type */
    TRT_NONE = 0x0, 

    /** Burst transaction*/
    TRT_BURST = 0x1, 

    /** Single bus width transaction */
    TRT_SINGLE = 0x2, 

    /** Last transaction in the block */
    TRT_LAST = 0x4
}
transaction_type_t;

/*!
 * Transfer status
 */
typedef enum
{
    /** Transfer stopped */
    STOPPED, 

    /** Transfer running */
    RUNNING, 

    /** Transfer stopping */
    STOPPING
} transfer_state_t;

/*!
 * This structure holds the hardware descriptor at the start (the LLP [Link
 * List Pointer] will point to this), and additional information required
 * to manage the DMA.
 */
typedef struct
{
    struct /* Hardware descriptor                                             */
    {
        u32 source_addr;             /*!< LLI.SARx                            */
        u32 dest_addr;               /*!< LLI.DARx                            */
        u32 link;                    /*!< LLI.LLPx                            */
        u32 control;                 /*!< LLI.CTLx[31:0]                      */
        u32 block_ts;                /*!< LLI.CTLx[63:32]                     */
        u32 sstat;                   /*!< LLI.SSTATx                          */
        u32 dstat;                   /*!< LLI.DSTATx                          */
    }
    hw;
    struct/* Software descriptor                                              */
    {
        pc302_dma_endpoint_t src;    /*!< The source endpoint                 */
        pc302_dma_endpoint_t dst;    /*!< The destination endpoint            */
        size_t count;                /*!< Bytes to transfer                   */
    }
    sw;
} __attribute__ ((__aligned__(4))) descriptor_t;

/*!
 * Internal state associated with a DMA transfer. Each client will construct
 * one of these structures using the API functions prior to data transfer
 */
struct pc302_dma_xfr_tag
{
    void *cookie;                       /*!< Client context                   */
    pc302_dma_list_t list;              /*!< Multi-block list or NULL if direct
                                           xfr                                */
    pc302_dma_t dma;                    /*!< Link to the engine context       */
    unsigned channel;                   /*!< The channel number used for this
                                           transfer                           */
    transfer_state_t state;             /*!< The current state of the transfer*/
    pc302_dma_endpoint_t src;           /*!< The source endpoint              */
    pc302_dma_endpoint_t dst;           /*!< The destination endpoint         */
    pc302_dma_handshake_t src_handshake;/*!< Params for src                   */
    pc302_dma_handshake_t dst_handshake;/*!< Params for dst                   */
    unsigned src_hw_handshaking_on;     /*!< Is src hardware handshaking on?  */
    unsigned dst_hw_handshaking_on;     /*!< Is dst hardware handshaking on?  */
    pc302_dma_sg_t gather;              /*!< Source gather parameters         */
    pc302_dma_sg_t scatter;             /*!< Destination scatter parameters   */
    size_t count;                       /*!< Bytes to transfer                */
    u32 flags;                          /*!< The transfer flags               */
    u32 config_low;                     /*!< Mirror of low CFGx register      */
    u32 config_high;                    /*!< Mirror of high CFGx register     */
};

/*!
 * Structure for storing the DMA internal state. One of these exists for each
 * DMA engine in the system and is shared by all clients.  Consequently client
 * processes need to be very careful in what they do with this structure and
 * only use the API functions to alter parameters for the channels they are
 * using. The only exception will be the turning off, on, up, or down of
 * debugging information via the lvl parameter (and pc302_dma_set_debug_level()
 * function)
 */
struct pc302_dma_tag
{
    void __iomem *membase;           /*!< Virtual base address of the DMA     */
    int id;                          /*!< DMA ID                              */
    struct
    {
        int ( *handler )( void *cookie, int errno );
                                     /*!< Handler function ptr for channel    */
        struct pc302_dma_xfr_tag
                           transfer; /*!< Channel Transfer parameters         */
    }
    channel[PC302_DMA_CHANNELS];
    spinlock_t spinlock;             /*!< Thread synchronisation              */
    struct device *device;           /*!< Pointer to the physical device      */
    unsigned lvl;                    /*!< The Debug level. Set globally at
                                        compile time, or defined dynamically  */
    unsigned last_int_channel;       /*!< The last interrupt channel found    */
    u32 handshaking_inuse;           /*!< Bit field indicating which H/W
                                         interfaces are in use                */
    u32 allocatedChannels;           /*!< Bit field indicating which channels
                                         are  allocated                       */
    u32 allocatedInterrupts;         /*!< Bit field indicating which channels
                                        are allocated to receive interrupts   */
    struct tasklet_struct tasklet;   /*!< Tasklet for the servicing interrupts*/
    struct
    {
        pc302_dma_xfr_t dma_xfr;
        int irq_type;
        unsigned int_channel;
    }
    fifo[PC302_DMA_FIFO_SIZE];       /*!< FIFO for queuing deferred interrupt
                                        handling                             */
    unsigned readPtr;                /*!< FIFO read pointer                  */
    unsigned writePtr;               /*!< FIFO write pointer                 */
};

/*!
 * Internal state associated with a DMA multi-block list
 */
struct pc302_dma_list_tag
{
    descriptor_t *descriptor_list;   /*!< Pointer the HW/SW list              */
    dma_addr_t dma_descriptor_list;  /*!< DMA address of the list             */
    unsigned length;                 /*!< Total list items                    */
    unsigned items;                  /*!< Current used items                  */
    descriptor_t *last_desc;         /*!< Pointer to the last in the list     */
    pc302_dma_t dma;                 /*!< Link to the engine context          */
    pc302_dma_xfr_t dma_xfr;         /*!< Transfer associated with the list   */
    unsigned is_multi_block_src;     /*!< 1 if the source is a multi-block list.
                                        0 if the transfer is multi-block with
                                        auto-reload or continuous set         */
    unsigned is_multi_block_dst;     /*!< 1 if the destination is a multi-block
                                        list type. 0 if the transfer is multi-
                                        block with auto-reload or continuous  */
    descriptor_t *current_block;     /*!< When running holds the block currently
                                        transferring                          */
    struct device *device;           /*!< Pointer to the device               */
};

/*****************************************************************************
 * Global variables & module parameters
 *****************************************************************************/

/*!
 * Burst size look up table.
 * This table contains a list of supported burst sizes for the DMA. The DMA
 * FIFOs are 32 bytes which defines the maximum size.
 */
static u32 pc302_dma_msize_lookup[PC302_DMA_MS_AUTO] =
{
    1,    /*!< 1 word transfer width (Corresponding to PC302_DMA_MS_1_TRW)    */
    4,    /*!< 4 word transfer width (Corresponding to PC302_DMA_MS_4_TRW)    */
    8,    /*!< 8 word transfer width (Corresponding to PC302_DMA_MS_8_TRW)    */
    16,   /*!< 16 word transfer width (Corresponding to PC302_DMA_MS_16_TRW)  */
    32     /*!< 32 word (maximum) transfer width (Corresponding to
              PC302_DMA_MS_32_TRW)                                            */
};

/*!
 * Bus width loop up table
 * This table is used to determine the maximum possible burst width for each
 * master on each DMA. Master 4 on DMAC2 only supports a bus widths of up to
 * 32 bits.The table takes the form:
 *            Max Bus width[DMAC number][Master number]
 */
static pc302_tr_width_t
    pc302_dma_buswidth_lookup[PC302_NUM_OF_DMAS][PC302_DMA_MASTERS] =
    {
         {
             PC302_DMA_TR_WIDTH64, /*!< DMA 1 master 1 - max width = 64 bits  */
             PC302_DMA_TR_WIDTH64, /*!< DMA 1 master 2 - max width = 64 bits  */
             PC302_DMA_TR_WIDTH64, /*!< DMA 1 master 3 - max width = 64 bits  */
             PC302_DMA_TR_WIDTH64, /*!< DMA 1 master 4 - max width = 64 bits  */
         }, 
         {
             PC302_DMA_TR_WIDTH64, /*!< DMA 2 master 1 - max width = 64 bits  */
             PC302_DMA_TR_WIDTH64, /*!< DMA 2 master 2 - max width = 64 bits  */
             PC302_DMA_TR_WIDTH64, /*!< DMA 2 master 3 - max width = 64 bits  */
             PC302_DMA_TR_WIDTH32  /*!< DMA 2 master 4 - max width = 32 bits  */
         }
     };

/*!
 * Hardware interface lookup table
 * This table is used to determine which hardware interfaces may be used with
 * each master on each DMA. This table does not provide complete checking. It
 * is impossible to determine if the end point address is consistent with the
 * hardware selected interface selected. What can be checked is that the
 * hardware interface exists and can be used with the master chosen. The table
 * takes the form:
 *    Masters supported[DMAC][H/W interface number]
 *          where Masters supported is logic OR of the following bit fields:
 *                 0001 -> PC302_DMA_MASTER1 supported
 *                 0010 -> PC302_DMA_MASTER2 supported
 *                 0100 -> PC302_DMA_MASTER3 supported
 *                 1000 -> PC302_DMA_MASTER4 supported
 */
static u32 pc302_dma_hw_if_support_lookup
    [PC302_NUM_OF_DMAS][PC302_DMA_HANDSHAKING_IFS] =
        {
            {
                0xF,   /*!< DMA 1 Interface 0 -> All masters supported        */
                0xF,   /*!< DMA 1 Interface 1 -> All masters supported        */
                0xF,   /*!< DMA 1 Interface 2 -> All masters supported        */
                0xF,   /*!< DMA 1 Interface 3 -> All masters supported        */
                0xF,   /*!< DMA 1 Interface 4 -> All masters supported        */
                0xF,   /*!< DMA 1 Interface 5 -> All masters supported        */
                0xF,   /*!< DMA 1 Interface 6 -> All masters supported        */
                0xF     /*!< DMA 1 Interface 7 -> All masters supported       */
            }, 
            {
                0x0,   /*!< DMA 2 Interface 0 -> Not connected                */
                0x0,   /*!< DMA 2 Interface 1 -> Not connected                */
                0x0,   /*!< DMA 2 Interface 2 -> Not connected                */
                0x0,   /*!< DMA 2 Interface 3 -> Not connected                */
                0x0,   /*!< DMA 2 Interface 4 -> Not connected                */
                0x8,   /*!< DMA 2 Interface 5 -> Only Master 4 supported      */
                0x8,   /*!< DMA 2 Interface 6 -> Only Master 4 supported      */
                0x8     /*!< DMA 2 Interface 7 -> Only Master 4 supported     */
            }
        };

/*!
 * Transfer Type / Flow Controller encoding table.
 * The index number corresponds to the transfer type as specified in the
 * pc302_dma_tt_fc_t enumeration. Using this table the transfer type can be
 * determined from the source and destination end point parameters.
 * Note: Some of these paths have not been tested on hardware and cannot be
 * used unless the PC302_DMA_ENABLE_UNTESTED_PATHS macro is set.
 */
struct pc302_dma_tt_fc_lookup_tag
{
    struct {
        unsigned char src_periph;   /*!< Source is peripheral not memory      */
        unsigned char src_fc;       /*!< Source is a flow controller          */
        unsigned char dst_periph;   /*!< Destination is peripheral not memory */
        unsigned char dst_fc;       /*!< Destination is a flow controller     */
    }
    path;
    unsigned tested;                /*!< Whether this path has been tested    */
}
static pc302_dma_tt_fc_lookup[PC302_DMA_TRANSFER_TYPES] =
{
    {{0, 0, 0, 0}, 1},    /*!< Memory to Memory,        DMA Flow Controller   */
    {{0, 0, 1, 0}, 1},    /*!< Memory to Peripheral,    DMA Flow Controller   */
    {{1, 0, 0, 0}, 1},    /*!< Peripheral to Memory,    DMA Flow Controller   */
    {{1, 0, 1, 0}, 0},    /*!< Peripheral to Peripheral, DMA Flow Controller  */
    {{1, 1, 0, 0}, 0},    /*!< Peripheral to Memory,    SRC Flow Controller   */
    {{1, 1, 1, 0}, 0},    /*!< Peripheral to Peripheral, SRC Flow Controller  */
    {{0, 0, 1, 1}, 0},    /*!< Memory to Peripheral,    DST Flow Controller   */
    {{1, 0, 1, 1}, 0}     /*!< Peripheral to Peripheral, DST Flow Controller  */
};

/*!
 * Keep a copy of platform handles for internal use 
 */
struct
{
    struct platform_device *dev;
    pc302_dma_t dma;
}
static dmac[PC302_NUM_OF_DMAS];
 
/*****************************************************************************
 * Private function prototypes
 *****************************************************************************/

/*!
 * \brief Initialise the DMA engine and enable it
 *
 * Post-conditions:
 *   1. DMA engine enabled
 *   2. All channels disabled.
 *
 * \param dma The DMA handle
 */
static void
pc302_dma_hw_initialise(pc302_dma_t dma);

/*!
 * \brief Disable the DMA hardware
 *
 * Post-conditions:
 *   1. DMA engines disabled.
 *
 * \param dma The DMA handle
 *
 * Notes:
 *   The DMA channel will not be disabled immediately. The DMA will continue
 *   any transfers in progress and then stop
 *
 */
static void
pc302_dma_hw_shutdown(pc302_dma_t dma);

/*!
 * \brief Allocate a transfer
 *
 * Pre-conditions:
 *   1. The engine is initialised.
 *
 * Post-conditions:
 *   1. One of the DMA channels is marked as in use
 *   2. The associated transfer is initialised.
 *
 * \param dma The DMA handle
 * \param handler The callback function pointer
 *
 * \return The allocated transfer handle, NULL on error
*/
__must_check static pc302_dma_xfr_t
pc302_dma_alloc_xfr(pc302_dma_t dma, 
                    int ( *handler )( void *cookie,
                                      int errno ) );

/*!
 * \brief Release a transfer and its DMA channel so that it becomes available to
 * be allocated.
 *
 * Pre-conditions:
 *   1. The transfer is allocated by pc302_dma_alloc_xfr.
 *
 * Post-conditions:
 *   1. The transfer is available for allocation. Any hardware interfaces
 *      that were used by the transfer are made available.
 *
 * \param dma_xfr The transfer handle
 */
static void
pc302_dma_free_xfr(pc302_dma_xfr_t dma_xfr);

/*!
 * \brief Allocate the list for multi-block transfers
 *
 * Post-conditions:
 *   1. Heap memory allocated
 *   2. The allocated list is initialised
 *
 * \param dma The DMA handle
 * \param count The maximum number of elements for the list
 *
 * \return Pointer to the newly allocated list, NULL on error
*/
__must_check static pc302_dma_list_t
pc302_dma_alloc_list(pc302_dma_t dma, 
                     size_t count);

/*!
 * \brief Calculate the Transfer Type and Flow control field values.
 *
 * Given the periph_not_mem and flow controller flags for the source and
 * destination endpoints, work out the TT_FC field value.
 *
 * Pre-conditions:
 *   1. Memory endpoints must not be flow controllers.
 *   2. Source and destination endpoints cannot be both flow controllers.
 *
 * Notes:
 *   1. If both src_fc and dst_fc are unset, it is assumed that DMA flow
 *      control is required
 *
 * \param dma The DMA handle
 * \param src_periph 1 for source is a peripheral, 
 *                   0 for source is a memory
 * \param src_fc     1 for source is the flow controller
 * \param dst_periph 1 for destination is a peripheral, 
 *                   0 for memory is a peripheral
 * \param dst_fc     1 for destination is the flow controller
 *
 * \return           Any element from pc302_dma_tt_fc_t, or 0 on no match
 */
__must_check static pc302_dma_tt_fc_t
pc302_dma_lookup_tt_fc(pc302_dma_t dma, 
                       unsigned src_periph, 
                       unsigned src_fc, 
                       unsigned dst_periph, 
                       unsigned dst_fc);

/*!
 * \brief Build the contents of the control register (CTL[0:32]) given a source
 * and destination endpoint.
 *
 * Pre-conditions:
 *   1. The source and destination endpoints are valid.
 *
 * Notes:
 *   1. The DMA_LLP_SRC_EN and DMA_LLP_DST_EN bits are set by the
 *      pc302_dma_build_llp_control_register function so that we do not set
 *      these bits when used for the last element of a multi-block list.
 *
 * \param dma DMA handle
 * \param src Source endpoint parameters
 * \param dst Destination endpoint parameters
 *
 * \return SUCCESS or -EINVAL for Invalid parameter
 */
__must_check static u32
pc302_dma_build_control_register(pc302_dma_t dma, 
                                 pc302_dma_endpoint_t *src, 
                                 pc302_dma_endpoint_t *dst);

/*!
 * \brief Build the contents of the control register source / dest block
 * chaining bits for the link list.
 *
 * Pre-conditions:
 *   1. A valid list must be provided
 *
 * \param dma DMA handle
 * \param list Pointer to a valid transfer list
 *
 * \return The 32-bit value to be programmed into the control register
 */
__must_check static u32
pc302_dma_build_llp_control_register(pc302_dma_t dma, 
                                     pc302_dma_list_t list);

/*!
 * \brief Build the contents of the control register (CTL[0:32]) from a source
 * and destination endpoint
 *
 * Pre-conditions:
 *   1. The transfer is not running.
 *   2. The transfer is valid.
 *
 * Post-conditions:
 *   1. DMA registers are written.
 *
 * Notes:
 *   1. For linked list transfers, the list is allocated and setup in the
 *      transfer.
 * \param dma_xfr   - The transfer handle
 *
 * \return SUCCESS, or -EINVAL on error
*/
__must_check static int
pc302_dma_update_xfr_registers(pc302_dma_xfr_t dma_xfr);

/*!
 * \brief Get/update the current state of a DMA transfer
 *
 * Pre-conditions:
 *   1. The transfer is allocated and initialised.
 *
 * Post-conditions:
 *   1. The transfer state is updated if the hardware indicates that it is
 *      not running
 *
 * Notes:
 *   1. Transfers are started in software and can stop in hardware. This
 *      function gets the combined state.
 *
 * \param dma_xfr   - The transfer handle
 *
 * \return            The transfer state
 */
__must_check static transfer_state_t
pc302_dma_get_xfr_state(pc302_dma_xfr_t dma_xfr);

/*!
 * \brief Configure one or more interrupt registers in one operation
 *
 * Pre-conditions:
 *   1. The engine is initialised.
 *
 * Post-conditions:
 *   1. Several interrupt registers are updated
 *
 * \param dma The engine context
 * \param irq_types A bit-field of the interrupt types to be configured.
 *                  Any combination of dma_irq_type_t:
 *                      PC302_DMA_INT_BLOCK, 
 *                      PC302_DMA_INT_DST_TRANSACTION, 
 *                      PC302_DMA_INT_ERROR, 
 *                      PC302_DMA_INT_SRC_TRANSACTION, 
 *                      PC302_DMA_INT_TRANSFER, 
 *                      PC302_DMA_INT_ALL
 * \param operation_type Operation to perform. Any of:
 *                         PC302_DMA_INTERRUPT_RAW (read status register), 
 *                         PC302_DMA_INTERRUPT_STATUS (read masked interrupts), 
 *                         PC302_DMA_INTERRUPT_MASK (set/clear mask bits), or
 *                         PC302_DMA_INTERRUPT_CLEAR (clear interrupt)
 * \param value The value to write to the interrupt registers. Any of:
 *                  PC302_DMA_IRQ_ENABLE_CHANNEL(channel number) for updating
 *                  mask registers, 
 *                  PC302_DMA_IRQ_DISABLE_CHANNEL(channel number) for updating
 *                  mask registers, or
 *                  PC302_DMA_IRQ_CHANNEL(channel number) for clearing
 *                  interrupts
 */
static void
pc302_dma_write_int_registers(pc302_dma_t dma, 
                              dma_irq_type_t irq_types, 
                              unsigned operation_type, 
                              u32 value);

/*!
 * \brief Return the transfer request signals for software handshaking that need
 * to be asserted for the next transaction
 *
 * Pre-conditions:
 *   1. The transfer is running.
 *   2. The bytes_left is divisible by the transfer width.
 *   3. The endpoint uses software handshaking.
 *
 * \param dma_xfr The transfer handle
 * \param endpoint The endpoint for which the transfer request is to be made
 * \param bytes_left The number of bytes of the block left to transfer (after
 *                   this transfer)
 *
 * \return Bit field. Any of (transaction_type_t):
 *           TRT_BURST, 
 *           TRT_LAST, 
 *           TRT_SINGLE, or
 *           TRT_NONE
 */
__must_check static transaction_type_t
pc302_dma_determine_transfer_type(pc302_dma_xfr_t dma_xfr, 
                                  pc302_dma_endpoint_t *endpoint, 
                                  unsigned *bytes_left);

/*!
 * \brief Checks that a transfer request (software handshaking) is not in
 * progress.
 *
 * Pre-conditions:
 *   1. The transfer is running.
 *   2. The endpoint uses software handshaking.
 *
 * \param dma_xfr The transfer handle
 * \param src_dst The source or destination of the transfer.
 *
 * \return FAIL if transfer request is in progress, or SUCCESS if no 
 *         request is in progress
 */
__must_check static unsigned
pc302_dma_get_transaction_state(pc302_dma_xfr_t dma_xfr, 
                                pc302_dma_endpoint_type_t src_dst);

/*!
 * \brief Set the DMA transfer request registers to trigger a request.
 *
 * Pre-conditions:
 *   1 The transfer is running.
 *   2 The endpoint uses software handshaking.
 *
 * Post-conditions:
 *   1 DMA transfer request registers updated.
 *
 * \param dma_xfr The transfer handle
 * \param src_dst Selects the source or destination endpoint.
 * \param type Transfer type to request. Any of (transaction_type_t):
 *                TRT_NONE, 
 *                TRT_BURST, 
 *                TRT_SINGLE, 
 *                TRT_LAST
 */
static void
pc302_dma_transfer_request(pc302_dma_xfr_t dma_xfr, 
                           pc302_dma_endpoint_type_t src_dst, 
                           transaction_type_t type);

/*!
 * \brief This function deals with the 'auto' setting for the transaction
 * size.
 *
 * This function chooses the best transaction size and updates the endpoint
 * parameters.
 *
 * Pre-conditions:
 *   1. The count parameter is divisible by the transfer width.
 *   2. The block size needs to be known and set by calling code.
 *
 * Post-conditions:
 *   1 The endpoint is updated with the chosen burst size.
 *
 * Notes:
 *   1. The transaction size that results in the fewest transactions (burst and
 *      single) to complete the block is chosen.
 *
 * \param dma The DMA handle
 * \param endpoint The endpoint parameters
 * \param count The size of the block in bytes
 */
static void
pc302_dma_set_auto_burst_length(pc302_dma_t dma, 
                                pc302_dma_endpoint_t *endpoint, 
                                size_t count);

/*!
 * \brief Do some parameter checks on endpoint parameters.
 *
 * \param dma The DMA handle
 * \param src The source endpoint parameters
 * \param dst The destination endpoint parameters
 * \param count The block size
 *
 * \return SUCCESS, or FAIL - Checking failed
 */
__must_check static unsigned
pc302_dma_params_valid(pc302_dma_t dma, 
                       pc302_dma_endpoint_t *src, 
                       pc302_dma_endpoint_t *dst, 
                       size_t count);

/*!
 * \brief Set the handshaking parameter in the transfer endpoint
 *
 * Pre-conditions:
 *   1. The transfer in not running.
 *
 * Post-conditions:
 *   1. The transfer is updated. Hardware is not updated
 *
 * \param dma_xfr The transfer handle
 * \param src_handshake The source handshaking parameters
 * \param dst_handshake The destination handshaking parameters
 */
static void
pc302_dma_setup_handshaking(pc302_dma_xfr_t dma_xfr, 
                            pc302_dma_handshake_t *src_handshake, 
                            pc302_dma_handshake_t *dst_handshake);

/*!
 * \brief Check the handshaking parameters. Check if any handshaking interface
 * is used on more than one endpoint.
 *
 * Pre-conditions:
 *   1. Engine is initialised.
 *   2. The handshaking interfaces in use are maintained in the engine context.
 *
 * \param dma The engine context
 * \param src The source endpoint information
 * \param dst The destination endpoint information
 * \param src_handshake The handshaking parameters for the source or NULL if
 *                        not using hardware handshaking parameters
 * \param dst_handshake The handshaking parameters for the destination or NULL
 *                        if not using hardware handshaking parameters
 *
 * \return            0 - Success, 1 - Checking failed
 */
__must_check static unsigned
pc302_dma_handshaking_params_valid(pc302_dma_t dma, 
                                   pc302_dma_endpoint_t *src, 
                                   pc302_dma_endpoint_t *dst, 
                                   pc302_dma_handshake_t *src_handshake, 
                                   pc302_dma_handshake_t *dst_handshake);

/*!
 * \brief Allows a client process to register to receive an interrupt for the
 * DMAC and DMA channel specified.
 *
 * The function will keep a count of which channels are currently registered for
 * interrupts. On the first channel registered, the DMA requests IRQ for the
 * generic ISR function pc302_dma_isr(). Thus an interrupt can only be generated
 * for the DMA when there is at least at one client process registered to
 * receive an interrupt
 *
 * Notes:
 *   1. This function performs no parameter checking. It assumes that the
 *      calling function has the right to enable interrupts on the channel and
 *      DMA specified.
 *   2. As this function only requests an IRQ when a client process requires it,
 *      it means that at system start up if IRQs are examined the DMAs will
 *      appear to have no IRQs registered.
 *
 * \param dma The DMA handle.
 * \param bitField The DMA channel(s) for the IRQ to be registered against
 *
 * \return SUCCESS, or EINVAL for Invalid parameter
 */
__must_check static int
pc302_dma_request_irq(pc302_dma_t dma, 
                      u32 bitField);

/*!
 * \brief Function that allows a client to process to de-register itself from
 * receiving an interrupt for the DMA and channel specified.
 *
 * \param dma The DMA handle
 * \param bitField The DMA channel(s) to exclude from IRQ
 *
 * \return SUCCESS, EINVAL for Invalid parameter
 */
__must_check static int
pc302_dma_free_irq(pc302_dma_t dma, 
                   u32 bitField);

/*!
 * \brief Generic ISR function called when a DMA transfer is complete.
 *
 * Hands over control to client callback function specified with the transfer
 * functions pc302_dma_setup_direct_xfr() or pc302_dma_setup_list_xfr()
 *
 * \param irq Interrupt number
 * \param dev Ptr to the client/device structure
 *
 * \return IRQ_HANDLED for Interrupt handled, IRQ_NONE for interrupt not handled
 */
__must_check static irqreturn_t
pc302_dma_isr(int irq, 
              void *dev);

/*!
 * \brief Generic DPR for processing interrupts run under a tasklet
 *
 * \param unused Unused parameter
 */
static void
pc302_dma_do_tasklet(unsigned long dmaP);

/*!
 * \brief Print the bits of the low dword of the control register
 *
 * \param reg The value of the low part of the control register
 */
static void
pc302_dma_print_ctl_low(pc302_dma_t dma, 
                        u32 reg);

/*!
 * \brief Print the bits of the high dword of the control register
 *
 * \param reg The value of the low part of the control register
 */
static void
pc302_dma_print_ctl_high(pc302_dma_t dma, 
                         u32 reg);

/*!
 * \brief Read the register specified
 *
 * \param dma The dma handle
 * \param offset_addr The offset address of the register to read
 *
 * \return The value of the register read
 */
__must_check static u32
pc302_dma_ioread32(pc302_dma_t dma, 
                   unsigned long offset_addr);

/*!
 *  \brief Write to the address specified
 *
 * \param dma The dma handle
 * \param v Value to write
 * \param offset_addr The address to write
 */
static void
pc302_dma_iowrite32(pc302_dma_t dma, 
                    u32 v, 
                    unsigned long offset_addr);

/*!
 * \brief Platform probe function
 *
 * \param pdev Platform device
 */
__must_check static int
pc302_dma_drv_probe(struct platform_device *pdev);

/*!
 * \brief Platform remove function
 *
 * \param pdev Platform device
 */
__must_check static int
pc302_dma_drv_remove(struct platform_device *pdev);

/*****************************************************************************
 * Private function declarations
 *****************************************************************************/

/*****************************************************************************
   Generic ISR function called when a DMA transfer is complete.
******************************************************************************/
static irqreturn_t
pc302_dma_isr(int irq, 
              void *dev)
{
    pc302_dma_t dma = dev;
    pc302_dma_xfr_t dma_xfr=NULL;
    dma_irq_type_t irq_type = 0;

    unsigned int_channel = 0;
    u32 hw_int_status_block = 0;
    u32 hw_int_status_dst_trx_complete = 0;
    u32 hw_int_status_error = 0;
    u32 hw_int_status_src_trx_complete = 0;
    u32 hw_int_status_transfer_complete = 0;
    unsigned loop_count = 0;
    unsigned tmp = 0;

    int ret=IRQ_NONE;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return IRQ_NONE;
    }

    /* Read all of the interrupt status registers to see which ones are set */
    hw_int_status_block = pc302_dma_ioread32(dma, 
        PC302_DMA_BLOCK_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));

    hw_int_status_dst_trx_complete = pc302_dma_ioread32(dma, 
        PC302_DMA_DST_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));

    hw_int_status_error = pc302_dma_ioread32(dma, 
        PC302_DMA_ERROR_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));

    hw_int_status_src_trx_complete = pc302_dma_ioread32(dma, 
        PC302_DMA_SRC_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));

    hw_int_status_transfer_complete = pc302_dma_ioread32(dma, 
        PC302_DMA_TXFER_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));

    /* Now find a channel number. Do a round robin search to find any
       interrupt to avoid starving any channels */
    int_channel = dma->last_int_channel+1;
    int_channel %= PC302_DMA_CHANNELS;
    for(loop_count=0;loop_count<PC302_DMA_CHANNELS;loop_count++)
    {
        irq_type = 0;

        /* The order of this set of statements determines the order in which
           interrupt types are processed */
        if (hw_int_status_transfer_complete & (1 << int_channel))
        {
            irq_type |= PC302_DMA_INT_TRANSFER;
        }
        if (hw_int_status_block & (1 << int_channel))
        {
            irq_type |= PC302_DMA_INT_BLOCK;
        }
        if (hw_int_status_dst_trx_complete & (1 << int_channel))
        {
            irq_type |= PC302_DMA_INT_DST_TRANSACTION;
        }
        if (hw_int_status_src_trx_complete & (1 << int_channel))
        {
            irq_type |= PC302_DMA_INT_SRC_TRANSACTION;
        }
        if (hw_int_status_error & (1 << int_channel))
        {
            irq_type |= PC302_DMA_INT_ERROR;
        }

        /* Break if the channel status is set and the channel is enabled.  */
        if (irq_type > 0)
        {
            /* Channel found, set the transfer handle to be returned */
            dma_xfr = &dma->channel[int_channel].transfer;
            if (dma_xfr != NULL)
            {
                /* Reset the interrupt we are servicing */
                pc302_dma_clear_int(dma_xfr, irq_type);

                /* Record the channel so that we do not service it next time
                   without looking for others first.  */
                dma->last_int_channel = int_channel;

                if (dma->channel[int_channel].handler == NULL)
                {
                    PRINTE_RATELIMIT("chan %d: IRQ=%d DMA Handler function ptr "
                            "is NULL\n", dma_xfr->channel, irq);
                }
                else
                {
                    /* call the handler function */
                    tmp = dma->writePtr +1;
                    tmp %= PC302_DMA_FIFO_SIZE;
                    if (tmp == dma->readPtr)
                    {
                        PRINTE("Cannot schedule interrupt processing for "
                            "chan %d, IRQ %d because queue is full\n",
                            dma_xfr->channel, irq);
                    }
                    else
                    { 
                        PRINTD(dma, PC302_DMA_LVL_DEBUG, 
                            "Queuing handle 0x%p dma_xfr=%p int type=%d\n", 
                            dma->channel[int_channel].handler,
                            dma_xfr, irq_type);

                        dma->fifo[dma->writePtr].int_channel = int_channel;
                        dma->fifo[dma->writePtr].dma_xfr = dma_xfr;
                        dma->fifo[dma->writePtr].irq_type = irq_type;
                        tasklet_schedule(&dma->tasklet);

                        dma->writePtr = tmp;
                    }
                }

                ret = IRQ_HANDLED;
            }
        }

        /* Move on to the next channel */
        int_channel++;
        int_channel %= PC302_DMA_CHANNELS;
    }

    if (ret == IRQ_NONE)
    {
        PRINTE_RATELIMIT("no xfr found\n");
    }

    return ret;
}

/*****************************************************************************
   Generic DPR for servicing interrupts under a tasklet
******************************************************************************/
static void
pc302_dma_do_tasklet(unsigned long dmaP)
{
    pc302_dma_t dma = (pc302_dma_t)dmaP;

    while(dma->readPtr != dma->writePtr)
    {
        PRINTD(dma, PC302_DMA_LVL_DEBUG, 
              "calling handle 0x%p dma_xfr=%p irq_type=%d\n", 
                   dma->channel[dma->fifo[dma->readPtr].int_channel].handler,
                   dma->fifo[dma->readPtr].dma_xfr,
                   dma->fifo[dma->readPtr].irq_type );
        dma->channel[dma->fifo[dma->readPtr].int_channel].handler(
                    dma->fifo[dma->readPtr].dma_xfr->cookie,
                    dma->fifo[dma->readPtr].irq_type );
   
        dma->readPtr++;
        dma->readPtr %= PC302_DMA_FIFO_SIZE;  
    }
}

/*****************************************************************************
   Read the register specified
******************************************************************************/
static u32
pc302_dma_ioread32(pc302_dma_t dma, 
                   unsigned long offset_addr)
{
    void __iomem *p = dma->membase + offset_addr;

    u32 v = ioread32(p);

    PRINTD(dma, PC302_DMA_LVL_TRACE_IO, "ioread32(0x%08lx) = 0x%08x\n", 
        offset_addr, v);

    return v;
}

/*****************************************************************************
   Write to the address specified
******************************************************************************/
static void
pc302_dma_iowrite32(pc302_dma_t dma, 
                    u32 v, 
                    unsigned long offset_addr)
{
    void __iomem *p = dma->membase + offset_addr;

    PRINTD(dma, PC302_DMA_LVL_TRACE_IO, "iowrite32(0x%08lx) = 0x%08x\n", 
        offset_addr, v);

    iowrite32(v, p);
}

/*****************************************************************************
   Initialise the DMA engine and enable it.
******************************************************************************/
static void
pc302_dma_hw_initialise(pc302_dma_t dma)
{
    unsigned ch = 0;

    if (dma == NULL)
    {
        PRINTE("NULL DMA handle\n");
        return;
    }

    PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");

    /* Disable the hardware before we initialise it */
    pc302_dma_hw_shutdown(dma);

    /* Switch all channels off and enable the DMA engine */
    for (ch = 0; ch < PC302_DMA_CHANNELS; ch++)
    {
        pc302_dma_iowrite32(dma, PC302_DMA_DISABLE_CHANNEL(ch), 
            PC302_DMA_CHANNEL_ENABLE_REG_OFFSET);
    }

    /* Enable the engine */
    pc302_dma_iowrite32(dma, PC302_DMA_ENABLE, 
        PC302_DMA_CONFIGURATION_REG_OFFSET);
}

/*****************************************************************************
   Disable the DMA hardware.
   The DMA channel will not be disabled immediately. The DMA will continue
   any transfers in progress and then stop.
******************************************************************************/
static void
pc302_dma_hw_shutdown(pc302_dma_t dma)
{
    if (dma == NULL)
    {
        PRINTE("NULL DMA handle\n");
        return;
    }

    PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");

    /* Disable the DMA */
    pc302_dma_iowrite32(dma, ~PC302_DMA_ENABLE, 
        PC302_DMA_CONFIGURATION_REG_OFFSET);
}

/*****************************************************************************
   Allocate a transfer
*****************************************************************************/
static pc302_dma_xfr_t
pc302_dma_alloc_xfr(pc302_dma_t dma, 
                    int ( *handler )( void *cookie,
                                      int errno ))
{
    pc302_dma_xfr_t dma_xfr=NULL;
    unsigned ch=0;

    /* Check that the engine is initialised */
    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return NULL;
    }


    /* Find a free channel */
    for (ch = 0; ch < PC302_DMA_CHANNELS; ch++)
    {
        if ((dma->allocatedChannels & (1 << ch)) == 0)
        {
            break;
        }
    }

    if (ch == PC302_DMA_CHANNELS)
    {
        PRINTD(dma, PC302_DMA_LVL_WARNING, "All channels in use\n");
        return NULL;
    }

    /* Pointer to the new transfer */
    dma_xfr = &dma->channel[ch].transfer;

    /* Reset the structure */
    memset(dma_xfr, 0, sizeof(struct pc302_dma_xfr_tag));

    /* Set the link to the engine context */
    dma_xfr->dma = dma;

    /* Set the channel number and allocated flag */
    dma->allocatedChannels |= (1 << ch);
    dma->channel[ch].handler = handler;
    dma_xfr->channel = ch;

    PRINTD(dma, PC302_DMA_LVL_DEBUG, "xfr=%p\n", dma_xfr);
    return dma_xfr;
}

/*****************************************************************************
   Release a transfer and its DMA channel so that it becomes
   available to be allocated.
*****************************************************************************/
static void
pc302_dma_free_xfr(pc302_dma_xfr_t dma_xfr)
{
    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return;
    }

    /* Reset the allocated flag */
    dma_xfr->dma->allocatedChannels &= ~((unsigned)(1 << dma_xfr->channel));
    dma_xfr->dma->channel[dma_xfr->channel].handler = NULL;

    /* Release any handshaking interfaces used in the transfer */
    if (dma_xfr->src_hw_handshaking_on)
    {
        dma_xfr->dma->handshaking_inuse &=
             ~((unsigned)(1 << dma_xfr->src_handshake.hwInterface));
    }

    if (dma_xfr->dst_hw_handshaking_on)
    {
        dma_xfr->dma->handshaking_inuse &=
             ~((unsigned)(1 << dma_xfr->dst_handshake.hwInterface));
    }
}

/*****************************************************************************
   Allocate the list for multi-block transfers
*****************************************************************************/
static pc302_dma_list_t
pc302_dma_alloc_list(pc302_dma_t dma, 
                     size_t count)
{
    pc302_dma_list_t list=NULL;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return NULL;
    }

    /* Allocate the object */
    list = kzalloc(sizeof(struct pc302_dma_list_tag), GFP_KERNEL);
    if (list == NULL)
    {
        /* No memory, return null */
        PRINTE("Out of memory to allocate list\n");
        return NULL;
    }

    /* Hold the device in the list for freeing */
    list->device = dma->device;

    /* Allocate the list. This must be on a 32-bit word boundary for the
       hardware, and must be cache less, coherent buffer */
    list->descriptor_list = dma_alloc_coherent(list->device, 
            (sizeof(descriptor_t) * count), 
            &list->dma_descriptor_list, GFP_KERNEL);
    if (list->descriptor_list == NULL)
    {
        /* No memory, free the object and return null */
        PRINTE("Out of memory to allocate list\n");
        kfree(list);
        return NULL;
    }

    /* Reset the list */
    memset(list->descriptor_list, 0, (sizeof(descriptor_t) * count));

    PRINTD(dma, PC302_DMA_LVL_DEBUG, "list=0x%p\n", list);
    return list;
}

/*****************************************************************************
   Given the periph_not_mem and flow controller flags for the source and
   destination endpoints, work out the TT_FC (Transfer Type / Flow Control)
   field value.
*****************************************************************************/
static pc302_dma_tt_fc_t
pc302_dma_lookup_tt_fc(pc302_dma_t dma, 
                       unsigned src_periph, 
                       unsigned src_fc, 
                       unsigned dst_periph, 
                       unsigned dst_fc)
{
    pc302_dma_tt_fc_t tt_fc=0;
    struct pc302_dma_tt_fc_lookup_tag match =
                                 {{src_periph, src_fc, dst_periph, dst_fc}};

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return PC302_DMA_TRANSFER_TYPES_RANGE_FAIL;
    }

    for (tt_fc = 0; tt_fc < PC302_DMA_TRANSFER_TYPES; tt_fc++)
    {
        if (memcmp(&pc302_dma_tt_fc_lookup[tt_fc].path, &match.path, 
            (sizeof(unsigned char)*4)) == 0)
        {
#ifndef PC302_DMA_ENABLE_UNTESTED_PATHS
            if (pc302_dma_tt_fc_lookup[tt_fc].tested == 0)
            {
                PRINTE("An untested on\nhardware path has "
                    "been selected. Recompile with "
                    "PC302_DMA_ENABLE_UNTESTED_PATHS defined\n"
                    "SRC_PER=%d SRC_FC=%d DST_PER=%d DST_FC=%d\n", 
                     src_periph, src_fc, dst_periph, dst_fc);
                return PC302_DMA_TRANSFER_TYPES_RANGE_FAIL;
            }
#endif

            PRINTD(dma, PC302_DMA_LVL_DEBUG, "tt_fc=%d\n", tt_fc);
            return tt_fc;
        }
    }

    PRINTD(dma, PC302_DMA_LVL_WARNING, " Invalid parameters SRC_PER=%d "
        "SRC_FC=%d DST_PER=%d DST_FC=%d. Assuming mem_to_mem t/f using "
        "DMA FC\n", src_periph, src_fc, dst_periph, dst_fc);

    return PC302_DMA_TRANSFER_TYPES_RANGE_FAIL;
}

/*****************************************************************************
   Given a source and destination endpoint, build the contents of
   the control register (CTL[0:32]).
*****************************************************************************/
static u32
pc302_dma_build_control_register(pc302_dma_t dma, 
                                 pc302_dma_endpoint_t *src, 
                                 pc302_dma_endpoint_t *dst)
{
    u32 tt_fc=0;                      /* New TT_FC reg */
    u32 control = 0;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return ERROR_INDICATED;
    }

    if ((src == NULL) || (dst == NULL))
    {
        PRINTE("NULL SRC and / or DST ptrs are null (0x%p, 0x%p)\n", 
            src, dst);
        return ERROR_INDICATED;
    }

    /* Enable the channel global interrupt */
    control |= PC302_DMA_INT_EN;

    /* Set the Source/Destination Transfer Width */
    control |= (((u32)src->tr_width << PC302_DMA_SRC_TR_WIDTH_SHIFT)
            & PC302_DMA_SRC_TR_WIDTH_MASK);
    control |= (((u32)dst->tr_width << PC302_DMA_DST_TR_WIDTH_SHIFT)
            & PC302_DMA_DST_TR_WIDTH_MASK);

    /* Set the Source/Destination address incrementing */
    control |=
        (((u32)src->addr_inc << PC302_DMA_SINC_SHIFT) & PC302_DMA_SINC_MASK);
    control |=
        (((u32)dst->addr_inc << PC302_DMA_DINC_SHIFT) & PC302_DMA_DINC_MASK);

    /* Set the Source/Destination transaction length */
    control |= ((src->msize << PC302_DMA_SRC_MSIZE_SHIFT)
            & PC302_DMA_SRC_MSIZE_MASK);
    control |= ((dst->msize << PC302_DMA_DST_MSIZE_SHIFT)
            & PC302_DMA_DST_MSIZE_MASK);

    /* Scatter gather enable */
    if (src->enable_sg)
    {
        control |= PC302_DMA_SRC_GATHER_EN;
    }
    if (dst->enable_sg)
    {
        control |= PC302_DMA_DST_SCATTER_EN;
    }

    /* Set the transfer type and flow control */
    tt_fc = pc302_dma_lookup_tt_fc(
                dma, 
                src->periph_not_mem, 
                src->flow_controller, 
                dst->periph_not_mem, 
                dst->flow_controller);
    if (tt_fc == PC302_DMA_TRANSFER_TYPES_RANGE_FAIL)
    {
        /* No match found - return error status */
        return ERROR_INDICATED;
    }

    control |= ((tt_fc << PC302_DMA_TT_FC_SHIFT) & PC302_DMA_TT_FC_MASK);

    /* Set the Source/Destination master select */
    control |= (((u32)src->master << PC302_DMA_SMS_SHIFT) & PC302_DMA_SMS_MASK);
    control |= (((u32)dst->master << PC302_DMA_DMS_SHIFT) & PC302_DMA_DMS_MASK);

    PRINTD(dma, PC302_DMA_LVL_DEBUG, "Exit, control=0x%08x\n", control);

    return control;
}

/*****************************************************************************
   Given a linked list, build the contents of the control register
   source / dest block chaining bits.
*****************************************************************************/
static u32
pc302_dma_build_llp_control_register(pc302_dma_t dma, 
                                     pc302_dma_list_t list)
{
    u32 control = 0;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return ERROR_INDICATED;
    }

    /* Set the source/destination block chaining enable */
    if (list != NULL)
    { /* Multi-block and no auto-reload */
        if (list->is_multi_block_src)
        {
            control |= PC302_DMA_LLP_SRC_EN;
        }
        if (list->is_multi_block_dst)
        {
            control |= PC302_DMA_LLP_DST_EN;
        }
    }
    else
    {
        PRINTE("NULL block list ptr\n");
        control = ERROR_INDICATED;
    }

    PRINTD(dma, PC302_DMA_LVL_DEBUG, "Exit, control=0x%08x\n", control);

    return control;
}

/*****************************************************************************
   Given a source and destination endpoint, build the contents of
   the control register (CTL[0:32]).
*****************************************************************************/
static int
pc302_dma_update_xfr_registers(pc302_dma_xfr_t dma_xfr)
{
    pc302_dma_endpoint_t *src=NULL;
    pc302_dma_endpoint_t *dst=NULL;
    u32 block_ts=0;                   /* New BLOCK_TS reg */
    u32 sg=0;                         /* New scatter/gather reg */
    u32 control=0;                    /* New control reg */
    descriptor_t *desc=NULL;
    int ret = SUCCESS;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return -EINVAL;
    }

    /* Check / update the transfer status */
    if (pc302_dma_get_xfr_state(dma_xfr) == RUNNING)
    {
        PRINTE("Transfer is running\n");
        return -EINVAL;
    }

    /* If we are using a list, the src and dst endpoint information is
       held in the list. The first element in the list sets the registers.
       If we are not using a list, the src and dst
       endpoint information is held in the transfer itself.  */

    /* Write the linked list pointer. This will be NULL if we have not
     * setup for linked multi-block.  */
    if (dma_xfr->list == NULL)
    {
        src = &dma_xfr->src; /* src and dst endpoint info must be in the */
        dst = &dma_xfr->dst; /* transfer */

        pc302_dma_iowrite32(dma_xfr->dma, 0, 
            PC302_DMA_N_LLP_REG_OFFSET(dma_xfr->channel));
    }
    else
    {
        desc = dma_xfr->list->descriptor_list;

        src = &desc->sw.src; /* src and dst endpoint info must be in the */
        dst = &desc->sw.dst; /* first element of the list */

        /* Set the LLP register to the start of the block list */
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, "Head LLP reg = 0x%p\n", 
            (void *)dma_xfr->list->dma_descriptor_list);
        pc302_dma_iowrite32(dma_xfr->dma, dma_xfr->list->dma_descriptor_list, 
                PC302_DMA_N_LLP_REG_OFFSET(dma_xfr->channel));
    }

    /* Write the Source/Destination address registers */
    PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, 
        "src addr = 0x%08x, dst addr = 0x08%x\n", src->dma_addr, dst->dma_addr);
    pc302_dma_iowrite32(dma_xfr->dma, src->dma_addr, 
        PC302_DMA_N_SRC_ADDR_REG_OFFSET(dma_xfr->channel));
    pc302_dma_iowrite32(dma_xfr->dma, dst->dma_addr, 
        PC302_DMA_N_DST_ADDR_REG_OFFSET(dma_xfr->channel));

    /* Write the control register
       Debug prints provided by pc302_dma_build_llp_control_register &
                               pc302_dma_build_llp_control_register     */
    control = pc302_dma_build_control_register(dma_xfr->dma, src, dst);
    if (control == ERROR_INDICATED)
    {
        return -EINVAL; /* Error message printed in function */
    }

    if (dma_xfr->list)
    {
        control |=
            pc302_dma_build_llp_control_register(dma_xfr->dma, dma_xfr->list);
        if (control == ERROR_INDICATED)
        {
            return -EINVAL; /* Error message printed in function */
        }
    }

    pc302_dma_iowrite32(dma_xfr->dma, control, 
        PC302_DMA_N_CTRL_REG_OFFSET(dma_xfr->channel));

    /* Write the block transfer size (top dword of control register) */
    block_ts = dma_xfr->count / TR_BYTES(src);
    PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, "block_ts = 0x%08x\n", block_ts);
    pc302_dma_iowrite32(dma_xfr->dma, block_ts, 
        PC302_DMA_N_BLOCK_SIZE_REG_OFFSET(dma_xfr->channel));

    /* Build the low configuration register */
    dma_xfr->config_low = 0;

    /* Set the channel priority */
    dma_xfr->config_low |= ((READ_HEX_DIGIT((u32)dma_xfr->flags, 0) <<
        PC302_DMA_CH_PRIOR_SHIFT) & PC302_DMA_CH_PRIOR_MASK);

    /* Set software handshaking for source and destination */
    if (dma_xfr->src_hw_handshaking_on)
    {
        /* Is hardware handshaking, set polarity */
        if (dma_xfr->src_handshake.active_low)
        {
            dma_xfr->config_low |= PC302_DMA_SRC_HS_POL;
        }
    }
    else
    {
        /* Set software handshaking */
        dma_xfr->config_low |= PC302_DMA_HS_SEL_SRC;
    }

    if (dma_xfr->dst_hw_handshaking_on)
    {
        /* Is hardware handshaking, set polarity */
        if (dma_xfr->dst_handshake.active_low)
        {
            dma_xfr->config_low |= PC302_DMA_DST_HS_POL;
        }
    }
    else
    {
        /* Set software handshaking */
        dma_xfr->config_low |= PC302_DMA_HS_SEL_DST;
    }

    /* Set the source and destination auto-reload */
    if (src->auto_reload)
    {
        dma_xfr->config_low |= PC302_DMA_RELOAD_SRC;
    }
    if (dst->auto_reload)
    {
        dma_xfr->config_low |= PC302_DMA_RELOAD_DST;
    }

    /* Write the low configuration register */
    PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, "CTRLx (Low) = 0x%08x\n", 
        dma_xfr->config_low);
    pc302_dma_iowrite32(dma_xfr->dma, dma_xfr->config_low, 
        PC302_DMA_N_LOW_CONFIG_REG_OFFSET(dma_xfr->channel));

    /* Build the high configuration register */
    dma_xfr->config_high = 0;

    /* Set the flow control mode */
    if (dma_xfr->flags & PC302_DMA_DST_WAIT)
    {
        dma_xfr->config_high |= PC302_DMA_FC_MODE;
    }

    /* Set the FIFO mode */
    if (dma_xfr->flags & PC302_DMA_FIFO_WAIT)
    {
        dma_xfr->config_high |= PC302_DMA_FIFO_MODE;
    }

    /* Set the protection control */
    dma_xfr->config_high |= ((READ_HEX_DIGIT((u32)dma_xfr->flags, 1)
          << PC302_DMA_PROTCTL_SHIFT) & PC302_DMA_PROTCTL_MASK);

    /* Set the src/dst handshaking peripheral */
    if (dma_xfr->src_hw_handshaking_on)
    {
        dma_xfr->config_high |=
            (((u32)dma_xfr->src_handshake.hwInterface <<
            PC302_DMA_SRC_PER_SHIFT) & PC302_DMA_SRC_PER_MASK);
    }
    if (dma_xfr->dst_hw_handshaking_on)
    {
        dma_xfr->config_high |=
            (((u32)dma_xfr->dst_handshake.hwInterface <<
            PC302_DMA_DST_PER_SHIFT) & PC302_DMA_DST_PER_MASK);
    }

    /* Write the high configuration register */
    PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, "CTRLx (High) = 0x%08x\n", 
        dma_xfr->config_high);
    pc302_dma_iowrite32(dma_xfr->dma, dma_xfr->config_high, 
        PC302_DMA_N_HIGH_CONFIG_REG_OFFSET(dma_xfr->channel));

    /* Set the Scatter/Gather registers */
    sg = 0;
    sg |= ((dma_xfr->gather.count / TR_BYTES(src)
                << PC302_DMA_SG_COUNT_SHIFT) & PC302_DMA_SG_COUNT_MASK);
    sg |= ((dma_xfr->gather.interval / TR_BYTES(src)
                << PC302_DMA_SG_INTERVAL_SHIFT) & PC302_DMA_SG_INTERVAL_MASK);
    pc302_dma_iowrite32(dma_xfr->dma, sg, 
        PC302_DMA_N_SRC_GATHER_REG_OFFSET(dma_xfr->channel));

    sg = 0;
    sg |= ((dma_xfr->scatter.count / TR_BYTES(dst)
                << PC302_DMA_SG_COUNT_SHIFT) & PC302_DMA_SG_COUNT_MASK);
    sg |= ((dma_xfr->scatter.interval / TR_BYTES(dst)
                << PC302_DMA_SG_INTERVAL_SHIFT) & PC302_DMA_SG_INTERVAL_MASK);
    PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, "S/G = 0x%08x\n", sg);
    pc302_dma_iowrite32(dma_xfr->dma, sg, 
        PC302_DMA_N_DST_SCATTER_REG_OFFSET(dma_xfr->channel));

   return ret;
}

/*****************************************************************************
   Get/update the current state of a DMA transfer
*****************************************************************************/
static transfer_state_t
pc302_dma_get_xfr_state(pc302_dma_xfr_t dma_xfr)
{
    u32 hw_state=0;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return STOPPED;
    }

    /* Read the channel enables */
    hw_state = pc302_dma_ioread32(dma_xfr->dma, 
        PC302_DMA_CHANNEL_ENABLE_REG_OFFSET);

    /* If the channel is not enabled, it has stopped - update the state*/
    if ((hw_state & PC302_DMA_CHANNEL(dma_xfr->channel)) == 0)
    {
        dma_xfr->state = STOPPED;
    }

    PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, "state = %d\n", dma_xfr->state);

    return dma_xfr->state;
}

/*****************************************************************************
   Write many interrupt registers at once.
*****************************************************************************/
static void
pc302_dma_write_int_registers(pc302_dma_t dma, 
                              dma_irq_type_t irq_types, 
                              unsigned operation_type, 
                              u32 value)
{
    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter (dma=0x%p, int types="
            "0x%08x operation type=0x%08x value=0x%08x)\n", 
            (void *)dma, irq_types, operation_type, value);
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return;
    }

    if (irq_types & PC302_DMA_INT_BLOCK)
    {
        pc302_dma_iowrite32(dma, value, 
            PC302_DMA_BLOCK_COMPLETE_REG_OFFSET(operation_type));
    }

    if (irq_types & PC302_DMA_INT_DST_TRANSACTION)
    {
        pc302_dma_iowrite32(dma, value, 
            PC302_DMA_DST_TRX_COMPLETE_REG_OFFSET(operation_type));
    }

    if (irq_types & PC302_DMA_INT_ERROR)
    {
        pc302_dma_iowrite32(dma, value, 
            PC302_DMA_ERROR_REG_OFFSET(operation_type));
    }

    if (irq_types & PC302_DMA_INT_SRC_TRANSACTION)
    {
        pc302_dma_iowrite32(dma, value, 
            PC302_DMA_SRC_TRX_COMPLETE_REG_OFFSET(operation_type));
    }

    if (irq_types & PC302_DMA_INT_TRANSFER)
    {
        pc302_dma_iowrite32(dma, value, 
            PC302_DMA_TXFER_COMPLETE_REG_OFFSET(operation_type));
    }
}

/*****************************************************************************
   Return the transfer request signals for software handshaking
   that need to be asserted for the next transaction
*****************************************************************************/
static transaction_type_t
pc302_dma_determine_transfer_type(pc302_dma_xfr_t dma_xfr, 
                                  pc302_dma_endpoint_t *endpoint, 
                                  unsigned *bytes_left)
{
    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_TRACE, "Enter: (xfr=0x%p "
            "endpoint=0x%p, bytes left = %d)\n", 
            (void *)dma_xfr, (void *)endpoint, *bytes_left);
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return TRT_NONE;
    }

    if (dma_xfr->state != RUNNING)
    {
        PRINTE("Transfer not running\n");
        return TRT_NONE;
    }

    if (endpoint == NULL)
    {
        PRINTE("NULL endpoint handle\n");
        return TRT_NONE;
    }

    /* Check that the endpoint is a flow controller. (Need also to check that
       the endpoint is not using H/W flow control but the information is not
       available in this function) */
    if (!endpoint->flow_controller)
    {
        PRINTE("End point is not a flow controller\n");
        return TRT_NONE;
    }

    /* Check that the number of bytes left is divisible by the transfer width */
    if (BURST_NOT_DIVISIBLE_BY_TRANSFER_WIDTH(endpoint, *bytes_left))
    {
        PRINTE("Bytes left (%d) is not divisible by the transfer width (%d)\n", 
            *bytes_left, TR_BYTES(endpoint));
        return TRT_NONE;
    }

    /* Select the most desirable transaction type - starting with 
       burst transfers */
    if (*bytes_left > BURST_BYTES(endpoint))
    {
        *bytes_left -= BURST_BYTES(endpoint);
        return TRT_BURST; /* Burst in the middle of a transfer */
    }

    if (*bytes_left == BURST_BYTES(endpoint))
    {
        *bytes_left -= BURST_BYTES(endpoint);
        return TRT_BURST | TRT_LAST; /* Burst at the end of a transfer */
    }

    /* Single transactions up to the last */
    if (*bytes_left > TR_BYTES(endpoint))
    {
        *bytes_left -= TR_BYTES(endpoint);
        /* For a single transfer, set the single and burst bits */
        return TRT_SINGLE | TRT_BURST;
    }

    if (*bytes_left == TR_BYTES(endpoint))
    {
        *bytes_left -= TR_BYTES(endpoint);
        /* For a single transfer, set the single and burst bits */
        return TRT_SINGLE | TRT_BURST | TRT_LAST;
    }

    /* No more left to transfer */
    return TRT_NONE;
}

/*****************************************************************************
   Checks that a transfer request (software handshaking) is not in progress.
*****************************************************************************/
static unsigned
pc302_dma_get_transaction_state(pc302_dma_xfr_t dma_xfr, 
                                pc302_dma_endpoint_type_t src_dst)
{
    u32 status=0;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return FAIL;
    }

    if (dma_xfr->state != RUNNING)
    {
        PRINTE("Transfer not running\n");
        return FAIL;
    }

    /* Need to check that the endpoint is using S/W flow control but this
       information is not available in this function */

    /* Get the burst or single status bits, burst for flow controllers */
    if (src_dst == PC302_DMA_SRC)
    {
        if (dma_xfr->src.flow_controller == 0)
        {
            status = pc302_dma_ioread32(dma_xfr->dma, 
                PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(
                    PC302_DMA_SINGLE_TRANSFER));
        }
        else
        {
            status = pc302_dma_ioread32(dma_xfr->dma, 
                PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(PC302_DMA_BURST_TRANSFER));
        }
    }
    else
    {
        if (dma_xfr->dst.flow_controller == 0)
        {
            status = pc302_dma_ioread32(dma_xfr->dma, 
                PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(
                    PC302_DMA_SINGLE_TRANSFER));
        }
        else
        {
            status = pc302_dma_ioread32(dma_xfr->dma, 
                PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(PC302_DMA_BURST_TRANSFER));
        }
    }

    /* Return fail if the burst bit is set for the channel */
    if (status & PC302_DMA_CHANNEL(dma_xfr->channel))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, 
          "S/W handshaking in progress\n");
        return FAIL;
    }
    else
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_DEBUG, 
          "S/W handshaking not in progress\n");
        return SUCCESS;
    }
}

/*****************************************************************************
   Set the DMA transfer request registers to trigger a request.
*****************************************************************************/
static void
pc302_dma_transfer_request(pc302_dma_xfr_t dma_xfr, 
                           pc302_dma_endpoint_type_t src_dst, 
                           transaction_type_t type)
{
    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return;
    }

    if (dma_xfr->state != RUNNING)
    {
        PRINTE("Transfer not running\n");
        return;
    }

    if ((type & TRT_LAST) != 0)
    {
        if (src_dst == PC302_DMA_SRC)
        {
            pc302_dma_iowrite32(dma_xfr->dma, 
                PC302_DMA_REQ_CHANNEL(dma_xfr->channel), 
                PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(PC302_DMA_LAST_TRANSFER));
        }
        else
        {
            pc302_dma_iowrite32(dma_xfr->dma, 
                PC302_DMA_REQ_CHANNEL(dma_xfr->channel), 
                PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(PC302_DMA_LAST_TRANSFER));
        }
    }

    if (type & TRT_SINGLE)
    {
        if (src_dst == PC302_DMA_SRC)
        {
            pc302_dma_iowrite32(dma_xfr->dma, 
                PC302_DMA_REQ_CHANNEL(dma_xfr->channel), 
                PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(
                    PC302_DMA_SINGLE_TRANSFER));
        }
        else
        {
            pc302_dma_iowrite32(dma_xfr->dma, 
                PC302_DMA_REQ_CHANNEL(dma_xfr->channel), 
                PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(
                    PC302_DMA_SINGLE_TRANSFER));
        }
    }

    if (type & TRT_BURST)
    {
        if (src_dst == PC302_DMA_SRC)
        {
            pc302_dma_iowrite32(dma_xfr->dma, 
                PC302_DMA_REQ_CHANNEL(dma_xfr->channel), 
                PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(PC302_DMA_BURST_TRANSFER));
        }
        else
        {
            pc302_dma_iowrite32(dma_xfr->dma, 
                PC302_DMA_REQ_CHANNEL(dma_xfr->channel), 
                PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(PC302_DMA_BURST_TRANSFER));
        }
    }
}

/*****************************************************************************
   This function deals with the 'auto' setting for the transaction size. It
   chooses the best transaction size and updates the endpoint parameters.
*****************************************************************************/
static
void pc302_dma_set_auto_burst_length(pc302_dma_t dma, 
                                     pc302_dma_endpoint_t *endpoint, 
                                     size_t count)
{
    unsigned cost=0;
    unsigned cheapest_cost = 10000;    /* A big number! */
    pc302_msize_t cheapest = 0;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return;
    }

    if (endpoint == NULL)
    {
        PRINTE("NULL endpoint handle\n");
        return;
    }

    /* Check that we has an auto type burst size */
    if (endpoint->msize != PC302_DMA_MS_AUTO)
    {
        /* Client has selected the burst size - this function not required */
        return;
    }

    /* Check that the number of bytes left is divisible by the transfer width */
    if (BURST_NOT_DIVISIBLE_BY_TRANSFER_WIDTH(endpoint, count))
    {
        PRINTE("Burst size (%d) is not divisible by the transfer width (%d)\n", 
            count, TR_BYTES(endpoint));
        return;
    }

    /* Try each burst size in turn and make costing. Cheapest wins. */
    for (endpoint->msize = 0; endpoint->msize < PC302_DMA_MS_AUTO;
          endpoint->msize++)
    {
        /* If the count is smaller than the transaction size, skip
         * the rest as only bigger ones follow */
        if (BURST_BYTES(endpoint) > count)
        {
            break;
        }

        /* The cost is the number of burst transactions plus the
         * number of single transactions needed to complete the
         * block.
         */
        cost = count / BURST_BYTES(endpoint);
        cost += (count % BURST_BYTES(endpoint)) / TR_BYTES(endpoint);

        /* If this is cheaper, remember it */
        if (cost <= cheapest_cost)
        {
            cheapest_cost = cost;
            cheapest = endpoint->msize;
        }
    }

    /* Set the cheapest in the endpoint */
    PRINTD(dma, PC302_DMA_LVL_DEBUG, "msize set to %d\n", endpoint->msize);
    endpoint->msize = cheapest;
}

/*****************************************************************************
   Do some parameter checks on endpoint parameters.
*****************************************************************************/
static unsigned
pc302_dma_params_valid(pc302_dma_t dma, 
                       pc302_dma_endpoint_t *src, 
                       pc302_dma_endpoint_t *dst, 
                       size_t count)
{
    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return FAIL;
    }

    /* Check that the source input parameters are in range */
    if (src != NULL)
    {
        /* Master is in range */
        if (src->master >= PC302_DMA_MASTERS)
        {
            PRINTE("Master out of range\n");
            return FAIL;
        }

        /* Bus width supported by master */
        if (src->tr_width >
            pc302_dma_buswidth_lookup[dma->id][src->master])
        {
            PRINTE("Master does not support specified bus width. Master=%d "
                 "Bus width=%d\n", src->master, src->tr_width);
            return FAIL;
        }

        /* Burst size is in range */
        if (src->msize > PC302_DMA_MS_AUTO)
        {
            PRINTE("Burst size is out of range. msize=%d\n", src->msize);
            return FAIL ;
        }

        /* Check that the source transfer width is suitable for the number of
           bytes to be transferred */
        if ((count % TR_BYTES(src)) != 0)
        {
            PRINTE("Block size (%d) not divisible by transfer width\n",
              count);
            return FAIL;
        }

        /* Check that we are not setting memory as a flow controller */
        if (src->periph_not_mem == 0 && src->flow_controller)
        {
            PRINTE("Memory endpoint cannot be flow controller\n");
            return FAIL;
        }
    }

    if (dst != NULL)
    {
        /* Master is in range */
        if (dst->master >= PC302_DMA_MASTERS)
        {
            PRINTE("Master out of range\n");
            return FAIL;
        }

        /* Bus width supported by master */
        if (dst->tr_width >
            pc302_dma_buswidth_lookup[dma->id][dst->master])
        {
            PRINTE("Master does not support specified bus width. Master=%d "
                 "Bus width=%d\n", dst->master, dst->tr_width);
            return FAIL;
        }

        /* Burst size is in range */
        if (dst->msize > PC302_DMA_MS_AUTO)
        {
            PRINTE("Burst size is out of range. msize=%d\n", dst->msize);
            return FAIL ;
        }

        if (dst->periph_not_mem == 0 && dst->flow_controller)
        {
            PRINTE("Memory endpoint cannot be flow controller\n");
            return FAIL;
        }
    }

    /* Check that both the source and destination are not flow controllers */
    if (src != NULL && dst != NULL)
    {
        if (src->flow_controller && dst->flow_controller)
        {
            PRINTE("Only one endpoint can be flow controller\n");
            return FAIL;
        }
    }

    return SUCCESS;
}

/*****************************************************************************
   Set the handshaking parameter in the transfer endpoint
*****************************************************************************/
static void
pc302_dma_setup_handshaking(pc302_dma_xfr_t dma_xfr, 
                            pc302_dma_handshake_t *src_handshake, 
                            pc302_dma_handshake_t *dst_handshake)
{
    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL transfer handle\n");
        return;
    }

    if (dma_xfr->state == RUNNING)
    {
        PRINTE("Attempting to configure a running transfer\n");
        return;
    }

    /* Copy the parameters */
    if (src_handshake != NULL)
    {
        dma_xfr->src_handshake = *src_handshake;
        dma_xfr->src_hw_handshaking_on = 1;

        /* Mark the interface as in use */
        dma_xfr->dma->handshaking_inuse |= (1 << src_handshake->hwInterface);
    }

    if (dst_handshake != NULL)
    {
        dma_xfr->dst_handshake = *dst_handshake;
        dma_xfr->dst_hw_handshaking_on = 1;

        /* Mark the interface as in use */
        dma_xfr->dma->handshaking_inuse |= (1 << dst_handshake->hwInterface);
    }
}

/*****************************************************************************
   Check the handshaking parameters. Check if any handshaking interface is
   used on more than one endpoint.
*****************************************************************************/
static unsigned
pc302_dma_handshaking_params_valid(pc302_dma_t dma, 
                            pc302_dma_endpoint_t *src, 
                            pc302_dma_endpoint_t *dst, 
                            pc302_dma_handshake_t *src_handshake, 
                            pc302_dma_handshake_t *dst_handshake)
{
    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return FAIL;
    }

    if (src_handshake != NULL)
    {
        /* Test for the situation where the H/W interface is specified
           is already in use */
        if ((dma->handshaking_inuse & (1 <<src_handshake->hwInterface)) != 0)
        {
            PRINTE("Source handshaking interface in use\n");
            return FAIL;
        }

        /* Test for the situation where the H/W interface is not supported
           by the master specified */
        if (!(pc302_dma_hw_if_support_lookup[dma->id][
            src_handshake->hwInterface] & (1 << src->master)))
        {
            PRINTE("Source handshaking interface (%d) is not supported "
               "by master selected (%d)\n", src_handshake->hwInterface, 
               src->master);
            return FAIL;
        }
    }

    if (dst_handshake != NULL)
    {
        if ((dma->handshaking_inuse & (1 << dst_handshake->hwInterface)) != 0)
        {
            PRINTE("Destination handshaking interface in use\n");
            return FAIL;
        }

        if (!(pc302_dma_hw_if_support_lookup[dma->id][
            dst_handshake->hwInterface] & (1 << dst->master)))
        {
            PRINTE("Destination handshaking interface (%d) is not "
               "supported by master selected (%d)\n", 
               dst_handshake->hwInterface, dst->master);
            return FAIL;
        }

    }

    if (src_handshake != NULL && dst_handshake != NULL)
    {
        if (src_handshake->hwInterface == dst_handshake->hwInterface)
        {
            PRINTE("Same Source and Destination handshaking interface\n");
            return FAIL;
        }
    }

    return SUCCESS;
}

/*****************************************************************************
   Print the bits of the low dword of the control register
*****************************************************************************/
static void
pc302_dma_print_ctl_low(pc302_dma_t dma, u32 reg)
{
    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return;
    }

    PRINT_BIT("INT_EN", PC302_DMA_INT_EN);
    PRINT_FIELD("DST_TR_WIDTH", PC302_DMA_DST_TR_WIDTH_MASK, 
       PC302_DMA_DST_TR_WIDTH_SHIFT);
    PRINT_FIELD("SRC_TR_WIDTH", PC302_DMA_SRC_TR_WIDTH_MASK, 
        PC302_DMA_SRC_TR_WIDTH_SHIFT);
    PRINT_FIELD("DINC", PC302_DMA_DINC_MASK, PC302_DMA_DINC_SHIFT);
    PRINT_FIELD("SINC", PC302_DMA_SINC_MASK, PC302_DMA_SINC_SHIFT);
    PRINT_FIELD("DST_MSIZE", PC302_DMA_DST_MSIZE_MASK, 
        PC302_DMA_DST_MSIZE_SHIFT);
    PRINT_FIELD("SRC_MSIZE", PC302_DMA_SRC_MSIZE_MASK, 
        PC302_DMA_SRC_MSIZE_SHIFT);
    PRINT_BIT("SRC_GATHER_EN", PC302_DMA_SRC_GATHER_EN);
    PRINT_BIT("DST_SCATTER_EN", PC302_DMA_DST_SCATTER_EN);
    PRINT_FIELD("TT_FC", PC302_DMA_TT_FC_MASK, PC302_DMA_TT_FC_SHIFT);
    PRINT_FIELD("DMS", PC302_DMA_DMS_MASK, PC302_DMA_DMS_SHIFT);
    PRINT_FIELD("SMS", PC302_DMA_SMS_MASK, PC302_DMA_SMS_SHIFT);
    PRINT_BIT("LLP_DST_EN", PC302_DMA_LLP_DST_EN);
    PRINT_BIT("LLP_SRC_EN", PC302_DMA_LLP_SRC_EN);
}

/*****************************************************************************
   Print the bits of the high dword of the control register
*****************************************************************************/
static void
pc302_dma_print_ctl_high(pc302_dma_t dma, u32 reg)
{
    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return;
    }

    PRINT_FIELD("BLOCK_TS", PC302_DMA_BLOCK_TS_MASK, PC302_DMA_BLOCK_TS_SHIFT);
    PRINT_BIT("DONE", PC302_DMA_DONE);
}

/*****************************************************************************
 * Public function declarations
 *****************************************************************************/

/*****************************************************************************
   Set up a DMA transfer between src and dst
*****************************************************************************/
pc302_dma_xfr_t
pc302_dma_setup_direct_xfr(pc302_dma_t dma, 
                           pc302_dma_endpoint_t *src, 
                           pc302_dma_endpoint_t *dst, 
                           pc302_dma_handshake_t *src_handshake, 
                           pc302_dma_handshake_t *dst_handshake, 
                           size_t count, 
                           u32 flags, 
                           int ( *handler )( void *cookie,
                                             int errno ),
                           void *cookie)
{
    struct pc302_dma_xfr_tag *dma_xfr = NULL;
    unsigned long spinlock_flags = 0;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return NULL;
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&dma->spinlock, spinlock_flags);

    /* Check we have source and destination */
    if (src == NULL || (dst == NULL))
    {
        PRINTE("src or dst NULL\n");
        goto EXIT;
    }

    /* Do various parameter checks and exit if they fail */
    if (pc302_dma_params_valid(dma, src, dst, count))
    {
        goto EXIT;
    }

    /* Check the handshaking parameters */
    if (pc302_dma_handshaking_params_valid(
            dma, src, dst, src_handshake, dst_handshake))
    {
        goto EXIT;
    }

    /* Allocate a transfer object including DMA channel */
    dma_xfr = pc302_dma_alloc_xfr(dma, handler);
    if (dma_xfr == NULL)
    {
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
    pc302_dma_set_auto_burst_length(dma, &dma_xfr->src, count);
    pc302_dma_set_auto_burst_length(dma, &dma_xfr->dst, count);

    /* Setup the handshaking */
    pc302_dma_setup_handshaking(dma_xfr, src_handshake, dst_handshake);

    /* Disable all interrupts for the channel */
    pc302_dma_write_int_registers(dma, PC302_DMA_INT_ALL, 
            PC302_DMA_INTERRUPT_MASK, 
            PC302_DMA_IRQ_DISABLE_CHANNEL(dma_xfr->channel));

EXIT:
    spin_unlock_irqrestore(&dma->spinlock, spinlock_flags);
    return dma_xfr;
}

/*****************************************************************************
   Start a DMA transfer on a channel previously set up with the
   pc302_dma_setup_direct_xfr() or pc302_dma_setup_list_xfr() functions.
*****************************************************************************/
int
pc302_dma_start(pc302_dma_xfr_t dma_xfr)
{
    int ret = SUCCESS;
    unsigned long spinlock_flags = 0;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return -EINVAL;
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);

    /* Check and set the transfer state */
    if (pc302_dma_get_xfr_state(dma_xfr) != STOPPED)
    {
        /* Already running error */
        PRINTE("Transfer running\n");
        ret = -EINVAL;
        goto EXIT;
    }

    /* Reset the current block if using multi-block list */
    if (dma_xfr->list != NULL)
    {
        dma_xfr->list->current_block = dma_xfr->list->descriptor_list;
    }

    /* Write the DMA hardware */
    ret = pc302_dma_update_xfr_registers(dma_xfr);
    if (ret != SUCCESS)
    {
        goto EXIT;
    }

    /* Start the DMA channel */
    pc302_dma_iowrite32(dma_xfr->dma,
        PC302_DMA_ENABLE_CHANNEL(dma_xfr->channel), 
        PC302_DMA_CHANNEL_ENABLE_REG_OFFSET);
    dma_xfr->state = RUNNING;

EXIT:
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   Abort a DMA transfer
*****************************************************************************/
int
pc302_dma_abort(pc302_dma_xfr_t dma_xfr)
{
    int ret = SUCCESS;
    u32 status=0;
    unsigned long timeout_jiffies=0;
    unsigned long spinlock_flags = 0;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return -EINVAL;
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);

    /* Check and set the transfer state */
    if (pc302_dma_get_xfr_state(dma_xfr) != RUNNING)
    {
        /* Already stopped. It is not unusual for this to happen */
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_NOTICE, 
            "Aborting a stopped transfer\n");

        /* Disable the channel */
        pc302_dma_iowrite32(dma_xfr->dma, 
            PC302_DMA_DISABLE_CHANNEL(dma_xfr->channel), 
            PC302_DMA_CHANNEL_ENABLE_REG_OFFSET);

        goto EXIT; /* Return 0 as function has succeeded */
    }
    dma_xfr->state = STOPPING;

    /* Set the suspend bit, wait for the FIFO to clear before
     * disabling the channel
     */
    dma_xfr->config_low |= PC302_DMA_CH_SUSP;
    pc302_dma_iowrite32(dma_xfr->dma, dma_xfr->config_low, 
            PC302_DMA_N_LOW_CONFIG_REG_OFFSET(dma_xfr->channel));

    /* Release the API spinlock while waiting as interrupts may
     * need to be serviced to empty the FIFO and these will try
     * to grab the lock.
     * This is safe as we are not updating state while polling.
     */
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);

    /* Poll the FIFO_EMPTY bit */
    timeout_jiffies = jiffies + ABORT_TIMEOUT;
    while (1)
    {
        /* Check for time out */
        if (time_after(jiffies, timeout_jiffies))
        {
            PRINTE("Timed out waiting for FIFO to empty\n");
            ret = -EIO;
            break;
        }

        /* Check for the empty FIFO empty */
        status = pc302_dma_ioread32(dma_xfr->dma, 
            PC302_DMA_N_LOW_CONFIG_REG_OFFSET(dma_xfr->channel));
        if (status & PC302_DMA_FIFO_EMPTY)
        {
            break;
        }
        cpu_relax();
    }

    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);

    /* Disable the channel */
    pc302_dma_iowrite32(dma_xfr->dma, 
        PC302_DMA_DISABLE_CHANNEL(dma_xfr->channel), 
        PC302_DMA_CHANNEL_ENABLE_REG_OFFSET);

    /* Now stopped, set state */
    dma_xfr->state = STOPPED;

EXIT:
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   Purpose: Stop a DMA transfer and release resources
*****************************************************************************/
int
pc302_dma_release(pc302_dma_xfr_t dma_xfr)
{
    unsigned long spinlock_flags = 0;
    int ret = SUCCESS;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return -EINVAL;
    }

    /* Make sure the transfer has is stopped */
    ret = pc302_dma_abort(dma_xfr);
    if (ret)
    {
       return ret; 
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);

    /* Reset any interrupts that have been left pending */
    pc302_dma_write_int_registers(dma_xfr->dma, PC302_DMA_INT_ALL, 
         PC302_DMA_INTERRUPT_CLEAR, PC302_DMA_IRQ_CHANNEL(dma_xfr->channel));

    /* Disconnect from multi-block list */
    if (dma_xfr->list != NULL)
    {
        dma_xfr->list->dma_xfr = NULL;
    }

    /* Free the resources */
    pc302_dma_free_xfr(dma_xfr);

    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   Creates a DMA buffer list that can be assigned to a multi-block transfer.
*****************************************************************************/
pc302_dma_list_t
pc302_dma_list_create(pc302_dma_t dma, 
                      size_t count)
{
    pc302_dma_list_t list=NULL;
    unsigned long spinlock_flags = 0;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return NULL;
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&dma->spinlock, spinlock_flags);

    /* Allocate the list */
    list = pc302_dma_alloc_list(dma, count);
    if (list == NULL)
    {
        PRINTE("Not enough resources\n");
        goto EXIT;
    }

    /* Setup the structure */
    list->dma = dma;
    list->dma_xfr = NULL;
    list->items = 0;
    list->last_desc = NULL;
    list->length = count;
    list->is_multi_block_src = 1;
    list->is_multi_block_dst = 1;

EXIT:
    spin_unlock_irqrestore(&dma->spinlock, spinlock_flags);
    return list;
}

/*****************************************************************************
   Add an entry to the end of a DMA multi-block list
*****************************************************************************/
int
pc302_dma_list_add(pc302_dma_list_t list, 
                   pc302_dma_endpoint_t *src, 
                   pc302_dma_endpoint_t *dst, 
                   pc302_dma_handshake_t *src_handshake, 
                   pc302_dma_handshake_t *dst_handshake, 
                   size_t count)
{
    int ret = SUCCESS;
    descriptor_t *new_desc=NULL;
    dma_addr_t dma_new_desc=0;
    unsigned long spinlock_flags = 0;

    if (TRANSFER_LIST_HANDLE_ENABLED(list))
    {
        PRINTD(list->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer list handle\n");
        return -EINVAL;
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&list->dma->spinlock, spinlock_flags);

    /* Do various parameter checks and exit if they fail */
    if (pc302_dma_params_valid(list->dma, src, dst, count))
    {
        ret = -EINVAL; /* PRINTEs provided in the function */
        goto EXIT;
    }

    /* Check the block and handshaking parameters before adding the block
       to the list */
    if (pc302_dma_handshaking_params_valid(list->dma, src, dst, src_handshake, 
            dst_handshake))
    {
        ret = -EINVAL;
        goto EXIT; /* PRINTEs provided in the function */
    }


    /* Check we have room */
    if (list->length == list->items)
    {
        PRINTE("List full\n");
        ret = -ENOMEM;
        goto EXIT;
    }

    /* Check for transfers running the list */
    if (list->dma_xfr != NULL)
    {
        if (pc302_dma_get_xfr_state(list->dma_xfr) != STOPPED)
        {
            /* Cannot modify a list that is active */
            PRINTE("Transfer running\n");
            ret = -EBUSY;
            goto EXIT;
        }
    }

    /* If this is the first item, we must have both src and dst parameters */
    if (list->items == 0)
    {
        if ((src == NULL) || (dst == NULL))
        {
            PRINTE("First list element must have both source and "
                "destination endpoints\n");
            ret = -EINVAL;
            goto EXIT;
        }
    }
    /* If this is the second element and src or dst are NULL, 
     * we are not using a list for the endpoint
     */
    else if (list->items)
    {
        if ((src == NULL) && (dst == NULL))
        {
            /* Either src or dst must be set */
            PRINTE("List element must have either source or destination "
                "endpoints\n");
            ret = -EINVAL;
            goto EXIT;
        }

        /* If the source is not set, we are not using a list for it */
        if (src == NULL)
        {
            list->is_multi_block_src = 0;
        }

        /* If the destination is not set, we are not using a list for it */
        if (dst == NULL)
        {
            list->is_multi_block_dst = 0;
        }
    }
    else
    { /* Not the first or second item */
        /* Check that, once we have chosen a non list type multi-block transfer
         * (auto-reload or continuous), we must stick to it.
         */
        if (list->is_multi_block_src)
        {
            if (src == NULL)
            {
                PRINTE("NULL SRC descriptor in multi block list\n");
                ret = -EINVAL;
                goto EXIT;
            }
        }
        else
        {
            if (src != NULL)
            {
                PRINTE("Non NULL SRC descriptor in multi block list\n");
                ret = -EINVAL;
                goto EXIT;
            }
        }

        if (list->is_multi_block_dst)
        {
            if (dst == NULL)
            {
                PRINTE("NULL DST descriptor in multi block list\n");
                ret = -EINVAL;
                goto EXIT;
            }
        }
        else
        {
            if (dst != NULL)
            {
                PRINTE("Non NULL DST descriptor in multi block list\n");
                ret = -EINVAL;
                goto EXIT;
            }
        }
    }

    /* Get a pointer to the new descriptor */
    new_desc = &(list->descriptor_list[list->items]);

    /* Get the physical address (DMA address) of the new descriptor */
    dma_new_desc = list->dma_descriptor_list +
        (sizeof(descriptor_t) * list->items);

    /* Allocate the new descriptor */
    list->items++;

    /* Record the first source/destination for the non-list type
     * transfers in the software list.
     */
    if (src != NULL)
    {
        new_desc->sw.src = *src;
        /* Deal with auto burst length */
        pc302_dma_set_auto_burst_length(list->dma, &new_desc->sw.src, count);
    }

    if (dst != NULL)
    {
        new_desc->sw.dst = *dst;
        /* Deal with auto burst length */
        pc302_dma_set_auto_burst_length(list->dma, &new_desc->sw.dst, count);
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
    if (list->last_desc != NULL)
    {
        list->last_desc->hw.link = dma_new_desc;
        list->last_desc->hw.control |=
                pc302_dma_build_llp_control_register(list->dma, list);
        if (list->last_desc->hw.control == ERROR_INDICATED)
        {
            ret = -EINVAL; /* PRINTEs provided in function */
            goto EXIT;
        }
    }

    /* Where either the source or destination is not list type, and is null
     * at this point, use the endpoint descriptor at the head of the list
     */
    if (src == NULL)
    {
        src = &list->descriptor_list->sw.src;
    }
    if (dst == NULL)
    {
        dst = &list->descriptor_list->sw.dst;
    }

    /* Setup the new hardware block descriptor */
    new_desc->hw.source_addr = src->dma_addr;
    new_desc->hw.dest_addr = dst->dma_addr;
    new_desc->hw.control = pc302_dma_build_control_register(list->dma, src,
        dst);
    if (new_desc->hw.control == ERROR_INDICATED)
    {
        ret = -EINVAL; /* PRINTEs provided in function */
        goto EXIT;
    }
    new_desc->hw.block_ts = count / TR_BYTES(src);

    /* The new last descriptor is now the one we just created */
    list->last_desc = new_desc;

EXIT:
    spin_unlock_irqrestore(&list->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   This function resets a list to the state that it was in just after
   it was created when calling pc302_dma_list_create
*****************************************************************************/
int
pc302_dma_list_clear(pc302_dma_list_t list)
{
    int ret = SUCCESS;
    unsigned long spinlock_flags = 0;

    if (TRANSFER_LIST_HANDLE_ENABLED(list))
    { 
        PRINTD(list->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else 
    { 
        PRINTE("NULL  transfer list handle\n"); 
        return -EINVAL; 
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&list->dma->spinlock, spinlock_flags);

    /* Check for transfers running the list */
    if (list->dma_xfr != NULL)
    {
        if (pc302_dma_get_xfr_state(list->dma_xfr) != STOPPED)
        {
            /* Cannot modify a list that is active */
            PRINTE("Transfer running\n");
            ret = -EBUSY;
            goto EXIT;
        }
    }

    /* Reset the list */
    list->items = 0;
    list->last_desc = NULL;
    list->is_multi_block_src = 1;
    list->is_multi_block_dst = 1;

EXIT:
    spin_unlock_irqrestore(&list->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   This function frees the resources allocated by a list
*****************************************************************************/
int
pc302_dma_list_destroy(pc302_dma_list_t list)
{
    int ret = SUCCESS;
    struct pc302_dma_list_tag local_list;
    unsigned long spinlock_flags = 0;

    if (TRANSFER_LIST_HANDLE_ENABLED(list))
    { 
        PRINTD(list->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else 
    { 
        PRINTE("NULL  transfer list handle\n"); 
        return -EINVAL; 
    }

    /* Initialise local_list */
    memset(&local_list, 0, sizeof(struct pc302_dma_list_tag));

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&list->dma->spinlock, spinlock_flags);

    /* Take a copy of the list before we free it so that we can free
     * the DMA coherent area outside of the spinlock. This is done
     * because dma_free_coherent must not be called with interrupts
     * disabled.
     */
    local_list = *list;

    /* Make sure there are no references to the list from transfers.
     * A transfers using the list needs to be destroyed before the
     * list can be destroyed.
     */
    if (list->dma_xfr != NULL)
    {
        /* Cannot destroy a list that is referenced */
        PRINTE("List in use\n");
        ret = -EBUSY;
        goto EXIT;
    }

    /* Free the object */
    kfree(list);

EXIT:
    spin_unlock_irqrestore(&local_list.dma->spinlock, spinlock_flags);

    /* Free the descriptor list if we are not here because of an error */
    if (ret == SUCCESS)
    {
        dma_free_coherent(local_list.device, 
            (sizeof(descriptor_t) * local_list.length), 
            local_list.descriptor_list, local_list.dma_descriptor_list);
    }

    return ret;
}

/*****************************************************************************
   Set up a multi-block DMA transfer
*****************************************************************************/
pc302_dma_xfr_t
pc302_dma_setup_list_xfr(pc302_dma_list_t list, 
                         pc302_dma_handshake_t *src_handshake, 
                         pc302_dma_handshake_t *dst_handshake, 
                         u32 flags, 
                         int ( *handler )( void *cookie,
                                           int errno),
                         void *cookie)
{
    struct pc302_dma_xfr_tag *dma_xfr = NULL;
    unsigned long spinlock_flags = 0;

    if (TRANSFER_LIST_HANDLE_ENABLED(list))
    { 
        PRINTD(list->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else 
    { 
        PRINTE("NULL  transfer list handle\n"); 
        return NULL; 
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);

    /* Check that the list is not in use with another transfer */
    if (list->dma_xfr != NULL)
    {
        PRINTE("List in use\n");
        goto EXIT;
    }

    /* Allocate a transfer object including DMA channel */
    dma_xfr = pc302_dma_alloc_xfr(list->dma, handler);
    if (dma_xfr == NULL)
    {
        PRINTE("Out of resources\n");
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
    pc302_dma_setup_handshaking(dma_xfr, src_handshake, dst_handshake);

    /* Disable all interrupts for the channel */
    pc302_dma_write_int_registers(list->dma, PC302_DMA_INT_ALL, 
            PC302_DMA_INTERRUPT_MASK, 
            PC302_DMA_IRQ_DISABLE_CHANNEL(dma_xfr->channel));

EXIT:
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
    return dma_xfr;
}

/*****************************************************************************
   Enable interrupt generation of a number of types of interrupt on a
   DMA transfer.
*****************************************************************************/
int pc302_dma_enable_int(pc302_dma_xfr_t dma_xfr, 
                          unsigned irq_types)
{
    unsigned long spinlock_flags = 0;
    int ret = SUCCESS;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return -EINVAL;
    }

    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);
    pc302_dma_write_int_registers(dma_xfr->dma, irq_types, 
         PC302_DMA_INTERRUPT_MASK, 
         PC302_DMA_IRQ_ENABLE_CHANNEL(dma_xfr->channel));
    ret = pc302_dma_request_irq(dma_xfr->dma, (1<<dma_xfr->channel));
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   Disable interrupt generation of a number of types of interrupt on a
   DMA transfer
*****************************************************************************/
int
pc302_dma_disable_int(pc302_dma_xfr_t dma_xfr, 
                      unsigned irq_types)
{
    unsigned long spinlock_flags = 0;
    int ret = SUCCESS;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return -EINVAL;
    }

    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);
    pc302_dma_write_int_registers(dma_xfr->dma, irq_types, 
        PC302_DMA_INTERRUPT_MASK, 
        PC302_DMA_IRQ_DISABLE_CHANNEL(dma_xfr->channel));
    ret = pc302_dma_free_irq(dma_xfr->dma, (1<<dma_xfr->channel));
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   Clears a number of interrupt status bits, ready for new interrupt
   generation.
*****************************************************************************/
void
pc302_dma_clear_int(pc302_dma_xfr_t dma_xfr, 
                    unsigned irq_types)
{
    unsigned long spinlock_flags = 0;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return;
    }

    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);
    pc302_dma_write_int_registers(dma_xfr->dma, irq_types, 
            PC302_DMA_INTERRUPT_CLEAR, PC302_DMA_IRQ_CHANNEL(dma_xfr->channel));
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
}

/*****************************************************************************
   Get the raw status of a DMA transfer. The value returned is a logical OR
   of one or more dma_irq_type_t types
*****************************************************************************/
unsigned
pc302_dma_get_raw_status(pc302_dma_xfr_t dma_xfr, 
                         unsigned irq_types)
{
    dma_irq_type_t ret = 0; /* No interrupts selected */
    u32 hw_int_status = 0;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return 0; /* No status bits set */
    }

    if (irq_types & PC302_DMA_INT_BLOCK)
    {
        hw_int_status = pc302_dma_ioread32(dma_xfr->dma, 
            PC302_DMA_BLOCK_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
        if (hw_int_status & PC302_DMA_IRQ_CHANNEL(dma_xfr->channel))
        {
            ret |= PC302_DMA_INT_BLOCK;
        }
    }

    if (irq_types & PC302_DMA_INT_DST_TRANSACTION)
    {
        hw_int_status = pc302_dma_ioread32(dma_xfr->dma, 
            PC302_DMA_DST_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
        if (hw_int_status & PC302_DMA_IRQ_CHANNEL(dma_xfr->channel))
        {
            ret |= PC302_DMA_INT_DST_TRANSACTION;
        }
    }

    if (irq_types & PC302_DMA_INT_ERROR)
    {
        hw_int_status = pc302_dma_ioread32(dma_xfr->dma, 
            PC302_DMA_ERROR_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
        if (hw_int_status & PC302_DMA_IRQ_CHANNEL(dma_xfr->channel))
        {
            ret |= PC302_DMA_INT_ERROR;
        }
    }

    if (irq_types & PC302_DMA_INT_SRC_TRANSACTION)
    {
        hw_int_status = pc302_dma_ioread32(dma_xfr->dma, 
            PC302_DMA_SRC_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
        if (hw_int_status & PC302_DMA_IRQ_CHANNEL(dma_xfr->channel))
        {
            ret |= PC302_DMA_INT_SRC_TRANSACTION;
        }
    }

    if (irq_types & PC302_DMA_INT_TRANSFER)
    {
        hw_int_status = pc302_dma_ioread32(dma_xfr->dma, 
            PC302_DMA_TXFER_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
        if (hw_int_status & PC302_DMA_IRQ_CHANNEL(dma_xfr->channel))
        {
            ret |= PC302_DMA_INT_TRANSFER;
        }
    }

    return ret;
}

/*****************************************************************************
   This function is used to setup the scatter or gather parameters of
   a DMA transfer.
*****************************************************************************/
int
pc302_dma_setup_sg(pc302_dma_xfr_t dma_xfr, 
                   pc302_dma_sg_t *sg, 
                   pc302_dma_endpoint_type_t src_dst)
{
    int ret = SUCCESS;
    unsigned long spinlock_flags = 0;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return -EINVAL;
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);

    /* Check the transfer state */
    if (pc302_dma_get_xfr_state(dma_xfr) != STOPPED)
    {
        /* Already running error */
        PRINTE("Transfer running\n");
        ret = -EINVAL;
        goto EXIT;
    }

    /* Copy the parameters */
    if (src_dst == PC302_DMA_SRC)
    {
        dma_xfr->gather = *sg;
    }
    else
    {
        dma_xfr->scatter = *sg;
    }

EXIT:
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   This function is used to request a new transaction on the source or
   destination of a transfer.
*****************************************************************************/
int
pc302_dma_request_transaction(pc302_dma_xfr_t dma_xfr, 
                              pc302_dma_endpoint_type_t src_dst, 
                              unsigned *bytes_left)
{
    pc302_dma_endpoint_t *endpoint = NULL;
    descriptor_t *curr_desc = NULL;
    int ret = SUCCESS;
    transaction_type_t xfr_type = TRT_NONE;
    unsigned long spinlock_flags = 0;

    if (TRANSFER_HANDLE_ENABLED(dma_xfr))
    {
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL  transfer handle\n");
        return -EINVAL;
    }

    /* Simply lock the API against multiple threads */
    spin_lock_irqsave(&dma_xfr->dma->spinlock, spinlock_flags);

    /* Check the transfer state */
    if (pc302_dma_get_xfr_state(dma_xfr) != RUNNING)
    {
        /* Not running error */
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO,"Transfer not running\n");
        ret = -EINVAL;
        goto EXIT;
    }

    /* Check that a transfer is not already in progress */
    if (pc302_dma_get_transaction_state(dma_xfr, src_dst))
    {
        /* Transaction in progress error */
        PRINTD(dma_xfr->dma, PC302_DMA_LVL_INFO, 
               "Transaction currently in progress\n");
        ret = -EBUSY;
        goto EXIT;
    }

    /* If this is not a flow controlling endpoint, always do a
     * burst transfer. The hardware will enter the single transaction
     * region automatically and inter-operate our request appropriately
     */
    if ((src_dst == PC302_DMA_SRC && dma_xfr->src.flow_controller == 0)
     || (src_dst == PC302_DMA_DST && dma_xfr->dst.flow_controller == 0))
    {

        pc302_dma_transfer_request(dma_xfr, src_dst, 
            TRT_SINGLE | TRT_BURST);
    }
    else
    {  /* Endpoint is a flow controller */

        /* Handle non-list differently from list transfers */
        if (dma_xfr->list == NULL)
        {
            /* Get the correct endpoint */
            if (src_dst == PC302_DMA_SRC)
            {
                endpoint = &dma_xfr->src;
            }
            else
            {
                endpoint = &dma_xfr->dst;
            }
        }
        else
        { /* Multi-block list type */
            /* Find the current running block */
            curr_desc = dma_xfr->list->current_block;

            /* Get the correct endpoint */
            if (src_dst == PC302_DMA_SRC)
            {
                /* If auto-reload type, use the first element */
                if (dma_xfr->list->is_multi_block_src == 0)
                {
                    curr_desc = &dma_xfr->list->descriptor_list[0];
                }
                endpoint = &curr_desc->sw.src;
            }
            else
            {  /* Dst endpoint */
                /* If auto-reload type, use the first element */
                if (dma_xfr->list->is_multi_block_dst == 0)
                {
                    curr_desc = &dma_xfr->list->descriptor_list[0];
                }
                endpoint = &curr_desc->sw.dst;
            }
        } /* multi-block list type transfer */

        /* Work out the transfer type based in the endpoint in use */
        xfr_type = pc302_dma_determine_transfer_type(dma_xfr, 
            endpoint, bytes_left);

        /* do nothing and exit if we have nothing more to transfer */
        if (xfr_type != TRT_NONE)
        {
            /* Request the transfer */
            pc302_dma_transfer_request(dma_xfr, src_dst, xfr_type);
        }
    }   /* endpoint is a flow controller */

EXIT:
    spin_unlock_irqrestore(&dma_xfr->dma->spinlock, spinlock_flags);
    return ret;
}

/*****************************************************************************
   Get the device handle of one of the PC302 DMA controllers.
*****************************************************************************/
pc302_dma_t
pc302_dma_get_dma_handle(unsigned dmaNumber)
{
    if (dmaNumber < 2)
    {
        return dmac[dmaNumber].dma;
    }
    else
    {
        PRINTE("Returning NULL as DMA handle\n");
        return NULL;
    }
}

/*****************************************************************************
   Print out all DMA configuration registers for all channels, including the
   information held in multi-block lists.
*****************************************************************************/
void
pc302_dma_dump_regs(pc302_dma_t dma)
{
    unsigned i=0;
    u32 reg=0;
    u32 llp=0;
    u32 head_llp=0;
    void __iomem *virt_addr = NULL;
    int block=0;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return;
    }

    /* Spinlocks not required as no DMA register or handle parameters are
       being updsted */

    for (i =0;i< PC302_DMA_CHANNELS;i++)
    {
        printk("CHANNEL %u:\n", i);

        PRINT_REG("SAR", PC302_DMA_N_SRC_ADDR_REG_OFFSET(i));
        PRINT_REG("DAR", PC302_DMA_N_DST_ADDR_REG_OFFSET(i));
        PRINT_REG("LLP", PC302_DMA_N_LLP_REG_OFFSET(i));
        head_llp = reg;

        PRINT_REG("CTLlow", PC302_DMA_N_CTRL_REG_OFFSET(i));
        pc302_dma_print_ctl_low(dma, reg);
        PRINT_REG("CTLhigh", PC302_DMA_N_BLOCK_SIZE_REG_OFFSET(i));
        pc302_dma_print_ctl_high(dma, reg);

        PRINT_REG("CFGlow", PC302_DMA_N_LOW_CONFIG_REG_OFFSET(i));
        PRINT_FIELD("CH_PRIOR", PC302_DMA_CH_PRIOR_MASK, 
            PC302_DMA_CH_PRIOR_SHIFT);
        PRINT_BIT("CH_SUSP", PC302_DMA_CH_SUSP);
        PRINT_BIT("FIFO_EMPTY", PC302_DMA_FIFO_EMPTY);
        PRINT_BIT("HS_SEL_DST", PC302_DMA_HS_SEL_DST);
        PRINT_BIT("HS_SEL_SRC", PC302_DMA_HS_SEL_SRC);
        PRINT_BIT("DST_HS_POL", PC302_DMA_DST_HS_POL);
        PRINT_BIT("SRC_HS_POL", PC302_DMA_SRC_HS_POL);
        PRINT_FIELD("MAX_ABRST", PC302_DMA_MAX_ABRST_MASK, 
            PC302_DMA_MAX_ABRST_SHIFT);
        PRINT_BIT("RELOAD_SRC", PC302_DMA_RELOAD_SRC);
        PRINT_BIT("RELOAD_DST", PC302_DMA_RELOAD_DST);

        PRINT_REG("CFGhigh", PC302_DMA_N_HIGH_CONFIG_REG_OFFSET(i));
        PRINT_BIT("FCMODE", PC302_DMA_FC_MODE);
        PRINT_BIT("FIFO_MODE", PC302_DMA_FIFO_MODE);
        PRINT_FIELD("PROTCTL", PC302_DMA_PROTCTL_MASK, PC302_DMA_PROTCTL_SHIFT);
        PRINT_FIELD("SRC_PER", PC302_DMA_SRC_PER_MASK, PC302_DMA_SRC_PER_SHIFT);
        PRINT_FIELD("DEST_PER", PC302_DMA_DST_PER_MASK, 
            PC302_DMA_DST_PER_SHIFT);

        PRINT_REG("SGR", PC302_DMA_N_SRC_GATHER_REG_OFFSET(i));
        PRINT_FIELD("SGI", PC302_DMA_SG_INTERVAL_MASK, 
            PC302_DMA_SG_INTERVAL_SHIFT);
        PRINT_FIELD("SGC", PC302_DMA_SG_COUNT_MASK, PC302_DMA_SG_COUNT_SHIFT);

        PRINT_REG("DSR", PC302_DMA_N_DST_SCATTER_REG_OFFSET(i));
        PRINT_FIELD("DSI", PC302_DMA_SG_INTERVAL_MASK, 
            PC302_DMA_SG_INTERVAL_SHIFT);
        PRINT_FIELD("DSC", PC302_DMA_SG_COUNT_MASK, PC302_DMA_SG_COUNT_SHIFT);

        if (head_llp != 0)
        {
            /* Link list pointer is not NULL indicating that this channel
               has a list of descriptors */
            printk("\n    MULTI BLOCK LIST:\n");
            llp = head_llp;
            do
            {
                /* Convert to a virtual address */
                virt_addr = ioremap(llp, 
                    (unsigned long)sizeof(descriptor_t));
                printk("        BLOCK %d phy addr=0x%p, virt_addr=0x%p\n", 
                    block, (void *)llp, virt_addr);
                block++;

                PRINT_LLP_REG("LLP.SAR", virt_addr); /* source_addr */
                PRINT_LLP_REG("LLP.DAR", virt_addr+4); /* dest_addr */
                PRINT_LLP_REG("LLP.LLP", virt_addr+8); /* link */
                PRINT_LLP_REG("LLP.CTLlow", virt_addr+12); /*control */
                pc302_dma_print_ctl_low(dma, reg);
                PRINT_LLP_REG("LLP.CTLhigh", virt_addr+16); /*block_ts */
                pc302_dma_print_ctl_high(dma, reg);

                llp = (u32)(((descriptor_t *)virt_addr)->hw.link);
                iounmap(virt_addr);
                printk("\n");
            }
            while ((llp != 0) && (llp != head_llp));
        }
    }

    printk("\n    INTERRUPTS:\n");

    PRINT_CH_REG("RawTfr", 
        PC302_DMA_TXFER_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
    PRINT_CH_REG("RawBlock", 
        PC302_DMA_BLOCK_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
    PRINT_CH_REG("RawSrcTran", 
        PC302_DMA_SRC_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
    PRINT_CH_REG("RawDstTran", 
        PC302_DMA_DST_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));
    PRINT_CH_REG("RawErr", 
        PC302_DMA_ERROR_REG_OFFSET(PC302_DMA_INTERRUPT_RAW));

    PRINT_CH_REG("StatusTfr", 
        PC302_DMA_TXFER_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));
    PRINT_CH_REG("StatusBlock", 
        PC302_DMA_BLOCK_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));
    PRINT_CH_REG("StatusSrcTran", 
        PC302_DMA_SRC_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));
    PRINT_CH_REG("StatusDstTran", 
        PC302_DMA_DST_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));
    PRINT_CH_REG("StatusErr", 
        PC302_DMA_ERROR_REG_OFFSET(PC302_DMA_INTERRUPT_STATUS));

    PRINT_CH_REG("MaskTfr", 
        PC302_DMA_TXFER_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_MASK));
    PRINT_CH_REG("MaskBlock", 
        PC302_DMA_BLOCK_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_MASK));
    PRINT_CH_REG("MaskSrcTran", 
        PC302_DMA_SRC_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_MASK));
    PRINT_CH_REG("MaskDstTran", 
        PC302_DMA_DST_TRX_COMPLETE_REG_OFFSET(PC302_DMA_INTERRUPT_MASK));
    PRINT_CH_REG("MaskErr", 
        PC302_DMA_ERROR_REG_OFFSET(PC302_DMA_INTERRUPT_MASK));

    PRINT_REG("StatusInt", PC302_DMA_IRQ_STATUS_REG_OFFSET);
    PRINT_BIT("TFR", PC302_DMA_STATUS_TFR);
    PRINT_BIT("BLOCK", PC302_DMA_STATUS_BLOCK);
    PRINT_BIT("SRCT", PC302_DMA_STATUS_SRCT);
    PRINT_BIT("DSTT", PC302_DMA_STATUS_DSTT);
    PRINT_BIT("ERR", PC302_DMA_STATUS_ERR);

    printk("\n    HANDSHAKING:\n");
    PRINT_CH_REG("ReqSrcReg", 
        PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(PC302_DMA_BURST_TRANSFER));
    PRINT_CH_REG("ReqDstReg", 
        PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(PC302_DMA_BURST_TRANSFER));
    PRINT_CH_REG("SglReqSrcReg", 
        PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(PC302_DMA_SINGLE_TRANSFER));
    PRINT_CH_REG("SglReqDstReg", 
        PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(PC302_DMA_SINGLE_TRANSFER));
    PRINT_CH_REG("LstSrcReg", 
        PC302_DMA_SRC_TRX_REQUEST_REG_OFFSET(PC302_DMA_LAST_TRANSFER));
    PRINT_CH_REG("LstDstReg", 
        PC302_DMA_DST_TRX_REQUEST_REG_OFFSET(PC302_DMA_LAST_TRANSFER));

    printk("\n    MISC:\n");
    PRINT_REG("DmaCfgRe", PC302_DMA_CONFIGURATION_REG_OFFSET);
    PRINT_CH_REG("ChEnReg", PC302_DMA_CHANNEL_ENABLE_REG_OFFSET);
    PRINT_REG("DmaIdReg", PC302_DMA_ID_REG_OFFSET);
    PRINT_REG("DmaTestReg", PC302_DMA_TEST_REG_OFFSET);
    printk("\n");
}

/*****************************************************************************
   Set a debug level for the DMA handle
*****************************************************************************/
void
pc302_dma_set_debug_level(pc302_dma_t dma, 
                          pc302_dma_lvl_t lvl)
{
    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_INFO, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return;
    }

    dma->lvl = lvl;
}

/*****************************************************************************
 * Platform data structures
 *****************************************************************************/

/*!
 * Definition of platform driver functions
 */
static struct platform_driver dmac_driver =
{
    .probe      = pc302_dma_drv_probe, 
    .remove     = pc302_dma_drv_remove, 
    .driver     =
    {
        .name   = CARDNAME, 
    }
};

/*****************************************************************************
   Platform driver probe function
*****************************************************************************/
static int
pc302_dma_drv_probe(struct platform_device *pdev)
{
    int id  = ((struct platform_device *)pdev)->id;
    struct resource *pdata = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    struct resource *r=NULL;
    struct pc302_dma_tag *dma=NULL;
    int ret = SUCCESS;

    dma = kzalloc(sizeof(struct pc302_dma_tag), GFP_KERNEL);
    if (dma == NULL)
    {
        /* No memory */
        PRINTE("Out of memory for device %s\n", pdata->name);
        return -ENOMEM;
    }

    if (pdata == NULL)
    {
        PRINTE("%s: could not allocate device.\n", pdata->name);
        kfree(dma);
        return -EINVAL;
    }

    r = request_mem_region(pdata->start, (pdata->end-pdata->start),
        pdata->name);
    if (r != NULL)
    {
        dma->membase = ioremap(pdata->start, (pdata->end-pdata->start));
        if (dma->membase == NULL)
        {
            release_resource(r);
            kfree(dma);
            ret = -EIO;
        }
    }
    else
    {
        kfree(dma);
        ret = -EBUSY;
    }
    
    dmac[id].dev = pdev;
    dmac[id].dma = dma;

    if (ret != SUCCESS)
    {
         PRINTE("Mapping %s error, hw=%x, virt=%p, size=%d, err=%d\n", 
            pdata->name, pdata->start, dma->membase, (pdata->end-pdata->start), 
            ret);
         return ret;
    }

    /* Set the default debug level */
    dma->lvl = PC302_DMA_DEBUG_LEVEL;
    dma->id = id;

    /* Get a pointer to the underlying device for dma_alloc_coherent calls */
    dma->device = &pdev->dev;
    dma->last_int_channel = 0;

    /* Allocate a spinlock for each engine */
    spin_lock_init(&dma->spinlock);

    /* Initialise engine hardware */
    pc302_dma_hw_initialise(dma);

    platform_set_drvdata(pdev, dma);

    tasklet_init(&dma->tasklet,pc302_dma_do_tasklet,(unsigned long)dma);

    return SUCCESS;
}

/*****************************************************************************
   Platform driver remove function
*****************************************************************************/
static int
pc302_dma_drv_remove(struct platform_device *pdev)
{
    struct pc302_dma_tag *dma = platform_get_drvdata(pdev);
    struct resource *pdata = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    int ret = SUCCESS;

    if (dma != NULL)
    {
        pc302_dma_hw_shutdown(dma);

        ret = pc302_dma_free_irq(dma, 0xFF); /* Free all channels at once */

        iounmap(dma->membase);
        release_mem_region(pdata->start, (pdata->end-pdata->start));

        tasklet_kill(&dma->tasklet);

        kfree(dma);
    }

    platform_set_drvdata(pdev, NULL);

    return ret;
}

/*****************************************************************************
   Allows a client process to register to receive an interrupt for
   the DMAC and DMA channel specified.
*****************************************************************************/
static int
pc302_dma_request_irq(pc302_dma_t dma, 
                      u32 bitField)
{
    int ret = SUCCESS;
    struct resource *r=NULL;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");
    }
    else
    {
        PRINTE("NULL DMA handle / DMA engine not enabled\n");
        return -EINVAL;
    }

    if (dma->allocatedInterrupts == 0)
    {
        /* IRQ has not been requested for this DMA handle. Need to determine
           which DMA (0 or 1) this handle is controlling */
        r = platform_get_resource(dmac[dma->id].dev, IORESOURCE_IRQ, 0);
        if (r == NULL)
        {
            PRINTE("cannot get DMA resource for DMAC%d\n",dma->id);
        }
        else
        {
            ret = request_irq(r->start, &pc302_dma_isr, IRQF_DISABLED, 
                    r->name, dma);
            if (ret)
            {
                PRINTE("cannot assign IRQ for DMAC%d. Ret code = %d\n",
                    dma->id, ret);
            }
        }
    }

    dma->allocatedInterrupts |= bitField;
    return ret;
}

/*****************************************************************************
   Function that allows a client to process to deregister itself from
   receiving an interrupt for the DMA and channel specified.
*****************************************************************************/
static int
pc302_dma_free_irq(pc302_dma_t dma, 
                   u32 bitField)
{
    struct resource *r=NULL;

    if (DMA_ENGINE_ENABLED(dma))
    {
        PRINTD(dma, PC302_DMA_LVL_TRACE, "Enter\n");

        if (dma->allocatedInterrupts)
        {
            dma->allocatedInterrupts &= ~((unsigned)(bitField));
            if (dma->allocatedInterrupts == 0)
            {
                r = platform_get_resource(dmac[dma->id].dev, IORESOURCE_IRQ, 0);
                if (r == NULL)
                {
                    PRINTE("cannot free DMA resource for DMAC%d\n",dma->id);
                }
                else
                {
                    free_irq(r->start, dma);
                }
            }
        }
    }

    return SUCCESS;
}

/*****************************************************************************
   Driver init function
*****************************************************************************/
static int __init dmac_init_module(void)
{
    int ret = SUCCESS;

    ret = platform_driver_register(&dmac_driver);
    if (ret != 0)
    {
        printk(KERN_INFO "%s " CONFIG_LOCALVERSION " " __DATE__ " "  __TIME__
            " failed to load\n", TITLE);
        return ret;
    }

    printk(KERN_INFO "%s " CONFIG_LOCALVERSION " " __DATE__ " "  __TIME__
        " loaded\n", TITLE);

    return ret;
}

/*****************************************************************************
   Driver cleanup function
*****************************************************************************/
static void __exit dmac_cleanup_module(void)
{
    platform_driver_unregister(&dmac_driver);

    printk(KERN_INFO "%s " CONFIG_LOCALVERSION " " __DATE__ " "  __TIME__
        " unloaded\n", TITLE);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("picoChip PC302 DMA Driver");
MODULE_AUTHOR("Andrew Watkins");

/* Module exports functions */
module_init(dmac_init_module);
module_exit(dmac_cleanup_module);

EXPORT_SYMBOL(pc302_dma_setup_direct_xfr);
EXPORT_SYMBOL(pc302_dma_start);
EXPORT_SYMBOL(pc302_dma_abort);
EXPORT_SYMBOL(pc302_dma_release);
EXPORT_SYMBOL(pc302_dma_list_create);
EXPORT_SYMBOL(pc302_dma_list_add);
EXPORT_SYMBOL(pc302_dma_list_clear);
EXPORT_SYMBOL(pc302_dma_list_destroy);
EXPORT_SYMBOL(pc302_dma_setup_list_xfr);
EXPORT_SYMBOL(pc302_dma_enable_int);
EXPORT_SYMBOL(pc302_dma_disable_int);
EXPORT_SYMBOL(pc302_dma_clear_int);
EXPORT_SYMBOL(pc302_dma_get_raw_status);
EXPORT_SYMBOL(pc302_dma_setup_sg);
EXPORT_SYMBOL(pc302_dma_request_transaction);
EXPORT_SYMBOL(pc302_dma_get_dma_handle);
EXPORT_SYMBOL(pc302_dma_dump_regs);
EXPORT_SYMBOL(pc302_dma_set_debug_level);

