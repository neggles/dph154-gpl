/*******************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 ******************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All inquiries to support@picochip.com
 *
 */

/*!
 * \file pc302_dmac.h
 * \brief This module provides the interface to the low level DMA functionality.
 *
 *\verbatim
 *
 * Introduction
 * ------------
 *
 * This is an informal description of the software interface for the PC302 DMA
 * driver. This API allows kernel modules to access the DMA capabilities of the
 * PC302. No user space interface to the DMA driver is provided. The API is
 * loosely based on the PC102 8560 DMA driver interfaces with the addition of
 * the standard Linux ARM framework support.
 * 
 * Glossary
 * --------
 *
 * Transfer -       A DMA transfer is the entire operation of moving data from
 *                  one place to another, from start to finish. It is made up
 *                  of 1 or more blocks, i.e. single vs multi-block.
 *
 * Block -          The transfer of a number of data items. The size of the
 *                  block can be controlled either by the sending or receiving
 *                  end or by the DMA controller itself. Blocks can be scatter/
 *                  gather. A DMA transfer can be multi-block if either a multi-
 *                  block transfer list is used or if auto-reloading at one end
 *                  of the transfer is employed.
 *
 * Transaction -    For efficiency a number of data items can be moved at one
 *                  time. This is also called Burst Transaction. A Block (or
 *                  scatter / gather region) can be a whole number of
 *                  transactions in size in which case it fits neatly into a
 *                  number of burst transactions. Otherwise smaller transactions
 *                  are needed at the end of the block. These single
 *                  transactions transfer a single data item at one time.
 *
 * Transfer Width - The number of bytes in a data item.
 *
 * Handshaking -    In order to progress the movement of data, handshaking
 *                  signals to the DMA hardware tell it when to start a new
 *                  transaction. The handshaking signal can be generated in
 *                  hardware (by a peripheral hardware signal), or by software
 *                  (by setting a bit in a register).
 *
 * Notes:
 *   The references to scatter / gather in these source files are inherited from
 *   the Synopsys documentation. They are used in linked list multi-block
 *   transfers and are equivalent to the term 'Striding' used in other DMA
 *   controllers.
 *
 * References
 * ----------
 *
 * The information in this document needs to be read along side the
 * DesignWare DW_ahb_dmac Databook Version 2.11a April 11, 2008 Synopsys
 *
 * High Level interfaces
 * ---------------------
 *
 * The driver software is layered as follows:
 *
 *                     -----------------------------------
 *                     | One or more client applications |
 *                     -----------------------------------
 *                     |         PC302 DMA API           |
 *                     -----------------------------------
 *                     |         PC302 Hardware          |
 *                     -----------------------------------
 *\endverbatim
 *
 *****************************************************************************/

#ifndef __PC302_DMAC__
#define __PC302_DMAC__

/*******************************************************************************
 * Macros
 ******************************************************************************/

/*!
 *  \brief Main flags used to configure optional transfer parameters
 *
 *  Flag values for setting the channel priority. Used in functions
 *  pc302_dma_setup_direct_xfr() and pc302_dma_setup_list_xfr().
 *  PC302_DMA_CH_PRIOR_7 is the highest priority and
 *  PC302_DMA_CH_PRIOR the lowest.
 */
#define PC302_DMA_CH_PRIOR_0      (0x00000000)
#define PC302_DMA_CH_PRIOR_1      (0x00000001)
#define PC302_DMA_CH_PRIOR_2      (0x00000002)
#define PC302_DMA_CH_PRIOR_3      (0x00000003)
#define PC302_DMA_CH_PRIOR_4      (0x00000004)
#define PC302_DMA_CH_PRIOR_5      (0x00000005)
#define PC302_DMA_CH_PRIOR_6      (0x00000006)
#define PC302_DMA_CH_PRIOR_7      (0x00000007)
/*!
 *  Flag values used to set the channel protection.
 */
#define PC302_DMA_PROTCTL_0       (0x00000000)
#define PC302_DMA_PROTCTL_1       (0x00000010)
#define PC302_DMA_PROTCTL_2       (0x00000020)
#define PC302_DMA_PROTCTL_3       (0x00000030)
/*!
 *  Optional flag used to configure the internal DMA FIFO mode. When set a
 *  transfer will only be requested or started when at least half the FIFO is
 *  empty or full depending on the direction of transfer, or this is the end
 *  of a burst transaction.
 *  When unset (the default) a transfer is requested or started if there is
 *  sufficient space or data available for a single transfer.
 */
#define PC302_DMA_FIFO_WAIT      (0x00000100)
/*!
 *  Optional flag for configuring the DMA Flow Control Mode. When unset (the
 *  default), source transactions are processed as soon as they occur.
 *  When set source transactions are not processed until after a destination
 *  request has been received. The idea of this mode is that only data that can
 *  transferred to the destination channel is read from the source.
 */
#define PC302_DMA_DST_WAIT       (0x00000200)

/*******************************************************************************
 * External Data structures & types
 ******************************************************************************/

/*!
 * \brief This enum defines the flags that can be used to set the debugging
 * level for a DMA
 */
typedef enum
{
    /** Report warning conditions only such as a unexpected return parameter */
    PC302_DMA_LVL_WARNING,

    /** Report normal but significant conditions */
    PC302_DMA_LVL_NOTICE,

    /** Provide informational messages such as external API calls */
    PC302_DMA_LVL_INFO,

    /** Provide debug level messages such as key register values */
    PC302_DMA_LVL_DEBUG,

    /** Provide trace messages such as internal function calls */
    PC302_DMA_LVL_TRACE,

    /** Provide register IO trace messages */
    PC302_DMA_LVL_TRACE_IO
} pc302_dma_lvl_t;

/*!
 * \brief This structure defines the context of the driver.
 * It holds the internal state of the driver and is purposely hidden from
 * client code.
 */
typedef struct pc302_dma_tag *pc302_dma_t;

/*!
 * \brief This structure defines the handle for representing a dma transfer.
 * Internally it will hold a channel number and the state of the channel.
 * It is hidden from the client code.
*/
typedef struct pc302_dma_xfr_tag *pc302_dma_xfr_t;

/*!
 * \brief This structure contains the context of a DMA multi-block list.
 * It holds all internal state of the list and is hidden from client code.
 */
typedef struct pc302_dma_list_tag *pc302_dma_list_t;

/*!
 * \brief This enum defines the type of source or destination endpoint
 */
typedef enum
{
    /** Endpoint type is a source */
    PC302_DMA_SRC,

    /** Endpoint type is a destination */
    PC302_DMA_DST
} pc302_dma_endpoint_type_t;

/*!
 * \brief This enum selects the bus master to use for the transfer.
 * It is used to define the CTLx.SMS and CTLx.DMS bits.
 */
typedef enum
{
    PC302_DMA_MASTER1     = 0x0,
    PC302_DMA_MASTER2     = 0x1,
    PC302_DMA_MASTER3     = 0x2,
    PC302_DMA_MASTER4     = 0x3,
} pc302_master_t;

/*!
 * \brief This enum determines how the address (source or destination) will be
 * incremented after each read or write. It is used to define the CTLx.SINC and
 * DINC bits.
 */
typedef enum
{
    /** Increment the address pointer between reads / writes */
    PC302_DMA_ADDR_INCREMENT = 0x0,

    /** Decrement the address pointer between reads / writes */
    PC302_DMA_ADDR_DECREMENT = 0x1,

    /** Do not increment or decrement the pointer between reads / writes.
        (Scatter / gather is disabled with this option) */
    PC302_DMA_ADDR_NO_CHANGE = 0x2
} pc302_increment_t;

/*!
 * \brief This enum sets the bus width (in bits) to use for the transfer.
 * It is used to define the CTLx.SRC_TR_WIDTH and DST_TR_WIDTH bits.
 */
typedef enum
{
    /** Use a bus width of 8 bits */
    PC302_DMA_TR_WIDTH8 = 0x0,

    /** Use a bus width of 16 bits */
    PC302_DMA_TR_WIDTH16 = 0x1,

    /** Use a bus width of 32 bits */
    PC302_DMA_TR_WIDTH32 = 0x2,

    /** Use a bus width of 64 bits */
    PC302_DMA_TR_WIDTH64 = 0x3
} pc302_tr_width_t;

/*!
 * \brief This enum sets the burst transaction length (in transfer bus widths) to use.
 * The maximum size of the burst is limited by the FIFO size in hardware.
 * It is used to define the CTLx.SRC_MSIZE and DST_MSIZE bits.
 */
typedef enum
{
    /** Use a burst transaction size of 1 transfer bus widths */
    PC302_DMA_MS_1_TRW = 0x0,

    /** Use a burst transaction size of 4 transfer bus widths */
    PC302_DMA_MS_4_TRW = 0x1,

    /** Use a burst transaction size of 8 transfer bus widths */
    PC302_DMA_MS_8_TRW = 0x2,

    /** Use a burst transaction size of 16 transfer bus widths */
    PC302_DMA_MS_16_TRW = 0x3,

    /** Use a burst transaction size of 32 transfer bus widths. This is the maximum
        size permitted */
    PC302_DMA_MS_32_TRW = 0x4,

    /** Automatic setting of the burst transaction. With this option the burst
        transaction Length will be set according to the number of bytes (the
        count) to be transferred and the chosen transfer width. The burst length
        is chosen to minimise the total number of transactions, that is the
        number of burst transactions plus the number of single transactions
        required to complete the transfer. */
    PC302_DMA_MS_AUTO
} pc302_msize_t;

/*!
 * \brief This structure is used to specify the scatter/gather parameters of a
 * channel. This information is required when a transfer is set up and is used
 * in the * SGRx and DSRx registers. Note, these SG settings apply to the entire
 * DMA transfer i.e. all blocks of a multi block transfer (where enabled).
 */
typedef struct
{
    /** Length of the scatter / gather frame in bytes */
    size_t count;

    /** The number of bytes between frames */
    unsigned interval;
} pc302_dma_sg_t;

/*!
 * /brief This structure is used to specify the hardware handshaking for the
 * source or destination of a DMA transfer. The data is used to setup the
 * CFGx registers. If the handshaking is not setup for a transfer (by not
 * calling pc302_dma_setup_handshaking()), the DMA will default to software
 * handshaking.
 */
typedef struct
{
    /** The handshaking interface to be used * (0-(DMA_HANDSHAKING_IFS-1)) */
    unsigned hwInterface;

    /** The polarity of the handshaking, 1 for active low. */
    int active_low;
} pc302_dma_handshake_t;

/*!
 * /brief This enum contains bit flags used to define the interrupt types to be
 * enabled, disabled or queried. Each field is a bit-mask that can be logically
 * ORed together to specify more than one interrupt type.
 */
typedef enum
{
    /** Generate an interrupt on DMA block transfer completion to the
        destination endpoint. If this interrupt is enabled and the DMA is a
        multi-block type (either via multi-block list or auto-reloading), then
        the DMA will wait after transferring each block for this interrupt to be
        cleared. */
    PC302_DMA_INT_BLOCK           = 1,

    /** Generate an interrupt on completion of the last transfer to the
        destination endpoint that concludes a transaction (single or burst).
        It is triggered by software or hardware handshaking, and is typically
        used in software handshaking to generate a request for the next
        transaction. */
    PC302_DMA_INT_DST_TRANSACTION = 2,

    /** Interrupt set when an ERROR response is received by the DMA.
        The transfer is canceled and the channel disabled. */
    PC302_DMA_INT_ERROR           = 4,

    /** Generate an interrupt on DMA block transfer completion from the
        source endpoint. If this interrupt is enabled and the DMA is a
        multi-block type (either via multi-block list or auto-reloading), then
        the DMA will wait after transferring each block for this interrupt to be
        cleared. */
    PC302_DMA_INT_SRC_TRANSACTION = 8,

    /** Interrupt generated on DMA transfer completion to the destination
        endpoint */
    PC302_DMA_INT_TRANSFER        = 16,

    /** Enable all  interrupt types. */
    PC302_DMA_INT_ALL             = 31,
} dma_irq_type_t;

/*!
 * \brief This type is used to specify the source or destination of a DMA
 * transfer
 *
 * At its simplest, it holds the physical address of the data buffer. This data
 * is used to configure the source/destination address and control registers
 * (SARx, DARx and CTLx). This does not control the channel configuration or
 * scatter/gather registers (see note 3). The structure is used in configuring
 * single or multi-block list type transfers as the information within is used
 * in each element of the multi-block list.
 *
 * Notes:
 *
 * 1. Either the source or destination can be the flow controller, or neither
 *  but not both. If neither is the flow controller, the DMA is defaulted to be
 *  the controller. The flow controller will determine the block size
 *  transferred. If a flow controller is assigned which employs hardware
 *  handshaking the 'count' size specified by pc302_dma_setup_direct_xfr() or
 *  pc302_dma_list_add() is ignored. A memory endpoint cannot be a flow
 *  controller. The memory endpoint does not require handshaking (hardware or
 *  software) and will proceed to transfer immediately without waiting for a
 *  transaction request.
 *
 * 2. Auto reload mode and direct transfers. If auto-reload is set for either
 *  the source or destination end points (or both), the transfer will restart
 *  again at the end of the last transaction. The transfer will be one-time only
 *  if both source and destination auto_reload is false. If one of the end
 *  points is not set to auto-reload (but the other is), its physical address
 *  will not be reloaded. Instead the address used will continue from where it
 *  stopped at the end of the last transaction. This is known as continuous
 *  mode.
 *
 * 3. Auto reload and list transfers. When applied to multi-block lists, either
 *  the source or destination (but not both) end points can be set to
 *  auto_reload. Only the first block of a multi-block list can be set when its
 *  auto_reload flags is true. Thereafter the end point must not be set in the
 *  subsequent blocks of the list (pass NULL as the pc302_dma_list_add endpoint
 *  param). If the first element of the multi-block list is the only one with
 *  the end points set but the auto_reload flag is not set, the DMA will be in
 *  continuous mode where the physical address will continue from the end of the
 *  last block.
 *
 * 4. The auto_reload information is set in the configuration register. This is
 *  the only information passed to that register from this structure. All other
 *  data goes into the writing the control register.
 *
 * 5. Scatter/Gather will not be enabled unless the SG parameters are set by
 *  calling pc302_dma_setup_sg().
 */
typedef struct
{
    /** Physical address of the data */
    dma_addr_t dma_addr;

    /** The interface layer used for communications */
    pc302_master_t master;

    /** True if the endpoint is peripheral not memory */
    int periph_not_mem;

    /** True if the endpoint is the flow controller */
    int flow_controller;

    /** True if Scatter/gather enabled */
    int enable_sg;

    /** Increment/decrement or fixed addr */
    pc302_increment_t addr_inc;

    /** The endpoint Transfer Width */
    pc302_tr_width_t tr_width;

    /** The Burst Transaction size */
    pc302_msize_t msize;

    /** True if auto-reload the physical address at end of block */
    int auto_reload;
} pc302_dma_endpoint_t;

/*******************************************************************************
 * Public function declarations
 *****************************************************************************/

/*!
 * \brief Set up a DMA transfer between source and destination.
 *
 * The DMA set up by this call will not start transferring data until
 * pc302_dma_start() is called.
 *
 * The data structure pointed to by the src and dst parameters are copied
 * by the call so can be destroyed once the call returns.
 *
 * The DMA channel set up by this call will have:
 *  - All interrupts disabled
 *  - Channel disabled but not suspended
 *  - Source and Destination address set to the parameters
 *  - Single block transfer mode
 *  - Transfer size set to count
 *  - Source and Destination Master interface set to the parameters
 *  - Transfer Type and Flow control set according to parameters
 *  - No Scatter or Gather.
 *  - Source/Dest Burst Length set according to parameters
 *  - Source/Dest address increment set according to parameters
 *  - Source/Dest Transfer Width set according to parameters
 *  - Protocol Control set according to flags
 *  - Transfer FIFO mode set according to flags
 *  - Data pre-fetching is set according to flags
 *  - Automatic source/destination reload set by the parameters
 *  - Channel priority set according to flags
 *  - Source/Dest handshaking setup according to parameters.
 *
 * If neither the src or dst are auto-reload, the transfer setup by this call is
 * once only and will stop when the data has been transferred. Multi-block
 * transfers will wait between blocks if the block interrupt is enabled.
 *
 * The 'count' parameter is used in one of three ways:
 *
 * 1. If neither end of the transfer is a flow controller (the DMA engine is
 *    flow controller), the count parameter is used to set the block size in the
 *    DMA control register.
 *
 * 2. If one end of the transfer is the flow controller and has hardware
 *    handshaking, the count is ignored as the flow controller will control the
 *    amount of data in the block.
 *
 * 3. If one end of the transfer is a flow controller but software handshaking
 *    is used, the count parameter is ignored. In this situation client software
 *    may not know the size of the block at the point when the transfer is
 *    setup. The bytes_left parameter of pc302_dma_request_transaction function
 *    is used to tell the driver the data left in the block when each
 *    transaction is requested. pc302_dma_request_transaction() can then decide
 *    the type of transaction to request.
 *
 * \param dma The dma handle
 * \param src Transfer source details
 * \param dst Transfer destination details
 * \param src_handshaking The handshaking parameters for transfer source or NULL
 *                      if hardware handshaking is not used
 * \param dst_handshaking The handshaking parameters for transfer destination or
 *                      NULL if hardware handshaking is not used.
 * \param count The number of bytes to transfer. If either end of the transfer
 *              is a hardware handshaking flow controller, this is ignored.
 * \param flags A logical OR of one or more of the following flags:
 * \verbatim
                 PC302_DMA_PROTCTL_0   - Sets the PROTCTL bits (only 1 of these
                 PC302_DMA_PROTCTL_1     may be selected)
                 PC302_DMA_PROTCTL_2
                 PC302_DMA_PROTCTL_3
                 PC302_DMA_FIFO_MODE   - Sets the FIFO mode
                 PC302_DMA_FC_MODE     - Sets the Flow Control mode
                 PC302_DMA_CH_PRIOR_0  \
                 PC302_DMA_CH_PRIOR_1
                 PC302_DMA_CH_PRIOR_1  - Sets the channel priority
                 PC302_DMA_CH_PRIOR_2    from PC302_DMA_CH_PRIOR_0
                 PC302_DMA_CH_PRIOR_3    (lowest) to
                 PC302_DMA_CH_PRIOR_4    PC302_DMA_CH_PRIOR_7
                 PC302_DMA_CH_PRIOR_5    (highest)
                 PC302_DMA_CH_PRIOR_6
                 PC302_DMA_CH_PRIOR_7  /
 * \endverbatim
 * \param handler The handler function to call on interrupt
 * \param cookie Pointer to be returned as client data in the callback function
 *
 * \return Handle to the new transfer created or NULL if something went wrong.
 */
__must_check pc302_dma_xfr_t
pc302_dma_setup_direct_xfr(pc302_dma_t dma,
			   pc302_dma_endpoint_t *src,
			   pc302_dma_endpoint_t *dst,
                           pc302_dma_handshake_t *src_handshaking,
                           pc302_dma_handshake_t *dst_handshaking,
                           size_t count,
                           u32 flags,
                           int ( *handler )( void *cookie,
                                             int errno ),
                           void *cookie);

/*!
 * \brief Start a DMA transfer on a channel previously set up with the
 * pc302_dma_setup_direct_xfr() or pc302_dma_setup_list_xfr() functions.
 *
 * Once started, the transfer is locked and will not be modifiable. This
 * includes a DMA multi-block list that is associated with a transfer. Transfers
 * will continue to completion asynchronously. Once finished, they can be
 * restarted.
 *
 * \param dma_xfr The transfer handle.
 *
 * \return OK on success, EINVAL if the transfer is already running
 */
__must_check int
pc302_dma_start(pc302_dma_xfr_t dma_xfr);

/*!
 * \brief Abort a DMA transfer.
 *
 * Once aborted, the transfer can be restarted. Also the transfer can be
 * modified once aborted. If the transfer is restarted, it will continue from
 * the start of the single transfer buffer or multi-block list.
 *
 * \param dma_xfr The transfer handle.
 *
 * \return OK on success, or EINVAL if the transfer is not running
 */
__must_check int
pc302_dma_abort(pc302_dma_xfr_t dma_xfr);

/*!
 * \brief Stop a DMA transfer and release resources.
 *
 * Once released, the transfer can not be restarted and has to be setup once
 * more. The transfer handle will become invalid on exit of this function.
 *
 * \param dma_xfr The transfer handle.
 *
 * \return OK on success, or the appropriate error code
 */
__must_check int
pc302_dma_release(pc302_dma_xfr_t dma_xfr);

/*!
 * \brief Set a debug level for the DMA handle
 *
 * Default debug levels for all DMA handles may be specified at compile time
 * using the macro CONFIG_PC302_DMA_DEBUG. This function will change the debug
 * level for the DMA specified.
 *
 * \param dma The dma handle
 * \param lvl The debug level to apply. Can be any of pc302_dma_lvl_t
 */
void
pc302_dma_set_debug_level(pc302_dma_t dma,
                          pc302_dma_lvl_t lvl);

/*!
 * \brief Allocate the list for multi-block transfers.
 *
 * The heap memory is utilised and the list is initialised. The length of the
 * list allocated is set at the calling time.
 *
 * \param dma The DMA handle
 * \param count The maximum number of elements to the list.
 *
 * \return Handle of the newly allocated list
 */
__must_check pc302_dma_list_t
pc302_dma_list_create(pc302_dma_t dma,
                      size_t count);

/*!
 * \brief Add an entry to the end of a DMA multi-block list.
 *
 * The entry added to the list specifies a simple block transfer. The data
 * pointed to by the src and dst parameters are copied by the call so can be
 * destroyed once the call returns. This function will not modify a list that is
 * currently in use with a DMA operation.
 *
 * The first element of the list must have both src and dst endpoints. If either
 * endpoint (not both) is to be continuous, subsequent elements need to set the
 * endpoint to NULL. If either endpoint (not both) is to be auto-reload, the
 * first element needs to set the auto_reload flag and subsequent elements need
 * to set the endpoint to NULL.
 *
 * Note: Each block of the list can set a transfer width. Transfer width affects
 * the setting of scatter/gather which is set globally for the transfer.
 * Scatter / gather is setup in the hardware based on the transfer widths of the
 * first element of the block list. It is recommended that subsequent blocks use
 * the same transfer width (if SG is enabled), otherwise there will be undefined
 * effects.
 *
 * \param list The DMA multi-block list to add a new block to
 * \param src The source DMA endpoint parameters, or NULL if auto-reload or
 *            continuous modes selected
 * \param dst The destination endpoint parameters,or NULL if auto-reload or
 *            continuous modes selected
 * \param src_handshaking The handshaking parameters for transfer source or NULL
 *                      if hardware handshaking is not used
 * \param dst_handshaking The handshaking parameters for transfer destination or
 *                      NULL if hardware handshaking is not used.
 * \param count The number of bytes to transfer. If either end of the transfer
 *              is a hardware flow controller this parameter is ignored.
 *
 * \return
 * -       ENOMEM The list is full, no more items can be added
 * -       EBUSY  If the list is in use with a DMA transfer
 * -       ENOMEM The list is full, no more items can be added
 * -       EBUSY  If the list is in use with a DMA transfer
 * -       EINVAL The combination of source and destination endpoints is invalid
 */
__must_check int
pc302_dma_list_add(pc302_dma_list_t list,
                   pc302_dma_endpoint_t *src,
                   pc302_dma_endpoint_t *dst,
                   pc302_dma_handshake_t *src_handshaking,
                   pc302_dma_handshake_t *dst_handshaking,
                   size_t count);

/*!
 * \brief Reset a list to the state that it was in just after it was created
 * with pc302_dma_list_create().
 *
 * This function does not free the memory associated by the list, and it will
 * not modify a list that is currently in use with a DMA operation.
 *
 * \param list The DMA list returned from pc302_dma_list_create
 *
 * \return 0 on success, or EBUSY If the list is in use with a DMA transfer
 */
__must_check int
pc302_dma_list_clear(pc302_dma_list_t list);

/*!
 * \brief Frees the resources allocated by a list.
 *
 * Lists need to be freed after use to avoid a memory leak. This function will
 * not modify a list that is currently in use with a DMA operation.
 *
 * \param list The DMA list returned from pc302_dma_list_create()
 *
 * \return 0 on success, OR EBUSY If the list is in use with a DMA transfer
 */
__must_check int
pc302_dma_list_destroy(pc302_dma_list_t list);

/*!
 * \brief Set up a multi-block DMA transfer.
 *
 * The transfer is controlled by the list provided. The DMA list can be modified
 * once this function has been called but when the DMA operations is started
 * (by calling pc302_dma_start) the list will be locked and any attempt to
 * modify it will not be allowed. Data transfer is not started until the
 * pc302_dma_start() is called. The transfer setup by this call is multi-block
 * only and will stop when the end of the multi-block list is reached. In
 * addition the transfer will wait between blocks if the block interrupt is
 * enabled. Source and destination handshaking is setup according to the
 * parameters.
 *
 * \param list The DMA multi-block list containing the data blocks
 *             to be transferred.
 * \param src_handshaking The handshaking parameters for transfer source or NULL
 *                      if hardware handshaking is not used
 * \param dst_handshaking The handshaking parameters for transfer destination or
 *                      NULL if hardware handshaking is not used.
 * \param flags A logical OR of one or more flags:
 * \verbatim
              PC302_DMA_PROTCTL_0   - Sets the PROTCTL bits (only one of these)
              PC302_DMA_PROTCTL_1
              PC302_DMA_PROTCTL_2
              PC302_DMA_PROTCTL_3
              PC302_DMA_FIFO_MODE   - Sets the FIFO mode
              PC302_DMA_FC_MODE     - Sets the Flow Control mode
              PC302_DMA_CH_PRIOR_0  \
              PC302_DMA_CH_PRIOR_1
              PC302_DMA_CH_PRIOR_1  - Sets the channel priority
              PC302_DMA_CH_PRIOR_2    from PC302_DMA_CH_PRIOR_0
              PC302_DMA_CH_PRIOR_3    (lowest) to
              PC302_DMA_CH_PRIOR_4    PC302_DMA_CH_PRIOR_7
              PC302_DMA_CH_PRIOR_5    (highest)
              PC302_DMA_CH_PRIOR_6
              PC302_DMA_CH_PRIOR_7  /
 * \endverbatim
 * \param handler (DMA handler function to call on interrupt
 * \param cookie Pointer to be returned as client data in callback function
 *
 * \return
 * -       0 on success, or
 * -       ENOMEM No enough resources to complete the call
 * -       EBUSY  If the list is already in use with a DMA transfer
 * -       EINVAL Combination of parameters is illegal.
 */
__must_check pc302_dma_xfr_t
pc302_dma_setup_list_xfr(pc302_dma_list_t list,
                         pc302_dma_handshake_t *src_handshaking,
                         pc302_dma_handshake_t *dst_handshaking,
                         u32 flags,
                         int ( *handler )( void *cookie,
                                           int errno ),
                         void *cookie);

/*!
 * \brief Get the handle of specified of the PC302 DMA.
 *
 * This handle is to be used as a parameter to all other driver API functions.
 *
 *\param dmaNumber The DMA number, (0 or 1)
 *
 * return DMA handle, or NULL on error
 */
__must_check pc302_dma_t
pc302_dma_get_dma_handle(unsigned dmaNumber);

/*!
 * \brief Enable interrupt generation of a number of types of interrupt on a DMA
 * transfer.
 *
 * \param dma_xfr The transfer handle.
 * \param irq_types The types of interrupt, any logical OR combination of
 *                 bit field in enum dma_irq_type_t
 * \return 0 on success, or EINVAL on error
 */
__must_check int
pc302_dma_enable_int(pc302_dma_xfr_t dma_xfr,
                     unsigned irq_types);

/*!
 * \brief Disable interrupt generation of a number of types of interrupt on a
 * DMA transfer.
 *
 * \param dma_xfr The transfer handle.
 * \param irq_types The types of interrupt, any logical OR combination of
 *                  bit field in enum dma_irq_type_t
 * \return 0 on success, or EINVAL on error
 */
__must_check int
pc302_dma_disable_int(pc302_dma_xfr_t dma_xfr,
                      unsigned irq_types);

/*!
 * \brief Clears a number of interrupt status bits, ready for new interrupt
 * generation.
 *
 * \param dma_xfr The transfer handle.
 * \param irq_types The types of interrupt, any logical OR combination of
 *                  bit field in enum dma_irq_type_t
 */
void
pc302_dma_clear_int(pc302_dma_xfr_t dma_xfr,
                    unsigned irq_types);

/*!
 * \brief Get the raw status of a DMA transfer
 *
 * This function can be used to see if one or more of the specified interrupts
 * have become set. The value returned is a logical OR of the dma_irq_type_t
 * type, and can by used by the client application as a polling function.
 *
 * \param dma_xfr The transfer handle.
 * \param irq_types The types of interrupt, any logical OR combination of
 *                  bit field in enum dma_irq_type_t
 *
 * \return
 * -       A bit mask where a '1' bit indicates the status type is set.
 * -       Any combination irq_types is permitted
 */
__must_check unsigned
pc302_dma_get_raw_status(pc302_dma_xfr_t dma_xfr,
                         unsigned irq_types);

/*!
 * \brief This function is used to setup the scatter or gather parameters of a
 * DMA transfer.
 *
 * If SG is not setup, the DMA transfer defaults to switching the SG capability
 * off. The following diagram and note describes how the parameters in
 * pc302_dma_sg_t configure the scatter / gather.
 * \verbatim
  
                        |                   |
                        |                   |
                        |-------------------|   ^
                        |    frame 0        |   |-- count (bytes)
                     ^  |-------------------|   v
         interval  --|  |                   |
          (bytes)    |  |                   |
                     |  |                   |
                     |  |                   |
                     v  |-------------------|
                        |    frame 1        |
                        |-------------------|
                        |                   |
                        |                   |
 * \endverbatim
 * Note the alignment of the interval (sg_stride) and count (sg_count) settings
 * for a transfer affect the automatic Burst Transaction Length settings as the
 * hardware has to be set up for transfer in bursts. Both count (the number of
 * bytes to scatter or gather) and stride (the interval between frames) must be
 * a multiple of the Transfer Width of the endpoint. To achieve this the
 * sg_count and sg_stride will values will be truncated as necessary.
 *
 * \param dma_xfr The transfer handle.
 * \param sg The scatter / gather  parameters
 * \param src_dst 1 for source, and 0 for destination scatter / gather.
 *
 * \return 0 on success, or EBUSY if the DMA transfer is in progress and so
 *         cannot be modified at this time
 *
 */
__must_check int
pc302_dma_setup_sg(pc302_dma_xfr_t dma_xfr,
                   pc302_dma_sg_t *sg,
                   pc302_dma_endpoint_type_t src_dst);

/*!
 * \brief Request a new transaction on the source or destination of a transfer.
 *
 * The DMA endpoint (source or destination) needs to be configured for software
 * handshaking, otherwise this function has no effect. For flow controlling
 * endpoints the type of transaction, burst, single or last is calculated from
 * the bytes_left parameter. For non-flow controlling endpoints the type of
 * transfer is calculated by the hardware (the endpoint enters the single
 * transaction region automatically).
 *
 * Notes:
 *
 * 1. When requesting transactions of a flow controlling endpoint the bytes_left
 * parameter can be set to a high value (say 1000) if it is not known how big
 * the block is. This will cause burst endpoints.
 *
 * 2. Once the size of the block is known the bytes_left should be set
 * appropriately. The bytes_left will be updated by the function and the new
 * updated value should be used in the next call to this function.
 *
 * 3. Once the block size is known client code should continue to call this
 * function with the value of bytes_left returned from the previous call. The
 * call must not revert to setting a high value (as in 1).
 *
 * 4. Client code should not call this function with zero bytes left if the last
 * call specified many bytes left (as in 1). The reason is that the function
 * needs to have prior knowledge of the end of the block so that it can set the
 * last transaction flag on the last transaction.
 *
 * 5. The bytes_left must be divisible by the transfer width of the source and
 * destination endpoints.
 *
 * 6. Burst transactions are chosen up to the point where the last burst
 * transaction would spill over the end of the DMA buffer (size indicated by
 * 'count'). Then single transactions are chosen for the remaining data.
 * Finally, a last transaction is chosen for the final transaction.
 *
 * \param dma_xfr The transfer handle.
 * \param src_dst 1 for source and 0 for destination
 * \param bytes_left The number of bytes left to transfer in the block. This is
 *                   used only for flow controlling endpoints and is updated
 *                   when the function returns. When this reached zero, no more
 *                   transactions should be requested for this endpoint.
 * \return
 * -       0 on success, or
 * -       EBUSY The DMA transfer is not in progress and so transaction cannot
 *               be requested at this time.
 * -       EFAULT Something went wrong and the driver did not keep track of the
 *                state of the hardware.
 * -       EINVAL The transfer is not running.
 */
__must_check int
pc302_dma_request_transaction(pc302_dma_xfr_t dma_xfr,
                              pc302_dma_endpoint_type_t src_dst,
                              unsigned *bytes_left);

/*!
 * \brief Output all DMA configuration registers for all channels, including the
 * information held in multi-block lists.
 *
 * \param dma The DMA handle.
 */
void
pc302_dma_dump_regs(
    pc302_dma_t dma);

#endif /* __PC302_DMAC__ */

