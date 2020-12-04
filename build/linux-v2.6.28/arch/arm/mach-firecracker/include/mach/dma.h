/*
 *  include/asm-arm/arch-firecracker/dma.h
 *
 *  BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * Description: See arch/arm/mach-firecracker/dma.c
 *
 * This module provides the interface to the firecracker low level DMA
 * functionality.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */



#ifndef __FIRECRACKER_DMA_H__
#define __FIRECRACKER_DMA_H__


/*
 * This is the platform device platform_data structure
 */
struct plat_firecracker_dma {
	void __iomem	*membase;	/* Virtual base pointer */
};


/*****************************************************************************
3.2.5. firecracker_dma_t
------------------------

This is the context of the driver. It holds all internal state of the driver
and is hidden from client code.
Outside scope.
*****************************************************************************/
typedef struct firecracker_dma_tag *firecracker_dma_t;


/*****************************************************************************
3.2.1. firecracker_dma_xfr_t 
----------------------------

This is a handle representing a dma transfer. Internally it will hold a channel
number and the state of the channel.
*****************************************************************************/
typedef struct firecracker_dma_xfr_tag *firecracker_dma_xfr_t;


/*****************************************************************************
3.2.8. firecracker_dma_list_t
-----------------------------

This is the context of a DMA multi-block list. It holds all internal state of the
list and is hidden from client code.
Outside scope.
*****************************************************************************/
typedef struct firecracker_dma_list_tag *firecracker_dma_list_t;


/*****************************************************************************
3.2.2.2. ahb_master_t
This applies to the CTLx.SMS and CTLx.DMS bits. This selects the bus to use
for transfer.
*****************************************************************************/
typedef enum {
    AHB_MASTER1 = 0,
    AHB_MASTER2 = 1,
    AHB_MASTER3 = 2,
    AHB_MASTER4 = 3
} ahb_master_t;


/*****************************************************************************
3.2.2.3. increment_t
This applies to the CTLx.SINC and DINC bits.
Where an endpoint is NO_CHANGE, facilities like the scatter/gather is disabled.
*****************************************************************************/
typedef enum {
    INCREMENT = 0,
    DECREMENT = 1,
    NO_CHANGE = 2
} increment_t;


/*****************************************************************************
3.2.2.4. tr_width_t 
This applies to the CTLx.SRC_TR_WIDTH and DST_TR_WIDTH bits. This sets the 
transfer width of an endpoint in bits.
*****************************************************************************/
typedef enum {
    TR_WIDTH8 = 0,
    TR_WIDTH16 = 1,
    TR_WIDTH32 = 2,
    TR_WIDTH64 = 3,
    TR_WIDTH128 = 4,
    TR_WIDTH256 = 5
} tr_width_t;


/*****************************************************************************
3.2.2.5. msize_t
This applies to the CTLx.SRC_MSIZE and DST_MSIZE bits. This sets the burst
transaction length where the number of bytes transferred in a burst is
the transfer width x msize.

3.2.2.6. Automatic setting for burst transaction.
The Burst Transaction Length is set according to the alignment of the count
parameter and the chosen transfer width. The burst length is chosen to minimise the
total number of transactions, that is the number of burst transactions plus the 
number of single transactions.
*****************************************************************************/
typedef enum {
    MS_1_TRW    = 0,
    MS_4_TRW    = 1,
    MS_8_TRW    = 2,
    MS_16_TRW   = 3,
    MS_32_TRW   = 4,
    MS_AUTO     = 5         /* See 3.2.2.6 */
} msize_t;


/*****************************************************************************
3.2.3. firecracker_dma_sg_t
---------------------------

This type is used to specify the scatter/gather parameters of a channel. 
This information is used when a transfer is set up and is used in the SGRx
and DSRx registers. Note, these SG settings apply to the entire DMA transfer
i.e. all blocks of a multi block transfer (where enabled).

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


Note the alignment of the sg_stride and sg_count settings for a transfer
affect the automatic Burst Transaction Length settings as the hardware has
to be set up transfer in bursts. 
The size of the SG frame and stride between SG frames must be a multiple 
of the Transfer Width of the endpoint. The sg_count and sg_stride will
be truncated.
*****************************************************************************/
typedef struct {
    unsigned int count;             /* Length of the SG frame in bytes */
    unsigned int interval;          /* Number of bytes between frames */
} firecracker_dma_sg_t;


/*****************************************************************************
3.2.4. firecracker_dma_handshake_t
----------------------------------

This type is used to specify the handshaking of the source or destination 
of a DMA transfer. This data is used to setup the CFGx registers.
If the handshaking is not setup for a transfer (by calling 
firecracker_dma_setup_handshaking), it will default to software handshaking.
*****************************************************************************/
typedef struct {
    unsigned int interface;         /* The handshaking interface to be used
                                     * (0-3) */
    int active_low;                 /* The polarity of the handshaking. 
                                       TRUE for active low. */
} firecracker_dma_handshake_t;


/*****************************************************************************
3.2.6. dma_int_type_t
---------------------
This is a bit-mask that specifies a number of interrupt types.
A number of interrupt types are specified by OR'ing several definitions, 
e.g. "INT_DST_TRANS | INT_ERROR | INT_SRC_TRANS" for transactions and errors.

Note, the order that this enum is defined governs the order in which the 
interrupts are serviced by the firecracker_dma_int_get_xfr function.

3.2.6.1. INT_BLOCK
This interrupt is generated on DMA block transfer completion
to the destination peripheral.
If this interrupt is enabled and the DMA is a multi-block type (either via
multi-block list or auto-reloading), then DMA will wait after transferring
each block for this interrupt to be cleared.

3.2.6.2. INT_DST_TRANSACTION | INT_SRC_TRANSACTION
These interrupts are generated on completion of the last physical bus transfer
to/from the destination/source endpoint that concludes a transaction (single 
or burst). Software or hardware handshaking.
These interrupts would typically be used in software handshaking to generate
a request for the next transaction.

3.2.6.3. INT_ERROR
This flag is set when an ERROR response is received from an AHB slave on the
HRESP bus during a DMA transfer. In addition the transfer is cancelled and
the channel disabled.

3.2.6.4. INT_TRANSFER
This interrupt is generated on DMA transfer completion to the destination 
peripheral.

3.2.6.5. INT_ALL
All of the interrupt types.
*****************************************************************************/
typedef enum {
    INT_BLOCK           = 1,
    INT_DST_TRANSACTION = 2,
    INT_ERROR           = 4,
    INT_SRC_TRANSACTION = 8,
    INT_TRANSFER        = 16,
    INT_ALL             = 255
} dma_int_type_t;


/*****************************************************************************
3.2.2. firecracker_dma_endpoint_t
---------------------------------

This type is used to specify a source or destination of a DMA transfer. At its
simplest, it holds the physical address of the data buffer. This data is used 
to configure the source/destination address and control registers 
(SARx, DARx and CTLx). This information does not control the channel 
configuration or scatter/gather registers (note 3). This structure is used in 
configuring single or multi-block list type transfers as the information 
within is used in each element of the multi-block list.


Notes
1.  Either the source or destination can be the flow controller or nether but 
    not both. If nether is flow controller, the DMA is defaulted to the 
    controller. The flow controller will control the block size transferred. If
    a flow controller is assigned which employs hardware handshaking, 
    the 'count' size specified by firecracker_dma_setup_direct_xfr or 
    firecracker_dma_list_add is ignored.
    A memory endpoint cannot be a flow controller.
    Memory endpoint does not require handshaking (hardware or software) and will
    proceed to transfer immediately without waiting for a transaction request.

2.  If auto-reload is set for either a source or dest. direct transfer (or both),
    the transfer will restart again at the end of the last. The transfer will be
    one-time only if both source and destination auto_reload is false.
    If the one of src/desc is not auto-reload, its physical address will not
    be reloaded and will continue from where it stopped at the end of the last
    block (continuous).
    When applied to multi-block lists, either src or dest (but not both) can be
    auto_reload. Only the first of a multi-block list can be set when its 
    auto_reload flags is true. Thereafter, the endpoint must not be set in the list
    (pass NULL to the firecracker_dma_list_add endpoint param).
    Note, if the first element of the multi-block list is set only (subsequent
    elements are NULL) and the first element does not set the auto_reload flag,
    the endpoint will be in continuous mode where the physical address will
    continue from the end of the last block.

3.  The auto_reload information is set in the configuration register. This is the
    only information passed to the config. register from this structure. Everything
    else go's into the control reg.

4.  Scatter/Gather will not be enabled unless a SG parameters are set by calling
    firecracker_dma_setup_sg.


*****************************************************************************/
typedef struct firecracker_dma_endpoint_tag {
    dma_addr_t dma_addr;            /* Physical address of the data */
    ahb_master_t ahb_master_select; /* The interface layer used for comms */
    int periph_not_mem;             /* TRUE if the endpoint is peripheral 
                                       not memory */
    int flow_controller;            /* TRUE if the endpoint is the 
                                       flow controller */
    int enable_sg;                  /* TRUE if Scatter/gather enabled */
    increment_t addr_inc;           /* Increment/decrement or fixed addr */
    tr_width_t tr_width;            /* The endpoint Transfer Width */
    msize_t msize;                  /* The Burst Transaction size */
    int auto_reload;                /* TRUE if auto-reload the physical 
                                       address at end of block (note 2) */
} firecracker_dma_endpoint_t;


/* flag values used in the parameter to the firecracker_dma_setup_direct_xfr and
 * firecracker_dma_setup_list_xfr
 */
#define CH_PRIOR_MASK               (0x0000000f)
#define CH_PRIOR_SHIFT              (0)
#define CH_PRIOR_0                  (0x00000000)
#define CH_PRIOR_1                  (0x00000001)
#define CH_PRIOR_2                  (0x00000002)
#define CH_PRIOR_3                  (0x00000003)
#define PROTCTL_MASK                (0x000000f0)
#define PROTCTL_SHIFT               (4)
#define PROTCTL_0                   (0x00000000)
#define PROTCTL_1                   (0x00000010)
#define PROTCTL_2                   (0x00000020)
#define PROTCTL_3                   (0x00000030)
#define FIFO_MODE                   (0x00000100)
#define FC_MODE                     (0x00000200)

/* Source/Destination specifiers */
#define SRC 1
#define DST 0


/* DMA programming interface functions. Refer to arch/arm/mach-firecracker/dma.c for 
 * details of these functions 
 */
firecracker_dma_xfr_t firecracker_dma_setup_direct_xfr(
        firecracker_dma_t dma,
        firecracker_dma_endpoint_t *src, firecracker_dma_endpoint_t *dst,
        firecracker_dma_handshake_t *src_handshaking,
        firecracker_dma_handshake_t *dst_handshaking,
        unsigned int count, unsigned int flags, void *cookie);

int firecracker_dma_start(firecracker_dma_xfr_t dma_xfr);

int firecracker_dma_abort(firecracker_dma_xfr_t dma_xfr);

void firecracker_dma_release(firecracker_dma_xfr_t dma_xfr);

void firecracker_dma_set_debug_level(firecracker_dma_t dma, int lvl);


firecracker_dma_list_t firecracker_dma_list_create(
        firecracker_dma_t dma, unsigned int count);

int firecracker_dma_list_add(
        firecracker_dma_list_t list,
        firecracker_dma_endpoint_t *src, firecracker_dma_endpoint_t *dst,
        unsigned int count);

int firecracker_dma_list_clear(firecracker_dma_list_t list);

int firecracker_dma_list_destroy(firecracker_dma_list_t list);

firecracker_dma_xfr_t firecracker_dma_setup_list_xfr(
        firecracker_dma_list_t list,
        firecracker_dma_handshake_t *src_handshaking,
        firecracker_dma_handshake_t *dst_handshaking,
        unsigned int flags, void *cookie);

void firecracker_dma_enable_int(
        firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types);

void firecracker_dma_disable_int(
        firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types);

void firecracker_dma_clear_int(
        firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types);

firecracker_dma_xfr_t firecracker_dma_int_get_xfr(
        firecracker_dma_t dma, dma_int_type_t *interript_type,
        void **cookie);

int firecracker_dma_handle_block_int(
        firecracker_dma_xfr_t dma_xfr, unsigned int *blocks_left);

dma_int_type_t firecracker_dma_get_raw_status(
    firecracker_dma_xfr_t dma_xfr, dma_int_type_t int_types);

int firecracker_dma_setup_sg(
    firecracker_dma_xfr_t dma_xfr,
    firecracker_dma_sg_t *sg, int src_dst);

int firecracker_dma_request_transaction(
        firecracker_dma_xfr_t dma_xfr, int src_dst,
        unsigned int *bytes_left);

firecracker_dma_t firecracker_dma_get_dmac_handle(int controller);

void firecracker_dma_dump_regs(firecracker_dma_t dma);


#endif /* __FIRECRACKER_DMA_H__ */








