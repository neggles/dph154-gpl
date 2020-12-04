/*
 * This file contains the PicoArray Crypto SRAM driver functions to facilitate IPsec crypto.
 *
 * File: picosram.c
 * Author: Dean Jenkins <djenkins@mvista.com>
 *
 * 2008 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/* kernel related headers */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

/* Kernel crypto related headers */
#include <linux/crypto.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/aes.h>
#include <crypto/sha.h>

#include <asm/io.h>

#include "picocrypto.h"

/******************
 * local #defines *
 ******************/

#undef USE_MEMCPY_FOR_SRAM
#define USE_MEMCPY_FOR_SRAM	1

#undef DEBUG_PDU_MANAGEMENT
/* #define DEBUG_PDU_MANAGEMENT	1	*/

#undef DEBUG_PDU_MANAGEMENT_NO_ROOM
/* #define DEBUG_PDU_MANAGEMENT_NO_ROOM	1 */

#undef DEBUG_BDR_MANAGEMENT
/* #define DEBUG_BDR_MANAGEMENT	1	*/

#undef DEBUG_SRAM
/* #define DEBUG_SRAM	1	*/

#undef VERBOSE_SRAM
/* #define VERBOSE_SRAM	1	*/

#ifdef DEBUG_SRAM
#define SRAM_DEBUG(fmt,args...)	printk( fmt ,##args )
#else
#define SRAM_DEBUG(fmt,args...)
#endif

#define SRAM_INFO(fmt,args...)	printk( fmt ,##args )
#define SRAM_ERROR(fmt,args...)	printk( fmt ,##args )

static void copy_all_context_structures( struct private_driver_data *pdata );

/****************************
 * SRAM Interface functions *
 ****************************/

/*
 * SRAM Memory Map
 *
 * header block
 * context structure block 
 * BD ring #1
 * BD ring #2
 * BD ring #3
 * BD ring #4
 * buffer pool
 */

/*
 * Writes 4 bytes to the SRAM on a 4 byte boundary
 * Inputs:
 * 	base = virtual address
 *	offset = offset from base address
 *	data = data to be written 
 * Outputs:
 *	writes information into the PicoArray SRAM
 * Return:
 *	void 
 */
inline static void sram_write_u32( u32 base, u32 offset, u32 data )
{
	wmb();
	iowrite32( cpu_to_le32( data ), (base + offset) );
}


/*
 * Writes 4 bytes to the SRAM on a 4 byte boundary always in SRAM little-endian
 * Used for writing crypto data + keys to the SRAM
 * Inputs:
 * 	base = virtual address
 *	offset = offset from base address
 *	data = data to be written 
 * Outputs:
 *	writes information into the PicoArray SRAM
 * Return:
 *	void 
 */
inline static void sram_write_u32_le( u32 base, u32 offset, u32 data )
{
	wmb();
	iowrite32( data, (base + offset) );
}


/*
 * Writes 2 bytes to the SRAM on a 2 byte boundary
 * Inputs:
 * 	base = virtual address
 *	offset = offset from base address
 *	data = data to be written 
 * Outputs:
 *	writes information into the PicoArray SRAM
 * Return:
 *	void 
 */
inline static void sram_write_u16( u32 base, u32 offset, u16 data )
{
	wmb();
	iowrite16( cpu_to_le16( data ), (base + offset) );
}


/*
 * Writes a bytes to the SRAM on a byte boundary
 * Inputs:
 * 	base = virtual address
 *	offset = offset from base address
 *	data = data to be written 
 * Outputs:
 *	writes information into the PicoArray SRAM
 * Return:
 *	void 
 */
inline static void sram_write_u8( u32 base, u32 offset, u8 data )
{
	wmb();
	iowrite8( data, (base + offset) );
}


/*
 * Reads 4 bytes from the SRAM on a 4 byte boundary
 * Inputs:
 * 	base = virtual address
 *	offset = offset from base address
 * Outputs:
 *	Reads information from the PicoArray SRAM
 * Return:
 *	4 byte data from the SRAM 
 */
inline static u32 sram_read_u32( u32 base, u32 offset )
{
	rmb();
	return le32_to_cpu( ioread32( ( base + offset ) ) );
}


/*
 * Reads 4 bytes from the SRAM on a 4 byte boundary always in SRAM little-endian
 * Used for reading crypto data + keys from the SRAM
 * Inputs:
 * 	base = virtual address
 *	offset = offset from base address
 * Outputs:
 *	Reads information from the PicoArray SRAM
 * Return:
 *	4 byte data from the SRAM 
 */
inline static u32 sram_read_u32_le( u32 base, u32 offset )
{
	rmb();
	return ioread32( ( base + offset ) );
}


/*
 * Reads 2 bytes from the SRAM on a 2 byte boundary
 * Inputs:
 * 	base = virtual address
 *	offset = offset from base address
 * Outputs:
 *	Reads information from the PicoArray SRAM
 * Return:
 *	2 byte data from the SRAM 
 */
inline static u16 sram_read_u16( u32 base, u32 offset )
{
	rmb();
	return le16_to_cpu( ioread16( ( base + offset ) ) );
}


/*
 * Reads a byte from the SRAM on a byte boundary
 * Inputs:
 * 	base = virtual address
 *	offset = offset from base address
 * Outputs:
 *	Reads information from the PicoArray SRAM
 * Return:
 *	a byte data from the SRAM 
 */
inline static u8 sram_read_u8( u32 base, u32 offset )
{
	rmb();
	return ioread8( base + offset );
}


/* Debug support function */
static void sram_hexdump( char *name, u32 buf_address, u32 len)
{
	u32 pos;
	u32 data;

	SRAM_INFO( KERN_INFO "%s: address 0x%08X: len %d\n", name, buf_address, len );

	/* step up in 4 byte steps */
	for ( pos = 0; pos < len; pos += 4 ) {
		data = sram_read_u32( buf_address, pos );
		SRAM_INFO( KERN_INFO "0x%08X: 0x%04X: 0x%08X\n", buf_address + pos, pos, data );
	}

}


/* Debug function */
void dump_sram_structures( struct private_driver_data *pdata )
{
	/* Note dump will be modified by interruptting decrypt, encrypt calls and async events */

	SRAM_INFO( KERN_INFO "Dumping the SRAM structures\n" );

	sram_hexdump( "Header", pdata->header_address, SRAM_HEADER_SIZE );

	sram_hexdump( "Context", pdata->context_info.context_base_address,
		SRAM_CTX_SIZE * SRAM_CTX_MAX_NUMBER );

	sram_hexdump( "Context shadow", (u32) &pdata->context_info.shadow[ 0 ],
		sizeof(pdata->context_info.shadow) );

	sram_hexdump( "Ring 1 info", (u32) &pdata->bdr_info[ 0 ], sizeof( pdata->bdr_info[ 0 ] ) );
	sram_hexdump( "Ring 1", pdata->bdr_info[ 0 ].bdr_base_address, SRAM_BD_SIZE * SRAM_BD_RING1_NUMBER );
	sram_hexdump( "Ring 2 info", (u32) &pdata->bdr_info[ 1 ], sizeof( pdata->bdr_info[ 1 ] ) );
	sram_hexdump( "Ring 2", pdata->bdr_info[ 1 ].bdr_base_address, SRAM_BD_SIZE * SRAM_BD_RING2_NUMBER );
	sram_hexdump( "Ring 3 info", (u32) &pdata->bdr_info[ 2 ], sizeof( pdata->bdr_info[ 2 ] ) );
	sram_hexdump( "Ring 3", pdata->bdr_info[ 2 ].bdr_base_address, SRAM_BD_SIZE * SRAM_BD_RING3_NUMBER );
	sram_hexdump( "Ring 4 info", (u32) &pdata->bdr_info[ 3 ], sizeof( pdata->bdr_info[ 3 ] ) );
	sram_hexdump( "Ring 4", pdata->bdr_info[ 3 ].bdr_base_address, SRAM_BD_SIZE * SRAM_BD_RING4_NUMBER );

	sram_hexdump( "PDU pool info", (u32) &pdata->pdu_info, sizeof( pdata->pdu_info ) );
	sram_hexdump( "PDU pool data", pdata->pdu_info.pdu_pool_base_address, pdata->pdu_info.pdu_pool_size );

}


/* Debug function */
char *sram_confirmed_state_text( u32 cnf_state )
{
	char *ptext = NULL;

	switch ( cnf_state ) {
	case HEADER_CNF_STATE_READY:
		ptext = "READY";
		break;

	case HEADER_CNF_STATE_CLOSED:
		ptext = "CLOSED";
		break;

	case HEADER_CNF_STATE_OPENED:
		ptext = "OPENED";
		break;

	default:
		ptext = "INVALID";
		break;
	}

	return ( ptext );
}



/*
 * Perform the CLOSE procedure
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	writes information into the PicoArray SRAM Request State
 *	and reads the Confirmed State
 * Return:
 *	0 for success or 1 for failure or 2 for already in correct state (READY or CLOSED).
 */
u8 request_sram_pico_close( struct private_driver_data *pdata )
{
	u32 cnf_state;
	u8 error = 0;

	/* grab the mutex to prevent clash */
	mutex_lock( &pdata->pico_state_mutex );

	SRAM_DEBUG( KERN_INFO "Pending CLOSE_REQ\n");

	/* read the current PicoArray Confirmed State */
	cnf_state = sram_read_u32( pdata->header_address, SRAM_HEADER_CNF_STATE_OFFSET );
	SRAM_DEBUG( KERN_INFO "PicoArray Confirmed State was %08X: %s\n",
		cnf_state, sram_confirmed_state_text( cnf_state ) );

	switch ( cnf_state ) {
	case HEADER_CNF_STATE_READY:
	case HEADER_CNF_STATE_CLOSED:
		/* do nothing */
		error = 2;
		break;

	case HEADER_CNF_STATE_OPENED:
		/* the picoArray is in the OPENED state so */
		/* request the PicoArray enters the closed state */
		sram_write_u32( pdata->header_address, SRAM_HEADER_REQ_STATE_OFFSET, HEADER_REQ_STATE_CLOSE_REQ );

		/* wait a while to time-out the CLOSED confirmation */
		msleep( CNF_STATE_TIMEOUT );
 
		/* now read the new Confirmed State */
		cnf_state = sram_read_u32( pdata->header_address, SRAM_HEADER_CNF_STATE_OFFSET );
		SRAM_DEBUG( KERN_INFO "PicoArray new Confirmed State is %08X: %s\n",
			cnf_state, sram_confirmed_state_text( cnf_state ) );

		if ( cnf_state == HEADER_CNF_STATE_CLOSED ) {
			SRAM_DEBUG( KERN_INFO "CLOSED confirmed\n");
		} else {
			SRAM_ERROR( KERN_ERR "ERROR: Failed to put the PicoArray into the CLOSED state: state is %08X: %s\n",
				cnf_state, sram_confirmed_state_text( cnf_state ) );
#ifndef PICO_FIRMWARE_NOT_AVAILABLE
			error = 1;
#endif
		}

		break;

	default:
		SRAM_ERROR( KERN_ERR "ERROR: PicoArray Confirmed State is invalid: %08X: %s\n",
			cnf_state, sram_confirmed_state_text( cnf_state ) );
#ifndef PICO_FIRMWARE_NOT_AVAILABLE
		error = 1;
#endif
		break;
	}

	/*
	 * write NO_REQ to the Request State so that the PicoArray will see the next request
	 */
	sram_write_u32( pdata->header_address, SRAM_HEADER_REQ_STATE_OFFSET, HEADER_REQ_STATE_NO_REQ );

	/* wait a while to allow the picoArray to see it */
	msleep( CNF_STATE_TIMEOUT );

	SRAM_DEBUG( KERN_INFO "Completed CLOSE_REQ\n");

	mutex_unlock( &pdata->pico_state_mutex );

	return ( error );
}


/*
 * Perform the OPEN procedure
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	writes information into the PicoArray SRAM Request State
 *	and reads the Confirmed State
 * Return:
 *	0 for success or 1 for failure or 2 for already in the OPENED state.
 */
u8 request_sram_pico_open( struct private_driver_data *pdata )
{
	u32 cnf_state;
	u8 error = 0;
	u32 version;

	/* grab the mutex to prevent clash */
	mutex_lock( &pdata->pico_state_mutex );

	SRAM_DEBUG( KERN_INFO "Pending OPEN_REQ\n");

	copy_all_context_structures( pdata );
	
	/* read the current PicoArray Confirmed State */
	cnf_state = sram_read_u32( pdata->header_address, SRAM_HEADER_CNF_STATE_OFFSET );
	SRAM_DEBUG( KERN_INFO "PicoArray Confirmed State was %08X: %s\n",
		cnf_state, sram_confirmed_state_text( cnf_state ) );

	switch ( cnf_state ) {
	case HEADER_CNF_STATE_READY:
	case HEADER_CNF_STATE_CLOSED:
		/* if in the READY or CLOSED confirmed states then */
		/* request the PicoArray enters the open state */
		sram_write_u32( pdata->header_address, SRAM_HEADER_REQ_STATE_OFFSET, HEADER_REQ_STATE_OPEN_REQ );

		/* wait a while to time-out the OPENED confirmation */
		msleep( CNF_STATE_TIMEOUT );
 
		/* now read the new Confirmed State */
		cnf_state = sram_read_u32( pdata->header_address, SRAM_HEADER_CNF_STATE_OFFSET );
		SRAM_DEBUG( KERN_INFO "PicoArray new Confirmed State is %08X: %s\n",
			cnf_state, sram_confirmed_state_text( cnf_state ) );

		if ( cnf_state == HEADER_CNF_STATE_OPENED ) {
			SRAM_DEBUG( KERN_INFO "OPENED confirmed\n");
		} else {
			SRAM_ERROR( KERN_ERR "ERROR: Failed to put the PicoArray into the OPENED state: state is %08X: %s\n",
				cnf_state, sram_confirmed_state_text( cnf_state ) );
#ifndef PICO_FIRMWARE_NOT_AVAILABLE
			error = 1;
#endif
		}

		break;

	case HEADER_CNF_STATE_OPENED:
		/* do nothing */
		error = 2;
		break;

	default:
		SRAM_ERROR( KERN_ERR "ERROR: PicoArray Confirmed State is invalid: %08X: %s\n",
			cnf_state, sram_confirmed_state_text( cnf_state ) );
#ifndef PICO_FIRMWARE_NOT_AVAILABLE
		error = 1;
#endif
		break;
	}

	if ( ( error == 0 ) || ( error == 2 ) ) {
		/* read the PicoArray firmware version */
		version = sram_read_u32( pdata->header_address, SRAM_HEADER_VERSION_OFFSET );
		SRAM_DEBUG( KERN_INFO "PicoArray version is %08X\n", version );
	}

	/*
	 * write NO_REQ to the Request State so that the PicoArray will see the next request
	 */
	sram_write_u32( pdata->header_address, SRAM_HEADER_REQ_STATE_OFFSET, HEADER_REQ_STATE_NO_REQ );

	/* wait a while to allow the picoArray to see it */
	msleep( CNF_STATE_TIMEOUT );

	SRAM_DEBUG( KERN_INFO "Completed OPEN_REQ\n");

	mutex_unlock( &pdata->pico_state_mutex );

	return ( error );
}


/*
 * Read the Confirmed State
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	reads the Confirmed State
 * Return:
 *	the Confirmed State.
 */
u32 read_sram_confirmed_state( struct private_driver_data *pdata )
{
	u32 cnf_state;

	/* grab the mutex to prevent clash */
	mutex_lock( &pdata->pico_state_mutex );

	/* read the current PicoArray Confirmed State */
	cnf_state = sram_read_u32( pdata->header_address, SRAM_HEADER_CNF_STATE_OFFSET );
	SRAM_DEBUG( KERN_INFO "PicoArray Confirmed State was %08X: %s\n",
		cnf_state, sram_confirmed_state_text( cnf_state ) );

	mutex_unlock( &pdata->pico_state_mutex );

	return ( cnf_state );
}


/*
 * Check whether the picoArray has been programmed with the IPsec image
 * This is far from infallible at the moment.
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	reads the Confirmed State
 * Return:
 *	the Confirmed State.
 */
u8 read_pico_active_state( struct private_driver_data *pdata )
{
	u32 picoState = 0;
	u8  picoActive = 0;

	picoState = read_sram_confirmed_state( pdata );
	
	switch ( picoState ) {
	case HEADER_CNF_STATE_READY:
	case HEADER_CNF_STATE_CLOSED:
	case HEADER_CNF_STATE_OPENED:
		picoActive = 1;
		break;

	default:
		picoActive = 0;
		break;
	}
	
	return picoActive;
}

/*
 * Initialises the SRAM header but ignores the picoArray state
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	writes information into the PicoArray SRAM
 * Return:
 *	void 
 */
static void init_sram_header( struct private_driver_data *pdata )
{
	/* set the BD ring #1 pointer, relative to start of the SRAM */
	sram_write_u32( pdata->header_address, SRAM_HEADER_BDR_PTR1_OFFSET,
		 pdata->bdr_info[ 0 ].bdr_base_address - pdata->header_address );

	/* set the BD ring #2 pointer, relative to start of the SRAM */
	sram_write_u32( pdata->header_address, SRAM_HEADER_BDR_PTR2_OFFSET,
		 pdata->bdr_info[ 1 ].bdr_base_address - pdata->header_address );

	/* set the BD ring #3 pointer, relative to start of the SRAM */
	sram_write_u32( pdata->header_address, SRAM_HEADER_BDR_PTR3_OFFSET,
		 pdata->bdr_info[ 2 ].bdr_base_address - pdata->header_address );

	/* set the BD ring #4 pointer, relative to start of the SRAM */
	sram_write_u32( pdata->header_address, SRAM_HEADER_BDR_PTR4_OFFSET,
		 pdata->bdr_info[ 3 ].bdr_base_address - pdata->header_address );
}


/*
 * Initialises the specified Buffer Descriptor ring
 * Inputs:
 *	pbdr_info = pointer to bdr mananagement information
 *	number_bds = number of bds in the ring
 * Outputs:
 *	writes information into the PicoArray SRAM buffer descriptor ring
 * Return:
 *	void 
 */
static void init_buffer_descriptor_ring( struct bdr_management *pbdr_info, u8 number_bds )
{
	u32 pos;
	u32 ring_base = pbdr_info->bdr_base_address;

	/* Initialise the spin lock */
	spin_lock_init( &pbdr_info->bdr_lock );

	/* save the total number of bds for the ring */
	pbdr_info->total_num_bds = number_bds;

	/* indicate no bds are in use */
	pbdr_info->num_bds_in_use = 0;

	/* start allocating from the first bd in the ring */
	pbdr_info->last_free_bd = ring_base;

	/* current bd in the ring to be processed */
	pbdr_info->current_bd_processed = ring_base;


	/* zero out the buffer ring */
	/* increment in 4 byte steps */
	for ( pos = 0; pos < ( SRAM_BD_SIZE * number_bds ); pos += 4 ) {
		/* Initialise the memory to zero */  
		sram_write_u32( ring_base, pos, 0 );
	}

	if ( number_bds >= 1 ) {
		/* on the last BD, set the wrap bit */
		sram_write_u32( ring_base,
			( SRAM_BD_SIZE * ( number_bds - 1 ) ) + SRAM_BD_CONTROL_FLAGS_OFFSET,
			BD_WRAP_BIT );
	}
}


/*
 * Allocate a buffer descriptor from a BD ring. 
 * Allocation will be in buffer order.
 * Inputs:
 *	pbdr_info = pointer to bdr mananagement information
 *	pwrap_bit_flag = pointer to bd wrap bit flag
 * Outputs:
 *	allocates a buffer descriptor from a BD ring and indicates the wrap bit
 * Return:
 *	address of allocated BD or 0 if error occurred
 */
u32 alloc_sram_buffer_descriptor( struct bdr_management *pbdr_info, u16 *pwrap_bit_flag )
{
	u32 allocated_bd;

	/* get the number of bds */
	u8 number_bds = pbdr_info->total_num_bds;

	/* get the base address of the ring buffer */
	u32 ring_base = pbdr_info->bdr_base_address;

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	/* protect critical code from async event */
	spin_lock_irqsave( &pbdr_info->bdr_lock, pbdr_info->bdr_flags );
#endif

	/* are any bds free ? */
	if ( pbdr_info->num_bds_in_use < number_bds ) {
		/* there is a free bd so use it */
		pbdr_info->num_bds_in_use++;

#ifdef DEBUG_BDR_MANAGEMENT
		SRAM_INFO( KERN_INFO "Alloc: BDs in use: %02d\n", pbdr_info->num_bds_in_use );
#endif

		/* allocate to last free bd */
		allocated_bd = pbdr_info->last_free_bd;

		/* identify next "free bd" */
		/* if all bds are in use then the last free bd is actually currently in use */

		/* move to the next free bd */
		pbdr_info->last_free_bd += SRAM_BD_SIZE;

		if ( pbdr_info->last_free_bd >= ( ring_base + ( SRAM_BD_SIZE * number_bds ) ) ) {
			/* reached end of the ring so wrap around */
			pbdr_info->last_free_bd = ring_base;

			/* indicate wrap */
			*pwrap_bit_flag = BD_WRAP_BIT;

		} else {
			/* no wrap */
			*pwrap_bit_flag = 0;
		}

	} else {
		/* failed to allocate a buffer descriptor */
		allocated_bd = 0;

		/* no wrap */
		*pwrap_bit_flag = 0;
	}

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	spin_unlock_irqrestore( &pbdr_info->bdr_lock, pbdr_info->bdr_flags );
#endif

	return ( allocated_bd );
}


/*
 * Free the current processed buffer descriptor from a BD ring.
 * FREES MUST BE IN THE SAME SEQUENCE ORDER AS THE ALLOCATION eg. in buffer order. 
 * Inputs:
 *	pbdr_info = pointer to bdr mananagement information
 * Outputs:
 *	free the current proceessed buffer descriptor from a BD ring
 * Return:
 *	Address of the next db to process or 0 if no active db
 */
u32 finished_sram_buffer_descriptor( struct bdr_management *pbdr_info )
{
	u32 current_bd;

	/* get the base address of the ring buffer */
	u32 ring_base = pbdr_info->bdr_base_address;

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	/* protect critical code from async event */
	spin_lock_irqsave( &pbdr_info->bdr_lock, pbdr_info->bdr_flags );
#endif

	/* are there any allocated bds ? */
	if ( pbdr_info->num_bds_in_use >= 1 ) {
		/* free up the bd */
		pbdr_info->num_bds_in_use--;

#ifdef DEBUG_BDR_MANAGEMENT
		SRAM_INFO( KERN_INFO "Finished: BDs in use: %02d\n", pbdr_info->num_bds_in_use );
#endif

		/* Mark the current BD as now free */
		mark_bd_sram_free( pbdr_info->current_bd_processed );

		/* free is achieved by moving to the next db to be processed */
		pbdr_info->current_bd_processed += SRAM_BD_SIZE;

		if ( pbdr_info->current_bd_processed >=
			( ring_base + ( SRAM_BD_SIZE * pbdr_info->total_num_bds ) ) ) {
			/* reached end of the ring so wrap around */
			pbdr_info->current_bd_processed = ring_base;
		}
	}

	/* are there any more allocated bds ? */
	if ( pbdr_info->num_bds_in_use >= 1 ) {
		/* report back the current bd */
		current_bd = pbdr_info->current_bd_processed;

	} else {
		/* no active db */
		current_bd = 0;
	}

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	spin_unlock_irqrestore( &pbdr_info->bdr_lock, pbdr_info->bdr_flags );
#endif

	return ( current_bd );
}


/*
 * Get the current processed buffer descriptor from a BD ring.
 * Inputs:
 *	pbdr_info = pointer to bdr mananagement information
 * Outputs:
 *	gets the current proceessed buffer descriptor from a BD ring
 * Return:
 *	Address of the current db to process or 0 if no active bd 
 */
u32 get_current_sram_buffer_descriptor( struct bdr_management *pbdr_info )
{
	u32 current_bd;

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	/* protect critical code from async event */
	spin_lock_irqsave( &pbdr_info->bdr_lock, pbdr_info->bdr_flags );
#endif

	/* are there any allocated bds ? */
	if ( pbdr_info->num_bds_in_use >= 1 ) {
		/* report back the current bd */
		current_bd = pbdr_info->current_bd_processed;

	} else {
		/* no active db */
		current_bd = 0;
	}

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	spin_unlock_irqrestore( &pbdr_info->bdr_lock, pbdr_info->bdr_flags );
#endif

	return ( current_bd );
}


/*
 * Mark the bd as free in a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 * Outputs:
 *	Marks the bd as free in the control flags and clears the error flags.
 *	Also makes sure to keep the wrap flag 
 * Return:
 *	None.
 */
void mark_bd_sram_free( u32 allocated_bd )
{
	u16 control_err_flags;

	/* read the control and error flags */
	control_err_flags = sram_read_u16( allocated_bd, SRAM_BD_CONTROL_FLAGS_OFFSET );

#ifdef DEBUG_BDR_MANAGEMENT
	SRAM_INFO( KERN_INFO "Freeing BD: BEFORE control_err_flags %02X\n", control_err_flags );
#endif

	/* clear status and error flags eg. mark it free, make sure not to lose the wrap bit */
	control_err_flags &= ~BD_STATUS_MASK & 0x00FF;
	control_err_flags |= BD_STATUS_FREE;

#ifdef DEBUG_BDR_MANAGEMENT
	SRAM_INFO( KERN_INFO "Freeing BD: AFTER control_err_flags %02X\n", control_err_flags );
#endif

	/* write the control and error flags back */
	sram_write_u16( allocated_bd, SRAM_BD_CONTROL_FLAGS_OFFSET, control_err_flags );
}


/*
 * Mark the bd as ready in a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 *	wrap_bit_flag = value of the WRAP_BIT
 * Outputs:
 *	Marks the bd as ready in the control flags and clears the error flags.
 *	Also makes sure to keep the wrap flag 
 * Return:
 *	None.
 */
void mark_bd_sram_ready( u32 allocated_bd, u16 wrap_bit_flag )
{
	u16 control_err_flags;

	/* clear status flags and mark it ready and clear error flags, make sure not to lose the wrap bit */
	control_err_flags = wrap_bit_flag | BD_STATUS_READY;

	/* write the control and error flags together */
	sram_write_u16( allocated_bd, SRAM_BD_CONTROL_FLAGS_OFFSET, control_err_flags );
}


/*
 * Get the bd status from a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 * Outputs:
 *	Reads the control and error flags to generate a status.
 * Return:
 *	status as enum bd_status.
 */
enum bd_status get_bd_sram_status( u32 allocated_bd )
{
	enum bd_status status = BD_NO_PROCESSED_DATA;
	u16 control_err_flags;
	u8 control_flags;
	u8 error_flags;

	/* read the control and error flags */
	control_err_flags = sram_read_u16( allocated_bd, SRAM_BD_CONTROL_FLAGS_OFFSET );

	/* get status flags */
	control_flags = control_err_flags & BD_STATUS_MASK;

	if ( control_flags == BD_STATUS_PROCESSED ) {
		/* PicoArray has processed the BD */

		/* get the error flags */
		error_flags = ( control_err_flags >> 8 );

		if ( error_flags == 0 ) {
			/* all is good */
			status = BD_DATA_PROCESSED;

		} else {
			SRAM_ERROR( KERN_ERR "ERROR: PicoArray reported an error: 0x%02X at 0x%08X\n",
 					error_flags, allocated_bd );

			/* else an error occured */
			status = BD_ERROR_OCCURRED;
		}
	} else if ( control_flags != BD_STATUS_FREE ) {
			/* indicate data is pending processing */
			status = BD_DATA_PENDING;
	}

	return ( status );
}


/*
 * Insert the enc_dec_len into a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 *	enc_dec_len = length of the encryption/decryption data in bytes
 * Outputs:
 *	Insert the enc_dec_len into a buffer descriptor.
 * Return:
 *	None.
 */
void insert_bd_sram_enc_dec_len( u32 allocated_bd, u32 enc_dec_len )
{
	/* write the enc_dec_len to the bd and convert to number of bits */
	sram_write_u16( allocated_bd, SRAM_BD_ENC_DEC_LENGTH_OFFSET, (u16) ( enc_dec_len * 8 ) );
}


/*
 * Insert the context structure address into a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 *	context_address = address of the context structure relative to the start of sram
 * Outputs:
 *	Insert the context structure address into a buffer descriptor.
 * Return:
 *	None.
 */
void insert_bd_sram_context_address( u32 allocated_bd, u16 context_address )
{
	/* write the context structure address to the bd */
	sram_write_u16( allocated_bd, SRAM_BD_CTX_PTR_OFFSET, context_address );
}


/*
 * Insert the PDU address into a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 *	pdu_address = address of the pdu relative to the start of sram
 * Outputs:
 *	Insert the PDU address into a buffer descriptor.
 * Return:
 *	None.
 */
void insert_bd_sram_pdu_address( u32 allocated_bd, u16 pdu_address )
{
	/* write the PDU address to the bd */
	sram_write_u16( allocated_bd, SRAM_BD_PDU_PTR_OFFSET, pdu_address );
}


/*
 * TODO: IMPLEMENT LOCKING
 * Get the absolute PDU address from a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 * Outputs:
 *	Reads the PDU address from a buffer descriptor.
 *	Checks that the PDU is in use
 *	Converts the PDU address to absolute
 * Return:
 *	PDU absolute address or 0 for PDU not in use error
 */
u32 get_bd_sram_pdu_address( struct private_driver_data *pdata, u32 allocated_bd )
{
	u32 pdu_address; 
	u32 pdu_state;

	/* convert relative address to virtual absolute address */
	pdu_address = sram_read_u16( allocated_bd, SRAM_BD_PDU_PTR_OFFSET ) + pdata->header_address;

	/* check that the PDU is in use */
	pdu_state = sram_read_u32( pdu_address - SRAM_PDU_HEADER_SIZE, SRAM_PDU_HDR_STATE_OFFSET );

	if ( ( pdu_state & PDU_HDR_STATE_MASK ) != PDU_HDR_STATE_IN_USE ) {
		SRAM_ERROR( KERN_ERR "ERROR: BD 0x%08X contains a PDU address that is not in use: 0x%08X\n",
			allocated_bd, pdu_address - SRAM_PDU_HEADER_SIZE );

		pdu_address = 0;	/* indicate error */
	}

	return ( pdu_address );
}


/*
 * Insert the AEAD Request pointer into a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 *	req = pointer to AEAD request structure.
 * Outputs:
 *	Insert the AEAD Request pointer into a buffer descriptor.
 * Return:
 *	None.
 */
void insert_bd_sram_aead_request( u32 allocated_bd, struct aead_request *req )
{
	/* write the AEAD Request pointer to the bd */
	sram_write_u32( allocated_bd, SRAM_BD_AEAD_REQ_PTR_OFFSET, (u32) req );
}


/*
 * Get the AEAD Request pointer from a buffer descriptor.
 * Inputs:
 *	allocated_bd = address of the buffer descriptor.
 * Outputs:
 *	Gets the AEAD Request pointer from a buffer descriptor.
 * Return:
 *	AEAD Request pointer.
 */
struct aead_request *get_bd_sram_aead_request( u32 allocated_bd )
{
	/* reads the AEAD Request pointer from the bd */
	return ( (struct aead_request *) sram_read_u32( allocated_bd, SRAM_BD_AEAD_REQ_PTR_OFFSET ) );
}


/*
 * Initialises the shadow block of context structures.  The SRAM copy is not initialised
 * here, since the picoArray may not be active yet.  This will be called once at driver
 * startup.
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 *	number_contexts = number of contexts in the block of context structures
 * Outputs:
 *	writes information into the PicoArray SRAM context structure block
 * Return:
 *	void 
 */
void init_context_structure_block( struct private_driver_data *pdata, u8 number_contexts )
{
	u32 ctx;

	/* save the number of context structures within the block */
	pdata->context_info.total_num_ctxs = number_contexts;

	/* Initialise the working variables */
	pdata->context_info.last_free_ctx = 0;
	pdata->context_info.num_ctx_in_use = 0;

	/* Initialise the memory to zero */  
	memset( pdata->context_info.shadow, 0, sizeof(pdata->context_info.shadow) );

	/* set the status to free on each context */
	/* increment in context steps */
	for ( ctx = 0; ctx < number_contexts; ctx++ ) {
		/* Initialise the status to free */  
		pdata->context_info.shadow[ctx].state = CTX_FREE;
	}
}


/*
 * Copies a single context from the driver's shadow copy to SRAM
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 *	ctx   = index of the context to copy from driver memory to SRAM
 * Outputs:
 *	writes information into the PicoArray SRAM context structure block
 * Return:
 *	void 
 */
static void copy_context_structure( struct private_driver_data *pdata, int ctx )
{
	u32 pos;
	u32 *shadow_p = (u32*)&pdata->context_info.shadow[ctx];

	u8 current_state = atomic_read( &pdata->driver_transit_state );
	
	SRAM_DEBUG( KERN_INFO "copy_context_structure: %d %d\n", current_state, ctx );

	/* Only copy to SRAM if the picoArray is active */
	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
	
		/* get the base address of the particular context structure to be written to */
		u32 context_base = pdata->context_info.context_base_address + ctx*SRAM_CTX_SIZE;
	
		SRAM_DEBUG( KERN_INFO "copy_context_structure: from %p to 0x%08X\n", shadow_p, context_base );
		
		/* Copy from shadow context to active context */
		for ( pos = 0; pos < SRAM_CTX_SIZE; pos += 4 ) {
			sram_write_u32( context_base, pos, *shadow_p );
			shadow_p++;
		}
	}
}


/*
 * Copies all the context structures from driver's shadow copy to SRAM
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	writes information into the PicoArray SRAM context structure block
 * Return:
 *	void 
 */
static void copy_all_context_structures( struct private_driver_data *pdata )
{
	u32 pos;
	u32 *shadow_p = (u32*)pdata->context_info.shadow;

	/* get the base address of the block of context structures */
	u32 context_base = pdata->context_info.context_base_address;
	u32 number_contexts = pdata->context_info.total_num_ctxs;

	SRAM_DEBUG( KERN_INFO "copy_all_context_structures: from %p to 0x%08X\n", shadow_p, context_base );
	
	for ( pos = 0; pos < (SRAM_CTX_SIZE * number_contexts); pos += 4 ) {
		sram_write_u32( context_base, pos, *shadow_p );
		shadow_p++;
	}
}


/*
 * Allocate a context structure, uses the context status flag 
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	allocates a context structure from the PicoArray SRAM context structure block
 * Return:
 *	address of allocated context structure or 0 if error occurred
 */
u32 alloc_sram_context_structure( struct private_driver_data *pdata )
{
	u32 allocated_ctx;
	u32 data;

	/* get the number of contexts */
	u8 number_contexts = pdata->context_info.total_num_ctxs;

	/* get the base address of the block of context structures */
	u32 context_base = pdata->context_info.context_base_address;

	/* are any contexts free ?   We look in the shadow copy. */
	if ( pdata->context_info.num_ctx_in_use < number_contexts ) {
		/* there is a free context so use it */
		pdata->context_info.num_ctx_in_use++;

		/* allocate to last free context.  This is the address where the active copy will be
		 * and is returned to the caller to allow it to link the BD to the context in SRAM.
		 */
		allocated_ctx = context_base + (pdata->context_info.last_free_ctx * SRAM_CTX_SIZE);

		/* mark the shadow context as used */
		pdata->context_info.shadow[pdata->context_info.last_free_ctx].state = CTX_IN_USE;

		/* identify a new free context in the shadow area */
		if ( pdata->context_info.num_ctx_in_use < number_contexts ) {

			do {
				/* move to the next context structure */
				pdata->context_info.last_free_ctx++;

				if ( pdata->context_info.last_free_ctx >= number_contexts ) {
					/* reached end of block so wrap around */
					pdata->context_info.last_free_ctx = 0;
				}

				/* read the context status */
				data = pdata->context_info.shadow[pdata->context_info.last_free_ctx].state;

			} while ( data != CTX_FREE );

		} else {
			/* none available */
			pdata->context_info.last_free_ctx = -1;
		}
	} else {
		/* failed to allocate a context structure */
		allocated_ctx = 0;
	}

	return ( allocated_ctx );
}


/*
 * Map from a context address in the SRAM to a context index.  The driver uses
 * the former for linking from BDs to contexts, but we need the latter for
 * easy access to contexts in the shadow area.
 * Inputs:
 * 	pdata         = pointer to private data structure (not NULL)
 *	allocated_ctx = address of context that is to be accessed
 * Outputs:
 * Return:
 *	Index of the context or -1 if an error occurred
 */
int map_context_structure_to_index( struct private_driver_data *pdata, u32 allocated_ctx )
{
	u32  context_base    = pdata->context_info.context_base_address;
	u8   number_contexts = pdata->context_info.total_num_ctxs;
	int  ctx             = -1;

	if ( ( allocated_ctx >= context_base ) && ( allocated_ctx < ( context_base + number_contexts*SRAM_CTX_SIZE) ) ) {
		ctx = (allocated_ctx - context_base) / SRAM_CTX_SIZE;
	}
	
	return ctx;
}

/*
 * Free a context structure, uses the context status flag.
 * Frees can be in any order.
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 *	allocated_ctx = address of context that is to be freed
 * Outputs:
 *	frees a context structure from the PicoArray SRAM context structure block
 * Return:
 *	0 for success or 1 if an error occurred
 */
u32 free_sram_context_structure( struct private_driver_data *pdata, u32 allocated_ctx )
{
	u32 error = 0;
	u32 data;
	int ctx = map_context_structure_to_index( pdata, allocated_ctx );

	if ( ctx >= 0 ) {
		/* check that the context is in use */
		data = pdata->context_info.shadow[ctx].state;
	
		if ( data == CTX_IN_USE ) {
			/* context address is valid */
	
			if ( pdata->context_info.num_ctx_in_use == pdata->context_info.total_num_ctxs ) {
				/* if all contexts active then allocate this context as the last free context */
				pdata->context_info.last_free_ctx = ctx;
			}
	
			/* count down the context in use counter */
			pdata->context_info.num_ctx_in_use--;
	
			/* mark the context as free */
			pdata->context_info.shadow[ctx].state = CTX_FREE;
	
		} else {
			/* address is invalid */
			error = 1;
		}
		
	} else {
		/* address is invalid */
		error = 1;
	}

	return ( error );
}


/*
 * Writes the authentication and encryption keys to the shadow
 * context area and copies to the SRAM context structure if possible.
 * Inputs:
 *	allocated_ctx = address of context
 *	pall_keys = pointer to all keys string
 *	authkeylen = length of the authentication key
 *	enckeylen = length of the encryption key
 * Outputs:
 *	Writes the keys to the SRAM context structure
 * Return:
 *	none
 */
void write_sram_context_keys( struct private_driver_data *pdata, u32 allocated_ctx, const u8 *pall_keys, u8 authkeylen, u8 enckeylen )
{
	int ctx = map_context_structure_to_index( pdata, allocated_ctx );

	if (ctx >= 0) {
		/* copy authentication key */
		if ( authkeylen == MAX_AUTHKEY_LEN ) {
	
			/* copy the authentication key to the SRAM context */
			memcpy(pdata->context_info.shadow[ctx].auth_key, pall_keys, MAX_AUTHKEY_LEN);
		} else {
			/* no key so fill SRAM context with zeros */
			memset(pdata->context_info.shadow[ctx].auth_key, 0, MAX_AUTHKEY_LEN);
		}
	
		/* copy encryption key */
		if ( enckeylen == MAX_ENCKEY_LEN ) {
	
			/* copy the encryption key to the SRAM context */
			memcpy(pdata->context_info.shadow[ctx].aes_key, pall_keys+authkeylen, MAX_ENCKEY_LEN);
		} else {
			/* no key so fill SRAM context with zeros */
			memset(pdata->context_info.shadow[ctx].aes_key, 0, MAX_ENCKEY_LEN);
		}
	
		copy_context_structure( pdata, ctx );

	/* MV Debug */
#ifdef DEBUG_SRAM
		{
			unsigned int pos;
	
			SRAM_INFO( KERN_INFO "input key: \n");
			for ( pos = 0; pos < ( authkeylen + enckeylen ); pos++ ) {
				SRAM_INFO( KERN_INFO "%02X\n", pall_keys[pos] );
			}
	
			sram_hexdump( "SRAM keys", allocated_ctx + SRAM_CTX_AUTHKEY_OFFSET,
				MAX_AUTHKEY_LEN + MAX_ENCKEY_LEN );
	
		}
#endif
	}
}


/*
 * Writes the SEQH value to the shadow context area and copies to
 * the SRAM context structure if possible.
 * ESN not supported so set SEQH to zero.
 * TODO: in order to support ESN, SEQH needs to be moved to the PDU.
 * Inputs:
 *	seqh = seqh value ( should be zero as ESN not supported )
 * Outputs:
 *	Writes SEQH to the SRAM context structure
 * Return:
 *	none
 */
void write_sram_context_seqh( struct private_driver_data *pdata, u32 allocated_ctx, u32 seqh )
{
	int ctx = map_context_structure_to_index( pdata, allocated_ctx );

	if (ctx >= 0) {
		/* write SEQH to the context structure */
		pdata->context_info.shadow[ctx].seqh = seqh;
		copy_context_structure( pdata, ctx );
	}
}


/*
 * Writes the context flags to the shadow context area and copies to
 * the SRAM context structure if possible.
 * Inputs:
 *	allocated_ctx = address of context
 *	flags = context flags
 * Outputs:
 *	Writes the context flags to the SRAM context structure
 * Return:
 *	none
 */
void write_sram_context_flags( struct private_driver_data *pdata, u32 allocated_ctx, u8 flags )
{
	int ctx = map_context_structure_to_index( pdata, allocated_ctx );

	if (ctx >= 0) {
	/* write the context flags to the context structure */
		pdata->context_info.shadow[ctx].flags = flags;
		copy_context_structure( pdata, ctx );
	}
}


/*
 * Initialises the PDU pool
 * Inputs:
 *	ppdu_info = pointer to pdu mananagement information
 * Outputs:
 *	writes information into the pdu management structure
 * Return:
 *	void 
 */
static void init_pdu_pool( struct pdu_management *ppdu_info )
{
	/* calculate the pool end address, points 1 byte past the end */
	ppdu_info->pdu_pool_end_address = ppdu_info->pdu_pool_base_address + ppdu_info->pdu_pool_size;

	/* Initialise the spin lock */
	spin_lock_init( &ppdu_info->pdu_lock );

	/* initialise the first PDU address to start at the base of the pool */
	ppdu_info->next_alloc_pdu = ppdu_info->pdu_pool_base_address;

	/* initialise the oldest PDU address to be same as the next to be allocated */
	ppdu_info->oldest_pdu = ppdu_info->next_alloc_pdu;

	/* initialise the metric of how many PDUs are in use */
	ppdu_info->num_pdus_in_use = 0;

	/* initialise the metric of how many PDUs are allocated */
	ppdu_info->num_pdus_alloced = 0;
}


/*
 * Allocates a PDU from the PDU pool
 * Inputs:
 *	ppdu_info = pointer to pdu mananagement information
 *	pdu_size = requested PDU size
 *	bdr_num = the number of the BDR so that the oldest PDU can be processed
 * Outputs:
 *	Allocates a PDU
 * Return:
 *	address of the PDU or 0 if an error occurred
 */
u32 alloc_sram_pdu( struct pdu_management *ppdu_info, u32 pdu_size, enum bdr_number bdr_num )
{
	u32 test_next_alloc_pdu;	/* test next PDU without committing to allocate it */

	/* PDUs are allocated on 4 byte boundaries and butt up to each other in the pool */
	/* The pool contains variable length PDUs for maximum efficiency */

	u32 allocated_pdu = 0;
	
	/* first align size to a 4 byte boundary */
	pdu_size = ( pdu_size + 0x3 ) & (~0x3);

	if ( pdu_size > PDU_MAX_SIZE ) {
		/* requested PDU is too large */
		SRAM_ERROR( KERN_ERR "ERROR: %s Requested PDU too large %d > %d\n", __FUNCTION__, pdu_size, PDU_MAX_SIZE );
		goto pdu_error;
	}

	/* reserve space for the PDU header (ARMs usage) */
	pdu_size += SRAM_PDU_HEADER_SIZE;

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	/* protect critical code from async event */
	spin_lock_irqsave( &ppdu_info->pdu_lock, ppdu_info->pdu_flags );
#endif

	/* use a working copy to test for the next PDU allocation */
	test_next_alloc_pdu = ppdu_info->next_alloc_pdu;

#ifdef DEBUG_PDU_MANAGEMENT
	SRAM_INFO( KERN_INFO "Alloc: oldest PDU is 0x%08X, current next is 0x%08X\n",
		ppdu_info->oldest_pdu, ppdu_info->next_alloc_pdu );
#endif

	/* is there room at the end of the pool ? reserve space for a PDU header at the end of the pool */
	if ( test_next_alloc_pdu >=
		( ppdu_info->pdu_pool_end_address - pdu_size - SRAM_PDU_HEADER_SIZE ) ) {
		/*
		 * Check before correcting for the end of the pool in case the oldest PDU
		 * is near the end of the pool.
		 */
		if ( ppdu_info->oldest_pdu >=
			( ppdu_info->pdu_pool_end_address - pdu_size - SRAM_PDU_HEADER_SIZE ) ) {
			/* oldest is also near the end of the pool */

			if ( test_next_alloc_pdu < ppdu_info->oldest_pdu ) {
				/* ERROR: out of room because allocation would move past the oldest PDU */  
#ifdef DEBUG_PDU_MANAGEMENT_NO_ROOM
				SRAM_INFO( KERN_INFO "PDU alloc out of room: dead area\n" );
#endif
				goto pdu_no_room;
			}
		}

		/* not enough space at pool end, so wrap around to the start of the pool */
		test_next_alloc_pdu = ppdu_info->pdu_pool_base_address;

		if ( test_next_alloc_pdu == ppdu_info->oldest_pdu ) {
			/* ERROR: out of room because allocation would be on top of the oldest PDU */  
#ifdef DEBUG_PDU_MANAGEMENT_NO_ROOM
			SRAM_INFO( KERN_INFO "PDU alloc out of room: on top of oldest PDU\n" );
#endif
			goto pdu_no_room;
		}


	}

	if ( test_next_alloc_pdu < ppdu_info->oldest_pdu ) {
		/* allocation pointer is heading towards the oldest PDU */

		if ( ( test_next_alloc_pdu + pdu_size ) >= ppdu_info->oldest_pdu ) {
			/* ERROR: out of room because allocation would include the oldest PDU */  
#ifdef DEBUG_PDU_MANAGEMENT_NO_ROOM
			SRAM_INFO( KERN_INFO "PDU alloc out of room: includes oldest PDU\n" );
#endif
			goto pdu_no_room;
		}
	}

	/* do the allocation, remember to reserve space for the ARM PDU header */
	allocated_pdu = test_next_alloc_pdu + SRAM_PDU_HEADER_SIZE;

	if ( test_next_alloc_pdu != ppdu_info->next_alloc_pdu ) {
		/*
		 * a wrap has occurred at the end of the pool, so flag it
		 * flag the reserved header of the last PDU as the last PDU header in the pool
		 */ 
		sram_write_u32( ppdu_info->next_alloc_pdu, SRAM_PDU_HDR_STATE_OFFSET, PDU_HDR_STATE_POOL_WRAP | bdr_num );

		/* insert PDU pointer in the first PDU in the pool */
		sram_write_u32( test_next_alloc_pdu, SRAM_PDU_HDR_PTR_OFFSET, test_next_alloc_pdu + pdu_size );

#ifdef DEBUG_PDU_MANAGEMENT
		SRAM_INFO( KERN_INFO "Alloc: Making last header as pool end\n" );
#endif
	}

	/* flag this allocated PDU as in use, may be the wrapped PDU */ 
	sram_write_u32( test_next_alloc_pdu, SRAM_PDU_HDR_STATE_OFFSET, PDU_HDR_STATE_IN_USE | bdr_num );

	/* move past the end of the allocation to define the base of the next allocation */
	test_next_alloc_pdu += pdu_size;

	/*
	 * update the PDU header pointer to point to the next PDU to be allocate
	 * could be in the reserved PDU header at the end of the pool
	 */ 
	sram_write_u32( ppdu_info->next_alloc_pdu, SRAM_PDU_HDR_PTR_OFFSET, test_next_alloc_pdu );

	/* update the next allocation PDU pointer */
	ppdu_info->next_alloc_pdu = test_next_alloc_pdu;

#ifdef DEBUG_PDU_MANAGEMENT
	SRAM_INFO( KERN_INFO "Alloc: oldest PDU is 0x%08X, NEW next is 0x%08X\n",
		ppdu_info->oldest_pdu, ppdu_info->next_alloc_pdu );
#endif

	/* record the number of in use PDUs eg. excludes freed PDUs */
	ppdu_info->num_pdus_in_use++;

	/* record the number of alloced eg. includes freed PDUs */
	ppdu_info->num_pdus_alloced++;

#ifdef DEBUG_PDU_MANAGEMENT
	SRAM_INFO( KERN_INFO "PDU alloc: allocated %d, in use %d\n",
		ppdu_info->num_pdus_alloced, ppdu_info->num_pdus_in_use );
#endif

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	spin_unlock_irqrestore( &ppdu_info->pdu_lock, ppdu_info->pdu_flags );
#endif

#ifdef DEBUG_SRAM
	SRAM_DEBUG( KERN_INFO "PDU allocated: 0x%08X size: 0x%08X\n", allocated_pdu, pdu_size );
#endif
	return ( allocated_pdu );

pdu_no_room:
#ifdef DEBUG_PDU_MANAGEMENT_NO_ROOM
	SRAM_ERROR( KERN_ERR "ERROR: No room in PDU pool to allocate this PDU: allocated %d, in use %d\n",
		ppdu_info->num_pdus_alloced, ppdu_info->num_pdus_in_use );
#endif

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	spin_unlock_irqrestore( &ppdu_info->pdu_lock, ppdu_info->pdu_flags );
#endif

pdu_error:
	return ( allocated_pdu );
}


/*
 * Get the BDR number of the oldest PDU
 * Inputs:
 *	ppdu_info = pointer to pdu mananagement information
 *	pbdr_num = pointer to a bdr_num variable of the calling function
 * Outputs:
 *	reads the BDR number out of the PDU header state field
 * Return:
 *	0 for success and 1 for failure
 */
u8 get_bdr_num_of_oldest_pdu( struct pdu_management *ppdu_info, enum bdr_number *pbdr_num )
{
	u32 oldest_state;
	u8 error = 0;
	enum bdr_number tmp_bdr_num;

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	/* protect critical code from async event */
	spin_lock_irqsave( &ppdu_info->pdu_lock, ppdu_info->pdu_flags );
#endif

#ifdef DEBUG_PDU_MANAGEMENT
	SRAM_INFO( KERN_INFO "Called %s\n", __FUNCTION__ );
#endif

	/* read the state of the oldest PDU */
	oldest_state = sram_read_u32( ppdu_info->oldest_pdu, SRAM_PDU_HDR_STATE_OFFSET );

	if ( ( oldest_state & PDU_HDR_STATE_MASK ) == PDU_HDR_STATE_IN_USE ) {
		/* bdr_num only valid for PDUs that are in use */

		/* get the bdr_num of the oldest allocated PDU */
		tmp_bdr_num = ( oldest_state & PDU_HDR_STATE_BDR_MASK );

		if ( tmp_bdr_num < BDR_TOTAL_NUM ) {
			/* inform the calling function */
			*pbdr_num = tmp_bdr_num;
#ifdef DEBUG_PDU_MANAGEMENT
			SRAM_INFO( KERN_INFO "Oldest PDU is in BDR number %d\n", tmp_bdr_num );
#endif
		} else {
			/* out of range */
			error = 1;
		}
	} else {
		/* bdr_num not available, probably due to PDU being freed */
		error = 1;
	}

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	spin_unlock_irqrestore( &ppdu_info->pdu_lock, ppdu_info->pdu_flags );
#endif

	return ( error );
}


/*
 * Frees a PDU from the PDU pool
 * Inputs:
 *	ppdu_info = pointer to pdu mananagement information
 *	allocated_pdu = address of the allocated PDU
 * Outputs:
 *	Frees a PDU
 * Return:
 *	0 for success and 1 for failure
 */
u8 free_sram_pdu( struct pdu_management *ppdu_info, u32 allocated_pdu )
{
	/* point to the header of the PDU */ 
	u32 freeing_pdu = allocated_pdu - SRAM_PDU_HEADER_SIZE;
	u32 next_pdu;
	u32 pdu_state;
	u32 oldest_state;
	u8 error = 0;

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	/* protect critical code from async event */
	spin_lock_irqsave( &ppdu_info->pdu_lock, ppdu_info->pdu_flags );
#endif

	if ( allocated_pdu == 0 )
	{
		SRAM_ERROR( KERN_ERR "ERROR: Attempting to free invalid PDU\n" );
		error = 1;
		goto free_error;
	}

#ifdef DEBUG_PDU_MANAGEMENT
	sram_hexdump( "Free PDU Header", freeing_pdu, SRAM_PDU_HEADER_SIZE );
#endif

	/* read the header of the PDU to be freed */
	next_pdu = sram_read_u32( freeing_pdu, SRAM_PDU_HDR_PTR_OFFSET );
	pdu_state = sram_read_u32( freeing_pdu, SRAM_PDU_HDR_STATE_OFFSET );

	if ( ( pdu_state & PDU_HDR_STATE_MASK ) != PDU_HDR_STATE_IN_USE ) {
		SRAM_ERROR( KERN_ERR "ERROR: Attempting to free a PDU that is not in use: 0x%08X\n", freeing_pdu );
		error = 1;
		goto free_error;
	}

	/* read the state of the oldest PDU */
	oldest_state = sram_read_u32( ppdu_info->oldest_pdu, SRAM_PDU_HDR_STATE_OFFSET );

	if ( ( oldest_state & PDU_HDR_STATE_MASK ) == PDU_HDR_STATE_POOL_WRAP ) {
		/*
		 * oldest was marked as the end of the pool in the reserved PDU header
		 * by the allocator so wrap to start of the pool
		 */
		ppdu_info->oldest_pdu = ppdu_info->pdu_pool_base_address;

#ifdef DEBUG_PDU_MANAGEMENT
		SRAM_INFO( KERN_INFO "Freeing: wrap detected\n" );
#endif
	}

	if ( freeing_pdu == ppdu_info->oldest_pdu ) {
		/* now freeing the oldest PDU */

		/*
		 * spin through the linked PDUs to update the oldest
		 * PDU pointer by jumping freed PDUs
		 */
		while ( next_pdu != ppdu_info->next_alloc_pdu ) {
			/* not reached end of the allocated PDUs so spin */

			/* read the state of the next PDU in the list */
			pdu_state = sram_read_u32( next_pdu, SRAM_PDU_HDR_STATE_OFFSET );

			if ( ( pdu_state & PDU_HDR_STATE_MASK ) == PDU_HDR_STATE_FREE ) {
				/* this PDU also free so move to the next PDU */
				next_pdu = sram_read_u32( next_pdu, SRAM_PDU_HDR_PTR_OFFSET );

				/* one less PDU allocated */
				ppdu_info->num_pdus_alloced--;

			} else if ( ( pdu_state & PDU_HDR_STATE_MASK ) == PDU_HDR_STATE_POOL_WRAP ) {
				/* end of the pool found, so do a wrap */
				next_pdu = ppdu_info->pdu_pool_base_address;

#ifdef DEBUG_PDU_MANAGEMENT
				SRAM_INFO( KERN_INFO "Freeing: doing a wrap\n" );
#endif
			} else {
				/* assume in use */
				break;
			}
		}

		/* count the oldest PDU as unallocated */
		ppdu_info->num_pdus_alloced--;

		/* update the oldest PDU pointer */
		ppdu_info->oldest_pdu = next_pdu;

#ifdef DEBUG_PDU_MANAGEMENT
		SRAM_INFO( KERN_INFO "New oldest PDU is 0x%08X, next is 0x%08X\n",
			ppdu_info->oldest_pdu, ppdu_info->next_alloc_pdu );
#endif

	} else {
		/* This is a random free that is not the oldest PDU */
		
		/* Mark this PDU as free, BDR num not specified */
		sram_write_u32( freeing_pdu, SRAM_PDU_HDR_STATE_OFFSET, PDU_HDR_STATE_FREE );

		SRAM_DEBUG( KERN_INFO "PDU freed randomly\n" );
	}

	/* update the number of in use PDUs */
	ppdu_info->num_pdus_in_use--;

#ifdef DEBUG_PDU_MANAGEMENT
	SRAM_INFO( KERN_INFO "PDU free: allocated %d, in use %d\n",
		ppdu_info->num_pdus_alloced, ppdu_info->num_pdus_in_use );
#endif

free_error:

#if ( SRAM_LOCKING == LOCK_SPINLOCK_IRQ ) 
	spin_unlock_irqrestore( &ppdu_info->pdu_lock, ppdu_info->pdu_flags );
#endif

	return ( error );
}


/*
 * Copy crypto data into a PDU
 * Inputs:
 *	pdu_address = address within an allocated PDU
 *	pcrypto = pointer to the crypto data (source)
 *	length = length of crypto data in bytes
 * Outputs:
 *	Copies crypto data
 * Return:
 *	none
 */
void copy_crypto_to_pdu_sram( u32 pdu_address, u8 *pcrypto, u32 length )
{
#ifdef USE_MEMCPY_FOR_SRAM

        memcpy((void*)pdu_address, pcrypto, length);
        
#else /* memcpy */

#ifdef DEBUG_SRAM
	u32 debug_address = pdu_address;
	u32 debug_length = length;
#endif

	/* Watch out IP packets are in BIG ENDIAN format */

	if ( ( ( (u32) pcrypto ) & 0x00000003 ) == 0 ) {
		/* crypto data is on a 4 byte boundary so copy 4 bytes at a time */

		u32 *pcrypto_u32 = (u32 * ) pcrypto; 

		/* convert length into number of 4 byte words */ 
		length = (length + 3)/4;

		/*
		 * COPY CRYTPO DATA ALWAYS IN "LITTLE-ENDIAN" FORMAT TO THE SRAM
		 * REGARDLESS OF THE CPU'S ENDIANNESS BECAUSE THE DATA IS IN CRYPTO FORMAT
		 * eg. a bit stream
		 */
		while ( length > 0 ) {
			sram_write_u32_le( pdu_address, 0, *pcrypto_u32++ );
			length--;
			pdu_address += 4;
		}

		SRAM_DEBUG( KERN_INFO "Used 4 byte copy\n" ); 
	} else {
		/* not on a 4 byte boundary so copy 1 byte at a time */
		while ( length > 0 ) {
			sram_write_u8( pdu_address++, 0, *pcrypto++ );
			length--;
		}
	}

#ifdef DEBUG_SRAM
	sram_hexdump( "Copy to PDU", debug_address, debug_length );
#endif

#endif /* memcpy */
}


/*
 * Copy crypto data from a PDU
 * Inputs:
 *	pdu_address = address within an allocated PDU
 *	pcrypto = pointer to the crypto data (destination)
 *	length = length of crypto data in bytes
 * Outputs:
 *	Copies crypto data 
 * Return:
 *	none
 */
void copy_crypto_from_pdu_sram( u32 pdu_address, u8 *pcrypto, u32 length )
{
#ifdef USE_MEMCPY_FOR_SRAM

        memcpy(pcrypto, (void*)pdu_address, length);
        
#else /* memcpy */

#ifdef DEBUG_SRAM
	sram_hexdump( "Copy from PDU", pdu_address, length );
#endif
	/* Watch out IP packets are in BIG ENDIAN format */

	if ( ( ( (u32) pcrypto ) & 0x00000003 ) == 0 ) {
		/* crypto data is on a 4 byte boundary so read 4 bytes at a time */

		u32 *pcrypto_u32 = (u32 * ) pcrypto; 

		/* convert length into number of 4 byte words */ 
		length = (length + 3)/4;

		/*
		 * READ CRYTPO DATA ALWAYS IN "LITTLE-ENDIAN" FORMAT FROM THE SRAM
		 * REGARDLESS OF THE CPU'S ENDIANNESS BECAUSE THE DATA IS IN CRYPTO FORMAT
		 * eg. a bit stream
		 */
		while ( length > 0 ) {
			*pcrypto_u32++ = sram_read_u32_le( pdu_address, 0 );
			length--;
			pdu_address += 4;
		}

		SRAM_DEBUG( KERN_INFO "Used 4 byte read\n" ); 
	} else {
		/* not on a 4 byte boundary so read 1 byte at a time */
		while ( length > 0 ) {
			*pcrypto++ = sram_read_u8( pdu_address++, 0 );
			length--;
		}
	}
#endif /* memcpy */
}


/*
 * Compare crypto data with PDU data
 * Inputs:
 *	pdu_address = address within an allocated PDU
 *	pcrypto = pointer to the crypto data (destination)
 *	length = length of crypto data in bytes
 * Outputs:
 *	Compares crypto data 
 * Return:
 *	0 for OK, 1 for mismatch
 */
u8 cmp_crypto_from_pdu_sram( u32 pdu_address, u8 *pcrypto, u32 length )
{
	u8 err = 0;
	u32 error_address = pdu_address;
	u32 error_length = length;

	/* Watch out IP packets are in BIG ENDIAN format */
	/* TODO: Consider comparing using u32 instead */
	while ( length > 0 ) {
		if ( *pcrypto++ != sram_read_u8( pdu_address++, 0 ) ) {
			err = 1;
		}
		length--;
	}

	if ( err == 1 ) {
		sram_hexdump( "PDU ERROR", error_address, error_length );
	}

	return ( err );
}


/*
 * (Re-)Initialises the Buffer Descriptor Rings
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	writes information into the PicoArray SRAM
 * Return:
 *	None. 
 */
void init_sram_bdrs( struct private_driver_data *pdata )
{
	/* Initialise the BD ring #1 */
	init_buffer_descriptor_ring( &pdata->bdr_info[ BDR_NUM_1 ], SRAM_BD_RING1_NUMBER );

	/* Initialise the BD ring #2 */
	init_buffer_descriptor_ring( &pdata->bdr_info[ BDR_NUM_2 ], SRAM_BD_RING2_NUMBER );

	/* Initialise the BD ring #3 */
	init_buffer_descriptor_ring( &pdata->bdr_info[ BDR_NUM_3 ], SRAM_BD_RING3_NUMBER );

	/* Initialise the BD ring #4 */
	init_buffer_descriptor_ring( &pdata->bdr_info[ BDR_NUM_4 ], SRAM_BD_RING4_NUMBER );
}


/*
 * Initialises the SRAM structures
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	writes information into the PicoArray SRAM
 * Return:
 *	0 for success and 1 for failiure 
 */
u8 init_sram_structures( struct private_driver_data *pdata )
{
	u8 picoActive = 0;

	SRAM_DEBUG( KERN_INFO "init_sram_structures called\n");
	
	/* calcuate the header address */
	pdata->header_address = (u32) pdata->vert_sram_base_address; 


	/* calculate the context block base address allowing space for the header */
	pdata->context_info.context_base_address = pdata->header_address + SRAM_HEADER_SIZE;

	picoActive = read_pico_active_state( pdata );
	SRAM_DEBUG( KERN_INFO "picoActive is %d\n", picoActive);
		

	/* calculate the buffer descriptor ring #1 start address allowing space for contexts */
	pdata->bdr_info[ 0 ].bdr_base_address = pdata->context_info.context_base_address +
		( SRAM_CTX_SIZE * SRAM_CTX_MAX_NUMBER );

	/* calculate the buffer descriptor ring #2 start address allowing space for ring #1 */
	pdata->bdr_info[ 1 ].bdr_base_address = pdata->bdr_info[ 0 ].bdr_base_address +
		( SRAM_BD_SIZE * SRAM_BD_RING1_NUMBER );

	/* calculate the buffer descriptor ring #3 start address allowing space for ring #2 */
	pdata->bdr_info[ 2 ].bdr_base_address = pdata->bdr_info[ 1 ].bdr_base_address +
		( SRAM_BD_SIZE * SRAM_BD_RING2_NUMBER );

	/* calculate the buffer descriptor ring #4 start address allowing space for ring #3 */
	pdata->bdr_info[ 3 ].bdr_base_address = pdata->bdr_info[ 2 ].bdr_base_address +
		( SRAM_BD_SIZE * SRAM_BD_RING3_NUMBER );


	/* calculate the pdu buffer pool base address allowing space for ring #4 */
	pdata->pdu_info.pdu_pool_base_address = pdata->bdr_info[ 3 ].bdr_base_address +
		( SRAM_BD_SIZE * SRAM_BD_RING4_NUMBER );

	/* calculate the pdu pool size */
	if ( ( pdata->pdu_info.pdu_pool_base_address - pdata->header_address ) < pdata->sram_size ) {
		pdata->pdu_info.pdu_pool_size = pdata->sram_size - (pdata->pdu_info.pdu_pool_base_address - pdata->header_address );
	} else {
		SRAM_ERROR( KERN_ERR "ERROR: Insufficient SRAM space for the PDU pool\n" );
		pdata->pdu_info.pdu_pool_size = 0;
	}

	SRAM_DEBUG( KERN_INFO "PDU pool size is %d bytes\n", pdata->pdu_info.pdu_pool_size );

	/* set the transit state to all SW */
	atomic_set( &pdata->driver_transit_state, DRIVER_ALL_SW );

	SRAM_DEBUG( KERN_INFO "Set to SW mode\n" );

	if ( picoActive ) {
	
		SRAM_DEBUG( KERN_INFO "writing all data structures\n");
		
		/* try to CLOSE the picoArray but run in SW Fallback mode anyway */
		request_sram_pico_close( pdata );
	
		/* Initialise the SRAM header */
		init_sram_header( pdata );

#ifdef VERBOSE_SRAM
		sram_hexdump( "Header", pdata->header_address, SRAM_HEADER_SIZE );
#endif

		/* Initialise the context structure block */
		copy_all_context_structures( pdata );
		
#ifdef VERBOSE_SRAM
		sram_hexdump( "Context", pdata->context_info.context_base_address,
			SRAM_CTX_SIZE * SRAM_CTX_MAX_NUMBER );
#endif

		/* Initialise all the Buffer Descriptor Rings */
		init_sram_bdrs( pdata );

#ifdef VERBOSE_SRAM
		sram_hexdump( "Ring 1", pdata->bdr_info[ BDR_NUM_1 ].bdr_base_address, SRAM_BD_SIZE * SRAM_BD_RING1_NUMBER );
		sram_hexdump( "Ring 2", pdata->bdr_info[ BDR_NUM_2 ].bdr_base_address, SRAM_BD_SIZE * SRAM_BD_RING2_NUMBER );
		sram_hexdump( "Ring 3", pdata->bdr_info[ BDR_NUM_3 ].bdr_base_address, SRAM_BD_SIZE * SRAM_BD_RING3_NUMBER );
		sram_hexdump( "Ring 4", pdata->bdr_info[ BDR_NUM_4 ].bdr_base_address, SRAM_BD_SIZE * SRAM_BD_RING4_NUMBER );
#endif

		/* Initialise the PDU pool */
		init_pdu_pool( &pdata->pdu_info );

#ifdef VERBOSE_SRAM
		sram_hexdump( "PDU pool info", (u32) &pdata->pdu_info, sizeof( pdata->pdu_info ) );
#endif

#ifdef APPLICATION_NOT_AVAILABLE
		/* Test code to OPEN the picoArray when the Linux Application is not available */
		/* put the picoAray into the OPENED state */
		if( request_sram_pico_open( pdata ) == 1 ) {
			/* Failed to OPEN the picoArray */
	
			SRAM_DEBUG( KERN_INFO "Set to SW mode\n" );
	
			/* set the transit state to all SW Fallback */
			atomic_set( &pdata->driver_transit_state, DRIVER_ALL_SW );
		} else {
	
			SRAM_DEBUG( KERN_INFO "Set to HW mode\n" );

			/* set the transit state to all HW */
			atomic_set( &pdata->driver_transit_state, DRIVER_ALL_HW );
		}
#endif
	}

	return( picoActive );
}

