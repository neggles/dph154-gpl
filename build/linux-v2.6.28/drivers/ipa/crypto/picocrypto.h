/*
 * This file contains the PicoArray Crypto driver #defines, types and prototypes.
 *
 * File: picocrypto.h
 * Author: Dean Jenkins <djenkins@mvista.com>
 *
 * 2008 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*******************
 * common #defines *
 *******************/

#undef PICO_FIRMWARE_NOT_AVAILABLE 
/* If the IPsec picoArray firmware becomes available them comment out this #define */
/* #define PICO_FIRMWARE_NOT_AVAILABLE	1	*/

#undef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
/* If it is desired to process the BDRs after the end of an enc/dec then define this #define */
#define ENABLE_BDR_PROCESSING_AFTER_ENC_DEC	1

#undef APPLICATION_NOT_AVAILABLE
/* If the Linux Application becomes available then comment out this #define */
/* #define APPLICATION_NOT_AVAILABLE	1	*/

/* We want to use a single BDR as there are issues with PDU memory pool fragmentation when
   two BDRs are used */
#undef USE_TWO_BDRS
/* define to use 2 BDRs; 1 for encryption and 1 for decryption else use only 1 BDR for both functions */
/* #define USE_TWO_BDRS	1 */

/* driver locking type */
#define LOCK_NONE		(0)
#define	LOCK_SPINLOCK_IRQ	(2)

#define DRIVER_LOCKING	LOCK_SPINLOCK_IRQ 
#define SRAM_LOCKING	LOCK_NONE 


#define MAX_AUTHKEY_LEN		(20)
#define MAX_ENCKEY_LEN		(16)
#define MAX_RAWKEY_LEN		( sizeof( struct rtattr ) + MAX_AUTHKEY_LEN + MAX_ENCKEY_LEN + 12 )	/* TODO: Check length */

#define CTX_FLAGS_UNKNOWN	(0)	/* context crypto flags are unknown */
#define CTX_FLAGS_SET		(1)	/* context crypto flags have been set */

#define CNF_STATE_TIMEOUT	(100)	/* Confirmed State change time-out in ms */
#define BD_PROCESS_TIMEOUT	(100)	/* Processing of an active BD time-out in ms */

#define PICO_MAX_ENC_LENGTH	(1600)	/* max PicoArray encryption/decryption length */
 
/* SRAM header offsets */
#define SRAM_HEADER_REQ_STATE_OFFSET	(0x00)
#define HEADER_REQ_STATE_CLOSE_REQ	(0xDEADBEEF)
#define HEADER_REQ_STATE_OPEN_REQ	(0x36B9A2F1)
#define HEADER_REQ_STATE_NO_REQ		(0x00000000)

#define SRAM_HEADER_CNF_STATE_OFFSET	(0x04)
#define HEADER_CNF_STATE_READY		(0xC4E70D85)
#define HEADER_CNF_STATE_CLOSED		(0xDEADDEAD)
#define HEADER_CNF_STATE_OPENED		(0xD28C379A)

#define SRAM_HEADER_VERSION_OFFSET	(0x08)
#define SRAM_HEADER_BDR_PTR1_OFFSET	(0x0C)
#define SRAM_HEADER_BDR_PTR2_OFFSET	(0x10)
#define SRAM_HEADER_BDR_PTR3_OFFSET	(0x14)
#define SRAM_HEADER_BDR_PTR4_OFFSET	(0x18)
#define SRAM_HEADER_SIZE		(0x40)	/* total size of header block (multiple of 4 bytes) */

/* SRAM context offsets */
#define SRAM_CTX_AUTHKEY_OFFSET		(0x00)
#define SRAM_CTX_ENCKEY_OFFSET		(0x14)
#define SRAM_CTX_SEQH_OFFSET		(0x24)
#define SRAM_CTX_FLAGS_OFFSET		(0x28)	/* 1 byte */
#define CTX_FLAG_ENC_DIR		(0x00)
#define CTX_FLAG_DEC_DIR		(0x80)
#define CTX_FLAG_ESN_ON			(0x40)	/* Enhanced seq number */
#define CTX_FLAG_ESN_OFF		(0x00)	/* Normal seq number */
#define CTX_FLAG_AUTH_ON		(0x20)
#define CTX_FLAG_NON_AUTH		(0x00)
#define CTX_FLAG_ENC_ON			(0x10)
#define CTX_FLAG_NULL_ENC		(0x00)

#define SRAM_CTX_FREE_STATUS_OFFSET	(0x2C)	/* MV ADDED: context status */
#define CTX_FREE			(0xDEADDEAD)	/* Free status */
#define CTX_IN_USE			(0xBEEFBEEF)	/* In use status */

#define SRAM_CTX_SIZE			(0x30)	/* total size of each crypto context structure (multiple of 4 bytes) */

#define SRAM_CTX_MAX_NUMBER		(10)	/* maximum number of contexts */

/* SRAM buffer descriptor BD offsets */
#define SRAM_BD_CONTROL_FLAGS_OFFSET	(0x00)	/* 1 byte */
#define	BD_WRAP_BIT			(0x80)
#define BD_STATUS_FREE			(0x00)	/* 2 bits */
#define BD_STATUS_READY			(0x01)	/* 2 bits */
#define BD_STATUS_PROCESSING		(0x02)	/* 2 bits */
#define BD_STATUS_PROCESSED		(0x03)	/* 2 bits */
#define BD_STATUS_MASK			(0x03)

#define SRAM_BD_ERROR_FLAGS_OFFSET	(0x01)	/* 1 byte */
#define SRAM_BD_ENC_DEC_LENGTH_OFFSET	(0x02)	/* 2 bytes */
#define SRAM_BD_CTX_PTR_OFFSET		(0x04)	/* 2 bytes */
#define SRAM_BD_PDU_PTR_OFFSET		(0x06)	/* 2 bytes */
#define SRAM_BD_AEAD_REQ_PTR_OFFSET	(0x08)	/* MV ADDED: AEAD request pointer */
#define SRAM_BD_SIZE			(0x10)	/* total size of each buffer descriptor (multiple of 4 bytes) */

#ifdef USE_TWO_BDRS
#define SRAM_BD_RING1_NUMBER		(40)	/* number of BDs in ring #1 */	/* encrypt */
#define SRAM_BD_RING2_NUMBER		(40)	/* number of BDs in ring #2 */	/* decrypt */
#else
#define SRAM_BD_RING1_NUMBER		(80)	/* number of BDs in ring #1 */	/* encrypt and decrypt */
#define SRAM_BD_RING2_NUMBER		(1)	/* number of BDs in ring #2 */	/* not used */
#endif
#define SRAM_BD_RING3_NUMBER		(1)	/* number of BDs in ring #3 */
#define SRAM_BD_RING4_NUMBER		(1)	/* number of BDs in ring #4 */

/* PDU */
#define SRAM_PDU_HEADER_SIZE		(8)	/* Reserve 8 bytes for a PDU header for the ARM's usage */
#define SRAM_PDU_HDR_PTR_OFFSET		(0)	/* Header field contains an ARM pointer to the next PDU */
#define SRAM_PDU_HDR_STATE_OFFSET	(4)  	/* Header field containing state flag */
#define PDU_HDR_STATE_FREE		(0xDEADDED0)	/* lowest nibble resevered for holding the BDR num */
#define PDU_HDR_STATE_IN_USE		(0xBEEFF0D0)	/* lowest nibble resevered for holding the BDR num */
#define PDU_HDR_STATE_POOL_WRAP		(0xD0BACED0)	/* lowest nibble resevered for holding the BDR num */
#define PDU_HDR_STATE_MASK		(0xFFFFFFF0)	/* lowest nibble resevered for holding the BDR num */
#define PDU_HDR_STATE_BDR_MASK		(0x0000000F)
/* offsets relative to PDU body after the PDU header */
#define SRAM_PDU_ASSOC_OFFSET		(0)	/* ASSOC data offset in the PDU eg. ESP header*/
#define SRAM_PDU_IV_OFFSET		(8)	/* IV data offset in the PDU */
#define SRAM_PDU_AUTHENC_OFFSET 	(8+16)	/* AUTHENC data offset in the PDU */
#define PDU_ASSOC_IV_LENGTH		(8+16)	/* associated field + IV field lengths in PDU */
#define PDU_AUTH_FIELD_LENGTH		(12)	/* length of the authentication field in the PDU */

/* max PDU size in bytes */
#define PDU_MAX_SIZE		( PDU_ASSOC_IV_LENGTH + PICO_MAX_ENC_LENGTH + PDU_AUTH_FIELD_LENGTH )

/* driver transit states, used in atomic variable */
#define	DRIVER_ALL_SW		(0)	/* ( input is SW, output is SW ) All activity is using SW based encryption/decryption */
#define DRIVER_SWITCH_TO_HW	(1) 	/* ( input is HW, output is SW ) Switching over to HW based encryption/decryption */
#define	DRIVER_ALL_HW		(2)	/* ( input is HW, output is HW ) All activity is using HW based encryption/decryption */
#define DRIVER_SWITCH_TO_SW	(3) 	/* ( input is SW, output is HW ) Switching over to SW based encryption/decryption */


/* This is probably overkill, but we allow all the BDs to complete at once. */
#define MAX_COMPLETED_AEADS (SRAM_BD_RING1_NUMBER+SRAM_BD_RING2_NUMBER+SRAM_BD_RING3_NUMBER+SRAM_BD_RING4_NUMBER)
//#define MAX_COMPLETED_AEADS 4

/********************
 * enum definitions *
 ********************/
enum bd_status {
	BD_DATA_PROCESSED,	/* by PicoArray */
	BD_NO_PROCESSED_DATA,
	BD_DATA_PENDING,
	BD_ERROR_OCCURRED
};

enum bdr_number {
	BDR_NUM_1 = 0,
	BDR_NUM_2,
	BDR_NUM_3,
	BDR_NUM_4,
	BDR_TOTAL_NUM	/* Total number of Buffer Descriptor Rings */
};

/* select 1 or 2 Buffer Descriptor Rings */ 
#ifdef USE_TWO_BDRS
#define BDR_NUM_FOR_ENC		BDR_NUM_1	/* BDR used for encryption */ 
#define BDR_NUM_FOR_DEC		BDR_NUM_2	/* BDR used for decryption */ 
#else
#define BDR_NUM_FOR_ENC		BDR_NUM_1	/* BDR used for encryption */ 
#define BDR_NUM_FOR_DEC		BDR_NUM_1	/* BDR used for decryption */ 
#endif

/*************************
 * structure definitions *
 *************************/

struct ctx_data {
	u32 auth_key[5];
	u32 aes_key[4];
	u32 seqh;
	u8 flags;
	u8 unused1;
	u8 unused2;
	u8 unused3;
	u32 state;
};

/*
 * Context management structure
 */
struct ctx_management {
	u32 context_base_address;	/* base address of the context structure block */
	u32 total_num_ctxs;		/* total number of contexts */
	u32 num_ctx_in_use;		/* number of active contexts */
	u32 last_free_ctx;		/* address of the last free context */ 
	struct ctx_data shadow[SRAM_CTX_MAX_NUMBER];
};


/*
 * Buffer Descriptor Ring management structure
 */
struct bdr_management {
	u32 bdr_base_address;		/* base address of the buffer descriptor ring */
	u32 total_num_bds;		/* total number of buffer descriptors in this ring */
	u32 num_bds_in_use;		/* number of active buffer descriptors */
	u32 last_free_bd;		/* address of the last free buffer descriptor */ 
	u32 current_bd_processed;	/* address of the current db to be processed */

	spinlock_t bdr_lock;		/* spinlock to lock async events */
	unsigned long bdr_flags;	/* irq flags for spinlock */
};


/*
 * PDU management structure
 */
struct pdu_management {
	u32 pdu_pool_base_address;	/* base address of the PDU pool */
	u32 pdu_pool_end_address;	/* end address of the PDU pool */
	u32 pdu_pool_size;		/* size of the PDU pool */
	u32 next_alloc_pdu;		/* address of the next free PDU to be allocated */ 
	u32 oldest_pdu;			/* address of the oldest PDU in the pool not yet freed */	
	u32 num_pdus_in_use;		/* metric to see how many PDUs are in use */
	u32 num_pdus_alloced;		/* metric to see how many PDUs have been allocated (some maybe marked free) */

	spinlock_t pdu_lock;		/* spinlock to lock async events */
	unsigned long pdu_flags;	/* irq flags for spinlock */
};


/*
 * When we recover AEAD requests from the picoArray we can read a few of them
 * with interrupts off, before we pass them back to the kernel with interrupts on.
 * We only need to have the pointer to the request and the status.
 */
struct picocrypto_aead_completion {
	u32 sram_pdu;
	struct aead_request *req;
	int err;
};


/*
 * Private driver data structure
 */
struct private_driver_data {
	u32 sram_size;						/* multiple of 4 bytes */	
	u32 phys_sram_base_address;				/* multiple of 4 bytes */
	void *vert_sram_base_address;				/* multiple of 4 bytes */
	u32 header_address;					/* multiple of 4 bytes */
	struct ctx_management context_info;			/* Context management */
	struct bdr_management bdr_info[ BDR_TOTAL_NUM ];	/* BDR rings */
	struct pdu_management pdu_info;				/* PDU management */

	struct list_head alg_list;	/* list of registered algorithms */

	spinlock_t driver_lock;		/* spinlock to lock async events */
	
	atomic_t number_of_pdus;
	atomic_t number_of_pdu_drops;
	atomic_t number_of_bd_1_pre_drops;
	atomic_t number_of_bd_2_pre_drops;
	atomic_t number_of_bd_1_drops;
	atomic_t number_of_bd_2_drops;
	atomic_t number_of_dec_size_drops;
	atomic_t number_of_enc_size_drops;
	atomic_t largest_dec_dropped;
	atomic_t largest_enc_dropped;
	u32 max_irq_off_time;	/* max ticks of HW clock that irqs are disabled */
	u32 max_test_time_1;	/* test timer 1 */
	u32 max_test_time_2;	/* test timer 2 */
	u32 max_test_time_3;	/* test timer 3 */
	u32 max_test_time_4;	/* test timer 4 */
	const char* culprit;

	struct picocrypto_aead_completion completed[MAX_COMPLETED_AEADS];
	int completed_head;
	int completed_tail;
	struct tasklet_struct	tasklet;
	
	struct mutex pico_state_mutex;	/* mutex for accessing the picoArray state variables */

	atomic_t driver_transit_state;		/* atomic state variable to inidicate transit state between */
						/* hw and sw encryption/decryption */
};


/* 
 * Private ctx data stored in the Kernel tfm structure.
 * Memory is alloced by __crypto_alloc_tfm() in crypto/apic.c
 */
struct private_ctx_data {
	struct private_driver_data *driver_info;
	u32 authsize;		/* size of the authentication field */
	u32 context_addr;	/* allocated SRAM context structure address */
	u8 authkey[ MAX_AUTHKEY_LEN ];	/* authentication key */
	u8 authkey_len;			/* authentication key length in bytes */
	u8 enckey[ MAX_ENCKEY_LEN ];	/* encryption key */
	u8 enckey_len;			/* encryption key length in bytes */
	char rawkey[ MAX_RAWKEY_LEN ];	/* TODO: check size. raw key for SW encrytion */
	int rawkey_len;			/* length of rawkey encoded for struct rtattr */ 
	u8 flags_set;			/* CTX_FLAGS_UNKNOWN = not set, CTX_FLAGS_SET = set */
	u8 enc_dec_dir;			/* CTX_FLAG_ENC_DIR for encryption, CTX_FLAG_DEC_DIR for decryption */
};


/***********************
 * function prototypes *
 ***********************/

extern void dump_sram_structures( struct private_driver_data *pdata );

extern u32 alloc_sram_context_structure( struct private_driver_data *pdata );
extern u32 free_sram_context_structure( struct private_driver_data *pdata, u32 allocated_ctx );
extern void write_sram_context_keys( struct private_driver_data *pdata, u32 allocated_ctx, const u8 *pall_keys, u8 authkeylen, u8 enckeylen );
extern void write_sram_context_seqh( struct private_driver_data *pdata, u32 allocated_ctx, u32 seqh );
extern void write_sram_context_flags( struct private_driver_data *pdata, u32 allocated_ctx, u8 flags );
extern u8 init_sram_structures( struct private_driver_data *pdata );
extern u32 alloc_sram_buffer_descriptor( struct bdr_management *pbdr_info, u16 *pwrap_bit_flag );
extern u32 finished_sram_buffer_descriptor( struct bdr_management *pbdr_info );
extern u32 get_current_sram_buffer_descriptor( struct bdr_management *pbdr_info );
extern void insert_bd_sram_aead_request( u32 allocated_bd, struct aead_request *req );
extern struct aead_request *get_bd_sram_aead_request( u32 allocated_bd );
extern void insert_bd_sram_enc_dec_len( u32 allocated_bd, u32 enc_dec_len );
extern void insert_bd_sram_context_address( u32 allocated_bd, u16 context_address );
extern void insert_bd_sram_pdu_address( u32 allocated_bd, u16 pdu_address );
extern u32 get_bd_sram_pdu_address( struct private_driver_data *pdata, u32 allocated_bd );
extern void mark_bd_sram_free( u32 allocated_bd );
extern void mark_bd_sram_ready( u32 allocated_bd, u16 wrap_bit_flag );
extern enum bd_status get_bd_sram_status( u32 allocated_bd );
extern u32 alloc_sram_pdu( struct pdu_management *ppdu_info, u32 pdu_size, enum bdr_number bdr_num );
extern u8 get_bdr_num_of_oldest_pdu( struct pdu_management *ppdu_info, enum bdr_number *pbdr_num );
extern u8 free_sram_pdu( struct pdu_management *ppdu_info, u32 allocated_pdu );
extern void copy_crypto_to_pdu_sram( u32 pdu_address, u8 *pcrypto, u32 length );
extern void copy_crypto_from_pdu_sram( u32 pdu_address, u8 *pcrypto, u32 length );
extern u8 cmp_crypto_from_pdu_sram( u32 pdu_address, u8 *pcrypto, u32 length );
extern u8 request_sram_pico_close( struct private_driver_data *pdata );
extern u8 request_sram_pico_open( struct private_driver_data *pdata );
extern u32 read_sram_confirmed_state( struct private_driver_data *pdata );
extern u8 read_pico_active_state( struct private_driver_data *pdata );
extern void init_sram_bdrs( struct private_driver_data *pdata );
extern void init_context_structure_block( struct private_driver_data *pdata, u8 number_contexts );

