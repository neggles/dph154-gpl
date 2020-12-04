/*
 * This file contains the PicoArray Crypto driver to facilitate IPsec crypto.
 *
 * File: picodriver.c
 * Author: Dean Jenkins <djenkins@mvista.com>
 *
 * 2008 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*
 * Ancestry acknowledgements (all GPL code):
 * talitos - Freescale Integrated Security Engine (SEC) device driver
 * Copyright (c) 2008 Freescale Semiconductor, Inc.
 *
 * Scatterlist Crypto API glue code copied from files with the following:
 * Copyright (c) 2006-2007 Herbert Xu <herbert@gondor.apana.org.au>
 */

/* kernel related headers */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/hw_random.h>

/* Kernel crypto related headers */
#include <linux/crypto.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/aes.h>
#include <crypto/sha.h>

#include <asm/io.h>

#if defined(CONFIG_ARCH_FIRECRACKER)
#  include <mach/pc20x/pc20x.h>
#  include <mach/pc20x/timer.h>
#elif defined(CONFIG_ARCH_PC302)
#  include <mach/pc302/pc302.h>
#  include <mach/pc302/timer.h>
#else  /* CONFIG_ARCH_... */
#  error "Unknown architecture"
#endif /* CONFIG_ARCH_... */

#include <mach/platform.h>
#include <mach/hardware.h>

#include "picocrypto.h"
#include "swcrypto.h"

/******************
 * local #defines *
 ******************/
#define CPETD_TIMER_NUM 3

#define DRVNAME	"pico-crypto-ipsec"
#define DRVVERS	"20091020"

#if defined(CONFIG_ARCH_FIRECRACKER)
#  define SRAM_PHYS_ADDRESS	(0x10000000)
#elif defined(CONFIG_ARCH_PC302)
#  define SRAM_PHYS_ADDRESS	(0x20000000)
#else  /* CONFIG_ARCH_... */
#  error "Unknown architecture"
#endif /* CONFIG_ARCH_... */

#define SRAM_LENGTH		(0x00010000)	/* 64KB */

#undef PDU_SRAM_CMP
/* #define PDU_SRAM_CMP	1	*/	/* TEST CODE: Compare src with pdu, NULL/NON only */

#undef DEBUG_DRIVER
/* #define DEBUG_DRIVER	1   */

#ifdef DEBUG_DRIVER
#define DRIVER_DEBUG(fmt,args...)	printk( fmt ,##args )
#else
#define DRIVER_DEBUG(fmt,args...)
#endif

#define DRIVER_INFO(fmt,args...)	printk( fmt ,##args )
#define DRIVER_ERROR(fmt,args...)	printk( fmt ,##args )

#if defined(CONFIG_ARCH_FIRECRACKER)
#  define TIMER_VAL_REG (IO_ADDRESS(PC20X_TIMERS_BASE + TIMER_N_CURRENT_VALUE_REG_OFFSET(CPETD_TIMER_NUM)))
#  define TIMER_CTL_REG (IO_ADDRESS(PC20X_TIMERS_BASE + TIMER_N_CONTROL_REG_OFFSET(CPETD_TIMER_NUM)))
#  define TIMER_CNT_REG (IO_ADDRESS(PC20X_TIMERS_BASE + TIMER_N_LOAD_COUNT_REG_OFFSET(CPETD_TIMER_NUM)))
#  define TIMER_CTL_REG_ENABLE_BIT (TIMER_ENABLE)
#elif defined(CONFIG_ARCH_PC302)
#  define TIMER_VAL_REG (IO_ADDRESS(PC302_TIMER_BASE + TIMERNCURRENTVALUEREGOFFSET(CPETD_TIMER_NUM)))
#  define TIMER_CTL_REG (IO_ADDRESS(PC302_TIMER_BASE + TIMERNCONTROLREGOFFSET(CPETD_TIMER_NUM)))
#  define TIMER_CNT_REG (IO_ADDRESS(PC302_TIMER_BASE + TIMERNLOADCOUNTREGOFFSET(CPETD_TIMER_NUM)))
#  define TIMER_CTL_REG_ENABLE_BIT (TIMERENABLE)
#else  /* CONFIG_ARCH_... */
#  error "Unknown architecture"
#endif /* CONFIG_ARCH_... */
/*******************************
 * local structure definitions *
 *******************************/

/*
 * structure that holds info about a driver algorithm
 */
struct picocrypto_alg_template {
	char name[ CRYPTO_MAX_ALG_NAME ];
	char driver_name[ CRYPTO_MAX_ALG_NAME ];
	u32 priority;
	u32 blocksize;
	struct aead_alg aead;
};


/*
 * an algorithm record structure for registration list
 * also allow driver info to be retrieved
 */
struct picocrypto_alg_record {
	struct list_head entry;		/* allow records to be linked together */
	struct crypto_alg crypto_alg;	/* Kernel API algorithm structure contain our alg info */
	struct private_driver_data *driver_info;	/* driver information */
};

/****************************
 * Local forward prototypes *
 ****************************/
static struct picocrypto_alg_record *picocrypto_alg_alloc( struct private_driver_data *pdata,
	struct picocrypto_alg_template *template );

static int init_private_ctx_data(struct crypto_tfm *tfm);
static void release_private_ctx_data(struct crypto_tfm *tfm);

static int picocrypto_setkey( struct crypto_aead *authenc, const u8 *key,
	unsigned int keylen );
static int picocrypto_setauthsize( struct crypto_aead *authenc,
	unsigned int authsize );

static int picocrypto_encrypt( struct aead_request *req );
static int picocrypto_decrypt( struct aead_request *req );
static int picocrypto_givencrypt( struct aead_givcrypt_request *req );
static int picocrypto_encrypt_non( struct aead_request *req );
static int picocrypto_decrypt_non( struct aead_request *req );
static int picocrypto_givencrypt_non( struct aead_givcrypt_request *req );
static int picocrypto_encrypt_null( struct aead_request *req );
static int picocrypto_decrypt_null( struct aead_request *req );
static int picocrypto_givencrypt_null( struct aead_givcrypt_request *req );
static int picocrypto_encrypt_null_non( struct aead_request *req );
static int picocrypto_decrypt_null_non( struct aead_request *req );
static int picocrypto_givencrypt_null_non( struct aead_givcrypt_request *req );


/*********************************
 * Scatterlist support functions *
 *********************************/

/*
 * derive number of elements in scatterlist
 */
static int sg_count(struct scatterlist *sg_list, int nbytes)
{
	struct scatterlist *sg = sg_list;
	int sg_nents = 0;

	/* MV fixed bug, nbytes can go negative if whole segment length not used */
	while ( ( nbytes > 0 ) && ( sg != NULL ) ) {
		sg_nents++;
		nbytes -= sg->length;
		sg = sg_next( sg );
	}

	if( nbytes > 0 ) {
		DRIVER_ERROR( KERN_ERR "ERROR: sg_count(), total scatterlist length too short\n" );
	}

	return ( sg_nents );
}


/*********************************
 * Crypto AEAD API to the kernel *
 *********************************/

/*
 * Array that holds info about all the driver/PicoArray algorithms
 */
static struct picocrypto_alg_template crypto_driver_algs[] = {
	/*
	 * NORMAL MODE
	 * added for supporting CBC-AES-128 enc and HMAC-SHA-1-96 auth 
	 */
	{
		.name = "authenc(hmac(sha1),cbc(aes))",
		.driver_name = "authenc-hmac-sha1-cbc-aes-picocrypto",
		.priority = 2000,
		.blocksize = AES_BLOCK_SIZE,
		.aead = {
			.setkey = picocrypto_setkey,
			.setauthsize = picocrypto_setauthsize,
			.encrypt = picocrypto_encrypt,
			.decrypt = picocrypto_decrypt,
			.givencrypt = picocrypto_givencrypt,
			.geniv = "<built-in>",
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
			},
	},
	/*
	 * TEST MODE
	 * added for supporting CBC-AES-128 enc and NON auth 
	 */
	{
		.name = "authenc(digest_null,cbc(aes))",
		.driver_name = "authenc-digest-null-cbc-aes-picocrypto",
		.priority = 2000,
		.blocksize = AES_BLOCK_SIZE,
		.aead = {
			.setkey = picocrypto_setkey,
			.setauthsize = picocrypto_setauthsize,
			.encrypt = picocrypto_encrypt_non,
			.decrypt = picocrypto_decrypt_non,
			.givencrypt = picocrypto_givencrypt_non,
			.geniv = "<built-in>",
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = 0,	/* NON Authentication */
		},
	},
	/*
	 * TEST MODE
	 * added for supporting NULL enc and HMAC-SHA-1-96 auth 
	 */
	{
		.name = "authenc(hmac(sha1),ecb(cipher_null))",
		.driver_name = "authenc-hmac-sha1-ecb-null-picocrypto",
		.priority = 2000,
		.blocksize = 1,	/* NULL_BLOCK_SIZE */
		.aead = {
			.setkey = picocrypto_setkey,
			.setauthsize = picocrypto_setauthsize,
			.encrypt = picocrypto_encrypt_null,
			.decrypt = picocrypto_decrypt_null,
			.givencrypt = picocrypto_givencrypt_null,
			.geniv = "<built-in>",
			.ivsize = 0,		/* NULL Encryption */
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
	},
	/*
	 * TEST MODE
	 * added for supporting NULL enc and NON auth 
	 */
	{
		.name = "authenc(digest_null,ecb(cipher_null))",
		.driver_name = "authenc-digest-null-ecb-null-picocrypto",
		.priority = 2000,
		.blocksize = 1,	/* NULL_BLOCK_SIZE */
		.aead = {
			.setkey = picocrypto_setkey,
			.setauthsize = picocrypto_setauthsize,
			.encrypt = picocrypto_encrypt_null_non,
			.decrypt = picocrypto_decrypt_null_non,
			.givencrypt = picocrypto_givencrypt_null_non,
			.geniv = "<built-in>",
			.ivsize = 0,		/* NULL Encryption */
			.maxauthsize = 0,	/* NON Authentication */
		},
	},
};


/*
 * Allocate a picocrypto record to hold the alogithm details before registration
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 *	template = supported algorithm details ready for registration
 * Outputs:
 *	pdata structure updated with linked list of registered crypto
 *	algorithms
 * Return:
 *	0 = success, otherwise an error has occurred 
 */
static struct picocrypto_alg_record *picocrypto_alg_alloc( struct private_driver_data *pdata,
	struct picocrypto_alg_template *template )
{
	struct picocrypto_alg_record *palg;	/* points to our local record */
	struct crypto_alg *pkalg;	/* points to a Kernel algorithm structure */

	/* allocate a record */
	palg = kzalloc( sizeof( struct picocrypto_alg_record ), GFP_KERNEL );
	if ( palg == NULL ) {
		/* allocation failed */
		return NULL;
	}

	/*
	 * allow other our functions to access the private data
	 * by using container of
	 */
	palg->driver_info = pdata;

	/* point to Kernel algorithm structure in the record */
	pkalg = &palg->crypto_alg;

	/* insert the name of the algorithm */
	snprintf( pkalg->cra_name, CRYPTO_MAX_ALG_NAME, "%s", template->name );
	snprintf( pkalg->cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s",
		template->driver_name);

	/* copy details from the template and enter fixed settings */
	pkalg->cra_module = THIS_MODULE;
	pkalg->cra_priority = template->priority;
	pkalg->cra_flags = CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC;
	pkalg->cra_blocksize = template->blocksize;
	pkalg->cra_alignmask = 0;
	pkalg->cra_type = &crypto_aead_type;	/* Pick the AEAD Kernel API */
	/* tell the API to allocate memory for our private ctx data */
	pkalg->cra_ctxsize = sizeof( struct private_ctx_data );
	pkalg->cra_init = init_private_ctx_data;	/* init function for our private ctx data */
	pkalg->cra_exit = release_private_ctx_data;	/* release function for our private ctx data */
	pkalg->cra_u.aead = template->aead;	/* copy our AEAD settings */

	return palg;
}

/*
 * Register the driver's crypto algorithms using the Kernel's Crypto API
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	pdata structure updated with linked list of registered crypto
 *	algorithms
 * Return:
 *	0 = success, otherwise an error has occurred 
 */
static int register_driver_crypto( struct private_driver_data *pdata )
{
	int err;
	int i;

	DRIVER_DEBUG( KERN_INFO "Registering the driver's crypto algorithms...\n" );

	WARN_ON( pdata == NULL );

	if ( pdata == NULL ) {
		/* pointer to private data structure */
		return -ENOMEM;
	}

	/* initialise the linked list */
	INIT_LIST_HEAD( &pdata->alg_list );

	/* register the crypto algorithms that the PicoArray supports */
	for (i = 0; i < ARRAY_SIZE( crypto_driver_algs ); i++) {
		struct picocrypto_alg_record *palg;

		/* allocate an algorithm record for the linked list */
		palg = picocrypto_alg_alloc( pdata, &crypto_driver_algs[i] );
		if ( palg == NULL ) {
			err = -ENOMEM;
			goto err_out;
		}

		/* Call the Kernel API to register this algorithm. */
		/* container of can be used to get back palg */
		err = crypto_register_alg( &palg->crypto_alg );
		if ( err ) {
			/* failed to register the algorithm */
			DRIVER_ERROR( KERN_ERR "ERROR: Failed to register driver algorithm: %s\n",
				palg->crypto_alg.cra_name );
			kfree( palg );

			err = -EPERM;
			goto err_out;

		} else {
			/* add the registered algorithm record to the linked list */
			list_add_tail( &palg->entry, &pdata->alg_list );
		}
	}

	return 0;

/* error handlers */
err_out:
	return err;

}


/*
 * Unregister the driver's crypto algorithms using the Kernel's Crypto API
 * Inputs:
 * 	pdata = pointer to private data structure (not NULL)
 * Outputs:
 *	pdata structure updated with linked list of unregistered crypto
 *	algorithms
 * Return:
 *	0 = success, otherwise an error has occurred 
 */
static int unregister_driver_crypto( struct private_driver_data *pdata )
{
	struct picocrypto_alg_record *palg;
	struct picocrypto_alg_record *pnext;

	DRIVER_DEBUG( KERN_INFO "Unregistering the driver's crypto algorithms...\n" );

	WARN_ON( pdata == NULL );

	if ( pdata == NULL ) {
		/* pointer to private data structure */
		return -ENOMEM;
	}

	/* Unregister the algorithms by spinning through the linked list */
	list_for_each_entry_safe( palg, pnext, &pdata->alg_list, entry ) {
		crypto_unregister_alg( &palg->crypto_alg );
		list_del( &palg->entry );
		kfree( palg );
	}

	return 0;
}


/**********************************
 * Driver's Crypto AEAD functions *
 **********************************/

/*
 * Function to initialise our private context data, called during the
 * keying and rekeying process of IKEv2.
 * Called once per transform instance
 * Inputs:
 * 	tfm = pointer to transform structure
 * Outputs:
 *	Initialises our private ctx data at the end of the tfm structure
 * Return:
 *	0 = success, otherwise an error has occurred 
 */
static int init_private_ctx_data( struct crypto_tfm *tfm )
{
	/* get a pointer to the picocrypto algorithm record */
	unsigned long irq_flags;
	u32 time_measure;
	struct crypto_alg *alg = tfm->__crt_alg;
	struct picocrypto_alg_record *precord =
		 container_of( alg, struct picocrypto_alg_record, crypto_alg );

	/*
	 * get a pointer to the unitialised private context
	 * data allocated from the kernel
	 */
	struct private_ctx_data *pctx = crypto_tfm_ctx( tfm );

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	/*
	 * store the private driver info pointer into the context data
	 * to allow crypto functions to have access to it
	 */
	pctx->driver_info = precord->driver_info;

	/* protect critical code from interruptig context alloc and release calls */
	spin_lock_irqsave( &pctx->driver_info->driver_lock, irq_flags );
	time_measure = ioread32(__io(TIMER_VAL_REG));

	/* allocate a SRAM context structure */
	pctx->context_addr = alloc_sram_context_structure( pctx->driver_info );

	time_measure -= ioread32(__io(TIMER_VAL_REG));
	if (pctx->driver_info->max_irq_off_time < time_measure) {
		pctx->driver_info->max_irq_off_time = time_measure;
		pctx->driver_info->culprit = "init_private_ctx_data";
	}
	spin_unlock_irqrestore( &pctx->driver_info->driver_lock, irq_flags );

	if ( pctx->context_addr == 0 ) {
		DRIVER_ERROR( KERN_ERR "ERROR: Failed to allocate a SRAM context structure\n" );
	}

	return 0;
}


/*
 * Function to release our private context data when the keys are dead.
 * Called once per transform instance
 * Inputs:
 * 	tfm = pointer to transform structure
 * Outputs:
 *	Context data is dead
 * Return:
 *	None 
 */
static void release_private_ctx_data(struct crypto_tfm *tfm)
{
	unsigned long irq_flags;
	u32 time_measure;
	struct private_ctx_data *pctx = crypto_tfm_ctx( tfm );

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	/* protect critical code from interruptig context alloc and release calls */
	spin_lock_irqsave( &pctx->driver_info->driver_lock, irq_flags );
	time_measure = ioread32(__io(TIMER_VAL_REG));

	/* free the SRAM context structure */
	free_sram_context_structure( pctx->driver_info, pctx->context_addr );

	time_measure -= ioread32(__io(TIMER_VAL_REG));
	if (pctx->driver_info->max_irq_off_time < time_measure) {
		pctx->driver_info->max_irq_off_time = time_measure;
		pctx->driver_info->culprit = "release_private_ctx_data";
	}
	spin_unlock_irqrestore( &pctx->driver_info->driver_lock, irq_flags );

	pctx->context_addr = 0;
}


/*
 * Function to set the keys, called during the
 * algorithm selection process by upper layers. eg. IKEv2.
 * Called once per transform instance
 * Inputs:
 * 	authenc = pointer to authentication and encryption details
 *	all_keys = to rtattr struct and needs to use RTA macros
 *		to allow dereferencing.
 *		all_key contains the encryption key followed by the auth key.
 *	total_keylen = key length and needs to be corrected using RTA macros
 * Outputs:
 *	Update our private ctx data with new authentication and encryption
 *	details.
 * Return:
 *	0 = success, otherwise an error has occurred 
 */
static int picocrypto_setkey( struct crypto_aead *authenc, const u8 *all_keys,
	unsigned int total_keylen )
{
	/* get access to the private context data for this transform */
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	/* use rtnetlink info from include/linux/rtnetlink.h */
	struct rtattr *rta = (void *)all_keys;
	struct crypto_authenc_key_param *param;

	unsigned int authkeylen;
	unsigned int enckeylen;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if ( !RTA_OK(rta, total_keylen) ) {
		DRIVER_ERROR( KERN_ERR "BADKEY: failed OK\n" );
		goto badkey;
	}

	if ( rta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM ) {
		DRIVER_ERROR( KERN_ERR "BADKEY: failed TYPE\n" );
		goto badkey;
	}

	if ( RTA_PAYLOAD(rta) < sizeof(*param) ) {
		DRIVER_ERROR( KERN_ERR "BADKEY: failed SIZEOF\n" );
		goto badkey;
	}

	/* save the raw keys in the context for use with software based encryption */
	if ( ( sizeof( struct rtattr ) + total_keylen ) <= MAX_RAWKEY_LEN ) {
		memcpy( &pctx->rawkey[0], &all_keys[ 0 ], ( sizeof( struct rtattr ) + total_keylen ) );
		pctx->rawkey_len = total_keylen;
	} else {
		DRIVER_ERROR( KERN_ERR "BADKEY: rawkey array too small: rtattr = %d, total_keylen = %d, max = %d\n",
			sizeof( struct rtattr ), total_keylen, MAX_RAWKEY_LEN );
		goto badkey;
	}

	param = RTA_DATA( rta );
	enckeylen = be32_to_cpu(param->enckeylen);

	/* update the pointers */
	all_keys += RTA_ALIGN( rta->rta_len );
	total_keylen -= RTA_ALIGN( rta->rta_len );

	if ( total_keylen < enckeylen ) {
		DRIVER_ERROR( KERN_ERR "BADKEY: failed length check\n" );
		goto badkey;
	}

	/* calculate the auth key length */
	authkeylen = total_keylen - enckeylen;

	DRIVER_DEBUG( KERN_INFO "setkey: total keylen %d\n", total_keylen );
	DRIVER_DEBUG( KERN_INFO "setkey: enckeylen %d bits\n", enckeylen * 8 );
	DRIVER_DEBUG( KERN_INFO "setkey: authkeylen %d bits\n", authkeylen * 8 );

	if ( authkeylen > MAX_AUTHKEY_LEN ) {
		DRIVER_ERROR( KERN_ERR "BADKEY: authentication key too long\n" );
		goto badkey;
	}

	if ( enckeylen > MAX_ENCKEY_LEN ) {
		DRIVER_ERROR( KERN_ERR "BADKEY: encryption key too long\n" );
		goto badkey;
	}

	/* grab the keys and put into crypto SW context */
	memcpy( &pctx->authkey[0], &all_keys[ 0 ], authkeylen );
	pctx->authkey_len = authkeylen;

	/* grab the encryption key and put into crypto SW context */
	memcpy( &pctx->enckey[0], &all_keys[ authkeylen ], enckeylen );
	pctx->enckey_len = enckeylen;

	/* indicate that the crypto direction is unknown */
	pctx->flags_set = CTX_FLAGS_UNKNOWN;

	/* grab the keys and put into PicoArray SRAM context */
	write_sram_context_keys( pctx->driver_info, pctx->context_addr, &all_keys[0], authkeylen, enckeylen );

	return 0;

badkey:
	DRIVER_ERROR( KERN_ERR "Received a BADKEY\n" );
	crypto_aead_set_flags( authenc, CRYPTO_TFM_RES_BAD_KEY_LEN );
	return -EINVAL;
}
	
/*
 * Function to set the authsize
 * Inputs:
 *	authenc = session info
 * 	authsize = size of the authentication field
 * Outputs:
 *	Update our private ctx data with new authentication tag size.
 * Return:
 *	0 = success, otherwise an error has occurred 
 */
static int picocrypto_setauthsize( struct crypto_aead *authenc,
	unsigned int authsize )
{
	/* get access to the private data for this transform */
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	DRIVER_DEBUG( KERN_INFO "setauth: authsize %d bits\n", authsize * 8 );

	/* save the size of the authentication field */
	pctx->authsize = authsize;

	return 0;
}


/***************************
 * Common Enc/Dec routines *
 ***************************/

/*
 * Function to copy assoc crypto data into the PDU
 * Inputs:
 *	req = AEAD request information
 *	sram_pdu = address of the allocated PDU
 * Outputs:
 *	Copies the assoc crypto data into the PDU
 * Return:
 *	None.
 */
static void copy_assoc_to_pdu( struct aead_request *req, u32 sram_pdu )
{
	struct scatterlist *sg = req->assoc;
	u32 assoc_len = req->assoclen;	/* total length of the associated data */
	u32 seqment_tot = sg_count( sg, assoc_len );	/* number of segments */
	u8 *passoc_data;
	u32 seqment_len;
	u32 pdu_pos = SRAM_PDU_ASSOC_OFFSET;	/* start at the assoc field in the PDU */

	while ( ( seqment_tot > 0 ) && ( assoc_len > 0 )  ) {
		/* get a pointer to the assoc data */
		passoc_data = sg_virt( sg );

		/* get the length of this seqment */
		seqment_len = sg->length;

		/* assoc length might be shorter than the segment length */
		if ( assoc_len < seqment_len ) {
			seqment_len = assoc_len;
		}

		/* copy this segment to the PDU */
		copy_crypto_to_pdu_sram( sram_pdu + pdu_pos, passoc_data, seqment_len );

		/* update the assoc length */
		assoc_len -= seqment_len;

		/* move pdu pos onto the next location */ 
		pdu_pos += seqment_len;

		/* move to the next segment */
		sg = sg_next(sg);
		seqment_tot--;
	}
} 


/*
 * Function to copy IV crypto data into the PDU
 * Inputs:
 *	req = AEAD request information
 *	sram_pdu = address of the allocated PDU
 * Outputs:
 *	Copies the IV crypto data into the PDU
 *	IV field may be empty but copy anyway
 * Return:
 *	None.
 */
static void copy_iv_to_pdu( struct aead_request *req, u32 sram_pdu )
{
	u8 *piv_data = req->iv;

	/* copy the IV data to the PDU */
	copy_crypto_to_pdu_sram( sram_pdu + SRAM_PDU_IV_OFFSET, piv_data, AES_BLOCK_SIZE );
} 


/*
 * Function to copy enc/dec + auth crypto data into the PDU
 * Inputs:
 *	req = AEAD request information
 *	sram_pdu = address of the allocated PDU
 * 	data_length = length of enc_dec + auth size
 * Outputs:
 *	Copies the enc/dec + auth crypto data into the PDU
 * Return:
 *	None.
 * Note do not use req->cryptlen as the length is different for encryption and decryption.
 */
static void copy_authenc_to_pdu( struct aead_request *req, u32 sram_pdu, u32 data_length )
{
	struct scatterlist *sg = req->src;

	u32 crypt_len = data_length;
	u32 seqment_tot = sg_count( sg, crypt_len );	/* number of segments */
	u8 *psrc_data;
	u32 seqment_len;
	u32 pdu_pos = SRAM_PDU_AUTHENC_OFFSET;	/* start at the AUTHENC field in the PDU */

	while ( ( seqment_tot > 0 ) && ( crypt_len > 0 )  ) {
		/* get a pointer to the src data */
		psrc_data = sg_virt( sg );

		/* get the length of this seqment */
		seqment_len = sg->length;

		/* crypt length might be shorter than the segment length */
		if ( crypt_len < seqment_len ) {
			seqment_len = crypt_len;
		}

		/* copy this segment to the PDU */
		copy_crypto_to_pdu_sram( sram_pdu + pdu_pos, psrc_data, seqment_len );

		/* TEST CODE: NUKE the src/dst data to check copy from PDU works */
#ifndef PDU_SRAM_CMP
		memset( psrc_data, 0, seqment_len );
#endif
		/* update the crypt length */
		crypt_len -= seqment_len;

		/* move pdu pos onto the next location */ 
		pdu_pos += seqment_len;

		/* move to the next segment */
		sg = sg_next(sg);
		seqment_tot--;
	}

	if ( crypt_len != 0 ) {
		DRIVER_ERROR( KERN_ERR "ERROR: Failed to copy all crypto data from src\n" );
	}
} 


/*
 * Function to copy enc/dec + auth crypto data from the PDU
 * Inputs:
 *	req = AEAD request information
 *	sram_pdu = address of the allocated PDU
 * Outputs:
 *	Copies the enc/dec + auth crypto data from the PDU
 * Return:
 *	None.
 */
static void copy_authenc_from_pdu( struct aead_request *req, u32 sram_pdu )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	struct scatterlist *sg = req->dst;
	u32 crypt_len; 
	u32 seqment_tot;	/* total number of segments */
	u8 *pdst_data;
	u32 seqment_len;
	u32 pdu_pos = SRAM_PDU_AUTHENC_OFFSET;	/* start at the AUTHENC field in the PDU */

	/* calculate the length of the (plaintext or ciphertext) plus authsize ( enc only ) */
	if ( pctx->enc_dec_dir == CTX_FLAG_DEC_DIR ) {
		/* for decryption need to modify cryptlen to remove the length of the auth field */
		crypt_len = req->cryptlen - pctx->authsize;
	} else {
		/* for encryption need to modify cryptlen to add the legth of the auth field */
		crypt_len = req->cryptlen + pctx->authsize;
	}

	seqment_tot = sg_count( sg, crypt_len );	/* number of segments */

	while ( ( seqment_tot > 0 ) && ( crypt_len > 0 )  ) {
		/* get a pointer to the dst data */
		pdst_data = sg_virt( sg );

		/* get the length of this seqment */
		seqment_len = sg->length;

		/* crypt length might be shorter than the segment length */
		if ( crypt_len < seqment_len ) {
			seqment_len = crypt_len;
		}

#ifndef PDU_SRAM_CMP
		/* copy from PDU to this segment */
		copy_crypto_from_pdu_sram( sram_pdu + pdu_pos, pdst_data, seqment_len );
#else
		{
			/* Test for corrupted PDU in SRAM */
			u8 res;
			res = cmp_crypto_from_pdu_sram( sram_pdu + pdu_pos, pdst_data, seqment_len );

			if ( res == 1 ) {
				DRIVER_ERROR( KERN_ERR "ERROR: PDU dst mismatch with src\n" );
			}
		}
#endif
		/* update the crypt length */
		crypt_len -= seqment_len;

		/* move pdu pos onto the next location */ 
		pdu_pos += seqment_len;

		/* move to the next segment */
		sg = sg_next(sg);
		seqment_tot--;
	}

	if ( crypt_len != 0 ) {
		DRIVER_ERROR( KERN_ERR "ERROR: Failed to copy all crypto data to dst\n" );
	}
} 


/*
 * MUST HAVE LOCK BEFORE CALLING
 * THIS FUNCTION NEEDS TO RE-ENTRANT
 * Function to process a BD
 * Inputs:
 *	pdata = pointer to the private driver data
 *	psram_bd = pointer to the BD sram variable
 *	bdr_num = number of the BD ring
 *	cleanup = 1 to indicate to clean up the AEAD req
 * Outputs:
 *	Checks the BD and processes any completed PDUs
 *	updates *psram_bd to point to the next BD
 *  updates *aead_pp to NULL or the aead request that's completed
 *  updates *aead_err_p to return code to pass to aead completion
 * Return:
 *	1 for processed or 0 for failure
 */
static u8 process_a_bd( struct private_driver_data *pdata, u32 *psram_bd, u8 bdr_num, u8 cleanup, struct aead_request **aead_pp, int *aead_err_p, u32* sram_pdu_p )
{
	/* re-entrancy check */
	struct aead_request *req;
	u32 sram_bd = *psram_bd;
	u32 sram_pdu;	/* address of the current PDU */
	enum bd_status bd_status; 
	u8 processed = 1;

	*aead_pp = NULL;
	*sram_pdu_p = 0;
	
	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( sram_bd == 0 ) {
		/* Buffer Descriptor must not be 0 */
		processed = 0;
		goto bad_bd;
	}

	/* get the status of the BD, has the PicoArray processed it ? */
	bd_status = get_bd_sram_status( sram_bd );

#ifdef PICO_FIRMWARE_NOT_AVAILABLE
	if ( cleanup != 1 ) {
		/* IP.ACCESS REMOVE WHEN PICOARRAY FIRMWARE IS AVAILABLE */ 
		bd_status = BD_DATA_PROCESSED;
	}
#endif

	switch ( bd_status ) {
	case BD_DATA_PROCESSED:
		/* PicoArray has completed the PDU so give back to Kernel */

		DRIVER_DEBUG( KERN_INFO "Called: %s: BD_DATA_PROCESSED\n", __FUNCTION__ );

		/* retrive the AEAD Request pointer from the buffer descriptor */
		req = get_bd_sram_aead_request( sram_bd );

		/* calculate PDU address */
		sram_pdu = get_bd_sram_pdu_address( pdata, sram_bd );

		DRIVER_DEBUG( KERN_INFO "Called: %s: BD_DATA_PROCESSED: PDU address 0x%08X\n", __FUNCTION__, sram_pdu );

		/* Indicate finished with current BD and get next BD to process */
		finished_sram_buffer_descriptor( &pdata->bdr_info[ bdr_num ] );

		if ( req != NULL ) {
			DRIVER_DEBUG( KERN_INFO "aead_request_complete called\n" );

			/* WARNING: CALLBACK CAN CAUSE RE-CURSION INTO THE ENCRYPTION/DECRYPTION FUNCTIONS */
			/* Therefore, make sure all SRAM handling for the current PDU is finished first */

			/* tell the Kernel crypto completed */
			*aead_pp = req;
			*aead_err_p = 0;
			*sram_pdu_p = sram_pdu;
		}
		else
		{
			DRIVER_DEBUG( KERN_INFO "Called: %s: BD_DATA_PROCESSED: before free: PDU address 0x%08X\n", __FUNCTION__, sram_pdu );

			/* free the PDU */
			if ( sram_pdu != 0 && free_sram_pdu( &pdata->pdu_info, sram_pdu ) != 0 ) {
				/* Free failed */
				/* TODO: Remove this test code */
				DRIVER_ERROR( KERN_ERR "DUMP: BD_DATA_PROCESSED\n" );
				dump_sram_structures( pdata );
				BUG();
			}
		}

		break;

	case BD_NO_PROCESSED_DATA:
		DRIVER_DEBUG( KERN_INFO "Called: %s: BD_NO_PROCESSED_DATA\n", __FUNCTION__ );

		/* no PDU completed so exit this BDR check */
		processed = 0;
		break;

	case BD_DATA_PENDING:
		if( cleanup == 1 ) {
			DRIVER_DEBUG( KERN_INFO "Called: %s: BD_DATA_PENDING: cleanup\n", __FUNCTION__ );

			/* process the AEAD Req to allow the driver to be removed */

			/* retrive the AEAD Request pointer from the buffer descriptor */
			req = get_bd_sram_aead_request( sram_bd );

			/* calculate PDU address */
			sram_pdu = get_bd_sram_pdu_address( pdata, sram_bd );

			/* free the PDU */
			if ( sram_pdu != 0 && free_sram_pdu( &pdata->pdu_info, sram_pdu ) != 0 ) {
				/* Free failed */
				/* TODO: Remove this test code */
				DRIVER_ERROR( KERN_ERR "DUMP: BD_DATA_PENDING\n" );
				dump_sram_structures( pdata );
			}

			/* Indicate finished with current BD and get next BD to process */
			finished_sram_buffer_descriptor( &pdata->bdr_info[ bdr_num ] );

			if ( req != NULL ) {

				DRIVER_DEBUG( KERN_INFO "Cleaned up an AEAD Req in a BD\n" );

				/* WARNING: CALLBACK CAN CAUSE RE-CURSION INTO THE ENCRYPTION/DECRYPTION FUNCTIONS */
				/* Therefore, make sure all SRAM handling for the current PDU is finished first */

				/* tell the Kernel crypto completed with error */
				*aead_pp = req;
				*aead_err_p = -EINVAL;
			}

		} else {
			DRIVER_DEBUG( KERN_INFO "Called: %s: BD_DATA_PENDING: picoArray\n", __FUNCTION__ );

			/* Allow the PicoArray to process this BD */
			processed = 0;
		}

		break;

	case BD_ERROR_OCCURRED:
		DRIVER_DEBUG( KERN_INFO "Called: %s: BD_ERROR_OCCURRED\n", __FUNCTION__ );

		/* PicoArray has reported an error so inform the Kernel */

		/* retrive the AEAD Request pointer from the buffer descriptor */
		req = get_bd_sram_aead_request( sram_bd );

		/* calculate PDU address */
		sram_pdu = get_bd_sram_pdu_address( pdata, sram_bd );

		/* free the PDU */
		if ( sram_pdu != 0 && free_sram_pdu( &pdata->pdu_info, sram_pdu ) != 0 ) {
			/* Free failed */
			/* TODO: Remove this test code */
			DRIVER_ERROR( KERN_ERR "DUMP: BD_ERROR_OCCURRED\n" );
			dump_sram_structures( pdata );
		}

		/* Indicate finished with current BD and get next BD to process */
		finished_sram_buffer_descriptor( &pdata->bdr_info[ bdr_num ] );

		if ( req != NULL ) {

			DRIVER_DEBUG( KERN_ERR "aead_request_complete called with error\n" );

			/* WARNING: CALLBACK CAN CAUSE RE-CURSION INTO THE ENCRYPTION/DECRYPTION FUNCTIONS */
			/* Therefore, make sure all SRAM handling for the current PDU is finished first */

			/* tell the Kernel crypto completed with error */
			*aead_pp = req;
			*aead_err_p = -EINVAL;
		}

		break;
	}

	/* get the new current BD after possible re-cursion */
	*psram_bd = get_current_sram_buffer_descriptor( &pdata->bdr_info[ bdr_num ] );

bad_bd:

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	/* check for re-entrancy */
	return ( processed );
}


/*
 * Function to check for any active the BDs
 * Inputs:
 *	pdata = pointer to the private driver data
 * Outputs:
 *	Checks each BDR for any active BD
 * Return:
 *	0 = no active BDs or 1 = an active BD has been found
 */
static u8 check_active_bds( struct private_driver_data *pdata )
{
	unsigned long irq_flags;
	u32 time_measure;
	u8 bdr_num;	/* BDR number */
	u32 sram_bd;	/* current sram buffer descriptor to process */
	u8 found_active_bd = 0;

	/* protect critical code from decrypt and encrypt calls */
	spin_lock_irqsave( &pdata->driver_lock, irq_flags );
	time_measure = ioread32(__io(TIMER_VAL_REG));

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	/* spin through each Buffer Descriptor Ring */
	for ( bdr_num = 0; bdr_num < BDR_TOTAL_NUM; bdr_num++ )
	{
		/* check Buffer Descriptor Ring for any active BDs */
		sram_bd = get_current_sram_buffer_descriptor( &pdata->bdr_info[ bdr_num ] );

		if ( sram_bd != 0 ) {
			/* found an active BD */
			found_active_bd = 1;
			break;
		}
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	time_measure -= ioread32(__io(TIMER_VAL_REG));
	if (pdata->max_irq_off_time < time_measure) {
		pdata->max_irq_off_time = time_measure;
		pdata->culprit = "check_active_bds";
	}
	spin_unlock_irqrestore( &pdata->driver_lock, irq_flags );

	return ( found_active_bd );
}


/*
 * MUST HAVE LOCK BEFORE USING THIS FUNCTION
 * Function to check whether there is a spare BD in a given BDR
 * Inputs:
 *	pdata = pointer to the private driver data
 *	bdr_num = number of the given BDR
 * Outputs:
 *	Checks to whether the maximum number of BDs is use
 * Return:
 *	0 = no free BDs or 1 = there is at least 1 free BD
 */
static inline u8 check_for_free_bd( struct private_driver_data *pdata, enum bdr_number bdr_num )
{
	u8 found_free_bd = 0;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if ( pdata->bdr_info[ bdr_num ].num_bds_in_use < pdata->bdr_info[ bdr_num ].total_num_bds ) {
		/* there is a free BD */
		found_free_bd = 1;
	} else {
		DRIVER_DEBUG( KERN_ERR "ERROR: BDR #%d is full\n", bdr_num + 1 );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( found_free_bd );
}


/*
 * Function to process the BDRs
 * Inputs:
 *	pdata = pointer to the private driver data
 *	cleanup = 1 to indicate to clean up any remaing AEAD reqs
 * Outputs:
 *	Checks each BDR and processes any completed PDUs
 * Return:
 *	None.
 */
static u8 process_bdrs( struct private_driver_data *pdata, u8 cleanup )
{
	/* re-entrancy check */
	unsigned long irq_flags;
	u32 time_measure;
	u32 test2;
	static atomic_t process_bdrs_check = ATOMIC_INIT(0);
	u8 bdr_num;	/* BDR number */
	u32 sram_bd;	/* current sram buffer descriptor to process */
	u8 exit;
	int next_completed;

	/* re-entrancy test */
	if ( atomic_read( &process_bdrs_check ) != 0 ) {
		DRIVER_DEBUG( KERN_ERR "WARNING: RE-ENTRANCY DETECTED in %s: returning to caller\n", __FUNCTION__ );
		return 0;
	}

	/* protect critical code from decrypt and encrypt calls */
	spin_lock_irqsave( &pdata->driver_lock, irq_flags );
	time_measure = ioread32(__io(TIMER_VAL_REG));

	/* check for re-entrancy */
	atomic_inc( &process_bdrs_check );

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	next_completed = pdata->completed_head+1;
	if (next_completed >= MAX_COMPLETED_AEADS)
	{
		next_completed = 0;
	}
	
	/* spin through each Buffer Descriptor Ring */
	for ( bdr_num = 0; bdr_num < BDR_TOTAL_NUM; bdr_num++ )
	{
		exit = 1;

		/* check Buffer Descriptor Ring for any active BDs */
		sram_bd = get_current_sram_buffer_descriptor( &pdata->bdr_info[ bdr_num ] );

		while ( ( sram_bd != 0 ) && ( exit == 1 ) && ( next_completed != pdata->completed_tail ) ) {

			/* process the BD, sram_bd updated with the next BD address */
			struct aead_request *req = NULL;
			int                 err = 0;
			u32                 sram_pdu = 0;
			
			test2 = ioread32(__io(TIMER_VAL_REG));
			exit = process_a_bd( pdata, &sram_bd, bdr_num, cleanup, &req, &err, &sram_pdu );
			test2 -= ioread32(__io(TIMER_VAL_REG));
			if (pdata->max_test_time_2 < test2 ) {
				pdata->max_test_time_2 = test2 ;
			}
			
			/* Accumulate completed aead requests */
			if (req != NULL) {
			
				int idx = pdata->completed_head;
				
				pdata->completed[idx].req = req;
				pdata->completed[idx].err = err;
				pdata->completed[idx].sram_pdu = sram_pdu;
				
				pdata->completed_head = next_completed;
				
				next_completed++;
				if (next_completed >= MAX_COMPLETED_AEADS)
				{
					next_completed = 0;
				}
			}
		}
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	/* check for re-entrancy */
	atomic_dec( &process_bdrs_check );

	time_measure -= ioread32(__io(TIMER_VAL_REG));
	if (pdata->max_irq_off_time < time_measure) {
		pdata->max_irq_off_time = time_measure;
		pdata->culprit = "process_bdrs search";
	}
	spin_unlock_irqrestore( &pdata->driver_lock, irq_flags );

	tasklet_schedule(&pdata->tasklet);
	
	return 0;
}
		
static void picocrypto_tasklet_callback(unsigned long data)
{
	struct private_driver_data *pdata = (struct private_driver_data *)data;
	unsigned long irq_flags;
	u32 time_measure;
	
	/* Once the interrupts have been re-enabled we can process all
	   the completed aead requests without the risk of calling a slow
	   path with interrupts disabled. */
	spin_lock_irqsave( &pdata->driver_lock, irq_flags );
	time_measure = ioread32(__io(TIMER_VAL_REG));
		
	while (pdata->completed_tail != pdata->completed_head) {
	
		struct aead_request *req     = pdata->completed[pdata->completed_tail].req;
		int                 err      = pdata->completed[pdata->completed_tail].err;
		u32                 sram_pdu = pdata->completed[pdata->completed_tail].sram_pdu;
		
		pdata->completed_tail++;
		if (pdata->completed_tail >= MAX_COMPLETED_AEADS)
		{
			pdata->completed_tail = 0;
		}
		
		time_measure -= ioread32(__io(TIMER_VAL_REG));
		if (pdata->max_irq_off_time < time_measure) {
			pdata->max_irq_off_time = time_measure;
			pdata->culprit = "picocrypto_tasklet_callback 1";
		}
		spin_unlock_irqrestore( &pdata->driver_lock, irq_flags );
	
		if ( sram_pdu ) {
		
			/* read the encrypted or decrypted data into the dst scatterlist */
			copy_authenc_from_pdu( req, sram_pdu );

			spin_lock_irqsave( &pdata->driver_lock, irq_flags );
			time_measure = ioread32(__io(TIMER_VAL_REG));
			
			/* free the PDU */
			if ( free_sram_pdu( &pdata->pdu_info, sram_pdu ) != 0 ) {
				/* Free failed */
				/* TODO: Remove this test code */
				DRIVER_ERROR( KERN_ERR "DUMP: BD_DATA_PROCESSED\n" );
				dump_sram_structures( pdata );
				BUG();
			}
			
			time_measure -= ioread32(__io(TIMER_VAL_REG));
			if (pdata->max_irq_off_time < time_measure) {
				pdata->max_irq_off_time = time_measure;
				pdata->culprit = "picocrypto_tasklet_callback 3";
			}
			spin_unlock_irqrestore( &pdata->driver_lock, irq_flags );
		}

		aead_request_complete( req, err );
		
		spin_lock_irqsave( &pdata->driver_lock, irq_flags );
		time_measure = ioread32(__io(TIMER_VAL_REG));
	}
	
	time_measure -= ioread32(__io(TIMER_VAL_REG));
	if (pdata->max_irq_off_time < time_measure) {
		pdata->max_irq_off_time = time_measure;
		pdata->culprit = "picocrypto_tasklet_callback 3";
	}
	spin_unlock_irqrestore( &pdata->driver_lock, irq_flags );
}


/*
 * Called in an Interrupt (ATOMIC) context so must not sleep.
 * This function is re-entrant
 * Function to add the crypto data to the Buffer Descriptor Ring
 * Inputs:
 *	req = AEAD request information
 *	pctx = pointer to private context information
 *	bdr_num = number of the BDR to use
 * Outputs:
 *	Inserts a Buffer Descriptor into a ring.
 *	Allocates a PDU and populates it with IV, ciphertext and auth data.
 * Return:
 *	-EINPROGRESS = added async offload OK, otherwise -EBUSY for an error has occurred 
 */
static int picocrypto_add_to_bdr( struct aead_request *req, struct private_ctx_data *pctx, enum bdr_number bdr_num )
{
	/* re-entrancy check */
	unsigned long irq_flags;
	u32 time_measure;
	u32 sram_bd;
	u32 sram_pdu;
	u32 pdu_size;
	u16 wrap_bit_flag;
	u8 *error_msg;
	u32 enc_dec_len;	/* plaintext or chiphertext length of the data */

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	atomic_inc( &pctx->driver_info->number_of_pdus );
	
	/* calculate the length of the plaintext or ciphertext */
	if ( pctx->enc_dec_dir == CTX_FLAG_DEC_DIR ) {
		/* for decryption need to modify cryptlen to remove the length of the auth field */
		enc_dec_len = req->cryptlen - pctx->authsize;
	} else {
		/* for encryption cryptlen does not include the auth field length so it correct */
		enc_dec_len = req->cryptlen;
	}

	/* check for exceeding maximum enclength of the PicoArray */
	if ( enc_dec_len > PICO_MAX_ENC_LENGTH ) {
		/* encryption/decryption data is too long for the PicoArray */
		if ( pctx->enc_dec_dir == CTX_FLAG_DEC_DIR ) {
			atomic_inc( &pctx->driver_info->number_of_dec_size_drops );
			if (enc_dec_len > atomic_read( &pctx->driver_info->largest_dec_dropped ))
			{
				atomic_set( &pctx->driver_info->largest_dec_dropped, enc_dec_len );
			}
		} else {
			atomic_inc( &pctx->driver_info->number_of_enc_size_drops );
			if (enc_dec_len > atomic_read( &pctx->driver_info->largest_enc_dropped ))
			{
				atomic_set( &pctx->driver_info->largest_enc_dropped, enc_dec_len );
			}
		}
		error_msg = "Max enc/dec length exceeded";
		goto invalid_request_error;
	}

	/* calculate the size of the PDU */
	pdu_size = PDU_ASSOC_IV_LENGTH + enc_dec_len + pctx->authsize;

	/* protect critical code from async events */
	spin_lock_irqsave( &pctx->driver_info->driver_lock, irq_flags );
	time_measure = ioread32(__io(TIMER_VAL_REG));

	/*
	 * first check that a BD is free before attempting to alloc a PDU,
	 * otherwise it wastes a PDU allocation
	 */
	if ( check_for_free_bd( pctx->driver_info, bdr_num ) == 0 ) {
		/* no free BD so abandon */
		error_msg = "No free BD";
		if (bdr_num) {
			atomic_inc( &pctx->driver_info->number_of_bd_2_pre_drops );
		} else {
			atomic_inc( &pctx->driver_info->number_of_bd_1_pre_drops );
		}
		goto no_bd_error;
	}

	/* Try allocating a new PDU but with no recovery action if no PDUs are available */ 

	/* try allocating a PDU for passing the crypto data to the SRAM */
	sram_pdu = alloc_sram_pdu( &pctx->driver_info->pdu_info, pdu_size, bdr_num );

	if ( sram_pdu == 0 ) {
		/* no PDUs available even after processing BDs in this ring */
		error_msg = "No PDUs available";
		atomic_inc( &pctx->driver_info->number_of_pdu_drops );
		goto no_pdu_error;
	}


	/* Get a BD from Buffer Descriptor ring */
	sram_bd = alloc_sram_buffer_descriptor( &pctx->driver_info->bdr_info[ bdr_num ], &wrap_bit_flag );

	if( sram_bd == 0 ) {
		/* no BDs available */
		if (bdr_num) {
			atomic_inc( &pctx->driver_info->number_of_bd_2_drops );
		} else {
			atomic_inc( &pctx->driver_info->number_of_bd_1_drops );
		}
		error_msg = "No BDs available";
		goto busy_error;
	}

	time_measure -= ioread32(__io(TIMER_VAL_REG));
	if (pctx->driver_info->max_irq_off_time < time_measure) {
		pctx->driver_info->max_irq_off_time = time_measure;
		pctx->driver_info->culprit = "picocrypto_add_to_bdr 1";
	}
	spin_unlock_irqrestore( &pctx->driver_info->driver_lock, irq_flags );

	/* insert the length in bytes of the encryption/decryption data into the bd */
	insert_bd_sram_enc_dec_len( sram_bd, enc_dec_len );

	/* insert the associated relative context address into the bd */
	insert_bd_sram_context_address( sram_bd, (u16) ( pctx->context_addr - pctx->driver_info->header_address ) );

	/* insert the AEAD Request pointer into the bd for later retrival */
	insert_bd_sram_aead_request( sram_bd, req );

	/* insert the relative address of the PDU into the bd */
	insert_bd_sram_pdu_address( sram_bd, (u16) (sram_pdu - pctx->driver_info->header_address ) );

	/* copy assoc crypto data into the PDU */
	copy_assoc_to_pdu( req, sram_pdu ); 

	/* copy IV crypto data into the PDU, may be empty */
	copy_iv_to_pdu( req, sram_pdu );

	/* copy authenc crypto data into the PDU */
	copy_authenc_to_pdu( req, sram_pdu, enc_dec_len + pctx->authsize ); 

	/* tell the PicoArray to process the buffer descriptor */
	mark_bd_sram_ready( sram_bd, wrap_bit_flag );

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	/* inidicate performing an async offload */
	return -EINPROGRESS;

busy_error:
	/* free the PDU ( in an out of sequence fashion ) */
	free_sram_pdu( &pctx->driver_info->pdu_info, sram_pdu );

no_bd_error:
no_pdu_error:

	time_measure -= ioread32(__io(TIMER_VAL_REG));
	if (pctx->driver_info->max_irq_off_time < time_measure) {
		pctx->driver_info->max_irq_off_time = time_measure;
		pctx->driver_info->culprit = "picocrypto_add_to_bdr 2";
	}
	spin_unlock_irqrestore( &pctx->driver_info->driver_lock, irq_flags );

invalid_request_error:

	DRIVER_DEBUG( KERN_ERR "ERROR: Dropped crypto packet during insertion to BDR: %s\n", error_msg );

	/* TEST CODE */
/*	dump_sram_structures( pctx->driver_info );	*/

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	/* indicate something went wrong and packet needs to be dropped */
	return -EBUSY;
}


/*****************************************
 * Decryption routines, IV data provided *
 *****************************************/

/*
 * Function to setup decryption of AES ciphertext plus HMAC auth field
 * Inputs:
 *	req = AEAD request information
 * Outputs:
 *	Update our private ctx data with decryption context flags.
 *	Inserts a Buffer Descriptor.
 *	Allocates a PDU and populates it with IV, ciphertext and auth data.
 * Return:
 *	-EINPROGRESS = added async offload OK, otherwise -EBUSY for an error has occurred 
 */
static int picocrypto_decrypt( struct aead_request *req )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW decryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This decryption has AES and HMAC */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_DEC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_AUTH_ON | CTX_FLAG_ENC_ON );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_DEC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first decryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( req, pctx, BDR_NUM_FOR_DEC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW decryption */
		result = swcrypto_do_it( req, SW_CRYPTO_AUTH, SW_DO_DEC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_decrypt_non( struct aead_request *req )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW decryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This decryption has AES and non auth */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_DEC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_NON_AUTH | CTX_FLAG_ENC_ON );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_DEC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first decryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( req, pctx, BDR_NUM_FOR_DEC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW decryption */
		result = swcrypto_do_it( req, SW_CRYPTO_NON, SW_DO_DEC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_decrypt_null( struct aead_request *req )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW decryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This decryption has null decryption and HMAC */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_DEC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_AUTH_ON | CTX_FLAG_NULL_ENC );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_DEC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first decryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( req, pctx, BDR_NUM_FOR_DEC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW decryption */
		result = swcrypto_do_it( req, SW_NULL_AUTH, SW_DO_DEC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_decrypt_null_non( struct aead_request *req )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW decryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This decryption has null decryption and non auth */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_DEC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_NON_AUTH | CTX_FLAG_NULL_ENC );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_DEC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first decryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( req, pctx, BDR_NUM_FOR_DEC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW decryption */
		result = swcrypto_do_it( req, SW_NULL_NON, SW_DO_DEC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


/********************************************
 * Encryption routines, IV data is provided *
 ********************************************/

static int picocrypto_encrypt( struct aead_request *req )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW encryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This encryption has AES and HMAC */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_ENC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_AUTH_ON | CTX_FLAG_ENC_ON );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_ENC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first encryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}

		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( req, pctx, BDR_NUM_FOR_ENC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW encryption */
		result = swcrypto_do_it( req, SW_CRYPTO_AUTH, SW_DO_ENC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_encrypt_non( struct aead_request *req )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW encryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This encryption has AES and non authentication*/
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_ENC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_NON_AUTH | CTX_FLAG_ENC_ON );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_ENC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first encryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( req, pctx, BDR_NUM_FOR_ENC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW encryption */
		result = swcrypto_do_it( req, SW_CRYPTO_NON, SW_DO_ENC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_encrypt_null( struct aead_request *req )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW encryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This encryption has null encryption and HMAC */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_ENC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_AUTH_ON | CTX_FLAG_NULL_ENC );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_ENC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first encryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( req, pctx, BDR_NUM_FOR_ENC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW encryption */
		result = swcrypto_do_it( req, SW_NULL_AUTH, SW_DO_ENC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_encrypt_null_non( struct aead_request *req )
{
	struct crypto_aead *authenc = crypto_aead_reqtfm( req );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW encryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This encryption has null encryption and non authentication */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_ENC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_NON_AUTH | CTX_FLAG_NULL_ENC );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_ENC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first encryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( req, pctx, BDR_NUM_FOR_ENC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW encryption */
		result = swcrypto_do_it( req, SW_NULL_NON, SW_DO_ENC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


/*************************************
 * Encryption with IV data generator *
 *************************************/

static int picocrypto_givencrypt( struct aead_givcrypt_request *req )
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm( areq );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	/* generate IV data */
	/* TODO: IV data needs to be suitably random, is this good enough ? */
	get_random_bytes( areq->iv, crypto_aead_ivsize( authenc ) );

	/* 
	 * req->giv[0..15] contains the IV data that goes into the ESP packet
	 * Make the areq->[0..15] for encrypt routine be the same
	 * as req->giv[0..15] for the ESP packet
	 */ 
	memcpy( req->giv, areq->iv, crypto_aead_ivsize( authenc ) );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW encryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This encryption has AES and HMAC */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_ENC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_AUTH_ON | CTX_FLAG_ENC_ON );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_ENC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first encryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( areq, pctx, BDR_NUM_FOR_ENC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW encryption */
		result = swcrypto_do_it( areq, SW_CRYPTO_AUTH, SW_DO_ENC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_givencrypt_non( struct aead_givcrypt_request *req )
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm( areq );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	/* generate IV data */
	/* TODO: IV data needs to be suitably random, is this good enough ? */
	get_random_bytes( areq->iv, crypto_aead_ivsize( authenc ) );

	/* 
	 * req->giv[0..15] contains the IV data that goes into the ESP packet
	 * Make the areq->[0..15] for encrypt routine be the same
	 * as req->giv[0..15] for the ESP packet
	 */ 
	memcpy( req->giv, areq->iv, crypto_aead_ivsize( authenc ) );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW encryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This encryption has AES and non authentication */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_ENC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_NON_AUTH | CTX_FLAG_ENC_ON );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_ENC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first encryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( areq, pctx, BDR_NUM_FOR_ENC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW encryption */
		result = swcrypto_do_it( areq, SW_CRYPTO_NON, SW_DO_ENC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_givencrypt_null( struct aead_givcrypt_request *req )
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm( areq );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW encryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This encryption has non encryption and HMAC */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_ENC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_AUTH_ON | CTX_FLAG_NULL_ENC );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_ENC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first encryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( areq, pctx, BDR_NUM_FOR_ENC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW encryption */
		result = swcrypto_do_it( areq, SW_NULL_AUTH, SW_DO_ENC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


static int picocrypto_givencrypt_null_non( struct aead_givcrypt_request *req )
{
	struct aead_request *areq = &req->areq;
	struct crypto_aead *authenc = crypto_aead_reqtfm( areq );
	struct private_ctx_data *pctx = crypto_aead_ctx( authenc );

	int result;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Called: %s\n", __FUNCTION__ );

	if( pctx->context_addr == 0 )
	{
		/* We couldn't allocate a context for the picoArray to use,
		 * so fall back to SW encryption */
		current_state = DRIVER_ALL_SW;
	}
	else
	{
		if( pctx->flags_set == CTX_FLAGS_UNKNOWN ) {
			/* This encryption has null encryption and non authentication */
			write_sram_context_flags( pctx->driver_info, pctx->context_addr,
				CTX_FLAG_ENC_DIR | CTX_FLAG_ESN_OFF | CTX_FLAG_NON_AUTH | CTX_FLAG_NULL_ENC );
	
			/* set the direction in our context data */
			pctx->enc_dec_dir = CTX_FLAG_ENC_DIR;
	
			/* set SEQH to zero, ESN not supported */
			write_sram_context_seqh( pctx->driver_info, pctx->context_addr, 0 );
	
			/* set the context crypto flags upon the first encryption */
			pctx->flags_set = CTX_FLAGS_SET;
		}
	
		current_state = atomic_read( &pctx->driver_info->driver_transit_state );
	}

	if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
		/* Insert the crypto data into a buffer descriptor */
		result = picocrypto_add_to_bdr( areq, pctx, BDR_NUM_FOR_ENC );

#ifdef ENABLE_BDR_PROCESSING_AFTER_ENC_DEC
		/* process all the BDRs for any completed PDUs */
		process_bdrs( pctx->driver_info, 0 );
#endif	

	} else {
		/* Give to SW encryption */
		result = swcrypto_do_it( areq, SW_NULL_NON, SW_DO_ENC );
	}

	DRIVER_DEBUG( KERN_INFO "Exited: %s\n", __FUNCTION__ );

	return ( result );
}


/*************************
 * Async event test code *
 *************************/

/* allow container of to be used */
struct work_item {
	struct delayed_work dwork;
	struct private_driver_data *pdata;
};	


/* GLOBALS */
static struct workqueue_struct *atest_work_queue = NULL;
/* create a work item */
struct work_item *atest_work_item = NULL;

void atest_work_fn( struct work_struct *pwork )
{
	u8 current_state;
	
	/* get back the delayed work item */
	struct delayed_work *pdwork = 
		container_of( pwork, struct delayed_work, work );

	struct work_item *pitem =
		container_of( pdwork, struct work_item, dwork );

	/* process all the BDRs for any completed PDUs */
	current_state = atomic_read( &pitem->pdata->driver_transit_state );

	if ( current_state != DRIVER_ALL_SW ) {
		process_bdrs( pitem->pdata, 0 );
	}

	/* add it back again, reuse the existing malloced structure */
	/* initialise the delayed work item */
	INIT_DELAYED_WORK( pdwork, &atest_work_fn );

	/* add work item to the work queue after a delay */
#if 1
	queue_delayed_work( atest_work_queue, pdwork, HZ/200 );	/* 5ms */
#else
	queue_delayed_work( atest_work_queue, pdwork, HZ/10 );	/* 100ms */
#endif
}


static void start_async_test( struct private_driver_data *pdata )
{
	/* create a work queue for simulating the async event */
	atest_work_queue = create_singlethread_workqueue( "async_test" );
	if( atest_work_queue == NULL ) {
		DRIVER_ERROR( KERN_ERR "Failed to create work queue\n" );
		return;
	}

	/* allocate memory for the work item */
	atest_work_item = kmalloc( sizeof( struct work_item ), GFP_KERNEL );
	if( atest_work_item == NULL ) {
		DRIVER_ERROR( KERN_ERR "Failed to create work item\n" );
		return;
	}

	/* make a note of the private driver info for the work function to use */
	atest_work_item->pdata = pdata;

	/* initialise the dealyed work item */
	INIT_DELAYED_WORK( &atest_work_item->dwork, &atest_work_fn );

	/* add work item to the work queue after a delay */
	queue_delayed_work( atest_work_queue, &atest_work_item->dwork, 5*HZ );
}


static void stop_async_test( struct private_driver_data *pdata )
{
	if( atest_work_queue != NULL ) {
		/* empty the work queue */
		flush_workqueue( atest_work_queue );

		/* finished with the work queue so delete it */
		destroy_workqueue( atest_work_queue );
	}

	if( atest_work_item != NULL ) {
		/* free the work item */
		kfree( atest_work_item );
		atest_work_item = NULL;
	}
}


/*************************
 * sysfs access function *
 *************************/
/*
 * Global variable for sysfs to allow the private
 * data structures to be accessed.
 */

struct private_driver_data *pglob_sysfs_private = NULL;

static ssize_t picocrypto_show_zero( struct device_driver *drv, char *buf )
{
        return scnprintf( buf, PAGE_SIZE, "%d\n", 0 );
}


static ssize_t picocrypto_clear_aead_reqs( struct device_driver *drv, const char *buf, size_t count )
{
	DRIVER_DEBUG( KERN_INFO "Trying to clean up AEAD Reqs in the BDRs...\n" );

	if ( pglob_sysfs_private != NULL ) {
		/* process any remaining AEAD reqs in the BDRs to allow the driver to be removed */
		u8 current_state = atomic_read( &pglob_sysfs_private->driver_transit_state );

		if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
			process_bdrs( pglob_sysfs_private, 1 );
		}
	}

        return count;
}


static ssize_t picocrypto_dump_sram( struct device_driver *drv, const char *buf, size_t count )
{
	DRIVER_INFO( KERN_INFO "Trying to dump the SRAM...\n" );

	if ( pglob_sysfs_private != NULL ) {
		/* dump the SRAM contents */
		dump_sram_structures( pglob_sysfs_private );
	}

        return count;
}

/* TODO: needs to return the result state back to user-land */
static ssize_t picocrypto_open_pico( struct device_driver *drv, const char *buf, size_t count )
{
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Trying to open the picoArray...\n" );

	if ( pglob_sysfs_private != NULL ) {

		if ( ! read_pico_active_state( pglob_sysfs_private ) ) {
			/* failed to OPEN the picoArray */

			/* fallback to using SW */
			atomic_set( &pglob_sysfs_private->driver_transit_state, DRIVER_ALL_SW );

			DRIVER_DEBUG( KERN_INFO "Set to SW mode\n" );
			
		} else {

			u8 picoActive = 1;
			
			current_state = atomic_read( &pglob_sysfs_private->driver_transit_state );
			
			if (current_state != DRIVER_ALL_HW)
			{
				picoActive = init_sram_structures( pglob_sysfs_private );
			}
			
			/* try opening the picoArray */
			if ( ( ! picoActive)  || ( request_sram_pico_open( pglob_sysfs_private ) == 1 ) ) {
				/* failed to OPEN the picoArray */
	
				/* fallback to using SW */
				atomic_set( &pglob_sysfs_private->driver_transit_state, DRIVER_ALL_SW );
	
				DRIVER_DEBUG( KERN_INFO "Set to SW mode\n" );
	
			} else {
				/* read the current transit state */
				current_state = atomic_read( &pglob_sysfs_private->driver_transit_state );
	
				switch ( current_state ) {
				case DRIVER_ALL_HW:
					/* Already in HW enc/dec therefore do not change state */
					break;
	
				case DRIVER_SWITCH_TO_HW:
					/* Error condition, switch to HW had previously failed ? */
					/* fall-through */
	
				case DRIVER_ALL_SW:
					/* Expected state */
					/* fall-through */
	
				case DRIVER_SWITCH_TO_SW:
					/* Error condition, switch to SW had previously failed ? */
	
					/*
					* change the transit state to redirect input to HW enc/dec
					* to allow SW to finish processing all remaining jobs.
					*/
					atomic_set( &pglob_sysfs_private->driver_transit_state, DRIVER_SWITCH_TO_HW );
	
					break;
				}
	
				/* No need to wait for SW to finish processing all pending jobs because it is synchronous */
	
				/*
				* No SW jobs pending so it is safe to 
				* change the transit state to redirect output of HW enc/dec.
				* Note. still possible for the picoArray to fail to enter the OPENED state.
				*/
				atomic_set( &pglob_sysfs_private->driver_transit_state, DRIVER_ALL_HW );
	
				DRIVER_DEBUG( KERN_INFO "Set to HW mode\n" );
			}
		}
	}

        return count;
}


/* TODO: needs to return the result state back to user-land */
/* TODO: WARNING: THERE IS NO LOCKING BETWEEN CHECKING ACTIVE BDs AND RE-SETTING THE BDRS */
/* SPIN-LOCKS ARE NOT POSSIBLE DUE TO SLEEPING WITHIN THE CLOSE_REQ FUNCTION */ 
/* TODO: Needs locking against other sysfs close and opens */ 
static ssize_t picocrypto_close_pico( struct device_driver *drv, const char *buf, size_t count )
{
	u8 error = 0;
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Trying to close the picoArray...\n" );

	if ( pglob_sysfs_private != NULL ) {

		/* read the current transit state */
		current_state = atomic_read( &pglob_sysfs_private->driver_transit_state );

		switch ( current_state ) {
		case DRIVER_ALL_SW:
			/* Already in SW enc/dec therefore do not change state */
			break;

		case DRIVER_SWITCH_TO_HW:
			/* Error condition, switch to HW had previously failed ? */
			/* fall-through */

		case DRIVER_ALL_HW:
			/* Expected state */
			/* fall-through */

		case DRIVER_SWITCH_TO_SW:
			/* Error condition, switch to SW had previously failed ? */

			/*
			 * change the transit state to redirect input to SW enc/dec
			 * to allow hw to finish processing all remaining active BDs.
			 */
			atomic_set( &pglob_sysfs_private->driver_transit_state, DRIVER_SWITCH_TO_SW );

			/* check for any active BDs before closing */ 
			if ( check_active_bds( pglob_sysfs_private ) == 1 ) {
	
				/* a BD is active so wait until it has been processed */
				msleep( BD_PROCESS_TIMEOUT );
	
				if ( check_active_bds( pglob_sysfs_private ) == 1 ) {
	
					/* there is still at least 1 active BD so cannot close the picoArray */
					error = 1;
				}
			}
			
			if ( error == 0 ) {
				/*
				* No active BDs so it is safe to 
				* change the transit state to redirect output of SW enc/dec.
				* Note. still possible for the picoArray to fail to enter the CLOSED state.
				*/
				atomic_set( &pglob_sysfs_private->driver_transit_state, DRIVER_ALL_SW );
	
				/* try closing the picoArray */
				if ( request_sram_pico_close( pglob_sysfs_private ) == 0 ) {
					/* successfully ACTIONED a change to the CLOSED state */
					/* Ignore the no change of state */
	
					/*
					* CAUTION: ALL BDRs MUST NOT BE IN USE BEFORE
					* RE-INITIALISING THE BDR's CURRENT BD POINTER
					* TO THE START OF THE RING TO MATCH THE
					* PICOARRAYS POINTERS UPON ENTERING THE CLOSED STATE.
					*/ 
					init_sram_structures( pglob_sysfs_private );
				}
			} else {
				DRIVER_ERROR( KERN_ERR "ERROR: Cannot CLOSE picoArray due to at least 1 active BD\n" );
			}
			break;
		}

		DRIVER_DEBUG( KERN_INFO "Set to SW mode\n" );
	}

        return count;
}


static ssize_t picocrypto_pico_status( struct device_driver *drv, char *buf )
{
	u32 cnf_state = 0;

	DRIVER_DEBUG( KERN_INFO "Trying to obtain the picoArray status...\n" );

	if ( pglob_sysfs_private != NULL ) {

		/* read picoArray confirmed status */
		cnf_state = read_sram_confirmed_state( pglob_sysfs_private );
	}

        return scnprintf( buf, PAGE_SIZE, "0x%08X\n", cnf_state );
}


static ssize_t picocrypto_show_mode( struct device_driver *drv, char *buf )
{
	u8 *mode = "Unknown";
	u8 current_state;

	DRIVER_DEBUG( KERN_INFO "Trying to obtain the driver's mode...\n" );

	if ( pglob_sysfs_private != NULL ) {

		/* get the current state of the driver */
		current_state = atomic_read( &pglob_sysfs_private->driver_transit_state );

		switch ( current_state ) {
		case DRIVER_ALL_HW:
			mode = "HW mode";
			break;

		case DRIVER_SWITCH_TO_HW:
			mode = "HW mode (switching)";
			break;

		case DRIVER_ALL_SW:
			mode = "SW mode";
			break;

		case DRIVER_SWITCH_TO_SW:
			mode = "SW mode (switching)";
			break;
		}
	}

        return scnprintf( buf, PAGE_SIZE, "%s\n", mode );
}


static ssize_t picocrypto_show_stats( struct device_driver *drv, char *buf )
{
	u32 max_irq_off = 0;
	u32 test1 = 0;
	u32 test2 = 0;
	u32 test3 = 0;
	u32 test4 = 0;
	u32 number_of_pdu_drops = 0;
	u32 number_of_bd_1_pre_drops = 0;
	u32 number_of_bd_2_pre_drops = 0;
	u32 number_of_bd_1_drops = 0;
	u32 number_of_bd_2_drops = 0;
	u32 number_of_dec_size_drops = 0;
	u32 number_of_enc_size_drops = 0;
	u32 largest_dec_dropped = 0;
	u32 largest_enc_dropped = 0;
	u32 number_of_pdus = 0;
	const char* culprit = "None";

	if ( pglob_sysfs_private != NULL ) {

		/* get the current state of the driver */
		max_irq_off = pglob_sysfs_private->max_irq_off_time;
		culprit = pglob_sysfs_private->culprit;
		test1 = pglob_sysfs_private->max_test_time_1;
		test2 = pglob_sysfs_private->max_test_time_2;
		test3 = pglob_sysfs_private->max_test_time_3;
		test4 = pglob_sysfs_private->max_test_time_4;
		number_of_pdu_drops = atomic_read( &pglob_sysfs_private->number_of_pdu_drops );
		number_of_bd_1_pre_drops = atomic_read( &pglob_sysfs_private->number_of_bd_1_pre_drops );
		number_of_bd_2_pre_drops = atomic_read( &pglob_sysfs_private->number_of_bd_2_pre_drops );
		number_of_bd_1_drops = atomic_read( &pglob_sysfs_private->number_of_bd_1_drops );
		number_of_bd_2_drops = atomic_read( &pglob_sysfs_private->number_of_bd_2_drops );
		number_of_dec_size_drops = atomic_read( &pglob_sysfs_private->number_of_dec_size_drops );
		number_of_enc_size_drops = atomic_read( &pglob_sysfs_private->number_of_enc_size_drops );
		largest_dec_dropped = atomic_read( &pglob_sysfs_private->largest_dec_dropped );
		largest_enc_dropped = atomic_read( &pglob_sysfs_private->largest_enc_dropped );
		number_of_pdus = atomic_read( &pglob_sysfs_private->number_of_pdus );
		
	}

	return scnprintf( buf, PAGE_SIZE, "max_irq_off=%d in %s\n"
	                                  "total_pdus=%d pdu_drops=%d \n"
	                                  "dec_size_drops=%d enc_size_drops=%d largest_dec_drop=%d largest_enc_drop=%d\n"
	                                  "bd_1_pre_drops=%d bd_2_pre_drops=%d bd_1_drops=%d bd_2_drops=%d\n"
	                                  "test1=%d test2=%d test3=%d test4=%d\n",
	                                   max_irq_off, culprit,
	                                   number_of_pdus, number_of_pdu_drops,
	                                   number_of_dec_size_drops, number_of_enc_size_drops, largest_dec_dropped, largest_enc_dropped,
	                                   number_of_bd_1_pre_drops, number_of_bd_2_pre_drops, number_of_bd_1_drops, number_of_bd_2_drops,
	                                   test1, test2, test3, test4
	                 );
}


/* sysfs global structures */
/* TODO: Check S_IRUGO | S_IWUSR settings */
DRIVER_ATTR( clear_aead_reqs, S_IRUGO | S_IWUSR, picocrypto_show_zero,
	    picocrypto_clear_aead_reqs );

DRIVER_ATTR( dump_sram, S_IRUGO | S_IWUSR, picocrypto_show_zero,
	    picocrypto_dump_sram );

DRIVER_ATTR( open_pico, S_IRUGO | S_IWUSR, picocrypto_show_zero,
	    picocrypto_open_pico );

DRIVER_ATTR( close_pico, S_IRUGO | S_IWUSR, picocrypto_show_zero,
	    picocrypto_close_pico );

DRIVER_ATTR( pico_status, S_IRUGO | S_IWUSR, picocrypto_pico_status,
	    NULL );

DRIVER_ATTR( show_mode, S_IRUGO | S_IWUSR, picocrypto_show_mode,
	    NULL );

DRIVER_ATTR( show_stats, S_IRUGO | S_IWUSR, picocrypto_show_stats,
	    NULL );

/*************************************************
 * Module initialisation and deregistration code *
 *************************************************/

int picocrypto_probe(struct platform_device *pdev)
{
	struct private_driver_data *pdata;

	DRIVER_DEBUG( KERN_INFO "Probing PicoArray...\n" );

	/* allocate some zeored out memory for private data, sleep is possible */
	pdata = kmalloc( sizeof( struct private_driver_data ), GFP_KERNEL );
 
	if( pdata == NULL ) {
		DRIVER_ERROR( KERN_ERR "Failed to allocate memory for private data\n" );
		goto err_alloc;
	}

	/* Initialise the spin lock */
	spin_lock_init( &pdata->driver_lock );

	/* Initialise the mutex locks */
	mutex_init( &pdata->pico_state_mutex );

	pdata->completed_head = 0;
	pdata->completed_tail = 0;
	
	atomic_set( &pdata->number_of_pdus, 0);
	atomic_set( &pdata->number_of_pdu_drops, 0);
	atomic_set( &pdata->number_of_bd_1_pre_drops, 0);
	atomic_set( &pdata->number_of_bd_2_pre_drops, 0);
	atomic_set( &pdata->number_of_bd_1_drops, 0);
	atomic_set( &pdata->number_of_bd_2_drops, 0);
	atomic_set( &pdata->number_of_dec_size_drops, 0);
	atomic_set( &pdata->number_of_enc_size_drops, 0);
	atomic_set( &pdata->largest_dec_dropped, 0);
	atomic_set( &pdata->largest_enc_dropped, 0);

	pdata->max_irq_off_time = 0;
	pdata->max_test_time_1 = 0;
	pdata->max_test_time_2 = 0;
	pdata->max_test_time_3 = 0;
	pdata->max_test_time_4 = 0;
	pdata->culprit = "None";
	
	tasklet_init(&pdata->tasklet, picocrypto_tasklet_callback, (unsigned long)pdata);

	/*
	 * add a pointer to the private data in the platform device structure.
	 * This allows the private data to be retrieved later.
	 */
	platform_set_drvdata( pdev, pdata );

	/* PicoArray SRAM physical base address */
	pdata->phys_sram_base_address = SRAM_PHYS_ADDRESS;
	pdata->sram_size = SRAM_LENGTH;	/* PicoArray SRAM length */

	/* Register the SRAM memory with the Kernel to make sure this driver has 100% control of it */
	if ( request_mem_region( pdata->phys_sram_base_address, pdata->sram_size, DRVNAME ) == NULL ) {
		/* failed */
		DRIVER_ERROR( KERN_ERR "Failed to register the SRAM memory\n" );
		goto err_alloc2;
	}

	/* Map the SRAM physical addresses to virtual addresses */
	pdata->vert_sram_base_address = ioremap_nocache( pdata->phys_sram_base_address, pdata->sram_size );

	if ( pdata->vert_sram_base_address == NULL ) {
		/* failed */
		DRIVER_ERROR( KERN_ERR "Failed to ioremap the SRAM memory\n" );
		goto err_alloc3;
	}

	init_context_structure_block( pdata, SRAM_CTX_MAX_NUMBER );
	
	init_sram_structures( pdata );

	/* Register the driver/PicoArray algorithms */
	if ( register_driver_crypto( pdata ) != 0 ) {
		/* failed to register the algorithms */
		DRIVER_ERROR( KERN_ERR "Failed to register the algorithms with the Kernel\n" );
		goto err_alloc4;
	}

	/* allocate the Software based algorithm transforms using the Kernel generic crypto */
	if ( swcrypto_alloc_tfms() != 0 ) {
		/* failed to allocate the SW algorithms */
		/* Note the algorithms are unregistered by swcrypto_alloc_tfms() upon failure */
		DRIVER_ERROR( KERN_ERR "Failed to allocate the Software based algorithms with the Kernel\n" );
		goto err_alloc4;
	}

	/* Save the private data structure for use by the sysfs functions */
	pglob_sysfs_private = pdata;


	/* test code */
#if 1
	start_async_test( pdata );	
#endif

	return 0;

/* Error handling */

err_alloc4:
	/* unregister the algorithms */
	unregister_driver_crypto( pdata );

err_alloc3:
	/* unregister the SRAM memory */
	release_mem_region( pdata->phys_sram_base_address, pdata->sram_size );

err_alloc2:
	tasklet_kill(&pdata->tasklet);
		
	/* free the private data */
	platform_set_drvdata( pdev, NULL );
	kfree( pdata );

	/* clear the private data structure used by the sysfs functions */
	pglob_sysfs_private = NULL;

err_alloc:
	return -ENOMEM;
}


int picocrypto_remove( struct platform_device *pdev )
{
	struct private_driver_data *pdata;

	DRIVER_INFO( KERN_INFO "Removing driver...\n" );

	/* release the Software based algorithm transforms */
	swcrypto_release_tfms();

	/* get access to the private data */
	pdata = platform_get_drvdata( pdev );

	/* is the private data available ? */
	if ( pdata != NULL ) {

		u8 current_state;
		
		/* test code */
		stop_async_test( pdata );	

		tasklet_kill(&pdata->tasklet);
		
		/* try closing the picoArray to stop it processing any BDs */
		request_sram_pico_close( pdata );

		/* process any remaining AEAD reqs in the BDRs to allow the driver to be removed */
		current_state = atomic_read( &pdata->driver_transit_state );

		if ( ( current_state == DRIVER_ALL_HW ) || ( current_state == DRIVER_SWITCH_TO_HW ) ) {
			process_bdrs( pdata, 1 );
		}

		/* Unregister the algorithms from the Kernel */
		unregister_driver_crypto( pdata );

		/* umap the IOMEM space */
		iounmap( pdata->vert_sram_base_address );

		/* release the SRAM memory region */
		release_mem_region( pdata->phys_sram_base_address, pdata->sram_size );

		/* free the private data */
		kfree( pdata );

		/* clear the private data structure used by the sysfs functions */
		pglob_sysfs_private = NULL;
	}

	return 0;
}


struct platform_driver picocrypto_platform = {
	.probe = picocrypto_probe,
	.remove = picocrypto_remove,
	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		/* Must match name of platform device */
		.name = DRVNAME,
		.owner	= THIS_MODULE,
	},
};


static int __init picocrypto_init(void)
{
	int err;

	DRIVER_INFO( KERN_INFO "PicoCrypto initialising version %s...\n", DRVVERS );

	err = platform_driver_register( &picocrypto_platform );

	if( err ) {
		DRIVER_ERROR( KERN_ERR "Failed to register the driver for %s. Check kernel platform support.\n",
			picocrypto_platform.driver.name );
		goto error_init;
	}

	err = driver_create_file( &picocrypto_platform.driver, &driver_attr_clear_aead_reqs );

	if( err ) {
		DRIVER_ERROR( KERN_ERR "Failed to create sysfs file clear_aead_reqs for the driver for %s.\n",
			picocrypto_platform.driver.name );
		goto error_aead_clear;
	}

	err = driver_create_file( &picocrypto_platform.driver, &driver_attr_dump_sram );

	if( err ) {
		DRIVER_ERROR( KERN_ERR "Failed to create sysfs file dump_sram for the driver for %s.\n",
			picocrypto_platform.driver.name );
		goto error_sysfs_dump;
	}

	err = driver_create_file( &picocrypto_platform.driver, &driver_attr_open_pico );

	if( err ) {
		DRIVER_ERROR( KERN_ERR "Failed to create sysfs file open_pico for the driver for %s.\n",
			picocrypto_platform.driver.name );
		goto error_sysfs_open_pico;
	}

	err = driver_create_file( &picocrypto_platform.driver, &driver_attr_close_pico );

	if( err ) {
		DRIVER_ERROR( KERN_ERR "Failed to create sysfs file close_pico for the driver for %s.\n",
			picocrypto_platform.driver.name );
		goto error_sysfs_close_pico;
	}

	err = driver_create_file( &picocrypto_platform.driver, &driver_attr_pico_status );

	if( err ) {
		DRIVER_ERROR( KERN_ERR "Failed to create sysfs file pico_status for the driver for %s.\n",
			picocrypto_platform.driver.name );
		goto error_sysfs_pico_status;
	}

	err = driver_create_file( &picocrypto_platform.driver, &driver_attr_show_mode );

	if( err ) {
		DRIVER_ERROR( KERN_ERR "Failed to create sysfs file show_mode for the driver for %s.\n",
			picocrypto_platform.driver.name );
		goto error_sysfs_show_mode;
	}

	err = driver_create_file( &picocrypto_platform.driver, &driver_attr_show_stats );

	if( err ) {
		DRIVER_ERROR( KERN_ERR "Failed to create sysfs file show_stats for the driver for %s.\n",
			picocrypto_platform.driver.name );
		goto error_sysfs_show_stats;
	}

	return ( err );

/* error handlers */

error_sysfs_show_stats:
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_show_stats );

error_sysfs_show_mode:
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_show_mode );

error_sysfs_pico_status:
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_pico_status );

error_sysfs_close_pico:
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_close_pico );

error_sysfs_open_pico:
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_open_pico );

error_sysfs_dump:
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_dump_sram );

error_aead_clear:
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_clear_aead_reqs );

error_init:
	platform_driver_unregister( &picocrypto_platform );

	return err;
}

static void __exit picocrypto_exit(void)
{
	DRIVER_INFO( KERN_INFO "PicoCrypto deregistering...\n" );

	driver_remove_file( &picocrypto_platform.driver, &driver_attr_show_stats );
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_show_mode );
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_pico_status );
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_close_pico );
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_open_pico );
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_dump_sram );
	driver_remove_file( &picocrypto_platform.driver, &driver_attr_clear_aead_reqs );

	platform_driver_unregister( &picocrypto_platform );
}

module_init(picocrypto_init);
module_exit(picocrypto_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dean Jenkins <djenkins@mvista.com>");
MODULE_DESCRIPTION("MontaVista IPsec PicoArray Crypto driver");
