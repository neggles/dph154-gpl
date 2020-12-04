/******************************************************************************
 *
 * This file contains the Software based cryptographic calls to the Kernel's Crypto functions.
 *
 * File: swcrypto.c
 * Author: Dean Jenkins <djenkins@mvista.com>
 *
 * 2009 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *****************************************************************************
 * Filename: 
 *****************************************************************************/

/* kernel related headers */
#include <linux/kernel.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>

/* Kernel crypto related headers */
#include <linux/crypto.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>

#include "picocrypto.h"
#include "swcrypto.h"

/******************
 * local #defines *
 ******************/

#undef DEBUG_SWCRYPTO
/* #define DEBUG_SWCRYPTO	1	*/

#ifdef DEBUG_SWCRYPTO
#define SWCRYPTO_DEBUG(fmt,args...)	printk( fmt ,##args )
#else
#define SWCRYPTO_DEBUG(fmt,args...)
#endif

#define SWCRYPTO_INFO(fmt,args...)	printk( fmt ,##args )
#define SWCRYPTO_ERROR(fmt,args...)	printk( fmt ,##args )


/***************
 * local types *
 ***************/

struct sw_tfm {
	struct crypto_aead *tfm;
	char *driver_name;
};


/******************
 * local globals  *
 ******************/

/* array to store the pointers of the SW transforms */
static struct sw_tfm swcrypto_tfms[ NUM_SW_TFMS ] = {
	/* SW_CRYPTO_AUTH */
	{
		.tfm = NULL,
		.driver_name = "authenc(hmac(sha1-generic),cbc(aes-generic))",
	},
	/* SW_CRYPTO_NON */
	{
		.tfm = NULL,
		.driver_name = "authenc(digest_null-generic,cbc(aes-generic))",
	},
	/* SW_NULL_AUTH */
	{
		.tfm = NULL,
		.driver_name = "authenc(hmac(sha1-generic),ecb-cipher_null)",
	},
	/* SW_NULL_NON */
	{
		.tfm = NULL,
		.driver_name = "authenc(digest_null-generic,ecb-cipher_null)",
	}
};


/*********************************
 * SW Crypto Interface functions *
 *********************************/

/*
 * Allocs SW algorithm transforms
 * Inputs:
 *	None
 * Outputs:
 *	allocates all the required SW based algorithm transforms
 * Return:
 *	0 for success else 1 for failure
 */
u8 swcrypto_alloc_tfms( void )
{
	u8 error = 0;
	u8 tfm_cnt;
	u8 tfm_err_cnt;

	for ( tfm_cnt = 0; tfm_cnt < NUM_SW_TFMS; tfm_cnt++ ) {

		swcrypto_tfms[ tfm_cnt ].tfm = crypto_alloc_aead( swcrypto_tfms[ tfm_cnt ].driver_name,
							0, 0 );

		if ( IS_ERR( swcrypto_tfms[ tfm_cnt ].tfm ) ) {
			SWCRYPTO_ERROR( KERN_ERR "ERROR: Failed to load transform for %s\n",
				swcrypto_tfms[ tfm_cnt ].driver_name );
			error = 1;
			break;
		}
	}

	if ( error == 1 ) {
		/* on error unallocate tfms */
		for ( tfm_err_cnt = 0; tfm_err_cnt < tfm_cnt; tfm_err_cnt++ ) {
			crypto_free_aead( swcrypto_tfms[ tfm_err_cnt ].tfm );
			swcrypto_tfms[ tfm_err_cnt ].tfm = NULL;
		}
	}

	return ( error );
}


/*
 * Releases allocated SW algorithm transforms
 * Inputs:
 *	None
 * Outputs:
 *	Releases all the allocated SW based algorithm transforms
 * Return:
 *	None
 */
void swcrypto_release_tfms( void )
{
	u8 tfm_cnt;

	for ( tfm_cnt = 0; tfm_cnt < NUM_SW_TFMS; tfm_cnt++ ) {
		if ( swcrypto_tfms[ tfm_cnt ].tfm != NULL ) {
			crypto_free_aead( swcrypto_tfms[ tfm_cnt ].tfm );
			swcrypto_tfms[ tfm_cnt ].tfm = NULL;
		}
	}
}


/*
 * Actions the Software based crypto
 * Inputs:
 *	req = pointer to the crypto request
 *	tfm_num = the number of algorithm transform in swcrypto_tfms[]
 *	encdec = flag for decryption or encryption
 * Outputs:
 *	Substitutes the picoArray tfm for the SW tfm
 *	Actions the SW encryption or decryption
 * Return:
 *	Error code
 */
int swcrypto_do_it( struct aead_request *req, u8 tfm_num, u8 encdec )
{
	int error = 0;
	int ret;

	/* Get the pointer to the appropriate SW algorithm transform */
	struct crypto_aead *sw_tfm = swcrypto_tfms[ tfm_num ].tfm;

	/* get the hw algorithm transform info */
	struct crypto_aead *hw_tfm = crypto_aead_reqtfm( req );
	/* get the context data for the hw request */
	struct private_ctx_data *pctx = crypto_aead_ctx( hw_tfm );

	struct aead_request *sw_req;

	SWCRYPTO_DEBUG( KERN_INFO "%s called\n", __FUNCTION__ );

	SWCRYPTO_DEBUG( KERN_INFO "hw: reqsize %d, sw: reqsize %d\n",
		crypto_aead_reqsize( hw_tfm ), crypto_aead_reqsize( sw_tfm ) );

	crypto_aead_clear_flags( sw_tfm, ~0 );

	/* set the key for SW algorithm transform */
	ret = crypto_aead_setkey( sw_tfm, &pctx->rawkey[0], pctx->rawkey_len );
	if( ret ) {
		SWCRYPTO_ERROR( KERN_ERR "%s: ERROR: failed to set the key\n", __FUNCTION__ );
		return -EBUSY;
	}

#ifdef DEBUG_SWCRYPTO
	dump_aead_req_enckey( pctx );
#endif

	/* set the authsize ( authentication field length ) for SW algorithm transform */
	ret = crypto_aead_setauthsize( sw_tfm, pctx->authsize );
	if( ret ) {
		SWCRYPTO_ERROR( KERN_ERR "%s: ERROR: failed to set the authsize of %d\n", __FUNCTION__, pctx->authsize );
		return -EBUSY;
	}

#ifdef DEBUG_SWCRYPTO
	dump_aead_req_authkey( pctx );
#endif

	/*****************/
	/* Use a new req */
	/*****************/

	/* allocate a request for the software algorithm */
	sw_req = aead_request_alloc( sw_tfm, GFP_ATOMIC );
	if (!sw_req) {
		SWCRYPTO_ERROR( KERN_ERR "ERROR: failed to allocate request\n" );
		return -EBUSY;
	}


	/* transfer HW request info into the SW request */

	aead_request_set_callback( sw_req, req->base.flags, req->base.complete,
				  req->base.data );
	aead_request_set_crypt( sw_req, req->src, req->dst, req->cryptlen, req->iv );
	aead_request_set_assoc( sw_req, req->assoc, req->assoclen );

#ifdef DEBUG_SWCRYPTO
	dump_aead_req_assoc( sw_req );
	dump_aead_req_iv( sw_req );
	dump_aead_req_src( sw_req );
#endif

	/* do it */
	if ( encdec == SW_DO_DEC ) {
		SWCRYPTO_DEBUG( KERN_INFO "Doing SW req decryption\n" );

		/* action decryption */
		error = crypto_aead_decrypt( sw_req );
	} else {
		SWCRYPTO_DEBUG( KERN_INFO "Doing SW req encryption\n" );

		/* action encryption */
		error = crypto_aead_encrypt( sw_req );
	}

#ifdef DEBUG_SWCRYPTO
	/* now print out the result */
	dump_aead_req_dst( sw_req );
#endif

	/* free the SW AEAD request as now finished with it */
	aead_request_free( sw_req );

	SWCRYPTO_DEBUG( KERN_INFO "OK so far...error: %d tfm_num: %d\n", error, tfm_num );

	return (error);
}



void dump_aead_req_assoc( struct aead_request *req )
{
	u8 *psrc_data;
	u32 seqment_len;
	u8 pos;

	/* get a pointer to the assoc data */
	psrc_data = sg_virt( req->assoc );

	/* get the length of this seqment */
	seqment_len = req->assoc->length;

	SWCRYPTO_DEBUG( KERN_INFO "DUMP assoc data: ptr = %p, len = %d\n", psrc_data, seqment_len );

	for ( pos = 0; pos < seqment_len; pos++ ) {
		SWCRYPTO_DEBUG( KERN_INFO "data[%d] = %02X\n", pos, psrc_data[ pos ] );
	}
} 


void dump_aead_req_iv( struct aead_request *req )
{
	u8 *psrc_data;
	u32 len;
	u8 pos;

	/* get a pointer to the IV data */
	psrc_data = req->iv;

	/* get the length */
	len = 16;

	SWCRYPTO_DEBUG( KERN_INFO "DUMP iv data: ptr = %p, len = %d\n", psrc_data, len );

	for ( pos = 0; pos < len; pos++ ) {
		SWCRYPTO_DEBUG( KERN_INFO "data[%d] = %02X\n", pos, psrc_data[ pos ] );
	}
}


void dump_aead_req_src( struct aead_request *req )
{
	u8 *psrc_data;
	u32 seqment_len;
	u8 pos;

	/* get a pointer to the src data */
	psrc_data = sg_virt( req->src );

	/* get the length of this seqment */
	seqment_len = req->src->length;

	SWCRYPTO_DEBUG( KERN_INFO "DUMP src: ptr = %p, len = %d\n", psrc_data, seqment_len );

	for ( pos = 0; pos < seqment_len; pos++ ) {
		SWCRYPTO_DEBUG( KERN_INFO "data[%d] = %02X\n", pos, psrc_data[ pos ] );
	}
}


void dump_aead_req_dst( struct aead_request *req )
{
	u8 *pdst_data;
	u32 seqment_len;
	u8 pos;

	/* get a pointer to the dst data */
	pdst_data = sg_virt( req->dst );

	/* get the length of this seqment */
	seqment_len = req->src->length;

	SWCRYPTO_DEBUG( KERN_INFO "DUMP dst: ptr = %p, len = %d\n", pdst_data, seqment_len );

	for ( pos = 0; pos < seqment_len; pos++ ) {
		SWCRYPTO_DEBUG( KERN_INFO "data[%d] = %02X\n", pos, pdst_data[ pos ] );
	}
}


void dump_aead_req_enckey( struct private_ctx_data *pctx )
{
	u32 len = pctx->enckey_len;
	u8 pos;

	SWCRYPTO_DEBUG( KERN_INFO "DUMP enckey data: len = %d\n", len );

	for ( pos = 0; pos < len; pos++ ) {
		SWCRYPTO_DEBUG( KERN_INFO "data[%d] = %02X\n", pos, pctx->enckey[ pos ] );
	}
}


void dump_aead_req_authkey( struct private_ctx_data *pctx )
{
	u32 len = pctx->authkey_len;
	u8 pos;

	SWCRYPTO_DEBUG( KERN_INFO "DUMP authkey data: len = %d\n", len );

	for ( pos = 0; pos < len; pos++ ) {
		SWCRYPTO_DEBUG( KERN_INFO "data[%d] = %02X\n", pos, pctx->authkey[ pos ] );
	}
}


