/*
 * This file contains the Software based Crypto #defines, types and prototypes.
 *
 * File: swcrypto.h
 * Author: Dean Jenkins <djenkins@mvista.com>
 *
 * 2009 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

/*******************
 * common #defines *
 *******************/

#define NUM_SW_TFMS	(4)	/* Number of SW transform algorithms */

/* SW transform algorithms */
#define SW_CRYPTO_AUTH	(0)
#define SW_CRYPTO_NON	(1)
#define SW_NULL_AUTH	(2)
#define SW_NULL_NON	(3)

#define SW_DO_ENC	(0)	/* do SW encryption */
#define SW_DO_DEC	(1)	/* do SW decryption */


/***********************
 * function prototypes *
 ***********************/

extern u8 swcrypto_alloc_tfms( void );
extern void swcrypto_release_tfms( void );

extern int swcrypto_do_it( struct aead_request *req, u8 tfm_num, u8 encdec );

extern void dump_aead_req_assoc( struct aead_request *req );
extern void dump_aead_req_iv( struct aead_request *req );
extern void dump_aead_req_src( struct aead_request *req );
extern void dump_aead_req_dst( struct aead_request *req );
extern void dump_aead_req_enckey( struct private_ctx_data *pctx );
extern void dump_aead_req_authkey( struct private_ctx_data *pctx );

