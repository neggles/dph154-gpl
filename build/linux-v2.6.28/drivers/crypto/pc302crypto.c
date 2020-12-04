#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <crypto/algapi.h>
#include <crypto/des.h>
#include <crypto/authenc.h>
#include <crypto/aead.h>
#include <crypto/internal/skcipher.h>
#include <asm/io.h>
#include <linux/rtnetlink.h> /* For rt attributes (struct rtattr) for encoding
                              * encryption and authentication keys in authenc
                              * algorithms. */

#include <mach/pc302/pc302.h>
#include <mach/pc302/spacc.h>
#include <mach/hardware.h>

#undef  PROFILE_PC302CRYPTO_USING_HIRES_TIMER

/* The threshold for the number of packets that should complete in the SPAcc
 * before the STAT_CNT interrupt is raised. Incresing this value will reduce
 * the number of interrupts raised to the CPU. */
#define STAT_IRQ_THRESHOLD_IPSEC   ( 24 )
#define STAT_IRQ_THRESHOLD_L2      ( 64 )

/* The threshold for the number of entries in the CMD FIFO available before
 * the CMD0_CNT interrupt is raised. Incresing this value will reduce the
 * number of interrupts raised to the CPU. */
#define CMD0_IRQ_THRESHOLD   ( 1 )

/* The timeout period (in jiffies) for a PDU. When the the number of PDUs in
 * flight is greater than the STAT_IRQ_THRESHOLD or 0 the timer is disabled.
 * When there are packets in flight but lower than the threshold, we enable
 * the timer and at expiry, attempt to remove any processed packets from the
 * queue and if there are still packets left, schedule the timer again. */
#define PACKET_TIMEOUT      ( 2 )

/*! The priority to register each algorithm with. */
#define PC302_CRYPTO_ALG_PRIORITY       ( 10000 )

/*! Max length for an AES key. */
#define PC302_CRYPTO_AES_MAX_KEY_LEN    ( 32 )
/*! Length of an AES IV in bytes. */
#define PC302_CRYPTO_AES_IV_LEN         ( 16 )
/*! Length of the AES salt mask. */
#define PC302_CRYPTO_AES_SALT_MASK_LEN  ( 16 )

/*! Length of a DES IV. */
#define PC302_CRYPTO_DES_IV_LEN         ( 8 )
/*! Length of a DES key. */
#define PC302_CRYPTO_DES_KEY_LEN        ( 8 )

/*! Length of a kasumi f8 encryption key in bytes. */
#define PC302_CRYPTO_KASUMI_F8_KEY_LEN  ( 16 )
/*! Length of a kasumi f8 IV. */
#define PC302_CRYPTO_KASUMI_F8_IV_LEN   ( 8 )
/*! Length of the reserved field in the kasumi f8 context. */
#define PC302_CRYPTO_KASUMI_F8_RSVD_LEN ( 8 )

/*! Cipher key page size in the IPSec crypto engine. */
#define PC302_CRYPTO_IPSEC_CIPHER_PG_SZ ( 64 )
/*! Hash key page size in the IPSec crypto engine. */
#define PC302_CRYPTO_IPSEC_HASH_PG_SZ   ( 64 )
/*! Maximum number of key contexts in the IPSec engine. */
#define PC302_CRYPTO_IPSEC_MAX_CTXS     ( 32 )
/*! The depth of the request FIFO in the IPSec engine. */
#define PC302_CRYPTO_IPSEC_FIFO_SZ      ( 32 )

/*! Cipher key page size in the SRTP crypto engine. */
#define PC302_CRYPTO_SRTP_CIPHER_PG_SZ  ( 64 )
/*! Hash key page size in the SRTP crypto engine. */
#define PC302_CRYPTO_SRTP_HASH_PG_SZ    ( 32 )
/*! Maximum number of key contexts in the SRTP engine. */
#define PC302_CRYPTO_SRTP_MAX_CTXS      ( 128 )
/*! The depth of the request FIFO in the SRTP engine. */
#define PC302_CRYPTO_SRTP_FIFO_SZ       ( 128 )

/*! Cipher key page size in the L2 crypto engine. */
#define PC302_CRYPTO_L2_CIPHER_PG_SZ    ( 64 )
/*! Hash key page size in the L2 crypto engine. */
#define PC302_CRYPTO_L2_HASH_PG_SZ      ( 64 )
/*! Maximum number of key contexts in the L2 crypto engine. */
#define PC302_CRYPTO_L2_MAX_CTXS        ( 128 )
/*! The depth of the request FIFO in the SRTP engine. */
#define PC302_CRYPTO_L2_FIFO_SZ         ( 128 )

/*! The maximum hash key page size of all engines. */
#define PC302_CRYPTO_MAX_HASH_PG_SZ     ( PC302_CRYPTO_IPSEC_HASH_PG_SZ )

/*! The maximum length of a DDT. If we exceed this then we probably have an
 *  architectural problem if the fragmentation is this bad. If this isn't true
 *  and we have some really clever scheme then make this bigger. */
#define MAX_DDT_LEN                     ( 64 )

/*! Debug enumeration. High integer values are of lower priority. */
enum debug_levels
{
    DBG_ERROR = 0,  /*!< Critical errors. */
    DBG_WARN,       /*!< Warning messages. */
    DBG_NOTICE,     /*!< Notices of abnormal conditions. */
    DBG_TRACE,      /*!< Trace level messages for debug. */
    DBG_IO,         /*!< All I/O requests. */
};

/*! The current debug level of the driver. */
#define DEBUG_LVL   ( DBG_NOTICE )

/*!
 * Print a debug message to the console. If _lvl is less than or equal to the
 * current debug level then the message is printed.
 *
 * @param _lvl The level of the message.
 * @param _fmt The message to print.
 */
#define PRINTD( _lvl, _fmt, ... ) \
    ({ \
        if ( _lvl <= DEBUG_LVL ) \
            printk( "pc302crypto.c:%u: " _fmt "\n", __LINE__, \
                    ##__VA_ARGS__ ); \
    })

struct pc302crypt_generic_ctx;

enum pc302crypt_direction
{
    ENCRYPT,
    DECRYPT,
};

/*!
 * \brief DDT format. This must match the hardware DDT format exactly.
 */
struct pc302crypt_ddt
{
    u32                         p;      /*!< The address of the buffer to
                                         *   transfer. */

    u32                         len;    /*!< The length of the buffer. */
};

/* DDT's must be aligned to a 64 byte boundary. */
#define __ddt_align             __attribute__((aligned(64)))

struct pc302crypt_engine_ctx
{
    struct pc302crypt_ddt       __ddt_align src_ddt[MAX_DDT_LEN];
    struct pc302crypt_ddt       __ddt_align dst_ddt[MAX_DDT_LEN];
};

struct pc302crypt_ddt_info
{
    struct pc302crypt_ddt_info *next;
};

/*!
 * \brief Asynchronous crypto request structure.
 *
 * This structure defines a request that is either queued for processing or
 * being processed.
 */
struct pc302crypt_req
{
    /*! The asynchronous crypto request from the crypto API. */
    struct crypto_async_request *req;

    /*! The result of the operation. Only valid once the ciphering has
     * completed. */
    int                         result;

    /*! The position in the request queue. */
    struct list_head            list;

    /*! The processing direction. */
    enum pc302crypt_direction   dir;

    /*! The context slot in the engine that is associated with the request. */
    unsigned                    ctx_id;

    /*! The destination for a generated IV for geniv requests. */
    u8                          *giv;

    /*! The length of the generated IV. */
    size_t                      giv_len;

    /*! The sequence number for giv algos. */
    u64                         seq;

    /*! The physical address of the DMA mapped giv. */
    dma_addr_t                  giv_pa;

    /*! The length of the source and destination DDTs. Make both the same
     * length for simplicity. */
    unsigned                    ddt_len;

    dma_addr_t                  src_addr;
    dma_addr_t                  dst_addr;
};

/*!
 * \brief Crypto queue. Allows queueing of pc302crypto_req requests.
 *
 * This is used rather than the crypto API crypto queue as we want to add
 * pc302crypto_req structures and to be able to force addition even if the
 * request can't backlog. This is used when a request that can't backlog has
 * actually completed but we want to defer the completion function.
 */
struct pc302crypt_queue
{
    struct list_head            list;       /*!< list of requests. */

    struct list_head            *backlog;   /*!< The position in the list
                                             *   where the backlog starts. */

    unsigned                    qlen;       /*!< The number of entries in the
                                             *   queue. */

    unsigned                    max_qlen;   /*!< The maximum number of entries
                                             *   that the queue may hold. */
};

/*!
 * \brief Definition of a PC302 crypto engine.
 */
struct pc302crypt_engine
{
    /*! The engine name. */
    const char                  *name;

    /*! The device entry. */
    struct device               *dev;

    /*! Register base address. */
    void __iomem                *regs;

    /*! Cipher key context base address. */
    void __iomem                *cipher_ctx_base;

    /*! Hash key base address. */
    void __iomem                *hash_key_base;

    /*! The DDT buffer. */
    struct pc302crypt_engine_ctx    *ddt_buf;
    
    struct pc302crypt_ddt_info    *ddt_info;
    struct pc302crypt_ddt_info    *free_ddt_info;
    void                          *zeroed_page;
    dma_addr_t                    zeroed_src_addr;
    struct scatterlist            zeroed_list;

    /*! The physical address of the DDT buffer. */
    dma_addr_t                  ddt_buf_phys;

    /*! Hardware lock for sequencing commands. */
    spinlock_t                  hw_lock;

    /*! Maximum number of contexts that the engine can hold. */
    unsigned                    max_ctxs;

    /*! The algorithms that can be registered for the engine. */
    struct pc302crypt_alg       *algs;

    /* The number of algorithms that may be registered. */
    unsigned                    num_algs;

    /*! Algorithms registered for this engine. */
    struct list_head            registered_algs;

    /*! The cipher key context page size in bytes. */
    size_t                      cipher_pg_sz;

    /*! The hash key context page size in bytes. */
    size_t                      hash_pg_sz;

    /*! Queue for requests. */
    struct pc302crypt_queue     pending;

    /*! Queue for completed requests. */
    struct pc302crypt_queue     completed;

    /*! Tasklet for completing asynchronous requests by callbacks. */
    struct tasklet_struct       complete;

    /*! The size of the request fifo. */
    unsigned                    fifo_sz;

    /*! The number of requests 'in flight'. */
    unsigned                    in_flight;

    /*! The number of requests allowed before an interrupt. */
    unsigned                    stat_irq_threshold;

    /*! The timer for packet timeouts. */
    struct timer_list           packet_timeout;
};

/*! Type mask for a crypto algorithm. */
#define PC302_CRYPTO_TYPE_MASK          ( 0x0F00 )
/*! Type mask for block algorithms. */
#define PC302_CRYPTO_BLOCK_MASK         ( 0x000F )
/*! Type mask for hash algorithms. */
#define PC302_CRYPTO_HASH_MASK          ( 0x00F0 )
/*! The algorithm is a block cipher. */
#define PC302_CRYPTO_BLOCK              ( 0x0100 )
/*! The algorithm is a hash. */
#define PC302_CRYPTO_HASH               ( 0x0200 )
/*! The algorithm is a combined mode (authenc). */
#define PC302_CRYPTO_COMBINED           ( 0x0300 )

/*! Algorithm type mask. */
#define PC302_CRYPTO_ALG_MASK           ( 0x00FF )
/*! AES block cipher. */
#define PC302_CRYPTO_ALG_AES            ( 0x0001 | PC302_CRYPTO_BLOCK )
/*! DES block cipher. */
#define PC302_CRYPTO_ALG_DES            ( 0x0002 | PC302_CRYPTO_BLOCK )
/*! Triple DES block cipher. */
#define PC302_CRYPTO_ALG_3DES           ( 0x0003 | PC302_CRYPTO_BLOCK )
/*! Kasumi F8 block cipher. */
#define PC302_CRYPTO_ALG_KASUMI         ( 0x0004 | PC302_CRYPTO_BLOCK )
/*! SHA1 hash. */
#define PC302_CRYPTO_ALG_SHA1           ( 0x0010 | PC302_CRYPTO_HASH )
/*! MD5 hash. */
#define PC302_CRYPTO_ALG_MD5            ( 0x0020 | PC302_CRYPTO_HASH )
/*! SHA256 hash. */
#define PC302_CRYPTO_ALG_SHA256         ( 0x0030 | PC302_CRYPTO_HASH )

/*! Block cipher mode of operation mask. */
#define PC302_CRYPTO_ABLK_MODE_MASK     ( 0xF000 )
/*! CBC mode. */
#define PC302_CRYPTO_ABLK_MODE_CBC      ( 0x1000 )
/*! ECB mode. */
#define PC302_CRYPTO_ABLK_MODE_ECB      ( 0x2000 )
/*! F8 mode. */
#define PC302_CRYPTO_ABLK_MODE_F8       ( 0x3000 )

/*! Combined mode cbc(aes) and hmac(sha1). */
#define PC302_CRYPTO_ALG_AUTHENC_AES_CBC_HMAC_SHA1 \
    ( PC302_CRYPTO_COMBINED | \
      PC302_CRYPTO_ALG_AES | \
      PC302_CRYPTO_ABLK_MODE_CBC | \
      PC302_CRYPTO_ALG_SHA1 )

/*! Combined mode cbc(aes) and hmac(sha256). */
#define PC302_CRYPTO_ALG_AUTHENC_AES_CBC_HMAC_SHA256 \
    ( PC302_CRYPTO_COMBINED | \
      PC302_CRYPTO_ALG_AES | \
      PC302_CRYPTO_ABLK_MODE_CBC | \
      PC302_CRYPTO_ALG_SHA256 )

/*! Combined mode cbc(aes) and hmac(md5). */
#define PC302_CRYPTO_ALG_AUTHENC_AES_CBC_HMAC_MD5 \
    ( PC302_CRYPTO_COMBINED | \
      PC302_CRYPTO_ALG_AES | \
      PC302_CRYPTO_ABLK_MODE_CBC | \
      PC302_CRYPTO_ALG_MD5 )

/*! Combined mode cbc(des3_ede) and hmac(sha1). */
#define PC302_CRYPTO_ALG_AUTHENC_3DES_CBC_HMAC_SHA1 \
    ( PC302_CRYPTO_COMBINED | \
      PC302_CRYPTO_ALG_3DES | \
      PC302_CRYPTO_ABLK_MODE_CBC | \
      PC302_CRYPTO_ALG_SHA1 )

/*! Combined mode cbc(aes) and hmac(sha256). */
#define PC302_CRYPTO_ALG_AUTHENC_3DES_CBC_HMAC_SHA256 \
    ( PC302_CRYPTO_COMBINED | \
      PC302_CRYPTO_ALG_3DES | \
      PC302_CRYPTO_ABLK_MODE_CBC | \
      PC302_CRYPTO_ALG_SHA256 )

/*! Combined mode cbc(aes) and hmac(md5). */
#define PC302_CRYPTO_ALG_AUTHENC_3DES_CBC_HMAC_MD5 \
    ( PC302_CRYPTO_COMBINED | \
      PC302_CRYPTO_ALG_3DES | \
      PC302_CRYPTO_ABLK_MODE_CBC | \
      PC302_CRYPTO_ALG_MD5 )

/*!
 * \brief PC302 definition of a crypto algorithm.
 */
struct pc302crypt_alg
{
    /*! The cipher type. */
    unsigned long               type;

    /*! The algorithm as it is registered with the crypto API. */
    struct crypto_alg           alg;

    /*! The engine that processes the algorithm. */
    struct pc302crypt_engine    *engine;

    /*! List head for storing registered algorithms for each engine. */
    struct list_head            entry;
};

/*!
 * \brief Generic context structure for any algorithm type.
 */
struct pc302crypt_generic_ctx
{
    /*! The engine that the context is associated with. */
    struct pc302crypt_engine    *engine;

    /*! The type of operation. */
    int                         flags;
};

/*!
 * \brief Block cipher context.
 */
struct pc302crypt_ablk_ctx
{
    /*! The algorithm independent generic information. */
    struct pc302crypt_generic_ctx   generic;

    /*! The cipher key. */
    u8                          key[ PC302_CRYPTO_AES_MAX_KEY_LEN ];

    /*! The length of the key in bytes. */
    u8                          key_len;

    /*! The fallback cipher. If the operation can't be done in hardware,
     *  fallback to a software version. */
    struct crypto_ablkcipher    *sw_cipher;
};

/*!
 * \brief AEAD cipher context.
 */
struct pc302crypt_aead_ctx
{
    /*! The algorithm indenpendent generic information. */
    struct pc302crypt_generic_ctx   generic;

    /*! The block cipher key. */
    u8                          cipher_key[ PC302_CRYPTO_AES_MAX_KEY_LEN ];

    /*! The hash key. */
    u8                          hash_ctx[ PC302_CRYPTO_IPSEC_HASH_PG_SZ ];

    /*! The length of the cipher key in bytes. */
    u8                          cipher_key_len;

    /*! The length of the hash key in bytes. */
    u8                          hash_key_len;

    /*! The fallback cipher. */
    struct crypto_aead          *sw_cipher;

    /*! The length of the hash output. */
    size_t                      auth_size;

    /*! The salt for givencrypt requests. */
    u8                          salt[ PC302_CRYPTO_AES_IV_LEN ];
};

static int
pc302crypt_ablk_setkey( struct crypto_ablkcipher *cipher,
                        const u8 *key,
                        unsigned int len );

static int
pc302crypt_ablk_encrypt( struct ablkcipher_request *req );

static int
pc302crypt_ablk_decrypt( struct ablkcipher_request *req );

static int
pc302crypt_ablk_cra_init( struct crypto_tfm *tfm );

static void
pc302crypt_ablk_cra_exit( struct crypto_tfm *tfm );

static int
pc302crypt_aead_setkey( struct crypto_aead *tfm,
                        const u8 *key,
                        unsigned int keylen );

static int
pc302crypt_aead_encrypt( struct aead_request *req );

static int
pc302crypt_aead_givencrypt( struct aead_givcrypt_request *req );

static int
pc302crypt_aead_decrypt( struct aead_request *req );

static int
pc302crypt_aead_cra_init( struct crypto_tfm *tfm );

static void
pc302crypt_aead_cra_exit( struct crypto_tfm *tfm );

static int
pc302crypt_aead_setauthsize( struct crypto_aead *tfm,
                             unsigned int authsize );

static int
ablk_null_givencrypt( struct skcipher_givcrypt_request *req )
{
    return crypto_ablkcipher_encrypt(&req->creq);
}

static int
ablk_null_givdecrypt( struct skcipher_givcrypt_request *req )
{
    return crypto_ablkcipher_decrypt(&req->creq);
}

static struct pc302crypt_alg ipsec_engine_algs[] = {
    {
        .type           = PC302_CRYPTO_ALG_AES | PC302_CRYPTO_ABLK_MODE_CBC,
        .alg            = {
            .cra_name           = "cbc(aes)",
            .cra_driver_name    = "cbc-aes-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER |
                                  CRYPTO_ALG_ASYNC |
                                  CRYPTO_ALG_NEED_FALLBACK,
            .cra_blocksize      = 16,
            .cra_ctxsize        = sizeof( struct pc302crypt_ablk_ctx ),
            .cra_type           = &crypto_ablkcipher_type,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher     = {
                .setkey         = pc302crypt_ablk_setkey,
                .encrypt        = pc302crypt_ablk_encrypt,
                .decrypt        = pc302crypt_ablk_decrypt,
                .min_keysize    = 16,
                .max_keysize    = 32,
                .ivsize         = 16,
            },
            .cra_init           = pc302crypt_ablk_cra_init,
            .cra_exit           = pc302crypt_ablk_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_AES | PC302_CRYPTO_ABLK_MODE_ECB,
        .alg            = {
            .cra_name           = "ecb(aes)",
            .cra_driver_name    = "ecb-aes-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER |
                                  CRYPTO_ALG_ASYNC |
                                  CRYPTO_ALG_NEED_FALLBACK,
            .cra_blocksize      = 16,
            .cra_ctxsize        = sizeof( struct pc302crypt_ablk_ctx ),
            .cra_type           = &crypto_ablkcipher_type,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher     = {
                .setkey         = pc302crypt_ablk_setkey,
                .encrypt        = pc302crypt_ablk_encrypt,
                .decrypt        = pc302crypt_ablk_decrypt,
                .min_keysize    = 16,
                .max_keysize    = 32,
            },
            .cra_init           = pc302crypt_ablk_cra_init,
            .cra_exit           = pc302crypt_ablk_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_DES | PC302_CRYPTO_ABLK_MODE_CBC,
        .alg            = {
            .cra_name           = "cbc(des)",
            .cra_driver_name    = "cbc-des-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 8,
            .cra_ctxsize        = sizeof( struct pc302crypt_ablk_ctx ),
            .cra_type           = &crypto_ablkcipher_type,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher     = {
                .setkey         = pc302crypt_ablk_setkey,
                .encrypt        = pc302crypt_ablk_encrypt,
                .decrypt        = pc302crypt_ablk_decrypt,
                .min_keysize    = 8,
                .max_keysize    = 8,
                .ivsize         = 8,
            },
            .cra_init           = pc302crypt_ablk_cra_init,
            .cra_exit           = pc302crypt_ablk_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_DES | PC302_CRYPTO_ABLK_MODE_ECB,
        .alg            = {
            .cra_name           = "ecb(des)",
            .cra_driver_name    = "ecb-des-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 8,
            .cra_ctxsize        = sizeof( struct pc302crypt_ablk_ctx ),
            .cra_type           = &crypto_ablkcipher_type,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher     = {
                .setkey         = pc302crypt_ablk_setkey,
                .encrypt        = pc302crypt_ablk_encrypt,
                .decrypt        = pc302crypt_ablk_decrypt,
                .min_keysize    = 8,
                .max_keysize    = 8,
            },
            .cra_init           = pc302crypt_ablk_cra_init,
            .cra_exit           = pc302crypt_ablk_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_3DES | PC302_CRYPTO_ABLK_MODE_CBC,
        .alg            = {
            .cra_name           = "cbc(des3_ede)",
            .cra_driver_name    = "cbc-des3-ede-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 8,
            .cra_ctxsize        = sizeof( struct pc302crypt_ablk_ctx ),
            .cra_type           = &crypto_ablkcipher_type,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher     = {
                .setkey         = pc302crypt_ablk_setkey,
                .encrypt        = pc302crypt_ablk_encrypt,
                .decrypt        = pc302crypt_ablk_decrypt,
                .min_keysize    = 24,
                .max_keysize    = 24,
                .ivsize         = 8,
            },
            .cra_init           = pc302crypt_ablk_cra_init,
            .cra_exit           = pc302crypt_ablk_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_DES | PC302_CRYPTO_ABLK_MODE_ECB,
        .alg            = {
            .cra_name           = "ecb(des3_ede)",
            .cra_driver_name    = "ecb-des3-ede-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 8,
            .cra_ctxsize        = sizeof( struct pc302crypt_ablk_ctx ),
            .cra_type           = &crypto_ablkcipher_type,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher     = {
                .setkey         = pc302crypt_ablk_setkey,
                .encrypt        = pc302crypt_ablk_encrypt,
                .decrypt        = pc302crypt_ablk_decrypt,
                .min_keysize    = 24,
                .max_keysize    = 24,
            },
            .cra_init           = pc302crypt_ablk_cra_init,
            .cra_exit           = pc302crypt_ablk_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_AUTHENC_AES_CBC_HMAC_SHA1,
        .alg            = {
            .cra_name           = "authenc(hmac(sha1),cbc(aes))",
            .cra_driver_name    = "authenc-hmac-sha1-cbc-aes-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_AEAD |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 16,
            .cra_ctxsize        = sizeof( struct pc302crypt_aead_ctx ),
            .cra_type           = &crypto_aead_type,
            .cra_module         = THIS_MODULE,
            .cra_aead           = {
                .setkey         = pc302crypt_aead_setkey,
                .setauthsize    = pc302crypt_aead_setauthsize,
                .encrypt        = pc302crypt_aead_encrypt,
                .decrypt        = pc302crypt_aead_decrypt,
                .givencrypt     = pc302crypt_aead_givencrypt,
                .ivsize         = 16,
                .maxauthsize    = 20,
            },
            .cra_init           = pc302crypt_aead_cra_init,
            .cra_exit           = pc302crypt_aead_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_AUTHENC_AES_CBC_HMAC_SHA256,
        .alg            = {
            .cra_name           = "authenc(hmac(sha256),cbc(aes))",
            .cra_driver_name    = "authenc-hmac-sha256-cbc-aes-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_AEAD |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 16,
            .cra_ctxsize        = sizeof( struct pc302crypt_aead_ctx ),
            .cra_type           = &crypto_aead_type,
            .cra_module         = THIS_MODULE,
            .cra_aead           = {
                .setkey         = pc302crypt_aead_setkey,
                .setauthsize    = pc302crypt_aead_setauthsize,
                .encrypt        = pc302crypt_aead_encrypt,
                .decrypt        = pc302crypt_aead_decrypt,
                .givencrypt     = pc302crypt_aead_givencrypt,
                .ivsize         = 16,
                .maxauthsize    = 32,
            },
            .cra_init           = pc302crypt_aead_cra_init,
            .cra_exit           = pc302crypt_aead_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_AUTHENC_AES_CBC_HMAC_MD5,
        .alg            = {
            .cra_name           = "authenc(hmac(md5),cbc(aes))",
            .cra_driver_name    = "authenc-hmac-md5-cbc-aes-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_AEAD |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 16,
            .cra_ctxsize        = sizeof( struct pc302crypt_aead_ctx ),
            .cra_type           = &crypto_aead_type,
            .cra_module         = THIS_MODULE,
            .cra_aead           = {
                .setkey         = pc302crypt_aead_setkey,
                .setauthsize    = pc302crypt_aead_setauthsize,
                .encrypt        = pc302crypt_aead_encrypt,
                .decrypt        = pc302crypt_aead_decrypt,
                .givencrypt     = pc302crypt_aead_givencrypt,
                .ivsize         = 16,
                .maxauthsize    = 16,
            },
            .cra_init           = pc302crypt_aead_cra_init,
            .cra_exit           = pc302crypt_aead_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_AUTHENC_3DES_CBC_HMAC_SHA1,
        .alg            = {
            .cra_name           = "authenc(hmac(sha1),cbc(des3_ede))",
            .cra_driver_name    = "authenc-hmac-sha1-cbc-3des-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_AEAD |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 8,
            .cra_ctxsize        = sizeof( struct pc302crypt_aead_ctx ),
            .cra_type           = &crypto_aead_type,
            .cra_module         = THIS_MODULE,
            .cra_aead           = {
                .setkey         = pc302crypt_aead_setkey,
                .setauthsize    = pc302crypt_aead_setauthsize,
                .encrypt        = pc302crypt_aead_encrypt,
                .decrypt        = pc302crypt_aead_decrypt,
                .givencrypt     = pc302crypt_aead_givencrypt,
                .ivsize         = 8,
                .maxauthsize    = 20,
            },
            .cra_init           = pc302crypt_aead_cra_init,
            .cra_exit           = pc302crypt_aead_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_AUTHENC_3DES_CBC_HMAC_SHA256,
        .alg            = {
            .cra_name           = "authenc(hmac(sha256),cbc(des3_ede))",
            .cra_driver_name    = "authenc-hmac-sha256-cbc-3des-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_AEAD |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 8,
            .cra_ctxsize        = sizeof( struct pc302crypt_aead_ctx ),
            .cra_type           = &crypto_aead_type,
            .cra_module         = THIS_MODULE,
            .cra_aead           = {
                .setkey         = pc302crypt_aead_setkey,
                .setauthsize    = pc302crypt_aead_setauthsize,
                .encrypt        = pc302crypt_aead_encrypt,
                .decrypt        = pc302crypt_aead_decrypt,
                .givencrypt     = pc302crypt_aead_givencrypt,
                .ivsize         = 8,
                .maxauthsize    = 32,
            },
            .cra_init           = pc302crypt_aead_cra_init,
            .cra_exit           = pc302crypt_aead_cra_exit,
        },
    },
    {
        .type           = PC302_CRYPTO_ALG_AUTHENC_3DES_CBC_HMAC_MD5,
        .alg            = {
            .cra_name           = "authenc(hmac(md5),cbc(des3_ede))",
            .cra_driver_name    = "authenc-hmac-md5-cbc-3des-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_AEAD |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 8,
            .cra_ctxsize        = sizeof( struct pc302crypt_aead_ctx ),
            .cra_type           = &crypto_aead_type,
            .cra_module         = THIS_MODULE,
            .cra_aead           = {
                .setkey         = pc302crypt_aead_setkey,
                .setauthsize    = pc302crypt_aead_setauthsize,
                .encrypt        = pc302crypt_aead_encrypt,
                .decrypt        = pc302crypt_aead_decrypt,
                .givencrypt     = pc302crypt_aead_givencrypt,
                .ivsize         = 8,
                .maxauthsize    = 16,
            },
            .cra_init           = pc302crypt_aead_cra_init,
            .cra_exit           = pc302crypt_aead_cra_exit,
        },
    },
};

/* Although we never want to use f8(kasumi) as a geniv cipher, we need to make
 * it one so that the kernel doesn't try to wrap it up in a generic IV
 * generator. If we let it do this then as it doesn't know about the f8 mode
 * or Kasumi algorithm it fails. Best to tell it we have our own IV generator
 * and turn givencrypt/givdecrypt requests into normal encrypt/decrypt
 * requests. */
static struct pc302crypt_alg l2_engine_algs[] = {
    {
        .type           = PC302_CRYPTO_ALG_KASUMI |
                          PC302_CRYPTO_ABLK_MODE_F8,
        .alg            = {
            .cra_name           = "f8(kasumi)",
            .cra_driver_name    = "f8-kasumi-pc302",
            .cra_priority       = PC302_CRYPTO_ALG_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_GIVCIPHER |
                                  CRYPTO_ALG_ASYNC,
            .cra_blocksize      = 8,
            .cra_ctxsize        = sizeof( struct pc302crypt_ablk_ctx ),
            .cra_type           = &crypto_ablkcipher_type,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher     = {
                .setkey         = pc302crypt_ablk_setkey,
                .encrypt        = pc302crypt_ablk_encrypt,
                .decrypt        = pc302crypt_ablk_decrypt,
                .givencrypt     = ablk_null_givencrypt,
                .givdecrypt     = ablk_null_givdecrypt,
                .min_keysize    = 16,
                .max_keysize    = 16,
                .ivsize         = 8,
                .geniv          = "internal",
            },
            .cra_init           = pc302crypt_ablk_cra_init,
            .cra_exit           = pc302crypt_ablk_cra_exit,
        },
    },
};

/*! The IPSec offload engine. */
static struct pc302crypt_engine ipsec_engine = {
    .name               = "ipsec_engine",
    .hw_lock            = SPIN_LOCK_UNLOCKED,
    .max_ctxs           = PC302_CRYPTO_IPSEC_MAX_CTXS,
    .cipher_pg_sz       = PC302_CRYPTO_IPSEC_CIPHER_PG_SZ,
    .hash_pg_sz         = PC302_CRYPTO_IPSEC_HASH_PG_SZ,
    .fifo_sz            = PC302_CRYPTO_IPSEC_FIFO_SZ,
    .algs               = ipsec_engine_algs,
    .num_algs           = ARRAY_SIZE( ipsec_engine_algs ),
    .stat_irq_threshold = STAT_IRQ_THRESHOLD_IPSEC,
};

/*! The layer 2 offload engine. */
static struct pc302crypt_engine l2_engine = {
    .name               = "l2_engine",
    .hw_lock            = SPIN_LOCK_UNLOCKED,
    .max_ctxs           = PC302_CRYPTO_L2_MAX_CTXS,
    .cipher_pg_sz       = PC302_CRYPTO_L2_CIPHER_PG_SZ,
    .hash_pg_sz         = PC302_CRYPTO_L2_HASH_PG_SZ,
    .fifo_sz            = PC302_CRYPTO_L2_FIFO_SZ,
    .algs               = l2_engine_algs,
    .num_algs           = ARRAY_SIZE( l2_engine_algs ),
    .stat_irq_threshold = STAT_IRQ_THRESHOLD_L2,
};

/*!
 * From a generic context, get the block cipher context containing it.
 *
 * @param ctx The generic context.
 * @return Returns a pointer to the block cipher context on success,
 * NULL on failure.
 */
static inline struct pc302crypt_ablk_ctx *
to_pc302crypt_ablk_ctx( struct pc302crypt_generic_ctx *ctx )
{
    return ctx ? container_of( ctx, struct pc302crypt_ablk_ctx, generic ) :
        NULL;
}

#ifdef PROFILE_PC302CRYPTO_USING_HIRES_TIMER
static u32 profileN = 0;
static u32 profileTotalSum = 0;
static u32 profileTotalSumSquares = 0;
static u32 profileTestSum = 0;
static u32 profileTestSumSquares = 0;
static u32 profileDelaySum = 0;
static u32 profileTotal=0;
static u32 profileTotalStart=0;
static u32 profileTest=0;
static u32 profileTestStart=0;

static inline u32
pc302crypt_get_cycles(void)
{
    return ioread32(__io(IO_ADDRESS(PC302_RTCLK_BASE + RTCLK_CCV_REG_OFFSET)));
}
#endif

/*!
 * From a generic context, get the AEAD cipher context containing it.
 *
 * @param ctx The generic context.
 * @return Returns a pointer to the AEAD cipher context on success,
 * NULL on failure.
 */
static inline struct pc302crypt_aead_ctx *
to_pc302crypt_aead_ctx( struct pc302crypt_generic_ctx *ctx )
{
    return ctx ? container_of( ctx, struct pc302crypt_aead_ctx, generic ) :
        NULL;
}

static inline struct pc302crypt_alg *
to_pc302crypt_alg( struct crypto_alg *alg );

/*!
 * Write to a register in the crypto offload engine.
 *
 * @param engine The engine to write to.
 * @param offset The offset of the register from the engine base address.
 * @param value The value to write to the register.
 */
static inline void
pc302crypt_reg_write( struct pc302crypt_engine *engine,
                      unsigned long offset,
                      u32 value )
{
    void __iomem *addr = engine->regs + offset;
    PRINTD( DBG_IO, "reg_wr, engine=\"%s\": 0x%04x := 0x%08x", engine->name,
            ( unsigned )offset, value );
    iowrite32( value, addr );
}

/*!
 * Read from a register in the crypto offload engine.
 *
 * @param engine The engine to read from.
 * @param offset The offset of the register from the engine base address.
 * @return Returns the value of the register.
 */
static inline u32
pc302crypt_reg_read( struct pc302crypt_engine *engine,
                     unsigned long offset )
{
    void __iomem *addr = engine->regs + offset;
    u32 val = ioread32( addr );
    PRINTD( DBG_IO, "reg_rd, engine=\"%s\": 0x%04x == 0x%08x", engine->name,
            ( unsigned )offset, val );
    return val;
}

/*!
 * Given a cipher context, and a context number, get the base address of the
 * context page.
 *
 * @param ctx The context of the algorithm.
 * @param indx The index of the page to use inside the engine.
 * @param cipher_ctx Boolean flag indicating that the context is a cipher and
 * not a hash.
 * @return Returns the address of the context page where the key/context may
 * be written.
 */
static inline void __iomem *
pc302crypt_ctx_page_addr( struct pc302crypt_generic_ctx *ctx,
                          unsigned indx,
                          int cipher_ctx )
{
    if ( cipher_ctx )
    {
        return ctx->engine->cipher_ctx_base +
               ( indx * ctx->engine->cipher_pg_sz );
    }
    else
    {
        return ctx->engine->hash_key_base +
               ( indx * ctx->engine->hash_pg_sz );
    }
}

static inline void
memcpy_toio32( u32 __iomem *dst,
               const void *src,
               unsigned count )
{
    u32 *src32 = ( u32 * )src;
    while ( count-- )
        iowrite32( *src32++, dst++ );
}

/*!
 * Write a key and IV pair into the correct context format for the engine.
 *
 * @param ctx The context to write into the engine.
 * @param page_addr The address of the context page to write into.
 * @param key The key to write.
 * @param key_len The length of the key in bytes.
 * @param iv The IV.
 * @param iv_len The length of the IV in bytes.
 */
static void
pc302crypt_cipher_write_ctx( struct pc302crypt_generic_ctx *ctx,
                             void __iomem *page_addr,
                             const u8 *key,
                             size_t key_len,
                             const u8 *iv,
                             size_t iv_len )
{
    void __iomem *key_ptr;
    void __iomem *iv_ptr;

    switch ( ( ctx->flags & PC302_CRYPTO_BLOCK_MASK ) | PC302_CRYPTO_BLOCK )
    {
        case PC302_CRYPTO_ALG_AES:
            key_ptr = page_addr;
            iv_ptr = page_addr + PC302_CRYPTO_AES_MAX_KEY_LEN;
            break;

        case PC302_CRYPTO_ALG_3DES:
        case PC302_CRYPTO_ALG_DES:
            key_ptr = page_addr + PC302_CRYPTO_DES_IV_LEN;
            iv_ptr = page_addr;
            break;

        case PC302_CRYPTO_ALG_KASUMI:
            key_ptr = page_addr;
            iv_ptr = page_addr + PC302_CRYPTO_KASUMI_F8_KEY_LEN;
            break;

        default:
            BUG();
    }

    memcpy_toio32( key_ptr, key, key_len / 4 );
    memcpy_toio32( iv_ptr, iv, iv_len / 4 );
}

/*! Context load mode. */
enum context_load_mode
{
    CTX_CIPHER,
    CTX_HASH,
    CTX_COMBINED,
};

/*!
 * Load a context into the engines context memory.
 *
 * @param ctx The context to load.
 * @param iv The IV of the operation.
 * @param ivlen The length of the IV in bytes.
 * @param mode The context load type.
 * @return Returns the index of the context page where the context was loaded.
 */
static unsigned
pc302crypt_load_ctx( struct pc302crypt_generic_ctx *ctx,
                     u8 *iv,
                     size_t ivlen,
                     enum context_load_mode mode )
{
    unsigned indx;

    /* We failed to find a context slot. There are the same number of context
     * pages as there are slots in the FIFO so it should be impossible to get
     * a context slot. */
    BUG_ON( NULL == ctx->engine->free_ddt_info );

    indx = ctx->engine->free_ddt_info - ctx->engine->ddt_info;
    
    ctx->engine->free_ddt_info = ctx->engine->ddt_info[ indx ].next;
    ctx->engine->ddt_info[ indx ].next = NULL;

    if ( CTX_CIPHER == mode )
    {
        struct pc302crypt_ablk_ctx *ablk_ctx = to_pc302crypt_ablk_ctx( ctx );
        pc302crypt_cipher_write_ctx( ctx,
                                     pc302crypt_ctx_page_addr( ctx, indx, 1 ),
                                     ablk_ctx->key, ablk_ctx->key_len,
                                     iv, ivlen );
        pc302crypt_reg_write( ctx->engine, SPA_KEY_SZ_REG_OFFSET,
                              ablk_ctx->key_len |
                              indx << SPA_KEY_SZ_CTX_INDEX_OFFSET |
                              ( 1 << SPA_KEY_SZ_CIPHER_OFFSET ) );
    }
    else if ( CTX_COMBINED == mode )
    {
        struct pc302crypt_aead_ctx *aead_ctx = to_pc302crypt_aead_ctx( ctx );

        /* Write the cipher context. */
        pc302crypt_cipher_write_ctx( ctx,
                                     pc302crypt_ctx_page_addr( ctx, indx, 1 ),
                                     aead_ctx->cipher_key,
                                     aead_ctx->cipher_key_len,
                                     iv, ivlen );
        pc302crypt_reg_write( ctx->engine, SPA_KEY_SZ_REG_OFFSET,
                              aead_ctx->cipher_key_len |
                              indx << SPA_KEY_SZ_CTX_INDEX_OFFSET |
                              ( 1 << SPA_KEY_SZ_CIPHER_OFFSET ) );

        /* Write the hash context. */
        memcpy_toio32( pc302crypt_ctx_page_addr( ctx, indx, 0 ),
                       &aead_ctx->hash_ctx, aead_ctx->hash_key_len / 4 );
        pc302crypt_reg_write( ctx->engine, SPA_KEY_SZ_REG_OFFSET,
                              aead_ctx->hash_key_len |
                              indx << SPA_KEY_SZ_CTX_INDEX_OFFSET |
                              ( 0 << SPA_KEY_SZ_CIPHER_OFFSET ) );
    }
    else
    {
        /* Unsupported context type. */
        BUG();
    }

    return indx;
}

/*!
 * Unload a context from the crypto engine.
 *
 * @param ctx The context to unload.
 * @param indx The context page address to unload.
 */
static void
pc302crypt_unload_ctx( struct pc302crypt_generic_ctx *ctx,
                       unsigned indx )
{
    ctx->engine->ddt_info[ indx ].next = ctx->engine->free_ddt_info;
    ctx->engine->free_ddt_info = &ctx->engine->ddt_info[ indx ];
}

/*!
 * Initialise a PC302 crypto queue for max_qlen entries.
 *
 * @param queue The queue to initialise.
 * @param max_qlen The maximum number of entries that the queue may hold.
 */
static void
pc302crypt_queue_init( struct pc302crypt_queue *queue,
                       unsigned max_qlen )
{
    INIT_LIST_HEAD(&queue->list);
    queue->backlog = &queue->list;
    queue->qlen = 0;
    queue->max_qlen = max_qlen;
}

/*!
 * Add a crypto request into the crypto queue.
 *
 * @param queue The queue to add the request into.
 * @param request The request to add to the queue.
 * @return Returns -EINPROGRESS if the request is added, other negative values
 * on failure.
 */
static int
pc302crypt_queue_req( struct pc302crypt_queue *queue,
                      struct pc302crypt_req *request )
{
    int err = -EINPROGRESS;

    if ( unlikely( queue->qlen >= queue->max_qlen ) )
    {
        err = -EBUSY;
        if ( !( request->req->flags & CRYPTO_TFM_REQ_MAY_BACKLOG ) )
            goto out;
        if ( queue->backlog == &queue->list )
             queue->backlog = &request->list;
    }

    queue->qlen++;
    list_add_tail( &request->list, &queue->list );

out:
    return err;
}

/*!
 * Force a request to be added to the crypto queue. This is useful when we
 * have completed requests and backlogging does not matter.
 *
 * @param queue The queue to add the request to.
 * @param request The request to add to the queue.
 * @param Returns -EINPROGRESS.
 */
static int
pc302crypt_queue_req_force( struct pc302crypt_queue *queue,
                            struct pc302crypt_req *request )
{
    if ( queue->qlen >= queue->max_qlen && queue->backlog == &queue->list )
        queue->backlog = &request->list;

    queue->qlen++;
    list_add_tail( &request->list, &queue->list );

    return -EINPROGRESS;
}

/*!
 * Remove a request from the head of a crypto queue.
 *
 * @param queue The queue to remove the request from.
 * @return Returns the request from the queue head on success, NULL on
 * failure.
 */
static struct pc302crypt_req *
pc302crypt_dequeue_req( struct pc302crypt_queue *queue )
{
    struct list_head *request;

    if ( unlikely( !queue->qlen ) )
        return NULL;

    queue->qlen--;

    if ( queue->backlog != &queue->list )
        queue->backlog = queue->backlog->next;

    request = queue->list.next;
    list_del( request );

    return list_entry( request, struct pc302crypt_req, list );
}

/*!
 * Count the number of scatterlist entries in a scatterlist.
 *
 * @param sg_list The scatterlist to count.
 * @param nbytes The number of bytes in the scatterlist.
 * @return Returns the number of entries in the scatterlist.
 */
static int
sg_count( struct scatterlist *sg_list,
          int nbytes )
{
    struct scatterlist *sg = sg_list;
    int sg_nents = 0;

    while ( nbytes > 0 )
    {
        ++sg_nents;
        nbytes -= sg->length;
        sg = sg_next( sg );
    }

    return sg_nents;
}

/*!
 * Take a crypto request and scatterlists for the data and turn them into DDTs
 * for passing to the crypto engines. This also DMA maps the data so that the
 * crypto engines can DMA to/from them.
 *
 * @param req The request to convert.
 * @param payload The payload data.
 * @param nbytes The number of bytes in the payload scatterlist.
 * @param dir The direction of the transfer.
 * @return Returns the DDT list on success, NULL on failure.
 */
static struct pc302crypt_ddt *
pc302crypt_sg_to_ddt( struct pc302crypt_req *req,
                      struct scatterlist *payload,
                      unsigned nbytes,
                      enum dma_data_direction dir )
{
    unsigned nents = sg_count( payload, nbytes );
    struct scatterlist *cur;
    unsigned i = 0;
    struct pc302crypt_ablk_ctx *ablk_ctx = crypto_tfm_ctx( req->req->tfm );
    struct pc302crypt_engine *engine = ablk_ctx->generic.engine;
    struct pc302crypt_ddt *ddt = dir == DMA_TO_DEVICE ?
        engine->ddt_buf[ req->ctx_id ].src_ddt :
        engine->ddt_buf[ req->ctx_id ].dst_ddt;
    unsigned mapped_ents = dma_map_sg( engine->dev, payload, nents, dir );
    unsigned nddt_entries = mapped_ents;

    /* If we can't make a DDT large enough then fail. */
    if ( nddt_entries + 1 > MAX_DDT_LEN )
    {
        PRINTD( DBG_WARN, "DDT of %u required (>%u)", nddt_entries + 1,
                MAX_DDT_LEN );
        return NULL;
    }

    for_each_sg( payload, cur, mapped_ents, i )
    {
        ddt[ i ].p = sg_dma_address( cur );
        ddt[ i ].len = sg_dma_len( cur );
    }

    ddt[ nddt_entries ].p = 0;
    ddt[ nddt_entries ].len = 0;

    return ddt;
}

static int
pc302crypt_aead_make_ddts( struct pc302crypt_req *req,
                           u8 *giv,
                           u64 seq )
{
    int ret = 0;
    struct aead_request *areq =
        container_of( req->req, struct aead_request, base );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( req->req->tfm->__crt_alg );
    struct pc302crypt_ablk_ctx *aead_ctx = crypto_tfm_ctx( req->req->tfm );
    struct pc302crypt_engine *engine = aead_ctx->generic.engine;
    struct pc302crypt_ddt *src_ddt = engine->ddt_buf[ req->ctx_id ].src_ddt;
    struct pc302crypt_ddt *dst_ddt = engine->ddt_buf[ req->ctx_id ].dst_ddt;
    unsigned ivsize = alg->alg.cra_aead.ivsize;
    unsigned nents = sg_count( areq->src, areq->cryptlen );
    int assoc_ents = dma_map_sg( engine->dev, areq->assoc,
                                 sg_count( areq->assoc, areq->assoclen ),
                                 DMA_TO_DEVICE );
    dma_addr_t iv_addr;
    struct scatterlist *cur;
    unsigned src_pos = 0;
    unsigned dst_pos = 0;
    unsigned i;
    int ddt_len;
    int dst_ents = 0;
    int src_ents = 0;

    if ( areq->src != areq->dst )
    {
        src_ents = dma_map_sg( engine->dev, areq->src, nents,
                               DMA_TO_DEVICE );
        dst_ents = dma_map_sg( engine->dev, areq->dst, nents,
                               DMA_FROM_DEVICE );
    }
    else
    {
        src_ents = dma_map_sg( engine->dev, areq->src, nents,
                               DMA_BIDIRECTIONAL );
    }

    /* Number of DDT entries. +2 because we need to include the IV and the
     * NULL terminator. */
    ddt_len = src_ents + dst_ents + assoc_ents + 2;

    if ( giv )
        iv_addr = dma_map_single( engine->dev, giv, ivsize,
                                  DMA_BIDIRECTIONAL );
    else
        iv_addr = dma_map_single( engine->dev, areq->iv, ivsize,
                                  DMA_TO_DEVICE );
    req->giv_pa = iv_addr;

    if ( src_ents < 0 || assoc_ents < 0 )
    {
        PRINTD( DBG_WARN, "failed to map payload or associated data" );
        ret = -EIO;
        goto out;
    }

    req->ddt_len = ddt_len;

    for_each_sg( areq->assoc, cur, assoc_ents, i )
    {
        src_ddt[ src_pos ].p = sg_dma_address( cur );
        src_ddt[ src_pos ].len = sg_dma_len( cur );
        ++src_pos;

        if ( ENCRYPT == req->dir )
        {
            dst_ddt[ dst_pos ].p = sg_dma_address( cur );
            dst_ddt[ dst_pos ].len = sg_dma_len( cur );
            ++dst_pos;
        }
    }

    src_ddt[ src_pos ].p = iv_addr;
    src_ddt[ src_pos ].len = ivsize;
    ++src_pos;

    if ( giv || ENCRYPT == req->dir )
    {
        dst_ddt[ dst_pos ].p = iv_addr;
        dst_ddt[ dst_pos ].len = ivsize;
        ++dst_pos;
    }

    for_each_sg( areq->src, cur, src_ents, i )
    {
        src_ddt[ src_pos ].p = sg_dma_address( cur );
        src_ddt[ src_pos ].len = sg_dma_len( cur );
        ++src_pos;
        if ( areq->src == areq->dst )
        {
            dst_ddt[ dst_pos ].p = sg_dma_address( cur );
            dst_ddt[ dst_pos ].len = sg_dma_len( cur );
            ++dst_pos;
        }
    }

    if ( areq->src != areq->dst )
        for_each_sg( areq->dst, cur, dst_ents, i )
        {
            dst_ddt[ dst_pos ].p = sg_dma_address( cur );
            dst_ddt[ dst_pos ].len = sg_dma_len( cur );
            ++dst_pos;
        }

    src_ddt[ src_pos ].p = 0;
    src_ddt[ src_pos ].len = 0;
    dst_ddt[ dst_pos ].p = 0;
    dst_ddt[ dst_pos ].len = 0;

    ret = 0;

out:
    return ret;
}

static void
pc302crypt_aead_kill_ddts( struct pc302crypt_req *req )
{
    struct aead_request *areq =
        container_of( req->req, struct aead_request, base );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( req->req->tfm->__crt_alg );
    struct pc302crypt_ablk_ctx *aead_ctx = crypto_tfm_ctx( req->req->tfm );
    struct pc302crypt_engine *engine = aead_ctx->generic.engine;
    unsigned ivsize = alg->alg.cra_aead.ivsize;
    unsigned nents = sg_count( areq->src, areq->cryptlen );
    if ( areq->src != areq->dst )
    {
        dma_unmap_sg( engine->dev, areq->src, nents, DMA_TO_DEVICE );
        dma_unmap_sg( engine->dev, areq->dst,
                      sg_count( areq->dst, areq->cryptlen ), DMA_FROM_DEVICE );
    }
    else
    {
        dma_unmap_sg( engine->dev, areq->src, nents, DMA_BIDIRECTIONAL );
    }
    dma_unmap_sg( engine->dev, areq->assoc,
                  sg_count( areq->assoc, areq->assoclen ), DMA_TO_DEVICE );

    dma_unmap_single( engine->dev, req->giv_pa, ivsize,
                      DMA_BIDIRECTIONAL );
}

/*!
 * Free a DDT list.
 *
 * @param req The request to free the DDT list for.
 * @param assoc The associated data for the request. If this is a block cipher
 * then this may be NULL.
 * @param payload The payload data for the operation.
 * @param giv The generated IV.
 * @param ivlen The length of the generated IV.
 * @param nbytes The number of bytes in the payload scatterlist.
 * @param dir The direction of the operation.
 */
static void
pc302crypt_free_ddt( struct pc302crypt_req *req,
                     struct scatterlist *payload,
                     unsigned nbytes,
                     enum dma_data_direction dir )
{
    if (payload)
    {
        unsigned nents = sg_count( payload, nbytes );
        struct pc302crypt_ablk_ctx *ablk_ctx = crypto_tfm_ctx( req->req->tfm );
        struct pc302crypt_engine *engine = ablk_ctx->generic.engine;
        dma_unmap_sg( engine->dev, payload, nents, dir );
    }
}

/*!
 * Set key for a DES operation in an AEAD cipher. This also performs weak key
 * checking if required.
 *
 * @param aead The transform to set the key for.
 * @param key The key to set.
 * @param len The length of the key in bytes.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_aead_des_setkey( struct crypto_aead *aead,
                            const u8 *key,
                            unsigned int len )
{
    struct crypto_tfm *tfm = crypto_aead_tfm( aead );
    struct pc302crypt_aead_ctx *ctx = crypto_tfm_ctx( tfm );
    int err = 0;
    u32 tmp[ DES_EXPKEY_WORDS ];

    err = des_ekey( tmp, key );
    if ( unlikely( !err ) &&
            ( crypto_aead_get_flags( aead) ) & CRYPTO_TFM_REQ_WEAK_KEY )
    {
        tfm->crt_flags |= CRYPTO_TFM_RES_WEAK_KEY;
        return -EINVAL;
    }
    err = 0;

    memcpy( ctx->cipher_key, key, len );
    ctx->cipher_key_len = len;

    return err;
}

/*!
 * Set the key for the AES block cipher component of the AEAD transform. 
 *
 * @param aead The transform to set the key for.
 * @param key The key to set.
 * @param len The length of the key in bytes.
 *
 * @return Returns zero on success, negative on failure, and 1 if we require a
 * fallback.
 */
static int
pc302crypt_aead_aes_setkey( struct crypto_aead *aead,
                            const u8 *key,
                            unsigned int len )
{
    struct crypto_tfm *tfm = crypto_aead_tfm( aead );
    struct pc302crypt_aead_ctx *ctx = crypto_tfm_ctx( tfm );

    /* IPSec engine only supports 128 and 256 bit AES keys. If we get a
     * request for any other size (192 bits) then we need to do a software
     * fallback. */
    if ( !( 16 == len || 32 == len ) )
        return 1;

    memcpy( ctx->cipher_key, key, len );
    ctx->cipher_key_len = len;

    return 0;
}

/*!
 * Set the key for an AEAD operation.
 *
 * @param tfm The transform to set the key for.
 * @param key The key to set.
 * @param keylen The length of the key in bytes.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_aead_setkey( struct crypto_aead *tfm,
                        const u8 *key,
                        unsigned int keylen )
{
    struct pc302crypt_aead_ctx *ctx = crypto_aead_ctx( tfm );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( tfm->base.__crt_alg );
    struct rtattr *rta = (void *)key;
    struct crypto_authenc_key_param *param;
    unsigned int authkeylen;
    unsigned int enckeylen;
    int err;

    if ( !RTA_OK( rta, keylen ) )
        goto badkey;

    if ( rta->rta_type != CRYPTO_AUTHENC_KEYA_PARAM )
        goto badkey;

    if ( RTA_PAYLOAD( rta ) < sizeof( *param ) )
        goto badkey;

    param = RTA_DATA( rta );
    enckeylen = be32_to_cpu( param->enckeylen );

    key += RTA_ALIGN( rta->rta_len );
    keylen -= RTA_ALIGN( rta->rta_len );

    if ( keylen < enckeylen )
        goto badkey;

    authkeylen = keylen - enckeylen;

    if ( enckeylen > PC302_CRYPTO_AES_MAX_KEY_LEN )
        goto badkey;

    /* Set the block cipher key. */
    switch ( ( alg->type & PC302_CRYPTO_BLOCK_MASK ) | PC302_CRYPTO_BLOCK )
    {
        case PC302_CRYPTO_ALG_AES:
            err = pc302crypt_aead_aes_setkey( tfm, key + authkeylen,
                                              enckeylen );
            /* We need a fallback so we need to set the fallback algorithm to
             * use the block and hash key pair. */
            if ( 1 == err && ctx->sw_cipher )
            {
                /* Set the fallback transform to use the same request flags as
                 * the hardware transform. */
                ctx->sw_cipher->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
                ctx->sw_cipher->base.crt_flags |=
                    ( tfm->base.crt_flags & CRYPTO_TFM_REQ_MASK );

                err = crypto_aead_setkey( ctx->sw_cipher, key, keylen );
            }
            else if ( 1 == err && !ctx->sw_cipher )
                err = -EINVAL;
            break;

        case PC302_CRYPTO_ALG_3DES:
        case PC302_CRYPTO_ALG_DES:
            err = pc302crypt_aead_des_setkey( tfm, key + authkeylen,
                                              enckeylen );
            break;

        default:
            BUG();
            break;
    }

    if ( err )
        goto badkey;

    memcpy( ctx->hash_ctx, key, authkeylen );
    ctx->hash_key_len = authkeylen;

    return 0;

badkey:
    crypto_aead_set_flags( tfm, CRYPTO_TFM_RES_BAD_KEY_LEN );
    return -EINVAL;
}

/*!
 * Set the authsize field of an AEAD request.
 *
 * @param tfm The transform to set the authsize for.
 * @param authsize The size of the authentication data.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_aead_setauthsize( struct crypto_aead *tfm,
                             unsigned int authsize )
{
    struct pc302crypt_aead_ctx *ctx = crypto_tfm_ctx( crypto_aead_tfm( tfm ) );
    ctx->auth_size = authsize;
    return 0;
}

/*!
 * Check if an AEAD request requires a fallback operation. Some requests can't
 * be completed in hardware because the hardware may not support certain key
 * sizes. In these cases we need to complete the request in software.
 *
 * @param req The request to check.
 * @return Returns 1 if the request needs a fallback, zero otherwise.
 */
static int
pc302crypt_aead_need_fallback( struct pc302crypt_req *req )
{
    struct aead_request *aead_req;
    struct pc302crypt_aead_ctx *ctx;
    struct crypto_tfm *tfm = req->req->tfm;
    struct crypto_alg *alg = req->req->tfm->__crt_alg;
    struct pc302crypt_alg *pc302_alg = to_pc302crypt_alg( alg );

    ctx = crypto_tfm_ctx( tfm );

    aead_req = container_of( req->req, struct aead_request, base );
    /* If we have a non-supported key-length, then we need to do a
     * software fallback. */
    if ( ( ( pc302_alg->type & PC302_CRYPTO_ALG_MASK ) |
           PC302_CRYPTO_BLOCK ) == PC302_CRYPTO_ALG_AES &&
         ( &ipsec_engine == ctx->generic.engine ) &&
         !( 16 == ctx->cipher_key_len || 32 == ctx->cipher_key_len ) )
    {
        return 1;
    }

    return 0;
}

/*!
 * Perform a software fallback of an AEAD request.
 *
 * @param req The request to run.
 * @param alg_type The algorithm type.
 * @param dir The direction of the request. Must be either ENCRYPT or DECRYPT.
 * @return Returns the success/failure status of the fallback operation.
 */
static int
pc302crypt_aead_do_fallback( struct aead_request *req,
                             unsigned alg_type,
                             enum pc302crypt_direction dir )
{
    struct crypto_tfm *old_tfm = crypto_aead_tfm( crypto_aead_reqtfm( req ) );
    struct pc302crypt_aead_ctx *ctx = crypto_tfm_ctx( old_tfm );
    int err;

    if ( ctx->sw_cipher )
    {
        /* Change the request to use the software fallback transform, and once
         * the ciphering has completed, put the old transform back into the
         * request.
         */
        aead_request_set_tfm( req, ctx->sw_cipher );
        err = ( ENCRYPT == dir ) ? crypto_aead_encrypt( req ) :
            crypto_aead_decrypt( req );
        aead_request_set_tfm( req, __crypto_aead_cast( old_tfm ) );
    }
    else
        err = -EINVAL;

    return err;
}

/*!
 * Submit an AEAD request to the crypto offload engine.
 *
 * @param req The request to submit.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_aead_submit( struct pc302crypt_req *req )
{
    struct crypto_tfm *tfm = req->req->tfm;
    struct pc302crypt_aead_ctx *ctx = crypto_tfm_ctx( tfm );
    struct aead_request *aead_req =
        container_of( req->req, struct aead_request, base );
    struct crypto_alg *alg = req->req->tfm->__crt_alg;
    struct pc302crypt_alg *pc302_alg = to_pc302crypt_alg( alg );
    struct pc302crypt_engine *engine = ctx->generic.engine;
    unsigned proc_len;
    unsigned assoc_len;
    u32 ctrl;

    req->result = -EINPROGRESS;
    req->ctx_id = pc302crypt_load_ctx( &ctx->generic,
                                       aead_req->iv,
                                       alg->cra_aead.ivsize,
                                       CTX_COMBINED );

    pc302crypt_aead_make_ddts( req, req->giv, req->seq );

    req->src_addr = engine->ddt_buf_phys +
        ( req->ctx_id * sizeof( struct pc302crypt_engine_ctx ) ) +
        offsetof( struct pc302crypt_engine_ctx, src_ddt );
    req->dst_addr = engine->ddt_buf_phys +
        ( req->ctx_id * sizeof( struct pc302crypt_engine_ctx ) ) +
        offsetof( struct pc302crypt_engine_ctx, dst_ddt );

    /* Set the source and destination DDT pointers. */
    pc302crypt_reg_write( engine, SPA_SRC_PTR_REG_OFFSET,
                          ( u32 )req->src_addr );
    pc302crypt_reg_write( engine, SPA_DST_PTR_REG_OFFSET,
                          ( u32 )req->dst_addr );
    pc302crypt_reg_write( engine, SPA_OFFSET_REG_OFFSET, 0 );

    assoc_len = aead_req->assoclen;
    proc_len = aead_req->cryptlen + assoc_len;

    /* If we aren't generating an IV, then we need to include the IV in the
     * associated data so that it is included in the hash. */
    if ( !req->giv )
    {
        assoc_len += crypto_aead_ivsize( crypto_aead_reqtfm( aead_req ) );
        proc_len += crypto_aead_ivsize( crypto_aead_reqtfm( aead_req ) );
    }
    else
        proc_len += req->giv_len;

    /* If we are decrypting, we need to take the length of the ICV out of the
     * processing length. */
    if ( req->dir == DECRYPT )
        proc_len -= ctx->auth_size;
    
    /* Set the total processing length. */
    pc302crypt_reg_write( engine, SPA_PROC_LEN_REG_OFFSET, proc_len );
    pc302crypt_reg_write( engine, SPA_AAD_LEN_REG_OFFSET, assoc_len );
    pc302crypt_reg_write( engine, SPA_ICV_LEN_REG_OFFSET, ctx->auth_size );
    pc302crypt_reg_write( engine, SPA_ICV_OFFSET_REG_OFFSET, 0 );
    pc302crypt_reg_write( engine, SPA_AUX_INFO_REG_OFFSET, 0 );

    /* Setup the control register. */
    ctrl = 0;

    switch ( ( pc302_alg->type & PC302_CRYPTO_BLOCK_MASK ) | PC302_CRYPTO_BLOCK )
    {
        case PC302_CRYPTO_ALG_DES:
        case PC302_CRYPTO_ALG_3DES:
            ctrl |= SPA_CTRL_CIPH_ALG_DES;
            break;
        case PC302_CRYPTO_ALG_AES:
            ctrl |= SPA_CTRL_CIPH_ALG_AES;
            break;
        case PC302_CRYPTO_ALG_KASUMI:
            ctrl |= SPA_CTRL_CIPH_ALG_KASUMI;
            break;
        default:
            BUG();
    }

    switch ( ( pc302_alg->type & PC302_CRYPTO_HASH_MASK ) | PC302_CRYPTO_HASH )
    {
        case PC302_CRYPTO_ALG_SHA1:
            ctrl |= SPA_CTRL_HASH_ALG_SHA;
            ctrl |= SPA_CTRL_HASH_MODE_HMAC;
            break;
        case PC302_CRYPTO_ALG_SHA256:
            ctrl |= SPA_CTRL_HASH_ALG_SHA256;
            ctrl |= SPA_CTRL_HASH_MODE_HMAC;
            break;
        case PC302_CRYPTO_ALG_MD5:
            ctrl |= SPA_CTRL_HASH_ALG_MD5;
            ctrl |= SPA_CTRL_HASH_MODE_HMAC;
            break;
        default:
            BUG();
    }

    switch ( pc302_alg->type & PC302_CRYPTO_ABLK_MODE_MASK )
    {
        case PC302_CRYPTO_ABLK_MODE_CBC:
            ctrl |= SPA_CTRL_CIPH_MODE_CBC;
            break;
        case PC302_CRYPTO_ABLK_MODE_ECB:
            ctrl |= SPA_CTRL_CIPH_MODE_ECB;
            break;
        case PC302_CRYPTO_ABLK_MODE_F8:
            ctrl |= SPA_CTRL_CIPH_MODE_F8;
            break;
        default:
            BUG();
    }

    /* Set the context page. */
    ctrl |= ( ( req->ctx_id  << SPA_CTRL_CTX_IDX ) |
              ( 1 << SPA_CTRL_ICV_APPEND ) |
              ( ( ENCRYPT == req->dir ) ? ( 1 << SPA_CTRL_ENCRYPT_IDX ) : 0 ) |
              ( ( ENCRYPT == req->dir ) ? ( 1 << SPA_CTRL_AAD_COPY ) : 0 ) );
    if ( DECRYPT == req->dir )
        ctrl |= ( 1 << SPA_CTRL_KEY_EXP );

    /* Make sure all writes are finished before we start the operation. */
    wmb();

    /* If we are not over the IRQ threshold, then schedule a timeout. If we
     * are over the threshold then we can simply delete the timer as we'll get
     * an interrupt and reenable the timer when we pass below the threshold. */
    engine->in_flight++;
    if ( engine->in_flight == 1 )
    {
        PRINTD( DBG_TRACE, "schedule timeout" );
        mod_timer( &engine->packet_timeout, jiffies + PACKET_TIMEOUT );
    }
    else if ( engine->in_flight == engine->stat_irq_threshold )
    {
        PRINTD( DBG_TRACE, "over threshold, disable timer" );
        del_timer( &engine->packet_timeout );
    }

    pc302crypt_reg_write( engine, SPA_CTRL_REG_OFFSET, ctrl );

    return -EINPROGRESS;
}

/*!
 * Setup an AEAD request for processing. This will configure the engine, load
 * the context and then start the packet processing.
 *
 * @param req The request to process.
 * @param giv Pointer to destination address for a generated IV. If the
 * request does not need to generate an IV then this should be set to NULL.
 * @param seq Sequence number for the request. Only valid if giv is not NULL.
 * @param alg_type The algorithm type.
 * @param dir The direction of the request. Must be ENCRYPT or DECRYPT.
 * @return Returns -EINPROGRESS if the packet is queued for processing.
 */
static int
pc302crypt_aead_setup( struct aead_request *req,
                       u8 *giv,
                       u64 seq,
                       unsigned alg_type,
                       enum pc302crypt_direction dir )
{
    struct crypto_alg *alg = req->base.tfm->__crt_alg;
    struct pc302crypt_engine *engine = to_pc302crypt_alg( alg )->engine;
    struct pc302crypt_req *dev_req = aead_request_ctx( req );
    unsigned long flags;
    int err = -EINPROGRESS;
    unsigned ivsize = crypto_aead_ivsize( crypto_aead_reqtfm( req ) );

    if ( !dev_req )
        return -ENOMEM;

    dev_req->giv = giv;
    dev_req->giv_len = ivsize;
    dev_req->seq = seq;

    dev_req->req = &req->base;
    dev_req->dir = dir;

    dev_req->result = -EBUSY;

    if ( unlikely( pc302crypt_aead_need_fallback( dev_req ) ) )
    {
        err = pc302crypt_aead_do_fallback( req, alg_type, dir );
        pc302crypt_aead_kill_ddts( dev_req );
        goto out;
    }

    spin_lock_irqsave( &engine->hw_lock, flags );
    err = pc302crypt_queue_req( &engine->pending, dev_req );

    /* If we were unable to put the request in because the transform can't
     * backlog, then return early. */
    if ( -EBUSY == err && !( req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG ) )
    {
        spin_unlock_irqrestore( &engine->hw_lock, flags );
        goto out;
    }
    else if ( -EINPROGRESS == err )
    {
        err = pc302crypt_aead_submit( dev_req );
    }
    else
    {
        /* We have added the request to the queue so it is pending and
         * effectively in progress. */
        err = -EINPROGRESS;
    }

    spin_unlock_irqrestore( &engine->hw_lock, flags );

out:
    return err;
}

/*!
 * Perform an encrypt request for an AEAD cipher.
 *
 * @param req The request to encrypt.
 * @return Returns the success/failure status of the operation.
 */
static int
pc302crypt_aead_encrypt( struct aead_request *req )
{
    struct crypto_aead *aead = crypto_aead_reqtfm( req );
    struct crypto_tfm *tfm = crypto_aead_tfm( aead );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( tfm->__crt_alg );

    return pc302crypt_aead_setup( req, NULL, 0, alg->type, ENCRYPT );
}

/*!
 * Perform a givencrypt request for an AEAD cipher.
 *
 * @param req The request to encrypt.
 * @return Returns the success/failure status of the operation.
 */
static int
pc302crypt_aead_givencrypt( struct aead_givcrypt_request *req )
{
    struct crypto_aead *tfm = aead_givcrypt_reqtfm( req );
    struct pc302crypt_aead_ctx *ctx = crypto_aead_ctx( tfm );
    size_t ivsize = crypto_aead_ivsize( tfm );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( tfm->base.__crt_alg );
    unsigned len;
    __be64 seq;

    memcpy( req->areq.iv, ctx->salt, ivsize );
    len = ivsize;
    if ( ivsize > sizeof( u64 ) )
    {
        memset( req->giv, 0, ivsize - sizeof( u64 ) );
        len = sizeof( u64 );
    }
    seq = cpu_to_be64( req->seq );
    memcpy( req->giv + ivsize - len, &seq, len );

    return pc302crypt_aead_setup( &req->areq, req->giv, ivsize, alg->type,
                                  ENCRYPT );
}

/*!
 * Perform a decrypt request for an AEAD cipher.
 *
 * @param req The request to decrypt.
 * @return Returns the success/failure status of the operation.
 */
static int
pc302crypt_aead_decrypt( struct aead_request *req )
{
    struct crypto_aead *aead = crypto_aead_reqtfm( req );
    struct crypto_tfm *tfm = crypto_aead_tfm( aead );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( tfm->__crt_alg );

    return pc302crypt_aead_setup( req, NULL, 0, alg->type, DECRYPT );
}

/*!
 * Initialise a new AEAD context. This is responsible for allocating the
 * fallback cipher and initialising the context.
 *
 * @param tfm The transform to create the context for.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_aead_cra_init( struct crypto_tfm *tfm )
{
    struct pc302crypt_aead_ctx *ctx = crypto_tfm_ctx( tfm );
    struct crypto_alg *alg = tfm->__crt_alg;
    struct pc302crypt_alg *pc302_alg = to_pc302crypt_alg( alg );
    struct pc302crypt_engine *engine = pc302_alg->engine;

    ctx->generic.flags = pc302_alg->type;
    ctx->generic.engine = engine;
    ctx->sw_cipher =
        crypto_alloc_aead( alg->cra_name, 0,
                           CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK );
    if ( IS_ERR( ctx->sw_cipher ) )
    {
        PRINTD( DBG_WARN, "failed to allocate fallback for %s",
                alg->cra_name );
        ctx->sw_cipher = NULL;
    }

    get_random_bytes( ctx->salt, sizeof( ctx->salt ) );

    tfm->crt_aead.reqsize = sizeof( struct pc302crypt_req );

    return 0;
}

/*!
 * Destructor for an AEAD context. This is called when the transform is freed
 * and must free the fallback cipher.
 *
 * @param tfm The transform being freed.
 */
static void
pc302crypt_aead_cra_exit( struct crypto_tfm *tfm )
{
    struct pc302crypt_aead_ctx *ctx = crypto_tfm_ctx(tfm);

    if ( ctx->sw_cipher )
        crypto_free_aead( ctx->sw_cipher );
    ctx->sw_cipher = NULL;
}

/*!
 * Set the DES key for a block cipher transform. This also performs weak key
 * checking if the transform has requested it.
 *
 * @param cipher The transform to set the key of.
 * @param key The key to set.
 * @param len The length of the key in bytes.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_des_setkey( struct crypto_ablkcipher *cipher,
                       const u8 *key,
                       unsigned int len )
{
    struct crypto_tfm *tfm = crypto_ablkcipher_tfm( cipher );
    struct pc302crypt_ablk_ctx *ctx = crypto_tfm_ctx( tfm );
    int err = 0;
    u32 tmp[ DES_EXPKEY_WORDS ];

    err = des_ekey( tmp, key );
    if ( unlikely( !err ) &&
            ( crypto_ablkcipher_get_flags( cipher ) &
              CRYPTO_TFM_REQ_WEAK_KEY ) )
    {
        tfm->crt_flags |= CRYPTO_TFM_RES_WEAK_KEY;
        return -EINVAL;
    }
    err = 0;

    memcpy( ctx->key, key, len );
    ctx->key_len = len;

    return err;
}

/*!
 * Set the key for an AES block cipher. Some key lengths are not supported in
 * hardware so this must also check whether a fallback is needed.
 *
 * @param cipher The transform to set the key for.
 * @param key The key to set.
 * @param len The length of the key in bytes.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_aes_setkey( struct crypto_ablkcipher *cipher,
                       const u8 *key,
                       unsigned int len )
{
    struct crypto_tfm *tfm = crypto_ablkcipher_tfm( cipher );
    struct pc302crypt_ablk_ctx *ctx = crypto_tfm_ctx( tfm );
    int err = 0;

    /* IPSec engine only supports 128 and 256 bit AES keys. If we get a
     * request for any other size (192 bits) then we need to do a software
     * fallback. */
    if ( !( 16 == len || 32 == len ) && ctx->sw_cipher )
    {
        /* Set the fallback transform to use the same request flags as the
         * hardware transform. */
        ctx->sw_cipher->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
        ctx->sw_cipher->base.crt_flags |=
            ( cipher->base.crt_flags & CRYPTO_TFM_REQ_MASK );

        err = crypto_ablkcipher_setkey( ctx->sw_cipher, key, len );
        if ( err )
            goto sw_setkey_failed;
    }
    else if ( !( 16 == len || 32 == len ) && !ctx->sw_cipher )
        err = -EINVAL;

    memcpy( ctx->key, key, len );
    ctx->key_len = len;

sw_setkey_failed:
    return err;
}

/*!
 * Set the key for a Kasumi F8 block cipher.
 *
 * @param cipher The transform to set the key for.
 * @param key The key to set.
 * @param len The length of the key in bytes.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_kasumi_f8_setkey( struct crypto_ablkcipher *cipher,
                             const u8 *key,
                             unsigned int len )
{
    struct crypto_tfm *tfm = crypto_ablkcipher_tfm( cipher );
    struct pc302crypt_ablk_ctx *ctx = crypto_tfm_ctx( tfm );

    memcpy( ctx->key, key, len );
    ctx->key_len = len;

    return 0;
}

/*!
 * Set the key for a block cipher transform.
 *
 * @param cipher The transform to set the key for.
 * @param key The key to set.
 * @param len The length of the key in bytes.
 * @return Returns zero on success, negative on failure.
 */
static int
pc302crypt_ablk_setkey( struct crypto_ablkcipher *cipher,
                        const u8 *key,
                        unsigned int len )
{
    struct crypto_tfm *tfm = crypto_ablkcipher_tfm( cipher );
    struct pc302crypt_ablk_ctx *ctx = crypto_tfm_ctx( tfm );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( tfm->__crt_alg );
    int err = 0;

    if ( len > PC302_CRYPTO_AES_MAX_KEY_LEN )
    {
        crypto_ablkcipher_set_flags( cipher, CRYPTO_TFM_RES_BAD_KEY_LEN );
        err = -EINVAL;
        goto bad_key_len;
    }

    switch ( ( alg->type & PC302_CRYPTO_BLOCK_MASK ) | PC302_CRYPTO_BLOCK )
    {
        case PC302_CRYPTO_ALG_AES:
            err = pc302crypt_aes_setkey( cipher, key, len );
            break;

        case PC302_CRYPTO_ALG_3DES:
        case PC302_CRYPTO_ALG_DES:
            err = pc302crypt_des_setkey( cipher, key, len );
            break;

        case PC302_CRYPTO_ALG_KASUMI:
            err = pc302crypt_kasumi_f8_setkey( cipher, key, len );
            break;

        default:
            BUG();
            break;
    }

    if ( err )
        goto out;

    if ( ctx->sw_cipher )
    {
        /* Set the fallback transform to use the same request flags as the
         * hardware transform. */
        ctx->sw_cipher->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
        ctx->sw_cipher->base.crt_flags |=
            ( cipher->base.crt_flags & CRYPTO_TFM_REQ_MASK );

        err = crypto_ablkcipher_setkey( ctx->sw_cipher, key, len );
        if ( err )
            goto sw_setkey_failed;
    }

    return err;

bad_key_len:
    PRINTD( DBG_WARN, "bad key length (%u)", len );
    return err;

sw_setkey_failed:
    PRINTD( DBG_WARN, "failed to set key for fallback cipher (%d)", err );
    tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
    tfm->crt_flags |= ( ctx->sw_cipher->base.crt_flags & CRYPTO_TFM_RES_MASK );
out:
    return err;
}

static int
pc302crypt_ablk_need_fallback( struct pc302crypt_req *req )
{
    struct ablkcipher_request *ablk_req;
    struct pc302crypt_ablk_ctx *ctx;
    struct crypto_tfm *tfm = req->req->tfm;
    struct crypto_alg *alg = req->req->tfm->__crt_alg;
    struct pc302crypt_alg *pc302_alg = to_pc302crypt_alg( alg );

    ctx = crypto_tfm_ctx( tfm );

    ablk_req = ablkcipher_request_cast( req->req );
    /* If we have a non-supported key-length, then we need to do a software
     * fallback. */
    if ( ( ( pc302_alg->type & PC302_CRYPTO_ALG_MASK ) |
                PC302_CRYPTO_BLOCK ) == PC302_CRYPTO_ALG_AES &&
            ( &ipsec_engine == ctx->generic.engine ) &&
            !( 16 == ctx->key_len || 32 == ctx->key_len ) )
    {
        return 1;
    }

    return 0;
}

static int
pc302crypt_ablk_submit( struct pc302crypt_req *req )
{
    struct crypto_tfm *tfm = req->req->tfm;
    struct pc302crypt_ablk_ctx *ctx = crypto_tfm_ctx( tfm );
    struct ablkcipher_request *ablk_req = ablkcipher_request_cast( req->req );
    struct crypto_alg *alg = req->req->tfm->__crt_alg;
    struct pc302crypt_alg *pc302_alg = to_pc302crypt_alg( alg );
    struct pc302crypt_engine *engine = ctx->generic.engine;
    u32 ctrl;

    req->result = -EINPROGRESS;
    req->ctx_id = pc302crypt_load_ctx( &ctx->generic, ablk_req->info,
                                       alg->cra_ablkcipher.ivsize,
                                       CTX_CIPHER );

    if (ablk_req->src)
    {
#ifdef PROFILE_PC302CRYPTO_USING_HIRES_TIMER
    profileTestStart = pc302crypt_get_cycles();
#endif

        if ( !pc302crypt_sg_to_ddt( req, ablk_req->src, ablk_req->nbytes,
                                DMA_TO_DEVICE ) )
            return -ENOMEM;
    }
                
    if ( !pc302crypt_sg_to_ddt( req, ablk_req->dst, ablk_req->nbytes,
                                DMA_FROM_DEVICE ) )
        return -ENOMEM;

    if (ablk_req->src)
    {
        req->src_addr = engine->ddt_buf_phys +
            ( req->ctx_id * sizeof( struct pc302crypt_engine_ctx ) ) +
            offsetof( struct pc302crypt_engine_ctx, src_ddt );
#ifdef PROFILE_PC302CRYPTO_USING_HIRES_TIMER
    profileTest = pc302crypt_get_cycles() - profileTestStart;
#endif
    }
    else
    {
        req->src_addr = engine->zeroed_src_addr;
    }
    
    req->dst_addr = engine->ddt_buf_phys +
        ( req->ctx_id * sizeof( struct pc302crypt_engine_ctx ) ) +
        offsetof( struct pc302crypt_engine_ctx, dst_ddt );

    /* Set the source and destination DDT pointers. */
    pc302crypt_reg_write( engine, SPA_SRC_PTR_REG_OFFSET,
                          ( u32 )req->src_addr );
    pc302crypt_reg_write( engine, SPA_DST_PTR_REG_OFFSET,
                          ( u32 )req->dst_addr );
    pc302crypt_reg_write( engine, SPA_OFFSET_REG_OFFSET, 0 );

    /* Set the total processing length. */
    pc302crypt_reg_write( engine, SPA_PROC_LEN_REG_OFFSET,
                          ablk_req->nbytes );
    pc302crypt_reg_write( engine, SPA_ICV_OFFSET_REG_OFFSET, 0 );
    pc302crypt_reg_write( engine, SPA_AUX_INFO_REG_OFFSET, 0 );
    pc302crypt_reg_write( engine, SPA_AAD_LEN_REG_OFFSET, 0 );

    /* Setup the control register. */
    ctrl = 0;

    switch ( ( pc302_alg->type & PC302_CRYPTO_BLOCK_MASK ) | PC302_CRYPTO_BLOCK )
    {
        case PC302_CRYPTO_ALG_DES:
        case PC302_CRYPTO_ALG_3DES:
            ctrl |= SPA_CTRL_CIPH_ALG_DES;
            break;
        case PC302_CRYPTO_ALG_AES:
            ctrl |= SPA_CTRL_CIPH_ALG_AES;
            break;
        case PC302_CRYPTO_ALG_KASUMI:
            ctrl |= SPA_CTRL_CIPH_ALG_KASUMI;
            break;
        default:
            BUG();
    }

    switch ( pc302_alg->type & PC302_CRYPTO_ABLK_MODE_MASK )
    {
        case PC302_CRYPTO_ABLK_MODE_CBC:
            ctrl |= SPA_CTRL_CIPH_MODE_CBC;
            break;
        case PC302_CRYPTO_ABLK_MODE_ECB:
            ctrl |= SPA_CTRL_CIPH_MODE_ECB;
            break;
        case PC302_CRYPTO_ABLK_MODE_F8:
            ctrl |= SPA_CTRL_CIPH_MODE_F8;
            break;
        default:
            BUG();
    }

    /* Set the context page. */
    ctrl |= ( req->ctx_id ) << SPA_CTRL_CTX_IDX;
    ctrl |= ( ENCRYPT == req->dir ) ? ( 1 << SPA_CTRL_ENCRYPT_IDX ) : 0;
    if ( DECRYPT == req->dir )
        ctrl |= ( 1 << SPA_CTRL_KEY_EXP );

    /* Make sure all writes are finished before we start the operation. */
    wmb();

    /* If we are not over the IRQ threshold, then schedule a timeout. If we
     * are over the threshold then we can simply delete the timer as we'll get
     * an interrupt and reenable the timer when we pass below the threshold. */
    engine->in_flight++;
    if ( engine->in_flight == 1 )
    {
        PRINTD( DBG_TRACE, "schedule timeout" );
        mod_timer( &engine->packet_timeout, jiffies + PACKET_TIMEOUT );
    }
    else if ( engine->in_flight == engine->stat_irq_threshold )
    {
        PRINTD( DBG_TRACE, "over threshold, disable timer" );
        del_timer( &engine->packet_timeout );
    }

    pc302crypt_reg_write( engine, SPA_CTRL_REG_OFFSET, ctrl );

    return -EINPROGRESS;
}

static int
pc302crypt_ablk_do_fallback( struct ablkcipher_request *req,
                             unsigned alg_type,
                             enum pc302crypt_direction dir )
{
    struct crypto_tfm *old_tfm =
        crypto_ablkcipher_tfm( crypto_ablkcipher_reqtfm( req ) );
    struct pc302crypt_ablk_ctx *ctx = crypto_tfm_ctx( old_tfm );
    int err;

    /* Change the request to use the software fallback transform, and once the
     * ciphering has completed, put the old transform back into the request.
     */
    if ( ctx->sw_cipher )
    {
        ablkcipher_request_set_tfm( req, ctx->sw_cipher );
        err = ( ENCRYPT == dir ) ? crypto_ablkcipher_encrypt( req ) :
            crypto_ablkcipher_decrypt( req );
        ablkcipher_request_set_tfm( req, __crypto_ablkcipher_cast( old_tfm ) );
    }
    else
        err = -EINVAL;

    return err;
}

static int
pc302crypt_ablk_setup( struct ablkcipher_request *req,
                       unsigned alg_type,
                       enum pc302crypt_direction dir )
{
    struct crypto_alg *alg = req->base.tfm->__crt_alg;
    struct pc302crypt_engine *engine = to_pc302crypt_alg( alg )->engine;
    struct pc302crypt_req *dev_req = ablkcipher_request_ctx( req );
    unsigned long flags;
    int err = -EINPROGRESS;

#ifdef PROFILE_PC302CRYPTO_USING_HIRES_TIMER
    profileTotal=0;
    profileTest=0;
    profileTestStart=0;
#endif

    if ( !dev_req )
        return -ENOMEM;

    dev_req->req = &req->base;
    dev_req->result = -EBUSY;
    dev_req->dir = dir;

    if ( pc302crypt_ablk_need_fallback( dev_req ) )
    {
        err = pc302crypt_ablk_do_fallback( req, alg_type, dir );
        pc302crypt_free_ddt( dev_req, req->src,
                             req->nbytes, DMA_TO_DEVICE );
        pc302crypt_free_ddt( dev_req, req->dst,
                             req->nbytes, DMA_FROM_DEVICE );
        goto out;
    }

    spin_lock_irqsave( &engine->hw_lock, flags );
#ifdef PROFILE_PC302CRYPTO_USING_HIRES_TIMER
    profileTotalStart = pc302crypt_get_cycles();
#endif
    err = pc302crypt_queue_req( &engine->pending, dev_req );

    /* If we were unable to put the request in because the transform can't
     * backlog, then return early. */
    if ( -EBUSY == err && !( req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG ) )
    {
        spin_unlock_irqrestore( &engine->hw_lock, flags );
        goto out;
    }
    else if ( -EINPROGRESS == err )
    {
        err = pc302crypt_ablk_submit( dev_req );
    }
    else
    {
        /* We have added the request to the queue so it is pending and
         * effectively in progress. */
        err = -EINPROGRESS;
    }

#ifdef PROFILE_PC302CRYPTO_USING_HIRES_TIMER
    profileTotal = pc302crypt_get_cycles() - profileTotalStart;
#endif

    spin_unlock_irqrestore( &engine->hw_lock, flags );
    
#ifdef PROFILE_PC302CRYPTO_USING_HIRES_TIMER
    if (profileTest)
    {
        profileN++;
        profileTotalSum += profileTotal;
//        profileTotalSumSquares += profileTotal*profileTotal;
        profileTestSum += profileTest;
//        profileTestSumSquares += profileTest*profileTest;
        if (profileTestStart)
        {
            profileDelaySum += profileTestStart-profileTotalStart;
        }
    }
    
    if (profileN == 100000)
    {
        printk("N=%u S=%u S2=%u ST=%u ST2=%u DLY=%u\n",
                profileN, profileTotalSum, profileTotalSumSquares, profileTestSum, profileTestSumSquares, profileDelaySum);
        profileN = 0;
        profileTotalSum = 0;
        profileTotalSumSquares = 0;
        profileTestSum = 0;
        profileTestSumSquares = 0;
        profileDelaySum = 0;
    }
#endif
out:
    return err;
}

static int
pc302crypt_ablk_cra_init( struct crypto_tfm *tfm )
{
    struct pc302crypt_ablk_ctx *ctx = crypto_tfm_ctx( tfm );
    struct crypto_alg *alg = tfm->__crt_alg;
    struct pc302crypt_alg *pc302_alg = to_pc302crypt_alg( alg );
    struct pc302crypt_engine *engine = pc302_alg->engine;

    ctx->generic.flags = pc302_alg->type;
    ctx->generic.engine = engine;
    if ( alg->cra_flags & CRYPTO_ALG_NEED_FALLBACK )
    {
        ctx->sw_cipher =
            crypto_alloc_ablkcipher( alg->cra_name, 0,
                                     CRYPTO_ALG_ASYNC |
                                     CRYPTO_ALG_NEED_FALLBACK );
        if ( IS_ERR( ctx->sw_cipher ) )
        {
            PRINTD( DBG_WARN, "failed to allocate fallback for %s",
                    alg->cra_name );
            ctx->sw_cipher = NULL;
        }
    }

    tfm->crt_ablkcipher.reqsize = sizeof( struct pc302crypt_req );

    return 0;
}

static void
pc302crypt_ablk_cra_exit( struct crypto_tfm *tfm )
{
    struct pc302crypt_ablk_ctx *ctx = crypto_tfm_ctx(tfm);

    if ( ctx->sw_cipher )
        crypto_free_ablkcipher( ctx->sw_cipher );
    ctx->sw_cipher = NULL;
}

/*
 * Given a struct crypto_alg, find the pc302crypt_alg entry.
 *
 * \param alg The algorithm search for.
 * \return Returns the pc302 representation on success, NULL on failure.
 */
static inline struct pc302crypt_alg *
to_pc302crypt_alg( struct crypto_alg *alg )
{
    return alg ? container_of( alg, struct pc302crypt_alg, alg ) : NULL;
}

static int
pc302crypt_ablk_encrypt( struct ablkcipher_request *req )
{
    struct crypto_ablkcipher *cipher = crypto_ablkcipher_reqtfm( req );
    struct crypto_tfm *tfm = crypto_ablkcipher_tfm( cipher );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( tfm->__crt_alg );

    return pc302crypt_ablk_setup( req, alg->type, ENCRYPT );
}

static int
pc302crypt_ablk_decrypt( struct ablkcipher_request *req )
{
    struct crypto_ablkcipher *cipher = crypto_ablkcipher_reqtfm( req );
    struct crypto_tfm *tfm = crypto_ablkcipher_tfm( cipher );
    struct pc302crypt_alg *alg = to_pc302crypt_alg( tfm->__crt_alg );

    return pc302crypt_ablk_setup( req, alg->type, DECRYPT );
}

static int
pc302crypt_fifo_stat_empty( struct pc302crypt_engine *engine )
{
    u32 fifo_stat = pc302crypt_reg_read( engine, SPA_FIFO_STAT_REG_OFFSET );

    return !!( fifo_stat & SPA_FIFO_STAT_EMPTY );
}

static int
pc302crypt_fifo_cmd_full( struct pc302crypt_engine *engine )
{
    u32 fifo_stat = pc302crypt_reg_read( engine, SPA_FIFO_STAT_REG_OFFSET );

    return !!( fifo_stat & SPA_FIFO_CMD_FULL );
}

static void
pc302crypt_process_done( struct pc302crypt_engine *engine )
{
    struct pc302crypt_req *req;
    struct pc302crypt_ablk_ctx *ablk_ctx;
    struct pc302crypt_aead_ctx *aead_ctx;
    struct pc302crypt_generic_ctx *generic_ctx = NULL;
    int err;
    unsigned long flags;

    spin_lock_irqsave( &engine->hw_lock, flags );

    while ( !pc302crypt_fifo_stat_empty( engine ) )
    {
        req = pc302crypt_dequeue_req( &engine->pending );
        BUG_ON( !req );

        err = pc302crypt_queue_req_force( &engine->completed, req );

        /* POP the status register. */
        pc302crypt_reg_write( engine, SPA_STAT_POP_REG_OFFSET, ~0 );
        wmb();

        /* Read the status of the operation. */
        req->result = ( pc302crypt_reg_read( engine, SPA_STATUS_REG_OFFSET ) &
            SPA_STATUS_RES_CODE_MASK ) >> SPA_STATUS_RES_CODE_OFFSET;

        /* Convert the SPAcc error status into the standard POSIX error
         * codes. */
        switch ( req->result )
        {
            case SPA_STATUS_OK:
                req->result = 0;
                break;

            case SPA_STATUS_ICV_FAIL:
                req->result = -EBADMSG;
                break;

            case SPA_STATUS_MEMORY_ERROR:
                PRINTD( DBG_WARN, "memory error triggered" );
                req->result = -EFAULT;
                break;

            case SPA_STATUS_BLOCK_ERROR:
                PRINTD( DBG_WARN, "block error triggered" );
                req->result = -EIO;
                break;

            default:
                BUG();
        }

        switch ( crypto_tfm_alg_type( req->req->tfm ) )
        {
            case CRYPTO_ALG_TYPE_BLKCIPHER:
            case CRYPTO_ALG_TYPE_ABLKCIPHER:
            case CRYPTO_ALG_TYPE_GIVCIPHER:
                ablk_ctx = crypto_tfm_ctx( req->req->tfm );
                generic_ctx = &ablk_ctx->generic;
                break;

            case CRYPTO_ALG_TYPE_AEAD:
                aead_ctx = crypto_tfm_ctx( req->req->tfm );
                generic_ctx = &aead_ctx->generic;
                break;

            default:
                WARN( 1, "invalid transform type completed (%d)\n",
                      crypto_tfm_alg_type( req->req->tfm ) );
                break;
        }

        if ( likely( generic_ctx ) )
            pc302crypt_unload_ctx( generic_ctx, req->ctx_id );
    }

    spin_unlock_irqrestore( &engine->hw_lock, flags );

    tasklet_schedule( &engine->complete );
}

static irqreturn_t
pc302crypt_spacc_irq( int irq,
                      void *dev )
{
    struct pc302crypt_engine *engine = ( struct pc302crypt_engine * )dev;
    u32 irq_stat = pc302crypt_reg_read( engine, SPA_IRQ_STAT_REG_OFFSET );

    /* Clear the interrupts. */
    pc302crypt_reg_write( engine, SPA_IRQ_STAT_REG_OFFSET, irq_stat );

    /* Process the completed packets. */
    pc302crypt_process_done( engine );

    return IRQ_HANDLED;
}

static void
pc302crypt_packet_timeout( unsigned long data )
{
    struct pc302crypt_engine *engine = ( struct pc302crypt_engine * )data;

    PRINTD( DBG_TRACE, "packet timeout, %u in flight", engine->in_flight );
    pc302crypt_process_done( engine );
}

static int
pc302crypt_req_submit( struct pc302crypt_req *req )
{
    struct crypto_alg *alg = req->req->tfm->__crt_alg;

    if ( CRYPTO_ALG_TYPE_AEAD & alg->cra_flags )
        return pc302crypt_aead_submit( req );
    else if ( CRYPTO_ALG_TYPE_ABLKCIPHER & alg->cra_flags ||
              CRYPTO_ALG_TYPE_GIVCIPHER & alg->cra_flags )
        return pc302crypt_ablk_submit( req );

    BUG();
}

static void
pc302crypt_spacc_complete( unsigned long data )
{
    struct pc302crypt_engine *engine = ( struct pc302crypt_engine * )data;
    struct pc302crypt_req *req;
    struct ablkcipher_request *ablk_req;
    struct aead_request *aead_req;
    struct pc302crypt_alg *alg;
    struct list_head *l;
    unsigned long flags;

    for (;;)
    {
        spin_lock_irqsave( &engine->hw_lock, flags );
        req = pc302crypt_dequeue_req( &engine->completed );
        spin_unlock_irqrestore( &engine->hw_lock, flags );

        if ( !req )
            break;

        alg = to_pc302crypt_alg( req->req->tfm->__crt_alg );
        switch ( alg->type & PC302_CRYPTO_TYPE_MASK )
        {
            case PC302_CRYPTO_BLOCK:
                ablk_req =
                    container_of( req->req, struct ablkcipher_request, base );

                pc302crypt_free_ddt( req, ablk_req->src,
                                     ablk_req->nbytes, DMA_TO_DEVICE );
                pc302crypt_free_ddt( req, ablk_req->dst,
                                     ablk_req->nbytes, DMA_FROM_DEVICE );
                break;

            case PC302_CRYPTO_COMBINED:
                aead_req =
                    container_of( req->req, struct aead_request, base );

                pc302crypt_aead_kill_ddts( req );
                break;

            default:
                BUG();
        }

        if ( req->req->complete )
            req->req->complete( req->req, req->result );

        --engine->in_flight;

        /* If we have no remaining packets then we can stop the timer. */
        if ( !engine->in_flight )
        {
            PRINTD( DBG_TRACE, "no packets remaining. del timer" );
            del_timer( &engine->packet_timeout );
        }
    }

    /*
     * Need to iterate over the pending queue starting from the HEAD and skip
     * over the requests that have the result field set to -EINPROGRESS and
     * then submit num_completed requests to the hardware.
     */
    spin_lock_irqsave( &engine->hw_lock, flags );
    list_for_each( l, &engine->pending.list )
    {
        if ( pc302crypt_fifo_cmd_full( engine ) )
            break;

        req = container_of( l, struct pc302crypt_req, list );

        if ( -EINPROGRESS == req->result )
            continue;
        else if ( -EBUSY == req->result )
            req->result = pc302crypt_req_submit( req );
    }

    /* If we have less packets in flight than the threshold, kick the
     * timer. */
    if ( engine->in_flight && engine->in_flight < engine->stat_irq_threshold )
    {
        PRINTD( DBG_TRACE, "below threshold, kick timer" );
        mod_timer( &engine->packet_timeout, jiffies + PACKET_TIMEOUT );
    }

    spin_unlock_irqrestore( &engine->hw_lock, flags );
}

static void
pc302crypt_release_and_unmap( struct resource *resource,
                              void __iomem *iomem )
{
    unsigned long size = resource->end - resource->start + 1;
    iounmap( iomem );
    release_mem_region( resource->start, size );
}

static void __iomem *
pc302crypt_request_and_map( struct resource *resource,
                            const char *name )
{
    unsigned long size = resource->end - resource->start + 1;
    void __iomem *ret;

    struct resource *req = request_mem_region( resource->start, size, name );

    if ( !req )
        goto req_fail;

    ret = ioremap( req->start, size );
    if ( !ret )
    {
        printk( KERN_INFO "pc302crypt: ioremap failed\n" );
        goto remap_failed;
    }

    return ret;

remap_failed:
    release_resource( req );

req_fail:

    return NULL;
}

static int
spacc_probe( struct platform_device *pdev )
{
    int err;
    int ret = -EINVAL;
    unsigned i;
    struct pc302crypt_alg *alg;
    struct pc302crypt_alg *next;
    struct resource *mem_resource =
        platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    struct resource *irq =
        platform_get_resource( pdev, IORESOURCE_IRQ, 0 );
    struct pc302crypt_engine *engine;

    if ( !strcmp( "ipsec_engine", pdev->name ) )
        engine = &ipsec_engine;
    else if ( !strcmp( "l2_engine", pdev->name ) )
        engine = &l2_engine;
    else
    {
        printk( KERN_INFO "invalid SPAcc engine (%s)\n", pdev->name );
        return -EINVAL;
    }

    if ( !mem_resource )
    {
        printk( KERN_INFO
                "pc302crypt: no memory resource for engine\n" );
        goto out;
    }

    engine->dev = &pdev->dev;

    engine->zeroed_page = ( void * )get_zeroed_page( GFP_KERNEL );
    if ( !engine->zeroed_page )
    {
        printk( KERN_INFO "failed to allocate a zeroed page (%s)\n", pdev->name );
        goto out;
    }

    engine->regs =
        pc302crypt_request_and_map( mem_resource, engine->name );
    if ( !engine->regs )
    {
        printk( KERN_INFO "pc302crypt: memory map failed\n" );
        goto map_fail;
    }
    engine->cipher_ctx_base =
        engine->regs + SPA_CIPH_KEY_BASE_REG_OFFSET;
    engine->hash_key_base =
        engine->regs + SPA_HASH_KEY_BASE_REG_OFFSET;

    ret = request_irq( irq->start, pc302crypt_spacc_irq, 0, engine->name,
                       engine );
    if ( ret )
    {
        PRINTD( DBG_ERROR, "failed to request IRQ\n" );
        goto irq_fail;
    }

    /* Register all of the algorithms that we support with the crypto
     * subsystem. */
    INIT_LIST_HEAD( &engine->registered_algs );
    for ( i = 0; i < engine->num_algs; ++i )
    {
        engine->algs[ i ].engine = engine;
        err = crypto_register_alg( &engine->algs[ i ].alg );
        if ( !err )
        {
            list_add_tail( &engine->algs[ i ].entry,
                           &engine->registered_algs );
            /* If we add at least one algorithm then we consider this as a
             * success. */
            ret = 0;
        }
        PRINTD( ( !err ? DBG_TRACE : DBG_WARN ), "%s alg \"%s\"",
                ( !err ) ? "registered" : "failed to register",
                engine->algs[ i ].alg.cra_name );
    }

    if ( !ret )
    {
        pc302crypt_queue_init( &engine->pending, engine->fifo_sz );
        pc302crypt_queue_init( &engine->completed, 1 );
        tasklet_init( &engine->complete, pc302crypt_spacc_complete,
                      ( unsigned long )engine );
    }

    /* Configure the interrupts. We only use the STAT_CNT interrupt as we only
     * submit a new packet for processing when we complete another in the
     * queue. This minimizes time spent in the interrupt handler. */
    pc302crypt_reg_write( engine, SPA_IRQ_CTRL_REG_OFFSET,
                          engine->stat_irq_threshold << SPA_IRQ_CTRL_STAT_CNT_OFFSET );
    pc302crypt_reg_write( engine, SPA_IRQ_EN_REG_OFFSET,
                          SPA_IRQ_EN_STAT_EN | SPA_IRQ_EN_GLBL_EN );

    setup_timer( &engine->packet_timeout, pc302crypt_packet_timeout,
                 ( unsigned long )engine );

    engine->ddt_buf = dma_alloc_coherent( engine->dev, (engine->max_ctxs + 1) *
            sizeof( struct pc302crypt_engine_ctx ), &engine->ddt_buf_phys,
            GFP_KERNEL );
    if ( !engine->ddt_buf )
    {
        ret = -ENOMEM;
        goto ddt_buf_failed;
    }
    
    memset(engine->ddt_buf, 0, (engine->max_ctxs + 1) * sizeof( struct pc302crypt_engine_ctx ));
    
    engine->ddt_info = kmalloc( (engine->max_ctxs + 1) * 
              sizeof( struct pc302crypt_ddt_info ), GFP_KERNEL );
    if ( !engine->ddt_info )
    {
        ret = -ENOMEM;
        goto ddt_info_failed;
    }
    
    memset(engine->ddt_info, 0, (engine->max_ctxs + 1) * sizeof( struct pc302crypt_ddt_info ));
    
    engine->free_ddt_info = engine->ddt_info;
    for ( i = 0; i < engine->max_ctxs - 1; ++i )
    {
        engine->ddt_info[i].next = &engine->ddt_info[i+1];
    }
    engine->ddt_info[engine->max_ctxs - 1].next = 0;

    sg_init_table( &engine->zeroed_list, 1 );
    sg_set_buf( &engine->zeroed_list, engine->zeroed_page, 4096 );
    dma_map_sg( engine->dev, &engine->zeroed_list, 1, DMA_TO_DEVICE );

    engine->ddt_buf[ engine->max_ctxs ].src_ddt[ 0 ].p   = sg_dma_address( &engine->zeroed_list );
    engine->ddt_buf[ engine->max_ctxs ].src_ddt[ 0 ].len = sg_dma_len( &engine->zeroed_list );
    engine->ddt_buf[ engine->max_ctxs ].src_ddt[ 1 ].p   = 0;
    engine->ddt_buf[ engine->max_ctxs ].src_ddt[ 1 ].len = 0;
    
    engine->zeroed_src_addr = engine->ddt_buf_phys + 
            ( engine->max_ctxs * sizeof( struct pc302crypt_engine_ctx ) ) +
            offsetof( struct pc302crypt_engine_ctx, src_ddt );
            
    return ret;

ddt_info_failed:
    dma_free_coherent( engine->dev, engine->max_ctxs *
            sizeof( struct pc302crypt_engine_ctx ), engine->ddt_buf,
            engine->ddt_buf_phys );

ddt_buf_failed:
    del_timer( &engine->packet_timeout );

    list_for_each_entry_safe( alg, next, &engine->registered_algs,
                              entry )
    {
        list_del( &alg->entry );
        crypto_unregister_alg( &alg->alg );
    }

map_fail:
irq_fail:
    free_page(( unsigned long )engine->zeroed_page);
out:


    return ret;
}

static int
spacc_remove( struct platform_device *pdev )
{
    struct pc302crypt_alg *alg;
    struct pc302crypt_alg *next;
    struct resource *mem_resource =
        platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    struct resource *irq =
        platform_get_resource( pdev, IORESOURCE_IRQ, 0 );
    struct pc302crypt_engine *engine = NULL;

    if ( !strcmp( "ipsec_engine", pdev->name ) )
        engine = &ipsec_engine;
    else if ( !strcmp( "l2_engine", pdev->name ) )
        engine = &l2_engine;

    BUG_ON( !engine );
    BUG_ON( !mem_resource );

    del_timer( &engine->packet_timeout );

    pc302crypt_release_and_unmap( mem_resource, engine->regs );
    free_irq( irq->start, engine );

    kfree(engine->ddt_info);
    
    dma_free_coherent( engine->dev, (engine->max_ctxs + 1) *
            sizeof( struct pc302crypt_engine_ctx ), engine->ddt_buf,
            engine->ddt_buf_phys );

    list_for_each_entry_safe( alg, next, &engine->registered_algs,
                              entry )
    {
        list_del( &alg->entry );
        crypto_unregister_alg( &alg->alg );
    }

    dma_unmap_sg(engine->dev, &engine->zeroed_list, 1, DMA_TO_DEVICE );
    free_page(( unsigned long )engine->zeroed_page);
    
    return 0;
}

static struct platform_driver ipsec_driver = {
    .probe  = spacc_probe,
    .remove = spacc_remove,
    .driver = {
        .name = "ipsec_engine",
    },
};

static struct platform_driver l2_driver = {
    .probe  = spacc_probe,
    .remove = spacc_remove,
    .driver = {
        .name = "l2_engine",
    },
};

static int
pc302crypt_init( void )
{
    int ret = platform_driver_register( &ipsec_driver );
    if ( ret )
    {
        PRINTD( DBG_ERROR, "failed to register platform driver" );
        goto out;
    }

    ret = platform_driver_register( &l2_driver );
    if ( ret )
    {
        PRINTD( DBG_ERROR, "failed to register platform driver" );
        goto l2_failed;
    }

    goto out;

l2_failed:
    platform_driver_unregister( &ipsec_driver );

out:
    return ret;
}

static void
pc302crypt_exit( void )
{
    platform_driver_unregister( &ipsec_driver );
    platform_driver_unregister( &l2_driver );
}

module_init( pc302crypt_init );
module_exit( pc302crypt_exit );

MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Jamie Iles" );
