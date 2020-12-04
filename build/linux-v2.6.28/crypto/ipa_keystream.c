/****************************************************************************
 *
 * IP.ACCESS -
 *
 * Copyright (c) 2010 ip.access Ltd.
 *
 ****************************************************************************
 *
 * $Id: PC202/kernel/kernel-2.6.28/linux-v2.6.28-ipa-3.2.4-patch 1.4.2.18.1.2 2011/11/29 21:21:59GMT Mark Powell (mp3) Exp  $
 *
 ****************************************************************************
 *
 * File Description : keystream driver using an mmap interface
 *
 * Control and status is via ioctl calls.  Keystream data and status is made
 * available via mmap().  Crypto engine HW DMAs directly into the mmap area,
 * so care must be taken to ensure that the CPU and DMA don't overwrite each
 * other's output.
 *
 * The mmap area is a series of contiguous requests (wrapping when necessary).
 * Each request starts with a header of 32 bytes (one cache line) that holds
 * information to pass back to the caller (pduLength, contextId etc).  The
 * header is then followed by a block of keystream data that's rounded up to
 * a multiple of 32 bytes to ensure the next header is still cache aligned.
 * When a request is made, the count values that are used to form the IV for
 * the cipher are stored temporarily in the buffer that the result will be
 * stored in (one per PDU, saved in each PDU's space).  The counts are read
 * again just before the encryption is started.  This is the only time when
 * the CPU should access the DMA area in the kernel.  We have to ensure that
 * nothing we do will corrupt DMAed results.  When the counts are written
 * we explicitly flush them to SDRAM.  Similarly after we read the count we
 * invalidate the line we read to ensure that nothing will be tempted to
 * clean the line.  This shouldn't be necessary if we don't write to the
 * line in the meantime, but just in case something else is tempted to dirty
 * the line...
 *
 ****************************************************************************/


/****************************************************************************
 * Standard Library Includes
 ****************************************************************************/
#include <linux/cdev.h>
#include <linux/configfs.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/mm.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>


/****************************************************************************
 * Project Includes
 ****************************************************************************/
#include <linux/ipa/ipa_keystream.h>


/****************************************************************************
 * Private Definitions
 ****************************************************************************/
#define ALLOW_CACHED_USER_READS 

#define IPAC_DEVICE_MAJOR  242

/* We support on f8(kasumi) in this driver */
#define IPAC_CIPHER_KEY_LENGTH_BITS   128
#define IPAC_IV_LENGTH_BITS           64

#define IPAC_KEY_BYTES  ((IPAC_CIPHER_KEY_LENGTH_BITS + 7) / 8)
#define IPAC_IV_LENGTH_BYTES          ((IPAC_IV_LENGTH_BITS + 7) / 8)
#define IPAC_IV_LENGTH_LONGS          ((IPAC_IV_LENGTH_BITS + 31) / 32)


#define DEBUG_BASIC   ( DBG_ALWAYS | DBG_ERROR | DBG_WARN | DBG_NOTICE )
#define DEBUG_MASK    ( DEBUG_BASIC )

#define PRINTD( _mask, _fmt, ... ) \
    ({ \
        if ( _mask & DEBUG_MASK ) \
            printk( "%s:%u: " _fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__ ); \
    })


/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef enum   IpacDirectionTag     IpacDirection;

typedef struct IpacDeviceTag        IpacDevice;
typedef struct IpacDeviceNodeTag    IpacDeviceNode;
typedef struct IpacContextTag       IpacContext;
typedef struct IpacCipherRequestTag IpacCipherRequest;

enum IpacDebugOptions
{
    DBG_ALWAYS     =  (1 <<  0),  /* Used for getting messages during development*/
    DBG_ERROR      =  (1 <<  1),  /* Critical errors. */
    DBG_WARN       =  (1 <<  2),  /* Warning messages. */
    DBG_NOTICE     =  (1 <<  3),  /* Notices of abnormal conditions that the driver should handle. */
    DBG_FUNC_CALLS =  (1 <<  4),  /* Tracing of function entry and exit. */
    DBG_BUFFER     =  (1 <<  5),  /* Request buffer handling. */
    DBG_MEM_CACHE  =  (1 <<  6),  /* Track requests use of the mem cache */
    DBG_CIPHER     =  (1 <<  7),  /* Track creation and deletion of cipher contexts */
    DBG_CIPHER_REQ =  (1 <<  8),  /* Track cipher requests */
    DBG_MMAP       =  (1 <<  9),  /* Track mmap operations */
    DBG_SCATTER    =  (1 << 10),  /* Track scatter-gather */
    DBG_PDU_COUNT  =  (1 << 11),  /* Track counting of PDUs within a request */
};

enum IpacDirectionTag
{
    IPAC_UPLINK,
    IPAC_DOWNLINK
};

/* We have an array of contexts */
struct IpacContextTag
{
    struct crypto_ablkcipher   *cipher_p;                 /* Pointer to active cipher used for new requests */
    u32                        cipherActiveRequests;      /* How many requests have been submitted with the current cipher */
    u32                        ivFixedPart;               /* The combination of binding ID and direction */
    
    struct crypto_ablkcipher   *oldCipher_p;              /* Pointer to old cipher used for old requests when switching */
    u32                        oldCipherActiveRequests;   /* How many requests have still to finish with the old cipher */
};

/* The driver supports a number of device nodes that are held in an array.
 * Information specific to each node is held in the following structure.
 */
struct IpacDeviceNodeTag
{
    u8                 name[16];                    /* A name for the node */
    spinlock_t         lock;                        /* Lock to allow callbacks and user space calls */
    int                useCount;                    /* How many clients have opened this node (max one allowed) */
    IpacDirection      direction;                   /* Whether this node is for uplink or downlink */
    struct kmem_cache  *requestCache_p;             /* memory pool used for submitting requests */
    struct device      *dev_p;                      /* Pointer to the device created for this node */
    u8                 *vm_start;                   /* The start of the memory buffer as seen by the user */
    u8                 *startOfPool_p;              /* The start of the memory buffer as seen by the kernel */
    u8                 *endOfPool_p;                /* One past the end of the memory buffer */
    u8                 *firstRequest_p;             /* Pointer to the first (oldest) request */
    u8                 *lastRequest_p;              /* Pointer to a marker after the last (newest request) */
    u32                requestsPending;             /* The number of requests that the user has waiting */
    u8                 releasingNode;               /* Flag to indicate that the client is closing */
    u8                 inIoctlCall;                 /* Flag to indicate that the client task is running IOCTL */
    IpacContext        context[IPAC_NUM_CONTEXTS];  /* Cipher contexts for this node */
};
 

/* Have dynamically allocated requests */
struct IpacCipherRequestTag
{
    struct ablkcipher_request  *areq_p;
    u32                        iv[IPAC_IV_LENGTH_LONGS];
    struct scatterlist         src;
    struct scatterlist         dst[2]; /* dest buffer could be split across two pages */
    struct list_head           list;
    IpacRequest                *request_p;
};
  
/* Info about the driver/device */
struct IpacDeviceTag {
    struct cdev     cdev;
    dev_t           devno;
    struct class    *sysfs_class;
};

/****************************************************************************
 * Private Function Prototypes (Must be declared static)
 ****************************************************************************/

/* Callback functions for cipher */
static void ipacCipherCallback( struct crypto_async_request *req_p, int err );
static void ipacCipherComplete( struct crypto_async_request *req_p, int err );

/* Module operations */
static int ipacRelease( struct inode *inode, struct file *filp );
static int ipacOpen( struct inode *inode, struct file *filp );
static int ipacMmap( struct file *filp, struct vm_area_struct *vma );
static int ipacIoctl( struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg );
static ssize_t ipacRead( struct file *filp, char __user *buf, size_t buf_len, loff_t *offset );

/* Mmap operations */
static void ipacVmaOpen( struct vm_area_struct *vma );
static void ipacVmaClose( struct vm_area_struct *vma );
static int ipacVmaFault( struct vm_area_struct *vma, struct vm_fault *vmf );

/****************************************************************************
 * Private Constants (Must be declared static)
 ****************************************************************************/


/****************************************************************************
 * Exported Variables
 ****************************************************************************/

 
/****************************************************************************
 * Private Variables (Must be declared static)
 ****************************************************************************/
 
static struct file_operations ipacFileOps = {
    .owner    = THIS_MODULE,
    .open     = ipacOpen,
    .release  = ipacRelease,
    .ioctl    = ipacIoctl,
    .read     = ipacRead,
    .mmap     = ipacMmap,
};

static struct vm_operations_struct ipacNopageVmOps = {
    .open     = ipacVmaOpen,
    .close    = ipacVmaClose,
    .fault    = ipacVmaFault,
};


/* Zeroed page used for generating keystreams. */
static void           *ipacZeroedPage = NULL;
static IpacDevice      ipacDevice;
static IpacDeviceNode  ipacNodes[IPAC_MAX_DRIVER_NODES];


/******************************************************************************
 * Function Name : ipacFlushRequestInfo
 * Inputs        : request_p - Pointer to request header to flush.
 *
 * Description   : This function will clean a single request header from the
 *                 data cache to the SDRAM.  If the user space is accessing the
 *                 request headers then this is necessary to ensure that any
 *                 updates made in kernel space are seen in user space.
 *
 *****************************************************************************/
static void ipacFlushRequestInfo(const IpacRequest  *request_p)
{
    u8  *start_p = (u8*)request_p;
    u8  *end_p   = ((u8*)(request_p+1)) - 1;

    PRINTD(DBG_FUNC_CALLS, "ipacFlushRequestInfo(%p)", request_p);
 
    dmac_clean_range(start_p, end_p);
    
    PRINTD(DBG_FUNC_CALLS, "ipacFlushRequestInfo out");
}

#ifdef ALLOW_CACHED_USER_READS 
/******************************************************************************
 * Function Name : ipacInvalidateCipherData
 * Inputs        : start_p - Pointer to first byte to invalidate
 *                 end_p   - Pointer to last byte to invalidate
 *
 * Description   : This function will invalidate the cache line(s) holding a
 *                 cipher stream.  That ensures that the user app can see new
 *                 data, but still get the advantages of burst reads via the
 *                 data cache.
 *
 *****************************************************************************/
static void ipacInvalidateCipherData(u8* start_p, u8* end_p)
{
    PRINTD(DBG_FUNC_CALLS, "ipacInvalidateCipherData(%p, %p)", start_p, end_p);
 
    dmac_inv_range(start_p, end_p);
    
    PRINTD(DBG_FUNC_CALLS, "ipacInvalidateCipherData out");
}
#endif

/******************************************************************************
 * Function Name : ipacCreateCipher
 * Inputs        : request_p - Pointer to request header to flush.
 *
 * Description   : Create a block cipher and set the key
 *
 *****************************************************************************/
static struct crypto_ablkcipher* ipacCreateCipher(const u8*  key_p,
                                                  u32        keyLen)
{
    struct crypto_ablkcipher  *cipher = NULL;
    int                       ret;
    
    PRINTD(DBG_FUNC_CALLS, "ipacCreateCipher(%p, %u)", key_p, keyLen);
 
    if (IPAC_KEY_BYTES != keyLen)
    {
        PRINTD(DBG_NOTICE,
               "Unexpected length for f8(kasumi) key.  Wanted %d, but got %d",
               IPAC_KEY_BYTES, keyLen);
               
        cipher = NULL;
    }
    else
    {
        PRINTD(DBG_CIPHER, "Allocating cipher for f8(kasumi)");
        cipher = crypto_alloc_ablkcipher( "f8(kasumi)", 0, 0);
        if ( IS_ERR( cipher ) )
        {
            PRINTD(DBG_NOTICE, "Failed to allocate ablkcipher for f8(kasumi)");
            cipher = NULL;
        }
        else
        {
            PRINTD(DBG_CIPHER, "Allocated cipher %p for f8(kasumi)", cipher);
        
            PRINTD(DBG_CIPHER, "Setting key (len=%d) for cipher %p",
                                keyLen, cipher);
                                
            PRINTD(DBG_CIPHER, "%02X %02X %02X %02X %02X ...",
                                key_p[0], key_p[1], key_p[2], key_p[3], key_p[4]);
            
            ret = crypto_ablkcipher_setkey( cipher, key_p, keyLen );
            if (ret)
            {
                PRINTD(DBG_NOTICE,
                       "Freeing ablkcipher %p for f8(kasumi); setkey failure",
                        cipher);
                        
                crypto_free_ablkcipher( cipher );
                cipher = NULL;
            }
            else
            {
                PRINTD(DBG_CIPHER, "Setkey for cipher %p OK", cipher);
            }
        }
    }
        
    PRINTD(DBG_FUNC_CALLS, "ipacCreateCipher returned %p", cipher);
 
    return cipher;
}

/******************************************************************************
 * Function Name : ipacSetupContext
 * Inputs        : context_p   - Pointer to the context to configure.
 *                 ioctl_p     - Pointer to data passed by user request
 *                 direction   - uplink/downlink 
 *                 saveCurrent - whether to backup existing cipher info
 *                 cipher_p    - Pointer to the cipher to associate with the
 *                               context.
 *
 * Description   : Setup a context for a supplied cipher.  If necessary the
 *                 existing cipher will be backed up so that requests that
 *                 are already queued will be able to complete.
 *
 *****************************************************************************/
static void ipacSetupContext(IpacContext              *context_p,
                            IpacContextIoctl          *ioctl_p,
                            u32                       direction,
                            int                       saveCurrent,
                            struct crypto_ablkcipher  *cipher_p)
{
    u32  bearerId  = ioctl_p->bearerId;
   
    PRINTD(DBG_FUNC_CALLS, "ipacSetupContext(%p, %p, %u, %d, %p)",
                            context_p, ioctl_p, direction, saveCurrent, cipher_p);
 
    context_p->ivFixedPart = htonl(((bearerId & 0x1F) << 27) | ((direction & 1) << 26));
    
    if (saveCurrent)
    {
        /* Keep the old cipher around for a while if there are requests that 
         * still have to be actioned with the current cipher.
         */
        if (context_p->cipherActiveRequests)
        {
            PRINTD(DBG_CIPHER, "Backing up cipher %p to oldCipher", context_p->cipher_p);
            context_p->oldCipher_p             = context_p->cipher_p;
            context_p->oldCipherActiveRequests = context_p->cipherActiveRequests;
        }
        else
        {
            PRINTD(DBG_CIPHER, "Freeing cipher %p; update with no clients", context_p->cipher_p);
            crypto_free_ablkcipher( context_p->cipher_p );
            context_p->oldCipher_p             = NULL;
            context_p->oldCipherActiveRequests = 0;
        }
    }
    
    context_p->cipher_p = cipher_p;
    context_p->cipherActiveRequests = 0;
    
    PRINTD(DBG_FUNC_CALLS, "ipacSetupContext out");
}

/******************************************************************************
 * Function Name : ipacAllocateContext
 * Inputs        : filp    - file info associated with user's file descriptor
 *                 ioctl_p - Pointer to the context info passed by the user.
 *
 * Description   : Look for a free context and allocate it to the user.
 *                 Configure the context with the data passed by the user.
 *
 *****************************************************************************/
static int ipacAllocateContext(struct file       *filp, 
                               IpacContextIoctl  *ioctl_p)
{
    struct crypto_ablkcipher   *cipher_p = NULL;
    u32                        keyLength = IPAC_KEY_BYTES;
    IpacDeviceNode             *node_p   = filp->private_data;
    int                        ret       = -EBUSY;
    int                        contextId;
    
    PRINTD(DBG_FUNC_CALLS, "ipacAllocateContext(%p, %p)", filp, ioctl_p);

    /* Create the cipher before we acquire the spin lock.  Once we have
     * the spin lock, attempt to associate the cipher with an unused context.
     * If that association fails, we must free the cipher after releasing
     * the spin lock.
     */ 
    cipher_p = ipacCreateCipher(&ioctl_p->cipherKey[0], keyLength);
    if (cipher_p == NULL)
    {
        PRINTD(DBG_NOTICE, "Unable to create cipher");
        return -ENOMEM;
    }
    
    spin_lock_bh( &node_p->lock );
    
    /* Search for an unused context (one that doesn't have an associated ciper) */
    for (contextId=0; contextId<IPAC_NUM_CONTEXTS; ++contextId)
    {
        IpacContext *context_p = &node_p->context[contextId];
        
        if (context_p->cipher_p == NULL)
        {
            ipacSetupContext(context_p, ioctl_p, node_p->direction, 0, cipher_p);
            ret = contextId;
            break;
        }
    }

    spin_unlock_bh( &node_p->lock );
    
    if (ret == -EBUSY)
    {
        /* Failed to find unused context.  Discard the new cipher */
        crypto_free_ablkcipher( cipher_p );
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacAllocateContext returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacDeallocateContext
 * Inputs        : filp      - file info associated with user's file descriptor
 *                 contextId - Index of the context to be freed
 *
 * Description   : Free a context previously allocated to the user.
 *
 *****************************************************************************/
static int ipacDeallocateContext(struct file  *filp,
                                 int          contextId)
{
    IpacDeviceNode *node_p    = filp->private_data;
    IpacContext    *context_p = NULL;
    int            ret        = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacDeallocateContext(%p, %d)", filp, contextId);
 
    spin_lock_bh( &node_p->lock );
    
    if ((contextId < 0) || (contextId >= IPAC_NUM_CONTEXTS))
    {
        PRINTD(DBG_NOTICE, "Invalid contextId %d", contextId);
        ret = -EINVAL;
    }
    else
    {
        context_p = &node_p->context[contextId];
    
        if (context_p->cipher_p == NULL)
        {
            PRINTD(DBG_NOTICE, "No cipher associated with context to be deallocated");
            ret = -ENOENT;
        }
        else if ((context_p->oldCipher_p != NULL) && (context_p->cipherActiveRequests != 0))
        {
            PRINTD(DBG_NOTICE, "Already have two ciphers running, so deallocate can't be done yet");
            ret = -EBUSY;
        }
        else
        {
            /* Deactivate current cipher, but watch for existing requests */
            if (context_p->cipherActiveRequests)
            {
                PRINTD(DBG_CIPHER, "Saving cipher %p as oldCipher", context_p->cipher_p);
                context_p->oldCipher_p             = context_p->cipher_p;
                context_p->oldCipherActiveRequests = context_p->cipherActiveRequests;
            }
            else
            {
                PRINTD(DBG_CIPHER, "Releasing cipher %p ", context_p->cipher_p);
                crypto_free_ablkcipher( context_p->cipher_p );
            }
        
            context_p->cipher_p             = NULL;
            context_p->cipherActiveRequests = 0;
            
            ret = 0;
        }
    }
    
    spin_unlock_bh( &node_p->lock );

    PRINTD(DBG_FUNC_CALLS, "ipacDeallocateContext returned %d", ret);
 
    return ret;
}

/******************************************************************************
 * Function Name : ipacUpdateContext
 * Inputs        : filp    - file info associated with user's file descriptor
 *                 ioctl_p - Pointer to the context info passed by the user.
 *
 * Description   : Update a context with the information passed by the user.
 *
 *****************************************************************************/
static int ipacUpdateContext(struct file       *filp,
                             IpacContextIoctl  *ioctl_p)
{
    struct crypto_ablkcipher   *cipher_p  = NULL;
    u32                        keyLength  = IPAC_KEY_BYTES;
    IpacDeviceNode             *node_p    = filp->private_data;
    int                        contextId  = ioctl_p->contextId;
    IpacContext                *context_p = NULL;
    int                        ret        = -EINVAL;
    
    PRINTD(DBG_FUNC_CALLS, "ipacUpdateContext(%p, %p)", filp, ioctl_p);
 
    /* Create the cipher before we acquire the spin lock.    Once we have
     * the spin lock, attempt to update the context with the new cipher.
     * If that update fails, we must free the cipher after releasing
     * the spin lock.
     */ 
    cipher_p = ipacCreateCipher(&ioctl_p->cipherKey[0], keyLength);
    if (cipher_p == NULL)
    {
        PRINTD(DBG_NOTICE, "Unable to create cipher");
        return -ENOMEM;
    }
    
    spin_lock_bh( &node_p->lock );
    
    if ((contextId < 0) || (contextId >= IPAC_NUM_CONTEXTS))
    {
        PRINTD(DBG_NOTICE, "Invalid contextId %d", contextId);
        ret = -EINVAL;
    }
    else
    {
        context_p = &(node_p->context[contextId]);
    
        if ((context_p->cipher_p != NULL) && (context_p->oldCipher_p != NULL))
        {
            /* Both in use */
            PRINTD(DBG_NOTICE, "contextId %d is too busy", contextId);
            ret = -EAGAIN;
        }
        else if (context_p->cipher_p != NULL)
        {
            /* Current is in use, move to old */
            ipacSetupContext(context_p, ioctl_p, node_p->direction, 1, cipher_p);
            ret = contextId;
        }
        else
        {
            /* Current is not in use, can't update */
            PRINTD(DBG_NOTICE, "contextId %d is not in use", contextId);
            ret = -ENOENT;
        }
    }
    
    spin_unlock_bh( &node_p->lock );
    
    if (ret < 0)
    {
        /* Failed to update context.  Discard the new cipher */
        crypto_free_ablkcipher( cipher_p );
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacUpdateContext returned %d", ret);
 
    return ret;
}

/******************************************************************************
 * Function Name : ipacSetRequestParameters
 * Inputs        : request_p - Pointer to request structure to fill in
 *                 needed    - Number of bytes to allow for keystream data
 *                 filp      - file info associated with user's file descriptor
 *                 start_p   - Pointer to start request info from user
 *
 * Description   : Setup a request structure with enough information to start
 *                 the the request and to return the results to the user once
 *                 the request has completed.
 *
 *****************************************************************************/
static void ipacSetRequestParameters(IpacRequest     *request_p,
                                     u32             needed,
                                     struct file     *filp,
                                     IpacStartIoctl  *start_p)
{
    IpacDeviceNode  *node_p    = filp->private_data;
    IpacContext     *context_p = &node_p->context[start_p->contextId];

    PRINTD(DBG_FUNC_CALLS, "ipacSetRequestParameters(%p, %u, %p, %p)",
                            request_p, needed, filp, start_p);

    /* This info has to be returned to the user */ 
    request_p->transactionId = start_p->transactionId;
    request_p->contextId     = start_p->contextId;
    request_p->numberOfPdus  = start_p->numPdus;
    request_p->pduLength     = start_p->pduLength;

    /* This info is needed to submit the request and handle the completion */
    request_p->channel       = node_p - ipacNodes;
    request_p->cipher_p      = context_p->cipher_p;
    request_p->ivFixedPart   = context_p->ivFixedPart;
    request_p->reservedBytes = needed;
    request_p->pdusCompleted = 0;
    
    /* Track use of each cipher so we know when it's safe to clean them up */
    context_p->cipherActiveRequests += start_p->numPdus;
    
    PRINTD(DBG_FUNC_CALLS, "ipacSetRequestParameters out");
}


/******************************************************************************
 * Function Name : ipacSubmitRequest
 * Inputs        : node_p    - Pointer to device node
 *                 request_p - Pointer to request header in memory buffer
 *
 * Description   : Takes the information for a cipher request from the
 *                 memory pool and submits it to the cipher engine.
 *
 *****************************************************************************/
static int ipacSubmitRequest(IpacDeviceNode  *node_p,
                             IpacRequest     *request_p,
                             IpacStartIoctl  *start_p)
{
    u8  *pduBuffer_p   = NULL;
    u32 *counts_p      = start_p->counts;
    int  ret           = 0;
    u32  pdu;
    
    PRINTD(DBG_FUNC_CALLS, "ipacSubmitRequest(%p, %p, %p)", node_p, request_p, start_p);
 
    if (request_p == NULL)
    {
        PRINTD(DBG_ERROR, "Invalid (null) request_p");
        ret = -EINVAL;
        goto out;
    }
    
    pduBuffer_p = (u8*)(request_p+1);
    
    request_p->state = IPAC_REQUEST_CIPHERING;

    for (pdu=0; pdu<request_p->numberOfPdus; ++pdu)
    {
        IpacCipherRequest  *cipherRequest_p = NULL;
        u32                count            = 0;
        int                clen             = request_p->pduLength;
       
        PRINTD(DBG_MEM_CACHE, "Allocating request from cache %s for count %u",
                               node_p->name, count);
                               
        if (copy_from_user(&count, counts_p, sizeof(u32)))
        {
            PRINTD(DBG_NOTICE, "Unable to copy counts[%d/%d] from user space at %p",
                                pdu, start_p->numPdus, start_p->counts);
            ret = -EINVAL;
            
            /* Since we haven't submitted all the PDUs we will terminiate
             * waiting for results early too */
            request_p->numberOfPdus  = pdu;
            
            goto out;
        }
        counts_p++;
                               
        cipherRequest_p = kmem_cache_alloc( node_p->requestCache_p, GFP_ATOMIC );
        if ( !cipherRequest_p )
        {
            PRINTD(DBG_NOTICE, "Unable to allocate cipherRequest_p");
            ret = -ENOMEM;
            
            /* Since we haven't submitted all the PDUs we will terminiate
             * waiting for results early too */
            request_p->numberOfPdus  = pdu;
            
            goto out;
        }
        
        PRINTD(DBG_MEM_CACHE, "Allocated %p from request cache %s for count %u",
                               cipherRequest_p, node_p->name, count);
        
        cipherRequest_p->request_p = request_p;
        
        PRINTD(DBG_CIPHER_REQ, "Allocating cipher request for cipher %p",
                                request_p->cipher_p);
                                
        cipherRequest_p->areq_p = ablkcipher_request_alloc( request_p->cipher_p, GFP_ATOMIC );
        if ( !cipherRequest_p->areq_p )
        {
            PRINTD(DBG_NOTICE, "Unable to allocate cipher request for cipher %p", request_p->cipher_p);
            ret = -ENOMEM;
            
            PRINTD(DBG_MEM_CACHE, "Freeing request %p to request cache %s", cipherRequest_p, node_p->name);
            kmem_cache_free( node_p->requestCache_p, cipherRequest_p );
            
            
            /* Since we haven't submitted all the PDUs we will terminiate
             * waiting for results early too */
            request_p->numberOfPdus  = pdu;
            
            goto out;
        }
        
        PRINTD(DBG_CIPHER_REQ, "Allocated cipher request %p for cipher %p",
                                cipherRequest_p->areq_p, request_p->cipher_p);

        /* Create the IV for the request. */
        cipherRequest_p->iv[0] = htonl(count);
        cipherRequest_p->iv[1] = request_p->ivFixedPart;

        /* Single memory page in use */
        sg_init_table( &cipherRequest_p->dst[0], 1 );
        sg_set_buf( &cipherRequest_p->dst[0], pduBuffer_p, clen );
        
        PRINTD(DBG_CIPHER_REQ, "Setting callback for cipher request %p", cipherRequest_p->areq_p);
        
        ablkcipher_request_set_callback( cipherRequest_p->areq_p, CRYPTO_TFM_REQ_MAY_BACKLOG,
                                            ipacCipherCallback, request_p );
                                            
        PRINTD(DBG_CIPHER_REQ, "Setting crypt params for cipher request %p", cipherRequest_p->areq_p);
        
        ablkcipher_request_set_crypt( cipherRequest_p->areq_p, 0,
                                      &cipherRequest_p->dst[0], clen, cipherRequest_p->iv );
    
        PRINTD(DBG_CIPHER_REQ, "Starting encrypt for cipher request %p", cipherRequest_p->areq_p);
        
        ret = crypto_ablkcipher_encrypt( cipherRequest_p->areq_p );
        if ( !ret )
        {
            /* We have completed synchronously. Call the completion function.
                * This will free the request for us. */
            PRINTD(DBG_CIPHER_REQ, "Encrypt for cipher request %p finished synchronously",
                                    cipherRequest_p->areq_p);
                                    
            ipacCipherComplete( &cipherRequest_p->areq_p->base, 0 );
        }
        else if ( -EINPROGRESS == ret )
        {
            PRINTD(DBG_CIPHER_REQ, "Encrypt for cipher request %p is in progress",
                                    cipherRequest_p->areq_p);
                                    
            ret = 0;
        }
        else
        {
            PRINTD(DBG_NOTICE, "encryption failed with %d", ret );
            
            ret = -ENOMEM;
            
            /* Since we haven't submitted all the PDUs we will terminiate
             * waiting for results early too */
            request_p->numberOfPdus  = pdu;
            
            goto out;
        }
        
        pduBuffer_p += request_p->pduLength;
    }
    
out:
    PRINTD(DBG_BUFFER, "After submit requestpool=%p, end=%p, first=%p, last=%p",
                        node_p->startOfPool_p, node_p->endOfPool_p, node_p->firstRequest_p,
                        node_p->lastRequest_p);
    PRINTD(DBG_FUNC_CALLS, "ipacSubmitRequest returned %d", ret);
 
    return 0;
}

/******************************************************************************
 * Function Name : ipacCipherComplete
 * Inputs        : req_p - Pointer to the cipher request that was submitted
 *                 err   - Status of the cipher request
 *
 * Description   : Called when the cipher engine has completed a cipher
 *                 request.  The request in the memory pool is updated.
 *
 *****************************************************************************/
static void ipacCipherComplete(struct crypto_async_request  *req_p,
                               int                          err)
{
    IpacRequest               *request_p       = req_p->data;
    struct ablkcipher_request *ablk_req_p      = ablkcipher_request_cast( req_p );
    IpacCipherRequest         *cipherRequest_p = container_of( ablk_req_p->dst, IpacCipherRequest, dst[0] );
    IpacDeviceNode            *node_p          = &ipacNodes[request_p->channel];
    
    PRINTD(DBG_FUNC_CALLS, "ipacCipherComplete(%p, %d)", req_p, err);
 
    if ( !err )
    {
        struct crypto_ablkcipher  *cipher_p     = request_p->cipher_p;
        int                        contextId = request_p->contextId;
        IpacContext         *context_p    = &(node_p->context[contextId]);
        
        if (cipher_p == context_p->cipher_p)
        {
            PRINTD(DBG_CIPHER_REQ, "Completed cipher request %p for current cipher %p", ablk_req_p, cipher_p);
            
            context_p->cipherActiveRequests--;
        }
        else if (cipher_p == context_p->oldCipher_p)
        {
            PRINTD(DBG_CIPHER_REQ, "Completed cipher request %p for old cipher %p", ablk_req_p, cipher_p);
            
            context_p->oldCipherActiveRequests--;
            
            if (context_p->oldCipherActiveRequests == 0)
            {
                /* This request will not be used any more */
                PRINTD(DBG_CIPHER, "Freeing old cipher %p; no old clients left", cipher_p);
                context_p->oldCipher_p = NULL;
                crypto_free_ablkcipher( cipher_p );
            }
        }
        
        request_p->pdusCompleted++;
        PRINTD(DBG_PDU_COUNT, "Completed %d pdus of %d", request_p->pdusCompleted, request_p->numberOfPdus);
        if (request_p->pdusCompleted == request_p->numberOfPdus)
        {
            /* Entire request has been completed and can now be returned */
            request_p->state = IPAC_REQUEST_COMPLETED;
        }
    }
    else
    {
        PRINTD(DBG_NOTICE, "encryption on keystream channel %lu failed "
                "with code %d", request_p->channel, err );
    }
    
    PRINTD(DBG_CIPHER_REQ, "Freeing cipher request %p", cipherRequest_p->areq_p);
    ablkcipher_request_free( cipherRequest_p->areq_p );
    cipherRequest_p->areq_p = NULL;
    
    PRINTD(DBG_MEM_CACHE, "Freeing request %p to request cache %s", cipherRequest_p, node_p->name);
    kmem_cache_free( node_p->requestCache_p, cipherRequest_p );
    
    PRINTD(DBG_FUNC_CALLS, "ipacCipherComplete out");
}

/******************************************************************************
 * Function Name : ipaCipherCallback
 * Inputs        : req_p - Pointer to the cipher request that was submitted
 *                 err   - Status of the cipher request
 *
 * Description   : Cipher engine callback routine.  Calls completion function.
 *
 *****************************************************************************/
static void ipacCipherCallback(struct crypto_async_request  *req_p,
                               int                          err)
{
    IpacRequest     *request_p = req_p->data;
    IpacDeviceNode  *node_p    = &ipacNodes[request_p->channel];
   
    PRINTD(DBG_FUNC_CALLS, "ipacCipherCallback(%p, %d)", req_p, err);
 
    spin_lock_bh( &node_p->lock );
    
    ipacCipherComplete( req_p, err );
    
    spin_unlock_bh( &node_p->lock );
    
    PRINTD(DBG_FUNC_CALLS, "ipacCipherCallback out");
}

/******************************************************************************
 * Function Name : ipacAllocateRequestBuffer
 * Inputs        : filp      - file info for user's file descriptor
 *                 start_p   - Pointer to the user's request information
 *                 numPdus   - Number of PDUs to allocate
 *                 pduLength - Size of each PDU in bytes
 * Outputs       : buffer_pp - Pointer to allocated request header
 *
 * Description   : Checks for space for the PDUs.  If the caller has passed
 *                 in a non-NULL buffer_pp then the buffer space will actually
 *                 be reserved and the header of the reserved buffer will be
 *                 returned in buffer_pp.
 *
 *****************************************************************************/
static int ipacAllocateRequestBuffer(struct file     *filp,
                                     IpacStartIoctl  *start_p,
                                     u32             numPdus,
                                     u32             pduLength,
                                     IpacRequest     **buffer_pp)
{
    IpacDeviceNode  *node_p     = filp->private_data;
    IpacRequest     *buffer     = NULL;
    IpacRequest     *lastBuffer = NULL;
    IpacRequest     *wrapBuffer = NULL;
    static u32      align       = 0xFFFFFFFF;
    int             needed      = numPdus * pduLength;
    int             ret         = -ENOMEM;
    
    PRINTD(DBG_FUNC_CALLS, "ipacAllocateRequestBuffer(%p, %p, %u, %u, %p)",
                            filp, start_p, numPdus, pduLength, buffer_pp);
    
    spin_lock_bh( &node_p->lock );
    
    if (align == 0xFFFFFFFF)
    {
        /* Determine the minimum alignment requirement for the IpacRequest - 1 */
        align = offsetof(struct {u8 dummy; IpacRequest x; }, x) - 1;
        if (align < 31)
        {
             align = 31;
        }
        PRINTD(DBG_BUFFER, "buffer alignment is %u", align+1);
    }
    
    /* Round the needed bytes up to keep the headers correctly aligned. */
    needed += align;
    needed |= align;
    needed -= align;

    PRINTD(DBG_BUFFER, "Need %u bytes for %u of %u", needed, numPdus, pduLength);
    
    if (node_p->firstRequest_p > node_p->lastRequest_p)
    {
        /* Buffer has wrapped, so the available space lies between the last and first
         *                |......L         F.....  |
         */
        int midSpace = node_p->firstRequest_p - node_p->lastRequest_p - 2*sizeof(IpacRequest);
        
        PRINTD(DBG_BUFFER, "MID space is %d bytes", midSpace);

        if (midSpace > needed)
        {
            PRINTD(DBG_BUFFER, "Can use MID space");
            if (buffer_pp)
            {
                buffer  = (IpacRequest *)node_p->lastRequest_p;
                
                node_p->lastRequest_p += needed + sizeof(IpacRequest);
                lastBuffer = (IpacRequest *)node_p->lastRequest_p;
                
                PRINTD(DBG_BUFFER, "Inserting new=%p, last=%p", buffer, lastBuffer);
                memset(lastBuffer, 0, sizeof(lastBuffer));
                ipacSetRequestParameters(buffer, needed, filp, start_p);
                
                lastBuffer->state = IPAC_REQUEST_FINAL;
                buffer->state = IPAC_REQUEST_WAITING;

                *buffer_pp = buffer;
            }
            
            ret = 0;
        }
    }
    else
    {
        /* Buffer hasn't wrapped, so there's potentially space both after the last and
         * before the first.
         *           |    F.......L   |
         */
         int headSpace = node_p->firstRequest_p - node_p->startOfPool_p - 2*sizeof(IpacRequest);
         int tailSpace = node_p->endOfPool_p    - node_p->lastRequest_p - 2*sizeof(IpacRequest);
         
         PRINTD(DBG_BUFFER, "HEAD space is %d bytes", headSpace);
         PRINTD(DBG_BUFFER, "TAIL space is %d bytes", tailSpace);

         if (tailSpace >= needed)
         {
            PRINTD(DBG_BUFFER, "Can use TAIL space");
            if (buffer_pp)
            {
                buffer  = (IpacRequest *)node_p->lastRequest_p;
                node_p->lastRequest_p += needed + sizeof(IpacRequest);
                lastBuffer = (IpacRequest *)node_p->lastRequest_p;
                
                PRINTD(DBG_BUFFER, "Appending new=%p, last=%p", buffer, lastBuffer);
                memset(lastBuffer, 0, sizeof(lastBuffer));
                ipacSetRequestParameters(buffer, needed, filp, start_p);
                
                lastBuffer->state = IPAC_REQUEST_FINAL;
                buffer->state = IPAC_REQUEST_WAITING;

                *buffer_pp = buffer;
            }
            
            ret = 0;
         }
         else if (headSpace >= needed)
         {
            PRINTD(DBG_BUFFER, "Can use HEAD space");
            if (buffer_pp)
            {
                wrapBuffer = (IpacRequest *)node_p->lastRequest_p;
                buffer = (IpacRequest *)node_p->startOfPool_p;
                node_p->lastRequest_p = node_p->startOfPool_p + needed + sizeof(IpacRequest);
                lastBuffer = (IpacRequest *)node_p->lastRequest_p;
                
                PRINTD(DBG_BUFFER, "Wrapping wrap=%p, new=%p, last=%p", wrapBuffer, buffer, lastBuffer);
                ipacSetRequestParameters(buffer, needed, filp, start_p);
                memset(lastBuffer, 0, sizeof(lastBuffer));
                
                lastBuffer->state = IPAC_REQUEST_FINAL;
                wrapBuffer->state = IPAC_REQUEST_WRAPPED;
                buffer->state = IPAC_REQUEST_WAITING;

                *buffer_pp = buffer;
            }
            
            ret = 0;
         }
    }
    
    PRINTD(DBG_BUFFER, "After alloc request pool=%p, end=%p, first=%p, last=%p",
                        node_p->startOfPool_p, node_p->endOfPool_p, node_p->firstRequest_p,
                        node_p->lastRequest_p);
    
    spin_unlock_bh( &node_p->lock );

    if (lastBuffer)
    {
        ipacFlushRequestInfo(lastBuffer);
    }
    if (wrapBuffer)
    {
        ipacFlushRequestInfo(wrapBuffer);
    }
    if (buffer)
    {
        ipacFlushRequestInfo(buffer);
    }

    PRINTD(DBG_FUNC_CALLS, "ipacAllocateRequestBuffer returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacCheckSpace
 * Inputs        : filp             - Info associated with user's file desc.
 *                 maxSpaceRequired - Number of bytes needed.
 *
 * Description   : Ioctl handler function for determining if there is space
 *                 available for a request.
 *
 *****************************************************************************/
static int ipacCheckSpace(struct file  *filp,
                          u32          maxSpaceRequired)
{
    IpacDeviceNode     *node_p = filp->private_data;
    int                ret     = 0;
    
    PRINTD(DBG_FUNC_CALLS, "ipacCheckSpace(%p, %u)", filp, maxSpaceRequired);
    
    if (IPAC_MAX_NUM_BUFFERS_ALLOWED && (node_p->requestsPending >= IPAC_MAX_NUM_BUFFERS_ALLOWED))
    {
        PRINTD(DBG_NOTICE, "More than the maximum number of requests would be active");
        ret = -EBUSY;
        goto out;
    }
    
    ret = ipacAllocateRequestBuffer(filp, NULL, 1, maxSpaceRequired, NULL);
    
    PRINTD(DBG_FUNC_CALLS, "ipacCheckSpace returned %d", ret);

out:    
    return ret;
}

/******************************************************************************
 * Function Name : ipacStartCipher
 * Inputs        : filp    - Info associated with user's file descriptor.
 *                 start_p - Pointer to the user data passed in the ioctl
 *
 * Description   : Validate a user request for generating cipher data.  If
 *                 it's OK then add the request to the buffer and mark it
 *                 waiting.  If the request can then be submitted to the
 *                 cipher engine then do that too.
 *
 *****************************************************************************/
static int ipacStartCipher(struct file     *filp,
                           IpacStartIoctl  *start_p)
{
    IpacDeviceNode  *node_p      = filp->private_data;
    IpacRequest     *request_p   = NULL;
    int             ret          = -ENOMEM;
   
    PRINTD(DBG_FUNC_CALLS, "ipacStartCipher(%p, %p)", filp, start_p);
    
    
    /* Ensure we have a valid context */
    if (!node_p->context[start_p->contextId].cipher_p)
    {
        PRINTD(DBG_NOTICE, "Context is not valid");
        ret = -EINVAL;
        goto out;
    }
   
    /* Ensure the pduLength is a multiple of 4 */
    if (start_p->pduLength % 4)
    {
        PRINTD(DBG_NOTICE, "PDU is not a mutiple of 4 bytes");
        ret = -EINVAL;
        goto out;
    }
    
    if (IPAC_MAX_NUM_BUFFERS_ALLOWED && (node_p->requestsPending >= IPAC_MAX_NUM_BUFFERS_ALLOWED))
    {
        PRINTD(DBG_NOTICE, "More than the maximum number of requests would be active");
        ret = -EBUSY;
        goto out;
    }
    
    PRINTD(DBG_PDU_COUNT, "Requesting %u pdus of %u bytes", start_p->numPdus, start_p->pduLength);
    ret = ipacAllocateRequestBuffer(filp, start_p, start_p->numPdus, start_p->pduLength, &request_p);
    if (ret)
    {
        PRINTD(DBG_NOTICE, "Request failed %d", ret);
        goto out;
    }
    
    node_p->requestsPending++;
    
    ret = ipacSubmitRequest(node_p, request_p, start_p);

out:
    
    PRINTD(DBG_FUNC_CALLS, "ipacStartCipher returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipaCheckCryptoReady
 * Inputs        : filp - Info associated with user's file descriptor.
 *
 * Description   : Check whether there are any requests that have finished
 *                 that can be returned to the user.
 *
 *****************************************************************************/
static int ipacCheckCryptoReady(struct file  *filp)
{
    IpacDeviceNode     *node_p    = filp->private_data;
    IpacRequest        *request_p = NULL;
    IpacRequest        *flush_p   = NULL;
    int                ret        = 0;
   
    PRINTD(DBG_FUNC_CALLS, "ipacCheckCryptoReady(%p)", filp);
    
    spin_lock_bh( &node_p->lock );
    
    request_p = (IpacRequest*)node_p->firstRequest_p;
    
    PRINTD(DBG_BUFFER, "CheckCryptoReady firstRequest_p is %p(%d)",
                        node_p->firstRequest_p, request_p->state);
 
    if (request_p->state == IPAC_REQUEST_WRAPPED)
    {
        PRINTD(DBG_BUFFER, "Reached end of the buffer.  Wrapping to start of buffer");
        flush_p = request_p;
        node_p->firstRequest_p = node_p->startOfPool_p;
        request_p = (IpacRequest*)node_p->firstRequest_p;
    }
    
    if (request_p->state == IPAC_REQUEST_COMPLETED)
    {
        PRINTD(DBG_BUFFER, "Marking returned buffer at %p", request_p);
        request_p->state = IPAC_REQUEST_RETURNED;

        ret = node_p->firstRequest_p - node_p->startOfPool_p;
    }
    else if (request_p->state == IPAC_REQUEST_RETURNED)
    {
        PRINTD(DBG_BUFFER, "Buffer is still marked as returned at %p", request_p);

        ret = node_p->firstRequest_p - node_p->startOfPool_p;
    }
    else
    {
        ret = -EBUSY;
    }
    
    spin_unlock_bh( &node_p->lock );

    if (flush_p)
    {
        ipacFlushRequestInfo(flush_p);
    }
    
    ipacFlushRequestInfo(request_p);
    
    PRINTD(DBG_FUNC_CALLS, "ipacCheckCryptoReady returned %d", ret);

    return ret;    
}

/******************************************************************************
 * Function Name : ipacRead
 * Inputs        : filp    - Info associated with user's file descriptor
 *                 buf     - User buffer to fill
 *                 buf_len - Max number of bytes to return
 *                 offset  - file offset (not used)
 *
 * Description   : This provides an alternative interface to the mmap().
 *                 Using this it is possible to read the cipher data back
 *                 with a read() system call.  This function was added for
 *                 initial testing and is not intended for use under normal
 *                 circumstances.
 *
 *****************************************************************************/
static ssize_t ipacRead(struct file  *filp,
                        char __user  *buf,
                        size_t       buf_len,
                        loff_t       *offset )
{
    IpacDeviceNode  *node_p    = filp->private_data;
    IpacRequest     *request_p = NULL;
    int             ret        = 0;
   
    PRINTD(DBG_FUNC_CALLS, "ipacRead(%p, %p, %d, %p)", filp, buf, buf_len, offset);
    
    spin_lock_bh( &node_p->lock );
    
    request_p = (IpacRequest*)node_p->firstRequest_p;
    
    PRINTD(DBG_BUFFER, "Read firstRequest_p is %p(%d)", node_p->firstRequest_p, request_p->state);
 
    if (request_p->state == IPAC_REQUEST_WRAPPED)
    {
        PRINTD(DBG_BUFFER, "Reached end of the buffer.  Wrapping to start of buffer");
        node_p->firstRequest_p = node_p->startOfPool_p;
        request_p = (IpacRequest*)node_p->firstRequest_p;
    }
    
    if ((request_p->state == IPAC_REQUEST_COMPLETED) || (request_p->state == IPAC_REQUEST_RETURNED))
    {
        size_t  bytes_to_copy = min( buf_len, sizeof(IpacRequest)+request_p->numberOfPdus*request_p->pduLength);

        /* If an attempt at reading fails, the user space app is free to either try again,
         * or discard the data by unloading it and continuing.
         */
        PRINTD(DBG_BUFFER, "Marking returned buffer at %p", request_p);
        request_p->state = IPAC_REQUEST_RETURNED;
        
        spin_unlock_bh( &node_p->lock );
    
        ret = copy_to_user( buf, request_p, bytes_to_copy );
        if ( ret )
        {
            PRINTD(DBG_NOTICE, "Unable to copy request_p to user space at %p", buf);
            ret = -EFAULT;
        }
        else
        {
            ipacFlushRequestInfo(request_p);
    
            ret = bytes_to_copy;
        }
    }
    else
    {
        spin_unlock_bh( &node_p->lock );
        ret = -EBUSY;
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacRead returned %d", ret);
    
    return ret;
}


/******************************************************************************
 * Function Name : ipacBufferUnload
 * Inputs        : filp - Info associated with user's file descriptor
 *
 * Description   : This function is called (via an ioctl) when the user has
 *                 finished with the cipher data in a request buffer.  It
 *                 allows the driver to reclaim the buffer space for further
 *                 requests.
 *
 *****************************************************************************/
static int ipacBufferUnload(struct file *filp)
{
    IpacDeviceNode  *node_p          = filp->private_data;
    IpacRequest     *request_p       = NULL;
    IpacRequest     *flushPreWrap_p  = NULL;
    IpacRequest     *flushResult_p   = NULL;
    IpacRequest     *flushPostWrap_p = NULL;
    int             ret              = 0;
   
    PRINTD(DBG_FUNC_CALLS, "ipacBufferUnload(%p)", filp);
    
    PRINTD(DBG_BUFFER, "BufferUnload firstRequest_p is %p", node_p->firstRequest_p);
 
    spin_lock_bh( &node_p->lock );
    
    request_p = (IpacRequest*)node_p->firstRequest_p;
    
    if (request_p->state == IPAC_REQUEST_WRAPPED)
    {
        PRINTD(DBG_BUFFER, "Reached end of the buffer.  Wrapping to start of buffer");
        request_p->state = IPAC_REQUEST_UNUSED;
        flushPreWrap_p = request_p;
        node_p->firstRequest_p = node_p->startOfPool_p;
        request_p = (IpacRequest*)node_p->firstRequest_p;
    }
    
    if (request_p->state != IPAC_REQUEST_RETURNED)
    {
        spin_unlock_bh( &node_p->lock );
        PRINTD(DBG_NOTICE, "Buffer being unloaded isn't completed");
        ret = -EINVAL;
    }
    else
    {
#ifdef ALLOW_CACHED_USER_READS 
        u8   *start_p = ((u8*)request_p - node_p->startOfPool_p) + node_p->vm_start;
        u8   *end_p   = start_p + request_p->reservedBytes + sizeof(IpacRequest);
#endif

        PRINTD(DBG_BUFFER, "Moving past %d bytes of space", request_p->reservedBytes);
    
        flushResult_p = request_p;
        
        request_p->state = IPAC_REQUEST_UNUSED;
        node_p->firstRequest_p += request_p->reservedBytes + sizeof(IpacRequest);
    
        request_p = (IpacRequest*)node_p->firstRequest_p;
    
        node_p->requestsPending--;

        if (request_p->state == IPAC_REQUEST_WRAPPED)
        {
            PRINTD(DBG_BUFFER, "Reached end of the buffer.  Wrapping to start of buffer");
            request_p->state = IPAC_REQUEST_UNUSED;
            flushPostWrap_p = request_p;
            node_p->firstRequest_p = node_p->startOfPool_p;
        }
        
        spin_unlock_bh( &node_p->lock );

#ifdef ALLOW_CACHED_USER_READS
        ipacInvalidateCipherData(start_p, end_p);
#endif
        
        ipacFlushRequestInfo(flushResult_p);
        
        if (flushPostWrap_p)
        {
            ipacFlushRequestInfo(flushPostWrap_p);
        }
        
        PRINTD(DBG_BUFFER, "After buffer unload pool=%p, end=%p, first=%p, last=%p",
                            node_p->startOfPool_p, node_p->endOfPool_p, node_p->firstRequest_p,
                            node_p->lastRequest_p);
    }

    if (flushPreWrap_p)
    {    
        ipacFlushRequestInfo(flushPreWrap_p);
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacBufferUnload returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacIoctl
 * Inputs        : inode - inode associated with the dev node
 *                 filp  - info associated with the user's file descriptor
 *                 cmd   - the ioctl cmd
 *                 arg   - the arg associated with the cmd
 *
 * Description   : This is the ioctl handler for the driver.  It passes any
 *                 recognised commands to specific handler functions.
 *
 *****************************************************************************/
static int ipacIoctl( struct inode   *inode,
                      struct file    *filp,
                      unsigned int    cmd,
                      unsigned long   arg )
{
    IpacStartIoctl    startIoctl;
    IpacContextIoctl  contextIoctl;
    int               ret          = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacIoctl(%p, %p, %u, %lu)", inode, filp, cmd, arg);
    
    /* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
    if (_IOC_TYPE(cmd) != IPAC_IOCTL_MAGIC){
        PRINTD(DBG_NOTICE, "Invalid ioctl command 0x%x (arg=%lu)", cmd, arg );
        ret = -ENOTTY;
    }
    else if (_IOC_NR(cmd) > IPAC_IOCTL_MAXNR)
    {
        PRINTD(DBG_NOTICE, "Invalid ioctl command 0x%x (arg=%lu)", cmd, arg );
        ret = -ENOTTY;
    }
    else
    {
        IpacDeviceNode  *node_p       = filp->private_data;

        /* Atomically check whether the file is closing and, if not, indicate
         * to the ipacRelease function that we are busy processing an ioctl
         * request.
         */
        spin_lock_bh( &node_p->lock );
        if (node_p->releasingNode)
        {
            spin_unlock_bh( &node_p->lock );
            ret = -EBUSY;
        }
        else
        {
            node_p->inIoctlCall++;
            spin_unlock_bh( &node_p->lock );

            switch (cmd)
            {
            case IPAC_IOCTL_ALLOC_CONTEXT:
                if (copy_from_user( &contextIoctl, (void*)arg, sizeof(contextIoctl)))
                {
                    PRINTD(DBG_NOTICE, "Unable to copy ioctl data from user");
                    ret = -EINVAL;
                }
                else if (contextIoctl.contextId < 0)
                {
                    ret = ipacAllocateContext(filp, &contextIoctl);
                }
                else
                {
                    ret = ipacUpdateContext(filp, &contextIoctl);
                }
                break;
        
            case IPAC_IOCTL_DEALLOC_CONTEXT:
                ret = ipacDeallocateContext(filp, arg);
                break;
        
            case IPAC_IOCTL_CHECK_SPACE:
                ret = ipacCheckSpace(filp, arg);
                break;
        
            case IPAC_IOCTL_START_CRYPTO:
                if (copy_from_user( &startIoctl, (void*)arg, sizeof(startIoctl)))
                {
                    PRINTD(DBG_NOTICE, "Unable to copy ioctl data from user");
                    ret = -EINVAL;
                }
                else
                {
                    ret = ipacStartCipher(filp, &startIoctl);
                }
                break;
        
            case IPAC_IOCTL_CHECK_CRYPTO_READY:
                ret = ipacCheckCryptoReady(filp);
                break;
        
            case IPAC_IOCTL_BUFFER_UNLOAD:
                ret = ipacBufferUnload(filp);
                break;
        
            default:
                /* redundant, as cmd was checked against MAXNR */
                ret = -ENOTTY;
            }

            /* No longer busy in ioctl */
            spin_lock_bh( &node_p->lock );
            if (node_p->inIoctlCall > 0)
            {
                node_p->inIoctlCall--;
            }
            spin_unlock_bh( &node_p->lock );
        }
    }

    PRINTD(DBG_FUNC_CALLS, "ipacIoctl returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacVmaOpen
 * Inputs        : vma - Pointer to the vma associated with the mmap
 *
 * Description   : Dummy function
 *
 *****************************************************************************/
static void ipacVmaOpen(struct vm_area_struct *vma)
{
    PRINTD(DBG_FUNC_CALLS, "ipacVmaOpen(%p)", vma );
    PRINTD(DBG_FUNC_CALLS, "ipacVmaOpen out");
}

/******************************************************************************
 * Function Name : ipacVmaClose
 * Inputs        : vma - Pointer to the vma associated with the mmap
 *
 * Description   : Dummy function
 *
 *****************************************************************************/
static void ipacVmaClose(struct vm_area_struct *vma)
{
    PRINTD(DBG_FUNC_CALLS, "ipacVmaClose(%p)", vma );
    PRINTD(DBG_FUNC_CALLS, "ipacVmaClose out");
}

/******************************************************************************
 * Function Name : ipacVmaFault 
 * Inputs        : vma - Pointer to the vma associated with the mmap
 *                 vmf - Pointer to a structure for passing page fault info.
 *
 * Description   : This is the mmap page fault handler.  As the user space
 *                 accesses pages in the mapped area, this is called to map
 *                 the user space to the kernel pages used for the buffer.
 *
 *****************************************************************************/
static int ipacVmaFault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    IpacDeviceNode     *node_p  = vma->vm_private_data;
    struct page        *page    = NULL;
    u8*                offset   = NULL;
    int                ret      = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacVmaFault(%p, %p)", vma, vmf);
    
    PRINTD(DBG_MMAP, "Offsets %p %lx %p.", vmf->virtual_address, vma->vm_start, node_p->startOfPool_p );

    offset = ((unsigned long)vmf->virtual_address - vma->vm_start) + node_p->startOfPool_p;
    if (offset >= node_p->endOfPool_p)
    {
        PRINTD(DBG_NOTICE, "Address %p is beyond end of buffer %p.", offset, node_p->endOfPool_p );
        ret = VM_FAULT_SIGBUS;
    }
    else
    {
        PRINTD(DBG_MMAP, "Offset is %p.", offset );
        
        page = virt_to_page(offset);
        PRINTD(DBG_MMAP, "Page is %p.", page );
        get_page(page);
        vmf->page = page;
    }

    PRINTD(DBG_FUNC_CALLS, "ipacVmaFault returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacMmap
 * Inputs        : filp - Info associated with the user's file descriptor.
 *                 vma  - vma structure describing mapped area.
 *
 * Description   : Defines a non-cached mapping for the mmap()ed area requested
 *                 by the user.
 *
 *****************************************************************************/
static int ipacMmap( struct file *filp, struct vm_area_struct *vma )
{
    IpacDeviceNode     *node_p = filp->private_data;
    int                ret     = 0;
    
    PRINTD(DBG_FUNC_CALLS, "ipacMmap(%p, %p)", filp, vma);
    
    vma->vm_ops = &ipacNopageVmOps;
    vma->vm_flags |= VM_RESERVED | VM_IO;
    vma->vm_private_data = node_p;
#ifndef ALLOW_CACHED_USER_READS
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif
    ipacVmaOpen(vma);

    node_p->vm_start = (u8*)vma->vm_start;

    PRINTD(DBG_FUNC_CALLS, "ipacMmap returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacOpen
 * Inputs        : inode - inode associated with the device
 *                 filp  - user's file descriptor info
 *
 * Description   : The open() call handler for the device.
 *
 *****************************************************************************/
/*
 * Open a keystream device node. This uses the minor number of the device node
 * to find the channel to associate with. The device is marked as unconfigured
 * until the client has set the direction using an ioctl call
 */
static int ipacOpen( struct inode *inode, struct file  *filp )
{
    int               minor      = iminor( inode );
    unsigned          minor_base = MINOR( ipacDevice.devno );
    unsigned          channel    = minor - minor_base;
    IpacDeviceNode    *node_p    = NULL;
    int               contextId  = 0;
    IpacRequest       *buffer    = NULL;
    int               ret        = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacOpen(%p, %p)", inode, filp);
    
    if ( channel >= IPAC_MAX_DRIVER_NODES )
    {
        PRINTD(DBG_NOTICE, "Invalid device number %d is > %d", channel, IPAC_MAX_DRIVER_NODES);
        ret = -ENODEV;
    }
    else
    {
        node_p = &ipacNodes[ channel ];
    
        spin_lock_bh( &node_p->lock );
    
        if ( node_p->useCount != 0)
        {
            spin_unlock_bh( &node_p->lock );
            PRINTD(DBG_NOTICE, "Device number %d is already in use", channel);
            ret = -EBUSY;
        }
        else
        {
            node_p->useCount = 1;
            
            for (contextId=0; contextId<IPAC_NUM_CONTEXTS; ++contextId)
            {
                node_p->context[contextId].cipher_p    = NULL;
                node_p->context[contextId].oldCipher_p = NULL;
            }
        
            node_p->firstRequest_p = node_p->startOfPool_p;
            node_p->lastRequest_p  = node_p->startOfPool_p;
        
            node_p->requestsPending = 0;

            buffer = (IpacRequest*)(node_p->firstRequest_p);
            buffer->state = IPAC_REQUEST_FINAL;
        
            /* Odd channels are for downlink, even for uplink */    
            if (channel & 1)
            {
                node_p->direction = IPAC_DOWNLINK;
            }
            else
            {
                node_p->direction = IPAC_UPLINK;
            }
            
            spin_unlock_bh( &node_p->lock );
            
            ipacFlushRequestInfo(buffer);
            
            filp->private_data = node_p;
        }
    }

    PRINTD(DBG_FUNC_CALLS, "ipacOpen returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacRelease
 * Inputs        : inode - inode associated with the device
 *                 filp  - user's file descriptor info
 *
 * Description   : The close() call handler for the device. 
 *
 *****************************************************************************/
static int ipacRelease(struct inode  *inode,
                       struct file   *filp)
{
    IpacDeviceNode     *node_p   = filp->private_data;
    int                contextId = 0;
    int                ret       = 0;
    int                busy      = 0;
    int                retries   = 100; /* Allow at least 200 ms to complete */
    
    PRINTD(DBG_FUNC_CALLS, "ipacRelease(%p, %p)", inode, filp);
    
    spin_lock_bh( &node_p->lock );

    /* Wait till any last active IOCTL call has been completed and all the
     * active cipher requests for this node have completed.
     */
    node_p->releasingNode = 1;

    do
    {
        busy = 0;

        retries--;

        if (node_p->inIoctlCall)
        {
            busy = 1;
        }
        else
        {
            for (contextId=0; contextId<IPAC_NUM_CONTEXTS; ++contextId)
            {
                IpacContext   *context_p = &(node_p->context[contextId]);
                
                if (context_p->oldCipherActiveRequests || context_p->cipherActiveRequests)
                {
                    /* Have to wait for at least one request to complete */
                    busy = 1;
                    break;
                }
            }
        }

        if (busy)
        {
            /* Wait for activity to stop.  This shouldn't take long. */
            spin_unlock_bh( &node_p->lock );
            msleep(2);
            spin_lock_bh( &node_p->lock );
        }
        
    } while (busy && (retries > 0));

    if (busy)
    {
        ret = -EBUSY;
    }
    else
    {
        /* All requests are complete, and new IOCTL calls are being bounced,
         * so there can be no more requests and it's safe to release all the
         * cipher contexts.
         */
        for (contextId=0; contextId<IPAC_NUM_CONTEXTS; ++contextId)
        {
            IpacContext  *context_p = &(node_p->context[contextId]);
        
            if (context_p->cipher_p != NULL)
            {
                PRINTD(DBG_CIPHER, "Freeing current cipher %p in release", context_p->cipher_p);
                crypto_free_ablkcipher( context_p->cipher_p );
                context_p->cipher_p    = NULL;
            }
        
            if (context_p->oldCipher_p != NULL)
            {
                PRINTD(DBG_CIPHER, "Freeing old cipher %p in release", context_p->oldCipher_p);
                crypto_free_ablkcipher( context_p->oldCipher_p );
                context_p->oldCipher_p = NULL;
            }
        }

        /* Allow this node to be reopened. */
        node_p->useCount = 0;
    }

    node_p->releasingNode = 0;
    spin_unlock_bh( &node_p->lock );

    PRINTD(DBG_FUNC_CALLS, "ipacRelease returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipac_init
 * Inputs        : -
 *
 * Description   : Called when the module is loaded.
 *
 *****************************************************************************/
static int ipac_init( void )
{
    IpacDeviceNode  *node_p = NULL;
    IpacRequest     *buffer = NULL;
    int             ret     = -ENOMEM;
    int             i;

    PRINTD(DBG_FUNC_CALLS, "ipac_init()");

    for (i=0; i<IPAC_MAX_DRIVER_NODES; ++i)
    {
        memset(&ipacNodes[i], 0, sizeof(IpacDeviceNode));

        spin_lock_init( &ipacNodes[i].lock );
        strcpy(ipacNodes[i].name, "reqx");
        ipacNodes[i].name[3] = '0' + i;
    }
    
    ipacZeroedPage = ( void * )get_zeroed_page( GFP_KERNEL );
    if ( !ipacZeroedPage )
    {
        PRINTD(DBG_ERROR, KERN_INFO "failed to allocate zeroed page" );
        goto out;
    }

    cdev_init( &ipacDevice.cdev, &ipacFileOps );
    ipacDevice.cdev.owner = THIS_MODULE;
    ipacDevice.cdev.ops   = &ipacFileOps;

    ipacDevice.devno = MKDEV(IPAC_DEVICE_MAJOR, 0);

    ret = register_chrdev_region( ipacDevice.devno, IPAC_MAX_DRIVER_NODES, "ipa_crypto" );
    if ( ret )
    {
        PRINTD(DBG_ERROR, "failed to allocate dev node range" );
        goto alloc_failed;
    }

    ipacDevice.sysfs_class = class_create( THIS_MODULE, "ipa_crypto" );
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        ipacNodes[i].dev_p =  device_create( ipacDevice.sysfs_class, NULL,
                                             MKDEV( MAJOR( ipacDevice.devno ),
                                             MINOR( ipacDevice.devno ) + i ), NULL,
                                             "ipa_crypto%u", i );
        if (ipacNodes[i].dev_p == NULL)
        {
            PRINTD(DBG_ERROR, "failed to create device %d", ret );
            ret = -ENOMEM;
            goto device_creation_failed;
        }

        ipacNodes[i].dev_p->coherent_dma_mask = 0xFFFFFFFF;
        ipacNodes[i].dev_p->dma_mask = &(ipacNodes[i].dev_p->coherent_dma_mask);
    }


    /* Create a request cache for each node */
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        node_p = &ipacNodes[i];
        
        node_p->requestCache_p = kmem_cache_create( node_p->name, sizeof(IpacCipherRequest), 0, 0, NULL );
        if (node_p->requestCache_p == NULL)
        {
            ret = -ENOMEM;
            PRINTD(DBG_ERROR, "failed to allocate memory for pool" );
            goto dma_cache_alloc_failed;
        }
    }
    
    ret = cdev_add( &ipacDevice.cdev, ipacDevice.devno, IPAC_MAX_DRIVER_NODES );
    if ( ret )
    {
        PRINTD(DBG_ERROR, "failed to add cdev" );
        goto cdev_add_failed;
    }
    
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        node_p = &ipacNodes[i];
        
        node_p->startOfPool_p = kmalloc(IPAC_KEYSTREAM_BUFFER_SIZE, GFP_KERNEL | __GFP_DMA);
        if ( node_p->startOfPool_p == NULL )
        {
            ret = -ENOMEM;
            PRINTD(DBG_ERROR, "failed to allocate memory for pool" );
            goto dma_alloc_failed;
        }

        node_p->endOfPool_p    = node_p->startOfPool_p + IPAC_KEYSTREAM_BUFFER_SIZE;
        node_p->firstRequest_p = node_p->startOfPool_p;
        node_p->lastRequest_p  = node_p->startOfPool_p;
        
        node_p->requestsPending = 0;

        buffer = (IpacRequest*)(node_p->firstRequest_p);
        buffer->state = IPAC_REQUEST_FINAL;
        ipacFlushRequestInfo(buffer);
    }

    ret = 0;    
    goto out;

dma_alloc_failed:
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        node_p = &ipacNodes[i];
        
        if (node_p->startOfPool_p)
        {
            kfree(node_p->startOfPool_p);
            node_p->startOfPool_p = NULL;
        }
    }
    cdev_del( &ipacDevice.cdev );
dma_cache_alloc_failed:
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        node_p = &ipacNodes[i];
        
        if (node_p->requestCache_p)
        {
            kmem_cache_destroy( node_p->requestCache_p );
            node_p->requestCache_p = NULL;
        }
    }
device_creation_failed:
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        if (ipacNodes[i].dev_p)
        {
            device_destroy( ipacDevice.sysfs_class,
                            MKDEV( MAJOR( ipacDevice.devno ),
                            MINOR( ipacDevice.devno ) + i ) );
        }
    }
    class_destroy( ipacDevice.sysfs_class );
cdev_add_failed:
    unregister_chrdev_region( ipacDevice.devno, IPAC_MAX_DRIVER_NODES );
alloc_failed:
    free_page( ( unsigned long )ipacZeroedPage );
out:
    
    PRINTD(DBG_FUNC_CALLS, "ipac_init returned %d", ret);

    return ret;
}

/******************************************************************************
 * Function Name : ipac_exit
 * Inputs        : -
 *
 * Description   : Cleanup when the module is unloaded
 *
 *****************************************************************************/
static void ipac_exit( void )
{
    IpacDeviceNode    *node_p = NULL;
    int               i;

    PRINTD(DBG_FUNC_CALLS, "ipac_exit()");

    /* TODO Will have to clean up all allocated memory before unloading the driver */

    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        node_p = &ipacNodes[i];
        
        if (node_p->startOfPool_p)
        {
            kfree(node_p->startOfPool_p);
            node_p->startOfPool_p = NULL;
        }

        if (node_p->requestCache_p)
        {
            kmem_cache_destroy( node_p->requestCache_p );
            node_p->requestCache_p = NULL;
        }
    }
    
    cdev_del( &ipacDevice.cdev );
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        device_destroy( ipacDevice.sysfs_class,
                        MKDEV( MAJOR( ipacDevice.devno ),
                               MINOR( ipacDevice.devno ) + i ) );
    }
    class_destroy( ipacDevice.sysfs_class );

    unregister_chrdev_region( ipacDevice.devno, IPAC_MAX_DRIVER_NODES );
    free_page( ( unsigned long )ipacZeroedPage );
    
    PRINTD(DBG_FUNC_CALLS, "ipac_exit out");
}

module_init( ipac_init );
module_exit( ipac_exit );

MODULE_AUTHOR("ip.access");
MODULE_DESCRIPTION("ipacrypto user interface driver for f8(kasumi)");
MODULE_LICENSE( "GPL" );
