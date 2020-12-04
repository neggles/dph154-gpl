#include <linux/cdev.h>
#include <linux/configfs.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

/*! The number of keystream generator device nodes to create. */
#define MAX_KS_NODES    96

#define MAX_IV_LEN      32

/*
 * This keystream generator provides a configfs based interface to the kernel
 * crypto API to use block ciphers to create a stream cipher and export the
 * keystream into userspace.
 *
 * To configure a keystream generator, create a new directory in
 * /config/keystream. For example:
 *
 *  mkdir /config/keystream/ksgen
 *
 * The directory will be populated with "alg, channel, iv, key and pool_size"
 * attributes. To configure the instance:
 *
 *  1. Write the algorithm type into "alg" with no trailing newline. e.g.
 *      echo -n "cbc(aes)" > /config/keystream/ksgen/alg
 *  2. Write the keystream size (in bits).
 *      echo -n "80" > /config/keystream/ksgen/ks_sz
 *  3. Set the number of keystreams to maintain.
 *      echo -n "16" > /config/keystream/ksgen/pool_size
 *  4. Write the key.
 *      echo -n "deadbeefdeadbeefdeadbeefdeadbeef" > \
 *          /config/keystream/ksgen/key
 *  5. Read the channel number and open the corresponding device node and
 *     a series of IVs for each keystream to be generated.
 *      dd if=iv1.bin of=/dev/keystream0 bs=16 count=1
 *      dd if=iv2.bin of=/dev/keystream0 bs=16 count=1
 *  6. Read the keystreams.
 *      dd if=/dev/keystream0 of=ks1.bin bs=80 count=1
 *      dd if=/dev/keystream0 of=ks2.bin bs=80 count=2
 * The number in the "channel" attribute corresponds to the /dev/keystreamX
 * device node. Reading from this device file will read the keystream into
 * userspace. Writing an IV will trigger a new keystream to be generated.
 */

static struct
{
    struct cdev     cdev;
    dev_t           devno;
    struct class    *sysfs_class;
} ks_dev;

static int
ks_open( struct inode *inode,
         struct file *filp );

static int
ks_release( struct inode *inode,
            struct file *filp );

static ssize_t
ks_read( struct file *filp,
         char __user *buf,
         size_t len,
         loff_t *offset );

static ssize_t
ks_write( struct file *filp,
          const char __user *buf,
          size_t len,
          loff_t *offset );

static loff_t
ks_llseek( struct file *filp,
           loff_t offs,
           int origin );

static unsigned int
ks_poll( struct file *filp,
         struct poll_table_struct *pollt );

static struct file_operations ks_fops = {
    .owner              = THIS_MODULE,
    .open               = ks_open,
    .release            = ks_release,
    .read               = ks_read,
    .write              = ks_write,
    .llseek             = ks_llseek,
    .poll		= ks_poll,
};

/* Request structure used to store temporary per-request information. We
 * create a kmem_cache for these to reduce the impact of the dynamic
 * allocation. We will need to allocate one of these for every keystream
 * request that we get and return it once the encryption has finished. We can
 * use the scatterlist entry to get the ks_req from the async crypto
 * completion function.
 */
struct ks_req
{
    struct ablkcipher_request   *areq;
    u8                          iv[ MAX_IV_LEN ];
    struct scatterlist          src;
    struct scatterlist          dst;
};

/*
 * The keystream generator structure. This stores the state of an individual
 * generator and maintains a buffer of keystream. Each generator is configured
 * by a configfs item and uses the kernel crypto API to maintain the
 * keystream.
 *
 * Once configured, the buffer will be filled with keystream which may be read
 * through an associated device node.
 */
struct ks_generator
{
    struct config_item          item;
    spinlock_t                  lock;
    unsigned                    minor;
    enum {
        KS_UNCONFIGURED,
        KS_CONFIGURED,
        KS_ACTIVE,
        KS_ENCRYPTING,
    }                           state;
    /* The number of users. No need for an atomic as we protect this with the
     * spinlock whenever we need to access it. */
    unsigned                    use_count;

    /* Cache for crypto requests. */
    struct kmem_cache           *req_cache;

    /* Wait queue for anything waiting for encryption to finish. */
    wait_queue_head_t           waitq;

    /* The keystream FIFO. */
    u8                          *buf;
    size_t                      buf_sz;

    /* If the user reads a partial keystream, then store the rest here. This
     * simplifies the reading logic to know how to resume next time. */
    u8                          *remainder;
    size_t                      remainder_bytes;

    /*
     * FIFO status: rptr is where we should commence reading from. wptr is
     * where the next keystream should be generated.
     */
    unsigned long               rptr;
    unsigned long               wptr;
    unsigned long               tail;
    unsigned long               used;

    /* The size of each keystream to generate. */
    unsigned                    ks_sz;
    /* The number of padding bytes to make keystreams cache aligned. */
    unsigned                    padding;
    /* The maximum number of keystreams to generate. */
    unsigned                    pool_sz;

    /* Crypto structures. */
    ssize_t                     key_len;
    u8                          *key;
    char                        alg[ CRYPTO_MAX_ALG_NAME ];
    struct crypto_ablkcipher    *cipher;
    /* The number of encryption requests in progress. Increment this when we
     * dispatch a new request and decrement it on completion. Once this
     * reaches 0 we can set the state back to KS_ACTIVE. */
    unsigned                    in_flight;

    struct semaphore		sem;
};

/* The registered keystream generators. The index in the array signifies the
 * relative minor number for the device node. */
static struct ks_generator *ks_slots[ MAX_KS_NODES ];

/* Zeroed page used for generating keystreams. */
static void *zeroed_page;

static void
remove_ks_generator( struct ks_generator *ks );

/*
 * Non-locking variant of ks_cipher_complete(). Used when the keystream
 * generator is already locked. This is useful for synchronous requests.
 */
static void
__ks_cipher_complete( struct crypto_async_request *req,
                      int err )
{
    struct ks_generator *ks = req->data;
    struct ablkcipher_request *ablk_req = ablkcipher_request_cast( req );
    struct ks_req *kreq = container_of( ablk_req->src, struct ks_req, src );
    BUG_ON( !ks );

    if ( !err )
    {
        ks->used += ablk_req->nbytes;
        /* If this is the last request in flight to complete then we can set
         * the state back to KS_ACTIVE. This state change will allow us to
         * rekey or reconfigure. */
        if ( 0 == --ks->in_flight )
            ks->state = KS_ACTIVE;
        /* Copy the encryption of the last block for the IV chaining for the
         * next encryption. */
        wake_up_interruptible( &ks->waitq );
    }
    else
        printk( KERN_WARNING "encryption on keystream channel %u failed "
                "with code %d\n", ks->minor, err );

    ablkcipher_request_free( kreq->areq );
    kreq->areq = NULL;
    kmem_cache_free( ks->req_cache, kreq );
}

/*
 * Handle a crypto request completion. This will be called when the encryption
 * has been completed and the data in the buffer is the keystream. This is a
 * locking variant of __ks_cipher_complete suitable for asynchronous
 * completion functions.
 */
static void
ks_cipher_complete( struct crypto_async_request *req,
                    int err )
{
    struct ks_generator *ks = req->data;

    BUG_ON( !ks );
    spin_lock_bh( &ks->lock );
    __ks_cipher_complete( req, err );
    spin_unlock_bh( &ks->lock );
}

/*
 * Open a keystream device node. This uses the minor number of the device node
 * to find the channel to associate with. If the channel is not yet configured
 * then reject the request.
 */
static int
ks_open( struct inode *inode,
         struct file *filp )
{
    int minor = iminor( inode );
    unsigned minor_base = MINOR( ks_dev.devno );
    unsigned channel = minor - minor_base;
    struct ks_generator *ks;
    int ret = 0;

    /* Make sure that the keystream generator has been configured. If it
     * hasn't then there is no sense in opening the device node. */
    if ( channel >= MAX_KS_NODES || !ks_slots[ channel ] )
        return -ENODEV;

    ks = ks_slots[ channel ];

    spin_lock_bh( &ks->lock );
    filp->private_data = config_item_get( &ks->item );

    if ( KS_UNCONFIGURED == ks->state )
    {
        ret = -EBADF;
        config_item_put( &ks->item );
        goto out;
    }
    
    ks->state = KS_ACTIVE;
    ++ks->use_count;

    sema_init( &ks->sem, 1 );

out:
    spin_unlock_bh( &ks->lock );

    return ret;
}

static int
ks_release( struct inode *inode,
            struct file *filp )
{
    struct ks_generator *ks = filp->private_data;

    spin_lock_bh( &ks->lock );
    config_item_put( &ks->item );
    /* If we are the last user then put the generator back into the configured
     * state. Someone else may want to use it again. */
    if ( 0 == --ks->use_count )
        ks->state = KS_CONFIGURED;
    spin_unlock_bh( &ks->lock );

    return 0;
}

/*
 * Write to the device node. The write function takes an array of IV's to use
 * for keystream generation and for each IV triggers a new keystream
 * generation which can be read back from the device node once the encryption
 * has completed.
 *
 * Unlike the configfs attributes we expect the IV to be passed in as a binary
 * array. We do this as the keystream will be read as binary for the efficient
 * XOR operation and the same application will be generating the IVs.
 */
static ssize_t
ks_write( struct file *filp,
          const char __user *buf,
          size_t len,
          loff_t *offp )
{
    ssize_t space;
    size_t clen;
    int ret = 0;
    struct ks_generator *ks = filp->private_data;
    ssize_t written = 0;
    struct ks_req *req;
    unsigned ivsize;

    if ( down_interruptible( &ks->sem ) )
        return -ERESTARTSYS;

    ret = -EBUSY;
    if ( ks->state == KS_UNCONFIGURED )
        goto out;

    ivsize = crypto_ablkcipher_ivsize( ks->cipher );

    /* For each IV in the input buffer trigger a new ciphering request. If the
     * buffer size is not a multiple of the IV size then process as many as we
     * can. */
    while ( written < len && ( len - written ) >= ivsize )
    {
        /* Allocate a request. If the allocation fails then we return an error
         * even though some of the requests may have started succesfully.
         * Unfortunate if this happens but not a lot we can do. */
        ret = -ENOMEM;
        req = kmem_cache_alloc( ks->req_cache, GFP_ATOMIC );
        if ( !req )
            goto out;
        req->areq = ablkcipher_request_alloc( ks->cipher, GFP_ATOMIC );
        if ( !req )
        {
            kmem_cache_free( ks->req_cache, req );
            goto out;
        }

        /* Get the IV for the request. */
        ret = -EFAULT;
        ret = copy_from_user( req->iv, buf + written, ivsize );
        if ( ret )
        {
            ablkcipher_request_free( req->areq );
            kmem_cache_free( ks->req_cache, req );
            goto out;
        }

        spin_lock_bh( &ks->lock );
        /* Writing is easy. Each IV generates a single keystream and as the
         * buffer size is a multiple of the keystream size we don't have to
         * worry about wrapping the fifo. */
        if ( ks->wptr >= ks->rptr )
            space = ( ks->buf_sz - ks->wptr ) + ks->rptr;
        else
            space = ks->rptr - ks->wptr;
        /*
         * We leave a 1 keystream gap between to read pointer and write
         * pointer so we can differentiate between FIFO full and FIFO empty.
         * (Note: ks->used isn't updated until the ciphering completes so we'd
         * need to maintain another status variable to avoid this.
         */
        space -= ( ks->ks_sz + ks->padding );
        if ( space < ks->ks_sz + ks->padding )
        {
            spin_unlock_bh( &ks->lock );
            ablkcipher_request_free( req->areq );
            kmem_cache_free( ks->req_cache, req );
            goto out;
        }
        clen = ks->ks_sz + ks->padding;

        sg_init_table( &req->src, 1 );
        sg_init_table( &req->dst, 1 );
        sg_set_buf( &req->src, zeroed_page, clen );
        sg_set_buf( &req->dst, ks->buf + ks->wptr, clen );
        ablkcipher_request_set_callback( req->areq, CRYPTO_TFM_REQ_MAY_BACKLOG,
                                         ks_cipher_complete, ks );
        ablkcipher_request_set_crypt( req->areq, &req->src, &req->dst,
                                      ks->ks_sz, req->iv );

        ret = crypto_ablkcipher_encrypt( req->areq );
        if ( !ret )
        {
            /* We have completed synchronously. Call the completion function.
             * This will free the request for us. */
            __ks_cipher_complete( &req->areq->base, 0 );
        }
        else if ( -EINPROGRESS == ret )
        {
            ks->state = KS_ENCRYPTING;
            ++ks->in_flight;
            ret = 0;
        }
        else
        {
            printk( KERN_WARNING "keystream: encryption failed with %d\n",
                    ret );
            spin_unlock_bh( &ks->lock );
            goto out;
        }

        ks->wptr = ( ks->wptr + ks->ks_sz + ks->padding );
        if ( ks->wptr > ks->buf_sz )
            ks->wptr -= ks->buf_sz;

        written += ivsize;
        spin_unlock_bh( &ks->lock );
    }

out:
    up( &ks->sem );
    return ret ?: written;
}

static unsigned int
ks_poll( struct file *filp,
         struct poll_table_struct *pollt )
{
    unsigned int ret = 0;
    struct ks_generator *ks = filp->private_data;

    poll_wait( filp, &ks->waitq, pollt );

    if ( ks->used != 0 )
        ret |= POLLIN;
    if ( ks->buf_sz != ks->used )
        ret |= POLLOUT;

    return ret;
}

/*
 * Read from the keystream generator. Currently we only support blocking
 * reads. This will put as much keystream as possible into the user buffer. If
 * the user doesn't need all of the keystream then they must either read it
 * all and discard the excess or explicitly seek to the next keystream.
 *
 * The user does not have to read a multiple of the keystream length so it is
 * possible that the read may wrap the fifo and require 2 separate reads.
 */
static ssize_t
ks_read( struct file *filp,
         char __user *buf,
         size_t len,
         loff_t *offset )
{
    struct ks_generator *ks = filp->private_data;
    int ret;
    unsigned long read;
    size_t to_read;

    if ( down_interruptible( &ks->sem ) )
        return -ERESTARTSYS;

    if ( wait_event_interruptible( ks->waitq, 0 != ks->used ) )
    {
        up( &ks->sem );
	return -ERESTARTSYS;
    }

    /* Try and read any remainder bytes first. */
    to_read = min( ks->remainder_bytes, len );
    if ( copy_to_user( buf, ks->remainder, to_read ) )
    {
        ret = -EFAULT;
        goto out;
    }

    /*
     * If we didn't read all of the remainder, then don't read new keystreams.
     */
    if ( ks->remainder_bytes )
    {
        ret = to_read;
        goto out;
    }

    read = to_read;
    to_read = min( ks->used, len - read );

    while ( to_read )
    {
        size_t to_copy = min( to_read, ks->ks_sz );
        if ( copy_to_user( buf + read, ks->buf + ks->rptr, to_copy ) )
        {
            ret = -EFAULT;
            goto out;
        }

        if ( ks->ks_sz != to_copy )
        {
            /*
             * Partial copy. We need to put the rest into the remainder
             * buffer.
             */
            memmove( ks->remainder, ks->buf + ks->rptr + to_copy,
                     ks->ks_sz - to_copy );
            ks->remainder_bytes = ks->ks_sz - to_copy;
        }

        read += to_copy;
        ks->rptr = ks->rptr + ks->ks_sz + ks->padding;
        if ( ks->rptr > ks->buf_sz )
            ks->rptr -= ks->buf_sz;

        to_read -= to_copy;

        ks->used -= ks->ks_sz;
    }

    ret = read;

out:
    up( &ks->sem );
    return ret;
}

static loff_t
ks_llseek( struct file *filp,
           loff_t offs,
           int origin )
{
    struct ks_generator *ks = filp->private_data;
    int ret = 0;
    /* Don't allow offsets bigger than 32 bits. We'll never have 4GB of
     * keystream so it would be pointless to support this. We also don't allow
     * seeking backwards as we have discarded the data. */
    unsigned long offs32 = ( unsigned long )offs;
    unsigned long skipped = 0;
    unsigned whole_keystreams;
    unsigned partial_bytes;

    if ( down_interruptible( &ks->sem ) )
        return -ERESTARTSYS;

    if ( offs >= ( 1LL << 32 ) || offs < 0 )
    {
        up( &ks->sem );
        return -EOVERFLOW;
    }

    spin_lock_bh( &ks->lock );
    offs32 = min( offs32, ks->used );
    switch ( origin )
    {
        case SEEK_CUR:
            /*
             * The offset is in keystream bytes. We need to take the remainder
             * bytes and the padding into account when we seek.
             */
            if ( ks->remainder_bytes )
            {
                skipped = min( offs32, ( unsigned long )ks->remainder_bytes );
                ks->remainder_bytes -= skipped;
                ks->used -= skipped;
            }

            /* If there is still some remainder left, then we're done. */
            if ( ks->remainder_bytes )
                goto out;

            /*
             * Now skip the rest of the whole keystreams and copy the
             * remainder if we need to.
             */
            whole_keystreams = ( offs32 - skipped ) / ks->ks_sz;
            partial_bytes = ( offs32 - skipped ) % ks->ks_sz;

            /* Skip past the whole keystreams. */
            ks->rptr = ks->rptr + ( whole_keystreams *
                    ( ks->ks_sz + ks->padding ) );
            if ( ks->rptr > ks->buf_sz )
                ks->rptr -= ks->buf_sz;

            /* Copy the excess and move to the next keystream. */
            memcpy( ks->remainder, ks->buf + ks->rptr + partial_bytes,
                    ks->ks_sz - partial_bytes );
            ks->remainder_bytes = ks->ks_sz - partial_bytes;

            ks->rptr = ks->rptr + ( ks->ks_sz + ks->padding );
            if ( ks->rptr > ks->buf_sz )
                ks->rptr -= ks->buf_sz;

            ks->used -= offs32;
            break;

        default:
            ret = -EINVAL;
            break;
    }

out:
    spin_unlock_bh( &ks->lock );

    up( &ks->sem );
    return ret ?: ( filp->f_pos += offs );
}

static inline struct ks_generator *
to_ks_generator( struct config_item *item )
{
    return item ? container_of( item, struct ks_generator, item ) : NULL;
}

CONFIGFS_ATTR_STRUCT( ks_generator );

#define KS_GENERATOR_ATTR( _name, _mode, _show, _store )    \
    struct ks_generator_attribute ks_attr_##_name =         \
        __CONFIGFS_ATTR( _name, _mode, _show, _store )

#define KS_GENERATOR_ATTR_RO( _name, _show )    \
    struct ks_generator_attribute ks_attr_##_name =         \
        __CONFIGFS_ATTR_RO( _name, _show )

static ssize_t
ks_generator_alg_show( struct ks_generator *ks,
                       char *page )
{
    return sprintf( page, "%s\n", ks->alg );
}

/* Store the algorithm. This will trigger the existing setup to be flushed and
 * to allocate the new transform with a fresh key. */
static ssize_t
ks_generator_alg_store( struct ks_generator *ks,
                        const char *page,
                        size_t len )
{
    int ret;

    spin_lock_bh( &ks->lock );

    /* If the keystream is in use, then don't allow us to be reconfigured. */
    ret = -EBUSY;
    if ( KS_ACTIVE == ks->state || KS_ENCRYPTING == ks->state )
    {
        spin_unlock_bh( &ks->lock );
        goto out;
    }

    if ( ks->cipher )
    {
        crypto_free_ablkcipher( ks->cipher );
        ks->cipher = NULL;
    }

    if ( ks->key )
        kfree( ks->key );

    ks->state = KS_UNCONFIGURED;

    spin_unlock_bh( &ks->lock );

    /* Allocate the new transform. */
    strncpy( ks->alg, page, sizeof( ks->alg ) - 1 );
    ks->cipher = crypto_alloc_ablkcipher( ks->alg, 0, 0);
    if ( IS_ERR( ks->cipher ) )
    {
        /* Couldn't allocate the cipher. Clear the buffer to make sure that
         * there is no confusion. */
        ks->alg[ 0 ] = '\0';
        ret = PTR_ERR( ks->cipher );
        ks->cipher = NULL;
        goto out;
    }

    /* Reserve space for the key and IV. */
    ret = -ENOMEM;
    ks->key = kzalloc( ks->cipher->base.__crt_alg->cra_ablkcipher.max_keysize,
                       GFP_KERNEL );
    if ( !ks->key )
        goto key_fail;
    ks->key_len = -1;

    ks->buf_sz = crypto_ablkcipher_blocksize( ks->cipher );
    ks->buf = kzalloc( ks->buf_sz, GFP_KERNEL | GFP_DMA );
    if ( !ks->buf )
        goto buf_fail;

    ret = len;
    goto out;

buf_fail:
    kfree( ks->key );
    ks->key = NULL;
key_fail:
    crypto_free_ablkcipher( ks->cipher );
out:
    return ret;
}
KS_GENERATOR_ATTR( alg, 0600, ks_generator_alg_show, ks_generator_alg_store );

static ssize_t
ks_generator_key_show( struct ks_generator *ks,
                       char *page )
{
    unsigned i = 0;
    char *pos = page;

    if ( NULL == ks->cipher )
        return -EINVAL;

    spin_lock_bh( &ks->lock );
    if ( ks->key_len < 0 )
    {
        spin_unlock_bh( &ks->lock );
        return -EINVAL;
    }

    for ( ; i < ks->key_len; ++i )
        pos += sprintf( pos, "%02x", ks->key[ i ] );
    *pos++ = '\n';
    *pos++ = '\0';

    spin_unlock_bh( &ks->lock );

    return pos - page;
}

static ssize_t
ks_generator_key_store( struct ks_generator *ks,
                        const char *page,
                        size_t len )
{
    const char *pos = page;
    unsigned i = 0;
    unsigned int val;
    int ret;
    /* Temporary buffer for key extraction. */
    char tmpbuf[ 3 ] = { 0, 0, 0 };

    /* Key must be a byte multiple in ASCII hex. */
    if ( len % 2 )
        return -EINVAL;

    if ( NULL == ks->cipher )
        return -EINVAL;

    /* Wait for the encryption to finish. After waking we need to make sure
     * that another process hasn't started encryption so we need to retest the
     * state and possible sleep again. */
    if ( KS_ENCRYPTING == ks->state )
    {
again:
        wait_event_interruptible( ks->waitq,
                                  !( KS_ENCRYPTING == ks->state ) );
        spin_lock_bh( &ks->lock );
        if ( KS_ENCRYPTING == ks->state )
        {
            spin_unlock_bh( &ks->lock );
            goto again;
        }
    }
    else
        spin_lock_bh( &ks->lock );

    for ( ; i < len / 2; ++i )
    {
        tmpbuf[ 0 ] = *pos;
        tmpbuf[ 1 ] = *( pos + 1 );
        sscanf( tmpbuf, "%02x", &val );
        ks->key[ i ] = val;
        pos += 2;
    }

    ks->key_len = len / 2;
    ret = crypto_ablkcipher_setkey( ks->cipher, ks->key, ks->key_len ) ?: len;

    /* Flush the buffer. We just need to set the read and write pointers to
     * zero so that the FIFO appears empty. */
    ks->wptr = ks->rptr = ks->remainder_bytes = 0;

    /* If we have set the key then we can now mark this as configured. If the
     * setkey fails, then don't allow any more keystream generation until we
     * put a new good key in. */
    if ( ret == len )
        ks->state = KS_CONFIGURED;
    else
        ks->state = KS_UNCONFIGURED;

    spin_unlock_bh( &ks->lock );

    return ret;
}
KS_GENERATOR_ATTR( key, 0600, ks_generator_key_show, ks_generator_key_store );

static ssize_t
ks_generator_ks_sz_show( struct ks_generator *ks,
                         char *page )
{
    spin_lock_bh( &ks->lock );
    sprintf( page, "%u\n", ks->ks_sz );
    spin_unlock_bh( &ks->lock );

    return strlen( page );
}

static ssize_t
ks_generator_ks_sz_store( struct ks_generator *ks,
                          const char *page,
                          size_t len )
{
    int ret;

    /* Wait for the encryption to finish. */
    if ( KS_ENCRYPTING == ks->state )
        return -EBUSY;

    /* Can't configure until we know the block cipher size. */
    if ( !ks->cipher )
        return -EINVAL;

    ks->ks_sz = simple_strtoul( page, NULL, 0 );
    ks->padding = roundup_pow_of_two( ks->ks_sz ) - ks->ks_sz;
    /* The keystream size must be a multiple of the block cipher size. */
    if ( ks->ks_sz % crypto_ablkcipher_blocksize( ks->cipher ) )
        return -EINVAL;

    spin_lock_bh( &ks->lock );

    if ( ks->buf )
    {
        kfree( ks->buf );
        ks->buf = NULL;
        ks->buf_sz = 0;
    }

    /* Allocate the buffer for storing the keystream. Make sure that the
     * buffer is a multiple of the cipher block size and that we have at least
     * one block. */
    ks->buf_sz = ( ks->ks_sz + ks->padding ) * ks->pool_sz;
    if ( !ks->buf_sz )
        ks->buf_sz =
            ( 2 * ( crypto_ablkcipher_blocksize( ks->cipher ) + ks->padding ) );
    ks->buf = kmalloc( ks->buf_sz, GFP_ATOMIC | GFP_DMA );

    if ( !ks->buf )
    {
        ks->buf_sz = 0;
        ret = -ENOMEM;
    }
    else
        ret = len;

    ks->remainder = kmalloc( ks->ks_sz, GFP_ATOMIC );
    if ( !ks->remainder )
    {
        kfree( ks->buf );
        ks->buf = NULL;
        ks->buf_sz = 0;
        ret = -ENOMEM;
    }
    ks->remainder_bytes = 0;

    spin_unlock_bh( &ks->lock );

    return ret;
}
KS_GENERATOR_ATTR( ks_size, 0600, ks_generator_ks_sz_show,
                   ks_generator_ks_sz_store );

static ssize_t
ks_generator_pool_sz_show( struct ks_generator *ks,
                           char *page )
{
    spin_lock_bh( &ks->lock );
    sprintf( page, "%u\n", ks->pool_sz );
    spin_unlock_bh( &ks->lock );

    return strlen( page );
}

static ssize_t
ks_generator_pool_sz_store( struct ks_generator *ks,
                          const char *page,
                          size_t len )
{
    int ret;

    /* Wait for the encryption to finish. */
    if ( KS_ENCRYPTING == ks->state )
        return -EBUSY;

    /* Can't configure until we know the block cipher size. */
    if ( !ks->cipher )
        return -EINVAL;

    spin_lock_bh( &ks->lock );

    if ( ks->buf )
    {
        kfree( ks->buf );
        ks->buf = NULL;
        ks->buf_sz = 0;
    }

    /* Allocate the buffer for storing the keystream. Make sure that the
     * buffer is a multiple of the cipher block size and that we have at least
     * one block. */
    ks->pool_sz = simple_strtoul( page, NULL, 0 );
    ks->buf_sz = ( ks->ks_sz + ks->padding ) * ks->pool_sz;
    if ( !ks->buf_sz )
        ks->buf_sz =
            ( 2 * ( crypto_ablkcipher_blocksize( ks->cipher ) + ks->padding ) );
    ks->buf = kmalloc( ks->buf_sz, GFP_ATOMIC | GFP_DMA );

    if ( !ks->buf )
    {
        ks->buf_sz = 0;
        ret = -ENOMEM;
    }
    else
        ret = len;

    spin_unlock_bh( &ks->lock );

    return ret;
}
KS_GENERATOR_ATTR( pool_size, 0600, ks_generator_pool_sz_show,
                   ks_generator_pool_sz_store );

static ssize_t
ks_generator_channel_show( struct ks_generator *ks,
                           char *page )
{
    return sprintf( page, "%u\n", ks->minor );
}
KS_GENERATOR_ATTR_RO( channel, ks_generator_channel_show );

static struct configfs_attribute *ks_generator_attrs[] = {
    &ks_attr_alg.attr,
    &ks_attr_key.attr,
    &ks_attr_ks_size.attr,
    &ks_attr_pool_size.attr,
    &ks_attr_channel.attr,
    NULL,
};

CONFIGFS_ATTR_OPS( ks_generator );

static void
ks_generator_release( struct config_item *item )
{
    struct ks_generator *ks = to_ks_generator( item );

    /* Wait for the encryption to finish. If we are releasing then we can't
     * have any users left so we don't need to lock but we do need to make
     * sure that any requests in flight have finished. */
    if ( KS_ENCRYPTING == ks->state )
        wait_event_interruptible( ks->waitq,
                                  !( KS_ENCRYPTING == ks->state ) );

    if ( ks->cipher )
    {
        crypto_free_ablkcipher( ks->cipher );
        ks->cipher = NULL;
    }

    remove_ks_generator( ks );

    if ( ks->key )
        kfree( ks->key );
    if ( ks->buf )
        kfree( ks->buf );

    kmem_cache_destroy( ks->req_cache );

    kfree( ks );
}

static struct configfs_item_operations ks_generator_item_ops = {
    .show_attribute     = ks_generator_attr_show,
    .store_attribute    = ks_generator_attr_store,
    .release            = ks_generator_release,
};

static struct config_item_type ks_generator_type = {
    .ct_item_ops        = &ks_generator_item_ops,
    .ct_attrs           = ks_generator_attrs,
    .ct_owner           = THIS_MODULE,
};

struct ks_generator_group {
    struct config_group     group;
};

static inline struct ks_generator_group *
to_ks_generator_group( struct config_item *item )
{
    return item ? container_of( to_config_group( item ),
                                struct ks_generator_group, group ) : NULL;
}

static struct config_item *
ks_generator_make_item( struct config_group *group,
                        const char *name );

static struct configfs_group_operations ks_generator_group_ops = {
    .make_item          = ks_generator_make_item,
};

static int
add_ks_generator( struct ks_generator *ks )
{
    unsigned i;
    int ret = -EBUSY;

    for ( i = 0; i < MAX_KS_NODES; ++i )
        if ( !ks_slots[ i ] )
        {
            ks_slots[ i ] = ks;
            ks->minor = i;
            ret = 0;
            break;
        }

    return ret;
}

static void
remove_ks_generator( struct ks_generator *ks )
{
    ks_slots[ ks->minor ] = NULL;
}

static struct config_item *
ks_generator_make_item( struct config_group *group,
                        const char *name )
{
    int ret;
    struct ks_generator *ks = kzalloc( sizeof( *ks ), GFP_KERNEL );
    if ( !ks )
        return ERR_PTR( -ENOMEM );
    config_item_init_type_name( &ks->item, name, &ks_generator_type );

    spin_lock_init( &ks->lock );

    ks->rptr        = ks->wptr = 0;
    ks->buf         = NULL;
    ks->key         = NULL;
    ks->alg[ 0 ]    = '\0';
    ks->cipher      = NULL;
    ks->used        = 0;
    ks->ks_sz       = 128;
    ks->pool_sz     = 32;

    ret = -ENOMEM;
    ks->req_cache   = kmem_cache_create( "ks_generator",
                                         sizeof( struct ks_req ),
                                         0, 0, NULL );
    if ( !ks->req_cache )
    {
        kfree( ks );
        return ERR_PTR( ret );
    }

    init_waitqueue_head( &ks->waitq );

    ret = add_ks_generator( ks );
    if ( ret )
    {
        kfree( ks );
        return ERR_PTR( ret );
    }
    return &ks->item;
}

static void
ks_generator_group_release( struct config_item *item )
{
    kfree( to_ks_generator_group( item ) );
}

static struct configfs_item_operations ks_group_item_ops = {
    .release            = ks_generator_group_release,
};

static struct config_item_type ks_group_type = {
    .ct_item_ops        = &ks_group_item_ops,
    .ct_group_ops       = &ks_generator_group_ops,
    .ct_owner           = THIS_MODULE,
};

static struct configfs_subsystem ks_generator_subsys = {
    .su_group = {
        .cg_item = {
            .ci_namebuf = "keystream",
            .ci_type    = &ks_group_type,
        },
    },
};

static int
ks_init( void )
{
    int ret;
    unsigned i;

    ret = -ENOMEM;
    zeroed_page = ( void * )get_zeroed_page( GFP_KERNEL );
    if ( !zeroed_page )
        goto out;

    cdev_init( &ks_dev.cdev, &ks_fops );
    ks_dev.cdev.owner = THIS_MODULE;
    ks_dev.cdev.ops = &ks_fops;

    ret = alloc_chrdev_region( &ks_dev.devno, 0, MAX_KS_NODES, "keystream" );
    if ( ret )
    {
        printk( KERN_INFO "failed to allocate dev node range\n" );
        goto out;
    }

    ks_dev.sysfs_class = class_create( THIS_MODULE, "keystream" );
    for ( i = 0; i < MAX_KS_NODES; ++i )
        device_create( ks_dev.sysfs_class, NULL,
                       MKDEV( MAJOR( ks_dev.devno ),
                              MINOR( ks_dev.devno ) + i ), NULL,
                       "keystream%u", i );

    ret = cdev_add( &ks_dev.cdev, ks_dev.devno, MAX_KS_NODES );
    if ( ret )
        printk( KERN_INFO "failed to add cdev\n" );

    config_group_init( &ks_generator_subsys.su_group );
    mutex_init( &ks_generator_subsys.su_mutex );

    ret = configfs_register_subsystem( &ks_generator_subsys );
out:
    return ret;
}

static void
ks_exit( void )
{
    int i;

    free_page( ( unsigned long )zeroed_page );
    configfs_unregister_subsystem( &ks_generator_subsys );
    for ( i = 0; i < MAX_KS_NODES; ++i )
        device_destroy( ks_dev.sysfs_class,
                        MKDEV( MAJOR( ks_dev.devno ),
                               MINOR( ks_dev.devno ) + i ) );
    class_destroy( ks_dev.sysfs_class );
    cdev_del( &ks_dev.cdev );
    unregister_chrdev_region( ks_dev.devno, MAX_KS_NODES );
}

module_init( ks_init );
module_exit( ks_exit );

MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Jamie Iles" );
