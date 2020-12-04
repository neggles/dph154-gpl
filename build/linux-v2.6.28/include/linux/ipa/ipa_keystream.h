/* -*- C -*-
 * ipacrypto.h -- definitions for the ipacrypto keystream driver
 *
 * Copyright (C) 2010 ip.access Ltd
 *
 */

#ifndef IPA_KEYSTREAM_H_INCLUDED
#define IPA_KEYSTREAM_H_INCLUDED

#include <linux/ioctl.h>


/*
 * Ioctl definitions
 */

/* Use 'K' as magic number */
#define IPAC_IOCTL_MAGIC          'K'

#define IPAC_IOCTL_ALLOC_CONTEXT         _IO(IPAC_IOCTL_MAGIC,   0)
#define IPAC_IOCTL_DEALLOC_CONTEXT       _IO(IPAC_IOCTL_MAGIC,   1)
#define IPAC_IOCTL_CHECK_SPACE           _IO(IPAC_IOCTL_MAGIC,   2)
#define IPAC_IOCTL_START_CRYPTO          _IO(IPAC_IOCTL_MAGIC,   3)
#define IPAC_IOCTL_CHECK_CRYPTO_READY    _IO(IPAC_IOCTL_MAGIC,   4)
#define IPAC_IOCTL_BUFFER_UNLOAD         _IO(IPAC_IOCTL_MAGIC,   5)

#define IPAC_IOCTL_MAXNR          5

/* The driver will limit the maximum number of requests that can be submitted
 * to the cipher engine at any one time.  Each request can consist of multiple
 * PDUs and these are submitted together.  This limit is therefore the maximum
 * number of PDUs that can be requested in a single request.
 */
#define IPAC_MAX_ALLOWED_CIPHER_REQUESTS  0

/* We have two nodes, one for uplink (/dev/ipacrypto_0) and another for downlink
 * (/dev/ipacrypto_1)
 */
#define IPAC_MAX_DRIVER_NODES         2

/* Each node can support a certain number of ciphering contexts */
#define IPAC_NUM_CONTEXTS             96

/* This is the maximum number of requests that can be made at once on each node.
 * If this is set to 0 then the number will be unlimited.
 */
#define IPAC_MAX_NUM_BUFFERS_ALLOWED  0

/* Each driver node has a memory buffer for results.  This buffer stores multiple
 * requests.  A request has a header with info to be passed back to the caller and
 * that's followed by the keystream data.  This size of the buffer is set by the
 * following definition.
 */
#define IPAC_KEYSTREAM_BUFFER_SIZE  65536

/* We support on f8(kasumi) in this driver */
#define IPAC_CIPHER_KEY_LENGTH_BITS   128
#define IPAC_IV_LENGTH_BITS           64

#define IPAC_CIPHER_KEY_LENGTH_BYTES  ((IPAC_CIPHER_KEY_LENGTH_BITS + 7) / 8)
#define IPAC_IV_LENGTH_BYTES          ((IPAC_IV_LENGTH_BITS + 7) / 8)
#define IPAC_IV_LENGTH_LONGS          ((IPAC_IV_LENGTH_BITS + 31) / 32)

typedef struct IpacContextIoctlTag  IpacContextIoctl;
typedef struct IpacStartIoctlTag    IpacStartIoctl;

typedef enum   IpacStateTag         IpacState;
typedef struct IpacRequestTag       IpacRequest;

struct IpacContextIoctlTag {
    unsigned char     cipherKey[IPAC_CIPHER_KEY_LENGTH_BYTES];
    unsigned char     bearerId;
    signed char       contextId;
};

struct IpacStartIoctlTag {
    unsigned long     transactionId;
    unsigned          *counts;
    unsigned short    numPdus;
    unsigned short    pduLength;
    signed long       contextId;
};


enum IpacStateTag
{
    IPAC_REQUEST_UNUSED,    /* Request freed. */
    IPAC_REQUEST_WAITING,   /* Request not yet submitted to cipher engine */
    IPAC_REQUEST_CIPHERING, /* Request submitted to the cipher engine */
    IPAC_REQUEST_COMPLETED, /* All PDUs in request have been ciphered */
    IPAC_REQUEST_RETURNED,  /* Results have been passed back to the user */
    IPAC_REQUEST_WRAPPED,   /* Next request is at start of buffer */
    IPAC_REQUEST_FINAL      /* No more requests in the buffer */
};

/* We have transactions in a ring buffer in memory */
struct IpacRequestTag {
    IpacState                  state;         /* The current state of the request */
    unsigned long              transactionId; /* The transaction ID */
    struct crypto_ablkcipher   *cipher_p;     /* Pointer to the cipher that was used for this request */
    unsigned long              channel;       /* Which channel/node this request was for */
    signed long                contextId;     /* The context used for the request */
    unsigned short             reservedBytes; /* Number of bytes to skip to get to the next header */
    unsigned short             numberOfPdus;  /* How many PDUs were requested */
    unsigned short             pduLength;     /* How many bytes in each PDU */
    unsigned short             pdusCompleted; /* How many PDU have already been ciphered. */
    unsigned                   ivFixedPart;   /* The combination of binding ID and direction */
};

#endif /* IPA_KEYSTREAM_H_INCLUDED */
