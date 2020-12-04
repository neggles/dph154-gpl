/******************************************************************************
 *
 * Copyright (c) 2009 ip.access Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * 22 Apr 2009 Created by Simon D Hughes.
 *
 * Debug card driver.  Communicates with FPGA on debug card via EBI.
 *****************************************************************************
 * Filename: 
 *****************************************************************************/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/uio.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#if defined(CONFIG_ARCH_FIRECRACKER)
#include <mach/pc20x/pc20x.h>
#elif defined(CONFIG_ARCH_PC302)
#include <mach/pc302/pc302.h>
#include <mach/pc302/ebi.h>
#else  /* CONFIG_ARCH_... */
#  error "Unknown architecture"
#endif /* CONFIG_ARCH_... */

#include <mach/hardware.h>

#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
#include <mach/fpga_debug.h>
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

#define IPA_DEBUG_MAJOR 212
#define DEVICE_NAME "ipa_debug"

#define TEST_FPGA_INTERFACE 0

#define noIPA_VERBOSE 1

#ifdef IPA_VERBOSE 
#define IPA_PRINTK(A) printk A
#else
#define IPA_PRINTK(A)
#endif

#define START_OF_MESSAGE    0xFAFA
#define ESCAPE_SEQUENCE     0xFAF8
#define SWITCH_OF_CHANNEL   0xFAF9
#define END_OF_MESSAGE      0xFAFB

/* Defined in PC202 programmer's guide, section 3.2.1
 * Start address for external chip 2.
 */
#define DEBUG_FPGA_BASE_ADDRESS                     IO_ADDRESS(0x40000000)

#define LWORD_TO_BYTE(A)                            ((A) * 4)

#define DEBUG_FPGA_BASE_OFFSET                      0
#define FPGA_VERSION_REGISTER(bASE)                 ((bASE) + DEBUG_FPGA_BASE_OFFSET)
#define FPGA_BUFFER_READ_CONTROL_REGISTER(bASE)     ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x01))
#define FPGA_TEST_REGISTER(bASE)                    ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x02))
#define FPGA_BUFFER_DEPTH_REGISTER(bASE)            ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x03))
#define FPGA_DATA_LOSS_TOKEN1_REGISTER(bASE)        ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x04))
#define FPGA_DATA_LOSS_TOKEN2_REGISTER(bASE)        ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x05))
#define FPGA_CLEAR_BUFFER_REGISTER(bASE)            ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x06))
#define FPGA_ARM_PID_ENABLE_REGISTER(bASE)          ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x10))
#define FPGA_ARM_PID_VALUE_REGISTER(bASE)           ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x11))
#define FPGA_ARM_MESSAGE_WRITE_REGISTER(bASE)       ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x12))
#define FPGA_ARM_MESSAGE_TERMINATE_REGISTER(bASE)   ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x13))
#define FPGA_ARM_NUM_WORDS_REGISTER(bASE)           ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x14))
#define FPGA_ARM_MESSAGE_READ_REGISTER(bASE)        ((bASE) + DEBUG_FPGA_BASE_OFFSET + LWORD_TO_BYTE(0x15))

#define DEBUG_FPGA_SIZEOF_REG_MAP             LWORD_TO_BYTE(0x20)

/* Buffer Read Control values */
#define BUFFER_DATA_GOES_TO_USB               0x00
#define BUFFER_DATA_GOES_TO_ARM               0x01

/* Dataloss token, magic pattern for detecting overruns */
#define DATA_LOSS_TOKEN1                       0xdead
#define DATA_LOSS_TOKEN2                       0xfeed

/* Number of words in the FPGA message buffer */
#define DEFAULT_BUFFER_DEPTH                  64

#if defined(CONFIG_ARCH_FIRECRACKER)

/* Defined in PC202 programmer's guide, appendix N
 * EBI Configuration
 */
#define EBI_CONFIG_BASE_ADDRESS               IO_ADDRESS(PC20X_EBI_BASE) /* 0xFFE20000 */
#define EBI_SRAM_ADDR_CONFIG_BANK_2           EBI_CONFIG_BASE_ADDRESS + 0x0000005C
#define EBI_SRAM_TIMING_CONFIG_SET_1          EBI_CONFIG_BASE_ADDRESS + 0x00000098

/* The EBI clock is normally 70MHz, however early versions of the silicon
 * needed to be run at a slower clock speed so for them the EBI clock
 * is running at 40MHz.
 */
#define EBI_40MHZ_CLOCK_PERIOD_NS             25
#define EBI_70MHZ_CLOCK_PERIOD_NS             14
#define EBI_CLOCK_PERIOD_NS                   EBI_40MHZ_CLOCK_PERIOD_NS

/* Address Map Register, see appendix N.3
 */
#define ADDR_CONFIG_RESERVED_BITS             0x00000100
#define ADDR_CONFIG_MEM_TYPE_SRAM             0x00000020
#define ADDR_CONFIG_MEM_TYPE_FLASH            0x00000040
#define ADDR_CONFIG_MEM_SIZE_NONE             0x00000000
#define ADDR_CONFIG_MEM_SIZE_512MB            0x0000000E

#define SRAM_ADDR_CONFIG_MASK                 ( 0x0000007F )
#define SRAM_ADDR_CONFIG_FOR_FPGA             ( ADDR_CONFIG_RESERVED_BITS | \
                                                ADDR_CONFIG_MEM_TYPE_SRAM | \
                                                ADDR_CONFIG_MEM_SIZE_512MB )

/* Static Memory Timing Register, see appendix N.3
 */
#define TIMING_CONFIG_RESERVED_BITS_31_27_MASK   0xF8000000
#define TIMING_CONFIG_RESERVED_BITS_31_27        0x10000000
#define TIMING_CONFIG_DATA_READY_MASK            0x04000000
#define TIMING_CONFIG_IS_DATA_READY_DEV          0x04000000
#define TIMING_CONFIG_IS_NOT_DATA_READY_DEV      0x00000000
#define TIMING_CONFIG_RESERVED_BITS_25_23_MASK   0x03800000
#define TIMING_CONFIG_RESERVED_BITS_25_23        0x00000000
/* Page mode Read Cycle time - n/a for SRAM, use default value */
#define TIMING_CONFIG_T_PRC_MASK                 0x00780000
#define TIMING_CONFIG_T_PRC_2_CYCLES             0x00080000
/* Bus Turn Around time - must be at least 50ns (2 cycles @ 40MHz or 3.5 @ 70MHz) */
#define TIMING_CONFIG_T_BTA_MASK                 0x00070000
#define TIMING_CONFIG_T_BTA_2_CYCLES             0x00040000
#define TIMING_CONFIG_T_BTA_3_CYCLES             0x00060000
/* Write Pulse width - using default (2 clock cycles) */
#define TIMING_CONFIG_T_WP_MASK                  0x0000FC00
#define TIMING_CONFIG_T_WP_2_CYCLES              0x00000400
#define TIMING_CONFIG_T_WP_3_CYCLES              0x00000800
/* WRite address hold time - using default (0.5 clock cycle) */
#define TIMING_CONFIG_T_WR_MASK                  0x00000300
#define TIMING_CONFIG_T_WR_HALF_CYCLE            0x00000100
/* write Address Setup time - using default (1.0 clock cycle) */
#define TIMING_CONFIG_T_AS_MASK                  0x000000C0
#define TIMING_CONFIG_T_AS_1_CYCLE               0x00000080
/* Read Cycle time - using default (3.0 clock cycle) */
#define TIMING_CONFIG_T_RC_MASK                  0x0000003F
#define TIMING_CONFIG_T_RC_3_CYCLES              0x00000005

#define SRAM_TIMING_CONFIG_MASK        ( 0x07FFFFFF )
#define SRAM_TIMING_CONFIG_FOR_FPGA    ( TIMING_CONFIG_RESERVED_BITS_31_27 | \
                                         TIMING_CONFIG_IS_NOT_DATA_READY_DEV | \
                                         TIMING_CONFIG_RESERVED_BITS_25_23 | \
                                         TIMING_CONFIG_T_PRC_2_CYCLES | \
                                         TIMING_CONFIG_T_BTA_2_CYCLES | \
                                         TIMING_CONFIG_T_WP_3_CYCLES | \
                                         TIMING_CONFIG_T_WR_HALF_CYCLE | \
                                         TIMING_CONFIG_T_AS_1_CYCLE | \
                                         TIMING_CONFIG_T_RC_3_CYCLES )

#elif defined(CONFIG_ARCH_PC302)

/* Defined in PC302 programmer's guide, appendix D
 * EBI Configuration
 */
#define EBI_CONFIG_BASE_ADDRESS               IO_ADDRESS(PC302_EBI_BASE) /* 0xFFE20000 */
#define EBI_SRAM_ADDR_CONFIG_BANK_2           (EBI_CONFIG_BASE_ADDRESS + SMASK0_REG_OFFSET)
#define EBI_SRAM_TIMING_CONFIG_SET_1          (EBI_CONFIG_BASE_ADDRESS + STATIC_MEM_TIMSET2_REG_OFFSET)

#define EBI_40MHZ_CLOCK_PERIOD_NS             25
#define EBI_70MHZ_CLOCK_PERIOD_NS             14

/* Address Map Register, see appendix D.3
 */

#define SRAM_ADDR_CONFIG_MASK \
    (EBI_SMSKR_REG_SELECT_MASK << EBI_SMSKR_REG_SELECT_SHIFT) | \
    (EBI_SMSKR_MEM_TYPE_MASK << EBI_SMSKR_MEM_TYPE_SHIFT) | \
    (EBI_SMSKR_MEM_SIZE_MASK << EBI_SMSKR_MEM_SIZE_SHIFT)

#define SRAM_ADDR_CONFIG_FOR_FPGA \
    (EBI_REG_SELECT_TIMING_SET_2 << EBI_SMSKR_REG_SELECT_SHIFT) | \
    (EBI_MEM_TYPE_SRAM << EBI_SMSKR_MEM_TYPE_SHIFT) | \
    (EBI_MEM_SIZE_512MB << EBI_SMSKR_MEM_SIZE_SHIFT)

/* Static Memory Timing Register, see appendix D.3
 */
#define SRAM_TIMING_CONFIG_MASK \
    ( EBI_SMTMGR_REG_CLK_SYNC_MASK << EBI_SMTMGR_REG_CLK_SYNC_SHIFT) | \
    ( EBI_SMTMGR_REG_READY_MODE_MASK << EBI_SMTMGR_REG_READY_MODE_SHIFT) | \
    ( EBI_SMTMGR_REG_PAGE_READ_CYCLE_MASK << EBI_SMTMGR_REG_PAGE_READ_CYCLE_SHIFT) | \
    ( EBI_SMTMGR_REG_BUS_TURN_AROUND_MASK << EBI_SMTMGR_REG_BUS_TURN_AROUND_SHIFT) | \
    ( EBI_SMTMGR_REG_WRITE_PULSE_MASK << EBI_SMTMGR_REG_WRITE_PULSE_SHIFT) | \
    ( EBI_SMTMGR_REG_ADDR_HOLD_MASK << EBI_SMTMGR_REG_ADDR_HOLD_SHIFT) | \
    ( EBI_SMTMGR_REG_ADDR_SETUP_MASK << EBI_SMTMGR_REG_ADDR_SETUP_SHIFT) | \
    ( EBI_SMTMGR_REG_TIMING_READ_CYCLE_MASK)

#define EBI_SMTMGR_REG_PAGE_READ_CYCLE_2_CYCLES         1
#define EBI_SMTMGR_REG_BUS_TURN_AROUND_2_CYCLES         4
#define EBI_SMTMGR_REG_WRITE_PULSE_3_CYCLES             2
#define EBI_SMTMGR_REG_ADDR_HOLD_HALF_CYCLE             1
#define EBI_SMTMGR_REG_ADDR_SETUP_1_CYCLE               2
#define EBI_SMTMGR_REG_TIMING_READ_CYCLE_3_CYCLES       5

#define SRAM_TIMING_CONFIG_FOR_FPGA \
    ( 0 << EBI_SMTMGR_REG_CLK_SYNC_SHIFT) | \
    ( 0 << EBI_SMTMGR_REG_READY_MODE_SHIFT) | \
    ( EBI_SMTMGR_REG_PAGE_READ_CYCLE_2_CYCLES << EBI_SMTMGR_REG_PAGE_READ_CYCLE_SHIFT ) | \
    ( EBI_SMTMGR_REG_BUS_TURN_AROUND_2_CYCLES << EBI_SMTMGR_REG_BUS_TURN_AROUND_SHIFT ) | \
    ( EBI_SMTMGR_REG_WRITE_PULSE_3_CYCLES << EBI_SMTMGR_REG_WRITE_PULSE_SHIFT ) | \
    ( EBI_SMTMGR_REG_ADDR_HOLD_HALF_CYCLE << EBI_SMTMGR_REG_ADDR_HOLD_SHIFT ) | \
    ( EBI_SMTMGR_REG_ADDR_SETUP_1_CYCLE << EBI_SMTMGR_REG_ADDR_SETUP_SHIFT ) | \
    ( EBI_SMTMGR_REG_TIMING_READ_CYCLE_3_CYCLES )

#else  /* CONFIG_ARCH_... */

#  error "Unknown architecture"

#endif /* CONFIG_ARCH_... */

/* module init return codes */
#define IPA_DEBUG_MODULE_LOAD_SUCCESS  0
#define IPA_DEBUG_MODULE_LOAD_ERROR   -1

static spinlock_t  mySpinLock = SPIN_LOCK_UNLOCKED;

static struct cdev ipa_debug_cdev;
static void *fpgaRegBaseAddr = 0;        /* io mapped address fix me: put in driver ctx */

static ssize_t ipa_debug_read ( struct file* fileHandle,
                                char __user* inputBuffer,
                                size_t       length,
                                loff_t*      offset )
{
    ssize_t numBytesRead = 0;
    ssize_t numArmWordToRead = 0;

    unsigned long fpgaVersion = 0;
    //void *fpgaRegBaseAddr = 0;

    IPA_PRINTK( ( KERN_INFO "Reading %d bytes of message\n", length) );

    //fpgaRegBaseAddr = ioremap(DEBUG_FPGA_BASE_ADDRESS,DEBUG_FPGA_SIZEOF_REG_MAP);
    
    fpgaVersion = ioread32(FPGA_VERSION_REGISTER(fpgaRegBaseAddr));
    IPA_PRINTK( ( KERN_INFO "FPGA version 0x%lx for the IPA Debug card\n",fpgaVersion) );
    
    if (inputBuffer)
    {
        numArmWordToRead = ioread32(FPGA_ARM_NUM_WORDS_REGISTER(fpgaRegBaseAddr));
        IPA_PRINTK( ( KERN_INFO "%d words available\n", numArmWordToRead) );

        for (numBytesRead = 0; 
             ((numBytesRead < length) && 
             (numBytesRead < (sizeof(unsigned short)*numArmWordToRead))); 
             numBytesRead+=sizeof(unsigned short))
        {
            unsigned long wordRead = 0;
            unsigned char kernelWord[2];
            wordRead = ioread32(FPGA_ARM_MESSAGE_READ_REGISTER(fpgaRegBaseAddr));
            kernelWord[0] = (wordRead >> 8) & 0xff;
            kernelWord[1] = wordRead & 0xff;
            if (copy_to_user(inputBuffer+numBytesRead,kernelWord,2))
            {
                return -EFAULT;
            }
        }

    }

    //iounmap(fpgaRegBaseAddr);

    return numBytesRead;
}

static ssize_t ipa_debug_write ( struct file*       fileHandle,
                                 const char __user* outputBuffer,
                                 size_t             length,
                                 loff_t*            offset )
{
    size_t                         index;
    unsigned short                 msgWord;
    const unsigned short __user*   readPtr = (const unsigned short __user*)outputBuffer;
    unsigned long                  readTest = 0;
    unsigned long                  flags;
   
    IPA_PRINTK( ( KERN_INFO "Writing %d bytes of message\n", length) );

    if ((length < 2) || (length & 1) || !access_ok(VERIFY_READ, (void __user*)readPtr, length)) {
        IPA_PRINTK( ( KERN_INFO "Write had invalid parameteri %d %p\n", length, outputBuffer) );
        return 0;
    }

    /* Check for alignment in the buffer */
    if (((unsigned long)outputBuffer) & 1) {
        IPA_PRINTK( ( KERN_INFO "Write had misalignment %p\n", outputBuffer) );
        return 0;
    }

    spin_lock_irqsave(&mySpinLock, flags);
    
    /* Begin the message and generate the time stamp */
    iowrite32(START_OF_MESSAGE, FPGA_ARM_PID_VALUE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the message length, allowing for the PID that follows */
    iowrite32(length+2, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the PID */
    iowrite32(current->pid, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write all but the last one */
    for (index=2; index<length; index+=2) {
        __get_user(msgWord, readPtr);
        iowrite32(msgWord, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readPtr++;
    }

    /* Last one terminates */
    __get_user(msgWord, readPtr);
    iowrite32(msgWord, FPGA_ARM_MESSAGE_TERMINATE_REGISTER(fpgaRegBaseAddr));

    spin_unlock_irqrestore(&mySpinLock, flags);
    
    return length;
}

static ssize_t ipa_debug_aio_write ( struct kiocb*        iocb,
                                     const struct iovec*  iov,
                                     unsigned long        count,
                                     loff_t               offset )
{
    ssize_t                        index  = 0;
    __kernel_size_t                length = 0;
    unsigned short                 msgWord;
    __kernel_size_t                base   = 0;
    unsigned long                  element;
    const unsigned short __user*   readPtr;
    unsigned long                  readTest = 0;
    unsigned long                  flags;
    
    IPA_PRINTK( ( KERN_INFO "Writing vector of %ld elements\n", count) );

    /* Find the total number of bytes to be written (length), check that
       each separate element is a whole number of 2-byte words and get
       the size of the largest element. */
    for (element=0; element<count; ++element) {

        __kernel_size_t  element_length = iov[element].iov_len;

        if (element_length & 1) {
            IPA_PRINTK( ( KERN_INFO "ipa_debug: Buffer has odd length %d\n", element_length) );
            return 0;
        }

        if (!access_ok(VERIFY_READ, (void __user*)iov[element].iov_base, element_length)) {
            IPA_PRINTK( ( KERN_INFO "ipa_debug: Buffer not accessible\n") );
            return 0;
        }

        length += element_length;
    }

    IPA_PRINTK( ( KERN_INFO "Writing vectors with total of %d bytes\n", length) );

    if (length < 2) {
        /* Nothing to do */
        return 0;
    }

    spin_lock_irqsave(&mySpinLock, flags);
    
    /* Begin the message and generate the time stamp */
    iowrite32(START_OF_MESSAGE, FPGA_ARM_PID_VALUE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the message length, allowing for the PID that follows */
    iowrite32(length+2, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the PID */
    iowrite32(current->pid, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    readPtr = (const unsigned short __user*)iov[0].iov_base;
	
    for (element=0; element<count; ++element) {

        __kernel_size_t   element_len = iov[element].iov_len;

        readPtr = (const unsigned short __user*)iov[element].iov_base;

        if (element_len+base >= length-2) {
            element_len = length-2 - base;
        }
        base += element_len;

        for (index=0; index<element_len; index+=2) {
            __get_user(msgWord, readPtr);
            iowrite32(msgWord, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
            readPtr++;
        }
    }

    /* Last one terminates */
    __get_user(msgWord, readPtr);
    iowrite32(msgWord, FPGA_ARM_MESSAGE_TERMINATE_REGISTER(fpgaRegBaseAddr));

    spin_unlock_irqrestore(&mySpinLock, flags);
    
    return length;
}

#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
static void ipaDebugFpgaKernelCommon(const char* text, int n, int num1, int num2, int num3, int num4)
{
    __kernel_size_t                length = 0;
    unsigned short                 msgWord;
    unsigned long                  readTest = 0;
    unsigned long                  flags;
    
    /* Find the total number of bytes to be written (length), check that
       each separate element is a whole number of 2-byte words and get
       the size of the largest element. */
    length = ((strlen(text)+2) & ~1) + 12 + 4*n;

    spin_lock_irqsave(&mySpinLock, flags);
    
    /* Begin the message and generate the time stamp */
    iowrite32(START_OF_MESSAGE, FPGA_ARM_PID_VALUE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the message length */
    iowrite32(length, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the PID */
    iowrite32(0, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the AppName token */
    iowrite32(0, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    iowrite32(0, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write "K" */
    iowrite32(0x004B, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the Text token */
    iowrite32(0, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    iowrite32(0, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    /* Write the text */
    msgWord = 0;
    
    while (*text) {
    
        msgWord = *text++;
        
        if (*text) {
            msgWord |= (*text++) << 8;
            iowrite32(msgWord, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
            readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));
            msgWord = 0;
        } else {
            break;
        }
    }
    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

    if (n > 0) {
        iowrite32(msgWord, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));
    
        iowrite32((num1 & 0xFFFF), FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

        msgWord = num1 >> 16;
    }
    if (n > 1) {
        iowrite32(msgWord, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));
    
        iowrite32((num2 & 0xFFFF), FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

        msgWord = num2 >> 16;
    }
    if (n > 2) {
        iowrite32(msgWord, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));
    
        iowrite32((num3 & 0xFFFF), FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

        msgWord = num3 >> 16;
    }
    if (n > 3) {
        iowrite32(msgWord, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));
    
        iowrite32((num4 & 0xFFFF), FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
        readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

        msgWord = num4 >> 16;
    }
    
    /* Last one terminates */
    iowrite32(msgWord, FPGA_ARM_MESSAGE_TERMINATE_REGISTER(fpgaRegBaseAddr));
    
    spin_unlock_irqrestore(&mySpinLock, flags);
}

static void ipaDebugFpgaKernel0(const char* text)
{
    ipaDebugFpgaKernelCommon(text, 0, 0, 0, 0, 0);
}

static void ipaDebugFpgaKernel1(const char* text, int num1)
{
    ipaDebugFpgaKernelCommon(text, 1, num1, 0, 0, 0);
}

static void ipaDebugFpgaKernel2(const char* text, int num1, int num2)
{
    ipaDebugFpgaKernelCommon(text, 2, num1, num2, 0, 0);
}

static void ipaDebugFpgaKernel3(const char* text, int num1, int num2, int num3)
{
    ipaDebugFpgaKernelCommon(text, 3, num1, num2, num3, 0);
}

static void ipaDebugFpgaKernel4(const char* text, int num1, int num2, int num3, int num4)
{
    ipaDebugFpgaKernelCommon(text, 4, num1, num2, num3, num4);
}
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

/* File operations - driver entry points */
static struct file_operations ipa_debug_fops =
{
    .owner     = THIS_MODULE,
    .read      = ipa_debug_read,
    .write     = ipa_debug_write,
    .aio_write = ipa_debug_aio_write,
};

/* Called when module is loaded */
static int __init ipa_debug_init(void)
{
    int devNum = 0;
    int error = 0;
    int status = IPA_DEBUG_MODULE_LOAD_SUCCESS;
    
    printk ( KERN_INFO "IPA Debug driver built " __DATE__ " " __TIME__ "\n" );
    
    /* Set up Cdev */
    cdev_init ( &ipa_debug_cdev, &ipa_debug_fops );
    ipa_debug_cdev.owner = THIS_MODULE;
    ipa_debug_cdev.ops = &ipa_debug_fops;
    
    /* Register the class device */
    devNum = MKDEV ( IPA_DEBUG_MAJOR, 0 );
    error = cdev_add ( &ipa_debug_cdev, devNum, 1 );
    if ( error )
    {
        printk ( KERN_ERR "Error %d adding the IPA Debug driver\n", error );
        status = IPA_DEBUG_MODULE_LOAD_ERROR;
    }
    else
    {
        /* Register the major number */
        register_chrdev_region ( MKDEV ( IPA_DEBUG_MAJOR, 0 ), 1, DEVICE_NAME );
    
        /* Set up EBI for device 2 - the FPGA acts as SRAM on CS2 of this bus */
        if (request_region(EBI_SRAM_ADDR_CONFIG_BANK_2, sizeof(unsigned long), DEVICE_NAME) &&
            request_region(EBI_SRAM_TIMING_CONFIG_SET_1, sizeof(unsigned long), DEVICE_NAME) &&
            request_mem_region(DEBUG_FPGA_BASE_ADDRESS, DEBUG_FPGA_SIZEOF_REG_MAP, DEVICE_NAME))
        {
            unsigned ebiRegisterRead;
            unsigned long fpgaVersion = 0;
            //unsigned long readCtrlVal = 0;
            unsigned long readTest = 0;
            //void *fpgaRegBaseAddr = 0;

            IPA_PRINTK( ( KERN_ERR "Setting EBI registers for the IPA Debug driver\n") );
            ebiRegisterRead = inl(EBI_SRAM_ADDR_CONFIG_BANK_2);
            ebiRegisterRead &= ~SRAM_ADDR_CONFIG_MASK;
            ebiRegisterRead |= SRAM_ADDR_CONFIG_FOR_FPGA;
            outl(ebiRegisterRead, EBI_SRAM_ADDR_CONFIG_BANK_2);
            
            ebiRegisterRead = inl(EBI_SRAM_TIMING_CONFIG_SET_1);
            ebiRegisterRead &= SRAM_TIMING_CONFIG_MASK;
            ebiRegisterRead |= SRAM_TIMING_CONFIG_FOR_FPGA;
            outl(ebiRegisterRead, EBI_SRAM_TIMING_CONFIG_SET_1);
        
            fpgaRegBaseAddr = ioremap(DEBUG_FPGA_BASE_ADDRESS,DEBUG_FPGA_SIZEOF_REG_MAP);

            fpgaVersion = ioread32(FPGA_VERSION_REGISTER(fpgaRegBaseAddr));
            printk ( KERN_INFO "FPGA version 0x%lx for the IPA Debug card\n",fpgaVersion);

            /* Set read direction to USB */
            iowrite32(BUFFER_DATA_GOES_TO_USB,FPGA_BUFFER_READ_CONTROL_REGISTER(fpgaRegBaseAddr));

            /* FIXME - EBI timings needs work to prevent successive writes going
             * out too quickly.  Workaround for now is to add a read between writes
             * to use BTA to hold off nect write.
             */

            /* force delay between writes */
            readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

            /* Allow allow PIDs through - no filtering */
            iowrite32(0xffff,FPGA_ARM_PID_ENABLE_REGISTER(fpgaRegBaseAddr));

            /* force delay between writes */
            readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

            /* Set token for buffer overrun indication */
            iowrite32(DATA_LOSS_TOKEN1, FPGA_DATA_LOSS_TOKEN1_REGISTER(fpgaRegBaseAddr));
            readTest = ioread32(FPGA_DATA_LOSS_TOKEN1_REGISTER(fpgaRegBaseAddr));
            printk ( KERN_INFO "Data loss token1 = 0x%lx\n",readTest);

            /* force delay between writes */
            readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

            /* Set token for buffer overrun indication */
            iowrite32(DATA_LOSS_TOKEN2, FPGA_DATA_LOSS_TOKEN2_REGISTER(fpgaRegBaseAddr));
            readTest = ioread32(FPGA_DATA_LOSS_TOKEN2_REGISTER(fpgaRegBaseAddr));
            printk ( KERN_INFO "Data loss token2 = 0x%lx\n",readTest);

            /* force delay between writes */
            readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

            /* Set message buffer depth in FPGA */
            iowrite32(DEFAULT_BUFFER_DEPTH, FPGA_BUFFER_DEPTH_REGISTER(fpgaRegBaseAddr));

            //iounmap(fpgaRegBaseAddr);
            if (TEST_FPGA_INTERFACE) {

                int  i;
                int  wordsToRead = 0;

                /* Display current values of the registers we can read from */
                readTest = ioread32(FPGA_BUFFER_READ_CONTROL_REGISTER(fpgaRegBaseAddr));
                printk( KERN_INFO "Control Reg is %lx\n", readTest);

                readTest = ioread32(FPGA_DATA_LOSS_TOKEN1_REGISTER(fpgaRegBaseAddr));
                printk( KERN_INFO "Data loss token1 = %lx\n", readTest);

                readTest = ioread32(FPGA_DATA_LOSS_TOKEN2_REGISTER(fpgaRegBaseAddr));
                printk( KERN_INFO "Data loss token2 = %lx\n", readTest);

                readTest = ioread32(FPGA_ARM_PID_ENABLE_REGISTER(fpgaRegBaseAddr));
                printk( KERN_INFO "ARM PID Enable register is %lx\n", readTest);

                /* Check each data bit using the test register */
                for (i=0; i<=16; i++) {

                    int check = 1<<i;

                    iowrite32(check, FPGA_TEST_REGISTER(fpgaRegBaseAddr));
                    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));
                    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

                    printk( KERN_INFO "test reg: wrote %x read %lx\n", check, readTest);
                }

                /* Check that we can write data to the buffer and read it back */
                iowrite32(BUFFER_DATA_GOES_TO_ARM,FPGA_BUFFER_READ_CONTROL_REGISTER(fpgaRegBaseAddr));
                readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

                iowrite32(0,FPGA_CLEAR_BUFFER_REGISTER(fpgaRegBaseAddr));
                readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

                iowrite32(1, FPGA_ARM_PID_VALUE_REGISTER(fpgaRegBaseAddr));
                readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

                for (i=0; i<=16; i++) {

                    int check = 1<<i;

                    iowrite32(check, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
                    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));
                }

                iowrite32(0xFFFF, FPGA_ARM_MESSAGE_TERMINATE_REGISTER(fpgaRegBaseAddr));
                readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

                wordsToRead = ioread32(FPGA_ARM_NUM_WORDS_REGISTER(fpgaRegBaseAddr));
                printk( KERN_INFO "Number of words to read is %d\n", wordsToRead);
                
                for (i=0; i<wordsToRead; i++) {

                    readTest = ioread32(FPGA_ARM_MESSAGE_READ_REGISTER(fpgaRegBaseAddr));
                    printk( KERN_INFO "Value %d is %lx\n", i, readTest);
                }

                /* Switch back to sending data to the USB interface */
                iowrite32(BUFFER_DATA_GOES_TO_USB,FPGA_BUFFER_READ_CONTROL_REGISTER(fpgaRegBaseAddr));

                iowrite32(0xFAFA, FPGA_ARM_PID_VALUE_REGISTER(fpgaRegBaseAddr));
                readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

                iowrite32(38, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
                readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

                iowrite32(0x5555, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
                readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

                for (i=0; i<=16; i++) {

                    int check = 1<<i;

                    iowrite32(check, FPGA_ARM_MESSAGE_WRITE_REGISTER(fpgaRegBaseAddr));
                    readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));
                }

                iowrite32(0xFFFF, FPGA_ARM_MESSAGE_TERMINATE_REGISTER(fpgaRegBaseAddr));
                readTest = ioread32(FPGA_TEST_REGISTER(fpgaRegBaseAddr));

            }

            release_region(EBI_SRAM_ADDR_CONFIG_BANK_2, sizeof(unsigned long));
            release_region(EBI_SRAM_TIMING_CONFIG_SET_1, sizeof(unsigned long));
        }
        else
        {
            printk ( KERN_ERR "Error accessing EBI registers for the IPA Debug driver\n");
            status = IPA_DEBUG_MODULE_LOAD_ERROR;
        }
    
    }    
 
#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
    if (status == IPA_DEBUG_MODULE_LOAD_SUCCESS)
    {
        ipaFpgaDebug0Ptr = ipaDebugFpgaKernel0;
        ipaFpgaDebug1Ptr = ipaDebugFpgaKernel1;
        ipaFpgaDebug2Ptr = ipaDebugFpgaKernel2;
        ipaFpgaDebug3Ptr = ipaDebugFpgaKernel3;
        ipaFpgaDebug4Ptr = ipaDebugFpgaKernel4;
    }
    else
    {
        ipaFpgaDebug0Ptr = NULL;
        ipaFpgaDebug1Ptr = NULL;
        ipaFpgaDebug2Ptr = NULL;
        ipaFpgaDebug3Ptr = NULL;
        ipaFpgaDebug4Ptr = NULL;
    }
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

    return status;
}

/* Called to unload the module */
static void __exit ipa_debug_exit(void)
{
#if defined(CONFIG_IPACCESS_FPGA_DEBUG)
    ipaFpgaDebug0Ptr = NULL;
    ipaFpgaDebug1Ptr = NULL;
    ipaFpgaDebug2Ptr = NULL;
    ipaFpgaDebug3Ptr = NULL;
    ipaFpgaDebug4Ptr = NULL;
#endif /* defined(CONFIG_IPACCESS_FPGA_DEBUG) */

    iounmap(fpgaRegBaseAddr);
    fpgaRegBaseAddr = 0;
    release_mem_region(DEBUG_FPGA_BASE_ADDRESS, DEBUG_FPGA_SIZEOF_REG_MAP);
    unregister_chrdev_region ( MKDEV ( IPA_DEBUG_MAJOR, 0 ), 1 );
    cdev_del ( &ipa_debug_cdev );
}

module_init( ipa_debug_init );
module_exit( ipa_debug_exit );

MODULE_AUTHOR ( "IP Access Ltd" );
MODULE_DESCRIPTION ( "IPA Debug Card driver" );
MODULE_LICENSE("GPL");
