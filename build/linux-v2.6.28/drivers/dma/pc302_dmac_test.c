/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
 * \file pc302_dmac_test.c
 *
 * \brief This module tests the PC302 DMAs, DMA controller, and API.
 *
 * The PC302 DMAC has a lot of functionality some of which cannot be tested
 * or easily tested on the PC302. Specifically it supports both hardware and 
 * software flow control where the flow controller can be the DMA itself, or
 * the source or destination peripheral. Additionally the DMAC supports single
 * and  burst transfers using a single descriptor loaded into its internal
 * registers, or a list of descriptors in external memory. The DMA can also
 * handle variable bus widths up to 64 bits from several master ports, and
 * permits scatter gather transfers. Finally there are two DMA instances in the
 * PC302 which do not connect to exactly the same interfaces.
 *
 * This test module attempts to test some of the functionality listed above,
 * as well as the kernel API. The following is a summary of the functionality
 * that is tested:
 * - Direct (i.e. a single descriptor) transfers.
 * - Multiple descriptor (i.e. from a list) transfers.
 * - Transfers to and from peripherals under software handshaking. The
 *   peripheral is simulated using a memory location, and software handshaking
 *   means that the transfer of data to the end point is conducted under the
 *   control of the client software (using pc302_dma_request_transaction).
 * - Transfers to and from peripherals under software handshaking and 
 *   peripheral flow control. These tests simulate the situation where the
 *   size of the data coming from the peripheral source or going to the
 *   peripheral destination is not known, and pc302_dma_request_transaction
 *   has to be called one or more times.
 *
 * In summary this module tests:
 * - Memory to memory transfers
 * - Memory to peripheral and peripheral to peripheral transfers.
 * - Descriptor reuse testing
 * - Interrupt testing
 * - 64 bit bus widths, single and burst transfers
 * - There is no hardware interface testing as all the hardware interfaces
 *   connect directly to the picoArray where special embedded picoArray code
 *   would be required as an interface.
 */

/*
 * Copyright (c) 2009 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All inquiries to support@picochip.com
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/autoconf.h>
#include <linux/delay.h>

#include <mach/io.h>
#include <mach/hardware.h>
#include <mach/pc302_dmac.h>

/*****************************************************************************
 * Magic numbers
 *****************************************************************************/

/*!
 * A name for this module
 */
#define TITLE "pc302 DMA Controller Driver Test Module"

/*!
 * Return status for the tests
 */
#define SUCCESS           (0)
#define FAIL              (1)

/*!
 * Specify the delay (in jiffies) to use in the schedule_timeout()
 * function. 
 */
#define SMALL_DELAY       (50)

/*!
 * Specify a character to put into constant payloads
 */
#define TEST_VALUE       (0xbe)
 
/*!
 * Module default load parameter (which DMAC to test)
 */
static int dmac_to_test=0;
module_param(dmac_to_test, int, 0);

/*!
 * Module parameter description (which DMAC to test)
 */
MODULE_PARM_DESC(dmac_to_test, "Specify which DMAC to test (Default is 0)");

/*!
 * Module default load parameter (max block size to test)
 */
static int max_block_size=2048;
module_param(max_block_size, int, 1);

/*!
 * Module parameter description (max block size to test)
 */
MODULE_PARM_DESC(max_block_size, 
    "Specify the maximum block size (Default is 2048)");

/*!
 * Module default load parameter (tests to run)
 */
static unsigned int tests_to_run=0xFFFFFFFF;
module_param(tests_to_run, int, 0);

/*!
 * Module parameter description (tests to run)
 */
MODULE_PARM_DESC(tests_to_run, 
    "Specify the tests to run - 1 bit for each test (Default is 0xFFFFFFFF)");

/*****************************************************************************
 * Macros
 *****************************************************************************/

/*!
 * Macro which prints out the cause of the error
 */
#define FAIL_TEST(_why) \
    ( { \
        test_state |= FAIL; \
        printk("TEST FAILURE: %s\n",_why); \
    } )

/*!
 * Reset test result
 */
#define RESET_TEST_RES()  (test_state = SUCCESS)

/*!
 * Set test result (failed)
 */
#define TEST_FAILED       (test_state == FAIL)

/*!
 * Number of descriptors to chain in multi block tests
 */
#define LIST_SIZE         (10)

/*!
 * Number of descriptors to to reuse in multi block tests
 */
#define REUSE_LIST_SIZE   (5)

/*!
 * Number of tests
 */
#define NUM_TESTS         (21)

/*****************************************************************************
 * Private data structure & types
 *****************************************************************************/

/*!
 * Function prototype for all tests
 */
typedef void (*tf)(void);

/*!
 * Structure for holding client data. Used in tests involving interrupts
 */
typedef struct
{
    unsigned irq_state;          /*<! Total interrupts                     */
    unsigned tlxwi2_trf_ints;    /*<! Completed Transfer interrupts        */
    unsigned tlxwi2_block_ints;  /*<! Completed Block interrupts           */
    unsigned tlxwi2_srctrx_ints; /*<! Source Transfer completed interrupts */
    unsigned tlxwi2_dsttrx_ints; /*<! Destination transfer completed 
                                      interrupts                           */
    unsigned tlxwi2_err_ints;    /*<! Error interrupts                     */
    unsigned blocks_left;        /*<! Number of blocks left to transfer    */
    pc302_dma_xfr_t xfr;         /*<! The DMA transfer handle              */
}
client_data_interrupt_test_t;

/*****************************************************************************
 * Private function prototypes
 *****************************************************************************/

/*!
 * \brief Return the DMAC handle
 *
 * \param num DMA number
 *
 * \return DMA context handle, 0 for error
 */
static pc302_dma_t
get_dmac(int num);

/*!
 * \brief Fill the specified area of memory with random data
 *
 * \param vaddr Starting address
 * \param size Size of memory
 *
 * \return none
 */
static void
rand_fill(void *vaddr, 
          unsigned size);

/*!
 * \brief Perform a checksum on the area of memory specified
 *
 * \param vaddr Starting address
 * \param size Size of memory
 *
 * \return 0 Success, -1 Fail
 */
static unsigned
checksum(void *vaddr, 
         unsigned size);

/*!
 * \brief Allocate an area of non cached memory
 *
 * \param vaddr Virtual address of memory allocated
 * \param size  Size of memory to allocate
 * \param do_checksum Flag
 *                    Set to 1 if the memory is to be filled with random
 *                    data with a checksum added to the final 4 bytes.
 *
 * \return Physical address for memory allocated, 0 on error
 */
static dma_addr_t
get_dma_buffer(void **vaddr, 
               unsigned size, 
               unsigned do_checksum);

/*!
 * \brief Perform a small delay
 */
static void
shortDelay(void);

/******************************************************************************
* Interrupt handler function definitions
******************************************************************************/
/*!
 * \brief Interrupt handler for the Direct transfer test using interrupts
 *
 * \param cookie Pointer to data associated with this interrupt
 * \param errno The type of interrupt that triggered the event
 */
static int
test_direct_xfr_with_interrupts_irq(void *cookie, 
                                    int errno);

/*!
 * \brief Interrupt handler for the List transfer test using interrupts
 *
 * \param cookie Pointer to data associated with this interrupt
 * \param errno The type of interrupt that triggered the event
*/
static int
test_list_xfr_with_interrupts_irq(void *cookie, 
                                  int errno);

/*!
 * \brief Interrupt handler for the List to list transfer test with interrupts
 * and block control
 *
 * \param cookie Pointer to data associated with this interrupt
 * \param errno The type of interrupt that triggered the event
 */
static int
test_list_xfr_with_interrupts2_irq(void *cookie, 
                                   int errno);

/*!
 * \brief Interrupt handler for the simple direct transfer test using source
 * and destinmation software handshaking controlled by interrupts
 *
 * \param cookie Pointer to data associated with this interrupt
 * \param errno The type of interrupt that triggered the event
 */
static int
test_src_dst_sw_handshaking_int_irq(void *cookie, 
                                    int errno);

/******************************************************************************
* DMAC tests definitions
******************************************************************************/

/*!
 * \brief Direct memory to memory transfer test with no interrupts
 *
 * \par Test 1 (tests_to_run: 0x00000001)
 * This test will perform a direct memory to memory transfer without using
 * interrupts as follows:
 * - Sets the debug level to maximum (0xFFFF)
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize of 1
 * - Transfers a total of 4*MAX_BLOCK_SIZE bytes
 * - Performs a checksum on the data transferred
 */
static void
test_direct_xfr(void);

/*!
 * \brief Direct memory to memory transfer of an entire list without interrupts
 *
 * \par Test 2 (tests_to_run: 0x00000002)
 * This test will perform a list memory to memory transfer using a list
 * and no interrupts. It does the following:
 * - Sets up a list of LIST_SIZE blocks of size max_block_size
 * - Fills all blocks with pseudo random data
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize of 1
 * - Performs a checksum on the each block of data transferred
 */
static void
test_list_list_xfr(void);

/*!
 * \brief Direct memory to memory transfer test using interrupts
 *
 * \par Test 3 (tests_to_run: 0x00000004)
 * This test will perform a direct memory to memory transfer using interrupts.
 * It does the following:
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize of 1
 * - Installs an interrupt handler to process interrupts
 * - Enables Block completion, Transfer completion, and Error detection
 *   interrupts
 * - Counts the interrupts received.
 * - Transfers a total of 4*MAX_BLOCK_SIZE bytes
 * - Performs a checksum on the data transferred
 */
static void
test_direct_xfr_with_interrupts(void);

/*!
 * \brief List memory to list memory transfer test using interrupts
 * 
 * \par Test 4 (tests_to_run: 0x00000008)
 * This test will perform a direct memory to memory transfer using a list
 * and interrupts. It does the following:
 * - Sets up a list of LIST_SIZE blocks of size max_block_size
 * - Fills all blocks with pseudo random data
 * - Installs an interrupt handler to process interrupts
 * - Enables Block completion, Transfer completion, and Error detection
 *   interrupts
 * - Counts the interrupts received.
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize of 1
 * - Performs a checksum on the each block of data transferred
 */
static void
test_list_xfr_with_interrupts(void);

/*!
 * \brief List memory to list memory transfer test with interrupts and block
 * control.
 *
 * \par Test 5 (tests_to_run: 0x00000010)
 * This test will perform a list memory to memory transfer using a list,
 * interrupts and block control. It does the following:
 * - Sets up a list of LIST_SIZE blocks of size max_block_size
 * - Fills all blocks with pseudo random data
 * - Installs a interrupt handler to process interrupts
 * - Enables Block completion, Transfer completion, and Error detection
 *   interrupts
 * - Counts the interrupts received, and confirms they are correct.
 * - Confirms that all blocks have been transferred.
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize of 1
 * - Performs a checksum on the each block of data transferred
 */
static void
test_list_xfr_with_interrupts2(void);

/*!
 * \brief Direct memory to memory transfer test without interrupts
 * and various block sizes
 *
 * \par Test 6 (tests_to_run: 0x00000020)
 * This test perform a single direct transfer without interrupts using an
 * increasing blocks size as follows:
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to automatic
 * - Transfers block sizes from 8 to MAX_BLOCK_SIZE bytes in step of 8.
 * - Performs a checksum on the data transferred
 */
static void
test_xfr_block_sizes(void);

/*!
 * \brief Direct peripheral to memory transfer using source software
 * handshaking
 *
 * \par Test 7 (tests_to_run: 0x00000040)
 * The test uses a memory location as a dummy peripheral source
 * and then requests the transfer of the element. It:
 * - Uses PC302_DMA_MASTER as the source and destination port
 * - Uses 64 bit transfer width, with a msize set to automatic
 * - Requests a transfer of size max_block_size
 * - Performs a checksum on the data transferred
 */
static void
test_src_sw_handshaking(void);

/*!
 * \brief Direct memory to peripheral transfer using destination
 * software handshaking
 *
 * \par Test 8 (tests_to_run: 0x00000080)
 * The test uses a memory location as a dummy peripheral destination
 * and then requests the transfer of the element. It:
 * - Uses PC302_DMA_MASTER as the source and destination port
 * - Uses 64 bit transfer width, with a msize set to automatic
 * - Requests a transfer of size max_block_size
 * - Performs a checksum on the data transferred
 */
static void
test_dst_sw_handshaking(void);

/*!
 * \brief Direct peripheral to peripheral transfer using source and
 * destination software handshaking.
 *
 * \par Test 9: (tests_to_run: 0x00000100)
 * The test uses a memory location as a dummy peripheral source and
 * another memory location as a dummy destination. It then requests
 * the transfer of the element as follows:
 * - Uses PC302_DMA_MASTER1 as the source and destination port
 * - Uses 64 bit transfer width, with a msize set to automatic
 * - Requests a transfer of size max_block_size
 * - Performs a checksum on the data transferred
 */
static void
test_src_dst_sw_handshaking(void);

/*!
 * \brief Direct peripheral to peripheral transfer test using source and
 * destination software handshaking controlled by interrupts.
 *
 * \par Test 10: (tests_to_run: 0x00000200)
 * This test uses memory locations as dummy source and destination and
 * then requests performs a software controlled DMA request as following:
 * - Fills the block with pseudo random data
 * - Installs a interrupt handler to process interrupts
 * - Enables Transfer completion, Source transfer completion,
 *   destination transfer completion, and Error detection interrupts
 * - Counts the interrupts received, and confirms they are correct.
 * - Confirms that all blocks have been transferred.
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize of 1
 * - Transfers a block size of max_block_size
 * - Performs a checksum on the each block of data transferred
 */
static void
test_src_dst_sw_handshaking_int(void);

/*!
 * \brief Direct peripheral to peripheral transfer test using source and
 * destination software handshaking and various block sizes.
 *
 * \par Test 11: (tests_to_run: 0x00000400)
 * This test uses memory locations as dummy source and destination and
 * does the following:
 * - Fills the block with pseudo random data 
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to automatic
 * - Requests a transfers of blocks ranging from 8 to max_block_size bytes
 *   in steps od 8
 * - Performs a checksum on the data transferred
 */
static void
test_src_dst_sw_handshaking_block_sizes(void);

/*!
 * \brief Direct peripheral to peripheral transfer using source and destination
 * software handshaking and source flow control.
 *
 * \par Test 12: (tests_to_run: 0x00000800)
 *  The test uses memory locations as dummy source and destination, and
 * does the following:
 * - Fills the block with pseudo random data
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to automatic
 * - Transfers a block size of max_block_size
 * - Performs a checksum on the data transferred
 */
static void
test_src_dst_sw_hand_src_flow_controller(void);

/*!
 * \brief Direct peripheral to peripheral transfer using source and destination
 * software handshaking and destination flow control.
 *
 * \par Test 13: (tests_to_run: 0x00001000)
 * The test uses memory locations as dummy source and destination and
 * does the following:
 * - Fills the block with pseudo random data
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to automatic
 * - Transfers a block size of max_block_size
 * - Performs a checksum on the data transferred
*/
static void
test_src_dst_sw_hand_dst_flow_controller(void);

/*!
 * \brief Direct peripheral to peripheral transfer, source and destination
 * software handshaking, and source flow control with various block sizes.
 *
 * \par Test 14: (tests_to_run: 0x00002000)
 * The test uses memory locations as dummy source and destination and
 * does the following:
 * - Fills the block with pseudo random data
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to automatic
 * - Transfers a block size of max_block_size
 * - Performs a checksum on the data transferred
 */
static void
test_src_dst_sw_handshaking_src_fc_block_sizes(void);

/*!
 * \brief Direct peripheral to peripheral transfer, source and destination
 * software handshaking, and destination flow control with various block sizes.
 *
 * \par Test 15: (tests_to_run: 0x00004000)
 * The test uses memory locations as dummy source and destination and
 * does the following:
 * - Fills the block with pseudo random data
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to automatic
 * - Transfers block sizes from 8 to MAX_BLOCK_SIZE bytes in step of 8.
 * - Performs a checksum on the data transferred
 */
static void
test_src_dst_sw_handshaking_dst_fc_block_sizes(void);

/*!
 * \brief Direct memory to peripheral transfer with destination software
 * handshaking and destination flow control.
 *
 * \par Test 16: (tests_to_run: 0x00008000)
 * The test uses memory locations as a dummy destination and does the following:
 * - Fills the block with pseudo random data
 * - Use PC302_DMA_MASTER2 as the source port
 * - Use PC302_DMA_MASTER3 as the destination port
 * - Use 64 bit transfer width, with a msize set to automatic
 * - Transfers a block size of max_block_size
 * - Performs a checksum on the data transferred
 */
static void
test_dst_sw_hand_dst_flow_controller(void);

/*!
 * \brief Direct peripheral to memory various block size transfers
 * with source software handshaking and source flow control
 *
 * \par Test 17: (tests_to_run: 0x00010000)
 * The test uses memory locations as a dummy source and does the following:
 * - Fills the block with pseudo random data
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to automatic
 * - Transfers block size from 8 to max_block_size in steps of 8
 * - Performs a checksum on the data transferred
 */
static void
test_src_sw_handshaking_src_fc_block_sizes(void);

/*!
 * \brief Test that reloads the transfers list
 *
 * \par Test 18: (tests_to_run: 0x00020000)
 * This test uses a list containing one element on the source side referenced
 * multiple times, and LIST_SIZE separate elements on the destination side.
 * When the test is run the same source element is used multiple times.
 * - Fills the block with pseudo random data
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to 1
 * - Transfers a block size of max_block_size
 * - Performs a checksum on the data transferred
 */
static void
test_reload_to_list_xfr(void);

/*!
 * \brief Test the reuse of DMA descriptors
 *
 * \par Test 19: (tests_to_run: 0x00040000)
 * Test that sets up a list of transfers, performs the transfer, reuses
 * some of the elements of the list, and proves that the second transfer works
 * properly.
 * - Fills the block with pseudo random data
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to 1
 * - Transfers a block size of max_block_size
 * - Performs a checksum on the data transferred
 */
static void
test_list_list_xfr_reuse_list(void);

/*!
 * \brief Direct peripheral to peripheral transfer using hardware handshaking.
 *
 * \par Test 20: (tests_to_run: 0x00080000)
 * This test uses memory locations as dummy source and peripherals and sets
 * up a transfer with a hardware interface. The parameters used are:
 * - Use PC302_DMA_MASTER4 as the source and destination port
 * - Use 64 bit transfer width, with a msize set to 4 
 * \note
 * - This function does not actually transfer any data as the hardware
 *   to drive the handshaking is not setup. This test is only for checking
 *   the register settings.
 * - This test can not be run on DMAC 1 as PC302_DMA_MASTER4 connects to the
 *   AXI2CFG bus rather than the AXI2PICO bus.
 */
static void
test_direct_xfr_hw_handshaking(void);

/*!
 * \brief List with single element transfer test
 *
 * \par Test 21: (tests_to_run: 0x00100000)
 * This test does a memory to memory list transfer using a list size of 1:
 * - Use PC302_DMA_MASTER1 as the source and destination port
 * - Use 64 bit transfer width, with a msize of 1
 * - Transfers a total of 4*MAX_BLOCK_SIZE bytes
 * - Performs a checksum on the data transferred
 */
static void
test_single_element_list_list_xfr(void);

/*!
 * \brief Run all tests specified in structure
 */
static void
run_tests(void);

/*****************************************************************************
 * Global variables & module parameters
 *****************************************************************************/

/*!
 * State of all tests run so far
 */
static int test_state=SUCCESS;

/*!
 * DMAC device handle
 */
static struct device *device;

/*!
 *   Array of tests that will be carried out
 */
static struct test_tag
{
     /** Brief description of test */
    char *name;

    /** Test function pointer     */
    tf test_fn;
}
tests[NUM_TESTS] =
{
    /*! Test 1: 0x00000001 */
    {"Simple direct transfer test", test_direct_xfr}, 

    /*! Test 2: 0x00000002 */
    {"List to list transfer test", test_list_list_xfr}, 

    /*! Test 3: 0x00000004 */
    {"Simple direct transfer test with interrupts", 
             test_direct_xfr_with_interrupts}, 

    /*! Test 4: 0x00000008 */
    {"List to list transfer test with interrupts", 
             test_list_xfr_with_interrupts}, 

    /*! Test 5: 0x00000010 */
    {"List to list transfer test with interrupts and block control", 
             test_list_xfr_with_interrupts2}, 

    /*! Test 6: 0x00000020 */
    {"Simple direct transfers of various block sizes", 
             test_xfr_block_sizes}, 

    /*! Test 7: 0x00000040 */
    {"Simple direct transfer using source software handshaking", 
             test_src_sw_handshaking}, 

    /*! Test 8: 0x00000080 */
    {"Simple direct transfer using destination software handshaking", 
             test_dst_sw_handshaking}, 

    /*! Test 9: 0x00000100 */
    {"Simple direct transfer using source and destination software "
        "handshaking", test_src_dst_sw_handshaking}, 

    /*! Test 10: 0x00000200 */
    {"Simple direct transfer using src/dest sw handshaking "
        "controlled by interrupts", test_src_dst_sw_handshaking_int}, 

    /*! Test 11: 0x00000400 */
    {"Simple direct transfer, src/dest sw handshaking, various block sizes", 
             test_src_dst_sw_handshaking_block_sizes}, 

    /*! Test 12: 0x0000800 */
    {"Simple direct transfer, src/dest sw handshaking, source flow "
             "controller", test_src_dst_sw_hand_src_flow_controller}, 

    /*! Test 13: 0x00001000 */
    {"Simple direct transfer, src/dest sw handshaking, dest flow controller", 
             test_src_dst_sw_hand_dst_flow_controller}, 

    /*! Test 14: 0x00002000 */
    {"Simple direct transfer, src/dest sw handshaking, source FC & various "
        "block sizes", test_src_dst_sw_handshaking_src_fc_block_sizes}, 

    /*! Test 15: 0x00004000 */
    {"Simple direct transfer, src/dest sw handshaking, dest FC & various "
        "block sizes", test_src_dst_sw_handshaking_dst_fc_block_sizes}, 

    /*! Test 16: 0x00008000 */
    {"Simple direct transfer, dest sw handshaking, dest flow controller", 
             test_dst_sw_hand_dst_flow_controller}, 

    /*! Test 17: 0x00010000 */
    {"Simple direct transfer, src sw handshaking, source FC & various "
        "block sizes", test_src_sw_handshaking_src_fc_block_sizes}, 

    /*! Test 18: 0x00020000 */
    {"Reload Block to list transfer test", test_reload_to_list_xfr}, 

    /*! Test 19: 0x00040000 */
    {"List to list transfer test, reuse list", test_list_list_xfr_reuse_list}, 

    /*! Test 20: 0x00080000 */
    {"Simple direct transfer test using hardware handshaking", 
             test_direct_xfr_hw_handshaking}, 

    /*! Test 21: 0x00100000 */
    {"Single element list to list transfer test", 
             test_single_element_list_list_xfr}, 
};

/*****************************************************************************
 * Private function declarations
 *****************************************************************************/

/* Helper functions */

/*****************************************************************************
   Return the DMAC handle
******************************************************************************/
static pc302_dma_t
get_dmac(int num)
{
    pc302_dma_t dma = pc302_dma_get_dma_handle(num);

    if (!dma)
    {
        FAIL_TEST("pc302_dma_get_dma_handle");
        return NULL;
    }
    else
    {
        printk("Returning handle for DMAC %d\n", num);
        return dma;
    }
}

/*****************************************************************************
   Fill the specified area of memory with random data.
   Use data on the stack as a way of generating pseudo random data

   This function uses the address of 'i' in the stack as a starting
   point to produce pseudo random data
******************************************************************************/
static void
rand_fill(void *vaddr, 
          unsigned size)
{
    unsigned i=0;
    unsigned *p=(unsigned int*)(vaddr);

    for (i = 0; i < size/4; i++)
    {
        p[i]  = *(&i + i);     /* any random data on stack */
    }
}

/*****************************************************************************
   This function performs a checksum for the area of memory specified
******************************************************************************/
static unsigned
checksum(void *vaddr, 
         unsigned size)
{
    unsigned i=0;
    unsigned *p=NULL;
    unsigned chk = 0;
    unsigned bits = 0;

    for (i = 0; i < size; i += 4)
    {
        p = (unsigned *)(((unsigned char *)vaddr) + i);

        chk = chk + *p;     /* Checksum */
        bits = bits | *p;
    }

    /* if all data is zero, return nonzero */
    if (bits == 0)
    {
        return -1;
    }

    return chk;
}

/*****************************************************************************
   Allocate an area of non cached memory
******************************************************************************/
static dma_addr_t
get_dma_buffer(void **vaddr, 
               unsigned size, 
               unsigned do_checksum)
{
    dma_addr_t paddr=0x0;
    unsigned *p=NULL;

    *vaddr = dma_alloc_coherent(device, size, &paddr, GFP_KERNEL);
    if (!(*vaddr))
    {
        FAIL_TEST("dma_alloc_coherent");
        return 0;
    }

    if (do_checksum)
    {
        rand_fill(*vaddr, size);
        /* Put the -ve checksum at the end */
        p = (unsigned *)(((unsigned char *)(*vaddr)) + size-4);
        *p = -checksum(*vaddr, size - 4);
    }
    else
    {
        /* Fill buffer with a constant octet */
        memset(*vaddr, TEST_VALUE, size);
    }

    return paddr;
}

/*****************************************************************************
   Perform a small delay be scheduling a timeout with the number of
   jiffies supplied
******************************************************************************/
static void
shortDelay(void)
{
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(SMALL_DELAY); /* Delay in jiffies */
}

/* Test functions... */
/*****************************************************************************
   Simple direct transfer test with no interrupts
******************************************************************************/
static void
test_direct_xfr(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr=NULL;
    void *src_vaddr=NULL;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Set debug level */
    pc302_dma_set_debug_level(dma, 0xFFFF /* Max level */);

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_1_TRW;
    src.auto_reload = 0;

    dst = src;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, max_block_size, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    udelay(100);

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
    pc302_dma_set_debug_level(dma, 0);
}

/*****************************************************************************
   Direct transfer using hardware handshaking.
******************************************************************************/
static void
test_direct_xfr_hw_handshaking(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_handshake_t src_hs= {0, 0};
    void *dest_vaddr=NULL;
    void *src_vaddr=NULL;

    /* DMAC2 does not have an H/W interfaces to the AXI2PICO bus */
    if (dmac_to_test == 0)
    {
        /* Get a handle to a controller */
        dma = get_dmac(dmac_to_test);
        if (TEST_FAILED)
        {
            goto end;
        }

        /* Setup src hardware handshaking parameters */
        src_hs.hwInterface = 1;
        src_hs.active_low = 0;

        /* Setup src/dest parameters */
        src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
        src.master = PC302_DMA_MASTER4;
        src.periph_not_mem = 1;
        src.flow_controller = 0;
        src.enable_sg = 0;
        src.addr_inc = PC302_DMA_ADDR_NO_CHANGE;
        src.tr_width = PC302_DMA_TR_WIDTH64;
        src.msize = PC302_DMA_MS_4_TRW;
        src.auto_reload = 0;

        dst = src;
        dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

        xfr = pc302_dma_setup_direct_xfr(
                dma, &src, &dst, &src_hs, NULL, max_block_size*2, 
                PC302_DMA_PROTCTL_1, NULL, NULL);
        if (!xfr)
        {
            FAIL_TEST("pc302_dma_setup_direct_xfr");
            goto end;
        }

        if (pc302_dma_start(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_start");
            goto end;
        }

        pc302_dma_dump_regs(dma);

        udelay(100);

end:
        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        } 

        if (dest_vaddr)
        {
            dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
        }
        if (src_vaddr)
        {
            dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
        }
    }
}

/*****************************************************************************
   Simple direct transfer test without interrupts using various block sizes
******************************************************************************/
static void
test_xfr_block_sizes(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr=NULL;
    void *src_vaddr=NULL;
    unsigned bsize=0;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst = src;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);
    /* Test block sizes */
    for (bsize = 8; bsize < max_block_size; bsize += 8)
    {
        memset(dest_vaddr, TEST_VALUE, max_block_size);

        xfr = pc302_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL, NULL);
        if (!xfr)
        {
            FAIL_TEST("pc302_dma_setup_direct_xfr");
            break;
        }

        if (pc302_dma_start(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_start");
            break;
        }

        udelay(100);

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0)
        {
            FAIL_TEST("Check destination");
            break;
        }

        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        } 
        xfr = NULL;
    }

    if (bsize < max_block_size)
    {
        printk("Failed at block size %u\n", bsize);
    }

end:
    if (xfr)
    {
        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        }
    }

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple transfer of an entire list without interrupts
******************************************************************************/
static void
test_list_list_xfr(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src[LIST_SIZE]; /* Cannot be initialised statically */
    pc302_dma_endpoint_t dst[LIST_SIZE];
    pc302_dma_list_t list=NULL;
    void *dest_vaddr[LIST_SIZE];
    void *src_vaddr[LIST_SIZE];
    unsigned i=0;
    unsigned buf_sz = max_block_size;

    for (i = 0; i < LIST_SIZE; i++)
    {
        src_vaddr[i] = NULL;
        dest_vaddr[i] = NULL;
    }

    /* Initialize endpoints */
    memset(src, 0, sizeof(pc302_dma_endpoint_t)*LIST_SIZE);
    memset(dst, 0, sizeof(pc302_dma_endpoint_t)*LIST_SIZE);

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    list = pc302_dma_list_create(dma, LIST_SIZE);
    if (!list)
    {
        FAIL_TEST("pc302_dma_list_create");
        goto end;
    }

    /* Setup 10 elements of the list */
    for (i = 0; i < LIST_SIZE; i++)
    {
        /* Setup src/dest parameters */
        src[i].dma_addr = get_dma_buffer(&src_vaddr[i], buf_sz, 1);
        src[i].master = PC302_DMA_MASTER1;
        src[i].periph_not_mem = 0;
        src[i].flow_controller = 0;
        src[i].enable_sg = 0;
        src[i].addr_inc = PC302_DMA_ADDR_INCREMENT;
        src[i].tr_width = PC302_DMA_TR_WIDTH64;
        src[i].msize = PC302_DMA_MS_1_TRW;
        src[i].auto_reload = 0;

        dst[i] = src[i];
        dst[i].dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (pc302_dma_list_add(list, &src[i], &dst[i], NULL, NULL, buf_sz) != 0)
        {
            FAIL_TEST("pc302_dma_list_add");
            goto end;
        }
    }

    xfr = pc302_dma_setup_list_xfr(list, NULL, NULL, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_list_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    shortDelay();

    for (i = 0; i < LIST_SIZE; i++)
    {
        if (checksum(dest_vaddr[i], buf_sz) != 0)
        {
            FAIL_TEST("Checksum destination");
        }
    }

    /* Check contents of list */
    pc302_dma_dump_regs(dma);

end:
    /* Destroy the transfer */
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    /* Destroy the list */
    if (pc302_dma_list_destroy(list) != 0)
    {
        FAIL_TEST("pc302_dma_list_destroy");
    }

    for (i = 0; i < LIST_SIZE; i++)
    {
        if (src_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, src_vaddr[i], src[i].dma_addr);
        }
        if (dest_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, dest_vaddr[i], dst[i].dma_addr);
        }
    }
}

/*****************************************************************************
   Single element list transfer test
******************************************************************************/
static void
test_single_element_list_list_xfr(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_list_t list=NULL;
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned buf_sz = max_block_size;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    list = pc302_dma_list_create(dma, 1);
    if (!list)
    {
        FAIL_TEST("pc302_dma_list_create");
        goto end;
    }

    /* Setup list of 1 element */
    src.dma_addr = get_dma_buffer(&src_vaddr, buf_sz, 1);
    src.master = PC302_DMA_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_1_TRW;
    src.auto_reload = 0;

    dst = src;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, buf_sz, 0);

    if (pc302_dma_list_add(list, &src, &dst, NULL, NULL, buf_sz) != 0)
    {
        FAIL_TEST("pc302_dma_list_add");
        goto end;
    }

    xfr = pc302_dma_setup_list_xfr(list, NULL, NULL, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_list_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    shortDelay();

    if (checksum(dest_vaddr, buf_sz) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

end:
    /* Destroy the transfer */
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    /* Destroy the list */
    if (pc302_dma_list_destroy(list) != 0)
    {
        FAIL_TEST("pc302_dma_list_destroy");
        goto end;
    }

    if (src_vaddr)
    {
        dma_free_coherent(device, buf_sz, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, buf_sz, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Test that of transfers can be reused
******************************************************************************/
static void
test_list_list_xfr_reuse_list(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src[LIST_SIZE+REUSE_LIST_SIZE];
    pc302_dma_endpoint_t dst[LIST_SIZE+REUSE_LIST_SIZE];
    pc302_dma_list_t list=NULL;
    void *dest_vaddr[LIST_SIZE+REUSE_LIST_SIZE];
    void *src_vaddr[LIST_SIZE+REUSE_LIST_SIZE];
    unsigned i=0;
    unsigned buf_sz = max_block_size;

    for (i = 0; i < (LIST_SIZE+REUSE_LIST_SIZE); i++)
    {
        src_vaddr[i] = NULL;
        dest_vaddr[i] = NULL;
    }

    /* Initialize endpoints */
    memset(src, 0, sizeof(pc302_dma_endpoint_t)*(LIST_SIZE+REUSE_LIST_SIZE));
    memset(dst, 0, sizeof(pc302_dma_endpoint_t)*(LIST_SIZE+REUSE_LIST_SIZE));

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    list = pc302_dma_list_create(dma, LIST_SIZE);
    if (!list)
    {
        FAIL_TEST("pc302_dma_list_create");
        goto end;
    }

    xfr = pc302_dma_setup_list_xfr(list, NULL, NULL, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_list_xfr");
        goto end;
    }

    /* Setup LIST_SIZE elements of the list for first transfer */
    for (i = 0; i < LIST_SIZE; i++)
    {
        /* Setup src/dest parameters */
        src[i].dma_addr = get_dma_buffer(&src_vaddr[i], buf_sz, 1);
        src[i].master = PC302_DMA_MASTER1;
        src[i].periph_not_mem = 0;
        src[i].flow_controller = 0;
        src[i].enable_sg = 0;
        src[i].addr_inc = PC302_DMA_ADDR_INCREMENT;
        src[i].tr_width = PC302_DMA_TR_WIDTH64;
        src[i].msize = PC302_DMA_MS_1_TRW;
        src[i].auto_reload = 0;

        dst[i] = src[i];
        dst[i].dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (pc302_dma_list_add(list, &src[i], &dst[i], NULL, NULL, buf_sz) != 0)
        {
            FAIL_TEST("pc302_dma_list_add");
            goto end;
        }
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    shortDelay();

    for (i = 0; i < LIST_SIZE; i++)
    {
        if (checksum(dest_vaddr[i], buf_sz) != 0)
        {
            FAIL_TEST("Checksum destination");
        }
    }

    /* Reset the list and reuse it for second transfer */
    if (pc302_dma_list_clear(list) != 0)
    {
        FAIL_TEST("pc302_dma_list_clear");
        goto end;
    }

    /* Setup REUSE_LIST_SIZE elements of the list */
    for (i = LIST_SIZE; i < (LIST_SIZE+REUSE_LIST_SIZE); i++)
    {
        /* Setup src/dest parameters */
        src[i].dma_addr = get_dma_buffer(&src_vaddr[i], buf_sz, 1);
        src[i].master = PC302_DMA_MASTER1;
        src[i].periph_not_mem = 0;
        src[i].flow_controller = 0;
        src[i].enable_sg = 0;
        src[i].addr_inc = PC302_DMA_ADDR_INCREMENT;
        src[i].tr_width = PC302_DMA_TR_WIDTH64;
        src[i].msize = PC302_DMA_MS_1_TRW;
        src[i].auto_reload = 0;

        dst[i] = src[i];
        dst[i].dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);
        if (pc302_dma_list_add(list, &src[i], &dst[i], NULL, NULL, buf_sz) != 0)
        {
            FAIL_TEST("pc302_dma_list_add");
            goto end;
        }
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    shortDelay();

    for (i = 0; i < REUSE_LIST_SIZE; i++)
    {
        if (checksum(dest_vaddr[i], buf_sz) != 0)
        {
            FAIL_TEST("Checksum destination");
        }
    }

end:
    /* Destroy the transfer */
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    /* Destroy the list */
    if (pc302_dma_list_destroy(list) != 0)
    {
        FAIL_TEST("pc302_dma_list_destroy");
        goto end;
    }

    for (i = 0; i < (LIST_SIZE+REUSE_LIST_SIZE); i++)
    {
        if (src_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, src_vaddr[i], src[i].dma_addr);
        }
        if (dest_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, dest_vaddr[i], dst[i].dma_addr);
        }
    }
}

/*****************************************************************************
   Test that reloads the transfers list
******************************************************************************/
static void
test_reload_to_list_xfr(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst[LIST_SIZE];
    pc302_dma_list_t list=NULL;
    void *dest_vaddr[LIST_SIZE];
    void *src_vaddr = NULL;
    unsigned i=0;
    unsigned buf_sz = max_block_size;

    for (i = 0; i < (LIST_SIZE); i++)
    {
        dest_vaddr[i] = NULL;
    }

    /* Initialize endpoints */
    memset(dst, 0, sizeof(pc302_dma_endpoint_t)*LIST_SIZE);

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    list = pc302_dma_list_create(dma, LIST_SIZE);
    if (!list)
    {
        FAIL_TEST("pc302_dma_list_create");
        goto end;
    }

    /* Setup src/dest parameters */
    src.master = PC302_DMA_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_1_TRW;

    /* Reload the source block so that the source block get copied to all
     * list elements of the destination
     */
    src.auto_reload = 1;

    dst[0].master = PC302_DMA_MASTER1;
    dst[0].periph_not_mem = 0;
    dst[0].flow_controller = 0;
    dst[0].enable_sg = 0;
    dst[0].addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst[0].tr_width = PC302_DMA_TR_WIDTH64;
    dst[0].msize = PC302_DMA_MS_1_TRW;
    dst[0].auto_reload = 0;

    src.dma_addr = get_dma_buffer(&src_vaddr, buf_sz, 1);
    dst[0].dma_addr = get_dma_buffer(&dest_vaddr[0], buf_sz, 0);

    /* Setup first element of the list */
    if (pc302_dma_list_add(list, &src, &dst[0], NULL, NULL, buf_sz) != 0)
    {
        FAIL_TEST("pc302_dma_list_add");
        goto end;
    }

    /* Setup elements 2 to LIST_SIZE of the list */
    for (i = 1; i < LIST_SIZE; i++)
    {
        dst[i] = dst[0];
        dst[i].dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        /* No source endpoint for subsequent elements */
        if (pc302_dma_list_add(list, NULL, &dst[i], NULL, NULL, buf_sz) != 0)
        {
            FAIL_TEST("pc302_dma_list_add");
            goto end;
        }
    }

    xfr = pc302_dma_setup_list_xfr(list, NULL, NULL, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_list_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    shortDelay();

    for (i = 0; i < LIST_SIZE; i++)
    {
        if (checksum(dest_vaddr[i], buf_sz) != 0)
        {
            FAIL_TEST("Checksum destination");
        }
    }

end:
    /* Destroy the transfer */
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    /* Destroy the list */
    if (pc302_dma_list_destroy(list) != 0)
    {
        FAIL_TEST("pc302_dma_list_destroy");
        goto end;
    }

    if (src_vaddr)
    {
        dma_free_coherent(device, buf_sz, src_vaddr, src.dma_addr);
    }
    for (i = 0; i < (LIST_SIZE); i++)
    {
        if (dest_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, dest_vaddr[i], dst[i].dma_addr);
        }
    }
}

/*****************************************************************************
   Interrupt handler for the Direct transfer test using interrupts
******************************************************************************/
static int
test_direct_xfr_with_interrupts_irq(void *cookie, 
                                    int errno)
{
    return (*((int *)cookie))++;
}

/*****************************************************************************
   Direct transfer test using interrupts
******************************************************************************/
static void
test_direct_xfr_with_interrupts(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned irq_state = 0;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_1_TRW;
    src.auto_reload = 0;

    dst = src;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, max_block_size, 0, 
                             &test_direct_xfr_with_interrupts_irq, &irq_state);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }

    if (pc302_dma_enable_int(xfr, PC302_DMA_INT_BLOCK |
           PC302_DMA_INT_ERROR | PC302_DMA_INT_TRANSFER) != 0)
    {
       FAIL_TEST("pc302_dma_enable_int");
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    udelay(100);

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

    if (irq_state != 1)
    {
        FAIL_TEST("Wrong number of interrupts occurred");
    }
    
    printk("%u interrupts received\n", irq_state);

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (pc302_dma_disable_int(xfr, PC302_DMA_INT_BLOCK |
           PC302_DMA_INT_ERROR | PC302_DMA_INT_TRANSFER) != 0)
    {
        FAIL_TEST("pc302_dma_disable_int");
    }

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Interrupt handler for the List transfer test using interrupts
******************************************************************************/
static int
test_list_xfr_with_interrupts_irq(void *cookie, 
                                  int errno)
{
    return (*((int *)cookie))++;
}

/*****************************************************************************
   List transfer test using interrupts
******************************************************************************/
static void
test_list_xfr_with_interrupts(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src[LIST_SIZE];
    pc302_dma_endpoint_t dst[LIST_SIZE];
    pc302_dma_list_t list=NULL;
    void *dest_vaddr[LIST_SIZE];
    void *src_vaddr[LIST_SIZE];
    unsigned i=0;
    unsigned buf_sz = max_block_size;
    unsigned irq_state = 0;

    for (i = 0; i < LIST_SIZE; i++)
    {
        src_vaddr[i] = NULL;
        dest_vaddr[i] = NULL;
    }

    /* Initialize endpoints */
    memset(src, 0, sizeof(pc302_dma_endpoint_t)*LIST_SIZE);
    memset(dst, 0, sizeof(pc302_dma_endpoint_t)*LIST_SIZE);

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    list = pc302_dma_list_create(dma, LIST_SIZE);
    if (!list)
    {
        FAIL_TEST("pc302_dma_list_create");
        goto end;
    }

    /* Setup src/dest parameters */
    src[0].master = PC302_DMA_MASTER1;
    src[0].periph_not_mem = 0;
    src[0].flow_controller = 0;
    src[0].enable_sg = 0;
    src[0].addr_inc = PC302_DMA_ADDR_INCREMENT;
    src[0].tr_width = PC302_DMA_TR_WIDTH64;
    src[0].msize = PC302_DMA_MS_1_TRW;
    src[0].auto_reload = 0;

    /* Setup LIST_SIZE elements of the list */
    for (i = 0; i < LIST_SIZE; i++)
    {
        dst[i] = src[0];
        src[i] = src[0];
        src[i].dma_addr = get_dma_buffer(&src_vaddr[i], buf_sz, 1);
        dst[i].dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (pc302_dma_list_add(list, &src[i], &dst[i], NULL, NULL, buf_sz) != 0)
        {
            FAIL_TEST("pc302_dma_list_add");
            goto end;
        }
    }

    xfr = pc302_dma_setup_list_xfr(list, NULL, NULL, 0, 
                   &test_list_xfr_with_interrupts_irq, &irq_state);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_list_xfr");
        goto end;
    }

    if (pc302_dma_enable_int(xfr, PC302_DMA_INT_BLOCK |
              PC302_DMA_INT_ERROR | PC302_DMA_INT_TRANSFER) != 0)
    {
       FAIL_TEST("pc302_dma_enable_int");
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    shortDelay();

    for (i = 0; i < LIST_SIZE; i++)
    {
        if (checksum(dest_vaddr[i], buf_sz) != 0)
        {
            FAIL_TEST("Checksum destination");
        }
    }

    if (irq_state != LIST_SIZE)
    {
        FAIL_TEST("Wrong number of interrupts occurred");
    }

end:
    /* Destroy the transfer */
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    /* Destroy the list */
    if (pc302_dma_list_destroy(list) != 0)
    {
        FAIL_TEST("pc302_dma_list_destroy");
        goto end;
    }

    printk("%u interrupts received\n", irq_state);

    if (pc302_dma_disable_int(xfr, PC302_DMA_INT_BLOCK |
              PC302_DMA_INT_ERROR | PC302_DMA_INT_TRANSFER) != 0)
    {
        FAIL_TEST("pc302_dma_disable_int");
    }

    for (i = 0; i < LIST_SIZE; i++)
    {
        if (src_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, src_vaddr[i], src[i].dma_addr);
        }
        if (dest_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, dest_vaddr[i], dst[i].dma_addr);
        }
    }
}

/*****************************************************************************
   List to list transfer test with interrupts and block control
******************************************************************************/
static int
test_list_xfr_with_interrupts2_irq(void *cookie, 
                                   int errno)
{
    client_data_interrupt_test_t *p = (client_data_interrupt_test_t *)cookie;
    unsigned valid_irq = 0;

    p->irq_state++;

    if (errno & PC302_DMA_INT_BLOCK)
    {
        p->tlxwi2_block_ints++;
        p->blocks_left--;
        valid_irq = 1;
    }

    if (errno &  PC302_DMA_INT_DST_TRANSACTION)
    {
        p->tlxwi2_dsttrx_ints++;
        valid_irq = 1;
    }

    if (errno & PC302_DMA_INT_ERROR)
    {
        p->tlxwi2_err_ints++;
        valid_irq = 1;
    }

    if (errno & PC302_DMA_INT_SRC_TRANSACTION)
    {
        p->tlxwi2_srctrx_ints++;
        valid_irq = 1;
    }

    if (errno & PC302_DMA_INT_TRANSFER)
    {
        p->tlxwi2_trf_ints++;
        valid_irq = 1;
    }

    if (!valid_irq)
    {
        char string[32];
        (void)sprintf(string,"BAD INT TYPE (%d)\n",errno);
        FAIL_TEST(string);
        return -EINVAL;
    }

    return 0;
}

/*****************************************************************************
   List to list transfer test with interrupts and block control.
******************************************************************************/
static void
test_list_xfr_with_interrupts2(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src[LIST_SIZE];
    pc302_dma_endpoint_t dst[LIST_SIZE];
    pc302_dma_list_t list=NULL;
    void *dest_vaddr[LIST_SIZE];
    void *src_vaddr[LIST_SIZE];
    unsigned i=0;
    unsigned buf_sz = max_block_size;
    client_data_interrupt_test_t p = {0, 0, 0, 0, 0, 0, 0};

    for (i = 0; i < LIST_SIZE; i++)
    {
        src_vaddr[i] = NULL;
        dest_vaddr[i] = NULL;
    }

    /* Initialize endpoints */
    memset(src, 0, sizeof(pc302_dma_endpoint_t)*LIST_SIZE);
    memset(dst, 0, sizeof(pc302_dma_endpoint_t)*LIST_SIZE);

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    list = pc302_dma_list_create(dma, LIST_SIZE);
    if (!list)
    {
        FAIL_TEST("pc302_dma_list_create");
        goto end;
    }

    /* Setup src/dest parameters */
    src[0].master = PC302_DMA_MASTER1;
    src[0].periph_not_mem = 0;
    src[0].flow_controller = 0;
    src[0].enable_sg = 0;
    src[0].addr_inc = PC302_DMA_ADDR_INCREMENT;
    src[0].tr_width = PC302_DMA_TR_WIDTH64;
    src[0].msize = PC302_DMA_MS_1_TRW;
    src[0].auto_reload = 0;

    /* Setup elements of the list */
    for (i = 0; i < LIST_SIZE; i++)
    {
        dst[i] = src[0];
        src[i] = src[0];

        src[i].dma_addr = get_dma_buffer(&src_vaddr[i], buf_sz, 1);
        dst[i].dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (pc302_dma_list_add(list, &src[i], &dst[i], NULL, NULL, buf_sz) != 0)
        {
            FAIL_TEST("pc302_dma_list_add");
            goto end;
        }
    }

    xfr = pc302_dma_setup_list_xfr(list, NULL, NULL, 0, 
                  &test_list_xfr_with_interrupts2_irq, &p);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_list_xfr");
        goto end;
    }

    if (pc302_dma_enable_int(xfr, PC302_DMA_INT_BLOCK |
        PC302_DMA_INT_ERROR | PC302_DMA_INT_TRANSFER) != 0)
    {
       FAIL_TEST("pc302_dma_enable_int");
    }

    /* Set the blocks counter so that the driver can stop the transfer
       when we get to the end of the block list.  */
    p.blocks_left = LIST_SIZE;

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    shortDelay();

    /* The transfer should have stopped by itself. Check that the
       remaining blocks counter reached zero.  */
    if (p.blocks_left != 0)
    {
        FAIL_TEST("Blocks left not zero!");
    }

    for (i = 0; i < LIST_SIZE; i++)
    {
        if (checksum(dest_vaddr[i], buf_sz) != 0)
        {
            FAIL_TEST("Checksum destination");
        }
    }

    if (p.irq_state != LIST_SIZE)
    {
        FAIL_TEST("Wrong number of total interrupts received");
    }
    if (p.tlxwi2_trf_ints != 1)
    {
        FAIL_TEST("Wrong number of transfer complete interrupts occurred");
    }
    if (p.tlxwi2_block_ints != LIST_SIZE)
    {
        FAIL_TEST("Wrong number of block interrupts occurred");
    }
    if (p.tlxwi2_srctrx_ints != 0)
    {
        FAIL_TEST("Wrong number of total interrupts occurred");
    }
    if (p.tlxwi2_dsttrx_ints != 0)
    {
        FAIL_TEST("Wrong number of total interrupts occurred");
    }
    if (p.tlxwi2_err_ints != 0)
    {
        FAIL_TEST("Error interrupts have occured");
    }

    printk("%u total interrupts received\n", p.irq_state);
    printk("%u TRF interrupts received\n", p.tlxwi2_trf_ints);
    printk("%u BLOCK interrupts received\n", p.tlxwi2_block_ints);
    printk("%u SRC Trans interrupts received\n", p.tlxwi2_srctrx_ints);
    printk("%u DST Trans interrupts received\n", p.tlxwi2_dsttrx_ints);
    printk("%u ERR interrupts received\n", p.tlxwi2_err_ints);
    printk("%u blocks left\n", p.blocks_left);

end:
    /* Destroy the transfer */
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    /* Destroy the list */
    if (pc302_dma_list_destroy(list) != 0)
    {
        FAIL_TEST("pc302_dma_list_destroy");
        goto end;
    }

    if (pc302_dma_disable_int(xfr, PC302_DMA_INT_BLOCK |
        PC302_DMA_INT_ERROR | PC302_DMA_INT_TRANSFER) != 0)
    {
        FAIL_TEST("pc302_dma_disable_int");
    }

    for (i = 0; i < LIST_SIZE; i++)
    {
        if (src_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, src_vaddr[i], src[i].dma_addr);
        }
        if (dest_vaddr[i])
        {
            dma_free_coherent(device, buf_sz, dest_vaddr[i], dst[i].dma_addr);
        }
    }
}

/*****************************************************************************
   Simple direct transfer using source software handshaking
******************************************************************************/
static void
test_src_sw_handshaking(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr=NULL;
    void *src_vaddr=NULL;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
       transaction requests on the source to progress the transfer */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;
    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, max_block_size, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    /* Request transactions until the transfer stops */
    while (pc302_dma_request_transaction(
                          xfr, PC302_DMA_SRC, NULL) != -EINVAL) ;

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple direct transfer using destination software handshaking
******************************************************************************/
static void
test_dst_sw_handshaking(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr=NULL;
    void *src_vaddr=NULL;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
       to do transaction requests on the destination to progress the transfer */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, max_block_size, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    /* Request transactions until the transfer stops */
    while (pc302_dma_request_transaction(xfr, 
                               PC302_DMA_DST, NULL) != -EINVAL) ;

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
    Simple direct transfer using source and destination software handshaking.
******************************************************************************/
static void
test_src_dst_sw_handshaking(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
       transaction requests on the source to progress the transfer */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
       to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, max_block_size, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    /* Request transactions until the transfer stops */
    while (1)
    {
        unsigned rc_s, rc_d;
        rc_s = pc302_dma_request_transaction(xfr, PC302_DMA_SRC, NULL);
        rc_d = pc302_dma_request_transaction(xfr, PC302_DMA_DST, NULL);
        if (rc_s == -EINVAL && rc_d == -EINVAL)
        {
            break;
        }
    }

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple direct transfer using source and destination software
   handshaking and source flow control.
******************************************************************************/
static void
test_src_dst_sw_hand_src_flow_controller(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned bytes_left = max_block_size;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
       transaction requests on the source to progress the transfer */
    src.periph_not_mem = 1;

    /* Make the source is the flow controller. This means that the source
       endpoint will control when the end of a block arrives through the
       bytes_left parameter of the pc302_dma_request_transaction
       function.  */
    src.flow_controller = 1;

    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
       to do transaction requests on the destination to progress the transfer */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, bytes_left, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, bytes_left, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    /* Request transactions until the transfer stops */
    while (1)
    {
        unsigned rc_s, rc_d;
        rc_s = pc302_dma_request_transaction(xfr, PC302_DMA_SRC, &bytes_left);
        rc_d = pc302_dma_request_transaction(xfr, PC302_DMA_DST, NULL);
        if (rc_s == -EINVAL && rc_d == -EINVAL)
        {
            break;
        }
    }

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple direct transfer using source and destination software
   handshaking and destination flow control.
******************************************************************************/
static void
test_src_dst_sw_hand_dst_flow_controller(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned bytes_left=0;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
       transaction requests on the source to progress the transfer */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
       to do transaction requests on the destination to progress the transfer */
    dst.periph_not_mem = 1;

    /* Make the destination the flow controller. This means that the
       destination endpoint will control when the end of a block arrives
       through the bytes_left parameter of the
       pc302_dma_request_transaction function.  */
    dst.flow_controller = 1;

    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, bytes_left, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    /* Request transactions until the transfer stops */
    bytes_left = max_block_size;
    while (1)
    {
        unsigned rc_s, rc_d;
        rc_s = pc302_dma_request_transaction(xfr, PC302_DMA_SRC, NULL);
        rc_d = pc302_dma_request_transaction(xfr, PC302_DMA_DST, &bytes_left);
        if (rc_s == -EINVAL && rc_d == -EINVAL)
        {
            break;
        }
    }

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple direct transfer, dest sw handshaking, dest flow controller
******************************************************************************/
static void
test_dst_sw_hand_dst_flow_controller(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned bytes_left = 0;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER2;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER3;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
       to do transaction requests on the destination to progress the transfer */
    dst.periph_not_mem = 1;

    /* Make the destination the flow controller. This means that the destination
       endpoint will control when the end of a block arrives through the
       bytes_left parameter of the pc302_dma_request_transaction function.  */
    dst.flow_controller = 1;

    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, bytes_left, 0, NULL, NULL);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    /* Request transactions until the transfer stops */
    bytes_left = max_block_size;
    while (1)
    {
        unsigned rc_d;
        rc_d = pc302_dma_request_transaction(xfr, PC302_DMA_DST, &bytes_left);
        if (rc_d == -EINVAL)
        {
            break;
        }
    }

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple direct transfer, src / dest sw handshaking, various block sizes
******************************************************************************/
static void
test_src_dst_sw_handshaking_block_sizes(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned bsize = 0;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
       transaction requests on the source to progress the transfer */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
       to do transaction requests on the destination to progress the transfer */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    /* Test block sizes */
    for (bsize = 8; bsize < max_block_size; bsize += 8)
    {
        memset(dest_vaddr, TEST_VALUE, max_block_size);

        xfr = pc302_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL, NULL);
        if (!xfr)
        {
            FAIL_TEST("pc302_dma_setup_direct_xfr");
            break;
        }

        if (pc302_dma_start(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_start");
            break;
        }

        /* Request transactions until the transfer stops */
        while (1)
        {
            unsigned rc_s, rc_d;
            rc_s = pc302_dma_request_transaction(xfr, PC302_DMA_SRC, NULL);
            rc_d = pc302_dma_request_transaction(xfr, PC302_DMA_DST, NULL);
            if (rc_s == -EINVAL && rc_d == -EINVAL)
            {
                break;
            }
        }

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0)
        {
            FAIL_TEST("Check destination");
            break;
        }

        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        } 
        xfr = NULL;
    }

    if (bsize < max_block_size)
    {
        printk("Failed at block size %u\n", bsize);
    }

end:
    if (xfr)
    {
        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        }
    }
    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple direct transfer, src / dest sw handshaking, source flow
   control and various block sizes
******************************************************************************/
static void
test_src_dst_sw_handshaking_src_fc_block_sizes(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned bsize = 0;
    unsigned bytes_left = 0;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
       transaction requests on the source to progress the transfer */
    src.periph_not_mem = 1;

    /* Make the source the flow controller. This means that the source endpoint
       will control when the end of a block arrives through the bytes_left
       parameter of the pc302_dma_request_transaction function.  */
    src.flow_controller = 1;

    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
       to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    /* Test block sizes */
    for (bsize = 8; bsize < max_block_size; bsize += 8)
    {
        memset(dest_vaddr, TEST_VALUE, max_block_size);

        xfr = pc302_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL, NULL);
        if (!xfr)
        {
            FAIL_TEST("pc302_dma_setup_direct_xfr");
            break;
        }

        if (pc302_dma_start(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_start");
            break;
        }

        /* Request transactions until the transfer stops */
        bytes_left = bsize;
        while (1)
        {
            unsigned rc_s, rc_d;
            rc_s = pc302_dma_request_transaction(xfr, PC302_DMA_SRC, 
                       &bytes_left);
            rc_d = pc302_dma_request_transaction(xfr, PC302_DMA_DST, NULL);
            if (rc_s == -EINVAL && rc_d == -EINVAL)
            {
                break;
            }
        }

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0)
        {
            FAIL_TEST("Check destination");
            break;
        }

        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        } 
        xfr = NULL;
    }

    if (bsize < max_block_size)
    {
        printk("Failed at block size %u\n", bsize);
    }

end:
    if (xfr)
    {
        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        }
    }
    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple direct transfer, source sw handshaking, source flow
   control and various block sizes
******************************************************************************/
static void
test_src_sw_handshaking_src_fc_block_sizes(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned bsize = 0;
    unsigned bytes_left = 0;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
       transaction requests on the source to progress the transfer */
    src.periph_not_mem = 1;

    /* Make the source the flow controller. This means that the source endpoint
       will control when the end of a block arrives through the bytes_left
       parameter of the pc302_dma_request_transaction function.  */
    src.flow_controller = 1;

    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;
    dst.periph_not_mem = 0;
    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    /* Test block sizes */
    for (bsize = 8; bsize < max_block_size; bsize += 8)
    {
        memset(dest_vaddr, TEST_VALUE, max_block_size);

        xfr = pc302_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL, NULL);
        if (!xfr)
        {
            FAIL_TEST("pc302_dma_setup_direct_xfr");
            break;
        }

        if (pc302_dma_start(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_start");
            break;
        }

        /* Request transactions until the transfer stops */
        bytes_left = bsize;
        while (1)
        {
            unsigned rc_s;
            rc_s = pc302_dma_request_transaction(xfr, PC302_DMA_SRC, 
                &bytes_left);
            if (rc_s == -EINVAL)
            {
                break;
            }
        }

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0)
        {
            FAIL_TEST("Check destination");
            break;
        }

        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        } 
        xfr = NULL;
    }

    if (bsize < max_block_size)
    {
        printk("Failed at block size %u\n", bsize);
    }

end:
    if (xfr)
    {
        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        }
    }
    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Simple direct transfer, src / dest sw handshaking, destination
   flow control and various block sizes
******************************************************************************/
static void
test_src_dst_sw_handshaking_dst_fc_block_sizes(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    unsigned bsize = 0;
    unsigned bytes_left = 0;

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
       transaction requests on the source to progress the transfer */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
       to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    /* Make the destination the flow controller. This means that the destination
       endpoint will control when the end of a block arrives through the
       bytes_left parameter of the pc302_dma_request_transaction function.  */
    dst.flow_controller = 1;

    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    /* Test block sizes */
    for (bsize = 8; bsize < max_block_size; bsize += 8)
    {
        memset(dest_vaddr, TEST_VALUE, max_block_size);

        xfr = pc302_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL, NULL);
        if (!xfr)
        {
            FAIL_TEST("pc302_dma_setup_direct_xfr");
            break;
        }

        if (pc302_dma_start(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_start");
            break;
        }

        /* Request transactions until the transfer stops */
        bytes_left = bsize;
        while (1)
        {
            unsigned rc_s, rc_d;
            rc_s = pc302_dma_request_transaction(xfr, PC302_DMA_SRC, NULL);
            rc_d = pc302_dma_request_transaction(xfr, PC302_DMA_DST, 
                &bytes_left);
            if (rc_s == -EINVAL && rc_d == -EINVAL)
            {
                break;
            }
        }

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0)
        {
            FAIL_TEST("Check destination");
            break;
        }

        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        }
        xfr = NULL; 
    }

    if (bsize < max_block_size)
    {
        printk("Failed at block size %u\n", bsize);
    }

end:
    if (xfr)
    {
        if (pc302_dma_release(xfr) != 0)
        {
            FAIL_TEST("pc302_dma_release fail");
        }
    }
    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Interrupt handler for the simple direct transfer test using src / dest
   sw handshaking controlled by interrupts
******************************************************************************/
static int
test_src_dst_sw_handshaking_int_irq(void *cookie, 
                                    int errno)
{
    client_data_interrupt_test_t *p = (client_data_interrupt_test_t *)cookie;
    unsigned valid_irq=0;

    /* Repeat the following, for each interrupt type */
    p->irq_state++;

    if (errno &  PC302_DMA_INT_BLOCK)
    {
        p->tlxwi2_block_ints++;
        valid_irq = 1;
    }

    if (errno &  PC302_DMA_INT_DST_TRANSACTION)
    {
        p->tlxwi2_dsttrx_ints++;
        /* Service the destination transaction complete interrupt
         * by requesting a new transaction */
        if (pc302_dma_request_transaction(p->xfr, 
                PC302_DMA_DST, NULL) == EBUSY)
        {
            FAIL_TEST("pc302_dma_request_transaction Busy");
        }
        valid_irq = 1;
    }

    if (errno &  PC302_DMA_INT_ERROR)
    {
        p->tlxwi2_err_ints++;
        valid_irq = 1;
    }

    if (errno &  PC302_DMA_INT_SRC_TRANSACTION)
    {
        p->tlxwi2_srctrx_ints++;
        /* Service the source transaction complete interrupt
         * by requesting a new transaction */
        if (pc302_dma_request_transaction(p->xfr, 
                PC302_DMA_SRC, NULL) == EBUSY)
        {
            FAIL_TEST("pc302_dma_request_transaction Busy");
        }
        valid_irq = 1;
    }

    if (errno &  PC302_DMA_INT_TRANSFER)
    {
        p->tlxwi2_trf_ints++;
        valid_irq = 1;
    }

    if (!valid_irq)
    {
        char string[32];
        (void)sprintf(string,"BAD INT TYPE %d\n",errno);
        FAIL_TEST(string);
        return -EINVAL;
    }

    return 0;
}

/*****************************************************************************
   Simple direct transfer test using src / dest sw handshaking
   controlled by interrupts
******************************************************************************/
static void
test_src_dst_sw_handshaking_int(void)
{
    pc302_dma_t dma=NULL;
    pc302_dma_xfr_t xfr=NULL;
    pc302_dma_endpoint_t src = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    pc302_dma_endpoint_t dst = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    void *dest_vaddr = NULL;
    void *src_vaddr = NULL;
    client_data_interrupt_test_t p = {0, 0, 0, 0, 0, 0, 0};

    /* Get a handle to a controller */
    dma = get_dmac(dmac_to_test);
    if (TEST_FAILED)
    {
        goto end;
    }

    /* Setup src/dest parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, max_block_size, 1);
    src.master = PC302_DMA_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = PC302_DMA_ADDR_INCREMENT;
    src.tr_width = PC302_DMA_TR_WIDTH64;
    src.msize = PC302_DMA_MS_AUTO;
    src.auto_reload = 0;

    dst.master = PC302_DMA_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = PC302_DMA_ADDR_INCREMENT;
    dst.tr_width = PC302_DMA_TR_WIDTH64;
    dst.msize = PC302_DMA_MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, max_block_size, 0);

    xfr = pc302_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, max_block_size, 0, 
                  &test_src_dst_sw_handshaking_int_irq, &p);
    if (!xfr)
    {
        FAIL_TEST("pc302_dma_setup_direct_xfr");
        goto end;
    }
    p.xfr = xfr;

    if (pc302_dma_enable_int(xfr, 
          PC302_DMA_INT_DST_TRANSACTION |
          PC302_DMA_INT_SRC_TRANSACTION |
          PC302_DMA_INT_ERROR |
          PC302_DMA_INT_TRANSFER) != 0)
    {
        FAIL_TEST("pc302_dma_enable_int"); 
    }

    if (pc302_dma_start(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_start");
        goto end;
    }

    /* Start the process by requesting the first transactions, When these
     * are complete, the interrupt will trigger subsequent ones */
    if (pc302_dma_request_transaction(xfr, PC302_DMA_SRC, NULL) != 0)
    {
        FAIL_TEST("pc302_dma_request_transaction for SRC");
    }
    if (pc302_dma_request_transaction(xfr, PC302_DMA_DST, NULL) != 0)
    {
        FAIL_TEST("pc302_dma_request_transaction for DST");
    }

    shortDelay();

    if (checksum(dest_vaddr, max_block_size) != 0)
    {
        FAIL_TEST("Checksum destination");
    }

    if (max_block_size == 2048)
    {
      /* Can only predict interrupt counts at this block size */
      if (p.irq_state != 8)
      {
          FAIL_TEST("Expected a total of 8 interrupts");
      }
      if (p.tlxwi2_srctrx_ints != 8)
      {
          FAIL_TEST("Expected 8 source transfer complete interrupts");
      }
      if (p.tlxwi2_dsttrx_ints != 8)
      {
          FAIL_TEST("Expected 8 destination transfer complete interrupts");
      }
    }
    if (p.tlxwi2_trf_ints != 1)
    {
        FAIL_TEST("Expected a single transfer complete interrupt");
    }
    if (p.tlxwi2_block_ints != 0)
    {
        FAIL_TEST("Expected no block interrupts");
    }
    if (p.tlxwi2_err_ints != 0)
    {
        FAIL_TEST("Expected 0 error interrupts");
    }

    printk("%u total interrupts received\n", p.irq_state);
    printk("%u TRF interrupts received\n", p.tlxwi2_trf_ints);
    printk("%u BLOCK interrupts received\n", p.tlxwi2_block_ints);
    printk("%u SRC Trans interrupts received\n", p.tlxwi2_srctrx_ints);
    printk("%u DST Trans interrupts received\n", p.tlxwi2_dsttrx_ints);
    printk("%u ERR interrupts received\n", p.tlxwi2_err_ints);

end:
    if (pc302_dma_release(xfr) != 0)
    {
        FAIL_TEST("pc302_dma_release fail");
    } 

    if (pc302_dma_disable_int(xfr, 
          PC302_DMA_INT_DST_TRANSACTION |
          PC302_DMA_INT_SRC_TRANSACTION |
          PC302_DMA_INT_ERROR |
          PC302_DMA_INT_TRANSFER) != 0)
    {
        FAIL_TEST("pc302_dma_disable_int");
    }

    if (src_vaddr)
    {
        dma_free_coherent(device, max_block_size, src_vaddr, src.dma_addr);
    }
    if (dest_vaddr)
    {
        dma_free_coherent(device, max_block_size, dest_vaddr, dst.dma_addr);
    }
}

/*****************************************************************************
   Run all tests specified in structure
******************************************************************************/
static void
run_tests(void)
{
    unsigned res = SUCCESS;
    unsigned testnum=0;

    for (testnum = 0; testnum < NUM_TESTS; testnum++)
    {
        if (tests_to_run & (1 << testnum))
        {
            printk(KERN_INFO "dmat: <%u> START %s\n", testnum +1,
                tests[testnum].name);

            RESET_TEST_RES();

            tests[testnum].test_fn();

            printk(KERN_INFO "dmat: TEST <%u> %s\n", testnum + 1, 
                    (test_state == FAIL) ? "FAILED" : "SUCCESS");

            if (test_state == FAIL)
            {
                res = FAIL;
            }
        }
    }

    if (res == SUCCESS)
    {
        printk(KERN_INFO "dmat: Passed all tests.\n");
    }
    else
    {
        printk(KERN_INFO "dmat: FAILED SOME TESTS!!\n");
    }
}

/*****************************************************************************
   Driver test init function
******************************************************************************/
static int __init pc302_dma_test_module_init(void)
{
    printk(KERN_INFO "%s " CONFIG_LOCALVERSION " " __DATE__ " " __TIME__
            " Loaded\n", TITLE);

    run_tests();

    return 0;
}

/*****************************************************************************
   Driver test exit function
******************************************************************************/
static void __exit pc302_dma_test_module_exit(void)
{
    printk(KERN_INFO "%s " CONFIG_LOCALVERSION " " __DATE__ " " __TIME__
            " Unloaded\n", TITLE);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("picoChip PC302 DMA Test Driver");
MODULE_AUTHOR("Andrew Watkins");

module_init(pc302_dma_test_module_init);
module_exit(pc302_dma_test_module_exit);

