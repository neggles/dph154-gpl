/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 *  linux/arch/arm/mach-firecracker/dma_test.c
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * Description:
 *
 * This module tests the firecracker DMA driver API.
 *
 * References:
 *
 * dma_api.txt contains details on the programming interface functions.
 * Synopsys DesignWare DW_ahb_dmac Databook Version 2.07a December 14, 2005.
 * 
 * See linux/arch/arm/mach-firecracker/dma.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
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

#include <mach/io.h>
#include <mach/hardware.h>
#include <mach/dma.h>

/* Simply do the DMA tests in the module init function and return the test 
 * status when the module exits.
 * The insmod call will return the test result.
 */

#define PASS 0
#define FAIL -1

#define FAIL_TEST(_why)                                                     \
{                                                                           \
    test_state |= FAIL;                                                     \
    printk("<%u> TEST FAILURE: "_why"\n", testnumi);                        \
}

#define RESET_TEST_RES()    test_state = PASS;
#define TEST_FAILED         (test_state == FAIL)

static volatile int test_state;

static struct device *device;

int testnumi;

/* Helper functions */
static firecracker_dma_t get_dmac(int num)
{
    firecracker_dma_t dma;
    dma = firecracker_dma_get_dmac_handle(num);
    if (!dma) FAIL_TEST("firecracker_dma_get_dmac_handle");

    return dma;
}

static void rand_fill(void *vaddr, unsigned int size)
{
    unsigned int i;
    unsigned int *p;

    for (i = 0; i < size; i += 4) {
        p = (unsigned int *)(((unsigned char *)vaddr) + i);

        *p = *(&i + i);     /* any random data on stack */
    }
}

static unsigned int checksum(void *vaddr, unsigned int size)
{
    unsigned int i;
    unsigned int *p;
    unsigned int chk = 0;
    unsigned int bits = 0;

    for (i = 0; i < size; i += 4) {
        p = (unsigned int *)(((unsigned char *)vaddr) + i);

        chk = chk + *p;     /* Checksum */
        bits = bits | *p;
    }

    /* if all data is zero, return nonzero */
    if (bits == 0) {
        return -1;
    }

    return chk;
}

static dma_addr_t get_dma_buffer(void **ret_vaddr, unsigned int size, int do_checksum)
{
    dma_addr_t paddr;
    void *vaddr;
    unsigned int *p;

    vaddr = dma_alloc_coherent(device, size, &paddr, GFP_ATOMIC);

    if (!vaddr) {
        FAIL_TEST("dma_alloc_coherent");
        return 0;
    }

    if (do_checksum) {
        rand_fill(vaddr, size);
        /* Put the -ve checksum at the end */
        p = (unsigned int *)(((unsigned char *)vaddr) + size-4);
        *p = -checksum(vaddr, size - 4);
    }
    else {
        memset(vaddr, 0xbe, size);
    }

    if (ret_vaddr) {
        *ret_vaddr = vaddr;
    }

    return paddr;
}

static void wait_a_bit(void)
{
    set_current_state( TASK_INTERRUPTIBLE );
    schedule_timeout( 50 );
}

/* Test functions... */
static void test_direct_xfr(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_1_TRW;
    src.auto_reload = 0;

    dst = src;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, 1024*4, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

 //   firecracker_dma_dump_regs(dma);

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    udelay(100);

    if (checksum(dest_vaddr, 1024*4) != 0) {
        FAIL_TEST("Checksum destination");
    }
    
    firecracker_dma_release(xfr);
}


/* Direct transfer using hardware handshaking. This does not actually transfer
 * any data as the hardware to drive the handshaking is not setup.
 * This test is only for checking the register settings.
 */
static void test_direct_xfr_hw_handshaking(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    firecracker_dma_handshake_t src_hs;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src hardware handshaking parameters */
    src_hs.interface = 1;
    src_hs.active_low = 0;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER4;
    src.periph_not_mem = 1;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = NO_CHANGE;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_4_TRW;
    src.auto_reload = 0;

    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);
    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;
    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_4_TRW;
    dst.auto_reload = 0;
    
    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, &src_hs, NULL, 1024*2, PROTCTL_1, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    firecracker_dma_dump_regs(dma);

    udelay(100);

    /* This test does not actually transfer any data as it relies on hardware
     * handshaking.
     */
#if 0
    if (checksum(dest_vaddr, 1024*4) != 0) {
        FAIL_TEST("Checksum destination");
    }
#endif
    
    firecracker_dma_release(xfr);
}


static void test_xfr_block_sizes(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    void *src_vaddr;
    unsigned int bsize;

    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst = src;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);

    /* Test block sizes 1 to 512 */
    for (bsize = 4; bsize < 512; bsize += 4) {

        memset(dest_vaddr, 0xbe, 1024*4);

        xfr = firecracker_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL);

        if (!xfr) {
            FAIL_TEST("firecracker_dma_setup_direct_xfr");
            break;
        }

        if (firecracker_dma_start(xfr) != 0) {
            FAIL_TEST("firecracker_dma_start");
            break;
        }

        udelay(100);

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0) {
            FAIL_TEST("Check destination");
            break;
        }

        firecracker_dma_release(xfr);
    }

    if (bsize < 512) {
        printk("Failed at block size %u\n", bsize);
    }
}

static void test_list_list_xfr(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    firecracker_dma_list_t list;
    void *dest_vaddr[10];
    int i;
    int elms = 10;
    unsigned int buf_sz = 1024 * 4;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    list = firecracker_dma_list_create(dma, elms);
    if (!list) {
        FAIL_TEST("firecracker_dma_list_create");
        return;
    }

    /* Setup 10 elements of the list */
    for (i = 0; i < elms; i++) {

        /* Setup src/dst parameters */
        src.dma_addr = get_dma_buffer(NULL, buf_sz, 1);
        src.ahb_master_select = AHB_MASTER1;
        src.periph_not_mem = 0;
        src.flow_controller = 0;
        src.enable_sg = 0;
        src.addr_inc = INCREMENT;
        src.tr_width = TR_WIDTH32;
        src.msize = MS_1_TRW;
        src.auto_reload = 0;

        dst = src;
        dst.dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (firecracker_dma_list_add(list, &src, &dst, buf_sz) != 0) {
            FAIL_TEST("firecracker_dma_list_add");
            return;
        }
    }

    xfr = firecracker_dma_setup_list_xfr(
            list, NULL, NULL, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_list_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    wait_a_bit();

#if 0   /* Now the list will stop automatically when it gets to the end */
    /* List transfer will continue until it is aborted */
    if (firecracker_dma_abort(xfr) != 0) {
        FAIL_TEST("firecracker_dma_abort");
        return;
    }
#endif

    for (i = 0; i < elms; i++) {
        if (checksum(dest_vaddr[i], buf_sz) != 0) {
            FAIL_TEST("Checksum destination");
        }
    }

    /* Destroy the transfer */
    firecracker_dma_release(xfr);

    /* Destroy the list */
    if (firecracker_dma_list_destroy(list) != 0) {
        FAIL_TEST("firecracker_dma_list_destroy");
        return;
    }
}


static void test_single_element_list_list_xfr(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    firecracker_dma_list_t list;
    void *dest_vaddr[10];
    int i;
    int elms = 1;
    unsigned int buf_sz = 1024 * 4;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    list = firecracker_dma_list_create(dma, elms);
    if (!list) {
        FAIL_TEST("firecracker_dma_list_create");
        return;
    }

    /* Setup 1 element of the list */
    for (i = 0; i < elms; i++) {

        /* Setup src/dst parameters */
        src.dma_addr = get_dma_buffer(NULL, buf_sz, 1);
        src.ahb_master_select = AHB_MASTER1;
        src.periph_not_mem = 0;
        src.flow_controller = 0;
        src.enable_sg = 0;
        src.addr_inc = INCREMENT;
        src.tr_width = TR_WIDTH32;
        src.msize = MS_1_TRW;
        src.auto_reload = 0;

        dst = src;
        dst.dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (firecracker_dma_list_add(list, &src, &dst, buf_sz) != 0) {
            FAIL_TEST("firecracker_dma_list_add");
            return;
        }
    }

    xfr = firecracker_dma_setup_list_xfr(
            list, NULL, NULL, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_list_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    wait_a_bit();

#if 0   /* Now the list will stop automatically when it gets to the end */
    /* List transfer will continue until it is aborted */
    if (firecracker_dma_abort(xfr) != 0) {
        FAIL_TEST("firecracker_dma_abort");
        return;
    }
#endif

    for (i = 0; i < elms; i++) {
        if (checksum(dest_vaddr[i], buf_sz) != 0) {
            FAIL_TEST("Checksum destination");
        }
    }

    /* Destroy the transfer */
    firecracker_dma_release(xfr);

    /* Destroy the list */
    if (firecracker_dma_list_destroy(list) != 0) {
        FAIL_TEST("firecracker_dma_list_destroy");
        return;
    }
}


static void test_list_list_xfr_reuse_list(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    firecracker_dma_list_t list;
    void *dest_vaddr[10];
    int i;
    int elms = 10;
    unsigned int buf_sz = 1024 * 4;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    list = firecracker_dma_list_create(dma, elms);
    if (!list) {
        FAIL_TEST("firecracker_dma_list_create");
        return;
    }

    xfr = firecracker_dma_setup_list_xfr(
            list, NULL, NULL, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_list_xfr");
        return;
    }

    /* Setup 10 elements of the list */
    for (i = 0; i < elms; i++) {

        /* Setup src/dst parameters */
        src.dma_addr = get_dma_buffer(NULL, buf_sz, 1);
        src.ahb_master_select = AHB_MASTER1;
        src.periph_not_mem = 0;
        src.flow_controller = 0;
        src.enable_sg = 0;
        src.addr_inc = INCREMENT;
        src.tr_width = TR_WIDTH32;
        src.msize = MS_1_TRW;
        src.auto_reload = 0;

        dst = src;
        dst.dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (firecracker_dma_list_add(list, &src, &dst, buf_sz) != 0) {
            FAIL_TEST("firecracker_dma_list_add");
            return;
        }
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    wait_a_bit();

#if 0   /* List xfr will stop automatically when it gets to the end */
    /* List transfer will continue until it is aborted */
    if (firecracker_dma_abort(xfr) != 0) {
        FAIL_TEST("firecracker_dma_abort");
        return;
    }
#endif

    for (i = 0; i < elms; i++) {
        if (checksum(dest_vaddr[i], buf_sz) != 0) {
            FAIL_TEST("Checksum destination");
        }
    }

    /* Reset the list and reuse it */
    if (firecracker_dma_list_clear(list) != 0) {
        FAIL_TEST("firecracker_dma_list_clear");
        return;
    }

    /* Setup 5 elements of the list */
    elms = 5;
    for (i = 0; i < elms; i++) {

        /* Setup src/dst parameters */
        src.dma_addr = get_dma_buffer(NULL, buf_sz, 1);
        src.ahb_master_select = AHB_MASTER1;
        src.periph_not_mem = 0;
        src.flow_controller = 0;
        src.enable_sg = 0;
        src.addr_inc = INCREMENT;
        src.tr_width = TR_WIDTH32;
        src.msize = MS_1_TRW;
        src.auto_reload = 0;

        dst = src;
        dst.dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (firecracker_dma_list_add(list, &src, &dst, buf_sz) != 0) {
            FAIL_TEST("firecracker_dma_list_add");
            return;
        }
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    wait_a_bit();

#if 0   /* List xfr will stop automatically when it gets to the end */
    /* List transfer will continue until it is aborted */
    if (firecracker_dma_abort(xfr) != 0) {
        FAIL_TEST("firecracker_dma_abort");
        return;
    }
#endif

    for (i = 0; i < elms; i++) {
        if (checksum(dest_vaddr[i], buf_sz) != 0) {
            FAIL_TEST("Checksum destination");
        }
    }

    /* Destroy the transfer */
    firecracker_dma_release(xfr);

    /* Destroy the list */
    if (firecracker_dma_list_destroy(list) != 0) {
        FAIL_TEST("firecracker_dma_list_destroy");
        return;
    }
}



static void test_reload_to_list_xfr(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    firecracker_dma_list_t list;
    void *dest_vaddr[10];
    int i;
    int elms = 10;
    unsigned int buf_sz = 1024 * 4;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    list = firecracker_dma_list_create(dma, elms);
    if (!list) {
        FAIL_TEST("firecracker_dma_list_create");
        return;
    }

    /* Setup src/dst parameters */
    src.ahb_master_select = AHB_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_1_TRW;

    /* Reload the source block so that the source block get copied to all
     * list elements of the destination
     */
    src.auto_reload = 1;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;
    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_1_TRW;
    dst.auto_reload = 0;

    src.dma_addr = get_dma_buffer(NULL, buf_sz, 1);
    dst.dma_addr = get_dma_buffer(&dest_vaddr[0], buf_sz, 0);

    /* Setup first element of the list */
    if (firecracker_dma_list_add(list, &src, &dst, buf_sz) != 0) {
        FAIL_TEST("firecracker_dma_list_add");
        return;
    }

    /* Setup elements 2 to 10 of the list */
    for (i = 1; i < elms; i++) {

        dst.dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        /* No source endpoint for subsequent elements */
        if (firecracker_dma_list_add(list, NULL, &dst, buf_sz) != 0) {
            FAIL_TEST("firecracker_dma_list_add");
            return;
        }
    }

    xfr = firecracker_dma_setup_list_xfr(
            list, NULL, NULL, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_list_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    wait_a_bit();

#if 0   /* Transfer will stop when it reaches the end of the list */
    /* Auto-reload transfer will continue until it is aborted */
    if (firecracker_dma_abort(xfr) != 0) {
        FAIL_TEST("firecracker_dma_abort");
        return;
    }
#endif

    for (i = 0; i < elms; i++) {
        if (checksum(dest_vaddr[i], buf_sz) != 0) {
            FAIL_TEST("Checksum destination");
        }
    }

    /* Destroy the transfer */
    firecracker_dma_release(xfr);

    /* Destroy the list */
    if (firecracker_dma_list_destroy(list) != 0) {
        FAIL_TEST("firecracker_dma_list_destroy");
        return;
    }
}


static int test_direct_xfr_with_interrupts_irq_state = 0;
static unsigned int test_cookie = 0xbebebebe;

static irqreturn_t test_direct_xfr_with_interrupts_irq(int irq, void *dev_id, struct pt_regs *regs)
{
    firecracker_dma_t dma = (firecracker_dma_t) dev_id; 
    firecracker_dma_xfr_t xfr;
    dma_int_type_t int_type;
    unsigned int *cookie_addr;

    /* Get the handle of the transfer that caused the interrupt */
    int_type = INT_ALL;
    xfr = firecracker_dma_int_get_xfr(dma, &int_type, (void **)&cookie_addr);
    if (xfr == NULL) {
        FAIL_TEST("firecracker_dma_int_get_xfr");
        return IRQ_HANDLED;
    }

    if (cookie_addr != &test_cookie) {
        FAIL_TEST("Incorrect cookie not passed to interrupt");
    }

    test_direct_xfr_with_interrupts_irq_state++;

    /* Reset the interrupt we are servicing */
    firecracker_dma_clear_int(xfr, int_type);

    /* Handle any other interrupts that may have occurred at the same 
     * time
     */
    while (1) {

        int_type = INT_ALL;
        xfr = firecracker_dma_int_get_xfr(dma, &int_type, NULL);
        if (xfr == NULL) {
            break;
        }

        test_direct_xfr_with_interrupts_irq_state++;

        firecracker_dma_clear_int(xfr, int_type);
    }

    return IRQ_HANDLED;
}


static void test_direct_xfr_with_interrupts(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Get the interrupt */
    if (request_irq(
            IRQ_DMA_1, &test_direct_xfr_with_interrupts_irq,
            SA_INTERRUPT, "test_direct_xfr_with_interrupts", dma) != 0) {
        FAIL_TEST("request_irq");
        return;
    }

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_1_TRW;
    src.auto_reload = 0;

    dst = src;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, 1024*4, 0, &test_cookie);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    firecracker_dma_enable_int(xfr, INT_BLOCK | INT_ERROR | INT_TRANSFER);

    test_direct_xfr_with_interrupts_irq_state = 0;

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    udelay(100);

    if (checksum(dest_vaddr, 1024*4) != 0) {
        FAIL_TEST("Checksum destination");
    }

    if (test_direct_xfr_with_interrupts_irq_state == 0) {
        FAIL_TEST("No interrupt occurred");
    }

    printk("%u interrupts received\n", test_direct_xfr_with_interrupts_irq_state);

    free_irq(IRQ_DMA_1, dma);
    firecracker_dma_release(xfr);
}


static int test_list_xfr_with_interrupts_irq_state = 0;

static irqreturn_t test_list_xfr_with_interrupts_irq(int irq, void *dev_id, struct pt_regs *regs)
{
    firecracker_dma_t dma = (firecracker_dma_t) dev_id; 
    firecracker_dma_xfr_t xfr;
    dma_int_type_t int_type = INT_ALL;

    while (1) {

        /* Get the handle of the transfer that caused the interrupt */
        int_type = INT_ALL;
        xfr = firecracker_dma_int_get_xfr(dma, &int_type, NULL);
        if (xfr == NULL) {
            break;
        }

        test_list_xfr_with_interrupts_irq_state++;

        /* Reset the interrupt we are servicing */
        firecracker_dma_clear_int(xfr, int_type);
    }

    return IRQ_HANDLED;
}


static void test_list_xfr_with_interrupts(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    firecracker_dma_list_t list;
    void *dest_vaddr[10];
    int i;
    int elms = 10;
    unsigned int buf_sz = 1024 * 4;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Get the interrupt */
    if (request_irq(
            IRQ_DMA_1, &test_list_xfr_with_interrupts_irq,
            SA_INTERRUPT, "test_list_xfr_with_interrupts", dma) != 0) {
        FAIL_TEST("request_irq");
        return;
    }

    list = firecracker_dma_list_create(dma, elms);
    if (!list) {
        FAIL_TEST("firecracker_dma_list_create");
        return;
    }

    /* Setup src/dst parameters */
    src.ahb_master_select = AHB_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_1_TRW;
    src.auto_reload = 0;

    dst = src;

    /* Setup 10 elements of the list */
    for (i = 0; i < elms; i++) {

        src.dma_addr = get_dma_buffer(NULL, buf_sz, 1);
        dst.dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (firecracker_dma_list_add(list, &src, &dst, buf_sz) != 0) {
            FAIL_TEST("firecracker_dma_list_add");
            return;
        }
    }

    xfr = firecracker_dma_setup_list_xfr(
            list, NULL, NULL, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_list_xfr");
        return;
    }

    firecracker_dma_enable_int(xfr, INT_BLOCK | INT_ERROR | INT_TRANSFER);

    test_list_xfr_with_interrupts_irq_state = 0;

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    wait_a_bit();

#if 0       /* List will stop at the end */
    if (firecracker_dma_abort(xfr) != 0) {
        FAIL_TEST("firecracker_dma_abort");
        return;
    }
#endif

    for (i = 0; i < elms; i++) {
        if (checksum(dest_vaddr[i], buf_sz) != 0) {
            FAIL_TEST("Checksum destination");
        }
    }

    if (test_list_xfr_with_interrupts_irq_state == 0) {
        FAIL_TEST("No interrupt occurred");
    }

    /* Destroy the transfer */
    firecracker_dma_release(xfr);

    /* Destroy the list */
    if (firecracker_dma_list_destroy(list) != 0) {
        FAIL_TEST("firecracker_dma_list_destroy");
        return;
    }

    free_irq(IRQ_DMA_1, dma);

    printk("%u interrupts received\n", test_list_xfr_with_interrupts_irq_state);
}


static int test_list_xfr_with_interrupts2_irq_state = 0;
static int tlxwi2_trf_ints = 0;
static int tlxwi2_block_ints = 0;
static int tlxwi2_srctrx_ints = 0;
static int tlxwi2_dsttrx_ints = 0;
static int tlxwi2_err_ints = 0;
static unsigned int blocks_left;

static irqreturn_t test_list_xfr_with_interrupts2_irq(int irq, void *dev_id, struct pt_regs *regs)
{
    firecracker_dma_t dma = (firecracker_dma_t) dev_id; 
    firecracker_dma_xfr_t xfr;
    dma_int_type_t int_type;

    /* Repeat the following, for each interrupt type */
    while (1) {

        /* Get the handle of the transfer that caused the interrupt */
        int_type = INT_ALL;
        xfr = firecracker_dma_int_get_xfr(dma, &int_type, NULL);
        if (xfr == NULL) {
            break;
        }

        test_list_xfr_with_interrupts2_irq_state++;

        switch (int_type) {
            case INT_BLOCK:
                tlxwi2_block_ints++;

#if 0   /* No-longer needs to handle block interrupts for list->list multi
           block transfers. Only need to do this when auto-reload or continuous
           type transfers are involved. */
            
                /* The DMA driver needs to handle the block interrupt for 
                 * proper termination of the list transfer 
                 */
                firecracker_dma_handle_block_int(xfr, &blocks_left);
#else
                /* track the blocks remaining count to keep the test happy as
                 * we do not do the firecracker_dma_handle_block_int any more
                 */
                blocks_left--;
#endif
                break;

            case INT_DST_TRANSACTION:
                tlxwi2_dsttrx_ints++;
                break;

            case INT_ERROR:
                tlxwi2_err_ints++;
                break;

            case INT_SRC_TRANSACTION:
                tlxwi2_srctrx_ints++;
                break;

            case INT_TRANSFER:
                tlxwi2_trf_ints++;
                break;

            default:
                FAIL_TEST("BAD INT TYPE");
        }

        /* Reset the interrupt we are servicing */
        firecracker_dma_clear_int(xfr, int_type);
    }

    return IRQ_HANDLED;
}


static void test_list_xfr_with_interrupts2(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    firecracker_dma_list_t list;
    void *dest_vaddr[10];
    int i;
    int elms = 3;
    unsigned int buf_sz = 1024 * 4;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Get the interrupt */
    if (request_irq(
            IRQ_DMA_1, &test_list_xfr_with_interrupts2_irq,
            SA_INTERRUPT, "test_list_xfr_with_interrupts2", dma) != 0) {
        FAIL_TEST("request_irq");
        return;
    }

    list = firecracker_dma_list_create(dma, elms);
    if (!list) {
        FAIL_TEST("firecracker_dma_list_create");
        return;
    }

    /* Setup src/dst parameters */
    src.ahb_master_select = AHB_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_1_TRW;
    src.auto_reload = 0;

    dst = src;

    /* Setup 10 elements of the list */
    for (i = 0; i < elms; i++) {

        src.dma_addr = get_dma_buffer(NULL, buf_sz, 1);
        dst.dma_addr = get_dma_buffer(&dest_vaddr[i], buf_sz, 0);

        if (firecracker_dma_list_add(list, &src, &dst, buf_sz) != 0) {
            FAIL_TEST("firecracker_dma_list_add");
            return;
        }
    }

    xfr = firecracker_dma_setup_list_xfr(
            list, NULL, NULL, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_list_xfr");
        return;
    }

    firecracker_dma_enable_int(xfr, INT_BLOCK | INT_ERROR | INT_TRANSFER);

    test_list_xfr_with_interrupts2_irq_state = 0;

    /* Set the blocks counter so that the driver can stop the transfer
     * when we get to the end of the block list.
     */
    blocks_left = elms;

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    wait_a_bit();

    /* The transfer should have stopped by itself. Check that the
     * remaining blocks counter reached zero.
     */
    if (blocks_left != 0) {
        FAIL_TEST("Blocks left not zero!");
    }

    for (i = 0; i < elms; i++) {
        if (checksum(dest_vaddr[i], buf_sz) != 0) {
            FAIL_TEST("Checksum destination");
        }
    }

    if (test_list_xfr_with_interrupts2_irq_state == 0) {
        FAIL_TEST("No interrupt occurred");
    }

    if (tlxwi2_srctrx_ints != 0 || tlxwi2_dsttrx_ints != 0) {
        FAIL_TEST("Unexpected transaction interrupts");
    }

    if (tlxwi2_trf_ints != 1) {
        FAIL_TEST("Expected a single transfer complete interrupt");
    }

    /* Destroy the transfer */
    firecracker_dma_release(xfr);

    /* Destroy the list */
    if (firecracker_dma_list_destroy(list) != 0) {
        FAIL_TEST("firecracker_dma_list_destroy");
        return;
    }

    free_irq(IRQ_DMA_1, dma);

    printk("%u total interrupts received\n", test_list_xfr_with_interrupts2_irq_state);
    printk("%u TRF interrupts received\n", tlxwi2_trf_ints);
    printk("%u BLOCK interrupts received\n", tlxwi2_block_ints);
    printk("%u SRC Trans interrupts received\n", tlxwi2_srctrx_ints);
    printk("%u DST Trans interrupts received\n", tlxwi2_dsttrx_ints);
    printk("%u ERR interrupts received\n", tlxwi2_err_ints);
    printk("%u blocks left\n", blocks_left);

}



static void test_src_sw_handshaking(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;
    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, 1024*4, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    /* Request transactions until the transfer stops */
    while (firecracker_dma_request_transaction(xfr, SRC, NULL) != EINVAL) ;

    if (checksum(dest_vaddr, 1024*4) != 0) {
        FAIL_TEST("Checksum destination");
    }


    firecracker_dma_release(xfr);
}



static void test_dst_sw_handshaking(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, 1024*4, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    /* Request transactions until the transfer stops */
    while (firecracker_dma_request_transaction(xfr, DST, NULL) != EINVAL) ;

    if (checksum(dest_vaddr, 1024*4) != 0) {
        FAIL_TEST("Checksum destination");
    }


    firecracker_dma_release(xfr);
}


static void test_src_dst_sw_handshaking(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, 1024*4, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    /* Request transactions until the transfer stops */
    while (1) {
        int rc_s, rc_d;
        rc_s = firecracker_dma_request_transaction(xfr, SRC, NULL);
        rc_d = firecracker_dma_request_transaction(xfr, DST, NULL);
        if (rc_s == EINVAL && rc_d == EINVAL) {
            break;
        }
    }

    if (checksum(dest_vaddr, 1024*4) != 0) {
        FAIL_TEST("Checksum destination");
    }


    firecracker_dma_release(xfr);
}


static void test_src_dst_sw_hand_src_flow_controller(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    unsigned int bytes_left = 1024;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    /* Make the source the flow controller. This means that the source endpoint
     * will control when the end of a block arrives through the bytes_left 
     * parameter of the firecracker_dma_request_transaction function.
     */
    src.flow_controller = 1;
    
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, bytes_left, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, bytes_left, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    /* Request transactions until the transfer stops */
    while (1) {
        int rc_s, rc_d;
        rc_s = firecracker_dma_request_transaction(xfr, SRC, &bytes_left);
        rc_d = firecracker_dma_request_transaction(xfr, DST, NULL);
        if (rc_s == EINVAL && rc_d == EINVAL) {
            break;
        }
    }

    if (checksum(dest_vaddr, 1024) != 0) {
        FAIL_TEST("Checksum destination");
    }


    firecracker_dma_release(xfr);
}


static void test_src_dst_sw_hand_dst_flow_controller(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    unsigned int bytes_left;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    /* Make the source the flow controller. This means that the source endpoint
     * will control when the end of a block arrives through the bytes_left 
     * parameter of the firecracker_dma_request_transaction function.
     */
    dst.flow_controller = 1;

    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, bytes_left, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    /* Request transactions until the transfer stops */
    bytes_left = 1024;
    while (1) {
        int rc_s, rc_d;
        rc_s = firecracker_dma_request_transaction(xfr, SRC, NULL);
        rc_d = firecracker_dma_request_transaction(xfr, DST, &bytes_left);
        if (rc_s == EINVAL && rc_d == EINVAL) {
            break;
        }
    }

    if (checksum(dest_vaddr, 1024) != 0) {
        FAIL_TEST("Checksum destination");
    }


    firecracker_dma_release(xfr);
}



static void test_dst_sw_hand_dst_flow_controller(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    unsigned int bytes_left;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024, 1);
    src.ahb_master_select = AHB_MASTER1;
    src.periph_not_mem = 0;
    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    /* Make the source the flow controller. This means that the source endpoint
     * will control when the end of a block arrives through the bytes_left 
     * parameter of the firecracker_dma_request_transaction function.
     */
    dst.flow_controller = 1;

    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, bytes_left, 0, NULL);

    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    /* Request transactions until the transfer stops */
    bytes_left = 1024;
    while (1) {
        int rc_d;
        rc_d = firecracker_dma_request_transaction(xfr, DST, &bytes_left);
        if (rc_d == EINVAL) {
            break;
        }
    }

    if (checksum(dest_vaddr, 1024) != 0) {
        FAIL_TEST("Checksum destination");
    }


    firecracker_dma_release(xfr);
}


static void test_src_dst_sw_handshaking_block_sizes(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    void *src_vaddr;
    unsigned int bsize;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);

    /* Test block sizes 1 to 512 */
    for (bsize = 4; bsize < 512; bsize += 4) {

        memset(dest_vaddr, 0xbe, 1024*4);

        xfr = firecracker_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL);

        if (!xfr) {
            FAIL_TEST("firecracker_dma_setup_direct_xfr");
            break;
        }

        if (firecracker_dma_start(xfr) != 0) {
            FAIL_TEST("firecracker_dma_start");
            break;
        }

        /* Request transactions until the transfer stops */
        while (1) {
            int rc_s, rc_d;
            rc_s = firecracker_dma_request_transaction(xfr, SRC, NULL);
            rc_d = firecracker_dma_request_transaction(xfr, DST, NULL);
            if (rc_s == EINVAL && rc_d == EINVAL) {
                break;
            }
        }

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0) {
            FAIL_TEST("Check destination");
            break;
        }

        firecracker_dma_release(xfr);
    }

    if (bsize < 512) {
        printk("Failed at block size %u\n", bsize);
    }
}




static void test_src_dst_sw_handshaking_src_fc_block_sizes(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    void *src_vaddr;
    unsigned int bsize;
    unsigned int bytes_left;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, 1024, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    /* Make the source the flow controller. This means that the source endpoint
     * will control when the end of a block arrives through the bytes_left 
     * parameter of the firecracker_dma_request_transaction function.
     */
    src.flow_controller = 1;
    
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024, 0);

    /* Test block sizes 1 to 512 */
    for (bsize = 4; bsize < 512; bsize += 4) {

        memset(dest_vaddr, 0xbe, 1024*4);

        xfr = firecracker_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL);

        if (!xfr) {
            FAIL_TEST("firecracker_dma_setup_direct_xfr");
            break;
        }

        if (firecracker_dma_start(xfr) != 0) {
            FAIL_TEST("firecracker_dma_start");
            break;
        }

        /* Request transactions until the transfer stops */
        bytes_left = bsize;
        while (1) {
            int rc_s, rc_d;
            rc_s = firecracker_dma_request_transaction(xfr, SRC, &bytes_left);
            rc_d = firecracker_dma_request_transaction(xfr, DST, NULL);
            if (rc_s == EINVAL && rc_d == EINVAL) {
                break;
            }
        }

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0) {
            FAIL_TEST("Check destination");
            break;
        }

        firecracker_dma_release(xfr);
    }

    if (bsize < 512) {
        printk("Failed at block size %u\n", bsize);
    }
}




static void test_src_sw_handshaking_src_fc_block_sizes(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    void *src_vaddr;
    unsigned int bsize;
    unsigned int bytes_left;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, 1024, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    /* Make the source the flow controller. This means that the source endpoint
     * will control when the end of a block arrives through the bytes_left 
     * parameter of the firecracker_dma_request_transaction function.
     */
    src.flow_controller = 1;
    
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;
    dst.periph_not_mem = 0;
    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024, 0);

    /* Test block sizes 1 to 512 */
    for (bsize = 4; bsize < 512; bsize += 4) {

        memset(dest_vaddr, 0xbe, 1024*4);

        xfr = firecracker_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL);

        if (!xfr) {
            FAIL_TEST("firecracker_dma_setup_direct_xfr");
            break;
        }

        if (firecracker_dma_start(xfr) != 0) {
            FAIL_TEST("firecracker_dma_start");
            break;
        }

        /* Request transactions until the transfer stops */
        bytes_left = bsize;
        while (1) {
            int rc_s;
            rc_s = firecracker_dma_request_transaction(xfr, SRC, &bytes_left);
            if (rc_s == EINVAL) {
                break;
            }
        }

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0) {
            FAIL_TEST("Check destination");
            break;
        }

        firecracker_dma_release(xfr);
    }

    if (bsize < 512) {
        printk("Failed at block size %u\n", bsize);
    }
}




static void test_src_dst_sw_handshaking_dst_fc_block_sizes(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
    void *src_vaddr;
    unsigned int bsize;
    unsigned int bytes_left;
        
    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(&src_vaddr, 1024, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    /* Make the source the flow controller. This means that the source endpoint
     * will control when the end of a block arrives through the bytes_left 
     * parameter of the firecracker_dma_request_transaction function.
     */
    dst.flow_controller = 1;
    
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024, 0);

    /* Test block sizes 1 to 512 */
    for (bsize = 4; bsize < 512; bsize += 4) {

        memset(dest_vaddr, 0xbe, 1024*4);

        xfr = firecracker_dma_setup_direct_xfr(
                dma, &src, &dst, NULL, NULL, bsize, 0, NULL);

        if (!xfr) {
            FAIL_TEST("firecracker_dma_setup_direct_xfr");
            break;
        }

        if (firecracker_dma_start(xfr) != 0) {
            FAIL_TEST("firecracker_dma_start");
            break;
        }

        /* Request transactions until the transfer stops */
        bytes_left = bsize;
        while (1) {
            int rc_s, rc_d;
            rc_s = firecracker_dma_request_transaction(xfr, SRC, NULL);
            rc_d = firecracker_dma_request_transaction(xfr, DST, &bytes_left);
            if (rc_s == EINVAL && rc_d == EINVAL) {
                break;
            }
        }

        if (memcmp(src_vaddr, dest_vaddr, bsize) != 0) {
            FAIL_TEST("Check destination");
            break;
        }

        firecracker_dma_release(xfr);
    }

    if (bsize < 512) {
        printk("Failed at block size %u\n", bsize);
    }
}



static int test_src_dst_sw_handshaking_int_state = 0;

static irqreturn_t test_src_dst_sw_handshaking_int_irq(int irq, void *dev_id, struct pt_regs *regs)
{
    firecracker_dma_t dma = (firecracker_dma_t) dev_id; 
    firecracker_dma_xfr_t xfr;
    dma_int_type_t int_type = INT_ALL;

    /* Repeat the following, for each interrupt type */
    while (1) {

        /* Get the handle of the transfer that caused the interrupt */
        int_type = INT_ALL;
        xfr = firecracker_dma_int_get_xfr(dma, &int_type, NULL);
        if (xfr == NULL) {
            /* No more interrupts to service, exit */
            break;
        }

        test_src_dst_sw_handshaking_int_state++;

        switch (int_type) {
            case INT_BLOCK:
                tlxwi2_block_ints++;
                break;

            case INT_DST_TRANSACTION:
                tlxwi2_dsttrx_ints++;
                /* Service the destination transaction complete interrupt
                 * by requesting a new transaction
                 */
                if (firecracker_dma_request_transaction(xfr, DST, NULL) == EBUSY) {
                    FAIL_TEST("firecracker_dma_request_transaction Busy");
                }
                
                break;

            case INT_ERROR:
                tlxwi2_err_ints++;
                break;

            case INT_SRC_TRANSACTION:
                tlxwi2_srctrx_ints++;
                /* Service the source transaction complete interrupt
                 * by requesting a new transaction
                 */
                if (firecracker_dma_request_transaction(xfr, SRC, NULL) == EBUSY) {
                    FAIL_TEST("firecracker_dma_request_transaction Busy");
                }
                
                break;

            case INT_TRANSFER:
                tlxwi2_trf_ints++;
                break;

            default:
                FAIL_TEST("BAD INT TYPE");
        }

        /* Reset the interrupt we are servicing */
        firecracker_dma_clear_int(xfr, int_type);
    }

    return IRQ_HANDLED;
}


static void test_src_dst_sw_handshaking_int(void)
{
    firecracker_dma_t dma;
    firecracker_dma_xfr_t xfr;
    firecracker_dma_endpoint_t src, dst;
    void *dest_vaddr;
        
    tlxwi2_trf_ints = 0;
    tlxwi2_block_ints = 0;
    tlxwi2_srctrx_ints = 0;
    tlxwi2_dsttrx_ints = 0;
    tlxwi2_err_ints = 0;

    /* Get a handle to a controller */
    dma = get_dmac(0);
    if (TEST_FAILED) return;

    /* Get the interrupt */
    if (request_irq(
            IRQ_DMA_1, &test_src_dst_sw_handshaking_int_irq,
            SA_INTERRUPT, "test_src_dst_sw_handshaking_int_irq", dma) != 0) {
        FAIL_TEST("request_irq");
        return;
    }

    /* Setup src/dst parameters */
    src.dma_addr = get_dma_buffer(NULL, 1024*4, 1);
    src.ahb_master_select = AHB_MASTER1;

    /* Pretend the source is a peripheral. This means that we will have to do
     * transaction requests on the source to progress the transfer
     */
    src.periph_not_mem = 1;

    src.flow_controller = 0;
    src.enable_sg = 0;
    src.addr_inc = INCREMENT;
    src.tr_width = TR_WIDTH32;
    src.msize = MS_AUTO;
    src.auto_reload = 0;

    dst.ahb_master_select = AHB_MASTER1;
    dst.periph_not_mem = 0;

    /* Pretend the destination is a peripheral. This means that we will have
     * to do transaction requests on the destination to progress the transfer
     */
    dst.periph_not_mem = 1;

    dst.flow_controller = 0;
    dst.enable_sg = 0;
    dst.addr_inc = INCREMENT;
    dst.tr_width = TR_WIDTH32;
    dst.msize = MS_AUTO;
    dst.auto_reload = 0;
    dst.dma_addr = get_dma_buffer(&dest_vaddr, 1024*4, 0);

    xfr = firecracker_dma_setup_direct_xfr(
            dma, &src, &dst, NULL, NULL, 1024*4, 0, NULL);
    if (!xfr) {
        FAIL_TEST("firecracker_dma_setup_direct_xfr");
        return;
    }

    firecracker_dma_enable_int(xfr, 
            INT_DST_TRANSACTION | INT_SRC_TRANSACTION | INT_ERROR | INT_TRANSFER);

    if (firecracker_dma_start(xfr) != 0) {
        FAIL_TEST("firecracker_dma_start");
        return;
    }

    /* Start the process by requesting the first transactions, When these
     * are complete, the interrupt will trigger subsequent ones
     */
    firecracker_dma_request_transaction(xfr, SRC, NULL);
    firecracker_dma_request_transaction(xfr, DST, NULL);

    wait_a_bit();

    if (checksum(dest_vaddr, 1024*4) != 0) {
        FAIL_TEST("Checksum destination");
    }

    if (tlxwi2_trf_ints != 1) {
        FAIL_TEST("Expected a single transfer complete interrupt");
    }

    free_irq(IRQ_DMA_1, dma);
    firecracker_dma_release(xfr);

    printk("%u total interrupts received\n", test_src_dst_sw_handshaking_int_state);
    printk("%u TRF interrupts received\n", tlxwi2_trf_ints);
    printk("%u BLOCK interrupts received\n", tlxwi2_block_ints);
    printk("%u SRC Trans interrupts received\n", tlxwi2_srctrx_ints);
    printk("%u DST Trans interrupts received\n", tlxwi2_dsttrx_ints);
    printk("%u ERR interrupts received\n", tlxwi2_err_ints);


}


typedef void (*tf)(void);

/* Array of tests */
static struct test_tag {
    char name[200];
    tf test_fn;
} tests[] = {
    {"Simple direct transfer test", test_direct_xfr},
    {"List to list transfer test", test_list_list_xfr},
    {"Simple direct transfer test with interrupts", test_direct_xfr_with_interrupts},
    {"List to list transfer test with interrupts", test_list_xfr_with_interrupts},
    {"List to list transfer test with interrupts and block control", test_list_xfr_with_interrupts2},
    {"Simple direct transfers of various block sizes", test_xfr_block_sizes},
    {"Simple direct transfer using source software handshaking", test_src_sw_handshaking},
    {"Simple direct transfer using destination software handshaking", test_dst_sw_handshaking},
    {"Simple direct transfer using source and destination software handshaking", test_src_dst_sw_handshaking},
    {"Simple direct transfer using source and destination software handshaking controlled by interrupts", test_src_dst_sw_handshaking_int},
    {"Simple direct transfer, src/dst sw handshaking, various block sizes", test_src_dst_sw_handshaking_block_sizes},
    {"Simple direct transfer, src/dst sw handshaking, source flow controller", test_src_dst_sw_hand_src_flow_controller},
    {"Simple direct transfer, src/dst sw handshaking, dest flow controller", test_src_dst_sw_hand_dst_flow_controller},
    {"Simple direct transfer, src/dst sw handshaking, source FC & various block sizes", test_src_dst_sw_handshaking_src_fc_block_sizes},
    {"Simple direct transfer, src/dst sw handshaking, desc FC & various block sizes", test_src_dst_sw_handshaking_dst_fc_block_sizes},
    {"Simple direct transfer, dst sw handshaking, dest flow controller", test_dst_sw_hand_dst_flow_controller},
    {"Simple direct transfer, src sw handshaking, source FC & various block sizes", test_src_sw_handshaking_src_fc_block_sizes},
    {"Reload Block to list transfer test", test_reload_to_list_xfr},
    {"List to list transfer test, reuse list", test_list_list_xfr_reuse_list},
    {"Simple direct transfer test using hardware handshaking", test_direct_xfr_hw_handshaking},
    {"Single element list to list transfer test", test_single_element_list_list_xfr},
};

#define NUM_TESTS (sizeof(tests) / sizeof(struct test_tag))


int run_tests(void)
{
    int res = PASS;

    for (testnumi = 0; testnumi < NUM_TESTS; testnumi++) {
        printk(KERN_INFO "dmat: <%u> START %s\n", testnumi, tests[testnumi].name);

        RESET_TEST_RES();
        
        tests[testnumi].test_fn();

        printk(KERN_INFO "dmat: TEST <%u> %s\n", testnumi,
                (test_state == FAIL) ? "FAILED" : "PASSED");

        if (test_state == FAIL) {
            res = FAIL;
        }
            
    }

    if (res == PASS) {
        printk(KERN_INFO "dmat: Passed all tests.\n");
    }
    else {
        printk(KERN_INFO "dmat: FAILED SOME TESTS!!\n");
    }

    return PASS;
}




static int dmac_test_drv_probe(struct platform_device *pdev)
{
    device = &pdev->dev;

    run_tests();

    return 0;
}


static int dmac_test_drv_remove(struct platform_device *pdev)
{
    return 0;
}


#define CARDNAME "pc20x-dmac-test"
static struct platform_driver dmac_test_driver = {
    .probe      = dmac_test_drv_probe,
    .remove     = dmac_test_drv_remove,
    .driver     = {
        .name   = CARDNAME,
    }
};


static int firecracker_dma_test_module_init(void)
{
    int ret;

    ret = platform_driver_register(&dmac_test_driver);
    if (ret != 0) {
        printk("Failed to register DMAC test driver\n");
        return ret;
    }

    printk(KERN_INFO "DMAC test loaded\n");

    return ret;
}


static void firecracker_dma_test_module_exit(void)
{
    platform_driver_unregister(&dmac_test_driver);
}

module_init(firecracker_dma_test_module_init);
module_exit(firecracker_dma_test_module_exit);



