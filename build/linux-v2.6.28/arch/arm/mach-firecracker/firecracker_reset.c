/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 *  linux/arch/arm/mach-firecracker/firecracker_reset.c
 *
 * Copyright (c) 2007 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

#include <linux/semaphore.h>
#include <linux/module.h>
#include <mach/reset.h>

static void *external_reset_cookie = NULL;
static reset_t external_reset_fn = NULL;
static DECLARE_MUTEX(reg_lock);


/* firecracker_reset is called from the kernel when it wishes to reset.
 * This function will initiate a reset if an external reset handler
 * has been registered. Typically, the CPE20X FPGA driver registers a reset
 * handler as it is able to perform hardware resets of the board
 */
void firecracker_reset(char mode)
{
    if (!down_interruptible(&reg_lock)) {
        if (external_reset_fn != NULL) {
            external_reset_fn(mode, external_reset_cookie);
        }
        else {
            printk(KERN_WARNING "No reset handler installed. Entering infinite loop\n");
        }
        up(&reg_lock);
    }
}

/* register_reset_handler is used by drivers that want to advertise 
 * their ability to hard reset the kernel.
 * This is first-come-first-served. The first driver to register
 * the reset handler is the one that will actually be used when
 * a reset is requested.
 * Subsequent registrations are ignored.
 * The reset param is a finction pointer. See reset_t.
 * The cookie is passed to the reset function when called.
 */
void register_reset_handler(reset_t reset, void *cookie)
{
    if (!down_interruptible(&reg_lock)) {
        if (external_reset_fn == NULL) {
            external_reset_fn = reset;
            external_reset_cookie = cookie;
        }
        up(&reg_lock);
    }
}

/* deregister_reset_handler tells the kernel that the reset handler
 * has disappeared and it can no longer be used. The same parameters
 * as the register_reset_handler need to be passed.
 * If the handler was not the first to register and therefore was ignored,
 * it will not be de registered either.
 */
void deregister_reset_handler(reset_t reset, void *cookie)
{
    if (!down_interruptible(&reg_lock)) {
        if (external_reset_fn == reset && external_reset_cookie == cookie) {
            external_reset_fn = NULL;
            external_reset_cookie = NULL;
        }
        up(&reg_lock);
    }
}

EXPORT_SYMBOL(firecracker_reset);
EXPORT_SYMBOL(register_reset_handler);
EXPORT_SYMBOL(deregister_reset_handler);

