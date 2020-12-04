/* linux/include/asm-arm/arch-firecracker/reset.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * Firecracker - Reset handler
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_RESET_H
#define __ASM_ARCH_RESET_H __FILE__

/* Function pointer to a function able to reset the board. This function
 * is passed to the kernel by a driver.
 */
typedef void (*reset_t)(char mode, void *cookie);

/* Trigger a reset (called by kernel) */
void firecracker_reset(char mode);

/* Reset registration (called by drivers) */
void register_reset_handler(reset_t reset, void *cookie);
void deregister_reset_handler(reset_t reset, void *cookie);


#endif /* __ASM_ARCH_RESET_H */

