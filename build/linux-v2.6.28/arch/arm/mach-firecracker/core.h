/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05 
 *****************************************************************************/

/*
 *  linux/arch/arm/mach-firecracker/core.h
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

#ifndef __ASM_ARCH_FIRECRACKER_H__
#define __ASM_ARCH_FIRECRACKER_H__

#include <linux/amba/bus.h>

extern void __init firecracker_init(void);
extern void __init firecracker_init_irq(void);
extern void __init firecracker_map_io(void);
extern void __init firecracker_fixup(
        struct machine_desc *md,
        struct tag *tag,
        char ** from,
        struct meminfo *mi);
extern struct sys_timer firecracker_timer;


#endif /* __ASM_ARCH_FIRECRACKER_H__ */
