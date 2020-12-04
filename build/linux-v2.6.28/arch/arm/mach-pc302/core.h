/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05 
 *****************************************************************************/

/*
 *  linux/arch/arm/mach-pc302/core.h
 *
 * Copyright (c) 2008 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

#ifndef __ASM_ARCH_PC302_H__
#define __ASM_ARCH_PC302_H__

#include <linux/amba/bus.h>

extern void __init pc302_core_init(void);
extern void __init pc302_init_irq(void);
extern void __init pc302_map_io(void);
extern struct sys_timer pc302_timer;

#endif /* __ASM_ARCH_PC302_H__ */
