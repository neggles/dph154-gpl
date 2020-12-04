/* debug-macro.S
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Debugging macro include header
 *
 * Copyright (c) 2008 picoChip Designs Ltd.
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

		.macro	addruart,rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx,      #0xfe000000	@ virtual base
		movne	\rx,      #0x80000000
		orr	\rx, \rx, #0x00230000       @ UART1
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
