/*
 * arch/ppc/platforms/picohdp.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * picoHDP board definitions
 *
 * Copyright 2005 Zetetica Ltd <david.warman@zetetica.co.uk>
 *
 * derived from MPC8560 ADS support by Kumar Gala <kumar.gala@freescale.com>
 * Copyright 2004 Freescale Semiconductor Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __MACH_PICOHDP_H
#define __MACH_PICOHDP_H

#include <syslib/ppc85xx_setup.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <asm/ppcboot.h>

#define BOARD_CCSRBAR           ((uint)0x80000000)

extern int picohdp_show_cpuinfo(struct seq_file *m);
extern void picohdp_init_IRQ(void) __init;
extern void picohdp_map_io(void) __init;

/* PCI interrupt controller */
/* We don't have any real PCI slots - there is only 1 PCI device,
 * the HiFn crypto processor in "slot" 17, on external IRQ8.
 */
#define PIRQA           MPC85xx_IRQ_EXT8
#define PIRQB           0
#define PIRQC           0
#define PIRQD           0

#define MPC85XX_PCI1_LOWER_MEM  0x40000000
#define MPC85XX_PCI1_UPPER_MEM  0x47ffffff

#define MPC85XX_PCI1_LOWER_IO   0x48000000
#define MPC85XX_PCI1_UPPER_IO   0x4fffffff

#define MPC85XX_PCI1_IO_SIZE    0x01000000

#define MPC85XX_PCI1_IO_BASE    0x40000000
#define MPC85XX_PCI1_MEM_OFFSET 0x00000000

#define CPM_MAP_ADDR	(CCSRBAR + MPC85xx_CPM_OFFSET)

/* Added in here during the port from 2.6.17 kernel to 2.6.18 kernel */
#define F1_RXCLK    11
#define F1_TXCLK    10
#define F2_RXCLK    15
#define F2_TXCLK    16
#define F3_RXCLK    13
#define F3_TXCLK    14

#endif				/* __MACH_PICOHDP_H */
