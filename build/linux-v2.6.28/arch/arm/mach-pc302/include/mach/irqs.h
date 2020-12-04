/* Copyright (c) 2008 picoChip Designs Ltd.
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __IRQS_H__
#define __IRQS_H__

/*****************************************************************************/
/* IRQ Index & Bit Field Manipulation                                        */
/*****************************************************************************/

/* VIC0 IRQ Indexes */
#define IRQ_EMAC            31 + 32
#define IRQ_NPMUIRQ         30 + 32
#define IRQ_NDMAEXTERRIRQ   29 + 32
#define IRQ_NDMASIRQ        28 + 32
#define IRQ_NDMAIRQ         27 + 32
#define IRQ_DMAC2           26 + 32
#define IRQ_DMAC1           25 + 32
#define IRQ_IPSEC           24 + 32
#define IRQ_SRTP            23 + 32
#define IRQ_AES             22 + 32
#define IRQ_AXI2PICO8       21 + 32
#define IRQ_AXI2PICO7       20 + 32
#define IRQ_AXI2PICO6       19 + 32
#define IRQ_AXI2PICO5       18 + 32
#define IRQ_AXI2PICO4       17 + 32
#define IRQ_AXI2PICO3       16 + 32
#define IRQ_AXI2PICO2       15 + 32
#define IRQ_AXI2PICO1       14 + 32
#define IRQ_AXI2PICO0       13 + 32
#define IRQ_AXI2CFG         12 + 32
#define IRQ_WDG             11 + 32
#define IRQ_SSI             10 + 32
#define IRQ_AXI_RD_ERR      9  + 32
#define IRQ_AXI_WR_ERR      8  + 32
#define IRQ_TIMER3          7  + 32
#define IRQ_TIMER2          6  + 32
#define IRQ_TIMER1          5  + 32
#define IRQ_TIMER0          4  + 32
#define IRQ_COMMTX          3  + 32
#define IRQ_COMMRX          2  + 32
#define IRQ_SWI             1  + 32

#define VIC0_IRQ_USED_MASK  0xFFFFFFFE00000000LLU

/* VIC1 IRQ Indexes */
#define IRQ_UART1           10
#define IRQ_UART2           9
#define IRQ_RTC             8
#define IRQ_GPIO7           7
#define IRQ_GPIO6           6
#define IRQ_GPIO5           5
#define IRQ_GPIO4           4
#define IRQ_GPIO3           3
#define IRQ_GPIO2           2
#define IRQ_GPIO1           1
#define IRQ_GPIO0           0

#define VIC1_IRQ_USED_MASK  0x7FF

#define NR_IRQS             64

#endif /* __IRQS_H__ */
