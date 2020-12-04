/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright(C) 2006 picoChip(R) Designs Ltd.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
 
/*****************************************************************************
 *
 * Description: picoChip PC20x ARM Subsystem Peripherals Base Addresses
 *
 *****************************************************************************/
#ifndef __PC20X_H
#define __PC20X_H

#define PC20X_MEM_IF_BASE       0xffe00000
#define PC20X_EBI_BASE          0xffe20000
#define PC20X_UART1_BASE        0xffe40000
#define PC20X_UART2_BASE        0xffe50000
#define PC20X_TIMER_BASE        0xffe60000
#define PC20X_WDOG_BASE         0xffe70000
#define PC20X_GPIO_BASE         0xffe80000
#define PC20X_RTC_CLK_BASE      0xffe90000
#define PC20X_REMAP_BASE        0xffea0000
#define PC20X_AHB_2_PICO_BASE   0xffec0000
#define PC20X_DMAC1_BASE        0xfff00000
#define PC20X_DMAC2_BASE        0xfff10000
#define PC20X_EMAC_BASE         0xfff60000
#define PC20X_SLAVE_PROCIF_BASE 0xfffc0000
#define PC20X_VIC_BASE          0xfffffc00

#endif /* __PC20X_H */
