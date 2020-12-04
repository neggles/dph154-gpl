/*
 *  linux/include/asm-arm/arch-firecracker/memory.h
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#define PHYS_OFFSET	UL(0x00000000)

/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *              address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *              to an address that the kernel can use.
 */
#define __virt_to_bus(x)	((x) - PAGE_OFFSET)
#define __bus_to_virt(x)	((x) + PAGE_OFFSET)

#ifdef CONFIG_DISCONTIGMEM

/*
 * The nodes are the followings:
 *
 *   node 0: 0x0000.0000 - 0x03ff.ffff
 *   node 1: 0x0400.0000 - 0x07ff.ffff
 *   node 2: 0x0800.0000 - 0x0bff.ffff
 *   node 3: 0x0c00.0000 - 0x0fff.ffff
 */

/*
 * Given a kernel address, find the home node of the underlying memory.
 */
#define KVADDR_TO_NID(addr) \
	(((unsigned long)(addr) - PAGE_OFFSET) >> NODE_MAX_MEM_SHIFT)

/*
 * Given a page frame number, convert it to a node id.
 */
#define PFN_TO_NID(pfn) \
	(((pfn) - PHYS_PFN_OFFSET) >> (NODE_MAX_MEM_SHIFT - PAGE_SHIFT))

/*
 * Given a kaddr, ADDR_TO_MAPBASE finds the owning node of the memory
 * and return the mem_map of that node.
 */
#define ADDR_TO_MAPBASE(kaddr)  NODE_MEM_MAP(KVADDR_TO_NID(kaddr))

/*
 * Given a page frame number, find the owning node of the memory
 * and return the mem_map of that node.
 */
#define PFN_TO_MAPBASE(pfn)     NODE_MEM_MAP(PFN_TO_NID(pfn))

/*
 *  Given a kaddr, LOCAL_MEM_MAP finds the owning node of the memory
 *  and returns the index corresponding to the appropriate page in the
 *  node's mem_map.
 */
#define LOCAL_MAP_NR(addr) \
        (((unsigned long)(addr) & (NODE_MAX_MEM_SIZE - 1)) >> PAGE_SHIFT)

#define NODE_MAX_MEM_SHIFT	26
#define NODE_MAX_MEM_SIZE	(1 << NODE_MAX_MEM_SHIFT)

#endif /* CONFIG_DISCONTIGMEM */

#endif

