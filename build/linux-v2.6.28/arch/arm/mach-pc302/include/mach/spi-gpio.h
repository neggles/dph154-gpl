/* linux/include/asm-arm/arch-pc302/spi-gpio.h
 *
 * BSP Version: 3.1.2, RevisionID: c296f5c, Date: 20090731 10:20:02
 *
 * Taken from linux/include/asm-arm/arch-firecracker/spi.h
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * Firecracker - SPI Controller platform_device info
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_SPIGPIO_H
#define __ASM_ARCH_SPIGPIO_H __FILE__

#include <linux/spi/spi.h>

struct pc302_spi_info {

    u8 cs0_active;
    u8 cs1_active;
    u8 cs2_active;
    u8 cs3_active;
	
    unsigned long board_size;
    struct spi_board_info *board_info;
};


#endif /* __ASM_ARCH_SPIGPIO_H */
