/* linux/include/asm-arm/arch-firecracker/spi-gpio.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
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

struct firecracker_spigpio_info;
struct spi_board_info;

struct firecracker_spigpio_info {
    unsigned char pin_clk;
    unsigned char pin_mosi;
    unsigned char pin_miso;
    unsigned char pin_cs;

    unsigned long board_size;
    struct spi_board_info *board_info;
};

struct pc302_spi_info {
    u8 cs0_active;
    u8 cs1_active;
    u8 cs2_active;
    u8 cs3_active;
	
    unsigned long board_size;
    struct spi_board_info *board_info;
};


#endif /* __ASM_ARCH_SPIGPIO_H */
