/* linux/include/asm-arm/arch-firecracker/gpio_assignment.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * ARM GPIO  assignments for various boards 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_GPIO_ASSIGNMENTS_H
#define __ASM_ARCH_GPIO_ASSIGNMENTS_H __FILE__

#include <mach/pc20x/gpio.h>

/* CPE SPI GPIOs */
#define ARM_GPIO_SPI_MOSI   GPIO(0)
#define ARM_GPIO_SPI_MISO   GPIO(1)
#define ARM_GPIO_SPI_CLK    GPIO(2)
#define ARM_GPIO_SPI_CS0    GPIO(3)
#define ARM_GPIO_SPI_CS1    GPIO(4)

/* IP202FF GPIOs */
#define ARM_GPIO_HARD_RESET GPIO(1)
#define ARM_GPIO_CPU_LED    GPIO(5)


#endif /* __ASM_ARCH_GPIO_ASSIGNMENTS_H */
