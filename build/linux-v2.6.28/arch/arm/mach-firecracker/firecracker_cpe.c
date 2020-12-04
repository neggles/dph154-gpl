/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 *  linux/arch/arm/mach-firecracker/firecracker_cpe.c
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/picochip/fpga_cpe20x.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/spi-gpio.h>
#include <mach/pc20x/gpio.h>
#include <mach/gpio_assignment.h>

/* PC20x gpio assignments */
#include <linux/picochip/devices/pc202.h>

#include <asm/mach/arch.h>

#include <linux/i2c-gpio.h>

#include "core.h"


/* SPI:
*/

static struct fpga_platform_data rc_fpga_pdata = {
    .fpga_device_id     = FPGA_RC_ID,
};

static struct fpga_platform_data ad_fpga_pdata = {
    .fpga_device_id     = FPGA_AD_ID,
};

/* Note: The FPGA SPI can go up to 40MHz in theory but our bit-banged
 * driver will never get as fast as that. I set 20MHz for safety, the
 * bit-banged driver will go as fast as it can (zero delay).
 */
static struct spi_board_info firecracker_cpe_spi_board_info[] = {
    [0] = { /* AD FPGA */
        .modalias       = "fpga_cpe20x",
        .bus_num        = 0,
        .chip_select    = 0,
        .max_speed_hz   = 20000000,
        .platform_data  = &ad_fpga_pdata,
    },
    [1] = { /* RC_FPGA */
        .modalias       = "fpga_cpe20x",
        .bus_num        = 0,
        .chip_select    = 1,
        .max_speed_hz   = 20000000,
        .platform_data  = &rc_fpga_pdata,
    },
};


static struct firecracker_spigpio_info spi_platform_data = {
    /* Pin mapping taken from PC7205-2 Register Map */
    .pin_clk        = ARM_GPIO_SPI_CLK,
    .pin_mosi       = ARM_GPIO_SPI_MOSI,
    .pin_miso       = ARM_GPIO_SPI_MISO,
    .pin_cs         = ARM_GPIO_SPI_CS0 | ARM_GPIO_SPI_CS1,
    .board_size     = ARRAY_SIZE(firecracker_cpe_spi_board_info),
    .board_info     = firecracker_cpe_spi_board_info,
};


static struct platform_device spi_device = {
    .name           = "pc20x-spi-gpio",
    .id             = 0,
    .dev            = {
        .coherent_dma_mask = 0xffffffff,
        .platform_data = &spi_platform_data,
    },
};

/* I2C:
 */

static struct i2c_gpio_platform_data i2c_gpio_data = {
        .sda_pin	= PC202_GPIO_PIN_SDGPIO_14,
        .scl_pin	= PC202_GPIO_PIN_SDGPIO_15,
        .sda_is_open_drain  = 0,
        .scl_is_open_drain  = 1,
        .scl_is_output_only = 1,
        .udelay = 1,
};

static struct platform_device pc202_i2c_device = {
	.name		= "i2c-gpio",
	.id		= 0,
	.dev		= {
		.platform_data	= &i2c_gpio_data,
	},
};


static int __init firecracker_cpe_init(void)
{
    if ( machine_is_pc72052_i10_revb() || machine_is_pc7802() ) {

        platform_device_register(&spi_device);
    }

    if ( machine_is_pc7802() ) {
        platform_device_register(&pc202_i2c_device);
    }

        return 0;
}

arch_initcall(firecracker_cpe_init);

#if defined(CONFIG_MACH_PC72052_I10_REVB)
/* MACH_TYPE_PC72052_I10_REVB */
MACHINE_START(PC72052_I10_REVB, "pc7205-2 1.0. Rev B BOM")
    /* Maintainer: picoChip */
    .phys_io        = 0xffe00000,
    .io_pg_offst    = ((0xfee00000) >> 18) & 0xfffc,
    .boot_params    = 0x00000100,
    .map_io         = firecracker_map_io,
    .init_irq       = firecracker_init_irq,
    .timer          = &firecracker_timer,
    .init_machine   = firecracker_init,
    .fixup          = firecracker_fixup,
MACHINE_END
#endif

#if defined(CONFIG_MACH_PC7802)
/* MACH_TYPE_PC7802 */
MACHINE_START(PC7802, "pc7802 Platform")
    /* Maintainer: picoChip */
    .phys_io        = 0xffe00000,
    .io_pg_offst    = ((0xfee00000) >> 18) & 0xfffc,
    .boot_params    = 0x00000100,
    .map_io         = firecracker_map_io,
    .init_irq       = firecracker_init_irq,
    .timer          = &firecracker_timer,
    .init_machine   = firecracker_init,
    .fixup          = firecracker_fixup,
MACHINE_END
#endif
