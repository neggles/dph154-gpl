/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 *  linux/arch/arm/mach-pc302/pc302_cpe.c
 *
 * Copyright (c) 2008 picoChip Designs Ltd.
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

#ifndef CONFIG_IPACCESS_IP302FF
#include <linux/picochip/fpga_cpe20x.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/flash.h>
#include <linux/spi/spi.h>
#endif

#ifdef CONFIG_IPACCESS_IP302FF
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio.h>
#include <mach/spi-gpio.h>
#endif

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>

#include "core.h"

#if defined(CONFIG_IPACCESS_IP302FF) && (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE))

/* i2c */
static struct i2c_gpio_platform_data pc302_i2c_bus_data = {
	.sda_pin = PC302_GPIO_PIN_ARM_2,
	.scl_pin = PC302_GPIO_PIN_ARM_0,
	.udelay  = 2,  /* Between 100kHz and 400kHz, nominally 250kHz */
	.timeout = 100,
};

static struct platform_device pc302_i2c_bus_device = {
	.name		= "i2c-gpio",
	.id		= 0,
	.dev = {
		.platform_data = &pc302_i2c_bus_data,
	}
};

static struct i2c_board_info __initdata pc302_i2c_devices[] = {
	{
		I2C_BOARD_INFO("max6635",   0x4B),
	},
	{
		I2C_BOARD_INFO("atmel_twi", 0x29),
	},
	{
		I2C_BOARD_INFO("ad7995",    0x28),
	},
	{
		I2C_BOARD_INFO("micrel",    0x5F),
	},
};

#endif /* CONFIG_IPACCESS_IP302FF && (CONFIG_I2C_GPIO || CONFIG_I2C_GPIO_MODULE) */


#if defined(CONFIG_IPACCESS_IP302FF) && (defined(CONFIG_SPI_PC302_IPA) || defined(CONFIG_SPI_PC302_IPA_MODULE))

/* SPI:
*/
    
/* According to internet, the Synopsys device will deselect the chip select between words
 * in SPI modes 0 and 2, so we have to use modes 3 and 1 respectively instead.
 */
static struct spi_board_info ip302ff_spi_board_info[] = {
    [0] = { /* Thermal Sensor */
        .modalias       = "max6662",
        .bus_num        = 0,
        .chip_select    = 3,
        .max_speed_hz   = 10000, /* 10kHz */
        .mode           = SPI_3WIRE | SPI_MODE_3,  /* Idle clock is high.  Latch on rising edge (second edge) */
        .platform_data  = NULL,
    },
    [1] = { /* Reference Clock Control DAC */
        .modalias       = "dac7512",
        .bus_num        = 0,
        .chip_select    = 2,
        .max_speed_hz   = 10000, /* 10kHz (not 20MHz, we'd get only 80 CPU cycles between writes) */
        .mode           = SPI_MODE_1,  /* Idle clock is low.  Latch on falling edge (second edge) */
        .platform_data  = NULL,
    },
};


static void
pc302spi_platform_release( struct device *dev )
{
    /* This function is intentionally left blank. */
}

static struct resource pc302spi_resources[] = {
    {
        .start = PC302_SSI_BASE,
        .end   = PC302_SSI_BASE + 0xffff,
        .flags = IORESOURCE_MEM,
    },
    {
        .start = IRQ_SSI,
        .end   = IRQ_SSI,
        .flags = IORESOURCE_IRQ,
    },
};

static struct pc302_spi_info pc302spi_platform_data = {
    .cs0_active     = 0,  /* EBI: NOR Flash */
    .cs1_active     = 0,  /* EBI: Debug interface */
    .cs2_active     = 1,  /* SPI: Reference Clock Control DAC */
    .cs3_active     = 1,  /* SPI: Thermal Sensor */
    .board_size     = ARRAY_SIZE(ip302ff_spi_board_info),
    .board_info     = ip302ff_spi_board_info,
};


static struct platform_device pc302spi_device = {
    .name = "pc302-spi",
    .id = 0,
    .dev = {
        .coherent_dma_mask = 0xffffffff,
        .release = pc302spi_platform_release,
        .platform_data = &pc302spi_platform_data,
    },
    .num_resources = ARRAY_SIZE( pc302spi_resources ),
    .resource = pc302spi_resources,
};

#endif /* CONFIG_IPACCESS_IP302FF && (CONFIG_SPI_PC302_IPA || CONFIG_SPI_PC302_IPA_MODULE) */

#if defined(CONFIG_SPI_PC302) || defined(CONFIG_SPI_PC302_MODULE)

/* PC7302 platforms have had a variety of different SPI Flash devices fitted.
 *
 * Spansion S25FL128P (128 Mbit) devices
 * Numonyx M25P05 (512 Kbit) devices
 *
 * This setup should be fine for all of them with the proviso that the
 * partition called "SPI: Data" may not actually be available for use.
 *
 * Note: When this BSP is ported to other hardware platforms this structure
 *       will need to be modified appropriately.
 */
static struct mtd_partition pc7302_spi_flash_partitions[] = {
    {
        .name   = "SPI: First Stage Boot Loader",
        .offset = 0,
        .size   = SZ_64K,
    },
    {
        .name   = "SPI: Data",
        .offset = MTDPART_OFS_APPEND,
        .size   = MTDPART_SIZ_FULL,
    },
};

static struct flash_platform_data pc7302_spi_flash_data = {

    .name       = "pc7302 spi flash",
    .parts      = pc7302_spi_flash_partitions,
    .nr_parts   = ARRAY_SIZE(pc7302_spi_flash_partitions),
};

static struct spi_board_info pc7302_spi_board_info[] __initdata = {
    {
	.modalias	= "m25p80",
	.platform_data  = &pc7302_spi_flash_data,
        .mode           = SPI_MODE_3,
        .max_speed_hz	= 2000000,
        .chip_select	= 0,
    }
};

#endif /* defined(CONFIG_SPI_PC302) || defined(CONFIG_SPI_PC302_MODULE) */

static int __init pc7302_init(void)
{
#if defined(CONFIG_SPI_PC302) || defined(CONFIG_SPI_PC302_MODULE)
    spi_register_board_info(pc7302_spi_board_info,
                            ARRAY_SIZE(pc7302_spi_board_info));
#endif

#if defined(CONFIG_IPACCESS_IP302FF) && (defined(CONFIG_SPI_PC302_IPA) || defined(CONFIG_SPI_PC302_IPA_MODULE))
    platform_device_register(&pc302spi_device);
#endif
    
#if defined(CONFIG_IPACCESS_IP302FF) && (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE))
    platform_device_register(&pc302_i2c_bus_device);
    i2c_register_board_info(0, pc302_i2c_devices, ARRAY_SIZE(pc302_i2c_devices));
#endif
    
    return 0;
}
arch_initcall(pc7302_init);

MACHINE_START(PC7302, "PC7302")
    /* Maintainer: picoChip */
    .phys_io        = 0x80000000,
    .io_pg_offst    = (IO_ADDRESS(0x80000000) >> 18) & 0xfffc,
    .boot_params    = 0x00000100,
    .map_io         = pc302_map_io,
    .init_irq       = pc302_init_irq,
    .timer          = &pc302_timer,
    .init_machine   = pc302_core_init,
MACHINE_END
