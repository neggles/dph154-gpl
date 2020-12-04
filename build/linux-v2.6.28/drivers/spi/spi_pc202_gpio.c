/**
 * \file pc202spi.c
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright 2009 picoChip Designs LTD, All Rights Reserved.
 * http://www.picochip.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * This file implements an SPI driver for PC202
 * Taken from spi_s3c24xx_gpio.c
 *
 * PC202 GPIO based SPI driver
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <linux/picochip/gpio.h>
#include <linux/picochip/devices/pc202.h>

#include <mach/hardware.h>
#include <mach/spi-gpio.h>
#include <mach/gpio_assignment.h>

/* Internal state data structure */
struct pc202_spigpio {
    /* Bitbang helper context */
    struct spi_bitbang bitbang;

    /* platform data passed at probe time */
    struct firecracker_spigpio_info *info;
};

/* Conversion functions from bit fields to pin numbers */
static int
gpio_set_pin_high(u8 pin)
{
    /* This function will set one or more pins supplied from the pin bit field.
       gpio_set_value() accepts a single pin number. Value must be 0 or
       non-zero */

    int ret = 0;
    unsigned n = 0;
    while(pin)
    {
        if (pin & (1 << n))
        {
            ret = gpio_set_value(PC202_GPIO_PIN_ARM_0+n, 1);
            if (ret < 0)
            {
                return ret;
            }
            pin &= ~(1<< n);
        }
        n++;
    }

    return 0;
}

static int
gpio_set_pin_low(u8 pin)
{
    /* This function will set one or more pins supplied from the pin bit field.
       gpio_set_value() accepts a single pin number. Value must be 0 or
       non-zero */

    int ret = 0;
    unsigned n=0;
    while(pin)
    {
        if (pin & (1 << n))
        {
            ret = gpio_set_value(PC202_GPIO_PIN_ARM_0+n, 0);
            if (ret < 0)
            {
                return ret;
            }

            pin &= ~(1<< n);
        }
        n++;
    }

    return 0;
}

static int
gpio_get_pin_state(u8 pin)
{
    /* This function returns the state of one or more pins supplied in the
       pin bit field. gpio_get_value() accepts a single pin number and
       returns a zero or non-zero number */

    int ret = 0;
    unsigned n=0;
    while(pin)
    {
        if (pin & (1 << n))
        {
            int value = gpio_get_value(PC202_GPIO_PIN_ARM_0+n);
            if (value < 0)
            {
                return value;
            }
            else if (value != 0)
            {
                ret |= (1 << n);
            }

            pin &= ~(1<< n);
        }
        n++;
    }

    return ret;
}

static int
gpio_set_pin_input(u8 pin)
{
    /* This function will set the direction of one or more gpio supplied in
       the pin bit field */

    int ret = 0;
    unsigned n=0;
    while(pin)
    {
        if (pin & (1 << n))
        {
            ret = gpio_direction_input(PC202_GPIO_PIN_ARM_0+n);
            if (ret < 0)
            {
                return ret;
            }

            pin &= ~(1<< n);
        }
        n++;
    }

    return 0;
}
 
static int
gpio_set_pin_output(u8 pin)
{
    /* This function will set the direction of one or more gpio supplied in
       the pin bit field */

    int ret = 0;
    unsigned n=0;
    while(pin)
    {
        if (pin & (1 << n))
        {
            ret = gpio_direction_output(PC202_GPIO_PIN_ARM_0+n, 0);
            if (ret < 0)
            {
                return ret;
            }

            pin &= ~(1<< n);
        }
        n++;
    }

    return 0;
}

static int
GPIO_REQUEST( unsigned pin,
              const char *label )
{
    /* This function will convert the pin bit field into a series of calls
       using a single pin number */

    int ret = 0;
    unsigned n=0;
    while(pin)
    {
        if (pin & (1 << n))
        {
            ret = gpio_request(PC202_GPIO_PIN_ARM_0+n, label);
            if (ret < 0)
            {
                return ret;
            }

            pin &= ~(1<< n);
        }
        n++;
    }

    return 0;
}
 
static void
GPIO_FREE( unsigned pin )
{
    /* This function will convert the pin bit field into a series of calls
       using a single pin number */

    unsigned n=0;
    while(pin)
    {
        if (pin & (1 << n))
        {
            gpio_free(PC202_GPIO_PIN_ARM_0+n);
            pin &= ~(1<< n);
        }
        n++;
    }
}
 
/* Get out internal state from the spi device */
static inline struct pc202_spigpio *spidev_to_sg(struct spi_device *spi)
{
    return spi->controller_data;
}

/* Called by the bitbanged code to set the clock pin */
static inline void setsck(struct spi_device *dev, int on)
{
    struct pc202_spigpio *sg = spidev_to_sg(dev);
    int ret;

    if (on) 
    {
        ret = gpio_set_pin_high(sg->info->pin_clk);
        if ( ret )
            printk( KERN_ALERT "failed to set GPIO value (high) "
                "for clk pin(s) 0x%08x\n", sg->info->pin_clk );
    }
    else
    {
        ret = gpio_set_pin_low(sg->info->pin_clk);
        if ( ret )
            printk( KERN_ALERT "failed to set GPIO value (low) "
                "for clk pin(s) 0x%08x\n", sg->info->pin_clk );
    }
}

/* Called by the bitbanged code to set the mosi pin */
static inline void setmosi(struct spi_device *dev, int on)
{
    struct pc202_spigpio *sg = spidev_to_sg(dev);
    int ret;

    if (on)
    {
        ret = gpio_set_pin_high(sg->info->pin_mosi);
        if ( ret )
            printk( KERN_ALERT "failed to set GPIO value (high) for "
                "mosi pin(s) 0x%08x\n",
                sg->info->pin_mosi );
    }
    else
    {
        ret = gpio_set_pin_low(sg->info->pin_mosi);
        if ( ret )
            printk( KERN_ALERT "failed to set GPIO value (low) for "
                "mosi pin(s) 0x%08x\n", sg->info->pin_mosi );
    }
}

/* Called by the bitbanged code to get the miso pin */
static inline u32 getmiso(struct spi_device *dev)
{
    int ret;
    struct pc202_spigpio *sg = spidev_to_sg(dev);
    ret = gpio_get_pin_state(sg->info->pin_miso);
    if (ret < 0)
        printk( KERN_ALERT "failed to read GPIO value for miso pin %dd\n",
            sg->info->pin_miso );
    return ret ? 1 : 0; 
}

/* Called by the bitbanged code to do a delay */
#define spidelay(x) ndelay(x)

/* Include the bitbanged code */
#define EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>

/* Called by the bitbanged code to send and receive in mode 0 or 2. This
 * function relies on bitbanged code for its operation
 */
static u32 pc202_spigpio_txrx_mode0(struct spi_device *spi,
        unsigned nsecs, u32 word, u8 bits)
{
    printk(KERN_DEBUG "spi txrx0 mode%u word = 0x%08x, bits=%u\n",
          spi->mode, word, bits);
    return bitbang_txrx_be_cpha0(
            spi, nsecs, (spi->mode & SPI_CPOL) ? 1 : 0, word, bits);
}

/* Called by the bitbanged code to send and receive in mode 1 or 3. This
 * function relies on bitbanged code for its operation
 */
static u32 pc202_spigpio_txrx_mode1(struct spi_device *spi,
        unsigned nsecs, u32 word, u8 bits)
{
    printk(KERN_DEBUG "spi txrx1 mode%u word = 0x%08x, bits=%u\n",
         spi->mode, word, bits);
    return bitbang_txrx_be_cpha1(
            spi, nsecs, (spi->mode & SPI_CPOL) ? 1 : 0, word, bits);
}

/* Called by the bitbanged code to select the chip */
static void pc202_spigpio_chipselect(struct spi_device *dev, int value)
{
    struct pc202_spigpio *sg = spidev_to_sg(dev);
    int ret;
    int gpio_pin = dev->chip_select == 0 ? ARM_GPIO_SPI_CS0 : ARM_GPIO_SPI_CS1;

    /* Reset any active chip selects */
    ret = gpio_set_pin_low(sg->info->pin_cs);
    if (ret)
        printk( KERN_ALERT "failed to set GPIO value (low) for cs pin(s) "
            "0x%08x\n", sg->info->pin_cs );

    /* If the chip is to be activated, set its chip select pattern */
    if (value == BITBANG_CS_ACTIVE)
    {
        printk(KERN_DEBUG "spi CS high 0x%02x\n", dev->chip_select);
        ret = gpio_set_pin_high(gpio_pin);
        if (ret)
            printk(KERN_ALERT "failed to set GPIO value (high) for gpio "
                "pin(s) 0x%08x (ret=%d)\n", gpio_pin, ret ); 
    }
    else
    {
        printk(KERN_DEBUG "spi CS low\n");

        /* Provide one clock cycle to reset the SPI transfer */
        setsck(dev, 0);
        setsck(dev, 1);
    }
}


/* Probe function for the GPIO bitbashed SPI. This uses the bitbash helper
 * code for its implementation.
 */
static int pc202_spigpio_probe(struct platform_device *dev)
{
    struct spi_master   *master;
    struct pc202_spigpio  *sp;
    int ret;
    int i;

    master = spi_alloc_master(&dev->dev, sizeof(struct pc202_spigpio));
    if (master == NULL) {
        dev_err(&dev->dev, "failed to allocate spi master\n");
        ret = -ENOMEM;
        goto err;
    }

    sp = spi_master_get_devdata(master);

    platform_set_drvdata(dev, sp);

    /* We have 2 masters - one for each FPGA. */
    master->num_chipselect = 2;
    master->bus_num = 0;

    /* copy in the platform data */
    sp->info = dev->dev.platform_data;

    /* setup spi bitbang adaptor */
    sp->bitbang.master = spi_master_get(master);
    sp->bitbang.chipselect = pc202_spigpio_chipselect;

    sp->bitbang.txrx_word[SPI_MODE_0] = pc202_spigpio_txrx_mode0;
    sp->bitbang.txrx_word[SPI_MODE_1] = pc202_spigpio_txrx_mode1;
    sp->bitbang.txrx_word[SPI_MODE_2] = pc202_spigpio_txrx_mode0;
    sp->bitbang.txrx_word[SPI_MODE_3] = pc202_spigpio_txrx_mode1;

    /* acquire spi pins */
    ret = GPIO_REQUEST(sp->info->pin_clk, "SPI CLK");
    if (ret)
        printk( KERN_ALERT "failed to acquire SPI CLK pin(s) 0x%08x\n",
            sp->info->pin_clk );

    ret = GPIO_REQUEST(sp->info->pin_mosi, "SPI MOSI");
    if (ret)
        printk( KERN_ALERT "failed to acquire SPI MOSI pin(s) 0x%08x\n",
            sp->info->pin_mosi );

    ret = GPIO_REQUEST(sp->info->pin_cs, "SPI CS");
    if (ret)
        printk(KERN_ALERT "failed to acquire SPI CS pin(s) 0x%08x\n",
            sp->info->pin_cs );

    ret = GPIO_REQUEST(sp->info->pin_miso, "SPI MISO");
    if (ret)
        printk(KERN_ALERT "failed to acquire SPI MISO pin(s) 0x%08x\n",
            sp->info->pin_miso );

    /* set state of spi pins */
    ret = gpio_set_pin_low(sp->info->pin_clk);
    if (ret)
        printk(KERN_ALERT "failed to set GPIO value (low) for clk pin(s) "
            "0x%08x\n", sp->info->pin_clk );

    ret = gpio_set_pin_low(sp->info->pin_mosi);
    if (ret)
        printk( KERN_ALERT "failed to set GPIO value (low) for mosi pin(s) "
            "0x%08x\n", sp->info->pin_mosi );

    ret = gpio_set_pin_low(sp->info->pin_cs);
    if (ret)
        printk(KERN_ALERT "failed to set GPIO value (low) for cs pin(s) "
            "0x%08x\n", sp->info->pin_cs );

    /* Set the pin directions */
    ret = gpio_set_pin_output(sp->info->pin_clk);
    if (ret)
        printk(KERN_ALERT "failed to set GPIO direction (output) for clk "
            "pin(s) 0x%08x\n", sp->info->pin_clk );

    ret = gpio_set_pin_output(sp->info->pin_mosi);
    if (ret)
        printk(KERN_ALERT "failed to set GPIO direction (output) for mosi "
            "pin(s) 0x%08x\n", sp->info->pin_mosi );

    ret = gpio_set_pin_input(sp->info->pin_miso);
    if (ret)
        printk(KERN_ALERT "failed to set GPIO direction (input) for miso "
            "pin(s) 0x%08x\n", sp->info->pin_miso );

    ret = gpio_set_pin_output(sp->info->pin_cs);
    if (ret)
        printk(KERN_ALERT "failed to set GPIO direction (output) for cs pin(s) "
            "0x%08x\n", sp->info->pin_cs );

    ret = spi_bitbang_start(&sp->bitbang);
    if (ret)
        goto err_no_bitbang;

    /* register the chips to go with the board */

    for (i = 0; i < sp->info->board_size; i++) {
        dev_info(&dev->dev, "registering %p: %s\n",
                &sp->info->board_info[i],
                sp->info->board_info[i].modalias);

        sp->info->board_info[i].controller_data = sp;
        spi_new_device(master, sp->info->board_info + i);
    }

    return 0;

err_no_bitbang:
    spi_master_put(sp->bitbang.master);
err:
    return ret;

}

/* Remove the GPIO bitbashed SPI interface */
static int pc202_spigpio_remove(struct platform_device *dev)
{
    struct pc202_spigpio *sp = platform_get_drvdata(dev);

    spi_bitbang_stop(&sp->bitbang);
    spi_master_put(sp->bitbang.master);

    GPIO_FREE(sp->info->pin_clk);
    GPIO_FREE(sp->info->pin_mosi);
    GPIO_FREE(sp->info->pin_cs);
    GPIO_FREE(sp->info->pin_miso);

    return 0;
}

/* all gpio should be held over suspend/resume, so we should
 * not need to deal with this
 */

#define pc202_spigpio_suspend NULL
#define pc202_spigpio_resume NULL

/* GPIO bitbashed SPI interface declaration structure */
static struct platform_driver pc202_spigpio_drv = {
    .probe      = pc202_spigpio_probe,
    .remove     = pc202_spigpio_remove,
    .suspend    = pc202_spigpio_suspend,
    .resume     = pc202_spigpio_resume,
    .driver     = {
        .name   = "pc20x-spi-gpio",
        .owner  = THIS_MODULE,
    },
};

/* Module initialisation, declare the SPI interface */
static int __init pc202_spigpio_init(void)
{
    printk("pc20x SPI-GPIO driver, version 0.01 loaded\n");
    return platform_driver_register(&pc202_spigpio_drv);
}

/* Module shut-down, remove the SPI interface */
static void __exit pc202_spigpio_exit(void)
{
    platform_driver_unregister(&pc202_spigpio_drv);
}

module_init(pc202_spigpio_init);
module_exit(pc202_spigpio_exit);

MODULE_DESCRIPTION("pc20x bitbang SPI Driver");
MODULE_LICENSE("GPL");


