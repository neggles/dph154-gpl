/* linux/drivers/spi/spi_pc302_gpio.c
 *
 *
 * Based on spi_firecracker_gpio.c which was taken from spi_s3c24xx_gpio.c;
 *
 * Copyright (c) 2009 ip.access Ltd
 *
 * PC302 GPIO based SPI driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include <mach/hardware.h>
#include <mach/spi-gpio.h>
#include <mach/gpio.h>

/* ENABLE_DEBUG - to add printf's showing activity on the SPI interface */
#undef ENABLE_DEBUG

/* Internal state data structure */
struct pc302_spigpio {
    /* Bitbang helper context */
    struct spi_bitbang bitbang;

    /* platform data passed at probe time */
    struct pc7302_spigpio_info *info;
};

/* Get out internal state from the spi device */
static inline struct pc302_spigpio *spidev_to_sg(struct spi_device *spi)
{
    return spi->controller_data;
}

/* Called by the bitbanged code to set the clock pin */
static inline void setsck(struct spi_device *dev, int on)
{
    struct pc302_spigpio *sg = spidev_to_sg(dev);
    if (on) {
        gpio_set_value(sg->info->pin_clk, 1);
    }
    else {
        gpio_set_value(sg->info->pin_clk, 0);
    }
}

/* Called by the bitbanged code to set the mosi pin */
static inline void setmosi(struct spi_device *dev, int on)
{
    struct pc302_spigpio *sg = spidev_to_sg(dev);
    if (on) {
        gpio_set_value(sg->info->pin_mosi, 1);
    }
    else {
        gpio_set_value(sg->info->pin_mosi, 0);
    }
}

/* Called by the bitbanged code to get the miso pin */
static inline u32 getmiso(struct spi_device *dev)
{
    (void)dev;
    return 0;
}

/* Called by the bitbanged code to do a delay */
#define spidelay(x) ndelay(x)

/* Include the bitbanged code */
#define EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>

/* Called by the bitbanged code to send and receive in mode 0 or 2. This
 * function relies on bitbanged code for its operation
 */
static u32 pc302_spigpio_txrx_mode0(struct spi_device *spi,
        unsigned nsecs, u32 word, u8 bits)
{
    /* TODO Debug, remove */
#ifdef ENABLE_DEBUG
    printk("spi txrx0 mode%u word = 0x%08x, bits=%u\n", spi->mode, word, bits);
#endif
    return bitbang_txrx_be_cpha0(
            spi, nsecs, (spi->mode & SPI_CPOL) ? 1 : 0, word, bits);
}

/* Called by the bitbanged code to send and receive in mode 1 or 3. This
 * function relies on bitbanged code for its operation
 */
static u32 pc302_spigpio_txrx_mode1(struct spi_device *spi,
        unsigned nsecs, u32 word, u8 bits)
{
    /* TODO Debug, remove */
#ifdef ENABLE_DEBUG
    printk("spi txrx1 mode%u word = 0x%08x, bits=%u\n", spi->mode, word, bits);
#endif
    return bitbang_txrx_be_cpha1(
            spi, nsecs, (spi->mode & SPI_CPOL) ? 1 : 0, word, bits);
}

/* Called by the bitbanged code to select the chip */
static void pc302_spigpio_chipselect(struct spi_device *dev, int value)
{
    struct pc302_spigpio *sg = spidev_to_sg(dev);
    int gpio_pin = dev->chip_select == 0 ? sg->info->pin_cs_0 : sg->info->pin_cs_1;

    /* Reset any active chip selects */
    gpio_set_value(sg->info->pin_cs_0, 1);
    gpio_set_value(sg->info->pin_cs_1, 1);

    /* If the chip is to be activated, set its chip select pattern */
    if (value == BITBANG_CS_ACTIVE) {

#ifdef ENABLE_DEBUG
        printk("spi CS 0x%02x asserted\n", dev->chip_select);
#endif

        gpio_set_value(gpio_pin, 0);
    }
    else {

#ifdef ENABLE_DEBUG
        printk("spi CS de-asserted\n");
#endif
        /* Provide one clock cycle to reset the SPI transfer */
        setsck(dev, 0);
        setsck(dev, 1);
    }
}


/* Probe function for the GPIO bitbashed SPI. This uses the bitbash helper
 * code for its implementation.
 */
static int pc302_spigpio_probe(struct platform_device *dev)
{
    struct spi_master   *master;
    struct pc302_spigpio  *sp;
    int ret = 0;
    int i;

    master = spi_alloc_master(&dev->dev, sizeof(struct pc302_spigpio));
    if (master == NULL) {
        dev_err(&dev->dev, "failed to allocate spi master\n");
        ret = -ENOMEM;
        goto err;
    }

    sp = spi_master_get_devdata(master);

    platform_set_drvdata(dev, sp);

    /* We have 2 chip selects. */
    master->num_chipselect = 2;
    master->bus_num = 0;

    /* copy in the platform data */
    sp->info = dev->dev.platform_data;

    /* setup spi bitbang adaptor */
    sp->bitbang.master = spi_master_get(master);
    sp->bitbang.chipselect = pc302_spigpio_chipselect;

    sp->bitbang.txrx_word[SPI_MODE_0] = pc302_spigpio_txrx_mode0;
    sp->bitbang.txrx_word[SPI_MODE_1] = pc302_spigpio_txrx_mode1;
    sp->bitbang.txrx_word[SPI_MODE_2] = pc302_spigpio_txrx_mode0;
    sp->bitbang.txrx_word[SPI_MODE_3] = pc302_spigpio_txrx_mode1;

    /* set state of spi pins */
    ret = gpio_request(sp->info->pin_mosi, "PC302 BB SPI MOSI");
    if (ret)
    {
        goto err_no_mosi;
    }
    ret = gpio_request(sp->info->pin_clk,  "PC302 BB SPI CLK");
    if (ret)
    {
        goto err_no_clk;
    }
    ret = gpio_request(sp->info->pin_cs_0, "PC302 BB SPI CS0");
    if (ret)
    {
        goto err_no_cs0;
    }
    ret = gpio_request(sp->info->pin_cs_1, "PC302 BB SPI CS1");
    if (ret)
    {
        goto err_no_cs1;
    }

    gpio_set_value(sp->info->pin_clk,  0);
    gpio_set_value(sp->info->pin_mosi, 0);
    gpio_set_value(sp->info->pin_cs_0, 1);
    gpio_set_value(sp->info->pin_cs_1, 1);

    gpio_direction_output(sp->info->pin_clk,  0);
    gpio_direction_output(sp->info->pin_mosi, 0);
    gpio_direction_output(sp->info->pin_cs_0, 1);
    gpio_direction_output(sp->info->pin_cs_1, 1);

    gpio_set_value(sp->info->pin_clk,  0);
    gpio_set_value(sp->info->pin_mosi, 0);
    gpio_set_value(sp->info->pin_cs_0, 1);
    gpio_set_value(sp->info->pin_cs_1, 1);

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
    gpio_free(sp->info->pin_cs_1);
err_no_cs1:
    gpio_free(sp->info->pin_cs_0);
err_no_cs0:
    gpio_free(sp->info->pin_clk);
err_no_clk:
    gpio_free(sp->info->pin_mosi);
err_no_mosi:
    spi_master_put(sp->bitbang.master);
err:
    return ret;

}

/* Remove the GPIO bitbashed SPI interface */
static int pc302_spigpio_remove(struct platform_device *dev)
{
    struct pc302_spigpio *sp = platform_get_drvdata(dev);

    spi_bitbang_stop(&sp->bitbang);
    spi_master_put(sp->bitbang.master);

    gpio_free(sp->info->pin_cs_1);
    gpio_free(sp->info->pin_cs_0);
    gpio_free(sp->info->pin_clk);
    gpio_free(sp->info->pin_mosi);
    
    return 0;
}

/* all gpio should be held over suspend/resume, so we should
 * not need to deal with this
 */

#define pc302_spigpio_suspend NULL
#define pc302_spigpio_resume NULL

/* GPIO bitbashed SPI interface declaration structure */
static struct platform_driver pc302_spigpio_drv = {
    .probe      = pc302_spigpio_probe,
    .remove     = pc302_spigpio_remove,
    .suspend    = pc302_spigpio_suspend,
    .resume     = pc302_spigpio_resume,
    .driver     = {
        .name   = "pc302-spi-gpio",
        .owner  = THIS_MODULE,
    },
};

/* Module initialisation, declare the SPI interface */
static int __init pc302_spigpio_init(void)
{
    printk("pc302 SPI-GPIO driver, version 0.01 loaded\n");
    return platform_driver_register(&pc302_spigpio_drv);
}

/* Module shut-down, remove the SPI interface */
static void __exit pc302_spigpio_exit(void)
{
    platform_driver_unregister(&pc302_spigpio_drv);
}

module_init(pc302_spigpio_init);
module_exit(pc302_spigpio_exit);

MODULE_DESCRIPTION("pc302 bitbang SPI Driver");
MODULE_LICENSE("GPL");


