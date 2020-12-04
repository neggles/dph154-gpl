/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file pc302_spi.c
* \brief SPI driver for the PC302.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

/* Includes ---------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <linux/spi/spi.h>
#include <mach/hardware.h>
#include "pc302_spi.h"

/* Macros ------------------------------------------------------------------ */
/*!
 * The name used for the platform device and driver to allow Linux to match up
 * the correct ends.
 */
#define CARDNAME "pc302-spi"

/*!
 * A name for this module
 */
#define TITLE "PC302 SPI Driver"

/*!
 * AHB Bus clock frequency
 */
#define PC302_AHB_CLOCK_FREQ    (PC302_TIMER_FREQ)

/*!
 * Max number of queued SPI messages this driver can handle
 */
 #define PC302_MAX_QUEUED_SPI_MESSAGES (2)

/* Constants --------------------------------------------------------------- */
/* !
 *  \brief This structure is used for generic and device specific
 *         spi driver data.
 */
struct pc302_spi
{
    void __iomem    *regs;

    /* Max clock speed that the SPI block can run at */
    unsigned int    spi_max_clock;

    /* Min clock speed that the SPI block can run at */
    unsigned int    spi_min_clock;

    /* Pointer to hold location of the Tx data 'copy' */
    char            *local_buf;

    /* Length of the Tx data 'copy' */
    unsigned int    byte_count_tx;
};

/*!
 * \brief Function return codes
 */
enum
{
    SUCCESS = 0,    /* Successful outcome */
    FAILURE = 1     /* Error response */
};

/* Types ------------------------------------------------------------------- */

/* Prototypes--------------------------------------------------------------- */
/*!
 * \brief Read a 16 bit value from a register in the SPI block
 *
 * \param master Pointer to the spi_master struct
 * \param register_offset The register to read from
 *
 * \return The 16 bit value read from the hardware
 */
static u16
spi_ioread16(struct spi_master *master,
             unsigned int register_offset);

/*!
 * \brief Write a 16 bit value to a register in the SPI block
 *
 * \param master Pointer to the spi_master struct
 * \param value The value to write
 * \param register_offset The register to write to
 */
static void
spi_iowrite16(struct spi_master *master,
              u16 value,
              unsigned int register_offset);

/*!
 * \brief Setup the SPI block ready for a SPI transfer
 *
 * \param spi Pointer to the spi_device struct
 *
 * \return 0 on success, non zero on error
 */
static int
pc302spi_setup(struct spi_device *spi);

/*!
 * \brief Cleanup things after a transfer
 *
 * \param spi Pointer to the spi_device struct
 */
static void
pc302spi_cleanup(struct spi_device *spi);

/*!
 * \brief Perform an SPI transfer
 *
 * \param spi Pointer to the spi_device struct
 * \param mesg Pointer to the spi_message struct
 *
 * \return 0 on success, non zero on error
 *
 * Note: It is imperative that the Tx fifo never empties during operation,
 *       if this happens the chip select will be negated by the SPI block,
 *       the spi flash will 'forget' what it is doing and it will
 *       all end in tears !
 */
static int
pc302spi_transfer(struct spi_device *spi,
                  struct spi_message *mesg);

/*!
 * \brief Activate the SPI chip select
 *
 * \param spi Pointer to the spi_device struct
 *
 * Note: This does not actually assert a chip select,
 *       it merely controls the muxing in the axi2cfg block to allow
 *       the SPI block to take control of the chip select pins.
 */
static void
pc302spi_cs_activate(struct spi_device *spi);

/*!
 * \brief De-Activate the SPI chip select
 *
 * \param spi Pointer to the spi_device struct
 *
 * Note: This does not actually negate a chip select,
 *       it merely controls the muxing in the axi2cfg block to allow
 *       the EBI block to take control of the chip select pins.
 */
static void
pc302spi_cs_deactivate(struct spi_device *spi);

/*!
 * Platform driver 'probe' method.
 *
 * \param pdev A pointer to the platform_device structure.
 *
 * \return Zero on success, non zero on error.
 */
static int
spi_drv_probe(struct platform_device *pdev);

/*!
 * Platform driver 'remove' method.
 *
 * \param pdev A pointer to the platform_device structure.
 *
 * \return Zero on success, non zero on error.
 */
static int
spi_drv_remove(struct platform_device *pdev);

/* Functions --------------------------------------------------------------- */

static u16
spi_ioread16(struct spi_master *master,
             unsigned int register_offset)
{
    struct pc302_spi *priv = spi_master_get_devdata(master);
    void __iomem *p = priv->regs + register_offset;

    return ioread16(p);
}

static void
spi_iowrite16(struct spi_master *master,
              u16 value,
              unsigned int register_offset)
{
    struct pc302_spi *priv = spi_master_get_devdata(master);
    void __iomem *p = priv->regs + register_offset;

    iowrite16(value, p);
}

static int
pc302spi_setup(struct spi_device *spi)
{
    struct spi_master   *master = spi->master;
    struct pc302_spi *priv = spi_master_get_devdata(master);

    u32 ctrlr0 = 0;
    u32 sckdv = 0;

    /* Have we been passed a valid combination of bus and cs ?
     * Note: PC302 device has a single SPI controller (bus) and
     *       4 possible SPI chip selects
     */
    if (master->bus_num > (s16)(PC302_MAX_NUMBER_SPI_BUSSES - 1))
    {
        /* Oops, request bus is out of range. */
        printk(KERN_ERR "%s: SPI 'bus' (%d) out of range. "
               "(We only have %d SPI bus(ses) available).\n",
               CARDNAME, (unsigned int)master->bus_num,
               (unsigned int)PC302_MAX_NUMBER_SPI_BUSSES);
        return -EINVAL;
    }

    /* Have we been passed a valid chip select ? */
    if (spi->chip_select > (u8)(PC302_MAX_NUMBER_SPI_CS - 1))
    {
        /* Oops, requested chip select is out of range */
        printk(KERN_ERR "%s: SPI 'cs' (%d) out of range. "
               "(We only have %d 'cs' available.\n",
               CARDNAME, (unsigned int)spi->chip_select,
               (unsigned int)PC302_MAX_NUMBER_SPI_CS);
        return -EINVAL;
    }

    /* Have we been passed a valid number of bits per word ? */
    if ((spi->bits_per_word == 0) || (spi->bits_per_word != 8))
    {
        /* Set default bits per word */
        spi->bits_per_word = 8;
    }

    /* Have we been passed a valid SPI bus clock rate ? */
    if ((spi->max_speed_hz < priv->spi_min_clock) ||
        (spi->max_speed_hz > priv->spi_max_clock))
    {
    	/* Oops, we do not support this requested SPI bus clock rate */
        printk(KERN_ERR "%s: SPI bus 'hz' (%d) out of range. "
               "(Min = %d Hz, Max = %d Hz).\n", CARDNAME,
               (unsigned int)spi->max_speed_hz,
               priv->spi_min_clock, priv->spi_max_clock);
        return -EINVAL;
    }

    /* Have we been passed a valid SPI mode ? */
    if ((spi->mode & (SPI_CPHA | SPI_CPOL)) != SPI_MODE_3)
    {
        /* Oops, we only support spi mode 3 */
        printk(KERN_ERR "%s: SPI 'mode' out of range. "
               "(We only support SPI 'mode' %d).\n",
               CARDNAME, (unsigned int)SPI_MODE_3);
        return -EINVAL;
    }

    /* Is the user trying to use active high chip selects ? */
    if ((spi->mode & SPI_CS_HIGH ) == SPI_CS_HIGH)
    {
        /* Oops, we only support active low chip selects */
        printk(KERN_ERR "%s: Only active low chip selects supported\n",
               CARDNAME);
        return -EINVAL;
    }

    /* Is the user trying to send the data LSB first ? */
    if ((spi->mode & SPI_LSB_FIRST ) == SPI_LSB_FIRST)
    {
        /* Oops, we only support data transmission MSB first */
        printk(KERN_ERR "%s: Only MSB first data transmission supported\n",
               CARDNAME);
        return -EINVAL;
    }

    /* Disable SPI operations
     * Note: We can't program up the block registers unless
     *       the block is disabled
     */
    spi_iowrite16(master, PC302_SPI_DISABLE, SSI_ENABLE_REG_REG_OFFSET);

    /* Program up some bits in Control Register 0.
     * Note: As TMOD bits set to 00 (Transmit & Receive),
     *       we do not need to worry about the ctrlr1 register
     *
     * Note: Only 8 bit data supported.
     */
    ctrlr0 = PC302_SPI_DATA_FRM_8_BIT;

    /* Set the clock phase */
    if (spi->mode & SPI_CPHA)
    {
        ctrlr0 |= PC302_SPI_SCPH;
    }

    /* Set the clock polarity */
    if (spi->mode & SPI_CPOL)
    {
        ctrlr0 |= PC302_SPI_SCPOL;
    }

    spi_iowrite16(master, ctrlr0, SSI_CTRL_REG_0_REG_OFFSET);

    /* Setup the SPI bus clock rate */
    sckdv = (PC302_AHB_CLOCK_FREQ / spi->max_speed_hz);
    spi_iowrite16(master, sckdv, SSI_BAUD_RATE_SEL_REG_OFFSET);

    /* Mask all interrupts from the SPI block */
    spi_iowrite16(master, PC302_SPI_MASK_ALL_INTS, SSI_IMR_REG_OFFSET);

    return SUCCESS;
}

static void
pc302spi_cleanup(struct spi_device *spi)
{
    struct spi_master   *master = spi->master;

    /* Disable SPI operations */
    spi_iowrite16(master, PC302_SPI_DISABLE, SSI_ENABLE_REG_REG_OFFSET);
}

static int
pc302spi_transfer(struct spi_device *spi,
                  struct spi_message *mesg)
{
    struct spi_master   *master = spi->master;
    struct pc302_spi *priv = spi_master_get_devdata(master);
    struct spi_transfer *transfer = NULL;

    unsigned int num_lists = 0;
    unsigned int adjust_len = 0;

    int ret = 0;

    char data = 0;

    u16 status = 0;

    u32 sckdv = 0;

    char *tx_buf;
    char *rx_buf;

    unsigned long flags;

    mesg->actual_length = 0;
    mesg->status = 0;

    /* Check for invalid messages and transfers */
    if (list_empty(&mesg->transfers) || !mesg->complete)
    {
        return -ENOMSG;
    }

    /* Count the number list entries */
    list_for_each_entry(transfer, &mesg->transfers, transfer_list)
    {
        num_lists++;
    }

    /* This driver assumes that we will only ever have 2 messages queued
     * up. If we have any more then we can't cope !
     */
    if (num_lists > PC302_MAX_QUEUED_SPI_MESSAGES)
    {
        WARN(1, "%s: Too many SPI messages queued up\n", CARDNAME);
        return -EMSGSIZE;
    }

    /* Disable interrupts */
    local_irq_save(flags);

    list_for_each_entry(transfer, &mesg->transfers, transfer_list)
    {
        unsigned int bits_per_word = spi->bits_per_word;
        unsigned int tx_byte_count = 0;
        unsigned int byte_count = 0;

        if (transfer == NULL)
        {
            ret = -ENOMSG;
            goto out_msg_rejected;
        }

        if ((transfer->tx_buf == NULL) &&
            (transfer->rx_buf == NULL) &&
            transfer->len)
        {
	    printk(KERN_ERR "%s: Message rejected : "
                   "invalid transfer data buffers\n",
                   CARDNAME);

	    ret = -EINVAL;
            goto out_msg_rejected;
        }

        /* Initialise our 'local' copy of the tx and rx buffer pointers */
        rx_buf = (char *)transfer->rx_buf;
        tx_buf = (char *)transfer->tx_buf;

        /* Do we need to adjust the transfer size ? */
        if (transfer->bits_per_word)
        {
            bits_per_word = transfer->bits_per_word;
        }

        if (bits_per_word != 8)
        {
	    printk(KERN_ERR "%s: Message rejected : "
                   "invalid transfer size (%d bits)\n",
                   CARDNAME, bits_per_word);

	    ret = -EINVAL;
            goto out_msg_rejected;
        }

        /* Do we need to adjust the transfer speed ? */
        if (transfer->speed_hz &&
            (transfer->speed_hz < priv->spi_min_clock))
        {
	    printk(KERN_ERR "%s: Message rejected : "
                   "device min speed (%d Hz) exceeds "
		   "required transfer speed (%d Hz)\n",
		   CARDNAME, priv->spi_min_clock, transfer->speed_hz);

	    ret = -EINVAL;
            goto out_msg_rejected;
        }

        /* Do we need to adjust the transfer speed ? */
        if (transfer->speed_hz &&
            (transfer->speed_hz > priv->spi_max_clock))
        {
	    printk(KERN_ERR "%s: Message rejected : "
                   "device max speed (%d Hz) is less than the "
		   "required transfer speed (%d Hz)\n",
		   CARDNAME, priv->spi_max_clock, transfer->speed_hz);

	    ret = -EINVAL;
            goto out_msg_rejected;
        }

        /* Setup the new SPI bus clock rate */
        if (transfer->speed_hz)
        {
            sckdv = (PC302_AHB_CLOCK_FREQ / transfer->speed_hz);
            spi_iowrite16(master, sckdv, SSI_BAUD_RATE_SEL_REG_OFFSET);
        }

        /* Assert the required SPI chip select */
        pc302spi_cs_activate(spi);

        /* Enable SPI block operations */
        spi_iowrite16(master, PC302_SPI_ENABLE, SSI_ENABLE_REG_REG_OFFSET);

        /* Work out what we are being asked to do.
         * This is all a bit horrible, and relies on
         * some knowledge of how this driver will be used !
         *
         * Note: This is necessary because the SPI block only asserts
         *       the chip select when there is data in the tx fifo.
         */

        if ((tx_buf != NULL) && (rx_buf == NULL) &&
            (num_lists > 1)  && (priv->local_buf == NULL))
        {
            /* This is a multi-part transaction starting with a transmit,
             * therefore we take a local copy of the pointer to the
             * data to tx, and the tx data length so we can use it
             * with the next transaction.
             *
             * Remember, we need to be vary careful about the chip select.
             */

            /* Update our private data */
            priv->local_buf = tx_buf;
            priv->byte_count_tx = transfer->len;

            /* We now have a copy of the data so we can null the
             * tx data pointer.
             *
             * Note: This stops this data being transmitted now, and waits
             *       for the next list transaction to be processed.
             */
            tx_buf = NULL;
        }

        adjust_len = priv->byte_count_tx;
        for (byte_count = 0; byte_count < (transfer->len + adjust_len); )
        {
	    if ((tx_buf == NULL) && (rx_buf == NULL))
            {
                /* We can bail out and not do anything here */
                break;
            }

            status = spi_ioread16(master, SSI_STATUS_REG_OFFSET);

            /* Transmit data */
            while (tx_byte_count < priv->byte_count_tx)
            {
                /* The Tx fifo is 16 entries deep, we should never fill
                   this up sending a 'command' to the flash.  Therefore
                   we do not check the tx fifo status */
                spi_iowrite16(master, *priv->local_buf++,
                              SSI_DATA_REG_OFFSET);
                tx_byte_count++;
            }

            while ((tx_byte_count < (transfer->len + adjust_len)) &&
                   (status & PC302_SPI_STATUS_TFNF))
            {
                /* Lets just fill the tx fifo */
                if (tx_buf)
                {
                    spi_iowrite16(master, *tx_buf++,
                                  SSI_DATA_REG_OFFSET);
                }
                else
                {
                    /* Just transmit zeros */
                    spi_iowrite16(master, 0, SSI_DATA_REG_OFFSET);
                }
                tx_byte_count++;

                /* We may fill the tx fifo, so a re-read of
                   the status register is a good idea */
               status = spi_ioread16(master, SSI_STATUS_REG_OFFSET);
            }

            /* Receive Data */
            if (status & PC302_SPI_STATUS_RFNE)
            {
                /* We have some data available in the receive fifo */
                if (rx_buf)
                {
                    /* We have somewhere to put it */
                    data = (char)spi_ioread16(master, SSI_DATA_REG_OFFSET);

                    /* If we transmitted any saved data we need to throw
                     * away priv->byte_count_tx chars from the rx fifo
                     */
                    if (priv->byte_count_tx)
                    {
                        /* Bin the read data */
                        priv->byte_count_tx--;
                    }
                    else
                    {
                        /* Save the read data */
                        *rx_buf++ = data;
                    }
                }
                else
                {
                    /* Just read the fifo and dump the data */
                    (void)spi_ioread16(master, SSI_DATA_REG_OFFSET);
                }

                /* Increment the loop count */
                byte_count++;
	    }
        }

        /* wait until the transfer is completely done before we
         * deactivate CS.
	 */
        do
        {
            status = spi_ioread16(master, SSI_STATUS_REG_OFFSET);
            status &= PC302_SPI_STATUS_BUSY;

	 } while (status);

        mesg->actual_length += transfer->len;
    }

    /* Enable interrupts */
    local_irq_restore(flags);

    pc302spi_cs_deactivate(spi);

    /* Clear up our private data */
    if (priv->local_buf)
    {
        priv->local_buf = NULL;
        priv->byte_count_tx = 0;
    }

    mesg->status = SUCCESS;
    if (mesg->complete)
    {
        mesg->complete(mesg->context);
    }

    return SUCCESS;

out_msg_rejected:
    /* Message rejected and not queued */

    /* Enable interrupts */
    local_irq_restore(flags);

    mesg->status = ret;
    if (mesg->complete)
    {
        mesg->complete(mesg->context);
    }

    return ret;
}

static void
pc302spi_cs_activate(struct spi_device *spi)
{
    struct spi_master   *master = spi->master;

    u32 syscfg_mask = (AXI2CFG_DECODE_MUX_0 |
                       AXI2CFG_DECODE_MUX_1 |
                       AXI2CFG_DECODE_MUX_2 |
                       AXI2CFG_DECODE_MUX_3);

    /* Make sure the SPI block is disabled */
    spi_iowrite16(master, PC302_SPI_DISABLE,
                  SSI_ENABLE_REG_REG_OFFSET);

    /* Write to the Slave Enable Register */
    spi_iowrite16(master, (u16)(1 << spi->chip_select),
                  SSI_SLAVE_ENABLE_REG_OFFSET);

    /* Sort out the SPI/EBI chip select muxing.
     * Note: Set all chip select muxing to be SPI
     */
    syscfg_update(syscfg_mask, 0);
}

static void
pc302spi_cs_deactivate(struct spi_device *spi)
{
    struct spi_master   *master = spi->master;

    u32 syscfg_mask = (AXI2CFG_DECODE_MUX_0 |
                       AXI2CFG_DECODE_MUX_1 |
                       AXI2CFG_DECODE_MUX_2 |
                       AXI2CFG_DECODE_MUX_3);

    /* Make sure the SPI is disabled */
    spi_iowrite16(master, PC302_SPI_DISABLE,
                  SSI_ENABLE_REG_REG_OFFSET);

    /* Write to the Slave Enable Register,
     * Note: Just disable all chip selects for now
     */
    spi_iowrite16(master, PC302_SPI_SLAVES_DISABLE,
                  SSI_SLAVE_ENABLE_REG_OFFSET);

    /* Sort out the SPI/EBI chip select muxing.
     * Note: Set all chip select muxing to be EBI
     */
    syscfg_update(syscfg_mask, syscfg_mask);
}

static int
spi_drv_probe(struct platform_device *pdev)
{
    struct resource     *regs;
    struct spi_master   *master;
    struct pc302_spi    *priv;
    int ret;

    regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!regs)
    {
        /* Oops, we can't obtain any resources */
        printk(KERN_ERR "%s: could not obtain platform resources.\n",
                         CARDNAME);
        ret = -EINVAL;
        goto out;
    }

    if (!request_mem_region(regs->start, (regs->end - regs->start) + 1,
                            CARDNAME))
    {
        /* Oops, we can't obtain the required memory region */
        printk(KERN_ERR "%s: memory mapping error, Address=0x%08x,"
                        " Size=0x%08x\n", CARDNAME, regs->start,
                         (regs->end - regs->start) + 1);
        ret = -ENOMEM;
        goto out;
    }

    master = spi_alloc_master(&pdev->dev, sizeof *priv);
    if (!master)
    {
        /* Oops, something wrong here */
        printk(KERN_ERR "%s: could not allocate spi_master structure.\n",
                         CARDNAME);
        ret = -ENOMEM;
	goto out_alloc_failed;
    }

    master->bus_num = pdev->id;
    master->num_chipselect = 4;
    master->setup = pc302spi_setup;
    master->transfer = pc302spi_transfer;
    master->cleanup = pc302spi_cleanup;
    platform_set_drvdata(pdev, master);

    priv = spi_master_get_devdata(master);

    priv->regs = ioremap(regs->start, (regs->end - regs->start) + 1);
    if (!priv->regs)
    {
        /* Oops, we can't remap io memory */
        printk(KERN_ERR "%s: could not remap io addresses.\n",
                         CARDNAME);
	ret = -ENOMEM;
	goto out_ioremap_failed;
    }

    /* Max clock speed that the SPI block can run at */
    priv->spi_max_clock = (PC302_AHB_CLOCK_FREQ / PC302_MIN_SPI_CLK_DIVIDER);

    /* Min clock speed that the SPI block can run at */
    priv->spi_min_clock = (PC302_AHB_CLOCK_FREQ / PC302_MAX_SPI_CLK_DIVIDER);

    /* Initialise some private variables */
    priv->byte_count_tx = 0;
    priv->local_buf = NULL;

    ret = spi_register_master(master);
    if (ret != 0)
    {
        /* Oops, we can't register as a spi master */
        printk(KERN_ERR "%s: could not register a spi master.\n",
                         CARDNAME);
	goto out_registration_failed;
    }

    return SUCCESS;

out_registration_failed:

out_ioremap_failed:
    iounmap(priv->regs);

out_alloc_failed:
    release_mem_region (regs->start, (regs->end - regs->start) + 1);
    spi_master_put(master);

out:
    printk(KERN_ERR "%s: SPI driver registration failed\n", CARDNAME);
    return ret;
}

static int
spi_drv_remove(struct platform_device *pdev)
{
    struct resource     *regs = platform_get_resource
                                (pdev, IORESOURCE_MEM, 0);
    struct spi_master   *master = platform_get_drvdata(pdev);
    struct pc302_spi    *priv = spi_master_get_devdata(master);

    iounmap(priv->regs);
    release_mem_region(regs->start, (regs->end - regs->start) + 1);
    spi_unregister_master(master);

    return SUCCESS;
}

/*!
 * Platform driver data structure.
 */
static struct
platform_driver spi_driver =
{
    .probe      = spi_drv_probe,
    .remove     = spi_drv_remove,
    .driver     =
    {
        .name   = CARDNAME,
    }
};

static int
spi_init_module(void)
{
    int ret = 0;

    /* Register the platform driver */
    ret = platform_driver_register(&spi_driver);
    if (ret != 0)
    {
        printk(KERN_INFO "%s " CONFIG_LOCALVERSION " failed to loaded\n",
               TITLE);
        return ret;
    }

    printk(KERN_INFO "%s " CONFIG_LOCALVERSION " loaded\n", TITLE);

    return ret;
}

static void
spi_cleanup_module(void)
{
    platform_driver_unregister(&spi_driver);
    printk(KERN_INFO "%s " CONFIG_LOCALVERSION " unloaded\n", TITLE);
}

module_init(spi_init_module);
module_exit(spi_cleanup_module);

MODULE_AUTHOR("picoChip");
MODULE_DESCRIPTION("picoChip PC302 SPI Controller Driver");
MODULE_LICENSE("GPL");
