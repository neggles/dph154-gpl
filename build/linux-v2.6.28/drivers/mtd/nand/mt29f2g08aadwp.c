/* Copyright (c) 2009 picoChip Designs Ltd.
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

/*
 * mt29f2g08aadwp.c - Micron MT29F2G08AADWP NAND flash MTD driver FOR PC302
 *
 * Authors: Eric Le Bihan <eric.lebihan@sagem.com>
 *
 * Copyright (c) 2009 SAGEM
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <asm/io.h>

#include <mach/pc302/pc302.h>

/* Macros ------------------------------------------------------------------ */
#define NAND_BASE_ADDRESS EBI_CS2_BASE
#define NAND_SIZE         (2 * SZ_1K)

/* Define which gpio bits are used to control the NAND Flash on the
 * PC7302 PLatform.
 *
 * Note: These pin definitions mean that we can only use NAND
 *       Flash if we are running from RAM and have NOT booted
 *       the device from parallel NOR Flash.
 *
 * Note: These GPIO bits are all ARM GPIO bits.
 */
#define CLE     4
#define ALE     3
#define NCE     2
#define READY   1

/* Constants --------------------------------------------------------------- */
static struct mtd_info *mt29f2g08aadwp_mtd;
static void __iomem *base_address;
static unsigned int  scan_done = 0;

/*
 * Define partitions for flash devices
 */
static struct mtd_partition partition_info[] = {
	{ .name		= "Boot",
	  .offset	= 0,
	  .size		= SZ_128K
        },
        { .name		= "Redundant Boot",
	  .offset	= SZ_128K,
	  .size		= SZ_128K
        },
        { .name		= "Boot Environment",
	  .offset	= SZ_128K * 4,
	  .size		= SZ_128K
        },
        { .name		= "Redundant Boot Environment",
	  .offset	= SZ_128K * 5,
	  .size		= SZ_128K
        },
        { .name		= "Kernel",
	  .offset	= SZ_1M,
	  .size		= 2 * SZ_1M
        },
        { .name		= "File System",
	  .offset	= MTDPART_OFS_APPEND,
	  .size		= 64 * SZ_1M
        },
        { .name		= "Data",
	  .offset	= MTDPART_OFS_APPEND,
	  .size		= MTDPART_SIZ_FULL
        },
};

#define NUM_PARTITIONS 7

#define MSG_PFX "mt29f2g08aadwp: "

/* Prototypes--------------------------------------------------------------- */

/*!
 * \brief Hardware specific access to control-lines of the NAND Flash.
 *
 * \param mtd, pointer to the mtd_info structure
 * \param dat, data to write to the device
 * \param ctrl, control data to set up the transaction
 *
 */
static void mt29f2g08aadwp_cmd_ctrl(struct mtd_info *mtd,
				    int dat,
				    unsigned int ctrl);

/*!
 * \brief Return the state of the NAND busy output.
 *
 * \param mtd, pointer to the mtd_info structure
 * \return 0 - nand busy
 *         1 - nand ready
 *
 */
static int mt29f2g08aadwp_dev_ready(struct mtd_info *mtd);

/*!
 * \brief Module initialisation function.
 *
 * \return Zero on success, non zero on error.
 */
static int __init mt29f2g08aadwp_nand_init(void);

/*!
 * \brief Module exit function.
 */
static void mt29f2g08aadwp_nand_exit(void);

/* Functions --------------------------------------------------------------- */

static void mt29f2g08aadwp_cmd_ctrl(struct mtd_info *mtd,
				    int dat,
				    unsigned int ctrl)
{

	register struct nand_chip *this = mtd->priv;
	int err = -1;

	if (ctrl & NAND_CTRL_CHANGE) {

		if (ctrl & NAND_NCE) {

			err = gpio_set_value(NCE, 0);

			if (err)
				printk(KERN_WARNING MSG_PFX
				       "GPIO for nCE can not be set to 0 "
                                       "(%d)\n",
				       err);

			if (ctrl & NAND_CLE)
				err = gpio_set_value(CLE, 1);
			else
				err = gpio_set_value(CLE, 0);

			if (err)
				printk(KERN_WARNING MSG_PFX
				       "GPIO for CLE can not be set (%d)\n",
				       err);

			if (ctrl & NAND_ALE)
				err = gpio_set_value(ALE, 1);
			else
				err = gpio_set_value(ALE, 0);

			if (err)
				printk(KERN_WARNING MSG_PFX
				       "GPIO for ALE can not be set (%d)\n",
				       err);

		} else {
			err = gpio_set_value(NCE, 1);

			if (err)
				printk(KERN_WARNING MSG_PFX
				       "GPIO for nCE can not be set to 1 "
                                       "(%d)\n",
				       err);
		}
	}

	if (dat != NAND_CMD_NONE)
		writeb(dat, this->IO_ADDR_W);
}

static int mt29f2g08aadwp_dev_ready(struct mtd_info *mtd)
{
	return gpio_get_value(READY)? 1 : 0;
}

static int __init mt29f2g08aadwp_nand_init(void)
{
	struct nand_chip *this;
	int err = 0;

	/* Set GPIOs */
	err = gpio_request(READY, "NAND R/B");
	if (err)
		return -EIO;

	err = gpio_request(NCE, "NAND nCE");
	if (err)
		return -EIO;

	err = gpio_request(ALE, "NAND ALE");
	if (err)
		return -EIO;

	err = gpio_request(CLE, "NAND CLE");
	if (err)
		return -EIO;

	err = gpio_direction_input(READY);
	if (err)
		return -EIO;

	err = gpio_direction_output(NCE, 1);
	if (err)
		return -EIO;

	err = gpio_direction_output(ALE, 0);
	if (err)
		return -EIO;

	err = gpio_direction_output(CLE, 0);
	if (err)
		return -EIO;

	/* Allocate memory for MTD device structure and private data */
	mt29f2g08aadwp_mtd = kmalloc(sizeof(struct mtd_info) +
				     sizeof(struct nand_chip), GFP_KERNEL);
	if (!mt29f2g08aadwp_mtd) {
		printk (KERN_DEBUG  MSG_PFX
			"Unable to allocate NAND MTD device structure.\n");
		err = -ENOMEM;
		goto out_err;
	}

	/* Initialize structures */
	memset((char *) mt29f2g08aadwp_mtd,
	       0,
	       sizeof(struct mtd_info) + sizeof(struct nand_chip));

	/* Get pointer to private data */
	this = (struct nand_chip *) (&mt29f2g08aadwp_mtd[1]);
	/* Link the private data with the MTD structure */
	mt29f2g08aadwp_mtd->priv  = this;
	mt29f2g08aadwp_mtd->owner = THIS_MODULE;

	this->ecc.mode = NAND_ECC_SOFT;

	base_address = NULL;

	if (request_mem_region(NAND_BASE_ADDRESS, NAND_SIZE,
			       "MT29F2G08AADWP NAND Memory")) {

		base_address = ioremap(NAND_BASE_ADDRESS, NAND_SIZE);

		if (!base_address) {
			err = -EIO;
			printk(KERN_DEBUG MSG_PFX "can not remap nand "
                               "region!\n");
		}

	} else {
		err = -EBUSY;
		printk(KERN_DEBUG MSG_PFX "can not allocate nand IO "
                       "resource!\n");
	}

	if (err < 0)
		goto out_err;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = base_address;
	this->IO_ADDR_W = base_address;

	this->cmd_ctrl	= mt29f2g08aadwp_cmd_ctrl;
	this->dev_ready = mt29f2g08aadwp_dev_ready;

	/* Scan to find existance of the device */
	if (nand_scan(mt29f2g08aadwp_mtd, 1)) {
		err = -ENXIO;
		goto out_err;
	}

	scan_done = 1;

	add_mtd_device((struct mtd_info *) mt29f2g08aadwp_mtd);
	add_mtd_partitions(mt29f2g08aadwp_mtd,
			   partition_info,
			   NUM_PARTITIONS);
	goto out;

out_err:
	mt29f2g08aadwp_nand_exit();
out:
	return err;
}

static void mt29f2g08aadwp_nand_exit(void)
{
	if (scan_done) {
		nand_release(mt29f2g08aadwp_mtd);
		scan_done = 0;
	}

	gpio_free(READY);
	gpio_free(NCE);
	gpio_free(ALE);
	gpio_free(CLE);

	if (base_address) {
		iounmap(base_address);
		release_mem_region(NAND_BASE_ADDRESS, NAND_SIZE);
		base_address = NULL;
	}

	kfree(mt29f2g08aadwp_mtd);
}

module_init(mt29f2g08aadwp_nand_init);
module_exit(mt29f2g08aadwp_nand_exit);

MODULE_AUTHOR("Eric Le Bihan");
MODULE_DESCRIPTION("MT29F2G08AADWP MTD NAND driver");
MODULE_LICENSE("GPL");
