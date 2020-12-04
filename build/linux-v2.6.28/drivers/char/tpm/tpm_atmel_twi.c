/*
 * Copyright (C) 2009 ip.access Ltd
 *
 * Authors:
 *
 * Device driver for TCG/TCPA TPM (trusted platform module).
 * Specifications at www.trustedcomputinggroup.org	 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2 of the
 * License.
 * 
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include "tpm.h"

/* read status bits */
enum tpm_atmel_twi_read_status {
	TPM_ATMEL_TWI_STATUS_BUSY  = 0x01,
	TPM_ATMEL_TWI_STATUS_READY = 0x02
};

enum tpm_atmel_twi_defaults {
	TPM_ATMEL_TWI_SHORT_TIMEOUT = 750,	/* ms */
	TPM_ATMEL_TWI_LONG_TIMEOUT  = 2000,	/* 2 sec */
};

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x29, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(atmel_twi);

static const struct i2c_device_id atmel_twi_id[] = {
	{ "atmel_twi", atmel_twi },
	{ }
};
MODULE_DEVICE_TABLE(i2c, atmel_twi_id);

/* device specific function to read from the TPM using I2C */
static int tpm_atmel_twi_recv(struct tpm_chip *chip, u8 *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(chip->dev);
	u8 *hdr = buf + 2;
	u32 size;
	int len;
	
	dev_dbg(chip->dev, "recv %d bytes:\n", count);
	
	/* start reading the result header 2(tag)+4(length)+4(operation) */
	if (count < 10) {
		return -EIO;
	}

	len = i2c_master_recv(client, buf, 10);
	if (len < 10) {
		dev_err(chip->dev, "error reading header\n");
		return -EIO;
	}
	
	/* size of the data received */
	size = *hdr++;
	size <<= 8;
	size |= *hdr++;
	size <<= 8;
	size |= *hdr++;
	size <<= 8;
	size |= *hdr;

	dev_dbg(chip->dev, "reply is %d bytes\n", size);
	
	if (count < size) {
		dev_err(chip->dev, "Recv size(%d) less than needed space(%d)\n", count, size);
		return -EIO;
	}

	/* read all the data available, if we didn't get it all anyway. */
	if (len != size)
	{
		len = i2c_master_recv(client, buf, size);
		if (len < size) {
			dev_err(chip->dev, "error reading data (%d)\n", len);
			return -EIO;
		}
	}
	
	return len;
}

/* device specific function to write to the TPM using I2C */
static int tpm_atmel_twi_send(struct tpm_chip *chip, u8 *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(chip->dev);
	int len;

	dev_dbg(chip->dev, "send %d bytes:\n", count);
	
	len = i2c_master_send(client, buf, count);
	
	dev_dbg(chip->dev, "send returned %d\n", len);
	return len;
}

/* device specific function to cancel operations on the TPM */
static void tpm_atmel_twi_cancel(struct tpm_chip *chip)
{
	/* There doesn't seem to be a way to cancel an operation on the
	 * AT97SC3204T.
	 */
	dev_dbg(chip->dev, "cancel\n");
}

/* device specific function to poll for TPM status using I2C */
static u8 tpm_atmel_twi_status(struct tpm_chip *chip)
{
	struct i2c_client *client = to_i2c_client(chip->dev);
	struct i2c_adapter *adapter = client->adapter;
	int ret;
	
	dev_dbg(chip->dev, "status\n");
	
	/* The AT97SC3204T will return a NACK in reponse to its address when
	 * it is busy, so a quick read will give us BUSY/IDLE status.
	 */
	ret = i2c_smbus_xfer(adapter, client->addr, 0, I2C_SMBUS_READ, 0, I2C_SMBUS_QUICK, NULL);
	if (ret < 0) {
		dev_dbg(chip->dev, "busy\n");
		return TPM_ATMEL_TWI_STATUS_BUSY;
	} else {
		dev_dbg(chip->dev, "idle\n");
		return 0;
	}
}

/* Check for the presence of a TPM on I2C.
 * Return 0 if detection is successful, -ENODEV otherwise
 */
static int atmel_twi_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if (kind <= 0) {
		int ret;

		/* Try to start a write to the device.  That should return an ACK
		 * if the device is idle, which is the state a newly reset TPM should
		 * be in.  We stop the write transaction immediately after getting the
		 * ACK/NACK so no data is actually transferred.
		 */
		ret = i2c_smbus_xfer(adapter, client->addr, 0, 0, 0, I2C_SMBUS_QUICK, NULL);
		if (ret < 0)
		{
			dev_dbg(&adapter->dev, "detection failed\n");
			return -ENODEV;
		}
		
	} else
		dev_dbg(&adapter->dev, "detection forced\n");

	strlcpy(info->type, "atmel_twi", I2C_NAME_SIZE);
	
	dev_dbg(&adapter->dev, "detection succeeded\n");

	return 0;
}

/* We use the standard operations provided by the tpm module */
static const struct file_operations atmel_twi_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = tpm_open,
	.read = tpm_read,
	.write = tpm_write,
	.release = tpm_release,
};

/* We provide the standard sysfs files provided by the tpm module
 * for version 1.2 
 */
static DEVICE_ATTR(pubek, S_IRUGO, tpm_show_pubek, NULL);
static DEVICE_ATTR(pcrs, S_IRUGO, tpm_show_pcrs, NULL);
static DEVICE_ATTR(enabled, S_IRUGO, tpm_show_enabled, NULL);
static DEVICE_ATTR(active, S_IRUGO, tpm_show_active, NULL);
static DEVICE_ATTR(owned, S_IRUGO, tpm_show_owned, NULL);
static DEVICE_ATTR(temp_deactivated, S_IRUGO, tpm_show_temp_deactivated,
		   NULL);
static DEVICE_ATTR(caps, S_IRUGO, tpm_show_caps_1_2, NULL);
static DEVICE_ATTR(cancel, S_IWUSR |S_IWGRP, NULL, tpm_store_cancel);

static struct attribute* atmel_twi_attrs[] = {
	&dev_attr_pubek.attr,
	&dev_attr_pcrs.attr,
	&dev_attr_enabled.attr,
	&dev_attr_active.attr,
	&dev_attr_owned.attr,
	&dev_attr_temp_deactivated.attr,
	&dev_attr_caps.attr,
	&dev_attr_cancel.attr,
	NULL,
};

static struct attribute_group atmel_twi_attr_grp = { .attrs = atmel_twi_attrs };

/* Register the vendor specific functions with the TPM module */
static const struct tpm_vendor_specific tpm_atmel_twi = {
	.recv = tpm_atmel_twi_recv,
	.send = tpm_atmel_twi_send,
	.cancel = tpm_atmel_twi_cancel,
	.status = tpm_atmel_twi_status,
	.req_complete_mask = TPM_ATMEL_TWI_STATUS_BUSY,
	.req_complete_val = 0,
	.req_canceled = TPM_ATMEL_TWI_STATUS_READY,
	.attr_group = &atmel_twi_attr_grp,
	.miscdev = { .fops = &atmel_twi_ops, },
};

static int atmel_twi_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct  tpm_chip *chip;
	int     err;

	dev_info(&client->dev, "%s chip found\n", client->name);

	if (!(chip = tpm_register_hardware(&client->dev, &tpm_atmel_twi))) {
		err = -ENODEV;
		goto exit;
	}

	chip->vendor.iobase = 0;
	chip->vendor.base = 0;
	chip->vendor.have_region = 0;
	chip->vendor.region_size = 0;

	/* Use defaults for the timeouts when performing initialisation operations */
	chip->vendor.timeout_a = msecs_to_jiffies(TPM_ATMEL_TWI_SHORT_TIMEOUT);
	chip->vendor.timeout_b = msecs_to_jiffies(TPM_ATMEL_TWI_LONG_TIMEOUT);
	chip->vendor.timeout_c = msecs_to_jiffies(TPM_ATMEL_TWI_SHORT_TIMEOUT);
	chip->vendor.timeout_d = msecs_to_jiffies(TPM_ATMEL_TWI_SHORT_TIMEOUT);
	
	/* On reset, the TPM is effectively initialised by a TPM_Init command.
	 * The TPM is then in a state where it waits for a TPM_Startup command
	 * to inform it which type of initialization is required (see TPM Main
	 * Part 3 Commands, Section 3.1).  Neither the kernel module TPM driver
	 * nor the trousers library normally send this TPM_Startup command,
	 * presumably because the BIOS would normally be repsonsible for doing
	 * it.  On the PC302 this has to be done somewhere, and since we need
	 * to read the chip's timeouts before continuing, we do the TPM_Startup
	 * here to wake the device.
	 */
	tpm_startup(chip);
	tpm_get_timeouts(chip);
	tpm_continue_selftest(chip);
	
	return 0;

exit:
	return err;
}

static int atmel_twi_remove(struct i2c_client *client)
{
	struct tpm_chip *chip = dev_get_drvdata(&client->dev);

	tpm_remove_hardware(chip->dev);
	return 0;
}

static struct i2c_driver atmel_twi_driver = {
	.class		= I2C_CLASS_ALL,
	.driver = {
		.name	= "atmel_twi",
	},
	.probe		= atmel_twi_probe,
	.remove		= atmel_twi_remove,
	.id_table	= atmel_twi_id,
	.detect		= atmel_twi_detect,
	.address_data	= &addr_data,
};

static int __init atmel_twi_init(void)
{
	return i2c_add_driver(&atmel_twi_driver);
}

static void __exit atmel_twi_exit(void)
{
	i2c_del_driver(&atmel_twi_driver);
}

module_init(atmel_twi_init);
module_exit(atmel_twi_exit);

MODULE_AUTHOR("ip.access");
MODULE_DESCRIPTION("TPM Driver for Atmel AT97SC3204T");
MODULE_VERSION("2.0");
MODULE_LICENSE("GPL");
