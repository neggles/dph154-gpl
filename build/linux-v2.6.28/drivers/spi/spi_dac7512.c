/*
* dac7512.c -- support DAC7512 12-bit Digital to Analogue Converter
*
* Copyright (C) 2009 ip.access Ltd
*
* Based on at25.c
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>

#include <linux/ipa/dac7512.h>

/* Writes to the DAC are always 16-bits */
#define  DAC7512_TRANSFER_SIZE            (2)

struct dac7512_data {
	struct miscdevice      miscdev;
	struct spi_device     *spi;                         /* SPI master we're using */
	struct mutex           lock;                        /* Required for read-modify-write of config reg */
	u8                     buf[DAC7512_TRANSFER_SIZE];  /* DMA safe buffer for SPI transfers */
	int                    lastValue;
};

static struct dac7512_data  *dac7512_device = NULL;

/* Writing to the DAC7512 requires a 16-bit command containing the output value */
static int dac7512_writeRegister(struct dac7512_data *dac7512, u16 val)
{
	struct spi_transfer t = {
	        .tx_buf     = dac7512->buf,
	        .len        = DAC7512_TRANSFER_SIZE,
	    };
	struct spi_message  m;
	int ret = 0;
	
	dac7512->buf[0] = (val >> 8) & 0xFF;
	dac7512->buf[1] = val & 0xFF;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	mutex_lock(&dac7512->lock);
	
	ret = spi_sync(dac7512->spi, &m);
	
	if (ret == 0)
	{
		dac7512->lastValue = val;
	}
	
	mutex_unlock(&dac7512->lock);
	
	return ret;
}

/* Can't read from the device, but we can return the last value written. */
static ssize_t dac7512_show_dac( struct device *dev, struct device_attribute *attr, char *buf )
{
	struct dac7512_data  *dac7512 = NULL;
	int                   written = 0;
	
	dac7512 = dev_get_drvdata(dev);
	
	if (dac7512)
	{
		mutex_lock(&dac7512->lock);
		
		written = scnprintf( buf, PAGE_SIZE, "lastValue=%d\n", dac7512->lastValue );
		
		mutex_unlock(&dac7512->lock);
	}
	
	return written;
}

static ssize_t dac7512_store_dac( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	u16  dacValue = 0;
	int  valid    = 0;
	int  remain   = count;
	u16  dacLimit = 0xFFF;
	
	/* Skip initial space. */
	while (remain && isspace(*buf))
	{
		buf++;
		remain--;
	}
	
	/* Read digits. */
	while (remain && isdigit(*buf))
	{
		if (dacValue < 0x1000)
		{
			dacValue *= 10;
			dacValue += *buf - '0';
			
			/* Saw at least one digit */
			valid = 1;
		}
		else
		{
			/* Saw too many digits, result will be too big. */
			valid = 0;
		}
		
		buf++;
		remain--;
	}
	
	/* Check that trailing characters are all space. */
	while (remain && isspace(*buf))
	{
		buf++;
		remain--;
	}
	
	if (valid && (remain == 0) && (dacValue <= dacLimit))
	{
		struct dac7512_data  *dac7512 = dev_get_drvdata(dev);
	
		if (dac7512)
		{
			int  ret = dac7512_writeRegister(dac7512, dacValue);
			
			if (ret < 0)
			{
				printk("dac7512: write failed %d\n", ret);
				return -EIO;
			}
		}
	}
	else
	{
		printk("dac7512: invalid value\n");
		return -EINVAL;
	}
	
	return count;
}

static DEVICE_ATTR( dac7512_dac, S_IRUGO | S_IWUSR, dac7512_show_dac, dac7512_store_dac );



/* Baud rate */
static ssize_t dac7512_show_baud( struct device *dev, struct device_attribute *attr, char *buf )
{
        struct dac7512_data  *dac7512 = NULL;

        dac7512 = dev_get_drvdata(dev);

        if (dac7512)
        {
                return scnprintf( buf, PAGE_SIZE, "%d\n", dac7512->spi->max_speed_hz );
        }
        else
        {
                return scnprintf( buf, PAGE_SIZE, "???\n" );
        }
}

static ssize_t dac7512_store_baud( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
        struct dac7512_data  *dac7512 = NULL;
        int                   speed;

        if ((1 != sscanf(buf, "%d", &speed)) || (speed > 100000000) || (speed < 3052))
        {
                return -EINVAL;
        }

        dac7512 = dev_get_drvdata(dev);
        if (!dac7512)
        {
                return -EIO;
        }

        dac7512->spi->max_speed_hz = speed;

        spi_setup(dac7512->spi);

        return count;
}

static DEVICE_ATTR( dac7512_baud, S_IRUGO | S_IWUSR, dac7512_show_baud, dac7512_store_baud );



/* SPI mode */
static ssize_t dac7512_show_mode( struct device *dev, struct device_attribute *attr, char *buf )
{
        struct dac7512_data  *dac7512 = NULL;

        dac7512 = dev_get_drvdata(dev);

        if (dac7512)
        {
                return scnprintf( buf, PAGE_SIZE, "%d\n", dac7512->spi->mode & 3 );
        }
        else
        {
                return scnprintf( buf, PAGE_SIZE, "???\n" );
        }
}

static ssize_t dac7512_store_mode( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
        struct dac7512_data  *dac7512 = NULL;
        int                   mode;

        if ((1 != sscanf(buf, "%d", &mode)) || (mode > 3) || (mode < 0))
        {
                return -EINVAL;
        }

        dac7512 = dev_get_drvdata(dev);
        if (!dac7512)
        {
                return -EIO;
        }

        dac7512->spi->mode &= ~3;
        dac7512->spi->mode |= mode;

        spi_setup(dac7512->spi);

        return count;
}

static DEVICE_ATTR( dac7512_mode, S_IRUGO | S_IWUSR, dac7512_show_mode, dac7512_store_mode );




static int dac7512_open(struct inode *inode, struct file *filp)
{
	int                   result = 0;
	
	filp->private_data  = dac7512_device;
	
	return result;
}

static int dac7512_release(struct inode *inode, struct file *filp)
{
	if (filp->private_data)
	{
		filp->private_data = 0;
	}
	
	return 0;
}

static int dac7512_ioctl(struct inode   *inode_p,
                struct file    *filp,
                unsigned int   cmd,
                unsigned long  arg)
{
	struct dac7512_data  *dac7512 = NULL;
	int                   ret     = 0;

	/* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
	if (_IOC_TYPE(cmd) != DAC7512_IOCTL_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd)    > DAC7512_IOCTL_MAXNR) return -ENOTTY;
	
	dac7512 = filp->private_data;
	
	switch (cmd)
	{
	case DAC7512_IOCTL_WRITE_VALUE:
		ret = dac7512_writeRegister(dac7512, arg);
		break;
		
		
	default:  /* redundant, as cmd was checked against MAXNR */
		ret = -ENOTTY;
	}
	
	return ret;
}

static struct file_operations dac7512_fops = {
	.owner      = THIS_MODULE,
	.open       = dac7512_open,
	.release    = dac7512_release,
	.ioctl      = dac7512_ioctl,
};


static int dac7512_probe(struct spi_device *spi)
{
	struct dac7512_data  *dac7512 = NULL;
	int                   err;
	
	if (dac7512_device != NULL)
	{
		err = -EBUSY;
		goto err_no_dac7512_data;
	}
	
	if (!(dac7512 = kzalloc(sizeof *dac7512, GFP_KERNEL))) {
		err = -ENOMEM;
		goto err_no_dac7512_data;
	}

	dac7512->lastValue = -1;
	
	dac7512->miscdev.fops   = &dac7512_fops,
	dac7512->miscdev.name   = "dac7512",
	dac7512->miscdev.minor  =  DAC7512_MINOR,  /* Our /dev is read only, so use a fixed value */
	
	err = misc_register( &dac7512->miscdev );
	if ( err )
	{
		printk( KERN_INFO "failed to register dac7512 misc device\n" );
		goto err_no_misc_device;
	}
	
	mutex_init(&dac7512->lock);
	dac7512->spi = spi_dev_get(spi);
	dev_set_drvdata(&spi->dev, dac7512);
	
	err = device_create_file(&spi->dev, &dev_attr_dac7512_dac);
	if (err)
	{
		goto err_no_sysfs_dac;
	}
	
	err = device_create_file(&spi->dev, &dev_attr_dac7512_baud);
	if (err)
	{
		goto err_no_sysfs_baud;
	}
	
	err = device_create_file(&spi->dev, &dev_attr_dac7512_mode);
	if (err)
	{
		goto err_no_sysfs_mode;
	}
	
	dac7512_device = dac7512;
	
	return 0;
	

err_no_sysfs_mode:
	device_remove_file( &spi->dev, &dev_attr_dac7512_baud);
err_no_sysfs_baud:
	device_remove_file( &spi->dev, &dev_attr_dac7512_dac);
err_no_sysfs_dac:
	misc_deregister( &dac7512->miscdev );
err_no_misc_device:
	kfree(dac7512);
err_no_dac7512_data:
	
	return err;
}

static int __devexit dac7512_remove(struct spi_device *spi)
{
	struct dac7512_data	*dac7512;
	
	dac7512 = dev_get_drvdata(&spi->dev);
	device_remove_file( &spi->dev, &dev_attr_dac7512_mode);
	device_remove_file( &spi->dev, &dev_attr_dac7512_baud);
	device_remove_file( &spi->dev, &dev_attr_dac7512_dac);
	misc_deregister( &dac7512->miscdev );
	kfree(dac7512);
	dac7512_device = NULL;
	
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct spi_driver dac7512_driver = {
	.driver = {
		.name		= "dac7512",
		.owner		= THIS_MODULE,
	},
	.probe		= dac7512_probe,
	.remove		= __devexit_p(dac7512_remove),
};

static int __init dac7512_init(void)
{
	return spi_register_driver(&dac7512_driver);
}
module_init(dac7512_init);

static void __exit dac7512_exit(void)
{
	spi_unregister_driver(&dac7512_driver);
}
module_exit(dac7512_exit);

MODULE_DESCRIPTION("Driver for DAC7512");
MODULE_AUTHOR("ip.access Ltd");
MODULE_LICENSE("GPL");

