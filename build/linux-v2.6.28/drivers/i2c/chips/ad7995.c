/*
    ad7995.c - driver for AD7995

    Copyright (C) 2009 ip.access Ltd

    Based on i2c/chips/max6875.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/ctype.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/ipa/ad7995.h>

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x28, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(ad7995);

static const struct i2c_device_id ad7995_id[] = {
	{ "ad7995", ad7995 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad7995_id);

struct ad7995_data {
	struct i2c_client     *client;
	struct miscdevice      miscdev;
	struct mutex           lock;                       /* Required for read-modify-write of config reg */
	u8                     config;
	u8                     buf[8];
};


static struct ad7995_data  *ad7995_device = NULL;

/* Conversion register.  Read only. */
static ssize_t ad7995_show_sample( struct device            *dev, 
                                   struct device_attribute  *attr,
                                   char                     *buf )
{
	struct ad7995_data  *ad7995 = NULL;
	int                  written = 0;
	int                  len;
	int                  i;
	
	ad7995 = dev_get_drvdata(dev);
	
	if (ad7995)
	{
		int  numChannels = 0;
		
		mutex_lock(&ad7995->lock);

		/* Try to read up to 4 samples (8 bytes) in the case where multiple
		 * channels are enabled.
		 */
		if (ad7995->config & 0x80)
		{
			numChannels++;
		}
		if (ad7995->config & 0x40)
		{
			numChannels++;
		}
		if (ad7995->config & 0x20)
		{
			numChannels++;
		}
		if (ad7995->config & 0x10)
		{
			numChannels++;
		}
		
		len = i2c_master_recv(ad7995->client, ad7995->buf, 2*numChannels);
		
		if (len < 0)
		{
			mutex_unlock(&ad7995->lock);
			written = scnprintf( buf, PAGE_SIZE, "Failed to convert (%d)\n", len);
		}
		else
		{
			int  sample[4] = {-1, -1, -1, -1};
			
			for (i=0; i<len-1; i+=2)
			{
				int value;
				int channel = (ad7995->buf[i] >> 4) & 3;
				value = ad7995->buf[i] & 0x0F;
				value <<= 8;
				value |= ad7995->buf[i+1];
				sample[channel] = value;
			}
			
			mutex_unlock(&ad7995->lock);
			
			written = scnprintf( buf, PAGE_SIZE, "adc[0]=%d, adc[1]=%d, adc[2]=%d, adc[3]=%d\n",
			                                      sample[0], sample[1], sample[2], sample[3]);
		}
	}
	
	return written;
}

static ssize_t ad7995_store_sample( struct device            *dev, 
                                    struct device_attribute  *attr,
                                    const char               *buf,
                                    size_t                    count )
{
	return -EIO;
}

static DEVICE_ATTR( ad7995_sample, S_IRUGO | S_IWUSR, ad7995_show_sample, ad7995_store_sample );



/* Configuration register */
static ssize_t ad7995_show_config( struct device            *dev, 
                                   struct device_attribute  *attr, 
                                   char                     *buf )
{
	struct ad7995_data  *ad7995 = NULL;
	int                  written = 0;
	
	ad7995 = dev_get_drvdata(dev);
	
	if (ad7995)
	{
		u8  config = ad7995->config;
		
		written = scnprintf( buf, PAGE_SIZE, "config=%c%c%c%c%c%c%c%c\n",
		                                      (config & 0x80) ? '3' : '-',
		                                      (config & 0x40) ? '2' : '-',
		                                      (config & 0x20) ? '1' : '-',
		                                      (config & 0x10) ? '0' : '-',
		                                      (config & 0x08) ? 'R' : 'r',
		                                      (config & 0x04) ? 'F' : 'f',
		                                      (config & 0x02) ? 'B' : 'b',
		                                      (config & 0x01) ? 'S' : 's');
	}
	
	return written;
}

static ssize_t ad7995_store_config( struct device            *dev, 
                                    struct device_attribute  *attr, 
                                    const char               *buf,
                                    size_t                    count)
{
    struct ad7995_data  *ad7995 = NULL;
    u8                   value  = 0;
    int                  i;
	
	for (i=0; i<count; ++i)
	{
		char c = buf[i];
				
		if (c == 0)
		{
			break;
		}
	
		/* Accept lower case to clear a flag and upper case to set it */        
		switch (c)
		{
		case '3':
			value |= 0x80;
			break;
		case '2':
			value |= 0x40;
			break;
		case '1':
			value |= 0x20;
			break;
		case '0':
			value |= 0x10;
			break;
		case 'R':
			value |= 0x08;
			break;
		case 'F':
			value |= 0x04;
			break;
		case 'B':
			value |= 0x02;
			break;
		case 'S':
			value |= 0x01;
			break;
		case 'r':
			value &= ~0x08;
			break;
		case 'f':
			value &= ~0x04;
			break;
		case 'b':
			value &= ~0x02;
			break;
		case 's':
			value &= ~0x01;
			break;
		}
	}
	
	ad7995 = dev_get_drvdata(dev);
	
	if (ad7995)
	{
		mutex_lock(&ad7995->lock);
	
		ad7995->config = value;
		i2c_smbus_write_byte(ad7995->client, ad7995->config);
		
		mutex_unlock(&ad7995->lock);
	}
	
	return count;
}

static DEVICE_ATTR( ad7995_config, S_IRUGO | S_IWUSR, ad7995_show_config, ad7995_store_config );


static int ad7995_open(struct inode  *inode,
                       struct file   *filp)
{
	int                   result = 0;
	
	filp->private_data  = ad7995_device;
	
	return result;
}

static int ad7995_release(struct inode  *inode,
                          struct file   *filp)
{
	if (filp->private_data)
	{
		filp->private_data = 0;
	}
	
	return 0;
}

static int ad7995_ioctl(struct inode   *inode_p,
                        struct file    *filp,
                        unsigned int    cmd,
                        unsigned long   arg)
{
	struct ad7995_data  *ad7995 = NULL;
	int                   ret     = 0;
	
	/* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
	if (_IOC_TYPE(cmd) != AD7995_IOCTL_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd)    > AD7995_IOCTL_MAXNR) return -ENOTTY;
	
	ad7995 = filp->private_data;
	
	switch (cmd)
	{
	case AD7995_IOCTL_READ_ADC:
		{
			int            numChannels = 0;
			Ad7995Samples  samples;
			int            len;
			int            i;
			
			samples.adc[0] = -1;
			samples.adc[1] = -1;
			samples.adc[2] = -1;
			samples.adc[3] = -1;
			
			mutex_lock(&ad7995->lock);
	
			if (ad7995->config & 0x80)
			{
				numChannels++;
			}
			if (ad7995->config & 0x40)
			{
				numChannels++;
			}
			if (ad7995->config & 0x20)
			{
				numChannels++;
			}
			if (ad7995->config & 0x10)
			{
				numChannels++;
			}
			
			/* Try to read up to 4 samples (8 bytes) in the case where multiple
			 * channels are enabled.
			 */
			len = i2c_master_recv(ad7995->client, ad7995->buf, 2*numChannels);
			
			if (len < 0)
			{
				mutex_unlock(&ad7995->lock);
				
				ret = -EIO;
			}
			else
			{
				for (i=0; i<len-1; i+=2)
				{
					int  value;
					int  channel;
					
					channel = (ad7995->buf[i] >> 4) & 3;
					
					value = ad7995->buf[i] & 0x0F;
					value <<= 8;
					value |= ad7995->buf[i+1];
					
					samples.adc[channel] = value;
				}
				
				mutex_unlock(&ad7995->lock);
			
				if (copy_to_user((void*)arg, &samples, sizeof(samples)))
				{
					ret = -EFAULT;
				}
			}
		}
		break;
	
	case AD7995_IOCTL_WRITE_CONFIG:
		{
			Ad7995Config  config;
			
			if (copy_from_user(&config, (void*)arg, sizeof(config)))
			{
				ret = -EFAULT;
				break;
			}
		
			mutex_lock(&ad7995->lock);
			
			ad7995->config = config.flags;
			ret = i2c_smbus_write_byte(ad7995->client, ad7995->config);
			
			mutex_unlock(&ad7995->lock);
		}
		break;
		
	case AD7995_IOCTL_READ_CONFIG:
		{
			Ad7995Config  config;
			
			config.flags = ad7995->config;
			
			if (copy_to_user((void*)arg, &config, sizeof(config)))
			{
				ret = -EFAULT;
			}
		}
		break;
		
	default:  /* redundant, as cmd was checked against MAXNR */
		ret = -ENOTTY;
	}
	
	return ret;
}

static struct file_operations ad7995_fops = {
	.owner      = THIS_MODULE,
	.open       = ad7995_open,
	.release    = ad7995_release,
	.ioctl      = ad7995_ioctl,
};


/* Return 0 if detection is successful, -ENODEV otherwise */
static int ad7995_detect(struct i2c_client      *client,
                         int                     kind,
                         struct i2c_board_info  *info)
{
	struct i2c_adapter *adapter = client->adapter;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
						| I2C_FUNC_SMBUS_READ_BYTE))
		return -ENODEV;
	
	strlcpy(info->type, "ad7995", I2C_NAME_SIZE);
	
	return 0;
}

static int ad7995_probe(struct i2c_client           *client,
                        const struct i2c_device_id  *id)
{
	struct device       *dev      = &client->dev;
	struct ad7995_data  *ad7995  = NULL;
	int                  err;
	
	ad7995 = kzalloc(sizeof(struct ad7995_data), GFP_KERNEL);
	if (ad7995 == NULL)
	{
		err = -ENOMEM;
		goto err_no_mem;
	}
	
	ad7995->client = client;
	
	ad7995->miscdev.fops   = &ad7995_fops;
	ad7995->miscdev.name   = "ad7995";
	ad7995->miscdev.minor  =  AD7995_MINOR;  /* Our /dev is read only, so use a fixed value */
	
	err = misc_register( &ad7995->miscdev );
	if ( err )
	{
		printk( KERN_INFO "failed to register ad7995 misc device\n" );
		goto err_no_misc_device;
	}
	
	/* Init i2c_client */
	i2c_set_clientdata(client, ad7995);
	mutex_init(&ad7995->lock);

	/* Default to a single channel */	
	ad7995->config = 0x10;
	i2c_smbus_write_byte(client, ad7995->config);
	
	err = device_create_file(dev, &dev_attr_ad7995_config);
	if (err)
	{
		goto err_no_sysfs_config;
	}
	
	err = device_create_file(dev, &dev_attr_ad7995_sample);
	if (err)
	{
		goto err_no_sysfs_sample;
	}
	
	ad7995_device = ad7995;
	
	return 0;

err_no_sysfs_sample:
	device_remove_file( dev, &dev_attr_ad7995_config);
err_no_sysfs_config:
	misc_deregister( &ad7995->miscdev );
err_no_misc_device:
	kfree(ad7995);
err_no_mem:
	return err;
}

static int ad7995_remove(struct i2c_client *client)
{
	struct ad7995_data  *ad7995 = i2c_get_clientdata(client);
	struct device       *dev    = &client->dev;
	
	ad7995_device = NULL;
	device_remove_file( dev, &dev_attr_ad7995_sample);
	device_remove_file( dev, &dev_attr_ad7995_config);
	misc_deregister( &ad7995->miscdev );
	kfree(ad7995);
	
	return 0;
}

static struct i2c_driver ad7995_driver = {
	.class		= I2C_CLASS_ALL,
	.driver = {
		.name	= "ad7995",
	},
	.probe		= ad7995_probe,
	.remove		= ad7995_remove,
	.id_table	= ad7995_id,
	.detect		= ad7995_detect,
	.address_data	= &addr_data,
};

static int __init ad7995_init(void)
{
	return i2c_add_driver(&ad7995_driver);
}

static void __exit ad7995_exit(void)
{
	i2c_del_driver(&ad7995_driver);
}


MODULE_DESCRIPTION("Driver for AD7995 ADC");
MODULE_AUTHOR("ip.access Ltd");
MODULE_LICENSE("GPL");

module_init(ad7995_init);
module_exit(ad7995_exit);
