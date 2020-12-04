/*
    max6635.c - driver for MAX6635

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

#include <linux/ipa/max6635.h>

#define  MAX6635_TRANSFER_SIZE               (8)

#define  MAX6635_LIMIT_REG_MIN               (-256)
#define  MAX6635_LIMIT_REG_MAX               (255)

#define  MAX6635_TEMP_REG                    (0x00)
#define  MAX6635_CONFIG_REG                  (0x01)
#define  MAX6635_THYST_REG                   (0x02)
#define  MAX6635_TMAX_REG                    (0x03)
#define  MAX6635_TLOW_REG                    (0x04)
#define  MAX6635_THIGH_REG                   (0x05)

#define  MAX6635_CONFIG_SHUTDOWN_MODE        (1 << 0)
#define  MAX6635_CONFIG_INTERRUPT_MODE       (1 << 1)
#define  MAX6635_CONFIG_OVER_TEMP_POLARITY   (1 << 2)
#define  MAX6635_CONFIG_ALERT_POLARITY       (1 << 3)
#define  MAX6635_CONFIG_FAULT_QUEUE          (1 << 4)
#define  MAX6635_CONFIG_SMB_TIMEOUT_DISABLE  (1 << 5)

#define  MAX6635_STATUS_TLOW                 (1 <<  0)
#define  MAX6635_STATUS_THIGH                (1 <<  1)
#define  MAX6635_STATUS_TOVER                (1 <<  2)

/* Do not scan - the MAX6875 access method will write to some EEPROM chips */
static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(max6635);


struct max6635_data {
    struct i2c_client     *client;
    struct miscdevice      miscdev;
    struct mutex           lock;                        /* Required for read-modify-write of config reg */
    u8                     buf[MAX6635_TRANSFER_SIZE];  /* DMA safe buffer for SPI transfers */
};


static struct max6635_data  *max6635_device = NULL;



/* Temperature register.  Read only. */
static ssize_t max6635_show_temp( struct device *dev, struct device_attribute *attr, char *buf )
{
    struct max6635_data  *max6635 = NULL;
    s32                   value;
    int                   written = 0;
    
    max6635 = dev_get_drvdata(dev);
    
    if (max6635)
    {
        mutex_lock(&max6635->lock);
        
        value = i2c_smbus_read_word_data(max6635->client, MAX6635_TEMP_REG);
        
        mutex_unlock(&max6635->lock);
        
        if (value < 0)
        {
            written = scnprintf( buf, PAGE_SIZE, "read failure %d\n", value );
        }
        else
        {
            int units     = (value >> 3) & 0x1FFF;
            int fract     = 0;
            int negative  = 0;
            
            /* Convert -ve to +ve and hold a sign flag */
            if (units & 0x1000)
            {
                units = 0x2000 - units;
                negative  = 1;
            }
    
            /* Each code corresponds to 0.0625 degrees C */
            units *= 625;
            
            /* Round from 4 decimal places to 2 decimal places */
            units += 50;
            units /= 100;
    
            /* Split whole and fractional parts */
            fract = units % 100;
            units /= 100;
            
            written = scnprintf( buf, PAGE_SIZE, "%s%d.%02d Tover=%d Thigh=%d Tlow=%d\n",
                                 negative ? "-" : "", units, fract,
                                 !!(value & MAX6635_STATUS_TOVER),
                                 !!(value & MAX6635_STATUS_THIGH),
                                 !!(value & MAX6635_STATUS_TLOW)
                               );
        }
    }
    
    return written;
}

static ssize_t max6635_store_temp( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
    return -EIO;
}

static DEVICE_ATTR( max6635_temp, S_IRUGO | S_IWUSR, max6635_show_temp, max6635_store_temp   );



/* Configuration register */
static ssize_t max6635_show_config( struct device *dev, struct device_attribute *attr, char *buf )
{
    struct max6635_data  *max6635 = NULL;
    s32                   value;
    int                   written = 0;
    char                  a;
    char                  f;
    char                  i;
    char                  o;
    char                  s;
    
    max6635 = dev_get_drvdata(dev);
    
    if (max6635)
    {
        mutex_lock(&max6635->lock);
        
        value = i2c_smbus_read_byte_data(max6635->client, MAX6635_CONFIG_REG);
        
        mutex_unlock(&max6635->lock);
    
        if (value < 0)
        {
            written = scnprintf( buf, PAGE_SIZE, "read failure %d\n", value );
        }
        else
        {
            a = (value & MAX6635_CONFIG_ALERT_POLARITY)     ? 'A' : 'a';
            f = (value & MAX6635_CONFIG_FAULT_QUEUE)        ? 'F' : 'f';
            i = (value & MAX6635_CONFIG_INTERRUPT_MODE)     ? 'I' : 'i';
            o = (value & MAX6635_CONFIG_OVER_TEMP_POLARITY) ? 'O' : 'o';
            s = (value & MAX6635_CONFIG_SHUTDOWN_MODE)      ? 'S' : 's';
            
            written = scnprintf( buf, PAGE_SIZE,
                    "%c:alert polarity %s\n"
                    "%c:fault queue %s\n"
                    "%c:interrupt mode %s\n"
                    "%c:over temperature polarity %s\n"
                    "%c:shutdown mode %s\n",
                    a, isupper(a)?"ON":"OFF",
                    f, isupper(f)?"ON":"OFF",
                    i, isupper(i)?"ON":"OFF",
                    o, isupper(o)?"ON":"OFF",
                    s, isupper(s)?"ON":"OFF"
                );
        }
    }
    
    return written;
}

static ssize_t max6635_store_config( struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct max6635_data  *max6635     = NULL;
    int                   ret         = 0;
    u16                   configMask  = 0;
    u16                   configValue = 0;
    s32                   value;
    int                   i;
    
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
        case 'A':
            configMask  |=  MAX6635_CONFIG_ALERT_POLARITY;
            configValue |=  MAX6635_CONFIG_ALERT_POLARITY;
            break;
            
        case 'a':
            configMask  |=  MAX6635_CONFIG_ALERT_POLARITY;
            configValue &= ~MAX6635_CONFIG_ALERT_POLARITY;
            break;
            
        case 'F':
            configMask  |=  MAX6635_CONFIG_FAULT_QUEUE;
            configValue |=  MAX6635_CONFIG_FAULT_QUEUE;
            break;
            
        case 'f':
            configMask  |=  MAX6635_CONFIG_FAULT_QUEUE;
            configValue &= ~MAX6635_CONFIG_FAULT_QUEUE;
            break;
            
        case 'I':
            configMask  |=  MAX6635_CONFIG_INTERRUPT_MODE;
            configValue |=  MAX6635_CONFIG_INTERRUPT_MODE;
            break;
            
        case 'i':
            configMask  |=  MAX6635_CONFIG_INTERRUPT_MODE;
            configValue &= ~MAX6635_CONFIG_INTERRUPT_MODE;
            break;
            
        case 'O':
            configMask  |=  MAX6635_CONFIG_OVER_TEMP_POLARITY;
            configValue |=  MAX6635_CONFIG_OVER_TEMP_POLARITY;
            break;
            
        case 'o':
            configMask  |=  MAX6635_CONFIG_OVER_TEMP_POLARITY;
            configValue &= ~MAX6635_CONFIG_OVER_TEMP_POLARITY;
            break;
            
        case 'S':
            configMask  |=  MAX6635_CONFIG_SHUTDOWN_MODE;
            configValue |=  MAX6635_CONFIG_SHUTDOWN_MODE;
            break;
            
        case 's':
            configMask  |=  MAX6635_CONFIG_SHUTDOWN_MODE;
            configValue &= ~MAX6635_CONFIG_SHUTDOWN_MODE;
            break;
            
        case ' ':
        case '\t':
        case '\n':
        case '\r':
            break;
            
        default:
            printk("max6635 config invalid char[%d]=%d\n", i, c);
            return -EINVAL;
        }
    }
    
    max6635 = dev_get_drvdata(dev);
    
    if (max6635)
    {
        mutex_lock(&max6635->lock);
    
        value = i2c_smbus_read_byte_data(max6635->client, MAX6635_CONFIG_REG);
        if (value < 0)
        {
            printk("max6635:config failed to read %d\n", value);
            return -EIO;
        }
        else
        {
            value &= ~configMask;
            value |= configValue;
            
            ret = i2c_smbus_write_byte_data(max6635->client, MAX6635_CONFIG_REG, value);
            if (ret)
            {
                printk("max6635:config failed to write %d\n", ret);
                return -EIO;
            }
        }
    
        mutex_unlock(&max6635->lock);
    }
    
    return count;
}

static DEVICE_ATTR( max6635_config, S_IRUGO | S_IWUSR, max6635_show_config, max6635_store_config );


static int max6635_extractLimitValue(u16 value)
{
    int temp = (value >> 7) & 0x1FF;
    
    if (value & 0x8000)
    {
        temp -= 0x200;
    }
    
    return temp;
}

/* The limit registers are 9-bit twos-complement in the MSBs.  The 7 LSBs are not used.
* The value is in degrees C.
*/
static ssize_t max6635_show_limit_register( struct device *dev, char *buf, u8 regReadCmd )
{
    struct max6635_data  *max6635 = NULL;
    s32                   value;
    int                   written = 0;
    
    max6635 = dev_get_drvdata(dev);
    
    if (max6635)
    {
        mutex_lock(&max6635->lock);
    
        value = i2c_smbus_read_word_data(max6635->client, regReadCmd);
        
        mutex_unlock(&max6635->lock);
        
        if (value < 0)
        {
            written = scnprintf( buf, PAGE_SIZE, "read failure %d\n", value );
        }
        else
        {
            int temp = max6635_extractLimitValue(value);
            
            written = scnprintf( buf, PAGE_SIZE, "%d\n", temp );
        }
    }
    
    return written;
}
    
/* The limit registers are 9-bit twos-complement in the MSBs.  The 7 LSBs are not used.
* The value is in degrees C.
*/
static ssize_t max6635_store_limit_register( struct device *dev, const char *buf,
                                            size_t count, u8 regWriteCmd )
{
    u16  t        = 0;
    int  valid    = 0;
    int  negative = 0;
    int  remain   = count;
    u16  limit    = MAX6635_LIMIT_REG_MAX;
    
    /* Skip initial space. */
    while (remain && isspace(*buf))
    {
        buf++;
        remain--;
    }
    
    /* Check for sign */    
    if (remain && (*buf == '-'))
    {
        negative  = 1;
        limit = -MAX6635_LIMIT_REG_MIN;
        
        buf++;
        remain--;
    }
    
    /* Read digits. */
    while (remain && isdigit(*buf))
    {
        if (t < 0x1000)
        {
            t *= 10;
            t += *buf - '0';
            
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
    
    if (valid && (remain == 0) && (t <= limit))
    {
        struct max6635_data  *max6635 = NULL;
    
        if (negative)
        {
            t = 0x200 - t;
        }
        
        t <<= 7;
                
        max6635 = dev_get_drvdata(dev);
    
        if (max6635)
        {
            int  ret;
            
            mutex_lock(&max6635->lock);
    
            ret = i2c_smbus_write_word_data(max6635->client, regWriteCmd, t);
            
            mutex_unlock(&max6635->lock);
            
            if (ret < 0)
            {
                printk("max6635:store limit, failed to write %d\n", ret);
                return -EIO;
            }
        }
    }
    else
    {
        printk("max6635:store limit, invalid value\n");
        return -EINVAL;
    }
    
    return count;
}


/* Hyst limit register */
static ssize_t max6635_show_thyst( struct device *dev, struct device_attribute *attr, char *buf )
{
    return max6635_show_limit_register( dev, buf, MAX6635_THYST_REG );
}

static ssize_t max6635_store_thyst( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
    return max6635_store_limit_register( dev, buf, count, MAX6635_THYST_REG );
}

static DEVICE_ATTR( max6635_thyst, S_IRUGO | S_IWUSR, max6635_show_thyst, max6635_store_thyst  );



/* Max limit register */
static ssize_t max6635_show_tmax( struct device *dev, struct device_attribute *attr, char *buf )
{
    return max6635_show_limit_register( dev, buf, MAX6635_TMAX_REG );
}

static ssize_t max6635_store_tmax( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
    return max6635_store_limit_register( dev, buf, count, MAX6635_TMAX_REG );
}

static DEVICE_ATTR( max6635_tmax, S_IRUGO | S_IWUSR, max6635_show_tmax, max6635_store_tmax   );



/* Low limit register */
static ssize_t max6635_show_tlow( struct device *dev, struct device_attribute *attr, char *buf )
{
    return max6635_show_limit_register( dev, buf, MAX6635_TLOW_REG );
}

static ssize_t max6635_store_tlow( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
    return max6635_store_limit_register( dev, buf, count, MAX6635_TLOW_REG );
}

static DEVICE_ATTR( max6635_tlow, S_IRUGO | S_IWUSR, max6635_show_tlow, max6635_store_tlow   );



/* High limit register */
static ssize_t max6635_show_thigh( struct device *dev, struct device_attribute *attr, char *buf )
{
    return max6635_show_limit_register( dev, buf, MAX6635_THIGH_REG );
}

static ssize_t max6635_store_thigh( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
    return max6635_store_limit_register( dev, buf, count, MAX6635_THIGH_REG );
}

static DEVICE_ATTR( max6635_thigh, S_IRUGO | S_IWUSR, max6635_show_thigh, max6635_store_thigh  );


static int max6635_open(struct inode *inode, struct file *filp)
{
    int                   result = 0;
    
    filp->private_data  = max6635_device;
    
    return result;
}

static int max6635_release(struct inode *inode, struct file *filp)
{
    if (filp->private_data)
    {
        filp->private_data = 0;
    }
    
    return 0;
}

static int max6635_writeLimitRegister(struct max6635_data *max6635, u8 cmd, int temp)
{
    int  ret = 0;
    
    if (temp != MAX6635_DONT_UPDATE_LIMIT)
    {
        if ((temp < MAX6635_LIMIT_REG_MIN) || (temp > MAX6635_LIMIT_REG_MAX))
        {
            ret = -EINVAL;
        }
        else
        {
            u16 regVal;
             
            if (temp < 0)
            {
                regVal = 0x200 + temp;
            }
            else
            {
                regVal = temp;
            }
             
            regVal <<= 7;
            
            ret = i2c_smbus_write_word_data(max6635->client, cmd, regVal);
        }
    }
    
    return ret;
}

static int max6635_readLimitRegister(struct max6635_data *max6635, u8 cmd, int* temp)
{
    s32  value;
    
    value = i2c_smbus_read_word_data(max6635->client, cmd);
    if (value < 0)
    {
        return value;
    }
    
    *temp = max6635_extractLimitValue(value);
    
    return 0;
}

static int max6635_ioctl(struct inode   *inode_p,
                struct file    *filp,
                unsigned int   cmd,
                unsigned long  arg)
{
    struct max6635_data  *max6635 = NULL;
    int                   ret     = 0;

    /* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
    if (_IOC_TYPE(cmd) != MAX6635_IOCTL_MAGIC) return -ENOTTY;
    if (_IOC_NR(cmd)    > MAX6635_IOCTL_MAXNR) return -ENOTTY;
    
    max6635 = filp->private_data;
    
    switch (cmd)
    {
    case MAX6635_IOCTL_READ_TEMP:
        {
            Max6635Status  status;
            s32            value;
            
            value = i2c_smbus_read_word_data(max6635->client, MAX6635_TEMP_REG);
            if (value < 0)
            {
                ret = value;
                break;
            }
            
            status.flags   = value & 7;
            
            status.scaledCelcius = (value >> 3) & 0x1FFF;
            if (status.scaledCelcius & 0x1000)
            {
                status.scaledCelcius = 0x2000 - status.scaledCelcius;
            }
            
            if (copy_to_user((void*)arg, &status, sizeof(status)))
            {
                ret = -EFAULT;
            }
        }
        break;
    
    case MAX6635_IOCTL_WRITE_CONFIG:
        {
            Max6635Config  config;
            
            if (copy_from_user(&config, (void*)arg, sizeof(config)))
            {
                ret = -EFAULT;
                break;
            }
        
            ret = i2c_smbus_write_byte_data(max6635->client, MAX6635_CONFIG_REG, config.flags);
        }
        break;
        
    case MAX6635_IOCTL_READ_CONFIG:
        {
            Max6635Config  config;
            s32            value;
            
            value = i2c_smbus_read_byte_data(max6635->client, MAX6635_CONFIG_REG);
            if (value < 0)
            {
                ret = value;
                break;
            }
            
            config.flags = value;
            
            if (copy_to_user((void*)arg, &config, sizeof(config)))
            {
                ret = -EFAULT;
            }
        }
        break;
        
    case MAX6635_IOCTL_WRITE_LIMITS:
        {
            Max6635Limits  limits;
            
            if (copy_from_user(&limits, (void*)arg, sizeof(limits)))
            {
                ret = -EFAULT;
                break;
            }
            
            ret = max6635_writeLimitRegister(max6635, MAX6635_THIGH_REG, limits.thigh);
            if (ret)
            {
                break;
            }
            
            ret = max6635_writeLimitRegister(max6635, MAX6635_TLOW_REG, limits.tlow);
            if (ret)
            {
                break;
            }
            
            ret = max6635_writeLimitRegister(max6635, MAX6635_TMAX_REG, limits.tmax);
            if (ret)
            {
                break;
            }
            
            ret = max6635_writeLimitRegister(max6635, MAX6635_THYST_REG, limits.thyst);
        }
        break;
        
    case MAX6635_IOCTL_READ_LIMITS:
        {
            Max6635Limits  limits;
            
            ret = max6635_readLimitRegister(max6635, MAX6635_THIGH_REG, &limits.thigh);
            if (ret)
            {
                break;
            }
            
            ret = max6635_readLimitRegister(max6635, MAX6635_TLOW_REG, &limits.tlow);
            if (ret)
            {
                break;
            }
            
            ret = max6635_readLimitRegister(max6635, MAX6635_TMAX_REG, &limits.tmax);
            if (ret)
            {
                break;
            }
            
            ret = max6635_readLimitRegister(max6635, MAX6635_THYST_REG, &limits.thyst);
            if (ret)
            {
                break;
            }
            
            if (copy_to_user((void*)arg, &limits, sizeof(limits)))
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

static struct file_operations max6635_fops = {
    .owner      = THIS_MODULE,
    .open       = max6635_open,
    .release    = max6635_release,
    .ioctl      = max6635_ioctl,
};


/* Return 0 if detection is successful, -ENODEV otherwise */
static int max6635_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
				     | I2C_FUNC_SMBUS_READ_BYTE))
		return -ENODEV;

	strlcpy(info->type, "max6635", I2C_NAME_SIZE);

	return 0;
}

static int max6635_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    struct device        *dev      = &client->dev;
    struct max6635_data  *max6635  = NULL;
    int                   err;

    max6635 = kzalloc(sizeof(struct max6635_data), GFP_KERNEL);
    if (max6635 == NULL)
    {
        err = -ENOMEM;
        goto err_no_mem;
    }

    max6635->client = client;

    max6635->miscdev.fops   = &max6635_fops;
    max6635->miscdev.name   = "max6635";
    max6635->miscdev.minor  =  MAX6635_MINOR;  /* Our /dev is read only, so use a fixed value */
    
    err = misc_register( &max6635->miscdev );
    if ( err )
    {
        printk( KERN_INFO "failed to register max6635 misc device\n" );
        goto err_no_misc_device;
    }
    
    /* Init i2c_client */
    i2c_set_clientdata(client, max6635);
    mutex_init(&max6635->lock);

    err = device_create_file(dev, &dev_attr_max6635_temp);
    if (err)
    {
        goto err_no_sysfs_temp;
    }
    err = device_create_file(dev, &dev_attr_max6635_config);
    if (err)
    {
        goto err_no_sysfs_config;
    }
    err = device_create_file(dev, &dev_attr_max6635_thyst);
    if (err)
    {
        goto err_no_sysfs_thyst;
    }
    err = device_create_file(dev, &dev_attr_max6635_tmax);
    if (err)
    {
        goto err_no_sysfs_tmax;
    }
    err = device_create_file(dev, &dev_attr_max6635_tlow);
    if (err)
    {
        goto err_no_sysfs_tlow;
    }
    err = device_create_file(dev, &dev_attr_max6635_thigh);
    if (err)
    {
        goto err_no_sysfs_thigh;
    }
    
    max6635_device = max6635;
	
    return 0;

err_no_sysfs_thigh:
    device_remove_file( dev, &dev_attr_max6635_tlow);
err_no_sysfs_tlow:
    device_remove_file( dev, &dev_attr_max6635_tmax);
err_no_sysfs_tmax:
    device_remove_file( dev, &dev_attr_max6635_thyst);
err_no_sysfs_thyst:
    device_remove_file( dev, &dev_attr_max6635_config);
err_no_sysfs_config:
    device_remove_file( dev, &dev_attr_max6635_temp);
err_no_sysfs_temp:
    misc_deregister( &max6635->miscdev );
err_no_misc_device:
	kfree(max6635);
err_no_mem:
	return err;
}

static int max6635_remove(struct i2c_client *client)
{
	struct max6635_data *max6635 = i2c_get_clientdata(client);
    struct device       *dev     = &client->dev;

    max6635_device = NULL;
    
    device_remove_file( dev, &dev_attr_max6635_thigh);
    device_remove_file( dev, &dev_attr_max6635_tlow);
    device_remove_file( dev, &dev_attr_max6635_tmax);
    device_remove_file( dev, &dev_attr_max6635_thyst);
    device_remove_file( dev, &dev_attr_max6635_config);
    device_remove_file( dev, &dev_attr_max6635_temp);
    
    misc_deregister( &max6635->miscdev );
    
	kfree(max6635);

	return 0;
}

static const struct i2c_device_id max6635_id[] = {
	{ "max6635", 0 },
	{ }
};

static struct i2c_driver max6635_driver = {
	.driver = {
		.name	= "max6635",
	},
	.probe		= max6635_probe,
	.remove		= max6635_remove,
	.id_table	= max6635_id,

	.detect		= max6635_detect,
	.address_data	= &addr_data,
};

static int __init max6635_init(void)
{
	return i2c_add_driver(&max6635_driver);
}

static void __exit max6635_exit(void)
{
	i2c_del_driver(&max6635_driver);
}


MODULE_DESCRIPTION("Driver for Maxim 6635");
MODULE_AUTHOR("ip.access Ltd");
MODULE_LICENSE("GPL");

module_init(max6635_init);
module_exit(max6635_exit);
