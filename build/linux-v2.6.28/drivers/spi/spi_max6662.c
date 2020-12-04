/*
* max6662.c -- support Maxim 6662 Temperature Sensor
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
#include <asm/uaccess.h>

#ifdef CONFIG_ARCH_PC302
#include <mach/gpio.h>
#endif

#include <linux/ipa/max6662.h>

#define  MAX6662_TRANSFER_SIZE             (3)

#define  MAX6662_LIMIT_REG_MIN             (-256)
#define  MAX6662_LIMIT_REG_MAX             (255)

#define  MAX6662_READ_TEMP_CMD             (0xC1)

#define  MAX6662_READ_CONFIG_CMD           (0xC3)
#define  MAX6662_WRITE_CONFIG_CMD          (0x83)

#define  MAX6662_READ_THYST_CMD            (0xC5)
#define  MAX6662_WRITE_THYST_CMD           (0x85)

#define  MAX6662_READ_TMAX_CMD             (0xC7)
#define  MAX6662_WRITE_TMAX_CMD            (0x87)

#define  MAX6662_READ_TLOW_CMD             (0xC9)
#define  MAX6662_WRITE_TLOW_CMD            (0x89)

#define  MAX6662_READ_THIGH_CMD            (0xCB)
#define  MAX6662_WRITE_THIGH_CMD           (0x8B)

#define  MAX6662_CONFIG_SHUTDOWN_MODE      (1 <<  8)
#define  MAX6662_CONFIG_INTERRUPT_MODE     (1 <<  9)
#define  MAX6662_CONFIG_OVER_TEMP_POLARITY (1 << 10)
#define  MAX6662_CONFIG_ALERT_POLARITY     (1 << 11)
#define  MAX6662_CONFIG_FAULT_QUEUE        (1 << 12)

#define  MAX6662_STATUS_TLOW               (1 <<  0)
#define  MAX6662_STATUS_THIGH              (1 <<  1)
#define  MAX6662_STATUS_TOVER              (1 <<  2)


struct max6662_data {
	struct miscdevice      miscdev;
	struct spi_device     *spi;                         /* SPI master we're using */
	struct mutex           lock;                        /* Required for read-modify-write of config reg */
	u16                    buf[MAX6662_TRANSFER_SIZE];  /* DMA safe buffer for SPI transfers */
};

static struct max6662_data  *max6662_device = NULL;

/* Writing to the MAX6662 requires an 8-bit command and a 16-bit value */
static int max6662_writeRegister(struct max6662_data *max6662, u8 cmd, u16 val)
{
	struct spi_transfer t = {
	        .rx_buf  = NULL,
	        .tx_buf  = max6662->buf,
	    };
	struct spi_message  m;
	
	/* In 3Wire mode we'll use Microwire and pass a command and a single
	 * 16-bit data word.  Otherwise, we'll use SPI mode and pass three
	 * 8-bit words.  Note that the length is always specified in bytes,
	 * so two 16-bit words needs 4 bytes.  Data is passed MSB first.
	 */
	if (max6662->spi->mode & SPI_3WIRE)
	{
		t.len           = 4,
		t.bits_per_word = 16;
		
		max6662->buf[0] = cmd;
		max6662->buf[1] = val;
	}
	else
	{
		u8 *p = (u8*)max6662->buf;
		
		t.len           = 3,
		t.bits_per_word = 8;
		
		p[0] = cmd;
		p[1] = (val >> 8) & 0xFF;
		p[2] = val & 0xFF ;
	}
	
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	return spi_sync(max6662->spi, &m);
}

/* Reading from the MAX6662 requires an 8-bit command and returns a 16-bit value */
static int max6662_readRegister(struct max6662_data *max6662, u8 cmd, u16 *val_p)
{
	struct spi_transfer t = {
	        .rx_buf  = max6662->buf,
	        .tx_buf  = max6662->buf,
	    };
	struct spi_message  m;
	int                 ret;
	u16                 val;
	
	/* In 3Wire mode we'll use Microwire and pass a command and read a
	 * single 16-bit data word.  Otherwise, we'll use SPI mode and transfer
	 * three 8-bit words, one out, two in.  Note that the length is always
	 * specified in bytes, so two 16-bit words needs 4 bytes.  Data is
	 * passed MSB first.
	 */
	if (max6662->spi->mode & SPI_3WIRE)
	{
		t.len           = 4,
		t.bits_per_word = 16;
		
		max6662->buf[0] = cmd;
		max6662->buf[1] = 0xFFFF;
	}
	else
	{
		u8 *p = (u8*)max6662->buf;
		
		t.len           = 3,
		t.bits_per_word = 8;
		
		p[0] = cmd;
		p[1] = 0xFF;
		p[2] = 0xFF;
	}
	
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	ret = spi_sync(max6662->spi, &m);
	if (ret < 0)
	{
		return ret;
	}
	
	if (max6662->spi->mode & SPI_3WIRE)
	{
		val = max6662->buf[1];
	}
	else
	{
		u8 *p = (u8*)max6662->buf;
		
		val   = p[1];
		val <<= 8;
		val  |= p[2];
	}
	
	if (val_p)
	{
		*val_p = val;
	}
	
	return 0;
}

/* Temperature register.  Read only. */
static ssize_t max6662_show_temp( struct device *dev, struct device_attribute *attr, char *buf )
{
	struct max6662_data  *max6662 = NULL;
	u16                   value;
	int                   ret;
	int                   written = 0;
	
	max6662 = dev_get_drvdata(dev);
	
	if (max6662)
	{
		mutex_lock(&max6662->lock);
		
		ret = max6662_readRegister(max6662, MAX6662_READ_TEMP_CMD, &value);
		
		mutex_unlock(&max6662->lock);
		
		if (ret < 0)
		{
			written = scnprintf( buf, PAGE_SIZE, "read failure %d\n", ret );
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
			                     !!(value & MAX6662_STATUS_TOVER),
			                     !!(value & MAX6662_STATUS_THIGH),
			                     !!(value & MAX6662_STATUS_TLOW)
			                   );
		}
	}
	
	return written;
}

static ssize_t max6662_store_temp( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	return -EIO;
}

static DEVICE_ATTR( max6662_temp, S_IRUGO | S_IWUSR, max6662_show_temp, max6662_store_temp   );



/* Configuration register */
static ssize_t max6662_show_config( struct device *dev, struct device_attribute *attr, char *buf )
{
	struct max6662_data  *max6662 = NULL;
	u16                   value;
	int                   written = 0;
	char                  a;
	char                  f;
	char                  i;
	char                  o;
	char                  s;
	
	max6662 = dev_get_drvdata(dev);
	
	if (max6662)
	{
		int  ret;
			
		mutex_lock(&max6662->lock);
		
		ret = max6662_readRegister(max6662, MAX6662_READ_CONFIG_CMD, &value);
		
		mutex_unlock(&max6662->lock);
	
		if (ret < 0)
		{
			written = scnprintf( buf, PAGE_SIZE, "read failure %d\n", ret );
		}
		else
		{
			a = (value & MAX6662_CONFIG_ALERT_POLARITY)     ? 'A' : 'a';
			f = (value & MAX6662_CONFIG_FAULT_QUEUE)        ? 'F' : 'f';
			i = (value & MAX6662_CONFIG_INTERRUPT_MODE)     ? 'I' : 'i';
			o = (value & MAX6662_CONFIG_OVER_TEMP_POLARITY) ? 'O' : 'o';
			s = (value & MAX6662_CONFIG_SHUTDOWN_MODE)      ? 'S' : 's';
			
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

static ssize_t max6662_store_config( struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct max6662_data  *max6662     = NULL;
	int                   ret         = 0;
	u16                   configMask  = 0;
	u16                   configValue = 0;
	u16                   value;
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
			configMask  |=  MAX6662_CONFIG_ALERT_POLARITY;
			configValue |=  MAX6662_CONFIG_ALERT_POLARITY;
			break;
			
		case 'a':
			configMask  |=  MAX6662_CONFIG_ALERT_POLARITY;
			configValue &= ~MAX6662_CONFIG_ALERT_POLARITY;
			break;
			
		case 'F':
			configMask  |=  MAX6662_CONFIG_FAULT_QUEUE;
			configValue |=  MAX6662_CONFIG_FAULT_QUEUE;
			break;
			
		case 'f':
			configMask  |=  MAX6662_CONFIG_FAULT_QUEUE;
			configValue &= ~MAX6662_CONFIG_FAULT_QUEUE;
			break;
			
		case 'I':
			configMask  |=  MAX6662_CONFIG_INTERRUPT_MODE;
			configValue |=  MAX6662_CONFIG_INTERRUPT_MODE;
			break;
			
		case 'i':
			configMask  |=  MAX6662_CONFIG_INTERRUPT_MODE;
			configValue &= ~MAX6662_CONFIG_INTERRUPT_MODE;
			break;
			
		case 'O':
			configMask  |=  MAX6662_CONFIG_OVER_TEMP_POLARITY;
			configValue |=  MAX6662_CONFIG_OVER_TEMP_POLARITY;
			break;
			
		case 'o':
			configMask  |=  MAX6662_CONFIG_OVER_TEMP_POLARITY;
			configValue &= ~MAX6662_CONFIG_OVER_TEMP_POLARITY;
			break;
			
		case 'S':
			configMask  |=  MAX6662_CONFIG_SHUTDOWN_MODE;
			configValue |=  MAX6662_CONFIG_SHUTDOWN_MODE;
			break;
			
		case 's':
			configMask  |=  MAX6662_CONFIG_SHUTDOWN_MODE;
			configValue &= ~MAX6662_CONFIG_SHUTDOWN_MODE;
			break;
			
		case ' ':
		case '\t':
		case '\n':
		case '\r':
			break;
			
		default:
			printk("max6662 config invalid char[%d]=%d\n", i, c);
			return -EINVAL;
		}
	}
	
	max6662 = dev_get_drvdata(dev);
	
	if (max6662)
	{
		mutex_lock(&max6662->lock);
	
		ret = max6662_readRegister(max6662, MAX6662_READ_CONFIG_CMD, &value);
		if (ret)
		{
			printk("max6662:config failed to read %d\n", ret);
			return -EIO;
		}
		else
		{
			value &= ~configMask;
			value |= configValue;
			
			ret = max6662_writeRegister(max6662, MAX6662_WRITE_CONFIG_CMD, value);
			if (ret)
			{
				printk("max6662:config failed to write %d\n", ret);
				return -EIO;
			}
		}
	
		mutex_unlock(&max6662->lock);
	}
	
	return count;
}

static DEVICE_ATTR( max6662_config, S_IRUGO | S_IWUSR, max6662_show_config, max6662_store_config );


static int max6662_extractLimitValue(u16 value)
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
static ssize_t max6662_show_limit_register( struct device *dev, char *buf, u8 regReadCmd )
{
	struct max6662_data  *max6662 = NULL;
	u16                   value;
	int                   ret;
	int                   written = 0;
	
	max6662 = dev_get_drvdata(dev);
	
	if (max6662)
	{
		mutex_lock(&max6662->lock);
	
		ret = max6662_readRegister(max6662, regReadCmd, &value);
		
		mutex_unlock(&max6662->lock);
		
		if (ret < 0)
		{
			written = scnprintf( buf, PAGE_SIZE, "read failure %d\n", ret );
		}
		else
		{
			int temp = max6662_extractLimitValue(value);
			
			written = scnprintf( buf, PAGE_SIZE, "%d\n", temp );
		}
	}
	
	return written;
}
    
/* The limit registers are 9-bit twos-complement in the MSBs.  The 7 LSBs are not used.
* The value is in degrees C.
*/
static ssize_t max6662_store_limit_register( struct device *dev, const char *buf,
                                            size_t count, u8 regWriteCmd )
{
	u16  t        = 0;
	int  valid    = 0;
	int  negative = 0;
	int  remain   = count;
	u16  limit    = MAX6662_LIMIT_REG_MAX;
	
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
		limit = -MAX6662_LIMIT_REG_MIN;
		
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
		struct max6662_data  *max6662 = NULL;
	
		if (negative)
		{
			t = 0x200 - t;
		}
		
		t <<= 7;
				
		max6662 = dev_get_drvdata(dev);
	
		if (max6662)
		{
			int  ret;
			
			mutex_lock(&max6662->lock);
	
			ret = max6662_writeRegister(max6662, regWriteCmd, t);
			
			mutex_unlock(&max6662->lock);
			
			if (ret < 0)
			{
				printk("max6662:store limit, failed to write %d\n", ret);
				return -EIO;
			}
		}
	}
	else
	{
		printk("max6662:store limit, invalid value\n");
		return -EINVAL;
	}
	
	return count;
}


/* Hyst limit register */
static ssize_t max6662_show_thyst( struct device *dev, struct device_attribute *attr, char *buf )
{
	return max6662_show_limit_register( dev, buf, MAX6662_READ_THYST_CMD );
}

static ssize_t max6662_store_thyst( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	return max6662_store_limit_register( dev, buf, count, MAX6662_WRITE_THYST_CMD );
}

static DEVICE_ATTR( max6662_thyst, S_IRUGO | S_IWUSR, max6662_show_thyst, max6662_store_thyst  );



/* Max limit register */
static ssize_t max6662_show_tmax( struct device *dev, struct device_attribute *attr, char *buf )
{
	return max6662_show_limit_register( dev, buf, MAX6662_READ_TMAX_CMD );
}

static ssize_t max6662_store_tmax( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	return max6662_store_limit_register( dev, buf, count, MAX6662_WRITE_TMAX_CMD );
}

static DEVICE_ATTR( max6662_tmax, S_IRUGO | S_IWUSR, max6662_show_tmax, max6662_store_tmax   );



/* Low limit register */
static ssize_t max6662_show_tlow( struct device *dev, struct device_attribute *attr, char *buf )
{
	return max6662_show_limit_register( dev, buf, MAX6662_READ_TLOW_CMD );
}

static ssize_t max6662_store_tlow( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	return max6662_store_limit_register( dev, buf, count, MAX6662_WRITE_TLOW_CMD );
}

static DEVICE_ATTR( max6662_tlow, S_IRUGO | S_IWUSR, max6662_show_tlow, max6662_store_tlow   );



/* High limit register */
static ssize_t max6662_show_thigh( struct device *dev, struct device_attribute *attr, char *buf )
{
	return max6662_show_limit_register( dev, buf, MAX6662_READ_THIGH_CMD );
}

static ssize_t max6662_store_thigh( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	return max6662_store_limit_register( dev, buf, count, MAX6662_WRITE_THIGH_CMD );
}

static DEVICE_ATTR( max6662_thigh, S_IRUGO | S_IWUSR, max6662_show_thigh, max6662_store_thigh  );



/* Baud rate */
static ssize_t max6662_show_baud( struct device *dev, struct device_attribute *attr, char *buf )
{
	struct max6662_data  *max6662 = NULL;
	
	max6662 = dev_get_drvdata(dev);
	
	if (max6662)
	{
		return scnprintf( buf, PAGE_SIZE, "%d\n", max6662->spi->max_speed_hz );
	}
	else
	{
		return scnprintf( buf, PAGE_SIZE, "???\n" );
	}
}

static ssize_t max6662_store_baud( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	struct max6662_data  *max6662 = NULL;
	int                   speed;

	if ((1 != sscanf(buf, "%d", &speed)) || (speed > 100000000) || (speed < 3052))
	{
		return -EINVAL;
	}

	max6662 = dev_get_drvdata(dev);
	if (!max6662)
	{
		return -EIO;
	}

	max6662->spi->max_speed_hz = speed;
	
	spi_setup(max6662->spi);
	
	return count;
}

static DEVICE_ATTR( max6662_baud, S_IRUGO | S_IWUSR, max6662_show_baud, max6662_store_baud );



/* SPI mode */
static ssize_t max6662_show_mode( struct device *dev, struct device_attribute *attr, char *buf )
{
	struct max6662_data  *max6662 = NULL;
	
	max6662 = dev_get_drvdata(dev);
	
	if (max6662)
	{
		return scnprintf( buf, PAGE_SIZE, "%d\n", max6662->spi->mode & 3 );
	}
	else
	{
		return scnprintf( buf, PAGE_SIZE, "???\n" );
	}
}

static ssize_t max6662_store_mode( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	struct max6662_data  *max6662 = NULL;
	int                   mode;

	if ((1 != sscanf(buf, "%d", &mode)) || (mode > 3) || (mode < 0))
	{
		return -EINVAL;
	}

	max6662 = dev_get_drvdata(dev);
	if (!max6662)
	{
		return -EIO;
	}

	max6662->spi->mode &= ~3;
	max6662->spi->mode |= mode;

	spi_setup(max6662->spi);

	return count;
}

static DEVICE_ATTR( max6662_mode, S_IRUGO | S_IWUSR, max6662_show_mode, max6662_store_mode );



/* SPI 3-Wire */
static ssize_t max6662_show_3wire( struct device *dev, struct device_attribute *attr, char *buf )
{
	struct max6662_data  *max6662 = NULL;
	
	max6662 = dev_get_drvdata(dev);
	
	if (max6662)
	{
		return scnprintf( buf, PAGE_SIZE, "%d\n", (max6662->spi->mode & SPI_3WIRE) ? 1 : 0 );
	}
	else
	{
		return scnprintf( buf, PAGE_SIZE, "???\n" );
	}
}

static ssize_t max6662_store_3wire( struct device *dev, struct device_attribute *attr, const char *buf, size_t count )
{
	struct max6662_data  *max6662 = NULL;
	int                   threeWire;

	if ((1 != sscanf(buf, "%d", &threeWire)) || (threeWire > 1) || (threeWire < 0))
	{
		return -EINVAL;
	}

	max6662 = dev_get_drvdata(dev);
	if (!max6662)
	{
		return -EIO;
	}

	if (threeWire)
	{
		max6662->spi->mode |= SPI_3WIRE;
	}
	else
	{
		max6662->spi->mode &= ~SPI_3WIRE;
	}

	spi_setup(max6662->spi);

	return count;
}

static DEVICE_ATTR( max6662_3wire, S_IRUGO | S_IWUSR, max6662_show_3wire, max6662_store_3wire );





static int max6662_open(struct inode *inode, struct file *filp)
{
	int                   result = 0;
	
	filp->private_data  = max6662_device;
	
	return result;
}

static int max6662_release(struct inode *inode, struct file *filp)
{
	if (filp->private_data)
	{
		filp->private_data = 0;
	}
	
	return 0;
}

static int max6662_writeLimitRegister(struct max6662_data *max6662, u8 cmd, int temp)
{
	int  ret = 0;
	
	if (temp != MAX6662_DONT_UPDATE_LIMIT)
	{
		if ((temp < MAX6662_LIMIT_REG_MIN) || (temp > MAX6662_LIMIT_REG_MAX))
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
			
			ret = max6662_writeRegister(max6662, cmd, regVal);
		}
	}
	
	return ret;
}

static int max6662_readLimitRegister(struct max6662_data *max6662, u8 cmd, int* temp)
{
	int  ret = 0;
	u16  regVal;
	
	ret = max6662_readRegister(max6662, cmd, &regVal);
	if (ret)
	{
		return ret;
	}
	
	*temp = max6662_extractLimitValue(regVal);
	
	return 0;
}

static int max6662_ioctl(struct inode   *inode_p,
                struct file    *filp,
                unsigned int   cmd,
                unsigned long  arg)
{
	struct max6662_data  *max6662 = NULL;
	int                   ret     = 0;

	/* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
	if (_IOC_TYPE(cmd) != MAX6662_IOCTL_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd)    > MAX6662_IOCTL_MAXNR) return -ENOTTY;
	
	max6662 = filp->private_data;
	
	switch (cmd)
	{
	case MAX6662_IOCTL_READ_TEMP:
		{
			Max6662Status  status;
			u16            value;
			
			ret = max6662_readRegister(max6662, MAX6662_READ_TEMP_CMD, &value);
			if (ret)
			{
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
	
	case MAX6662_IOCTL_WRITE_CONFIG:
		{
			Max6662Config  config;
			
			if (copy_from_user(&config, (void*)arg, sizeof(config)))
			{
				ret = -EFAULT;
				break;
			}
		
			ret = max6662_writeRegister(max6662, MAX6662_WRITE_CONFIG_CMD, config.flags);
		}
		break;
		
	case MAX6662_IOCTL_READ_CONFIG:
		{
			Max6662Config  config;
			u16            value;
			
			ret = max6662_readRegister(max6662, MAX6662_READ_CONFIG_CMD, &value);
			if (ret)
			{
				break;
			}
			
			config.flags = value;
			
			if (copy_to_user((void*)arg, &config, sizeof(config)))
			{
				ret = -EFAULT;
			}
		}
		break;
		
	case MAX6662_IOCTL_WRITE_LIMITS:
		{
			Max6662Limits  limits;
			
			if (copy_from_user(&limits, (void*)arg, sizeof(limits)))
			{
				ret = -EFAULT;
				break;
			}
			
			ret = max6662_writeLimitRegister(max6662, MAX6662_WRITE_THIGH_CMD, limits.thigh);
			if (ret)
			{
				break;
			}
			
			ret = max6662_writeLimitRegister(max6662, MAX6662_WRITE_TLOW_CMD, limits.tlow);
			if (ret)
			{
				break;
			}
			
			ret = max6662_writeLimitRegister(max6662, MAX6662_WRITE_TMAX_CMD, limits.tmax);
			if (ret)
			{
				break;
			}
			
			ret = max6662_writeLimitRegister(max6662, MAX6662_WRITE_THYST_CMD, limits.thyst);
		}
		break;
		
	case MAX6662_IOCTL_READ_LIMITS:
		{
			Max6662Limits  limits;
			
			ret = max6662_readLimitRegister(max6662, MAX6662_READ_THIGH_CMD, &limits.thigh);
			if (ret)
			{
				break;
			}
			
			ret = max6662_readLimitRegister(max6662, MAX6662_READ_TLOW_CMD, &limits.tlow);
			if (ret)
			{
				break;
			}
			
			ret = max6662_readLimitRegister(max6662, MAX6662_READ_TMAX_CMD, &limits.tmax);
			if (ret)
			{
				break;
			}
			
			ret = max6662_readLimitRegister(max6662, MAX6662_READ_THYST_CMD, &limits.thyst);
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

static struct file_operations max6662_fops = {
	.owner      = THIS_MODULE,
	.open       = max6662_open,
	.release    = max6662_release,
	.ioctl      = max6662_ioctl,
};


static int max6662_probe(struct spi_device *spi)
{
	struct max6662_data  *max6662 = NULL;
	int                   err;
	
	if (max6662_device != NULL)
	{
		err = -EBUSY;
		goto err_no_max6662_data;
	}
	
	if (!(max6662 = kzalloc(sizeof *max6662, GFP_KERNEL))) {
		err = -ENOMEM;
		goto err_no_max6662_data;
	}
	
	max6662->miscdev.fops   = &max6662_fops;
	max6662->miscdev.name   = "max6662";
	max6662->miscdev.minor  =  MAX6662_MINOR;  /* Our /dev is read only, so use a fixed value */
	
	err = misc_register( &max6662->miscdev );
	if ( err )
	{
		printk( KERN_INFO "failed to register max6662 misc device\n" );
		goto err_no_misc_device;
	}
	
	mutex_init(&max6662->lock);
	max6662->spi = spi_dev_get(spi);
	dev_set_drvdata(&spi->dev, max6662);

#ifdef CONFIG_ARCH_PC302
	/* Just in case we can't use the SSI block's chip selects, we'll allow
	 * the platform to specify a GPIO line to use as a chip select via the
	 * platform_data pointer.  This will be the number of the GPIO to use.
	 */
	if (spi->dev.platform_data)
	{
		err = gpio_request((unsigned)(spi->dev.platform_data), "max_cs");
		if (err)
		{
			printk( KERN_INFO "failed to request max6662 gpio CS\n" );
			goto err_request_max_cs;
		}

		gpio_direction_output((unsigned)(spi->dev.platform_data), (spi->mode & SPI_CS_HIGH) ? 0 : 1);
		gpio_set_value((unsigned)(spi->dev.platform_data), (spi->mode & SPI_CS_HIGH) ? 0 : 1);
	}
#endif
	
	err = device_create_file(&spi->dev, &dev_attr_max6662_temp);
	if (err)
	{
		goto err_no_sysfs_temp;
	}
	err = device_create_file(&spi->dev, &dev_attr_max6662_config);
	if (err)
	{
		goto err_no_sysfs_config;
	}
	err = device_create_file(&spi->dev, &dev_attr_max6662_thyst);
	if (err)
	{
		goto err_no_sysfs_thyst;
	}
	err = device_create_file(&spi->dev, &dev_attr_max6662_tmax);
	if (err)
	{
		goto err_no_sysfs_tmax;
	}
	err = device_create_file(&spi->dev, &dev_attr_max6662_tlow);
	if (err)
	{
		goto err_no_sysfs_tlow;
	}
	err = device_create_file(&spi->dev, &dev_attr_max6662_thigh);
	if (err)
	{
		goto err_no_sysfs_thigh;
	}
	err = device_create_file(&spi->dev, &dev_attr_max6662_baud);
	if (err)
	{
		goto err_no_sysfs_baud;
	}
	err = device_create_file(&spi->dev, &dev_attr_max6662_mode);
	if (err)
	{
		goto err_no_sysfs_mode;
	}
	
	err = device_create_file(&spi->dev, &dev_attr_max6662_3wire);
	if (err)
	{
		goto err_no_sysfs_3wire;
	}
	
	max6662_device = max6662;
	
	return 0;
	

err_no_sysfs_3wire:
	device_remove_file( &spi->dev, &dev_attr_max6662_mode);
err_no_sysfs_mode:
	device_remove_file( &spi->dev, &dev_attr_max6662_baud);
err_no_sysfs_baud:
	device_remove_file( &spi->dev, &dev_attr_max6662_thigh);
err_no_sysfs_thigh:
	device_remove_file( &spi->dev, &dev_attr_max6662_tlow);
err_no_sysfs_tlow:
	device_remove_file( &spi->dev, &dev_attr_max6662_tmax);
err_no_sysfs_tmax:
	device_remove_file( &spi->dev, &dev_attr_max6662_thyst);
err_no_sysfs_thyst:
	device_remove_file( &spi->dev, &dev_attr_max6662_config);
err_no_sysfs_config:
	device_remove_file( &spi->dev, &dev_attr_max6662_temp);
err_no_sysfs_temp:
#ifdef CONFIG_ARCH_PC302
	if (spi->dev.platform_data)
	{
		gpio_free((unsigned)(spi->dev.platform_data));
	}
err_request_max_cs:
#endif
	misc_deregister( &max6662->miscdev );
err_no_misc_device:
	kfree(max6662);
err_no_max6662_data:
	
	return err;
}

static int __devexit max6662_remove(struct spi_device *spi)
{
	struct max6662_data	*max6662;
	
	max6662 = dev_get_drvdata(&spi->dev);
	device_remove_file( &spi->dev, &dev_attr_max6662_3wire);
	device_remove_file( &spi->dev, &dev_attr_max6662_mode);
	device_remove_file( &spi->dev, &dev_attr_max6662_baud);
	device_remove_file( &spi->dev, &dev_attr_max6662_thigh);
	device_remove_file( &spi->dev, &dev_attr_max6662_tlow);
	device_remove_file( &spi->dev, &dev_attr_max6662_tmax);
	device_remove_file( &spi->dev, &dev_attr_max6662_thyst);
	device_remove_file( &spi->dev, &dev_attr_max6662_config);
	device_remove_file( &spi->dev, &dev_attr_max6662_temp);
#ifdef CONFIG_ARCH_PC302
	if (spi->dev.platform_data)
	{
		gpio_free((unsigned)(spi->dev.platform_data));
	}
#endif
	misc_deregister( &max6662->miscdev );
	kfree(max6662);
	max6662_device = NULL;
	
	return 0;
}

/*-------------------------------------------------------------------------*/

static struct spi_driver max6662_driver = {
	.driver = {
		.name		= "max6662",
		.owner		= THIS_MODULE,
	},
	.probe		= max6662_probe,
	.remove		= __devexit_p(max6662_remove),
};

static int __init max6662_init(void)
{
	return spi_register_driver(&max6662_driver);
}
module_init(max6662_init);

static void __exit max6662_exit(void)
{
	spi_unregister_driver(&max6662_driver);
}
module_exit(max6662_exit);

MODULE_DESCRIPTION("Driver for Maxim 6662");
MODULE_AUTHOR("ip.access Ltd");
MODULE_LICENSE("GPL");

