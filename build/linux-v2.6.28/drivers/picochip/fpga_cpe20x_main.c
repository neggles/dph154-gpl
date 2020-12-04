/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file fpga_cpe20x_main.c
 * \brief SPI connected FPGA device driver
 *
 * CPLD access functions
 *
 */

/*****************************************************************************
 * Includes
 *****************************************************************************/

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/picochip/fpga_cpe20x.h>
#include <mach/reset.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <asm/mach-types.h>

#include "cpld_cpe20x.h"

/*****************************************************************************
 * Magic numbers
 *****************************************************************************/

/* definition of success */
#define SUCCESS     (0)

/* name by which the module is known */
#define MODNAME     "fpga_cpe20x"

/* our major number for the device node */
#define FPGA_MAJOR   (248)

/* our minor number for the device node */
#define FPGA_MINOR   (0)

/* device name by which we are known under /proc/devices, /sys, etc */
#define FPGA_DEVNAME "cpld"

/* SPI bits_per_word */
#define FPGA_SPI_ACCESS_WIDTH 16

/* define ENABLE_TEST to enable testing of FPGA access by writing and
 * reading back the contents of the test register during probe.
 */
#define ENABLE_TEST

/*****************************************************************************
 * Macros
 *****************************************************************************/

/*
 * Debugging macros.
 */

#define PRINTD(lvl, format, ...)  \
    do \
    { \
        if(debug>=lvl) \
        { \
            printk(KERN_INFO PREFIX format, ##__VA_ARGS__);\
        }\
    } while(0)

#define PRINTE(format, ...) \
    printk(KERN_ERR __FILE__ ":%d:" format, __LINE__, ##__VA_ARGS__)

/*****************************************************************************
 * Data structure & types
 *****************************************************************************/

/* A parameter to switch in the test code */
static int regtest = 0;
module_param(regtest, int, S_IRUGO);
MODULE_PARM_DESC(regtest,
    "Set to '1' or '0' to 'enable' or 'disable' the fpga register test");

/* Driver state data */
struct fpga
{
    struct spi_device   *spi;
    struct fpga_platform_data *pdata;
};

/* private device structure for each open device instance */
typedef struct cpld_dev
{
    int minor;
}
cpld_dev_t;

/*****************************************************************************
 * Private function prototypes
 *****************************************************************************/

/* file operations */
static int
fpga_fops_open(struct inode *inode,
               struct file *filp);

static int
fpga_fops_release(struct inode *inode,
                  struct file *filp);

static ssize_t
fpga_fops_read(struct file *filp,
               char __user *buf,
               size_t bufsize,
               loff_t *offset);

static ssize_t
fpga_fops_write(struct file *filp,
                const char __user *buf,
                size_t bufsize,
                loff_t *offset);

static int
fpga_fops_ioctl(struct inode *inode,
                struct file *filp,
                unsigned int cmd,
                unsigned long arg);

/*****************************************************************************
 * Global variables & module parameters
 *****************************************************************************/

static struct file_operations fpga_fops =
{
    .owner      = THIS_MODULE,
    .open       = fpga_fops_open,
    .release    = fpga_fops_release,
    .read       = fpga_fops_read,
    .write      = fpga_fops_write,
    .ioctl      = fpga_fops_ioctl,
};

static struct miscdevice miscdev =
{
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "fpga",
    .fops   = &fpga_fops,
};

/*****************************************************************************
 * Private function declarations
 *****************************************************************************/

/* find_fpga is used with bus_for_each_dev to find the SPI connected device
 * that has a particular FPGA ID.
 */
static int
find_fpga(struct device *device,
          void *data)
{
    struct fpga *fpga;

    struct spi_device *sdev = to_spi_device(device);

    unsigned int fpga_device_id = *(unsigned int *)data;

    if (!strcmp(sdev->modalias, MODNAME))
    {
        fpga = dev_get_drvdata(&sdev->dev);

        if (fpga->pdata->fpga_device_id == fpga_device_id)
        {
            return (int)fpga;
        }
    }
    return 0;
}

/* Return a handle to the FPGA device that has the device ID, or null
 * if not found. The handle returned should be used with the read and
 * write functions.
 */
void
*fpga_get_handle(unsigned int fpga_device_id)
{
    return (struct fpga *)bus_for_each_dev(
                &spi_bus_type, NULL, &fpga_device_id, find_fpga);
}


/* Read one of the FPGA registers by assembling an SPI transfer sequence
 * that does the read.
 *
 * From spi_sync, used to do the SPI transfer:
 *
 * This call may only be used from a context that may sleep.  The sleep
 * is non-interruptible, and has no timeout.
 */
static u16 read_reg(struct spi_device *spi, unsigned int reg_addr)
{
    struct spi_message msg;
    struct spi_transfer xaddr, xdata;
    u16 addr_cmd, reg;
    int err;

    /* Reset local structures */
    memset(&xaddr, 0, sizeof(struct spi_transfer));
    memset(&xdata, 0, sizeof(struct spi_transfer));

    /* Setup a transfer to read the device ID from the FPGA and check it
     * against out platform data.
     */
    spi_message_init(&msg);

    /* Transmit the register address and command */
    addr_cmd =
        ((reg_addr << FPGA_SPI_ADDR_SHIFT) & FPGA_SPI_ADDR_MASK) |
        FPGA_SPI_CMD_R_NW;
    xaddr.tx_buf = &addr_cmd;
    xaddr.len = 2;     /* Two bytes */

    /* Set the next transfer to receive the register contents */
    xdata.rx_buf = &reg;
    xdata.len = 2;     /* Two bytes */

    spi_message_add_tail(&xaddr, &msg);
    spi_message_add_tail(&xdata, &msg);

    /* Do the register read. Will wait for completion */
    err = spi_sync(spi, &msg);
    if (err != 0)
    {
        printk(KERN_DEBUG "FPGA read failed\n");
        return 0;
    }

    return reg;
}

/* Write one of the FPGA registers by assembling an SPI transfer sequence
 * that does the write.
 *
 * From spi_sync, used to do the SPI transfer:
 *
 * This call may only be used from a context that may sleep.  The sleep
 * is non-interruptible, and has no timeout.
 */
static void
 write_reg(struct spi_device *spi,
           unsigned int reg_addr, u16 value)
{
    struct spi_message msg;
    struct spi_transfer xaddr, xdata;
    u16 addr_cmd;
    int err;

    /* Reset local structures */
    memset(&xaddr, 0, sizeof(struct spi_transfer));
    memset(&xdata, 0, sizeof(struct spi_transfer));

    /* Setup a transfer to read the device ID from the FPGA and check it
     * against out platform data.
     */
    spi_message_init(&msg);

    /* Transmit the register address and command */
    addr_cmd = (reg_addr << FPGA_SPI_ADDR_SHIFT) & FPGA_SPI_ADDR_MASK;
    xaddr.tx_buf = &addr_cmd;
    xaddr.len = 2;     /* Two bytes */

    /* Set the next transfer to transmit the register contents */
    xdata.tx_buf = &value;
    xdata.len = 2;     /* Two bytes */

    spi_message_add_tail(&xaddr, &msg);
    spi_message_add_tail(&xdata, &msg);

    /* Do the register write. Will wait for completion */
    err = spi_sync(spi, &msg);
    if (err != 0)
    {
        printk(KERN_DEBUG "FPGA write failed\n");
    }
}

#ifdef ENABLE_TEST
/* Test functionality that reads all the registers and prints them out. Also
 * writes the test register and checks that it reads the same value back.
 */
static void
test_fpga(struct spi_device *spi,
          u16 id_reg)
{
    int i;
    char opstr[100];
    char *p;
    u16 reg;

    /* read/write the test register */
    for (i = 0; i < 16; ++i)
    {
        write_reg(spi, FPGA_TEST_REG, (1 << i));
        reg = read_reg(spi, FPGA_TEST_REG);
        if ((1 << i) != reg)
        {
            printk("FPGA ID 0x%04x read/write test failed, wrote 0x%04x, "
                   "read 0x%04x. Aborting test.\n", id_reg, (1 << i), reg);
            return;
        }
    }

    memset(opstr, 0, 100);

    /* Read registers 0 to 80 */
    printk("FPGA ID 0x%04x Registers:\n", id_reg);
    for (i = 0; i <= 80; ++i)
    {
        if (i % 8 == 0)
        {
            if (i > 0)
            {
                printk("%s\n", opstr);
                memset(opstr, 0, 100);
            }
            p = opstr;
            p += sprintf(p, "0x%02x:", i);
        }

        reg = read_reg(spi, i);
        p += sprintf(p, " 0x%04x", reg);
    }
}
#endif

/* API function to read an FPGA register using above function */
u16 fpga_read_reg(void *handle, unsigned int reg_addr)
{
    struct fpga *fpga = (struct fpga *)handle;

    return read_reg(fpga->spi, reg_addr);
}

/* API function to write an FPGA register using above function */
void
fpga_write_reg(void *handle,
               unsigned int reg_addr,
               u16 value)
{
    struct fpga *fpga = (struct fpga *)handle;

    write_reg(fpga->spi, reg_addr, value);
}

/* A function that we pass to the kernel so that it can do a hard reset */
static void
fpga_board_reset(char mode, void *cookie)
{
    struct fpga *fpga = (struct fpga *)cookie;
    unsigned i;

    /* RESET the board!! */
    for ( i = 0; i < 10; ++i )
    {
        if (machine_is_pc72052_i10_revb())
        {
            write_reg(fpga->spi, FPGA_PICOARRAY_RESET_REG, 0);
        }
        else if (machine_is_pc7802())
        {
            write_reg(fpga->spi, FPGA_PICOARRAY_RESET_REG, 1);
        }
        else
        {
            /* Do Nothing */
        }
    }

    /* We shouldn't get here. If we do then the FPGA has failed to reset the
     * board. There is nothing we can do so lets make the failure known to the
     * wider world. */
    printk( KERN_EMERG "fpga reset failed after 10 attempts\n" );
    BUG();
}

/* Probe function for the FPGA driver. This checks the device ID in the
 * FPGA device with the platform data and succeeds the probe if there
 * is a match. If no FPGA device is present, the SPI driver will read zero
 * for all registers and the device ID match will fail (providing we do
 * not use device ID zero!).
 */
static int
fpga_probe(struct spi_device *spi)
{
    struct fpga_platform_data *pdata = spi->dev.platform_data;
    struct fpga *fpga;
    u16 id_reg;
    int err = 0;

    /* Set the SPI access width for our device */
    spi->bits_per_word = FPGA_SPI_ACCESS_WIDTH;

    /* Set the SPI mode */
    spi->mode = SPI_MODE_2;

    /* Set the SPI mode for this device */
    spi_setup(spi);

    /* Read the FPGA Device ID register */
    id_reg = read_reg(spi, FPGA_TYPE_ID_REG);

    /* Test the FPGA register access */
#ifdef ENABLE_TEST
    if (regtest)
    {
        test_fpga(spi, id_reg);
    }
#endif

    /* check the device id */
    if (pdata->fpga_device_id !=
            (id_reg & FPGA_DEVICE_ID_MASK) >> FPGA_DEVICE_ID_SHIFT)
    {
        /* Mismatch. Fail probe */
        err = -ENODEV;
        /* Indicate to the driver not finding the device */
        pdata->fpga_device_id = ~(pdata->fpga_device_id);
    }

    /* Allocate our local context */
    fpga = kzalloc(sizeof(struct fpga), GFP_KERNEL);
    if (!fpga)
    {
        err = -ENOMEM;
        goto err_exit;
    }

    /* Set the device state context */
    fpga->spi = spi;
    fpga->pdata = pdata;

    /* Associate our local context with the device */
    dev_set_drvdata(&spi->dev, fpga);

    if (err == 0)
    {
        printk("Found " MODNAME " device id %u\n", pdata->fpga_device_id);

        /* Register a reset handler. The FPGA is able to reset the board
        * and we need to tell the kernel.
        */
        register_reset_handler(fpga_board_reset, fpga);

        return 0;
    }

 err_exit:
    return err;
}

/* Remove the FPGA device */
static int
fpga_remove(struct spi_device *spi)
{
    struct fpga *fpga = dev_get_drvdata(&spi->dev);

    /* De-register the reset handler */
    deregister_reset_handler(fpga_board_reset, fpga);

    /* Free out context */
    kfree(fpga);

    printk("Unregistered " MODNAME " device\n");
    return 0;
}

/* Driver declaration structure for this FPGA driver */
static struct spi_driver fpga_driver =
{
    .driver =
    {
        .name   = MODNAME,
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },
    .probe      = fpga_probe,
    .remove     = fpga_remove,
};

/* Module initialisation, declare the FPGA driver */
static int
__init fpga_module_init(void)
{
    int returnCode;

    if (misc_register(&miscdev))
        return -EFAULT;

    /* Register the driver in spi land */
    returnCode = spi_register_driver(&fpga_driver);

    printk(KERN_INFO "%s: fpga device driver " __DATE__ " "
          __TIME__ "\n", MODNAME);

    return returnCode;
}

/* Module exit, remove the FPGA driver */
static void
__exit fpga_module_exit(void)
{
    misc_deregister(&miscdev);

    spi_unregister_driver(&fpga_driver);

    printk(KERN_INFO "%s: fpga device device driver module removed\n", MODNAME);
}

/*****************************************************************************
 * Character device file operations (fops) functions.
 *****************************************************************************/

static int
fpga_fops_open(struct inode *inode,
               struct file *filp)
{
    int ret = SUCCESS;

    cpld_dev_t *dev;

    dev = (cpld_dev_t *)kzalloc(sizeof(cpld_dev_t), GFP_KERNEL);
    if (!dev)
    {
        ret = (-ENOMEM);
    }
    else
    {
        dev->minor = iminor(inode);
        filp->private_data = dev;
    }

    return ret;
}

static int
fpga_fops_release(struct inode *inode,
                  struct file *filp)
{
    int ret = SUCCESS;

    cpld_dev_t *dev = filp->private_data;

    kfree(dev);

    return ret;
}

/*
 * fops read method - currently unimplemented
 */
static ssize_t
fpga_fops_read(struct file *filp,
               char __user *buf,
               size_t bufsize,
               loff_t *offset)
{
    int ret = SUCCESS;
    return ret;
}

/*
 * fops write method - currently unimplemented
 */
static ssize_t
fpga_fops_write(struct file *filp,
                const char __user *buf,
                size_t bufsize,
                loff_t *offset)
{
    int ret = SUCCESS;

    return ret;
}

/*
 * Currently, ioctl() is the only method we support.
 */
static int
fpga_fops_ioctl(struct inode *inode,
                struct file *filp,
                unsigned int cmd,
                unsigned long arg)
{
    int ret = SUCCESS;
    struct fpga *fpgaHandle;

    cpld_reg_param_t params;

    /* Check that the requested command is in range */
    if (_IOC_TYPE(cmd) != PICO_CPLD_IOC_MAGIC)
    {
        return -ENOTTY;
    }

    if (_IOC_NR(cmd) > PICO_CPLD_IOC_MAXNUM)
    {
        return -ENOTTY;
    }

    switch (cmd)
    {
        case PICO_CPLD_IOCWRITEREG :
        {
            if (copy_from_user(&params, (const void __user *)arg,
                               sizeof(params)))
            {
                ret = -EFAULT;
                break;
            }

            if (params.device >= PICO_CPLD_MAX_NUM_DEV)
            {
                PRINTE("invalid device %d\n", params.device);
                ret = -ENODEV;
                break;
            }

            /* In user space the AD_FPGA is designated as device '0'
               in fpga driver land the AD_FPGA is device '2' */
            /* In user space the RC_FPGA is designated as device '1'
               in fpga driver land the RC_FPGA is device '1' */
            /* Therefore some translation is required...*/
            if (params.device == 0)
            {
                /* The user wants to access the AD_FPGA */
                params.device = 2;
            }

            if (params.reg >= PICO_CPLD_REG_NREGS)
            {
                PRINTE("invalid register %d\n", params.reg);
                ret = -EINVAL;
                break;
            }

            /* obtain a handle for the specified device */
            fpgaHandle = fpga_get_handle((unsigned int)params.device);
            if (!fpgaHandle)
            {
                ret = -ENODEV;
                break;
            }
            /* do the write */
            fpga_write_reg(fpgaHandle, (unsigned int)params.reg, params.data);
            break;
        }

        case PICO_CPLD_IOCREADREG :
        {
            if (copy_from_user(&params, (const void __user *)arg,
                               sizeof(params)))
            {
                ret = -EFAULT;
                break;
            }

            if (params.device >= PICO_CPLD_MAX_NUM_DEV)
            {
                PRINTE("invalid device %d\n", params.device);
                ret = -ENODEV;
                break;
            }

            /* In user space the AD_FPGA is designated as device '0'
               in fpga driver land the AD_FPGA is device '2' */
            /* In user space the RC_FPGA is designated as device '1'
               in fpga driver land the RC_FPGA is device '1' */
            /* Therefore some translation is required...*/
            if (params.device == 0)
            {
                /* The user wants to access the AD_FPGA */
                params.device = 2;
            }

            if (params.reg >= PICO_CPLD_REG_NREGS)
            {
                PRINTE("invalid register %d\n", params.reg);
                ret = -EINVAL;
                break;
            }

            /* obtain a handle for the specified device */
            fpgaHandle = fpga_get_handle((unsigned int)params.device);
            if (!fpgaHandle)
            {
                ret = -ENODEV;
                break;
            }
            /* do the read */
            params.data = fpga_read_reg(fpgaHandle, params.reg);

            if (copy_to_user((void __user *)arg, &params, sizeof(params)))
            {
                ret = -EFAULT;
            }
            break;
        }

        default :
        {
            PRINTE("invalid ioctl command 0x%08X\n", cmd);
            return -ENOTTY;
        }
    }

    return ret;
}

/*
 * Things we need to make known to the world at large.
 */
module_init(fpga_module_init);
module_exit(fpga_module_exit);
EXPORT_SYMBOL(fpga_get_handle);
EXPORT_SYMBOL(fpga_read_reg);
EXPORT_SYMBOL(fpga_write_reg);

MODULE_AUTHOR("picoChip");
MODULE_DESCRIPTION("picoChip FPGA Driver");
MODULE_LICENSE("GPL");

