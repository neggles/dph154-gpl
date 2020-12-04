/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright 2005 Zetetica Ltd.  <david.warman@zetetica.co.uk>
 * Copyright (c) 2006 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file cpld_hdp.c
 * \brief HDP102 CPLD access
 *
 * picoHDP CPLD register access
 *     - provides interface for other device drivers, no direct access
 *       from user space at present.  Should attempt API compatibility
 *      with Airspan asmax/cpld driver.
 *
 */

#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/sysfs.h>

#include <asm/cpld_hdp102.h>

/** asmax version has Fan RPM control and FPGA control which do not
 *  exist on this target
 */

/* Exported functions */

EXPORT_SYMBOL(cpld_fpga_reset);
EXPORT_SYMBOL(cpld_fpga_reset_assert);
EXPORT_SYMBOL(cpld_fpga_reset_deassert);
EXPORT_SYMBOL(cpld_pico_reset_assert);
EXPORT_SYMBOL(cpld_pico_reset_deassert);
EXPORT_SYMBOL(cpld_reg_read);
EXPORT_SYMBOL(cpld_reg_write);
EXPORT_SYMBOL(cpld_dma_enable);
EXPORT_SYMBOL(cpld_dma_disable);
EXPORT_SYMBOL(cpld_dma_reset);


/* Dummy fan functions (no fan control on this target */
EXPORT_SYMBOL(cpld_fan_on);
EXPORT_SYMBOL(cpld_fan_off);

#define PRINTD(lvl, format, ...)  \
    do { \
        if(debug>=lvl)\
        { \
            printk(KERN_INFO PREFIX format, ##__VA_ARGS__); \
        } \
    } while(0)

#define PRINTE(format, ...)  \
    printk(KERN_ERR __FILE__ ":%d:" format, __LINE__, ##__VA_ARGS__)


/*******************************************************************************
Module Parameters

   The major device number is assigned dynamically, unless:
   1) CPLD_MAJOR is #defined non-zero in the Makefile
   2) "major" is set at module load time on the insmod command line
*******************************************************************************/
#ifndef CPLD_MAJOR
#define CPLD_MAJOR 248
#endif

#define CPLD_DEVNAME "picoHDP-cpld"

static int major = CPLD_MAJOR;
module_param(major, int, 0); /* Device major number */

/*** internal data structures ***/

typedef struct cpld
{
    struct miscdevice miscdev;           /* character device info */
    volatile u16* regs;                  /* pointer to the actual registers */
    u16 mirror[PICO_CPLD_REG_NREGS];  /* mirrors for bit-access */
} cpld_t;

/* private device structure for each open device instance */
typedef struct cpld_dev
{
    int minor;
} cpld_dev_t;

static int
cpld_open(struct inode *inode,
          struct file *filp);

static int
cpld_ioctl(struct inode *inode,
           struct file *filp,
           unsigned int cmd,
           unsigned long arg);

static ssize_t
cpld_read(struct file *filp,
          char *buf,
          size_t bufsize,
          loff_t *offset);

static ssize_t
cpld_write(struct file *filp,
           const char *buf,
           size_t bufsize,
           loff_t *offset);

static int
cpld_release(struct inode *inode,
             struct file *filp);

static struct file_operations cpld_fops =
{
    .open = cpld_open,
    .ioctl = cpld_ioctl,
    .read = cpld_read,
    .write = cpld_write,
    .release = cpld_release,
};

static cpld_t cpld =
{
    .miscdev =
    {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "cpld",
        .fops   = &cpld_fops,
    },
};

/* internal functions */
static int
request_and_map(const char* name,
                unsigned long address,
                int size,
                void** virt_addr);

static int
cpld_set_dma_map(int map_number)
{
    cpld_dma_disable(0);
    cpld_dma_disable(1);
    cpld_dma_disable(2);
    cpld_dma_disable(3);
    udelay(10);
    cpld_dma_reset(0);
    cpld_dma_reset(1);
    cpld_dma_reset(2);
    cpld_dma_reset(3);
    cpld_reg_write(PICO_CPLD_REG_DMA_SELECT, map_number);
    cpld_dma_reset(0);
    cpld_dma_reset(1);
    cpld_dma_reset(2);
    cpld_dma_reset(3);

    return 0;
}

static const char *mapping_names[] = {
    "ALL_PICOARRAY_0",
    "ALL_PICOARRAY_1",
    "SPLIT",
    "ALL_PICOARRAY_2",
    "ALL_PICOARRAY_3",
};

static ssize_t
cpld_show_dma_mapping(struct device *dev,
                      struct device_attribute *attr,
                      char *buf)
{
    int mapping = cpld_reg_read(PICO_CPLD_REG_DMA_SELECT);
    const char *name;

    /* Bottom 4 bits show the mapping. */
    mapping &= 0xF;

    if (mapping >= 0 && mapping < ARRAY_SIZE(mapping_names))
        name = mapping_names[mapping];
    else
        name = "INVALID";

    return sprintf(buf, "%s\n", name);
}

static ssize_t
cpld_store_dma_mapping(struct device *dev,
                       struct device_attribute *attr,
                       const char *buf,
                       size_t count)
{
    unsigned i = 0;
    int mapping = -1;
    size_t len = count;

    /* Strip the newline if there is one. */
    for (i = 0; i < count; ++i)
        if (buf[i] == '\n')
        {
            len = i;
            break;
        }

    for (i = 0; i < ARRAY_SIZE(mapping_names); ++i)
        if (!strncmp(mapping_names[i], buf, len))
        {
            mapping = i;
            break;
        }

    if (mapping >= 0)
        cpld_set_dma_map(mapping);
    return mapping >= 0 ? count : -EINVAL;
}
DEVICE_ATTR(dma_mapping, 0644, cpld_show_dma_mapping, cpld_store_dma_mapping);

static struct attribute *cpld_attrs[] = {
    &dev_attr_dma_mapping.attr,
    NULL,
};

static struct attribute_group cpld_attr_group = {
    .attrs = cpld_attrs,
};

static int
cpld_init(void)
{
    int err, i;
    cpld.regs = 0;    /* Ensure null on failure */

    /*** do char device setup ***/

    if (misc_register(&cpld.miscdev))
        return -EFAULT;

    err = request_and_map("picoHDP CPLD", PICOHDP_CPLD_BASE,
                          PICO_CPLD_REG_BLOCKSIZE, (void**)&(cpld.regs));
    if (err)
    {
        PRINTE("%s: cannot map, major=%d; err=%d\n",
               CPLD_DEVNAME, major, err);
    }
    else
    {
        /* Mapped ok */
        printk(KERN_INFO "cpld: picoChip HDP target CPLD driver\n");
    }

    /* initialise mirrors to zero */
    for (i = 0; i < PICO_CPLD_REG_NREGS; i++)
    {
        cpld.mirror[i] = 0;
    }

    err = sysfs_create_group(&cpld.miscdev.this_device->kobj,
                             &cpld_attr_group);
    if (err)
        goto fail;

    return err;

fail:
    misc_deregister(&cpld.miscdev);
    iounmap(cpld.regs);
    release_mem_region(PICOHDP_CPLD_BASE, PICO_CPLD_REG_BLOCKSIZE);

    return err;
}

static void
cpld_exit(void)
{
    sysfs_remove_group(&cpld.miscdev.this_device->kobj, &cpld_attr_group);
    /* unregister the char device */
    misc_deregister(&cpld.miscdev);

    if (cpld.regs != 0)
    {
        iounmap(cpld.regs);
        release_mem_region(PICOHDP_CPLD_BASE,
                           PICO_CPLD_REG_BLOCKSIZE);
    }
    printk(KERN_INFO "cpld: picoChip CPLD driver unloaded\n");
}

/* Dummy functions */
void
cpld_fpga_reset(int fpga)
{
}

void
cpld_fpga_reset_assert(int fpga)
{
}

void
cpld_fpga_reset_deassert(int fpga)
{
}

/* Only reset_assert is used; our reset is auto-clearing */
void
cpld_pico_reset_assert(int pico)
{
    u16 resetbit = 0;
    u16 regReadValue;

    switch(pico)
    {
        case 0:
            resetbit = PICO_CPLD_BIT_LEDRESET_PA0RESET;
            break;
        case 1:
            resetbit = PICO_CPLD_BIT_LEDRESET_PA1RESET;
            break;
#ifdef CONFIG_PICOHDP203
        case 2:
            resetbit = PICO_CPLD_BIT_LEDRESET_PA2RESET;
            break;
        case 3:
            resetbit = PICO_CPLD_BIT_LEDRESET_PA3RESET;
            break;
#endif
        default:
            printk(KERN_INFO "Warning: CPLD driver asked to reset "
                "picoChip %d, which does not exist\n", pico);
            break;
    }

    if (cpld.regs != 0)
    {
        /* Reset bits are active low, but read back as '1';
         * the reset returns to the inactive state automatically
         */
        /* cpld.regs[PICO_CPLD_REG_LEDRESET] &= resetbit; */

        /* For 'some reason' we need to do a couple of dummy reads from the
           cpld before we get 'real' data */

        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];

        cpld.regs[PICO_CPLD_REG_LEDRESET] = regReadValue & ~resetbit;

        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
    }
}

void
cpld_pico_reset_deassert(int pico)
{
    /* Reset is self-clearing on this target */
    /* not any more... */

    u16 resetbit = 0;
    u16 regReadValue;
    switch(pico)
    {
        case 0:
            resetbit = PICO_CPLD_BIT_LEDRESET_PA0RESET;
            break;
        case 1:
            resetbit = PICO_CPLD_BIT_LEDRESET_PA1RESET;
            break;
#ifdef CONFIG_PICOHDP203
        case 2:
            resetbit = PICO_CPLD_BIT_LEDRESET_PA2RESET;
            break;
        case 3:
            resetbit = PICO_CPLD_BIT_LEDRESET_PA3RESET;
            break;
#endif
        default:
            printk(KERN_INFO "Warning: CPLD driver asked to reset "
                "picoChip %d, which does not exist\n", pico);
            break;
    }
    if (cpld.regs != 0)
    {
        /* Reset bits are active low, but read back as '1';
         * the reset returns to the inactive state automatically
         */
        /* cpld.regs[PICO_CPLD_REG_LEDRESET] &= resetbit; */

        /* For 'some reason' we need to do a couple of dummy reads from the
           cpld before we get 'real' data */

        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];

        cpld.regs[PICO_CPLD_REG_LEDRESET] = regReadValue | resetbit;

        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
        regReadValue = cpld.regs[PICO_CPLD_REG_LEDRESET];
    }
}

/* Host CTRL Register - not implemented on this target, so dummy functions */
void
cpld_fan_on(int fan)
{
}

void
cpld_fan_off(int fan)
{
}

/* CPLD Version Register - not used at present */
u8
cpld_version(void)
{
    return 0;
}

/* Not used on this target */
u8
cpld_diagnostics(void)
{
    return 0;
}

/* Helper function for gaining access to a physical region */
static int
request_and_map(const char* name,
                unsigned long address,
                int size,
                void** virt_addr)
{
    int err = 0;
    struct resource *r;

    r = request_mem_region(address, size, name);
    if (r != NULL)
    {
        *virt_addr = ioremap(address, size);
        if (*virt_addr == NULL)
        {
            release_resource(r);
            err = -EIO;
        }
    }
    else
    {
        err = -EBUSY;
        printk(KERN_DEBUG "cpld: failed to allocate %s\n", name);
    }

    return err;
}

/*
 * CPLD char device File Ops methods
 */
static int
cpld_open(struct inode *inode,
          struct file *filp)
{
    int ret = 0;
    cpld_dev_t *dev;

    dev = (cpld_dev_t *)kzalloc(sizeof(cpld_dev_t), GFP_KERNEL);
    if (!dev)
    {
        ret = -ENOMEM;
    }
    else
    {
        dev->minor = iminor(inode);
        filp->private_data = dev;
    }
    return ret;
}

static int
cpld_ioctl(struct inode *inode,
           struct file *filp,
           unsigned int cmd,
           unsigned long arg)
{
    cpld_reg_param_t params;
    int ret = 0;

    /* check that the requested command is in range */
    if (_IOC_TYPE(cmd) != PICO_CPLD_IOC_MAGIC)
        return -ENOTTY;
    if (_IOC_NR(cmd) > PICO_CPLD_IOC_MAXNUM)
        return -ENOTTY;

    switch (cmd)
    {
        case PICO_CPLD_IOCWRITEREG :
        {
            if (copy_from_user(&params, (void *)arg, sizeof(params)))
            {
                ret = -EFAULT;
                break;
            }
            if (params.reg >= PICO_CPLD_REG_NREGS)
            {
                PRINTE("invalid register %d",
                    params.reg);
                ret = -EINVAL;
                break;
            }
            cpld.regs[params.reg] = params.data;
            break;
        }
        case PICO_CPLD_IOCREADREG :
        {
            if (copy_from_user(&params, (void *)arg, sizeof(params)))
            {
                ret = -EFAULT;
                break;
            }
            if (params.reg >= PICO_CPLD_REG_NREGS)
            {
                PRINTE("invalid register %d",
                    params.reg);
                ret = -EINVAL;
                break;
            }
            params.data = cpld_reg_read(params.reg);
            if (copy_to_user((void *)arg, &params, sizeof(params)))
                ret = -EFAULT;
            break;
        }
        default:
            PRINTE("invalid ioctl command 0x%08X", cmd);
            return -ENOTTY;
    }
    return ret;
}

/*
 * Wrapper function for CPLD register reads.
 */
u16
cpld_reg_read(unsigned char regno)
{
    if (regno >= PICO_CPLD_REG_NREGS)
    {
        PRINTE("invalid register %d", regno);
        return 0;
    }
    return cpld.regs[regno];
}

/*
 * Wrapper function for CPLD register writes.
 */
void
cpld_reg_write(unsigned char regno, u16 data)
{
    if (regno >= PICO_CPLD_REG_NREGS)
    {
        PRINTE("invalid register %d", regno);
        return;
    }
    cpld.mirror[regno] = data;
    cpld.regs[regno] = cpld.mirror[regno];
}

/*
 * Write a value to a register using a bitmask. An internal mirror maintains
 * the state of the un-altered bits.
 */
void
cpld_bit_write(unsigned char regno, u16 data, u16 mask)
{
    if (regno >= PICO_CPLD_REG_NREGS)
    {
        PRINTE("invalid register %d", regno);
        return;
    }
    cpld.mirror[regno] &= ~mask;
    cpld.mirror[regno] |= (data & mask);
    cpld.regs[regno] = cpld.mirror[regno];
    wmb();
}

/*
 * Enable a DMA state machine. The DMA state machines have been added
 * to get around a problem with the MPC8560's DMAC whereby external start/
 * pause mode will prevent software control or initiation of transfers
 * and will not otherwise stop at the end of a descriptor list.
 * CPLD VHDL mod credits to Duncan Smith at PicoChip.
 */
void
cpld_dma_enable(unsigned char chan)
{
    if (chan >= PICO_CPLD_DMA_NCHANS)
    {
        PRINTE("invalid channel number %d", chan);
        return;
    }

    /* set enable bit, clear reset bit */
    cpld_bit_write(PICO_CPLD_REG_DREQCTRL, 0xf0, 0x11 << chan );
}

/*
 * Disable a DMA state machine.
 */
void
cpld_dma_disable(unsigned char chan)
{
    if (chan >= PICO_CPLD_DMA_NCHANS)
    {
        PRINTE("invalid channel number %d", chan);
        return;
    }

    /* clear enable and reset bits */
    cpld_bit_write(PICO_CPLD_REG_DREQCTRL, 0x00, 0x11 << chan );
}

/*
 * Reset the DMA state machine in the CPLD.
 */
void
cpld_dma_reset(unsigned char chan)
{
    if (chan >= PICO_CPLD_DMA_NCHANS)
    {
        PRINTE("invalid channel number %d", chan);
        return;
    }

    /* set then clear the reset bit to start a DMA */
    cpld_bit_write(PICO_CPLD_REG_DREQCTRL, 0xff, 0x01 << chan );
    cpld_bit_write(PICO_CPLD_REG_DREQCTRL, 0x00, 0x01 << chan );
}

/*
 * fops read method (currently unimplemented)
 */
static
ssize_t cpld_read(struct file *filp,
                  char *buf,
                  size_t bufsize,
                  loff_t *offset)
{
    return 0;
}

/*
 * fops write method (currently unimplemented)
 */
static
ssize_t cpld_write(struct file *filp,
                   const char *buf,
                   size_t bufsize,
                   loff_t *offset)
{
    return 0;
}

/*
 * fops release (close) method
 */
static int
cpld_release(struct inode *inode,
             struct file *filp)
{
    cpld_dev_t *dev = filp->private_data;

    kfree(dev);
    return 0;
}

/*
 * Module details
 */
MODULE_AUTHOR("David Warman <david.warman@zetetica.co.uk>, with picoChip mods");
MODULE_DESCRIPTION("picoChip CPLD driver.");
MODULE_LICENSE("GPL");

module_init(cpld_init);
module_exit(cpld_exit);

