/*
 * picoHDP flash mappings (128M version)
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * David Warman <david.warman@zetetica.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 29082006 - Ben Tucker, ported to the pc20x.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

/* PC20x based platforms */
#if defined(CONFIG_MACH_PC72052_I10_REVA) || \
    defined(CONFIG_MACH_PC72052_I10_REVB) || \
    defined(CONFIG_MACH_FIRECRACKER_SVB)  || \
    defined(CONFIG_MACH_PC7802)

#include <asm/sizes.h>
#include <mach/platform.h>

/* Partitioning used on the PC20x based platforms (SVB20x/CPE20x) */
#define FLASH_BASE          FIRECRACKER_FLASH_BASE
#define FLASH_SIZE          FIRECRACKER_FLASH_SIZE
#define FLASH_WIDTH         FIRECRACKER_FLASH_WIDTH

/* Bootloader at start */
#define FLASH_ADDR_BOOT     FLASH_BASE
#define FLASH_SIZE_BOOT     SZ_256K

/* Bootloader environment follows bootloader */
#define FLASH_ADDR_BOOTENV  (FLASH_ADDR_BOOT + FLASH_SIZE_BOOT)
#define FLASH_SIZE_BOOTENV  SZ_256K

/* Kernel follows... */
#define FLASH_ADDR_KERNEL   (FLASH_ADDR_BOOTENV + FLASH_SIZE_BOOTENV)
#define FLASH_SIZE_KERNEL   SZ_2M

/* Filesystem takes up the rest of the flash */
#define FLASH_ADDR_JFFS2    (FLASH_ADDR_KERNEL + FLASH_SIZE_KERNEL)
#define FLASH_SIZE_JFFS2    MTDPART_SIZ_FULL    /* The rest of the available
                                                   flash (~125.5 Mbytes). */
#define FLASH_PARTITIONS    4

/* PC302 based platforms */
#elif defined(CONFIG_MACH_PC7302)

#include <asm/sizes.h>
#include <mach/platform.h>

/* Partitioning used on the PC302 based platforms (SVB302/PC7302) */
#define FLASH_BASE          PC302_FLASH_BASE
#define FLASH_SIZE          PC302_FLASH_SIZE
#define FLASH_WIDTH         PC302_FLASH_WIDTH

/* Bootloader at start */
#define FLASH_ADDR_BOOT     FLASH_BASE
#define FLASH_SIZE_BOOT     SZ_128K

/* Bootloader environment follows bootloader */
#define FLASH_ADDR_BOOTENV  (FLASH_ADDR_BOOT + FLASH_SIZE_BOOT)
#define FLASH_SIZE_BOOTENV  SZ_128K

/* Kernel follows... */
#define FLASH_ADDR_KERNEL   (FLASH_ADDR_BOOTENV + FLASH_SIZE_BOOTENV)
#define FLASH_SIZE_KERNEL   SZ_2M

/* Filesystem takes up the rest of the flash */
#define FLASH_ADDR_JFFS2    (FLASH_ADDR_KERNEL + FLASH_SIZE_KERNEL)
#define FLASH_SIZE_JFFS2    MTDPART_SIZ_FULL    /* The rest of the available
                                                   flash (~62.75 Mbytes). */
#define FLASH_PARTITIONS    4

#else

/* Partitioning used on the PPC based platforms (HDP102/HDP203) */

#define FLASH_BASE          0xF8000000
#define FLASH_SIZE          0x08000000
#define FLASH_WIDTH         4

#define FLASH_ADDR_JFFS2    0xF8000000
#define FLASH_SIZE_JFFS2    0x07E00000

#define FLASH_ADDR_KERNEL   0xFFE00000
#define FLASH_SIZE_KERNEL   0x00180000

#define FLASH_ADDR_BOOTENV  0xFFF80000
#define FLASH_SIZE_BOOTENV  0x00040000

#define FLASH_ADDR_BOOT     0xFFFC0000
#define FLASH_SIZE_BOOT     0x00040000

#define FLASH_PARTITIONS    4

#endif

/* partition_info gives details on the logical partitions that the split the
 * single flash device into.
 */
static struct mtd_partition partition_info[]={
    {
        .name   = "Application",
        .offset = FLASH_ADDR_JFFS2 - FLASH_BASE,
        .size   = FLASH_SIZE_JFFS2
    },
    {
        .name   = "Kernel",
        .offset = FLASH_ADDR_KERNEL - FLASH_BASE,
        .size   = FLASH_SIZE_KERNEL
    },
    {
        .name   = "Boot Environment",
        .offset = FLASH_ADDR_BOOTENV - FLASH_BASE,
        .size   = FLASH_SIZE_BOOTENV
    },
    {
        .name   = "Boot",
        .offset = FLASH_ADDR_BOOT - FLASH_BASE,
        .size   = FLASH_SIZE_BOOT
    }
};


static struct mtd_info *mymtd;

struct map_info picohdp_map = {
    .name = "picoFlash",
    .size = FLASH_SIZE,
    .phys = FLASH_BASE,
    .bankwidth = FLASH_WIDTH,
};

int __init init_picohdp(void)
{
    printk(KERN_NOTICE "picoFlash: 0x%x at 0x%x\n",
            FLASH_SIZE, FLASH_BASE);
    picohdp_map.virt = ioremap(FLASH_BASE, FLASH_SIZE);

    if (!picohdp_map.virt) {
        printk("Failed to ioremap\n");
        return -EIO;
    }
    simple_map_init(&picohdp_map);

    mymtd = do_map_probe("cfi_probe", &picohdp_map);
    if (mymtd) {
        mymtd->owner = THIS_MODULE;
        add_mtd_device(mymtd);
        add_mtd_partitions(mymtd, partition_info, FLASH_PARTITIONS);
        return 0;
    }

    iounmap((void *)picohdp_map.virt);
    return -ENXIO;
}

static void __exit cleanup_picohdp(void)
{
    if (mymtd) {
        del_mtd_device(mymtd);
        map_destroy(mymtd);
    }
    if (picohdp_map.virt) {
        iounmap((void *)picohdp_map.virt);
        picohdp_map.virt = 0;
    }
}

module_init(init_picohdp);
module_exit(cleanup_picohdp);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Warman <david.warman@zetetica.co.uk>");
MODULE_DESCRIPTION("MTD map for picoChip development platforms");
