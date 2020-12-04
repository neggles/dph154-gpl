/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file debug.c
 * \brief picoIf debug implementation.
 *
 * This file implements functions for reading and writing debug messages
 * through debugfs.
 */

#include <linux/module.h>
#include <stdarg.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/configfs.h>
#include <linux/kfifo.h>
#include <asm/uaccess.h>
#include "debug.h"

#ifdef CONFIG_DEBUG_FS

/*! The maximum number of bytes that may be stored in the ring buffer. If the
 *  ring fills, the oldest data will be lost first. */
#define DEBUG_RING_SIZE  32768

/*! The maximum length of a debug message. Messages longer than this will be
 *  truncated. */
#define MAX_MSG_LEN     256

/*! The current picoIf debug level. */
static u8 picoif_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current PC202 debug level. */
static u8 pc202_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current PC203 debug level. */
static u8 pc203_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current PC302 debug level. */
static u8 pc302_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current procif debug level. */
static u8 procif_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current AXI2Cfg debug level. */
static u8 axi2cfg_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current GPR interrupt transport module debug level. */
static u8 gpr_int_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current DMA transport module debug level. */
static u8 dma_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current HwIf transport module debug level. */
static u8 hwif_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current HwIf2 transport module debug level. */
static u8 hwif2_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;
/*! The current debug module debug level. */
static u8 debug_dbg_lvl = CONFIG_PICOIF_DEFAULT_DEBUG_LEVEL;

/*! picoIf debug component. */
struct dbg_component picoif_dbg_component = {
    .curr_lvl   = &picoif_dbg_lvl,
    .name       = "picoif",
};

/*! PC202 debug component. */
struct dbg_component pc202_dbg_component = {
    .curr_lvl   = &pc202_dbg_lvl,
    .name       = "pc202",
};

/*! PC203 debug component. */
struct dbg_component pc203_dbg_component = {
    .curr_lvl   = &pc203_dbg_lvl,
    .name       = "pc203",
};

/*! PC302 debug component. */
struct dbg_component pc302_dbg_component = {
    .curr_lvl   = &pc302_dbg_lvl,
    .name       = "pc302",
};

/*! Procif debug component. */
struct dbg_component procif_dbg_component = {
    .curr_lvl   = &procif_dbg_lvl,
    .name       = "procif",
};

/*! AXI2Cfg debug component. */
struct dbg_component axi2cfg_dbg_component = {
    .curr_lvl   = &axi2cfg_dbg_lvl,
    .name       = "axi2cfg",
};

/*! GPR interrupt debug component. */
struct dbg_component gpr_int_dbg_component = {
    .curr_lvl   = &gpr_int_dbg_lvl,
    .name       = "gpr_int",
};

/*! DMA debug component. */
struct dbg_component dma_dbg_component = {
    .curr_lvl   = &dma_dbg_lvl,
    .name       = "dma",
};

/*! HwIF debug component. */
struct dbg_component hwif_dbg_component = {
    .curr_lvl   = &hwif_dbg_lvl,
    .name       = "hwif",
};

/*! HwIF2 debug component. */
struct dbg_component hwif2_dbg_component = {
    .curr_lvl   = &hwif2_dbg_lvl,
    .name       = "hwif2",
};

/*! Debug module debug component. */
struct dbg_component debug_dbg_component = {
    .curr_lvl   = &debug_dbg_lvl,
    .name       = "debug",
};

/*! The debug ring itself. */
static struct kfifo *debug_ring;

/*! The spinlock protecting the ring. */
static spinlock_t debug_ring_lock;

/*! Temorary structure for storing messages before writing to the ring buffer.
 */
static char msg_buf[ MAX_MSG_LEN ];

/*!
 * Move the len bytes of msg_buf into the debug ring.
 *
 * @param len The number of bytes to move into the debug ring. If there isn't
 * enough space, then discard some old data to make sure it fits.
 * @return Returns the number of bytes put into the debug ring.
 */
ssize_t
static __pc_debug_put( size_t len )
{
    size_t space;
    ssize_t ret;

    space = DEBUG_RING_SIZE - __kfifo_len( debug_ring );

    /* If there isn't enough space in the fifo for the message, then discard
     * the oldest data so that we can fit the latest message in. */
    if ( len > space )
        debug_ring->in += ( space - len );

    ret = __kfifo_put( debug_ring, msg_buf, len );

    return ret;
}

ssize_t
pc_debug_print( const char *component,
                const char *fmt,
                ... )
{
    va_list args;
    int len;
    int added;
    unsigned long flags;

    spin_lock_irqsave( debug_ring->lock, flags );

    len = sprintf( msg_buf, "[%s] ", component );
    added = __pc_debug_put( len );
#ifdef CONFIG_PICOIF_DEBUG_TO_CONSOLE
    printk( msg_buf );
#endif /* CONFIG_PICOIF_DEBUG_TO_CONSOLE */

    va_start( args, fmt );
    len = vsnprintf( msg_buf, sizeof( msg_buf ), fmt, args );
#ifdef CONFIG_PICOIF_DEBUG_TO_CONSOLE
    printk( msg_buf );
#endif /* CONFIG_PICOIF_DEBUG_TO_CONSOLE */
    va_end( args );
    added += __pc_debug_put( len );

    spin_unlock_irqrestore( debug_ring->lock, flags );

    return added;
}
EXPORT_SYMBOL( pc_debug_print );

/*!
 * Dump the debug ring to the console.
 */
void
debug_dump( void )
{
    char tmp;
    unsigned long flags;

    spin_lock_irqsave( debug_ring->lock, flags );
    while ( kfifo_get( debug_ring, &tmp, 1 ) )
        printk( "%c", tmp );
    printk( "\n" );
    spin_unlock_irqrestore( debug_ring->lock, flags );
}
EXPORT_SYMBOL( debug_dump );

/*!
 * Open method for the debug ring file.
 *
 * @param inode The inode of the debugfs log entry.
 * @param filp File structure for the opened instance.
 * @return Always returns zero.
 */
static int
debug_log_open( struct inode *inode,
                struct file *filp )
{
    return 0;
}

/*!
 * Read from the debug ring.
 *
 * @param filp The file structure for the instance.
 * @param buf The buffer to write the debug messages into.
 * @param count The maximum number of bytes to read.
 * @param offp The offset in the file.
 * @return Returns the number of bytes read from the ring.
 */
static ssize_t
debug_log_read( struct file *filp,
                char __user *buf,
                size_t count,
                loff_t *offp )
{
    char *tmp_buf = kmalloc( count, GFP_KERNEL );
    int ret = -ENOMEM;
    unsigned nbytes;
    if ( !tmp_buf )
        goto out;

    nbytes = kfifo_get( debug_ring, tmp_buf, count );
    ret = copy_to_user( buf, tmp_buf, nbytes );
    if ( !ret )
        ret = nbytes;
out:
    if ( tmp_buf )
        kfree( tmp_buf );
    return ret;
}

/*!
 * Seek the debug log. The log isn't seekable as the output is consumed as it
 * is read.
 *
 * @param filp File structure for the open instance.
 * @param orig The original offset in the file.
 * @param offset The offset to seek from orig.
 * @return Always returns zero.
 */
static loff_t
debug_log_lseek( struct file *filp,
                 loff_t orig,
                 int offset )
{
    return orig;
}

/*!
 * Release method for the debug log.
 *
 * @param inode The inode of the debugfs log entry.
 * @param filp The file structure for the open instance.
 * @return Always returns zero.
 */
static int
debug_log_release( struct inode *inode,
                   struct file *filp )
{
    return 0;
}

/*! File operations for the debugfs log entry. */
static struct file_operations debug_log_fops = {
    .open       = debug_log_open,
    .read       = debug_log_read,
    .llseek     = debug_log_lseek,
    .release    = debug_log_release,
};

struct dentry *
pc_debug_create_log_file( struct dentry *parent )
{
    debug_ring = kfifo_alloc( DEBUG_RING_SIZE, GFP_KERNEL, &debug_ring_lock );
    if ( !debug_ring )
        goto fail;

    PRINTD( COMPONENT_DEBUG, DBG_TRACE, "debugfs log initialised" );

    debugfs_create_u8( "picoif_dbg_lvl", 0644, parent, &picoif_dbg_lvl );
#ifdef CONFIG_PICOIF_PC202
    debugfs_create_u8( "pc202_dbg_lvl", 0644, parent, &pc202_dbg_lvl );
#endif /* CONFIG_PICOIF_PC202 */
#ifdef CONFIG_PICOIF_PC203
    debugfs_create_u8( "pc203_dbg_lvl", 0644, parent, &pc203_dbg_lvl );
#endif /* CONFIG_PICOIF_PC203 */
#ifdef CONFIG_PICOIF_PC302
    debugfs_create_u8( "pc302_dbg_lvl", 0644, parent, &pc302_dbg_lvl );
#endif /* CONFIG_PICOIF_PC302 */
#if ( defined(CONFIG_PICOIF_PC202) || defined(CONFIG_PICOIF_PC203) )
    debugfs_create_u8( "procif_dbg_lvl", 0644, parent, &procif_dbg_lvl );
#endif /* CONFIG_PICOIF_PC202 || CONFIG_PICOIF_PC203 */
#ifdef CONFIG_PICOIF_PC302
    debugfs_create_u8( "axi2cfg_dbg_lvl", 0644, parent, &axi2cfg_dbg_lvl );
#endif /* CONFIG_PICOIF_PC302 */
    debugfs_create_u8( "gpr_int_dbg_lvl", 0644, parent, &gpr_int_dbg_lvl );
    debugfs_create_u8( "dma_dbg_lvl", 0644, parent, &dma_dbg_lvl );
    debugfs_create_u8( "hwif_dbg_lvl", 0644, parent, &hwif_dbg_lvl );
    debugfs_create_u8( "hwif2_dbg_lvl", 0644, parent, &hwif2_dbg_lvl );
    debugfs_create_u8( "debug_dbg_lvl", 0644, parent, &debug_dbg_lvl );

    spin_lock_init( &debug_ring_lock );

    return debugfs_create_file( "log", 0444, parent, NULL, &debug_log_fops );
fail:
    return NULL;
}

void
pc_debug_close( void )
{
    kfifo_free( debug_ring );
}

#else  /* CONFIG_DEBUG_FS */

/*!
 * Create a log file for the driver in debugfs. If debugfs is not enabled,
 * this will do nothing.
 *
 * @param parent Ignored.
 * @return Always returns NULL.
 */
struct dentry *
debug_create_log_file( struct dentry *parent )
{
    return NULL;
}

ssize_t
pc_debug_print( const char *fmt,
                ... )
{
    return -ENODEV;
}

#endif /* CONFIG_DEBUG_FS */
