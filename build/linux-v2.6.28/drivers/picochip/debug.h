/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/* Copyright (c) 2009 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file debug.h
 * \brief picoIf debug definitions.
 *
 * This file defines a set of macros and functions for adding debug into the
 * picoIf driver. All debug information is exported through debugfs.
 *
 * \page debugging Debugging the picoIf driver
 *
 * \section introduction Introduction
 *
 * The picoIf kernel module provides some debug capability through debugfs. If
 * the kernel is compiled with debugfs support, then picoIf will create a
 * hierarchy in /debug/picoif. A typical hierarchy (depending on what
 * transports and devices are enabled) may look like:
 *
 * \code
 * root@picocpe:~# ls /debug/picoif/
 * debug_dbg_lvl    gpr_int_dbg_lvl  pc202_dbg_lvl    procif_dbg_lvl
 * dma_dbg_lvl      log              picoif_dbg_lvl
 * \endcode
 *
 * Each of these *lvl files contain the current logging level of that
 * component where higher numbers are decreasing priority. These levels can be
 * queried and set by accessing the file. For example, to set the PC202 debug
 * level to 5:
 *
 * \code
 * root@picocpe:~# echo -n "5" > /debug/picoif/pc202_dbg_lvl
 * root@picocpe:~# cat /debug/picoif/pc202_dbg_lvl
 * 5
 * root@picocpe:~#
 * \endcode
 *
 * The debug log itself can be read through the /debug/picoif/log file. Note
 * that reading this file will consume the log and the log itself is not
 * seekable. The log is implemented as a circular buffer, and if the buffer
 * fills, the oldest data is discarded to allow new messages to be logged.
 *
 * To prevent the debug information from slowing down applications by printing
 * to the serial console, by default the debug messages won't be printed to
 * the console. If you do want to send debug messages to the console in
 * addition to the ring buffer, then DBG_TO_CONSOLE should be defined in
 * debug.c. To disable this, simply undef it.
 *
 * \section addingnewcomponents Adding new debug components
 *
 * To add new debug components:
 *  \li Create a static u8 for the current debug level of the component in
 *  debug.c
 *  \li Create a dbg_component in debug.c for the new component.
 *  \li In pc_debug_create_log_file(), add a debugfs_create_u8() call for the
 *  new components debug level. This will export the debug level through
 *  debugfs.
 *  \li In debug.h, create a new COMPONENT_xxx and corresponding extern
 *  dbg_component declaration so that the component can be used in the code.
 *
 * \section debugfromkernel Printing the debug from kernelspace
 *
 * It is also possible to dump the contents of the debug ring buffer from
 * kernel space. This is achieved with debug_dump().
 */
#ifndef __PICOIF_DEBUG_H__
#define __PICOIF_DEBUG_H__

#include <linux/fs.h>

/*!
 * \brief Debug component structure. Used so that we can control debug levels
 * through debugfs and prefix messages with a component name.
 */
struct dbg_component
{
    u8          *curr_lvl;  /*!< Pointer to the current debug level of the
                             *   component. */
    const char  *name;      /*!< The name of the debug component. */
};

extern struct dbg_component picoif_dbg_component;
extern struct dbg_component pc202_dbg_component;
extern struct dbg_component pc203_dbg_component;
extern struct dbg_component pc302_dbg_component;
extern struct dbg_component procif_dbg_component;
extern struct dbg_component axi2cfg_dbg_component;
extern struct dbg_component gpr_int_dbg_component;
extern struct dbg_component dma_dbg_component;
extern struct dbg_component hwif_dbg_component;
extern struct dbg_component hwif2_dbg_component;
extern struct dbg_component debug_dbg_component;

/*! Main picoIf debug component. */
#define COMPONENT_PICOIF    (&picoif_dbg_component)
/*! PC202 debug component. */
#define COMPONENT_PC202     (&pc202_dbg_component)
/*! PC203 debug component. */
#define COMPONENT_PC203     (&pc203_dbg_component)
/*! PC302 debug component. */
#define COMPONENT_PC302     (&pc302_dbg_component)
/*! procif debug component. */
#define COMPONENT_PROCIF    (&procif_dbg_component)
/*! AXI2Cfg debug component. */
#define COMPONENT_AXI2CFG   (&axi2cfg_dbg_component)
/*! GPR IRQ transport module component. */
#define COMPONENT_GPR_INT   (&gpr_int_dbg_component)
/*! DMA transport module component. */
#define COMPONENT_DMA       (&dma_dbg_component)
/*! HwIF transport module component. */
#define COMPONENT_HWIF     (&hwif_dbg_component)
/*! HwIF2 transport module component. */
#define COMPONENT_HWIF2     (&hwif2_dbg_component)
/*! Debug module component. */
#define COMPONENT_DEBUG     (&debug_dbg_component)

/*! Debug levels for the debug subsystem. Higher numeric values are of lower
 *  importance. */
enum dbg_level
{
    DBG_ERROR,
    DBG_WARN,
    DBG_TRACE,
    DBG_TRIVIAL,
};

#undef DBG_SHOW_LOCATION

#ifdef DBG_SHOW_LOCATION
/*!
 * Print a debug message if the current debug level is appropriate for the
 * severity of the debug message.
 *
 * @param _component The component to tag the message with.
 * @param _level The level of the message.
 * @param _fmt The format string for the message.
 */
#define PRINTD( _component, _level, _fmt, ... ) \
    do { \
        if ( _level < *( _component->curr_lvl ) ) { \
            pc_debug_print( _component->name, "%s %u: " _fmt "\n", __FILE__, \
                            __LINE__, ##__VA_ARGS__ ); \
        } \
    } while ( 0 )
#else /* DBG_SHOW_LOCATION */
/*!
 * Print a debug message if the current debug level is appropriate for the
 * severity of the debug message.
 *
 * @param _component The component to tag the message with.
 * @param _level The level of the message.
 * @param _fmt The format string for the message.
 */
#define PRINTD( _component, _level, _fmt, ... ) \
    do { \
        if ( _level < *( _component->curr_lvl ) ) { \
            pc_debug_print( _component->name, "%s(): " _fmt "\n", \
                            __FUNCTION__, ##__VA_ARGS__ ); \
        } \
    } while ( 0 )
#endif /* DBG_SHOW_LOCATION */

/*!
 * Print a debug message into the debug ring.
 *
 * @param component The name of the component responsible for the message.
 * @param fmt The format string for the message.
 * @return Returns the number of bytes written into the ring.
 */
ssize_t pc_debug_print( const char *component,
                        const char *fmt,
                        ... );

/*!
 * Dump the debug ring to the console.
 */
void debug_dump( void );

/*!
 * Create an entry for the picoIf log in debugfs.
 *
 * @param parent The directory to create the log file in.
 * @return Returns a dentry pointer to the log file on success, NULL on
 * failure.
 */
struct dentry *
pc_debug_create_log_file( struct dentry *parent );

/*!
 * Destroy the debug ring.
 */
void
pc_debug_close( void );

#endif /* __PICOIF_DEBUG_H__ */
