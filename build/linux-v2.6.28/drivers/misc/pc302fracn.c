/**
 * \file pc302fracn.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright 2008 picoChip Designs LTD, All Rights Reserved.
 * http://www.picochip.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * This file implements a driver for configuring the Fractional-N synthesizer
 * in the picoChip PC302 device.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/picochip/picoif.h>
#include <linux/gpio.h>
#include <mach/pc302fracn.h>
#include <mach/gpio.h>
#include <mach/pc302/pc302.h>
#include <mach/pc302/axi2cfg.h>

/* CAEID of the Frac-N synth. */
#define FRACN_CAEID             ( 0x0578 )
#define FRACN_M_N_REG_OFFSET    ( 0x000A )  /* M&N register offset. */
#define FRACN_K_LOW_REG_OFFSET  ( 0x000B )  /* K[15:0] register offset. */
#define FRACN_K_HI_REG_OFFSET   ( 0x000C )  /* K[31:16] register offset. */
#define FRACN_CTRL_REG_OFFSET   ( 0x000D )  /* Control register offset. */
#define FRACN_LL_REG_OFFSET     ( 0x000E )  /* Lower limit register offset. */
#define FRACN_UL_REG_OFFSET     ( 0x000F )  /* Upper limit register offset. */
#define FRACN_STATUS_REG_OFFSET ( 0x0010 )  /* Status register offset. */

/* Offset and mask for the M value in the M&N register. */
#define FRACN_M_OFFSET          ( 8 )
#define FRACN_M_MASK            ( 0xFF << FRACN_M_OFFSET )

/* Offset and mask for the N value in the M&N register. */
#define FRACN_N_OFFSET          ( 0 )
#define FRACN_N_MASK            ( 0xFF << FRACN_N_OFFSET )

/* Mask for the pulse width in the upper and lower limit registers. */
#define FRACN_CV_PULSE_WIDTH_MASK   ( 0x7FF )

/* Offset and mask for the load instruction bit in the control register. */
#define FRACN_LOAD_OFFSET       ( 0 )
#define FRACN_LOAD_MASK         ( 1 << FRACN_LOAD_OFFSET )

/* Offset and mask for the reset instruction bit in the control register. */
#define FRACN_RESET_OFFSET       ( 2 )
#define FRACN_RESET_MASK         ( 1 << FRACN_RESET_OFFSET )

/**
 * The miscdevice for the driver. We don't need a device node but this
 * provides the driver setup to allow us to use sysfs with minimal overhead.
 */
static struct miscdevice fracn_mdev = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "pc302fracn",
};

/**
 * Read a register from the Frac-N over the config bus.
 *
 * \param reg_num The register to read.
 * \param val Pointer to the result variable.
 * \return Returns zero on success, non-zero on failure.
 */
static int
fracn_read_reg( u16 reg_num,
                u16 *val )
{
    int ret = picoif_config_read( 0, FRACN_CAEID, reg_num, 1, val );
    return ( 1 == ret ) ? 0 : -EIO;
}

/**
 * Write a register in the Frac-N via the config bus.
 *
 * \param reg_num The register to write to.
 * \param mask Mask of the bits to write.
 * \param val The value to write to the masked bits.
 * \return Returns zero on success, non-zero on failure.
 */
static int
fracn_write_reg( u16 reg_num,
                 u16 mask,
                 u16 val )
{
    u16 tmp;
    int ret = picoif_config_read( 0, FRACN_CAEID, reg_num, 1, &tmp );
    if ( 1 != ret )
        return -EIO;
    tmp &= ~mask;
    tmp |= ( val & mask );
    ret = picoif_config_write( 0, FRACN_CAEID, reg_num, 1, &tmp );

    return ( 1 == ret ) ? 0 : -EIO;
}

/* Get M. */
static int
fracn_get_m( u8 *val )
{
    u16 tmp;
    if ( fracn_read_reg( FRACN_M_N_REG_OFFSET, &tmp ) )
        return -EIO;
    *val = ( tmp & FRACN_M_MASK ) >> FRACN_M_OFFSET;
    return 0;
}
EXPORT_SYMBOL( fracn_get_m );

/* Set M. */
static int
fracn_set_m( u8 val )
{
    return fracn_write_reg( FRACN_M_N_REG_OFFSET, FRACN_M_MASK,
                            val << FRACN_M_OFFSET );
}
EXPORT_SYMBOL( fracn_set_m );

/* Get N. */
static int
fracn_get_n( u8 *val )
{
    u16 tmp;
    if ( fracn_read_reg( FRACN_M_N_REG_OFFSET, &tmp ) )
        return -EIO;
    *val = ( tmp & FRACN_N_MASK ) >> FRACN_N_OFFSET;
    return 0;
}
EXPORT_SYMBOL( fracn_get_n );

/* Set N. */
static int
fracn_set_n( u8 val )
{
    return fracn_write_reg( FRACN_M_N_REG_OFFSET, FRACN_N_MASK,
                            val << FRACN_N_OFFSET );
}
EXPORT_SYMBOL( fracn_set_n );

/* Get K. */
static int
fracn_get_k( u32 *val )
{
    u16 k_low;
    u16 k_high;

    if ( fracn_read_reg( FRACN_K_LOW_REG_OFFSET, &k_low ) )
        return -EIO;
    if ( fracn_read_reg( FRACN_K_HI_REG_OFFSET, &k_high ) )
        return -EIO;

    *val = k_high << 16 | k_low;

    return 0;
}
EXPORT_SYMBOL( fracn_get_k );

/* Set K. */
static int
fracn_set_k( u32 val )
{
    u16 k_high = ( val >> 16 ) & 0xFFFF;
    u16 k_low = val & 0xFFFF;

    if ( fracn_write_reg( FRACN_K_HI_REG_OFFSET, 0xFFFF, k_high ) )
        return -EIO;

    if ( fracn_write_reg( FRACN_K_LOW_REG_OFFSET, 0xFFFF, k_low ) )
        return -EIO;

    return 0;
}
EXPORT_SYMBOL( fracn_set_k );

/* Get the control voltage pulse lower limit. */
static int
fracn_get_cv_pulse_ll( u16 *val )
{
    if ( fracn_read_reg( FRACN_LL_REG_OFFSET, val ) )
        return -EIO;
    *val &= FRACN_CV_PULSE_WIDTH_MASK;
    return 0;
}
EXPORT_SYMBOL( fracn_get_cv_pulse_ll );

#define FRACN_CV_PULSE_LL_MAX   ( 0x7FF )

/* Set the control voltage pulse lower limit. */
static int
fracn_set_cv_pulse_ll( u16 val )
{
    if ( val > FRACN_CV_PULSE_LL_MAX )
        return -EINVAL;

    return fracn_write_reg( FRACN_UL_REG_OFFSET,
                            FRACN_CV_PULSE_WIDTH_MASK, val );
}
EXPORT_SYMBOL( fracn_set_cv_pulse_ll );

/* Get the control voltage pulse upper limit. */
static int
fracn_get_cv_pulse_ul( u16 *val )
{
    if ( fracn_read_reg( FRACN_UL_REG_OFFSET, val ) )
        return -EIO;
    *val &= FRACN_CV_PULSE_WIDTH_MASK;
    return 0;
}
EXPORT_SYMBOL( fracn_get_cv_pulse_ul );

#define FRACN_CV_PULSE_UL_MAX   ( 0x7FF )

/* Set the control voltage pulse upper limit. */
static int
fracn_set_cv_pulse_ul( u16 val )
{
    if ( val > FRACN_CV_PULSE_UL_MAX )
        return -EINVAL;

    return fracn_write_reg( FRACN_UL_REG_OFFSET,
                            FRACN_CV_PULSE_WIDTH_MASK, val );
}
EXPORT_SYMBOL( fracn_set_cv_pulse_ul );

/* Get the status register value. */
static int
fracn_get_status( u16 *val )
{
    if ( fracn_read_reg( FRACN_STATUS_REG_OFFSET, val ) )
        return -EIO;
    return 0;
}
EXPORT_SYMBOL( fracn_get_status );

/* Reset the Frac-N synth. */
static int
fracn_reset( void )
{
    return fracn_write_reg( FRACN_CTRL_REG_OFFSET, FRACN_RESET_MASK,
                            1 << FRACN_RESET_OFFSET );
}
EXPORT_SYMBOL( fracn_reset );

/* Load the Frac-N with the new M, N and K values. */
static int
fracn_load( void )
{
    int ret = fracn_write_reg( FRACN_CTRL_REG_OFFSET, FRACN_LOAD_MASK,
                               1 << FRACN_LOAD_OFFSET );
    if ( ret )
        goto out;

    ret = fracn_write_reg( FRACN_CTRL_REG_OFFSET, FRACN_LOAD_MASK,
                           0 << FRACN_LOAD_OFFSET );

out:
    return ret;
}
EXPORT_SYMBOL( fracn_load );

/* Show the current value of M. */
static ssize_t
fracn_sysfs_show_m( struct device *dev,
                    struct device_attribute *attr,
                    char *buf )
{
    u8 m;
    int ret = fracn_get_m( &m );
    if ( ret )
        return ret;

    return sprintf( buf, "%u\n", m );
}

/* Store a new value of M. */
static ssize_t
fracn_sysfs_store_m( struct device *dev,
                     struct device_attribute *attr,
                     const char *buf,
                     size_t count )
{
    int ret = fracn_set_m( simple_strtoul( buf, NULL, 0 ) );
    return ( !ret ) ? count : ret;
}
DEVICE_ATTR( m, 0644, fracn_sysfs_show_m, fracn_sysfs_store_m );

/* Show the current value of N. */
static ssize_t
fracn_sysfs_show_n( struct device *dev,
                    struct device_attribute *attr,
                    char *buf )
{
    u8 n;
    int ret = fracn_get_n( &n );
    if ( ret )
        return ret;

    return sprintf( buf, "%u\n", n );
}

/* Store a new value of N. */
static ssize_t
fracn_sysfs_store_n( struct device *dev,
                     struct device_attribute *attr,
                     const char *buf,
                     size_t count )
{
    int ret = fracn_set_n( simple_strtoul( buf, NULL, 0 ) );
    return ( !ret ) ? count : ret;
}
DEVICE_ATTR( n, 0644, fracn_sysfs_show_n, fracn_sysfs_store_n );

/* Show the current value of K. */
static ssize_t
fracn_sysfs_show_k( struct device *dev,
                    struct device_attribute *attr,
                    char *buf )
{
    u32 k;
    int ret = fracn_get_k( &k );
    if ( ret )
        return ret;

    return sprintf( buf, "%u\n", k );
}

/* Store a new value of K. */
static ssize_t
fracn_sysfs_store_k( struct device *dev,
                     struct device_attribute *attr,
                     const char *buf,
                     size_t count )
{
    int ret = fracn_set_k( simple_strtoul( buf, NULL, 0 ) );
    return ( !ret ) ? count : ret;
}
DEVICE_ATTR( k, 0644, fracn_sysfs_show_k, fracn_sysfs_store_k );

/* Show the current control voltage pulse lower limit. */
static ssize_t
fracn_sysfs_show_cv_pulse_ll( struct device *dev,
                              struct device_attribute *attr,
                              char *buf )
{
    u16 cv_pulse_ll;
    int ret = fracn_get_cv_pulse_ll( &cv_pulse_ll );
    if ( ret )
        return ret;

    return sprintf( buf, "%u\n", cv_pulse_ll );
}

/* Store a new current control voltage pulse lower limit. */
static ssize_t
fracn_sysfs_store_cv_pulse_ll( struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count )
{
    int ret = fracn_set_cv_pulse_ll( simple_strtoul( buf, NULL, 0 ) );
    return ( !ret ) ? count : ret;
}
DEVICE_ATTR( cv_pulse_ll, 0644, fracn_sysfs_show_cv_pulse_ll,
             fracn_sysfs_store_cv_pulse_ll );

/* Show the current control voltage pulse upper limit. */
static ssize_t
fracn_sysfs_show_cv_pulse_ul( struct device *dev,
                              struct device_attribute *attr,
                              char *buf )
{
    u16 cv_pulse_ul;
    int ret = fracn_get_cv_pulse_ul( &cv_pulse_ul );
    if ( ret )
        return ret;

    return sprintf( buf, "%u\n", cv_pulse_ul );
}

/* Store a new current control voltage pulse upper limit. */
static ssize_t
fracn_sysfs_store_cv_pulse_ul( struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count )
{
    int ret = fracn_set_cv_pulse_ul( simple_strtoul( buf, NULL, 0 ) );
    return ( !ret ) ? count : ret;
}
DEVICE_ATTR( cv_pulse_ul, 0644, fracn_sysfs_show_cv_pulse_ul,
             fracn_sysfs_store_cv_pulse_ul );

/* Show whether the VCXO is running fast or not. */
static ssize_t
fracn_sysfs_show_running_fast( struct device *dev,
                               struct device_attribute *attr,
                               char *buf )
{
    u16 status;
    int ret = fracn_get_status( &status );
    int running_fast;
    if ( ret )
        return ret;

    running_fast =
        ( status & FRACN_RUNNING_FAST_MASK ) >> FRACN_RUNNING_FAST_OFFSET;

    return sprintf( buf, "%u\n", running_fast );
}
DEVICE_ATTR( running_fast, 0444, fracn_sysfs_show_running_fast, NULL );

/* Show whether the VCXO is running slow or not. */
static ssize_t
fracn_sysfs_show_running_slow( struct device *dev,
                               struct device_attribute *attr,
                               char *buf )
{
    u16 status;
    int ret = fracn_get_status( &status );
    int running_slow;
    if ( ret )
        return ret;

    running_slow =
        ( status & FRACN_RUNNING_SLOW_MASK ) >> FRACN_RUNNING_SLOW_OFFSET;

    return sprintf( buf, "%u\n", running_slow );
}
DEVICE_ATTR( running_slow, 0444, fracn_sysfs_show_running_slow, NULL );

/* Show the current control voltage pulse width. */
static ssize_t
fracn_sysfs_show_ctrl_v_pulse_width( struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf )
{
    u16 status;
    int ret = fracn_get_status( &status );
    int pulse_width;
    if ( ret )
        return ret;

    pulse_width = ( status & FRACN_CTRL_V_PULSE_WIDTH_MASK ) >>
                    FRACN_CTRL_V_PULSE_WIDTH_OFFSET;

    return sprintf( buf, "%u\n", pulse_width );
}
DEVICE_ATTR( ctrl_v_pulse_width, 0444, fracn_sysfs_show_ctrl_v_pulse_width,
             NULL );

/* Show the current control voltage under limit. */
static ssize_t
fracn_sysfs_show_ctrl_v_under_limit( struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf )
{
    u16 status;
    int ret = fracn_get_status( &status );
    int under_limit;
    if ( ret )
        return ret;

    under_limit = ( status & FRACN_CTRL_V_UNDER_LIMIT_MASK ) >>
                    FRACN_CTRL_V_UNDER_LIMIT_OFFSET;

    return sprintf( buf, "%u\n", under_limit );
}
DEVICE_ATTR( ctrl_v_under_limit, 0444, fracn_sysfs_show_ctrl_v_under_limit,
             NULL );

/* Show the current control voltage over limit. */
static ssize_t
fracn_sysfs_show_ctrl_v_over_limit( struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf )
{
    u16 status;
    int ret = fracn_get_status( &status );
    int over_limit;
    if ( ret )
        return ret;

    over_limit = ( status & FRACN_CTRL_V_OVER_LIMIT_MASK ) >>
                    FRACN_CTRL_V_OVER_LIMIT_OFFSET;

    return sprintf( buf, "%u\n", over_limit );
}
DEVICE_ATTR( ctrl_v_over_limit, 0444, fracn_sysfs_show_ctrl_v_over_limit,
             NULL );

/* Show whether the synth is locked or not (1 == synth not locked). */
static ssize_t
fracn_sysfs_show_not_locked( struct device *dev,
                             struct device_attribute *attr,
                             char *buf )
{
    u16 status;
    int ret = fracn_get_status( &status );
    int not_locked;
    if ( ret )
        return ret;

    not_locked = ( status & FRACN_NOT_LOCKED_MASK ) >>
                    FRACN_NOT_LOCKED_OFFSET;

    return sprintf( buf, "%u\n", not_locked );
}
DEVICE_ATTR( not_locked, 0444, fracn_sysfs_show_not_locked,
             NULL );

/* Reset the synth. Writing any value will trigger a reset. */
static ssize_t
fracn_sysfs_store_reset( struct device *dev,
                         struct device_attribute *attr,
                         const char *buf,
                         size_t count )
{
    int ret = fracn_reset();
    return ( !ret ) ? count : ret;
}
DEVICE_ATTR( reset, 0200, NULL, fracn_sysfs_store_reset );

/* Load the new values of M, N and K into the synth. */
static ssize_t
fracn_sysfs_store_load( struct device *dev,
                        struct device_attribute *attr,
                        const char *buf,
                        size_t count )
{
    int ret = fracn_load();
    return ( !ret ) ? count : ret;
}
DEVICE_ATTR( load, 0200, NULL, fracn_sysfs_store_load );

/* The attributes to export via sysfs. */
static struct attribute *fracn_attrs[] = {
    &dev_attr_m.attr,
    &dev_attr_n.attr,
    &dev_attr_k.attr,
    &dev_attr_cv_pulse_ll.attr,
    &dev_attr_cv_pulse_ul.attr,
    &dev_attr_running_fast.attr,
    &dev_attr_running_slow.attr,
    &dev_attr_ctrl_v_pulse_width.attr,
    &dev_attr_ctrl_v_under_limit.attr,
    &dev_attr_ctrl_v_over_limit.attr,
    &dev_attr_not_locked.attr,
    &dev_attr_reset.attr,
    &dev_attr_load.attr,
    NULL,
};

/* Create an attribute group to add and remove. */
static struct attribute_group fracn_attr_group = {
    .attrs = fracn_attrs,
};

static int
fracn_init( void )
{
    int ret = misc_register( &fracn_mdev );
    u16 val = 0;
    if ( ret )
        return ret;

    ret = sysfs_create_group( &fracn_mdev.this_device->kobj,
                              &fracn_attr_group );
    if ( ret )
        goto sysfs_fail;

    /* Wake the Frac-N up. */
    if (1 != picoif_config_write(0, FRACN_CAEID, 0xA060, 1, &val))
        goto sysfs_fail;

    if ( gpio_request( PC302_GPIO_PIN_SDGPIO_0, "Frac-N" ) )
        goto gpio_fail;

    syscfg_update( AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_MASK,
		   AXI2CFG_SYS_CONFIG_FREQ_SYNTH_MUX_MASK );

    /* Load the Frac-N with the initial default M, N and K values. */
    (void)fracn_load();

    return ret;

gpio_fail:
    sysfs_remove_group( &fracn_mdev.this_device->kobj, &fracn_attr_group );

sysfs_fail:
    misc_deregister( &fracn_mdev );

    return ret;
}

static void
fracn_exit( void )
{
    gpio_free( PC302_GPIO_PIN_SDGPIO_0 );
    sysfs_remove_group( &fracn_mdev.this_device->kobj, &fracn_attr_group );
    misc_deregister( &fracn_mdev );
}

module_init( fracn_init );
module_exit( fracn_exit );
MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Jamie Iles" );
