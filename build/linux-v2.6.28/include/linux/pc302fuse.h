/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright (c) 2006-2008 picoChip Designs Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

#ifndef __PC302FUSE_H__
#define __PC302FUSE_H__

#include <linux/ioctl.h>

/**
 * \brief Data structure used for passing commands from userspace to kernel
 * space.
 */
typedef struct
{
    /** The offset of the fuse (bit number). */
    unsigned offset;

    /**
     * The value the fuse is set to if reading, invalid if blowing a fuse. If
     * reading the estimated elapsed VDDQ time, this will be the time in
     * micro-seconds.
     */
    int value;

} pcfuse_t;

/** The base to register ioctl commands from. */
#define PC302_FUSE_IOCTL_BASE   'p'

#define PC302_FUSE_IOCTL_START  ( 0xA0 )

/** Get the status of a fuse. */
#define PC302_FUSE_GET          _IOWR( PC302_FUSE_IOCTL_BASE, \
        PC302_FUSE_IOCTL_START + 0, pcfuse_t )
/** Blow a fuse. */
#define PC302_FUSE_BLOW         _IOWR( PC302_FUSE_IOCTL_BASE, \
        PC302_FUSE_IOCTL_START + 1, pcfuse_t )
/** Get the estimated VDDQ elapsed time. */
#define PC302_FUSE_GET_VDDQ     _IOR( PC302_FUSE_IOCTL_BASE, \
        PC302_FUSE_IOCTL_START + 2, pcfuse_t )

#define PC302_FUSE_IOCTL_NUM_IOCTL  ( 3 )

/** The total number of fuses in the PC302 (including reserved bits). */
#define PC302_FUSE_NUM_FUSES    ( 4096 )

/**
 * \brief Structure to represent a fuse range.
 */
struct fuse_range_t
{
    /** The index of the first fuse in the range. */
    int start;

    /** The index of the last fuse in the range. */
    int end;
};

/** Create a fuse_range_t given the start and end positions. */
#define PC302_FUSE_MK_RANGE( _start, _end ) \
    ( const struct fuse_range_t ) { .start = _start, .end = _end }

/*
 * Begin fuse bit and range definitions.
 *
 * FB is an abbreviation of *F*use *B*it.
 * FR is an abbreviation of *F*use *R*ange.
 */

#define PC302_FB_SECURE_BOOT                              ( 992 )
#define PC302_FB_DISABLE_TRUSTZONE                        ( 993 )
#define PC302_FB_GLOBAL_LAST_TIME_PROGRAM                 ( 994 )
#define PC302_FB_DISABLE_DEBUG                            ( 995 )
#define PC302_FB_DISABLE_ISC                              ( 996 )
#define PC302_FB_DISABLE_JTAG                             ( 997 )
#define PC302_FB_DISABLE_INVASIVE_DEBUG_IN_SECURE         ( 998 )
#define PC302_FB_DISABLE_NON_INVASIVE_DEBUG_IN_SECURE     ( 999 )
#define PC302_FB_DISABLE_CP15_REGISTER                    ( 1000 )
#define PC302_FB_DUAL_SINGLE_MEMIF                        ( 1001 )
#define PC302_FB_DISABLE_NON_SECURE_PARALLEL_FLASH        ( 1002 )

#define PC302_FUSE_ROBP_BASE                              ( 928 )
#define PC302_FUSE_LTP_BASE                               ( 938 )
#define PC302_FUSE_DJTAGRK_BASE                           ( 948 )

/* Secure boot. */
#define PC302_FR_SECURE_BOOTSTRAP           PC302_FUSE_MK_RANGE( 0, 127 )
#define PC302_FB_SECURE_BOOTSTRAP_ROPB      ( PC302_FUSE_ROBP_BASE + 0 )
#define PC302_FB_SECURE_BOOTSTRAP_LTP       ( PC302_FUSE_LTP_BASE + 0 )
#define PC302_FB_SECURE_BOOTSTRAP_DJRK      ( PC302_FUSE_DJTAGRK_BASE + 0 )

#define PC302_FR_COUNTER_IV                 PC302_FUSE_MK_RANGE( 128, 255 )
#define PC302_FB_COUNTER_IV_ROPB            ( PC302_FUSE_ROBP_BASE + 1 )
#define PC302_FB_COUNTER_IV_LTP             ( PC302_FUSE_LTP_BASE + 1 )
#define PC302_FB_COUNTER_IV_DJRK            ( PC302_FUSE_DJTAGRK_BASE + 1 )

/* Key 2. */
#define PC302_FR_KEY2                       PC302_FUSE_MK_RANGE( 256, 383 )
#define PC302_FB_KEY2_ROPB                  ( PC302_FUSE_ROBP_BASE + 2 )
#define PC302_FB_KEY2_LTP                   ( PC302_FUSE_LTP_BASE + 2 )
#define PC302_FB_KEY2_DJRK                  ( PC302_FUSE_DJTAGRK_BASE + 2 )

/* Key 3. */
#define PC302_FR_KEY3                       PC302_FUSE_MK_RANGE( 384, 511 )
#define PC302_FB_KEY3_ROPB                  ( PC302_FUSE_ROBP_BASE + 3 )
#define PC302_FB_KEY3_LTP                   ( PC302_FUSE_LTP_BASE + 3 )
#define PC302_FB_KEY3_DJRK                  ( PC302_FUSE_DJTAGRK_BASE + 3 )

/* Key 4. */
#define PC302_FR_KEY4                       PC302_FUSE_MK_RANGE( 512, 639 )
#define PC302_FB_KEY4_ROPB                  ( PC302_FUSE_ROBP_BASE + 4 )
#define PC302_FB_KEY4_LTP                   ( PC302_FUSE_LTP_BASE + 4 )
#define PC302_FB_KEY4_DJRK                  ( PC302_FUSE_DJTAGRK_BASE + 4 )

/* Key 5. */
#define PC302_FR_KEY5                       PC302_FUSE_MK_RANGE( 640, 767 )
#define PC302_FB_KEY5_ROPB                  ( PC302_FUSE_ROBP_BASE + 5 )
#define PC302_FB_KEY5_LTP                   ( PC302_FUSE_LTP_BASE + 5 )
#define PC302_FB_KEY5_DJRK                  ( PC302_FUSE_DJTAGRK_BASE + 5 )

/* Die identification number. */
#define PC302_FR_DIE_IDENT                  PC302_FUSE_MK_RANGE( 768, 895 )
#define PC302_FB_DIE_IDENT_ROPB             ( PC302_FUSE_ROBP_BASE + 6 )
#define PC302_FB_DIE_IDENT_LTP              ( PC302_FUSE_LTP_BASE + 6 )
#define PC302_FB_DIE_IDENT_DJRK             ( PC302_FUSE_DJTAGRK_BASE + 6 )

/* Customer partition 1. */
#define PC302_FR_PARTITION_1                PC302_FUSE_MK_RANGE( 1024, 2047 )
#define PC302_FB_PARTITION_1_ROPB           ( PC302_FUSE_ROBP_BASE + 7 )
#define PC302_FB_PARTITION_1_LTP            ( PC302_FUSE_LTP_BASE + 7 )
#define PC302_FB_PARTITION_1_DJRK           ( PC302_FUSE_DJTAGRK_BASE + 7 )

/* Customer partition 2. */
#define PC302_FR_PARTITION_2                PC302_FUSE_MK_RANGE( 2048, 3071 )
#define PC302_FB_PARTITION_2_ROPB           ( PC302_FUSE_ROBP_BASE + 8 )
#define PC302_FB_PARTITION_2_LTP            ( PC302_FUSE_LTP_BASE + 8 )
#define PC302_FB_PARTITION_2_DJRK           ( PC302_FUSE_DJTAGRK_BASE + 8 )

/* Customer partition 3. */
#define PC302_FR_PARTITION_3                PC302_FUSE_MK_RANGE( 3072, 4095 )
#define PC302_FB_PARTITION_3_ROPB           ( PC302_FUSE_ROBP_BASE + 9 )
#define PC302_FB_PARTITION_3_LTP            ( PC302_FUSE_LTP_BASE + 9 )
#define PC302_FB_PARTITION_3_DJRK           ( PC302_FUSE_DJTAGRK_BASE + 9 )

#endif /* !__PC302FUSE_H__ */

