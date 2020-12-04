/******************************************************************************
 *
 * Copyright (c) 2009 ip.access Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * 22 Apr 2009 Created by Simon D Hughes.
 *
 * header for remainder table
 *****************************************************************************
 * Filename: 
 *****************************************************************************/

/* remainder_table
 *  lookup table of jiffies (32 bit word) remainders
 *  table of remainders of 2^32 % period for 1 < period < 63
 *  using a table allows us to avoid using a spin lock for ioctls that 
 *  change the period and hence need the remainder recalculating (as this 
 *  would cause a race condition between the ioctl syscall and the timer 
 *  callback which needs to access period and remainder). 
 *  generated using code in cpetd_test.c
 *
 *  defined in header file so can be shared between module and test app.
 */
#define CPETD_MAX_PERIOD        63  /* 2^32 for computing remainder. jiffies is 32bit number */

#define CPTED_DECLARE_REMAINDER_TABLE(__varname)                                    \
    static unsigned char __varname[] = {                                            \
   -1,   0,   0,   1,   0,   1,   4,   4,   0,   4,   6,   4,   4,   9,   4,   1,   \
    0,   1,   4,   6,  16,   4,   4,  12,  16,  21,  22,  22,   4,  16,  16,   4,   \
    0,   4,  18,  11,   4,   7,   6,  22,  16,  37,   4,  16,   4,  31,  12,  42,   \
    16,  39,  46,   1,  48,  42,  22,  26,  32,  25,  16,  51,  16,  57,   4,   4   \
    }
