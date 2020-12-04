/* -*- C -*-
 * dac7512.h -- definitions for the Burr Brown DAC7512 SPI DAC
 *
 * Copyright (C) 2009 ip.access Ltd
 *
 */

#include <linux/ioctl.h>


/*
 * Ioctl definitions
 */

/* Use 'D' as magic number */
#define DAC7512_IOCTL_MAGIC            'D'

#define DAC7512_IOCTL_WRITE_VALUE      _IO(DAC7512_IOCTL_MAGIC,   0)

#define DAC7512_IOCTL_MAXNR            0
