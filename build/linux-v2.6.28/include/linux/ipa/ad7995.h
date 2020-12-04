/* -*- C -*-
 * ad7995.h -- definitions for the Analog Devices AD7995 ADC
 *
 * Copyright (C) 2009 ip.access Ltd
 *
 */

#include <linux/ioctl.h>


/*
 * Ioctl definitions
 */

/* Use 'A' as magic number */
#define AD7995_IOCTL_MAGIC          'A'

#define AD7995_IOCTL_READ_ADC       _IO(AD7995_IOCTL_MAGIC,   0)
#define AD7995_IOCTL_WRITE_CONFIG   _IO(AD7995_IOCTL_MAGIC,   1)
#define AD7995_IOCTL_READ_CONFIG    _IO(AD7995_IOCTL_MAGIC,   2)

#define AD7995_IOCTL_MAXNR          2

#define AD7995_CONFIG_SAMPLE_DELAY     0x01
#define AD7995_CONFIG_BIT_TRIAL_DELAY  0x02
#define AD7995_CONFIG_FILTER           0x04
#define AD7995_CONFIG_REF_SELECT       0x08
#define AD7995_CONFIG_CHAN_0           0x10
#define AD7995_CONFIG_CHAN_1           0x20
#define AD7995_CONFIG_CHAN_2           0x40
#define AD7995_CONFIG_CHAN_3           0x80

typedef struct Ad7995ConfigTag
{
	unsigned char  flags;
	
} Ad7995Config;

typedef struct Ad7995SamplesTag
{
	int  adc[4];
	
} Ad7995Samples;
