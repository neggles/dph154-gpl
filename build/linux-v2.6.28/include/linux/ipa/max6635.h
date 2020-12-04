/* -*- C -*-
 * max6635.h -- definitions for the Maxim MAX6635 I2C Thermal Sensor
 *
 * Copyright (C) 2009 ip.access Ltd
 *
 */

#include <linux/ioctl.h>


/*
 * Ioctl definitions
 */

/* Use 'T' as magic number */
#define MAX6635_IOCTL_MAGIC          'T'

#define MAX6635_IOCTL_READ_TEMP      _IO(MAX6635_IOCTL_MAGIC,   0)
#define MAX6635_IOCTL_WRITE_CONFIG   _IO(MAX6635_IOCTL_MAGIC,   1)
#define MAX6635_IOCTL_READ_CONFIG    _IO(MAX6635_IOCTL_MAGIC,   2)
#define MAX6635_IOCTL_WRITE_LIMITS   _IO(MAX6635_IOCTL_MAGIC,   3)
#define MAX6635_IOCTL_READ_LIMITS    _IO(MAX6635_IOCTL_MAGIC,   4)

#define MAX6635_IOCTL_MAXNR          4

typedef struct Max6635ConfigTag
{
	unsigned long  flags;
	
} Max6635Config;

/* Range of limits is 255 to -256 degrees C */
#define  MAX6635_DONT_UPDATE_LIMIT     999

typedef struct Max6635LimitsTag
{
	int  thyst;
	int  tmax;
	int  tlow;
	int  thigh;
	
} Max6635Limits;

typedef struct Max6635StatusTag
{
	signed   long  scaledCelcius;
	unsigned long  flags;
	
} Max6635Status;
