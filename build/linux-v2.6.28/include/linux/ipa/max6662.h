/* -*- C -*-
 * max6662.h -- definitions for the Maxim MAX6662 SPI Thermal Sensor
 *
 * Copyright (C) 2009 ip.access Ltd
 *
 */

#include <linux/ioctl.h>


/*
 * Ioctl definitions
 */

/* Use 'T' as magic number */
#define MAX6662_IOCTL_MAGIC          'T'

#define MAX6662_IOCTL_READ_TEMP      _IO(MAX6662_IOCTL_MAGIC,   0)
#define MAX6662_IOCTL_WRITE_CONFIG   _IO(MAX6662_IOCTL_MAGIC,   1)
#define MAX6662_IOCTL_READ_CONFIG    _IO(MAX6662_IOCTL_MAGIC,   2)    /*sdh fix me: may need modifying */
#define MAX6662_IOCTL_WRITE_LIMITS   _IO(MAX6662_IOCTL_MAGIC,   3)
#define MAX6662_IOCTL_READ_LIMITS    _IO(MAX6662_IOCTL_MAGIC,   4)

#define MAX6662_IOCTL_MAXNR          4

typedef struct Max6662ConfigTag
{
	unsigned long  flags;
	
} Max6662Config;

/* Range of limits is 255 to -256 degrees C */
#define  MAX6662_DONT_UPDATE_LIMIT     999

typedef struct Max6662LimitsTag
{
	int  thyst;
	int  tmax;
	int  tlow;
	int  thigh;
	
} Max6662Limits;

typedef struct Max6662StatusTag
{
	signed   long  scaledCelcius;
	unsigned long  flags;
	
} Max6662Status;
