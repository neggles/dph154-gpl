#ifndef __MACH_GPIO_H__
/* Copyright (c) 2008 picoChip Designs Ltd.
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define __MACH_GPIO_H__

/* GPIO pin numbers are defined in linux/picochip/devices/pc202.h */
#include <linux/picochip/devices/pc202.h>

int
gpio_is_valid( int gpio );

int
gpio_request( unsigned gpio,
              const char *label );

void
gpio_free( unsigned gpio );

int
gpio_direction_input( unsigned gpio );

int
gpio_direction_output( unsigned gpio,
                       int value );

int
gpio_cansleep( unsigned gpio );

int
gpio_export( unsigned gpio,
             bool direction_may_change );

int
gpio_unexport( unsigned gpio );

int
gpio_get_value_cansleep( unsigned gpio );

int
gpio_set_value_cansleep( unsigned gpio,
                         int value );

int
gpio_set_value( unsigned gpio,
                int value );

int
gpio_get_value( unsigned gpio );

int
gpio_to_irq( unsigned gpio );

int
irq_to_gpio( unsigned irq );

#endif /* __MACH_GPIO_H__ */
