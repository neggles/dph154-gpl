/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright(C) 2006 picoChip(R) Designs Ltd.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
 
/*****************************************************************************
 *
 * Description: picoChip PC20x ARM Subsystem
 *
 *              GPIO Register Definitions
 *
 *****************************************************************************/ 
#ifndef __PC20X_GPIO_H
#define __PC20X_GPIO_H

/* Gpio pins for Port A, numbered 0 to N-1 */
#define NumberPortAPins                             8

/* Register definitions for GPIO */
#define GpioPortAOutputDataRegOffset                (0x00)
#define GpioPortADataDirectionRegOffset             (0x04)
#define GpioPortAInputDataRegOffset                 (0x50)

/* Register definitions for GPIO interrupts */
#define GpioPortAInterruptEnableRegOffset           (0x30)
#define GpioPortAInterruptMaskRegOffset             (0x34)
#define GpioPortAInterruptLevelRegOffset            (0x38)
#define GpioPortAInterruptPolarityRegOffset         (0x3c)
#define GpioPortAInterruptStatusRegOffset           (0x40)
#define GpioPortAInterruptRawStatusRegOffset        (0x44)
#define GpioPortAInterruptEOIRegOffset              (0x4c)
#define GpioPortAInterruptSyncLevelRegOffset        (0x60)

/* Gpio identification register */
#define GpioIdentifierRegOffset                     (0x64)


/* Bit definitions */
#define Gpio(__N)                                   (0x01 << (__N))

/* Synchronisation level register bits */
#define SynchroniseLevelInterrupts                  (0x01)


#endif /* __PC20X_GPIO_H */



