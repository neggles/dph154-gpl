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
 *              picoArray Register Definitions
 *
 *****************************************************************************/
#ifndef __PC20X_PICOARRAY_H
#define __PC20X_PICOARRAY_H

/* Register address offsets from picoArray Base */

/* DMA 0 to 3 available */
#define PicoDmaNConfigRegOffset(__N)		        (0x00 + 0x0c * (__N))
#define PicoDmaNStatusRegOffset(__N)                (0x04 + 0x0c * (__N))
#define PicoDmaNDataRegOffset(__N)                  (0x08 + 0x0c * (__N))

/* GPR 0 to 24 available */
#define PicoGprNConfigRegOffset(__N)                (0x30 + 0x0c * (__N))
#define PicoGprNStatusRegOffset(__N)                (0x34 + 0x0c * (__N))
#define PicoGprNDataRegOffset(__N)                  (0x38 + 0x0c * (__N))

/* ID registers */
#define PicoGpr25ConfigIDRegOffset                  (0x15c)
#define PicoGpr25StatusIDRegOffset                  (0x160)
#define PicoGpr25DataIDRegOffset                    (0x164)

/* Interrupt status */
/************* TODO - The data sheet is incomplete. Unknown address **********/
#define PicoInterruptStatusRegOffset                (0x00)  /* TODO Not correct! */   

/* PicoDmaNConfigRegOffset bit definitions. The watermark value can be 0 to 64 */
#define PicoDmaInterruptEnable                      (0x00000001)
#define PicoDmaEnable                               (0x00000002)
#define PicoDmaFIFOWatermarkMask                    (0x000001fc)
#define PicoDmaFIFOWatermarkN(__N)                  (__N << 2)

/* PicoGprNConfigRegOffset bit definitions. */
#define PicoGprInterruptEnable                      (0x00000001)

/* PicoDmaNStatusRegOffset bit definitions */
#define PicoDmaSignalEnable                         (0x00000001)
#define PicoDmaSignalDirection                      (0x00000002)
#define PicoDmaSignalBlocking                       (0x00000004)
#define PicoDmaState                                (0x00000008)
#define PicoDmaFIFOLevel                            (0x000007f0)
#define PicoDmaFifoSingle                           (0x00000800)

/* PicoGprNStatusRegOffset bit definitions */
#define PicoGprSignalEnable                         (0x00000001)
#define PicoGprSignalDirection                      (0x00000002)
#define PicoGprSignalBlocking                       (0x00000004)
#define PicoGprState                                (0x00000008)

/* PicoInterruptStatusRegOffset bit definitions */
#define PicoDmaNInterruptStatus(__N)                (0x00000001 << __N)
#define PicoGprNInterruptStatus(__N)                (0x00000010 << __N)


#endif /* __PC20X_PICOARRAY_H */
