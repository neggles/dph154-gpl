/*
 * Copyright(C) 2006 picoChip(R) Designs Ltd.
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
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
#ifndef __FIRECRACKER_PC20X_PICOARRAY_H
#define __FIRECRACKER_PC20X_PICOARRAY_H

/* Register address offsets from picoArray Base */

/* DMA 0 to 3 available */
#define PICO_DMA_N_CONFIG_REG_OFFSET(__N)		        (0x00 + 0x0c * (__N))
#define PICO_DMA_N_STATUS_REG_OFFSET(__N)               (0x04 + 0x0c * (__N))
#define PICO_DMA_N_DATAREGOFFSET(__N)                   (0x08 + 0x0c * (__N))

/* GPR 0 to 24 available */
#define PICO_GPR_N_CONFIG_REG_OFFSET(__N)               (0x30 + 0x0c * (__N))
#define PICO_GPR_N_STATUS_REG_OFFSET(__N)               (0x34 + 0x0c * (__N))
#define PICO_GPR_N_DATA_REG_OFFSET(__N)                 (0x38 + 0x0c * (__N))

/* ID registers */
#define PICO_GPR25_CONFIGID_REG_OFFSET                  (0x15c)
#define PICO_GPR25_STATUSID_REG_OFFSET                  (0x160)
#define PICO_GPR25_DATAID_REG_OFFSET                    (0x164)

/* Interrupt status */
/************* TODO - The data sheet is incomplete. Unknown address **********/
#define PICO_INTERRUPT_STATUS_REG_OFFSET                (0x00)  /* TODO Not correct! */   

/* PicoDmaNConfigRegOffset bit definitions. The watermark value can be 0 to 64 */
#define PICO_DMA_INTERRUPT_ENABLE                       (0x00000001)
#define PICO_DMA_ENABLE                                 (0x00000002)
#define PICO_DMA_FIFO_WATERMARK_MASK                    (0x000001fc)
#define PICO_DMA_FIFO_WATERMARK_N(__N)                  (__N << 2)

/* PicoGprNConfigRegOffset bit definitions. */
#define PICO_GPR_INTERRUPT_ENABLE                       (0x00000001)

/* PicoDmaNStatusRegOffset bit definitions */
#define PICO_DMA_SIGNAL_ENABLE                          (0x00000001)
#define PICO_DMA_SIGNAL_DIRECTION                       (0x00000002)
#define PICO_DMA_SIGNAL_BLOCKING                        (0x00000004)
#define PICO_DMA_STATE                                  (0x00000008)
#define PICO_DMA_FIFO_LEVEL                             (0x000007f0)
#define PICO_DMA_FIFO_SINGLE                            (0x00000800)

/* PicoGprNStatusRegOffset bit definitions */
#define PICO_GPR_SIGNAL_ENABLE                          (0x00000001)
#define PICO_GPR_SIGNAL_DIRECTION                       (0x00000002)
#define PICO_GPR_SIGNAL_BLOCKING                        (0x00000004)
#define PICO_GPR_STATE                                  (0x00000008)

/* PicoInterruptStatusRegOffset bit definitions */
#define PICO_DMA_N_INTERRUPT_STATUS(__N)                (0x00000001 << __N)
#define PICO_GPR_N_INTERRUPT_STATUS(__N)                (0x00000010 << __N)


#endif /* __FIRECRACKER_PC20X_PICOARRAY_H */
