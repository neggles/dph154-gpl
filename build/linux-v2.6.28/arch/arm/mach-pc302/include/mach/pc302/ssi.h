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

#ifndef PC302_SSI_H
#define PC302_SSI_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

/* Functional Registers */

#define SSI_CTRL_REG_0_REG_OFFSET                   0x00
#define SSI_CTRL_REG_1_REG_OFFSET                   0x04
#define SSI_ENABLE_REG_REG_OFFSET                   0x08
#define SSI_MW_CTRL_REG_OFFSET                      0x0C
#define SSI_SLAVE_ENABLE_REG_OFFSET                 0x10
#define SSI_BAUD_RATE_SEL_REG_OFFSET                0x14
#define SSI_TX_FIFO_THRESHOLD_REG_OFFSET            0x18
#define SSI_RX_FIFO_THRESHOLD_REG_OFFSET            0x1C
#define SSI_TX_FIFO_LEVEL_REG_OFFSET                0x20
#define SSI_RX_FIFO_LEVEL_REG_OFFSET                0x24
#define SSI_STATUS_REG_OFFSET                       0x28
#define SSI_IMR_REG_OFFSET                          0x2C
#define SSI_ISR_REG_OFFSET                          0x30
#define SSI_RAW_ISR_REG_OFFSET                      0x34
#define SSI_TX_FIFO_OVRFLOW_INT_CLEAR_REG_OFFSET    0x38
#define SSI_RX_FIFO_OVRFLOW_INT_CLEAR_REG_OFFSET    0x3C
#define SSI_RX_FIFO_UNDFLOW_INT_CLEAR_REG_OFFSET    0x40
#define SSI_MM_INT_CLEAR_REG_OFFSET                 0x44
#define SSI_INT_CLEAR_REG_OFFSET                    0x48
#define SSI_DMA_CTRL_REG_OFFSET                     0x4C
#define SSI_DMA_TX_DATA_LEVEL_REG_OFFSET            0x50
#define SSI_DMA_RX_DATA_LEVEL_REG_OFFSET            0x54
#define SSI_DATA_REG_OFFSET                         0x60                        

/* Identification Registers */

#define SSI_ID_REG_OFFSET                           0x58
#define SSI_COMP_VERSION_REG_OFFSET                 0x5C

/*****************************************************************************/
/* Register Reset Values                                                     */
/*****************************************************************************/

/* Functional Registers */

#define SSI_CTRL_REG_0_REG_RESET                    0x0000
#define SSI_CTRL_REG_1_REG_RESET                    0x0000
#define SSI_ENABLE_REG_REG_RESET                    0x0000
#define SSI_MW_CTRL_REG_RESET                       0x0000
#define SSI_SLAVE_ENABLE_REG_RESET                  0x0000
#define SSI_BAUD_RATE_SEL_REG_RESET                 0x0000
#define SSI_TX_FIFO_THRESHOLD_REG_RESET             0x0000
#define SSI_RX_FIFO_THRESHOLD_REG_RESET             0x0000
#define SSI_TX_FIFO_LEVEL_REG_RESET                 0x0000
#define SSI_RX_FIFO_LEVEL_REG_RESET                 0x0000
#define SSI_STATUS_REG_RESET                        0x0000
#define SSI_IMR_REG_RESET                           0x003F
#define SSI_ISR_REG_RESET                           0x0000
#define SSI_RAW_ISR_REG_RESET                       0x0000
#define SSI_TX_FIFO_OVRFLOW_INT_CLEAR_REG_RESET     0x0000
#define SSI_RX_FIFO_OVRFLOW_INT_CLEAR_REG_RESET     0x0000
#define SSI_RX_FIFO_UNDFLOW_INT_CLEAR_REG_RESET     0x0000
#define SSI_MM_INT_CLEAR_REG_RESET                  0x0000
#define SSI_INT_CLEAR_REG_RESET                     0x0000
#define SSI_DMA_CTRL_REG_RESET                      0x0000
#define SSI_DMA_TX_DATA_LEVEL_REG_RESET             0x0000
#define SSI_DMA_RX_DATA_LEVEL_REG_RESET             0x0000
#define SSI_DATA_REG_RESET                          0x0000

/* Identification Registers */

/* ** Need to check on these values */

#define SSI_ID_REG_RESET                            0x44570007
#define SSI_COMP_VERSION_REG_RESET                  0x3331312A

#endif /* PC302_SSI_H */
