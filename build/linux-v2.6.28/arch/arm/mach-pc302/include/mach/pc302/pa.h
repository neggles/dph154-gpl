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

#ifndef PC302_PA_H
#define PC302_PA_H

#define PA_TIMECOUNT 32

#define PA_AEID_CTRL_0    0x0008
#define PA_AEID_TURBO     0x0028
#define PA_AEID_UL_TB_FMT 0x0038
#define PA_AEID_UL_DSPRD  0x0049
#define PA_AEID_UL_SYM    0x0058
#define PA_AEID_CTRL_06   0x0068
#define PA_AEID_MEMIF     0x0088
#define PA_AEID_AXI2PICO  0x00A8

#define PA_WHATAMI_REG_ADDR 0xa064

#define  PA_WHATAMI_STAN2           0x00
#define  PA_WHATAMI_RESVD1          0x01
#define  PA_WHATAMI_MEM             0x02
#define  PA_WHATAMI_SWITCH          0x03
#define  PA_WHATAMI_CTRL            0x04
#define  PA_WHATAMI_MEMIF           0x05
#define  PA_WHATAMI_RESVD6          0x06
#define  PA_WHATAMI_PAI             0x07
#define  PA_WHATAMI_RESVD8          0x08
#define  PA_WHATAMI_RESVD9          0x09
#define  PA_WHATAMI_RESVD10         0x0A
#define  PA_WHATAMI_MAXIM           0x0B
#define  PA_WHATAMI_AXI2PICO        0x0C
#define  PA_WHATAMI_DL_SYM          0x0D
#define  PA_WHATAMI_DL_HS_TB        0x1D
#define  PA_WHATAMI_DL_HS_RM        0x2D
#define  PA_WHATAMI_DL_HS_SCCH      0x3D
#define  PA_WHATAMI_DL_HS_CHIP      0x4D
#define  PA_WHATAMI_DL_CHIP         0x5D
#define  PA_WHATAMI_DL_SAMP         0x6D
#define  PA_WHATAMI_DL_RX_SCH       0x7D
#define  PA_WHATAMI_UL_SAMP         0x0E
#define  PA_WHATAMI_UL_DSPRD        0x1E
#define  PA_WHATAMI_UL_RACH         0x2E
#define  PA_WHATAMI_UL_EDPDCH       0x3E
#define  PA_WHATAMI_UL_SYM          0x4E
#define  PA_WHATAMI_UL_SRCH         0x5E
#define  PA_WHATAMI_UL_TB_FMT       0x6E
#define  PA_WHATAMI_UL_VITERBI      0x7E
#define  PA_WHATAMI_UL_TURBO        0x8E

#define PA_CONFIG_WRITE            0x00010000
#define PA_CONFIG_READ             0x00020000
#define PA_CONFIG_ADDR             0x00040000
#define PA_CONFIG_AEID             0x00080000
#define PA_CONFIG_VALID            0x00010000
#define PA_CONFIG_FAIL             0x00020000

#define PA_STATUS_VALID		   0
#define PA_STATUS_FAIL		   1
#define PA_STATUS_TIMEOUT	   2

#endif /* PC302_PA_H */
