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

#ifndef PC302_RTC_H
#define PC302_RTC_H

/* Constants -------------------------------------------------------------- */

/*****************************************************************************/
/* Register Offset Addresses                                                 */
/*****************************************************************************/

#define RTCLK_CCV_REG_OFFSET                0x00
#define RTCLK_CM_REG_OFFSET                 0x04
#define RTCLK_CL_REG_OFFSET                 0x08
#define RTCLK_CC_REG_OFFSET                 0x0C
#define RTCLK_STAT_REG_OFFSET               0x10
#define RTCLK_RSTAT_REG_OFFSET              0x14
#define RTCLK_EOI_REG_OFFSET                0x18
#define RTCLK_COMP_VERSION_REG_OFFSET       0x1C

/*****************************************************************************/;
/* Register Reset Values                                                     */;
/*****************************************************************************/;

#define RTCLK_CCV_REG_RESET                 0x00000000
#define RTCLK_CM_REG_RESET                  0x00000000
#define RTCLK_CL_REG_RESET                  0x00000000
#define RTCLK_CC_REG_RESET                  0x00000000
#define RTCLK_STAT_REG_RESET                0x00000000
#define RTCLK_RSTAT_REG_RESET               0x00000000
#define RTCLK_EOI_REG_RESET                 0x00000000
#define RTCLK_COMP_VERSION_REG_RESET        0x3230312a

/*****************************************************************************/
/* Register Bit Field Manipulation                                           */
/*****************************************************************************/

#define RTCLK_CC_REG_MASK_IDX	            1
#define RTCLK_CC_REG_EN_IDX		    0

#define RTCLK_CC_REG_MASK_MASK	            1 << RTCLK_CC_REG_MASK_IDX
#define RTCLK_CC_REG_EN_MASK	            1 << RTCLK_CC_REG_EN_IDX

#endif /* PC302_RTCG_H */
