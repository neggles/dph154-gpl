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
 *              Remap Block Register Definitions
 *
 *****************************************************************************/
#ifndef __PC20X_RAP_H
#define __PC20X_RAP_H
 
#define RapPauseModeRegOffset   0x00
#define RapIdCodeRegOffset	0x04
#define RapRemapModeRegOffset	0x08
#define RapResetStatusRegOffset	0x0c
#define RapResetStatusClrOffset	0x10
#define RapCompVersionOffset	0x14

#define RapIdCodeValue		0x00000000

#define RapResetStatusSoftIdx	0
#define RapResetStatusWdogIdx	1
#define RapResetStatusGlblIdx	2

#define RapResetStatusSoftMask	(1 << RapResetStatusSoftIdx)
#define RapResetStatusWdogMask	(1 << RapResetStatusWdogIdx)
#define RapResetStatusGlblMask	(1 << RapResetStatusGlblIdx)

#define RapPauseModeRegReset	0x00000000
#define RapIdCodeRegReset	RapIdCodeValue
#define RapRemapModeRegReset	0x00000000
#define RapResetStatusRegReset	0x00000000
#define RapResetStatusClrReset	0x00000000
#define RapCompVersionReset	0x3230322b

#define REMAP_NORMAL_MODE       0x00000001

#endif /* __PC20X_RAP_H */
