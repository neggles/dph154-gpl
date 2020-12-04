/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file timer.c
* \brief Useful functions for timer implementation.
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

/*
 * (C) Copyright 2003
 * Texas Instruments <www.ti.com>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Alex Zuepke <azu@sysgo.de>
 *
 * (C) Copyright 2002-2004
 * Gary Jennejohn, DENX Software Engineering, <gj@denx.de>
 *
 * (C) Copyright 2004
 * Philippe Robin, ARM Ltd. <philippe.robin@arm.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <arm926ejs.h>
#include <asm/arch/timer.h>

/*
 * The timer is a decrementer, it runs at a frequency of CONFIG_SYS_HZ.
 */

/* We have CONFIG_SYS_HZ timer ticks per second */
#define TICKS_PER_HZ        (CONFIG_SYS_HZ)

/* The number of clock ticks per 1 uS */
#define TICKS_PER_US        (TICKS_PER_HZ / (1000 * 1000))

/* The number of clock ticks per 'x' uS */
#define USEC_TO_COUNT(x)    ((x) * TICKS_PER_US)

/* The time (in seconds) for 'x' timer ticks */
#define TICKS_TO_HZ(x)	    ((x) / TICKS_PER_HZ)

/* Macro to read the 32 bit timer.
 * Since the timer decrements, we invert the read value
 * to give us an incrementing count value.
 */
#define READ_TIMER()        ( ~(*(volatile unsigned int *)\
                                (CFG_TIMERBASE +\
                                 TIMERNCURRENTVALUEREGOFFSET(0))) )

/* Timer init function */
int timer_init(void)
{
	/* Our timer will have already been started,
         * therefore we do nothing.
         */

        return 0;
}

/* Restart counting from 0 */
void reset_timer (void)
{
        /* Our timer is a free running timer,
         * we never restart it.
         */
}

/* Return how many HZ passed since "base" */
ulong get_timer (ulong base)
{
	return  TICKS_TO_HZ(READ_TIMER()) - base;
}

/* Delay 'usec' micro seconds */
void udelay (unsigned long usec)
{
	ulong current, end;

	/* Read the current timer value */
        current = READ_TIMER();

        /* Work out what the final timer value needs to be */
	end = current + USEC_TO_COUNT(usec);

        while ((signed)(end - READ_TIMER()) > 0)
        {
            /* Wait while counting... */
	    continue;
        }
}
