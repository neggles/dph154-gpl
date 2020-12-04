/**
 * \file pc302fracn.h
 *
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 * Copyright 2008 picoChip Designs LTD, All Rights Reserved.
 * http://www.picochip.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * This file defines an API for configuring the Fractional-N synthesizer in
 * the picoChip PC302 device.
 */
#ifndef __PC302FRACN_H__
#define __PC302FRACN_H__

/* Frac-N status register bits. */

/* Running fast offset and mask. */
#define FRACN_RUNNING_FAST_OFFSET ( 15 )
#define FRACN_RUNNING_FAST_MASK ( 1 << FRACN_RUNNING_FAST_OFFSET )

/* Running slow offset and mask. */
#define FRACN_RUNNING_SLOW_OFFSET ( 14 )
#define FRACN_RUNNING_SLOW_MASK ( 1 << FRACN_RUNNING_SLOW_OFFSET )

/* Control voltage pulse width offset and mask. */
#define FRACN_CTRL_V_PULSE_WIDTH_OFFSET ( 3 )
#define FRACN_CTRL_V_PULSE_WIDTH_MASK \
    ( 0x1FFF << FRACN_CTRL_V_PULSE_WIDTH_OFFSET )

/* VCXO control voltage under limit offset and mask. */
#define FRACN_CTRL_V_UNDER_LIMIT_OFFSET ( 2 )
#define FRACN_CTRL_V_UNDER_LIMIT_MASK ( 1 << FRACN_CTRL_V_UNDER_LIMIT_OFFSET )

/* VCXO control voltage over limit offset and mask. */
#define FRACN_CTRL_V_OVER_LIMIT_OFFSET ( 1 )
#define FRACN_CTRL_V_OVER_LIMIT_MASK ( 1 << FRACN_CTRL_V_OVER_LIMIT_OFFSET )

/* Frequency synthesizer not locked offset and mask. */
#define FRACN_NOT_LOCKED_OFFSET ( 0 )
#define FRACN_NOT_LOCKED_MASK ( 1 << FRACN_NOT_LOCKED_OFFSET )

/**
 * Get the current M value.
 *
 * \param val The destination to store the value of M.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_get_m( u8 *val );

/**
 * Set a new value of M.
 *
 * \param val The new value of M.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_set_m( u8 val );

/**
 * Get the current N value.
 *
 * \param val The destination to store the value of N.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_get_n( u8 *val );

/**
 * Store a new value of N.
 *
 * \param val The new value of N.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_set_n( u8 val );

/**
 * Get the current K value.
 *
 * \param val The destination to store the value of K.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_get_k( u32 *val );

/**
 * Store a new value of K.
 *
 * \param val The new value of K.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_set_k( u32 val );

/**
 * Get the current control voltage pulse lower limit.
 *
 * \param val The destination to store the result in.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_get_cv_pulse_ll( u16 *val );

/**
 * Set a new control voltage pulse lower limit.
 *
 * \param val The new value to set.
 * \return Returns zero on success, non-zero on failure. */
static int fracn_set_cv_pulse_ll( u16 val );

/**
 * Get the current control voltage pulse upper limit.
 *
 * \param val The destination to store the result in.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_get_cv_pulse_ul( u16 *val );

/**
 * Set a new control voltage pulse upper limit.
 *
 * \param val The new value to set.
 * \return Returns zero on success, non-zero on failure. */
static int fracn_set_cv_pulse_ul( u16 val );

/**
 * Get the contents of the Frac-N status register.
 *
 * \param val The destination to store the status in.
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_get_status( u16 *val );

/**
 * Reset the Frac-N synth.
 *
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_reset( void );

/**
 * Load the new values of M, N and K into the Frac-N synth.
 *
 * \return Returns zero on success, non-zero on failure.
 */
static int fracn_load( void );

#endif /* __PC302FRACN_H__ */
