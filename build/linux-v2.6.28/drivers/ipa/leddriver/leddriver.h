/*
 * FILE NAME leddriver.h
 *
 * Copyright (c) 2007 ip.access Ltd.
 *
 * BRIEF MODULE DESCRIPTION
 *  Driver for LED
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#if !defined ( INCLUDED_LEDDRIVER_H )
#define INCLUDED_LEDDRIVER_H

/****************************************************************************
 * Standard Library Includes
 ****************************************************************************/ 
# if defined( USER_SPACE_TESTHARNESS )
#  include <sys/time.h>
#  include <signal.h>
#  include <stdlib.h>
#  include <stdio.h>
# else   // KERNEL Module
#  include <linux/timer.h>
#  include <linux/module.h>
#  include <linux/moduleparam.h>
#  include <linux/init.h>
#  include <linux/kernel.h>   /* printk() */
#  include <linux/slab.h>     /* kmalloc() */
#  include <linux/fs.h>       /* everything... */
#  include <linux/errno.h>    /* error codes */
#  include <linux/types.h>    /* size_t */
#  include <linux/proc_fs.h>
#  include <asm/uaccess.h>
#  include <linux/jiffies.h>
#  include <linux/cdev.h>
#  include <linux/string.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/
#if defined(LEDDRIVER_DEBUG)
#   if defined(USER_SPACE_TESTHARNESS)
#       define PDEBUG(fmt, args...) printf( "leddriver: " fmt, ## args)
#   else
#       define PDEBUG(fmt, args...) printk( KERN_DEBUG "leddriver: " fmt, ## args)
#   endif /* USER_SPACE_TESTHARNESS */
#else
#   define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif


/* Macros for thread serialisation */
#define ACQUIRE(iNSTANCE)  \
    spin_lock_irqsave(&(iNSTANCE->spinlock), iNSTANCE->spinlock_flags);

#define RELEASE(iNSTANCE)  \
    spin_unlock_irqrestore(&(iNSTANCE->spinlock), iNSTANCE->spinlock_flags);

/****************************************************************************
 * Constants
 ****************************************************************************/
#define INDEFINITE_REPEAT       -1
#define ON_PERMANENT            1
#define ON_IGNORE               0
#define OFF_IGNORE              0
#define OFF_PERMANENT           1

#define MAX_NUM_LED_INSTANCES   2
#define MAX_NUM_PATTERNS        10
#define MAX_NUM_PATTERNS_STR    "10"
#define MAX_NUM_STATES          10
#define MAX_NUM_STATES_STR      "10"
#define MAX_NUM_SEQUENCES       12
#define SEQUENCE_NAME_LEN       30

/****************************************************************************
 * Types
 ****************************************************************************/
typedef enum 
{
    LED_ON,
    LED_OFF
}LedState;

typedef struct {
    int onPeriodMs;
    int offPeriodMs;
} LedFlashPattern;

typedef struct {
    int pattern;
    unsigned char gpioOnHigh;
    unsigned char gpioOnLow;
    unsigned char gpioOffHigh;
    unsigned char gpioOffLow;
    int repeat;
    int delay_before_next_pattern_ms;
    int nextState;
} LedSequenceEntry;

typedef struct
{
    char name[SEQUENCE_NAME_LEN];
    LedSequenceEntry sequenceEntry[MAX_NUM_STATES];
    int numStates;
} SequencePatternTable;


typedef struct {
    LedSequenceEntry *currentSequence_p;
    int currentSequenceIndex;
    int state;
    LedState  ledState;
    int repeatCount;
} LedStateMachine;

/*
 * Device structure
 */
typedef struct LeddriverDevTag
{
    int             instance;
    char            sequenceCode;
    int             numPatterns;
    int             numSequences;
    LedFlashPattern *ledPattenTable;
    SequencePatternTable     *sequenceTable;
    LedStateMachine stateMachine;
    
#if !defined (USER_SPACE_TESTHARNESS)
    struct cdev     cdev;
    struct timer_list ledTimer;
    spinlock_t spinlock;
    unsigned long spinlock_flags;
#endif
} LeddriverDev;
 /****************************************************************************
 * Function Prototypes
 ****************************************************************************/
extern void create_proc_fs( void );
extern void remove_proc_fs( void );
extern int parse_pattern_buffer( char* buf, unsigned long count );
extern int parse_sequence_buffer( char* buf, unsigned long count );

/****************************************************************************
 * Exported Variables
 ****************************************************************************/
extern int leddriverNumDevs;
extern LeddriverDev leddev[MAX_NUM_LED_INSTANCES];
 
#endif /* INCLUDED_LEDDRIVER_H */
