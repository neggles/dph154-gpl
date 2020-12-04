/*
 * FILE NAME leddriver_procfs.c
 *
 * Copyright (c) 2008 ip.access Ltd.
 *
 * BRIEF MODULE DESCRIPTION
 *  Driver for LED (procfs handler)
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

/****************************************************************************
 * Standard Library Includes
 ****************************************************************************/ 
#include "leddriver.h"

/****************************************************************************
  Private Definitions
 ****************************************************************************/

/****************************************************************************
  Private Types
 ****************************************************************************/


 /****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int proc_write( struct file *file, const char __user *buffer,
               unsigned long count, void *data);
static int proc_read (char    *buf_p,   char    **start_p,
               off_t   offset, int     count,   int     *eof_p, void    *data_p);

static int update_pattern_entry( int devNum, int patternIdx, int onPeriodMs, int offPeriodMs);

static int update_sequence_entry( int devNum, char sequenceLetter, char *sequenceName,
                     int numStates, LedSequenceEntry *sequences_p );

static int print_patterns( char *buf_p, int count, int devNum );
static int print_sequences( char *buf_p, int count, int devNum );

static int modify_pattern_entry(int devNum, int patternIdx, int onPeriodMs, int offPeriodMs);
static int modify_sequence_entry( int devNum, char sequenceLetter, char *sequenceName,
                     int numStates, LedSequenceEntry *sequences_p );
                     
static int update_pattern_entry( int devNum, int patternIdx, int onPeriodMs, int offPeriodMs);
static int update_sequence_entry( int devNum, char sequenceLetter, char *sequenceName,
                     int numStates, LedSequenceEntry *sequences_p );
static int process_sequence_entries(char *buf, char **ptr_p, int numStates, LedSequenceEntry *sequences_p);

/****************************************************************************
 * Private Constants
 ****************************************************************************/
#define PATTERN_PARSER_EXIT -1
#define FIND_DEV_INSTANCE   20
#define FIND_PATTERN_IDX    21
#define FIND_ON_PERIOD      22
#define FIND_OFF_PERIOD     23

#define PATTERN_MAGIC_VAL   100
#define SEQUENCE_MAGIC_VAL  200

/****************************************************************************
 * Exported Variables
 ****************************************************************************/
 
/****************************************************************************
 * Private Variables (Must be declared static)
 ****************************************************************************/
static struct proc_dir_entry * proc_leddriver = NULL;

 /****************************************************************************
 * Function Name  : remove_proc_fs entries
 * Description    : Removes the 'files' in /proc/
 ****************************************************************************/
void remove_proc_fs( void )
{

    remove_proc_entry( "patterns", proc_leddriver );
    remove_proc_entry( "sequences", proc_leddriver );
    remove_proc_entry( "driver/leddriver", NULL );

}
 /****************************************************************************
 * Function Name  : create_proc_fs entries
 * Description    : creates the 'files' in /proc/
 ****************************************************************************/
void create_proc_fs( void )
{
    struct proc_dir_entry * patterns;
    struct proc_dir_entry * sequences;

    PDEBUG("Adding procfs entries... \n");
    if (!proc_leddriver)
    {
        PDEBUG("Adding proc/driver/leddriver...\n");

        proc_leddriver = proc_mkdir( "driver/leddriver",
                                NULL);
    }

    if (proc_leddriver)
    {
        patterns = create_proc_entry( "patterns",
                                S_IFREG | S_IWUSR,
                                proc_leddriver);
                                    
        if (patterns)
        {
            patterns->read_proc = proc_read;
            patterns->write_proc = proc_write;
            patterns->data = (void*)PATTERN_MAGIC_VAL;
        }
        else
        {
            PDEBUG("Failed to add /proc/driver/leddriver/patterns !\n");
        }
    
    
        sequences = create_proc_entry( "sequences",
                                    S_IFREG | S_IWUSR,
                                    proc_leddriver);
        if (sequences)
        {
            sequences->read_proc = proc_read;
            sequences->write_proc = proc_write;
            sequences->data = (void*)SEQUENCE_MAGIC_VAL;
        }
        else
        {
            PDEBUG("Failed to add /proc/driver/leddriver/sequences !\n");
        }
    }
    else
    {
        PDEBUG("Failed to add /proc/driver/leddriver folder !\n");
    }
}

/*
# format of patterns (up to 10 patterns 0-9 can be added) in the form devInstance:PatternIndex, On period, Off period
#   0:0,50,50
#   0:1,50,200
#   0:2,300,50
#   0:3,200,1800
#   0:4,1000,1000
#   0:5,1800,200
*/
/****************************************************************************
 * Function Name  : parse_pattern_buffer
 * Description    : Parses /proc/drivers/leddriver/patterns buffer. Can also be called
 *                  from intialisation to provide default patterns
 ****************************************************************************/
int parse_pattern_buffer( char* buf, unsigned long count )
{
    char *ptr = buf;
    int limit = 0;
    int devNum = 0, patternIdx = 0, onPeriodMs = 50, offPeriodMs = 50;

    printk("procesing ...\n%s\n", buf);

    while ( sscanf(ptr, "%d:%d,%d,%d\n", &devNum, &patternIdx, &onPeriodMs, &offPeriodMs) == 4 )
    {
        if( update_pattern_entry( devNum, patternIdx, onPeriodMs, offPeriodMs) == -1)
        {
            break; // Something went wrong
        }
        

        ptr++;
        ptr = strstr(ptr,"\n"); // find the next CR
        if (NULL == ptr)
            break; // we can't find any more lines - exit

        // Catch endless loops that otherwise may take down the board
        if ( ++limit > 100 )
            break;
    }   

    return (count);
}
/*
# format of sequences (up to 12 Sequences A-L can be added)
#    in the form devInstance:SequenceIndex,numStates,"SequenceName",
        {Pattern, gpioOnHigh, gpioOnLow,gpioOffHigh,gpioOffLow,RepeatCount,Del before next patt, nextState}
#"0:A,1,\"No IP Address (off)\",{0,32,0,32,0,-1,0,0}\n"
#"0:B,1,\"Not Provisioned\",{5,36,0,32,4,-1,0,0}\n"
#"0:C,1,\"Provisioned\",{6,36,0,32,4,-1,0,0}\n"
#"0:D,1,\"In Test\",{6,36,0,32,4,-1,0,0}\n"
#"0:E,1,\"No Service\",{7,36,0,32,4,-1,0,0}\n"   // Alt is : {7,96,4,32,68,-1,0,0}
#"0:F,1,\"Service Available (on)\",{1,36,0,32,4,-1,0,0}\n"
#"0:G,1,\"Button Pressed\",{2,36,0,32,4,-1,0,0}\n"
#"0:H,1,\"Factory Restore\",{3,36,0,32,4,-1,0,0}\n"
#"0:I,1,\"Firmware Upgrade\",{4,36,0,32,4,-1,0,0}\n"
#"0:J,1,\"Fault (red)\",{1,0,36,0,36,-1,0,0}\n"  // Fault uses Sys LED Red (Turn 32 OFF!)
#"0:K,3,\"SOS\",{3,36,0,32,4,3,500,1},{4,36,0,32,4,3,500,2},{3,36,0,32,4,3,1000,0}\n";
*/
/****************************************************************************
 * Function Name  : parse_sequence_buffer
 * Description    : Parses /proc/drivers/leddriver/sequences buffer. Can also be called
 *                  from intialisation to provide default sequences
 ****************************************************************************/
int parse_sequence_buffer( char* buf, unsigned long count )
{
    char *ptr = buf;
    char *oldPtr, *start_p, *end_p;
    int limit = 0, bCont;
    int devNum = 0, numStates = 1, actualNumStates;
    char sequenceLetter;
    char sequenceName[SEQUENCE_NAME_LEN];
    LedSequenceEntry *sequences_base_p = NULL;
    
    sequences_base_p = kmalloc( MAX_NUM_STATES * sizeof(LedSequenceEntry), GFP_KERNEL );
    if (!sequences_base_p)
    {
        // Abort
        printk("Kernel malloc failed to allocate sequences\n");
        return count;
    }

    printk("procesing ...\n%s\n", ptr);
    
    bCont = 1;
    while ( bCont )
    {
        if ( sscanf(ptr, "%d:%c,%d", &devNum, &sequenceLetter,&numStates) == 3)
        {
            /* Find sequenceName */
            end_p = NULL;
            start_p = strstr(ptr,"\"");
            if( start_p )
            {
                start_p++; // move to next char after "
                end_p = strstr(start_p,"\"");
                if( end_p )
                {   
                    *end_p = '\0';
                    strncpy((char*) &sequenceName, start_p, sizeof(sequenceName)-1);
                    sequenceName[SEQUENCE_NAME_LEN-1] = '\0';
                    ptr = end_p + 1;
                }
            }
        
            if ( start_p == NULL || end_p == NULL )
            {
                bCont = 0;
                printk("ptr=%d, can't find quotes for sequenceName! (%s)\n",start_p-buf, start_p);
                break;  // We can't find a quote
            }
                    
            if (count < (ptr-buf))
            {
                bCont = 0;
                printk("ptr=%d, not enough chars left in string to continue\n",start_p-buf);
                break;
            }

            if (numStates <1 || numStates >= MAX_NUM_STATES)
            {
                bCont = 0;
                printk("ptr=%d, numStates = %d (out of range!)\n",start_p-buf, numStates);
                break;
            }
            
            memset(sequences_base_p, 0, MAX_NUM_STATES * sizeof(LedSequenceEntry));
            actualNumStates = process_sequence_entries(buf, &ptr, numStates, sequences_base_p);
            if (actualNumStates != numStates)
            {                            
                bCont = 0;
                printk("ptr=%d, number of states (%d) doesn't match for sequence %s\n",ptr-buf, numStates, sequenceName);
                break;                            
            }
            update_sequence_entry( devNum, sequenceLetter, sequenceName, numStates, sequences_base_p);
        }
        else
        {
            bCont = 0;
        }

        /* Try to process the next sequence */
        oldPtr = ptr;
        ptr = strstr(oldPtr,"\n");
        if (NULL == ptr) break;
        
        // Catch endless loops that otherwise may take down the board
        if ( ++limit > 100 ) break;
    }   
    
    kfree(sequences_base_p);
    
    return (count);
} 

/************************* LOCAL FUNCTIONS BELOW ****************************/

/****************************************************************************
 * Function Name  : proc_read
 * Description    : Handles reads to proc filesystem
 ****************************************************************************/
static int proc_read (char    *buf_p,
                        char    **start_p,
                        off_t   offset,
                        int     count,
                        int     *eof_p,
                        void    *data_p)
{
    int                 len = 0;
    int                 devNum;
    int magicValue = (int) data_p;
            
    
    len += sprintf (buf_p + len,"\nleddriverNumDevs = %d\n", leddriverNumDevs);
    
    for (devNum = 0; devNum < leddriverNumDevs; devNum++)
    {
        if ( magicValue == PATTERN_MAGIC_VAL )
        {
            len += print_patterns(buf_p + len, count-len, devNum );
        }
        else if ( magicValue == SEQUENCE_MAGIC_VAL )
        {
            len += print_sequences(buf_p + len, count-len, devNum );
        }
    }
        
    *eof_p = 1;
    return len;
}
/****************************************************************************
 * Function Name  : proc_write
 * Description    : Handles writes to proc filesystem
 ****************************************************************************/
 static int proc_write( struct file *file, const char __user *buffer,
               unsigned long count, void *data)
{
    char *localBuf;
    int magicValue = (int) data;
    
    if (count > 1024)
    {
        count = 1024;
    }

    localBuf = kmalloc(count+1,GFP_KERNEL);

    if (localBuf)
    {
        if (copy_from_user(localBuf, buffer, count))
        {
            kfree(localBuf);
            return -EFAULT;
        }
        else
        {
            localBuf[count] = '\0'; // Null term string

            if ( magicValue == PATTERN_MAGIC_VAL )
            {
                count = parse_pattern_buffer(localBuf, count );
            }
            else if ( magicValue == SEQUENCE_MAGIC_VAL )
            {
                count = parse_sequence_buffer(localBuf, count );
            }
            kfree(localBuf);

            return count;
        }

    }
    return -EFAULT;
        
}

/****************************************************************************
 * Function Name  : print_patterns
 * Description    : Dumps pattern enries for a particular device
 *                  buf is a buffer to sprintf into, Count is size of buffer
 ****************************************************************************/
static int print_patterns( char *buf_p, int count, int devNum )
{
    int i;
    int len = 0;
    LeddriverDev *dev_p = &leddev[devNum];
            
    for( i = 0; i < dev_p->numPatterns; i++)
    {
        len += sprintf (buf_p + len,"%d:%d,%d,%d\n",
                            devNum,
                            i,
                            dev_p->ledPattenTable[i].onPeriodMs,
                            dev_p->ledPattenTable[i].offPeriodMs);
    }

    return len;
}
/****************************************************************************
 * Function Name  : print_sequences
 * Description    : Dumps sequence enries for a particular device
 *                  buf is a buffer to sprintf into, Count is size of buffer
 ****************************************************************************/
static int print_sequences( char *buf_p, int count, int devNum )
{
    int j;
    int i;
    int len = 0;
    LeddriverDev *dev_p = &leddev[devNum];
    LedSequenceEntry * sequences_p;
    
    for ( j = 0; j < dev_p->numSequences ; j++)
    {
        int numStates = dev_p->sequenceTable[j].numStates;
        
        len += sprintf (buf_p + len,"%d:%c,%d,\"%s\"",
                        devNum,
                        'A' + j,
                        numStates,
                        dev_p->sequenceTable[j].name);

        sequences_p = dev_p->sequenceTable[j].sequenceEntry;
        
        for( i = 0; i < numStates; i++)
        {
            len += sprintf (buf_p + len, ",{%d,%d,%d,%d,%d,%d,%d,%d}",
                        sequences_p->pattern,
                        sequences_p->gpioOnHigh,
                        sequences_p->gpioOnLow,
                        sequences_p->gpioOffHigh,
                        sequences_p->gpioOffLow,
                        sequences_p->repeat,
                        sequences_p->delay_before_next_pattern_ms,
                        sequences_p->nextState );
            sequences_p++;
            
            if (len > count - 100) // Allow some headroom    
                break;
        }
        
        len += sprintf (buf_p + len,"\n");
        if (len > count - 100)
            break;
    }
    
    return len;
}
/****************************************************************************
 * Function Name  : modify_pattern_entry
 * Description    : Actually modifies the pattern table - assumes it is safe
 *                  to use this data structure prior to use.
 ****************************************************************************/
static int modify_pattern_entry(int devNum, int patternIdx, int onPeriodMs, int offPeriodMs)
{
    LeddriverDev *dev_p;
     
    if(devNum < 0 || devNum >= leddriverNumDevs)
    {
        printk("devNum is out of range! (%d)\n", devNum);
        return -1;
    }
    if( patternIdx >= MAX_NUM_PATTERNS )
    {
        printk("patternIdx is out of range! (%d)\n", patternIdx);
        return -1;
    }
    
    dev_p = &leddev[devNum];

    /* Get spinlock */
    ACQUIRE(dev_p);
    
    dev_p->ledPattenTable[patternIdx].onPeriodMs = onPeriodMs;
    dev_p->ledPattenTable[patternIdx].offPeriodMs = offPeriodMs;

    /* Update the largest valid pattern index - Note we can't decrease this by design */
    if ( patternIdx >= dev_p->numPatterns)
    {
        dev_p->numPatterns = patternIdx + 1;
    }

    /* Release spinlock */
    RELEASE(dev_p);
    
    return 0;
}
/****************************************************************************
 * Function Name  : update_pattern_entry
 * Description    : Validates values are in range before calling modify function
 *                  - assumes it is safe to use this data structure prior to use.
 ****************************************************************************/
static int update_pattern_entry( int devNum, int patternIdx, int onPeriodMs, int offPeriodMs)
{
    PDEBUG("update_pattern_entry: %d %d %d %d\n", devNum, patternIdx, onPeriodMs, offPeriodMs);
    
    if ( devNum < 0 || devNum > leddriverNumDevs)
    {
        printk("devNum (%d) is out of range, should be 0-%d\n",devNum, leddriverNumDevs);
        return -1;
    }

    if ( patternIdx < 0 || patternIdx >= MAX_NUM_PATTERNS)
    {
        printk("patternIdx (%d) is out of range, should be 0-" MAX_NUM_PATTERNS_STR "\n",patternIdx);
        return -1;
    }
    
    if ( (onPeriodMs != ON_IGNORE) && (onPeriodMs != ON_PERMANENT) && (onPeriodMs < 50 ))
    {
        printk("onPeriodMs (%d) is out of range, should be >50 ms\n",onPeriodMs);
        return -1;
    }

    if ( (offPeriodMs != OFF_IGNORE) && (offPeriodMs != OFF_PERMANENT) && (offPeriodMs < 50 ))
    {
        printk("offPeriodMs (%d) is out of range, should be >50 ms\n",offPeriodMs);
        return -1;
    }

    return (modify_pattern_entry( devNum, patternIdx,  onPeriodMs, offPeriodMs));
}

/**************************** SEQUENCES BELOW *******************************/

/****************************************************************************
 * Function Name  : modify_sequence_entry
 * Description    : Actually modifies the sequence table - assumes it is safe
 *                  to use this data structure prior to use.
 ****************************************************************************/
static int modify_sequence_entry( int devNum, char sequenceLetter, char *sequenceName,
                     int numStates, LedSequenceEntry *sequences_p )
{
    LeddriverDev *dev_p;
    int sequenceNum;
    int i;
     
    if(devNum < 0 || devNum >= leddriverNumDevs)
    {
        printk("devNum is out of range! (%d)\n", devNum);
        return -1;
    }
    if( numStates >= MAX_NUM_STATES )
    {
        printk("numStates is out of range! (%d)\n", numStates);
        return -1;
    }

    dev_p = &leddev[devNum];

    sequenceNum = sequenceLetter - 'A';
    if ( (sequenceNum < 0) || (sequenceNum >= MAX_NUM_SEQUENCES) )
    {
        printk("sequenceLetter (%c) out of range\n", sequenceLetter);
        return -1;
    }

    /* Get spinlock */
    ACQUIRE(dev_p);
    
    strncpy(dev_p->sequenceTable[sequenceNum].name, sequenceName, SEQUENCE_NAME_LEN-1);
    dev_p->sequenceTable[sequenceNum].name[SEQUENCE_NAME_LEN-1] ='\0';
    dev_p->sequenceTable[sequenceNum].numStates = numStates;

    for (i = 0; i < numStates; i++ )
    {  
        dev_p->sequenceTable[sequenceNum].sequenceEntry[i] = *sequences_p;
        sequences_p++;
    }

    /* Update the largest valid sequence index - Note we can't decrease this by design */
    if ( sequenceNum >= dev_p->numSequences)
    {
        dev_p->numSequences = sequenceNum + 1;
    }   

    /* Release spinlock */
    RELEASE(dev_p);
    
    return 0;   
}

/****************************************************************************
 * Function Name  : update_sequence_entry
 * Description    : Validates values are in range before calling modify function
 *                  - assumes it is safe to use this data structure prior to use.
 ****************************************************************************/
static int update_sequence_entry( int devNum, char sequenceLetter, char *sequenceName,
                     int numStates, LedSequenceEntry *sequences_p )
{
    int i;
    LedSequenceEntry * base_p = sequences_p;

    for( i = 0; i < numStates; i++)
    {
        sequences_p++;
        /* validate that these are sensible values */
        if (sequences_p->pattern < 0 || sequences_p->pattern > MAX_NUM_PATTERNS)
        {
            printk("In state %d pattern (%d) is out of range, should be 0-" MAX_NUM_PATTERNS_STR "\n",
                        i, sequences_p->pattern);
            return -1;
        }
        
        if (sequences_p->nextState < 0 || sequences_p->nextState >= numStates)
        {
            printk("In state %d nextState (%d) is out of range, should be 0-%d\n",
                        i, sequences_p->nextState, numStates);
            return -1;
        }       
    }
    
    if ( devNum < 0 || devNum > leddriverNumDevs)
    {
        printk("devNum (%d) is out of range, should be 0-%d\n",devNum, leddriverNumDevs);
        return -1;
    }

    if ( sequenceLetter < 'A' || (sequenceLetter >= 'A' + MAX_NUM_SEQUENCES))
    {
        printk("sequenceLetter (%c) is out of range, should be A-%c\n",sequenceLetter, 'A' + MAX_NUM_SEQUENCES);
        return -1;
    }
    
    if ( numStates < 1 || numStates > MAX_NUM_STATES)
    {
        printk("numStates (%d) is out of range, should be 1-" MAX_NUM_STATES_STR "\n",numStates);
        return -1;
    }

    /* Everything looks okay */
    PDEBUG("Trying to add %d %c \"%s\" %d...\n",devNum, sequenceLetter, sequenceName, numStates);
    return ( modify_sequence_entry(devNum, sequenceLetter, sequenceName, numStates, base_p));
}

/****************************************************************************
 * Function Name  : process_sequence_entries
 * Description    : Parses the sequence entries and populates array of sequnces
 ****************************************************************************/
static int process_sequence_entries(char *buf, char **ptr_p, int numStates, LedSequenceEntry *sequences_p)
{
    int i, actualNumStates = 0;
    char *oldPtr, *ptr = *ptr_p;
    int pattern, gpioOnHigh, gpioOnLow, gpioOffHigh, gpioOffLow, repeat, delay_before_next_pattern_ms, nextState;
    
    for (i = 0; (i < numStates) && (i < MAX_NUM_STATES); i++)
    {
//printk("DBG: %d, %s\n", i, ptr);
        oldPtr = ptr;
        ptr = strstr(ptr,"{"); // find the next '{'
        if (NULL == ptr)
        {
            printk("ptr=%d, Can't find any more '{' in %s on %d)\n", oldPtr-buf, oldPtr, i);
            break;
        }
        if ( sscanf(ptr, "{%d,%d,%d,%d,%d,%d,%d,%d}",  &pattern, &gpioOnHigh, &gpioOnLow, &gpioOffHigh,
                &gpioOffLow, &repeat, &delay_before_next_pattern_ms, &nextState ) == 8 )
        {
            sequences_p->pattern = pattern;
            sequences_p->gpioOnHigh = gpioOnHigh;
            sequences_p->gpioOnLow = gpioOnLow;
            sequences_p->gpioOffHigh = gpioOffHigh;
            sequences_p->gpioOffLow = gpioOffLow;
            sequences_p->repeat = repeat;
            sequences_p->delay_before_next_pattern_ms = delay_before_next_pattern_ms;
            sequences_p->nextState = nextState;
            sequences_p++;
            actualNumStates++;
            ptr++;  // skip the open { and next time we should find the next entry
        }
        else
        {
            printk("ptr: %d - failed to parse %s\n", ptr-buf, ptr );
            break;
        }
    }

    // Update return variables
    *ptr_p = ptr;
    
    if (i != numStates)
    {
        return 0;   // Error
    }
    else
    {
        return actualNumStates;
    }
     
}
/* end of leddriver_procfs.c */
