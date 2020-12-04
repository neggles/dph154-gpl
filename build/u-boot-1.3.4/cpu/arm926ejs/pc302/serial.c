/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/
 
/*!
* \file
* \brief serial.c
*
* Copyright (c) 2006-2008 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/ 

#include <common.h>

#ifdef CFG_DW_APB_UART

/* Includes ----------------------------------------------------------------- */
#include <asm/arch/pc302.h>
#include <asm/arch/uart.h>

/* Macros ------------------------------------------------------------------- */
#if !defined(CONFIG_CONS_INDEX)
#error	"No console index specified."
#elif (CONFIG_CONS_INDEX < 1) || (CONFIG_CONS_INDEX > 2)
#error	"Invalid console index value specified."
#endif

#define IO_WRITE(addr, val) (*(volatile unsigned int *)(addr) = (val))
#define IO_READ(addr) (*(volatile unsigned int *)(addr))

/* uart[] array is zero based whilst CONFIG_CONS_INDEX is '1' based */
#define CONSOLE_PORT (CONFIG_CONS_INDEX - 1)

/* Constants ---------------------------------------------------------------- */
static volatile unsigned char *const uart[] = {
                                                (void *)PC302_UART1_BASE,
                                                (void *)PC302_UART2_BASE
                                              };

/* Prototypes---------------------------------------------------------------- */
/*!
 *
 * Write a character to the uart
 *
 * Note: This function will block if the Tx FIFO is full
 *
 * \param uartNumb Which uart to use
 * \param c The character to output
 *
 */ 
static void dwApbUart_putc (int uartNum, char c);

/*!
 *
 * Read a character from the uart
 *          
 * Note: This function will block if the Rx FIFO is empty
 *
 * \param uartNumb Which uart to use   
 */   
static int dwApbUart_getc (int uartNum);

/*!
 *
 * Test if there is a character available to be read from the uart
 *
 * \param uartNumb Which uart to use     
 *
 * \return 0 no character availbale to be read
 *         1 character available to be read
 *
 */  
static int dwApbUart_tstc (int uartNum);

/* Functions ---------------------------------------------------------------- */

/*****************************************************************************
 *
 * serial_init()
 *
 * Purpose: Initialise the console uart
 *
 * Args:    
 *
 * Returns:
 *
 *****************************************************************************/
int serial_init (void)
{
    unsigned int lcrRegister;

    unsigned int baudRate = CONFIG_BAUDRATE;
    unsigned int uartNum = CONSOLE_PORT;
    unsigned int divisor;
    unsigned int temp;

    /* Setup the baud rate */
    
    /* Wait while the UART is busy... */
    while ((IO_READ(uart[uartNum] + UART_UART_STATUS_REG_OFFSET) & UART_UART_STATUS_BUSY_MASK));

    /* Set the Divisor Latch Access Bit in the Line Control Register */
    lcrRegister = IO_READ(uart[uartNum] + UART_LINE_CTRL_REG_OFFSET);
    lcrRegister = lcrRegister | UART_LINE_CTRL_DLAB_MASK;
    IO_WRITE (uart[uartNum] + UART_LINE_CTRL_REG_OFFSET, lcrRegister);
  
#ifdef CONFIG_PC302_SIMULATION        
    /* If running in RTL simulation land just set the baud rate to the highest value possible */
    IO_WRITE (uart[uartNum] + UART_DIVISOR_LOW_REG_OFFSET, 0x01);
    IO_WRITE (uart[uartNum] + UART_DIVISOR_HIGH_REG_OFFSET, 0x00);
#else
    /* Baud Rate = Baud Rate Gen Clock / (16 * divisor) */ 
    temp = 16 * baudRate;
    divisor = CONFIG_DW_APB_UART_CLOCK / temp;
    
    IO_WRITE (uart[uartNum] + UART_DIVISOR_LOW_REG_OFFSET, (divisor & UART_DIVISOR_MASK));
    IO_WRITE (uart[uartNum] + UART_DIVISOR_HIGH_REG_OFFSET, ((divisor >> 8) & UART_DIVISOR_MASK));
#endif        

    /* Clear the Divisor Latch Access Bit in the Line Control Register */
    lcrRegister = IO_READ(uart[uartNum] + UART_LINE_CTRL_REG_OFFSET);
    lcrRegister = lcrRegister  & ~(UART_LINE_CTRL_DLAB_MASK); 
    IO_WRITE (uart[uartNum] + UART_LINE_CTRL_REG_OFFSET, lcrRegister);

    /* Enable the Rx & Tx fifos */
    IO_WRITE (uart[uartNum] + UART_FIFO_CTRL_REG_OFFSET, UART_FIFO_CTRL_ENABLE);
	        
     /* Setup the UART for...
       8 data bit word length
       1 stop bit
       parity disabled */
                  
    /* Wait while the UART is busy... */
    while ((IO_READ(uart[uartNum] + UART_UART_STATUS_REG_OFFSET) & UART_UART_STATUS_BUSY_MASK)); 
   
    lcrRegister = IO_READ(uart[uartNum] + UART_LINE_CTRL_REG_OFFSET);
    lcrRegister = lcrRegister | UART_LINE_CTRL_DLS_8BITS;
    lcrRegister = lcrRegister & UART_LINE_CTRL_1STOP_BIT;
    lcrRegister = lcrRegister & UART_LINE_CTRL_PARITY_DISABLE;
    IO_WRITE (uart[uartNum] + UART_LINE_CTRL_REG_OFFSET, lcrRegister);
	
    return (0);
}

/*****************************************************************************
 *
 * serial_putc()
 *
 * Purpose: Write a character to the console uart 
 *
 * Args:    
 *
 * Returns:
 *
 *****************************************************************************/
void serial_putc(const char c)
{
    if (c == '\n')
    {
        dwApbUart_putc (CONSOLE_PORT, '\r');
    }

    dwApbUart_putc (CONSOLE_PORT, c);
}

/*****************************************************************************
 *
 * serial_puts()
 *
 * Purpose: Write a string to the console uart 
 *
 * Args:    
 *
 * Returns:
 *
 *****************************************************************************/

void serial_puts(const char *s)
{

    while (*s)
    {
        serial_putc(*s++);
    }
        
}

/*****************************************************************************
 *
 * serial_getc()
 *
 * Purpose: Read a character from the console uart 
 *
 * Args:    
 *
 * Returns:
 *
 *****************************************************************************/

int serial_getc(void)
{
	
    return dwApbUart_getc(CONSOLE_PORT);
    
}

/*****************************************************************************
 *
 * serial_tstc()
 *
 * Purpose: Test to see if there are any characters availale to be read
 *          from the console uart 
 *
 * Args:    
 *
 * Returns: 0 - no character availbale in the Rx FIFO
 *          1 - character available in the Rx FIFO
 *
 *****************************************************************************/
int serial_tstc(void)
{

    return dwApbUart_tstc(CONSOLE_PORT);

}

/*****************************************************************************
 *
 * serial_setbtg()
 *
 * Purpose: set the uart baud rate. 
 *
 * Args:    
 *
 * Returns: 
 *
 *****************************************************************************/
void serial_setbrg(void)
{
	/* Not used right now ! */
}

static void dwApbUart_putc (int uartNum, char c)
{
	
    /* Wait until there is space in the Tx FIFO... */
	
    while (!(IO_READ(uart[uartNum] + UART_LINE_STATUS_REG_OFFSET) & UART_LINE_STATUS_THRE_MASK));
    
    /* Send the character */
    IO_WRITE (uart[uartNum] + UART_TX_HOLDING_REG_OFFSET, c);
}
    
static int dwApbUart_getc (int uartNum)
{

    /* Wait until there is a character in the Rx FIFO... */
    while (!dwApbUart_tstc(CONSOLE_PORT));

    /* Go and read a character... */
    return (IO_READ (uart[uartNum] + UART_RX_BUFFER_REG_OFFSET));
    
}
   
static int dwApbUart_tstc (int uartNum)
{

    return (IO_READ (uart[uartNum] + UART_LINE_STATUS_REG_OFFSET) & UART_LINE_STATUS_DATA_READY_MASK);

}

#endif
