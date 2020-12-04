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

/* Includes ---------------------------------------------------------------- */
#include <common.h>

#ifdef CFG_DW_APB_UART

#include <asm/arch/pc20x.h>
#include <asm/arch/uart.h>

#if !defined(CONFIG_CONS_INDEX)
#error	"No console index specified."
#elif (CONFIG_CONS_INDEX < 1) || (CONFIG_CONS_INDEX > 2)
#error	"Invalid console index value specified."
#endif

/* Macros ------------------------------------------------------------------ */
#define IO_WRITE(addr, val) (*(volatile unsigned int *)(addr) = (val))
#define IO_READ(addr) (*(volatile unsigned int *)(addr))

/* uart[] array is zero based whilst CONFIG_CONS_INDEX is '1' based */
#define CONSOLE_PORT (CONFIG_CONS_INDEX - 1)

/* Constants --------------------------------------------------------------- */
static volatile unsigned char *const uart[] = {
                                                (void *)PC20X_UART1_BASE,
                                                (void *)PC20X_UART2_BASE
                                              };

/* Prototypes--------------------------------------------------------------- */
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


/* Functions --------------------------------------------------------------- */

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
 ****************************************************************************/
int serial_init (void)
{
    unsigned int lcrRegister;

    unsigned int baudRate = CONFIG_BAUDRATE;
    unsigned int uartNum = CONSOLE_PORT;
    unsigned int divisor;
    unsigned int temp;
    

#ifdef CONFIG_PC20X_SIMULATION
    /* If running in RTL simulation land, perform some test bench 'magic' */
	
    /* Program the 'TrickBox' to start up the uart baud rate clock */
    *(volatile unsigned int *)(0xe0000058) = 0x1;

    /* Program the 'TrickBox' to cross wire the two on chip uarts
       Uart1 tx -> Uart2 rx
       Uart1 rx <- Uart2 tx */
    *(volatile unsigned int *)(0xe0000050) = 0x4;
#endif

    /* Setup the baud rate */
    
    /* Wait while the UART is busy... */
    while ((IO_READ(uart[uartNum] + UartUartStatusRegOffset) & UartUartStatusBusyMask));

    /* Set the Divisor Latch Access Bit in the Line Control Register */
    lcrRegister = IO_READ(uart[uartNum] + UartLineCtrlRegOffset);
    lcrRegister = lcrRegister | UartLineCtrlDLABMask;
    IO_WRITE (uart[uartNum] + UartLineCtrlRegOffset, lcrRegister);
  
#ifdef CONFIG_PC20X_SIMULATION        
    /* If running in RTL simulation land just set the baud rate to the highest value possible */

    IO_WRITE (uart[uartNum] + UartDivisorLowRegOffset, 0x01);
    IO_WRITE (uart[uartNum] + UartDivisorHighRegOffset, 0x00);
    
#else
    /* Baud Rate = Baud Rate Gen Clock / (16 * divisor) */
         
    temp = 16 * baudRate;
    divisor = CONFIG_DW_APB_UART_CLOCK / temp;
    
    IO_WRITE (uart[uartNum] + UartDivisorLowRegOffset, (divisor & UartDivisorMask));
    IO_WRITE (uart[uartNum] + UartDivisorHighRegOffset, ((divisor >> 8) & UartDivisorMask));

#endif        

    /* Clear the Divisor Latch Access Bit in the Line Control Register */
    lcrRegister = IO_READ(uart[uartNum] + UartLineCtrlRegOffset);
    lcrRegister = lcrRegister  & ~(UartLineCtrlDLABMask); 
    IO_WRITE (uart[uartNum] + UartLineCtrlRegOffset, lcrRegister);

    /* Enable the Rx & Tx fifos */
    IO_WRITE (uart[uartNum] + UartFIFOCtrlRegOffset, UartFIFOCtrlEnable);
	        
     /* Setup the UART for...
       8 data bit word length
       1 stop bit
       parity disabled */
                  
    /* Wait while the UART is busy... */
    while ((IO_READ(uart[uartNum] + UartUartStatusRegOffset) & UartUartStatusBusyMask)); 
   
    lcrRegister = IO_READ(uart[uartNum] + UartLineCtrlRegOffset);
    lcrRegister = lcrRegister | UartLineCtrlDLS8bits;
    lcrRegister = lcrRegister & UartLineCtrl1StopBit;
    lcrRegister = lcrRegister & UartLineCtrlParityDisable;
    IO_WRITE (uart[uartNum] + UartLineCtrlRegOffset, lcrRegister);
	
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
 ****************************************************************************/
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
 ****************************************************************************/

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
 ****************************************************************************/

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
 ****************************************************************************/
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
 ****************************************************************************/
void serial_setbrg(void)
{
	/* Not used right now ! */
}

static void dwApbUart_putc (int uartNum, char c)
{
	
    /* Wait until there is space in the Tx FIFO... */
	
    while (!(IO_READ(uart[uartNum] + UartLineStatusRegOffset) & UartLineStatusTHREMask));
    
    /* Send the character */
    IO_WRITE (uart[uartNum] + UartTXHoldingRegOffset, c);
}
      
static int dwApbUart_getc (int uartNum)
{

    /* Wait until there is a character in the Rx FIFO... */
    while (!dwApbUart_tstc(CONSOLE_PORT));

    /* Go and read a character... */
    return (IO_READ (uart[uartNum] + UartRXBufferRegOffset));
    
}

static int dwApbUart_tstc (int uartNum)
{

    return (IO_READ (uart[uartNum] + UartLineStatusRegOffset) & UartLineStatusDataReadyMask);

}


/* Used to turn U-Boot booting into a self-checking testcase for the Firecracker simulation environment */

void uart2_serial_init(void)
{
	int lcrRegister;

	/* Setup uart2 in an identical fashion to uart 1 */
         
       	/* Set up the baud rate divisor registers */
 
	/*   Wait while UART is busy... */
        while ((*(volatile unsigned int *)(PC20X_UART2_BASE + UartUartStatusRegOffset) & UartUartStatusBusyMask));

	/*   Set DLAB bit in LCR */
        lcrRegister = *(volatile unsigned int *)(PC20X_UART2_BASE + UartLineCtrlRegOffset);
	lcrRegister = lcrRegister | UartLineCtrlDLABMask; 
        *(volatile unsigned int *)(PC20X_UART2_BASE + UartLineCtrlRegOffset) = lcrRegister;
  
	/*   Write to the DLL register */
        *(volatile unsigned int *)(PC20X_UART2_BASE + UartDivisorLowRegOffset) = 0x01;

        /*   Clear DLAB bit in LCR */
        lcrRegister = *(volatile unsigned int *)(PC20X_UART2_BASE + UartLineCtrlRegOffset);
	lcrRegister = lcrRegister  & ~(UartLineCtrlDLABMask); 
        *(volatile unsigned int *)(PC20X_UART2_BASE + UartLineCtrlRegOffset) = lcrRegister;

	/* Setup the FIFO Control Register  - Enable the Rx / Tx fifos */
        *(volatile unsigned int *)(PC20X_UART2_BASE + UartFIFOCtrlRegOffset) = 0x01;

	/* Setup the Line Control Register */
	
	/*   Wait while UART is busy... */
        while ((*(volatile unsigned int *)(PC20X_UART2_BASE + UartUartStatusRegOffset) & UartUartStatusBusyMask));
        
	/*   Setup for 8 data bit word length */
	*(volatile unsigned int *)(PC20X_UART2_BASE + UartLineCtrlRegOffset) |= UartLineCtrlDLS8bits;
}

void uart2_send_command (void)
{

	while ( ! (*(volatile unsigned int *)(PC20X_UART2_BASE + UartLineStatusRegOffset) & UartLineStatusTHREMask));

	/* Write a '?' character to the Tx FIFO... */
        *(char *)(PC20X_UART2_BASE + UartTXHoldingRegOffset) = '?';

	while ( ! (*(volatile unsigned int *)(PC20X_UART2_BASE + UartLineStatusRegOffset) & UartLineStatusTHREMask));

	/* Write a '\n' character to the Tx FIFO... */
        *(char *)(PC20X_UART2_BASE + UartTXHoldingRegOffset) = '\n';
}

#endif
