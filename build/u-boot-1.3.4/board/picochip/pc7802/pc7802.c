/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file pc7802.c
* \brief Various useful functions for use on a PC7802 Platform.
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

/* Includes ---------------------------------------------------------------- */
#include <common.h>
#include <asm/arch/pc20x.h>
#include <asm/arch/timer.h>
#include <asm/arch/gpio.h>

/* Constants --------------------------------------------------------------- */
DECLARE_GLOBAL_DATA_PTR;

/*!
 * \brief SDGPIO identifiers for PC202 devices.
 *
 * This enum defines a list of (SD)GPIO sources that are available for
 * use in PC202 devices.
 */
enum picoifGpioPinNum_PC202
{
    PC202_GPIO_PIN_INVAL = -1,  /*< Invalid pin configuration. */
    PC202_GPIO_PIN_SDGPIO_0,    /* SDGPIO pin identifiers. */
    PC202_GPIO_PIN_SDGPIO_1,
    PC202_GPIO_PIN_SDGPIO_2,
    PC202_GPIO_PIN_SDGPIO_3,
    PC202_GPIO_PIN_SDGPIO_4,
    PC202_GPIO_PIN_SDGPIO_5,
    PC202_GPIO_PIN_SDGPIO_6,
    PC202_GPIO_PIN_SDGPIO_7,
    PC202_GPIO_PIN_SDGPIO_8,
    PC202_GPIO_PIN_SDGPIO_9,
    PC202_GPIO_PIN_SDGPIO_10,
    PC202_GPIO_PIN_SDGPIO_11,
    PC202_GPIO_PIN_SDGPIO_12,
    PC202_GPIO_PIN_SDGPIO_13,
    PC202_GPIO_PIN_SDGPIO_14,
    PC202_GPIO_PIN_SDGPIO_15,
    PC202_GPIO_PIN_SDGPIO_16,
    PC202_GPIO_PIN_SDGPIO_17,
    PC202_GPIO_PIN_SDGPIO_18,
    PC202_GPIO_PIN_SDGPIO_19,
    PC202_GPIO_PIN_SDGPIO_20,
    PC202_GPIO_PIN_SDGPIO_21,
    PC202_GPIO_PIN_SDGPIO_22,
    PC202_GPIO_PIN_SDGPIO_23,
    PC202_GPIO_PIN_SDGPIO_24,
    PC202_GPIO_PIN_SDGPIO_25,
    PC202_GPIO_PIN_SDGPIO_26,
    PC202_GPIO_PIN_SDGPIO_27,
    PC202_GPIO_PIN_SDGPIO_28,
    PC202_GPIO_PIN_SDGPIO_29,
    PC202_GPIO_PIN_SDGPIO_30,
    PC202_GPIO_PIN_SDGPIO_31,
    PICO_NUM_GPIOS,
};

/* Macros ------------------------------------------------------------------ */
/* Configuration port write bit positions. */
#define CAEID_BIT_MASK      ( 1 << 19 )     /*!< Bit 19 - AE ID signal. */
#define CADDR_BIT_MASK      ( 1 << 18 )     /*!< Bit 18 - AE ADDR signal. */
#define CREAD_BIT_MASK      ( 1 << 17 )     /*!< Bit 17 - READ data signal. */
#define CWRITE_BIT_MASK     ( 1 << 16 )     /*!< Bit 16 - WRITE data signal. */

#define RB_FAIL_MASK        ( 1 << 17 )     /*!< Bit 17 - readback failed. */
#define RB_VALID_MASK       ( 1 << 16 )     /*!< Bit 16 - readback valid. */

#define RETRIES             ( 10 )          /*!< The number of retries for a \
                                             *   procif config read. */

#define PROCIF_CONFIG_READ_REG_OFFSET       ( 0x78 )
#define PROCIF_CONFIG_WRITE_REG_OFFSET      ( 0x7C )

/** PC202 AHB2Pico CAEID. */
#define PC202_AHB2PICO_CAEID                ( 0x0008 )

/** The address of the sleep register in the AHB2Pico. */
#define PC202_AHB2PICO_SLEEP_REG            ( 0xA060 )

/** The AHB2PICO wake up command */
#define PC202_AHB2PICO_WAKE_UP              ( 0 )

/** The base address of SD-GPIO config registers in the AHB2Pico. */
#define PC202_GPIO_SD_PIN_CONFIG_BASE       ( 0x9800 )

/** The address of the LSB SD-GPIOs input value register in the AHB2Pico. */
#define PC202_GPIO_SD_INPUT_VAL_REG_LOW     ( 0x9880 )

/** The address of the MSB SD_GPIO input value register in the AHB2Pico. */
#define PC202_GPIO_SD_INPUT_VAL_REG_HIGH    ( 0x9881 )

/** The address of the LSB SD-GPIOs control value register in the AHB2Pico. */
#define PC202_GPIO_SD_CONTROL_VAL_REG_LOW   ( 0x9882 )

/** The address of the MSB SD_GPIO control value register in the AHB2Pico. */
#define PC202_GPIO_SD_CONTROL_VAL_REG_HIGH  ( 0x9883 )

/** The address of the LSB SD-GPIOs output value register in the AHB2Pico. */
#define PC202_GPIO_SD_OUTPUT_VAL_REG_LOW    ( 0x9884 )

/** The address of the MSB SD_GPIO output value register in the AHB2Pico. */
#define PC202_GPIO_SD_OUTPUT_VAL_REG_HIGH   ( 0x9885 )

/** The spacing between SD-GPIO config registers. */
#define PC202_GPIO_SD_PIN_CONFIG_SPACING    ( 4 )

/**
 * Macro to get the address of a config register for a SD-GPIO pin.
 *
 * \param _n The SD-GPIO pin number.
 * \return Returns the base address of the register.
 */
#define PC202_GPIO_SD_PIN_CONFIG( _n ) \
    PC202_GPIO_SD_PIN_CONFIG_BASE + ( (_n) * PC202_GPIO_SD_PIN_CONFIG_SPACING )

/** Control source bit. */
#define PC202_GPIO_SD_CONFIG_CS_MASK ~( 1 << 15 )

/** Define the number of SD-GPIOs allocated to lower register*/
#define PC202_NUM_LSBS ( 16 + PC202_GPIO_PIN_SDGPIO_0 )

/** PC202 PROCIF CAEID */
#define PA_AEID_PROCINT2                ( 0x0048 )

/** The address of the device id register in the PROCIF */
#define PA_PROCINT2_DEVICEID            ( 0x0030 )

/* picoArray device Ids */
#define PC202_DEVICE_ID                 ( 0x0010 )
#define PC205_DEVICE_ID                 ( 0x0012 )

/* FPGA Configuration Done signal */
#define FPGA_CONFIG_DONE                (PC202_GPIO_PIN_SDGPIO_27)

/* Timeout (Sec) to wait for the fpga to configure */
#define FGPA_CONFIG_DONE_TIMEOUT    (5)

/* FPGA Board Id register */
#define FPGA_DID_REG                (0)

/* FPGA Code version register */
#define FPGA_VERSION_REG            (1)

#define FPGA_MAJOR_VERSION_MASK     (0xFF00)
#define FPGA_MAJOR_VERSION_SHIFT    (8)

#define FPGA_MINOR_VERSION_MASK     (0x00FF)
#define FPGA_MINOR_VERSION_SHIFT    (0)

/* Prototypes--------------------------------------------------------------- */
/*!
 *
 * Write a 32 bit number to some address
 *
 * \param val, 32 bit value to write
 * \param addr, address to write to
 *
 */
static void picoif_out32(u32 val,
                         void * addr);

/*!
 *
 * Read a 32 bit value from some address
 *
 * \param addr, address to read from
 * \return the 32 bit value read
 *
 */
static u32 picoif_in32(void * addr);

/*!
 *
 * Perform a picoArray config bus read
 *
 * \param aeid, aeid to use
 * \param ae_addr, ae address to use
 * \param buf, buffer to store the read data
 * \param count, the number of reads to perform
 * \return, the number of words read of negative on error
 *
 */
static int
procif_config_read(u16 aeid,
                   u16 ae_addr,
                   u16 *buf,
                   u16 count);

/*!
 *
 * Perform a picoArray config bus write
 *
 * \param aeid, aeid to use
 * \param ae_addr, ae address to use
 * \param buf, buffer contaning the data to write
 * \param count, the number of writes to perform
 * \return, the number of words written
 *
 */
static int
procif_config_write(u16 aeid,
                    u16 ae_addr,
                    u16 *buf,
                    u16 count);

/*!
 *
 * Start timer #0 in free running mode
 *
 */
static void pc20x_timer_0_start(void);


/*!
 *
 *  Obtaine the state of the fpga config done signal.
 *
 * \return 1 for config done,
 *         0 config not complete,
 *         negative on error
 */

static int
fpga_config_done (void);

/*!
 *
 *  Read FPGA registers via bit bashed SPI
 *
 * \param reg The FPGA register number to read
 * \return Data read from FPGA
 */
static unsigned short fpga_read(unsigned char reg);

/*!
 *
 *  Display the board type
 *
 * \param id_reg The id register contents read from the FPGA.
 */
static void display_board_type (unsigned short id_reg);

/*!
 *
 *  Display some FPGA version information
 *
 * \param vers_reg The version register contents read from the FPGA.
 */
static void display_fpga_info (unsigned short vers_reg);

static int initialise_sdgpio (void);

/*!
 * Setup the direction for an sg gpio pin
 *
 * \param pin_num The pin to set the direction of
 * \param input Set to 1 for an input, 0 for an output
 * \return Returns 0 on seccess non zero on error
 */
static int
pc202gpio_sd_set_direction(int pin_num,
                           int input);

/*!
 * Get the value of an input SDGPIO pin.
 *
 * \param pin The pin to get the value of.
 * \return Returns the value read on success, negative on failure.
 */
static int
pc202gpio_sd_get_value(int pin_num);

/* Functions --------------------------------------------------------------- */

/*****************************************************************************
 *
 * show_boot_progress()
 *
 * Purpose: Indicate booting progress
 *
 * Note: see U-Boot README for a list of 'progress' values.
 *
 *****************************************************************************/
#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
	printf("Boot reached stage %d\n", progress);
}
#endif

/*****************************************************************************
 *
 * board_init()
 *
 * Purpose: Hardware platform initialisation functions
 *
 * Returns: 0 - Success
 *
 *****************************************************************************/
int board_init (void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = 0x00000100;
        gd->bd->bi_arch_number = MACH_TYPE_PC7802;
	gd->flags = 0;

	icache_enable ();

        pc20x_timer_0_start();

	return 0;
}

/*****************************************************************************
 *
 * checkboard()
 *
 * Purpose: Display some useful hardware platform information.
 *
 * Returns: 0 - Success
 *
 *****************************************************************************/
int checkboard (void)
{
    u16 device_type;

    int ret;

    unsigned long timebase;

    printf("Build: picoChip "PICOCHIP_PLATFORM" \n");

    /* What picoChip device are we running on ? */
    printf("Device: ");

    ret = procif_config_read (PA_AEID_PROCINT2,
                             PA_PROCINT2_DEVICEID,
                             &device_type,
                             1);
    if (ret < 0)
    {
        /* Oops we didn't have a successful read */
        printf("Failed to read the device id !\n");
    }
    else
    {
        /* We have a successful read... */
        switch (device_type)
        {
            case PC202_DEVICE_ID:
            {
                printf("PC202\n");
                break;
            }
            case PC205_DEVICE_ID:
            {
                printf("PC205\n");
                break;
            }
            default:
            {
                printf("Unknown !\n");
            }
        }
    }

    /* Obtain some information from the FPGA */

    /* Before we can read from the fpge we need to make sure it
       has been configured */
    ret = initialise_sdgpio();
    if (ret == 0)
    {
        /* Now we wait for the fpga to configure */
        timebase = get_timer(0);
        do
        {
            ret = fpga_config_done ();

            if (ret < 0)
            {
                break;
            }

            if (ret)
            {
                break;
            }
        }
        while (get_timer(timebase) < FGPA_CONFIG_DONE_TIMEOUT);

        if (ret < 0)
        {
            printf("Can't read from the SDGPIO !\n");
        }
        else if (ret)
        {
            /* The fpga has configured */
            unsigned short fpga_id_reg = fpga_read(FPGA_DID_REG);
            unsigned short fpga_ver_reg = fpga_read(FPGA_VERSION_REG);

            /* Print out the board info */
            if (fpga_id_reg != 0xffff)
            {
                /* We have an FPGA present */
                display_board_type (fpga_id_reg);
            }
            else
            {
                printf("Failed to read the FPGA !\n");
            }

            /* Print out the FPGA info */
            if (fpga_ver_reg != 0xffff)
            {
                /* We have an FPGA present */
                display_fpga_info (fpga_ver_reg);
            }
        }
        else
        {
            printf("FPGA not configured !\n");
        }
    }
    else
    {
        printf("SDGPIO not configured !\n");
    }

    return 0;
}

/*****************************************************************************
 *
 * misc_init_r()
 *
 * Purpose: Miscellaneous platform dependent initialisations
 *
 * Returns: 0 - Success
 *
 *****************************************************************************/
int misc_init_r (void)
{
    /* Not used right now, function template left here as a place holder */
    return 0;
}

/*****************************************************************************
 *
 * dram_init()
 *
 * Purpose: Initialize the DDR SDRAM info in the board data structure
 *
 * Returns: 0 - Success
 *
 *****************************************************************************/
int dram_init (void)
{

#ifndef CONFIG_PC20X_2_DDR_RAM_BANKS
/* We want a 4 DDR Bank setup then */

    gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
    gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
    gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
    gd->bd->bi_dram[1].size = PHYS_SDRAM_2_SIZE;
    gd->bd->bi_dram[2].start = PHYS_SDRAM_3;
    gd->bd->bi_dram[2].size = PHYS_SDRAM_3_SIZE;
    gd->bd->bi_dram[3].start = PHYS_SDRAM_4;
    gd->bd->bi_dram[3].size = PHYS_SDRAM_4_SIZE;

#else   /* We want a 2 DDR Bank setup then */

    gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
    gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
    gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
    gd->bd->bi_dram[1].size = PHYS_SDRAM_2_SIZE;

#endif

    return 0;
}

static void picoif_out32(u32 val,
                         void * addr)
{
    *(unsigned int *)(addr) = val;
}

static u32 picoif_in32(void * addr)
{
    return *(unsigned int *)(addr);
}

static int
procif_config_read(u16 aeid,
                   u16 ae_addr,
                   u16 *buf,
                   u16 count)
{
    u32 val;
    void *write_p = (void *)
                    (PC20X_SLAVE_PROCIF_BASE + PROCIF_CONFIG_WRITE_REG_OFFSET);
    void *read_p = (void *)
                   (PC20X_SLAVE_PROCIF_BASE + PROCIF_CONFIG_READ_REG_OFFSET);
    u16 to_read = count;
    u16 rc;
    unsigned i;
    unsigned retries;

    val = aeid | CAEID_BIT_MASK;
    picoif_out32( val, write_p );

    while ( to_read )
    {
        /* Output the address to read from. */
        val = ( ae_addr + ( count - to_read ) ) | CADDR_BIT_MASK;
        picoif_out32( val, write_p );

        /* Dispatch the read requests. */
        rc = ( to_read > 64 ) ? 64 : to_read;
        val = CREAD_BIT_MASK;
        for ( i = 0; i < rc; ++i )
        {
            picoif_out32( val, write_p );
        }

        /* Now read the values. */
        for ( i = 0; i < rc; ++i )
        {
            retries = RETRIES;
            while ( retries )
            {
                val = picoif_in32( read_p );
                if ( val & ( RB_VALID_MASK | RB_FAIL_MASK ) )
                    break;
                --retries;
            }

            if ( !retries || ( val & RB_FAIL_MASK ) )
            {
                printf ( "config read %04x@%04x failed\n", aeid,
                        ( ae_addr + ( count - to_read ) + i ) );
                if ( !retries )
                    printf ( "timed out\n" );

                break;
            }
            else
                buf[ ( count - to_read ) + i ] = val & 0xFFFF;
        }

        if ( val & RB_FAIL_MASK )
            break;

        to_read -= rc;
    }

    return !( val & RB_FAIL_MASK ) ? count : -1;
}

static int
procif_config_write(u16 aeid,
                    u16 ae_addr,
                    u16 *buf,
                    u16 count)
{
    u32 val;
    void *write_p = (void *)
                    (PC20X_SLAVE_PROCIF_BASE + PROCIF_CONFIG_WRITE_REG_OFFSET);
    unsigned i;

    val = aeid | CAEID_BIT_MASK;
    picoif_out32( val, write_p );

    /* Output the address to read from. */
    val = ae_addr | CADDR_BIT_MASK;
    picoif_out32( val, write_p );

    /* Now read the values. */
    for ( i = 0; i < count; ++i )
    {
        val = buf[ i ] | CWRITE_BIT_MASK;
        picoif_out32( val, write_p );
    }

    return i;
}

static void pc20x_timer_0_start(void)
{
    /* Make sure timer #0 is disabled */
    *(volatile unsigned int *)(CFG_TIMERBASE + TimerNControlRegOffset(0)) = 0;

    /* Initialise the timer #0 to all 1's.  We do this  because we want to run
       the timer in free running mode. */
    *(volatile unsigned int *)(CFG_TIMERBASE + TimerNLoadCountRegOffset(0)) =
                               0xFFFFFFFF;

    /* Start timer #0 in free running mode */
    *(volatile unsigned int *)(CFG_TIMERBASE + TimerNControlRegOffset(0)) =
                              (TimerInterruptMask | TimerEnable);
}

static int
fpga_config_done (void)
{
    return ( pc202gpio_sd_get_value( FPGA_CONFIG_DONE ) );
}

static unsigned short fpga_read(unsigned char reg)
{
#define SPI_SH_DELAY 1
#define TXRX_LEN 2
#define CS0     (1 << 3)
#define CLK     (1 << 2)
#define MISO    (1 << 1)
#define MOSI    (1 << 0)
    unsigned int odr, idr, ddr;
    unsigned char bits;
    unsigned short txrx_buf[TXRX_LEN];
    int i;
    unsigned int chip_select = CS0;

    odr = *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) & 0xf0;
    ddr = *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortADataDirectionRegOffset) & 0xf0;

    /* Setup GPIO directions and initial state */

    /* CS0, MOSI and MISO low. Set CLK high */
    odr |= CLK;
    *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;

    /* CS0, CLK and MOSI outputs, MISO input */
    ddr |= (CS0 | CLK | MOSI);
    *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortADataDirectionRegOffset) = ddr;

    /* Set chip select */
    odr |= chip_select;
    *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;

    /* Setup the txrx buffer */
    txrx_buf[0] = (reg << 8) | (1 << 7);  /* The address to access and read command bit */
    txrx_buf[1] = 0;            /* The data read will appear here */

    /* transfer each word */
    for (i = 0; i < TXRX_LEN; i++)
    {
        bits = 16;
        /* if (cpol == 0) this is SPI_MODE_0; else this is SPI_MODE_2 */

        /* clock starts at inactive polarity */
        for (txrx_buf[i] <<= (16 - bits); bits; bits--)
        {
            /* setup MSB (to slave) on trailing edge */
            if (txrx_buf[i] & (1 << 15))
            {
                odr |= MOSI;
            }
            else
            {
                odr &= ~MOSI;
            }
            *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;

            udelay(SPI_SH_DELAY);	/* T(setup) */

            /* Set CLK low */
            odr &= ~CLK;
            *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;

            udelay(SPI_SH_DELAY);

            /* sample MSB (from slave) on leading edge */
            txrx_buf[i] <<= 1;
            idr = *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAInputDataRegOffset);
            if (idr & MISO)
            {
                txrx_buf[i] |= 0x01;
            }

            /* Set CLK high */
            odr |= CLK;
            *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;
        }
    }

    /* Reset chip select and provide one clock cycle to reset the transfer */
    udelay(SPI_SH_DELAY);
    odr &= ~(chip_select);
    *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;

    odr &= ~CLK;
    *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;

    udelay(SPI_SH_DELAY);
    odr |= CLK;
    *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;

    /* The result is left in the second word */
    return txrx_buf[1];
}

static void display_board_type (unsigned short id_reg)
{
    puts("Board: ");
    printf("%04x\n", id_reg);
}

static void display_fpga_info (unsigned short vers_reg)
{
    puts("FPGA:  ");
    printf("v%x.%x\n",
           (vers_reg & FPGA_MAJOR_VERSION_MASK) >> FPGA_MAJOR_VERSION_SHIFT,
           (vers_reg & FPGA_MINOR_VERSION_MASK) >> FPGA_MINOR_VERSION_SHIFT);
}

static int initialise_sdgpio (void)
{
    int ret;

    u16 data = PC202_AHB2PICO_WAKE_UP;

    /* We are initialising the sdgpio, therefore before we do anything
     * else we need to wake up the AHB2PICO block. */
    ret = procif_config_write( PC202_AHB2PICO_CAEID,
                               PC202_AHB2PICO_SLEEP_REG, &data, 1 );
    if ( 1 != ret )
    {
        printf("failed to wake up the AHB2PICO block.\n");
        return -1;
    }

    /* Setup FPGA_CONFIG_DONE as an input */
    ret = pc202gpio_sd_set_direction( FPGA_CONFIG_DONE, 1 );
    if ( 0 != ret )
    {
        printf("failed to set the direction for sd gpio pin %d.\n",
               FPGA_CONFIG_DONE );
        return -1;
    }
    return 0;
}

static int
pc202gpio_sd_set_direction(int pin_num,
                           int input)
{
    int ret=0;
    u16 data=0;
    u16 reg=0;

    ret = procif_config_read( PC202_AHB2PICO_CAEID,
                              PC202_GPIO_SD_PIN_CONFIG
                              ( pin_num - PC202_GPIO_PIN_SDGPIO_0 ),
                              &data, 1 );
    if ( 1 != ret )
    {
        printf( "failed to read config register for SDGPIO \
                pin %u\n", pin_num );
        return -1;
    }

    data &= PC202_GPIO_SD_CONFIG_CS_MASK;
    ret = procif_config_write( PC202_AHB2PICO_CAEID,
                               PC202_GPIO_SD_PIN_CONFIG
                               ( pin_num - PC202_GPIO_PIN_SDGPIO_0 ),
                               &data, 1 );
    if ( 1 != ret )
    {
        printf( "failed to write config register for SDGPIO \
                pin %u\n", pin_num );
        return -1;
    }

    /* Configure the pin to drive or not drive the output as appropriate. */
    if ( pin_num < PC202_NUM_LSBS)
        reg = PC202_GPIO_SD_CONTROL_VAL_REG_LOW;
    else
        reg = PC202_GPIO_SD_CONTROL_VAL_REG_HIGH;

    ret = procif_config_read( PC202_AHB2PICO_CAEID, reg, &data, 1 );
    if ( 1 != ret )
    {
        printf( "failed to read SDGPIO control value register\n" );
        return -1;
    }

    if ( input )
        data &= ~( 1 << ( pin_num - PC202_NUM_LSBS ) );
    else
        data |= ( 1 << ( pin_num - PC202_NUM_LSBS ) );

    ret = procif_config_write( PC202_AHB2PICO_CAEID, reg, &data, 1 );
    if ( 1 != ret )
    {
        printf( "failed to write the control value register "
                "for SDGPIO pin %u\n", pin_num );
        return -1;
    }

    /* The output value upper register needs to be written in order for the configuration
    change to take effect. This includes any reads of these registers. */

    ret = procif_config_read( PC202_AHB2PICO_CAEID,
                              PC202_GPIO_SD_OUTPUT_VAL_REG_HIGH,
                              &data, 1);
    if ( 1 != ret )
    {
        printf( "failed to read the high value register "
                "for SDGPIO pin %u\n", pin_num );
        return -1;
    }

    ret = procif_config_write( PC202_AHB2PICO_CAEID,
                               PC202_GPIO_SD_OUTPUT_VAL_REG_HIGH,
                               &data, 1);
    if ( 1 != ret )
    {
        printf( "failed to write the high value register "
                "for SDGPIO pin %u\n", pin_num );
        return -1;
    }

    return 0;
}

static int
pc202gpio_sd_get_value(int pin_num)
{
    int ret=0;

    /* Digital mode */
    u16 data[2]={0,0};

    ret = procif_config_read( PC202_AHB2PICO_CAEID,
                              PC202_GPIO_SD_INPUT_VAL_REG_LOW, data, 2 );
    if ( 2 != ret )
    {
        printf( "failed to read SDGPIO input value regs\n" );
        return -1;
    }

    if ( pin_num < PC202_NUM_LSBS )
    {
        return !!( data[0] & ( 1 << ( pin_num - PC202_NUM_LSBS ) ) );
    }
    else
    {
        return !!( data[1] & ( 1 << ( pin_num - PC202_NUM_LSBS ) ) );
    }
}
