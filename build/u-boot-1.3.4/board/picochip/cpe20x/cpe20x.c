/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file cpe20x.c
* \brief Various useful functions for use on a CPE20X Platform.
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

/* Macros ------------------------------------------------------------------ */
#define PA_TIMECOUNT 32
#define PROCIFCONFIGREADREGOFFSET   0x78
#define PROCIFCONFIGWRITEREGOFFSET  0x7C
#define PA_CONFIG_WRITE             0x00010000
#define PA_CONFIG_READ              0x00020000
#define PA_CONFIG_ADDR              0x00040000
#define PA_CONFIG_AEID              0x00080000
#define PA_CONFIG_VALID             0x00010000
#define PA_CONFIG_FAIL              0x00020000
#define PA_CONFIG_TIMEOUTOCCURED    0x00040000
#define PA_AEID_PROCINT2            0x0048
#define PA_PROCINT2_DEVICEID        0x0030

#define PA_DEVICE_ID_MASK           0xFFFF
#define PC202_DEVICE_ID             0x00000010
#define PC203_DEVICE_ID             0x00000011
#define PC205_DEVICE_ID             0x00000012

/* FPGA Device ID register */
#define FPGA_DID_REG                (0)

/* The device ID reg changed between v1 and v2 */
#define V1_FPGA_DID_BOARD_TYPE_SHIFT   (4)
#define V1_FPGA_DID_BOARD_TYPE_MASK    (0xff << V1_FPGA_DID_BOARD_TYPE_SHIFT)

#define V2_FPGA_DID_BOARD_TYPE_SHIFT   (8)
#define V2_FPGA_DID_BOARD_TYPE_MASK    (0xff << V2_FPGA_DID_BOARD_TYPE_SHIFT)
#define V2_FPGA_DID_PCB_BUILD_SHIFT    (4)
#define V2_FPGA_DID_PCB_BUILD_MASK     (0x0f << V2_FPGA_DID_PCB_BUILD_SHIFT)

#define FPGA_DID_DEVICE_ID_SHIFT    (0)
#define FPGA_DID_DEVICE_ID_MASK     (0x0f << FPGA_DID_DEVICE_ID_SHIFT)

/* Board types: */
#define BOARD_TYPE_PC7205           (6)
#define BOARD_TYPE_HDP203           (7)

/* PCB Builds: TODO validate */
#define PCB_BUILD_PC72052_I10_REVB  (0)

/* Device IDs: */
#define DID_RC_FPGA                 (1)
#define DID_AD_FPGA                 (2)

/* FPGA version register */
#define FPGA_VERSION_REG            (1)

#define FPGA_VERS_CODE_REL_SHIFT    (8)
#define FPGA_VERS_CODE_REL_MASK     (0xff << FPGA_VERS_CODE_REL_SHIFT)
#define FPGA_VERS_CODE_REV_SHIFT    (0)
#define FPGA_VERS_CODE_REV_MASK     (0xff << FPGA_VERS_CODE_REV_SHIFT)

/* Prototypes--------------------------------------------------------------- */
/*!
 *
 * Start timer #0 in free running mode
 *
 */
static void pc20xTimer0Start(void);

/*!
 *
 *  Read proc-if configuration space
 *
 * \param procAEID AEID of block in picoArray being accessed
 * \param procConfigAddress Address in the picoArray being accessed
 * \return Data read from AEID & Addr + state of read
 *         bit 16 = valid
 *         bit 17 = fail from picoArray
 *         bit 18 = timeout occured
 */
static unsigned int procConfigRead (unsigned int procAEID, unsigned int procConfigAddress);

/*!
 *
 *  Read FPGA registers via bit bashed SPI
 *
 * \param fpga The FPGA we want to read from
 * \param reg The FPGA register number to read
 * \return Data read from FPGA
 */
static unsigned short FPGARead(unsigned int fpga, unsigned char reg);

/*!
 *
 *  Display the board type
 *
 * \param vers_reg The version register contents read from the FPGA(s).
 * \param id_reg The id register contents read from the FPGA(s)
 */
static void display_board_type (unsigned short vers_reg,
                                unsigned short id_reg);

/*!
 *
 *  Display some FPGA version information
 *
 * \param vers_reg The version register contents read from the FPGA(s).
 * \param id_reg The id register contents read from the FPGA(s)
 */
static void display_fpga_info (unsigned short vers_reg,
                               unsigned short id_reg);


#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
	printf("Boot reached stage %d\n", progress);
}
#endif

/*
 * Miscellaneous platform dependent initialisations
 */

int board_init (void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = 0x00000100;
        gd->bd->bi_arch_number = MACH_TYPE_PC72052_I10_REVB;
	gd->flags = 0;

	icache_enable ();

        pc20xTimer0Start();

	return 0;
}

int checkboard (void)
{
    unsigned int deviceType;

    puts("Build: picoChip "PICOCHIP_PLATFORM" \n");

    /* What device are we running on ? */
    puts("Device: ");

    deviceType = procConfigRead (PA_AEID_PROCINT2, PA_PROCINT2_DEVICEID);
    if ((deviceType & PA_CONFIG_VALID) == PA_CONFIG_VALID)
    {
        /* We have a successful read... */
        deviceType = deviceType & PA_DEVICE_ID_MASK;
        switch (deviceType)
        {
            case PC202_DEVICE_ID:
            {
                puts("PC202\n");
                break;
            }
            case PC203_DEVICE_ID:
            {
                puts("PC203\n");
                break;
            }
            case PC205_DEVICE_ID:
            {
                puts("PC205\n");
                break;
            }
            default:
            {
                puts("Unknown !\n");
            }
        }
    }
    else
    {
        /* Didn't have a successful read */
        puts("Failed to read the device id !\n");
    }

    /* Obtain some information from the FPGA(s) */
    {
        unsigned short rc_id_reg = FPGARead(DID_RC_FPGA, FPGA_DID_REG);
        unsigned short rc_vers_reg = FPGARead(DID_RC_FPGA, FPGA_VERSION_REG);

        unsigned short ad_id_reg = FPGARead(DID_AD_FPGA, FPGA_DID_REG);
        unsigned short ad_vers_reg = FPGARead(DID_AD_FPGA, FPGA_VERSION_REG);

        /* Print out the board info */
        if (rc_id_reg != 0xffff)
        {
            /* We have an RC FPGA */
            display_board_type (rc_vers_reg, rc_id_reg);
        }
        else if (ad_id_reg != 0xffff)
        {
            /* We have an AD FPGA */
            display_board_type (ad_vers_reg, ad_id_reg);
        }
        else
        {
            puts("Failed to read the FPGA(s) !\n");
        }

        /* Print out the FPGA info */
        if (rc_id_reg != 0xffff)
        {
            /* We have an RC FPGA */
            display_fpga_info (rc_vers_reg, rc_id_reg);
        }

        if (ad_id_reg != 0xffff)
        {
            /* We have an AD FPGA */
            display_fpga_info (ad_vers_reg, ad_id_reg);
        }
    }

    return 0;
}

/*****************************************************************************
 *
 * misc_init_r()
 *
 * Purpose: Not used right now, function template left here as a place holder
 *
 ****************************************************************************/
int misc_init_r (void)
{
    /* Not used right now, function template left here as a place holder */
    return 0;
}

/*****************************************************************************
 *
 * dram_init()
 *
 * Purpose: Initialize the DDR-RAM info in the board data structure
 *
 ****************************************************************************/
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

static void pc20xTimer0Start(void)
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

static unsigned int procConfigRead ( unsigned int procAEID, unsigned int procConfigAddress)
{
    unsigned int failed = 0;
    unsigned int valid  = 0;
    unsigned int timeout = PA_TIMECOUNT;
    unsigned int procReadData;

    /* Write the AEID */
    *(volatile unsigned int *)(PC20X_SLAVE_PROCIF_BASE + PROCIFCONFIGWRITEREGOFFSET) = (PA_CONFIG_AEID | procAEID);

    /* Write the address being accessed */
    *(volatile unsigned int *)(PC20X_SLAVE_PROCIF_BASE + PROCIFCONFIGWRITEREGOFFSET) = (PA_CONFIG_ADDR | procConfigAddress);

    /* Write request for a read access */
    *(volatile unsigned int *)(PC20X_SLAVE_PROCIF_BASE + PROCIFCONFIGWRITEREGOFFSET) = PA_CONFIG_READ;

    /* Perform reads */

    while( timeout != 0 && failed != 1 && valid != 1)
    {
        /* Read the procif */
        procReadData =  *(volatile unsigned int *)(PC20X_SLAVE_PROCIF_BASE + PROCIFCONFIGREADREGOFFSET);

        /* if valid bit set then set flag */
        if(procReadData & PA_CONFIG_VALID)
        {
            valid = 1;
        }

        /* if fail bit set then set flag */
        if(procReadData & PA_CONFIG_FAIL)
        {
            failed = 1;
        }

        /* increment timeout count */
        timeout--;
    }

    if(timeout == 0)
    {
        procReadData = procReadData | PA_CONFIG_TIMEOUTOCCURED;
    }

    return procReadData;
}

static unsigned short FPGARead(unsigned int fpga, unsigned char reg)
{
#define SPI_SH_DELAY 1
#define TXRX_LEN 2
#define CS1     (1 << 4)
#define CS0     (1 << 3)
#define CLK     (1 << 2)
#define MISO    (1 << 1)
#define MOSI    (1 << 0)
    unsigned int odr, idr, ddr;
    unsigned char bits;
    unsigned short txrx_buf[TXRX_LEN];
    int i;
    unsigned int chip_select = 0;

    if (fpga == DID_RC_FPGA)
    {
        /* Trying to access the RC FPGA */
        chip_select = CS1;
    }
    else
    {
        /* trying to access the AD FPGA */
        chip_select = CS0;
    }

    odr = *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) & 0xe0;
    ddr = *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortADataDirectionRegOffset) & 0xe0;

    /* Setup GPIO directions and initial state */

    /* CS1, CS0, MOSI and MISO low. Set CLK high */
    odr |= CLK;
    *(volatile unsigned int *)(PC20X_GPIO_BASE + GpioPortAOutputDataRegOffset) = odr;

    /* CS1, CS0, CLK and MOSI outputs, MISO input */
    ddr |= (CS1 | CS0 | CLK | MOSI);
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

static void display_board_type (unsigned short vers_reg,
                                unsigned short id_reg)
{
    unsigned short board_type;

    if (((vers_reg & FPGA_VERS_CODE_REV_MASK) >> FPGA_VERS_CODE_REV_SHIFT) > 1)
    {
        board_type = (id_reg & V2_FPGA_DID_BOARD_TYPE_MASK) >> V2_FPGA_DID_BOARD_TYPE_SHIFT;
    }
    else
    {
        board_type = (id_reg & V1_FPGA_DID_BOARD_TYPE_MASK) >> V1_FPGA_DID_BOARD_TYPE_SHIFT;
    }

    puts("Board: ");
    switch (board_type)
    {
        case BOARD_TYPE_PC7205:
        {
            puts("PC7205\n");
            break;
        }
        case BOARD_TYPE_HDP203:
        {
            puts("HDP203\n");
            break;
        }
        default:
        {
            printf("Unknown (%u)\n", board_type);
        }
    }
}

static void display_fpga_info (unsigned short vers_reg,
                               unsigned short id_reg)
{
    puts("FPGA:  ");
    switch ((id_reg & FPGA_DID_DEVICE_ID_MASK) >> FPGA_DID_DEVICE_ID_SHIFT)
    {
        case DID_RC_FPGA:
        {
            puts("RC_FPGA");
            break;
        }
        case DID_AD_FPGA:
        {
            puts("AD_FPGA");
            break;
        }
        default:
        {
            printf("Unknown type (%u)", (id_reg & FPGA_DID_DEVICE_ID_MASK) >> FPGA_DID_DEVICE_ID_SHIFT);
        }
    }
    printf(", v%u r%u\n",
            (vers_reg & FPGA_VERS_CODE_REV_MASK) >> FPGA_VERS_CODE_REV_SHIFT,
            (vers_reg & FPGA_VERS_CODE_REL_MASK) >> FPGA_VERS_CODE_REL_SHIFT);
}
