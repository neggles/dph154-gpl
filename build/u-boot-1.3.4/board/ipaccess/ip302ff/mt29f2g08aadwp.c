/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file .mt29f2g08aadwp.c
* \brief Support for the NAND Flash device fitted on PC7302 platform.
*
* Copyright (c) 2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/
 
/*
 * (C) Copyright 2009 SAGEM Communications
 * (C) Copyright 2006 DENX Software Engineering
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

#ifdef CONFIG_CMD_NAND

#include <asm/arch/pc302.h>
#include <asm/arch/gpio.h>
#include <nand.h>

/* Macros ------------------------------------------------------------------ */

/* Define which gpio bits are used to control the NAND Flash
 *
 * Note: These pin definitions mean that we can only use NAND
 *       Flash if we are running U-Boot from RAM and have NOT booted
 *       the device from parallel NOR Flash.
 *
 * Note: These GPIO bits are all ARM GPIO bits.
 */
#define CLE   GPIO_BIT_4
#define ALE   GPIO_BIT_3
#define NCE   GPIO_BIT_2
#define READY GPIO_BIT_1

/*!
 * \brief Hardware specific access to control-lines
 * \param mtd, pointer to the mtd_info structure
 * \param dat, data to write to the device
 * \param ctrl, control data to set up the transaction
 *
 */
static void mt29f2g08aadwp_cmd_ctrl(struct mtd_info *mtd,
				    int dat,
				    unsigned int ctrl)
{
    struct nand_chip *this = mtd->priv;

    unsigned int odr;

    /* Read the data register for port A,
       Note: This will return the state of the pins programmed
             to be outputs.
    */
    odr = *(volatile unsigned int *)(PC302_GPIO_BASE +
				     GPIO_SW_PORT_A_DR_REG_OFFSET) & 0xFF;

    if (ctrl & NAND_CTRL_CHANGE)
    {
        if (ctrl & NAND_NCE)
        {
            /* Assert the chip select */
            odr &= ~NCE;
            *(volatile unsigned int *)(PC302_GPIO_BASE +
				       GPIO_SW_PORT_A_DR_REG_OFFSET) = odr;

	    if (ctrl & NAND_CLE)
            {
	        /* Assert CLE */
                odr |= CLE;
		*(volatile unsigned int *)(PC302_GPIO_BASE +
					   GPIO_SW_PORT_A_DR_REG_OFFSET) = odr;
            }
    	    else
	    {
                /* Negate CLE */
                odr &= ~CLE;
		*(volatile unsigned int *)(PC302_GPIO_BASE +
					   GPIO_SW_PORT_A_DR_REG_OFFSET) = odr;
            }

	    if (ctrl & NAND_ALE)
	    {
                /* Assert ALE */
                odr |= ALE;
		*(volatile unsigned int *)(PC302_GPIO_BASE +
					   GPIO_SW_PORT_A_DR_REG_OFFSET) = odr;
            }
	    else
            {
		/* Negate ALE */
                odr &= ~ALE;
		*(volatile unsigned int *)(PC302_GPIO_BASE +
					   GPIO_SW_PORT_A_DR_REG_OFFSET) = odr;
            }
        }
        else
        {
	    /* Negate the chip select */
            odr |= NCE;
	    *(volatile unsigned int *)(PC302_GPIO_BASE +
		     		       GPIO_SW_PORT_A_DR_REG_OFFSET) = odr;

        }
    }

    /* If we have data to write, write it */
    if (dat != NAND_CMD_NONE)
    {
	*(volatile unsigned char *)(this->IO_ADDR_W) = (unsigned char)dat;
    }

}

/*!
 * \brief Return the state of the NAND busy output
 * \param mtd, pointer to the mtd_info structure
 * \return 0 - nand busy
 *         1 - nand ready
 *
 */
static int mt29f2g08aadwp_dev_ready(struct mtd_info *mtd)
{
    unsigned int idr;

    idr = *(volatile unsigned int *)(PC302_GPIO_BASE +
			             GPIO_EXT_PORT_A_REG_OFFSET) & 0xFF;

    return ((idr & READY) == READY)? 1: 0;
}

/*
 * Board-specific NAND initialization. The following members of the
 * argument are board-specific (per include/linux/mtd/nand.h):
 * - IO_ADDR_R?: address to read the 8 I/O lines of the flash device
 * - IO_ADDR_W?: address to write the 8 I/O lines of the flash device
 * - hwcontrol: hardwarespecific function for accesing control-lines
 * - dev_ready: hardwarespecific function for  accesing device ready/busy line
 * - enable_hwecc?: function to enable (reset)  hardware ecc generator. Must
 *   only be provided if a hardware ECC is available
 * - eccmode: mode of ecc, see defines
 * - chip_delay: chip dependent delay for transfering data from array to
 *   read regs (tR)
 * - options: various chip options. They can partly be set to inform
 *   nand_scan about special functionality. See the defines for further
 *   explanation
 * Members with a "?" were not set in the merged testing-NAND branch,
 * so they are not set here either.
 */
int board_nand_init(struct nand_chip *nand)
{
    unsigned int ddr, dr;

    /* Setup the gpio data direction register */
    ddr = *(volatile unsigned int *)(PC302_GPIO_BASE +
			             GPIO_SW_PORT_A_DDR_REG_OFFSET) & 0xFF;

    /* The outputs */
    ddr |= (CLE | ALE | NCE);

    /* The inputs */
    ddr &= ~READY;

    *(volatile unsigned int *)(PC302_GPIO_BASE +
	    		       GPIO_SW_PORT_A_DDR_REG_OFFSET) = ddr;

    /* Setup a 'safe' initial value on the nand control pins */
    dr = NCE;
    dr &= ~(CLE | ALE);
    *(volatile unsigned int *)(PC302_GPIO_BASE +
		               GPIO_SW_PORT_A_DR_REG_OFFSET) = dr;

    /* Populate some members of the nand structure */
    nand->cmd_ctrl = mt29f2g08aadwp_cmd_ctrl;
    nand->ecc.mode = NAND_ECC_SOFT;
    nand->dev_ready = mt29f2g08aadwp_dev_ready;
    nand->IO_ADDR_R = (void __iomem *)CONFIG_SYS_NAND_BASE;
    nand->IO_ADDR_W = (void __iomem *)CONFIG_SYS_NAND_BASE;

    return 0;
}
#endif /* CONFIG_CMD_NAND */
