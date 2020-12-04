/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file emac.c
* \brief Ethernet driver for the PC302.
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

#ifdef CFG_DW_EMAC

#include <malloc.h>
#include <net.h>
#ifdef CONFIG_MICREL_SWITCH
#include <i2c.h>
#endif /* CONFIG_MICREL_SWITCH */
#include <asm/io.h>
#include <asm/arch/pc302.h>
#include <asm/arch/emac.h>
#include <asm/arch/mii_phy.h>
#include <asm/arch/utilities.h>

/* Macros ------------------------------------------------------------------ */

/* !
 *  \brief Timeout value (in uS) for various EMAC operations
 */
#define EMAC_TX_TIMEOUT             (1000)

/* !
 *  \brief Timeout value (in uS) for various PHY operations
 */
#define EMAC_PHY_TIMEOUT            (4000000)

/* !
 *  \brief Length (in bytes) of a MAC address
 */
#define EMAC_LENGTH_OF_MAC_ADDRESS  (6)

/* !
 *  \brief Default receive and transmit ring lengths
 */
#define EMAC_RX_NUM_DESCRIPTOR      (8)
#define EMAC_TX_NUM_DESCRIPTOR      (2)
#define EMAC_DESCRIPTOR_BUF_SIZE    (2048)

/* !
 *  \brief Values used in the emac_priv structure
 */
#define EMAC_PHY_SPEED_10           (0)
#define EMAC_PHY_SPEED_100          (1)
#define EMAC_PHY_DUPLEX_HALF        (0)
#define EMAC_PHY_DUPLEX_FULL        (1)
#define EMAC_PHY_LINK_DOWN          (0)
#define EMAC_PHY_LINK_UP            (1)
#define EMAC_PHY_AUTO_NEG_COMPLETE  (1)

/* !
 *  \brief Macros used to read from, and write to, emac registers
 */
#define EMAC_READ(__offset) \
                 (*((volatile u32*)(PC302_EMAC_BASE + __offset)))

#define EMAC_WRITE(__value, __offset) \
                  (*((volatile u32*)(PC302_EMAC_BASE + __offset)) = __value)

/*!
 * \brief Define the auto-negotiation advertisment register
 *        value.
 *
 * Note: This advertises 100 mpbs capability only.
 */
#define PHY_AUTO_NEG_ADVERT_VALUE   (0x0181)


#define MICREL_PORT1_CTRL12_REG 0x1C
#define MICREL_PORT2_CTRL12_REG 0x2C
#define MICREL_PORT2_CTRL13_REG 0x2D

#define MICREL_PORT1_STATUS_REG 0x1E
#define MICREL_PORT2_STATUS_REG 0x2E
#define MP1S_AN_DONE            (1 << 6)
#define MP1S_LSTATUS            (1 << 5)
#define MP1S_LPA_PAUSE_CAP      (1 << 4)
#define MP1S_LPA_100FULL        (1 << 3)
#define MP1S_LPA_100HALF        (1 << 2)
#define MP1S_LPA_10FULL         (1 << 1)
#define MP1S_LPA_10HALF         (1 << 0)

#define MC13_POWER_DOWN_PHY     8

/* Constants --------------------------------------------------------------- */

/* !
 *  \brief This structure defines the Tx and Rx descriptor format
 */
struct emac_dma_descriptor
{
    unsigned int buffer;
    unsigned int status;
};

/* !
 *  \brief This structure is used to hold private data for the network code
 */
struct emac_priv
{
    /* Useful phy state */
    unsigned int auto_negotiation;
    unsigned int speed;
    unsigned int duplex;
    unsigned int link;

    /* Rx descriptor count */
    unsigned int rx_desc;

    /* Tx descriptor count */
    unsigned int tx_desc;
};

/* !
 *  \brief Statically assign some memory for the Rx & Tx,
 *         descriptors, and the rx buffers
 */
volatile static struct
emac_dma_descriptor rx_descriptor[EMAC_RX_NUM_DESCRIPTOR];

volatile static struct
emac_dma_descriptor tx_descriptor[EMAC_TX_NUM_DESCRIPTOR];

/* The rx buffer has to be 64 bit aligned otherwise very bad things happen */
__attribute__((aligned(8))) static unsigned char
rx_buffer[EMAC_RX_NUM_DESCRIPTOR][EMAC_DESCRIPTOR_BUF_SIZE];

/* Types ------------------------------------------------------------------- */

/* Prototypes--------------------------------------------------------------- */

/*!
 *
 * Read a register in a phy connected to the emac management port.
 *
 * \param phy_id The phy id of the phy to access
 * \param register_number The register to read from
 * \return The value read
 *
 */
static unsigned short emac_mii_read(unsigned int phy_id,
                                    unsigned int register_number);

/*!
 *
 * Write to a register in a phy connected to the emac management port.
 *
 * \param phy_id The phy id of the phy to access
 * \param register_number The register to write to
 * \param data The data to write
 *
 */
static void emac_mii_write(unsigned int phy_id,
                           unsigned int register_number,
                           unsigned short data);
/*!
 * Initialise the phy connected to the emac management port.
 *
 * \param dev Pointer to the eth_device structure
 *
 */
static int emac_init_phy(struct eth_device *dev);

/*!
 *
 * Obtain the link speed from the phy.
 *
 * \param dev Pointer to the eth_device structure
 *
 */
#ifndef CONFIG_MICREL_SWITCH
static void emac_phy_get_link_speed(struct eth_device *dev);
#else
/* not used for micrel_phy */
#endif /* CONFIG_MICREL_SWITCH */

/*!
 *
 * Obtain the link status from the phy.
 *
 * \param dev Pointer to the eth_device structure
 *
 */
#ifndef CONFIG_MICREL_SWITCH
static void emac_phy_get_link_status(struct eth_device *dev);
#else
static void micrel_phy_get_link_status(struct eth_device *dev);
#endif /* CONFIG_MICREL_SWITCH */

/*!
 *
 * emac_set_mac_addr()
 *
 * Set up the MAC address in the emac.
 *
 * \param dev Pointer to the eth_device structure
 *
 */
static void emac_set_mac_addr(struct eth_device *dev);

/*!
 *
 * Initialise the emac registers.
 *
 * \param dev Pointer to the eth_device structure
 *
 */
static void emac_startup(struct eth_device *dev);

/*!
 *
 * Initialise the emac buffer descriptors and phy.
 *
 * \param dev Pointer to the eth_device structure
 * \param bis Pointer to the board init structure
 * \return Zero on success, non zero on error
 *
 */
static int emac_open(struct eth_device *dev, bd_t *bis);

/*!
 *
 * Receive a packet
 *
 * \param dev Pointer to the eth_device structure
 * \return Zero on success, non zero on error
 *
 */
static int emac_rx_packet(struct eth_device *dev);

/*!
 *
 * Transmit a packet
 *
 * \param dev Pointer to the eth_device structure
 * \param packet Pointer to the packet data to transmit
 * \param length  Length (in bytes) of the packet to send
 * \return Zero on success, non zero on error
 *
 */
static int emac_tx_packet(struct eth_device *dev,
                          volatile void *packet,
                          int length);

/*!
 *
 *  Stop the emac
 *
 * \param dev Pointer to the eth_device structure
 *
 */
static void emac_halt(struct eth_device *dev);

/*!
 *
 * Initialise and register the network driver with the U-Boot network code.
 *
 * \param bis Pointer to the board init structure
 *
 */
void pc302_eth_initialize(bd_t * bis);

/* Functions --------------------------------------------------------------- */

static unsigned short emac_mii_read(unsigned int phy_id,
                                    unsigned int register_number)
{
    unsigned int write_data = 0x60020000;
    unsigned int phy_management_idle = 0;
    unsigned short value_read = 0;

    /* Mask input parameters */
    phy_id &= EMAC_PHY_ID_MASK;
    register_number &= EMAC_PHY_REG_MASK;

    write_data |= ((phy_id << EMAC_PHY_ID_SHIFT) |
                  (register_number << EMAC_PHY_REG_SHIFT));

    EMAC_WRITE(write_data, EMAC_PHY_MAINTAIN_REG_OFFSET);

    /* Wait for the phy access to complete */
    do
    {
        phy_management_idle = EMAC_READ(EMAC_NETWORK_STATUS_REG_OFFSET);
        phy_management_idle &= EMAC_PHY_MANAGEMENT_IDLE;
    }
    while(!phy_management_idle);

    /* Read back the data obtained from the phy */
    value_read = (unsigned short)EMAC_READ(EMAC_PHY_MAINTAIN_REG_OFFSET);

    return(value_read);
}

static void emac_mii_write(unsigned int phy_id,
                           unsigned int register_number,
                           unsigned short data)
{
    unsigned int write_data = 0x50020000;
    unsigned int phy_management_idle = 0;

    /* Mask input parameters */
    phy_id &= EMAC_PHY_ID_MASK;
    register_number &= EMAC_PHY_REG_MASK;

    write_data |= ((phy_id << EMAC_PHY_ID_SHIFT) |
                   (register_number << EMAC_PHY_REG_SHIFT) | data);
    EMAC_WRITE(write_data, EMAC_PHY_MAINTAIN_REG_OFFSET);

    /* Wait for the phy access to complete */
    do
    {
        phy_management_idle = EMAC_READ(EMAC_NETWORK_STATUS_REG_OFFSET);
        phy_management_idle &= EMAC_PHY_MANAGEMENT_IDLE;
    }
    while (!phy_management_idle);
}

static int emac_init_phy(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;

    /* Initialise the phy status parameters in the private data structure */
    priv->auto_negotiation = ~(EMAC_PHY_AUTO_NEG_COMPLETE);
    priv->link = EMAC_PHY_LINK_DOWN;
    priv->speed = EMAC_PHY_SPEED_10;
    priv->duplex = EMAC_PHY_DUPLEX_HALF;

#ifndef CONFIG_MICREL_SWITCH
    /* 
     * MII PHY i/f
     */
    {
        unsigned int network_control_register = 0;
        unsigned int network_config_register = 0;
        unsigned short phy_control = 0;

        /* Set phy management MDC Clock to 200 MHz (pclk) / 96 */
        network_config_register = EMAC_READ(EMAC_NETWORK_CFG_REG_OFFSET);
        network_config_register &= EMAC_MDC_CLOCK_DIV_MASK;
        network_config_register |= EMAC_MDC_CLOCK_DIV_96;
        EMAC_WRITE(network_config_register, EMAC_NETWORK_CFG_REG_OFFSET);

        /* Enable phy management */
        network_control_register = EMAC_READ(EMAC_NETWORK_CTRL_REG_OFFSET);
        network_control_register |= EMAC_MDIO_ENABLE;
        EMAC_WRITE(network_control_register, EMAC_NETWORK_CTRL_REG_OFFSET);

        /* If we are running on PC302 Rev D silicon and we are using a
         * Reduced MII (RMII) connected Ethernet Phy then we need the
         * link speed to be 100 mbps.
         */
        if ((pc302_read_device_revision() == PC302_REV_D) &&
            pc302_get_rmii_enabled())
        {
            /* Are we already set for 100 mpbs ? */
            emac_phy_get_link_speed(dev);
            if (priv->speed == EMAC_PHY_SPEED_100)
            {
                /* No need to do anything */
            }
            else
            {
                /* Setup the phy auto-negotiation advertisment register */
                emac_mii_write(CONFIG_PHY_ADDR, PHY_ANAR,
                               PHY_AUTO_NEG_ADVERT_VALUE);

                /* Re-start auto-negotiation */
                emac_mii_write(CONFIG_PHY_ADDR, PHY_BMCR,
                               (PHY_BMCR_AUTO_NEG_ENABLE | PHY_BMCR_RESTART_NEG));

                /* Allow some time for the auto-negotiation process to start */
                udelay(100);
            }
        }

        emac_phy_get_link_status(dev);

        if (priv->link == EMAC_PHY_LINK_DOWN)
        {
            /* Oops, no valid link established, time to bail out */
            return (1);
        }
        else
        {
            /* We have a valid link established */

            /* Obtain the link speed */
            emac_phy_get_link_speed(dev);

            /* Obtain the link duplex setting */
            phy_control = emac_mii_read(CONFIG_PHY_ADDR, PHY_BMCR);
            if (phy_control & PHY_BMCR_FULL_DUPLEX)
            {
                priv->duplex = EMAC_PHY_DUPLEX_FULL;
            }
        }
    }
#else /* CONFIG_MICREL_SWITCH */
    /* 
     * I2C PHY i/f
     */
    micrel_phy_get_link_status(dev);
    if (priv->link == EMAC_PHY_LINK_DOWN)
    {
        /* Oops, no valid link established, time to bail out */
        return (1);
    }
#endif /* CONFIG_MICREL_SWITCH */

    /* Report the phy setup */
    if (priv->speed == EMAC_PHY_SPEED_100)
    {
        printf("%s: 100Mbit/s\n", dev->name);
    }
    else
    {
        printf("%s: 10Mbit/s\n", dev->name);
    }

    if (priv->link == EMAC_PHY_DUPLEX_FULL)
    {
        printf("%s: Full-duplex mode\n", dev->name);
    }
    else
    {
        printf("%s: Half-duplex mode\n", dev->name);
    }

    return(0);
}


#ifndef CONFIG_MICREL_SWITCH

static void emac_phy_get_link_speed(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    unsigned short phy_control = 0;
    unsigned short phy_status = 0;
    unsigned short ana = 0;
    unsigned short anlpa = 0;
    unsigned int timebase = 0;

    /* Check to make sure the phy has auto-negotiation enabled */
    phy_control = emac_mii_read(CONFIG_PHY_ADDR, PHY_BMCR);
    if (phy_control & PHY_BMCR_AUTO_NEG_ENABLE)
    {
        /* Auto-negotiation is enabled
         * now need to check on auto-negotiation progress
         */
        phy_status = emac_mii_read(CONFIG_PHY_ADDR, PHY_BMSR);
        if (phy_status & PHY_BMSR_AUTO_NEG_ABLE)
        {
            /* The phy is auto-negotiation capable */
            timebase = get_timer(0);
            do
            {
                phy_status = emac_mii_read(CONFIG_PHY_ADDR, PHY_BMSR);
                if (phy_status & PHY_BMSR_AUTO_NEG_COMPLETE)
                {
                    break;
                }
            }
            while (get_timer (timebase) < EMAC_PHY_TIMEOUT);

            /* Read the auto-negotiation advertisment register */
            ana = emac_mii_read(CONFIG_PHY_ADDR, PHY_ANAR);

            /* Read the auto-negotiation link partner ability register */
            anlpa = emac_mii_read(CONFIG_PHY_ADDR, PHY_ANLPAR);

            anlpa &= ana;

            if (anlpa & (PHY_ANLPAR_100TXF | PHY_ANLPAR_100TXH))
            {
                priv->speed = EMAC_PHY_SPEED_100;
            }
            else
            {
                priv->speed = EMAC_PHY_SPEED_10;
            }
        }
        else
        {
            /* We are not capable of performing auto-negotiation
             * so we just get the speed settings from the Phy control reg
             */
            if (phy_control & PHY_BMCR_100MB)
            {
                priv->speed = EMAC_PHY_SPEED_100;
            }
            else
            {
                priv->speed = EMAC_PHY_SPEED_10;
            }
        }
    }
    else
    {
        /* Auto-negotiation is not enabled
         * so we just get the speed settings from the Phy control reg
         */
        if (phy_control & PHY_BMCR_100MB)
        {
            priv->speed = EMAC_PHY_SPEED_100;
        }
        else
        {
            priv->speed = EMAC_PHY_SPEED_10;
        }
    }
}

static void emac_phy_get_link_status(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    unsigned  short phy_status = 0;
    unsigned int timebase = 0;

    /* Check on auto-negotiation progress */
    phy_status = emac_mii_read(CONFIG_PHY_ADDR, PHY_BMSR);
    if (phy_status & PHY_BMSR_AUTO_NEG_ABLE)
    {
        /* The phy is auto-negotiation capable */
        timebase = get_timer(0);
        do
        {
            phy_status = emac_mii_read(CONFIG_PHY_ADDR, PHY_BMSR);
            if (phy_status & PHY_BMSR_AUTO_NEG_COMPLETE)
            {
                /* We have a successful auto-negotiation */
                printf("%s: Auto-Negotiation complete\n", dev->name);
                priv->auto_negotiation = EMAC_PHY_AUTO_NEG_COMPLETE;
                break;
            }
        }
        while (get_timer(timebase) < EMAC_PHY_TIMEOUT);
    }

    if (phy_status & PHY_BMSR_LINK_UP)
    {
        /* We have a valid link established */
        printf("%s: Link up\n", dev->name);
        priv->link = EMAC_PHY_LINK_UP;
    }
    else
    {
        /* We do not have a valid link established */
        printf("%s: Link down !\n", dev->name);
        priv->link = EMAC_PHY_LINK_DOWN;
    }
}

#else /* CONFIG_MICREL_SWITCH */

static void micrel_phy_get_link_status(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    unsigned int timebase = 0;
    int lpa;

    printf("%s: polling for Auto-Negotiation complete\n", dev->name);
    timebase = get_timer(0);
    do
    {
        /* Read link status */
        lpa = i2c_reg_read(CFG_MICREL_SWITCH_ADDR, MICREL_PORT1_STATUS_REG);
        if (lpa & MP1S_AN_DONE)
        {
            /* We have a successful auto-negotiation */
            printf("%s: Auto-Negotiation complete\n", dev->name);
            priv->auto_negotiation = EMAC_PHY_AUTO_NEG_COMPLETE;
            break;
        }
    }
    while (get_timer(timebase) < EMAC_PHY_TIMEOUT);

    if ((lpa & MP1S_LSTATUS) == 0)
    {
        /* We do not have a valid link established */
        printf("%s: Link down !\n", dev->name);
        priv->link = EMAC_PHY_LINK_DOWN;
    }
    else
    {
        /* We have a valid link established */
        printf("%s: Link up\n", dev->name);
        priv->link = EMAC_PHY_LINK_UP;

        /* Report back link speed etc */
        priv->speed = EMAC_PHY_SPEED_10;
        priv->duplex = EMAC_PHY_DUPLEX_HALF;
        if (lpa & (MP1S_LPA_100FULL | MP1S_LPA_100HALF)) {
            priv->speed = EMAC_PHY_SPEED_100;

            if (lpa & MP1S_LPA_100FULL) {
                priv->duplex = EMAC_PHY_DUPLEX_FULL;
            }
        } else {
            if (lpa & MP1S_LPA_10FULL) {
                priv->duplex = EMAC_PHY_DUPLEX_FULL;
            }
        }
    }

} /* micrel_phy_get_link_status() */

#endif /* CONFIG_MICREL_SWITCH */

static void emac_set_mac_addr(struct eth_device *dev)
{
    unsigned int mac_addr_bottom = 0;
    unsigned int mac_addr_top = 0;

    mac_addr_bottom = *((unsigned int *)dev->enetaddr);
    mac_addr_top = *((unsigned short *)(dev->enetaddr + 4));

    EMAC_WRITE(mac_addr_bottom, EMAC_SPEC_ADDR_1_BOT_31_0_REG_OFFSET);
    EMAC_WRITE(mac_addr_top, EMAC_SPEC_ADDR_1_TOP_47_32_REG_OFFSET);
}

static void emac_startup(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    unsigned int network_config_register = 0;
    unsigned int network_control_register = 0;
    unsigned int dma_config_register = 0;

    /* Make sure the Tx & Rx are halted */
    network_control_register = EMAC_READ(EMAC_NETWORK_CTRL_REG_OFFSET);
    network_control_register &= ~(EMAC_RX_ENABLE | EMAC_TX_ENABLE);
    EMAC_WRITE(network_control_register, EMAC_NETWORK_CTRL_REG_OFFSET);

    /* Setup the Rx Buffer Queue Base Address */
    EMAC_WRITE((unsigned int)&rx_descriptor, EMAC_RX_BUFF_Q_BASE_ADDR_REG_OFFSET);

    /* Setup the Tx Buffer Queue Base Address */
    EMAC_WRITE((unsigned int)&tx_descriptor, EMAC_TX_BUFF_Q_BASE_ADDR_REG_OFFSET);

    /* Setup the size of the DMA Receive Buffer */
    dma_config_register = EMAC_READ(EMAC_DMA_CFG_REG_OFFSET);
    dma_config_register &= ~(EMAC_DMA_RX_BUFFER_SIZE_MASK);
    dma_config_register |= EMAC_DMA_RX_BUFFER_SIZE;
    EMAC_WRITE(dma_config_register, EMAC_DMA_CFG_REG_OFFSET);

    /* Setup the Network Configuration Register */
    network_config_register = EMAC_READ(EMAC_NETWORK_CFG_REG_OFFSET);
    network_config_register |= EMAC_64_BIT_AMBA_DATA_BUS_WITDH;
    network_config_register |= EMAC_LENGTH_FIELD_ERROR_FRAME_DISCARD;
    network_config_register |= EMAC_FCS_REMOVE;

    if (priv->duplex == EMAC_PHY_DUPLEX_FULL)
    {
        network_config_register |= EMAC_FULL_DUPLEX;
    }
    else
    {
        network_config_register &= ~(EMAC_FULL_DUPLEX);
    }

    if (priv->speed == EMAC_PHY_SPEED_100)
    {
        network_config_register |= EMAC_SPEED_100_MBPS;
    }
    else
    {
        network_config_register &= ~(EMAC_SPEED_100_MBPS);
    }

    EMAC_WRITE(network_config_register, EMAC_NETWORK_CFG_REG_OFFSET);

    /* Setup the Network Control Register */
    network_control_register = EMAC_READ(EMAC_NETWORK_CTRL_REG_OFFSET);
    network_control_register |= (EMAC_RX_ENABLE | EMAC_TX_ENABLE);
    EMAC_WRITE(network_control_register, EMAC_NETWORK_CTRL_REG_OFFSET);
}

static int emac_open(struct eth_device *dev, bd_t *bis)
{
    struct emac_priv *priv = dev->priv;
    unsigned int i = 0;;
    unsigned int buffer_address = 0;
    unsigned int return_code = 0;

    /* Initialise the Rx descriptor count */
    priv->rx_desc = 0;

    /* Initialise the Tx descriptor count */
    priv->tx_desc = 0;

    /* Initialise the Rx descriptors */
    for (i = 0; i < EMAC_RX_NUM_DESCRIPTOR; i++)
    {
        buffer_address = (unsigned int)&rx_buffer[i][0];
        if (i == (EMAC_RX_NUM_DESCRIPTOR - 1))
        {
            /* we are on the last descriptor entry */
            buffer_address |= EMAC_RX_DESC_WRAP;
        }
        rx_descriptor[i].buffer = buffer_address;
        rx_descriptor[i].status = 0;
    }

    /* Initialise the Tx descriptors */
    for (i = 0; i < EMAC_TX_NUM_DESCRIPTOR; i++)
    {
        tx_descriptor[i].buffer = 0;
        tx_descriptor[i].status = EMAC_TX_DESC_HOST_OWN;

        if (i == (EMAC_TX_NUM_DESCRIPTOR - 1))
        {
            /* we are on the last descriptor entry */
            tx_descriptor[i].status |= EMAC_TX_DESC_WRAP;
        }
    }

    /* Check out the phy status */
    return_code = emac_init_phy(dev);
    if (return_code != 0)
    {
        /* Oops, we've had an error */
        return(return_code);
    }

    /* Initialise the emac registers */
    emac_startup(dev);

    return (0);
}

static int emac_rx_packet(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    unsigned int start_of_frame = 0;
    unsigned int end_of_frame = 0;
    unsigned int frame_error = 0;
    unsigned int length = 0;
    void * buffer_start;

    if (rx_descriptor[priv->rx_desc].buffer & EMAC_RX_DESC_HOST_OWN)
    {
        /* We have some received data */

        /* Make sure we have a whole frame */
        start_of_frame = rx_descriptor[priv->rx_desc].status & EMAC_RX_DESC_START_OF_FRAME;
        end_of_frame = rx_descriptor[priv->rx_desc].status & EMAC_RX_DESC_END_OF_FRAME;
        if (start_of_frame && end_of_frame)
        {
            /* We have a complete frame */
            length = rx_descriptor[priv->rx_desc].status & EMAC_RX_DESC_LENGTH_MASK;

            /* Noodle the buffer start address for the higher level network stack
               The start address should be 8 byte aligned, also bits 0 & 1 can be
               set by the emac, so these need to be masked out as well */

            buffer_start = (void *)
                           (rx_descriptor[priv->rx_desc].buffer & 0xFFFFFFF8);

            /* Send received packet to the higher network layers */
            NetReceive(buffer_start, length);
        }
        else
        {
            /* Oops, not a complete frame */
            frame_error++;
        }

        /* Reclaim the buffer just used */
        rx_descriptor[priv->rx_desc].buffer &= ~(EMAC_RX_DESC_HOST_OWN);

        /* Increment the Rx descriptor counter */
        priv->rx_desc++;
        if (priv->rx_desc == EMAC_RX_NUM_DESCRIPTOR)
        {
            /* We have exhausted the supply of Rx descriptors */
            priv->rx_desc = 0;
        }
    }

    if (frame_error)
    {
        printf ("frame error\n");
        return (1);
    }

    return(0);
}

static int emac_tx_packet(struct eth_device *dev,
                          volatile void *packet,
                          int length)
{
    struct emac_priv *priv = dev->priv;
    unsigned int i = 0;
    unsigned int network_control_register = 0;

    /* Set up the Tx descriptor */

    /* Make sure the wrap bit is set for the last descriptor */
    if (priv->tx_desc == (EMAC_TX_NUM_DESCRIPTOR - 1))
    {
        /* we are on the last descriptor entry */
        tx_descriptor[priv->tx_desc].status = EMAC_TX_DESC_WRAP;
    }

    tx_descriptor[priv->tx_desc].status |= length & EMAC_TX_BUFFER_LENGTH_MASK;
    tx_descriptor[priv->tx_desc].status |= EMAC_TX_LAST_BUFFER;
    tx_descriptor[priv->tx_desc].status &= ~(EMAC_TX_NO_CRC_APPEND);
    tx_descriptor[priv->tx_desc].status &= ~(EMAC_TX_DESC_HOST_OWN);

    /* Setup the Tx descriptor buffer */
    tx_descriptor[priv->tx_desc].buffer = (unsigned int)packet;

    /* Start the packet transmission */
    network_control_register = EMAC_READ(EMAC_NETWORK_CTRL_REG_OFFSET);
    network_control_register |= EMAC_START_TX;
    EMAC_WRITE(network_control_register, EMAC_NETWORK_CTRL_REG_OFFSET);

    /* Wait for transmission to complete */
    for (i = 0; i <= EMAC_TX_TIMEOUT; i++)
    {
	if (tx_descriptor[priv->tx_desc].status & EMAC_TX_DESC_HOST_OWN)
        {
            /* The emac has completed transmission */
            break;
        }
	udelay(1);
    }

    /* Increment the Tx descriptor counter */
    priv->tx_desc++;
    if (priv->tx_desc == EMAC_TX_NUM_DESCRIPTOR)
    {
        /* We have exhausted the supply of Tx descriptors */
        priv->tx_desc = 0;
    }

    /* We could add some error reporting in here, but no one cares anyway */
    return(0);
}

static void emac_halt(struct eth_device *dev)
{
    unsigned int status_register = 0;

    /* Halt the Tx & Rx */
    EMAC_WRITE(0, EMAC_NETWORK_CTRL_REG_OFFSET);

    /* Clear the statistics counters */
    EMAC_WRITE(EMAC_CLEAR_STATS_REGISTERS, EMAC_NETWORK_CTRL_REG_OFFSET);

    /* Clear the Tx status registers */
    status_register = EMAC_READ(EMAC_TX_STATUS_REG_OFFSET);
    EMAC_WRITE(status_register, EMAC_TX_STATUS_REG_OFFSET);

    /* Clear the Rx status registers */
    status_register = EMAC_READ(EMAC_RX_STATUS_REG_OFFSET);
    EMAC_WRITE(status_register, EMAC_RX_STATUS_REG_OFFSET);
}

void pc302_eth_initialize(bd_t * bis)
{
    struct eth_device *dev = NULL;
    struct emac_priv *priv = NULL;

#ifdef CONFIG_MICREL_SWITCH
    /* Disable the BTS port on the switch */
    i2c_reg_write(CFG_MICREL_SWITCH_ADDR, MICREL_PORT2_CTRL13_REG, MC13_POWER_DOWN_PHY);
#endif /*  CONFIG_MICREL_SWITCH */

    /* Create some storage for useful structures */
    dev = (struct eth_device *) malloc(sizeof (*dev));
    priv = (struct emac_priv *) malloc(sizeof (*priv));
    dev->priv = priv;

    sprintf(dev->name, "pc302_emac");

    /* Reset the private data */
    memset(priv, 0, sizeof(struct emac_priv));

    /* Copy the hardware address of the interface */
    memcpy(dev->enetaddr, bis->bi_enetaddr, EMAC_LENGTH_OF_MAC_ADDRESS);

    /* Set the hardware MAC address */
    emac_set_mac_addr(dev);

    dev->init = emac_open;
    dev->recv = emac_rx_packet;
    dev->send = emac_tx_packet;
    dev->halt = emac_halt;

    /* Register our emac with the networking environment */
    eth_register(dev);
}

#endif /* CFG_DW_EMAC */
