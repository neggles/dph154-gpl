/*
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *
 *  board/picochip/cpe20x/firecracker_emac.c
 *
 * Copyright (c) 2007 picoChip Designs Ltd.
 *
 * Description:
 *
 * This module provides the network interface to the firecracker EMAC
 * hardware.
 *
 * References:
 *
 * Synopsys DesignWare Ethernet Universal Databook Version 3.2
 *  November 2005.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * v0.03 03jan07    - Port to the bootloader (U-Boot)
 * v0.02 02jan07    - Fixed performance issues by using SLAB allocator.
 *                  - Use hardware checksumming.
 *                  - Disable fragmented packet handling code.
 * v0.01 09nov06    - Initial version
 */

#include <common.h>
#include <malloc.h>
#include <net.h>
#include <asm/io.h>
#include <pci.h>

#include <asm/arch/pc20x.h>
#include <asm/arch/emac.h>


/* Stop and reset timeouts. These are pure guesswork! */
#define EMAC_RESET_US                       (50)
#define EMAC_STOP_US                        (50)
#define EMAC_STOP_COUNT                     (10)


/* EMAC_RX_CONTROL_FIELD_INIT and EMAC_TX_CONTROL_FIELD_INIT are the setup
 * value of the receive and transmit descriptor control field respectively
 */
#define EMAC_RX_CONTROL_FIELD_INIT          (0)
#define EMAC_TX_CONTROL_FIELD_INIT                  \
        (EMAC_TX_DESC_CNTL_ENABLE_INT_COM |         \
        EMAC_TX_DESC_CNTL_FIRST_SEG |               \
        EMAC_TX_DESC_CNTL_LAST_SEG)


/* EMAC_RX_REJECTION_MASK is a collection of the receive descriptor status
 * bits that if set indicate that the received data is invalid and should
 * be discarded.
 */
#define EMAC_RX_REJECTION_MASK                      \
    (EMAC_DESC_STATUS_ERR_SUM |                     \
     EMAC_RX_DESC_STATUS_FAIL_SRC_ADDR |            \
     EMAC_RX_DESC_STATUS_LEN_ERR |                  \
     EMAC_RX_DESC_STATUS_DRIBBLE_ERR)

/* Timeout value (in uS) for various PHY operations */
#define EMAC_PHY_TIMEOUT            (4000000)

/* timeout for various */
#define TOUT_LOOP   100000

/* MII_WAIT_TIMEOUT - defines the number of jiffies to wait for the MII
 * busy flag to be reset by the hardware.
 */
#define MII_WAIT_TIMEOUT 10

/* EMAC_DMA_BUS_WIDTH defines the bus width of the EMAC DMA */
#define EMAC_DMA_BUS_WIDTH_SHIFT    (2)
#define EMAC_DMA_BUS_WIDTH          (1 << EMAC_DMA_BUS_WIDTH_SHIFT) /* 4 */
#define EMAC_DMA_BUS_WIDTH_MASK     (EMAC_DMA_BUS_WIDTH - 1)        /* b'0011 */

/* The maximum size of a single DMA transfer, allowing for size alignment */
#define EMAC_MAX_DMA_LENGTH         (2032)

/* The maximum packet size that can be handled by this driver */
#define EMAC_MAX_PACKET_LENGTH      (EMAC_MAX_DMA_LENGTH * 2)

/* The length mask to be applied to the receive descriptors */
#define EMAC_RX_LENGTH_MASK         (0xf)
#define EMAC_RX_LENGTH_ALLIGN       (16)

#define ETH_LENGTH_OF_ADDRESS 6

#if 0
/* Default fixed MAC address to use in this driver. The hardware is read
 * at initialisation and if no valid address is found there, this
 * one is used.
 */
static unsigned char default_mac_address[ETH_LENGTH_OF_ADDRESS] = {
    0x00, 0x15, 0xe1, 0x00, 0x00, 0x00
};
#endif

/* Default receive and transmit ring lengths */
#define EMAC_RX_RING_LENGTH 8
#define EMAC_TX_RING_LENGTH 1

/* CSR_CLOCK_RANGE defines the clock range setting to use */
#define EMAC_CSR_CLOCK_RANGE EMAC_GMII_CSR_RANGE_100_150

/* Phy registers */
#define PHY_CTL 0x00
#define PHY_STS 0x01

/* Phy control bits */
#define RESET       0x8000   /* reset, NB read back until 0 */
#define AUTONEG_EN  0x1000   /* Autonegotiation enable */
#define POWER_DOWN  0x0800   /* Power down */
#define ISOLATE     0x0400   /* Isolate mii i/f electrically */
#define AUTONEG     0x0200   /* Restart Autonegotiation */
#define COLLISION   0x0080   /* Collision test */
#define SPEED_10    0x0000   /* 10 Mbit/s */
#define SPEED_100   0x2000   /* 100 Mbit/s */
#define SPEED_1000  0x0040   /* 1000 Mbit/s */
#define DUPLEX      0x0100   /* Duplex mode */
#define LOOPBACK    0x4000

/* Phy status bits: */
#define AUTOCMPLT            0x0020 /* Autonegotiation completed */
#define REMOTE_FAULT         0x0010 /* Remote fault condition */
#define AUTONEG_CAPABLE      0x0008 /* Autonegotiation capable */
#define LINK                 0x0004 /* Link status */
#define JABBER_DETECT        0x0002 /* Jabber detect */
#define EXTENDED_REGS        0x0001 /* Extended resister capabilities */


/* emac_hw_desc mirrors the EMAC DMA descriptor. It must have the same
 * endian and packing as the hardware.
 * These are used for both receive and transmit descriptors.
 */
struct emac_dma_desc {
    u32 status;
    u32 control;
    u32 buffer1;
    u32 buffer2;
};


/* emac_desc structure is a structure that the accessed by the DMA hardware.
 * Along with the DMA accessed data (that must appear at the start of the
 * structure) this also holds management data used by the driver.
 * A facility of the DMA hardware that allows a gap between descriptors
 * in memory is utilised for the additional management information
 */
struct emac_desc {
    /* Hardware Descriptor (must be first) */
    struct emac_dma_desc dma;

    /* Pointer to the device that uses this descriptor */
    struct eth_device *dev;

    /* Pointer to the ring used by this descriptor */
    struct emac_ring *ring;

    /* Pointer to the associated socket buffer wired up to this
     * descriptor or NULL if it is not wired up to a socket buffer.
     */
    volatile void *skb;

    /* The length of the mapped buffer associated with this descriptor.
     * Used when unmapping.
     */
    unsigned int length;
};


/* emac_aligned_desc is a type that is big enough to hold the emac_desc
 * padded up to the end of the last word (4 bytes). It is used for size
 * information when allocating or traversing a descriptor ring by casting
 * to this type.
 */
struct emac_aligned_desc {
    u32 data[((sizeof(struct emac_desc) - 1) / 4) + 1];
};

/* emac_ring structure holds ring management and the address in
 * DMA able memory of the hardware descriptor ring.
 */
struct emac_ring {

    /* Pointer to the start of the descriptor ring in DMA able memory */
    struct emac_aligned_desc *desc;

    /* Physical address of the above descriptor ring, for the hardware */
    dma_addr_t dma_ring_start;

    /* The maximum number of elements in the ring */
    unsigned int length;

    /* The number of elements not in use */
    unsigned int space;

    /* The next to be processed by the driver */
    struct emac_desc *next;
    unsigned int next_idx;

    /* The next to be processed by the hardware */
    struct emac_desc *next_dma;
    unsigned int next_dma_idx;

    /* Pointer to the device that uses this ring */
    struct eth_device *dev;
};


struct eth_device_stats {

    unsigned long   rx_packets;     /* total packets received   */
    unsigned long   tx_packets;     /* total packets transmitted    */
    unsigned long   rx_bytes;       /* total bytes received     */
    unsigned long   tx_bytes;       /* total bytes transmitted  */
    unsigned long   rx_errors;      /* bad packets received     */
    unsigned long   tx_errors;      /* packet transmit problems */
    unsigned long   rx_dropped;     /* no space in linux buffers    */
    unsigned long   tx_dropped;     /* no space available in linux  */
    unsigned long   multicast;      /* multicast packets received   */
    unsigned long   collisions;

    /* detailed rx_errors: */
    unsigned long   rx_length_errors;
    unsigned long   rx_over_errors;     /* receiver ring buff overflow  */
    unsigned long   rx_crc_errors;      /* recved pkt with crc error    */
    unsigned long   rx_frame_errors;    /* recv'd frame alignment error */
    unsigned long   rx_fifo_errors;     /* recv'r fifo overrun      */
    unsigned long   rx_missed_errors;   /* receiver missed packet   */

    /* detailed tx_errors */
    unsigned long   tx_aborted_errors;
    unsigned long   tx_carrier_errors;
    unsigned long   tx_fifo_errors;
    unsigned long   tx_heartbeat_errors;
    unsigned long   tx_window_errors;

    /* for cslip etc */
    unsigned long   rx_compressed;
    unsigned long   tx_compressed;
};


/* This structure is private to each device. It is used to pass
 * packets in and out, so there is place for a packet
 */
struct emac_priv {

    /* Statistics counters */
    struct eth_device_stats stats;

    /* Transmit and Receive descriptor rings */
    struct emac_ring* tx_ring;
    struct emac_ring* rx_ring;

    /* PHY state */
    int speed;
    int duplex;
    int link;
    int oldspeed;
    int oldduplex;
    int oldlink;

    /* Levels of speed, duplex and link */
#define PHY_SPEED_10    10
#define PHY_SPEED_100   100
#define PHY_DUPLEX_HALF 0
#define PHY_DUPLEX_FULL 1
#define PHY_LINK_DOWN   0
#define PHY_LINK_UP     1

    /* Mirrored device registers */
    u32 op_mode;
};


/* Static data structures: */
static struct emac_ring rings[2];       /* One receive and one transmit ring */
#define RING_DESC_BUF_SIZE  1024*10     /* Big enough for the Rx or Tx descs */
static unsigned char descriptor_buf[2][RING_DESC_BUF_SIZE];
#define RX_SKB_BUFLEN 1600
static unsigned char rx_skb_buf[EMAC_RX_RING_LENGTH][RX_SKB_BUFLEN];


/* Debugging levels: */
#define LVL_FATAL       2   /* fatal error conditions         */
#define LVL_ERR         3   /* error conditions         */
#define LVL_WARNING     4   /* warning conditions           */
#define LVL_NOTICE      5   /* normal but significant condition */
#define LVL_INFO        6   /* informational            */
#define LVL_DEBUG       7   /* debug-level messages         */
#define LVL_TRACE       8   /* trace messages         */
#define LVL_TRACE_IO    9   /* Register IO trace messages         */


/* Define ENABLE_DEBUGGING to include code for debugging via printf.
 * The value of ENABLE_DEBUGGING specifies the default debug level.
 */
//#define ENABLE_DEBUGGING LVL_TRACE
#undef ENABLE_DEBUGGING


/* Macros for register read/write. These hide the virtual addressing.
 */
#ifndef ENABLE_DEBUGGING

#define EMAC_READ(__offset) \
    *((volatile u32*)(PC20X_EMAC_BASE + __offset))

#define EMAC_WRITE(__value, __offset) \
    *((volatile u32*)(PC20X_EMAC_BASE + __offset)) = __value

#define DB(__params)

#else /* ENABLE_DEBUGGING */

#define EMAC_READ(__offset) \
    debug_ioread32(PC20X_EMAC_BASE + __offset)

#define EMAC_WRITE(__value, __offset) \
    debug_iowrite32(__value, (PC20X_EMAC_BASE + __offset))

#define DB(__params) dbug_print __params

/* Debugging level for this module */
static volatile int debug_lvl = ENABLE_DEBUGGING;

static void dbug_print(int lvl, const char *fmt, ...)
{
    static char lvl_ch[10] = {
        '0', '1', 'F', 'E', 'W', 'N', 'I', 'D', 'T', '9'
    };
#define MAX_FORMAT_SIZE 1024
    char buf[MAX_FORMAT_SIZE];
    va_list args;
    va_start(args, fmt);

    if (debug_lvl >= lvl) {

        vsprintf(buf, fmt, args);
        printf("emac <%c>: %s", lvl_ch[lvl], buf);
    }
}

static void debug_iowrite32(u32 val, unsigned int paddr)
{
    DB((LVL_TRACE_IO, "iowrite32(0x%08x)=0x%08x\n", paddr, val));
    *((volatile u32*)paddr) = val;
}

static unsigned int debug_ioread32(unsigned int paddr)
{
    unsigned int val = *((volatile u32*)paddr);
    DB((LVL_TRACE_IO, "ioread32(0x%08x)=0x%08x\n", paddr, val));
    return val;
}

static void print_ping_seq(volatile void *skb)
{
    unsigned char *buf = (unsigned char *)skb;
    DB((LVL_DEBUG, "ping seq - 0x%02x 0x%02x\n", buf[0x28], buf[0x29]));
}

static void print_ring(struct emac_ring *ring, char host, char dma, u32 hw_next)
{
    int i;
    char str[200];
    static volatile u32 hw_next_off;
    static volatile int nw_next_idx;

    hw_next_off = hw_next - ring->dma_ring_start;
    nw_next_idx = hw_next_off / sizeof(struct emac_aligned_desc);

    for (i = 0; i < ring->length; ++i) {
        if (i == nw_next_idx) {
            if (((struct emac_desc *)(&ring->desc[i]))->dma.status &
                    EMAC_DESC_STATUS_OWNER) {
                str[i] = 'D';
            }
            else {
                str[i] = 'H';
            }
        }
        else {
            if (((struct emac_desc *)(&ring->desc[i]))->dma.status &
                    EMAC_DESC_STATUS_OWNER) {
                str[i] = 'd';
            }
            else {
                str[i] = 'h';
            }
        }
    }
    str[i] = 0;

    DB((LVL_DEBUG, "%s\n", str));
}

static void print_rings(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    static int rate_limit = 0;
    u32 hw_next_tx = EMAC_READ(EMAC_DMA_CURR_TX_DESC_REG_OFFSET);
    u32 hw_next_rx = EMAC_READ(EMAC_DMA_CURR_RX_DESC_REG_OFFSET);

    if ((++rate_limit % 8) == 0) {
        print_ring(priv->rx_ring, 'v', '.', hw_next_rx);
        print_ring(priv->tx_ring, '.', '^', hw_next_tx);
    }
}

#endif /* ENABLE_DEBUGGING */


/*
 * Move the next pointer in a ring on to the next descriptor
 */
static struct emac_desc *emac_next_desc(struct emac_ring *ring)
{
    ring->next_idx++;

    if (ring->next_idx == ring->length) {
        ring->next_idx = 0;
    }

    ring->next = (struct emac_desc *)&ring->desc[ring->next_idx];

    DB((LVL_TRACE, "Next idx %u\n", ring->next_idx));

    return ring->next;
}


/*
 * Move the next DMA pointer in a ring on to the next descriptor
 */
static struct emac_desc *emac_next_dma_desc(struct emac_ring *ring)
{
    ring->next_dma_idx++;

    if (ring->next_dma_idx == ring->length) {
        ring->next_dma_idx = 0;
    }

    ring->next_dma = (struct emac_desc *)&ring->desc[ring->next_dma_idx];

    DB((LVL_TRACE, "Next DMA idx %u\n", ring->next_dma_idx));

    return ring->next_dma;
}



/* Reset the ring pointers state */
static void emac_reset_ring_state(struct emac_ring *ring)
{
    ring->space = ring->length;
    ring->next_idx = 0;
    ring->next = (struct emac_desc *)ring->desc;
    ring->next_dma_idx = 0;
    ring->next_dma = (struct emac_desc *)ring->desc;
}





/* Read the bus for PHY at addr mii_id, register regnum, and
 * return the value.  Clears miimcom first.  All PHY
 */
static int emac_mii_read(int mii_id, int regnum)
{
    unsigned int value = 0;

    /* The emac_phy_[read|write] functions ensure that their operation
     * has completed before they exit so we do not have to wait
     * for a previous operation to finish here and we can go ahead and
     * use the MII interface.
     */

    /* Setup the address register and start the read */
    EMAC_WRITE(
            ((mii_id << EMAC_GMII_ADDRESS_SHIFT) |
             (regnum << EMAC_GMII_GMII_REG_SHIFT) |
             EMAC_CSR_CLOCK_RANGE |
             EMAC_GMII_BUSY),
            EMAC_MAC_GMII_ADDR_REG_OFFSET);

    /* Wait for the MII to finish */
    udelay(1000);

    /* Read back the register contents */
    value = EMAC_READ(EMAC_MAC_GMII_DATA_REG_OFFSET);

    return value;
}


/* Enable/disable, EMAC DMA */
static void emac_dma_control(
        struct eth_device *dev,
        int rx_not_tx,
        int enable)
{
    struct emac_priv *priv = dev->priv;
    u32 mask;

    /* Update the mirror */
    if (rx_not_tx) {
        mask = EMAC_START_RX;
    }
    else {
        mask = EMAC_START_TX;
    }

    if (enable) {
        priv->op_mode |= mask;
    }
    else {
        priv->op_mode &= ~mask;
    }

    /* Write the mirror to the hardware */
    EMAC_WRITE(priv->op_mode, EMAC_DMA_MODE_REG_OFFSET);
}

/* Make sure the EMAC DMA transmission is not suspended */
static void emac_hw_tx_resume(struct eth_device *dev)
{
    u32 reg;

    reg = EMAC_READ(EMAC_DMA_STATUS_REG_OFFSET);
    if ((reg & EMAC_DMA_TX_STATE_MASK) == EMAC_DMA_TX_SUSPENDED) {
        EMAC_WRITE(0, EMAC_DMA_TX_DEMAND_REG_OFFSET);
    }
}


/* Make sure the EMAC DMA reception is not suspended */
static void emac_hw_rx_resume(struct eth_device *dev)
{
    u32 reg;

    reg = EMAC_READ(EMAC_DMA_STATUS_REG_OFFSET);
    if ((reg & EMAC_DMA_RX_STATE_MASK) == EMAC_DMA_RX_SUSPENDED) {
        EMAC_WRITE(0, EMAC_DMA_RX_DEMAND_REG_OFFSET);
    }
}


/* There are multiple MAC Address register pairs on some controllers
 * This function sets the numth pair to a given address
 */
void emac_set_mac_for_addr(u8 *addr)
{
    EMAC_WRITE((addr[5]<<8) | addr[4],
            EMAC_MAC_ADDR_N_HIGH_REG_OFFSET(0));
    EMAC_WRITE((addr[3]<<24) | (addr[2]<<16) | (addr[1]<<8) | addr[0],
            EMAC_MAC_ADDR_N_LOW_REG_OFFSET(0));
}


/* Record the receive errors given a descriptor. If there are no
 * errors, return 0.
 */
static int emac_rx_errors(struct eth_device *dev, struct emac_desc *desc)
{
    struct emac_priv *priv = dev->priv;
    struct eth_device_stats *stats = &priv->stats;

    /* Deal with errors in descriptors, and descriptors
     * that hold partial frames. We only support a frame that fits
     * into one descriptor.
     */
    if (((desc->dma.status & EMAC_RX_REJECTION_MASK) != 0) ||
            ((desc->dma.status & EMAC_RX_DESC_STATUS_FIRST_DESC) == 0) ||
            ((desc->dma.status & EMAC_RX_DESC_STATUS_LAST_DESC) == 0)) {

        /* Record the error packets */
        stats->rx_errors++;

        /* Record the length errors */
        if (desc->dma.status & EMAC_RX_DESC_STATUS_LEN_ERR) {
            stats->rx_length_errors++;
        }

        /* Record the overflow errors */
        else if (desc->dma.status & EMAC_RX_DESC_STATUS_OV_ERR) {
            stats->rx_over_errors++;
        }

        /* Record the CRC errors */
        else if (desc->dma.status & EMAC_RX_DESC_STATUS_CRC_ERR) {
            stats->rx_crc_errors++;
        }

        /* Otherwise record a frame error */
        else if (desc->dma.status & EMAC_DESC_STATUS_ERR_SUM) {
            stats->rx_frame_errors++;
        }

        return 1;
    }

    return 0;
}


/* Initialises the EMAC and EMAC DMA registers and starts transmission and
 * reception. Does not allocate buffers.
 */
static void emac_start(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    u32 reg;
    unsigned int skip;

    DB((LVL_TRACE, "emac_start\n"));

    /* Set Host bus access parameters */
    skip = (sizeof(struct emac_aligned_desc) -
            sizeof(struct emac_dma_desc)) / EMAC_DMA_BUS_WIDTH;
    EMAC_WRITE(
            EMAC_BURST_LENGTH_8 |
            ((skip << EMAC_DMA_SKIP_SHIFT) & EMAC_DMA_SKIP_MASK),
            EMAC_DMA_BUS_MODE_REG_OFFSET);

    /* Set the descriptor ring start addresses */
    EMAC_WRITE(priv->rx_ring->dma_ring_start, EMAC_DMA_RX_LIST_REG_OFFSET);
    EMAC_WRITE(priv->tx_ring->dma_ring_start, EMAC_DMA_TX_LIST_REG_OFFSET);

    /* Set the MAC filter register up */
    emac_set_mac_for_addr(dev->enetaddr);

    /* Set up the frame filter control */
    reg = 0;     /* Use defaults */
    EMAC_WRITE(reg, EMAC_MAC_FRAME_FLT_REG_OFFSET);

    /* Set the flow control reg */
    reg = 0;
    if (priv->duplex == PHY_DUPLEX_FULL) {
        reg |= (EMAC_RX_FLOW_ENABLE | EMAC_TX_FLOW_ENABLE);
    }
    EMAC_WRITE(reg, EMAC_MAC_FLOW_CONTROL_REG_OFFSET);

    /* Set the hardware up for duplex. Same settings for 10 and 100 speed */
    reg =
        EMAC_FRAME_BURST_ENABLE |
        EMAC_PORT_SELECT |
        EMAC_CHECKSUM_OFFLOAD |
        EMAC_TX_ENABLE |
        EMAC_RX_ENABLE;

    if (priv->duplex == PHY_DUPLEX_HALF) {
        reg |= EMAC_DIABLE_RECEIVE_OWN;
    }
    else {
        reg |= EMAC_DUPLEX_MODE;
    }

    /* Set the control register.
     * Starts the MAC reception and transmission
     */
    EMAC_WRITE(reg, EMAC_MAC_CONFIG_REG_OFFSET);

    /* Set the operation mode */
    priv->op_mode = 0;

    /* Start the DMA reception and transmission */
    emac_dma_control(dev, 1, 1);      /* Enable Rx */
    emac_dma_control(dev, 0, 1);      /* Enable Tx */

    DB((LVL_TRACE, "emac_start complete\n"));
}


/* Stop the receive and transmit rings. Stop interrupts. Reset the
 * EMAC DMA controller. Does not free any ring buffers.
 */
static void emac_reset(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    u32 reg;
    unsigned int stops;

    DB((LVL_TRACE, "emac_reset\n"));

    /* Stop the DMA hardware */
    emac_dma_control(dev, 1, 0);    /* Disable Rx */
    emac_dma_control(dev, 0, 0);    /* Disable Tx */

    /* Stop the MAC hardware */
    EMAC_WRITE(0, EMAC_MAC_CONFIG_REG_OFFSET);

    /* Busy wait for the hardware to stop */
    stops = EMAC_STOP_COUNT;
    do {
        /* Wait a while for the reset to happen */
        udelay(EMAC_STOP_US);

        reg = EMAC_READ(EMAC_DMA_STATUS_REG_OFFSET);
        if ((reg & (EMAC_DMA_TX_STATE_MASK | EMAC_DMA_RX_STATE_MASK)) == 0) {
            break;
        }
    } while(--stops > 0);

    if (stops == 0) {
        /* Timeout waiting for stop */
        printf("emac: Can't stop EMAC DMA\n");
    }

    /* Set the reset bit to ensure the EMAC DMA is not running */
    EMAC_WRITE(EMAC_SOFT_RESET, EMAC_DMA_BUS_MODE_REG_OFFSET);

    /* Wait a while for the reset to happen */
    udelay(EMAC_RESET_US);

    /* Check that the reset has happened */
    reg = EMAC_READ(EMAC_DMA_BUS_MODE_REG_OFFSET);
    if ((reg & EMAC_SOFT_RESET) != 0) {
        /* Reset has not occurred. Error */
        printf("emac: Can't reset EMAC DMA\n");
    }

    /* Reset the receive rings state */
    emac_reset_ring_state(priv->rx_ring);

    DB((LVL_TRACE, "emac_reset complete\n"));
}


/* Called every time the controller might need to be made
 * aware of new link state.  The PHY code conveys this
 * information through variables in the phydev structure, and this
 * function converts those variables into the appropriate
 * register values, and can bring down the device if needed.
 */
static void emac_adjust_link(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    int new_state = 0;

    if (priv->link) {

        /* Now we make sure that we can be in full duplex mode.
         * If not, we operate in half-duplex mode. */
        if (priv->duplex != priv->oldduplex) {
            new_state = 1;
            priv->oldduplex = priv->duplex;
        }

        if (priv->speed != priv->oldspeed) {
            new_state = 1;
            priv->oldspeed = priv->speed;
        }

        if (!priv->oldlink) {
            new_state = 1;
            priv->oldlink = 1;
        }

        /* Something changed. Stop everything, set the new mode and
         * restart everything again once more.
         */
        if (new_state) {

            emac_reset(dev);
            emac_start(dev);
        }
    }
    else if (priv->oldlink) {
        new_state = 1;
        priv->oldlink = 0;
        priv->oldspeed = 0;
        priv->oldduplex = -1;

        emac_reset(dev);
    }
}


/* Initializes driver's PHY state, and attaches to the PHY.
 * Returns 0 on success.
 */
static int emac_init_phy(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    int speed_100, speed_1000;
    int timebase = 0;

    priv->oldlink = 0;
    priv->oldspeed = 0;
    priv->oldduplex = -1;

    /* Check on phy auto-negotiation progress */
    if ((emac_mii_read(CONFIG_PHY_ADDR, PHY_STS) & AUTONEG_CAPABLE) ==
            AUTONEG_CAPABLE) {

        /* The phy is auto-negotiation capable */

        timebase = get_timer (0);
        do {
            if ((emac_mii_read(CONFIG_PHY_ADDR, PHY_STS) & AUTOCMPLT) ==
                    AUTOCMPLT) {
                printf("Ethernet PHY auto-negotiation complete\n" );
                break;
            }
        }
        while (get_timer (timebase) < EMAC_PHY_TIMEOUT);
    }

    if ((emac_mii_read(CONFIG_PHY_ADDR, PHY_STS) & LINK) == 0) {
        printf("Ethernet link: is down !\n");
        priv->link = 0;
        return 1;
    }
    else {
        printf("Ethernet link: is up\n");
        priv->link = 1;
    }

    priv->duplex = (emac_mii_read(CONFIG_PHY_ADDR, PHY_CTL) & DUPLEX) != 0;
    printf("Ethernet link: %s-duplex mode\n", priv->duplex ? "Full" : "Half");

    speed_100 = (emac_mii_read(CONFIG_PHY_ADDR, PHY_CTL) & SPEED_100) != 0;
    speed_1000 = (emac_mii_read(CONFIG_PHY_ADDR, PHY_CTL) & SPEED_1000) != 0;
    printf("Ethernet link: 10%s Mbit/s\n", speed_1000 ? "00" : speed_100? "0" : "");

    if (speed_1000) {
        priv->speed = 1000;
    }
    else if (speed_100) {
        priv->speed = 100;
    }
    else {
        priv->speed = 10;
    }

    emac_adjust_link(dev);

    return 0;
}

/*
 * Allocate a descriptor ring
 */
static struct emac_ring *emac_allocate_ring(
        struct eth_device *dev,
        unsigned int length,
        unsigned int ring_num)
{
    struct emac_ring *ring = NULL;
    struct emac_desc *desc;
    int i;

    /* First allocate the ring structure itself (use static data) */
    ring = &rings[ring_num];

    ring->desc = (struct emac_aligned_desc *)&descriptor_buf[ring_num][0];
    /* Align on word boundary */
    ring->desc =
        (struct emac_aligned_desc *)(((unsigned int)ring->desc + 3) & 0xfffffffc);

    /* DMA address same as descriptor address */
    ring->dma_ring_start = (dma_addr_t)ring->desc;

    /* Zero the DMA descriptors */
    memset(ring->desc, 0, sizeof(struct emac_aligned_desc) * length);

    /* Setup the ring structure */
    ring->length = length;
    emac_reset_ring_state(ring);
    ring->dev = dev;

    /* Setup all the descriptors */
    for (i = 0; i < length; ++i) {

        /* pointer to the descriptor */
        desc = (struct emac_desc *)&ring->desc[i];

        /* Setup the descriptor */
        desc->dev = dev;
        desc->ring = ring;

        /* Mark it as owned by the HOST so the hardware will not use it */
        desc->dma.status &= ~EMAC_DESC_STATUS_OWNER;
    }

    return ring;
}


/*
 * Setup the descriptor fields to point to the data buffer.
 * If this function fails, it descriptor is not changed
 */
static int emac_wireup_desc_dma(
        struct eth_device *dev,
        struct emac_desc *desc,
        volatile void *ptr,
        unsigned int length)
{
    dma_addr_t dma_addr;
    int ret = 0;
    u32 buf1_length = 0, buf2_length = 0;

    /* Limit the length to the maximum. When we allocate a socket
     * buffer of this length, a larger buffer is allocated and we
     * cannot handle it.
     */
    if (length > (EMAC_MAX_DMA_LENGTH * 2)) {
        length = (EMAC_MAX_DMA_LENGTH * 2);
    }

    /* Create a DMA mapping for the whole data buffer */
    dma_addr = (dma_addr_t)ptr;

    /* Record the direction and length for unmapping later */
    desc->length = length;

    /* Decrement the space left on the ring if we are not
     * replacing an existing buffer
     */
    if (desc->dma.buffer1 == 0) {
        desc->ring->space--;
    }

    /* Set the first DMA buffer pointer to the start of the buffer */
    desc->dma.buffer1 = (u32)dma_addr;

    /* If the length is less that the DMA length, use the first buffer only,
     * otherwise use both buffers.
     */
    if (length <= EMAC_MAX_DMA_LENGTH) {

        buf1_length = length;
    }
    else {

        /* Set the maximum size for the first buffer */
        buf1_length = EMAC_MAX_DMA_LENGTH;

        /* Set the length of the second buffer */
        buf2_length = length - EMAC_MAX_DMA_LENGTH;

        /* Set the second DMA pointer, part way through the mapped area */
        desc->dma.buffer2 = (u32)dma_addr + EMAC_MAX_DMA_LENGTH;
    }

    /* Modify the control field */
    buf1_length =
        (buf1_length << EMAC_DESC_CNTL_TBS1_SHIFT) & EMAC_DESC_CNTL_TBS1_MASK;
    buf2_length =
        (buf2_length << EMAC_DESC_CNTL_TBS2_SHIFT) & EMAC_DESC_CNTL_TBS2_MASK;
    desc->dma.control &= ~EMAC_DESC_CNTL_TBS1_MASK;
    desc->dma.control &= ~EMAC_DESC_CNTL_TBS2_MASK;
    desc->dma.control = desc->dma.control | buf1_length | buf2_length;

    return ret;
}

/*
 * Get the status of a packet given the first descriptor that is wired up to
 * it. This function looks at the ownership of the first and last descriptors
 * and decides if the packet is owned by the host, hardware or shared between
 * both.
 */
#define EMAC_DMA_OWNED 0
#define EMAC_HOST_OWNED 1
#define EMAC_JOINTLY_OWNED 2
static int emac_get_next_dma_packet_status(struct emac_ring *ring)
{
    struct emac_desc *head_desc = ring->next_dma;

    /* The next_dma descriptor must be the head of a packet */
    if (head_desc->skb == NULL) {
        DB((LVL_ERR, "emac: Internal Ring ERROR\n"));
        return 0;
    }

    if ((head_desc->dma.status & EMAC_DESC_STATUS_OWNER) != 0) {
        return EMAC_DMA_OWNED;
    }

    return EMAC_HOST_OWNED;
}



void emac_release(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;

    /* Force an update to the link status immediately, do not wait for the PHY
     * driver code to do it's poll. This saves time and is less complex
     * as we do not have to implement a wait. If the PHY driver happens to
     * trigger the link down around the same time as we do this, a second
     * link down event will be swallowed.
     */
    priv->link = 0;
    emac_adjust_link(dev);

    /* Free the descriptor rings */
    priv->rx_ring = NULL;
    priv->tx_ring = NULL;
}


/*
 * Open and close
 */

static int emac_open(struct eth_device *dev, bd_t * bis)
{
    struct emac_priv *priv = dev->priv;
    struct sk_buff *skb;
    int ret = 1;
    int i;
    struct emac_desc *desc = NULL;

    /* Allocate receive descriptor ring */
    priv->rx_ring = emac_allocate_ring(dev, EMAC_RX_RING_LENGTH, 0);
    if (priv->rx_ring == NULL) {
        ret = 0;
        DB((LVL_ERR, "Failed to allocate RX ring\n"));
        goto out;
    }

    /* Allocate transmit descriptor ring */
    priv->tx_ring = emac_allocate_ring(dev, EMAC_TX_RING_LENGTH, 1);
    if (priv->tx_ring == NULL) {
        ret = 0;
        DB((LVL_ERR, "Failed to allocate TX ring\n"));
        goto out;
    }

    /* Allocate receive socket buffers and setup receive descriptors */
    for (i = 0; i < EMAC_RX_RING_LENGTH; ++i) {

        /* Handy pointer to the descriptor */
        desc = (struct emac_desc *)&priv->rx_ring->desc[i];

        /* Setup the receive control */
        desc->dma.control = EMAC_RX_CONTROL_FIELD_INIT;

        skb = (void *)&rx_skb_buf[i][0];

        /* Connect the socket buffer to the descriptor */
        emac_wireup_desc_dma(dev, desc, skb, RX_SKB_BUFLEN);

        /* Associate the descriptor with the socket buffer */
        desc->skb = skb;

        /* Mark it as owned by the DMA so the hardware will fill it */
        desc->dma.status |= EMAC_DESC_STATUS_OWNER;
    }

    /* Mark the last receive descriptor as the end of the ring */
    desc->dma.control |= EMAC_DESC_CNTL_END_RING;

    /* Setup transmit descriptors */
    for (i = 0; i < EMAC_TX_RING_LENGTH; ++i) {

        /* Handy pointer to the descriptor */
        desc = (struct emac_desc *)&priv->tx_ring->desc[i];

        desc->dma.control = EMAC_TX_CONTROL_FIELD_INIT;
    }

    /* Mark the last transmit descriptor as the end of the ring */
    desc->dma.control |= EMAC_DESC_CNTL_END_RING;

    /* Configure the PHY and start link status monitoring */
    ret = emac_init_phy(dev);
    if (ret != 0) {
        DB((LVL_ERR, "Failed to initialise PHY driver\n"));
        goto out;
    }

    return 0;
out:
    if (ret != 0) {
        emac_release(dev);
    }
    return ret;
}


/* Pass a new received socket buffer to the networking layer */
static int emac_pass_up_skb(
        struct eth_device *dev,
        volatile void *skb,
        int data_length)
{
#ifdef ENABLE_DEBUGGING
    //print_ping_seq(skb);
#endif

    /* Pass the buffer to the client */
    NetReceive(skb, data_length);

    return 0;
}

/* Processing of status bits. This does a similar job to the interrupt handler
 * except it is polled.
 */
static void emac_status_processing(struct eth_device *dev)
{
    u32 statusword;

    /* Retrieve the interrupt status */
    statusword = EMAC_READ(EMAC_DMA_STATUS_REG_OFFSET);
    /* Clear all interrupt status bits we will service */
    EMAC_WRITE(statusword, EMAC_DMA_STATUS_REG_OFFSET);

    /* Handle TX/RX stopped interrupt by restarting */
    if (statusword & (EMAC_TX_STOPPED_INT | EMAC_RX_STOPPED_INT)) {
        DB((LVL_WARNING, "Rx or Tx stopped seen, Restarting\n"));
        emac_dma_control(dev, 1, 1);      /* Enable Rx */
        emac_dma_control(dev, 0, 1);      /* Enable Tx */
    }
}


/*
 * The poll implementation.
 */
static int emac_poll(struct eth_device *dev)
{
    struct emac_priv *priv = dev->priv;
    struct emac_desc *desc;
    struct emac_ring *ring = priv->rx_ring;
    volatile void *full_skb;
    unsigned int data_length = 0;

    DB((LVL_TRACE, "emac_poll\n"));

    /* Process status bits */
    emac_status_processing(dev);

    if (emac_get_next_dma_packet_status(ring) == EMAC_HOST_OWNED) {

        /* Handy pointer to this descriptor, and the socket buffer
         * that is full of received data
         */
        desc = ring->next_dma;
        full_skb = desc->skb;

        /* Move on to the next packet for future processing */
        emac_next_dma_desc(ring);

        /* Discard bad frames and count errors */
        if (emac_rx_errors(dev, desc) == 0) {

            /* Get the received length from the descriptor */
            data_length = (desc->dma.status & EMAC_RX_DESC_STATUS_FLEN_MASK)
                >> EMAC_RX_DESC_STATUS_FLEN_SHIFT;

            /* Pass the socket buffer that has received data to
             * the network layer
             */
            emac_pass_up_skb(dev, full_skb, data_length);

            /* Re-use the socket buffer that is already connected to the
             * descriptor. Set the ownership to DMA owned.
             */
            desc->dma.status |= EMAC_DESC_STATUS_OWNER;
        }
    }

    /* Kick the hardware to continue receiving if it is suspended as there
     * are now more descriptors available for the DMA to write into.
     */
    emac_hw_rx_resume(dev);

    return data_length;
}



/*
 * Transmit a packet (called by the kernel)
 */
int emac_tx(struct eth_device *dev, volatile void *skb, int length)
{
    struct emac_priv *priv = dev->priv;
    struct emac_desc *desc, *first_desc;
    struct emac_ring *ring = priv->tx_ring;
    int i = 0;

    DB((LVL_TRACE, "emac_tx\n"));

#ifdef ENABLE_DEBUGGING
    //print_ping_seq(skb);
#endif

    /* Add the socket buffer to the transmit ring. We assume that the
     * socket buffer is not larger that a page. This is a valid assumption
     * as we have a maximum MTU of 4k (a page) and the networking layer
     * should not be sending us frames larger than this.
     */

    /* Handy pointer to the first transmit descriptor to be added */
    first_desc = ring->next;
    desc = first_desc;

    /* Set the descriptor to point to the buffer  */
    emac_wireup_desc_dma(dev, desc, skb, length);

    /* Associate the first descriptor with the socket buffer head */
    desc->skb = skb;

    /* Move on to the next descriptor where the
     * following frame will be added
     */
    emac_next_desc(ring);

    /* Set the ownership so that the packet will be sent */
    first_desc->dma.status |= EMAC_DESC_STATUS_OWNER;

    /* Kick the EMAC DMA */
    emac_hw_tx_resume(dev);

    /* Wait for the send to complete */
    while (first_desc->dma.status & EMAC_DESC_STATUS_OWNER){
        if (i++ > TOUT_LOOP) {
            DB((LVL_WARNING, "emac: tx timeout\n"));
            return 0;
        }
        udelay(10); /* give the nic a chance to write to the register */

        /* Process status bits */
        emac_status_processing(dev);
    }

    return 1;
}

/*
 * The init function (sometimes called probe).
 */
void pc20x_eth_initialize(bd_t * bis)
{
    struct eth_device *nic = NULL;
    struct emac_priv *priv = NULL;

    nic = (struct eth_device *) malloc(sizeof (*nic));
    priv = (struct emac_priv *) malloc(sizeof (*priv));
    nic->priv = priv;

    sprintf(nic->name, "pc20x_emac");

    /* Reset the private data */
    memset(priv, 0, sizeof(struct emac_priv));

    /* Assign the hardware address of the interface */
    memcpy(nic->enetaddr, bis->bi_enetaddr, ETH_LENGTH_OF_ADDRESS);

    nic->init = emac_open;
    nic->recv = emac_poll;
    nic->send = emac_tx;
    nic->halt = emac_release;

    eth_register(nic);
}
