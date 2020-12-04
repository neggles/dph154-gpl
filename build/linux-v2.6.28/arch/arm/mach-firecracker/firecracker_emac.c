/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * linux/arch/arm/mach-firecracker/firecracker_emac.c
 *
 * Copyright (c) 2006 picoChip Designs Ltd.
 *
 * Description:
 *
 * This module provides the network interface to the firecracker EMAC
 * hardware.
 *
 * References:
 *
 * Synopsys DesignWare Ethernet Universal Databook Version 3.2
 * November 2005.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/* Includes ---------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <mach/io.h>
#include <asm/delay.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>

/* Required for hw timestamping */
#include <net/timestamping.h>

/* Required for DP82640 Ethernet Phy as used on the
 * PC7802 platform
 */
#include <mach/dp83640.h>

/* Constants --------------------------------------------------------------- */

/* Default fixed MAC address to use in this driver. The hardware is read
 * at initialisation and if no valid address is found there, this
 * one is used.
 */
static unsigned char default_mac_address[ETH_ALEN] =
{
    0x00, 0x15, 0xe1, 0x00, 0x00, 0x00
};

/* Used to store the io remapped base address of the EMAC block */
static void __iomem *mem_region = NULL;

/* Forward structure reference */
struct emac_dma_desc;

/* emac_desc structure holds management data used by the driver */
struct emac_desc {
    /* Pointer to the hardware descriptor associated with this software one */
    struct emac_dma_desc *hw_desc;

    /* Pointer to the device that uses this descriptor */
    struct net_device *dev;

    /* Pointer to the ring used by this descriptor */
    struct emac_ring *ring;

    /* Pointer to the associated socket buffer wired up to this
     * descriptor or NULL if it is not wired up to a socket buffer.
     */
    struct sk_buff *skb;

    /* Pointer to the associated socket buffer fragment wired up to this
     * descriptor or NULL if it is not wired up to a socket buffer fragment.
     */
    struct skb_frag_struct *frag;

    /* The direction of the mapped buffer associated with this descriptor.
     * Used when unmapping.
     */
    enum dma_data_direction dir;

    /* The length of the mapped buffer associated with this descriptor.
     * Used when unmapping.
     */
    unsigned int length;
};

/* emac_hw_desc mirrors the EMAC DMA descriptor. It must have the same
 * endian and packing as the hardware.
 * These are used for both receive and transmit descriptors.
 * A facility of the DMA hardware that allows a gap between descriptors
 * in memory is utilised for the additional pointer to management information
 * This structure must be a multiple of 4 bytes in size.
 */
struct emac_dma_desc {
    u32 status;
    u32 control;
    u32 buffer1;
    u32 buffer2;
    struct emac_desc *sw_desc;
};

/* emac_ring structure holds ring management and the address in
 * DMA able memory of the hardware descriptor ring.
 */
struct emac_ring {

    /* Pointer to the start of the descriptor ring in DMA capable memory */
    struct emac_dma_desc *hw_desc;

    /* Physical address of the above descriptor ring, for the hardware */
    dma_addr_t dma_ring_start;

    /* Pointer to the software descriptor ring that holds driver management
     * data. This ring mirrors the hardware one.
     */
    struct emac_desc *sw_desc;

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
    struct net_device *dev;

    /* Thread synchronisation for the ring */
    spinlock_t lock;

    /* The number of work items scheduled for the ring */
    atomic_t pending;

    /* A flag indicating if the ring is stopping */
    atomic_t stopping;

    /* A wait queue for waiting for pending ring work to complete */
    wait_queue_head_t pending_waitq;

    /* Pointer to the struct device for mapping operations */
    struct device *device;
};

/* This structure is private to each device */
struct emac_priv {

    /* Statistics counters */
    struct net_device_stats stats;

    /* Transmit and Receive descriptor rings */
    struct emac_ring* tx_ring;
    struct emac_ring* rx_ring;

    /* Declare a work function for two level transmit interrupt processing.
     * We get two level receive interrupt processing 'for free' as we use the
     * interrupt mitigating NAPI interface. This suite our hardware as
     * interrupts are not queued.
     */
    struct workqueue_struct *tx_int_workqueue;
    struct work_struct tx_int_work;

    /* Declare a work function that is triggered by the PHY driver's callback
     * to update the link status. This will be responsible for starting and
     * stopping the hardware.
     */
    struct workqueue_struct *link_adjust_workqueue;
    struct work_struct link_adjust_work;

    /* PHY device and state */
    struct phy_device *phydev;
    int speed;
    int duplex;
    int link;
    int oldspeed;
    int oldduplex;
    int oldlink;

    /* Used to address a particular
     * Phy on the mdio bus
     */
    int phy_address;

    /* Levels of speed, duplex and link */
#define PHY_SPEED_10    10
#define PHY_SPEED_100   100
#define PHY_DUPLEX_HALF 0
#define PHY_DUPLEX_FULL 1
#define PHY_LINK_DOWN   0
#define PHY_LINK_UP     1

    /* Required field, controls messages */
    u32 msg_enable;

    /* Pointer to the struct device for mapping operations */
    struct device *device;
    struct net_device *self;

    /* Mirrored device registers */
    u32 op_mode;
    u32 ie_mask;
    u32 hash_low;
    u32 hash_high;

    /* Thread synchronisation for the above register mirrors */
    spinlock_t mirror_lock;

    /* For VLAN */
    struct vlan_group *vlgrp;

    struct napi_struct napi;

    struct platform_device  *pdev;

    /* Used to indicate whether hw timestamping
       of received packets is required or not */
    unsigned int hw_timestamp_flag;

    /* Used for sysfs access to the Ethernet Phy registers */
    unsigned int phy_register;
};

/* Globals ----------------------------------------------------------------- */
/*!
 * \brief Keep a local pointer to the netdev structure.
 *        Note: used for sysfs functions only.
 */
static struct net_device *sysfs_ndev;

/* Macros ------------------------------------------------------------------ */

/* Used to indicate successful function return value */
#define SUCCESS (0)

/* GET_MAC_FROM_EMAC enables code that reads the interface MAC address
 * from the EMAC (first filter) at initialisation.
 */
#define GET_MAC_FROM_EMAC

/* The interrupt mask for normal transmit interrupts */
#define EMAC_NORM_TX_INT_MASK                       \
        (EMAC_TX_INT)

/* The interrupt mask for normal receive interrupts */
#define EMAC_NORM_RX_INT_MASK                       \
        (EMAC_RX_INT)

/* The interrupt mask for abnormal interrupts */
#define EMAC_ABNORM_INT_MASK                        \
        (EMAC_FATAL_BUS_ERROR_INT |                 \
        EMAC_TX_UNDERFLOW_INT |                     \
        EMAC_RX_OVERFLOW_INT |                      \
        EMAC_TX_JABBER_TIMEOUT_INT |                \
        EMAC_TX_STOPPED_INT |                       \
        EMAC_RX_STOPPED_INT |                       \
        EMAC_RX_UNAVAILABLE_INT)

/* The interrupt mask for summery interrupts */
#define EMAC_SUMMERY_INT_MASK                       \
        (EMAC_NORMAL_SUMM_INT |                     \
        EMAC_ABNORMAL_SUMM_INT)

/* Stop and reset timeouts. */
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

/* EMAC_CHECKSUM_LENGTH defines the number of bytes added to the received
 * frame when hardware checksumming is enabled (IPC).
 */
#define EMAC_CHECKSUM_LENGTH                (2)

/* The number of bytes at the end of the frame for the ethernet frame check
 * sequence (CRC). Defined by ethernet.
 */
#define ETHERNET_FCS_LEN                    (4)

/* MII_WAIT_TIMEOUT - defines the number of jiffies to wait for the MII
 * busy flag to be reset by the hardware.
 */
#define MII_WAIT_TIMEOUT 10

/* Various names */
#define CARDNAME "pc20x-emac"
#define TITLE "pc20x EMAC"
#define VERSION "0.06"

/* Timeout for waiting for a ring to stop */
#define EMAC_RING_STOP_WAIT_TIMEOUT 2000

/* EMAC_DMA_BUS_WIDTH defines the bus width of the EMAC DMA */
#define EMAC_DMA_BUS_WIDTH_SHIFT    (2)
#define EMAC_DMA_BUS_WIDTH          (1 << EMAC_DMA_BUS_WIDTH_SHIFT) /* 4 */
#define EMAC_DMA_BUS_WIDTH_MASK     (EMAC_DMA_BUS_WIDTH - 1)        /* b'0011 */

/* The maximum size of a single DMA transfer, allowing for size alignment */
#define EMAC_MAX_DMA_LENGTH         (2032)

/* The maximum packet size that can be handled by this driver */
#define EMAC_MAX_PACKET_LENGTH      (EMAC_MAX_DMA_LENGTH * 2)

/* Size of receive buffers */
#define RX_BUFFER_SIZE		(2048)

/* The length mask to be applied to the receive descriptors */
#define EMAC_RX_LENGTH_MASK         (0xf)
#define EMAC_RX_LENGTH_ALLIGN       (16)

/* The length of a DMA descriptor used to calculate the descriptor skip */
#define EMAC_DMA_DESCRIPTOR_LENGTH  (16)

/* EMAC_TIMEOUT is set in the net device structure setting the timeout
 * for transmission.
 */
#define EMAC_TIMEOUT 1000

/* Default receive and transmit ring lengths */
#define EMAC_RX_RING_LENGTH 100
#define EMAC_TX_RING_LENGTH 100

/* The poll quota (must be less than the ring length) */
#define EMAC_NAPI_POLL_WEIGHT 64

/* CSR_CLOCK_RANGE defines the clock range setting to use */
#define EMAC_CSR_CLOCK_RANGE EMAC_GMII_CSR_RANGE_100_150

/* Support MII PHY abilities */
#define EMAC_MII_SUPPORTED \
    (SUPPORTED_10baseT_Half \
     | SUPPORTED_10baseT_Full \
     | SUPPORTED_100baseT_Half \
     | SUPPORTED_100baseT_Full \
     | SUPPORTED_Autoneg \
     | SUPPORTED_MII)

/* Various configurable parameters */
static int watchdog = EMAC_TIMEOUT;
module_param(watchdog, int, 0400);
MODULE_PARM_DESC(watchdog, "transmit timeout in milliseconds");

static int rx_ring_length = EMAC_RX_RING_LENGTH;
module_param(rx_ring_length, int, 0);
MODULE_PARM_DESC(rx_ring_length, "Receive Descriptor Ring Length");

static int tx_ring_length = EMAC_TX_RING_LENGTH;
module_param(tx_ring_length, int, 0);
MODULE_PARM_DESC(tx_ring_length, "Transmit Descriptor Ring Length");

static int napi_poll_weight = EMAC_NAPI_POLL_WEIGHT;
module_param(napi_poll_weight, int, 0);
MODULE_PARM_DESC(napi_poll_weight, "Receive NAPI Poll Weight");

/* Debugging levels: */
#define LVL_FATAL       2   /* fatal error conditions */
#define LVL_ERR         3   /* error conditions */
#define LVL_WARNING     4   /* warning conditions */
#define LVL_NOTICE      5   /* normal but significant condition */
#define LVL_INFO        6   /* informational */
#define LVL_DEBUG       7   /* debug-level messages */
#define LVL_TRACE       8   /* trace messages */
#define LVL_TRACE_IO    9   /* Register IO trace messages */

#if CONFIG_FIRECRACKER_EMAC_DEBUG > 0

/* Define ENABLE_DEBUGGING to include code for debugging via printk.
 * The value of ENABLE_DEBUGGING specifies the default debug level.
 */
#define ENABLE_DEBUGGING CONFIG_FIRECRACKER_EMAC_DEBUG

#else

#undef ENABLE_DEBUGGING

#endif

/* Macros for register read/write. These hide the virtual addressing.
 */
#ifndef ENABLE_DEBUGGING

#define EMAC_READ(__offset) \
    ioread32(__io(IO_ADDRESS(PC20X_EMAC_BASE + __offset)))

#define EMAC_WRITE(__value, __offset) \
    iowrite32(__value, __io(IO_ADDRESS(PC20X_EMAC_BASE + __offset)))

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

        if (printk_ratelimit() || lvl < LVL_TRACE_IO) {

            vsnprintf(buf, MAX_FORMAT_SIZE, fmt, args);
            printk("emac <%c>: %s", lvl_ch[lvl], buf);
        }
        else {
            printk("%c", lvl_ch[lvl]);
        }
    }
}

static void debug_iowrite32(u32 val, unsigned int paddr)
{
    DB((LVL_TRACE_IO, "iowrite32(0x%08x)=0x%08x\n", paddr, val));
    iowrite32(val, __io(IO_ADDRESS(paddr)));
}

static unsigned int debug_ioread32(unsigned int paddr)
{
    unsigned int val = ioread32(__io(IO_ADDRESS(paddr)));
    DB((LVL_TRACE_IO, "ioread32(0x%08x)=0x%08x\n", paddr, val));
    return val;
}

static void print_ring(struct emac_ring *ring, char *prefix, u32 hw_next, u32 sw_next_idx)
{
    int i;
    char str_ow[200];
    char str_po[200];
    static volatile u32 hw_next_off;
    static volatile int hw_next_idx;

    hw_next_off = hw_next - ring->dma_ring_start;
    hw_next_idx = hw_next_off / sizeof(struct emac_dma_desc);

    for (i = 0; i < ring->length; ++i) {
        if (i == hw_next_idx) {
            if (i == sw_next_idx) {
                str_po[i] = 'X';
            }
            else {
                str_po[i] = '^';
            }
        }
        else if (i == sw_next_idx) {
            str_po[i] = 'v';
        }
        else {
            str_po[i] = ' ';
        }

        if (ring->hw_desc[i].buffer1 == 0)
        {
            if (ring->hw_desc[i].status & EMAC_DESC_STATUS_OWNER) {
                str_ow[i] = 'd';
            }
            else {
                str_ow[i] = 'h';
            }
        }
        else {
            if (ring->hw_desc[i].status & EMAC_DESC_STATUS_OWNER) {
                str_ow[i] = 'D';
            }
            else {
                str_ow[i] = 'H';
            }
        }
    }
    str_ow[i] = 0;
    str_po[i] = 0;

    DB((LVL_DEBUG, "%s OW: %s\n", prefix, str_ow));
    DB((LVL_DEBUG, "%s PO: %s\n", prefix, str_po));
}

static void print_rings(struct net_device *dev, int now)
{
    struct emac_priv *priv = netdev_priv(dev);
    static int rate_limit = 0;
    u32 hw_next_tx = /*EMAC_READ(EMAC_DMA_CURR_TX_DESC_REG_OFFSET);*/
        ioread32(__io(IO_ADDRESS(PC20X_EMAC_BASE + EMAC_DMA_CURR_TX_DESC_REG_OFFSET)));
    u32 hw_next_rx = /*EMAC_READ(EMAC_DMA_CURR_RX_DESC_REG_OFFSET);*/
        ioread32(__io(IO_ADDRESS(PC20X_EMAC_BASE + EMAC_DMA_CURR_RX_DESC_REG_OFFSET)));

    if (debug_lvl >= LVL_TRACE) {
        now = 1;
    }

    if (((++rate_limit % 8) == 0) || now) {
        print_ring(priv->tx_ring, "TX", hw_next_tx, priv->tx_ring->next_idx);
        print_ring(priv->rx_ring, "RX", hw_next_rx, priv->rx_ring->next_dma_idx);
    }
}

/* Read all the PHY registers and print them out */
static int emac_mii_read(struct mii_bus *bus, int mii_id, int regnum);
static void emac_dump_phy_regs(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);
    int i;
    int reg[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 16, 17, 18, 19, 20, 30, -1
    };
    int val;

    for (i = 0; reg[i] != -1; i++) {
        val = emac_mii_read(mii_bus, priv->phy_address, reg[i]);

        DB((LVL_DEBUG, "PHY reg[%u] = 0x%04x\n", reg[i], val));
    }
}

#endif /* ENABLE_DEBUGGING */

/* Prototypes--------------------------------------------------------------- */

static u32
emac_ioread32 (unsigned int register_offset);

static void
emac_iowrite32 (u32 value,
                unsigned int register_offset);

static struct emac_desc
*emac_next_desc(struct emac_ring *ring);

static struct emac_desc
*emac_next_dma_desc(struct emac_ring *ring);

static struct emac_desc
*emac_previous_desc(struct emac_ring *ring);

static void
emac_reset_ring_state(struct emac_ring *ring);

static void
emac_set_ring_ownership(struct emac_ring *ring,
                        int host_not_dma);

static int
emac_phy_busy_wait(void);

static int
emac_mii_read(struct mii_bus *bus,
              int mii_id,
              int regnum);

static int
emac_mii_write(struct mii_bus *bus,
               int mii_id,
               int regnum,
               u16 value);

static void
emac_interrupt_control(struct net_device *dev,
                       int rx_not_tx,
                       int enable);

static void
emac_dma_control(struct net_device *dev,
                 int rx_not_tx,
                 int enable);

static void
emac_hw_tx_resume(struct net_device *dev);

static void
emac_hw_rx_resume(struct net_device *dev);

static void
emac_set_mac_for_addr(struct net_device *dev,
                      int num, u8 *addr,
                      int source_flt);

static void
emac_get_mac(struct net_device *dev,
             u8 *addr);

static void
emac_reset_mac(struct net_device *dev,
               int num);

static void
emac_calc_hash_for_addr(struct net_device *dev,
                        u8 *addr);

static void
emac_set_hash(struct net_device *dev);

static int
emac_rx_errors(struct net_device *dev,
               struct emac_dma_desc *desc);

static int
emac_tx_errors(struct net_device *dev,
               struct emac_dma_desc *desc);

static void
emac_unmap_skb_frag(struct net_device *dev,
                    struct emac_desc *desc);

static void
emac_free_skb_frag(struct net_device *dev,
                   struct emac_desc *desc);

static void
emac_free_ring_skb(struct net_device *dev,
                   struct emac_ring *ring);

static struct sk_buff
*emac_allocate_skb(struct net_device *dev);

static void
emac_wait_stop_ring(struct emac_ring *ring);

static void
emac_start(struct net_device *dev);

static void
emac_reset(struct net_device *dev);

static void
emac_adjust_link_trigger(struct net_device *dev);

static void
emac_adjust_link(struct work_struct *param);

static int
emac_init_phy(struct net_device *dev);

static ssize_t
emac_sysfs_1588_seconds_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf);

static ssize_t
emac_sysfs_1588_seconds_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count);

static ssize_t
emac_sysfs_1588_nano_seconds_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

static ssize_t
emac_sysfs_1588_nano_seconds_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

static ssize_t
emac_sysfs_1588_timer_increment_show(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf);

static ssize_t
emac_sysfs_1588_timer_increment_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf,
                                      size_t count);

static ssize_t
emac_sysfs_phy_register_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf);

static ssize_t
emac_sysfs_phy_register_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count);

static ssize_t
emac_sysfs_phy_register_value_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf);

static ssize_t
emac_sysfs_phy_register_value_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count);

static int
emac_sysfs_add(struct platform_device *pdev);

static void
emac_sysfs_remove(struct platform_device *pdev);

static int
emac_init_dp83640_phy(struct net_device *dev);

static void
emac_dp83640_select_page(struct net_device *dev,
                         unsigned int page);

static int
emac_dp83640_set_1588_seconds(struct net_device *dev,
                              u32 timer_seconds);

static int
emac_dp83640_get_1588_seconds(struct net_device *dev,
                              u32 *timer_seconds);

static int
emac_dp83640_set_1588_nano_seconds(struct net_device *dev,
                                   u32 timer_nano_seconds);

static int
emac_dp83640_get_1588_nano_seconds(struct net_device *dev,
                                   u32 *timer_nano_seconds);

static int
emac_dp83640_set_1588_timer_increment(struct net_device *dev,
                                      u32 rate);

static int
emac_dp83640_get_1588_timer_increment(struct net_device *dev,
                                      u32 *rate);

static void
emac_free_ring(struct emac_ring *ring);

static struct emac_ring
*emac_allocate_ring(struct net_device *dev,
                    unsigned int length);

static int
emac_wireup_desc_dma(struct net_device *dev,
                     struct emac_desc *desc,
                     void *ptr,
                     unsigned int length,
                     enum dma_data_direction dir);

static int
emac_get_next_dma_packet_status(struct emac_ring *ring);

static void
emac_tx_interrupt_work(struct work_struct *param);

static irqreturn_t
emac_interrupt(int irq,
               void *dev_id);

int
emac_release(struct net_device *dev);

static int
emac_open(struct net_device *dev);

int
emac_config(struct net_device *dev,
            struct ifmap *map);

static int
emac_pass_up_skb(struct net_device *dev,
                 struct sk_buff *skb,
                 u32 status);

static int
emac_poll(struct napi_struct *napi_s,
          int budget);

int
emac_tx(struct sk_buff *skb,
        struct net_device *dev);

static void
emac_tx_timeout(struct net_device *dev);

int
emac_ioctl(struct net_device *dev,
           struct ifreq *rq,
           int cmd);

static int
emac_hwtstamp_ioctl(struct net_device *dev,
                    struct ifreq *ifr,
                    int cmd);

struct
net_device_stats *emac_stats(struct net_device *dev);

static void
emac_vlan_rx_register(struct net_device *dev,
                      struct vlan_group *grp);

static void
emac_vlan_rx_kill_vid(struct net_device *dev,
                      uint16_t vid);

static void
emac_set_multi(struct net_device *dev);

void
emac_init(struct net_device *dev,
          struct platform_device *pdev);

static int
emac_mii_probe(struct device *dev);

static int
emac_mii_remove(struct device *dev);

static int
emac_drv_probe(struct platform_device *pdev);

static int
emac_drv_remove(struct platform_device *pdev);

/* Functions --------------------------------------------------------------- */

/*
 *  Read an emac register
 */
static u32
emac_ioread32 (unsigned int register_offset)
{
    void __iomem *p = mem_region + register_offset;

    u32 value = ioread32(p);

    return value;
}

/*
 * Write to an emac register
 */
static void
emac_iowrite32 (u32 value,
                unsigned int register_offset)
{
    void __iomem *p = mem_region + register_offset;

    iowrite32(value, p);
}

/*
 * Move the next pointer in a ring on to the next sw descriptor
 */
static struct emac_desc
*emac_next_desc(struct emac_ring *ring)
{
    ring->next_idx++;

    if (ring->next_idx == ring->length) {
        ring->next_idx = 0;
    }

    ring->next = &ring->sw_desc[ring->next_idx];

    DB((LVL_TRACE, "Next idx %u\n", ring->next_idx));

    return ring->next;
}

/*
 * Move the next DMA pointer in a ring on to the next sw descriptor
 */
static struct emac_desc
*emac_next_dma_desc(struct emac_ring *ring)
{
    ring->next_dma_idx++;

    if (ring->next_dma_idx == ring->length) {
        ring->next_dma_idx = 0;
    }

    ring->next_dma = &ring->sw_desc[ring->next_dma_idx];

    DB((LVL_TRACE, "Next DMA idx %u\n", ring->next_dma_idx));

    return ring->next_dma;
}

/*
 * Move the next pointer in a ring back to the previous descriptor
 */
static struct emac_desc
*emac_previous_desc(struct emac_ring *ring)
{
    if (ring->next_idx == 0) {
        ring->next_idx = ring->length-1;
    }
    else {
        ring->next_idx--;
    }

    ring->next = &ring->sw_desc[ring->next_idx];

    return ring->next;
}

/* Reset the ring pointers state */
static void
emac_reset_ring_state(struct emac_ring *ring)
{
    ring->space = ring->length;
    ring->next_idx = 0;
    ring->next = ring->sw_desc;
    ring->next_dma_idx = 0;
    ring->next_dma = ring->sw_desc;
    atomic_set(&ring->pending, 0);
}

/* Set the ownership flag on all elements of a ring */
static void
emac_set_ring_ownership(struct emac_ring *ring,
                        int host_not_dma)
{
    int i;

    for (i = 0; i < ring->length; ++i) {
        if (host_not_dma) {
            ring->hw_desc[i].status &= ~EMAC_DESC_STATUS_OWNER;
        }
        else {
            ring->hw_desc[i].status |= EMAC_DESC_STATUS_OWNER;
        }
    }
}

/* Wait with timeout for the PHY busy signal to be reset */
static int
emac_phy_busy_wait(void)
{
    int res = 0;
    unsigned long timeout_jiffies;
    u32 addr_reg;

    timeout_jiffies = jiffies + MII_WAIT_TIMEOUT;
    while (1) {

        /* Check for time out */
        if (time_after(jiffies, timeout_jiffies)) {
        
            /* Check for the busy bit again before we leave.  If we were suspended
               by a higher priority process just before checking the time, then
               when control gets back here, the timer may have already expired, but
               the busy bit might have been cleared in the meantime.  The longer we
               are delayed, the more likely it is that we will return a timeout in
               error.  That then causes a reset of the ethernet interface. */
            addr_reg = EMAC_READ(EMAC_MAC_GMII_ADDR_REG_OFFSET);
            if ((addr_reg & EMAC_GMII_BUSY) == 0) {
                break;
            }
            
            DB((LVL_ERR, "Timed out waiting for MII\n"));
            res = -ETIMEDOUT;
            break;
        }

        /* Check for the busy bit */
        addr_reg = emac_ioread32(EMAC_MAC_GMII_ADDR_REG_OFFSET);
        if ((addr_reg & EMAC_GMII_BUSY) == 0) {
            break;
        }
    }

    return res;
}

/* Read the bus for PHY at addr mii_id, register regnum, and
 * return the value.  Clears miimcom first.  All PHY
 */
static int
emac_mii_read(struct mii_bus *bus,
              int mii_id,
              int regnum)
{
    int res = 0;
    unsigned int value = 0;

    /* The emac_phy_[read|write] functions ensure that their operation
     * has completed before they exit so we do not have to wait
     * for a previous operation to finish here and we can go ahead and
     * use the MII interface.
     */

    /* Setup the address register and start the read */
    emac_iowrite32((u32)
                   ((mii_id << EMAC_GMII_ADDRESS_SHIFT) |
                    (regnum << EMAC_GMII_GMII_REG_SHIFT) |
                    EMAC_CSR_CLOCK_RANGE |
                    EMAC_GMII_BUSY),
                   EMAC_MAC_GMII_ADDR_REG_OFFSET);

    /* Wait for the MII to finish */
    res = emac_phy_busy_wait();
    if (res != 0) {
        DB((LVL_ERR, "Timeout waiting for PHY\n"));
        goto out;
    }

    /* Read back the register contents */
    value = emac_ioread32(EMAC_MAC_GMII_DATA_REG_OFFSET);

out:
    return value;
}

/* Write value to the PHY at mii_id at register regnum,
 * on the bus, waiting until the write is done before returning.
 */
static int
emac_mii_write(struct mii_bus *bus,
               int mii_id,
               int regnum,
               u16 value)
{
    /* The emac_phy_[read|write] functions ensure that their operation
     * has completed before they exit so we do not have to wait
     * for a previous operation to finish here and we can go ahead and
     * use the MII interface.
     */

    /* Set the register contents to be written */
    emac_iowrite32((u32)value, EMAC_MAC_GMII_DATA_REG_OFFSET);

    /* Setup the address register and start the write */
    emac_iowrite32((u32)
                   ((mii_id << EMAC_GMII_ADDRESS_SHIFT) |
                    (regnum << EMAC_GMII_GMII_REG_SHIFT) |
                    EMAC_CSR_CLOCK_RANGE |
                    EMAC_GMII_WRITE |
                    EMAC_GMII_BUSY),
                   EMAC_MAC_GMII_ADDR_REG_OFFSET);

    /* Wait for the MII to finish */
    emac_phy_busy_wait();

    return 0;
}

/* Enable/disable, receive/transmit interrupts */
static void
emac_interrupt_control(struct net_device *dev,
                       int rx_not_tx,
                       int enable)
#define RX 1
#define TX 0
#define ENABLE 1
#define DISABLE 0
{
    struct emac_priv *priv = netdev_priv(dev);
    unsigned long flags;
    u32 mask;

    /* Thread synchronisation required */
    spin_lock_irqsave(&priv->mirror_lock, flags);

    /* Update the mirror */
    if (rx_not_tx) {
        mask = EMAC_NORM_RX_INT_MASK;
    }
    else {
        mask = EMAC_NORM_TX_INT_MASK;
    }

    if (enable) {
        priv->ie_mask |= mask;
    }
    else {
        priv->ie_mask &= ~mask;
    }

    /* Write the IE mirror to the hardware */
    emac_iowrite32(priv->ie_mask, EMAC_DMA_IE_REG_OFFSET);

    /* Release thread sync */
    spin_unlock_irqrestore(&priv->mirror_lock, flags);
}

/* Enable/disable, EMAC DMA */
static void
emac_dma_control(struct net_device *dev,
                 int rx_not_tx,
                 int enable)
{
    struct emac_priv *priv = netdev_priv(dev);
    unsigned long flags;
    u32 mask;

    /* Thread synchronisation required */
    spin_lock_irqsave(&priv->mirror_lock, flags);

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
    emac_iowrite32(priv->op_mode, EMAC_DMA_MODE_REG_OFFSET);

    /* Release thread sync */
    spin_unlock_irqrestore(&priv->mirror_lock, flags);
}

/* Make sure the EMAC DMA transmission is not suspended */
static void
emac_hw_tx_resume(struct net_device *dev)
{
    u32 reg;

    reg = emac_ioread32(EMAC_DMA_STATUS_REG_OFFSET);
    if ((reg & EMAC_DMA_TX_STATE_MASK) == EMAC_DMA_TX_SUSPENDED) {
        emac_iowrite32(0, EMAC_DMA_TX_DEMAND_REG_OFFSET);
    }
}

/* Make sure the EMAC DMA reception is not suspended */
static void
emac_hw_rx_resume(struct net_device *dev)
{
    u32 reg;

    reg = emac_ioread32(EMAC_DMA_STATUS_REG_OFFSET);
    if ((reg & EMAC_DMA_RX_STATE_MASK) == EMAC_DMA_RX_SUSPENDED) {
        emac_iowrite32(0, EMAC_DMA_RX_DEMAND_REG_OFFSET);
    }
}


/* There are multiple MAC Address register pairs on some controllers
 * This function sets the numth pair to a given address
 */
static void
emac_set_mac_for_addr(struct net_device *dev,
                      int num, u8 *addr,
                      int source_flt)
{
    u32 reg = (addr[5] << 8) | addr[4];

    /* Exact match zero is always enabled for destination */
    if (num > 0) {
        reg |= EMAC_MAC_ADDR_ENABLE;

        /* Use source filtering */
        if (source_flt) {
            reg |= EMAC_MAC_ADDR_SOURCE;
        }
    }

    emac_iowrite32(reg, EMAC_MAC_ADDR_N_HIGH_REG_OFFSET(num));

    emac_iowrite32((addr[3]<<24) | (addr[2]<<16) | (addr[1]<<8) | addr[0],
            EMAC_MAC_ADDR_N_LOW_REG_OFFSET(num));
}

/* Read the hardware for a MAC address. This would be used when we expect
 * the hardware to be setup by the bootloader.
 * This function reads the first MAC filter. We expect the interface address
 * to be set in the first MAC filter. If the MAC read is invalid, it uses
 * the default that is hardcoded in the driver.
 */
static void
emac_get_mac(struct net_device *dev,
             u8 *addr)
{
#ifdef GET_MAC_FROM_EMAC
    u32 reg;

    /* Read the high filter reg. This contains the 5th and 6th bytes */
    reg = emac_ioread32(EMAC_MAC_ADDR_N_HIGH_REG_OFFSET(0));

    addr[4] = reg & 0xff;
    reg >>= 8;
    addr[5] = reg & 0xff;

    /* Read the low filter reg. This contains the first to 4th bytes */
    reg = emac_ioread32(EMAC_MAC_ADDR_N_LOW_REG_OFFSET(0));

    addr[0] = reg & 0xff;
    reg >>= 8;
    addr[1] = reg & 0xff;
    reg >>= 8;
    addr[2] = reg & 0xff;
    reg >>= 8;
    addr[3] = reg & 0xff;

    if (!is_valid_ether_addr(addr)) {
        printk(KERN_INFO "%s: Invalid MAC Address found in EMAC filter\n",
               dev->name);
        goto set_default_mac;
    }
    return;

#endif  /* GET_MAC_FROM_EMAC */

set_default_mac:
    memcpy(addr, default_mac_address, ETH_ALEN);
    printk(KERN_INFO "%s: Using MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
           dev->name, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

static void
emac_reset_mac(struct net_device *dev,
               int num)
{
    if (num > 0) {
        emac_iowrite32(0, EMAC_MAC_ADDR_N_HIGH_REG_OFFSET(num));
        emac_iowrite32(0, EMAC_MAC_ADDR_N_LOW_REG_OFFSET(num));
    }
}

static void
emac_calc_hash_for_addr(struct net_device *dev,
                        u8 *addr)
{
    struct emac_priv *priv = netdev_priv(dev);
    int i, lsb, b;
    u32 crc = 0xFFFFFFFF;
    u32 poly = 0xEDB88320;
    int hash_bit;
    u8 byte;

    /* Munge the address bytes into a crc */
    for (i = 0; i < ETH_ALEN; i++) {

        /* Get the next byte of the address */
        byte = addr[i];

        /* Munge the byte into the crc */
        for (b = 0; b < 8; ++b) {

            lsb = (crc ^ byte) & 1;
            crc >>= 1;
            if (lsb != 0) crc ^= poly;
            byte >>= 1;
        }
    }

    /* Upper 6 bits is the bit number */
    hash_bit = (crc>>26) & 0x3F;

    /* Set the bit in the hash registers to allow the address through the
     * filtering.
     */
    if (hash_bit >= 32) {
        priv->hash_high |= (1 << (hash_bit - 32));
    }
    else {
        priv->hash_low |= (1 << hash_bit);
    }
}

static void
emac_set_hash(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    emac_iowrite32(priv->hash_high, EMAC_MAC_HASH_HIGH_REG_OFFSET);
    emac_iowrite32(priv->hash_low, EMAC_MAC_HASH_LOW_REG_OFFSET);
}

/* Record the receive errors given a descriptor. If there are no
 * errors, return 0.
 */
static int
emac_rx_errors(struct net_device *dev,
               struct emac_dma_desc *desc)
{
    u32 status = desc->status;

    /* Deal with errors in descriptors, and descriptors
     * that hold partial frames. We only support a frame that fits
     * into one descriptor.
     */
    if (((status & EMAC_RX_REJECTION_MASK) != 0) ||
            ((status & EMAC_RX_DESC_STATUS_FIRST_DESC) == 0) ||
            ((status & EMAC_RX_DESC_STATUS_LAST_DESC) == 0)) {
        return 1;
    }
    return 0;
}

/* Record the transmit errors given a descriptor. If there are no
 * errors, return 0.
 */
static int
emac_tx_errors(struct net_device *dev,
               struct emac_dma_desc *desc)
{
    u32 status = desc->status;

    if (status & EMAC_DESC_STATUS_ERR_SUM) {
        return 1;
    }
    return 0;
}

/* Unmaps the socket buffer or fragment attached to a descriptor */
static void
emac_unmap_skb_frag(struct net_device *dev,
                    struct emac_desc *desc)
{
    struct emac_priv *priv = netdev_priv(dev);
    u32 buffer1 = desc->hw_desc->buffer1;

    if (buffer1 != 0) {
        dma_unmap_single(priv->device, buffer1, desc->length, desc->dir);
    }
}

/*
 * Disconnect, unmap and free a socket buffer connected to a descriptor.
 */
static void
emac_free_skb_frag(struct net_device *dev,
                   struct emac_desc *desc)
{
    struct emac_dma_desc *hw_desc = desc->hw_desc;

    /* Mark the descriptor as owned by the host to prevent the hardware
     * writing into it when we have unmapped it (for safety).
     */
    hw_desc->status &= ~EMAC_DESC_STATUS_OWNER;

    /* Unmap the fragment or socket buffer */
    emac_unmap_skb_frag(dev, desc);

    /* If a socket buffer is connected, free it. This frees all the
     * socket buffer fragments also and must arrange for the descriptors
     * holding the fragments to be unmapped before we get to the head
     * socket buffer
     */
    if (desc->skb != NULL) {

        /* Free the socket buffer. This includes all fragments which should
         * have been unmapped by previous calls to this function.
         */
        dev_kfree_skb(desc->skb);
    }

    /* Make sure the socket buffer association is reset in the descriptor */
    desc->skb = NULL;
    desc->frag = NULL;

    /* If we freed something here, update the space left */
    if (hw_desc->buffer1 != 0) {

        /* Increment the space left on the ring */
        desc->ring->space++;
    }

    /* Mark as unused */
    hw_desc->buffer1 = 0;
}


static void
emac_free_ring_skb(struct net_device *dev,
                   struct emac_ring *ring)
{
    int i;

    /* Free any socket buffers we may have. Do this in the
     * reverse order to that which they were added so that we ensure
     * a socket buffer's fragments are unmapped before the socket
     * buffer itself is freed.
     */
    for (i = 0; i < ring->length; ++i) {

        /* Move back to the previous descriptor */
        emac_previous_desc(ring);

        /* Free the buffer associated with the descriptor */
        emac_free_skb_frag(dev, ring->next);
    }

    /* Reset the ring pointers */
    emac_reset_ring_state(ring);
}

/*
 * Allocate a socket buffer.
 */
static struct sk_buff
*emac_allocate_skb(struct net_device *dev)
{
    struct sk_buff *skb;

    /* Allocate the socket buffer. Always allocate maximum size */
    skb = dev_alloc_skb(RX_BUFFER_SIZE);
    if (skb == NULL) {
        DB((LVL_WARNING, "emac: dev_alloc_skb failed\n"));
        goto out;
    }

    /* Align IP on 16 Bit boundary */
    skb_reserve(skb, 2);

out:
    return skb;
}

/* Wait (block) for the ring handler thread to complete it's last scheduled
 * run.
 */
static void
emac_wait_stop_ring(struct emac_ring *ring)
{
    long ret;

    DB((LVL_TRACE, "emac_wait_stop_ring\n"));

    /* Set the stopping state */
    atomic_set(&ring->stopping, 1);

    DB((LVL_DEBUG, "wait pending count = %u\n", atomic_read(&ring->pending)));

    /* Wait for the pending count to become zero */
    ret = wait_event_interruptible_timeout(
            ring->pending_waitq,
            (atomic_read(&ring->pending) == 0),
            EMAC_RING_STOP_WAIT_TIMEOUT);

    if (ret == 0) {
        DB((LVL_ERR, "Timed out waiting for ring to stop\n"));
    }
    else if (ret == -ERESTARTSYS) {
        DB((LVL_ERR, "Signal received while waiting for ring to stop\n"));
    }

    /* Re-set the stopping state */
    atomic_set(&ring->stopping, 0);
}

/* Initialises the EMAC and EMAC DMA registers and starts transmission and
 * reception. Does not allocate buffers.
 */
static void
emac_start(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    u32 reg;
    unsigned int skip;

    DB((LVL_TRACE, "emac_start\n"));

    /* Set Host bus access parameters */
    skip = (sizeof(struct emac_dma_desc) -
            EMAC_DMA_DESCRIPTOR_LENGTH) / EMAC_DMA_BUS_WIDTH;
    emac_iowrite32(
            EMAC_BURST_LENGTH_8 |
            ((skip << EMAC_DMA_SKIP_SHIFT) & EMAC_DMA_SKIP_MASK),
            EMAC_DMA_BUS_MODE_REG_OFFSET);

    /* Set the descriptor ring start addresses */
    emac_iowrite32(priv->rx_ring->dma_ring_start, EMAC_DMA_RX_LIST_REG_OFFSET);
    emac_iowrite32(priv->tx_ring->dma_ring_start, EMAC_DMA_TX_LIST_REG_OFFSET);

    /* Set the MAC filter register up */
    emac_set_mac_for_addr(dev, 0, dev->dev_addr, 0);

    /* Set up the frame filter control */
    reg = 0;     /* Use defaults */
    emac_iowrite32(reg, EMAC_MAC_FRAME_FLT_REG_OFFSET);

    /* Set the flow control reg */
    reg = 0;
    if (priv->duplex == PHY_DUPLEX_FULL) {
        reg |= (EMAC_RX_FLOW_ENABLE | EMAC_TX_FLOW_ENABLE);
    }
    emac_iowrite32(reg, EMAC_MAC_FLOW_CONTROL_REG_OFFSET);

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
    emac_iowrite32(reg, EMAC_MAC_CONFIG_REG_OFFSET);

    /* Set the operation mode */
    priv->op_mode =
        EMAC_OPERATE_ON_SECOND_FRAME |
        (0x3 << EMAC_TX_THRESHOLD_SHIFT);   /* Set high tx threshold. This
                                               avoids the TX underflow
                                               errors */

    /* Setup the MMI statistics hardware, no interrupts, wrapping counters */
    emac_iowrite32(EMAC_MMC_INT_ALL, EMAC_MMC_RX_INT_MASK_OFFSET);
    emac_iowrite32(EMAC_MMC_INT_ALL, EMAC_MMC_TX_INT_MASK_OFFSET);
    emac_iowrite32(0, EMAC_MMC_CONTROL_OFFSET);

    /* Initialise hw_timestamp_flag */
    priv->hw_timestamp_flag = HWTSTAMP_FILTER_NONE;

    /* Initialise the phy_register (sysfs access) value */
    priv->phy_register = 0;

    /* Enable the interrupts */
    priv->ie_mask = EMAC_ABNORM_INT_MASK | EMAC_SUMMERY_INT_MASK;
    emac_interrupt_control(dev, RX, ENABLE);    /* Enable Rx IRQ */
    emac_interrupt_control(dev, TX, ENABLE);    /* Enable Tx IRQ */

    /* Start the DMA reception and transmission */
    emac_dma_control(dev, RX, ENABLE);      /* Enable Rx */
    emac_dma_control(dev, TX, ENABLE);      /* Enable Tx */

    /* Tell the stack to continue sending us packets */
    netif_wake_queue(dev);

    DB((LVL_TRACE, "emac_start complete\n"));
}

/* Stop the receive and transmit rings. Stop interrupts. Reset the
 * EMAC DMA controller. Does not free any ring buffers.
 */
static void
emac_reset(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    u32 reg;
    unsigned int stops;

    DB((LVL_TRACE, "emac_reset\n"));

    /* This can be called from anywhere, while the rings are in any state.
     * We need to do thread synchronisation and make sure no transmit
     * or receive threads are running before we continue.
     */

    /* Make sure the hard_start_xmit function is not running and will not
     * be called
     */
    netif_tx_disable(dev);

    /* Disable all interrupts */
    priv->ie_mask = 0;
    emac_interrupt_control(dev, RX, DISABLE);    /* Disable Rx */
    emac_interrupt_control(dev, TX, DISABLE);    /* Disable Tx */

    /* Wait for work-queues. This blocks until the handler function has
     * finished running and no others are pending.
     * As the interrupts are disabled at this point, the handler will not
     * be scheduled to run again.
     * This handles the thread synchronisation with ring handlers.
     */
    if (priv->tx_ring != NULL) {
        emac_wait_stop_ring(priv->tx_ring);
    }
    if (priv->rx_ring != NULL) {
        emac_wait_stop_ring(priv->rx_ring);
    }

    /* Stop the DMA hardware */
    emac_dma_control(dev, RX, DISABLE);    /* Disable Rx */
    emac_dma_control(dev, TX, DISABLE);    /* Disable Tx */

    /* Stop the MAC hardware */
    emac_iowrite32(0, EMAC_MAC_CONFIG_REG_OFFSET);

    /* Busy wait for the hardware to stop */
    stops = EMAC_STOP_COUNT;
    do {
        /* Wait a while for the reset to happen */
        udelay(EMAC_STOP_US);

        reg = emac_ioread32(EMAC_DMA_STATUS_REG_OFFSET);
        if ((reg & (EMAC_DMA_TX_STATE_MASK | EMAC_DMA_RX_STATE_MASK)) == 0) {
            break;
        }
    } while(--stops > 0);

    if (stops == 0) {
        /* Timeout waiting for stop */
        printk(KERN_ERR "%s: Can't stop EMAC DMA\n", dev->name);
    }

    /* Set the reset bit to ensure the EMAC DMA is not running */
    emac_iowrite32(EMAC_SOFT_RESET, EMAC_DMA_BUS_MODE_REG_OFFSET);

    /* Wait a while for the reset to happen */
    udelay(EMAC_RESET_US);

    /* Check that the reset has happened */
    reg = emac_ioread32(EMAC_DMA_BUS_MODE_REG_OFFSET);
    if ((reg & EMAC_SOFT_RESET) != 0) {
        /* Reset has not occurred. Error */
        printk(KERN_ERR "%s: Can't reset EMAC DMA\n", dev->name);
    }

    /* Make sure no socket buffers are left attached to the
     * transmit descriptor ring. Free any that are found.
     * We may just have disabled the interrupt before the transmit
     * had a chance to complete.
     */
    emac_free_ring_skb(dev, priv->tx_ring);

    /* Reset the receive rings state */
    emac_reset_ring_state(priv->rx_ring);

    /* Discard received packets transferred into the receive
     * ring after the interrupts were disabled but before the DMA was
     * disabled.
     */
    emac_set_ring_ownership(priv->rx_ring, 0);

#ifdef ENABLE_DEBUGGING
    print_rings(dev, 1);
#endif

    DB((LVL_TRACE, "emac_reset complete\n"));
}

/* Schedule handling of updated link */
static void
emac_adjust_link_trigger(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);

    /* Copy the updated state to out local context. This allows
     * the PHY driver to update state asynchronously
     */
    priv->link = priv->phydev->link;
    priv->duplex = priv->phydev->duplex;
    priv->speed = priv->phydev->speed;

//    DB((LVL_DEBUG, "Link adjust event, link %s\n", priv->link ? "up":"down"));

    /* Schedule the handler */
    queue_work(priv->link_adjust_workqueue, &priv->link_adjust_work);
}

/* Called every time the controller might need to be made
 * aware of new link state.  The PHY code conveys this
 * information through variables in the phydev structure, and this
 * function converts those variables into the appropriate
 * register values, and can bring down the device if needed.
 */
static void
emac_adjust_link(struct work_struct *param)
{
    struct emac_priv *priv = container_of(param, struct emac_priv, link_adjust_work);
    struct net_device *dev = priv->self;
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

    /* Print new link state */
    if (new_state && priv->phydev && netif_msg_link(priv)) {
        phy_print_status(priv->phydev);

#ifdef ENABLE_DEBUGGING
        emac_dump_phy_regs(dev);
#endif
    }
}

/* Initializes driver's PHY state, and attaches to the PHY.
 * Returns 0 on success.
 */
static int
emac_init_phy(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct phy_device *phydev;
    char phy_id[BUS_ID_SIZE];

    int ret = 0;

    priv->oldlink = 0;
    priv->oldspeed = 0;
    priv->oldduplex = -1;

    /* The hardware is hard wired to the phy */
#define PHY_BUS_ID "0"

    snprintf(phy_id, BUS_ID_SIZE, PHY_ID_FMT, PHY_BUS_ID, priv->phy_address);

    /* Attach to the PHY */
    phydev = phy_connect(dev, phy_id, &emac_adjust_link_trigger, 0,
                         PHY_INTERFACE_MODE_MII);
    if (IS_ERR(phydev)) {
        DB((LVL_ERR, "Could not attach to PHY\n"));
        return PTR_ERR(phydev);
    }

    printk(KERN_INFO "%s: Connected to Phy (id = 0x%08x)\n", dev->name,
           phydev->phy_id);

    /* Remove any features not supported by the controller */
    phydev->supported &= EMAC_MII_SUPPORTED;
    phydev->advertising = phydev->supported;

    priv->phydev = phydev;

    if (machine_is_pc7802())
    {
        /* Set up the DP83640 Phy as used on the PC7802 platform */
        ret = emac_init_dp83640_phy(dev);
        if (0 != ret)
        {
            printk(KERN_INFO "%s: Problem with initialising the DP83640\n",
                   dev->name);
        }
    }

    return 0;
}

static int
emac_init_dp83640_phy(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    int value;

    /* Select register bank 4 */
    emac_dp83640_select_page(dev, DP83640_PAGE_SEL_4);

    /* Reset the PTP clock and associated logic */
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)DP83640_PTP_RESET);

    /* The reset bit is not self clearing */
    udelay (1);
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)0);

    /* Enable the PTP Clock */
    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          DP83640_PTP_CTRL_REG);
    value |= DP83640_PTP_ENABLE;
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)value);

    return SUCCESS;
}

static void
emac_dp83640_select_page(struct net_device *dev,
                         unsigned int page)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    int value;

    /* Select the phy register page */
    page &= DP83640_PAGE_SEL_MASK;

    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          DP83640_MII_PAGE_SELECT_REG);
    value &= DP83640_PAGE_SEL_MASK;
    value |= page;
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_MII_PAGE_SELECT_REG,
                         (u16)value);
}

static int
emac_dp83640_set_1588_seconds(struct net_device *dev,
                              u32 timer_seconds)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    int value;

    u16 ns_low, ns_high, sec_low, sec_high;

    /* Select register bank 4 */
    emac_dp83640_select_page(dev, DP83640_PAGE_SEL_4);

    /* Read back the current timer value */
    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          DP83640_PTP_CTRL_REG);
    value |= DP83640_PTP_READ_CLK;
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)value);

    ns_low = emac_mii_read(mii_bus,
                           priv->phy_address,
                           DP83640_PTP_TIME_DATA_REG);
    ns_high = emac_mii_read(mii_bus,
                            priv->phy_address,
                            DP83640_PTP_TIME_DATA_REG);
    sec_low = emac_mii_read(mii_bus,
                            priv->phy_address,
                            DP83640_PTP_TIME_DATA_REG);
    sec_high = emac_mii_read(mii_bus,
                             priv->phy_address,
                             DP83640_PTP_TIME_DATA_REG);

   /* Write back the new seconds value
    * Note: we preserve the ns value
    */
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_TIME_DATA_REG,
                         ns_low);
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_TIME_DATA_REG,
                         ns_high);
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_TIME_DATA_REG,
                         (u16)(timer_seconds & 0xFFFF));
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_TIME_DATA_REG,
                         (u16)((timer_seconds & 0xFFFF0000) >> 16));

    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          DP83640_PTP_CTRL_REG);
    value |=  DP83640_PTP_LOAD_CLK;
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)value);

    return SUCCESS;
}

static int
emac_dp83640_get_1588_seconds(struct net_device *dev,
                              u32 *timer_seconds)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    int value;

    u16 ns_low, ns_high, sec_low, sec_high;
    u32 sec_read;

    /* Select register bank 4 */
    emac_dp83640_select_page(dev, DP83640_PAGE_SEL_4);

    /* Read back the current timer value */
    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          DP83640_PTP_CTRL_REG);
    value |= DP83640_PTP_READ_CLK;
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)value);

    ns_low = emac_mii_read(mii_bus,
                           priv->phy_address,
                           DP83640_PTP_TIME_DATA_REG);
    ns_high = emac_mii_read(mii_bus,
                            priv->phy_address,
                            DP83640_PTP_TIME_DATA_REG);
    sec_low = emac_mii_read(mii_bus,
                            priv->phy_address,
                            DP83640_PTP_TIME_DATA_REG);
    sec_high = emac_mii_read(mii_bus,
                             priv->phy_address,
                             DP83640_PTP_TIME_DATA_REG);

    sec_read = (u32)sec_high;
    sec_read <<= 16;
    sec_read |= sec_low;
    *timer_seconds = sec_read;

    return SUCCESS;
}

static int
emac_dp83640_set_1588_nano_seconds(struct net_device *dev,
                                   u32 timer_nano_seconds)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    int value;

    u16 ns_low, ns_high, sec_low, sec_high;

    /* Select register bank 4 */
    emac_dp83640_select_page(dev, DP83640_PAGE_SEL_4);

    /* Read back the current timer value */
    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          DP83640_PTP_CTRL_REG);
    value |= DP83640_PTP_READ_CLK;
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)value);

    ns_low = emac_mii_read(mii_bus,
                           priv->phy_address,
                           DP83640_PTP_TIME_DATA_REG);
    ns_high = emac_mii_read(mii_bus,
                            priv->phy_address,
                            DP83640_PTP_TIME_DATA_REG);
    sec_low = emac_mii_read(mii_bus,
                            priv->phy_address,
                            DP83640_PTP_TIME_DATA_REG);
    sec_high = emac_mii_read(mii_bus,
                             priv->phy_address,
                             DP83640_PTP_TIME_DATA_REG);

   /* Write back the new nano seconds value
    * Note: we preserve the seconds value
    */
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_TIME_DATA_REG,
                         (u16)(timer_nano_seconds & 0xFFFF));
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_TIME_DATA_REG,
                         (u16)((timer_nano_seconds & 0xFFFF0000) >> 16));
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_TIME_DATA_REG,
                         sec_low);
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_TIME_DATA_REG,
                         sec_high);

    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          DP83640_PTP_CTRL_REG);
    value |=  DP83640_PTP_LOAD_CLK;
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)value);

    return SUCCESS;
}

static int
emac_dp83640_get_1588_nano_seconds(struct net_device *dev,
                                   u32 *timer_nano_seconds)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    int value;

    u16 ns_low, ns_high, sec_low, sec_high;
    u32 ns_read;

    /* Select register bank 4 */
    emac_dp83640_select_page(dev, DP83640_PAGE_SEL_4);

    /* Read back the current timer value */
    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          DP83640_PTP_CTRL_REG);
    value |= DP83640_PTP_READ_CLK;
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_CTRL_REG,
                         (u16)value);

    ns_low = emac_mii_read(mii_bus,
                           priv->phy_address,
                           DP83640_PTP_TIME_DATA_REG);
    ns_high = emac_mii_read(mii_bus,
                            priv->phy_address,
                            DP83640_PTP_TIME_DATA_REG);
    sec_low = emac_mii_read(mii_bus,
                            priv->phy_address,
                            DP83640_PTP_TIME_DATA_REG);
    sec_high = emac_mii_read(mii_bus,
                             priv->phy_address,
                             DP83640_PTP_TIME_DATA_REG);

    ns_read = (u32)ns_high;
    ns_read <<= 16;
    ns_read |= ns_low;
    *timer_nano_seconds = ns_read;

    return SUCCESS;
}

static int
emac_dp83640_set_1588_timer_increment(struct net_device *dev,
                                      u32 rate)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    /* Select register bank 4 */
    emac_dp83640_select_page(dev, DP83640_PAGE_SEL_4);

    /* Write to the rate high register first */
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_RATE_HIGH_REG,
                         (u16)((rate & 0xFFFF0000) >> 16));

    /* Finally write to the rate low register */
    (void)emac_mii_write(mii_bus,
                         priv->phy_address,
                         DP83640_PTP_RATE_LOW_REG,
                         (u16)(rate & 0xFFFF));

    return SUCCESS;
}

static int
emac_dp83640_get_1588_timer_increment(struct net_device *dev,
                                      u32 *rate)
{

    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    u16 rate_low, rate_high;
    u32 rate_read;

    /* Select register bank 4 */
    emac_dp83640_select_page(dev, DP83640_PAGE_SEL_4);

    /* Read the rate value from the Phy */
    rate_low = emac_mii_read(mii_bus,
                             priv->phy_address,
                             DP83640_PTP_RATE_LOW_REG);
    rate_high = emac_mii_read(mii_bus,
                              priv->phy_address,
                              DP83640_PTP_RATE_HIGH_REG);

    rate_read = (u32)rate_high;
    rate_read <<= 16;
    rate_read |= rate_low;
    *rate = rate_read;

    return SUCCESS;
}

/*
 * Free a RX/TX ring
 */
static void
emac_free_ring(struct emac_ring *ring)
{
    if (ring != NULL) {

        /* Free hardware descriptors */
        if (ring->hw_desc != NULL) {
            dma_free_coherent(ring->device,
                    sizeof(struct emac_dma_desc) * ring->length,
                    ring->hw_desc, ring->dma_ring_start);
        }

        /* Free software descriptors */
        if (ring->sw_desc != NULL) {
            kfree(ring->sw_desc);
        }

        /* Free ring */
        kfree(ring);
    }
}

/*
 * Allocate a RX/TX ring
 */
static struct emac_ring
*emac_allocate_ring(struct net_device *dev,
                    unsigned int length)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct emac_ring *ring = NULL;
    int i;

    /* First allocate the ring structure itself */
    ring = (struct emac_ring *)kzalloc(sizeof(struct emac_ring), GFP_ATOMIC);
    if (ring == NULL) {
        DB((LVL_ERR, "Failed to allocate buffer\n"));
        goto out_fail;
    }

    /* Allocate the software descriptors */
    ring->sw_desc = (struct emac_desc *)kzalloc(
            sizeof(struct emac_desc) * length, GFP_ATOMIC);
    if (ring->sw_desc == NULL) {
        DB((LVL_ERR, "Failed to allocate buffer\n"));
        goto out_fail;
    }

    /* Next allocate the hardware descriptors. They must be on
     * word boundaries. We need a coherent buffer so to avoid the
     * cache. This means that changes to the descriptors will be
     * seen immediately by the hardware.
     */
    ring->hw_desc = (struct emac_dma_desc *)dma_alloc_coherent(
            priv->device, sizeof(struct emac_dma_desc) * length,
            &ring->dma_ring_start, GFP_ATOMIC);
    if (ring->hw_desc == NULL) {
        DB((LVL_ERR, "Failed to allocate DMA buffer\n"));
        goto out_fail;
    }

    /* Zero the DMA descriptors */
    memset(ring->hw_desc, 0, sizeof(struct emac_dma_desc) * length);

    /* Setup the ring structure */
    ring->length = length;
    emac_reset_ring_state(ring);
    ring->dev = dev;
    ring->device = priv->device;

    /* Setup all the descriptors */
    for (i = 0; i < length; ++i) {

        /* Setup the descriptor */
        ring->sw_desc[i].dev = dev;
        ring->sw_desc[i].ring = ring;

        /* Mark it as owned by the HOST so the hardware will not use it */
        ring->hw_desc[i].status &= ~EMAC_DESC_STATUS_OWNER;

        /* Link the SW-HW descriptors */
        ring->sw_desc[i].hw_desc = &ring->hw_desc[i];
        ring->hw_desc[i].sw_desc = &ring->sw_desc[i];
    }

    /* Setup the thread lock */
    spin_lock_init(&ring->lock);

    /* Setup the stopping wait queue */
    init_waitqueue_head(&ring->pending_waitq);

    return ring;

    /* Error handling, release resources and return NULL */
out_fail:
    emac_free_ring(ring);
    return NULL;
}

/*
 * Setup the descriptor fields to point to the data buffer.
 * If this function fails, it descriptor is not changed
 */
static int
emac_wireup_desc_dma(struct net_device *dev,
                     struct emac_desc *desc,
                     void *ptr,
                     unsigned int length,
                     enum dma_data_direction dir)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct emac_dma_desc *hw_desc = desc->hw_desc;
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
    dma_addr = dma_map_single(priv->device, ptr, length, dir);
    if (dma_addr == 0) {
        ret = -ENOMEM;
        DB((LVL_ERR, "emac: dma_map_single failed\n"));
        goto out;
    }

    /* Record the direction and length for unmapping later */
    desc->dir = dir;
    desc->length = length;

    /* Decrement the space left on the ring if we are not
     * replacing an existing buffer
     */
    if (hw_desc->buffer1 == 0) {
        desc->ring->space--;
    }

    /* Set the first DMA buffer pointer to the start of the buffer */
    hw_desc->buffer1 = (u32)dma_addr;

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
        hw_desc->buffer2 = (u32)dma_addr + EMAC_MAX_DMA_LENGTH;
    }

    /* Modify the control field */
    buf1_length =
        (buf1_length << EMAC_DESC_CNTL_TBS1_SHIFT) & EMAC_DESC_CNTL_TBS1_MASK;
    buf2_length =
        (buf2_length << EMAC_DESC_CNTL_TBS2_SHIFT) & EMAC_DESC_CNTL_TBS2_MASK;
    hw_desc->control &= ~EMAC_DESC_CNTL_TBS1_MASK;
    hw_desc->control &= ~EMAC_DESC_CNTL_TBS2_MASK;
    hw_desc->control = hw_desc->control | buf1_length | buf2_length;

out:
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
static int
emac_get_next_dma_packet_status(struct emac_ring *ring)
{
    struct emac_desc *head_desc = ring->next_dma;
    struct emac_dma_desc *head_hw_desc = head_desc->hw_desc;
#ifdef CONFIG_FIRECRACKER_EMAC_FRAG_TX
    struct emac_dma_desc *last_hw_desc;
    unsigned int last_idx;
#endif

    /* The next_dma descriptor must be the head of a packet */
    if (head_desc->skb == NULL) {
        DB((LVL_ERR, "emac: Internal Ring ERROR\n"));
        return 0;
    }

#ifdef CONFIG_FIRECRACKER_EMAC_FRAG_TX

    /* Get a pointer to the last descriptor in the packet */
    last_idx = ring->next_dma_idx + skb_shinfo(head_desc->skb)->nr_frags;
    if (last_idx >= ring->length) {
        last_idx -= ring->length;
    }
    last_hw_desc = &ring->hw_desc[last_idx];

    /* Work out who owns the packet */
    if ((head_hw_desc->status & EMAC_DESC_STATUS_OWNER) != 0 &&
            (last_hw_desc->status & EMAC_DESC_STATUS_OWNER) != 0) {
        return EMAC_DMA_OWNED;
    }

    if ((head_hw_desc->status & EMAC_DESC_STATUS_OWNER) == 0 &&
            (last_hw_desc->status & EMAC_DESC_STATUS_OWNER) == 0) {
        return EMAC_HOST_OWNED;
    }

    return EMAC_JOINTLY_OWNED;

#else   /* CONFIG_FIRECRACKER_EMAC_FRAG_TX */

    if ((head_hw_desc->status & EMAC_DESC_STATUS_OWNER) != 0) {
        return EMAC_DMA_OWNED;
    }
    return EMAC_HOST_OWNED;

#endif  /* CONFIG_FIRECRACKER_EMAC_FRAG_TX */

}

/*
 * Normal transmit interrupt handling, free up transmitted packets
 * This is called from a work queue and is not in interrupt context.
 */
static void
emac_tx_interrupt_work(struct work_struct *param)
{
    struct emac_priv *priv = container_of(param, struct emac_priv, tx_int_work);
    struct net_device *dev = priv->self;
    struct emac_ring *ring = priv->tx_ring;
    struct emac_desc *head_desc, *desc;

    /* Thread synchronisation with the transmit function is required here */
    spin_lock_bh(&ring->lock);

    DB((LVL_TRACE, "emac_tx_interrupt_work\n"));

    /* While there are more transmitted packets to free, free them. We
     * need to check that the whole packet is sent before starting to
     * free it. We 'look ahead' down the transmit ring at the descriptor
     * that is wired to the last fragment of the packet. If that one has been
     * sent, we know the whole packet has been sent.
     * This is what emac_get_next_dma_packet_status does for us.
     */
    while (ring->next_dma->skb != NULL &&
            emac_get_next_dma_packet_status(ring) == EMAC_HOST_OWNED) {

        /* Hold the descriptor containing the head of the frame while
         * we unmap fragments. We will free the head last.
         */
        head_desc = ring->next_dma;

        /* Count the transmit errors */
        emac_tx_errors(dev, head_desc->hw_desc);

        /* Move on to the first fragment (or the next frame if
         * there are none)
         */
        desc = emac_next_dma_desc(ring);

        /* Continue unmapping fragments until we get to the
         * end of the frame. This is shown by the absence of a fragment which
         * will be the case weather or not following packets are in the
         * ring.
         */
        while (desc->frag != NULL) {

            /* Count the transmit errors */
            emac_tx_errors(dev, desc->hw_desc);

            /* Free the buffer associated with the descriptor */
            emac_free_skb_frag(dev, desc);

            /* Move on to the next fragment */
            desc = emac_next_dma_desc(ring);
        }

        /* Unmap and free the head last */
        emac_free_skb_frag(dev, head_desc);
    }

    /* We have finished this work item. Maintain the pending count */
    atomic_dec(&ring->pending);
    DB((LVL_DEBUG, "Tx finished, pending count = %u\n",
                atomic_read(&ring->pending)));

    /* Re-enable transmit interrupt, if we are not stopping */
    if (atomic_read(&ring->stopping) == 0) {
        emac_interrupt_control(dev, TX, ENABLE);

        /* Start the stack (if stopped) as there in now more space on the
         * transmit ring
         */
        /* Use netif_wake_queue instead of netif_start_queue here as
         * we want transmission to continue immediately.
         */
        netif_wake_queue(dev);
    }

    /* Release thread sync */
    spin_unlock_bh(&ring->lock);

    /* Wake up any thread that is waiting for the transmit handler
     * to complete
     */
    wake_up_interruptible(&ring->pending_waitq);
}


/*
 * The EMAC interrupt handler.
 */
static irqreturn_t
emac_interrupt(int irq,
               void *dev_id)
{
    struct net_device *dev = (struct net_device *)dev_id;
    struct emac_priv *priv = netdev_priv(dev);
    u32 statusword;

#ifdef ENABLE_DEBUGGING
    print_rings(dev, 0);
#endif

    /* Retrieve the interrupt status */
    statusword = emac_ioread32(EMAC_DMA_STATUS_REG_OFFSET) & priv->ie_mask;
    /* Clear all interrupt status bits we will service */
    emac_iowrite32(statusword, EMAC_DMA_STATUS_REG_OFFSET);

    DB((LVL_DEBUG, "Interrupt, status 0x%08x. netdev state 0x%08x\n", statusword, dev->state));

    /* Handle Fatal bus error */
    if (statusword & EMAC_FATAL_BUS_ERROR_INT) {
        DB((LVL_ERR, "Fatal Bus Error Interrupt Seen\n"));
    }

    /* Handle normal receive interrupts and receive ring full interrupt */
    if (statusword & EMAC_NORM_RX_INT_MASK) {

        DB((LVL_DEBUG, "Handle normal Rx interrupt\n"));

        /* Disable further receive interrupts */
        emac_interrupt_control(dev, RX, DISABLE);

        /* Schedule receive polling */
        atomic_inc(&priv->rx_ring->pending);
        DB((LVL_DEBUG, "Rx pending count = %u\n",
                    atomic_read(&priv->rx_ring->pending)));
        netif_rx_schedule(dev, &priv->napi);
    }

    /* Handle normal transmit interrupts */
    if (statusword & EMAC_NORM_TX_INT_MASK) {

        DB((LVL_DEBUG, "Handle normal Tx interrupt\n"));

        /* Disable transmit interrupts */
        emac_interrupt_control(dev, TX, DISABLE);

        /* Normal transmit handling. Queue work item */
        atomic_inc(&priv->tx_ring->pending);
        DB((LVL_DEBUG, "Tx pending count = %u\n",
                    atomic_read(&priv->tx_ring->pending)));
        queue_work(priv->tx_int_workqueue, &priv->tx_int_work);
    }

    /* Rx unavailable */
    if (statusword & (EMAC_RX_UNAVAILABLE_INT)) {
        DB((LVL_WARNING, "RX Unavailable seen\n"));
        priv->stats.rx_over_errors++;
    }

    /* Handle under/overflows */
    if (statusword & (EMAC_TX_UNDERFLOW_INT | EMAC_RX_OVERFLOW_INT)) {
        DB((LVL_WARNING, "Underflow or overflow seen\n"));
    }

    /* Handle transmit jabber */
    if (statusword & EMAC_TX_JABBER_TIMEOUT_INT) {
        DB((LVL_WARNING, "Transmit Jabber seen\n"));
    }

    /* Handle TX/RX stopped interrupt by restarting */
    if (statusword & (EMAC_TX_STOPPED_INT | EMAC_RX_STOPPED_INT)) {
        DB((LVL_WARNING, "Rx or Tx stopped seen, Restarting\n"));
        emac_dma_control(dev, RX, ENABLE);      /* Enable Rx */
        emac_dma_control(dev, TX, ENABLE);      /* Enable Tx */
    }

    return IRQ_HANDLED;
}

int
emac_release(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);

    /* Stop PHY link status monitor */
    if (priv->phydev != NULL) {
        phy_stop(priv->phydev);
        phy_disconnect(priv->phydev);
    }
    priv->phydev = NULL;

    /* Force an update to the link status immediately, do not wait for the PHY
     * driver code to do it's poll. This saves time and is less complex
     * as we do not have to implement a wait. If the PHY driver happens to
     * trigger the link down around the same time as we do this, a second
     * link down event will be swallowed.
     */
    netif_carrier_off(dev);
    priv->link = 0;
    queue_work(priv->link_adjust_workqueue, &priv->link_adjust_work);

    /* Wait for the link down and reset of hardware */
    if (priv->link_adjust_workqueue != NULL) {
        flush_workqueue(priv->link_adjust_workqueue);
        destroy_workqueue(priv->link_adjust_workqueue);
    }

    if (priv->tx_int_workqueue != NULL) {
        flush_workqueue(priv->tx_int_workqueue);
        destroy_workqueue(priv->tx_int_workqueue);
    }

    /* Release the interrupt, if we have it */
    free_irq(dev->irq, dev);

    if (priv->rx_ring != NULL) {
        /* Free the receive socket buffers */
        emac_free_ring_skb(dev, priv->rx_ring);
    }

    if (priv->tx_ring != NULL) {
        /* Free any transmit socket buffers we may have */
        emac_free_ring_skb(dev, priv->tx_ring);
    }

    /* Free the descriptor rings */
    emac_free_ring(priv->rx_ring);
    emac_free_ring(priv->tx_ring);
    priv->rx_ring = NULL;
    priv->tx_ring = NULL;

    /* Stop transmission */
    netif_stop_queue(dev);

    napi_disable(&priv->napi);

    return 0;
}

/*
 * Open and close
 */

static int
emac_open(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct sk_buff *skb;
    int ret = 0;
    int i;
    struct emac_desc *sw_desc = NULL;
    struct emac_dma_desc *hw_desc = NULL;

    /* Setup a work queue for the transmit interrupt
     * second level processing
     */
    priv->tx_int_workqueue = create_singlethread_workqueue("EMAC TX");
    if (priv->tx_int_workqueue == NULL) {
        ret = -ENOMEM;
        DB((LVL_ERR, "Failed to create TX work queue \n"));
        goto out;
    }

    /* Setup a work item for the above work queue */
    INIT_WORK(&priv->tx_int_work, emac_tx_interrupt_work);

    /* Setup a work queue for the link adjustment handling */
    priv->link_adjust_workqueue = create_singlethread_workqueue("EMAC LINK");
    if (priv->link_adjust_workqueue == NULL) {
        ret = -ENOMEM;
        DB((LVL_ERR, "Failed to create LINK work queue \n"));
        goto out;
    }

    /* Setup a work item for the above work queue */
    INIT_WORK(&priv->link_adjust_work, emac_adjust_link);

    /* Register for interrupts emac_interrupt */
    ret = request_irq(
            dev->irq, &emac_interrupt, IRQF_DISABLED, dev->name, dev);
    if (ret != 0) {
        DB((LVL_ERR, "Failed to register IRQ\n"));
        goto out;
    }

    /* Allocate receive descriptor ring */
    priv->rx_ring = emac_allocate_ring(dev, rx_ring_length);
    if (priv->rx_ring == NULL) {
        ret = -ENOMEM;
        DB((LVL_ERR, "Failed to allocate RX ring\n"));
        goto out;
    }

    /* Allocate transmit descriptor ring */
    priv->tx_ring = emac_allocate_ring(dev, tx_ring_length);
    if (priv->tx_ring == NULL) {
        ret = -ENOMEM;
        DB((LVL_ERR, "Failed to allocate TX ring\n"));
        goto out;
    }

    /* Allocate receive socket buffers and setup receive descriptors */
    for (i = 0; i < rx_ring_length; ++i) {

        /* Handy pointer to the descriptor */
        sw_desc = &priv->rx_ring->sw_desc[i];
        hw_desc = sw_desc->hw_desc;

        /* Setup the receive control */
        hw_desc->control = EMAC_RX_CONTROL_FIELD_INIT;

        /* Allocate a socket buffer and connect it to the descriptor */
        skb = emac_allocate_skb(dev);
        if (skb == NULL) {
            ret = -ENOMEM;
            DB((LVL_ERR, "Failed to allocate RX SKB\n"));
            goto out;
        }

        /* Connect the socket buffer to the descriptor */
        ret = emac_wireup_desc_dma(
                dev, sw_desc, skb->tail,
                skb_tailroom(skb) & ~EMAC_RX_LENGTH_MASK, DMA_FROM_DEVICE);
        if (ret != 0) {
            DB((LVL_ERR, "Failed to map RX SKB\n"));
            goto out;
        }

        /* Associate the descriptor with the socket buffer */
        sw_desc->skb = skb;
        sw_desc->frag = NULL;

        /* Mark it as owned by the DMA so the hardware will fill it */
        hw_desc->status |= EMAC_DESC_STATUS_OWNER;
    }

    /* Mark the last receive descriptor as the end of the ring */
    hw_desc->control |= EMAC_DESC_CNTL_END_RING;

    /* Setup transmit descriptors */
    for (i = 0; i < tx_ring_length; ++i) {

        hw_desc = &priv->tx_ring->hw_desc[i];
        hw_desc->control = EMAC_TX_CONTROL_FIELD_INIT;
    }

    /* Mark the last transmit descriptor as the end of the ring */
    hw_desc->control |= EMAC_DESC_CNTL_END_RING;

    /* Configure the PHY. Start link status monitoring */
    ret = emac_init_phy(dev);
    if (ret != 0) {
        DB((LVL_ERR, "Failed to initialise PHY driver\n"));
        goto out;
    }

    napi_enable(&priv->napi);

    /* Tell the PHY driver we are ready for business, when the link up
     * event occurs, we will start the EMAC DMA hardware (see emac_start)
     */
    phy_start(priv->phydev);

    /* Tell the upper levels not to send us packets to be transmitted, until
     * we have established a link
     */
    netif_carrier_off(dev);
    netif_tx_disable(dev);

    return 0;
out:
    if (ret != 0) {
        emac_release(dev);
    }
    return ret;
}

/*
 * Configuration changes (passed on by ifconfig)
 */
int
emac_config(struct net_device *dev,
            struct ifmap *map)
{
    if (dev->flags & IFF_UP) /* can't act on a running interface */
        return -EBUSY;

    /* Don't allow changing the I/O address */
    if (map->base_addr != dev->base_addr) {
        printk(KERN_WARNING "%s: Can't change I/O address\n", dev->name);
        return -EOPNOTSUPP;
    }

    /* Don't allow changing the IRQ */
    if (map->irq != dev->irq) {
        printk(KERN_WARNING "%s: Can't change IRQ\n", dev->name);
        return -EOPNOTSUPP;
    }

    /* ignore other fields */
    return 0;
}
/* Pass a new received socket buffer to the networking layer */
static int
emac_pass_up_skb(struct net_device *dev,
                 struct sk_buff *skb,
                 u32 status)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);
    unsigned int data_length;
    unsigned short vlan_tag = 0;

    /* Get the received length from the descriptor */
    data_length =
        (status & EMAC_RX_DESC_STATUS_FLEN_MASK) >>
        EMAC_RX_DESC_STATUS_FLEN_SHIFT;

    /* Set the length of the data received in the socket buffer.
     * Trim the received Ethernet FCS and the EMAC checksum from the
     * received buffer so that the networking stack only sees the
     * IP payload. This is done so that the networking stack believes
     * the checksum we will pass in via skb->csum and will not spend time
     * generating its own checksum.
     */
    skb_put(skb, data_length - ETHERNET_FCS_LEN - EMAC_CHECKSUM_LENGTH);

    /* To do hardware checksumming, we need to get the IP payload
     * checksum into skb->csum.
     * The EMAC adds the 1's complement checksum after the packet data.
     * We invert this and pass it in.
     */
    skb->csum = (skb->data[data_length-1] << 8) | skb->data[data_length-2];
    skb->csum = (~skb->csum) & 0xffff;
    skb->ip_summed = 1;   /* Checked by hardware */

    /* Set up the Ethernet device that received the packet */
    skb->dev = dev;

    /* If the hardware tells us it is a VLAN frame, get the tag from the
     * data before we remove the header.
     * This is often done by the hardware but not in our case, we have
     * to look inside the header ourselves.
     * Remove the VLAN header data from the received frame by shifting the
     * source and destination MAC addresses. This turns the VLAN frame
     * into a normal Ethernet packet which is what the network stack
     * expects from us.
     */
    if (status & EMAC_RX_DESC_STATUS_VLAN) {

        /* Ignore the return value. If the hardware tells us it is VLAN
         * and it is not, something much more serious has occurred.
         */
        vlan_get_tag(skb, &vlan_tag);

        /* Shift the MAC addresses to make it into an normal packet */
        memmove(skb->data + VLAN_HLEN, skb->data, ETH_ALEN + ETH_ALEN);
        skb_pull(skb, VLAN_HLEN);
    }

    /* Get the protocol type of the packet */
    skb->protocol = eth_type_trans(skb, dev);

    /* If this is a VLAN packet the hardware will tell us */
    if ((status & EMAC_RX_DESC_STATUS_VLAN) && priv->vlgrp) {

        /* Pass the packet to the VLAN layer */
        return vlan_hwaccel_receive_skb(skb, priv->vlgrp, vlan_tag);
    }

    if (machine_is_pc7802())
    {
        if (priv->hw_timestamp_flag == HWTSTAMP_FILTER_ALL)
        {
            /* We have been asked to include the hardware timestamp of the
               received packet in the socket buffer */

            ktime_t hw_timestamp;

            int value;

            u16 ns_low, ns_high, sec_low, sec_high;

            /* Obtain the current timestamp value from the DP83640
             * Ethernet Phy
             */

            /* Select register bank 4 */
            emac_dp83640_select_page(dev, DP83640_PAGE_SEL_4);

            /* Read back the current timer value */
            value = emac_mii_read(mii_bus,
                                  priv->phy_address,
                                  DP83640_PTP_CTRL_REG);
            value |= DP83640_PTP_READ_CLK;
            (void)emac_mii_write(mii_bus,
                                 priv->phy_address,
                                 DP83640_PTP_CTRL_REG,
                                 (u16)value);

            ns_low = emac_mii_read(mii_bus,
                                   priv->phy_address,
                                   DP83640_PTP_TIME_DATA_REG);
            ns_high = emac_mii_read(mii_bus,
                                    priv->phy_address,
                                    DP83640_PTP_TIME_DATA_REG);
            sec_low = emac_mii_read(mii_bus,
                                    priv->phy_address,
                                    DP83640_PTP_TIME_DATA_REG);
            sec_high = emac_mii_read(mii_bus,
                                     priv->phy_address,
                                     DP83640_PTP_TIME_DATA_REG);

            hw_timestamp.tv.sec = (sec_high << 16) | sec_low;
            hw_timestamp.tv.nsec = (ns_high << 16) | ns_low;

            /* Set hardware timestamp value in the socket buffer */
            skb_hwtstamp_set(skb, hw_timestamp);
        }
    }

    /* Pass the packet to the network layer */
    return netif_receive_skb(skb);
}

/*
 * The poll implementation.
 */
static int
emac_poll(struct napi_struct *napi_s, int budget)
{
    int npackets = 0;
    struct emac_priv *priv = container_of(napi_s, struct emac_priv, napi);
    struct net_device *dev = priv->self;
    struct emac_desc *desc;
    struct emac_dma_desc *hw_desc;
    struct emac_ring *ring = priv->rx_ring;
    struct sk_buff *new_skb, *full_skb;
    int congestion = NET_RX_SUCCESS;

    DB((LVL_TRACE, "emac_poll\n"));

    /* While there are more transmitted packets to free, free them. */
    /* While there are more packets and the stack wants more */
    /* While the network stack is not reporting congestion */
    while (npackets < budget &&
           emac_get_next_dma_packet_status(ring) == EMAC_HOST_OWNED &&
           congestion == NET_RX_SUCCESS) {

        /* Handy pointer to this descriptor, and the socket buffer
         * that is full of received data
         */
        desc = ring->next_dma;
        hw_desc = desc->hw_desc;
        full_skb = desc->skb;

        /* Move on to the next packet for future processing */
        emac_next_dma_desc(ring);

        /* Discard bad frames and count errors */
        if (emac_rx_errors(dev, hw_desc) != 0) {

            /* Re-use the socket buffer that is already connected to the
             * descriptor. Set the ownership to DMA owned.
             */
            hw_desc->status |= EMAC_DESC_STATUS_OWNER;

            /* Continue processing the next packet */
            continue;
        }

        /* Allocate a socket buffer to replace the one we are about to
         * send to the networking layer.
         */
        new_skb = emac_allocate_skb(dev);
        if (new_skb == NULL) {

            /* Socket buffer allocation failure. Debug message */
            DB((LVL_NOTICE, "Packet Dropped\n"));

            /* Record the dropped packets */
            priv->stats.rx_dropped++;

            /* Re-use the socket buffer that is already connected to the
             * descriptor as we cannot allocate a new one. Mark the
             * descriptor for filling by the EMAC DMA, thus overwriting
             * the received data.
             */
            hw_desc->status |= EMAC_DESC_STATUS_OWNER;

            /* Continue processing the next packet */
            continue;
        }

        /* Unmap the socket buffer that has received data */
        emac_unmap_skb_frag(dev, desc);

        /* Pass the socket buffer that has received data to
         * the network layer
         */
        congestion = emac_pass_up_skb(dev, full_skb, hw_desc->status);

        /* Handle congestion in the network stack */
        switch (congestion) {
            case NET_RX_CN_LOW:
                DB((LVL_NOTICE, "Receive congestion Low\n"));
                break;
            case NET_RX_CN_MOD:
                DB((LVL_NOTICE, "Receive congestion Medium\n"));
                break;
            case NET_RX_CN_HIGH:
                DB((LVL_NOTICE, "Receive congestion High\n"));
                break;
            case NET_RX_DROP:
                break;
        }

        /* Count the packets sent to the network layer so that we
         * do not exceed our quota.
         */
        npackets++;

        /* Connect the newly allocated socket buffer to the descriptor */
        if (emac_wireup_desc_dma(
                dev, desc, new_skb->tail,
                skb_tailroom(new_skb) & ~EMAC_RX_LENGTH_MASK,
                DMA_FROM_DEVICE) != 0) {

            /* This really needs to succeed. If it fails, all we can do
             * is leave the descriptor owned by the DMA. When the DMA comes
             * around the ring to this descriptor again, it will stop.
             * Closing and re-opening the driver will restart reception
             * in this situation.
             */
            DB((LVL_ERR, "Mmap fail! Reception disabled.\n"));

            dev_kfree_skb(new_skb);
            new_skb = NULL;
            desc->skb = NULL;
            hw_desc->buffer1 = 0;
            ring->space++;
            continue;
        }

        /* Associate the descriptor with the new socket buffer */
        desc->skb = new_skb;

        /* Mark it as owned by the DMA so the hardware will fill it */
        hw_desc->status |= EMAC_DESC_STATUS_OWNER;
    }

    /* If we processed all packets, we're done; tell the kernel and
     * re-enable interrupts
     */
    if (emac_get_next_dma_packet_status(ring) != EMAC_HOST_OWNED) {

        /* Stop polling, we have received everything */
        atomic_dec(&ring->pending);
        DB((LVL_DEBUG, "Rx poll finished, pending count = %u\n",
                    atomic_read(&ring->pending)));

        netif_rx_complete(dev, &priv->napi);

        /* Enable the receive interrupts */
        if (atomic_read(&ring->stopping) == 0) {
            emac_interrupt_control(dev, RX, ENABLE);
        }

        /* Wake up any thread that is waiting for the receive handler
         * to complete
         */
        wake_up_interruptible(&ring->pending_waitq);

        return npackets;
    }

    DB((LVL_DEBUG, "Rx poll unfinished, pending count = %u\n",
                atomic_read(&ring->pending)));

    /* Kick the hardware to continue receiving if it is suspended as there
     * are now more descriptors available for the DMA to write into.
     */
    emac_hw_rx_resume(dev);

    /* We couldn't process everything. */
    return npackets;
}

/*
 * Transmit a packet (called by the kernel)
 */
int
emac_tx(struct sk_buff *skb,
        struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    int ret = NETDEV_TX_OK;
    struct emac_desc *desc, *first_desc;
    struct emac_dma_desc *hw_desc;
    struct emac_ring *ring = priv->tx_ring;
#ifdef CONFIG_FIRECRACKER_EMAC_FRAG_TX
    unsigned int f;
    unsigned int nr_frags;
    struct skb_frag_struct *frag;
#endif

    DB((LVL_TRACE, "emac_tx\n"));

    /* Thread synchronisation with the transmit function is required here */
    spin_lock_bh(&ring->lock);

#ifndef CONFIG_FIRECRACKER_EMAC_FRAG_TX

    /* We do not like fragmented packets */
    if (skb_shinfo(skb)->nr_frags > 0) {
        DB((LVL_WARNING, "Fragmented Tx packet, dropping\n"));
        goto out_fail;
    }

    /* Check that we have enough space on the ring */
    if (ring->space == 0) {
#ifdef ENABLE_DEBUGGING
        print_rings(dev, 1);
#endif /* ENABLE_DEBUGGING */
        /* Stop the stack to give us time to clear the backlog */
        netif_stop_queue(dev);
        ret = NETDEV_TX_BUSY;
        DB((LVL_NOTICE, "TX ring full. Deferring. TX clean pending count = %u\n", atomic_read(&ring->pending)));
        priv->stats.tx_fifo_errors++;
        goto out;
    }

#else /* CONFIG_FIRECRACKER_EMAC_FRAG_TX */

    /* Check that we have enough space on the ring for the fragmented
     * packet, head plus fragments.
     */
    nr_frags = skb_shinfo(skb)->nr_frags;
    if (ring->space < (nr_frags + 1)) {
        /* Stop the stack to give us time to clear the backlog */
        netif_stop_queue(dev);
        ret = NETDEV_TX_BUSY;
        DB((LVL_NOTICE, "TX ring full. Deferring.\n"));
        priv->stats.tx_fifo_errors++;
        goto out;
    }

#endif  /* CONFIG_FIRECRACKER_EMAC_FRAG_TX */

    /* We are required to save the time stamp */
    dev->trans_start = jiffies;

    /* Add the socket buffer to the transmit ring. We assume that the
     * socket buffer is not larger that a page. This is a valid assumption
     * as we have a maximum MTU of 4k (a page) and the networking layer
     * should not be sending us frames larger than this.
     */

    /* Handy pointer to the first transmit descriptor to be added */
    first_desc = ring->next;
    desc = first_desc;
    hw_desc = desc->hw_desc;

    /* Set the descriptor to point to the socket buffer. Data is passed
     * to the hardware hence DMA_TO_DEVICE.
     */
    ret = emac_wireup_desc_dma(
            dev, desc, skb->data, skb->len - skb->data_len, DMA_TO_DEVICE);
    if (ret != 0) {
        /* If the mmap fails, exit discarding the packet. This is safe as
         * we have not changed the ring yet.
         */
        ret = NETDEV_TX_OK;
        DB((LVL_ERR, "Failed to map TX SKB. Discarding packet\n"));
        goto out_fail;
    }

    /* Associate the first descriptor with the socket buffer head */
    desc->skb = skb;
    desc->frag = NULL;


#ifdef CONFIG_FIRECRACKER_EMAC_FRAG_TX

    /* Mark the first descriptor as the start of the frame */
    hw_desc->control |= EMAC_TX_DESC_CNTL_FIRST_SEG;
    hw_desc->control &= ~EMAC_TX_DESC_CNTL_LAST_SEG;

    if (nr_frags > 0) {
        DB((LVL_NOTICE, "Fragmented Tx packet\n"));
    }

    /* Deal with fragments before marking the first as owned by DMA */

    /* For each fragment of the shared info, add it to a descriptor */
    for (f = 0; f < nr_frags; f++) {

        /* Move on to the next descriptor */
        desc = emac_next_desc(ring);
        hw_desc = desc->hw_desc;

        /* Handy pointer to the next fragment */
        frag = &skb_shinfo(skb)->frags[f];

        /* Set the descriptor to point to the fragment */
        ret = emac_wireup_desc_dma(dev, desc,
                page_address(frag->page) + frag->page_offset, frag->size,
                DMA_TO_DEVICE);
        if (ret != 0) {
            /* If the mmap fails, move the next pointer back to the previous
             * and set as the end of the packet. The partial packet will
             * be transmitted. This is the cleanest way to deal with this
             * unusual error path.
             */
            desc = emac_previous_desc(ring);
            hw_desc = desc->hw_desc;
            ret = NETDEV_TX_OK;
            break;
        }

        /* Associate the descriptor with the buffer fragment */
        desc->skb = NULL;
        desc->frag = frag;

        /* Mark as nether the first or last descriptor */
        hw_desc->control &= ~EMAC_TX_DESC_CNTL_FIRST_SEG;
        hw_desc->control &= ~EMAC_TX_DESC_CNTL_LAST_SEG;

        /* Mark the descriptor as owned by the DMA */
        hw_desc->status |= EMAC_DESC_STATUS_OWNER;
    }

    /* Mark the last descriptor as the end of the frame */
    hw_desc->control |= EMAC_TX_DESC_CNTL_LAST_SEG;

#endif  /* CONFIG_FIRECRACKER_EMAC_FRAG_TX */

    /* Move on to the next descriptor where the
     * following frame will be added
     */
    emac_next_desc(ring);

    /* Now that the tail fragments have been passed to DMA ownership,
     * mark the first (head) descriptor as owned by the DMA. This
     * avoids a race condition between the DMA hardware reading fragments
     * at the same time as we are adding them to the ring. The DMA will
     * only start reading fragments when the first (head) ownership
     * is passed to the DMA. At this point, all subsequent fragments
     * will have been setup.
     */
    first_desc->hw_desc->status |= EMAC_DESC_STATUS_OWNER;

    /* Make sure the hardware is sending and not suspended */
    emac_hw_tx_resume(dev);

    goto out;


out_fail:
    /* Free the socket buffer as we are discarding it */
    dev_kfree_skb_any(skb);

out:
    /* Release thread sync */
    spin_unlock_bh(&ring->lock);

    return ret;
}

/*
 * Deal with a transmit timeout.
 */
static void
emac_tx_timeout(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);

    DB((LVL_WARNING, "Transmit timeout at %ld, latency %ld\n", jiffies,
                jiffies - dev->trans_start));

    /* Simulate a dropped link and get the PHY link maintaining code to
     * reset the link and hardware.
     * I assume the link is up when this is called.
     */
    priv->oldlink = 0;
    priv->link = 1;
    queue_work(priv->link_adjust_workqueue, &priv->link_adjust_work);
}

/*
 * Support for ethtool PHY operations
 */
static int
emac_get_settings(struct net_device *dev,
                       struct ethtool_cmd *cmd)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct phy_device *phydev = priv->phydev;

    if (!phydev)
    {
	return -ENODEV;
    }

    return phy_ethtool_gset(phydev, cmd);
}

static int
emac_set_settings(struct net_device *dev,
                       struct ethtool_cmd *cmd)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct phy_device *phydev = priv->phydev;

    if (!phydev)
    {
        return -ENODEV;
    }

    return phy_ethtool_sset(phydev, cmd);
}

static void
emac_get_drvinfo(struct net_device *dev,
	              struct ethtool_drvinfo *info)
{
    struct emac_priv *priv = netdev_priv(dev);

    strcpy(info->driver, priv->pdev->dev.driver->name);
    strcpy(info->bus_info, priv->pdev->dev.bus_id);
}

static struct
ethtool_ops emac_ethtool_ops =
{
    .get_settings   = emac_get_settings,
    .set_settings   = emac_set_settings,
    .get_drvinfo    = emac_get_drvinfo,
    .get_link	    = ethtool_op_get_link,
};

/*
 * Ioctl commands
 */
int
emac_ioctl(struct net_device *dev,
           struct ifreq *rq,
           int cmd)
{
    if (machine_is_pc7802())
    {
        switch (cmd)
        {
            case SIOCSHWTSTAMP:
	        return emac_hwtstamp_ioctl(dev, rq, cmd);

            default:
	        return -EOPNOTSUPP;
        }
    }
    else
    {
        return 0;
    }
}

static int emac_hwtstamp_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct hwtstamp_config config;

    if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
    {
        return -EFAULT;
    }

    /* reserved for future extensions */
    if (config.flags)
    {
	return -EINVAL;
    }

    /* We do not support hardware timestamping of outgoing packets,
       leave the frame work in here for possible future software
       enhancements */
    switch (config.tx_type)
    {
        case HWTSTAMP_TX_OFF:
	    break;

        case HWTSTAMP_TX_ON:
	    break;

        default:
	    return -ERANGE;
    }

    switch (config.rx_filter_type)
    {
        case HWTSTAMP_FILTER_NONE:
            /* Hardware timestamping not required */
            priv->hw_timestamp_flag = HWTSTAMP_FILTER_NONE;

            printk(KERN_INFO "%s: Hardware timestamping of received "
                             "packets disabled\n", dev->name);
            break;

        case HWTSTAMP_FILTER_ALL:
	    /* Hardware timestamping of received packets is required */
            priv->hw_timestamp_flag = HWTSTAMP_FILTER_ALL;

            printk(KERN_INFO "%s: Hardware timestamping of received "
                             "packets enabled\n", dev->name);
	    config.rx_filter_type = HWTSTAMP_FILTER_ALL;
	    break;

        case HWTSTAMP_FILTER_SOME:
            printk(KERN_INFO "%s: HWTSTAMP_FILTER_SOME not "
                             "supported by driver\n", dev->name);
            break;

        case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
            printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V1_L4_EVENT"
                             " not supported by driver\n", dev->name);
            break;

        case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V1_L4_SYNC"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_L4_EVENT"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_L4_SYNC"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_L2_EVENT"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_L2_SYNC"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_EVENT:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_EVENT"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_SYNC:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_SYNC"
                              " not supported by driver\n", dev->name);
            break;

	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
             printk(KERN_INFO "%s: HWTSTAMP_FILTER_PTP_V2_DELAY_REQ"
                              " not supported by driver\n", dev->name);
            break;

        default:
	    return -ERANGE;
    }

    return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT : 0;
}

/*
 * Return statistics to the caller
 */
struct net_device_stats
*emac_stats(struct net_device *dev)
{
    struct net_device_stats *stats =
        &((struct emac_priv *)netdev_priv(dev))->stats;

    /* Read the hardware statistics into our local statistics */
    stats->tx_packets = emac_ioread32(EMAC_MMC_TX_FRAME_COUNT_GB);
    stats->rx_packets = emac_ioread32(EMAC_MMC_RX_FRAME_COUNT_GB);
    stats->tx_bytes = emac_ioread32(EMAC_MMC_TX_BYTE_COUNT_GB);
    stats->rx_bytes = emac_ioread32(EMAC_MMC_RX_BYTE_COUNT_GB);

    /* bad packets received (total frames minus good frames) */
    stats->rx_errors =
        stats->rx_packets -
        emac_ioread32(EMAC_MMC_RX_BROADCAST_FRAMES_G) -
        emac_ioread32(EMAC_MMC_RX_MULTICAST_FRAMES_G) -
        emac_ioread32(EMAC_MMC_RX_UNICAST_FRAMES_G);

    /* packet transmit problems (total frames minus good frames) */
    stats->tx_errors =
        stats->tx_packets - emac_ioread32(EMAC_MMC_TX_FRAME_COUNT_G);

    /* transmit under run */
    stats->tx_dropped = emac_ioread32(EMAC_MMC_TX_UNDERFLOW);

    /* multicast packets received */
    stats->multicast = emac_ioread32(EMAC_MMC_RX_MULTICAST_FRAMES_G);

    /* packets sent after collisions in half duplex mode */
    stats->collisions =
        emac_ioread32(EMAC_MMC_TX_SINGLE_COLL_G) +
        emac_ioread32(EMAC_MMC_TX_MULTI_COLL_G);

    /* frames received with length error */
    stats->rx_length_errors = emac_ioread32(EMAC_MMC_RX_LENGTH_ERROR);

    /* received packet with CRC error */
    stats->rx_crc_errors = emac_ioread32(EMAC_MMC_RX_CRC_ERROR);

    /* received frame alignment error */
    stats->rx_frame_errors = emac_ioread32(EMAC_MMC_RX_ALLIGN_ERROR);

    /* received FIFO overrun */
    stats->rx_fifo_errors =
        (emac_ioread32(EMAC_DMA_STATS_REG_OFFSET) &&
        EMAC_FIFO_OVERFLOW_MASK) >> EMAC_FIFO_OVERFLOW_SHIFT;

    /* receiver missed packet */
    stats->rx_missed_errors =
        (emac_ioread32(EMAC_DMA_STATS_REG_OFFSET) &&
        EMAC_MISSED_FRAMES_MASK) >> EMAC_MISSED_FRAMES_SHIFT;

    /* transmit aborted due to excessive collisions */
    stats->tx_aborted_errors = emac_ioread32(EMAC_MMC_TX_EXCESS_COLL);

    /* transmit aborted due to carrier sense error */
    stats->tx_carrier_errors = emac_ioread32(EMAC_MMC_TX_CARRIER_ERR);

    /* the self-test indicates my transceiver might be broken */
    stats->tx_heartbeat_errors = 0;

    /* transmit aborted due late collision */
    stats->tx_window_errors = emac_ioread32(EMAC_MMC_TX_LATE_COLL);

    stats->rx_compressed = 0;
    stats->tx_compressed = 0;

    return stats;
}

/* Enables and disables VLAN insertion/extraction */
static void
emac_vlan_rx_register(struct net_device *dev,
                      struct vlan_group *grp)
{
    struct emac_priv *priv = netdev_priv(dev);

    priv->vlgrp = grp;
}

static void
emac_vlan_rx_kill_vid(struct net_device *dev,
                      uint16_t vid)
{
    /* Nothing to do. We do not support filtering */
}

/* Update the hash table based on the current list of multicast
 * addresses we subscribe to.  Also, change the promiscuity of
 * the device based on the flags (this function is called
 * whenever dev->flags is changed */
static void
emac_set_multi(struct net_device *dev)
{
    struct emac_priv *priv = netdev_priv(dev);
    struct dev_mc_list *mc_ptr;
    u32 ff_reg = 0;

    if(dev->flags & IFF_PROMISC) {
        if (netif_msg_drv(priv))
            printk(KERN_INFO "%s: Entering promiscuous mode.\n",
                    dev->name);

        /* Set the PR (Promiscuous Mode) frame filter mode */
        ff_reg |= EMAC_PROMISCUOUS_MODE;
    }

    /* Deal with multicast */
    if(dev->flags & IFF_ALLMULTI) {

        /* Set the PM (Pass All Multicast) frame filter mode */
        ff_reg |= EMAC_PASS_ALL_MULTICAST;
    }
    else {

        int idx;

        /* zero out the hash */
        emac_iowrite32(0, EMAC_MAC_HASH_HIGH_REG_OFFSET);
        emac_iowrite32(0, EMAC_MAC_HASH_LOW_REG_OFFSET);

        idx = 1;

        /* Clear the hash masks */
        priv->hash_high = 0;
        priv->hash_low = 0;

        if (dev->mc_count > (EMAC_MAX_MAC_ADDRS - 1)) {
            /* Use hash tables for filtering more that 16 addresses */
            for (mc_ptr = dev->mc_list; mc_ptr; mc_ptr = mc_ptr->next) {

                emac_calc_hash_for_addr(dev, mc_ptr->dmi_addr);
            }

            /* Set the HMC (Hash MultiCast) bit of the filter control */
            ff_reg |= EMAC_HASH_MULTICAST;
        }

        else if (dev->mc_count > 0) {
            /* Use perfect filtering if between 0 and 15 addresses */

            /* Parse the list, and set the appropriate bits */
            for (mc_ptr = dev->mc_list; mc_ptr; mc_ptr = mc_ptr->next) {

                emac_set_mac_for_addr(dev, idx, mc_ptr->dmi_addr, 0);
                idx++;
            }
        }

        /* Clear out any remaining exact match addresses */
        for (; idx < EMAC_MAX_MAC_ADDRS; ++idx) {
            emac_reset_mac(dev, idx);
        }

        /* Set the calculated final hash masks */
        emac_set_hash(dev);
    }

    /* Set the frame filter control */
    emac_iowrite32(ff_reg, EMAC_MAC_FRAME_FLT_REG_OFFSET);
}


/*
 * The init function (sometimes called probe).
 * It is invoked by register_netdev()
 */
void
emac_init(struct net_device *dev,
          struct platform_device *pdev)
{
    struct emac_priv *priv = netdev_priv(dev);

    /*
     * Then, assign other fields in dev, using ether_setup() and some
     * hand assignments
     */
    ether_setup(dev); /* assign some of the fields */

    dev->open               = emac_open;
    dev->stop               = emac_release;
    dev->set_config         = emac_config;
    dev->hard_start_xmit    = emac_tx;
    dev->do_ioctl           = emac_ioctl;
    dev->get_stats          = emac_stats;
    dev->tx_timeout         = emac_tx_timeout;
    dev->watchdog_timeo     = msecs_to_jiffies(watchdog);
    dev->set_multicast_list = emac_set_multi;
    dev->vlan_rx_register   = emac_vlan_rx_register;
    dev->vlan_rx_kill_vid   = emac_vlan_rx_kill_vid;

    /* Setup our features */
    dev->features           |= NETIF_F_HW_VLAN_RX;
#ifdef CONFIG_FIRECRACKER_EMAC_FRAG_TX
    /* We will accept fragmented transmit packets */
    dev->features           |= NETIF_F_SG;
#endif

    /* Reset the private data */
    memset(priv, 0, sizeof(struct emac_priv));

    netif_napi_add(dev, &priv->napi, emac_poll, napi_poll_weight);

    dev->ethtool_ops = &emac_ethtool_ops;

    /* Get a pointer to the struct device for mapping operations */
    priv->device = &pdev->dev;
    priv->pdev = pdev;
    priv->self = dev;

    /* Setup the thread lock for mirrored registers */
    spin_lock_init(&priv->mirror_lock);

    /* Assign the hardware address of the interface */
    emac_get_mac(dev, dev->dev_addr);

    /* Set up the Phy Address */
    if (machine_is_pc7802())
    {
        /* On the PC7802 platform the Phy Address is 1 */
        priv->phy_address = 1;
    }
    else
    {
        /* On the PC7202/5 the PC20x is running in reverse
         * MII mode.  The Phy Address of the internal Phy
         * is 0.
         */
        priv->phy_address = 0;
    }
}

/* Sysfs functions and data structures */
static ssize_t
emac_sysfs_1588_seconds_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
    u32 value;

    (void)emac_dp83640_get_1588_seconds(sysfs_ndev, &value);

    return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t
emac_sysfs_1588_seconds_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count)
{
    unsigned long value = 0;
    char *endp;

    value = simple_strtoul(buf, &endp, 0);

    if (buf != endp)
    {
        /* We have some valid data to write */
        (void)emac_dp83640_set_1588_seconds(sysfs_ndev, (u32)value);
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(1588_timer_seconds, (S_IRUGO | S_IWUGO),
                   emac_sysfs_1588_seconds_show,
                   emac_sysfs_1588_seconds_store);

static ssize_t
emac_sysfs_1588_nano_seconds_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
    u32 value;

    (void)emac_dp83640_get_1588_nano_seconds(sysfs_ndev, &value);

    return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t
emac_sysfs_1588_nano_seconds_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count)
{
    unsigned long value = 0;
    char *endp;

    value = simple_strtoul(buf, &endp, 0);

    if (buf != endp)
    {
        /* We have some valid data to write */
        (void)emac_dp83640_set_1588_nano_seconds(sysfs_ndev, (u32)value);
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(1588_timer_nano_seconds, (S_IRUGO | S_IWUGO),
                   emac_sysfs_1588_nano_seconds_show,
                   emac_sysfs_1588_nano_seconds_store);

static ssize_t
emac_sysfs_1588_timer_increment_show(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
    u32 value;

    (void)emac_dp83640_get_1588_timer_increment(sysfs_ndev, &value);

    return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t
emac_sysfs_1588_timer_increment_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf,
                                      size_t count)
{
    unsigned long value = 0;
    char *endp;

    value = simple_strtoul(buf, &endp, 0);

    if (buf != endp)
    {
        /* We have some valid data */
        (void)emac_dp83640_set_1588_timer_increment(sysfs_ndev, (u32)value);
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(1588_timer_increment, (S_IRUGO | S_IWUGO),
                   emac_sysfs_1588_timer_increment_show,
                   emac_sysfs_1588_timer_increment_store);

static ssize_t
emac_sysfs_phy_register_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf)
{
    struct emac_priv *priv = netdev_priv(sysfs_ndev);

    return snprintf(buf, PAGE_SIZE, "%u\n", priv->phy_register);
}

static ssize_t
emac_sysfs_phy_register_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count)
{
    struct emac_priv *priv = netdev_priv(sysfs_ndev);

    unsigned long value = 0;
    char *endp;

    value = simple_strtoul(buf, &endp, 0);

    if (buf != endp)
    {
        /* We have some valid data to write */
        priv->phy_register = (u32)value;
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(phy_register, (S_IRUGO | S_IWUGO),
                   emac_sysfs_phy_register_show,
                   emac_sysfs_phy_register_store);

static ssize_t
emac_sysfs_phy_register_value_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    struct emac_priv *priv = netdev_priv(sysfs_ndev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    int value;

    value = emac_mii_read(mii_bus,
                          priv->phy_address,
                          priv->phy_register);

    return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t
emac_sysfs_phy_register_value_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    struct emac_priv *priv = netdev_priv(sysfs_ndev);
    struct mii_bus *mii_bus = (struct mii_bus *)dev_get_drvdata(priv->device);

    unsigned long value = 0;
    char *endp;

    value = simple_strtoul(buf, &endp, 0);

    if (buf != endp)
    {
        emac_mii_write(mii_bus,
                       priv->phy_address,
                       priv->phy_register,
                       (u16)value);
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(phy_register_value, (S_IRUGO | S_IWUGO),
                   emac_sysfs_phy_register_value_show,
                   emac_sysfs_phy_register_value_store);

/*!
 * The group of sysfs attributes that should be added
 * to the sysfs filesystem.
 */
static struct attribute *emac_sysfs_attrs[] =
{
    &dev_attr_1588_timer_seconds.attr,
    &dev_attr_1588_timer_nano_seconds.attr,
    &dev_attr_1588_timer_increment.attr,
    &dev_attr_phy_register.attr,
    &dev_attr_phy_register_value.attr,
    NULL
};

/*!
 * Add the sysfs attributes as a group
 */
static struct attribute_group emac_attr_group =
{
    .attrs = emac_sysfs_attrs
};

static int
emac_sysfs_add(struct platform_device *pdev)
{
    return sysfs_create_group(&pdev->dev.kobj, &emac_attr_group);
}

static void
emac_sysfs_remove(struct platform_device *pdev)
{
    sysfs_remove_group(&pdev->dev.kobj, &emac_attr_group);
}

/*
 * MII Platform stuff
 */
static int
emac_mii_probe(struct device *dev)
{
    struct mii_bus *new_bus;
    int err = 0;
    int i;
    static int mii_irqs[PHY_MAX_ADDR];
    int mii_bus_id = 0;

    if (NULL == dev)
        return -EINVAL;

    new_bus = mdiobus_alloc();

    if (NULL == new_bus) {
        DB((LVL_ERR, "Failed to init mdiobus\n"));
        return -ENOMEM;
    }

    new_bus->name = "pc20x MII Bus";
    new_bus->read = &emac_mii_read;
    new_bus->write = &emac_mii_write;
    snprintf(new_bus->id, MII_BUS_ID_SIZE, "%x", mii_bus_id);

    new_bus->priv = NULL;

    /* No irqs */
    for (i = 0; i < PHY_MAX_ADDR; ++i) {
        mii_irqs[i] = -1;
    }
    new_bus->irq = mii_irqs;

    new_bus->parent = dev;
    dev_set_drvdata(dev, new_bus);

    err = mdiobus_register(new_bus);

    if (0 != err) {
        printk (KERN_ERR "%s: Cannot register as MDIO bus\n", new_bus->name);
        goto reg_map_fail;
    }

    return 0;

reg_map_fail:
    mdiobus_free(new_bus);

    return err;
}

static int
emac_mii_remove(struct device *dev)
{
    struct mii_bus *bus = dev_get_drvdata(dev);

    mdiobus_unregister(bus);

    dev_set_drvdata(dev, NULL);

    kfree(bus);

    return 0;
}

/*
 * Finally, the platform/module stuff
 */
static int
emac_drv_probe(struct platform_device *pdev)
{
    struct net_device *ndev;

    int ret;

    struct resource *mem = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    struct resource *irq = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );

    if ( !mem || !irq )
    {
        /* Oops, we do not have valid resources */
        printk (KERN_ERR "%s: Could not obtain platform resources.\n",
                          CARDNAME);
        return -EINVAL;
    }

    if (!request_mem_region(mem->start, (mem->end - mem->start) + 1, CARDNAME))
    {
        printk (KERN_ERR "%s: Memory mapping error, Address=0x%08x,"
                         " Size=0x%08x\n", CARDNAME, mem->start,
                          (mem->end - mem->start) + 1);
        return -ENOMEM;
    }

    mem_region = ioremap(mem->start, (mem->end - mem->start) + 1);
    if (!mem_region)
    {
        /* Oops, not a successful ioremap */
        printk (KERN_ERR "%s: Could not remap io addresses.\n",
                          CARDNAME);

        ret = -ENOMEM;
        goto out_alloc_failed;
    }

    ndev = alloc_etherdev(sizeof(struct emac_priv));
    if (!ndev)
    {
        printk(KERN_ERR "%s: Could not allocate net_device structure.\n",
                         CARDNAME);
        ret = -ENOMEM;
        goto out_alloc_failed;
    }

    SET_NETDEV_DEV(ndev, &pdev->dev);

    ndev->dma = (unsigned char)-1;
    ndev->irq = irq->start;

    platform_set_drvdata(pdev, ndev);

    emac_init(ndev, pdev);

    /* Register it */
    ret = register_netdev(ndev);
    if (ret != 0)
    {
        printk("%s: Error %i registering device \"%s\"\n", CARDNAME,
                ret, ndev->name);
        goto out_free_netdev;
    }

    if (machine_is_pc7802())
    {
        /* Initislise our local copy for sysfs use */
        sysfs_ndev = ndev;

        /* Setup sysfs support */
        ret = emac_sysfs_add(pdev);
        if (ret)
        {
            /* Oops, something not right here */
            printk(KERN_ERR "%s: failed to create sysfs device attributes\n",
                   CARDNAME);
        }
    }

    /* We have success */
    printk(KERN_INFO "%s version %s loaded\n", TITLE, VERSION);
    return 0;

 out_free_netdev:
    free_netdev(ndev);

 out_alloc_failed:
    release_mem_region (mem->start, (mem->end - mem->start) + 1);

    /* We have failed in some way */
    printk(KERN_ERR "%s: registration failed.\n", CARDNAME);
    return ret;
}


static int
emac_drv_remove(struct platform_device *pdev)
{
    struct net_device *ndev = platform_get_drvdata(pdev);
    struct resource *mem = platform_get_resource( pdev, IORESOURCE_MEM, 0 );

    platform_set_drvdata(pdev, NULL);

    unregister_netdev(ndev);

    /* All resources should have been released when the interface was closed */
    free_netdev(ndev);

    if (mem_region)
    {
        iounmap(mem_region);
        release_mem_region (mem->start, (mem->end - mem->start) + 1);
    }

    /* Remove sysfs support */
    emac_sysfs_remove(pdev);

    return 0;
}

static struct platform_driver emac_driver =
{
    .probe      = emac_drv_probe,
    .remove     = emac_drv_remove,
    .driver     =
    {
        .name   = CARDNAME,
    }
};

static struct device_driver emac_mii_driver =
{
    .name = "pc20x-mii",
    .bus = &platform_bus_type,
    .probe = emac_mii_probe,
    .remove = emac_mii_remove,
};

static int __init
emac_init_module(void)
{
    int ret;

    ret = platform_driver_register(&emac_driver);
    if (ret != 0)
    {
        DB((LVL_ERR, "Failed to register EMAC driver\n"));
        goto out;
    }

    ret = driver_register(&emac_mii_driver);
    if (ret != 0)
    {
        DB((LVL_ERR, "Failed to register MII driver\n"));
        goto out_deregister_platform;
    }

    return ret;

out_deregister_platform:
    platform_driver_unregister(&emac_driver);

out:
    return ret;
}

static void __exit
emac_cleanup_module(void)
{
    driver_unregister(&emac_mii_driver);
    platform_driver_unregister(&emac_driver);
}

module_init(emac_init_module);
module_exit(emac_cleanup_module);

MODULE_AUTHOR("picoChip");
MODULE_DESCRIPTION( "picoChip PC20x Ethernet Driver" );
MODULE_LICENSE("GPL");
