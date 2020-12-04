/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * \file pc302_emac.c
 * \brief This module provides the kernel network interface to the
 *        PC302 EMAC hardware.
 *
 * The driver is based on the Atmel MACB Ethernet Controller driver
 * (macb.c/h) Copyright (C) 2004-2006 Atmel Corporation.
 *
 * Copyright (c) 2008 picoChip Designs Ltd.
 * Copyright (c) 2010 ip.access Ltd.
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
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/autoconf.h>
#include <linux/sysfs.h>
#include <linux/if_vlan.h>
#include "pc302_emac.h"
#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH
#include "pc302_i2c_phy.h"
#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */

/* Required for hw timestamping */
#include <net/timestamping.h>

/* Constants --------------------------------------------------------------- */

/*!
 * Data structure used to represent the DMA descriptors used for
 * packet reception and transmission.
 */
struct dma_desc
{
    u32	addr;
    u32	ctrl;
};

/*!
 * Data structure used during packet transmission.
 */
struct ring_info
{
    struct sk_buff  *skb;
    dma_addr_t	    mapping;
    void            *vaddr;
};

/*!
 * Data structure used for storage of EMAC (hardware) collected statistics.
 */
struct pc302emac_stats
{
    u32 tx_octets_31_0;
    u32 tx_octets_47_32;
    u32 tx_frames;
    u32 tx_broadcast_frames;
    u32 tx_multicast_frames;
    u32 tx_pause_frames;
    u32 tx_64_byte_frames;
    u32 tx_65_127_byte_frames;
    u32 tx_128_255_byte_frames;
    u32 tx_256_511_byte_frames;
    u32 tx_512_1023_byte_frames;
    u32 tx_1024_1518_byte_frames;
    u32 tx_greater_than_1518_byte_frames;
    u32 tx_underrun;
    u32 tx_single_collision_frames;
    u32 tx_multiple_collision_frames;
    u32 tx_excessive_collisions;
    u32 tx_late_colloisions;
    u32 tx_deferred_frames;
    u32 tx_carrier_sense_errors;
    u32 rx_octets_31_0;
    u32 rx_octets_47_32;
    u32 rx_frames;
    u32 rx_broadcast_frames;
    u32 rx_multicast_frames;
    u32 rx_pause_frames;
    u32 rx_64_byte_frames;
    u32 rx_65_127_byte_frames;
    u32 rx_128_255_byte_frames;
    u32 rx_256_511_byte_frames;
    u32 rx_512_1023_byte_frames;
    u32 rx_1024_1518_byte_frames;
    u32 rx_greater_than_1518_byte_frames;
    u32 rx_undersized_frames;
    u32 rx_oversize_frames;
    u32 rx_jabbers;
    u32 rx_frame_check_sequence_errors;
    u32 rx_length_field_frame_errors;
    u32 rx_symbol_errors;
    u32 rx_alignment_errors;
    u32 rx_resource_errors;
    u32 rx_overruns;
    u32 rx_ip_header_checksum_errors;
    u32 rx_tcp_checksum_errors;
    u32 rx_udp_checksum_errors;
};

/*!
 * Data structure used to hold driver private data.
 */
struct pc302_emac
{
    void __iomem	    *regs;

    unsigned int	    rx_tail;
    struct dma_desc	    *rx_ring;
    void		    *rx_buffers;

    unsigned int	    tx_head, tx_tail;
    struct dma_desc	    *tx_ring;
    struct ring_info	    *tx_skb;

    spinlock_t		    lock;
    struct platform_device  *pdev;
    struct net_device	    *dev;
    struct napi_struct	    napi;
    struct net_device_stats stats;
    struct pc302emac_stats  hw_stats;

    dma_addr_t		    rx_ring_dma;
    dma_addr_t		    tx_ring_dma;
    dma_addr_t		    rx_buffers_dma;

    unsigned int	    rx_pending, tx_pending;

    struct mii_bus	    *mii_bus;
    struct phy_device	    *phy_dev;
    int                     phy_probed;
    int 	            link;
    int 	            speed;
    int 	            duplex;

    int hwaddr_failure;
    
    /* Used to indicate whether hw timestamping
       of received packets is required or not */
    unsigned int            hw_timestamp_flag;

    /* For VLAN */
    struct vlan_group *vlgrp;

    /* Cache of tx buffers. */
    struct dma_pool         *tx_buf_pool;
};

/*!
 * Data structure used for phy operations.
 */
struct eth_platform_data
{
    u32 phy_mask;
    u8  phy_irq_pin;    /* PHY IRQ */
    u8  is_rmii;        /* using RMII interface? */
};

/*!
 * Default MAC address to use in this driver.
 * The EMAC hardware is read at initialisation and if no
 * valid mac address is found there, this one is used.
 *
 * Note: This is based on the picoChip OUI, see
 * http://standards.ieee.org/regauth/oui/index.shtml
 * for more information.
 */
static unsigned char default_mac_address[ETH_ALEN] =
{
    0x00, 0x15, 0xe1, 0x00, 0x00, 0x00
};

/*!
 * \brief Function return codes
 */
enum return_codes
{
    SUCCESS = 0,            /* Successful outcome */
    DROPPED_A_PACKET = 1,   /* Used by pc302emac_rx_frame()
                               to indicate a dropped frame */
    START_XMIT_ERROR = 1,   /* Used by the pc302emac_start_xmit()
                               to indicate a problem */
    BIT_IS_ONE = 1,         /* Used by the hash functions */
    BIT_IS_ZERO = 0         /* Used by the hash functions */
};

/* Globals ----------------------------------------------------------------- */
/*!
 * \brief Keep a local pointer to the netdev structure.
 *        Note: used for sysfs functions only.
 */
static struct net_device *sysfs_ndev;

/* Macros ------------------------------------------------------------------ */

/*!
 * The name used for the platform device and driver to allow Linux to match up
 * the correct ends.
 */
#define CARDNAME "pc302-emac"

/*!
 * The timeout value specified in jiffies used in Phy accesses.
 */
#define PC302_MII_WAIT_TIMEOUT  (100)

/*!
 * Macros used in the definition of the DMA Rx ring buffers.
 */
#define RX_BUFFER_SIZE		(2048)
#define RX_RING_SIZE		(32)
#define RX_RING_BYTES		(sizeof(struct dma_desc) * RX_RING_SIZE)

/*!
 * Make the IP header word-aligned (the ethernet header is 14 bytes)
 */
#define RX_OFFSET		(2)

/*!
 * Macros used in the definition of the DMA Tx ring buffers.
 */
#define TX_BUFFER_SIZE          (2048)
#define TX_RING_SIZE		(32)
#define DEF_TX_RING_PENDING	(TX_RING_SIZE - 1)
#define TX_RING_BYTES		(sizeof(struct dma_desc) * TX_RING_SIZE)

#define TX_RING_GAP(priv)					    \
	(TX_RING_SIZE - (priv)->tx_pending)

#define TX_BUFFS_AVAIL(priv)                                        \
	(((priv)->tx_tail <= (priv)->tx_head) ?			    \
	 (priv)->tx_tail + (priv)->tx_pending - (priv)->tx_head :   \
	 (priv)->tx_tail - (priv)->tx_head - TX_RING_GAP(priv))

#define NEXT_TX(n)		(((n) + 1) & (TX_RING_SIZE - 1))

#define NEXT_RX(n)		(((n) + 1) & (RX_RING_SIZE - 1))

/* minimum number of free TX descriptors before waking up TX process */
#define EMAC_TX_WAKEUP_THRESH	(TX_RING_SIZE / 4)

#define EMAC_RX_INT_FLAGS	(EMAC_ENABLE_RX_COMPLETE \
                                 | EMAC_ENABLE_RX_USED_BIT_READ	\
				 | EMAC_ENABLE_RX_OVERRUN)

#define EMAC_TX_INT_FLAGS    ( EMAC_ENABLE_TX_COMPLETE \
                             | EMAC_ENABLE_TX_BUFF_UNDERRUN \
                             | EMAC_ENABLE_RETRY_LIMIT_EXCEEDED \
                             | EMAC_ENABLE_TRANSMIT_CORRUPTION_AHB_ERROR )

#define EMAC_TX_ERROR_FLAGS  ( EMAC_TRANSMIT_UNDERRUN \
                             | EMAC_TRANSMIT_CORRUPTION_AHB_ERROR \
                             | EMAC_TRANSMIT_RETRY_LIMIT_EXCEEDED )
                             
/*!
 * Define the size (in bytes) of the received frame checksum (CRC)
 */
#define RX_FCS_SIZE		(4)

/* Prototypes--------------------------------------------------------------- */

/*!
 * Read a 32 bit value from the EMAC hardware.
 *
 * \param dev A pointer to the net_device structure.
 * \param register_offset The offset (from the base address of the EMAC) of the
 *                        particular register we wish to read.
 * \return The value read.
 */
static u32
emac_ioread32(struct net_device *dev,
              unsigned int register_offset);

/*!
 * Write a 32 bit value to the the EMAC hardware.
 *
 * \param dev A pointer to the net_device structure.
 * \param value The value to write.
 * \param register_offset The offset (from the base address of the EMAC) of the
 *                        particular register we wish to write to.
 */
static void
emac_iowrite32(struct net_device *dev,
               u32 value,
               unsigned int register_offset);

/*!
 * Set the MAC address used by the EMAC hardware.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_set_hwaddr(struct net_device *dev);

/*!
 * Read the EMAC hardware for a MAC address. We expect this to
 * have been setup by the bootloader.
 * If this MAC address is valid we'll use it. If not, the
 * default MAC address will be used.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_get_hwaddr(struct net_device *dev);

/*!
 * Read the EMAC hardware for a MAC address. We expect this to
 * have been setup by the bootloader.
 * Confirm that this matches the MAC address we programmed.If this MAC address is valid we'll use it. If not, the
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_check_hwaddr(struct net_device *dev);

#ifndef CONFIG_PC302_MICREL_VLAN_SWITCH
/*!
 * Read from an Ethernet Phy conneccted to the EMAC management bus.
 *
 * \param bus A pointer to the mii_bus structure.
 * \param mii_id The Phy ID of the Phy we wish to read from.
 * \param regnum The register number of the Phy register we wish to read from.
 * \return The value read from the Phy.
 */
static int
pc302emac_mdio_read(struct mii_bus *bus,
                    int mii_id,
                    int regnum);

/*!
 * Write to an Ethernet Phy conneccted to the EMAC management bus.
 *
 * \param bus A pointer to the mii_bus structure.
 * \param mii_id The Phy ID of the Phy we wish to write to.
 * \param regnum The register number of the Phy register we wish to write to.
 * \param value The value we wish to write to the Phy.
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_mdio_write(struct mii_bus *bus,
                     int mii_id,
                     int regnum,
		     u16 value);

/*!
 * Wait with timeout for the Phy access to complete.
 *
 * \param priv A pointer to the pc302_emac private data structure.
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_phy_busy_wait(struct pc302_emac *priv);

/*!
 * Reset the Ethernet Phy conneccted to the EMAC management bus.
 *
 * \param bus A pointer to the mii_bus structure.
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_mdio_reset(struct mii_bus *bus);

/*!
 * Handle any link changes detected by the Ethernet Phy.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_handle_link_change(struct net_device *dev);

/*!
 * Probe for Ethernet Phys on the Mii bus.
 *
 * \param dev A pointer to the net_device structure.
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_mii_probe(struct net_device *dev);

/*!
 * Initialise the Mii bus.
 *
 * \param dev A pointer to the net_device structure.
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_mii_init(struct net_device *dev);
#endif /* ! CONFIG_PC302_MICREL_VLAN_SWITCH */

/*!
 * Update the statistics counters located in the private driver structure,
 * from the statistics counters contained in the EMAC hardware.
 *
 * \param priv A pointer to the pc302_emac private data structure.
 */
static void
pc302emac_update_stats(struct pc302_emac *priv);

/*!
 * Handle 'packet transmit' related interrupts.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_tx(struct net_device *dev);

/*!
 * Obtain the nS portion of the hardware timestamp from received packets.
 *
 * \param skb A pointer to the socket buffer used for the received packet.
 *
 * \return The value of nS.
 */
static inline s32
pc302emac_get_ns_from_pkt(struct sk_buff *skb);

/*!
 * Handle received frames.
 *
 * \param dev A pointer to the net_device structure.
 * \param first_frag Index to the first fragment of a received packet
 * \param last_frag Index to the last fragment of a received packet
 *
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_rx_frame(struct net_device *dev,
                   unsigned int first_frag,
		   unsigned int last_frag);

/*!
 * Used to discard partial received frames.
 *
 * \param dev A pointer to the net_device structure.
 * \param begin Index to the first fragment of a recieved packet
 *              to be discarded
 * \param end Index to the last fragment of a received packet to
 *            to be discarded.
 */
static void
discard_partial_frame(struct net_device *dev,
                      unsigned int begin,
                      unsigned int end);

/*!
 * Handle received frames.
 * This () is called from pc302emac_poll (NAPI implementation).
 *
 * \param dev A pointer to the net_device structure.
 * \param budget Number of packets to process.
 *
 * \return The number of frames received.
 */
static int
pc302emac_rx(struct net_device *dev,
             int budget);

/*!
 * Poll for received packets (NAPI implementation).
 *
 * \param napi A pointer to the napi_struct data structure.
 * \param budget Number of packets to process.
 *
 * \return The number of packets received.
 */
static int
pc302emac_poll(struct napi_struct *napi,
               int budget);

/*!
 * EMAC driver interrupt handler.
 *
 * \param irq The irq number being handled.
 * \param dev_id Some useful device information.
 *
 * \return Zero on interrupt not handled, non zero on interrupt handled.
 */
static irqreturn_t
pc302emac_interrupt(int irq,
                    void *dev_id);

/*!
 * EMAC packet transmission.
 *
 * \param skb A pointer to a socket buffer containing the packet
 *            data for transmission.
 * \param dev A pointer to the net_device structure.
 *
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_start_xmit(struct sk_buff *skb,
                     struct net_device *dev);

/*!
 * Free EMAC DMA buffers.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_free_consistent(struct net_device *dev);

/*!
 * Allocate EMAC DMA buffers.
 *
 * \param dev A pointer to the net_device structure.
 *
 * \return Zero on success, non zero on error.
 */

static int
pc302emac_alloc_consistent(struct net_device *dev);

/*!
 * Initialise the DMA Tx and Rx descriptors.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_init_rings(struct net_device *dev);

/*!
 * Reset the EMAC hardware.
 *
 * Note: There is no software reset bit available to us, so we just
 *       write known values to certain important registers.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_reset_hw(struct net_device *dev);

/*!
 * Initialise the EMAC hardware.
 * Note: Interrupts get enabled in here.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_init_hw(struct net_device *dev);

/*!
 * Return a bit value from the hash table.
 *
 * Note: This function is called from hash_bit_value()
 *
 * \param bitnr The bit number we are interested in.
 * \param addr The MAC Address
 *
 * \return The hash index.
 */
static inline int
hash_bit_value(int bitnr,
               __u8 *addr);

/*!
 * Return the hash index value for the MAC specified address.
 *
 * \param addr The MAC Address
 *
 * \return The hash index.
 */
static int
hash_get_index(__u8 *addr);

/*!
 * Add multicast addresses to the internal multicast-hash table.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_sethashtable(struct net_device *dev);

/*!
 * Driver 'set_multicast_list' method.
 *
 * This function is used when the device flags change.
 *
 * \param dev A pointer to the net_device structure.
 */
static void
pc302emac_set_rx_mode(struct net_device *dev);

/*!
 * Driver 'open' method.
 *
 * This function is used to open the Ethernet interface for business.
 *
 * \param dev A pointer to the net_device structure.
 *
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_open(struct net_device *dev);

/*!
 * Driver 'stop' method.
 *
 * This function is used to stop the Ethernet interface.
 *
 * \param dev A pointer to the net_device structure.
 *
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_close(struct net_device *dev);

/*!
 * Driver 'get statistics' method.
 *
 * This function is used whenever an application need to obtain
 * statistic information from the Ethernet interface.
 *
 * \param dev A pointer to the net_device structure.
 *
 * \return A pointer to net_device_stats statistics.
 */
static struct
net_device_stats *pc302emac_get_stats(struct net_device *dev);

static int
pc302emac_get_settings(struct net_device *dev,
                       struct ethtool_cmd *cmd);

static int
pc302emac_set_settings(struct net_device *dev,
                       struct ethtool_cmd *cmd);

static void
pc302emac_get_drvinfo(struct net_device *dev,
	              struct ethtool_drvinfo *info);

/*!
 * Driver 'ioctl' method.
 *
 * Performs interface specific ioctl commands.
 *
 * \param dev A pointer to the net_device structure.
 * \param rq A pointer to the interface request structure
 *           used for socket ioctl's.
 * \param cmd The requested command.
 * \return A pointer to net_device_stats statistics.
 */
static int
pc302emac_ioctl(struct net_device *dev,
                struct ifreq *rq,
                int cmd);

/*!
 * Control hardware time stamping of received packets.
 *
 * Note: We only support hardware timestamping for received packets.
 *
 * \param dev A pointer to the net_device structure.
 * \param ifr A pointer to the interface request structure.
 * \param cmd The ioctl requested command.
 *
 */
static int
pc302emac_hwtstamp_ioctl(struct net_device *dev,
                         struct ifreq *ifr,
                         int cmd);

/*!
 * sysfs show method for displaying the 1588 seconds register.
 *
 * \param dev A pointer to the device structure.
 * \param attr A pointer to the device_attribute structure.
 * \param buf A pointer the character buffer to use for returning
 *            the requested data.
 * \return The number of characters in buf.
 */
static ssize_t
pc302emac_sysfs_1588_seconds_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

/*!
 * sysfs store method for updating the 1588 seconds register.
 *
 * \param dev A pointer to the device structure.
 * \param attr A pointer to the device_attribute structure.
 * \param buf A pointer the character buffer providing the
 *            user data.
 * \param count The number of characters in buf.
 * \return The number of characters in buf.
 */
static ssize_t
pc302emac_sysfs_1588_seconds_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

/*!
 * sysfs show method for displaying the 1588 nano seconds register.
 *
 * \param dev A pointer to the device structure.
 * \param attr A pointer to the device_attribute structure.
 * \param buf A pointer the character buffer to use for returning
 *            the requested data.
 * \return The number of characters in buf.
 */
static ssize_t
pc302emac_sysfs_1588_nano_seconds_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf);

/*!
 * sysfs store method for updating the 1588 nano seconds register.
 *
 * \param dev A pointer to the device structure.
 * \param attr A pointer to the device_attribute structure.
 * \param buf A pointer the character buffer providing the
 *            user data.
 * \param count The number of characters in buf.
 * \return The number of characters in buf.
 */
static ssize_t
pc302emac_sysfs_1588_nano_seconds_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf,
                                        size_t count);

/*!
 * sysfs store method for updating the 1588 timer adjust.
 *
 * Note: This accesses a write only register.
 *
 * \param dev A pointer to the device structure.
 * \param attr A pointer to the device_attribute structure.
 * \param buf A pointer the character buffer providing the
 *            user data.
 * \param count The number of characters in buf.
 * \return The number of characters in buf.
 */
static ssize_t
pc302emac_sysfs_1588_timer_adjust_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf,
                                        size_t count);

/*!
 * sysfs show method for displaying the 1588 timer increment register.
 *
 * \param dev A pointer to the device structure.
 * \param attr A pointer to the device_attribute structure.
 * \param buf A pointer the character buffer to use for returning
 *            the requested data.
 * \return The number of characters in buf.
 */
static ssize_t
pc302emac_sysfs_1588_timer_increment_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf);


/*!
 * sysfs store method for updating the 1588 timer increment register.
 *
 * \param dev A pointer to the device structure.
 * \param attr A pointer to the device_attribute structure.
 * \param buf A pointer the character buffer providing the
 *            user data.
 * \param count The number of characters in buf.
 * \return The number of characters in buf.
 */
static ssize_t
pc302emac_sysfs_1588_timer_increment_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count);

/*!
 * Add the sysfs files.
 *
 * \param pdev A pointer to the platform_device structure.
 *
 * \return Zero on success, non zero on error.
 */
static int
pc302emac_sysfs_add(struct platform_device *pdev);

/*!
 * Remove the sysfs files.
 *
 * \param pdev A pointer to the platform_device structure.
 */
static void
pc302emac_sysfs_remove(struct platform_device *pdev);

/*!
 * Platform driver 'probe' method.
 *
 * \param pdev A pointer to the platform_device structure.
 *
 * \return Zero on success, non zero on error.
 */
static int
emac_drv_probe(struct platform_device *pdev);

/*!
 * Platform driver 'remove' method.
 *
 * \param pdev A pointer to the platform_device structure.
 *
 * \return Zero on success, non zero on error.
 */
static int
emac_drv_remove(struct platform_device *pdev);

static void emac_vlan_rx_register(struct net_device *dev, struct vlan_group *grp);

static void emac_vlan_rx_kill_vid(struct net_device *dev, uint16_t vid);

/*!
 * Module initialisation function.
 *
 * \return Zero on success, non zero on error.
 */
static int
emac_init_module(void);

/*!
 * Module exit function.
 */
static void
emac_cleanup_module(void);

/* Functions --------------------------------------------------------------- */

static u32
emac_ioread32(struct net_device *dev,
              unsigned int register_offset)
{
    struct pc302_emac *priv = netdev_priv(dev);
    void __iomem *p = priv->regs + register_offset;

    return ioread32(p);
}

static void
emac_iowrite32(struct net_device *dev,
               u32 value,
               unsigned int register_offset)
{
    struct pc302_emac *priv = netdev_priv(dev);
    void __iomem *p = priv->regs + register_offset;

    iowrite32(value, p);
}

static void
pc302emac_set_hwaddr(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    u32 bottom = 0;
    u32 top = 0;
    u32 old_bottom = 0;
    u32 old_top = 0;

    /* Obtain bytes 1-4 of the MAC address from the EMAC hardware */
    old_bottom = emac_ioread32(dev, EMAC_SPEC_ADDR_1_BOT_31_0_REG_OFFSET);
    
    /* Obtain bytes 5-6 of the MAC address from the EMAC hardware */
    old_top = emac_ioread32(dev, EMAC_SPEC_ADDR_1_TOP_47_32_REG_OFFSET);
    
    printk("MAC to be written to HW is %02X:%02X:%02X:%02X:%02X:%02X in pid=%d (%s)\n",
        dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
        dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5],
        current->pid, current->comm);
        
    printk("MAC bottom register was %08X\n", old_bottom);
    printk("MAC top register was %04X\n", old_top);
    
    /* Obtain the least significant 4 bytes of the MAC address from dev */
    bottom = dev->dev_addr[3] << 24 |
             dev->dev_addr[2] << 16 |
             dev->dev_addr[1] << 8  |
             dev->dev_addr[0];

    /* Obtain the most sognificant 2 bytes of the MAC address from dev */
    top = dev->dev_addr[5] << 8 |
          dev->dev_addr[4];

    printk("MAC bottom register will be %08X\n", bottom);
    
    /* Program the EMAC Specific Address #1 'bottom' register */
    emac_iowrite32(dev, bottom, EMAC_SPEC_ADDR_1_BOT_31_0_REG_OFFSET);

    printk("MAC top register will be %04X\n", top);
    
    /* Program the EMAC Specific Address #1 'top' register */
    emac_iowrite32(dev, (u32)top, EMAC_SPEC_ADDR_1_TOP_47_32_REG_OFFSET);
        
    show_stack(0, NULL);
    
    priv->hwaddr_failure = 0;
}

static void
pc302emac_get_hwaddr(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    u32 bottom = 0;
    u16 top = 0;
    u8 addr[ETH_ALEN];

    /* Obtain bytes 1-4 of the MAC address from the EMAC hardware */
    bottom = emac_ioread32(dev, EMAC_SPEC_ADDR_1_BOT_31_0_REG_OFFSET);
    
    printk("MAC bottom register was %08X\n", bottom);

    /* Obtain bytes 5-6 of the MAC address from the EMAC hardware */
    top = emac_ioread32(dev, EMAC_SPEC_ADDR_1_TOP_47_32_REG_OFFSET);

    printk("MAC top register was %04X\n", top);

    addr[0] = bottom & 0xff;
    addr[1] = (bottom >> 8) & 0xff;
    addr[2] = (bottom >> 16) & 0xff;
    addr[3] = (bottom >> 24) & 0xff;
    addr[4] = top & 0xff;
    addr[5] = (top >> 8) & 0xff;

    printk("MAC read from HW is %02X:%02X:%02X:%02X:%02X:%02X in pid=%d (%s)\n",
        addr[0], addr[1], addr[2],
        addr[3], addr[4], addr[5],
        current->pid, current->comm);
    
    if (is_valid_ether_addr(addr))
    {
    /* We have read a valid MAC address from the EMAC hardware,
           copy it into the net_device structure */
        memcpy(dev->dev_addr, addr, ETH_ALEN);
    }
    else
    {
    /* We have read an invalid MAC address from the EMAC hardware,
           use the default instead */
        printk(KERN_INFO "%s: invalid MAC address read from hardware, "
                         "using default\n", dev->name);
    memcpy(dev->dev_addr, default_mac_address, ETH_ALEN);
    }
    
    printk("MAC saved in struct net_device is %02X:%02X:%02X:%02X:%02X:%02X\n",
        dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
        dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);
        
    show_stack(0, NULL);
    
    priv->hwaddr_failure = 0;
}

static void
pc302emac_check_hwaddr(struct net_device *dev)
{
    static int last_pid = -1;
    struct pc302_emac *priv = netdev_priv(dev);
    u32 bottom = 0;
    u16 top = 0;
    u8 addr[ETH_ALEN];

    /* Obtain bytes 1-4 of the MAC address from the EMAC hardware */
    bottom = emac_ioread32(dev, EMAC_SPEC_ADDR_1_BOT_31_0_REG_OFFSET);
    
    /* Obtain bytes 5-6 of the MAC address from the EMAC hardware */
    top = emac_ioread32(dev, EMAC_SPEC_ADDR_1_TOP_47_32_REG_OFFSET);

    addr[0] = bottom & 0xff;
    addr[1] = (bottom >> 8) & 0xff;
    addr[2] = (bottom >> 16) & 0xff;
    addr[3] = (bottom >> 24) & 0xff;
    addr[4] = top & 0xff;
    addr[5] = (top >> 8) & 0xff;

    if (memcmp(dev->dev_addr, addr, ETH_ALEN))
    {
        printk("MAC saved in struct net_device is %02X:%02X:%02X:%02X:%02X:%02X"
               " but the MAC in HW(%p) is %02X:%02X:%02X:%02X:%02X:%02X lastpid=%d thispid=%d (%s)\n",
               dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
               dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5],
               priv->regs,
               addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],
               last_pid, current->pid, current->comm);
               
        priv->hwaddr_failure = 1;
    }
    
    last_pid = current->pid;
}

#ifndef CONFIG_PC302_MICREL_VLAN_SWITCH
static int
pc302emac_mdio_read(struct mii_bus *bus,
                    int mii_id,
                    int regnum)
{
    struct pc302_emac *priv = bus->priv;

    int ret = SUCCESS;

    u32 write_data = EMAC_PHY_READ;
    u32 value = 0;

    /* Mask input parameters */
    mii_id &= EMAC_PHY_ID_MASK;
    regnum &= EMAC_PHY_REG_MASK;

    /* Create the data to write to the EMAC (Phy) */
    write_data |= ((mii_id << EMAC_PHY_ID_SHIFT) |
                   (regnum << EMAC_PHY_REG_SHIFT));

    /* Write to the EMAC (Phy) */
    emac_iowrite32(priv->dev, write_data, EMAC_PHY_MAINTAIN_REG_OFFSET);

    /* Ensure the write happens */
    wmb();

    /* Wait for the phy access to complete */
    ret = pc302emac_phy_busy_wait(priv);

    BUG_ON (ret != SUCCESS);

    /* Read back the data obtained from the phy */
    value = emac_ioread32(priv->dev, EMAC_PHY_MAINTAIN_REG_OFFSET);
    value &= EMAC_PHY_DATA_MASK;

    return (int)value;
}

static int
pc302emac_mdio_write(struct mii_bus *bus,
                     int mii_id,
                     int regnum,
		     u16 value)
{
    struct pc302_emac *priv = bus->priv;

    int ret = SUCCESS;

    u32 write_data = EMAC_PHY_WRITE;

    /* Mask input parameters */
    mii_id &= EMAC_PHY_ID_MASK;
    regnum &= EMAC_PHY_REG_MASK;

    /* Create the data to write to the EMAC (Phy) */
    write_data |= ((mii_id << EMAC_PHY_ID_SHIFT) |
                   (regnum << EMAC_PHY_REG_SHIFT) | value);

    /* Write to the EMAC (Phy) */
    emac_iowrite32(priv->dev, write_data, EMAC_PHY_MAINTAIN_REG_OFFSET);

    /* Ensure the write happens */
    wmb();

    /* Wait for the phy access to complete */
    ret = pc302emac_phy_busy_wait(priv);

    BUG_ON (ret != SUCCESS);

    return ret;
}

/* Wait with timeout for the PHY busy signal to be reset */
static int
pc302emac_phy_busy_wait(struct pc302_emac *priv)
{
    unsigned long timeout_jiffies = jiffies + PC302_MII_WAIT_TIMEOUT;

    int ret = SUCCESS;

    u32 status = 0;

    while (1)
    {
        /* Check for time out */
        if (time_after(jiffies, timeout_jiffies))
        {
            /* Oops we might have timed out.  Check one last time to handle
             * the case where the CPU was busy elsewhere for a long time
             * between the previous check and the call to time_after.
             */
            status = emac_ioread32(priv->dev, EMAC_NETWORK_STATUS_REG_OFFSET);
            if ((status & EMAC_PHY_MANAGEMENT_IDLE) == 0)
            {
                ret = -ETIMEDOUT;
            }
            break;
        }

        /* Check for phy access complete */
        status = emac_ioread32(priv->dev, EMAC_NETWORK_STATUS_REG_OFFSET);
        if (status & EMAC_PHY_MANAGEMENT_IDLE)
        {
            /* Phy access has completed */
            break;
        }

        cpu_relax();
    }

    return ret;
}

static int
pc302emac_mdio_reset(struct mii_bus *bus)
{
    return SUCCESS;
}
#endif /* ! CONFIG_PC302_MICREL_VLAN_SWITCH */

static void
pc302emac_handle_link_change(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    struct phy_device *phydev = priv->phy_dev;

    unsigned long flags = 0;

    int status_change = 0;

    u32 config = 0;

    spin_lock_irqsave(&priv->lock, flags);

    if (phydev->link)
    {
	if ((priv->speed != phydev->speed) || (priv->duplex != phydev->duplex))
        {
	    config = emac_ioread32(dev, EMAC_NETWORK_CFG_REG_OFFSET);
	    config &= ~(EMAC_SPEED_100_MBPS | EMAC_FULL_DUPLEX);

	    if (phydev->duplex)
            {
		config |= EMAC_FULL_DUPLEX;
            }

            if (phydev->speed == SPEED_100)
            {
		config |= EMAC_SPEED_100_MBPS;
            }

            emac_iowrite32(dev, config, EMAC_NETWORK_CFG_REG_OFFSET);

	    priv->speed = phydev->speed;
	    priv->duplex = phydev->duplex;
	    status_change = 1;
	}
    }

    if (phydev->link != priv->link)
    {
	if (!phydev->link)
        {
	    priv->speed = 0;
	    priv->duplex = -1;
	}
	priv->link = phydev->link;

	status_change = 1;
    }

    spin_unlock_irqrestore(&priv->lock, flags);

    if (status_change)
    {
	if (phydev->link)
        {
	    printk(KERN_INFO "%s: link up (%d/%s)\n",
			      dev->name, phydev->speed,
			      DUPLEX_FULL == phydev->duplex ? "Full":"Half");
        }
	else
        {
	    printk(KERN_INFO "%s: link down\n", dev->name);
        }
    }
}

#ifndef CONFIG_PC302_MICREL_VLAN_SWITCH
static int
pc302emac_mii_probe(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    struct phy_device *phydev = NULL;
    struct eth_platform_data *pdata = NULL;

    int phy_addr = 0;

    /* find the first phy */
    for (phy_addr = 0; phy_addr < PHY_MAX_ADDR; phy_addr++)
    {
    	if (priv->mii_bus->phy_map[phy_addr])
        {
	    phydev = priv->mii_bus->phy_map[phy_addr];
	    break;
	}
    }

    if (phydev)
    {
        pdata = priv->pdev->dev.platform_data;

        /* attach the mac to the phy */
        if (pdata && pdata->is_rmii)
        {
            phydev = phy_connect(dev, phydev->dev.bus_id,
                                 &pc302emac_handle_link_change,
                                 0, PHY_INTERFACE_MODE_RMII);
        }
        else
        {
            phydev = phy_connect(dev, phydev->dev.bus_id,
                                 &pc302emac_handle_link_change,
                                 0, PHY_INTERFACE_MODE_MII);
        }

        if (IS_ERR(phydev))
        {
            printk(KERN_ERR "%s: could not attach to Phy\n", dev->name);
            return PTR_ERR(phydev);
        }

        /* mask with MAC supported features */
        phydev->supported &= PHY_BASIC_FEATURES;

        phydev->advertising = phydev->supported;

        priv->link = 0;
        priv->speed = 0;
        priv->duplex = -1;
        priv->phy_dev = phydev;
    }
    else
    {
        u32 config;
	printk(KERN_ERR "%s: no Phy found, assuming direct MII\n", dev->name);
        priv->link = 1;
        priv->speed = SPEED_100;
        priv->duplex = DUPLEX_FULL;
        priv->phy_dev = NULL;

        config = emac_ioread32(dev, EMAC_NETWORK_CFG_REG_OFFSET);
        config |= EMAC_SPEED_100_MBPS | EMAC_FULL_DUPLEX;
        emac_iowrite32(dev, config, EMAC_NETWORK_CFG_REG_OFFSET);
    }

    priv->phy_probed = 1;

    return SUCCESS;
}

static int
pc302emac_mii_init(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    struct eth_platform_data *pdata = NULL;

    int ret = -ENXIO;
    int i = 0;

    u32 control = 0;

    /* Enable managment port */
    control = emac_ioread32(dev, EMAC_NETWORK_CTRL_REG_OFFSET);
    control |= EMAC_MDIO_ENABLE;
    emac_iowrite32(dev, control, EMAC_NETWORK_CTRL_REG_OFFSET);

    priv->mii_bus->name = "pc302_mii_bus";
    priv->mii_bus->read = &pc302emac_mdio_read;
    priv->mii_bus->write = &pc302emac_mdio_write;
    priv->mii_bus->reset = &pc302emac_mdio_reset;
    snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "%02x", priv->pdev->id);
    priv->mii_bus->priv = priv;
    priv->mii_bus->parent = &priv->dev->dev;

    pdata = priv->pdev->dev.platform_data;

    if (pdata)
    {
	priv->mii_bus->phy_mask = pdata->phy_mask;
    }

    priv->mii_bus->irq = kmalloc(sizeof(int)*PHY_MAX_ADDR, GFP_KERNEL);
    if (!priv->mii_bus->irq)
    {

        printk(KERN_ERR "%s: could not allocate memory for mii_bus.irq\n",
               dev->name);
        ret = -ENOMEM;
	goto err_out;
    }

    /* Phy interrupts not supported */
    for (i = 0; i < PHY_MAX_ADDR; i++)
    {
	priv->mii_bus->irq[i] = PHY_POLL;
    }

    platform_set_drvdata(priv->dev, &priv->mii_bus);

    if (mdiobus_register(priv->mii_bus))
    {
	printk(KERN_ERR "%s: could not register mdio bus\n", dev->name);
        ret = -ENXIO;
        goto err_out_free_mii_bus_irq;
    }

    if (pc302emac_mii_probe(priv->dev) != 0)
    {
	printk(KERN_ERR "%s: pc302emac_mii_probe() failed\n", dev->name);
        ret = -ENXIO;
        goto err_out_unregister_mdio_bus;
    }

    return SUCCESS;

err_out_unregister_mdio_bus:
    mdiobus_unregister(priv->mii_bus);

err_out_free_mii_bus_irq:
    kfree(priv->mii_bus->irq);

err_out:
    return ret;
}

#else /* CONFIG_PC302_MICREL_VLAN_SWITCH */

static int
pc302emac_i2c_probe(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    struct phy_device *phydev = NULL;

    phydev = i2c_phy_connect(dev, "", &pc302emac_handle_link_change,
                             0, PHY_INTERFACE_MODE_MII);

    if (IS_ERR(phydev))
    {
	printk(KERN_ERR "%s: could not attach to Phy\n", dev->name);
	return PTR_ERR(phydev);
    }

    /* mask with MAC supported features */
    phydev->supported &= PHY_BASIC_FEATURES;

    phydev->advertising = phydev->supported;

    priv->link = 0;
    priv->speed = 0;
    priv->duplex = -1;
    priv->phy_dev = phydev;

    return SUCCESS;
}

#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */

static void
pc302emac_update_stats(struct pc302_emac *priv)
{
    u32 __iomem *reg = priv->regs + EMAC_OCTETS_TX_31_0_REG_OFFSET;
    u32 *p = &priv->hw_stats.tx_octets_31_0;
    u32 *end = &priv->hw_stats.rx_udp_checksum_errors + 1;

    for(; p < end; p++, reg++)
    {
        /* Read from the EMAC status registers (reg) and update the statistics
           counters (*p) in the private data structure */
        *p += ioread32(reg);
    }
}

static void
pc302emac_tx(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);

    unsigned int tail = 0;
    unsigned int head = 0;

    unsigned int bufferReset = 0;

    u32 status = 0;
    u32 control = 0;

    status = emac_ioread32(dev, EMAC_TX_STATUS_REG_OFFSET);
    emac_iowrite32(dev, status, EMAC_TX_STATUS_REG_OFFSET);

    if (status & EMAC_TRANSMIT_HRESP_NOT_OK)
    {
        printk(KERN_ERR "%s: TX hresp not OK\n", dev->name);
    }

    if (status & EMAC_TX_ERROR_FLAGS)
    {
        int i;

        bufferReset = 1;

        if (status & EMAC_TRANSMIT_UNDERRUN)
        {
            printk(KERN_ERR "%s: Tx underrun, resetting buffers\n", dev->name);
        }

        if (status & EMAC_TRANSMIT_CORRUPTION_AHB_ERROR)
        {
            printk(KERN_ERR "%s: Tx corruption (bus error), resetting buffers\n", dev->name);
        }

        if (status & EMAC_TRANSMIT_RETRY_LIMIT_EXCEEDED)
        {
            printk(KERN_ERR "%s: Tx retry limit exceeded, resetting buffers\n", dev->name);
        }

        /* If we're resetting the head and tail pointers for the ring then we
         * have to tell the transmitter to reset its next available TX buffer
         * descriptor to the beginning of the TX buffer descriptor list.  The
         * only way we can do that is by disabling the transmitter.
         */
        control = emac_ioread32(dev, EMAC_NETWORK_CTRL_REG_OFFSET);
        control &= ~EMAC_TX_ENABLE;
        emac_iowrite32(dev, control, EMAC_NETWORK_CTRL_REG_OFFSET);
        
        emac_iowrite32(dev, 0xFFFFFFFF, EMAC_TX_STATUS_REG_OFFSET);
        
	head = priv->tx_head;

	/*Mark all the buffer as used to avoid sending a lost buffer*/
	for (i = 0; i < TX_RING_SIZE; i++)
        {
	    priv->tx_ring[i].ctrl = EMAC_TX_DESC_HOST_OWN;
        }

        /* free transmit buffer in upper layer*/
	for (tail = priv->tx_tail; tail != head; tail = NEXT_TX(tail))
        {
	    struct ring_info *rp = &priv->tx_skb[tail];
	    struct sk_buff *skb = rp->skb;

	    BUG_ON(skb == NULL);

	    rmb();

            dma_pool_free(priv->tx_buf_pool, rp->vaddr, rp->mapping);

	    rp->skb = NULL;
	    dev_kfree_skb_irq(skb);
	}

	priv->tx_head = priv->tx_tail = 0;

        /* Now that the SW and HW agree where the next TX descriptor should
         * be read from, we can restart the transmitter.
         */    
        control = emac_ioread32(dev, EMAC_NETWORK_CTRL_REG_OFFSET);
        control |= EMAC_TX_ENABLE;
        emac_iowrite32(dev, control, EMAC_NETWORK_CTRL_REG_OFFSET);
        
    }

    if (!(status & EMAC_TRANSMIT_COMPLETE))
    {
        if (netif_queue_stopped(priv->dev) && bufferReset)
        {
            printk(KERN_ERR "%s: TX buffer reset with queue stopped.  Resuming queue\n", dev->name);
            netif_wake_queue(priv->dev);
        }

	/*
	 * This may happen when a buffer becomes complete
	 * between reading the ISR and scanning the
	 * descriptors.  Nothing to worry about.
	 */
	return;
    }

    head = priv->tx_head;
    for (tail = priv->tx_tail; tail != head; tail = NEXT_TX(tail))
    {
        struct ring_info *rp = &priv->tx_skb[tail];
	struct sk_buff *skb = rp->skb;
	u32 bufstat;

	BUG_ON(skb == NULL);

	rmb();
	bufstat = priv->tx_ring[tail].ctrl;

	if (!(bufstat & EMAC_TX_DESC_HOST_OWN))
        {
	    break;
        }

        dma_pool_free(priv->tx_buf_pool, rp->vaddr, rp->mapping);

	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len;
	rp->skb = NULL;
	dev_kfree_skb_irq(skb);
    }

    /* Make sure that any other pending packets get sent by triggering the
     * transmitter again. */
    control = emac_ioread32(dev, EMAC_NETWORK_CTRL_REG_OFFSET);
    control |= EMAC_START_TX;
    emac_iowrite32(dev, control, EMAC_NETWORK_CTRL_REG_OFFSET);

    priv->tx_tail = tail;
    if (netif_queue_stopped(priv->dev) &&
        (TX_BUFFS_AVAIL(priv) > EMAC_TX_WAKEUP_THRESH))
    {
	netif_wake_queue(priv->dev);
    }
}

static inline s32
pc302emac_get_ns_from_pkt(struct sk_buff *skb)
{
    /* Assuming the mac inserts the ns portion of the hardware
       timestamp in little endian format */
    return *(s32 *)(skb->data + (skb->len - RX_FCS_SIZE));
}

static int
pc302emac_rx_frame(struct net_device *dev,
                   unsigned int first_frag,
		   unsigned int last_frag)
{
    struct pc302_emac *priv = netdev_priv(dev);
    struct sk_buff *skb = NULL;

    unsigned int len = 0;
    unsigned int frag = 0;
    unsigned int offset = 0;
    unsigned int frag_len = RX_BUFFER_SIZE;
    u32 csum_status = 0;
    unsigned short vlan_tag = 0;

    ktime_t hw_timestamp;
    s32 sec_from_hardware = 0;
    s32 ns_from_hardware = 0;

    len = priv->rx_ring[last_frag].ctrl & EMAC_RX_DESC_LENGTH_MASK;

    skb = dev_alloc_skb(len + RX_OFFSET);
    if (!skb)
    {
	priv->stats.rx_dropped++;
	for (frag = first_frag; ; frag = NEXT_RX(frag))
        {
	    priv->rx_ring[frag].addr &= ~(EMAC_RX_DESC_HOST_OWN);
	    if (frag == last_frag)
            {
		break;
            }
	}
	wmb();
	return DROPPED_A_PACKET;
    }

    skb_reserve(skb, RX_OFFSET);

    /* Check the status of the Rx checksum offload calculation */
    csum_status =
        priv->rx_ring[last_frag].ctrl & EMAC_RX_DESC_CSUM_OFFLOAD_MASK;
    if (!csum_status)
    {
        /* Neither the IP header nor the TCP/UDP checksum was checked */
        skb->ip_summed = CHECKSUM_NONE;
    }
    else
    {
        /* Checksums checked ok */
        skb->ip_summed = CHECKSUM_UNNECESSARY;
    }

    skb_put(skb, len);

    for (frag = first_frag; ; frag = NEXT_RX(frag))
    {
	if (offset + frag_len > len)
        {
            BUG_ON(frag != last_frag);
	    frag_len = len - offset;
	}

        skb_copy_to_linear_data_offset(skb, offset,
				       (priv->rx_buffers +
				       (RX_BUFFER_SIZE * frag)),
				       frag_len);
	offset += RX_BUFFER_SIZE;
	priv->rx_ring[frag].addr &= ~(EMAC_RX_DESC_HOST_OWN);
	wmb();

	if (frag == last_frag)
        {
	    break;
        }
    }

    priv->stats.rx_packets++;
    priv->stats.rx_bytes += len;
    priv->dev->last_rx = jiffies;

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
    if (priv->rx_ring[last_frag].ctrl & EMAC_RX_DESC_VLAN_TAG) {

        /* Ignore the return value. If the hardware tells us it is VLAN
         * and it is not, something much more serious has occurred.
         */
        vlan_get_tag(skb, &vlan_tag);

        /* Shift the MAC addresses to make it into an normal packet */
        memmove(skb->data + VLAN_HLEN, skb->data, ETH_ALEN + ETH_ALEN);
        skb_pull(skb, VLAN_HLEN);
    }
    
    if (priv->hw_timestamp_flag == HWTSTAMP_FILTER_ALL)
    {
        /* We have been asked to include the hardware timestamp of the
           received packet in the socket buffer */

        /* Read seconds value from hardware */
        sec_from_hardware =
        (s32)emac_ioread32(dev, EMAC_1588_TIMER_SECONDS_REG_OFFSET);

        /* Read nS value from the hardware */
        ns_from_hardware =
        (s32)emac_ioread32(dev, EMAC_1588_TIMER_NANO_SECONDS_REG_OFFSET);

        /* Re read seconds value from hardware */
        hw_timestamp.tv.sec =
        (s32)emac_ioread32(dev, EMAC_1588_TIMER_SECONDS_REG_OFFSET);

        /* Read the nS portion of the timestamp from
           the received packet */
        hw_timestamp.tv.nsec = pc302emac_get_ns_from_pkt(skb);

        /* Check for a seconds roll over */
        if (sec_from_hardware != hw_timestamp.tv.sec)
        {
	    /* We have a seconds roll over, therefore
               use the initial value read. */
            hw_timestamp.tv.sec = sec_from_hardware;
	}
        else if (ns_from_hardware < hw_timestamp.tv.nsec)
        {
            /* Note: This 'adjustment' to the seconds value relies
                     on the fact that less than 1 second has elapsed
                     between the packet being received and the time
                     now. */
            hw_timestamp.tv.sec--;
        }

        /* Set hardware timestamp value in the socket buffer */
        skb_hwtstamp_set(skb, hw_timestamp);
    }

    skb->protocol = eth_type_trans(skb, priv->dev);

    if ( (priv->rx_ring[last_frag].ctrl & EMAC_RX_DESC_VLAN_TAG)  && priv->vlgrp) {
        /* Pass the packet to the VLAN layer */
        vlan_hwaccel_receive_skb(skb, priv->vlgrp, vlan_tag);
    }
    else
    {
        /* Pass received packet up to higher networking layers */
        netif_receive_skb(skb);
    }

    return SUCCESS;
}

static void
discard_partial_frame(struct net_device *dev,
                      unsigned int begin,
                      unsigned int end)
{
    struct pc302_emac *priv = netdev_priv(dev);

    unsigned int frag = 0;

    for (frag = begin; frag != end; frag = NEXT_RX(frag))
    {
	priv->rx_ring[frag].addr &= ~(EMAC_RX_DESC_HOST_OWN);
    }
    wmb();

    /*
     * When this happens, the hardware stats registers for
     * whatever caused this is updated, so we don't have to record
     * anything.
     */
}

static int
pc302emac_rx(struct net_device *dev,
             int budget)
{
    struct pc302_emac *priv = netdev_priv(dev);

    int received = 0;
    int first_frag = -1;
    int dropped = 0;

    unsigned int tail = priv->rx_tail;

    for (; budget > 0; tail = NEXT_RX(tail))
    {
	u32 addr, ctrl;

	rmb();
	addr = priv->rx_ring[tail].addr;
	ctrl = priv->rx_ring[tail].ctrl;

	if (!(addr & EMAC_RX_DESC_HOST_OWN))
        {
	    break;
        }

	if (ctrl & EMAC_RX_DESC_START_OF_FRAME)
        {
	    if (first_frag != -1)
            {
		discard_partial_frame(dev, first_frag, tail);
            }
	    first_frag = tail;
	}

	if (ctrl & EMAC_RX_DESC_END_OF_FRAME)
        {
	    BUG_ON(first_frag == -1);

	    dropped = pc302emac_rx_frame(dev, first_frag, tail);
	    first_frag = -1;
	    if (!dropped)
            {
	        received++;
		budget--;
	    }
	}
    }

    if (first_frag != -1)
    {
	priv->rx_tail = first_frag;
    }
    else
    {
	priv->rx_tail = tail;
    }

    return received;
}

static int
pc302emac_poll(struct napi_struct *napi,
               int budget)
{
    struct pc302_emac *priv = container_of(napi, struct pc302_emac, napi);
    struct net_device *dev = priv->dev;

    int work_done = 0;

    u32 status = 0;

    status = emac_ioread32(dev, EMAC_RX_STATUS_REG_OFFSET);
    emac_iowrite32(dev, status, EMAC_RX_STATUS_REG_OFFSET);

    work_done = 0;
    if (!status)
    {
	/*
	 * This may happen if an interrupt was pending before
	 * this function was called last time, and no packets
	 * have been received since.
	 */
	netif_rx_complete(dev, napi);
	goto out;
    }

    if (!(status & EMAC_RX_BUFFER_NOT_AVAIL) &&
        !(status & EMAC_FRAME_RECEIVED))
    {
	dev_warn(&priv->pdev->dev,
	         "No RX buffers complete, status = %02lx\n",
		 (unsigned long)status);
	netif_rx_complete(dev, napi);
	goto out;
    }

    work_done = pc302emac_rx(dev, budget);
    if (work_done < budget)
    {
	netif_rx_complete(dev, napi);
    }

    /*
     * We've done what we can to clean the buffers. Make sure we
     * get notified when new packets arrive.
     */
out:
    emac_iowrite32(dev, EMAC_RX_INT_FLAGS, EMAC_INT_ENABLE_REG_OFFSET );

    return work_done;
}

static irqreturn_t
pc302emac_interrupt(int irq,
                    void *dev_id)
{
    struct net_device *dev = dev_id;
    struct pc302_emac *priv = netdev_priv(dev);

    u32 status = 0;

    /* Let find out why we are here */
    status = emac_ioread32(dev, EMAC_INT_STATUS_REG_OFFSET);

    if (unlikely(!status))
    {
	/* False alarm */
        return IRQ_NONE;
    }

    spin_lock(&priv->lock);

    while (status)
    {
	/* close possible race with dev_close */
	if (unlikely(!netif_running(dev)))
        {
	    /* Disable all interrupts */
            printk("Disable all interrupts in irq handler due to close\n");
            emac_iowrite32(dev, 0xFFFFFFFF, EMAC_INT_DISABLE_REG_OFFSET);
	    break;
	}

        if (status & EMAC_RX_INT_FLAGS)
        {
            /* We have an Rx related interrupt */
            if (netif_rx_schedule_prep(dev, &priv->napi))
            {
	        /*
	         * There's no point taking any more interrupts
	         * until we have processed the buffers
	         */
	        emac_iowrite32(dev, EMAC_RX_INT_FLAGS,
                               EMAC_INT_DISABLE_REG_OFFSET);
	        __netif_rx_schedule(dev, &priv->napi);
	    }
        }

        if (status & EMAC_TX_INT_FLAGS)
        {
            /* We have a Tx related interrupt */
            pc302emac_tx(dev);
        }

        if (status & EMAC_ENABLE_HRESP_NOT_OK)
        {
	    printk(KERN_ERR "%s: DMA bus error: HRESP not OK\n", dev->name);
            BUG();
        }

        /* Check for further EMAC interrupts before exiting */
        status = emac_ioread32(dev, EMAC_INT_STATUS_REG_OFFSET);
    }

    spin_unlock(&priv->lock);

    return IRQ_HANDLED;
}

static int
pc302emac_start_xmit(struct sk_buff *skb,
                     struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);

    dma_addr_t mapping = 0;

    unsigned int len = 0;
    unsigned int entry = 0;

    u32 ctrl = 0;
    u32 control = 0;

    len = skb->len;
    spin_lock_irq(&priv->lock);

    if (!priv->hwaddr_failure)
    {
        pc302emac_check_hwaddr(dev);
    }

    /* This is a hard error, log it. */
    if (TX_BUFFS_AVAIL(priv) < 1)
    {
        netif_stop_queue(dev);
	spin_unlock_irq(&priv->lock);

        printk(KERN_ERR "%s: BUG! Tx Ring full when queue awake!\n",
                        dev->name);
	return START_XMIT_ERROR;
    }

    entry = priv->tx_head;

    priv->tx_skb[entry].vaddr =
        dma_pool_alloc(priv->tx_buf_pool, GFP_ATOMIC, &mapping );
    memcpy(priv->tx_skb[entry].vaddr, skb->data, len);
    priv->tx_skb[entry].skb = skb;
    priv->tx_skb[entry].mapping = mapping;

    ctrl = len & EMAC_TX_BUFFER_LENGTH_MASK;
    ctrl |= EMAC_TX_LAST_BUFFER;
    if (entry == (TX_RING_SIZE - 1))
    {
	ctrl |= EMAC_TX_DESC_WRAP;
    }

    priv->tx_ring[entry].addr = priv->tx_skb[entry].mapping;
    priv->tx_ring[entry].ctrl = ctrl;
    wmb();

    entry = NEXT_TX(entry);
    priv->tx_head = entry;

    control = emac_ioread32(dev, EMAC_NETWORK_CTRL_REG_OFFSET);
    control |= EMAC_START_TX;
    emac_iowrite32(dev, control, EMAC_NETWORK_CTRL_REG_OFFSET);

    if (TX_BUFFS_AVAIL(priv) < 1)
    {
	netif_stop_queue(dev);
    }

    spin_unlock_irq(&priv->lock);

    dev->trans_start = jiffies;

    return SUCCESS;
}

static void
pc302emac_free_consistent(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);

    if (priv->tx_skb)
    {
	kfree(priv->tx_skb);
	priv->tx_skb = NULL;
    }

    if (priv->rx_ring)
    {
        dma_free_coherent(&priv->pdev->dev, RX_RING_BYTES,
		          priv->rx_ring, priv->rx_ring_dma);
	priv->rx_ring = NULL;
    }

    if (priv->tx_ring)
    {
        dma_free_coherent(&priv->pdev->dev, TX_RING_BYTES,
		          priv->tx_ring, priv->tx_ring_dma);
	priv->tx_ring = NULL;
    }

    if (priv->rx_buffers)
    {
	dma_free_coherent(&priv->pdev->dev,
		          RX_RING_SIZE * RX_BUFFER_SIZE,
		          priv->rx_buffers, priv->rx_buffers_dma);
	priv->rx_buffers = NULL;
    }
}

static int
pc302emac_alloc_consistent(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);

    int size = 0;

    size = TX_RING_SIZE * sizeof(struct ring_info);
    priv->tx_skb = kmalloc(size, GFP_KERNEL);
    if (!priv->tx_skb)
    {
	goto out_err;
    }

    size = RX_RING_BYTES;
    priv->rx_ring = dma_alloc_coherent(&priv->pdev->dev, size,
				       &priv->rx_ring_dma, GFP_KERNEL);
    if (!priv->rx_ring)
    {
	goto out_err;
    }

    printk(KERN_DEBUG "%s: allocated Rx ring of %d bytes at %08lx "
                      "(mapped %p)\n", dev->name, size,
                      (unsigned long)priv->rx_ring_dma, priv->rx_ring);

    size = TX_RING_BYTES;
    priv->tx_ring = dma_alloc_coherent(&priv->pdev->dev, size,
				       &priv->tx_ring_dma, GFP_KERNEL);
    if (!priv->tx_ring)
    {
	goto out_err;
    }

    printk(KERN_DEBUG "%s: allocated Tx ring of %d bytes at %08lx "
                      "(mapped %p)\n",	dev->name, size,
                      (unsigned long)priv->tx_ring_dma, priv->tx_ring);

    size = RX_RING_SIZE * RX_BUFFER_SIZE;
    priv->rx_buffers = dma_alloc_coherent(&priv->pdev->dev, size,
					  &priv->rx_buffers_dma, GFP_KERNEL);
    if (!priv->rx_buffers)
    {
	goto out_err;
    }

    printk(KERN_DEBUG "%s: allocated Rx buffers of %d bytes at %08lx "
                      "(mapped %p)\n", dev->name, size,
                      (unsigned long)priv->rx_buffers_dma, priv->rx_buffers);

    return SUCCESS;

out_err:
    pc302emac_free_consistent(dev);
    return -ENOMEM;
}

static void
pc302emac_init_rings(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);

    int i = 0;
    dma_addr_t addr = 0;;

    /* Rx Ring */
    addr = priv->rx_buffers_dma;
    for (i = 0; i < RX_RING_SIZE; i++)
    {
        priv->rx_ring[i].addr = addr;
        priv->rx_ring[i].ctrl = 0;
	addr += RX_BUFFER_SIZE;
    }

    /* Mark the end of the ring */
    priv->rx_ring[RX_RING_SIZE - 1].addr |= EMAC_RX_DESC_WRAP;

    /* Tx Ring */
    for (i = 0; i < TX_RING_SIZE; i++)
    {
        priv->tx_ring[i].addr = 0;
        priv->tx_ring[i].ctrl = EMAC_TX_DESC_HOST_OWN;
    }

    /* Mark the end of the ring */
    priv->tx_ring[TX_RING_SIZE - 1].ctrl |= EMAC_TX_DESC_WRAP;

    priv->rx_tail = priv->tx_head = priv->tx_tail = 0;
}

static void
pc302emac_reset_hw(struct net_device *dev)
{
    /* Make sure we have the write buffer for ourselves */
    wmb();

     /* Disable Rx and Tx */
    emac_iowrite32(dev, 0, EMAC_NETWORK_CTRL_REG_OFFSET);

    /* Clear the EMAC statistics counters */
    emac_iowrite32(dev, EMAC_CLEAR_STATS_REGISTERS,
                        EMAC_NETWORK_CTRL_REG_OFFSET);

    /* Clear The Tx and Rx status flags */
    emac_iowrite32(dev, 0xFFFFFFFF, EMAC_TX_STATUS_REG_OFFSET);
    emac_iowrite32(dev, 0xFFFFFFFF, EMAC_RX_STATUS_REG_OFFSET);

    /* Disable all interrupts */
    emac_iowrite32(dev, 0xFFFFFFFF, EMAC_INT_DISABLE_REG_OFFSET);
    emac_ioread32(dev, EMAC_INT_STATUS_REG_OFFSET);
}

static void
pc302emac_init_hw(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);

    u32 dma_config_register = 0;
    u32 config = 0;
    u32 control = 0;
    u32 interrupt_enable = 0;

    pc302emac_reset_hw(dev);
    pc302emac_set_hwaddr(dev);

    /* Setup the size of the DMA Receive Buffer */
    dma_config_register = emac_ioread32(dev, EMAC_DMA_CFG_REG_OFFSET);
    dma_config_register &= ~(EMAC_DMA_RX_BUFFER_SIZE_MASK);
    dma_config_register |= EMAC_DMA_RX_BUFFER_SIZE;
    emac_iowrite32(dev, dma_config_register, EMAC_DMA_CFG_REG_OFFSET);

    /* Network configuration register setup */
    config = emac_ioread32(dev, EMAC_NETWORK_CFG_REG_OFFSET);

    /* Required for PC302 */
    config |= EMAC_64_BIT_AMBA_DATA_BUS_WITDH;

    /* Pause Enable */
    config |= EMAC_PAUSE_ENABLE;
    
    /* Enable reception of 1536 byte frames */
    config |= EMAC_RX_1536_BYTE_FRAMES;

    /* Discard frames with length errors */
    config |= EMAC_LENGTH_FIELD_ERROR_FRAME_DISCARD;
    if (priv->dev->flags & IFF_PROMISC)
    {
    	/* Copy all Frames */
        config |= EMAC_COPY_ALL_FRAMES;
    }
    if (!(priv->dev->flags & IFF_BROADCAST))
    {
	/* No Broadcast frames */
        config |= EMAC_NO_BROADCAST_FRAMES;
    }

    /* Enable Rx Checksum offload */
    config |= EMAC_RX_CHKSUM_OFFLOAD_ENABLE;

    emac_iowrite32(dev, config, EMAC_NETWORK_CFG_REG_OFFSET);

    /* Initialize Tx and Rx buffer base address registers */
    emac_iowrite32(dev, priv->rx_ring_dma,
                   EMAC_RX_BUFF_Q_BASE_ADDR_REG_OFFSET);
    emac_iowrite32(dev, priv->tx_ring_dma,
                   EMAC_TX_BUFF_Q_BASE_ADDR_REG_OFFSET);

    /* Network control register setup */
    control = emac_ioread32(dev, EMAC_NETWORK_CTRL_REG_OFFSET);

    /* Enable the EMAC to store the nS portion of the timestamp
     * of the received packet in the crc field of the received packet.
     * Note: This functionality is required for packet timestamp
     *       reporting.
     */
    control |= EMAC_STORE_TX_TIMESTAMP_IN_CRC;

    control |= EMAC_RX_ENABLE;
    control |= EMAC_TX_ENABLE;
    control |= EMAC_MDIO_ENABLE;
    emac_iowrite32(dev, control, EMAC_NETWORK_CTRL_REG_OFFSET);

    /* Initialise hw_timestamp_flag */
    priv->hw_timestamp_flag = HWTSTAMP_FILTER_NONE;

    /* Enable interrupts */
    interrupt_enable = EMAC_ENABLE_RX_COMPLETE
                     | EMAC_ENABLE_RX_USED_BIT_READ
                     | EMAC_ENABLE_TX_BUFF_UNDERRUN
                     | EMAC_ENABLE_TRANSMIT_CORRUPTION_AHB_ERROR
                     | EMAC_ENABLE_TX_COMPLETE
                     | EMAC_ENABLE_RETRY_LIMIT_EXCEEDED
                     | EMAC_ENABLE_RX_OVERRUN
                     | EMAC_ENABLE_HRESP_NOT_OK;

    emac_iowrite32(dev, interrupt_enable, EMAC_INT_ENABLE_REG_OFFSET);
}

/*
 * The hash address register is 64 bits long and takes up two
 * locations in the memory map.  The least significant bits are stored
 * in EMAC_HSL and the most significant bits in EMAC_HSH.
 *
 * The unicast hash enable and the multicast hash enable bits in the
 * network configuration register enable the reception of hash matched
 * frames. The destination address is reduced to a 6 bit index into
 * the 64 bit hash register using the following hash function.  The
 * hash function is an exclusive or of every sixth bit of the
 * destination address.
 *
 * hi[5] = da[5] ^ da[11] ^ da[17] ^ da[23] ^ da[29] ^ da[35] ^ da[41] ^ da[47]
 * hi[4] = da[4] ^ da[10] ^ da[16] ^ da[22] ^ da[28] ^ da[34] ^ da[40] ^ da[46]
 * hi[3] = da[3] ^ da[09] ^ da[15] ^ da[21] ^ da[27] ^ da[33] ^ da[39] ^ da[45]
 * hi[2] = da[2] ^ da[08] ^ da[14] ^ da[20] ^ da[26] ^ da[32] ^ da[38] ^ da[44]
 * hi[1] = da[1] ^ da[07] ^ da[13] ^ da[19] ^ da[25] ^ da[31] ^ da[37] ^ da[43]
 * hi[0] = da[0] ^ da[06] ^ da[12] ^ da[18] ^ da[24] ^ da[30] ^ da[36] ^ da[42]
 *
 * da[0] represents the least significant bit of the first byte
 * received, that is, the multicast/unicast indicator, and da[47]
 * represents the most significant bit of the last byte received.  If
 * the hash index, hi[n], points to a bit that is set in the hash
 * register then the frame will be matched according to whether the
 * frame is multicast or unicast.  A multicast match will be signalled
 * if the multicast hash enable bit is set, da[0] is 1 and the hash
 * index points to a bit set in the hash register.  A unicast match
 * will be signalled if the unicast hash enable bit is set, da[0] is 0
 * and the hash index points to a bit set in the hash register.  To
 * receive all multicast frames, the hash register should be set with
 * all ones and the multicast hash enable bit should be set in the
 * network configuration register.
 */

static inline int
hash_bit_value(int bitnr,
               __u8 *addr)
{
    if (addr[bitnr / 8] & (1 << (bitnr % 8)))
    {
	return BIT_IS_ONE;
    }
    return BIT_IS_ZERO;
}

static int
hash_get_index(__u8 *addr)
{
    int i = 0;
    int j = 0;
    int bitval = 0;
    int hash_index = 0;

    for (j = 0; j < 6; j++)
    {
	for (i = 0, bitval = 0; i < 8; i++)
        {
	    bitval ^= hash_bit_value(i*6 + j, addr);
        }

        hash_index |= (bitval << j);
    }

    return hash_index;
}

static void
pc302emac_sethashtable(struct net_device *dev)
{
    struct dev_mc_list *curr = NULL;
    unsigned long mc_filter[2];
    unsigned int i = 0;
    unsigned int bitnr = 0;

    mc_filter[0] = mc_filter[1] = 0;

    curr = dev->mc_list;
    for (i = 0; i < dev->mc_count; i++, curr = curr->next)
    {
        if (!curr)
        {
            break;	/* unexpected end of list */
        }

	bitnr = hash_get_index(curr->dmi_addr);
	mc_filter[bitnr >> 5] |= 1 << (bitnr & 31);
    }

    emac_iowrite32(dev, (u32)mc_filter[0], EMAC_HASH_BOT_31_0_REG_OFFSET);
    emac_iowrite32(dev, (u32)mc_filter[1], EMAC_HASH_TOP_63_32_REG_OFFSET);
}

static void
pc302emac_set_rx_mode(struct net_device *dev)
{
    u32 config = 0;

    config = emac_ioread32(dev, EMAC_NETWORK_CFG_REG_OFFSET);

    if (dev->flags & IFF_PROMISC)
    {
    	/* Enable promiscuous mode */
	config |= EMAC_COPY_ALL_FRAMES;
    }
    else if (dev->flags & (~IFF_PROMISC))
    {
	/* Disable promiscuous mode */
	config &= ~(EMAC_COPY_ALL_FRAMES);
    }
    if (dev->flags & IFF_ALLMULTI)
    {
	/* Enable all multicast mode */
	emac_iowrite32(dev, 0xFFFFFFFF, EMAC_HASH_BOT_31_0_REG_OFFSET);
	emac_iowrite32(dev, 0xFFFFFFFF, EMAC_HASH_TOP_63_32_REG_OFFSET);
	config |= EMAC_MULTICAST_HASH_ENABLE;
    }
    else if (dev->mc_count > 0)
    {
	/* Enable specific multicasts */
	pc302emac_sethashtable(dev);
	config |= EMAC_MULTICAST_HASH_ENABLE;
    }
    else if (dev->flags & (~IFF_ALLMULTI))
    {
	/* Disable all multicast mode */
	emac_iowrite32(dev, 0, EMAC_HASH_BOT_31_0_REG_OFFSET);
	emac_iowrite32(dev, 0, EMAC_HASH_TOP_63_32_REG_OFFSET);
	config &= ~(EMAC_MULTICAST_HASH_ENABLE);
    }

    emac_iowrite32(dev, config, EMAC_NETWORK_CFG_REG_OFFSET);
}

static int
pc302emac_open(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    int ret = 0;

    /* If the phy is not yet registered, retry later */
    if (!priv->phy_dev && !priv->phy_probed)
    {
    	return -EAGAIN;
    }

    if (!is_valid_ether_addr(dev->dev_addr))
    {
    	return -EADDRNOTAVAIL;
    }

    ret = pc302emac_alloc_consistent(dev);
    if (ret)
    {
	printk(KERN_ERR "%s: unable to allocate DMA memory (error %d)\n",
                         dev->name, ret);
	return ret;
    }

    napi_enable(&priv->napi);

    pc302emac_init_rings(dev);
    pc302emac_init_hw(dev);

    /* schedule a link state check */
    if (priv->phy_dev)
    {
        phy_start(priv->phy_dev);
    }
    else
    {
        netif_carrier_on(dev);
    }

    netif_start_queue(dev);

    return SUCCESS;
}

static int
pc302emac_close(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    unsigned long flags = 0;
    int i;

    printk("Closing emac interface %s\n", dev->name);
    printk("Register dump:\n");
    for (i=0; i<0x28; i+=4)
    {
        printk("  0x%03X:0x%08X\n", i, emac_ioread32(dev, i));
    }
    for (i=0x30; i<0x40; i+=4)
    {
        printk("  0x%03X:0x%08X\n", i, emac_ioread32(dev, i));
    }
    for (i=0x80; i<0xC4; i+=4)
    {
        printk("  0x%03X:0x%08X\n", i, emac_ioread32(dev, i));
    }
    for (i=0x100; i<0x1B4; i+=4)
    {
        printk("  0x%03X:0x%08X\n", i, emac_ioread32(dev, i));
    }
    for (i=0x1C8; i<0x200; i+=4)
    {
        printk("  0x%03X:0x%08X\n", i, emac_ioread32(dev, i));
    }
    printk("RX tail=%u, pending=%u\n", priv->rx_tail, priv->rx_pending);
    printk("RX ring is at %p\n", priv->rx_ring);
    if (priv->rx_ring)
    {
        for (i=0; i<RX_RING_SIZE; i++)
        {
            printk("  rx_dma[%02d]: addr=0x%08X ctrl=0x%08X\n", i, priv->rx_ring[i].addr, priv->rx_ring[i].ctrl);
        }
    }
    printk("TX head=%u tail=%u, pending=%u, avail=%u\n", priv->tx_head, priv->tx_tail, priv->tx_pending, TX_BUFFS_AVAIL(priv));
    printk("TX ring is at %p\n", priv->tx_ring);
    if (priv->tx_ring)
    {
        for (i=0; i<TX_RING_SIZE; i++)
        {
            printk("  tx_dma[%02d]: addr=0x%08X ctrl=0x%08X\n", i, priv->tx_ring[i].addr, priv->tx_ring[i].ctrl);
        }
    }
    printk("TX is %sstopped\n", netif_queue_stopped(dev)?"":"not ");
    printk("Link=%d, speed=%d, duplex=%d\n", priv->link, priv->speed, priv->duplex);

    netif_stop_queue(dev);
    napi_disable(&priv->napi);

    if (priv->phy_dev)
    {
        phy_stop(priv->phy_dev);
    }

    spin_lock_irqsave(&priv->lock, flags);
    pc302emac_reset_hw(dev);
    netif_carrier_off(dev);
    spin_unlock_irqrestore(&priv->lock, flags);

    pc302emac_free_consistent(dev);

    return SUCCESS;
}

static struct
net_device_stats *pc302emac_get_stats(struct net_device *dev)
{
    struct pc302_emac *priv = netdev_priv(dev);
    struct net_device_stats *stats = &priv->stats;
    struct pc302emac_stats *hwstat = &priv->hw_stats;

    /* Update statistics from the EMAC hardware counters */
    pc302emac_update_stats(priv);

    /* Convert the EMAC hardware counter values into net_device_stats */
    stats->rx_errors = (hwstat->rx_frame_check_sequence_errors +
                        hwstat->rx_alignment_errors +
                        hwstat->rx_resource_errors +
                        hwstat->rx_overruns +
                        hwstat->rx_oversize_frames +
                        hwstat->rx_jabbers +
                        hwstat->rx_undersized_frames +
                        hwstat->rx_length_field_frame_errors);

    stats->tx_errors = (hwstat->tx_late_colloisions +
                        hwstat->tx_excessive_collisions +
                        hwstat->tx_underrun +
                        hwstat->tx_carrier_sense_errors);

    stats->collisions = (hwstat->tx_single_collision_frames +
                         hwstat->tx_multiple_collision_frames +
                         hwstat->tx_excessive_collisions);

    stats->rx_length_errors = (hwstat->rx_oversize_frames +
                               hwstat->rx_jabbers +
                               hwstat->rx_undersized_frames +
                               hwstat->rx_length_field_frame_errors);

    stats->rx_over_errors = hwstat->rx_resource_errors;

    stats->rx_crc_errors = hwstat->rx_frame_check_sequence_errors;

    stats->rx_frame_errors = hwstat->rx_alignment_errors;

    stats->rx_fifo_errors = hwstat->rx_overruns;

    stats->tx_aborted_errors = hwstat->tx_excessive_collisions;

    stats->tx_carrier_errors = hwstat->tx_carrier_sense_errors;

    stats->tx_fifo_errors = hwstat->tx_underrun;

    return stats;
}

static int
pc302emac_get_settings(struct net_device *dev,
                       struct ethtool_cmd *cmd)
{
#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH
    return -ENODEV;
#else /* CONFIG_PC302_MICREL_VLAN_SWITCH */
    struct pc302_emac *priv = netdev_priv(dev);
    struct phy_device *phydev = priv->phy_dev;

    if (!phydev)
    {
	return -ENODEV;
    }

    return phy_ethtool_gset(phydev, cmd);
#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */
}

static int
pc302emac_set_settings(struct net_device *dev,
                       struct ethtool_cmd *cmd)
{
#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH
    return -ENODEV;
#else /* CONFIG_PC302_MICREL_VLAN_SWITCH */
    struct pc302_emac *priv = netdev_priv(dev);
    struct phy_device *phydev = priv->phy_dev;

    if (!phydev)
    {
        return -ENODEV;
    }

    return phy_ethtool_sset(phydev, cmd);
#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */
}

static void
pc302emac_get_drvinfo(struct net_device *dev,
	              struct ethtool_drvinfo *info)
{
    struct pc302_emac *priv = netdev_priv(dev);

    strcpy(info->driver, priv->pdev->dev.driver->name);
    strcpy(info->bus_info, priv->pdev->dev.bus_id);
}

static struct
ethtool_ops pc302emac_ethtool_ops =
{
    .get_settings   = pc302emac_get_settings,
    .set_settings   = pc302emac_set_settings,
    .get_drvinfo    = pc302emac_get_drvinfo,
    .get_link	    = ethtool_op_get_link,
};

static int
pc302emac_ioctl(struct net_device *dev,
                struct ifreq *rq,
                int cmd)
{
    struct pc302_emac *priv = netdev_priv(dev);
    struct phy_device *phydev = priv->phy_dev;

    /* Is the network interface up and running ?*/
    if (!netif_running(dev))
    {
	return -EINVAL;
    }

    switch (cmd)
    {
        case SIOCGMIIPHY:
            /* Fall through intentional */

        case SIOCGMIIREG:
            /* Fall through intentional */

        case SIOCSMIIREG:
#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH
	    return -EOPNOTSUPP;
#else /* CONFIG_PC302_MICREL_VLAN_SWITCH */
            /* Do we have a connected phy ? */
            if (phydev)
            {
                return phy_mii_ioctl(phydev, if_mii(rq), cmd);
            }
            else
            {
                return -ENODEV;
            }
#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */

        case SIOCSHWTSTAMP:
	    return pc302emac_hwtstamp_ioctl(dev, rq, cmd);

        default:
	    return -EOPNOTSUPP;
    }
}

static int pc302emac_hwtstamp_ioctl(struct net_device *dev,
                                    struct ifreq *ifr,
                                    int cmd)
{
    struct pc302_emac *priv = netdev_priv(dev);
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

/* Sysfs functions and data structures */
static ssize_t
pc302emac_sysfs_1588_seconds_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
    u32 value = emac_ioread32(sysfs_ndev,
                              EMAC_1588_TIMER_SECONDS_REG_OFFSET);

    return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t
pc302emac_sysfs_1588_seconds_store(struct device *dev,
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
        emac_iowrite32(sysfs_ndev, (u32)value,
                       EMAC_1588_TIMER_SECONDS_REG_OFFSET);
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(1588_timer_seconds, (S_IRUGO | S_IWUGO),
                   pc302emac_sysfs_1588_seconds_show,
                   pc302emac_sysfs_1588_seconds_store);

static ssize_t
pc302emac_sysfs_1588_nano_seconds_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    u32 value = emac_ioread32(sysfs_ndev,
                              EMAC_1588_TIMER_NANO_SECONDS_REG_OFFSET);

    return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t
pc302emac_sysfs_1588_nano_seconds_store(struct device *dev,
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
        emac_iowrite32(sysfs_ndev,
                       (u32)value, EMAC_1588_TIMER_NANO_SECONDS_REG_OFFSET);
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(1588_timer_nano_seconds, (S_IRUGO | S_IWUGO),
                   pc302emac_sysfs_1588_nano_seconds_show,
                   pc302emac_sysfs_1588_nano_seconds_store);

static ssize_t
pc302emac_sysfs_1588_timer_adjust_store(struct device *dev,
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
        emac_iowrite32(sysfs_ndev, (u32)value,
                       EMAC_1588_TIMER_ADJUST_REG_OFFSET);
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(1588_timer_adjust, S_IWUGO,
                   NULL,
                   pc302emac_sysfs_1588_timer_adjust_store);

static ssize_t
pc302emac_sysfs_1588_timer_increment_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    u32 value = emac_ioread32(sysfs_ndev,
                              EMAC_1588_TIMER_INCREMENT_REG_OFFSET);

    return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t
pc302emac_sysfs_1588_timer_increment_store(struct device *dev,
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
        emac_iowrite32(sysfs_ndev, (u32)value,
                       EMAC_1588_TIMER_INCREMENT_REG_OFFSET);
    }

    return (ssize_t)count;
}

static DEVICE_ATTR(1588_timer_increment, (S_IRUGO | S_IWUGO),
                   pc302emac_sysfs_1588_timer_increment_show,
                   pc302emac_sysfs_1588_timer_increment_store);

/*!
 * The group of sysfs attributes that should be added
 * to the sysfs filesystem.
 */
static struct attribute *pc302emac_attrs[] =
{
    &dev_attr_1588_timer_seconds.attr,
    &dev_attr_1588_timer_nano_seconds.attr,
    &dev_attr_1588_timer_adjust.attr,
    &dev_attr_1588_timer_increment.attr,
    NULL
};

/*!
 * Add the sysfs attributes as a group
 */
static struct attribute_group pc302emac_attr_group =
{
    .attrs = pc302emac_attrs
};

static int
pc302emac_sysfs_add(struct platform_device *pdev)
{
    return sysfs_create_group(&pdev->dev.kobj, &pc302emac_attr_group);
}

static void
pc302emac_sysfs_remove(struct platform_device *pdev)
{
    sysfs_remove_group(&pdev->dev.kobj, &pc302emac_attr_group);
}

static int
emac_drv_probe(struct platform_device *pdev)
{
    struct resource *regs;
    struct resource *irq;
    struct net_device *dev;
    struct pc302_emac *priv;
    struct phy_device *phydev;

    int ret = -ENXIO;

    u32 config;

    /* Obtain some platform resources */
    regs = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    irq = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );

    if (!regs || ! irq)
    {
        /* Oops, we can't obtain any resources */
        printk(KERN_ERR "%s: could not obtain platform resources.\n",
                         CARDNAME);
        ret = -EINVAL;
        goto out;
    }

    if (!request_mem_region(regs->start, (regs->end - regs->start) + 1,
                            CARDNAME))
    {
        /* Oops, we can't obtain the required memory region */
        printk(KERN_ERR "%s: memory mapping error, Address=0x%08x,"
                        " Size=0x%08x\n", CARDNAME, regs->start,
                         (regs->end - regs->start) + 1);
        ret = -ENOMEM;
        goto out;
    }

    dev = alloc_etherdev(sizeof(*priv));
    if (!dev)
    {
        /* Oops, we can't allocate the net_device structure */
        printk(KERN_ERR "%s: could not allocate net_device structure.\n",
                         CARDNAME);
        ret = -ENOMEM;
        goto out_alloc_failed;
    }

    SET_NETDEV_DEV(dev, &pdev->dev);

    priv = netdev_priv(dev);
    
    priv->hwaddr_failure = 0;
    
#ifndef CONFIG_PC302_MICREL_VLAN_SWITCH
    priv->mii_bus = mdiobus_alloc();
#endif /* ! CONFIG_PC302_MICREL_VLAN_SWITCH */
    priv->pdev = pdev;
    priv->dev = dev;
    /* Create a DMA pool for the transmit buffers. We use this to copy the
     * transmit data into here to make sure that we are aligned to a 16 bit
     * boundary. */
    priv->tx_buf_pool =
        dma_pool_create("pc302emac_tx", &pdev->dev, TX_BUFFER_SIZE, 16, 0 );
    if (!priv->tx_buf_pool)
    {
        printk(KERN_ERR "%s: failed to allocate tx buf cache\n", CARDNAME);
        ret = -ENOMEM;
        goto out_tx_cache_failed;
    }

    spin_lock_init(&priv->lock);

    priv->regs = ioremap(regs->start, (regs->end - regs->start) + 1);
    if (!priv->regs)
    {
        /* Oops, we can't remap io memory */
        printk(KERN_ERR "%s: could not remap io addresses.\n",
                         CARDNAME);
	ret = -ENOMEM;
	goto out_ioremap_failed;
    }

    dev->irq = irq->start;
    ret = request_irq(dev->irq, pc302emac_interrupt, 0,
		      dev->name, dev);
    if (ret)
    {
        printk(KERN_ERR "%s: unable to request IRQ %d (error %d)\n",
		        CARDNAME, dev->irq, ret);
        ret = -ENXIO;
        goto out_req_irq_failed;
    }

    dev->open = pc302emac_open;
    dev->stop = pc302emac_close;
    dev->hard_start_xmit = pc302emac_start_xmit;
    dev->get_stats = pc302emac_get_stats;
    dev->set_multicast_list = pc302emac_set_rx_mode;
    dev->vlan_rx_register   = emac_vlan_rx_register;
    dev->vlan_rx_kill_vid   = emac_vlan_rx_kill_vid;
    dev->do_ioctl = pc302emac_ioctl;
    netif_napi_add(dev, &priv->napi, pc302emac_poll, 16);
    dev->ethtool_ops = &pc302emac_ethtool_ops;

    dev->features           |= NETIF_F_HW_VLAN_RX;

    dev->base_addr = regs->start;

    /* Set MII management clock divider */
    config = emac_ioread32(dev, EMAC_NETWORK_CFG_REG_OFFSET);
    config &= EMAC_MDC_CLOCK_DIV_MASK;
    config |= EMAC_MDC_CLOCK_DIV_96;
    emac_iowrite32(dev, config, EMAC_NETWORK_CFG_REG_OFFSET);

    /* Get MAC address */
    pc302emac_get_hwaddr(dev);

    priv->tx_pending = DEF_TX_RING_PENDING;

    /* Register the network device */
    ret = register_netdev(dev);
    if (ret)
    {
	printk(KERN_ERR "%s: could not register net device\n", CARDNAME);

        ret = -ENOMEM;
        goto out_reg_netdev_failed;
    }

#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH
    if (pc302emac_i2c_probe(priv->dev) != 0)
    {
	printk(KERN_ERR "%s: pc302emac_i2c_probe() failed\n", dev->name);
        ret = -ENXIO;
        goto out_mii_init_failed;
    }
#else /* !CONFIG_PC302_MICREL_VLAN_SWITCH */
    if (pc302emac_mii_init(dev) != 0)
    {
        ret = -ENXIO;
        goto out_mii_init_failed;
    }
#endif /* !CONFIG_PC302_MICREL_VLAN_SWITCH */

    platform_set_drvdata(pdev, dev);

    /* Initislise our local copy for sysfs use */
    sysfs_ndev = dev;

    /* Setup sysfs support */
    ret = pc302emac_sysfs_add(pdev);
    if (ret)
    {
        /* Oops, something not right here */
        printk(KERN_ERR "%s: failed to create sysfs device attributes\n",
               CARDNAME);
    }

    printk(KERN_INFO "%s: Ethernet driver " CONFIG_LOCALVERSION " loaded\n",
           CARDNAME);

    phydev = priv->phy_dev;
    if (phydev)
    {
        printk(KERN_INFO "%s: attached Phy driver [%s] "
                         "(mii_bus:phy_addr=%s, irq=%d)\n",
                         dev->name, phydev->drv->name, phydev->dev.bus_id,
                         phydev->irq);
    }
    else
    {
        printk(KERN_INFO "%s: no Phy found\n", dev->name);
    }
    return SUCCESS;

out_mii_init_failed:
    unregister_netdev(dev);

out_reg_netdev_failed:
    free_netdev(dev);

out_req_irq_failed:
    free_irq(dev->irq, dev);

out_ioremap_failed:
    iounmap(priv->regs);

out_tx_cache_failed:
out_alloc_failed:
    release_mem_region (regs->start, (regs->end - regs->start) + 1);

out:
    /* We have failed in some way */
    printk(KERN_ERR "%s: Ethernet driver registration failed\n", CARDNAME);
    return ret;
}

static int
emac_drv_remove(struct platform_device *pdev)
{
    struct net_device *dev = NULL;
    struct pc302_emac *priv = NULL;

    dev = platform_get_drvdata(pdev);

    if (dev)
    {
        priv = netdev_priv(dev);
#ifndef CONFIG_PC302_MICREL_VLAN_SWITCH
	if (priv->phy_dev)
        {
	    phy_disconnect(priv->phy_dev);
        }
	mdiobus_unregister(priv->mii_bus);
	kfree(priv->mii_bus->irq);
        mdiobus_free(priv->mii_bus);
#endif /* ! CONFIG_PC302_MICREL_VLAN_SWITCH */
	unregister_netdev(dev);
	free_irq(dev->irq, dev);
	iounmap(priv->regs);
        dma_pool_destroy(priv->tx_buf_pool);
	free_netdev(dev);
	platform_set_drvdata(pdev, NULL);

        /* Remove sysfs support */
        pc302emac_sysfs_remove(pdev);
    }

    return SUCCESS;
}

/* Enables and disables VLAN insertion/extraction */
static void emac_vlan_rx_register(
        struct net_device *dev, struct vlan_group *grp)
{
    struct pc302_emac *priv = netdev_priv(dev);

    priv->vlgrp = grp;
}

static void emac_vlan_rx_kill_vid(struct net_device *dev, uint16_t vid)
{
    /* Nothing to do. We do not support filtering */
}

/*!
 * Platform driver data structure.
 */
static struct
platform_driver emac_driver =
{
    .probe      = emac_drv_probe,
    .remove     = emac_drv_remove,
    .driver     =
    {
        .name   = CARDNAME,
    }
};

static int
emac_init_module(void)
{
    int ret = 0;

#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH
    ret = pc302_i2c_phy_init();
    if (ret != 0)
    {
        printk(KERN_ERR "%s: Failed to register Micrel phy driver\n", CARDNAME);
        return ret;
    }
#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */

    /* Register the platform driver */
    ret = platform_driver_register(&emac_driver);
    if (ret != 0)
    {
        printk(KERN_ERR "%s: Failed to register EMAC driver\n", CARDNAME);
#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH
        pc302_i2c_phy_uninit();
#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */
    }

    return ret;
}

static void
emac_cleanup_module(void)
{
#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH
    pc302_i2c_phy_uninit();
#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */
    platform_driver_unregister(&emac_driver);
}

module_init(emac_init_module);
module_exit(emac_cleanup_module);

MODULE_AUTHOR("picoChip");
MODULE_DESCRIPTION("picoChip PC302 Ethernet Driver");
MODULE_LICENSE("GPL");
