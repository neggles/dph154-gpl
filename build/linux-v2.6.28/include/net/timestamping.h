#ifndef _NET_TIMESTAMPING_H
#define _NET_TIMESTAMPING_H

#include <linux/socket.h>   /* for SO_TIMESTAMPING */

/**
 * user space linux/socket.h might not have these defines yet:
 * provide fallback
 */
#if !defined(__kernel__) && !defined(SO_TIMESTAMPING)
# define SO_TIMESTAMPING         37
# define SCM_TIMESTAMPING        SO_TIMESTAMPING
#endif

/** %SO_TIMESTAMPING gets an integer bit field comprised of these values */
enum {
	SOF_TIMESTAMPING_TX_HARDWARE = (1<<0),
	SOF_TIMESTAMPING_TX_SOFTWARE = (1<<1),
	SOF_TIMESTAMPING_RX_HARDWARE = (1<<2),
	SOF_TIMESTAMPING_RX_SOFTWARE = (1<<3),
	SOF_TIMESTAMPING_SOFTWARE = (1<<4),
	SOF_TIMESTAMPING_SYS_HARDWARE = (1<<5),
	SOF_TIMESTAMPING_RAW_HARDWARE = (1<<6),
	SOF_TIMESTAMPING_MASK = (SOF_TIMESTAMPING_RAW_HARDWARE - 1) | SOF_TIMESTAMPING_RAW_HARDWARE
};

#if !defined(__kernel__) && !defined(SIOCSHWTSTAMP)
# define SIOCSHWTSTAMP 0x89b0
#endif

/** %SIOCSHWTSTAMP expects a struct ifreq with a ifr_data pointer to this struct  */
struct hwtstamp_config {
	int flags;           /**\u2264 no flags defined right now, must be zero */
	int tx_type;         /**\u2264 one of HWTSTAMP_TX_* */
	int rx_filter_type;  /**\u2264 one of HWTSTAMP_RX_* */
};

/** possible values for hwtstamp_config->tx_type */
enum {
	/**
	 * no outgoing packet will need hardware time stamping;
	 * should a packet arrive which asks for it, no hardware
	 * time stamping will be done
	 */
	HWTSTAMP_TX_OFF,

	/**
	 * enables hardware time stamping for outgoing packets;
	 * the sender of the packet decides which are to be
	 * time stamped by setting SOF_TIMESTAMPING_TX_SOFTWARE
	 * before sending the packet
	 */
	HWTSTAMP_TX_ON,
};

/** possible values for hwtstamp_config->rx_filter_type */
enum {
	/** time stamp no incoming packet at all */
	HWTSTAMP_FILTER_NONE,

	/** time stamp any incoming packet */
	HWTSTAMP_FILTER_ALL,

        /** return value: time stamp all packets requested plus some others */
        HWTSTAMP_FILTER_SOME,

	/** PTP v1, UDP, any kind of event packet */
	HWTSTAMP_FILTER_PTP_V1_L4_EVENT,
	/** PTP v1, UDP, Sync packet */
	HWTSTAMP_FILTER_PTP_V1_L4_SYNC,
	/** PTP v1, UDP, Delay_req packet */
	HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ,
	/** PTP v2, UDP, any kind of event packet */
	HWTSTAMP_FILTER_PTP_V2_L4_EVENT,
	/** PTP v2, UDP, Sync packet */
	HWTSTAMP_FILTER_PTP_V2_L4_SYNC,
	/** PTP v2, UDP, Delay_req packet */
	HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ,

	/** 802.AS1, Ethernet, any kind of event packet */
	HWTSTAMP_FILTER_PTP_V2_L2_EVENT,
	/** 802.AS1, Ethernet, Sync packet */
	HWTSTAMP_FILTER_PTP_V2_L2_SYNC,
	/** 802.AS1, Ethernet, Delay_req packet */
	HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ,

	/** PTP v2/802.AS1, any layer, any kind of event packet */
	HWTSTAMP_FILTER_PTP_V2_EVENT,
	/** PTP v2/802.AS1, any layer, Sync packet */
	HWTSTAMP_FILTER_PTP_V2_SYNC,
	/** PTP v2/802.AS1, any layer, Delay_req packet */
	HWTSTAMP_FILTER_PTP_V2_DELAY_REQ,
};

#endif /* _NET_TIMESTAMPING_H */
