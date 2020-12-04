/* 
 *****************************************************************************
 * 
 * Copyright (c) 2010 ip.access Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *****************************************************************************
 */
#ifdef CONFIG_PC302_MICREL_VLAN_SWITCH

#include <linux/kernel.h>
#include <linux/autoconf.h>
#include <linux/phy.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/err.h>
#include "pc302_i2c_phy.h"


#define MICREL_PORT1_CTRL12_REG 0x1C
#define MICREL_PORT2_CTRL12_REG 0x2C

#define MICREL_PORT1_STATUS_REG 0x1E
#define MICREL_PORT2_STATUS_REG 0x2E
#define MP1S_AN_DONE            (1 << 6)
#define MP1S_LSTATUS            (1 << 5)
#define MP1S_LPA_PAUSE_CAP      (1 << 4)
#define MP1S_LPA_100FULL        (1 << 3)
#define MP1S_LPA_100HALF        (1 << 2)
#define MP1S_LPA_10FULL         (1 << 1)
#define MP1S_LPA_10HALF         (1 << 0)

struct micrel_phy_device {
        struct phy_device *phy_device;
        struct i2c_client *client;
        int started;
};

static struct micrel_phy_device *i2c_phy_device;




/**
 * micrel_phy_config_aneg - restart auto-negotiation or write BMCR
 * @phydev: target phy_device struct
 *
 * Description: If auto-negotiation is enabled, we configure the
 *   advertising, and then restart auto-negotiation.  If it is not
 *   enabled, then we write the BMCR.
 */
static int micrel_phy_config_aneg(struct phy_device *phydev)
{
        return 0;
}


/**
 * micrel_phy_read_status - check the link status and update current link state
 * @phydev: target phy_device struct
 *
 * Description: Check the link, then figure out the current state
 *   by comparing what we advertise with what the link partner
 *   advertises.  Start by checking the gigabit possibilities,
 *   then move on to 10/100.
 */
static int micrel_phy_read_status(struct phy_device *phydev)
{
        struct micrel_phy_device *micrel_phy;
	int lpa;

        micrel_phy = phydev->priv;
        if (micrel_phy->client == NULL) {
                printk(KERN_WARNING "%s: i2c client ptr is NULL\n", __FUNCTION__);
                return 0;
        }

	/* Read link status */
        lpa = i2c_smbus_read_byte_data(micrel_phy->client, MICREL_PORT1_STATUS_REG);
	if (lpa < 0)
		return lpa;

	if ((lpa & MP1S_LSTATUS) == 0)
		phydev->link = 0;
	else
		phydev->link = 1;

        phydev->speed = SPEED_10;
        phydev->duplex = DUPLEX_HALF;
        phydev->pause = phydev->asym_pause = 0;

        if (lpa & (MP1S_LPA_100FULL | MP1S_LPA_100HALF)) {
                phydev->speed = SPEED_100;
			
                if (lpa & MP1S_LPA_100FULL) {
                        phydev->duplex = DUPLEX_FULL;
                }
        } else {
                if (lpa & MP1S_LPA_10FULL) {
                        phydev->duplex = DUPLEX_FULL;
                }
        }

        if (phydev->duplex == DUPLEX_FULL) {
                phydev->pause = lpa & MP1S_LPA_PAUSE_CAP ? 1 : 0;
        }

        return 0;
}


static int micrel_phy_config_init(struct phy_device *phydev)
{
	u32 features;

        /* Only advertise minimal features to the net dev */
	features = (SUPPORTED_MII);
        features |= SUPPORTED_100baseT_Full;
        features |= SUPPORTED_100baseT_Half;
        features |= SUPPORTED_10baseT_Full;
        features |= SUPPORTED_10baseT_Half;

	phydev->supported = features;
	phydev->advertising = features;

	return 0;
}




        
static struct phy_driver micrel_phy_driver = {
	.phy_id		= 0xffffffff,
	.phy_id_mask	= 0xffffffff,
	.name		= "Micrel I2C PHY",
	.config_init	= micrel_phy_config_init,
	.features	= 0,
	.config_aneg	= micrel_phy_config_aneg,
	.read_status	= micrel_phy_read_status,
	.driver		= {.owner= THIS_MODULE, },
};


static struct micrel_phy_device *get_micrel_phy(const char *bus_id)
{
        return i2c_phy_device;
}


/* 
 * This is called from both the netdev side and the I2C side.
 * When both have been configured, we start the PHY state machine.
 */
static void start_phy(struct micrel_phy_device *micrel_phy)
{
        if (!micrel_phy->started)
        {
                struct phy_device *phydev = micrel_phy->phy_device;
                
                if (phydev->attached_dev && micrel_phy->client)
                {
                        phy_start_machine(phydev, NULL);

                        micrel_phy->started = 1;
                }
        }
}



/**
 * Create an instance of a I2C PHY device
 */
struct micrel_phy_device * i2c_phy_create(int addr, u32 phy_id)
{
	struct micrel_phy_device *i2cphydev;
	struct phy_device *phydev;

        i2cphydev = kzalloc(sizeof(*i2cphydev), GFP_KERNEL);
	if (NULL == i2cphydev)
		return NULL;

        phydev = phy_device_create(NULL, addr, phy_id);
        if (IS_ERR(phydev)) {
                kfree(i2cphydev);
		return NULL;
        }

        phydev->drv = &micrel_phy_driver;
        phydev->interface = PHY_INTERFACE_MODE_MII;
        phydev->autoneg = AUTONEG_DISABLE;
        phydev->irq = PHY_POLL;

        /* Assume PHY is up from bot */
        phydev->state = PHY_RUNNING;

        i2cphydev->phy_device = phydev;
        phydev->priv = i2cphydev;

	return i2cphydev;
}
EXPORT_SYMBOL(i2c_phy_create);


int i2c_phy_attach(struct phy_device *phydev, struct net_device *dev,
                                  const char *bus_id, u32 flags, phy_interface_t interface)
{
	if (phydev->attached_dev) {
		printk(KERN_ERR "micrel phy is already attached\n");
		return -EBUSY;
	}

	phydev->attached_dev = dev;

	phydev->dev_flags = flags;

	phydev->interface = interface;

	/* Do initial configuration here, now that
	 * we have certain key parameters
	 * (dev_flags and interface) */
	if (phydev->drv->config_init) {
		int err;

		err = phydev->drv->config_init(phydev);

		if (err < 0)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL(i2c_phy_attach);


/**
 * i2c_phy_connect - connect an ethernet device to a PHY device
 * @dev: the network device to connect
 * @bus_id: the id string of the PHY device to connect
 * @handler: callback function for state change notifications
 * @flags: PHY device's dev_flags
 * @interface: PHY device's interface
 *
 * Description: Convenience function for connecting ethernet
 *   devices to PHY devices.  The default behavior is for
 *   the PHY infrastructure to handle everything, and only notify
 *   the connected driver when the link status changes.  If you
 *   don't want, or can't use the provided functionality, you may
 *   choose to call only the subset of functions which provide
 *   the desired functionality.
 */
struct phy_device *
i2c_phy_connect(struct net_device *dev, const char *bus_id,
		void (*handler)(struct net_device *), u32 flags,
		phy_interface_t interface)
{
	struct micrel_phy_device *micrel_phy;
        int err;

        micrel_phy = get_micrel_phy(bus_id);
        if (micrel_phy == NULL)
	{
		printk(KERN_ERR "%s: no i2c Phy found\n", __FUNCTION__);
		return ERR_PTR(-ENODEV);
	}

	err = i2c_phy_attach(micrel_phy->phy_device, dev, bus_id, flags, interface);
	if (err)
		return ERR_PTR(err);

        phy_prepare_link(micrel_phy->phy_device, handler);

        start_phy(micrel_phy);

	return micrel_phy->phy_device;
}
EXPORT_SYMBOL(i2c_phy_connect);


/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x5F, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(i2c_phy);

static const struct i2c_device_id i2c_phy_id[] = {
	{ "micrel", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_phy_id);



static int i2c_phy_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
	int ret = 0;

	dev_info(&client->dev, "i2c_phy: %s chip found\n", client->name);

        if (i2c_phy_device)
        {
                i2c_set_clientdata(client, i2c_phy_device);
                i2c_phy_device->client = client;

                start_phy(i2c_phy_device);
        }
        else
        {
                printk(KERN_WARNING "%s: phy device is NULL\n", __FUNCTION__);
        }
	return ret;
}

static int i2c_phy_remove(struct i2c_client *client)
{
        struct phy_device *phydev;

        phydev = i2c_get_clientdata(client);
        phy_device_free(phydev);
        i2c_set_clientdata(client, NULL);

	return 0;
}


static struct i2c_driver micrel_i2c_driver = {
	.class		= I2C_CLASS_ALL,
	.driver = {
		.name	= "micrel_i2c_phy",
	},
	.probe		= i2c_phy_probe,
	.remove		= i2c_phy_remove,
	.id_table	= i2c_phy_id,
	.address_data	= &addr_data,
};




int
pc302_i2c_phy_init(void)
{
        /* ID of Micrel PHY */
        const u32 phy_id = 0x00221430;

        i2c_phy_device = i2c_phy_create(1, phy_id);
        if (i2c_phy_device == NULL) {
                printk(KERN_ERR "Failed to allocate micrel phy struct\n");
                return -ENOMEM;
        }

        return i2c_add_driver(&micrel_i2c_driver);
}

void
pc302_i2c_phy_uninit(void)
{
        i2c_del_driver(&micrel_i2c_driver);
}

#endif /* CONFIG_PC302_MICREL_VLAN_SWITCH */
