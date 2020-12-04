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
#ifndef PC302_I2C_PHY_H
#define PC302_I2C_PHY_H

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */


int pc302_i2c_phy_init(void);
void pc302_i2c_phy_uninit(void);

struct phy_device * i2c_phy_connect(
    struct net_device *dev, const char *bus_id,
    void (*handler)(struct net_device *), u32 flags,
    phy_interface_t interface);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* PC302_I2C_PHY_H */
