/*
 * Marvell 88e6xxx Ethernet switch single-chip support
 *
 * Copyright (c) 2008 Marvell Semiconductor
 *
 * Copyright (c) 2015 CMC Electronics, Inc.
 *	Added support for VLAN Table Unit operations
 *
 * Copyright (c) 2016 Andrew Lunn <andrew@lunn.ch>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_bridge.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <net/dsa.h>

#include "mv88e6xxx.h"
#include "global1.h"
#include "global2.h"



static struct net_device *ethdev;
extern  int ixgbe_mii_ioctl(struct net_device *netdev, struct ifreq *ifr,int cmd);

static void assert_reg_lock(struct mv88e6xxx_chip *chip)
{
	if (unlikely(!mutex_is_locked(&chip->reg_lock))) {
		dev_err(chip->dev, "Switch registers lock not held!\n");
		dump_stack();
	}
}

/* The switch ADDR[4:1] configuration pins define the chip SMI device address
 * (ADDR[0] is always zero, thus only even SMI addresses can be strapped).
 *
 * When ADDR is all zero, the chip uses Single-chip Addressing Mode, assuming it
 * is the only device connected to the SMI master. In this mode it responds to
 * all 32 possible SMI addresses, and thus maps directly the internal devices.
 *
 * When ADDR is non-zero, the chip uses Multi-chip Addressing Mode, allowing
 * multiple devices to share the SMI interface. In this mode it responds to only
 * 2 registers, used to indirectly access the internal SMI devices.
 */

static int mv88e6xxx_smi_read(struct mv88e6xxx_chip *chip,
			      int addr, int reg, u16 *val)
{
	if (!chip->smi_ops)
		return -EOPNOTSUPP;

	return chip->smi_ops->read(chip, addr, reg, val);
}

static int mv88e6xxx_smi_write(struct mv88e6xxx_chip *chip,
			       int addr, int reg, u16 val)
{
	if (!chip->smi_ops)
		return -EOPNOTSUPP;

	return chip->smi_ops->write(chip, addr, reg, val);
}


static int mdiobus_read_nested(struct mii_bus * bus, int addr, int reg)
{
	int ret;
	struct ifreq ifr;
	struct mii_ioctl_data* mii = (struct mii_ioctl_data *)(&ifr.ifr_data);
	if (!ethdev)
		return -EOPNOTSUPP;
	mii->phy_id  = addr;
	mii->reg_num = reg;
	mii->val_in  = 0;
	mii->val_out = 0;

	ret = ixgbe_mii_ioctl(ethdev, &ifr, SIOCGMIIREG);
	if(ret)
		return ret;
	printk("read success val:0x%x", mii->val_out);
	return  mii->val_out;	

}

static int mdiobus_write_nested(struct mii_bus *bus,
					   int addr, int reg, u16 val)
{
	int ret;
	struct ifreq ifr;
	struct mii_ioctl_data* mii = (struct mii_ioctl_data *)(&ifr.ifr_data);
	if (!ethdev)
		return -EOPNOTSUPP;
	mii->phy_id  = addr;
	mii->reg_num = reg;
	mii->val_in  = val;
	mii->val_out = 0;

	ret = ixgbe_mii_ioctl(ethdev, &ifr, SIOCSMIIREG);
	if(ret)
		return ret;
	val= mii->val_out;
	printk("write success val:0x%x", val);

	return 0;
}


static int mv88e6xxx_smi_single_chip_read(struct mv88e6xxx_chip *chip,
					  int addr, int reg, u16 *val)
{
	int ret;

	ret = mdiobus_read_nested(chip->bus, addr, reg);
	if (ret < 0)
		return ret;

	*val = ret & 0xffff;

	return 0;
}

static int mv88e6xxx_smi_single_chip_write(struct mv88e6xxx_chip *chip,
					   int addr, int reg, u16 val)
{
	int ret;

	ret = mdiobus_write_nested(chip->bus, addr, reg, val);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct mv88e6xxx_bus_ops mv88e6xxx_smi_single_chip_ops = {
	.read = mv88e6xxx_smi_single_chip_read,
	.write = mv88e6xxx_smi_single_chip_write,
};

static int mv88e6xxx_smi_multi_chip_wait(struct mv88e6xxx_chip *chip)
{
	int ret;
	int i;

	for (i = 0; i < 16; i++) {
		ret = mdiobus_read_nested(chip->bus, chip->sw_addr, SMI_CMD);
		if (ret < 0)
			return ret;

		if ((ret & SMI_CMD_BUSY) == 0)
			return 0;
	}

	return -ETIMEDOUT;
}

static int mv88e6xxx_smi_multi_chip_read(struct mv88e6xxx_chip *chip,
					 int addr, int reg, u16 *val)
{
	int ret;

	/* Wait for the bus to become free. */
	ret = mv88e6xxx_smi_multi_chip_wait(chip);
	if (ret < 0)
		return ret;

	/* Transmit the read command. */
	ret = mdiobus_write_nested(chip->bus, chip->sw_addr, SMI_CMD,
				   SMI_CMD_OP_22_READ | (addr << 5) | reg);
	if (ret < 0)
		return ret;

	/* Wait for the read command to complete. */
	ret = mv88e6xxx_smi_multi_chip_wait(chip);
	if (ret < 0)
		return ret;

	/* Read the data. */
	ret = mdiobus_read_nested(chip->bus, chip->sw_addr, SMI_DATA);
	if (ret < 0)
		return ret;

	*val = ret & 0xffff;

	return 0;
}

static int mv88e6xxx_smi_multi_chip_write(struct mv88e6xxx_chip *chip,
					  int addr, int reg, u16 val)
{
	int ret;

	/* Wait for the bus to become free. */
	ret = mv88e6xxx_smi_multi_chip_wait(chip);
	if (ret < 0)
		return ret;

	/* Transmit the data to write. */
	ret = mdiobus_write_nested(chip->bus, chip->sw_addr, SMI_DATA, val);
	if (ret < 0)
		return ret;

	/* Transmit the write command. */
	ret = mdiobus_write_nested(chip->bus, chip->sw_addr, SMI_CMD,
				   SMI_CMD_OP_22_WRITE | (addr << 5) | reg);
	if (ret < 0)
		return ret;

	/* Wait for the write command to complete. */
	ret = mv88e6xxx_smi_multi_chip_wait(chip);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct mv88e6xxx_bus_ops mv88e6xxx_smi_multi_chip_ops = {
	.read = mv88e6xxx_smi_multi_chip_read,
	.write = mv88e6xxx_smi_multi_chip_write,
};

int mv88e6xxx_read(struct mv88e6xxx_chip *chip, int addr, int reg, u16 *val)
{
	int err;

	assert_reg_lock(chip);

	err = mv88e6xxx_smi_read(chip, addr, reg, val);
	if (err)
		return err;

	printk("<- addr: 0x%.2x reg: 0x%.2x val: 0x%.4x\n",
		addr, reg, *val);

	return 0;
}

int mv88e6xxx_write(struct mv88e6xxx_chip *chip, int addr, int reg, u16 val)
{
	int err;

	assert_reg_lock(chip);

	err = mv88e6xxx_smi_write(chip, addr, reg, val);
	if (err)
		return err;

	printk("-> addr: 0x%.2x reg: 0x%.2x val: 0x%.4x\n",
		addr, reg, val);

	return 0;
}

static int mv88e6xxx_port_read(struct mv88e6xxx_chip *chip, int port, int reg,
			       u16 *val)
{
	int addr = chip->info->port_base_addr + port;

	return mv88e6xxx_read(chip, addr, reg, val);
}

static int mv88e6xxx_port_write(struct mv88e6xxx_chip *chip, int port, int reg,
				u16 val)
{
	int addr = chip->info->port_base_addr + port;

	return mv88e6xxx_write(chip, addr, reg, val);
}

static int mv88e6xxx_phy_read(struct mv88e6xxx_chip *chip, int phy,
			      int reg, u16 *val)
{
	int addr = phy; /* PHY devices addresses start at 0x0 */

	if (!chip->info->ops->phy_read)
		return -EOPNOTSUPP;

	return chip->info->ops->phy_read(chip, addr, reg, val);
}

static int mv88e6xxx_phy_write(struct mv88e6xxx_chip *chip, int phy,
			       int reg, u16 val)
{
	int addr = phy; /* PHY devices addresses start at 0x0 */

	if (!chip->info->ops->phy_write)
		return -EOPNOTSUPP;

	return chip->info->ops->phy_write(chip, addr, reg, val);
}

static int mv88e6xxx_phy_page_get(struct mv88e6xxx_chip *chip, int phy, u8 page)
{
	if (!mv88e6xxx_has(chip, MV88E6XXX_FLAG_PHY_PAGE))
		return -EOPNOTSUPP;

	return mv88e6xxx_phy_write(chip, phy, PHY_PAGE, page);
}

static void mv88e6xxx_phy_page_put(struct mv88e6xxx_chip *chip, int phy)
{
	int err;

	/* Restore PHY page Copper 0x0 for access via the registered MDIO bus */
	err = mv88e6xxx_phy_write(chip, phy, PHY_PAGE, PHY_PAGE_COPPER);
	if (unlikely(err)) {
		printk("failed to restore PHY %d page Copper (%d)\n",
			phy, err);
	}
}

static int mv88e6xxx_phy_page_read(struct mv88e6xxx_chip *chip, int phy,
				   u8 page, int reg, u16 *val)
{
	int err;

	/* There is no paging for registers 22 */
	if (reg == PHY_PAGE)
		return -EINVAL;

	err = mv88e6xxx_phy_page_get(chip, phy, page);
	if (!err) {
		err = mv88e6xxx_phy_read(chip, phy, reg, val);
		mv88e6xxx_phy_page_put(chip, phy);
	}

	return err;
}

static int mv88e6xxx_phy_page_write(struct mv88e6xxx_chip *chip, int phy,
				    u8 page, int reg, u16 val)
{
	int err;

	/* There is no paging for registers 22 */
	if (reg == PHY_PAGE)
		return -EINVAL;

	err = mv88e6xxx_phy_page_get(chip, phy, page);
	if (!err) {
		err = mv88e6xxx_phy_write(chip, phy, PHY_PAGE, page);
		mv88e6xxx_phy_page_put(chip, phy);
	}

	return err;
}

static int mv88e6xxx_serdes_read(struct mv88e6xxx_chip *chip, int reg, u16 *val)
{
	return mv88e6xxx_phy_page_read(chip, ADDR_SERDES, SERDES_PAGE_FIBER,
				       reg, val);
}

static int mv88e6xxx_serdes_write(struct mv88e6xxx_chip *chip, int reg, u16 val)
{
	return mv88e6xxx_phy_page_write(chip, ADDR_SERDES, SERDES_PAGE_FIBER,
					reg, val);
}

int mv88e6xxx_wait(struct mv88e6xxx_chip *chip, int addr, int reg, u16 mask)
{
	int i;

	for (i = 0; i < 16; i++) {
		u16 val;
		int err;

		err = mv88e6xxx_read(chip, addr, reg, &val);
		if (err)
			return err;

		if (!(val & mask))
			return 0;

		usleep_range(1000, 2000);
	}

	printk("Timeout while waiting for switch\n");
	return -ETIMEDOUT;
}

/* Indirect write to single pointer-data register with an Update bit */
int mv88e6xxx_update(struct mv88e6xxx_chip *chip, int addr, int reg, u16 update)
{
	u16 val;
	int err;

	/* Wait until the previous operation is completed */
	err = mv88e6xxx_wait(chip, addr, reg, BIT(15));
	if (err)
		return err;

	/* Set the Update bit to trigger a write operation */
	val = BIT(15) | update;

	return mv88e6xxx_write(chip, addr, reg, val);
}

static int mv88e6xxx_ppu_disable(struct mv88e6xxx_chip *chip)
{
	u16 val;
	int i, err;

	err = mv88e6xxx_g1_read(chip, GLOBAL_CONTROL, &val);
	if (err)
		return err;

	err = mv88e6xxx_g1_write(chip, GLOBAL_CONTROL,
				 val & ~GLOBAL_CONTROL_PPU_ENABLE);
	if (err)
		return err;

	for (i = 0; i < 16; i++) {
		err = mv88e6xxx_g1_read(chip, GLOBAL_STATUS, &val);
		if (err)
			return err;

		usleep_range(1000, 2000);
		if ((val & GLOBAL_STATUS_PPU_MASK) != GLOBAL_STATUS_PPU_POLLING)
			return 0;
	}

	return -ETIMEDOUT;
}

static int mv88e6xxx_ppu_enable(struct mv88e6xxx_chip *chip)
{
	u16 val;
	int i, err;

	err = mv88e6xxx_g1_read(chip, GLOBAL_CONTROL, &val);
	if (err)
		return err;

	err = mv88e6xxx_g1_write(chip, GLOBAL_CONTROL,
				 val | GLOBAL_CONTROL_PPU_ENABLE);
	if (err)
		return err;

	for (i = 0; i < 16; i++) {
		err = mv88e6xxx_g1_read(chip, GLOBAL_STATUS, &val);
		if (err)
			return err;

		usleep_range(1000, 2000);
		if ((val & GLOBAL_STATUS_PPU_MASK) == GLOBAL_STATUS_PPU_POLLING)
			return 0;
	}

	return -ETIMEDOUT;
}

static void mv88e6xxx_ppu_reenable_work(struct work_struct *ugly)
{
	struct mv88e6xxx_chip *chip;

	chip = container_of(ugly, struct mv88e6xxx_chip, ppu_work);

	mutex_lock(&chip->reg_lock);

	if (mutex_trylock(&chip->ppu_mutex)) {
		if (mv88e6xxx_ppu_enable(chip) == 0)
			chip->ppu_disabled = 0;
		mutex_unlock(&chip->ppu_mutex);
	}

	mutex_unlock(&chip->reg_lock);
}

static void mv88e6xxx_ppu_reenable_timer(unsigned long _ps)
{
	struct mv88e6xxx_chip *chip = (void *)_ps;

	schedule_work(&chip->ppu_work);
}

static int mv88e6xxx_ppu_access_get(struct mv88e6xxx_chip *chip)
{
	int ret;

	mutex_lock(&chip->ppu_mutex);

	/* If the PHY polling unit is enabled, disable it so that
	 * we can access the PHY registers.  If it was already
	 * disabled, cancel the timer that is going to re-enable
	 * it.
	 */
	if (!chip->ppu_disabled) {
		ret = mv88e6xxx_ppu_disable(chip);
		if (ret < 0) {
			mutex_unlock(&chip->ppu_mutex);
			return ret;
		}
		chip->ppu_disabled = 1;
	} else {
		del_timer(&chip->ppu_timer);
		ret = 0;
	}

	return ret;
}

static void mv88e6xxx_ppu_access_put(struct mv88e6xxx_chip *chip)
{
	/* Schedule a timer to re-enable the PHY polling unit. */
	mod_timer(&chip->ppu_timer, jiffies + msecs_to_jiffies(10));
	mutex_unlock(&chip->ppu_mutex);
}

static void mv88e6xxx_ppu_state_init(struct mv88e6xxx_chip *chip)
{
	mutex_init(&chip->ppu_mutex);
	INIT_WORK(&chip->ppu_work, mv88e6xxx_ppu_reenable_work);
	init_timer(&chip->ppu_timer);
	chip->ppu_timer.data = (unsigned long)chip;
	chip->ppu_timer.function = mv88e6xxx_ppu_reenable_timer;
}

static void mv88e6xxx_ppu_state_destroy(struct mv88e6xxx_chip *chip)
{
	del_timer_sync(&chip->ppu_timer);
}

static int mv88e6xxx_phy_ppu_read(struct mv88e6xxx_chip *chip, int addr,
				  int reg, u16 *val)
{
	int err;

	err = mv88e6xxx_ppu_access_get(chip);
	if (!err) {
		err = mv88e6xxx_read(chip, addr, reg, val);
		mv88e6xxx_ppu_access_put(chip);
	}

	return err;
}

static int mv88e6xxx_phy_ppu_write(struct mv88e6xxx_chip *chip, int addr,
				   int reg, u16 val)
{
	int err;

	err = mv88e6xxx_ppu_access_get(chip);
	if (!err) {
		err = mv88e6xxx_write(chip, addr, reg, val);
		mv88e6xxx_ppu_access_put(chip);
	}

	return err;
}

static bool mv88e6xxx_6065_family(struct mv88e6xxx_chip *chip)
{
	return chip->info->family == MV88E6XXX_FAMILY_6065;
}

static bool mv88e6xxx_6095_family(struct mv88e6xxx_chip *chip)
{
	return chip->info->family == MV88E6XXX_FAMILY_6095;
}

static bool mv88e6xxx_6097_family(struct mv88e6xxx_chip *chip)
{
	return chip->info->family == MV88E6XXX_FAMILY_6097;
}

static bool mv88e6xxx_6165_family(struct mv88e6xxx_chip *chip)
{
	return chip->info->family == MV88E6XXX_FAMILY_6165;
}

static bool mv88e6xxx_6185_family(struct mv88e6xxx_chip *chip)
{
	return chip->info->family == MV88E6XXX_FAMILY_6185;
}

static bool mv88e6xxx_6320_family(struct mv88e6xxx_chip *chip)
{
	return chip->info->family == MV88E6XXX_FAMILY_6320;
}

static bool mv88e6xxx_6351_family(struct mv88e6xxx_chip *chip)
{
	return chip->info->family == MV88E6XXX_FAMILY_6351;
}

static bool mv88e6xxx_6352_family(struct mv88e6xxx_chip *chip)
{
	return chip->info->family == MV88E6XXX_FAMILY_6352;
}

/* We expect the switch to perform auto negotiation if there is a real
 * phy. However, in the case of a fixed link phy, we force the port
 * settings from the fixed link settings.
 */
static void mv88e6xxx_adjust_link(struct dsa_switch *ds, int port,
				  struct phy_device *phydev)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	u16 reg;
	int err;

	//if (!phy_is_pseudo_fixed_link(phydev))
	//	return;

	mutex_lock(&chip->reg_lock);

	err = mv88e6xxx_port_read(chip, port, PORT_PCS_CTRL, &reg);
	if (err)
		goto out;

	reg &= ~(PORT_PCS_CTRL_LINK_UP |
		 PORT_PCS_CTRL_FORCE_LINK |
		 PORT_PCS_CTRL_DUPLEX_FULL |
		 PORT_PCS_CTRL_FORCE_DUPLEX |
		 PORT_PCS_CTRL_UNFORCED);

	reg |= PORT_PCS_CTRL_FORCE_LINK;
	if (phydev->link)
		reg |= PORT_PCS_CTRL_LINK_UP;

	if (mv88e6xxx_6065_family(chip) && phydev->speed > SPEED_100)
		goto out;

	switch (phydev->speed) {
	case SPEED_1000:
		reg |= PORT_PCS_CTRL_1000;
		break;
	case SPEED_100:
		reg |= PORT_PCS_CTRL_100;
		break;
	case SPEED_10:
		reg |= PORT_PCS_CTRL_10;
		break;
	default:
		pr_info("Unknown speed");
		goto out;
	}

	reg |= PORT_PCS_CTRL_FORCE_DUPLEX;
	if (phydev->duplex == DUPLEX_FULL)
		reg |= PORT_PCS_CTRL_DUPLEX_FULL;

	if ((mv88e6xxx_6352_family(chip) || mv88e6xxx_6351_family(chip)) &&
	    (port >= mv88e6xxx_num_ports(chip) - 2)) {
		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID)
			reg |= PORT_PCS_CTRL_RGMII_DELAY_RXCLK;
		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID)
			reg |= PORT_PCS_CTRL_RGMII_DELAY_TXCLK;
		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
			reg |= (PORT_PCS_CTRL_RGMII_DELAY_RXCLK |
				PORT_PCS_CTRL_RGMII_DELAY_TXCLK);
	}
	mv88e6xxx_port_write(chip, port, PORT_PCS_CTRL, reg);

out:
	mutex_unlock(&chip->reg_lock);
}

static int _mv88e6xxx_stats_wait(struct mv88e6xxx_chip *chip)
{
	u16 val;
	int i, err;

	for (i = 0; i < 10; i++) {
		err = mv88e6xxx_g1_read(chip, GLOBAL_STATS_OP, &val);
		if ((val & GLOBAL_STATS_OP_BUSY) == 0)
			return 0;
	}

	return -ETIMEDOUT;
}

static int _mv88e6xxx_stats_snapshot(struct mv88e6xxx_chip *chip, int port)
{
	int err;

	if (mv88e6xxx_6320_family(chip) || mv88e6xxx_6352_family(chip))
		port = (port + 1) << 5;

	/* Snapshot the hardware statistics counters for this port. */
	err = mv88e6xxx_g1_write(chip, GLOBAL_STATS_OP,
				 GLOBAL_STATS_OP_CAPTURE_PORT |
				 GLOBAL_STATS_OP_HIST_RX_TX | port);
	if (err)
		return err;

	/* Wait for the snapshotting to complete. */
	return _mv88e6xxx_stats_wait(chip);
}

static void _mv88e6xxx_stats_read(struct mv88e6xxx_chip *chip,
				  int stat, u32 *val)
{
	u32 value;
	u16 reg;
	int err;

	*val = 0;

	err = mv88e6xxx_g1_write(chip, GLOBAL_STATS_OP,
				 GLOBAL_STATS_OP_READ_CAPTURED |
				 GLOBAL_STATS_OP_HIST_RX_TX | stat);
	if (err)
		return;

	err = _mv88e6xxx_stats_wait(chip);
	if (err)
		return;

	err = mv88e6xxx_g1_read(chip, GLOBAL_STATS_COUNTER_32, &reg);
	if (err)
		return;

	value = reg << 16;

	err = mv88e6xxx_g1_read(chip, GLOBAL_STATS_COUNTER_01, &reg);
	if (err)
		return;

	*val = value | reg;
}

static struct mv88e6xxx_hw_stat mv88e6xxx_hw_stats[] = {
	{ "in_good_octets",	8, 0x00, BANK0, },
	{ "in_bad_octets",	4, 0x02, BANK0, },
	{ "in_unicast",		4, 0x04, BANK0, },
	{ "in_broadcasts",	4, 0x06, BANK0, },
	{ "in_multicasts",	4, 0x07, BANK0, },
	{ "in_pause",		4, 0x16, BANK0, },
	{ "in_undersize",	4, 0x18, BANK0, },
	{ "in_fragments",	4, 0x19, BANK0, },
	{ "in_oversize",	4, 0x1a, BANK0, },
	{ "in_jabber",		4, 0x1b, BANK0, },
	{ "in_rx_error",	4, 0x1c, BANK0, },
	{ "in_fcs_error",	4, 0x1d, BANK0, },
	{ "out_octets",		8, 0x0e, BANK0, },
	{ "out_unicast",	4, 0x10, BANK0, },
	{ "out_broadcasts",	4, 0x13, BANK0, },
	{ "out_multicasts",	4, 0x12, BANK0, },
	{ "out_pause",		4, 0x15, BANK0, },
	{ "excessive",		4, 0x11, BANK0, },
	{ "collisions",		4, 0x1e, BANK0, },
	{ "deferred",		4, 0x05, BANK0, },
	{ "single",		4, 0x14, BANK0, },
	{ "multiple",		4, 0x17, BANK0, },
	{ "out_fcs_error",	4, 0x03, BANK0, },
	{ "late",		4, 0x1f, BANK0, },
	{ "hist_64bytes",	4, 0x08, BANK0, },
	{ "hist_65_127bytes",	4, 0x09, BANK0, },
	{ "hist_128_255bytes",	4, 0x0a, BANK0, },
	{ "hist_256_511bytes",	4, 0x0b, BANK0, },
	{ "hist_512_1023bytes", 4, 0x0c, BANK0, },
	{ "hist_1024_max_bytes", 4, 0x0d, BANK0, },
	{ "sw_in_discards",	4, 0x10, PORT, },
	{ "sw_in_filtered",	2, 0x12, PORT, },
	{ "sw_out_filtered",	2, 0x13, PORT, },
	{ "in_discards",	4, 0x00 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_filtered",	4, 0x01 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_accepted",	4, 0x02 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_bad_accepted",	4, 0x03 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_good_avb_class_a", 4, 0x04 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_good_avb_class_b", 4, 0x05 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_bad_avb_class_a", 4, 0x06 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_bad_avb_class_b", 4, 0x07 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "tcam_counter_0",	4, 0x08 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "tcam_counter_1",	4, 0x09 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "tcam_counter_2",	4, 0x0a | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "tcam_counter_3",	4, 0x0b | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_da_unknown",	4, 0x0e | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "in_management",	4, 0x0f | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_queue_0",	4, 0x10 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_queue_1",	4, 0x11 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_queue_2",	4, 0x12 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_queue_3",	4, 0x13 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_queue_4",	4, 0x14 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_queue_5",	4, 0x15 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_queue_6",	4, 0x16 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_queue_7",	4, 0x17 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_cut_through",	4, 0x18 | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_octets_a",	4, 0x1a | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_octets_b",	4, 0x1b | GLOBAL_STATS_OP_BANK_1, BANK1, },
	{ "out_management",	4, 0x1f | GLOBAL_STATS_OP_BANK_1, BANK1, },
};

static bool mv88e6xxx_has_stat(struct mv88e6xxx_chip *chip,
			       struct mv88e6xxx_hw_stat *stat)
{
	switch (stat->type) {
	case BANK0:
		return true;
	case BANK1:
		return mv88e6xxx_6320_family(chip);
	case PORT:
		return mv88e6xxx_6095_family(chip) ||
			mv88e6xxx_6185_family(chip) ||
			mv88e6xxx_6097_family(chip) ||
			mv88e6xxx_6165_family(chip) ||
			mv88e6xxx_6351_family(chip) ||
			mv88e6xxx_6352_family(chip);
	}
	return false;
}

static uint64_t _mv88e6xxx_get_ethtool_stat(struct mv88e6xxx_chip *chip,
					    struct mv88e6xxx_hw_stat *s,
					    int port)
{
	u32 low;
	u32 high = 0;
	int err;
	u16 reg;
	u64 value;

	switch (s->type) {
	case PORT:
		err = mv88e6xxx_port_read(chip, port, s->reg, &reg);
		if (err)
			return UINT64_MAX;

		low = reg;
		if (s->sizeof_stat == 4) {
			err = mv88e6xxx_port_read(chip, port, s->reg + 1, &reg);
			if (err)
				return UINT64_MAX;
			high = reg;
		}
		break;
	case BANK0:
	case BANK1:
		_mv88e6xxx_stats_read(chip, s->reg, &low);
		if (s->sizeof_stat == 8)
			_mv88e6xxx_stats_read(chip, s->reg + 1, &high);
	}
	value = (((u64)high) << 16) | low;
	return value;
}

static void mv88e6xxx_get_strings(struct dsa_switch *ds, int port,
				  uint8_t *data)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	struct mv88e6xxx_hw_stat *stat;
	int i, j;

	for (i = 0, j = 0; i < ARRAY_SIZE(mv88e6xxx_hw_stats); i++) {
		stat = &mv88e6xxx_hw_stats[i];
		if (mv88e6xxx_has_stat(chip, stat)) {
			memcpy(data + j * ETH_GSTRING_LEN, stat->string,
			       ETH_GSTRING_LEN);
			j++;
		}
	}
}

static int mv88e6xxx_get_sset_count(struct dsa_switch *ds)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	struct mv88e6xxx_hw_stat *stat;
	int i, j;

	for (i = 0, j = 0; i < ARRAY_SIZE(mv88e6xxx_hw_stats); i++) {
		stat = &mv88e6xxx_hw_stats[i];
		if (mv88e6xxx_has_stat(chip, stat))
			j++;
	}
	return j;
}

static void mv88e6xxx_get_ethtool_stats(struct dsa_switch *ds, int port,
					uint64_t *data)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	struct mv88e6xxx_hw_stat *stat;
	int ret;
	int i, j;

	mutex_lock(&chip->reg_lock);

	ret = _mv88e6xxx_stats_snapshot(chip, port);
	if (ret < 0) {
		mutex_unlock(&chip->reg_lock);
		return;
	}
	for (i = 0, j = 0; i < ARRAY_SIZE(mv88e6xxx_hw_stats); i++) {
		stat = &mv88e6xxx_hw_stats[i];
		if (mv88e6xxx_has_stat(chip, stat)) {
			data[j] = _mv88e6xxx_get_ethtool_stat(chip, stat, port);
			j++;
		}
	}

	mutex_unlock(&chip->reg_lock);
}

static int mv88e6xxx_get_regs_len(struct dsa_switch *ds, int port)
{
	return 32 * sizeof(u16);
}

static void mv88e6xxx_get_regs(struct dsa_switch *ds, int port,
			       struct ethtool_regs *regs, void *_p)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int err;
	u16 reg;
	u16 *p = _p;
	int i;

	regs->version = 0;

	memset(p, 0xff, 32 * sizeof(u16));

	mutex_lock(&chip->reg_lock);

	for (i = 0; i < 32; i++) {

		err = mv88e6xxx_port_read(chip, port, i, &reg);
		if (!err)
			p[i] = reg;
	}

	mutex_unlock(&chip->reg_lock);
}

static int _mv88e6xxx_atu_wait(struct mv88e6xxx_chip *chip)
{
	return mv88e6xxx_g1_wait(chip, GLOBAL_ATU_OP, GLOBAL_ATU_OP_BUSY);
}

static int mv88e6xxx_get_eee(struct dsa_switch *ds, int port,
			     struct ethtool_eee *e)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	u16 reg;
	int err;

	if (!mv88e6xxx_has(chip, MV88E6XXX_FLAG_EEE))
		return -EOPNOTSUPP;

	mutex_lock(&chip->reg_lock);

	err = mv88e6xxx_phy_read(chip, port, 16, &reg);
	if (err)
		goto out;

	e->eee_enabled = !!(reg & 0x0200);
	e->tx_lpi_enabled = !!(reg & 0x0100);

	err = mv88e6xxx_port_read(chip, port, PORT_STATUS, &reg);
	if (err)
		goto out;

	e->eee_active = !!(reg & PORT_STATUS_EEE);
out:
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_set_eee(struct dsa_switch *ds, int port,
			     struct phy_device *phydev, struct ethtool_eee *e)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	u16 reg;
	int err;

	if (!mv88e6xxx_has(chip, MV88E6XXX_FLAG_EEE))
		return -EOPNOTSUPP;

	mutex_lock(&chip->reg_lock);

	err = mv88e6xxx_phy_read(chip, port, 16, &reg);
	if (err)
		goto out;

	reg &= ~0x0300;
	if (e->eee_enabled)
		reg |= 0x0200;
	if (e->tx_lpi_enabled)
		reg |= 0x0100;

	err = mv88e6xxx_phy_write(chip, port, 16, reg);
out:
	mutex_unlock(&chip->reg_lock);

	return err;
}
		
static int mv88e6xxx_port_bridge_join(struct dsa_switch *ds, int port,
				      struct net_device *bridge)
{
#if 0
	struct mv88e6xxx_chip *chip = ds->priv;
	int i, err = 0;

	mutex_lock(&chip->reg_lock);

	/* Assign the bridge and remap each port's VLANTable */
	chip->ports[port].bridge_dev = bridge;

	for (i = 0; i < mv88e6xxx_num_ports(chip); ++i) {
		if (chip->ports[i].bridge_dev == bridge) {
			err = _mv88e6xxx_port_based_vlan_map(chip, i);
			if (err)
				break;
		}
	}

	mutex_unlock(&chip->reg_lock);

	return err;
#else
	return 0;
#endif
}

static void mv88e6xxx_port_bridge_leave(struct dsa_switch *ds, int port)
{
#if 0
	struct mv88e6xxx_chip *chip = ds->priv;
	struct net_device *bridge = chip->ports[port].bridge_dev;
	int i;

	mutex_lock(&chip->reg_lock);

	/* Unassign the bridge and remap each port's VLANTable */
	chip->ports[port].bridge_dev = NULL;

	for (i = 0; i < mv88e6xxx_num_ports(chip); ++i)
		if (i == port || chip->ports[i].bridge_dev == bridge)
			if (_mv88e6xxx_port_based_vlan_map(chip, i))
				printk("failed to remap\n");

	mutex_unlock(&chip->reg_lock);
#else
		return 0;
#endif


}

static int mv88e6xxx_switch_reset(struct mv88e6xxx_chip *chip)
{
	bool ppu_active = mv88e6xxx_has(chip, MV88E6XXX_FLAG_PPU_ACTIVE);
	u16 is_reset = (ppu_active ? 0x8800 : 0xc800);
	struct gpio_desc *gpiod = chip->reset;
	unsigned long timeout;
	u16 reg;
	int err;
	int i;

	/* Set all ports to the disabled state. */
	for (i = 0; i < mv88e6xxx_num_ports(chip); i++) {
		err = mv88e6xxx_port_read(chip, i, PORT_CONTROL, &reg);
		if (err)
			return err;

		err = mv88e6xxx_port_write(chip, i, PORT_CONTROL,
					   reg & 0xfffc);
		if (err)
			return err;
	}

	/* Wait for transmit queues to drain. */
	usleep_range(2000, 4000);

	/* Reset the switch. Keep the PPU active if requested. The PPU
	 * needs to be active to support indirect phy register access
	 * through global registers 0x18 and 0x19.
	 */
	if (ppu_active)
		err = mv88e6xxx_g1_write(chip, 0x04, 0xc000);
	else
		err = mv88e6xxx_g1_write(chip, 0x04, 0xc400);
	if (err)
		return err;

	/* Wait up to one second for reset to complete. */
	timeout = jiffies + 1 * HZ;
	while (time_before(jiffies, timeout)) {
		err = mv88e6xxx_g1_read(chip, 0x00, &reg);
		if (err)
			return err;

		if ((reg & is_reset) == is_reset)
			break;
		usleep_range(1000, 2000);
	}
	if (time_after(jiffies, timeout))
		err = -ETIMEDOUT;
	else
		err = 0;

	return err;
}

static int mv88e6xxx_serdes_power_on(struct mv88e6xxx_chip *chip)
{
	u16 val;
	int err;

	/* Clear Power Down bit */
	err = mv88e6xxx_serdes_read(chip, MII_BMCR, &val);
	if (err)
		return err;

	if (val & BMCR_PDOWN) {
		val &= ~BMCR_PDOWN;
		err = mv88e6xxx_serdes_write(chip, MII_BMCR, val);
	}

	return err;
}

static int mv88e6xxx_setup_port(struct mv88e6xxx_chip *chip, int port)
{
	struct dsa_switch *ds = chip->ds;
	int err;
	u16 reg;

	if (mv88e6xxx_6352_family(chip) || mv88e6xxx_6351_family(chip) ||
	    mv88e6xxx_6165_family(chip) || mv88e6xxx_6097_family(chip) ||
	    mv88e6xxx_6185_family(chip) || mv88e6xxx_6095_family(chip) ||
	    mv88e6xxx_6065_family(chip) || mv88e6xxx_6320_family(chip)) {
		/* MAC Forcing register: don't force link, speed,
		 * duplex or flow control state to any particular
		 * values on physical ports, but force the CPU port
		 * and all DSA ports to their maximum bandwidth and
		 * full duplex.
		 */
		err = mv88e6xxx_port_read(chip, port, PORT_PCS_CTRL, &reg);
		if (dsa_is_cpu_port(ds, port) || dsa_is_dsa_port(ds, port)) {
			reg &= ~PORT_PCS_CTRL_UNFORCED;
			reg |= PORT_PCS_CTRL_FORCE_LINK |
				PORT_PCS_CTRL_LINK_UP |
				PORT_PCS_CTRL_DUPLEX_FULL |
				PORT_PCS_CTRL_FORCE_DUPLEX;
			if (mv88e6xxx_6065_family(chip))
				reg |= PORT_PCS_CTRL_100;
			else
				reg |= PORT_PCS_CTRL_1000;
		} else {
			reg |= PORT_PCS_CTRL_UNFORCED;
		}

		err = mv88e6xxx_port_write(chip, port, PORT_PCS_CTRL, reg);
		if (err)
			return err;
	}

	/* Port Control: disable Drop-on-Unlock, disable Drop-on-Lock,
	 * disable Header mode, enable IGMP/MLD snooping, disable VLAN
	 * tunneling, determine priority by looking at 802.1p and IP
	 * priority fields (IP prio has precedence), and set STP state
	 * to Forwarding.
	 *
	 * If this is the CPU link, use DSA or EDSA tagging depending
	 * on which tagging mode was configured.
	 *
	 * If this is a link to another switch, use DSA tagging mode.
	 *
	 * If this is the upstream port for this switch, enable
	 * forwarding of unknown unicasts and multicasts.
	 */
	reg = 0;
	if (mv88e6xxx_6352_family(chip) || mv88e6xxx_6351_family(chip) ||
	    mv88e6xxx_6165_family(chip) || mv88e6xxx_6097_family(chip) ||
	    mv88e6xxx_6095_family(chip) || mv88e6xxx_6065_family(chip) ||
	    mv88e6xxx_6185_family(chip) || mv88e6xxx_6320_family(chip))
		reg = PORT_CONTROL_IGMP_MLD_SNOOP |
		PORT_CONTROL_USE_TAG | PORT_CONTROL_USE_IP |
		PORT_CONTROL_STATE_FORWARDING;
	if (dsa_is_cpu_port(ds, port)) {
		if (mv88e6xxx_has(chip, MV88E6XXX_FLAG_EDSA))
			reg |= PORT_CONTROL_FRAME_ETHER_TYPE_DSA |
				PORT_CONTROL_FORWARD_UNKNOWN_MC;
		else
			reg |= PORT_CONTROL_DSA_TAG;
		reg |= PORT_CONTROL_EGRESS_ADD_TAG |
			PORT_CONTROL_FORWARD_UNKNOWN;
	}
	if (dsa_is_dsa_port(ds, port)) {
		if (mv88e6xxx_6095_family(chip) ||
		    mv88e6xxx_6185_family(chip))
			reg |= PORT_CONTROL_DSA_TAG;
		if (mv88e6xxx_6352_family(chip) ||
		    mv88e6xxx_6351_family(chip) ||
		    mv88e6xxx_6165_family(chip) ||
		    mv88e6xxx_6097_family(chip) ||
		    mv88e6xxx_6320_family(chip)) {
			reg |= PORT_CONTROL_FRAME_MODE_DSA;
		}

		if (port == dsa_upstream_port(ds))
			reg |= PORT_CONTROL_FORWARD_UNKNOWN |
				PORT_CONTROL_FORWARD_UNKNOWN_MC;
	}
	if (reg) {
		err = mv88e6xxx_port_write(chip, port, PORT_CONTROL, reg);
		if (err)
			return err;
	}

	/* If this port is connected to a SerDes, make sure the SerDes is not
	 * powered down.
	 */
	if (mv88e6xxx_has(chip, MV88E6XXX_FLAGS_SERDES)) {
		err = mv88e6xxx_port_read(chip, port, PORT_STATUS, &reg);
		if (err)
			return err;
		reg &= PORT_STATUS_CMODE_MASK;
		if ((reg == PORT_STATUS_CMODE_100BASE_X) ||
		    (reg == PORT_STATUS_CMODE_1000BASE_X) ||
		    (reg == PORT_STATUS_CMODE_SGMII)) {
			err = mv88e6xxx_serdes_power_on(chip);
			if (err < 0)
				return err;
		}
	}

	/* Port Control 2: don't force a good FCS, set the maximum frame size to
	 * 10240 bytes, disable 802.1q tags checking, don't discard tagged or
	 * untagged frames on this port, do a destination address lookup on all
	 * received packets as usual, disable ARP mirroring and don't send a
	 * copy of all transmitted/received frames on this port to the CPU.
	 */
	reg = 0;
	if (mv88e6xxx_6352_family(chip) || mv88e6xxx_6351_family(chip) ||
	    mv88e6xxx_6165_family(chip) || mv88e6xxx_6097_family(chip) ||
	    mv88e6xxx_6095_family(chip) || mv88e6xxx_6320_family(chip) ||
	    mv88e6xxx_6185_family(chip))
		reg = PORT_CONTROL_2_MAP_DA;

	if (mv88e6xxx_6352_family(chip) || mv88e6xxx_6351_family(chip) ||
	    mv88e6xxx_6165_family(chip) || mv88e6xxx_6320_family(chip))
		reg |= PORT_CONTROL_2_JUMBO_10240;

	if (mv88e6xxx_6095_family(chip) || mv88e6xxx_6185_family(chip)) {
		/* Set the upstream port this port should use */
		reg |= dsa_upstream_port(ds);
		/* enable forwarding of unknown multicast addresses to
		 * the upstream port
		 */
		if (port == dsa_upstream_port(ds))
			reg |= PORT_CONTROL_2_FORWARD_UNKNOWN;
	}

	reg |= PORT_CONTROL_2_8021Q_DISABLED;

	if (reg) {
		err = mv88e6xxx_port_write(chip, port, PORT_CONTROL_2, reg);
		if (err)
			return err;
	}

	/* Port Association Vector: when learning source addresses
	 * of packets, add the address to the address database using
	 * a port bitmap that has only the bit for this port set and
	 * the other bits clear.
	 */
	reg = 1 << port;
	/* Disable learning for CPU port */
	if (dsa_is_cpu_port(ds, port))
		reg = 0;

	err = mv88e6xxx_port_write(chip, port, PORT_ASSOC_VECTOR, reg);
	if (err)
		return err;

	/* Egress rate control 2: disable egress rate control. */
	err = mv88e6xxx_port_write(chip, port, PORT_RATE_CONTROL_2, 0x0000);
	if (err)
		return err;

	if (mv88e6xxx_6352_family(chip) || mv88e6xxx_6351_family(chip) ||
	    mv88e6xxx_6165_family(chip) || mv88e6xxx_6097_family(chip) ||
	    mv88e6xxx_6320_family(chip)) {
		/* Do not limit the period of time that this port can
		 * be paused for by the remote end or the period of
		 * time that this port can pause the remote end.
		 */
		err = mv88e6xxx_port_write(chip, port, PORT_PAUSE_CTRL, 0x0000);
		if (err)
			return err;

		/* Port ATU control: disable limiting the number of
		 * address database entries that this port is allowed
		 * to use.
		 */
		err = mv88e6xxx_port_write(chip, port, PORT_ATU_CONTROL,
					   0x0000);
		/* Priority Override: disable DA, SA and VTU priority
		 * override.
		 */
		err = mv88e6xxx_port_write(chip, port, PORT_PRI_OVERRIDE,
					   0x0000);
		if (err)
			return err;

		/* Port Ethertype: use the Ethertype DSA Ethertype
		 * value.
		 */
		if (mv88e6xxx_has(chip, MV88E6XXX_FLAG_EDSA)) {
			err = mv88e6xxx_port_write(chip, port, PORT_ETH_TYPE,
						   ETH_P_EDSA);
			if (err)
				return err;
		}

		/* Tag Remap: use an identity 802.1p prio -> switch
		 * prio mapping.
		 */
		err = mv88e6xxx_port_write(chip, port, PORT_TAG_REGMAP_0123,
					   0x3210);
		if (err)
			return err;

		/* Tag Remap 2: use an identity 802.1p prio -> switch
		 * prio mapping.
		 */
		err = mv88e6xxx_port_write(chip, port, PORT_TAG_REGMAP_4567,
					   0x7654);
		if (err)
			return err;
	}

	/* Rate Control: disable ingress rate limiting. */
	if (mv88e6xxx_6352_family(chip) || mv88e6xxx_6351_family(chip) ||
	    mv88e6xxx_6165_family(chip) || mv88e6xxx_6097_family(chip) ||
	    mv88e6xxx_6320_family(chip)) {
		err = mv88e6xxx_port_write(chip, port, PORT_RATE_CONTROL,
					   0x0001);
		if (err)
			return err;
	} else if (mv88e6xxx_6185_family(chip) || mv88e6xxx_6095_family(chip)) {
		err = mv88e6xxx_port_write(chip, port, PORT_RATE_CONTROL,
					   0x0000);
		if (err)
			return err;
	}

	/* Port Control 1: disable trunking, disable sending
	 * learning messages to this port.
	 */
	err = mv88e6xxx_port_write(chip, port, PORT_CONTROL_1, 0x0000);
	if (err)
		return err;

	/* Port based VLAN map: give each port the same default address
	 * database, and allow bidirectional communication between the
	 * CPU and DSA port(s), and the other ports.
	 */
	err = _mv88e6xxx_port_fid_set(chip, port, 0);
	if (err)
		return err;

	err = _mv88e6xxx_port_based_vlan_map(chip, port);
	if (err)
		return err;

	/* Default VLAN ID and priority: don't set a default VLAN
	 * ID, and set the default packet priority to zero.
	 */
	return mv88e6xxx_port_write(chip, port, PORT_DEFAULT_VLAN, 0x0000);
}

int mv88e6xxx_g1_set_switch_mac(struct mv88e6xxx_chip *chip, u8 *addr)
{
	int err;

	err = mv88e6xxx_g1_write(chip, GLOBAL_MAC_01, (addr[0] << 8) | addr[1]);
	if (err)
		return err;

	err = mv88e6xxx_g1_write(chip, GLOBAL_MAC_23, (addr[2] << 8) | addr[3]);
	if (err)
		return err;

	err = mv88e6xxx_g1_write(chip, GLOBAL_MAC_45, (addr[4] << 8) | addr[5]);
	if (err)
		return err;

	return 0;
}

static int mv88e6xxx_g1_set_age_time(struct mv88e6xxx_chip *chip,
				     unsigned int msecs)
{
	const unsigned int coeff = chip->info->age_time_coeff;
	const unsigned int min = 0x01 * coeff;
	const unsigned int max = 0xff * coeff;
	u8 age_time;
	u16 val;
	int err;

	if (msecs < min || msecs > max)
		return -ERANGE;

	/* Round to nearest multiple of coeff */
	age_time = (msecs + coeff / 2) / coeff;

	err = mv88e6xxx_g1_read(chip, GLOBAL_ATU_CONTROL, &val);
	if (err)
		return err;

	/* AgeTime is 11:4 bits */
	val &= ~0xff0;
	val |= age_time << 4;

	return mv88e6xxx_g1_write(chip, GLOBAL_ATU_CONTROL, val);
}

static int mv88e6xxx_set_ageing_time(struct dsa_switch *ds,
				     unsigned int ageing_time)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int err;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_g1_set_age_time(chip, ageing_time);
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_g1_setup(struct mv88e6xxx_chip *chip)
{
	struct dsa_switch *ds = chip->ds;
	u32 upstream_port = dsa_upstream_port(ds);
	u16 reg;
	int err;

	/* Enable the PHY Polling Unit if present, don't discard any packets,
	 * and mask all interrupt sources.
	 */
	reg = 0;
	if (mv88e6xxx_has(chip, MV88E6XXX_FLAG_PPU) ||
	    mv88e6xxx_has(chip, MV88E6XXX_FLAG_PPU_ACTIVE))
		reg |= GLOBAL_CONTROL_PPU_ENABLE;

	err = mv88e6xxx_g1_write(chip, GLOBAL_CONTROL, reg);
	if (err)
		return err;

	/* Configure the upstream port, and configure it as the port to which
	 * ingress and egress and ARP monitor frames are to be sent.
	 */
	reg = upstream_port << GLOBAL_MONITOR_CONTROL_INGRESS_SHIFT |
		upstream_port << GLOBAL_MONITOR_CONTROL_EGRESS_SHIFT |
		upstream_port << GLOBAL_MONITOR_CONTROL_ARP_SHIFT;
	err = mv88e6xxx_g1_write(chip, GLOBAL_MONITOR_CONTROL, reg);
	if (err)
		return err;

	/* Disable remote management, and set the switch's DSA device number. */
	err = mv88e6xxx_g1_write(chip, GLOBAL_CONTROL_2,
				 GLOBAL_CONTROL_2_MULTIPLE_CASCADE |
				 (ds->index & 0x1f));
	if (err)
		return err;

	/* Clear all the VTU and STU entries */
	err = _mv88e6xxx_vtu_stu_flush(chip);
	if (err < 0)
		return err;

	/* Set the default address aging time to 5 minutes, and
	 * enable address learn messages to be sent to all message
	 * ports.
	 */
	err = mv88e6xxx_g1_write(chip, GLOBAL_ATU_CONTROL,
				 GLOBAL_ATU_CONTROL_LEARN2ALL);
	if (err)
		return err;

	err = mv88e6xxx_g1_set_age_time(chip, 300000);
	if (err)
		return err;

	/* Clear all ATU entries */
	err = _mv88e6xxx_atu_flush(chip, 0, true);
	if (err)
		return err;

	/* Configure the IP ToS mapping registers. */
	err = mv88e6xxx_g1_write(chip, GLOBAL_IP_PRI_0, 0x0000);
	if (err)
		return err;
	err = mv88e6xxx_g1_write(chip, GLOBAL_IP_PRI_1, 0x0000);
	if (err)
		return err;
	err = mv88e6xxx_g1_write(chip, GLOBAL_IP_PRI_2, 0x5555);
	if (err)
		return err;
	err = mv88e6xxx_g1_write(chip, GLOBAL_IP_PRI_3, 0x5555);
	if (err)
		return err;
	err = mv88e6xxx_g1_write(chip, GLOBAL_IP_PRI_4, 0xaaaa);
	if (err)
		return err;
	err = mv88e6xxx_g1_write(chip, GLOBAL_IP_PRI_5, 0xaaaa);
	if (err)
		return err;
	err = mv88e6xxx_g1_write(chip, GLOBAL_IP_PRI_6, 0xffff);
	if (err)
		return err;
	err = mv88e6xxx_g1_write(chip, GLOBAL_IP_PRI_7, 0xffff);
	if (err)
		return err;

	/* Configure the IEEE 802.1p priority mapping register. */
	err = mv88e6xxx_g1_write(chip, GLOBAL_IEEE_PRI, 0xfa41);
	if (err)
		return err;

	/* Clear the statistics counters for all ports */
	err = mv88e6xxx_g1_write(chip, GLOBAL_STATS_OP,
				 GLOBAL_STATS_OP_FLUSH_ALL);
	if (err)
		return err;

	/* Wait for the flush to complete. */
	err = _mv88e6xxx_stats_wait(chip);
	if (err)
		return err;

	return 0;
}

#if 0
static int mv88e6xxx_setup(struct dsa_switch *ds)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int err;
	int i;

	chip->ds = ds;
	ds->slave_mii_bus = chip->mdio_bus;

	mutex_lock(&chip->reg_lock);

	err = mv88e6xxx_switch_reset(chip);
	if (err)
		goto unlock;

	/* Setup Switch Port Registers */
	for (i = 0; i < mv88e6xxx_num_ports(chip); i++) {
		err = mv88e6xxx_setup_port(chip, i);
		if (err)
			goto unlock;
	}

	/* Setup Switch Global 1 Registers */
	err = mv88e6xxx_g1_setup(chip);
	if (err)
		goto unlock;

	/* Setup Switch Global 2 Registers */
	if (mv88e6xxx_has(chip, MV88E6XXX_FLAG_GLOBAL2)) {
		err = mv88e6xxx_g2_setup(chip);
		if (err)
			goto unlock;
	}

unlock:
	mutex_unlock(&chip->reg_lock);

	return err;
}
#endif

static int mv88e6xxx_set_addr(struct dsa_switch *ds, u8 *addr)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int err;

	if (!chip->info->ops->set_switch_mac)
		return -EOPNOTSUPP;

	mutex_lock(&chip->reg_lock);
	err = chip->info->ops->set_switch_mac(chip, addr);
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_mdio_read(struct mii_bus *bus, int phy, int reg)
{
	struct mv88e6xxx_chip *chip = bus->priv;
	u16 val;
	int err;

	if (phy >= mv88e6xxx_num_ports(chip))
		return 0xffff;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_phy_read(chip, phy, reg, &val);
	mutex_unlock(&chip->reg_lock);

	return err ? err : val;
}

static int mv88e6xxx_mdio_write(struct mii_bus *bus, int phy, int reg, u16 val)
{
	struct mv88e6xxx_chip *chip = bus->priv;
	int err;

	if (phy >= mv88e6xxx_num_ports(chip))
		return 0xffff;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_phy_write(chip, phy, reg, val);
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_mdio_register(struct mv88e6xxx_chip *chip,
				   struct device_node *np)
{
	#if 0
	static int index;
	struct mii_bus *bus;
	int err;

	if (np)
		chip->mdio_np = of_get_child_by_name(np, "mdio");

	bus = devm_mdiobus_alloc(chip->dev);
	if (!bus)
		return -ENOMEM;

	bus->priv = (void *)chip;
	if (np) {
		bus->name = np->full_name;
		snprintf(bus->id, MII_BUS_ID_SIZE, "%s", np->full_name);
	} else {
		bus->name = "mv88e6xxx SMI";
		snprintf(bus->id, MII_BUS_ID_SIZE, "mv88e6xxx-%d", index++);
	}

	bus->read = mv88e6xxx_mdio_read;
	bus->write = mv88e6xxx_mdio_write;
	bus->parent = chip->dev;

	if (chip->mdio_np)
		err = of_mdiobus_register(bus, chip->mdio_np);
	else
		err = mdiobus_register(bus);
	if (err) {
		dev_err(chip->dev, "Cannot register MDIO bus (%d)\n", err);
		goto out;
	}
	chip->mdio_bus = bus;

	return 0;

out:
	if (chip->mdio_np)
		of_node_put(chip->mdio_np);

	return err;
	#endif
	return 0;
}

static void mv88e6xxx_mdio_unregister(struct mv88e6xxx_chip *chip)

{
	struct mii_bus *bus = chip->mdio_bus;

	mdiobus_unregister(bus);

	if (chip->mdio_np)
		of_node_put(chip->mdio_np);
}

#ifdef CONFIG_NET_DSA_HWMON

static int mv88e61xx_get_temp(struct dsa_switch *ds, int *temp)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	u16 val;
	int ret;

	*temp = 0;

	mutex_lock(&chip->reg_lock);

	ret = mv88e6xxx_phy_write(chip, 0x0, 0x16, 0x6);
	if (ret < 0)
		goto error;

	/* Enable temperature sensor */
	ret = mv88e6xxx_phy_read(chip, 0x0, 0x1a, &val);
	if (ret < 0)
		goto error;

	ret = mv88e6xxx_phy_write(chip, 0x0, 0x1a, val | (1 << 5));
	if (ret < 0)
		goto error;

	/* Wait for temperature to stabilize */
	usleep_range(10000, 12000);

	ret = mv88e6xxx_phy_read(chip, 0x0, 0x1a, &val);
	if (ret < 0)
		goto error;

	/* Disable temperature sensor */
	ret = mv88e6xxx_phy_write(chip, 0x0, 0x1a, val & ~(1 << 5));
	if (ret < 0)
		goto error;

	*temp = ((val & 0x1f) - 5) * 5;

error:
	mv88e6xxx_phy_write(chip, 0x0, 0x16, 0x0);
	mutex_unlock(&chip->reg_lock);
	return ret;
}

static int mv88e63xx_get_temp(struct dsa_switch *ds, int *temp)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int phy = mv88e6xxx_6320_family(chip) ? 3 : 0;
	u16 val;
	int ret;

	*temp = 0;

	mutex_lock(&chip->reg_lock);
	ret = mv88e6xxx_phy_page_read(chip, phy, 6, 27, &val);
	mutex_unlock(&chip->reg_lock);
	if (ret < 0)
		return ret;

	*temp = (val & 0xff) - 25;

	return 0;
}

static int mv88e6xxx_get_temp(struct dsa_switch *ds, int *temp)
{
	struct mv88e6xxx_chip *chip = ds->priv;

	if (!mv88e6xxx_has(chip, MV88E6XXX_FLAG_TEMP))
		return -EOPNOTSUPP;

	if (mv88e6xxx_6320_family(chip) || mv88e6xxx_6352_family(chip))
		return mv88e63xx_get_temp(ds, temp);

	return mv88e61xx_get_temp(ds, temp);
}

static int mv88e6xxx_get_temp_limit(struct dsa_switch *ds, int *temp)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int phy = mv88e6xxx_6320_family(chip) ? 3 : 0;
	u16 val;
	int ret;

	if (!mv88e6xxx_has(chip, MV88E6XXX_FLAG_TEMP_LIMIT))
		return -EOPNOTSUPP;

	*temp = 0;

	mutex_lock(&chip->reg_lock);
	ret = mv88e6xxx_phy_page_read(chip, phy, 6, 26, &val);
	mutex_unlock(&chip->reg_lock);
	if (ret < 0)
		return ret;

	*temp = (((val >> 8) & 0x1f) * 5) - 25;

	return 0;
}

static int mv88e6xxx_set_temp_limit(struct dsa_switch *ds, int temp)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int phy = mv88e6xxx_6320_family(chip) ? 3 : 0;
	u16 val;
	int err;

	if (!mv88e6xxx_has(chip, MV88E6XXX_FLAG_TEMP_LIMIT))
		return -EOPNOTSUPP;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_phy_page_read(chip, phy, 6, 26, &val);
	if (err)
		goto unlock;
	temp = clamp_val(DIV_ROUND_CLOSEST(temp, 5) + 5, 0, 0x1f);
	err = mv88e6xxx_phy_page_write(chip, phy, 6, 26,
				       (val & 0xe0ff) | (temp << 8));
unlock:
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_get_temp_alarm(struct dsa_switch *ds, bool *alarm)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int phy = mv88e6xxx_6320_family(chip) ? 3 : 0;
	u16 val;
	int ret;

	if (!mv88e6xxx_has(chip, MV88E6XXX_FLAG_TEMP_LIMIT))
		return -EOPNOTSUPP;

	*alarm = false;

	mutex_lock(&chip->reg_lock);
	ret = mv88e6xxx_phy_page_read(chip, phy, 6, 26, &val);
	mutex_unlock(&chip->reg_lock);
	if (ret < 0)
		return ret;

	*alarm = !!(val & 0x40);

	return 0;
}
#endif /* CONFIG_NET_DSA_HWMON */

static int mv88e6xxx_get_eeprom_len(struct dsa_switch *ds)
{
	struct mv88e6xxx_chip *chip = ds->priv;

	return chip->eeprom_len;
}

static int mv88e6xxx_get_eeprom(struct dsa_switch *ds,
				struct ethtool_eeprom *eeprom, u8 *data)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int err;

	if (!chip->info->ops->get_eeprom)
		return -EOPNOTSUPP;

	mutex_lock(&chip->reg_lock);
	err = chip->info->ops->get_eeprom(chip, eeprom, data);
	mutex_unlock(&chip->reg_lock);

	if (err)
		return err;

	eeprom->magic = 0xc3ec4951;

	return 0;
}

static int mv88e6xxx_set_eeprom(struct dsa_switch *ds,
				struct ethtool_eeprom *eeprom, u8 *data)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int err;

	if (!chip->info->ops->set_eeprom)
		return -EOPNOTSUPP;

	if (eeprom->magic != 0xc3ec4951)
		return -EINVAL;

	mutex_lock(&chip->reg_lock);
	err = chip->info->ops->set_eeprom(chip, eeprom, data);
	mutex_unlock(&chip->reg_lock);

	return err;
}

static const struct mv88e6xxx_ops mv88e6085_ops = {
	.set_switch_mac = mv88e6xxx_g1_set_switch_mac,
	.phy_read = mv88e6xxx_phy_ppu_read,
	.phy_write = mv88e6xxx_phy_ppu_write,
};

static const struct mv88e6xxx_ops mv88e6095_ops = {
	.set_switch_mac = mv88e6xxx_g1_set_switch_mac,
	.phy_read = mv88e6xxx_phy_ppu_read,
	.phy_write = mv88e6xxx_phy_ppu_write,
};

static const struct mv88e6xxx_ops mv88e6123_ops = {
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_read,
	.phy_write = mv88e6xxx_write,
};

static const struct mv88e6xxx_ops mv88e6131_ops = {
	.set_switch_mac = mv88e6xxx_g1_set_switch_mac,
	.phy_read = mv88e6xxx_phy_ppu_read,
	.phy_write = mv88e6xxx_phy_ppu_write,
};

static const struct mv88e6xxx_ops mv88e6161_ops = {
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_read,
	.phy_write = mv88e6xxx_write,
};

static const struct mv88e6xxx_ops mv88e6165_ops = {
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_read,
	.phy_write = mv88e6xxx_write,
};

static const struct mv88e6xxx_ops mv88e6171_ops = {
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6172_ops = {
	.get_eeprom = mv88e6xxx_g2_get_eeprom16,
	.set_eeprom = mv88e6xxx_g2_set_eeprom16,
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6175_ops = {
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6176_ops = {
	.get_eeprom = mv88e6xxx_g2_get_eeprom16,
	.set_eeprom = mv88e6xxx_g2_set_eeprom16,
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6185_ops = {
	.set_switch_mac = mv88e6xxx_g1_set_switch_mac,
	.phy_read = mv88e6xxx_phy_ppu_read,
	.phy_write = mv88e6xxx_phy_ppu_write,
};

static const struct mv88e6xxx_ops mv88e6185_ops = {
	.set_switch_mac = mv88e6xxx_g1_set_switch_mac,
	.phy_read = mv88e6xxx_phy_ppu_read,
	.phy_write = mv88e6xxx_phy_ppu_write,
};

static const struct mv88e6xxx_ops mv88e6240_ops = {
	.get_eeprom = mv88e6xxx_g2_get_eeprom16,
	.set_eeprom = mv88e6xxx_g2_set_eeprom16,
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6320_ops = {
	.get_eeprom = mv88e6xxx_g2_get_eeprom16,
	.set_eeprom = mv88e6xxx_g2_set_eeprom16,
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6321_ops = {
	.get_eeprom = mv88e6xxx_g2_get_eeprom16,
	.set_eeprom = mv88e6xxx_g2_set_eeprom16,
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6350_ops = {
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6351_ops = {
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_ops mv88e6352_ops = {
	.get_eeprom = mv88e6xxx_g2_get_eeprom16,
	.set_eeprom = mv88e6xxx_g2_set_eeprom16,
	.set_switch_mac = mv88e6xxx_g2_set_switch_mac,
	.phy_read = mv88e6xxx_g2_smi_phy_read,
	.phy_write = mv88e6xxx_g2_smi_phy_write,
};

static const struct mv88e6xxx_info mv88e6xxx_table[] = {
	[MV88E6085] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6085,
		.family = MV88E6XXX_FAMILY_6097,
		.name = "Marvell 88E6085",
		.num_databases = 4096,
		.num_ports = 10,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6097,
		.ops = &mv88e6085_ops,
	},

	[MV88E6095] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6095,
		.family = MV88E6XXX_FAMILY_6095,
		.name = "Marvell 88E6095/88E6095F",
		.num_databases = 256,
		.num_ports = 11,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6095,
		.ops = &mv88e6095_ops,
	},

	[MV88E6123] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6123,
		.family = MV88E6XXX_FAMILY_6165,
		.name = "Marvell 88E6123",
		.num_databases = 4096,
		.num_ports = 3,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6165,
		.ops = &mv88e6123_ops,
	},

	[MV88E6131] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6131,
		.family = MV88E6XXX_FAMILY_6185,
		.name = "Marvell 88E6131",
		.num_databases = 256,
		.num_ports = 8,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6185,
		.ops = &mv88e6131_ops,
	},

	[MV88E6161] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6161,
		.family = MV88E6XXX_FAMILY_6165,
		.name = "Marvell 88E6161",
		.num_databases = 4096,
		.num_ports = 6,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6165,
		.ops = &mv88e6161_ops,
	},

	[MV88E6165] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6165,
		.family = MV88E6XXX_FAMILY_6165,
		.name = "Marvell 88E6165",
		.num_databases = 4096,
		.num_ports = 6,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6165,
		.ops = &mv88e6165_ops,
	},

	[MV88E6171] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6171,
		.family = MV88E6XXX_FAMILY_6351,
		.name = "Marvell 88E6171",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6351,
		.ops = &mv88e6171_ops,
	},

	[MV88E6172] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6172,
		.family = MV88E6XXX_FAMILY_6352,
		.name = "Marvell 88E6172",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6352,
		.ops = &mv88e6172_ops,
	},

	[MV88E6175] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6175,
		.family = MV88E6XXX_FAMILY_6351,
		.name = "Marvell 88E6175",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6351,
		.ops = &mv88e6175_ops,
	},

	[MV88E6176] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6176,
		.family = MV88E6XXX_FAMILY_6352,
		.name = "Marvell 88E6176",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6352,
		.ops = &mv88e6176_ops,
	},

	[MV88E6185] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6185,
		.family = MV88E6XXX_FAMILY_6185,
		.name = "Marvell 88E6185",
		.num_databases = 256,
		.num_ports = 10,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6185,
		.ops = &mv88e6185_ops,
	},
	[MV88E6190] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6190,
		.family = MV88E6XXX_FAMILY_6390,
		.name = "Marvell 88E6190",
		.num_databases = 4096,
		.num_ports = 11,	/* 10 + Z80 */
		.port_base_addr = 0x0,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.g1_irqs = 9,
		.flags = MV88E6XXX_FLAGS_FAMILY_6390,
		.ops = &mv88e6190_ops,
	},
	[MV88E6240] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6240,
		.family = MV88E6XXX_FAMILY_6352,
		.name = "Marvell 88E6240",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6352,
		.ops = &mv88e6240_ops,
	},

	[MV88E6320] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6320,
		.family = MV88E6XXX_FAMILY_6320,
		.name = "Marvell 88E6320",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6320,
		.ops = &mv88e6320_ops,
	},

	[MV88E6321] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6321,
		.family = MV88E6XXX_FAMILY_6320,
		.name = "Marvell 88E6321",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6320,
		.ops = &mv88e6321_ops,
	},

	[MV88E6350] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6350,
		.family = MV88E6XXX_FAMILY_6351,
		.name = "Marvell 88E6350",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6351,
		.ops = &mv88e6350_ops,
	},

	[MV88E6351] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6351,
		.family = MV88E6XXX_FAMILY_6351,
		.name = "Marvell 88E6351",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6351,
		.ops = &mv88e6351_ops,
	},

	[MV88E6352] = {
		.prod_num = PORT_SWITCH_ID_PROD_NUM_6352,
		.family = MV88E6XXX_FAMILY_6352,
		.name = "Marvell 88E6352",
		.num_databases = 4096,
		.num_ports = 7,
		.port_base_addr = 0x10,
		.global1_addr = 0x1b,
		.age_time_coeff = 15000,
		.flags = MV88E6XXX_FLAGS_FAMILY_6352,
		.ops = &mv88e6352_ops,
	},
};

static const struct mv88e6xxx_info *mv88e6xxx_lookup_info(unsigned int prod_num)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mv88e6xxx_table); ++i)
		if (mv88e6xxx_table[i].prod_num == prod_num)
			return &mv88e6xxx_table[i];

	return NULL;
}

static int mv88e6xxx_detect(struct mv88e6xxx_chip *chip)
{
	const struct mv88e6xxx_info *info;
	unsigned int prod_num, rev;
	u16 id;
	int err;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_port_read(chip, 0, PORT_SWITCH_ID, &id);
	mutex_unlock(&chip->reg_lock);
	if (err)
		return err;

	prod_num = (id & 0xfff0) >> 4;
	rev = id & 0x000f;

	info = mv88e6xxx_lookup_info(prod_num);
	if (!info)
		return -ENODEV;

	/* Update the compatible info with the probed one */
	chip->info = info;

	err = mv88e6xxx_g2_require(chip);
	if (err)
		return err;

	dev_info(chip->dev, "switch 0x%x detected: %s, revision %u\n",
		 chip->info->prod_num, chip->info->name, rev);

	return 0;
}

static struct mv88e6xxx_chip *mv88e6xxx_alloc_chip(struct device *dev)
{
	struct mv88e6xxx_chip *chip;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return NULL;

	chip->dev = dev;

	mutex_init(&chip->reg_lock);

	return chip;
}

static void mv88e6xxx_phy_init(struct mv88e6xxx_chip *chip)
{
	if (mv88e6xxx_has(chip, MV88E6XXX_FLAG_PPU))
		mv88e6xxx_ppu_state_init(chip);
}

static void mv88e6xxx_phy_destroy(struct mv88e6xxx_chip *chip)
{
	if (mv88e6xxx_has(chip, MV88E6XXX_FLAG_PPU))
		mv88e6xxx_ppu_state_destroy(chip);
}

static int mv88e6xxx_smi_init(struct mv88e6xxx_chip *chip,
			      struct mii_bus *bus, int sw_addr)
{
	/* ADDR[0] pin is unavailable externally and considered zero */
	if (sw_addr & 0x1)
		return -EINVAL;

	if (sw_addr == 0)
		chip->smi_ops = &mv88e6xxx_smi_single_chip_ops;
	else if (mv88e6xxx_has(chip, MV88E6XXX_FLAGS_MULTI_CHIP))
		chip->smi_ops = &mv88e6xxx_smi_multi_chip_ops;
	else
		return -EINVAL;

	chip->bus = bus;
	chip->sw_addr = sw_addr;

	return 0;
}

static enum dsa_tag_protocol mv88e6xxx_get_tag_protocol(struct dsa_switch *ds)
{
	struct mv88e6xxx_chip *chip = ds->priv;

	if (mv88e6xxx_has(chip, MV88E6XXX_FLAG_EDSA))
		return DSA_TAG_PROTO_EDSA;

	return DSA_TAG_PROTO_DSA;
}

static const char *mv88e6xxx_drv_probe(struct device *dsa_dev,
				       struct device *host_dev, int sw_addr,
				       void **priv)
{
	struct mv88e6xxx_chip *chip;
	struct mii_bus *bus;
	int err;

	bus = dsa_host_dev_to_mii_bus(host_dev);
	if (!bus)
		return NULL;

	chip = mv88e6xxx_alloc_chip(dsa_dev);
	if (!chip)
		return NULL;

	/* Legacy SMI probing will only support chips similar to 88E6085 */
	chip->info = &mv88e6xxx_table[MV88E6085];

	err = mv88e6xxx_smi_init(chip, bus, sw_addr);
	if (err)
		goto free;

	err = mv88e6xxx_detect(chip);
	if (err)
		goto free;

	mv88e6xxx_phy_init(chip);

	err = mv88e6xxx_mdio_register(chip, NULL);
	if (err)
		goto free;

	*priv = chip;

	return chip->info->name;
free:
	devm_kfree(dsa_dev, chip);

	return NULL;
}

#if 0

static int mv88e6xxx_port_mdb_prepare(struct dsa_switch *ds, int port,
				      const struct switchdev_obj_port_mdb *mdb,
				      struct switchdev_trans *trans)
{
	/* We don't need any dynamic resource from the kernel (yet),
	 * so skip the prepare phase.
	 */

	return 0;
}

static void mv88e6xxx_port_mdb_add(struct dsa_switch *ds, int port,
				   const struct switchdev_obj_port_mdb *mdb,
				   struct switchdev_trans *trans)
{
	struct mv88e6xxx_chip *chip = ds->priv;

	mutex_lock(&chip->reg_lock);
	if (mv88e6xxx_port_db_load_purge(chip, port, mdb->addr, mdb->vid,
					 GLOBAL_ATU_DATA_STATE_MC_STATIC))
		printk( "failed to load multicast MAC address\n");
	mutex_unlock(&chip->reg_lock);
}

static int mv88e6xxx_port_mdb_del(struct dsa_switch *ds, int port,
				  const struct switchdev_obj_port_mdb *mdb)
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int err;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_port_db_load_purge(chip, port, mdb->addr, mdb->vid,
					   GLOBAL_ATU_DATA_STATE_UNUSED);
	mutex_unlock(&chip->reg_lock);

	return err;
}

static int mv88e6xxx_port_mdb_dump(struct dsa_switch *ds, int port,
				   struct switchdev_obj_port_mdb *mdb,
				   int (*cb)(struct switchdev_obj *obj))
{
	struct mv88e6xxx_chip *chip = ds->priv;
	int err;

	mutex_lock(&chip->reg_lock);
	err = mv88e6xxx_port_db_dump(chip, port, &mdb->obj, cb);
	mutex_unlock(&chip->reg_lock);

	return err;
}

#endif

static struct dsa_switch_ops mv88e6xxx_switch_ops = {
	.probe			= mv88e6xxx_drv_probe,
	.get_tag_protocol	= mv88e6xxx_get_tag_protocol,
	.setup			= mv88e6xxx_setup,
	.set_addr		= mv88e6xxx_set_addr,
	.adjust_link		= mv88e6xxx_adjust_link,
	.get_strings		= mv88e6xxx_get_strings,
	.get_ethtool_stats	= mv88e6xxx_get_ethtool_stats,
	.get_sset_count		= mv88e6xxx_get_sset_count,
	.set_eee		= mv88e6xxx_set_eee,
	.get_eee		= mv88e6xxx_get_eee,
#ifdef CONFIG_NET_DSA_HWMON
	.get_temp		= mv88e6xxx_get_temp,
	.get_temp_limit		= mv88e6xxx_get_temp_limit,
	.set_temp_limit		= mv88e6xxx_set_temp_limit,
	.get_temp_alarm		= mv88e6xxx_get_temp_alarm,
#endif
	.get_eeprom_len		= mv88e6xxx_get_eeprom_len,
	.get_eeprom		= mv88e6xxx_get_eeprom,
	.set_eeprom		= mv88e6xxx_set_eeprom,
	.get_regs_len		= mv88e6xxx_get_regs_len,
	.get_regs		= mv88e6xxx_get_regs,
	.set_ageing_time	= mv88e6xxx_set_ageing_time,
	.port_bridge_join	= mv88e6xxx_port_bridge_join,
	.port_bridge_leave	= mv88e6xxx_port_bridge_leave,
	.port_stp_state_set	= mv88e6xxx_port_stp_state_set,
	.port_fast_age		= mv88e6xxx_port_fast_age,
	.port_vlan_filtering	= mv88e6xxx_port_vlan_filtering,
#if 0 
	.port_vlan_prepare	= mv88e6xxx_port_vlan_prepare,
	.port_vlan_add		= mv88e6xxx_port_vlan_add,
	.port_vlan_del		= mv88e6xxx_port_vlan_del,
	.port_vlan_dump		= mv88e6xxx_port_vlan_dump,
	.port_fdb_prepare       = mv88e6xxx_port_fdb_prepare,
	.port_fdb_add           = mv88e6xxx_port_fdb_add,
	.port_fdb_del           = mv88e6xxx_port_fdb_del,
	.port_fdb_dump          = mv88e6xxx_port_fdb_dump,
	.port_mdb_prepare       = mv88e6xxx_port_mdb_prepare,
	.port_mdb_add           = mv88e6xxx_port_mdb_add,
	.port_mdb_del           = mv88e6xxx_port_mdb_del,
	.port_mdb_dump          = mv88e6xxx_port_mdb_dump,
#endif
};

static int mv88e6xxx_register_switch(struct mv88e6xxx_chip *chip,
				     struct device_node *np)
{
	struct dsa_switch *ds;

	ds = kzalloc(sizeof(*ds), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

	ds->dev = NULL;
	ds->priv = chip;
	ds->ops = &mv88e6xxx_switch_ops;
	ds->index = 0;
    

	chip->ds = ds;
	return 0;
	///return dsa_register_switch(ds, np);
}

static void mv88e6xxx_unregister_switch(struct mv88e6xxx_chip *chip)
{
	dsa_unregister_switch(chip->ds);
}

static struct mv88e6xxx_chip *chip;

static mv88e6xxx_init()
{
	
	u32 eeprom_len;
	int err;
	chip = mv88e6xxx_alloc_chip(NULL);
	if (!chip)
		return -ENOMEM;

	ethdev = dev_get_by_index(&init_net, 2);
    if(ethdev == NULL){
            printk("init mdio fail\n");
            return -1;
    }

	err = mv88e6xxx_smi_init(chip, NULL, 0);
	err = mv88e6xxx_detect(chip);
	if (err)
		return err;
	mv88e6xxx_phy_init(chip);

	err = mv88e6xxx_register_switch(chip, NULL);
	if (err) {
		return err;
	}
	
}

#if 0 
static int mv88e6xxx_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct device_node *np = dev->of_node;
	const struct mv88e6xxx_info *compat_info;
	struct mv88e6xxx_chip *chip;
	u32 eeprom_len;
	int err;

	compat_info = of_device_get_match_data(dev);
	if (!compat_info)
		return -EINVAL;

	chip = mv88e6xxx_alloc_chip(dev);
	if (!chip)
		return -ENOMEM;

	chip->info = compat_info;

	err = mv88e6xxx_smi_init(chip, mdiodev->bus, mdiodev->addr);
	if (err)
		return err;

	err = mv88e6xxx_detect(chip);
	if (err)
		return err;

	mv88e6xxx_phy_init(chip);

	chip->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(chip->reset))
		return PTR_ERR(chip->reset);

	if (chip->info->ops->get_eeprom &&
	    !of_property_read_u32(np, "eeprom-length", &eeprom_len))
		chip->eeprom_len = eeprom_len;

	err = mv88e6xxx_mdio_register(chip, np);
	if (err)
		return err;

	err = mv88e6xxx_register_switch(chip, np);
	if (err) {
		mv88e6xxx_mdio_unregister(chip);
		return err;
	}

	return 0;
}

#endif

static void mv88e6xxx_remove(struct mdio_device *mdiodev)
{
	//struct dsa_switch *ds = dev_get_drvdata(&mdiodev->dev);
	//struct mv88e6xxx_chip *chip = ds->priv;

	//mv88e6xxx_phy_destroy(chip);
	//mv88e6xxx_unregister_switch(chip);
}

static const struct of_device_id mv88e6xxx_of_match[] = {
	{
		.compatible = "marvell,mv88e6085",
		.data = &mv88e6xxx_table[MV88E6085],
	},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, mv88e6xxx_of_match);



module_init(mv88e6xxx_init);

static void __exit mv88e6xxx_cleanup(void)
{

	kfree(chip->ds);
	kfree(chip);
	
	return 0;
}
module_exit(mv88e6xxx_cleanup);

MODULE_AUTHOR("Lennert Buytenhek <buytenh@wantstofly.org>");
MODULE_DESCRIPTION("Driver for Marvell 88E6XXX ethernet switch chips");
MODULE_LICENSE("GPL");
