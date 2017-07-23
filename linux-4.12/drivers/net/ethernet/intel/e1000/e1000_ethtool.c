/*******************************************************************************
 * Intel PRO/1000 Linux driver
 * Copyright(c) 1999 - 2006 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Linux NICS <linux.nics@intel.com>
 * e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 ******************************************************************************/

/* ethtool support for e1000 */

#include "e1000.h"
#include <linux/jiffies.h>
#include <linux/uaccess.h>

enum {
	NETDEV_STATS, E1000_STATS
};

struct e1000_stats {
	char stat_string[ETH_GSTRING_LEN];
	int type;
	int sizeof_stat;
	int stat_offset;
};

#define E1000_STAT(m) E1000_STATS, sizeof(((struct e1000_adapter *)0)->m), offsetof(struct e1000_adapter, m)
#define E1000_NETDEV_STAT(m) NETDEV_STATS, sizeof(((struct net_device *)0)->m), offsetof(struct net_device, m)

const struct e1000_stats e1000_gstrings_stats[] =
{
{ "rx_packets", E1000_STAT(stats.gprc) },
{ "tx_packets", E1000_STAT(stats.gptc) },
{ "rx_bytes", E1000_STAT(stats.gorcl) },
{ "tx_bytes", E1000_STAT(stats.gotcl) },
{ "rx_broadcast", E1000_STAT(stats.bprc) },
{ "tx_broadcast", E1000_STAT(stats.bptc) },
{ "rx_multicast", E1000_STAT(stats.mprc) },
{ "tx_multicast", E1000_STAT(stats.mptc) },
{ "rx_errors", E1000_STAT(stats.rxerrc) },
{ "tx_errors", E1000_STAT(stats.txerrc) },
{ "tx_dropped", E1000_NETDEV_STAT(stats.tx_dropped) },
{ "multicast", E1000_STAT(stats.mprc) },
{ "collisions", E1000_STAT(stats.colc) },
{ "rx_length_errors", E1000_STAT(stats.rlerrc) },
{ "rx_over_errors", E1000_NETDEV_STAT(stats.rx_over_errors) },
{ "rx_crc_errors", E1000_STAT(stats.crcerrs) },
{ "rx_frame_errors", E1000_NETDEV_STAT(stats.rx_frame_errors) },
{ "rx_no_buffer_count", E1000_STAT(stats.rnbc) },
{ "rx_missed_errors", E1000_STAT(stats.mpc) },
{ "tx_aborted_errors", E1000_STAT(stats.ecol) },
{ "tx_carrier_errors", E1000_STAT(stats.tncrs) },
{ "tx_fifo_errors", E1000_NETDEV_STAT(stats.tx_fifo_errors) },
{ "tx_heartbeat_errors", E1000_NETDEV_STAT(stats.tx_heartbeat_errors) },
{ "tx_window_errors", E1000_STAT(stats.latecol) },
{ "tx_abort_late_coll", E1000_STAT(stats.latecol) },
{ "tx_deferred_ok", E1000_STAT(stats.dc) },
{ "tx_single_coll_ok", E1000_STAT(stats.scc) },
{ "tx_multi_coll_ok", E1000_STAT(stats.mcc) },
{ "tx_timeout_count", E1000_STAT(tx_timeout_count) },
{ "tx_restart_queue", E1000_STAT(restart_queue) },
{ "rx_long_length_errors", E1000_STAT(stats.roc) },
{ "rx_short_length_errors", E1000_STAT(stats.ruc) },
{ "rx_align_errors", E1000_STAT(stats.algnerrc) },
{ "tx_tcp_seg_good", E1000_STAT(stats.tsctc) },
{ "tx_tcp_seg_failed", E1000_STAT(stats.tsctfc) },
{ "rx_flow_control_xon", E1000_STAT(stats.xonrxc) },
{ "rx_flow_control_xoff", E1000_STAT(stats.xoffrxc) },
{ "tx_flow_control_xon", E1000_STAT(stats.xontxc) },
{ "tx_flow_control_xoff", E1000_STAT(stats.xofftxc) },
{ "rx_long_byte_count", E1000_STAT(stats.gorcl) },
{ "rx_csum_offload_good", E1000_STAT(hw_csum_good) },
{ "rx_csum_offload_errors", E1000_STAT(hw_csum_err) },
{ "alloc_rx_buff_failed", E1000_STAT(alloc_rx_buff_failed) },
{ "tx_smbus", E1000_STAT(stats.mgptc) },
{ "rx_smbus", E1000_STAT(stats.mgprc) },
{ "dropped_smbus", E1000_STAT(stats.mgpdc) }, };

#define E1000_QUEUE_STATS_LEN 0
#define E1000_GLOBAL_STATS_LEN ARRAY_SIZE(e1000_gstrings_stats)
#define E1000_STATS_LEN (E1000_GLOBAL_STATS_LEN + E1000_QUEUE_STATS_LEN)

static const char e1000_gstrings_test[][ETH_GSTRING_LEN] =
{ "Register test  (offline)", "Eeprom test    (offline)", "Interrupt test (offline)", "Loopback test  (offline)", "Link test   (on/offline)" };

#define E1000_TEST_LEN	ARRAY_SIZE(e1000_gstrings_test)

static int e1000_get_link_ksettings(struct net_device *netdev, struct ethtool_link_ksettings *cmd) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 supported, advertising;

	if (hw->media_type == e1000_media_type_copper) {
		supported = (SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full | SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full | SUPPORTED_1000baseT_Full | SUPPORTED_Autoneg | SUPPORTED_TP);
		advertising = ADVERTISED_TP;

		if (hw->autoneg == 1) {
			advertising |= ADVERTISED_Autoneg;
			/* the e1000 autoneg seems to match ethtool nicely */
			advertising |= hw->autoneg_advertised;
		}

		cmd->base.port = PORT_TP;
		cmd->base.phy_address = hw->phy_addr;
	} else {
		supported = (SUPPORTED_1000baseT_Full | SUPPORTED_FIBRE | SUPPORTED_Autoneg);

		advertising = (ADVERTISED_1000baseT_Full | ADVERTISED_FIBRE | ADVERTISED_Autoneg);

		cmd->base.port = PORT_FIBRE;
	}

	if (er32(STATUS) & E1000_STATUS_LU) {
		e1000_get_speed_and_duplex(hw, &adapter->link_speed, &adapter->link_duplex);
		cmd->base.speed = adapter->link_speed;

		/* unfortunately FULL_DUPLEX != DUPLEX_FULL
		 * and HALF_DUPLEX != DUPLEX_HALF
		 */
		if (adapter->link_duplex == FULL_DUPLEX)
			cmd->base.duplex = DUPLEX_FULL;
		else
			cmd->base.duplex = DUPLEX_HALF;
	} else {
		cmd->base.speed = SPEED_UNKNOWN;
		cmd->base.duplex = DUPLEX_UNKNOWN;
	}

	cmd->base.autoneg = ((hw->media_type == e1000_media_type_fiber) || hw->autoneg) ? AUTONEG_ENABLE : AUTONEG_DISABLE;

	/* MDI-X => 1; MDI => 0 */
	if ((hw->media_type == e1000_media_type_copper) && netif_carrier_ok(netdev))
		cmd->base.eth_tp_mdix = (!!adapter->phy_info.mdix_mode ? ETH_TP_MDI_X : ETH_TP_MDI);
	else
		cmd->base.eth_tp_mdix = ETH_TP_MDI_INVALID;

	if (hw->mdix == AUTO_ALL_MODES)
		cmd->base.eth_tp_mdix_ctrl = ETH_TP_MDI_AUTO;
	else
		cmd->base.eth_tp_mdix_ctrl = hw->mdix;

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported, supported);
	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising, advertising);

	return 0;
}

static int e1000_set_link_ksettings(struct net_device *netdev, const struct ethtool_link_ksettings *cmd) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 advertising;

	ethtool_convert_link_mode_to_legacy_u32(&advertising, cmd->link_modes.advertising);

	/* MDI setting is only allowed when autoneg enabled because
	 * some hardware doesn't allow MDI setting when speed or
	 * duplex is forced.
	 */
	if (cmd->base.eth_tp_mdix_ctrl) {
		if (hw->media_type != e1000_media_type_copper)
			return -EOPNOTSUPP;

		if ((cmd->base.eth_tp_mdix_ctrl != ETH_TP_MDI_AUTO) && (cmd->base.autoneg != AUTONEG_ENABLE)) {
			e_err(drv, "forcing MDI/MDI-X state is not supported when link speed and/or duplex are forced\n");
			return -EINVAL;
		}
	}

	while (test_and_set_bit(__E1000_RESETTING, &adapter->flags))
		msleep(1);

	if (cmd->base.autoneg == AUTONEG_ENABLE) {
		hw->autoneg = 1;
		if (hw->media_type == e1000_media_type_fiber)
			hw->autoneg_advertised = ADVERTISED_1000baseT_Full | ADVERTISED_FIBRE | ADVERTISED_Autoneg;
		else
			hw->autoneg_advertised = advertising | ADVERTISED_TP | ADVERTISED_Autoneg;
	} else {
		u32 speed = cmd->base.speed;
		/* calling this overrides forced MDI setting */
		if (e1000_set_spd_dplx(adapter, speed, cmd->base.duplex)) {
			clear_bit(__E1000_RESETTING, &adapter->flags);
			return -EINVAL;
		}
	}

	/* MDI-X => 2; MDI => 1; Auto => 3 */
	if (cmd->base.eth_tp_mdix_ctrl) {
		if (cmd->base.eth_tp_mdix_ctrl == ETH_TP_MDI_AUTO)
			hw->mdix = AUTO_ALL_MODES;
		else
			hw->mdix = cmd->base.eth_tp_mdix_ctrl;
	}

	/* reset the link */

	if (netif_running(adapter->netdev)) {
		e1000_down(adapter);
		e1000_up(adapter);
	} else {
		e1000_reset(adapter);
	}
	clear_bit(__E1000_RESETTING, &adapter->flags);
	return 0;
}

static u32 e1000_get_link(struct net_device *netdev) {
	struct e1000_adapter *adapter = netdev_priv(netdev);

	/* If the link is not reported up to netdev, interrupts are disabled,
	 * and so the physical link state may have changed since we last
	 * looked. Set get_link_status to make sure that the true link
	 * state is interrogated, rather than pulling a cached and possibly
	 * stale link state from the driver.
	 */
	if (!netif_carrier_ok(netdev))
		adapter->hw.get_link_status = 1;

	return e1000_has_link(adapter);
}

static void e1000_get_pauseparam(struct net_device *netdev, struct ethtool_pauseparam *pause) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	pause->autoneg = (adapter->fc_autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE);

	if (hw->fc == E1000_FC_RX_PAUSE) {
		pause->rx_pause = 1;
	} else if (hw->fc == E1000_FC_TX_PAUSE) {
		pause->tx_pause = 1;
	} else if (hw->fc == E1000_FC_FULL) {
		pause->rx_pause = 1;
		pause->tx_pause = 1;
	}
}

static int e1000_set_pauseparam(struct net_device *netdev, struct ethtool_pauseparam *pause) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	int retval = 0;

	adapter->fc_autoneg = pause->autoneg;

	while (test_and_set_bit(__E1000_RESETTING, &adapter->flags))
		msleep(1);

	if (pause->rx_pause && pause->tx_pause)
		hw->fc = E1000_FC_FULL;
	else if (pause->rx_pause && !pause->tx_pause)
		hw->fc = E1000_FC_RX_PAUSE;
	else if (!pause->rx_pause && pause->tx_pause)
		hw->fc = E1000_FC_TX_PAUSE;
	else if (!pause->rx_pause && !pause->tx_pause)
		hw->fc = E1000_FC_NONE;

	hw->original_fc = hw->fc;

	if (adapter->fc_autoneg == AUTONEG_ENABLE) {
		if (netif_running(adapter->netdev)) {
			e1000_down(adapter);
			e1000_up(adapter);
		} else {
			e1000_reset(adapter);
		}
	} else
		retval = ((hw->media_type == e1000_media_type_fiber) ? e1000_setup_link(hw) : e1000_force_mac_fc(hw));

	clear_bit(__E1000_RESETTING, &adapter->flags);
	return retval;
}

static u32 e1000_get_msglevel(struct net_device *netdev) {
	struct e1000_adapter *adapter = netdev_priv(netdev);

	return adapter->msg_enable;
}

static void e1000_set_msglevel(struct net_device *netdev, u32 data) {
	struct e1000_adapter *adapter = netdev_priv(netdev);

	adapter->msg_enable = data;
}

static int e1000_get_regs_len(struct net_device *netdev) {
#define E1000_REGS_LEN 32
	return E1000_REGS_LEN * sizeof(u32);
}

static void e1000_get_regs(struct net_device *netdev, struct ethtool_regs *regs, void *p) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 *regs_buff = p;
	u16 phy_data;

	memset(p, 0, E1000_REGS_LEN * sizeof(u32));

	regs->version = (1 << 24) | (hw->revision_id << 16) | hw->device_id;

	regs_buff[0] = er32(CTRL);
	regs_buff[1] = er32(STATUS);

	regs_buff[2] = er32(RCTL);
	regs_buff[3] = er32(RDLEN);
	regs_buff[4] = er32(RDH);
	regs_buff[5] = er32(RDT);
	regs_buff[6] = er32(RDTR);

	regs_buff[7] = er32(TCTL);
	regs_buff[8] = er32(TDLEN);
	regs_buff[9] = er32(TDH);
	regs_buff[10] = er32(TDT);
	regs_buff[11] = er32(TIDV);

	regs_buff[12] = hw->phy_type; /* PHY type (IGP=1, M88=0) */
	if (hw->phy_type == e1000_phy_igp) {
		e1000_write_phy_reg(hw, IGP01E1000_PHY_PAGE_SELECT, IGP01E1000_PHY_AGC_A);
		e1000_read_phy_reg(hw, IGP01E1000_PHY_AGC_A & IGP01E1000_PHY_PAGE_SELECT, &phy_data);
		regs_buff[13] = (u32) phy_data; /* cable length */
		e1000_write_phy_reg(hw, IGP01E1000_PHY_PAGE_SELECT, IGP01E1000_PHY_AGC_B);
		e1000_read_phy_reg(hw, IGP01E1000_PHY_AGC_B & IGP01E1000_PHY_PAGE_SELECT, &phy_data);
		regs_buff[14] = (u32) phy_data; /* cable length */
		e1000_write_phy_reg(hw, IGP01E1000_PHY_PAGE_SELECT, IGP01E1000_PHY_AGC_C);
		e1000_read_phy_reg(hw, IGP01E1000_PHY_AGC_C & IGP01E1000_PHY_PAGE_SELECT, &phy_data);
		regs_buff[15] = (u32) phy_data; /* cable length */
		e1000_write_phy_reg(hw, IGP01E1000_PHY_PAGE_SELECT, IGP01E1000_PHY_AGC_D);
		e1000_read_phy_reg(hw, IGP01E1000_PHY_AGC_D & IGP01E1000_PHY_PAGE_SELECT, &phy_data);
		regs_buff[16] = (u32) phy_data; /* cable length */
		regs_buff[17] = 0; /* extended 10bt distance (not needed) */
		e1000_write_phy_reg(hw, IGP01E1000_PHY_PAGE_SELECT, 0x0);
		e1000_read_phy_reg(hw, IGP01E1000_PHY_PORT_STATUS & IGP01E1000_PHY_PAGE_SELECT, &phy_data);
		regs_buff[18] = (u32) phy_data; /* cable polarity */
		e1000_write_phy_reg(hw, IGP01E1000_PHY_PAGE_SELECT, IGP01E1000_PHY_PCS_INIT_REG);
		e1000_read_phy_reg(hw, IGP01E1000_PHY_PCS_INIT_REG & IGP01E1000_PHY_PAGE_SELECT, &phy_data);
		regs_buff[19] = (u32) phy_data; /* cable polarity */
		regs_buff[20] = 0; /* polarity correction enabled (always) */
		regs_buff[22] = 0; /* phy receive errors (unavailable) */
		regs_buff[23] = regs_buff[18]; /* mdix mode */
		e1000_write_phy_reg(hw, IGP01E1000_PHY_PAGE_SELECT, 0x0);
	} else {
		e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_STATUS, &phy_data);
		regs_buff[13] = (u32) phy_data; /* cable length */
		regs_buff[14] = 0; /* Dummy (to align w/ IGP phy reg dump) */
		regs_buff[15] = 0; /* Dummy (to align w/ IGP phy reg dump) */
		regs_buff[16] = 0; /* Dummy (to align w/ IGP phy reg dump) */
		e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, &phy_data);
		regs_buff[17] = (u32) phy_data; /* extended 10bt distance */
		regs_buff[18] = regs_buff[13]; /* cable polarity */
		regs_buff[19] = 0; /* Dummy (to align w/ IGP phy reg dump) */
		regs_buff[20] = regs_buff[17]; /* polarity correction */
		/* phy receive errors */
		regs_buff[22] = adapter->phy_stats.receive_errors;
		regs_buff[23] = regs_buff[13]; /* mdix mode */
	}
	regs_buff[21] = adapter->phy_stats.idle_errors; /* phy idle errors */
	e1000_read_phy_reg(hw, PHY_1000T_STATUS, &phy_data);
	regs_buff[24] = (u32) phy_data; /* phy local receiver status */
	regs_buff[25] = regs_buff[24]; /* phy remote receiver status */
	if (hw->mac_type >= e1000_82540 && hw->media_type == e1000_media_type_copper) {
		regs_buff[26] = er32(MANC);
	}
}

static int e1000_get_eeprom_len(struct net_device *netdev) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	return hw->eeprom.word_size * 2;
}

static int e1000_get_eeprom(struct net_device *netdev, struct ethtool_eeprom *eeprom, u8 *bytes) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u16 *eeprom_buff;
	int first_word, last_word;
	int ret_val = 0;
	u16 i;

	if (eeprom->len == 0)
		return -EINVAL;

	eeprom->magic = hw->vendor_id | (hw->device_id << 16);

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;

	eeprom_buff = kmalloc(sizeof(u16) * (last_word - first_word + 1), GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	if (hw->eeprom.type == e1000_eeprom_spi)
		ret_val = e1000_read_eeprom(hw, first_word, last_word - first_word + 1, eeprom_buff);
	else {
		for (i = 0; i < last_word - first_word + 1; i++) {
			ret_val = e1000_read_eeprom(hw, first_word + i, 1, &eeprom_buff[i]);
			if (ret_val)
				break;
		}
	}

	/* Device's eeprom is always little-endian, word addressable */
	for (i = 0; i < last_word - first_word + 1; i++)
		le16_to_cpus(&eeprom_buff[i]);

	memcpy(bytes, (u8 *) eeprom_buff + (eeprom->offset & 1), eeprom->len);
	kfree(eeprom_buff);

	return ret_val;
}

static int e1000_set_eeprom(struct net_device *netdev, struct ethtool_eeprom *eeprom, u8 *bytes) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u16 *eeprom_buff;
	void *ptr;
	int max_len, first_word, last_word, ret_val = 0;
	u16 i;

	if (eeprom->len == 0)
		return -EOPNOTSUPP;

	if (eeprom->magic != (hw->vendor_id | (hw->device_id << 16)))
		return -EFAULT;

	max_len = hw->eeprom.word_size * 2;

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;
	eeprom_buff = kmalloc(max_len, GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	ptr = (void *) eeprom_buff;

	if (eeprom->offset & 1) {
		/* need read/modify/write of first changed EEPROM word
		 * only the second byte of the word is being modified
		 */
		ret_val = e1000_read_eeprom(hw, first_word, 1, &eeprom_buff[0]);
		ptr++;
	}
	if (((eeprom->offset + eeprom->len) & 1) && (ret_val == 0)) {
		/* need read/modify/write of last changed EEPROM word
		 * only the first byte of the word is being modified
		 */
		ret_val = e1000_read_eeprom(hw, last_word, 1, &eeprom_buff[last_word - first_word]);
	}

	/* Device's eeprom is always little-endian, word addressable */
	for (i = 0; i < last_word - first_word + 1; i++)
		le16_to_cpus(&eeprom_buff[i]);

	memcpy(ptr, bytes, eeprom->len);

	for (i = 0; i < last_word - first_word + 1; i++)
		eeprom_buff[i] = cpu_to_le16(eeprom_buff[i]);

	ret_val = e1000_write_eeprom(hw, first_word, last_word - first_word + 1, eeprom_buff);

	/* Update the checksum over the first part of the EEPROM if needed */
	if ((ret_val == 0) && (first_word <= EEPROM_CHECKSUM_REG))
		e1000_update_eeprom_checksum(hw);

	kfree(eeprom_buff);
	return ret_val;
}

static void e1000_get_drvinfo(struct net_device *netdev, struct ethtool_drvinfo *drvinfo) {
	struct e1000_adapter *adapter = netdev_priv(netdev);

	strlcpy(drvinfo->driver, DRV_NAME, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, DRV_VERSION, sizeof(drvinfo->version));

	strlcpy(drvinfo->bus_info, pci_name(adapter->pdev), sizeof(drvinfo->bus_info));
}

static void e1000_get_ringparam(struct net_device *netdev, struct ethtool_ringparam *ring) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	e1000_mac_type mac_type = hw->mac_type;
	struct e1000_tx_ring *txdr = adapter->tx_ring;
	struct e1000_rx_ring *rxdr = adapter->rx_ring;

	ring->rx_max_pending = (mac_type < e1000_82544) ? E1000_MAX_RXD : E1000_MAX_82544_RXD;
	ring->tx_max_pending = (mac_type < e1000_82544) ? E1000_MAX_TXD : E1000_MAX_82544_TXD;
	ring->rx_pending = rxdr->count;
	ring->tx_pending = txdr->count;
}

static int e1000_set_ringparam(struct net_device *netdev, struct ethtool_ringparam *ring) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	e1000_mac_type mac_type = hw->mac_type;
	struct e1000_tx_ring *txdr, *tx_old;
	struct e1000_rx_ring *rxdr, *rx_old;
	int i, err;

	if ((ring->rx_mini_pending) || (ring->rx_jumbo_pending))
		return -EINVAL;

	while (test_and_set_bit(__E1000_RESETTING, &adapter->flags))
		msleep(1);

	if (netif_running(adapter->netdev))
		e1000_down(adapter);

	tx_old = adapter->tx_ring;
	rx_old = adapter->rx_ring;

	err = -ENOMEM;
	txdr = kcalloc(adapter->num_tx_queues, sizeof(struct e1000_tx_ring), GFP_KERNEL);
	if (!txdr)
		goto err_alloc_tx;

	rxdr = kcalloc(adapter->num_rx_queues, sizeof(struct e1000_rx_ring), GFP_KERNEL);
	if (!rxdr)
		goto err_alloc_rx;

	adapter->tx_ring = txdr;
	adapter->rx_ring = rxdr;

	rxdr->count = max(ring->rx_pending, (u32) E1000_MIN_RXD);
	rxdr->count = min(rxdr->count, (u32)(mac_type < e1000_82544 ? E1000_MAX_RXD : E1000_MAX_82544_RXD));
	rxdr->count = ALIGN(rxdr->count, REQ_RX_DESCRIPTOR_MULTIPLE);
	txdr->count = max(ring->tx_pending, (u32) E1000_MIN_TXD);
	txdr->count = min(txdr->count, (u32)(mac_type < e1000_82544 ? E1000_MAX_TXD : E1000_MAX_82544_TXD));
	txdr->count = ALIGN(txdr->count, REQ_TX_DESCRIPTOR_MULTIPLE);

	for (i = 0; i < adapter->num_tx_queues; i++)
		txdr[i].count = txdr->count;
	for (i = 0; i < adapter->num_rx_queues; i++)
		rxdr[i].count = rxdr->count;

	if (netif_running(adapter->netdev)) {
		/* Try to get new resources before deleting old */
		err = e1000_setup_all_rx_resources(adapter);
		if (err)
			goto err_setup_rx;
		err = e1000_setup_all_tx_resources(adapter);
		if (err)
			goto err_setup_tx;

		/* save the new, restore the old in order to free it,
		 * then restore the new back again
		 */

		adapter->rx_ring = rx_old;
		adapter->tx_ring = tx_old;
		e1000_free_all_rx_resources(adapter);
		e1000_free_all_tx_resources(adapter);
		kfree(tx_old);
		kfree(rx_old);
		adapter->rx_ring = rxdr;
		adapter->tx_ring = txdr;
		err = e1000_up(adapter);
		if (err)
			goto err_setup;
	}

	clear_bit(__E1000_RESETTING, &adapter->flags);
	return 0;
	err_setup_tx: e1000_free_all_rx_resources(adapter);
	err_setup_rx: adapter->rx_ring = rx_old;
	adapter->tx_ring = tx_old;
	kfree(rxdr);
	err_alloc_rx: kfree(txdr);
	err_alloc_tx: e1000_up(adapter);
	err_setup: clear_bit(__E1000_RESETTING, &adapter->flags);
	return err;
}

static int e1000_get_sset_count(struct net_device *netdev, int sset) {
	switch (sset) {
		case ETH_SS_TEST:
			return E1000_TEST_LEN;
		case ETH_SS_STATS:
			return E1000_STATS_LEN;
		default:
			return -EOPNOTSUPP;
	}
}

static int e1000_wol_exclusion(struct e1000_adapter *adapter, struct ethtool_wolinfo *wol) {
	struct e1000_hw *hw = &adapter->hw;
	int retval = 1; /* fail by default */

	switch (hw->device_id) {
		case E1000_DEV_ID_82542:
		case E1000_DEV_ID_82543GC_FIBER:
		case E1000_DEV_ID_82543GC_COPPER:
		case E1000_DEV_ID_82544EI_FIBER:
		case E1000_DEV_ID_82546EB_QUAD_COPPER:
		case E1000_DEV_ID_82545EM_FIBER:
		case E1000_DEV_ID_82545EM_COPPER:
		case E1000_DEV_ID_82546GB_QUAD_COPPER:
		case E1000_DEV_ID_82546GB_PCIE:
			/* these don't support WoL at all */
			wol->supported = 0;
			break;
		case E1000_DEV_ID_82546EB_FIBER:
		case E1000_DEV_ID_82546GB_FIBER:
			/* Wake events not supported on port B */
			if (er32(STATUS) & E1000_STATUS_FUNC_1) {
				wol->supported = 0;
				break;
			}
			/* return success for non excluded adapter ports */
			retval = 0;
			break;
		case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
			/* quad port adapters only support WoL on port A */
			if (!adapter->quad_port_a) {
				wol->supported = 0;
				break;
			}
			/* return success for non excluded adapter ports */
			retval = 0;
			break;
		default:
			/* dual port cards only support WoL on port A from now on
			 * unless it was enabled in the eeprom for port B
			 * so exclude FUNC_1 ports from having WoL enabled
			 */
			if (er32(STATUS) & E1000_STATUS_FUNC_1 && !adapter->eeprom_wol) {
				wol->supported = 0;
				break;
			}

			retval = 0;
	}

	return retval;
}

static void e1000_get_wol(struct net_device *netdev, struct ethtool_wolinfo *wol) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	wol->supported = WAKE_UCAST | WAKE_MCAST | WAKE_BCAST | WAKE_MAGIC;
	wol->wolopts = 0;

	/* this function will set ->supported = 0 and return 1 if wol is not
	 * supported by this hardware
	 */
	if (e1000_wol_exclusion(adapter, wol) || !device_can_wakeup(&adapter->pdev->dev))
		return;

	/* apply any specific unsupported masks here */
	switch (hw->device_id) {
		case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
			/* KSP3 does not support UCAST wake-ups */
			wol->supported &= ~WAKE_UCAST;

			if (adapter->wol & E1000_WUFC_EX)
				e_err(drv, "Interface does not support directed "
					"(unicast) frame wake-up packets\n");
			break;
		default:
			break;
	}

	if (adapter->wol & E1000_WUFC_EX)
		wol->wolopts |= WAKE_UCAST;
	if (adapter->wol & E1000_WUFC_MC)
		wol->wolopts |= WAKE_MCAST;
	if (adapter->wol & E1000_WUFC_BC)
		wol->wolopts |= WAKE_BCAST;
	if (adapter->wol & E1000_WUFC_MAG)
		wol->wolopts |= WAKE_MAGIC;
}

static int e1000_set_wol(struct net_device *netdev, struct ethtool_wolinfo *wol) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	if (wol->wolopts & (WAKE_PHY | WAKE_ARP | WAKE_MAGICSECURE))
		return -EOPNOTSUPP;

	if (e1000_wol_exclusion(adapter, wol) || !device_can_wakeup(&adapter->pdev->dev))
		return wol->wolopts ? -EOPNOTSUPP : 0;

	switch (hw->device_id) {
		case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
			if (wol->wolopts & WAKE_UCAST) {
				e_err(drv, "Interface does not support directed "
					"(unicast) frame wake-up packets\n");
				return -EOPNOTSUPP;
			}
			break;
		default:
			break;
	}

	/* these settings will always override what we currently have */
	adapter->wol = 0;

	if (wol->wolopts & WAKE_UCAST)
		adapter->wol |= E1000_WUFC_EX;
	if (wol->wolopts & WAKE_MCAST)
		adapter->wol |= E1000_WUFC_MC;
	if (wol->wolopts & WAKE_BCAST)
		adapter->wol |= E1000_WUFC_BC;
	if (wol->wolopts & WAKE_MAGIC)
		adapter->wol |= E1000_WUFC_MAG;

	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	return 0;
}

static int e1000_set_phys_id(struct net_device *netdev, enum ethtool_phys_id_state state) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	switch (state) {
		case ETHTOOL_ID_ACTIVE:
			e1000_setup_led(hw);
			return 2;

		case ETHTOOL_ID_ON:
			e1000_led_on(hw);
			break;

		case ETHTOOL_ID_OFF:
			e1000_led_off(hw);
			break;

		case ETHTOOL_ID_INACTIVE:
			e1000_cleanup_led(hw);
	}

	return 0;
}

static int e1000_get_coalesce(struct net_device *netdev, struct ethtool_coalesce *ec) {
	struct e1000_adapter *adapter = netdev_priv(netdev);

	if (adapter->hw.mac_type < e1000_82545)
		return -EOPNOTSUPP;

	if (adapter->itr_setting <= 4)
		ec->rx_coalesce_usecs = adapter->itr_setting;
	else
		ec->rx_coalesce_usecs = 1000000 / adapter->itr_setting;

	return 0;
}

static int e1000_set_coalesce(struct net_device *netdev, struct ethtool_coalesce *ec) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	if (hw->mac_type < e1000_82545)
		return -EOPNOTSUPP;

	if ((ec->rx_coalesce_usecs > E1000_MAX_ITR_USECS) || ((ec->rx_coalesce_usecs > 4) && (ec->rx_coalesce_usecs < E1000_MIN_ITR_USECS)) || (ec->rx_coalesce_usecs == 2))
		return -EINVAL;

	if (ec->rx_coalesce_usecs == 4) {
		adapter->itr = adapter->itr_setting = 4;
	} else if (ec->rx_coalesce_usecs <= 3) {
		adapter->itr = 20000;
		adapter->itr_setting = ec->rx_coalesce_usecs;
	} else {
		adapter->itr = (1000000 / ec->rx_coalesce_usecs);
		adapter->itr_setting = adapter->itr & ~3;
	}

	if (adapter->itr_setting != 0)
		ew32(ITR, 1000000000 / (adapter->itr * 256));
	else
		ew32(ITR, 0);

	return 0;
}

static int e1000_nway_reset(struct net_device *netdev) {
	struct e1000_adapter *adapter = netdev_priv(netdev);

	if (netif_running(netdev))
		e1000_reinit_locked(adapter);
	return 0;
}

static void e1000_get_ethtool_stats(struct net_device *netdev, struct ethtool_stats *stats, u64 *data) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	int i;
	char *p = NULL;
	const struct e1000_stats *stat = e1000_gstrings_stats;

	e1000_update_stats(adapter);
	for (i = 0; i < E1000_GLOBAL_STATS_LEN; i++) {
		switch (stat->type) {
			case NETDEV_STATS:
				p = (char *) netdev + stat->stat_offset;
				break;
			case E1000_STATS:
				p = (char *) adapter + stat->stat_offset;
				break;
			default:
				WARN_ONCE(1, "Invalid E1000 stat type: %u index %d\n", stat->type, i);
				break;
		}

		if (stat->sizeof_stat == sizeof(u64))
			data[i] = *(u64 *) p;
		else
			data[i] = *(u32 *) p;

		stat++;
	}
	/* BUG_ON(i != E1000_STATS_LEN); */
}

static void e1000_get_strings(struct net_device *netdev, u32 stringset, u8 *data) {
	u8 *p = data;
	int i;

	switch (stringset) {
		case ETH_SS_TEST:
			memcpy(data, e1000_gstrings_test, sizeof(e1000_gstrings_test));
			break;
		case ETH_SS_STATS:
			for (i = 0; i < E1000_GLOBAL_STATS_LEN; i++) {
				memcpy(p, e1000_gstrings_stats[i].stat_string, ETH_GSTRING_LEN);
				p += ETH_GSTRING_LEN;
			}
			/* BUG_ON(p - data != E1000_STATS_LEN * ETH_GSTRING_LEN); */
			break;
	}
}

static const struct ethtool_ops e1000_ethtool_ops =
{
.get_drvinfo = e1000_get_drvinfo,
.get_regs_len = e1000_get_regs_len,
.get_regs = e1000_get_regs,
.get_wol = e1000_get_wol,
.set_wol = e1000_set_wol,
.get_msglevel = e1000_get_msglevel,
.set_msglevel = e1000_set_msglevel,
.nway_reset = e1000_nway_reset,
.get_link = e1000_get_link,
.get_eeprom_len = e1000_get_eeprom_len,
.get_eeprom = e1000_get_eeprom,
.set_eeprom = e1000_set_eeprom,
.get_ringparam = e1000_get_ringparam,
.set_ringparam = e1000_set_ringparam,
.get_pauseparam = e1000_get_pauseparam,
.set_pauseparam = e1000_set_pauseparam,
.self_test = e1000_diag_test,
.get_strings = e1000_get_strings,
.set_phys_id = e1000_set_phys_id,
.get_ethtool_stats = e1000_get_ethtool_stats,
.get_sset_count = e1000_get_sset_count,
.get_coalesce = e1000_get_coalesce,
.set_coalesce = e1000_set_coalesce,
.get_ts_info = ethtool_op_get_ts_info,
.get_link_ksettings = e1000_get_link_ksettings,
.set_link_ksettings = e1000_set_link_ksettings, };

void e1000_set_ethtool_ops(struct net_device *netdev) {
	netdev->ethtool_ops = &e1000_ethtool_ops;
}
