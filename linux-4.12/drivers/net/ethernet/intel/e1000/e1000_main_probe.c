#include "e1000.h"

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV|NETIF_MSG_PROBE|NETIF_MSG_LINK)
#define DEFAULT_DBG_LEVEL (-1)

extern const struct net_device_ops e1000_netdev_ops;

static int e1000_is_need_ioport(struct pci_dev *pdev);
static int e1000_init_hw_struct(struct e1000_adapter *adapter, struct e1000_hw *hw);
static void e1000_dump_eeprom(struct e1000_adapter *adapter);
static void e1000_update_phy_info_task(struct work_struct *work);
static int e1000_clean(struct napi_struct *napi, int budget);
static void e1000_set_itr(struct e1000_adapter *adapter);
static unsigned int e1000_update_itr(struct e1000_adapter *adapter, u16 itr_setting, int packets, int bytes);
static void e1000_watchdog(struct work_struct *work);
static void e1000_reset_task(struct work_struct *work);
static void e1000_82547_tx_fifo_stall_task(struct work_struct *work);
static int e1000_sw_init(struct e1000_adapter *adapter);
static bool e1000_clean_tx_irq(struct e1000_adapter *adapter, struct e1000_tx_ring *tx_ring);
static void e1000_smartspeed(struct e1000_adapter *adapter);
static void e1000_dump(struct e1000_adapter *adapter);
static void e1000_regdump(struct e1000_adapter *adapter);
static int e1000_alloc_queues(struct e1000_adapter *adapter);

/**
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
	struct net_device *netdev;
	struct e1000_adapter *adapter;
	struct e1000_hw *hw;

	static int cards_found;
	static int global_quad_port_a; /* global ksp3 port a indication */
	int i, err, pci_using_dac;
	u16 eeprom_data = 0;
	u16 tmp = 0;
	u16 eeprom_apme_mask = E1000_EEPROM_APME;
	int bars, need_ioport;

	need_ioport = e1000_is_need_ioport(pdev);
	if (need_ioport) {
		bars = pci_select_bars(pdev, IORESOURCE_MEM | IORESOURCE_IO);
		err = pci_enable_device(pdev);
	} else {
		bars = pci_select_bars(pdev, IORESOURCE_MEM);
		err = pci_enable_device_mem(pdev);
	}

	pci_request_selected_regions(pdev, bars, DRV_NAME);
	pci_set_master(pdev);
	pci_save_state(pdev);

	netdev = alloc_etherdev(sizeof(struct e1000_adapter));
	SET_NETDEV_DEV(netdev, &pdev->dev);

	pci_set_drvdata(pdev, netdev);

	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->msg_enable = netif_msg_init(DEFAULT_DBG_LEVEL, DEFAULT_MSG_ENABLE);
	adapter->bars = bars;
	adapter->need_ioport = need_ioport;

	hw = &adapter->hw;
	hw->back = adapter;
	hw->hw_addr = pci_ioremap_bar(pdev, BAR_0);

	if (adapter->need_ioport) {
		for (i = BAR_1; i <= BAR_5; i++) {
			if (pci_resource_len(pdev, i) == 0)
				continue;
			if (pci_resource_flags(pdev, i) & IORESOURCE_IO) {
				hw->io_base = pci_resource_start(pdev, i);
				break;
			}
		}
	}

	/* make ready for any if (hw->...) below */
	e1000_init_hw_struct(adapter, hw);

	/* there is a workaround being applied below that limits
	 * 64-bit DMA addresses to 64-bit hardware.  There are some
	 * 32-bit adapters that Tx hang when given 64-bit DMA addresses
	 */
	pci_using_dac = 0;
	if ((hw->bus_type == e1000_bus_type_pcix) && !dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))) {
		pci_using_dac = 1;
	} else {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			pr_err("No usable DMA config, aborting\n");
			goto err_dma;
		}
	}

	netdev->netdev_ops = &e1000_netdev_ops;
	e1000_set_ethtool_ops(netdev);
	netdev->watchdog_timeo = 5 * HZ;
	netif_napi_add(netdev, &adapter->napi, e1000_clean, 64);

	strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

	adapter->bd_number = cards_found;

	/* setup the private structure */

	err = e1000_sw_init(adapter);
	if (err)
		goto err_sw_init;

	err = -EIO;
	if (hw->mac_type == e1000_ce4100) {
		hw->ce4100_gbe_mdio_base_virt = ioremap(pci_resource_start(pdev, BAR_1), pci_resource_len(pdev, BAR_1));

		if (!hw->ce4100_gbe_mdio_base_virt)
			goto err_mdio_ioremap;
	}

	if (hw->mac_type >= e1000_82543) {
		netdev->hw_features = NETIF_F_SG | NETIF_F_HW_CSUM | NETIF_F_HW_VLAN_CTAG_RX;
		netdev->features = NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_FILTER;
	}

	if ((hw->mac_type >= e1000_82544) && (hw->mac_type != e1000_82547))
		netdev->hw_features |= NETIF_F_TSO;

	netdev->priv_flags |= IFF_SUPP_NOFCS;

	netdev->features |= netdev->hw_features;
	netdev->hw_features |= (NETIF_F_RXCSUM | NETIF_F_RXALL | NETIF_F_RXFCS);

	if (pci_using_dac) {
		netdev->features |= NETIF_F_HIGHDMA;
		netdev->vlan_features |= NETIF_F_HIGHDMA;
	}

	netdev->vlan_features |= (NETIF_F_TSO | NETIF_F_HW_CSUM | NETIF_F_SG);

	/* Do not set IFF_UNICAST_FLT for VMWare's 82545EM */
	if (hw->device_id != E1000_DEV_ID_82545EM_COPPER || hw->subsystem_vendor_id != PCI_VENDOR_ID_VMWARE)
		netdev->priv_flags |= IFF_UNICAST_FLT;

	/* MTU range: 46 - 16110 */
	netdev->min_mtu = ETH_ZLEN - ETH_HLEN;
	netdev->max_mtu = MAX_JUMBO_FRAME_SIZE - (ETH_HLEN + ETH_FCS_LEN);

	adapter->en_mng_pt = e1000_enable_mng_pass_thru(hw);

	/* initialize eeprom parameters */
	if (e1000_init_eeprom_params(hw)) {
		e_err(probe, "EEPROM initialization failed\n");
		goto err_eeprom;
	}

	/* before reading the EEPROM, reset the controller to
	 * put the device in a known good starting state
	 */

	e1000_reset_hw(hw);

	/* make sure the EEPROM is good */
	if (e1000_validate_eeprom_checksum(hw) < 0) {
		e_err(probe, "The EEPROM Checksum Is Not Valid\n");
		e1000_dump_eeprom(adapter);
		/* set MAC address to all zeroes to invalidate and temporary
		 * disable this device for the user. This blocks regular
		 * traffic while still permitting ethtool ioctls from reaching
		 * the hardware as well as allowing the user to run the
		 * interface after manually setting a hw addr using
		 * `ip set address`
		 */
		memset(hw->mac_addr, 0, netdev->addr_len);
	} else {
		/* copy the MAC address out of the EEPROM */
		if (e1000_read_mac_addr(hw))
			e_err(probe, "EEPROM Read Error\n");
	}
	/* don't block initialization here due to bad MAC address */
	memcpy(netdev->dev_addr, hw->mac_addr, netdev->addr_len);

	if (!is_valid_ether_addr(netdev->dev_addr))
		e_err(probe, "Invalid MAC Address\n");

	INIT_DELAYED_WORK(&adapter->watchdog_task, e1000_watchdog);
	INIT_DELAYED_WORK(&adapter->fifo_stall_task, e1000_82547_tx_fifo_stall_task);
	INIT_DELAYED_WORK(&adapter->phy_info_task, e1000_update_phy_info_task);
	INIT_WORK(&adapter->reset_task, e1000_reset_task);

	e1000_check_options(adapter);

	/* Initial Wake on LAN setting
	 * If APM wake is enabled in the EEPROM,
	 * enable the ACPI Magic Packet filter
	 */

	switch (hw->mac_type) {
	case e1000_82542_rev2_0:
	case e1000_82542_rev2_1:
	case e1000_82543:
		break;
	case e1000_82544:
		e1000_read_eeprom(hw, EEPROM_INIT_CONTROL2_REG, 1, &eeprom_data);
		eeprom_apme_mask = E1000_EEPROM_82544_APM;
		break;
	case e1000_82546:
	case e1000_82546_rev_3:
		if (er32(STATUS) & E1000_STATUS_FUNC_1) {
			e1000_read_eeprom(hw, EEPROM_INIT_CONTROL3_PORT_B, 1, &eeprom_data);
			break;
		}
		/* Fall Through */
	default:
		e1000_read_eeprom(hw, EEPROM_INIT_CONTROL3_PORT_A, 1, &eeprom_data);
		break;
	}
	if (eeprom_data & eeprom_apme_mask)
		adapter->eeprom_wol |= E1000_WUFC_MAG;

	/* now that we have the eeprom settings, apply the special cases
	 * where the eeprom may be wrong or the board simply won't support
	 * wake on lan on a particular port
	 */
	switch (pdev->device) {
	case E1000_DEV_ID_82546GB_PCIE:
		adapter->eeprom_wol = 0;
		break;
	case E1000_DEV_ID_82546EB_FIBER:
	case E1000_DEV_ID_82546GB_FIBER:
		/* Wake events only supported on port A for dual fiber
		 * regardless of eeprom setting
		 */
		if (er32(STATUS) & E1000_STATUS_FUNC_1)
			adapter->eeprom_wol = 0;
		break;
	case E1000_DEV_ID_82546GB_QUAD_COPPER_KSP3:
		/* if quad port adapter, disable WoL on all but port A */
		if (global_quad_port_a != 0)
			adapter->eeprom_wol = 0;
		else
			adapter->quad_port_a = true;
		/* Reset for multiple quad port adapters */
		if (++global_quad_port_a == 4)
			global_quad_port_a = 0;
		break;
	}

	/* initialize the wol settings based on the eeprom settings */
	adapter->wol = adapter->eeprom_wol;
	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	/* Auto detect PHY address */
	if (hw->mac_type == e1000_ce4100) {
		for (i = 0; i < 32; i++) {
			hw->phy_addr = i;
			e1000_read_phy_reg(hw, PHY_ID2, &tmp);

			if (tmp != 0 && tmp != 0xFF)
				break;
		}

		if (i >= 32)
			goto err_eeprom;
	}

	/* reset the hardware with the new settings */
	e1000_reset(adapter);

	strcpy(netdev->name, "eth%d");
	err = register_netdev(netdev);
	if (err)
		goto err_register;

	e1000_vlan_filter_on_off(adapter, false);

	/* print bus type/speed/width info */
	e_info(probe, "(PCI%s:%dMHz:%d-bit) %pM\n", ((hw->bus_type == e1000_bus_type_pcix) ? "-X" : ""),
			((hw->bus_speed == e1000_bus_speed_133) ? 133 : (hw->bus_speed == e1000_bus_speed_120) ? 120 : (hw->bus_speed == e1000_bus_speed_100) ? 100 :
				(hw->bus_speed == e1000_bus_speed_66) ? 66 : 33), ((hw->bus_width == e1000_bus_width_64) ? 64 : 32), netdev->dev_addr);

	/* carrier off reporting is important to ethtool even BEFORE open */
	netif_carrier_off(netdev);

	e_info(probe, "Intel(R) PRO/1000 Network Connection\n");

	cards_found++;
	return 0;

	err_register: err_eeprom: e1000_phy_hw_reset(hw);

	if (hw->flash_address)
		iounmap(hw->flash_address);
	kfree(adapter->tx_ring);
	kfree(adapter->rx_ring);
	err_dma: err_sw_init: err_mdio_ioremap: iounmap(hw->ce4100_gbe_mdio_base_virt);
	iounmap(hw->hw_addr);
	err_ioremap: free_netdev(netdev);
	err_alloc_etherdev: pci_release_selected_regions(pdev, bars);
	err_pci_reg: pci_disable_device(pdev);
	return err;
}

/**
 * e1000_is_need_ioport - determine if an adapter needs ioport resources or not
 * @pdev: PCI device information struct
 **/
static int e1000_is_need_ioport(struct pci_dev *pdev) {
	switch (pdev->device) {
	case E1000_DEV_ID_82540EM:
	case E1000_DEV_ID_82540EM_LOM:
	case E1000_DEV_ID_82540EP:
	case E1000_DEV_ID_82540EP_LOM:
	case E1000_DEV_ID_82540EP_LP:
	case E1000_DEV_ID_82541EI:
	case E1000_DEV_ID_82541EI_MOBILE:
	case E1000_DEV_ID_82541ER:
	case E1000_DEV_ID_82541ER_LOM:
	case E1000_DEV_ID_82541GI:
	case E1000_DEV_ID_82541GI_LF:
	case E1000_DEV_ID_82541GI_MOBILE:
	case E1000_DEV_ID_82544EI_COPPER:
	case E1000_DEV_ID_82544EI_FIBER:
	case E1000_DEV_ID_82544GC_COPPER:
	case E1000_DEV_ID_82544GC_LOM:
	case E1000_DEV_ID_82545EM_COPPER:
	case E1000_DEV_ID_82545EM_FIBER:
	case E1000_DEV_ID_82546EB_COPPER:
	case E1000_DEV_ID_82546EB_FIBER:
	case E1000_DEV_ID_82546EB_QUAD_COPPER:
		return true;
	default:
		return false;
	}
}

/**
 * e1000_init_hw_struct - initialize members of hw struct
 * @adapter: board private struct
 * @hw: structure used by e1000_hw.c
 *
 * Factors out initialization of the e1000_hw struct to its own function
 * that can be called very early at init (just after struct allocation).
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 * Returns negative error codes if MAC type setup fails.
 */
static int e1000_init_hw_struct(struct e1000_adapter *adapter, struct e1000_hw *hw) {
	struct pci_dev *pdev = adapter->pdev;

	/* PCI config space info */
	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_id = pdev->subsystem_device;
	hw->revision_id = pdev->revision;

	pci_read_config_word(pdev, PCI_COMMAND, &hw->pci_cmd_word);

	hw->max_frame_size = adapter->netdev->mtu +
	ENET_HEADER_SIZE + ETHERNET_FCS_SIZE;
	hw->min_frame_size = MINIMUM_ETHERNET_FRAME_SIZE;

	/* identify the MAC */
	if (e1000_set_mac_type(hw)) {
		e_err(probe, "Unknown MAC Type\n");
		return -EIO;
	}

	switch (hw->mac_type) {
	default:
		break;
	case e1000_82541:
	case e1000_82547:
	case e1000_82541_rev_2:
	case e1000_82547_rev_2:
		hw->phy_init_script = 1;
		break;
	}

	e1000_set_media_type(hw);
	e1000_get_bus_info(hw);

	hw->wait_autoneg_complete = false;
	hw->tbi_compatibility_en = true;
	hw->adaptive_ifs = true;

	/* Copper options */

	if (hw->media_type == e1000_media_type_copper) {
		hw->mdix = AUTO_ALL_MODES;
		hw->disable_polarity_correction = false;
		hw->master_slave = E1000_MASTER_SLAVE;
	}

	return 0;
}

/* Dump the eeprom for users having checksum issues */
static void e1000_dump_eeprom(struct e1000_adapter *adapter) {
	struct net_device *netdev = adapter->netdev;
	struct ethtool_eeprom eeprom;
	const struct ethtool_ops *ops = netdev->ethtool_ops;
	u8 *data;
	int i;
	u16 csum_old, csum_new = 0;

	eeprom.len = ops->get_eeprom_len(netdev);
	eeprom.offset = 0;

	data = kmalloc(eeprom.len, GFP_KERNEL);
	if (!data)
		return;

	ops->get_eeprom(netdev, &eeprom, data);

	csum_old = (data[EEPROM_CHECKSUM_REG * 2]) + (data[EEPROM_CHECKSUM_REG * 2 + 1] << 8);
	for (i = 0; i < EEPROM_CHECKSUM_REG * 2; i += 2)
		csum_new += data[i] + (data[i + 1] << 8);
	csum_new = EEPROM_SUM - csum_new;

	pr_err("/*********************/\n");
	pr_err("Current EEPROM Checksum : 0x%04x\n", csum_old);
	pr_err("Calculated              : 0x%04x\n", csum_new);

	pr_err("Offset    Values\n");
	pr_err("========  ======\n");
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 16, 1, data, 128, 0);

	pr_err("Include this output when contacting your support provider.\n");
	pr_err("This is not a software error! Something bad happened to\n");
	pr_err("your hardware or EEPROM image. Ignoring this problem could\n");
	pr_err("result in further problems, possibly loss of data,\n");
	pr_err("corruption or system hangs!\n");
	pr_err("The MAC Address will be reset to 00:00:00:00:00:00,\n");
	pr_err("which is invalid and requires you to set the proper MAC\n");
	pr_err("address manually before continuing to enable this network\n");
	pr_err("device. Please inspect the EEPROM dump and report the\n");
	pr_err("issue to your hardware vendor or Intel Customer Support.\n");
	pr_err("/*********************/\n");

	kfree(data);
}

/**
 * e1000_update_phy_info_task - get phy info
 * @work: work struct contained inside adapter struct
 *
 * Need to wait a few seconds after link up to get diagnostic information from
 * the phy
 */
static void e1000_update_phy_info_task(struct work_struct *work) {
	struct e1000_adapter
	*adapter = container_of(work,
			struct e1000_adapter,
			phy_info_task.work);

	e1000_phy_get_info(&adapter->hw, &adapter->phy_info);
}

/**
 * e1000_clean - NAPI Rx polling callback
 * @adapter: board private structure
 **/
static int e1000_clean(struct napi_struct *napi, int budget) {
	struct e1000_adapter
	*adapter = container_of(napi, struct e1000_adapter,
			napi);
	int tx_clean_complete = 0, work_done = 0;

	tx_clean_complete = e1000_clean_tx_irq(adapter, &adapter->tx_ring[0]);

	adapter->clean_rx(adapter, &adapter->rx_ring[0], &work_done, budget);

	if (!tx_clean_complete)
		work_done = budget;

	/* If budget not fully consumed, exit the polling mode */
	if (work_done < budget) {
		if (likely(adapter->itr_setting & 3))
			e1000_set_itr(adapter);
		napi_complete_done(napi, work_done);
		if (!test_bit(__E1000_DOWN, &adapter->flags))
			e1000_irq_enable(adapter);
	}

	return work_done;
}

enum latency_range {
	lowest_latency = 0, low_latency = 1, bulk_latency = 2, latency_invalid = 255
};

static void e1000_set_itr(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u16 current_itr;
	u32 new_itr = adapter->itr;

	if (unlikely(hw->mac_type < e1000_82540))
		return;

	/* for non-gigabit speeds, just fix the interrupt rate at 4000 */
	if (unlikely(adapter->link_speed != SPEED_1000)) {
		current_itr = 0;
		new_itr = 4000;
		goto set_itr_now;
	}

	adapter->tx_itr = e1000_update_itr(adapter, adapter->tx_itr, adapter->total_tx_packets, adapter->total_tx_bytes);
	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (adapter->itr_setting == 3 && adapter->tx_itr == lowest_latency)
		adapter->tx_itr = low_latency;

	adapter->rx_itr = e1000_update_itr(adapter, adapter->rx_itr, adapter->total_rx_packets, adapter->total_rx_bytes);
	/* conservative mode (itr 3) eliminates the lowest_latency setting */
	if (adapter->itr_setting == 3 && adapter->rx_itr == lowest_latency)
		adapter->rx_itr = low_latency;

	current_itr = max(adapter->rx_itr, adapter->tx_itr);

	switch (current_itr) {
	/* counts and packets in update_itr are dependent on these numbers */
	case lowest_latency:
		new_itr = 70000;
		break;
	case low_latency:
		new_itr = 20000; /* aka hwitr = ~200 */
		break;
	case bulk_latency:
		new_itr = 4000;
		break;
	default:
		break;
	}

	set_itr_now: if (new_itr != adapter->itr) {
		/* this attempts to bias the interrupt rate towards Bulk
		 * by adding intermediate steps when interrupt rate is
		 * increasing
		 */
		new_itr = new_itr > adapter->itr ? min(adapter->itr + (new_itr >> 2), new_itr) : new_itr;
		adapter->itr = new_itr;
		ew32(ITR, 1000000000 / (new_itr * 256));
	}
}

/**
 * e1000_update_itr - update the dynamic ITR value based on statistics
 * @adapter: pointer to adapter
 * @itr_setting: current adapter->itr
 * @packets: the number of packets during this measurement interval
 * @bytes: the number of bytes during this measurement interval
 *
 *      Stores a new ITR value based on packets and byte
 *      counts during the last interrupt.  The advantage of per interrupt
 *      computation is faster updates and more accurate ITR for the current
 *      traffic pattern.  Constants in this function were computed
 *      based on theoretical maximum wire speed and thresholds were set based
 *      on testing data as well as attempting to minimize response time
 *      while increasing bulk throughput.
 *      this functionality is controlled by the InterruptThrottleRate module
 *      parameter (see e1000_param.c)
 **/
static unsigned int e1000_update_itr(struct e1000_adapter *adapter, u16 itr_setting, int packets, int bytes) {
	unsigned int retval = itr_setting;
	struct e1000_hw *hw = &adapter->hw;

	if (unlikely(hw->mac_type < e1000_82540))
		goto update_itr_done;

	if (packets == 0)
		goto update_itr_done;

	switch (itr_setting) {
	case lowest_latency:
		/* jumbo frames get bulk treatment*/
		if (bytes / packets > 8000)
			retval = bulk_latency;
		else if ((packets < 5) && (bytes > 512))
			retval = low_latency;
		break;
	case low_latency: /* 50 usec aka 20000 ints/s */
		if (bytes > 10000) {
			/* jumbo frames need bulk latency setting */
			if (bytes / packets > 8000)
				retval = bulk_latency;
			else if ((packets < 10) || ((bytes / packets) > 1200))
				retval = bulk_latency;
			else if ((packets > 35))
				retval = lowest_latency;
		} else if (bytes / packets > 2000)
			retval = bulk_latency;
		else if (packets <= 2 && bytes < 512)
			retval = lowest_latency;
		break;
	case bulk_latency: /* 250 usec aka 4000 ints/s */
		if (bytes > 25000) {
			if (packets > 35)
				retval = low_latency;
		} else if (bytes < 6000) {
			retval = low_latency;
		}
		break;
	}

	update_itr_done: return retval;
}

/**
 * e1000_watchdog - work function
 * @work: work struct contained inside adapter struct
 **/
static void e1000_watchdog(struct work_struct *work) {
	struct e1000_adapter
	*adapter = container_of(work,
			struct e1000_adapter,
			watchdog_task.work);
	struct e1000_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	struct e1000_tx_ring *txdr = adapter->tx_ring;
	u32 link, tctl;

	link = e1000_has_link(adapter);
	if ((netif_carrier_ok(netdev)) && link)
		goto link_up;

	if (link) {
		if (!netif_carrier_ok(netdev)) {
			u32 ctrl;
			bool txb2b = true;
			/* update snapshot of PHY registers on LSC */
			e1000_get_speed_and_duplex(hw, &adapter->link_speed, &adapter->link_duplex);

			ctrl = er32(CTRL);
			pr_info("%s NIC Link is Up %d Mbps %s, "
					"Flow Control: %s\n", netdev->name, adapter->link_speed, adapter->link_duplex == FULL_DUPLEX ? "Full Duplex" : "Half Duplex", ((ctrl & E1000_CTRL_TFCE) && (ctrl &
			E1000_CTRL_RFCE)) ? "RX/TX" : ((ctrl &
			E1000_CTRL_RFCE) ? "RX" : ((ctrl &
			E1000_CTRL_TFCE) ? "TX" : "None")));

			/* adjust timeout factor according to speed/duplex */
			adapter->tx_timeout_factor = 1;
			switch (adapter->link_speed) {
			case SPEED_10:
				txb2b = false;
				adapter->tx_timeout_factor = 16;
				break;
			case SPEED_100:
				txb2b = false;
				/* maybe add some timeout factor ? */
				break;
			}

			/* enable transmits in the hardware */
			tctl = er32(TCTL);
			tctl |= E1000_TCTL_EN;
			ew32(TCTL, tctl);

			netif_carrier_on(netdev);
			if (!test_bit(__E1000_DOWN, &adapter->flags))
				schedule_delayed_work(&adapter->phy_info_task, 2 * HZ);
			adapter->smartspeed = 0;
		}
	} else {
		if (netif_carrier_ok(netdev)) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;
			pr_info("%s NIC Link is Down\n", netdev->name);
			netif_carrier_off(netdev);

			if (!test_bit(__E1000_DOWN, &adapter->flags))
				schedule_delayed_work(&adapter->phy_info_task, 2 * HZ);
		}

		e1000_smartspeed(adapter);
	}

	link_up: e1000_update_stats(adapter);

	hw->tx_packet_delta = adapter->stats.tpt - adapter->tpt_old;
	adapter->tpt_old = adapter->stats.tpt;
	hw->collision_delta = adapter->stats.colc - adapter->colc_old;
	adapter->colc_old = adapter->stats.colc;

	adapter->gorcl = adapter->stats.gorcl - adapter->gorcl_old;
	adapter->gorcl_old = adapter->stats.gorcl;
	adapter->gotcl = adapter->stats.gotcl - adapter->gotcl_old;
	adapter->gotcl_old = adapter->stats.gotcl;

	e1000_update_adaptive(hw);

	if (!netif_carrier_ok(netdev)) {
		if (E1000_DESC_UNUSED(txdr) + 1 < txdr->count) {
			/* We've lost link, so the controller stops DMA,
			 * but we've got queued Tx work that's never going
			 * to get done, so reset controller to flush Tx.
			 * (Do the reset outside of interrupt context).
			 */
			adapter->tx_timeout_count++;
			schedule_work(&adapter->reset_task);
			/* exit immediately since reset is imminent */
			return;
		}
	}

	/* Simple mode for Interrupt Throttle Rate (ITR) */
	if (hw->mac_type >= e1000_82540 && adapter->itr_setting == 4) {
		/* Symmetric Tx/Rx gets a reduced ITR=2000;
		 * Total asymmetrical Tx or Rx gets ITR=8000;
		 * everyone else is between 2000-8000.
		 */
		u32 goc = (adapter->gotcl + adapter->gorcl) / 10000;
		u32 dif = (adapter->gotcl > adapter->gorcl ? adapter->gotcl - adapter->gorcl : adapter->gorcl - adapter->gotcl) / 10000;
		u32 itr = goc > 0 ? (dif * 6000 / goc + 2000) : 8000;

		ew32(ITR, 1000000000 / (itr * 256));
	}

	/* Cause software interrupt to ensure rx ring is cleaned */
	ew32(ICS, E1000_ICS_RXDMT0);

	/* Force detection of hung controller every watchdog period */
	adapter->detect_tx_hung = true;

	/* Reschedule the task */
	if (!test_bit(__E1000_DOWN, &adapter->flags))
		schedule_delayed_work(&adapter->watchdog_task, 2 * HZ);
}

static void e1000_reset_task(struct work_struct *work) {
	struct e1000_adapter
*adapter =
		container_of(work, struct e1000_adapter, reset_task);

																																																																																																																																																																																																																																																		e_err(drv, "Reset adapter\n");
	e1000_reinit_locked(adapter);
}

/**
 * e1000_82547_tx_fifo_stall_task - task to complete work
 * @work: work struct contained inside adapter struct
 **/
static void e1000_82547_tx_fifo_stall_task(struct work_struct *work) {
	struct e1000_adapter
	*adapter = container_of(work,
			struct e1000_adapter,
			fifo_stall_task.work);
	struct e1000_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	u32 tctl;

	if (atomic_read(&adapter->tx_fifo_stall)) {
		if ((er32(TDT) == er32(TDH)) && (er32(TDFT) == er32(TDFH)) && (er32(TDFTS) == er32(TDFHS))) {
			tctl = er32(TCTL);
			ew32(TCTL, tctl & ~E1000_TCTL_EN);
			ew32(TDFT, adapter->tx_head_addr);
			ew32(TDFH, adapter->tx_head_addr);
			ew32(TDFTS, adapter->tx_head_addr);
			ew32(TDFHS, adapter->tx_head_addr);
			ew32(TCTL, tctl);
			E1000_WRITE_FLUSH();

			adapter->tx_fifo_head = 0;
			atomic_set(&adapter->tx_fifo_stall, 0);
			netif_wake_queue(netdev);
		} else if (!test_bit(__E1000_DOWN, &adapter->flags)) {
			schedule_delayed_work(&adapter->fifo_stall_task, 1);
		}
	}
}

/**
 * e1000_sw_init - Initialize general software structures (struct e1000_adapter)
 * @adapter: board private structure to initialize
 *
 * e1000_sw_init initializes the Adapter private data structure.
 * e1000_init_hw_struct MUST be called before this function
 **/
static int e1000_sw_init(struct e1000_adapter *adapter) {
	adapter->rx_buffer_len = MAXIMUM_ETHERNET_VLAN_SIZE;

	adapter->num_tx_queues = 1;
	adapter->num_rx_queues = 1;

	if (e1000_alloc_queues(adapter)) {
		e_err(probe, "Unable to allocate memory for queues\n");
		return -ENOMEM;
	}

	/* Explicitly disable IRQ since the NIC can be in any state. */
	e1000_irq_disable(adapter);

	spin_lock_init(&adapter->stats_lock);

	set_bit(__E1000_DOWN, &adapter->flags);

	return 0;
}

/**
 * e1000_clean_tx_irq - Reclaim resources after transmit completes
 * @adapter: board private structure
 **/
static bool e1000_clean_tx_irq(struct e1000_adapter *adapter, struct e1000_tx_ring *tx_ring) {
	struct e1000_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	struct e1000_tx_desc *tx_desc, *eop_desc;
	struct e1000_tx_buffer *buffer_info;
	unsigned int i, eop;
	unsigned int count = 0;
	unsigned int total_tx_bytes = 0, total_tx_packets = 0;
	unsigned int bytes_compl = 0, pkts_compl = 0;

	i = tx_ring->next_to_clean;
	eop = tx_ring->buffer_info[i].next_to_watch;
	eop_desc = E1000_TX_DESC(*tx_ring, eop);

	while ((eop_desc->upper.data & cpu_to_le32(E1000_TXD_STAT_DD)) && (count < tx_ring->count)) {
		bool cleaned = false;
		dma_rmb(); /* read buffer_info after eop_desc */
		for (; !cleaned; count++) {
			tx_desc = E1000_TX_DESC(*tx_ring, i);
			buffer_info = &tx_ring->buffer_info[i];
			cleaned = (i == eop);

			if (cleaned) {
				total_tx_packets += buffer_info->segs;
				total_tx_bytes += buffer_info->bytecount;
				if (buffer_info->skb) {
					bytes_compl += buffer_info->skb->len;
					pkts_compl++;
				}

			}
			e1000_unmap_and_free_tx_resource(adapter, buffer_info);
			tx_desc->upper.data = 0;

			if (unlikely(++i == tx_ring->count))
				i = 0;
		}

		eop = tx_ring->buffer_info[i].next_to_watch;
		eop_desc = E1000_TX_DESC(*tx_ring, eop);
	}

	/* Synchronize with E1000_DESC_UNUSED called from e1000_xmit_frame,
	 * which will reuse the cleaned buffers.
	 */
	smp_store_release(&tx_ring->next_to_clean, i);

	netdev_completed_queue(netdev, pkts_compl, bytes_compl);

#define TX_WAKE_THRESHOLD 32
	if (unlikely(count && netif_carrier_ok(netdev) &&
	E1000_DESC_UNUSED(tx_ring) >= TX_WAKE_THRESHOLD)) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();

		if (netif_queue_stopped(netdev) && !(test_bit(__E1000_DOWN, &adapter->flags))) {
			netif_wake_queue(netdev);
			++adapter->restart_queue;
		}
	}

	if (adapter->detect_tx_hung) {
		/* Detect a transmit hang in hardware, this serializes the
		 * check with the clearing of time_stamp and movement of i
		 */
		adapter->detect_tx_hung = false;
		if (tx_ring->buffer_info[eop].time_stamp && time_after(jiffies, tx_ring->buffer_info[eop].time_stamp + (adapter->tx_timeout_factor * HZ)) && !(er32(STATUS) & E1000_STATUS_TXOFF)) {

			/* detected Tx unit hang */
			e_err(drv, "Detected Tx Unit Hang\n"
					"  Tx Queue             <%lu>\n"
					"  TDH                  <%x>\n"
					"  TDT                  <%x>\n"
					"  next_to_use          <%x>\n"
					"  next_to_clean        <%x>\n"
					"buffer_info[next_to_clean]\n"
					"  time_stamp           <%lx>\n"
					"  next_to_watch        <%x>\n"
					"  jiffies              <%lx>\n"
					"  next_to_watch.status <%x>\n", (unsigned long )(tx_ring - adapter->tx_ring), readl(hw->hw_addr + tx_ring->tdh), readl(hw->hw_addr + tx_ring->tdt), tx_ring->next_to_use,
					tx_ring->next_to_clean, tx_ring->buffer_info[eop].time_stamp, eop, jiffies, eop_desc->upper.fields.status);
			e1000_dump(adapter);
			netif_stop_queue(netdev);
		}
	}
	adapter->total_tx_bytes += total_tx_bytes;
	adapter->total_tx_packets += total_tx_packets;
	netdev->stats.tx_bytes += total_tx_bytes;
	netdev->stats.tx_packets += total_tx_packets;
	return count < tx_ring->count;
}

/**
 * e1000_smartspeed - Workaround for SmartSpeed on 82541 and 82547 controllers.
 * @adapter:
 **/
static void e1000_smartspeed(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u16 phy_status;
	u16 phy_ctrl;

	if ((hw->phy_type != e1000_phy_igp) || !hw->autoneg || !(hw->autoneg_advertised & ADVERTISE_1000_FULL))
		return;

	if (adapter->smartspeed == 0) {
		/* If Master/Slave config fault is asserted twice,
		 * we assume back-to-back
		 */
		e1000_read_phy_reg(hw, PHY_1000T_STATUS, &phy_status);
		if (!(phy_status & SR_1000T_MS_CONFIG_FAULT))
			return;
		e1000_read_phy_reg(hw, PHY_1000T_STATUS, &phy_status);
		if (!(phy_status & SR_1000T_MS_CONFIG_FAULT))
			return;
		e1000_read_phy_reg(hw, PHY_1000T_CTRL, &phy_ctrl);
		if (phy_ctrl & CR_1000T_MS_ENABLE) {
			phy_ctrl &= ~CR_1000T_MS_ENABLE;
			e1000_write_phy_reg(hw, PHY_1000T_CTRL, phy_ctrl);
			adapter->smartspeed++;
			if (!e1000_phy_setup_autoneg(hw) && !e1000_read_phy_reg(hw, PHY_CTRL, &phy_ctrl)) {
				phy_ctrl |= (MII_CR_AUTO_NEG_EN |
				MII_CR_RESTART_AUTO_NEG);
				e1000_write_phy_reg(hw, PHY_CTRL, phy_ctrl);
			}
		}
		return;
	} else if (adapter->smartspeed == E1000_SMARTSPEED_DOWNSHIFT) {
		/* If still no link, perhaps using 2/3 pair cable */
		e1000_read_phy_reg(hw, PHY_1000T_CTRL, &phy_ctrl);
		phy_ctrl |= CR_1000T_MS_ENABLE;
		e1000_write_phy_reg(hw, PHY_1000T_CTRL, phy_ctrl);
		if (!e1000_phy_setup_autoneg(hw) && !e1000_read_phy_reg(hw, PHY_CTRL, &phy_ctrl)) {
			phy_ctrl |= (MII_CR_AUTO_NEG_EN |
			MII_CR_RESTART_AUTO_NEG);
			e1000_write_phy_reg(hw, PHY_CTRL, phy_ctrl);
		}
	}
	/* Restart process after E1000_SMARTSPEED_MAX iterations */
	if (adapter->smartspeed++ == E1000_SMARTSPEED_MAX)
		adapter->smartspeed = 0;
}

/*
 * e1000_dump: Print registers, tx ring and rx ring
 */
static void e1000_dump(struct e1000_adapter *adapter) {
	/* this code doesn't handle multiple rings */
	struct e1000_tx_ring *tx_ring = adapter->tx_ring;
	struct e1000_rx_ring *rx_ring = adapter->rx_ring;
	int i;

	if (!netif_msg_hw(adapter))
		return;

	/* Print Registers */
	e1000_regdump(adapter);

	/* transmit dump */
	pr_info("TX Desc ring0 dump\n");

	/* Transmit Descriptor Formats - DEXT[29] is 0 (Legacy) or 1 (Extended)
	 *
	 * Legacy Transmit Descriptor
	 *   +--------------------------------------------------------------+
	 * 0 |         Buffer Address [63:0] (Reserved on Write Back)       |
	 *   +--------------------------------------------------------------+
	 * 8 | Special  |    CSS     | Status |  CMD    |  CSO   |  Length  |
	 *   +--------------------------------------------------------------+
	 *   63       48 47        36 35    32 31     24 23    16 15        0
	 *
	 * Extended Context Descriptor (DTYP=0x0) for TSO or checksum offload
	 *   63      48 47    40 39       32 31             16 15    8 7      0
	 *   +----------------------------------------------------------------+
	 * 0 |  TUCSE  | TUCS0  |   TUCSS   |     IPCSE       | IPCS0 | IPCSS |
	 *   +----------------------------------------------------------------+
	 * 8 |   MSS   | HDRLEN | RSV | STA | TUCMD | DTYP |      PAYLEN      |
	 *   +----------------------------------------------------------------+
	 *   63      48 47    40 39 36 35 32 31   24 23  20 19                0
	 *
	 * Extended Data Descriptor (DTYP=0x1)
	 *   +----------------------------------------------------------------+
	 * 0 |                     Buffer Address [63:0]                      |
	 *   +----------------------------------------------------------------+
	 * 8 | VLAN tag |  POPTS  | Rsvd | Status | Command | DTYP |  DTALEN  |
	 *   +----------------------------------------------------------------+
	 *   63       48 47     40 39  36 35    32 31     24 23  20 19        0
	 */
	pr_info("Tc[desc]     [Ce CoCsIpceCoS] [MssHlRSCm0Plen] [bi->dma       ] leng  ntw timestmp         bi->skb\n");
	pr_info("Td[desc]     [address 63:0  ] [VlaPoRSCm1Dlen] [bi->dma       ] leng  ntw timestmp         bi->skb\n");

	if (!netif_msg_tx_done(adapter))
		goto rx_ring_summary;

	for (i = 0; tx_ring->desc && (i < tx_ring->count); i++) {
		struct e1000_tx_desc *tx_desc = E1000_TX_DESC(*tx_ring, i);
		struct e1000_tx_buffer *buffer_info = &tx_ring->buffer_info[i];
		struct my_u {
			__le64 a;
			__le64 b;
		};
		struct my_u *u = (struct my_u *) tx_desc;
		const char *type;

		if (i == tx_ring->next_to_use && i == tx_ring->next_to_clean)
			type = "NTC/U";
		else if (i == tx_ring->next_to_use)
			type = "NTU";
		else if (i == tx_ring->next_to_clean)
			type = "NTC";
		else
			type = "";

		pr_info("T%c[0x%03X]    %016llX %016llX %016llX %04X  %3X %016llX %p %s\n", ((le64_to_cpu(u->b) & (1 << 20)) ? 'd' : 'c'), i, le64_to_cpu(u->a), le64_to_cpu(u->b), (u64) buffer_info->dma,
				buffer_info->length, buffer_info->next_to_watch, (u64) buffer_info->time_stamp, buffer_info->skb, type);
	}

	rx_ring_summary:
	/* receive dump */
	pr_info("\nRX Desc ring dump\n");

	/* Legacy Receive Descriptor Format
	 *
	 * +-----------------------------------------------------+
	 * |                Buffer Address [63:0]                |
	 * +-----------------------------------------------------+
	 * | VLAN Tag | Errors | Status 0 | Packet csum | Length |
	 * +-----------------------------------------------------+
	 * 63       48 47    40 39      32 31         16 15      0
	 */
	pr_info("R[desc]      [address 63:0  ] [vl er S cks ln] [bi->dma       ] [bi->skb]\n");

	if (!netif_msg_rx_status(adapter))
		goto exit;

	for (i = 0; rx_ring->desc && (i < rx_ring->count); i++) {
		struct e1000_rx_desc *rx_desc = E1000_RX_DESC(*rx_ring, i);
		struct e1000_rx_buffer *buffer_info = &rx_ring->buffer_info[i];
		struct my_u {
			__le64 a;
			__le64 b;
		};
		struct my_u *u = (struct my_u *) rx_desc;
		const char *type;

		if (i == rx_ring->next_to_use)
			type = "NTU";
		else if (i == rx_ring->next_to_clean)
			type = "NTC";
		else
			type = "";

		pr_info("R[0x%03X]     %016llX %016llX %016llX %p %s\n", i, le64_to_cpu(u->a), le64_to_cpu(u->b), (u64) buffer_info->dma, buffer_info->rxbuf.data, type);
	} /* for */

	/* dump the descriptor caches */
	/* rx */
	pr_info("Rx descriptor cache in 64bit format\n");
	for (i = 0x6000; i <= 0x63FF; i += 0x10) {
		pr_info("R%04X: %08X|%08X %08X|%08X\n", i, readl(adapter->hw.hw_addr + i + 4), readl(adapter->hw.hw_addr + i), readl(adapter->hw.hw_addr + i + 12), readl(adapter->hw.hw_addr + i + 8));
	}
	/* tx */
	pr_info("Tx descriptor cache in 64bit format\n");
	for (i = 0x7000; i <= 0x73FF; i += 0x10) {
		pr_info("T%04X: %08X|%08X %08X|%08X\n", i, readl(adapter->hw.hw_addr + i + 4), readl(adapter->hw.hw_addr + i), readl(adapter->hw.hw_addr + i + 12), readl(adapter->hw.hw_addr + i + 8));
	}
	exit: return;
}

#define NUM_REGS 38 /* 1 based count */
static void e1000_regdump(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u32 regs[NUM_REGS];
	u32 *regs_buff = regs;
	int i = 0;

	static const char * const reg_name[] = {
		"CTRL",
		"STATUS",
		"RCTL",
		"RDLEN",
		"RDH",
		"RDT",
		"RDTR",
		"TCTL",
		"TDBAL",
		"TDBAH",
		"TDLEN",
		"TDH",
		"TDT",
		"TIDV",
		"TXDCTL",
		"TADV",
		"TARC0",
		"TDBAL1",
		"TDBAH1",
		"TDLEN1",
		"TDH1",
		"TDT1",
		"TXDCTL1",
		"TARC1",
		"CTRL_EXT",
		"ERT",
		"RDBAL",
		"RDBAH",
		"TDFH",
		"TDFT",
		"TDFHS",
		"TDFTS",
		"TDFPC",
		"RDFH",
		"RDFT",
		"RDFHS",
		"RDFTS",
		"RDFPC" };

	regs_buff[0] = er32(CTRL);
	regs_buff[1] = er32(STATUS);

	regs_buff[2] = er32(RCTL);
	regs_buff[3] = er32(RDLEN);
	regs_buff[4] = er32(RDH);
	regs_buff[5] = er32(RDT);
	regs_buff[6] = er32(RDTR);

	regs_buff[7] = er32(TCTL);
	regs_buff[8] = er32(TDBAL);
	regs_buff[9] = er32(TDBAH);
	regs_buff[10] = er32(TDLEN);
	regs_buff[11] = er32(TDH);
	regs_buff[12] = er32(TDT);
	regs_buff[13] = er32(TIDV);
	regs_buff[14] = er32(TXDCTL);
	regs_buff[15] = er32(TADV);
	regs_buff[16] = er32(TARC0);

	regs_buff[17] = er32(TDBAL1);
	regs_buff[18] = er32(TDBAH1);
	regs_buff[19] = er32(TDLEN1);
	regs_buff[20] = er32(TDH1);
	regs_buff[21] = er32(TDT1);
	regs_buff[22] = er32(TXDCTL1);
	regs_buff[23] = er32(TARC1);
	regs_buff[24] = er32(CTRL_EXT);
	regs_buff[25] = er32(ERT);
	regs_buff[26] = er32(RDBAL0);
	regs_buff[27] = er32(RDBAH0);
	regs_buff[28] = er32(TDFH);
	regs_buff[29] = er32(TDFT);
	regs_buff[30] = er32(TDFHS);
	regs_buff[31] = er32(TDFTS);
	regs_buff[32] = er32(TDFPC);
	regs_buff[33] = er32(RDFH);
	regs_buff[34] = er32(RDFT);
	regs_buff[35] = er32(RDFHS);
	regs_buff[36] = er32(RDFTS);
	regs_buff[37] = er32(RDFPC);

	pr_info("Register dump\n");
	for (i = 0; i < NUM_REGS; i++)
		pr_info("%-15s  %08x\n", reg_name[i], regs_buff[i]);
}

/**
 * e1000_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 *
 * We allocate one ring per queue at run-time since we don't know the
 * number of queues at compile-time.
 **/
static int e1000_alloc_queues(struct e1000_adapter *adapter) {
	adapter->tx_ring = kcalloc(adapter->num_tx_queues, sizeof(struct e1000_tx_ring), GFP_KERNEL);
	if (!adapter->tx_ring)
		return -ENOMEM;

	adapter->rx_ring = kcalloc(adapter->num_rx_queues, sizeof(struct e1000_rx_ring), GFP_KERNEL);
	if (!adapter->rx_ring) {
		kfree(adapter->tx_ring);
		return -ENOMEM;
	}

	return E1000_SUCCESS;
}
