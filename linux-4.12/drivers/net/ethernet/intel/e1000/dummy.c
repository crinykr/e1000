#include "e1000.h"

/**
 * e1000_get_hw_dev - return device
 * used by hardware layer to print debugging information
 *
 **/
struct net_device *e1000_get_hw_dev(struct e1000_hw *hw) {
	struct e1000_adapter *adapter = hw->back;
	return adapter->netdev;
}

static int e1000_link_test(struct e1000_adapter *adapter, u64 *data);
static int e1000_reg_test(struct e1000_adapter *adapter, u64 *data);
static int e1000_eeprom_test(struct e1000_adapter *adapter, u64 *data);
static int e1000_intr_test(struct e1000_adapter *adapter, u64 *data);
static int e1000_loopback_test(struct e1000_adapter *adapter, u64 *data);
static irqreturn_t e1000_test_intr(int irq, void *data);
static bool reg_pattern_test(struct e1000_adapter *adapter, u64 *data, int reg, u32 mask, u32 write);
static bool reg_set_and_check(struct e1000_adapter *adapter, u64 *data, int reg, u32 mask, u32 write);
static int e1000_setup_desc_rings(struct e1000_adapter *adapter);
static int e1000_setup_loopback_test(struct e1000_adapter *adapter);
static int e1000_run_loopback_test(struct e1000_adapter *adapter);
static void e1000_loopback_cleanup(struct e1000_adapter *adapter);
static void e1000_free_desc_rings(struct e1000_adapter *adapter);
static int e1000_set_phy_loopback(struct e1000_adapter *adapter);
static void e1000_create_lbtest_frame(struct sk_buff *skb, unsigned int frame_size);
static int e1000_check_lbtest_frame(const unsigned char *data, unsigned int frame_size);
static int e1000_nonintegrated_phy_loopback(struct e1000_adapter *adapter);
static int e1000_integrated_phy_loopback(struct e1000_adapter *adapter);
static void e1000_phy_reset_clk_and_crs(struct e1000_adapter *adapter);
static void e1000_phy_disable_receiver(struct e1000_adapter *adapter);

void e1000_diag_test(struct net_device *netdev, struct ethtool_test *eth_test, u64 *data) {
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	bool if_running = netif_running(netdev);

	set_bit(__E1000_TESTING, &adapter->flags);
	if (eth_test->flags == ETH_TEST_FL_OFFLINE) {
		/* Offline tests */

		/* save speed, duplex, autoneg settings */
		u16 autoneg_advertised = hw->autoneg_advertised;
		u8 forced_speed_duplex = hw->forced_speed_duplex;
		u8 autoneg = hw->autoneg;

		e_info(hw, "offline testing starting\n");

		/* Link test performed before hardware reset so autoneg doesn't
		 * interfere with test result
		 */
		if (e1000_link_test(adapter, &data[4]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		if (if_running)
			/* indicate we're in test mode */
			e1000_close(netdev);
		else
			e1000_reset(adapter);

		if (e1000_reg_test(adapter, &data[0]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		e1000_reset(adapter);
		if (e1000_eeprom_test(adapter, &data[1]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		e1000_reset(adapter);
		if (e1000_intr_test(adapter, &data[2]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		e1000_reset(adapter);
		/* make sure the phy is powered up */
		e1000_power_up_phy(adapter);
		if (e1000_loopback_test(adapter, &data[3]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		/* restore speed, duplex, autoneg settings */
		hw->autoneg_advertised = autoneg_advertised;
		hw->forced_speed_duplex = forced_speed_duplex;
		hw->autoneg = autoneg;

		e1000_reset(adapter);
		clear_bit(__E1000_TESTING, &adapter->flags);
		if (if_running)
			e1000_open(netdev);
	} else {
		e_info(hw, "online testing starting\n");
		/* Online tests */
		if (e1000_link_test(adapter, &data[4]))
			eth_test->flags |= ETH_TEST_FL_FAILED;

		/* Online tests aren't run; pass by default */
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;

		clear_bit(__E1000_TESTING, &adapter->flags);
	}
	msleep_interruptible(4 * 1000);
}

static int e1000_link_test(struct e1000_adapter *adapter, u64 *data) {
	struct e1000_hw *hw = &adapter->hw;
	*data = 0;
	if (hw->media_type == e1000_media_type_internal_serdes) {
		int i = 0;

		hw->serdes_has_link = false;

		/* On some blade server designs, link establishment
		 * could take as long as 2-3 minutes
		 */
		do {
			e1000_check_for_link(hw);
			if (hw->serdes_has_link)
				return *data;
			msleep(20);
		} while (i++ < 3750);

		*data = 1;
	} else {
		e1000_check_for_link(hw);
		if (hw->autoneg) /* if auto_neg is set wait for it */
			msleep(4000);

		if (!(er32(STATUS) & E1000_STATUS_LU))
			*data = 1;
	}
	return *data;
}

#define REG_PATTERN_TEST(reg, mask, write)			     \
	do {							     \
		if (reg_pattern_test(adapter, data,		     \
			     (hw->mac_type >= e1000_82543)   \
			     ? E1000_##reg : E1000_82542_##reg,	     \
			     mask, write))			     \
			return 1;				     \
	} while (0)

#define REG_SET_AND_CHECK(reg, mask, write)			     \
	do {							     \
		if (reg_set_and_check(adapter, data,		     \
			      (hw->mac_type >= e1000_82543)  \
			      ? E1000_##reg : E1000_82542_##reg,     \
			      mask, write))			     \
			return 1;				     \
	} while (0)

static int e1000_reg_test(struct e1000_adapter *adapter, u64 *data) {
	u32 value, before, after;
	u32 i, toggle;
	struct e1000_hw *hw = &adapter->hw;

	/* The status register is Read Only, so a write should fail.
	 * Some bits that get toggled are ignored.
	 */

	/* there are several bits on newer hardware that are r/w */
	toggle = 0xFFFFF833;

	before = er32(STATUS);
	value = (er32(STATUS) & toggle);
	ew32(STATUS, toggle);
	after = er32(STATUS) & toggle;
	if (value != after) {
		e_err(drv, "failed STATUS register test got: "
			"0x%08X expected: 0x%08X\n", after, value);
		*data = 1;
		return 1;
	}
	/* restore previous status */
	ew32(STATUS, before);

	REG_PATTERN_TEST(FCAL, 0xFFFFFFFF, 0xFFFFFFFF);
	REG_PATTERN_TEST(FCAH, 0x0000FFFF, 0xFFFFFFFF);
	REG_PATTERN_TEST(FCT, 0x0000FFFF, 0xFFFFFFFF);
	REG_PATTERN_TEST(VET, 0x0000FFFF, 0xFFFFFFFF);

	REG_PATTERN_TEST(RDTR, 0x0000FFFF, 0xFFFFFFFF);
	REG_PATTERN_TEST(RDBAH, 0xFFFFFFFF, 0xFFFFFFFF);
	REG_PATTERN_TEST(RDLEN, 0x000FFF80, 0x000FFFFF);
	REG_PATTERN_TEST(RDH, 0x0000FFFF, 0x0000FFFF);
	REG_PATTERN_TEST(RDT, 0x0000FFFF, 0x0000FFFF);
	REG_PATTERN_TEST(FCRTH, 0x0000FFF8, 0x0000FFF8);
	REG_PATTERN_TEST(FCTTV, 0x0000FFFF, 0x0000FFFF);
	REG_PATTERN_TEST(TIPG, 0x3FFFFFFF, 0x3FFFFFFF);
	REG_PATTERN_TEST(TDBAH, 0xFFFFFFFF, 0xFFFFFFFF);
	REG_PATTERN_TEST(TDLEN, 0x000FFF80, 0x000FFFFF);

	REG_SET_AND_CHECK(RCTL, 0xFFFFFFFF, 0x00000000);

	before = 0x06DFB3FE;
	REG_SET_AND_CHECK(RCTL, before, 0x003FFFFB);
	REG_SET_AND_CHECK(TCTL, 0xFFFFFFFF, 0x00000000);

	if (hw->mac_type >= e1000_82543) {
		REG_SET_AND_CHECK(RCTL, before, 0xFFFFFFFF);
		REG_PATTERN_TEST(RDBAL, 0xFFFFFFF0, 0xFFFFFFFF);
		REG_PATTERN_TEST(TXCW, 0xC000FFFF, 0x0000FFFF);
		REG_PATTERN_TEST(TDBAL, 0xFFFFFFF0, 0xFFFFFFFF);
		REG_PATTERN_TEST(TIDV, 0x0000FFFF, 0x0000FFFF);
		value = E1000_RAR_ENTRIES;
		for (i = 0; i < value; i++) {
			REG_PATTERN_TEST(RA + (((i << 1) + 1) << 2), 0x8003FFFF, 0xFFFFFFFF);
		}
	} else {
		REG_SET_AND_CHECK(RCTL, 0xFFFFFFFF, 0x01FFFFFF);
		REG_PATTERN_TEST(RDBAL, 0xFFFFF000, 0xFFFFFFFF);
		REG_PATTERN_TEST(TXCW, 0x0000FFFF, 0x0000FFFF);
		REG_PATTERN_TEST(TDBAL, 0xFFFFF000, 0xFFFFFFFF);
	}

	value = E1000_MC_TBL_SIZE;
	for (i = 0; i < value; i++)
		REG_PATTERN_TEST(MTA + (i << 2), 0xFFFFFFFF, 0xFFFFFFFF);

	*data = 0;
	return 0;
}

static int e1000_eeprom_test(struct e1000_adapter *adapter, u64 *data) {
	struct e1000_hw *hw = &adapter->hw;
	u16 temp;
	u16 checksum = 0;
	u16 i;

	*data = 0;
	/* Read and add up the contents of the EEPROM */
	for (i = 0; i < (EEPROM_CHECKSUM_REG + 1); i++) {
		if ((e1000_read_eeprom(hw, i, 1, &temp)) < 0) {
			*data = 1;
			break;
		}
		checksum += temp;
	}

	/* If Checksum is not Correct return error else test passed */
	if ((checksum != (u16) EEPROM_SUM) && !(*data))
		*data = 2;

	return *data;
}

static int e1000_intr_test(struct e1000_adapter *adapter, u64 *data) {
	struct net_device *netdev = adapter->netdev;
	u32 mask, i = 0;
	bool shared_int = true;
	u32 irq = adapter->pdev->irq;
	struct e1000_hw *hw = &adapter->hw;

	*data = 0;

	/* NOTE: we don't test MSI interrupts here, yet
	 * Hook up test interrupt handler just for this test
	 */
	if (!request_irq(irq, e1000_test_intr, IRQF_PROBE_SHARED, netdev->name, netdev))
		shared_int = false;
	else if (request_irq(irq, e1000_test_intr, IRQF_SHARED, netdev->name, netdev)) {
		*data = 1;
		return -1;
	}
	e_info(hw, "testing %s interrupt\n", (shared_int ? "shared" : "unshared"));

	/* Disable all the interrupts */
	ew32(IMC, 0xFFFFFFFF);
	E1000_WRITE_FLUSH();
	msleep(10);

	/* Test each interrupt */
	for (; i < 10; i++) {
		/* Interrupt to test */
		mask = 1 << i;

		if (!shared_int) {
			/* Disable the interrupt to be reported in
			 * the cause register and then force the same
			 * interrupt and see if one gets posted.  If
			 * an interrupt was posted to the bus, the
			 * test failed.
			 */
			adapter->test_icr = 0;
			ew32(IMC, mask);
			ew32(ICS, mask);
			E1000_WRITE_FLUSH();
			msleep(10);

			if (adapter->test_icr & mask) {
				*data = 3;
				break;
			}
		}

		/* Enable the interrupt to be reported in
		 * the cause register and then force the same
		 * interrupt and see if one gets posted.  If
		 * an interrupt was not posted to the bus, the
		 * test failed.
		 */
		adapter->test_icr = 0;
		ew32(IMS, mask);
		ew32(ICS, mask);
		E1000_WRITE_FLUSH();
		msleep(10);

		if (!(adapter->test_icr & mask)) {
			*data = 4;
			break;
		}

		if (!shared_int) {
			/* Disable the other interrupts to be reported in
			 * the cause register and then force the other
			 * interrupts and see if any get posted.  If
			 * an interrupt was posted to the bus, the
			 * test failed.
			 */
			adapter->test_icr = 0;
			ew32(IMC, ~mask & 0x00007FFF);
			ew32(ICS, ~mask & 0x00007FFF);
			E1000_WRITE_FLUSH();
			msleep(10);

			if (adapter->test_icr) {
				*data = 5;
				break;
			}
		}
	}

	/* Disable all the interrupts */
	ew32(IMC, 0xFFFFFFFF);
	E1000_WRITE_FLUSH();
	msleep(10);

	/* Unhook test interrupt handler */
	free_irq(irq, netdev);

	return *data;
}

static int e1000_loopback_test(struct e1000_adapter *adapter, u64 *data) {
	*data = e1000_setup_desc_rings(adapter);
	if (*data)
		goto out;
	*data = e1000_setup_loopback_test(adapter);
	if (*data)
		goto err_loopback;
	*data = e1000_run_loopback_test(adapter);
	e1000_loopback_cleanup(adapter);

	err_loopback: e1000_free_desc_rings(adapter);
	out: return *data;
}

static irqreturn_t e1000_test_intr(int irq, void *data) {
	struct net_device *netdev = (struct net_device *) data;
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;

	adapter->test_icr |= er32(ICR);

	return IRQ_HANDLED;
}

static bool reg_pattern_test(struct e1000_adapter *adapter, u64 *data, int reg, u32 mask, u32 write) {
	struct e1000_hw *hw = &adapter->hw;
	static const u32 test[] =
	{ 0x5A5A5A5A, 0xA5A5A5A5, 0x00000000, 0xFFFFFFFF };
	u8 __iomem
	*address = hw->hw_addr + reg;
	u32 read;
	int i;

	for (i = 0; i < ARRAY_SIZE(test); i++) {
		writel(write & test[i], address);
		read = readl(address);
		if (read != (write & test[i] & mask)) {
			e_err(drv, "pattern test reg %04X failed: "
				"got 0x%08X expected 0x%08X\n", reg, read, (write & test[i] & mask));
			*data = reg;
			return true;
		}
	}
	return false;
}

static bool reg_set_and_check(struct e1000_adapter *adapter, u64 *data, int reg, u32 mask, u32 write) {
	struct e1000_hw *hw = &adapter->hw;
	u8 __iomem
	*address = hw->hw_addr + reg;
	u32 read;

	writel(write & mask, address);
	read = readl(address);
	if ((read & mask) != (write & mask)) {
		e_err(drv, "set/check reg %04X test failed: "
			"got 0x%08X expected 0x%08X\n", reg, (read & mask), (write & mask));
		*data = reg;
		return true;
	}
	return false;
}

static int e1000_setup_desc_rings(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_tx_ring *txdr = &adapter->test_tx_ring;
	struct e1000_rx_ring *rxdr = &adapter->test_rx_ring;
	struct pci_dev *pdev = adapter->pdev;
	u32 rctl;
	int i, ret_val;

	/* Setup Tx descriptor ring and Tx buffers */

	if (!txdr->count)
		txdr->count = E1000_DEFAULT_TXD;

	txdr->buffer_info = kcalloc(txdr->count, sizeof(struct e1000_tx_buffer), GFP_KERNEL);
	if (!txdr->buffer_info) {
		ret_val = 1;
		goto err_nomem;
	}

	txdr->size = txdr->count * sizeof(struct e1000_tx_desc);
	txdr->size = ALIGN(txdr->size, 4096);
	txdr->desc = dma_zalloc_coherent(&pdev->dev, txdr->size, &txdr->dma, GFP_KERNEL);
	if (!txdr->desc) {
		ret_val = 2;
		goto err_nomem;
	}
	txdr->next_to_use = txdr->next_to_clean = 0;

	ew32(TDBAL, ((u64 ) txdr->dma & 0x00000000FFFFFFFF));
	ew32(TDBAH, ((u64 ) txdr->dma >> 32));
	ew32(TDLEN, txdr->count * sizeof(struct e1000_tx_desc));
	ew32(TDH, 0);
	ew32(TDT, 0);
	ew32(TCTL, E1000_TCTL_PSP | E1000_TCTL_EN | E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT | E1000_FDX_COLLISION_DISTANCE << E1000_COLD_SHIFT);

	for (i = 0; i < txdr->count; i++) {
		struct e1000_tx_desc *tx_desc = E1000_TX_DESC(*txdr, i);
		struct sk_buff *skb;
		unsigned int size = 1024;

		skb = alloc_skb(size, GFP_KERNEL);
		if (!skb) {
			ret_val = 3;
			goto err_nomem;
		}
		skb_put(skb, size);
		txdr->buffer_info[i].skb = skb;
		txdr->buffer_info[i].length = skb->len;
		txdr->buffer_info[i].dma = dma_map_single(&pdev->dev, skb->data, skb->len, DMA_TO_DEVICE);
		if (dma_mapping_error(&pdev->dev, txdr->buffer_info[i].dma)) {
			ret_val = 4;
			goto err_nomem;
		}
		tx_desc->buffer_addr = cpu_to_le64(txdr->buffer_info[i].dma);
		tx_desc->lower.data = cpu_to_le32(skb->len);
		tx_desc->lower.data |= cpu_to_le32(E1000_TXD_CMD_EOP | E1000_TXD_CMD_IFCS | E1000_TXD_CMD_RPS);
		tx_desc->upper.data = 0;
	}

	/* Setup Rx descriptor ring and Rx buffers */

	if (!rxdr->count)
		rxdr->count = E1000_DEFAULT_RXD;

	rxdr->buffer_info = kcalloc(rxdr->count, sizeof(struct e1000_rx_buffer), GFP_KERNEL);
	if (!rxdr->buffer_info) {
		ret_val = 5;
		goto err_nomem;
	}

	rxdr->size = rxdr->count * sizeof(struct e1000_rx_desc);
	rxdr->desc = dma_zalloc_coherent(&pdev->dev, rxdr->size, &rxdr->dma, GFP_KERNEL);
	if (!rxdr->desc) {
		ret_val = 6;
		goto err_nomem;
	}
	rxdr->next_to_use = rxdr->next_to_clean = 0;

	rctl = er32(RCTL);
	ew32(RCTL, rctl & ~E1000_RCTL_EN);
	ew32(RDBAL, ((u64 ) rxdr->dma & 0xFFFFFFFF));
	ew32(RDBAH, ((u64 ) rxdr->dma >> 32));
	ew32(RDLEN, rxdr->size);
	ew32(RDH, 0);
	ew32(RDT, 0);
	rctl = E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_SZ_2048 | E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF | (hw->mc_filter_type << E1000_RCTL_MO_SHIFT);
	ew32(RCTL, rctl);

	for (i = 0; i < rxdr->count; i++) {
		struct e1000_rx_desc *rx_desc = E1000_RX_DESC(*rxdr, i);
		u8 *buf;

		buf = kzalloc(E1000_RXBUFFER_2048 + NET_SKB_PAD + NET_IP_ALIGN, GFP_KERNEL);
		if (!buf) {
			ret_val = 7;
			goto err_nomem;
		}
		rxdr->buffer_info[i].rxbuf.data = buf;

		rxdr->buffer_info[i].dma = dma_map_single(&pdev->dev, buf + NET_SKB_PAD + NET_IP_ALIGN, E1000_RXBUFFER_2048, DMA_FROM_DEVICE);
		if (dma_mapping_error(&pdev->dev, rxdr->buffer_info[i].dma)) {
			ret_val = 8;
			goto err_nomem;
		}
		rx_desc->buffer_addr = cpu_to_le64(rxdr->buffer_info[i].dma);
	}

	return 0;

	err_nomem: e1000_free_desc_rings(adapter);
	return ret_val;
}

static int e1000_setup_loopback_test(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl;

	if (hw->media_type == e1000_media_type_fiber || hw->media_type == e1000_media_type_internal_serdes) {
		switch (hw->mac_type) {
			case e1000_82545:
			case e1000_82546:
			case e1000_82545_rev_3:
			case e1000_82546_rev_3:
				return e1000_set_phy_loopback(adapter);
			default:
				rctl = er32(RCTL);
				rctl |= E1000_RCTL_LBM_TCVR;
				ew32(RCTL, rctl);
				return 0;
		}
	} else if (hw->media_type == e1000_media_type_copper) {
		return e1000_set_phy_loopback(adapter);
	}

	return 7;
}

static int e1000_run_loopback_test(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_tx_ring *txdr = &adapter->test_tx_ring;
	struct e1000_rx_ring *rxdr = &adapter->test_rx_ring;
	struct pci_dev *pdev = adapter->pdev;
	int i, j, k, l, lc, good_cnt, ret_val = 0;
	unsigned long time;

	ew32(RDT, rxdr->count - 1);

	/* Calculate the loop count based on the largest descriptor ring
	 * The idea is to wrap the largest ring a number of times using 64
	 * send/receive pairs during each loop
	 */

	if (rxdr->count <= txdr->count)
		lc = ((txdr->count / 64) * 2) + 1;
	else
		lc = ((rxdr->count / 64) * 2) + 1;

	k = l = 0;
	for (j = 0; j <= lc; j++) { /* loop count loop */
		for (i = 0; i < 64; i++) { /* send the packets */
			e1000_create_lbtest_frame(txdr->buffer_info[i].skb, 1024);
			dma_sync_single_for_device(&pdev->dev, txdr->buffer_info[k].dma, txdr->buffer_info[k].length, DMA_TO_DEVICE);
			if (unlikely(++k == txdr->count))
				k = 0;
		}
		ew32(TDT, k);
		E1000_WRITE_FLUSH();
		msleep(200);
		time = jiffies; /* set the start time for the receive */
		good_cnt = 0;
		do { /* receive the sent packets */
			dma_sync_single_for_cpu(&pdev->dev, rxdr->buffer_info[l].dma, E1000_RXBUFFER_2048, DMA_FROM_DEVICE);

			ret_val = e1000_check_lbtest_frame(rxdr->buffer_info[l].rxbuf.data + NET_SKB_PAD + NET_IP_ALIGN, 1024);
			if (!ret_val)
				good_cnt++;
			if (unlikely(++l == rxdr->count))
				l = 0;
			/* time + 20 msecs (200 msecs on 2.4) is more than
			 * enough time to complete the receives, if it's
			 * exceeded, break and error off
			 */
		} while (good_cnt < 64 && time_after(time + 20, jiffies));

		if (good_cnt != 64) {
			ret_val = 13; /* ret_val is the same as mis-compare */
			break;
		}
		if (time_after_eq(jiffies, time + 2)) {
			ret_val = 14; /* error code for time out error */
			break;
		}
	} /* end loop count loop */
	return ret_val;
}

static void e1000_loopback_cleanup(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u32 rctl;
	u16 phy_reg;

	rctl = er32(RCTL);
	rctl &= ~(E1000_RCTL_LBM_TCVR | E1000_RCTL_LBM_MAC);
	ew32(RCTL, rctl);

	switch (hw->mac_type) {
		case e1000_82545:
		case e1000_82546:
		case e1000_82545_rev_3:
		case e1000_82546_rev_3:
		default:
			hw->autoneg = true;
			e1000_read_phy_reg(hw, PHY_CTRL, &phy_reg);
			if (phy_reg & MII_CR_LOOPBACK) {
				phy_reg &= ~MII_CR_LOOPBACK;
				e1000_write_phy_reg(hw, PHY_CTRL, phy_reg);
				e1000_phy_reset(hw);
			}
			break;
	}
}

static void e1000_free_desc_rings(struct e1000_adapter *adapter) {
	struct e1000_tx_ring *txdr = &adapter->test_tx_ring;
	struct e1000_rx_ring *rxdr = &adapter->test_rx_ring;
	struct pci_dev *pdev = adapter->pdev;
	int i;

	if (txdr->desc && txdr->buffer_info) {
		for (i = 0; i < txdr->count; i++) {
			if (txdr->buffer_info[i].dma)
				dma_unmap_single(&pdev->dev, txdr->buffer_info[i].dma, txdr->buffer_info[i].length, DMA_TO_DEVICE);
			if (txdr->buffer_info[i].skb)
				dev_kfree_skb(txdr->buffer_info[i].skb);
		}
	}

	if (rxdr->desc && rxdr->buffer_info) {
		for (i = 0; i < rxdr->count; i++) {
			if (rxdr->buffer_info[i].dma)
				dma_unmap_single(&pdev->dev, rxdr->buffer_info[i].dma, E1000_RXBUFFER_2048, DMA_FROM_DEVICE);
			kfree(rxdr->buffer_info[i].rxbuf.data);
		}
	}

	if (txdr->desc) {
		dma_free_coherent(&pdev->dev, txdr->size, txdr->desc, txdr->dma);
		txdr->desc = NULL;
	}
	if (rxdr->desc) {
		dma_free_coherent(&pdev->dev, rxdr->size, rxdr->desc, rxdr->dma);
		rxdr->desc = NULL;
	}

	kfree(txdr->buffer_info);
	txdr->buffer_info = NULL;
	kfree(rxdr->buffer_info);
	rxdr->buffer_info = NULL;
}

static int e1000_set_phy_loopback(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u16 phy_reg = 0;
	u16 count = 0;

	switch (hw->mac_type) {
		case e1000_82543:
			if (hw->media_type == e1000_media_type_copper) {
				/* Attempt to setup Loopback mode on Non-integrated PHY.
				 * Some PHY registers get corrupted at random, so
				 * attempt this 10 times.
				 */
				while (e1000_nonintegrated_phy_loopback(adapter) && count++ < 10)
					;
				if (count < 11)
					return 0;
			}
			break;

		case e1000_82544:
		case e1000_82540:
		case e1000_82545:
		case e1000_82545_rev_3:
		case e1000_82546:
		case e1000_82546_rev_3:
		case e1000_82541:
		case e1000_82541_rev_2:
		case e1000_82547:
		case e1000_82547_rev_2:
			return e1000_integrated_phy_loopback(adapter);
		default:
			/* Default PHY loopback work is to read the MII
			 * control register and assert bit 14 (loopback mode).
			 */
			e1000_read_phy_reg(hw, PHY_CTRL, &phy_reg);
			phy_reg |= MII_CR_LOOPBACK;
			e1000_write_phy_reg(hw, PHY_CTRL, phy_reg);
			return 0;
	}

	return 8;
}

static void e1000_create_lbtest_frame(struct sk_buff *skb, unsigned int frame_size) {
	memset(skb->data, 0xFF, frame_size);
	frame_size &= ~1;
	memset(&skb->data[frame_size / 2], 0xAA, frame_size / 2 - 1);
	memset(&skb->data[frame_size / 2 + 10], 0xBE, 1);
	memset(&skb->data[frame_size / 2 + 12], 0xAF, 1);
}

static int e1000_check_lbtest_frame(const unsigned char *data, unsigned int frame_size) {
	frame_size &= ~1;
	if (*(data + 3) == 0xFF) {
		if ((*(data + frame_size / 2 + 10) == 0xBE) && (*(data + frame_size / 2 + 12) == 0xAF)) {
			return 0;
		}
	}
	return 13;
}

static int e1000_nonintegrated_phy_loopback(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl_reg;
	u16 phy_reg;

	/* Setup the Device Control Register for PHY loopback test. */

	ctrl_reg = er32(CTRL);
	ctrl_reg |= (E1000_CTRL_ILOS | /* Invert Loss-Of-Signal */
	E1000_CTRL_FRCSPD | /* Set the Force Speed Bit */
	E1000_CTRL_FRCDPX | /* Set the Force Duplex Bit */
	E1000_CTRL_SPD_1000 | /* Force Speed to 1000 */
	E1000_CTRL_FD); /* Force Duplex to FULL */

	ew32(CTRL, ctrl_reg);

	/* Read the PHY Specific Control Register (0x10) */
	e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, &phy_reg);

	/* Clear Auto-Crossover bits in PHY Specific Control Register
	 * (bits 6:5).
	 */
	phy_reg &= ~M88E1000_PSCR_AUTO_X_MODE;
	e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, phy_reg);

	/* Perform software reset on the PHY */
	e1000_phy_reset(hw);

	/* Have to setup TX_CLK and TX_CRS after software reset */
	e1000_phy_reset_clk_and_crs(adapter);

	e1000_write_phy_reg(hw, PHY_CTRL, 0x8100);

	/* Wait for reset to complete. */
	udelay(500);

	/* Have to setup TX_CLK and TX_CRS after software reset */
	e1000_phy_reset_clk_and_crs(adapter);

	/* Write out to PHY registers 29 and 30 to disable the Receiver. */
	e1000_phy_disable_receiver(adapter);

	/* Set the loopback bit in the PHY control register. */
	e1000_read_phy_reg(hw, PHY_CTRL, &phy_reg);
	phy_reg |= MII_CR_LOOPBACK;
	e1000_write_phy_reg(hw, PHY_CTRL, phy_reg);

	/* Setup TX_CLK and TX_CRS one more time. */
	e1000_phy_reset_clk_and_crs(adapter);

	/* Check Phy Configuration */
	e1000_read_phy_reg(hw, PHY_CTRL, &phy_reg);
	if (phy_reg != 0x4100)
		return 9;

	e1000_read_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL, &phy_reg);
	if (phy_reg != 0x0070)
		return 10;

	e1000_read_phy_reg(hw, 29, &phy_reg);
	if (phy_reg != 0x001A)
		return 11;

	return 0;
}

static int e1000_integrated_phy_loopback(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl_reg = 0;
	u32 stat_reg = 0;

	hw->autoneg = false;

	if (hw->phy_type == e1000_phy_m88) {
		/* Auto-MDI/MDIX Off */
		e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, 0x0808);
		/* reset to update Auto-MDI/MDIX */
		e1000_write_phy_reg(hw, PHY_CTRL, 0x9140);
		/* autoneg off */
		e1000_write_phy_reg(hw, PHY_CTRL, 0x8140);
	}

	ctrl_reg = er32(CTRL);

	/* force 1000, set loopback */
	e1000_write_phy_reg(hw, PHY_CTRL, 0x4140);

	/* Now set up the MAC to the same speed/duplex as the PHY. */
	ctrl_reg = er32(CTRL);
	ctrl_reg &= ~E1000_CTRL_SPD_SEL; /* Clear the speed sel bits */
	ctrl_reg |= (E1000_CTRL_FRCSPD | /* Set the Force Speed Bit */
	E1000_CTRL_FRCDPX | /* Set the Force Duplex Bit */
	E1000_CTRL_SPD_1000 | /* Force Speed to 1000 */
	E1000_CTRL_FD); /* Force Duplex to FULL */

	if (hw->media_type == e1000_media_type_copper && hw->phy_type == e1000_phy_m88)
		ctrl_reg |= E1000_CTRL_ILOS; /* Invert Loss of Signal */
	else {
		/* Set the ILOS bit on the fiber Nic is half
		 * duplex link is detected.
		 */
		stat_reg = er32(STATUS);
		if ((stat_reg & E1000_STATUS_FD) == 0)
			ctrl_reg |= (E1000_CTRL_ILOS | E1000_CTRL_SLU);
	}

	ew32(CTRL, ctrl_reg);

	/* Disable the receiver on the PHY so when a cable is plugged in, the
	 * PHY does not begin to autoneg when a cable is reconnected to the NIC.
	 */
	if (hw->phy_type == e1000_phy_m88)
		e1000_phy_disable_receiver(adapter);

	udelay(500);

	return 0;
}

static void e1000_phy_reset_clk_and_crs(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;
	u16 phy_reg;

	/* Because we reset the PHY above, we need to re-force TX_CLK in the
	 * Extended PHY Specific Control Register to 25MHz clock.  This
	 * value defaults back to a 2.5MHz clock when the PHY is reset.
	 */
	e1000_read_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL, &phy_reg);
	phy_reg |= M88E1000_EPSCR_TX_CLK_25;
	e1000_write_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL, phy_reg);

	/* In addition, because of the s/w reset above, we need to enable
	 * CRS on TX.  This must be set for both full and half duplex
	 * operation.
	 */
	e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, &phy_reg);
	phy_reg |= M88E1000_PSCR_ASSERT_CRS_ON_TX;
	e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, phy_reg);
}

static void e1000_phy_disable_receiver(struct e1000_adapter *adapter) {
	struct e1000_hw *hw = &adapter->hw;

	/* Write out to PHY registers 29 and 30 to disable the Receiver. */
	e1000_write_phy_reg(hw, 29, 0x001F);
	e1000_write_phy_reg(hw, 30, 0x8FFC);
	e1000_write_phy_reg(hw, 29, 0x001A);
	e1000_write_phy_reg(hw, 30, 0x8FF0);
}
