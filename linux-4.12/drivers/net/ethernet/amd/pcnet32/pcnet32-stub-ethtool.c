/*
 * pcnet32-stub-ethtool.c
 *
 *  Created on: 2017. 7. 30.
 *      Author: root
 */
#include "pcnet32.h"

static int pcnet32_loopback_test(struct net_device *dev, uint64_t * data1);
static void pcnet32_netif_stop(struct net_device *dev);
static void pcnet32_netif_start(struct net_device *dev);
static void pcnet32_realloc_tx_ring(struct net_device *dev, struct pcnet32_private *lp, unsigned int size);
static void pcnet32_realloc_rx_ring(struct net_device *dev, struct pcnet32_private *lp, unsigned int size);

static const char pcnet32_gstrings_test[][ETH_GSTRING_LEN] = {
	"Loopback test  (offline)" };

static void pcnet32_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info) {
	struct pcnet32_private *lp = netdev_priv(dev);

	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	if (lp->pci_dev)
		strlcpy(info->bus_info, pci_name(lp->pci_dev), sizeof(info->bus_info));
	else
		snprintf(info->bus_info, sizeof(info->bus_info), "VLB 0x%lx", dev->base_addr);
}

static u32 pcnet32_get_msglevel(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	return lp->msg_enable;
}

static void pcnet32_set_msglevel(struct net_device *dev, u32 value) {
	struct pcnet32_private *lp = netdev_priv(dev);
	lp->msg_enable = value;
}

static int pcnet32_nway_reset(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long flags;
	int r = -EOPNOTSUPP;

	if (lp->mii) {
		spin_lock_irqsave(&lp->lock, flags);
		r = mii_nway_restart(&lp->mii_if);
		spin_unlock_irqrestore(&lp->lock, flags);
	}
	return r;
}

static u32 pcnet32_get_link(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long flags;
	int r;

	spin_lock_irqsave(&lp->lock, flags);
	if (lp->mii) {
		r = mii_link_ok(&lp->mii_if);
	} else if (lp->chip_version == PCNET32_79C970A) {
		ulong ioaddr = dev->base_addr; /* card base I/O address */
		/* only read link if port is set to TP */
		if (!lp->autoneg && lp->port_tp)
			r = (lp->a->read_bcr(ioaddr, 4) != 0xc0);
		else
			/* link always up for AUI port or port auto select */
			r = 1;
	} else if (lp->chip_version > PCNET32_79C970A) {
		ulong ioaddr = dev->base_addr; /* card base I/O address */
		r = (lp->a->read_bcr(ioaddr, 4) != 0xc0);
	} else { /* can not detect link on really old chips */
		r = 1;
	}
	spin_unlock_irqrestore(&lp->lock, flags);

	return r;
}

static void pcnet32_get_ringparam(struct net_device *dev, struct ethtool_ringparam *ering) {
	struct pcnet32_private *lp = netdev_priv(dev);

	ering->tx_max_pending = TX_MAX_RING_SIZE;
	ering->tx_pending = lp->tx_ring_size;
	ering->rx_max_pending = RX_MAX_RING_SIZE;
	ering->rx_pending = lp->rx_ring_size;
}

static int pcnet32_set_ringparam(struct net_device *dev, struct ethtool_ringparam *ering) {
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long flags;
	unsigned int size;
	ulong ioaddr = dev->base_addr;
	int i;

	if (ering->rx_mini_pending || ering->rx_jumbo_pending)
		return -EINVAL;

	if (netif_running(dev))
		pcnet32_netif_stop(dev);

	spin_lock_irqsave(&lp->lock, flags);
	lp->a->write_csr(ioaddr, CSR0, CSR0_STOP); /* stop the chip */

	size = min(ering->tx_pending, (unsigned int) TX_MAX_RING_SIZE);

	/* set the minimum ring size to 4, to allow the loopback test to work
	 * unchanged.
	 */
	for (i = 2; i <= PCNET32_LOG_MAX_TX_BUFFERS; i++) {
		if (size <= (1 << i))
			break;
	}
	if ((1 << i) != lp->tx_ring_size)
		pcnet32_realloc_tx_ring(dev, lp, i);

	size = min(ering->rx_pending, (unsigned int) RX_MAX_RING_SIZE);
	for (i = 2; i <= PCNET32_LOG_MAX_RX_BUFFERS; i++) {
		if (size <= (1 << i))
			break;
	}
	if ((1 << i) != lp->rx_ring_size)
		pcnet32_realloc_rx_ring(dev, lp, i);

	lp->napi.weight = lp->rx_ring_size / 2;

	if (netif_running(dev)) {
		pcnet32_netif_start(dev);
		pcnet32_restart(dev, CSR0_NORMAL);
	}

	spin_unlock_irqrestore(&lp->lock, flags);

	netif_info(lp, drv, dev, "Ring Param Settings: RX: %d, TX: %d\n", lp->rx_ring_size, lp->tx_ring_size);

	return 0;
}

static void pcnet32_get_strings(struct net_device *dev, u32 stringset, u8 *data) {
	memcpy(data, pcnet32_gstrings_test, sizeof(pcnet32_gstrings_test));
}

void pcnet32_ethtool_test(struct net_device *dev, struct ethtool_test *test, u64 * data) {
	struct pcnet32_private *lp = netdev_priv(dev);
	int rc;

	if (test->flags == ETH_TEST_FL_OFFLINE) {
		rc = pcnet32_loopback_test(dev, data);
		if (rc) {
			netif_printk(lp, hw, KERN_DEBUG, dev, "Loopback test failed\n");
			test->flags |= ETH_TEST_FL_FAILED;
		} else
			netif_printk(lp, hw, KERN_DEBUG, dev, "Loopback test passed\n");
	} else
		netif_printk(lp, hw, KERN_DEBUG, dev, "No tests to run (specify 'Offline' on ethtool)\n");
} /* end pcnet32_ethtool_test */

static int pcnet32_set_phys_id(struct net_device *dev, enum ethtool_phys_id_state state) {
	struct pcnet32_private *lp = netdev_priv(dev);
	const struct pcnet32_access *a = lp->a;
	ulong ioaddr = dev->base_addr;
	unsigned long flags;
	int i;

	switch (state) {
	case ETHTOOL_ID_ACTIVE:
		/* Save the current value of the bcrs */
		spin_lock_irqsave(&lp->lock, flags);
		for (i = 4; i < 8; i++)
			lp->save_regs[i - 4] = a->read_bcr(ioaddr, i);
		spin_unlock_irqrestore(&lp->lock, flags);
		return 2; /* cycle on/off twice per second */

	case ETHTOOL_ID_ON:
	case ETHTOOL_ID_OFF:
		/* Blink the led */
		spin_lock_irqsave(&lp->lock, flags);
		for (i = 4; i < 8; i++)
			a->write_bcr(ioaddr, i, a->read_bcr(ioaddr, i) ^ 0x4000);
		spin_unlock_irqrestore(&lp->lock, flags);
		break;

	case ETHTOOL_ID_INACTIVE:
		/* Restore the original value of the bcrs */
		spin_lock_irqsave(&lp->lock, flags);
		for (i = 4; i < 8; i++)
			a->write_bcr(ioaddr, i, lp->save_regs[i - 4]);
		spin_unlock_irqrestore(&lp->lock, flags);
	}
	return 0;
}

static int pcnet32_get_regs_len(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	int j = lp->phycount * PCNET32_REGS_PER_PHY;

	return (PCNET32_NUM_REGS + j) * sizeof(u16);
}

static void pcnet32_get_regs(struct net_device *dev, struct ethtool_regs *regs, void *ptr) {
	int i, csr0;
	u16 *buff = ptr;
	struct pcnet32_private *lp = netdev_priv(dev);
	const struct pcnet32_access *a = lp->a;
	ulong ioaddr = dev->base_addr;
	unsigned long flags;

	spin_lock_irqsave(&lp->lock, flags);

	csr0 = a->read_csr(ioaddr, CSR0);
	if (!(csr0 & CSR0_STOP)) /* If not stopped */
		pcnet32_suspend(dev, &flags, 1);

	/* read address PROM */
	for (i = 0; i < 16; i += 2)
		*buff++ = inw(ioaddr + i);

	/* read control and status registers */
	for (i = 0; i < 90; i++)
		*buff++ = a->read_csr(ioaddr, i);

	*buff++ = a->read_csr(ioaddr, 112);
	*buff++ = a->read_csr(ioaddr, 114);

	/* read bus configuration registers */
	for (i = 0; i < 30; i++)
		*buff++ = a->read_bcr(ioaddr, i);

	*buff++ = 0; /* skip bcr30 so as not to hang 79C976 */

	for (i = 31; i < 36; i++)
		*buff++ = a->read_bcr(ioaddr, i);

	/* read mii phy registers */
	if (lp->mii) {
		int j;
		for (j = 0; j < PCNET32_MAX_PHYS; j++) {
			if (lp->phymask & (1 << j)) {
				for (i = 0; i < PCNET32_REGS_PER_PHY; i++) {
					lp->a->write_bcr(ioaddr, 33, (j << 5) | i);
					*buff++ = lp->a->read_bcr(ioaddr, 34);
				}
			}
		}
	}

	if (!(csr0 & CSR0_STOP)) /* If not stopped */
		pcnet32_clr_suspend(lp, ioaddr);

	spin_unlock_irqrestore(&lp->lock, flags);
}

static int pcnet32_get_sset_count(struct net_device *dev, int sset) {
	switch (sset) {
	case ETH_SS_TEST:
		return PCNET32_TEST_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static int pcnet32_get_link_ksettings(struct net_device *dev, struct ethtool_link_ksettings *cmd) {
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long flags;
	int r = -EOPNOTSUPP;

	spin_lock_irqsave(&lp->lock, flags);
	if (lp->mii) {
		mii_ethtool_get_link_ksettings(&lp->mii_if, cmd);
		r = 0;
	} else if (lp->chip_version == PCNET32_79C970A) {
		if (lp->autoneg) {
			cmd->base.autoneg = AUTONEG_ENABLE;
			if (lp->a->read_bcr(dev->base_addr, 4) == 0xc0)
				cmd->base.port = PORT_AUI;
			else
				cmd->base.port = PORT_TP;
		} else {
			cmd->base.autoneg = AUTONEG_DISABLE;
			cmd->base.port = lp->port_tp ? PORT_TP : PORT_AUI;
		}
		cmd->base.duplex = lp->fdx ? DUPLEX_FULL : DUPLEX_HALF;
		cmd->base.speed = SPEED_10;
		ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
		SUPPORTED_TP | SUPPORTED_AUI);
		r = 0;
	}
	spin_unlock_irqrestore(&lp->lock, flags);
	return r;
}

static int pcnet32_set_link_ksettings(struct net_device *dev, const struct ethtool_link_ksettings *cmd) {
	struct pcnet32_private *lp = netdev_priv(dev);
	ulong ioaddr = dev->base_addr;
	unsigned long flags;
	int r = -EOPNOTSUPP;
	int suspended, bcr2, bcr9, csr15;

	spin_lock_irqsave(&lp->lock, flags);
	if (lp->mii) {
		r = mii_ethtool_set_link_ksettings(&lp->mii_if, cmd);
	} else if (lp->chip_version == PCNET32_79C970A) {
		suspended = pcnet32_suspend(dev, &flags, 0);
		if (!suspended)
			lp->a->write_csr(ioaddr, CSR0, CSR0_STOP);

		lp->autoneg = cmd->base.autoneg == AUTONEG_ENABLE;
		bcr2 = lp->a->read_bcr(ioaddr, 2);
		if (cmd->base.autoneg == AUTONEG_ENABLE) {
			lp->a->write_bcr(ioaddr, 2, bcr2 | 0x0002);
		} else {
			lp->a->write_bcr(ioaddr, 2, bcr2 & ~0x0002);

			lp->port_tp = cmd->base.port == PORT_TP;
			csr15 = lp->a->read_csr(ioaddr, CSR15) & ~0x0180;
			if (cmd->base.port == PORT_TP)
				csr15 |= 0x0080;
			lp->a->write_csr(ioaddr, CSR15, csr15);
			lp->init_block->mode = cpu_to_le16(csr15);

			lp->fdx = cmd->base.duplex == DUPLEX_FULL;
			bcr9 = lp->a->read_bcr(ioaddr, 9) & ~0x0003;
			if (cmd->base.duplex == DUPLEX_FULL)
				bcr9 |= 0x0003;
			lp->a->write_bcr(ioaddr, 9, bcr9);
		}
		if (suspended)
			pcnet32_clr_suspend(lp, ioaddr);
		else if (netif_running(dev))
			pcnet32_restart(dev, CSR0_NORMAL);
		r = 0;
	}
	spin_unlock_irqrestore(&lp->lock, flags);
	return r;
}

const struct ethtool_ops pcnet32_ethtool_ops = { //
			.get_drvinfo = pcnet32_get_drvinfo, //
			.get_msglevel = pcnet32_get_msglevel,
			.set_msglevel = pcnet32_set_msglevel,
			.nway_reset = pcnet32_nway_reset,
			.get_link = pcnet32_get_link,
			.get_ringparam = pcnet32_get_ringparam,
			.set_ringparam = pcnet32_set_ringparam,
			.get_strings = pcnet32_get_strings,
			.self_test = pcnet32_ethtool_test,
			.set_phys_id = pcnet32_set_phys_id,
			.get_regs_len = pcnet32_get_regs_len,
			.get_regs = pcnet32_get_regs,
			.get_sset_count = pcnet32_get_sset_count,
			.get_link_ksettings = pcnet32_get_link_ksettings,
			.set_link_ksettings = pcnet32_set_link_ksettings, };

/**********************
 * Extra Function
 **********************/

static int pcnet32_loopback_test(struct net_device *dev, uint64_t * data1) {
	struct pcnet32_private *lp = netdev_priv(dev);
	const struct pcnet32_access *a = lp->a; /* access to registers */
	ulong ioaddr = dev->base_addr; /* card base I/O address */
	struct sk_buff *skb; /* sk buff */
	int x, i; /* counters */
	int numbuffs = 4; /* number of TX/RX buffers and descs */
	u16 status = 0x8300; /* TX ring status */
	__le16 teststatus; /* test of ring status */
	int rc; /* return code */
	int size; /* size of packets */
	unsigned char *packet; /* source packet data */
	static const int data_len = 60; /* length of source packets */
	unsigned long flags;
	unsigned long ticks;

	rc = 1; /* default to fail */

	if (netif_running(dev))
		pcnet32_netif_stop(dev);

	spin_lock_irqsave(&lp->lock, flags);
	lp->a->write_csr(ioaddr, CSR0, CSR0_STOP); /* stop the chip */

	numbuffs = min(numbuffs, (int) min(lp->rx_ring_size, lp->tx_ring_size));

	/* Reset the PCNET32 */
	lp->a->reset(ioaddr);
	lp->a->write_csr(ioaddr, CSR4, 0x0915); /* auto tx pad */

	/* switch pcnet32 to 32bit mode */
	lp->a->write_bcr(ioaddr, 20, 2);

	/* purge & init rings but don't actually restart */
	pcnet32_restart(dev, 0x0000);

	lp->a->write_csr(ioaddr, CSR0, CSR0_STOP); /* Set STOP bit */

	/* Initialize Transmit buffers. */
	size = data_len + 15;
	for (x = 0; x < numbuffs; x++) {
		skb = netdev_alloc_skb(dev, size);
		if (!skb) {
			netif_printk(lp, hw, KERN_DEBUG, dev, "Cannot allocate skb at line: %d!\n",
			__LINE__);
			goto clean_up;
		}
		packet = skb->data;
		skb_put(skb, size); /* create space for data */
		lp->tx_skbuff[x] = skb;
		lp->tx_ring[x].length = cpu_to_le16(-skb->len);
		lp->tx_ring[x].misc = 0;

		/* put DA and SA into the skb */
		for (i = 0; i < 6; i++)
			*packet++ = dev->dev_addr[i];
		for (i = 0; i < 6; i++)
			*packet++ = dev->dev_addr[i];
		/* type */
		*packet++ = 0x08;
		*packet++ = 0x06;
		/* packet number */
		*packet++ = x;
		/* fill packet with data */
		for (i = 0; i < data_len; i++)
			*packet++ = i;

		lp->tx_dma_addr[x] = pci_map_single(lp->pci_dev, skb->data, skb->len, PCI_DMA_TODEVICE);
		if (pci_dma_mapping_error(lp->pci_dev, lp->tx_dma_addr[x])) {
			netif_printk(lp, hw, KERN_DEBUG, dev, "DMA mapping error at line: %d!\n",
			__LINE__);
			goto clean_up;
		}
		lp->tx_ring[x].base = cpu_to_le32(lp->tx_dma_addr[x]);
		wmb(); /* Make sure owner changes after all others are visible */
		lp->tx_ring[x].status = cpu_to_le16(status);
	}

	x = a->read_bcr(ioaddr, 32); /* set internal loopback in BCR32 */
	a->write_bcr(ioaddr, 32, x | 0x0002);

	/* set int loopback in CSR15 */
	x = a->read_csr(ioaddr, CSR15) & 0xfffc;
	lp->a->write_csr(ioaddr, CSR15, x | 0x0044);

	teststatus = cpu_to_le16(0x8000);
	lp->a->write_csr(ioaddr, CSR0, CSR0_START); /* Set STRT bit */

	/* Check status of descriptors */
	for (x = 0; x < numbuffs; x++) {
		ticks = 0;
		rmb();
		while ((lp->rx_ring[x].status & teststatus) && (ticks < 200)) {
			spin_unlock_irqrestore(&lp->lock, flags);
			msleep(1);
			spin_lock_irqsave(&lp->lock, flags);
			rmb();
			ticks++;
		}
		if (ticks == 200) {
			netif_err(lp, hw, dev, "Desc %d failed to reset!\n", x);
			break;
		}
	}

	lp->a->write_csr(ioaddr, CSR0, CSR0_STOP); /* Set STOP bit */
	wmb();
	if (netif_msg_hw(lp) && netif_msg_pktdata(lp)) {
		netdev_printk(KERN_DEBUG, dev, "RX loopback packets:\n");

		for (x = 0; x < numbuffs; x++) {
			netdev_printk(KERN_DEBUG, dev, "Packet %d: ", x);
			skb = lp->rx_skbuff[x];
			for (i = 0; i < size; i++)
				pr_cont(" %02x", *(skb->data + i));
			pr_cont("\n");
		}
	}

	x = 0;
	rc = 0;
	while (x < numbuffs && !rc) {
		skb = lp->rx_skbuff[x];
		packet = lp->tx_skbuff[x]->data;
		for (i = 0; i < size; i++) {
			if (*(skb->data + i) != packet[i]) {
				netif_printk(lp, hw, KERN_DEBUG, dev, "Error in compare! %2x - %02x %02x\n", i, *(skb->data + i), packet[i]);
				rc = 1;
				break;
			}
		}
		x++;
	}

	clean_up: *data1 = rc;
	pcnet32_purge_tx_ring(dev);

	x = a->read_csr(ioaddr, CSR15);
	a->write_csr(ioaddr, CSR15, (x & ~0x0044)); /* reset bits 6 and 2 */

	x = a->read_bcr(ioaddr, 32); /* reset internal loopback */
	a->write_bcr(ioaddr, 32, (x & ~0x0002));

	if (netif_running(dev)) {
		pcnet32_netif_start(dev);
		pcnet32_restart(dev, CSR0_NORMAL);
	} else {
		pcnet32_purge_rx_ring(dev);
		lp->a->write_bcr(ioaddr, 20, 4); /* return to 16bit mode */
	}
	spin_unlock_irqrestore(&lp->lock, flags);

	return rc;
} /* end pcnet32_loopback_test  */

static void pcnet32_netif_stop(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);

	netif_trans_update(dev); /* prevent tx timeout */
	napi_disable(&lp->napi);
	netif_tx_disable(dev);
}

static void pcnet32_netif_start(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	ulong ioaddr = dev->base_addr;
	u16 val;

	netif_wake_queue(dev);
	val = lp->a->read_csr(ioaddr, CSR3);
	val &= 0x00ff;
	lp->a->write_csr(ioaddr, CSR3, val);
	napi_enable(&lp->napi);
}

/*
 * Allocate space for the new sized tx ring.
 * Free old resources
 * Save new resources.
 * Any failure keeps old resources.
 * Must be called with lp->lock held.
 */
void pcnet32_realloc_tx_ring(struct net_device *dev, struct pcnet32_private *lp, unsigned int size) {
	dma_addr_t new_ring_dma_addr;
	dma_addr_t *new_dma_addr_list;
	struct pcnet32_tx_head *new_tx_ring;
	struct sk_buff **new_skb_list;
	unsigned int entries = BIT(size);

	pcnet32_purge_tx_ring(dev);

	new_tx_ring = pci_zalloc_consistent(lp->pci_dev, sizeof(struct pcnet32_tx_head) * entries, &new_ring_dma_addr);
	if (new_tx_ring == NULL)
		return;

	new_dma_addr_list = kcalloc(entries, sizeof(dma_addr_t), GFP_ATOMIC);
	if (!new_dma_addr_list)
		goto free_new_tx_ring;

	new_skb_list = kcalloc(entries, sizeof(struct sk_buff *), GFP_ATOMIC);
	if (!new_skb_list)
		goto free_new_lists;

	kfree(lp->tx_skbuff);
	kfree(lp->tx_dma_addr);
	pci_free_consistent(lp->pci_dev, sizeof(struct pcnet32_tx_head) * lp->tx_ring_size, lp->tx_ring, lp->tx_ring_dma_addr);

	lp->tx_ring_size = entries;
	lp->tx_mod_mask = lp->tx_ring_size - 1;
	lp->tx_len_bits = (size << 12);
	lp->tx_ring = new_tx_ring;
	lp->tx_ring_dma_addr = new_ring_dma_addr;
	lp->tx_dma_addr = new_dma_addr_list;
	lp->tx_skbuff = new_skb_list;
	return;

	free_new_lists: kfree(new_dma_addr_list);
	free_new_tx_ring: pci_free_consistent(lp->pci_dev, sizeof(struct pcnet32_tx_head) * entries, new_tx_ring, new_ring_dma_addr);
}

/*
 * Allocate space for the new sized rx ring.
 * Re-use old receive buffers.
 *   alloc extra buffers
 *   free unneeded buffers
 *   free unneeded buffers
 * Save new resources.
 * Any failure keeps old resources.
 * Must be called with lp->lock held.
 */
void pcnet32_realloc_rx_ring(struct net_device *dev, struct pcnet32_private *lp, unsigned int size) {
	dma_addr_t new_ring_dma_addr;
	dma_addr_t *new_dma_addr_list;
	struct pcnet32_rx_head *new_rx_ring;
	struct sk_buff **new_skb_list;
	int new, overlap;
	unsigned int entries = BIT(size);

	new_rx_ring = pci_zalloc_consistent(lp->pci_dev, sizeof(struct pcnet32_rx_head) * entries, &new_ring_dma_addr);
	if (new_rx_ring == NULL)
		return;

	new_dma_addr_list = kcalloc(entries, sizeof(dma_addr_t), GFP_ATOMIC);
	if (!new_dma_addr_list)
		goto free_new_rx_ring;

	new_skb_list = kcalloc(entries, sizeof(struct sk_buff *), GFP_ATOMIC);
	if (!new_skb_list)
		goto free_new_lists;

	/* first copy the current receive buffers */
	overlap = min(entries, lp->rx_ring_size);
	for (new = 0; new < overlap; new++) {
		new_rx_ring[new] = lp->rx_ring[new];
		new_dma_addr_list[new] = lp->rx_dma_addr[new];
		new_skb_list[new] = lp->rx_skbuff[new];
	}
	/* now allocate any new buffers needed */
	for (; new < entries; new++) {
		struct sk_buff *rx_skbuff;
		new_skb_list[new] = netdev_alloc_skb(dev, PKT_BUF_SKB);
		rx_skbuff = new_skb_list[new];
		if (!rx_skbuff) {
			/* keep the original lists and buffers */
			netif_err(lp, drv, dev, "%s netdev_alloc_skb failed\n", __func__);
			goto free_all_new;
		}
		skb_reserve(rx_skbuff, NET_IP_ALIGN);

		new_dma_addr_list[new] = pci_map_single(lp->pci_dev, rx_skbuff->data,
		PKT_BUF_SIZE, PCI_DMA_FROMDEVICE);
		if (pci_dma_mapping_error(lp->pci_dev, new_dma_addr_list[new])) {
			netif_err(lp, drv, dev, "%s dma mapping failed\n", __func__);
			dev_kfree_skb(new_skb_list[new]);
			goto free_all_new;
		}
		new_rx_ring[new].base = cpu_to_le32(new_dma_addr_list[new]);
		new_rx_ring[new].buf_length = cpu_to_le16(NEG_BUF_SIZE);
		new_rx_ring[new].status = cpu_to_le16(0x8000);
	}
	/* and free any unneeded buffers */
	for (; new < lp->rx_ring_size; new++) {
		if (lp->rx_skbuff[new]) {
			if (!pci_dma_mapping_error(lp->pci_dev, lp->rx_dma_addr[new]))
				pci_unmap_single(lp->pci_dev, lp->rx_dma_addr[new],
				PKT_BUF_SIZE, PCI_DMA_FROMDEVICE);
			dev_kfree_skb(lp->rx_skbuff[new]);
		}
	}

	kfree(lp->rx_skbuff);
	kfree(lp->rx_dma_addr);
	pci_free_consistent(lp->pci_dev, sizeof(struct pcnet32_rx_head) * lp->rx_ring_size, lp->rx_ring, lp->rx_ring_dma_addr);

	lp->rx_ring_size = entries;
	lp->rx_mod_mask = lp->rx_ring_size - 1;
	lp->rx_len_bits = (size << 4);
	lp->rx_ring = new_rx_ring;
	lp->rx_ring_dma_addr = new_ring_dma_addr;
	lp->rx_dma_addr = new_dma_addr_list;
	lp->rx_skbuff = new_skb_list;
	return;

	free_all_new: while (--new >= lp->rx_ring_size) {
		if (new_skb_list[new]) {
			if (!pci_dma_mapping_error(lp->pci_dev, new_dma_addr_list[new]))
				pci_unmap_single(lp->pci_dev, new_dma_addr_list[new],
				PKT_BUF_SIZE, PCI_DMA_FROMDEVICE);
			dev_kfree_skb(new_skb_list[new]);
		}
	}
	kfree(new_skb_list);
	free_new_lists: kfree(new_dma_addr_list);
	free_new_rx_ring: pci_free_consistent(lp->pci_dev, sizeof(struct pcnet32_rx_head) * entries, new_rx_ring, new_ring_dma_addr);
}

