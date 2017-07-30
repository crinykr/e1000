/*
 * PCnet-PCI II (Am79C970A)
 *
 * lspci -v
 *
 * 00:03.0 Ethernet controller: Advanced Micro Devices, Inc. [AMD] 79c970 [PCnet32 LANCE] (rev 10)
 * Subsystem: Advanced Micro Devices, Inc. [AMD] PCnet - Fast 79C971
 * Flags: bus master, medium devsel, latency 64, IRQ 19
 * I/O ports at d020 [size=32]
 * Memory at f0000000 (32-bit, non-prefetchable) [size=4K]
 * Kernel driver in use: pcnet32
 * Kernel modules: pcnet32
 */

#include "pcnet32.h"

MODULE_AUTHOR("Thomas Bogendoerfer");
MODULE_DESCRIPTION("Driver for PCnet32 and PCnetPCI based ethercards");
MODULE_LICENSE("GPL");

int cards_found;
int pcnet32_debug;

static const struct pci_device_id pcnet32_pci_tbl[] =
{ // PCI device identifiers for "new style" Linux PCI Device Drivers
	{ PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LANCE_HOME), }, //
	{ PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LANCE), },
	{ PCI_DEVICE(PCI_VENDOR_ID_TRIDENT, PCI_DEVICE_ID_AMD_LANCE), .class = (PCI_CLASS_NETWORK_ETHERNET << 8), .class_mask = 0xffff00, }, /* Adapters that were sold with IBM's RS/6000 or pSeries hardware have the incorrect vendor id. */
	{ /* terminate list */} };

MODULE_DEVICE_TABLE( pci, pcnet32_pci_tbl);

static struct pci_driver pcnet32_driver =
{ //
	.name = DRV_NAME, //
	.id_table = pcnet32_pci_tbl,
	.probe = pcnet32_probe_pci,
	.remove = pcnet32_remove_one,
	.suspend = pcnet32_pm_suspend,
	.resume = pcnet32_pm_resume, };

static int __init pcnet32_init_module(void)
{
	printk("### pcnet32_init_module(%s)\n", __TIME__);

	pcnet32_debug = netif_msg_init(-1, PCNET32_MSG_DEFAULT); // debug level : -1

	pci_register_driver(&pcnet32_driver);

	return cards_found ? 0 : -ENODEV;
}

module_init( pcnet32_init_module);

struct net_device *pcnet32_dev;

static const char pcnet32_gstrings_test[][ETH_GSTRING_LEN] =
{ "Loopback test  (offline)" };

int options[MAX_UNITS];
int full_duplex[MAX_UNITS];
int homepna[MAX_UNITS];

/*
 * Allocate space for the new sized tx ring.
 * Free old resources
 * Save new resources.
 * Any failure keeps old resources.
 * Must be called with lp->lock held.
 */
static void pcnet32_realloc_tx_ring(struct net_device *dev, struct pcnet32_private *lp, unsigned int size) {
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
static void pcnet32_realloc_rx_ring(struct net_device *dev, struct pcnet32_private *lp, unsigned int size) {
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

#ifdef CONFIG_NET_POLL_CONTROLLER
static void pcnet32_poll_controller(struct net_device *dev) {
	disable_irq(dev->irq);
	pcnet32_interrupt(0, dev);
	enable_irq(dev->irq);
}
#endif

/*
 * lp->lock must be held.
 */
static int pcnet32_suspend(struct net_device *dev, unsigned long *flags, int can_sleep) {
	int csr5;
	struct pcnet32_private *lp = netdev_priv(dev);
	const struct pcnet32_access *a = lp->a;
	ulong ioaddr = dev->base_addr;
	int ticks;

	/* really old chips have to be stopped. */
	if (lp->chip_version < PCNET32_79C970A)
		return 0;

	/* set SUSPEND (SPND) - CSR5 bit 0 */
	csr5 = a->read_csr(ioaddr, CSR5);
	a->write_csr(ioaddr, CSR5, csr5 | CSR5_SUSPEND);

	/* poll waiting for bit to be set */
	ticks = 0;
	while (!(a->read_csr(ioaddr, CSR5) & CSR5_SUSPEND)) {
		spin_unlock_irqrestore(&lp->lock, *flags);
		if (can_sleep)
			msleep(1);
		else
			mdelay(1);
		spin_lock_irqsave(&lp->lock, *flags);
		ticks++;
		if (ticks > 200) {
			netif_printk(lp, hw, KERN_DEBUG, dev, "Error getting into suspend!\n");
			return 0;
		}
	}
	return 1;
}

static void pcnet32_clr_suspend(struct pcnet32_private *lp, ulong ioaddr) {
	int csr5 = lp->a->read_csr(ioaddr, CSR5);
	/* clear SUSPEND (SPND) - CSR5 bit 0 */
	lp->a->write_csr(ioaddr, CSR5, csr5 & ~CSR5_SUSPEND);
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

static void pcnet32_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info) {
	struct pcnet32_private *lp = netdev_priv(dev);

	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	if (lp->pci_dev)
		strlcpy(info->bus_info, pci_name(lp->pci_dev), sizeof(info->bus_info));
	else
		snprintf(info->bus_info, sizeof(info->bus_info), "VLB 0x%lx", dev->base_addr);
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

static int pcnet32_get_sset_count(struct net_device *dev, int sset) {
	switch (sset) {
		case ETH_SS_TEST:
			return PCNET32_TEST_LEN;
		default:
			return -EOPNOTSUPP;
	}
}

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

const struct ethtool_ops pcnet32_ethtool_ops =
{ //
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

const struct net_device_ops pcnet32_netdev_ops =
{ //
	.ndo_open = pcnet32_open, //
	.ndo_stop = pcnet32_close,
	.ndo_start_xmit = pcnet32_start_xmit,
	.ndo_tx_timeout = pcnet32_tx_timeout,
	.ndo_get_stats = pcnet32_get_stats,
	.ndo_set_rx_mode = pcnet32_set_multicast_list,
	.ndo_do_ioctl = pcnet32_ioctl,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = pcnet32_poll_controller,
#endif
	};

static int pcnet32_open(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	struct pci_dev *pdev = lp->pci_dev;
	unsigned long ioaddr = dev->base_addr;
	u16 val;
	int i;
	int rc;
	unsigned long flags;

	if (request_irq(dev->irq, pcnet32_interrupt, lp->shared_irq ? IRQF_SHARED : 0, dev->name, (void *) dev)) {
		return -EAGAIN;
	}

	spin_lock_irqsave(&lp->lock, flags);
	/* Check for a valid station address */
	if (!is_valid_ether_addr(dev->dev_addr)) {
		rc = -EINVAL;
		goto err_free_irq;
	}

	/* Reset the PCNET32 */
	lp->a->reset(ioaddr);

	/* switch pcnet32 to 32bit mode */
	lp->a->write_bcr(ioaddr, 20, 2);

	netif_printk(lp, ifup, KERN_DEBUG, dev, "%s() irq %d tx/rx rings %#x/%#x init %#x\n", __func__, dev->irq, (u32)(lp->tx_ring_dma_addr), (u32)(lp->rx_ring_dma_addr), (u32)(lp->init_dma_addr));

	lp->autoneg = !!(lp->options & PCNET32_PORT_ASEL);
	lp->port_tp = !!(lp->options & PCNET32_PORT_10BT);
	lp->fdx = !!(lp->options & PCNET32_PORT_FD);

	/* set/reset autoselect bit */
	val = lp->a->read_bcr(ioaddr, 2) & ~2;
	if (lp->options & PCNET32_PORT_ASEL)
		val |= 2;
	lp->a->write_bcr(ioaddr, 2, val);

	/* handle full duplex setting */
	if (lp->mii_if.full_duplex) {
		val = lp->a->read_bcr(ioaddr, 9) & ~3;
		if (lp->options & PCNET32_PORT_FD) {
			val |= 1;
			if (lp->options == (PCNET32_PORT_FD | PCNET32_PORT_AUI))
				val |= 2;
		} else if (lp->options & PCNET32_PORT_ASEL) {
			/* workaround of xSeries250, turn on for 79C975 only */
			if (lp->chip_version == 0x2627)
				val |= 3;
		}
		lp->a->write_bcr(ioaddr, 9, val);
	}

	/* set/reset GPSI bit in test register */
	val = lp->a->read_csr(ioaddr, 124) & ~0x10;
	if ((lp->options & PCNET32_PORT_PORTSEL) == PCNET32_PORT_GPSI)
		val |= 0x10;
	lp->a->write_csr(ioaddr, 124, val);

	/* Allied Telesyn AT 2700/2701 FX are 100Mbit only and do not negotiate */
	if (pdev && pdev->subsystem_vendor == PCI_VENDOR_ID_AT && (pdev->subsystem_device == PCI_SUBDEVICE_ID_AT_2700FX || pdev->subsystem_device == PCI_SUBDEVICE_ID_AT_2701FX)) {
		if (lp->options & PCNET32_PORT_ASEL) {
			lp->options = PCNET32_PORT_FD | PCNET32_PORT_100;
			netif_printk(lp, link, KERN_DEBUG, dev, "Setting 100Mb-Full Duplex\n");
		}
	}
	if (lp->phycount < 2) {
		/*
		 * 24 Jun 2004 according AMD, in order to change the PHY,
		 * DANAS (or DISPM for 79C976) must be set; then select the speed,
		 * duplex, and/or enable auto negotiation, and clear DANAS
		 */
		if (lp->mii && !(lp->options & PCNET32_PORT_ASEL)) {
			lp->a->write_bcr(ioaddr, 32, lp->a->read_bcr(ioaddr, 32) | 0x0080);
			/* disable Auto Negotiation, set 10Mpbs, HD */
			val = lp->a->read_bcr(ioaddr, 32) & ~0xb8;
			if (lp->options & PCNET32_PORT_FD)
				val |= 0x10;
			if (lp->options & PCNET32_PORT_100)
				val |= 0x08;
			lp->a->write_bcr(ioaddr, 32, val);
		} else {
			if (lp->options & PCNET32_PORT_ASEL) {
				lp->a->write_bcr(ioaddr, 32, lp->a->read_bcr(ioaddr, 32) | 0x0080);
				/* enable auto negotiate, setup, disable fd */
				val = lp->a->read_bcr(ioaddr, 32) & ~0x98;
				val |= 0x20;
				lp->a->write_bcr(ioaddr, 32, val);
			}
		}
	} else {
		int first_phy = -1;
		u16 bmcr;
		u32 bcr9;
		struct ethtool_cmd ecmd =
		{ .cmd = ETHTOOL_GSET };

		/*
		 * There is really no good other way to handle multiple PHYs
		 * other than turning off all automatics
		 */
		val = lp->a->read_bcr(ioaddr, 2);
		lp->a->write_bcr(ioaddr, 2, val & ~2);
		val = lp->a->read_bcr(ioaddr, 32);
		lp->a->write_bcr(ioaddr, 32, val & ~(1 << 7)); /* stop MII manager */

		if (!(lp->options & PCNET32_PORT_ASEL)) {
			/* setup ecmd */
			ecmd.port = PORT_MII;
			ecmd.transceiver = XCVR_INTERNAL;
			ecmd.autoneg = AUTONEG_DISABLE;
			ethtool_cmd_speed_set(&ecmd, (lp->options & PCNET32_PORT_100) ?
			SPEED_100 :
																			SPEED_10);
			bcr9 = lp->a->read_bcr(ioaddr, 9);

			if (lp->options & PCNET32_PORT_FD) {
				ecmd.duplex = DUPLEX_FULL;
				bcr9 |= (1 << 0);
			} else {
				ecmd.duplex = DUPLEX_HALF;
				bcr9 |= ~(1 << 0);
			}
			lp->a->write_bcr(ioaddr, 9, bcr9);
		}

		for (i = 0; i < PCNET32_MAX_PHYS; i++) {
			if (lp->phymask & (1 << i)) {
				/* isolate all but the first PHY */
				bmcr = mdio_read(dev, i, MII_BMCR);
				if (first_phy == -1) {
					first_phy = i;
					mdio_write(dev, i, MII_BMCR, bmcr & ~BMCR_ISOLATE);
				} else {
					mdio_write(dev, i, MII_BMCR, bmcr | BMCR_ISOLATE);
				}
				/* use mii_ethtool_sset to setup PHY */
				lp->mii_if.phy_id = i;
				ecmd.phy_address = i;
				if (lp->options & PCNET32_PORT_ASEL) {
					mii_ethtool_gset(&lp->mii_if, &ecmd);
					ecmd.autoneg = AUTONEG_ENABLE;
				}
				mii_ethtool_sset(&lp->mii_if, &ecmd);
			}
		}
		lp->mii_if.phy_id = first_phy;
		netif_info(lp, link, dev, "Using PHY number %d\n", first_phy);
	}

#ifdef DO_DXSUFLO
	if (lp->dxsuflo) { /* Disable transmit stop on underflow */
		val = lp->a->read_csr(ioaddr, CSR3);
		val |= 0x40;
		lp->a->write_csr(ioaddr, CSR3, val);
	}
#endif

	lp->init_block->mode = cpu_to_le16((lp->options & PCNET32_PORT_PORTSEL) << 7);
	pcnet32_load_multicast(dev);

	if (pcnet32_init_ring(dev)) {
		rc = -ENOMEM;
		goto err_free_ring;
	}

	napi_enable(&lp->napi);

	/* Re-initialize the PCNET32, and start it when done. */
	lp->a->write_csr(ioaddr, 1, (lp->init_dma_addr & 0xffff));
	lp->a->write_csr(ioaddr, 2, (lp->init_dma_addr >> 16));

	lp->a->write_csr(ioaddr, CSR4, 0x0915); /* auto tx pad */
	lp->a->write_csr(ioaddr, CSR0, CSR0_INIT);

	netif_start_queue(dev);

	if (lp->chip_version >= PCNET32_79C970A) {
		/* Print the link status and start the watchdog */
		pcnet32_check_media(dev, 1);
		mod_timer(&lp->watchdog_timer, PCNET32_WATCHDOG_TIMEOUT);
	}

	i = 0;
	while (i++ < 100)
		if (lp->a->read_csr(ioaddr, CSR0) & CSR0_IDON)
			break;
	/*
	 * We used to clear the InitDone bit, 0x0100, here but Mark Stockton
	 * reports that doing so triggers a bug in the '974.
	 */
	lp->a->write_csr(ioaddr, CSR0, CSR0_NORMAL);

	netif_printk(lp, ifup, KERN_DEBUG, dev, "pcnet32 open after %d ticks, init block %#x csr0 %4.4x\n", i, (u32)(lp->init_dma_addr), lp->a->read_csr(ioaddr, CSR0));

	spin_unlock_irqrestore(&lp->lock, flags);

	return 0; /* Always succeed */

	err_free_ring:
	/* free any allocated skbuffs */
	pcnet32_purge_rx_ring(dev);

	/*
	 * Switch back to 16bit mode to avoid problems with dumb
	 * DOS packet driver after a warm reboot
	 */
	lp->a->write_bcr(ioaddr, 20, 4);

	err_free_irq: spin_unlock_irqrestore(&lp->lock, flags);
	free_irq(dev->irq, dev);
	return rc;
}

static void pcnet32_tx_timeout(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long ioaddr = dev->base_addr, flags;

	spin_lock_irqsave(&lp->lock, flags);
	/* Transmitter timeout, serious problems. */
	if (pcnet32_debug & NETIF_MSG_DRV)
		pr_err("%s: transmit timed out, status %4.4x, resetting\n", dev->name, lp->a->read_csr(ioaddr, CSR0));
	lp->a->write_csr(ioaddr, CSR0, CSR0_STOP);
	dev->stats.tx_errors++;
	if (netif_msg_tx_err(lp)) {
		int i;
		printk(KERN_DEBUG
			" Ring data dump: dirty_tx %d cur_tx %d%s cur_rx %d.",
			lp->dirty_tx, lp->cur_tx, lp->tx_full ? " (full)" : "",
			lp->cur_rx);
		for (i = 0; i < lp->rx_ring_size; i++)
			printk("%s %08x %04x %08x %04x", i & 1 ? "" : "\n ", le32_to_cpu(lp->rx_ring[i].base), (-le16_to_cpu(lp->rx_ring[i].buf_length)) & 0xffff, le32_to_cpu(lp->rx_ring[i].msg_length),
				le16_to_cpu(lp->rx_ring[i].status));
		for (i = 0; i < lp->tx_ring_size; i++)
			printk("%s %08x %04x %08x %04x", i & 1 ? "" : "\n ", le32_to_cpu(lp->tx_ring[i].base), (-le16_to_cpu(lp->tx_ring[i].length)) & 0xffff, le32_to_cpu(lp->tx_ring[i].misc),
				le16_to_cpu(lp->tx_ring[i].status));
		printk("\n");
	}
	pcnet32_restart(dev, CSR0_NORMAL);

	netif_trans_update(dev); /* prevent tx timeout */
	netif_wake_queue(dev);

	spin_unlock_irqrestore(&lp->lock, flags);
}

static netdev_tx_t pcnet32_start_xmit(struct sk_buff *skb, struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long ioaddr = dev->base_addr;
	u16 status;
	int entry;
	unsigned long flags;

	spin_lock_irqsave(&lp->lock, flags);

	netif_printk(lp, tx_queued, KERN_DEBUG, dev, "%s() called, csr0 %4.4x\n", __func__, lp->a->read_csr(ioaddr, CSR0));

	/* Default status -- will not enable Successful-TxDone
	 * interrupt when that option is available to us.
	 */
	status = 0x8300;

	/* Fill in a Tx ring entry */

	/* Mask to ring buffer boundary. */
	entry = lp->cur_tx & lp->tx_mod_mask;

	/* Caution: the write order is important here, set the status
	 * with the "ownership" bits last. */

	lp->tx_ring[entry].length = cpu_to_le16(-skb->len);

	lp->tx_ring[entry].misc = 0x00000000;

	lp->tx_dma_addr[entry] = pci_map_single(lp->pci_dev, skb->data, skb->len, PCI_DMA_TODEVICE);
	if (pci_dma_mapping_error(lp->pci_dev, lp->tx_dma_addr[entry])) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		goto drop_packet;
	}
	lp->tx_skbuff[entry] = skb;
	lp->tx_ring[entry].base = cpu_to_le32(lp->tx_dma_addr[entry]);
	wmb(); /* Make sure owner changes after all others are visible */
	lp->tx_ring[entry].status = cpu_to_le16(status);

	lp->cur_tx++;
	dev->stats.tx_bytes += skb->len;

	/* Trigger an immediate send poll. */
	lp->a->write_csr(ioaddr, CSR0, CSR0_INTEN | CSR0_TXPOLL);

	if (lp->tx_ring[(entry + 1) & lp->tx_mod_mask].base != 0) {
		lp->tx_full = 1;
		netif_stop_queue(dev);
	}
	drop_packet: spin_unlock_irqrestore(&lp->lock, flags);
	return NETDEV_TX_OK;
}

/* The PCNET32 interrupt handler. */
static irqreturn_t pcnet32_interrupt(int irq, void *dev_id) {
	struct net_device *dev = dev_id;
	struct pcnet32_private *lp;
	unsigned long ioaddr;
	u16 csr0;
	int boguscnt = MAX_INTERRUPT_WORK;

	ioaddr = dev->base_addr;
	lp = netdev_priv(dev);

	spin_lock(&lp->lock);

	csr0 = lp->a->read_csr(ioaddr, CSR0);
	while ((csr0 & 0x8f00) && --boguscnt >= 0) {
		if (csr0 == 0xffff)
			break; /* PCMCIA remove happened */
		/* Acknowledge all of the current interrupt sources ASAP. */
		lp->a->write_csr(ioaddr, CSR0, csr0 & ~0x004f);

		netif_printk(lp, intr, KERN_DEBUG, dev, "interrupt  csr0=%#2.2x new csr=%#2.2x\n", csr0, lp->a->read_csr(ioaddr, CSR0));

		/* Log misc errors. */
		if (csr0 & 0x4000)
			dev->stats.tx_errors++; /* Tx babble. */
		if (csr0 & 0x1000) {
			/*
			 * This happens when our receive ring is full. This
			 * shouldn't be a problem as we will see normal rx
			 * interrupts for the frames in the receive ring.  But
			 * there are some PCI chipsets (I can reproduce this
			 * on SP3G with Intel saturn chipset) which have
			 * sometimes problems and will fill up the receive
			 * ring with error descriptors.  In this situation we
			 * don't get a rx interrupt, but a missed frame
			 * interrupt sooner or later.
			 */
			dev->stats.rx_errors++; /* Missed a Rx frame. */
		}
		if (csr0 & 0x0800) {
			netif_err(lp, drv, dev, "Bus master arbitration failure, status %4.4x\n", csr0);
			/* unlike for the lance, there is no restart needed */
		}
		if (napi_schedule_prep(&lp->napi)) {
			u16 val;
			/* set interrupt masks */
			val = lp->a->read_csr(ioaddr, CSR3);
			val |= 0x5f00;
			lp->a->write_csr(ioaddr, CSR3, val);

			__napi_schedule(&lp->napi);
			break;
		}
		csr0 = lp->a->read_csr(ioaddr, CSR0);
	}

	netif_printk(lp, intr, KERN_DEBUG, dev, "exiting interrupt, csr0=%#4.4x\n", lp->a->read_csr(ioaddr, CSR0));

	spin_unlock(&lp->lock);

	return IRQ_HANDLED;
}

static int pcnet32_close(struct net_device *dev) {
	unsigned long ioaddr = dev->base_addr;
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long flags;

	del_timer_sync(&lp->watchdog_timer);

	netif_stop_queue(dev);
	napi_disable(&lp->napi);

	spin_lock_irqsave(&lp->lock, flags);

	dev->stats.rx_missed_errors = lp->a->read_csr(ioaddr, 112);

	netif_printk(lp, ifdown, KERN_DEBUG, dev, "Shutting down ethercard, status was %2.2x\n", lp->a->read_csr(ioaddr, CSR0));

	/* We stop the PCNET32 here -- it occasionally polls memory if we don't. */
	lp->a->write_csr(ioaddr, CSR0, CSR0_STOP);

	/*
	 * Switch back to 16bit mode to avoid problems with dumb
	 * DOS packet driver after a warm reboot
	 */
	lp->a->write_bcr(ioaddr, 20, 4);

	spin_unlock_irqrestore(&lp->lock, flags);

	free_irq(dev->irq, dev);

	spin_lock_irqsave(&lp->lock, flags);

	pcnet32_purge_rx_ring(dev);
	pcnet32_purge_tx_ring(dev);

	spin_unlock_irqrestore(&lp->lock, flags);

	return 0;
}

static struct net_device_stats *pcnet32_get_stats(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long ioaddr = dev->base_addr;
	unsigned long flags;

	spin_lock_irqsave(&lp->lock, flags);
	dev->stats.rx_missed_errors = lp->a->read_csr(ioaddr, 112);
	spin_unlock_irqrestore(&lp->lock, flags);

	return &dev->stats;
}

/* taken from the sunlance driver, which it took from the depca driver */
static void pcnet32_load_multicast(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	volatile struct pcnet32_init_block *ib = lp->init_block;
	volatile __le16 *mcast_table = (__le16 *) ib->filter;
	struct netdev_hw_addr *ha;
	unsigned long ioaddr = dev->base_addr;
	int i;
	u32 crc;

	/* set all multicast bits */
	if (dev->flags & IFF_ALLMULTI) {
		ib->filter[0] = cpu_to_le32(~0U);
		ib->filter[1] = cpu_to_le32(~0U);
		lp->a->write_csr(ioaddr, PCNET32_MC_FILTER, 0xffff);
		lp->a->write_csr(ioaddr, PCNET32_MC_FILTER + 1, 0xffff);
		lp->a->write_csr(ioaddr, PCNET32_MC_FILTER + 2, 0xffff);
		lp->a->write_csr(ioaddr, PCNET32_MC_FILTER + 3, 0xffff);
		return;
	}
	/* clear the multicast filter */
	ib->filter[0] = 0;
	ib->filter[1] = 0;

	/* Add addresses */
	netdev_for_each_mc_addr(ha, dev)
	{
		crc = ether_crc_le(6, ha->addr);
		crc = crc >> 26;
		mcast_table[crc >> 4] |= cpu_to_le16(1 << (crc & 0xf));
	}
	for (i = 0; i < 4; i++)
		lp->a->write_csr(ioaddr, PCNET32_MC_FILTER + i, le16_to_cpu(mcast_table[i]));
}

/*
 * Set or clear the multicast filter for this adaptor.
 */
static void pcnet32_set_multicast_list(struct net_device *dev) {
	unsigned long ioaddr = dev->base_addr, flags;
	struct pcnet32_private *lp = netdev_priv(dev);
	int csr15, suspended;

	spin_lock_irqsave(&lp->lock, flags);
	suspended = pcnet32_suspend(dev, &flags, 0);
	csr15 = lp->a->read_csr(ioaddr, CSR15);
	if (dev->flags & IFF_PROMISC) {
		/* Log any net taps. */
		netif_info(lp, hw, dev, "Promiscuous mode enabled\n");
		lp->init_block->mode = cpu_to_le16(0x8000 | (lp->options & PCNET32_PORT_PORTSEL) << 7);
		lp->a->write_csr(ioaddr, CSR15, csr15 | 0x8000);
	} else {
		lp->init_block->mode = cpu_to_le16((lp->options & PCNET32_PORT_PORTSEL) << 7);
		lp->a->write_csr(ioaddr, CSR15, csr15 & 0x7fff);
		pcnet32_load_multicast(dev);
	}

	if (suspended) {
		pcnet32_clr_suspend(lp, ioaddr);
	} else {
		lp->a->write_csr(ioaddr, CSR0, CSR0_STOP);
		pcnet32_restart(dev, CSR0_NORMAL);
		netif_wake_queue(dev);
	}

	spin_unlock_irqrestore(&lp->lock, flags);
}

static int pcnet32_ioctl(struct net_device *dev, struct ifreq *rq, int cmd) {
	struct pcnet32_private *lp = netdev_priv(dev);
	int rc;
	unsigned long flags;

	/* SIOC[GS]MIIxxx ioctls */
	if (lp->mii) {
		spin_lock_irqsave(&lp->lock, flags);
		rc = generic_mii_ioctl(&lp->mii_if, if_mii(rq), cmd, NULL);
		spin_unlock_irqrestore(&lp->lock, flags);
	} else {
		rc = -EOPNOTSUPP;
	}

	return rc;
}

static int pcnet32_pm_suspend(struct pci_dev *pdev, pm_message_t state) {
	struct net_device *dev = pci_get_drvdata(pdev);

	if (netif_running(dev)) {
		netif_device_detach(dev);
		pcnet32_close(dev);
	}
	pci_save_state(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));
	return 0;
}

static int pcnet32_pm_resume(struct pci_dev *pdev) {
	struct net_device *dev = pci_get_drvdata(pdev);

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	if (netif_running(dev)) {
		pcnet32_open(dev);
		netif_device_attach(dev);
	}
	return 0;
}

static void pcnet32_remove_one(struct pci_dev *pdev) {
	struct net_device *dev = pci_get_drvdata(pdev);

	if (dev) {
		struct pcnet32_private *lp = netdev_priv(dev);

		unregister_netdev(dev);
		pcnet32_free_ring(dev);
		release_region(dev->base_addr, PCNET32_TOTAL_SIZE);
		pci_free_consistent(lp->pci_dev, sizeof(*lp->init_block), lp->init_block, lp->init_dma_addr);
		free_netdev(dev);
		pci_disable_device(pdev);
	}
}

static void __exit pcnet32_cleanup_module(void)
{
	struct net_device *next_dev;

	while (pcnet32_dev) {
		struct pcnet32_private *lp = netdev_priv(pcnet32_dev);
		next_dev = lp->next;
		unregister_netdev(pcnet32_dev);
		pcnet32_free_ring(pcnet32_dev);
		release_region(pcnet32_dev->base_addr, PCNET32_TOTAL_SIZE);
		pci_free_consistent(lp->pci_dev, sizeof(*lp->init_block),
			lp->init_block, lp->init_dma_addr);
		free_netdev(pcnet32_dev);
		pcnet32_dev = next_dev;
	}

	pci_unregister_driver(&pcnet32_driver);
}

module_exit( pcnet32_cleanup_module);

/*
 * Local variables:
 *  c-indent-level: 4
 *  tab-width: 8
 * End:
 */
