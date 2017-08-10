#define __FILE__ "stub-pci-probe"
#include "pcnet32.h"

extern const struct ethtool_ops pcnet32_ethtool_ops;
extern const struct net_device_ops pcnet32_netdev_ops;

extern const struct pcnet32_access pcnet32_wio;
extern const struct pcnet32_access pcnet32_dwio;

extern int pcnet32_debug;

static int pcnet32_poll(struct napi_struct *napi, int budget);
static int pcnet32_rx(struct net_device *dev, int budget);
static int pcnet32_tx(struct net_device *dev);
static void pcnet32_rx_entry(struct net_device *dev, struct pcnet32_private *lp, struct pcnet32_rx_head *rxp, int entry);
static int pcnet32_alloc_ring(struct net_device *dev, const char *name);
static void pcnet32_watchdog(struct net_device *dev);

/* pcnet32_probe1
 *  Called from pcnet_probe_pci.
 */
int pcnet32_probe1(unsigned long ioaddr, int shared, struct pci_dev *pdev) {
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	struct pcnet32_private *lp;
	int i, media;
	int fdx, mii, fset, dxsuflo, sram;
	int chip_version;
	char *chipname;
	struct net_device *dev;
	const struct pcnet32_access *a = NULL;
	u8 promaddr[ETH_ALEN];
	int ret = -ENODEV;

	/* reset the chip */
	pcnet32_wio_reset(ioaddr);

	/* NOTE: 16-bit check is first, otherwise some older PCnet chips fail */
	if (pcnet32_wio_read_csr(ioaddr, 0) == 4 && pcnet32_wio_check(ioaddr))
		a = &pcnet32_wio;

	chip_version = a->read_csr(ioaddr, 88) | (a->read_csr(ioaddr, 89) << 16);
	chip_version = (chip_version >> 12) & 0xffff;

	/* initialize variables */
	fdx = mii = fset = dxsuflo = sram = 0;

	if (chip_version == 0x2621) {
		chipname = "PCnet/PCI II 79C970A"; /* PCI */
		fdx = 1;
	}

	dev = alloc_etherdev(sizeof(*lp));

	SET_NETDEV_DEV(dev, &pdev->dev);

	pr_info("%s at %#3lx,", chipname, ioaddr);

	/* In most chips, after a chip reset, the ethernet address is read from the
	 * station address PROM at the base address and programmed into the
	 * "Physical Address Registers" CSR12-14.
	 * As a precautionary measure, we read the PROM values and complain if
	 * they disagree with the CSRs.  If they miscompare, and the PROM addr
	 * is valid, then the PROM addr is used.
	 */
	for (i = 0; i < 3; i++) {
		unsigned int val;
		val = a->read_csr(ioaddr, i + 12) & 0x0ffff;
		/* There may be endianness issues here. */
		dev->dev_addr[2 * i] = val & 0x0ff;
		dev->dev_addr[2 * i + 1] = (val >> 8) & 0x0ff;
	}

	/* read PROM address and compare with CSR address */
	for (i = 0; i < ETH_ALEN; i++)
		promaddr[i] = inb(ioaddr + i);

	if (!ether_addr_equal(promaddr, dev->dev_addr) || !is_valid_ether_addr(dev->dev_addr)) {
		if (is_valid_ether_addr(promaddr)) {
			pr_cont(" warning: CSR address invalid,\n");
			pr_info("    using instead PROM address of");
			memcpy(dev->dev_addr, promaddr, ETH_ALEN);
		}
	}

	/* if the ethernet address is not valid, force to 00:00:00:00:00:00 */
	if (!is_valid_ether_addr(dev->dev_addr))
		eth_zero_addr(dev->dev_addr);

	pr_cont(" %pM", dev->dev_addr);

	dev->base_addr = ioaddr;
	lp = netdev_priv(dev);
	/* pci_alloc_consistent returns page-aligned memory, so we do not have to check the alignment */
	lp->init_block = pci_alloc_consistent(pdev, sizeof(*lp->init_block), &lp->init_dma_addr);
	lp->pci_dev = pdev;

	lp->dev = dev;

	spin_lock_init(&lp->lock);

	lp->name = chipname;
	lp->shared_irq = shared;
	lp->tx_ring_size = TX_RING_SIZE; /* default tx ring size */
	lp->rx_ring_size = RX_RING_SIZE; /* default rx ring size */
	lp->tx_mod_mask = lp->tx_ring_size - 1;
	lp->rx_mod_mask = lp->rx_ring_size - 1;
	lp->tx_len_bits = (PCNET32_LOG_TX_BUFFERS << 12);
	lp->rx_len_bits = (PCNET32_LOG_RX_BUFFERS << 4);
	lp->mii_if.full_duplex = fdx;
	lp->mii_if.phy_id_mask = 0x1f;
	lp->mii_if.reg_num_mask = 0x1f;
	lp->dxsuflo = dxsuflo;
	lp->mii = mii;
	lp->chip_version = chip_version;
	lp->msg_enable = pcnet32_debug;
	lp->options = PCNET32_PORT_10BT; /* force default port to TP on 79C970A so link detection can work */
	lp->mii_if.dev = dev;
	lp->mii_if.mdio_read = mdio_read;
	lp->mii_if.mdio_write = mdio_write;

	/* napi.weight is used in both the napi and non-napi cases */
	lp->napi.weight = lp->rx_ring_size / 2;

	netif_napi_add(dev, &lp->napi, pcnet32_poll, lp->rx_ring_size / 2);

	lp->a = a;

	/* prior to register_netdev, dev->name is not yet correct */
	pcnet32_alloc_ring(dev, pci_name(lp->pci_dev));

	/* detect special T1/E1 WAN card by checking for MAC address */
	if (dev->dev_addr[0] == 0x00 && dev->dev_addr[1] == 0xe0 && dev->dev_addr[2] == 0x75)
		lp->options = PCNET32_PORT_FD | PCNET32_PORT_GPSI;

	lp->init_block->mode = cpu_to_le16(0x0003); /* Disable Rx and Tx. */
	lp->init_block->tlen_rlen = cpu_to_le16(lp->tx_len_bits | lp->rx_len_bits);
	for (i = 0; i < 6; i++)
		lp->init_block->phys_addr[i] = dev->dev_addr[i];
	lp->init_block->filter[0] = 0x00000000;
	lp->init_block->filter[1] = 0x00000000;
	lp->init_block->rx_ring = cpu_to_le32(lp->rx_ring_dma_addr);
	lp->init_block->tx_ring = cpu_to_le32(lp->tx_ring_dma_addr);

	/* switch pcnet32 to 32bit mode */
	a->write_bcr(ioaddr, 20, 2);

	a->write_csr(ioaddr, 1, (lp->init_dma_addr & 0xffff));
	a->write_csr(ioaddr, 2, (lp->init_dma_addr >> 16));

	/* use the IRQ provided by PCI */
	dev->irq = pdev->irq;
	pr_cont(" assigned IRQ %d\n", dev->irq);

	init_timer(&lp->watchdog_timer);
	lp->watchdog_timer.data = (unsigned long) dev;
	lp->watchdog_timer.function = (void *) &pcnet32_watchdog;

	/* The PCNET32-specific entries in the device structure. */
	dev->netdev_ops = &pcnet32_netdev_ops;
	dev->ethtool_ops = &pcnet32_ethtool_ops;
	dev->watchdog_timeo = (5 * HZ);

	/* Fill in the generic fields of the device structure. */
	register_netdev(dev);

	pci_set_drvdata(pdev, dev);

	pr_info("%s: registered as %s\n", dev->name, lp->name);

	/* enable LED writes */
	a->write_bcr(ioaddr, 2, a->read_bcr(ioaddr, 2) | 0x1000);

	return 0;
}

static int pcnet32_poll(struct napi_struct *napi, int budget) {
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	struct pcnet32_private
	*lp = container_of(napi, struct pcnet32_private, napi);
	struct net_device *dev = lp->dev;
	unsigned long ioaddr = dev->base_addr;
	unsigned long flags;
	int work_done;
	u16 val;

	work_done = pcnet32_rx(dev, budget);

	spin_lock_irqsave(&lp->lock, flags);
	if (pcnet32_tx(dev)) {
		/* reset the chip to clear the error condition, then restart */
		lp->a->reset(ioaddr);
		lp->a->write_csr(ioaddr, CSR4, 0x0915); /* auto tx pad */
		pcnet32_restart(dev, CSR0_START);
		netif_wake_queue(dev);
	}

	if (work_done < budget && napi_complete_done(napi, work_done)) {
		/* clear interrupt masks */
		val = lp->a->read_csr(ioaddr, CSR3);
		val &= 0x00ff;
		lp->a->write_csr(ioaddr, CSR3, val);

		/* Set interrupt enable. */
		lp->a->write_csr(ioaddr, CSR0, CSR0_INTEN);
	}

	spin_unlock_irqrestore(&lp->lock, flags);
	return work_done;
}

static int pcnet32_rx(struct net_device *dev, int budget) {
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	struct pcnet32_private *lp = netdev_priv(dev);
	int entry = lp->cur_rx & lp->rx_mod_mask;
	struct pcnet32_rx_head *rxp = &lp->rx_ring[entry];
	int npackets = 0;

	/* If we own the next entry, it's a new packet. Send it up. */
	while (npackets < budget && (short) le16_to_cpu(rxp->status) >= 0) {
		pcnet32_rx_entry(dev, lp, rxp, entry);
		npackets += 1;
		/*
		 * The docs say that the buffer length isn't touched, but Andrew
		 * Boyd of QNX reports that some revs of the 79C965 clear it.
		 */
		rxp->buf_length = cpu_to_le16(NEG_BUF_SIZE);
		wmb(); /* Make sure owner changes after others are visible */
		rxp->status = cpu_to_le16(0x8000);
		entry = (++lp->cur_rx) & lp->rx_mod_mask;
		rxp = &lp->rx_ring[entry];
	}

	return npackets;
}

static int pcnet32_tx(struct net_device *dev) {
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned int dirty_tx = lp->dirty_tx;
	int delta;
	int must_restart = 0;

	while (dirty_tx != lp->cur_tx) {
		int entry = dirty_tx & lp->tx_mod_mask;
		int status = (short) le16_to_cpu(lp->tx_ring[entry].status);

		if (status < 0)
			break; /* It still hasn't been Txed */

		lp->tx_ring[entry].base = 0;

		if (status & 0x4000) {
			/* There was a major error, log it. */
			int err_status = le32_to_cpu(lp->tx_ring[entry].misc);
			dev->stats.tx_errors++;
			netif_err(lp, tx_err, dev, "Tx error status=%04x err_status=%08x\n", status, err_status);
			if (err_status & 0x04000000)
				dev->stats.tx_aborted_errors++;
			if (err_status & 0x08000000)
				dev->stats.tx_carrier_errors++;
			if (err_status & 0x10000000)
				dev->stats.tx_window_errors++;
#ifndef DO_DXSUFLO
			if (err_status & 0x40000000) {
				dev->stats.tx_fifo_errors++;
				/* Ackk!  On FIFO errors the Tx unit is turned off! */
				/* Remove this verbosity later! */
				netif_err(lp, tx_err, dev, "Tx FIFO error!\n");
				must_restart = 1;
			}
#else
			if (err_status & 0x40000000) {
				dev->stats.tx_fifo_errors++;
				if (!lp->dxsuflo) { /* If controller doesn't recover ... */
					/* Ackk!  On FIFO errors the Tx unit is turned off! */
					/* Remove this verbosity later! */
					netif_err(lp, tx_err, dev, "Tx FIFO error!\n");
					must_restart = 1;
				}
			}
#endif
		} else {
			if (status & 0x1800)
				dev->stats.collisions++;
			dev->stats.tx_packets++;
		}

		/* We must free the original skb */
		if (lp->tx_skbuff[entry]) {
			pci_unmap_single(lp->pci_dev, lp->tx_dma_addr[entry], lp->tx_skbuff[entry]->len, PCI_DMA_TODEVICE);
			dev_kfree_skb_any(lp->tx_skbuff[entry]);
			lp->tx_skbuff[entry] = NULL;
			lp->tx_dma_addr[entry] = 0;
		}
		dirty_tx++;
	}

	delta = (lp->cur_tx - dirty_tx) & (lp->tx_mod_mask + lp->tx_ring_size);
	if (delta > lp->tx_ring_size) {
		netif_err(lp, drv, dev, "out-of-sync dirty pointer, %d vs. %d, full=%d\n", dirty_tx, lp->cur_tx, lp->tx_full);
		dirty_tx += lp->tx_ring_size;
		delta -= lp->tx_ring_size;
	}

	if (lp->tx_full && netif_queue_stopped(dev) && delta < lp->tx_ring_size - 2) {
		/* The ring is no longer full, clear tbusy. */
		lp->tx_full = 0;
		netif_wake_queue(dev);
	}
	lp->dirty_tx = dirty_tx;

	return must_restart;
}

/*
 * process one receive descriptor entry
 */

static void pcnet32_rx_entry(struct net_device *dev, struct pcnet32_private *lp, struct pcnet32_rx_head *rxp, int entry) {
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	int status = (short) le16_to_cpu(rxp->status) >> 8;
	int rx_in_place = 0;
	struct sk_buff *skb;
	short pkt_len;

	if (status != 0x03) { /* There was an error. */
		/*
		 * There is a tricky error noted by John Murphy,
		 * <murf@perftech.com> to Russ Nelson: Even with full-sized
		 * buffers it's possible for a jabber packet to use two
		 * buffers, with only the last correctly noting the error.
		 */
		if (status & 0x01) /* Only count a general error at the */
			dev->stats.rx_errors++; /* end of a packet. */
		if (status & 0x20)
			dev->stats.rx_frame_errors++;
		if (status & 0x10)
			dev->stats.rx_over_errors++;
		if (status & 0x08)
			dev->stats.rx_crc_errors++;
		if (status & 0x04)
			dev->stats.rx_fifo_errors++;
		return;
	}

	pkt_len = (le32_to_cpu(rxp->msg_length) & 0xfff) - 4;

	/* Discard oversize frames. */
	if (unlikely(pkt_len > PKT_BUF_SIZE)) {
		netif_err(lp, drv, dev, "Impossible packet size %d!\n", pkt_len);
		dev->stats.rx_errors++;
		return;
	}
	if (pkt_len < 60) {
		netif_err(lp, rx_err, dev, "Runt packet!\n");
		dev->stats.rx_errors++;
		return;
	}

	if (pkt_len > RX_COPYBREAK) {
		struct sk_buff *newskb;
		dma_addr_t new_dma_addr;

		newskb = netdev_alloc_skb(dev, PKT_BUF_SKB);
		/*
		 * map the new buffer, if mapping fails, drop the packet and
		 * reuse the old buffer
		 */
		if (newskb) {
			skb_reserve(newskb, NET_IP_ALIGN);
			new_dma_addr = pci_map_single(lp->pci_dev, newskb->data,
			PKT_BUF_SIZE, PCI_DMA_FROMDEVICE);
			if (pci_dma_mapping_error(lp->pci_dev, new_dma_addr)) {
				netif_err(lp, rx_err, dev, "DMA mapping error.\n");
				dev_kfree_skb(newskb);
				skb = NULL;
			} else {
				skb = lp->rx_skbuff[entry];
				pci_unmap_single(lp->pci_dev, lp->rx_dma_addr[entry],
				PKT_BUF_SIZE, PCI_DMA_FROMDEVICE);
				skb_put(skb, pkt_len);
				lp->rx_skbuff[entry] = newskb;
				lp->rx_dma_addr[entry] = new_dma_addr;
				rxp->base = cpu_to_le32(new_dma_addr);
				rx_in_place = 1;
			}
		} else
			skb = NULL;
	} else
		skb = netdev_alloc_skb(dev, pkt_len + NET_IP_ALIGN);

	if (skb == NULL) {
		dev->stats.rx_dropped++;
		return;
	}
	if (!rx_in_place) {
		skb_reserve(skb, NET_IP_ALIGN);
		skb_put(skb, pkt_len); /* Make room */
		pci_dma_sync_single_for_cpu(lp->pci_dev, lp->rx_dma_addr[entry], pkt_len, PCI_DMA_FROMDEVICE);
		skb_copy_to_linear_data(skb, (unsigned char *) (lp->rx_skbuff[entry]->data), pkt_len);
		pci_dma_sync_single_for_device(lp->pci_dev, lp->rx_dma_addr[entry], pkt_len, PCI_DMA_FROMDEVICE);
	}
	dev->stats.rx_bytes += skb->len;
	skb->protocol = eth_type_trans(skb, dev);
	netif_receive_skb(skb);
	dev->stats.rx_packets++;
}

/* if any allocation fails, caller must also call pcnet32_free_ring */
static int pcnet32_alloc_ring(struct net_device *dev, const char *name) {
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	struct pcnet32_private *lp = netdev_priv(dev);

	lp->tx_ring = pci_alloc_consistent(lp->pci_dev, sizeof(struct pcnet32_tx_head) * lp->tx_ring_size, &lp->tx_ring_dma_addr);
	if (lp->tx_ring == NULL) {
		netif_err(lp, drv, dev, "Consistent memory allocation failed\n");
		return -ENOMEM;
	}

	lp->rx_ring = pci_alloc_consistent(lp->pci_dev, sizeof(struct pcnet32_rx_head) * lp->rx_ring_size, &lp->rx_ring_dma_addr);
	if (lp->rx_ring == NULL) {
		netif_err(lp, drv, dev, "Consistent memory allocation failed\n");
		return -ENOMEM;
	}

	lp->tx_dma_addr = kcalloc(lp->tx_ring_size, sizeof(dma_addr_t), GFP_ATOMIC);
	if (!lp->tx_dma_addr)
		return -ENOMEM;

	lp->rx_dma_addr = kcalloc(lp->rx_ring_size, sizeof(dma_addr_t), GFP_ATOMIC);
	if (!lp->rx_dma_addr)
		return -ENOMEM;

	lp->tx_skbuff = kcalloc(lp->tx_ring_size, sizeof(struct sk_buff *), GFP_ATOMIC);
	if (!lp->tx_skbuff)
		return -ENOMEM;

	lp->rx_skbuff = kcalloc(lp->rx_ring_size, sizeof(struct sk_buff *), GFP_ATOMIC);
	if (!lp->rx_skbuff)
		return -ENOMEM;

	return 0;
}

/*
 * Check for loss of link and link establishment.
 * Could possibly be changed to use mii_check_media instead.
 */

static void pcnet32_watchdog(struct net_device *dev) {
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long flags;

	/* Print the link status if it has changed */
	spin_lock_irqsave(&lp->lock, flags);
	pcnet32_check_media(dev, 0);
	spin_unlock_irqrestore(&lp->lock, flags);

	mod_timer(&lp->watchdog_timer, round_jiffies(PCNET32_WATCHDOG_TIMEOUT));
}
