#define __FILE__ "stub-netdev"
#include "pcnet32.h"

static irqreturn_t pcnet32_interrupt(int irq, void *dev_id);
static void pcnet32_load_multicast(struct net_device *dev);

int pcnet32_open(struct net_device *dev) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

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
		struct ethtool_cmd ecmd = {
			.cmd = ETHTOOL_GSET };

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

int pcnet32_close(struct net_device *dev) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

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

static netdev_tx_t pcnet32_start_xmit(struct sk_buff *skb, struct net_device *dev) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

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

static void pcnet32_tx_timeout(struct net_device *dev) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long ioaddr = dev->base_addr, flags;

	spin_lock_irqsave(&lp->lock, flags);
	/* Transmitter timeout, serious problems. */
	//if (pcnet32_debug & NETIF_MSG_DRV)
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

static struct net_device_stats *pcnet32_get_stats(struct net_device *dev) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long ioaddr = dev->base_addr;
	unsigned long flags;

	spin_lock_irqsave(&lp->lock, flags);
	dev->stats.rx_missed_errors = lp->a->read_csr(ioaddr, 112);
	spin_unlock_irqrestore(&lp->lock, flags);

	return &dev->stats;
}

/*
 * Set or clear the multicast filter for this adaptor.
 */
static void pcnet32_set_multicast_list(struct net_device *dev) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

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
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

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

#ifdef CONFIG_NET_POLL_CONTROLLER
static void pcnet32_poll_controller(struct net_device *dev) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

	disable_irq(dev->irq);
	pcnet32_interrupt(0, dev);
	enable_irq(dev->irq);
}
#endif

const struct net_device_ops pcnet32_netdev_ops = { //
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

/**********************
 * Extra Function
 **********************/

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
