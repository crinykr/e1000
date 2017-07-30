#include "pcnet32.h"

static int pcnet32_check_otherphy(struct net_device *dev);

/*
 * Theory of Operation
 *
 * This driver uses the same software structure as the normal lance driver.
 * So look for a verbose description in lance.c.
 * The differences to the normal lance driver is the use of the 32bit mode of PCnet32 and PCnetPCI chips.
 * Because these chips are 32bit chips, there is no 16MB limitation and we don't need bounce buffers.
 */

void pcnet32_netif_stop(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);

	netif_trans_update(dev); /* prevent tx timeout */
	napi_disable(&lp->napi);
	netif_tx_disable(dev);
}

void pcnet32_netif_start(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	ulong ioaddr = dev->base_addr;
	u16 val;

	netif_wake_queue(dev);
	val = lp->a->read_csr(ioaddr, CSR3);
	val &= 0x00ff;
	lp->a->write_csr(ioaddr, CSR3, val);
	napi_enable(&lp->napi);
}

void pcnet32_purge_rx_ring(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	int i;

	/* free all allocated skbuffs */
	for (i = 0; i < lp->rx_ring_size; i++) {
		lp->rx_ring[i].status = 0; /* CPU owns buffer */
		wmb(); /* Make sure adapter sees owner change */
		if (lp->rx_skbuff[i]) {
			if (!pci_dma_mapping_error(lp->pci_dev, lp->rx_dma_addr[i]))
				pci_unmap_single(lp->pci_dev, lp->rx_dma_addr[i],
				PKT_BUF_SIZE, PCI_DMA_FROMDEVICE);
			dev_kfree_skb_any(lp->rx_skbuff[i]);
		}
		lp->rx_skbuff[i] = NULL;
		lp->rx_dma_addr[i] = 0;
	}
}

/* the pcnet32 has been issued a stop or reset.  Wait for the stop bit
 * then flush the pending transmit operations, re-initialize the ring,
 * and tell the chip to initialize.
 */
void pcnet32_restart(struct net_device *dev, unsigned int csr0_bits) {
	struct pcnet32_private *lp = netdev_priv(dev);
	unsigned long ioaddr = dev->base_addr;
	int i;

	/* wait for stop */
	for (i = 0; i < 100; i++)
		if (lp->a->read_csr(ioaddr, CSR0) & CSR0_STOP)
			break;

	if (i >= 100)
		netif_err(lp, drv, dev, "%s timed out waiting for stop\n", __func__);

	pcnet32_purge_tx_ring(dev);
	if (pcnet32_init_ring(dev))
		return;

	/* ReInit Ring */
	lp->a->write_csr(ioaddr, CSR0, CSR0_INIT);
	i = 0;
	while (i++ < 1000)
		if (lp->a->read_csr(ioaddr, CSR0) & CSR0_IDON)
			break;

	lp->a->write_csr(ioaddr, CSR0, csr0_bits);
}

void pcnet32_free_ring(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);

	kfree(lp->tx_skbuff);
	lp->tx_skbuff = NULL;

	kfree(lp->rx_skbuff);
	lp->rx_skbuff = NULL;

	kfree(lp->tx_dma_addr);
	lp->tx_dma_addr = NULL;

	kfree(lp->rx_dma_addr);
	lp->rx_dma_addr = NULL;

	if (lp->tx_ring) {
		pci_free_consistent(lp->pci_dev, sizeof(struct pcnet32_tx_head) * lp->tx_ring_size, lp->tx_ring, lp->tx_ring_dma_addr);
		lp->tx_ring = NULL;
	}

	if (lp->rx_ring) {
		pci_free_consistent(lp->pci_dev, sizeof(struct pcnet32_rx_head) * lp->rx_ring_size, lp->rx_ring, lp->rx_ring_dma_addr);
		lp->rx_ring = NULL;
	}
}

/*
 * The LANCE has been halted for one reason or another (busmaster memory
 * arbitration error, Tx FIFO underflow, driver stopped it to reconfigure,
 * etc.).  Modern LANCE variants always reload their ring-buffer
 * configuration when restarted, so we must reinitialize our ring
 * context before restarting.  As part of this reinitialization,
 * find all packets still on the Tx ring and pretend that they had been
 * sent (in effect, drop the packets on the floor) - the higher-level
 * protocols will time out and retransmit.  It'd be better to shuffle
 * these skbs to a temp list and then actually re-Tx them after
 * restarting the chip, but I'm too lazy to do so right now.  dplatt@3do.com
 */

void pcnet32_purge_tx_ring(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	int i;

	for (i = 0; i < lp->tx_ring_size; i++) {
		lp->tx_ring[i].status = 0; /* CPU owns buffer */
		wmb(); /* Make sure adapter sees owner change */
		if (lp->tx_skbuff[i]) {
			if (!pci_dma_mapping_error(lp->pci_dev, lp->tx_dma_addr[i]))
				pci_unmap_single(lp->pci_dev, lp->tx_dma_addr[i], lp->tx_skbuff[i]->len, PCI_DMA_TODEVICE);
			dev_kfree_skb_any(lp->tx_skbuff[i]);
		}
		lp->tx_skbuff[i] = NULL;
		lp->tx_dma_addr[i] = 0;
	}
}

/* Initialize the PCNET32 Rx and Tx rings. */
int pcnet32_init_ring(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	int i;

	lp->tx_full = 0;
	lp->cur_rx = lp->cur_tx = 0;
	lp->dirty_rx = lp->dirty_tx = 0;

	for (i = 0; i < lp->rx_ring_size; i++) {
		struct sk_buff *rx_skbuff = lp->rx_skbuff[i];
		if (rx_skbuff == NULL) {
			lp->rx_skbuff[i] = netdev_alloc_skb(dev, PKT_BUF_SKB);
			rx_skbuff = lp->rx_skbuff[i];
			if (!rx_skbuff) {
				/* there is not much we can do at this point */
				netif_err(lp, drv, dev, "%s netdev_alloc_skb failed\n", __func__);
				return -1;
			}
			skb_reserve(rx_skbuff, NET_IP_ALIGN);
		}

		rmb();
		if (lp->rx_dma_addr[i] == 0) {
			lp->rx_dma_addr[i] = pci_map_single(lp->pci_dev, rx_skbuff->data,
			PKT_BUF_SIZE, PCI_DMA_FROMDEVICE);
			if (pci_dma_mapping_error(lp->pci_dev, lp->rx_dma_addr[i])) {
				/* there is not much we can do at this point */
				netif_err(lp, drv, dev, "%s pci dma mapping error\n", __func__);
				return -1;
			}
		}
		lp->rx_ring[i].base = cpu_to_le32(lp->rx_dma_addr[i]);
		lp->rx_ring[i].buf_length = cpu_to_le16(NEG_BUF_SIZE);
		wmb(); /* Make sure owner changes after all others are visible */
		lp->rx_ring[i].status = cpu_to_le16(0x8000);
	}
	/* The Tx buffer address is filled in as needed, but we do need to clear
	 * the upper ownership bit. */
	for (i = 0; i < lp->tx_ring_size; i++) {
		lp->tx_ring[i].status = 0; /* CPU owns buffer */
		wmb(); /* Make sure adapter sees owner change */
		lp->tx_ring[i].base = 0;
		lp->tx_dma_addr[i] = 0;
	}

	lp->init_block->tlen_rlen = cpu_to_le16(lp->tx_len_bits | lp->rx_len_bits);
	for (i = 0; i < 6; i++)
		lp->init_block->phys_addr[i] = dev->dev_addr[i];
	lp->init_block->rx_ring = cpu_to_le32(lp->rx_ring_dma_addr);
	lp->init_block->tx_ring = cpu_to_le32(lp->tx_ring_dma_addr);
	wmb(); /* Make sure all changes are visible */
	return 0;
}

/*
 * Show the status of the media.  Similar to mii_check_media however it
 * correctly shows the link speed for all (tested) pcnet32 variants.
 * Devices with no mii just report link state without speed.
 *
 * Caller is assumed to hold and release the lp->lock.
 */

void pcnet32_check_media(struct net_device *dev, int verbose) {
	struct pcnet32_private *lp = netdev_priv(dev);
	int curr_link;
	int prev_link = netif_carrier_ok(dev) ? 1 : 0;
	u32 bcr9;

	if (lp->mii) {
		curr_link = mii_link_ok(&lp->mii_if);
	} else if (lp->chip_version == PCNET32_79C970A) {
		ulong ioaddr = dev->base_addr; /* card base I/O address */
		/* only read link if port is set to TP */
		if (!lp->autoneg && lp->port_tp)
			curr_link = (lp->a->read_bcr(ioaddr, 4) != 0xc0);
		else
			/* link always up for AUI port or port auto select */
			curr_link = 1;
	} else {
		ulong ioaddr = dev->base_addr; /* card base I/O address */
		curr_link = (lp->a->read_bcr(ioaddr, 4) != 0xc0);
	}
	if (!curr_link) {
		if (prev_link || verbose) {
			netif_carrier_off(dev);
			netif_info(lp, link, dev, "link down\n");
		}
		if (lp->phycount > 1) {
			curr_link = pcnet32_check_otherphy(dev);
			prev_link = 0;
		}
	} else if (verbose || !prev_link) {
		netif_carrier_on(dev);
		if (lp->mii) {
			if (netif_msg_link(lp)) {
				struct ethtool_cmd ecmd = {
					.cmd = ETHTOOL_GSET };
				mii_ethtool_gset(&lp->mii_if, &ecmd);
				netdev_info(dev, "link up, %uMbps, %s-duplex\n", ethtool_cmd_speed(&ecmd), (ecmd.duplex == DUPLEX_FULL) ? "full" : "half");
			}
			bcr9 = lp->a->read_bcr(dev->base_addr, 9);
			if ((bcr9 & (1 << 0)) != lp->mii_if.full_duplex) {
				if (lp->mii_if.full_duplex)
					bcr9 |= (1 << 0);
				else
					bcr9 &= ~(1 << 0);
				lp->a->write_bcr(dev->base_addr, 9, bcr9);
			}
		} else {
			netif_info(lp, link, dev, "link up\n");
		}
	}
}

static int pcnet32_check_otherphy(struct net_device *dev) {
	struct pcnet32_private *lp = netdev_priv(dev);
	struct mii_if_info mii = lp->mii_if;
	u16 bmcr;
	int i;

	for (i = 0; i < PCNET32_MAX_PHYS; i++) {
		if (i == lp->mii_if.phy_id)
			continue; /* skip active phy */
		if (lp->phymask & (1 << i)) {
			mii.phy_id = i;
			if (mii_link_ok(&mii)) {
				/* found PHY with active link */
				netif_info(lp, link, dev, "Using PHY number %d\n", i);

				/* isolate inactive phy */
				bmcr = mdio_read(dev, lp->mii_if.phy_id, MII_BMCR);
				mdio_write(dev, lp->mii_if.phy_id, MII_BMCR, bmcr | BMCR_ISOLATE);

				/* de-isolate new phy */
				bmcr = mdio_read(dev, i, MII_BMCR);
				mdio_write(dev, i, MII_BMCR, bmcr & ~BMCR_ISOLATE);

				/* set new phy address */
				lp->mii_if.phy_id = i;
				return 1;
			}
		}
	}
	return 0;
}
