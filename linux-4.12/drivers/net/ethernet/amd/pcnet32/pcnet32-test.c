#include "pcnet32.h"

static int pcnet32_loopback_test(struct net_device *dev, uint64_t * data1);

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
