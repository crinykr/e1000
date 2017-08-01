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

extern struct pci_driver pcnet32_driver;
extern int cards_found;
extern struct net_device *pcnet32_dev;

int pcnet32_debug;

static int __init pcnet32_init_module(void)
{
	printk("### pcnet32_init_module(%s)\n", __TIME__);

	pcnet32_debug = netif_msg_init(-1, PCNET32_MSG_DEFAULT); // debug level : -1

	pci_register_driver(&pcnet32_driver);

	return cards_found ? 0 : -ENODEV;
}

static void __exit pcnet32_cleanup_module(void)
{
	struct net_device *next_dev;

	printk("### pcnet32_cleanup_module-1(%s)\n", __TIME__);

	while (pcnet32_dev) {
		printk("### pcnet32_cleanup_module-2(%s)\n", __TIME__);
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

module_init( pcnet32_init_module);
module_exit( pcnet32_cleanup_module);

MODULE_AUTHOR("Thomas Bogendoerfer");
MODULE_DESCRIPTION("Driver for PCnet32 and PCnetPCI based ethercards");
MODULE_LICENSE("GPL");
