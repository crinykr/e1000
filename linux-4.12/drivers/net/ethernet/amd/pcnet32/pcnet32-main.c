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
#define __FILE__ "main"
#include "pcnet32.h"

extern struct pci_driver pcnet32_driver;

int pcnet32_debug;

static int __init pcnet32_init_module(void)
{
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	pcnet32_debug = netif_msg_init(-1, PCNET32_MSG_DEFAULT); // debug level : -1
	pci_register_driver(&pcnet32_driver);

	return 0;
}

static void __exit pcnet32_cleanup_module(void)
{
	printk("@(%s:%s)\n", __FILE__, __FUNCTION__);

	pci_unregister_driver(&pcnet32_driver);
}

module_init( pcnet32_init_module);
module_exit( pcnet32_cleanup_module);

MODULE_AUTHOR("Thomas Bogendoerfer");
MODULE_DESCRIPTION("Driver for PCnet32 and PCnetPCI based ethercards");
MODULE_LICENSE("GPL");
