#define __FILE__ "stub-pci"
#include "pcnet32.h"

int pcnet32_probe1(unsigned long, int, struct pci_dev *);

int pcnet32_probe_pci(struct pci_dev *pdev, const struct pci_device_id *ent) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

	unsigned long ioaddr;

	pci_enable_device(pdev);
	pci_set_master(pdev);
	ioaddr = pci_resource_start(pdev, 0);
	pci_set_dma_mask(pdev, PCNET32_DMA_MASK);
	request_region(ioaddr, PCNET32_TOTAL_SIZE, "pcnet32_probe_pci");

	return pcnet32_probe1(ioaddr, 1, pdev);
}

static void pcnet32_remove_one(struct pci_dev *pdev) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

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

static int pcnet32_pm_suspend(struct pci_dev *pdev, pm_message_t state) {
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

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
	printk("!!! [%s:%d] - (%s:%s)\n", current->comm, current->pid, __FILE__, __FUNCTION__);

	struct net_device *dev = pci_get_drvdata(pdev);

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	if (netif_running(dev)) {
		pcnet32_open(dev);
		netif_device_attach(dev);
	}
	return 0;
}

const struct pci_device_id pcnet32_pci_tbl[] = { // PCI device identifiers for "new style" Linux PCI Device Drivers
			{
				PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LANCE_HOME), }, //
			{
				PCI_DEVICE(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LANCE), },
			{
				PCI_DEVICE(PCI_VENDOR_ID_TRIDENT, PCI_DEVICE_ID_AMD_LANCE),
				.class = (PCI_CLASS_NETWORK_ETHERNET << 8),
				.class_mask = 0xffff00, }, /* Adapters that were sold with IBM's RS/6000 or pSeries hardware have the incorrect vendor id. */
			{ /* terminate list */} };

MODULE_DEVICE_TABLE( pci, pcnet32_pci_tbl);

struct pci_driver pcnet32_driver = { //
			.name = DRV_NAME, //
			.probe = pcnet32_probe_pci,
			.remove = pcnet32_remove_one,
			.suspend = pcnet32_pm_suspend,
			.resume = pcnet32_pm_resume,
			.id_table = pcnet32_pci_tbl, };
