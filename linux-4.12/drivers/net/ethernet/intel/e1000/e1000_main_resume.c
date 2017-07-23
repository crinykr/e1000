#include "e1000.h"

int e1000_resume(struct pci_dev *pdev) {
	printk("### e1000_resume(%s)\n", __TIME__);

	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 err;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_save_state(pdev);

	if (adapter->need_ioport)
		err = pci_enable_device(pdev);
	else
		err = pci_enable_device_mem(pdev);
	if (err) {
		pr_err("Cannot enable PCI device from suspend\n");
		return err;
	}
	pci_set_master(pdev);

	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);

	if (netif_running(netdev)) {
		err = e1000_request_irq(adapter);
		if (err)
			return err;
	}

	e1000_power_up_phy(adapter);
	e1000_reset(adapter);
	ew32(WUS, ~0);

	e1000_init_manageability(adapter);

	if (netif_running(netdev))
		e1000_up(adapter);

	netif_device_attach(netdev);

	return 0;
}
