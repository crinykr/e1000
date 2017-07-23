#include "e1000.h"

int e1000_suspend(struct pci_dev *pdev, pm_message_t state) {
	printk("### e1000_suspend(%s)\n", __TIME__);

	int retval;
	bool wake;

	retval = __e1000_shutdown(pdev, &wake);
	if (retval)
		return retval;

	if (wake) {
		pci_prepare_to_sleep(pdev);
	} else {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}

	return 0;
}
