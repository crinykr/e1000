#include "e1000.h"

void e1000_shutdown(struct pci_dev *pdev) {
	bool wake;

	__e1000_shutdown(pdev, &wake);

	if (system_state == SYSTEM_POWER_OFF) {
		pci_wake_from_d3(pdev, wake);
		pci_set_power_state(pdev, PCI_D3hot);
	}
}
