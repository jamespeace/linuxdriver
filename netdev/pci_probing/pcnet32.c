
#define DRV_NAME	"pcnet32"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/crc32.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>

#include <asm/bitops.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

/*
 * PCI device identifiers for "new style" Linux PCI Device Drivers
 */
static struct pci_device_id pcnet32_pci_tbl[] = {
    { PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LANCE, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
#if 0
    /*
     * Adapters that were sold with IBM's RS/6000 or pSeries hardware have
     * the incorrect vendor id.
     */
    { PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LANCE_HOME, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { PCI_VENDOR_ID_TRIDENT, PCI_DEVICE_ID_AMD_LANCE, PCI_ANY_ID, PCI_ANY_ID,
	    PCI_CLASS_NETWORK_ETHERNET << 8, 0xffff00, 0 },
#endif
    { 0, }
};

static int __devinit
pcnet32_probe_pci(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    printk("<1>slot_name:%s (id %04x:%04x irq %x)\n", pci_name(pdev), pdev->vendor, pdev->device, pdev->irq);

     return 0;
}


static void __devexit pcnet32_remove_one(struct pci_dev *pdev)
{
    printk("<1>Remove slot_name:%s (id %04x:%04x irq %x)\n", pci_name(pdev), pdev->vendor, pdev->device, pdev->irq);
}

static struct pci_driver pcnet32_driver = {
    .name	= DRV_NAME,
    .probe	= pcnet32_probe_pci,
    .remove	= __devexit_p(pcnet32_remove_one),
    .id_table	= pcnet32_pci_tbl,
};


MODULE_AUTHOR("Jared Wu");
MODULE_DESCRIPTION("Driver for PCnet32 and PCnetPCI based ethercards");
MODULE_LICENSE("GPL");

#define PCNET32_MSG_DEFAULT (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK)

static int __init pcnet32_init_module(void)
{
    /* find the PCI devices */
    if ( pci_register_driver(&pcnet32_driver) < 0 )
      return -ENODEV;
      
    return  0;
}

static void __exit pcnet32_cleanup_module(void)
{
	pci_unregister_driver(&pcnet32_driver);
}

module_init(pcnet32_init_module);
module_exit(pcnet32_cleanup_module);
