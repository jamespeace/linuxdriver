#include <linux/version.h>
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

#define DRV_NAME	"pcnet32"

#define PCNET32_PORT_AUI      0x00
#define PCNET32_PORT_10BT     0x01
#define PCNET32_PORT_GPSI     0x02
#define PCNET32_PORT_MII      0x03

#define PCNET32_PORT_PORTSEL  0x03
#define PCNET32_PORT_ASEL     0x04
#define PCNET32_PORT_100      0x40
#define PCNET32_PORT_FD	      0x80

#define PCNET32_TOTAL_SIZE	0x20

#define PCNET32_WATCHDOG_TIMEOUT (jiffies + (2 * HZ))
#define PCNET32_BLINK_TIMEOUT	(jiffies + (HZ/4))

/* The PCNET32 Rx and Tx ring descriptors. */
struct pcnet32_rx_head {
    u32 base;
    s16 buf_length;
    s16 status;
    u32 msg_length;
    u32 reserved;
};

struct pcnet32_tx_head {
    u32 base;
    s16 length;
    s16 status;
    u32 misc;
    u32 reserved;
};

/* The PCNET32 32-Bit initialization block, described in databook. */
struct pcnet32_init_block {
    u16 mode;
    u16 tlen_rlen;
    u8	phys_addr[6];
    u16 reserved;
    u32 filter[2];
    /* Receive and transmit ring base, along with extra bits. */
    u32 rx_ring;
    u32 tx_ring;
};

/*
 * Set the number of Tx and Rx buffers, using Log_2(# buffers).
 * Reasonable default values are 4 Tx buffers, and 16 Rx buffers.
 * That translates to 2 (4 == 2^^2) and 4 (16 == 2^^4).
 */
#ifndef PCNET32_LOG_TX_BUFFERS
#define PCNET32_LOG_TX_BUFFERS 4
#define PCNET32_LOG_RX_BUFFERS 5
#endif

#define TX_RING_SIZE		(1 << (PCNET32_LOG_TX_BUFFERS))
#define TX_RING_MOD_MASK	(TX_RING_SIZE - 1)
#define TX_RING_LEN_BITS	((PCNET32_LOG_TX_BUFFERS) << 12)

#define RX_RING_SIZE		(1 << (PCNET32_LOG_RX_BUFFERS))
#define RX_RING_MOD_MASK	(RX_RING_SIZE - 1)
#define RX_RING_LEN_BITS	((PCNET32_LOG_RX_BUFFERS) << 4)

/* Offsets from base I/O address. */
#define PCNET32_WIO_RDP		0x10
#define PCNET32_WIO_RAP		0x12
#define PCNET32_WIO_RESET	0x14
#define PCNET32_WIO_BDP		0x16

#define PCNET32_DWIO_RDP	0x10
#define PCNET32_DWIO_RAP	0x14
#define PCNET32_DWIO_RESET	0x18
#define PCNET32_DWIO_BDP	0x1C

#define PKT_BUF_SZ		1544

/* PCnet32 access functions */
struct pcnet32_access {
    u16 (*read_csr)(unsigned long, int);
    void (*write_csr)(unsigned long, int, u16);
    u16 (*read_bcr)(unsigned long, int);
    void (*write_bcr)(unsigned long, int, u16);
    u16 (*read_rap)(unsigned long);
    void (*write_rap)(unsigned long, u16);
    void (*reset)(unsigned long);
};


/*
 * The first three fields of pcnet32_private are read by the ethernet device
 * so we allocate the structure should be allocated by pci_alloc_consistent().
 */
struct pcnet32_private {
    /* The Tx and Rx ring entries must be aligned on 16-byte boundaries in 32bit mode. */
    struct pcnet32_rx_head    rx_ring[RX_RING_SIZE];
    struct pcnet32_tx_head    tx_ring[TX_RING_SIZE];
    struct pcnet32_init_block init_block;
    dma_addr_t		dma_addr;	/* DMA address of beginning of this
					   object, returned by
					   pci_alloc_consistent */
    struct pci_dev	*pci_dev;	/* Pointer to the associated pci device
					   structure */
    const char		*name;
    /* The saved address of a sent-in-place packet/buffer, for skfree(). */
    struct sk_buff	*tx_skbuff[TX_RING_SIZE];
    struct sk_buff	*rx_skbuff[RX_RING_SIZE];
    dma_addr_t		tx_dma_addr[TX_RING_SIZE];
    dma_addr_t		rx_dma_addr[RX_RING_SIZE];
    struct pcnet32_access	a;
    spinlock_t		lock;		/* Guard lock */
    unsigned int	cur_rx, cur_tx;	/* The next free ring entry */
    unsigned int	dirty_rx, dirty_tx; /* The ring entries to be free()ed. */
    struct net_device_stats stats;
    char		tx_full;
    int			options;
    int	shared_irq:1,			/* shared irq possible */
	dxsuflo:1,			/* disable transmit stop on uflo */
	mii:1;				/* mii port available */
    struct net_device	*next;
    struct mii_if_info	mii_if;
    struct timer_list	watchdog_timer;
    struct timer_list	blink_timer;
    u32			msg_enable;	/* debug message level */
};

enum pci_flags_bit {
    PCI_USES_IO=1, PCI_USES_MEM=2, PCI_USES_MASTER=4,
    PCI_ADDR0=0x10<<0, PCI_ADDR1=0x10<<1, PCI_ADDR2=0x10<<2, PCI_ADDR3=0x10<<3,
};


static u16 pcnet32_wio_read_csr (unsigned long addr, int index)
{
    outw (index, addr+PCNET32_WIO_RAP);
    return inw (addr+PCNET32_WIO_RDP);
}

static void pcnet32_wio_write_csr (unsigned long addr, int index, u16 val)
{
    outw (index, addr+PCNET32_WIO_RAP);
    outw (val, addr+PCNET32_WIO_RDP);
}

static u16 pcnet32_wio_read_bcr (unsigned long addr, int index)
{
    outw (index, addr+PCNET32_WIO_RAP);
    return inw (addr+PCNET32_WIO_BDP);
}

static void pcnet32_wio_write_bcr (unsigned long addr, int index, u16 val)
{
    outw (index, addr+PCNET32_WIO_RAP);
    outw (val, addr+PCNET32_WIO_BDP);
}

static u16 pcnet32_wio_read_rap (unsigned long addr)
{
    return inw (addr+PCNET32_WIO_RAP);
}

static void pcnet32_wio_write_rap (unsigned long addr, u16 val)
{
    outw (val, addr+PCNET32_WIO_RAP);
}

static void pcnet32_wio_reset (unsigned long addr)
{
    inw (addr+PCNET32_WIO_RESET);
}

#if 0  // Masked by Jared
static int pcnet32_wio_check (unsigned long addr)
{
    outw (88, addr+PCNET32_WIO_RAP);
    return (inw (addr+PCNET32_WIO_RAP) == 88);
}
#endif
static struct pcnet32_access pcnet32_wio = {
    .read_csr	= pcnet32_wio_read_csr,
    .write_csr	= pcnet32_wio_write_csr,
    .read_bcr	= pcnet32_wio_read_bcr,
    .write_bcr	= pcnet32_wio_write_bcr,
    .read_rap	= pcnet32_wio_read_rap,
    .write_rap	= pcnet32_wio_write_rap,
    .reset	= pcnet32_wio_reset
};

/* This routine assumes that the lp->lock is held */
static int mdio_read(struct net_device *dev, int phy_id, int reg_num)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    unsigned long ioaddr = dev->base_addr;
    u16 val_out;

    if (!lp->mii)
	return 0;

    lp->a.write_bcr(ioaddr, 33, ((phy_id & 0x1f) << 5) | (reg_num & 0x1f));
    val_out = lp->a.read_bcr(ioaddr, 34);

    return val_out;
}

/* This routine assumes that the lp->lock is held */
static void mdio_write(struct net_device *dev, int phy_id, int reg_num, int val)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    unsigned long ioaddr = dev->base_addr;

    if (!lp->mii)
	return;

    lp->a.write_bcr(ioaddr, 33, ((phy_id & 0x1f) << 5) | (reg_num & 0x1f));
    lp->a.write_bcr(ioaddr, 34, val);
}

#if 0  // Masked by Jared
static void pcnet32_watchdog(struct net_device *dev)
{
    struct pcnet32_private *lp = dev->priv;
    unsigned long flags;

    /* Print the link status if it has changed */
    if (lp->mii) {
	spin_lock_irqsave(&lp->lock, flags);
	mii_check_media (&lp->mii_if, netif_msg_link(lp), 0);
	spin_unlock_irqrestore(&lp->lock, flags);
    }

    mod_timer (&(lp->watchdog_timer), PCNET32_WATCHDOG_TIMEOUT);
}
#endif
/*
 * PCI device identifiers for "new style" Linux PCI Device Drivers
 */
static struct pci_device_id pcnet32_pci_tbl[] = {
    { PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LANCE_HOME, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_LANCE, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    /*
     * Adapters that were sold with IBM's RS/6000 or pSeries hardware have
     * the incorrect vendor id.
     */
    { PCI_VENDOR_ID_TRIDENT, PCI_DEVICE_ID_AMD_LANCE, PCI_ANY_ID, PCI_ANY_ID,
	    PCI_CLASS_NETWORK_ETHERNET << 8, 0xffff00, 0 },
    { 0, }
};

static int pcnet32_open(struct net_device *dev)
{
  printk("<1>%s[%d]\n", __FUNCTION__, __LINE__);
  netif_start_queue(dev);

  return 0;
}

static int  pcnet32_start_xmit(struct sk_buff *skb, struct net_device *dev){

  printk("<1>%s[%d]\n", __FUNCTION__, __LINE__);

  dev_kfree_skb(skb);
  return 0;
}


static int  pcnet32_close(struct net_device *dev) {

  printk("<1>%s[%d]\n", __FUNCTION__, __LINE__);
  netif_stop_queue(dev);
  return 0;
}

#if 0  // Masked by Jared
static struct net_device_stats *pcnet32_get_stats(struct net_device *dev) {

  printk("<1>%s[%d]\n", __FUNCTION__, __LINE__);

  return 0;
}
static int pcnet32_ioctl(struct net_device *dev, struct ifreq *rq, int cmd) {

  printk("<1>%s[%d]\n", __FUNCTION__, __LINE__);

  return 0;
}

static void pcnet32_tx_timeout (struct net_device *dev) {

  printk("<1>%s[%d]\n", __FUNCTION__, __LINE__);
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
static const struct net_device_ops pcnet32_netdev_ops = {
	.ndo_open		= pcnet32_open,
	.ndo_stop 		= pcnet32_close,
	.ndo_start_xmit		= pcnet32_start_xmit,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};
#endif

static int __devinit
pcnet32_probe1(unsigned long ioaddr, int shared, struct pci_dev *pdev)
{
    struct pcnet32_private *lp;
    dma_addr_t lp_dma_addr;
    int i;
    struct net_device *dev;
    struct pcnet32_access *a =&pcnet32_wio;
    u8 promaddr[6];
    int ret = -ENODEV;

    // reset the chip, p.98
    pcnet32_wio_reset(ioaddr);

    // Enable bit 6(Burst read), bit 5(Burst write); 
    // bit 11 document "reserved"
    a->write_bcr(ioaddr, 18, (a->read_bcr(ioaddr, 18) | 0x0860));
    //  Set Received FIFO watermark, at least 64K bytes. p.131
    a->write_csr(ioaddr, 80, (a->read_csr(ioaddr, 80) & 0x0C00) | 0x0c00);

    // Allocates and sets up an ethernet device
    dev = alloc_etherdev(0);
    if (!dev) {
    	ret = -ENOMEM;
    	goto err_release_region;
    }
    SET_NETDEV_DEV(dev, &pdev->dev);

    // read PROM address
    for (i = 0; i < 6; i++)
    	promaddr[i] = inb(ioaddr + i);
    memcpy(dev->dev_addr, promaddr, 6);

    dev->base_addr = ioaddr;
    // To allocate and map large (PAGE_SIZE) consistent DMA region
    //   pdev: pci_dev
    //   sizeof(lp): the length of the region you want to allocate.
    //   lp_dma_addr: return the DMA handler
    //   lp: return the virtual address which you can use to access it from the CPU
    if ((lp = pci_alloc_consistent(pdev, sizeof(*lp), &lp_dma_addr)) == NULL) {
	ret = -ENOMEM;
	goto err_free_netdev;
    }

    memset(lp, 0, sizeof(*lp));
    lp->dma_addr = lp_dma_addr;
    lp->pci_dev = pdev;

    spin_lock_init(&lp->lock);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    dev->priv = lp;
#endif
    lp->name = "PCnet/FAST 79C971";
    lp->shared_irq = shared;
    lp->mii_if.full_duplex = 0x01;
    lp->mii_if.phy_id_mask = 0x1f;
    lp->mii_if.reg_num_mask = 0x1f;
    lp->dxsuflo = 0x01;  // disable transmit stop on uflo
    lp->mii = 0x01;
    lp->mii_if.dev = dev;
    lp->mii_if.mdio_read = mdio_read;
    lp->mii_if.mdio_write = mdio_write;
    lp->a = *a;

    // p.101 Some registers are initialized by the initialization block
    // p.111 CSR0[0-15]/CSR1[16-31] initialization block address
    // p.154 BCR20 bit8; 0: 16 bits mode, 1: 32 bits mode
    lp->init_block.mode = le16_to_cpu(0x0003);	// Disable Rx and Tx.
    // p.157 CSR76,CSR78 p.131
    lp->init_block.tlen_rlen = le16_to_cpu(TX_RING_LEN_BITS | RX_RING_LEN_BITS);

    // Set the MAC Address into initialization block
    for (i = 0; i < 6; i++)
	lp->init_block.phys_addr[i] = dev->dev_addr[i];
    lp->init_block.filter[0] = 0x00000000;
    lp->init_block.filter[1] = 0x00000000;
    lp->init_block.rx_ring = (u32)le32_to_cpu(lp->dma_addr +
	    offsetof(struct pcnet32_private, rx_ring));
    lp->init_block.tx_ring = (u32)le32_to_cpu(lp->dma_addr +
	    offsetof(struct pcnet32_private, tx_ring));

    // switch pcnet32 to 32bit mode. BCR20 p.153
    a->write_bcr(ioaddr, 20, 2);

    // set the initialization block address to CSR1. p111 (low byte)
    a->write_csr(ioaddr, 1, (lp->dma_addr + offsetof(struct pcnet32_private,
		    init_block)) & 0xffff);
		// set the initialization block address to CSR2. p111 (high byte)
    a->write_csr(ioaddr, 2, (lp->dma_addr + offsetof(struct pcnet32_private,
		    init_block)) >> 16);
	
	dev->irq = pdev->irq; // use the IRQ provided by PCI

    // Set the mii phy_id so that we can query the link state
    if (lp->mii)
	lp->mii_if.phy_id = ((lp->a.read_bcr (ioaddr, 33)) >> 5) & 0x1f;
#if 0  // Masked by Jared
    init_timer (&lp->watchdog_timer);
    lp->watchdog_timer.data = (unsigned long) dev;
    lp->watchdog_timer.function = (void *) &pcnet32_watchdog;
#endif
    // The PCNET32-specific entries in the device structure.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    dev->open = &pcnet32_open;
    dev->hard_start_xmit = &pcnet32_start_xmit;
    dev->stop = &pcnet32_close;
#else
    dev->netdev_ops = &pcnet32_netdev_ops;
#endif
#if 0  // Masked by Jared
//    dev->get_stats = &pcnet32_get_stats;
//    dev->do_ioctl = &pcnet32_ioctl;
//    dev->ethtool_ops = &pcnet32_ethtool_ops;
//    dev->set_multicast_list = &pcnet32_set_multicast_list;
//    dev->tx_timeout = pcnet32_tx_timeout;
    dev->watchdog_timeo = (5*HZ);
#endif

    // Fill in the generic fields of the device structure.
    if (register_netdev(dev))
	goto err_free_consistent;

    pci_set_drvdata(pdev, dev);

    a->write_bcr(ioaddr, 2, 0x1002);	// BCR2 bit1: auto-negotiate p.139

    return 0;

err_free_consistent:
    pci_free_consistent(lp->pci_dev, sizeof(*lp), lp, lp->dma_addr);
err_free_netdev:
    free_netdev(dev);
err_release_region:
    release_region(ioaddr, PCNET32_TOTAL_SIZE);
    return ret;
}

static int __devinit
pcnet32_probe_pci(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    unsigned long ioaddr;
    int err;
    
    printk("<1>name:%s (id %04x:%04x irq %x)\n", pci_name(pdev), pdev->vendor, pdev->device, pdev->irq);

    err = pci_enable_device(pdev);
    if (err < 0) {
	return err;
    }

    ioaddr = pci_resource_start (pdev, 0);
    if (!ioaddr) {
	return -ENODEV;
    }

    if (!pci_dma_supported(pdev, 0xFFFFFFFF )) {
	return -ENODEV;
    }
    if (request_region(ioaddr, PCNET32_TOTAL_SIZE, "pcnet32_probe_pci") == NULL) {
	return -EBUSY;
    }


    return pcnet32_probe1(ioaddr, 1, pdev);
}


static void __devexit pcnet32_remove_one(struct pci_dev *pdev)
{
	 struct net_device *dev = pci_get_drvdata(pdev);
	 
	 printk("<1>Remove name:%s (id %04x:%04x irq %x)\n", pci_name(pdev), pdev->vendor, pdev->device, pdev->irq);
   
   if (dev) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
   	struct pcnet32_private *lp = dev->priv;
#else
	struct pcnet32_private *lp = netdev_priv(dev);
#endif
   	unregister_netdev(dev);
   	release_region(dev->base_addr, PCNET32_TOTAL_SIZE);
	pci_disable_device(pdev);
   	pci_free_consistent(lp->pci_dev, sizeof(*lp), lp, lp->dma_addr);
   	free_netdev(dev);
   	pci_set_drvdata(pdev, NULL);
  }
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
