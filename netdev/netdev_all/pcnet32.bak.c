#define DRV_NAME	"pcnet32"
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

MODULE_DEVICE_TABLE (pci, pcnet32_pci_tbl);

static struct net_device *pcnet32_dev;

static int max_interrupt_work = 2;
static int rx_copybreak = 200;

#define PCNET32_PORT_AUI      0x00
#define PCNET32_PORT_10BT     0x01
#define PCNET32_PORT_GPSI     0x02
#define PCNET32_PORT_MII      0x03

#define PCNET32_PORT_PORTSEL  0x03
#define PCNET32_PORT_ASEL     0x04
#define PCNET32_PORT_100      0x40
#define PCNET32_PORT_FD	      0x80

#define PCNET32_DMA_MASK 0xffffffff

#define PCNET32_WATCHDOG_TIMEOUT (jiffies + (2 * HZ))
#define PCNET32_BLINK_TIMEOUT	(jiffies + (HZ/4))

static const char pcnet32_gstrings_test[][ETH_GSTRING_LEN] = {
    "Loopback test  (offline)"
};
#define PCNET32_TEST_LEN (sizeof(pcnet32_gstrings_test) / ETH_GSTRING_LEN)

#define PCNET32_NUM_REGS 168

#define MAX_UNITS 8	/* More are supported, limit only on options */
static int options[MAX_UNITS];

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

#define PKT_BUF_SZ		1544

/* Offsets from base I/O address. */
#define PCNET32_WIO_RDP		0x10
#define PCNET32_WIO_RAP		0x12
#define PCNET32_WIO_RESET	0x14
#define PCNET32_WIO_BDP		0x16

#define PCNET32_DWIO_RDP	0x10
#define PCNET32_DWIO_RAP	0x14
#define PCNET32_DWIO_RESET	0x18
#define PCNET32_DWIO_BDP	0x1C

#define PCNET32_TOTAL_SIZE	0x20

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
    int	mii:1;				/* mii port available */
    struct net_device	*next;
    struct mii_if_info	mii_if;
    struct timer_list	blink_timer;
    u32			msg_enable;	/* debug message level */
};

static int  pcnet32_probe_pci(struct pci_dev *, const struct pci_device_id *);
static int  pcnet32_probe1(unsigned long, int, struct pci_dev *);
static int  pcnet32_open(struct net_device *);
static int  pcnet32_init_ring(struct net_device *);
static int  pcnet32_start_xmit(struct sk_buff *, struct net_device *);
static int  pcnet32_rx(struct net_device *);
static irqreturn_t pcnet32_interrupt(int, void *);
static int  pcnet32_close(struct net_device *);
static int mdio_read(struct net_device *dev, int phy_id, int reg_num);
static void mdio_write(struct net_device *dev, int phy_id, int reg_num, int val);
static void pcnet32_restart(struct net_device *dev, unsigned int csr0_bits);

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

static struct pcnet32_access pcnet32_wio = {
    .read_csr	= pcnet32_wio_read_csr,
    .write_csr	= pcnet32_wio_write_csr,
    .read_bcr	= pcnet32_wio_read_bcr,
    .write_bcr	= pcnet32_wio_write_bcr,
    .read_rap	= pcnet32_wio_read_rap,
    .write_rap	= pcnet32_wio_write_rap,
    .reset	= pcnet32_wio_reset
};

static int __devinit
pcnet32_probe_pci(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    unsigned long ioaddr;
    int err;

    err = pci_enable_device(pdev);
    if (err < 0) {
	    printk(KERN_ERR "failed to enable device -- err=%d\n", err);
	return err;
    }
    pci_set_master(pdev);

    ioaddr = pci_resource_start (pdev, 0);
    if (!ioaddr) {
	printk (KERN_ERR "card has no PCI IO resources, aborting\n");
	return -ENODEV;
    }

    if (!pci_dma_supported(pdev, PCNET32_DMA_MASK)) {
	printk(KERN_ERR "architecture does not support 32bit PCI busmaster DMA\n");
	return -ENODEV;
    }
    if (request_region(ioaddr, PCNET32_TOTAL_SIZE, "pcnet32_probe_pci") == NULL) {
	printk(KERN_ERR "io address range already allocated\n");
	return -EBUSY;
    }

    return pcnet32_probe1(ioaddr, 1, pdev);
}

static int __devinit
pcnet32_probe1(unsigned long ioaddr, int shared, struct pci_dev *pdev)
{
    struct pcnet32_private *lp;
    dma_addr_t lp_dma_addr;
    int i;
    struct net_device *dev;
    struct pcnet32_access *a = NULL;
    u8 promaddr[6];
    int ret = -ENODEV;

    // reset the chip, p.98
    pcnet32_wio_reset(ioaddr);
    a = &pcnet32_wio;

    // Allocates and sets up an ethernet device
    dev = alloc_etherdev(0);
    if (!dev) {
	printk(KERN_ERR "Memory allocation failed.\n");
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
	printk(KERN_ERR "Consistent memory allocation failed.\n");
	ret = -ENOMEM;
	goto err_free_netdev;
    }

    memset(lp, 0, sizeof(*lp));
    lp->dma_addr = lp_dma_addr;
    lp->pci_dev = pdev;

    spin_lock_init(&lp->lock);

    SET_NETDEV_DEV(dev, &pdev->dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    dev->priv = lp;
#else
#endif
    lp->name = "PCnet/PCI II 79C970A";
    lp->mii_if.full_duplex = 1;
    lp->mii_if.phy_id_mask = 0x1f;
    lp->mii_if.reg_num_mask = 0x1f;
    lp->mii = 0;
    lp->options = PCNET32_PORT_ASEL;
    lp->mii_if.dev = dev;
    lp->mii_if.mdio_read = mdio_read;
    lp->mii_if.mdio_write = mdio_write;
    lp->a = *a;

    // p.101 Some registers are initialized by the initialization block
    // p.111 CSR0[0-15]/CSR1[16-31] initialization block address
    // p.154 BCR20 bit8; 0: 16 bits mode, 1: 32 bits mode
    lp->init_block.mode = le16_to_cpu(0x0003);	// Disable Rx and Tx
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
    printk(" assigned IRQ %d.\n", dev->irq);

    // The PCNET32-specific entries in the device structure.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    dev->open = &pcnet32_open;
    dev->hard_start_xmit = &pcnet32_start_xmit;
    dev->stop = &pcnet32_close;
#else
    dev->netdev_ops = &pcnet32_netdev_ops;
#endif

    // Fill in the generic fields of the device structure.
    if (register_netdev(dev))
	goto err_free_consistent;

    pci_set_drvdata(pdev, dev);

    // BCR2 bit1: auto-negotiate p.139
    a->write_bcr(ioaddr, 2, 0x1002);

    return 0;

err_free_consistent:
    pci_free_consistent(lp->pci_dev, sizeof(*lp), lp, lp->dma_addr);
err_free_netdev:
    free_netdev(dev);
err_release_region:
    release_region(ioaddr, PCNET32_TOTAL_SIZE);
    return ret;
}


static int
pcnet32_open(struct net_device *dev)
{
printk("<1>%s[%d]\n", __FUNCTION__, __LINE__);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    unsigned long ioaddr = dev->base_addr;
    int i;
    int rc;
    unsigned long flags;

printk("<1>%s[%d]\n", __FUNCTION__, __LINE__);
    if (request_irq(dev->irq, &pcnet32_interrupt,
		    IRQF_SHARED, dev->name, (void *)dev)) {
	return -EAGAIN;
    }

    spin_lock_irqsave(&lp->lock, flags);

    // Reset the PCNET32
    lp->a.reset (ioaddr);

    // switch pcnet32 to 32bit mode. BCR20 p.153
    lp->a.write_bcr (ioaddr, 20, 2);

    if (netif_msg_ifup(lp))
	printk(KERN_DEBUG "%s: pcnet32_open() irq %d tx/rx rings %#x/%#x init %#x.\n",
	       dev->name, dev->irq,
	       (u32) (lp->dma_addr + offsetof(struct pcnet32_private, tx_ring)),
	       (u32) (lp->dma_addr + offsetof(struct pcnet32_private, rx_ring)),
	       (u32) (lp->dma_addr + offsetof(struct pcnet32_private, init_block)));

    // BCR2 bit1: auto-negotiate p.139
    lp->a.write_bcr(ioaddr, 2, lp->a.read_bcr (ioaddr, 2) | 0x2 );

    // handle full duplex setting
    if (lp->mii_if.full_duplex) {
	// Set BCR9 bit 0,1 p.148
       lp->a.write_bcr (ioaddr, 9, lp->a.read_bcr (ioaddr, 9) | 0x3 );
    }

    // p.101 Some registers are initialized by the initialization block
    // p.111 CSR0[0-15]/CSR1[16-31] initialization block address
    // p.154 BCR20 bit8; 0: 16 bits mode, 1: 32 bits mode
    // Disable Rx and Tx.
    lp->init_block.mode = le16_to_cpu((lp->options & PCNET32_PORT_PORTSEL) << 7);

    if (pcnet32_init_ring(dev)) {
	rc = -ENOMEM;
	goto err_free_ring;
    }

    /* Re-initialize the PCNET32, and start it when done. */
    // set the initialization block address to CSR1. p111 (low byte)
    lp->a.write_csr (ioaddr, 1, (lp->dma_addr +
		offsetof(struct pcnet32_private, init_block)) & 0xffff);
    // set the initialization block address to CSR2. p111 (high byte)
    lp->a.write_csr (ioaddr, 2, (lp->dma_addr +
		offsetof(struct pcnet32_private, init_block)) >> 16);
    // CSR4
    // Bit 11: Auto Pad tranmist.
    // Bit 8: Miss frame counter overflow.
    // Bit 5: Receive collection counter overflow.
    // Bit 2: Transmit start mask
    // Bit 0: Jabber error mask
    lp->a.write_csr (ioaddr, 4, 0x0915);
    // Bit 0: begin the initialization procedure. read initialzation block, set INIT (bit 0) clears STOP bit
    lp->a.write_csr (ioaddr, 0, 0x0001);

    netif_start_queue(dev);

    // If we have mii, print the link status and start the watchdog
    if (lp->mii) {
	mii_check_media (&lp->mii_if, netif_msg_link(lp), 1);
    }

    i = 0;
    while (i++ < 100)
	if (lp->a.read_csr (ioaddr, 0) & 0x0100)
	    break;

    // We used to clear the InitDone bit, 0x0100
    lp->a.write_csr (ioaddr, 0, 0x0042);

    if (netif_msg_ifup(lp))
	printk(KERN_DEBUG "%s: pcnet32 open after %d ticks, init block %#x csr0 %4.4x.\n",
		dev->name, i, (u32) (lp->dma_addr +
		    offsetof(struct pcnet32_private, init_block)),
		lp->a.read_csr(ioaddr, 0));

    spin_unlock_irqrestore(&lp->lock, flags);

    return 0;	// Always succeed

err_free_ring:
    // free any allocated skbuffs
    for (i = 0; i < RX_RING_SIZE; i++) {
	lp->rx_ring[i].status = 0;
	if (lp->rx_skbuff[i]) {
	    pci_unmap_single(lp->pci_dev, lp->rx_dma_addr[i], PKT_BUF_SZ-2,
		    PCI_DMA_FROMDEVICE);
	    dev_kfree_skb(lp->rx_skbuff[i]);
	}
	lp->rx_skbuff[i] = NULL;
	lp->rx_dma_addr[i] = 0;
    }

    // Switch back to 16bit mode to avoid problems with dumb
    // DOS packet driver after a warm reboot
    lp->a.write_bcr (ioaddr, 20, 4);

    spin_unlock_irqrestore(&lp->lock, flags);
    free_irq(dev->irq, dev);
    return rc;
}

static void
pcnet32_purge_tx_ring(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    int i;

    for (i = 0; i < TX_RING_SIZE; i++) {
	lp->tx_ring[i].status = 0;	/* CPU owns buffer */
	wmb();	/* Make sure adapter sees owner change */
	if (lp->tx_skbuff[i]) {
	    pci_unmap_single(lp->pci_dev, lp->tx_dma_addr[i],
		    lp->tx_skbuff[i]->len, PCI_DMA_TODEVICE);
	    dev_kfree_skb_any(lp->tx_skbuff[i]);
	}
	lp->tx_skbuff[i] = NULL;
	lp->tx_dma_addr[i] = 0;
    }
}

static int pcnet32_init_ring(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    int i;

    lp->tx_full = 0;
    lp->cur_rx = lp->cur_tx = 0;
    lp->dirty_rx = lp->dirty_tx = 0;

    for (i = 0; i < RX_RING_SIZE; i++) {
	struct sk_buff *rx_skbuff = lp->rx_skbuff[i];
	if (rx_skbuff == NULL) {
		  // Allocate sk_buffer here
	    if (!(rx_skbuff = lp->rx_skbuff[i] = dev_alloc_skb (PKT_BUF_SZ))) {
		/* there is not much, we can do at this point */
		return -1;
	    }
	    skb_reserve (rx_skbuff, 2);
	}

	rmb();
	if (lp->rx_dma_addr[i] == 0)
	    lp->rx_dma_addr[i] = pci_map_single(lp->pci_dev, rx_skbuff->tail,
		    PKT_BUF_SZ-2, PCI_DMA_FROMDEVICE);
	lp->rx_ring[i].base = (u32)le32_to_cpu(lp->rx_dma_addr[i]);
	lp->rx_ring[i].buf_length = le16_to_cpu(2-PKT_BUF_SZ);
	wmb();	/* Make sure owner changes after all others are visible */
	lp->rx_ring[i].status = le16_to_cpu(0x8000);
    }
    /* The Tx buffer address is filled in as needed, but we do need to clear
     * the upper ownership bit. */
    for (i = 0; i < TX_RING_SIZE; i++) {
	lp->tx_ring[i].status = 0;	/* CPU owns buffer */
	wmb();	/* Make sure adapter sees owner change */
	lp->tx_ring[i].base = 0;
	lp->tx_dma_addr[i] = 0;
    }
    
    // Set the ring buffer length
    lp->init_block.tlen_rlen = le16_to_cpu(TX_RING_LEN_BITS | RX_RING_LEN_BITS);
    for (i = 0; i < 6; i++)
    // Set the MAC address
	lp->init_block.phys_addr[i] = dev->dev_addr[i];
	  // Set the RX ring buffer address
    lp->init_block.rx_ring = (u32)le32_to_cpu(lp->dma_addr +
	    offsetof(struct pcnet32_private, rx_ring));
	  // Set the TX ring buffer address
    lp->init_block.tx_ring = (u32)le32_to_cpu(lp->dma_addr +
	    offsetof(struct pcnet32_private, tx_ring));
    wmb();	/* Make sure all changes are visible */
    return 0;
}

/* the pcnet32 has been issued a stop or reset.  Wait for the stop bit
 * then flush the pending transmit operations, re-initialize the ring,
 * and tell the chip to initialize.
 */
static void
pcnet32_restart(struct net_device *dev, unsigned int csr0_bits)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    unsigned long ioaddr = dev->base_addr;
    int i;

    /* wait for stop */
    for (i=0; i<100; i++)
	if (lp->a.read_csr(ioaddr, 0) & 0x0004)
	   break;

    if (i >= 100 && netif_msg_drv(lp))
	printk(KERN_ERR "%s: pcnet32_restart timed out waiting for stop.\n",
		dev->name);

    pcnet32_purge_tx_ring(dev);
    if (pcnet32_init_ring(dev))
	return;

    /* ReInit Ring */
    lp->a.write_csr (ioaddr, 0, 1);
    i = 0;
    while (i++ < 1000)
	if (lp->a.read_csr (ioaddr, 0) & 0x0100)
	    break;

    lp->a.write_csr (ioaddr, 0, csr0_bits);
}

static int
pcnet32_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    unsigned long ioaddr = dev->base_addr;
    u16 status;
    int entry;
    unsigned long flags;

    spin_lock_irqsave(&lp->lock, flags);

    if (netif_msg_tx_queued(lp)) {
	printk(KERN_DEBUG "%s: pcnet32_start_xmit() called, csr0 %4.4x.\n",
	       dev->name, lp->a.read_csr(ioaddr, 0));
    }

    /* Default status -- will not enable Successful-TxDone
     * interrupt when that option is available to us.
     */
    status = 0x8300;

    // Fill in a Tx ring entry
    // Mask to ring buffer boundary.
    entry = lp->cur_tx & TX_RING_MOD_MASK;

    // Caution: the write order is important here, set the status
    // with the "ownership" bits last.
    // p.164 the two's complement of the length of the buffer
    lp->tx_ring[entry].length = le16_to_cpu(-skb->len);
    // clear all status registers
    lp->tx_ring[entry].misc = 0x00000000;

    // Point to the address of the transmitting skb
    lp->tx_skbuff[entry] = skb;
    lp->tx_dma_addr[entry] = pci_map_single(lp->pci_dev, skb->data, skb->len,
	    PCI_DMA_TODEVICE);
    lp->tx_ring[entry].base = (u32)le32_to_cpu(lp->tx_dma_addr[entry]);
    wmb(); /* Make sure owner changes after all others are visible */

    // Default status -- will not enable Successful-TxDone
    // interrupt when that option is available to us.
    // OWN=1 owned by pcnet32 controller.
    // STP=1 ENP=1 start of packet and end of packet indicates no data chaining.
    lp->tx_ring[entry].status = le16_to_cpu(status);

    lp->cur_tx++;
    lp->stats.tx_bytes += skb->len;

    // Trigger an immediate send poll. 
    // CSR0 bit3 Transmit demand, bit6 eable interrupt. p.110
    lp->a.write_csr (ioaddr, 0, 0x0048);

    dev->trans_start = jiffies;

    if (lp->tx_ring[(entry+1) & TX_RING_MOD_MASK].base != 0) {
	lp->tx_full = 1;
	netif_stop_queue(dev);
    }
    spin_unlock_irqrestore(&lp->lock, flags);
    return 0;
}

/* The PCNET32 interrupt handler. */
static irqreturn_t
pcnet32_interrupt(int irq, void *dev_id)
{
    struct net_device *dev = dev_id;
    struct pcnet32_private *lp;
    unsigned long ioaddr;
    u16 csr0,rap;
    int boguscnt =  max_interrupt_work;
    int must_restart;

    if (!dev) {
	printk (KERN_DEBUG "%s(): irq %d for unknown device\n",__FUNCTION__, irq);
	return IRQ_NONE;
    }

    ioaddr = dev->base_addr;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    lp = dev->priv;
#else
    lp = netdev_priv(dev);
#endif

    spin_lock(&lp->lock);

    rap = lp->a.read_rap(ioaddr);
    while ((csr0 = lp->a.read_csr (ioaddr, 0)) & 0x8f00 && --boguscnt >= 0) {
	if (csr0 == 0xffff) {
	    break;			/* PCMCIA remove happened */
	}
	/* Acknowledge all of the current interrupt sources ASAP. */
	lp->a.write_csr (ioaddr, 0, csr0 & ~0x004f);

	must_restart = 0;

	if (netif_msg_intr(lp))
	    printk(KERN_DEBUG "%s: interrupt  csr0=%#2.2x new csr=%#2.2x.\n",
		   dev->name, csr0, lp->a.read_csr (ioaddr, 0));

	if (csr0 & 0x0400)		/* Rx interrupt */
	    pcnet32_rx(dev);

	if (csr0 & 0x0200) {		/* Tx-done interrupt */
	    unsigned int dirty_tx = lp->dirty_tx;
	    int delta;

	    while (dirty_tx != lp->cur_tx) {
		int entry = dirty_tx & TX_RING_MOD_MASK;
		int status = (short)le16_to_cpu(lp->tx_ring[entry].status);

		if (status < 0)
		    break;		/* It still hasn't been Txed */

		lp->tx_ring[entry].base = 0;

		if (status & 0x4000) {
		    /* There was an major error, log it. */
		    int err_status = le32_to_cpu(lp->tx_ring[entry].misc);
		    lp->stats.tx_errors++;
		    if (netif_msg_tx_err(lp))
			printk(KERN_ERR "%s: Tx error status=%04x err_status=%08x\n",
				dev->name, status, err_status);
		    if (err_status & 0x04000000) lp->stats.tx_aborted_errors++;
		    if (err_status & 0x08000000) lp->stats.tx_carrier_errors++;
		    if (err_status & 0x10000000) lp->stats.tx_window_errors++;
		    if (err_status & 0x40000000) {
			lp->stats.tx_fifo_errors++;
			/* Ackk!  On FIFO errors the Tx unit is turned off! */
			/* Remove this verbosity later! */
			if (netif_msg_tx_err(lp))
			    printk(KERN_ERR "%s: Tx FIFO error! CSR0=%4.4x\n",
				    dev->name, csr0);
			must_restart = 1;
		    }
		} else {
		    if (status & 0x1800)
			lp->stats.collisions++;
		    lp->stats.tx_packets++;
		}

		/* We must free the original skb */
		if (lp->tx_skbuff[entry]) {
		    pci_unmap_single(lp->pci_dev, lp->tx_dma_addr[entry],
			lp->tx_skbuff[entry]->len, PCI_DMA_TODEVICE);
		    dev_kfree_skb_irq(lp->tx_skbuff[entry]);
		    lp->tx_skbuff[entry] = NULL;
		    lp->tx_dma_addr[entry] = 0;
		}
		dirty_tx++;
	    }

	    delta = (lp->cur_tx - dirty_tx) & (TX_RING_MOD_MASK + TX_RING_SIZE);
	    if (delta > TX_RING_SIZE) {
		if (netif_msg_drv(lp))
		    printk(KERN_ERR "%s: out-of-sync dirty pointer, %d vs. %d, full=%d.\n",
			    dev->name, dirty_tx, lp->cur_tx, lp->tx_full);
		dirty_tx += TX_RING_SIZE;
		delta -= TX_RING_SIZE;
	    }

	    if (lp->tx_full &&
		netif_queue_stopped(dev) &&
		delta < TX_RING_SIZE - 2) {
		/* The ring is no longer full, clear tbusy. */
		lp->tx_full = 0;
		netif_wake_queue (dev);
	    }
	    lp->dirty_tx = dirty_tx;
	}

	/* Log misc errors. */
	if (csr0 & 0x4000) lp->stats.tx_errors++; /* Tx babble. */
	if (csr0 & 0x1000) {
	    /*
	     * this happens when our receive ring is full. This shouldn't
	     * be a problem as we will see normal rx interrupts for the frames
	     * in the receive ring. But there are some PCI chipsets (I can
	     * reproduce this on SP3G with Intel saturn chipset) which have
	     * sometimes problems and will fill up the receive ring with
	     * error descriptors. In this situation we don't get a rx
	     * interrupt, but a missed frame interrupt sooner or later.
	     * So we try to clean up our receive ring here.
	     */
	    pcnet32_rx(dev);
	    lp->stats.rx_errors++; /* Missed a Rx frame. */
	}
	if (csr0 & 0x0800) {
	    if (netif_msg_drv(lp))
		printk(KERN_ERR "%s: Bus master arbitration failure, status %4.4x.\n",
			dev->name, csr0);
	    /* unlike for the lance, there is no restart needed */
	}

	if (must_restart) {
	    /* reset the chip to clear the error condition, then restart */
	    lp->a.reset(ioaddr);
	    lp->a.write_csr(ioaddr, 4, 0x0915);
	    pcnet32_restart(dev, 0x0002);
	    netif_wake_queue(dev);
	}
    }

    /* Set interrupt enable. */
    lp->a.write_csr (ioaddr, 0, 0x0040);
    lp->a.write_rap (ioaddr,rap);

    if (netif_msg_intr(lp))
	printk(KERN_DEBUG "%s: exiting interrupt, csr0=%#4.4x.\n",
		dev->name, lp->a.read_csr (ioaddr, 0));

    spin_unlock(&lp->lock);

    return IRQ_HANDLED;
}

static int
pcnet32_rx(struct net_device *dev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    int entry = lp->cur_rx & RX_RING_MOD_MASK;
    int boguscnt = RX_RING_SIZE / 2;

    /* If we own the next entry, it's a new packet. Send it up. */
    while ((short)le16_to_cpu(lp->rx_ring[entry].status) >= 0) {
	int status = (short)le16_to_cpu(lp->rx_ring[entry].status) >> 8;

	if (status != 0x03) {			/* There was an error. */
	    /*
	     * There is a tricky error noted by John Murphy,
	     * <murf@perftech.com> to Russ Nelson: Even with full-sized
	     * buffers it's possible for a jabber packet to use two
	     * buffers, with only the last correctly noting the error.
	     */
	    if (status & 0x01)	/* Only count a general error at the */
		lp->stats.rx_errors++; /* end of a packet.*/
	    if (status & 0x20) lp->stats.rx_frame_errors++;
	    if (status & 0x10) lp->stats.rx_over_errors++;
	    if (status & 0x08) lp->stats.rx_crc_errors++;
	    if (status & 0x04) lp->stats.rx_fifo_errors++;
	    lp->rx_ring[entry].status &= le16_to_cpu(0x03ff);
	} else {
	    /* Malloc up new buffer, compatible with net-2e. */
	    short pkt_len = (le32_to_cpu(lp->rx_ring[entry].msg_length) & 0xfff)-4;
	    struct sk_buff *skb;

	    /* Discard oversize frames. */
	    if (unlikely(pkt_len > PKT_BUF_SZ - 2)) {
		if (netif_msg_drv(lp))
		    printk(KERN_ERR "%s: Impossible packet size %d!\n",
			    dev->name, pkt_len);
		lp->stats.rx_errors++;
	    } else if (pkt_len < 60) {
		if (netif_msg_rx_err(lp))
		    printk(KERN_ERR "%s: Runt packet!\n", dev->name);
		lp->stats.rx_errors++;
	    } else {
		int rx_in_place = 0;

		if (pkt_len > rx_copybreak) {
		    struct sk_buff *newskb;

		    if ((newskb = dev_alloc_skb(PKT_BUF_SZ))) {
			skb_reserve (newskb, 2);
			skb = lp->rx_skbuff[entry];
			pci_unmap_single(lp->pci_dev, lp->rx_dma_addr[entry],
				PKT_BUF_SZ-2, PCI_DMA_FROMDEVICE);
			skb_put (skb, pkt_len);
			lp->rx_skbuff[entry] = newskb;
			newskb->dev = dev;
			lp->rx_dma_addr[entry] =
			    pci_map_single(lp->pci_dev, newskb->tail,
				    PKT_BUF_SZ-2, PCI_DMA_FROMDEVICE);
			lp->rx_ring[entry].base = le32_to_cpu(lp->rx_dma_addr[entry]);
			rx_in_place = 1;
		    } else
			skb = NULL;
		} else {
		    skb = dev_alloc_skb(pkt_len+2);
		}

		if (skb == NULL) {
		    int i;
		    if (netif_msg_drv(lp))
			printk(KERN_ERR "%s: Memory squeeze, deferring packet.\n",
				dev->name);
		    for (i = 0; i < RX_RING_SIZE; i++)
			if ((short)le16_to_cpu(lp->rx_ring[(entry+i)
				    & RX_RING_MOD_MASK].status) < 0)
			    break;

		    if (i > RX_RING_SIZE -2) {
			lp->stats.rx_dropped++;
			lp->rx_ring[entry].status |= le16_to_cpu(0x8000);
			wmb();	/* Make sure adapter sees owner change */
			lp->cur_rx++;
		    }
		    break;
		}
		skb->dev = dev;
		if (!rx_in_place) {
		    skb_reserve(skb,2); /* 16 byte align */
		    skb_put(skb,pkt_len);	/* Make room */
		    pci_dma_sync_single_for_cpu(lp->pci_dev,
						lp->rx_dma_addr[entry],
						PKT_BUF_SZ-2,
						PCI_DMA_FROMDEVICE);
		    skb_copy_to_linear_data(skb,
				 (unsigned char *)(lp->rx_skbuff[entry]->data),
				 pkt_len);
/*		    eth_copy_and_sum(skb,
			    (unsigned char *)(lp->rx_skbuff[entry]->tail),
			    pkt_len,0);
*/		    pci_dma_sync_single_for_device(lp->pci_dev,
						   lp->rx_dma_addr[entry],
						   PKT_BUF_SZ-2,
						   PCI_DMA_FROMDEVICE);
		}
		lp->stats.rx_bytes += skb->len;
		skb->protocol=eth_type_trans(skb,dev);
		netif_rx(skb);
		dev->last_rx = jiffies;
		lp->stats.rx_packets++;
	    }
	}
	/*
	 * The docs say that the buffer length isn't touched, but Andrew Boyd
	 * of QNX reports that some revs of the 79C965 clear it.
	 */
	lp->rx_ring[entry].buf_length = le16_to_cpu(2-PKT_BUF_SZ);
	wmb(); /* Make sure owner changes after all others are visible */
	lp->rx_ring[entry].status |= le16_to_cpu(0x8000);
	entry = (++lp->cur_rx) & RX_RING_MOD_MASK;
	if (--boguscnt <= 0) break;	/* don't stay in loop forever */
    }

    return 0;
}

static int
pcnet32_close(struct net_device *dev)
{
    unsigned long ioaddr = dev->base_addr;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
    struct pcnet32_private *lp = dev->priv;
#else
    struct pcnet32_private *lp = netdev_priv(dev);
#endif
    int i;
    unsigned long flags;

    netif_stop_queue(dev);

    spin_lock_irqsave(&lp->lock, flags);

    lp->stats.rx_missed_errors = lp->a.read_csr (ioaddr, 112);

    if (netif_msg_ifdown(lp))
	printk(KERN_DEBUG "%s: Shutting down ethercard, status was %2.2x.\n",
	       dev->name, lp->a.read_csr (ioaddr, 0));

    /* We stop the PCNET32 here -- it occasionally polls memory if we don't. */
    lp->a.write_csr (ioaddr, 0, 0x0004);

    /*
     * Switch back to 16bit mode to avoid problems with dumb
     * DOS packet driver after a warm reboot
     */
    lp->a.write_bcr (ioaddr, 20, 4);

    spin_unlock_irqrestore(&lp->lock, flags);

    free_irq(dev->irq, dev);

    spin_lock_irqsave(&lp->lock, flags);

    /* free all allocated skbuffs */
    for (i = 0; i < RX_RING_SIZE; i++) {
	lp->rx_ring[i].status = 0;
	wmb();		/* Make sure adapter sees owner change */
	if (lp->rx_skbuff[i]) {
	    pci_unmap_single(lp->pci_dev, lp->rx_dma_addr[i], PKT_BUF_SZ-2,
		    PCI_DMA_FROMDEVICE);
	    dev_kfree_skb(lp->rx_skbuff[i]);
	}
	lp->rx_skbuff[i] = NULL;
	lp->rx_dma_addr[i] = 0;
    }

    for (i = 0; i < TX_RING_SIZE; i++) {
	lp->tx_ring[i].status = 0;	/* CPU owns buffer */
	wmb();		/* Make sure adapter sees owner change */
	if (lp->tx_skbuff[i]) {
	    pci_unmap_single(lp->pci_dev, lp->tx_dma_addr[i],
		    lp->tx_skbuff[i]->len, PCI_DMA_TODEVICE);
	    dev_kfree_skb(lp->tx_skbuff[i]);
	}
	lp->tx_skbuff[i] = NULL;
	lp->tx_dma_addr[i] = 0;
    }

    spin_unlock_irqrestore(&lp->lock, flags);

    return 0;
}

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

static void __devexit pcnet32_remove_one(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);

    if (dev) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
   	struct pcnet32_private *lp = dev->priv;
#else
	struct pcnet32_private *lp = netdev_priv(dev);
#endif

	unregister_netdev(dev);
	release_region(dev->base_addr, PCNET32_TOTAL_SIZE);
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

/* An additional parameter that may be passed in... */
static int num_params;

module_param(max_interrupt_work, int, 0);
MODULE_PARM_DESC(max_interrupt_work, DRV_NAME " maximum events handled per interrupt");
module_param(rx_copybreak, int, 0);
MODULE_PARM_DESC(rx_copybreak, DRV_NAME " copy breakpoint for copy-only-tiny-frames");
module_param_array(options, int, &num_params, 0);
MODULE_PARM_DESC(options, DRV_NAME " initial option setting(s) (0-15)");
MODULE_DESCRIPTION("Driver for PCnet32 and PCnetPCI based ethercards");
MODULE_LICENSE("GPL");

static int __init pcnet32_init_module(void)
{
    /* find the PCI devices */
    if ( pci_register_driver(&pcnet32_driver) < 0 )
      return -ENODEV;

    return 0;
}

static void __exit pcnet32_cleanup_module(void)
{
    struct net_device *next_dev;

    while (pcnet32_dev) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
	struct pcnet32_private *lp = pcnet32_dev->priv;
#else
        struct pcnet32_private *lp = netdev_priv(pcnet32_dev);
#endif
	next_dev = lp->next;
	unregister_netdev(pcnet32_dev);
	release_region(pcnet32_dev->base_addr, PCNET32_TOTAL_SIZE);
	pci_free_consistent(lp->pci_dev, sizeof(*lp), lp, lp->dma_addr);
	free_netdev(pcnet32_dev);
	pcnet32_dev = next_dev;
    }

    pci_unregister_driver(&pcnet32_driver);
}

module_init(pcnet32_init_module);
module_exit(pcnet32_cleanup_module);
