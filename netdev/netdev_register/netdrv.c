#include<linux/module.h>
#include<linux/etherdevice.h>
#include<linux/netdevice.h>

static int netdrv_open(struct net_device *dev)
{
	printk("netdrv_open called\n");
	netif_start_queue(dev);
	return 0;
}

static int netdrv_release(struct net_device *dev)
{
	printk("netdrv_release called\n");
	netif_stop_queue(dev);
	return 0;
}

static int netdrv_xmit(struct sk_buff *skb, struct net_device *dev)
{
	printk("dummy xmit function called....\n");
	dev_kfree_skb(skb);
	return 0;
}

#if 1
static const struct net_device_ops netdrv_ops = {
	.ndo_open		= netdrv_open,
	.ndo_stop 		= netdrv_release,
	.ndo_start_xmit		= netdrv_xmit,
};
#endif	

struct net_device *netdrv;

int netdrv_init_module(void)
{
	unsigned char addr[6]={0x00,0x00,0xAA,0xBB,0xCC,0xDD};

	netdrv = alloc_etherdev(0);
	//strcpy(netdrv->name,"netdrv");
#if 0
	netdrv->open=&netdrv_open;
	netdrv->stop=&netdrv_release;
	netdrv->hard_start_xmit=&netdrv_xmit;
#else
	netdrv->netdev_ops = &netdrv_ops;
#endif
	memcpy(netdrv->dev_addr, addr, 6);
	if ( register_netdev(netdrv) < 0 ) {
		printk("netdrv: register_netdev() fail\n");
		return -ENODEV;
	}

	return 0;
}

void netdrv_cleanup(void)
{
	printk("<0>Cleaning Up the Module\n");
	unregister_netdev(netdrv);
	free_netdev(netdrv);
}

module_init(netdrv_init_module);
module_exit(netdrv_cleanup);
