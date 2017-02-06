#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

#define MAJOR_NR		BLK_MAJOR
#define DEVICE_NAME		"blk"
#define DEVICE_REQUEST		blk_request
#define DEVICE_NR(device)	(MINOR(device))
#define DEVICE_ON(device)
#define DEVICE_OFF(device)
#define DEVICE_NO_RANDOM
#define BLK_MAJOR		42

#include <linux/blk.h>

MODULE_LICENSE("GPL");


/* forward declarations for _fops */
static int blk_open(struct inode *inode, struct file *file);
static int blk_release(struct inode *inode, struct file *file);
static int blk_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg);
static int blk_media_change(kdev_t dev);
static int blk_revalidate(kdev_t dev);

static struct block_device_operations blkdrv = {
	open:				blk_open,
	release:    blk_release,
	ioctl:			blk_ioctl,
	check_media_change:	blk_media_change,
	revalidate:	blk_revalidate,
	owner:			THIS_MODULE,
};

static void blk_request(request_queue_t *q)
{
radimo_begin:
        printk("blk_request:\n");

        end_request(1);
        goto radimo_begin; 
}

static int blk_media_change(kdev_t dev)
{
		printk("media has changed\n");
	
	/* 0 means medium has not changed, while 1 indicates a change */
	return 0;
}

static int blk_revalidate(kdev_t dev)
{
	printk("revalidate\n");
	
	/* just return 0, check_disk_change ignores it anyway */
	return 0;
}

static int blk_release(struct inode *inode, struct file *file)
{
	printk("closed\n");
	MOD_DEC_USE_COUNT;
	return 0;
}

static int blk_open(struct inode *inode, struct file *file)
{
	printk("opened\n");
	MOD_INC_USE_COUNT;

	return 0;
}

static int blk_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
  printk("ioctl\n");

	return 0;
}
	
static int __init blk_init(void)
{
	int res;
	/* register block device */
	res = register_blkdev(BLK_MAJOR, "blk", &blkdrv);
	if (res) {
		printk("couldn't register block device\n");
		return res;
	}
	
	return 0;
}

static void __exit blk_cleanup(void)
{
	unregister_blkdev(BLK_MAJOR, "blk");

	printk("unloaded\n");
}	

module_init(blk_init);
module_exit(blk_cleanup);

