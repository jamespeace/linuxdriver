#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>

static struct cdev mycdev;
static int major = 230;

int dev_open(struct inode *inode, struct file *fs)
{
	printk("dev_open\n");
	return 0;
}

int dev_release(struct inode *inode, struct file *fs)
{
	printk("dev_release\n");
	return 0;
}

ssize_t dev_read(struct file *fs, char __user *buffer, size_t size, loff_t *lo) {
	printk("dev_read\n");
	return 0;
}

ssize_t dev_write(struct file *fs, const char __user *buffer, size_t size, loff_t *lo)
{
	printk("dev_wtite\n");
	return -1;
}

static struct file_operations fops = {
	.read = dev_read,
	.write = dev_write,
	.open = dev_open,
	.release = dev_release
};

static int __init hello_2_init(void)
{
	register_chrdev_region(MKDEV(major, 0), 1, "char_reg");
	cdev_init(&mycdev, &fops);
	cdev_add(&mycdev, MKDEV(major, 0), 1);

	return 0;
}

static void __exit hello_2_exit(void)
{
	cdev_del(&mycdev);
	unregister_chrdev_region(MKDEV(major, 0), 1);
}

module_init(hello_2_init);
module_exit(hello_2_exit);

MODULE_LICENSE("GPL");

