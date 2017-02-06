#include <linux/version.h>
#include <linux/kernel.h>    // need for kernel alert
#include <linux/module.h>
#include <linux/init.h>         // need for __init and __exit
#include <asm/uaccess.h>     // for ssize_t
#include <linux/fs.h>          // for struct file_operations
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/cdev.h>
static dev_t devno=0;
static struct cdev mycdev;
#endif

static int major=230;

int dev_open (struct inode *inode, struct file *fs) {
  printk("dev_open\n");  

  return 0;
}

int dev_release (struct inode *inode, struct file *fs) {
  printk("dev_release\n");  

  return 0;
}

ssize_t dev_read (struct file *fs, char __user *buffer, size_t size, loff_t *lo) {
  printk("dev_read\n");

  return 0;
}

ssize_t dev_write (struct file *fs, const char __user *buffer, size_t size, loff_t *lo) {
  printk("dev_write\n");

  return -1;
}
// To do: declare the file operation structure
static struct file_operations fops = {
    .read = NULL,
    .write = NULL,
    .open = NULL,
    .release = NULL
};

static int __init hello_2_init(void) 
{ 
  // To do: register the character device driver
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

#else

#endif
 
    return 0;
}

static void __exit hello_2_exit(void) 
{ 
// To do: unregister the character device driver
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#else
#endif
}

module_init(hello_2_init);
module_exit(hello_2_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jared");
MODULE_DESCRIPTION("Register Character Driver!");
MODULE_SUPPORTED_DEVICE("none");
