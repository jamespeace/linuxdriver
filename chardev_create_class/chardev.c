#include <linux/version.h>
#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/fs.h>
#include <linux/init.h>  
#include <linux/major.h>  
#include <linux/device.h>  // For class_create(), device_create()
#include <linux/poll.h>  
 
#include <asm/uaccess.h> /* for put_user */

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/cdev.h>
static struct cdev mycdev;
#endif

#define SUCCESS 0
#define DEVICE_NAME "chardev"
#define BUF_LEN 80

static int Major=0;
static char Message[BUF_LEN], *msg_Ptr;

// To do: declare a class for udev used


static int device_open(struct inode *inode, struct file *file)
{
    static int counter = 0;
    sprintf(Message,"I already told you %d times Hello world!\n", counter++);
    msg_Ptr = Message;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    try_module_get(THIS_MODULE);
#else
    MOD_INC_USE_COUNT;
#endif
    return SUCCESS;
}
static int device_release(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    module_put(THIS_MODULE);
#else
    MOD_DEC_USE_COUNT;
#endif
    return 0;
}

static ssize_t device_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
    int bytes_read = 0;
    if (*msg_Ptr == 0) 
        return 0;
    // Actually put the data into the buffer
    while (length && *msg_Ptr) {
	// the kernel data segment to the user data segment. 
        put_user(*(msg_Ptr++), buffer++);
        length--;
        bytes_read++;
    }

    // Most read functions return the number of bytes put into the buffer
    return bytes_read;
}

static ssize_t device_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    printk ("<1>Sorry, this operation isn't supported.\n");
    return -EINVAL;
}

static struct file_operations fops = {
    .read = device_read,
    .write = device_write,
    .open = device_open,
    .release = device_release
};
// Functions

int __init init_module(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    dev_t devno=0;

    if(Major) {
        if ( register_chrdev_region(MKDEV(Major,0), 1, DEVICE_NAME) < 0 ) {
            printk ("register_chrdev_region() fail\n");
            return -1;
        }
    }
    else {
        if (alloc_chrdev_region(&devno, 0, 1, DEVICE_NAME) < 0) {
           printk ("alloc_chrdev_region() fail\n");
           return -1;
        }
        Major=MAJOR(devno);
    }
    cdev_init(&mycdev, &fops);
    mycdev.owner=THIS_MODULE;
    if(cdev_add(&mycdev, MKDEV(Major,0), 1)) {
        printk ("Error adding cdev\n");
    }
#else
    Major = register_chrdev(0, DEVICE_NAME, &fops);
    if (Major < 0) {
        printk ("Registering the character device failed with %d\n", Major);
        return -1;
    }
#endif

    // To do: create your own class under /sysfs 

    // To do: register your own device in sysfs, and this will cause udev to create corresponding device node */

    printk("<1>I was assigned major number %d. To talk to\n", Major);
    printk("<1>the driver, create a dev file with\n");
    printk("'mknod /dev/hello c %d 0'.\n", Major);
    printk("<1>Try various minor numbers. Try to cat and echo to\n");
    printk("the device file.\n");
    printk("<1>Remove the device file and module when done.\n");
    return 0;
}

void __exit cleanup_module(void)
{
    // To do: delete device node under /dev
    // To do: delete class created by us

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    cdev_del(&mycdev);
    unregister_chrdev_region(MKDEV(Major, 0), 1);
#else
    /* Unregister the device */
    int ret = unregister_chrdev(Major, DEVICE_NAME);
    if (ret < 0) 
        printk("Error in unregister_chrdev: %d\n", ret);
#endif
    printk("unregister_chrdev\n");
}

MODULE_LICENSE ("GPL");
