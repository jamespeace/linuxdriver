#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h> /* for put_user */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/cdev.h>
static dev_t devno=0;
static struct cdev mycdev;
#endif
/* Prototypes - this would normally go in a .h file */
int init_module(void);
void cleanup_module(void);
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);

#define SUCCESS 0
#define DEVICE_NAME "chardev" /* Dev name as it appears in /proc/devices */
#define BUF_LEN 80

// Global variables are declared as static, so are global within the file.
static int Major=0; // Major number assigned to our device driver

static char Message[BUF_LEN], *msg_Ptr; // The Message the device will give when asked
static struct file_operations fops = {
    .read = device_read,
    .write = device_write,
    .open = device_open,
    .release = device_release
};
// Functions
int init_module(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
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
    printk("<1>I was assigned major number %d. To talk to\n", Major);
    printk("<1>the driver, create a dev file with\n");
    printk("'mknod /dev/hello c %d 0'.\n", Major);
    printk("<1>Try various minor numbers. Try to cat and echo to\n");
    printk("the device file.\n");
    printk("<1>Remove the device file and module when done.\n");
    return 0;
}

void cleanup_module(void)
{
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
static int device_open(struct inode *inode, struct file *file)
{
    static int counter = 0;

// To do: initialize the device

    return SUCCESS;
}
static int device_release(struct inode *inode, struct file *file)
{
    return 0;
}
/* Called when a process, which already opened the dev file, attempts to read from it. 
 * struct file *filp: the file operation
 * char *buffer: The buffer to fill with data
 * size_t length: The length of the buffer
 * loff_t *offset: Our offset in the file
 */
static ssize_t device_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
    int bytes_read = 0;
    if (*msg_Ptr == 0) 
        return 0;
    // Actually put the data into the buffer
    while (length && *msg_Ptr) {
	// To do: copy data to the user space buffer.

        length--;
        bytes_read++;
    }

    // Most read functions return the number of bytes put into the buffer
    return bytes_read;
}
/* Called when a process writes to dev file: echo "hi" > /dev/hello */
static ssize_t device_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    printk ("<1>Sorry, this operation isn't supported.\n");
    return -EINVAL;
}
MODULE_LICENSE("GPL");
