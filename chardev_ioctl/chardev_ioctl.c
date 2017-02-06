#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>   // for put_user
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/cdev.h>
static dev_t devno=0;
static struct cdev mycdev;
#endif
#include "chardev.h"

#define SUCCESS 0
#define DEVICE_NAME "hello1"  //Dev name as it appears in "/proc/devices"
#define BUF_LEN 80

static char Message[BUF_LEN]="The string is from llseek!";
static char *Message_Ptr;

static int device_open(struct inode *inode, struct file *file)
{
  Message_Ptr=Message;
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
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,4,0)
    MOD_DEC_USE_COUNT;
#endif

  return SUCCESS;
}

static ssize_t device_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
  int i=0;
  if(*Message_Ptr ==0 ) return 0;

 while( length && *Message_Ptr )
  {
    put_user(*(Message_Ptr++), buffer++);
    length--;
    i++;
  }
  put_user(*(Message_Ptr++), buffer++);
  i++;
  
  return i;
}

static ssize_t device_write(struct file *filp,const char *buffer, size_t length, loff_t *off)
{
  int i;

  for( i=0; i<length && i<BUF_LEN ; i++) {
  // To do: copy the data from kernel space to user space
	get_user(Message[i], buffer+i);
  }
  Message_Ptr=Message;

  return i;
}

int device_ioctl(struct inode *inode, struct file *file,unsigned int ioctl_num, unsigned long ioctl_param)
{
  int len;

  switch(ioctl_num)
  {
     case IOCTL_SET_MSG:

	// To do: Store the message
	 	len = strlen_user((char *)ioctl_param);
		device_write(file, (char *)ioctl_param, len, 0);
	 	break;
     case IOCTL_GET_MSG:

	// To do: return the message
		device_read(file, (char *)ioctl_param, 99, 0);
	 	break;
     case IOCTL_GET_NTH_BYTE:
		// To do: return the nth byte
		return Message[ioctl_param];
  }
  return SUCCESS;
}

static struct file_operations fops={
  .read = device_read,
  .write = device_write,
  .ioctl = device_ioctl,
  .open = device_open,
  .release = device_release
};

static int Major=0;

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

  printk("<1>I was assigned major number %d. To talk to \n",Major);
  printk("<1>To create a device file with mknod %s c %d 0\n",DEVICE_FILE_NAME,Major);

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
}
MODULE_LICENSE("GPL");
