#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fcntl.h>  // for O_NONBLOCK
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/err.h>
#else
#include <linux/errno.h>
#endif
#include <asm/uaccess.h>   // for put_user

#include <linux/sched.h>   // for wait queue, TASK_INTERRUPTIBLE

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/cdev.h>
static dev_t devno=0;
static struct cdev mycdev;
#endif
#include "chardev.h"

#define DEVICE_NAME "hello1"  //Dev name as it appears in "/proc/devices"
#define BUF_LEN 80

static char Message[BUF_LEN];

DECLARE_WAIT_QUEUE_HEAD(char_wq);
static int bEmpty = 0;
static int device_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 0)
	try_module_get(THIS_MODULE);
#else
	MOD_INC_USE_COUNT;
#endif
	return 0;
}

static int device_release(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 0)
	module_put(THIS_MODULE);	
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 4, 0)
	MOD_INC_USE_COUNT;
#endif
	return 0;
}

static int device_read(struct file *filp, char *buffer, size_t length, loff_t *off)
{
	char *from = Message;

	if (filp->f_flags)
}
