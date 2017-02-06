#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>

#define DEVICE_NAME "waitq"
// To do: declare a wait queue
DECLARE_WAIT_QUEUE_HEAD(wq);

// To do: declare one flags for waiting event, bSleep.
static int bSleep;

static int Major; // Major number assigned to our device driver

static int device_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        try_module_get(THIS_MODULE);
#else
        MOD_INC_USE_COUNT;
#endif
    return 0;
}
static int device_release(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    module_put(THIS_MODULE);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,4,0)
    MOD_DEC_USE_COUNT;
#endif
    return 0;
}

static ssize_t device_read(struct file *filp, char *buffer, size_t length, loff_t *offset)
{
    printk("I am going to sleep\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    // To do: wait for an event.
	bSleep = 0;
	wait_event_interruptible(wq, bSleep);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,4,0)

#endif
    return -EINVAL;
}

static ssize_t device_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    // To do: wake up the sleeping process.
	bSleep = 1;
	wake_up_interruptible(&wq);
    printk ("<1>Wake up the task in wait queue\n");
    return -EINVAL;
}

static struct file_operations fops = {
    .read = device_read,
    .write = device_write,
    .open = device_open,
    .release = device_release
};

int init_module(void)
{
    Major = register_chrdev(0, DEVICE_NAME, &fops);

    if( Major <0 )
    {
      printk("<1>Registering the character device failed with %d \n", Major);
      return Major;
    }
    return 0;
}

void cleanup_module(void)
{
    unregister_chrdev(Major, DEVICE_NAME);
}
