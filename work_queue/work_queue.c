#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
// To do: include workqueue.h

#else
// To do: include tqueue.h

#endif

#include <linux/interrupt.h>

struct my_data{
	int len;
	char *buf;
	unsigned long jiffies;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)
	struct work_struct mytq_task;
#endif
} mytq_data;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)
void mytq_fun (struct work_struct *work)
#else
void mytq_fun (void *ptr)
#endif
{
  // To do: implement your defered codes

}

int init_module(void)
{
	int len = 100;
	char *buf = "Hello from mytq_fun";
	
	mytq_data.len = len ;
	mytq_data.buf = buf ;
	mytq_data.jiffies = jiffies ;
	
	printk ("queued task at jiffies = %ld\n", jiffies );

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)
	// Initialize a work queue
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
	// To do: schedule a work
#else
	// To do : queue a task
#endif	

  return 0;
}

void cleanup_module(void)
{
	printk ("I cleaned up, jiffies = %ld\n", jiffies );
}

