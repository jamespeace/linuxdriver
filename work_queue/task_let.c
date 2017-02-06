#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h> /* for put_user */
#include <linux/interrupt.h>

#define DEVICE_NAME "taskQ" 


void mytl_fun (unsigned long t)
{
  // To do: implement your defered codes
}

DECLARE_TASKLET(my_tasklet, mytl_fun, 0);

int init_module(void)
{
	printk ("queued task at jiffies = %ld\n", jiffies );

	// To do : schedule a tasklet


  return 0;
}

void cleanup_module(void)
{
	printk ("I cleaned up, jiffies = %ld\n", jiffies );
}

