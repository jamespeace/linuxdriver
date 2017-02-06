#include <linux/kernel.h>    // need for kernel alert
#include <linux/module.h>
#include <linux/init.h>         // need for __init and __exit
#include <linux/proc_fs.h>
#include <linux/timer.h>

#include <linux/interrupt.h>
#define MAX_TIMEOUT_COUNT 20

// To do: declare a timer_list


static struct timeval current_time[MAX_TIMEOUT_COUNT];
static int timeout_count;
static struct proc_dir_entry *ktimer_proc;

ssize_t ktimer_proc_fn (char *buffer, char **buffer_location, off_t off, int buffer_length, int *eof, void *data) {
  int len = 0, i;
  
  if(off > 0) {
    return len;
  }

  for( i=0; i<MAX_TIMEOUT_COUNT; i++) {
  	len += sprintf(buffer+len,"[%d].tv_sec:%lu, [%d].tv_usec:%lu\n", i, current_time[i].tv_sec, i, current_time[i].tv_usec);
  }

  return len;
}

static void timer1_expire(unsigned long data)
{
  do_gettimeofday(&current_time[timeout_count]);
  timeout_count++;
  
  if( timeout_count < MAX_TIMEOUT_COUNT ) {
    // To do: add_timer() agian

  }
}

static int __init ktimer_init(void) 
{
  printk("HZ:%d\n",HZ);
  ktimer_proc=create_proc_read_entry("ktimer", 0644, NULL, ktimer_proc_fn, NULL);
  // To do: initialize the kernel timer 

  // To do: start the timer: add_timer()

  return 0;
}

static void __exit ktimer_exit(void) 
{
  remove_proc_entry("ktimer",NULL);
  // To do: delete the pending timer

}

module_init(ktimer_init);
module_exit(ktimer_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jiunnder2000@yahoo.com.tw");
MODULE_DESCRIPTION("kernel timer example");
