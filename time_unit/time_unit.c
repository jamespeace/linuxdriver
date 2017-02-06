#include <linux/kernel.h>    // for kernel alert
#include <linux/module.h>
#include <linux/init.h>         // for __init and __exit
#include <linux/delay.h>  // for mdelay, udelay
#include <linux/sched.h>  // for jiffies


static int __init time_unit_init(void) 
{ unsigned long jiffies_a, jiffies_b;
  
  jiffies_a= jiffies;
  jiffies_b= jiffies + HZ/10 ;
  printk("<1> jiffies: %lu\n", jiffies_a);
  printk("<1> HZ:%d, jiffies_b:jiffies+HZ/10:%lu\n", HZ, jiffies_b );
  mdelay(100);
  if( time_before(jiffies, jiffies_b) )
    printk(" jiffies:%lu < jiffies_b :%lu\n", jiffies, jiffies_b);
  else
    printk(" jiffies:%lu >= jiffies_b:%lu\n", jiffies, jiffies_b);

  return 0;
}

static void __exit time_unit_exit(void) 
{ 
  printk("<1>Bye Bye, jiffies:%lu\n", jiffies); 
}

module_init(time_unit_init);
module_exit(time_unit_exit);

