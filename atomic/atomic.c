/*
 *  atomic exercise code copyright is owned by the authors
 *  author: seebarla@gmail.com, jared65.wu@gmail.com
 */

#include <linux/kernel.h>    // need for kernel alert
#include <linux/module.h>
#include <linux/init.h>         // need for __init and __exit
#include <asm/atomic.h> // for atomic_t
#include <asm/bitops.h>	// for bits operation
/*
#ifdef CONFIG_SMP
#define __SMP__
#endif
*/
static int __init hello_2_init(void) 
{
	atomic_t v;
	unsigned long flags;
	
	atomic_set(&v, 3);
	printk("1.atomic_set(&v, 3);v:%d\n",atomic_read(&v));
	atomic_dec(&v);
	printk("2.atomic_dec(&v);v:%d\n",atomic_read(&v));
	atomic_inc(&v);
	printk("3.atomic_inc(&v);v:%d\n",atomic_read(&v));
	atomic_add(3,&v);
	printk("4.atomic_add(3,&v);v:%d\n",atomic_read(&v));
	atomic_sub(1,&v);
	printk("5.atomic_sub(1,&v);v:%d\n",atomic_read(&v));


	printk("set bit 1\n");
	set_bit(1, &flags);
	if ( test_bit(1, &flags) == 1 )
		printk("bit 1 is set\n");
	else
		printk("bit 1 is clear\n");

	printk("clear bit 1\n");
	clear_bit(1, &flags);
	if ( test_bit(1, &flags) == 1 )
                printk("bit 1 is set\n");
        else
                printk("bit 1 is clear\n");


	return 0;
}

static void __exit hello_2_exit(void) 
{ 
}

module_init(hello_2_init);
module_exit(hello_2_exit);

MODULE_AUTHOR("Jared");
MODULE_DESCRIPTION("atomic exercise");
