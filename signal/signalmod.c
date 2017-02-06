#include <linux/version.h>
#include <linux/kernel.h>    // need for kernel alert
#include <linux/module.h>
#include <linux/init.h>         // need for __init and __exit
#include <linux/sched.h> // for send_sig()

static int user_pid = 20;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
module_param(user_pid, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
#else
MODULE_PARM(user_pid, "i");
#endif
MODULE_PARM_DESC(user_pid, "The process id which will receive the USR1 signal");

static int __init signalmod_init(void) 
{
	struct task_struct *p;
#if 0
	p=find_task_by_pid(user_pid);
#else
	p=pid_task(get_pid(find_vpid(user_pid)), PIDTYPE_PID);
	if ( p )
		get_task_struct(p);
#endif
	send_sig( SIGUSR1, p, 0);
	send_sig( SIGUSR1, p, 0);	
	return 0;
}

static void __exit signalmod_exit(void) 
{ 
}

module_init(signalmod_init);
module_exit(signalmod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jared");
MODULE_DESCRIPTION("signal to user process!");
MODULE_SUPPORTED_DEVICE("no");
