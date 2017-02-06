#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>         // need for __init and __exit
#include <linux/proc_fs.h>
#include <asm/uaccess.h> /* for put_user */
#define PROCFS_NAME "proc_write"

static struct proc_dir_entry *proc;
static int write_value=0; // flag, 1:start the garbage collection, 0:stop the gargabe collection

static int proc_write(struct file *file, const char *buffer, unsigned long count, void *data) {
	char tmpbuf[4];
	int len=0;
	// To do: get and parser the input string
	
	return len;

}

static int __init proc_write_init(void) 
{ 
    // To do: create /proc/proc_write file

    return 0;
}

static void __exit proc_write_exit(void) 
{ 
    // To do: remove /proc/proc_write file
}

module_init(proc_write_init);
module_exit(proc_write_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jiunnder2000@yahoo.com.tw");
MODULE_DESCRIPTION("Create /proc/proc_write sample!");
MODULE_SUPPORTED_DEVICE("no");
