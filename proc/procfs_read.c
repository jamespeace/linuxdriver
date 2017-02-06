#include <linux/module.h> /* Specifically, a module */
#include <linux/kernel.h> /* We're doing kernel work */
#include <linux/proc_fs.h> /* Necessary because we use the proc fs */
struct proc_dir_entry *Our_Proc_File;
struct proc_dir_entry *our_dir;

ssize_t procfile_read(char *buffer,char **buffer_location,off_t offset, int buffer_length, int *eof, void *data)
{
    int len = 0; /* The number of bytes actually used */
    static int count = 1;
    
    printk(KERN_INFO "inside /proc/test : procfile_read\n");
    if (offset > 0) {
        printk(KERN_INFO "offset %d : /proc/test : procfile_read, wrote %d Bytes\n", (int)(offset), len);
         len=0;
    }
    else {
    
      //Fill the buffer and get its length
      len = sprintf(buffer,"For the %d time, go away!\n", count++);
    }
    return len;
}

int init_module()
{
    int rv = 0;

    // To do: create /proc/ourdir directory

    // To do: create /proc/ourdir/test file


    printk(KERN_INFO "Trying to create /proc/ourdir/test:\n");
    if (Our_Proc_File == NULL) {
    	rv = -ENOMEM;
    	remove_proc_entry("test", our_dir);
    	printk(KERN_INFO "Error: Could not initialize /proc/ourdir/test\n");
    } else {
    	printk(KERN_INFO "Success!\n");
    }
    return rv;
}
void cleanup_module()
{
    // To do: remove /proc/ourdir/test file

    // To do: remove /proc/ourdir directory

    printk(KERN_INFO "/proc/ourdir/test removed\n");
}

