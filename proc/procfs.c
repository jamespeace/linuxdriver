#include <linux/module.h> /* Specifically, a module */
#include <linux/kernel.h> /* We're doing kernel work */
#include <linux/proc_fs.h> /* Necessary because we use the proc fs */

//To do: declare a struct proc_dir_entry


ssize_t procfile_read(char *buffer,char **buffer_location,off_t offset, int buffer_length, int *eof, void *data)
{
  // To do: return a message to user space
  int len=0;

  return len;
}

int init_module()
{
    int rv = 0;

    // To do: create a proc entry and register its callback function
    
    return rv;
}
void cleanup_module()
{
    // To do: remove the proc entry

    printk(KERN_INFO "/proc/test removed\n");
}
