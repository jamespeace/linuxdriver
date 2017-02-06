//procfs.c  create a "file" in /proc, which allows both input and output.
#include <linux/version.h>
#include <linux/kernel.h> /* We're doing kernel work */
#include <linux/module.h> /* Specifically, a module */
#include <linux/proc_fs.h> /* Necessary because we use proc fs */
#include <asm/uaccess.h> /* for get_user and put_user */
#include <linux/sched.h>
/*
* Here we keep the last message received, to prove
* that we can process our input
*/
#define MESSAGE_LENGTH 80
static char Message[MESSAGE_LENGTH];
static struct proc_dir_entry *Our_Proc_File;
#define PROC_ENTRY_FILENAME "rw_test"

// struct file *filp: see include/linux/fs.h 
// char *buffer: buffer to fill with data 
// loff_t * offset: length of the buffer
static ssize_t module_output(struct file *filp, char *buffer, size_t length, loff_t * offset)
{
    static int finished = 0;
    int i;
    char message[MESSAGE_LENGTH + 30];
// We return 0 to indicate end of file, that we have no more information. 
// Otherwise, processes will continue to read from us in an endless loop.

    if (finished) {
        finished = 0;
        return 0;
    }

// We use put_user to copy the string from the kernel's
// memory segment to the memory segment of the process
// that called us. get_user, BTW, is
// used for the reverse.

    sprintf(message, "Last input:%s", Message);
    for (i = 0; i < length && message[i]; i++)
        put_user(message[i], buffer + i);

// Notice, we assume here that the size of the message
// is below len, or it will be received cut. In a real
// life situation, if the size of the message is less
// than len then we'd return len and on the second call
// start filling the buffer with the len+1'th byte of
// the message.

    finished = 1;
    return i; // Return the number of bytes "read"
}
static ssize_t module_input(struct file *filp, const char *buff, size_t len, loff_t * off)
{
  int i;

// Put the input into Message, where module_output
// will later be able to use it

  for (i = 0; i < MESSAGE_LENGTH - 1 && i < len; i++)
    get_user(Message[i], buff + i);
  Message[i] = '\0'; // we want a standard, zero terminated string
  return i;
}
// This function decides whether to allow an operation (return zero) or not allow 
// it (return a non.zero which indicates why it is not allowed).
//
// The operation can be one of the following values:
// 0 . Execute (run the "file" . meaningless in our case)
// 2 . Write (input to the kernel module)
// 4 . Read (output from the kernel module)
//
// This is the real function that checks file
// permissions. The permissions returned by ls .l are
// for referece only, and can be overridden here.
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
static int module_permission(struct inode *inode, int op, struct nameidata *foo)
#else
static int module_permission(struct inode *inode, int op)
#endif
{
// We allow everybody to read from our module, but
// only root (uid 0) may write to it
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
    if (op == 4 || (op == 2 && current->euid == 0))
      return 0;
    return -EACCES;
#else
    /* Look at user permissions */
    int mode = ( inode->i_mode >> 6 );

    printk(" op & 0x07 :%x, %x, %x\n", (op & 0x07),  MAY_READ, MAY_WRITE);
    if ( ((op & 0x07) & MAY_READ ) || ( current_euid() == 0 && ( op & 0x07) & MAY_WRITE) )
    	return 0;

    return -EACCES;
#endif
// If it's anything else, access is denied
}
// The file is opened . we don't really care about
// that, but it does mean we need to increment the
// module's reference count.
int module_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
	try_module_get(THIS_MODULE);
#else
	MOD_INC_USE_COUNT;
#endif
    return 0;
}
// The file is closed . again, interesting only because
// of the reference count.
int module_close(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    module_put(THIS_MODULE);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,4,0)
    MOD_DEC_USE_COUNT;
#endif
    return 0;
}
static struct file_operations File_Ops_4_Our_Proc_File = {
    .read = module_output,
    .write = module_input,
    .open = module_open,
    .release = module_close,
};
// Inode operations for our proc file. We need it so
// we'll have some place to specify the file operations
// structure we want to use, and the function we use for
// permissions. It's also possible to specify functions
// to be called for anything else which could be done to
// an inode (although we don't bother, we just put NULL).
static struct inode_operations Inode_Ops_4_Our_Proc_File = {
    .permission = module_permission, /* check for permissions */
};
// Module initialization and cleanup
int init_module()
{
    int rv = 0;
    Our_Proc_File = create_proc_entry(PROC_ENTRY_FILENAME, 0644, NULL);
    Our_Proc_File->proc_iops = &Inode_Ops_4_Our_Proc_File;
    Our_Proc_File->proc_fops = &File_Ops_4_Our_Proc_File;
    Our_Proc_File->mode = S_IFREG | S_IRUGO | S_IWUSR;
    Our_Proc_File->uid = 0;
    Our_Proc_File->gid = 0;
    if (Our_Proc_File == NULL) {
        rv = -ENOMEM;       //remove_proc_entry(PROC_ENTRY_FILENAME, &proc_root);
        remove_proc_entry(PROC_ENTRY_FILENAME, NULL);
        printk(KERN_INFO "Error: Could not initialize /proc/test\n");
    }
    return rv;
}
void cleanup_module()
{
    remove_proc_entry(PROC_ENTRY_FILENAME, NULL);
}
