/*
 *  uart driver exercise code copyright is owned by the authors
 *  author: seebarla@gmail.com, jared65.wu@gmail.com
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h> /* printk() */
#include <linux/errno.h>  /* error codes */
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/poll.h>
#include <linux/sched.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/cdev.h>
static struct cdev mycdev;
static dev_t devno=0;
#endif

#include "ringbuf.h"

int uart_major = 0;
int uart_minor = 0;
typedef short word;

#define PORT_COM1       0x3f8
#define PORT_COM2       0x2f8
#define PORT_COM3       0x3e8
#define PORT_COM4       0x2e8
#define COM_IRQ		4

//SOME CONSTANTS FOR PROGRAMMING THE UART

#define REG_RHR         0
#define REG_THR         0
#define REG_IER         1
#define REG_IIR         2
#define REG_LCR         3
#define REG_MCR         4
#define REG_LSR         5
#define REG_MSR         6
#define REG_SCRATCH     7

//LCR-related constants
#define PARITY_NONE     0
#define PARITY_ODD      8
#define PARITY_EVEN     24
#define PARITY_MARK     20
#define PARITY_SPACE    28

#define STOP_ONE        0
#define STOP_TWO        4

#define BITS_5          0
#define BITS_6          1
#define BITS_7          2
#define BITS_8          3

#define DLR_ON          128
#define COM1_IRQ_ON     8
#define REC_IRQ_ON      1

// Define the LSR bitmap
#include <linux/version.h>
#define DATA_READY		0x01
#define READ_OVERRUN		0x02
#define PARITY_ERROR		0x04
#define FRAME_ERROR		0x08
#define BREAK_INTERRUPT		0x10
#define THRE_EMPTY		0x20
#define TRAMSMITTER_EMPTY	0x40
#define PARITY_FRAMING_ERROR	0x80

// Define the IIR bitmap
#define IIR_RECEIVED_AVAILABLE	0x04

// To do: declare a wait queue
DECLARE_WAIT_QUEUE_HEAD(wq);
static int bSleep=0;

static unsigned char *prbuf, *pwbuf;
static struct ring_buffer ring;      // Declared the ring buffer pointer

static int uart_open (struct inode *inode, struct file *filp)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        try_module_get(THIS_MODULE);
#else
        MOD_INC_USE_COUNT;
#endif
    printk("<1>uart_open():\n");
    return 0;
}

static int uart_close(struct inode *inode, struct file *file)
{
    printk("<1>uart_close():\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    module_put(THIS_MODULE);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,4,0)
    MOD_DEC_USE_COUNT;
#endif
    return 0;
}

int uart_read (struct file * filp, char *buf, size_t count, loff_t *t)
{
    int rcount=0;

    while ( (rcount < count) && !ring_buffer_isenmpty(&ring) ) {
        *(prbuf+rcount)=ring_buffer_get (&ring);
        rcount++;
    }

    if( rcount != 0 ) {
      if( copy_to_user(buf, prbuf, rcount) != 0)
        return -EFAULT;
    }

    return rcount;
}

int uart_write (struct file * filp, const char *buf, size_t count, loff_t *t)
{
    int i=0;

    if( copy_from_user(pwbuf, buf, count) != 0 ) {
      printk("uart_write(): count:%d\n", count);
      return -EFAULT;
    }
    
    printk("uart_write(): count:%d\n", count);

    // Wait until transmit holding register empty
    while( ! (inb(PORT_COM1+REG_LSR) & THRE_EMPTY ) );

    while( i < count ) {
      if( inb(PORT_COM1+REG_LSR) & THRE_EMPTY ) {
        outb(*(pwbuf+i), PORT_COM1);
        i++;
      }
    }

    return i;
}

unsigned int uart_poll (struct file *filp, struct poll_table_struct *pwait) {
  unsigned int mask=0;

  // To do: poll_wait()
     
  // To do: Check if status has been changed and return a bitmap

  return mask;                                                   
}

// To do: add uart_poll() into uart_fops
struct file_operations uart_fops = {
    .read=uart_read,
    .write=uart_write,
    .open=uart_open,
    .poll=NULL,
    .release = uart_close
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
static irqreturn_t uart_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t uart_interrupt(int irq, void *dev_id)
#endif
{ 
  unsigned char reg, ch;

  reg = inb(PORT_COM1+REG_IIR);

  if ( reg & IIR_RECEIVED_AVAILABLE ) {
    printk("uart_interrupt(): iir:%x rx interrupt\n", reg);

    while ( inb(PORT_COM1+REG_LSR) & DATA_READY ) {
      ch=inb(PORT_COM1);
      ring_buffer_put (&ring, ch);
    }

    // To do: Set bSleep and wakeup the select()/poll() method

  }
  else {
    printk("uart_interrupt(): iir:%x unknown\n", reg);
  }

  return IRQ_HANDLED;
}

int __init init_module(void)
{
    int result;
    word divisor;
    unsigned char reg;
    
    struct resource *base_res;

    printk("<1>""uart: INIT_MOD\n");
    
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    base_res = request_region(PORT_COM1,8,"uart");
    if (!base_res) 
    {
        printk("<1>""uart: can't get I/O address 0x%x\n",PORT_COM1);
        return 1;
    }
#else
    if ( check_region(PORT_COM1, 8) ) {
        printk(KERN_INFO "uart: Can't get I/O address 0x%x\n", PORT_COM1);
        return -1;
    }

    request_region(PORT_COM1, 8, "uart");
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    if(uart_major) {
        if ( register_chrdev_region(MKDEV(uart_major,0), 1, "uart") < 0 ) {
            printk ("register_chrdev_region() fail\n");
            release_region(PORT_COM1,8);
            return -1;
        }
    }
    else {
        if (alloc_chrdev_region(&devno, 0, 1, "uart") < 0) {
            printk ("alloc_chrdev_region() fail\n");
            release_region(PORT_COM1,8);
            return -1;
        }
        uart_major=MAJOR(devno);
    }
    cdev_init(&mycdev, &uart_fops);
    mycdev.owner=THIS_MODULE;
    if(cdev_add(&mycdev, MKDEV(uart_major,0), 1)) {
        printk ("Error adding cdev\n");
        unregister_chrdev_region(MKDEV(uart_major, 0), 1);
        release_region(PORT_COM1,8);
    }
#else
    uart_major = register_chrdev(0, "uart", &uart_fops);
    if (uart_major < 0) {
	printk("<1>" "uart: can't get major number\n");
        release_region(PORT_COM1,8);
        return -1;
    }
#endif

    prbuf = (unsigned char *)__get_free_page(GFP_KERNEL);
    pwbuf = (unsigned char *)__get_free_page(GFP_KERNEL);
    if ( !prbuf || !pwbuf ) {
        release_region(PORT_COM1,8);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        cdev_del(&mycdev);
        unregister_chrdev_region(MKDEV(uart_major, 0), 1);
#else
	unregister_chrdev(uart_major, "uart");
#endif
        return -ENOMEM;
    }

    outb(DLR_ON,(PORT_COM1 + REG_LCR));
    reg = inb(PORT_COM1+REG_LCR);

    printk("<1>""REG_LCR %x\n", reg); 
    divisor = 48;
    outw(divisor, PORT_COM1);
  
    outb((BITS_8 | PARITY_NONE | STOP_ONE), (PORT_COM1 + REG_LCR));
    
    result = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
    result = request_irq(COM_IRQ, uart_interrupt,SA_INTERRUPT,"uart",NULL);
#else
    result = request_irq(COM_IRQ, uart_interrupt,IRQF_DISABLED,"uart",NULL);
#endif
    
    if(result) {
    	printk("<1>""can't register irq\n");
        if( prbuf) free_page((unsigned long)prbuf);
        if( pwbuf) free_page((unsigned long)pwbuf);
        release_region(PORT_COM1,8);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        cdev_del(&mycdev);
        unregister_chrdev_region(MKDEV(uart_major, 0), 1);
#else
	unregister_chrdev(uart_major, "uart");
#endif
        return -EFAULT;
    }
    else {
    	printk("<1>""interrupt on\n");
    	outb(COM1_IRQ_ON,(PORT_COM1 + REG_MCR));
    	outb(REC_IRQ_ON,(PORT_COM1 + REG_IER));
    }

    ring_buffer_init (&ring);
    bSleep=0;
   
    return 0;
}

void __exit cleanup_module(void)
{
    free_irq(COM_IRQ, NULL);
    if( prbuf) free_page((unsigned long)prbuf);
    if( pwbuf) free_page((unsigned long)pwbuf);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    cdev_del(&mycdev);
    unregister_chrdev_region(MKDEV(uart_major, 0), 1);
#else
    /* Unregister the device */
    if ( unregister_chrdev(uart_major, "uart") < 0) 
        printk("Error in unregister_chrdev\n");
#endif
    release_region(PORT_COM1,8);
}
