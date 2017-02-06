/*
 *  uart driver exercise code copyright is owned by the authors
 *  author: seebarla@gmail.com, jared65.wu@msa.hinet.net
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
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/cdev.h>
static struct cdev mycdev;
static dev_t devno=0;
#endif

#include "ringbuf.h"

#define DEV_NUM 2

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

// To do: declare your bottom half
void wakeup_tasklet (unsigned long arg)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
  bSleep=1;
#endif
  wake_up_interruptible(&wq);
}
DECLARE_TASKLET(uart_tasklet, wakeup_tasklet, 0);

static unsigned char *prbuf, *pwbuf;
static struct ring_buffer ring;   // Declared the ring buffer pointer
static unsigned int ioports[DEV_NUM]={PORT_COM1, PORT_COM2};
static unsigned int irqs[DEV_NUM]={COM_IRQ, 3};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
static irqreturn_t uart_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t uart_interrupt(int irq, void *dev_id)
#endif
{ 
  unsigned char reg, ch, i;

  for ( i=0; i<DEV_NUM; i++ ) {
    reg = inb( ioports[i] + REG_IIR);

    if ( reg & IIR_RECEIVED_AVAILABLE ) {
      printk("uart_interrupt(): iir:%x rx interrupt\n", reg);

      while ( inb(ioports[i]+REG_LSR) & DATA_READY ) {
        ch=inb(ioports[i]);
        ring_buffer_put (&ring, ch);
      }
      // To do: schedule the bottom half
      tasklet_schedule(&uart_tasklet);
    }
    else {
      printk("uart_interrupt(): iir:%x unknown\n", reg);
    }
  }

  return IRQ_HANDLED;
}

static int uart_open (struct inode *inode, struct file *filp)
{
  struct resource *base_res;
  unsigned char reg;
  word divisor;

  // To do: get minor number

  printk("<1>uart_open(): minor:%d\n", uart_minor);

  // To do: request IO ports for different device

  // To do: configure 2400 baud rate for different device
 
  // To do: configure N, 8, 1 for different device

  // To do: request irq and enable interrupt for different device

  return 0;
}

static int uart_close(struct inode *inode, struct file *file)
{
  // To do: get minor number
  uart_minor =(MINOR(inode->i_rdev)&0x0f);
  printk("<1>uart_close(): minor:%d\n", uart_minor);

  // To do: release irq 

  // To do: release io ports

  return 0;
}

int uart_read (struct file * filp, char *buf, size_t count, loff_t *t)
{
    int rcount=0;

    if( ring_buffer_isenmpty(&ring) ) {
      // To do: wait for an event.
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
      bSleep=0;
      wait_event_interruptible(wq, bSleep);
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,4,0)
      interruptible_sleep_on(&wq);
#endif
    }

    while ( (rcount < count) && !ring_buffer_isenmpty(&ring) ) {
      *(prbuf+rcount)=ring_buffer_get(&ring);
      printk("uart_int: %c\n", *(prbuf+rcount));
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
    while( ! (inb(ioports[0]+REG_LSR) & THRE_EMPTY ) );

    while( i < count ) {
      if( inb(ioports[0]+REG_LSR) & THRE_EMPTY ) {
        outb(*(pwbuf+i), ioports[0]);
        i++;
      }
    }

    return i;
}

struct file_operations uart_fops = {
    .read=uart_read,
    .write=uart_write,
    .open=uart_open,
    .release = uart_close
};

int init_module(void)
{
  printk("<1>""uart: INIT_MOD\n");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
  if(uart_major) {
      if ( register_chrdev_region(MKDEV(uart_major,0), 1, "uart") < 0 ) {
          printk ("register_chrdev_region() fail\n");
          return -1;
      }
  }
  else {
      if (alloc_chrdev_region(&devno, 0, 1, "uart") < 0) {
          printk ("alloc_chrdev_region() fail\n");
          return -1;
      }
      uart_major=MAJOR(devno);
  }
  cdev_init(&mycdev, &uart_fops);
  mycdev.owner=THIS_MODULE;
  if(cdev_add(&mycdev, MKDEV(uart_major,0), 1)) {
      printk ("Error adding cdev\n");
      unregister_chrdev_region(MKDEV(uart_major, 0), 1);
  }
#else
  uart_major = register_chrdev(0, "uart", &uart_fops);
  if (uart_major < 0) {
     printk("<1>" "uart: can't get major number\n");
     return -1;
  }
#endif

  ring_buffer_init (&ring);
  prbuf = (unsigned char *)__get_free_page(GFP_KERNEL);
  pwbuf = (unsigned char *)__get_free_page(GFP_KERNEL);
  if ( !prbuf || !pwbuf ) {
    return -ENOMEM;
  }

  return 0;
}

void cleanup_module(void)
{
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
}



