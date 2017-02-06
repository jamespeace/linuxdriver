#include <linux/kernel.h>  // need for kernel alert
#include <linux/module.h>
#include <linux/init.h>    // need for __init and __exit
#include <linux/mm.h>      // need for page allocation
#include <linux/slab.h>    // need for memory allocation
#include <asm/string.h>    // need for string operations
#include <linux/vmalloc.h> //need for vmalloc

char *pBuf1, *pBuf2;
unsigned long pFreeBuf;

static int __init kma_init(void) 
{
  pFreeBuf = __get_free_page(GFP_KERNEL);

  strcpy( (char *)pFreeBuf, "Copy to Buf!\n");
  printk("strlen(pFreeBuf):%d,pFreeBuf:%s\n", strlen((char *)pFreeBuf), (char*)pFreeBuf);
  
  pBuf1 = kmalloc( 40, GFP_KERNEL);
  memset( pBuf1, 0, sizeof(pBuf1));
  strcat( pBuf1, (char *)pFreeBuf);
  printk("strlen(pBuf1):%d,pBuf1:%s\n", strlen((char *)pBuf1), (char*)pBuf1);

  pBuf2 = vmalloc(40);
  sprintf(pBuf2,"%s",pBuf1);
  printk("strlen(pBuf2):%d,pBuf2:%s\n", strlen((char *)pBuf2), (char*)pBuf2);

  return 0;
}

static void __exit kma_exit(void) 
{
  vfree(pBuf2);
  kfree(pBuf1);
  free_pages(pFreeBuf, 0);
}

module_init(kma_init);
module_exit(kma_exit);

