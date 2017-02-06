#include <linux/slab.h>
#include "ringbuf.h"
 
/*  
 *  initialize the ring buffer 
 */
int ring_buffer_init (struct ring_buffer *pring)
{
  pring->head = 0;
  pring->tail = 1;
  
  return 0;
}

/*  return:
 *    0: ring buffer is not full
 *    1: ring buffer is full
 */
int ring_buffer_isfull (struct ring_buffer *pring)
{
	return ( pring->head == pring->tail ) ? 1 : 0;
}

/*  return:
 *    0: ring buffer is not empty
 *    1: ring buffer is empty
 */
int ring_buffer_isenmpty (struct ring_buffer *pring)
{
	return ( ( (pring->head + 1) % RING_SIZE ) == pring->tail ) ? 1 : 0;
}

/*  return:
 *    0: ring buffer is full, add character fail
 *    1: add character success
 */
int ring_buffer_put (struct ring_buffer *pring, unsigned char ch)
{
  if ( !ring_buffer_isfull(pring) ) {
    pring->buf[pring->tail]=ch;
    pring->tail=(pring->tail+1)%RING_SIZE;
    return 1;
  }
  return 0;
}
 
/*  return:
 *    0: ring buffer is empty, get character fail
 *    else: return the character
 */
unsigned char ring_buffer_get (struct ring_buffer *pring)
{
  unsigned char ch;

  if ( !ring_buffer_isenmpty(pring) ) {
    ch = pring->buf[ pring->head+1 ];
    pring->head=(pring->head+1)%RING_SIZE;
    return ch;
  }
  return 0;
}
