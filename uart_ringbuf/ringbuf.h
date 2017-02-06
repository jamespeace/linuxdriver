#ifndef __RINGBUF__
#define __RINGBUF__

#define RING_SIZE 256
 
struct ring_buffer
{
  unsigned char buf[RING_SIZE];
  unsigned int tail;
  unsigned int head;
};

/*  
 *    initialize the ring buffer
 */
int ring_buffer_init (struct ring_buffer *pring);


/*  return:
 *    0: ring buffer is not empty
 *    1: ring buffer is empty
 */
int ring_buffer_isempty (struct ring_buffer *pring);

/*  return:
 *    0: ring buffer is not full
 *    1: ring buffer is full
 */
int ring_buffer_isfull (struct ring_buffer *pring);

/*  return:
 *    0: ring buffer is full, add character fail
 *    1: add character success
 */
int ring_buffer_put (struct ring_buffer *pring, unsigned char ch);

/*  return:
 *    0: ring buffer is empty, get character fail
 *    else: return the character
 */
unsigned char ring_buffer_get (struct ring_buffer *pring);


#endif // end of #ifndef __RINGBUF__

