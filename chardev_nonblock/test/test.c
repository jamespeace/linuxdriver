#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<stdlib.h>
#include<errno.h>

#define MAX_BYTES 80

main(int argc, char *argv[])
{
  int fd;
  size_t bytes;
  char buffer[MAX_BYTES];

  if (argc != 2) {
    printf("Usage: %s <filename>\n");
    puts("Reads the content of a file, but doesn't wait for input\n");
    exit(-1);
  }
#ifdef NONBLOCK
  fd=open(argv[1], O_RDONLY|O_NONBLOCK);
#else
  fd=open(argv[1], O_RDONLY);
#endif
  if( fd == -1 ) {
    if( errno ==EAGAIN )
      printf("Open would block\n");
    else
      printf("Open failed\n");
    exit(-1);
  }

  do {
    int i;

    bytes = read(fd, buffer, MAX_BYTES);
    if( bytes == -1 ) {
      if( errno == EAGAIN )
        printf("Normally I'd block, but you told me not to\n");
      else
        printf("Another read error\n");
      exit(-1);
    }

    if( bytes > 0 ) {
      for( i=0; i<bytes; i++)
        putchar(buffer[i]);
    }
  } while ( bytes > 0 );

}
