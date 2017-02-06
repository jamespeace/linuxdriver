#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

int main(int argc , char *argv[])
{
  int fh, cnt=0;
  char bfr;
 
  printf("I/O test for UART driver \n");
  printf("Press Ctrl C to exit\n");
  fh = open("/dev/uart", O_RDWR);
  while(1) {  
    cnt = read(fh, &bfr, 1);
    if(cnt > 0) {
      printf("read %d characters: %c\n", cnt, bfr);
    }
  }
  close(fh);

  return 0;
}
