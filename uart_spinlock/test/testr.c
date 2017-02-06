#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

int main(int argc , char *argv[])
{
  int fh, cnt=0;
  char bfr[64];
 
  printf("I/O test for UART driver \n");
  printf("Press Ctrl C to exit\n");
  fh = open("/dev/uart", O_RDWR);
  while(1) {  
    memset(bfr, 0, sizeof(bfr));
    cnt = read(fh, bfr, sizeof(bfr));
    if(cnt > 0) {
      printf("read %d characters: %s\n", cnt, bfr);
    }
  }
  close(fh);

  return 0;
}
