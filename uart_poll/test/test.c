#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

int main(int argc , char *argv[])
{
  int fh, cnt=0, ret, ndevs;
  char bfr[64];
  fd_set	readset;
 
  printf("Poll the uart driver and stdin \n");
  printf("Press Ctrl C to exit\n");
  fh = open("/dev/uart", O_RDWR);
  if ( fh == -1 ) {
    printf("open /dev/uart fail\n");
    exit(0);
  }
  while(1) {  
    FD_ZERO( &readset );
    FD_SET( fh, &readset );
    FD_SET( STDIN_FILENO, &readset );
    ndevs = 1+fh;
    ret = select( ndevs, &readset, NULL, NULL, NULL );
    if ( ret <= 0 ) {
      printf( "Error occured!\n");
      break;
    }
    else {
      if ( FD_ISSET( fh, &readset ) ) {
        memset(bfr, 0, sizeof(bfr));
        cnt = read(fh, bfr, sizeof(bfr));
        printf("read %d characters: %s\n", cnt, bfr);
      }
      else if ( FD_ISSET( STDIN_FILENO, &readset ) ) {
        scanf ("%s",bfr);
        write(fh, bfr, strlen(bfr)+1);
      }
    }
  }
  close(fh);

  return 0;
}
