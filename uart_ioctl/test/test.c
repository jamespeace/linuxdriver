#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <curses.h>
#include <sys/ioctl.h>
#include <termios.h>

// IOCTL
#define IOCTL_SET_BAUD _IOW(255,0,int)

int main(int argc , char *argv[])
{
  int fh, cnt=0, divisor;
  char bfr, baud_choice;
  struct termios original_t, new_t;

  // Disable line buffering
  tcgetattr(fileno(stdin), &original_t);
  memcpy(&new_t, &original_t, sizeof(new_t));
  new_t.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(fileno(stdin), TCSAFLUSH, &new_t);
  fflush(stdin);
 
  printf("UART driver: baud test program \n");
  printf("select a baud rate: \n");
  printf(" a: 57600 \n");
  printf(" b: 38400 \n");
  printf(" c: 19200 \n");
  printf(" d: 9600 \n");
  printf(" e: 4800 \n");
  printf(" else: exit program\n");
  baud_choice=getchar();

  fh = open("/dev/uart", O_RDWR);

  switch(baud_choice) {
   case 'a':
      ioctl(fh, IOCTL_SET_BAUD, 57600);
      break;
   case 'b':
      ioctl(fh, IOCTL_SET_BAUD, 38400);
      break;
   case 'c':
      ioctl(fh, IOCTL_SET_BAUD, 19200);
      break;
   case 'd':
      ioctl(fh, IOCTL_SET_BAUD, 9600);
      break;
   case 'e':
      ioctl(fh, IOCTL_SET_BAUD, 4800);
      break;
   default:
      close(fh);
      exit(0);
  }

  while(1) {  
    printf("Input char:");
    bfr=getchar();
    cnt = write(fh, &bfr, sizeof(char));
    if(cnt > 0) {
      printf("write %d characters: %c\n", cnt, bfr);
    }
  }
  close(fh);

  // Restore line buffering
  tcsetattr(fileno(stdin), TCSANOW, &original_t);

  return 0;
}
