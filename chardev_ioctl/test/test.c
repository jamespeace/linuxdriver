#include "../chardev.h"

#include<fcntl.h>	//open
#include<unistd.h>	//exit
#include<sys/ioctl.h>	//ioctl

main()
{
  int file_desc,retval;
  char *msg= "Message passed by ioctl\n", c, message[100];

  if( ( file_desc=open(DEVICE_FILE_NAME,0) ) < 0 )
  {
    printf("Can't open device file: %s\n",DEVICE_FILE_NAME);
    exit(-1);
  }

  if ( ioctl(file_desc,IOCTL_GET_MSG,message) < 0 ) {
    printf("ioctl_get_msg failed\n");
    exit(-1);
  }
  printf("get_msg message:%s\n",message);

  c=(char)ioctl(file_desc,IOCTL_GET_NTH_BYTE, 0 );
  printf("get_0th_byte message:%c\n",c);

  if ( ioctl(file_desc,IOCTL_SET_MSG,msg) < 0) {
    printf("ioctl_get_msg failed\n");
    exit(-1);
  }
  

  if ( ioctl(file_desc,IOCTL_GET_MSG,message) < 0 ) {
    printf("ioctl_get_msg failed\n");
    exit(-1);
  }
  printf("get_msg message:%s\n",message);

  close(file_desc);
}
