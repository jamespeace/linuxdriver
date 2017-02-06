#include<fcntl.h>	//open
#include<unistd.h>	//exit
#include<sys/ioctl.h>	//ioctl
#include<errno.h>       //errno

static char *dev="/dev/hello",message[100];

main()
{
  int file_desc;

  if( ( file_desc=open(dev,O_RDWR) ) < 0 )
  {
    printf("Can't open device file: %s\n",dev);
    exit(-1);
  }

  if ( read(file_desc,message,sizeof(message)) < 0 ) {
    printf("ioctl_get_msg failed\n");
    exit(-1);
  }
  printf("get_msg message:%s\n",message);

  if( write(file_desc,"test", 5) ) {
    perror("write fail:\n");
  }

  close(file_desc);
  return 0;
}
