#include<fcntl.h>	//open
#include<unistd.h>	//exit
#include<sys/ioctl.h>	//ioctl

static char *dev="/dev/hello",message[100];

main()
{
  int file_desc;

  // open the device file, ex: /dev/hello

  // read from the device file

  printf("get_msg message:%s\n",message);

  // write to the device file

  // close file

  return 0;
}
