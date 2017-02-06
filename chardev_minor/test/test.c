#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>	//open
#include<unistd.h>	//exit
#include<sys/ioctl.h>	//ioctl

static char message[100];

main()
{
  int file_desc;
  int i;
  char dev[20];

  for ( i=0; i < 2; i++ ) {
    // open the device file, ex: /dev/memdev0
    sprintf(dev, "/dev/memdev%d", i);
  
    file_desc=open(dev, O_RDWR);
    if( file_desc <0) {
       perror("file open failk");
    }

    // write to the device file
    sprintf(message, "Write to the device:%d", i);
    write(file_desc, message, sizeof(message));

    // read from the device file
    read(file_desc, message, sizeof(message));
    printf("get_msg message:%s\n",message);

    // close file
    close(file_desc);
  }

  return 0;
}
