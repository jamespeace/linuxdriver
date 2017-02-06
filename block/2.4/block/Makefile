# standards
INCLUDE		= /usr/src/linux-$(shell uname -r)/include
CC 		= gcc
CFLAGS 		= -D__KERNEL__ -isystem $(INCLUDE) -DMODULE -Wall -O2

TARGET	= blk
SRC	= blk.c	

all: $(TARGET).o

clean:
	rm -f *.o *~ core
