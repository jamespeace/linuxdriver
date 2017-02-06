MODCFLAGS = -Wall -O2 -D__KERNEL__ -DMODULE -isystem /usr/src/linux-2.4.18-3/include
CC = gcc
TARGET = 5896l2
TARGET1 = 5896l3

all:  $(TARGET1).o

$(TARGET).o:$(TARGET).c
	$(CC) $(MODCFLAGS) -c $(TARGET).c

$(TARGET1).o:$(TARGET1).c
	$(CC) $(MODCFLAGS) -c $(TARGET1).c

clean:
	rm -rf $(TARGET).o $(TARGET1).o
