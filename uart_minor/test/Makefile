TARGET1=testw_uart0
TARGET2=testr_uart1

all: $(TARGET1) $(TARGET2)

CC = gcc

CFLAGS = -g -O2 -Wall

$(TARGET1): $(TARGET1).c
	$(CC) $(TARGET1).c  -o $(TARGET1)

$(TARGET2): $(TARGET2).c
	$(CC) $(TARGET2).c  -o $(TARGET2)

clean:
	rm -f $(TARGET1) $(TARGET2)
