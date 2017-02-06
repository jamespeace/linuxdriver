TARGET1=testw
TARGET2=testr

all: $(TARGET1) $(TARGET2)

CC = gcc

CFLAGS = -g -O2 -Wall

$(TARGET2): $(TARGET2).c
	$(CC) $(TARGET2).c  -o $(TARGET2)

$(TARGET1): $(TARGET1).c
	$(CC) -D__LINE_BUF__ $(TARGET1).c  -o $(TARGET1)_line
	$(CC) $(TARGET1).c  -o $(TARGET1)

clean:
	rm -f $(TARGET1) $(TARGET1)_line $(TARGET2)
