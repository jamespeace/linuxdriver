TARGET=test

all: $(TARGET)

CC = gcc

CFLAGS = -g -O2 -Wall

$(TARGET): $(TARGET).c
	$(CC) -D__LINE_BUF__ $(TARGET).c  -o $(TARGET)_line
	$(CC) $(TARGET).c  -o $(TARGET)

clean:
	rm -f $(TARGET) $(TARGET)_line
