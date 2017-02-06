TARGET=test

all: $(TARGET)

CC = gcc

CFLAGS = -g -O2 -Wall

$(TARGET): $(TARGET).c
	$(CC) $(TARGET).c  -o $(TARGET)

clean:
	rm -f $(TARGET)
