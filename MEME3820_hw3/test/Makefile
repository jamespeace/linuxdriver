AP=test

all: $(AP).c
	$(CC) -DNONBLOCK $(AP).c -o $(AP)-nonblock
	$(CC) $(AP).c -o $(AP)-block
clean:
	rm -f $(AP)-block $(AP)-nonblock
