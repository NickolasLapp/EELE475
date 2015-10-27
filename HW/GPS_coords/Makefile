CC=gcc
CFLAGS=-Wall -I ../include
OBJ = GPS_parser.o
DEPS = GPS_parser.h
%.o: %.c
	$(CC) -ggdb -c -o $@ $< $(CFLAGS)

all-am: GPS_parser

GPS_parser: GPS_parser.c
	$(CC) -ggdb -o $(OBJ) $(DEPS) GPS_parser.c

.PHONY: clean

clean:
	rm -f *.o GPS_parser
