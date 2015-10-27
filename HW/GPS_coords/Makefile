CC=gcc
CFLAGS=-Wall -Wextra -Wunused -Werror -Wno-missing-field-initializers -Wno-missing-braces
OBJ = GPS_parser
DEPS = GPS_parser.h
%.o: %.c
	$(CC) -ggdb -c -o $@ $< $(CFLAGS)

all-am: GPS_parser

GPS_parser: GPS_parser.c test.c
	$(CC) -ggdb -o $(OBJ) $(DEPS) GPS_parser.c test.c $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o GPS_parser
