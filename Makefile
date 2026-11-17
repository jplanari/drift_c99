# Build host (gcc). Per a SPARC: CC=sparc-gaisler-elf-gcc (BCC) amb -O2, sense test/.
CC     ?= gcc
CSTD   = -std=c99
WARN   = -Wall -Wextra -pedantic -Werror
CFLAGS = $(CSTD) $(WARN) -O2 -Iinclude $(EXTRA)

SRC = src/hs_detect.c src/kf4.c src/tracker.c

all: test_host

test_host: test/test_host.c $(SRC)
	$(CC) $(CFLAGS) -o $@ test/test_host.c $(SRC) -lm

test: test_host
	./test_host

clean:
	rm -f test_host

.PHONY: all test clean
