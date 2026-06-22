# Test Makefile for host-runnable unit tests
# Usage: make -C tests

CC = /nix/store/qd70v8g0561vm8m33kmnp79z00cgyi5n-gcc-wrapper-15.2.0/bin/gcc
CFLAGS = -Wall -Wextra -I. -I../src -DROBOT=1 -include rm-mr32.h

SRC = ../src/nav_graph.c
TEST = test_nav_graph.c

.PHONY: all clean

all: test_nav_graph
	./test_nav_graph

test_nav_graph: $(TEST) $(SRC)
	$(CC) $(CFLAGS) -o $@ $(TEST) $(SRC) -lm

clean:
	rm -f test_nav_graph
