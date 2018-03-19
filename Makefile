TARGET = raft

.PHONY: default all clean

default: $(TARGET)
all: default

HEADERS = $(shell find include -name '*.h')
SOURCES = $(shell find src -name '*.c')
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))

LIBS = -lpthread -lrt -lm
CC = gcc
CFLAGS = -g -Wall -Iinclude/fusion -Iinclude/platform

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)

