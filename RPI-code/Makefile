INCLUDE_DIRS = 
LIB_DIRS = 
CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= 

HFILES= 
CFILES= q2.c mutex.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

#all:	q2 mutex seqgen
all:	main

clean:
	-rm -f *.o *.d
	-rm -f main

q2: q2.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread

mutex: mutex.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread

seqgen: seqgen.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread

main: main.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
