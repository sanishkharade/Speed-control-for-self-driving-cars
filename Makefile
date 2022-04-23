INCLUDE_DIRS = 
LIB_DIRS = 

CC=gcc
CPP=g++

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= 
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video

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

main: main.o process.o capture.o
	$(CPP) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lpthread `pkg-config --libs opencv` $(CPPLIBS)

	
depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
