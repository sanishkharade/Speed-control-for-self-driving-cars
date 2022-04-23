CC=g++
CFLAGS = -Wall -Werror
LDFLAGS = 
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video

OBJFILES = process.o capture.o main.o

TARGET = main

all: $(TARGET)

$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES) $(LDFLAGS) `pkg-config --libs opencv` $(CPPLIBS)

clean:
	rm -f $(OBJFILES) $(TARGET) *~
