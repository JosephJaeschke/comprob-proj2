CC = g++

CFLAGS  = -O2 -I. -I pqp/include
LDFLAGS	= -L. -L pqp/lib
LDLIBS  = -lPQP -lm      

.SUFFIXES: .cpp

SRCS    = piano.cpp

OBJECTS	= piano.o

TARGET  = piano

CLEAN   = $(OBJECTS) $(TARGET)

.cpp.o:
	$(CC) ${CFLAGS} -c $<

$(TARGET): $(OBJECTS)
	g++ -c quaternion.cpp -o quaternion.o
	$(CC) $(CFLAGS) quaternion.o -o $(TARGET) $(OBJECTS) -L. $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)
	rm quaternion.o
