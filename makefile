# include dependencies; rules below
ifeq ($(UNAME),incl.defs)
	include incl.defs
endif

# Directories
SRC=./src
BIN=./bin

CC=g++
CCFLAGS=-std=c++11
SOURCES= app_routing.cpp #StudentRecord.cpp
OBJECTS= app_routing.o #StudentRecord.o

main: $(OBJECTS)
	$(CC) $(CCFLAGS) $(OBJECTS) -o app_routing $(LIBS)

.cpp.o:
	$(CC) $(CCFLAGS) -c $<

# Type "make depend" to make dependencies
depend:
	$(CC) -M $(SOURCES) > incl.defs

run:
	./app_routing

clean:
	rm *.o
	rm main
	rm incl.defs
	rm *.tar.gz
tar:
	tar -zcvf MZRTAD001.tar.gz makefile README.txt *.cpp *.h .git