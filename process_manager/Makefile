MAKE=make
CC=clang++
OUTPUT=process_manager
LIBS= -lpthread -lwiringPi -lwiringPiDev -lstdc++fs
CFLAGS=-std=c++17 -Wall

SRC_FILES= process_manager_main.cpp

OBJ=	$(patsubst %.cpp,%.o,$(SRC_FILES))


main: $(OBJ)
	$(CC) $(OBJ) -o $(OUTPUT) $(LIBS) $(CFLAGS)

all:	main process_manager

process_manager:
	+$(MAKE) -C process_manager

# All the .o files in the current directory.
%.o:	%.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	find . -iname "*.o" -delete

# Make and run, takes parameter PORT=?0
run:
	make && ./$(OUTPUT)

.PHONY:	process_manager
