MAKE=make
CC=clang++
OUTPUT=log_analyzer
LIBS+=-lstdc++fs
CFLAGS=-std=c++17 -Wall
IDIR=../erkir/include

PORT?=0

SRC_FILES= log_analyzer_main.cpp \
					 log_analyzer.cpp \
					 ../gps_parser.cpp \
					 ../util.cpp 

SRC_FILES+= $(wildcard ../erkir/src/*.cpp)

OBJ=	$(patsubst %.cpp,%.o,$(SRC_FILES))

all:	main 

main: $(OBJ)
	$(CC) $(OBJ) -o $(OUTPUT) $(LIBS) $(CFLAGS)

%.o:	%.cpp
	$(CC) -o $@ -c $< $(CFLAGS) $(LIBS) -I $(IDIR)

clean:
	rm ./*.o

.PHONY:	main
