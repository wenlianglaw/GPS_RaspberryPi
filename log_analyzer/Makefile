MAKE=make
CC=clang++
FILE=gps_raspberry_pi_main
OUTPUT=log_analyzer
LIBS+=#-lstdc++fs
CFLAGS=-std=c++17 -Wall

PORT?=0

SRC_FILES= log_analyzer_main.cpp \
					 log_analyzer.cpp \
					 ../gps_parser.cpp \
					 ../util.cpp

OBJ=	$(patsubst %.cpp,%.o,$(SRC_FILES))

all:	main 

main: $(OBJ)
	$(CC) $(OBJ) -o $(OUTPUT) $(LIBS) $(CFLAGS)

%.o:	%.cpp
	$(CC) -o $@ -c $< $(CFLAGS) $(LIBS)

clean:
	rm ./*.o

.PHONY:	main
