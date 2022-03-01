CC=g++
FILE=gps_raspberry_pi_main
OUTPUT=gps_raspberry_pi
LIBS= -lpthread -lwiringPi -lwiringPiDev
CFLAGS=-std=c++17 -Wall

SRC_FILES= gps_parser.cpp \
					 gps_raspberry_pi_main.cpp \
					 util.cpp \
					 file_writer.cpp

OBJ=	$(patsubst %.cpp,%.o,$(SRC_FILES))

all:	main 

main: $(OBJ)
	$(CC) $(OBJ) -o $(OUTPUT) $(LIBS) $(CFLAGS)

%.o:	%.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm ./*.o

# Tests
gps_parser_test: gps_parser.o util.o
	$(CC) $@.cpp $^ -o $@ $(CFLAGS)
	./$@

util_test: util.o
	$(CC) $@.cpp $^ -o $@ $(CFLAGS)
	./$@

.PHONY:	util_test gps_parser_test main
