CC=g++
FILE=gps_raspberry_pi_main
OUTPUT=gps_raspberry_pi
LIBS= -lpthread -lwiringPi -lwiringPiDev
CFLAGS=-std=c++17 -Wall

PORT?=0

SRC_FILES= gps_parser.cpp \
					 gps_raspberry_pi_main.cpp \
					 util.cpp \
					 file_writer.cpp

TEST_FOLDER= ./tests

OBJ=	$(patsubst %.cpp,%.o,$(SRC_FILES))

all:	main 

main: $(OBJ)
	$(CC) $(OBJ) -o $(OUTPUT) $(LIBS) $(CFLAGS)

%.o:	%.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm ./*.o

# Make and run
run:
	make && ./gps_raspberry_pi /dev/ttyUSB$(PORT)

# Tests
gps_parser_test:	gps_parser.o util.o
	$(CC) $(TEST_FOLDER)/$@.cpp $^ -o $@ $(CFLAGS)
	./$@

util_test:	util.o
	$(CC) $(TEST_FOLDER)/$@.cpp $^ -o $@ $(CFLAGS)
	./$@

tests:	gps_parser_test util_test

.PHONY:	util_test gps_parser_test main
