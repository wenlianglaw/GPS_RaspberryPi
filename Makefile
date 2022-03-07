MAKE=make
CC=clang++
OUTPUT=gps_raspberry_pi
LIBS= -lpthread -lwiringPi -lwiringPiDev
CFLAGS=-std=c++17 -Wall

PORT?=0
FULL_PORT=/dev/ttyUSB$(PORT)

SRC_FILES= gps_parser.cpp \
					 gps_raspberry_pi_main.cpp \
					 util.cpp \
					 file_writer.cpp

TEST_FOLDER= ./tests

OBJ=	$(patsubst %.cpp,%.o,$(SRC_FILES))


# Default entry.
# GPS parser in current directory.
main: $(OBJ)
	$(CC) $(OBJ) -o $(OUTPUT) $(LIBS) $(CFLAGS)

all:	main log_analyzer

# Log analyzer in ./log_analyzer directory
log_analyzer:
	+$(MAKE) -C log_analyzer

# All the .o files in the current directory.
%.o:	%.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	find . -iname *.o -delete

# Make and run, takes parameter PORT=?0
run:
	make && ./$(OUTPUT) $(FULL_PORT)

# Tests
tests:	gps_parser_test util_test

gps_parser_test:	gps_parser.o util.o
	$(CC) $(TEST_FOLDER)/$@.cpp $^ -o $@ $(CFLAGS)
	./$@

util_test:	util.o
	$(CC) $(TEST_FOLDER)/$@.cpp $^ -o $@ $(CFLAGS)
	./$@


.PHONY:	util_test gps_parser_test main log_analyzer run
