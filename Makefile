CC=g++
FILE=gps_raspberry_pi_main
OUTPUT=gps_raspberry_pi
LIBS= -lpthread#-lwiringPi -lwiringPiDev
CFLAGS=-std=c++17 -Wall

SRC_FILES= gps_parser.cpp \
					 gps_raspberry_pi_main.cpp \
					 util.cpp

OBJ=	$(patsubst %.cpp,%.o,$(SRC_FILES))

all:	main clean
	echo OBJ $(OBJ)

main: $(OBJ)
	$(CC) $(OBJ) -o $(OUTPUT) $(LIBS) $(CFLAGS)

%.o:	%.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm ./*.o

# Tests
gps_parser_test: gps_parser.o
	$(CC) gps_parser_test.cpp -o gps_parser_test $(CFLAGS)
	./$@
	rm ./$@

util_test: util.o
	$(CC) util_test.cpp util.o -o util_test $(CFLAGS)
	./$@
	rm $@


.PHONY:	util_test gps_parser_test main
