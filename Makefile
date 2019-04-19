CC=g++
FILE=gps_raspberry_pi_main
OUTPUT=gps_raspberry_pi
LIB=-lwiringPi -lwiringPiDev -lpthread
CFLAG=-std=c++17 -Wall

all:	test compile

compile:
	$(CC) $(FILE).cpp -o $(OUTPUT) $(CFLAG) $(LIB)

.PHONY:	util_test gps_parser_test

util_test:
	$(CC) util_test.cpp -o util_test $(CFLAG)
	./util_test


gps_parser_test:
	$(CC) gps_parser_test.cpp -o gps_parser_test $(CFLAG)
	./gps_parser_test
