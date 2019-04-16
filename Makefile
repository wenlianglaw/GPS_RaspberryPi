COMPILER=g++
FILE=gps_raspberry_pi
LIB=-lwiringPi -lwiringPiDev -lpthread
CFLAG=-std=c++17 -Wall

all:	comp

comp:
	$(COMPILER) $(FILE).cpp -o $(FILE) $(CFLAG) $(LIB)
