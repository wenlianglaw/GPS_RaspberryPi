COMPILER=g++
FILE=1
LIB=-lwiringPi -lwiringPiDev -lpthread
CFLAG=-std=c++17 -Wall

all:	comp

comp:
	$(COMPILER) $(FILE).cpp -o $(FILE) $(CFLAG) $(LIB)
