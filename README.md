# GPS_RaspberryPi
This is an experimental project, made just for fun and learning purpose.  But GPS module works pretty well!

Hardwares:
  * Adafruit Ultimate GPS HAT for Raspberry Pi
  * UART to USB cabel
  * Raspberry 3 B
  
C++ compile -std=c++1z / -std=c++17

# Setting up
Enable uart on Raspberry Pi.  
Connect GPS module to Raspberry Pi using UART-USB cabel.  
The port is then /dev/USB0.  
Using wiringpi library to open /dev/USB0, setting the baudrate to 9600 and you can receive the GPS messages now.

# Design
Using a global GPS message pool.  
2 threads.  First thread receives the GPS messages and insert it to the global GPS message pool.  Second thread parse the
message pool every second.

## PDOP, HDOP and VDOP

