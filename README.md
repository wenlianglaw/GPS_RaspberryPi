# GPS_RaspberryPi
This is an experimental project, made just for fun and learning purpose.  The GPS module works pretty well!  ~5m error.

Hardwares:
  * Adafruit Ultimate GPS HAT for Raspberry Pi
  * UART to USB cabel
  * Raspberry 3 B
  
C++ compile -std=c++1z / -std=c++17

# Setting up
Enable uart on Raspberry Pi.  
Connect GPS module to Raspberry Pi using UART-USB cabel.  
The port is then /dev/ttyUSB0.  
Using wiringpi library to open /dev/ttyUSB0, setting the baudrate to 9600 and you can receive the GPS messages now.

# Run
```
git clone https://github.com/wenlianglaw/GPS_RaspberryPi
cd GPS_RaspberryPi
make
./gps_raspberry_pi {Your Device Name} [Fix Ratio]
```
For example:  my device name on my raspberry is /dev/ttyUSB0 -- depends on which the port I plugged in.  
My GPS ratio has a const coordinate fix 1.666,  I don't know it is only my gps module or all the modules of this modle or 
other reasons, but it is 1.666.

```make util_test``` to test util library.  
```make gps_parser_test``` to test parser.


# Design
Using a global GPS message pool.  
2 threads.  The first thread receives the GPS message and insert it to the global GPS message pool.  The second thread parses
the message pool every second.

