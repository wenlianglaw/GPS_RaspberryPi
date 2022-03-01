# GPS\_RaspberryPi
This is an experimental project, made just for fun and learning purpose.  The GPS module works pretty well!  ~5m error.

Hardwares:
  * Adafruit Ultimate GPS HAT for Raspberry Pi
  * UART to USB cabel
  * Raspberry 3 B
  
C++ compile -std=c++1z / -std=c++17

# Setting up
Enable uart on Raspberry Pi.  
Connect GPS module to Raspberry Pi using UART-USB cabel.  GPS module RX to USB TX.  GPS TX to USB RX.
The port is then /dev/ttyUSB0 or /dev/ttyUSB1, depends on which USB you connect.

# Run

```
git clone https://github.com/wenlianglaw/GPS_RaspberryPi
cd GPS_RaspberryPi
make
./gps_raspberry_pi {Your Device Name}
```

For example:

```
./gps_raspberry_pi {Your Device Name}
```

```make tests``` to run tests.

# Design
There are 2 threads.  One receives the GPS messages and one parse the messages.
The GPS raw message will be stored into a log file.
