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
./gps_raspberry_pi /dev/ttyUSB0
```

Make and run shortcut

```make run [PORT=?]```

For example, ```make run```

Or ```make run PORT=0``` for 

```
make && ./gps\_raspberry\_pi /dev/ttyUSB0
```


```make tests``` to run tests.

# Some design logs
GPS modlue is connected to the Raspberry Pi board through the USB port `ttyUSB?`.

This program starts two threads to read and parse GPS sentenses respectively.

The read thread reads the data from the USB port.  A GPS sentence always starts with '$' and end with '[CR][LF]'.  Once it receives a full sentense it will write this sentence to a shared pool.

The parse thread parses this sentense from this shared pool.

Upon a successful parse, the raw message and the parsed message will be logged to the log files.  The default log dir is `./log/`.  For example, the default log file name for the raw message is './log/raw_{date}_{HH}-{HH+1}.log'.

(TODO)
There is a tool to parse the raw log file offline, and it will interact with some MAP APIs to visulize the routes.
