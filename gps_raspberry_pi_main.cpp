#include "util.h" 
#include "gps_parser.h"
#include "file_writer.h"

#include <iostream>
#include <ostream>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include <time.h>
#include <chrono>
#include <vector>
#include <ctime>
#include <time.h>
#include <mutex>
#include <fstream>
#include <queue>
#include <iomanip>
#include <condition_variable>
#include <sstream>

using namespace std;

std::mutex g_mutex;
std::condition_variable g_cv;
std::vector<string> g_gps_sentence_pool;

// Last received stamp
// Default: 24 hours future.
std::chrono::time_point<std::chrono::system_clock> g_last_reived_stamp;

// Max time for not receiving data in seconds.
int g_max_time_not_receive_data = 2;

// Not receiving data error str. 
constexpr char NOT_RECEIVE_DATA_ERR[] =
              "Hasn't recieved GPS data for 2 seconds.";


void InitLastReceivedStamp() {
  g_last_reived_stamp = 
    std::chrono::system_clock::now() + std::chrono::hours(24);
}

void RefreshLastReceivedStamp() {
  g_last_reived_stamp = std::chrono::system_clock::now();
}

bool NotReceivedDataForXSeconds(int seconds) {
  auto now = std::chrono::system_clock::now();
  return now - g_last_reived_stamp  > std::chrono::seconds(seconds);
}

int ConnectToSerialPort(const std::string& port) {
  int fd = ::serialOpen(port.c_str(), 9600);
  return fd;
}

void ParseGPS_Thread(){
  Print(DEBUG, "Starting ParseGPS_Thread");

  gps_parser::GPSUnit gps_unit;
  gps_parser::GPSParser parser;

  gps_parser::FileWriter file_writer; 

  while(true) {
    std::unique_lock<std::mutex> lk(g_mutex);
    g_cv.wait(lk, []{return !g_gps_sentence_pool.empty();});
    auto gps_sentence_pool = g_gps_sentence_pool;
    g_gps_sentence_pool.clear();
    lk.unlock();
    while( gps_sentence_pool.size() ){
      string gps_statement = gps_sentence_pool.back();
      gps_sentence_pool.pop_back();
      try{
        Print(DEBUG, "thread:Parse GPS:", gps_statement);
        bool parsed = parser.Parse(gps_statement, &gps_unit);
        if (parsed) {
          file_writer.WriteRawMessage(gps_statement);
        } else {
          Print(ERROR, "Not parsed ", gps_statement);
        }
      } catch(...){
        std::cerr << "Failed to parse a GPS message. Will continue."  << std::endl;
      }
    }
  }
}

void ReceiveGPS_Thread(const std::string& port){
  Print(DEBUG, "Starting ReceiveGPS_Thread");
  string buffer;
  int fd = ConnectToSerialPort(port);
  InitLastReceivedStamp();

  auto handle_exception = [&](const auto& e) {
      std::cerr << e.what() << std::endl;
      std::cerr << "Will restart in 1 seconds." << std::endl;

      std::this_thread::sleep_for(std::chrono::seconds(1));
      serialClose(fd);

      fd = ConnectToSerialPort(port);
      InitLastReceivedStamp();
      buffer.clear();
  };

  while(true) { 
    try {
      if (NotReceivedDataForXSeconds(1)) {
        throw std::runtime_error(NOT_RECEIVE_DATA_ERR);
      }
      if (fd < 0) {
        throw std::runtime_error("Cannot open serial");
      }

      if(::serialDataAvail(fd)){
        int ch = ::serialGetchar(fd);
        if( ch >= 0 && ch <= 128 ){
          if( (char)ch == '$' && buffer.size() ){
            std::unique_lock<std::mutex> lk(g_mutex);
            g_gps_sentence_pool.push_back(std::move(buffer));
            lk.unlock();
            g_cv.notify_all();
            RefreshLastReceivedStamp();
          }
          buffer.push_back(ch);
        }
      }
    } catch( const std::runtime_error& e){
      handle_exception(e);
    } catch( const std::exception& e){
      handle_exception(e);
    }
  }
  serialClose(fd);
}

void Help(){
  cout<<"./gps_raspberry_pi device"<<endl;
  cout<<"e.g\n\traspberry_pi /dev/ttyUSB0"<<endl;
  cout<<"e.g\n\traspberry_pi /dev/ttyUSB1"<<endl;
}


int main(int argc, char**argv){
  wiringPiSetup();
  Help();
  if(argc <2)  return 1;
  std::string serial_port = argv[1];

  thread receive_gps(ReceiveGPS_Thread, serial_port);
  thread parse_gps(ParseGPS_Thread);
  receive_gps.join();
  parse_gps.join();

  return 0;
}
