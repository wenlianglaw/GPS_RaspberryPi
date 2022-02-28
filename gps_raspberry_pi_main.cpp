#include "util.h" 
#include "gps_parser.h"

#include <iostream>
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
#include <condition_variable>

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
  Print(DEBUG, "Parsing...");
  while(true) {
    std::unique_lock<std::mutex> lk(g_mutex);
    g_cv.wait(lk, []{return !g_gps_sentence_pool.empty();});
    auto gps_sentence_pool = g_gps_sentence_pool;
    g_gps_sentence_pool.clear();
    lk.unlock();
    while( gps_sentence_pool.size() ){
      string gps_sentence = gps_sentence_pool.back();
      gps_sentence_pool.pop_back();
      try{
        Print(INFO, "thread:Parse sentence:", gps_sentence);
        gps_parser::Parse(gps_sentence, &gps_unit);
      }catch(exception e){
        std::cerr << "ParseGPS_Thread Failed"  << std::endl;
        cout<<e.what()<<endl;

        std::unique_lock<std::mutex> lk(g_mutex);
        g_gps_sentence_pool.clear();
        lk.unlock();
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
    try{
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
  cout<<"./gps_raspberry_pi device [error fix multiplier]"<<endl;
  cout<<"e.g\n\traspberry_pi /dev/ttyUSB0 1.667"<<endl;
  cout<<"e.g\n\traspberry_pi /dev/ttyUSB1 1.667"<<endl;
}

int main(int argc, char**argv){
  wiringPiSetup();
  Help();
  if(argc <2)  return 1;
  if(argc==3) gps_parser::fix = std::stof(string(argv[2]));

  std::string serial_port(argv[1]);

  thread receive_gps( ReceiveGPS_Thread, serial_port );
  thread parse_gps(ParseGPS_Thread);
  receive_gps.join();
  parse_gps.join();

  return 0;
}
