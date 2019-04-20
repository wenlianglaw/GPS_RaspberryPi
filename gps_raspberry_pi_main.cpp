//#define DISABLE_INFO_MSG
//#define DISABLE_DEBUG_MSG

#undef DISABLE_INFO_MSG

#include "util.h" 
#include "gps_parser.h"

#include <iostream>
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

using namespace std;

std::mutex g_mutex;
vector<string> g_gps_sentence_pool;

void ParseGPS_Thread(){
  Print(DEBUG, "Starting ParseGPS_Thread");
  GPSUnit gps_unit;
  Print(DEBUG, "Parsing...");
  while(true){
    g_mutex.lock();
    auto gps_sentence_pool = g_gps_sentence_pool;
    g_gps_sentence_pool.clear();
    g_mutex.unlock();

    while( gps_sentence_pool.size() ){
      string gps_sentence = gps_sentence_pool.back();
      gps_sentence_pool.pop_back();
      try{
        Print(INFO, "thread:Parse sentence:", gps_sentence);
        gps_parser::Parse(gps_sentence, &gps_unit);
      }catch(exception e){
        Print(ERROR, "ParseGPS_Thread");
        cout<<e.what()<<endl;
        exit (-1);
      }
    }
    std::this_thread::sleep_for(chrono::seconds(1));
  }
}

void ReceiveGPS_Thread( int fd ){
  Print(DEBUG, "Starting ReceiveGPS_Thread");
  string buffer;
  while(true){
    try{
      if(::serialDataAvail(fd)){
        int ch = ::serialGetchar(fd);
        if( ch >= 0 && ch <= 128 ){
          if( (char)ch == '$' && buffer.size() ){
            g_mutex.lock();
            g_gps_sentence_pool.push_back(std::move(buffer));
            g_mutex.unlock();
          }
          buffer.push_back(ch);
        }
      }
    }catch( exception e){
      Print(ERROR, "ReceiveGPS_Thread");
      cout<<e.what()<<endl;
      exit(-1);
    }
  }
}

void Help(){
  cout<<"./gps_raspberry_pi device [fix]"<<endl;
  cout<<"e.g\n\traspberry_pi /dev/ttyUSB0 1.667"<<endl;
}

int main(int argc, char**argv){
  wiringPiSetup();
  Help();
  if(argc <2)  return 1;
  if(argc==3) gps_parser::fix = stof(string(argv[2]));

  int fd = serialOpen(argv[1], 9600);
  if( fd < 0 ) { cout<<"Cannot open serial"<<endl; return 1; }

  try{
    thread receive_gps( ReceiveGPS_Thread, fd );
    thread parse_gps(ParseGPS_Thread);
    receive_gps.join();
    parse_gps.join();
  }catch(exception e){
    Print(ERROR, "Main exception");
    cout<<e.what()<<endl;
    exit(-1);
  }

  cout<<"Exiting..."<<endl;
  serialClose(fd);

  return 0;
}
