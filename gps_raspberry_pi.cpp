#include "util.hpp"

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
#include <string_view>


using namespace std;


std::mutex g_mutex;

vector<string_view> g_gps_sentence_pool;

class GPSParser{
  public:
    // http://aprs.gids.nl/nmea
    void Parse(string_view gps){
      auto words = StrSplit(gps, ",");
      if( StartWith(gps, "$GPVTG") ){
        /* Track Made Good and Ground Spped.
           eg1. $GPVTG,360.0,T,348.7,M,000.0,N,000.0,K*43
           eg2. $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K

           054.7,T      True track made good
           034.4,M      Magnetic track made good
           005.5,N      Ground speed, knots
           010.2,K      Ground speed, Kilometers per hour


           eg3. $GPVTG,t,T,,,s.ss,N,s.ss,K*hh
           1    = Track made good
           2    = Fixed text 'T' indicates that track made good is relative to true north
           3    = not used
           4    = not used
           5    = Speed over ground in knots
           6    = Fixed text 'N' indicates that speed over ground in in knots
           7    = Speed over ground in kilometers/hour
           8    = Fixed text 'K' indicates that speed over ground is in kilometers/hour
           9    = Checksum
           The actual track made good and speed relative to the ground.

           $--VTG,x.x,T,x.x,M,x.x,N,x.x,K
           x.x,T = Track, degrees True
           x.x,M = Track, degrees Magnetic
           x.x,N = Speed, knots
           x.x,K = Speed, Km/hr
           */
      } else if( StartWith(gps, "$GPRMC") ) {
        Print(INFO, "Parsing:", gps);
        ParseGPRMC(words);
      } else if( StartWith(gps, "$GPGSA") ) {
        Print(INFO, "Parsing:", gps);
        ParseGPGSA(words);
      } else if( StartWith(gps, "$GPGGA") ){
        Print(INFO, "Parsing:", gps);
        ParseGPGGA(words);
      } else{
        Print(ERROR, "Not a PGS sentence. Skip.");
      }
    }
  private:
    void ParseGPGSA(const vector<string_view>& words){
      /*
       * $GPGSA
       GPS DOP and active satellites

       eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
       eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35


       1    = Mode:
       M=Manual, forced to operate in 2D or 3D
       A=Automatic, 3D/2D
       2    = Mode:
       1=Fix not available
       2=2D
       3=3D
       3-14 = IDs of SVs used in position fix (null for unused fields)
       15   = PDOP
       16   = HDOP
       17   = VDOP
       */
      //TODO
      int i=1;
      string_view mode = words[i++];
      string_view dim_2_3 = words[i++];
      
      
    }

    void ParseGPRMC(const vector<string_view>& words){
      /*
       * Recommended minimum specific GPS/Transit data

       eg1. $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
       eg2. $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68


       225446       Time of fix 22:54:46 UTC
       A            Navigation receiver warning A = OK, V = warning
       4916.45,N    Latitude 49 deg. 16.45 min North
       12311.12,W   Longitude 123 deg. 11.12 min West
       000.5        Speed over ground, Knots
       054.7        Course Made Good, True
       191194       Date of fix  19 November 1994
       020.3,E      Magnetic variation 20.3 deg East
       *68          mandatory checksum


       eg3. $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
       1    2    3    4    5     6    7    8      9     10  11 12


       1   220516     Time Stamp
       2   A          validity - A-ok, V-invalid
       3   5133.82    current Latitude
       4   N          North/South
       5   00042.24   current Longitude
       6   W          East/West
       7   173.8      Speed in knots
       8   231.8      True course
       9   130694     Date Stamp
       10  004.2      Variation
       11  W          East/West
       12  *70        checksum


       eg4. $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
       1    = UTC of position fix
       2    = Data status (V=navigation receiver warning)
       3    = Latitude of fix
       4    = N or S
       5    = Longitude of fix
       6    = E or W
       7    = Speed over ground in knots
       8    = Track made good in degrees True
       9    = UT date
       10   = Magnetic variation degrees (Easterly var. subtracts from true course)
       11   = E or W
       12   = Checksum
       */

      int i=1;
      // Time
      string_view gps_time = words[i++];
      ParseGpsTime(gps_time);

      // Navigation receiver warning 
      string_view nav_receiver_warning = words[i++];
      if( !nav_receiver_warning.empty() ){
        if(nav_receiver_warning == "V"){
          Print(INFO, "Nav Receiver Warning.");
        }
      }

      // Lat, long
      string_view lat = words[i++],
      ne = words[i++],
      longi = words[i++],
      sw = words[i++];
      ParseLatAndLong(lat, longi, ne, sw);

      // Speed over ground, Knots
      string_view str_speed_over_ground = words[i++];
      if(!str_speed_over_ground.empty()){
        float speed_over_ground;
        ParseNumber(str_speed_over_ground, &speed_over_ground);
        Print(INFO, "Current Speed is:", speed_over_ground);
      }

      // Course Made Good, True
      string_view str_cmg = words[i++];
      if(!str_cmg.empty()){
        float cmg;
        ParseNumber(str_cmg, &cmg);
        Print(INFO, "Course Made Good is:", cmg);
      }

      // 191194       Date of fix  19 November 1994
      string_view date = words[i++];
      if(!date.empty()){
        int day;
        ParseNumber(date.substr(0,2), &day);
        int month;
        ParseNumber(date.substr(2,4), &month);
        int year;
        ParseNumber(date.substr(4), &year);
        Print(INFO, "Gps date:", day, " ", month, " ", year + 1900);
      }

      // 020.3,E      Magnetic variation 20.3 deg East
      string_view str_magnetic_variation = words[i++];
      string_view mag_ew = to_string(words[i][0]);
      
      // *68          mandatory checksum
      string_view check_sum = words[i].substr(1);
    }

    void ParseGPGGA(const vector<string_view>& words){
      /*
         $GPGGA
         Global Positioning System Fix Data

         Name	Example Data	Description
         Sentence Identifier	$GPGGA	Global Positioning System Fix Data
         Time	170834	17:08:34 Z
         Latitude	4124.8963, N	41d 24.8963' N or 41d 24' 54" N
         Longitude	08151.6838, W	81d 51.6838' W or 81d 51' 41" W
         Fix Quality:
         - 0 = Invalid
         - 1 = GPS fix
         - 2 = DGPS fix	1	Data is from a GPS fix
         Number of Satellites	05	5 Satellites are in view
         Horizontal Dilution of Precision (HDOP)	1.5	Relative accuracy of horizontal position
         Altitude	280.2, M	280.2 meters above mean sea level
         Height of geoid above WGS84 ellipsoid	-34.0, M	-34.0 meters
         Time since last DGPS update	blank	No last update
         DGPS reference station id	blank	No station id
         Checksum	*75	Used by program to check for transmission errors
         Courtesy of Brian McClure, N8PQI.

         Global Positioning System Fix Data. Time, position and fix related data for a GPS receiver.

         eg2. $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx

         hhmmss.ss = UTC of position
         llll.ll = latitude of position
         a = N or S
         yyyyy.yy = Longitude of position
         a = E or W
         x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
         xx = number of satellites in use
         x.x = horizontal dilution of precision
         x.x = Antenna altitude above mean-sea-level
         M = units of antenna altitude, meters
         x.x = Geoidal separation
         M = units of geoidal separation, meters
         x.x = Age of Differential GPS data (seconds)
         xxxx = Differential reference station ID
         eg3. $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
         1    = UTC of Position
         2    = Latitude
         3    = N or S
         4    = Longitude
         5    = E or W
         6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
         7    = Number of satellites in use [not those in view]
         8    = Horizontal dilution of position
         9    = Antenna altitude above/below mean sea level (geoid)
         10   = Meters  (Antenna height unit)
         11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
         mean sea level.  -=geoid is below WGS-84 ellipsoid)
         12   = Meters  (Units of geoidal separation)
         13   = Age in seconds since last update from diff. reference station
         14   = Diff. reference station ID#
         15   = Checksum
         */
      PrintContainer(DEBUG, words);
      Print(DEBUG, "words.size=", words.size());
      int i = 1;
      // i = 1 Time
      string_view gps_time = words[i++];
      ParseGpsTime(gps_time);
    
     // Latitude, longitude
      string_view latitude = words[i++], NE = words[i++];
      string_view longitude = words[i++], SW = words[i++];
      ParseLatAndLong(latitude, longitude, NE, SW);

      // i = 6 fix quailty
      // 0 = INVALID
      // 1 = GPS fix
      // 2 = DGPS fix
      string_view str_fix_quality = string(words[i++]);
      if( !str_fix_quality.empty() ){
        int fix_quality;
        ParseNumber(str_fix_quality, &fix_quality);
        switch(fix_quality){
          case 1: Print(INFO, "GPS fix");
          case 2: Print(INFO, "DGPS fix");
          default:Print(INFO, "Invalid fix quality");
        }
      }

      // number of satellites
      string_view str_umber_of_satellites = words[i++];
      if( !str_umber_of_satellites.empty()){
        int number_of_satellites;
        ParseNumber(str_umber_of_satellites, &number_of_satellites);
        Print(INFO, "Satellites in use:", number_of_satellites);
      }

      Print(DEBUG, "i==", i);
      // Horizontal Dilution of Precision (HDOP)
      string_view str_hdop = words[i++];
      Print(DEBUG, "i==", i);

      // i = 10. Altitude
      string_view str_altitude = words[i++];
      string_view altitute_unit = words[i++];
      if( !str_altitude.empty()){
        float altitude;
        ParseNumber(str_altitude, &altitude);
        Print(INFO, "Altitude: ", altitude," ", altitute_unit);
      }


      Print(DEBUG, "i==", i);
      // Height of geoid above WGS85 ellipsoid
      string_view str_height = words[i++];
      string_view height_unit = words[i++];
      if(!str_height.empty()){
        float height;
        ParseNumber(str_height, &height);
        Print(INFO, "Height: ", height," ", height_unit);
      }

      Print(DEBUG, "i==", i);
      // Time since last DGPS update
      // DGPS reference station id and DGPS ref.statio.id  format x.x
      string_view dgps_station = words[i++];
      Print(DEBUG, "i==", i);

      // i = 14 Check sum
      if( i < words.size()-1 ){
        string_view check_sum = words[i++];
        Print(DEBUG, "i==", i);
      }
    }

    void ParseGpsTime(string_view time){
      string gps_time = string(time);
      struct tm parsed_tm;
      gps_time.insert(2,":");
      gps_time.insert(5,":");
      strptime(gps_time.c_str(), "%H:%M:%S", &parsed_tm);

      auto ca_time = parsed_tm;
      ca_time.tm_hour = (ca_time.tm_hour - 7)%24;
      Print(INFO, "UTC time is ", AscGpsTime(&parsed_tm));
      Print(INFO, "CA time is ", AscGpsTime(&ca_time));
    }

    void ParseLatAndLong(string_view lat, string_view longi,
                        string_view NE, string_view SW){
      string latitude = string(lat), longitude = string(longi);
      if( longitude.size() && latitude.size()){
        string google_map_url = "www.google.com/maps/place/";
        auto dot_pos = latitude.find('.');
        latitude.insert(dot_pos-2,".");
        latitude.erase(dot_pos+1,1);

        dot_pos = longitude.find('.');
        longitude.insert(dot_pos-2,".");
        longitude.erase(dot_pos+1,1);

        google_map_url.append(latitude).append(NE).append("+").append(longitude).append(SW);
        Print(INFO, google_map_url);
      }
    }
};

void ParseGPS_Thread( GPSParser* gps){
  while(true){
    g_mutex.lock();
    auto gps_sentence_pool = g_gps_sentence_pool;
    g_gps_sentence_pool.clear();
    g_mutex.unlock();

    while( gps_sentence_pool.size() ){
      string_view gps_sentence = gps_sentence_pool.back();
      gps_sentence_pool.pop_back();
      try{
        gps->Parse(gps_sentence);
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
    }
  }
}


int main(int argc, char**argv){
  wiringPiSetup();

  int fd = serialOpen("/dev/ttyUSB0", 9600);
  if( fd < 0 ) { cout<<"Cannot open serial"<<endl; return 1; }

  GPSParser gps;
  try{
    thread receive_gps( ReceiveGPS_Thread, fd );
    thread parse_gps( ParseGPS_Thread, &gps );
    receive_gps.join();
    parse_gps.join();
  }catch(exception e){
    Print(ERROR, "Main exception");
    cout<<e.what()<<endl;
  }

  cout<<"Exiting..."<<endl;
  serialClose(fd);

  return 0;
}
