#include <iostream>
#include <thread>
#include <chrono>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <vector>
#include "util.hpp"
#include <ctime>
#include <time.h>
#include <mutex>

using namespace std;


std::mutex g_mutex;

vector<string> g_gps_sentence_pool;

// http://aprs.gids.nl/nmea
void ParseGPS(const string& gps){
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

    } else if( StartWith(gps, "$GPRMC") ){
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

    } else if( StartWith(gps, "$GPGSA") ){
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

    } else if( StartWith(gps, "$GPGGA") ){
        cout<<"Parsing:"<<endl<<gps<<endl;
        PrintContainer(words);
        Print(INFO, "words.size=", words.size());
        int i = 1;
        // i = 1 Time
        string gps_time = words[i++];
        // i = 2,3  Latitude, longitude
        string latitude = words[i++], NE = words[i++];
        // i = 4,5
        string longitude = words[i++], SW = words[i++];
        // 0 = INVALID
        // 1 = GPS fix
        // 2 = DGPS fix
        // i = 6
        int fix_quality = ParseNumberi(words[i++]);
        int number_of_satellites = ParseNumberi(words[i++]);
        Print(DEBUG, "i==", i);
        // Horizontal Dilution of Precision (HDOP)
        float hdop = ParseNumberf(words[i++]);
        float altitude = ParseNumberf(words[i++]);
        Print(DEBUG, "i==", i);
        // i = 10. Unit
        string altitute_unit = words[i++];
        Print(DEBUG, "i==", i);
        // Height of geoid above WGS85 ellipsoid
        float height = ParseNumberf(words[i++]);
        string height_uniq = words[i++];
        Print(DEBUG, "i==", i);
        // Time since last DGPS update
        // DGPS reference station id and DGPS ref.statio.id  format x.x
        string dgps_station = words[i++];
        Print(DEBUG, "i==", i);
        // i = 14
        string check_sum = words[i++];
        Print(DEBUG, "i==", i);

        // time
        struct tm parsed_tm;
        gps_time.insert(2,":");
        gps_time.insert(5,":");
        strptime(gps_time.c_str(), "%H:%M:%S", &parsed_tm);

        auto ca_time = parsed_tm;
        ca_time.tm_hour = (ca_time.tm_hour - 7)%24;
        Print(INFO, "UTC time is ", AscGpsTime(&parsed_tm));
        Print(INFO, "CA time is ", AscGpsTime(&ca_time));

        // long and lat
        if( longitude.size() && latitude.size()){
            string google_map_url = "www.google.com/maps/place/";
            auto dot_pos = latitude.find('.');
            latitude.insert(dot_pos-2,".");
            latitude.erase(dot_pos+1,1);

            dot_pos = longitude.find('.');
            longitude.insert(dot_pos-2,".");
            longitude.erase(dot_pos+1,1);

            google_map_url += latitude + NE + "+" + longitude + SW;
            Print(INFO, google_map_url);
        }
    } else{
        Print(ERROR, "Not a PGS sentence. Skip.");
    }
}


void ParseGPS_Thread(){
    while(true){
        g_mutex.lock();
        auto gps_sentence = g_gps_sentence_pool;
        g_gps_sentence_pool.clear();
        while( gps_sentence.size() ){
            string gps = gps_sentence.back();
            gps_sentence.pop_back();
            try{
                ParseGPS(gps);
            }catch(exception e){
                Print(ERROR, "ParseGPS_Thread");
                cout<<e.what()<<endl;
                exit (-1);
            }
        }
        g_mutex.unlock();
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

    try{
        thread receive_gps( ReceiveGPS_Thread, fd );
        thread parse_gps( ParseGPS_Thread );
        receive_gps.join();
        parse_gps.join();
    }catch(exception e){
        Print(ERROR, "Main exception");
        cout<<e.what()<<endl;
    }

    cout<<"Exiting..."<<endl;
    cout<<"Exiting..."<<endl;
    cout<<"Exiting..."<<endl;
    serialClose(fd);

    return 0;
}
