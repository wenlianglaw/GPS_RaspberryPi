#include "gps_parser.h"


#include <string>
#include <time.h>
#include <chrono>
#include <vector>
#include <ctime>
#include <time.h>
#include <mutex>
#include <fstream>

#include "util.h"
#include "config.h"

using std::string;
using std::vector;

namespace gps_parser{

  class GPSParser::Impl {
    public:
      bool Parse(const string& gps_msg, GPSUnit* gps_unit); 

    private:
      bool ParseGPGSA(const std::vector<std::string>& words, GPSUnit* gps_unit);
      bool ParseGPVTG(const std::vector<std::string>& words, GPSUnit* gps_unit);
      bool ParseGPRMC(const std::vector<std::string>& words, GPSUnit* gps_unit);
      bool ParseGPGGA(const std::vector<std::string>& words, GPSUnit* gps_unit);
      bool ParseGPGSV(const std::vector<std::string>& words, GPSUnit* gps_unit);
      void ParseGpsTime(const std::string& time);
      void ParseLatAndLong(const std::string& lat, const std::string& longi,
          const std::string& NE, const std::string& SW,
          GPSUnit* gps_unit);
  };

  bool GPSParser::Parse(const string& gps_msg, GPSUnit* gps_unit){
    return pimpl_->Parse(gps_msg, gps_unit);
  }

  GPSParser::GPSParser(): pimpl_(std::make_unique<Impl>()) {}
  GPSParser::~GPSParser() = default;


  bool GPSParser::Impl::Parse(const string& gps_msg, GPSUnit* gps_unit){
        Print(INFO, gps_msg);
        auto words = StrSplit(gps_msg, ",");
        bool parsed = false;

        if( StartWith(gps_msg, "$GPVTG") ){
          parsed =  ParseGPVTG(words, gps_unit);
        } else if( StartWith(gps_msg, "$GPRMC") ) {
          parsed =  ParseGPRMC(words, gps_unit);
        } else if( StartWith(gps_msg, "$GPGSA") ) {
          parsed =  ParseGPGSA(words, gps_unit);
        } else if( StartWith(gps_msg, "$GPGGA") ){
          parsed =  ParseGPGGA(words, gps_unit);
        } else if( StartWith(gps_msg, "$GPGSV") ){
          parsed =  ParseGPGSV(words, gps_unit);
        } else{
          Print(ERROR,"-----------------------------");
          Print(ERROR, "A GPS sentence that cannot parse. Skipped.");
          Print(ERROR, gps_msg);
          Print(ERROR,"-----------------------------");
          parsed =  false;
        }
        if (!parsed) {
          Print(INFO, "The above message cannot be parsed.");
        }
        return parsed;
  }

  /*
   * https://docs.novatel.com/OEM7/Content/Logs/GPGSV.htm
    |----------+-----------+-----------------------------------------------------------|
    | field    | structure | description                                               |
    |----------+-----------+-----------------------------------------------------------|
    | 1        | $GPGSV    | Log header. For information about the log headers,        |
    |          |           | see ASCII, Abbreviated ASCII or Binary.                   |
    |----------+-----------+-----------------------------------------------------------|
    | 2        | # msgs    | Total number of messages (1-9)                            |
    |----------+-----------+-----------------------------------------------------------|
    | 3        | msg #     | Message number (1-9)                                      |
    |----------+-----------+-----------------------------------------------------------|
    | 4        | # sats    | Total number of satellites in view. May be different than |
    |          |           | the number of satellites in use (see also the GPGGA log)  |
    |----------+-----------+-----------------------------------------------------------|
    | 5        | prn       | Satellite PRN number                                      |
    |          |           | GPS = 1 to 32                                             |
    |          |           | Galileo = 1 to 36                                         |
    |          |           | BeiDou = 1 to 63                                          |
    |          |           | NavIC = 1 to 14                                           |
    |          |           | QZSS = 1 to 10                                            |
    |          |           | SBAS = 33 to 64 (add 87 for PRN#s)                        |
    |          |           | GLONASS = 65 to 96 1                                      |
    |----------+-----------+-----------------------------------------------------------|
    | 6        | elev      | Elevation, degrees, 90 maximum                            |
    |----------+-----------+-----------------------------------------------------------|
    | 7        | azimuth   | Azimuth, degrees True, 000 to 359                         |
    |----------+-----------+-----------------------------------------------------------|
    | 8        | SNR       | SNR (C/No) 00-99 dB, null when not tracking               |
    |----------+-----------+-----------------------------------------------------------|
    | ...      | ...       | Next satellite PRN number, elev, azimuth, SNR,            |
    | ...      | ...       | ...                                                       |
    | ...      | ...       | Last satellite PRN number, elev, azimuth, SNR,            |
    |----------+-----------+-----------------------------------------------------------|
    | variable | system ID | GNSS system ID. See Table: System and Signal IDs.         |
    |          |           | This field is only output if the NMEAVERSION is 4.11      |
    |          |           | (see the NMEAVERSION command)                             |
    |----------+-----------+-----------------------------------------------------------|
    | variable | *xx       | Check sum                                                 |
    |----------+-----------+-----------------------------------------------------------|
    | variable | [CR][LF]  | Sentence terinator                                        |
    |----------+-----------+-----------------------------------------------------------|
   */
  bool GPSParser::Impl::ParseGPGSV(const std::vector<std::string>& words, GPSUnit* gps_unit) {
    Print(DEBUG, "Words.size", words.size());

    if (words.size() < 8) {
      return false;
    }
    
    size_t i = 1;
    // 1 number of msgs
    std::string number_of_msgs = words[i++];
    if (number_of_msgs.empty()) {
      Print(WARNING, "No message found in GPGSV.");
      PrintContainer(WARNING, words);
      return false;
    }
    gps_unit->number_of_msgs_ = std::stoi(number_of_msgs);

    // 2 msg number
    std::string msg_no = words[i++];
    if (msg_no.empty()) {
      Print(WARNING, "No message found in GPGSV.");
      PrintContainer(WARNING, words);
      return false;
    }
    gps_unit->msg_no_ = std::stoi(msg_no);

    // 3 sats in view
      std::string sat_in_view = words[i++];
      if (!sat_in_view.empty()) {
        gps_unit->satellites_int_view_ = std::stoi(sat_in_view);
      }

    // The last word is sys id and checksum.  Previous are all satellites.
    while (i<=words.size()-1) {
      // 4 prn
      std::string prn = words[i++];
      if (!prn.empty()) {
        gps_unit->satellite_prn_.push_back(std::stoi(prn));
      }
      if (i>=words.size()-1) {
        break;
      }

      // 5: elev
      std::string elev = words[i++];
      if (!elev.empty()) {
        gps_unit->elevation_.push_back(std::stoi(elev));
      }
      if (i>=words.size()-1) {
        break;
      }

      // 6: azimuth
      std::string azimuth = words[i++];
      if (!azimuth.empty()) {
        gps_unit->azimuth_.push_back(std::stoi(azimuth));
      }
      if (i>=words.size()-1) {
        break;
      }
      
      // 7: SNR
      std::string snr = words[i++];
      if (!snr.empty()) {
        gps_unit->snr_.push_back(std::stoi(snr));
      }
    }

    // Last: (snr or system ID) and check sum
    std::vector<string> sum = StrSplit(words[i++], "*");
    if (sum.size() < 2){
      return false;
    }
    if (gps_unit->snr_.size() != gps_unit->azimuth_.size()) {
      if (!sum[0].empty()) {
        gps_unit->snr_.push_back(std::stoi(sum[0]));
      }
    } else {
      gps_unit->system_id_ = sum[0];
    }
    gps_unit->check_sum_ = "*" + sum[1];

    return true;
  }

  /*
   * $GPGSA
   * GPS DOP and active satellites
   *
   *   eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
   *   eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35
   *
   *   1    = Mode:
   *     M=Manual, forced to operate in 2D or 3D
   *     A=Automatic, 3D/2D
   *   2    = Mode:
   *     1=Fix not available
   *     2=2D
   *     3=3D
   *   3-14 = IDs of SVs used in position fix (null for unused fields)
   *   15   = PDOP
   *   16   = HDOP
   *   17   = VDOP & CheckSum
   */
  bool GPSParser::Impl::ParseGPGSA(const vector<string>& words, GPSUnit* gps_unit){
    if (words.size() < 18) {
      return false;
    }

    // 1    = Mode:
    gps_unit->manual_auto_mode_ = words[1];

    // 2    = Mode:
    const string& str_mode_2d_3d = words[2];
    if(!str_mode_2d_3d.empty()){
      gps_unit->mode_2d_3d_ = stoi(str_mode_2d_3d);
    }

    // 15   = PDOP
    const string& str_pdop = words[15];
    if(!str_pdop.empty()){
      gps_unit->pdop_ = stof(str_pdop);
    }

    // 16   = HDOP
    const string& str_hdop = words[16];
    if(!str_hdop.empty()){
      gps_unit->hdop_ = stof(str_hdop);
    }

    // 17   = VDOP & CheckSum
    std::string str_vdop_checksum = words[17];
    auto split = ::StrSplit(str_vdop_checksum, "*");
    gps_unit->vdop_ = stof(split[0]);
    if (split.size() > 1){
      gps_unit->check_sum_ = "*" + split[1];
    }
    return true;
  }

  /*
   * Track Made Good and Ground Spped.
   * eg1. $GPVTG,360.0,T,348.7,M,000.0,N,000.0,K*43
   * eg2. $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K
   *
   *   054.7,T      True track made good
   *   034.4,M      Magnetic track made good
   *   005.5,N      Ground speed, knots
   *   010.2,K      Ground speed, Kilometers per hour
   *
   *
   *   eg3. $GPVTG,t,T,,,s.ss,N,s.ss,K*hh
   *   1    = Track made good
   *   2    = Fixed text 'T' indicates that track made good is relative to true north
   *   3    = not used
   *   4    = not used
   *   5    = Speed over ground in knots
   *   6    = Fixed text 'N' indicates that speed over ground in in knots
   *   7    = Speed over ground in kilometers/hour
   *   8    = Fixed text 'K' indicates that speed over ground is in kilometers/hour
   *   9    = Checksum
   *   The actual track made good and speed relative to the ground.
   *
   *   $--VTG,x.x,T,x.x,M,x.x,N,x.x,K
   *     x.x,T = Track, degrees True
   *     x.x,M = Track, degrees Magnetic
   *     x.x,N = Speed, knots
   *     x.x,K = Speed, Km/hr
   */
  bool GPSParser::Impl::ParseGPVTG(const vector<string>& words, GPSUnit* gps_unit){
    if (words.size() < 9) {
      return false;
    }

    // 1    = Track made good
    const string& str_track_made_good = words[1];
    if(!str_track_made_good.empty()){
      gps_unit->track_made_good_ = stof(str_track_made_good);
    }
    // 2   = Fixed text 'T' indicates that track made good is relative to true north
    gps_unit->track_made_good_t_ = words[2];

    // 3    = not used
    // 4    = not used
    // 5    = Speed over ground in knots
    const string& str_speed_over_ground_kt = words[5];
    if(!str_speed_over_ground_kt.empty()){
      gps_unit->speed_over_ground_knots_ = stof(str_speed_over_ground_kt);
    }
    // 6    = Fixed text 'N' indicates that speed over ground in in knots
    gps_unit->speed_over_ground_knots_unit_ = words[6];

    // 7    = Speed over ground in kilometers/hour
    const string& str_speed_over_ground_km = words[7];
    if(!str_speed_over_ground_km.empty()){
      gps_unit->speed_over_ground_kmh_ = stof(str_speed_over_ground_km);
    }
    // 8    = Fixed text 'K' indicates that speed over ground is in kilometers/hour
    std::string check_sum = words[8];
    auto check_sum_split = ::StrSplit(check_sum, "*");

    gps_unit->speed_over_ground_kmh_unit_ = check_sum_split[0];

    // 9    = Checksum
    if (check_sum_split.size() > 1){
      gps_unit->check_sum_ = "*" + check_sum_split[1];
    }

    return true;
  }

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


   eg4. $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
   1    = UTC of position fix
   2    = Data status (V=navigation receiver warning)
   3    = Latitude of fix
   4    = N or S
   5    = Longitude of fix
   6    = E or W
   7    = Speed over ground in knots
   8    = Track made good in degrees True
   9    = UTC date
   10   = Magnetic variation degrees (Easterly var. subtracts from true course)
   11   = E or W
   12   = Positioning system mode indicator.
   13   = Checksum *xx
   */
  bool GPSParser::Impl::ParseGPRMC(const vector<string>& words, GPSUnit* gps_unit){
    if (words.size() < 13) {
      return false;
    }

    // 1 Time
    string gps_time = words[1];
    ParseGpsTime(gps_time);

    // 2. Navigation receiver warning 
    gps_unit->nav_receiver_warning_ = words[2];
    string nav_receiver_warning = gps_unit->nav_receiver_warning_;
    if( !nav_receiver_warning.empty() ){
      if(nav_receiver_warning == "V"){
        Print(DEBUG, "Nav Receiver Warning.");
      }
    }

    // 3. Lat, long
    string lat = words[3],
    ne = words[4],
    longi = words[5],
    sw = words[6];
    ParseLatAndLong(lat, longi, ne, sw, gps_unit);

    // 7. Speed over ground, Knots
    string str_speed_over_ground = words[7];
    if(!str_speed_over_ground.empty()){
      gps_unit->speed_over_ground_knots_ = stof(str_speed_over_ground);
      Print(DEBUG, "Current Speed is:", gps_unit->speed_over_ground_knots_, "knots");
    }

    // 8. Course Made Good, True
    string str_cmg = words[8];
    if(!str_cmg.empty()){
      gps_unit->course_made_good_ = stof(str_cmg);
      Print(DEBUG, "Course Made Good is:", gps_unit->course_made_good_ , "degree");
    }

    // 9. 191194       Date of fix  19 November 1994
    gps_unit->date_ = words[9];
    string date = gps_unit->date_;
    if(!date.empty()){
      int day = stoi(date.substr(0,2));
      int month = stoi(date.substr(2,2));
      int year = stoi(date.substr(4));
      Print(DEBUG, "Gps date:", day, " ", month, " ", year + 2000);
    }

    // 10. 020.3,E      Magnetic variation 20.3 deg East
    string str_magnetic_variation = words[10];
    if(!str_magnetic_variation.empty()){
      gps_unit->magnetic_variation_ = stof(str_magnetic_variation);
    }

    // 11. E or W
    gps_unit->magnetic_variation_EW_ = words[11];

    // 12.  mode ind and checksum
    // *68 mandatory checksum
    string check_sum = words[12];
    auto check_sum_split = ::StrSplit(check_sum, "*");
    gps_unit->mode_ind_ = check_sum_split[0];

    if (check_sum_split.size() > 1) {
      gps_unit->check_sum_ = "*" + check_sum_split[1];
    }
    return true;
  }

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

     Global Positioning System Fix Data. Time, position and fix related data 
     for a GPS receiver.

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
  bool GPSParser::Impl::ParseGPGGA(const vector<string>& words, GPSUnit* gps_unit){
    Print(DEBUG, "words.size=", words.size());

    if (words.size() < 15) {
      return false;
    }

    // i = 1 Time
    string gps_time = words[1];
    ParseGpsTime(gps_time);

    // Latitude, longitude
    string latitude = words[2], ne = words[3];
    string longitude = words[4], sw = words[5];

    // i = 6 fix quailty
    // 0 = INVALID
    // 1 = GPS fix
    // 2 = DGPS fix
    string str_quality_indicator = words[6];
    if( !str_quality_indicator.empty() ){
      gps_unit->quality_indicator_ = stoi(str_quality_indicator);
      switch(gps_unit->quality_indicator_){
        case 1: Print(DEBUG, "GPS fix");break;
        case 2: Print(DEBUG, "DGPS fix");break;
        default:Print(DEBUG, "Invalid fix quality");break;
      }
    }

    // 7. number of satellites
    string str_umber_of_satellites = words[7];
    if( !str_umber_of_satellites.empty()){
      gps_unit->num_of_satellites_ = stoi(str_umber_of_satellites);
      Print(DEBUG, "Satellites in use:", gps_unit->num_of_satellites_);
    }

    // 8. Horizontal Dilution of Precision (HDOP)
    string str_hdop = words[8];
    Print(DEBUG, "HDOP:", str_hdop);
    if(!str_hdop.empty()){
      gps_unit->hdop_ = stof(str_hdop);
    }
    if( !latitude.empty() && !longitude.empty() &&
        !ne.empty() && !sw.empty()){
      ParseLatAndLong(latitude, longitude, ne, sw, gps_unit); 
    }

    // 9. Altitude
    string str_altitude = words[9];
    // 10. Altitude unit
    gps_unit->antenna_altitude_unit_ = words[10];
    if( !str_altitude.empty()){
      gps_unit->antena_altitude_sea_ = stof(str_altitude);
      Print(DEBUG, "Altitude: ", gps_unit->antena_altitude_sea_,
          " ", gps_unit->antenna_altitude_unit_);
    }

    // 11. Height of geoid above WGS85 ellipsoid
    string str_geoid_sep = words[11];
    // 12. 11's unit
    gps_unit->geoidal_separation_unit_ = words[12];
    if(!str_geoid_sep.empty()){
      gps_unit->geoidal_separation_ = stof(str_geoid_sep);
      Print(DEBUG, "Height: ", gps_unit->geoidal_separation_,
          " ", gps_unit->geoidal_separation_unit_);
    }

    // 13.  Time since last DGPS update
    string str_time_since_last_dgps_update = words[13];
    if(!str_time_since_last_dgps_update.empty()){
      gps_unit->seconds_since_last_diff_ = 
        stof(str_time_since_last_dgps_update);
    }

    // 14. DGPS reference station id and DGPS ref.statio.id  format x.x
    // and check sum
    auto dgps_checksum_split = StrSplit(words[14], "*");
    gps_unit->diff_ref_station_id_ = dgps_checksum_split[0];

    if (dgps_checksum_split.size() > 1){
      gps_unit->check_sum_ = "*" + dgps_checksum_split[1];
    }
    return true;
  }

  void GPSParser::Impl::ParseGpsTime(const string& time){
    string gps_time = time;
    struct tm parsed_tm;
    strptime(gps_time.c_str(), "%H%M%S", &parsed_tm);

    auto ca_time = parsed_tm;
    const int local_time_diff = -8;

    ca_time.tm_hour = (ca_time.tm_hour + 24 + local_time_diff) % 24;
    Print(DEBUG, "UTC time is ", AscGpsTime(&parsed_tm));
    Print(DEBUG, "CA time is ", AscGpsTime(&ca_time));
  }

  void GPSParser::Impl::ParseLatAndLong(const string& lat, const string& longi,
      const string& NE, const string& SW,
      GPSUnit* gps_unit){
    string latitude = lat, longitude = longi;
    if( longitude.size() && latitude.size()) {
      string google_map_url = "www.google.com/maps/place/";
      auto dot_pos = latitude.find('.');
      latitude.insert(dot_pos-2,".");
      latitude.erase(dot_pos+1,1);
      auto words = StrSplit(latitude, ".");
      if (!words[1].empty()) {
        words[1] = std::to_string((int)(stoi(words[1])*GPS_MODULE_FIX));
      }
      latitude = words[0] + "." + words[1];
      gps_unit->latitude_ = stof(latitude);

      dot_pos = longitude.find('.');
      longitude.insert(dot_pos-2,".");
      longitude.erase(dot_pos+1,1);
      words = StrSplit(longitude, ".");
      if (!words[1].empty()) {
        words[1] = std::to_string((int)(stoi(words[1])*GPS_MODULE_FIX));
      }
      longitude = words[0] + "." + words[1];
      gps_unit->longitutde_ = stof(longitude);

      google_map_url += latitude + NE + "+" + longitude + SW;
      gps_unit->google_map_url_ = google_map_url;
      Print(DEBUG, google_map_url);
    }
  }

} // namespace gps_parser
