#pragma once

#include <string>
#include <vector>

using std::string;

namespace gps_parser {
class GPSUnit {
public:
  /* GPGGA */
  float latitude_ = 0.0f;
  string NS_ = "N";
  float longitude_ = 0.0f;
  string EW_ = "E";
  string google_map_url_;
  // 225446 Time of fix 22:54:46 UTC
  string time_;
  // GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
  int quality_indicator_ = 0;
  // Number of satellites in use [not those in view]
  int num_of_satellites_ = 0;
  // Antenna altitude above/below mean sea level (geoid)
  float antena_altitude_sea_ = 0.0f;
  // Meters  (Antenna height unit)
  string antenna_altitude_unit_ = "M";
  // Geoidal separation (Diff. between WGS-84 earth ellipsoid and
  // mean sea level.  -=geoid is below WGS-84 ellipsoid)
  float geoidal_separation_ = 0.0f;
  // Meters  (Units of geoidal separation)
  string geoidal_separation_unit_ = "M";
  // Age in seconds since last update from diff. reference station
  float seconds_since_last_diff_ = 0.0f;
  // Diff. reference station ID#  x.x
  string diff_ref_station_id_;

  /* GPRMC & GPVTG */
  // A Navigation receiver warning A;
  string nav_receiver_warning_ = "A";
  // 191194       Date of fix  19 November 1994
  string date_;
  // 4916.45,N    Latitude 49 deg. 16.45 min North
  float latitude_degree_ = 0.0f;
  string latitude_deg_NS_ = "N";
  // 12311.12,W   Longitude 123 deg. 11.12 min West
  float longitude_degree_ = 0.0f;
  string longitude_deg_SW_ = "S";
  // 000.5        Speed over ground.  knots and km/h
  float speed_over_ground_knots_ = 0.0f;
  string speed_over_ground_knots_unit_ = "N";
  float speed_over_ground_kmh_ = 0.0f;
  string speed_over_ground_kmh_unit_ = "K";
  // 054.7        Course Made Good, True
  float course_made_good_ = 0.0f;
  // 020.3,E      Magnetic variation 20.3 deg East
  float magnetic_variation_ = 0.0f;
  string magnetic_variation_EW_ = "E";
  // 12 Positioning system mode indicator.
  std::string mode_ind_;

  /* GPVTG */
  // 054.7,T Track made good
  float track_made_good_ = 0.0f;
  string track_made_good_t_ = "T";

  /* GPGSA */
  /*
   *   1   ;
   *     M=Manual, forced to operate in 2D or 3D
   *     A=Automatic, 3D/2D
   */
  string manual_auto_mode_;

  /*
   * 2   ;
   *   1=Fix not available
   *   2=2D
   *   3=3D
   */
  int mode_2d_3d_ = 1;

  // TDOP (clock offset)
  float pdop_ = 0.0f;
  // HDOP (latitude/longitude)
  float hdop_ = 0.0f;
  // VDOP (altitude)
  float vdop_ = 0.0f;

  /******* GPGSV ***********/
  // https://docs.novatel.com/OEM7/Content/Logs/GPGSV.htm

  // symbol x
  int number_of_msgs_ = 1;
  // symbol x
  int msg_no_ = 1;
  // symbol xx
  int satellites_int_view_ = 0;

  // symbol xx
  // Satellite PRN number
  // GPS = 1 to 32
  // Galileo = 1 to 36
  // BeiDou = 1 to 63
  // NavIC = 1 to 14
  // QZSS = 1 to 10
  // SBAS = 33 to 64 (add 87 for PRN#s)
  // GLONASS = 65 to 96 1
  //
  // Satellite i's prn number.
  std::vector<int> satellite_prn_;

  // symbol xx
  // Elevation, degrees, 90 maximum
  //
  // Satellite i's elevation.
  std::vector<int> elevation_;

  // symbol xxx
  // Azimuth, degrees True, 000 to 359
  //
  // Satellite i's azimuth.
  std::vector<int> azimuth_;

  // symbol xx
  // SNR (C/No) 00-99 dB, null when not tracking
  //
  // Satellite i's snr.
  std::vector<int> snr_;

  // GNSS system ID. See Table: System and Signal IDs
  // (https://docs.novatel.com/OEM7/Content/Logs/GPGRS.htm#System)
  //
  // This field is only output if the NMEAVERSION is 4.11 (see the NMEAVERSION
  // command) https://docs.novatel.com/OEM7/Content/Commands/NMEAVERSION.htm
  std::string system_id_;

  // symbol *hh
  std::string check_sum_;

  void Clear() { *this = GPSUnit(); }
};
} // namespace gps_parser
