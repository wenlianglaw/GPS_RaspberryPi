#pragma once

#include <string>

using std::string;

class GPSUnit{
  public:
  /* GPGGA */
  float latitude_;
  string NS_;
  float longitutde_;
  string EW_;
  string google_map_url_;
  // 225446 Time of fix 22:54:46 UTC
  string time_;
  // GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
  int quality_indicator_;
  // Number of satellites in use [not those in view]
  int num_of_satellites_;
  // Antenna altitude above/below mean sea level (geoid)
  float antena_altitude_sea_;
  // Meters  (Antenna height unit)
  string antenna_altitude_unit_;
  // Geoidal separation (Diff. between WGS-84 earth ellipsoid and
  // mean sea level.  -=geoid is below WGS-84 ellipsoid)
  float geoidal_separation_;
  // Meters  (Units of geoidal separation)
  string geoidal_separation_unit_;
  // Age in seconds since last update from diff. reference station
  float seconds_since_last_diff_;
  // Diff. reference station ID#  x.x
  string diff_ref_station_id_;

  /* GPRMC & GPVTG */
  // A Navigation receiver warning A;
  string nav_receiver_warning_;
  // 191194       Date of fix  19 November 1994
  string date_;
  // 4916.45,N    Latitude 49 deg. 16.45 min North
  float latitude_degree_;
  string latitude_deg_NS_;
  // 12311.12,W   Longitude 123 deg. 11.12 min West
  float longitude_degree_;
  string longitude_deg_SW_;
  // 000.5        Speed over ground.  knots and km/h
  float speed_over_ground_knots_;
  string speed_over_ground_knots_unit_;
  float speed_over_ground_kmh_;
  string speed_over_ground_kmh_unit_;
  // 054.7        Course Made Good, True
  float course_made_good_;
  // 020.3,E      Magnetic variation 20.3 deg East
  float magnetic_variation_;
  string magnetic_variation_EW_;

  /* GPVTG */
  // 054.7,T Track made good
  float track_made_good_;
  string track_made_good_t_;

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
  int mode_2d_3d_;

  // TDOP (clock offset)
  float pdop_;
  // HDOP (latitude/longitude)
  float hdop_;
  // VDOP (altitude)
  float vdop_;

  GPSUnit(){
    /* GPGGA */
    latitude_ = 0.0f;
    NS_ = "N";
    longitutde_ = 0.0f;
    EW_ = "E";
    google_map_url_ = "";
    // 225446 Time of fix 22:54:46 UTC
    time_ = "145902";
    // GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
    quality_indicator_ = 0;
    // Number of satellites in use [not those in view]
    num_of_satellites_ = 0;
    // Antenna altitude above/below mean sea level (geoid)
    antena_altitude_sea_ = 0.0f;
    // Meters  (Antenna height unit)
    antenna_altitude_unit_ = "M";
    // Geoidal separation (Diff. between WGS-84 earth ellipsoid and
    // mean sea level.  -=geoid is below WGS-84 ellipsoid)
    geoidal_separation_ = 0.0f;
    // Meters  (Units of geoidal separation)
    geoidal_separation_unit_ = "M";
    // Age in seconds since last update from diff. reference station
    seconds_since_last_diff_ = 0.0f;
    // Diff. reference station ID#  x.x
    diff_ref_station_id_ = "";

    /* GPRMC & GPVTG */
    // A Navigation receiver warning A = OK, V = warning
    nav_receiver_warning_ = "A";
    // 191194       Date of fix  19 November 1994
    date_ = "010119";
    // 4916.45,N    Latitude 49 deg. 16.45 min North
    latitude_degree_ = 0.0f;
    latitude_deg_NS_ = "N";
    // 12311.12,W   Longitude 123 deg. 11.12 min West
    longitude_degree_ = 0.0f;
    longitude_deg_SW_ = "S";
    // 000.5        Speed over ground.  knots and km/h
    speed_over_ground_knots_ = 0.0f;
    speed_over_ground_knots_unit_ = "N";
    speed_over_ground_kmh_ = 0.0f;
    speed_over_ground_kmh_unit_ = "K";
    // 054.7        Course Made Good, True
    course_made_good_ = 0.0f;
    // 020.3,E      Magnetic variation 20.3 deg East
    magnetic_variation_ = 0.0f;
    magnetic_variation_EW_ = "E";

    /* GPVTG */
    // 054.7,T Track made good
    track_made_good_ = 0.0f;
    track_made_good_t_ = "T";

    /* GPGSA */
    /* 
     *   1    = Mode:
     *     M=Manual, forced to operate in 2D or 3D
     *     A=Automatic, 3D/2D
     */
    manual_auto_mode_ = "M";

    /*
     * 2    = Mode:
     *   1=Fix not available
     *   2=2D
     *   3=3D
     */
    mode_2d_3d_ = 1;

    // TDOP (clock offset)
    pdop_ = 0.0f;
    // HDOP (latitude/longitude)
    hdop_ = 0.0f;
    // VDOP (altitude)
    vdop_ = 0.0f;
  }

  void Clear(){
    *this = GPSUnit();
  }
};


