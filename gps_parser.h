#ifndef GPS_RASPBERRYPI_GPS_PARSER_H_
#define GPS_RASPBERRYPI_GPS_PARSER_H_

#include <vector>
#include <string>

#include "gps_unit.h"

using std::string;

namespace gps_parser{
  /* IMPORTANT     IMPORTANT
   * This fix comes with the GPS module. */
  static float fix = 1.6667;

  /* http://aprs.gids.nl/nmea
   * Parse a gps sentence into gps_unit.
   */
  void Parse(const std::string& gps_msg, GPSUnit* gps_unit);
} // namespace gps_parser


#endif // GPS_RASPBERRYPI_GPS_PARSER_H_

