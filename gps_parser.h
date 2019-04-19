#pragma once

#include <vector>
#include <string>

#include "gps_unit.h"

using std::string;
using std::vector;


namespace gps_parser{
  /* IMPORTANT     IMPORTANT
   * This fix comes with the GPS module. */
  float fix = 1.6667;

  /* http://aprs.gids.nl/nmea
   * Parse a gps sentence into gps_unit.
   */
  void Parse(const std::string& gps_msg, GPSUnit* gps_unit);

} // namespace gps_parser
