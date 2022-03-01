#ifndef GPS_RASPBERRYPI_GPS_PARSER_H_
#define GPS_RASPBERRYPI_GPS_PARSER_H_

#include "gps_unit.h"

#include <vector>
#include <string>
#include <memory>


using std::string;

namespace gps_parser{
  class GPSParser {
    // This class hides a lot of private funcs and variables.
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    public:
      GPSParser();
      ~GPSParser();

      /* http://aprs.gids.nl/nmea
       * Parse a gps sentence into gps_unit.
       *
       * Returns:
       *   true if this message is parsed.
       *   false if any error occurs.
       */
      bool Parse(const std::string& gps_msg, GPSUnit* gps_unit);
  };

} // namespace gps_parser


#endif // GPS_RASPBERRYPI_GPS_PARSER_H_

