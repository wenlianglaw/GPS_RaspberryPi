#undef DISABLE_DEBUG_MSG

#include "../gps_parser.h"

#include "testcommon.h"
#include <iostream>

using gps_parser::GPSUnit;
using gps_parser::GPSParser;

int g_test_pass = true;

template  <typename T, typename U>
void AssertEuqal(T a, U b) {
  if (a!=b) {
    std::cout << "***Test failed: " << a << "!= " << b << std::endl;
    g_test_pass = false;
  }
}


// Workflow test: assert no fails.
TEST(GPSPARSER_GPGGA){
  GPSParser parser;
  GPSUnit gps_unit;
  std::string test_str = "$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60";
  parser.Parse(test_str, &gps_unit);
}

TEST(GPSPARSER_GPGSV1) {
  GPSParser parser;
  GPSUnit unit;

  std::string test_str = "$GPGSV,3,1,11,18,87,050,48,22,56,250,49,21,55,122,49,03,40,284,47*78";
  parser.Parse(test_str, &unit);
  AssertEuqal(unit.number_of_msgs_, 3);
  AssertEuqal(unit.msg_no_, 1);
  AssertEuqal(unit.satellites_int_view_, 11);
  AssertEuqal(unit.satellite_prn_.size(), 4);
  AssertEuqal(unit.elevation_.size(), 4);
  AssertEuqal(unit.azimuth_.size(), 4);
  AssertEuqal(unit.snr_.size(), 4);
  // satellite 0
  AssertEuqal(unit.satellite_prn_[0], 18);
  AssertEuqal(unit.elevation_[0], 87);
  AssertEuqal(unit.azimuth_[0], 50);
  AssertEuqal(unit.snr_[0], 48);
  // satellite 1
  AssertEuqal(unit.satellite_prn_[1], 22);
  AssertEuqal(unit.elevation_[1], 56);
  AssertEuqal(unit.azimuth_[1], 250);
  AssertEuqal(unit.snr_[1], 49);
  // satellite 2
  AssertEuqal(unit.satellite_prn_[2], 21);
  AssertEuqal(unit.elevation_[2], 55);
  AssertEuqal(unit.azimuth_[2], 122);
  AssertEuqal(unit.snr_[2], 49);
  // satellite 3
  AssertEuqal(unit.satellite_prn_[3], 3);
  AssertEuqal(unit.elevation_[3], 40);
  AssertEuqal(unit.azimuth_[3], 284);
  AssertEuqal(unit.snr_[3], 47);
  // sum
  AssertEuqal(unit.check_sum_, "*78");
}


TEST(GPSPARSER_GPGSV2) {
  GPSParser parser;
  GPSUnit unit;
  std::string test_str = "$GPGSV,3,3,11,09,15,107,44,14,11,196,41,07,03,173,*4D";
  parser.Parse(test_str, &unit);

  AssertEuqal(unit.number_of_msgs_, 3);
  AssertEuqal(unit.msg_no_, 3);
  AssertEuqal(unit.satellites_int_view_, 11);
  AssertEuqal(unit.satellite_prn_.size(), 3);
  AssertEuqal(unit.elevation_.size(), 3);
  AssertEuqal(unit.azimuth_.size(), 3);
  AssertEuqal(unit.snr_.size(), 2);
  // satellite 0
  AssertEuqal(unit.satellite_prn_[0], 9);
  AssertEuqal(unit.elevation_[0], 15);
  AssertEuqal(unit.azimuth_[0], 107);
  AssertEuqal(unit.snr_[0], 44);
  // satellite 1
  AssertEuqal(unit.satellite_prn_[1], 14);
  AssertEuqal(unit.elevation_[1], 11);
  AssertEuqal(unit.azimuth_[1], 196);
  AssertEuqal(unit.snr_[1], 41);
  // satellite 2
  AssertEuqal(unit.satellite_prn_[2], 7);
  AssertEuqal(unit.elevation_[2], 3);
  AssertEuqal(unit.azimuth_[2], 173);
  // sum
  AssertEuqal(unit.check_sum_, "*4D");
}

TEST(GPSPARSER_GPGSV3) {
  GPSParser parser;
  GPSUnit unit;
  std::string test_str = "$GPGSV,4,4,13,08,02,219,*4B";
  parser.Parse(test_str, &unit);

  AssertEuqal(unit.number_of_msgs_, 4);
  AssertEuqal(unit.msg_no_, 4);
  AssertEuqal(unit.satellites_int_view_, 13);
  AssertEuqal(unit.satellite_prn_.size(), 1);
  AssertEuqal(unit.elevation_.size(), 1);
  AssertEuqal(unit.azimuth_.size(), 1);
  AssertEuqal(unit.snr_.size(), 0);
  // satellite 0
  AssertEuqal(unit.satellite_prn_[0], 8);
  AssertEuqal(unit.elevation_[0], 2);
  AssertEuqal(unit.azimuth_[0], 219);
  // sum
  AssertEuqal(unit.check_sum_, "*4B");
}



// TODO
// Workflow test: assert no fails.
TEST(GPSPARSER_GPGSA){
}

void RunTests(){
  RUN_TEST(GPSPARSER_GPGGA);
  RUN_TEST(GPSPARSER_GPGSA);
  RUN_TEST(GPSPARSER_GPGSV1);
  RUN_TEST(GPSPARSER_GPGSV2);
  RUN_TEST(GPSPARSER_GPGSV3);

  if (g_test_pass) {
    std::cout<<"All GPS Parser tests passed!"<<std::endl;
  } else {
    std::cout<<"GPS Parser tests failed!  See details above."<<std::endl;
  }
}


int main(){
  RunTests();
  return 0;
}
