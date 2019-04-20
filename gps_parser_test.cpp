#undef DISABLE_DEBUG_MSGo

#include "gps_parser.h"

#include "testcommon.h"
#include <iostream>

using namespace std;

const char k_gpgga_test[] = "";
const char k_gpgsa_test[] = "$GPGSA,A,1,,,,,,,,,,,,,,,*1E";

TEST(GPSPARSER_WORKFLOW_GPGGA){
  GPSUnit gps_unit;
  gps_parser::Parse(k_gpgsa_test, &gps_unit);
  // assert no fail
}

TEST(GPSPARSER_WORKFLOW_GPGSA){
}

void RunTests(){
  RUN_TEST(GPSPARSER_WORKFLOW_GPGGA);
  RUN_TEST(GPSPARSER_WORKFLOW_GPGSA);

  std::cout<<"All GPS Parser tests passed!"<<std::endl;
}


int main(){
  RunTests();

  return 0;
}


