#undef DISABLE_DEBUG_MSG

#include "gps_parser.h"

#include "testcommon.h"
#include <iostream>

using namespace std;

const char k_gpgga_test[] = "";
const char k_gpgsa_test[] = "$GPGSA,A,1,,,,,,,,,,,,,,,*1E";

// Workflow test: assert no fails.
TEST(GPSPARSER_WORKFLOW_GPGGA){
  gps_parser::GPSUnit gps_unit;
  gps_parser::Parse(k_gpgsa_test, &gps_unit);
}

// TODO
// Workflow test: assert no fails.
TEST(GPSPARSER_WORKFLOW_GPGSA){
}

void RunTests(){
  RUN_TEST(GPSPARSER_WORKFLOW_GPGGA);
  RUN_TEST(GPSPARSER_WORKFLOW_GPGSA);

  std::cout<<"All GPS Parser tests passed!"<<std::endl;
}


int main(){
  gps_parser::fix = 1.666;
  RunTests();

  return 0;
}
