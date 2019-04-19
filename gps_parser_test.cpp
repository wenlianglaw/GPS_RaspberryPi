#include "gps_parser.h"
#include "testcommon.h"
#include <iostream>

const char k_gpgga_test[] = "";
const char k_gpgsa_test[] = "";

TEST(GPSPARSER_GPGGA){
}

TEST(GPSPARSER_GPGSA){
}

void RunTests(){
  RUN_TEST(GPSPARSER_GPGGA);
  RUN_TEST(GPSPARSER_GPGSA);

  std::cout<<"All GPS Parser tests passed!"<<std::endl;
}


int main(){
  RunTests();

  return 0;
}


