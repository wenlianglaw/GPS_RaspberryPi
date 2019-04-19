#include "util.h"
#include "testcommon.h"
#include <string>
#include <vector>

using namespace std;

TEST(TEST_STRSPLIT){
  string a = "a,b,c";
  auto b = StrSplit(a, ",");
  vector<string> eq_v = {"a","b","c"};
  assert(b == eq_v);
}

TEST(TEST_STRSPLIT2){
  string a = "a123";
  auto b = StrSplit(a, ".");
  assert( b == vector<string>{"a123"} );
}

TEST(TEST_STRSPLIT3){
  string a = "";
  auto b = StrSplit(a," ");
  assert( b == vector<string>{""});
}

TEST(TEST_STRSPLIT4){
  string a = ",,,";
  auto b = StrSplit(a,",");
  auto cmp = vector<string>{"","","",""};
  assert( b == cmp );
}

TEST(TEST_STARTWITH_EQUAL){
  string a = "$GPGSV,1,2,3,4,,,";
  assert(StartWith(a, "$GPGSV"));
}

TEST(TEST_STARTWITH_NOTEQUAL){
  string a = "$GPGSV,1,2,3,4,,,";
  assert(!StartWith(a, "$GPGSA"));
}

void TestUtil(){
  RUN_TEST(TEST_STRSPLIT);
  RUN_TEST(TEST_STRSPLIT2);
  RUN_TEST(TEST_STRSPLIT3);
  RUN_TEST(TEST_STRSPLIT4);
  RUN_TEST(TEST_STARTWITH_EQUAL);
  RUN_TEST(TEST_STARTWITH_NOTEQUAL);
}

int main(){
  TestUtil();
  return 0;
}
