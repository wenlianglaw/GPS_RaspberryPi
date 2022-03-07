#include "../util.h"

#include <string>
#include <vector>

#include "testcommon.h"

using namespace std;

TEST(TEST_STRSPLIT) {
  string a = "a,b,c";
  auto b = StrSplit(a, ",");
  assert(b == vector<string>({"a", "b", "c"}));
}

TEST(TEST_STRSPLIT2) {
  string a = "a123";
  auto b = StrSplit(a, ".");
  assert(b == vector<string>{"a123"});
}

TEST(TEST_STRSPLIT3) {
  string a = "";
  auto b = StrSplit(a, " ");
  assert(b == vector<string>{""});
}

TEST(TEST_STRSPLIT4) {
  string a = ",,,";
  auto b = StrSplit(a, ",");
  auto cmp = vector<string>{"", "", "", ""};
  assert(b == cmp);
}

TEST(TEST_STARTWITH_EQUAL) {
  string a = "$GPGSV,1,2,3,4,,,";
  assert(StartWith(a, "$GPGSV"));

  a = "$GP";
  assert(StartWith(a, ""));
  assert(StartWith(a, "$"));
  assert(StartWith(a, "$G"));
  assert(StartWith(a, "$GP"));
}

TEST(TEST_STARTWITH_NOTEQUAL) {
  string a = "$GPGSV,1,2,3,4,,,";
  assert(!StartWith(a, "$GPGSA"));

  a = "$GP";
  assert(!StartWith(a, "."));
  assert(!StartWith(a, "$GPA"));
}

void TestUtil() {
  RUN_TEST(TEST_STRSPLIT);
  RUN_TEST(TEST_STRSPLIT2);
  RUN_TEST(TEST_STRSPLIT3);
  RUN_TEST(TEST_STRSPLIT4);
  RUN_TEST(TEST_STARTWITH_EQUAL);
  RUN_TEST(TEST_STARTWITH_NOTEQUAL);

  std::cout << "All Util tests passed!" << std::endl;
}

int main() {
  TestUtil();
  return 0;
}
