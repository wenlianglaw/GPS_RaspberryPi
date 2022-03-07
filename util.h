#pragma once

#include <cassert>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include "config.h"
#include "gps_unit.h"

#define Level int
#define DEBUG 0
#define INFO 1
#define WARNING 2
#define ERROR 3

#define _UNUSED_ __attribute__((unused))

using std::cout;
using std::endl;
using std::string;
using std::vector;

// Note: when word is empty always returns true.
bool StartWith(const string& str, const string& word);

// Split string |str| by |delim| and returns each word in a vector.
vector<string> StrSplit(const string& str, const string& delim);

// Convert a GPS time |tm* tm| to string.
// The difference between the built-in function is
// GPS time only contains HHMMSS. Therefore we don't
// need to print extra info, whereas the built-in fucntion prints.
string AscGpsTime(std::tm* tm);

template <typename T>
void PrintContainer(Level l, const T& container) {
#ifdef DISABLE_DEBUG_MSG
  if (l == DEBUG) return;
#endif

#ifdef DISABLE_INFO_MSG
  if (l == INFO) return;
#endif

  for (auto it = container.begin(); it != container.end(); it++) {
    cout << *it << " ";
  }
  cout << endl;
}

template <typename... T>
void Print(Level l, T... args) {
#ifdef DISABLE_DEBUG_MSG
  if (l == DEBUG) return;
#endif

#ifdef DISABLE_INFO_MSG
  if (l == INFO) return;
#endif

  (cout << ... << args);
  cout << endl;
}

