#pragma once

#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <ctime>

using std::string;
using std::vector;
using std::cout;
using std::endl;

#define Level int
#define DEBUG 0
#define INFO 1
#define WARNING 2
#define ERROR 3

#define _UNUSED_ __attribute__((unused))

bool StartWith(const string& str, const string& word);
vector<string> StrSplit( const string& str, const string& delim );
string AscGpsTime( std::tm* tm );

template <typename T>
void PrintContainer(Level l, const T& container){
#ifdef DISABLE_DEBUG_MSG
  if(l == DEBUG) return;
#endif
#ifdef DISABLE_DEBUG_MSG
  if(l == INFO) return;
#endif
  for(auto it = container.begin(); it!=container.end(); it++){
    cout<<*it<<" ";
  }
  cout<<endl;
}

template <typename ...T>
void Print(Level l,  T... args ){
#ifdef DISABLE_DEBUG_MSG
  if(l == DEBUG) return;
#endif
#ifdef DISABLE_INFO_MSG
  if(l == INFO) return;
#endif
  (cout<<...<<args);
  cout<<endl;
}

