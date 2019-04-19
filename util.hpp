#pragma once

#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <ctime>

using namespace std;

#define Level int
#define DEBUG 0
#define INFO 1
#define WARNING 2
#define ERROR 3

#define _UNUSED_ __attribute__((unused))

bool StartWith(const string& str, const string& word){
  if(str.size() < word.size()) return false;
  bool same = true;
  unsigned int i = 0;
  for( ;i<word.size() && same; i++)
    same = word[i] == str[i];
  return i == word.size();
}


vector<string> StrSplit( const string& str, const string& delim ){
  vector<string> ret;
  size_t last = 0, next = 0;

  while( (next = str.find(delim,last)) != string::npos ){
    if(next-last == 0){
      ret.push_back("");
    }else{
      ret.push_back( str.substr(last, next-last) );
    }
    next++;
    last = next;
  }
  ret.push_back(str.substr(last));
  return ret;
}

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

int ParseNumberi(const string& str){
  return std::stoi(str);
}

float ParseNumberf(const string& str){
  if(str == "") return 0.0f;
  return std::stof(str);
}

string AscGpsTime( std::tm* tm ){
  char ch[32];
  std::sprintf(ch, "%d:%d:%d\n", tm->tm_hour, tm->tm_min, tm->tm_sec);
  return string(ch);
}
