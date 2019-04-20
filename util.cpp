#include "util.h"

bool StartWith(const string& str, const string& word){
  if(str.size() < word.size()) return false;
  bool same = true;
  unsigned int i = 0;
  for( ;i<word.size() && same; i++)
    same = word[i] == str[i];
  return same && i == word.size();
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

string AscGpsTime( std::tm* tm ){
  char ch[32];
  std::sprintf(ch, "%d:%d:%d\n", tm->tm_hour, tm->tm_min, tm->tm_sec);
  return string(ch);
}


// TODO Going to use protobuf.
// Then add test.
void WriteGPSUnit(const gps_parser::GPSUnit& gps_unit,
    const string& filename){
}
void ParseGPSUnitFromBytes(const char* ch, unsigned int size,
    gps_parser::GPSUnit* gps_unit){
}
