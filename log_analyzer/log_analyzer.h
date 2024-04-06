#include <memory>
#include <string>

#include "../gps_parser.h"

namespace gps_parser {

// This is the structure that will be written into the analyze
// files.
struct CustomGPSMsgToWrite {
  float latitude_;
  std::string ew_;
  float longitude_;
  std::string ns_;
  // "%Y-%m-%d %H:%M:%S"
  std::string date_time_;
};

/* This Log analyzer takes the raw GPS logs, parse them and write the parsed
 * result to the parsed files.  The parsed file names start with `file_prefix_`.
 */
class LogAnalyzer {
public:
  LogAnalyzer();
  ~LogAnalyzer();

public:
  // If dir_or_file is directory.  Analyze all the logs in
  // the direcotry.
  // If it is a file, only analyze this file.
  void Analyze(const std::string& dir_or_file);

private:
  class LogAnalyzerImpl;
  std::unique_ptr<LogAnalyzerImpl> pimpl_;

};

} // namespace gps_parser
