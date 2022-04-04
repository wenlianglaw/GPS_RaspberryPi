#include <memory>
#include <string>
#include <string_view>

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

class LogAnalyzer {
 public:
  std::string file_prefix_ = "parsed_";
  std::unique_ptr<GPSParser> gps_parser_;

 private:
  std::string dir_or_file_;

 public:
  LogAnalyzer();
  ~LogAnalyzer() = default;

 public:
  // If dir_or_file is directory.  Analyze all the logs in
  // the direcotry.
  // If it is a file, only analyze this file.
  void Analyze(std::string_view dir_or_file);

 private:
  // Analyzes a file.  If filename is not a file, then do nothing.
  void AnalyzeAFile(std::string_view filename);

  // Analyzes all the logs in the directory.
  void AnalyzeADir(std::string_view dir_name);

  void WriteMessageToFile(std::string_view raw_message,
                          std::string_view filename);

  std::string GetAnalyzeFileName(std::string_view source_filename);
};

}  // namespace gps_parser
