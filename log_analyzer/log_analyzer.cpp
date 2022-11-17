#include "log_analyzer.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "../gps_unit.h"
#include "../util.h"

namespace fs = std::filesystem;

namespace gps_parser {

class LogAnalyzer::LogAnalyzerImpl {
public:
  LogAnalyzerImpl() : gps_parser_(std::make_unique<GPSParser>()) {}
  ~LogAnalyzerImpl() {
  }

public:
  std::string dir_or_file_;
  std::unique_ptr<GPSParser> gps_parser_;
  std::string file_prefix_ = "parsed_";

  void Analyze(std::string_view dir_or_file) {
    if (fs::is_directory(dir_or_file)) {
      AnalyzeADir(dir_or_file);
    }

    if (fs::is_regular_file(dir_or_file)) {
      AnalyzeFile(dir_or_file);
    }
  }

  // Analyzes a file.  If filename is not a file, then do nothing.
  void AnalyzeFile(std::string_view filename) {

    // Only analyze the normal file.
    if (!fs::is_regular_file(filename)) {
      return;
    }

    std::ifstream in(filename, std::istream::in);
    std::string line;
    GPSUnit gps_unit;

    Print(INFO, "Analyze file: ", filename);

    while (std::getline(in, line)) {
      Print(INFO, "Parsing line: ", line);
      gps_parser_->Parse(line, &gps_unit);
    }
  }

  // Analyzes all the logs in the directory.
  void AnalyzeADir(std::string_view dir_name) {

    Print(INFO, "Analyze dir: ", dir_name);
    for (auto &&dir_entry : fs::directory_iterator{dir_name}) {
      Analyze(dir_entry.path().string());
    }
  }

  void WriteMessageToFile(std::string_view raw_message,
                          std::string_view filename) {
    // TODO
  }

  std::string GetAnalyzeFileName(std::string_view source_filename) {
    return file_prefix_ + std::string(source_filename);
  }
};

LogAnalyzer::LogAnalyzer() : pimpl_(std::make_unique<LogAnalyzerImpl>()) {}
LogAnalyzer::~LogAnalyzer(){}

void LogAnalyzer::Analyze(std::string_view dir_or_file) {
  pimpl_->Analyze(dir_or_file);
}

} // namespace gps_parser
