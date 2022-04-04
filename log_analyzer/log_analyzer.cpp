#include "log_analyzer.h"

#include <filesystem>
#include <fstream>

#include "../gps_unit.h"

namespace fs = std::filesystem;

namespace gps_parser {

LogAnalyzer::LogAnalyzer() : gps_parser_(std::make_unique<GPSParser>()) {}

void LogAnalyzer::Analyze(std::string_view dir_or_file) {
  if (fs::is_directory(dir_or_file)) {
    AnalyzeADir(dir_or_file);
  }

  if (fs::is_regular_file(dir_or_file)) {
    AnalyzeAFile(dir_or_file);
  }
}

void LogAnalyzer::AnalyzeADir(std::string_view dir_name) {
  for (auto&& dir_entry : fs::directory_iterator{dir_name}) {
    AnalyzeAFile(dir_entry.path().string());
  }
}

void LogAnalyzer::AnalyzeAFile(std::string_view filename) {
  if (!fs::is_regular_file(filename)) {
    return;
  }
}

std::string LogAnalyzer::GetAnalyzeFileName(std::string_view source_filename) {
  return file_prefix_ + std::string(source_filename);
}

}  // namespace gps_parser
