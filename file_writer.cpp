#include "file_writer.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

namespace fs = std::filesystem;

namespace gps_parser {

static constexpr char DEFAULT_OUTPUT_DIR[] = "./log/";
static constexpr char DEFAULT_RAW_LOG_PREFIX[] = "raw";

FileWriter::FileWriter() { Init(); }

FileWriter::~FileWriter() {
  if (raw_log_file_.is_open()) {
    raw_log_file_.close();
  }
}

void FileWriter::ChangeOutputDir(std::string_view new_dir) {
  output_dir_ = new_dir;
  CreateOutPutDir();
}

std::string FileWriter::GetOutputDir() { return output_dir_; }

void FileWriter::Init() {
  output_dir_ = GetDefaultOutputDir();
  UpdateRawLogFileName();
  CreateOutPutDir();
}

std::string FileWriter::GetDefaultOutputDir() { return DEFAULT_OUTPUT_DIR; }

std::string FileWriter::GetLogFileSuffix() {
  // Gets the time objects.
  auto now = std::chrono::system_clock::now();
  auto tm_t = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&tm_t);

  // parse the date and hour
  int current_hour = 0;
  int next_hour = 0;
  char date[32];
  char str_hour[8];

  std::strftime(str_hour, 4, "%H", &tm);
  current_hour = std::stoi(std::string(str_hour));
  next_hour = (current_hour + 1) % 25;

  std::strftime(date, 32, "%Y%m%d", &tm);

  std::string suffix = "_" + std::string(date) + "_" +
                       std::to_string(current_hour) + "-" +
                       std::to_string(next_hour) + ".log";

  return suffix;
}

void FileWriter::UpdateRawLogFileName() {
  std::string filename =
      output_dir_ + std::string(DEFAULT_RAW_LOG_PREFIX) + GetLogFileSuffix();
  if (filename != raw_log_file_name_) {
    raw_log_file_name_ = filename;
    raw_log_file_.close();
    raw_log_file_ = std::ofstream(raw_log_file_name_, std::ofstream::app);
  }
}

void FileWriter::WriteMessage(std::string_view msg) {
  UpdateRawLogFileName();
  if (raw_log_file_.fail()) {
    std::cerr << "Fail to open the raw log file." << std::endl;
    return;
  }
  // Removes the CR (Carriage Return) char.
  msg.remove_suffix(msg.size() - msg.find_last_of('\r'));
  raw_log_file_ << msg << std::endl;
}

void FileWriter::CreateOutPutDir() {
  if (!fs::exists(output_dir_)) {
    fs::create_directories(output_dir_);
  }
  fs::permissions(output_dir_, fs::perms::owner_all | fs::perms::group_all,
                  fs::perm_options::add);
}

}  // namespace gps_parser

