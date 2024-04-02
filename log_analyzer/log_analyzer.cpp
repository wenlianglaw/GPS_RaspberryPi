#include "log_analyzer.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

// snprintf
#include <stdio.h>

#define DISABLE_INFO_MSG
#include "../gps_unit.h"
#include "../util.h"

#include "../gps_parser.h"

namespace fs = std::filesystem;

namespace gps_parser {

class LogAnalyzer::LogAnalyzerImpl {
public:
  LogAnalyzerImpl() : gps_parser_(std::make_unique<GPSParser>()) {
    ReadGoogleApiKeyFromFile(kGoogleApiFilepath);
  }
  ~LogAnalyzerImpl() {}

public:
  static constexpr const char *kGoogleApiFilepath = "../GOOGLE_MAP_API";
  // Two GPS locations that greater than this distance will be considered as
  // "far", and will be marked on the Google Map.
  static constexpr float kXMeterConsideredAsFar = 10.0f;
  std::unique_ptr<GPSParser> gps_parser_;
  std::string google_api_key_;

  // All the links from these logs.
  // pair<date time, google map link>
  std::vector<std::pair<std::string, std::string>> google_map_links_;

  void Analyze(std::string_view dir_or_file) {
    AnalyzeImpl(dir_or_file);

    // Writes the paths into an html file.
    fs::path dir_name = dir_or_file;
    if (fs::is_regular_file(dir_or_file)) {
      dir_name = fs::path(dir_or_file).parent_path();
    }
    fs::path html_path = dir_name.append("paths.html");

    std::ofstream html(html_path, std::ofstream::out);
    html << GenerateHtml(google_map_links_);
    html.close();
  }

  void AnalyzeImpl(std::string_view dir_or_file) {
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

    if (fs::path(filename).extension() != ".log" ||
        !StartWith(fs::path(filename).filename().string(), "raw")) {
      return;
    }

    std::cout << "Analyzing file: " << filename << std::endl;
    std::ifstream in(std::string(filename), std::istream::in);
    std::string line;
    std::ofstream out(GetAnalyzeFileName(std::string(filename)),
                      std::ofstream::out | std::ofstream::app);
    GPSUnit gps_unit;
    // Valid GPS records in this file.
    std::vector<GPSUnit> gps_records;
    while (std::getline(in, line)) {
      int start = line.find("$");
      if (start == std::string::npos) {
        continue;
      }
      line = line.substr(start);
      std::stringstream parsed_gps_msg;
      parsed_gps_msg << std::setprecision(6) << gps_unit.time_ << " "
                     << gps_unit.latitude_ << gps_unit.NS_ << " "
                     << gps_unit.longitude_ << gps_unit.EW_ << " "
                     << gps_unit.google_map_url_;
      PRINT(INFO, parsed_gps_msg.str());
      // If this record is valid:
      // - Writes to parsed log.
      if (gps_parser_->Parse(line, &gps_unit)) {
        if (!gps_unit.time_.empty() && gps_unit.latitude_ != 0.0f &&
            gps_unit.longitude_ != 0.0f) {
          out << parsed_gps_msg.str() << std::endl;
          gps_records.push_back(gps_unit);
        }
      }
    } // while get line

    // At the end, put a Google map link.
    std::string google_map_link = GenerateGoogleMapLink(gps_records);
    google_map_links_.push_back(
        {fs::path(filename).filename().string(), google_map_link});
    out << google_map_link << std::endl;
    out.close();
  } // AnalyzeFile

  // Analyzes all the logs in the directory.
  void AnalyzeADir(std::string_view dir_name) {

    std::cout << "Analyze dir: " << dir_name << std::endl;
    for (auto &&dir_entry : fs::directory_iterator{dir_name}) {
      AnalyzeImpl(dir_entry.path().string());
    }
  }

  std::string GetAnalyzeFileName(const std::string &source_filename) {
    std::string dir_name = fs::path(source_filename).parent_path();
    std::string new_filename =
        std::string("parsed_") + fs::path(source_filename).filename().string();
    fs::path fullpath = dir_name;
    fullpath /= new_filename;
    PRINT(INFO, fullpath);
    return fullpath.string();
  }

  void ReadGoogleApiKeyFromFile(std::string_view google_api_key_filename) {
    std::ifstream in(std::string(google_api_key_filename), std::istream::in);
    std::string line;
    while (std::getline(in, line)) {
      if (StartWith(line, "key:")) {
        google_api_key_ = line.substr(4);
        return;
      }
    }
    google_api_key_ = "API key not found.";
    in.close();
  }

  // Generate a static map link from for a list of GPS records.
  std::string GenerateGoogleMapLink(const std::vector<GPSUnit> &records) {
    if (records.empty()) {
      return "No GPS records.";
    }

    const char *size = "800x800";
    int zoom_lvl = GetZoomLevel(records, 800);
    char link[8192] = {0};
    // When the GPS record location is "too far" from the last location, put it
    // on the Google map.
    int selected_data_pts = 1;
    std::string path = "color:0xff0000ff|weight:2";
    GPSUnit last_location = records[0];
    {
      path += "|" + ConvertToGoogleStylePath(last_location);
      for (int i = 1; i < records.size(); i++) {
        if (DistanceInMeter(records[i], last_location) >=
            kXMeterConsideredAsFar) {
          last_location = records[i];
          path += "|" + ConvertToGoogleStylePath(last_location);
          selected_data_pts++;
        }
      }
    }

    if (selected_data_pts > 1) {
      snprintf(link, sizeof(link),
               "https://maps.googleapis.com/maps/api/"
               "staticmap?size=%s&zoom=%s&path=%s&key=%s",
               size, std::to_string(zoom_lvl).c_str(), path.c_str(),
               google_api_key_.c_str());
    } else {
      // If there is only one data point, place a map Marker instead of showing
      // paths.
      snprintf(link, sizeof(link),
               "https://maps.googleapis.com/maps/api/"
               "staticmap?markers=color:red|%s&zoom=%s&size=%s&key=%s",
               ConvertToGoogleStylePath(last_location).c_str(),
               std::to_string(zoom_lvl).c_str(), size, google_api_key_.c_str());
    }

    return std::string(link);
  }

private:
  // Formula: https://www.movable-type.co.uk/scripts/latlong.html
  double DistanceInMeter(const GPSUnit &loc_a, const GPSUnit &loc_b) {
    double lat1 = loc_a.latitude_;
    double lat2 = loc_b.latitude_;
    double long1 = loc_a.longitude_;
    double long2 = loc_b.longitude_;

    // Earth radius ~6371km
    const double r = 6371e3;
    double phi1 = lat1 * M_PI / 180.0f;
    double phi2 = lat2 * M_PI / 180.0f;
    double delta_phi = (lat2 - lat1) * M_PI / 180;
    double delta_lambda = (long2 - long1) * M_PI / 180;

    double a = std::sin(delta_phi / 2) * std::sin(delta_lambda / 2) +
               std::cos(phi1) * std::cos(phi2) * std::sin(delta_lambda / 2) *
                   std::sin(delta_lambda / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    double d = r * c;

    PRINT(INFO, "Distance is ", d);

    return d;
  }

  std::string ConvertToGoogleStylePath(const GPSUnit &unit) {
    std::string lat_sign;
    std::string long_sign;

    if (unit.NS_ == "S") {
      lat_sign = "-";
    }
    if (unit.EW_ == "W") {
      long_sign = "-";
    }
    return lat_sign + std::to_string(unit.latitude_) + "," + long_sign +
           std::to_string(unit.longitude_);
  }

  int GetZoomLevel(const std::vector<GPSUnit> &records, int size) {
    float left = records[0].longitude_;
    float right = records[0].longitude_;
    float top = records[0].latitude_;
    float bot = records[0].latitude_;

    // Doesn't work for cross 0, but we don't care since its not our use case.
    for (int i = 1; i < records.size(); i++) {
      const GPSUnit &gps = records[i];
      if (gps.longitude_ < left) {
        left = gps.longitude_;
      }
      if (gps.longitude_ > right) {
        right = gps.longitude_;
      }
      if (gps.latitude_ < bot) {
        bot = gps.latitude_;
      }
      if (gps.latitude_ > top) {
        top = gps.latitude_;
      }
    }

    GPSUnit b1, b2;
    b1.longitude_ = left;
    b1.latitude_ = bot;
    b2.longitude_ = right;
    b2.latitude_ = bot;
    double width = DistanceInMeter(b1, b2);

    b1.longitude_ = left;
    b1.latitude_ = bot;
    b2.longitude_ = left;
    b2.latitude_ = top;
    double height = DistanceInMeter(b1, b2);
    double cmp = std::max(width, height);

    // lvl 16: 800 pixel ~ 400m
    int lvl = 16;
    // size to meter
    size /= 2;
    // Leave some blank margin for the map.
    cmp *= 2.5;
    while (lvl > 0) {
      if (size <= cmp) {
        size *= 2;
        lvl--;
      } else {
        break;
      }
    }
    return lvl;
  } // GetZoomLevel

  std::string
  GenerateHtml(const std::vector<std::pair<std::string, std::string>>
                   &google_map_links) {
    std::stringstream ss;
    for (const auto &[filename, link] : google_map_links) {
      ss << filename << std::endl;
      ss << "<img src=\"" << link << "\" >" << std::endl;
    }
    return ss.str();
  }
};

LogAnalyzer::LogAnalyzer() : pimpl_(std::make_unique<LogAnalyzerImpl>()) {}
LogAnalyzer::~LogAnalyzer() {}

void LogAnalyzer::Analyze(std::string_view dir_or_file) {
  pimpl_->Analyze(dir_or_file);
}

} // namespace gps_parser
