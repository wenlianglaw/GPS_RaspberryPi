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

#include "../erkir/include/sphericalpoint.h"
#include "../gps_parser.h"

namespace fs = std::filesystem;

namespace gps_parser {

class LogAnalyzer::LogAnalyzerImpl {
public:
  LogAnalyzerImpl() : gps_parser_(std::make_unique<GPSParser>()) {
    ReadGoogleApiKeyFromFile(kGoogleApiFilepath);
  }
  ~LogAnalyzerImpl() { cout << "exiting..." << endl; }

public:
  static constexpr const char *kGoogleApiFilepath = "../GOOGLE_MAP_API";
  // Two GPS locations that greater than this distance will be considered as
  // "far", and will be marked on the Google Map.
  static constexpr float kXMeterConsideredAsFar = 50.0f;
  std::unique_ptr<GPSParser> gps_parser_;
  std::string google_api_key_;

  // All the links from these logs.
  // pair<date time, google map link>
  std::vector<std::pair<std::string, std::string>> google_map_links_;

  void Analyze(const std::string &dir_or_file) {
    AnalyzeImpl(dir_or_file);

    // Writes the paths into an html file.
    fs::path dir_name = dir_or_file;
    if (fs::is_regular_file(dir_or_file)) {
      dir_name = fs::path(dir_or_file).parent_path();
    }
    fs::path html_path = dir_name.append("paths.html");

    std::ofstream html(html_path, std::ofstream::out);
    html << GenerateHtml(google_map_links_);
    google_map_links_.clear();
    html.close();

    std::cout << "html is written to " << html_path << std::endl;
    std::cout << "Analyze finished." << std::endl;
  }

  void AnalyzeImpl(const std::string &dir_or_file) {
    if (fs::is_directory(dir_or_file)) {
      AnalyzeADir(dir_or_file);
    }

    if (fs::is_regular_file(dir_or_file)) {
      AnalyzeFile(dir_or_file);
    }
  }

  // Analyzes a file.  If filename is not a file, then do nothing.
  void AnalyzeFile(const std::string &filename) {

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
    cout << gps_records.size() << endl;
    std::vector<std::string> google_map_links =
        GenerateGoogleMapLink(gps_records);
    for (int i = 0; i < google_map_links.size(); i++) {
      google_map_links_.push_back(
          {fs::path(filename).filename().string() + "_" + std::to_string(i),
           google_map_links[i]});
      out << google_map_links[i] << std::endl;
    }
    out.close();
  } // AnalyzeFile

  // Analyzes all the logs in the directory.
  void AnalyzeADir(const std::string &dir_name) {

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

  void ReadGoogleApiKeyFromFile(std::string google_api_key_filename) {
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
  std::vector<std::string>
  GenerateGoogleMapLink(const std::vector<GPSUnit> &records) {
    if (records.empty()) {
      return {"No GPS records."};
    }

    std::vector<std::string> links;

    const char *size = "800x800";
    int zoom_lvl = GetZoomLevel(records, 800);
    char link[8192] = {0};
    // When the GPS record location is "too far" from the last location, put it
    // on the Google map.
    int selected_data_pts = 1;
    std::string path = "color:0xff0000ff|weight:2";
    // TODO finish this.
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

    links.emplace_back(link);
    return links;
  }

private:
  // https://github.com/vahancho/erkir
  double DistanceInMeter(const GPSUnit &loc_a, const GPSUnit &loc_b) {
    double lat1 = loc_a.latitude_;
    double long1 = loc_a.longitude_;
    double lat2 = loc_b.latitude_;
    double long2 = loc_b.longitude_;
    erkir::spherical::Point p1{lat1, -long1};
    erkir::spherical::Point p2{lat2, -long2};
    double d = p1.distanceTo(p2);
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
    if (records.empty())
      return 16;
    double left = records[0].longitude_;
    double right = records[0].longitude_;
    double top = records[0].latitude_;
    double bot = records[0].latitude_;

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

void LogAnalyzer::Analyze(const std::string &dir_or_file) {
  pimpl_->Analyze(dir_or_file);
}

} // namespace gps_parser
