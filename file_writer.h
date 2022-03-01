/* The file_rw stands for file writer.
 * It provides the funtion to write the GPS message or the gps_unit to a local file.
 */

#ifndef FILE_WR_H_
#define FILE_WR_H_

#include <memory>
#include <string>
#include <string_view>
#include <fstream>

#include "gps_unit.h"

namespace gps_parser{
class FileWriter {
  public:
    FileWriter();
    ~FileWriter();

    void Init();

    // Specify a directory to write the log files.
    // If not set, it will use the default output dir.
    void ChangeOutputDir(std::string_view new_dir);

    // Gets the current output dir.
    std::string GetOutputDir();

    // Writes the raw message to the raw log file.
    void WriteRawMessage(std::string_view msg);

    // Writes the field that we are interested to the parsed log file.
    void WriteGpsUnit(GPSUnit unit);

  private:
    std::string output_dir_;

    // The raw messages go here.
    std::string raw_log_file_name_;
    std::ofstream raw_log_file_;

    // The gps unit messages go here.
    std::string gps_unit_log_file_name_;
    std::ofstream gps_unit_log_file_;

  private:
    std::string GetDefaultOutputDir();
    void UpdateRawLogFileName();
    void UpdateGpsUnitLogFileName();

    // Creates output dir if it doesn't exist.
    void CreateOutPutDir();

    // This function returns the log file suffix.
    // _{date}_{HH}-{HH+1}.log
    // Example:
    // _2022301_04-05.log
    // _2022301_23-00.log
    std::string GetLogFileSuffix();
};

}  // namespace gps_parser{

#endif
