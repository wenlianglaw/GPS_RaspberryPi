/* This class writes the GPS message.
 * This is the only feature it provides: It creates the filename based on the
 * message data and time.
 */

#ifndef FILE_WR_H_
#define FILE_WR_H_

#include <fstream>
#include <memory>
#include <string>
#include <string_view>

namespace gps_parser {

// Stateful file writer class. 
// Use to append message to files.
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
  void WriteMessage(std::string_view msg);

private:
  std::string output_dir_;

  // The raw messages go here.
  std::string raw_log_file_name_;
  std::ofstream raw_log_file_;

private:
  std::string GetDefaultOutputDir();
  void UpdateRawLogFileName();

  // Creates output dir if it doesn't exist.
  void CreateOutPutDir();

  // This function returns the log file suffix.
  // _{date}_{HH}-{HH+1}.log
  // Example:
  // _2022301_04-05.log
  // _2022301_23-00.log
  std::string GetLogFileSuffix();
};

} // namespace gps_parser

#endif
