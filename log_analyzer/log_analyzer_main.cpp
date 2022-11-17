#include "log_analyzer.h"
#include <iostream>
#include <string>

using gps_parser::LogAnalyzer;

void PrintHelp() {
  std::string help_str = R"str(
Usage: log_analyzer raw_logs_dir
)str";

  std::cout << help_str << std::endl;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    PrintHelp();
    return 0;
  }

  std::string input_dir(argv[1]);
  LogAnalyzer analyzer;
  analyzer.Analyze(input_dir);

  return 0;
}
