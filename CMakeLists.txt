cmake_minimum_required(VERSION 3.10)

project(gps_raspberry_pi_main)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# cc_binary
add_executable(gps_raspberry_pi_main gps_raspberry_pi_main.cpp)

# Subdirectories
add_subdirectory(log_analyzer)
add_subdirectory(tests)

# Extra libraries
add_library(gps_parser gps_parser.cpp)
add_library(util util.cpp)
list(APPEND EXTRA_LIBS gps_parser)
list(APPEND EXTRA_LIBS util)

# deps
target_link_libraries(gps_raspberry_pi_main PUBLIC ${EXTRA_LIBS})
target_include_directories(
  gps_raspberry_pi_main
  PUBLIC "${PROJECT_BINARY_DIR}"
  ${EXTRA_INCLUDES})


# run tests
add_custom_target(test
  COMMAND ./util_test
  COMMAND ./gps_parser_test
  DEPENDS util_test gps_parser_test
  WORKING_DIRECTORY ./tests)

