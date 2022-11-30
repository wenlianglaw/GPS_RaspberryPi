project(log_analyzer_main)

add_library(log_analyzer log_analyzer.cpp)

# cc_binary
add_executable(log_analyzer_main log_analyzer_main.cpp)

# Extra libs
list(APPEND EXTRA_LIBS log_analyzer)
list(APPEND EXTRA_LIBS util)
list(APPEND EXTRA_LIBS gps_parser)

# Extra includes
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/..")

# deps
target_link_libraries(log_analyzer_main PUBLIC ${EXTRA_LIBS})

target_include_directories(
  log_analyzer_main
  PUBLIC "${PROJECT_BINARY_DIR}"
  ${EXTRA_INCLUDES})
