project(tests)

add_executable(gps_parser_test gps_parser_test.cpp)
add_executable(util_test util_test.cpp)

list(APPEND EXTRA_LIBS gps_parser)
list(APPEND EXTRA_LIBS util)

target_link_libraries(gps_parser_test PUBLIC ${EXTRA_LIBS})
target_link_libraries(util_test PUBLIC ${EXTRA_LIBS})
