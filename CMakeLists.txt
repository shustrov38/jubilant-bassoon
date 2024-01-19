cmake_minimum_required(VERSION 3.0.0)
project(jubilant-bassoon VERSION 0.1.0 LANGUAGES C CXX)

set(LOG_IN_CSV ON)
set(NEED_SANITIZER OFF)

add_subdirectory(3rd-party)
add_subdirectory(sources)

configure_file(resources/GeistMono-Regular.otf ./sources/GeistMono-Regular.otf COPYONLY)