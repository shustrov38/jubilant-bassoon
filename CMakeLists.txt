cmake_minimum_required(VERSION 3.0.0)
project(jubilant-bassoon VERSION 0.1.0 LANGUAGES C CXX)

set(NEED_SANITIZER OFF)

set(RESOURCES_DIR ${CMAKE_CURRENT_LIST_DIR}/resources)

add_subdirectory(3rd-party)
add_subdirectory(sources)