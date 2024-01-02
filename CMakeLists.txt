cmake_minimum_required(VERSION 3.0.0)
project(jubilant-bassoon VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(Boost REQUIRED system)

set(MAIN_TARGET_NAME jubilant-bassoon)

add_executable(${MAIN_TARGET_NAME} main.cpp)
target_include_directories(${MAIN_TARGET_NAME} PRIVATE ${Boost_INCLUDE_DIR})
target_link_libraries(${MAIN_TARGET_NAME} ${Boost_LIBRARIES})
