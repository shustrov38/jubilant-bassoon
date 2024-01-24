cmake_minimum_required(VERSION 3.0.0)

file(GLOB PROJECT_SOURCES *.cpp ${COMMON_DIR}*.cpp ${PHISICS_DIR}/*.cpp)
file(GLOB PROJECT_HEADERS *.h ${COMMON_DIR}*.h ${PHISICS_DIR}/*.h)

add_executable(${MAIN_TARGET_NAME}-sfml ${PROJECT_SOURCES} ${PROJECT_HEADERS})
target_link_libraries(${MAIN_TARGET_NAME}-sfml PRIVATE ImGui-SFML::ImGui-SFML)
target_include_directories(${MAIN_TARGET_NAME}-sfml PUBLIC ${PHISICS_DIR} ${COMMON_DIR})

configure_file(${RESOURCES_DIR}/GeistMono-Regular.otf ${CMAKE_CURRENT_BINARY_DIR}/GeistMono-Regular.otf COPYONLY)