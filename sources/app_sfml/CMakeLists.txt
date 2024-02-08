cmake_minimum_required(VERSION 3.0.0)

file(GLOB PROJECT_SOURCES *.cpp ${COMMON_DIR}*.cpp ${PHISICS_DIR}/*.cpp)
file(GLOB PROJECT_HEADERS *.h ${COMMON_DIR}*.h ${PHISICS_DIR}/*.h)

set(TARGET_NAME "${MAIN_TARGET_NAME}-sfml")

add_executable(${TARGET_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS})
target_link_libraries(${TARGET_NAME} PRIVATE ImGui-SFML::ImGui-SFML)
target_include_directories(${TARGET_NAME} PUBLIC ${PHISICS_DIR} ${COMMON_DIR})
set_target_properties(${TARGET_NAME} PROPERTIES INSTALL_RPATH $ORIGIN/../lib)

install(TARGETS ${TARGET_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
configure_file(${RESOURCES_DIR}/GeistMono-Regular.otf ${CMAKE_INSTALL_PREFIX}/res/GeistMono-Regular.otf COPYONLY)