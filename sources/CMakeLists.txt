cmake_minimum_required(VERSION 3.0.0)
project(jubilant-bassoon VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)

set(MAIN_TARGET_NAME jubilant-bassoon)

if(${LOG_IN_CSV})
    add_compile_definitions(LOG_CSV)
endif()

add_compile_options(-Wall -Wpedantic -Wextra -Werror -Wno-reorder -Wno-unused-parameter -Wno-unknown-pragmas)        
if(${NEED_SANITIZER})
    add_compile_options(-fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)   
    add_link_options(-fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
endif()

file(GLOB PROJECT_SOURCES *.cpp)
file(GLOB PROJECT_HEADERS *.h)

add_executable(${MAIN_TARGET_NAME} ${PROJECT_SOURCES} ${PROJECT_HEADERS})
target_link_libraries(${MAIN_TARGET_NAME} PRIVATE ImGui-SFML::ImGui-SFML)