cmake_minimum_required(VERSION 3.0.0)
project(jubilant-bassoon VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)

set(MAIN_TARGET_NAME jubilant-bassoon)

add_compile_options(-Wall -Wpedantic -Wextra -Werror -Wno-reorder -Wno-unused-parameter -Wno-unknown-pragmas)        
if(${NEED_SANITIZER})
    add_compile_options(-fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)   
    add_link_options(-fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
endif()

set(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/common)
set(PHISICS_DIR ${CMAKE_CURRENT_LIST_DIR}/phisics)

add_subdirectory(app_cli)
add_subdirectory(app_sfml)