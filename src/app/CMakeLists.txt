cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(app VERSION 0.0.2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # explicitly disallow backwards compatibility - project is C++17 heavy
set(CMAKE_CXX_EXTENSIONS OFF)  # disallow compiler specific features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_executable(${PROJECT_NAME} main.cpp adapter.cpp adapter.h json.hpp)

target_link_libraries(${PROJECT_NAME} fusion)
target_link_libraries(${PROJECT_NAME} network)
