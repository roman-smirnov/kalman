cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(fusion VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # explicitly disallow backwards compatibility - project is C++17 heavy
set(CMAKE_CXX_EXTENSIONS OFF)  # disallow compiler specific features

add_library(${PROJECT_NAME} laser_filter.cpp radar_filter.cpp evaluation.cpp extended_filter.cpp models.h fusion.h common.h)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR})