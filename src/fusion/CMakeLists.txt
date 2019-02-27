cmake_minimum_required(VERSION 3.13)
project(fusion VERSION 0.0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} fusion.cpp laser_filter.cpp radar_filter.cpp evaluation.cpp fusion.h models.h)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR})
