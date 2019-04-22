cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
project(kalman VERSION 0.1.0 LANGUAGES CXX)

#message(${PROJECT_VERSION})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # explicitly disallow backwards compatibility - project is C++17 heavy
set(CMAKE_CXX_EXTENSIONS OFF)  # disallow compiler specific features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_subdirectory(src/network)
add_subdirectory(src/fusion)
add_subdirectory(src/app)
