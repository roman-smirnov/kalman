cmake_minimum_required(VERSION 3.13)
project(kalman VERSION 0.0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_subdirectory(src/network)
add_subdirectory(src/fusion)
add_subdirectory(src/app)
