cmake_minimum_required(VERSION 3.13)
project(network VERSION 0.0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} server.cpp server.h)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/include)
    target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/opt/openssl/include)
    target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/lib)
    target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/opt/openssl/lib)
#    link_directories(/usr/local/Cellar/libuv/1*/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

target_link_libraries(${PROJECT_NAME} z ssl uv uWS)
target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR})