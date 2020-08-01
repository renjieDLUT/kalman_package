cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 14)
project(ukf)
include_directories("include" /usr/include/eigen3)

add_library(ukf src/ukf.cpp)
add_executable(main src/main.cpp)
target_link_libraries(main ukf)