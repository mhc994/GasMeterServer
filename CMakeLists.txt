cmake_minimum_required(VERSION 3.12)
project(gasMeterUp)

set(CMAKE_CXX_STANDARD 17)

add_executable(gasMeterUp main.cpp)
add_executable(save save.cpp)
add_executable(staticExp withSHT20.cpp)