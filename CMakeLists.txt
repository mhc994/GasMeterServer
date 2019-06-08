cmake_minimum_required(VERSION 3.5)
project(gasMeterUp)

set(CMAKE_CXX_STANDARD 14)

find_package (Threads)
add_executable(monitor monitor.cpp)
target_link_libraries (monitor ${CMAKE_THREAD_LIBS_INIT})
