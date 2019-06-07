cmake_minimum_required(VERSION 3.12)
project(gasMeterUp)

set(CMAKE_CXX_STANDARD 17)

find_package(Curses REQUIRED)
find_package(opencv REQUIRED)

message( "find opencv " ${OpenCV_VERSION})

include_directories(${CURSES_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(gasMeterUp main.cpp)

add_executable(save save.cpp)
target_link_libraries(save ${CURSES_LIBRARIES} ${OpenCV_LIBS} )

add_executable(staticExp withSHT20.cpp)
add_executable(gasDetector gasDetector.cpp)
add_executable(gasDetectorRecv gasDetectorRecv.cpp)

add_executable(paint paint.cpp)
target_link_libraries(paint ${OpenCV_LIBS} )

add_executable(monitor monitor.cpp)