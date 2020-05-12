cmake_minimum_required (VERSION 3.17.1 FATAL_ERROR)

project(Trajectory_Intercept_System)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES src/main.cpp)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Trajectory_Intercept_System ${SOURCE_FILES})

target_link_libraries(Trajectory_Intercept_System ${OpenCV_LIBS})