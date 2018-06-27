cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(cloudScaling)

find_package(PCL 1.7 REQUIRED)
set (CMAKE_CXX_STANDARD 11)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloudScaling cloudScaling.cpp)
target_link_libraries (cloudScaling ${PCL_LIBRARIES})
