cmake_minimum_required(VERSION 3.15)
project(cc_pcl)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(classifiy_normal  classifiy_normal.cpp ) #normal_estimation.cpp cmake-build-debug/normal_estimate.h
target_link_libraries (classifiy_normal ${PCL_LIBRARIES})

