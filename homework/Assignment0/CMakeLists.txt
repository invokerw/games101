cmake_minimum_required (VERSION 2.8.12)
project (Transformation)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("${EIGEN3_INCLUDE_DIR}")

add_executable (Transformation main.cpp)
