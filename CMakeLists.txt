cmake_minimum_required(VERSION 3.7)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(kalmanTest)

set(EIGEN_INC_DIR /usr/include/eigen-3.2.10/)
include_directories(${EIGEN_INC_DIR})

set(SOURCE_FILES src/KalmanFilter.cpp src/KalmanFilter_test.cpp)
add_executable(kalmanTest ${SOURCE_FILES})