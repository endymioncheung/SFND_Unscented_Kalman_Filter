project(UKF)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/ukf.cpp src/ukf.h)
add_executable(ukf ${sources})
target_link_libraries(ukf)