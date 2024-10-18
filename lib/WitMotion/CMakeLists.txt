cmake_minimum_required(VERSION 3.10)
project(WitMotion VERSION 0.1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_library(WitMotion wit_c_sdk.c Com.cpp)







