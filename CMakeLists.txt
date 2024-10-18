cmake_minimum_required(VERSION 3.10)
project(Main VERSION 0.1 LANGUAGES C CXX)

add_executable(Main main.cpp)

add_subdirectory(lib/NatorsSDK)
add_subdirectory(lib/WitMotion)

target_include_directories(Main PUBLIC "${PROJECT_SOURCE_DIR}/lib/NatorsSDK/include" "${PROJECT_SOURCE_DIR}/lib/WitMotion")

target_link_libraries(Main NTSDK)
target_link_libraries(Main WitMotion)