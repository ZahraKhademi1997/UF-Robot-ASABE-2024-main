cmake_minimum_required(VERSION 3.1.0)

project(IntrinsicExample)

add_executable(intrinsic_example intrinsic_example.cpp)
set_property(TARGET intrinsic_example PROPERTY CXX_STANDARD 11)

find_package(realsense2 REQUIRED)
target_link_libraries(intrinsic_example realsense2)
