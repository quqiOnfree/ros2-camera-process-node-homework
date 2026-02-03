cmake_minimum_required(VERSION 3.24)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

project(ros2_ws)

add_subdirectory(src/detect)
add_subdirectory(src/usb_cam)
