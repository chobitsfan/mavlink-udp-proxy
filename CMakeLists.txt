cmake_minimum_required(VERSION 3.8)
project(mavlink_udp)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -Wno-address-of-packed-member")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(${ament_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_executable(mavlink_udp my_mavlink_udp.cpp)
ament_target_dependencies(mavlink_udp rclcpp std_msgs visualization_msgs geometry_msgs sensor_msgs nav_msgs)

ament_package()
