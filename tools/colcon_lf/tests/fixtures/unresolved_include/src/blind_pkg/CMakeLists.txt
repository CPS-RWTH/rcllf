# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.18)
project(blind_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
add_executable(blind_node src/blind_node.cpp)
ament_target_dependencies(blind_node rclcpp std_msgs)
install(TARGETS blind_node DESTINATION lib/${PROJECT_NAME})

ament_package()
