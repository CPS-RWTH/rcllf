# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.18)
project(param_topic_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
add_executable(param_topic_node src/param_topic_node.cpp)
ament_target_dependencies(param_topic_node rclcpp std_msgs)
install(TARGETS param_topic_node DESTINATION lib/${PROJECT_NAME})

ament_package()
