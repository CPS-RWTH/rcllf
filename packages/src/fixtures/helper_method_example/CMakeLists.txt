# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(helper_method_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Test node: helper methods, a parameter-named topic
add_executable(helper_node src/helper_node.cpp)
target_include_directories(helper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(helper_node rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)

install(TARGETS helper_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Export headers so downstream lf_generated_* packages can include them.
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)

# This was the only package in the repo with no BUILD_TESTING block, so it
# was the only one getting zero ament lint coverage while trivially
# "passing" colcon test with 0 tests.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
