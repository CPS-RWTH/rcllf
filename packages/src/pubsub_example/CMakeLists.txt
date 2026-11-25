# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(pubsub_example)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

# Publisher node
add_executable(publisher_node src/publisher_node.cpp)
target_include_directories(publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(publisher_node rclcpp custom_interfaces)

# Subscriber node
add_executable(subscriber_node src/subscriber_node.cpp)
target_include_directories(subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(subscriber_node rclcpp custom_interfaces)

# Install targets
install(TARGETS
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export headers so downstream lf_generated_* packages can include them.
ament_export_include_directories(include)
ament_export_dependencies(rclcpp custom_interfaces)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
