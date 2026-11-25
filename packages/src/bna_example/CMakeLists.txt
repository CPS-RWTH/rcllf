# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(bna_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(rcllf_message_filters REQUIRED)

add_executable(anchor_sensor_node src/anchor_sensor_node.cpp)
target_include_directories(anchor_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(anchor_sensor_node rclcpp custom_interfaces)

add_executable(candidate_sensor_node src/candidate_sensor_node.cpp)
target_include_directories(candidate_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(candidate_sensor_node rclcpp custom_interfaces)

add_executable(bna_fusion_node src/bna_fusion_node.cpp)
target_include_directories(bna_fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bna_fusion_node rclcpp custom_interfaces message_filters
  rcllf_message_filters)

install(TARGETS anchor_sensor_node candidate_sensor_node bna_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export headers so downstream lf_generated_* packages can include them.
ament_export_include_directories(include)
ament_export_dependencies(rclcpp custom_interfaces message_filters rcllf_message_filters)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
