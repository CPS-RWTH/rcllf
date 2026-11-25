# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(qos_hazard_example)

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
find_package(message_filters REQUIRED)

# Publisher node (drives the hazard subscriptions below)
add_executable(hazard_publisher_node src/hazard_publisher_node.cpp)
target_include_directories(hazard_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hazard_publisher_node rclcpp custom_interfaces)

# best_effort QoS subscription -> RCLLF-H001
add_executable(best_effort_subscriber_node src/best_effort_subscriber_node.cpp)
target_include_directories(best_effort_subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(best_effort_subscriber_node rclcpp custom_interfaces)

# rclcpp::SensorDataQoS() named-profile subscription -> also RCLLF-H001
# (named-profile recognition, QoS hardening)
add_executable(sensor_data_subscriber_node src/sensor_data_subscriber_node.cpp)
target_include_directories(sensor_data_subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_data_subscriber_node rclcpp custom_interfaces)

# Depth-1 subscription queue -> RCLLF-H002
add_executable(shallow_queue_subscriber_node src/shallow_queue_subscriber_node.cpp)
target_include_directories(shallow_queue_subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(shallow_queue_subscriber_node rclcpp custom_interfaces)

# Two-input ApproximateTime synchronizer: admitted (no hazard)
add_executable(message_filters_subscriber_node src/message_filters_subscriber_node.cpp)
target_include_directories(message_filters_subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(message_filters_subscriber_node rclcpp custom_interfaces message_filters)

# Subscriber feeding a message_filters::Cache -> RCLLF-H003 (unsupported form)
add_executable(message_filters_cache_node src/message_filters_cache_node.cpp)
target_include_directories(message_filters_cache_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(message_filters_cache_node rclcpp custom_interfaces message_filters)

# Chained rclcpp::QoS builder calls (.reliable().durability_volatile()
# .keep_last(n), .transient_local().keep_all())
add_executable(chained_qos_subscriber_node src/chained_qos_subscriber_node.cpp)
target_include_directories(chained_qos_subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(chained_qos_subscriber_node rclcpp custom_interfaces)

# Publishers whose reliability the analysis cannot see, one on no
# edge and one of two twins -> startup RCLLF-H000 per publisher
add_executable(unknown_qos_publisher_node src/unknown_qos_publisher_node.cpp)
target_include_directories(unknown_qos_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(unknown_qos_publisher_node rclcpp custom_interfaces)

# Install targets
install(TARGETS
  hazard_publisher_node
  best_effort_subscriber_node
  sensor_data_subscriber_node
  shallow_queue_subscriber_node
  message_filters_subscriber_node
  message_filters_cache_node
  chained_qos_subscriber_node
  unknown_qos_publisher_node
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
ament_export_dependencies(rclcpp custom_interfaces message_filters)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
