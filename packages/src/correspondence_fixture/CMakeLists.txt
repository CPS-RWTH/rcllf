# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(correspondence_fixture)

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
find_package(std_msgs REQUIRED)

# Publisher node
add_executable(corr_publisher src/corr_publisher.cpp)
target_include_directories(corr_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_publisher rclcpp std_msgs)

# Subscriber node (subscribes /corr/a and /corr/b)
add_executable(corr_subscriber src/corr_subscriber.cpp)
target_include_directories(corr_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_subscriber rclcpp std_msgs)

# Subscriber node (subscribes /corr/a only)
add_executable(corr_subscriber_a src/corr_subscriber_a.cpp)
target_include_directories(corr_subscriber_a PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_subscriber_a rclcpp std_msgs)

# Shared join fixture
add_executable(corr_fanin_relay src/corr_fanin_relay.cpp)
target_include_directories(corr_fanin_relay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_fanin_relay rclcpp std_msgs)

# Install targets
add_executable(corr_ping src/corr_ping.cpp)
target_include_directories(corr_ping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_ping rclcpp std_msgs)

add_executable(corr_ping_echo src/corr_ping_echo.cpp)
target_include_directories(corr_ping_echo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_ping_echo rclcpp std_msgs)

# Install targets
add_executable(corr_pong src/corr_pong.cpp)
target_include_directories(corr_pong PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_pong rclcpp std_msgs)

# Relay fixture (W3 slice S3)
add_executable(corr_publisher_one src/corr_publisher_one.cpp)
target_include_directories(corr_publisher_one PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_publisher_one rclcpp std_msgs)

# Relay fixture (W3 slice S3)
add_executable(corr_relay src/corr_relay.cpp)
target_include_directories(corr_relay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_relay rclcpp std_msgs)

# Relay fixture (W3 slice S3)
add_executable(corr_relay_sink src/corr_relay_sink.cpp)
target_include_directories(corr_relay_sink PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_relay_sink rclcpp std_msgs)

# Shared-port fixture
add_executable(corr_two_group_publisher src/corr_two_group_publisher.cpp)
target_include_directories(corr_two_group_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_two_group_publisher rclcpp std_msgs)

# Shared-port fixture
add_executable(corr_reentrant_publisher src/corr_reentrant_publisher.cpp)
target_include_directories(corr_reentrant_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(corr_reentrant_publisher rclcpp std_msgs)


# Install targets
install(TARGETS
  corr_publisher
  corr_subscriber
  corr_subscriber_a
  corr_ping
  corr_ping_echo
  corr_pong
  corr_publisher_one
  corr_relay
  corr_relay_sink
  corr_two_group_publisher
  corr_reentrant_publisher
  corr_fanin_relay
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
ament_export_dependencies(rclcpp std_msgs)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
