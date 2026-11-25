# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(basic_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(basic_components SHARED
  src/timer_publisher_node.cpp
  src/relay_node.cpp
  src/timer_and_subscription_node.cpp
  src/listener_node.cpp
  src/timer_only_node.cpp
  src/dual_listener_node.cpp
  src/dual_timer_publisher_node.cpp
  src/dual_sub_timer_publisher_node.cpp
)

ament_target_dependencies(basic_components
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_nodes(basic_components
  "basic_components::TimerPublisherNode"
  "basic_components::RelayNode"
  "basic_components::TimerAndSubscriptionNode"
  "basic_components::ListenerNode"
  "basic_components::TimerOnlyNode"
  "basic_components::DualListenerNode"
  "basic_components::DualTimerPublisherNode"
  "basic_components::DualSubscriptionTimerPublisherNode"
)

install(TARGETS
  basic_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Export headers so downstream lf_generated_* packages can include them.
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
