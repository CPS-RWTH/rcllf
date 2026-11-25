# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(lifecycle_component_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(include)

add_library(lifecycle_component_example SHARED
  src/lifecycle_component_node.cpp
  src/unchained_activate_node.cpp
)

ament_target_dependencies(lifecycle_component_example
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  lifecycle_msgs
)

# The registration that puts this managed lifecycle node on the component path.
# This single call is what this fixture is about: it makes the analyzer set
# is_component, which routes the node to LFComponentManager, which never
# registers it in the node registry, so nothing ever drives its lifecycle.
rclcpp_components_register_nodes(lifecycle_component_example
  "lifecycle_component_example::LifecycleComponentNode"
  "lifecycle_component_example::UnchainedActivateNode"
)

install(TARGETS
  lifecycle_component_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Export headers so downstream lf_generated_* packages can include them.
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
