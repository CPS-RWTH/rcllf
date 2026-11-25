# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(sentry_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories(include)

add_library(sentry_pipeline SHARED
  src/workload.cpp
  src/scenario_env.cpp
  src/imu_node.cpp
  src/lidar_node.cpp
  src/camera_node.cpp
  src/fusion_node.cpp
  src/detector_node.cpp
  src/planner_node.cpp
  src/controller_node.cpp
  src/actuator_sink_node.cpp
  src/diagnostics_sink_node.cpp
)

ament_target_dependencies(sentry_pipeline
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  custom_interfaces
)

rclcpp_components_register_nodes(sentry_pipeline
  "sentry_pipeline::ImuNode"
  "sentry_pipeline::LidarNode"
  "sentry_pipeline::CameraNode"
  "sentry_pipeline::FusionNode"
  "sentry_pipeline::DetectorNode"
  "sentry_pipeline::PlannerNode"
  "sentry_pipeline::ControllerNode"
  "sentry_pipeline::ActuatorSinkNode"
  "sentry_pipeline::DiagnosticsSinkNode"
)


install(TARGETS
  sentry_pipeline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Export headers so downstream lf_generated_* packages can include them.
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs geometry_msgs custom_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scenario_env test/test_scenario_env.cpp src/scenario_env.cpp)
  ament_target_dependencies(test_scenario_env rclcpp)
endif()

ament_package()
