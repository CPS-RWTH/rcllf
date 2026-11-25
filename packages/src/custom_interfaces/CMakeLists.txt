# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "msg/FusedState.msg"
  "msg/Detections.msg"
  "msg/Plan.msg"
  "msg/BrakeCmd.msg"
  "msg/SyncSample.msg"
  "srv/ProcessData.srv"
  "action/Analyze.action"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
