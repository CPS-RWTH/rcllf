# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(rcllf_tf2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Header-only: AcquisitionLookup and the weak runtime hooks.
add_library(rcllf_tf2 INTERFACE)
target_include_directories(rcllf_tf2 INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(rcllf_tf2 INTERFACE cxx_std_17)
target_link_libraries(rcllf_tf2 INTERFACE
  ${geometry_msgs_TARGETS} rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS rcllf_tf2 EXPORT export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_acquisition_lookup test/test_acquisition_lookup.cpp TIMEOUT 60)
  target_link_libraries(test_acquisition_lookup rcllf_tf2)
  # Stock mode: no ros2_lf_tf2_* definitions, the weak symbols stay null.
  ament_add_gtest(test_acquisition_lookup_stock test/test_acquisition_lookup_stock.cpp TIMEOUT 120)
  target_link_libraries(test_acquisition_lookup_stock rcllf_tf2)
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(geometry_msgs rclcpp tf2 tf2_ros)
ament_package()
