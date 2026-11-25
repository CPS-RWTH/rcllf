# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(same_tag_fixture)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# One executable per node class. Spelled out:
# the analyzer resolves executables from literal add_executable calls.
add_executable(st_source src/st_source.cpp)
target_include_directories(st_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(st_source rclcpp std_msgs)

add_executable(st_stage src/st_stage.cpp)
target_include_directories(st_stage PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(st_stage rclcpp std_msgs)

add_executable(st_stage_in src/st_stage_in.cpp)
target_include_directories(st_stage_in PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(st_stage_in rclcpp std_msgs)

add_executable(st_stage_out src/st_stage_out.cpp)
target_include_directories(st_stage_out PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(st_stage_out rclcpp std_msgs)

add_executable(st_sink src/st_sink.cpp)
target_include_directories(st_sink PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(st_sink rclcpp std_msgs)

add_executable(st_multi_sink src/st_multi_sink.cpp)
target_include_directories(st_multi_sink PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(st_multi_sink rclcpp std_msgs)

install(TARGETS
  st_source
  st_stage
  st_stage_in
  st_stage_out
  st_sink
  st_multi_sink
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
