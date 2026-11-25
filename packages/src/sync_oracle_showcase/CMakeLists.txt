# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.8)
project(sync_oracle_showcase)

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

# The dpkg version strings of the two pinned packages (domain
# mf-sync/1). The policies are header templates, so the version compiled
# against here is the version of the code that runs; versions.hpp is
# installed, so the lf_generated_* build reads the same strings.
function(sync_dpkg_version pkg out)
  execute_process(
    COMMAND dpkg-query -W -f=\${Version} ${pkg}
    OUTPUT_VARIABLE v OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE rc)
  if(NOT rc EQUAL 0 OR v STREQUAL "")
    set(v "unknown")
  endif()
  set(${out} "${v}" PARENT_SCOPE)
endfunction()
sync_dpkg_version(ros-jazzy-message-filters SYNC_MF_VERSION)
sync_dpkg_version(ros-jazzy-rclcpp SYNC_RCLCPP_VERSION)
configure_file(cmake/versions.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/sync_oracle_showcase/versions.hpp @ONLY)

set(SYNC_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# One add_executable per node, spelled out: colcon lf-analyze resolves an
# executable to its source from these lines.
add_executable(corpus_player src/corpus_player.cpp)
target_include_directories(corpus_player PUBLIC ${SYNC_INCLUDES})
ament_target_dependencies(corpus_player rclcpp custom_interfaces message_filters)
add_executable(exact_sync_node src/exact_sync_node.cpp)
target_include_directories(exact_sync_node PUBLIC ${SYNC_INCLUDES})
ament_target_dependencies(exact_sync_node rclcpp custom_interfaces message_filters)
add_executable(approx_sync_node src/approx_sync_node.cpp)
target_include_directories(approx_sync_node PUBLIC ${SYNC_INCLUDES})
ament_target_dependencies(approx_sync_node rclcpp custom_interfaces message_filters)
add_executable(branch_worker src/branch_worker.cpp)
target_include_directories(branch_worker PUBLIC ${SYNC_INCLUDES})
ament_target_dependencies(branch_worker rclcpp custom_interfaces message_filters)

# Oracle O1: plain executable, no ROS graph (test/sync_oracle.cpp).
add_executable(sync_oracle test/sync_oracle.cpp)
target_include_directories(sync_oracle PUBLIC ${SYNC_INCLUDES})
ament_target_dependencies(sync_oracle rclcpp custom_interfaces message_filters)

install(TARGETS
  corpus_player exact_sync_node approx_sync_node branch_worker sync_oracle
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/ DESTINATION include)
install(DIRECTORY launch corpora DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rclcpp custom_interfaces message_filters)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sync_oracle test/test_sync_oracle.cpp)
  target_include_directories(test_sync_oracle PUBLIC ${SYNC_INCLUDES})
  target_compile_definitions(test_sync_oracle PRIVATE
    SYNC_CORPUS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/corpora")
  ament_target_dependencies(test_sync_oracle rclcpp custom_interfaces message_filters)
endif()

ament_package()
