# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(intra_process_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(intra_process_publisher_component SHARED
  src/intra_process_publisher_component.cpp
)
target_include_directories(intra_process_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(intra_process_publisher_component PUBLIC cxx_std_17)
ament_target_dependencies(intra_process_publisher_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(intra_process_publisher_component
  "intra_process_example::IntraProcessPublisherComponent")

add_library(intra_process_subscriber_component SHARED
  src/intra_process_subscriber_component.cpp
)
target_include_directories(intra_process_subscriber_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(intra_process_subscriber_component PUBLIC cxx_std_17)
ament_target_dependencies(intra_process_subscriber_component
  rclcpp
  rclcpp_components
  std_msgs
)
rclcpp_components_register_nodes(intra_process_subscriber_component
  "intra_process_example::IntraProcessSubscriberComponent")

install(TARGETS
  intra_process_publisher_component
  intra_process_subscriber_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install and export headers so downstream lf_generated_* packages can include them.
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
