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
project(tf_broadcaster_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(static_tf_broadcaster_node
  src/static_tf_broadcaster_node.cpp
)
target_include_directories(static_tf_broadcaster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(static_tf_broadcaster_node PUBLIC cxx_std_17)
ament_target_dependencies(static_tf_broadcaster_node
  rclcpp
  tf2_ros
  geometry_msgs
)

add_executable(dynamic_tf_broadcaster_node
  src/dynamic_tf_broadcaster_node.cpp
)
target_include_directories(dynamic_tf_broadcaster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(dynamic_tf_broadcaster_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamic_tf_broadcaster_node
  rclcpp
  tf2_ros
  geometry_msgs
)

install(TARGETS
  static_tf_broadcaster_node
  dynamic_tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp tf2_ros geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
