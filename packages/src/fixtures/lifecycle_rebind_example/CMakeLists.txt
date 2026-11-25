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
project(lifecycle_rebind_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# Lifecycle unbind on deactivate and re-bind on a later activate.
# One executable per node, each spelled out for the analyzer.
add_executable(ticker_node src/ticker_node.cpp)
add_executable(sleeper_node src/sleeper_node.cpp)
add_executable(supervisor_node src/supervisor_node.cpp)
add_executable(listener_node src/listener_node.cpp)
add_executable(responder_node src/responder_node.cpp)
# A lifecycle subscription made in on_configure and reset in on_cleanup.
add_executable(feeder_node src/feeder_node.cpp)
add_executable(reader_node src/reader_node.cpp)
add_executable(driver_node src/driver_node.cpp)
# The reader with its twin topic from a launch parameter.
add_executable(param_reader_node src/param_reader_node.cpp)
set(rebind_nodes ticker_node sleeper_node supervisor_node listener_node responder_node
  feeder_node reader_node driver_node param_reader_node)
foreach(target ${rebind_nodes})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target}
    rclcpp rclcpp_action rclcpp_lifecycle std_msgs example_interfaces)
endforeach()

install(TARGETS ${rebind_nodes}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install and export headers so downstream lf_generated_* packages can include them.
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action rclcpp_lifecycle std_msgs example_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
