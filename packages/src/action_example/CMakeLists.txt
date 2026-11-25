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
project(action_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(action_server_node
  src/action_server_node.cpp
)
target_include_directories(action_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_server_node PUBLIC cxx_std_17)
ament_target_dependencies(action_server_node
  rclcpp
  rclcpp_action
  custom_interfaces
)

add_executable(action_client_node
  src/action_client_node.cpp
)
target_include_directories(action_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_client_node PUBLIC cxx_std_17)
ament_target_dependencies(action_client_node
  rclcpp
  rclcpp_action
  custom_interfaces
)

add_executable(action_cancel_client_node
  src/action_cancel_client_node.cpp
)
target_include_directories(action_cancel_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_cancel_client_node PUBLIC cxx_std_17)
ament_target_dependencies(action_cancel_client_node
  rclcpp
  rclcpp_action
  custom_interfaces
)

add_executable(action_pair_client_node
  src/action_pair_client_node.cpp
)
target_include_directories(action_pair_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_pair_client_node PUBLIC cxx_std_17)
ament_target_dependencies(action_pair_client_node
  rclcpp
  rclcpp_action
  custom_interfaces
)

add_executable(action_dual_server_node
  src/action_dual_server_node.cpp
)
target_include_directories(action_dual_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_dual_server_node PUBLIC cxx_std_17)
ament_target_dependencies(action_dual_server_node
  rclcpp
  rclcpp_action
  custom_interfaces
)

add_executable(action_dual_client_node
  src/action_dual_client_node.cpp
)
target_include_directories(action_dual_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_dual_client_node PUBLIC cxx_std_17)
ament_target_dependencies(action_dual_client_node
  rclcpp
  rclcpp_action
  custom_interfaces
)

add_executable(action_chain_client_node
  src/action_chain_client_node.cpp
)
target_include_directories(action_chain_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(action_chain_client_node PUBLIC cxx_std_17)
ament_target_dependencies(action_chain_client_node
  rclcpp
  rclcpp_action
  custom_interfaces
)

install(TARGETS
  action_server_node
  action_client_node
  action_cancel_client_node
  action_pair_client_node
  action_dual_server_node
  action_dual_client_node
  action_chain_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action custom_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # SIGINT (context shutdown) during a goal must not abort the server.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_action_server_shutdown test/test_action_server_shutdown.cpp TIMEOUT 120)
  target_include_directories(test_action_server_shutdown PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_action_server_shutdown rclcpp rclcpp_action custom_interfaces)
endif()

ament_package()
