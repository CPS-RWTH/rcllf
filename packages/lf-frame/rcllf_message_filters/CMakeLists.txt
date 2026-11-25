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
project(rcllf_message_filters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)

# Header-only: the BNA engine (pure C++17) and the message_filters policy.
add_library(rcllf_message_filters INTERFACE)
target_include_directories(rcllf_message_filters INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(rcllf_message_filters INTERFACE cxx_std_17)
target_link_libraries(rcllf_message_filters INTERFACE
  message_filters::message_filters rclcpp::rclcpp)

# Replay tool: the engine with the falsification mutants compiled in.
add_executable(bna_replay src/bna_replay.cpp)
target_link_libraries(bna_replay rcllf_message_filters)
target_compile_definitions(bna_replay PRIVATE RCLLF_BNA_MUTANTS)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS rcllf_message_filters EXPORT export_${PROJECT_NAME})
install(TARGETS bna_replay DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bna_engine test/test_bna_engine.cpp)
  target_link_libraries(test_bna_engine rcllf_message_filters)

  find_package(geometry_msgs REQUIRED)
  ament_add_gtest(test_bna_policy test/test_bna_policy.cpp TIMEOUT 60)
  target_link_libraries(test_bna_policy rcllf_message_filters
    ${geometry_msgs_TARGETS})
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(message_filters rclcpp)
ament_package()
