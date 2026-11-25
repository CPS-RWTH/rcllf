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
project(tf2_acq_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcllf_tf2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tf_source_node src/tf_source_node.cpp)
target_compile_features(tf_source_node PUBLIC cxx_std_17)
ament_target_dependencies(tf_source_node rclcpp tf2_ros geometry_msgs)

add_executable(acq_sampler_node src/acq_sampler_node.cpp)
target_compile_features(acq_sampler_node PUBLIC cxx_std_17)
ament_target_dependencies(acq_sampler_node rclcpp tf2 tf2_ros geometry_msgs)
target_link_libraries(acq_sampler_node rcllf_tf2::rcllf_tf2)

install(TARGETS tf_source_node acq_sampler_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY corpus/ DESTINATION share/${PROJECT_NAME}/corpus)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # O1: an independent tf2::BufferCore replay of corpus/t1_t7_events.tsv
  # (not the adapter), checked against the hand table corpus/t1_t7_expected.tsv.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_o1_oracle test/test_o1_oracle.cpp TIMEOUT 60)
  target_compile_definitions(test_o1_oracle PRIVATE
    CORPUS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/corpus")
  ament_target_dependencies(test_o1_oracle tf2 geometry_msgs)
endif()

ament_package()
