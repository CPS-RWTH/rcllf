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

# Fixture: nodes that log now() next to the LF
# logical tag, so a run under rcllf shows whether node clocks follow LF time,
# plus a node that sleeps on ROS time.
cmake_minimum_required(VERSION 3.8)
project(clock_fixture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(clock_timer_component SHARED src/clock_timer_component.cpp)
target_include_directories(clock_timer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(clock_timer_component PUBLIC cxx_std_17)
target_link_libraries(clock_timer_component ${CMAKE_DL_LIBS})
ament_target_dependencies(clock_timer_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(clock_timer_component "clock_fixture::ClockTimer")

add_executable(clock_sub_node src/clock_sub_node.cpp)
target_include_directories(clock_sub_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(clock_sub_node ${CMAKE_DL_LIBS})
ament_target_dependencies(clock_sub_node rclcpp std_msgs)

install(TARGETS clock_timer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
add_executable(clock_ros_sleep_node src/clock_ros_sleep_node.cpp)
target_include_directories(clock_ros_sleep_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(clock_ros_sleep_node ${CMAKE_DL_LIBS})
ament_target_dependencies(clock_ros_sleep_node rclcpp)

install(TARGETS clock_sub_node clock_ros_sleep_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
# Exported so lf_generated_* packages can compile the staged node sources.
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

ament_export_dependencies(rclcpp rclcpp_components std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
