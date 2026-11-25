# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

cmake_minimum_required(VERSION 3.18)
project(brake_fusion)

# The rcllf analyzer parses the nodes with the flags from compile_commands.json,
# which it needs to resolve the MuJoCo include of the simulator.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # No multiply-add contraction into FMA (GCC's default on aarch64): the nodes
  # must round the same way on every CPU.
  add_compile_options(-Wall -Wextra -Wpedantic -ffp-contract=off)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

set(DEPS
  rclcpp rclcpp_components geometry_msgs message_filters nav_msgs rosgraph_msgs
  sensor_msgs std_msgs custom_interfaces)

# The plant (MuJoCo 3.14.0, fetched and pinned there) sits behind the
# rcllf_plant adapter and Transactor.
find_package(rcllf_plant REQUIRED)


include_directories(include)

# One component library per node, loaded by the component container and by
# rcllf; one standalone executable per node. Each executable's source
# holds only main(); the node class lives in <node>_node.cpp. The targets are
# spelled out (no foreach) because the rcllf analyzer reads this file
# statically.

add_library(simulator_component SHARED src/simulator_node.cpp)
ament_target_dependencies(simulator_component ${DEPS})
target_link_libraries(simulator_component rcllf_plant::rcllf_plant rcllf_plant::rcllf_plant_mujoco)
rclcpp_components_register_nodes(simulator_component "brake_fusion::SimulatorNode")
add_executable(simulator_node src/simulator_main.cpp)
target_link_libraries(simulator_node simulator_component)
ament_target_dependencies(simulator_node ${DEPS})
set_target_properties(simulator_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

add_library(radar_component SHARED src/radar_node.cpp)
ament_target_dependencies(radar_component ${DEPS})
rclcpp_components_register_nodes(radar_component "brake_fusion::RadarNode")
add_executable(radar_node src/radar_main.cpp)
target_link_libraries(radar_node radar_component)
ament_target_dependencies(radar_node ${DEPS})

add_library(camera_component SHARED src/camera_node.cpp)
ament_target_dependencies(camera_component ${DEPS})
rclcpp_components_register_nodes(camera_component "brake_fusion::CameraNode")
add_executable(camera_node src/camera_main.cpp)
target_link_libraries(camera_node camera_component)
ament_target_dependencies(camera_node ${DEPS})

add_library(wheel_speed_component SHARED src/wheel_speed_node.cpp)
ament_target_dependencies(wheel_speed_component ${DEPS})
rclcpp_components_register_nodes(wheel_speed_component "brake_fusion::WheelSpeedNode")
add_executable(wheel_speed_node src/wheel_speed_main.cpp)
target_link_libraries(wheel_speed_node wheel_speed_component)
ament_target_dependencies(wheel_speed_node ${DEPS})

add_library(radar_tracker_component SHARED src/radar_tracker_node.cpp)
ament_target_dependencies(radar_tracker_component ${DEPS})
rclcpp_components_register_nodes(radar_tracker_component "brake_fusion::RadarTrackerNode")
add_executable(radar_tracker_node src/radar_tracker_main.cpp)
target_link_libraries(radar_tracker_node radar_tracker_component)
ament_target_dependencies(radar_tracker_node ${DEPS})

add_library(exact_fusion_component SHARED src/exact_fusion_node.cpp)
ament_target_dependencies(exact_fusion_component ${DEPS})
rclcpp_components_register_nodes(exact_fusion_component "brake_fusion::ExactFusionNode")
add_executable(exact_fusion_node src/exact_fusion_main.cpp)
target_link_libraries(exact_fusion_node exact_fusion_component)
ament_target_dependencies(exact_fusion_node ${DEPS})

add_library(approximate_fusion_component SHARED src/approximate_fusion_node.cpp)
ament_target_dependencies(approximate_fusion_component ${DEPS})
rclcpp_components_register_nodes(approximate_fusion_component "brake_fusion::ApproximateFusionNode")
add_executable(approximate_fusion_node src/approximate_fusion_main.cpp)
target_link_libraries(approximate_fusion_node approximate_fusion_component)
ament_target_dependencies(approximate_fusion_node ${DEPS})


add_library(controller_component SHARED src/controller_node.cpp)
ament_target_dependencies(controller_component ${DEPS})
rclcpp_components_register_nodes(controller_component "brake_fusion::ControllerNode")
add_executable(controller_node src/controller_main.cpp)
target_link_libraries(controller_node controller_component)
ament_target_dependencies(controller_node ${DEPS})

install(TARGETS
  simulator_component radar_component camera_component
  wheel_speed_component exact_fusion_component approximate_fusion_component
  controller_component radar_tracker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  simulator_node radar_node camera_node wheel_speed_node exact_fusion_node
  approximate_fusion_node controller_node radar_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${DEPS} rcllf_plant)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  # The supervised plant timeout from outside the process.
  # simulator_fault_node is simulator_node on rcllf_plant_fault, the only
  # Transactor build that honours RCLLF_PLANT_FAULT; it is not installed.
  add_executable(simulator_fault_node src/simulator_main.cpp src/simulator_node.cpp)
  ament_target_dependencies(simulator_fault_node ${DEPS})
  target_link_libraries(simulator_fault_node
    rcllf_plant::rcllf_plant_fault rcllf_plant::rcllf_plant_mujoco)
  ament_add_pytest_test(plant_timeout test/test_plant_timeout.py TIMEOUT 120
    ENV SIM_FAULT_NODE=$<TARGET_FILE:simulator_fault_node>)
  # Simulator pacing, real_time_factor.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_simulator_pace test/test_simulator_pace.cpp TIMEOUT 120)
  target_link_libraries(test_simulator_pace simulator_component)
  ament_target_dependencies(test_simulator_pace ${DEPS})
  # Collision event whenever the vehicle reaches the obstacle.
  ament_add_gtest(test_simulator_collision test/test_simulator_collision.cpp TIMEOUT 180)
  target_link_libraries(test_simulator_collision simulator_component)
  ament_target_dependencies(test_simulator_collision ${DEPS})
  # Stock startup timeout of a /wheel/speed stream that never starts.
  ament_add_gtest(test_wheel_startup_timeout test/test_wheel_startup_timeout.cpp TIMEOUT 60)
  if(TARGET test_wheel_startup_timeout)
    target_link_libraries(test_wheel_startup_timeout controller_component)
    ament_target_dependencies(test_wheel_startup_timeout ${DEPS})
  endif()
  # Modeled brake actuation delay, stamp-ordered. Own
  # main(): rclcpp::init before the node tests.
  ament_add_gtest(test_brake_actuator test/test_brake_actuator.cpp TIMEOUT 120
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_brake_actuator simulator_component)
  ament_target_dependencies(test_brake_actuator ${DEPS})
  # Node-level injected loss of one publication.
  ament_add_gtest(test_publish_drop test/test_publish_drop.cpp)
  # Fate of every ApproximateTime sample.
  ament_add_gtest(test_sample_ledger test/test_sample_ledger.cpp)
  if(TARGET test_sample_ledger)
    ament_target_dependencies(test_sample_ledger ${DEPS})
  endif()
  # Closing speed: no unchecked stamp estimate.
  ament_add_gtest(test_closing_speed test/test_closing_speed.cpp)
  # radar_tracker update order.
  ament_add_gtest(test_radar_tracker test/test_radar_tracker.cpp)
  if(TARGET test_radar_tracker)
    ament_target_dependencies(test_radar_tracker rclcpp sensor_msgs)
  endif()
  # The gtests that construct nodes run on UDPv4 only, without the
  # Fast DDS shared-memory transport, whose input-channel lock hung one plain
  # gtest run >18 min. test_simulator_pace also sets it in its own code.
  # plant_timeout's simulator child hung the same way before its first log
  # line, 3 of 3 runs beside a full /dev/shm; the child inherits the variable.
  foreach(_bf_node_test test_simulator_pace test_simulator_collision test_brake_actuator
      test_wheel_startup_timeout plant_timeout)
    if(TEST ${_bf_node_test})
      set_property(TEST ${_bf_node_test} APPEND PROPERTY ENVIRONMENT
        "FASTDDS_BUILTIN_TRANSPORTS=UDPv4")
    endif()
  endforeach()
endif()

ament_package()
