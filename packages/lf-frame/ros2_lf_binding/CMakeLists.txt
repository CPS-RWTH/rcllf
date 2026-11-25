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
project(ros2_lf_binding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl REQUIRED)
find_package(tracetools REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ros2_lf_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(type_description_interfaces REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

# --- reactor-c as a static library ---
set(REACTOR_C_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/reactor-c)

set(LF_SINGLE_THREADED 1)
set(LOG_LEVEL 2)  # LOG_LEVEL_INFO
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(${REACTOR_C_DIR}/include)
include_directories(${REACTOR_C_DIR}/include/api)
include_directories(${REACTOR_C_DIR}/include/core)
include_directories(${REACTOR_C_DIR}/include/core/federated)
include_directories(${REACTOR_C_DIR}/include/core/modal_models)
include_directories(${REACTOR_C_DIR}/include/core/platform)
include_directories(${REACTOR_C_DIR}/include/core/threaded)
include_directories(${REACTOR_C_DIR}/include/core/utils)

add_subdirectory(${REACTOR_C_DIR} ${CMAKE_CURRENT_BINARY_DIR}/reactor-c)

# reactor-c's CMakeLists unconditionally includes test/Tests.cmake (no
# BUILD_TESTING guard upstream), which registers its own test executables
# with per-target -Werror. Those test sources rely on assert() to "use"
# their variables, so any -DNDEBUG build (Release/RelWithDebInfo) strips the
# asserts and aborts the whole build on -Werror=unused-variable. We never run
# reactor-c's suite from this package: exclude its test targets from the
# default build and disable their ctest registrations so `colcon test` is
# unaffected.
get_property(_reactor_c_targets DIRECTORY ${REACTOR_C_DIR} PROPERTY BUILDSYSTEM_TARGETS)
foreach(_target IN LISTS _reactor_c_targets)
  if(_target MATCHES "_test_c$")
    set_target_properties(${_target} PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()
endforeach()
get_property(_reactor_c_tests DIRECTORY ${REACTOR_C_DIR} PROPERTY TESTS)
foreach(_test IN LISTS _reactor_c_tests)
  set_property(TEST ${_test} DIRECTORY ${REACTOR_C_DIR} PROPERTY DISABLED TRUE)
endforeach()

# The platform clock (lf_unix_clock_support.c) logs through
# reactor-c's util.c, but upstream never declares that edge, so the linker
# scans libreactor-c.a once, before the platform archive. Debug builds hide
# this: tag.c's LF_ASSERTN pulls util.c.o in early, and NDEBUG compiles that
# reference out. Without the edge a Release libros2_lf_binding.so lacks util.c.o
# and every executable linking it pulls util.c.o itself, which needs the
# lfc-generated _lf_get_environments. Declaring the cycle makes CMake repeat
# libreactor-c.a after the platform archive.
cmake_policy(SET CMP0079 NEW)
target_link_libraries(lf-low-level-platform-impl PRIVATE reactor-c)

# --- ros2_lf_binding shared library ---
add_library(ros2_lf_binding SHARED
  src/rid_table.cpp
  src/time_source.cpp
  src/clock_guard.cpp
  src/clock_publisher.cpp
  src/runtime.cpp
  src/node_binding.cpp
  src/bind_check.cpp
  src/service_pairing.cpp
  src/reaction_dispatch.cpp
  src/action_channels.cpp
  src/action_client_guard.cpp
  src/take_mode.cpp
  src/match_warmup.cpp
  src/pair_match.cpp
  src/tag_log.cpp
  src/xtag_link.cpp
  src/policy_table.cpp
  src/node_registry.cpp
  src/thread_role.cpp
  src/ingress.cpp
  src/external_input.cpp
  src/output_deadline.cpp
  src/plant_record.cpp
  src/plant_replay.cpp
  src/correspondence.cpp
  src/corr_runtime.cpp
  src/semantic_trace.cpp
  src/handle_matching.cpp
  src/input_period.cpp
  src/silence_deadline.cpp
  src/bna_runtime.cpp
  src/tf2_ingress.cpp
  src/tf2_runtime.cpp
  src/lifecycle_gate.cpp
  src/readback.cpp
)

# The `rcllf` LTTng UST provider. Optional at build: without
# lttng-ust the emit layer compiles to a no-op sink and no marker is installed.
option(RCLLF_TRACE_PROVIDER "Build the rcllf LTTng UST provider when lttng-ust is found" ON)
set(RCLLF_HAVE_LTTNG FALSE)
if(RCLLF_TRACE_PROVIDER)
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(LTTNG_UST QUIET lttng-ust)
  endif()
  if(LTTNG_UST_FOUND)
    set(RCLLF_HAVE_LTTNG TRUE)
  else()
    message(WARNING "rcllf trace provider disabled: lttng-ust not found")
  endif()
endif()
ament_package_xml()
target_compile_definitions(ros2_lf_binding PRIVATE
  RCLLF_BINDING_VERSION="${ros2_lf_binding_VERSION}")
# Fault-injection test hooks (RCLLF_TEST_CORR_DROP / _DELAY /
# _SKIP_PREDICT) exist only in builds that ask for them. Measurement builds
# keep the default OFF and contain no hook code. The older hooks
# RCLLF_TEST_CORR_DROP_TOKEN / _DROP_SAMPLE / _DRAIN_DELAY are gated too.
option(RCLLF_CORR_TEST_HOOKS "Compile correspondence fault-injection test hooks" OFF)
if(RCLLF_CORR_TEST_HOOKS)
  message(WARNING "ros2_lf_binding: RCLLF_CORR_TEST_HOOKS=ON, fault-injection hooks compiled in")
  target_compile_definitions(ros2_lf_binding PRIVATE RCLLF_CORR_TEST_HOOKS=1)
endif()
if(RCLLF_HAVE_LTTNG)
  target_sources(ros2_lf_binding PRIVATE src/rcllf_tp.c src/rcllf_stop_tp.c)
  target_compile_definitions(ros2_lf_binding PRIVATE RCLLF_HAVE_LTTNG=1)
  target_include_directories(ros2_lf_binding PRIVATE ${LTTNG_UST_INCLUDE_DIRS})
  target_link_libraries(ros2_lf_binding ${LTTNG_UST_LIBRARIES})
endif()

target_include_directories(ros2_lf_binding PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(ros2_lf_binding PRIVATE
  src/
  ${REACTOR_C_DIR}/include
  ${REACTOR_C_DIR}/include/api
  ${REACTOR_C_DIR}/include/core
  ${REACTOR_C_DIR}/include/core/utils
)

# _lf_get_environments is generated by lfc into the final binary that links
# ros2_lf_binding; allow it to be unresolved in the shared library itself.
target_link_options(ros2_lf_binding PRIVATE -Wl,--allow-shlib-undefined)
target_link_libraries(ros2_lf_binding reactor-c lib ${CMAKE_DL_LIBS})
ament_target_dependencies(ros2_lf_binding
  rclcpp
  rclcpp_action
  rclcpp_components
  rcl
  rcl_lifecycle
  composition_interfaces
  nlohmann_json
  rosgraph_msgs
  ros2_lf_msgs
  lifecycle_msgs
  rcl_interfaces
  type_description_interfaces
  rosidl_typesupport_introspection_cpp
)

# --- Install ---
install(TARGETS ros2_lf_binding
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# The generator copies this marker into the static trace manifest.
if(RCLLF_HAVE_LTTNG)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/trace_provider
    "rcllf ${LTTNG_UST_VERSION}\n")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/trace_provider
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# The publication gate is compiled into every generated executable
# (its rcl_* definitions must live in the executable to preempt librcl).
# The clock gate is compiled in next to it (its
# rclcpp::Clock definitions must live in the executable to preempt librclcpp).
install(FILES src/publication_gate.cpp src/clock_gate.cpp src/action_gate.cpp
  DESTINATION share/${PROJECT_NAME}/src
)


# Optional diagnostic: an LD_PRELOAD probe that traces when DDS matches each
# reader (see README.md, "Optional diagnostics"). Needs lttng-ust for its
# `rcllf_dds` provider.
if(RCLLF_HAVE_LTTNG)
  add_library(rcllf_match_probe SHARED src/match_probe_preload.cpp)
  ament_target_dependencies(rcllf_match_probe rcl)
  target_include_directories(rcllf_match_probe PRIVATE src/ ${LTTNG_UST_INCLUDE_DIRS})
  target_link_libraries(rcllf_match_probe ${LTTNG_UST_LIBRARIES} ${CMAKE_DL_LIBS})
  install(TARGETS rcllf_match_probe
    LIBRARY DESTINATION lib
  )
endif()

# --- Export for downstream packages (lfc-generated) ---
ament_export_include_directories(include)
ament_export_libraries(ros2_lf_binding)
ament_export_dependencies(
  rclcpp rclcpp_action rclcpp_components rcl composition_interfaces rosgraph_msgs ros2_lf_msgs
  lifecycle_msgs rcl_interfaces type_description_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # Test-only: exercises the type-erased registry with a lifecycle node.
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)  # test_external_input stamp anchors
  find_package(tf2_msgs REQUIRED)  # test_external_input TFMessage anchor

  # Regression test of publication identity and ownership: a bounded probe
  # executable and its runner script.
  add_executable(publication_identity_probe test/publication_identity_probe.cpp)
  ament_target_dependencies(
    publication_identity_probe
    rclcpp
    rclcpp_lifecycle
    std_msgs
  )
  target_link_libraries(publication_identity_probe nlohmann_json::nlohmann_json)
  add_test(
    NAME publication_identity_probe
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/run_publication_identity_probe.py
      --probe $<TARGET_FILE:publication_identity_probe>
  )
  set_tests_properties(publication_identity_probe PROPERTIES TIMEOUT 120)


  # test_correspondence_state — pure state machine (T2)
  ament_add_gtest(test_correspondence_state test/test_correspondence_state.cpp)
  target_include_directories(test_correspondence_state PRIVATE include src/)
  target_link_libraries(test_correspondence_state ros2_lf_binding)
  target_link_options(test_correspondence_state PRIVATE -Wl,--allow-shlib-undefined)

  # test_bna_runtime — BNA registry, dispatch context, wake and missing
  ament_add_gtest(test_bna_runtime test/test_bna_runtime.cpp test/bna_hooks_layout.cpp)
  target_include_directories(test_bna_runtime PRIVATE include src/)
  target_link_libraries(test_bna_runtime ros2_lf_binding)
  target_link_options(test_bna_runtime PRIVATE -Wl,--allow-shlib-undefined)

  # test_tf2_runtime — rcllf_tf2 adapter registry, wakes, ingest notices
  ament_add_gtest(test_tf2_runtime test/test_tf2_runtime.cpp)
  target_include_directories(test_tf2_runtime PRIVATE include src/)
  target_link_libraries(test_tf2_runtime ros2_lf_binding)
  target_link_options(test_tf2_runtime PRIVATE -Wl,--allow-shlib-undefined)

  # test_tf2_detach_inflight — a tf2 callback in flight at detach (F91-3)
  ament_add_gtest(test_tf2_detach_inflight test/test_tf2_detach_inflight.cpp TIMEOUT 60)
  target_include_directories(test_tf2_detach_inflight PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_tf2_detach_inflight ros2_lf_binding)
  target_link_options(test_tf2_detach_inflight PRIVATE -Wl,--allow-shlib-undefined -rdynamic)

  # test_tf2_reattach — a listener group re-enabled outside LF is detached
  # again and traced at the next LF ingest
  ament_add_gtest(test_tf2_reattach test/test_tf2_reattach.cpp TIMEOUT 60)
  target_include_directories(test_tf2_reattach PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_tf2_reattach ros2_lf_binding)
  target_link_options(test_tf2_reattach PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_tf2_detach_inflight rclcpp rosgraph_msgs)
  ament_target_dependencies(test_tf2_reattach rclcpp rosgraph_msgs)

  # test_semantic_trace — emit layer with a fake sink (T2)
  ament_add_gtest(test_semantic_trace test/test_semantic_trace.cpp)
  target_include_directories(test_semantic_trace PRIVATE include src/)
  target_link_libraries(test_semantic_trace ros2_lf_binding)
  target_link_options(test_semantic_trace PRIVATE -Wl,--allow-shlib-undefined)

  # test_publication_gate — gate, startup checks, ingress + dispatch (T3-T6)
  ament_add_gtest(test_publication_gate
    test/test_publication_gate.cpp src/publication_gate.cpp TIMEOUT 120)
  target_include_directories(test_publication_gate PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_publication_gate ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_publication_gate PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_publication_gate rclcpp rcl std_msgs)

  # test_local_now: a local edge decided when its publish returns, the
  # delivery lock and the local match warmup
  ament_add_gtest(test_local_now
    test/test_local_now.cpp src/publication_gate.cpp TIMEOUT 180)
  target_include_directories(test_local_now PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_local_now ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_local_now PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_local_now rclcpp rcl std_msgs)
  if(RCLLF_CORR_TEST_HOOKS)  # the hook test knows which build it runs in
    target_compile_definitions(test_local_now PRIVATE RCLLF_CORR_TEST_HOOKS=1)
  endif()

  # test_action_channels: counted action channels with the action gate linked
  # in, drain of every sample sent and LOST on a missing one
  find_package(example_interfaces REQUIRED)
  ament_add_gtest(test_action_channels
    test/test_action_channels.cpp src/action_gate.cpp TIMEOUT 120)
  target_include_directories(test_action_channels PRIVATE include src/)
  target_link_libraries(test_action_channels ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_action_channels PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_action_channels rclcpp rclcpp_action example_interfaces)

  # test_service_pairing: in-system requests told from outside ones by the
  # request header GID, responses filtered by the gate's sequence numbers,
  # and the RCLLF-R004 second-server refusal
  ament_add_gtest(test_service_pairing
    test/test_service_pairing.cpp src/action_gate.cpp TIMEOUT 120)
  target_include_directories(test_service_pairing PRIVATE include src/)
  target_link_libraries(test_service_pairing ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_service_pairing PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_service_pairing rclcpp example_interfaces)

  # test_action_client_guard: a goal's callbacks across a deactivate, with
  # ClientBase's send members interposed by the action gate
  ament_add_gtest(test_action_client_guard
    test/test_action_client_guard.cpp src/action_gate.cpp TIMEOUT 120)
  target_include_directories(test_action_client_guard PRIVATE include src/)
  target_link_libraries(test_action_client_guard ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_action_client_guard PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(
    test_action_client_guard rclcpp rclcpp_action example_interfaces)

  # test_match_probe: the probe keys on rmw handles, so a struct rcl copies
  # after init (every rosout publisher) never leaves it a dangling address
  if(TARGET rcllf_match_probe)
    ament_add_gtest(test_match_probe test/test_match_probe.cpp TIMEOUT 60)
    target_link_libraries(test_match_probe rcllf_match_probe ${CMAKE_DL_LIBS})
    ament_target_dependencies(test_match_probe rcl std_msgs)
  endif()

  # test_semantic_producers — rcllf producers with a fake sink (T3)
  # It points g_lf_environments_stub at its own environment (lf_start).
  ament_add_gtest(test_semantic_producers
    test/test_semantic_producers.cpp src/publication_gate.cpp test/lf_environments_stub.cpp
    TIMEOUT 120)
  target_include_directories(test_semantic_producers PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_semantic_producers ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_semantic_producers PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_semantic_producers rclcpp rcl std_msgs)

  # Standing probe: the executable-level publication gate preempts librcl (T0).
  ament_add_gtest(publication_gate_probe test/publication_gate_probe.cpp TIMEOUT 60)
  target_link_options(publication_gate_probe PRIVATE -rdynamic)
  target_link_libraries(publication_gate_probe ${CMAKE_DL_LIBS})
  ament_target_dependencies(publication_gate_probe rclcpp rcl std_msgs)

  # Feasibility gate for the one-scheduler ingress role (T1).
  ament_add_gtest(ingress_role_probe test/ingress_role_probe.cpp TIMEOUT 60)
  target_include_directories(ingress_role_probe PRIVATE include src/)
  target_link_libraries(ingress_role_probe ros2_lf_binding)
  target_link_options(ingress_role_probe PRIVATE -Wl,--allow-shlib-undefined)
  ament_target_dependencies(
    ingress_role_probe
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    composition_interfaces
    lifecycle_msgs
    rcl_interfaces
  )

  # test_thread_role — one-scheduler guard: refusal message, count
  ament_add_gtest(test_thread_role test/test_thread_role.cpp)
  target_include_directories(test_thread_role PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_thread_role ros2_lf_binding)
  target_link_options(test_thread_role PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_thread_role rclcpp)

  # test_ingress — ingress mailbox, wake edges, close/refusal, backstop
  ament_add_gtest(test_ingress test/test_ingress.cpp TIMEOUT 120)
  target_include_directories(test_ingress PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_ingress ros2_lf_binding)
  target_link_options(test_ingress PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_ingress rclcpp rclcpp_components rcl_interfaces std_msgs)

  # test_external_input — external_topics subscriptions via ingress
  ament_add_gtest(test_external_input test/test_external_input.cpp TIMEOUT 120)
  target_include_directories(test_external_input PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_external_input ros2_lf_binding)
  target_link_options(test_external_input PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(
    test_external_input rclcpp std_msgs geometry_msgs tf2_msgs
    rosidl_typesupport_introspection_cpp)

  # test_output_deadline — the per-publisher output deadline check
  ament_add_gtest(test_output_deadline test/test_output_deadline.cpp TIMEOUT 60)
  target_include_directories(test_output_deadline PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_output_deadline ros2_lf_binding)
  target_link_options(test_output_deadline PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_output_deadline rclcpp)

  # test_plant_record — recording with dispositions
  ament_add_gtest(test_plant_record test/test_plant_record.cpp TIMEOUT 60)
  target_include_directories(test_plant_record PRIVATE include src/)
  target_link_libraries(test_plant_record ros2_lf_binding)
  target_link_options(test_plant_record PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_plant_record rclcpp std_msgs)

  # test_plant_replay — reading a recording for admitted-only replay
  ament_add_gtest(test_plant_replay test/test_plant_replay.cpp TIMEOUT 60)
  target_include_directories(test_plant_replay PRIVATE include src/)
  target_link_libraries(test_plant_replay ros2_lf_binding)
  target_link_options(test_plant_replay PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_plant_replay rclcpp)

  # test_bind_check — a subscription without a RID refuses binding
  ament_add_gtest(test_bind_check test/test_bind_check.cpp)
  target_include_directories(test_bind_check PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_bind_check ros2_lf_binding)
  target_link_options(test_bind_check PRIVATE -Wl,--allow-shlib-undefined)
  ament_target_dependencies(test_bind_check rclcpp std_msgs rcl_interfaces)

  # test_rid_table — pure unit test, no ROS2 init required
  ament_add_gtest(test_rid_table test/test_rid_table.cpp)
  target_include_directories(test_rid_table PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_rid_table ros2_lf_binding)
  target_link_options(test_rid_table PRIVATE -Wl,--allow-shlib-undefined)
  ament_target_dependencies(test_rid_table rclcpp)

  # test_readback — the executable exports a stub environment, worker count and
  # patch markers as an lfc program does
  ament_add_gtest(test_readback test/test_readback.cpp)
  target_include_directories(test_readback PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_readback ros2_lf_binding)
  target_link_options(test_readback PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_readback rclcpp nlohmann_json)

  # test_time_source — provides stub _lf_get_environments to inject logical time
  ament_add_gtest(test_time_source test/test_time_source.cpp)
  target_include_directories(test_time_source PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_time_source ros2_lf_binding)
  target_link_options(test_time_source PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_time_source rclcpp)

  # test_node_clock — node clocks follow LF logical time;
  # provides its own _lf_get_environments stub to inject logical time
  ament_add_gtest(test_node_clock test/test_node_clock.cpp TIMEOUT 60)
  target_include_directories(test_node_clock PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_node_clock ros2_lf_binding)
  target_link_options(test_node_clock PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_node_clock
    rclcpp rclcpp_lifecycle rclcpp_components composition_interfaces rosgraph_msgs)

  # test_clock_gate: clock uses rcllf refuses or reports;
  # links the executable gate and provides its own _lf_get_environments stub
  ament_add_gtest(test_clock_gate test/test_clock_gate.cpp src/clock_gate.cpp TIMEOUT 120)
  target_include_directories(test_clock_gate PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_clock_gate ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_clock_gate PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_clock_gate rclcpp rcl)

  # test_reaction_dispatch — guard-path tests for invoke_timer / invoke_sub
  ament_add_gtest(test_reaction_dispatch test/test_reaction_dispatch.cpp)
  target_include_directories(test_reaction_dispatch PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_reaction_dispatch ros2_lf_binding)
  target_link_options(test_reaction_dispatch PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_reaction_dispatch rclcpp)

  # test_node_registry — pure unit test for the non-component node registry
  ament_add_gtest(test_node_registry test/test_node_registry.cpp)
  target_include_directories(test_node_registry PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_node_registry ros2_lf_binding)
  target_link_options(test_node_registry PRIVATE -Wl,--allow-shlib-undefined)
  ament_target_dependencies(test_node_registry rclcpp rclcpp_lifecycle)

  # test_lifecycle_binding — transitions are driven before binding
  # publication_gate.cpp too: generated executables carry it, and it reports
  # lifecycle transitions to the gate (round 3, finding 1).
  # action_gate.cpp: the action client guard needs its interposition.
  ament_add_gtest(test_lifecycle_binding test/test_lifecycle_binding.cpp src/publication_gate.cpp
    src/action_gate.cpp)
  target_include_directories(test_lifecycle_binding PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_lifecycle_binding ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_lifecycle_binding PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  find_package(example_interfaces REQUIRED)
  ament_target_dependencies(
    test_lifecycle_binding rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle rosgraph_msgs
    lifecycle_msgs composition_interfaces example_interfaces)

  # test_lifecycle_gate — lifecycle state per tag, the next-tag rule
  ament_add_gtest(test_lifecycle_gate test/test_lifecycle_gate.cpp)
  target_include_directories(test_lifecycle_gate PRIVATE include src/)
  target_link_libraries(test_lifecycle_gate ros2_lf_binding)
  target_link_options(test_lifecycle_gate PRIVATE -Wl,--allow-shlib-undefined)

  # test_clock_publisher — Off/OnTimeChange/OnEveryReaction mode unit tests
  ament_add_gtest(test_clock_publisher test/test_clock_publisher.cpp)
  target_include_directories(test_clock_publisher PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_clock_publisher ros2_lf_binding)
  target_link_options(test_clock_publisher PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_clock_publisher rclcpp rosgraph_msgs ros2_lf_msgs)

  # test_take_mode — ROS2_LF_TAKE_MODE parsing + skip/block take behavior
  # The suite runs close to the 60 s default (59.5 s locally).
  # 300 s: the real-time polls make the differential fuzz tests take
  # about 80 s on a 24-core host (1 ms take timeout per miss, 1000 schedules).
  # lf_environments_stub.cpp: the stall backstop tests set a stop tag.
  ament_add_gtest(test_take_mode test/test_take_mode.cpp test/lf_environments_stub.cpp
    TIMEOUT 300)
  target_include_directories(test_take_mode PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_take_mode ros2_lf_binding)
  target_link_options(test_take_mode PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_take_mode rclcpp rosgraph_msgs)

  # test_per_subscription_exit — a static PerSubscription holding the entry
  # of a subscription from an unloaded library exits cleanly
  add_library(per_subscription_exit_lib MODULE test/per_subscription_exit_lib.cpp)
  # No STB_GNU_UNIQUE symbols, so dlclose really unloads it, as class_loader does.
  target_compile_options(per_subscription_exit_lib PRIVATE -fno-gnu-unique)
  ament_target_dependencies(per_subscription_exit_lib rclcpp rosgraph_msgs)
  add_executable(per_subscription_exit_probe test/per_subscription_exit_probe.cpp)
  target_include_directories(per_subscription_exit_probe PRIVATE src/)
  target_link_libraries(per_subscription_exit_probe dl)
  ament_target_dependencies(per_subscription_exit_probe rclcpp)
  ament_add_gtest(test_per_subscription_exit test/test_per_subscription_exit.cpp TIMEOUT 60)
  target_compile_definitions(test_per_subscription_exit PRIVATE
    PROBE_PATH="$<TARGET_FILE:per_subscription_exit_probe>"
    PROBE_LIB_PATH="$<TARGET_FILE:per_subscription_exit_lib>")
  add_dependencies(test_per_subscription_exit
    per_subscription_exit_probe per_subscription_exit_lib)

  # test_match_warmup — cross-federate DDS match warm-up, two participants
  ament_add_gtest(test_match_warmup test/test_match_warmup.cpp src/match_warmup.cpp
    src/pair_match.cpp TIMEOUT 60)
  target_include_directories(test_match_warmup PRIVATE src/)
  ament_target_dependencies(test_match_warmup rclcpp std_msgs nlohmann_json)

  # test_policy_table — exec_config.json loading + (node_class, topic) resolution
  ament_add_gtest(test_policy_table test/test_policy_table.cpp)
  target_include_directories(test_policy_table PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_policy_table ros2_lf_binding)
  target_link_options(test_policy_table PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_policy_table rclcpp nlohmann_json)

  # test_component_parsing — parse_component_list: old/new/mixed schema,
  # entry-scoped field extraction (per-instance RIDs runtime slice)
  ament_add_gtest(test_component_parsing test/test_component_parsing.cpp)
  target_include_directories(test_component_parsing PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_component_parsing ros2_lf_binding)
  target_link_options(test_component_parsing PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_component_parsing rclcpp nlohmann_json)

  # test_component_manager — find_by_class exact-match vs. legacy fallback
  # gating, instance-keyed load bookkeeping (per-instance RIDs runtime slice)
  ament_add_gtest(test_component_manager test/test_component_manager.cpp)
  target_include_directories(test_component_manager PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_component_manager ros2_lf_binding)
  target_link_options(test_component_manager PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_component_manager rclcpp rclcpp_components composition_interfaces)

  # test_silence_deadline — the logical deadline window of a silent edge
  ament_add_gtest(test_silence_deadline test/test_silence_deadline.cpp
    src/silence_deadline.cpp TIMEOUT 60)
  target_include_directories(test_silence_deadline PRIVATE src/)
  ament_target_dependencies(test_silence_deadline rclcpp)

  # test_input_period — the declared minimum input period is checked
  ament_add_gtest(test_input_period test/test_input_period.cpp TIMEOUT 60)
  target_include_directories(test_input_period PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_input_period ros2_lf_binding)
  target_link_options(test_input_period PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_input_period rclcpp)

  # test_handle_matching — one to one timer and subscription binding
  ament_add_gtest(test_handle_matching test/test_handle_matching.cpp)
  target_include_directories(test_handle_matching PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_handle_matching ros2_lf_binding)
  target_link_options(test_handle_matching PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_handle_matching
    rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle composition_interfaces rosgraph_msgs
    example_interfaces)

  # test_shutdown_signals — the teardown finalizes the rmw context and a
  # SIGINT/SIGTERM after shutdown began only counts
  ament_add_gtest(test_shutdown_signals test/test_shutdown_signals.cpp TIMEOUT 60)
  target_include_directories(test_shutdown_signals PRIVATE include src/)
  target_link_libraries(test_shutdown_signals ros2_lf_binding)
  target_link_options(test_shutdown_signals PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_shutdown_signals rclcpp)

  # test_stop_bound — a signalled process whose LF stop hangs still ends, and
  # RCLLF_EXIT_WITH_PARENT ends it when its launch dies
  ament_add_gtest(test_stop_bound test/test_stop_bound.cpp TIMEOUT 60)
  target_include_directories(test_stop_bound PRIVATE include src/)
  target_link_libraries(test_stop_bound ros2_lf_binding)
  target_link_options(test_stop_bound PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_stop_bound rclcpp)

  # test_startup_bound — a start whose node construction blocks in the
  # middleware ends at RCLLF_STARTUP_TIMEOUT_MS with code 124
  ament_add_gtest(test_startup_bound test/test_startup_bound.cpp TIMEOUT 60)
  target_include_directories(test_startup_bound PRIVATE include src/)
  target_link_libraries(test_startup_bound ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_startup_bound PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_startup_bound rclcpp)

  # test_signal_end_count — a SIGINT-ended federate still delivers its
  # end-of-run COMPLETE count and ends without an abort
  # publication_gate.cpp: corr startup's gate self-check wants it, as in a
  # generated executable.
  ament_add_gtest(test_signal_end_count test/test_signal_end_count.cpp
    src/publication_gate.cpp TIMEOUT 60)
  target_include_directories(test_signal_end_count PRIVATE
    include
    src/
    ${REACTOR_C_DIR}/include
    ${REACTOR_C_DIR}/include/core
  )
  target_link_libraries(test_signal_end_count ros2_lf_binding ${CMAKE_DL_LIBS})
  target_link_options(test_signal_end_count PRIVATE -Wl,--allow-shlib-undefined -rdynamic)
  ament_target_dependencies(test_signal_end_count rclcpp rcl std_msgs ros2_lf_msgs)
endif()

ament_package()
