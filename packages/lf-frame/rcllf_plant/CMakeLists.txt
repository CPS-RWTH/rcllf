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

cmake_minimum_required(VERSION 3.18)
project(rcllf_plant)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # No multiply-add contraction into FMA (GCC's default on aarch64): the plant
  # must round the same on every CPU, as brake_fusion does.
  add_compile_options(-Wall -Wextra -Wpedantic -ffp-contract=off)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

# MuJoCo, fetched as aeb_mujoco_sim does: the pinned official Linux release, checked against its
# SHA-256, unpacked into the build directory. Set MUJOCO_ROOT to build offline.
set(MUJOCO_VERSION 3.14.0)
# One official tarball per CPU, each with its published SHA-256 (aarch64 for the
# Jetson Orin boards).
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(_mj_arch x86_64)
  set(MUJOCO_SHA256 326f0da78a7767cc18fab7205c993a771b5f19eda913648f958ee3f625240944)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(_mj_arch aarch64)
  set(MUJOCO_SHA256 1bdbc32c310c6de38664bbd3bcb01f939abc70c1e9d6402ae3442664d1e0da4c)
endif()
set(MUJOCO_ROOT "" CACHE PATH "Unpacked MuJoCo ${MUJOCO_VERSION} release (downloaded when empty)")
if(NOT MUJOCO_ROOT)
  if(NOT _mj_arch)
    message(FATAL_ERROR "rcllf_plant pins the x86_64 and aarch64 MuJoCo releases, set MUJOCO_ROOT")
  endif()
  set(_mj_name mujoco-${MUJOCO_VERSION})
  set(_mj_tar ${CMAKE_BINARY_DIR}/${_mj_name}-linux-${_mj_arch}.tar.gz)
  if(NOT EXISTS ${CMAKE_BINARY_DIR}/${_mj_name}/lib/libmujoco.so.${MUJOCO_VERSION})
    file(DOWNLOAD
      https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${_mj_name}-linux-${_mj_arch}.tar.gz
      ${_mj_tar}
      EXPECTED_HASH SHA256=${MUJOCO_SHA256}
      STATUS _mj_status)
    list(GET _mj_status 0 _mj_code)
    if(NOT _mj_code EQUAL 0)
      message(FATAL_ERROR "MuJoCo download failed: ${_mj_status}")
    endif()
    file(ARCHIVE_EXTRACT INPUT ${_mj_tar} DESTINATION ${CMAKE_BINARY_DIR}
      PATTERNS ${_mj_name}/include ${_mj_name}/lib)
  endif()
  set(MUJOCO_ROOT ${CMAKE_BINARY_DIR}/${_mj_name})
endif()
add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
  IMPORTED_LOCATION ${MUJOCO_ROOT}/lib/libmujoco.so.${MUJOCO_VERSION}
  IMPORTED_SONAME libmujoco.so.${MUJOCO_VERSION}
  INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_ROOT}/include)

set(_inc
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# The Transactor. rcllf_plant_fault is the same source with the A4 fault hook
# (RCLLF_PLANT_FAULT) compiled in. Only test executables link it (brake_fusion's
# simulator_fault_node under BUILD_TESTING); rcllf_plant refuses to start when
# the variable is set, so a production binary never honours a fault.
add_library(rcllf_plant SHARED src/transactor.cpp)
target_include_directories(rcllf_plant PUBLIC ${_inc})
target_link_libraries(rcllf_plant Threads::Threads)
if(BUILD_TESTING)
  # Built, installed and exported only for tests: a release install tree has
  # no Transactor that honours RCLLF_PLANT_FAULT.
  add_library(rcllf_plant_fault SHARED src/transactor.cpp)
  target_include_directories(rcllf_plant_fault PUBLIC ${_inc})
  target_compile_definitions(rcllf_plant_fault PRIVATE RCLLF_PLANT_FAULT_INJECTION)
  target_link_libraries(rcllf_plant_fault Threads::Threads)
  install(TARGETS rcllf_plant_fault
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endif()

# The MuJoCo backend. Its header keeps the MuJoCo types out (forward
# declarations), so users need no MuJoCo include path.
add_library(rcllf_plant_mujoco SHARED src/mujoco_backend.cpp)
target_include_directories(rcllf_plant_mujoco PUBLIC ${_inc})
target_link_libraries(rcllf_plant_mujoco PRIVATE mujoco)
set_target_properties(rcllf_plant_mujoco PROPERTIES INSTALL_RPATH "$ORIGIN")


install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS rcllf_plant rcllf_plant_mujoco
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# The backend loads MuJoCo by its SONAME, so only the versioned file ships.
install(FILES ${MUJOCO_ROOT}/lib/libmujoco.so.${MUJOCO_VERSION} DESTINATION lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # A2 (no request, no advancement), A3 (stale and duplicate requests),
  # MISMATCH, clock tolerance and the in-process timeout.
  ament_add_gtest(test_transactor test/test_transactor.cpp TIMEOUT 120)
  target_link_libraries(test_transactor rcllf_plant rcllf_plant_mujoco)
  ament_add_gtest(test_mujoco_backend test/test_mujoco_backend.cpp TIMEOUT 120)
  target_link_libraries(test_mujoco_backend rcllf_plant rcllf_plant_mujoco)
  ament_add_gtest(test_transactor_fault test/test_transactor_fault.cpp TIMEOUT 120)
  target_link_libraries(test_transactor_fault rcllf_plant_fault rcllf_plant_mujoco)
endif()

ament_package()
