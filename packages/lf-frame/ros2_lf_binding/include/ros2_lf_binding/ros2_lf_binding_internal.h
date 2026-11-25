// Copyright 2026 rcllf Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file ros2_lf_binding_internal.h
/// @brief C++ API for generated ros2_lf_reaction_ids_init().
///
/// Included only by lfc-generated code (ros2_lf_reaction_ids_init.cpp).
/// Not part of the public C API. This header is for generated code only and
/// is not a stable API: it may change in any release without notice.

#ifndef ROS2_LF_BINDING__ROS2_LF_BINDING_INTERNAL_H_
#define ROS2_LF_BINDING__ROS2_LF_BINDING_INTERNAL_H_

#include <cstddef>
#include <cstdint>

// Pull in the RidEntry type and registration functions from the internal
// rid_table implementation.  The generated file lives inside the same
// colcon package as ros2_lf_binding so we can include internal headers.
#include "ros2_lf_binding/rid_table_api.h"

#endif  // ROS2_LF_BINDING__ROS2_LF_BINDING_INTERNAL_H_
