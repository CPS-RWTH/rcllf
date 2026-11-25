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


/// @file tf2_runtime_hooks.h
/// @brief The rcllf runtime calls rcllf_tf2::AcquisitionLookup uses.
///
/// Declared weak here, so a node built against this header runs under stock
/// ROS with every symbol null (the adapter then decides on the node clock with
/// its own timer). Under rcllf, ros2_lf_binding defines them, see
/// ros2_lf_binding/tf2_runtime_api.h, which repeats these prototypes. A binding
/// test includes both headers, so a signature drift fails to compile.

#ifndef RCLLF_TF2__TF2_RUNTIME_HOOKS_H_
#define RCLLF_TF2__TF2_RUNTIME_HOOKS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shared with the other header, defined once per translation unit.
#ifndef RCLLF_TF2_TYPES_DEFINED
#define RCLLF_TF2_TYPES_DEFINED
/// What the binding calls back into one registered AcquisitionLookup.
typedef struct rcllf_tf2_hooks
{
  void * ctx;
  /// The generated tf2_wake reaction ran at this tag (a requested deadline).
  void (* wake)(void * ctx, int64_t time, uint32_t micro);
  /// A /tf or /tf_static ingest reaction of the node fed the Buffer at this tag.
  void (* ingested)(void * ctx, int64_t time, uint32_t micro);
  /// The generated shutdown reaction ran at this tag.
  void (* close)(void * ctx, int64_t time, uint32_t micro);
} rcllf_tf2_hooks;

#define RCLLF_TF2_NOT_CONSTRUCTING (-1)  // register outside a node construction
#endif  // RCLLF_TF2_TYPES_DEFINED

/// 1 when an rcllf runtime is running in this process.
__attribute__((weak)) int ros2_lf_tf2_active(void);
/// Register an adapter of the node under construction. Several
/// adapters per node are allowed. Returns a handle >= 0 or
/// RCLLF_TF2_NOT_CONSTRUCTING.
__attribute__((weak)) int ros2_lf_tf2_register(const rcllf_tf2_hooks * hooks);
/// Forget a handle. No hook of it runs after this returns.
__attribute__((weak)) void ros2_lf_tf2_unregister(int handle);
/// The current LF tag; returns 0, or -1 outside an LF reaction (a thread that
/// is not an LF worker).
__attribute__((weak)) int ros2_lf_tf2_current(int64_t * time, uint32_t * micro);
/// Ask for a wake at logical time `deadline` (microstep 0).
__attribute__((weak)) void ros2_lf_tf2_request_wake(int handle, int64_t deadline);
/// One `RCLLF_TF2` JSON line, to stderr and the binding trace channel.
__attribute__((weak)) void ros2_lf_tf2_trace(const char * json);
/// execution.yaml `nodes.<node>.tf2` for the adapter `handle`:
/// 1 when set, then `*wait_bound_ns` (> 0, else unset) and `on_timeout`
/// ("" unset, else "reject", "failure" or "fallback:<name>", at most `cap`
/// bytes with the NUL); 0 when the node has no such keys.
__attribute__((weak)) int ros2_lf_tf2_config(
  int handle, int64_t * wait_bound_ns, char * on_timeout, uint32_t cap);

#ifdef __cplusplus
}
#endif

#endif  // RCLLF_TF2__TF2_RUNTIME_HOOKS_H_
