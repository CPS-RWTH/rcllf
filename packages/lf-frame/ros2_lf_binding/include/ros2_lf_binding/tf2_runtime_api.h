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


/// @file tf2_runtime_api.h
/// @brief Binding side of rcllf_tf2::AcquisitionLookup.
///
/// The first block repeats the prototypes rcllf_tf2 declares weak in
/// tf2_runtime_hooks.h. test_tf2_runtime includes both headers, so a signature
/// drift fails to compile. The second block is called by generated code: the
/// wake scheduling loop after every user reaction of a node with an adapter,
/// the ingest notice after its /tf and /tf_static reactions, and the wake and
/// shutdown reactions.

#ifndef ROS2_LF_BINDING__TF2_RUNTIME_API_H_
#define ROS2_LF_BINDING__TF2_RUNTIME_API_H_

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

// -- called by the adapter (weak on its side) --------------------------------

int ros2_lf_tf2_active(void);
int ros2_lf_tf2_register(const rcllf_tf2_hooks * hooks);
void ros2_lf_tf2_unregister(int handle);
int ros2_lf_tf2_current(int64_t * time, uint32_t * micro);
void ros2_lf_tf2_request_wake(int handle, int64_t deadline);
void ros2_lf_tf2_trace(const char * json);
int ros2_lf_tf2_config(int handle, int64_t * wait_bound_ns, char * on_timeout, uint32_t cap);

// -- called by generated code ------------------------------------------------

/// The delay (ns, from the current tag) of the earliest requested wake of
/// `node_key` not yet scheduled, marking it scheduled, or -1 when none is
/// left. Generated code schedules `tf2_wake` once per returned value.
int64_t ros2_lf_tf2_take_wake(const char * node_key);

/// The `tf2_wake` reaction: every adapter of the node decides its due requests.
void ros2_lf_tf2_wake(const char * node_key);

/// After a /tf or /tf_static ingest reaction: every adapter of the node
/// re-evaluates its pending requests against the Buffer.
void ros2_lf_tf2_ingested(const char * node_key);

/// The shutdown reaction: every adapter of the node traces its open requests.
void ros2_lf_tf2_close(const char * node_key);

#ifdef __cplusplus
}
#endif

#endif  // ROS2_LF_BINDING__TF2_RUNTIME_API_H_
