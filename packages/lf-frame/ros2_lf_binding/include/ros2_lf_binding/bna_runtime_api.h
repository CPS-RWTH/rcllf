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

/// @file bna_runtime_api.h
/// @brief Binding side of the BNA synchronizer.
///
/// The first block repeats the prototypes rcllf_message_filters declares weak
/// in bna_runtime_hooks.h. test_bna_runtime includes both headers, so a
/// signature drift fails to compile. The second block is called by generated
/// code: a startup declaration per sync input drain, the wake scheduling loop
/// and the wake and shutdown reactions.

#ifndef ROS2_LF_BINDING__BNA_RUNTIME_API_H_
#define ROS2_LF_BINDING__BNA_RUNTIME_API_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shared with the other header, defined once per translation unit.
#ifndef RCLLF_BNA_TYPES_DEFINED
#define RCLLF_BNA_TYPES_DEFINED
typedef struct rcllf_bna_hooks
{
  void * ctx;
  void (* missing)(
    void * ctx, int input, uint64_t epoch, const char * gid, uint64_t ordinal,
    int64_t time, uint32_t micro);
  void (* wake)(void * ctx, int64_t time, uint32_t micro);
  void (* close)(void * ctx, const char * reason, int64_t time, uint32_t micro);
} rcllf_bna_hooks;

typedef void (* rcllf_bna_missing_stamped_fn)(
  void * ctx, int input, uint64_t epoch, const char * gid, uint64_t ordinal,
  int64_t stamp, int64_t time, uint32_t micro);

typedef struct rcllf_bna_dispatch
{
  int64_t time;
  uint32_t micro;
  uint64_t epoch;
  uint64_t ordinal;
  char gid[64];
} rcllf_bna_dispatch;

#define RCLLF_BNA_OK 0
#define RCLLF_BNA_NOT_CONSTRUCTING (-1)
#define RCLLF_BNA_MULTIPLE (-2)
#define RCLLF_BNA_NOT_DRAIN (-3)
#define RCLLF_BNA_WRONG_INPUT (-4)
#endif  // RCLLF_BNA_TYPES_DEFINED

// -- called by the policy (weak on its side) ---------------------------------

int ros2_lf_bna_active(void);
int ros2_lf_bna_register(const rcllf_bna_hooks * hooks, uint64_t * epoch);
void ros2_lf_bna_set_missing_stamped(int handle, rcllf_bna_missing_stamped_fn fn);
void ros2_lf_bna_unregister(int handle);
int ros2_lf_bna_current(int handle, int input, rcllf_bna_dispatch * out);
void ros2_lf_bna_request_wake(int handle, int64_t deadline);
void ros2_lf_bna_trace(const char * json);

// -- called by generated code ------------------------------------------------

/// Startup: the corr drain reaction `reaction_id` delivers BNA input `input`
/// (0 anchor, 1 candidate) of the policy in node `node_key`.
void ros2_lf_bna_declare_input(const char * node_key, int reaction_id, int input);

/// Startup: the BNA input drained by `reaction_id` has
/// stamp_provenance lf_clock and a declared max_stamp_lag: every header stamp
/// is at least its publish tag minus `max_stamp_lag_ns`. A lost message then
/// raises the engine's watermark to that bound, never above the true stamp.
/// Inputs without this declaration keep the plain missing report.
void ros2_lf_bna_input_stamp_lag(int reaction_id, int64_t max_stamp_lag_ns);

/// The delay (ns, from the current tag) of the earliest requested wake of
/// `node_key` not yet scheduled, marking it scheduled, or -1 when none is
/// left. Generated code schedules `bna_wake` once per returned value.
int64_t ros2_lf_bna_take_wake(const char * node_key);

/// The `bna_wake` reaction: run the engine's due deadlines at this tag.
void ros2_lf_bna_wake(const char * node_key);

/// The shutdown reaction: close the engine with reason `shutdown`.
void ros2_lf_bna_close(const char * node_key);

#ifdef __cplusplus
}
#endif

#endif  // ROS2_LF_BINDING__BNA_RUNTIME_API_H_
