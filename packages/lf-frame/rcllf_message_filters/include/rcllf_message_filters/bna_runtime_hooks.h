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

/// @file bna_runtime_hooks.h
/// @brief The rcllf runtime calls the BNA policy uses.
///
/// Declared weak here, so a node built against this header runs under stock
/// ROS with every symbol null (the policy then uses its own clock and timer
/// thread). Under rcllf, ros2_lf_binding defines them, see
/// ros2_lf_binding/bna_runtime_api.h, which repeats these prototypes. A
/// binding test includes both headers, so a signature drift fails to compile.

#ifndef RCLLF_MESSAGE_FILTERS__BNA_RUNTIME_HOOKS_H_
#define RCLLF_MESSAGE_FILTERS__BNA_RUNTIME_HOOKS_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shared with the other header, defined once per translation unit.
#ifndef RCLLF_BNA_TYPES_DEFINED
#define RCLLF_BNA_TYPES_DEFINED
/// What the binding calls back into one registered policy.
typedef struct rcllf_bna_hooks
{
  void * ctx;
  /// A join on a declared input drain ended missing, after the deadline
  /// and message_lost handlers ran.
  void (* missing)(
    void * ctx, int input, uint64_t epoch, const char * gid, uint64_t ordinal,
    int64_t time, uint32_t micro);
  /// The generated wake reaction ran at this tag.
  void (* wake)(void * ctx, int64_t time, uint32_t micro);
  /// The generated shutdown reaction ran at this tag.
  void (* close)(void * ctx, const char * reason, int64_t time, uint32_t micro);
} rcllf_bna_hooks;

/// `missing` with a lower bound of the lost sample's header stamp
/// (its publish tag minus the input's declared max_stamp_lag). Not a field of
/// rcllf_bna_hooks, so a policy built against the 4-pointer struct keeps its
/// layout: it is set by ros2_lf_bna_set_missing_stamped after registering.
typedef void (* rcllf_bna_missing_stamped_fn)(
  void * ctx, int input, uint64_t epoch, const char * gid, uint64_t ordinal,
  int64_t stamp, int64_t time, uint32_t micro);

/// The current corr dispatch, filled by ros2_lf_bna_current.
typedef struct rcllf_bna_dispatch
{
  int64_t time;
  uint32_t micro;
  uint64_t epoch;
  uint64_t ordinal;
  char gid[64];
} rcllf_bna_dispatch;

#define RCLLF_BNA_OK 0
#define RCLLF_BNA_NOT_CONSTRUCTING (-1)  // register outside a node construction
#define RCLLF_BNA_MULTIPLE (-2)          // a second BNA policy in one node
#define RCLLF_BNA_NOT_DRAIN (-3)         // admission outside a corr drain
#define RCLLF_BNA_WRONG_INPUT (-4)       // the drain is not declared as this input
#endif  // RCLLF_BNA_TYPES_DEFINED

/// 1 when an rcllf runtime is running in this process.
__attribute__((weak)) int ros2_lf_bna_active(void);

/// Register the policy of the node under construction.
/// Returns a handle >= 0 and the run epoch, or a negative RCLLF_BNA_* code.
__attribute__((weak)) int ros2_lf_bna_register(const rcllf_bna_hooks * hooks, uint64_t * epoch);

/// Route the missing reports of `handle` that know a stamp bound
/// to `fn` (nullptr restores the plain `missing` hook). A binding without
/// this symbol leaves every report on `missing`.
__attribute__((weak)) void ros2_lf_bna_set_missing_stamped(
  int handle, rcllf_bna_missing_stamped_fn fn);

/// Forget a handle. No hook of it runs after this returns.
__attribute__((weak)) void ros2_lf_bna_unregister(int handle);

/// The tag and key of the drain dispatching input `input` on this thread.
/// RCLLF_BNA_OK, RCLLF_BNA_NOT_DRAIN or RCLLF_BNA_WRONG_INPUT.
__attribute__((weak)) int ros2_lf_bna_current(int handle, int input, rcllf_bna_dispatch * out);

/// Ask for a wake at logical time `deadline` (microstep 0).
__attribute__((weak)) void ros2_lf_bna_request_wake(int handle, int64_t deadline);

/// One `RCLLF_BNA` JSON line, to stderr and the corr trace channel.
__attribute__((weak)) void ros2_lf_bna_trace(const char * json);

#ifdef __cplusplus
}
#endif

#endif  // RCLLF_MESSAGE_FILTERS__BNA_RUNTIME_HOOKS_H_
