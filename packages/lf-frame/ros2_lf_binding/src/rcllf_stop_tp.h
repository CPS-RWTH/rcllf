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

/// @file rcllf_stop_tp.h
/// @brief LTTng UST provider `rcllf_stop`: one `mark` event per
/// milestone of the controlled shutdown in the process that reaches it:
/// "stop_requested", "egress_drain_begin/end", "ingress_drain_begin/end",
/// "ingress_joined", "reconciled", "context_fini" (`detail`: the rcl return
/// code of the context finalization, else 0). Together with
/// `rcllf:publication` and `rcllf:sample` it gives the stop timeline of a run.

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER rcllf_stop

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./rcllf_stop_tp.h"

// NOLINT(build/header_guard): LTTng multi-read provider header, no plain guard
#if !defined(RCLLF_STOP_TP_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define RCLLF_STOP_TP_H_

#include <lttng/tracepoint.h>

#include <stdint.h>

TRACEPOINT_EVENT(
  rcllf_stop, mark,
  TP_ARGS(const char *, what, int64_t, detail),
  TP_FIELDS(
    ctf_string(what, what)
    ctf_integer(int64_t, detail, detail)))

#endif  // RCLLF_STOP_TP_H_

#include <lttng/tracepoint-event.h>
