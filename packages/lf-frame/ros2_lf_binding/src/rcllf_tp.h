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

/// @file rcllf_tp.h
/// @brief LTTng UST provider `rcllf` (D1). Every event and field is
/// generated from rcllf_trace_schema.h, so the wire format cannot drift from
/// the schema. Only included when the binding is built with lttng-ust.

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER rcllf

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./rcllf_tp.h"

// NOLINT(build/header_guard): LTTng multi-read provider header, no plain guard
#if !defined(RCLLF_TP_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define RCLLF_TP_H_

#include <lttng/tracepoint.h>

#include "rcllf_tp_structs.h"  // NOLINT(build/include_subdir)

#define RCLLF_TP_F_int(type, name) ctf_integer(type, name, e->name)
#define RCLLF_TP_F_text(type, name) ctf_string(name, e->name)
#define RCLLF_TP_F_gid(type, name) ctf_array(uint8_t, name, e->name, RCLLF_TRACE_GID_LEN)
#define RCLLF_TP_F_totals(type, name) ctf_array(uint64_t, name, e->name, RCLLF_CAT_COUNT)
#define RCLLF_TP_F(kind, type, name) RCLLF_TP_F_ ## kind(type, name)

#define RCLLF_TP_EVENT(ev, catv) \
  TRACEPOINT_EVENT( \
    rcllf, ev, \
    TP_ARGS(uint64_t, epoch, uint64_t, seq, const struct rcllf_tp_ ## ev *, e), \
    TP_FIELDS( \
      ctf_integer(uint64_t, epoch, epoch) \
      ctf_integer(uint8_t, cat, catv) \
      ctf_integer(uint64_t, seq, seq) \
      RCLLF_FIELDS_ ## ev(RCLLF_TP_F)))

RCLLF_TRACE_EVENTS(RCLLF_TP_EVENT)

#endif  // RCLLF_TP_H_

#include <lttng/tracepoint-event.h>
