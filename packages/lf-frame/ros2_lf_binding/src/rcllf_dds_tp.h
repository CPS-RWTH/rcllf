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

/// @file rcllf_dds_tp.h
/// @brief LTTng UST provider `rcllf_dds`: when DDS matched a
/// reader to a writer and a writer to more readers, each bounded by the last
/// poll that found the lower count. `source` says what detected the increase:
/// "event" (the rmw matched event callback) or "poll". Used only by
/// the match probe preload, never by rcllf.

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER rcllf_dds

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./rcllf_dds_tp.h"

// NOLINT(build/header_guard): LTTng multi-read provider header, no plain guard
#if !defined(RCLLF_DDS_TP_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define RCLLF_DDS_TP_H_

#include <lttng/tracepoint.h>

#include <stdint.h>

TRACEPOINT_EVENT(
  rcllf_dds, reader_matched,
  TP_ARGS(
    const void *, rmw_subscription_handle, const char *, topic_name, uint64_t, publishers,
    int64_t, unmatched_age_ns, const char *, source),
  TP_FIELDS(
    ctf_integer_hex(const void *, rmw_subscription_handle, rmw_subscription_handle)
    ctf_string(topic_name, topic_name)
    ctf_integer(uint64_t, publishers, publishers)
    // ns from the last observation that found no matched publisher to this event
    ctf_integer(int64_t, unmatched_age_ns, unmatched_age_ns)
    ctf_string(source, source)))

// One event per increase of a writer's matched subscription count.
TRACEPOINT_EVENT(
  rcllf_dds, writer_matched,
  TP_ARGS(
    const void *, rmw_publisher_handle, const char *, topic_name, uint64_t, subscriptions,
    int64_t, unmatched_age_ns, const char *, source),
  TP_FIELDS(
    ctf_integer_hex(const void *, rmw_publisher_handle, rmw_publisher_handle)
    ctf_string(topic_name, topic_name)
    ctf_integer(uint64_t, subscriptions, subscriptions)
    // ns from the last observation that found fewer matched subscriptions to this event
    ctf_integer(int64_t, unmatched_age_ns, unmatched_age_ns)
    ctf_string(source, source)))

#endif  // RCLLF_DDS_TP_H_

#include <lttng/tracepoint-event.h>
