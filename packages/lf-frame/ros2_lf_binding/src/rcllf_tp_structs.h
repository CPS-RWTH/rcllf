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

/// @file rcllf_tp_structs.h
/// @brief One plain C struct per `rcllf` event, generated from the schema
/// field lists. Producers fill these and hand them to the emit
/// layer; the provider reads them field by field.
#ifndef RCLLF_TP_STRUCTS_H_
#define RCLLF_TP_STRUCTS_H_

#include <stdint.h>

#include "ros2_lf_binding/rcllf_trace_schema.h"

#define RCLLF_ST_F_int(type, name) type name;
#define RCLLF_ST_F_text(type, name) type name;
#define RCLLF_ST_F_gid(type, name) type name[RCLLF_TRACE_GID_LEN];
#define RCLLF_ST_F_totals(type, name) type name[RCLLF_CAT_COUNT];
#define RCLLF_ST_F(kind, type, name) RCLLF_ST_F_ ## kind(type, name)
#define RCLLF_ST_EVENT(ev, catv) struct rcllf_tp_ ## ev {RCLLF_FIELDS_ ## ev(RCLLF_ST_F)};

RCLLF_TRACE_EVENTS(RCLLF_ST_EVENT)

#undef RCLLF_ST_EVENT

#endif  // RCLLF_TP_STRUCTS_H_
