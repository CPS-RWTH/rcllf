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
"""Python mirror of `rcllf_trace_schema.h` (schema `rcllf.events/1`).

The C header is the source of truth. `tests/semantic/test_schema_parity.py`
parses it and fails naming the first drifting event, field or code.
"""

from __future__ import annotations

SCHEMA_NAME = "rcllf.events"
SCHEMA_MAJOR = 1
SCHEMA_MINOR = 8
PROVIDER_VERSION = 1
GID_LEN = 16
NO_EDGE = 0xFFFFFFFF
NO_RID = -1

COMMON_FIELDS = ("epoch", "cat", "seq")

CATEGORIES = {
    "LIFECYCLE": 0,
    "PUBLICATION": 1,
    "DELIVERY": 2,
    "SAMPLE": 3,
    "DISPATCH": 4,
    "CLEANUP": 5,
    "POLICY": 6,
    "TUPLE": 7,
}
CAT_COUNT = 8
CAT_NAMES = {v: k.lower() for k, v in CATEGORIES.items()}

OUTCOMES = {
    "PENDING": 0,
    "MATCHED": 1,
    "NOT_READY": 2,
    "TIMED_OUT": 3,
    "LOST": 4,
    "REJECTED": 5,
    "UNBOUND": 6,
    "UNSUPPORTED": 7,
    "DUPLICATE": 8,
    "OVERFLOW": 9,
    "EPOCH_CLOSE": 10,
    "PUSH_OVERFLOW": 11,
    "SCHEDULE_REFUSED": 12,
    "READER_REPLACED": 13,
    "NONE": 255,
}
SAMPLE_OUTCOMES = {
    "RETAINED": 0,
    "UNSUPPORTED": 1,
    "UNRESOLVED": 2,
    "LATE": 3,
    "DUPLICATE": 4,
    "REJECTED_IDENTITY": 5,
    "OVERFLOW": 6,
    "EPOCH_CLOSE": 7,
    "STALE_READER": 8,
    "POOLED": 9,  # held for a member that does not know its writer yet
}
ORIGINS = {
    "ADMITTED": 0,
    "NO_CONTEXT": 1,
    "UNDECLARED_EFFECT": 2,
    "EDGE_NOT_ADMITTED": 3,
    "UNSUPPORTED_API": 4,
    "NESTED": 5,
}
PUB_RESULTS = {"ACCEPTED": 0, "FAILED": 1}
APIS = {"PUBLISH": 0, "SERIALIZED": 1, "LOANED": 2}
ENTRY_KINDS = {"TIMER": 0, "LEGACY_SUB": 1, "CORR_SUB": 2, "SERVICE": 3, "ACTION": 4,
               "EXTERNAL_SUB": 5}
CARRIER_OPS = {"EMIT": 0, "ARRIVE": 1, "DRAIN": 2, "REFUSED": 3}
CLEANUP_REASONS = {"SHUTDOWN": 0, "EPOCH_CLOSE": 1}
#: dispatch_end.missing_hook
MISSING_HOOKS = {"NA": 0, "NONE": 1, "SHUTDOWN": 2, "DEADLINE": 3, "MESSAGE_LOST": 4}
#: dispatch_end.cutoff of a dispatch without a cutoff. A sentinel, never a tag.
NO_CUTOFF = -1


def cutoff_elapsed(cutoff: int, lf_start_time_ns: int) -> int | None:
    """``dispatch_end.cutoff`` as ns after ``lf_start``, ``None`` for :data:`NO_CUTOFF`.

    The trace_corr JSON names the same value ``cutoff_elapsed`` (-1 for none).
    A dispatch has no cutoff on a local_now edge (writer and reader in one
    process, decided when the publish returns), on an edge without a timeout and
    when it is not a correspondence drain. A local edge that rebinds its
    subscription or is deferred is not local_now and has a cutoff when it has a
    timeout. Subtracting the start tag from the sentinel
    gives a large negative number that looks like a tag.
    """
    cutoff = int(cutoff)
    return None if cutoff == NO_CUTOFF else cutoff - int(lf_start_time_ns)
#: What became of a message on an external topic.
EXT_DISPOSITIONS = {
    "ARRIVAL": 0, "ADMITTED": 1, "LATE": 2, "OVERRUN": 3, "COLLISION": 4,
    "AFTER_STOP": 5, "REFUSED": 6,
    # A TFMessage whose transforms carry no single stamp.
    "TF_MIXED_STAMPS": 7,
}

#: enum name in the header -> (C prefix, mirror table)
ENUMS = {
    "rcllf_trace_cat": ("RCLLF_CAT_", CATEGORIES),
    "rcllf_trace_outcome": ("RCLLF_OUTCOME_", OUTCOMES),
    "rcllf_trace_sample_outcome": ("RCLLF_SAMPLE_", SAMPLE_OUTCOMES),
    "rcllf_trace_origin": ("RCLLF_ORIGIN_", ORIGINS),
    "rcllf_trace_pub_result": ("RCLLF_PUB_", PUB_RESULTS),
    "rcllf_trace_api": ("RCLLF_API_", APIS),
    "rcllf_trace_entry_kind": ("RCLLF_ENTRY_", ENTRY_KINDS),
    "rcllf_trace_carrier_op": ("RCLLF_CARRIER_", CARRIER_OPS),
    "rcllf_trace_cleanup_reason": ("RCLLF_CLEANUP_", CLEANUP_REASONS),
    "rcllf_trace_missing_hook": ("RCLLF_MISSING_HOOK_", MISSING_HOOKS),
    "rcllf_trace_ext_disposition": ("RCLLF_EXT_", EXT_DISPOSITIONS),
}

#: event -> (category, ((ctf_kind, name), ...)) beyond COMMON_FIELDS
EVENTS: dict[str, tuple[str, tuple[tuple[str, str], ...]]] = {
    "run_begin": ("LIFECYCLE", (
        ("int", "schema_major"), ("int", "schema_minor"), ("text", "binding_version"),
        ("text", "static_manifest_hash"), ("text", "generated_source_hash"),
        ("text", "model_hash"), ("text", "build_id"), ("text", "executable"),
        ("int", "pid"), ("text", "boot_id"), ("text", "host"),
        ("int", "clock_monotonic_ns"), ("int", "clock_realtime_ns"),
        ("int", "clock_lf_physical_ns"), ("int", "clock_read_gap_ns"),
        ("int", "res_monotonic_ns"), ("int", "res_realtime_ns"),
        ("int", "lf_start_time_ns"), ("int", "lf_clock_sync"), ("text", "ancestry"),
        ("text", "run_token"),
    )),
    "provider": ("LIFECYCLE", (
        ("int", "category"), ("int", "provider_version"), ("int", "compiled_in"),
    )),
    "backlog": ("LIFECYCLE", (
        ("gid", "gid"), ("int", "first_ordinal"), ("int", "last_ordinal"), ("int", "refused"),
    )),
    "lf_start": ("LIFECYCLE", (
        ("int", "lf_start_time_ns"), ("int", "lf_start_microstep"),
    )),
    "publication": ("PUBLICATION", (
        ("gid", "gid"), ("int", "ordinal"), ("int", "result"), ("int", "api"),
        ("int", "origin"), ("int", "rid"), ("int", "dispatch_id"),
        ("int", "tag_time"), ("int", "tag_microstep"),
    )),
    "expectation": ("DELIVERY", (
        ("gid", "gid"), ("int", "ordinal"), ("int", "edge"), ("int", "instance"),
        ("int", "generation"),
    )),
    "carrier": ("DELIVERY", (
        ("int", "edge"), ("int", "op"), ("int", "count"), ("int", "release_tag_time"),
    )),
    "terminal": ("DELIVERY", (
        ("gid", "gid"), ("int", "ordinal"), ("int", "edge"), ("int", "outcome"),
    )),
    "sample": ("SAMPLE", (
        ("int", "edge"), ("gid", "gid"), ("int", "ordinal"), ("int", "outcome"),
        ("int", "retained_id"), ("int", "arrival_physical_ns"),
    )),
    "dispatch_begin": ("DISPATCH", (
        ("int", "dispatch_id"), ("int", "rid"), ("int", "entry_kind"), ("int", "edge"),
        ("gid", "gid"), ("int", "ordinal"), ("int", "tag_time"), ("int", "tag_microstep"),
    )),
    "dispatch_end": ("DISPATCH", (
        ("int", "dispatch_id"), ("int", "outcome"), ("int", "retained_id"),
        ("int", "join_wait_ns"), ("int", "callback_ran"),
        # Cutoff tag time (-1 none) and missing_hook code.
        ("int", "cutoff"), ("int", "missing_hook"),
    )),
    "cleanup": ("CLEANUP", (
        ("int", "released_retained"), ("int", "carrier_records_closed"), ("int", "reason"),
    )),
    # One message on a declared external topic.
    "external_arrival": ("SAMPLE", (
        ("int", "rid"), ("gid", "gid"), ("int", "ordinal"), ("int", "record_id"),
        ("int", "arrival_physical_ns"),
        # The stamp anchor's view of the message.
        ("int", "stamp_ns"), ("int", "tag_ns"), ("int", "insert_ns"), ("int", "disposition"),
    )),
    # An infrastructure service request handed to its
    # service by the ingress drain, the callback_start JSON line as an event.
    "ingress_callback_start": ("DISPATCH", (
        ("text", "entry"), ("int", "rid"), ("text", "node"), ("text", "callback"),
        ("int", "lf_thread_id"), ("int", "gettid"), ("int", "tag_time"),
        ("int", "tag_microstep"), ("int", "ingress_record_id"),
    )),
    # A late publish of a topic with output_deadline_ns,
    # published anyway; lateness_ns = physical_ns - tag_time > bound_ns.
    "actuation_late": ("PUBLICATION", (
        ("int", "rid"), ("text", "topic"), ("int", "tag_time"), ("int", "tag_microstep"),
        ("int", "physical_ns"), ("int", "lateness_ns"), ("int", "bound_ns"),
    )),
    # rcllf_tf2::AcquisitionLookup request and outcome.
    "tf2_request": ("SAMPLE", (
        ("text", "node"), ("int", "id"), ("text", "target"), ("text", "source"),
        ("int", "stamp_ns"), ("int", "tag_time"), ("int", "tag_microstep"),
        ("int", "wait_bound_ns"), ("text", "on_timeout"),
        # The adapter's ordinal on its node and the
        # Buffer's available interval for the frame pair.
        ("int", "adapter"), ("int", "oldest_common_ns"), ("int", "latest_common_ns"),
    )),
    "tf2_outcome": ("SAMPLE", (
        ("text", "node"), ("int", "id"), ("text", "outcome"), ("int", "stamp_ns"),
        ("int", "tag_time"), ("int", "tag_microstep"), ("int", "wait_logical_ns"),
        ("int", "wait_physical_ns"), ("int", "latest_common_ns"),
        ("int", "adapter"), ("int", "oldest_common_ns"),
    )),
    "run_end_publisher": ("LIFECYCLE", (
        ("gid", "gid"), ("int", "tracked"), ("int", "accepted"), ("int", "failed"),
        ("int", "refused_no_context"), ("int", "refused_undeclared_effect"),
        ("int", "refused_edge_not_admitted"), ("int", "nested"), ("int", "unsupported_api"),
        ("int", "first_traced_ordinal"), ("int", "untracked"),
    )),
    "run_end_edge": ("LIFECYCLE", (
        ("int", "edge"), ("int", "expectations"), ("int", "matched"), ("int", "terminal_other"),
        ("int", "samples_taken"), ("int", "samples_retained"), ("int", "orphan_dispatches"),
        ("int", "count_mismatch"),
    )),
    "run_end": ("LIFECYCLE", (
        ("totals", "totals"), ("totals", "attempts"), ("int", "n_publishers"),
        ("int", "n_edges"), ("int", "untracked_publishes"), ("int", "identity_faults"),
        ("int", "reconciled"), ("int", "clock_monotonic_ns"), ("int", "clock_realtime_ns"),
        ("int", "clock_lf_physical_ns"), ("int", "clock_read_gap_ns"),
    )),
}

#: Events of the run_end group: emitted after trace close, seq 0, not counted.
CLOSE_GROUP = frozenset({"run_end_publisher", "run_end_edge", "run_end"})


def event_category(event: str) -> int:
    """Category number of *event* (KeyError for an unknown event)."""
    return CATEGORIES[EVENTS[event][0]]


def field_names(event: str) -> tuple[str, ...]:
    """All field names of *event* including the common three, in wire order."""
    return COMMON_FIELDS + tuple(name for _, name in EVENTS[event][1])
