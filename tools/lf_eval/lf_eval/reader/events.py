# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Typed trace event records.

One flat dataclass covers every ``ros2:*`` tracepoint: the event name plus a
``fields`` mapping. A flat shape (rather than one class per tracepoint) is
deliberate — the set of ``ros2:*`` tracepoints changes between ROS distros, and
metrics only ever look up a handful of named fields.

Field values are normalised by the reader:

* pointer-like fields (``callback``, ``*_handle``, ``message``, ``subscription``,
  ``timer_handle``, ...) become Python ``int``;
* integer fields (``period``, ``queue_depth``, ``taken``, ``timestamp``,
  ``source_timestamp``) become ``int``;
* string fields (``symbol``, ``node_name``, ``topic_name``, ``namespace``)
  become ``str`` with quotes stripped.

``timestamp_ns`` is the CTF clock value. For the babeltrace-v1 CLI backend it is
nanoseconds since midnight (the v1 text format prints wall-clock time of day);
metrics only ever use *differences*, so the epoch is irrelevant as long as it is
consistent within one trace. See :mod:`lf_eval.reader.babeltrace_cli`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional


@dataclass
class DiscardStats:
    """Trace-loss signal accumulated while reading a CTF trace (M7).

    This is the bt2/CTF-side **cross-check** count — independent of the
    orchestrator's authoritative ``metadata.yaml``'s ``events_discarded``
    field (summed from ``lttng list <session>``'s per-channel report, read
    after ``stop`` and before ``destroy``). The two are deliberately
    separate numbers in separate files, never two writers racing to fill one
    field — ``metadata.yaml``'s value is authoritative for a run's trace-loss
    status; this one is a corroborating (or contradicting) signal surfaced in
    ``metrics.json``'s data-quality block.

    :param discarded_events: sum of per-stream discarded **event** counts
        (``_DiscardedEventsMessage`` on the bt2 backend). ``0`` when the
        trace reported no discards.
    :param discarded_packets: sum of per-stream discarded **packet** counts
        (``_DiscardedPacketsMessage``). Packet loss implies an unknown number
        of lost events within those packets.
    :param known: ``False`` when the backend cannot determine discard counts
        at all (the babeltrace-CLI fallback, absent clear stderr signal) —
        callers must not treat ``0`` and "unknown" as the same thing.
    """

    discarded_events: int = 0
    discarded_packets: int = 0
    known: bool = True

    @property
    def total(self) -> Optional[int]:
        """Combined discard count, or ``None`` when not :attr:`known`."""
        if not self.known:
            return None
        return self.discarded_events + self.discarded_packets

    @property
    def any_discarded(self) -> bool:
        """``True`` when the trace is known to have lost events/packets."""
        return self.known and (self.discarded_events > 0 or self.discarded_packets > 0)

    def to_dict(self) -> dict[str, Any]:
        return {
            "known": self.known,
            "discarded_events": self.discarded_events if self.known else None,
            "discarded_packets": self.discarded_packets if self.known else None,
        }

# Fields whose values are pointers/handles printed as 0x-hex by babeltrace.
POINTER_FIELDS = frozenset(
    {
        "callback",
        "message",
        "subscription",
        "subscription_handle",
        "rmw_subscription_handle",
        "publisher_handle",
        "rmw_publisher_handle",
        "node_handle",
        "rmw_handle",
        "timer_handle",
        "handle",
        "service_handle",
        "rmw_service_handle",
        "client_handle",
        "rmw_client_handle",
        "state_machine",
        "callback_group",
    }
)

# Fields that are plain (decimal) integers.
INT_FIELDS = frozenset(
    {
        "period",
        "queue_depth",
        "taken",
        "timestamp",
        "source_timestamp",
        "is_intra_process",
        "version",
        "state",
        "function_symbol",
        "cpu_id",
    }
)

# Fields that are strings.
STRING_FIELDS = frozenset(
    {
        "symbol",
        "node_name",
        "namespace",
        "topic_name",
        "service_name",
        "label",
        "state_label",
        "start_label",
        "goal_label",
    }
)


@dataclass(frozen=True, slots=True)
class TraceEvent:
    """One ``ros2:*`` userspace tracepoint occurrence.

    :param name: tracepoint name **without** the ``ros2:`` provider prefix
        (e.g. ``callback_start``).
    :param timestamp_ns: CTF clock value in nanoseconds.
    :param fields: normalised event payload.
    :param cpu_id: the ``cpu_id`` stream context, when present.
    :param pid: ``vpid`` context, when the session enabled it (often absent —
        ``docker/trace.sh`` does not add ``--context`` options).
    :param tid: ``vtid`` context, when present.
    :param procname: ``procname`` context, when present.
    """

    name: str
    timestamp_ns: int
    fields: Mapping[str, Any] = field(default_factory=dict)
    cpu_id: Optional[int] = None
    pid: Optional[int] = None
    tid: Optional[int] = None
    procname: Optional[str] = None

    def get(self, key: str, default: Any = None) -> Any:
        """Look up a payload field."""
        return self.fields.get(key, default)

    def __getitem__(self, key: str) -> Any:
        return self.fields[key]


def coerce_field(key: str, raw: Any) -> Any:
    """Normalise one raw field value according to its well-known name.

    Unknown field names are passed through with a best-effort numeric parse so
    that new tracepoints still yield usable values.
    """
    if isinstance(raw, (int, float)) and not isinstance(raw, bool):
        return raw
    if not isinstance(raw, str):
        return raw
    text = raw.strip()
    if key in STRING_FIELDS:
        return text[1:-1] if len(text) >= 2 and text[0] == text[-1] == '"' else text
    if text.startswith('"') and text.endswith('"') and len(text) >= 2:
        return text[1:-1]
    if key in POINTER_FIELDS or text.lower().startswith("0x"):
        try:
            return int(text, 16)
        except ValueError:
            return text
    if key in INT_FIELDS or _looks_integral(text):
        try:
            return int(text, 10)
        except ValueError:
            return text
    return text


def _looks_integral(text: str) -> bool:
    body = text[1:] if text[:1] in "+-" else text
    return bool(body) and body.isdigit()
