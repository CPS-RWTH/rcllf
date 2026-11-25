# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Preferred CTF backend: the ``bt2`` babeltrace2 Python bindings.

``bt2`` is a **system** package (``python3-bt2`` on Debian/Ubuntu, shipped in the
ROS2 tracing images); it is deliberately not a pip dependency of ``lf_eval``.
When it is missing, :mod:`lf_eval.reader.reader` falls back to the CLI backend.

Compared to the text CLI this gives real typed field values and the true CTF
clock (nanoseconds since the Unix epoch) rather than time-of-day, so timestamps
are directly comparable across traces.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterator, Optional

from lf_eval.reader.events import DiscardStats, TraceEvent

_CONTEXT_KEYS = ("vpid", "vtid", "procname")


def is_available() -> bool:
    """True when the ``bt2`` bindings can be imported."""
    try:
        import bt2  # noqa: F401
    except Exception:
        return False
    return True


def read_events(
    tracing_dir: Path, discards: Optional[DiscardStats] = None
) -> Iterator[TraceEvent]:
    """Stream events out of a CTF directory using ``bt2``.

    :param discards: when given, accumulates the trace-loss cross-check (M7)
        as messages stream past — bt2 surfaces discarded-event/packet counts
        as their own message types (``_DiscardedEventsMessageConst``/
        ``_DiscardedPacketsMessageConst``) interleaved with the real
        ``_EventMessageConst`` stream, so this is a side effect of the same
        iteration loop that already discards non-event messages (it was
        discarding them silently before; now it also *counts* the two kinds
        that indicate real trace loss, distinct from control/stream-end
        messages that carry no loss information at all). Mutated in place
        rather than returned, so ``read_events``'s own return type — a plain
        ``Iterator[TraceEvent]`` the existing ``list(read_trace(...))``
        caller already expects — does not change.
    """
    import bt2

    for msg in bt2.TraceCollectionMessageIterator(str(tracing_dir)):
        if discards is not None:
            if isinstance(msg, bt2._DiscardedEventsMessageConst):
                discards.discarded_events += _message_count(msg)
                continue
            if isinstance(msg, bt2._DiscardedPacketsMessageConst):
                discards.discarded_packets += _message_count(msg)
                continue
        if type(msg) is not bt2._EventMessageConst:
            continue
        event = msg.event
        name = event.name
        if ":" in name:
            name = name.split(":", 1)[1]

        fields = _as_dict(event.payload_field)
        common = _as_dict(getattr(event, "common_context_field", None))
        specific = _as_dict(getattr(event, "specific_context_field", None))
        context = {**common, **specific}

        packet_ctx = _as_dict(
            getattr(getattr(event, "packet", None), "context_field", None)
        )
        cpu_id = packet_ctx.get("cpu_id")
        if cpu_id is None:
            cpu_id = context.get("cpu_id")

        yield TraceEvent(
            name=name,
            timestamp_ns=msg.default_clock_snapshot.ns_from_origin,
            fields=fields,
            cpu_id=_as_int(cpu_id),
            pid=_as_int(context.get("vpid")),
            tid=_as_int(context.get("vtid")),
            procname=_as_str(context.get("procname")),
        )


def _message_count(msg) -> int:
    """Best-effort discard count from a bt2 discarded-events/packets message.

    ``msg.count`` is only present when the trace's clock lets bt2 compute an
    exact figure; otherwise it is ``None`` (loss is known to have happened,
    magnitude isn't). Treated as ``1`` in that case — a lower bound, not a
    fabricated exact count — so :attr:`DiscardStats.any_discarded` still
    trips even when the precise number is unavailable.
    """
    count = getattr(msg, "count", None)
    return int(count) if isinstance(count, int) else 1


def _as_dict(struct) -> dict[str, object]:
    """Convert a bt2 structure field into a plain dict of Python scalars."""
    if struct is None:
        return {}
    out: dict[str, object] = {}
    try:
        keys = list(struct)
    except TypeError:  # pragma: no cover - not a structure
        return {}
    for key in keys:
        try:
            out[key] = _to_python(struct[key])
        except Exception:  # pragma: no cover - unsupported field class
            continue
    return out


def _to_python(value):
    """Unwrap a bt2 field into an int/float/str/list."""
    import bt2

    if isinstance(value, bt2._StringFieldConst):
        return str(value)
    if isinstance(value, (bt2._IntegerFieldConst, bt2._EnumerationFieldConst)):
        return int(value)
    if isinstance(value, bt2._RealFieldConst):
        return float(value)
    if isinstance(value, bt2._ArrayFieldConst):
        return [_to_python(item) for item in value]
    if isinstance(value, bt2._StructureFieldConst):
        return _as_dict(value)
    return value


def _as_int(value) -> Optional[int]:
    if isinstance(value, bool) or value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_str(value) -> Optional[str]:
    return str(value) if value is not None else None
