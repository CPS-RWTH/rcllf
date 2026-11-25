# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Fallback CTF backend: parse ``babeltrace2``/``babeltrace`` CLI text output.

Preferred over nothing, but the ``bt2`` Python bindings
(:mod:`lf_eval.reader.bt2_backend`) are used when importable — they give real
typed fields and a real clock, instead of the text format's time-of-day.

Text format (both v1 and v2 ``--clock-cycles``-free default output)::

    [HH:MM:SS.nnnnnnnnn] (+D.DDDDDDDDD) host ros2:EVENT: { cpu_id = 5 }, { a = 1, b = "x" }

The line is split structurally rather than with one big regex, because payloads
contain nested braces and bracketed arrays (``gid = [ [0] = 1, ... ]``) that a
single ``\\{(.+)\\}`` group mis-captures. Timestamps are **time of day**, so a
trace crossing midnight would wrap; :func:`_parse_timestamp_ns` therefore
monotonically unwraps by adding whole days.
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path
from typing import Iterator, Optional

from lf_eval.reader.events import DiscardStats, TraceEvent, coerce_field

_DAY_NS = 86_400_000_000_000

# [HH:MM:SS.nnnnnnnnn] (+delta) hostname provider:EVENT: <rest>
_HEAD_RE = re.compile(
    r"^\[(?P<ts>\d{2}:\d{2}:\d{2}\.\d{9})\]"
    r"\s+\((?:[^)]*)\)"
    r"\s+(?P<host>\S+)"
    r"\s+(?P<provider>[A-Za-z_][\w+-]*):(?P<event>\w+):"
    r"\s*(?P<rest>.*)$"
)

# key = value  where value is a quoted string, hex, decimal, or bare token.
_FIELD_RE = re.compile(
    r'(?P<key>\w+)\s*=\s*'
    r'(?P<val>"(?:[^"\\]|\\.)*"|0[xX][0-9A-Fa-f]+|[+-]?\d+(?:\.\d+)?|[A-Za-z_]\w*)'
)


def cli_binary() -> Optional[str]:
    """Return the babeltrace CLI to use, preferring v2. ``None`` if absent."""
    for name in ("babeltrace2", "babeltrace"):
        path = shutil.which(name)
        if path:
            return path
    return None


def is_available() -> bool:
    """True when a babeltrace CLI is on ``PATH``."""
    return cli_binary() is not None


def _parse_timestamp_ns(text: str, prev_ns: Optional[int]) -> int:
    """Parse ``HH:MM:SS.nnnnnnnnn`` into ns, unwrapping midnight rollover.

    ``prev_ns`` is the previously returned (already unwrapped) value; because
    babeltrace emits events in chronological order, a value that would go
    backwards by more than half a day means the clock wrapped past midnight.
    """
    hh, mm, rest = text.split(":")
    ss, ns = rest.split(".")
    value = (
        int(hh) * 3_600_000_000_000
        + int(mm) * 60_000_000_000
        + int(ss) * 1_000_000_000
        + int(ns)
    )
    if prev_ns is not None:
        days, rem = divmod(prev_ns, _DAY_NS)
        value += days * _DAY_NS
        if rem - (value - days * _DAY_NS) > _DAY_NS // 2:
            value += _DAY_NS
    return value


def _split_top_level(rest: str) -> list[str]:
    """Split ``{ a }, { b }`` into the top-level brace groups' inner text."""
    groups: list[str] = []
    depth = 0
    start = 0
    in_string = False
    escaped = False
    for i, ch in enumerate(rest):
        if in_string:
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == '"':
                in_string = False
            continue
        if ch == '"':
            in_string = True
        elif ch == "{":
            if depth == 0:
                start = i + 1
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                groups.append(rest[start:i])
    return groups


def parse_line(line: str, prev_ns: Optional[int] = None) -> Optional[TraceEvent]:
    """Parse one babeltrace text line into a :class:`TraceEvent`.

    Returns ``None`` for lines that are not event records (warnings, blanks).
    """
    match = _HEAD_RE.match(line)
    if not match:
        return None

    groups = _split_top_level(match.group("rest"))
    context: dict[str, object] = {}
    payload: dict[str, object] = {}
    if len(groups) >= 2:
        # Last group is the payload; everything before it is stream/event context.
        for grp in groups[:-1]:
            context.update(_parse_fields(grp))
        payload = _parse_fields(groups[-1])
    elif len(groups) == 1:
        # Only one group: babeltrace omits the payload block for argument-less
        # tracepoints, so a lone group is context (it carries cpu_id).
        only = _parse_fields(groups[0])
        if set(only) <= {"cpu_id", "vpid", "vtid", "procname", "ip"}:
            context = only
        else:
            payload = only

    return TraceEvent(
        name=match.group("event"),
        timestamp_ns=_parse_timestamp_ns(match.group("ts"), prev_ns),
        fields=payload,
        cpu_id=_as_int(context.get("cpu_id")),
        pid=_as_int(context.get("vpid")),
        tid=_as_int(context.get("vtid")),
        procname=context.get("procname") if isinstance(context.get("procname"), str) else None,
    )


def _parse_fields(text: str) -> dict[str, object]:
    """Extract ``key = value`` pairs, skipping array element indices."""
    out: dict[str, object] = {}
    for m in _FIELD_RE.finditer(text):
        key = m.group("key")
        if key in out:
            continue  # first occurrence wins (array elements reuse names)
        out[key] = coerce_field(key, m.group("val"))
    return out


def _as_int(value: object) -> Optional[int]:
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def read_events(
    tracing_dir: Path, discards: Optional[DiscardStats] = None
) -> Iterator[TraceEvent]:
    """Stream events out of a CTF directory via the babeltrace CLI.

    :param discards: when given, marked ``known=False`` (M7) — the text CLI's
        output gives no reliable per-stream discarded-event/packet count (no
        equivalent of the bt2 backend's ``_DiscardedEventsMessageConst``), so
        this backend is honest about not knowing rather than fabricating a
        count. Left untouched (``known`` stays whatever the caller set it to)
        if a caller pre-populates it, so a caller can't mistake "the CLI
        backend didn't check" for "the CLI backend confirmed zero discards."
    """
    binary = cli_binary()
    if binary is None:  # pragma: no cover - guarded by the caller
        raise RuntimeError("no babeltrace CLI on PATH")
    if discards is not None:
        discards.known = False

    proc = subprocess.Popen(
        [binary, str(tracing_dir)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    prev_ns: Optional[int] = None
    try:
        for line in proc.stdout:
            event = parse_line(line.rstrip("\n"), prev_ns)
            if event is not None:
                prev_ns = event.timestamp_ns
                yield event
    finally:
        proc.stdout.close()
        proc.wait()
