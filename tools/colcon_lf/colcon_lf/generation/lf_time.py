# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Parse LF time literals into nanoseconds.

Inverse of ``lf_emitter._ns_to_lf_time``. Accepts the LF spelling used by
``--after-delay`` and ``execution.yaml`` (``1 msec``, ``500 usec``, ``2 sec``)
as well as the terse spelling used by federation YAML (``5ms``, ``100ns``), so
one parser serves both without forcing either config surface to change syntax.

A bad value raises ``ValueError`` rather than falling back to a default: a
silently-ignored delay is exactly the failure mode that made ``--after-delay``
a dead parameter.
"""

from __future__ import annotations

import re

# Nanosecond multipliers, matching lf_emitter's thresholds.
_NS = 1
_US = 1_000
_MS = 1_000_000
_S = 1_000_000_000

# Both spellings of every unit. LF's own grammar accepts the long forms; the
# short forms exist because federation YAML (partition.py) already used them.
_UNITS: dict[str, int] = {
    "nsec": _NS, "nsecs": _NS, "ns": _NS,
    "usec": _US, "usecs": _US, "us": _US,
    "msec": _MS, "msecs": _MS, "ms": _MS,
    "sec": _S, "secs": _S, "s": _S, "second": _S, "seconds": _S,
}

_TIME_RE = re.compile(
    r"^(?P<amount>\d+(?:\.\d+)?)\s*(?P<unit>[a-z]+)?$",
    re.IGNORECASE,
)


def parse_lf_time_ns(value: str | int | None, *, source: str = "") -> int | None:
    """Parse an LF time literal into integer nanoseconds.

    Returns None when *value* is None or an empty/whitespace-only string, so
    callers can distinguish "not configured" from "configured as zero".

    A bare ``0`` is valid and needs no unit; any other bare number is rejected,
    since an unsuffixed delay is ambiguous and silently guessing a unit is how
    a wrong delay reaches generated output unnoticed.
    """
    if value is None:
        return None
    if isinstance(value, bool):  # bool is an int subclass; never a valid time
        raise ValueError(f"{_prefix(source)}invalid time literal {value!r}")
    if isinstance(value, int):
        if value < 0:
            raise ValueError(f"{_prefix(source)}time literal must not be negative: {value!r}")
        return value

    text = value.strip()
    if not text:
        return None

    match = _TIME_RE.match(text)
    if not match:
        raise ValueError(
            f"{_prefix(source)}cannot parse time literal {value!r} "
            f"(expected e.g. '1 msec', '500 usec', '5ms', or '0')"
        )

    amount = float(match.group("amount"))
    unit = match.group("unit")

    if unit is None:
        # Only zero is meaningful without a unit.
        if amount == 0:
            return 0
        raise ValueError(
            f"{_prefix(source)}time literal {value!r} needs a unit "
            f"(e.g. '{match.group('amount')} msec'); only '0' may omit one"
        )

    multiplier = _UNITS.get(unit.lower())
    if multiplier is None:
        raise ValueError(
            f"{_prefix(source)}unknown time unit {unit!r} in {value!r} "
            f"(expected one of: nsec, usec, msec, sec)"
        )

    ns = amount * multiplier
    if ns != int(ns):
        raise ValueError(
            f"{_prefix(source)}time literal {value!r} is not a whole number of "
            f"nanoseconds"
        )
    return int(ns)


def format_lf_time_ns(ns: int) -> str:
    """Format nanoseconds as the shortest exact LF time literal (``20 sec``,
    ``1500 msec``, ``7 nsec``), the form the generated target block uses."""
    if not isinstance(ns, int) or isinstance(ns, bool) or ns < 0:
        raise ValueError(f"cannot format {ns!r} as an LF time")
    for unit, size in (("sec", 1_000_000_000), ("msec", 1_000_000), ("usec", 1_000)):
        if ns % size == 0:
            return f"{ns // size} {unit}"
    return f"{ns} nsec"


def _prefix(source: str) -> str:
    return f"{source}: " if source else ""
