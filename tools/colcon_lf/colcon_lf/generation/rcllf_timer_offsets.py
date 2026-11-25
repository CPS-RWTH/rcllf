# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Run-time timer offsets for a generated program.

lf-generate copies this file next to every generated launch file, which
imports it by path: a launch file must run without colcon_lf installed, so
this module uses the standard library only.

``RCLLF_TIMER_OFFSETS`` names timers built with
``nodes.<node>.timers.<timer>.offset_runtime: true`` and their offsets::

    RCLLF_TIMER_OFFSETS="controller/control_callback=5 msec"
    RCLLF_TIMER_OFFSETS="/ns/radar/scan_callback=0, controller/control_callback=12ms"

Each entry is ``<node>/<timer callback>=<integer> <unit>`` (``0`` needs no
unit), entries are separated by ``,`` or ``;``. The launch file turns each entry
into the program option ``--<parameter> <value> <unit>``: a main-reactor
parameter that reactor-c sets in ``main()``, before the start tag. A name that
is not a run-time timer of this build is refused, never ignored: a federate
silently skips an option it does not know, so the check has to happen here.
"""

import re

ENV = "RCLLF_TIMER_OFFSETS"

_UNITS_NS = {
    "ns": 1, "nsec": 1, "nsecs": 1,
    "us": 1_000, "usec": 1_000, "usecs": 1_000,
    "ms": 1_000_000, "msec": 1_000_000, "msecs": 1_000_000,
    "s": 1_000_000_000, "sec": 1_000_000_000, "secs": 1_000_000_000,
    "second": 1_000_000_000, "seconds": 1_000_000_000,
}
_VALUE = re.compile(r"^\s*(\d+)\s*([a-z]*)\s*$")
_INT32_MAX = 2**31 - 1  # reactor-c parses the number with strtol into an int


def _norm(name: str) -> str:
    return name.strip().lstrip("/")


def parse_offset_ns(text: str) -> int:
    """``"5 msec"`` -> 5000000. Raises ValueError on anything else."""
    m = _VALUE.match(text)
    if not m:
        raise ValueError(f"offset {text.strip()!r} is not '<integer> <unit>'")
    number, unit = int(m.group(1)), m.group(2)
    if not unit:
        if number != 0:
            raise ValueError(f"offset {text.strip()!r} needs a unit (ns, us, ms or s)")
        return 0
    if unit not in _UNITS_NS:
        raise ValueError(f"offset {text.strip()!r}: unknown unit {unit!r}")
    return number * _UNITS_NS[unit]


def _as_option(ns: int) -> list[str]:
    """The coarsest exact unit, so the number fits reactor-c's int."""
    for unit, scale in (("sec", 10**9), ("msec", 10**6), ("usec", 10**3), ("nsec", 1)):
        if ns % scale == 0 and ns // scale <= _INT32_MAX:
            return [str(ns // scale), unit]
    raise ValueError(f"offset {ns} ns does not fit the program's time option")


def timer_offset_args(table: dict, own=None, text: str | None = None) -> list[str]:
    """Program options for the offsets in *text* (default: ``$RCLLF_TIMER_OFFSETS``).

    *table* maps ``"<node>/<timer>"`` of every run-time timer of the build
    (the whole federation for a federate) to its parameter name. *own* is the
    set of parameters this program takes (``None``: all of them); entries for
    another federate's timers are checked, then left to that federate.
    Raises RuntimeError, naming the known timers, on an unknown name, a
    duplicate or a malformed value.
    """
    import os

    if text is None:
        text = os.environ.get(ENV, "")
    known = {_norm(k): v for k, v in table.items()}
    listing = ", ".join(sorted(known)) or "none (no timer of this build has offset_runtime: true)"
    args: list[str] = []
    seen: set[str] = set()
    for entry in re.split(r"[,;]", text):
        if not entry.strip():
            continue
        name, sep, value = entry.partition("=")
        name = _norm(name)
        if not sep or not name:
            raise RuntimeError(f"{ENV}: entry {entry.strip()!r} is not '<node>/<timer>=<offset>'")
        if name not in known:
            raise RuntimeError(
                f"{ENV}: no run-time timer {name!r} in this build. "
                f"Run-time timers: {listing}")
        if name in seen:
            raise RuntimeError(f"{ENV}: timer {name!r} given twice")
        seen.add(name)
        try:
            option = _as_option(parse_offset_ns(value))
        except ValueError as exc:
            raise RuntimeError(f"{ENV}: {name}: {exc}") from None
        param = known[name]
        if own is None or param in own:
            args += [f"--{param}"] + option
    return args
