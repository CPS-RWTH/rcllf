# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Runtime-side ``/proc`` CPU sampler, exposed as ``lf-eval sample-cpu``.

This must run
*while the system under test is running*, so it stays a separate command rather
than part of ``lf-eval analyze``; it writes ``cpu_proc.json`` into the run
directory, which the :mod:`lf_eval.metrics.cpu` metric later reads.

Changes from the original:

* the hardcoded ``n_cores_machine = 12`` becomes :func:`machine_cores`
  (``os.cpu_count()``, overridable via ``--cores``);
* pure functions are importable for testing (unchanged semantics).

``/proc/<pid>/stat`` field 14 = ``utime``, 15 = ``stime`` (1-indexed). Field 2
(``comm``) is parenthesised and may itself contain spaces and parens, so the
numeric tail is found by splitting on the **last** ``)``.
"""

from __future__ import annotations

import json
import os
import time
from typing import Iterable, Optional


def machine_cores(override: Optional[int] = None) -> int:
    """Number of CPUs to normalise "percent of machine" against."""
    if override and override > 0:
        return int(override)
    return os.cpu_count() or 1


def read_proc_stat(pid: int, proc_root: str = "/proc") -> "tuple[int, int] | None":
    """Return ``(utime, stime)`` in clock ticks for ``pid``, or ``None`` if gone."""
    path = os.path.join(proc_root, str(pid), "stat")
    try:
        with open(path, "r") as handle:
            data = handle.read()
    except (FileNotFoundError, ProcessLookupError, PermissionError, NotADirectoryError):
        return None
    rparen = data.rfind(")")
    if rparen == -1:
        return None
    tail = data[rparen + 2:].split()
    # After comm, field 3 (state) is tail[0]; utime is field 14 -> tail[11],
    # stime field 15 -> tail[12].
    try:
        return (int(tail[11]), int(tail[12]))
    except (IndexError, ValueError):
        return None


def cores_used(
    utime0: int,
    stime0: int,
    utime1: int,
    stime1: int,
    wall_s: float,
    clk_tck: int,
) -> float:
    """CPU-seconds consumed over the window / wall-seconds = cores used."""
    if wall_s <= 0 or clk_tck <= 0:
        return 0.0
    cpu_ticks = (utime1 - utime0) + (stime1 - stime0)
    return (cpu_ticks / clk_tck) / wall_s


def discover_pids(
    patterns: Iterable[str], proc_root: str = "/proc"
) -> "dict[int, str]":
    """Map ``pid -> cmdline`` for processes whose cmdline contains any pattern."""
    patterns = list(patterns)
    found: dict[int, str] = {}
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        try:
            with open(os.path.join(proc_root, entry, "cmdline"), "rb") as handle:
                cmd = (
                    handle.read()
                    .replace(b"\0", b" ")
                    .decode(errors="replace")
                    .strip()
                )
        except (FileNotFoundError, ProcessLookupError, PermissionError, NotADirectoryError):
            continue
        if cmd and any(p in cmd for p in patterns):
            found[pid] = cmd
    return found


def aggregate_cores(per_pid: dict) -> float:
    """Sum of per-process cores used."""
    return round(sum(v["cores"] for v in per_pid.values()), 4)


def format_pid_list(pids: Iterable[int]) -> str:
    """Bare newline-separated sorted pids, for ``--list-pids`` (taskset input).

    No JSON, no summary line — safe to parse as integers directly.
    """
    return "\n".join(str(p) for p in sorted(pids))


def build_report(
    per_pid: dict,
    wall_s: float,
    clk_tck: int,
    warmup_s: float,
    window_s: float,
    cores: Optional[int] = None,
) -> dict:
    """Assemble the ``cpu_proc.json`` payload."""
    n_cores = machine_cores(cores)
    aggregate = aggregate_cores(per_pid)
    return {
        "clk_tck": clk_tck,
        "wall_s": round(wall_s, 3),
        "warmup_s": warmup_s,
        "window_s": window_s,
        "per_pid": per_pid,
        "aggregate_cores": aggregate,
        "n_cores_machine": n_cores,
        "aggregate_pct_one_core": round(aggregate * 100, 1),
        "aggregate_pct_machine": round(aggregate * 100 / n_cores, 1),
    }


def sample(
    patterns: Iterable[str],
    warmup_s: float = 10.0,
    window_s: float = 50.0,
    proc_root: str = "/proc",
    cores: Optional[int] = None,
    sleep=time.sleep,
    clock=time.monotonic,
) -> dict:
    """Sample steady-state CPU usage of matching processes.

    ``sleep`` and ``clock`` are injectable so tests need not actually wait.
    """
    clk_tck = os.sysconf("SC_CLK_TCK")
    sleep(warmup_s)
    pids = discover_pids(patterns, proc_root=proc_root)
    if not pids:
        raise LookupError(f"no pids matched {list(patterns)}")

    start: dict[int, tuple[int, int]] = {}
    t_start = clock()
    for pid in pids:
        stat = read_proc_stat(pid, proc_root=proc_root)
        if stat:
            start[pid] = stat
    sleep(window_s)
    wall_s = clock() - t_start

    per_pid: dict[int, dict] = {}
    for pid, cmd in pids.items():
        first = start.get(pid)
        last = read_proc_stat(pid, proc_root=proc_root)
        if not first or not last:
            continue
        per_pid[pid] = {
            "cmd": cmd,
            "utime0": first[0],
            "stime0": first[1],
            "utime1": last[0],
            "stime1": last[1],
            "cores": round(
                cores_used(first[0], first[1], last[0], last[1], wall_s, clk_tck), 4
            ),
        }

    return build_report(
        per_pid,
        wall_s=wall_s,
        clk_tck=clk_tck,
        warmup_s=warmup_s,
        window_s=window_s,
        cores=cores,
    )


def write_report(report: dict, out_path: str) -> None:
    """Write a sampler report as JSON."""
    with open(out_path, "w") as handle:
        json.dump(report, handle, indent=2)


# --- per-CPU utilisation -----------------------------------------
#
# ``/proc/stat`` ``cpuN`` lines: user nice system idle iowait irq softirq steal
# (guest time is already part of user). Busy is everything but idle + iowait.


def read_cpu_times(proc_root: str = "/proc") -> "dict[int, tuple[int, int]]":
    """``cpu index -> (busy ticks, total ticks)`` from ``/proc/stat``; empty if unreadable."""
    out: dict[int, tuple[int, int]] = {}
    try:
        with open(os.path.join(proc_root, "stat"), "r") as handle:
            lines = handle.read().splitlines()
    except OSError:
        return out
    for line in lines:
        name, _, rest = line.partition(" ")
        if not name.startswith("cpu") or not name[3:].isdigit():
            continue
        try:
            ticks = [int(v) for v in rest.split()[:8]]
        except ValueError:
            continue
        if len(ticks) < 4:
            continue
        idle = ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
        total = sum(ticks)
        out[int(name[3:])] = (total - idle, total)
    return out


def per_cpu_busy(
    first: "dict[int, tuple[int, int]]", last: "dict[int, tuple[int, int]]"
) -> "dict[int, float]":
    """Busy fraction (0..1) of every CPU between two :func:`read_cpu_times` samples."""
    out: dict[int, float] = {}
    for cpu, (busy0, total0) in first.items():
        if cpu not in last:
            continue
        busy1, total1 = last[cpu]
        dt = total1 - total0
        out[cpu] = round((busy1 - busy0) / dt, 4) if dt > 0 else 0.0
    return out
