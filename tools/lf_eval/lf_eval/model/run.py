# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The :class:`Run` object — one experiment directory, loaded.

An experiment run directory (produced by ``docker/trace.sh``, see
``docs/features/tracing.md``) looks like::

    <UTC-timestamp>_<mode>_<scenario>_r<repeat>/
      metadata.yaml     # timestamp, mode, duration, system, label, hostname, ...
      tracing/          # LTTng CTF trace
      bags/recording/   # ros2 bag (mcap) — not read by lf_eval
      logs/
      cpu_proc.json     # optional, written by `lf-eval sample-cpu`

``mode`` is an **open string**, not a closed enum: ``load_metadata`` passes any
value through unchanged. :data:`MODES` is a documented tuple of well-known
values, kept for default colour/label/ordering purposes only (see
``lf_eval.plots.style``) — ``lfc``/``lf``/``stock`` (pre-Phase-6 data) plus the
Phase 6 per-executor vocabulary ``stock-single``/``stock-multi``/
``stock-events``/``rslcpp``. A run with any other mode string works
identically; it just falls back to a deterministically-assigned colour and its
own raw string as a label.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

from lf_eval.model.system import SystemModel
from lf_eval.model.timeline import REQUIRED_EVENTS, Timeline, build_timeline

METADATA_FILENAME = "metadata.yaml"
CPU_PROC_FILENAME = "cpu_proc.json"

# Well-known experiment modes, in display order. Not a closed enum — any string
# is accepted by `load_metadata`/`Run.mode`; this tuple only seeds the default
# colour/label/ordering tables in `lf_eval.plots.style`.
MODES = ("stock-single", "stock-multi", "stock-events", "lfc", "rslcpp", "lf", "stock")


@dataclass
class RunMetadata:
    """Contents of ``metadata.yaml`` (all fields optional)."""

    mode: str = "unknown"
    duration_s: float = 0.0
    label: str = ""
    system: str = ""
    timestamp: str = ""
    hostname: str = ""
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class Run:
    """One experiment run: metadata plus its callback timeline.

    ``drop_counts`` is populated by :func:`load_run` (and may be set directly by
    tests via :func:`make_run`); it is typed loosely to keep
    :mod:`lf_eval.model` free of a dependency on :mod:`lf_eval.metrics`.

    ``discard_stats`` is the bt2/CTF-side trace-loss **cross-check** (M7) —
    independent of, and never a substitute for, ``metadata.yaml``'s
    authoritative ``events_discarded`` field (read directly by the report
    layer, see :mod:`lf_eval.report`). ``None`` until :func:`load_run`
    populates it; :func:`make_run` leaves it ``None`` unless a test passes one
    explicitly.

    ``runtime_topology`` is the pub/sub init topology that :func:`load_run`
    collects from the same single decode, so the topology metric never reads
    the trace a second time. ``None`` for a :func:`make_run` run.
    """

    path: Path
    name: str
    metadata: RunMetadata
    timeline: Timeline
    cpu_proc: Optional[dict] = None
    drop_counts: Any = None
    discard_stats: Any = None
    window: Any = None
    runtime_topology: Any = None
    #: the timeline of the whole run when ``timeline`` is cut to a window. Message
    #: identity is paired on it, so a message published before the window still
    #: names its publisher (``None``: ``timeline`` is the whole run).
    full_timeline: Optional[Timeline] = None

    @property
    def mode(self) -> str:
        return self.metadata.mode

    @property
    def window_ns(self) -> Optional[tuple[int, int]]:
        """``[lo, hi)`` of the analysis window in trace ns, ``None`` without a window."""
        if self.window is None or self.full_timeline is None:
            return None
        return self.timeline.first_ns, self.timeline.last_ns

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"Run({self.name!r}, mode={self.mode!r}, nodes={len(self.timeline.invocations)})"


def load_metadata(run_dir: Path) -> RunMetadata:
    """Read ``metadata.yaml``; missing file yields defaults."""
    path = Path(run_dir) / METADATA_FILENAME
    if not path.is_file():
        return RunMetadata()
    import yaml

    with path.open() as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return RunMetadata()
    known = {"mode", "duration", "label", "system", "timestamp", "hostname"}
    return RunMetadata(
        mode=str(data.get("mode", "unknown")),
        duration_s=float(data.get("duration") or 0.0),
        label=str(data.get("label", "")),
        system=str(data.get("system", "")),
        timestamp=str(data.get("timestamp", "")),
        hostname=str(data.get("hostname", "")),
        extra={k: v for k, v in data.items() if k not in known},
    )


def load_cpu_proc(run_dir: Path) -> Optional[dict]:
    """Read a ``cpu_proc.json`` written by ``lf-eval sample-cpu``, if present."""
    import json

    path = Path(run_dir) / CPU_PROC_FILENAME
    if not path.is_file():
        return None
    try:
        with path.open() as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


#: Events of steady-state execution. A measurement window filters only these;
#: registration events (node, timer, subscription init) are kept whatever their
#: time, so windowed callbacks still resolve to their nodes.
RUNTIME_EVENTS = frozenset({
    "callback_start", "callback_end", "rclcpp_publish", "rcl_publish", "rclcpp_intra_publish",
    "rmw_publish",
    "rmw_take", "rcl_take", "rclcpp_take",
})


#: Modes whose clock is simulated: their trace timestamps are wall
#: time but cover more simulated time, so a wall-clock window has no meaning.
#: Their physical numbers are descriptive only and reproducibility uses the
#: whole run, so they are analysed without a window.
SIMULATED_MODES = frozenset({"rslcpp"})


def window_for_mode(window: Optional["Window"], mode: str) -> Optional["Window"]:
    return None if mode in SIMULATED_MODES else window


class WindowError(ValueError):
    """The run is too short for the requested measurement window."""


@dataclass(frozen=True)
class Window:
    """Measurement window: skip ``warmup_s`` after the run's first
    ``callback_start``, then keep ``steady_s`` seconds (the rest of the run when
    ``None``). Equal windows make variants with different run lengths comparable
    (stock stops itself at ``--duration-s``, an lfc launch runs until the driver
    stops it)."""

    warmup_s: float
    steady_s: Optional[float] = None

    def to_dict(self) -> dict:
        return {"warmup_s": self.warmup_s, "steady_s": self.steady_s,
                "anchor": "first callback_start"}


def window_bounds(events: list, window: "Window") -> tuple[int, int]:
    """``[lo, hi)`` in trace ns. Refuses a window the run does not cover."""
    starts = [e.timestamp_ns for e in events if e.name == "callback_start"]
    if not starts:
        raise WindowError("no callback_start in the run, nothing to anchor the window on")
    last = max(e.timestamp_ns for e in events if e.name in RUNTIME_EVENTS)
    lo = min(starts) + int(window.warmup_s * 1e9)
    if window.steady_s is None:
        return lo, last + 1
    hi = lo + int(window.steady_s * 1e9)
    if hi > last:
        raise WindowError(
            f"window [{window.warmup_s}s, +{window.steady_s}s] ends after the last runtime "
            f"event ({(last - min(starts)) / 1e9:.3f}s into the run)")
    return lo, hi


def apply_window(events: list, window: "Window") -> list:
    """Runtime events inside the window, plus every non-runtime event."""
    lo, hi = window_bounds(events, window)
    return [e for e in events if e.name not in RUNTIME_EVENTS or lo <= e.timestamp_ns < hi]


def windowed_timeline(events: list, window: "Window", **kwargs):
    """Timeline of the window whose span is the window itself, so rates and
    CPU cores divide by the window length, not by startup plus window."""
    lo, hi = window_bounds(events, window)
    timeline = build_timeline(apply_window(events, window), **kwargs)
    timeline.first_ns, timeline.last_ns = lo, hi
    return timeline


def load_run(
    run_dir: Path,
    system: Optional[SystemModel] = None,
    backend: Optional[str] = None,
    window: Optional[Window] = None,
) -> Run:
    """Load one run directory: metadata, CTF trace, and callback timeline.

    :param run_dir: the experiment run directory (or a bare CTF directory).
    :param system: when given, its node names filter out ROS infrastructure
        callbacks (``rosbag2_recorder``, parameter services, ...).
    :param backend: forced reader backend, see :func:`lf_eval.reader.read_trace`.
    """
    from lf_eval.metrics.drops import DROP_EVENTS, collect_drop_counts
    from lf_eval.metrics.topology import TOPOLOGY_EVENTS, collect_runtime_topology
    from lf_eval.reader import DiscardStats, read_trace

    run_dir = Path(run_dir)
    node_filter = _node_filter(system)

    # One pass over the (large) trace feeds both consumers: the event stream is
    # buffered into a list of only the events we actually need, so the CTF
    # directory is decoded exactly once. `discards` accumulates as a side
    # effect of that same pass (M7's bt2-side cross-check) — it is only
    # meaningful once the list() below has fully consumed the generator.
    discards = DiscardStats()
    wanted = set(REQUIRED_EVENTS) | set(DROP_EVENTS) | set(TOPOLOGY_EVENTS)
    events = list(
        read_trace(run_dir, backend=backend, event_names=wanted, discards=discards)
    )

    metadata = load_metadata(run_dir)
    window = window_for_mode(window, metadata.mode)
    resolver = system.canonical_of if system is not None else None
    full = build_timeline(events, node_filter=node_filter, resolver=resolver)
    # Drops are an accounting over the whole run: a window edge would separate
    # a publish inside it from its delivery just after it and invent losses.
    drop_counts = collect_drop_counts(events, full, system=system)
    timeline = full if window is None else windowed_timeline(
        events, window, node_filter=node_filter, resolver=resolver)

    return Run(
        path=run_dir,
        name=run_dir.name,
        metadata=metadata,
        timeline=timeline,
        cpu_proc=load_cpu_proc(run_dir),
        drop_counts=drop_counts,
        discard_stats=discards,
        window=window,
        # Topology from the same decode,
        # over the whole run like the drop accounting.
        runtime_topology=collect_runtime_topology(events),
        full_timeline=None if window is None else full,
    )


def make_run(
    name: str,
    timeline: Timeline,
    mode: str = "unknown",
    path: Optional[Path] = None,
    duration_s: float = 0.0,
    cpu_proc: Optional[dict] = None,
    drop_counts: Any = None,
) -> Run:
    """Construct a :class:`Run` directly from a timeline.

    The seam that makes every metric testable without CTF data: tests build a
    timeline from synthetic events and wrap it here.
    """
    return Run(
        path=path or Path(name),
        name=name,
        metadata=RunMetadata(mode=mode, duration_s=duration_s, label=name),
        timeline=timeline,
        cpu_proc=cpu_proc,
        drop_counts=drop_counts,
    )


def _node_filter(system: Optional[SystemModel]) -> Optional[set[str]]:
    """Node-name allow-list derived from a system model."""
    if system is None:
        return None
    names: set[str] = set()
    for node in system.nodes.values():
        names.update({node.key, node.name, node.class_name})
    return names or None
