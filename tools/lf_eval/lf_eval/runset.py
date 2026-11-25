# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Run-set inclusion check of the benchmark protocol.

Each run is included or excluded with every reason that applies, before any
analysis. Excluded runs stay in the table: the protocol requires exclusions to
remain visible, never silently dropped.

Reasons:

- ``no_metadata``: no ``metadata.yaml`` (the benchmark orchestrator writes it only after the
  variant ran: a failed launch, an abnormal exit or leftover processes from an
  earlier run end the run before it).
- ``trace_loss:<channel>``: the run manifest reports discarded events or lost
  packets on a channel the run used, or could not tell (``unknown``, missing or
  not a count).
- ``trace_loss:no_channels``: ``trace_manifest.json`` exists but lists no
  channel, so it vouches for nothing. ``trace_loss:manifest_malformed``: it is
  not a JSON object.
- ``trace_loss:all``: metadata ``events_discarded`` is above 0 or not a count.
  A run without ``trace_manifest.json`` (an older orchestrator) is judged by this
  field alone, and a missing field there is not a loss.
- ``shorter_than_window``: the run's wall duration is below warm-up plus
  window (a necessary condition, ``lf-eval analyze`` enforces the exact one).
- ``semantic_incomplete``: an lfc run whose admitted-edge claims are not all
  complete (stock and RSLCPP have no provider, so their claims are
  ``unsupported`` by design and do not exclude them).
- ``metadata_malformed``: ``metadata.yaml`` exists but is not a YAML mapping.

The default check fails closed: where the version crashed
(unparseable YAML or timestamp, a malformed manifest or count) or let a run
through (non-mapping metadata read as ``no_metadata``, a manifest without
channels), it now excludes the run with the reason above; an unparseable
``started_at``/``ended_at`` reads as ``shorter_than_window``, and metadata
``events_discarded`` counts even when a manifest exists. On well-formed,
loss-free runs its output is unchanged.

Metadata integrity (added after the protocol v1 freeze; opt-in).
Each check runs only when its expectation is given, and a missing or
unparseable field fails closed:

- ``metadata_missing:<field>`` / ``metadata_invalid:<field>`` /
  ``metadata_mismatch:<field>`` for ``git_sha`` (40 lowercase hex),
  ``git_dirty`` (a YAML boolean, must be false), ``duration`` (finite number,
  seconds) and ``scenario`` (string).
- ``identity_missing:<file>``: uniform identity was requested and the run lacks
  one of :data:`WORKLOAD_FILES`.
- ``identity_not_uniform:git_sha`` / ``identity_not_uniform:workload``: the
  selected runs do not all share one source SHA or one workload digest. Exact
  equality over the whole set, no majority vote: every run carrying a value is
  excluded, so a mixed set is visible in full rather than repaired.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional, Sequence

from lf_eval.model.run import METADATA_FILENAME

#: Per-run workload inputs the benchmark orchestrator writes into every run directory; their
#: digest is the workload identity of a run.
WORKLOAD_FILES = ("scenario_params.yaml", "scenario_overlay.env", "system.json")

_SHA_RE = re.compile(r"[0-9a-f]{40}")


@dataclass(frozen=True)
class Expectations:
    """Frozen-protocol metadata every run must carry (all optional)."""

    git_sha: Optional[str] = None
    duration_s: Optional[float] = None
    scenario: Optional[str] = None
    require_clean_tree: bool = False
    require_uniform_identity: bool = False

    def requested(self) -> bool:
        return any(v not in (None, False) for v in asdict(self).values())

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _load_metadata(run_dir: Path) -> tuple[dict, Optional[str]]:
    """``(metadata, reason)``: reason is ``no_metadata`` or ``metadata_malformed``."""
    import yaml

    path = Path(run_dir) / METADATA_FILENAME
    if not path.is_file():
        return {}, "no_metadata"
    try:
        data = yaml.safe_load(path.read_text())
    except (yaml.YAMLError, UnicodeDecodeError):
        return {}, "metadata_malformed"
    if data is None:
        return {}, "no_metadata"
    if not isinstance(data, dict):
        return {}, "metadata_malformed"
    return data, None


def valid_sha(value: Any) -> bool:
    return isinstance(value, str) and _SHA_RE.fullmatch(value) is not None


def _metadata_reasons(md: dict, expect: Expectations) -> list[str]:
    reasons: list[str] = []

    def check(field: str, valid, matches) -> None:
        if md.get(field) is None:
            reasons.append(f"metadata_missing:{field}")
        elif not valid(md[field]):
            reasons.append(f"metadata_invalid:{field}")
        elif not matches(md[field]):
            reasons.append(f"metadata_mismatch:{field}")

    if expect.git_sha is not None or expect.require_uniform_identity:
        check("git_sha", valid_sha, lambda v: expect.git_sha is None or v == expect.git_sha)
    if expect.require_clean_tree:
        check("git_dirty", lambda v: isinstance(v, bool), lambda v: v is False)
    if expect.duration_s is not None:
        check("duration",
              lambda v: isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v),
              lambda v: abs(float(v) - expect.duration_s) <= 1e-9)
    if expect.scenario is not None:
        check("scenario", lambda v: isinstance(v, str), lambda v: v == expect.scenario)
    return reasons


def workload_digest(run_dir: Path) -> tuple[Optional[str], list[str]]:
    """sha256 over the run's :data:`WORKLOAD_FILES`, or None plus the missing files."""
    outer = hashlib.sha256()
    missing = []
    for name in WORKLOAD_FILES:
        path = Path(run_dir) / name
        if not path.is_file():
            missing.append(name)
            continue
        outer.update(f"{name}\0{hashlib.sha256(path.read_bytes()).hexdigest()}\n".encode())
    return (None, missing) if missing else (outer.hexdigest(), [])


def run_identity(run_dir: Path) -> dict[str, Optional[str]]:
    """``git_sha`` (only if well-formed) and ``workload`` digest of one run."""
    md, _ = _load_metadata(run_dir)
    sha = md.get("git_sha")
    return {"git_sha": sha if valid_sha(sha) else None, "workload": workload_digest(run_dir)[0]}


def _seconds(md: dict, key: str) -> Optional[float]:
    raw = md.get(key)
    if not raw:
        return None
    try:
        return datetime.fromisoformat(str(raw).replace("Z", "+00:00")).timestamp()
    except (ValueError, OverflowError, OSError):
        return None


def _count(value: Any) -> Optional[int]:
    """*value* as a non-negative event or packet count, None if it is not one."""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value if value >= 0 else None
    if isinstance(value, str) and value.isdigit():
        return int(value)
    return None


def trace_loss(run_dir: Path, md: dict) -> list[str]:
    """``trace_loss:*`` reasons of one run (manifest channels and metadata)."""
    reasons: list[str] = []
    manifest_path = Path(run_dir) / "trace_manifest.json"
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text())
        except (ValueError, UnicodeDecodeError):
            manifest = None
        if not isinstance(manifest, dict):
            return ["trace_loss:manifest_malformed"]
        channels = manifest.get("channels")
        if not isinstance(channels, dict) or not channels:
            reasons.append("trace_loss:no_channels")
            channels = {}
        for name, ch in sorted(channels.items()):
            ch = ch if isinstance(ch, dict) else {}
            d, lost = _count(ch.get("discarded")), _count(ch.get("lost_packets"))
            if d is None or lost is None or d > 0 or lost > 0:
                reasons.append(f"trace_loss:{name}")
    discarded = md.get("events_discarded")
    if discarded is not None and _count(discarded) != 0:
        reasons.append("trace_loss:all")
    return reasons


def covers_window(md: dict, window_total_s: float) -> bool:
    """Whether the run is at least *window_total_s* long (necessary, not exact)."""
    if md.get("simulated_horizon_s") is not None:
        # simulated time (RSLCPP): the horizon, not the wall duration, is the run length
        horizon = md["simulated_horizon_s"]
        return (isinstance(horizon, (int, float)) and not isinstance(horizon, bool)
                and math.isfinite(horizon) and horizon >= window_total_s)
    start, end = _seconds(md, "started_at"), _seconds(md, "ended_at")
    return start is not None and end is not None and end - start >= window_total_s


def _semantic_block(run_dir: Path) -> dict:
    from lf_eval.semantic.reconcile import analyze_run

    return analyze_run(run_dir)


def check_run(run_dir: Path, window_total_s: Optional[float] = None,
              expect: Optional[Expectations] = None, semantic: bool = True) -> list[str]:
    """Exclusion reasons for one run, [] means included.

    Set-level identity (``expect.require_uniform_identity``) needs the whole
    selection and is applied by :func:`check_runs`; here it only requires a
    well-formed ``git_sha``. ``semantic=False`` skips ``semantic_incomplete``,
    the one check that reads the raw trace.
    """
    run_dir = Path(run_dir)
    md, failed = _load_metadata(run_dir)
    if failed:
        return [failed]
    reasons: list[str] = []
    if expect is not None:
        reasons += _metadata_reasons(md, expect)
    reasons += trace_loss(run_dir, md)
    if window_total_s is not None and not covers_window(md, window_total_s):
        reasons.append("shorter_than_window")
    if semantic and str(md.get("mode")) == "lfc":
        claims = _semantic_block(run_dir).get("claims") or []
        admitted = [c for c in claims if c.get("coverage") != "unsupported"]
        if not admitted or any(c.get("integrity") != "complete" or c.get("overall") != "supported"
                               for c in admitted):
            reasons.append("semantic_incomplete")
    return reasons


def check_runs(run_dirs: Sequence[Path], window_total_s: Optional[float] = None,
               expect: Optional[Expectations] = None) -> list[dict[str, Any]]:
    """One row per run: ``run``, ``mode``, ``included``, ``reasons``."""
    run_dirs = [Path(d) for d in run_dirs]
    all_reasons = [check_run(d, window_total_s, expect) for d in run_dirs]
    if expect is not None and expect.require_uniform_identity:
        idents = []
        for d, reasons in zip(run_dirs, all_reasons):
            ident = run_identity(d)
            reasons += [f"identity_missing:{f}" for f in workload_digest(d)[1]]
            idents.append(ident)
        for key in ("git_sha", "workload"):
            if len({i[key] for i in idents if i[key] is not None}) > 1:
                for ident, reasons in zip(idents, all_reasons):
                    if ident[key] is not None:
                        reasons.append(f"identity_not_uniform:{key}")
    rows = []
    for d, reasons in zip(run_dirs, all_reasons):
        rows.append({"run": d.name, "mode": str(_load_metadata(d)[0].get("mode", "unknown")),
                     "included": not reasons, "reasons": reasons})
    return rows
