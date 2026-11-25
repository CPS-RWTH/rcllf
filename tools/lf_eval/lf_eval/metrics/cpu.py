# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""CPU utilisation and overhead.

Two estimates, reported side by side with their signed relative difference:

* **/proc sampling** — ``cpu_proc.json`` in the run directory, written by
  ``lf-eval sample-cpu`` while the system runs. Authoritative: it counts *all*
  CPU time of the matched processes, including executor spin, DDS threads and
  kernel time attributed to the process.
* **trace-derived** — total in-callback time (sum of ``callback_end -
  callback_start`` over every attributed callback) divided by the trace's wall
  span. This is *callback wall time*, not CPU time, and not a bound on CPU in
  either direction. It misses time in the executor between callbacks, in DDS
  transport threads and in any callback the timeline could not attribute, and it
  counts time a callback spends preempted or blocked as if it were running.

Because the trace figure measures a different quantity, no tolerance between
the two can be derived and the metric issues no verdict on their gap. It
reports ``trace_vs_proc = (trace - proc) / proc`` per run and per mode, so a
reader sees the size and the sign of the gap. It sets no tolerance flag: no
threshold has a recorded basis. On the
sentry runs the gap is systematic per mode, not noise around zero.

Event/field assumptions
-----------------------
Trace side uses ``callback_start``/``callback_end`` only. ``/proc`` side needs no
trace at all. When neither ``cpu_proc.json`` nor a usable timeline span exists,
the metric reports ``available=False``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional, Sequence

from lf_eval.metrics.registry import Metric, MetricResult, register
from lf_eval.metrics.sample_cpu import machine_cores
from lf_eval.model.chain import Chain
from lf_eval.model.run import Run
from lf_eval.model.system import SystemModel

@register("cpu")
class CpuMetric(Metric):
    """CPU utilisation from ``/proc`` sampling and from the trace."""

    summary = "CPU utilisation/overhead (/proc sampling beside the trace estimate)"

    def compute(
        self,
        runs: Sequence[Run],
        system: Optional[SystemModel] = None,
        chains: Sequence[Chain] = (),
        cores: Optional[int] = None,
        **options: Any,
    ) -> MetricResult:
        n_cores = machine_cores(cores)
        per_run: dict[str, Any] = {}
        # Unrounded per-run figures. per_run shows 4 decimals, and a mode summary
        # built from those lost a 4e-5 gap to 0.0, so the summary aggregates these
        # instead.
        raw: dict[str, dict[str, Optional[float]]] = {}
        summary_rows: list[dict[str, Any]] = []

        for run in runs:
            trace_cores = trace_cores_used(run)
            proc = run.cpu_proc or {}
            proc_cores = proc.get("aggregate_cores")
            proc_cores = float(proc_cores) if isinstance(proc_cores, (int, float)) else None

            entry: dict[str, Any] = {
                "mode": run.mode,
                "n_cores_machine": proc.get("n_cores_machine", n_cores),
                "trace_cores": None if trace_cores is None else round(trace_cores, 4),
                "trace_span_s": round(run.timeline.duration_ns / 1e9, 3),
                "trace_callback_time_s": round(
                    run.timeline.cpu_busy_ns() / 1e9, 4
                ),
                "proc_cores": None if proc_cores is None else round(proc_cores, 4),
                "per_node_cores": (
                    {
                        (system.display_of(k) if system is not None else k): v
                        for k, v in per_node_cores(run).items()
                    }
                ),
            }
            if proc_cores is not None and trace_cores is not None:
                entry["trace_vs_proc"] = trace_vs_proc(proc_cores, trace_cores)
            entry["pct_one_core"] = (
                round((proc_cores if proc_cores is not None else trace_cores or 0.0) * 100, 1)
            )
            entry["pct_machine"] = round(
                entry["pct_one_core"] / entry["n_cores_machine"], 2
            )
            per_run[run.name] = entry
            raw[run.name] = {"proc_cores": proc_cores, "trace_cores": trace_cores}

            if trace_cores is not None or proc_cores is not None:
                summary_rows.append(
                    {
                        "run": run.name,
                        "mode": run.mode,
                        "proc_cores": entry["proc_cores"],
                        "trace_cores": entry["trace_cores"],
                        "pct_machine": entry["pct_machine"],
                        "trace_vs_proc": entry.get("trace_vs_proc"),
                    }
                )

        if not summary_rows:
            return MetricResult(
                name=self.name,
                available=False,
                note=(
                    "no CPU data: run `lf-eval sample-cpu` during the experiment "
                    "to produce cpu_proc.json, or supply a trace with callbacks"
                ),
            )

        aggregate = aggregate_by_mode(per_run, raw)
        note = ""
        if all(info["proc_cores"] is None for info in per_run.values()):
            note = (
                "trace-derived only (no cpu_proc.json): this is CALLBACK WALL TIME, "
                "not CPU. It excludes executor and DDS time outside callbacks and "
                "includes time a callback spent preempted or blocked"
            )

        return MetricResult(
            name=self.name,
            data={"per_run": per_run, "by_mode": aggregate},
            summary_rows=summary_rows,
            note=note,
        )

    def plot(self, result: MetricResult, outdir: Path) -> list[Path]:
        if not result.available:
            return []
        return _plot_cpu(result, Path(outdir))


def trace_cores_used(run: Run) -> Optional[float]:
    """Trace-derived cores used: in-callback time / trace wall span."""
    span_ns = run.timeline.duration_ns
    if span_ns <= 0:
        return None
    busy_ns = run.timeline.cpu_busy_ns()
    if busy_ns <= 0:
        return None
    return busy_ns / span_ns


def per_node_cores(run: Run) -> dict[str, float]:
    """Per-node trace-derived cores used, descending."""
    span_ns = run.timeline.duration_ns
    if span_ns <= 0:
        return {}
    values = {
        node: sum(inv.duration_ns for inv in invs) / span_ns
        for node, invs in run.timeline.invocations.items()
    }
    return {
        node: round(value, 5)
        for node, value in sorted(values.items(), key=lambda kv: -kv[1])
    }


def mean_range(values: Sequence[float]) -> tuple[float, float, float]:
    """``(mean, min, max)`` rounded to 4dp; zeros for an empty input."""
    if not values:
        return (0.0, 0.0, 0.0)
    return (
        round(sum(values) / len(values), 4),
        round(min(values), 4),
        round(max(values), 4),
    )


def trace_vs_proc(proc_cores: float, trace_cores: float) -> Optional[float]:
    """Signed relative gap ``(trace - proc) / proc``, unrounded.

    No verdict: the two figures measure different quantities (module
    docstring), so no tolerance between them follows from either measurement.
    ``None`` when ``proc_cores`` is not positive, where the ratio is undefined.
    """
    if proc_cores <= 0:
        return None
    return (trace_cores - proc_cores) / proc_cores


def aggregate_by_mode(per_run: dict[str, dict],
                      raw: Optional[dict[str, dict]] = None) -> dict[str, dict]:
    """Mean+range per mode — the ``report_cpu.py`` deliverable-table row.

    ``raw`` holds each run's unrounded ``proc_cores`` and ``trace_cores``. Means,
    ranges, percentages and the gap are computed from those and rounded only
    on output. Without ``raw`` the per-run values are used as given.
    """
    by_mode: dict[str, list[dict]] = {}
    for name, info in per_run.items():
        values = dict(info)
        if raw is not None and name in raw:
            values.update(raw[name])
        by_mode.setdefault(info.get("mode", "unknown"), []).append(values)

    out: dict[str, dict] = {}
    for mode, entries in by_mode.items():
        proc_vals = [e["proc_cores"] for e in entries if e["proc_cores"] is not None]
        trace_vals = [e["trace_cores"] for e in entries if e["trace_cores"] is not None]
        proc_mean, proc_min, proc_max = mean_range(proc_vals)
        trace_mean, trace_min, trace_max = mean_range(trace_vals)
        n_cores = entries[0].get("n_cores_machine") or 1
        ref_vals = proc_vals or trace_vals
        reference = sum(ref_vals) / len(ref_vals) if ref_vals else 0.0
        out[mode] = {
            "n_runs": len(entries),
            "proc_cores_mean": proc_mean,
            "proc_cores_range": [proc_min, proc_max],
            "trace_cores_mean": trace_mean,
            "trace_cores_range": [trace_min, trace_max],
            "pct_one_core": round(reference * 100, 1),
            "pct_machine": round(reference * 100 / n_cores, 2),
            "trace_vs_proc": _paired_gap(entries),
        }
    return out


def _paired_gap(entries: list[dict]) -> Optional[float]:
    """The mode's ``trace_vs_proc`` over the runs that have both figures only, so a
    run without ``cpu_proc.json`` cannot shift one mean and not the other."""
    both = [e for e in entries if e["proc_cores"] is not None and e["trace_cores"] is not None]
    if not both:
        return None
    return trace_vs_proc(sum(e["proc_cores"] for e in both) / len(both),
                         sum(e["trace_cores"] for e in both) / len(both))


def _plot_cpu(result: MetricResult, outdir: Path) -> list[Path]:
    """Bar chart: cores used per run (/proc vs trace) grouped by mode."""
    import numpy as np

    from lf_eval.plots import style

    per_run = result.data.get("per_run", {})
    if not per_run:
        return []

    names = list(per_run)
    modes = [per_run[n].get("mode", "unknown") for n in names]
    proc = [per_run[n]["proc_cores"] or 0.0 for n in names]
    trace = [per_run[n]["trace_cores"] or 0.0 for n in names]

    fig, ax = style.figure(
        "CPU Utilisation",
        "/proc sampling vs trace-derived callback wall time (trace is not CPU)",
        figsize=(max(8.0, 1.6 * len(names)), 5.0),
    )
    x = np.arange(len(names))
    width = 0.38
    ax.bar(
        x - width / 2,
        proc,
        width,
        label="/proc sampling",
        color=[style.mode_color(m) for m in modes],
        alpha=0.9,
    )
    ax.bar(
        x + width / 2,
        trace,
        width,
        label="trace-derived",
        color=[style.mode_color(m) for m in modes],
        alpha=0.45,
        hatch="//",
    )
    ax.set_xticks(x)
    ax.set_xticklabels([n.replace("_", "\n") for n in names], fontsize=7)
    ax.set_ylabel("Cores used")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    return [style.save(fig, outdir, "cpu_utilisation.png")]
