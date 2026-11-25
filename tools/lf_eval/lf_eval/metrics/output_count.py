# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``output_count``: published values per output topic, from ``outputs.jsonl``.

The first metric written against the results-folder interface (run spec plan
3c). It reads a run's ``outputs.jsonl`` (``rcllf.result/1``), not
the trace, so it works on folders whose traces were not captured or trimmed.

Per run it counts the rows of each topic inside the horizon. The horizon is
an explicit option (``horizon_ns``, filled by ``ros2 lf eval`` from the run's
``run.json`` horizon): an rcllf row counts when ``tag_ns <= horizon_ns`` (the
tag relative to the start tag); a stock row, which has no tag, counts when its
``publish_ns`` lies within ``horizon_ns`` of the run's first output publish
(the R4 stock horizon rule). Without the option every row counts.

``compare()`` gives, per topic and for the total, the group means of the
baseline and the candidate and ``delta = candidate - baseline``.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Optional, Sequence

from lf_eval.metrics.registry import Metric, MetricResult, register
from lf_eval.model.chain import Chain
from lf_eval.model.system import SystemModel

TOTAL = "(total)"


def _rows_of(run: Any) -> Optional[list[dict[str, Any]]]:
    rows = getattr(run, "outputs", None)
    if rows is not None:
        return list(rows)
    path = Path(getattr(run, "path", "")) / "outputs.jsonl"
    if not path.is_file():
        return None
    return [json.loads(x) for x in path.read_text().splitlines() if x.strip()]


def count_rows(rows: Sequence[dict[str, Any]], horizon_ns: Optional[int]) -> dict[str, int]:
    """Rows per topic inside the horizon (see the module docstring)."""
    first_pub = min((r["publish_ns"] for r in rows
                     if r.get("tag_ns") is None and r.get("publish_ns") is not None),
                    default=None)
    counts: dict[str, int] = {}
    for r in rows:
        if horizon_ns is not None:
            if r.get("tag_ns") is not None:
                if r["tag_ns"] > horizon_ns:
                    continue
            elif r.get("publish_ns") is not None and first_pub is not None:
                if r["publish_ns"] - first_pub > horizon_ns:
                    continue
        counts[r["topic"]] = counts.get(r["topic"], 0) + 1
    return counts


@register("output_count")
class OutputCountMetric(Metric):
    """Published values per ``capture.outputs`` topic, per run and group mean."""

    summary = "Published values per output topic (outputs.jsonl), group mean"
    inputs = ("outputs",)
    options_schema = {"horizon_ns": int}

    def compute(
        self,
        runs: Sequence[Any],
        system: Optional[SystemModel] = None,
        chains: Sequence[Chain] = (),
        horizon_ns: Optional[int] = None,
        **_legacy: Any,
    ) -> MetricResult:
        per_run: dict[str, dict[str, int]] = {}
        missing = []
        for run in runs:
            rows = _rows_of(run)
            if rows is None:
                missing.append(run.name)
                continue
            counts = count_rows(rows, horizon_ns)
            counts[TOTAL] = sum(counts.values())
            per_run[run.name] = counts
        if missing or not per_run:
            return MetricResult(name=self.name, available=False,
                                note=f"no outputs.jsonl in {', '.join(missing) or 'any run'}")
        topics = sorted({t for c in per_run.values() for t in c} - {TOTAL}) + [TOTAL]
        per_topic = {}
        for t in topics:
            vals = [c.get(t, 0) for c in per_run.values()]
            per_topic[t] = {"mean": sum(vals) / len(vals), "min": min(vals), "max": max(vals),
                            "runs": len(vals)}
        rows = [{"topic": t, **per_topic[t]} for t in topics]
        return MetricResult(
            name=self.name,
            data={"per_run": per_run, "per_topic": per_topic, "horizon_ns": horizon_ns},
            summary_rows=rows,
        )

    def compare(self, baseline: MetricResult, candidate: MetricResult) -> list[dict[str, Any]]:
        b = baseline.data["per_topic"]
        c = candidate.data["per_topic"]
        topics = sorted((set(b) | set(c)) - {TOTAL}) + [TOTAL]
        out = []
        for t in topics:
            mb = b[t]["mean"] if t in b else 0.0
            mc = c[t]["mean"] if t in c else 0.0
            out.append({"topic": t, "baseline": mb, "candidate": mc, "delta": mc - mb})
        return out
