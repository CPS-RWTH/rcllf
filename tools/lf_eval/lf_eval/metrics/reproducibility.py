# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Cross-run reproducibility: per-node callback sequence identity.

For each node, the run's callback sequence is the list of which of the node's
callbacks fired, in start order: ``timer`` or the subscribed topic. Repeated
runs of one mode are compared with the mode's first run over their common
length. A mode is reproducible when no node diverges.

Only nodes with two or more distinct callbacks can diverge (a one-callback node
always repeats the same label), so they are reported as ``informative_nodes``.
The order *between* nodes is deliberately not compared: parallel executors may
start independent nodes in any physical order without any semantic difference.

Evidence class: trace callback order, without message identity. It answers
"does each node see its inputs in the same order on every run?", the
reproducibility question for every variant including stock and RSLCPP.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any, Optional, Sequence

from lf_eval.metrics.registry import Metric, MetricResult, register
from lf_eval.model.chain import Chain
from lf_eval.model.run import Run
from lf_eval.model.system import SystemModel

EVIDENCE = "trace_callback_order"


def node_sequences(run: Run) -> dict[str, list[str]]:
    """``{node: [label, ...]}`` in start order, label ``timer`` or the topic."""
    tl = run.timeline
    labelled: dict[str, list[tuple[int, str]]] = defaultdict(list)
    for node, invs in tl.timer_invocations.items():
        labelled[node] += [(inv.start_ns, "timer") for inv in invs]
    for (node, topic), invs in tl.topic_invocations.items():
        labelled[node] += [(inv.start_ns, topic) for inv in invs]
    return {node: [lab for _, lab in sorted(items)] for node, items in labelled.items()}


def first_divergence(a: Sequence[str], b: Sequence[str]) -> Optional[int]:
    """Index of the first differing label over the common length, else None."""
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return i
    return None


@register("reproducibility")
class ReproducibilityMetric(Metric):
    summary = "cross-run per-node callback sequence identity, per mode"
    cross_run = True

    def compute(
        self,
        runs: Sequence[Run],
        system: Optional[SystemModel] = None,
        chains: Sequence[Chain] = (),
        **options: Any,
    ) -> MetricResult:
        by_mode: dict[str, list[Run]] = defaultdict(list)
        for run in runs:
            by_mode[run.mode].append(run)
        per_mode: dict[str, Any] = {}
        rows = []
        for mode, mruns in sorted(by_mode.items()):
            seqs = {r.name: node_sequences(r) for r in mruns}
            ref_name = mruns[0].name
            ref = seqs[ref_name]
            informative = sorted(n for n, s in ref.items() if len(set(s)) > 1)
            divergences = []
            compared = {}
            for r in mruns[1:]:
                for node in sorted(set(ref) | set(seqs[r.name])):
                    a, b = ref.get(node, []), seqs[r.name].get(node, [])
                    compared[node] = min(compared.get(node, len(a)), len(a), len(b))
                    idx = first_divergence(a, b)
                    if idx is not None:
                        divergences.append({"run": r.name, "node": node, "first_divergence": idx,
                                            "common_length": min(len(a), len(b))})
            verdict = None if len(mruns) < 2 else not divergences
            per_mode[mode] = {
                "runs": len(mruns), "reference_run": ref_name, "reproducible": verdict,
                "informative_nodes": informative, "compared_length": compared,
                "divergences": divergences,
            }
            rows.append({"mode": mode, "runs": len(mruns),
                         "reproducible": {None: "n/a", True: "yes", False: "no"}[verdict],
                         "diverging_nodes": len({d["node"] for d in divergences})})
        return MetricResult(
            name=self.name, data={"per_mode": per_mode}, summary_rows=rows,
            available=bool(runs), evidence=EVIDENCE,
            note="" if any(len(v) > 1 for v in by_mode.values()) else "needs two or more runs per mode",
        )
