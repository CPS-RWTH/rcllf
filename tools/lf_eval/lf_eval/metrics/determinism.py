# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Determinism: causal ordering and cross-run reproducibility.

Three sub-metrics, all ported from ``analyze_traces.py`` and generalised off a
hardcoded system:

1. **Ordering invariant** (per run). For every ungated hop of the chain, each
   paired downstream ``callback_start`` must occur at or after the upstream
   ``callback_end`` — i.e. a downstream callback must not begin before the
   upstream that feeds it has finished. Violations mean the two overlapped, which
   under stock ROS2 happens whenever independent executor threads race.
   Timer-gated hops are excluded: their downstream fires on its own tick and is
   legitimately unordered with respect to any single upstream activation.
   The verdict is ``FAIL`` on a violation, ``INCOMPLETE`` when the evidence
   does not cover the chain (an identity-paired hop with an unresolved
   activation, an ungated hop without a single pair, or a hop paired by
   timestamps, which proves no pair), ``PASS`` only with none of these, and
   ``N/A`` when no hop could be paired and no activation was left
   unresolved (activations without a traced upstream are INCOMPLETE). ``INCOMPLETE`` never counts as
   a pass. Identity pairs come from the run's evidence
   (:func:`lf_eval.metrics.e2e_latency.run_evidence`): the rcllf dispatch
   trace of an lfc run, else the ROS trace. Timestamps pair a hop only when
   the run has no identity at all.

2. **Timer jitter** (per run). Inter-arrival intervals of each timer node's
   ``callback_start`` versus the period declared in ``system.json``. Lower
   dispersion = more predictable release times.

3. **Cross-run consistency** (needs >= 2 runs of the same mode). Per hop,
   pairwise KS tests between same-mode runs, plus a coefficient of variation of
   per-run means. A deterministic runtime should reproduce its latency
   distribution run to run; a large KS statistic / CoV indicates it does not.
   Every run with at least one paired sample takes part. The two-sample KS
   test is defined for any sample sizes and its p-value already accounts for
   them, so each comparison reports ``n_a`` and ``n_b`` instead of dropping
   small runs. Runs of the mode with no paired sample on a hop are listed under
   ``excluded_runs`` with that reason; short runs are never
   dropped silently.

Event/field assumptions
-----------------------
Same as :mod:`lf_eval.metrics.e2e_latency`: ``callback_start``/``callback_end``
with the timeline's message identity where the trace carries it (the
``pairing`` of each hop says whether identity or timestamp pairing ran), plus
``system.json`` timer periods (or the ``rcl_timer_init`` ``period``
field recorded in the timeline when ``system.json`` lacks them). scipy is
optional: without it KS p-values are ``NaN`` but the statistic is still computed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence

from lf_eval.metrics.registry import Metric, MetricResult, register
from lf_eval.metrics.e2e_latency import (
    PAIRING_IDENTITY,
    PAIRING_TIMESTAMP,
    Evidence,
    hop_latency_samples,
    identity_pairs,
    run_evidence,
    timeline_evidence,
)
from lf_eval.metrics.stats import (
    coefficient_of_variation,
    ks_2samp,
    pair_forward,
    summarize,
)
from lf_eval.model.chain import Chain
from lf_eval.model.run import Run
from lf_eval.model.system import SystemModel
from lf_eval.model.timeline import Timeline
from lf_eval.semantic.claims import label_heuristic

@dataclass
class OrderingResult:
    """Outcome of the causal-ordering check."""

    pairs_checked: int
    violations: int
    details: list[dict[str, Any]]
    #: hop label -> pairing used for it (``message_identity`` or ``timestamp``)
    pairing: dict[str, str] = field(default_factory=dict)
    #: pairs dropped by an explicit ``max_gap_ns`` (timestamp pairing only)
    excluded_by_window: int = 0
    #: identity pairing: downstream activations whose upstream the trace does not name
    unresolved: int = 0
    #: identity pairing: downstream activations before any upstream input
    startup: int = 0
    #: ungated hops of the chain that produced no pair to check
    hops_without_pairs: list[str] = field(default_factory=list)

    @property
    def verdict(self) -> str:
        if self.violations:
            return "FAIL"
        if self.pairs_checked == 0 and not self.unresolved:
            return "N/A"
        if (self.unresolved or self.hops_without_pairs
                or PAIRING_TIMESTAMP in self.pairing.values()):
            return "INCOMPLETE"
        return "PASS"

    @property
    def violation_rate(self) -> float:
        if self.pairs_checked == 0:
            return 0.0
        return self.violations / self.pairs_checked

    def to_dict(self) -> dict[str, Any]:
        return {
            "verdict": self.verdict,
            "pairs_checked": self.pairs_checked,
            "violations": self.violations,
            "violation_rate_pct": round(100.0 * self.violation_rate, 4),
            "first_violations": self.details[:5],
            "pairing": dict(self.pairing),
            "excluded_by_window": self.excluded_by_window,
            "unresolved": self.unresolved,
            "startup": self.startup,
            "hops_without_pairs": list(self.hops_without_pairs),
        }


@register("determinism")
class DeterminismMetric(Metric):
    """Causal ordering, timer jitter, and cross-run reproducibility."""

    summary = "Ordering invariant, timer jitter, and cross-run reproducibility"
    cross_run = True

    def compute(
        self,
        runs: Sequence[Run],
        system: Optional[SystemModel] = None,
        chains: Sequence[Chain] = (),
        max_gap_ns: int = 0,
        **options: Any,
    ) -> MetricResult:
        if not runs:
            return MetricResult(
                name=self.name, available=False, note="no runs supplied"
            )

        timer_periods = system.timer_periods() if system is not None else {}
        per_run: dict[str, Any] = {}
        summary_rows: list[dict[str, Any]] = []

        for run in runs:
            evidence = run_evidence(run)
            ordering = {
                chain.name: compute_ordering(
                    run.timeline, chain, max_gap_ns=max_gap_ns, evidence=evidence
                )
                for chain in chains
            }
            jitter = compute_timer_jitter(run.timeline, timer_periods)
            if system is not None:
                # Report jitter under display names (canonical ids are internal).
                jitter = {system.display_of(k): v for k, v in jitter.items()}
            per_run[run.name] = {
                "mode": run.mode,
                "ordering": {k: v.to_dict() for k, v in ordering.items()},
                "timer_jitter": jitter,
            }
            for chain_name, res in ordering.items():
                summary_rows.append(
                    {
                        "run": run.name,
                        "mode": run.mode,
                        "chain": chain_name,
                        "ordering": res.verdict,
                        "violations": f"{res.violations}/{res.pairs_checked}",
                        "rate_pct": round(100.0 * res.violation_rate, 3),
                        "unresolved": res.unresolved,
                    }
                )

        consistency = compute_consistency(runs, chains, max_gap_ns=max_gap_ns)

        note = ""
        if not chains:
            note = (
                "no chain resolved — ordering invariant skipped; "
                "timer jitter still reported"
            )
        if len(runs) < 2:
            note = (note + "; " if note else "") + (
                "single run — cross-run consistency needs >= 2 runs of one mode"
            )

        available = bool(summary_rows) or bool(
            any(info["timer_jitter"] for info in per_run.values())
        )
        # Overlap counting is timestamp-based: a heuristic profile.
        return label_heuristic(
            MetricResult(
                name=self.name,
                data={"per_run": per_run, "consistency": consistency},
                summary_rows=summary_rows,
                available=available,
                note=note,
            ),
            [r.path for r in runs],
        )

    def plot(self, result: MetricResult, outdir: Path) -> list[Path]:
        if not result.available:
            return []
        return _plot_determinism(result, Path(outdir))


def compute_ordering(
    timeline: Timeline,
    chain: Chain,
    max_gap_ns: int = 0,
    evidence: Optional[Evidence] = None,
) -> OrderingResult:
    """Check that paired downstream starts follow upstream ends.

    *evidence* defaults to the identity of *timeline* itself.
    """
    if evidence is None:
        evidence = timeline_evidence(timeline)
    violations = 0
    checked = 0
    excluded = 0
    unresolved = startup = 0
    no_pairs: list[str] = []
    details: list[dict[str, Any]] = []
    pairing: dict[str, str] = {}

    for hop in chain.ungated_hops:
        if evidence.graph is not None:
            # The downstream callback that took a message against the callback
            # that published it: an overlap is a real race, not a mispairing.
            pairing[hop.label] = PAIRING_IDENTITY
            hp = identity_pairs(timeline, hop, evidence)
            unresolved += hp.unresolved
            startup += hp.startup
            pairs = [(_Span(up.start, up.end), dn.start) for up, dn in hp.pairs]
        else:
            up_invs = timeline.invocations.get(hop.upstream, [])
            dn_invs = timeline.sub_invocations.get(hop.downstream) or timeline.invocations.get(
                hop.downstream, []
            )
            if not up_invs or not dn_invs:
                no_pairs.append(hop.label)
                continue
            pairing[hop.label] = PAIRING_TIMESTAMP
            dn_starts = sorted(inv.start_ns for inv in dn_invs)
            # Pair on upstream *start* (as the predecessor did) so that a downstream
            # which began during the upstream callback is still caught: pairing on
            # the end would silently skip exactly the overlapping case.
            by_start = {inv.start_ns: inv for inv in up_invs}
            pairs = []
            for up_start, dn_start in pair_forward([inv.start_ns for inv in up_invs], dn_starts):
                if max_gap_ns > 0 and dn_start - up_start > max_gap_ns:
                    excluded += 1
                else:
                    inv = by_start[up_start]
                    pairs.append((_Span(inv.start_ns, inv.end_ns), dn_start))
        if not pairs:
            no_pairs.append(hop.label)
        for up_inv, dn_start in pairs:
            checked += 1
            if dn_start < up_inv.end_ns:
                violations += 1
                if len(details) < 10:
                    details.append(
                        {
                            "upstream": hop.upstream,
                            "downstream": hop.downstream,
                            "upstream_start_ns": up_inv.start_ns,
                            "upstream_end_ns": up_inv.end_ns,
                            "downstream_start_ns": dn_start,
                            "overlap_ns": up_inv.end_ns - dn_start,
                        }
                    )

    return OrderingResult(pairs_checked=checked, violations=violations, details=details,
                          pairing=pairing, excluded_by_window=excluded, unresolved=unresolved,
                          startup=startup, hops_without_pairs=no_pairs)


@dataclass(frozen=True)
class _Span:
    """Start and end of the upstream activation of one checked pair, in ns."""

    start_ns: int
    end_ns: int


def compute_timer_jitter(
    timeline: Timeline,
    timer_periods: Optional[dict[str, float]] = None,
) -> dict[str, dict[str, float]]:
    """Inter-arrival statistics for each timer node.

    Expected periods come from ``system.json`` when available, else from the
    ``rcl_timer_init`` ``period`` field captured in the timeline.
    """
    import numpy as np

    timer_periods = timer_periods or {}
    traced_periods = _traced_periods(timeline)
    out: dict[str, dict[str, float]] = {}

    for node, invs in timeline.timer_invocations.items():
        if len(invs) < 3:
            continue
        expected = timer_periods.get(node) or traced_periods.get(node)
        starts = np.asarray([inv.start_ns for inv in invs], dtype=float)
        intervals_ms = np.diff(starts) / 1_000_000.0
        if intervals_ms.size < 2:
            continue
        entry = {
            "count": int(intervals_ms.size),
            "mean_period_ms": round(float(np.mean(intervals_ms)), 4),
            "std_ms": round(float(np.std(intervals_ms)), 4),
            "p95_interval_ms": round(float(np.percentile(intervals_ms, 95)), 4),
        }
        if expected:
            jitter = np.abs(intervals_ms - expected)
            entry.update(
                {
                    "expected_period_ms": float(expected),
                    "mean_jitter_ms": round(float(np.mean(jitter)), 4),
                    "max_jitter_ms": round(float(np.max(jitter)), 4),
                    "p95_jitter_ms": round(float(np.percentile(jitter, 95)), 4),
                    "p99_jitter_ms": round(float(np.percentile(jitter, 99)), 4),
                }
            )
        out[node] = entry
    return out


def _traced_periods(timeline: Timeline) -> dict[str, float]:
    """Timer periods recovered from ``rcl_timer_init``, in ms."""
    out: dict[str, float] = {}
    for info in timeline.callbacks.values():
        if info.kind == "timer" and info.node and info.period_ns:
            out.setdefault(info.node, info.period_ns / 1_000_000.0)
    return out


def compute_consistency(
    runs: Sequence[Run],
    chains: Sequence[Chain],
    max_gap_ns: int = 0,
) -> dict[str, Any]:
    """Pairwise KS tests + CoV of per-run means, grouped by mode."""
    import numpy as np

    by_mode: dict[str, list[Run]] = {}
    for run in runs:
        by_mode.setdefault(run.mode, []).append(run)

    out: dict[str, Any] = {}
    for mode, mode_runs in by_mode.items():
        if len(mode_runs) < 2:
            continue
        # hop label -> run name -> raw latencies (us)
        samples: dict[str, dict[str, list[float]]] = {}
        empty: dict[str, list[str]] = {}
        for run in mode_runs:
            for chain in chains:
                for hop in chain.hops:
                    values = _hop_latencies(
                        run.timeline, hop, max_gap_ns=max_gap_ns,
                        evidence=run_evidence(run),
                    )
                    if values:
                        samples.setdefault(hop.label, {})[run.name] = values
                    else:
                        empty.setdefault(hop.label, []).append(run.name)

        hop_out: dict[str, Any] = {}
        # A hop with fewer than two sampled runs has nothing to compare, but its
        # runs are still named, so a reader sees why the hop has no KS result.
        too_few: dict[str, dict[str, str]] = {}
        for label in sorted(set(samples) | set(empty)):
            per_run = samples.get(label, {})
            if len(per_run) < 2:
                too_few[label] = {
                    **{k: "only run with a paired sample on this hop" for k in per_run},
                    **{k: "no paired sample on this hop" for k in sorted(set(empty.get(label, ())))},
                }
                continue
            names = sorted(per_run)
            comparisons = []
            for i in range(len(names)):
                for j in range(i + 1, len(names)):
                    a, b = per_run[names[i]], per_run[names[j]]
                    ks = ks_2samp(a, b)
                    if ks is None:
                        continue
                    stat, pvalue = ks
                    comparisons.append(
                        {
                            "run_a": names[i],
                            "run_b": names[j],
                            "n_a": len(a),
                            "n_b": len(b),
                            "ks_statistic": round(stat, 4),
                            "p_value": None
                            if pvalue != pvalue  # NaN when scipy is absent
                            else round(pvalue, 6),
                            "mean_diff_us": round(
                                float(np.mean(a) - np.mean(b)), 3
                            ),
                        }
                    )
            means = [float(np.mean(v)) for v in per_run.values()]
            hop_out[label] = {
                "n_runs": len(per_run),
                "n_samples_per_run": {k: len(v) for k, v in per_run.items()},
                "excluded_runs": {k: "no paired sample on this hop"
                                  for k in sorted(set(empty.get(label, ())))},
                "mean_us_per_run": {k: round(float(np.mean(v)), 3) for k, v in per_run.items()},
                "cov_of_means": round(coefficient_of_variation(means), 5),
                "max_ks_statistic": round(
                    max((c["ks_statistic"] for c in comparisons), default=0.0), 4
                ),
                "comparisons": comparisons,
            }

        if hop_out or too_few:
            out[mode] = {"n_runs": len(mode_runs), "hops": hop_out}
            if too_few:
                out[mode]["hops_without_comparison"] = too_few

    return out


def _hop_latencies(timeline: Timeline, hop, max_gap_ns: int,
                   evidence: Optional[Evidence] = None) -> list[float]:
    """Raw per-hop latencies in microseconds, paired as in ``e2e_latency``."""
    return hop_latency_samples(timeline, hop, max_gap_ns=max_gap_ns,
                               evidence=evidence).values_us


def _plot_determinism(result: MetricResult, outdir: Path) -> list[Path]:
    """Ordering violation rate per run, and timer-jitter std per node."""
    import numpy as np

    from lf_eval.plots import style

    per_run = result.data.get("per_run", {})
    written: list[Path] = []

    # ── Plot 1: ordering violation rate per run ──────────────────────────
    rows = []
    for name, info in per_run.items():
        for chain_name, ordering in info.get("ordering", {}).items():
            if ordering.get("pairs_checked", 0) > 0:
                rows.append(
                    (name, info.get("mode", "unknown"), ordering["violation_rate_pct"],
                     ordering["violations"], ordering["pairs_checked"])
                )
    if rows:
        fig, ax = style.figure(
            "Ordering Invariant: Violation Rate",
            "downstream callback started before its upstream finished",
            figsize=(max(7.0, 1.6 * len(rows)), 5.0),
        )
        labels = [r[0].replace("_", "\n") for r in rows]
        rates = [r[2] for r in rows]
        bars = ax.bar(
            range(len(rows)),
            rates,
            color=[style.mode_color(r[1]) for r in rows],
            alpha=0.85,
        )
        for bar, row in zip(bars, rows):
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height(),
                f"{row[3]}/{row[4]}",
                ha="center",
                va="bottom",
                fontsize=8,
            )
        ax.set_xticks(range(len(rows)))
        ax.set_xticklabels(labels, fontsize=7)
        ax.set_ylabel("Violation rate (%)")
        ax.set_ylim(0, max(max(rates) * 1.25, 1.0))
        ax.grid(axis="y", alpha=0.3)
        written.append(style.save(fig, outdir, "ordering_invariant.png"))

    # ── Plot 2: timer jitter std per node, grouped by mode ───────────────
    jitter: dict[str, dict[str, list[float]]] = {}
    for info in per_run.values():
        mode = info.get("mode", "unknown")
        for node, stats in info.get("timer_jitter", {}).items():
            jitter.setdefault(node, {}).setdefault(mode, []).append(stats["std_ms"])

    if jitter:
        nodes = sorted(jitter)
        modes = style.present_modes({m for per in jitter.values() for m in per})
        fig, ax = style.figure(
            "Timer Jitter (inter-arrival std) per Node",
            "lower = more regular release times",
            figsize=(max(8.0, 1.1 * len(nodes)), 5.0),
        )
        x = np.arange(len(nodes))
        width = 0.8 / max(len(modes), 1)
        offsets = np.linspace(
            -width * (len(modes) - 1) / 2, width * (len(modes) - 1) / 2, len(modes)
        )
        for offset, mode in zip(offsets, modes):
            values = [
                float(np.mean(jitter[n][mode])) if mode in jitter[n] else 0.0
                for n in nodes
            ]
            ax.bar(
                x + offset,
                values,
                width,
                color=style.mode_color(mode),
                alpha=0.85,
                label=style.mode_label(mode),
            )
        ax.set_xticks(x)
        ax.set_xticklabels(
            [style.short_label(n) for n in nodes], rotation=30, ha="right", fontsize=8
        )
        ax.set_ylabel("Interval std (ms)")
        ax.legend(fontsize=8)
        ax.grid(axis="y", alpha=0.3)
        written.append(style.save(fig, outdir, "timer_jitter_bars.png"))

    return written
