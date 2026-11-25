# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""End-to-end and per-hop chain latency.

Event/field assumptions
-----------------------
Consumes ``callback_start`` / ``callback_end`` (paired by the ``callback``
pointer), attributed to nodes by :mod:`lf_eval.model.timeline`, and the message
identity the timeline attaches to them from ``rmw_take`` / ``rmw_publish``
(see "Message identity" there). Two pairings exist, and every chain and hop
result says which one produced it (``pairing``):

``message_identity``
    Used for a hop when its upstream node has rmw-attributed publishes and its
    downstream node attributed takes. The pairing rule is
    :func:`lf_eval.chain_latency.upstream_of`, the one the chain walk uses. A
    downstream activation is paired with the invocation that published the
    message it consumed. A downstream *timer* activation of a gated hop consumed
    no message, so it is paired with the publisher of the newest message of the
    upstream that the downstream node took before the tick (data age). Every
    downstream activation ends in one counted outcome (``paired``,
    ``unresolved``, ``startup``, ``other_input``, ``not_chain_data``, see
    :mod:`lf_eval.chain_latency`): an activation whose data may be the
    upstream's but is not named by the trace is ``unresolved`` and never paired
    with an older message. No window applies: the pairs are causal by
    construction. ``complete`` is true when no activation is unresolved.
``timestamp``
    The fallback when the trace has no identity on the hop (no ``vtid``
    context, intra-process delivery). Each upstream ``callback_end`` pairs with
    the first downstream ``callback_start`` after it. This is a *heuristic* with
    no guaranteed error direction: a hop may pair with a downstream invocation
    triggered by a different message, and an unrelated invocation that starts
    before the causal one makes the result too small. ``max_gap_ns`` (CLI
    ``--max-gap-ms``) optionally drops pairs further apart. It has no default:
    a fixed cutoff has no basis independent of the system, so
    without it nothing is dropped, and with it the dropped pairs are counted in
    ``excluded_by_window``.

The end-to-end latency of a chain is sink activation end minus source activation
start. With identity on every hop it is the backward walk of
:func:`lf_eval.chain_latency.walk_chain` from each sink activation. Otherwise
each source activation walks forward hop by hop to the first downstream
invocation starting after the current one's end.

Generalisation over the predecessor
-----------------------------------
``compute_e2e_latency`` there hardcoded one fixed
path *including* the "wait
for the next 100 ms BehaviorPlanner tick" step. Here that behaviour falls out of
the chain model: a hop whose upstream is **timer-gated** (owns a timer and has
subscriptions) pairs against the downstream node's *timer* invocations, because
data genuinely waits for the next tick. Gated hops are reported separately so
they do not swamp the plots.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence

from lf_eval.metrics.registry import Metric, MetricResult, register
from lf_eval.metrics.stats import Summary, pair_forward, summarize
from lf_eval.model.chain import Chain, Hop
from lf_eval.model.run import Run
from lf_eval.model.system import SystemModel
from lf_eval.model.timeline import Invocation, Timeline
from lf_eval.semantic.claims import label_heuristic

PAIRING_IDENTITY = "message_identity"
PAIRING_TIMESTAMP = "timestamp"


@dataclass
class Samples:
    """Latency samples of one hop or chain and how they were paired."""

    values_us: list[float] = field(default_factory=list)
    pairing: str = PAIRING_TIMESTAMP
    #: identity pairing: activations whose data may be the upstream's but is
    #: not named by the trace (never paired with an older message instead)
    unresolved: int = 0
    #: timestamp pairing: pairs dropped by an explicit ``max_gap_ns``
    excluded_by_window: int = 0
    #: identity pairing: activations before the first upstream input (startup)
    startup: int = 0
    #: identity pairing: activations that consumed another node's message
    other_input: int = 0
    #: identity pairing: activations on a channel no traced node publishes
    not_chain_data: int = 0
    #: identity pairing: downstream (hop) or sink (chain) activations examined
    activations: int = 0
    #: identity pairing: :data:`IDENTITY_LFC` or :data:`IDENTITY_ROS`
    identity_source: Optional[str] = None
    #: timestamp pairing: why the run has no identity evidence
    no_identity: Optional[str] = None

    @property
    def complete(self) -> Optional[bool]:
        """Identity pairing: no unresolved activation and at least one sample.

        ``None`` for timestamp pairing, which proves no pair either way.
        """
        if self.pairing != PAIRING_IDENTITY:
            return None
        return self.unresolved == 0 and bool(self.values_us)

    def info(self) -> dict[str, Any]:
        out: dict[str, Any] = {"pairing": self.pairing, "unresolved": self.unresolved,
                               "excluded_by_window": self.excluded_by_window,
                               "complete": self.complete}
        if self.pairing != PAIRING_IDENTITY:
            out["no_identity"] = self.no_identity
        else:
            judged = len(self.values_us) + self.unresolved
            out.update({"identity_source": self.identity_source,
                        "activations": self.activations, "paired": len(self.values_us),
                        "startup": self.startup, "other_input": self.other_input,
                        "not_chain_data": self.not_chain_data,
                        # paired share of the activations that carry this data
                        "coverage": round(len(self.values_us) / judged, 6) if judged else None})
        return out


@dataclass
class HopPairs:
    """Identity pairs of one hop and the count of every other outcome."""

    pairs: list[tuple[Invocation, Invocation]] = field(default_factory=list)
    unresolved: int = 0
    startup: int = 0
    other_input: int = 0
    not_chain_data: int = 0
    activations: int = 0


@register("e2e_latency")
class E2ELatencyMetric(Metric):
    """Per-chain end-to-end latency plus per-hop breakdown."""

    summary = "Per-chain end-to-end latency distributions and per-hop latency"

    def compute(
        self,
        runs: Sequence[Run],
        system: Optional[SystemModel] = None,
        chains: Sequence[Chain] = (),
        max_gap_ns: int = 0,
        include_raw: bool = False,
        **options: Any,
    ) -> MetricResult:
        if not chains:
            return MetricResult(
                name=self.name,
                available=False,
                note=(
                    "no chain resolved — pass --system system.json (a chain is "
                    "derived from its topic graph) or an explicit --chain A->B->C"
                ),
            )

        per_run: dict[str, Any] = {}
        summary_rows: list[dict[str, Any]] = []
        any_sample = False
        #: e2e values paired over all runs and chains, fewer than 2 per chain
        #: when no chain has stats (summarize needs 2 values for a distribution)
        any_paired = 0
        # Raw samples for plotting, kept out of `data` so metrics.json stays
        # small unless --keep-raw was requested.
        raw_samples: dict[str, list[float]] = {}

        for run in runs:
            chain_results: dict[str, Any] = {}
            evidence = run_evidence(run)
            for chain in chains:
                e2e_samples = e2e_latency_samples(run.timeline, chain, max_gap_ns=max_gap_ns,
                                                  evidence=evidence)
                e2e = summarize(e2e_samples.values_us, label=chain.label)
                hops = []
                for hop in chain.hops:
                    # every hop gets a record, an empty one included: a hop
                    # with no resolved sample must show its counters
                    samples = hop_latency_samples(run.timeline, hop, max_gap_ns=max_gap_ns,
                                                  evidence=evidence)
                    stats = summarize(samples.values_us, label=hop.label)
                    hops.append((hop, stats, samples))
                if e2e is not None:
                    raw_samples.setdefault(run.mode, []).extend(e2e.raw_us)
                chain_results[chain.name] = {
                    "nodes": list(chain.display_nodes),
                    "source": chain.source,
                    "e2e": e2e.to_dict(include_raw) if e2e else None,
                    "e2e_pairing": e2e_samples.info(),
                    "hops": {
                        hop.label: {
                            # below 2 values there are no stats, but the count stays true
                            **(stats.to_dict(include_raw) if stats
                               else {"count": len(samples.values_us)}),
                            "gated": hop.gated,
                            **samples.info(),
                        }
                        for hop, stats, samples in hops
                    },
                    # identity evidence of the chain and of every hop has no
                    # unresolved activation (None when a timestamp pairing ran)
                    "complete": _all_complete([e2e_samples] + [s for _h, _s, s in hops]),
                }
                if e2e is not None:
                    any_sample = True
                any_paired += len(e2e_samples.values_us)
                # A chain without a sample keeps its row (n 0 and its counters),
                # so an empty chain is visible rather than missing.
                summary_rows.append(
                    {
                        "run": run.name,
                        "mode": run.mode,
                        "chain": chain.label,
                        "n": e2e.count if e2e else len(e2e_samples.values_us),
                        "mean_ms": round(e2e.mean_us / 1000.0, 3) if e2e else None,
                        "p95_ms": round(e2e.p95_us / 1000.0, 3) if e2e else None,
                        "p99_ms": round(e2e.p99_us / 1000.0, 3) if e2e else None,
                        "pairing": e2e_samples.pairing,
                        "unresolved": e2e_samples.unresolved,
                        "startup": e2e_samples.startup,
                        "coverage": e2e_samples.info().get("coverage"),
                        "complete": chain_results[chain.name]["complete"],
                    }
                )
            per_run[run.name] = {"mode": run.mode, "chains": chain_results}

        if not any_sample:
            # Report both the canonical ids (what pairing actually keys on) and
            # the raw trace names, so a naming mismatch is immediately obvious
            # rather than looking like missing data.
            observed = sorted({n for r in runs for n in r.timeline.nodes})
            raw_names = sorted(
                {v for r in runs for v in r.timeline.trace_names.values()}
            )
            wanted = sorted({n for c in chains for n in c.nodes})
            if any_paired:
                note = (f"{any_paired} chain sample(s) paired, fewer than 2 per chain, "
                        f"which is too few for a distribution")
            else:
                note = (
                    f"no chain activation could be paired; chain nodes {wanted} vs "
                    f"nodes observed in the trace {observed}"
                )
            if raw_names != observed:
                note += f" (raw trace node names: {raw_names})"
            # The rows stay: they carry the counters that say why nothing
            # paired, e.g. every activation unresolved.
            return MetricResult(
                name=self.name,
                data={"per_run": per_run},
                summary_rows=summary_rows,
                available=False,
                note=note,
            )

        result = MetricResult(
            name=self.name,
            data={"per_run": per_run},
            summary_rows=summary_rows,
        )
        # Attached, not serialised: `MetricResult.to_dict` only emits the
        # declared fields, so plots get full distributions for free.
        result.raw_samples_by_mode = raw_samples  # type: ignore[attr-defined]
        # Timestamp pairing is a physical profile, never identity.
        return label_heuristic(result, [r.path for r in runs])

    def plot(self, result: MetricResult, outdir: Path) -> list[Path]:
        if not result.available:
            return []
        return _plot_latency(result, Path(outdir))


def compute_e2e(
    timeline: Timeline,
    chain: Chain,
    max_gap_ns: int = 0,
) -> Optional[Summary]:
    """End-to-end latency summary along ``chain``: source start -> sink end."""
    return summarize(e2e_latency_samples(timeline, chain, max_gap_ns).values_us,
                     label=chain.label)


def compute_hops(
    timeline: Timeline,
    chain: Chain,
    max_gap_ns: int = 0,
) -> list[tuple[Hop, Summary]]:
    """Per-hop latency summaries: upstream ``callback_end`` -> downstream ``callback_start``."""
    out: list[tuple[Hop, Summary]] = []
    for hop in chain.hops:
        stats = summarize(hop_latency_samples(timeline, hop, max_gap_ns).values_us,
                          label=hop.label)
        if stats is not None:
            out.append((hop, stats))
    return out


@dataclass
class Evidence:
    """Where the message identity of one run comes from.

    ``graph`` is the identity graph of the whole run (:class:`lf_eval.chain_latency.Graph`),
    ``None`` when the trace names no message at all. ``source`` is
    :data:`IDENTITY_LFC` (the rcllf dispatch trace: a ``CORR_SUB`` dispatch
    names the publication it consumed), :data:`IDENTITY_ROS` (``rmw_publish`` /
    ``rmw_take`` attributed to callbacks) or ``None``. ``window`` ``[lo, hi)``
    selects the activations that are judged, the graph is never cut.
    """

    graph: Any = None
    source: Optional[str] = None
    window: Optional[tuple[int, int]] = None
    note: Optional[str] = None


IDENTITY_LFC = "lfc_dispatch"
IDENTITY_ROS = "ros_trace"


def timeline_has_identity(timeline: Timeline) -> bool:
    """Whether the ROS trace names messages at all (thread context and rmw events).

    A trace without one successful attributed take or publish (no ``vtid``
    context, or no ``rmw_*`` events recorded) can only be paired by timestamps.
    One that has them is judged by identity on every hop: a hop whose messages
    the trace does not name then shows as unresolved activations instead of
    silently switching to timestamps.
    """
    ident = timeline.identity
    return bool(ident.get("takes_attributed") or ident.get("publishes_attributed"))


def timeline_evidence(timeline: Timeline, window: Optional[tuple[int, int]] = None,
                      inputs: Optional[dict] = None) -> Evidence:
    """:class:`Evidence` from a ROS-trace timeline, ``graph`` None without identity.

    *inputs*: declared inputs per timeline node (:func:`declared_timeline_inputs`).
    Without them no callback is excluded as ``not_chain_data``.
    """
    if not timeline_has_identity(timeline):
        return Evidence(note="the trace has no attributed rmw_take or rmw_publish")
    from lf_eval.chain_latency import timeline_graph

    return Evidence(timeline_graph(timeline, inputs), IDENTITY_ROS, window)


def declared_timeline_inputs(system: dict) -> dict:
    """:func:`lf_eval.chain_latency.declared_inputs` keyed by the timeline's node ids.

    :func:`lf_eval.model.run.load_run` names timeline nodes by
    :meth:`SystemModel.canonical_of`, while ``declared_inputs`` keys by the
    ``system.json`` node key, so each key is mapped through the same resolver.
    A key the model cannot resolve keeps its own name.
    """
    from lf_eval.chain_latency import declared_inputs
    from lf_eval.model.system import parse_system

    model = parse_system(system)
    return {(model.canonical_of(k) or k): v for k, v in declared_inputs(system).items()}


def run_evidence(run: Run) -> Evidence:
    """Cached :func:`_load_run_evidence` (the e2e and determinism metrics share it)."""
    cached = getattr(run, "_identity_evidence", None)
    if cached is None:
        cached = _load_run_evidence(run)
        run._identity_evidence = cached  # type: ignore[attr-defined]
    return cached


def _load_run_evidence(run: Run) -> Evidence:
    """The identity evidence of *run*: the dispatch trace for lfc, else the ROS trace.

    A run directory with ``rcllf_trace_manifest.json`` is an lfc run whose ROS
    trace does not see the runtime's message flow (the rcllf runtime delivers
    by dispatch, not by an rclcpp take), so its pairs come from the dispatch
    trace. A manifest whose dispatch trace cannot be read yields no evidence,
    with the reason in ``note``; it never falls back to the ROS trace.
    """
    window = run.window_ns
    path = Path(run.path) if run.path is not None else None
    if path is not None and (path / LFC_MANIFEST).is_file():
        import json

        from lf_eval.chain_latency import lfc_graph
        from lf_eval.semantic.reader import read_run

        try:
            static = json.loads((path / LFC_MANIFEST).read_text())
            system = json.loads((path / "system.json").read_text())
            events, stats = read_run(path)
        except (OSError, ValueError) as exc:
            return Evidence(note=f"lfc dispatch trace unreadable: {exc}")
        if stats.read_error:
            return Evidence(note=f"lfc dispatch trace unreadable: {stats.read_error}")
        return Evidence(lfc_graph(events, static, system), IDENTITY_LFC, window)
    timeline = run.full_timeline if run.full_timeline is not None else run.timeline
    inputs = None
    if path is not None and (path / "system.json").is_file():
        import json

        try:
            inputs = declared_timeline_inputs(json.loads((path / "system.json").read_text()))
        except (OSError, ValueError) as exc:
            return Evidence(note=f"system.json unreadable: {exc}")
    return timeline_evidence(timeline, window, inputs)


#: written by the rcllf runtime next to the trace of every lfc run
LFC_MANIFEST = "rcllf_trace_manifest.json"


def _evidence(timeline: Timeline, evidence: Optional[Evidence]) -> Evidence:
    return evidence if evidence is not None else timeline_evidence(timeline)


def _downstream_activations(graph, hop: Hop, window: Optional[tuple[int, int]]) -> list:
    """Activations of the hop's downstream node that carry its data, inside *window*.

    The graph counterpart of :func:`_downstream_invocations`: the timer
    activations of a gated hop, else the subscription (or ``CORR_SUB``)
    activations, else every activation of the node.
    """
    from lf_eval.chain_latency import in_window

    acts = graph.by_node.get(hop.downstream, [])
    timers = [a for a in acts if a.timer]
    subs = [a for a in acts if not a.timer]
    chosen = timers if hop.gated and timers else (subs or acts)
    return [a for a in chosen if in_window(a, window)]


def identity_pairs(timeline: Timeline, hop: Hop, evidence: Optional[Evidence] = None) -> HopPairs:
    """``(upstream, downstream)`` activation pairs of ``hop`` by message identity.

    Downstream activations are those of :func:`_downstream_activations`, each
    judged by :func:`lf_eval.chain_latency.upstream_of` without the state hop
    for a subscription that consumed another input (it carries no data of this
    hop, ``other_input``). A timer activation takes the newest-input rule. The
    pairs are :class:`lf_eval.chain_latency.Activation` objects (``start``,
    ``end`` in ns). Empty when the evidence has no graph.
    """
    from lf_eval import chain_latency as cl

    ev = _evidence(timeline, evidence)
    out = HopPairs()
    if ev.graph is None:
        return out
    for act in _downstream_activations(ev.graph, hop, ev.window):
        out.activations += 1
        outcome, src, _gated = cl.upstream_of(ev.graph, act, hop.upstream,
                                              state_hop_for_other_inputs=False)
        if outcome == cl.PAIRED:
            out.pairs.append((src, act))
        elif outcome == cl.UNRESOLVED:
            out.unresolved += 1
        elif outcome == cl.STARTUP:
            out.startup += 1
        elif outcome == cl.OTHER_INPUT:
            out.other_input += 1
        else:
            out.not_chain_data += 1
    return out


def _all_complete(samples: list[Samples]) -> Optional[bool]:
    """True only when every sample set is complete identity evidence.

    False when any identity-paired set is incomplete. None otherwise: a set
    paired by timestamps proves nothing either way, so it never lets the chain
    count as complete.
    """
    flags = [s.complete for s in samples]
    if any(f is False for f in flags):
        return False
    return True if flags and all(f is True for f in flags) else None


def hop_latency_samples(timeline: Timeline, hop: Hop, max_gap_ns: int = 0,
                        evidence: Optional[Evidence] = None) -> Samples:
    """Upstream end -> downstream start per paired message.

    Identity pairing whenever the run has identity evidence, on every hop:
    a hop whose messages the evidence does not name reports its activations
    as ``unresolved``. Timestamp pairing only when the run has no identity at
    all, and then ``complete`` is ``None``.
    """
    ev = _evidence(timeline, evidence)
    if ev.graph is not None:
        hp = identity_pairs(timeline, hop, ev)
        return Samples([(dn.start - up.end) / 1_000.0 for up, dn in hp.pairs],
                       PAIRING_IDENTITY, unresolved=hp.unresolved, startup=hp.startup,
                       other_input=hp.other_input, not_chain_data=hp.not_chain_data,
                       activations=hp.activations, identity_source=ev.source)
    up_invs = _upstream_invocations(timeline, hop)
    dn_invs = _downstream_invocations(timeline, hop)
    if not up_invs or not dn_invs:
        return Samples(no_identity=ev.note)
    pairs = pair_forward([inv.end_ns for inv in up_invs], [inv.start_ns for inv in dn_invs])
    kept = [t - a for a, t in pairs if max_gap_ns <= 0 or t - a <= max_gap_ns]
    return Samples([v / 1_000.0 for v in kept], PAIRING_TIMESTAMP,
                   excluded_by_window=len(pairs) - len(kept), no_identity=ev.note)


def e2e_latency_samples(timeline: Timeline, chain: Chain, max_gap_ns: int = 0,
                        evidence: Optional[Evidence] = None) -> Samples:
    """End-to-end latency samples along ``chain``: source start -> sink end.

    With identity evidence, the backward walk of
    :func:`lf_eval.chain_latency.walk_chain` from every sink activation in the
    window. Without any, each source activation advances to the first
    invocation of the next node that *starts* after the current node's end,
    and activations that cannot traverse the whole chain are dropped.
    """
    if len(chain.nodes) < 2:
        return Samples()
    ev = _evidence(timeline, evidence)
    if ev.graph is not None:
        from lf_eval.chain_latency import walk_chain

        walk = walk_chain(ev.graph, list(chain.nodes), sink_window=ev.window)
        return Samples([v / 1_000.0 for v in walk["physical_ns"]], PAIRING_IDENTITY,
                       unresolved=sum(walk["unresolved"].values()),
                       startup=sum(walk["startup"].values()),
                       not_chain_data=walk["not_chain_data"], activations=walk["sinks"],
                       identity_source=ev.source)

    source_invs = _source_invocations(timeline, chain.nodes[0])
    if not source_invs:
        return Samples(no_identity=ev.note)

    # Pre-sort each downstream node's invocations once.
    downstream: list[tuple[Hop, list[Invocation]]] = []
    for hop in chain.hops:
        invs = _downstream_invocations(timeline, hop)
        if not invs:
            return Samples(no_identity=ev.note)
        downstream.append((hop, invs))

    latencies_us: list[float] = []
    excluded = 0
    for source in source_invs:
        cursor_end = source.end_ns
        ok = True
        for hop, invs in downstream:
            nxt = _first_after(invs, cursor_end)
            if nxt is None:
                ok = False
                break
            if max_gap_ns > 0 and nxt.start_ns - cursor_end > max_gap_ns:
                excluded += 1
                ok = False
                break
            cursor_end = nxt.end_ns
        if ok and cursor_end > source.start_ns:
            latencies_us.append((cursor_end - source.start_ns) / 1_000.0)

    return Samples(latencies_us, PAIRING_TIMESTAMP, excluded_by_window=excluded,
                   no_identity=ev.note)


def _source_invocations(timeline: Timeline, node: str) -> list[Invocation]:
    """Activations of a chain's source node — its timer, if it has one."""
    timer = timeline.timer_invocations.get(node)
    return timer if timer else timeline.invocations.get(node, [])


def _upstream_invocations(timeline: Timeline, hop: Hop) -> list[Invocation]:
    """Invocations that *emit* data across ``hop``.

    For a gated upstream only the timer callback publishes (its subscription
    callbacks merely absorb data), so pairing must start from the timer end —
    otherwise half the pairs would measure from an absorbing subscription and
    the mean would be a meaningless blend of the two.
    """
    if hop.gated:
        timer = timeline.timer_invocations.get(hop.upstream)
        if timer:
            return timer
    return timeline.invocations.get(hop.upstream, [])


def _downstream_invocations(timeline: Timeline, hop: Hop) -> list[Invocation]:
    """Invocations that carry data across ``hop``.

    For a gated hop the data is republished by the downstream node's *timer*, so
    that is what must be paired against; otherwise the subscription callbacks
    (falling back to all invocations when subscriptions were not resolvable).
    """
    if hop.gated:
        timer = timeline.timer_invocations.get(hop.downstream)
        if timer:
            return timer
    subs = timeline.sub_invocations.get(hop.downstream)
    if subs:
        return subs
    return timeline.invocations.get(hop.downstream, [])


def _first_after(invs: Sequence[Invocation], after_ns: int) -> Optional[Invocation]:
    """First invocation starting after ``after_ns``."""
    starts = [inv.start_ns for inv in invs]
    idx = bisect.bisect_right(starts, after_ns)
    return invs[idx] if idx < len(invs) else None


def _plot_latency(result: MetricResult, outdir: Path) -> list[Path]:
    """Two standard plots: E2E histogram+CDF, and per-hop mean bars."""
    import numpy as np

    from lf_eval.plots import style

    per_run = result.data.get("per_run", {})
    written: list[Path] = []

    # ── Plot 1: E2E distribution + CDF, grouped by mode ──────────────────
    by_mode: dict[str, list[float]] = {}
    means_by_mode: dict[str, list[float]] = {}
    # Prefer the raw samples attached by compute() (always present); fall back
    # to whatever --keep-raw serialised into the payload.
    attached = getattr(result, "raw_samples_by_mode", None) or {}
    for mode, values in attached.items():
        by_mode.setdefault(mode, []).extend(v / 1000.0 for v in values)
    for info in per_run.values():
        mode = info.get("mode", "unknown")
        for chain in info.get("chains", {}).values():
            e2e = chain.get("e2e")
            if not e2e:
                continue
            means_by_mode.setdefault(mode, []).append(e2e["mean_ms"])
            raw = e2e.get("raw_us")
            if raw and mode not in attached:
                by_mode.setdefault(mode, []).extend(v / 1000.0 for v in raw)

    if by_mode:
        fig, axes = style.figure(
            "End-to-End Chain Latency", figsize=(12, 5), ncols=2
        )
        # Rare outliers (a single stalled activation) otherwise stretch the axis
        # so far that every distribution collapses into one bar. Clip the view
        # to the bulk and say so; the CDF still shows the full range's shape.
        combined = np.concatenate([np.asarray(v) for v in by_mode.values()])
        upper = float(np.percentile(combined, 99.5))
        lower = float(np.min(combined))
        span = max(upper - lower, 1e-6)
        view = (lower - 0.05 * span, upper + 0.05 * span)
        clipped = int((combined > upper).sum())

        bins = np.linspace(view[0], view[1], 41)
        for mode in style.present_modes(by_mode):
            arr = np.asarray(by_mode[mode])
            axes[0].hist(
                arr,
                bins=bins,
                color=style.mode_color(mode),
                alpha=0.7,
                label=f"{style.mode_label(mode)} (n={arr.size})",
            )
            ordered = np.sort(arr)
            cdf = np.arange(1, ordered.size + 1) / ordered.size
            axes[1].plot(
                ordered,
                cdf,
                color=style.mode_color(mode),
                linewidth=2,
                label=f"{style.mode_label(mode)} (mean={ordered.mean():.2f}ms)",
            )
            for pct, linestyle in ((50, "--"), (95, ":"), (99, "-.")):
                axes[1].axvline(
                    float(np.percentile(ordered, pct)),
                    color=style.mode_color(mode),
                    linestyle=linestyle,
                    alpha=0.5,
                    linewidth=1,
                )
        axes[0].set_xlim(*view)
        axes[0].set_xlabel("Latency (ms)")
        axes[0].set_ylabel("Count")
        axes[0].set_title(
            "Distribution"
            + (f" (≤p99.5; {clipped} outlier(s) clipped)" if clipped else "")
        )
        axes[0].legend(fontsize=8)
        axes[0].grid(axis="y", alpha=0.3)
        axes[1].set_xlim(*view)
        axes[1].set_xlabel("Latency (ms)")
        axes[1].set_ylabel("CDF")
        axes[1].set_title("CDF (dashed=p50, dotted=p95, dash-dot=p99)")
        axes[1].legend(fontsize=8)
        axes[1].grid(alpha=0.3)
        written.append(style.save(fig, outdir, "e2e_latency_cdf.png"))
    elif means_by_mode:
        # No raw samples retained (include_raw=False): bar the per-run means.
        fig, ax = style.figure("End-to-End Chain Latency", "mean per run")
        modes = style.present_modes(means_by_mode)
        ax.bar(
            range(len(modes)),
            [float(np.mean(means_by_mode[m])) for m in modes],
            color=[style.mode_color(m) for m in modes],
            alpha=0.85,
        )
        ax.set_xticks(range(len(modes)))
        ax.set_xticklabels([style.mode_label(m) for m in modes], fontsize=9)
        ax.set_ylabel("Mean E2E latency (ms)")
        ax.grid(axis="y", alpha=0.3)
        written.append(style.save(fig, outdir, "e2e_latency_cdf.png"))

    # ── Plot 2: per-hop mean latency bars, ungated hops only ─────────────
    hop_means: dict[str, dict[str, list[float]]] = {}
    without_stats: set[str] = set()
    for info in per_run.values():
        mode = info.get("mode", "unknown")
        for chain in info.get("chains", {}).values():
            for label, stats in chain.get("hops", {}).items():
                if stats.get("gated"):
                    continue  # dominated by the gating period, plotted apart
                if stats.get("mean_ms") is None:
                    # fewer than 2 samples: no mean to draw, the hop is named below
                    without_stats.add(label)
                    continue
                hop_means.setdefault(label, {}).setdefault(mode, []).append(
                    stats["mean_ms"]
                )

    if hop_means:
        labels = list(hop_means)
        modes = style.present_modes(
            {m for per in hop_means.values() for m in per}
        )
        fig, ax = style.figure(
            "Per-Hop Latency along Chain",
            "upstream callback_end → downstream callback_start (gated hops omitted)",
            figsize=(max(8.0, 1.4 * len(labels)), 5.0),
        )
        x = np.arange(len(labels))
        width = 0.8 / max(len(modes), 1)
        offsets = np.linspace(
            -width * (len(modes) - 1) / 2, width * (len(modes) - 1) / 2, len(modes)
        )
        for offset, mode in zip(offsets, modes):
            # a run mode without a mean on a hop draws no bar, never a 0 bar
            values = [
                float(np.mean(hop_means[label][mode]))
                if mode in hop_means[label]
                else np.nan
                for label in labels
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
            [style.short_label(label) for label in labels],
            rotation=30,
            ha="right",
            fontsize=8,
        )
        ax.set_ylabel("Mean hop latency (ms)")
        if without_stats:
            ax.set_title("hops with fewer than 2 samples in a run, not drawn: "
                         + ", ".join(style.short_label(h) for h in sorted(without_stats)),
                         fontsize=8)
        ax.legend(fontsize=8)
        ax.grid(axis="y", alpha=0.3)
        written.append(style.save(fig, outdir, "hop_latency_bars.png"))

    return written
