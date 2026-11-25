# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Soundness/topology-diff checker: static ``system.json`` vs runtime graph.

A soundness checker v1, shipped as an advisory ``topology`` metric in ``lf_eval``,
topics only, comparing the statically extracted :class:`~lf_eval.model.system.SystemModel`
(from ``system.json``) against a runtime-observed pub/sub graph, reconciled via the
model's existing ``canonical_id``/``aliases`` scheme. This catches exactly the
"silent missing edge" failure class named in an external review: helpers/macros/dynamic topic
names that make the analyzer under-report a real pub/sub connection.

Static side
-----------
Node -> topic pub/sub edges come straight from ``SystemModel.nodes[*].publishes`` /
``.subscribes`` (populated from ``system.json``'s per-node ``publishers``/
``subscribers`` lists, see :func:`lf_eval.model.system.parse_system`).

Runtime side
------------
``Timeline`` (``lf_eval/model/timeline.py``) does not retain node-level publisher
topic data at all (publishers have no callback to attribute events to, so
``Timeline.callbacks`` only ever holds timer/subscription entries), and its
subscription-side ``topic_invocations`` is activity-gated — a subscription that
was created but never delivered a message during the trace window would falsely
read as "not present at runtime" from ``Timeline`` alone. Both would corrupt the
"missing from model" / "unobserved at runtime" findings this checker exists to
produce.

So the runtime side is built here from a **fresh, filtered read of the same
trace** (``rcl_node_init`` / ``rcl_publisher_init`` / ``rcl_subscription_init``),
mirroring the handle-chain approach in :mod:`lf_eval.model.timeline` and the
"re-derive from raw events" pattern already used by
:func:`lf_eval.metrics.drops.collect_drop_counts` — these are *init* events (they
fire once per publisher/subscription regardless of whether any message ever
flowed), which is what makes them the right primitive for a topology diff as
opposed to the activity-based data ``Timeline`` already exposes for other
metrics.

:func:`collect_runtime_topology` takes a bare event stream and has no dependency
on a real run directory, so it is unit-testable exactly like
``collect_drop_counts`` (:mod:`tests.metrics.test_topology`). :meth:`TopologyMetric.compute`
additionally know how to source that stream for a real :class:`~lf_eval.model.run.Run`
by re-reading ``run.path``'s trace; a run built via
:func:`~lf_eval.model.run.make_run` (as every metric test does, with no real
``tracing/`` directory on disk) degrades to a per-run "runtime side unavailable"
note rather than raising — see :data:`OWED note <_RUNTIME_UNAVAILABLE_NOTE>`
below. A run from :func:`~lf_eval.model.run.load_run` already carries
``run.runtime_topology``, collected from the single decode that also feeds the
timeline, so this metric does not decode the CTF a second time.
Only a run without that field falls
back to reading ``run.path``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

from lf_eval.metrics.registry import Metric, MetricResult, register
from lf_eval.model.chain import Chain
from lf_eval.model.run import Run
from lf_eval.model.system import SystemModel, canonical_key

#: Events the runtime side needs: node identity plus pub/sub *init* (not
#: activity) events, so a topic that never carried traffic during the trace
#: window still counts as "observed at runtime".
TOPOLOGY_EVENTS = frozenset(
    {"rcl_node_init", "rcl_publisher_init", "rcl_subscription_init"}
)

#: Topics that are ROS/rclcpp infrastructure rather than application data flow.
#: Excluded by default from "runtime edge missing from model" findings only —
#: an infrastructure topic that *is* declared in the model is still compared
#: normally. Module constant (per plan item 1) plus an option
#: (``include_infra_topics=True``) to opt back in.
DEFAULT_EXCLUDED_TOPICS = frozenset(
    {"/parameter_events", "/rosout", "/tf", "/tf_static"}
)

#: Runtime node-name prefixes belonging to tooling rather than the application.
#: ``ros2 launch`` spawns its own lifecycle-manager node named
#: ``launch_ros_<pid>``, which subscribes to the transition-event topics of the
#: lifecycle nodes it manages. It is not in ``system.json`` because it was never
#: meant to be, so both its edges and its unresolvable identity are noise rather
#: than findings. Matched on the trace spelling before node
#: resolution is attempted.
DEFAULT_EXCLUDED_NODE_PREFIXES = ("launch_ros_",)


def _is_orchestrator_node(node_name: str) -> bool:
    """Whether a runtime node name belongs to launch/tooling, not the system."""
    bare = node_name.strip().lstrip("/")
    return any(bare.startswith(p) for p in DEFAULT_EXCLUDED_NODE_PREFIXES)


#: system.json schema versions this metric knows how to read. Anything else is
#: reported as a note rather than silently trusted or crashed on (design
#: decision 4 note; see also the schema_version fix in model/system.py).
KNOWN_SCHEMA_VERSIONS = frozenset({3})

_RUNTIME_UNAVAILABLE_NOTE = (
    "runtime side unavailable for this run: {reason} — the topology metric "
    "re-reads {path}'s trace directly for rcl_publisher_init/"
    "rcl_subscription_init init events (Timeline does not retain node-level "
    "publisher data); a run built without a real tracing/ directory on disk "
    "(e.g. make_run in tests) has no runtime side to compare"
)


def _normalise_topic(topic: str) -> str:
    """Slash-tolerant topic key: compare ``"a/b"`` and ``"/a/b"`` as equal."""
    return topic.strip().lstrip("/")


def _display_topic(topic: str) -> str:
    """Canonical display form for a topic name: always leading-slash.

    Findings report this form regardless of whether the static model or the
    runtime trace happened to supply the (possibly slash-less) original
    string, so ``missing_from_model`` and ``unobserved_in_runtime`` entries
    are directly comparable to each other.
    """
    return "/" + _normalise_topic(topic)


@dataclass
class RuntimeTopology:
    """Node -> topic pub/sub sets observed at runtime for one trace."""

    #: node_name (trace spelling) -> topics it published to.
    publishes: dict[str, set[str]] = field(default_factory=dict)
    #: node_name (trace spelling) -> topics it subscribed to.
    subscribes: dict[str, set[str]] = field(default_factory=dict)

    @property
    def nodes(self) -> set[str]:
        return set(self.publishes) | set(self.subscribes)

    @property
    def is_empty(self) -> bool:
        return not self.publishes and not self.subscribes


def collect_runtime_topology(events: Iterable) -> RuntimeTopology:
    """Build a :class:`RuntimeTopology` from a raw ``ros2:*`` event stream.

    Consumes ``rcl_node_init`` (handle -> node name) plus ``rcl_publisher_init``
    / ``rcl_subscription_init`` (handle -> node handle + topic name), the same
    handle-chain shape :mod:`lf_eval.model.timeline` uses. Node identity is kept
    as the literal trace spelling (``rcl_node_init.node_name``); reconciliation
    against a :class:`~lf_eval.model.system.SystemModel` happens in the caller
    via :meth:`~lf_eval.model.system.SystemModel.resolve_node`, exactly as
    :mod:`lf_eval.model.timeline` defers canonicalisation to its ``resolver``
    argument rather than baking model knowledge into event parsing.
    """
    # Handles are addresses, unique only within one process: key by vpid too.
    node_names: dict[tuple, str] = {}
    topo = RuntimeTopology()

    # Two-pass-free: rcl_node_init always precedes rcl_publisher_init /
    # rcl_subscription_init for the same node in the real tracepoint order
    # (node construction happens before its publishers/subscriptions), but a
    # trace started mid-run could still see a pub/sub init with no matching
    # node_init. Those are kept keyed on a synthetic "handle:<n>" identity
    # rather than silently dropped, so an unattributable edge is still visible
    # in the runtime totals instead of vanishing.
    pending: list[tuple[str, tuple, str]] = []  # (kind, (pid, node_handle), topic)

    for event in events:
        name = event.name
        if name == "rcl_node_init":
            handle = event.get("node_handle")
            node_name = event.get("node_name")
            if isinstance(handle, int) and isinstance(node_name, str):
                node_names[(event.pid, handle)] = node_name
        elif name == "rcl_publisher_init":
            handle = event.get("node_handle")
            topic = event.get("topic_name")
            if isinstance(handle, int) and isinstance(topic, str) and topic:
                pending.append(("pub", (event.pid, handle), topic))
        elif name == "rcl_subscription_init":
            handle = event.get("node_handle")
            topic = event.get("topic_name")
            if isinstance(handle, int) and isinstance(topic, str) and topic:
                pending.append(("sub", (event.pid, handle), topic))

    for kind, node_handle, topic in pending:
        node = node_names.get(node_handle, f"<unresolved-handle-{node_handle[1]:#x}>")
        bucket = topo.publishes if kind == "pub" else topo.subscribes
        bucket.setdefault(node, set()).add(topic)

    return topo


@dataclass
class TopologyDiff:
    """Result of reconciling one runtime topology against a static model."""

    #: (canonical_node_id_or_display, topic, direction) tuples observed at
    #: runtime but absent from the static model. The dangerous class: a real
    #: connection the analyzer never extracted.
    missing_from_model: list[dict[str, str]] = field(default_factory=list)
    #: static edges never observed at runtime (node/topic never traced, or the
    #: run never exercised that path).
    unobserved_in_runtime: list[dict[str, str]] = field(default_factory=list)
    #: runtime node names that could not be resolved against the model at all.
    unmatched_nodes: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "missing_from_model": list(self.missing_from_model),
            "unobserved_in_runtime": list(self.unobserved_in_runtime),
            "unmatched_nodes": list(self.unmatched_nodes),
            "counts": {
                "missing_from_model": len(self.missing_from_model),
                "unobserved_in_runtime": len(self.unobserved_in_runtime),
                "unmatched_nodes": len(self.unmatched_nodes),
            },
        }


def diff_topology(
    runtime: RuntimeTopology,
    system: SystemModel,
    excluded_topics: frozenset[str] = DEFAULT_EXCLUDED_TOPICS,
) -> TopologyDiff:
    """Reconcile a runtime topology against a static :class:`SystemModel`.

    Node matching uses :meth:`SystemModel.resolve_node` (the ``canonical_id``/
    ``aliases`` scheme already built for the ``e2e_latency``/``drops`` metrics).
    Topic matching is slash-tolerant (``"a/b"`` == ``"/a/b"``), consistent with
    how :mod:`lf_eval.metrics.drops` normalises topic keys.

    Infrastructure topics (:data:`DEFAULT_EXCLUDED_TOPICS`) are excluded only
    from the ``missing_from_model`` finding — a runtime-only ``/rosout``
    publisher is expected on every real node and would otherwise swamp the
    dangerous-class findings with noise the static extractor never claimed to
    capture; they still count normally as declared/unobserved when the model
    *does* declare them.
    """
    diff = TopologyDiff()
    excluded = {_normalise_topic(t) for t in excluded_topics}

    # ── static side, keyed by canonical node id ─────────────────────────
    static_pub: dict[str, set[str]] = {}
    static_sub: dict[str, set[str]] = {}
    for node in system.nodes.values():
        # Derived infrastructure edges (action status/feedback endpoints,
        # lifecycle transition events) count as declared for the
        # missing_from_model direction — see
        # lf_eval.model.system.derive_infrastructure_edges.
        static_pub[node.canonical_id] = {
            _normalise_topic(t)
            for t in tuple(node.publishes) + tuple(node.infra_publishes)
            if t
        }
        static_sub[node.canonical_id] = {
            _normalise_topic(t)
            for t in tuple(node.subscribes) + tuple(node.infra_subscribes)
            if t
        }

    # ── runtime side, resolved to canonical ids where possible ──────────
    seen_unmatched: set[str] = set()

    def _resolved_runtime(
        buckets: dict[str, set[str]], direction: str, static_side: dict[str, set[str]]
    ) -> None:
        for raw_node, topics in buckets.items():
            if _is_orchestrator_node(raw_node):
                # launch/tooling node: neither its edges nor its unresolvable
                # identity are findings about the application.
                continue
            resolved = system.resolve_node(raw_node)
            if resolved is None:
                seen_unmatched.add(raw_node)
                cid = None
            else:
                cid = resolved.canonical_id
            declared = static_side.get(cid, set()) if cid is not None else set()
            for topic in sorted(topics):
                norm = _normalise_topic(topic)
                if norm not in declared:
                    if norm in excluded:
                        continue
                    diff.missing_from_model.append(
                        {
                            "node": resolved.display_name if resolved else raw_node,
                            "topic": _display_topic(topic),
                            "direction": direction,
                        }
                    )

    _resolved_runtime(runtime.publishes, "publish", static_pub)
    _resolved_runtime(runtime.subscribes, "subscribe", static_sub)

    # ── static edges never observed at runtime ───────────────────────────
    runtime_pub_by_cid: dict[str, set[str]] = {}
    runtime_sub_by_cid: dict[str, set[str]] = {}
    for raw_node, topics in runtime.publishes.items():
        resolved = system.resolve_node(raw_node)
        if resolved is not None:
            runtime_pub_by_cid.setdefault(resolved.canonical_id, set()).update(
                _normalise_topic(t) for t in topics
            )
    for raw_node, topics in runtime.subscribes.items():
        resolved = system.resolve_node(raw_node)
        if resolved is not None:
            runtime_sub_by_cid.setdefault(resolved.canonical_id, set()).update(
                _normalise_topic(t) for t in topics
            )

    # The unobserved direction deliberately uses only *explicitly declared*
    # edges: a derived infrastructure edge that no run happened to exercise is
    # not evidence of an extraction gap, and reporting it would just trade
    # missing_from_model noise for unobserved_in_runtime noise.
    declared_pub = {
        node.canonical_id: {_normalise_topic(t) for t in node.publishes if t}
        for node in system.nodes.values()
    }
    declared_sub = {
        node.canonical_id: {_normalise_topic(t) for t in node.subscribes if t}
        for node in system.nodes.values()
    }

    for cid, topics in declared_pub.items():
        seen = runtime_pub_by_cid.get(cid, set())
        for topic in sorted(topics - seen):
            diff.unobserved_in_runtime.append(
                {
                    "node": system.display_of(cid),
                    "topic": _display_topic(topic),
                    "direction": "publish",
                }
            )
    for cid, topics in declared_sub.items():
        seen = runtime_sub_by_cid.get(cid, set())
        for topic in sorted(topics - seen):
            diff.unobserved_in_runtime.append(
                {
                    "node": system.display_of(cid),
                    "topic": _display_topic(topic),
                    "direction": "subscribe",
                }
            )

    diff.unmatched_nodes = sorted(seen_unmatched)
    return diff


@register("topology")
class TopologyMetric(Metric):
    """Soundness/topology-diff: static ``system.json`` vs runtime pub/sub graph."""

    summary = "Static vs runtime pub/sub topology diff (soundness checker)"

    def compute(
        self,
        runs: Sequence[Run],
        system: Optional[SystemModel] = None,
        chains: Sequence[Chain] = (),
        **options: Any,
    ) -> MetricResult:
        if system is None:
            return MetricResult(
                name=self.name,
                available=False,
                note=(
                    "no system model given — pass --system so the static side "
                    "of the diff (system.json's declared pub/sub edges) can be "
                    "compared against the runtime trace"
                ),
            )

        schema_version = getattr(system, "schema_version", None)
        schema_note = ""
        if schema_version is not None and schema_version not in KNOWN_SCHEMA_VERSIONS:
            schema_note = (
                f"system.json reports schema_version={schema_version!r}, which "
                f"this metric has not been validated against (known: "
                f"{sorted(KNOWN_SCHEMA_VERSIONS)}); findings below may be "
                "incomplete or spurious rather than crashing outright"
            )

        fail_on_missing_edge = bool(options.get("fail_on_missing_edge", False))
        include_infra_topics = bool(options.get("include_infra_topics", False))
        excluded_topics = (
            frozenset() if include_infra_topics else DEFAULT_EXCLUDED_TOPICS
        )

        per_run: dict[str, Any] = {}
        summary_rows: list[dict[str, Any]] = []
        any_available = False
        any_missing = False

        for run in runs:
            runtime = self._runtime_topology_for(run)
            if runtime is None or runtime.is_empty:
                per_run[run.name] = {
                    "mode": run.mode,
                    "note": _RUNTIME_UNAVAILABLE_NOTE.format(
                        reason="no pub/sub init events could be read",
                        path=run.path,
                    ),
                }
                continue

            any_available = True
            diff = diff_topology(runtime, system, excluded_topics=excluded_topics)
            entry = diff.to_dict()
            if schema_note:
                entry["schema_note"] = schema_note
            per_run[run.name] = {"mode": run.mode, **entry}

            if diff.missing_from_model:
                any_missing = True

            summary_rows.append(
                {
                    "run": run.name,
                    "mode": run.mode,
                    "missing_from_model": len(diff.missing_from_model),
                    "unobserved_in_runtime": len(diff.unobserved_in_runtime),
                    "unmatched_nodes": len(diff.unmatched_nodes),
                }
            )

        if not any_available:
            return MetricResult(
                name=self.name,
                data={"per_run": per_run},
                available=False,
                note=(
                    "no runtime pub/sub data collected for any run — the trace "
                    "must include rcl_node_init/rcl_publisher_init/"
                    "rcl_subscription_init (docker/trace.sh enables all ros2:* "
                    "events, so this usually means an empty/filtered trace or "
                    "a run with no on-disk tracing/ directory, e.g. a "
                    "synthetic test run)"
                ),
            )

        data: dict[str, Any] = {"per_run": per_run}
        note = schema_note
        if fail_on_missing_edge and any_missing:
            # lf_eval metrics have no CLI-level pass/fail exit-code channel
            # today (every metric is advisory/descriptive; see
            # lf_eval/metrics/registry.py's module docstring). Without CLI
            # exit-code wiring, the only honest signal
            # this metric can give a caller that wants CI-blocking behaviour
            # is this in-band flag — `lf-eval` itself still exits 0.
            data["failed"] = True
            fail_note = (
                "fail_on_missing_edge=True and at least one run has a "
                "missing_from_model finding — no CLI exit-code channel exists "
                "yet for metrics (OWED to plan Slice 6: soundness runner + "
                "wiring); this flag currently only sets data['failed'] and "
                "this note"
            )
            note = f"{note} {fail_note}".strip() if note else fail_note

        return MetricResult(
            name=self.name,
            data=data,
            summary_rows=summary_rows,
            note=note,
        )

    @staticmethod
    def _runtime_topology_for(run: Run) -> Optional[RuntimeTopology]:
        """Best-effort runtime topology for one run.

        Uses ``run.runtime_topology`` when :func:`~lf_eval.model.run.load_run`
        already collected it from its single decode. Otherwise re-reads
        ``run.path``'s trace filtered to :data:`TOPOLOGY_EVENTS` (see the module
        docstring for why ``Timeline`` alone cannot supply the publisher
        side). Returns ``None`` (never raises) when the run has no
        readable trace on disk, which is the normal case for a
        :func:`~lf_eval.model.run.make_run`-built synthetic run in tests.
        """
        if run.runtime_topology is not None:
            return run.runtime_topology
        try:
            from lf_eval.reader import BackendUnavailableError, read_trace
        except ImportError:  # pragma: no cover - reader always ships with lf_eval
            return None

        try:
            events = list(
                read_trace(run.path, event_names=set(TOPOLOGY_EVENTS))
            )
        except (FileNotFoundError, BackendUnavailableError, OSError, RuntimeError):
            return None

        return collect_runtime_topology(events)

    def plot(self, result: MetricResult, outdir: Path) -> list[Path]:
        # No plot for v1 — the finding lists are the primary artifact; a
        # future iteration could render a graph diff (nodes/edges highlighted
        # by finding class). Not part of this slice's scope.
        return []
