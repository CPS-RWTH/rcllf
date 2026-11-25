# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Message-exact end-to-end latency per declared chain.

Every activation of a chain's sink is walked backwards, message by message, to
the source activation whose data it carries. The latency of that sample is
the sink activation's end minus the source activation's start (physical), and
for ``lfc`` also the sink's tag minus the source's tag (logical).

One hop of the walk, at node ``n[i]`` with activation ``a``, finds the
activation of ``n[i-1]`` that produced the data ``a`` used. The rule is
:func:`upstream_of`, shared with the per-hop pairing of
:mod:`lf_eval.metrics.e2e_latency`, and each hop ends in one of these outcomes:

``paired``
    ``a`` consumed a message of ``n[i-1]``: that message's publisher. Or ``a``
    is a timer, or consumed another input (a gated or state hop): the
    publisher of the newest input of ``n[i]`` from ``n[i-1]`` that completed
    before ``a`` began. This is data age (last input to output), the usual
    cause-effect metric.
``unresolved``
    The data ``a`` used may have come from ``n[i-1]`` but the trace does not
    name it: ``a`` (or, for a state hop, the newest input before it) consumed a
    message whose publication is not in the trace on a channel ``n[i-1]``
    publishes, or a subscription activation has no attributed message, or the
    newest input from ``n[i-1]`` was still running when ``a`` began. An older
    input is never used instead, since that would attribute the sample to the
    wrong source activation. The uncertainty of a state hop propagates the
    same way: a newer input of unknown origin hides every older one.
``startup``
    ``a`` is a timer (or a state hop) that began before ``n[i]`` received any
    input from ``n[i-1]``. Its output carries no data of ``n[i-1]``, which is
    a startup sample, not missing evidence. It is counted, never silent.
``not_chain_data``
    A callback of a subscription the system model does not declare as an
    input of its node (:func:`foreign_input`), such as rclcpp's
    ``/parameter_events``. A sink activation of that kind is no
    chain activation and is counted only in ``not_chain_data`` (per topic in
    ``not_chain_data_topics``). An activation further up the chain triggered
    by such a channel takes the state hop, like one triggered by another
    node's data. Whether a channel is foreign is decided by ``system.json``
    alone, never by what the trace recorded: a declared input whose
    publications are all missing from the trace is chain data with unknown
    origin, so it is ``unresolved``. An lfc
    ``CORR_SUB`` dispatch serves a manifest edge, a declared input by
    construction, so lfc excludes none.
``not_traceable``
    A hop that would be ``unresolved`` only because the data it needed came
    over an input the trace gives no message identity at all:
    a subscription served through rclcpp intra-process delivery, as in an
    RSLCPP run that loads every node into one process. Such a delivery
    bypasses ``rmw_publish`` and ``rmw_take``, so no message key exists on
    either side and the walk cannot pair the hop by construction. That is
    missing instrumentation, not a lost message, so it is counted apart from
    ``unresolved`` (per hop, with :data:`NOT_TRACEABLE_REASON`). The evidence
    is positive and per activation: its own ``callback_start`` carries
    ``is_intra_process=1`` (:func:`lacks_identity`, :func:`no_identity_inputs`).
    Only the stock walk applies it.

A channel is the topic of a stock message key and the publisher gid of an
lfc one, the first element of every message key.

Message identity comes from the trace, never from timestamps alone:

``stock``
    ``rmw_publish.timestamp`` is the source timestamp of the sample and
    ``rmw_take.source_timestamp`` the one of the taken sample. A publish is
    owned by the callback running on the same thread (``vpid``, ``vtid``) at
    that moment, and a successful take by the next callback that starts on the
    same thread.
``lfc``
    A ``CORR_SUB`` dispatch names the publication it consumes by (gid,
    ordinal), and the ``publication`` event names the dispatch that made it.

The lfc bound, from the declared configuration only, checked per sample: a
sample whose walk took only direct hops has logical latency exactly the sum of
the declared delays of its edges (``delay_ns`` of each admitted edge in the
run's trace manifest). A sample whose walk took a gated or state hop
sampled the source's stream, so its logical latency is at most that sum plus
the source timer's period (a newer source release at a tag no later than the
sink's tag would have reached the sink).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional

from lf_eval.model.system import canonical_key


@dataclass
class Activation:
    """One callback run (stock) or reaction dispatch (lfc) of a chain node."""

    node: str
    start: int
    end: int
    timer: bool
    consumed: Optional[tuple] = None
    tag: Optional[int] = None
    #: channel a subscription listens on, when known (the stock topic). Lets a
    #: subscription with no attributed message be told apart from chain data.
    channel: Optional[Any] = None
    #: the trace object this activation was built from (a timeline invocation)
    ref: Any = field(default=None, repr=False, compare=False)


@dataclass
class Graph:
    """Activations per node and the producer of every traced message."""

    by_node: dict[str, list[Activation]] = field(default_factory=dict)
    producer: dict[tuple, Activation] = field(default_factory=dict)
    #: every completed callback (stock) or dispatch that ran user code (lfc),
    #: chain node or not, as ``(start, wall ns)``, for the callback wall time column
    busy: list[tuple[int, int]] = field(default_factory=list)
    #: node -> channels it published on in the trace, keys withdrawn as
    #: duplicates included (a channel of the node stays its channel), plus the
    #: topics it declared a publisher for (stock, ``Timeline.declared_topics``)
    published: dict[str, set] = field(default_factory=dict)
    #: node -> longest activation, bounds the backward scan of a state hop
    longest: dict[str, int] = field(default_factory=dict)
    #: node -> the input topics the system model declares for it
    #: (:func:`declared_inputs`). A node absent here declares nothing, and
    #: then no callback of it is excluded as foreign.
    inputs: dict[str, frozenset] = field(default_factory=dict)
    #: ``(node, topic)`` subscriptions rclcpp can deliver intra-process
    #: (:func:`no_identity_inputs`, stock only, empty otherwise)
    no_identity: frozenset = frozenset()
    #: start timestamps of callbacks whose ``callback_start`` said
    #: ``is_intra_process=1`` (:meth:`_IdentityScan.intra_starts`, stock only)
    intra_starts: frozenset = frozenset()

    def add(self, act: Activation) -> None:
        self.by_node.setdefault(act.node, []).append(act)

    def add_publish(self, act: Activation, key: tuple, unique: bool = True) -> None:
        """Record that *act* published *key*; ``unique`` false withholds the producer."""
        self.published.setdefault(act.node, set()).add(key[0])
        if unique:
            self.producer[key] = act

    def sort(self) -> None:
        for node, acts in self.by_node.items():
            acts.sort(key=lambda a: a.start)
            self.longest[node] = max((a.end - a.start for a in acts), default=0)


def parse_chain(spec: str) -> list[str]:
    nodes = [p.strip() for p in spec.split("->") if p.strip()]
    if len(nodes) < 2:
        raise ValueError(f"chain {spec!r} needs at least two nodes")
    return nodes


def _node_resolver(system: dict):
    """Trace or manifest node name -> the system.json node key (canonical match)."""
    keys = {canonical_key(k): k for k in (system.get("nodes") or {})}
    for k, node in (system.get("nodes") or {}).items():
        for alias in (node.get("node_name"), node.get("name")):
            if isinstance(alias, str):
                keys.setdefault(canonical_key(alias), k)

    def resolve(name: Optional[str]) -> Optional[str]:
        return keys.get(canonical_key(name)) if name else None
    return resolve


def declared_inputs(system: dict) -> dict[str, frozenset]:
    """node key -> the resolved topics it subscribes, from ``communication.topics``.

    A trace carries fully qualified endpoint names (``rcl_subscription_init``
    ``topic_name``), while a node's own ``subscribers`` list keeps the name as
    written in the source: relative (``self_count``), private (``~/x``) or
    before a launch remap. Comparing those raw names excluded a declared
    relative input as ``not_chain_data``.
    ``communication.topics`` holds every endpoint after colcon_lf resolved its
    namespace and remaps (``comm_analysis/topic_mapper.resolve_name``), listed by
    node key, so that is the identity matched here.

    A node is left out (its inputs unknown, nothing of it ever foreign) when
    the model cannot name all its inputs: a subscriber with no static topic
    name (``topic_dynamic`` or an empty name, which topic_mapper skips), a
    node listed under the resolved topics fewer times than it has subscribers
    (one endpoint went unresolved), or a system.json without the
    ``communication`` section. Guessing a name there would turn a real input
    into excluded data. topic_mapper.build_communication lists a node once per
    named subscriber (comm_analysis/topic_mapper.py:147-151), also when a remap
    puts two of its subscribers on one topic, so the count per node equals its
    subscriber count exactly when every endpoint resolved. Accepting a node on
    any resolved subscriber took a partial set as complete and excluded the
    callbacks of its unresolved input (a review finding).
    """
    topics = (system.get("communication") or {}).get("topics")
    if not isinstance(topics, list):
        return {}
    resolved: dict[str, set] = {}
    endpoints: dict[str, int] = {}
    for t in topics:
        if not isinstance(t, dict) or not t.get("topic"):
            continue
        for sub in t.get("subscribers") or []:
            resolved.setdefault(str(sub), set()).add(str(t["topic"]))
            endpoints[str(sub)] = endpoints.get(str(sub), 0) + 1
    out: dict[str, frozenset] = {}
    for k, node in (system.get("nodes") or {}).items():
        subs = node.get("subscribers")
        if not isinstance(subs, list):
            continue
        named = all(isinstance(s, dict) and str(s.get("topic") or "").strip()
                    and not s.get("topic_dynamic") for s in subs)
        if not named or endpoints.get(k, 0) != len(subs):
            continue
        out[k] = frozenset(resolved.get(k, ()))
    return out


def foreign_input(graph: Graph, act: Activation) -> bool:
    """Is *act* a callback of a subscription the system model does not declare?

    Every rclcpp node also subscribes ``/parameter_events``, and a
    sink's callback on it is no chain output, so counting it as an unresolved
    sample of the last hop inflates that counter. The test is the callback's own
    subscription topic against the node's declared inputs (``system.json``),
    so it does not depend on what the trace happened to record: a declared input
    whose publications are all missing from the trace is still chain data, and
    its samples are unresolved. A node that declares no inputs, a timer, an lfc
    dispatch and a callback whose topic is unknown (a subscription traced
    without a topic name) are never foreign.
    """
    declared = graph.inputs.get(act.node)
    return (not act.timer and isinstance(act.channel, str) and declared is not None
            and act.channel not in declared)


def stock_graph(events: Iterable, system: dict) -> Graph:
    """Activations and message producers from ``ros2:*`` events.

    Attribution is :func:`lf_eval.model.timeline.build_timeline` (process
    scoped handles, a take owned by the next callback of its subscription on
    the thread, ambiguous takes and duplicate keys refused), so this walk and
    the ``e2e_latency`` metric see the same activations and messages.
    """
    from lf_eval.model.timeline import build_timeline

    resolve = _node_resolver(system)
    aliases = set()
    for k, node in (system.get("nodes") or {}).items():
        aliases.add(k)
        aliases.update(a for a in (node.get("node_name"), node.get("name")) if isinstance(a, str))
    scan = _IdentityScan()
    timeline = build_timeline(scan.feed(events), node_filter=aliases, resolver=resolve)
    graph = timeline_graph(timeline, declared_inputs(system))
    graph.busy = [(start, end - start) for start, end in timeline.completed]
    graph.no_identity = scan.no_identity(resolve)
    graph.intra_starts = scan.intra_starts()
    return graph


NOT_TRACEABLE_REASON = "intra-process: no message identity in this trace"


class _IdentityScan:
    """Collects, while the timeline consumes the stream, what :func:`lacks_identity` needs."""

    def __init__(self) -> None:
        self.node_names: dict[tuple, str] = {}      # (pid, node handle) -> node name
        self.subs: dict[tuple, tuple] = {}          # (pid, rcl sub handle) -> (node handle, topic)
        self.sub_of_obj: dict[tuple, int] = {}      # (pid, rclcpp subscription object) -> rcl sub handle
        self.ipb_objs: set[tuple] = set()           # (pid, subscription object) with an intra-process buffer
        #: callback_start timestamp -> whether every callback_start at it said is_intra_process=1
        self.start_intra: dict[int, bool] = {}

    def feed(self, events: Iterable):
        for event in events:
            name, pid = event.name, event.pid
            if name == "callback_start":
                ts = event.timestamp_ns
                self.start_intra[ts] = self.start_intra.get(ts, True) and event.get("is_intra_process") == 1
            elif name == "rcl_node_init":
                self.node_names[(pid, event.get("node_handle"))] = event.get("node_name")
            elif name == "rcl_subscription_init":
                self.subs[(pid, event.get("subscription_handle"))] = (
                    event.get("node_handle"), event.get("topic_name"))
            elif name == "rclcpp_subscription_init":
                self.sub_of_obj[(pid, event.get("subscription"))] = event.get("subscription_handle")
            elif name == "rclcpp_ipb_to_subscription":
                self.ipb_objs.add((pid, event.get("subscription")))
            yield event

    def no_identity(self, resolve) -> frozenset:
        return no_identity_inputs(self, resolve)

    def intra_starts(self) -> frozenset:
        """Start timestamps whose ``callback_start`` events all said ``is_intra_process=1``.

        An activation keeps no pid, thread or callback handle, only its start,
        so the flag is joined by that timestamp. When two callbacks start at
        the same nanosecond and one of them is not flagged (``0`` or the field
        absent), the timestamp is left out: a collision can only keep an
        activation ``unresolved``, never hide one.
        """
        return frozenset(ts for ts, intra in self.start_intra.items() if intra)


def no_identity_inputs(scan: _IdentityScan, resolve) -> frozenset:
    """``(node, topic)`` subscriptions rclcpp can deliver intra-process.

    This is the subscription-level half of the rule; the decision itself is
    per activation (:func:`lacks_identity`: its own ``callback_start`` says
    ``is_intra_process=1``). A subscription qualifies when its
    ``rclcpp_subscription_init`` object has an intra-process buffer
    (``rclcpp_ipb_to_subscription``). That guards the join the walk relies
    on: the timeline names an activation's input by (node, topic), and a
    callback flagged intra-process on a (node, topic) that has no buffer is a
    contradictory trace (a misattributed callback), so it stays
    ``unresolved``. The buffer comes from init events, recorded once at
    startup, not from per-message events that a disabled tracepoint or a lost
    event could remove.

    Two earlier conditions are gone: "a local
    publisher of the topic published intra-process" and "the rmw subscription
    never took a message". Both were inferences from per-message events, and
    the second from their absence: with ``rmw_take`` disabled or lost, an rmw
    callback whose publication is missing would have been hidden as
    ``not_traceable``. The per-activation flag replaces both, and it also
    keeps the rmw deliveries of a subscription served both ways
    ``unresolved`` while its intra-process ones are ``not_traceable``.

    The node and topic are keyed as the timeline keys a subscription
    activation (resolved node name, ``rcl_subscription_init.topic_name``). If
    one node has two subscriptions on a topic and only one qualifies, the pair
    is left out: the activation does not say which one it ran.
    """
    ipb_subs = {(pid, scan.sub_of_obj[(pid, obj)]) for pid, obj in scan.ipb_objs
                if (pid, obj) in scan.sub_of_obj}
    yes: set[tuple] = set()
    no: set[tuple] = set()
    for (pid, handle), (node_handle, topic) in scan.subs.items():
        trace_name = scan.node_names.get((pid, node_handle))
        if not isinstance(trace_name, str) or not isinstance(topic, str):
            continue
        key = ((resolve(trace_name) if resolve else None) or trace_name, topic)
        (yes if (pid, handle) in ipb_subs else no).add(key)
    return frozenset(yes - no)


def timeline_graph(timeline, inputs: Optional[dict[str, frozenset]] = None) -> Graph:
    """A :class:`Graph` of every node-attributed invocation of *timeline*.

    *inputs* are the declared inputs per node, keyed like the timeline's
    nodes (:func:`declared_inputs`). Without them no callback is foreign.
    """
    graph = Graph(inputs=dict(inputs or {}))
    for node, topics in timeline.declared_topics.items():
        graph.published.setdefault(node, set()).update(topics)
    acts: dict[int, Activation] = {}
    topic_of = {id(inv): topic for (_n, topic), invs in timeline.topic_invocations.items()
                for inv in invs}
    for timer, bucket in ((True, timeline.timer_invocations), (False, timeline.sub_invocations)):
        for node, invs in bucket.items():
            for inv in invs:
                act = Activation(node, inv.start_ns, inv.end_ns, timer,
                                 None if timer else inv.consumed,
                                 channel=None if timer else topic_of.get(id(inv)), ref=inv)
                acts[id(inv)] = act
                graph.add(act)
    for node, invs in timeline.invocations.items():
        for inv in invs:
            act = acts.get(id(inv))
            if act is None:
                continue
            for key in inv.published:
                prod = timeline.producer.get(key)
                graph.add_publish(act, key, unique=prod is not None and prod[1] is inv)
    graph.sort()
    return graph


def lfc_graph(events: Iterable, static: dict, system: dict) -> Graph:
    """Activations and message producers from ``rcllf:*`` events."""
    from lf_eval.semantic import schema

    resolve = _node_resolver(system)
    classes = {str(n["class_name"]): k for k, n in (system.get("nodes") or {}).items()}
    # Reaction ids restart in every federate binary: a federated
    # manifest keys them by federate, and each epoch's run_begin names the
    # federate executable (federate__<name>) whose rids it dispatches.
    rid_node: dict[tuple, str] = {}
    for r in static.get("rids") or []:
        node = classes.get(str(r["node"])) or resolve(str(r["node"]))
        if node:
            rid_node[(r.get("federate"), int(r["rid"]))] = node
    events = list(events)
    epoch_fed: dict[int, str] = {}
    if static.get("federated"):
        for ev in events:
            if ev.name == "run_begin":
                exe = str(ev.fields.get("executable", "")).rsplit("/", 1)[-1]
                epoch_fed[ev.epoch] = exe[len("federate__"):] if exe.startswith("federate__") else exe
    timer_kind, corr_kind = schema.ENTRY_KINDS["TIMER"], schema.ENTRY_KINDS["CORR_SUB"]
    begins: dict[tuple, Any] = {}
    acts: dict[tuple, Activation] = {}
    pubs: list[tuple[tuple, tuple]] = []
    graph = Graph()
    for ev in events:
        f = ev.fields
        if ev.name == "dispatch_begin":
            begins[(ev.epoch, int(f["dispatch_id"]))] = ev
        elif ev.name == "dispatch_end":
            key = (ev.epoch, int(f["dispatch_id"]))
            b = begins.pop(key, None)
            if b is None or not int(f.get("callback_ran", 1)):
                continue
            # the blocking join wait for the matched sample is not callback work
            graph.busy.append((b.ts, max(0, ev.ts - b.ts - int(f.get("join_wait_ns") or 0))))
            node = rid_node.get((epoch_fed.get(ev.epoch), int(b.fields["rid"])))
            kind = int(b.fields["entry_kind"])
            if node is None or kind not in (timer_kind, corr_kind):
                continue
            consumed = (b.gid(), int(b.fields["ordinal"])) if kind == corr_kind else None
            act = Activation(node, b.ts, ev.ts, kind == timer_kind, consumed, int(b.fields["tag_time"]))
            acts[key] = act
            graph.add(act)
        elif ev.name == "publication":
            pubs.append(((ev.epoch, int(f["dispatch_id"])), (ev.gid(), int(f["ordinal"]))))
    # A message named twice names no single publication: withdraw it.
    count: dict[tuple, int] = {}
    for _d, msg in pubs:
        count[msg] = count.get(msg, 0) + 1
    for dkey, msg in pubs:
        if dkey in acts:
            graph.add_publish(acts[dkey], msg, unique=count[msg] == 1)
    graph.sort()
    return graph


PAIRED, UNRESOLVED, STARTUP, NOT_CHAIN_DATA, OTHER_INPUT, NOT_TRACEABLE = (
    "paired", "unresolved", "startup", "not_chain_data", "other_input", "not_traceable")


def _input_from(graph: Graph, act: Activation, prev: str) -> tuple[str, Optional[Activation]]:
    """Where the input of subscription activation *act* came from, seen from *prev*.

    ``("foreign", None)``: a subscription the system model does not declare
    for the node (:func:`foreign_input`). ``("prev", src)``: a traced message
    of *prev*. ``("other", None)``: a traced message of another node, or an
    untraced one on a channel only other nodes publish. ``("unknown", None)``:
    maybe *prev*'s, that is an untraced message (or no attributed message) on a
    channel *prev* publishes, on a channel no node is seen publishing, or on an
    unknown channel. A channel nobody is seen publishing is never foreign by
    that fact: the trace may have lost every publication of it (a review
    finding).
    """
    if foreign_input(graph, act):
        return "foreign", None
    if act.consumed is not None:
        src = graph.producer.get(act.consumed)
        if src is not None:
            return ("prev", src) if src.node == prev else ("other", None)
        channel = act.consumed[0]
    else:
        channel = act.channel
        if channel is None:
            return "unknown", None
    if channel in graph.published.get(prev, ()):
        return "unknown", None
    if any(channel in chans for chans in graph.published.values()):
        return "other", None
    return "unknown", None


def upstream_of(graph: Graph, act: Activation, prev: str,
                state_hop_for_other_inputs: bool = True) -> tuple[str, Optional[Activation], bool]:
    """``(outcome, upstream activation, gated)`` of one hop, see the module docstring.

    *state_hop_for_other_inputs*: a subscription activation that consumed
    another node's data takes the state hop (the chain walk, data age) when
    true, and is ``other_input`` when false (a per-hop latency, where it is no
    activation of the hop at all).
    """
    outcome, up, gated, _deciders = _hop(graph, act, prev, state_hop_for_other_inputs)
    return outcome, up, gated


def _hop(graph: Graph, act: Activation, prev: str, state_hop_for_other_inputs: bool = True
         ) -> tuple[str, Optional[Activation], bool, tuple[Activation, ...]]:
    """:func:`upstream_of` plus, for ``unresolved``, the activations whose input decided it.

    Those are the activations whose message the walk needed named and could
    not: *act* itself on a direct hop; on a state hop every input still
    running when *act* began together with the newest finished input (if
    any), the two overlapping inputs, or the newest input of unknown origin.
    :func:`walk_chain` reads them to tell ``not_traceable`` apart from
    ``unresolved``.
    """
    if not act.timer:
        kind, src = _input_from(graph, act, prev)
        if kind == "prev":
            return PAIRED, src, False, ()
        if kind == "unknown":
            return UNRESOLVED, None, False, (act,)
        if not state_hop_for_other_inputs:
            return (OTHER_INPUT if kind == "other" else NOT_CHAIN_DATA), None, False, ()
    # State hop: the newest input of act.node from prev before act began. Any
    # input that may be prev's and is newer than the one found, or still
    # running when act began, leaves the hop unresolved.
    acts = graph.by_node.get(act.node, ())
    longest = graph.longest.get(act.node, 0)
    best: Optional[tuple[str, Optional[Activation], Activation]] = None
    # Inputs still running when act began. Each leaves the hop unresolved, but
    # does not end the scan: the newest finished input, which may have started
    # earlier, may be the one act read, so it decides the hop with them.
    # Finished inputs that began no later than the newest one and ran into it
    # (or contain it): which one wrote last is unknown. They do not end the
    # scan either, an older input may still be running.
    running: list[Activation] = []
    overlapping: list[Activation] = []
    idx = _bisect_start(acts, act.start)
    for cand in reversed(acts[:idx]):
        if best is not None and cand.start + longest < best[2].start:
            break  # no earlier candidate can end after the one found
        if cand is act or cand.timer:
            continue
        kind, src = _input_from(graph, cand, prev)
        if kind in ("other", "foreign"):
            continue
        if cand.end > act.start:
            running.append(cand)
        elif best is None:
            best = (kind, src, cand)
        elif cand.end > best[2].start:
            overlapping.append(cand)
    if overlapping:
        return UNRESOLVED, None, True, (*running, *overlapping, best[2])
    if running:
        finished = () if best is None else (best[2],)
        return UNRESOLVED, None, True, (*running, *finished)
    if best is None:
        return STARTUP, None, True, ()
    if best[0] == "unknown":
        return UNRESOLVED, None, True, (best[2],)
    return PAIRED, best[1], True, ()


def lacks_identity(graph: Graph, act: Activation) -> bool:
    """Is *act* a subscription delivery the trace gives no message identity?

    Positive evidence per activation: its own ``callback_start`` carries
    ``is_intra_process=1`` (:attr:`Graph.intra_starts`), so rclcpp ran it from
    the intra-process waitable, which has no ``rmw_take``. An activation with
    ``is_intra_process=0``, or a trace without the field, stays
    ``unresolved``; absence of an rmw take proves nothing, the tracepoint may
    be disabled or its events lost. Two further guards: the activation
    consumed no message (one that took an rmw message has identity, and a
    missing publication of it is a loss), and its input is a subscription
    with an intra-process buffer (:attr:`Graph.no_identity`,
    :func:`no_identity_inputs`).
    """
    return (not act.timer and act.consumed is None and act.start in graph.intra_starts
            and (act.node, act.channel) in graph.no_identity)


def _bisect_start(acts, start: int) -> int:
    """Index of the first activation of *acts* (sorted by start) starting at or after *start*."""
    lo, hi = 0, len(acts)
    while lo < hi:
        mid = (lo + hi) // 2
        if acts[mid].start < start:
            lo = mid + 1
        else:
            hi = mid
    return lo


def in_window(act: Activation, window: Optional[tuple[int, int]]) -> bool:
    """Whether *act* starts inside ``[lo, hi)``; every activation when *window* is None."""
    return window is None or window[0] <= act.start < window[1]


def walk_chain(graph: Graph, chain: list[str],
               sink_window: Optional[tuple[int, int]] = None) -> dict[str, Any]:
    """Backward walk from every sink activation; samples and per-outcome counts.

    *sink_window* ``[lo, hi)`` keeps only sink activations starting inside it.
    The walk itself reads the whole graph: a sample belongs to the window of
    its sink, and the message it carries may have been published before the
    window opened. Cutting the trace instead would leave that message without
    its publication and report a traced sample as unresolved.

    ``unresolved``, ``startup`` and ``not_traceable`` map ``"prev->node"``
    (``unresolved`` also ``source_not_timer``) to how many sink activations
    stopped there. A hop is ``not_traceable`` instead of ``unresolved`` when
    every activation whose input decided it (:func:`_hop`) lacks message
    identity (:func:`lacks_identity`); one of them with identity, such as an
    rmw input whose publication is missing, keeps it ``unresolved``.
    ``not_chain_data`` counts sink activations of an undeclared subscription
    (:func:`foreign_input`), ``not_chain_data_topics`` the same per topic.
    ``complete`` is true when every chain sink activation became a sample or
    a startup sample.
    """
    physical: list[int] = []
    logical: list[int] = []
    gated_flags: list[bool] = []
    unresolved: dict[str, int] = {}
    startup: dict[str, int] = {}
    not_traceable: dict[str, int] = {}
    not_chain: dict[str, int] = {}
    sinks = [s for s in graph.by_node.get(chain[-1], ()) if in_window(s, sink_window)]
    for sink in sinks:
        if foreign_input(graph, sink):
            not_chain[sink.channel] = not_chain.get(sink.channel, 0) + 1
            continue
        cur, gated = sink, False
        for i in range(len(chain) - 1, 0, -1):
            outcome, up, g, deciders = _hop(graph, cur, chain[i - 1])
            if outcome == UNRESOLVED and deciders and all(lacks_identity(graph, d) for d in deciders):
                outcome = NOT_TRACEABLE
            if outcome != PAIRED:
                hop = f"{chain[i - 1]}->{chain[i]}"
                if outcome == STARTUP:
                    startup[hop] = startup.get(hop, 0) + 1
                elif outcome == NOT_TRACEABLE:
                    not_traceable[hop] = not_traceable.get(hop, 0) + 1
                else:
                    unresolved[hop] = unresolved.get(hop, 0) + 1
                cur = None
                break
            gated |= g
            cur = up
        if cur is None:
            continue
        if not cur.timer:
            unresolved["source_not_timer"] = unresolved.get("source_not_timer", 0) + 1
            continue
        gated_flags.append(gated)
        physical.append(sink.end - cur.start)
        if sink.tag is not None and cur.tag is not None:
            logical.append(sink.tag - cur.tag)
    return {"sinks": len(sinks), "physical_ns": physical,
            "logical_ns": logical, "gated": gated_flags, "gated_samples": sum(gated_flags),
            "unresolved": unresolved, "startup": startup, "not_traceable": not_traceable,
            "not_chain_data": sum(not_chain.values()),
            "not_chain_data_topics": dict(sorted(not_chain.items())),
            "complete": not unresolved and not not_traceable}


def percentile(values: list[int], q: float) -> Optional[float]:
    """Nearest-rank percentile (no interpolation), ``None`` on no data."""
    if not values:
        return None
    s = sorted(values)
    return float(s[max(0, math.ceil(q / 100.0 * len(s)) - 1)])


def stats(values: list[int]) -> dict[str, Any]:
    return {"n": len(values), "p50_ns": percentile(values, 50), "p99_ns": percentile(values, 99),
            "max_ns": float(max(values)) if values else None}


def declared_bound(system: dict, chain: list[str], delays: Optional[dict] = None) -> dict[str, Any]:
    """Logical latency bounds of *chain* from the declared configuration.

    ``delays`` maps ``"src->dst"`` to the declared delay in ns of the admitted
    edge between them, or ``None`` when parallel admitted edges between the two
    nodes declare different delays (:func:`manifest_delays`). ``direct_ns`` is
    the exact latency of a sample with only direct hops, ``gated_ns`` the bound
    of a sample with a gated or state hop. Both are ``None`` when a hop has no
    admitted edge or an ambiguous one, or (``gated_ns``) the source has no
    single timer period.
    """
    nodes = system.get("nodes") or {}
    delays = delays or {}
    hops = [f"{a}->{b}" for a, b in zip(chain, chain[1:])]
    missing = [h for h in hops if h not in delays]
    ambiguous = [h for h in hops if h in delays and delays[h] is None]
    total = None if missing or ambiguous else sum(int(delays[h]) for h in hops)
    src_timers = [t for t in (nodes.get(chain[0]) or {}).get("timers") or [] if t.get("period_ms")]
    period = int(round(float(src_timers[0]["period_ms"]) * 1e6)) if len(src_timers) == 1 else None
    return {"sum_delays_ns": total, "hops_without_edge": missing, "ambiguous_hops": ambiguous,
            "source_period_ns": period, "direct_ns": total,
            "gated_ns": None if period is None or total is None else total + period}


def manifest_delays(static: dict) -> dict[str, Optional[int]]:
    """``"publisher node->subscriber node" -> delay_ns`` of the admitted edges.

    The walk does not record which edge a hop took, so a node pair with parallel
    admitted edges of different delays has no single delay: it maps to ``None``
    and the bound refuses that hop. Parallel edges with one delay keep it.
    """
    seen: dict[str, set[int]] = {}
    for e in static.get("edges") or []:
        if e.get("status") == "admitted":
            seen.setdefault(f"{e['publisher']['node']}->{e['subscriber']['node']}", set()).add(int(e["delay_ns"]))
    return {k: next(iter(v)) if len(v) == 1 else None for k, v in seen.items()}


def outside_bound(bound: dict[str, Any], logical: list[int], gated: list[bool]) -> list[tuple[int, bool]]:
    """Samples outside the bound: a direct one not exactly at it, a gated one past it."""
    out = []
    for v, g in zip(logical, gated):
        limit = bound["gated_ns"] if g else bound["direct_ns"]
        if limit is None or bound["direct_ns"] is None or v < bound["direct_ns"] or (v > limit if g else v != limit):
            out.append((v, g))
    return out


def in_callback(graph: Graph) -> dict[str, Any]:
    """Wall time inside callbacks over the traced span, in cores busy in callbacks.

    This is wall time, not CPU time: a callback that sleeps, blocks or is
    preempted still counts, and executor spin, DDS threads and the rcllf
    scheduler do not. It bounds CPU time in neither direction. The trace has no
    per-thread CPU counter, so CPU cost is the rusage measurement of the benchmark orchestrator.
    """
    if not graph.busy:
        return {"activations": 0, "busy_ns": 0, "span_ns": None, "cores": None}
    starts = [t for t, _ in graph.busy]
    ends = [t + d for t, d in graph.busy]
    busy, span = sum(d for _, d in graph.busy), max(ends) - min(starts)
    return {"activations": len(graph.busy), "busy_ns": busy, "span_ns": span,
            "cores": busy / span if span > 0 else None}


def chain_latency(run_dir: Path, chains: list[list[str]], mode: str) -> dict[str, Any]:
    """Latency samples of every chain of one run; ``mode`` is ``stock`` or ``lfc``."""
    run_dir = Path(run_dir)
    system = json.loads((run_dir / "system.json").read_text())
    static: dict = {}
    if mode == "lfc":
        from lf_eval.semantic.reader import read_run

        static = json.loads((run_dir / "rcllf_trace_manifest.json").read_text())
        events, _stats = read_run(run_dir)
        graph = lfc_graph(events, static, system)
    else:
        from lf_eval.reader.reader import read_trace

        graph = stock_graph(read_trace(run_dir), system)
    out: dict[str, Any] = {"run": run_dir.name, "mode": mode, "in_callback": in_callback(graph), "chains": {}}
    for chain in chains:
        w = walk_chain(graph, chain)
        row: dict[str, Any] = {"sinks": w["sinks"], "unresolved": w["unresolved"],
                               "startup": w["startup"], "not_chain_data": w["not_chain_data"],
                               "not_chain_data_topics": w["not_chain_data_topics"],
                               "gated_samples": w["gated_samples"],
                               "physical": stats(w["physical_ns"]), "physical_ns": w["physical_ns"]}
        if w["not_traceable"]:
            # only when present, so a run on the rmw path keeps its output byte for byte
            row["not_traceable"] = w["not_traceable"]
            row["not_traceable_reason"] = NOT_TRACEABLE_REASON
        if mode == "lfc":
            bound = declared_bound(system, chain, manifest_delays(static))
            logical = w["logical_ns"]
            outside = outside_bound(bound, logical, w["gated"])
            row.update({"logical": stats(logical), "bound": bound, "samples_outside_bound": len(outside),
                        "first_outside": {"logical_ns": outside[0][0], "gated": outside[0][1]} if outside else None,
                        "logical_ns": logical})
        out["chains"]["->".join(chain)] = row
    out["failures"] = latency_failures(out)
    return out


def latency_failures(result: dict[str, Any]) -> list[str]:
    """Why a run's latency result fails, empty when it passes.

    A declared chain without a single sample fails, and so does an ``lfc``
    chain with a sample outside its bound or without a bound (a hop with no
    admitted edge, or an ambiguous one). An unresolved sink activation fails
    the run too: its sample is missing evidence, and a statistic over the
    rest may hide exactly the case it lost. A ``not_traceable`` sink
    activation fails it for the same reason, under its own name so it does
    not read as a lost message. Startup samples (a tick before
    the first input of a hop) and sink activations on no chain channel are
    counted in the result and do not fail it.
    """
    out = []
    for name, c in result["chains"].items():
        if not c["physical"]["n"]:
            out.append(f"no_samples:{name}")
        unresolved = sum((c.get("unresolved") or {}).values())
        if unresolved:
            out.append(f"unresolved:{name}:{unresolved}")
        not_traceable = sum((c.get("not_traceable") or {}).values())
        if not_traceable:
            out.append(f"not_traceable:{name}:{not_traceable}")
        if result["mode"] != "lfc":
            continue
        if c["bound"]["direct_ns"] is None:
            out.append(f"no_bound:{name}:missing={c['bound']['hops_without_edge']}"
                       f":ambiguous={c['bound'].get('ambiguous_hops', [])}")
        elif c["samples_outside_bound"]:
            out.append(f"outside_bound:{name}:{c['samples_outside_bound']}")
    return out
