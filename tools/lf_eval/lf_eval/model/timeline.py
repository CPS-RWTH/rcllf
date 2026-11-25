# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Per-callback timelines built from trace events.

The central problem is **attribution**: ``callback_start``/``callback_end``
carry only an opaque ``callback`` pointer. Two independent resolution paths are
combined, which is what lets this module drop the predecessor script's
hardcoded ``APP_NODE_CLASSES`` allow-list:

1. **Handle chain (authoritative, needs no C++ symbol heuristics).**
   ``rcl_node_init`` gives ``node_handle -> node_name``. Timers link via
   ``rclcpp_timer_link_node`` (``timer_handle -> node_handle``) and
   ``rclcpp_timer_callback_added`` (``timer_handle -> callback``). Subscriptions
   link via ``rcl_subscription_init`` (``subscription_handle -> node_handle`` +
   ``topic_name``), ``rclcpp_subscription_init``
   (``subscription_handle -> subscription``) and
   ``rclcpp_subscription_callback_added`` (``subscription -> callback``).

2. **Symbol regex (fallback).** ``rclcpp_callback_register`` exposes the bound
   C++ symbol; ``(\\w+)::`` recovers the owning class when the handle chain is
   incomplete (e.g. traces started after node construction). Unlike the
   predecessor, the extracted name is *not* gated against a fixed allow-list —
   instead callers filter against the ``system.json`` node set, so any system
   works.

Timer vs. subscription is likewise decided by the handle chain first, falling
back to the predecessor's heuristic: a timer callback's bound symbol has no
``std::_Placeholder`` argument, a subscription callback's does.

Message identity
----------------
``rmw_publish.timestamp`` is the source timestamp of the published sample and
``rmw_take.source_timestamp`` the source timestamp of the taken one, so
``(topic, source timestamp)`` names one message on both sides. rclcpp's
executor takes a sample and runs the subscription callback on the same thread
right after, and a callback's publishes happen on its own thread between its
``callback_start`` and ``callback_end``. So every invocation records the
message it consumed (:attr:`Invocation.consumed`) and the messages it published
(:attr:`Invocation.published`), and :attr:`Timeline.producer` maps a message to
the invocation that published it. The attribution needs the ``vpid``/``vtid``
contexts and refuses rather than guesses: a take whose next callback on the
thread is not that subscription's callback, a second take on a thread before a
callback consumed the first, and a message key published twice leave the
message unattributed (counted in :attr:`Timeline.identity`). Intra-process
delivery bypasses ``rmw_publish``, so such edges carry no identity.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional

from lf_eval.model.system import canonical_key

# Extracts the owning class from a bound-callback symbol such as
# "std::_Bind<void (FrontLidarDriver::*(...))(...)>".
_CLASS_RE = re.compile(r"\((\w+)::")

# Events the timeline builder needs; used to pre-filter the reader stream.
REQUIRED_EVENTS = frozenset(
    {
        "rcl_node_init",
        "rcl_timer_init",
        "rclcpp_timer_link_node",
        "rclcpp_timer_callback_added",
        "rcl_subscription_init",
        "rclcpp_subscription_init",
        "rclcpp_subscription_callback_added",
        "rclcpp_callback_register",
        "callback_start",
        "callback_end",
        "rcl_publisher_init",
        "rclcpp_publish",
        "rcl_publish",
        "rmw_publish",
        "rmw_take",
        "rcl_take",
        "rclcpp_take",
    }
)

CallbackKind = str  # "timer" | "subscription" | "unknown"


@dataclass(frozen=True, slots=True)
class Invocation:
    """One matched ``callback_start``/``callback_end`` pair."""

    start_ns: int
    end_ns: int
    cpu_id: Optional[int] = None
    #: ``(topic, source timestamp)`` of the message this invocation took, when
    #: the trace attributes one to it (subscriptions only).
    consumed: Optional[tuple[str, int]] = None
    #: ``(topic, source timestamp)`` of every message it published via rmw.
    published: tuple[tuple[str, int], ...] = ()

    @property
    def duration_ns(self) -> int:
        return self.end_ns - self.start_ns

    @property
    def duration_us(self) -> float:
        return self.duration_ns / 1_000.0


@dataclass
class CallbackInfo:
    """What we know about one registered callback."""

    handle: int
    #: ``vpid`` of the process that registered it (``None`` without the context).
    pid: Optional[int] = None
    #: Node name exactly as the trace reported it (``rcl_node_init.node_name``).
    node: Optional[str] = None
    kind: CallbackKind = "unknown"
    topic: Optional[str] = None
    period_ns: Optional[int] = None
    symbol: Optional[str] = None
    #: Canonical id this callback was grouped under (see :func:`build_timeline`).
    canonical_id: Optional[str] = None


@dataclass
class Timeline:
    """Callback invocations from one trace, grouped by node.

    ``invocations`` merges every callback of a node (timer + subscription);
    ``timer_invocations`` and ``sub_invocations`` are the per-kind views.
    All lists are sorted by ``start_ns``.
    """

    #: ``(vpid, callback handle)`` -> what is known about that callback. The
    #: vpid is part of the key because two processes can reuse one address.
    callbacks: dict[tuple, CallbackInfo] = field(default_factory=dict)
    invocations: dict[str, list[Invocation]] = field(default_factory=dict)
    timer_invocations: dict[str, list[Invocation]] = field(default_factory=dict)
    sub_invocations: dict[str, list[Invocation]] = field(default_factory=dict)
    # (node, topic) -> invocations of that node's subscription to that topic
    topic_invocations: dict[tuple[str, str], list[Invocation]] = field(
        default_factory=dict
    )
    total_callback_starts: int = 0
    unmatched_starts: int = 0
    first_ns: Optional[int] = None
    last_ns: Optional[int] = None
    #: canonical id -> the node name the trace actually reported, for diagnostics
    #: when the two conventions differ (``publishernode`` vs ``publisher_node``).
    trace_names: dict[str, str] = field(default_factory=dict)
    #: message key -> (canonical node, invocation) that published it.
    producer: dict[tuple[str, int], tuple[str, Invocation]] = field(default_factory=dict)
    #: attribution counters, see "Message identity" in the module docstring.
    identity: dict[str, int] = field(default_factory=dict)
    #: ``(start_ns, end_ns)`` of every matched invocation, node resolved or not,
    #: in completion order (the callback wall time of :mod:`lf_eval.chain_latency`).
    completed: list[tuple[int, int]] = field(default_factory=list)
    #: canonical node -> topics it created a publisher for (``rcl_publisher_init``).
    #: A message on such a topic whose
    #: publish was not traced may still be that node's data.
    declared_topics: dict[str, set] = field(default_factory=dict)

    @property
    def nodes(self) -> list[str]:
        """Nodes that produced at least one invocation."""
        return sorted(self.invocations)

    @property
    def duration_ns(self) -> int:
        """Span between the first and last observed event."""
        if self.first_ns is None or self.last_ns is None:
            return 0
        return self.last_ns - self.first_ns

    def cpu_busy_ns(self, nodes: Optional[Iterable[str]] = None) -> int:
        """Total in-callback time, i.e. trace-derived CPU demand.

        Used by the ``cpu`` metric to cross-check ``/proc``-based sampling.
        Overlapping invocations of *different* nodes are summed (they genuinely
        run on different cores), so this is CPU-seconds, not wall-seconds.
        """
        selected = self.invocations if nodes is None else {
            n: self.invocations.get(n, []) for n in nodes
        }
        return sum(
            inv.duration_ns for invs in selected.values() for inv in invs
        )


def build_timeline(
    events: Iterable,
    node_filter: Optional[set[str]] = None,
    resolver: Optional[Callable[[str], Optional[str]]] = None,
) -> Timeline:
    """Build a :class:`Timeline` from a stream of :class:`TraceEvent` records.

    :param events: trace events, chronologically ordered.
    :param node_filter: when given, only keep callbacks whose resolved node name
        is in this set (matched via :func:`canonical_key`). Pass the
        ``system.json`` node identifiers to exclude ROS infrastructure nodes
        (``rosbag2_recorder``, parameter services, ``/rosout``).
    :param resolver: maps a trace node name to the canonical id the rest of the
        pipeline keys on — normally
        :meth:`~lf_eval.model.system.SystemModel.canonical_of`. Returning
        ``None`` means "not in the model", in which case the literal trace name
        is kept so unmodelled nodes still show up. Without a resolver the
        literal trace names are used throughout.

    Keying the timeline on canonical ids is what lets chains (derived from
    ``system.json``) pair against traces whose ``rcl_node_init.node_name``
    follows a different naming convention — ``publisher_node`` vs
    ``PublisherNode``. See :attr:`NodeSpec.canonical_id`.
    """
    timeline = Timeline()
    filter_keys = (
        {canonical_key(n) for n in node_filter} if node_filter is not None else None
    )

    # Every handle is an address inside one process, so two processes (two
    # federates, two component containers) can reuse one value for different
    # objects. Each registry is keyed by (vpid, handle); the vpid is None when
    # the trace has no process context, which is then one process.
    node_names: dict[tuple, str] = {}               # (pid, node_handle) -> node_name
    timer_node: dict[tuple, int] = {}               # (pid, timer_handle) -> node_handle
    timer_period: dict[tuple, int] = {}             # (pid, timer_handle) -> period ns
    timer_cb: dict[tuple, int] = {}                 # (pid, timer_handle) -> callback
    sub_node: dict[tuple, int] = {}                 # (pid, sub_handle) -> node_handle
    sub_topic: dict[tuple, str] = {}                # (pid, sub_handle) -> topic
    obj_to_handle: dict[tuple, int] = {}            # (pid, subscription) -> sub_handle
    sub_obj: dict[tuple, int] = {}                  # (pid, sub_handle) -> subscription
    sub_cb: dict[tuple, int] = {}                   # (pid, subscription) -> callback
    symbols: dict[tuple, str] = {}                  # (pid, callback) -> symbol

    # (pid, callback), or (pid, tid, callback) with thread context, ->
    # (start_ns, cpu_id) of the in-flight invocation. The thread tells apart two
    # invocations of one reentrant callback running at once on two threads.
    pending: dict[tuple, tuple[int, Optional[int]]] = {}
    raw: dict[tuple, list[Invocation]] = {}         # (pid, callback) -> invocations

    # Message identity.
    rmw_sub: dict[tuple, tuple[int, str]] = {}      # (pid, rmw sub) -> (sub_handle, topic)
    rmw_pub_topic: dict[tuple, str] = {}            # (pid, rmw pub) -> topic
    pub_decl: dict[tuple, set] = {}                 # (pid, node_handle) -> topics
    taken: dict[tuple, object] = {}                 # (pid, tid) -> (rmw sub, key) | _AMBIGUOUS
    # (pid, tid) -> [callback, consumed key, published keys] of the running callback
    running: dict[tuple, list] = {}
    # (topic, stamp) -> how often it was published, attributed to a callback or not
    published_keys: dict[tuple[str, int], int] = {}
    ident = timeline.identity
    for k in ("takes", "takes_attributed", "publishes_attributed", "duplicate_keys"):
        ident[k] = 0

    for event in events:
        name = event.name
        ts = event.timestamp_ns
        pid = event.pid
        if timeline.first_ns is None:
            timeline.first_ns = ts
        timeline.last_ns = ts

        if name == "rcl_node_init":
            handle = event.get("node_handle")
            node_name = event.get("node_name")
            if isinstance(handle, int) and isinstance(node_name, str):
                node_names[(pid, handle)] = node_name

        elif name == "rcl_timer_init":
            handle = event.get("timer_handle")
            period = event.get("period")
            if isinstance(handle, int) and isinstance(period, int):
                timer_period[(pid, handle)] = period

        elif name == "rclcpp_timer_link_node":
            handle = event.get("timer_handle")
            node_handle = event.get("node_handle")
            if isinstance(handle, int) and isinstance(node_handle, int):
                timer_node[(pid, handle)] = node_handle

        elif name == "rclcpp_timer_callback_added":
            handle = event.get("timer_handle")
            callback = event.get("callback")
            if isinstance(handle, int) and isinstance(callback, int):
                timer_cb[(pid, handle)] = callback

        elif name == "rcl_subscription_init":
            handle = event.get("subscription_handle")
            node_handle = event.get("node_handle")
            topic = event.get("topic_name")
            if isinstance(handle, int):
                if isinstance(node_handle, int):
                    sub_node[(pid, handle)] = node_handle
                if isinstance(topic, str):
                    sub_topic[(pid, handle)] = topic
                    rmw = event.get("rmw_subscription_handle")
                    if isinstance(rmw, int):
                        rmw_sub[(pid, rmw)] = (handle, topic)

        elif name == "rclcpp_subscription_init":
            handle = event.get("subscription_handle")
            obj = event.get("subscription")
            if isinstance(handle, int) and isinstance(obj, int):
                sub_obj[(pid, handle)] = obj
                obj_to_handle[(pid, obj)] = handle

        elif name == "rclcpp_subscription_callback_added":
            obj = event.get("subscription")
            callback = event.get("callback")
            if isinstance(obj, int) and isinstance(callback, int):
                sub_cb[(pid, obj)] = callback

        elif name == "rcl_publisher_init":
            rmw = event.get("rmw_publisher_handle")
            topic = event.get("topic_name")
            if isinstance(rmw, int) and isinstance(topic, str):
                rmw_pub_topic[(pid, rmw)] = topic
            if isinstance(topic, str):
                pub_decl.setdefault((pid, event.get("node_handle")), set()).add(topic)

        elif name == "rmw_take":
            stamp = event.get("source_timestamp")
            rmw = event.get("rmw_subscription_handle")
            thread = _thread(event)
            if (thread is not None and event.get("taken") == 1
                    and isinstance(stamp, int) and (pid, rmw) in rmw_sub):
                ident["takes"] += 1
                if thread in taken:
                    # Two takes before a callback: which one it handles is unknown.
                    taken[thread] = _AMBIGUOUS
                else:
                    taken[thread] = (rmw, (rmw_sub[(pid, rmw)][1], stamp))

        elif name == "rmw_publish":
            stamp = event.get("timestamp")
            thread = _thread(event)
            cur = running.get(thread) if thread is not None else None
            topic = rmw_pub_topic.get((pid, event.get("rmw_publisher_handle")))
            if isinstance(stamp, int) and topic is not None:
                published_keys[(topic, stamp)] = published_keys.get((topic, stamp), 0) + 1
                if cur is not None:
                    cur[2].append((topic, stamp))

        elif name == "rclcpp_callback_register":
            callback = event.get("callback")
            symbol = event.get("symbol")
            if isinstance(callback, int) and isinstance(symbol, str):
                symbols[(pid, callback)] = symbol

        elif name == "callback_start":
            callback = event.get("callback")
            if isinstance(callback, int):
                thread = _thread(event)
                pending[(pid, callback) if thread is None else (*thread, callback)] = (ts, event.cpu_id)
                timeline.total_callback_starts += 1
                if thread is not None:
                    consumed = None
                    take = taken.pop(thread, None)
                    if take is not None and take is not _AMBIGUOUS:
                        rmw, key = take
                        sub_handle = rmw_sub[(pid, rmw)][0]
                        owner = sub_cb.get((pid, sub_obj.get((pid, sub_handle))))
                        if owner == callback:
                            consumed = key
                            ident["takes_attributed"] += 1
                    running[thread] = [callback, consumed, []]

        elif name == "callback_end":
            callback = event.get("callback")
            if isinstance(callback, int):
                thread = _thread(event)
                started = pending.pop((pid, callback) if thread is None else (*thread, callback), None)
                cur = running.pop(thread, None) if thread is not None else None
                consumed, published = None, ()
                if cur is not None and cur[0] == callback:
                    consumed, published = cur[1], tuple(cur[2])
                if started is None:
                    timeline.unmatched_starts += 1
                else:
                    start_ns, cpu_id = started
                    timeline.completed.append((start_ns, ts))
                    raw.setdefault((pid, callback), []).append(
                        Invocation(start_ns=start_ns, end_ns=ts, cpu_id=cpu_id,
                                   consumed=consumed, published=published)
                    )

    timeline.unmatched_starts += len(pending)

    # ── resolve callbacks to nodes ────────────────────────────────────────
    info_by_cb: dict[tuple, CallbackInfo] = {}

    for (pid, timer_handle), callback in timer_cb.items():
        node_handle = timer_node.get((pid, timer_handle))
        info_by_cb[(pid, callback)] = CallbackInfo(
            handle=callback,
            pid=pid,
            node=node_names.get((pid, node_handle)) if node_handle is not None else None,
            kind="timer",
            period_ns=timer_period.get((pid, timer_handle)),
            symbol=symbols.get((pid, callback)),
        )

    for (pid, obj), callback in sub_cb.items():
        sub_handle = obj_to_handle.get((pid, obj))
        node_handle = sub_node.get((pid, sub_handle)) if sub_handle is not None else None
        info_by_cb[(pid, callback)] = CallbackInfo(
            handle=callback,
            pid=pid,
            node=node_names.get((pid, node_handle)) if node_handle is not None else None,
            kind="subscription",
            topic=sub_topic.get((pid, sub_handle)) if sub_handle is not None else None,
            symbol=symbols.get((pid, callback)),
        )

    # Fall back to the registered C++ symbol for anything still unresolved.
    for key in raw:
        info = info_by_cb.get(key)
        symbol = symbols.get(key)
        if info is None:
            info = CallbackInfo(handle=key[1], pid=key[0], symbol=symbol)
            info_by_cb[key] = info
        if info.node is None and symbol:
            info.node = class_from_symbol(symbol)
        if info.kind == "unknown" and symbol:
            info.kind = "timer" if is_timer_symbol(symbol) else "subscription"

    timeline.callbacks = info_by_cb

    # ── group invocations by canonical node id ───────────────────────────
    for key, invs in raw.items():
        info = info_by_cb.get(key)
        if info is None or not info.node:
            continue
        if filter_keys is not None and canonical_key(info.node) not in filter_keys:
            continue
        # Canonicalise: the trace name and the model's chain identifiers may
        # follow different conventions. Unmodelled nodes keep their trace name.
        node = (resolver(info.node) if resolver else None) or info.node
        info.canonical_id = node
        timeline.trace_names.setdefault(node, info.node)
        timeline.invocations.setdefault(node, []).extend(invs)
        if info.kind == "timer":
            timeline.timer_invocations.setdefault(node, []).extend(invs)
        elif info.kind == "subscription":
            timeline.sub_invocations.setdefault(node, []).extend(invs)
            if info.topic:
                timeline.topic_invocations.setdefault(
                    (node, info.topic), []
                ).extend(invs)

    for (pid, node_handle), topics in pub_decl.items():
        trace_name = node_names.get((pid, node_handle))
        if not trace_name:
            continue
        if filter_keys is not None and canonical_key(trace_name) not in filter_keys:
            continue
        node = (resolver(trace_name) if resolver else None) or trace_name
        timeline.declared_topics.setdefault(node, set()).update(topics)

    # Producer of every published message key. A key published twice names no
    # single message, so it is withdrawn rather than resolved to either. That
    # holds for a second publish outside any traced callback too.
    duplicates: set[tuple[str, int]] = {k for k, n in published_keys.items() if n > 1}
    for node, invs in timeline.invocations.items():
        for inv in invs:
            for key in inv.published:
                if key in timeline.producer or key in duplicates:
                    timeline.producer.pop(key, None)
                    duplicates.add(key)
                else:
                    timeline.producer[key] = (node, inv)
    ident["publishes_attributed"] = len(timeline.producer)
    ident["duplicate_keys"] = len(duplicates)

    for bucket in (
        timeline.invocations,
        timeline.timer_invocations,
        timeline.sub_invocations,
        timeline.topic_invocations,
    ):
        for invs in bucket.values():
            invs.sort(key=lambda inv: inv.start_ns)

    return timeline


_AMBIGUOUS = object()


def _thread(event) -> Optional[tuple[int, int]]:
    """``(vpid, vtid)`` of *event*, ``None`` when the contexts were not traced."""
    if event.pid is None or event.tid is None:
        return None
    return (event.pid, event.tid)


def class_from_symbol(symbol: str) -> Optional[str]:
    """Extract the owning class name from a bound-callback C++ symbol.

    Returns ``None`` when the symbol has no ``Class::`` pattern (e.g. a plain
    lambda). Unlike the predecessor script this applies no allow-list.
    """
    match = _CLASS_RE.search(symbol)
    return match.group(1) if match else None


def is_timer_symbol(symbol: str) -> bool:
    """Heuristic: timer callbacks take no arguments, so no ``std::_Placeholder``.

    Subscription callbacks are bound with a placeholder for the message
    argument. Only used when the handle chain could not classify the callback.
    """
    return "Placeholder" not in symbol
