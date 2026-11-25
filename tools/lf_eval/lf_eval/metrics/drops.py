# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Message drops and queue overflows.

Event/field assumptions (verified against real reference-system traces)
----------------------------------------------------------------------
ROS2 has **no** "message dropped" tracepoint, so drops are inferred from
counting asymmetries. Three signals, in decreasing directness:

1. **Failed takes** — ``rmw_take`` carries a ``taken`` field (0/1). A
   ``taken = 0`` event is a subscription wakeup that found no message: the
   executor was told data was ready but the middleware had nothing to hand over
   (spurious wakeup, or the message was already consumed). In a 60 s
   reference-system run this is ~1.4 % of takes. Reported per subscription and
   totalled; ``rmw_take`` gives ``rmw_subscription_handle``, which is *not* the
   ``subscription_handle`` used by ``rcl_subscription_init``, so topic
   attribution comes from ``rcl_subscription_init``'s
   ``rmw_subscription_handle`` field when present.

2. **Publish-vs-delivery deltas per topic** — ``rcl_publish`` carries
   ``publisher_handle``, which ``rcl_publisher_init`` maps to a
   ``topic_name``, giving a per-topic publish count. The delivered count is the
   number of subscription-callback invocations on that topic
   (``Timeline.topic_invocations``). ``delivered - published * n_subscribers``
   is then the shortfall.

   **Assumption / caveat:** a topic with *N* subscribers should deliver *N*
   callbacks per publish, so the expectation is ``published * N``. A negative
   delta (shortfall) indicates messages that were published but never reached a
   callback — the classic queue-overflow symptom when a publisher outruns a
   subscriber with a shallow QoS depth. A *positive* delta is also possible and
   is **not** a drop: it means callbacks ran for messages published before the
   trace started, or intra-process delivery bypassed ``rcl_publish``. Both
   directions are reported; only shortfalls are counted as drops.

3. **Startup skew** — messages in flight when tracing started/stopped make the
   counts approximate at the edges. The metric reports the raw counts alongside
   the deltas so a reader can judge whether a small delta is meaningful. No
   attempt is made to trim a warmup window; for a 60 s run the edge effect is a
   handful of messages.

Queue depths declared in ``system.json`` (``publishers[].qos_depth``) are
reported for context when available, because a shortfall on a depth-1 topic is
expected behaviour rather than a fault.

Per-reader sample accounting
----------------------------------------
``rmw_publish`` carries the sample's source ``timestamp`` and ``rmw_take`` the
``source_timestamp`` of the sample it handed over, so each published sample is
matched to each reader (``rmw_subscription_handle`` of a
``rcl_subscription_init`` with its ``queue_depth``). A sample a reader never
took is classified by what the trace shows on that reader:

* ``before_reader``: published before the reader was created.
* ``overwritten``: KEEP_LAST overwrite. At least ``queue_depth`` newer samples
  were published on the topic before the reader's next take attempt that would
  have returned this sample had it still been queued (a take that found nothing,
  or took this or a newer sample), or before the end of the trace when no such
  take follows.
* ``before_match``: published after the reader was created but before DDS
  matched it to a writer. A VOLATILE sample published then is never delivered.
  The match time is the last poll before the reader's first
  ``rcllf_dds:reader_matched`` event after the sample's writer was created
  that still found no matched publisher (the event time minus its
  ``unmatched_age_ns``). The probe traces every increase of the count. The match probe
  emits it for every variant alike. A reader without that event exempts
  nothing here.
* ``queued_at_end``: no such take follows and fewer than ``queue_depth`` newer
  samples do. It was still in the reader's queue when the run stopped.
* ``unclassified``: the reader attempted such a take before ``queue_depth``
  newer samples had arrived, so no overwrite explains the loss.

``published = received + before_reader + before_match + overwritten +
queued_at_end + unclassified`` per reader. ``before_first_receipt`` is reported
beside them and exempts nothing: it counts the samples the reader never took
that were published after it was created and before the oldest sample it ever
received, whatever their category. ``match_traced`` says whether the reader has
a traced match time. ``before_writer_match`` and ``after_both_match`` split
``unclassified`` by the probe's ``rcllf_dds:writer_matched`` events: a
loss published before the writer's relevant increase (the latest one after
this reader's creation that precedes the first sample the reader received
from that writer) is ``before_writer_match``, a later one
``after_both_match``. That increase may belong to another reader, so the
first is an upper bound and the second a lower bound on losses after both
sides matched. A loss whose writer was not traced is in neither. The output
check exempts ``before_writer_match`` as a startup discovery gap and fails
the rest.

Each traced increase has two ends: the bound above (the last
observation of the lower count) and its detection, the event's own time. The
probe detects an increase from the rmw matched event callback (``source``
``event``) where the rmw has one, else from its 1 ms poll (``poll``, also the
reading of a trace without the field). The match lies between the two ends.
The exemptions stay keyed to the bound. A loss after the writer bound counts
as ``after_both_match`` only if it was published at or after the detection on
both sides, else as ``within_match_resolution``: the trace cannot show that
both sides had matched, and the output check fails it like
``after_both_match``. When detections are supplied but the reader side has
none for the sample, a loss after the writer detection is
``no_reader_detection``, also failing: the trace cannot show the reader
matched. Callers that pass only bounds keep the split. ``DropCounts.match_sources`` counts the traced
increases per source. Before the reader's first receipt, a loss published
before a side's match detection is a discovery loss on any queue depth and gets
the writer split, never ``overwritten``, also when no take follows it but
``queue_depth`` newer samples do. Elsewhere a sample lost in
discovery and followed by ``queue_depth`` newer ones before the next take is
indistinguishable from an overwrite in this trace and counts as one.

Comparative metrics (overwrites, drops, delivery latency) are counted again in
an accounting window over the samples published in it. By default it runs
from the moment the last application reader of the system was created (the
latest ``rcl_subscription_init`` outside the infrastructure topics) to the
run's stop, its last application ``rmw_publish``, the same rule for every
variant. The window does not move with the traced match times, so every
variant is counted over the same rule. :meth:`DropCounts.set_window` replaces it, for example with a
window common to every run of a campaign. Inside a window that starts after
every reader exists ``before_reader`` is 0. The pass rule stays the whole run:
an unclassified loss fails it, inside the window or not. ``before_reader`` and
``before_match`` never fail it.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Sequence

from lf_eval.metrics.registry import Metric, MetricResult, register
from lf_eval.model.chain import Chain
from lf_eval.model.run import Run
from lf_eval.model.system import SystemModel

#: Events the drops metric needs beyond the timeline's own set.
DROP_EVENTS = frozenset(
    {
        "rcl_publisher_init",
        "rcl_subscription_init",
        "rcl_publish",
        "rclcpp_intra_publish",
        "rmw_take",
        "rmw_publish",
        "reader_matched",  # rcllf_dds, the match probe
        "writer_matched",  # rcllf_dds, the same probe, writer side
    }
)

# Topics that are ROS infrastructure rather than application data flow.
_INFRA_TOPIC_PREFIXES = ("/rosout", "/parameter_events", "/events/", "/clock")


@dataclass
class TopicCounts:
    """Publish/delivery counts for one topic."""

    topic: str
    published: int = 0
    delivered: int = 0
    subscribers: int = 0
    failed_takes: int = 0
    qos_depth: Optional[int] = None

    @property
    def expected(self) -> int:
        """Callbacks expected: one per publish per subscriber."""
        return self.published * self.subscribers

    @property
    def delta(self) -> int:
        """``delivered - expected``; negative means a shortfall."""
        return self.delivered - self.expected

    @property
    def dropped(self) -> int:
        """Shortfall magnitude (0 when deliveries met or exceeded expectation)."""
        return max(0, -self.delta)

    @property
    def drop_rate(self) -> float:
        return self.dropped / self.expected if self.expected > 0 else 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "published": self.published,
            "delivered": self.delivered,
            "subscribers": self.subscribers,
            "expected": self.expected,
            "delta": self.delta,
            "dropped": self.dropped,
            "drop_rate_pct": round(100.0 * self.drop_rate, 4),
            "failed_takes": self.failed_takes,
            "qos_depth": self.qos_depth,
        }


@dataclass
class ReaderCounts:
    """One reader's samples on one topic, matched by source timestamp."""

    topic: str
    node: str
    queue_depth: Optional[int] = None
    published: int = 0
    received: int = 0
    overwritten: int = 0
    before_reader: int = 0
    before_match: int = 0
    queued_at_end: int = 0
    unclassified: int = 0
    unmatched_takes: int = 0
    #: Diagnostic, no category: samples never taken, published after the reader
    #: was created and before its first receipt.
    before_first_receipt: int = 0
    #: Diagnostic split of ``unclassified`` by the writer side match:
    #: published before the sample's writer reached its final matched
    #: subscription count, or after it (the reader having matched too). A
    #: sample whose writer the probe did not trace is in neither.
    before_writer_match: int = 0
    after_both_match: int = 0
    #: The part of the writer split's later losses published before a side's
    #: detection of the relevant increase: inside the probe's resolution.
    within_match_resolution: int = 0
    #: Later losses with detections supplied but no reader side detection for
    #: the sample's writer: the trace cannot show the reader matched.
    no_reader_detection: int = 0
    #: Trace ns of the reader's first DDS match, None when not traced.
    matched_ns: Optional[int] = None
    #: The same counts over samples published inside the accounting window.
    window: Optional["ReaderCounts"] = None
    #: ``rmw_take - rmw_publish`` trace ns of every received sample (window only).
    latency_ns: list[int] = field(default_factory=list)
    #: ``(publish ns, category, latency ns or None)`` of every sample, for windows.
    outcomes: list[tuple[int, str, Optional[int]]] = field(default_factory=list, repr=False)
    #: Publish ns of the samples counted in ``before_first_receipt``, for windows.
    pre_receipt_ns: list[int] = field(default_factory=list, repr=False)
    #: ``(publish ns, "before_writer_match" | "within_match_resolution" |
    #: "no_reader_detection" | "after_both_match")``, for windows.
    writer_split: list[tuple[int, str]] = field(default_factory=list, repr=False)

    @property
    def dropped(self) -> int:
        """Samples the reader never took although it existed when they were published."""
        return self.overwritten + self.queued_at_end + self.unclassified

    @property
    def edge(self) -> str:
        return f"{self.topic}->{self.node}"

    @property
    def reconciles(self) -> bool:
        return self.published == (self.received + self.overwritten + self.before_reader
                                  + self.before_match + self.queued_at_end + self.unclassified)

    def to_dict(self) -> dict[str, Any]:
        return {
            "queue_depth": self.queue_depth,
            "published": self.published,
            "received": self.received,
            "overwritten": self.overwritten,
            "before_reader": self.before_reader,
            "before_match": self.before_match,
            "queued_at_end": self.queued_at_end,
            "unclassified": self.unclassified,
            "before_first_receipt": self.before_first_receipt,
            "before_writer_match": self.before_writer_match,
            "within_match_resolution": self.within_match_resolution,
            "no_reader_detection": self.no_reader_detection,
            "after_both_match": self.after_both_match,
            "match_traced": self.matched_ns is not None,
            "unmatched_takes": self.unmatched_takes,
            **({"window": self.window.window_dict()} if self.window is not None else {}),
        }

    def window_dict(self) -> dict[str, Any]:
        return {"published": self.published, "received": self.received, "overwritten": self.overwritten,
                "before_reader": self.before_reader, "before_match": self.before_match,
                "before_first_receipt": self.before_first_receipt,
                "before_writer_match": self.before_writer_match,
                "within_match_resolution": self.within_match_resolution,
                "no_reader_detection": self.no_reader_detection,
                "after_both_match": self.after_both_match,
                "queued_at_end": self.queued_at_end,
                "unclassified": self.unclassified, "dropped": self.dropped,
                "delivery_latency_ns": latency_summary(self.latency_ns)}


def latency_summary(values: Sequence[int]) -> dict[str, Any]:
    """Count, median, p99 and max of latencies in ns (nearest rank)."""
    if not values:
        return {"n": 0, "p50": None, "p99": None, "max": None}
    v = sorted(values)

    def rank(q: float) -> int:
        return v[min(len(v) - 1, max(0, -(-len(v) * q // 100) - 1))]

    return {"n": len(v), "p50": int(rank(50)), "p99": int(rank(99)), "max": v[-1]}


@dataclass
class DropCounts:
    """Drop-related counts for one run."""

    topics: dict[str, TopicCounts] = field(default_factory=dict)
    total_takes: int = 0
    failed_takes: int = 0
    unattributed_failed_takes: int = 0
    #: Per reader, keyed by ``topic->node``; empty when the trace has no
    #: ``rmw_publish`` source timestamps for the topic (intra-process only).
    readers: dict[str, ReaderCounts] = field(default_factory=dict)
    #: Accounting window (inclusive, trace ns), the run's stop (last application
    #: ``rmw_publish``) and the first traced event; None without readers.
    window_start_ns: Optional[int] = None
    window_end_ns: Optional[int] = None
    stop_ns: Optional[int] = None
    trace_begin_ns: Optional[int] = None
    #: Traced matched count increases per detection source, ``event`` or ``poll``.
    match_sources: dict[str, int] = field(default_factory=dict)

    def set_window(self, start_ns: int, end_ns: Optional[int]) -> None:
        """Count every reader's comparative numbers again over ``[start_ns, end_ns]``."""
        self.window_start_ns, self.window_end_ns = start_ns, end_ns
        for rc in self.readers.values():
            rc.window = window_counts(rc, start_ns, end_ns)

    @property
    def failed_take_rate(self) -> float:
        return self.failed_takes / self.total_takes if self.total_takes else 0.0

    @property
    def total_dropped(self) -> int:
        return sum(t.dropped for t in self.topics.values())

    @property
    def total_expected(self) -> int:
        return sum(t.expected for t in self.topics.values())

    def to_dict(self) -> dict[str, Any]:
        return {
            "total_takes": self.total_takes,
            "failed_takes": self.failed_takes,
            "failed_take_rate_pct": round(100.0 * self.failed_take_rate, 4),
            "unattributed_failed_takes": self.unattributed_failed_takes,
            "total_dropped": self.total_dropped,
            "total_expected": self.total_expected,
            "drop_rate_pct": round(
                100.0 * self.total_dropped / self.total_expected, 4
            )
            if self.total_expected
            else 0.0,
            "topics": {k: v.to_dict() for k, v in sorted(self.topics.items())},
            "readers": {k: v.to_dict() for k, v in sorted(self.readers.items())},
            "window_start_ns": self.window_start_ns,
            "window_end_ns": self.window_end_ns,
            "stop_ns": self.stop_ns,
            "trace_begin_ns": self.trace_begin_ns,
            "match_sources": dict(self.match_sources),
        }


@register("drops")
class DropsMetric(Metric):
    """Message drops / queue overflows inferred from counting asymmetries."""

    summary = "Message drops and queue overflows (publish-vs-delivered, failed takes)"

    def compute(
        self,
        runs: Sequence[Run],
        system: Optional[SystemModel] = None,
        chains: Sequence[Chain] = (),
        **options: Any,
    ) -> MetricResult:
        per_run: dict[str, Any] = {}
        summary_rows: list[dict[str, Any]] = []

        for run in runs:
            counts = getattr(run, "drop_counts", None)
            if counts is None:
                per_run[run.name] = {
                    "mode": run.mode,
                    "note": "no drop counts collected for this run",
                }
                continue
            per_run[run.name] = {"mode": run.mode, **counts.to_dict()}
            summary_rows.append(
                {
                    "run": run.name,
                    "mode": run.mode,
                    "published": sum(t.published for t in counts.topics.values()),
                    "delivered": sum(t.delivered for t in counts.topics.values()),
                    "dropped": counts.total_dropped,
                    "drop_pct": round(
                        100.0 * counts.total_dropped / counts.total_expected, 3
                    )
                    if counts.total_expected
                    else 0.0,
                    "failed_takes": counts.failed_takes,
                    "failed_take_pct": round(100.0 * counts.failed_take_rate, 3),
                }
            )

        if not summary_rows:
            return MetricResult(
                name=self.name,
                data={"per_run": per_run},
                available=False,
                note=(
                    "no publish/take events collected — the trace must include "
                    "rcl_publish / rmw_take (docker/trace.sh enables all ros2:* "
                    "events, so this usually means an empty or filtered trace)"
                ),
            )

        return MetricResult(
            name=self.name,
            data={"per_run": per_run},
            summary_rows=summary_rows,
        )

    def plot(self, result: MetricResult, outdir: Path) -> list[Path]:
        if not result.available:
            return []
        return _plot_drops(result, Path(outdir))


def collect_drop_counts(
    events,
    timeline,
    system: Optional[SystemModel] = None,
    include_infra: bool = False,
) -> DropCounts:
    """Count publishes, deliveries and failed takes from a trace event stream.

    Consumes the same event stream the timeline was built from (call it with a
    tee'd/duplicated iterable, or use :func:`collect_from_events` which does
    both in one pass).
    """
    counts = DropCounts()
    # Handles are addresses, unique only within one process: every handle map
    # is keyed by (vpid, handle).
    pub_topic: dict[tuple, str] = {}       # (pid, publisher_handle) -> topic
    rmw_sub_topic: dict[tuple, str] = {}   # (pid, rmw_subscription_handle) -> topic
    published: dict[str, int] = {}
    # Per publisher handle. An intra-process publish (rslcpp) emits
    # rclcpp_intra_publish and no rcl_publish; a message that also goes to an
    # inter-process subscriber emits both, so a publisher counts the larger one.
    rcl_pubs: dict[tuple, int] = {}
    intra_pubs: dict[tuple, int] = {}
    failed_by_topic: dict[str, int] = {}
    # Per-reader accounting. Handles are pointers, unique only within a process.
    node_names: dict[tuple, str] = {}
    rmw_pub_topic: dict[tuple, str] = {}
    readers: dict[tuple, tuple[str, Any, Any, int]] = {}  # topic, node handle, depth, created ns
    samples: dict[str, list[tuple[int, int]]] = {}       # topic -> (publish ns, source timestamp)
    takes: dict[tuple, list[tuple[int, Optional[int]]]] = {}  # reader -> (take ns, source timestamp)
    # Every matched count increase the probe traced, (detection ns, bound of the
    # last observation before it), per reader and per writer.
    reader_incs: dict[tuple, list[tuple[int, int]]] = {}
    writer_incs: dict[tuple, list[tuple[int, int]]] = {}
    writer_created: dict[tuple, int] = {}  # writer -> rcl_publisher_init ns
    sample_writer: dict[str, dict[int, tuple]] = {}  # topic -> source timestamp -> writer

    for event in events:
        name = event.name
        if counts.trace_begin_ns is None or event.timestamp_ns < counts.trace_begin_ns:
            counts.trace_begin_ns = event.timestamp_ns
        if name == "rcl_node_init":
            handle = event.get("node_handle")
            if isinstance(handle, int):
                node_names[(event.pid, handle)] = str(event.get("node_name") or "")
        elif name == "rcl_publisher_init":
            handle = event.get("publisher_handle")
            topic = event.get("topic_name")
            if isinstance(handle, int) and isinstance(topic, str):
                pub_topic[(event.pid, handle)] = topic
                rmw_handle = event.get("rmw_publisher_handle")
                if isinstance(rmw_handle, int):
                    rmw_pub_topic[(event.pid, rmw_handle)] = topic
                    writer_created[(event.pid, rmw_handle)] = event.timestamp_ns
        elif name == "rmw_publish":
            topic = rmw_pub_topic.get((event.pid, event.get("rmw_publisher_handle")))
            stamp = event.get("timestamp")
            if topic and isinstance(stamp, int) and stamp:
                samples.setdefault(topic, []).append((event.timestamp_ns, stamp))
                sample_writer.setdefault(topic, {})[stamp] = (event.pid, event.get("rmw_publisher_handle"))
        elif name == "rcl_subscription_init":
            rmw_handle = event.get("rmw_subscription_handle")
            topic = event.get("topic_name")
            if isinstance(rmw_handle, int) and isinstance(topic, str):
                rmw_sub_topic[(event.pid, rmw_handle)] = topic
                readers[(event.pid, rmw_handle)] = (topic, event.get("node_handle"),
                                                    event.get("queue_depth"), event.timestamp_ns)
        elif name == "reader_matched":  # rcllf_dds, the match probe
            handle = event.get("rmw_subscription_handle")
            if isinstance(handle, int):
                key = (event.pid, handle)
                # The probe's last poll that still found no publisher: the
                # reader was certainly unmatched then.
                age = event.get("unmatched_age_ns")
                bound = event.timestamp_ns - (age if isinstance(age, int) and age > 0 else 0)
                reader_incs.setdefault(key, []).append((event.timestamp_ns, bound))
                _count_source(counts, event)
        elif name == "writer_matched":  # rcllf_dds, writer side
            handle = event.get("rmw_publisher_handle")
            count = event.get("subscriptions")
            if isinstance(handle, int) and isinstance(count, int):
                key = (event.pid, handle)
                age = event.get("unmatched_age_ns")
                bound = event.timestamp_ns - (age if isinstance(age, int) and age > 0 else 0)
                writer_incs.setdefault(key, []).append((event.timestamp_ns, bound))
                _count_source(counts, event)
        elif name in ("rcl_publish", "rclcpp_intra_publish"):
            handle = event.get("publisher_handle")
            if isinstance(handle, int):
                per = rcl_pubs if name == "rcl_publish" else intra_pubs
                per[(event.pid, handle)] = per.get((event.pid, handle), 0) + 1
        elif name == "rmw_take":
            counts.total_takes += 1
            stamp = event.get("source_timestamp")
            takes.setdefault((event.pid, event.get("rmw_subscription_handle")), []).append(
                (event.timestamp_ns, stamp if event.get("taken") == 1 and isinstance(stamp, int) else None))
            if event.get("taken") == 0:
                counts.failed_takes += 1
                handle = event.get("rmw_subscription_handle")
                topic = rmw_sub_topic.get((event.pid, handle)) if isinstance(handle, int) else None
                if topic:
                    failed_by_topic[topic] = failed_by_topic.get(topic, 0) + 1
                else:
                    counts.unattributed_failed_takes += 1

    for handle in set(rcl_pubs) | set(intra_pubs):
        topic = pub_topic.get(handle)
        if topic:
            published[topic] = published.get(topic, 0) + max(
                rcl_pubs.get(handle, 0), intra_pubs.get(handle, 0))

    # Delivered counts come from the timeline's per-topic subscription callbacks.
    delivered: dict[str, int] = {}
    subscriber_count: dict[str, int] = {}
    for (_node, topic), invs in timeline.topic_invocations.items():
        delivered[topic] = delivered.get(topic, 0) + len(invs)
        subscriber_count[topic] = subscriber_count.get(topic, 0) + 1

    qos_depths = _qos_depths(system)

    for topic in set(published) | set(delivered) | set(failed_by_topic):
        if not include_infra and _is_infra_topic(topic):
            continue
        subs = subscriber_count.get(topic, 0)
        if subs == 0 and system is not None:
            subs = _declared_subscribers(system, topic)
        counts.topics[topic] = TopicCounts(
            topic=topic,
            published=published.get(topic, 0),
            delivered=delivered.get(topic, 0),
            subscribers=subs,
            failed_takes=failed_by_topic.get(topic, 0),
            qos_depth=qos_depths.get(topic),
        )

    created_all = [c for t, _n, _d, c in readers.values() if include_infra or not _is_infra_topic(t)]
    app_pubs = [t for topic, ss in samples.items() if include_infra or not _is_infra_topic(topic) for t, _ in ss]
    counts.stop_ns = max(app_pubs) if app_pubs else None
    for key, (topic, node_handle, depth, created) in readers.items():
        if topic not in samples or (not include_infra and _is_infra_topic(topic)):
            continue
        node = node_names.get((key[0], node_handle), "")
        writer_of = sample_writer.get(topic, {})
        reader_m, writer_m = _relevant_matches(
            reader_incs.get(key, []), created, samples[topic], takes.get(key, []), writer_of,
            writer_incs, writer_created)
        first = min((b for _e, b in reader_incs.get(key, [])), default=None)
        rc = classify_reader(topic, node, depth if isinstance(depth, int) else None, created,
                             samples[topic], takes.get(key, []), matched_ns=first,
                             writer_matched_ns={st: b for st, (b, _d) in writer_m.items()},
                             reader_matched_ns={st: b for st, (b, _d) in reader_m.items()},
                             writer_detected_ns={st: d for st, (_b, d) in writer_m.items()},
                             reader_detected_ns={st: d for st, (_b, d) in reader_m.items()})
        edge = rc.edge
        n = 2
        while edge in counts.readers:  # two readers of one node on one topic
            edge = f"{rc.edge}#{n}"
            n += 1
        counts.readers[edge] = rc
    if created_all:
        counts.set_window(max(created_all), counts.stop_ns)

    return counts


def _count_source(counts: DropCounts, event) -> None:
    """Count one traced increase under its detection source, ``poll`` when absent."""
    source = event.get("source")
    source = source if source in ("event", "poll") else "poll"
    counts.match_sources[source] = counts.match_sources.get(source, 0) + 1


def window_counts(rc: ReaderCounts, start_ns: int, end_ns: Optional[int]) -> ReaderCounts:
    """``rc``'s counts over the samples published in ``[start_ns, end_ns]`` (open end if None)."""
    w = ReaderCounts(topic=rc.topic, node=rc.node, queue_depth=rc.queue_depth,
                     matched_ns=rc.matched_ns)
    w.before_first_receipt = sum(
        1 for t in rc.pre_receipt_ns if t >= start_ns and (end_ns is None or t <= end_ns))
    for t, kind in rc.writer_split:
        if t >= start_ns and (end_ns is None or t <= end_ns):
            setattr(w, kind, getattr(w, kind) + 1)
    for t, kind, latency in rc.outcomes:
        if t < start_ns or (end_ns is not None and t > end_ns):
            continue
        w.published += 1
        setattr(w, kind, getattr(w, kind) + 1)
        if latency is not None:
            w.latency_ns.append(latency)
    return w


def _relevant_matches(reader_incs: Sequence[tuple[int, int]], created_ns: int,
                      samples: Sequence[tuple[int, int]],
                      takes: Sequence[tuple[int, Optional[int]]],
                      writer_of: dict[int, tuple],
                      writer_incs: dict[tuple, list[tuple[int, int]]],
                      writer_created: dict[tuple, int]
                      ) -> tuple[dict[int, tuple[int, int]], dict[int, tuple[int, int]]]:
    """Per sample, the match bounds that concern this reader and the sample's writer.

    Both sides trace every increase of their matched count, so a peer that
    leaves and is replaced shows as a new increase.

    * Reader side: the first increase the reader traced at or after the
      writer's creation, the earliest it can have matched that writer.
    * Writer side: of the writer's increases at or after this reader's
      creation, the latest whose bound precedes the first sample of that
      writer the reader received (the writer had matched the reader by then),
      else the latest. Other readers' later matches keep this an upper bound.
      Without one, the latest increase before the reader's creation: DDS can
      match inside rmw create, and the rmw matched event then fires before
      ``rcl_subscription_init`` is traced. Its bound precedes the
      creation and so every sample, so it never exempts a loss.

    Returns ``(reader side, writer side)`` keyed by source timestamp, each a
    ``(bound, detection ns)`` pair of the chosen increase. A sample whose side
    was not traced is absent.
    """
    pub_of = {st: t for t, st in samples}
    first_rx: dict[tuple, int] = {}  # writer -> publish ns of its first sample this reader took
    for _tn, st in takes:
        w = writer_of.get(st) if st is not None else None
        if w is not None and st in pub_of:
            first_rx[w] = min(first_rx.get(w, pub_of[st]), pub_of[st])
    reader_sorted = sorted(reader_incs)
    reader_by_writer: dict[tuple, Optional[tuple[int, int]]] = {}
    writer_by_writer: dict[tuple, Optional[tuple[int, int]]] = {}
    for w in set(writer_of.values()):
        born = writer_created.get(w)
        after = [(b, ev) for ev, b in reader_sorted if born is None or ev >= born]
        reader_by_writer[w] = after[0] if after else None
        incs = sorted(i for i in writer_incs.get(w, []) if i[0] >= created_ns)
        if not incs:
            incs = sorted(i for i in writer_incs.get(w, []) if i[0] < created_ns)[-1:]
        rx = first_rx.get(w)
        before_rx = [(b, ev) for ev, b in incs if rx is None or b <= rx]
        writer_by_writer[w] = before_rx[-1] if before_rx else ((incs[-1][1], incs[-1][0]) if incs else None)
    reader_ns = {st: reader_by_writer[w] for st, w in writer_of.items()
                 if reader_by_writer.get(w) is not None}
    writer_ns = {st: writer_by_writer[w] for st, w in writer_of.items()
                 if writer_by_writer.get(w) is not None}
    return reader_ns, writer_ns


def classify_reader(topic: str, node: str, depth: Optional[int], created_ns: int,
                    samples: Sequence[tuple[int, int]],
                    takes: Sequence[tuple[int, Optional[int]]],
                    matched_ns: Optional[int] = None,
                    writer_matched_ns: Optional[dict[int, int]] = None,
                    reader_matched_ns: Optional[dict[int, int]] = None,
                    writer_detected_ns: Optional[dict[int, int]] = None,
                    reader_detected_ns: Optional[dict[int, int]] = None) -> ReaderCounts:
    """Match one reader's takes to the topic's samples and classify the rest.

    :param samples: ``(publish trace ns, source timestamp)`` of every sample on the topic.
    :param takes: ``(take trace ns, source timestamp)`` of every take attempt of the
        reader, ``None`` for a take that found nothing (``taken = 0``).
    :param matched_ns: trace ns of the reader's first DDS match, None when not traced.
    :param writer_matched_ns: source timestamp -> bound of the poll before the
        writer side match relevant to this reader (``_relevant_matches``); a
        sample whose writer was not traced is absent.
    :param reader_matched_ns: source timestamp -> reader side match bound for the
        sample's writer. When given it replaces ``matched_ns`` per sample.
    :param writer_detected_ns: source timestamp -> detection ns of the writer
        side increase whose bound ``writer_matched_ns`` holds.
    :param reader_detected_ns: the same for the reader side. Without them each
        detection equals its bound and no loss is ``within_match_resolution``.
        With them a later loss lacking a reader detection is ``no_reader_detection``.
    The rules are in the module docstring. Without a queue depth no overwrite is
    assumed, so every loss a take could have seen stays unclassified.
    """
    rc = ReaderCounts(topic=topic, node=node, queue_depth=depth, matched_ns=matched_ns)
    samples = sorted(samples)
    pub_ns = [t for t, _ in samples]
    stamps = {s for _, s in samples}
    takes = sorted(takes, key=lambda x: x[0])
    take_ns = [t for t, _ in takes]
    taken: dict[int, int] = {}  # source timestamp -> first take ns
    for tn, s in takes:
        if s is not None:
            taken.setdefault(s, tn)
    rc.unmatched_takes = sum(1 for _, s in takes if s is not None and s not in stamps)
    need = depth if depth and depth > 0 else None
    by_stamp = {st: t for t, st in samples}
    first_rx = min((by_stamp[st] for st in taken if st in by_stamp), default=None)
    for i, (t, stamp) in enumerate(samples):
        rc.published += 1
        if stamp in taken:
            rc.received += 1
            rc.outcomes.append((t, "received", taken[stamp] - t))
            continue
        if t < created_ns:
            rc.before_reader += 1
            rc.outcomes.append((t, "before_reader", None))
            continue
        if first_rx is not None and t < first_rx:
            rc.before_first_receipt += 1
            rc.pre_receipt_ns.append(t)
        m_ns = reader_matched_ns.get(stamp) if reader_matched_ns is not None else matched_ns
        if m_ns is not None and t < m_ns:
            rc.before_match += 1
            rc.outcomes.append((t, "before_match", None))
            continue
        # The first take after the publish that would have returned this sample
        # had it still been queued: one that found nothing or took it or a newer one.
        j = bisect.bisect_right(take_ns, t)
        while j < len(takes) and takes[j][1] is not None and takes[j][1] < stamp:
            j += 1
        until = takes[j][0] if j < len(takes) else None
        newer = (bisect.bisect_right(pub_ns, until) if until is not None else len(samples)) - (i + 1)
        split = None
        w_ns = (writer_matched_ns or {}).get(stamp)
        if w_ns is not None:
            # A side's match lies between its bound and its detection.
            w_det = (writer_detected_ns or {}).get(stamp, w_ns)
            r_det = (reader_detected_ns or {}).get(stamp)
            detections = writer_detected_ns is not None or reader_detected_ns is not None
            if t < w_ns:
                split = "before_writer_match"
            elif t < w_det or (r_det is not None and t < r_det):
                split = "within_match_resolution"
            elif detections and r_det is None:
                split = "no_reader_detection"
            else:
                split = "after_both_match"
        # Before the reader's first receipt, a loss published before a side's match
        # detection is a discovery loss on any queue: the overwrite rule would hide
        # it on a depth-1 reader only. Without a later take, enough newer
        # samples still establish the loss; without either it stays queued_at_end.
        startup = split in ("before_writer_match", "within_match_resolution") and (
            first_rx is None or t < first_rx) and (
            until is not None or (need is not None and newer >= need))
        if startup:
            kind = "unclassified"
        elif need is not None and newer >= need:
            kind = "overwritten"
        elif until is None:
            kind = "queued_at_end"
        else:
            kind = "unclassified"
        if kind == "unclassified" and split is not None:
            setattr(rc, split, getattr(rc, split) + 1)
            rc.writer_split.append((t, split))
        setattr(rc, kind, getattr(rc, kind) + 1)
        rc.outcomes.append((t, kind, None))
    return rc


def _is_infra_topic(topic: str) -> bool:
    return any(topic.startswith(prefix) for prefix in _INFRA_TOPIC_PREFIXES)


def _qos_depths(system: Optional[SystemModel]) -> dict[str, int]:
    """Publisher QoS depth per topic, from ``system.json`` when loadable."""
    if system is None or system.source_path is None:
        return {}
    import json

    try:
        with open(system.source_path) as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return {}
    out: dict[str, int] = {}
    for node in (data.get("nodes") or {}).values():
        for pub in node.get("publishers") or []:
            topic = pub.get("topic")
            depth = pub.get("qos_depth")
            if isinstance(topic, str) and isinstance(depth, int):
                key = topic if topic.startswith("/") else f"/{topic}"
                out.setdefault(key, depth)
    return out


def _declared_subscribers(system: SystemModel, topic: str) -> int:
    """Subscriber count declared in ``system.json`` for a topic."""
    normalised = topic if topic.startswith("/") else f"/{topic}"
    for spec in system.topics:
        name = spec.name if spec.name.startswith("/") else f"/{spec.name}"
        if name == normalised:
            return len(spec.subscribers)
    return 0


def _plot_drops(result: MetricResult, outdir: Path) -> list[Path]:
    """Per-run drop and failed-take rates."""
    import numpy as np

    from lf_eval.plots import style

    per_run = {
        k: v for k, v in result.data.get("per_run", {}).items() if "total_takes" in v
    }
    if not per_run:
        return []

    names = list(per_run)
    modes = [per_run[n].get("mode", "unknown") for n in names]
    drop_pct = [per_run[n]["drop_rate_pct"] for n in names]
    take_pct = [per_run[n]["failed_take_rate_pct"] for n in names]

    fig, axes = style.figure(
        "Message Drops",
        "publish-vs-delivered shortfall and failed subscription takes",
        figsize=(max(9.0, 1.7 * len(names)), 5.0),
        ncols=2,
    )
    x = np.arange(len(names))
    labels = [n.replace("_", "\n") for n in names]
    colors = [style.mode_color(m) for m in modes]

    axes[0].bar(x, drop_pct, color=colors, alpha=0.9)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(labels, fontsize=7)
    axes[0].set_ylabel("Drop rate (%)")
    axes[0].set_title("Publish→callback shortfall")
    axes[0].grid(axis="y", alpha=0.3)

    axes[1].bar(x, take_pct, color=colors, alpha=0.9, hatch="//")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels, fontsize=7)
    axes[1].set_ylabel("Failed takes (%)")
    axes[1].set_title("rmw_take with taken = 0")
    axes[1].grid(axis="y", alpha=0.3)

    return [style.save(fig, outdir, "drops.png")]
