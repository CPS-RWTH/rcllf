# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Shared fixtures for lf_eval tests.

Everything above the reader layer is exercised on **synthetic in-Python events**
built by :func:`synth_events` — no CTF binaries are committed. The reader's own
integration test skips cleanly when no real trace or babeltrace backend exists.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional

import pytest

from lf_eval.reader.events import TraceEvent

# Root of the repository — three levels above tools/lf_eval/tests/
_REPO_ROOT = Path(__file__).resolve().parents[3]
_EXPERIMENTS = _REPO_ROOT / "output" / "experiments"
_PUBSUB_SYSTEM_JSON = (
    _REPO_ROOT / "packages" / "lf-analysis" / "pubsub_example" / "bringup" / "system.json"
)

MS = 1_000_000  # nanoseconds per millisecond


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return _REPO_ROOT


@pytest.fixture(scope="session")
def experiments_dir() -> Path:
    """``output/experiments/`` (gitignored; may not exist)."""
    return _EXPERIMENTS


@pytest.fixture(scope="session")
def pubsub_system_json() -> Path:
    """Extracted pubsub_example model (gitignored output; may not exist)."""
    return _PUBSUB_SYSTEM_JSON




@pytest.fixture(scope="session")
def real_run_dir() -> Optional[Path]:
    """A real experiment run dir with a ``tracing/`` subdir, or ``None``."""
    if not _EXPERIMENTS.is_dir():
        return None
    for child in sorted(_EXPERIMENTS.iterdir()):
        if child.is_dir() and (child / "tracing").is_dir() and not child.is_symlink():
            return child
    return None


# ── synthetic event construction ─────────────────────────────────────────


class EventBuilder:
    """Builds a coherent synthetic ``ros2:*`` event stream.

    Mirrors the real tracepoint sequence so the timeline builder's handle-chain
    resolution is genuinely exercised: node init, timer/subscription
    registration, then ``callback_start``/``callback_end`` pairs.
    """

    def __init__(self) -> None:
        self.events: list[TraceEvent] = []
        self._next_handle = 0x1000
        self.nodes: dict[str, int] = {}
        self.timer_callbacks: dict[str, int] = {}
        # (node, topic) -> callback handle
        self.sub_callbacks: dict[tuple[str, str], int] = {}
        self.publishers: dict[tuple[str, str], int] = {}
        self.rmw_pubs: dict[tuple[str, str], int] = {}
        self.rmw_subs: dict[tuple[str, str], int] = {}
        #: ``vpid`` stamped on every event; ``None`` models a trace without the
        #: thread contexts (no message identity).
        self.pid: Optional[int] = None

    def _handle(self) -> int:
        self._next_handle += 0x10
        return self._next_handle

    def _add(self, name: str, ts: int, tid: Optional[int] = None, **fields) -> None:
        self.events.append(TraceEvent(name=name, timestamp_ns=ts, fields=fields, cpu_id=0,
                                      pid=self.pid, tid=tid))

    # ── setup events ─────────────────────────────────────────────────────

    def add_node(self, name: str, ts: int = 0) -> int:
        handle = self._handle()
        self.nodes[name] = handle
        self._add("rcl_node_init", ts, node_handle=handle, node_name=name, namespace="/")
        return handle

    def add_timer(self, node: str, period_ms: float, ts: int = 0) -> int:
        node_handle = self.nodes[node]
        timer_handle = self._handle()
        callback = self._handle()
        self._add(
            "rcl_timer_init", ts, timer_handle=timer_handle, period=int(period_ms * MS)
        )
        self._add(
            "rclcpp_timer_link_node", ts, timer_handle=timer_handle, node_handle=node_handle
        )
        self._add(
            "rclcpp_timer_callback_added", ts, timer_handle=timer_handle, callback=callback
        )
        self._add(
            "rclcpp_callback_register",
            ts,
            callback=callback,
            symbol=f"std::_Bind<void ({node}::*({node}*))()>",
        )
        self.timer_callbacks[node] = callback
        return callback

    def add_subscription(self, node: str, topic: str, ts: int = 0) -> int:
        node_handle = self.nodes[node]
        sub_handle = self._handle()
        rmw_handle = self._handle()
        obj = self._handle()
        callback = self._handle()
        self._add(
            "rcl_subscription_init",
            ts,
            subscription_handle=sub_handle,
            node_handle=node_handle,
            rmw_subscription_handle=rmw_handle,
            topic_name=topic,
            queue_depth=10,
        )
        self._add(
            "rclcpp_subscription_init",
            ts,
            subscription_handle=sub_handle,
            subscription=obj,
        )
        self._add(
            "rclcpp_subscription_callback_added", ts, subscription=obj, callback=callback
        )
        self._add(
            "rclcpp_callback_register",
            ts,
            callback=callback,
            symbol=(
                f"std::_Bind<void ({node}::*({node}*, std::_Placeholder<1>))"
                "(std::shared_ptr<Msg>)>"
            ),
        )
        self.sub_callbacks[(node, topic)] = callback
        self.rmw_subs[(node, topic)] = rmw_handle
        return callback

    def add_publisher(self, node: str, topic: str, ts: int = 0) -> int:
        handle = self._handle()
        rmw_handle = self._handle()
        self._add(
            "rcl_publisher_init",
            ts,
            publisher_handle=handle,
            node_handle=self.nodes[node],
            rmw_publisher_handle=rmw_handle,
            topic_name=topic,
            queue_depth=10,
        )
        self.publishers[(node, topic)] = handle
        self.rmw_pubs[(node, topic)] = rmw_handle
        return handle

    # ── runtime events ───────────────────────────────────────────────────

    def invoke(self, callback: int, start_ns: int, duration_ns: int,
               tid: Optional[int] = None) -> None:
        """Emit a ``callback_start``/``callback_end`` pair."""
        self._add("callback_start", start_ns, tid=tid, callback=callback, is_intra_process=0)
        self._add("callback_end", start_ns + duration_ns, tid=tid, callback=callback)

    def timer_fire(self, node: str, start_ns: int, duration_ns: int) -> None:
        self.invoke(self.timer_callbacks[node], start_ns, duration_ns)

    def sub_fire(
        self, node: str, topic: str, start_ns: int, duration_ns: int
    ) -> None:
        self.invoke(self.sub_callbacks[(node, topic)], start_ns, duration_ns)

    def publish(self, node: str, topic: str, ts: int) -> None:
        self._add(
            "rcl_publish", ts, publisher_handle=self.publishers[(node, topic)], message=0x1
        )

    def take(self, node: str, topic: str, ts: int, taken: int = 1) -> None:
        self._add(
            "rmw_take",
            ts,
            rmw_subscription_handle=self.rmw_subs[(node, topic)],
            message=0x1,
            source_timestamp=0,
            taken=taken,
        )

    def rmw_publish(self, node: str, topic: str, ts: int, stamp: int, tid: int) -> None:
        """``rmw_publish`` of the sample with source timestamp ``stamp``."""
        self._add("rmw_publish", ts, tid=tid, rmw_publisher_handle=self.rmw_pubs[(node, topic)],
                  message=0x1, timestamp=stamp)

    def rmw_take(self, node: str, topic: str, ts: int, stamp: int, tid: int) -> None:
        """Successful ``rmw_take`` of the sample with source timestamp ``stamp``."""
        self._add("rmw_take", ts, tid=tid, rmw_subscription_handle=self.rmw_subs[(node, topic)],
                  message=0x1, source_timestamp=stamp, taken=1)

    def build(self) -> list[TraceEvent]:
        """Return the events in chronological order."""
        return sorted(self.events, key=lambda e: e.timestamp_ns)


@pytest.fixture
def builder() -> EventBuilder:
    return EventBuilder()


def synth_events(builder: EventBuilder) -> list[TraceEvent]:
    """Convenience alias for :meth:`EventBuilder.build`."""
    return builder.build()


@pytest.fixture
def pipeline_builder() -> EventBuilder:
    """A 3-node pipeline: A (100ms timer) -> B -> C, 20 clean activations.

    Timings are deterministic and non-overlapping so the ordering invariant
    passes and per-hop latency is exactly 2 ms.
    """
    b = EventBuilder()
    for node in ("NodeA", "NodeB", "NodeC"):
        b.add_node(node)
    b.add_timer("NodeA", 100.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")

    period = 100 * MS
    base = 1_000 * MS
    for i in range(20):
        t0 = base + i * period
        b.timer_fire("NodeA", t0, 1 * MS)                  # ends t0+1ms
        b.publish("NodeA", "/a_out", t0 + 1 * MS)
        b.take("NodeB", "/a_out", t0 + 3 * MS)
        b.sub_fire("NodeB", "/a_out", t0 + 3 * MS, 1 * MS)  # ends t0+4ms
        b.publish("NodeB", "/b_out", t0 + 4 * MS)
        b.take("NodeC", "/b_out", t0 + 6 * MS)
        b.sub_fire("NodeC", "/b_out", t0 + 6 * MS, 1 * MS)  # ends t0+7ms
    return b


@pytest.fixture
def identity_pipeline_builder() -> EventBuilder:
    """The ``pipeline_builder`` pipeline with message identity on every hop.

    Same callbacks and timings, plus thread context and ``rmw_publish`` /
    ``rmw_take`` events, so every hop pairs by identity (2 ms per hop).
    """
    b = EventBuilder()
    b.pid = 5  # thread context (vpid, vtid) is what lets a take be attributed
    for node in ("NodeA", "NodeB", "NodeC"):
        b.add_node(node)
    b.add_timer("NodeA", 100.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")

    period = 100 * MS
    base = 1_000 * MS
    for i in range(20):
        t0 = base + i * period
        b.invoke(b.timer_callbacks["NodeA"], t0, 1 * MS, tid=1)
        b.rmw_publish("NodeA", "/a_out", t0 + MS // 2, 1000 + i, tid=1)
        b.rmw_take("NodeB", "/a_out", t0 + 3 * MS - 1, 1000 + i, tid=2)
        b.invoke(b.sub_callbacks[("NodeB", "/a_out")], t0 + 3 * MS, 1 * MS, tid=2)
        b.rmw_publish("NodeB", "/b_out", t0 + 3 * MS + MS // 2, 2000 + i, tid=2)
        b.rmw_take("NodeC", "/b_out", t0 + 6 * MS - 1, 2000 + i, tid=3)
        b.invoke(b.sub_callbacks[("NodeC", "/b_out")], t0 + 6 * MS, 1 * MS, tid=3)
    return b


PIPELINE_SYSTEM_JSON = {
    "name": "pipeline",
    "schema_version": 3,
    "nodes": {
        "NodeA": {
            "name": "NodeA",
            "class_name": "NodeA",
            "timers": [{"period_ms": 100.0, "callback": "timer_callback"}],
            "publishers": [{"topic": "/a_out", "qos_depth": 10}],
            "subscribers": [],
        },
        "NodeB": {
            "name": "NodeB",
            "class_name": "NodeB",
            "timers": [],
            "publishers": [{"topic": "/b_out", "qos_depth": 10}],
            "subscribers": [{"topic": "/a_out", "callback": "cb"}],
        },
        "NodeC": {
            "name": "NodeC",
            "class_name": "NodeC",
            "timers": [],
            "publishers": [],
            "subscribers": [{"topic": "/b_out", "callback": "cb"}],
        },
    },
    "communication": {
        "topics": [
            {
                "topic": "/a_out",
                "message_type": "Msg",
                "publishers": ["NodeA"],
                "subscribers": ["NodeB"],
            },
            {
                "topic": "/b_out",
                "message_type": "Msg",
                "publishers": ["NodeB"],
                "subscribers": ["NodeC"],
            },
        ],
        "services": [],
        "actions": [],
    },
}


@pytest.fixture
def pipeline_system():
    """:class:`SystemModel` matching :func:`pipeline_builder`."""
    from lf_eval.model.system import parse_system

    return parse_system(PIPELINE_SYSTEM_JSON)


@pytest.fixture
def pipeline_chain(pipeline_system):
    """The derived A->B->C chain."""
    from lf_eval.model.chain import resolve_chains

    return resolve_chains(pipeline_system)[0]


def make_timeline(events: Iterable[TraceEvent], node_filter=None):
    """Build a timeline from synthetic events."""
    from lf_eval.model.timeline import build_timeline

    return build_timeline(events, node_filter=node_filter)


@pytest.fixture
def pipeline_run(pipeline_builder, pipeline_system):
    """A :class:`Run` built from the synthetic pipeline, drop counts included."""
    from lf_eval.metrics.drops import collect_drop_counts
    from lf_eval.model.run import make_run

    events = pipeline_builder.build()
    timeline = make_timeline(events)
    counts = collect_drop_counts(events, timeline, system=pipeline_system)
    return make_run(
        "synthetic_lfc_run1", timeline, mode="lfc", drop_counts=counts, duration_s=2.0
    )
