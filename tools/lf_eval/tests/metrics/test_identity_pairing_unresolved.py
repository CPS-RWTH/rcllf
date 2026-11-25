# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Identity pairing when the origin of an input is unknown.

A timer tick after an input whose message was never traced, or after an
ambiguous input, is unresolved, never paired with an older input. The tests
also guard per-process handles, startup samples, the records and summary rows
of hops and chains without a resolved sample, and the ordering verdict while
some activations are unresolved. Each regression test fails with the message
the defect produces.
"""

from __future__ import annotations

import pytest

from lf_eval.metrics.determinism import compute_ordering
from lf_eval.metrics.e2e_latency import (
    E2ELatencyMetric,
    e2e_latency_samples,
    hop_latency_samples,
    timeline_evidence,
)
from lf_eval.model.chain import Chain, Hop, build_chain
from lf_eval.model.run import make_run
from lf_eval.model.timeline import build_timeline

from tests.conftest import MS, EventBuilder


def _sensor_planner() -> EventBuilder:
    """Sensor (timer, publishes /raw) -> Planner (subscribes /raw, own timer)."""
    b = EventBuilder()
    b.pid = 7
    for node in ("Sensor", "Planner"):
        b.add_node(node)
    b.add_timer("Sensor", 100.0)
    b.add_publisher("Sensor", "/raw")
    b.add_subscription("Planner", "/raw")
    b.add_timer("Planner", 500.0)
    return b


def _release(b: EventBuilder, t0: int, stamp: int, traced_publish: bool = True) -> None:
    b.invoke(b.timer_callbacks["Sensor"], t0, 1 * MS, tid=1)
    if traced_publish:
        b.rmw_publish("Sensor", "/raw", t0 + MS // 2, stamp, tid=1)


def _absorb(b: EventBuilder, start: int, *stamps: int) -> None:
    for k, stamp in enumerate(stamps):
        b.rmw_take("Planner", "/raw", start - 1 - k, stamp, tid=2)
    b.invoke(b.sub_callbacks[("Planner", "/raw")], start, 1 * MS, tid=2)


GATED = Hop("Sensor", "Planner", gated=True)


def test_timer_after_an_input_of_unknown_origin_is_unresolved_not_older_paired():
    """the newest input's message was never traced, so the tick's data is unknown.

    A guard: the hop path already stopped here; the chain walk is checked by
    the test after the next one.
    """
    b = _sensor_planner()
    base = 1000 * MS
    _release(b, base, 1)
    _absorb(b, base + 2 * MS, 1)                       # clean input, stamp 1
    _release(b, base + 100 * MS, 2, traced_publish=False)
    _absorb(b, base + 102 * MS, 2)                     # newest input, producer untraced
    b.invoke(b.timer_callbacks["Planner"], base + 150 * MS, 1 * MS, tid=2)
    timeline = build_timeline(b.build())
    s = hop_latency_samples(timeline, GATED)
    assert (s.values_us, s.unresolved) == ([], 1), (
        f"the tick was paired with the older input: {s.values_us}")


def test_timer_after_an_ambiguous_input_is_unresolved_not_older_paired():
    """two takes before the newest input callback, so its message is unknown."""
    b = _sensor_planner()
    base = 1000 * MS
    _release(b, base, 1)
    _absorb(b, base + 2 * MS, 1)
    _release(b, base + 100 * MS, 2)
    _release(b, base + 110 * MS, 3)
    _absorb(b, base + 112 * MS, 2, 3)                  # two takes, one callback
    b.invoke(b.timer_callbacks["Planner"], base + 150 * MS, 1 * MS, tid=2)
    timeline = build_timeline(b.build())
    s = hop_latency_samples(timeline, GATED)
    assert (s.values_us, s.unresolved) == ([], 1), (
        f"the tick was paired with the older input: {s.values_us}")


def test_timer_before_any_input_is_a_startup_sample():
    """a tick before the first input is reported as a startup sample."""
    b = _sensor_planner()
    base = 1000 * MS
    b.invoke(b.timer_callbacks["Planner"], base - 50 * MS, 1 * MS, tid=2)
    _release(b, base, 1)
    _absorb(b, base + 2 * MS, 1)
    b.invoke(b.timer_callbacks["Planner"], base + 50 * MS, 1 * MS, tid=2)
    timeline = build_timeline(b.build())
    s = hop_latency_samples(timeline, GATED)
    assert (s.values_us, s.unresolved) == ([49_000.0], 0), (
        f"startup tick reported as values={s.values_us} unresolved={s.unresolved}")
    info = s.info()
    assert (info.get("startup"), info.get("complete")) == (1, True), (
        f"the startup tick is not reported in the tool output: {info}")


def test_chain_walk_through_a_timer_after_an_unknown_input_is_unresolved():
    """The e2e path: the walk and the hop share one rule."""
    b = _sensor_planner()
    b.add_node("Sink")
    b.add_publisher("Planner", "/plan")
    b.add_subscription("Sink", "/plan")
    base = 1000 * MS
    _release(b, base, 1)
    _absorb(b, base + 2 * MS, 1)
    _release(b, base + 100 * MS, 2, traced_publish=False)
    _absorb(b, base + 102 * MS, 2)
    b.invoke(b.timer_callbacks["Planner"], base + 150 * MS, 1 * MS, tid=2)
    b.rmw_publish("Planner", "/plan", base + 150 * MS + 10, 77, tid=2)
    b.rmw_take("Sink", "/plan", base + 160 * MS - 1, 77, tid=3)
    b.invoke(b.sub_callbacks[("Sink", "/plan")], base + 160 * MS, 1 * MS, tid=3)
    timeline = build_timeline(b.build())
    e2e = e2e_latency_samples(timeline, build_chain(["Sensor", "Planner", "Sink"]))
    assert (e2e.values_us, e2e.unresolved) == ([], 1), (
        f"the chain walk paired the tick with the older input: {e2e.values_us}")


def test_two_processes_with_identical_handles_keep_both_nodes():
    """handles are per-process addresses, a second process reuses them."""
    events = []
    for pid, node in ((1, "NodeA"), (2, "NodeB")):
        b = EventBuilder()
        b.pid = pid
        b.add_node(node)
        b.add_timer(node, 100.0)
        b.invoke(b.timer_callbacks[node], 1000 * MS + pid * MS, 1 * MS, tid=10 + pid)
        events += b.events
    events.sort(key=lambda e: e.timestamp_ns)
    timeline = build_timeline(events)
    got = {n: len(v) for n, v in timeline.invocations.items()}
    assert got == {"NodeA": 1, "NodeB": 1}, f"a reused handle merged two processes: {got}"


def test_identity_hop_without_a_resolved_pair_still_reports_its_counters():
    """an identity hop with only unresolved activations keeps its record."""
    b = _sensor_planner()
    base = 1000 * MS
    _release(b, base, 1)
    _absorb(b, base + 2 * MS, 1)
    _release(b, base + 100 * MS, 2, traced_publish=False)
    _absorb(b, base + 102 * MS, 2)
    b.invoke(b.timer_callbacks["Planner"], base + 150 * MS, 1 * MS, tid=2)
    timeline = build_timeline(b.build())
    chain = Chain(nodes=("Sensor", "Planner"), hops=(GATED,), name="sp")
    res = E2ELatencyMetric().compute([make_run("r", timeline, mode="stock")], chains=[chain])
    rec = res.data["per_run"]["r"]["chains"]["sp"]
    hop = rec["hops"].get("Sensor->Planner")
    assert hop is not None, "the hop with no resolved pair vanished from the output"
    assert (hop["count"], hop["unresolved"], hop["complete"]) == (0, 1, False)
    assert rec["complete"] is False


def test_chain_without_a_sample_keeps_its_summary_row():
    """a chain with no sample is listed with n 0, next to a chain with samples."""
    b = _sensor_planner()
    b.add_node("Idle")
    b.add_publisher("Planner", "/plan")
    b.add_subscription("Idle", "/plan")
    base = 1000 * MS
    for k in (1, 2):
        _release(b, base + (k - 1) * 100 * MS, k)
        _absorb(b, base + (k - 1) * 100 * MS + 2 * MS, k)
    timeline = build_timeline(b.build())
    chains = [
        Chain(nodes=("Sensor", "Planner"), hops=(Hop("Sensor", "Planner"),), name="raw"),
        Chain(nodes=("Planner", "Idle"), hops=(Hop("Planner", "Idle"),), name="idle"),
    ]
    res = E2ELatencyMetric().compute([make_run("r", timeline, mode="stock")], chains=chains)
    rows = sorted((r["chain"], r["n"]) for r in res.summary_rows)
    assert len(rows) == 2, f"the chain without a sample has no summary row: {rows}"
    assert sorted(n for _c, n in rows) == [0, 2], rows


def test_chain_with_an_untraced_hop_is_not_complete():
    """A hop the trace does not name leaves the chain incomplete.

    Pairing Planner->Idle by timestamps would report the chain
    ``complete: None``. The hop is judged by identity: three unresolved
    activations and ``complete: False``.
    """
    b = _sensor_planner()
    b.add_node("Idle")
    b.add_publisher("Planner", "/plan")
    b.add_subscription("Idle", "/plan")
    base = 1000 * MS
    for k in (1, 2, 3):
        t = base + (k - 1) * 100 * MS
        _release(b, t, k)
        _absorb(b, t + 2 * MS, k)
        # /plan carries no rmw_publish/rmw_take: Planner->Idle falls back to timestamps
        b.invoke(b.sub_callbacks[("Idle", "/plan")], t + 5 * MS, 1 * MS, tid=3)
    timeline = build_timeline(b.build())
    chain = Chain(nodes=("Sensor", "Planner", "Idle"),
                  hops=(Hop("Sensor", "Planner"), Hop("Planner", "Idle")), name="spi")
    res = E2ELatencyMetric().compute([make_run("r", timeline, mode="stock")], chains=[chain])
    rec = res.data["per_run"]["r"]["chains"]["spi"]
    pairing = {h: (v["pairing"], v["unresolved"]) for h, v in rec["hops"].items()}
    assert pairing == {"Sensor->Planner": ("message_identity", 0),
                       "Planner->Idle": ("message_identity", 3)}, (
        f"the untraced hop fell back to timestamps or lost its unresolved count: {pairing}")
    assert rec["complete"] is False, (
        f"a chain with an untraced hop was reported complete: {rec['complete']}")


def test_ordering_is_incomplete_not_pass_with_unresolved_activations():
    """no violation among the checked pairs is no PASS while some are unknown."""
    b = EventBuilder()
    b.pid = 9
    for node in ("NodeA", "NodeB"):
        b.add_node(node)
    b.add_timer("NodeA", 1000.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    base = 1000 * MS
    b.invoke(b.timer_callbacks["NodeA"], base, 1 * MS, tid=1)
    b.rmw_publish("NodeA", "/a_out", base + MS // 2, 1, tid=1)
    b.rmw_take("NodeB", "/a_out", base + 3 * MS - 1, 1, tid=2)
    b.invoke(b.sub_callbacks[("NodeB", "/a_out")], base + 3 * MS, 1 * MS, tid=2)
    # a callback whose message was never published in the trace
    b.rmw_take("NodeB", "/a_out", base + 10 * MS - 1, 2, tid=2)
    b.invoke(b.sub_callbacks[("NodeB", "/a_out")], base + 10 * MS, 1 * MS, tid=2)
    timeline = build_timeline(b.build())
    chain = Chain(nodes=("NodeA", "NodeB"), hops=(Hop("NodeA", "NodeB"),), name="ab")
    res = compute_ordering(timeline, chain)
    assert res.verdict == "INCOMPLETE", f"ordering verdict {res.verdict} on incomplete evidence"
    assert (res.violations, res.unresolved) == (0, 1)


@pytest.mark.parametrize("topic", ["/parameter_events", "/rosout"])
def test_infrastructure_topic_published_inside_a_callback_is_no_chain_data(topic):
    """A parameter service callback publishes /parameter_events itself."""
    b = EventBuilder()
    b.pid = 11
    for node in ("NodeA", "NodeB"):
        b.add_node(node)
    b.add_timer("NodeA", 1000.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_publisher("NodeA", topic)
    b.add_subscription("NodeB", "/a_out")
    b.add_subscription("NodeB", topic)
    base = 1000 * MS
    b.invoke(b.timer_callbacks["NodeA"], base, 1 * MS, tid=1)
    b.rmw_publish("NodeA", "/a_out", base + MS // 2, 1, tid=1)
    b.rmw_publish("NodeA", topic, base + MS // 2 + 1, 5, tid=1)
    b.rmw_take("NodeB", "/a_out", base + 3 * MS - 1, 1, tid=2)
    b.invoke(b.sub_callbacks[("NodeB", "/a_out")], base + 3 * MS, 1 * MS, tid=2)
    b.rmw_take("NodeB", topic, base + 5 * MS - 1, 5, tid=2)
    b.invoke(b.sub_callbacks[("NodeB", topic)], base + 5 * MS, 1 * MS, tid=2)
    timeline = build_timeline(b.build())
    # NodeB declares /a_out as its only input, so its callback on *topic* is foreign
    evidence = timeline_evidence(timeline, inputs={"NodeB": frozenset({"/a_out"})})
    e2e = e2e_latency_samples(timeline, build_chain(["NodeA", "NodeB"]), evidence=evidence)
    assert (len(e2e.values_us), e2e.unresolved) == (1, 0), (
        f"{topic} callback counted as chain data: values={e2e.values_us} "
        f"unresolved={e2e.unresolved}")
    assert getattr(e2e, "not_chain_data", None) == 1


def test_timer_after_two_overlapping_inputs_is_unresolved():
    """Inputs on two threads where the earlier-starting one contains the other.

    Which of the two wrote the state last is unknown, so the tick is unresolved.
    """
    b = _sensor_planner()
    base = 1000 * MS
    _release(b, base, 1)
    _release(b, base + 5 * MS, 2)
    b.rmw_take("Planner", "/raw", base + 10 * MS - 1, 1, tid=2)
    b.invoke(b.sub_callbacks[("Planner", "/raw")], base + 10 * MS, 40 * MS, tid=2)   # 10..50
    b.rmw_take("Planner", "/raw", base + 20 * MS - 1, 2, tid=3)
    b.invoke(b.sub_callbacks[("Planner", "/raw")], base + 20 * MS, 10 * MS, tid=3)   # 20..30
    b.invoke(b.timer_callbacks["Planner"], base + 60 * MS, 1 * MS, tid=4)
    timeline = build_timeline(b.build())
    s = hop_latency_samples(timeline, GATED)
    assert (s.values_us, s.unresolved) == ([], 1), (
        f"the tick was paired across two overlapping inputs: {s.values_us}")
