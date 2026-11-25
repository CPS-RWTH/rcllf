# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Latency pairs by message identity, no fixed pairing window.

``rmw_publish.timestamp`` and ``rmw_take.source_timestamp`` name one message on
both sides, so a hop pairs the invocation that consumed a message with the one
that published it. The old pairing took the first downstream start after the
upstream end and dropped everything past 200 ms (400 ms for gated hops).
"""

from __future__ import annotations

import pytest

from lf_eval.metrics.determinism import compute_ordering
from lf_eval.metrics.e2e_latency import (
    compute_e2e,
    compute_hops,
    e2e_latency_samples,
    hop_latency_samples,
    timeline_evidence,
)
from lf_eval.model.chain import Chain, Hop, build_chain
from lf_eval.model.timeline import build_timeline

from tests.conftest import MS, EventBuilder

A, B, C = 1, 2, 3  # one executor thread per node


def _chain_builder() -> EventBuilder:
    b = EventBuilder()
    b.pid = 4242
    for node in ("NodeA", "NodeB", "NodeC", "NodeX"):
        b.add_node(node)
    b.add_timer("NodeA", 1000.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_publisher("NodeX", "/x_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_subscription("NodeB", "/x_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")
    return b


def _a_to_b(b: EventBuilder, t0: int, gap: int, stamp: int) -> None:
    """A publishes at t0+1ms and ends at t0+1ms; B takes it ``gap`` after."""
    b.invoke(b.timer_callbacks["NodeA"], t0, 1 * MS, tid=A)
    b.rmw_publish("NodeA", "/a_out", t0 + MS // 2, stamp, tid=A)
    start = t0 + 1 * MS + gap
    b.rmw_take("NodeB", "/a_out", start - 1, stamp, tid=B)
    b.invoke(b.sub_callbacks[("NodeB", "/a_out")], start, 1 * MS, tid=B)


def test_hop_beyond_200ms_is_paired_not_dropped():
    b = _chain_builder()
    for i in range(5):
        _a_to_b(b, 1000 * MS + i * 1000 * MS, 250 * MS, stamp=10 + i)
    timeline = build_timeline(b.build())
    hops = dict((h.label, s) for h, s in compute_hops(timeline, build_chain(["NodeA", "NodeB"])))
    assert "NodeA->NodeB" in hops, "the 250 ms hop lost every pair to the window"
    assert hops["NodeA->NodeB"].count == 5
    assert hops["NodeA->NodeB"].mean_us == pytest.approx(250_000.0)


def test_unrelated_earlier_callback_is_not_paired():
    """B runs for /x_out between A's end and the take of A's message."""
    b = _chain_builder()
    for i in range(5):
        t0 = 1000 * MS + i * 1000 * MS
        b.invoke(b.timer_callbacks["NodeA"], t0, 1 * MS, tid=A)
        b.rmw_publish("NodeA", "/a_out", t0 + MS // 2, 100 + i, tid=A)
        # unrelated: a message of NodeX, taken 1 ms after A ended
        b.rmw_take("NodeB", "/x_out", t0 + 2 * MS - 1, 900 + i, tid=B)
        b.invoke(b.sub_callbacks[("NodeB", "/x_out")], t0 + 2 * MS, 1 * MS, tid=B)
        # causal: A's message, taken 4 ms after A ended
        b.rmw_take("NodeB", "/a_out", t0 + 5 * MS - 1, 100 + i, tid=B)
        b.invoke(b.sub_callbacks[("NodeB", "/a_out")], t0 + 5 * MS, 1 * MS, tid=B)
    timeline = build_timeline(b.build())
    hops = dict((h.label, s) for h, s in compute_hops(timeline, build_chain(["NodeA", "NodeB"])))
    assert hops["NodeA->NodeB"].count == 5
    assert hops["NodeA->NodeB"].mean_us == pytest.approx(4000.0), (
        "paired with the unrelated /x_out callback 1 ms after A's end")
    samples = hop_latency_samples(timeline, Hop("NodeA", "NodeB"))
    assert samples.pairing == "message_identity"


def test_e2e_walks_messages_through_a_slow_hop():
    b = _chain_builder()
    for i in range(4):
        t0 = 1000 * MS + i * 1000 * MS
        _a_to_b(b, t0, 300 * MS, stamp=200 + i)       # B runs t0+301 .. t0+302 ms
        b.rmw_publish("NodeB", "/b_out", t0 + 301 * MS + 10, 300 + i, tid=B)
        b.rmw_take("NodeC", "/b_out", t0 + 303 * MS - 1, 300 + i, tid=C)
        b.invoke(b.sub_callbacks[("NodeC", "/b_out")], t0 + 303 * MS, 1 * MS, tid=C)
    timeline = build_timeline(b.build())
    chain = build_chain(["NodeA", "NodeB", "NodeC"])
    stats = compute_e2e(timeline, chain)
    assert stats is not None and stats.count == 4, "chain samples lost to the window"
    # A start t0 -> C end t0+304 ms
    assert stats.mean_us == pytest.approx(304_000.0)
    assert e2e_latency_samples(timeline, chain).pairing == "message_identity"


def test_gated_hop_pairs_the_latest_message_before_the_tick():
    """A downstream timer is paired with the producer of the latest input."""
    b = EventBuilder()
    b.pid = 7
    for node in ("Sensor", "Planner"):
        b.add_node(node)
    b.add_timer("Sensor", 100.0)
    b.add_publisher("Sensor", "/raw")
    b.add_subscription("Planner", "/raw")
    b.add_timer("Planner", 500.0)
    base = 1000 * MS
    for i in range(10):  # Sensor every 100 ms, Planner absorbs at +2 ms
        t0 = base + i * 100 * MS
        b.invoke(b.timer_callbacks["Sensor"], t0, 1 * MS, tid=1)
        b.rmw_publish("Sensor", "/raw", t0 + MS // 2, i, tid=1)
        b.rmw_take("Planner", "/raw", t0 + 2 * MS - 1, i, tid=2)
        b.invoke(b.sub_callbacks[("Planner", "/raw")], t0 + 2 * MS, 1 * MS, tid=2)
    # Planner ticks 450 ms after the first Sensor release: the latest input is
    # Sensor's release 4 (t0 = base+400 ms, ends +401 ms), so 49 ms, where the
    # first-after rule would pair release 0 at 449 ms, past the old 400 ms.
    b.invoke(b.timer_callbacks["Planner"], base + 450 * MS, 1 * MS, tid=2)
    b.invoke(b.timer_callbacks["Planner"], base + 950 * MS, 1 * MS, tid=2)
    timeline = build_timeline(b.build())
    hop = Hop("Sensor", "Planner", gated=True)
    samples = hop_latency_samples(timeline, hop)
    assert samples.pairing == "message_identity"
    assert samples.values_us == pytest.approx([49_000.0, 49_000.0])


def test_two_takes_before_a_callback_attribute_nothing():
    b = _chain_builder()
    t0 = 1000 * MS
    b.invoke(b.timer_callbacks["NodeA"], t0, 1 * MS, tid=A)
    b.rmw_publish("NodeA", "/a_out", t0 + MS // 2, 1, tid=A)
    b.invoke(b.timer_callbacks["NodeA"], t0 + 2 * MS, 1 * MS, tid=A)
    b.rmw_publish("NodeA", "/a_out", t0 + 2 * MS + MS // 2, 2, tid=A)
    b.rmw_take("NodeB", "/a_out", t0 + 4 * MS, 1, tid=B)
    b.rmw_take("NodeB", "/a_out", t0 + 4 * MS + 1, 2, tid=B)
    b.invoke(b.sub_callbacks[("NodeB", "/a_out")], t0 + 5 * MS, 1 * MS, tid=B)
    timeline = build_timeline(b.build())
    (inv,) = timeline.sub_invocations["NodeB"]
    assert inv.consumed is None
    assert timeline.identity["takes"] == 2 and timeline.identity["takes_attributed"] == 0


def test_take_of_another_subscription_is_not_attributed():
    b = _chain_builder()
    t0 = 1000 * MS
    _a_to_b(b, t0, 1 * MS, stamp=5)
    # a take for /x_out, then the /a_out callback on the same thread
    b.rmw_take("NodeB", "/x_out", t0 + 10 * MS, 77, tid=B)
    b.invoke(b.sub_callbacks[("NodeB", "/a_out")], t0 + 11 * MS, 1 * MS, tid=B)
    timeline = build_timeline(b.build())
    consumed = [inv.consumed for inv in timeline.sub_invocations["NodeB"]]
    assert consumed == [("/a_out", 5), None]


def test_without_thread_context_the_timestamp_fallback_is_labelled(pipeline_builder,
                                                                   pipeline_chain):
    timeline = build_timeline(pipeline_builder.build())
    for hop in pipeline_chain.hops:
        samples = hop_latency_samples(timeline, hop)
        assert samples.pairing == "timestamp"
        assert samples.values_us == pytest.approx([2000.0] * 20)


def test_explicit_window_counts_what_it_excludes(pipeline_builder, pipeline_chain):
    timeline = build_timeline(pipeline_builder.build())
    samples = hop_latency_samples(timeline, pipeline_chain.hops[0], max_gap_ns=1 * MS)
    assert samples.values_us == [] and samples.excluded_by_window == 20
    e2e = e2e_latency_samples(timeline, pipeline_chain, max_gap_ns=1 * MS)
    assert e2e.values_us == [] and e2e.excluded_by_window == 20


def test_ordering_checks_the_publishing_callback():
    """B takes A's message while A still runs: a real overlap, found by identity."""
    b = _chain_builder()
    for i in range(3):
        t0 = 1000 * MS + i * 1000 * MS
        b.invoke(b.timer_callbacks["NodeA"], t0, 5 * MS, tid=A)
        b.rmw_publish("NodeA", "/a_out", t0 + 1 * MS, 50 + i, tid=A)
        b.rmw_take("NodeB", "/a_out", t0 + 2 * MS - 1, 50 + i, tid=B)
        b.invoke(b.sub_callbacks[("NodeB", "/a_out")], t0 + 2 * MS, 1 * MS, tid=B)
    timeline = build_timeline(b.build())
    chain = Chain(nodes=("NodeA", "NodeB"), hops=(Hop("NodeA", "NodeB"),), name="ab")
    res = compute_ordering(timeline, chain)
    assert (res.pairs_checked, res.violations) == (3, 3)
    assert res.pairing == {"NodeA->NodeB": "message_identity"}


def test_take_from_outside_the_system_is_not_unresolved():
    """A /parameter_events take (an input system.json does not declare) is no hop data.

    Seen on a stock sentry_pipeline trace: every node's /parameter_events
    callback counted as an unresolved sample of the hop into that node. Now
    the run's system.json decides (``communication.topics``), so the test
    passes the inputs that file would declare. Without them the take is
    unresolved, which is the conservative reading of an undeclared node.
    """
    b = _chain_builder()
    b.add_subscription("NodeB", "/parameter_events")
    b.add_subscription("NodeC", "/parameter_events")
    for i in range(3):
        t0 = 1000 * MS + i * 1000 * MS
        _a_to_b(b, t0, 1 * MS, stamp=400 + i)
        b.rmw_publish("NodeB", "/b_out", t0 + 2 * MS + 10, 500 + i, tid=B)
        b.rmw_take("NodeC", "/b_out", t0 + 4 * MS - 1, 500 + i, tid=C)
        b.invoke(b.sub_callbacks[("NodeC", "/b_out")], t0 + 4 * MS, 1 * MS, tid=C)
        for node, tid in (("NodeB", B), ("NodeC", C)):
            b.rmw_take(node, "/parameter_events", t0 + 50 * MS - 1, 9000 + i, tid=tid)
            b.invoke(b.sub_callbacks[(node, "/parameter_events")], t0 + 50 * MS, 1 * MS, tid=tid)
    timeline = build_timeline(b.build())
    evidence = timeline_evidence(timeline, inputs={
        "NodeB": frozenset({"/a_out", "/x_out"}), "NodeC": frozenset({"/b_out"})})
    samples = hop_latency_samples(timeline, Hop("NodeA", "NodeB"), evidence=evidence)
    assert (len(samples.values_us), samples.unresolved) == (3, 0), (
        f"{samples.unresolved} /parameter_events callbacks counted as unresolved hop samples")
    e2e = e2e_latency_samples(timeline, build_chain(["NodeA", "NodeB", "NodeC"]),
                              evidence=evidence)
    assert (len(e2e.values_us), e2e.unresolved) == (3, 0), (
        f"{e2e.unresolved} /parameter_events callbacks counted as unresolved chain samples")


def test_ambiguous_callback_is_unresolved_not_paired_with_an_older_message():
    """After two takes the callback's message is unknown, so it gets no pair.

    The latest-input rule is for timers only. A subscription callback that
    takes it pairs with the message of the callback before it.
    """
    b = _chain_builder()
    t0 = 1000 * MS
    _a_to_b(b, t0, 1 * MS, stamp=1)                          # clean pair, 1 ms
    b.invoke(b.timer_callbacks["NodeA"], t0 + 10 * MS, 1 * MS, tid=A)
    b.rmw_publish("NodeA", "/a_out", t0 + 10 * MS + 10, 2, tid=A)
    b.rmw_publish("NodeA", "/a_out", t0 + 10 * MS + 20, 3, tid=A)
    b.rmw_take("NodeB", "/a_out", t0 + 12 * MS, 2, tid=B)
    b.rmw_take("NodeB", "/a_out", t0 + 12 * MS + 1, 3, tid=B)
    b.invoke(b.sub_callbacks[("NodeB", "/a_out")], t0 + 13 * MS, 1 * MS, tid=B)
    timeline = build_timeline(b.build())
    samples = hop_latency_samples(timeline, Hop("NodeA", "NodeB"))
    assert (samples.values_us, samples.unresolved) == ([1000.0], 1), (
        f"the ambiguous callback was paired with an older message: {samples.values_us}")
    e2e = e2e_latency_samples(timeline, build_chain(["NodeA", "NodeB"]))
    assert (len(e2e.values_us), e2e.unresolved) == (1, 1), (
        f"the chain walk paired the ambiguous callback: {e2e.values_us}")


def test_hop_without_identity_in_a_trace_with_identity_is_unresolved_not_timestamp_paired():
    """A to B is intra-process (no rmw events). A's /status and B's /x_out are not.

    The trace names messages, so the hop is judged by identity: its three
    activations are unresolved, never paired by timestamps instead (a review
    finding). An earlier version of the rule switched this hop to
    timestamps and reported three samples with ``unresolved: 0``.
    """
    b = _chain_builder()
    b.add_publisher("NodeA", "/status")
    for i in range(3):
        t0 = 1000 * MS + i * 1000 * MS
        b.invoke(b.timer_callbacks["NodeA"], t0, 1 * MS, tid=A)
        b.rmw_publish("NodeA", "/status", t0 + MS // 2, 600 + i, tid=A)
        b.invoke(b.sub_callbacks[("NodeB", "/a_out")], t0 + 3 * MS, 1 * MS, tid=B)
        b.rmw_take("NodeB", "/x_out", t0 + 500 * MS - 1, 700 + i, tid=B)
        b.invoke(b.sub_callbacks[("NodeB", "/x_out")], t0 + 500 * MS, 1 * MS, tid=B)
    timeline = build_timeline(b.build())
    samples = hop_latency_samples(timeline, Hop("NodeA", "NodeB"))
    assert (samples.pairing, samples.values_us, samples.unresolved) == (
        "message_identity", [], 3), (
        f"a hop without identity was paired by {samples.pairing}: "
        f"values={samples.values_us} unresolved={samples.unresolved}")
    assert samples.complete is False


def test_key_also_published_outside_a_callback_is_withdrawn():
    b = _chain_builder()
    b.add_publisher("NodeX", "/a_out")
    t0 = 1000 * MS
    _a_to_b(b, t0, 1 * MS, stamp=7)
    b.rmw_publish("NodeX", "/a_out", t0 + MS // 2, 7, tid=9)  # no callback runs on tid 9
    timeline = build_timeline(b.build())
    samples = hop_latency_samples(timeline, Hop("NodeA", "NodeB"))
    assert (samples.values_us, samples.unresolved) == ([], 1), (
        f"a key published twice was attributed to NodeA: {samples.values_us}")
    assert timeline.identity["duplicate_keys"] == 1


def test_topic_with_only_duplicate_keys_counts_unresolved_in_the_chain():
    b = _chain_builder()
    t0 = 1000 * MS
    for k in range(2):
        b.invoke(b.timer_callbacks["NodeA"], t0 + k * 10 * MS, 1 * MS, tid=A)
        b.rmw_publish("NodeA", "/a_out", t0 + k * 10 * MS + 10, 7, tid=A)
    b.rmw_take("NodeB", "/a_out", t0 + 20 * MS, 7, tid=B)
    b.invoke(b.sub_callbacks[("NodeB", "/a_out")], t0 + 20 * MS + 1, 1 * MS, tid=B)
    timeline = build_timeline(b.build())
    e2e = e2e_latency_samples(timeline, build_chain(["NodeA", "NodeB"]))
    assert e2e.pairing == "message_identity"
    assert (len(e2e.values_us), e2e.unresolved) == (0, 1), (
        "the take of a duplicated key vanished from the chain without an unresolved count")


def test_overlapping_invocations_of_one_callback_on_two_threads():
    """A reentrant callback runs on threads 2 and 3 at once."""
    b = _chain_builder()
    t0 = 1000 * MS
    b.invoke(b.timer_callbacks["NodeA"], t0, 1 * MS, tid=A)
    b.rmw_publish("NodeA", "/a_out", t0 + 10, 1, tid=A)
    b.rmw_publish("NodeA", "/a_out", t0 + 20, 2, tid=A)
    cb = b.sub_callbacks[("NodeB", "/a_out")]
    b.rmw_take("NodeB", "/a_out", t0 + 2 * MS - 1, 1, tid=2)
    b.invoke(cb, t0 + 2 * MS, 10 * MS, tid=2)                 # ends t0+12 ms
    b.rmw_take("NodeB", "/a_out", t0 + 3 * MS - 1, 2, tid=3)
    b.invoke(cb, t0 + 3 * MS, 2 * MS, tid=3)                  # ends t0+5 ms
    timeline = build_timeline(b.build())
    samples = hop_latency_samples(timeline, Hop("NodeA", "NodeB"))
    assert sorted(samples.values_us) == pytest.approx([1000.0, 2000.0]), (
        f"an overlapping invocation of a reentrant callback was lost: {samples.values_us}")
    assert timeline.unmatched_starts == 0
