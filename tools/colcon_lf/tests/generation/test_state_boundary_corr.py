# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A node loop of zero-delay edges is admitted unless it is a causality loop.

The brake_fusion plant loop (simulator -> sensors -> fusion -> controller ->
simulator) is a zero-delay feedback loop. LF allows one as long as there is no
causality loop: a d = 0 in-process edge delivers at its publication tag, so the
loop with the latch declared after the step is admitted with same-tag delivery,
and the loop with the latch declared before the step is a causality loop that
generation refuses, naming its links. No edge is silently given a microstep.
"""

from __future__ import annotations

import pytest

from colcon_lf.exec_config.policies import REGISTRY, KEY_REQUIRE_ADMITTED

from .test_correspondence_emission import (
    MSG, CallbackInfo, NodeInfo, PublisherInfo, SubscriberInfo, TimerInfo, _graph, _pub, _sub,
    _tc, generate_declared,
)
from ..corr_declared import declared


def _plant(latch_publishes: bool = False) -> NodeInfo:
    return NodeInfo(
        name="sim", class_name="SimNode", package="test_pkg", source_file="/fake/sim.cpp",
        callbacks=[CallbackInfo(name="step", publishers_used=["/state"]),
                   CallbackInfo(name="on_cmd",
                                publishers_used=["/echo"] if latch_publishes else [])],
        timers=[TimerInfo(period_ms=1.0, callback="step")],
        publishers=[PublisherInfo(topic=t, message_type=MSG, created_in="constructor",
                                  unconditional=True) for t in ("/state", "/echo")],
        subscribers=[SubscriberInfo(topic="/cmd", message_type=MSG, callback="on_cmd")],
    )


def _loop(latch_publishes: bool = False):
    """sim /state -> sensor (latch, timer publishes /obs) -> ctl (latch, timer /cmd) -> sim."""
    sensor = NodeInfo(
        name="sensor", class_name="SensorNode", package="test_pkg",
        source_file="/fake/sensor.cpp",
        callbacks=[CallbackInfo(name="on_state"),
                   CallbackInfo(name="tick", publishers_used=["/obs"])],
        timers=[TimerInfo(period_ms=10.0, callback="tick")],
        publishers=[PublisherInfo(topic="/obs", message_type=MSG, created_in="constructor",
                                  unconditional=True)],
        subscribers=[SubscriberInfo(topic="/state", message_type=MSG, callback="on_state")],
    )
    ctl = NodeInfo(
        name="ctl", class_name="CtlNode", package="test_pkg", source_file="/fake/ctl.cpp",
        callbacks=[CallbackInfo(name="on_obs"),
                   CallbackInfo(name="tick", publishers_used=["/cmd"])],
        timers=[TimerInfo(period_ms=25.0, callback="tick")],
        publishers=[PublisherInfo(topic="/cmd", message_type=MSG, created_in="constructor",
                                  unconditional=True)],
        subscribers=[SubscriberInfo(topic="/obs", message_type=MSG, callback="on_obs")],
    )
    nodes = [ctl, _plant(latch_publishes), sensor, _sub("echo_sink", "EchoNode", "/echo")]
    # /cmd first: it is admitted before the loop exists, /obs is the edge
    # that would close it.
    topics = [_tc("/cmd", ["ctl"], ["sim"]), _tc("/state", ["sim"], ["sensor"]),
              _tc("/obs", ["sensor"], ["ctl"])]
    if latch_publishes:
        topics.append(_tc("/echo", ["sim"], ["echo_sink"]))
    return _graph(nodes, topics)


_ALL_SUBS_FIRST = {"ctl": "subscriptions_first", "sensor": "subscriptions_first"}


def _status(graph, order, **kw):
    system = generate_declared(graph, target_lfc=True, reaction_order=order,
                               corr_bounds=declared(graph, 1, **kw))
    return {e.topic: (e.status, e.reason) for e in system.corr_edges}


def test_step_before_latch_admits_the_whole_loop():
    got = _status(_loop(), {**_ALL_SUBS_FIRST, "sim": "timers_first"})
    assert got["/obs"] == ("admitted", None), (
        f"/obs closes no reaction cycle when sim's step precedes its /cmd latch: {got}")
    assert {v for v in got.values()} == {("admitted", None)}, got


def test_latch_edge_judged_last_is_admitted_too():
    # /cmd into the latch is judged after the rest of the loop is admitted:
    # sim reaches ctl, but the edge into a state boundary closes no cycle.
    graph = _loop()
    tcs = graph.communication.topics
    graph.communication.topics = tcs[1:] + tcs[:1]
    got = _status(graph, {**_ALL_SUBS_FIRST, "sim": "timers_first"})
    assert got["/cmd"] == ("admitted", None), got


def test_latch_before_step_is_a_causality_loop():
    # Under subscriptions_first the sim's /cmd latch is declared
    # before its step, which publishes /state, so every subscription on the
    # d = 0 loop reaches the next link at its tag: a causality loop, refused
    # at generation as lfc would (Cyclic dependency), with no implicit microstep.
    with pytest.raises(ValueError, match=r"zero-delay causality loop: .*"
                       r"/cmd \(ctl -> sim\), /obs \(sensor -> ctl\), /state \(sim -> sensor\)"):
        _status(_loop(), {**_ALL_SUBS_FIRST, "sim": "subscriptions_first"})
    # One link at d > 0 breaks it.
    graph = _loop()
    graph.communication.topics[0].after_delay_ns = 1_000_000
    got = _status(graph, {**_ALL_SUBS_FIRST, "sim": "subscriptions_first"})
    assert {v for v in got.values()} == {("admitted", None)}, got


def test_a_latch_that_publishes_is_admitted_too():
    # on_cmd publishing /echo, a topic off the loop, at the
    # delivery tag reaches no link of the loop, so there is no causality loop.
    got = _status(_loop(latch_publishes=True), {**_ALL_SUBS_FIRST, "sim": "timers_first"})
    assert got["/obs"] == ("admitted", None), got
    assert {v for v in got.values()} == {("admitted", None)}, got


def test_require_admitted_turns_a_refusal_into_an_error():
    graph = _loop()
    bounds = declared(graph, 1)
    bounds.require_admitted = True
    del bounds.burst["sensor"]  # /obs loses its publisher burst bound
    with pytest.raises(ValueError, match=r"require_admitted: .*/obs sensor -> ctl: burst_unbounded"):
        generate_declared(graph, target_lfc=True, reaction_order=_ALL_SUBS_FIRST,
                          corr_bounds=bounds)
    # Every edge admitted: no error.
    generate_declared(graph, target_lfc=True, corr_bounds=declared(graph, 1, require_admitted=True),
                      reaction_order=_ALL_SUBS_FIRST)
    assert REGISTRY[KEY_REQUIRE_ADMITTED].default is False


def _relay_graph():
    """relay's on_ext (fed from outside the system) publishes /out to sink."""
    return _graph([_sub("relay", "RelayNode", "/ext", publishes="/out"),
                   _sub("sink", "SinkNode", "/out")],
                  [_tc("/out", ["relay"], ["sink"])])


def test_require_admitted_refuses_a_declared_input_period_that_sizes_no_bound():
    # The binding checks only the
    # min_input_period_ms declarations a bound was sized from, so under
    # require_admitted any other declaration is a generation error that names
    # the node, the topic and the reason.
    graph = _relay_graph()
    bounds = declared(graph, 1, min_input_period_ms={("relay", "/ext"): 10.0})
    bounds.require_admitted = True
    # The declaration sizes /out: no error, the edge is admitted.
    system = generate_declared(graph, target_lfc=True, corr_bounds=bounds)
    assert [(e.topic, e.status) for e in system.corr_edges] == [("/out", "admitted")]
    # sink's /out input feeds no publisher, so its declaration sizes nothing.
    bounds.min_input_period_ms[("sink", "/out")] = 5.0
    with pytest.raises(ValueError, match=(
            r"require_admitted: min_input_period_ms declaration\(s\) size no bound; "
            r"sink:/out \(min_input_period_ms=5.0\): no admitted "
            r"correspondence edge is sized from this input")):
        generate_declared(graph, target_lfc=True, corr_bounds=bounds)
    del bounds.min_input_period_ms[("sink", "/out")]
    bounds.min_input_period_ms[("ghost", "/ext")] = 5.0
    with pytest.raises(ValueError, match=r"ghost:/ext .*no node of that key"):
        generate_declared(graph, target_lfc=True, corr_bounds=bounds)
    del bounds.min_input_period_ms[("ghost", "/ext")]
    bounds.min_input_period_ms[("relay", "/nope")] = 5.0
    with pytest.raises(ValueError, match=r"relay:/nope .*no subscription on that topic"):
        generate_declared(graph, target_lfc=True, corr_bounds=bounds)
    # Without require_admitted an unused declaration stays allowed.
    bounds.require_admitted = False
    generate_declared(graph, target_lfc=True, corr_bounds=bounds)
