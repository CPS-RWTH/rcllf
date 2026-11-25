# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Chain data is decided by the declared inputs of ``system.json``.

A channel no node is observed publishing was classified ``foreign`` and its
sink activations dropped out of coverage, so two valid samples plus one sample
of missing identity counted as complete. Whether a callback is chain data is
decided by the declared inputs of ``system.json`` (the rule), never
by what the trace recorded.

Each test fails with the message the defect produces.
"""

from __future__ import annotations

import json

from lf_eval.chain_latency import latency_failures, lfc_graph, stock_graph, walk_chain
from lf_eval.metrics.determinism import compute_ordering
from lf_eval.metrics.e2e_latency import E2ELatencyMetric, run_evidence
from lf_eval.model.chain import build_chain
from lf_eval.model.run import make_run
from lf_eval.model.timeline import build_timeline

from tests.test_chain_latency import (
    CORR, GID_A, STATIC, SYSTEM, TIMER, dispatch, ev, run_cb, setup_events)

GID_LOST = [9] * 16  # a publisher whose every publication is missing from the trace


def _lfc_mixed_events():
    """src -> mid: two traced releases, then a mid dispatch of an untraced gid.

    Dispatch d spans ``[ts, ts + 5]``. Expected by hand: d1 -> d2 is
    115 - 100 = 15 ns, d3 -> d4 is 315 - 300 = 15 ns, and d5 consumes
    ``(GID_LOST, 1)``, whose publication is not in the trace: unresolved.
    """
    events = []
    events += dispatch(1, 1, TIMER, 0, 100, pub=(GID_A, 1))
    events += dispatch(2, 2, CORR, 0, 110, GID_A, 1)
    events += dispatch(3, 1, TIMER, 10, 300, pub=(GID_A, 2))
    events += dispatch(4, 2, CORR, 10, 310, GID_A, 2)
    events += dispatch(5, 2, CORR, 20, 510, GID_LOST, 1)
    return events


def _assert_mixed_counted(w, what):
    assert w["unresolved"] == {"src->mid": 1} and w["not_chain_data"] == 0 and not w["complete"], (
        f"{what}: the activation of missing identity was excluded from coverage: "
        f"unresolved={w['unresolved']} not_chain_data={w['not_chain_data']} complete={w['complete']}")


def test_lfc_unobserved_gid_beside_valid_samples_is_unresolved_not_excluded():
    w = walk_chain(lfc_graph(_lfc_mixed_events(), STATIC, SYSTEM), ["src", "mid"])
    assert w["physical_ns"] == [115 - 100, 315 - 300]
    _assert_mixed_counted(w, "lfc")
    result = {"mode": "lfc", "chains": {"src->mid": {
        "physical": {"n": len(w["physical_ns"])}, "unresolved": w["unresolved"],
        "bound": {"direct_ns": 0}, "samples_outside_bound": 0}}}
    assert latency_failures(result) == ["unresolved:src->mid:1"], (
        f"the latency gate accepted a run with missing identity: {latency_failures(result)}")


def test_lfc_run_with_an_unobserved_gid_is_incomplete_and_ordering_incomplete(tmp_path):
    run_dir = tmp_path / "20261102T000000Z_lfc_default_r0"
    run_dir.mkdir()
    (run_dir / "system.json").write_text(json.dumps(SYSTEM))
    (run_dir / "rcllf_trace_manifest.json").write_text(json.dumps(STATIC))
    lines = [json.dumps({"stats": {}})] + [json.dumps(e.__dict__) for e in _lfc_mixed_events()]
    (run_dir / "semantic_events.jsonl").write_text("\n".join(lines) + "\n")
    chain = build_chain(["src", "mid"], name="c")
    run = make_run(run_dir.name, build_timeline([]), mode="lfc", path=run_dir)
    rec = E2ELatencyMetric().compute([run], chains=[chain]).data["per_run"][run_dir.name]["chains"]["c"]
    hop = rec["hops"]["src->mid"]
    got = (hop["paired"], hop["unresolved"], hop.get("not_chain_data"), rec["complete"])
    assert got == (2, 1, 0, False), (
        f"missing identity disappeared from coverage: paired, unresolved, not_chain_data, complete = {got}")
    order = compute_ordering(run.timeline, chain, evidence=run_evidence(run))
    assert order.verdict == "INCOMPLETE", f"ordering {order.verdict} with a missing-identity activation"


def test_stock_declared_input_whose_publisher_is_never_seen_is_unresolved():
    """The sink's declared input /c: gate's publisher init and publishes are all lost.

    No node of the trace publishes /c, which made the channel foreign. Two
    valid src -> mid samples (220 - 100, 420 - 300) stay, and the sink walk
    counts its activation unresolved.
    """
    events = [e for e in setup_events()
              if not (e.name == "rcl_publisher_init" and e.get("topic_name") == "/c")]
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += run_cb(20, 200, 220, tid=2, take=(201, 111))
    events += run_cb(10, 300, 310, pub=(501, 333))
    events += run_cb(20, 400, 420, tid=2, take=(201, 333))
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))      # gate's /c, never traced
    events.sort(key=lambda e: e.timestamp_ns)
    graph = stock_graph(events, SYSTEM)
    assert walk_chain(graph, ["src", "mid"])["physical_ns"] == [220 - 100, 420 - 300]
    w = walk_chain(graph, ["src", "mid", "gate", "sink"])
    assert w["unresolved"] == {"gate->sink": 1} and w["not_chain_data"] == 0, (
        f"stock: the activation of missing identity was excluded from coverage: "
        f"unresolved={w['unresolved']} not_chain_data={w['not_chain_data']}")


def test_stock_undeclared_subscription_is_not_chain_data_per_topic():
    """Under the declared rule: sink /parameter_events callbacks are
    no chain samples, whether or not their publication was traced."""
    events = setup_events() + [
        ev("rcl_subscription_init", 0, subscription_handle=410, node_handle=4,
           rmw_subscription_handle=411, topic_name="/parameter_events"),
        ev("rclcpp_subscription_init", 0, subscription_handle=410, subscription=412),
        ev("rclcpp_subscription_callback_added", 0, subscription=412, callback=41)]
    events += run_cb(41, 1200, 1210, tid=4, take=(411, 55))
    events += run_cb(41, 1300, 1310, tid=4)
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert (w["unresolved"], w["not_chain_data_topics"]) == ({}, {"/parameter_events": 2}), (
        f"undeclared subscription counted as chain data: {w['unresolved']} {w.get('not_chain_data_topics')}")


def _system_with_unresolved_sink_input():
    """sink declares /c and a second input written ``d`` in its source, which
    system.json resolved for no topic: the node's topic set would be {/c}."""
    system = json.loads(json.dumps(SYSTEM))
    system["nodes"]["sink"]["subscribers"].append({"topic": "d", "callback": "on_d"})
    return system


def test_node_with_an_unresolved_subscriber_has_unknown_inputs():
    """A node listed under one resolved topic while another
    of its subscribers resolved nowhere got the partial set {/c}, so a callback
    of its real input /d was excluded as undeclared."""
    from lf_eval.chain_latency import declared_inputs
    inputs = declared_inputs(_system_with_unresolved_sink_input())
    assert "sink" not in inputs, (
        f"a partial input set was used to exclude callbacks: sink -> {sorted(inputs['sink'])}")
    assert inputs["mid"] == frozenset({"/a"})


def test_stock_callback_of_an_unresolved_declared_input_is_unresolved():
    events = setup_events() + [
        ev("rcl_subscription_init", 0, subscription_handle=420, node_handle=4,
           rmw_subscription_handle=421, topic_name="/d"),
        ev("rclcpp_subscription_init", 0, subscription_handle=420, subscription=422),
        ev("rclcpp_subscription_callback_added", 0, subscription=422, callback=42)]
    events += run_cb(42, 1200, 1210, tid=4, take=(421, 55))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, _system_with_unresolved_sink_input()),
                   ["src", "mid", "gate", "sink"])
    assert (w["unresolved"], w["not_chain_data"]) == ({"gate->sink": 1}, 0), (
        f"stock: a callback of a declared but unresolved input was excluded as not chain data: "
        f"unresolved={w['unresolved']} not_chain_data_topics={w.get('not_chain_data_topics')}")
