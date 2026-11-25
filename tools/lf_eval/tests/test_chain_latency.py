# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Message-exact chain latency (W6)."""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

from lf_eval.chain_latency import (
    declared_bound, in_callback, latency_failures, lfc_graph, manifest_delays, outside_bound, percentile,
    stock_graph, walk_chain)
from lf_eval.reader.events import TraceEvent
from lf_eval.semantic import schema
from lf_eval.semantic.reader import SemEvent

# src (timer, 10 ms) -> mid (subscription) -> gate (timer 20 ms, gated) -> sink
SYSTEM = {"nodes": {
    "src": {"class_name": "Src", "timers": [{"callback": "tick", "period_ms": 10}], "subscribers": []},
    "mid": {"class_name": "Mid", "timers": [], "subscribers": [{"topic": "/a", "callback": "on_a"}]},
    "gate": {"class_name": "Gate", "timers": [{"callback": "tick", "period_ms": 20}],
             "subscribers": [{"topic": "/b", "callback": "on_b"}]},
    "sink": {"class_name": "Sink", "timers": [], "subscribers": [{"topic": "/c", "callback": "on_c"}]},
}, "communication": {"topics": [
    {"topic": "/a", "publishers": ["src"], "subscribers": ["mid"]},
    {"topic": "/b", "publishers": ["mid"], "subscribers": ["gate"]},
    {"topic": "/c", "publishers": ["gate"], "subscribers": ["sink"]},
]}}


def ev(name, ts, tid=1, **fields):
    return TraceEvent(name=name, timestamp_ns=ts, fields=fields, pid=7, tid=tid)


def setup_events():
    """Init events: nodes 1..4, src timer cb 10, mid sub cb 20, gate timer 30 + sub 31, sink 40."""
    out = []
    for h, name in ((1, "src"), (2, "mid"), (3, "gate"), (4, "sink")):
        out.append(ev("rcl_node_init", 0, node_handle=h, node_name=name))
    out += [ev("rclcpp_timer_link_node", 0, timer_handle=100, node_handle=1),
            ev("rclcpp_timer_callback_added", 0, timer_handle=100, callback=10),
            ev("rclcpp_timer_link_node", 0, timer_handle=300, node_handle=3),
            ev("rclcpp_timer_callback_added", 0, timer_handle=300, callback=30)]
    for sub, node, rmw, obj, cb, topic in ((200, 2, 201, 202, 20, "/a"), (310, 3, 311, 312, 31, "/b"),
                                           (400, 4, 401, 402, 40, "/c")):
        out += [ev("rcl_subscription_init", 0, subscription_handle=sub, node_handle=node,
                   rmw_subscription_handle=rmw, topic_name=topic),
                ev("rclcpp_subscription_init", 0, subscription_handle=sub, subscription=obj),
                ev("rclcpp_subscription_callback_added", 0, subscription=obj, callback=cb)]
    for pub, node, topic in ((501, 1, "/a"), (502, 2, "/b"), (503, 3, "/c")):
        out.append(ev("rcl_publisher_init", 0, node_handle=node, rmw_publisher_handle=pub,
                      topic_name=topic))
    return out


def run_cb(cb, start, end, tid=1, take=None, pub=None, is_intra=None):
    out = []
    if take:
        out.append(ev("rmw_take", start - 1, tid, taken=1, rmw_subscription_handle=take[0],
                      source_timestamp=take[1]))
    flag = {} if is_intra is None else {"is_intra_process": is_intra}
    out.append(ev("callback_start", start, tid, callback=cb, **flag))
    if pub:
        out.append(ev("rmw_publish", start + 1, tid, rmw_publisher_handle=pub[0], timestamp=pub[1]))
    out.append(ev("callback_end", end, tid, callback=cb))
    return out


def test_stock_walk_follows_source_timestamps_through_a_gated_hop():
    events = setup_events()
    # two src releases; mid forwards each; the gate timer at 1000 samples the newer one
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += run_cb(10, 300, 310, pub=(501, 333))
    events += run_cb(20, 400, 420, tid=2, take=(201, 111), pub=(502, 1111))   # older message first
    events += run_cb(31, 430, 440, tid=3, take=(311, 1111))
    events += run_cb(20, 500, 520, tid=2, take=(201, 333), pub=(502, 3333))
    events += run_cb(31, 530, 540, tid=3, take=(311, 3333))
    events += run_cb(30, 1000, 1010, tid=3, pub=(503, 7))
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["unresolved"] == {}
    # the sink ran at 1100..1150; the newest src data at the gate came from the release at 300
    assert w["physical_ns"] == [1150 - 300]
    assert w["gated_samples"] == 1


def test_stock_walk_does_not_attribute_a_publish_to_another_thread():
    """A falsification: owning publishes by time alone would credit src at 300 for /a 111."""
    events = setup_events()
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += run_cb(10, 300, 310, tid=9)            # src runs on another thread, publishes nothing
    events += run_cb(20, 400, 420, tid=2, take=(201, 111), pub=(502, 1111))
    events += run_cb(31, 430, 440, tid=3, take=(311, 1111))
    events += run_cb(30, 1000, 1010, tid=3, pub=(503, 7))
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["physical_ns"] == [1150 - 100]


def test_stock_sink_whose_message_was_never_traced_is_unresolved():
    events = setup_events() + run_cb(40, 1100, 1150, tid=4, take=(401, 99))
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["physical_ns"] == []
    assert w["unresolved"] == {"gate->sink": 1}, (
        f"a declared input with no traced publication was not counted unresolved: "
        f"unresolved={w['unresolved']} not_chain_data={w['not_chain_data']}")
    assert w["not_chain_data"] == 0


def _with_parameter_events_sub(events):
    """The sink also subscribes /parameter_events (callback 41), as every rclcpp node does."""
    return events + [
        ev("rcl_subscription_init", 0, subscription_handle=410, node_handle=4,
           rmw_subscription_handle=411, topic_name="/parameter_events"),
        ev("rclcpp_subscription_init", 0, subscription_handle=410, subscription=412),
        ev("rclcpp_subscription_callback_added", 0, subscription=412, callback=41)]


def test_stock_sink_parameter_events_callback_is_not_an_unresolved_sample():
    """A sink's /parameter_events callback is no chain output. The old
    walk counted it as an unresolved gate->sink sample."""
    events = _with_parameter_events_sub(setup_events())
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += run_cb(20, 200, 220, tid=2, take=(201, 111), pub=(502, 1111))
    events += run_cb(31, 230, 240, tid=3, take=(311, 1111))
    events += run_cb(30, 1000, 1010, tid=3, pub=(503, 7))
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))
    events += run_cb(41, 1200, 1210, tid=4, take=(411, 55))     # /parameter_events
    events += run_cb(41, 1300, 1310, tid=4, take=(411, 56))     # /parameter_events
    events += run_cb(41, 1320, 1330, tid=4)                     # /parameter_events, take untraced
    events += run_cb(40, 1400, 1450, tid=4, take=(401, 8))      # untraced /c: a real miss
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["unresolved"] == {"gate->sink": 1}, (
        f"/parameter_events callbacks counted as unresolved hop samples: {w['unresolved']}")
    assert w["not_chain_data_topics"] == {"/parameter_events": 3}
    assert w["physical_ns"] == [1150 - 100]
    unresolved = sum(w["unresolved"].values())
    assert w["sinks"] == len(w["physical_ns"]) + unresolved + w["not_chain_data"]


def test_stock_undeclared_input_is_excluded_even_when_its_topic_is_published():
    """The rule is the system model, not the trace: a sink callback on a topic the
    sink does not declare is no chain output even if a chain node publishes it."""
    events = setup_events() + [
        ev("rcl_subscription_init", 0, subscription_handle=420, node_handle=4,
           rmw_subscription_handle=421, topic_name="/b"),
        ev("rclcpp_subscription_init", 0, subscription_handle=420, subscription=422),
        ev("rclcpp_subscription_callback_added", 0, subscription=422, callback=42)]
    events += run_cb(20, 200, 220, tid=2, pub=(502, 1111))
    events += run_cb(42, 300, 310, tid=4, take=(421, 1111))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["not_chain_data_topics"] == {"/b": 1}
    assert w["unresolved"] == {}


def test_lfc_sink_input_from_an_untraced_publisher_is_unresolved():
    """lfc: a sink CORR_SUB dispatch serves a manifest edge, a declared input by
    construction. A publication missing from the trace is unresolved, whatever its gid."""
    events = lfc_events(30)
    events += dispatch(9, 5, CORR, 40, 500, [9] * 16, 1)        # gid never traced
    events += dispatch(10, 5, CORR, 50, 600, GID_C, 2)          # gate's gid, ordinal never traced
    w = walk_chain(lfc_graph(events, STATIC, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["unresolved"] == {"gate->sink": 2}, (
        f"an lfc input whose publication is not in the trace was not counted unresolved: "
        f"unresolved={w['unresolved']} not_chain_data={w['not_chain_data']}")
    assert w["not_chain_data"] == 0
    assert len(w["physical_ns"]) == 1


GID_A, GID_B, GID_C = [1] * 16, [2] * 16, [3] * 16
STATIC = {"rids": [{"rid": 1, "node": "Src", "callback": "tick"}, {"rid": 2, "node": "Mid", "callback": "on_a"},
                   {"rid": 3, "node": "Gate", "callback": "tick"}, {"rid": 4, "node": "Gate", "callback": "on_b"},
                   {"rid": 5, "node": "Sink", "callback": "on_c"}]}
DELAYS = {"src->mid": 0, "mid->gate": 0, "gate->sink": 0}
TIMER, CORR = schema.ENTRY_KINDS["TIMER"], schema.ENTRY_KINDS["CORR_SUB"]


def dispatch(did, rid, kind, tag, ts, gid=(), ordinal=0, pub=None, join_wait=0):
    def sem(name, t, **f):
        return SemEvent(name=name, ts=t, epoch=0, cat=0, seq=0, fields=f)
    out = [sem("dispatch_begin", ts, dispatch_id=did, rid=rid, entry_kind=kind, edge=0, gid=list(gid),
               ordinal=ordinal, tag_time=tag, tag_microstep=0)]
    if pub:
        out.append(sem("publication", ts + 1, gid=list(pub[0]), ordinal=pub[1], dispatch_id=did, rid=rid,
                       tag_time=tag, tag_microstep=0, result=0, api=0, origin=0))
    out.append(sem("dispatch_end", ts + 5, dispatch_id=did, outcome=1, retained_id=0, join_wait_ns=join_wait,
                   callback_ran=1))
    return out


def lfc_events(sink_tag):
    events = []
    events += dispatch(1, 1, TIMER, 0, 100, pub=(GID_A, 1))
    events += dispatch(2, 2, CORR, 0, 110, GID_A, 1, pub=(GID_B, 1))
    events += dispatch(3, 4, CORR, 0, 120, GID_B, 1)
    events += dispatch(4, 1, TIMER, 10, 200, pub=(GID_A, 2))
    events += dispatch(5, 2, CORR, 10, 210, GID_A, 2, pub=(GID_B, 2))
    events += dispatch(6, 4, CORR, 10, 220, GID_B, 2)
    events += dispatch(7, 3, TIMER, 20, 300, pub=(GID_C, 1))
    events += dispatch(8, 5, CORR, sink_tag, 400, GID_C, 1)
    return events


def test_lfc_walk_uses_publication_identity_and_reports_logical_latency():
    w = walk_chain(lfc_graph(lfc_events(20), STATIC, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["unresolved"] == {}
    assert w["logical_ns"] == [20 - 10]
    assert w["physical_ns"] == [405 - 200]


def test_bound_is_exact_for_direct_samples_and_adds_the_source_period_for_gated_ones():
    bound = declared_bound(SYSTEM, ["src", "mid", "gate", "sink"], DELAYS)
    assert (bound["direct_ns"], bound["gated_ns"]) == (0, 10_000_000)
    assert outside_bound(bound, [0, 10_000_000], [False, True]) == []
    # a direct sample one tag late, a gated one past the source period
    assert outside_bound(bound, [1, 10_000_001], [False, True]) == [(1, False), (10_000_001, True)]


def test_bound_sums_the_declared_edge_delays_and_refuses_an_undeclared_hop():
    static = {"edges": [{"status": "admitted", "delay_ns": 1_000_000, "publisher": {"node": a},
                         "subscriber": {"node": b}} for a, b in (("src", "mid"), ("mid", "gate"))]
              + [{"status": "legacy", "delay_ns": 5, "publisher": {"node": "gate"}, "subscriber": {"node": "sink"}}]}
    delays = manifest_delays(static)
    assert declared_bound(SYSTEM, ["src", "mid", "gate"], delays)["direct_ns"] == 2_000_000
    bound = declared_bound(SYSTEM, ["src", "mid", "gate", "sink"], delays)
    assert bound["direct_ns"] is None and bound["hops_without_edge"] == ["gate->sink"]
    assert outside_bound(bound, [2_000_000], [False]) == [(2_000_000, False)]


def test_lfc_sample_past_the_bound_is_reported():
    """A falsification: a sink dispatched one source period too late must fail the bound."""
    w = walk_chain(lfc_graph(lfc_events(20 + 10_000_000), STATIC, SYSTEM), ["src", "mid", "gate", "sink"])
    bound = declared_bound(SYSTEM, ["src", "mid", "gate", "sink"], DELAYS)
    assert w["gated"] == [True]
    assert outside_bound(bound, w["logical_ns"], w["gated"]) == [(10_000_010, True)]


def test_percentile_is_nearest_rank():
    assert percentile(list(range(1, 101)), 99) == 99.0
    assert percentile([5], 50) == 5.0
    assert percentile([], 50) is None


def test_in_callback_time_sums_every_completed_callback_over_the_traced_span():
    events = setup_events()
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += run_cb(20, 400, 420, tid=2, take=(201, 111))
    events += run_cb(99, 500, 530, tid=5)            # no chain node: still CPU work
    events += run_cb(40, 1100, 1150, tid=4)
    events.sort(key=lambda e: e.timestamp_ns)
    got = in_callback(stock_graph(events, SYSTEM))
    assert (got["activations"], got["busy_ns"], got["span_ns"]) == (4, 10 + 20 + 30 + 50, 1150 - 100)


def test_lfc_in_callback_time_excludes_the_blocking_join_wait():
    """A falsification: counting the join wait as CPU would give 5 ns, not 2."""
    graph = lfc_graph(dispatch(1, 2, CORR, 0, 100, GID_A, 1, join_wait=3), STATIC, SYSTEM)
    assert in_callback(graph)["busy_ns"] == 2


def test_stock_sink_whose_consumed_message_was_never_published_in_the_trace_is_unresolved():
    """A falsification: the old fallback credited the untraced /a 222 to the src release at 100."""
    events = setup_events()
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += run_cb(20, 200, 220, tid=2, take=(201, 111))
    events += run_cb(20, 500, 520, tid=2, take=(201, 222))   # its publication was never traced
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid"])
    assert w["physical_ns"] == [220 - 100], f"wrong-latency sample attributed: {w['physical_ns']}"
    assert w["unresolved"] == {"src->mid": 1}


def test_a_traced_message_of_another_input_keeps_the_state_hop():
    events = []
    events += dispatch(1, 1, TIMER, 0, 100, pub=(GID_A, 1))
    events += dispatch(2, 2, CORR, 0, 110, GID_A, 1, pub=(GID_B, 1))
    events += dispatch(3, 4, CORR, 0, 120, GID_B, 1)           # gate consumes mid's message
    events += dispatch(4, 1, TIMER, 10, 200, pub=(GID_A, 2))
    events += dispatch(5, 4, CORR, 10, 210, GID_A, 2)          # gate consumes src's message, another input
    w = walk_chain(lfc_graph(events, STATIC, SYSTEM), ["src", "mid", "gate"])
    assert w["unresolved"] == {}
    # the second gate dispatch carries mid's last message, so the src release at 100
    assert w["physical_ns"] == [125 - 100, 215 - 100] and w["gated"] == [False, True]


def _edge(a, b, delay, topic):
    return {"status": "admitted", "delay_ns": delay, "topic": topic, "publisher": {"node": a},
            "subscriber": {"node": b}}


def test_parallel_edges_with_one_delay_keep_the_bound():
    static = {"edges": [_edge("src", "mid", 1_000_000, "/a"), _edge("src", "mid", 1_000_000, "/a2"),
                        _edge("mid", "gate", 0, "/b"), _edge("gate", "sink", 0, "/c")]}
    bound = declared_bound(SYSTEM, ["src", "mid", "gate", "sink"], manifest_delays(static))
    assert bound["direct_ns"] == 1_000_000 and bound["ambiguous_hops"] == []


def test_parallel_edges_with_different_delays_refuse_the_bound():
    """A falsification: keyed by node pair, the last edge's 0 ns silently replaced the first edge's 1 ms."""
    static = {"edges": [_edge("src", "mid", 1_000_000, "/a"), _edge("src", "mid", 0, "/a2"),
                        _edge("mid", "gate", 0, "/b"), _edge("gate", "sink", 0, "/c")]}
    bound = declared_bound(SYSTEM, ["src", "mid", "gate", "sink"], manifest_delays(static))
    assert bound["direct_ns"] is None, f"bound {bound['direct_ns']} from one of two parallel edges"
    assert bound["ambiguous_hops"] == ["src->mid"] and bound["gated_ns"] is None
    assert outside_bound(bound, [0], [False]) == [(0, False)]


def _result(mode, n=1, outside=0, direct=0):
    row = {"physical": {"n": n}, "unresolved": {}, "sinks": n}
    if mode == "lfc":
        row.update(samples_outside_bound=outside,
                   bound={"direct_ns": direct, "hops_without_edge": [], "ambiguous_hops": []})
    return {"mode": mode, "chains": {"a->b": row}}


def test_latency_failures_cover_outside_bound_no_bound_and_no_samples():
    assert latency_failures(_result("lfc")) == []
    assert latency_failures(_result("stock")) == []
    assert latency_failures(_result("lfc", outside=3)) == ["outside_bound:a->b:3"]
    assert latency_failures(_result("lfc", direct=None)) == ["no_bound:a->b:missing=[]:ambiguous=[]"]
    assert latency_failures(_result("stock", n=0)) == ["no_samples:a->b"]


def test_chain_latency_cli_exits_1_on_samples_outside_the_bound(tmp_path, monkeypatch, capsys):
    """A falsification: the command printed the outside count and exited 0."""
    from lf_eval import chain_latency as cl
    from lf_eval.cli import main

    def fake(run_dir, chains, mode):
        r = _result("lfc", outside=2)
        r["chains"]["a->b"]["physical"].update(p50_ns=1.0, p99_ns=1.0, max_ns=1.0)
        r["failures"] = latency_failures(r)
        return r

    monkeypatch.setattr(cl, "chain_latency", fake)
    out = tmp_path / "lat.json"
    assert main(["chain-latency", str(tmp_path), "--mode", "lfc", "--chain", "a->b", "-o", str(out)]) == 1
    assert json.loads(out.read_text())["failures"] == ["outside_bound:a->b:2"]
    assert "FAIL outside_bound:a->b:2" in capsys.readouterr().err


def test_federated_rids_map_per_federate_executable():
    """Reaction ids restart in every federate binary, so rid 1 of the
    publisher federate and rid 1 of the subscriber federate are different nodes.
    The epoch's ``run_begin`` names the federate executable that owns its rids."""
    static = {"federated": True,
              "rids": [{"rid": 1, "node": "Src", "callback": "tick", "federate": "pub"},
                       {"rid": 1, "node": "Mid", "callback": "on_a", "federate": "sub"}]}

    def sem(name, t, epoch, **f):
        return SemEvent(name=name, ts=t, epoch=epoch, cat=0, seq=0, fields=f)

    events = [sem("run_begin", 1, 11, executable="/ws/lib/federate__pub"),
              sem("run_begin", 2, 22, executable="/ws/lib/federate__sub")]
    for epoch, did, kind, tag, ts, gid, ordinal, pub in (
            (11, 1, TIMER, 0, 100, (), 0, (GID_A, 1)),
            (22, 1, CORR, 5, 150, GID_A, 1, None)):
        events.append(sem("dispatch_begin", ts, epoch, dispatch_id=did, rid=1, entry_kind=kind, edge=0,
                          gid=list(gid), ordinal=ordinal, tag_time=tag, tag_microstep=0))
        if pub:
            events.append(sem("publication", ts + 1, epoch, gid=list(pub[0]), ordinal=pub[1], dispatch_id=did,
                              rid=1, tag_time=tag, tag_microstep=0, result=0, api=0, origin=0))
        events.append(sem("dispatch_end", ts + 5, epoch, dispatch_id=did, outcome=1, retained_id=0,
                          join_wait_ns=0, callback_ran=1))
    w = walk_chain(lfc_graph(events, static, SYSTEM), ["src", "mid"])
    assert w["unresolved"] == {}, f"federate rids were mixed up: unresolved {w['unresolved']}"
    assert w["logical_ns"] == [5] and w["physical_ns"] == [155 - 100]


# --- Declared inputs are resolved identities ---

def _sink_system(declared: str, resolved: str, **sub_extra) -> dict:
    """SYSTEM with the sink's input declared as written in the source (``declared``)
    and listed by colcon_lf under its resolved name (``resolved``)."""
    system = json.loads(json.dumps(SYSTEM))
    system["nodes"]["sink"]["subscribers"] = [dict({"topic": declared, "callback": "on_c"}, **sub_extra)]
    system["communication"]["topics"][2]["topic"] = resolved
    return system


def _sink_input_events(trace_topic):
    """The sink's only subscription is traced as ``trace_topic`` (None: no name);
    the gate publishes on it, but the sink's take carries an untraced stamp."""
    events = [e for e in setup_events()
              if not (e.name == "rcl_subscription_init" and e.get("subscription_handle") == 400)]
    fields = dict(subscription_handle=400, node_handle=4, rmw_subscription_handle=401)
    if trace_topic is not None:
        fields["topic_name"] = trace_topic
    events.insert(0, ev("rcl_subscription_init", 0, **fields))
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 99))
    return sorted(events, key=lambda e: e.timestamp_ns)


def _assert_unresolved_not_excluded(w, what):
    assert w["not_chain_data"] == 0 and w["unresolved"] == {"gate->sink": 1}, (
        f"{what}: a declared sink input was excluded as not_chain_data "
        f"{w['not_chain_data_topics']} instead of counted unresolved {w['unresolved']}")


def test_relative_declared_input_matches_its_resolved_trace_name():
    """The real self_loop fixture declares ``self_count`` and colcon_lf resolves it
    to ``/self_count``, the name rcl traces. Raw-name matching excluded it."""
    from pathlib import Path

    from lf_eval.chain_latency import declared_inputs
    fixture = (Path(__file__).resolve().parents[2] / "colcon_lf" / "tests" / "fixtures"
               / "self_loop_system.json")
    system = json.loads(fixture.read_text())
    assert system["nodes"]["self_loop_node"]["subscribers"][0]["topic"] == "self_count"
    assert declared_inputs(system)["self_loop_node"] == frozenset({"/self_count"}), (
        f"relative input not resolved: {declared_inputs(system)}")
    w = walk_chain(stock_graph(_sink_input_events("/c"), _sink_system("c", "/c")),
                   ["src", "mid", "gate", "sink"])
    _assert_unresolved_not_excluded(w, "relative name")


def test_namespaced_declared_input_matches_its_resolved_trace_name():
    # sink launched in namespace /robot: "c" resolves to /robot/c (topic_mapper.resolve_name)
    w = walk_chain(stock_graph(_sink_input_events("/robot/c"), _sink_system("c", "/robot/c")),
                   ["src", "mid", "gate", "sink"])
    _assert_unresolved_not_excluded(w, "namespace")


def test_remapped_declared_input_matches_its_resolved_trace_name():
    # launch remap /c:=/fused: the source says /c, rcl traces /fused
    w = walk_chain(stock_graph(_sink_input_events("/fused"), _sink_system("/c", "/fused")),
                   ["src", "mid", "gate", "sink"])
    _assert_unresolved_not_excluded(w, "remap")


def test_input_traced_without_a_topic_name_is_unresolved_not_excluded():
    w = walk_chain(stock_graph(_sink_input_events(None), SYSTEM), ["src", "mid", "gate", "sink"])
    _assert_unresolved_not_excluded(w, "missing trace name")


def test_input_without_a_static_model_name_leaves_the_node_unknown():
    """A dynamic topic (empty name, topic_dynamic) has no resolved entry: none of
    the sink's callbacks may be excluded, so its sample stays unresolved."""
    from lf_eval.chain_latency import declared_inputs
    system = _sink_system("", "/c", topic_dynamic=True)
    system["communication"]["topics"][2]["subscribers"] = []
    assert "sink" not in declared_inputs(system)
    w = walk_chain(stock_graph(_sink_input_events("/computed"), system), ["src", "mid", "gate", "sink"])
    _assert_unresolved_not_excluded(w, "missing model name")


# --- a node is known only when every input resolved ---
# The rule of chain_latency.py and the first two regressions of
# tests/metrics/test_identity_pairing_declared_inputs.py, on this file's helpers.

def _system_with_unresolved_sink_input():
    """sink declares a second input "d" that colcon_lf could not resolve (no topic entry)."""
    system = json.loads(json.dumps(SYSTEM))
    system["nodes"]["sink"]["subscribers"].append({"topic": "d", "callback": "on_d"})
    return system


def _second_sink_subscription(topic):
    """A second sink subscription (callback 42) traced on ``topic`` with one untraced take."""
    events = setup_events() + [
        ev("rcl_subscription_init", 0, subscription_handle=420, node_handle=4,
           rmw_subscription_handle=421, topic_name=topic),
        ev("rclcpp_subscription_init", 0, subscription_handle=420, subscription=422),
        ev("rclcpp_subscription_callback_added", 0, subscription=422, callback=42)]
    return events + run_cb(42, 1200, 1210, tid=4, take=(421, 55))


def test_node_with_an_unresolved_subscriber_has_unknown_inputs():
    from lf_eval.chain_latency import declared_inputs
    inputs = declared_inputs(_system_with_unresolved_sink_input())
    assert "sink" not in inputs, (
        f"a partial input set was used to exclude callbacks: sink -> {sorted(inputs['sink'])}")
    assert inputs["mid"] == frozenset({"/a"})


def test_stock_callback_of_an_unresolved_declared_input_is_unresolved():
    events = sorted(_second_sink_subscription("/d"), key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, _system_with_unresolved_sink_input()),
                   ["src", "mid", "gate", "sink"])
    assert (w["unresolved"], w["not_chain_data"]) == ({"gate->sink": 1}, 0), (
        f"the callback of the unresolved input /d was excluded as not_chain_data "
        f"{w['not_chain_data_topics']} instead of counted unresolved {w['unresolved']}")


def _system_with_two_sink_inputs_on_one_topic():
    """sink subscribes "c" and "d", launched with d:=/c: both endpoints resolve to /c,
    so topic_mapper lists sink twice under /c (pinned in colcon_lf
    test_topic_mapper.py::test_remap_of_two_subscribers_onto_one_topic_lists_the_node_twice)."""
    system = json.loads(json.dumps(SYSTEM))
    system["nodes"]["sink"]["subscribers"].append({"topic": "d", "callback": "on_d"})
    system["communication"]["topics"][2]["subscribers"] = ["sink", "sink"]
    return system


def test_two_inputs_remapped_onto_one_topic_keep_the_node_known():
    """A count rule must not mistake a shared topic for a missing endpoint: the
    sink stays known, its second /c subscription is a chain input (unresolved
    when its take is untraced) and /parameter_events is still excluded."""
    from lf_eval.chain_latency import declared_inputs
    system = _system_with_two_sink_inputs_on_one_topic()
    assert declared_inputs(system).get("sink") == frozenset({"/c"}), (
        f"two endpoints on one topic left the sink unknown: {declared_inputs(system)}")
    events = _with_parameter_events_sub(_second_sink_subscription("/c"))
    events += run_cb(41, 1300, 1310, tid=4, take=(411, 56))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, system), ["src", "mid", "gate", "sink"])
    assert w["unresolved"] == {"gate->sink": 1}, (
        f"the second /c subscription was not counted unresolved: {w['unresolved']}")
    assert w["not_chain_data_topics"] == {"/parameter_events": 1}, (
        f"a shared-topic sink lost its exclusion of /parameter_events: {w['not_chain_data_topics']}")


def test_stock_gate_tick_after_an_input_of_unknown_origin_is_unresolved():
    """Review finding: the newest /b input before the tick has no traced producer.

    The old walk skipped it and credited the tick to the older /b input of the
    src release at 100, a sample of 1150 - 100 with the wrong source.
    """
    events = setup_events()
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += run_cb(20, 200, 220, tid=2, take=(201, 111), pub=(502, 1111))
    events += run_cb(31, 300, 310, tid=3, take=(311, 1111))
    events += run_cb(31, 600, 610, tid=3, take=(311, 2222))    # /b 2222 never traced
    events += run_cb(30, 1000, 1010, tid=3, pub=(503, 7))
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["physical_ns"] == [], f"tick credited to an older input: {w['physical_ns']}"
    assert w["unresolved"] == {"mid->gate": 1} and w["complete"] is False


def test_stock_gate_tick_before_any_input_is_a_startup_sample():
    """Review finding: the tick before the first input is counted apart, not unresolved."""
    events = setup_events()
    events += run_cb(30, 50, 60, tid=3, pub=(503, 5))
    events += run_cb(40, 70, 80, tid=4, take=(401, 5))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["unresolved"] == {}, f"startup tick reported as unresolved={w['unresolved']}"
    assert w.get("startup") == {"mid->gate": 1}


# ── intra-process delivery has no message identity ────────────────

INTRA_FIXTURE = Path(__file__).parent / "fixtures" / "rslcpp_intra_648.json"


def _recorded_intra():
    """System and events recorded from a real RSLCPP run (scripts: evidence/followup-d3)."""
    rec = json.loads(INTRA_FIXTURE.read_text())
    events = [TraceEvent(name=e["name"], timestamp_ns=e["ts"], fields=e["fields"], cpu_id=e["cpu"],
                         pid=e["pid"], tid=e["tid"]) for e in rec["events"]]
    return rec, events


def test_rslcpp_intra_process_sink_is_not_traceable_not_unresolved():
    """Every rmw_take of an RSLCPP run is untaken, so each sink callback
    read consumed=None and the old walk counted it as an unresolved (lost) sample."""
    rec, events = _recorded_intra()
    graph = stock_graph(events, rec["system"])
    w = walk_chain(graph, ["controller_node", "actuator_sink_node"])
    n = rec["provenance"]["sink_callbacks_in_window"]
    assert w["unresolved"] == {}, f"intra-process sink counted as lost samples: unresolved={w['unresolved']}"
    assert w["not_traceable"] == {"controller_node->actuator_sink_node": n} and w["sinks"] == n
    assert w["physical_ns"] == [] and w["complete"] is False
    assert ("actuator_sink_node", "/sentry/cmd") in graph.no_identity


def test_rslcpp_chain_latency_json_and_table_report_not_traceable(tmp_path, monkeypatch, capsys):
    """The per-run chain_latency.json and the CLI table name the class and its reason."""
    from lf_eval.cli import main
    from lf_eval.reader import reader

    rec, events = _recorded_intra()
    (tmp_path / "system.json").write_text(json.dumps(rec["system"]))
    monkeypatch.setattr(reader, "read_trace", lambda run_dir: iter(events))
    out = tmp_path / "chain_latency.json"
    assert main(["chain-latency", str(tmp_path), "--mode", "stock",
                 "--chain", "controller_node->actuator_sink_node", "-o", str(out)]) == 1
    n = rec["provenance"]["sink_callbacks_in_window"]
    row = json.loads(out.read_text())["chains"]["controller_node->actuator_sink_node"]
    assert row["unresolved"] == {}, f"unresolved={row['unresolved']}"
    assert row["not_traceable"] == {"controller_node->actuator_sink_node": n}
    assert row["not_traceable_reason"] == "intra-process: no message identity in this trace"
    assert json.loads(out.read_text())["failures"] == [
        "no_samples:controller_node->actuator_sink_node",
        f"not_traceable:controller_node->actuator_sink_node:{n}"]
    table = capsys.readouterr().out
    assert "not_traceable" in table


def _intra_input(events, sub, rmw_sub, obj, publisher, pub_node, topic):
    """Make subscription *sub* intra-process capable and give *topic* an intra publisher.

    The events rclcpp traces for it: a second ``rclcpp_subscription_init`` for the
    intra-process waitable, its buffer (``rclcpp_ipb_to_subscription``), and the
    ``rcl_publisher_init`` whose handle ``rclcpp_intra_publish`` names.
    """
    return events + [
        ev("rclcpp_subscription_init", 0, subscription_handle=sub, subscription=obj),
        ev("rclcpp_ipb_to_subscription", 0, ipb=obj + 1, subscription=obj),
        ev("rcl_publisher_init", 0, node_handle=pub_node, publisher_handle=publisher,
           rmw_publisher_handle=publisher + 1, topic_name=topic)]


def intra_cb(cb, start, end, tid=1, take=None, pub=None, intra=None):
    """:func:`run_cb` plus an untaken rmw_take and an intra-process publish, as RSLCPP traces them."""
    out = run_cb(cb, start, end, tid, take=take, pub=pub)
    if intra:
        out.append(ev("rclcpp_intra_publish", start + 2, tid, publisher_handle=intra, message=1))
    return out


def test_mixed_run_keeps_a_lost_rmw_message_unresolved():
    """/a stays on rmw while /c is intra-process. A per-run rule ("the run has
    intra publishes") would hide the lost /a message as not traceable too."""
    events = _intra_input(setup_events(), 400, 401, 403, 603, 3, "/c")
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += run_cb(20, 200, 220, tid=2, take=(201, 111))      # a traced /a message
    events += run_cb(20, 300, 320, tid=2, take=(201, 99))       # /a 99 never published: lost
    events += run_cb(20, 400, 420, tid=2)                       # /a callback, take untraced
    events += intra_cb(30, 1000, 1010, tid=3, intra=603)
    events += [ev("rmw_take", 1099, 4, taken=0, rmw_subscription_handle=401, source_timestamp=0)]
    events += run_cb(40, 1100, 1150, tid=4, is_intra=1)         # delivered intra-process
    events.sort(key=lambda e: e.timestamp_ns)
    graph = stock_graph(events, SYSTEM)
    assert graph.no_identity == frozenset({("sink", "/c")})
    w = walk_chain(graph, ["src", "mid"])
    assert w["unresolved"] == {"src->mid": 2} and w["not_traceable"] == {}, (
        f"a lost rmw message was not counted unresolved: {w['unresolved']} {w['not_traceable']}")
    w = walk_chain(graph, ["gate", "sink"])
    assert w["unresolved"] == {} and w["not_traceable"] == {"gate->sink": 1}


def test_subscription_served_both_ways_splits_per_activation():
    """One intra-capable subscription, delivered both over rmw and intra-process.
    The rmw deliveries without a traced message (a taken one whose publication is
    missing, one with is_intra_process=0, one with the field absent) stay
    unresolved; only the one flagged is_intra_process=1 is not traceable."""
    events = _intra_input(setup_events(), 400, 401, 403, 603, 3, "/c")
    events += intra_cb(30, 1000, 1010, tid=3, intra=603)
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 77), is_intra=0)   # rmw /c 77, never published
    events += run_cb(40, 1200, 1250, tid=4, is_intra=0)                  # rmw, take untraced
    events += run_cb(40, 1300, 1350, tid=4)                              # flag absent
    events += run_cb(40, 1400, 1450, tid=4, is_intra=1)                  # intra-process
    events.sort(key=lambda e: e.timestamp_ns)
    graph = stock_graph(events, SYSTEM)
    assert graph.no_identity == frozenset({("sink", "/c")})
    w = walk_chain(graph, ["gate", "sink"])
    assert w["unresolved"] == {"gate->sink": 3}, f"unresolved={w['unresolved']}"
    assert w["not_traceable"] == {"gate->sink": 1}


def test_subscription_without_an_intra_process_buffer_keeps_the_rmw_rules():
    """A callback flagged is_intra_process=1 on a (node, topic) with no intra-process
    buffer is a contradictory trace: it stays unresolved."""
    events = setup_events() + [ev("rcl_publisher_init", 0, node_handle=3, publisher_handle=603,
                                  rmw_publisher_handle=604, topic_name="/c")]
    events += intra_cb(30, 1000, 1010, tid=3, intra=603)
    events += run_cb(40, 1100, 1150, tid=4, is_intra=1)
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["gate", "sink"])
    assert w["unresolved"] == {"gate->sink": 1} and w["not_traceable"] == {}


def test_state_hop_after_an_intra_process_input_is_not_traceable():
    """The gate tick's newest /b input came intra-process: the hop has no identity to name."""
    events = _intra_input(setup_events(), 310, 311, 313, 602, 2, "/b")
    events += run_cb(10, 100, 110, pub=(501, 111))
    events += intra_cb(20, 200, 220, tid=2, take=(201, 111), intra=602)
    events += run_cb(31, 300, 310, tid=3, is_intra=1)           # /b delivered intra-process
    events += run_cb(30, 1000, 1010, tid=3, pub=(503, 7))
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["src", "mid", "gate", "sink"])
    assert w["unresolved"] == {}, f"intra-process state input counted unresolved: {w['unresolved']}"
    assert w["not_traceable"] == {"mid->gate": 1}


def test_latency_failures_name_not_traceable_apart_from_unresolved():
    r = _result("stock")
    r["chains"]["a->b"]["not_traceable"] = {"a->b": 4}
    assert latency_failures(r) == ["not_traceable:a->b:4"]


def test_rmw_callback_with_a_missing_message_stays_unresolved_without_rmw_take():
    """Absence of ``rmw_take`` is no proof of intra-process
    delivery. With the rmw_take tracepoint off (every take dropped), a sink callback
    that rclcpp ran from rmw (``is_intra_process=0``) and whose message is missing
    must stay unresolved, even though its subscription is intra-capable and its
    topic has a local intra publisher. Only the activations whose own
    ``callback_start`` says ``is_intra_process=1`` are not traceable."""
    rec, events = _recorded_intra()
    lo, hi = rec["provenance"]["window_ns"]
    graph0 = stock_graph(events, rec["system"])
    sink_starts = {a.start for a in graph0.by_node["actuator_sink_node"]}
    events = [e for e in events if e.name != "rmw_take"]
    flipped = None
    for i, e in enumerate(events):
        if e.name == "callback_start" and e.timestamp_ns in sink_starts and lo <= e.timestamp_ns < hi:
            assert e.fields.get("is_intra_process") == 1
            events[i] = dataclasses.replace(e, fields={**e.fields, "is_intra_process": 0})
            flipped = e.timestamp_ns
            break
    assert flipped is not None
    w = walk_chain(stock_graph(events, rec["system"]), ["controller_node", "actuator_sink_node"])
    n = rec["provenance"]["sink_callbacks_in_window"]
    hop = "controller_node->actuator_sink_node"
    assert w["unresolved"] == {hop: 1}, (
        f"rmw callback (is_intra_process=0) with a missing message counted as not_traceable: "
        f"unresolved={w['unresolved']} not_traceable={w['not_traceable']}")
    assert w["not_traceable"] == {hop: n - 1} and w["sinks"] == n


def test_intra_flag_shared_start_timestamp_with_an_unflagged_callback_stays_unresolved():
    """The flag is joined by start timestamp. A flagged callback starting at the same
    nanosecond as the unflagged sink callback must not lend it the flag."""
    events = _intra_input(setup_events(), 400, 401, 403, 603, 3, "/c")
    events += intra_cb(30, 1000, 1010, tid=3, intra=603)
    events += run_cb(40, 1100, 1150, tid=4)                     # flag absent
    events += run_cb(10, 1100, 1105, tid=1, is_intra=1)         # other callback, same start
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["gate", "sink"])
    assert w["unresolved"] == {"gate->sink": 1} and w["not_traceable"] == {}, (
        f"unflagged sink borrowed the intra flag: {w['unresolved']} {w['not_traceable']}")


def test_state_hop_running_intra_input_does_not_hide_a_lost_rmw_input():
    """An intra-process /b input still running when the
    gate tick begins must not decide the hop alone when the newest finished /b input
    came over rmw and its publication is missing. The tick may have read that lost
    message, so the hop stays unresolved."""
    events = _intra_input(setup_events(), 310, 311, 313, 602, 2, "/b")
    events += intra_cb(20, 50, 60, tid=2, intra=602)
    events += run_cb(31, 100, 400, tid=3, is_intra=1)               # intra /b, runs past the tick
    events += run_cb(31, 200, 250, tid=5, take=(311, 2222), is_intra=0)  # rmw /b 2222, never published
    events += run_cb(30, 300, 310, tid=6, pub=(503, 7))             # gate tick
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["mid", "gate", "sink"])
    assert w["unresolved"] == {"mid->gate": 1} and w["not_traceable"] == {}, (
        f"lost rmw input hidden behind a running intra input: "
        f"unresolved={w['unresolved']} not_traceable={w['not_traceable']}")


def test_state_hop_running_intra_input_started_later_does_not_hide_an_older_lost_rmw_input():
    """The scan runs newest-first by start. An
    intra-process /b input that STARTED LATER than the newest finished /b input and is
    still running when the gate tick begins must not end the scan: the older finished
    input came over rmw with its publication missing, and the tick may have read that
    lost message (the intra callback had not finished writing). The hop stays unresolved."""
    events = _intra_input(setup_events(), 310, 311, 313, 602, 2, "/b")
    events += intra_cb(20, 50, 60, tid=2, intra=602)
    events += run_cb(31, 100, 150, tid=5, take=(311, 2222), is_intra=0)  # rmw /b 2222, never published
    events += run_cb(31, 200, 350, tid=3, is_intra=1)               # intra /b, starts later, runs past the tick
    events += run_cb(30, 300, 310, tid=6, pub=(503, 7))             # gate tick
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["mid", "gate", "sink"])
    assert w["unresolved"] == {"mid->gate": 1} and w["not_traceable"] == {}, (
        f"lost rmw input hidden behind a running intra input that started later: "
        f"unresolved={w['unresolved']} not_traceable={w['not_traceable']}")


def test_state_hop_overlapping_intra_inputs_do_not_hide_an_older_running_rmw_input():
    """Two overlapping finished intra-process /b inputs
    must not end the scan before an rmw /b input that started earlier and is still
    running when the gate tick begins. Its publication is missing and it may have
    written the state last, so the hop stays unresolved."""
    events = _intra_input(setup_events(), 310, 311, 313, 602, 2, "/b")
    events += intra_cb(20, 10, 20, tid=2, intra=602)
    events += run_cb(31, 50, 350, tid=5, take=(311, 3333), is_intra=0)  # rmw /b 3333, never published
    events += run_cb(31, 100, 210, tid=3, is_intra=1)               # intra /b
    events += run_cb(31, 200, 250, tid=7, is_intra=1)               # intra /b, overlaps the one above
    events += run_cb(30, 300, 310, tid=6, pub=(503, 7))             # gate tick
    events += run_cb(40, 1100, 1150, tid=4, take=(401, 7))
    events.sort(key=lambda e: e.timestamp_ns)
    w = walk_chain(stock_graph(events, SYSTEM), ["mid", "gate", "sink"])
    assert w["unresolved"] == {"mid->gate": 1} and w["not_traceable"] == {}, (
        f"lost rmw input still running hidden behind two overlapping intra inputs: "
        f"unresolved={w['unresolved']} not_traceable={w['not_traceable']}")
