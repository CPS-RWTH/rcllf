# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Publication correspondence emission (T7).

Generation-stage admission per topic edge (D1), the publisher-side emit and
per-edge ``corr_e<ID>`` port, the zero-delay main connection, the
subscriber-side arrive/drain/close reactions with their logical action
(D4, Ruling M2: any after-delay d >= 0 is admitted, d moves into the drain
wake) and the edge registration in the generated RID init source. Every
assertion names the edge it is about.
"""

from __future__ import annotations

import json
import re

import pytest

from colcon_lf.comm_analysis.topic_mapper import build_communication
from colcon_lf.comm_analysis.models import (
    ServiceConnection,
    SystemCommunication,
    TopicConnection,
)
from colcon_lf.generation.binding_header_emitter import collect_rid_table, emit_rid_init_cpp
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import _stub_add, generate_lf_system
from ..corr_declared import declared, generate_declared
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph, LaunchNodeAction
from colcon_lf.node_analysis.models import (
    CallbackGroupInfo,
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    ServiceClientInfo,
    ServiceServerInfo,
    SubscriberInfo,
    TimerInfo,
)
from colcon_lf.verb.generate import DEFAULT_AFTER_DELAY, _write_correspondence_json

MSG = "std_msgs::msg::Int32"


def _pub(name, cls, topic="data", *, durability=None, callbacks=None, **kw) -> NodeInfo:
    """Timer node whose one callback ``tick`` publishes *topic*."""
    callbacks = callbacks if callbacks is not None else [
        CallbackInfo(name="tick", publishers_used=[topic]),
    ]
    return NodeInfo(
        name=name, class_name=cls, package="test_pkg", source_file=f"/fake/{name}.cpp",
        callbacks=callbacks,
        timers=[TimerInfo(period_ms=100.0, callback=c.name) for c in callbacks],
        # As the clang analysis records a plain create call in the constructor.
        publishers=[PublisherInfo(topic=topic, message_type=MSG, qos_durability=durability,
                                  created_in="constructor", unconditional=True)],
        **kw,
    )


def _sub(name, cls, topic="data", *, durability=None, publishes=None, **kw) -> NodeInfo:
    """Node whose subscription callback ``on_data`` optionally publishes *publishes*."""
    return NodeInfo(
        name=name, class_name=cls, package="test_pkg", source_file=f"/fake/{name}.cpp",
        callbacks=[CallbackInfo(name="on_data", publishers_used=[publishes] if publishes else [])],
        subscribers=[SubscriberInfo(topic=topic, message_type=MSG, callback="on_data",
                                    qos_durability=durability)],
        publishers=[PublisherInfo(topic=publishes, message_type=MSG, created_in="constructor",
                                  unconditional=True)] if publishes else [],
        **kw,
    )


def _graph(nodes, topics, services=None) -> SystemGraph:
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={n.name: n for n in nodes},
        communication=SystemCommunication(topics=topics, services=services or []),
    )


def _tc(topic, pubs, subs, delay=None) -> TopicConnection:
    return TopicConnection(topic=topic, message_type=MSG, publishers=pubs,
                           subscribers=subs, after_delay_ns=delay)


def _edge(system, pub, sub):
    found = [e for e in system.corr_edges if (e.pub_node, e.sub_node) == (pub, sub)]
    assert len(found) == 1, f"edge {pub} -> {sub}: expected one table row, got {found}"
    return found[0]


def _reactor_block(text: str, name: str) -> str:
    start = text.index(f"reactor {name} {{")
    return text[start:text.index("\n}\n", start)]


def _pubsub(delay=None, **gen):
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"], delay)])
    system = generate_declared(graph, **gen)
    return graph, system, emit_lf_system(system)


EMIT0 = ("{ uint32_t n = ros2_lf_corr_emit(self, 0); if (n > 0) "
         "{ rcllf_corr_t c = {0, n}; lf_set(corr_e0, c); } }")


# ---------------------------------------------------------------------------
# Publisher side and connection
# ---------------------------------------------------------------------------


def test_admitted_publisher_emits_and_drops_the_stub():
    _, system, text = _pubsub()
    e = _edge(system, "pub", "sub")
    assert (e.status, e.edge_id, e.reason) == ("admitted", 0, None), f"edge {e.label()}: {e}"
    group = _reactor_block(text, "PubNode_DefaultGroup")
    assert "reaction(t_tick) rcllf_bound -> corr_e0 {=" in group, f"edge {e.label()}:\n{group}"
    assert "ros2_lf_invoke_timer(self, RID_PubNode_tick);\n        " + EMIT0 in group, (
        f"edge {e.label()}: emit block missing after the invoke:\n{group}")
    assert "lf_set(data," not in text, f"edge {e.label()}: stub lf_set survived:\n{group}"
    assert "output data:" not in text and "input data:" not in text, (
        f"edge {e.label()}: stub port survived:\n{text}")
    outer = _reactor_block(text, "PubNode")
    assert "output corr_e0: rcllf_corr_t" in outer, f"edge {e.label()}:\n{outer}"
    assert "default_group.corr_e0 -> corr_e0" in outer, f"edge {e.label()}: not forwarded"


def test_connection_has_no_after_for_admitted_edge():
    _, system, text = _pubsub(after_delay_ns=1_000_000)
    e = _edge(system, "pub", "sub")
    assert e.delay_ns == 1_000_000, f"edge {e.label()}: delay {e.delay_ns}"
    conns = [c for c in system.main_connections if c.source == "pub.corr_e0"]
    assert len(conns) == 1 and conns[0].destination == "sub.corr_e0", (
        f"edge {e.label()}: {system.main_connections}")
    assert conns[0].after_delay_ns is None, f"edge {e.label()}: corr link has an after"
    assert "pub.corr_e0 -> sub.corr_e0\n" in text, f"edge {e.label()}:\n{text}"
    assert "pub.data" not in text, f"edge {e.label()}: legacy stub link emitted:\n{text}"


def test_fanout_gets_one_port_and_connection_per_subscriber():
    graph = _graph([_pub("pub", "PubNode"), _sub("sa", "SubA"), _sub("sb", "SubB")],
                   [_tc("/data", ["pub"], ["sa", "sb"])])
    system = generate_declared(graph, after_delay_ns=2_000_000)
    text = emit_lf_system(system)
    ea, eb = _edge(system, "pub", "sa"), _edge(system, "pub", "sb")
    assert (ea.edge_id, eb.edge_id) == (0, 1), f"edges {ea.label()} / {eb.label()}: ids"
    outer = _reactor_block(text, "PubNode")
    for e in (ea, eb):
        port = f"corr_e{e.edge_id}"
        assert f"output {port}: rcllf_corr_t" in outer, f"edge {e.label()}: no {port}"
        assert f"ros2_lf_corr_emit(self, {e.edge_id})" in text, f"edge {e.label()}: no emit"
        link = f"pub.{port} -> {e.sub_node}.{port}\n"
        assert link in text, f"edge {e.label()}: no zero-delay link {link!r}:\n{text}"
    assert " after " not in text, f"edges {ea.label()}: fan-out link carries an after"
    assert "reaction(t_tick) rcllf_bound -> corr_e0, corr_e1 {=" in text


# ---------------------------------------------------------------------------
# Subscriber side
# ---------------------------------------------------------------------------


def test_subscriber_gets_one_port_triggered_drain_close_and_logical_action():
    """An in-process d = 0 edge has no separate arrive reaction.

    The drain takes the record port as a trigger, files the records and runs
    the first one at the publication tag (t, m). Records 2..k re-arm the
    logical action. A separate arrive would wake the drain at (t, m + 1).
    """
    _, system, text = _pubsub()
    e = _edge(system, "pub", "sub")
    assert e.same_tag, f"edge {e.label()}: an in-process d = 0 edge must be same-tag"
    group = _reactor_block(text, "SubNode_DefaultGroup")
    need = [
        "input corr_e0: rcllf_corr_t",
        "logical action corr_drain_e0",
        "reaction(corr_drain_e0, corr_e0) rcllf_bound -> corr_drain_e0 {=\n"
        "        int64_t w = -1;\n"
        "        if (corr_e0->is_present) { int64_t a = "
        "ros2_lf_corr_arrive(self, 0, corr_e0->value.count); if (a >= 0) { w = a; } }\n"
        "        if (w == 0 || corr_drain_e0->is_present) {\n"
        '            w = ros2_lf_corr_drain(self, 0, RID_SubNode_on_data, "data");\n'
        "        }\n"
        "        w = ros2_lf_corr_claim_wake(0, w);\n"
        "        if (w >= 0 && lf_schedule(corr_drain_e0, w) <= 0) "
        "{ ros2_lf_corr_schedule_refused(0); }",
        "reaction(shutdown) {=\n        ros2_lf_corr_close(0);\n    =}",
    ]
    for snippet in need:
        assert snippet in group, f"edge {e.label()}: missing {snippet!r} in\n{group}"
    assert "reaction(corr_e0) " not in group, (
        f"edge {e.label()}: a separate arrive reaction wakes the drain one microstep late")
    assert "ros2_lf_invoke_sub" not in text, f"edge {e.label()}: legacy invoke_sub left"
    assert "corr_e0 -> default_group.corr_e0" in _reactor_block(text, "SubNode"), (
        f"edge {e.label()}: input not forwarded into the group")


def test_subscriber_drain_is_woken_once_at_startup_to_arm_its_deadline_window():
    """A startup wake drains once at (start, 1), unbound.

    A producer that never publishes sends no record, so nothing else would run
    the drain that arms the logical deadline window (silence_deadline.hpp). The
    wake waits for no node binding: the drain it schedules runs at (start, 1),
    after every startup reaction, and that drain is bound.
    """
    _, system, text = _pubsub()
    e = _edge(system, "pub", "sub")
    group = _reactor_block(text, "SubNode_DefaultGroup")
    arm = ("reaction(startup) -> corr_drain_e0 {=\n        int64_t w = 0;\n"
           "        w = ros2_lf_corr_claim_wake(0, w);\n"
           "        if (w >= 0 && lf_schedule(corr_drain_e0, w) <= 0) "
           "{ ros2_lf_corr_schedule_refused(0); }\n    =}")
    assert arm in group, (
        f"edge {e.label()}: no startup wake arms the deadline window of a silent edge in\n{group}")
    reactor = next(r for r in system.reactors if r.name == "SubNode_DefaultGroup")
    kinds = [r.kind for r in reactor.reactions]
    assert kinds.count("corr_silence_arm") == 1, kinds


def test_split_drain_escape_restores_arrive_drain_close(monkeypatch):
    """RCLLF_LOCAL_SPLIT_DRAIN=1 gives the earlier arrive reaction back."""
    monkeypatch.setenv("RCLLF_LOCAL_SPLIT_DRAIN", "1")
    _, system, text = _pubsub()
    e = _edge(system, "pub", "sub")
    assert not e.same_tag
    group = _reactor_block(text, "SubNode_DefaultGroup")
    need = [
        "reaction(corr_e0) rcllf_bound -> corr_drain_e0 {=\n"
        "        int64_t w = ros2_lf_corr_arrive(self, 0, corr_e0->value.count);\n"
        "        w = ros2_lf_corr_claim_wake(0, w);\n"
        "        if (w >= 0 && lf_schedule(corr_drain_e0, w) <= 0) "
        "{ ros2_lf_corr_schedule_refused(0); }",
        "reaction(corr_drain_e0) rcllf_bound -> corr_drain_e0 {=\n"
        '        int64_t w = ros2_lf_corr_drain(self, 0, RID_SubNode_on_data, "data");\n'
        "        w = ros2_lf_corr_claim_wake(0, w);\n"
        "        if (w >= 0 && lf_schedule(corr_drain_e0, w) <= 0) "
        "{ ros2_lf_corr_schedule_refused(0); }",
    ]
    for snippet in need:
        assert snippet in group, f"edge {e.label()}: missing {snippet!r} in\n{group}"


def test_drain_forwards_the_subscribers_own_outgoing_edges():
    graph = _graph(
        [_pub("pub", "PubNode"), _sub("relay", "Relay", publishes="out"),
         _sub("sink", "Sink", topic="out")],
        [_tc("/data", ["pub"], ["relay"]), _tc("/out", ["relay"], ["sink"])],
    )
    text = emit_lf_system(generate_declared(graph))
    group = _reactor_block(text, "Relay_DefaultGroup")
    assert "reaction(corr_drain_e0, corr_e0) rcllf_bound -> corr_drain_e0, corr_e1 {=" in group, group
    drain = group[group.index("reaction(corr_drain_e0, corr_e0)"):]
    assert ("ros2_lf_corr_drain(self, 0, RID_Relay_on_data" in drain
            and "ros2_lf_corr_emit(self, 1)" in drain), f"edge /out (relay -> sink):\n{drain}"


def test_rid_table_keeps_drain_as_sub_and_skips_arrive_and_close():
    graph, system, _ = _pubsub()
    legacy_system = generate_declared(graph, federated=True)
    table = collect_rid_table({r.name: r for r in system.reactors}, graph)
    legacy = collect_rid_table({r.name: r for r in legacy_system.reactors}, graph)
    e = _edge(system, "pub", "sub")
    assert [(r[0], r[1], r[2], r[3], r[4]) for r in table] == [
        (r[0], r[1], r[2], r[3], r[4]) for r in legacy
    ], f"edge {e.label()}: RID table differs from legacy:\n{table}\n{legacy}"
    drain = [r for r in table if r[1] == "SubNode"]
    assert drain == [(e.sub_rid, "SubNode", "on_data", "sub", "data", 0, 0, -1, "")], (
        f"edge {e.label()}: {drain}")
    assert e.pub_rids == [next(r[0] for r in table if r[2] == "tick")], f"edge {e.label()}"
    assert not any(r[2] == "unknown" for r in table), f"edge {e.label()}: {table}"


def test_init_cpp_registers_admitted_edge_with_verb_default_delay():
    graph, system, _ = _pubsub(after_delay=DEFAULT_AFTER_DELAY)
    e = _edge(system, "pub", "sub")
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    assert '#include "ros2_lf_binding/corr_table_api.h"' in cpp
    want = (f'ros2_lf_binding::register_corr_edge({{0, "PubNode", {{{e.pub_rids[0]}}}, "data", '
            f'"SubNode", {e.sub_rid}, "data", 0LL, 100000000LL, 200000000LL, 1u, 4u, 1u, 8u}});')
    # Bounds by hand: one 100 ms timer, k 1, d 0 (the verb default),
    # T 100 ms (default take_timeout_ms), L = 2T = 200 ms,
    # W = d + L = 200 ms. N(200) = 3 so retain 4, N(0) = 1 so carrier 1.
    assert want in cpp, f"edge {e.label()}: {want!r} not in\n{cpp}"


def test_correspondence_json_rows(tmp_path):
    _, system, _ = _pubsub(after_delay_ns=0)
    _write_correspondence_json(tmp_path, system.corr_edges)
    rows = json.loads((tmp_path / "correspondence.json").read_text())
    assert rows == [{"id": 0, "topic": "/data", "pub": "PubNode", "sub": "SubNode",
                     "delay_ns": 0, "status": "admitted", "reason": None}], rows


# ---------------------------------------------------------------------------
# Legacy edges
# ---------------------------------------------------------------------------


def _two_cb_pub():
    return _pub("pub", "PubNode", callbacks=[
        CallbackInfo(name="tick", publishers_used=["data"]),
        CallbackInfo(name="tock", publishers_used=["data"]),
    ])


def _two_group_pub(kind="MutuallyExclusive"):
    """Two publishing timers on ``data``, each in its own callback group."""
    node = _two_cb_pub()
    if kind == "Reentrant":
        node.callback_groups = [CallbackGroupInfo(name="re_", type="Reentrant")]
        groups = ["re_", "re_"]
    else:
        node.callback_groups = [CallbackGroupInfo(name="ga_", type=kind),
                                CallbackGroupInfo(name="gb_", type=kind)]
        groups = ["ga_", "gb_"]
    for t, g in zip(node.timers, groups):
        t.callback_group = g
    return node


def _srv_sub():
    node = _sub("sub", "SubNode")
    node.callbacks[0].service_clients_used = ["svc"]
    node.service_clients = [ServiceClientInfo(service_name="svc", service_type="x::srv::Y",
                                              call_style="async_callback")]
    return node


LEGACY_CASES = {
    "transient_local": ([_pub("pub", "PubNode", durability="transient_local"),
                         _sub("sub", "SubNode")], {}),
    "federated": ([_pub("pub", "PubNode"), _sub("sub", "SubNode")], {"federated": True}),
    "publisher_callback_split": ([_two_group_pub(), _sub("sub", "SubNode")], {}),
    "subscriber_callback": ([_pub("pub", "PubNode"), _srv_sub()], {}),
}


def test_request_sending_subscription_writes_per_request_sent():
    # Review round 5, O1-1/O1-2: a subscription callback that sends a request
    # keeps kind sub and stays legacy, so its request port is written by the
    # counted write after the invoke (ros2_lf_service_sent), not once per
    # correspondence drain. The range_guard run with the edge admitted showed
    # request_shortfalls=43 for 14 requests in 56 messages.
    graph = _graph([_pub("pub", "PubNode"), _srv_sub()], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph, after_delay_ns=1_000_000)
    assert _edge(system, "pub", "sub").status == "legacy"
    (sub,) = [r for x in system.reactors for r in x.reactions
              if r.kind == "sub" and "svc_req" in r.effects]
    assert "ros2_lf_corr_drain" not in sub.body, sub.body
    assert 'ros2_lf_service_sent(RID_SubNode_on_data, "svc")' in sub.body, sub.body


@pytest.mark.parametrize("reason", sorted(LEGACY_CASES))
def test_refused_edge_stays_legacy_with_reason(reason):
    nodes, gen = LEGACY_CASES[reason]
    graph = _graph(nodes, [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph, after_delay_ns=1_000_000, **gen)
    text = emit_lf_system(system)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.reason, e.edge_id) == ("legacy", reason, None), (
        f"edge {e.label()}: expected legacy/{reason}, got {e.status}/{e.reason}")
    assert "pub.data -> sub.data after 1 msec" in text, (
        f"edge {e.label()}: legacy link lost its after:\n{text}")
    assert "corr_e" not in text, f"edge {e.label()}: legacy edge got a corr port"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    # The legacy row carries the node-local topic and the analyzed
    # reliability and depth (-1: no value), so the binding can check its QoS.
    want = f'register_corr_legacy_edge("/data", "PubNode", "SubNode", "{reason}", "data", "", -1);'
    assert want in cpp, f"edge {e.label()}: {want!r} not in\n{cpp}"


def test_multi_node_fan_in_is_admitted_as_one_group():
    # The publisher nodes of one subscription form one fan-in
    # group, admitted as a unit; the lead is the lowest edge id and the rank
    # the publisher's LF instance name order.
    graph = _graph([_pub("pa", "PubA"), _pub("pb", "PubB"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pa", "pb"], ["sub"])])
    system = generate_declared(graph, after_delay_ns=0)
    text = emit_lf_system(system)
    for pub, edge_id, rank in (("pa", 0, 0), ("pb", 1, 1)):
        e = _edge(system, pub, "sub")
        assert (e.status, e.reason) == ("admitted", None), f"edge {pub}->sub: {(e.status, e.reason)}"
        assert (e.edge_id, e.group_lead, e.fan_in_rank) == (edge_id, 0, rank), (
            f"edge {e.label()}: {(e.edge_id, e.group_lead, e.fan_in_rank)}")
        assert f"{pub}.corr_e{edge_id} -> sub.corr_e{edge_id}" in text, (
            f"edge {e.label()}:\n{text}")
    # No stub link survives, so neither does the per-publisher stub input.
    assert "data_fanin_1" not in text and "after 0" not in text, text


# ---------------------------------------------------------------------------
# Edges of lifecycle nodes
# ---------------------------------------------------------------------------

LIFECYCLE = "rclcpp_lifecycle::LifecycleNode"


def _lifecycle_pair(sub_created="constructor", sub_destroyed=None):
    """Lifecycle publisher made in on_configure, lifecycle subscriber."""
    pub = _pub("pub", "PubNode", base_class=LIFECYCLE)
    pub.publishers[0].is_lifecycle = True
    pub.publishers[0].created_in = "on_configure"
    sub = _sub("sub", "SubNode", base_class=LIFECYCLE)
    sub.subscribers[0].created_in = sub_created
    sub.subscribers[0].destroyed_in = sub_destroyed
    graph = _graph([pub, sub], [_tc("/data", ["pub"], ["sub"])])
    return graph, generate_declared(graph)


def test_lifecycle_edge_is_admitted_and_registers_its_publisher_for_rebind():
    graph, system = _lifecycle_pair()
    e = _edge(system, "pub", "sub")
    assert (e.status, e.reason, e.pub_rebind) == ("admitted", None, True), f"edge {e.label()}: {e}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    # Every positional field before pub_rebind is spelled out with its default.
    tail = ', 8u, ros2_lf_binding::CorrEdgeRole::kLocal, -1, "", "", -1, true});'
    assert line.endswith(tail), f"edge {e.label()}: {line!r} does not end with {tail!r}"
    assert e.to_json()["pub_rebind"] is True, f"edge {e.label()}: {e.to_json()}"


def test_plain_publisher_into_a_lifecycle_subscriber_needs_no_rebind():
    sub = _sub("sub", "SubNode", base_class=LIFECYCLE)
    sub.subscribers[0].created_in = "constructor"
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.pub_rebind) == ("admitted", False), f"edge {e.label()}: {e}"
    assert "pub_rebind" not in e.to_json(), f"edge {e.label()}: {e.to_json()}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    assert line.endswith(", 8u});"), f"edge {e.label()}: {line!r}"


# A lifecycle subscription made or reset in a hook, or with no
# creation site, is admitted as a deferred binding the runtime re-admits on
# every creation (``sub_rebind``), no longer kept legacy.
@pytest.mark.parametrize("created, destroyed", [("on_configure", None),
                                                ("constructor", "on_cleanup"),
                                                (None, None)])
def test_lifecycle_subscription_made_or_reset_in_a_hook_is_admitted_for_rebind(
        created, destroyed):
    graph, system = _lifecycle_pair(created, destroyed)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.reason, e.sub_rebind) == ("admitted", None, True), (
        f"edge {e.label()}: {e}")
    assert e.to_json()["sub_rebind"] is True, f"edge {e.label()}: {e.to_json()}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    tail = ', 8u, ros2_lf_binding::CorrEdgeRole::kLocal, -1, "", "", -1, true, true});'
    assert line.endswith(tail), f"edge {e.label()}: {line!r} does not end with {tail!r}"


@pytest.mark.parametrize("twin_callback", ["on_data", "on_twin"])
def test_analyzed_lifecycle_twins_with_a_hook_made_reader_stay_ambiguous(twin_callback):
    # No mechanism proves which twin a re-created reader
    # is, so twins the analysis sees keep the typed legacy reason and are never
    # admitted for rebind by order. Twins it does not see are refused at bind.
    sub = _sub("sub", "SubNode", base_class=LIFECYCLE)
    sub.callbacks += [CallbackInfo(name="on_twin")]
    sub.subscribers[0].created_in = "on_configure"
    sub.subscribers.append(SubscriberInfo(topic="data", message_type=MSG, callback=twin_callback,
                                          created_in="constructor"))
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub"])])
    e = _edge(generate_declared(graph), "pub", "sub")
    assert (e.status, e.reason, e.sub_rebind) == ("legacy", "ambiguous_endpoint", False), (
        f"edge {e.label()}: {e}")


def test_lifecycle_twins_listed_once_each_stay_ambiguous():
    # Once the analysis resolves the twin's topic, the topic lists
    # the reader once per subscription and each edge gets its own twin. A
    # hook-made reader with twins is still refused at bind (rebind_identity),
    # so neither edge may be admitted for rebind.
    sub = _sub("sub", "SubNode", base_class=LIFECYCLE)
    sub.callbacks += [CallbackInfo(name="on_twin")]
    sub.subscribers[0].created_in = "on_configure"
    sub.subscribers.append(SubscriberInfo(topic="data", message_type=MSG, callback="on_twin",
                                          created_in="on_configure"))
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub", "sub"])])
    edges = [e for e in generate_declared(graph).corr_edges if e.sub_node == "sub"]
    got = sorted((e.status, e.reason, e.sub_rebind) for e in edges)
    assert got == [("legacy", "ambiguous_endpoint", False)] * 2, (
        f"edges into the twin reader are {got}: a hook-made reader with twins is "
        "refused at bind as rebind_identity, so admitting it for rebind fails the run")


def test_plain_publisher_into_a_hook_made_lifecycle_subscription_spells_pub_rebind_false():
    sub = _sub("sub", "SubNode", base_class=LIFECYCLE)
    sub.subscribers[0].created_in = "on_configure"
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.pub_rebind, e.sub_rebind) == ("admitted", False, True), (
        f"edge {e.label()}: {e}")
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    tail = ', 8u, ros2_lf_binding::CorrEdgeRole::kLocal, -1, "", "", -1, false, true});'
    assert line.endswith(tail), f"edge {e.label()}: {line!r} does not end with {tail!r}"


def test_mixed_topic_keeps_stub_for_the_legacy_subscriber():
    lc = _sub("lc", "LcNode", base_class="rclcpp_lifecycle::LifecycleNode")
    lc.subscribers[0].qos_durability = "transient_local"
    graph = _graph(
        [_pub("pub", "PubNode"), _sub("sub", "SubNode"), lc],
        [_tc("/data", ["pub"], ["sub", "lc"])],
    )
    system = generate_declared(graph, after_delay_ns=1_000_000)
    text = emit_lf_system(system)
    ok, lc = _edge(system, "pub", "sub"), _edge(system, "pub", "lc")
    assert (ok.status, lc.status, lc.reason) == ("admitted", "legacy", "transient_local")
    group = _reactor_block(text, "PubNode_DefaultGroup")
    assert "reaction(t_tick) rcllf_bound -> data, corr_e0 {=" in group, (
        f"edge {lc.label()}: stub effect must stay for the legacy edge:\n{group}")
    # P17f: the legacy stub write adds the publishes the binding counted (n),
    # not the literal 1 of one callback run. The old pin asserted a write for
    # every tick, which is a phantom event for the legacy subscriber whenever
    # the tick publishes nothing (P17d note 6).
    assert EMIT0 in group and 'ros2_lf_topic_sent(' in group and (
        _stub_add("data", "n") in group), group
    assert "pub.corr_e0 -> sub.corr_e0\n" in text, f"edge {ok.label()}:\n{text}"
    assert "pub.data -> lc.data after 1 msec" in text, f"edge {lc.label()}:\n{text}"


def test_services_are_untouched():
    server = NodeInfo(
        name="srv", class_name="SrvNode", package="test_pkg", source_file="/fake/srv.cpp",
        callbacks=[CallbackInfo(name="handle")],
        service_servers=[ServiceServerInfo(service_name="svc", service_type="x::srv::Y",
                                           callback="handle")],
    )
    client = _pub("cli", "CliNode")
    client.callbacks[0].service_clients_used = ["svc"]
    client.service_clients = [ServiceClientInfo(service_name="svc", service_type="x::srv::Y",
                                                call_style="async_callback",
                                                response_callback="on_resp")]
    client.callbacks.append(CallbackInfo(name="on_resp"))
    graph = _graph([server, client, _sub("sub", "SubNode")],
                   [_tc("/data", ["cli"], ["sub"])],
                   [ServiceConnection(service="/svc", service_type="x::srv::Y",
                                      server="srv", clients=["cli"])])
    system = generate_declared(graph, after_delay_ns=1_000_000)
    text = emit_lf_system(system)
    assert "cli.svc_req -> srv.svc_req after 1 msec" in text, text
    assert "srv.svc_resp -> cli.svc_resp after 1 msec" in text, text
    assert 'if (ros2_lf_invoke_srv_server(self, RID_SrvNode_handle, "svc") > 0) {' in text
    # The publishing callback also sends a request (srv_send): legacy.
    e = _edge(system, "cli", "sub")
    assert (e.status, e.reason) == ("legacy", "publisher_callback_kind"), f"edge {e.label()}: {e}"
    assert all(e.topic == "/data" for e in system.corr_edges), system.corr_edges
    assert not re.search(r"corr_e\d", text), text


# ---------------------------------------------------------------------------
# Zero-delay loops
# ---------------------------------------------------------------------------


def _loop_graph(feed_through=True, latch_group=None, into_loop=False, state_delay=None):
    """A's timer publishes /cmd to B; B's subscription callback publishes /state back to A.

    With *feed_through* A's on_state also publishes /echo (no subscriber), a
    topic off the loop. With *into_loop* it publishes /cmd instead, onward
    into the loop. Without either on_state is a latch that only stores. With
    *latch_group* on_state sits in its own MutuallyExclusive group, another
    sub-reactor. *state_delay* is the /state link's after delay in ns.

    LF allows zero-delay feedback loops as long as there is no causality
    loop: the
    loop is a causality loop only when every subscription on it publishes
    onward into the loop at its tag (B always does, A only with *into_loop*,
    or with the latch declared before tick under subscriptions_first). That
    is refused at generation; every other variant is admitted and delivers at
    the publication tag.
    """
    a_pubs = ["/cmd"] if into_loop else (["/echo"] if feed_through else [])
    a = NodeInfo(
        name="a", class_name="ANode", package="test_pkg", source_file="/fake/a.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=["/cmd"]),
                   CallbackInfo(name="on_state", publishers_used=a_pubs)],
        timers=[TimerInfo(period_ms=100.0, callback="tick")],
        publishers=[PublisherInfo(topic="/cmd", message_type=MSG)]
        + ([PublisherInfo(topic="/echo", message_type=MSG)] if "/echo" in a_pubs else []),
        subscribers=[SubscriberInfo(topic="/state", message_type=MSG, callback="on_state",
                                    callback_group=latch_group)],
        callback_groups=([CallbackGroupInfo(name=latch_group, type="MutuallyExclusive")]
                         if latch_group else []),
    )
    b = _sub("b", "BNode", topic="/cmd", publishes="/state")
    return _graph([a, b], [_tc("/cmd", ["a"], ["b"]), _tc("/state", ["b"], ["a"], state_delay)])


def _echo_loop(**kw):
    """``generate_declared`` of the into-loop graph, its callback cycle cut by a declared period.

    A's on_state publishes /cmd, B's on_cmd /state: a callback cycle with no
    derivable rate (``rate_unbounded``), bounded by the true input period of
    A's /state subscription, as ``lf/corr_loop_echo.execution.yaml`` does.
    """
    graph = _loop_graph(into_loop=True, state_delay=kw.pop("state_delay", None))
    bounds = declared(graph, 1, min_input_period_ms={("a", "/state"): 100.0})
    return generate_declared(graph, target_lfc=True, corr_bounds=bounds, **kw)


def test_feedback_loop_is_admitted_on_both_edges():
    # A's on_state publishes /echo, off the loop, and tick (the
    # /cmd publisher) is declared before it (timers_first): no causality
    # loop. Both edges are admitted with no `after` and deliver at the
    # publication tag.
    system = generate_declared(_loop_graph(), target_lfc=True)
    cmd, state = _edge(system, "a", "b"), _edge(system, "b", "a")
    assert (cmd.status, cmd.edge_id) == ("admitted", 0), f"edge {cmd.label()}: {cmd}"
    assert (state.status, state.reason, state.edge_id) == ("admitted", None, 1), (
        f"edge {state.label()} closes a node loop with no causality loop and must be "
        f"admitted: {state}")
    assert cmd.same_tag and state.same_tag, (
        f"a loop with no causality loop delivers at the publication tag: {cmd} {state}")
    text = emit_lf_system(system)
    assert re.search(r"^\s*a\.corr_e0 -> b\.corr_e0$", text, re.M), text
    assert re.search(r"^\s*b\.corr_e1 -> a\.corr_e1$", text, re.M), text
    assert "after" not in text.split("main reactor")[-1], (
        "an admitted loop carries no after delay on its links:\n" + text)
    assert "b.state -> a.state" not in text and "a.cmd -> b.cmd" not in text, text
    assert "corr_arrive" not in {r.kind for r in system.reactors for r in r.reactions}, (
        "a same-tag loop edge has no split arrive (no implicit microstep)")


CAUSALITY_MSG = (
    r"zero-delay causality loop: the d = 0 link\(s\) "
    r"\[/cmd \(a -> b\), /state \(b -> a\)\] form a loop on which every subscription "
    r"callback publishes onward into the loop at the tag it receives, so no reaction of the "
    r"loop can run first \(lfc: Cyclic dependency\)\. LF allows a zero-delay feedback loop "
    r"only when it has no causality loop, and rcllf adds no implicit microstep\. Give one "
    r"link of the loop a logical delay d > 0 in execution\.yaml"
)


def test_causality_loop_is_refused_with_the_links_named():
    # A's on_state publishes /cmd, so every subscription on the
    # loop publishes onward into it at its tag (the corr_loop_echo fixture).
    with pytest.raises(ValueError, match=CAUSALITY_MSG):
        _echo_loop()


def test_causality_loop_with_one_link_delayed_runs():
    # d > 0 on /state breaks the causality loop. /cmd keeps its
    # same-tag delivery, /state delivers at t + d with its own arrive.
    system = _echo_loop(state_delay=1_000_000)
    cmd, state = _edge(system, "a", "b"), _edge(system, "b", "a")
    assert (cmd.status, state.status) == ("admitted", "admitted"), (cmd, state)
    assert cmd.same_tag and not state.same_tag and state.delay_ns == 1_000_000, (cmd, state)



def _launched_echo_loop(a_out: str, remaps: list[tuple[str, str]]):
    """The into-loop graph with A's /cmd publisher declared as *a_out*, joined through a launch.

    ``publishers_used`` holds node-local names. A
    declares its /cmd publisher as *a_out* (relative, or another name a
    launch remap in *remaps* turns into /cmd), and B its endpoints relative.
    The topics are resolved by ``build_communication`` from a real launch
    model, as the analyzer does.
    """
    a = NodeInfo(
        name="a", class_name="ANode", package="test_pkg", source_file="/fake/a.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=[a_out]),
                   CallbackInfo(name="on_state", publishers_used=[a_out])],
        timers=[TimerInfo(period_ms=100.0, callback="tick")],
        publishers=[PublisherInfo(topic=a_out, message_type=MSG)],
        subscribers=[SubscriberInfo(topic="state", message_type=MSG, callback="on_state")],
    )
    b = _sub("b", "BNode", topic="cmd", publishes="state")
    entries = [(a, remaps), (b, [])]
    actions = [LaunchNodeAction(package="test_pkg", executable=ni.name, name=ni.name,
                                remappings=list(rm)) for ni, rm in entries]
    graph = SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(
            root=LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg"),
            all_nodes=actions),
        nodes={"a": a, "b": b},
        communication=build_communication([(act, ni) for act, (ni, _) in zip(actions, entries)]),
    )
    assert sorted(t.topic for t in graph.communication.topics) == ["/cmd", "/state"], (
        graph.communication.topics)
    # The declared period is keyed by A's node-local subscription name.
    bounds = declared(graph, 1, min_input_period_ms={("a", "state"): 100.0})
    return generate_declared(graph, target_lfc=True, corr_bounds=bounds)


def test_causality_loop_with_relative_publisher_names_is_refused():
    # A publishes "cmd", B "state", both relative. The
    # loop check compares the node-local publisher name, so it still sees
    # every subscription publish onward into the loop and refuses it.
    with pytest.raises(ValueError, match=CAUSALITY_MSG):
        _launched_echo_loop("cmd", [])


def test_causality_loop_closed_by_a_remap_is_refused():
    # A declares its publisher "out"; only the launch
    # remap out:=/cmd closes the loop. It is a causality loop and refused.
    with pytest.raises(ValueError, match=CAUSALITY_MSG):
        _launched_echo_loop("out", [("out", "/cmd")])

def test_corr_arrive_reactions_are_declared_after_every_other_reaction():
    # lfc orders one reactor's reactions by declaration. An arrive
    # declared last, whose only effect is a logical action, is a sink in lfc's
    # causality graph. Only an edge with d > 0 (or deferred,
    # or cross-federate) has an arrive; the /state link here has d = 1 ms.
    system = _echo_loop(state_delay=1_000_000)
    seen = 0
    for reactor in system.reactors:
        kinds = [r.kind for r in reactor.reactions]
        if "corr_arrive" not in kinds:
            continue
        seen += 1
        first = kinds.index("corr_arrive")
        assert all(k == "corr_arrive" for k in kinds[first:]), (
            f"reactor {reactor.name} declares {kinds[first:]} after its first corr_arrive")
        for r in reactor.reactions:
            if r.kind == "corr_arrive":
                assert r.effects and all(not e.startswith("corr_e") and "." not in e
                                         for e in r.effects), (
                    f"reactor {reactor.name}: corr_arrive effects must be actions only: {r.effects}")
    assert seen == 1, [r.name for r in system.reactors]


def test_latch_after_the_publishing_reactions_closes_no_cycle():
    # A's on_state only stores /state and runs
    # after tick (timers_first), so /state never reaches A's /cmd at its tag.
    system = generate_declared(_loop_graph(feed_through=False), target_lfc=True)
    cmd, state = _edge(system, "a", "b"), _edge(system, "b", "a")
    assert cmd.status == "admitted", f"edge {cmd.label()}: {cmd}"
    assert (state.status, state.reason) == ("admitted", None), (
        f"edge {state.label()} ends in a latch after every publishing reaction of a, "
        f"a state boundary, yet it was refused: {state}")
    assert cmd.same_tag and state.same_tag, (cmd, state)


def test_latch_before_the_publishing_timer_is_a_causality_loop():
    # Under subscriptions_first the latch is declared before tick
    # in A's reactor, so LF orders tick after it and /state reaches /cmd at
    # its tag: lfc would report a cyclic dependency, and so does generation.
    with pytest.raises(ValueError, match=CAUSALITY_MSG):
        generate_declared(_loop_graph(feed_through=False), target_lfc=True,
                          reaction_order={"a": "subscriptions_first"})


def test_latch_in_another_group_is_admitted_too():
    # A callback group puts on_state in its own sub-reactor, and
    # lfc adds no dependency between two reactors, so even under
    # subscriptions_first the latch does not reach tick's /cmd at its tag.
    system = generate_declared(_loop_graph(feed_through=False, latch_group="latch_group_"),
                               target_lfc=True, reaction_order={"a": "subscriptions_first"})
    state = _edge(system, "b", "a")
    assert (state.status, state.reason) == ("admitted", None), (
        f"edge {state.label()} ends in a latch in another sub-reactor than a's publishing "
        f"timer, yet it was refused: {state}")


# ---------------------------------------------------------------------------
# Several publishing callbacks of one node
# ---------------------------------------------------------------------------

EMIT_TOCK = "ros2_lf_invoke_timer(self, RID_PubNode_tock);\n        " + EMIT0


def test_two_publishing_callbacks_in_one_group_are_admitted():
    graph = _graph([_two_cb_pub(), _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph, after_delay_ns=1_000_000)
    text = emit_lf_system(system)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.edge_id, e.pub_callbacks) == ("admitted", 0, ["tick", "tock"]), (
        f"edge {e.label()}: {e}")
    group = _reactor_block(text, "PubNode_DefaultGroup")
    for cb in ("tick", "tock"):
        assert f"reaction(t_{cb}) rcllf_bound -> corr_e0 {{=" in group, (
            f"edge {e.label()}: {cb} lacks the corr_e0 effect:\n{group}")
        assert f"ros2_lf_invoke_timer(self, RID_PubNode_{cb});\n        " + EMIT0 in group, (
            f"edge {e.label()}: {cb} lacks the emit line:\n{group}")
    assert group.count("output corr_e0: rcllf_corr_t") == 1, f"edge {e.label()}:\n{group}"
    outer = _reactor_block(text, "PubNode")
    assert outer.count("output corr_e0: rcllf_corr_t") == 1, (
        f"edge {e.label()}: corr_e0 declared more than once in the outer reactor:\n{outer}")
    assert outer.count("default_group.corr_e0 -> corr_e0") == 1, f"edge {e.label()}:\n{outer}"
    assert text.count("pub.corr_e0 -> sub.corr_e0\n") == 1, f"edge {e.label()}:\n{text}"


def test_two_publishing_callbacks_get_both_rids():
    graph = _graph([_two_cb_pub(), _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph, after_delay_ns=1_000_000)
    e = _edge(system, "pub", "sub")
    table = collect_rid_table({r.name: r for r in system.reactors}, graph)
    want = [r[0] for r in table if r[1] == "PubNode" and r[2] in ("tick", "tock")]
    assert len(want) == 2 and e.pub_rids == sorted(want), (
        f"edge {e.label()}: pub_rids {e.pub_rids}, publishing reactions {want}")
    cpp = emit_rid_init_cpp(table, "pkg", corr_edges=system.corr_edges)
    rids = ", ".join(str(r) for r in e.pub_rids)
    reg = (f'ros2_lf_binding::register_corr_edge({{0, "PubNode", {{{rids}}}, "data", '
           f'"SubNode", {e.sub_rid}, "data", 1000000LL, 100000000LL, 200000000LL, 1u, 7u, 2u, 8u}});')
    # Two 100 ms timers publish the edge: N(201 ms) = 3 + 3, N(1 ms) = 1 + 1.
    assert reg in cpp, f"edge {e.label()}: {reg!r} not in\n{cpp}"


@pytest.mark.parametrize("kind", ["MutuallyExclusive", "Reentrant"])
def test_publishing_callbacks_in_two_reactors_are_refused(kind):
    graph = _graph([_two_group_pub(kind), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph, after_delay_ns=1_000_000)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.reason) == ("legacy", "publisher_callback_split"), (
        f"edge {e.label()} ({kind}): {e}")


# ---------------------------------------------------------------------------
# The analyzed subscription reliability rides on the edge row
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("reliability", ["reliable", "best_effort"])
def test_admitted_edge_carries_the_analyzed_sub_reliability(reliability):
    # The binding compares the subscription's actual QoS at startup against
    # the reliability the analysis saw, so the registration carries it.
    sub = _sub("sub", "SubNode")
    sub.subscribers[0].qos_reliability = reliability
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.sub_reliability) == ("admitted", reliability), f"edge {e.label()}: {e}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    tail = f', 8u, ros2_lf_binding::CorrEdgeRole::kLocal, -1, "", "{reliability}"}});'
    assert line.endswith(tail), f"edge {e.label()}: {line!r} does not end with {tail!r}"


def test_unanalyzed_sub_reliability_keeps_the_spec_default():
    graph, system, _ = _pubsub()
    e = _edge(system, "pub", "sub")
    assert e.sub_reliability == "", f"edge {e.label()}: {e}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    assert line.endswith(", 8u});"), f"edge {e.label()}: {line!r}"


# ---------------------------------------------------------------------------
# Analyzed reader depth on admitted edges, analyzed QoS on legacy edges
# ---------------------------------------------------------------------------


def test_admitted_edge_carries_the_analyzed_sub_depth():
    # The binding compares the user's requested depth at startup against the
    # depth the analysis saw (RCLLF-H002), so the registration carries it.
    sub = _sub("sub", "SubNode")
    sub.subscribers[0].qos_depth = 5
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.sub_depth) == ("admitted", 5), f"edge {e.label()}: {e}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    line = next(ln for ln in cpp.splitlines() if "register_corr_edge" in ln)
    tail = ', 8u, ros2_lf_binding::CorrEdgeRole::kLocal, -1, "", "", 5});'
    assert line.endswith(tail), f"edge {e.label()}: {line!r} does not end with {tail!r}"


def test_legacy_edge_carries_the_analyzed_sub_qos():
    sub = _sub("sub", "SubNode")
    sub.subscribers[0].qos_reliability = "reliable"
    sub.subscribers[0].qos_depth = 1
    graph = _graph([_pub("pub", "PubNode", durability="transient_local"), sub],
                   [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.sub_reliability, e.sub_depth) == ("legacy", "reliable", 1), (
        f"edge {e.label()}: {e}")
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    want = ('register_corr_legacy_edge("/data", "PubNode", "SubNode", "transient_local", '
            '"data", "reliable", 1);')
    assert want in cpp, f"edge {e.label()}: {want!r} not in\n{cpp}"


def test_ambiguous_legacy_edge_carries_the_qos_its_readers_share():
    # Two subscriptions on the topic in one node leave the
    # edge ambiguous. The row must still carry what the analysis saw, else
    # the binding warns RCLLF-H001 and H002 on QoS the analysis did see.
    sub = _sub("sub", "SubNode")
    twin = SubscriberInfo(topic="data", message_type=MSG, callback="on_data",
                          qos_reliability="reliable", qos_depth=10)
    sub.subscribers[0].qos_reliability = "best_effort"
    sub.subscribers[0].qos_depth = 1
    sub.subscribers.append(twin)
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.reason) == ("legacy", "ambiguous_endpoint"), f"edge {e.label()}: {e}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    want = ('register_corr_legacy_edge("/data", "PubNode", "SubNode", "ambiguous_endpoint", '
            '"data", "best_effort", 1);')
    assert want in cpp, f"edge {e.label()}: {want!r} not in\n{cpp}"


def test_ambiguous_legacy_edge_with_twins_spelled_differently_still_carries_their_qos():
    # "data" and "/data" resolve to one topic in a
    # root-namespace node. The row falls back to the graph topic, which the
    # binding resolves, and still carries the weakest analyzed QoS.
    sub = _sub("sub", "SubNode")
    twin = SubscriberInfo(topic="/data", message_type=MSG, callback="on_data",
                          qos_reliability="reliable", qos_depth=10)
    sub.subscribers[0].qos_reliability = "best_effort"
    sub.subscribers[0].qos_depth = 1
    sub.subscribers.append(twin)
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.reason) == ("legacy", "ambiguous_endpoint"), f"edge {e.label()}: {e}"
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    want = ('register_corr_legacy_edge("/data", "PubNode", "SubNode", "ambiguous_endpoint", '
            '"", "best_effort", 1);')
    assert want in cpp, f"edge {e.label()}: {want!r} not in\n{cpp}"


# ---------------------------------------------------------------------------
# Analyzed reliability on every subscription RID
# ---------------------------------------------------------------------------


def test_every_subscription_rid_carries_its_own_analyzed_reliability():
    # The binding checks RCLLF-H000 and H001 per subscription at bind time, so
    # each RID row carries what the analysis saw on that subscription: twins
    # on one topic keep their own value, and a subscription no edge reads (an
    # input from outside the graph) is registered too. No value, no field.
    sub = _sub("sub", "SubNode")
    sub.callbacks += [CallbackInfo(name="on_twin"), CallbackInfo(name="on_ext")]
    sub.subscribers[0].qos_reliability = "best_effort"
    sub.subscribers += [
        SubscriberInfo(topic="data", message_type=MSG, callback="on_twin",
                       qos_reliability="reliable"),
        SubscriberInfo(topic="ext", message_type=MSG, callback="on_ext"),
    ]
    graph = _graph([_pub("pub", "PubNode"), sub], [_tc("/data", ["pub"], ["sub"])])
    system = generate_declared(graph)
    cpp = emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                            "pkg", corr_edges=system.corr_edges)
    rows = {cb: next((ln for ln in cpp.splitlines()
                      if "register_rid_entry" in ln and f'"SubNode", "{cb}"' in ln), None)
            for cb in ("on_data", "on_twin", "on_ext")}
    assert all(rows.values()), f"a subscription RID row is missing: {rows}\n{cpp}"
    assert rows["on_data"].endswith(', "best_effort"});'), rows["on_data"]
    assert rows["on_twin"].endswith(', "reliable"});'), rows["on_twin"]
    assert re.search(r', -?\d+, -?\d+\}\);$', rows["on_ext"]), (
        f"no analyzed reliability must leave the RidEntry default: {rows['on_ext']}")
