# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Same-period timers and same-topic subscriptions in one node.

The runtime binds each timer or subscription handle to exactly one reaction,
in the order the node creates them. The generator side of that contract:

* every timer and subscription RID carries its creation index into the
  generated ``ros2_lf_reaction_ids_init.cpp`` (``RidEntry::creation_index``);
* two subscriptions on one topic share one input port and one connection, so
  lfc accepts the program (it used to emit ports ``twin`` and ``twin_2`` and a
  duplicated top-level connection, which lfc rejects with "Port named 'twin'
  may only appear once on the right side of a connection").

To falsify: drop the ``creation_index`` element in ``collect_rid_table``
(binding_header_emitter.py) and ``test_rid_init_carries_creation_index`` fails;
restore the per-subscription port loop in ``_build_ports`` (lf_generator.py)
and ``test_same_topic_subscriptions_share_one_port`` fails on ``twin_2``.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.generation.binding_header_emitter import (
    collect_foreign_executor_acks,
    collect_rid_table,
    emit_rid_init_cpp,
)
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackGroupInfo,
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    SubscriberInfo,
    TimerInfo,
)


def _twin_subscriber() -> NodeInfo:
    return NodeInfo(
        name="twin_subscriber",
        class_name="TwinSubscriberNode",
        package="p",
        source_file=Path("/fake/s.cpp"),
        header_file=Path("/fake/s.hpp"),
        callbacks=[
            CallbackInfo(name="on_first"),
            CallbackInfo(name="on_second"),
            CallbackInfo(name="timer_1_callback"),
            CallbackInfo(name="timer_2_callback"),
        ],
        # Creation order: timer_1_ before timer_2_, first_ before second_.
        timers=[
            TimerInfo(period_ms=100.0, callback="timer_1_callback"),
            TimerInfo(period_ms=100.0, callback="timer_2_callback"),
        ],
        subscribers=[
            SubscriberInfo(topic="twin", message_type="std_msgs::msg::Int64", callback="on_first"),
            SubscriberInfo(topic="twin", message_type="std_msgs::msg::Int64", callback="on_second"),
        ],
    )


def _twin_publisher() -> NodeInfo:
    return NodeInfo(
        name="twin_publisher",
        class_name="TwinPublisherNode",
        package="p",
        source_file=Path("/fake/p.cpp"),
        header_file=Path("/fake/p.hpp"),
        callbacks=[CallbackInfo(name="timer_callback", publishers_used=["twin"])],
        timers=[TimerInfo(period_ms=100.0, callback="timer_callback")],
        publishers=[PublisherInfo(topic="twin", message_type="std_msgs::msg::Int64")],
    )


def _graph(subscribers: list[str]) -> SystemGraph:
    root = LaunchFileInfo(file_path=Path("/fake/twin_sub.launch.py"), package="p")
    return SystemGraph(
        name="twin_sub",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={"/twin_subscriber": _twin_subscriber(), "/twin_publisher": _twin_publisher()},
        communication=SystemCommunication(
            topics=[
                TopicConnection(
                    topic="/twin",
                    message_type="std_msgs::msg::Int64",
                    publishers=["twin_publisher"],
                    subscribers=subscribers,
                )
            ]
        ),
    )


def _rid_table(graph: SystemGraph) -> list:
    lf_system = generate_lf_system(graph, target_lfc=True)
    return collect_rid_table({r.name: r for r in lf_system.reactors}, graph)


def test_rid_table_records_creation_order():
    rows = {(r[1], r[2]): r for r in _rid_table(_graph(["twin_subscriber"]))}
    assert rows[("TwinSubscriberNode", "timer_1_callback")][6] == 0
    assert rows[("TwinSubscriberNode", "timer_2_callback")][6] == 1
    assert rows[("TwinSubscriberNode", "on_first")][6] == 0
    assert rows[("TwinSubscriberNode", "on_second")][6] == 1
    assert rows[("TwinPublisherNode", "timer_callback")][6] == 0


def test_rid_init_carries_creation_index():
    cpp = emit_rid_init_cpp(_rid_table(_graph(["twin_subscriber"])), "p")
    assert (
        '{RID_TwinSubscriberNode_timer_2_callback, "TwinSubscriberNode", '
        '"timer_2_callback", "timer", "", 100000000LL, 1, -1}'
    ) in cpp
    assert (
        '{RID_TwinSubscriberNode_on_second, "TwinSubscriberNode", "on_second", '
        '"sub", "twin", 0LL, 1, -1}'
    ) in cpp


def _grouped_node() -> NodeInfo:
    """Same-period timers in three callback groups, created A1, B1, A2, plus one
    timer whose group the analyzer could not resolve."""
    return NodeInfo(
        name="grouped",
        class_name="GroupedNode",
        package="p",
        source_file=Path("/fake/g.cpp"),
        header_file=Path("/fake/g.hpp"),
        callbacks=[CallbackInfo(name=n) for n in ("a1", "b1", "a2", "d1", "lost")],
        callback_groups=[
            CallbackGroupInfo(name="group_b_", type="MutuallyExclusive"),
            CallbackGroupInfo(name="group_a_", type="MutuallyExclusive"),
            CallbackGroupInfo(name="default_group", type="MutuallyExclusive"),
        ],
        timers=[
            TimerInfo(period_ms=100.0, callback="a1", callback_group="group_a_"),
            TimerInfo(period_ms=100.0, callback="b1", callback_group="group_b_"),
            TimerInfo(period_ms=100.0, callback="a2", callback_group="group_a_"),
            TimerInfo(period_ms=100.0, callback="d1", callback_group="default_group"),
            TimerInfo(period_ms=100.0, callback="lost", callback_group="made_elsewhere_"),
        ],
    )


def test_rid_table_records_callback_group_order():
    node = _grouped_node()
    root = LaunchFileInfo(file_path=Path("/fake/g.launch.py"), package="p")
    graph = SystemGraph(
        name="grouped",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={"/grouped": node},
        communication=SystemCommunication(topics=[]),
    )
    rows = {r[2]: r for r in _rid_table(graph)}
    # (creation index, group index): groups count in create_callback_group
    # order, without the default group (-1); an unresolved group is -2.
    got = {cb: (rows[cb][6], rows[cb][7]) for cb in ("a1", "b1", "a2", "d1", "lost")}
    assert got == {"a1": (0, 1), "b1": (1, 0), "a2": (2, 1), "d1": (3, -1), "lost": (4, -2)}, got


def _r002_model(provenance: str = "unresolved") -> SimpleNamespace:
    record = SimpleNamespace(
        provenance=provenance, reason="RCLLF-R002", subject={"node": "/grouped"}
    )
    return SimpleNamespace(records=[record])


def _grouped_graph() -> SimpleNamespace:
    return SimpleNamespace(nodes={"/grouped": _grouped_node()})


def test_acknowledged_r002_registers_foreign_executor_ack():
    acks = collect_foreign_executor_acks(_r002_model(), {"RCLLF-R002"}, _grouped_graph())
    assert acks == [("GroupedNode", "RCLLF-R002")], acks
    cpp = emit_rid_init_cpp([], "p", foreign_executor_acks=acks)
    assert (
        'ros2_lf_binding::register_foreign_executor_ack("GroupedNode", "RCLLF-R002");'
    ) in cpp, cpp


def test_foreign_executor_ack_uses_per_instance_binding_keys():
    # Two instances of one class bind as GroupedNode__a and GroupedNode__b
    # (binding_key.py), so the acknowledgement must carry those keys too, or
    # the binding finds none and refuses the tf2-style handles.
    records = [
        SimpleNamespace(provenance="unresolved", reason="RCLLF-R002", subject={"node": n})
        for n in ("/a", "/b")
    ]
    graph = SimpleNamespace(nodes={"/a": _grouped_node(), "/b": _grouped_node()})
    acks = collect_foreign_executor_acks(
        SimpleNamespace(records=records), {"RCLLF-R002"}, graph
    )
    assert acks == [("GroupedNode__a", "RCLLF-R002"), ("GroupedNode__b", "RCLLF-R002")], acks


@pytest.mark.parametrize(
    "model, acknowledged",
    [(_r002_model(), set()), (_r002_model("resolved"), {"RCLLF-R002"}), (None, {"RCLLF-R002"})],
)
def test_foreign_executor_ack_needs_acknowledged_unresolved_r002(model, acknowledged):
    # Without the acknowledgement the binding refuses every handle in a group
    # another executor spins.
    assert collect_foreign_executor_acks(model, acknowledged, _grouped_graph()) == []
    assert "register_foreign_executor_ack" not in emit_rid_init_cpp([], "p")


@pytest.mark.parametrize(
    "subscribers", [["twin_subscriber"], ["twin_subscriber", "twin_subscriber"]]
)
def test_same_topic_subscriptions_share_one_port(subscribers):
    lf_text = emit_lf_system(generate_lf_system(_graph(subscribers), target_lfc=True))
    sub_reactor = lf_text.split("reactor TwinSubscriberNode {", 1)[1].split("\n}\n", 1)[0]
    assert sub_reactor.count("input twin:") == 1
    assert "twin_2" not in lf_text
    # Both callbacks still react to the one port, in creation order.
    assert lf_text.index("RID_TwinSubscriberNode_on_first") < lf_text.index(
        "RID_TwinSubscriberNode_on_second"
    )
    main = lf_text.split("main reactor", 1)[1]
    assert main.count("-> twin_subscriber.twin") == 1, main


def _twin_corr_graph(second_callback: str = "on_second") -> SystemGraph:
    """The twin graph as ``map_communication`` lists it (once per subscription),
    with the publisher's k = 1 proven by the analyzer."""
    graph = _graph(["twin_subscriber", "twin_subscriber"])
    graph.nodes["/twin_publisher"].callbacks[0].publish_bounds = {"twin": 1}
    graph.nodes["/twin_subscriber"].subscribers[1].callback = second_callback
    return graph


def test_twin_subscriptions_get_one_correspondence_edge_each():
    # Each twin has its own
    # callback and RID, so each gets its own admitted edge, port and drain.
    # Falsify by making _twin_subscriptions return [None] * listed: both
    # edges fall back to ("legacy", "ambiguous_endpoint").
    system = generate_lf_system(_twin_corr_graph(), target_lfc=True)
    got = [(e.status, e.reason, e.edge_id, e.sub_callback) for e in system.corr_edges]
    assert got == [
        ("admitted", None, 0, "on_first"),
        ("admitted", None, 1, "on_second"),
    ], got
    rids = {r[2]: r[0] for r in collect_rid_table(
        {r.name: r for r in system.reactors}, _twin_corr_graph())}
    assert [e.sub_rid for e in system.corr_edges] == [rids["on_first"], rids["on_second"]]
    main = emit_lf_system(system).split("main reactor", 1)[1]
    assert "twin_publisher.corr_e0 -> twin_subscriber.corr_e0" in main, main
    assert "twin_publisher.corr_e1 -> twin_subscriber.corr_e1" in main, main


def test_twin_subscriptions_sharing_a_callback_stay_ambiguous():
    # Two subscriptions bound to one callback share one RID, so nothing tells
    # their handles apart and both edges stay legacy.
    system = generate_lf_system(_twin_corr_graph("on_first"), target_lfc=True)
    got = [(e.status, e.reason) for e in system.corr_edges]
    assert got == [("legacy", "ambiguous_endpoint")] * 2, got


@pytest.mark.parametrize("subscribers", [["twin_subscriber"], ["twin_subscriber", "twin_subscriber"]])
def test_publisher_listed_twice_reuses_the_twin_assignment(subscribers):
    # map_communication lists a node once per
    # publisher, so a node with two publishers on the topic appears twice in
    # pub_keys. The twin index used to count per (pub_key, sub_key) across both
    # passes and ran past the listing with "IndexError: list index out of
    # range". Each pass now reuses one assignment, the first pass admits and
    # the repeat pass refuses duplicate_subscription.
    graph = _twin_corr_graph()
    graph.communication.topics[0].publishers = ["twin_publisher", "twin_publisher"]
    graph.communication.topics[0].subscribers = subscribers
    system = generate_lf_system(graph, target_lfc=True)
    got = [(e.status, e.reason, e.sub_callback) for e in system.corr_edges]
    if len(subscribers) == 1:
        # One listing of a node with two subscriptions: ambiguous, never a crash.
        assert got == [("legacy", "ambiguous_endpoint", "")] * 2, got
    else:
        assert got == [
            ("admitted", None, "on_first"), ("admitted", None, "on_second"),
            ("legacy", "duplicate_subscription", "on_first"),
            ("legacy", "duplicate_subscription", "on_second"),
        ], got
