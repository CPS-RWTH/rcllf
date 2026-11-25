# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A message_filters synchronizer's publications ride its input activations.

The fusion node publishes /fused/range from the
synchronizer callback, which the library calls synchronously inside the input
subscription callbacks (``<cb>_via_<member>``). Before this the edge stayed
legacy as ``publisher_callback_kind`` because the raw ``sync_callback`` (no
reaction of its own) counted as a publishing callback. Now it is folded into
the input callbacks, and their burst is the policy bound (ExactTime 1,
ApproximateTime Q) times the sync callback's own publish bound.
"""

from __future__ import annotations

import dataclasses

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.exec_config.resolve import CorrBoundsConfig
from colcon_lf.generation.lf_generator import apply_sync_queue_params, generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.message_filters_scan import queue_parameter
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    MessageFiltersInfo,
    MessageFiltersSyncInfo,
    NodeInfo,
    PublisherInfo,
    SubscriberInfo,
    TimerInfo,
)
from colcon_lf.resolved_model.model import Record

MSG = "sensor_msgs::msg::Range"


def _timer_pub(name: str, topic: str) -> NodeInfo:
    return NodeInfo(
        name=name, class_name=name.title(), package="p", source_file=f"/f/{name}.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=[topic], publish_bounds={topic: 1})],
        timers=[TimerInfo(period_ms=10.0, callback="tick")],
        publishers=[PublisherInfo(topic=topic, message_type=MSG, created_in="constructor",
                                  unconditional=True)],
    )


def _fusion(policy: str, queue_size: int | None, queue_param: str | None = None) -> NodeInfo:
    body = dict(publishers_used=["/fused"], publish_bounds={"/fused": 1})
    return NodeInfo(
        name="fusion", class_name="Fusion", package="p", source_file="/f/fusion.cpp",
        callbacks=[
            CallbackInfo(name="sync_callback", callers=["<used, not called, in <constructor>>"],
                         **body),
            CallbackInfo(name="sync_callback_via_a_sub", **body),
            CallbackInfo(name="sync_callback_via_b_sub", **body),
        ],
        subscribers=[
            SubscriberInfo(topic="/a", message_type=MSG, callback="sync_callback_via_a_sub",
                           member_variable="a_sub_", sync_callback="sync_callback"),
            SubscriberInfo(topic="/b", message_type=MSG, callback="sync_callback_via_b_sub",
                           member_variable="b_sub_", sync_callback="sync_callback"),
        ],
        publishers=[PublisherInfo(topic="/fused", message_type=MSG, created_in="constructor",
                                  unconditional=True)],
        message_filters=MessageFiltersInfo(synchronizers=[MessageFiltersSyncInfo(
            member_variable="sync_", policy=policy, queue_size=queue_size,
            inputs=["a_sub_", "b_sub_"], callback="sync_callback", queue_param=queue_param,
        )]),
    )


def _consumer() -> NodeInfo:
    return NodeInfo(
        name="ctrl", class_name="Ctrl", package="p", source_file="/f/ctrl.cpp",
        callbacks=[CallbackInfo(name="on_fused")],
        subscribers=[SubscriberInfo(topic="/fused", message_type=MSG, callback="on_fused")],
    )


def _graph(fusion: NodeInfo) -> SystemGraph:
    nodes = [_timer_pub("a", "/a"), _timer_pub("b", "/b"), fusion, _consumer()]
    tc = [
        TopicConnection(topic="/a", message_type=MSG, publishers=["a"], subscribers=["fusion"]),
        TopicConnection(topic="/b", message_type=MSG, publishers=["b"], subscribers=["fusion"]),
        TopicConnection(topic="/fused", message_type=MSG, publishers=["fusion"],
                        subscribers=["ctrl"]),
    ]
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=LaunchFileInfo(file_path="/f/b.launch.py", package="p"),
                                 all_nodes=[]),
        nodes={n.name: n for n in nodes},
        communication=SystemCommunication(topics=tc, services=[]),
    )


def _fused_edge(graph: SystemGraph, bounds: CorrBoundsConfig | None = None):
    system = generate_lf_system(graph, corr_bounds=bounds or CorrBoundsConfig())
    found = [e for e in system.corr_edges if e.topic == "/fused"]
    assert len(found) == 1, found
    return found[0]


def test_approximate_time_edge_is_admitted_with_burst_q_and_a_reader_that_holds_it():
    e = _fused_edge(_graph(_fusion("ApproximateTime", 9)))
    assert (e.status, e.reason) == ("admitted", None), (
        f"fusion edge stays {e.status} ({e.reason}): the sync callback must be folded "
        "into its input activations")
    assert e.pub_callbacks == ["sync_callback_via_a_sub", "sync_callback_via_b_sub"], e
    # Q = 9 sync callbacks per add() times 1 publish per sync callback.
    assert e.burst == 9, e.bound_inputs
    # Undeclared reader depth: the default 8 raised to the burst.
    assert e.reader_depth == 9, e
    srcs = {v["source"] for v in e.bound_inputs["k_per_callback"].values()}
    assert srcs == {"proven x ApproximateTime Q=9 sync callbacks per add()"}, srcs


def test_exact_time_edge_bursts_once_per_input():
    e = _fused_edge(_graph(_fusion("ExactTime", 9)))
    assert (e.status, e.burst, e.reader_depth) == ("admitted", 1, 8), e


def test_unresolved_queue_size_keeps_the_edge_unsized():
    e = _fused_edge(_graph(_fusion("ApproximateTime", None, "queue_size")))
    assert (e.status, e.reason) == ("legacy", "burst_unbounded"), e


def test_declared_reader_depth_below_q_refuses():
    bounds = CorrBoundsConfig(reader_depth={("ctrl", "/fused"): 8})
    e = _fused_edge(_graph(_fusion("ApproximateTime", 9)), bounds)
    assert (e.status, e.reason) == ("legacy", "reader_depth_below_burst"), e


def test_resolved_parameter_value_becomes_the_queue_size():
    graph = _graph(_fusion("ApproximateTime", None, "queue_size"))
    rec = Record(contract_id="param:fusion/queue_size#value", property="value",
                 subject={"kind": "param", "node": "fusion", "name": "queue_size"}, value=12,
                 provenance="explicitly_configured", evidence="launch inline parameters")
    model = dataclasses.make_dataclass("M", [("records", list)])([rec])
    resolved = apply_sync_queue_params(graph, model)
    assert resolved.nodes["fusion"].message_filters.synchronizers[0].queue_size == 12
    e = _fused_edge(resolved)
    assert (e.status, e.burst, e.reader_depth) == ("admitted", 12, 12), e


def test_queue_parameter_reads_the_declared_parameter_name():
    code = ('const int64_t queue_size = this->declare_parameter<int64_t>("queue_size", 9);\n'
            'sync_ = std::make_shared<S>(SyncPolicy(static_cast<uint32_t>(queue_size)), a, b);')
    assert queue_parameter("SyncPolicy(static_cast<uint32_t>(queue_size))", code) == "queue_size"
    assert queue_parameter("SyncPolicy(q)", "int q = 3; q = declare_parameter(\"x\", 1);") is None
    assert queue_parameter("SyncPolicy(n_)", "n_ = compute();") is None
