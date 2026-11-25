# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for colcon_lf.comm_analysis.topic_mapper.

Phase 4 — 7 assertions covering:
- Basic pub/sub matching
- Timer-only node (no communication)
- Service server/client matching
- Action server/client matching
- Namespace resolution
- Remapping resolution
- Unmatched (orphaned) publisher

All tests use in-memory objects only — no Docker, no libclang, no filesystem.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.comm_analysis.topic_mapper import build_communication, _resolve_name
from colcon_lf.launch_analysis.models import LaunchNodeAction
from colcon_lf.node_analysis.models import (
    ActionClientInfo,
    ActionServerInfo,
    NodeInfo,
    PublisherInfo,
    ServiceClientInfo,
    ServiceServerInfo,
    SubscriberInfo,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _node(
    name: str,
    publishers: list[PublisherInfo] | None = None,
    subscribers: list[SubscriberInfo] | None = None,
    service_servers: list[ServiceServerInfo] | None = None,
    service_clients: list[ServiceClientInfo] | None = None,
    action_servers: list[ActionServerInfo] | None = None,
    action_clients: list[ActionClientInfo] | None = None,
) -> NodeInfo:
    return NodeInfo(
        name=name,
        class_name=name,
        package="test_pkg",
        source_file=Path("/dev/null"),
        publishers=publishers or [],
        subscribers=subscribers or [],
        service_servers=service_servers or [],
        service_clients=service_clients or [],
        action_servers=action_servers or [],
        action_clients=action_clients or [],
    )


def _launch(
    namespace: str | None = None,
    remappings: list[tuple[str, str]] | None = None,
) -> LaunchNodeAction:
    return LaunchNodeAction(
        package="test_pkg",
        executable="test_exe",
        namespace=namespace,
        remappings=remappings or [],
    )


# ---------------------------------------------------------------------------
# 1. Basic pub/sub match
# ---------------------------------------------------------------------------

def test_pubsub_match():
    """1: publisher_node publishes sensor_data; subscriber_node subscribes sensor_data."""
    pub_node = _node(
        "publisher_node",
        publishers=[PublisherInfo(topic="sensor_data", message_type="custom_interfaces::msg::SensorData")],
    )
    sub_node = _node(
        "subscriber_node",
        subscribers=[SubscriberInfo(topic="sensor_data", message_type="custom_interfaces::msg::SensorData", callback="topic_callback")],
    )
    comm = build_communication([(None, pub_node), (None, sub_node)])

    assert len(comm.topics) == 1
    tc = comm.topics[0]
    assert tc.topic == "/sensor_data"
    assert tc.publishers == ["publisher_node"]
    assert tc.subscribers == ["subscriber_node"]
    assert "SensorData" in tc.message_type
    assert comm.services == []
    assert comm.actions == []


# ---------------------------------------------------------------------------
# 2. Timer-only node — no communication
# ---------------------------------------------------------------------------

def test_timer_only_no_communication():
    """2: timer_node has no pub/sub/service/action → empty SystemCommunication."""
    timer_node = _node("timer_node")
    comm = build_communication([(None, timer_node)])

    assert comm.topics == []
    assert comm.services == []
    assert comm.actions == []


# ---------------------------------------------------------------------------
# 3. Service server/client match
# ---------------------------------------------------------------------------

def test_service_match():
    """3: service_server_node provides process_data; service_client_node calls it."""
    server_node = _node(
        "service_server_node",
        service_servers=[ServiceServerInfo(
            service_name="process_data",
            service_type="custom_interfaces::srv::ProcessData",
            callback="handle_process_data",
        )],
    )
    client_node = _node(
        "service_client_node",
        service_clients=[ServiceClientInfo(
            service_name="process_data",
            service_type="custom_interfaces::srv::ProcessData",
        )],
    )
    comm = build_communication([(None, server_node), (None, client_node)])

    assert comm.topics == []
    assert len(comm.services) == 1
    sc = comm.services[0]
    assert sc.service == "/process_data"
    assert sc.server == "service_server_node"
    assert sc.clients == ["service_client_node"]
    assert "ProcessData" in sc.service_type


# ---------------------------------------------------------------------------
# 4. Action server/client match
# ---------------------------------------------------------------------------

def test_action_match():
    """4: action_server_node provides analyze; action_client_node calls it."""
    server_node = _node(
        "action_server_node",
        action_servers=[ActionServerInfo(
            action_name="analyze",
            action_type="custom_interfaces::action::Analyze",
            goal_callback="handle_goal",
        )],
    )
    client_node = _node(
        "action_client_node",
        action_clients=[ActionClientInfo(
            action_name="analyze",
            action_type="custom_interfaces::action::Analyze",
        )],
    )
    comm = build_communication([(None, server_node), (None, client_node)])

    assert comm.topics == []
    assert comm.services == []
    assert len(comm.actions) == 1
    ac = comm.actions[0]
    assert ac.action == "/analyze"
    assert ac.server == "action_server_node"
    assert ac.clients == ["action_client_node"]


# ---------------------------------------------------------------------------
# 5. Namespace resolution
# ---------------------------------------------------------------------------

def test_namespace_resolution():
    """5: relative topic 'sensor_data' under namespace 'robot_1' resolves to '/robot_1/sensor_data'."""
    pub_node = _node(
        "publisher_node",
        publishers=[PublisherInfo(topic="sensor_data", message_type="SensorData")],
    )
    sub_node = _node(
        "subscriber_node",
        subscribers=[SubscriberInfo(topic="sensor_data", message_type="SensorData", callback="cb")],
    )
    launch = _launch(namespace="robot_1")
    comm = build_communication([(launch, pub_node), (launch, sub_node)])

    assert len(comm.topics) == 1
    assert comm.topics[0].topic == "/robot_1/sensor_data"
    assert comm.topics[0].publishers == ["publisher_node"]
    assert comm.topics[0].subscribers == ["subscriber_node"]


def test_namespace_resolution_unit():
    """5b: _resolve_name unit test — relative, absolute, and namespaced names."""
    assert _resolve_name("sensor_data", None, []) == "/sensor_data"
    assert _resolve_name("sensor_data", "robot_1", []) == "/robot_1/sensor_data"
    assert _resolve_name("/absolute", "robot_1", []) == "/absolute"
    assert _resolve_name("sensor_data", "robot_1", [("sensor_data", "/remapped")]) == "/remapped"


# ---------------------------------------------------------------------------
# 6. Remapping resolution
# ---------------------------------------------------------------------------

def test_remapping():
    """6: publisher remaps 'sensor_data' to '/remapped'; subscriber also remapped → connect on '/remapped'."""
    pub_node = _node(
        "publisher_node",
        publishers=[PublisherInfo(topic="sensor_data", message_type="SensorData")],
    )
    sub_node = _node(
        "subscriber_node",
        subscribers=[SubscriberInfo(topic="sensor_data", message_type="SensorData", callback="cb")],
    )
    remap = [("sensor_data", "/remapped")]
    comm = build_communication([
        (_launch(remappings=remap), pub_node),
        (_launch(remappings=remap), sub_node),
    ])

    assert len(comm.topics) == 1
    assert comm.topics[0].topic == "/remapped"
    assert comm.topics[0].publishers == ["publisher_node"]
    assert comm.topics[0].subscribers == ["subscriber_node"]


# ---------------------------------------------------------------------------
# 7. Unmatched (orphaned) publisher
# ---------------------------------------------------------------------------

def test_unmatched_publisher():
    """7: publisher with no matching subscriber → topic in output with empty subscribers."""
    pub_node = _node(
        "publisher_node",
        publishers=[PublisherInfo(topic="orphan_topic", message_type="SomeType")],
    )
    comm = build_communication([(None, pub_node)])

    assert len(comm.topics) == 1
    tc = comm.topics[0]
    assert tc.topic == "/orphan_topic"
    assert tc.publishers == ["publisher_node"]
    assert tc.subscribers == []


# ---------------------------------------------------------------------------
# 8. Unresolved (empty) topic names never join
# ---------------------------------------------------------------------------

def test_empty_topic_entries_never_join():
    """Two unrelated nodes with unresolvable topic names must not appear connected.

    ``clang_parser`` emits ``topic=""`` when a topic-name argument cannot be
    resolved statically. Keying both entries into
    ``pub_map``/``sub_map`` under the same resolved name ``"/"`` would fabricate a
    ``TopicConnection`` between two nodes that share nothing. Their surfacing
    mechanism is RCLLF-H004 (``features/modules/dynamic_topic.py``), not a
    made-up edge.
    """
    pub_node = _node(
        "dynamic_publisher_node",
        publishers=[PublisherInfo(topic="", message_type="SomeType")],
    )
    sub_node = _node(
        "unrelated_subscriber_node",
        subscribers=[SubscriberInfo(topic="", message_type="OtherType", callback="cb")],
    )
    comm = build_communication([(None, pub_node), (None, sub_node)])

    assert comm.topics == []
    assert all(tc.topic != "/" for tc in comm.topics)


def test_empty_topic_does_not_suppress_the_resolvable_edges_on_the_same_node():
    """A node's unresolvable endpoint is dropped; its literal endpoints still join."""
    pub_node = _node(
        "dynamic_publisher_node",
        publishers=[
            PublisherInfo(topic="", message_type="SomeType"),
            PublisherInfo(topic="status", message_type="StatusType"),
        ],
    )
    sub_node = _node(
        "static_subscriber_node",
        subscribers=[SubscriberInfo(topic="status", message_type="StatusType", callback="cb")],
    )
    comm = build_communication([(None, pub_node), (None, sub_node)])

    assert len(comm.topics) == 1
    tc = comm.topics[0]
    assert tc.topic == "/status"
    assert tc.publishers == ["dynamic_publisher_node"]
    assert tc.subscribers == ["static_subscriber_node"]


def test_whitespace_only_topic_is_treated_as_unresolved():
    pub_node = _node(
        "pub",
        publishers=[PublisherInfo(topic="   ", message_type="SomeType")],
    )
    comm = build_communication([(None, pub_node)])
    assert comm.topics == []


def test_node_keys_list_each_endpoint_by_its_instance():
    # Two instances of one class on the two ends of one edge.
    relay = dict(
        publishers=[PublisherInfo(topic="out", message_type="T")],
        subscribers=[SubscriberInfo(topic="in", message_type="T", callback="cb")],
        service_servers=[ServiceServerInfo(service_name="srv", service_type="S", callback="cb")],
        service_clients=[ServiceClientInfo(service_name="/b/srv", service_type="S")],
    )
    comm = build_communication(
        [(_launch(namespace="a", remappings=[("out", "/chat")]), _node("relay", **relay)),
         (_launch(namespace="b", remappings=[("in", "/chat")]), _node("relay", **relay))],
        node_keys=["a/relay", "b/relay"],
    )
    assert comm.endpoint_ids == "node_key"
    chat = next(t for t in comm.topics if t.topic == "/chat")
    assert (chat.publishers, chat.subscribers) == (["a/relay"], ["b/relay"])
    srv = next(s for s in comm.services if s.service == "/b/srv")
    assert (srv.server, srv.clients) == ("b/relay", ["a/relay", "b/relay"])


def test_without_node_keys_endpoints_are_node_names():
    comm = build_communication([(None, _node("relay", publishers=[
        PublisherInfo(topic="out", message_type="T")]))])
    assert comm.endpoint_ids == "node_name"
    assert comm.topics[0].publishers == ["relay"]


def test_remap_of_two_subscribers_onto_one_topic_lists_the_node_twice():
    # lf_eval chain_latency.declared_inputs counts a node's entries under the
    # resolved topics against its subscriber count (a review finding).
    # That holds only if each named subscriber is listed once, also when a remap
    # puts two of them on one topic. A dedup here would leave such a node unknown.
    sink = _node("sink", subscribers=[
        SubscriberInfo(topic="c", message_type="T", callback="on_c"),
        SubscriberInfo(topic="d", message_type="T", callback="on_d")])
    comm = build_communication([(_launch(remappings=[("d", "/c")]), sink)])
    assert [(t.topic, t.subscribers) for t in comm.topics] == [("/c", ["sink", "sink"])], (
        f"two subscribers remapped onto /c not listed once each: "
        f"{[(t.topic, t.subscribers) for t in comm.topics]}")
