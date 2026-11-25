# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Shared helpers for features/ tests — synthetic SystemGraph builders."""

from __future__ import annotations

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import NodeInfo, PublisherInfo, SubscriberInfo


def make_launch_graph() -> LaunchGraph:
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return LaunchGraph(root=root, all_nodes=[])


def make_sub(
    topic: str,
    *,
    qos_depth: int | None = None,
    qos_reliability: str | None = None,
    topic_dynamic: bool = False,
) -> SubscriberInfo:
    return SubscriberInfo(
        topic=topic,
        message_type="std_msgs/msg/String",
        callback="cb",
        qos_depth=qos_depth,
        qos_reliability=qos_reliability,
        topic_dynamic=topic_dynamic,
    )


def make_pub(
    topic: str,
    *,
    qos_depth: int | None = None,
    qos_reliability: str | None = None,
    topic_dynamic: bool = False,
) -> PublisherInfo:
    return PublisherInfo(
        topic=topic,
        message_type="std_msgs/msg/String",
        qos_depth=qos_depth,
        qos_reliability=qos_reliability,
        topic_dynamic=topic_dynamic,
    )


def make_node(
    name: str,
    class_name: str | None = None,
    subscribers: list[SubscriberInfo] | None = None,
    publishers: list[PublisherInfo] | None = None,
    detected_includes: list[str] | None = None,
) -> NodeInfo:
    return NodeInfo(
        name=name,
        class_name=class_name or name.title().replace("_", ""),
        package="test_pkg",
        source_file=f"/fake/{name}.cpp",
        subscribers=subscribers or [],
        publishers=publishers or [],
        detected_includes=detected_includes or [],
    )


def make_graph(
    nodes: dict[str, NodeInfo],
    topics: list[TopicConnection] | None = None,
) -> SystemGraph:
    return SystemGraph(
        name="test_system",
        launch_graph=make_launch_graph(),
        nodes=nodes,
        communication=SystemCommunication(topics=topics or []),
    )
