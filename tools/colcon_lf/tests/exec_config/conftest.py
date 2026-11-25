# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Shared helpers for exec_config tests."""

from __future__ import annotations

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import NodeInfo, SubscriberInfo, TimerInfo


def make_launch_graph() -> LaunchGraph:
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return LaunchGraph(root=root, all_nodes=[])


def make_node(
    name: str,
    class_name: str | None = None,
    subscriptions: list[str] | None = None,
    timers: list[str] | None = None,
) -> NodeInfo:
    """Build a NodeInfo with SubscriberInfo entries for each topic in *subscriptions*.

    *timers* is a list of timer callback names (``TimerInfo.callback``); each
    gets a null ``period_ms`` (unresolved), matching an un-patched analysis.
    """
    subs = [
        SubscriberInfo(topic=topic, message_type="std_msgs/msg/String", callback="cb")
        for topic in (subscriptions or [])
    ]
    timer_infos = [
        TimerInfo(period_ms=None, callback=cb) for cb in (timers or [])
    ]
    return NodeInfo(
        name=name,
        class_name=class_name or name.title().replace("_", ""),
        package="test_pkg",
        source_file=f"/fake/{name}.cpp",
        subscribers=subs,
        timers=timer_infos,
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
