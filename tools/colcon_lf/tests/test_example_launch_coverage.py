# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Launch analysis of example launch files that a docs page shows.

``multi_service_example``, ``reentrant_fanout_example`` and
``action_example/launch/dual.launch.py`` are named in ``docs/examples/`` but
no other test parses them. These checks keep their launch files from rotting:
each must parse and yield the nodes the docs page describes.

Requires a built + sourced ROS2 workspace (Docker).
"""

from __future__ import annotations

import pytest

launch_ros = pytest.importorskip("launch_ros", reason="launch_ros not available — run inside Docker")


def _nodes(graph):
    return {(n.package, n.executable) for n in graph.all_nodes}


def test_multi_service_bringup_has_server_and_client(launch_parser):
    # bringup.launch.py includes its two launch files through the install
    # space; the package needs example_interfaces to build.
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        get_package_share_directory("multi_service_example")
    except PackageNotFoundError:
        pytest.skip("multi_service_example is not installed (needs example_interfaces)")
    graph = launch_parser("multi_service_example", "bringup.launch.py")
    assert _nodes(graph) == {
        ("multi_service_example", "multi_service_server_node"),
        ("multi_service_example", "multi_service_client_node"),
    }


def test_reentrant_fanout_bringup_has_publisher_and_worker(launch_parser):
    graph = launch_parser("reentrant_fanout_example", "bringup.launch.py")
    assert _nodes(graph) == {
        ("reentrant_fanout_example", "fanout_publisher_node"),
        ("reentrant_fanout_example", "fanout_worker_node"),
    }


def test_action_dual_launch_has_dual_server_and_client(launch_parser):
    graph = launch_parser("action_example", "dual.launch.py")
    assert {(n.name, n.executable) for n in graph.all_nodes} == {
        ("dual_server", "action_dual_server_node"),
        ("dual_client", "action_dual_client_node"),
    }
