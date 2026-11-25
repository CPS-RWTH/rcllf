# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H017, intra-process communication switched on.

The diagnostic runs on a hand-built graph through the whole registry, so the
test also fails when the module is not registered. The real-source case is the
playground example ``intra_process_example/bringup``.

To falsify: remove ``intra_process`` from the module imports in
``features/registry.py`` and ``test_launch_extra_argument_is_refused`` fails
with ``RCLLF-H017`` missing from the emitted codes.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import DetectionContext
from colcon_lf.features.registry import run_detection
from colcon_lf.launch_analysis.models import LaunchNodeAction
from tests.features.conftest import make_graph, make_node

CODE_H017 = "RCLLF-H017"


def _codes(graph) -> list[str]:
    return [d.code for d in run_detection(DetectionContext(graph=graph))
            if d.code == CODE_H017]


def test_launch_extra_argument_is_refused():
    graph = make_graph({})
    graph.launch_graph.all_nodes.append(LaunchNodeAction(
        package="p", executable="p::Pub", name="pub", is_component=True,
        use_intra_process_comms=True))
    graph.launch_graph.all_nodes.append(LaunchNodeAction(
        package="p", executable="p::Sub", name="sub", is_component=True))
    diags = [d for d in run_detection(DetectionContext(graph=graph)) if d.code == CODE_H017]
    assert [d.node for d in diags] == ["pub"], f"expected only 'pub', got {diags}"
    assert "use_intra_process_comms" in diags[0].message


def test_source_option_is_refused(tmp_path):
    source = tmp_path / "node.cpp"
    source.write_text(
        "// use_intra_process_comms(true) in a comment does not count\n"
        "Node::Node() : rclcpp::Node(\"n\", rclcpp::NodeOptions()"
        ".use_intra_process_comms(true)) {}\n")
    node = make_node("n")
    node.source_file = source
    assert _codes(make_graph({"n": node})) == [CODE_H017]


def test_commented_or_false_option_is_not_refused(tmp_path):
    source = tmp_path / "node.cpp"
    source.write_text(
        "// .use_intra_process_comms(true)\n"
        "Node::Node() : rclcpp::Node(\"n\", rclcpp::NodeOptions()"
        ".use_intra_process_comms(false)) {}\n")
    node = make_node("n")
    node.source_file = source
    assert _codes(make_graph({"n": node})) == []
