# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Launch analysis reads a launch file the way ``ros2 launch`` does (review round 5, O2).

Each test writes a small launch file, parses it and compares the node as launch
would start it. Requires launch_ros (run inside Docker with ROS2 sourced).

To falsify: revert the namespace/name/remap binding in ``_extract_node`` and
``test_arguments_bind_in_namespace_name_and_remap`` fails with the namespace
``'$(var ns)'`` instead of ``'/robot'``.
"""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

pytest.importorskip("launch_ros", reason="launch_ros not available — run inside Docker")

from colcon_lf.launch_analysis import parser as launch_parser  # noqa: E402
from colcon_lf.launch_analysis.parser import (  # noqa: E402
    LaunchArgumentError,
    launch_argument_overrides,
    parse_launch,
)

_HEAD = """
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (
    ComposableNodeContainer, Node, PushRosNamespace, SetParameter, SetRemap)
from launch_ros.descriptions import ComposableNode, ParameterValue
"""


def _parse(tmp_path: Path, body: str, args: dict[str, str] | None = None):
    launch_parser.clear_parse_launch_cache()
    path = tmp_path / "t.launch.py"
    path.write_text(_HEAD + textwrap.dedent(body))
    return parse_launch(path, workspace_root=tmp_path, launch_args=args or {})


def _errors(graph) -> list[str]:
    return list(graph.root.errors)


def test_arguments_bind_in_namespace_name_and_remap(tmp_path):
    graph = _parse(tmp_path, """
    def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument('ns', default_value='a'),
            DeclareLaunchArgument('nm', default_value='talk'),
            DeclareLaunchArgument('topic', default_value='x'),
            Node(package='demo_nodes_cpp', executable='talker',
                 namespace=LaunchConfiguration('ns'), name=LaunchConfiguration('nm'),
                 remappings=[('chatter', LaunchConfiguration('topic'))]),
        ])
    """, {"ns": "robot", "topic": "y"})
    node = graph.all_nodes[0]
    assert (node.namespace, node.name, node.remappings) == (
        "/robot", "talk", [("chatter", "y")]), (
        f"namespace/name/remap not bound: {node.namespace!r} {node.name!r} {node.remappings!r}")


def test_an_argument_without_a_value_is_refused_by_name(tmp_path):
    graph = _parse(tmp_path, """
    def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument('ns'),
            Node(package='demo_nodes_cpp', executable='talker',
                 namespace=LaunchConfiguration('ns')),
        ])
    """)
    assert any("'ns'" in e for e in _errors(graph)), _errors(graph)


def test_set_remap_is_scoped_like_push_ros_namespace(tmp_path):
    graph = _parse(tmp_path, """
    def generate_launch_description():
        return LaunchDescription([
            GroupAction([
                SetRemap('chatter', 'inside'),
                Node(package='demo_nodes_cpp', executable='talker', name='a'),
            ]),
            Node(package='demo_nodes_cpp', executable='talker', name='b'),
        ])
    """)
    remaps = {n.name: n.remappings for n in graph.all_nodes}
    assert remaps == {"a": [("chatter", "inside")], "b": []}, remaps


def test_set_parameter_reaches_nodes_after_it_with_the_node_value_winning(tmp_path):
    graph = _parse(tmp_path, """
    def generate_launch_description():
        return LaunchDescription([
            SetParameter(name='use_sim_time', value=True),
            Node(package='demo_nodes_cpp', executable='talker', name='a'),
        ])
    """)
    assert graph.all_nodes[0].parameters.get("use_sim_time") is True, \
        graph.all_nodes[0].parameters


def test_a_launch_action_the_analysis_cannot_read_is_refused(tmp_path):
    graph = _parse(tmp_path, """
    def generate_launch_description():
        return LaunchDescription([Shutdown()])
    """)
    assert any("Shutdown" in e and "not supported" in e for e in _errors(graph)), _errors(graph)


def test_parameter_value_is_evaluated_to_its_type(tmp_path):
    graph = _parse(tmp_path, """
    def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument('n', default_value='5'),
            Node(package='demo_nodes_cpp', executable='talker', name='a',
                 parameters=[{'count': ParameterValue(
                     LaunchConfiguration('n'), value_type=int)}]),
        ])
    """)
    assert graph.all_nodes[0].parameters["count"] == 5, graph.all_nodes[0].parameters


def test_a_launch_argument_is_part_of_the_cache_key(tmp_path):
    body = """
    def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument('nm', default_value='one'),
            Node(package='demo_nodes_cpp', executable='talker',
                 name=LaunchConfiguration('nm')),
        ])
    """
    launch_parser.clear_parse_launch_cache()
    path = tmp_path / "t.launch.py"
    path.write_text(_HEAD + textwrap.dedent(body))
    first = parse_launch(path, workspace_root=tmp_path, launch_args={})
    second = parse_launch(path, workspace_root=tmp_path, launch_args={"nm": "two"})
    assert (first.all_nodes[0].name, second.all_nodes[0].name) == ("one", "two")


def test_an_undeclared_launch_argument_is_refused_listing_the_declared_ones(tmp_path):
    with pytest.raises(LaunchArgumentError, match="extra.*declares: nm"):
        _parse(tmp_path, """
        def generate_launch_description():
            return LaunchDescription([
                DeclareLaunchArgument('nm', default_value='one'),
                Node(package='demo_nodes_cpp', executable='talker'),
            ])
        """, {"extra": "true"})


def test_active_overrides_reach_parse_launch(tmp_path):
    body = """
    def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument('nm', default_value='one'),
            Node(package='demo_nodes_cpp', executable='talker',
                 name=LaunchConfiguration('nm')),
        ])
    """
    launch_parser.clear_parse_launch_cache()
    path = tmp_path / "t.launch.py"
    path.write_text(_HEAD + textwrap.dedent(body))
    with launch_argument_overrides({"nm": "three"}):
        graph = parse_launch(path, workspace_root=tmp_path)
    assert graph.all_nodes[0].name == "three"


def test_composable_intra_process_flag_is_captured(tmp_path):
    graph = _parse(tmp_path, """
    def generate_launch_description():
        return LaunchDescription([ComposableNodeContainer(
            name='c', namespace='', package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(package='demo_nodes_cpp',
                               plugin='demo_nodes_cpp::Talker', name='t',
                               extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(package='demo_nodes_cpp',
                               plugin='demo_nodes_cpp::Listener', name='l'),
            ])])
    """)
    flags = {n.name: n.use_intra_process_comms for n in graph.all_nodes}
    assert flags == {"t": True, "l": None}, flags


def test_two_launch_files_with_one_generated_name_are_reported():
    from colcon_lf.verb.analyze import _generated_name_collisions
    found = _generated_name_collisions([
        ("o2_adv", "mt_main.launch.py", Path("/x/mt_main.launch.py")),
        ("o2_adv", "mtmain.launch.py", Path("/x/mtmain.launch.py")),
    ])
    assert len(found) == 1 and found[0][0].startswith("lf_generated_o2_adv_"), found
