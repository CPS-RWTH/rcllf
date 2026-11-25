# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A launch Node's ``arguments=`` and ``ros_arguments=`` reach the lfc node.

launch_ros runs ``<exe> <arguments> --ros-args -r __node:=<name> -r
__ns:=<ns> <parameters> <remappings>`` and rcl takes the first matching remap
rule. Before the fix the analyzer's node key and topics ignored
``arguments=`` and the generated ``NodeOptions`` never carried them, so a
node renamed there kept its launch name in the lfc run. The expected values
below follow the stock run of
``launch_features_example/launch/16_node_arguments.launch.py`` (2026-10-13,
``ros2 node list`` / ``ros2 topic list`` / ``ros2 param get``).
"""

from __future__ import annotations

import pytest

from colcon_lf.comm_analysis.topic_mapper import build_communication
from colcon_lf.generation.factory_emitter import ros_args_for
from colcon_lf.generation.node_identity import build_node_identities
from colcon_lf.graph.hierarchical import _node_key, launch_parameter_values
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph, LaunchNodeAction
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    NodeInfo,
    ParameterInfo,
    PublisherInfo,
    SubscriberInfo,
)

MSG = "std_msgs::msg::String"


def launch_identity(action, own_name):
    # Imported here so the other tests fail on the bug, not on the import, at
    # a commit before the module existed.
    from colcon_lf.launch_analysis.ros_arguments import launch_identity as identity
    return identity(action, own_name)


# The fixture's first node: name='arg_node', renamed in arguments=.
RENAMING_ARGS = [
    "--ros-args", "-r", "__node:=renamed", "-r", "feature_output:=chat2",
    "-p", "timer_period_ms:=250", "-p", "renamed:message_prefix:=from_args",
    "-p", "feature_node:message_prefix:=code_name", "-p", "arg_node:timer_period_ms:=1",
]
# The third node's ros_arguments=, as the parser records them.
ROS_ARGUMENTS = ["--ros-args", "-r", "__node:=x", "-p", "timer_period_ms:=700"]


def _feature_node() -> NodeInfo:
    return NodeInfo(
        name="feature_node", class_name="FeatureNode", package="launch_features_example",
        source_file="/fake/feature_node.cpp",
        callbacks=[CallbackInfo(name="on_input")],
        publishers=[PublisherInfo(topic="feature_output", message_type=MSG)],
        subscribers=[SubscriberInfo(topic="feature_input", message_type=MSG, callback="on_input")],
        parameters=[ParameterInfo(name="timer_period_ms", type="int", default_value=500),
                    ParameterInfo(name="message_prefix", type="string", default_value="hello")],
    )


def _fixture_actions() -> list[LaunchNodeAction]:
    return [
        LaunchNodeAction(
            package="launch_features_example", executable="feature_node", name="arg_node",
            arguments=list(RENAMING_ARGS),
            parameters={"timer_period_ms": 300},
            parameter_sources=[{"params": {"timer_period_ms": 300}, "key": "/**"}],
        ),
        LaunchNodeAction(
            package="launch_features_example", executable="feature_node", name="listener",
            namespace="/ignored", arguments=["--ros-args", "-r", "__ns:=/grp"],
        ),
        LaunchNodeAction(
            package="launch_features_example", executable="feature_node", name="ros_node",
            namespace="/ra", arguments=list(ROS_ARGUMENTS),
        ),
    ]


def _graph(actions) -> SystemGraph:
    pairs = [(a, _feature_node()) for a in actions]
    return SystemGraph(
        name="fixture",
        launch_graph=LaunchGraph(
            root=LaunchFileInfo(file_path="/fake/x.launch.py", package="p"), all_nodes=actions),
        nodes={_node_key(a, ni): ni for a, ni in pairs},
        communication=build_communication(pairs),
    )


class TestAnalyzerIdentity:
    def test_node_keys_are_the_stock_node_list(self):
        # Stock `ros2 node list`: /grp/listener, /ra/x, /renamed.
        assert set(_graph(_fixture_actions()).nodes) == {"renamed", "/grp/listener", "/ra/x"}

    def test_topic_edge_follows_the_arguments_remap(self):
        # Stock `ros2 topic list` (without /parameter_events and /rosout).
        comm = _graph(_fixture_actions()).communication
        topics = {t.topic for t in comm.topics}
        assert topics == {"/chat2", "/feature_input", "/grp/feature_input", "/grp/feature_output",
                          "/ra/feature_input", "/ra/feature_output"}

    def test_launch_parameter_beats_the_earlier_p_rule(self):
        # Stock `ros2 param get /renamed`: timer_period_ms 300, message_prefix from_args.
        action = _fixture_actions()[0]
        got = launch_parameter_values(action, _feature_node(), {})
        assert {k: v for k, (v, _) in got.items()} == {
            "timer_period_ms": 300, "message_prefix": "from_args"}

    def test_ros_arguments_rename_and_parameter(self):
        # Stock: /ra/x with timer_period_ms 700, message_prefix hello.
        action = _fixture_actions()[2]
        assert _node_key(action, _feature_node()) == "/ra/x"
        got = launch_parameter_values(action, _feature_node(), {})
        assert {k: v for k, (v, _) in got.items()} == {"timer_period_ms": 700}

    @pytest.mark.parametrize("namespace, name, expected", [
        (None, "n", {"answer": 42}),          # /n: the rule's node
        (None, "other", {}),                  # /other: another node
        ("/grp", "n", {}),                    # /grp/n: a prefix never holds a namespace
    ])
    def test_node_targeted_p_rule(self, namespace, name, expected):
        # rcl keys `-p n:answer:=42` by /n and rclcpp applies it to the node
        # whose name after remapping is /n (stock `ros2 run`, 2026-10-13).
        action = LaunchNodeAction(package="p", executable="e", name=name, namespace=namespace,
                                  arguments=["--ros-args", "-p", "n:answer:=42"])
        assert dict(launch_identity(action, "code").params) == expected

    def test_node_targeted_p_rule_matches_the_remapped_name(self):
        action = LaunchNodeAction(package="p", executable="e", name="n", arguments=[
            "--ros-args", "-r", "__node:=m", "-p", "n:a:=1", "-p", "code:b:=2", "-p", "m:c:=3"])
        assert dict(launch_identity(action, "code").params) == {"c": 3}

    def test_remappings_node_rule_renames_a_nameless_node(self):
        action = LaunchNodeAction(package="p", executable="e", remappings=[("__node", "other")])
        assert launch_identity(action, "orig").name == "other"

    def test_launch_name_beats_a_later_remappings_node_rule(self):
        action = LaunchNodeAction(package="p", executable="e", name="given",
                                  remappings=[("__node", "other")])
        assert launch_identity(action, "orig").name == "given"

    def test_targeted_name_rule_applies_to_its_node_only(self):
        args = ["--ros-args", "-r", "someone_else:__node:=x", "-r", "orig:__node:=mine"]
        action = LaunchNodeAction(package="p", executable="e", name="given", arguments=args)
        assert launch_identity(action, "orig").name == "mine"

    def test_arguments_topic_rule_beats_remappings(self):
        action = LaunchNodeAction(package="p", executable="e", name="n",
                                  arguments=["--ros-args", "-r", "data:=first"],
                                  remappings=[("data", "second")])
        assert launch_identity(action, "n").remappings == (("data", "first"), ("data", "second"))


class TestGeneratedNodeOptions:
    def test_factory_arguments_carry_the_stock_identity(self):
        identities = build_node_identities(_graph(_fixture_actions()))
        renamed = identities["renamed"]
        args = ros_args_for(renamed.node_name, renamed.namespace, renamed.remappings,
                            renamed.param_args)
        assert args[:5] == ["--ros-args", "-r", "__node:=renamed", "-r", "feature_output:=chat2"]
        # -p rules first, then the launch parameters=, as on the stock command line.
        rest = args[5:]
        # Node-targeted rules keep their prefix; rclcpp matches it as stock does.
        assert rest[:8] == [
            "-p", "timer_period_ms:=250", "-p", "renamed:message_prefix:=from_args",
            "-p", "feature_node:message_prefix:=code_name", "-p", "arg_node:timer_period_ms:=1"]
        assert rest[8] == "--rcllf-params-yaml" and "timer_period_ms: 300" in rest[9]
        listener = identities["/grp/listener"]
        assert ros_args_for(listener.node_name, listener.namespace, listener.remappings) == [
            "--ros-args", "-r", "__node:=listener", "-r", "__ns:=/grp"]
        x = identities["/ra/x"]
        assert ros_args_for(x.node_name, x.namespace, x.remappings, x.param_args)[:7] == [
            "--ros-args", "-r", "__node:=x", "-r", "__ns:=/ra", "-p", "timer_period_ms:=700"]

    def test_remappings_node_rule_is_not_shadowed_by_the_code_name(self):
        action = LaunchNodeAction(package="launch_features_example", executable="feature_node",
                                  remappings=[("__node", "other")])
        identity = build_node_identities(_graph([action]))["other"]
        assert ros_args_for(identity.node_name, identity.namespace, identity.remappings) == [
            "--ros-args", "-r", "__node:=other"]

    @pytest.mark.parametrize("arguments, needle", [
        (["--verbose"], "'--verbose' is a command-line argument of the executable's main()"),
        (["--ros-args", "--params-file", "/x.yaml"], "'--params-file /x.yaml'"),
        (["--ros-args", "--log-level", "debug"], "'--log-level debug' is process-wide"),
        (["--ros-args", "-r", "__ns:=rel"], "rcl needs an absolute namespace"),
        (["--ros-args", "-r", "n:topic:=x"], "targeted at one node is not supported"),
        (["--ros-args", "-r"], "needs a 'name:=value' rule"),
        (["--ros-args", "-p", "x:=1", "--", "tail"], "'tail' is a command-line argument"),
        (["<unresolved substitution>"], "'<unresolved substitution>'"),
        (["--ros-args", "-p", "/n:a:=1"], "node prefix with '/' or a wildcard"),
        (["--ros-args", "-p", "*:a:=1"], "node prefix with '/' or a wildcard"),
    ])
    def test_unapplicable_arguments_are_refused(self, arguments, needle):
        action = LaunchNodeAction(package="launch_features_example", executable="feature_node",
                                  name="n", arguments=arguments)
        from colcon_lf.launch_analysis.ros_arguments import NodeArgumentsError

        with pytest.raises(NodeArgumentsError) as exc:
            build_node_identities(_graph([action]))
        assert needle in str(exc.value) and "launch node n" in str(exc.value)


def test_parser_records_ros_arguments(tmp_path):
    pytest.importorskip("launch_ros", reason="launch_ros not available — run inside Docker")
    from colcon_lf.launch_analysis.parser import clear_parse_launch_cache, parse_launch

    launch_dir = tmp_path / "src" / "test_pkg" / "launch"
    launch_dir.mkdir(parents=True)
    (launch_dir / "test.launch.py").write_text("""\
import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([Node(
        package='test_pkg', executable='n', name='a',
        arguments=['--ros-args', '-r', '__node:=b'], ros_arguments=['-r', 'x:=y'])])
""")
    clear_parse_launch_cache()
    graph = parse_launch(launch_dir / "test.launch.py", workspace_root=tmp_path)
    (node,) = graph.all_nodes
    assert node.arguments == ["--ros-args", "-r", "__node:=b", "--ros-args", "-r", "x:=y"]
    identity = launch_identity(node, "n")
    assert (identity.name, identity.remappings) == ("b", (("x", "y"),))
