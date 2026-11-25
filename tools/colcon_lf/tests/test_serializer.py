# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for colcon_lf.output.serializer and colcon_lf.output.writer.

Phase 6 — 3 TDD assertions required by the plan:
1. Round-trip: NodeInfo -> JSON -> NodeInfo (all fields preserved)
2. Round-trip: SystemGraph -> JSON -> SystemGraph (all layers preserved)
3. Writer creates expected directory structure with valid JSON files

All tests use in-memory mock objects — no Docker, no filesystem dependencies
(except writer tests which use tmp_path).
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from colcon_lf.comm_analysis.models import (
    ActionConnection,
    ServiceConnection,
    SystemCommunication,
    TopicConnection,
)
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import (
    LaunchArgument,
    LaunchFileInfo,
    LaunchGraph,
    LaunchGroup,
    LaunchNodeAction,
)
from colcon_lf.node_analysis.models import (
    ActionClientInfo,
    ActionServerInfo,
    CallbackGroupInfo,
    CallbackInfo,
    CallbackParameter,
    NodeInfo,
    ParameterInfo,
    PublisherInfo,
    ServiceClientInfo,
    ServiceServerInfo,
    SubscriberInfo,
    TimerInfo,
)
from colcon_lf.output.serializer import (
    deserialize_node_info,
    deserialize_system_graph,
    serialize,
)


# ---------------------------------------------------------------------------
# Fixtures: mock objects with all field types populated
# ---------------------------------------------------------------------------

@pytest.fixture
def sample_node_info() -> NodeInfo:
    return NodeInfo(
        name="publisher_node",
        class_name="PublisherNode",
        package="pubsub_example",
        source_file=Path("/workspace/packages/src/pubsub_example/src/publisher_node.cpp"),
        header_file=Path("/workspace/packages/src/pubsub_example/include/publisher_node.hpp"),
        base_class="rclcpp::Node",
        is_component=False,
        callbacks=[
            CallbackInfo(
                name="timer_callback",
                parameters=[],
                publishers_used=["sensor_data"],
                service_clients_used=[],
                action_clients_used=[],
                variables_read=["publisher_"],
                variables_written=["count_"],
            ),
            CallbackInfo(
                name="topic_callback",
                parameters=[
                    CallbackParameter(
                        name="msg",
                        type="custom_interfaces::msg::SensorData::SharedPtr",
                    ),
                ],
                publishers_used=[],
                service_clients_used=[],
                action_clients_used=[],
                variables_read=[],
                variables_written=["last_msg_"],
            ),
        ],
        callback_groups=[
            CallbackGroupInfo(name="exclusive_group_", type="MutuallyExclusive"),
            CallbackGroupInfo(name="reentrant_group_", type="Reentrant"),
        ],
        timers=[
            TimerInfo(period_ms=500.0, callback="timer_callback", callback_group="exclusive_group_"),
        ],
        subscribers=[
            SubscriberInfo(
                topic="sensor_data",
                message_type="custom_interfaces::msg::SensorData",
                callback="topic_callback",
                qos_depth=10,
                callback_group="reentrant_group_",
            ),
        ],
        service_servers=[
            ServiceServerInfo(
                service_name="process_data",
                service_type="custom_interfaces::srv::ProcessData",
                callback="handle_process_data",
                callback_group=None,
            ),
        ],
        action_servers=[
            ActionServerInfo(
                action_name="analyze",
                action_type="custom_interfaces::action::Analyze",
                goal_callback="handle_goal",
                cancel_callback="handle_cancel",
                accepted_callback="handle_accepted",
                callback_group=None,
            ),
        ],
        publishers=[
            PublisherInfo(
                topic="sensor_data",
                message_type="custom_interfaces::msg::SensorData",
                qos_depth=10,
            ),
        ],
        service_clients=[
            ServiceClientInfo(
                service_name="process_data",
                service_type="custom_interfaces::srv::ProcessData",
                response_callback="process_data_response_callback",
            ),
        ],
        action_clients=[
            ActionClientInfo(
                action_name="analyze",
                action_type="custom_interfaces::action::Analyze",
            ),
        ],
        parameters=[
            ParameterInfo(name="robot_name", type="string", default_value="my_robot", description="Robot name"),
            ParameterInfo(name="max_speed", type="double", default_value=2.5, description=None),
            ParameterInfo(name="debug_mode", type="bool", default_value=True, description=None),
            ParameterInfo(name="update_rate", type="int", default_value=20, description=None),
        ],
        member_variables=["publisher_", "subscription_", "timer_", "count_"],
        lifecycle_callbacks=["on_configure", "on_activate"],
        has_param_callback=True,
    )


@pytest.fixture
def sample_system_graph(sample_node_info: NodeInfo) -> SystemGraph:
    sub_node = NodeInfo(
        name="subscriber_node",
        class_name="SubscriberNode",
        package="pubsub_example",
        source_file=Path("/workspace/packages/src/pubsub_example/src/subscriber_node.cpp"),
        header_file=None,
        base_class="rclcpp::Node",
        subscribers=[
            SubscriberInfo(
                topic="sensor_data",
                message_type="custom_interfaces::msg::SensorData",
                callback="topic_callback",
                qos_depth=10,
            ),
        ],
        callbacks=[
            CallbackInfo(name="topic_callback", parameters=[
                CallbackParameter(name="msg", type="custom_interfaces::msg::SensorData::SharedPtr"),
            ]),
        ],
    )

    pub_launch = LaunchFileInfo(
        file_path=Path("/workspace/packages/src/pubsub_example/launch/publisher_node.launch.py"),
        package="pubsub_example",
        nodes=[
            LaunchNodeAction(package="pubsub_example", executable="publisher_node"),
        ],
    )
    sub_launch = LaunchFileInfo(
        file_path=Path("/workspace/packages/src/pubsub_example/launch/subscriber_node.launch.py"),
        package="pubsub_example",
        nodes=[
            LaunchNodeAction(package="pubsub_example", executable="subscriber_node"),
        ],
    )
    bringup = LaunchFileInfo(
        file_path=Path("/workspace/packages/src/pubsub_example/launch/bringup.launch.py"),
        package="pubsub_example",
        includes=[pub_launch, sub_launch],
        arguments=[
            LaunchArgument(name="namespace", default_value="robot_1"),
        ],
        groups=[
            LaunchGroup(
                namespace="robot_1",
                nodes=[
                    LaunchNodeAction(
                        package="pubsub_example",
                        executable="publisher_node",
                        namespace="robot_1",
                        remappings=[("sensor_data", "/remapped_data")],
                    ),
                ],
                condition=None,
            ),
        ],
    )

    all_nodes = [
        LaunchNodeAction(package="pubsub_example", executable="publisher_node"),
        LaunchNodeAction(package="pubsub_example", executable="subscriber_node"),
    ]

    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=bringup, all_nodes=all_nodes),
        nodes={
            "publisher_node": sample_node_info,
            "subscriber_node": sub_node,
        },
        communication=SystemCommunication(
            topics=[
                TopicConnection(
                    topic="sensor_data",
                    message_type="custom_interfaces::msg::SensorData",
                    publishers=["publisher_node"],
                    subscribers=["subscriber_node"],
                ),
            ],
            services=[
                ServiceConnection(
                    service="process_data",
                    service_type="custom_interfaces::srv::ProcessData",
                    server="service_server",
                    clients=["service_client"],
                ),
            ],
            actions=[
                ActionConnection(
                    action="analyze",
                    action_type="custom_interfaces::action::Analyze",
                    server="action_server",
                    clients=["action_client"],
                ),
            ],
        ),
        parameter_files={
            "params.yaml": {
                "parameter_node": {
                    "ros__parameters": {
                        "robot_name": "ros2_lf_robot",
                        "max_speed": 2.5,
                    }
                }
            }
        },
    )


# ---------------------------------------------------------------------------
# Assertion 1: Round-trip NodeInfo
# ---------------------------------------------------------------------------

class TestNodeInfoRoundTrip:
    def test_serialize_produces_valid_json(self, sample_node_info):
        json_str = serialize(sample_node_info)
        data = json.loads(json_str)
        assert data["name"] == "publisher_node"
        assert data["class_name"] == "PublisherNode"

    def test_path_serialized_as_string(self, sample_node_info):
        json_str = serialize(sample_node_info)
        data = json.loads(json_str)
        assert isinstance(data["source_file"], str)
        assert "publisher_node.cpp" in data["source_file"]

    def test_round_trip_preserves_name(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert restored.name == sample_node_info.name

    def test_round_trip_preserves_paths(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert isinstance(restored.source_file, Path)
        assert restored.source_file == sample_node_info.source_file
        assert isinstance(restored.header_file, Path)
        assert restored.header_file == sample_node_info.header_file

    def test_round_trip_preserves_callbacks(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert len(restored.callbacks) == len(sample_node_info.callbacks)
        assert restored.callbacks[0].name == "timer_callback"
        assert restored.callbacks[0].publishers_used == ["sensor_data"]
        assert restored.callbacks[0].variables_written == ["count_"]

    def test_round_trip_preserves_callback_parameters(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        topic_cb = restored.callbacks[1]
        assert len(topic_cb.parameters) == 1
        assert topic_cb.parameters[0].name == "msg"
        assert "SensorData" in topic_cb.parameters[0].type

    def test_round_trip_preserves_callback_groups(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert len(restored.callback_groups) == 2
        assert restored.callback_groups[0].type == "MutuallyExclusive"

    def test_round_trip_preserves_timers(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert restored.timers[0].period_ms == 500.0
        assert restored.timers[0].callback_group == "exclusive_group_"

    def test_round_trip_preserves_subscribers(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert restored.subscribers[0].topic == "sensor_data"
        assert restored.subscribers[0].qos_depth == 10
        assert restored.subscribers[0].callback_group == "reentrant_group_"

    def test_round_trip_preserves_services(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert restored.service_servers[0].service_name == "process_data"
        assert restored.service_clients[0].service_name == "process_data"

    def test_round_trip_preserves_actions(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert restored.action_servers[0].action_name == "analyze"
        assert restored.action_servers[0].goal_callback == "handle_goal"
        assert restored.action_clients[0].action_name == "analyze"

    def test_round_trip_preserves_parameters(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert len(restored.parameters) == 4
        names = [p.name for p in restored.parameters]
        assert "robot_name" in names
        assert "max_speed" in names
        # Check default values of various types
        robot_param = next(p for p in restored.parameters if p.name == "robot_name")
        assert robot_param.default_value == "my_robot"
        speed_param = next(p for p in restored.parameters if p.name == "max_speed")
        assert speed_param.default_value == 2.5

    def test_round_trip_preserves_metadata(self, sample_node_info):
        json_str = serialize(sample_node_info)
        restored = deserialize_node_info(json_str)
        assert restored.base_class == "rclcpp::Node"
        assert restored.is_component is False
        assert restored.has_param_callback is True
        assert restored.lifecycle_callbacks == ["on_configure", "on_activate"]
        assert "count_" in restored.member_variables

    def test_round_trip_preserves_none_header(self):
        node = NodeInfo(
            name="test",
            class_name="Test",
            package="test_pkg",
            source_file=Path("/test.cpp"),
            header_file=None,
        )
        json_str = serialize(node)
        restored = deserialize_node_info(json_str)
        assert restored.header_file is None


# ---------------------------------------------------------------------------
# Assertion 2: Round-trip SystemGraph
# ---------------------------------------------------------------------------

class TestSystemGraphRoundTrip:
    def test_serialize_produces_valid_json(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        data = json.loads(json_str)
        assert data["name"] == "bringup"

    def test_round_trip_preserves_name(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        assert restored.name == "bringup"

    def test_round_trip_preserves_nodes(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        assert len(restored.nodes) == 2
        assert "publisher_node" in restored.nodes
        assert "subscriber_node" in restored.nodes
        assert isinstance(restored.nodes["publisher_node"], NodeInfo)

    def test_round_trip_preserves_node_paths(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        pub = restored.nodes["publisher_node"]
        assert isinstance(pub.source_file, Path)

    def test_round_trip_preserves_topics(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        assert len(restored.communication.topics) == 1
        topic = restored.communication.topics[0]
        assert topic.topic == "sensor_data"
        assert topic.publishers == ["publisher_node"]
        assert topic.subscribers == ["subscriber_node"]

    def test_round_trip_preserves_services(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        assert len(restored.communication.services) == 1
        svc = restored.communication.services[0]
        assert svc.service == "process_data"

    def test_round_trip_preserves_actions(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        assert len(restored.communication.actions) == 1
        assert restored.communication.actions[0].action == "analyze"

    def test_round_trip_preserves_launch_graph(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        assert isinstance(restored.launch_graph, LaunchGraph)
        assert isinstance(restored.launch_graph.root, LaunchFileInfo)
        assert isinstance(restored.launch_graph.root.file_path, Path)
        assert restored.launch_graph.root.package == "pubsub_example"

    def test_round_trip_preserves_launch_includes(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        root = restored.launch_graph.root
        assert len(root.includes) == 2
        assert isinstance(root.includes[0], LaunchFileInfo)
        assert isinstance(root.includes[0].file_path, Path)

    def test_round_trip_preserves_launch_arguments(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        args = restored.launch_graph.root.arguments
        assert len(args) == 1
        assert args[0].name == "namespace"
        assert args[0].default_value == "robot_1"

    def test_round_trip_preserves_launch_groups(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        groups = restored.launch_graph.root.groups
        assert len(groups) == 1
        assert groups[0].namespace == "robot_1"
        assert len(groups[0].nodes) == 1

    def test_round_trip_preserves_remappings_as_tuples(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        node = restored.launch_graph.root.groups[0].nodes[0]
        assert len(node.remappings) == 1
        assert isinstance(node.remappings[0], tuple)
        assert node.remappings[0] == ("sensor_data", "/remapped_data")

    def test_round_trip_preserves_parameter_files(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        assert "params.yaml" in restored.parameter_files
        content = restored.parameter_files["params.yaml"]
        assert content["parameter_node"]["ros__parameters"]["robot_name"] == "ros2_lf_robot"

    def test_round_trip_preserves_all_nodes_list(self, sample_system_graph):
        json_str = serialize(sample_system_graph)
        restored = deserialize_system_graph(json_str)
        assert len(restored.launch_graph.all_nodes) == 2
        assert isinstance(restored.launch_graph.all_nodes[0], LaunchNodeAction)


# ---------------------------------------------------------------------------
# Assertion 3: Writer creates expected directory structure
# ---------------------------------------------------------------------------

class TestWriter:
    def test_creates_system_json(self, sample_system_graph, tmp_path):
        from colcon_lf.output.writer import write

        write(sample_system_graph, tmp_path / "out")
        system_json = tmp_path / "out" / "system.json"
        assert system_json.exists()
        data = json.loads(system_json.read_text())
        assert data["name"] == "bringup"

    def test_creates_node_json_files(self, sample_system_graph, tmp_path):
        from colcon_lf.output.writer import write

        write(sample_system_graph, tmp_path / "out")
        nodes_dir = tmp_path / "out" / "nodes"
        assert nodes_dir.is_dir()
        node_files = sorted(f.name for f in nodes_dir.iterdir())
        assert "publisher_node.json" in node_files
        assert "subscriber_node.json" in node_files

    def test_node_json_is_valid(self, sample_system_graph, tmp_path):
        from colcon_lf.output.writer import write

        write(sample_system_graph, tmp_path / "out")
        pub_json = tmp_path / "out" / "nodes" / "publisher_node.json"
        data = json.loads(pub_json.read_text())
        assert data["name"] == "publisher_node"
        assert data["class_name"] == "PublisherNode"

    def test_creates_config_yaml(self, sample_system_graph, tmp_path):
        from colcon_lf.output.writer import write

        write(sample_system_graph, tmp_path / "out")
        config_dir = tmp_path / "out" / "config"
        assert config_dir.is_dir()
        params = config_dir / "params.yaml"
        assert params.exists()
        import yaml
        content = yaml.safe_load(params.read_text())
        assert content["parameter_node"]["ros__parameters"]["robot_name"] == "ros2_lf_robot"

    def test_no_config_dir_when_no_params(self, tmp_path):
        from colcon_lf.output.writer import write

        graph = SystemGraph(
            name="empty",
            launch_graph=LaunchGraph(
                root=LaunchFileInfo(
                    file_path=Path("/test.launch.py"),
                    package="test",
                ),
            ),
        )
        write(graph, tmp_path / "out")
        assert not (tmp_path / "out" / "config").exists()

    def test_returns_output_path(self, sample_system_graph, tmp_path):
        from colcon_lf.output.writer import write

        result = write(sample_system_graph, tmp_path / "out")
        assert result == tmp_path / "out"
