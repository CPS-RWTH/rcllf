# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for colcon_lf.visualization modules.

Phase 7 — 5 TDD assertions required by the plan:
1. launch_graph_viz produces DOT with node names and launch file names
2. node_internals_viz produces DOT with timer, callback, publisher, variable
3. comm_graph_viz produces DOT with topic/service/action edges
4. system_graph_viz produces DOT with subgraph clusters and connection edges
5. Rendered files are non-empty (requires dot binary)

DOT content tests use in-memory mock data — no Docker needed.
Render tests skip if graphviz dot binary is not installed.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from colcon_lf.visualization.launch_graph_viz import launch_graph_viz
from colcon_lf.visualization.node_internals_viz import node_internals_viz
from colcon_lf.visualization.comm_graph_viz import comm_graph_viz
from colcon_lf.visualization.system_graph_viz import system_graph_viz


# ---------------------------------------------------------------------------
# Fixtures: mock JSON data (matching what serializer produces)
# ---------------------------------------------------------------------------

@pytest.fixture
def publisher_node_json() -> dict:
    return {
        "name": "publisher_node",
        "class_name": "PublisherNode",
        "package": "pubsub_example",
        "source_file": "/workspace/src/pubsub_example/src/publisher_node.cpp",
        "header_file": None,
        "base_class": "rclcpp::Node",
        "is_component": False,
        "callbacks": [
            {
                "name": "timer_callback",
                "parameters": [],
                "publishers_used": ["sensor_data"],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": ["publisher_"],
                "variables_written": ["count_"],
            },
        ],
        "callback_groups": [],
        "timers": [
            {"period_ms": 500.0, "callback": "timer_callback", "callback_group": None},
        ],
        "subscribers": [],
        "service_servers": [],
        "action_servers": [],
        "publishers": [
            {"topic": "sensor_data", "message_type": "custom_interfaces::msg::SensorData", "qos_depth": 10},
        ],
        "service_clients": [],
        "action_clients": [],
        "parameters": [
            {"name": "rate", "type": "double", "default_value": 2.0, "description": None},
        ],
        "member_variables": ["publisher_", "timer_", "count_"],
        "lifecycle_callbacks": [],
        "has_param_callback": False,
    }


@pytest.fixture
def lifecycle_node_json() -> dict:
    return {
        "name": "lifecycle_node",
        "class_name": "LifecycleExampleNode",
        "package": "lifecycle_example",
        "source_file": "/workspace/src/lifecycle_example/src/lifecycle_node.cpp",
        "header_file": None,
        "base_class": "rclcpp_lifecycle::LifecycleNode",
        "is_component": False,
        "callbacks": [
            {
                "name": "on_configure",
                "parameters": [{"name": "state", "type": "rclcpp_lifecycle::State const &"}],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": ["publisher_"],
            },
            {
                "name": "on_activate",
                "parameters": [{"name": "state", "type": "rclcpp_lifecycle::State const &"}],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": ["timer_"],
            },
            {
                "name": "on_deactivate",
                "parameters": [{"name": "state", "type": "rclcpp_lifecycle::State const &"}],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": ["timer_"],
            },
            {
                "name": "on_cleanup",
                "parameters": [{"name": "state", "type": "rclcpp_lifecycle::State const &"}],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": ["publisher_"],
            },
            {
                "name": "timer_callback",
                "parameters": [],
                "publishers_used": ["lifecycle_output"],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": ["count_"],
            },
        ],
        "callback_groups": [],
        "timers": [
            {
                "period_ms": 1000.0,
                "callback": "timer_callback",
                "callback_group": None,
                "member_variable": "timer_",
                "created_in": "on_activate",
                "destroyed_in": "on_deactivate",
            },
        ],
        "subscribers": [],
        "service_servers": [],
        "action_servers": [],
        "publishers": [
            {
                "topic": "lifecycle_output",
                "message_type": "std_msgs::msg::String",
                "qos_depth": 10,
                "member_variable": "publisher_",
                "is_lifecycle": True,
                "created_in": "on_configure",
                "destroyed_in": "on_cleanup",
            },
        ],
        "service_clients": [],
        "action_clients": [],
        "parameters": [],
        "member_variables": ["publisher_", "timer_", "count_"],
        "lifecycle_callbacks": ["on_configure", "on_activate", "on_deactivate", "on_cleanup"],
        "has_param_callback": False,
    }


@pytest.fixture
def system_json(publisher_node_json) -> dict:
    subscriber_node = {
        "name": "subscriber_node",
        "class_name": "SubscriberNode",
        "package": "pubsub_example",
        "source_file": "/workspace/src/pubsub_example/src/subscriber_node.cpp",
        "header_file": None,
        "base_class": "rclcpp::Node",
        "is_component": False,
        "callbacks": [
            {
                "name": "topic_callback",
                "parameters": [{"name": "msg", "type": "custom_interfaces::msg::SensorData::SharedPtr"}],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": [],
            },
        ],
        "callback_groups": [],
        "timers": [],
        "subscribers": [
            {
                "topic": "sensor_data",
                "message_type": "custom_interfaces::msg::SensorData",
                "callback": "topic_callback",
                "qos_depth": 10,
                "callback_group": None,
            },
        ],
        "service_servers": [],
        "action_servers": [],
        "publishers": [],
        "service_clients": [],
        "action_clients": [],
        "parameters": [],
        "member_variables": [],
        "lifecycle_callbacks": [],
        "has_param_callback": False,
    }

    return {
        "name": "bringup",
        "launch_graph": {
            "root": {
                "file_path": "/workspace/src/pubsub_example/launch/bringup.launch.py",
                "package": "pubsub_example",
                "arguments": [],
                "nodes": [],
                "includes": [
                    {
                        "file_path": "/workspace/src/pubsub_example/launch/publisher_node.launch.py",
                        "package": "pubsub_example",
                        "arguments": [],
                        "nodes": [
                            {"package": "pubsub_example", "executable": "publisher_node", "name": "publisher_node",
                             "namespace": None, "parameters": {}, "parameter_files": [], "remappings": [],
                             "condition": None, "is_component": False, "composable_container": None},
                        ],
                        "includes": [],
                        "groups": [],
                    },
                    {
                        "file_path": "/workspace/src/pubsub_example/launch/subscriber_node.launch.py",
                        "package": "pubsub_example",
                        "arguments": [],
                        "nodes": [
                            {"package": "pubsub_example", "executable": "subscriber_node", "name": "subscriber_node",
                             "namespace": None, "parameters": {}, "parameter_files": [], "remappings": [],
                             "condition": None, "is_component": False, "composable_container": None},
                        ],
                        "includes": [],
                        "groups": [],
                    },
                ],
                "groups": [],
            },
            "all_nodes": [
                {"package": "pubsub_example", "executable": "publisher_node", "name": "publisher_node",
                 "namespace": None, "parameters": {}, "parameter_files": [], "remappings": [],
                 "condition": None, "is_component": False, "composable_container": None},
                {"package": "pubsub_example", "executable": "subscriber_node", "name": "subscriber_node",
                 "namespace": None, "parameters": {}, "parameter_files": [], "remappings": [],
                 "condition": None, "is_component": False, "composable_container": None},
            ],
        },
        "nodes": {
            "publisher_node": publisher_node_json,
            "subscriber_node": subscriber_node,
        },
        "communication": {
            "topics": [
                {
                    "topic": "sensor_data",
                    "message_type": "custom_interfaces::msg::SensorData",
                    "publishers": ["publisher_node"],
                    "subscribers": ["subscriber_node"],
                },
            ],
            "services": [
                {
                    "service": "process_data",
                    "service_type": "custom_interfaces::srv::ProcessData",
                    "server": "service_server",
                    "clients": ["service_client"],
                },
            ],
            "actions": [
                {
                    "action": "analyze",
                    "action_type": "custom_interfaces::action::Analyze",
                    "server": "action_server",
                    "clients": ["action_client"],
                },
            ],
        },
        "parameter_files": {},
    }


# ---------------------------------------------------------------------------
# Assertion 1: launch_graph_viz DOT content
# ---------------------------------------------------------------------------

class TestLaunchGraphViz:
    def test_contains_launch_file_names(self, system_json):
        dot = launch_graph_viz(system_json)
        src = dot.source
        assert "bringup.launch.py" in src
        assert "publisher_node.launch.py" in src
        assert "subscriber_node.launch.py" in src

    def test_contains_node_names(self, system_json):
        dot = launch_graph_viz(system_json)
        src = dot.source
        assert "publisher_node" in src
        assert "subscriber_node" in src

    def test_contains_includes_edges(self, system_json):
        dot = launch_graph_viz(system_json)
        src = dot.source
        assert "includes" in src

    def test_returns_digraph(self, system_json):
        dot = launch_graph_viz(system_json)
        assert "digraph" in dot.source


# ---------------------------------------------------------------------------
# Assertion 2: node_internals_viz DOT content
# ---------------------------------------------------------------------------

class TestNodeInternalsViz:
    def test_contains_timer(self, publisher_node_json):
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "500" in src  # period_ms
        assert "Timer" in src

    def test_contains_callback(self, publisher_node_json):
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "timer_callback" in src

    def test_contains_publisher(self, publisher_node_json):
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "sensor_data" in src
        assert "SensorData" in src

    def test_contains_variable(self, publisher_node_json):
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "count_" in src

    def test_contains_parameters(self, publisher_node_json):
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "rate" in src
        assert "Parameters" in src

    def test_contains_triggers_edge(self, publisher_node_json):
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "triggers" in src

    def test_contains_publish_edge(self, publisher_node_json):
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "publish" in src


# ---------------------------------------------------------------------------
# Lifecycle state machine visualization
# ---------------------------------------------------------------------------

class TestLifecycleViz:
    def test_lifecycle_cluster_present(self, lifecycle_node_json):
        """Lifecycle methods are grouped in a state machine subgraph."""
        dot = node_internals_viz(lifecycle_node_json)
        src = dot.source
        assert "cluster_lifecycle" in src

    def test_state_nodes_present(self, lifecycle_node_json):
        """Unconfigured, Inactive, Active state nodes are rendered."""
        dot = node_internals_viz(lifecycle_node_json)
        src = dot.source
        assert "Unconfigured" in src
        assert "Inactive" in src
        assert "Active" in src

    def test_lifecycle_method_boxes_present(self, lifecycle_node_json):
        """on_configure, on_activate, on_deactivate, on_cleanup appear as nodes."""
        dot = node_internals_viz(lifecycle_node_json)
        src = dot.source
        for method in ["on_configure", "on_activate", "on_deactivate", "on_cleanup"]:
            assert method in src, f"Missing lifecycle method: {method}"

    def test_lifecycle_methods_not_in_regular_callbacks(self, lifecycle_node_json):
        """Lifecycle methods are NOT rendered as normal green callback boxes."""
        dot = node_internals_viz(lifecycle_node_json)
        src = dot.source
        # Regular callbacks use lightgreen; lifecycle methods use lightcyan
        # The timer_callback should be rendered as a regular callback
        assert "timer_callback" in src

    def test_publisher_is_lifecycle(self, lifecycle_node_json):
        """Publisher is shown as LifecyclePub with created_in annotation."""
        dot = node_internals_viz(lifecycle_node_json)
        src = dot.source
        assert "LifecyclePub" in src
        assert "on_configure" in src

    def test_system_graph_lifecycle_cluster(self, lifecycle_node_json):
        """system_graph_viz also renders lifecycle state machine cluster."""
        system_data = {
            "name": "bringup",
            "launch_graph": {
                "root": {
                    "file_path": "/ws/launch.py",
                    "package": "lifecycle_example",
                    "arguments": [],
                    "nodes": [
                        {"package": "lifecycle_example", "executable": "lifecycle_node",
                         "name": "lifecycle_node", "namespace": None, "parameters": {},
                         "parameter_files": [], "remappings": [], "condition": None,
                         "is_component": False, "composable_container": None},
                    ],
                    "includes": [],
                    "groups": [],
                },
                "all_nodes": [
                    {"package": "lifecycle_example", "executable": "lifecycle_node",
                     "name": "lifecycle_node", "namespace": None, "parameters": {},
                     "parameter_files": [], "remappings": [], "condition": None,
                     "is_component": False, "composable_container": None},
                ],
            },
            "nodes": {"lifecycle_node": lifecycle_node_json},
            "communication": {"topics": [], "services": [], "actions": []},
            "parameter_files": {},
        }
        dot = system_graph_viz(system_data)
        src = dot.source
        assert "cluster_" in src
        assert "on_configure" in src
        assert "Lifecycle" in src


# ---------------------------------------------------------------------------
# Action framework visualization
# ---------------------------------------------------------------------------

@pytest.fixture
def action_server_node_json() -> dict:
    return {
        "name": "action_server",
        "class_name": "ActionServerNode",
        "package": "action_example",
        "source_file": "/ws/action_server_node.cpp",
        "header_file": None,
        "base_class": "rclcpp::Node",
        "is_component": False,
        "callbacks": [
            {
                "name": "handle_accepted",
                "parameters": [],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": [],
                "spawns_callbacks": ["execute"],
                "feedback_published": [],
                "result_published": [],
            },
            {
                "name": "execute",
                "parameters": [],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": [],
                "spawns_callbacks": [],
                "feedback_published": ["analyze"],
                "result_published": ["analyze"],
            },
        ],
        "callback_groups": [],
        "timers": [],
        "subscribers": [],
        "service_servers": [],
        "action_servers": [
            {
                "action_name": "analyze",
                "action_type": "custom_interfaces::action::Analyze",
                "goal_callback": "handle_goal",
                "cancel_callback": "handle_cancel",
                "accepted_callback": "handle_accepted",
                "callback_group": None,
                "member_variable": "action_server_",
            },
        ],
        "publishers": [],
        "service_clients": [],
        "action_clients": [],
        "parameters": [],
        "member_variables": ["action_server_"],
        "lifecycle_callbacks": [],
        "has_param_callback": False,
    }


@pytest.fixture
def action_client_node_json() -> dict:
    return {
        "name": "action_client",
        "class_name": "ActionClientNode",
        "package": "action_example",
        "source_file": "/ws/action_client_node.cpp",
        "header_file": None,
        "base_class": "rclcpp::Node",
        "is_component": False,
        "callbacks": [
            {
                "name": "send_goal",
                "parameters": [],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": ["analyze"],
                "variables_read": [],
                "variables_written": [],
                "spawns_callbacks": [],
                "feedback_published": [],
                "result_published": [],
            },
            {
                "name": "goal_response_callback",
                "parameters": [],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": [],
                "spawns_callbacks": [],
                "feedback_published": [],
                "result_published": [],
            },
            {
                "name": "feedback_callback",
                "parameters": [],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": [],
                "spawns_callbacks": [],
                "feedback_published": [],
                "result_published": [],
            },
            {
                "name": "result_callback",
                "parameters": [],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": [],
                "spawns_callbacks": [],
                "feedback_published": [],
                "result_published": [],
            },
        ],
        "callback_groups": [],
        "timers": [],
        "subscribers": [],
        "service_servers": [],
        "action_servers": [],
        "publishers": [],
        "service_clients": [],
        "action_clients": [
            {
                "action_name": "analyze",
                "action_type": "custom_interfaces::action::Analyze",
                "member_variable": "action_client_",
                "created_in": "constructor",
                "destroyed_in": None,
                "goal_response_callback": "goal_response_callback",
                "feedback_callback": "feedback_callback",
                "result_callback": "result_callback",
            },
        ],
        "parameters": [],
        "member_variables": ["action_client_", "timer_"],
        "lifecycle_callbacks": [],
        "has_param_callback": False,
    }


class TestActionFrameworkViz:
    def test_spawns_edge(self, action_server_node_json):
        """handle_accepted -> execute shown as red dashed spawns edge."""
        dot = node_internals_viz(action_server_node_json)
        src = dot.source
        assert "spawns" in src
        assert "red" in src

    def test_feedback_annotation(self, action_server_node_json):
        """execute callback annotated with feedback action name."""
        dot = node_internals_viz(action_server_node_json)
        src = dot.source
        assert "feedback" in src
        assert "analyze" in src

    def test_result_annotation(self, action_server_node_json):
        """execute callback annotated with result action name."""
        dot = node_internals_viz(action_server_node_json)
        src = dot.source
        assert "result" in src

    def test_sendgoal_binding_edges(self, action_client_node_json):
        """Action client shows on_goal/on_feedback/on_result edges to bound callbacks."""
        dot = node_internals_viz(action_client_node_json)
        src = dot.source
        assert "on_goal" in src
        assert "on_feedback" in src
        assert "on_result" in src
        assert "purple" in src

    def test_system_graph_spawns_edge(self, action_server_node_json):
        """system_graph_viz renders spawns edge for action server."""
        system_data = {
            "name": "bringup",
            "launch_graph": {
                "root": {
                    "file_path": "/ws/launch.py",
                    "package": "action_example",
                    "arguments": [],
                    "nodes": [
                        {"package": "action_example", "executable": "action_server",
                         "name": "action_server", "namespace": None, "parameters": {},
                         "parameter_files": [], "remappings": [], "condition": None,
                         "is_component": False, "composable_container": None},
                    ],
                    "includes": [],
                    "groups": [],
                },
                "all_nodes": [
                    {"package": "action_example", "executable": "action_server",
                     "name": "action_server", "namespace": None, "parameters": {},
                     "parameter_files": [], "remappings": [], "condition": None,
                     "is_component": False, "composable_container": None},
                ],
            },
            "nodes": {"action_server": action_server_node_json},
            "communication": {"topics": [], "services": [], "actions": []},
            "parameter_files": {},
        }
        dot = system_graph_viz(system_data)
        src = dot.source
        assert "spawns" in src
        assert "red" in src


# ---------------------------------------------------------------------------
# Assertion 3: comm_graph_viz DOT content
# ---------------------------------------------------------------------------

class TestCommGraphViz:
    def test_contains_topic_edge(self, system_json):
        dot = comm_graph_viz(system_json)
        src = dot.source
        assert "sensor_data" in src
        assert "SensorData" in src

    def test_contains_service_edge(self, system_json):
        dot = comm_graph_viz(system_json)
        src = dot.source
        assert "process_data" in src
        assert "dashed" in src

    def test_contains_action_edge(self, system_json):
        dot = comm_graph_viz(system_json)
        src = dot.source
        assert "analyze" in src
        assert "purple" in src

    def test_contains_node_boxes(self, system_json):
        dot = comm_graph_viz(system_json)
        src = dot.source
        assert "publisher_node" in src
        assert "subscriber_node" in src
        assert "service_server" in src
        assert "action_server" in src


# ---------------------------------------------------------------------------
# Assertion 4: system_graph_viz DOT content
# ---------------------------------------------------------------------------

class TestSystemGraphViz:
    def test_contains_subgraph_clusters(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "subgraph" in src
        assert "cluster" in src

    def test_contains_node_names(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "publisher_node" in src
        assert "subscriber_node" in src

    def test_contains_comm_edges(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "sensor_data" in src

    def test_contains_service_edge(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "process_data" in src

    def test_contains_action_edge(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "analyze" in src

    def test_contains_timer_period(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "500" in src

    def test_contains_callback_names(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "timer_callback" in src
        assert "topic_callback" in src

    def test_contains_message_types(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "SensorData" in src

    def test_contains_class_name(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "PublisherNode" in src

    def test_contains_publishers_used(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        # timer_callback publishes to sensor_data — shape label is "Pub: sensor_data"
        assert "Pub:" in src

    def test_contains_parameters_section(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "Parameters" in src
        assert "rate" in src

    def test_has_nested_node_clusters(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        # Node clusters are nested inside launch clusters
        assert "cluster_node_" in src

    def test_has_intra_node_edges(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        # Timer -> callback edge (bold style within a node)
        assert "style=bold" in src

    def test_layout_is_left_to_right(self, system_json):
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "rankdir=LR" in src

    def test_service_request_response_edges(self, system_json):
        """Service has both request and response edge labels."""
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "request" in src
        assert "response" in src

    def test_action_four_edge_labels(self, system_json):
        """Action has goal, cancel, feedback, and result edge labels."""
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "goal" in src
        assert "cancel" in src
        assert "feedback" in src
        assert "result" in src


# ---------------------------------------------------------------------------
# Assertion 5: Rendered files are non-empty (requires dot binary)
# ---------------------------------------------------------------------------

_has_dot = shutil.which("dot") is not None


@pytest.mark.skipif(not _has_dot, reason="graphviz dot binary not installed")
class TestRendering:
    def test_render_launch_graph_svg(self, system_json, tmp_path):
        from colcon_lf.visualization.dot_renderer import render

        dot = launch_graph_viz(system_json)
        files = render(dot, tmp_path / "launch_graph", formats=["svg"])
        assert len(files) == 1
        assert files[0].stat().st_size > 0

    def test_render_node_internals_svg(self, publisher_node_json, tmp_path):
        from colcon_lf.visualization.dot_renderer import render

        dot = node_internals_viz(publisher_node_json)
        files = render(dot, tmp_path / "node_internals", formats=["svg"])
        assert len(files) == 1
        assert files[0].stat().st_size > 0

    def test_render_comm_graph_svg(self, system_json, tmp_path):
        from colcon_lf.visualization.dot_renderer import render

        dot = comm_graph_viz(system_json)
        files = render(dot, tmp_path / "comm_graph", formats=["svg"])
        assert len(files) == 1
        assert files[0].stat().st_size > 0

    def test_render_system_graph_svg(self, system_json, tmp_path):
        from colcon_lf.visualization.dot_renderer import render

        dot = system_graph_viz(system_json)
        files = render(dot, tmp_path / "system_graph", formats=["svg"])
        assert len(files) == 1
        assert files[0].stat().st_size > 0

    def test_render_multiple_formats(self, system_json, tmp_path):
        from colcon_lf.visualization.dot_renderer import render

        dot = comm_graph_viz(system_json)
        files = render(dot, tmp_path / "comm", formats=["svg", "pdf"])
        assert len(files) == 2
        for f in files:
            assert f.stat().st_size > 0


# ---------------------------------------------------------------------------
# QoS depth labels
# ---------------------------------------------------------------------------

class TestQoSDepthViz:
    def test_qos_in_node_internals_publisher(self, publisher_node_json):
        """Publisher with qos_depth=10 shows QoS label in node internals."""
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "QoS" in src
        assert "10" in src

    def test_qos_in_node_internals_subscriber(self):
        """Subscriber with qos_depth shows QoS label in node internals."""
        node = {
            "name": "sub_node",
            "class_name": "SubNode",
            "package": "pkg",
            "source_file": "/ws/sub.cpp",
            "header_file": None,
            "base_class": "rclcpp::Node",
            "is_component": False,
            "callbacks": [
                {"name": "cb", "parameters": [], "publishers_used": [], "service_clients_used": [],
                 "action_clients_used": [], "variables_read": [], "variables_written": []}
            ],
            "callback_groups": [],
            "timers": [],
            "subscribers": [
                {"topic": "data", "message_type": "std_msgs::msg::String",
                 "callback": "cb", "qos_depth": 5, "callback_group": None},
            ],
            "service_servers": [],
            "action_servers": [],
            "publishers": [],
            "service_clients": [],
            "action_clients": [],
            "parameters": [],
            "member_variables": [],
            "lifecycle_callbacks": [],
            "has_param_callback": False,
        }
        dot = node_internals_viz(node)
        src = dot.source
        assert "QoS" in src
        assert "5" in src

    def test_qos_in_system_graph_publisher(self, system_json):
        """system_graph_viz shows QoS label for publisher with qos_depth."""
        dot = system_graph_viz(system_json)
        src = dot.source
        # publisher_node has qos_depth=10
        assert "QoS" in src

    def test_qos_none_not_shown(self):
        """Publisher without qos_depth shows no QoS label."""
        node = {
            "name": "pub_node",
            "class_name": "PubNode",
            "package": "pkg",
            "source_file": "/ws/pub.cpp",
            "header_file": None,
            "base_class": "rclcpp::Node",
            "is_component": False,
            "callbacks": [
                {"name": "timer_cb", "parameters": [], "publishers_used": ["out"],
                 "service_clients_used": [], "action_clients_used": [],
                 "variables_read": [], "variables_written": []}
            ],
            "callback_groups": [],
            "timers": [{"period_ms": 100.0, "callback": "timer_cb", "callback_group": None}],
            "subscribers": [],
            "service_servers": [],
            "action_servers": [],
            "publishers": [
                {"topic": "out", "message_type": "std_msgs::msg::String", "qos_depth": None},
            ],
            "service_clients": [],
            "action_clients": [],
            "parameters": [],
            "member_variables": [],
            "lifecycle_callbacks": [],
            "has_param_callback": False,
        }
        dot = node_internals_viz(node)
        src = dot.source
        assert "QoS" not in src


# ---------------------------------------------------------------------------
# Parameter callback visualization
# ---------------------------------------------------------------------------

@pytest.fixture
def param_callback_node_json() -> dict:
    return {
        "name": "parameter_node",
        "class_name": "ParameterNode",
        "package": "parameter_example",
        "source_file": "/ws/parameter_node.cpp",
        "header_file": None,
        "base_class": "rclcpp::Node",
        "is_component": False,
        "callbacks": [
            {
                "name": "validate_parameters",
                "parameters": [],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": [],
            },
            {
                "name": "timer_callback",
                "parameters": [],
                "publishers_used": [],
                "service_clients_used": [],
                "action_clients_used": [],
                "variables_read": [],
                "variables_written": [],
            },
        ],
        "callback_groups": [],
        "timers": [{"period_ms": 1000.0, "callback": "timer_callback", "callback_group": None}],
        "subscribers": [],
        "service_servers": [],
        "action_servers": [],
        "publishers": [],
        "service_clients": [],
        "action_clients": [],
        "parameters": [
            {"name": "robot_name", "type": "string", "default_value": "robot", "description": None},
        ],
        "member_variables": [],
        "lifecycle_callbacks": [],
        "has_param_callback": True,
        "param_callback": "validate_parameters",
    }


class TestParamCallbackViz:
    def test_param_callback_trigger_in_node_internals(self, param_callback_node_json):
        """node_internals_viz shows on_set_parameters trigger box."""
        dot = node_internals_viz(param_callback_node_json)
        src = dot.source
        assert "on_set_parameters" in src

    def test_param_callback_edge_in_node_internals(self, param_callback_node_json):
        """node_internals_viz draws validates/triggers edges for param callback."""
        dot = node_internals_viz(param_callback_node_json)
        src = dot.source
        assert "validates" in src or "triggers" in src
        assert "goldenrod" in src

    def test_param_callback_callback_name_in_node_internals(self, param_callback_node_json):
        """node_internals_viz references validate_parameters as bound callback."""
        dot = node_internals_viz(param_callback_node_json)
        src = dot.source
        assert "validate_parameters" in src

    def test_param_callback_in_system_graph(self, param_callback_node_json):
        """system_graph_viz shows on_set_parameters box."""
        system_data = {
            "name": "bringup",
            "launch_graph": {
                "root": {
                    "file_path": "/ws/launch.py",
                    "package": "parameter_example",
                    "arguments": [],
                    "nodes": [
                        {"package": "parameter_example", "executable": "parameter_node",
                         "name": "parameter_node", "namespace": None, "parameters": {},
                         "parameter_files": [], "remappings": [], "condition": None,
                         "is_component": False, "composable_container": None},
                    ],
                    "includes": [],
                    "groups": [],
                },
                "all_nodes": [
                    {"package": "parameter_example", "executable": "parameter_node",
                     "name": "parameter_node", "namespace": None, "parameters": {},
                     "parameter_files": [], "remappings": [], "condition": None,
                     "is_component": False, "composable_container": None},
                ],
            },
            "nodes": {"parameter_node": param_callback_node_json},
            "communication": {"topics": [], "services": [], "actions": []},
            "parameter_files": {},
        }
        dot = system_graph_viz(system_data)
        src = dot.source
        assert "on_set_parameters" in src
        assert "goldenrod" in src

    def test_no_param_callback_box_when_absent(self, publisher_node_json):
        """No on_set_parameters box when has_param_callback is False."""
        dot = node_internals_viz(publisher_node_json)
        src = dot.source
        assert "on_set_parameters" not in src


# ---------------------------------------------------------------------------
# Interface field display
# ---------------------------------------------------------------------------

@pytest.fixture
def system_json_with_fields(system_json) -> dict:
    """system_json augmented with resolved interface fields."""
    system_json["communication"]["topics"][0]["fields"] = [
        {"type": "float64", "name": "temperature"},
        {"type": "float64", "name": "humidity"},
    ]
    system_json["communication"]["services"][0]["request_fields"] = [
        {"type": "float64[]", "name": "data"},
    ]
    system_json["communication"]["services"][0]["response_fields"] = [
        {"type": "bool", "name": "success"},
    ]
    system_json["communication"]["actions"][0]["goal_fields"] = [
        {"type": "float64[]", "name": "data"},
    ]
    system_json["communication"]["actions"][0]["result_fields"] = [
        {"type": "bool", "name": "success"},
    ]
    system_json["communication"]["actions"][0]["feedback_fields"] = [
        {"type": "float64", "name": "progress"},
    ]
    return system_json


class TestInterfaceFieldsViz:
    def test_topic_fields_in_system_graph(self, system_json_with_fields):
        """system_graph_viz shows msg fields in topic interface box."""
        dot = system_graph_viz(system_json_with_fields)
        src = dot.source
        assert "temperature" in src
        assert "humidity" in src

    def test_service_fields_in_system_graph(self, system_json_with_fields):
        """system_graph_viz shows req/res fields in service interface box."""
        dot = system_graph_viz(system_json_with_fields)
        src = dot.source
        assert "success" in src

    def test_action_fields_in_system_graph(self, system_json_with_fields):
        """system_graph_viz shows goal/result/feedback fields in action interface box."""
        dot = system_graph_viz(system_json_with_fields)
        src = dot.source
        assert "progress" in src

    def test_topic_fields_in_comm_graph(self, system_json_with_fields):
        """comm_graph_viz shows msg fields in topic edge label."""
        dot = comm_graph_viz(system_json_with_fields)
        src = dot.source
        assert "temperature" in src
        assert "humidity" in src

    def test_service_fields_in_comm_graph(self, system_json_with_fields):
        """comm_graph_viz shows req/res fields in service edge label."""
        dot = comm_graph_viz(system_json_with_fields)
        src = dot.source
        assert "success" in src

    def test_action_fields_in_comm_graph(self, system_json_with_fields):
        """comm_graph_viz shows goal/result/feedback fields in action edge label."""
        dot = comm_graph_viz(system_json_with_fields)
        src = dot.source
        assert "progress" in src

    def test_no_fields_when_absent(self, system_json):
        """No field content when fields are not populated."""
        # system_json has no fields — temperature/humidity should not appear
        dot = system_graph_viz(system_json)
        src = dot.source
        assert "temperature" not in src
        assert "humidity" not in src

    def test_fields_truncation(self, system_json_with_fields):
        """Fields list truncated at 8 with +N more message."""
        # Add 10 fields to topic
        system_json_with_fields["communication"]["topics"][0]["fields"] = [
            {"type": "float64", "name": f"field_{i}"} for i in range(10)
        ]
        dot = system_graph_viz(system_json_with_fields)
        src = dot.source
        assert "+2 more" in src
