# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for comm_analysis improvements, visualization enhancements, and output layer.

All tests use in-memory objects — no Docker, no libclang.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from typing import Any

import pytest

from colcon_lf.comm_analysis.models import (
    ActionConnection,
    ServiceConnection,
    SystemCommunication,
    TopicConnection,
)
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


# ===========================================================================
# Subscriber-only message type fallback
# ===========================================================================

class TestSubscriberOnlyMessageType:
    """Topic connections use subscriber message_type when publisher type is empty."""

    def test_subscriber_only_topic_has_type(self):
        """Subscriber-only topic gets message_type from subscriber."""
        sub_node = _node(
            "listener",
            subscribers=[SubscriberInfo(
                topic="data", message_type="std_msgs::msg::String", callback="cb",
            )],
        )
        comm = build_communication([(None, sub_node)])
        assert len(comm.topics) == 1
        assert comm.topics[0].message_type == "std_msgs::msg::String"
        assert comm.topics[0].subscribers == ["listener"]
        assert comm.topics[0].publishers == []

    def test_publisher_type_takes_precedence(self):
        """When both pub and sub have types, publisher type wins."""
        pub = _node("pub", publishers=[
            PublisherInfo(topic="t", message_type="PubType"),
        ])
        sub = _node("sub", subscribers=[
            SubscriberInfo(topic="t", message_type="SubType", callback="cb"),
        ])
        comm = build_communication([(None, pub), (None, sub)])
        assert comm.topics[0].message_type == "PubType"


# ===========================================================================
# Orphaned service/action clients
# ===========================================================================

class TestOrphanedServiceClients:
    """Service clients with no matching server are included in output."""

    def test_orphaned_service_client(self):
        """Service client with no server produces a connection with empty server."""
        client = _node("caller", service_clients=[
            ServiceClientInfo(service_name="missing_service",
                              service_type="std_srvs::srv::SetBool"),
        ])
        comm = build_communication([(None, client)])
        assert len(comm.services) == 1
        sc = comm.services[0]
        assert sc.service == "/missing_service"
        assert sc.server == ""
        assert sc.clients == ["caller"]
        assert sc.service_type == "std_srvs::srv::SetBool"

    def test_service_with_server_and_client(self):
        """Normal service connection still works."""
        server = _node("server", service_servers=[
            ServiceServerInfo(service_name="svc", service_type="T", callback="cb"),
        ])
        client = _node("client", service_clients=[
            ServiceClientInfo(service_name="svc", service_type="T"),
        ])
        comm = build_communication([(None, server), (None, client)])
        assert len(comm.services) == 1
        assert comm.services[0].server == "server"
        assert comm.services[0].clients == ["client"]


class TestOrphanedActionClients:
    """Action clients with no matching server are included in output."""

    def test_orphaned_action_client(self):
        """Action client with no server produces a connection with empty server."""
        client = _node("navigator", action_clients=[
            ActionClientInfo(action_name="navigate",
                             action_type="nav2::action::NavigateToPose"),
        ])
        comm = build_communication([(None, client)])
        assert len(comm.actions) == 1
        ac = comm.actions[0]
        assert ac.action == "/navigate"
        assert ac.server == ""
        assert ac.clients == ["navigator"]
        assert ac.action_type == "nav2::action::NavigateToPose"


# ===========================================================================
# Multiple publishers to one subscriber (fanout)
# ===========================================================================

class TestMultiplePublishers:
    """Multiple publishers to the same topic are captured."""

    def test_two_publishers_one_subscriber(self):
        """Two publishers and one subscriber all connected on same topic."""
        pub1 = _node("sensor_a", publishers=[
            PublisherInfo(topic="data", message_type="SensorData"),
        ])
        pub2 = _node("sensor_b", publishers=[
            PublisherInfo(topic="data", message_type="SensorData"),
        ])
        sub = _node("fusion", subscribers=[
            SubscriberInfo(topic="data", message_type="SensorData", callback="cb"),
        ])
        comm = build_communication([(None, pub1), (None, pub2), (None, sub)])
        assert len(comm.topics) == 1
        tc = comm.topics[0]
        assert set(tc.publishers) == {"sensor_a", "sensor_b"}
        assert tc.subscribers == ["fusion"]


# ===========================================================================
# Namespace isolation
# ===========================================================================

class TestNamespaceIsolation:
    """Same topic name in different namespaces stays separate."""

    def test_different_namespaces_no_match(self):
        """Pub in /ns1 and sub in /ns2 don't match."""
        pub = _node("pub", publishers=[
            PublisherInfo(topic="data", message_type="T"),
        ])
        sub = _node("sub", subscribers=[
            SubscriberInfo(topic="data", message_type="T", callback="cb"),
        ])
        comm = build_communication([
            (_launch(namespace="ns1"), pub),
            (_launch(namespace="ns2"), sub),
        ])
        assert len(comm.topics) == 2
        # Each topic should have only its own node
        for tc in comm.topics:
            assert len(tc.publishers) + len(tc.subscribers) == 1


# ===========================================================================
# Comm graph viz
# ===========================================================================

class TestCommGraphViz:
    """Test comm_graph_viz improvements."""

    def test_system_name_in_title(self):
        """System name appears as graph label."""
        from colcon_lf.visualization.comm_graph_viz import comm_graph_viz
        data = {"name": "my_system", "communication": {"topics": [], "services": [], "actions": []}}
        dot = comm_graph_viz(data)
        src = dot.source
        assert "my_system" in src

    def test_orphaned_publisher_shown(self):
        """Orphaned publisher produces a dashed edge to topic ellipse."""
        from colcon_lf.visualization.comm_graph_viz import comm_graph_viz
        data = {
            "name": "test",
            "communication": {
                "topics": [{"topic": "/orphan", "message_type": "T",
                            "publishers": ["pub_node"], "subscribers": []}],
                "services": [],
                "actions": [],
            },
        }
        dot = comm_graph_viz(data)
        src = dot.source
        assert "pub_node" in src
        assert "dashed" in src

    def test_orphaned_subscriber_shown(self):
        """Orphaned subscriber produces a dashed edge from topic ellipse."""
        from colcon_lf.visualization.comm_graph_viz import comm_graph_viz
        data = {
            "name": "test",
            "communication": {
                "topics": [{"topic": "/orphan", "message_type": "",
                            "publishers": [], "subscribers": ["sub_node"]}],
                "services": [],
                "actions": [],
            },
        }
        dot = comm_graph_viz(data)
        src = dot.source
        assert "sub_node" in src
        assert "dashed" in src

    def test_orphaned_service_client_shown(self):
        """Orphaned service client shown with (no server) diamond."""
        from colcon_lf.visualization.comm_graph_viz import comm_graph_viz
        data = {
            "name": "test",
            "communication": {
                "topics": [],
                "services": [{"service": "/missing_svc", "service_type": "T",
                              "server": "", "clients": ["caller"]}],
                "actions": [],
            },
        }
        dot = comm_graph_viz(data)
        src = dot.source
        assert "caller" in src
        assert "no server" in src


# ===========================================================================
# Writer tests
# ===========================================================================

class TestWriter:
    """Test output writer functionality."""

    def test_sanitize_filename(self):
        """Filenames are sanitized for cross-platform safety."""
        from colcon_lf.output.writer import _sanitize_filename
        assert _sanitize_filename("/robot_1/node") == "robot_1_node"
        assert _sanitize_filename("node:instance") == "node_instance"
        assert _sanitize_filename("node<1>") == "node_1_"
        assert _sanitize_filename("") == "unnamed"

    def test_write_creates_structure(self):
        """write() creates system.json and nodes/ directory."""
        from colcon_lf.graph.models import SystemGraph
        from colcon_lf.launch_analysis.models import LaunchGraph, LaunchFileInfo
        from colcon_lf.output.writer import write

        graph = SystemGraph(
            name="test",
            launch_graph=LaunchGraph(
                root=LaunchFileInfo(file_path=Path("/dev/null"), package="test"),
            ),
            nodes={"test_node": _node("test_node")},
            communication=SystemCommunication(),
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            result = write(graph, tmpdir)
            assert (result / "system.json").exists()
            assert (result / "nodes" / "test_node.json").exists()
            # Verify system.json is valid JSON
            data = json.loads((result / "system.json").read_text())
            assert data["name"] == "test"

    def test_write_never_deletes_existing_config_dir(self):
        """write() must never rmtree the output dir — a re-run of lf-analyze
        would otherwise destroy the user's exec-config files living in
        config/ (00_design.md section 2.1; review nit)."""
        from colcon_lf.graph.models import SystemGraph
        from colcon_lf.launch_analysis.models import LaunchGraph, LaunchFileInfo
        from colcon_lf.output.writer import write

        graph = SystemGraph(
            name="test",
            launch_graph=LaunchGraph(
                root=LaunchFileInfo(file_path=Path("/dev/null"), package="test"),
            ),
            nodes={"test_node": _node("test_node")},
            communication=SystemCommunication(),
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            config_dir = Path(tmpdir) / "config"
            config_dir.mkdir()
            user_file = config_dir / "execution.yaml"
            user_file.write_text("schema_version: 1\ndefaults: {}\n")

            # Re-running write() (as lf-analyze does on every invocation)
            # must not touch the user's config/ files.
            write(graph, tmpdir)
            write(graph, tmpdir)

            assert user_file.is_file()
            assert user_file.read_text() == "schema_version: 1\ndefaults: {}\n"


# ===========================================================================
# Serializer round-trip tests
# ===========================================================================

class TestSerializerRoundTrip:
    """Test serialize -> deserialize round-trip."""

    def test_node_info_round_trip(self):
        """NodeInfo survives serialize -> deserialize."""
        from colcon_lf.output.serializer import serialize, deserialize_node_info
        from colcon_lf.node_analysis.models import ParameterInfo

        node = _node("test_node", publishers=[
            PublisherInfo(topic="data", message_type="std_msgs::msg::String"),
        ])
        node.parameters.append(ParameterInfo(name="rate", type="int", default_value=10))

        json_str = serialize(node)
        restored = deserialize_node_info(json_str)

        assert restored.name == "test_node"
        assert len(restored.publishers) == 1
        assert restored.publishers[0].topic == "data"
        assert len(restored.parameters) == 1
        assert restored.parameters[0].default_value == 10
