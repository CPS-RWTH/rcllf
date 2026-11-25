# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for node analysis gap fixes (Gaps 1-5).

Gap 1: Helper method traversal
Gap 2: Parameter-to-topic resolution (in-memory unit tests)
Gap 3: Typedef alias resolution (in-memory unit tests)
Gap 4: add_parameter() pattern
Gap 5: Wrapper message types

Integration tests (Gap 1, 4) require a built workspace with libclang.
Unit tests (Gap 2, 3, 5) use in-memory objects only.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from colcon_lf.node_analysis.models import (
    NodeInfo,
    ParameterInfo,
    PublisherInfo,
    SubscriberInfo,
    ServiceServerInfo,
    ServiceClientInfo,
    ActionServerInfo,
    ActionClientInfo,
)
from colcon_lf.launch_analysis.models import LaunchNodeAction

# Skip integration tests if libclang is not available
try:
    import clang.cindex  # noqa: F401
    HAS_LIBCLANG = True
except ImportError:
    HAS_LIBCLANG = False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _node_info(
    name: str,
    publishers: list[PublisherInfo] | None = None,
    subscribers: list[SubscriberInfo] | None = None,
    service_servers: list[ServiceServerInfo] | None = None,
    service_clients: list[ServiceClientInfo] | None = None,
    action_servers: list[ActionServerInfo] | None = None,
    action_clients: list[ActionClientInfo] | None = None,
    parameters: list[ParameterInfo] | None = None,
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
        parameters=parameters or [],
    )


def _launch_node(name: str, **kwargs: Any) -> LaunchNodeAction:
    return LaunchNodeAction(
        package="test_pkg",
        executable=name,
        name=name,
        **kwargs,
    )


# ===========================================================================
# Gap 2: Parameter-to-topic resolution — unit tests (in-memory)
# ===========================================================================

class TestParameterToTopicResolution:
    """Test _resolve_placeholders_from_params in clang_parser."""

    def test_resolve_placeholder_from_param_default(self):
        """${speed_limit_topic} resolves to 'speed_limit' from param default."""
        from colcon_lf.node_analysis.clang_parser import _resolve_placeholders_from_params

        pubs = [PublisherInfo(topic="${speed_limit_topic}", message_type="")]
        params = [ParameterInfo(name="speed_limit_topic", type="string",
                                default_value="speed_limit")]
        pub_map: dict[str, str] = {}
        _resolve_placeholders_from_params(
            pubs, [], [], [], [], [], params, pub_map, {}, {},
        )
        assert pubs[0].topic == "speed_limit"

    def test_no_resolve_when_no_matching_param(self):
        """Placeholder remains when no parameter matches."""
        from colcon_lf.node_analysis.clang_parser import _resolve_placeholders_from_params

        subs = [SubscriberInfo(topic="${unknown_var}", message_type="",
                               callback="")]
        params = [ParameterInfo(name="other_param", type="string",
                                default_value="value")]
        _resolve_placeholders_from_params(
            [], subs, [], [], [], [], params, {}, {}, {},
        )
        assert subs[0].topic == "${unknown_var}"

    def test_resolve_subscriber_topic(self):
        """Subscriber topic placeholder resolved."""
        from colcon_lf.node_analysis.clang_parser import _resolve_placeholders_from_params

        subs = [SubscriberInfo(topic="${cmd_vel_topic}", message_type="",
                               callback="cb")]
        params = [ParameterInfo(name="cmd_vel_topic", type="string",
                                default_value="/cmd_vel")]
        _resolve_placeholders_from_params(
            [], subs, [], [], [], [], params, {}, {}, {},
        )
        assert subs[0].topic == "/cmd_vel"

    def test_resolve_service_name(self):
        """Service server name placeholder resolved."""
        from colcon_lf.node_analysis.clang_parser import _resolve_placeholders_from_params

        srvs = [ServiceServerInfo(service_name="${svc_name}", service_type="",
                                  callback="")]
        params = [ParameterInfo(name="svc_name", type="string",
                                default_value="my_service")]
        _resolve_placeholders_from_params(
            [], [], srvs, [], [], [], params, {}, {}, {},
        )
        assert srvs[0].service_name == "my_service"

    def test_non_placeholder_topic_unchanged(self):
        """Regular topic names (no ${}) are not modified."""
        from colcon_lf.node_analysis.clang_parser import _resolve_placeholders_from_params

        pubs = [PublisherInfo(topic="sensor_data", message_type="")]
        params = [ParameterInfo(name="sensor_data", type="string",
                                default_value="other")]
        pub_map: dict[str, str] = {}
        _resolve_placeholders_from_params(
            pubs, [], [], [], [], [], params, pub_map, {}, {},
        )
        assert pubs[0].topic == "sensor_data"

    def test_non_string_default_ignored(self):
        """Parameter with non-string default does not resolve placeholder."""
        from colcon_lf.node_analysis.clang_parser import _resolve_placeholders_from_params

        pubs = [PublisherInfo(topic="${rate}", message_type="")]
        params = [ParameterInfo(name="rate", type="int", default_value=10)]
        pub_map: dict[str, str] = {}
        _resolve_placeholders_from_params(
            pubs, [], [], [], [], [], params, pub_map, {}, {},
        )
        assert pubs[0].topic == "${rate}"


class TestLaunchLevelPlaceholderResolution:
    """Test _resolve_topic_placeholders in hierarchical.py (Gap 2 phase 2)."""

    def test_yaml_param_resolves_placeholder(self):
        """After YAML overlay, placeholder in topic name is resolved."""
        from colcon_lf.graph.hierarchical import _resolve_topic_placeholders

        node = _node_info(
            "controller",
            publishers=[PublisherInfo(topic="${output_topic}", message_type="")],
            parameters=[ParameterInfo(name="output_topic", type="string",
                                      default_value="cmd_vel")],
        )
        launch = _launch_node("controller")
        _resolve_topic_placeholders([(launch, node)])
        assert node.publishers[0].topic == "cmd_vel"

    def test_no_params_no_change(self):
        """Node with no parameters leaves placeholders untouched."""
        from colcon_lf.graph.hierarchical import _resolve_topic_placeholders

        node = _node_info(
            "node",
            subscribers=[SubscriberInfo(topic="${unknown}", message_type="",
                                        callback="")],
        )
        launch = _launch_node("node")
        _resolve_topic_placeholders([(launch, node)])
        assert node.subscribers[0].topic == "${unknown}"

    def test_resolved_placeholder_clears_topic_dynamic(self):
        """A placeholder resolved from a launch/YAML parameter is no longer dynamic.

        The clang parser flags a ``${param}`` topic as ``topic_dynamic``. Once the
        launch-level overlay fills it in, the flag must drop, otherwise the resolved
        model still records the endpoint's topic as unresolved.
        """
        from colcon_lf.graph.hierarchical import _resolve_topic_placeholders

        node = _node_info(
            "controller",
            publishers=[PublisherInfo(topic="${output_topic}", message_type="",
                                      topic_dynamic=True)],
            subscribers=[
                SubscriberInfo(topic="${input_topic}", message_type="", callback="",
                               topic_dynamic=True),
                SubscriberInfo(topic="${unknown}", message_type="", callback="",
                               topic_dynamic=True),
            ],
            parameters=[
                ParameterInfo(name="output_topic", type="string", default_value="cmd_vel"),
                ParameterInfo(name="input_topic", type="string", default_value="odom"),
            ],
        )
        _resolve_topic_placeholders([(_launch_node("controller"), node)])
        assert (node.publishers[0].topic, node.publishers[0].topic_dynamic) == ("cmd_vel", False)
        assert (node.subscribers[0].topic, node.subscribers[0].topic_dynamic) == ("odom", False)
        assert (node.subscribers[1].topic, node.subscribers[1].topic_dynamic) == ("${unknown}", True)

    def test_partial_or_empty_resolution_keeps_topic_dynamic(self):
        """A value that is empty or still holds a placeholder is not a known topic."""
        from colcon_lf.graph.hierarchical import _resolve_topic_placeholders

        node = _node_info(
            "controller",
            publishers=[
                PublisherInfo(topic="${nested}", message_type="", topic_dynamic=True),
                PublisherInfo(topic="${empty}", message_type="", topic_dynamic=True),
            ],
            parameters=[
                ParameterInfo(name="nested", type="string", default_value="cmd/${suffix}"),
                ParameterInfo(name="empty", type="string", default_value=""),
            ],
        )
        _resolve_topic_placeholders([(_launch_node("controller"), node)])
        assert (node.publishers[0].topic, node.publishers[0].topic_dynamic) == (
            "cmd/${suffix}", True), "partially resolved topic was marked known"
        assert (node.publishers[1].topic, node.publishers[1].topic_dynamic) == (
            "", True), "empty topic was marked known"


# ===========================================================================
# Gap 5: Wrapper message types — unit tests
# ===========================================================================

class TestWrapperMessageTypes:
    """Verify the _WRAPPER_MESSAGE_TYPES mapping exists and has expected entries."""

    def test_twist_publisher_type(self):
        from colcon_lf.node_analysis.clang_parser import _WRAPPER_MESSAGE_TYPES
        assert _WRAPPER_MESSAGE_TYPES["TwistPublisher"] == "geometry_msgs::msg::Twist"

    def test_twist_subscriber_type(self):
        from colcon_lf.node_analysis.clang_parser import _WRAPPER_MESSAGE_TYPES
        assert _WRAPPER_MESSAGE_TYPES["TwistSubscriber"] == "geometry_msgs::msg::Twist"

    def test_odom_subscriber_type(self):
        from colcon_lf.node_analysis.clang_parser import _WRAPPER_MESSAGE_TYPES
        assert _WRAPPER_MESSAGE_TYPES["OdomSubscriber"] == "nav_msgs::msg::Odometry"


# ===========================================================================
# Gap 1 + 4: Integration tests (require libclang + built workspace)
# ===========================================================================

@pytest.fixture(scope="session")
def node_parser(package_dir, packages_root):
    """Parse a C++ node file by package and filename (cached)."""
    from colcon_lf.node_analysis.clang_parser import parse_node

    _cache: dict[tuple[str, str], object] = {}

    def _parse(package_name: str, cpp_filename: str):
        key = (package_name, cpp_filename)
        if key not in _cache:
            path = package_dir(package_name) / "src" / cpp_filename
            assert path.exists(), f"Source file not found: {path}"
            _cache[key] = parse_node(path, workspace_root=packages_root)
        return _cache[key]

    return _parse


# ===========================================================================
# callback_group field on ServiceClientInfo and ActionClientInfo (model tests)
# ===========================================================================

class TestServiceClientCallbackGroupField:
    """ServiceClientInfo carries a callback_group field (model-level unit test)."""

    def test_service_client_default_callback_group_is_none(self):
        sc = ServiceClientInfo(service_name="foo", service_type="Foo")
        assert sc.callback_group is None

    def test_service_client_callback_group_can_be_set(self):
        sc = ServiceClientInfo(
            service_name="foo",
            service_type="Foo",
            callback_group="exclusive_group_",
        )
        assert sc.callback_group == "exclusive_group_"


class TestActionClientCallbackGroupField:
    """ActionClientInfo carries a callback_group field (model-level unit test)."""

    def test_action_client_default_callback_group_is_none(self):
        ac = ActionClientInfo(action_name="bar", action_type="Bar")
        assert ac.callback_group is None

    def test_action_client_callback_group_can_be_set(self):
        ac = ActionClientInfo(
            action_name="bar",
            action_type="Bar",
            callback_group="reentrant_group_",
        )
        assert ac.callback_group == "reentrant_group_"
