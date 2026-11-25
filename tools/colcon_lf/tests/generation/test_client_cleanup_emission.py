# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""node_factories.cpp registers typed client operations.

On deactivate the binding prunes a lifecycle node's pending service requests,
stops the callbacks of its action goals and cancels them; while the node is
inactive it reads goal ids and goal responses. rclcpp needs the client type
for each, so the generated factories register one functor per qualified
service type and one set of operations per qualified action type. An
unqualified action type is refused.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from colcon_lf.generation.factory_emitter import (
    UnguardedActionClientError,
    client_cleanup_lines,
    emit_node_factories_cpp,
)
from colcon_lf.verb.generate import _factory_entries


def _entry(**extra):
    return {
        "binding_key": "Sleeper", "class_name": "Sleeper", "package": "pkg",
        "header": "pkg/sleeper.hpp", "idx": 0, "node_name": "sleeper",
        "namespace": "", **extra,
    }


def _node(is_lifecycle, services=(), actions=()):
    return SimpleNamespace(
        package="pkg", class_name="Sleeper", header_file="sleeper.hpp",
        qualified_name="", name="sleeper", base_class="", is_lifecycle=is_lifecycle,
        service_clients=[SimpleNamespace(service_type=t) for t in services],
        action_clients=[SimpleNamespace(action_type=t) for t in actions],
    )


def test_no_client_types_leaves_the_file_unchanged():
    assert client_cleanup_lines([_entry()]) == ([], [])
    assert emit_node_factories_cpp([_entry()]) == emit_node_factories_cpp(
        [_entry(service_client_types=[], action_client_types=[])])


def test_service_and_action_types_register_typed_functors():
    cpp = emit_node_factories_cpp([_entry(
        service_client_types=["example_interfaces::srv::AddTwoInts"],
        action_client_types=["example_interfaces::action::Fibonacci"],
    )])
    assert '#include "example_interfaces/srv/add_two_ints.hpp"' in cpp
    assert '#include "example_interfaces/action/fibonacci.hpp"' in cpp
    assert '#include "rclcpp_action/rclcpp_action.hpp"' in cpp
    assert ("dynamic_cast<rclcpp::Client<example_interfaces::srv::AddTwoInts> *>"
            in cpp)
    assert "typed->prune_pending_requests()" in cpp
    assert ("dynamic_cast<rclcpp_action::Client<example_interfaces::action::Fibonacci> *>"
            in cpp)
    impl = "example_interfaces::action::Fibonacci::Impl"
    assert f"static_cast<const {impl}::GetResultService::Request *>(request)" in cpp
    assert f"static_cast<const {impl}::SendGoalService::Request *>(request)" in cpp
    assert f"static_cast<const {impl}::SendGoalService::Response *>(response)" in cpp
    assert "for (const auto & id : ids) {typed->stop_callbacks(id);}" in cpp
    assert "typed->async_cancel_all_goals();" in cpp
    assert "ros2_lf_register_action_client_ops(std::move(ops));" in cpp
    # Registered before any node is constructed.
    body = cpp.split("void ros2_lf_register_all_nodes() {", 1)[1]
    for registration in ("ros2_lf_register_client_cleanup",
                         "ros2_lf_register_action_client_ops"):
        assert body.index(registration) < body.index("ros2_lf_construct_node")


def test_each_type_registers_once_across_instances():
    t = "pkg_msgs::srv::GetMap"
    _, body = client_cleanup_lines([
        _entry(service_client_types=[t]), _entry(service_client_types=[t, t])])
    assert sum("ros2_lf_register_client_cleanup(" in line for line in body) == 1


def test_header_follows_rosidl_snake_case():
    includes, _ = client_cleanup_lines([_entry(
        service_client_types=["nav2_msgs::srv::ClearEntireCostmap"],
        action_client_types=["nav2_msgs::action::NavigateToPose"])])
    assert '#include "nav2_msgs/srv/clear_entire_costmap.hpp"' in includes
    assert '#include "nav2_msgs/action/navigate_to_pose.hpp"' in includes


def test_unqualified_service_type_is_skipped_with_a_comment():
    includes, body = client_cleanup_lines([_entry(
        service_client_types=["AddTwoInts", "pkg::action::NotAService"])])
    assert includes == []
    assert not any("register" in line for line in body)
    assert any("'AddTwoInts'" in line for line in body)
    assert any("'pkg::action::NotAService'" in line for line in body)


@pytest.mark.parametrize("type_name", ["Fibonacci", "pkg::srv::NotAnAction", ""])
def test_unqualified_action_type_is_refused(type_name):
    with pytest.raises(UnguardedActionClientError, match="could not be cancelled") as err:
        client_cleanup_lines([_entry(action_client_types=[type_name])])
    assert f"'{type_name}'" in str(err.value)
    assert "'Sleeper'" in str(err.value)


def test_factory_entries_carry_client_types_of_lifecycle_nodes_only():
    srv, act = "example_interfaces::srv::AddTwoInts", "example_interfaces::action::Fibonacci"
    keys = {"lc": "Lc", "plain": "Plain"}
    entries = _factory_entries(
        [("lc", _node(True, [srv], [act])), ("plain", _node(False, [srv], [act]))],
        keys, {})
    assert entries[0]["service_client_types"] == [srv]
    assert entries[0]["action_client_types"] == [act]
    assert entries[1]["service_client_types"] == []
    assert entries[1]["action_client_types"] == []
