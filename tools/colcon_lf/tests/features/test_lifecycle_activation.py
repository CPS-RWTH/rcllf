# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H005 — lifecycle hook override that skips the base call.

These tests cover the *diagnostic* half, driven off a hand-built graph. The
clang detection half (``_hook_chains_to_base``) needs real rclcpp headers and
lives in ``tests/test_clang_parser.py``.

To falsify, drop the `_owns_lifecycle_publisher` guard or the `is_lifecycle`
guard in ``features/modules/lifecycle_activation.py`` and the "stays quiet"
tests fail. To falsify the positive direction, stop populating
``NodeInfo.lifecycle_hooks_not_chained`` in the parser and
``test_unchained_on_activate_fires`` fails.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    LIFECYCLE_CHAINED_HOOKS,
    NodeInfo,
    PublisherInfo,
)


def _graph(node: NodeInfo, key: str = "n") -> SystemGraph:
    root = LaunchFileInfo(file_path=Path("/fake/bringup.launch.py"), package="test_pkg")
    return SystemGraph(
        name="s",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={key: node},
        communication=SystemCommunication(),
    )


def _node(
    *,
    base_class: str = "rclcpp_lifecycle::LifecycleNode",
    unchained: list[str] | None = None,
    with_publisher: bool = True,
    lifecycle_publisher: bool = True,
) -> NodeInfo:
    pubs = []
    if with_publisher:
        pubs.append(
            PublisherInfo(
                topic="out",
                message_type="std_msgs::msg::String",
                is_lifecycle=lifecycle_publisher,
            )
        )
    return NodeInfo(
        name="n",
        class_name="N",
        package="test_pkg",
        source_file=Path("/fake/n.cpp"),
        base_class=base_class,
        lifecycle_callbacks=["on_configure", "on_activate"],
        lifecycle_hooks_not_chained=unchained or [],
        publishers=pubs,
    )


def _detect(node: NodeInfo):
    from colcon_lf.features.diagnostics import DetectionContext
    from colcon_lf.features.modules.lifecycle_activation import detect

    return detect(DetectionContext(graph=_graph(node)))


class TestFires:
    def test_unchained_on_activate_fires(self):
        diags = _detect(_node(unchained=["on_activate"]))
        assert len(diags) == 1
        assert diags[0].code == "RCLLF-H005"

    def test_message_names_the_hook_and_the_consequence(self):
        """The message has to carry the near-silence warning, or it reads as style advice."""
        msg = _detect(_node(unchained=["on_activate"]))[0].message
        assert "on_activate" in msg
        assert "silently dropped" in msg
        assert "LifecyclePublisher warns once" in msg
        # It must name the fix, not just the problem.
        assert "rclcpp_lifecycle::LifecycleNode::on_activate(state)" in msg

    def test_on_deactivate_gets_its_own_consequence_text(self):
        msg = _detect(_node(unchained=["on_deactivate"]))[0].message
        assert "on_deactivate" in msg
        assert "continues after the node leaves the active state" in msg

    def test_both_hooks_give_two_diagnostics(self):
        diags = _detect(_node(unchained=["on_activate", "on_deactivate"]))
        assert len(diags) == 2

    def test_topic_is_named_so_the_user_can_find_it(self):
        assert "'out'" in _detect(_node(unchained=["on_activate"]))[0].message


class TestStaysQuiet:
    def test_chained_hook_is_silent(self):
        assert _detect(_node(unchained=[])) == []

    def test_node_without_lifecycle_publisher_is_silent(self):
        """Nothing to activate means skipping the base call costs nothing."""
        assert _detect(_node(unchained=["on_activate"], with_publisher=False)) == []

    def test_plain_publisher_on_a_lifecycle_node_is_silent(self):
        assert _detect(
            _node(unchained=["on_activate"], lifecycle_publisher=False)
        ) == []

    def test_plain_node_is_silent(self):
        """A non-lifecycle node has no base hook to chain to."""
        assert _detect(
            _node(base_class="rclcpp::Node", unchained=["on_activate"])
        ) == []

    def test_nav2_style_base_still_fires(self):
        """The broad lifecycle predicate has to reach wrapper bases too."""
        diags = _detect(
            _node(base_class="nav2_util::LifecycleNode", unchained=["on_activate"])
        )
        assert len(diags) == 1


class TestRegistration:
    def test_module_is_registered_with_warning_severity(self):
        from colcon_lf.features.diagnostics import Severity
        from colcon_lf.features.registry import REGISTRY, severity_for

        assert "lifecycle.base_call_chaining" in REGISTRY
        assert severity_for("RCLLF-H005") is Severity.WARNING

    def test_warning_not_error_so_generation_is_never_blocked(self):
        """Deliberate: the detection fails open, and a false positive must not refuse a build."""
        from colcon_lf.features.diagnostics import Severity
        from colcon_lf.features.registry import severity_for

        assert severity_for("RCLLF-H005") is not Severity.ERROR

    def test_only_the_two_hooks_with_real_base_work_are_checked(self):
        """on_configure/on_cleanup are no-ops on the base, so they must not be reported."""
        assert LIFECYCLE_CHAINED_HOOKS == ("on_activate", "on_deactivate")

    def test_playground_case_points_at_the_broken_fixture(self):
        from colcon_lf.features.registry import REGISTRY

        case = REGISTRY["lifecycle.base_call_chaining"].playground
        assert case.example == "lifecycle_component_example/unchained_activate"
        assert "RCLLF-H005" in case.expected_codes


class TestUnchainedHookOnConfigureIsIgnored:
    def test_on_configure_never_reported_even_if_listed(self):
        """Defence in depth: the parser filters these, and the module must too.

        If a future parser change started recording on_configure, a node that
        legitimately does not chain it would start warning for no reason.
        """
        diags = _detect(_node(unchained=["on_configure"]))
        assert [d.message for d in diags] == [] or all(
            "on_configure" not in d.message for d in diags
        )


@pytest.mark.parametrize("hook", LIFECYCLE_CHAINED_HOOKS)
def test_each_chained_hook_produces_a_diagnostic(hook):
    diags = _detect(_node(unchained=[hook]))
    assert len(diags) == 1
    assert hook in diags[0].message
