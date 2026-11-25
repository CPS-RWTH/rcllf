# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Managed lifecycle nodes registered as components take the non-component path.

Regression tests. A lifecycle node registered with
``rclcpp_components_register_nodes`` used to be handed to
``LFComponentManager``, which records it in ``class_to_id_`` but never in the
node registry, so ``ros2_lf_transition_node`` found nothing to drive. The node
was bound while still ``unconfigured``, owning none of the entities its
``on_configure``/``on_activate`` create, and the only symptom was a
``0 timer(s), 0 subscription(s)`` line at the end of binding.

To falsify any of these, change ``NodeInfo.takes_component_path`` back to
``return self.is_component``. The routing tests then fail because the lifecycle
component reappears in the component-instance list, which is exactly the bug.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.generation.binding_key import build_binding_keys
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    NodeInfo,
    TimerInfo,
    is_lifecycle_base,
)


def _launch_graph() -> LaunchGraph:
    root = LaunchFileInfo(file_path=Path("/fake/bringup.launch.py"), package="test_pkg")
    return LaunchGraph(root=root, all_nodes=[])


def _node(
    name: str,
    *,
    base_class: str = "rclcpp::Node",
    is_component: bool = False,
    lifecycle_callbacks: list[str] | None = None,
) -> NodeInfo:
    """A minimal node carrying one timer, so binding has something to find."""
    return NodeInfo(
        name=name,
        class_name="".join(part.capitalize() for part in name.split("_")),
        package="test_pkg",
        source_file=Path(f"/fake/{name}.cpp"),
        header_file=Path(f"/fake/{name}.hpp"),
        base_class=base_class,
        is_component=is_component,
        lifecycle_callbacks=lifecycle_callbacks or [],
        callbacks=[CallbackInfo(name="timer_callback")],
        timers=[TimerInfo(period_ms=1000.0, callback="timer_callback")],
    )


# ---------------------------------------------------------------------------
# The shared predicate
# ---------------------------------------------------------------------------

class TestIsLifecycleBase:
    @pytest.mark.parametrize(
        "base_class",
        [
            "rclcpp_lifecycle::LifecycleNode",
            "LifecycleNode",
            # The wrapper base every Nav2 server actually derives from. This is
            # the case that makes lifecycle component routing matter in practice, so it is pinned
            # explicitly rather than left to the substring rule by accident.
            "nav2_util::LifecycleNode",
        ],
    )
    def test_lifecycle_spellings(self, base_class):
        assert is_lifecycle_base(base_class) is True

    @pytest.mark.parametrize("base_class", ["rclcpp::Node", "Node", "some_pkg::Base"])
    def test_plain_spellings(self, base_class):
        assert is_lifecycle_base(base_class) is False

    def test_unresolved_base_falls_back_to_transition_hooks(self):
        """A missing header leaves base_class empty; the hooks still give it away."""
        assert is_lifecycle_base("", ["on_configure", "on_activate"]) is True

    def test_unresolved_base_without_hooks_is_not_lifecycle(self):
        assert is_lifecycle_base("", []) is False

    def test_plain_base_with_hooks_is_not_promoted(self):
        """The hooks arm is a fallback for an unknown base, not an override."""
        assert is_lifecycle_base("rclcpp::Node", ["on_configure"]) is False


# ---------------------------------------------------------------------------
# NodeInfo properties
# ---------------------------------------------------------------------------

class TestTakesComponentPath:
    def test_plain_component_still_takes_the_component_path(self):
        ni = _node("relay", is_component=True)
        assert ni.is_lifecycle is False
        assert ni.takes_component_path is True

    def test_lifecycle_component_is_rerouted(self):
        ni = _node(
            "supervised", base_class="rclcpp_lifecycle::LifecycleNode", is_component=True,
        )
        assert ni.is_lifecycle is True
        assert ni.takes_component_path is False

    def test_lifecycle_non_component_is_unaffected(self):
        ni = _node("standalone", base_class="rclcpp_lifecycle::LifecycleNode")
        assert ni.takes_component_path is False

    def test_plain_non_component_is_unaffected(self):
        assert _node("plain").takes_component_path is False

    def test_nav2_style_component_is_rerouted(self):
        ni = _node("planner", base_class="nav2_util::LifecycleNode", is_component=True)
        assert ni.takes_component_path is False

    def test_properties_are_not_serialized_fields(self):
        """Derived, so system.json needs no new field and cannot go stale."""
        import dataclasses

        d = dataclasses.asdict(_node("plain"))
        assert "is_lifecycle" not in d
        assert "takes_component_path" not in d


# ---------------------------------------------------------------------------
# Routing through generation
# ---------------------------------------------------------------------------

class TestComponentInstanceRouting:
    def _graph(self) -> SystemGraph:
        return SystemGraph(
            name="mixed",
            launch_graph=_launch_graph(),
            nodes={
                "relay": _node("relay", is_component=True),
                "supervised": _node(
                    "supervised",
                    base_class="rclcpp_lifecycle::LifecycleNode",
                    is_component=True,
                ),
            },
            communication=SystemCommunication(),
        )

    def test_lifecycle_component_is_absent_from_component_instances(self):
        """The list handed to LFComponentManager must not contain it."""
        from colcon_lf.generation.node_identity import build_node_identities
        from colcon_lf.verb.generate import build_component_instances

        graph = self._graph()
        entries = build_component_instances(
            graph, build_binding_keys(graph.nodes), build_node_identities(graph),
        )
        plugin_classes = {e["plugin_class"] for e in entries}
        assert "Supervised" not in plugin_classes
        # The plain component is untouched, so this is a targeted change and
        # not a blanket disabling of the component path.
        assert plugin_classes == {"Relay"}

    def test_reroute_is_reported(self, capsys):
        from colcon_lf.verb.generate import _report_lifecycle_component_reroute

        _report_lifecycle_component_reroute(self._graph())
        out = capsys.readouterr().out
        assert "Supervised" in out
        assert "driven before binding" in out and "#73" not in out
        # The plain component is not re-routed, so it is not named.
        assert "Relay" not in out

    def test_nothing_reported_when_no_lifecycle_components(self, capsys):
        graph = SystemGraph(
            name="plain",
            launch_graph=_launch_graph(),
            nodes={"relay": _node("relay", is_component=True)},
            communication=SystemCommunication(),
        )
        from colcon_lf.verb.generate import _report_lifecycle_component_reroute

        _report_lifecycle_component_reroute(graph)
        assert capsys.readouterr().out == ""


# ---------------------------------------------------------------------------
# The auto_transition opt-out trap
# ---------------------------------------------------------------------------

class TestLifecycleOptOutWarning:
    """``auto_transition: false`` produces a node nothing in the program activates.

    External tools can transition the node and the binding follows,
    but a node nobody activates stays idle for the whole run.
    """

    def _graph(self) -> SystemGraph:
        return SystemGraph(
            name="opt_out",
            launch_graph=_launch_graph(),
            nodes={
                "supervised": _node(
                    "supervised", base_class="rclcpp_lifecycle::LifecycleNode",
                ),
                "plain": _node("plain"),
            },
            communication=SystemCommunication(),
        )

    def test_opted_out_lifecycle_node_is_named(self, capsys):
        from colcon_lf.verb.generate import _warn_on_lifecycle_opt_out

        _warn_on_lifecycle_opt_out(self._graph(), {"supervised": False, "plain": True})
        err = capsys.readouterr().err
        assert "Supervised" in err
        assert "auto_transition: false" in err
        # The way out has to travel with the warning, otherwise the user has no
        # way to know who is expected to activate the node.
        assert "stay idle until" in err
        assert "ros2 lifecycle set" in err

    def test_plain_node_opting_out_is_not_warned_about(self, capsys):
        """The key is inert on a node with no managed lifecycle."""
        from colcon_lf.verb.generate import _warn_on_lifecycle_opt_out

        _warn_on_lifecycle_opt_out(self._graph(), {"supervised": True, "plain": False})
        assert capsys.readouterr().err == ""

    def test_default_bring_up_is_silent(self, capsys):
        from colcon_lf.verb.generate import _warn_on_lifecycle_opt_out

        _warn_on_lifecycle_opt_out(self._graph(), {"supervised": True, "plain": True})
        assert capsys.readouterr().err == ""

    def test_missing_key_is_treated_as_opted_in(self, capsys):
        """An absent entry means the registry default (True), not a warning."""
        from colcon_lf.verb.generate import _warn_on_lifecycle_opt_out

        _warn_on_lifecycle_opt_out(self._graph(), {})
        assert capsys.readouterr().err == ""
