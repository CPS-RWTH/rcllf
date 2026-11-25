# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for lf_generator module."""

from __future__ import annotations

import pytest

from colcon_lf.comm_analysis.models import (
    ActionConnection,
    ServiceConnection,
    SystemCommunication,
    TopicConnection,
)
from colcon_lf.generation.lf_generator import _stub_add, generate_lf_system
from ..corr_declared import generate_declared
from colcon_lf.graph.models import SystemGraph
from colcon_lf.comm_analysis.topic_mapper import build_communication, resolve_name
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph, LaunchNodeAction
from colcon_lf.node_analysis.models import (
    ActionClientInfo,
    ActionServerInfo,
    CallbackGroupInfo,
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    ServiceClientInfo,
    ServiceServerInfo,
    SubscriberInfo,
    TimerInfo,
)


def _make_launch_graph() -> LaunchGraph:
    """Minimal launch graph for testing."""
    root = LaunchFileInfo(
        file_path="/fake/bringup.launch.py",
        package="test_pkg",
    )
    return LaunchGraph(root=root, all_nodes=[])


def _make_publisher_node() -> NodeInfo:
    """A simple publisher node with a timer callback."""
    return NodeInfo(
        name="publisher_node",
        class_name="PublisherNode",
        package="pubsub_example",
        source_file="/fake/publisher_node.cpp",
        callbacks=[
            CallbackInfo(
                name="timer_callback",
                publishers_used=["sensor_data"],
            ),
        ],
        timers=[
            TimerInfo(period_ms=500.0, callback="timer_callback"),
        ],
        publishers=[
            PublisherInfo(topic="sensor_data", message_type="custom_interfaces::msg::SensorData"),
        ],
    )


def _make_subscriber_node() -> NodeInfo:
    """A simple subscriber node."""
    return NodeInfo(
        name="subscriber_node",
        class_name="SubscriberNode",
        package="pubsub_example",
        source_file="/fake/subscriber_node.cpp",
        callbacks=[
            CallbackInfo(name="topic_callback"),
        ],
        subscribers=[
            SubscriberInfo(
                topic="sensor_data",
                message_type="custom_interfaces::msg::SensorData",
                callback="topic_callback",
            ),
        ],
    )


class TestGenerateSimplePubSub:
    """Test generation of a simple publisher-subscriber system."""

    @pytest.fixture()
    def system(self):
        graph = SystemGraph(
            name="bringup",
            launch_graph=_make_launch_graph(),
            nodes={
                "publisher_node": _make_publisher_node(),
                "subscriber_node": _make_subscriber_node(),
            },
            communication=SystemCommunication(
                topics=[
                    TopicConnection(
                        topic="/sensor_data",
                        message_type="custom_interfaces::msg::SensorData",
                        publishers=["publisher_node"],
                        subscribers=["subscriber_node"],
                    ),
                ],
            ),
        )
        return generate_declared(graph)

    def test_system_name(self, system):
        assert system.name == "Bringup"

    def test_target(self, system):
        assert system.target == "CCpp"

    def test_reactor_count(self, system):
        # Each node emits a shell + DefaultGroup sub-reactor → 2 nodes × 2 = 4
        assert len(system.reactors) == 4

    def test_instance_count(self, system):
        assert len(system.main_instances) == 2

    def test_connection_count(self, system):
        assert len(system.main_connections) == 1

    def test_publisher_reactor(self, system):
        # Outer shell holds only external ports and a forwarding connection
        pub_shell = next(r for r in system.reactors if r.name == "PublisherNode")
        # The admitted edge's corr port replaces the stub port.
        assert len(pub_shell.ports) == 1
        assert pub_shell.ports[0].name == "corr_e0"
        assert pub_shell.ports[0].direction == "output"
        assert pub_shell.ports[0].type == "rcllf_corr_t"
        assert len(pub_shell.timers) == 0
        assert len(pub_shell.reactions) == 0
        # Timer and reaction live in the DefaultGroup sub-reactor
        pub_sub = next(r for r in system.reactors if r.name == "PublisherNode_DefaultGroup")
        assert len(pub_sub.timers) == 1
        assert pub_sub.timers[0].period == "500 msec"
        assert len(pub_sub.reactions) == 1
        assert pub_sub.reactions[0].triggers == ["t_timer_callback"]
        assert pub_sub.reactions[0].effects == ["corr_e0"]

    def test_subscriber_reactor(self, system):
        # Outer shell holds only the external input port
        sub_shell = next(r for r in system.reactors if r.name == "SubscriberNode")
        assert len(sub_shell.ports) == 1
        assert sub_shell.ports[0].name == "corr_e0"
        assert sub_shell.ports[0].direction == "input"
        assert len(sub_shell.timers) == 0
        assert len(sub_shell.reactions) == 0
        # Reaction lives in the DefaultGroup sub-reactor
        sub_sub = next(r for r in system.reactors if r.name == "SubscriberNode_DefaultGroup")
        # Drain and close replace the legacy invoke_sub reaction.
        # The in-process d = 0 edge has no separate arrive, the
        # drain takes the port so the first record runs at (t, m).
        # A startup reaction arms the subscription's deadline window.
        assert [r.kind for r in sub_sub.reactions] == ["sub", "corr_close", "corr_silence_arm"]
        assert sub_sub.reactions[0].triggers == ["corr_drain_e0", "corr_e0"]
        assert sub_sub.reactions[0].effects == ["corr_drain_e0"]

    def test_connection_has_delay(self, system):
        conn = system.main_connections[0]
        assert conn.source == "publisher_node.corr_e0"
        assert conn.destination == "subscriber_node.corr_e0"
        # The admitted edge's delay moved from the LF connection into the
        # correspondence edge (drain wake); the corr_e link itself has no after.
        assert conn.after_delay_ns is None
        assert system.corr_edges[0].delay_ns == 0  # default (no artificial delay)

    def test_instance_names(self, system):
        names = {i.name for i in system.main_instances}
        assert names == {"publisher_node", "subscriber_node"}


class TestGenerateTimerOnly:
    """Test a node with only a timer (no pub/sub)."""

    def test_timer_only_reactor(self):
        graph = SystemGraph(
            name="timer_test",
            launch_graph=_make_launch_graph(),
            nodes={
                "timer_node": NodeInfo(
                    name="timer_node",
                    class_name="TimerNode",
                    package="timer_example",
                    source_file="/fake/timer_node.cpp",
                    callbacks=[CallbackInfo(name="on_timer")],
                    timers=[TimerInfo(period_ms=1000.0, callback="on_timer")],
                ),
            },
        )
        system = generate_declared(graph)
        # Shell + DefaultGroup sub-reactor
        assert len(system.reactors) == 2
        shell = next(r for r in system.reactors if r.name == "TimerNode")
        assert len(shell.ports) == 0
        assert len(shell.timers) == 0
        assert len(shell.reactions) == 0
        sub = next(r for r in system.reactors if r.name == "TimerNode_DefaultGroup")
        assert len(sub.timers) == 1
        assert len(sub.reactions) == 1
        assert len(system.main_connections) == 0


class TestGenerateMultiConnection:
    """Test N publishers × M subscribers on the same topic."""

    def test_two_pubs_one_sub(self):
        graph = SystemGraph(
            name="multi",
            launch_graph=_make_launch_graph(),
            nodes={
                "pub_a": NodeInfo(
                    name="pub_a", class_name="PubA", package="test",
                    source_file="/fake/a.cpp",
                    callbacks=[CallbackInfo(name="cb_a", publishers_used=["data"])],
                    timers=[TimerInfo(period_ms=100.0, callback="cb_a")],
                    publishers=[PublisherInfo(topic="data", message_type="std_msgs::msg::String")],
                ),
                "pub_b": NodeInfo(
                    name="pub_b", class_name="PubB", package="test",
                    source_file="/fake/b.cpp",
                    callbacks=[CallbackInfo(name="cb_b", publishers_used=["data"])],
                    timers=[TimerInfo(period_ms=200.0, callback="cb_b")],
                    publishers=[PublisherInfo(topic="data", message_type="std_msgs::msg::String")],
                ),
                "sub_c": NodeInfo(
                    name="sub_c", class_name="SubC", package="test",
                    source_file="/fake/c.cpp",
                    callbacks=[CallbackInfo(name="cb_c")],
                    subscribers=[SubscriberInfo(
                        topic="data", message_type="std_msgs::msg::String", callback="cb_c",
                    )],
                ),
            },
            communication=SystemCommunication(
                topics=[
                    TopicConnection(
                        topic="/data",
                        message_type="std_msgs::msg::String",
                        publishers=["pub_a", "pub_b"],
                        subscribers=["sub_c"],
                    ),
                ],
            ),
        )
        system = generate_declared(graph)
        # 2 publishers × 1 subscriber = 2 connections
        assert len(system.main_connections) == 2


class TestGenerateDuplicateClass:
    """Two instances of one class get two *per-instance* reactor definitions.

    The
    dedup guard keys on the binding key (``generation/binding_key.py``), not the
    class name, so N >= 2 instances no longer share a definition, a RID range,
    or a registry slot.  Single-instance classes keep plain names (and hence
    byte-identical output) — see ``test_per_instance_rids.py``.
    """

    def test_per_instance_reactor_definitions(self):
        node_a = NodeInfo(
            name="node_a", class_name="SensorNode", package="test",
            source_file="/fake/sensor.cpp",
            callbacks=[CallbackInfo(name="cb", publishers_used=["out"])],
            timers=[TimerInfo(period_ms=100.0, callback="cb")],
            publishers=[PublisherInfo(topic="out", message_type="std_msgs::msg::Float64")],
        )
        node_b = NodeInfo(
            name="node_b", class_name="SensorNode", package="test",
            source_file="/fake/sensor.cpp",
            callbacks=[CallbackInfo(name="cb", publishers_used=["out"])],
            timers=[TimerInfo(period_ms=100.0, callback="cb")],
            publishers=[PublisherInfo(topic="out", message_type="std_msgs::msg::Float64")],
        )
        graph = SystemGraph(
            name="dup",
            launch_graph=_make_launch_graph(),
            nodes={"node_a": node_a, "node_b": node_b},
        )
        system = generate_declared(graph)
        # (shell + DefaultGroup sub) x 2 instances = 4 reactor definitions.
        assert len(system.reactors) == 4
        names = {r.name for r in system.reactors}
        assert names == {
            "SensorNode__node_a",
            "SensorNode__node_a_DefaultGroup",
            "SensorNode__node_b",
            "SensorNode__node_b_DefaultGroup",
        }
        assert len(system.main_instances) == 2
        assert {i.reactor_class for i in system.main_instances} == {
            "SensorNode__node_a", "SensorNode__node_b",
        }


class TestGenerateWithCallbackGroups:
    """Test reactor generation with callback groups → nested sub-reactors."""

    def _make_full_cbg_node(self) -> NodeInfo:
        return NodeInfo(
            name="cbg_node",
            class_name="CallbackGroupNode",
            package="callback_group_example",
            source_file="/fake/cbg.cpp",
            callbacks=[
                CallbackInfo(name="timer_a_callback"),
                CallbackInfo(name="timer_b_callback"),
                CallbackInfo(name="timer_c_callback"),
                CallbackInfo(name="chatter_callback"),
            ],
            callback_groups=[
                CallbackGroupInfo(name="exclusive_group_", type="MutuallyExclusive"),
                CallbackGroupInfo(name="reentrant_group_", type="Reentrant"),
            ],
            timers=[
                TimerInfo(period_ms=500.0, callback="timer_a_callback", callback_group="exclusive_group_"),
                TimerInfo(period_ms=1000.0, callback="timer_b_callback", callback_group="exclusive_group_"),
                TimerInfo(period_ms=750.0, callback="timer_c_callback", callback_group="reentrant_group_"),
            ],
            subscribers=[
                SubscriberInfo(
                    topic="chatter",
                    message_type="std_msgs::msg::String",
                    callback="chatter_callback",
                    callback_group="exclusive_group_",
                ),
            ],
        )

    def test_callback_group_node(self):
        node = self._make_full_cbg_node()
        graph = SystemGraph(
            name="cbg_test",
            launch_graph=_make_launch_graph(),
            nodes={"cbg_node": node},
        )
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            system = generate_declared(graph)
        outer = next(r for r in system.reactors if r.name == "CallbackGroupNode")
        # Outer shell: one input port (chatter), no timers, no user reactions
        assert len(outer.ports) == 1
        assert outer.ports[0].direction == "input"
        assert len(outer.timers) == 0
        assert len(outer.reactions) == 0
        # Two instances: one Mutex sub + one per-callback Reentrant sub
        assert len(outer.instances) == 2
        # Port forwarding for the chatter subscription
        assert len(outer.connections) > 0

    def test_callback_group_sub_reactors_in_system(self):
        """Sub-reactor definitions appear in LFSystem.reactors."""
        node = NodeInfo(
            name="cbg_node",
            class_name="CallbackGroupNode",
            package="callback_group_example",
            source_file="/fake/cbg.cpp",
            callbacks=[
                CallbackInfo(name="timer_a_callback"),
                CallbackInfo(name="timer_c_callback"),
            ],
            callback_groups=[
                CallbackGroupInfo(name="exclusive_group_", type="MutuallyExclusive"),
                CallbackGroupInfo(name="reentrant_group_", type="Reentrant"),
            ],
            timers=[
                TimerInfo(period_ms=500.0, callback="timer_a_callback", callback_group="exclusive_group_"),
                TimerInfo(period_ms=750.0, callback="timer_c_callback", callback_group="reentrant_group_"),
            ],
        )
        graph = SystemGraph(
            name="cbg_test",
            launch_graph=_make_launch_graph(),
            nodes={"cbg_node": node},
        )
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            system = generate_declared(graph)
        names = {r.name for r in system.reactors}
        # Outer shell + Mutex group sub + one per-callback Reentrant sub
        assert "CallbackGroupNode" in names
        assert "CallbackGroupNode_ExclusiveGroup" in names
        assert "CallbackGroupNode_TimerCCallback" in names
        assert "CallbackGroupNode_ReentrantGroup" not in names
        assert len(system.reactors) == 3


class TestDefaultOnlyCallbackGroup:
    """Nodes with the default callback group wrap callbacks in a DefaultGroup sub-reactor."""

    def test_default_group_produces_default_subreactor(self):
        node = NodeInfo(
            name="publisher_node",
            class_name="PublisherNode",
            package="pubsub_example",
            source_file="/fake/publisher_node.cpp",
            callbacks=[
                CallbackInfo(name="timer_callback", publishers_used=["sensor_data"]),
            ],
            callback_groups=[
                CallbackGroupInfo(name="default_group", type="MutuallyExclusive"),
            ],
            timers=[
                TimerInfo(period_ms=500.0, callback="timer_callback", callback_group="default_group"),
            ],
            publishers=[
                PublisherInfo(topic="sensor_data", message_type="std_msgs::msg::String"),
            ],
        )
        graph = SystemGraph(
            name="test",
            launch_graph=_make_launch_graph(),
            nodes={"publisher_node": node},
        )
        system = generate_declared(graph)
        # Shell + sub-reactor for the "default_group" callback group
        assert len(system.reactors) == 2
        names = {r.name for r in system.reactors}
        assert "PublisherNode" in names
        assert "PublisherNode_DefaultGroup" in names
        shell = next(r for r in system.reactors if r.name == "PublisherNode")
        # Shell has no user reactions or timers — those live in the sub
        assert len(shell.timers) == 0
        assert len(shell.reactions) == 0
        assert len(shell.instances) == 1
        sub = next(r for r in system.reactors if r.name == "PublisherNode_DefaultGroup")
        assert len(sub.timers) == 1
        assert len(sub.reactions) == 1


class TestReentrantGroupMapping:
    """Phase 4: validate correct Mutex/Reentrant group → reactor mapping."""

    import warnings as _warnings

    def _make_graph(self, node: NodeInfo) -> SystemGraph:
        return SystemGraph(
            name="test_bringup",
            launch_graph=_make_launch_graph(),
            nodes={"the_node": node},
        )

    def _gen(self, node: NodeInfo):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            return generate_declared(self._make_graph(node))

    def test_mutex_group_yields_single_subreactor(self):
        """One MutuallyExclusive group with 3 callbacks → one sub-reactor with 3 reactions."""
        node = NodeInfo(
            name="n", class_name="Foo", package="p", source_file="/f.cpp",
            callbacks=[
                CallbackInfo(name="cb_a"),
                CallbackInfo(name="cb_b"),
                CallbackInfo(name="cb_c"),
            ],
            callback_groups=[CallbackGroupInfo(name="grp_", type="MutuallyExclusive")],
            timers=[
                TimerInfo(period_ms=100.0, callback="cb_a", callback_group="grp_"),
                TimerInfo(period_ms=200.0, callback="cb_b", callback_group="grp_"),
                TimerInfo(period_ms=300.0, callback="cb_c", callback_group="grp_"),
            ],
        )
        system = self._gen(node)
        names = {r.name for r in system.reactors}
        assert "Foo" in names
        assert "Foo_Grp" in names
        assert len(system.reactors) == 2
        sub = next(r for r in system.reactors if r.name == "Foo_Grp")
        assert len(sub.reactions) == 3

    def test_reentrant_group_yields_per_callback_subreactors(self):
        """One Reentrant group with 3 callbacks → 3 per-callback sub-reactors."""
        node = NodeInfo(
            name="n", class_name="Bar", package="p", source_file="/f.cpp",
            callbacks=[
                CallbackInfo(name="cb_a"),
                CallbackInfo(name="cb_b"),
                CallbackInfo(name="cb_c"),
            ],
            callback_groups=[CallbackGroupInfo(name="rgrp_", type="Reentrant")],
            timers=[
                TimerInfo(period_ms=100.0, callback="cb_a", callback_group="rgrp_"),
                TimerInfo(period_ms=200.0, callback="cb_b", callback_group="rgrp_"),
                TimerInfo(period_ms=300.0, callback="cb_c", callback_group="rgrp_"),
            ],
        )
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            system = generate_declared(self._make_graph(node))
        names = {r.name for r in system.reactors}
        assert "Bar" in names
        assert "Bar_CbA" in names
        assert "Bar_CbB" in names
        assert "Bar_CbC" in names
        # Outer shell + 3 per-callback subs (no group-level aggregate)
        assert "Bar_Rgrp" not in names
        assert len(system.reactors) == 4
        for sub_name in ("Bar_CbA", "Bar_CbB", "Bar_CbC"):
            sub = next(r for r in system.reactors if r.name == sub_name)
            assert len(sub.reactions) == 1

    def test_reentrant_subscriber_port_forwarded(self):
        """Reentrant subscription callback → input port forwarded from outer to sub."""
        node = NodeInfo(
            name="n", class_name="Baz", package="p", source_file="/f.cpp",
            callbacks=[CallbackInfo(name="sub_cb")],
            callback_groups=[CallbackGroupInfo(name="rgrp_", type="Reentrant")],
            subscribers=[
                SubscriberInfo(topic="/data", message_type="std_msgs::msg::String",
                               callback="sub_cb", callback_group="rgrp_"),
            ],
        )
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            system = generate_declared(self._make_graph(node))
        outer = next(r for r in system.reactors if r.name == "Baz")
        # Outer has the /data input port
        assert any(p.direction == "input" and p.name == "data" for p in outer.ports)
        # One forwarding connection: outer.data → sub_cb.data
        assert any(
            conn.source == "data" and "sub_cb" in conn.destination
            for conn in outer.connections
        )
        # Sub-reactor has an input port for /data
        sub = next(r for r in system.reactors if r.name == "Baz_SubCb")
        assert any(p.direction == "input" for p in sub.ports)
        # Sub has one reaction triggered by the port
        assert len(sub.reactions) == 1

    def test_reentrant_publisher_port_forwarded(self):
        """Reentrant timer callback that publishes → output port forwarded back to outer."""
        node = NodeInfo(
            name="n", class_name="Qux", package="p", source_file="/f.cpp",
            callbacks=[CallbackInfo(name="pub_cb", publishers_used=["/out"])],
            callback_groups=[CallbackGroupInfo(name="rgrp_", type="Reentrant")],
            timers=[TimerInfo(period_ms=500.0, callback="pub_cb", callback_group="rgrp_")],
            publishers=[PublisherInfo(topic="/out", message_type="std_msgs::msg::String")],
        )
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            system = generate_declared(self._make_graph(node))
        outer = next(r for r in system.reactors if r.name == "Qux")
        # Outer has the /out output port
        assert any(p.direction == "output" for p in outer.ports)
        # Forwarding connection: sub.out → outer.out
        assert any(
            "pub_cb" in conn.source and conn.destination == "out"
            for conn in outer.connections
        )
        sub = next(r for r in system.reactors if r.name == "Qux_PubCb")
        assert any(p.direction == "output" for p in sub.ports)

    def test_cross_group_parallelism_structure(self):
        """Two MutuallyExclusive groups → two sibling sub-reactors on the outer."""
        node = NodeInfo(
            name="n", class_name="MultiGrp", package="p", source_file="/f.cpp",
            callbacks=[
                CallbackInfo(name="cb_a"),
                CallbackInfo(name="cb_b"),
            ],
            callback_groups=[
                CallbackGroupInfo(name="grp_a_", type="MutuallyExclusive"),
                CallbackGroupInfo(name="grp_b_", type="MutuallyExclusive"),
            ],
            timers=[
                TimerInfo(period_ms=100.0, callback="cb_a", callback_group="grp_a_"),
                TimerInfo(period_ms=200.0, callback="cb_b", callback_group="grp_b_"),
            ],
        )
        system = self._gen(node)
        outer = next(r for r in system.reactors if r.name == "MultiGrp")
        # Two sibling instances (one per Mutex group) → parallel under reactor-c
        assert len(outer.instances) == 2
        names = {r.name for r in system.reactors}
        assert "MultiGrp_GrpA" in names
        assert "MultiGrp_GrpB" in names

    def test_outer_reactor_has_no_user_reactions(self):
        """The outer node reactor carries only startup and port-forwarding, no reactions."""
        node = NodeInfo(
            name="n", class_name="NoReact", package="p", source_file="/f.cpp",
            callbacks=[
                CallbackInfo(name="timer_a"),
                CallbackInfo(name="timer_b"),
            ],
            callback_groups=[
                CallbackGroupInfo(name="mutex_", type="MutuallyExclusive"),
                CallbackGroupInfo(name="reent_", type="Reentrant"),
            ],
            timers=[
                TimerInfo(period_ms=100.0, callback="timer_a", callback_group="mutex_"),
                TimerInfo(period_ms=200.0, callback="timer_b", callback_group="reent_"),
            ],
        )
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            system = generate_declared(self._make_graph(node), target_lfc=True)
        outer = next(r for r in system.reactors if r.name == "NoReact")
        assert outer.reactions == []
        assert outer.startup_body is not None
        assert "ros2_lf_bind_node" in outer.startup_body

    def test_self_concurrency_warning_emitted(self):
        """A reentrant callback triggers a UserWarning about self-concurrency."""
        import warnings
        node = NodeInfo(
            name="n", class_name="WarnMe", package="p", source_file="/f.cpp",
            callbacks=[CallbackInfo(name="reent_cb")],
            callback_groups=[CallbackGroupInfo(name="rgrp_", type="Reentrant")],
            timers=[TimerInfo(period_ms=100.0, callback="reent_cb", callback_group="rgrp_")],
        )
        graph = self._make_graph(node)
        with pytest.warns(UserWarning, match="reentrant callback"):
            generate_declared(graph)

    def test_no_warning_for_mutex_only(self):
        """Mutex-only nodes emit no UserWarning about reentrant callbacks."""
        import warnings
        node = NodeInfo(
            name="n", class_name="Silent", package="p", source_file="/f.cpp",
            callbacks=[CallbackInfo(name="cb")],
            callback_groups=[CallbackGroupInfo(name="mutex_", type="MutuallyExclusive")],
            timers=[TimerInfo(period_ms=100.0, callback="cb", callback_group="mutex_")],
        )
        graph = self._make_graph(node)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            generate_declared(graph)
        reentrant_warnings = [x for x in w if "reentrant" in str(x.message).lower()]
        assert len(reentrant_warnings) == 0

    def test_reaction_ids_stable_with_mixed_groups(self):
        """RID assignment is sequential with no gaps across all sub-reactors."""
        node = NodeInfo(
            name="n", class_name="RidTest", package="p", source_file="/f.cpp",
            callbacks=[
                CallbackInfo(name="mutex_cb"),
                CallbackInfo(name="reent_cb"),
            ],
            callback_groups=[
                CallbackGroupInfo(name="mgrp_", type="MutuallyExclusive"),
                CallbackGroupInfo(name="rgrp_", type="Reentrant"),
            ],
            timers=[
                TimerInfo(period_ms=100.0, callback="mutex_cb", callback_group="mgrp_"),
                TimerInfo(period_ms=200.0, callback="reent_cb", callback_group="rgrp_"),
            ],
        )
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            system = generate_declared(self._make_graph(node))
        all_rids = sorted(
            r.reaction_id
            for reactor in system.reactors
            for r in reactor.reactions
            if r.reaction_id >= 0
        )
        assert all_rids == list(range(len(all_rids)))

    def test_two_mutex_groups_share_topic(self):
        """Two Mutex groups subscribing to the same topic share one outer port."""
        node = NodeInfo(
            name="n", class_name="ShareTopic", package="p", source_file="/f.cpp",
            callbacks=[
                CallbackInfo(name="cb_a"),
                CallbackInfo(name="cb_b"),
            ],
            callback_groups=[
                CallbackGroupInfo(name="grpa_", type="MutuallyExclusive"),
                CallbackGroupInfo(name="grpb_", type="MutuallyExclusive"),
            ],
            subscribers=[
                SubscriberInfo(topic="/shared", message_type="std_msgs::msg::String",
                               callback="cb_a", callback_group="grpa_"),
                SubscriberInfo(topic="/shared", message_type="std_msgs::msg::String",
                               callback="cb_b", callback_group="grpb_"),
            ],
        )
        system = self._gen(node)
        outer = next(r for r in system.reactors if r.name == "ShareTopic")
        # One outer input port per topic, forwarded to both groups
        input_ports = [p for p in outer.ports if p.direction == "input"]
        assert [p.name for p in input_ports] == ["shared"]
        # Two sibling sub-reactors, each with one forwarding connection
        sub_a = next(r for r in system.reactors if r.name == "ShareTopic_Grpa")
        sub_b = next(r for r in system.reactors if r.name == "ShareTopic_Grpb")
        assert any(p.direction == "input" for p in sub_a.ports)
        assert any(p.direction == "input" for p in sub_b.ports)
        dests = sorted(c.destination for c in outer.connections)
        assert dests == ["grpa.shared", "grpb.shared"]


class TestMainReactorNameCollision:
    """Main reactor name must not collide with a reactor class name."""

    def test_name_deconfliction(self):
        node = NodeInfo(
            name="timer_node",
            class_name="TimerNode",
            package="timer_example",
            source_file="/fake/timer_node.cpp",
            callbacks=[CallbackInfo(name="on_timer")],
            timers=[TimerInfo(period_ms=1000.0, callback="on_timer")],
        )
        graph = SystemGraph(
            name="timer_node",  # launch_stem_to_reactor_name → "TimerNode"
            launch_graph=_make_launch_graph(),
            nodes={"timer_node": node},
        )
        system = generate_declared(graph)
        # Main reactor should be renamed to avoid collision
        assert system.name == "TimerNodeSystem"
        # Reactor class should still be TimerNode
        assert system.reactors[0].name == "TimerNode"


class TestDigitLeadingLaunchStem:
    """A launch stem starting with a digit gave an invalid LF name.

    ``11_load_composable_nodes`` used to become ``11LoadComposableNodes``,
    which lfc rejects ("LF file names must not start with a number").
    """

    def test_main_reactor_name_is_valid_and_package_name_stable(self):
        from colcon_lf.generation.lf_emitter import emit_lf_system

        node = NodeInfo(
            name="timer_node",
            class_name="TimerNode",
            package="timer_example",
            source_file="/fake/timer_node.cpp",
            callbacks=[CallbackInfo(name="on_timer")],
            timers=[TimerInfo(period_ms=1000.0, callback="on_timer")],
        )
        graph = SystemGraph(
            name="11_load_composable_nodes",
            launch_graph=_make_launch_graph(),
            nodes={"timer_node": node},
        )
        system = generate_declared(graph)
        assert system.name == "Launch_11LoadComposableNodes"
        assert "main reactor Launch_11LoadComposableNodes" in emit_lf_system(system)
        from colcon_lf.generation.lf_naming import generated_package_stem

        assert generated_package_stem(system.name) == "11loadcomposablenodes"


class TestRuntimeToKeyMapping:
    """Test that connections resolve runtime node names to launch keys."""

    def test_runtime_name_differs_from_key(self):
        pub = NodeInfo(
            name="timerpublisher_node",  # Runtime name
            class_name="TimerPublisherNode",
            package="basic_components",
            source_file="/fake/timer_pub.cpp",
            callbacks=[CallbackInfo(name="cb", publishers_used=["timer_out"])],
            timers=[TimerInfo(period_ms=100.0, callback="cb")],
            publishers=[PublisherInfo(topic="timer_out", message_type="std_msgs::msg::String")],
        )
        sub = NodeInfo(
            name="relay_node",
            class_name="RelayNode",
            package="basic_components",
            source_file="/fake/relay.cpp",
            callbacks=[CallbackInfo(name="on_data")],
            subscribers=[SubscriberInfo(
                topic="timer_out", message_type="std_msgs::msg::String", callback="on_data",
            )],
        )
        graph = SystemGraph(
            name="system_full",
            launch_graph=_make_launch_graph(),
            nodes={
                "timer_publisher": pub,   # Launch key differs from runtime name
                "relay_node": sub,
            },
            communication=SystemCommunication(
                topics=[TopicConnection(
                    topic="/timer_out",
                    message_type="std_msgs::msg::String",
                    publishers=["timerpublisher_node"],   # Uses runtime name
                    subscribers=["relay_node"],
                )],
            ),
        )
        system = generate_declared(graph)
        assert len(system.main_connections) == 1
        conn = system.main_connections[0]
        # Connection should use launch keys (instance names), not runtime names
        assert conn.source == "timer_publisher.corr_e0"
        assert conn.destination == "relay_node.corr_e0"
        edge = system.corr_edges[0]
        assert (edge.pub_node, edge.sub_node) == ("timer_publisher", "relay_node")


class TestCustomAfterDelay:
    """Test that custom after delay is propagated."""

    def test_custom_delay(self):
        graph = SystemGraph(
            name="delay_test",
            launch_graph=_make_launch_graph(),
            nodes={
                "pub": NodeInfo(
                    name="pub", class_name="Pub", package="test",
                    source_file="/fake/pub.cpp",
                    callbacks=[CallbackInfo(name="cb", publishers_used=["t"])],
                    timers=[TimerInfo(period_ms=100.0, callback="cb")],
                    publishers=[PublisherInfo(topic="t", message_type="std_msgs::msg::String")],
                ),
                "sub": NodeInfo(
                    name="sub", class_name="Sub", package="test",
                    source_file="/fake/sub.cpp",
                    callbacks=[CallbackInfo(name="cb")],
                    subscribers=[SubscriberInfo(topic="t", message_type="std_msgs::msg::String", callback="cb")],
                ),
            },
            communication=SystemCommunication(
                topics=[TopicConnection(
                    topic="/t", message_type="std_msgs::msg::String",
                    publishers=["pub"], subscribers=["sub"],
                )],
            ),
        )
        system = generate_declared(graph, after_delay_ns=5_000_000)
        # The admitted edge's delay moved from the LF connection into the
        # correspondence edge (drain wake); the corr_e link itself has no after.
        assert system.main_connections[0].after_delay_ns is None
        assert system.corr_edges[0].delay_ns == 5_000_000


def _make_service_launch_graph() -> LaunchGraph:
    root = LaunchFileInfo(file_path="/fake/service_bringup.launch.py", package="service_example")
    return LaunchGraph(root=root, all_nodes=[])


class TestGenerateAsyncServiceServer:
    """Generator emits srv_server reaction for a service server node."""

    def _make_server_node(self) -> NodeInfo:
        return NodeInfo(
            name="service_server",
            class_name="ServiceServerNode",
            package="service_example",
            source_file="/fake/service_server_node.cpp",
            callbacks=[
                CallbackInfo(name="handle_process_data"),
            ],
            service_servers=[
                ServiceServerInfo(
                    service_name="/process_data",
                    service_type="custom_interfaces::srv::ProcessData",
                    callback="handle_process_data",
                ),
            ],
        )

    def test_server_has_request_and_response_ports(self):
        node = self._make_server_node()
        graph = SystemGraph(
            name="service_server",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_server": node},
        )
        system = generate_declared(graph, target_lfc=True)
        # Outer shell has req (input) and resp (output) ports
        server_reactor = next(r for r in system.reactors if r.name == "ServiceServerNode")
        port_names = {p.name: p.direction for p in server_reactor.ports}
        assert "process_data_req" in port_names
        assert port_names["process_data_req"] == "input"
        assert "process_data_resp" in port_names
        assert port_names["process_data_resp"] == "output"

    def test_server_has_srv_server_reaction(self):
        node = self._make_server_node()
        graph = SystemGraph(
            name="service_server",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_server": node},
        )
        system = generate_declared(graph, target_lfc=True)
        # Reaction lives in the DefaultGroup sub-reactor, not the outer shell.
        all_reactions = [r for reactor in system.reactors for r in reactor.reactions]
        srv_reactions = [r for r in all_reactions if r.kind == "srv_server"]
        assert len(srv_reactions) == 1
        reaction = srv_reactions[0]
        assert "process_data_req" in reaction.triggers
        assert "process_data_resp" in reaction.effects
        assert "ros2_lf_invoke_srv_server" in reaction.body

    def test_server_reaction_gets_rid(self):
        node = self._make_server_node()
        graph = SystemGraph(
            name="service_server",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_server": node},
        )
        system = generate_declared(graph, target_lfc=True)
        all_reactions = [r for reactor in system.reactors for r in reactor.reactions]
        srv_reactions = [r for r in all_reactions if r.kind == "srv_server"]
        assert srv_reactions[0].reaction_id >= 0


class TestGenerateAsyncServiceClient:
    """Generator emits srv_send + srv_client_cb reactions for an async client."""

    def _make_client_node(self) -> NodeInfo:
        return NodeInfo(
            name="service_client",
            class_name="ServiceClientNode",
            package="service_example",
            source_file="/fake/service_client_node.cpp",
            callbacks=[
                CallbackInfo(
                    name="timer_callback",
                    service_clients_used=["/process_data"],
                ),
            ],
            timers=[TimerInfo(period_ms=1000.0, callback="timer_callback")],
            service_clients=[
                ServiceClientInfo(
                    service_name="/process_data",
                    service_type="custom_interfaces::srv::ProcessData",
                    call_style="async_callback",
                    response_callback="response_callback",
                ),
            ],
        )

    def test_client_has_request_and_response_ports(self):
        node = self._make_client_node()
        graph = SystemGraph(
            name="service_client",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_client": node},
        )
        system = generate_declared(graph, target_lfc=True)
        # Find client reactor (may be outer shell or sub-reactor)
        client_reactor = next(r for r in system.reactors if r.name == "ServiceClientNode")
        port_names = {p.name: p.direction for p in client_reactor.ports}
        assert "process_data_req" in port_names
        assert port_names["process_data_req"] == "output"
        assert "process_data_resp" in port_names
        assert port_names["process_data_resp"] == "input"

    def test_client_has_srv_client_cb_reaction(self):
        node = self._make_client_node()
        graph = SystemGraph(
            name="service_client",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_client": node},
        )
        system = generate_declared(graph, target_lfc=True)
        all_reactions = [r for reactor in system.reactors for r in reactor.reactions]
        cb_reactions = [r for r in all_reactions if r.kind == "srv_client_cb"]
        assert len(cb_reactions) == 1
        reaction = cb_reactions[0]
        assert "process_data_resp" in reaction.triggers
        assert "ros2_lf_invoke_srv_client_cb" in reaction.body

    def test_timer_callback_upgraded_to_srv_send(self):
        node = self._make_client_node()
        graph = SystemGraph(
            name="service_client",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_client": node},
        )
        system = generate_declared(graph, target_lfc=True)
        all_reactions = [r for reactor in system.reactors for r in reactor.reactions]
        send_reactions = [r for r in all_reactions if r.kind == "srv_send"]
        assert len(send_reactions) == 1
        reaction = send_reactions[0]
        assert "ros2_lf_invoke_srv_send" in reaction.body

    def test_request_port_raised_per_request_sent(self):
        """O1-2 (review round 5): one request write per request really sent.

        Before the fix the body wrote the request port once per timer run,
        so a tick whose client found no server ran a handler for a request
        that never existed ("no request available").
        """
        node = self._make_client_node()
        graph = SystemGraph(
            name="service_client",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_client": node},
        )
        system = generate_declared(graph, target_lfc=True)
        reaction = next(
            r for reactor in system.reactors for r in reactor.reactions if r.kind == "srv_send"
        )
        assert 'ros2_lf_service_sent(' in reaction.body
        assert '"/process_data"' in reaction.body
        unconditional = _stub_add("process_data_req")
        assert all(
            line.strip() != unconditional for line in reaction.body.splitlines()
        ), reaction.body

    def test_subscription_sender_keeps_sub_kind(self):
        """O1-1 (review round 5): a request sent from a subscription callback.

        Before the fix the reaction got kind srv_send, whose RID the binding
        binds as a timer, and launch failed with "no unbound timer handle".
        """
        node = NodeInfo(
            name="service_client",
            class_name="ServiceClientNode",
            package="service_example",
            source_file="/fake/service_client_node.cpp",
            callbacks=[
                CallbackInfo(
                    name="on_range",
                    service_clients_used=["/process_data"],
                ),
            ],
            subscribers=[SubscriberInfo(
                topic="/range", message_type="std_msgs::msg::Float64", callback="on_range",
            )],
            service_clients=[
                ServiceClientInfo(
                    service_name="/process_data",
                    service_type="custom_interfaces::srv::ProcessData",
                    call_style="async_callback",
                    response_callback="response_callback",
                ),
            ],
        )
        graph = SystemGraph(
            name="service_client",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_client": node},
        )
        system = generate_declared(graph, target_lfc=True)
        reactions = [r for reactor in system.reactors for r in reactor.reactions]
        assert [r for r in reactions if r.kind == "srv_send"] == []
        sub = next(r for r in reactions if r.kind == "sub")
        assert "process_data_req" in sub.effects
        body = sub.body
        # The request write sits inside the if-ran block of the invoke.
        invoke_at = body.index("ros2_lf_invoke_sub")
        sent_at = body.index("ros2_lf_service_sent(")
        assert invoke_at < sent_at
        assert body[invoke_at - 4:invoke_at] == "if (", body

    def test_reaction_ids_sequential_with_services(self):
        node = self._make_client_node()
        graph = SystemGraph(
            name="service_client",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_client": node},
        )
        system = generate_declared(graph, target_lfc=True)
        all_rids = sorted(
            r.reaction_id
            for reactor in system.reactors
            for r in reactor.reactions
            if r.reaction_id >= 0
        )
        assert all_rids == list(range(len(all_rids)))


class TestServiceMainConnections:
    """Generator emits bidirectional connections for service pairs."""

    def _make_service_graph(self) -> SystemGraph:
        server = NodeInfo(
            name="service_server",
            class_name="ServiceServerNode",
            package="service_example",
            source_file="/fake/server.cpp",
            callbacks=[CallbackInfo(name="handle_process_data")],
            service_servers=[
                ServiceServerInfo(
                    service_name="/process_data",
                    service_type="custom_interfaces::srv::ProcessData",
                    callback="handle_process_data",
                ),
            ],
        )
        client = NodeInfo(
            name="service_client",
            class_name="ServiceClientNode",
            package="service_example",
            source_file="/fake/client.cpp",
            callbacks=[CallbackInfo(name="timer_callback", service_clients_used=["/process_data"])],
            timers=[TimerInfo(period_ms=1000.0, callback="timer_callback")],
            service_clients=[
                ServiceClientInfo(
                    service_name="/process_data",
                    service_type="custom_interfaces::srv::ProcessData",
                    call_style="async_callback",
                    response_callback="response_callback",
                ),
            ],
        )
        return SystemGraph(
            name="service_bringup",
            launch_graph=_make_service_launch_graph(),
            nodes={"service_server": server, "service_client": client},
            communication=SystemCommunication(
                services=[
                    ServiceConnection(
                        service="/process_data",
                        service_type="custom_interfaces::srv::ProcessData",
                        server="service_server",
                        clients=["service_client"],
                    ),
                ],
            ),
        )

    def test_two_connections_per_service(self):
        system = generate_declared(self._make_service_graph(), target_lfc=True)
        svc_conns = system.main_connections
        assert len(svc_conns) == 2

    def test_request_direction(self):
        system = generate_declared(self._make_service_graph(), target_lfc=True)
        req_conn = next(
            c for c in system.main_connections if "process_data_req" in c.source
        )
        assert "service_client" in req_conn.source
        assert "service_server" in req_conn.destination

    def test_response_direction(self):
        system = generate_declared(self._make_service_graph(), target_lfc=True)
        resp_conn = next(
            c for c in system.main_connections if "process_data_resp" in c.source
        )
        assert "service_server" in resp_conn.source
        assert "service_client" in resp_conn.destination

    def test_connections_have_after_delay(self):
        system = generate_declared(self._make_service_graph(), target_lfc=True)
        for conn in system.main_connections:
            assert conn.after_delay_ns is not None
            assert conn.after_delay_ns >= 0


# ===========================================================================
# Group-bucketing for service/action reactions
# ===========================================================================

def _make_simple_graph(name: str, node: NodeInfo) -> SystemGraph:
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    lg = LaunchGraph(root=root, all_nodes=[])
    return SystemGraph(name=name, launch_graph=lg, nodes={name: node})


class TestServiceServerGroupBucketing:
    """Service server reaction is placed in the correct group sub-reactor."""

    def _make_server_with_group(self, group_name: str | None, group_type: str = "MutuallyExclusive") -> NodeInfo:
        cbg = [CallbackGroupInfo(name=group_name, type=group_type)] if group_name else []
        return NodeInfo(
            name="server",
            class_name="SrvServerNode",
            package="test_pkg",
            source_file="/fake/srv_server.cpp",
            callbacks=[CallbackInfo(name="handle_req")],
            callback_groups=cbg,
            service_servers=[
                ServiceServerInfo(
                    service_name="do_thing",
                    service_type="std_srvs::srv::Trigger",
                    callback="handle_req",
                    callback_group=group_name,
                ),
            ],
        )

    def test_server_reaction_in_default_group_when_no_group(self):
        node = self._make_server_with_group(None)
        system = generate_declared(_make_simple_graph("srv_test", node), target_lfc=True)
        sub = next((r for r in system.reactors if r.name == "SrvServerNode_DefaultGroup"), None)
        assert sub is not None, "DefaultGroup sub-reactor not found"
        kinds = [r.kind for r in sub.reactions]
        assert "srv_server" in kinds

    def test_server_reaction_outer_has_no_srv_server(self):
        node = self._make_server_with_group(None)
        system = generate_declared(_make_simple_graph("srv_test", node), target_lfc=True)
        outer = next(r for r in system.reactors if r.name == "SrvServerNode")
        assert not any(r.kind == "srv_server" for r in outer.reactions)

    def test_server_reaction_in_named_group(self):
        node = self._make_server_with_group("exclusive_group_")
        system = generate_declared(_make_simple_graph("srv_test", node), target_lfc=True)
        # Sub-reactor name is derived from the group name
        sub = next(
            (r for r in system.reactors if r.name.startswith("SrvServerNode_") and r.name != "SrvServerNode"),
            None,
        )
        assert sub is not None
        assert any(r.kind == "srv_server" for r in sub.reactions)

    def test_sub_reactor_has_req_and_resp_ports(self):
        node = self._make_server_with_group(None)
        system = generate_declared(_make_simple_graph("srv_test", node), target_lfc=True)
        sub = next(r for r in system.reactors if r.name == "SrvServerNode_DefaultGroup")
        port_names = {p.name for p in sub.ports}
        assert "do_thing_req" in port_names
        assert "do_thing_resp" in port_names


class TestServiceClientGroupBucketing:
    """Service client response reaction is placed in the correct group sub-reactor."""

    def _make_client_node(self, group_name: str | None = None) -> NodeInfo:
        cbg = [CallbackGroupInfo(name=group_name, type="MutuallyExclusive")] if group_name else []
        return NodeInfo(
            name="client",
            class_name="SrvClientNode",
            package="test_pkg",
            source_file="/fake/srv_client.cpp",
            callbacks=[
                CallbackInfo(name="timer_cb", service_clients_used=["do_thing"]),
            ],
            callback_groups=cbg,
            timers=[TimerInfo(period_ms=1000.0, callback="timer_cb")],
            service_clients=[
                ServiceClientInfo(
                    service_name="do_thing",
                    service_type="std_srvs::srv::Trigger",
                    call_style="async_callback",
                    response_callback="on_response",
                    callback_group=group_name,
                ),
            ],
        )

    def test_client_cb_reaction_in_default_group(self):
        node = self._make_client_node(None)
        system = generate_declared(_make_simple_graph("cli_test", node), target_lfc=True)
        sub = next((r for r in system.reactors if r.name == "SrvClientNode_DefaultGroup"), None)
        assert sub is not None
        assert any(r.kind == "srv_client_cb" for r in sub.reactions)

    def test_client_cb_not_on_outer_reactor(self):
        node = self._make_client_node(None)
        system = generate_declared(_make_simple_graph("cli_test", node), target_lfc=True)
        outer = next(r for r in system.reactors if r.name == "SrvClientNode")
        assert not any(r.kind == "srv_client_cb" for r in outer.reactions)

    def test_sub_reactor_has_resp_port(self):
        node = self._make_client_node(None)
        system = generate_declared(_make_simple_graph("cli_test", node), target_lfc=True)
        sub = next(r for r in system.reactors if r.name == "SrvClientNode_DefaultGroup")
        port_names = {p.name for p in sub.ports}
        assert "do_thing_resp" in port_names


class TestActionGroupBucketing:
    """Action server/client reactions are placed in the correct group sub-reactor."""

    def _make_action_server_node(self, group_name: str | None = None) -> NodeInfo:
        cbg = [CallbackGroupInfo(name=group_name, type="MutuallyExclusive")] if group_name else []
        return NodeInfo(
            name="act_server",
            class_name="ActServerNode",
            package="test_pkg",
            source_file="/fake/act_server.cpp",
            callbacks=[CallbackInfo(name="handle_goal")],
            callback_groups=cbg,
            action_servers=[
                ActionServerInfo(
                    action_name="navigate",
                    action_type="nav2_msgs::action::NavigateToPose",
                    goal_callback="handle_goal",
                    callback_group=group_name,
                ),
            ],
        )

    def _make_action_client_node(self, group_name: str | None = None) -> NodeInfo:
        cbg = [CallbackGroupInfo(name=group_name, type="MutuallyExclusive")] if group_name else []
        return NodeInfo(
            name="act_client",
            class_name="ActClientNode",
            package="test_pkg",
            source_file="/fake/act_client.cpp",
            callbacks=[
                CallbackInfo(name="send_goal_cb"),
            ],
            callback_groups=cbg,
            timers=[TimerInfo(period_ms=2000.0, callback="send_goal_cb")],
            action_clients=[
                ActionClientInfo(
                    action_name="navigate",
                    action_type="nav2_msgs::action::NavigateToPose",
                    goal_response_callback="on_goal_response",
                    feedback_callback="on_feedback",
                    result_callback="on_result",
                    callback_group=group_name,
                ),
            ],
        )

    def test_action_server_reaction_in_default_group(self):
        node = self._make_action_server_node(None)
        system = generate_declared(_make_simple_graph("act_srv_test", node), target_lfc=True)
        sub = next((r for r in system.reactors if r.name == "ActServerNode_DefaultGroup"), None)
        assert sub is not None
        assert any(r.kind == "action_goal_server" for r in sub.reactions)

    def test_action_server_not_on_outer_reactor(self):
        node = self._make_action_server_node(None)
        system = generate_declared(_make_simple_graph("act_srv_test", node), target_lfc=True)
        outer = next(r for r in system.reactors if r.name == "ActServerNode")
        assert not any(r.kind == "action_goal_server" for r in outer.reactions)

    def test_action_client_all_three_reactions_in_default_group(self):
        node = self._make_action_client_node(None)
        system = generate_declared(_make_simple_graph("act_cli_test", node), target_lfc=True)
        sub = next((r for r in system.reactors if r.name == "ActClientNode_DefaultGroup"), None)
        assert sub is not None
        kinds = {r.kind for r in sub.reactions}
        assert "action_goal_cb" in kinds
        assert "action_feedback" in kinds
        assert "action_result_cb" in kinds

    def test_action_client_reactions_not_on_outer(self):
        node = self._make_action_client_node(None)
        system = generate_declared(_make_simple_graph("act_cli_test", node), target_lfc=True)
        outer = next(r for r in system.reactors if r.name == "ActClientNode")
        action_kinds = {"action_goal_cb", "action_feedback", "action_result_cb"}
        assert not any(r.kind in action_kinds for r in outer.reactions)

    def test_action_server_sub_has_goal_req_resp_ports(self):
        node = self._make_action_server_node(None)
        system = generate_declared(_make_simple_graph("act_srv_test", node), target_lfc=True)
        sub = next(r for r in system.reactors if r.name == "ActServerNode_DefaultGroup")
        port_names = {p.name for p in sub.ports}
        assert "navigate_goal_req" in port_names
        assert "navigate_goal_resp" in port_names


class TestActionAsyncChannelsCrossFederate:
    """Regression: feedback, result and cancel must cross a federate

    boundary the same way goal request/response already do. Before this fix,
    ``_build_connections`` built only 2 connections per action (goal_req,
    goal_resp); feedback/result/cancel ports existed only partially (client
    input feedback/result, no server output, no cancel ports at all) and were
    never wired at the top level, so the corresponding client reactions could
    never fire — confirmed against the real federated build (see
    /tmp/report-62.md before-test section).
    """

    def _make_action_server_node(self) -> NodeInfo:
        return NodeInfo(
            name="action_server",
            class_name="ActionServerNode",
            package="action_example",
            source_file="/fake/action_server_node.cpp",
            callbacks=[
                CallbackInfo(name="handle_goal"),
                CallbackInfo(name="handle_cancel"),
                CallbackInfo(name="handle_accepted"),
                CallbackInfo(name="execute"),
            ],
            action_servers=[
                ActionServerInfo(
                    action_name="analyze",
                    action_type="custom_interfaces::action::Analyze",
                    goal_callback="handle_goal",
                    cancel_callback="handle_cancel",
                    accepted_callback="handle_accepted",
                ),
            ],
        )

    def _make_action_client_node(self) -> NodeInfo:
        return NodeInfo(
            name="action_client",
            class_name="ActionClientNode",
            package="action_example",
            source_file="/fake/action_client_node.cpp",
            callbacks=[
                CallbackInfo(name="send_goal", action_clients_used=["analyze"]),
                CallbackInfo(name="goal_response_callback"),
                CallbackInfo(name="cancel_timer_callback", action_clients_used=["analyze"]),
                CallbackInfo(name="feedback_callback"),
                CallbackInfo(name="result_callback"),
            ],
            timers=[TimerInfo(period_ms=2000.0, callback="send_goal")],
            action_clients=[
                ActionClientInfo(
                    action_name="analyze",
                    action_type="custom_interfaces::action::Analyze",
                    goal_response_callback="goal_response_callback",
                    feedback_callback="feedback_callback",
                    result_callback="result_callback",
                ),
            ],
        )

    def _make_graph(self) -> SystemGraph:
        root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="action_example")
        lg = LaunchGraph(root=root, all_nodes=[])
        return SystemGraph(
            name="action_bringup",
            launch_graph=lg,
            nodes={
                "action_server": self._make_action_server_node(),
                "action_client": self._make_action_client_node(),
            },
            communication=SystemCommunication(
                actions=[
                    ActionConnection(
                        action="analyze",
                        action_type="custom_interfaces::action::Analyze",
                        server="action_server",
                        clients=["action_client"],
                    ),
                ],
            ),
        )

    def test_server_has_feedback_result_cancel_ports(self):
        system = generate_declared(self._make_graph(), target_lfc=True)
        sub = next(r for r in system.reactors if r.name == "ActionServerNode_DefaultGroup")
        port_names = {p.name for p in sub.ports}
        assert "analyze_feedback" in port_names
        assert "analyze_result" in port_names
        assert "analyze_cancel_req" in port_names
        assert "analyze_cancel_resp" in port_names

    def test_client_has_cancel_ports(self):
        system = generate_declared(self._make_graph(), target_lfc=True)
        sub = next(r for r in system.reactors if r.name == "ActionClientNode_DefaultGroup")
        port_names = {p.name for p in sub.ports}
        assert "analyze_cancel_req" in port_names
        assert "analyze_cancel_resp" in port_names

    def test_server_has_cancel_server_reaction(self):
        system = generate_declared(self._make_graph(), target_lfc=True)
        sub = next(r for r in system.reactors if r.name == "ActionServerNode_DefaultGroup")
        assert any(r.kind == "action_cancel_server" for r in sub.reactions)

    def test_client_has_cancel_cb_reaction(self):
        system = generate_declared(self._make_graph(), target_lfc=True)
        sub = next(r for r in system.reactors if r.name == "ActionClientNode_DefaultGroup")
        kinds = {r.kind for r in sub.reactions}
        assert "action_cancel_cb" in kinds

    def test_top_level_connections_cross_all_five_channels(self):
        """The falsifying assertion: before the fix this list had length 2
        (goal_req, goal_resp only); feedback/result/cancel_req/cancel_resp
        were never connected."""
        system = generate_declared(self._make_graph(), target_lfc=True)
        conns = system.main_connections
        by_port = {c.source.split(".")[-1]: c for c in conns}
        assert "analyze_goal_req" in by_port, sorted(by_port)
        assert "analyze_goal_resp" in by_port, sorted(by_port)
        assert "analyze_feedback" in by_port, (
            "feedback never crosses the federate boundary — the client's "
            "action_feedback reaction can never fire; sorted(by_port)="
            f"{sorted(by_port)}"
        )
        assert "analyze_result" in by_port, (
            "result never crosses the federate boundary — the client's "
            "action_result_cb reaction can never fire; sorted(by_port)="
            f"{sorted(by_port)}"
        )
        assert "analyze_cancel_req" in by_port, (
            "cancel request never crosses the federate boundary — the "
            f"server's action_cancel_server reaction can never fire; sorted(by_port)="
            f"{sorted(by_port)}"
        )
        assert "analyze_cancel_resp" in by_port, (
            "cancel response never crosses the federate boundary — the "
            f"client's action_cancel_cb reaction can never fire; sorted(by_port)="
            f"{sorted(by_port)}"
        )

        # Direction: feedback/result flow server -> client.
        assert "action_server" in by_port["analyze_feedback"].source
        assert "action_client" in by_port["analyze_feedback"].destination
        assert "action_server" in by_port["analyze_result"].source
        assert "action_client" in by_port["analyze_result"].destination
        # Direction: cancel request flows client -> server, response server -> client.
        assert "action_client" in by_port["analyze_cancel_req"].source
        assert "action_server" in by_port["analyze_cancel_req"].destination
        assert "action_server" in by_port["analyze_cancel_resp"].source
        assert "action_client" in by_port["analyze_cancel_resp"].destination


class TestCountedActionChannels:
    """Action channels are counted ports driven by real sends.

    Before the fix three 20 ms poll timers per server and client pair raised
    feedback, result and cancel_req on every tick, whether anything was sent
    or not, and the result request had no port at all (the server took it from
    its result poll). Now every channel is an ``int`` port that a reaction
    raises by what the binding saw it send (``ros2_lf_action_sent``), and the
    receiving reaction takes that many samples.
    """

    _CHANNELS = (
        "goal_req", "result_req", "cancel_req", "goal_resp", "result", "cancel_resp", "feedback",
    )

    def _graph(self, step_group: str | None = None) -> SystemGraph:
        server = TestActionAsyncChannelsCrossFederate()._make_action_server_node()
        # The stepped server: a constructor timer publishes feedback and ends
        # goals. "?" is what the analysis records for a goal handle member.
        server.callbacks.append(CallbackInfo(
            name="step", feedback_published=["?"], result_published=["?"]))
        server.timers = [TimerInfo(period_ms=100.0, callback="step", callback_group=step_group)]
        if step_group:
            server.callback_groups = [CallbackGroupInfo(name=step_group, type="MutuallyExclusive")]
        graph = TestActionAsyncChannelsCrossFederate()._make_graph()
        graph.nodes["action_server"] = server
        return graph

    @staticmethod
    def _reactor(system, name):
        return next(r for r in system.reactors if r.name == name)

    def test_no_poll_timer_or_poll_reaction(self):
        system = generate_declared(self._graph(), target_lfc=True)
        timers = [t.name for r in system.reactors for t in r.timers]
        assert not [t for t in timers if t.endswith("_poll")], timers
        kinds = [x.kind for r in system.reactors for x in r.reactions]
        assert not [k for k in kinds if k.endswith("_poll")], kinds

    def test_every_action_port_is_counted(self):
        system = generate_declared(self._graph(), target_lfc=True)
        for name in ("ActionServerNode", "ActionClientNode"):
            ports = {p.name: p for p in self._reactor(system, name).ports}
            for channel in self._CHANNELS:
                assert ports[f"analyze_{channel}"].type == "int", (name, channel)

    def test_result_request_has_port_connection_and_server_reaction(self):
        system = generate_declared(self._graph(), target_lfc=True)
        server = {p.name: p for p in self._reactor(system, "ActionServerNode").ports}
        client = {p.name: p for p in self._reactor(system, "ActionClientNode").ports}
        assert server["analyze_result_req"].direction == "input"
        assert client["analyze_result_req"].direction == "output"
        conns = {c.source: c.destination for c in system.main_connections}
        assert conns["action_client.analyze_result_req"] == "action_server.analyze_result_req"
        sub = self._reactor(system, "ActionServerNode_DefaultGroup")
        (rs,) = [r for r in sub.reactions if r.kind == "action_result_server"]
        assert rs.triggers == ["analyze_result_req"]
        assert rs.effects == ["analyze_result"]
        assert (
            'ros2_lf_invoke_action_result_server(self, RID_ActionServerNode_get_result_callback, '
            '"analyze", analyze_result_req->value);'
        ) in rs.body

    def test_server_reactions_take_the_count_and_claim_sends(self):
        system = generate_declared(self._graph(), target_lfc=True)
        sub = self._reactor(system, "ActionServerNode_DefaultGroup")
        (goal,) = [r for r in sub.reactions if r.kind == "action_goal_server"]
        assert goal.effects == ["analyze_goal_resp", "analyze_feedback", "analyze_result"]
        assert "analyze_goal_req->value);" in goal.body
        assert 'ros2_lf_action_sent("analyze", RCLLF_ACTION_GOAL_RESP)' in goal.body
        assert "lf_set(analyze_goal_resp, 0)" not in goal.body
        (step,) = [r for r in sub.reactions if r.kind == "timer"]
        assert {"analyze_feedback", "analyze_result"} <= set(step.effects)
        assert step.body.splitlines()[1:] == [
            '{ int n = ros2_lf_action_sent("analyze", RCLLF_ACTION_FEEDBACK); if (n > 0) '
            f'{{ {_stub_add("analyze_feedback", "n")} }} }}',
            '{ int n = ros2_lf_action_sent("analyze", RCLLF_ACTION_RESULT_RESP); if (n > 0) '
            f'{{ {_stub_add("analyze_result", "n")} }} }}',
        ]

    def test_client_reactions_claim_every_request(self):
        """A cancel request comes from the feedback callback in cancel.launch.py
        and a result request from the goal response handling, so every client
        reaction claims all three request channels."""
        system = generate_declared(self._graph(), target_lfc=True)
        sub = self._reactor(system, "ActionClientNode_DefaultGroup")
        by_kind = {r.kind: r for r in sub.reactions}
        for kind, port in (
            ("action_goal_cb", "analyze_goal_resp"), ("action_feedback", "analyze_feedback"),
            ("action_result_cb", "analyze_result"), ("action_cancel_cb", "analyze_cancel_resp"),
        ):
            r = by_kind[kind]
            assert r.triggers == [port]
            assert f"{port}->value);" in r.body
            assert r.effects == ["analyze_goal_req", "analyze_result_req", "analyze_cancel_req"]
        send = by_kind["action_goal_send"]
        assert 'ros2_lf_action_sent("analyze", RCLLF_ACTION_GOAL_REQ)' in send.body
        assert "lf_set(analyze_goal_req, 0)" not in send.body

    def test_writers_in_two_groups_merge_by_sum(self):
        """Feedback from the step timer's group and the goal reaction's group
        adds up into one event that carries both counts."""
        system = generate_declared(self._graph(step_group="step_group_"), target_lfc=True)
        node = self._reactor(system, "ActionServerNode")
        merges = {r.effects[0]: r for r in node.reactions if r.kind == "port_merge"}
        assert set(merges) == {"analyze_feedback", "analyze_result"}
        fb = merges["analyze_feedback"]
        assert "lf_schedule_int" not in fb.body
        # Every stub and action port merges by a checked sum.
        assert "__builtin_add_overflow(n, step_group.analyze_feedback->value, &n)" in fb.body
        assert "lf_set(analyze_feedback, n);" in fb.body

    def test_two_servers_on_one_node_get_distinct_rids_and_one_step_claims_both(self):
        """Two servers without a GetResult callback used to share the RID macro
        ``RID_<class>_get_result_callback``, a generation error."""
        import re

        graph = self._graph()
        server = graph.nodes["action_server"]
        second = ActionServerInfo(
            action_name="analyze_b", action_type="custom_interfaces::action::Analyze",
            goal_callback="handle_goal_b", cancel_callback="handle_cancel_b",
        )
        server.action_servers.append(second)
        server.callbacks += [
            CallbackInfo(name="handle_goal_b"), CallbackInfo(name="handle_cancel_b")]
        system = generate_declared(graph, target_lfc=True)
        sub = self._reactor(system, "ActionServerNode_DefaultGroup")
        # The shutdown abort (item 6) runs under the first goal server's RID.
        rids = [
            m for r in sub.reactions if r.kind != "action_shutdown"
            for m in re.findall(r"RID_\w+", r.body)
        ]
        assert len(rids) == len(set(rids)), rids
        assert "RID_ActionServerNode_get_result_callback_analyze" in rids
        assert "RID_ActionServerNode_get_result_callback_analyze_b" in rids
        (step,) = [r for r in sub.reactions if r.kind == "timer"]
        assert {
            "analyze_feedback", "analyze_result", "analyze_b_feedback", "analyze_b_result",
        } <= set(step.effects)
        (stop,) = [r for r in sub.reactions if r.kind == "action_shutdown"]
        assert stop.body.startswith(
            "ros2_lf_action_server_shutdown(self, RID_ActionServerNode_handle_goal);")
        assert stop.effects == [
            "analyze_feedback", "analyze_result", "analyze_b_feedback", "analyze_b_result"]

    def test_server_node_gets_one_shutdown_reaction_without_a_rid(self):
        """Item 6: the node's kept pre-shutdown callbacks run in a
        reaction(shutdown) under the goal server's RID, which claims what they
        send, and the reaction gets no RID or input port of its own."""
        system = generate_declared(self._graph(), target_lfc=True)
        sub = self._reactor(system, "ActionServerNode_DefaultGroup")
        (stop,) = [r for r in sub.reactions if r.kind == "action_shutdown"]
        assert stop.triggers == ["shutdown"]
        assert stop.reaction_id == -1
        assert stop.body.splitlines() == [
            "ros2_lf_action_server_shutdown(self, RID_ActionServerNode_handle_goal);",
            '{ int n = ros2_lf_action_sent("analyze", RCLLF_ACTION_FEEDBACK); '
            f'if (n > 0) {{ {_stub_add("analyze_feedback", "n")} }} }}',
            '{ int n = ros2_lf_action_sent("analyze", RCLLF_ACTION_RESULT_RESP); '
            f'if (n > 0) {{ {_stub_add("analyze_result", "n")} }} }}',
        ]
        assert "shutdown" not in {p.name for p in sub.ports}
        assert not [
            r for x in system.reactors if not x.name.startswith("ActionServerNode")
            for r in x.reactions if r.kind == "action_shutdown"
        ]

    def test_result_callback_claims_a_goal_on_another_action(self):
        """A result callback of ``analyze`` that sends a goal on
        ``analyze_b`` claimed only ``analyze``'s requests, so the send count was
        dropped and no ``analyze_b_goal_req`` event reached the server."""
        graph = self._graph()
        client = graph.nodes["action_client"]
        client.action_clients.append(ActionClientInfo(
            action_name="analyze_b", action_type="custom_interfaces::action::Analyze",
            result_callback="result_b"))
        client.callbacks.append(CallbackInfo(name="result_b"))
        system = generate_declared(graph, target_lfc=True)
        sub = self._reactor(system, "ActionClientNode_DefaultGroup")
        (result_a,) = [
            r for r in sub.reactions
            if r.kind == "action_result_cb" and r.triggers == ["analyze_result"]]
        assert "analyze_b_goal_req" in result_a.effects, result_a.effects
        assert 'ros2_lf_action_sent("analyze_b", RCLLF_ACTION_GOAL_REQ)' in result_a.body

    def test_service_callback_claims_the_result_it_sends(self):
        """A service callback that finishes a goal
        (``goal_handle->succeed``) raised no result port, so the count was
        dropped and the client never took the result."""
        graph = self._graph()
        server = graph.nodes["action_server"]
        server.service_servers.append(ServiceServerInfo(
            service_name="finish", service_type="std_srvs::srv::Trigger",
            callback="finish_cb"))
        server.callbacks.append(CallbackInfo(name="finish_cb", result_published=["analyze"]))
        system = generate_declared(graph, target_lfc=True)
        sub = self._reactor(system, "ActionServerNode_DefaultGroup")
        (srv,) = [r for r in sub.reactions if r.kind == "srv_server"]
        assert srv.effects == ["finish_resp", "analyze_feedback", "analyze_result"]
        assert 'ros2_lf_action_sent("analyze", RCLLF_ACTION_RESULT_RESP)' in srv.body

    def test_action_with_two_client_nodes_is_refused(self):
        """The counted response ports fan out to every client,
        but a response goes only to the client that asked, so the other client
        waited for it and traced LOST."""
        graph = self._graph()
        second = TestActionAsyncChannelsCrossFederate()._make_action_client_node()
        second.name, second.class_name = "action_client_2", "ActionClientNode2"
        graph.nodes["action_client_2"] = second
        graph.communication.actions[0].clients.append("action_client_2")
        with pytest.raises(ValueError, match="several client nodes"):
            generate_declared(graph, target_lfc=True)

    def test_an_action_named_by_the_tail_of_another_on_one_node_is_refused(self):
        """The binding counts the sends of ``/right/move`` also
        for the claim ``"move"``, so the two would share counts. ``/left/move``
        and ``/right/move`` pass (TestActionStemsUseDeclaredName)."""
        graph = self._graph()
        server = graph.nodes["action_server"]
        server.action_servers = [
            ActionServerInfo(action_name=name, action_type="custom_interfaces::action::Analyze",
                             goal_callback=f"goal_{i}")
            for i, name in enumerate(("move", "/right/move"))]
        server.callbacks += [CallbackInfo(name="goal_0"), CallbackInfo(name="goal_1")]
        with pytest.raises(ValueError, match="ends in the whole name"):
            generate_declared(graph, target_lfc=True)


class TestNamespacedTopicPortNames:
    """Regression: connection endpoints must reference ports that exist.

    ``_build_ports`` names ports from the node-local topic string the node
    passed to create_publisher/create_subscription ("sensor_data"), while
    ``TopicConnection.topic`` is the namespace-resolved name
    ("/robot_1/sensor_data"). When ``_build_connections`` derived the port name
    from the resolved topic it emitted ``.robot_1_sensor_data``, which no reactor
    declares, and lfc failed with "Not a port" / "Couldn't resolve reference to
    Variable". Reproduces the system_bringup failure.
    """

    @staticmethod
    def _namespaced_graph() -> SystemGraph:
        pub = NodeInfo(
            name="publisher_node", class_name="PublisherNode",
            package="pubsub_example", source_file="/fake/publisher_node.cpp",
            callbacks=[CallbackInfo(name="timer_callback", publishers_used=["sensor_data"])],
            timers=[TimerInfo(period_ms=500.0, callback="timer_callback")],
            # Node-local topic name, as extracted from the node source.
            publishers=[PublisherInfo(
                topic="sensor_data",
                message_type="custom_interfaces::msg::SensorData")],
        )
        sub = NodeInfo(
            name="subscriber_node", class_name="SubscriberNode",
            package="pubsub_example", source_file="/fake/subscriber_node.cpp",
            callbacks=[CallbackInfo(name="sub_callback")],
            subscribers=[SubscriberInfo(
                topic="sensor_data",
                message_type="custom_interfaces::msg::SensorData",
                callback="sub_callback")],
        )
        return SystemGraph(
            name="namespaced_system",
            launch_graph=_make_launch_graph(),
            nodes={"robot_1_publisher_node": pub, "robot_1_subscriber_node": sub},
            communication=SystemCommunication(
                topics=[TopicConnection(
                    # Namespace-resolved, as the comm analysis produces it.
                    topic="/robot_1/sensor_data",
                    message_type="custom_interfaces::msg::SensorData",
                    publishers=["publisher_node"],
                    subscribers=["subscriber_node"],
                )],
            ),
        )

    def test_connection_uses_node_local_port_name(self):
        system = generate_declared(self._namespaced_graph())
        assert len(system.main_connections) == 1
        conn = system.main_connections[0]
        # Admitted, so the link is corr_e0; the node-local names
        # still resolve by path suffix.
        assert conn.source == "robot_1_publisher_node.corr_e0"
        assert conn.destination == "robot_1_subscriber_node.corr_e0"
        edge = system.corr_edges[0]
        assert (edge.pub_topic, edge.sub_topic) == ("sensor_data", "sensor_data")

    def test_every_connection_endpoint_resolves_to_a_declared_port(self):
        """The invariant lfc enforces: no dangling port references."""
        system = generate_declared(self._namespaced_graph(), target_lfc=True)
        declared = {
            f"{inst}.{port.name}"
            for inst, reactor_class in (
                (i.name, i.reactor_class) for i in system.main_instances
            )
            for reactor in system.reactors if reactor.name == reactor_class
            for port in reactor.ports
        }
        for conn in system.main_connections:
            assert conn.source in declared, (
                f"connection source {conn.source!r} is not a declared port; "
                f"declared: {sorted(declared)}")
            assert conn.destination in declared, (
                f"connection destination {conn.destination!r} is not a declared "
                f"port; declared: {sorted(declared)}")


class TestMultiSegmentNodeLocalTopicPortNames:
    """Regression: node-local topics containing ``/`` must resolve to the

    same port name in both the declaration and the connection.

    ``_build_ports`` declares a port from the *full* node-local topic string
    via ``topic_to_port_name`` (e.g. ``"basic_components/timer_out"`` ->
    ``basic_components_timer_out``, matching basic_components' relay_node.cpp,
    which passes multi-segment topic literals to create_publisher /
    create_subscription). ``_build_connections`` previously derived the port
    name from only the *last path segment* of the connection's resolved topic
    (``"timer_out"``), which no reactor declares, and lfc failed with
    "Not a port" (ComboPipeline.lf). Both this multi-segment
    node-local case and the namespace-resolved case (single-segment
    node-local name, multi-segment resolved topic) must work simultaneously.
    """

    @staticmethod
    def _multi_segment_graph() -> SystemGraph:
        pub = NodeInfo(
            name="timer_node", class_name="TimerNode",
            package="basic_components", source_file="/fake/timer_node.cpp",
            callbacks=[CallbackInfo(name="timer_callback", publishers_used=["basic_components/timer_out"])],
            timers=[TimerInfo(period_ms=500.0, callback="timer_callback")],
            # Node-local topic name containing '/', as basic_components' nodes use.
            publishers=[PublisherInfo(
                topic="basic_components/timer_out",
                message_type="std_msgs::msg::String")],
        )
        sub = NodeInfo(
            name="relay_node", class_name="RelayNode",
            package="basic_components", source_file="/fake/relay_node.cpp",
            callbacks=[CallbackInfo(name="on_message")],
            subscribers=[SubscriberInfo(
                topic="basic_components/timer_out",
                message_type="std_msgs::msg::String",
                callback="on_message")],
        )
        return SystemGraph(
            name="combo_pipeline",
            launch_graph=_make_launch_graph(),
            nodes={"timer_node": pub, "relay_node": sub},
            communication=SystemCommunication(
                topics=[TopicConnection(
                    # Fully resolved (no ROS namespace applied, so just a
                    # leading slash prepended to the node-local topic).
                    topic="/basic_components/timer_out",
                    message_type="std_msgs::msg::String",
                    publishers=["timer_node"],
                    subscribers=["relay_node"],
                )],
            ),
        )

    def test_connection_uses_multi_segment_port_name(self):
        system = self._generate()
        assert len(system.main_connections) == 1
        conn = system.main_connections[0]
        # Admitted, so the link is corr_e0; the multi-segment
        # node-local names still resolve by path suffix.
        assert conn.source == "timer_node.corr_e0"
        assert conn.destination == "relay_node.corr_e0"
        edge = system.corr_edges[0]
        assert edge.status == "admitted", edge.reason
        assert "basic_components" in edge.pub_topic and "timer_out" in edge.sub_topic

    def test_every_connection_endpoint_resolves_to_a_declared_port(self):
        """The invariant lfc enforces: no dangling port references."""
        system = self._generate(target_lfc=True)
        declared = {
            f"{inst}.{port.name}"
            for inst, reactor_class in (
                (i.name, i.reactor_class) for i in system.main_instances
            )
            for reactor in system.reactors if reactor.name == reactor_class
            for port in reactor.ports
        }
        for conn in system.main_connections:
            assert conn.source in declared, (
                f"connection source {conn.source!r} is not a declared port; "
                f"declared: {sorted(declared)}")
            assert conn.destination in declared, (
                f"connection destination {conn.destination!r} is not a declared "
                f"port; declared: {sorted(declared)}")

    def _generate(self, *, target_lfc: bool = False):
        return generate_declared(self._multi_segment_graph(), target_lfc=target_lfc)


class TestFederatedBuildConnectionsPortNames:
    """Regression (federated path): ``_generate_federated`` in

    ``colcon_lf/verb/generate.py`` calls ``_build_connections`` directly
    (mirrors ``generate_lf_system``'s internals rather than calling it) to
    build the full connection set before per-federate classification. That
    call site must also pass ``nodes=`` — otherwise federated builds keep the
    wrong connections even after the single-process path (``generate_lf_system``) is
    fixed. This exercises ``_build_connections`` the same way
    ``_generate_federated`` does: with the ``runtime_to_key``/
    ``key_to_instance`` maps built by hand from ``graph.nodes`` (verb/generate.py
    lines ~698-707), rather than going through ``generate_lf_system``.
    """

    def test_multi_segment_topic_resolves_with_nodes_kwarg(self):
        from colcon_lf.generation.lf_generator import _build_connections
        from colcon_lf.generation.lf_naming import sanitize_lf_identifier

        graph = TestMultiSegmentNodeLocalTopicPortNames._multi_segment_graph()
        runtime_to_key = {ni.name: key for key, ni in graph.nodes.items()}
        key_to_instance = {key: sanitize_lf_identifier(key) for key in graph.nodes}

        connections = _build_connections(
            graph.communication,
            set(graph.nodes.keys()),
            runtime_to_key=runtime_to_key,
            key_to_instance=key_to_instance,
            nodes=graph.nodes,
        )
        assert len(connections) == 1
        conn = connections[0]
        assert conn.source == "timer_node.basic_components_timer_out"
        assert conn.destination == "relay_node.basic_components_timer_out"

    def test_omitting_nodes_kwarg_falls_back_to_last_segment(self):
        """Documents the fallback behavior when nodes= is not passed at all

        (distinct from being passed but not matching): every other caller
        that doesn't have node info handy still gets the earlier heuristic,
        not a crash.
        """
        from colcon_lf.generation.lf_generator import _build_connections
        from colcon_lf.generation.lf_naming import sanitize_lf_identifier

        graph = TestMultiSegmentNodeLocalTopicPortNames._multi_segment_graph()
        runtime_to_key = {ni.name: key for key, ni in graph.nodes.items()}
        key_to_instance = {key: sanitize_lf_identifier(key) for key in graph.nodes}

        connections = _build_connections(
            graph.communication,
            set(graph.nodes.keys()),
            runtime_to_key=runtime_to_key,
            key_to_instance=key_to_instance,
            # nodes= omitted on purpose.
        )
        assert len(connections) == 1
        conn = connections[0]
        assert conn.source == "timer_node.timer_out"
        assert conn.destination == "relay_node.timer_out"


def _declared_ports(system) -> set[str]:
    """Every ``instance.port`` the main reactor can reference."""
    return {
        f"{inst.name}.{port.name}"
        for inst in system.main_instances
        for reactor in system.reactors if reactor.name == inst.reactor_class
        for port in reactor.ports
    }


def _assert_connections_resolve(system) -> None:
    """The invariant lfc enforces: no dangling port references."""
    declared = _declared_ports(system)
    for conn in system.main_connections:
        for end in (conn.source, conn.destination):
            assert end in declared, (
                f"Couldn't resolve reference to Variable {end.split('.', 1)[1]!r}: "
                f"{end!r} is not a declared port; declared: {sorted(declared)}")


class TestNamespacedServicePortNames:
    """Regression: a namespaced service name links the ports it declares.

    ``_build_ports`` names service ports from the declared name, so
    ``/rebind/add`` gives ``rebind_add_req``/``rebind_add_resp``, while
    ``_build_connections`` took only the last segment of the resolved name and
    linked ``add_req``/``add_resp``, which lfc rejects with "Couldn't resolve
    reference to Variable 'add_req'".
    """

    @staticmethod
    def _graph(server_name: str, client_name: str, resolved: str) -> SystemGraph:
        server = NodeInfo(
            name="responder", class_name="Responder", package="rebind",
            source_file="/fake/responder.cpp",
            callbacks=[CallbackInfo(name="handle_add")],
            service_servers=[ServiceServerInfo(
                service_name=server_name,
                service_type="example_interfaces::srv::AddTwoInts",
                callback="handle_add")],
        )
        client = NodeInfo(
            name="sleeper", class_name="Sleeper", package="rebind",
            source_file="/fake/sleeper.cpp",
            callbacks=[CallbackInfo(name="tick", service_clients_used=[client_name])],
            timers=[TimerInfo(period_ms=100.0, callback="tick")],
            service_clients=[ServiceClientInfo(
                service_name=client_name,
                service_type="example_interfaces::srv::AddTwoInts",
                call_style="async_callback",
                response_callback="on_sum")],
        )
        return SystemGraph(
            name="rebind_system",
            launch_graph=_make_launch_graph(),
            nodes={"responder": server, "sleeper": client},
            communication=SystemCommunication(services=[ServiceConnection(
                service=resolved,
                service_type="example_interfaces::srv::AddTwoInts",
                server="responder", clients=["sleeper"])]),
        )

    @pytest.mark.parametrize("server_name,client_name,resolved,server_stem,client_stem", [
        ("/rebind/add", "/rebind/add", "/rebind/add", "rebind_add", "rebind_add"),
        ("/a/b/srv", "/a/b/srv", "/a/b/srv", "a_b_srv", "a_b_srv"),
        # Node-local names keep the short port (the earlier case).
        ("process_data", "process_data", "/service_example/process_data",
         "process_data", "process_data"),
        # Each endpoint keeps its own declared name.
        ("/rebind/add", "add", "/rebind/add", "rebind_add", "add"),
    ])
    def test_connections_use_each_endpoint_declared_port(
        self, server_name, client_name, resolved, server_stem, client_stem,
    ):
        system = generate_declared(
            self._graph(server_name, client_name, resolved), target_lfc=True)
        _assert_connections_resolve(system)
        pairs = {(c.source, c.destination) for c in system.main_connections}
        assert pairs == {
            (f"sleeper.{client_stem}_req", f"responder.{server_stem}_req"),
            (f"responder.{server_stem}_resp", f"sleeper.{client_stem}_resp"),
        }


class TestNamespacedActionPortNames:
    """Action ports and links take the declared name, like service ports, so
    ``/rebind/fib`` links ``rebind_fib_goal_req``."""

    @staticmethod
    def _graph(name: str) -> SystemGraph:
        server = NodeInfo(
            name="action_server", class_name="ActionServerNode", package="rebind",
            source_file="/fake/action_server.cpp",
            callbacks=[CallbackInfo(name=n) for n in (
                "handle_goal", "handle_cancel", "handle_accepted")],
            action_servers=[ActionServerInfo(
                action_name=name, action_type="example_interfaces::action::Fibonacci",
                goal_callback="handle_goal", cancel_callback="handle_cancel",
                accepted_callback="handle_accepted")],
        )
        client = NodeInfo(
            name="action_client", class_name="ActionClientNode", package="rebind",
            source_file="/fake/action_client.cpp",
            callbacks=[
                CallbackInfo(name="send_goal", action_clients_used=[name]),
                CallbackInfo(name="goal_response_callback"),
                CallbackInfo(name="feedback_callback"),
                CallbackInfo(name="result_callback"),
            ],
            timers=[TimerInfo(period_ms=1000.0, callback="send_goal")],
            action_clients=[ActionClientInfo(
                action_name=name, action_type="example_interfaces::action::Fibonacci",
                goal_response_callback="goal_response_callback",
                feedback_callback="feedback_callback",
                result_callback="result_callback")],
        )
        return SystemGraph(
            name="rebind_actions",
            launch_graph=_make_launch_graph(),
            nodes={"action_server": server, "action_client": client},
            communication=SystemCommunication(actions=[ActionConnection(
                action=name, action_type="example_interfaces::action::Fibonacci",
                server="action_server", clients=["action_client"])]),
        )

    @pytest.mark.parametrize("name", ["/rebind/fib", "/a/b/fib"])
    def test_namespaced_action_links_resolve(self, name):
        system = generate_declared(self._graph(name), target_lfc=True)
        _assert_connections_resolve(system)
        stem = name.lstrip("/").replace("/", "_")
        assert (f"action_client.{stem}_goal_req", f"action_server.{stem}_goal_req") in {
            (c.source, c.destination) for c in system.main_connections}

    @pytest.mark.parametrize("name", ["/rebind/fib", "/a/b/fib", "fib"])
    def test_action_invokes_carry_declared_name(self, name):
        """The binding resolves this string and binds the waitable of that name;
        ``"fib"`` resolves to ``/fib`` and leaves ``/rebind/fib`` unbound, so the
        server never takes a goal (lifecycle_rebind_example ``responder_goals``
        stayed 0)."""
        import re
        system = generate_declared(self._graph(name), target_lfc=True)
        invoked = {
            m.group(1)
            for r in system.reactors for rx in r.reactions
            for m in re.finditer(r'ros2_lf_invoke_action_\w+\([^,]+,\s*[^,]+,\s*"([^"]*)"',
                                 rx.body)
        }
        assert invoked == {name}, (
            f"action invokes name {sorted(invoked)}, declared {name!r}")


def _assert_unique_ports(system) -> None:
    """The invariant lfc enforces: one name per port in a reactor."""
    for reactor in system.reactors:
        names = [p.name for p in reactor.ports]
        dups = sorted({n for n in names if names.count(n) > 1})
        assert not dups, (
            f"Names of contained objects (inputs, outputs, actions, timers, "
            f"parameters, state, and reactors) must be unique: {dups} in "
            f"{reactor.name}")


def _generate_or_lfc_error(graph: SystemGraph, *expected: str):
    """Generate *graph*; on success, fail with the error lfc would raise.

    A refused generation must name every string in *expected* (the node and
    the clashing names), so the author can see what to rename.
    """
    try:
        system = generate_declared(graph, target_lfc=True)
    except ValueError as exc:
        for word in expected:
            assert word in str(exc), f"{word!r} missing from: {exc}"
        return
    _assert_unique_ports(system)
    _assert_connections_resolve(system)
    pytest.fail(f"generated a program instead of refusing ({expected})")


def _launch_graph(*actions: tuple[str, str, list]) -> LaunchGraph:
    """Launch graph of ``(name, namespace, remappings)`` node actions."""
    return LaunchGraph(
        root=LaunchFileInfo(file_path="/fake/bringup.launch.py", package="rebind"),
        all_nodes=[LaunchNodeAction(package="rebind", executable=name, name=name,
                                    namespace=ns, remappings=list(remaps))
                   for name, ns, remaps in actions])


def _adder(name: str, servers=(), clients=()) -> NodeInfo:
    return NodeInfo(
        name=name, class_name=name.title(), package="rebind",
        source_file=f"/fake/{name}.cpp",
        callbacks=[CallbackInfo(name="handle_add"), CallbackInfo(name="on_sum"),
                   CallbackInfo(name="tick", service_clients_used=list(clients))],
        timers=[TimerInfo(period_ms=100.0, callback="tick")] if clients else [],
        service_servers=[ServiceServerInfo(
            service_name=s, service_type="example_interfaces::srv::AddTwoInts",
            callback="handle_add") for s in servers],
        service_clients=[ServiceClientInfo(
            service_name=c, service_type="example_interfaces::srv::AddTwoInts",
            call_style="async_callback", response_callback="on_sum") for c in clients],
    )


class TestPrivateServiceNames:
    """A private ``~/api/add`` on ``responder`` in ``/ns`` serves
    ``/ns/responder/api/add``. Neither the analyzer
    nor the generator expanded ``~``, so the scoped match failed and the link
    named a port from the last segment, ``add_req``, that does not exist."""

    def test_resolve_name_expands_private_names(self):
        assert resolve_name("~/api/add", "/ns", [], node_name="responder") == \
            "/ns/responder/api/add"
        assert resolve_name("~/api/add", None, [], node_name="responder") == \
            "/responder/api/add"
        assert resolve_name("~", "/ns", [], node_name="responder") == "/ns/responder"
        assert resolve_name("~/api/add", "/ns", [("~/api/add", "/x")],
                            node_name="responder") == "/x"

    def test_private_remap_destination_expands(self):
        # A private remap target expands like a private name.
        assert resolve_name("~/api/add", "/ns", [("~/api/add", "~/api/other")],
                            node_name="responder") == "/ns/responder/api/other"

    def test_analyzer_pairs_private_server(self):
        server = _adder("responder", servers=["~/api/add"])
        client = _adder("sleeper", clients=["responder/api/add"])
        launch = _launch_graph(("responder", "/ns", []), ("sleeper", "/ns", []))
        comm = build_communication(list(zip(launch.all_nodes, [server, client])))
        assert [(s.service, s.server, s.clients) for s in comm.services] == [
            ("/ns/responder/api/add", "responder", ["sleeper"])]

    def test_private_server_links_its_declared_port(self):
        server = _adder("responder", servers=["~/api/add"])
        client = _adder("sleeper", clients=["responder/api/add"])
        graph = SystemGraph(
            name="private_system",
            launch_graph=_launch_graph(("responder", "/ns", []), ("sleeper", "/ns", [])),
            nodes={"/ns/responder": server, "/ns/sleeper": client},
            communication=SystemCommunication(services=[ServiceConnection(
                service="/ns/responder/api/add",
                service_type="example_interfaces::srv::AddTwoInts",
                server="responder", clients=["sleeper"])]),
        )
        system = generate_declared(graph, target_lfc=True)
        _assert_connections_resolve(system)
        assert ("ns_sleeper.responder_api_add_req", "ns_responder.api_add_req") in {
            (c.source, c.destination) for c in system.main_connections}


class TestAmbiguousServiceMatch:
    """Two declared names remapped onto one service (finding 2): the link fell
    back to the last segment, ``shared_req``, which no port carries. Generation
    must refuse and name the node and both declared names."""

    def test_two_declarations_on_one_service_refuse(self):
        server = _adder("responder", servers=["alpha", "beta"])
        client = _adder("sleeper", clients=["/shared"])
        graph = SystemGraph(
            name="ambiguous_system",
            launch_graph=_launch_graph(
                ("responder", "", [("alpha", "/shared"), ("beta", "/shared")]),
                ("sleeper", "", [])),
            nodes={"responder": server, "sleeper": client},
            communication=SystemCommunication(services=[ServiceConnection(
                service="/shared", service_type="example_interfaces::srv::AddTwoInts",
                server="responder", clients=["sleeper"])]),
        )
        _generate_or_lfc_error(graph, "responder", "alpha", "beta", "/shared")


class TestServicePortCollision:
    """``/a/b`` and ``/a_b`` both name ``a_b_req`` (finding 3, already so on
    main). Generation must refuse instead of declaring the port twice."""

    def test_colliding_service_names_refuse(self):
        server = _adder("responder", servers=["/a/b", "/a_b"])
        graph = SystemGraph(
            name="collision_system", launch_graph=_make_launch_graph(),
            nodes={"responder": server},
            communication=SystemCommunication(services=[
                ServiceConnection(service=s, service_type="example_interfaces::srv::AddTwoInts",
                                  server="responder", clients=[])
                for s in ("/a/b", "/a_b")]),
        )
        _generate_or_lfc_error(graph, "responder", "/a/b", "/a_b", "a_b_req")


class TestActionStemsUseDeclaredName:
    """``/a/fib`` and ``/b/fib`` on one client both took the stem ``fib`` and
    declared ``fib_goal_req`` twice (finding 4, already so on main). Action
    ports now take the full declared name, like service ports."""

    @staticmethod
    def _server(name: str, action: str) -> NodeInfo:
        return NodeInfo(
            name=name, class_name=name.title(), package="rebind",
            source_file=f"/fake/{name}.cpp",
            callbacks=[CallbackInfo(name=n) for n in (
                "handle_goal", "handle_cancel", "handle_accepted")],
            action_servers=[ActionServerInfo(
                action_name=action, action_type="example_interfaces::action::Fibonacci",
                goal_callback="handle_goal", cancel_callback="handle_cancel",
                accepted_callback="handle_accepted")],
        )

    def test_same_last_segment_actions_get_distinct_ports(self):
        actions = ["/a/fib", "/b/fib"]
        client = NodeInfo(
            name="action_client", class_name="ActionClientNode", package="rebind",
            source_file="/fake/action_client.cpp",
            callbacks=[CallbackInfo(name="send_goal", action_clients_used=actions)] + [
                CallbackInfo(name=n) for n in (
                    "goal_response_callback", "feedback_callback", "result_callback")],
            timers=[TimerInfo(period_ms=1000.0, callback="send_goal")],
            action_clients=[ActionClientInfo(
                action_name=a, action_type="example_interfaces::action::Fibonacci",
                goal_response_callback="goal_response_callback",
                feedback_callback="feedback_callback",
                result_callback="result_callback") for a in actions],
        )
        graph = SystemGraph(
            name="two_fibs", launch_graph=_make_launch_graph(),
            nodes={"server_a": self._server("server_a", "/a/fib"),
                   "server_b": self._server("server_b", "/b/fib"),
                   "action_client": client},
            communication=SystemCommunication(actions=[
                ActionConnection(action=a, action_type="example_interfaces::action::Fibonacci",
                                 server=s, clients=["action_client"])
                for a, s in (("/a/fib", "server_a"), ("/b/fib", "server_b"))]),
        )
        system = generate_declared(graph, target_lfc=True)
        _assert_unique_ports(system)
        _assert_connections_resolve(system)
        pairs = {(c.source, c.destination) for c in system.main_connections}
        assert ("action_client.a_fib_goal_req", "server_a.a_fib_goal_req") in pairs
        assert ("action_client.b_fib_goal_req", "server_b.b_fib_goal_req") in pairs
        timers = [t.name for r in system.reactors for t in r.timers]
        assert len(timers) == len(set(timers)), f"duplicate timers: {timers}"
