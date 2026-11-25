# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Phase 2 generator tests: lfc-path model additions and reaction-ID assignment."""

from __future__ import annotations

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.generation.lf_emitter import _ns_to_lf_time, emit_lf_system
from colcon_lf.generation.lf_generator import (
    DEFAULT_AFTER_DELAY_NS,
    STUB_TYPE,
    generate_lf_system,
)
from ..corr_declared import generate_declared
from colcon_lf.generation.lf_model import (
    LFConnection,
    LFPhysicalAction,
    LFReaction,
    LFReactor,
    LFReactorInstance,
    LFSystem,
    LFTimer,
)
from colcon_lf.graph.models import SystemGraph
from colcon_lf.node_analysis.models import (
    CallbackGroupInfo,
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    ServiceClientInfo,
    SubscriberInfo,
    TimerInfo,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _make_pubsub_graph() -> SystemGraph:
    return SystemGraph(
        name="pubsub_bringup",
        launch_graph=None,
        nodes={
            "publisher_node": NodeInfo(
                name="publisher_node",
                class_name="PublisherNode",
                package="pubsub_example",
                source_file="pub.cpp",
                timers=[TimerInfo(period_ms=500.0, callback="timer_callback")],
                publishers=[PublisherInfo(topic="/sensor_data", message_type="custom_interfaces::msg::SensorData")],
                callbacks=[
                    CallbackInfo(name="timer_callback", publishers_used=["/sensor_data"]),
                ],
            ),
            "subscriber_node": NodeInfo(
                name="subscriber_node",
                class_name="SubscriberNode",
                package="pubsub_example",
                source_file="sub.cpp",
                subscribers=[SubscriberInfo(topic="/sensor_data", message_type="custom_interfaces::msg::SensorData", callback="topic_callback")],
                callbacks=[
                    CallbackInfo(name="topic_callback"),
                ],
            ),
        },
        communication=SystemCommunication(
            topics=[TopicConnection(
                topic="/sensor_data",
                message_type="custom_interfaces::msg::SensorData",
                publishers=["publisher_node"],
                subscribers=["subscriber_node"],
            )],
        ),
    )


# ---------------------------------------------------------------------------
# LFModel: new fields
# ---------------------------------------------------------------------------

class TestLFModelNewFields:
    def test_lf_reaction_has_reaction_id(self):
        r = LFReaction(triggers=["t"], effects=[], body="")
        assert r.reaction_id == -1

    def test_lf_reaction_has_kind(self):
        r = LFReaction(triggers=["t"], effects=[], body="", kind="timer")
        assert r.kind == "timer"

    def test_lf_reactor_has_startup_body(self):
        r = LFReactor(name="Foo")
        assert r.startup_body is None

    def test_lf_reactor_has_physical_actions(self):
        r = LFReactor(name="Foo")
        assert r.physical_actions == []

    def test_lf_connection_has_after_delay_ns(self):
        c = LFConnection(source="a.p", destination="b.p")
        assert c.after_delay_ns is None

    def test_lf_connection_after_delay_ns_set(self):
        c = LFConnection(source="a.p", destination="b.p", after_delay_ns=5_000_000)
        assert c.after_delay_ns == 5_000_000

    def test_lf_physical_action(self):
        pa = LFPhysicalAction(name="pa_srv_resp", min_delay_ns=1_000)
        assert pa.name == "pa_srv_resp"
        assert pa.min_delay_ns == 1_000

    def test_lf_system_target_options(self):
        s = LFSystem(name="X", target="CCpp", target_options={"keepalive": True})
        assert s.target_options["keepalive"] is True

    def test_lf_system_preamble(self):
        s = LFSystem(name="X", target="CCpp", preamble='#include "foo.h"')
        assert "#include" in s.preamble


# ---------------------------------------------------------------------------
# ns_to_lf_time helper
# ---------------------------------------------------------------------------

class TestNsToLfTime:
    def test_zero(self):
        assert _ns_to_lf_time(0) == "0"

    def test_nanoseconds(self):
        assert _ns_to_lf_time(500) == "500 nsec"

    def test_microseconds(self):
        assert _ns_to_lf_time(5_000) == "5 usec"

    def test_milliseconds(self):
        assert _ns_to_lf_time(5_000_000) == "5 msec"

    def test_seconds(self):
        assert _ns_to_lf_time(1_000_000_000) == "1 sec"

    def test_fractional_ms(self):
        # 1500 ns = 1.5 us → represented as 1500 nsec (not exact us multiple)
        assert _ns_to_lf_time(1500) == "1500 nsec"


# ---------------------------------------------------------------------------
# Emitter: target options block
# ---------------------------------------------------------------------------

class TestEmitTargetOptions:
    def test_simple_target_emitted_without_block(self):
        s = LFSystem(name="X", target="CCpp")
        text = emit_lf_system(s)
        assert "target CCpp\n" in text

    def test_target_options_emitted_as_block(self):
        s = LFSystem(name="X", target="CCpp", target_options={"keepalive": True, "build-type": "RelWithDebInfo"})
        text = emit_lf_system(s)
        assert "target CCpp {" in text
        assert "keepalive: true" in text
        assert "build-type: RelWithDebInfo" in text

    def test_preamble_emitted(self):
        s = LFSystem(name="X", target="CCpp", preamble='#include "foo.h"\ntypedef int stub_t;')
        text = emit_lf_system(s)
        assert "preamble {=" in text
        assert '#include "foo.h"' in text
        assert "typedef int stub_t;" in text


# ---------------------------------------------------------------------------
# Emitter: startup reaction
# ---------------------------------------------------------------------------

class TestEmitStartupReaction:
    def test_startup_reaction_emitted(self):
        r = LFReactor(name="MyReactor", startup_body='ros2_lf_bind_node(self, "My");')
        s = LFSystem(name="Main", target="CCpp", reactors=[r])
        text = emit_lf_system(s)
        assert "reaction(startup)" in text
        assert 'ros2_lf_bind_node(self, "My");' in text

    def test_no_startup_reaction_when_none(self):
        r = LFReactor(name="MyReactor")
        s = LFSystem(name="Main", target="CCpp", reactors=[r])
        text = emit_lf_system(s)
        assert "reaction(startup)" not in text


# ---------------------------------------------------------------------------
# Emitter: physical actions
# ---------------------------------------------------------------------------

class TestEmitPhysicalAction:
    def test_physical_action_emitted(self):
        r = LFReactor(name="Foo", physical_actions=[LFPhysicalAction(name="pa_resp", min_delay_ns=1_000_000)])
        s = LFSystem(name="Main", target="CCpp", reactors=[r])
        text = emit_lf_system(s)
        assert "physical action pa_resp(1 msec): stub_t" in text

    def test_physical_action_zero_delay(self):
        r = LFReactor(name="Foo", physical_actions=[LFPhysicalAction(name="pa_resp", min_delay_ns=0)])
        s = LFSystem(name="Main", target="CCpp", reactors=[r])
        text = emit_lf_system(s)
        assert "physical action pa_resp(0): stub_t" in text


# ---------------------------------------------------------------------------
# Generator: stub_t ports and target_lfc
# ---------------------------------------------------------------------------

class TestTargetLfcMode:
    def test_stub_type_ports(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph, target_lfc=True)
        for reactor in system.reactors:
            for port in reactor.ports:
                # Admitted edges carry rcllf_corr_t ports.
                expected = "rcllf_corr_t" if port.name.startswith("corr_e") else STUB_TYPE
                assert port.type == expected, f"Port {port.name} type should be {expected}"

    def test_target_lfc_false_is_ignored(self):
        """The stock rendering is gone, False is a no-op."""
        graph = _make_pubsub_graph()
        system = generate_declared(graph, target_lfc=False)
        pub_reactor = next(r for r in system.reactors if r.name == "PublisherNode")
        assert {p.type for p in pub_reactor.ports} <= {"stub_t", "rcllf_corr_t"}

    def test_startup_body_set_in_lfc_mode(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph, target_lfc=True)
        # Startup lives on the outer (node-shell) reactors only; sub-reactors have None
        outer_reactors = [r for r in system.reactors if r.instances]
        for reactor in outer_reactors:
            assert reactor.startup_body is not None
            assert "ros2_lf_bind_node" in reactor.startup_body

    def test_lfc_reaction_body_timer(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph, target_lfc=True)
        # Reactions live in the DefaultGroup sub-reactor
        pub_sub = next(r for r in system.reactors if r.name == "PublisherNode_DefaultGroup")
        timer_reaction = next(r for r in pub_sub.reactions if r.kind == "timer")
        assert "ros2_lf_invoke_timer" in timer_reaction.body
        assert "RID_PublisherNode_timer_callback" in timer_reaction.body

    def test_lfc_reaction_body_sub(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph, target_lfc=True)
        sub_sub = next(r for r in system.reactors if r.name == "SubscriberNode_DefaultGroup")
        sub_reaction = next(r for r in sub_sub.reactions if r.kind == "sub")
        # The admitted subscription dispatches from its drain.
        assert "ros2_lf_corr_drain" in sub_reaction.body
        assert "/sensor_data" in sub_reaction.body

    def test_lfc_sets_effect_port(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph, target_lfc=True)
        pub_sub = next(r for r in system.reactors if r.name == "PublisherNode_DefaultGroup")
        timer_reaction = next(r for r in pub_sub.reactions if r.kind == "timer")
        assert "lf_set(" in timer_reaction.body

    def test_target_options_set_in_lfc_mode(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph, target_lfc=True)
        assert "keepalive" in system.target_options

    def test_preamble_set_in_lfc_mode(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph, target_lfc=True)
        assert "ros2_lf_binding" in system.preamble
        assert "stub_t" in system.preamble


# ---------------------------------------------------------------------------
# Generator: reaction-ID assignment
# ---------------------------------------------------------------------------

class TestReactionIdAssignment:
    def test_ids_are_sequential_from_zero(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph)
        all_rids = sorted(
            r.reaction_id
            for reactor in system.reactors
            for r in reactor.reactions
            if r.reaction_id >= 0
        )
        assert all_rids == list(range(len(all_rids)))

    def test_ids_are_deterministic(self):
        graph = _make_pubsub_graph()
        sys1 = generate_declared(graph)
        sys2 = generate_declared(graph)
        rids1 = sorted(r.reaction_id for reactor in sys1.reactors for r in reactor.reactions)
        rids2 = sorted(r.reaction_id for reactor in sys2.reactors for r in reactor.reactions)
        assert rids1 == rids2

    def test_timer_reaction_gets_id(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph)
        # Reactions live in the DefaultGroup sub-reactor
        pub_sub = next(r for r in system.reactors if r.name == "PublisherNode_DefaultGroup")
        assert pub_sub.reactions[0].reaction_id >= 0

    def test_total_rid_count(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph)
        # corr arrive/close reactions carry no RID.
        total = sum(1 for r in system.reactors for x in r.reactions if x.reaction_id >= 0)
        # pubsub has one timer cb + one sub cb = 2 reactions
        assert total == 2


# ---------------------------------------------------------------------------
# Generator: after_delay_ns default and per-link
# ---------------------------------------------------------------------------

class TestAfterDelayNs:
    """An admitted edge carries d on its correspondence edge."""

    def test_default_after_delay(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph)
        assert system.corr_edges[0].delay_ns == DEFAULT_AFTER_DELAY_NS

    def test_per_link_after_delay_from_topic_connection(self):
        graph = _make_pubsub_graph()
        graph.communication.topics[0].after_delay_ns = 2_000_000
        system = generate_declared(graph)
        assert system.corr_edges[0].delay_ns == 2_000_000

    def test_explicit_after_delay_ns_overrides_default(self):
        graph = _make_pubsub_graph()
        system = generate_declared(graph, after_delay_ns=10_000_000)
        assert system.corr_edges[0].delay_ns == 10_000_000

    def test_per_link_beats_explicit_default(self):
        graph = _make_pubsub_graph()
        graph.communication.topics[0].after_delay_ns = 3_000_000
        system = generate_declared(graph, after_delay_ns=10_000_000)
        # per-link wins
        assert system.corr_edges[0].delay_ns == 3_000_000


# ---------------------------------------------------------------------------
# Generator: physical actions for blocking service clients
# ---------------------------------------------------------------------------

class TestPhysicalActionsForBlockingClients:
    def _make_blocking_graph(self, call_style: str) -> SystemGraph:
        return SystemGraph(
            name="srv_bringup",
            launch_graph=None,
            nodes={
                "client_node": NodeInfo(
                    name="client_node",
                    class_name="ClientNode",
                    package="srv_example",
                    source_file="client.cpp",
                    timers=[TimerInfo(period_ms=1000.0, callback="timer_cb")],
                    service_clients=[
                        ServiceClientInfo(
                            service_name="/my_service",
                            service_type="example_interfaces::srv::AddTwoInts",
                            call_style=call_style,
                        )
                    ],
                    callbacks=[CallbackInfo(name="timer_cb")],
                ),
            },
            communication=SystemCommunication(),
        )

    def test_blocking_spin_generates_physical_action(self):
        graph = self._make_blocking_graph("blocking_spin")
        system = generate_declared(graph, target_lfc=True)
        reactor = next(r for r in system.reactors if r.name == "ClientNode")
        assert len(reactor.physical_actions) == 1
        assert "pa_" in reactor.physical_actions[0].name

    def test_blocking_future_generates_physical_action(self):
        graph = self._make_blocking_graph("blocking_future")
        system = generate_declared(graph, target_lfc=True)
        reactor = next(r for r in system.reactors if r.name == "ClientNode")
        assert len(reactor.physical_actions) == 1

    def test_async_callback_no_physical_action(self):
        graph = self._make_blocking_graph("async_callback")
        system = generate_declared(graph, target_lfc=True)
        reactor = next(r for r in system.reactors if r.name == "ClientNode")
        assert len(reactor.physical_actions) == 0


# ---------------------------------------------------------------------------
# Generator: timer offset_ms forwarded to LFTimer
# ---------------------------------------------------------------------------

class TestTimerOffsetMs:
    def test_nonzero_offset_forwarded(self):
        graph = SystemGraph(
            name="offset_bringup",
            launch_graph=None,
            nodes={
                "n": NodeInfo(
                    name="n",
                    class_name="Node",
                    package="pkg",
                    source_file="n.cpp",
                    timers=[TimerInfo(period_ms=100.0, callback="cb", offset_ms=50.0)],
                    callbacks=[CallbackInfo(name="cb")],
                )
            },
            communication=SystemCommunication(),
        )
        system = generate_declared(graph)
        # Timer lives in the DefaultGroup sub-reactor
        sub = next(r for r in system.reactors if r.name == "Node_DefaultGroup")
        assert sub.timers[0].offset == "50 msec"

    def test_zero_offset_emitted_as_zero(self):
        graph = SystemGraph(
            name="offset_bringup",
            launch_graph=None,
            nodes={
                "n": NodeInfo(
                    name="n",
                    class_name="Node",
                    package="pkg",
                    source_file="n.cpp",
                    timers=[TimerInfo(period_ms=100.0, callback="cb", offset_ms=0.0)],
                    callbacks=[CallbackInfo(name="cb")],
                )
            },
            communication=SystemCommunication(),
        )
        system = generate_declared(graph)
        sub = next(r for r in system.reactors if r.name == "Node_DefaultGroup")
        assert sub.timers[0].offset == "0"


# ---------------------------------------------------------------------------
# Binding header emitter
# ---------------------------------------------------------------------------

class TestBindingHeaderEmitter:
    def test_empty_table(self):
        from colcon_lf.generation.binding_header_emitter import emit_reaction_ids_header
        text = emit_reaction_ids_header([])
        assert "#define RID_COUNT 0" in text
        assert "#pragma once" in text

    def test_defines_emitted(self):
        from colcon_lf.generation.binding_header_emitter import emit_reaction_ids_header
        table = [
            (0, "PublisherNode", "timer_callback", "timer", ""),
            (1, "SubscriberNode", "topic_callback", "sub", "/sensor_data"),
        ]
        text = emit_reaction_ids_header(table)
        assert "#define RID_PublisherNode_timer_callback 0" in text
        assert "#define RID_SubscriberNode_topic_callback 1" in text
        assert "#define RID_COUNT 2" in text

    def test_topic_table_emitted(self):
        from colcon_lf.generation.binding_header_emitter import emit_reaction_ids_header
        table = [
            (0, "PublisherNode", "timer_callback", "timer", ""),
            (1, "SubscriberNode", "topic_callback", "sub", "/sensor_data"),
        ]
        text = emit_reaction_ids_header(table)
        assert "#ifdef ROS2_LF_DEFINE_TABLES" in text
        assert '"/sensor_data"' in text
