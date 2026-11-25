# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for lf_emitter module."""

from __future__ import annotations

import pytest

from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_model import (
    LFConnection,
    LFPort,
    LFReaction,
    LFReactor,
    LFReactorInstance,
    LFSystem,
    LFTimer,
)


def _make_pubsub_system() -> LFSystem:
    """Build the expected LF model for pubsub_example."""
    pub_reactor = LFReactor(
        name="PublisherNode",
        ports=[LFPort(name="sensor_data", type="SensorData", direction="output")],
        timers=[LFTimer(name="t_timer_callback", offset="0", period="500 msec")],
        reactions=[
            LFReaction(
                triggers=["t_timer_callback"],
                effects=["sensor_data"],
                body='lf_print("timer_callback triggered");',
            ),
        ],
    )
    sub_reactor = LFReactor(
        name="SubscriberNode",
        ports=[LFPort(name="sensor_data", type="SensorData", direction="input")],
        reactions=[
            LFReaction(
                triggers=["sensor_data"],
                effects=[],
                body='lf_print("topic_callback triggered");',
            ),
        ],
    )
    return LFSystem(
        name="Bringup",
        target="CCpp",
        reactors=[pub_reactor, sub_reactor],
        main_instances=[
            LFReactorInstance(name="publisher_node", reactor_class="PublisherNode"),
            LFReactorInstance(name="subscriber_node", reactor_class="SubscriberNode"),
        ],
        main_connections=[
            LFConnection(
                source="publisher_node.sensor_data",
                destination="subscriber_node.sensor_data",
                after_delay_ns=1_000_000,
            ),
        ],
    )


class TestEmitPubSubSystem:
    """Test emitting the pubsub_example system."""

    @pytest.fixture()
    def lf_text(self):
        return emit_lf_system(_make_pubsub_system())

    def test_target_declaration(self, lf_text):
        assert "target CCpp" in lf_text

    def test_publisher_reactor(self, lf_text):
        assert "reactor PublisherNode {" in lf_text

    def test_subscriber_reactor(self, lf_text):
        assert "reactor SubscriberNode {" in lf_text

    def test_main_reactor(self, lf_text):
        assert "main reactor Bringup {" in lf_text

    def test_output_port(self, lf_text):
        assert "output sensor_data: SensorData" in lf_text

    def test_input_port(self, lf_text):
        assert "input sensor_data: SensorData" in lf_text

    def test_timer(self, lf_text):
        assert "timer t_timer_callback(0, 500 msec)" in lf_text

    def test_reaction_trigger(self, lf_text):
        assert "reaction(t_timer_callback) -> sensor_data {=" in lf_text

    def test_reaction_body(self, lf_text):
        assert 'lf_print("timer_callback triggered");' in lf_text

    def test_subscriber_reaction(self, lf_text):
        assert "reaction(sensor_data) {=" in lf_text

    def test_instance(self, lf_text):
        assert "publisher_node = new PublisherNode()" in lf_text

    def test_connection_with_delay(self, lf_text):
        assert (
            "publisher_node.sensor_data -> subscriber_node.sensor_data after 1 msec"
            in lf_text
        )

    def test_closing_braces(self, lf_text):
        # Each reactor + main = 3 closing braces
        assert lf_text.count("\n}") >= 3

    def test_no_double_main(self, lf_text):
        # Only one main reactor
        assert lf_text.count("main reactor") == 1


class TestEmitEmptySystem:
    """Test emitting a system with no nodes."""

    def test_empty(self):
        system = LFSystem(name="Empty", target="CCpp")
        text = emit_lf_system(system)
        assert "target CCpp" in text
        assert "main reactor Empty {" in text


class TestEmitConnectionWithoutDelay:
    """Test internal connections (no after delay)."""

    def test_no_delay(self):
        system = LFSystem(
            name="Test",
            target="CCpp",
            main_connections=[
                LFConnection(source="a.x", destination="b.x"),
            ],
        )
        text = emit_lf_system(system)
        assert "a.x -> b.x" in text
        assert "after" not in text


class TestEmitNestedReactor:
    """Test emitting a reactor with nested instances and port forwarding."""

    def test_nested_instances(self):
        reactor = LFReactor(
            name="OuterNode",
            ports=[
                LFPort(name="data_in", type="String", direction="input"),
                LFPort(name="data_out", type="String", direction="output"),
            ],
            instances=[
                LFReactorInstance(name="grp", reactor_class="OuterNode_Group"),
            ],
            connections=[
                LFConnection(source="data_in", destination="grp.data_in"),
                LFConnection(source="grp.data_out", destination="data_out"),
            ],
        )
        system = LFSystem(
            name="Test",
            target="CCpp",
            reactors=[reactor],
        )
        text = emit_lf_system(system)
        assert "grp = new OuterNode_Group()" in text
        assert "data_in -> grp.data_in" in text
        assert "grp.data_out -> data_out" in text


class TestEmitMultipleEffects:
    """Test reaction with multiple effects."""

    def test_multi_effects(self):
        reactor = LFReactor(
            name="MultiPub",
            ports=[
                LFPort(name="out_a", type="A", direction="output"),
                LFPort(name="out_b", type="B", direction="output"),
            ],
            timers=[LFTimer(name="t_cb", offset="0", period="100 msec")],
            reactions=[
                LFReaction(
                    triggers=["t_cb"],
                    effects=["out_a", "out_b"],
                    body='lf_print("cb triggered");',
                ),
            ],
        )
        system = LFSystem(name="Test", target="CCpp", reactors=[reactor])
        text = emit_lf_system(system)
        assert "reaction(t_cb) -> out_a, out_b {=" in text
