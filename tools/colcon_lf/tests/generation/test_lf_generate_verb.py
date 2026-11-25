# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Integration test: generate LF code from the real pubsub_example system.json."""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from ..corr_declared import generate_declared
from colcon_lf.output.serializer import deserialize_system_graph

# Committed fixture, not a build artifact. This previously pointed at
# `output/analysis/pubsub_example_bringup/system.json`, which does not exist —
# so every test in this file silently SKIPPED and never caught the dead
# --after-delay parameter it explicitly asserts against. The real
# analysis output lives under `packages/lf-analysis/`, which is gitignored, so
# a committed copy is the only form that also runs in CI.
_PUBSUB_SYSTEM_JSON = (
    Path(__file__).resolve().parents[1] / "fixtures" / "pubsub_system.json"
)


@pytest.fixture(scope="module")
def lf_text() -> str:
    """Load the committed pubsub system.json, generate, and emit."""
    assert _PUBSUB_SYSTEM_JSON.is_file(), (
        f"committed fixture missing: {_PUBSUB_SYSTEM_JSON} — this test must never "
        f"silently skip again"
    )
    raw = _PUBSUB_SYSTEM_JSON.read_text()
    graph = deserialize_system_graph(raw)
    system = generate_declared(graph)
    return emit_lf_system(system)


class TestPubsubExampleIntegration:
    """End-to-end test with the real pubsub_example extraction output."""

    def test_target(self, lf_text):
        assert "target CCpp" in lf_text

    def test_publisher_reactor_exists(self, lf_text):
        assert "reactor PublisherNode {" in lf_text

    def test_subscriber_reactor_exists(self, lf_text):
        assert "reactor SubscriberNode {" in lf_text

    def test_main_reactor(self, lf_text):
        assert "main reactor Bringup {" in lf_text

    def test_timer_period(self, lf_text):
        assert "500 msec" in lf_text

    def test_output_port(self, lf_text):
        assert "output corr_e0: rcllf_corr_t" in lf_text

    def test_input_port(self, lf_text):
        assert "input corr_e0: rcllf_corr_t" in lf_text

    def test_connection(self, lf_text):
        assert "publisher_node.corr_e0 -> subscriber_node.corr_e0\n" in lf_text

    def test_connection_uses_library_default_when_unset(self, lf_text):
        """No after_delay passed here, so the library's last-resort 0 applies.

        The "1 msec" default belongs to the CLI (`--after-delay` /
        `execution.yaml`), not to `generate_lf_system`; this fixture calls the
        generator directly. See test_after_delay.py for the CLI-layer defaults.
        """
        # The admitted edge's delay is in the drain wake, the
        # corr link has no after at all.
        assert " after " not in lf_text

    def test_instances(self, lf_text):
        assert "publisher_node = new PublisherNode()" in lf_text
        assert "subscriber_node = new SubscriberNode()" in lf_text

    def test_reaction_stubs(self, lf_text):
        assert "ros2_lf_invoke_" in lf_text

    def test_no_double_main(self, lf_text):
        assert lf_text.count("main reactor") == 1

    def test_printer_node_count(self, lf_text):
        # One top-level reactor per node, plus one callback-group sub-reactor
        # each, plus 1 main. (This expectation read "== 2" while the whole class
        # was silently skipping; callback-group sub-reactors post-date it.)
        import re
        non_main = re.findall(r"^reactor \w+ \{", lf_text, re.MULTILINE)
        main = re.findall(r"^main reactor \w+ \{", lf_text, re.MULTILINE)
        assert sorted(non_main) == [
            "reactor PublisherNode {",
            "reactor PublisherNode_DefaultGroup {",
            "reactor SubscriberNode {",
            "reactor SubscriberNode_DefaultGroup {",
        ]
        assert len(main) == 1
