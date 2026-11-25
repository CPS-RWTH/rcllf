# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for resolve.py: per-key precedence, class-conflict, JSON emission.

The per-key precedence test uses the 00_design.md section 2.2/2.4 example
verbatim: node scope sets only take_timeout_ms, subscription scope sets only
take_mode, and the resolved row demonstrates the merge
(take_mode from subscription, take_timeout_ms from node).
"""

from __future__ import annotations

import pytest

from colcon_lf.comm_analysis.models import TopicConnection
from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.exec_config.resolve import resolve

from .conftest import make_graph, make_node


class TestPerKeyPrecedence:
    """The frozen 00_design.md section 2.2/2.4 example, verbatim."""

    def test_design_doc_example(self):
        graph = make_graph({
            "/robot_1/subscriber_node": make_node(
                "subscriber_node",
                class_name="SubscriberNode",
                subscriptions=["sensor_data"],
            )
        })
        raw = {
            "schema_version": 1,
            "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
            "nodes": {"/robot_1/subscriber_node": {"take_timeout_ms": 250}},
            "subscriptions": {
                "/robot_1/subscriber_node": {"sensor_data": {"take_mode": "block"}}
            },
        }
        config = ExecConfig.from_dict(raw, graph, source="test")
        resolved = resolve(config, graph)

        assert resolved.defaults == {"take_mode": "skip", "take_timeout_ms": 100}
        assert len(resolved.subscriptions) == 1
        row = resolved.subscriptions[0]
        assert row.node_class == "SubscriberNode"
        assert row.topic == "sensor_data"
        # Per-key merge: take_mode from subscription scope, take_timeout_ms
        # from node scope (no higher scope sets that key) — the exact
        # disambiguating case from the design doc.
        assert row.values == {"take_mode": "block", "take_timeout_ms": 250}

    def test_json_emission_matches_frozen_shape(self):
        graph = make_graph({
            "/robot_1/subscriber_node": make_node(
                "subscriber_node",
                class_name="SubscriberNode",
                subscriptions=["sensor_data"],
            )
        })
        raw = {
            "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
            "nodes": {"/robot_1/subscriber_node": {"take_timeout_ms": 250}},
            "subscriptions": {
                "/robot_1/subscriber_node": {"sensor_data": {"take_mode": "block"}}
            },
        }
        config = ExecConfig.from_dict(raw, graph, source="test")
        resolved = resolve(config, graph)
        emitted = resolved.to_json_dict()

        assert emitted["schema_version"] == 1
        assert emitted["defaults"] == {"take_mode": "skip", "take_timeout_ms": 100}
        assert emitted["subscriptions"] == [
            {
                "node_class": "SubscriberNode",
                "topic": "sensor_data",
                "take_mode": "block",
                "take_timeout_ms": 250,
            }
        ]


class TestBuiltinDefaults:
    def test_no_config_uses_registry_defaults(self):
        graph = make_graph({"n": make_node("n", subscriptions=["t"])})
        config = ExecConfig.empty()
        resolved = resolve(config, graph)
        assert resolved.defaults == {"take_mode": "skip", "take_timeout_ms": 100}
        assert resolved.subscriptions[0].values == {
            "take_mode": "skip",
            "take_timeout_ms": 100,
        }


class TestPerInstanceFold:
    """The fold is node key -> binding key, i.e. 1:1 with node instances.

    Two
    instances of one class each get their own row, so per-instance policy is
    representable.  What used to be a "conflicting values" hard error is now the
    supported case; the retained ``ValueError`` is a synthesis-bug tripwire
    (see ``TestSynthesisBugAssertion``).
    """

    def _two_feature_nodes(self):
        return make_graph({
            "primary_node": make_node(
                "primary_node", class_name="FeatureNode", subscriptions=["feature_input"]
            ),
            "secondary_node": make_node(
                "secondary_node", class_name="FeatureNode", subscriptions=["feature_input"]
            ),
        })

    def test_identical_values_across_instances_yield_two_rows(self):
        graph = self._two_feature_nodes()
        raw = {"defaults": {"take_mode": "block"}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        resolved = resolve(config, graph)
        # One row per instance, each under its own binding key.
        assert len(resolved.subscriptions) == 2
        assert {row.node_class for row in resolved.subscriptions} == {
            "FeatureNode__primary_node", "FeatureNode__secondary_node",
        }
        for row in resolved.subscriptions:
            assert len(row.node_keys) == 1

    def test_differing_take_mode_across_instances_yields_two_rows(self):
        graph = self._two_feature_nodes()
        raw = {
            "nodes": {
                "primary_node": {"take_mode": "block"},
                "secondary_node": {"take_mode": "skip"},
            }
        }
        config = ExecConfig.from_dict(raw, graph, source="test")
        resolved = resolve(config, graph)
        assert len(resolved.subscriptions) == 2
        by_key = {row.node_class: row for row in resolved.subscriptions}
        assert by_key["FeatureNode__primary_node"].values["take_mode"] == "block"
        assert by_key["FeatureNode__secondary_node"].values["take_mode"] == "skip"

    def test_single_instance_class_keeps_plain_name(self):
        graph = make_graph({
            "lonely_node": make_node(
                "lonely_node", class_name="FeatureNode", subscriptions=["feature_input"]
            ),
        })
        resolved = resolve(ExecConfig.empty(), graph)
        assert [row.node_class for row in resolved.subscriptions] == ["FeatureNode"]


class TestSynthesisBugAssertion:
    """The retained ValueError fires only on a binding-key synthesis bug."""

    def test_duplicate_binding_key_raises(self, monkeypatch):
        graph = make_graph({
            "primary_node": make_node(
                "primary_node", class_name="FeatureNode", subscriptions=["feature_input"]
            ),
            "secondary_node": make_node(
                "secondary_node", class_name="FeatureNode", subscriptions=["feature_input"]
            ),
        })
        # Craft a broken synthesis: both instances fold to one key.
        monkeypatch.setattr(
            "colcon_lf.generation.binding_key.build_binding_keys",
            lambda nodes: {key: "FeatureNode" for key in nodes},
        )
        with pytest.raises(ValueError, match="synthesis bug"):
            resolve(ExecConfig.empty(), graph)


class TestResolveWarnings:
    def test_no_warning_when_timeout_set_and_mode_skip(self):
        """A skip take waits up to take_timeout_ms for a counted sample."""
        graph = make_graph(
            {"n": make_node("n", subscriptions=["t"])},
            topics=[TopicConnection(topic="t", message_type="std_msgs/msg/String",
                                    publishers=["p"], subscribers=["n"])],
        )
        raw = {"subscriptions": {"n": {"t": {"take_timeout_ms": 250}}}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        resolved = resolve(config, graph)
        assert resolved.warnings == [], (
            "resolve warned about a set take_timeout_ms under take_mode skip, but a skip "
            f"take waits up to the timeout for a counted sample: {resolved.warnings}")

    def test_warns_when_timeout_set_on_an_external_topic(self):
        """An external_topics message is taken by the ingress thread before the
        reaction runs, so its take never waits and the timeout is inert."""
        graph = make_graph(
            {"n": make_node("n", subscriptions=["t"])},
            topics=[TopicConnection(topic="t", message_type="std_msgs/msg/String",
                                    publishers=[], subscribers=["n"])],
        )
        raw = {"external_topics": ["t"],
               "subscriptions": {"n": {"t": {"take_timeout_ms": 250}}}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        resolved = resolve(config, graph)
        (warning,) = resolved.warnings
        # The row is keyed by the binding key (the class name), not the node key.
        assert warning.startswith("N.t: 'maxwait_ms' is set for an external topic"), warning
        assert "the timeout is inert" in warning
        # The default timeout on an external topic says nothing.
        config = ExecConfig.from_dict({"external_topics": ["t"]}, graph, source="test")
        assert resolve(config, graph).warnings == []

    def test_no_warning_when_timeout_default_and_mode_skip(self):
        graph = make_graph({"n": make_node("n", subscriptions=["t"])})
        config = ExecConfig.empty()
        resolved = resolve(config, graph)
        assert resolved.warnings == []

    def test_no_warning_when_timeout_exceeds_after_delay(self):
        # A take's cutoff is publish tag + d + T, so T and the
        # link's after delay d are independent budgets. Under the after 0
        # default every T exceeds d, and the no-loss cells set T = 3600 s on
        # purpose; the old "exceeds after_delay_ns" warning fired on all of them.
        graph = make_graph(
            {"n": make_node("n", subscriptions=["t"])},
            topics=[
                TopicConnection(
                    topic="t",
                    message_type="std_msgs/msg/String",
                    publishers=[],
                    subscribers=["n"],
                    after_delay_ns=0,
                )
            ],
        )
        raw = {
            "subscriptions": {
                "n": {"t": {"take_mode": "block", "take_timeout_ms": 3_600_000}}
            }
        }
        config = ExecConfig.from_dict(raw, graph, source="test")
        resolved = resolve(config, graph)
        assert resolved.warnings == [], resolved.warnings

    def test_no_warning_when_timeout_within_after_delay(self):
        graph = make_graph(
            {"n": make_node("n", subscriptions=["t"])},
            topics=[
                TopicConnection(
                    topic="t",
                    message_type="std_msgs/msg/String",
                    publishers=[],
                    subscribers=["n"],
                    after_delay_ns=500_000_000,  # 500ms
                )
            ],
        )
        raw = {
            "subscriptions": {
                "n": {"t": {"take_mode": "block", "take_timeout_ms": 100}}
            }
        }
        config = ExecConfig.from_dict(raw, graph, source="test")
        resolved = resolve(config, graph)
        assert resolved.warnings == []
