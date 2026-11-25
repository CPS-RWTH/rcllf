# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for ExecConfig.load / from_dict: schema validation, slash tolerance."""

from __future__ import annotations

import textwrap

import pytest

from colcon_lf.exec_config.model import ExecConfig, topic_runtime_name

from .conftest import make_graph, make_node


class TestSchemaVersion:
    def test_missing_schema_version_defaults_ok(self):
        graph = make_graph({"n": make_node("n")})
        config = ExecConfig.from_dict({}, graph, source="test")
        assert config.schema_version == 1

    def test_bad_schema_version_raises(self):
        graph = make_graph({"n": make_node("n")})
        with pytest.raises(ValueError, match="test: unsupported schema_version"):
            ExecConfig.from_dict({"schema_version": 99}, graph, source="test")


class TestTopLevelKeys:
    def test_unknown_top_level_key_raises(self):
        graph = make_graph({"n": make_node("n")})
        with pytest.raises(ValueError, match="unknown top-level key"):
            ExecConfig.from_dict({"bogus": 1}, graph, source="test")

    def test_empty_dict_is_valid(self):
        graph = make_graph({"n": make_node("n")})
        config = ExecConfig.from_dict({}, graph, source="test")
        assert config.defaults == {}
        assert config.nodes == {}
        assert config.subscriptions == {}
        assert config.acknowledged_hazards == []


class TestPolicyKeyValidation:
    def test_unknown_policy_key_raises(self):
        graph = make_graph({"n": make_node("n")})
        raw = {"defaults": {"bogus_key": "x"}}
        with pytest.raises(ValueError, match="unknown policy key 'bogus_key'"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_bad_take_mode_value_raises(self):
        graph = make_graph({"n": make_node("n")})
        raw = {"defaults": {"take_mode": "sometimes"}}
        with pytest.raises(ValueError, match="must be one of"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_bad_take_timeout_value_raises(self):
        graph = make_graph({"n": make_node("n")})
        raw = {"defaults": {"take_timeout_ms": -5}}
        with pytest.raises(ValueError, match="must be an int > 0"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_valid_defaults_scope(self):
        graph = make_graph({"n": make_node("n")})
        raw = {"defaults": {"take_mode": "block", "take_timeout_ms": 250}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        assert config.defaults == {"take_mode": "block", "take_timeout_ms": 250}


class TestUnknownNodesAndTopics:
    def test_unknown_node_in_nodes_scope_raises(self):
        graph = make_graph({"n": make_node("n")})
        raw = {"nodes": {"/nonexistent": {"take_mode": "block"}}}
        with pytest.raises(ValueError, match="unknown node '/nonexistent'"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_unknown_node_in_subscriptions_scope_raises(self):
        graph = make_graph({"n": make_node("n", subscriptions=["t"])})
        raw = {"subscriptions": {"/nonexistent": {"t": {"take_mode": "block"}}}}
        with pytest.raises(ValueError, match="unknown node '/nonexistent'"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_unknown_topic_raises(self):
        graph = make_graph({"n": make_node("n", subscriptions=["t"])})
        raw = {"subscriptions": {"n": {"nonexistent_topic": {"take_mode": "block"}}}}
        with pytest.raises(ValueError, match="unknown topic 'nonexistent_topic'"):
            ExecConfig.from_dict(raw, graph, source="test")


class TestSlashTolerance:
    def test_node_key_with_leading_slash_accepted(self):
        graph = make_graph({"robot_1/subscriber_node": make_node(
            "subscriber_node", class_name="SubscriberNode", subscriptions=["sensor_data"]
        )})
        # Rename node key to have a leading slash to mimic system.json form.
        node_info = graph.nodes.pop("robot_1/subscriber_node")
        graph.nodes["/robot_1/subscriber_node"] = node_info

        raw = {"nodes": {"robot_1/subscriber_node": {"take_timeout_ms": 250}}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        # Normalized to the model's canonical (slashed) form.
        assert config.nodes == {"/robot_1/subscriber_node": {"take_timeout_ms": 250}}

    def test_node_key_without_leading_slash_matches_bare_model_key(self):
        graph = make_graph({"sim_timer": make_node("sim_timer", class_name="TimerNode")})
        raw = {"nodes": {"/sim_timer": {"take_timeout_ms": 250}}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        assert config.nodes == {"sim_timer": {"take_timeout_ms": 250}}

    def test_topic_key_with_leading_slash_accepted(self):
        graph = make_graph({"n": make_node("n", subscriptions=["sensor_data"])})
        raw = {"subscriptions": {"n": {"/sensor_data": {"take_mode": "block"}}}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        assert config.subscriptions == {"n": {"sensor_data": {"take_mode": "block"}}}


class TestRuntimeNameSpellings:
    """Keys resolve by runtime name; two spellings of one are refused."""

    def test_runtime_name_rules(self):
        # rcl_expand_topic_name: absolute as written, relative against the
        # node's namespace, '~' against the node's full name.
        assert topic_runtime_name("scan", "controller") == "/scan"
        assert topic_runtime_name("/scan", "/robot/controller") == "/scan"
        assert topic_runtime_name("scan", "/robot/controller") == "/robot/scan"
        assert topic_runtime_name("scan", "robot/controller") == "/robot/scan"
        assert topic_runtime_name("~/scan", "robot/controller") == "/robot/controller/scan"
        assert topic_runtime_name("~", "controller") == "/controller"

    def test_two_spellings_of_one_node_in_nodes_are_refused(self):
        graph = make_graph({"controller": make_node("controller")})
        raw = {"nodes": {"controller": {"take_timeout_ms": 250},
                         "/controller": {"take_mode": "block"}}}
        with pytest.raises(
            ValueError,
            match=r"nodes\.'/controller' and nodes\.'controller' both resolve to "
                  r"'/controller' at runtime",
        ):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_two_spellings_of_one_node_in_subscriptions_are_refused(self):
        graph = make_graph({"robot/n": make_node("n", subscriptions=["scan"])})
        raw = {"subscriptions": {"/robot/n": {"scan": {"take_timeout_ms": 250}},
                                 "robot/n": {"scan": {"take_mode": "block"}}}}
        with pytest.raises(
            ValueError,
            match=r"subscriptions\.'robot/n' and subscriptions\.'/robot/n' both resolve to "
                  r"'/robot/n' at runtime",
        ):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_two_spellings_of_one_topic_in_subscriptions_are_refused(self):
        graph = make_graph({"n": make_node("n", subscriptions=["sensor_data"])})
        raw = {"subscriptions": {"n": {"sensor_data": {"take_timeout_ms": 250},
                                       "/sensor_data": {"take_mode": "block"}}}}
        with pytest.raises(
            ValueError,
            match=r"subscriptions\.'n'\.'/sensor_data' and subscriptions\.'n'\."
                  r"'sensor_data' both resolve to '/sensor_data' at runtime",
        ):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_namespaced_relative_and_absolute_subscriptions_are_two_topics(self):
        # '/robot/n' subscribing to 'scan' (/robot/scan) and '/scan': two topics.
        graph = make_graph({"/robot/n": make_node("n", subscriptions=["scan", "/scan"])})
        raw = {"subscriptions": {"/robot/n": {"scan": {"take_timeout_ms": 250},
                                              "/scan": {"take_mode": "block"}}}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        assert config.subscriptions == {"/robot/n": {
            "scan": {"take_timeout_ms": 250}, "/scan": {"take_mode": "block"}}}

    def test_absolute_key_does_not_name_a_namespaced_relative_topic(self):
        # '/scan' is not the topic '/robot/n' reads as 'scan' (/robot/scan).
        graph = make_graph({"/robot/n": make_node("n", subscriptions=["scan"])})
        raw = {"subscriptions": {"/robot/n": {"/scan": {"take_mode": "block"}}}}
        with pytest.raises(
            ValueError,
            match=r"unknown topic '/scan' on node '/robot/n' \(it resolves to '/scan'; "
                  r"not a subscription in system.json, which subscribes to "
                  r"'scan' -> '/robot/scan'\)",
        ):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_one_node_in_nodes_and_subscriptions_is_fine(self):
        # The refusal is per section: nodes: and subscriptions: both name a node.
        graph = make_graph({"n": make_node("n", subscriptions=["scan"])})
        raw = {"nodes": {"n": {"take_timeout_ms": 250}},
               "subscriptions": {"/n": {"scan": {"take_mode": "block"}}}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        assert config.nodes == {"n": {"take_timeout_ms": 250}}
        assert config.subscriptions == {"n": {"scan": {"take_mode": "block"}}}


class TestAcknowledgedHazards:
    def test_valid_list(self):
        graph = make_graph({"n": make_node("n")})
        raw = {"acknowledged_hazards": ["RCLLF-H001"]}
        config = ExecConfig.from_dict(raw, graph, source="test")
        assert config.acknowledged_hazards == ["RCLLF-H001"]

    def test_non_list_raises(self):
        graph = make_graph({"n": make_node("n")})
        raw = {"acknowledged_hazards": "RCLLF-H001"}
        with pytest.raises(ValueError, match="acknowledged_hazards"):
            ExecConfig.from_dict(raw, graph, source="test")


class TestErrorPrefixing:
    def test_error_prefixed_with_source(self):
        graph = make_graph({"n": make_node("n")})
        with pytest.raises(ValueError, match=r"^my_source\.yaml: "):
            ExecConfig.from_dict({"bogus": 1}, graph, source="my_source.yaml")


class TestLoadFromFile:
    def test_load_reads_yaml_file(self, tmp_path):
        graph = make_graph({"n": make_node("n")})
        yaml_file = tmp_path / "execution.yaml"
        yaml_file.write_text(textwrap.dedent("""\
            schema_version: 1
            defaults:
              take_mode: block
        """), encoding="utf-8")
        config = ExecConfig.load(yaml_file, graph)
        assert config.defaults == {"take_mode": "block"}
        assert config.source == str(yaml_file)

    def test_empty_returns_defaults(self):
        config = ExecConfig.empty()
        assert config.schema_version == 1
        assert config.defaults == {}


class TestTimerPeriodOverride:
    """``nodes.<node>.timers.<timer>.period`` (maintainer decision P3)."""

    def test_valid_override_parses(self):
        graph = make_graph({"n": make_node("n", timers=["timer_callback"])})
        raw = {"nodes": {"n": {"timers": {"timer_callback": {"period": "5 msec"}}}}}
        config = ExecConfig.from_dict(raw, graph, source="test")
        assert config.nodes["n"]["timers"] == {"timer_callback": {"period": "5 msec"}}

    def test_unknown_timer_name_raises(self):
        graph = make_graph({"n": make_node("n", timers=["timer_callback"])})
        raw = {"nodes": {"n": {"timers": {"nonexistent": {"period": "5 msec"}}}}}
        with pytest.raises(ValueError, match="no timer named 'nonexistent' on 'n'"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_missing_period_key_raises(self):
        graph = make_graph({"n": make_node("n", timers=["timer_callback"])})
        raw = {"nodes": {"n": {"timers": {"timer_callback": {}}}}}
        with pytest.raises(ValueError, match="needs 'offset' or 'period'"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_empty_period_literal_raises(self):
        graph = make_graph({"n": make_node("n", timers=["timer_callback"])})
        raw = {"nodes": {"n": {"timers": {"timer_callback": {"period": ""}}}}}
        with pytest.raises(ValueError, match="must not be empty"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_non_time_literal_raises(self):
        graph = make_graph({"n": make_node("n", timers=["timer_callback"])})
        raw = {"nodes": {"n": {"timers": {"timer_callback": {"period": True}}}}}
        with pytest.raises(ValueError, match="must be a time literal"):
            ExecConfig.from_dict(raw, graph, source="test")

    def test_timers_alongside_other_node_policy_keys(self):
        graph = make_graph({"n": make_node("n", timers=["timer_callback"])})
        raw = {
            "nodes": {
                "n": {
                    "take_mode": "block",
                    "timers": {"timer_callback": {"period": "10 msec"}},
                }
            }
        }
        config = ExecConfig.from_dict(raw, graph, source="test")
        assert config.nodes["n"]["take_mode"] == "block"
        assert config.nodes["n"]["timers"] == {"timer_callback": {"period": "10 msec"}}
