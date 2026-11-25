# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Phase 1 extractor tests: QoS fields, timer offset, service client call style, schema version."""

from __future__ import annotations

import json
import warnings

import pytest

from colcon_lf.comm_analysis.models import TopicConnection
from colcon_lf.graph.models import SCHEMA_VERSION, SystemGraph
from colcon_lf.node_analysis.models import (
    PublisherInfo,
    ServiceClientInfo,
    SubscriberInfo,
    TimerInfo,
)
from colcon_lf.output.serializer import deserialize_system_graph, serialize


# ---------------------------------------------------------------------------
# Model field tests
# ---------------------------------------------------------------------------

class TestTimerInfo:
    def test_has_offset_ms_default(self):
        t = TimerInfo(period_ms=500.0, callback="timer_callback")
        assert t.offset_ms == 0.0

    def test_offset_ms_settable(self):
        t = TimerInfo(period_ms=500.0, callback="cb", offset_ms=10.0)
        assert t.offset_ms == 10.0


class TestPublisherInfo:
    def test_qos_fields_default_none(self):
        p = PublisherInfo(topic="/foo", message_type="std_msgs::msg::String")
        assert p.qos_reliability is None
        assert p.qos_durability is None
        assert p.qos_history is None
        assert p.qos_deadline_ms is None

    def test_qos_fields_settable(self):
        p = PublisherInfo(
            topic="/foo", message_type="std_msgs::msg::String",
            qos_reliability="reliable",
            qos_durability="volatile",
            qos_history="keep_last",
            qos_deadline_ms=100.0,
        )
        assert p.qos_reliability == "reliable"
        assert p.qos_durability == "volatile"
        assert p.qos_history == "keep_last"
        assert p.qos_deadline_ms == 100.0


class TestSubscriberInfo:
    def test_qos_fields_default_none(self):
        s = SubscriberInfo(topic="/foo", message_type="std_msgs::msg::String", callback="cb")
        assert s.qos_reliability is None
        assert s.qos_durability is None
        assert s.qos_history is None
        assert s.qos_deadline_ms is None


class TestServiceClientInfo:
    def test_call_style_default_none(self):
        c = ServiceClientInfo(service_name="/srv", service_type="MyType")
        assert c.call_style is None

    def test_call_style_values(self):
        for style in ("async_callback", "blocking_spin", "blocking_future", None):
            c = ServiceClientInfo(service_name="/s", service_type="T", call_style=style)
            assert c.call_style == style


class TestTopicConnection:
    def test_after_delay_ns_default_none(self):
        tc = TopicConnection(topic="/t", message_type="T", publishers=[], subscribers=[])
        assert tc.after_delay_ns is None

    def test_after_delay_ns_settable(self):
        tc = TopicConnection(
            topic="/t", message_type="T", publishers=[], subscribers=[],
            after_delay_ns=5_000_000,
        )
        assert tc.after_delay_ns == 5_000_000


# ---------------------------------------------------------------------------
# Schema version tests
# ---------------------------------------------------------------------------

class TestSchemaVersion:
    def test_constant_is_3(self):
        # v2 -> v3: added `diagnostics` (hazard detection).
        assert SCHEMA_VERSION == 3

    def test_system_graph_has_schema_version(self):
        from colcon_lf.launch_analysis.models import LaunchGraph, LaunchFileInfo
        sg = SystemGraph(
            name="test",
            launch_graph=LaunchGraph(root=LaunchFileInfo(file_path="x.launch.py", package="pkg")),
        )
        assert sg.schema_version == SCHEMA_VERSION

    def test_serialize_includes_schema_version(self):
        from colcon_lf.launch_analysis.models import LaunchGraph, LaunchFileInfo
        sg = SystemGraph(
            name="test",
            launch_graph=LaunchGraph(root=LaunchFileInfo(file_path="x.launch.py", package="pkg")),
        )
        data = json.loads(serialize(sg))
        assert data["schema_version"] == SCHEMA_VERSION

    def test_deserialize_missing_version_warns(self):
        from colcon_lf.launch_analysis.models import LaunchGraph, LaunchFileInfo
        sg = SystemGraph(
            name="test",
            launch_graph=LaunchGraph(root=LaunchFileInfo(file_path="x.launch.py", package="pkg")),
        )
        data = json.loads(serialize(sg))
        del data["schema_version"]
        json_str = json.dumps(data)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            result = deserialize_system_graph(json_str)
            assert any("schema_version" in str(warning.message) for warning in w)
        assert result.name == "test"

    def test_deserialize_old_version_warns(self):
        from colcon_lf.launch_analysis.models import LaunchGraph, LaunchFileInfo
        sg = SystemGraph(
            name="test",
            launch_graph=LaunchGraph(root=LaunchFileInfo(file_path="x.launch.py", package="pkg")),
        )
        data = json.loads(serialize(sg))
        data["schema_version"] = 1
        json_str = json.dumps(data)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            result = deserialize_system_graph(json_str)
            assert any("schema_version" in str(warning.message) for warning in w)
        assert result.name == "test"

    def test_deserialize_newer_version_warns(self):
        """Newer-than-current schema_version warns."""
        from colcon_lf.launch_analysis.models import LaunchGraph, LaunchFileInfo
        sg = SystemGraph(
            name="test",
            launch_graph=LaunchGraph(root=LaunchFileInfo(file_path="x.launch.py", package="pkg")),
        )
        data = json.loads(serialize(sg))
        data["schema_version"] = SCHEMA_VERSION + 1
        json_str = json.dumps(data)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            result = deserialize_system_graph(json_str)
            assert any("schema_version" in str(warning.message) for warning in w)
        assert result.name == "test"

    def test_deserialize_current_version_no_warning(self):
        from colcon_lf.launch_analysis.models import LaunchGraph, LaunchFileInfo
        sg = SystemGraph(
            name="test",
            launch_graph=LaunchGraph(root=LaunchFileInfo(file_path="x.launch.py", package="pkg")),
        )
        json_str = serialize(sg)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            result = deserialize_system_graph(json_str)
            schema_warnings = [x for x in w if "schema_version" in str(x.message)]
            assert not schema_warnings
        assert result.name == "test"


# ---------------------------------------------------------------------------
# QoS extraction helper tests (token-based, no libclang needed)
# ---------------------------------------------------------------------------

class TestQosExtraction:
    """Tests for _extract_qos_attrs without libclang by mocking cursor token output."""

    def _make_mock_cursor(self, token_str: str, n_children: int = 3):
        """Create a minimal mock cursor whose children[2] returns given tokens."""
        class MockDesc:
            def __init__(self):
                from unittest.mock import MagicMock
                self.kind = MagicMock()
                self.kind.__eq__ = lambda s, o: False

        class MockChild:
            def __init__(self, tokens):
                self._tokens = tokens
            def get_children(self):
                return []
            def walk_preorder(self):
                return iter([MockDesc()])

        class MockCursor:
            def __init__(self, token_str, n_children):
                self._token_str = token_str
                self._n = n_children
            def get_children(self):
                if self._n < 3:
                    return []
                # children[0], [1] are placeholders; children[2] is the QoS arg
                class PlaceHolder:
                    def get_children(self): return []
                    def walk_preorder(self): return iter([])
                return [PlaceHolder(), PlaceHolder(), MockChild(self._token_str)]

        return MockCursor(token_str, n_children)

    def _extract(self, token_str: str):
        from colcon_lf.node_analysis.clang_parser import _extract_qos_attrs

        # Patch _cursor_tokens to return split tokens from the child
        import colcon_lf.node_analysis.clang_parser as m
        orig = m._cursor_tokens

        def patched(c):
            if hasattr(c, "_tokens"):
                return c._tokens.split()
            return []

        m._cursor_tokens = patched
        try:
            cursor = self._make_mock_cursor(token_str)
            return _extract_qos_attrs(cursor)
        finally:
            m._cursor_tokens = orig

    def test_reliable(self):
        r = self._extract("rclcpp :: QoS ( 10 ) . reliable ( )")
        assert r["reliability"] == "reliable"

    def test_best_effort(self):
        r = self._extract("rclcpp :: QoS ( 10 ) . best_effort ( )")
        assert r["reliability"] == "best_effort"

    def test_transient_local(self):
        r = self._extract("rclcpp :: QoS ( 10 ) . transient_local ( )")
        assert r["durability"] == "transient_local"

    def test_keep_all(self):
        r = self._extract("rclcpp :: QoS ( 10 ) . keep_all ( )")
        assert r["history"] == "keep_all"

    def test_no_qos_keywords(self):
        r = self._extract("10")
        assert r == {"reliability": None, "durability": None, "history": None, "deadline_ms": None}


class TestQosExtractionWordMatches:
    """Only chained calls, enum tokens and named profiles set QoS.

    Before the fix the token stream was joined and searched for substrings, so a
    string literal or an identifier that merely contained ``best_effort`` (or
    ``reliable``, ``transient_local``, ``keep_all`` ...) set the analyzed value.
    """

    _extract = TestQosExtraction._extract
    _make_mock_cursor = TestQosExtraction._make_mock_cursor

    def test_string_literal_does_not_set_reliability(self):
        """The issue's reproduction: ``runtime_qos("best_effort_topic")``."""
        r = self._extract('runtime_qos ( "best_effort_topic" )')
        assert r["reliability"] is None

    def test_identifier_containing_word_does_not_set_reliability(self):
        r = self._extract("make_qos ( best_effort_mode )")
        assert r["reliability"] is None

    def test_bare_variable_named_like_method_does_not_set_reliability(self):
        r = self._extract("qos_for ( reliable )")
        assert r["reliability"] is None

    def test_literals_do_not_set_durability_history_or_profile(self):
        r = self._extract('make_qos ( "transient_local_keep_all" , "SensorDataQoS" )')
        assert r == {"reliability": None, "durability": None, "history": None,
                     "deadline_ms": None}

    def test_identifier_containing_profile_name_does_not_match(self):
        r = self._extract("MySensorDataQoSWrapper ( )")
        assert r["reliability"] is None

    def test_arrow_chained_best_effort(self):
        r = self._extract("qos -> best_effort ( )")
        assert r["reliability"] == "best_effort"

    def test_durability_volatile_chain(self):
        r = self._extract("rclcpp :: QoS ( 10 ) . durability_volatile ( )")
        assert r["durability"] == "volatile"

    def test_rmw_enum_tokens(self):
        r = self._extract(
            "rclcpp :: QoS ( 10 ) . reliability ( RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT )"
            " . history ( RMW_QOS_POLICY_HISTORY_KEEP_ALL )"
            " . durability ( RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL )")
        assert r["reliability"] == "best_effort"
        assert r["history"] == "keep_all"
        assert r["durability"] == "transient_local"

    def test_named_profile_still_recognized(self):
        r = self._extract("rclcpp :: SensorDataQoS ( )")
        assert r["reliability"] == "best_effort"

    def test_chain_next_to_literal_still_counts(self):
        r = self._extract('runtime_qos ( "best_effort_topic" ) . reliable ( )')
        assert r["reliability"] == "reliable"
