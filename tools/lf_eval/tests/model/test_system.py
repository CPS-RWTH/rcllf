# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the ``system.json`` topology model and chain derivation."""

from __future__ import annotations

import json

import pytest

from lf_eval.model.system import find_system_json, load_system, parse_system

from tests.conftest import PIPELINE_SYSTEM_JSON


def test_parses_nodes_and_timers(pipeline_system):
    assert set(pipeline_system.nodes) == {"NodeA", "NodeB", "NodeC"}
    assert pipeline_system.nodes["NodeA"].period_ms == pytest.approx(100.0)
    assert pipeline_system.nodes["NodeB"].period_ms is None


def test_timer_periods_maps_class_names(pipeline_system):
    assert pipeline_system.timer_periods() == {"NodeA": 100.0}


def test_zero_period_timers_are_ignored():
    data = {
        "name": "s",
        "nodes": {"N": {"name": "N", "class_name": "N", "timers": [{"period_ms": 0}]}},
        "communication": {"topics": []},
    }
    assert parse_system(data).nodes["N"].timers == ()


def test_resolve_node_is_case_insensitive(pipeline_system):
    assert pipeline_system.resolve_node("nodea").class_name == "NodeA"
    assert pipeline_system.resolve_node("NODEA").class_name == "NodeA"
    assert pipeline_system.resolve_node("/NodeA").class_name == "NodeA"
    assert pipeline_system.resolve_node("Nope") is None


def test_successors_builds_topic_dag(pipeline_system):
    edges = pipeline_system.successors()
    assert edges["NodeA"] == {"NodeB"}
    assert edges["NodeB"] == {"NodeC"}
    assert edges["NodeC"] == set()


def test_longest_chain_recovers_pipeline(pipeline_system):
    assert pipeline_system.longest_chain() == ["NodeA", "NodeB", "NodeC"]


def test_longest_chain_tolerates_cycles():
    data = {
        "name": "cyc",
        "nodes": {
            n: {"name": n, "class_name": n, "timers": [], "publishers": [], "subscribers": []}
            for n in ("A", "B")
        },
        "communication": {
            "topics": [
                {"topic": "/ab", "publishers": ["A"], "subscribers": ["B"]},
                {"topic": "/ba", "publishers": ["B"], "subscribers": ["A"]},
            ]
        },
    }
    chain = parse_system(data).longest_chain()
    assert len(chain) == 2
    assert set(chain) == {"A", "B"}


def test_longest_chain_empty_for_isolated_nodes():
    data = {
        "name": "iso",
        "nodes": {"A": {"name": "A", "class_name": "A"}},
        "communication": {"topics": []},
    }
    assert len(parse_system(data).longest_chain()) == 1


def test_chain_from_spec_arrow_syntax(pipeline_system):
    assert pipeline_system.chain_from_spec("NodeA->NodeC") == ["NodeA", "NodeC"]


def test_chain_from_spec_comma_and_case(pipeline_system):
    assert pipeline_system.chain_from_spec("nodea, nodeb") == ["NodeA", "NodeB"]


def test_chain_from_spec_rejects_unknown_node(pipeline_system):
    with pytest.raises(ValueError, match="unknown node"):
        pipeline_system.chain_from_spec("NodeA->Ghost")


def test_chain_from_spec_requires_two_nodes(pipeline_system):
    with pytest.raises(ValueError, match="at least two"):
        pipeline_system.chain_from_spec("NodeA")


def test_timer_gated_nodes_needs_timer_and_subscription():
    data = {
        "name": "g",
        "nodes": {
            # Timer + subscriptions => gated (re-times data on its own tick).
            "Fusion": {
                "name": "Fusion",
                "class_name": "Fusion",
                "timers": [{"period_ms": 100}],
                "subscribers": [{"topic": "/in"}],
                "publishers": [{"topic": "/out"}],
            },
            # Timer but no subscriptions => a pure source, not gated.
            "Source": {
                "name": "Source",
                "class_name": "Source",
                "timers": [{"period_ms": 100}],
                "subscribers": [],
                "publishers": [{"topic": "/in"}],
            },
            # Subscriptions but no timer => pass-through, not gated.
            "Sink": {
                "name": "Sink",
                "class_name": "Sink",
                "timers": [],
                "subscribers": [{"topic": "/out"}],
            },
        },
        "communication": {"topics": []},
    }
    assert parse_system(data).timer_gated_nodes() == {"Fusion"}


def test_load_system_reads_file(tmp_path):
    path = tmp_path / "system.json"
    path.write_text(json.dumps(PIPELINE_SYSTEM_JSON))
    model = load_system(path)
    assert model.name == "pipeline"
    assert model.source_path == path
    assert len(model.nodes) == 3


def test_find_system_json_returns_first_existing(tmp_path):
    real = tmp_path / "b.json"
    real.write_text("{}")
    assert find_system_json([tmp_path / "a.json", real]) == real
    assert find_system_json([tmp_path / "missing.json"]) is None


def test_missing_sections_do_not_crash():
    model = parse_system({})
    assert model.nodes == {}
    assert model.topics == ()
    assert model.longest_chain() == []


# ── real extracted models (skip when the gitignored outputs are absent) ──


def test_real_pubsub_system_json(pubsub_system_json):
    if not pubsub_system_json.is_file():
        pytest.skip("pubsub_example system.json not extracted on this host")
    model = load_system(pubsub_system_json)
    assert model.resolve_node("publisher_node") is not None
    # The 1-hop chain publisher -> subscriber must be derivable, as canonical
    # ids (= the system.json keys, which here equal the ROS2 node names the
    # trace reports — this is the pubsub-style divergence from class_name).
    assert model.longest_chain() == ["publisher_node", "subscriber_node"]
    assert model.timer_periods() == {"publisher_node": 500.0}
    # Display names stay class-based so output reads naturally.
    assert model.display_of("publisher_node") == "PublisherNode"


