# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for chain resolution and gated-hop detection."""

from __future__ import annotations

import pytest

from lf_eval.model.chain import build_chain, resolve_chains
from lf_eval.model.system import parse_system

GATED_SYSTEM = {
    "name": "gated",
    "nodes": {
        "Sensor": {
            "name": "Sensor",
            "class_name": "Sensor",
            "timers": [{"period_ms": 100}],
            "publishers": [{"topic": "/raw"}],
            "subscribers": [],
        },
        "Planner": {
            "name": "Planner",
            "class_name": "Planner",
            "timers": [{"period_ms": 100}],
            "publishers": [{"topic": "/cmd"}],
            "subscribers": [{"topic": "/raw"}],
        },
        "Actuator": {
            "name": "Actuator",
            "class_name": "Actuator",
            "timers": [],
            "publishers": [],
            "subscribers": [{"topic": "/cmd"}],
        },
    },
    "communication": {
        "topics": [
            {"topic": "/raw", "publishers": ["Sensor"], "subscribers": ["Planner"]},
            {"topic": "/cmd", "publishers": ["Planner"], "subscribers": ["Actuator"]},
        ]
    },
}


def test_derived_chain_from_topic_graph(pipeline_system):
    chains = resolve_chains(pipeline_system)
    assert len(chains) == 1
    chain = chains[0]
    assert chain.nodes == ("NodeA", "NodeB", "NodeC")
    assert chain.source == "derived"
    assert chain.label == "NodeA->NodeC"


def test_hops_are_consecutive_pairs(pipeline_chain):
    assert [h.label for h in pipeline_chain.hops] == ["NodeA->NodeB", "NodeB->NodeC"]


def test_explicit_spec_overrides_derivation(pipeline_system):
    chains = resolve_chains(pipeline_system, ["NodeA->NodeC"])
    assert chains[0].nodes == ("NodeA", "NodeC")
    assert chains[0].source == "spec"


def test_multiple_specs_get_distinct_names(pipeline_system):
    chains = resolve_chains(pipeline_system, ["NodeA->NodeB", "NodeB->NodeC"])
    assert [c.name for c in chains] == ["chain1", "chain2"]


def test_spec_without_system_is_an_error():
    with pytest.raises(ValueError, match="requires --system"):
        resolve_chains(None, ["A->B"])


def test_no_system_yields_no_chain():
    assert resolve_chains(None) == []


def test_single_node_system_yields_no_chain():
    model = parse_system(
        {"name": "s", "nodes": {"A": {"name": "A", "class_name": "A"}}, "communication": {}}
    )
    assert resolve_chains(model) == []


def test_gated_hop_is_flagged_on_the_upstream_node():
    """A hop whose upstream re-times data on its own tick must be marked gated."""
    model = parse_system(GATED_SYSTEM)
    assert model.timer_gated_nodes() == {"Planner"}
    chain = resolve_chains(model)[0]
    assert chain.nodes == ("Sensor", "Planner", "Actuator")
    by_label = {h.label: h.gated for h in chain.hops}
    # Sensor is a pure source (timer, no subs) => its hop is not gated.
    assert by_label["Sensor->Planner"] is False
    # Planner is timer-gated => data leaving it waits for its tick.
    assert by_label["Planner->Actuator"] is True


def test_ungated_hops_filters_gated():
    model = parse_system(GATED_SYSTEM)
    chain = resolve_chains(model)[0]
    assert [h.label for h in chain.ungated_hops] == ["Sensor->Planner"]


def test_build_chain_without_system_marks_nothing_gated():
    chain = build_chain(["A", "B", "C"])
    assert all(not h.gated for h in chain.hops)
    assert chain.nodes == ("A", "B", "C")


def test_chain_label_for_empty_nodes():
    chain = build_chain([], name="empty")
    assert chain.label == "empty"
    assert chain.hops == ()
