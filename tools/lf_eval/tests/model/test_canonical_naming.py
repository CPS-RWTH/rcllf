# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Regression tests for node-identity canonicalisation.

``system.json`` carries three identifiers per node (``key``, ``name``,
``class_name``) and the trace carries a fourth string
(``rcl_node_init.node_name``). Which ``system.json`` field the trace name matches
**depends on the system**, so no single field can be used for chain pairing:

===================  ==============  ==============  ==============  ==================
system               ``key``         ``name``        ``class_name``  trace node_name
===================  ==============  ==============  ==============  ==================
pubsub_example       publisher_node  publisher_node  PublisherNode   ``publisher_node``
reference-system     MPCController   mpccontroller   MPCController   ``MPCController``
===================  ==============  ==============  ==============  ==================

The pubsub trace matches ``name`` (differs from ``class_name`` entirely); the
refsys trace matches ``key``/``class_name`` (differs from ``name`` only in case).

Before the fix, chains resolved everything to ``class_name`` while timelines were
keyed by the raw trace name, so on pubsub_example ``e2e_latency`` paired nothing
and reported "unavailable" — and even an explicit
``--chain publisher_node->subscriber_node`` could not work around it because
``chain_from_spec`` normalised through ``class_name``.

Both shapes are covered here so neither regresses.
"""

from __future__ import annotations

import pytest

from lf_eval.metrics.e2e_latency import compute_e2e, compute_hops
from lf_eval.metrics.registry import get_metric
from lf_eval.model.chain import resolve_chains
from lf_eval.model.run import make_run
from lf_eval.model.system import canonical_key, parse_system
from lf_eval.model.timeline import build_timeline

from tests.conftest import MS, EventBuilder

# ── system.json fixtures reproducing each real-world naming shape ─────────

# pubsub_example shape: name == trace name, class_name differs.
PUBSUB_SHAPE = {
    "name": "bringup",
    "nodes": {
        "publisher_node": {
            "name": "publisher_node",
            "class_name": "PublisherNode",
            "timers": [{"period_ms": 500.0}],
            "publishers": [{"topic": "sensor_data", "qos_depth": 10}],
            "subscribers": [],
        },
        "subscriber_node": {
            "name": "subscriber_node",
            "class_name": "SubscriberNode",
            "timers": [],
            "publishers": [],
            "subscribers": [{"topic": "sensor_data"}],
        },
    },
    "communication": {
        "topics": [
            {
                "topic": "/sensor_data",
                "publishers": ["publisher_node"],
                "subscribers": ["subscriber_node"],
            }
        ]
    },
}

# reference-system shape: name is a LOWERCASED class name; trace reports the
# CamelCase class name.
REFSYS_SHAPE = {
    "name": "refsys",
    "nodes": {
        "FrontLidarDriver": {
            "name": "frontlidardriver",
            "class_name": "FrontLidarDriver",
            "timers": [{"period_ms": 100.0}],
            "publishers": [{"topic": "/FrontLidarDriver"}],
            "subscribers": [],
        },
        "MPCController": {
            "name": "mpccontroller",
            "class_name": "MPCController",
            "timers": [],
            "publishers": [{"topic": "/MPCController"}],
            "subscribers": [{"topic": "/FrontLidarDriver"}],
        },
        "VehicleInterface": {
            "name": "vehicleinterface",
            "class_name": "VehicleInterface",
            "timers": [],
            "publishers": [],
            "subscribers": [{"topic": "/MPCController"}],
        },
    },
    "communication": {
        "topics": [
            {
                "topic": "/FrontLidarDriver",
                "publishers": ["frontlidardriver"],
                "subscribers": ["mpccontroller"],
            },
            {
                "topic": "/MPCController",
                "publishers": ["mpccontroller"],
                "subscribers": ["vehicleinterface"],
            },
        ]
    },
}


def _pipeline_trace(node_names: list[str], topics: list[str], n: int = 20):
    """Build a trace whose rcl_node_init names are exactly ``node_names``.

    ``node_names[0]`` owns the timer; each subsequent node subscribes to the
    preceding topic. Timings give a 2 ms hop and (len-1)*... known E2E.
    """
    b = EventBuilder()
    for name in node_names:
        b.add_node(name)
    b.add_timer(node_names[0], 100.0)
    for i, topic in enumerate(topics):
        b.add_publisher(node_names[i], topic)
        b.add_subscription(node_names[i + 1], topic)

    base = 1000 * MS
    for k in range(n):
        t0 = base + k * 100 * MS
        b.timer_fire(node_names[0], t0, MS)
        cursor = t0 + MS
        for i, topic in enumerate(topics):
            b.publish(node_names[i], topic, cursor)
            b.take(node_names[i + 1], topic, cursor + 2 * MS)
            b.sub_fire(node_names[i + 1], topic, cursor + 2 * MS, MS)
            cursor = cursor + 3 * MS
    return b


# ── canonical_key ────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "a,b",
    [
        ("publisher_node", "PublisherNode"),
        ("mpccontroller", "MPCController"),
        ("/publisher_node", "publisher_node"),
        ("front-lidar-driver", "FrontLidarDriver"),
        ("VehicleInterface", "vehicleinterface"),
    ],
)
def test_canonical_key_collapses_naming_conventions(a, b):
    assert canonical_key(a) == canonical_key(b)


def test_canonical_key_keeps_distinct_nodes_distinct():
    assert canonical_key("publisher_node") != canonical_key("subscriber_node")


# ── pubsub shape: name != class_name ─────────────────────────────────────


def test_pubsub_shape_resolves_every_identifier():
    model = parse_system(PUBSUB_SHAPE)
    for ident in ("publisher_node", "PublisherNode", "/publisher_node", "publishernode"):
        node = model.resolve_node(ident)
        assert node is not None, ident
        assert node.canonical_id == "publisher_node"


def test_pubsub_shape_chain_uses_canonical_ids_and_class_display():
    model = parse_system(PUBSUB_SHAPE)
    chain = resolve_chains(model)[0]
    # Canonical ids key the timeline...
    assert chain.nodes == ("publisher_node", "subscriber_node")
    # ...while display names stay class-based for humans.
    assert chain.display_nodes == ("PublisherNode", "SubscriberNode")
    assert chain.label == "PublisherNode->SubscriberNode"


def test_pubsub_trace_names_pair_against_derived_chain():
    """THE regression: trace says publisher_node, chain derived from class_name."""
    model = parse_system(PUBSUB_SHAPE)
    chain = resolve_chains(model)[0]
    builder = _pipeline_trace(["publisher_node", "subscriber_node"], ["/sensor_data"])
    timeline = build_timeline(builder.build(), resolver=model.canonical_of)

    # Timeline is keyed canonically, so it lines up with chain.nodes.
    assert timeline.nodes == ["publisher_node", "subscriber_node"]

    stats = compute_e2e(timeline, chain)
    assert stats is not None, "e2e must pair (this returned None before the fix)"
    assert stats.count == 20
    # timer start t0 -> sub end t0+4ms
    assert stats.mean_us == pytest.approx(4000.0)

    hops = dict((h.label, s) for h, s in compute_hops(timeline, chain))
    assert hops["PublisherNode->SubscriberNode"].mean_us == pytest.approx(2000.0)


def test_pubsub_metric_is_available_end_to_end():
    model = parse_system(PUBSUB_SHAPE)
    chain = resolve_chains(model)[0]
    builder = _pipeline_trace(["publisher_node", "subscriber_node"], ["/sensor_data"])
    timeline = build_timeline(builder.build(), resolver=model.canonical_of)
    run = make_run("pubsub", timeline, mode="lfc")

    result = get_metric("e2e_latency").compute([run], system=model, chains=[chain])
    assert result.available is True, result.note
    assert result.summary_rows[0]["n"] == 20
    assert result.summary_rows[0]["chain"] == "PublisherNode->SubscriberNode"


def test_pubsub_ordering_checks_more_than_zero_pairs():
    """determinism reported 0/0 N/A before the fix."""
    model = parse_system(PUBSUB_SHAPE)
    chain = resolve_chains(model)[0]
    builder = _pipeline_trace(["publisher_node", "subscriber_node"], ["/sensor_data"])
    timeline = build_timeline(builder.build(), resolver=model.canonical_of)
    run = make_run("pubsub", timeline, mode="lfc")

    result = get_metric("determinism").compute([run], system=model, chains=[chain])
    row = result.summary_rows[0]
    # the synthetic trace has no rmw identity: timestamp pairs give no PASS
    assert row["ordering"] == "INCOMPLETE"
    assert not row["violations"].startswith("0/0"), "no pairs were checked"
    assert row["violations"] == "0/20"


def test_pubsub_chain_spec_accepts_either_convention():
    """--chain must work whether written snake_case or CamelCase."""
    model = parse_system(PUBSUB_SHAPE)
    snake = resolve_chains(model, ["publisher_node->subscriber_node"])[0]
    camel = resolve_chains(model, ["PublisherNode->SubscriberNode"])[0]
    assert snake.nodes == camel.nodes == ("publisher_node", "subscriber_node")

    # And the spec-built chain must actually pair against the trace.
    builder = _pipeline_trace(["publisher_node", "subscriber_node"], ["/sensor_data"])
    timeline = build_timeline(builder.build(), resolver=model.canonical_of)
    assert compute_e2e(timeline, snake) is not None


def test_pubsub_timer_periods_keyed_canonically():
    model = parse_system(PUBSUB_SHAPE)
    assert model.timer_periods() == {"publisher_node": 500.0}


# ── refsys shape: name is lowercased, trace uses class_name ──────────────


def test_refsys_shape_resolves_lowercased_name_and_class():
    model = parse_system(REFSYS_SHAPE)
    for ident in ("MPCController", "mpccontroller", "MPCCONTROLLER"):
        node = model.resolve_node(ident)
        assert node is not None, ident
        assert node.canonical_id == "MPCController"


def test_refsys_shape_chain_keeps_camelcase_display():
    model = parse_system(REFSYS_SHAPE)
    chain = resolve_chains(model)[0]
    assert chain.nodes == ("FrontLidarDriver", "MPCController", "VehicleInterface")
    # Display must NOT degrade to the lowercased `name` field.
    assert chain.display_nodes == (
        "FrontLidarDriver",
        "MPCController",
        "VehicleInterface",
    )


def test_refsys_trace_names_pair_against_derived_chain():
    """Guards against 'fixing' pubsub by switching chains to the `name` field."""
    model = parse_system(REFSYS_SHAPE)
    chain = resolve_chains(model)[0]
    builder = _pipeline_trace(
        ["FrontLidarDriver", "MPCController", "VehicleInterface"],
        ["/FrontLidarDriver", "/MPCController"],
    )
    timeline = build_timeline(builder.build(), resolver=model.canonical_of)
    assert timeline.nodes == ["FrontLidarDriver", "MPCController", "VehicleInterface"]

    stats = compute_e2e(timeline, chain)
    assert stats is not None
    assert stats.count == 20
    # t0 -> hop1 sub end (t0+4ms) -> hop2 sub end (t0+7ms)
    assert stats.mean_us == pytest.approx(7000.0)


def test_refsys_timer_periods_keyed_canonically():
    model = parse_system(REFSYS_SHAPE)
    assert model.timer_periods() == {"FrontLidarDriver": 100.0}


# ── resolver behaviour: filtering, fallback, absent model ────────────────


def test_unmodelled_node_keeps_its_literal_trace_name():
    """A node absent from system.json must still appear, under its trace name."""
    model = parse_system(PUBSUB_SHAPE)
    b = EventBuilder()
    b.add_node("publisher_node")
    b.add_node("some_unmodelled_node")
    b.add_timer("publisher_node", 500.0)
    b.add_timer("some_unmodelled_node", 500.0)
    b.timer_fire("publisher_node", 1000 * MS, MS)
    b.timer_fire("some_unmodelled_node", 2000 * MS, MS)

    timeline = build_timeline(b.build(), resolver=model.canonical_of)
    assert "publisher_node" in timeline.nodes
    assert "some_unmodelled_node" in timeline.nodes


def test_no_resolver_keeps_literal_trace_names():
    b = EventBuilder()
    b.add_node("publisher_node")
    b.add_timer("publisher_node", 500.0)
    b.timer_fire("publisher_node", 1000 * MS, MS)
    timeline = build_timeline(b.build())
    assert timeline.nodes == ["publisher_node"]


def test_node_filter_matches_across_conventions():
    """The system.json-derived filter must not drop nodes on a case mismatch."""
    b = EventBuilder()
    b.add_node("MPCController")
    b.add_node("rosbag2_recorder")
    b.add_timer("MPCController", 100.0)
    b.add_timer("rosbag2_recorder", 100.0)
    b.timer_fire("MPCController", 1000 * MS, MS)
    b.timer_fire("rosbag2_recorder", 2000 * MS, MS)

    # Filter written in the lowercased `name` convention; trace is CamelCase.
    timeline = build_timeline(b.build(), node_filter={"mpccontroller"})
    assert timeline.nodes == ["MPCController"]


def test_trace_names_are_retained_for_diagnostics():
    model = parse_system(PUBSUB_SHAPE)
    builder = _pipeline_trace(["publisher_node", "subscriber_node"], ["/sensor_data"])
    timeline = build_timeline(builder.build(), resolver=model.canonical_of)
    assert timeline.trace_names["publisher_node"] == "publisher_node"


def test_display_of_falls_back_to_unknown_id():
    model = parse_system(PUBSUB_SHAPE)
    assert model.display_of("not_a_node") == "not_a_node"


def test_canonical_of_returns_none_for_unknown():
    model = parse_system(PUBSUB_SHAPE)
    assert model.canonical_of("ghost") is None
    assert model.canonical_of("PublisherNode") == "publisher_node"
