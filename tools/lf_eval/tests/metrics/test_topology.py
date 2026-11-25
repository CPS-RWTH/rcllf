# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the topology (soundness/topology-diff) metric.

Exercises :func:`collect_runtime_topology` and :func:`diff_topology` directly
against synthetic events/models (mirroring ``tests/metrics/test_drops.py``),
plus the :class:`TopologyMetric` wrapper's graceful-degradation paths.
"""

from __future__ import annotations

from lf_eval.metrics.registry import get_metric
from lf_eval.metrics.topology import (
    DEFAULT_EXCLUDED_TOPICS,
    RuntimeTopology,
    TopologyMetric,
    collect_runtime_topology,
    diff_topology,
)
from lf_eval.model.run import make_run
from lf_eval.model.system import parse_system
from lf_eval.model.timeline import build_timeline

from tests.conftest import PIPELINE_SYSTEM_JSON, EventBuilder


def _runtime_from(builder: EventBuilder) -> RuntimeTopology:
    return collect_runtime_topology(builder.build())


# ── collect_runtime_topology ─────────────────────────────────────────────


def test_collects_publisher_and_subscriber_edges():
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_node("NodeB")
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    runtime = _runtime_from(b)
    assert runtime.publishes == {"NodeA": {"/a_out"}}
    assert runtime.subscribes == {"NodeB": {"/a_out"}}


def test_multiple_topics_per_node_accumulate():
    b = EventBuilder()
    b.add_node("N")
    b.add_publisher("N", "/one")
    b.add_publisher("N", "/two")
    runtime = _runtime_from(b)
    assert runtime.publishes == {"N": {"/one", "/two"}}


def test_empty_stream_yields_empty_topology():
    runtime = collect_runtime_topology([])
    assert runtime.is_empty
    assert runtime.nodes == set()


def test_init_event_counts_even_without_activity():
    """rcl_publisher_init/rcl_subscription_init fire once regardless of traffic."""
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_node("NodeB")
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    # No .publish()/.take()/.sub_fire() calls at all — pure init.
    runtime = _runtime_from(b)
    assert runtime.publishes == {"NodeA": {"/a_out"}}
    assert runtime.subscribes == {"NodeB": {"/a_out"}}


def test_unresolvable_node_handle_kept_not_dropped():
    from lf_eval.reader.events import TraceEvent

    events = [
        TraceEvent(
            "rcl_publisher_init",
            1000,
            {"publisher_handle": 1, "node_handle": 0xDEAD, "topic_name": "/x"},
        )
    ]
    runtime = collect_runtime_topology(events)
    assert len(runtime.publishes) == 1
    (node,) = runtime.publishes
    assert "unresolved-handle" in node
    assert runtime.publishes[node] == {"/x"}


def test_empty_topic_name_is_ignored():
    from lf_eval.reader.events import TraceEvent

    events = [
        TraceEvent(
            "rcl_node_init", 900, {"node_handle": 1, "node_name": "N", "namespace": "/"}
        ),
        TraceEvent(
            "rcl_publisher_init",
            1000,
            {"publisher_handle": 1, "node_handle": 1, "topic_name": ""},
        ),
    ]
    runtime = collect_runtime_topology(events)
    assert runtime.is_empty


# ── diff_topology: missing_from_model (runtime edge absent from static model) ──


def test_runtime_publish_missing_from_model_is_flagged(pipeline_system):
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/a_out")
    b.add_publisher("NodeA", "/surprise_topic")  # not declared for NodeA
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, pipeline_system)
    findings = {(f["node"], f["topic"], f["direction"]) for f in diff.missing_from_model}
    assert ("NodeA", "/surprise_topic", "publish") in findings
    assert ("NodeA", "/a_out", "publish") not in findings


def test_runtime_subscribe_missing_from_model_is_flagged(pipeline_system):
    b = EventBuilder()
    b.add_node("NodeC")
    b.add_subscription("NodeC", "/b_out")
    b.add_subscription("NodeC", "/unexpected")  # not declared for NodeC
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, pipeline_system)
    findings = {(f["node"], f["topic"], f["direction"]) for f in diff.missing_from_model}
    assert ("NodeC", "/unexpected", "subscribe") in findings
    assert ("NodeC", "/b_out", "subscribe") not in findings


# ── diff_topology: unobserved_in_runtime (static edge never traced) ──────


def test_static_edge_never_observed_is_flagged(pipeline_system):
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/a_out")
    # NodeB/NodeC never appear in the trace at all.
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, pipeline_system)
    findings = {(f["node"], f["topic"], f["direction"]) for f in diff.unobserved_in_runtime}
    assert ("NodeB", "/a_out", "subscribe") in findings
    assert ("NodeB", "/b_out", "publish") in findings
    assert ("NodeC", "/b_out", "subscribe") in findings
    # The one edge that *was* observed must not also appear as unobserved.
    assert ("NodeA", "/a_out", "publish") not in findings


def test_fully_matching_trace_has_no_findings(pipeline_system):
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_node("NodeB")
    b.add_node("NodeC")
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, pipeline_system)
    assert diff.missing_from_model == []
    assert diff.unobserved_in_runtime == []
    assert diff.unmatched_nodes == []


# ── reconciliation: canonical_id / aliases, unmatched nodes ──────────────


_ALIAS_SYSTEM_JSON = {
    "name": "refsys-like",
    "schema_version": 3,
    "nodes": {
        "MPCController": {
            "name": "mpccontroller",
            "class_name": "MPCController",
            "timers": [],
            "publishers": [{"topic": "/control"}],
            "subscribers": [],
        }
    },
    "communication": {"topics": [], "services": [], "actions": []},
}


def test_alias_reconciliation_matches_differently_cased_trace_name():
    """Trace reports the class-name spelling; system.json key differs only in case."""
    system = parse_system(_ALIAS_SYSTEM_JSON)
    b = EventBuilder()
    b.add_node("MPCController")
    b.add_publisher("MPCController", "/control")
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, system)
    assert diff.missing_from_model == []
    assert diff.unmatched_nodes == []


def test_unmatched_runtime_node_is_reported(pipeline_system):
    b = EventBuilder()
    b.add_node("GhostNode")
    b.add_publisher("GhostNode", "/ghost_topic")
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, pipeline_system)
    assert "GhostNode" in diff.unmatched_nodes
    # An unmatched node's edges cannot be compared to a declared set, but they
    # are still surfaced as missing_from_model (no declared set == absent).
    findings = {(f["node"], f["topic"]) for f in diff.missing_from_model}
    assert ("GhostNode", "/ghost_topic") in findings


# ── slash tolerance ───────────────────────────────────────────────────────


def test_topic_matching_is_slash_tolerant():
    data = {
        "name": "s",
        "schema_version": 3,
        "nodes": {
            "N": {
                "name": "N",
                "class_name": "N",
                "timers": [],
                "publishers": [{"topic": "a/b"}],  # no leading slash declared
                "subscribers": [],
            }
        },
        "communication": {"topics": [], "services": [], "actions": []},
    }
    system = parse_system(data)
    b = EventBuilder()
    b.add_node("N")
    b.add_publisher("N", "/a/b")  # trace always reports the resolved, slashed form
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, system)
    assert diff.missing_from_model == []
    assert diff.unobserved_in_runtime == []


# ── infrastructure topic exclusion ───────────────────────────────────────


def test_infra_topics_excluded_from_missing_by_default(pipeline_system):
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/a_out")
    for topic in DEFAULT_EXCLUDED_TOPICS:
        b.add_publisher("NodeA", topic)
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, pipeline_system)
    flagged_topics = {f["topic"] for f in diff.missing_from_model}
    assert flagged_topics.isdisjoint(DEFAULT_EXCLUDED_TOPICS)


def test_infra_topics_can_be_included_via_option(pipeline_system):
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/rosout")
    runtime = _runtime_from(b)
    diff = diff_topology(runtime, pipeline_system, excluded_topics=frozenset())
    flagged_topics = {f["topic"] for f in diff.missing_from_model}
    assert "/rosout" in flagged_topics


def test_declared_infra_topic_still_compared_normally():
    """An infra topic that IS declared in the model is not specially excluded."""
    data = {
        "name": "s",
        "schema_version": 3,
        "nodes": {
            "N": {
                "name": "N",
                "class_name": "N",
                "timers": [],
                "publishers": [{"topic": "/rosout"}],
                "subscribers": [],
            }
        },
        "communication": {"topics": [], "services": [], "actions": []},
    }
    system = parse_system(data)
    # Runtime never publishes /rosout at all -> should show as unobserved,
    # exclusion only suppresses missing_from_model noise, not this direction.
    runtime = RuntimeTopology()
    diff = diff_topology(runtime, system)
    findings = {(f["node"], f["topic"]) for f in diff.unobserved_in_runtime}
    assert ("N", "/rosout") in findings


# ── TopologyMetric: system=None / empty-trace / fail_on_missing_edge ─────


def test_metric_notes_when_system_is_none():
    run = make_run("r1", build_timeline([]), mode="lfc")
    result = get_metric("topology").compute([run], system=None)
    assert result.available is False
    assert "system" in result.note.lower()


def test_metric_notes_on_synthetic_run_with_no_tracing_dir(pipeline_system):
    """make_run-built runs have no real tracing/ dir; the runtime side degrades
    to a note rather than raising, matching the graceful-input contract every
    metric must have (see lf_eval/metrics/registry.py's module docstring)."""
    run = make_run("no_disk_trace", build_timeline([]), mode="lfc")
    result = get_metric("topology").compute([run], system=pipeline_system)
    assert result.available is False
    assert "no_disk_trace" not in (result.note or "") or "runtime" in result.note.lower()


def test_metric_fail_on_missing_edge_sets_failed_flag(monkeypatch, pipeline_system):
    """fail_on_missing_edge plumbs through **options like other metric options
    (see cli.py's `options = {"include_raw": ...}` -> compute(**options))."""
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/a_out")
    b.add_publisher("NodeA", "/unexpected")
    runtime = _runtime_from(b)

    monkeypatch.setattr(
        TopologyMetric, "_runtime_topology_for", staticmethod(lambda run: runtime)
    )
    run = make_run("r1", build_timeline([]), mode="lfc")

    result = get_metric("topology").compute(
        [run], system=pipeline_system, fail_on_missing_edge=True
    )
    assert result.available
    assert result.data.get("failed") is True
    assert "fail_on_missing_edge" in result.note


def test_metric_fail_on_missing_edge_false_by_default(monkeypatch, pipeline_system):
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/a_out")
    b.add_publisher("NodeA", "/unexpected")
    runtime = _runtime_from(b)

    monkeypatch.setattr(
        TopologyMetric, "_runtime_topology_for", staticmethod(lambda run: runtime)
    )
    run = make_run("r1", build_timeline([]), mode="lfc")

    result = get_metric("topology").compute([run], system=pipeline_system)
    assert result.available
    assert "failed" not in result.data


def test_metric_reports_summary_rows_via_injected_runtime(monkeypatch, pipeline_system):
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_node("NodeB")
    b.add_node("NodeC")
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")
    runtime = _runtime_from(b)

    monkeypatch.setattr(
        TopologyMetric, "_runtime_topology_for", staticmethod(lambda run: runtime)
    )
    run = make_run("clean_run", build_timeline([]), mode="lfc")

    result = get_metric("topology").compute([run], system=pipeline_system)
    assert result.available
    row = result.summary_rows[0]
    assert row["run"] == "clean_run"
    assert row["missing_from_model"] == 0
    assert row["unobserved_in_runtime"] == 0
    assert row["unmatched_nodes"] == 0


def test_metric_unknown_schema_version_notes_not_crashes(monkeypatch):
    data = dict(PIPELINE_SYSTEM_JSON)
    data["schema_version"] = 99
    system = parse_system(data)

    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/a_out")
    runtime = _runtime_from(b)

    monkeypatch.setattr(
        TopologyMetric, "_runtime_topology_for", staticmethod(lambda run: runtime)
    )
    run = make_run("r1", build_timeline([]), mode="lfc")

    result = get_metric("topology").compute([run], system=system)
    assert result.available
    assert "schema_version" in (result.note or "")
    assert "99" in (result.note or "")


def test_registered_in_metric_registry():
    from lf_eval.metrics import metric_names

    assert "topology" in metric_names()


# ── derived infrastructure edges ─────────────────────────────
#
# Actions and lifecycle nodes create topics that no `publishers`/`subscribers`
# list in system.json mentions, but which ARE derivable from data the model
# already carries. Deriving them (rather than blanket-excluding them) keeps a
# genuinely missing action/lifecycle edge reportable. Shapes below mirror the
# real Phase-7 soundness traces.


ACTION_SYSTEM_JSON = {
    "name": "action_example",
    "schema_version": 3,
    "nodes": {
        "/action_example/action_server": {
            "name": "action_server",
            "class_name": "ActionServerNode",
            "timers": [],
            "publishers": [],
            "subscribers": [],
        },
        "/action_example/action_client": {
            "name": "action_client",
            "class_name": "ActionClientNode",
            "timers": [],
            "publishers": [],
            "subscribers": [],
        },
    },
    "communication": {
        "topics": [],
        "services": [],
        "actions": [
            {
                "action": "/action_example/analyze",
                "action_type": "Analyze",
                "server": "action_server",
                "clients": ["action_client"],
            }
        ],
    },
}


LIFECYCLE_SYSTEM_JSON = {
    "name": "lifecycle_example",
    "schema_version": 3,
    "nodes": {
        "lifecycle_node": {
            "name": "lifecycle_node",
            "class_name": "LifecycleExampleNode",
            "base_class": "rclcpp_lifecycle::LifecycleNode",
            "lifecycle_callbacks": ["on_configure"],
            "timers": [],
            "publishers": [{"topic": "lifecycle_output"}],
            "subscribers": [],
        }
    },
    "communication": {"topics": [], "services": [], "actions": []},
}


def test_action_status_and_feedback_are_declared_for_server():
    """Server publishes <action>/_action/{status,feedback} — derived, not flagged."""
    system = parse_system(ACTION_SYSTEM_JSON)
    b = EventBuilder()
    b.add_node("action_server")
    b.add_publisher("action_server", "/action_example/analyze/_action/status")
    b.add_publisher("action_server", "/action_example/analyze/_action/feedback")
    diff = diff_topology(_runtime_from(b), system)
    assert diff.missing_from_model == []


def test_action_status_and_feedback_are_declared_for_client():
    """Client subscribes to the same two topics."""
    system = parse_system(ACTION_SYSTEM_JSON)
    b = EventBuilder()
    b.add_node("action_client")
    b.add_subscription("action_client", "/action_example/analyze/_action/status")
    b.add_subscription("action_client", "/action_example/analyze/_action/feedback")
    diff = diff_topology(_runtime_from(b), system)
    assert diff.missing_from_model == []


def test_action_goal_result_cancel_are_not_derived_as_topics():
    """goal/result/cancel are ROS *services*, so they must stay reportable.

    Deriving them as topics would suppress a real finding if one ever showed
    up on the topic side.
    """
    system = parse_system(ACTION_SYSTEM_JSON)
    b = EventBuilder()
    b.add_node("action_server")
    b.add_publisher("action_server", "/action_example/analyze/_action/goal")
    diff = diff_topology(_runtime_from(b), system)
    flagged = {f["topic"] for f in diff.missing_from_model}
    assert "/action_example/analyze/_action/goal" in flagged


def test_action_edge_on_unrelated_node_still_flagged():
    """Derivation is per-node: only the declared server/clients get the edges."""
    system = parse_system(ACTION_SYSTEM_JSON)
    b = EventBuilder()
    b.add_node("action_client")
    b.add_publisher("action_client", "/action_example/analyze/_action/status")
    diff = diff_topology(_runtime_from(b), system)
    flagged = {(f["topic"], f["direction"]) for f in diff.missing_from_model}
    # client subscribes to status; publishing it is not a derived edge
    assert ("/action_example/analyze/_action/status", "publish") in flagged


def test_lifecycle_transition_event_is_declared():
    """A lifecycle node's <node>/transition_event publisher is derived."""
    system = parse_system(LIFECYCLE_SYSTEM_JSON)
    b = EventBuilder()
    b.add_node("lifecycle_node")
    b.add_publisher("lifecycle_node", "/lifecycle_node/transition_event")
    diff = diff_topology(_runtime_from(b), system)
    assert diff.missing_from_model == []


def test_non_lifecycle_node_transition_event_still_flagged(pipeline_system):
    """Derivation is gated on the node actually being a lifecycle node."""
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/NodeA/transition_event")
    diff = diff_topology(_runtime_from(b), pipeline_system)
    flagged = {f["topic"] for f in diff.missing_from_model}
    assert "/NodeA/transition_event" in flagged


def test_launch_orchestrator_node_is_excluded():
    """`ros2 launch`'s own lifecycle-manager node is not an application node."""
    system = parse_system(LIFECYCLE_SYSTEM_JSON)
    b = EventBuilder()
    b.add_node("launch_ros_3479202")
    b.add_subscription("launch_ros_3479202", "/lifecycle_node/transition_event")
    diff = diff_topology(_runtime_from(b), system)
    assert diff.missing_from_model == []
    assert diff.unmatched_nodes == []


def test_message_filters_edges_remain_reported():
    """RCLLF-H003 subscriptions are true positives and must NOT be suppressed."""
    data = {
        "name": "qos_hazard_example",
        "schema_version": 3,
        "nodes": {
            "message_filters_subscriber_node": {
                "name": "message_filters_subscriber_node",
                "class_name": "MessageFiltersSubscriberNode",
                "timers": [],
                "publishers": [],
                "subscribers": [],
            }
        },
        "communication": {"topics": [], "services": [], "actions": []},
    }
    system = parse_system(data)
    b = EventBuilder()
    b.add_node("message_filters_subscriber_node")
    b.add_subscription("message_filters_subscriber_node", "/sync_topic_a")
    b.add_subscription("message_filters_subscriber_node", "/sync_topic_b")
    diff = diff_topology(_runtime_from(b), system)
    flagged = {f["topic"] for f in diff.missing_from_model}
    assert flagged == {"/sync_topic_a", "/sync_topic_b"}


_ORDER_PROBE = """
import json
from lf_eval.metrics.topology import RuntimeTopology, diff_topology
from lf_eval.model.system import parse_system
system = parse_system({"schema_version": 3, "nodes": {"n": {"name": "n", "class_name": "N",
    "timers": [], "publishers": [], "subscribers": []}},
    "communication": {"topics": [], "services": [], "actions": []}})
runtime = RuntimeTopology(publishes={"n": {"/t%d" % i for i in range(8)}})
print(json.dumps([f["topic"] for f in diff_topology(runtime, system).missing_from_model]))
"""


def test_missing_from_model_order_does_not_depend_on_the_hash_seed():
    """Two processes over the same runs write the same metrics.json.

    ``ros2 lf eval`` and ``lf-eval report`` on the same R3 runs listed
    ``/clock`` and ``/lf_clock`` in opposite orders: the runtime topic sets were
    walked in hash order, which ``PYTHONHASHSEED`` changes per process.
    """
    import json
    import os
    import subprocess
    import sys
    from pathlib import Path

    root = str(Path(__file__).resolve().parents[2])
    orders = set()
    for seed in range(6):
        env = dict(os.environ, PYTHONHASHSEED=str(seed),
                   PYTHONPATH=os.pathsep.join(filter(None, [root, os.environ.get("PYTHONPATH")])))
        out = subprocess.run([sys.executable, "-c", _ORDER_PROBE], env=env, check=True,
                             capture_output=True, text=True).stdout
        orders.add(tuple(json.loads(out)))
    assert len(orders) == 1, f"missing_from_model order depends on the hash seed: {sorted(orders)}"
    assert list(next(iter(orders))) == [f"/t{i}" for i in range(8)]
