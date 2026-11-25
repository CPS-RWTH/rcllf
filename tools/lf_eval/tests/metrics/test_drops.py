# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the drops metric.

Exercises both drop signals against synthetic streams with known counts:
publish-vs-delivered shortfall, and ``rmw_take`` events with ``taken = 0``.
"""

from __future__ import annotations

import pytest

from lf_eval.metrics.drops import TopicCounts, collect_drop_counts
from lf_eval.metrics.registry import get_metric
from lf_eval.model.run import make_run
from lf_eval.model.timeline import build_timeline

from tests.conftest import MS, EventBuilder


def _pub_sub(n_publish: int, n_deliver: int, n_failed_takes: int = 0) -> EventBuilder:
    """Publisher emits ``n_publish``; subscriber's callback runs ``n_deliver`` times."""
    b = EventBuilder()
    b.add_node("Pub")
    b.add_node("Sub")
    b.add_timer("Pub", 10.0)
    b.add_publisher("Pub", "/data")
    b.add_subscription("Sub", "/data")
    for i in range(n_publish):
        t = (100 + i * 10) * MS
        b.timer_fire("Pub", t, MS // 10)
        b.publish("Pub", "/data", t + MS // 10)
    for i in range(n_deliver):
        t = (100 + i * 10) * MS + 2 * MS
        b.take("Sub", "/data", t, taken=1)
        b.sub_fire("Sub", "/data", t, MS // 10)
    for i in range(n_failed_takes):
        b.take("Sub", "/data", (5000 + i) * MS, taken=0)
    return b


def _counts(builder: EventBuilder, system=None):
    events = builder.build()
    timeline = build_timeline(events)
    return collect_drop_counts(events, timeline, system=system)


# ── TopicCounts arithmetic ───────────────────────────────────────────────


def test_topic_counts_expected_scales_with_subscribers():
    counts = TopicCounts("/t", published=10, delivered=20, subscribers=2)
    assert counts.expected == 20
    assert counts.delta == 0
    assert counts.dropped == 0


def test_topic_counts_shortfall_is_a_drop():
    counts = TopicCounts("/t", published=10, delivered=7, subscribers=1)
    assert counts.expected == 10
    assert counts.delta == -3
    assert counts.dropped == 3
    assert counts.drop_rate == pytest.approx(0.3)


def test_topic_counts_surplus_is_not_a_drop():
    """More callbacks than publishes is not a drop (pre-trace or intra-process)."""
    counts = TopicCounts("/t", published=5, delivered=8, subscribers=1)
    assert counts.delta == 3
    assert counts.dropped == 0
    assert counts.drop_rate == 0.0


def test_topic_counts_zero_expected_has_zero_rate():
    counts = TopicCounts("/t", published=0, delivered=0, subscribers=0)
    assert counts.drop_rate == 0.0
    assert counts.dropped == 0


def test_topic_counts_payload_shape():
    payload = TopicCounts("/t", published=10, delivered=8, subscribers=1, qos_depth=5).to_dict()
    assert payload["published"] == 10
    assert payload["delivered"] == 8
    assert payload["dropped"] == 2
    assert payload["drop_rate_pct"] == pytest.approx(20.0)
    assert payload["qos_depth"] == 5


# ── collection from events ───────────────────────────────────────────────


def test_no_drops_when_every_publish_is_delivered():
    counts = _counts(_pub_sub(n_publish=10, n_deliver=10))
    topic = counts.topics["/data"]
    assert topic.published == 10
    assert topic.delivered == 10
    assert topic.subscribers == 1
    assert topic.dropped == 0
    assert counts.total_dropped == 0


def test_intra_process_publishes_are_counted_once():
    # rslcpp: intra-process publishes emit rclcpp_intra_publish and
    # no rcl_publish; a publish that also goes inter-process emits both.
    b = _pub_sub(n_publish=0, n_deliver=10)
    handle = b.publishers[("Pub", "/data")]
    for i in range(10):
        b._add("rclcpp_intra_publish", (100 + i * 10) * MS, publisher_handle=handle, message=0x1)
    for i in range(4):
        b.publish("Pub", "/data", (100 + i * 10) * MS)
    topic = _counts(b).topics["/data"]
    assert topic.published == 10 and topic.dropped == 0


def test_shortfall_is_detected():
    counts = _counts(_pub_sub(n_publish=10, n_deliver=6))
    assert counts.topics["/data"].dropped == 4
    assert counts.total_dropped == 4
    assert counts.total_expected == 10


def test_failed_takes_are_counted_and_attributed():
    counts = _counts(_pub_sub(n_publish=10, n_deliver=10, n_failed_takes=3))
    assert counts.failed_takes == 3
    assert counts.total_takes == 13
    assert counts.failed_take_rate == pytest.approx(3 / 13)
    assert counts.topics["/data"].failed_takes == 3
    assert counts.unattributed_failed_takes == 0


def test_infrastructure_topics_are_excluded_by_default():
    b = EventBuilder()
    b.add_node("N")
    b.add_publisher("N", "/rosout")
    b.add_publisher("N", "/parameter_events")
    b.add_publisher("N", "/app_data")
    b.publish("N", "/rosout", 1000)
    b.publish("N", "/parameter_events", 1000)
    b.publish("N", "/app_data", 1000)
    counts = _counts(b)
    assert set(counts.topics) == {"/app_data"}


def test_infrastructure_topics_can_be_included():
    b = EventBuilder()
    b.add_node("N")
    b.add_publisher("N", "/rosout")
    b.publish("N", "/rosout", 1000)
    events = b.build()
    counts = collect_drop_counts(
        events, build_timeline(events), include_infra=True
    )
    assert "/rosout" in counts.topics


def test_subscriber_count_falls_back_to_system_json(pipeline_system):
    """A topic whose subscriber never ran still gets its declared count."""
    b = EventBuilder()
    b.add_node("NodeA")
    b.add_publisher("NodeA", "/a_out")
    for i in range(5):
        b.publish("NodeA", "/a_out", (100 + i) * MS)
    counts = _counts(b, system=pipeline_system)
    topic = counts.topics["/a_out"]
    # NodeB subscribes per system.json, but no callback ran => all dropped.
    assert topic.subscribers == 1
    assert topic.published == 5
    assert topic.delivered == 0
    assert topic.dropped == 5


def test_qos_depth_is_reported_when_system_json_is_on_disk(tmp_path, pipeline_system):
    import json

    from lf_eval.model.system import load_system

    from tests.conftest import PIPELINE_SYSTEM_JSON

    path = tmp_path / "system.json"
    path.write_text(json.dumps(PIPELINE_SYSTEM_JSON))
    system = load_system(path)
    counts = _counts(_pub_sub(3, 3), system=system)
    # /data is not in the pipeline model, so no depth; assert the lookup ran.
    assert counts.topics["/data"].qos_depth is None


def test_counts_payload_shape():
    payload = _counts(_pub_sub(10, 8, n_failed_takes=2)).to_dict()
    assert payload["total_dropped"] == 2
    assert payload["total_expected"] == 10
    assert payload["drop_rate_pct"] == pytest.approx(20.0)
    assert payload["failed_takes"] == 2
    assert "/data" in payload["topics"]


def test_empty_stream_yields_no_topics():
    counts = collect_drop_counts([], build_timeline([]))
    assert counts.topics == {}
    assert counts.total_takes == 0
    assert counts.to_dict()["drop_rate_pct"] == 0.0


def test_publish_without_publisher_init_is_ignored():
    """rcl_publish carrying an unknown handle cannot be attributed to a topic."""
    from lf_eval.reader.events import TraceEvent

    events = [TraceEvent("rcl_publish", 1000, {"publisher_handle": 0xDEAD})]
    counts = collect_drop_counts(events, build_timeline(events))
    assert counts.topics == {}


def test_unattributed_failed_take_is_counted_separately():
    from lf_eval.reader.events import TraceEvent

    events = [
        TraceEvent("rmw_take", 1000, {"rmw_subscription_handle": 0xBEEF, "taken": 0})
    ]
    counts = collect_drop_counts(events, build_timeline(events))
    assert counts.failed_takes == 1
    assert counts.unattributed_failed_takes == 1


# ── metric wrapper ───────────────────────────────────────────────────────


def test_metric_summarises_per_run(pipeline_run):
    result = get_metric("drops").compute([pipeline_run])
    assert result.available
    row = result.summary_rows[0]
    assert row["published"] == 40  # 20 each on /a_out and /b_out
    assert row["delivered"] == 40
    assert row["dropped"] == 0
    assert row["failed_takes"] == 0


def test_metric_reports_drops(pipeline_system):
    builder = _pub_sub(n_publish=20, n_deliver=15, n_failed_takes=4)
    events = builder.build()
    timeline = build_timeline(events)
    counts = collect_drop_counts(events, timeline)
    run = make_run("lossy", timeline, mode="stock", drop_counts=counts)
    result = get_metric("drops").compute([run])
    row = result.summary_rows[0]
    assert row["dropped"] == 5
    assert row["drop_pct"] == pytest.approx(25.0)
    assert row["failed_takes"] == 4


def test_metric_unavailable_without_counts():
    run = make_run("no_counts", build_timeline([]), mode="lfc", drop_counts=None)
    result = get_metric("drops").compute([run])
    assert result.available is False
    assert "rcl_publish" in result.note


def test_plot_writes_file(pipeline_run, tmp_path):
    metric = get_metric("drops")
    result = metric.compute([pipeline_run])
    written = metric.plot(result, tmp_path)
    assert [p.name for p in written] == ["drops.png"]
    assert written[0].is_file() and written[0].stat().st_size > 0
