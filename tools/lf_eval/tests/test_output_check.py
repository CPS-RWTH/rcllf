# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Output checks of paced runs: the declared-configuration side.

The trace side runs on real runs in the smoke."""

from __future__ import annotations

import gzip
import json
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import pytest

from lf_eval.metrics.drops import classify_reader, collect_drop_counts, window_counts
from lf_eval.output_check import _callback_triggers, _system_timers, accounting_window, delivery_reasons
from lf_eval.reader.events import TraceEvent
from lf_eval.semantic.fixed_input import Corpus, expected_dispatches, expected_inputs

SYSTEM = {"nodes": {
    "imu_node": {"class_name": "ImuNode", "timers": [{"callback": "timer_callback", "period_ms": 5.0,
                                                      "offset_ms": 100.0}], "subscribers": []},
    "fusion_node": {"class_name": "FusionNode", "timers": [],
                    "subscribers": [{"callback": "imu_callback", "topic": "/sentry/imu"}]},
}}
STATIC = {
    "rids": [{"rid": 0, "node": "ImuNode", "callback": "timer_callback"},
             {"rid": 1, "node": "FusionNode", "callback": "imu_callback"}],
    "edges": [{"id": 0, "status": "admitted", "topic": "/sentry/imu", "delay_ns": 1_000_000,
               "publisher": {"rids": [0]}, "subscriber": {"rid": 1}}],
}


def test_declared_timers_carry_the_offset_in_ns():
    assert _system_timers(SYSTEM) == [("imu_node", "ImuNode", "timer_callback", 100_000_000, 5_000_000)]


def test_callback_triggers_name_timer_and_subscription():
    assert _callback_triggers(SYSTEM) == {"ImuNode.timer_callback": ("imu_node", None),
                                         "FusionNode.imu_callback": ("fusion_node", "/sentry/imu")}


def test_offset_oracle_has_no_tag_zero_and_follows_the_hop():
    _, cls, cb, off, per = _system_timers(SYSTEM)[0]
    corpus = Corpus(path=None, sha256="", corpus_id="paced", horizon_ns=110_000_000,
                    inputs=((cls, cb, off, per),),
                    publishes={"ImuNode.timer_callback": ("/sentry/imu",)}, min_repetitions=2)
    assert expected_inputs(corpus) == [(100_000_000, "ImuNode.timer_callback"),
                                       (105_000_000, "ImuNode.timer_callback"),
                                       (110_000_000, "ImuNode.timer_callback")]
    want, cut = expected_dispatches(corpus, STATIC)
    assert want == Counter({("ImuNode.timer_callback", 100_000_000): 1,
                            ("ImuNode.timer_callback", 105_000_000): 1,
                            ("ImuNode.timer_callback", 110_000_000): 1,
                            ("FusionNode.imu_callback", 101_000_000): 1,
                            ("FusionNode.imu_callback", 106_000_000): 1})
    assert cut == ["ImuNode.timer_callback@110000000 /sentry/imu -> FusionNode.imu_callback@111000000"]


def test_relative_subscriber_topics_resolve_like_the_trace_names_them():
    # The reference system declares relative topics ("FrontLidarDriver"); the
    # binding log of w6-refsys-smoke2 shows them resolved to "/FrontLidarDriver".
    # Unresolved, every subscription callback counted 0 invocations
    # ("published:/PointsTransformerFront:expected=0:observed=341").
    system = {
        "nodes": {
            "PointsTransformerFront": {"class_name": "PointsTransformerFront", "timers": [],
                                       "subscribers": [{"callback": "sub_callback", "topic": "FrontLidarDriver"}]},
            "Scoped": {"class_name": "Scoped", "timers": [],
                       "subscribers": [{"callback": "cb", "topic": "points"}]},
        },
        "launch_graph": {"root": {"nodes": [{"name": "PointsTransformerFront", "namespace": None},
                                            {"name": "Scoped", "namespace": "/car"}]}},
    }
    assert _callback_triggers(system) == {
        "PointsTransformerFront.sub_callback": ("PointsTransformerFront", "/FrontLidarDriver"),
        "Scoped.cb": ("Scoped", "/car/points")}


# Publish accounting per reader. The fixture is recorded from a real stock run
# (tests/fixtures/overwrites/extract.py, run named in the file): one depth-1 topic
# of the reference system whose reader was created 7 s after its publisher.

FIXTURE = Path(__file__).parent / "fixtures" / "overwrites" / "refsys_stock_euclidean_cluster_settings.json.gz"
TOPIC = "/EuclideanClusterSettings"
EDGE = "/EuclideanClusterSettings->EuclideanClusterDetector"


def _recorded_events() -> list[TraceEvent]:
    data = json.loads(gzip.open(FIXTURE, "rt").read())
    return [TraceEvent(name=n, timestamp_ns=t, pid=p, fields=f) for n, t, p, f in data["events"]]


def _counts(events: list[TraceEvent]):
    # Delivered callbacks equal successful takes in a stock executor (one take, one callback).
    taken = sum(1 for e in events if e.name == "rmw_take" and e.get("taken") == 1)
    timeline = SimpleNamespace(topic_invocations={("EuclideanClusterDetector", TOPIC): [None] * taken})
    return collect_drop_counts(events, timeline)


def test_recorded_stock_overwrites_pass_per_reader_and_fail_the_count_rule():
    counts = _counts(_recorded_events())
    tc, rc = counts.topics[TOPIC], counts.readers[EDGE]
    assert (tc.published, tc.delivered, tc.subscribers) == (912, 501, 1)
    # The count-only rule (no reader accounting) fails the run on these drops.
    assert delivery_reasons(TOPIC, tc, {}) == ["dropped:/EuclideanClusterSettings:411"]
    d = rc.to_dict()
    window = d.pop("window")
    assert d == {"queue_depth": 1, "published": 912, "received": 501, "overwritten": 135,
                 "before_reader": 276, "before_match": 0, "queued_at_end": 0, "unclassified": 0,
                 "before_first_receipt": 123, "match_traced": False, "unmatched_takes": 0,
                 "before_writer_match": 0, "within_match_resolution": 0, "no_reader_detection": 0,
                 "after_both_match": 0}
    assert rc.reconciles
    assert delivery_reasons(TOPIC, tc, {EDGE: rc}) == []
    # One reader: the window starts at its creation and holds everything but before_reader.
    created = next(e.timestamp_ns for e in _recorded_events() if e.name == "rcl_subscription_init")
    assert counts.window_start_ns == created
    assert {k: window[k] for k in ("published", "received", "overwritten", "dropped")} == {
        "published": 636, "received": 501, "overwritten": 135, "dropped": 135}
    assert window["delivery_latency_ns"]["n"] == 501


def test_a_take_that_should_have_seen_the_sample_makes_it_a_loss():
    events = _recorded_events()
    created = next(e.timestamp_ns for e in events if e.name == "rcl_subscription_init")
    reader = next(e for e in events if e.name == "rmw_take")
    taken = {e.get("source_timestamp") for e in events if e.name == "rmw_take"}
    first_rx = min(e.timestamp_ns for e in events if e.name == "rmw_publish" and e.get("timestamp") in taken)
    # After the reader's first receipt, so the startup gap category cannot absorb it.
    missing = next(e for e in events if e.name == "rmw_publish" and e.timestamp_ns > max(created, first_rx)
                   and e.get("timestamp") not in taken)
    # The reader polls right after that publish and finds nothing: no overwrite
    # can explain the sample's absence any more.
    events.append(TraceEvent(name="rmw_take", timestamp_ns=missing.timestamp_ns + 1, pid=reader.pid,
                             fields={**reader.fields, "source_timestamp": 0, "taken": 0}))
    counts = _counts(sorted(events, key=lambda e: e.timestamp_ns))
    rc = counts.readers[EDGE]
    assert (rc.overwritten, rc.unclassified) == (134, 1)
    assert delivery_reasons(TOPIC, counts.topics[TOPIC], {EDGE: rc}) == [f"lost:{EDGE}:1"]



# The same discovery loss on a depth-1 and a depth-8 reader. Recorded
# from depth-1 control run r17 (fixture names it) of the 2026-10-14 campaign: three
# samples of /EuclideanClusterDetector are lost before ObjectCollisionEstimator's
# first receipt, the last one 96 us before the probe saw the match.
GAP_FIXTURE = FIXTURE.parent / "refsys_stock_euclidean_cluster_detector_gap.json.gz"
GAP_TOPIC, GAP_NODE = "/EuclideanClusterDetector", "ObjectCollisionEstimator"
GAP_EDGE = f"{GAP_TOPIC}->{GAP_NODE}"


def _gap_counts(depth=None, takes=True):
    data = json.loads(gzip.open(GAP_FIXTURE, "rt").read())
    events = [TraceEvent(name=n, timestamp_ns=t, pid=p, fields=f) for n, t, p, f in data["events"]
              if takes or n != "rmw_take"]
    if depth is not None:
        events = [TraceEvent(name=e.name, timestamp_ns=e.timestamp_ns, pid=e.pid,
                             fields={**e.fields, "queue_depth": depth}) if e.name == "rcl_subscription_init"
                  else e for e in events]
    taken = sum(1 for e in events if e.name == "rmw_take" and e.get("taken") == 1)
    timeline = SimpleNamespace(topic_invocations={(GAP_NODE, GAP_TOPIC): [None] * taken})
    counts = collect_drop_counts(events, timeline)
    return counts, counts.readers[GAP_EDGE]


def test_a_depth1_discovery_loss_is_not_hidden_as_an_overwrite():
    counts, rc = _gap_counts()
    d = rc.to_dict()
    assert (d["queue_depth"], d["published"], d["received"], d["before_first_receipt"]) == (1, 249, 246, 3)
    assert (d["overwritten"], d["unclassified"], d["before_writer_match"], d["within_match_resolution"]) == (
        0, 3, 2, 1), (
        "a depth-1 reader's loss before its first receipt and before the probe saw the match was "
        f"counted as overwritten, hiding a discovery loss a depth-8 reader fails on, got {d}")
    assert delivery_reasons(GAP_TOPIC, counts.topics[GAP_TOPIC], {GAP_EDGE: rc}) == [f"lost:{GAP_EDGE}:1"]


def test_the_discovery_split_does_not_depend_on_the_queue_depth():
    shallow, deep = _gap_counts()[1].to_dict(), _gap_counts(depth=8)[1].to_dict()
    keys = ("overwritten", "unclassified", "before_writer_match", "within_match_resolution", "after_both_match")
    assert {k: shallow[k] for k in keys} == {k: deep[k] for k in keys}, (shallow, deep)


# The same r17 trace with every take trimmed, so no take
# follows the three discovery losses. Their newer samples still establish them.
@pytest.mark.parametrize("depth", [None, 8])
def test_a_discovery_loss_without_a_later_take_is_not_an_overwrite(depth):
    d = _gap_counts(depth, takes=False)[1].to_dict()
    n = d["queue_depth"]
    assert (d["published"], d["received"]) == (249, 0)
    assert (d["unclassified"], d["before_writer_match"], d["within_match_resolution"]) == (3, 2, 1), (
        f"a depth-{n} reader's discovery loss with no later take was counted as overwritten "
        f"instead of split by the match, got {d}")
    assert (d["overwritten"], d["queued_at_end"]) == (249 - 3 - n, n)


def _ev(name, t, **fields):
    return TraceEvent(name=name, timestamp_ns=t, pid=7, fields=fields)


def test_a_loss_after_its_reader_exists_fails_the_run_inside_or_before_the_window():
    # Reader A of /a is created at 100, reader B of /b at 300, so the window starts at 300.
    # /a carries samples 1, 3, 5, 7 and 9 (source timestamps) at 50, 150, 250, 350 and 450.
    # Sample 1 predates A. A takes 3 and 9. Right after 5 and 7 it polls and finds
    # nothing, before any newer sample arrived: both follow A's first receipt and are
    # losses, 5 before the window, 7 inside.
    ev = [_ev("rcl_node_init", 1, node_handle=1, node_name="A"),
          _ev("rcl_node_init", 2, node_handle=2, node_name="B"),
          _ev("rcl_publisher_init", 3, publisher_handle=11, rmw_publisher_handle=21, topic_name="/a"),
          _ev("rcl_subscription_init", 100, rmw_subscription_handle=31, node_handle=1, topic_name="/a",
              queue_depth=1),
          _ev("rcl_subscription_init", 300, rmw_subscription_handle=32, node_handle=2, topic_name="/b",
              queue_depth=1)]
    for t in (50, 150, 250, 350, 450):
        stamp = t // 50
        ev += [_ev("rcl_publish", t - 1, publisher_handle=11),
               _ev("rmw_publish", t, rmw_publisher_handle=21, timestamp=stamp)]
        if t > 100:
            got = stamp in (3, 9)
            ev.append(_ev("rmw_take", t + 5, rmw_subscription_handle=31, taken=int(got),
                          source_timestamp=stamp if got else 0))
    ev.sort(key=lambda e: e.timestamp_ns)
    counts = collect_drop_counts(ev, SimpleNamespace(topic_invocations={("A", "/a"): [None, None]}))
    rc = counts.readers["/a->A"]
    # Neither the startup category nor the window absorbs a loss after the reader existed.
    assert delivery_reasons("/a", counts.topics["/a"], {"/a->A": rc}) == ["lost:/a->A:2"]
    assert counts.window_start_ns == 300
    assert (rc.published, rc.received, rc.before_reader, rc.unclassified) == (5, 2, 1, 2)
    assert rc.reconciles
    assert (rc.window.published, rc.window.received, rc.window.unclassified) == (2, 1, 1)
    win = accounting_window(counts, [rc])
    assert (win["start_ns"], win["start_after_trace_begin_ns"], win["unclassified"]) == (300, 299, 1)
    assert win["delivery_latency_ns"] == {"n": 1, "p50": 5, "p99": 5, "max": 5}
    # The default window ends at the run's stop, its last application publish.
    assert (counts.stop_ns, win["end_ns"], win["stop_after_trace_begin_ns"]) == (450, 450, 449)
    assert win["per_s"]["published"] == round(2 / 150e-9, 3)  # 2 samples over 150 ns
    # A common window set from outside recounts the same outcomes over its span:
    # [300, 400] keeps 350 (the loss) and drops 450 (the take of 9).
    counts.set_window(300, 400)
    win = accounting_window(counts, [rc])
    assert (win["published"], win["received"], win["unclassified"]) == (1, 0, 1)
    assert win["per_s"]["unclassified"] == round(1 / 100e-9, 3)  # 1 loss over 100 ns
    assert counts.stop_ns == 450 and win["stop_after_trace_begin_ns"] == 449


def test_only_samples_before_the_traced_match_are_exempt():
    # Depth 8, the reader exists from 0 and DDS matches it at 35.
    # Samples 1 to 4 (10 to 40) are never taken, samples 5 and 6 are.
    samples = [(10, 1), (20, 2), (30, 3), (40, 4), (50, 5), (60, 6)]
    takes = [(25, None), (55, 5), (65, 6)]
    rc = classify_reader("/t", "n", 8, 0, samples, takes, matched_ns=35)
    # 1 to 3 predate the match. 4 was published after it and the take at 55 should
    # have returned it: a loss, although it also predates the first receipt.
    assert (rc.received, rc.before_match, rc.unclassified, rc.overwritten) == (2, 3, 1, 0)
    assert rc.before_first_receipt == 4 and rc.reconciles
    assert delivery_reasons("/t", SimpleNamespace(subscribers=1), {"/t->n": rc}) == ["lost:/t->n:1"]
    d = rc.to_dict()
    assert (d["before_match"], d["before_first_receipt"], d["match_traced"]) == (3, 4, True)
    # Without a traced match nothing is exempt: the first receipt alone is no evidence.
    rc = classify_reader("/t", "n", 8, 0, samples, takes)
    assert (rc.before_match, rc.unclassified, rc.before_first_receipt) == (0, 4, 4)
    assert rc.to_dict()["match_traced"] is False


def _match_probe_events():
    """One reader of /a, last seen unmatched at 25, four samples, the last two taken."""
    ev = [_ev("rcl_node_init", 1, node_handle=1, node_name="A"),
          _ev("rcl_publisher_init", 2, publisher_handle=11, rmw_publisher_handle=21, topic_name="/a"),
          _ev("rcl_subscription_init", 3, rmw_subscription_handle=31, node_handle=1, topic_name="/a",
              queue_depth=8),
          # Found matched at 27, the poll before (at 25) found no publisher.
          _ev("reader_matched", 27, rmw_subscription_handle=31, topic_name="/a", publishers=1,
              unmatched_age_ns=2),
          _ev("reader_matched", 45, rmw_subscription_handle=31, topic_name="/a", publishers=1,
              unmatched_age_ns=0)]
    for t in (10, 20, 30, 40):
        ev += [_ev("rcl_publish", t - 1, publisher_handle=11),
               _ev("rmw_publish", t, rmw_publisher_handle=21, timestamp=t)]
    ev += [_ev("rmw_take", 35, rmw_subscription_handle=31, taken=1, source_timestamp=30),
           _ev("rmw_take", 42, rmw_subscription_handle=31, taken=1, source_timestamp=40)]
    ev.sort(key=lambda e: e.timestamp_ns)
    return ev


def test_the_match_probe_event_sets_the_readers_match_time():
    # The rcllf_dds:reader_matched event of the reader's rmw handle, in its process.
    counts = collect_drop_counts(_match_probe_events(),
                                 SimpleNamespace(topic_invocations={("A", "/a"): [None, None]}))
    rc = counts.readers["/a->A"]
    assert rc.matched_ns == 25, "the match bound is the first event minus its unmatched age"
    assert (rc.received, rc.before_match, rc.unclassified) == (2, 2, 0)
    win = accounting_window(counts, [rc])
    # The window opens at the subscription init (3), so both startup samples fall inside it.
    assert (win["before_match"], win["before_first_receipt"]) == (2, 2), win


def test_load_run_reads_the_match_probe_event(tmp_path, monkeypatch):
    # load_run reads only the events its metrics name, so the match
    # probe's event must be one of them or every reader looks unmatched.
    from lf_eval.model.run import load_run

    events = _match_probe_events()

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        yield from (e for e in events if event_names is None or e.name in event_names)

    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    rc = load_run(tmp_path).drop_counts.readers["/a->A"]
    assert rc.matched_ns == 25, "load_run dropped rcllf_dds:reader_matched, so no reader has a match time"
    assert (rc.before_match, rc.unclassified) == (2, 0)


def test_writer_side_match_splits_the_unclassified_losses():
    # The reader matched at 5. The writer of samples 1 to 3
    # last raised its matched count at 30, the writer of sample 4 is untraced.
    samples = [(10, 1), (20, 2), (40, 3), (50, 4)]
    takes = [(12, None), (25, None), (45, None), (55, None)]
    rc = classify_reader("/t", "n", 8, 0, samples, takes, matched_ns=5,
                         writer_matched_ns={1: 30, 2: 30, 3: 30})
    assert rc.unclassified == 4 and rc.reconciles
    assert (rc.before_writer_match, rc.after_both_match) == (2, 1), (
        "the split must follow the writer's last match and skip the untraced writer")
    d = rc.to_dict()
    assert (d["before_writer_match"], d["after_both_match"]) == (2, 1)
    assert window_counts(rc, 15, None).window_dict()["before_writer_match"] == 1


def test_only_after_both_match_losses_fail_the_delivery():
    # A loss before the writer side match is a
    # startup discovery gap, reported in its own column but never a failure.
    samples = [(10, 1), (20, 2), (40, 3), (50, 4)]
    takes = [(12, None), (25, None), (45, None), (55, None)]
    tc = SimpleNamespace(subscribers=1)
    rc = classify_reader("/t", "n", 8, 0, samples, takes, matched_ns=5,
                         writer_matched_ns={1: 30, 2: 30})
    assert (rc.unclassified, rc.before_writer_match, rc.after_both_match) == (4, 2, 0)
    # Samples 3 and 4 have no traced writer match, so they still fail.
    assert delivery_reasons("/t", tc, {"/t->n": rc}) == ["lost:/t->n:2"], (
        "before_writer_match losses failed the delivery check")
    rc = classify_reader("/t", "n", 8, 0, samples[:2], takes[:2], matched_ns=5,
                         writer_matched_ns={1: 30, 2: 30})
    assert delivery_reasons("/t", tc, {"/t->n": rc}) == [], (
        "before_writer_match losses failed the delivery check")
    assert rc.to_dict()["before_writer_match"] == 2
    rc = classify_reader("/t", "n", 8, 0, samples, takes, matched_ns=5,
                         writer_matched_ns={1: 5, 2: 5, 3: 5, 4: 5})
    assert rc.after_both_match == 4
    assert delivery_reasons("/t", tc, {"/t->n": rc}) == ["lost:/t->n:4"], (
        "after_both_match must stay a failure for every variant")


def _writer_probe_events():
    """Reader of /a matched at 5, writer 21 matched one reader at 14 and its last at 30."""
    ev = [_ev("rcl_node_init", 1, node_handle=1, node_name="A"),
          _ev("rcl_publisher_init", 2, publisher_handle=11, rmw_publisher_handle=21, topic_name="/a"),
          _ev("rcl_subscription_init", 3, rmw_subscription_handle=31, node_handle=1, topic_name="/a",
              queue_depth=8),
          _ev("reader_matched", 6, rmw_subscription_handle=31, topic_name="/a", publishers=1,
              unmatched_age_ns=1),
          _ev("writer_matched", 15, rmw_publisher_handle=21, topic_name="/a", subscriptions=1,
              unmatched_age_ns=1),
          _ev("writer_matched", 33, rmw_publisher_handle=21, topic_name="/a", subscriptions=2,
              unmatched_age_ns=3)]
    for t in (10, 20, 30, 40):
        ev += [_ev("rcl_publish", t - 1, publisher_handle=11),
               _ev("rmw_publish", t, rmw_publisher_handle=21, timestamp=t)]
    ev += [_ev("rmw_take", 12, rmw_subscription_handle=31, taken=0),
           _ev("rmw_take", 35, rmw_subscription_handle=31, taken=1, source_timestamp=30),
           _ev("rmw_take", 45, rmw_subscription_handle=31, taken=0)]
    ev.sort(key=lambda e: e.timestamp_ns)
    return ev


def test_load_run_reads_the_writer_match_event(tmp_path, monkeypatch):
    # Samples 10 and 20 predate the writer's last match (30), 40 follows it.
    from lf_eval.model.run import load_run

    events = _writer_probe_events()

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        yield from (e for e in events if event_names is None or e.name in event_names)

    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    rc = load_run(tmp_path).drop_counts.readers["/a->A"]
    assert (rc.received, rc.before_match, rc.unclassified) == (1, 0, 3)
    assert (rc.before_writer_match, rc.after_both_match) == (2, 1), (
        "load_run dropped rcllf_dds:writer_matched, so no loss has a writer side match")


def test_a_replacement_reader_gets_the_writers_match_with_it():
    # Reader 31 of node A matches writer 21 at 5 and
    # is destroyed. Reader 32 of node B is created at 50, the writer's matched
    # count drops to 0 and rises back to 1 at 60 (bound 58), B matches at 56.
    ev = [_ev("rcl_node_init", 1, node_handle=1, node_name="A"),
          _ev("rcl_node_init", 1, node_handle=2, node_name="B"),
          _ev("rcl_publisher_init", 2, publisher_handle=11, rmw_publisher_handle=21, topic_name="/a"),
          _ev("rcl_subscription_init", 3, rmw_subscription_handle=31, node_handle=1, topic_name="/a",
              queue_depth=8),
          _ev("reader_matched", 5, rmw_subscription_handle=31, topic_name="/a", publishers=1,
              unmatched_age_ns=0),
          _ev("writer_matched", 6, rmw_publisher_handle=21, topic_name="/a", subscriptions=1,
              unmatched_age_ns=1),
          _ev("rcl_subscription_init", 50, rmw_subscription_handle=32, node_handle=2, topic_name="/a",
              queue_depth=8),
          _ev("reader_matched", 57, rmw_subscription_handle=32, topic_name="/a", publishers=1,
              unmatched_age_ns=1),
          _ev("writer_matched", 60, rmw_publisher_handle=21, topic_name="/a", subscriptions=1,
              unmatched_age_ns=2)]
    # 54 predates B's match, 57 follows it but predates the writer's match with B.
    for t in (54, 57, 65):
        ev += [_ev("rcl_publish", t - 1, publisher_handle=11),
               _ev("rmw_publish", t, rmw_publisher_handle=21, timestamp=t)]
    ev += [_ev("rmw_take", 70, rmw_subscription_handle=32, taken=1, source_timestamp=65)]
    ev.sort(key=lambda e: e.timestamp_ns)
    counts = collect_drop_counts(ev, SimpleNamespace(
        topic_invocations={("A", "/a"): [None], ("B", "/a"): [None]}))
    rc = counts.readers["/a->B"]
    assert (rc.received, rc.before_match, rc.unclassified) == (1, 1, 1), rc.to_dict()
    assert (rc.before_writer_match, rc.after_both_match) == (1, 0), (
        "a replacement reader's loss got the writer's old match time: "
        "it counted as after_both_match")


def test_depth_and_run_end_classify_the_rest():
    # depth 2, samples 1 to 4 published at 10 to 40, the reader takes sample 3 at 35.
    # Its first receipt is sample 0 (published at 5, taken at 7), so none of the
    # rest predates it.
    samples = [(5, 0), (10, 1), (20, 2), (30, 3), (40, 4)]
    rc = classify_reader("/t", "n", 2, 0, samples, [(7, 0), (35, 3)])
    # 1: two newer arrivals before the take, overwritten. 2: only one, lost.
    # 4: no take follows and only zero newer arrivals, still queued.
    assert (rc.received, rc.overwritten, rc.unclassified, rc.queued_at_end) == (2, 1, 1, 1)
    assert rc.reconciles
    # A take that finds nothing after sample 4 was published makes it a loss too.
    rc = classify_reader("/t", "n", 2, 0, samples, [(7, 0), (35, 3), (45, None)])
    assert (rc.unclassified, rc.queued_at_end) == (2, 0)
    # Without a queue depth nothing counts as overwrite.
    rc = classify_reader("/t", "n", None, 0, samples, [(7, 0), (35, 3)])
    assert (rc.overwritten, rc.unclassified, rc.queued_at_end) == (0, 2, 1)
