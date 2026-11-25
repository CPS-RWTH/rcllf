# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A loss inside the match probe's resolution is its own class, still a failure.

The probe traces each matched count increase at its detection time (the rmw matched
event, or the poll that saw it) with the age of the last observation of the lower count.
The match lies between the two. The startup exemptions stay keyed to the earlier end, and
``after_both_match`` needs a publish at or after the detection on both sides."""

from __future__ import annotations

from types import SimpleNamespace

from lf_eval.metrics.drops import classify_reader, collect_drop_counts, window_counts
from lf_eval.output_check import accounting_window, delivery_reasons
from lf_eval.reader.events import TraceEvent


def _ev(name, t, **fields):
    return TraceEvent(name=name, timestamp_ns=t, pid=7, fields=fields)


def _gap_run(reader_detect, reader_age, writer_detect, writer_age, lost, source=None, sub_init=3):
    """Writer 21 and reader 31 of /a, depth 8. ``lost`` samples are never taken, the
    take right after each finds nothing. Sample 9_200_000 is taken. Times in us
    scale, shaped like run ``..._r1`` of the match-gap campaign."""
    extra = {} if source is None else {"source": source}
    ev = [_ev("rcl_node_init", 1, node_handle=1, node_name="Visualizer"),
          _ev("rcl_publisher_init", 2, publisher_handle=11, rmw_publisher_handle=21, topic_name="/a"),
          _ev("rcl_subscription_init", sub_init, rmw_subscription_handle=31, node_handle=1, topic_name="/a",
              queue_depth=8),
          _ev("writer_matched", writer_detect, rmw_publisher_handle=21, topic_name="/a", subscriptions=1,
              unmatched_age_ns=writer_age, **extra),
          _ev("reader_matched", reader_detect, rmw_subscription_handle=31, topic_name="/a", publishers=1,
              unmatched_age_ns=reader_age, **extra)]
    if reader_detect is None:
        ev.pop()
    for t in (*lost, 9_200_000):
        ev += [_ev("rcl_publish", t - 1, publisher_handle=11),
               _ev("rmw_publish", t, rmw_publisher_handle=21, timestamp=t)]
    ev += [_ev("rmw_take", t + 5, rmw_subscription_handle=31, taken=0) for t in lost]
    ev += [_ev("rmw_take", 9_200_010, rmw_subscription_handle=31, taken=1, source_timestamp=9_200_000)]
    ev.sort(key=lambda e: e.timestamp_ns)
    counts = collect_drop_counts(ev, SimpleNamespace(topic_invocations={("Visualizer", "/a"): [None]}))
    return counts, counts.readers["/a->Visualizer"]


def test_a_loss_inside_the_reader_poll_gap_is_not_after_both_match():
    # Run _r1: the writer matched at 7_686_759 (bound 7_685_702). The reader's last
    # unmatched poll was at 9_042_165, the sample went out at 9_042_194 and the poll
    # saw the reader's increase at 9_043_226. The 9_100_000 loss follows both increases.
    _counts, rc = _gap_run(9_043_226, 1_061, 7_686_759, 1_057, lost=(9_042_194, 9_100_000))
    assert (rc.before_match, rc.unclassified, rc.before_writer_match) == (0, 2, 0), rc.to_dict()
    d = rc.to_dict()
    assert (d.get("within_match_resolution", 0), d["after_both_match"]) == (1, 1), (
        "a loss inside the match probe's poll gap counted as after_both_match: the trace cannot "
        f"show that both sides had matched when it was published, got {d}")
    # Still a failure, like every loss after the earliest possible match.
    assert delivery_reasons("/a", SimpleNamespace(subscribers=1), {"/a->Visualizer": rc}) == [
        "lost:/a->Visualizer:2"]


def test_a_loss_inside_the_writer_poll_gap_is_not_after_both_match():
    # Run _r9 (PointsTransformerRear): reader matched long before, the writer's last
    # unmatched poll at 7_733_896, the sample at 7_733_946, the increase seen at 7_734_953.
    _counts, rc = _gap_run(6_454_132, 1_058, 7_734_953, 1_057, lost=(7_733_946,))
    d = rc.to_dict()
    assert (d["before_writer_match"], d.get("within_match_resolution", 0), d["after_both_match"]) == (0, 1, 0), (
        f"a loss inside the writer side poll gap counted as after_both_match, got {d}")
    assert rc.reconciles


def test_the_event_time_classifies_what_the_poll_could_not():
    # The rmw matched event saw the reader's increase at 9_042_180 instead of the
    # poll's 9_043_226. The 9_042_194 loss now lies after both detections.
    _counts, rc = _gap_run(9_042_180, 15, 7_686_759, 1_057, lost=(9_042_194,), source="event")
    assert (rc.within_match_resolution, rc.after_both_match) == (0, 1), rc.to_dict()
    # An event after the publish keeps it in the resolution class, never exempt.
    _counts, rc = _gap_run(9_042_200, 35, 7_686_759, 1_057, lost=(9_042_194,), source="event")
    assert (rc.before_match, rc.before_writer_match, rc.within_match_resolution) == (0, 0, 1)
    # A publish before the last unmatched observation stays exempt as before.
    _counts, rc = _gap_run(9_042_200, 35, 7_686_759, 1_057, lost=(9_042_160,), source="event")
    assert (rc.before_match, rc.unclassified) == (1, 0)


def test_the_run_reports_which_source_timed_each_match():
    from lf_eval.output_check import match_source_summary

    counts, _rc = _gap_run(9_042_180, 15, 7_686_759, 1_057, lost=(9_042_194,), source="event")
    assert counts.match_sources == {"event": 2}
    assert match_source_summary(counts.match_sources)["source"] == "event"
    # Traces from the poll-only probe carry no source field and read as poll.
    counts, _rc = _gap_run(9_043_226, 1_061, 7_686_759, 1_057, lost=(9_042_194,))
    assert counts.match_sources == {"poll": 2}
    assert match_source_summary({"event": 3, "poll": 1}) == {"source": "mixed", "event": 3, "poll": 1}
    assert match_source_summary({})["source"] == "none"


def test_the_window_and_the_dict_carry_the_class():
    counts, rc = _gap_run(9_043_226, 1_061, 7_686_759, 1_057, lost=(9_042_194, 9_100_000))
    assert rc.to_dict()["within_match_resolution"] == 1
    assert window_counts(rc, 9_050_000, None).window_dict()["within_match_resolution"] == 0
    assert window_counts(rc, 0, None).window_dict()["within_match_resolution"] == 1
    win = accounting_window(counts, [rc])
    assert (win["within_match_resolution"], win["after_both_match"]) == (1, 1)


def test_without_detection_times_the_split_is_the_old_one():
    # Callers that pass only the bounds keep the split unchanged.
    samples = [(10, 1), (20, 2), (40, 3)]
    takes = [(12, None), (25, None), (45, None)]
    rc = classify_reader("/t", "n", 8, 0, samples, takes, matched_ns=5,
                         writer_matched_ns={1: 30, 2: 30, 3: 30})
    assert (rc.before_writer_match, rc.within_match_resolution, rc.after_both_match) == (2, 0, 1)


def test_a_writer_event_just_before_the_reader_init_trace_still_bounds_the_writer():
    # Run 20261011T134347Z..._r6 of the output check: the writer's matched event
    # fired 32 us before the reader's rcl_subscription_init (DDS matches inside rmw
    # create). The loss lies in the reader's resolution gap. It must not read as a
    # loss whose writer the probe did not see, and it must never become exempt.
    _counts, rc = _gap_run(6_840_486, 766, 6_671_024, 582, lost=(6_840_081,), source="event",
                           sub_init=6_671_056)
    d = rc.to_dict()
    assert (d["before_writer_match"], d["within_match_resolution"], d["after_both_match"]) == (0, 1, 0), (
        "a writer matched event traced just before rcl_subscription_init left the loss without a "
        f"writer bound, reported as a writer the probe did not see, got {d}")
    assert delivery_reasons("/a", SimpleNamespace(subscribers=1), {"/a->Visualizer": rc}) == [
        "lost:/a->Visualizer:1"]


def test_a_loss_without_a_reader_detection_is_not_after_both_match():
    # The writer matched at 7_686_759 (event), the reader's increase was never traced.
    # The 9_100_000 loss follows the writer detection, but nothing shows the reader matched.
    _counts, rc = _gap_run(None, None, 7_686_759, 15, lost=(9_100_000,), source="event")
    d = rc.to_dict()
    assert (d["after_both_match"], d.get("no_reader_detection", 0)) == (0, 1), (
        "a loss with a writer increase but no reader detection counted as after_both_match: the "
        f"trace cannot show the reader matched, got {d}")
    assert (rc.before_match, rc.before_writer_match, rc.within_match_resolution) == (0, 0, 0)
    assert delivery_reasons("/a", SimpleNamespace(subscribers=1), {"/a->Visualizer": rc}) == [
        "lost:/a->Visualizer:1"]


def test_bounds_only_callers_keep_the_old_split():
    # Without detection mappings a loss after the writer bound is after_both_match.
    rc = classify_reader("/a", "V", 8, 0, [(100, 1), (200, 2)], [(150, None), (250, 2)],
                         writer_matched_ns={1: 50, 2: 50})
    assert (rc.after_both_match, rc.no_reader_detection) == (1, 0), rc.to_dict()
