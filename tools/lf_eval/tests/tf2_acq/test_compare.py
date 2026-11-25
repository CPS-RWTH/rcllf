# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Reader and comparator of the tf2 acquisition showcase.

The fixtures are the RCLLF_TF2/ACQ lines of two real converted runs of
tf2_acq_example (2026-11-13, dev image with tf2_ros 0.36.23): the adapter as
shipped, and the falsification mutant (RCLLF_TF2_TEST_MUTANT=latest_on_timeout)
that answers a timed-out request with the latest transform. The hand table is
the showcase's own corpus/t1_t7_expected.tsv, derived before the first run.
"""

from pathlib import Path

from lf_eval import tf2_acq

HERE = Path(__file__).parent / "fixtures"
HAND = (Path(__file__).resolve().parents[4] / "packages" / "src" / "tf2_acq_example"
        / "corpus" / "t1_t7_expected.tsv")


def _load(name):
    log = (HERE / name).read_text()
    origin, ids = tf2_acq.showcase_ids(log)
    return tf2_acq.records(log), ids, origin


def test_converted_run_equals_the_hand_table():
    recs, ids, origin = _load("converted.log")
    assert ids == {("/acq_sampler", 0, i): c for i, c in
                   {1: "T1", 2: "T2", 3: "T5", 4: "T3", 5: "T6", 6: "T7", 7: "T4"}.items()}
    assert tf2_acq.compare(recs, ids, origin, tf2_acq.read_hand_table(HAND)) is None


def test_mutant_latest_on_timeout_is_named_at_the_t5_row():
    """Output verification 3: the latest-on-timeout mutant answers T5 with the
    held_link transform at stamp 0 instead of reject; the comparator names it."""
    recs, ids, origin = _load("mutant.log")
    assert tf2_acq.compare(recs, ids, origin, tf2_acq.read_hand_table(HAND)) == \
        "T5: outcome ok != reject"


def test_outcome_stream_is_relative_and_drops_the_physical_wait():
    recs, _, origin = _load("converted.log")
    stream = tf2_acq.outcome_stream(recs, origin)
    assert stream[0] == ("1\tok\t60000000\t0\t30000000\t0\t0.030000000,0.000000000,0.000000000,"
                         "0.000000000,0.000000000,0.000000000,1.000000000\t-")
    # T3 decided at the ingest of the 140 ms /tf, one microstep after its tag.
    assert stream[2].startswith("4\tok\t140000000\t1\t125000000\t20000000\t0.125000000,")
    assert stream[3] == "3\treject\t145000000\t0\t90000000\t55000000\t-\textrapolation"
    assert stream[5] == "6\tfailure\t235000000\t0\t170000000\t55000000\t-\tunknown_frame"
    assert not any("wait_physical" in s for s in stream)


def test_decision_time_off_by_a_microsecond_is_named():
    recs, ids, origin = _load("converted.log")
    for r in recs:
        if r.get("event") == "tf_outcome" and r["id"] == 7:
            r["time"] += 1000
    assert tf2_acq.compare(recs, ids, origin, tf2_acq.read_hand_table(HAND)) == \
        "T4: decided at 385.001 ms != 385 ms"


def test_open_request_at_shutdown_is_named():
    recs, ids, origin = _load("converted.log")
    recs.append({"event": "tf_open_at_shutdown", "node": "/acq_sampler", "id": 7})
    assert tf2_acq.compare(recs, ids, origin, []) == "T4: still open at shutdown"


def test_pin_check():
    assert tf2_acq.check_pin("0.36.23-1noble.20260915.172312") is None
    assert tf2_acq.check_pin("0.36.24-1noble") == "version_unpinned:tf2_ros 0.36.24-1noble"
    assert tf2_acq.check_pin("") == "version_unpinned:tf2_ros missing"


EVENTS = HAND.parent / "t1_t7_events.tsv"


def test_converted_run_matches_the_corpus_requests():
    recs, ids, origin = _load("converted.log")
    reqs = tf2_acq.read_requests(EVENTS)
    assert reqs["T3"] == (120, 125)
    assert tf2_acq.compare(recs, ids, origin, tf2_acq.read_hand_table(HAND), reqs) is None


def test_missing_request_record_is_named():
    """Dropping the tf_request records must not pass."""
    recs, ids, origin = _load("converted.log")
    recs = [r for r in recs if r.get("event") != "tf_request"]
    assert tf2_acq.compare(recs, ids, origin, tf2_acq.read_hand_table(HAND)) == \
        "T1: 0 requests, want 1"


def test_wrong_requested_stamp_is_named():
    """T6 (static frame) gives the same transform at any stamp; only the
    request check can tell a wrong stamp."""
    recs, ids, origin = _load("converted.log")
    for r in recs:
        if r.get("id") == 5 and r.get("event") in ("tf_request", "tf_outcome"):
            r["stamp"] += 1_000_000
    assert tf2_acq.compare(recs, ids, origin, tf2_acq.read_hand_table(HAND),
                           tf2_acq.read_requests(EVENTS)) == "T6: stamp 151.0 ms != 150 ms"


def test_reattached_ingress_fails_the_run():
    recs, ids, origin = _load("converted.log")
    recs.append({"event": "tf_ingress_reattached", "node": "/acq_sampler"})
    assert tf2_acq.compare(recs, ids, origin, tf2_acq.read_hand_table(HAND)) == \
        "ingress: listener group of /acq_sampler re-enabled outside LF"


def test_two_adapters_of_one_node_pair_by_adapter_and_id():
    """A second adapter of the sampler node also
    numbers its first request 1. Paired by (node, id) the two requests fall on
    one case and the run fails with "T1: 2 requests, want 1"; paired by
    (node, adapter, id) each outcome meets its own request."""
    recs, ids, origin = _load("converted.log")
    stamp = origin + 200_000_000
    recs += [
        {"event": "tf_request", "node": "/acq_sampler", "adapter": 1, "id": 1,
         "stamp": stamp, "time": origin + 200_000_000, "micro": 0},
        {"event": "tf_outcome", "node": "/acq_sampler", "adapter": 1, "id": 1,
         "outcome": "reject", "stamp": stamp, "time": origin + 255_000_000, "micro": 0,
         "wait_logical_ns": 55_000_000, "transform": None, "error": "extrapolation"},
    ]
    hand = tf2_acq.read_hand_table(HAND) + [("/acq_sampler#1/1", "reject", 255, 0.0, 0.0)]
    assert tf2_acq.compare(recs, ids, origin, hand) is None
    assert tf2_acq.pair_key(recs[-1]) == ("/acq_sampler", 1, 1)
    assert tf2_acq.pair_key({"node": "/n", "id": 3}) == ("/n", 0, 3)
    stream = tf2_acq.outcome_stream(recs, origin)
    assert stream[-1].startswith("1/1\treject\t255000000\t")
    assert stream[0].startswith("1\tok\t")


def test_two_nodes_with_one_id_keep_their_own_cases():
    """A second node's first adapter also
    numbers its first request 1. Mapped by id alone, its reject outcome is
    filed under T1 and the run fails with "T1: 2 requests, want 1"; mapped by
    (node, adapter, id) it is its own case and the showcase still matches."""
    recs, ids, origin = _load("converted.log")
    stamp = origin + 200_000_000
    recs += [
        {"event": "tf_request", "node": "/other", "adapter": 0, "id": 1,
         "stamp": stamp, "time": origin + 200_000_000, "micro": 0},
        {"event": "tf_outcome", "node": "/other", "adapter": 0, "id": 1,
         "outcome": "reject", "stamp": stamp, "time": origin + 255_000_000, "micro": 0,
         "wait_logical_ns": 55_000_000, "transform": None, "error": "extrapolation"},
    ]
    hand = tf2_acq.read_hand_table(HAND) + [("/other#0/1", "reject", 255, 0.0, 0.0)]
    assert tf2_acq.compare(recs, ids, origin, hand) is None
    assert tf2_acq.case_name(recs[-1], ids) == "/other#0/1"


def test_showcase_ids_names_the_node_of_each_case():
    """Two samplers in one log: each node keeps its own T1 for id 1."""
    log = ("[INFO] [1.0] [acq_sampler]: ACQ_START t0=5\n"
           "[INFO] [1.1] [acq_sampler]: ACQ_REQ case=T1 id=1\n"
           "[INFO] [1.1] [ns.second]: ACQ_REQ case=T9 id=1\n")
    origin, ids = tf2_acq.showcase_ids(log)
    assert origin == 5
    assert ids == {("/acq_sampler", 0, 1): "T1", ("/ns/second", 0, 1): "T9"}


def test_unavailable_interval_cell_matches_only_the_unavailable_record():
    """A reparented or zero-stamped
    path carries "unavailable" in both interval fields; the hand cell
    ``unavailable`` matches it, and a number in its place is named."""
    recs = [{"event": ev, "node": "/acq_sampler", "adapter": 0, "id": 1,
             "oldest_common": "unavailable", "latest_common": "unavailable"}
            for ev in ("tf_request", "tf_outcome")]
    ids = {("/acq_sampler", 0, 1): "T1"}
    row = ("T1", "unavailable", "unavailable", "unavailable", "unavailable")
    assert tf2_acq.compare_intervals(recs, ids, 0, [row]) is None
    recs[1]["oldest_common"] = 100_000_000
    assert tf2_acq.compare_intervals(recs, ids, 0, [row]) == \
        "T1: tf_outcome oldest_common 100.0 ms != unavailable"
    recs[1]["oldest_common"] = "unavailable"
    num = ("T1", "10", "unavailable", "unavailable", "unavailable")
    assert tf2_acq.compare_intervals(recs, ids, 0, [num]) == \
        "T1: tf_request oldest_common 'unavailable' != 10"
