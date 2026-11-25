# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The bna_example schedule against its hand table, and the trace checker."""

from __future__ import annotations

import pytest

from lf_eval.bna import oracle, schedule as s

CLOSE_MS = 2000


def _with_close(drop=None, mode="single", stamp_lag=None):
    return s.inputs(drop, mode, stamp_lag) + [{"ev": "close", "sync": 0, "reason": "shutdown",
                                         "tag": [CLOSE_MS * s.MS, 0]}]


@pytest.mark.parametrize("drop, mode, hand", [(None, "single", s.HAND),
                                               (s.DROP, "single", s.HAND_DROP),
                                               (None, "federated", s.HAND_FEDERATED),
                                               (s.DROP, "federated", s.HAND_FEDERATED_DROP),
                                               (None, "decentralized", s.HAND_DECENTRALIZED),
                                               (s.DROP, "decentralized",
                                                s.HAND_DECENTRALIZED_DROP)])
def test_oracle_reproduces_the_hand_table(drop, mode, hand):
    assert s.table(oracle.run(_with_close(drop, mode))) == hand


def _fake_trace(drop=None, start=5_000_000_000, epoch=7, wakes=True, mode="single",
                stamp_lag=None):
    """A trace as the runtime would write it: absolute times, real epoch and gids."""
    gid = {"anchor": "01.0f.aa", "candidate": "01.0f.bb"}
    events = _with_close(drop, mode, stamp_lag)
    outs = oracle.run(events)
    if wakes:
        in_tags = {tuple(r["tag"]) for r in events if "tag" in r}
        events = events + [{"ev": "wake", "sync": 0, "tag": r["tag"]} for r in outs
                           if tuple(r["tag"]) not in in_tags]
    recs = []
    for r in events + outs:
        r = dict(r)
        if "tag" in r:
            r["tag"] = [r["tag"][0] + start, r["tag"][1]]
        for f in ("key", "anchor", "partner"):
            if r.get(f):
                _, name, n = oracle.split_key(r[f])
                r[f] = f"{epoch}:{gid[name]}:{n}"
        for f in ("stamp", "partner_stamp"):
            if r.get(f) is not None:
                r[f] += start
        if r["ev"] == "params":
            r["epoch"] = epoch
        recs.append(r)
    return sorted(recs, key=lambda r: (r.get("tag", [0, 0]), r["ev"] not in oracle.INPUT_EVENTS))


@pytest.mark.parametrize("drop", [None, s.DROP])
@pytest.mark.parametrize("mode", sorted(s.MODES))
def test_check_accepts_the_expected_trace(drop, mode):
    assert s.check(_fake_trace(drop, mode=mode), drop, mode) is None


def test_the_federated_program_is_the_one_process_program():
    # At d = 0 the per-node layout admits every sample at the
    # tag one process does, so a one-process trace passes the federated check.
    assert s.inputs(mode="federated") == s.inputs(mode="single")
    assert s.check(_fake_trace(), None, "federated") is None


def test_check_reports_a_missing_wake():
    report = s.check(_fake_trace(wakes=False))
    assert report == "missing_wake: outcome A5 REJECTED(timeout) at [700000000, 0] has no wake event"


def test_check_reports_a_wrong_outcome():
    trace = _fake_trace()
    for r in trace:
        if r["ev"] == "decision" and r["outcome"] == "MATCHED" and r["anchor"].endswith(":4"):
            r["partner"] = r["partner"].replace(":4", ":5")
    assert s.check(trace).startswith("decision_mismatch at anchor 0:anchor:4")


def test_outcomes_do_not_depend_on_start_epoch_or_gids():
    a = s.outcomes(_fake_trace(start=5_000_000_000, epoch=7))
    b = s.outcomes(_fake_trace(start=9_000_000_000, epoch=8))
    assert a == b and a.count("\n") == len(s.HAND)


def test_replay_report_compares_replayed_outputs_with_the_trace():
    trace = _fake_trace()
    assert s.replay_report(trace, trace) is None
    last = max((r for r in trace if r["ev"] == "sample"), key=lambda r: r["tag"])
    short = [r for r in trace if r is not last]
    assert s.replay_report(trace, short) == (
        f"outcome_mismatch at sample {last['key']}: oracle EXPIRED, implementation none")


@pytest.mark.parametrize("mode, hand", [("single", s.HAND_RAISE),
                                        ("federated", s.HAND_FEDERATED_RAISE),
                                        ("decentralized", s.HAND_DECENTRALIZED_RAISE)])
def test_oracle_reproduces_the_raised_hand_table(mode, hand):
    # max_stamp_lag 0 on the candidate input decides A9 at the
    # loss report (900 ms, 1) instead of at C11's admission (1000 ms, 1).
    assert s.table(oracle.run(_with_close(s.DROP, mode, stamp_lag=0))) == hand
    assert ("D", "A9", "MATCHED C9", 1000, 1) in s.HAND_DROP
    assert ("D", "A9", "MATCHED C9", 900, 1) in hand


def test_check_tells_a_raised_trace_from_a_plain_drop():
    raised = _fake_trace(s.DROP, stamp_lag=0)
    assert s.check(raised, s.DROP, "single", 0) is None
    # Checked as a plain drop the stamped report and the early decision differ.
    assert s.check(raised, s.DROP, "single") is not None
    assert s.check(_fake_trace(s.DROP), s.DROP, "single", 0) is not None


def test_decentralized_burst_shares_the_first_microstep():
    # Decentralized per-node admits both samples of tick 4 (C6, C7)
    # at (400, 1); only C7's LATE verdict moves, from (400, 2) to (400, 1).
    dec = [r for r in s.inputs(mode="decentralized") if r.get("key", "").endswith(":7")]
    assert [r["tag"] for r in dec if r["input"] == 1] == [[400 * s.MS, 1]]
    assert ("S", "C7", "LATE", 400, 2) in s.HAND
    assert ("S", "C7", "LATE", 400, 1) in s.HAND_DECENTRALIZED
    assert [r for r in s.HAND if r[1] != "C7"] == [r for r in s.HAND_DECENTRALIZED if r[1] != "C7"]
    assert s.check(_fake_trace(mode="decentralized"), None, "decentralized") is None
    assert s.check(_fake_trace(mode="decentralized"), None, "federated") is not None
