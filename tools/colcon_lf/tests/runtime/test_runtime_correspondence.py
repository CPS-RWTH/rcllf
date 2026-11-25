# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Publication correspondence on real generated systems (T9).

Expected values come from the fixture source (``correspondence_fixture``:
200 ms timer, 10 ticks, ``/corr/a`` fans out to corr_sub_1 and corr_sub_2,
``/corr/b`` goes to corr_sub_1) and the default ``after 0`` delay, never from
a previous run. Evidence is the ``corr`` JSON trace lines, subscriber logs and
the reconcile summary printed at epoch close.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from collections import Counter

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

# CLI default after-delay: ``after 0``. An admitted
# in-process d = 0 edge delivers at the publication tag (t, m) itself, so a
# first hop from a timer at (t, 0) lands at microstep 0, and so does every
# later hop of a chain of such edges.
D_NS = 0
HOP_MICRO = 0
TICK_NS = 200_000_000


def _events(log: str, kind: str) -> list[dict]:
    out = []
    for m in re.findall(r'(\{"event":"corr".*\})', log):
        ev = json.loads(m)
        if ev.get("kind") == kind:
            out.append(ev)
    return out


def _reconcile(log: str) -> dict:
    found = re.findall(r'rcllf corr: (\{"event":"reconcile".*\})', log)
    assert found, "no reconcile summary:\n" + log[-2000:]
    return json.loads(found[-1])


def _edges(log: str) -> dict[tuple[str, str], int]:
    """(subscriber key, resolved topic) -> edge id, from the startup lines."""
    return {(sub, topic): int(e) for e, sub, topic in re.findall(
        r"rcllf corr: admitted edge (\d+) \S+ -> (\S+) on '([^']+)'", log)}


def _wrong_samples(log: str, ticks: list[int]) -> list[str]:
    """Per subscriber node, its /corr/a payloads must be tick 1's first, then tick order.

    Checked per node: at one tag the two nodes' dispatches may
    run, and log, in either order, so only each node's own sequence is fixed.
    """
    want = [f"t{t}k1" for t in ticks]
    heard: dict[str, list[str]] = {}
    for node, payload in re.findall(r"corr_sub: node=(corr_sub_\d) topic=/corr/a payload=(\S+)", log):
        heard.setdefault(node, []).append(payload)
    errors = []
    for node in ("corr_sub_1", "corr_sub_2"):
        got = heard.get(node, [])
        if got[:1] != want[:1]:
            errors.append(f"wrong sample: {node}'s first dispatch got payload {got[:1]}, "
                          f"expected tick 1 payload {want[0]}")
        elif got != want:
            errors.append(f"wrong sample: {node} got payloads {got}, expected {want} in tick order")
    return errors


def _run(monkeypatch, scenario: str, **env) -> rts.LaunchResult:
    case = rts.require_case("corr")
    monkeypatch.setenv("CORR_SCENARIO", scenario)
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    # Review m6: wait for the last publishing tick instead of a fixed budget.
    ticks = EXPECTED.get(scenario, EXPECTED["one"])[1]
    if not ticks:
        return rts.launch_and_capture(case, 2.6)
    last = f"corr_pub: tick={ticks[-1]} "
    return rts.launch_and_capture(case, 1.0, until=lambda log: last in log)


# scenario -> matched dispatches per (subscriber, topic), ticks that publish on /corr/a
EXPECTED = {
    "zero": ({}, []),
    "one": ({("CorrSubscriber", "/corr/a"): 10, ("CorrSubscriberA", "/corr/a"): 10}, list(range(1, 11))),
    "many": ({("CorrSubscriber", "/corr/a"): 30, ("CorrSubscriberA", "/corr/a"): 30}, list(range(1, 11))),
    "conditional": ({("CorrSubscriber", "/corr/a"): 5, ("CorrSubscriberA", "/corr/a"): 5}, [1, 3, 5, 7, 9]),
    "two_publishers": ({("CorrSubscriber", "/corr/a"): 10, ("CorrSubscriberA", "/corr/a"): 10,
                        ("CorrSubscriber", "/corr/b"): 10}, list(range(1, 11))),
    "fanout": ({("CorrSubscriber", "/corr/a"): 10, ("CorrSubscriberA", "/corr/a"): 10}, list(range(1, 11))),
    "stale": ({("CorrSubscriber", "/corr/a"): 10, ("CorrSubscriberA", "/corr/a"): 10}, list(range(1, 11))),
}


@pytest.mark.parametrize("scenario", sorted(EXPECTED))
def test_fixture_dispatches_exactly_what_was_published(monkeypatch, scenario):
    result = _run(monkeypatch, scenario)
    log = result.log
    assert result.finished_cleanly, "no clean shutdown:\n" + log[-2000:]
    edges = _edges(log)
    assert len(edges) == 3, f"expected 3 admitted edges, got {edges}"
    want, ticks = EXPECTED[scenario]
    dispatch = _events(log, "dispatch")
    got = Counter((e["edge"], e["outcome"]) for e in dispatch)
    for key, edge in edges.items():
        assert got.get((edge, "MATCHED"), 0) == want.get(key, 0), (
            f"{scenario}: edge {edge} {key}: {got}")
    assert sum(got.values()) == sum(want.values()), f"{scenario}: non-MATCHED dispatches {got}"
    per_tick = 3 if scenario == "many" else 1
    a_edge = edges[("CorrSubscriberA", "/corr/a")]
    tags = [(e["elapsed"], e["microstep"], e["ordinal"]) for e in dispatch if e["edge"] == a_edge]
    # Every record due at a tag is delivered at that tag in one
    # reaction (the callback once per record, in ordinal order), so the
    # per_tick records of `many` share the microstep instead of taking one each.
    expected_tags = [((t - 1) * TICK_NS + D_NS, HOP_MICRO) for t in ticks for _ in range(per_tick)]
    assert [(el, ms) for el, ms, _ in tags] == expected_tags, f"{scenario}: dispatch tags {tags}"
    ordinals = [o for _, _, o in tags]
    assert ordinals == sorted(ordinals) and len(set(ordinals)) == len(ordinals), ordinals
    rec = _reconcile(log)
    assert rec["reconciled"] is True and rec["retained"] == 0 and rec["carrier"] == 0, rec
    if scenario == "zero":
        assert all(e["expectations"] == 0 for e in rec["edges"]), rec
        assert '"entry":"ros2_lf_corr_drain"' not in log, "a subscriber callback ran"
    if scenario == "stale":
        errors = _wrong_samples(log, ticks)
        assert not errors, "\n".join(errors)
        # The helper sample reaches ingress only if a reader matched before it
        # was written; when it does, it is unsupported, never dispatched.
        taken_h = sum(rec["edges"][e]["samples"].get("unsupported", 0)
                      for e in (edges[("CorrSubscriber", "/corr/a")], a_edge))
        assert taken_h >= 1, f"helper sample never classified unsupported: {rec}"
        assert log.count("outside an LF reaction (no_context)") == 1


def _lines_of(log: str, node: str, payload: str) -> str:
    found = [ln for ln in log.splitlines() if f"node={node} topic=/corr/a payload={payload} " in ln]
    assert len(found) == 1, f"{node} {payload}: {found}"
    return found[0]


def test_stale_check_fails_on_a_wrong_sample_not_on_node_order():
    """Falsification for `_wrong_samples` on a recorded stale run (fixtures/README.md)."""
    log = (rts.FIXTURES_DIR / "corr_stale_real.log").read_text(encoding="utf-8")
    ticks = EXPECTED["stale"][1]
    assert _wrong_samples(log, ticks) == []

    # Every corr_sub_1 line after every corr_sub_2 line: no node's own order
    # changes, so no wrong sample, though the first two lines are t1k1, t2k1.
    b1 = _lines_of(log, "corr_sub_2", "t1k1")
    lines = log.splitlines()
    sub_1 = [ln for ln in lines if "node=corr_sub_1 topic=/corr/a " in ln]
    regrouped = "\n".join([ln for ln in lines if ln not in sub_1] + sub_1)
    assert _wrong_samples(regrouped, ticks) == []

    # corr_sub_2 handed tick 5's sample at its first dispatch.
    wrong_first = log.replace(b1, b1.replace("payload=t1k1 stamp=1", "payload=t5k1 stamp=5"))
    assert _wrong_samples(wrong_first, ticks) == [
        "wrong sample: corr_sub_2's first dispatch got payload ['t5k1'], expected tick 1 payload t1k1"]

    # corr_sub_1's ticks 3 and 4 dispatched in the wrong order.
    a3, a4 = _lines_of(log, "corr_sub_1", "t3k1"), _lines_of(log, "corr_sub_1", "t4k1")
    out_of_order = log.replace(a3, "\0").replace(a4, a3).replace("\0", a4)
    errors = _wrong_samples(out_of_order, ticks)
    assert len(errors) == 1 and errors[0].startswith(
        "wrong sample: corr_sub_1 got payloads ['t1k1', 't2k1', 't4k1', 't3k1',"), errors


@pytest.mark.parametrize("delay_ms", [0, 1, 2, 5, 10])
def test_double_signal_shutdown_releases_every_node(monkeypatch, delay_ms):
    """A second SIGINT straight to the binary during shutdown.

    The correspondence edges used to keep their subscriber nodes, and with
    them the DDS participant, alive into exit(), where Fast DDS's static
    factory destroyed the participant under its own threads. The binding now
    finalizes the rmw context before exit(), which rmw refuses while any node
    lives, and only counts late signals.
    """
    case = rts.require_case("corr")
    monkeypatch.setenv("CORR_SCENARIO", "conditional")
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    result = rts.launch_and_capture(
        case, 1.0, until=lambda log: "corr_pub: tick=9 " in log,
        second_signal_after_s=delay_ms / 1000)
    log = result.log
    assert "pure virtual method called" not in log, log[-2000:]
    assert result.finished_cleanly, "no clean shutdown:\n" + log[-2000:]
    shutdown = "\n".join(line for line in log.splitlines() if "rcllf:" in line or "rclcpp" in line)
    assert "rcllf: rmw context finalized at shutdown" in log, (
        "a node outlived the binding teardown, so the DDS participant would be "
        "destroyed inside exit() under its own threads:\n" + shutdown[-2000:])
    rec = _reconcile(log)
    assert rec["reconciled"] is True, rec
    matched = sorted(e["outcomes"].get("MATCHED", 0) for e in rec["edges"])
    assert matched == [0, 5, 5], f"MATCHED per edge {matched}, expected 5 on both /corr/a edges"


def test_restart_is_a_new_epoch(monkeypatch):
    epochs = []
    for _ in range(2):
        rec = _reconcile(_run(monkeypatch, "one").log)
        assert rec["reconciled"] is True, rec
        epochs.append(rec["epoch"])
    assert epochs[0] != epochs[1], epochs


def test_dropped_token_breaks_reconcile_and_exits_nonzero(monkeypatch):
    log = _run(monkeypatch, "one", RCLLF_TEST_CORR_DROP_TOKEN="2").log
    assert re.search(r"missing outcome: expectation \([0-9a-f]{32}, 2, edge \d\) has no terminal outcome", log), (
        log[-2000:])
    assert "exit code 3" in log, log[-1500:]


def test_dropped_sample_on_a_local_edge_is_lost_at_publish(monkeypatch):
    # A local edge is decided when its publish returns, so a
    # sample the reader never got is LOST (local_undelivered) right away; it
    # no longer waits for a cutoff and times out.
    log = _run(monkeypatch, "one", RCLLF_TEST_CORR_DROP_SAMPLE="2").log
    one = _events(log, "dispatch")
    bad = [(e["ordinal"], e["outcome"]) for e in one if e["outcome"] != "MATCHED"]
    assert bad == [(2, "LOST")], f"ordinal 2 outcome, expected LOST at publish: {bad}"
    # The loss is said when decided (console only, round 2) and counted
    # beside the accounting verdict, which it does not fail.
    assert "the sample never reached its reaction and is missing" in log, log[-2000:]
    rec = _reconcile(log)
    assert rec["reconciled"] is True and rec["missing_total"] == 1, rec
    many = _events(_run(monkeypatch, "many", RCLLF_TEST_CORR_DROP_SAMPLE="2").log, "dispatch")
    bad = [(e["ordinal"], e["outcome"]) for e in many if e["outcome"] != "MATCHED"]
    assert bad == [(2, "LOST")], f"ordinal 2 outcome, expected LOST at publish: {bad}"


def test_async_publication_mode_refuses_startup(monkeypatch):
    case = rts.require_case("corr")
    monkeypatch.setenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS")
    with pytest.raises(AssertionError, match="exited on its own") as info:
        rts.launch_and_capture(case, 3.0)
    assert "check publication_mode" in str(info.value) and "rc=" in str(info.value)


def test_pubsub_correspondence_matched_and_timing_parity(monkeypatch):
    """Case 1 and 10: every publish MATCHED with its own value, first dispatch at t+d, microstep 0."""
    case = rts.require_case("pubsub")
    monkeypatch.setenv("RCLLF_TRACE_CALLBACKS", "1")
    log = rts.launch_and_capture(case, 4.0).log
    emit, dispatch = _events(log, "emit"), _events(log, "dispatch")
    assert emit and dispatch, log[-2000:]
    assert all(e["outcome"] == "MATCHED" for e in dispatch), dispatch
    assert (dispatch[0]["elapsed"] - emit[0]["elapsed"], dispatch[0]["microstep"]) == (D_NS, HOP_MICRO)
    pubs = re.findall(r"Publishing: temp=([\d.]+)", log)
    heard = re.findall(r"I heard: temp=([\d.]+)", log)
    assert heard and heard == pubs[:len(heard)], (pubs, heard)
    assert _reconcile(log)["reconciled"] is True


# A missing message runs the deadline callback and the
# message_lost callback, each one registered, once, in the same reaction, with
# the same logical count. Expected values from the fixture source: `one`
# publishes ordinal t on /corr/a at tick t, elapsed (t - 1) * 200 ms; the
# drain runs at d = 0, microstep 0. Dropping ordinal 2 on corr_sub_1's
# edge makes ordinal 2 missing at tag (200 ms, 0), the first missing message
# of that subscription, so total 1 and change 1.
DROP_ORDINAL = 2
DROP_TAG = ((DROP_ORDINAL - 1) * TICK_NS + D_NS, HOP_MICRO)
_HOOK_LINE = re.compile(r"\[(corr_sub_\d)\]: (MISSING|LOST) sub=\S+ total=(\d+) change=(\d+)")


def _missing_run(monkeypatch, **env) -> tuple[str, dict, list[dict], list[tuple[str, str, int, int]]]:
    log = _run(monkeypatch, "one", RCLLF_TEST_CORR_DROP_SAMPLE=str(DROP_ORDINAL),
               ROS2_LF_TAKE_TIMEOUT_MS="100", **env).log
    edge = _edges(log)[("CorrSubscriber", "/corr/a")]
    odd = [e for e in _events(log, "dispatch") if e["outcome"] != "MATCHED"]
    assert [(e["edge"], e["ordinal"]) for e in odd] == [(edge, DROP_ORDINAL)], (
        f"expected one missing dispatch (edge {edge}, ordinal {DROP_ORDINAL}): {odd}")
    d = odd[0]
    assert (d["elapsed"], d["microstep"]) == DROP_TAG, (
        f"missing message delivered at {(d['elapsed'], d['microstep'])}, expected {DROP_TAG}")
    inv = _events(log, "missing_hook")
    calls = [(n, k, int(t), int(c)) for n, k, t, c in _HOOK_LINE.findall(log)]
    return log, d, inv, calls


@pytest.mark.parametrize("deadline,lost", [(True, True), (True, False), (False, True)])
def test_missing_message_runs_each_registered_hook_once(monkeypatch, deadline, lost):
    env = {"CORR_DEADLINE_HOOK": "1" if deadline else "0",
           "CORR_MESSAGE_LOST_HOOK": "1" if lost else "0"}
    _, d, inv, calls = _missing_run(monkeypatch, **env)
    want = (["deadline"] if deadline else []) + (["message_lost"] if lost else [])
    want_lines = ([("corr_sub_1", "MISSING", 1, 1)] if deadline else []) + (
        [("corr_sub_1", "LOST", 1, 1)] if lost else [])
    assert calls == want_lines, f"user hook calls {calls}, expected {want_lines}"
    # Both run inside the one dispatch at DROP_TAG (checked in _missing_run).
    assert d["missing_hooks"] == want, f"dispatch missing_hooks {d['missing_hooks']}, expected {want}"
    assert d["missing_hook"] == want[0], f"dispatch missing_hook {d['missing_hook']}, expected {want[0]}"
    assert [(e["ordinal"], e["total"], e["hooks"]) for e in inv] == [(DROP_ORDINAL, 1, want)], inv


def test_missing_message_without_hooks_runs_nothing(monkeypatch):
    _, d, inv, calls = _missing_run(monkeypatch, CORR_DEADLINE_HOOK="0", CORR_MESSAGE_LOST_HOOK="0")
    assert calls == [], f"a hook ran with none registered: {calls}"
    assert (d["missing_hook"], d["missing_hooks"]) == ("none", []), d
    assert inv == [], f"hook invocation records with none registered: {inv}"


# A best-effort publisher that floods /corr/lost_probe from another process
# after discovery, ending before the last tick. With asynchronous publication
# the depth-1 writer history overwrites 100 kB samples before they are sent, so
# the best-effort reader sees sequence gaps and Fast DDS reports SAMPLE_LOST
# (found with feasibility probes).
_BURST = """
import time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
rclpy.init()
node = rclpy.create_node("w3_lost_burst")
pub = node.create_publisher(Header, "/corr/lost_probe", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
time.sleep(0.5)
msg = Header(frame_id="x" * 100000)
for _ in range(3000):
    pub.publish(msg)
print("burst done", flush=True)
time.sleep(1.0)
"""


def test_physical_sample_lost_never_reaches_the_hook(monkeypatch):
    """A real Fast DDS SAMPLE_LOST on a best-effort probe subscription that registers the hooks: none run for it."""
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "79")
    env["RMW_FASTRTPS_PUBLICATION_MODE"] = "ASYNCHRONOUS"
    burst = subprocess.Popen([sys.executable, "-c", _BURST], env=env, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True)
    try:
        log, d, _, calls = _missing_run(
            monkeypatch, CORR_DEADLINE_HOOK="1", CORR_MESSAGE_LOST_HOOK="1")
    finally:
        burst_out = burst.communicate(timeout=30)[0]
    assert "burst done" in burst_out, burst_out
    physical = [int(t) for t in re.findall(r"PHYSICAL lost sub=corr_sub_1 total=(\d+)", log)]
    assert physical and physical[-1] > 0, f"DDS reported no physical SAMPLE_LOST: {physical}"
    assert calls == [("corr_sub_1", "MISSING", 1, 1), ("corr_sub_1", "LOST", 1, 1)], (
        f"physical events reached a hook: {calls}")
    assert d["missing_hooks"] == ["deadline", "message_lost"], d


_MS = 1_000_000
_REFUSED = re.compile(r'rcllf corr: (\{"event":"schedule_refused".*\})')


def _assert_no_refused_wake(log: str) -> None:
    """No lf_schedule of a drain wake was refused.

    The record wake and the logical deadline window's wake schedule one drain
    action; a second event at a tag that already holds one is refused by
    reactor-c, and the refusal terminalizes the edge's carrier.
    """
    refused = _REFUSED.findall(log)
    assert not refused, (
        f"{len(refused)} drain wake(s) refused, the deadline window's wake collided with a "
        f"queued drain event; first: {refused[:2]}")


def _silence_ends(log: str, edge: int) -> list[int]:
    """Window ends (elapsed ns) of the deadline calls on `edge`, each run at its end tag."""
    windows = [e for e in _events(log, "silence_deadline") if e["edge"] == edge]
    off = [(e["window_end_elapsed"], e["elapsed"], e["microstep"]) for e in windows
           if (e["elapsed"], e["microstep"]) != (e["window_end_elapsed"], 0)]
    assert not off, f"deadline calls not at their window end (end, elapsed, microstep): {off}"
    return [e["window_end_elapsed"] for e in windows]


def test_physical_deadline_events_never_reach_the_hooks(monkeypatch):
    """A 20 ms QoS deadline against a 200 ms timer: DDS counts physical misses, the hooks
    see only the logical ones, one per 20 ms window without a record and one for
    the dropped message.

    Expected values from the fixture source: corr_sub_1's edge carries a record at
    200 k ms for k = 0..9 (the dropped ordinal 2 at 200 ms is a missing record, which
    restarts the window too). A window ends at every 20 ms after the last record, so each
    200 ms gap holds nine that end before the next record, at 200 k + 20 j ms (j = 1..9),
    and after the last record at 1800 ms they go on at 1800 + 20 j ms until the stop.
    """
    log, d, _, calls = _missing_run(
        monkeypatch, CORR_DEADLINE_HOOK="1", CORR_MESSAGE_LOST_HOOK="1", CORR_PHYSICAL_DEADLINE_MS="20")
    physical = [int(t) for t in re.findall(r"PHYSICAL deadline sub=corr_sub_1 total=(-?\d+)", log)]
    # 9 gaps of 200 ms between the 10 ticks, each at least one missed 20 ms period.
    assert physical and physical[-1] >= 9, f"DDS counted no physical deadline misses: {physical}"
    _assert_no_refused_wake(log)
    ends = _silence_ends(log, d["edge"])
    gaps = [(200 * k + 20 * j) * _MS for k in range(9) for j in range(1, 10)]
    assert ends[:len(gaps)] == gaps, f"deadline windows between the records: {ends[:len(gaps)]}"
    tail = ends[len(gaps):]
    assert tail == [(1800 + 20 * j) * _MS for j in range(1, len(tail) + 1)], f"after the last record: {tail}"
    # The drop at 200 ms follows the nine windows of the first gap; totals count both.
    mine = [c for c in calls if c[0] == "corr_sub_1"]
    missing = [c for c in mine if c[1] == "MISSING"]
    assert missing == [("corr_sub_1", "MISSING", n, 1) for n in range(1, len(ends) + 2)], (
        f"hook calls are not the {len(ends)} logical windows plus the drop: {missing[:12]}")
    assert [c for c in mine if c[1] == "LOST"] == [("corr_sub_1", "LOST", 1, 1)], mine
    assert d["missing_hooks"] == ["deadline", "message_lost"], d


def test_deadline_window_wake_never_collides_with_a_queued_drain(monkeypatch):
    """A 300 ms deadline against a 200 ms timer runs no window before the last
    record and refuses no drain wake.

    The startup drain schedules the window's end at 300 ms. The record at 200 ms moves
    the end to 500 ms and schedules that, so the drain at 300 ms finds no window ended
    and wants 500 ms again, an event that is already queued: reactor-c refuses it and the
    carrier is terminalized, unless the binding claims each wake once. Expected values:
    records every 200 ms restart the window before it ends, so the first window ends 300
    ms after the last record (1800 ms), then every 300 ms until the stop.
    """
    log, d, _, calls = _missing_run(
        monkeypatch, CORR_DEADLINE_HOOK="1", CORR_MESSAGE_LOST_HOOK="1", CORR_PHYSICAL_DEADLINE_MS="300")
    _assert_no_refused_wake(log)
    ends = _silence_ends(log, d["edge"])
    assert ends == [(1800 + 300 * j) * _MS for j in range(1, len(ends) + 1)], (
        f"deadline windows: {ends}, expected none before the last record at 1800 ms")
    mine = [c for c in calls if c[0] == "corr_sub_1"]
    assert mine == [("corr_sub_1", "MISSING", 1, 1), ("corr_sub_1", "LOST", 1, 1)] + [
        ("corr_sub_1", "MISSING", n, 1) for n in range(2, len(ends) + 2)], mine


def test_deadline_callback_that_shuts_down_stops_message_lost(monkeypatch):
    """No user callback after shutdown has begun, even within one missing message."""
    case = rts.require_case("corr")
    for k, v in {"CORR_SCENARIO": "one", "RCLLF_TRACE_CALLBACKS": "1", "CORR_DEADLINE_HOOK": "1",
                 "CORR_MESSAGE_LOST_HOOK": "1", "CORR_DEADLINE_SHUTDOWN": "1",
                 "RCLLF_TEST_CORR_DROP_SAMPLE": str(DROP_ORDINAL), "ROS2_LF_TAKE_TIMEOUT_MS": "100"}.items():
        monkeypatch.setenv(k, v)
    log = rts.launch_and_capture(
        case, 1.0, allow_exit=True, until=lambda text: "calls rclcpp::shutdown" in text, settle_s=2.0).log
    assert "MISSING sub=corr_sub_1 calls rclcpp::shutdown" in log, log[-2000:]
    calls = [(n, k, int(t), int(c)) for n, k, t, c in _HOOK_LINE.findall(log)]
    assert calls == [("corr_sub_1", "MISSING", 1, 1)], f"a hook ran after shutdown began: {calls}"
    inv = _events(log, "missing_hook")
    assert [(e["ordinal"], e["hooks"]) for e in inv] == [(DROP_ORDINAL, ["deadline"])], (
        f"trace must list only the hook that ran: {inv}")
    d = [e for e in _events(log, "dispatch") if e["ordinal"] == DROP_ORDINAL and e["outcome"] != "MATCHED"]
    assert [(e["elapsed"], e["microstep"], e["missing_hook"], e["missing_hooks"]) for e in d] == [
        (*DROP_TAG, "deadline", ["deadline"])], d
