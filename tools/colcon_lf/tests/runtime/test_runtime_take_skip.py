# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Take mode skip drops a late sample by tag and never delivers it late.

``take_skip_fixture``: ``take_writer_node`` publishes ``id_base + tick`` on
/take every 100 ms from tick 5 on, in a loop, so the edge stays legacy
(``burst_unbounded``) and the subscriber's reaction takes through
``take_by_tag`` with the default skip. ``take_reader_node`` logs each sample
with its tag and each ``message_lost`` report. ``take_one`` has one writer,
``take_fan`` two (``take_writer_a`` ids from 0, ``take_writer_b`` ids from
1000) fanned in to one take port. Writers and reader run in one process, so
every write is in the tag log. The ``across_federates`` cases run ``take_one``
with writer and reader in two federates: the link carries
the writer's tag log events, and the expectations are the same.

Faults are injected by the writer named ``TAKE_SKIP_INJECT_NODE``, never by
sleep luck: an ID in ``TAKE_SKIP_LATE_IDS`` is published 50 ms after its tick
from another thread, half a period clear of both its own take (at the tick)
and the next one; an ID in ``TAKE_SKIP_LOST_IDS`` is never published. Both
still count as a write at their tick (``ros2_lf_claim_publish``), and the late
publish fills that claim.

A take that knows its trigger counted a write waits for that sample up to the
subscription's ``take_timeout_ms`` before it declares the miss, in real time
as in fast mode. The late publish at 50 ms is late by that
rule only when the bound is below it, so the cases run with
``ROS2_LF_TAKE_TIMEOUT_MS`` at ``TIMEOUT_MS`` (10 ms: the wait ends 40 ms
before the late publish, which lands 50 ms before the next take; an on-time
sample has 10 ms to reach the reader, where the no-wait rule gave it none). The
``late_in_time`` case keeps the default 100 ms bound: the same late publish is
then inside the wait and delivered at its own tag, nothing is skipped or
dropped, and the take is counted as ``waited``. Across federates that case
expects what ``late_next_on_time`` expects: the fill of a claimed sample
published late comes with the writer's next link message, so the reader
holds the sample it has in hand until then and drops it by its record
(docs/reference/troubleshooting.md, "Across federates"); the wait ends at
the bound without a delivery.

The expectations below were derived by hand from the drop-by-tag rule in
docs/reference/troubleshooting.md before the first run. Tag k is the tag of
writer tick k under the default ``after 0``. A take delivers the samples of its
own tag; a sample of an earlier tag is dropped when it arrives
(``late_dropped``, ``message_lost``); a lost sample costs nothing.

* ``late_next_on_time``: 10 is late. Tag 10 skips. Tag 11 holds 10 and 11: 10
  is dropped and reported, 11 delivered.
* ``lone_late``: 10 and 11 are late. Tag 10 skips. Tag 11 holds only 10: it is
  dropped and reported and tag 11 skips. Tag 12 holds 11 and 12: 11 is dropped
  and reported, 12 delivered. The miss and drop WARNs fire once each.
* ``late_in_time``: 10 is late, the bound is 100 ms. Tag 10 waits for 10 and
  delivers it; nothing is skipped or dropped and no WARN fires.
* ``lost``: 10 is never published. Tag 10 skips; every later tag delivers its
  own sample and nothing is dropped.
* ``fan_late``: 1010 (take_writer_b) is late. Tag 10 has two writes: the first
  take gets 10, the second finds nothing and skips. Tag 11 holds 1010, 11 and
  1011: 1010 is dropped and reported, 11 and 1011 delivered.
* ``control_one`` / ``control_fan``: no fault, every tag delivers its own
  samples, nothing skipped or dropped.
* ``burst_overflow``: tick 10 publishes 15 copies of 10 into the
  reader's KeepLast 10 history, in one process, so the reader keeps the newest
  10 and the first 5 are replaced before the take. Tag 10 has 15 writes: the
  first take finds the gap and reports the 5 replaced samples lost at tag 10
  (``reader_lost``, one gap WARN, ``message_lost`` change 5); ten takes
  deliver 10; the 5 takes of the replaced samples end at once, counted as
  skipped, with no poll to the take timeout (``poll_timeouts`` 0) and no miss
  WARN. Nothing is dropped as late. Before the fix each of those 5 takes
  polled to ``take_timeout_ms`` (5 x 100 ms in fast mode, one bound in real
  time) and the first raised the miss WARN. The case keeps the default 100 ms
  bound so a poll would show in ``poll_timeout_us``.

No case may fall back to the count rule or deliver an untagged sample. Each
case then checks the key properties: no sample delivered at a later tag than
its own, and no silent drop (every drop is in ``late_dropped`` and reported
to ``message_lost``).
"""

from __future__ import annotations

import os
import re
from pathlib import Path

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

D_NS = 0  # CLI default after-delay: ``after 0``
FIRST, INJECT, LAST = 5, 10, 20
FAN_B = 1000
TIMEOUT_MS = 10  # the take bound of every case but late_in_time

# Per case: launch case, fault env, the tags whose delivery differs from the
# tag's own samples, the message_lost reports as (tag, change), and the
# shutdown counters and one-shot WARN counts. ``timeout_ms`` overrides
# TIMEOUT_MS; ``waited`` is the least number of takes that waited.
EXPECTED = {
    "late_next_on_time": dict(
        case="take_one", env={"TAKE_SKIP_LATE_IDS": "10"},
        delivered={10: [], 11: [11]}, lost=[(11, 1)],
        skipped=1, late_dropped=1, miss=1, drop=1),
    "lone_late": dict(
        case="take_one", env={"TAKE_SKIP_LATE_IDS": "10,11"},
        delivered={10: [], 11: [], 12: [12]}, lost=[(11, 1), (12, 1)],
        skipped=2, late_dropped=2, miss=1, drop=1),
    "late_in_time": dict(
        case="take_one", env={"TAKE_SKIP_LATE_IDS": "10"}, timeout_ms=100,
        delivered={}, lost=[],
        skipped=0, late_dropped=0, miss=0, drop=0, waited=1),
    "lost": dict(
        case="take_one", env={"TAKE_SKIP_LOST_IDS": "10"},
        delivered={10: []}, lost=[],
        skipped=1, late_dropped=0, miss=1, drop=0),
    "fan_late": dict(
        case="take_fan", env={"TAKE_SKIP_LATE_IDS": "1010"}, inject="take_writer_b",
        delivered={10: [10], 11: [11, 1011]}, lost=[(11, 1)],
        skipped=1, late_dropped=1, miss=1, drop=1),
    "control_one": dict(
        case="take_one", env={}, delivered={}, lost=[],
        skipped=0, late_dropped=0, miss=0, drop=0),
    "control_fan": dict(
        case="take_fan", env={}, delivered={}, lost=[],
        skipped=0, late_dropped=0, miss=0, drop=0),
    "burst_overflow": dict(
        case="take_one", env={"TAKE_SKIP_BURST_IDS": "10", "TAKE_SKIP_BURST_N": "15"},
        timeout_ms=100, delivered={10: [10] * 10}, lost=[(10, 5)],
        skipped=5, late_dropped=0, miss=0, drop=0, gap=1, poll_timeouts=0),
}

# One-shot WARNs; the last two belong to the count rule and must never fire
# in one process: the fallback, and its lone-late delivery.
WARNS = {
    "miss": "no sample for a trigger",
    "drop": "dropped a late sample",
    "fallback": "the count rule applies",
    "untagged": "published outside any reaction",
    "lone": "cannot be told from an on-time one",
    "gap": "never reached a take",
}
for _exp in EXPECTED.values():
    _exp.update(fallback=0, untagged=0, lone=0)
    _exp.setdefault("gap", 0)

_TICK = re.compile(r"\[(take_writer(?:_a)?)\]: TSW tick=(\d+) t=(\d+)")
_SAMPLE = re.compile(r"\[take_reader\]: TSR t=(\d+) v=(\d+)")
_LOST = re.compile(r"\[take_reader\]: TSR lost t=(\d+) change=(\d+) total=(\d+)")
_STATS = re.compile(r"ros2_lf_take_stats: reason=shutdown .*")


def _own(case: str, tag: int) -> list[int]:
    return [tag, FAN_B + tag] if case == "take_fan" else [tag]


def parse(log: str, d_ns: int = D_NS) -> dict:
    """Deliveries and lost reports per tag index, shutdown counters, WARN counts."""
    tick_of = {int(t): int(k) for _, k, t in _TICK.findall(log)}

    def tag(t: str) -> int:
        k = tick_of.get(int(t) - d_ns)
        assert k is not None, f"reader time {t} is no writer tick + {d_ns} ns\n{log[-3000:]}"
        return k

    delivered: dict[int, list[int]] = {}
    for t, v in _SAMPLE.findall(log):
        delivered.setdefault(tag(t), []).append(int(v))
    # The reader's line: across federates the writer's federate prints its own.
    reader = "\n".join(line for line in log.splitlines() if "federate__take_writer" not in line)
    stats = _STATS.findall(reader)
    assert stats, f"no shutdown take_stats line\n{log[-3000:]}"
    skipped = re.search(r"sub_skipped=(\d+)", stats[-1])
    waited = re.search(r"sub_waited=(\d+)", stats[-1])
    dropped = re.search(r"late_dropped=(\d+)", stats[-1])
    reader_lost = re.search(r"reader_lost=(\d+)", stats[-1])
    poll_timeouts = re.search(r"poll_timeouts=(\d+)", stats[-1])
    return {
        "delivered": {k: sorted(v) for k, v in delivered.items()},
        "lost": [(tag(t), int(c)) for t, c, _ in _LOST.findall(log)],
        "skipped": int(skipped.group(1)),
        "waited": int(waited.group(1)) if waited else 0,
        "late_dropped": int(dropped.group(1)) if dropped else 0,
        "reader_lost": int(reader_lost.group(1)) if reader_lost else 0,
        "poll_timeouts": int(poll_timeouts.group(1)) if poll_timeouts else 0,
        **{key: log.count(text) for key, text in WARNS.items()},
    }


def _save(name: str, log: str) -> None:
    out = os.environ.get("RCLLF_TAKE_SKIP_LOG_DIR")
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        runs = len(list(Path(out).glob(f"{name}_r*.log")))
        (Path(out) / f"{name}_r{runs + 1}.log").write_text(log, encoding="utf-8")


# take_one with writer and reader in two federates
# (FEDERATE=per-node). The link carries the writer's tag log events with the
# count, so every case expects what it expects in one process. The link runs
# after the federated default delay,
# ``after 0``, so tag k is tick k (one microstep later).
XFED_D_NS = 0
# burst_overflow stays in one process: across federates the writer's own
# reliable KeepLast 10 history overflows too, and which samples reach the
# reader is the middleware's choice, not the reader replacement.
XFED = sorted(n for n, e in EXPECTED.items() if e["case"] == "take_one" and n != "burst_overflow")
# The claimed late sample's fill comes with the writer's next link message
# (see the module docstring): under the default bound it is held and dropped.
XFED_EXPECTED = {
    **EXPECTED,
    "late_in_time": dict(EXPECTED["late_next_on_time"], timeout_ms=100, waited=0),
}


def _federated_take_one() -> rts.RuntimeCase:
    install = rts._PACKAGES_ROOT / "install"
    pkg = "lf_generated_take_skip_fixture_takeone_federated"
    launch = sorted((install / pkg / "share" / pkg / "launch").glob("*.launch.py"))
    if not launch:
        if os.environ.get("RCLLF_REQUIRE_RUNTIME_SMOKE") == "1" and \
                os.environ.get("RCLLF_TAKE_SKIP_FEDERATED") == "1":
            pytest.fail(f"{pkg} not built (FEDERATE=per-node LF_LAUNCH=take_one "
                        "docker/lf_build.sh take_skip_fixture)")
        pytest.skip(f"{pkg} not built (FEDERATE=per-node LF_LAUNCH=take_one "
                    "docker/lf_build.sh take_skip_fixture)")
    return rts.RuntimeCase(source_pkg="take_skip_fixture", launch_stem="take_one", gen_pkg=pkg,
                           launch_file=launch[0].name)


@pytest.mark.parametrize("name", XFED)
def test_take_skip_drops_late_samples_by_tag_across_federates(monkeypatch, name):
    _check(monkeypatch, name, XFED_EXPECTED[name], _federated_take_one(), XFED_D_NS,
           f"xfed_{name}")


@pytest.mark.parametrize("name", sorted(EXPECTED))
def test_take_skip_drops_late_samples_by_tag(monkeypatch, name):
    exp = EXPECTED[name]
    _check(monkeypatch, name, exp, rts.require_case(exp["case"]), D_NS, name)


def _check(monkeypatch, name: str, exp: dict, case: rts.RuntimeCase, d_ns: int,
           label: str) -> None:
    monkeypatch.setenv("TAKE_SKIP_INJECT_NODE", exp.get("inject", "take_writer"))
    for key in ("TAKE_SKIP_LATE_IDS", "TAKE_SKIP_LOST_IDS", "TAKE_SKIP_BURST_IDS",
                "TAKE_SKIP_BURST_N"):
        monkeypatch.setenv(key, exp["env"].get(key, ""))
    monkeypatch.setenv("ROS2_LF_TAKE_TIMEOUT_MS", str(exp.get("timeout_ms", TIMEOUT_MS)))
    last = _own(exp["case"], LAST)
    result = rts.launch_and_capture(
        case, 2.0, until=lambda log: all(f"v={v}\n" in log for v in last))
    _save(label, result.log)
    assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
    assert "admitted edge" not in result.log, "the /take edge must stay legacy"
    got = parse(result.log, d_ns)
    want_delivered = {k: sorted(exp["delivered"].get(k, _own(exp["case"], k)))
                      for k in range(FIRST, LAST + 1)}
    got_delivered = {k: got["delivered"].get(k, []) for k in range(FIRST, LAST + 1)}
    assert got_delivered == want_delivered, (
        f"{name}: delivered per tag\n got {got_delivered}\nwant {want_delivered}")
    assert not [k for k in got["delivered"] if k < FIRST], f"{name}: delivery before tick {FIRST}"
    assert got["lost"] == exp["lost"], f"{name}: message_lost reports {got['lost']}"
    counts = {key: got[key] for key in ("skipped", "late_dropped", *WARNS)}
    want_counts = {key: exp[key] for key in counts}
    assert counts == want_counts, f"{name}: counters {counts}, want {want_counts}"
    assert got["waited"] >= exp.get("waited", 0), f"{name}: waited {got['waited']} takes"
    if "poll_timeouts" in exp:
        assert got["poll_timeouts"] == exp["poll_timeouts"], (
            f"{name}: {got['poll_timeouts']} take(s) polled to the timeout")
    # No silent drop: every drop and every replaced sample is reported to
    # message_lost, change by change.
    assert sum(c for _, c in got["lost"]) == got["late_dropped"] + got["reader_lost"]
