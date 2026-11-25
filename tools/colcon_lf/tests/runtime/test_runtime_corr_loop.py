# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Zero-delay feedback loops through correspondence edges.

Owner ruling 2026-11-05, after Edward Lee: "LF allows zero-delay feedback
loops as long as there is no causality loop."

``corr_loop`` (store-only loop, admitted): corr_ping's 200 ms timer publishes
``ping<N>`` on /corr/ping, corr_pong answers it with ``pong-ping<N>`` on
/corr/pong, and corr_ping's /corr/pong callback only logs. It is declared
after the timer (``timers_first``, the default), so it reaches no output at
its tag: no causality loop. Both links are d = 0 in one process, so both
deliver at the publication tag, no microstep. Expected per tick
N, at T = (N - 1) * 200 ms of elapsed logical time::

    ping sent  payload=ping<N>        (T, 0)   timer, emit on edge /corr/ping
    ping heard payload=ping<N>        (T, 0)   corr_pong, emit on /corr/pong
    pong heard payload=pong-ping<N>   (T, 0)   corr_ping, logs only

``corr_loop_echo`` (causality loop) is refused by ``colcon lf-generate``
(generation test ``test_causality_loop_is_refused_with_the_links_named``).
``corr_loop_echo_delayed`` is that loop with the /corr/pong link at d = 1 ms
(``lf/corr_loop_echo_delayed.execution.yaml``). Expected per tick N::

    ping sent  payload=ping<N>        (T, 0)         timer
    ping heard payload=ping<N>        (T, 0)         corr_pong, d = 0
    pong heard payload=pong-ping<N>   (T + 1 ms, 0)  corr_ping_echo, d = 1 ms
    ping sent  payload=ping<N>+       (T + 1 ms, 0)  same callback
    ping heard payload=ping<N>+       (T + 1 ms, 0)  corr_pong, logs only

The general rule both runs are checked against, from the ``corr`` JSON trace
(``RCLLF_TRACE_CALLBACKS=1``): a record published at (t, m) on a link with
delay d is dispatched at (t, m) when d = 0 and at (t + d, 0) when d > 0.
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

TICK_NS = 200_000_000
_LOGS: dict[str, str] = {}


def corr_events(log: str, kind: str) -> list[dict]:
    out = []
    for m in re.findall(r'(\{"event":"corr".*\})', log):
        ev = json.loads(m)
        if ev.get("kind") == kind:
            out.append(ev)
    return out


def admitted(log: str) -> dict[int, tuple[str, str, str, int]]:
    """edge id -> (publisher class, subscriber class, topic, d in ns), from the startup lines."""
    return {int(e): (pub, sub, topic, int(d)) for e, pub, sub, topic, d in re.findall(
        r"rcllf corr: admitted edge (\d+) (\S+) -> (\S+) on '([^']+)' \(d = (\d+) ns", log)}


def delivery_violations(log: str) -> list[str]:
    """Dispatches that break "(t, m) at d = 0, (t + d, 0) at d > 0"."""
    edges = admitted(log)
    emitted = {}
    for ev in corr_events(log, "emit"):
        for tok in ev.get("tokens", []):
            emitted.setdefault((ev["edge"], tok["ordinal"]), (ev["elapsed"], ev["microstep"]))
    errors = []
    for ev in corr_events(log, "dispatch"):
        key = (ev["edge"], ev["ordinal"])
        if key not in emitted:
            errors.append(f"edge {ev['edge']} ordinal {ev['ordinal']}: dispatched, never emitted")
            continue
        t, m = emitted[key]
        d = edges[ev["edge"]][3]
        want = (t, m) if d == 0 else (t + d, 0)
        got = (ev["elapsed"], ev["microstep"])
        if got != want or ev["outcome"] != "MATCHED":
            errors.append(f"edge {ev['edge']} ({edges[ev['edge']][2]}, d = {d} ns) ordinal "
                          f"{ev['ordinal']}: published at {(t, m)}, dispatched at {got} "
                          f"{ev['outcome']}, expected {want} MATCHED")
    return errors


def tick_lines(log: str) -> dict[int, list[tuple[str, str, int]]]:
    """tick N -> [(kind, '+' or '', node logical time ns)] in log order."""
    out: dict[int, list] = {}
    for kind, n, plus, t in re.findall(
            r"corr_loop: (ping sent|ping heard|pong heard) payload=(?:pong-)?ping(\d+)(\+?) t=(\d+)",
            log):
        out.setdefault(int(n), []).append((kind, plus, int(t)))
    return out


def check_reconcile(log: str) -> list[str]:
    rec = re.findall(r'rcllf corr: (\{"event":"reconcile".*\})', log)
    if not rec:
        return ["no reconcile summary"]
    r = json.loads(rec[-1])
    if not (r["reconciled"] is True and r["retained"] == 0 and r["carrier"] == 0):
        return [f"reconcile failed: {r}"]
    return []


def check_ticks(log: str, want: list[tuple[str, str, int]]) -> list[str]:
    """Each tick prints *want* (kind, plus, offset from the tick's first line in ns)."""
    problems = []
    ticks = tick_lines(log)
    ns = sorted(ticks)
    if len(ns) < 5 or ns != list(range(ns[0], ns[0] + len(ns))):
        return [f"ticks seen {ns}, expected at least 5 consecutive"]
    base = {}
    for i, n in enumerate(ns):
        lines = ticks[n]
        base[n] = lines[0][2]
        got = [(k, p, t - base[n]) for k, p, t in lines]
        if got != want and not (i == len(ns) - 1 and got == want[:len(got)]):
            problems.append(f"tick {n}: {got}, expected {want}")
    for a, b in zip(ns, ns[1:]):
        if base[b] - base[a] != TICK_NS:
            problems.append(f"ticks {a}->{b} are {base[b] - base[a]} ns apart, not 200 ms")
    return problems


def run_case(case_key: str, needle: str) -> str:
    case = rts.require_case(case_key)
    if case_key not in _LOGS:
        with pytest.MonkeyPatch.context() as mp:
            mp.setenv("RCLLF_TRACE_CALLBACKS", "1")
            result = rts.launch_and_capture(case, 1.0, until=lambda log: needle in log)
        log_dir = os.environ.get("RCLLF_CORR_LOOP_LOG_DIR")
        if log_dir:  # keep the raw run as evidence
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            (Path(log_dir) / f"{case_key}.log").write_text(result.log, encoding="utf-8")
        assert result.finished_cleanly, "no clean shutdown:\n" + result.log[-2000:]
        _LOGS[case_key] = result.log
    return _LOGS[case_key]


def check_loop_log(log: str) -> list[str]:
    """Every problem with a corr_loop (store-only loop) run log, empty when it is correct."""
    problems = []
    edges = set(admitted(log).values())
    want = {("CorrPing", "CorrPong", "/corr/ping", 0), ("CorrPong", "CorrPing", "/corr/pong", 0)}
    if edges != want:
        problems.append(f"admitted edges {sorted(edges)}, expected {sorted(want)}")
    if "rcllf corr: legacy edge" in log:
        problems.append("a legacy edge remains")
    errors = delivery_violations(log)
    problems += errors[:4]
    dispatched = {e: [ev for ev in corr_events(log, "dispatch") if ev["edge"] == e] for e in (0, 1)}
    for e, evs in dispatched.items():
        if len(evs) < 5:
            problems.append(f"edge {e}: only {len(evs)} dispatches")
        for ev in evs:  # ordinal N is tick N, published by the timer at ((N - 1) * 200 ms, 0)
            if (ev["elapsed"], ev["microstep"]) != ((ev["ordinal"] - 1) * TICK_NS, 0):
                problems.append(f"edge {e} ordinal {ev['ordinal']} at "
                                f"{(ev['elapsed'], ev['microstep'])}, expected "
                                f"{((ev['ordinal'] - 1) * TICK_NS, 0)}")
    problems += check_ticks(log, [("ping sent", "", 0), ("ping heard", "", 0),
                                  ("pong heard", "", 0)])
    return problems + check_reconcile(log)


def check_delayed_log(log: str) -> list[str]:
    """Every problem with a corr_loop_echo_delayed run log, empty when it is correct."""
    problems = []
    edges = set(admitted(log).values())
    want = {("CorrPingEcho", "CorrPong", "/corr/ping", 0),
            ("CorrPong", "CorrPingEcho", "/corr/pong", 1_000_000)}
    if edges != want:
        problems.append(f"admitted edges {sorted(edges)}, expected {sorted(want)}")
    if "rcllf corr: legacy edge" in log:
        problems.append("a legacy edge remains")
    if "ros2_lf_input_period: first violation" in log:
        problems.append("the declared 200 ms input period of corr_ping_echo:/corr/pong was violated")
    errors = delivery_violations(log)
    problems += errors[:4]
    if len(corr_events(log, "dispatch")) < 15:
        problems.append(f"only {len(corr_events(log, 'dispatch'))} dispatches")
    ms = 1_000_000
    problems += check_ticks(log, [("ping sent", "", 0), ("ping heard", "", 0),
                                  ("pong heard", "", ms), ("ping sent", "+", ms),
                                  ("ping heard", "+", ms)])
    return problems + check_reconcile(log)


def test_store_only_loop_delivers_both_ways_at_the_publication_tag():
    log = run_case("corr_loop", "pong heard payload=pong-ping8 ")
    problems = check_loop_log(log)
    assert not problems, "; ".join(problems) + "\n" + log[-1500:]


def test_causality_loop_with_one_delayed_link_runs():
    log = run_case("corr_loop_echo_delayed", "ping heard payload=ping8+")
    problems = check_delayed_log(log)
    assert not problems, "; ".join(problems) + "\n" + log[-1500:]
