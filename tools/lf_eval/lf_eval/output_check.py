# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Per-run output checks of an externally paced cost run.

A cost number only counts if the run produced the right output. Each run of a
campaign is checked against values derived from the declared configuration,
never from the run itself:

``logical-oracle`` (``lfc``, ``lfc-offset``)
    The fixed-input oracle of :mod:`lf_eval.semantic.fixed_input`, applied to a
    paced run. The admitted inputs are the timers of the run's own
    ``system.json`` (``offset_ms``, ``period_ms``), so ``lfc-offset`` expects its
    first release at the declared offset and none at tag 0. The publishes per
    callback come from the corpus file (declared application facts). A paced
    run has no logical horizon, so the check uses the last tag at which every
    timer has been released, minus one nanosecond: every input and every
    dispatch with a tag up to that horizon must match the oracle exactly, and
    the canonical trace must report no problem (consumption, edge, publisher,
    reconciliation).

    Tags are measured from reactor-c's start tag, which the binding traces in
    ``lf_start`` (schema 1.1, once per epoch at the first startup dispatch).
    The first release must then sit at the declared offset in logical time.
    A trace without ``lf_start`` (schema 1.0) falls back to the earliest
    observed tag as the origin, which hides a common startup offset. The
    oracle is then shifted by the smallest declared offset and the offset is
    checked physically instead: LF never processes a tag before physical time
    reaches it, so the first dispatch must come at least the declared offset
    after ``run_begin`` (a lower bound, not a timing result). The result says
    which origin was used, and the fallback adds the note
    ``lf_start_absent:physical_offset_check``.

``publish-accounting`` (every variant)
    From the ROS 2 trace, independent of the executor. (1) Timer releases:
    a wall-clock paced timer is due ``span / period + 1`` times between its
    first and last release; a simulated-clock run (``rslcpp``) is due
    ``horizon / period`` times. An rclcpp timer skips a period it could not
    serve in time, so fewer releases are recorded as ``missed`` (work the
    variant did not do, needed to read its CPU time) and do not fail the
    check. More releases than due, or a timer that never runs, fail it. (2) Publications: each topic is published once
    per invocation of every callback the corpus declares for it, so
    ``published(topic)`` equals the sum of those invocation counts. (3)
    Delivery: every publication reaches each subscriber. Slack: one per
    contributing callback in (2) and one per subscriber in (3), for the
    callback or message in flight when the run is stopped.

    With per-reader accounting (:mod:`lf_eval.metrics.drops`) delivery passes
    when every sample of every reader is received or classified, and
    ``before_reader`` (published before the reader existed),
    ``before_match`` (published before DDS matched the reader to a writer, as
    the match probe traced it) and ``before_writer_match`` (the part of
    ``unclassified`` published before the writer side matched the reader, the
    same startup discovery gap seen from the writer) are
    startup artifacts, reported in their own columns but never failing it.
    ``after_both_match`` (both sides had matched), ``within_match_resolution``
    (published after the earliest possible match but before the probe detected
    it on one side), ``no_reader_detection`` (after the writer detection
    with no reader side detection traced) and a loss whose writer was not traced fail it for
    every variant. ``match_source`` says how the probe detected the run's
    matches: ``event`` (rmw matched event callbacks), ``poll`` (its 1 ms poll),
    ``mixed`` or ``none``, with the count of traced increases per source. ``before_first_receipt`` (losses
    published before the reader's first received sample, whatever their
    category) and the number of readers without a traced match are reported
    beside them and exempt nothing. The comparative numbers
    (overwrites, drops, delivery latency) are reported under ``window``, raw
    and per second of window. By default the window runs from the latest
    application reader creation to the run's last application publish, one
    rule for every variant. ``window_ns`` (offsets from the first traced event)
    replaces it, so a campaign can count every run over one common span. The
    result states the window start, end and length per run.
"""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path
from typing import Any, Optional

CHECKS = ("logical-oracle", "publish-accounting")


def _system_timers(system: dict) -> list[tuple[str, str, str, int, int]]:
    """``(launch node, class, callback, offset_ns, period_ns)`` of every declared timer."""
    out = []
    for name, node in (system.get("nodes") or {}).items():
        for t in node.get("timers") or []:
            if t.get("period_ms") is None:
                continue
            out.append((name, str(node["class_name"]), str(t["callback"]),
                        int(round(float(t.get("offset_ms") or 0.0) * 1e6)),
                        int(round(float(t["period_ms"]) * 1e6))))
    return out


def _resolve_topic(topic: str, namespace: Optional[str]) -> str:
    """The fully qualified name the trace carries for a declared topic.

    system.json keeps topics as written in the source. A relative name (the
    reference system's ``"FrontLidarDriver"``) resolves against the node's
    launch namespace, the root namespace when there is none."""
    if topic.startswith("/"):
        return topic
    ns = (namespace or "").strip("/")
    return f"/{ns}/{topic}" if ns else f"/{topic}"


def _launch_namespaces(system: dict) -> dict[str, Optional[str]]:
    root = (system.get("launch_graph") or {}).get("root") or {}
    return {str(n.get("name")): n.get("namespace") for n in root.get("nodes") or [] if n.get("name")}


def _callback_triggers(system: dict) -> dict[str, tuple[str, Optional[str]]]:
    """``"Class.callback" -> (launch node, subscribed topic or None for a timer)``."""
    out: dict[str, tuple[str, Optional[str]]] = {}
    namespaces = _launch_namespaces(system)
    for name, node in (system.get("nodes") or {}).items():
        cls = str(node["class_name"])
        for t in node.get("timers") or []:
            out[f"{cls}.{t['callback']}"] = (name, None)
        for s in node.get("subscribers") or []:
            out[f"{cls}.{s['callback']}"] = (name, _resolve_topic(str(s["topic"]), namespaces.get(name)))
    return out


def check_logical_oracle(run_dir: Path, publishes: dict[str, tuple[str, ...]]) -> dict[str, Any]:
    from lf_eval.semantic.fixed_input import (
        Corpus, _diff_counter, _stats_reasons, canonical_trace, expected_dispatches, expected_inputs)
    from lf_eval.semantic.reader import read_run

    run_dir = Path(run_dir)
    reasons: list[str] = []
    system = json.loads((run_dir / "system.json").read_text())
    try:
        static = json.loads((run_dir / "rcllf_trace_manifest.json").read_text())
    except (OSError, ValueError):
        return {"check": "logical-oracle", "passed": False, "reasons": ["no_static_manifest"]}
    events, stats = read_run(run_dir)
    reasons += _stats_reasons(stats)
    trace = canonical_trace(events, static)
    reasons += trace.problems

    timers = _system_timers(system)
    last: dict[str, int] = {}
    for tag, name in trace.inputs:
        last[name] = max(last.get(name, -1), tag)
    names = [f"{cls}.{cb}" for _, cls, cb, _, _ in timers]
    missing = [n for n in names if n not in last]
    if missing or not names:
        reasons.append(f"timer_never_released:{','.join(missing) or 'no timers'}")
        return {"check": "logical-oracle", "passed": False, "reasons": reasons}
    horizon = min(last[n] for n in names) - 1
    run_begin = next((e for e in events if e.name == "run_begin"), None)
    start_traced = trace.origin != "earliest_tag"
    notes = [] if start_traced else ["lf_start_absent:physical_offset_check"]
    shift = 0 if start_traced else min(off for _, _, _, off, _ in timers)
    offset_gap = None
    if shift:
        dispatch_ts = [e.ts for e in events if e.name == "dispatch_begin"]
        if run_begin is None or not dispatch_ts:
            reasons.append("offset_unobservable:no run_begin or dispatch")
        else:
            offset_gap = min(dispatch_ts) - run_begin.ts
            if offset_gap < shift:
                reasons.append(f"offset_not_applied:first_dispatch_after_run_begin={offset_gap}ns<{shift}ns")
    corpus = Corpus(path=run_dir / "system.json", sha256="", corpus_id="paced", horizon_ns=horizon,
                    inputs=tuple((cls, cb, off - shift, per) for _, cls, cb, off, per in timers),
                    publishes=dict(publishes), min_repetitions=2)
    want_inputs = Counter(expected_inputs(corpus))
    got_inputs = Counter((t, n) for t, n in trace.inputs if t <= horizon)
    reasons += _diff_counter(want_inputs, got_inputs, "inputs")
    want, _cut = expected_dispatches(corpus, static)
    got = Counter({k: v for k, v in trace.dispatches.items() if k[1] <= horizon})
    reasons += _diff_counter(want, got, "dispatches")
    first = {n: min(t for t, m in trace.inputs if m == n) for n in names}
    return {
        "check": "logical-oracle",
        "passed": not reasons,
        "reasons": reasons,
        "horizon_ns": horizon,
        "first_release_ns": first,
        "declared_offset_ns": {f"{cls}.{cb}": off for _, cls, cb, off, _ in timers},
        "tag_origin": trace.origin,
        "notes": notes,
        "oracle_shift_ns": shift,
        "first_dispatch_after_run_begin_ns": offset_gap,
        "expected": {"inputs": sum(want_inputs.values()), "dispatches": sum(want.values())},
        "observed": {"inputs": sum(got_inputs.values()), "dispatches": sum(got.values())},
    }


def check_publish_accounting(run_dir: Path, publishes: dict[str, tuple[str, ...]],
                             simulated_horizon_s: Optional[float] = None,
                             window_ns: Optional[tuple[int, int]] = None) -> dict[str, Any]:
    from lf_eval.model.run import load_run
    from lf_eval.model.system import load_system

    run_dir = Path(run_dir)
    system = json.loads((run_dir / "system.json").read_text())
    run = load_run(run_dir, load_system(run_dir / "system.json"))
    counts = run.drop_counts
    if window_ns is not None and counts is not None and counts.trace_begin_ns is not None:
        counts.set_window(counts.trace_begin_ns + window_ns[0], counts.trace_begin_ns + window_ns[1])
    tl = run.timeline
    reasons: list[str] = []
    timers_out: dict[str, Any] = {}
    for node, cls, cb, _off, period in _system_timers(system):
        starts = sorted(i.start_ns for i in tl.timer_invocations.get(node, []))
        if len(starts) < 2:
            reasons.append(f"timer_releases:{node}:{len(starts)}")
            continue
        if simulated_horizon_s is not None:
            want = int(simulated_horizon_s * 1e9) // period
            tol = 2
        else:
            want = round((starts[-1] - starts[0]) / period) + 1
            tol = max(1, int(0.01 * want))
        missed = max(0, want - len(starts))
        timers_out[node] = {"expected": want, "observed": len(starts), "missed": missed}
        if len(starts) - want > tol:
            reasons.append(f"timer_releases:{node}:expected={want}:observed={len(starts)}")

    triggers = _callback_triggers(system)
    want_pub: Counter = Counter()
    contributors: Counter = Counter()
    for key, topics in publishes.items():
        if key not in triggers:
            reasons.append(f"undeclared_callback:{key}")
            continue
        node, topic_in = triggers[key]
        n = len(tl.timer_invocations.get(node, [])) if topic_in is None else len(
            tl.topic_invocations.get((node, topic_in), []))
        for topic in topics:
            want_pub[topic] += n
            contributors[topic] += 1
    topics = run.drop_counts.topics if run.drop_counts is not None else {}
    pubs_out: dict[str, Any] = {}
    for topic, want in sorted(want_pub.items()):
        tc = topics.get(topic)
        got = tc.published if tc is not None else 0
        pubs_out[topic] = {"expected": want, "observed": got}
        if abs(got - want) > contributors[topic]:
            reasons.append(f"published:{topic}:expected={want}:observed={got}")
    readers = run.drop_counts.readers if run.drop_counts is not None else {}
    delivery: dict[str, Any] = {}
    readers_out: dict[str, Any] = {}
    for topic in sorted(want_pub):
        tc = topics.get(topic)
        if tc is None:
            continue
        delivery[topic] = {"expected": tc.expected, "delivered": tc.delivered, "dropped": tc.dropped}
        mine = {edge: rc for edge, rc in readers.items() if rc.topic == topic}
        readers_out.update({edge: rc.to_dict() for edge, rc in sorted(mine.items())})
        reasons += delivery_reasons(topic, tc, mine)
    return {
        "check": "publish-accounting",
        "passed": not reasons,
        "reasons": reasons,
        "clock": "simulated" if simulated_horizon_s is not None else "wall",
        "timers": timers_out,
        "missed_releases": sum(t["missed"] for t in timers_out.values()),
        "published": pubs_out,
        "delivery": delivery,
        "overwritten": sum(r["overwritten"] for r in readers_out.values()),
        "before_reader": sum(r["before_reader"] for r in readers_out.values()),
        "before_match": sum(r["before_match"] for r in readers_out.values()),
        "before_first_receipt": sum(r["before_first_receipt"] for r in readers_out.values()),
        "unclassified": sum(r["unclassified"] for r in readers_out.values()),
        "before_writer_match": sum(r.get("before_writer_match", 0) for r in readers_out.values()),
        "within_match_resolution": sum(r.get("within_match_resolution", 0) for r in readers_out.values()),
        "no_reader_detection": sum(r.get("no_reader_detection", 0) for r in readers_out.values()),
        "after_both_match": sum(r.get("after_both_match", 0) for r in readers_out.values()),
        "match_source": match_source_summary(
            run.drop_counts.match_sources if run.drop_counts is not None else {}),
        "readers_without_match": sum(1 for r in readers_out.values() if not r["match_traced"]),
        "readers": readers_out,
        "window": accounting_window(run.drop_counts, [readers[e] for e in readers_out]),
    }


def match_source_summary(sources: dict[str, int]) -> dict[str, Any]:
    """Which detection source timed a run's traced matches, with the counts."""
    used = sorted(k for k, v in sources.items() if v)
    label = "none" if not used else used[0] if len(used) == 1 else "mixed"
    return {"source": label, **{k: sources[k] for k in used}}


def accounting_window(counts: Any, readers: list[Any]) -> Optional[dict[str, Any]]:
    """Comparative numbers over the run's accounting window, summed over ``readers``."""
    from lf_eval.metrics.drops import latency_summary

    if counts is None or counts.window_start_ns is None:
        return None
    wins = [r.window for r in readers if r.window is not None]
    begin = counts.trace_begin_ns or 0
    start, end = counts.window_start_ns, counts.window_end_ns
    length_s = (end - start) / 1e9 if end is not None and end > start else None
    keys = ("published", "received", "overwritten", "before_reader", "before_match", "before_first_receipt",
            "before_writer_match", "within_match_resolution", "no_reader_detection", "after_both_match",
            "queued_at_end", "unclassified", "dropped")
    totals = {k: sum(getattr(w, k) for w in wins) for k in keys}
    return {
        "start_rule": "latest application rcl_subscription_init, end at the last application rmw_publish",
        "start_ns": start,
        "end_ns": end,
        "start_after_trace_begin_ns": start - begin,
        "end_after_trace_begin_ns": end - begin if end is not None else None,
        "stop_after_trace_begin_ns": counts.stop_ns - begin if counts.stop_ns is not None else None,
        "length_s": round(length_s, 6) if length_s is not None else None,
        **totals,
        "per_s": {k: round(v / length_s, 3) for k, v in totals.items()} if length_s else None,
        "delivery_latency_ns": latency_summary([x for w in wins for x in w.latency_ns]),
    }


def delivery_reasons(topic: str, tc: Any, readers: dict[str, Any]) -> list[str]:
    """Why one topic's delivery fails publish-accounting (empty when it passes).

    With per-reader accounting (``readers``, module :mod:`lf_eval.metrics.drops`)
    every sample must be received or classified: KEEP_LAST overwrite is a
    measured result of the executor, not a loss, and an unclassified sample fails
    unless it predates the writer side match (``before_writer_match``).
    Without it (no ``rmw_publish`` source timestamps, intra-process only) only the
    counts remain, and more drops than subscribers fail.
    """
    if tc.subscribers == 0:
        return [f"no_subscriber_seen:{topic}"]
    if not readers:
        return [f"dropped:{topic}:{tc.dropped}"] if tc.dropped > tc.subscribers else []
    out = []
    for edge, rc in sorted(readers.items()):
        if not rc.reconciles:
            out.append(f"unreconciled:{edge}")
        # before_writer_match is a startup discovery gap like before_match.
        # within_match_resolution and no_reader_detection stay failures.
        failing = rc.unclassified - rc.before_writer_match
        if failing:
            out.append(f"lost:{edge}:{failing}")
    return out


def check_run(run_dir: Path, checks: list[str], publishes: dict[str, tuple[str, ...]],
              window_ns: Optional[tuple[int, int]] = None) -> dict[str, Any]:
    """Run the named checks on one run. ``passed`` only if every check passed.

    ``window_ns`` sets publish-accounting's accounting window (offsets from the
    first traced event) instead of the run's own.
    """
    import yaml

    run_dir = Path(run_dir)
    md = yaml.safe_load((run_dir / "metadata.yaml").read_text()) or {}
    horizon = md.get("simulated_horizon_s") if md.get("clock_domain", "").startswith("simulated") else None
    rows = []
    for name in checks:
        try:
            if name == "logical-oracle":
                rows.append(check_logical_oracle(run_dir, publishes))
            elif name == "publish-accounting":
                rows.append(check_publish_accounting(run_dir, publishes, horizon, window_ns))
            else:
                rows.append({"check": name, "passed": False, "reasons": [f"unknown_check:{name}"]})
        except Exception as exc:  # noqa: BLE001 - a broken run must fail its check, not the campaign
            rows.append({"check": name, "passed": False, "reasons": [f"error:{type(exc).__name__}:{exc}"]})
    return {"run": run_dir.name, "mode": md.get("mode"), "passed": bool(rows) and all(r["passed"] for r in rows),
            "checks": rows}
