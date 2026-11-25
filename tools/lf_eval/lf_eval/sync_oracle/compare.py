# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""First-difference comparison of a converted sync run with oracle O1.

Checks one synchronizer instance in a fixed order and returns the first report,
or ``None`` when the instance is on the domain and equal to O1:

1. ``version_unpinned:<package> <found>``: each stream's config record names
   the dpkg versions it ran with; both must carry the pinned upstream version
   (``4.11.17-*``, ``28.1.22-*``). Comparing the streams alone would pass when an
   image upgrade moves both sides together.
2. ``config_mismatch:<field>``: domain, policy, queue, versions, and the corpus
   digest the player logged against the one O1 read.
3. ``out_of_domain: ...``: an input without an admitted edge record, a legacy
   edge on an input topic, unequal input delays, a dispatch outcome other than
   MATCHED (LOST, TIMED_OUT, push_overflow, schedule_refused, ...), a binding
   reconcile failure on an input edge (or naming no edge), or an unfinished
   player, or a second subscription of the node on an input topic. Only the
   two synchronizer inputs of the instance count: the admitted edges into the
   node on the input topics. The binding's edge lines name the reactor and the
   topic, not the subscription, so the check requires the synchronizer input
   to be the node's only subscription on its topic and refuses otherwise:
   the binding admits one edge per (writer node, subscription) pair
   and prints a local, ingress or deferred-reader line for each, so two edges
   from one writer node into the node on an input topic are two
   subscriptions; and a drain under another subscription id (``rid``) that
   carries an input's writer is a second subscription on that topic. An
   input edge in the ingress or deferred form is refused as well, and so is
   an input with corpus rows but no drain (no ``rid`` ties its edge to a
   subscription) or with no corpus row but dispatch records. The rule rests
   on that binding invariant: a log that drains an edge the binding never
   printed an admitted line for is outside it.
4. ``arrival_order_mismatch at dispatch <k>``: the drained (input, row)
   sequence and its relative (tag, microstep) against lf_arrival_order.
5. ``tuple_mismatch at seq <k>``: the signal and drop records against O1's.
6. ``classification_mismatch at row <r>``: every delivered sample is
   signalled, dropped, overwritten or pending at end exactly once (ExactTime;
   ApproximateTime has no drop callback, so its unsignalled samples are
   ``unsignalled``).
"""

from __future__ import annotations

import json
import re
from typing import Optional

from lf_eval.sync_oracle.corpus import DOMAIN, arrival_order, row_of
from lf_eval.sync_oracle.order import Run, input_of_callback

PINS = {"message_filters": "4.11.17-", "rclcpp": "28.1.22-"}
CONFIG_FIELDS = ("domain", "policy", "queue", "message_filters", "rclcpp")
EVENTS = ("signal", "drop")
TICK_NS = 50_000_000  # corpus_player's timer period (corpus_player.cpp)
DEFAULT_INPUTS = ("sub_a", "sub_b")
DEFAULT_TOPICS = ("/sync/in0", "/sync/in1")  # sub_a_/sub_b_.subscribe() in exact/approx_sync_node.cpp


def parse_jsonl(text: str) -> list[dict]:
    return [json.loads(line) for line in text.splitlines() if line.strip().startswith("{")]


def short(e: Optional[dict]) -> str:
    """``signal 1 2`` / ``drop - 3`` / ``none``, the corpus expect form."""
    if e is None:
        return "none"
    return " ".join([e["ev"]] + ["-" if s is None else str(s["row"]) for s in e["slots"]])


def check_versions(config: dict, side: str) -> Optional[str]:
    for pkg, prefix in PINS.items():
        found = config.get(pkg, "")
        if not str(found).startswith(prefix):
            return f"version_unpinned:{pkg} {found or 'missing'} ({side})"
    return None


def compare_config(o1: list[dict], node: list[dict], corpus: dict,
                   player: list[dict]) -> Optional[str]:
    oc = next((r for r in o1 if r.get("ev") == "config"), None)
    nc = next((r for r in node if r.get("ev") == "config"), None)
    if oc is None or nc is None:
        return f"config_mismatch:missing ({'O1' if oc is None else 'converted'} has no config record)"
    for cfg, side in ((oc, "O1"), (nc, "converted")):
        report = check_versions(cfg, side)
        if report:
            return report
    if oc.get("impl") != "header" or "mutant" in oc:
        ran = oc.get("impl") if "mutant" not in oc else f"{oc.get('impl')} mutant {oc['mutant']}"
        return f"config_mismatch:impl O1 ran {ran}, oracle O1 is the header"
    if oc.get("domain") != DOMAIN:
        return f"config_mismatch:domain O1 {oc.get('domain')}, declared {DOMAIN}"
    for k in CONFIG_FIELDS:
        if oc.get(k) != nc.get(k):
            return f"config_mismatch:{k} O1 {oc.get(k)}, converted {nc.get(k)}"
    played = next((r.get("corpus") for r in player if r.get("ev") == "corpus"), None)
    for digest, side in ((oc.get("corpus"), "O1"), (played, "player")):
        if digest != corpus["digest"]:
            return f"config_mismatch:corpus {side} {digest}, file {corpus['digest']}"
    return None


def _input_drains(run: Run, node: str, inputs: tuple[str, str]) -> list[tuple[int, dict]]:
    """(input index, drain) of the synchronizer's input callbacks, in order."""
    out = []
    for d in run.drains_of(node):
        i = input_of_callback(d["callback"], inputs)
        if i is not None:
            out.append((i, d))
    return out


def input_edges(run: Run, node: str, topics: tuple[str, str]) -> dict[int, set[int]]:
    """Input index -> the admitted edges into *node* on that input's topic."""
    into = run.edges_into(node)
    return {i: {e for e, rec in into.items() if rec["topic"] == t} for i, t in enumerate(topics)}


def second_subscription(run: Run, node: str, by_input: dict[int, set[int]],
                        drains: list[tuple[int, dict]],
                        topics: tuple[str, str]) -> Optional[str]:
    """A report when *node* has a subscription besides the synchronizer's on
    an input topic, from the edge records or the drains' subscription ids."""
    for i, ids in by_input.items():
        for e in sorted(ids):
            if run.edges[e]["kind"] != "local":
                return (f"out_of_domain: {run.edges[e]['kind']} edge {e} into {node} "
                        f"on input {i}'s topic {topics[i]}")
        per_writer: dict[str, list[int]] = {}
        for e in sorted(ids):
            per_writer.setdefault(run.edges[e]["pub"], []).append(e)
        for pub, es in per_writer.items():
            if len(es) > 1:
                return (f"out_of_domain: second subscription on {topics[i]}: {len(es)} "
                        f"admitted edges {es} from {pub} into {node}")
    rids: dict[int, set] = {}
    writers: dict[str, int] = {}
    for i, d in drains:
        rids.setdefault(i, set()).add(d.get("rid"))
        writers.setdefault(d.get("pub_gid"), i)
    for i, r in sorted(rids.items()):
        if len(r) > 1:
            return (f"out_of_domain: second subscription on {topics[i]}: input {i} of "
                    f"{node} drained under subscription ids {sorted(r, key=str)}")
    mine = set().union(*rids.values()) if rids else set()
    for d in run.drains_of(node):
        i = writers.get(d.get("pub_gid"))
        if i is not None and d.get("rid") not in mine:
            return (f"out_of_domain: second subscription on {topics[i]}: {d.get('callback')} "
                    f"(rid {d.get('rid')}) of {node} drained input {i}'s writer "
                    f"{d.get('pub_gid')}")
    return None


_FAIL_EDGE = re.compile(r"\bedge (\d+)\b")


def check_domain(run: Run, node: str, corpus: dict, player: list[dict],
                 inputs: tuple[str, str] = DEFAULT_INPUTS,
                 topics: tuple[str, str] = DEFAULT_TOPICS) -> Optional[str]:
    by_input = input_edges(run, node, topics)
    for i in range(len(topics)):
        if not by_input[i]:
            return f"out_of_domain: input {i} of {node} has no admitted edge record"
    drains = _input_drains(run, node, inputs)
    report = second_subscription(run, node, by_input, drains, topics)
    if report:
        return report
    # The drains' rid is the only record of which subscription an input edge
    # serves: an input without drains is refused unless the corpus gives it
    # no row and nothing was dispatched on its edges.
    for i in range(len(topics)):
        rows = sum(1 for r in corpus["rows"] if r["input"] == i)
        if any(j == i for j, _ in drains):
            continue
        if rows:
            return (f"out_of_domain: input {i} of {node} drained none of its {rows} corpus "
                    f"rows, its subscription is not identified")
        sent = sorted({r["edge"] for r in run.dispatch if r["edge"] in by_input[i]})
        if sent:
            return (f"out_of_domain: input {i} of {node} has no corpus row but dispatch "
                    f"records on edge(s) {sent}")
    edge_input = {e: i for i, ids in by_input.items() for e in ids}
    edges = {e: run.edges[e] for e in edge_input}
    carried = {(edge_input[r["edge"]], r.get("gid")) for r in run.dispatch if r["edge"] in edges}
    for i, d in drains:
        if (i, d.get("pub_gid")) not in carried:
            return (f"out_of_domain: input {i} of {node} drained writer {d.get('pub_gid')} "
                    f"without an admitted edge record")
    for leg in run.legacy_into(node):
        if leg["topic"] in topics:
            return (f"out_of_domain: legacy edge {leg['pub']} -> {leg['sub']} on "
                    f"{leg['topic']} ({leg['reason']})")
    delays = sorted({e["delay_ns"] for e in edges.values()})
    if len(delays) > 1:
        return f"out_of_domain: unequal input delays {delays} ns"
    for rec in run.dispatch:
        if rec["edge"] in edges and rec.get("outcome") != "MATCHED":
            row = row_of(corpus["rows"], edge_input[rec["edge"]], rec["ordinal"])
            where = f"row {row['row']}" if row else "row unknown"
            return (f"out_of_domain: {rec.get('outcome')} on edge {rec['edge']} "
                    f"ordinal {rec['ordinal']} ({where})")
    for failure in run.failures:
        named = {int(x) for x in _FAIL_EDGE.findall(failure)}
        if not named or named & set(edges):
            return f"out_of_domain: {failure}"
    if not any(r.get("ev") == "done" for r in player):
        return "out_of_domain: incomplete (the corpus player logged no done record)"
    return None


def compare_order(run: Run, node: str, corpus: dict,
                  inputs: tuple[str, str] = DEFAULT_INPUTS) -> Optional[str]:
    """Drained (input, row) sequence and relative (tag, microstep) per dispatch.

    Expected: lf_arrival_order, every row at (tick - first tick) * 50 ms
    relative to the first dispatch, microstep 0 (one drain firing per tag).
    """
    expected = arrival_order(corpus["rows"])
    got = []
    for i, d in _input_drains(run, node, inputs):
        row = row_of(corpus["rows"], i, d.get("pub_seq", 0))
        got.append((i, row["row"] if row else None, d["tag"], d["microstep"]))
    t0 = got[0][2] if got else 0
    k0 = expected[0]["tick"] if expected else 0
    for k, (e, g) in enumerate(zip(expected, got)):
        want = (e["input"], e["row"], (e["tick"] - k0) * TICK_NS, 0)
        have = (g[0], g[1], g[2] - t0, g[3])
        if want != have:
            return (f"arrival_order_mismatch at dispatch {k}: expected input {want[0]} row "
                    f"{want[1]} at +{want[2]} ns microstep {want[3]}, converted input {have[0]} "
                    f"row {have[1]} at +{have[2]} ns microstep {have[3]}")
    if len(expected) != len(got):
        return (f"arrival_order_mismatch: expected {len(expected)} dispatches, "
                f"converted {len(got)}")
    return None


def _events(records: list[dict]) -> list[dict]:
    return [{k: v for k, v in r.items() if k != "node"} for r in records
            if r.get("ev") in EVENTS]


def compare_tuples(o1: list[dict], node: list[dict]) -> Optional[str]:
    oe, ne = _events(o1), _events(node)
    for k, (x, y) in enumerate(zip(oe, ne)):
        if x != y:
            if short(x) == short(y):
                return f"tuple_mismatch at seq {k}: O1 {x}, converted {y}"
            return f"tuple_mismatch at seq {k}: O1 {short(x)}, converted {short(y)}"
    if len(oe) != len(ne):
        k = min(len(oe), len(ne))
        return (f"tuple_mismatch at seq {k}: O1 {short(oe[k] if k < len(oe) else None)}, "
                f"converted {short(ne[k] if k < len(ne) else None)}")
    return None


def classify(corpus: dict, events: list[dict], policy: str) -> dict[int, list[str]]:
    """Row -> its classes; a correct stream gives every row exactly one."""
    classes: dict[int, list[str]] = {r["row"]: [] for r in corpus["rows"]}
    for e in _events(events):
        for s in e["slots"]:
            if s is not None:
                classes.setdefault(s["row"], []).append(
                    "signalled" if e["ev"] == "signal" else "dropped")
    order = arrival_order(corpus["rows"])
    for n, r in enumerate(order):
        if classes[r["row"]]:
            continue
        if policy != "exact":
            classes[r["row"]].append("unsignalled")
        elif any(x["input"] == r["input"] and x["stamp_ns"] == r["stamp_ns"]
                 for x in order[n + 1:]):
            classes[r["row"]].append("overwritten")
        else:
            classes[r["row"]].append("pending")
    return classes


def compare_classes(corpus: dict, o1: list[dict], node: list[dict],
                    policy: str) -> Optional[str]:
    oc, nc = classify(corpus, o1, policy), classify(corpus, node, policy)
    for row in sorted(set(oc) | set(nc)):
        if len(nc.get(row, [])) != 1 or oc.get(row) != nc.get(row):
            return (f"classification_mismatch at row {row}: O1 {oc.get(row)}, "
                    f"converted {nc.get(row)}")
    return None


def compare(corpus: dict, o1: list[dict], run: Run, node: str,
            player_node: str = "corpus_player",
            inputs: tuple[str, str] = DEFAULT_INPUTS,
            topics: tuple[str, str] = DEFAULT_TOPICS) -> Optional[str]:
    """The first report for one synchronizer instance, or None."""
    mine = run.sync.get(node, [])
    player = run.sync.get(player_node, [])
    policy = next((r.get("policy") for r in o1 if r.get("ev") == "config"), "")
    return (compare_config(o1, mine, corpus, player)
            or check_domain(run, node, corpus, player, inputs, topics)
            or compare_order(run, node, corpus, inputs)
            or compare_tuples(o1, mine)
            or compare_classes(corpus, o1, mine, policy))
