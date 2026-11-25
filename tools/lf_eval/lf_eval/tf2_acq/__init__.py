# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""tf2 acquisition-time lookups: reader and comparator.

``rcllf_tf2::AcquisitionLookup`` writes one ``RCLLF_TF2 {json}`` line per
request (``tf_request``) and per outcome (``tf_outcome``), plus
``tf_open_at_shutdown`` for a request still open when the program stops.

* :func:`records` parses those lines out of a run log.
* :func:`outcome_stream` turns the outcomes into text lines with every time
  relative to a run origin and without the physical wait, so two runs of a
  deterministic program give byte-identical streams (plan output
  verification 5).
* :func:`compare` checks named outcomes against a hand table (the showcase's
  ``corpus/t1_t7_expected.tsv``) and names the first case that differs.
* :func:`check_pin` reports ``version_unpinned:tf2_ros <found>`` when the run's
  tf2_ros is not the version the contract was checked against.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Optional

# Pinned dependency: the tf2_ros the acquisition-time contract was checked
# against (dev image of 2026-11-12, plan "Oracle setup"); the same value as
# colcon_lf.node_analysis.tf2_scan.TF2_ROS_PIN. dpkg versions carry a
# distribution suffix ("0.36.23-1noble..."), so the check is a prefix match.
TF2_ROS_PIN = "0.36.23"

# Translation tolerance of the plan (tf2 interpolation is linear; the hand
# values are exact to 1e-9 m).
TOLERANCE_M = 1e-9

_LINE = re.compile(r"RCLLF_TF2 (\{.*\})\s*$")


def records(log: str) -> list[dict]:
    """Every ``RCLLF_TF2`` JSON record of *log*, in log order."""
    out = []
    for line in log.splitlines():
        m = _LINE.search(line)
        if m:
            out.append(json.loads(m.group(1)))
    return out


_START = re.compile(r"ACQ_START t0=(\d+)")
# rclcpp's console line names the logger, i.e. the node ("ns.name" for /ns/name).
_REQ = re.compile(r"(?:\[([^\]\s]+)\]:\s*)?ACQ_REQ case=(\w+) id=(\d+)")

CaseKey = tuple[str, int, int]


def showcase_ids(log: str) -> tuple[int, dict[CaseKey, str]]:
    """The tf2_acq_example sampler's run origin (its first tick's tag, ns) and
    its case map ``(node, adapter, id) -> case name`` (see :func:`pair_key`),
    from its ``ACQ_START``/``ACQ_REQ`` lines. The node is the logger of the
    ``ACQ_REQ`` line, the adapter 0 (the sampler has one): ids count per
    adapter, so a second node's first adapter issues the same ids, and only
    the node tells their cases apart."""
    starts = _START.findall(log)
    if len(starts) != 1:
        raise ValueError(f"expected one ACQ_START line, found {len(starts)}")
    cases: dict[CaseKey, str] = {}
    for logger, case, rid in _REQ.findall(log):
        if not logger:
            raise ValueError(f"ACQ_REQ case={case} id={rid} without a logger name: "
                             "its node, and so its case, is unknown")
        key = ("/" + logger.replace(".", "/"), 0, int(rid))
        if cases.setdefault(key, case) != case:
            raise ValueError(f"ACQ_REQ id={rid} of {key[0]} names two cases: "
                             f"{cases[key]} and {case}")
    return int(starts[0]), cases


def pair_key(r: dict) -> CaseKey:
    """The pairing key of a request, outcome or open record: ``(node,
    adapter, id)``. Ids count per adapter, so two adapters of one node share
    ids; ``adapter`` (since trace schema 1.8) tells them apart and is 0 in an
    older log, which had one adapter per node only by convention."""
    return (r.get("node", ""), int(r.get("adapter", 0)), int(r["id"]))


def case_name(r: dict, case_of: dict[CaseKey, str]) -> str:
    """The showcase case of a record: *case_of* (from :func:`showcase_ids`)
    names the requests by their full :func:`pair_key`; any other record is
    ``<node>#<adapter>/<id>``, so two nodes' requests with one id never share
    a case."""
    key = pair_key(r)
    got = case_of.get(key)
    if got is not None:
        return got
    node, adapter, rid = key
    return f"{node}#{adapter}/{rid}"


def outcome_stream(recs: list[dict], origin_ns: int) -> list[str]:
    """One line per ``tf_outcome``: id, outcome, decision time and stamp in
    ns after *origin_ns*, microstep, logical wait, translation and error.
    The id of a node's second or later adapter is ``<adapter>/<id>`` (an
    adapter-0 id stays bare, so a one-adapter stream reads as before).

    The physical wait and the absolute times are left out: they differ from
    run to run by construction and are reported, never compared.
    """
    lines = []
    for r in recs:
        if r.get("event") != "tf_outcome":
            continue
        xyz = r.get("transform")
        t = "-" if xyz is None else ",".join(f"{v:.9f}" for v in xyz)
        lines.append("\t".join([
            (str(r["id"]) if int(r.get("adapter", 0)) == 0
             else f"{r['adapter']}/{r['id']}"),
            r["outcome"], str(r["time"] - origin_ns), str(r["micro"]),
            str(r["stamp"] - origin_ns), str(r["wait_logical_ns"]), t,
            # tf2's error text names absolute stamps; keep only its class.
            _error_class(r.get("error", "")),
        ]))
    return lines


def _error_class(error: str) -> str:
    if not error:
        return "-"
    if "extrapolation" in error:
        return "extrapolation"
    if "does not exist" in error:
        return "unknown_frame"
    return "other"


def read_hand_table(path: Path) -> list[tuple[str, str, int, float, float]]:
    """Rows ``(case, outcome, decided_ms, x, y)`` of a hand table TSV."""
    rows = []
    for line in Path(path).read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        case, outcome, decided_ms, x, y = line.split("\t")
        rows.append((case, outcome, int(decided_ms), float(x), float(y)))
    return rows


def read_requests(path: Path) -> dict[str, tuple[int, int]]:
    """``case -> (request tag ms, stamp ms)`` from the ``req`` rows of a corpus
    events TSV (``corpus/t1_t7_events.tsv``: kind, time_ms, micro, case:frame,
    stamp_ms, ...)."""
    out = {}
    for line in Path(path).read_text().splitlines():
        cells = line.split("\t")
        if cells[0] == "req":
            out[cells[3].split(":", 1)[0]] = (int(cells[1]), int(cells[4]))
    return out


def compare(recs: list[dict], case_of_id: dict[CaseKey, str], origin_ns: int,
            hand: list[tuple[str, str, int, float, float]],
            requests: Optional[dict[str, tuple[int, int]]] = None) -> Optional[str]:
    """``None`` when every hand row has exactly one equal outcome, else the
    first difference, naming the case (the request row of the corpus).

    Records pair by :func:`pair_key` ``(node, adapter, id)``. Equal means:
    one ``tf_request`` record per case whose stamp the outcome
    repeats, at ``origin + tag_ms`` with stamp ``origin + stamp_ms`` when
    *requests* gives them; same outcome name, decided at
    ``origin + decided_ms`` exactly (logical time), and for ``ok`` the
    translation within 1e-9 m. A ``tf_ingress_reattached`` record (tf2_ros's
    executor took the listener group back, so transforms may have reached the
    Buffer outside LF) fails the run.
    """
    for r in recs:
        if r.get("event") == "tf_ingress_reattached":
            return f"ingress: listener group of {r.get('node', '?')} re-enabled outside LF"
    outcomes: dict[str, list[dict]] = {}
    asked: dict[str, list[dict]] = {}
    for r in recs:
        if r.get("event") == "tf_outcome":
            outcomes.setdefault(case_name(r, case_of_id), []).append(r)
        elif r.get("event") == "tf_request":
            asked.setdefault(case_name(r, case_of_id), []).append(r)
    for r in recs:
        if r.get("event") == "tf_open_at_shutdown":
            return f"{case_name(r, case_of_id)}: still open at shutdown"
    for case, got in outcomes.items():
        req = asked.get(case, [])
        if len(req) != 1:
            return f"{case}: {len(req)} requests, want 1"
        if got[0]["stamp"] != req[0]["stamp"]:
            return f"{case}: outcome stamp {got[0]['stamp']} != requested {req[0]['stamp']}"
        if requests is not None and case in requests:
            tag_ms, stamp_ms = requests[case]
            if req[0]["time"] != origin_ns + tag_ms * 1_000_000:
                return (f"{case}: requested at {(req[0]['time'] - origin_ns) / 1e6} ms "
                        f"!= {tag_ms} ms")
            if req[0]["stamp"] != origin_ns + stamp_ms * 1_000_000:
                return (f"{case}: stamp {(req[0]['stamp'] - origin_ns) / 1e6} ms "
                        f"!= {stamp_ms} ms")
    known = {row[0] for row in hand}
    extra = sorted(set(outcomes) - known)
    if extra:
        return f"{extra[0]}: outcome without a hand row"
    for case, outcome, decided_ms, x, y in hand:
        got = outcomes.get(case, [])
        if len(got) != 1:
            return f"{case}: {len(got)} outcomes, want 1"
        g = got[0]
        if g["outcome"] != outcome:
            return f"{case}: outcome {g['outcome']} != {outcome}"
        want_t = origin_ns + decided_ms * 1_000_000
        if g["time"] != want_t:
            return f"{case}: decided at {(g['time'] - origin_ns) / 1e6} ms != {decided_ms} ms"
        if outcome == "ok":
            gx, gy = g["transform"][0], g["transform"][1]
            if abs(gx - x) > TOLERANCE_M or abs(gy - y) > TOLERANCE_M:
                return f"{case}: translation ({gx:.9f}, {gy:.9f}) != ({x:.9f}, {y:.9f})"
    return None


def _interval_cell(cell: str, origin_ns: int) -> Optional[int | str]:
    """A hand interval cell: ``null`` (tf2 cannot connect the frames),
    ``unavailable`` (the path holds no single exact range: a reparented child
    frame or a dynamic link stamped only at 0), ``abs:<ns>`` (an absolute
    time, 0 for an all-static path) or ms after the origin."""
    if cell == "null":
        return None
    if cell == "unavailable":
        return "unavailable"
    if cell.startswith("abs:"):
        return int(cell[4:])
    return origin_ns + int(round(float(cell) * 1_000_000))


def read_interval_table(path: Path) -> list[tuple[str, str, str, str, str]]:
    """Rows ``(case, request oldest, request latest, outcome oldest, outcome
    latest)`` of a hand interval TSV (cells as in :func:`_interval_cell`)."""
    rows = []
    for line in Path(path).read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        case, ro, rl, oo, ol = line.split("\t")
        rows.append((case, ro, rl, oo, ol))
    return rows


def compare_intervals(recs: list[dict], case_of_id: dict[CaseKey, str], origin_ns: int,
                      table: list[tuple[str, str, str, str, str]]) -> Optional[str]:
    """``None`` when every case's request and outcome carry the hand-derived
    available interval ``[oldest_common, latest_common]`` exactly (ns), else
    the first difference (acceptance criterion 4)."""
    by_case: dict[tuple[str, str], dict] = {}
    for r in recs:
        ev = r.get("event")
        if ev in ("tf_request", "tf_outcome"):
            by_case[(case_name(r, case_of_id), ev)] = r
    for case, ro, rl, oo, ol in table:
        for ev, (o_cell, l_cell) in (("tf_request", (ro, rl)), ("tf_outcome", (oo, ol))):
            r = by_case.get((case, ev))
            if r is None:
                return f"{case}: no {ev} record"
            for key, cell in (("oldest_common", o_cell), ("latest_common", l_cell)):
                if key not in r:
                    return f"{case}: {ev} without {key}"
                want = _interval_cell(cell, origin_ns)
                got = r[key]
                # bool is an int: a JSON true is no time.
                if got != want or isinstance(got, bool):
                    if got is None:
                        shown = "null"
                    elif isinstance(got, int) and not isinstance(got, bool):
                        shown = f"{(got - origin_ns) / 1e6} ms"
                    else:
                        shown = repr(got)
                    return f"{case}: {ev} {key} {shown} != {cell}"
    return None


def check_pin(found: str) -> Optional[str]:
    """``version_unpinned:tf2_ros <found>`` unless *found* is the pinned version."""
    if not found.startswith(TF2_ROS_PIN):
        return f"version_unpinned:tf2_ros {found or 'missing'}"
    return None
