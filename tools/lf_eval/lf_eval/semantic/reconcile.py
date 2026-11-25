# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Semantic reconciliation, chain reconstruction and the four identity metrics (D4/D5).

Everything here joins events by identity keys. Trace timestamps are read only
by `semantic_hop_latency` (a same-host physical profile), never to decide which
events belong together.
"""

from __future__ import annotations

import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Optional

from lf_eval.semantic import schema
from lf_eval.semantic.claims import EVIDENCE_IDENTITY, Claim, claims_block, clock_label
from lf_eval.semantic.completeness import (
    CHANNELS_READ,
    coverage,
    integrity,
    missing_producers,
    publisher_rids,
)
from lf_eval.semantic.fan_in import fan_in_groups, fan_in_order
from lf_eval.semantic.model import Epoch, build_epochs
from lf_eval.semantic.reader import ReadStats, SemEvent, read_run

OUTCOME_NAMES = {v: k for k, v in schema.OUTCOMES.items()}
SAMPLE_NAMES = {v: k for k, v in schema.SAMPLE_OUTCOMES.items()}
MATCHED = schema.OUTCOMES["MATCHED"]


def _key(e: SemEvent) -> tuple[str, int]:
    return (e.gid(), int(e.fields["ordinal"]))


def reconcile_edge(ep: Epoch, edge: int, admitted_by_rid: dict[int, set[int]]) -> list[str]:
    """Codes of `unreconciled(...)` for one admitted edge of epoch *ep*."""
    codes: list[str] = []
    exps = {k: v for k, v in ep.expectations.items() if k[0] == edge}
    for k in exps:
        n = len(ep.terminals.get(k, []))
        if n == 0:
            codes.append("missing_terminal")
        elif n > 1:
            codes.append("duplicate_terminal")
    for k in ep.terminals:
        if k[0] == edge and k not in exps:
            codes.append("terminal_without_expectation")
    by_retained = ep.samples_by_retained()
    for did, b in ep.dispatch_begin.items():
        if int(b.fields["edge"]) != edge:
            continue
        k = (edge, b.gid(), int(b.fields["ordinal"]))
        if k not in exps:
            codes.append("orphan_dispatch")
            continue
        end = ep.dispatch_end.get(did)
        if end is None:
            codes.append("dispatch_without_end")
            continue
        if int(end.fields["outcome"]) != MATCHED:
            continue
        s = by_retained.get(int(end.fields["retained_id"]))
        if s is None or (int(s.fields["edge"]), s.gid(), int(s.fields["ordinal"])) != k:
            codes.append("sample_key_mismatch")
        gid, o = k[1], k[2]
        bl = ep.backlogs.get(gid)
        if (gid, o) not in ep.publications and not (bl and o <= int(bl.fields["last_ordinal"])):
            codes.append("missing_publication")
    # An ACCEPTED admitted publication has one expectation per admitted edge of
    # its (writer, RID): once one publication of a (gid, rid) reaches this edge,
    # every one must; an admitted publication without any expectation is missing.
    exp_edges: dict[tuple[str, int], set[int]] = defaultdict(set)
    for (e2, g2, o2) in ep.expectations:
        exp_edges[(g2, o2)].add(e2)
    writer_edges: dict[tuple[str, int], set[int]] = defaultdict(set)
    admitted = []
    for (gid, o), pubs in ep.publications.items():
        p = pubs[0]
        if (int(p.fields["origin"]) != schema.ORIGINS["ADMITTED"]
                or int(p.fields["result"]) != schema.PUB_RESULTS["ACCEPTED"]):
            continue
        rid = int(p.fields["rid"])
        writer_edges[(gid, rid)] |= exp_edges.get((gid, o), set())
        admitted.append((gid, o, rid))
    for gid, o, rid in admitted:
        got = exp_edges.get((gid, o), set())
        if (edge in writer_edges[(gid, rid)] and edge not in got) or (
                not got and edge in admitted_by_rid.get(rid, set())):
            codes.append("missing_expectation")
    # A publish on this edge's writer from a reaction with no
    # declared effect on it reaches the subscriber outside the correspondence
    # path (its sample is only `unsupported`), so the edge is not reconciled.
    gids = {k[1] for k in exps} | {s.gid() for s in ep.samples if int(s.fields["edge"]) == edge}
    undeclared = schema.ORIGINS["UNDECLARED_EFFECT"]
    for gid in gids:
        endp = ep.end_publishers.get(gid)
        if (endp is not None and int(endp.fields.get("refused_undeclared_effect", 0)) > 0) or any(
                int(p.fields["origin"]) == undeclared
                for (g2, _o), ps in ep.publications.items() if g2 == gid for p in ps):
            codes.append("undeclared_publication")
    end = ep.end_edges.get(edge)
    if end is not None:
        matched = sum(1 for k in exps for t in ep.terminals.get(k, []) if int(t.fields["outcome"]) == MATCHED)
        other = sum(1 for k in exps for t in ep.terminals.get(k, []) if int(t.fields["outcome"]) != MATCHED)
        if (len(exps), matched, other) != (int(end.fields["expectations"]), int(end.fields["matched"]),
                                           int(end.fields["terminal_other"])):
            codes.append("summary_mismatch")
        if int(end.fields.get("orphan_dispatches", 0)) > 0:
            codes.append("producer_orphan_dispatch")
        if int(end.fields.get("count_mismatch", 0)) > 0:
            codes.append("producer_count_mismatch")
    if ep.run_end is not None and not int(ep.run_end.fields.get("reconciled", 0)):
        codes.append("in_process")
    return sorted(set(codes), key=codes.index)


def correspondence_table(ep: Epoch, edge: int) -> dict[str, int]:
    c: Counter = Counter()
    for k, ts in ep.terminals.items():
        if k[0] == edge:
            for t in ts:
                c[OUTCOME_NAMES.get(int(t.fields["outcome"]), str(t.fields["outcome"]))] += 1
    return dict(sorted(c.items()))


def chains(ep: Epoch) -> list[dict[str, Any]]:
    """Per source dispatch (one with no incoming key): the key walk to every sink.

    dispatch -> publication(dispatch_id) -> expectation(edge) -> sample -> dispatch_begin(same key).
    """
    by_key = {(int(b.fields["edge"]), b.gid(), int(b.fields["ordinal"])): b
              for b in ep.dispatch_begin.values() if int(b.fields["edge"]) != schema.NO_EDGE}
    pubs_by_dispatch: dict[int, list[SemEvent]] = defaultdict(list)
    for ps in ep.publications.values():
        for p in ps:
            pubs_by_dispatch[int(p.fields["dispatch_id"])].append(p)
    exp_edges: dict[tuple[str, int], list[int]] = defaultdict(list)
    for (edge, gid, o) in ep.expectations:
        exp_edges[(gid, o)].append(edge)

    def walk(did: int, depth: int) -> list[dict[str, Any]]:
        hops = []
        src = ep.dispatch_begin[did]
        for p in sorted(pubs_by_dispatch.get(did, []), key=lambda e: int(e.fields["ordinal"])):
            k = _key(p)
            for edge in sorted(exp_edges.get(k, [])):
                sink = by_key.get((edge, k[0], k[1]))
                end = ep.dispatch_end.get(int(sink.fields["dispatch_id"])) if sink else None
                hop = {
                    "edge": edge, "gid": k[0], "ordinal": k[1],
                    "outcome": OUTCOME_NAMES.get(int(end.fields["outcome"])) if end else None,
                    "sink_dispatch": int(sink.fields["dispatch_id"]) if sink else None,
                    "logical_delta_ns": (int(sink.fields["tag_time"]) - int(src.fields["tag_time"])) if sink else None,
                    "microstep": int(sink.fields["tag_microstep"]) if sink else None,
                }
                if sink and depth < 16:
                    hop["next"] = walk(int(sink.fields["dispatch_id"]), depth + 1)
                hops.append(hop)
        return hops

    out = []
    for did, b in sorted(ep.dispatch_begin.items()):
        if int(b.fields["edge"]) != schema.NO_EDGE or not pubs_by_dispatch.get(did):
            continue
        out.append({"source_dispatch": did, "rid": int(b.fields["rid"]),
                    "tag": [int(b.fields["tag_time"]), int(b.fields["tag_microstep"])],
                    "hops": walk(did, 0)})
    return out


def hop_latency(ep: Epoch, edge: int) -> dict[str, Any]:
    """Publication event to dispatch_begin of the same key, trace clock (same-host profile)."""
    vals = []
    for b in ep.dispatch_begin.values():
        if int(b.fields["edge"]) != edge:
            continue
        ps = ep.publications.get((b.gid(), int(b.fields["ordinal"])))
        if ps:
            vals.append(b.ts - ps[0].ts)
    if not vals:
        return {"n": 0}
    vals.sort()
    return {"n": len(vals), "min_ns": vals[0], "max_ns": vals[-1],
            "mean_ns": sum(vals) / len(vals), "p50_ns": vals[len(vals) // 2]}


def arrival_lag(ep: Epoch, edge: int) -> list[dict[str, Any]]:
    """Per key: sample arrival LF physical time minus the release tag of its dispatch."""
    by_key = {(int(b.fields["edge"]), b.gid(), int(b.fields["ordinal"])): b for b in ep.dispatch_begin.values()}
    out = []
    for s in ep.samples:
        if int(s.fields["edge"]) != edge:
            continue
        b = by_key.get((edge, s.gid(), int(s.fields["ordinal"])))
        if b is None:
            continue
        out.append({"gid": s.gid(), "ordinal": int(s.fields["ordinal"]),
                    "lag_ns": int(s.fields["arrival_physical_ns"]) - int(b.fields["tag_time"])})
    return sorted(out, key=lambda r: (r["gid"], r["ordinal"]))


def _read_json(path: Path) -> Optional[dict]:
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def analyze(
    events: list[SemEvent], stats: ReadStats, *, run_manifest: Optional[dict],
    static_manifest: Optional[dict], metadata: Optional[dict] = None,
) -> dict[str, Any]:
    """The `semantic` block (schema `rcllf.semantic_report/1`) for one run."""
    metadata = metadata or {}
    epochs = build_epochs(events)
    edges = list((static_manifest or {}).get("edges") or [])
    admitted_by_rid: dict[int, set[int]] = defaultdict(set)
    for e in edges:
        if e.get("status") == "admitted":
            for rid in publisher_rids(e):
                admitted_by_rid[rid].add(int(e["id"]))
    missing = missing_producers(epochs, static_manifest)
    resolved = metadata.get("resolved_model_hash") or (run_manifest or {}).get("resolved_model_hash")
    boot_ids = [ep.boot_id for ep in epochs.values()]
    claims: list[Claim] = []
    metrics: dict[str, Any] = {"semantic_correspondence": {}, "semantic_chain": {},
                               "semantic_hop_latency": {}, "semantic_arrival_lag": {}}
    # one process per generated executable today: the epoch that carries the edges
    ep = next(iter(sorted(epochs.values(), key=lambda x: -len(x.expectations))), None) if epochs else None
    # Each fan-in group's dispatch order. A writer's GID names it in
    # every process, so a publication is looked up in any epoch.
    order: dict[int, dict[str, Any]] = {}
    groups = fan_in_groups(edges)
    if ep and groups:
        pubs: dict = {}
        for other in epochs.values():
            for k, v in other.publications.items():
                pubs.setdefault(k, v)
        order = {lead: fan_in_order(ep, rows, pubs) for lead, rows in sorted(groups.items())}
        metrics["semantic_fan_in_order"] = {str(lead): r for lead, r in order.items()}
    for row in edges or []:
        subject = f"edge {row.get('id') if row.get('id') is not None else '-'} {row['topic']} {row['publisher']['node']}->{row['subscriber']['node']}"
        for kind in ("correspondence", "chain", "physical_hop_latency"):
            chans = CHANNELS_READ[kind]
            icodes = (integrity(ep, stats, run_manifest, static_manifest, resolved, chans) if ep
                      else ["no_run_begin"])
            for other in epochs.values():  # a second process fails the run too
                if other is not ep:
                    icodes += [c for c in integrity(other, stats, run_manifest, static_manifest,
                                                    resolved, chans) if c not in icodes]
            icodes += [f"missing_producer:{m}" for m in missing]
            cov, ccodes = coverage(kind, row, ep, run_manifest, static_manifest, metadata, stats)
            if len(epochs) > 1 and cov != "unsupported":
                # edges are reconciled inside one epoch: a publisher and
                # subscriber in different processes are not evaluated today
                cov, ccodes = "unsupported", ccodes + ["multi_epoch"]
            if kind == "physical_hop_latency" and len(set(boot_ids)) > 1:
                cov, ccodes = "unsupported", ccodes + ["clock_domain"]
            rcodes = reconcile_edge(ep, int(row["id"]), admitted_by_rid) if (
                ep and row.get("status") == "admitted") else []
            if order.get(row.get("group_lead"), {}).get("violations"):
                rcodes = rcodes + ["fan_in_order"]
            claims.append(Claim(
                subject=subject, kind=kind,
                integrity="incomplete" if icodes else "complete", integrity_codes=icodes,
                coverage=cov, coverage_codes=ccodes,
                reconciliation="unreconciled" if rcodes else "reconciled", reconciliation_codes=rcodes,
            ))
        if ep and row.get("status") == "admitted":
            eid = int(row["id"])
            metrics["semantic_correspondence"][str(eid)] = correspondence_table(ep, eid)
            metrics["semantic_hop_latency"][str(eid)] = hop_latency(ep, eid)
            metrics["semantic_arrival_lag"][str(eid)] = arrival_lag(ep, eid)
    if ep:
        metrics["semantic_chain"] = {"epoch": ep.epoch, "chains": chains(ep)}
    extra = {
        "epochs": [e.summary() for e in sorted(epochs.values(), key=lambda x: x.epoch)],
        "metrics": {k: {"evidence": EVIDENCE_IDENTITY, "data": v} for k, v in metrics.items()},
        "profiles": {"semantic_hop_latency": clock_label(boot_ids=boot_ids)},
        "epoch_integrity": {str(e.epoch): integrity(e, stats, run_manifest, static_manifest, resolved,
                                                    ("rcllf",)) for e in epochs.values()},
    }
    return claims_block(claims, extra)


def stock_manifest(system: dict) -> dict[str, Any]:
    """Static-manifest stand-in for a run without one: every publisher -> subscriber
    pair of system.json as an edge, no provider marker (coverage provider_absent)."""
    edges = []
    for t in (system.get("communication") or {}).get("topics") or []:
        for p in t.get("publishers") or []:
            for sub in t.get("subscribers") or []:
                edges.append({"topic": t["topic"], "status": "stock", "id": None,
                              "publisher": {"node": p, "rids": []},
                              "subscriber": {"node": sub, "rid": -1}})
    return {"schema": "rcllf.manifest/1", "kind": "static", "executables": [],
            "provider_marker": None, "edges": edges, "rids": []}


def analyze_run(run_dir: Path) -> dict[str, Any]:
    """Semantic block for a run directory (tracing/ or semantic_events.jsonl + manifests)."""
    import yaml

    run_dir = Path(run_dir)
    events, stats = read_run(run_dir)
    run_manifest = _read_json(run_dir / "trace_manifest.json")
    static = None
    static_path = run_dir / "rcllf_trace_manifest.json"
    if static_path.is_file():
        static = _read_json(static_path)
    elif run_manifest and run_manifest.get("static_manifest_path"):
        static = _read_json(Path(run_manifest["static_manifest_path"]))
    md = {}
    if (run_dir / "metadata.yaml").is_file():
        md = yaml.safe_load((run_dir / "metadata.yaml").read_text()) or {}
    if static is None and (run_dir / "system.json").is_file():
        # A stock (or older) run has no static manifest. Its claim subjects are
        # still the system's topic edges, so they report provider_absent instead
        # of silently producing no claims.
        static = stock_manifest(_read_json(run_dir / "system.json") or {})
    return analyze(events, stats, run_manifest=run_manifest, static_manifest=static, metadata=md)
