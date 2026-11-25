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
"""Event-stream integrity and provider/path coverage (D4).

Both are evaluated from evidence the run left behind: the run manifest
(`trace_manifest.json`, what the run recorder saw), the static manifest (what the
generator emitted) and the rcllf events themselves. Codes are exact strings,
listed in `docs/reference/trace-schema.md`.
"""

from __future__ import annotations

from fnmatch import fnmatch
from typing import Any, Optional, Sequence

from lf_eval.semantic import schema
from lf_eval.semantic.model import Epoch
from lf_eval.semantic.reader import ReadStats

RCLLF = "rcllf"
ROS2 = "ros2"

#: rcllf events a claim kind needs enabled (evaluated per event, D4)
REQUIRED_EVENTS: dict[str, tuple[str, ...]] = {
    "correspondence": ("run_begin", "run_end", "backlog", "publication", "expectation",
                       "terminal", "sample", "dispatch_begin", "dispatch_end"),
    "chain": ("run_begin", "run_end", "backlog", "publication", "expectation", "terminal",
              "sample", "dispatch_begin", "dispatch_end"),
    "physical_hop_latency": ("run_begin", "run_end", "publication", "dispatch_begin"),
    "drop_accounting": ("run_begin", "run_end", "expectation", "terminal", "sample"),
}
#: channels each claim kind reads (Q2: a drop fails only the claims reading that channel)
CHANNELS_READ: dict[str, tuple[str, ...]] = {
    "correspondence": (RCLLF,),
    "chain": (RCLLF,),
    "physical_hop_latency": (RCLLF, ROS2),
    "drop_accounting": (RCLLF,),
}


def _norm_hash(h: Optional[str]) -> str:
    h = h or ""
    return h.split(":", 1)[1] if h.startswith("sha256:") else h


def seq_codes(ep: Epoch) -> list[str]:
    """Received seq set per category must be exactly 1..total of the run_end snapshot."""
    codes: list[str] = []
    totals = list(ep.run_end.fields.get("totals", [])) if ep.run_end else []
    for cat in sorted(set(ep.seqs) | {c for c, t in enumerate(totals) if t}):
        got = ep.seqs.get(cat, [])
        name = schema.CAT_NAMES.get(cat, str(cat))
        if len(got) != len(set(got)):
            codes.append("seq_duplicate")
        want = int(totals[cat]) if cat < len(totals) else (max(got) if got else 0)
        missing = sorted(set(range(1, want + 1)) - set(got))
        if missing:
            codes.append(f"seq_gap:{name}:{missing[0]}")
        if any(s > want for s in got):
            codes.append("seq_duplicate")
    return codes


def publisher_rids(edge_row: dict) -> list[int]:
    """Publishing RIDs of a static-manifest edge: `rids`, or an
    older manifest's single `rid`; neither gives [] (no RID is admitted)."""
    pub = edge_row.get("publisher") or {}
    if "rids" in pub:
        return [int(r) for r in pub["rids"]]
    if "rid" in pub:
        return [int(pub["rid"])]
    return []


def integrity(
    ep: Epoch, stats: ReadStats, run_manifest: Optional[dict], static_manifest: Optional[dict],
    resolved_model_hash: Optional[str], channels: Sequence[str],
) -> list[str]:
    """Codes of `incomplete(...)` for epoch *ep* on *channels*; [] means complete."""
    codes: list[str] = []
    if not run_manifest or run_manifest.get("schema") != "rcllf.manifest/1":
        codes.append("no_run_manifest")
    else:
        mch = run_manifest.get("channels") or {}
        for ch in channels:
            info = mch.get(ch)
            if info is None:
                continue  # an absent channel is a coverage fact (provider_disabled)
            d = info.get("discarded", "unknown")
            if d == "unknown":
                codes.append(f"discards_unknown:{ch}")
            elif int(d) > 0:
                codes.append(f"discarded:{ch}")
            elif stats.discarded_events.get(ch, 0) + stats.discarded_packets.get(ch, 0) > 0:
                codes.append("crosscheck_discard")
    if stats.read_error:
        codes.append("read_error")
    if ep.run_begin is None:
        codes.append("no_begin")
    else:
        if int(ep.run_begin.fields.get("schema_major", -1)) != schema.SCHEMA_MAJOR:
            codes.append("schema_version")
        rb_static = _norm_hash(ep.run_begin.fields.get("static_manifest_hash"))
        rb_model = _norm_hash(ep.run_begin.fields.get("model_hash"))
        want_static = _norm_hash((run_manifest or {}).get("static_manifest_hash"))
        if not rb_static or (want_static and rb_static != want_static) or (
                resolved_model_hash and rb_model != _norm_hash(resolved_model_hash)):
            codes.append("manifest_mismatch")
    if ep.run_begin is not None and ep.run_end is None:
        codes.append("no_end")
    if ep.run_end is not None:
        # run_end counts its close group; a lost record silently disables the
        # per-publisher and per-edge cross-checks, so it fails closed here.
        if ep.end_publisher_records != int(ep.run_end.fields.get("n_publishers", -1)):
            codes.append("missing_close:publisher")
        if ep.duplicate_tracked_gids:
            codes.append("duplicate_publisher_gid")
        if len(ep.end_edges) != int(ep.run_end.fields.get("n_edges", -1)):
            codes.append("missing_close:edge")
    if RCLLF in channels:
        codes.extend(seq_codes(ep))
    return sorted(set(codes), key=codes.index)


def missing_producers(epochs: dict[int, Epoch], static_manifest: Optional[dict]) -> list[str]:
    """Executables the static manifest expects that left no run_begin (provider present)."""
    if not static_manifest or not static_manifest.get("provider_marker"):
        return []
    seen = {ep.executable.rsplit("/", 1)[-1] for ep in epochs.values() if ep.run_begin}
    return [e for e in static_manifest.get("executables", []) if e not in seen]


def _rcllf_rules(run_manifest: Optional[dict]) -> list[dict]:
    ch = ((run_manifest or {}).get("channels") or {}).get(RCLLF) or {}
    return list(ch.get("rules") or [])


def event_enablement(event: str, run_manifest: Optional[dict]) -> str:
    """`enabled`, `filtered` or `disabled` for rcllf:<event> per the run manifest rules."""
    full = f"{RCLLF}:{event}"
    state = "disabled"
    for r in _rcllf_rules(run_manifest):
        if not r.get("enabled", True) or not fnmatch(full, r["pattern"]):
            continue
        if any(fnmatch(full, x) for x in r.get("exclusions") or []):
            continue
        if r.get("filter"):
            state = "filtered" if state == "disabled" else state
        else:
            return "enabled"
    return state


def coverage(
    kind: str, edge_row: dict, ep: Optional[Epoch], run_manifest: Optional[dict],
    static_manifest: Optional[dict], metadata: Optional[dict], stats: Optional[ReadStats] = None,
) -> tuple[str, list[str]]:
    """(`covered`|`uncovered`|`unsupported`, codes) for one claim subject."""
    unsupported: list[str] = []
    uncovered: list[str] = []
    mode = str((metadata or {}).get("mode", ""))
    if not static_manifest or not static_manifest.get("provider_marker") or mode.startswith("stock"):
        unsupported.append("provider_absent")
    if edge_row.get("status") == "legacy":
        unsupported.append(f"legacy:{edge_row.get('reason', '')}")
    elif edge_row.get("status") == "admitted" and not publisher_rids(edge_row):
        unsupported.append("manifest_publisher")  # fail closed
    if unsupported:
        return "unsupported", unsupported
    for ev in REQUIRED_EVENTS[kind]:
        st = event_enablement(ev, run_manifest)
        if st == "disabled":
            uncovered.append(f"provider_disabled:{ev}")
        elif st == "filtered":
            uncovered.append(f"provider_filtered:{ev}")
    if ROS2 in CHANNELS_READ[kind]:
        ros2 = ((run_manifest or {}).get("channels") or {}).get(ROS2)
        if not ros2 or not any(fnmatch("ros2:callback_start", r["pattern"]) for r in ros2.get("rules", [])):
            uncovered.append("ros2_provider_missing")
    if ep is not None and ep.run_end is not None:
        totals = list(ep.run_end.fields.get("totals", []))
        attempts = list(ep.run_end.fields.get("attempts", []))
        for cat, (t, a) in enumerate(zip(totals, attempts)):
            if int(t) < int(a) and not any(c.startswith("provider_disabled") or c.startswith("provider_filtered")
                                           for c in uncovered):
                uncovered.append(f"provider_silent:{schema.CAT_NAMES[cat]}")
        pubs = [p for p in ep.end_publishers.values() if int(p.fields.get("tracked", 0))]
        domain = sum(int(p.fields["accepted"]) + int(p.fields["failed"]) + int(p.fields["nested"])
                     for p in pubs)
        if event_enablement("publication", run_manifest) == "enabled" and int(totals[schema.CATEGORIES["PUBLICATION"]]) < domain:
            uncovered.append("provider_silent:publication")
        pub_gid = _edge_publisher_gid(edge_row, ep)
        if pub_gid and int(ep.end_publishers[pub_gid].fields.get("unsupported_api", 0)) > 0:
            uncovered.append("uninstrumented_api")
        # A missing publication event only means a bypass when the event was recorded.
        bypass = event_enablement("publication", run_manifest) == "enabled" and _bypass(edge_row, ep)
        if int(ep.run_end.fields.get("identity_faults", 0)) > 0 or bypass:
            uncovered.append("path_bypass")
    out = sorted(set(uncovered), key=uncovered.index)
    return ("uncovered", out) if out else ("covered", [])


def _edge_publisher_gid(edge_row: dict, ep: Epoch) -> Optional[str]:
    """GID of the tracked publisher on this edge, from its expectations or samples."""
    eid = edge_row.get("id")
    for (edge, gid, _o) in ep.expectations:
        if edge == eid and gid in ep.end_publishers:
            return gid
    for s in ep.samples:
        if int(s.fields["edge"]) == eid and s.gid() in ep.end_publishers:
            return s.gid()
    return None


def _bypass(edge_row: dict, ep: Epoch) -> bool:
    """A sample from a local tracked GID with no publication event and outside its backlog."""
    eid = edge_row.get("id")
    for s in ep.samples:
        if int(s.fields["edge"]) != eid:
            continue
        g, o = s.gid(), int(s.fields["ordinal"])
        pub = ep.end_publishers.get(g)
        if pub is None or not int(pub.fields.get("tracked", 0)):
            continue
        backlog = ep.backlogs.get(g)
        if backlog is not None and o <= int(backlog.fields["last_ordinal"]):
            continue
        if (g, o) not in ep.publications:
            return True
    return False
