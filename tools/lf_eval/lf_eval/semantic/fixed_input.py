# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Fixed admitted-input semantic check.

A fixed-input run executes the generated program up to a logical horizon (the
reactor-c timeout), so its admitted inputs are a known set of timer releases.
This module checks a set of such runs against a corpus manifest
(``rcllf.fixed_input_corpus/1``):

- **identity**: every run is an ``lfc`` run of this corpus (metadata and run
  record), used the same horizon, generated program and model, and ended by
  itself at the horizon;
- **inputs**: every recorded input hash that has an archived copy recomputes
  (see :func:`verify_inputs` for which copies count);
- **completeness**: the protocol run check (trace loss, semantic claims) passes,
  and the trace has its ``run_begin``, zero identity faults, zero producer
  orphan/count-mismatch counters and exactly the static manifest's writers tracked;
- **corpus**: the observed timer dispatches are exactly the corpus inputs;
- **closure**: every other dispatch consumed, once, an admitted publication of
  the run on an admitted edge of its own reaction, at the publication's tag plus
  the edge delay;
- **oracle**: the dispatch multiset ``(callback, tag)`` equals the one derived
  from the corpus, its declared publishes and the static manifest. The static
  manifest is generator output of the system under test, so the oracle checks the
  run against the generator's own admitted topology, not against an independent
  model of the program;
- **trace identity**: every run's CTF metadata names a trace of its own: one trace
  UUID per run, distinct across the set, and ``trace_name`` equal to
  ``ros2_trace_<run directory>``, and every packet of every event stream beside
  the metadata carries that UUID (see :func:`ctf_trace_identity`);
- **reproduction**: the canonical logical traces of all runs are byte-identical.

The canonical logical trace keeps only logical observations: tags relative to the
start tag, microsteps, reactions, publication ordinals and outcomes. Physical
timestamps, process ids, epochs, dispatch ids, retained ids and GID bytes are
excluded, so the result is a semantic statement and never a latency one.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import struct
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

from lf_eval.semantic import schema
from lf_eval.semantic.model import build_epochs
from lf_eval.semantic.reader import ReadStats, SemEvent, read_run

CORPUS_SCHEMA = "rcllf.fixed_input_corpus/1"
REPORT_SCHEMA = "rcllf.fixed_input_report/1"
#: written by the benchmark orchestrator's fixed-input step into every run directory
RUN_RECORD = "fixed_input.json"

_KIND = {v: k for k, v in schema.ENTRY_KINDS.items()}
_OUTCOME = {v: k for k, v in schema.OUTCOMES.items()}
_SAMPLE = {v: k for k, v in schema.SAMPLE_OUTCOMES.items()}
_ORIGIN = {v: k for k, v in schema.ORIGINS.items()}
_CARRIER = {v: k for k, v in schema.CARRIER_OPS.items()}
_CLEANUP = {v: k for k, v in schema.CLEANUP_REASONS.items()}
#: zero-padded relative tags, so sorted lines are in tag order (13 digits: 2.7 h in ns)
TAG_WIDTH = 13
#: run_begin fields that must agree across the runs of one set
IDENTITY_FIELDS = ("static_manifest_hash", "generated_source_hash", "model_hash", "build_id",
                   "executable", "binding_version")
#: run-record input files that must exist (exec_config.json may be absent: built-in defaults)
REQUIRED_INPUTS = ("executable", "program.lf", "rcllf_trace_manifest.json", "resolved_model.json",
                   "scenario_overlay.env", "scenario_params.yaml", "system.json")
#: inputs the benchmark orchestrator's fixed-input step writes into the run directory itself
RUN_LOCAL_INPUTS = ("rcllf_trace_manifest.json", "scenario_overlay.env", "scenario_params.yaml",
                    "system.json")
#: archive directories, relative to the run set root (the run directory's parent),
#: that may hold copies of the other recorded inputs under their record name
ARCHIVE_INPUT_DIRS = ("inputs", "campaign/inputs")
#: the scope of every input-hash verdict in the report
INPUT_HASH_SCOPE = (
    "the runner hashes its inputs after the run ends; a recomputed match binds the archived "
    "bytes to that record, not to the files the process loaded at launch. Inputs with no "
    "archived copy are 'recorded_only': their hash was not independently checked")
#: tracked-publisher counters that must stay zero in a fixed-input run
PUBLISHER_FAULTS = ("failed", "refused_no_context", "refused_undeclared_effect",
                    "refused_edge_not_admitted", "nested", "unsupported_api", "untracked")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
#: LTTng session (and so CTF ``env.trace_name``) of a run: the run recorder names it
#: ``ros2_trace_<run_id>`` and the run id is the run directory's name
TRACE_NAME_PREFIX = "ros2_trace_"
#: the scope of every trace-identity verdict in the report
TRACE_IDENTITY_SCOPE = (
    "ctf_metadata: each run's trace UUID and trace_name are read from the bytes of its one "
    "CTF metadata file; the UUIDs are distinct across the set and each trace_name is "
    "ros2_trace_<run directory>; every packet header of every event-stream file beside the "
    "metadata (index files excluded) carries that UUID. This rejects a trace or its event "
    "streams copied between runs; it is a consistency check of unauthenticated bytes, not "
    "protection against forged trace files nor proof of which process wrote them. "
    "jsonl_fixture: no trace identity was checked (explicit test bypass); it is no evidence "
    "of independent live processes")
#: CTF 1.8 packetized metadata (TSDL spec, section 7.1): magic, trace uuid, checksum,
#: content and packet size in bits, compression, encryption, checksum scheme, major, minor
_CTF_MAGIC = 0x75D11D57
_CTF_PACKET = {"<": struct.Struct("<I16sIIIBBBBB"), ">": struct.Struct(">I16sIIIBBBBB")}
_UUID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
_TSDL_ASSIGN_RE = re.compile(r"^([A-Za-z_][\w.]*)\s*=\s*(.*)$", re.DOTALL)
_TSDL_BLOCK_RE = re.compile(r"^([A-Za-z_]\w*)\s*\{(.*)\}$", re.DOTALL)
#: LTTng-UST event-stream packet: header magic ``0xC1FC1FC1`` and trace uuid, then
#: stream_id, stream_instance_id, timestamp_begin/end, content_size and packet_size
#: (bits); the declarations below pin that layout, whitespace removed
_STREAM_MAGIC = 0xC1FC1FC1
_STREAM_HEADER = {o: struct.Struct(o + "I16s28xQQ") for o in "<>"}
#: header and context bytes up to and including cpu_id
_STREAM_MIN_PACKET = 84
_LTTNG_TYPEALIASES = {
    name: f"typealiasinteger{{size={bits};align=8;signed=false;{extra}}}:={name}"
    for name, bits, extra in (("uint8_t", 8, ""), ("uint32_t", 32, ""), ("uint64_t", 64, ""),
                              ("unsignedlong", 64, ""),
                              ("uint64_clock_monotonic_t", 64, "map=clock.monotonic.value;"))}
_LTTNG_PACKET_HEADER = ("packet.header:=struct{uint32_tmagic;uint8_tuuid[16];uint32_tstream_id;"
                        "uint64_tstream_instance_id;}")
_LTTNG_PACKET_CONTEXT = ("structpacket_context{uint64_clock_monotonic_ttimestamp_begin;"
                         "uint64_clock_monotonic_ttimestamp_end;uint64_tcontent_size;"
                         "uint64_tpacket_size;uint64_tpacket_seq_num;"
                         "unsignedlongevents_discarded;uint32_tcpu_id;}")


class CorpusError(ValueError):
    """The corpus manifest is malformed or does not fit the static manifest."""


@dataclass(frozen=True)
class Corpus:
    path: Path
    sha256: str
    corpus_id: str
    horizon_ns: int
    inputs: tuple[tuple[str, str, int, int], ...]   # (node, callback, offset_ns, period_ns)
    publishes: dict[str, tuple[str, ...]]          # "Node.callback" -> topics
    min_repetitions: int
    data: dict[str, Any] = field(repr=False, compare=False, default_factory=dict)


def load_corpus(path: Path) -> Corpus:
    import yaml

    path = Path(path)
    raw = path.read_bytes()
    data = yaml.safe_load(raw) or {}
    if data.get("schema") != CORPUS_SCHEMA:
        raise CorpusError(f"{path}: schema is {data.get('schema')!r}, expected {CORPUS_SCHEMA!r}")
    try:
        horizon = int(data["horizon"]["lf_timeout_ns"])
        inputs = tuple((str(i["node"]), str(i["callback"]), int(i["offset_ns"]), int(i["period_ns"]))
                       for i in data["inputs"])
        publishes = {str(k): tuple(str(t) for t in v)
                     for k, v in ((data.get("oracle") or {}).get("publishes") or {}).items()}
        corpus = Corpus(path=path, sha256=hashlib.sha256(raw).hexdigest(),
                        corpus_id=str(data["corpus_id"]), horizon_ns=horizon, inputs=inputs,
                        publishes=publishes, min_repetitions=int(data.get("min_repetitions", 2)),
                        data=data)
    except (KeyError, TypeError, ValueError) as exc:
        raise CorpusError(f"{path}: {type(exc).__name__}: {exc}") from exc
    if horizon <= 0 or not inputs:
        raise CorpusError(f"{path}: needs a positive horizon and at least one input")
    if corpus.min_repetitions < 2:
        raise CorpusError(f"{path}: min_repetitions must be at least 2")
    for node, cb, offset, period in inputs:
        if period <= 0 or offset < 0:
            raise CorpusError(f"{path}: input {node}.{cb} needs offset >= 0 and period > 0")
    return corpus


def expected_inputs(corpus: Corpus) -> list[tuple[int, str]]:
    """Sorted ``(tag_ns, "Node.callback")`` of every timer release at or before the horizon."""
    out = []
    for node, cb, offset, period in corpus.inputs:
        out += [(t, f"{node}.{cb}") for t in range(offset, corpus.horizon_ns + 1, period)]
    return sorted(out)


def _rid_names(static: dict) -> dict[int, str]:
    return {int(r["rid"]): f"{r['node']}.{r['callback']}" for r in static.get("rids") or []}


def expected_dispatches(corpus: Corpus, static: dict) -> tuple[Counter, list[str]]:
    """Dispatch multiset ``(callback, tag_ns)`` the corpus implies, and the cut-off chains.

    Every input dispatches its callback; each declared publish follows every
    admitted edge of that callback and topic to the subscriber at ``tag +
    delay_ns``. A hop past the horizon is listed as cut off instead.
    """
    names = _rid_names(static)
    by_name = {v: k for k, v in names.items()}
    for key in list(corpus.publishes) + [f"{n}.{c}" for n, c, _, _ in corpus.inputs]:
        if key not in by_name:
            raise CorpusError(f"{key} is not a reaction of the static manifest")
    hops: dict[tuple[int, str], list[tuple[str, int]]] = defaultdict(list)
    for e in static.get("edges") or []:
        if e.get("status") != "admitted":
            continue
        for rid in _edge_publisher_rids(e):
            hops[(rid, e["topic"])].append(
                (names[int(e["subscriber"]["rid"])], int(e["delay_ns"])))
    counts: Counter = Counter()
    cut: list[str] = []
    work = [(cb, t) for t, cb in expected_inputs(corpus)]
    while work:
        cb, t = work.pop()
        counts[(cb, t)] += 1
        for topic in corpus.publishes.get(cb, ()):
            for sub, delay in hops.get((by_name[cb], topic), []):
                if t + delay <= corpus.horizon_ns:
                    work.append((sub, t + delay))
                else:
                    cut.append(f"{cb}@{t} {topic} -> {sub}@{t + delay}")
    return counts, sorted(cut)


def _edge_publisher_rids(edge: dict) -> set[int]:
    pub = edge.get("publisher") or {}
    return {int(r) for r in pub.get("rids") or ([pub["rid"]] if pub.get("rid") is not None else [])}


def _static_writers(static: dict, names: dict[int, str]) -> set[str]:
    """``Node:topic`` labels of the admitted writers, in the canonical trace's label form."""
    out = set()
    for e in static.get("edges") or []:
        if e.get("status") == "admitted":
            nodes = sorted({names.get(r, f"rid{r}").split(".")[0] for r in _edge_publisher_rids(e)})
            out.add("+".join(nodes or ["?"]) + ":" + str(e.get("topic")))
    return out


def _consumption_problems(b: SemEvent, ep: Any, edges: dict[int, dict], names: dict[int, str],
                          where: str) -> list[str]:
    """A ``CORR_SUB`` dispatch must consume its key's admitted publication on its own edge.

    The dispatch names ``(edge, gid, ordinal)``. That publication must exist in
    this run, come from a reaction the static manifest lists as the edge's
    publisher, and the edge must be admitted with this dispatch's reaction as
    subscriber; the dispatch tag is the publication tag plus the edge's
    ``delay_ns``. Protocol reconciliation (``reconcile_edge``) checks key
    existence and sample identity per edge, but neither the subscriber, the
    publisher reaction nor the tag arithmetic.
    """
    f = b.fields
    edge = edges.get(int(f["edge"]))
    if edge is None or edge.get("status") != "admitted":
        return [f"unadmitted_edge:{where}:edge{int(f['edge'])}"]
    out = []
    sub = (edge.get("subscriber") or {}).get("rid")
    if sub is None or int(sub) != int(f["rid"]):
        out.append(f"edge_subscriber_mismatch:{where}:edge{int(f['edge'])}")
    pubs = ep.publications.get((b.gid(), int(f["ordinal"])))
    if not pubs:
        return out + [f"orphan_dispatch:{where}"]
    p = pubs[0].fields
    if int(p["rid"]) not in _edge_publisher_rids(edge):
        out.append(f"edge_publisher_mismatch:{where}:{names.get(int(p['rid']), int(p['rid']))}")
    want = int(p["tag_time"]) + int(edge["delay_ns"])
    if int(f["tag_time"]) != want:
        out.append(f"consumed_tag_mismatch:{where}:delta={int(f['tag_time']) - want}")
    return out


@dataclass
class LogicalTrace:
    lines: list[str]
    digest: str
    inputs: list[tuple[int, str]]
    dispatches: Counter
    problems: list[str]
    identity: dict[str, Any]
    origin: str = "earliest_tag"  # "lf_start", "run_begin" or "earliest_tag"


def canonical_trace(events: Sequence[SemEvent], static: dict) -> LogicalTrace:
    """The gid-free, physical-time-free logical trace of one run (sorted lines + sha256)."""
    problems: list[str] = []
    epochs = [e for e in build_epochs(events).values() if e.run_begin is not None or e.dispatch_begin]
    if len(epochs) != 1:
        problems.append(f"epochs:{len(epochs)}")
    if not epochs:
        return LogicalTrace([], hashlib.sha256(b"").hexdigest(), [], Counter(), problems, {})
    ep = max(epochs, key=lambda e: len(e.dispatch_begin))
    if ep.run_begin is None:
        problems.append("no_run_begin")
    names = _rid_names(static)
    edges = {int(e["id"]): e for e in static.get("edges") or [] if e.get("id") is not None}
    topics = {i: e["topic"] for i, e in edges.items()}

    def cb(rid: int) -> str:
        return names.get(int(rid), f"rid{rid}")

    # Tags are relative to reactor-c's start tag, traced by lf_start (schema
    # 1.1). run_begin precedes the start time, so its lf_start_time_ns is 0 in
    # every binding trace. Traces without lf_start (schema
    # 1.0 recordings) fall back to the earliest observed tag: a uniform shift is then
    # invisible, but missing tag-0 inputs still fail the corpus check.
    tag_origin = "lf_start"
    start = int(ep.lf_start.fields.get("lf_start_time_ns") or 0) if ep.lf_start else 0
    if not start:
        tag_origin = "run_begin"
        start = int((ep.run_begin.fields if ep.run_begin else {}).get("lf_start_time_ns") or 0)
    if not start:
        tag_origin = "earliest_tag"
        tags = [int(e.fields["tag_time"]) for e in ep.dispatch_begin.values()]
        tags += [int(p.fields["tag_time"]) for ps in ep.publications.values() for p in ps]
        start = min(tags) if tags else 0

    # A GID is random per process; name it by its publishing node and topic.
    nodes_of: dict[str, set[str]] = defaultdict(set)
    topics_of: dict[str, set[str]] = defaultdict(set)
    for (gid, _), ps in ep.publications.items():
        nodes_of[gid].update(cb(int(p.fields["rid"])).split(".")[0] for p in ps)
    for (edge, gid, _) in ep.expectations:
        topics_of[gid].add(topics.get(edge, f"edge{edge}"))
    labels: dict[str, str] = {}
    for gid in sorted(set(nodes_of) | set(topics_of)):
        labels[gid] = "+".join(sorted(nodes_of[gid]) or ["?"]) + ":" + "+".join(sorted(topics_of[gid]) or ["?"])
    seen: dict[str, str] = {}
    for gid, lab in labels.items():
        if lab in seen:
            problems.append(f"ambiguous_publisher:{lab}")
        seen[lab] = gid

    def label(gid: str) -> str:
        if not gid or set(gid) == {"0"}:
            return "-"
        return labels.get(gid, "unknown")

    lines: list[str] = []
    inputs: list[tuple[int, str]] = []
    dispatches: Counter = Counter()
    consumed: Counter = Counter()
    for did, b in ep.dispatch_begin.items():
        f = b.fields
        tag = int(f["tag_time"]) - start
        kind = _KIND.get(int(f["entry_kind"]), str(f["entry_kind"]))
        name = cb(int(f["rid"]))
        end = ep.dispatch_end.get(did)
        outcome = _OUTCOME.get(int(end.fields["outcome"]), "?") if end else "NO_END"
        ran = int(end.fields["callback_ran"]) if end else -1
        lab, ordinal = label(b.gid()), int(f["ordinal"])
        lines.append(f"D|{tag:0{TAG_WIDTH}d}|{int(f['tag_microstep'])}|{name}|{kind}|{int(f['edge'])}|{lab}|{ordinal}"
                     f"|{outcome}|{ran}")
        dispatches[(name, tag)] += 1
        if kind == "TIMER":
            inputs.append((tag, name))
        elif kind == "CORR_SUB":
            consumed[(int(f["edge"]), b.gid(), ordinal)] += 1
            problems += _consumption_problems(b, ep, edges, names, f"{name}@{tag}:{lab}#{ordinal}")
        else:
            problems.append(f"unadmitted_entry:{kind}:{name}@{tag}")
        if end is None:
            problems.append(f"no_dispatch_end:{name}@{tag}")
    for (edge, gid, ordinal), n in sorted(consumed.items()):
        if n > 1:
            problems.append(f"duplicate_consumption:edge{edge}:{label(gid)}#{ordinal}:x{n}")
    for (gid, ordinal), ps in ep.publications.items():
        for p in ps:
            f = p.fields
            origin = _ORIGIN.get(int(f["origin"]), str(f["origin"]))
            lines.append(f"P|{int(f['tag_time']) - start:0{TAG_WIDTH}d}|{int(f['tag_microstep'])}|{cb(int(f['rid']))}"
                         f"|{label(gid)}|{ordinal}|{int(f['result'])}|{origin}|{int(f['api'])}")
            if origin != "ADMITTED":
                problems.append(f"unadmitted_origin:{origin}:{label(gid)}#{ordinal}")
    for (edge, gid, ordinal), xs in ep.expectations.items():
        for x in xs:
            lines.append(f"X|{label(gid)}|{ordinal}|{edge}|{int(x.fields['instance'])}"
                         f"|{int(x.fields['generation'])}")
    for (edge, gid, ordinal), ts in ep.terminals.items():
        for t in ts:
            lines.append(f"T|{label(gid)}|{ordinal}|{edge}|{_OUTCOME.get(int(t.fields['outcome']), '?')}")
    for s in ep.samples:
        lines.append(f"S|{int(s.fields['edge'])}|{label(s.gid())}|{int(s.fields['ordinal'])}"
                     f"|{_SAMPLE.get(int(s.fields['outcome']), '?')}")
    for c in ep.carriers:
        f = c.fields
        lines.append(f"C|{int(f['edge'])}|{_CARRIER.get(int(f['op']), '?')}|{int(f['count'])}"
                     f"|{int(f['release_tag_time']) - start:0{TAG_WIDTH}d}")
    for k in ep.cleanups:
        f = k.fields
        lines.append(f"K|{int(f['released_retained'])}|{int(f['carrier_records_closed'])}"
                     f"|{_CLEANUP.get(int(f['reason']), '?')}")
    for edge, e in ep.end_edges.items():
        f = e.fields
        lines.append("E|" + "|".join(str(int(f[k])) for k in (
            "edge", "expectations", "matched", "terminal_other", "samples_taken",
            "samples_retained", "orphan_dispatches", "count_mismatch")))
        problems += [f"edge_{k}:edge{edge}={int(f[k])}" for k in ("orphan_dispatches", "count_mismatch")
                     if int(f[k])]
    untracked = 0
    tracked: list[str] = []
    for gid, e in ep.end_publishers.items():
        f = e.fields
        if not int(f.get("tracked", 0)):
            untracked += 1
            continue
        tracked.append(label(gid))
        lines.append(f"U|{label(gid)}|" + "|".join(str(int(f[k])) for k in (
            "accepted", "failed", "refused_no_context", "refused_undeclared_effect",
            "refused_edge_not_admitted", "nested", "unsupported_api", "untracked")))
        problems += [f"publisher_{k}:{label(gid)}={int(f[k])}" for k in PUBLISHER_FAULTS if int(f[k])]
    lines.append(f"U|untracked_publishers|{untracked}")
    # run_end.untracked_publishes counts publishes on every untracked publisher of
    # the process, also those outside the program (22 of 34 in the 2026-10-04
    # archive, 2,603 publishes), so it is not a fault by itself. The fault is an
    # admitted writer of the static manifest left untracked: there must be one
    # tracked publisher per manifest (publishing node, topic) pair, and each one
    # that published must be one of them. A tracked writer that never published
    # has no observations to name it by ("unknown"; corr_fanout's /corr/b).
    writers = _static_writers(static, names)
    if len(tracked) != len(writers):
        problems.append(f"tracked_writers:{len(tracked)}!={len(writers)}")
    problems += [f"tracked_writer_not_in_manifest:{w}" for w in sorted(set(tracked) - writers - {"unknown"})]
    if ep.run_end is None:
        problems.append("no_run_end")
    else:
        f = ep.run_end.fields
        lines.append("R|" + "|".join(f"{k}={int(f[k])}" for k in (
            "reconciled", "identity_faults", "untracked_publishes", "n_publishers", "n_edges")))
        if not int(f["reconciled"]):
            problems.append("not_reconciled")
        if int(f["identity_faults"]):
            problems.append(f"identity_faults:{int(f['identity_faults'])}")
    lines.sort()
    digest = hashlib.sha256(("\n".join(lines) + "\n").encode()).hexdigest()
    rb = ep.run_begin.fields if ep.run_begin else {}
    identity = {k: rb.get(k) for k in IDENTITY_FIELDS}
    return LogicalTrace(lines, digest, sorted(inputs), dispatches, problems, identity, tag_origin)


def _diff_counter(expected: Counter, observed: Counter, what: str) -> list[str]:
    missing = sorted((expected - observed).elements())
    extra = sorted((observed - expected).elements())
    out = []
    if missing:
        out.append(f"{what}_missing:{len(missing)}:first={missing[0]}")
    if extra:
        out.append(f"{what}_extra:{len(extra)}:first={extra[0]}")
    return out


def _read_json(path: Path) -> Optional[dict]:
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def _sha256_file(path: Path) -> Optional[str]:
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest() if Path(path).is_file() else None
    except OSError:
        return None


def verify_inputs(run_dir: Path, hashes: dict[str, Any]) -> tuple[list[str], dict[str, str]]:
    """Recompute the recorded input hashes from archived copies: ``(reasons, status)``.

    Rule, by record name (absolute paths in the record are never read):

    - :data:`RUN_LOCAL_INPUTS` must exist in the run directory and hash to the
      recorded value (``input_file_absent`` / ``input_hash_mismatch``);
    - any other name is looked up as ``<set root>/<dir>/<name>`` for each
      :data:`ARCHIVE_INPUT_DIRS` entry, the set root being the run directory's
      parent. Every copy found must hash to the recorded value; a name recorded
      as absent (``null``) must have no copy. With no copy the status is
      ``recorded_only``: the hash is carried, not checked.

    Status values: ``verified:run``, ``verified:<dir>[,<dir>]``, ``recorded_only``,
    ``recorded_absent``, ``absent``, ``mismatch``.
    """
    run_dir = Path(run_dir)
    reasons: list[str] = []
    status: dict[str, str] = {}
    for name, want in sorted(hashes.items()):
        if want is not None and not (isinstance(want, str) and _SHA256_RE.match(want)):
            reasons.append(f"malformed_record:input_sha256:{name}")
            status[name] = "mismatch"
            continue
        if name in RUN_LOCAL_INPUTS:
            if want is None:
                status[name] = "recorded_absent"
                continue
            got = _sha256_file(run_dir / name)
            if got is None:
                reasons.append(f"input_file_absent:{name}")
                status[name] = "absent"
            elif got != want:
                reasons.append(f"input_hash_mismatch:{name}")
                status[name] = "mismatch"
            else:
                status[name] = "verified:run"
            continue
        if "/" in name or "\\" in name or name in ("", ".", ".."):
            reasons.append(f"malformed_record:input_name:{name!r}")
            status[name] = "mismatch"
            continue
        found = [(d, _sha256_file(run_dir.parent / d / name)) for d in ARCHIVE_INPUT_DIRS
                 if (run_dir.parent / d / name).is_file()]
        bad = [d for d, got in found if got != want]
        reasons += [f"input_hash_mismatch:{name}@{d}" for d in bad]
        if bad:
            status[name] = "mismatch"
        elif found:
            status[name] = "verified:" + ",".join(d for d, _ in found)
        else:
            status[name] = "recorded_only" if want is not None else "recorded_absent"
    return reasons, status


def _record_reasons(record: dict[str, Any], corpus: Corpus) -> list[str]:
    """Run-record checks; a field of the wrong type is a rejection, never a crash."""
    reasons: list[str] = []
    if record.get("corpus_sha256") != corpus.sha256:
        reasons.append("corpus_changed")
    if record.get("corpus_id") != corpus.corpus_id:
        reasons.append(f"corpus_id_mismatch:{record.get('corpus_id')!r}")
    timeout = record.get("lf_timeout_ns")
    if type(timeout) is not int:
        reasons.append("malformed_record:lf_timeout_ns")
    elif timeout != corpus.horizon_ns:
        reasons.append("horizon_changed")
    hashes = record.get("input_sha256")
    if not isinstance(hashes, dict):
        reasons.append("malformed_record:input_sha256")
        hashes = {}
    reasons += [f"input_missing:{name}" for name in REQUIRED_INPUTS if not hashes.get(name)]
    launcher = record.get("launcher")
    if not isinstance(launcher, dict):
        reasons.append("malformed_record:launcher")
        launcher = {}
    rc = launcher.get("returncode")
    if launcher.get("stopped_by_driver") is not False or type(rc) is not int or rc != 0:
        reasons.append(f"not_self_terminated:rc={rc}"
                       f",signal={launcher.get('signal')},driver={launcher.get('stopped_by_driver')}")
    return reasons


def _metadata_reasons(run_dir: Path, corpus: Corpus) -> list[str]:
    """``metadata.yaml`` must name an lfc run of this corpus, else the run is not one."""
    from lf_eval.runset import _load_metadata

    md, failed = _load_metadata(run_dir)
    if failed:
        return [failed]
    reasons = []
    if md.get("mode") != "lfc":
        reasons.append(f"mode_not_lfc:{md.get('mode')!r}")
    for key, want in (("scenario", corpus.corpus_id), ("corpus_id", corpus.corpus_id),
                      ("corpus_sha256", corpus.sha256)):
        if md.get(key) != want:
            reasons.append(f"metadata_{key}_mismatch:{md.get(key)!r}")
    return reasons


def _record(run_dir: Path) -> Optional[dict]:
    data = _read_json(Path(run_dir) / RUN_RECORD)
    return data if isinstance(data, dict) else None


class CtfIdentityError(ValueError):
    """The CTF metadata does not name exactly one well-formed trace."""


def _utf8(data: bytes) -> str:
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise CtfIdentityError("not_utf8") from exc


def ctf_metadata_text(data: bytes) -> tuple[str, Optional[bytes]]:
    """TSDL text of a CTF metadata file and the trace UUID its packet headers carry.

    Packetized metadata (magic ``0x75D11D57``, either byte order) is unpacked packet
    by packet: every header must be CTF 1.8, uncompressed and unencrypted, with
    sizes that fit the file, and all packets must carry one UUID. Plain-text
    metadata must start with the ``/* CTF`` signature and carries no header UUID.
    """
    head = data[:4]
    if len(head) == 4 and struct.unpack("<I", head)[0] == _CTF_MAGIC:
        hdr = _CTF_PACKET["<"]
    elif len(head) == 4 and struct.unpack(">I", head)[0] == _CTF_MAGIC:
        hdr = _CTF_PACKET[">"]
    elif data.startswith(b"/* CTF"):
        return _utf8(data), None
    else:
        raise CtfIdentityError("not_ctf_metadata")
    parts, uuids, off = [], set(), 0
    while off < len(data):
        if len(data) - off < hdr.size:
            raise CtfIdentityError(f"truncated_packet_header@{off}")
        magic, uuid, _, content, size, comp, enc, _, major, minor = hdr.unpack_from(data, off)
        if magic != _CTF_MAGIC:
            raise CtfIdentityError(f"bad_packet_magic@{off}")
        if (major, minor) != (1, 8) or comp or enc:
            raise CtfIdentityError(f"unsupported_packet@{off}")
        if content % 8 or size % 8 or not hdr.size * 8 <= content <= size or off + size // 8 > len(data):
            raise CtfIdentityError(f"bad_packet_size@{off}")
        parts.append(data[off + hdr.size:off + content // 8])
        uuids.add(uuid)
        off += size // 8
    if len(uuids) != 1:
        raise CtfIdentityError(f"packet_uuids:{len(uuids)}")
    return _utf8(b"".join(parts)), uuids.pop()


_TSDL_STRING_RE = re.compile(r'"(?:[^"\\\n]|\\.)*"')


def _tsdl_statements(text: str) -> list[str]:
    """Statements at brace depth 0, split on ``;``: comments dropped, string literals kept."""
    out: list[str] = []
    cur: list[str] = []
    depth, i = 0, 0
    while i < len(text):
        if text.startswith("/*", i):
            end = text.find("*/", i + 2)
            if end < 0:
                raise CtfIdentityError("unterminated_comment")
            cur.append(" ")
            i = end + 2
            continue
        if text.startswith("//", i):
            end = text.find("\n", i)
            i = len(text) if end < 0 else end
            continue
        c = text[i]
        if c == '"':
            m = _TSDL_STRING_RE.match(text, i)
            if m is None:
                raise CtfIdentityError("unterminated_string")
            cur.append(m.group(0))
            i = m.end()
            continue
        depth += {"{": 1, "}": -1}.get(c, 0)
        if depth < 0:
            raise CtfIdentityError("unbalanced_braces")
        if c == ";" and depth == 0:
            out.append("".join(cur).strip())
            cur = []
        else:
            cur.append(c)
        i += 1
    if depth or "".join(cur).strip():
        raise CtfIdentityError("unterminated_statement")
    return [s for s in out if s]


def _tsdl_string(blocks: dict[str, list[str]], block: str, key: str) -> str:
    """The one string assigned to ``key`` at the top of the one ``block { ... }``."""
    if len(blocks.get(block, [])) != 1:
        raise CtfIdentityError(f"{block}_blocks:{len(blocks.get(block, []))}")
    values = [m.group(2).strip() for s in _tsdl_statements(blocks[block][0])
              if (m := _TSDL_ASSIGN_RE.match(s)) and m.group(1) == key]
    if len(values) != 1:
        raise CtfIdentityError(f"{block}.{key}:{len(values)}")
    m = re.fullmatch(r'"([^"\\]*)"', values[0])
    if m is None:
        raise CtfIdentityError(f"{block}.{key}:not_a_plain_string")
    return m.group(1)


def ctf_identity_from_text(text: str) -> tuple[str, str]:
    """``(trace uuid, env.trace_name)`` of TSDL text; anything not exactly one is an error.

    Only the top-level ``trace`` and ``env`` blocks count: the clock's ``uuid`` (the
    same for every run of a host) and the packet header's ``uuid[16]`` field are not
    the trace UUID.
    """
    blocks: dict[str, list[str]] = defaultdict(list)
    for s in _tsdl_statements(text):
        m = _TSDL_BLOCK_RE.match(s)
        if m:
            blocks[m.group(1)].append(m.group(2))
    uuid = _tsdl_string(blocks, "trace", "uuid")
    if not _UUID_RE.match(uuid):
        raise CtfIdentityError("trace.uuid:malformed")
    return uuid, _tsdl_string(blocks, "env", "trace_name")


def _compact(s: str) -> str:
    return "".join(s.split())


def ctf_stream_byte_order(text: str) -> str:
    """Struct byte order of the event-stream packets, once the layout is the pinned one.

    Only the LTTng-UST 2.x 64-bit layout is read (``_LTTNG_TYPEALIASES``,
    ``_LTTNG_PACKET_HEADER``, ``_LTTNG_PACKET_CONTEXT``, every stream's
    ``packet.context := struct packet_context``): every field is byte-aligned, so
    ``packet_size`` sits at a fixed offset. Any other declaration is an error, never
    a guess at where the sizes are.
    """
    statements = [_compact(s) for s in _tsdl_statements(text)]
    for name, want in _LTTNG_TYPEALIASES.items():
        got = [s for s in statements if s.startswith("typealias") and s.endswith(":=" + name)]
        if got != [want]:
            raise CtfIdentityError(f"typealias:{name}")
    if [s for s in statements if s.startswith("structpacket_context{")] != [_LTTNG_PACKET_CONTEXT]:
        raise CtfIdentityError("packet_context")
    blocks: dict[str, list[str]] = defaultdict(list)
    for s in _tsdl_statements(text):
        m = _TSDL_BLOCK_RE.match(s)
        if m:
            blocks[m.group(1)].append(m.group(2))
    trace = [_compact(s) for s in _tsdl_statements(blocks["trace"][0])] if len(blocks["trace"]) == 1 else []
    if [s for s in trace if s.startswith("packet.header")] != [_LTTNG_PACKET_HEADER]:
        raise CtfIdentityError("packet_header")
    order = [s for s in trace if s.startswith("byte_order=")]
    if order not in (["byte_order=le"], ["byte_order=be"]):
        raise CtfIdentityError("byte_order")
    if not blocks["stream"]:
        raise CtfIdentityError("stream_blocks:0")
    for body in blocks["stream"]:
        ctx = [s for s in map(_compact, _tsdl_statements(body)) if s.startswith("packet.context")]
        if ctx != ["packet.context:=structpacket_context"]:
            raise CtfIdentityError("stream_packet_context")
    return "<" if order == ["byte_order=le"] else ">"


def ctf_stream_packets(path: Path, uuid: bytes, order: str) -> int:
    """Packets of one event-stream file, each header checked against the trace ``uuid``.

    Walks the file packet by packet using each packet's own ``packet_size``: every
    packet starts with magic ``0xC1FC1FC1`` and carries ``uuid``, its sizes are whole
    bytes, cover the header and context and fit the file, and the last packet ends
    exactly at end of file. Reads one 64-byte header per packet, so memory is bounded
    and the walk ends after at most ``size // _STREAM_MIN_PACKET`` packets.
    Raises :class:`CtfStreamError` (``uuid_mismatch@<offset>`` or a malformed kind).
    """
    hdr = _STREAM_HEADER[order]
    with open(path, "rb") as f:
        size = os.fstat(f.fileno()).st_size
        if size == 0:
            raise CtfStreamError("empty")
        off, packets = 0, 0
        while off < size:
            f.seek(off)
            raw = f.read(hdr.size)
            if len(raw) < hdr.size:
                raise CtfStreamError(f"truncated_packet_header@{off}")
            magic, packet_uuid, content, packet = hdr.unpack(raw)
            if magic != _STREAM_MAGIC:
                raise CtfStreamError(f"bad_packet_magic@{off}")
            if packet_uuid != uuid:
                raise CtfStreamError(f"uuid_mismatch@{off}")
            if (content % 8 or packet % 8 or not _STREAM_MIN_PACKET * 8 <= content <= packet
                    or off + packet // 8 > size):
                raise CtfStreamError(f"bad_packet_size@{off}")
            off += packet // 8
            packets += 1
    return packets


class CtfStreamError(ValueError):
    """An event-stream file is not a run of well-formed packets of the expected trace."""


def _stream_reasons(stream_dir: Path, run_dir: Path, uuid: bytes,
                    order: str) -> tuple[int, int, list[str]]:
    """``(stream files, packets, reasons)`` for every event stream beside the metadata.

    Every regular file under ``stream_dir`` except ``metadata`` and the ``index/``
    tree (LTTng's ``*.idx`` packet indexes, written by the consumer, not the traced
    process) is an event stream; a symlink or other non-regular entry is a rejection.
    """
    files, packets, reasons = 0, 0, []
    for p in sorted(stream_dir.rglob("*")):
        rel = p.relative_to(stream_dir)
        if rel.parts[0] == "index" or p.name == "metadata" or (p.is_dir() and not p.is_symlink()):
            continue
        name = p.relative_to(run_dir).as_posix()
        if p.is_symlink() or not p.is_file():
            reasons.append(f"ctf_stream_malformed:{name}:not_a_regular_file")
            continue
        files += 1
        try:
            packets += ctf_stream_packets(p, uuid, order)
        except OSError as exc:
            reasons.append(f"ctf_stream_malformed:{name}:{type(exc).__name__}")
        except CtfStreamError as exc:
            kind, _, at = str(exc).partition("uuid_mismatch")
            reasons.append(f"ctf_stream_uuid_mismatch:{name}{at}" if not kind and at
                           else f"ctf_stream_malformed:{name}:{exc}")
    if not files and not reasons:
        reasons.append("ctf_stream_missing:no_event_streams")
    return files, packets, reasons


def ctf_trace_identity(run_dir: Path) -> tuple[Optional[dict[str, Any]], list[str]]:
    """Trace identity of a CTF run, read from its trace bytes: ``(identity, reasons)``.

    The run's ``tracing/`` tree must hold exactly one file named ``metadata`` (LTTng
    per-uid buffers: ``ust/uid/<uid>/<bits>/metadata``); none or several is a
    rejection, not a choice. The trace UUID and ``env.trace_name`` come from those
    bytes alone, never from the run record or the run manifest (both written by the
    runner), and ``trace_name`` must be ``ros2_trace_<run directory name>``. Every
    packet of every event stream beside the metadata must carry that trace UUID
    (:func:`ctf_stream_packets`), so streams copied from another run's trace fail
    with ``ctf_stream_uuid_mismatch`` even when the metadata is the run's own.
    """
    run_dir = Path(run_dir)
    files = sorted(p for p in (run_dir / "tracing").rglob("metadata") if p.is_file())
    if not files:
        return None, ["ctf_identity_missing:no_metadata"]
    if len(files) > 1:
        return None, [f"ctf_identity_ambiguous:{len(files)}_metadata_files"]
    try:
        text, header_uuid = ctf_metadata_text(files[0].read_bytes())
        uuid, name = ctf_identity_from_text(text)
    except OSError as exc:
        return None, [f"ctf_identity_malformed:{type(exc).__name__}"]
    except CtfIdentityError as exc:
        return None, [f"ctf_identity_malformed:{exc}"]
    if header_uuid is not None and header_uuid.hex() != uuid.replace("-", ""):
        return None, ["ctf_identity_malformed:packet_uuid_mismatch"]
    identity: dict[str, Any] = {
        "source": "ctf_metadata", "metadata": files[0].relative_to(run_dir).as_posix(),
        "uuid": uuid, "trace_name": name}
    want = TRACE_NAME_PREFIX + run_dir.resolve().name
    reasons = [] if name == want else [f"ctf_trace_name_mismatch:{name}"]
    try:
        order = ctf_stream_byte_order(text)
    except CtfIdentityError as exc:
        return identity, reasons + [f"ctf_stream_layout_unsupported:{exc}"]
    streams, packets, stream_reasons = _stream_reasons(
        files[0].parent, run_dir, bytes.fromhex(uuid.replace("-", "")), order)
    identity.update(event_streams=streams, stream_packets=packets)
    return identity, reasons + stream_reasons


def check_run(
    run_dir: Path, corpus: Corpus, *,
    protocol_check: Optional[Callable[[Path], list[str]]] = None,
    allow_jsonl_fixture: bool = False,
) -> tuple[dict[str, Any], LogicalTrace]:
    """One run: its row (``passed``, ``reasons``, identity, digest) and its logical trace.

    A run with a ``tracing/`` directory always needs its CTF trace identity. A run
    without one is read from ``semantic_events.jsonl`` and is rejected
    (``ctf_identity_missing:no_tracing_dir``) unless ``allow_jsonl_fixture`` is set,
    a Python-only bypass for synthetic test fixtures. Such a row carries
    ``trace_identity.source = "jsonl_fixture"``: no identity was checked.
    """
    run_dir = Path(run_dir)
    reasons: list[str] = []
    inputs: dict[str, str] = {}
    record = _record(run_dir)
    if record is None:
        reasons.append("no_fixed_input_record" if not (run_dir / RUN_RECORD).exists()
                       else "malformed_record")
        record = {}
    else:
        reasons += _record_reasons(record, corpus)
        hashes = record.get("input_sha256")
        input_reasons, inputs = verify_inputs(run_dir, hashes if isinstance(hashes, dict) else {})
        reasons += input_reasons
    reasons += _metadata_reasons(run_dir, corpus)
    trace_identity: Optional[dict[str, str]] = None
    if (run_dir / "tracing").is_dir():
        trace_identity, identity_reasons = ctf_trace_identity(run_dir)
        reasons += identity_reasons
    elif allow_jsonl_fixture:
        trace_identity = {"source": "jsonl_fixture"}
    else:
        reasons.append("ctf_identity_missing:no_tracing_dir")
    if protocol_check is None:
        from lf_eval.runset import check_run as protocol_check
    reasons += [f"protocol:{r}" for r in protocol_check(run_dir)]

    static = _read_json(run_dir / "rcllf_trace_manifest.json")
    if static is None:
        reasons.append("no_static_manifest")
        static = {}
    events, stats = read_run(run_dir)
    reasons += _stats_reasons(stats)
    trace = canonical_trace(events, static)
    reasons += trace.problems
    reasons += _diff_counter(Counter(expected_inputs(corpus)), Counter(trace.inputs), "corpus")
    try:
        want, cut = expected_dispatches(corpus, static)
        reasons += [f"oracle_cut_off:{c}" for c in cut[:3]]
        reasons += _diff_counter(want, trace.dispatches, "oracle")
    except CorpusError as exc:
        reasons.append(f"oracle_unavailable:{exc}")
    hashes = record.get("input_sha256")
    hashes = hashes if isinstance(hashes, dict) else {}
    row = {
        "run": run_dir.name,
        "passed": not reasons,
        "reasons": reasons,
        "digest": trace.digest,
        # The input file hashes (program, executable, overlay, params, manifests)
        # join the identity, so a rebuild between two runs of a set is reported.
        "identity": dict(trace.identity, corpus_sha256=record.get("corpus_sha256"),
                         lf_timeout_ns=record.get("lf_timeout_ns"),
                         resolved_model_hash=record.get("resolved_model_hash"),
                         **{f"sha256:{k}": v for k, v in sorted(hashes.items())}),
        "inputs": inputs,
        "trace_identity": trace_identity,
        "counts": {"inputs": len(trace.inputs), "dispatches": sum(trace.dispatches.values()),
                   "lines": len(trace.lines)},
    }
    return row, trace


def _stats_reasons(stats: ReadStats) -> list[str]:
    out = [f"trace_discarded:{ch}" for ch, n in sorted(stats.discarded_events.items()) if n]
    out += [f"trace_lost_packets:{ch}" for ch, n in sorted(stats.discarded_packets.items()) if n]
    if stats.read_error:
        out.append(f"trace_read_error:{stats.read_error}")
    return out


def first_divergence(ref: Sequence[str], other: Sequence[str]) -> dict[str, Any]:
    """The first line (in sorted order) present in one trace and not the other."""
    a, b = Counter(ref), Counter(other)
    only_ref, only_other = sorted((a - b).elements()), sorted((b - a).elements())
    return {"only_in_reference": len(only_ref), "only_in_run": len(only_other),
            "first_only_in_reference": only_ref[0] if only_ref else None,
            "first_only_in_run": only_other[0] if only_other else None}


def check_set(
    run_dirs: Sequence[Path], corpus: Corpus, *,
    protocol_check: Optional[Callable[[Path], list[str]]] = None,
    out_dir: Optional[Path] = None,
    allow_jsonl_fixture: bool = False,
) -> dict[str, Any]:
    """Verdict over a run set: ``pass`` only if every check holds on every run.

    ``allow_jsonl_fixture`` is passed to :func:`check_run`; it never relaxes the
    identity check of a run that has a ``tracing/`` directory.
    """
    rows, traces = [], []
    for d in run_dirs:
        row, trace = check_run(Path(d), corpus, protocol_check=protocol_check,
                               allow_jsonl_fixture=allow_jsonl_fixture)
        rows.append(row)
        traces.append(trace)
        if out_dir is not None:
            Path(out_dir).mkdir(parents=True, exist_ok=True)
            (Path(out_dir) / f"{row['run']}.logical.txt").write_text("\n".join(trace.lines) + "\n")
    reasons: list[str] = []
    if len(rows) < corpus.min_repetitions:
        reasons.append(f"too_few_runs:{len(rows)}<{corpus.min_repetitions}")
    paths = [Path(d).resolve() for d in run_dirs]
    for path, count in sorted(Counter(paths).items()):
        if count > 1:
            reasons.append(f"duplicate_run_path:{path.name}")
    for name, count in sorted(Counter(r["run"] for r in rows).items()):
        if count > 1:
            reasons.append(f"duplicate_run_name:{name}")
    indices = [(_record(Path(d)) or {}).get("repeat_index") for d in run_dirs]
    if any(type(i) is not int or i < 0 for i in indices):
        reasons.append("repeat_index_missing_or_invalid")
    elif sorted(indices) != list(range(len(rows))):
        reasons.append(f"repeat_indices:{','.join(map(str, sorted(indices)))}")
    for row, index in zip(rows, indices):
        match = re.search(r"(?:^|_)r(\d+)$", row["run"])
        if match is None or (type(index) is int and int(match.group(1)) != index):
            reasons.append(f"repeat_index_name_mismatch:{row['run']}")
    recorded = [(_record(Path(d)) or {}).get("recorded_at") for d in run_dirs]
    if any(not isinstance(t, str) or not t for t in recorded):
        reasons.append("recorded_at_missing_or_invalid")
    elif len(set(recorded)) != len(recorded):
        reasons.append("duplicate_recorded_at")
    # The record fields above are written by the runner and are distinct even on a
    # trace copied from another run; the CTF trace UUID is not.
    uuids = Counter(r["trace_identity"]["uuid"] for r in rows
                    if (r["trace_identity"] or {}).get("source") == "ctf_metadata")
    reasons += [f"duplicate_ctf_trace_uuid:{u}:x{n}" for u, n in sorted(uuids.items()) if n > 1]
    failed =[r["run"] for r in rows if not r["passed"]]
    if failed:
        reasons.append(f"runs_failed:{','.join(failed)}")
    divergences = []
    if rows:
        ref_id = rows[0]["identity"]
        for r in rows[1:]:
            diff = sorted(k for k in set(ref_id) | set(r["identity"])
                          if r["identity"].get(k) != ref_id.get(k))
            if diff:
                reasons.append(f"identity_mismatch:{r['run']}:{','.join(diff)}")
        for r, t in zip(rows[1:], traces[1:]):
            if r["digest"] != rows[0]["digest"]:
                divergences.append(dict(run=r["run"], reference=rows[0]["run"],
                                        **first_divergence(traces[0].lines, t.lines)))
        if divergences:
            reasons.append(f"not_reproduced:{len(divergences)}_of_{len(rows) - 1}")
    try:
        want, cut = expected_dispatches(corpus, _first_static(run_dirs) or {})
    except CorpusError:
        want, cut = Counter(), []
    return {
        "schema": REPORT_SCHEMA,
        "response": "logical (semantic); physical timing is excluded from every comparison",
        "corpus": {"id": corpus.corpus_id, "path": str(corpus.path), "sha256": corpus.sha256,
                   "horizon_ns": corpus.horizon_ns, "inputs": len(expected_inputs(corpus)),
                   "expected_dispatches": sum(want.values()), "cut_off_chains": len(cut)},
        "verdict": "pass" if not reasons else "fail",
        "reasons": reasons,
        "input_hashes": {"scope": INPUT_HASH_SCOPE,
                         "recorded_only": sorted({n for r in rows for n, s in r["inputs"].items()
                                                  if s == "recorded_only"})},
        "trace_identity": {"scope": TRACE_IDENTITY_SCOPE,
                           "sources": sorted({(r["trace_identity"] or {}).get("source", "none")
                                              for r in rows}),
                           "distinct_ctf_uuids": len(uuids)},
        "digests": sorted({r["digest"] for r in rows}),
        "divergences": divergences,
        "runs": rows,
    }


def _first_static(run_dirs: Sequence[Path]) -> Optional[dict]:
    for d in run_dirs:
        s = _read_json(Path(d) / "rcllf_trace_manifest.json")
        if s is not None:
            return s
    return None
