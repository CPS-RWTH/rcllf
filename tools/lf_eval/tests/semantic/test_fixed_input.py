# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Fixed admitted-input semantic check.

Two kinds of input. The canonical-trace properties run on the recorded
`corr_fanout` golden (a real CTF read). The run-set verdicts run on sentry-shaped
event streams produced by `_simulate` from the corpus and the real sentry static
manifest (golden/sentry_static, copied from `colcon lf-generate`). The recorded
sentry runs (`issue59-fixed-input-lfc/r5_git`) live outside the repo, so they are
not a fixture here, except one real CTF metadata file (golden/sentry_fixed_ctf,
the r0 trace's) for the trace-identity parser. The simulator exercises the
checker's logic only; it is not evidence about the runtime. Run directories carry
synthetic input files whose recorded hashes are their real SHA256, so the archive
input check runs for real.

Simulated runs have no CTF trace, so most tests call ``check_set`` with the
``allow_jsonl_fixture`` bypass (bound below); the trace-identity tests at the end
give each run a CTF metadata file and use the unbound checker.
"""

from __future__ import annotations

import functools
import hashlib
import json
import random
import shutil
import struct
from collections import Counter, defaultdict
from dataclasses import asdict, replace
from pathlib import Path

import pytest
import yaml

from lf_eval.cli import main as lf_eval_main
from lf_eval.semantic import fixed_input, schema
from lf_eval.semantic.fixed_input import (
    REQUIRED_INPUTS,
    CorpusError,
    CtfIdentityError,
    canonical_trace,
    ctf_identity_from_text,
    ctf_metadata_text,
    expected_dispatches,
    expected_inputs,
    first_divergence,
    load_corpus,
)
from lf_eval.semantic.reader import ReadStats, SemEvent, dump_jsonl, load_jsonl

from ..dev_only import golden_absent_reason, requires_golden, requires_lf_bench

needs_lf_bench = requires_lf_bench("the sentry corpus, corpora/sentry_fixed_input_v1.yaml")

#: simulated runs carry no CTF: the explicit fixture bypass, never the CLI default
check_set = functools.partial(fixed_input.check_set, allow_jsonl_fixture=True)

REPO = Path(__file__).resolve().parents[4]
CORPUS = REPO / "tools" / "lf_bench" / "corpora" / "sentry_fixed_input_v1.yaml"
GOLDEN = Path(__file__).parent / "golden"
STATIC = json.loads((GOLDEN / "sentry_static" / "rcllf_trace_manifest.json").read_text())
MS = 1_000_000


@pytest.fixture(scope="module")
def corpus():
    return load_corpus(CORPUS)


# ── corpus and oracle ──────────────────────────────────────────────────────


@needs_lf_bench
def test_corpus_v1_inputs_equal_the_horizon_arithmetic(corpus):
    per_cb = Counter(cb for _, cb in expected_inputs(corpus))
    # floor(2002.5 / period) + 1 releases from tag 0; premise P2 observed the same
    # counts on a plain LF program run with `-o 2002500 usec`.
    assert per_cb == {"ImuNode.timer_callback": 401, "LidarNode.timer_callback": 21,
                      "CameraNode.timer_callback": 51, "PlannerNode.timer_callback": 101}
    assert expected_inputs(corpus)[-1][0] == 2000 * MS


@needs_lf_bench
def test_oracle_counts_on_the_real_sentry_static_manifest(corpus):
    want, cut = expected_dispatches(corpus, STATIC)
    per_cb = Counter()
    for (cb, _), n in want.items():
        per_cb[cb] += n
    assert cut == []
    assert per_cb == {
        "ImuNode.timer_callback": 401, "LidarNode.timer_callback": 21,
        "CameraNode.timer_callback": 51, "PlannerNode.timer_callback": 101,
        "FusionNode.imu_callback": 401, "FusionNode.scan_callback": 21,
        "DetectorNode.image_callback": 51,
        "PlannerNode.state_callback": 422, "PlannerNode.detections_callback": 51,
        "DiagnosticsSinkNode.detections_callback": 51,
        "ControllerNode.plan_callback": 101, "ActuatorSinkNode.cmd_callback": 101,
    }
    # two 1 ms hops: the last release at 2000 ms is consumed at 2002 ms
    assert want[("PlannerNode.state_callback", 1997 * MS)] == 1   # imu at 1995 only
    assert want[("PlannerNode.state_callback", 2002 * MS)] == 2   # imu + lidar at 2000


@needs_lf_bench
def test_a_horizon_inside_a_chain_reports_the_cut_off_hops(corpus):
    short = replace(corpus, horizon_ns=2001_500_000)
    _, cut = expected_dispatches(short, STATIC)
    assert "ImuNode.timer_callback@2000000000 /sentry/imu -> FusionNode.imu_callback@2001000000" not in cut
    assert "FusionNode.imu_callback@2001000000 /sentry/state -> PlannerNode.state_callback@2002000000" in cut


@needs_lf_bench
def test_an_unknown_callback_in_the_corpus_is_refused(corpus):
    bad = replace(corpus, publishes={**corpus.publishes, "GhostNode.cb": ("/x",)})
    with pytest.raises(CorpusError, match="GhostNode.cb is not a reaction"):
        expected_dispatches(bad, STATIC)


@needs_lf_bench
def test_wrong_schema_is_refused(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text(CORPUS.read_text().replace("rcllf.fixed_input_corpus/1", "rcllf.other/1"))
    with pytest.raises(CorpusError, match="expected 'rcllf.fixed_input_corpus/1'"):
        load_corpus(p)


@needs_lf_bench
def test_one_repetition_is_not_a_reproduction_corpus(tmp_path):
    p = tmp_path / "c.yaml"
    data = yaml.safe_load(CORPUS.read_text())
    data["min_repetitions"] = 1
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(CorpusError, match="min_repetitions must be at least 2"):
        load_corpus(p)


# ── canonical trace on a recorded golden ──────────────────────────────────


def _golden(name: str):
    if not (GOLDEN / name).is_dir():
        pytest.skip(golden_absent_reason(GOLDEN / name))
    events, stats = load_jsonl(GOLDEN / name / "semantic_events.jsonl")
    static = json.loads((GOLDEN / name / "rcllf_trace_manifest.json").read_text())
    return events, stats, static


def test_canonical_trace_ignores_gid_bytes_physical_time_and_epoch():
    events, _, static = _golden("corr_fanout")
    ref = canonical_trace(events, static)
    assert ref.problems == []
    new_gid: dict[tuple, list] = {}

    def remap(e: SemEvent) -> SemEvent:
        f = dict(e.fields)
        if isinstance(f.get("gid"), list) and any(f["gid"]):
            # a new process draws new GIDs: a bijection, not the same bytes
            f["gid"] = new_gid.setdefault(tuple(f["gid"]), [len(new_gid) + 1] * 16)
        for k in ("tag_time", "release_tag_time", "lf_start_time_ns", "cutoff"):
            # lf_start (1.1) and cutoff (1.2, -1 none) are tags too; run_begin's 0 is no tag
            if k in f and int(f[k]) > 0 and not (e.name == "run_begin" and k == "lf_start_time_ns"):
                f[k] = int(f[k]) + 3_000_000_000
        return replace(e, fields=f, ts=e.ts + 5_000_000_000, epoch=e.epoch + 1, pid=1)

    moved = canonical_trace([remap(e) for e in events], static)
    assert moved.digest == ref.digest


def test_a_shifted_dispatch_tag_changes_the_digest_and_names_the_line():
    events, _, static = _golden("corr_fanout")
    ref = canonical_trace(events, static)
    i = next(i for i, e in enumerate(events) if e.name == "dispatch_begin" and int(e.fields["entry_kind"]) == 2)
    mutated = list(events)
    f = dict(events[i].fields, tag_microstep=1)
    mutated[i] = replace(events[i], fields=f)
    out = canonical_trace(mutated, static)
    assert out.digest != ref.digest
    div = first_divergence(ref.lines, out.lines)
    assert div["only_in_reference"] == 1 and div["only_in_run"] == 1
    assert div["first_only_in_run"].startswith("D|") and "|1|" in div["first_only_in_run"]


def test_a_missing_dispatch_end_is_a_problem():
    events, _, static = _golden("corr_fanout")
    did = next(int(e.fields["dispatch_id"]) for e in events if e.name == "dispatch_end")
    out = canonical_trace([e for e in events if not (e.name == "dispatch_end"
                                                     and int(e.fields["dispatch_id"]) == did)], static)
    assert any(p.startswith("no_dispatch_end:") for p in out.problems)


def test_lf_start_is_the_tag_origin_and_older_traces_fall_back_to_the_earliest_tag():
    # schema 1.1 (W6): run_begin precedes reactor-c's start time, lf_start carries it
    events, _, static = _golden("corr_fanout")
    # The golden is a schema 1.2 recording. Without its lf_start it reads as an
    # older trace, which is the case under test.
    events = [e for e in events if e.name != "lf_start"]
    old = canonical_trace(events, static)
    assert old.origin == "earliest_tag" and min(t for t, _ in old.inputs) == 0
    rb = next(e for e in events if e.name == "run_begin")
    earliest = min(int(e.fields["tag_time"]) for e in events if e.name == "dispatch_begin")
    lf_start = replace(rb, name="lf_start", cat=schema.event_category("lf_start"),
                       fields={"lf_start_time_ns": earliest - 4 * MS, "lf_start_microstep": 0})
    new = canonical_trace(events + [lf_start], static)
    assert new.origin == "lf_start" and new.problems == []
    assert min(t for t, _ in new.inputs) == 4 * MS
    assert sorted(t - 4 * MS for t, _ in new.inputs) == sorted(t for t, _ in old.inputs)


# ── sentry-shaped run sets (simulated streams, checker logic only) ─────────


def _simulate(corpus, *, seed: int, start: int = 1_790_000_000_000_000_000,
              drop_input=None, extra_legacy=False, swap_microstep=False,
              source_hash="src", identity_faults=0) -> list[SemEvent]:
    """Events a correct runtime would emit for the corpus, in a seed-shuffled physical order."""
    rng = random.Random(seed)
    names = {int(r["rid"]): f"{r['node']}.{r['callback']}" for r in STATIC["rids"]}
    rid_of = {v: k for k, v in names.items()}
    hops = defaultdict(list)
    for e in STATIC["edges"]:
        for rid in e["publisher"]["rids"]:
            hops[(rid, e["topic"])].append((int(e["id"]), int(e["subscriber"]["rid"]), int(e["delay_ns"])))
    gid = {}
    for key in sorted({(names[r].split(".")[0], t) for (r, t) in hops}):
        gid[key] = [rng.randrange(256) for _ in range(16)]
    pending = [(t, 0, rid_of[cb], None) for t, cb in expected_inputs(corpus)
               if (t, cb) != drop_input]
    out, seq, did, ordinals, micro = [], Counter(), 0, Counter(), Counter()

    def ev(name, **fields):
        cat = schema.event_category(name)
        seq[cat] += 1
        out.append(SemEvent(name=name, ts=0, epoch=seed + 1, cat=cat, seq=seq[cat], fields=fields))

    ev("run_begin", static_manifest_hash="h", generated_source_hash=source_hash, model_hash="m",
       build_id="b", executable="/x/SentryPipeline", binding_version="0.1.0")
    while pending:
        pending.sort(key=lambda x: (x[0], x[1], x[2]))
        tag, ms, rid, consumed = pending.pop(0)
        did += 1
        z = [0] * 16
        g, o, edge, kind = (consumed or (z, 0, schema.NO_EDGE, 0))
        if swap_microstep and names[rid] == "PlannerNode.state_callback" and ms == 1:
            ms = 0
        ev("dispatch_begin", dispatch_id=did, rid=rid, entry_kind=kind, edge=edge, gid=g, ordinal=o,
           tag_time=start + tag, tag_microstep=ms)
        for topic in corpus.publishes.get(names[rid], ()):
            key = (names[rid].split(".")[0], topic)
            ordinals[key] += 1
            pg, po = gid[key], ordinals[key]
            ev("publication", gid=pg, ordinal=po, result=0, api=0, origin=0, rid=rid, dispatch_id=did,
               tag_time=start + tag, tag_microstep=ms)
            for eid, sub, delay in hops[(rid, topic)]:
                ev("expectation", gid=pg, ordinal=po, edge=eid, instance=0, generation=0)
                ev("sample", edge=eid, gid=pg, ordinal=po, outcome=0, retained_id=rng.randrange(1, 10**6),
                   arrival_physical_ns=rng.randrange(10**9))
                ev("terminal", gid=pg, ordinal=po, edge=eid, outcome=1)
                t2 = tag + delay
                micro[(sub, t2)] += 1
                pending.append((t2, micro[(sub, t2)] - 1, sub, (pg, po, eid, 2)))
        ev("dispatch_end", dispatch_id=did, outcome=1 if consumed else 255, retained_id=0,
           join_wait_ns=rng.randrange(10**6), callback_ran=1)
    if extra_legacy:
        did += 1
        ev("dispatch_begin", dispatch_id=did, rid=rid_of["PlannerNode.state_callback"], entry_kind=1,
           edge=schema.NO_EDGE, gid=[0] * 16, ordinal=0, tag_time=start + 7 * MS, tag_microstep=0)
        ev("dispatch_end", dispatch_id=did, outcome=255, retained_id=0, join_wait_ns=0, callback_ran=1)
    for key in sorted(gid):
        ev("run_end_publisher", gid=gid[key], tracked=1, accepted=ordinals[key], failed=0,
           refused_no_context=0, refused_undeclared_effect=0, refused_edge_not_admitted=0, nested=0,
           unsupported_api=0, first_traced_ordinal=1, untracked=0)
    # an untracked publisher: the recorded r5_git runs end with 22 of them and
    # untracked_publishes=2603, so these publishes are not a fault
    ev("run_end_publisher", gid=[0xEE] * 16, tracked=0, accepted=0, failed=0, refused_no_context=0,
       refused_undeclared_effect=0, refused_edge_not_admitted=0, nested=0, unsupported_api=0,
       first_traced_ordinal=0, untracked=7)
    per_edge = Counter(int(e.fields["edge"]) for e in out if e.name == "expectation")
    for eid in sorted(int(e["id"]) for e in STATIC["edges"]):
        ev("run_end_edge", edge=eid, expectations=per_edge[eid], matched=per_edge[eid], terminal_other=0,
           samples_taken=per_edge[eid], samples_retained=per_edge[eid], orphan_dispatches=0,
           count_mismatch=0)
    ev("run_end", totals=[], attempts=[], n_publishers=len(gid) + 1, n_edges=8, untracked_publishes=7,
       identity_faults=identity_faults, reconciled=1)
    # Physical order: same-tag events of different reactors interleave per run.
    rng.shuffle(out)
    return out


#: Synthetic stand-ins for the set-level inputs. Their bytes are made up; their
#: recorded hashes are the real SHA256 of those bytes, so the archive check runs.
GLOBAL_INPUTS = {"executable": b"\x7fELF synthetic executable\n", "program.lf": b"target C;\n",
                 "resolved_model.json": b'{"synthetic": true}\n', "exec_config.json": b"{}\n"}


#: the run-local inputs a run directory carries (synthetic content, real hashes)
LOCAL_INPUTS = {"rcllf_trace_manifest.json": json.dumps(STATIC).encode(),
                "system.json": b'{"synthetic": "system"}\n',
                "scenario_overlay.env": b"SENTRY_SYNTHETIC=1\n",
                "scenario_params.yaml": b"synthetic: true\n"}


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _real_hashes() -> dict[str, str]:
    return {k: _sha(v) for k, v in {**LOCAL_INPUTS, **GLOBAL_INPUTS}.items()}


def _archive_inputs(root: Path, subdir: str = "inputs") -> Path:
    d = root / subdir
    d.mkdir(parents=True, exist_ok=True)
    for name, data in GLOBAL_INPUTS.items():
        (d / name).write_bytes(data)
    return d


def _run_dir(tmp_path, corpus, name, events, *, corpus_sha=None, launcher=None,
             input_sha256=None, metadata=None, record=None) -> Path:
    """A run directory shaped like ``lf-bench fixed-input`` output, inputs hashed for real."""
    d = tmp_path / name
    d.mkdir()
    dump_jsonl(events, ReadStats(), d / "semantic_events.jsonl")
    for fname, data in LOCAL_INPUTS.items():
        (d / fname).write_bytes(data)
    (d / "metadata.yaml").write_text(yaml.safe_dump(metadata if metadata is not None else {
        "mode": "lfc", "scenario": corpus.corpus_id, "corpus_id": corpus.corpus_id,
        "corpus_sha256": corpus.sha256, "repeat_index": int(name[1:])}))
    (d / "fixed_input.json").write_text(json.dumps(record if record is not None else {
        "corpus_id": corpus.corpus_id,
        "corpus_sha256": corpus_sha or corpus.sha256, "lf_timeout_ns": corpus.horizon_ns,
        "repeat_index": int(name[1:]),
        "recorded_at": f"2026-10-04T03:39:{int(name[1:]):02d}Z",
        "launcher": launcher or {"returncode": 0, "signal": None, "stopped_by_driver": False},
        "resolved_model_hash": "sha256:m",
        "input_sha256": input_sha256 if input_sha256 is not None else _real_hashes()}))
    return d


def _ok(_run_dir):
    return []


@needs_lf_bench
def test_five_independent_correct_runs_pass_with_one_digest(tmp_path, corpus):
    _archive_inputs(tmp_path)
    dirs = [_run_dir(tmp_path, corpus, f"r{i}", _simulate(corpus, seed=i)) for i in range(5)]
    rep = check_set(dirs, corpus, protocol_check=_ok)
    assert rep["verdict"] == "pass", rep["reasons"]
    assert len(rep["digests"]) == 1
    assert set(rep["runs"][0]["inputs"].values()) == {"verified:run", "verified:inputs"}
    assert rep["input_hashes"]["recorded_only"] == []
    assert rep["runs"][0]["counts"]["inputs"] == 574
    assert rep["runs"][0]["counts"]["dispatches"] == 1773
    assert rep["corpus"]["expected_dispatches"] == 1773


@needs_lf_bench
def test_repeating_one_valid_run_does_not_count_as_five_repetitions(tmp_path, corpus):
    run = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    rep = check_set([run] * 5, corpus, protocol_check=_ok)
    assert rep["verdict"] == "fail"
    assert "duplicate_run_path:r0" in rep["reasons"]
    assert "duplicate_run_name:r0" in rep["reasons"]
    assert "repeat_indices:0,0,0,0,0" in rep["reasons"]
    assert "duplicate_recorded_at" in rep["reasons"]


@needs_lf_bench
def test_renamed_copies_are_not_independent_repetitions(tmp_path, corpus):
    original = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    dirs = [original]
    for i in range(1, 5):
        copy = tmp_path / f"r{i}"
        shutil.copytree(original, copy)
        record = json.loads((copy / "fixed_input.json").read_text())
        record["repeat_index"] = i
        (copy / "fixed_input.json").write_text(json.dumps(record))
        metadata = yaml.safe_load((copy / "metadata.yaml").read_text())
        metadata["repeat_index"] = i
        (copy / "metadata.yaml").write_text(yaml.safe_dump(metadata))
        dirs.append(copy)
    report = check_set(dirs, corpus, protocol_check=_ok)
    assert report["verdict"] == "fail"
    assert report["reasons"] == ["duplicate_recorded_at"]


@needs_lf_bench
def test_a_missing_admitted_input_fails_corpus_and_oracle(tmp_path, corpus):
    dirs = [_run_dir(tmp_path, corpus, f"r{i}", _simulate(corpus, seed=i)) for i in range(4)]
    dirs.append(_run_dir(tmp_path, corpus, "r4", _simulate(
        corpus, seed=4, drop_input=(1000 * MS, "ImuNode.timer_callback"))))
    rep = check_set(dirs, corpus, protocol_check=_ok)
    assert rep["verdict"] == "fail"
    bad = rep["runs"][4]["reasons"]
    assert "corpus_missing:1:first=(1000000000, 'ImuNode.timer_callback')" in bad
    assert "oracle_missing:3:first=('FusionNode.imu_callback', 1001000000)" in bad
    assert "runs_failed:r4" in rep["reasons"]
    assert "not_reproduced:1_of_4" in rep["reasons"]


@needs_lf_bench
def test_an_unadmitted_entry_fails_closure(tmp_path, corpus):
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0, extra_legacy=True))
    rep = check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)
    assert "unadmitted_entry:LEGACY_SUB:PlannerNode.state_callback@7000000" in rep["runs"][0]["reasons"]


@needs_lf_bench
def test_a_divergent_microstep_is_not_reproduced_and_located(tmp_path, corpus):
    dirs = [_run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0)),
            _run_dir(tmp_path, corpus, "r1", _simulate(corpus, seed=1, swap_microstep=True))]
    rep = check_set(dirs, replace(corpus, min_repetitions=2), protocol_check=_ok)
    assert rep["verdict"] == "fail"
    assert rep["reasons"] == ["not_reproduced:1_of_1"]
    div = rep["divergences"][0]
    # the earliest swapped microstep: imu and lidar at 0 -> two state samples at 2 ms
    assert div["run"] == "r1"
    assert div["first_only_in_reference"].startswith("D|0000002000000|1|PlannerNode.state_callback")


@needs_lf_bench
def test_changed_corpus_driver_stop_and_identity_are_each_reported(tmp_path, corpus):
    dirs = [
        _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0)),
        _run_dir(tmp_path, corpus, "r1", _simulate(corpus, seed=1), corpus_sha="0" * 64),
        _run_dir(tmp_path, corpus, "r2", _simulate(corpus, seed=2),
                 launcher={"returncode": None, "signal": 15, "stopped_by_driver": True}),
        _run_dir(tmp_path, corpus, "r3", _simulate(corpus, seed=3, source_hash="other")),
    ]
    rep = check_set(dirs, corpus, protocol_check=_ok)
    assert "corpus_changed" in rep["runs"][1]["reasons"]
    assert "not_self_terminated:rc=None,signal=15,driver=True" in rep["runs"][2]["reasons"]
    assert "identity_mismatch:r1:corpus_sha256" in rep["reasons"]
    assert "identity_mismatch:r3:generated_source_hash" in rep["reasons"]
    assert "too_few_runs:4<5" in rep["reasons"]


@needs_lf_bench
def test_a_missing_input_file_and_a_rebuild_between_runs_are_reported(tmp_path, corpus):
    # No archived set-level inputs here: the global hashes are recorded_only and
    # only the cross-run identity comparison sees the rebuild.
    hashes = _real_hashes()
    dirs = [
        _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0), input_sha256=hashes),
        _run_dir(tmp_path, corpus, "r1", _simulate(corpus, seed=1),
                 input_sha256={**hashes, "executable": _sha(b"rebuilt")}),
        _run_dir(tmp_path, corpus, "r2", _simulate(corpus, seed=2),
                 input_sha256={**hashes, "program.lf": None}),
    ]
    rep = check_set(dirs, replace(corpus, min_repetitions=3), protocol_check=_ok)
    assert rep["runs"][0]["passed"] and rep["runs"][1]["passed"]
    assert rep["runs"][2]["reasons"] == ["input_missing:program.lf"]
    assert rep["runs"][0]["inputs"]["executable"] == "recorded_only"
    assert rep["input_hashes"]["recorded_only"] == ["exec_config.json", "executable", "program.lf",
                                                   "resolved_model.json"]
    assert "identity_mismatch:r1:sha256:executable" in rep["reasons"]
    assert "identity_mismatch:r2:sha256:program.lf" in rep["reasons"]
    # same logical trace: a rebuild is an identity failure, not a reproduction one
    assert not any(r.startswith("not_reproduced") for r in rep["reasons"])


# ── F2: recorded input hashes against archived copies ──────────────────────


@needs_lf_bench
@pytest.mark.parametrize("subdir", ["inputs", "campaign/inputs"])
def test_an_archived_global_input_that_does_not_hash_to_the_record_fails(tmp_path, corpus, subdir):
    archive = _archive_inputs(tmp_path, subdir)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    one = replace(corpus, min_repetitions=1)
    assert check_set([d], one, protocol_check=_ok)["verdict"] == "pass"
    (archive / "executable").write_bytes(b"\x7fELF a different build\n")
    row = check_set([d], one, protocol_check=_ok)["runs"][0]
    assert row["reasons"] == [f"input_hash_mismatch:executable@{subdir}"]
    assert row["inputs"]["executable"] == "mismatch"
    assert row["inputs"]["program.lf"] == f"verified:{subdir}"


@needs_lf_bench
def test_both_archive_dirs_are_checked_and_each_copy_must_match(tmp_path, corpus):
    _archive_inputs(tmp_path, "inputs")
    _archive_inputs(tmp_path, "campaign/inputs")
    (tmp_path / "campaign" / "inputs" / "program.lf").write_bytes(b"target Cpp;\n")
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    row = check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0]
    assert row["reasons"] == ["input_hash_mismatch:program.lf@campaign/inputs"]
    assert row["inputs"]["executable"] == "verified:inputs,campaign/inputs"


@needs_lf_bench
@pytest.mark.parametrize("fname", ["system.json", "scenario_overlay.env", "scenario_params.yaml",
                                   "rcllf_trace_manifest.json"])
def test_a_run_local_input_that_changed_or_is_absent_fails(tmp_path, corpus, fname):
    _archive_inputs(tmp_path)
    one = replace(corpus, min_repetitions=1)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    path = d / fname
    if fname == "rcllf_trace_manifest.json":
        # still valid JSON with the same content, other bytes
        path.write_text(json.dumps(STATIC, indent=1))
    else:
        path.write_bytes(path.read_bytes() + b"\n")
    assert check_set([d], one, protocol_check=_ok)["runs"][0]["reasons"] == [f"input_hash_mismatch:{fname}"]
    if fname != "rcllf_trace_manifest.json":
        path.unlink()
        assert check_set([d], one, protocol_check=_ok)["runs"][0]["reasons"] == [f"input_file_absent:{fname}"]


@needs_lf_bench
def test_an_input_recorded_absent_with_an_archived_copy_fails(tmp_path, corpus):
    _archive_inputs(tmp_path)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    rec = json.loads((d / "fixed_input.json").read_text())
    rec["input_sha256"]["exec_config.json"] = None
    (d / "fixed_input.json").write_text(json.dumps(rec))
    row = check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0]
    assert row["reasons"] == ["input_hash_mismatch:exec_config.json@inputs"]


@needs_lf_bench
def test_a_path_like_input_name_is_never_resolved(tmp_path, corpus):
    _archive_inputs(tmp_path)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    rec = json.loads((d / "fixed_input.json").read_text())
    rec["input_sha256"]["../inputs/program.lf"] = rec["input_sha256"]["program.lf"]
    (d / "fixed_input.json").write_text(json.dumps(rec))
    row = check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0]
    assert row["reasons"] == ["malformed_record:input_name:'../inputs/program.lf'"]


# ── F3: metadata identity, trace counters, malformed records ───────────────


@needs_lf_bench
@pytest.mark.parametrize("change,reason", [
    ({"mode": "stock-single"}, "mode_not_lfc:'stock-single'"),
    ({"mode": None}, "mode_not_lfc:None"),
    ({"scenario": "default"}, "metadata_scenario_mismatch:'default'"),
    ({"corpus_id": "other"}, "metadata_corpus_id_mismatch:'other'"),
    ({"corpus_sha256": "0" * 64}, f"metadata_corpus_sha256_mismatch:'{'0' * 64}'"),
])
def test_metadata_that_is_not_an_lfc_run_of_this_corpus_fails(tmp_path, corpus, change, reason):
    _archive_inputs(tmp_path)
    md = {"mode": "lfc", "scenario": corpus.corpus_id, "corpus_id": corpus.corpus_id,
          "corpus_sha256": corpus.sha256}
    md = {k: v for k, v in {**md, **change}.items() if v is not None}
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0), metadata=md)
    assert check_set([d], replace(corpus, min_repetitions=1),
                     protocol_check=_ok)["runs"][0]["reasons"] == [reason]


@needs_lf_bench
def test_absent_metadata_fails(tmp_path, corpus):
    _archive_inputs(tmp_path)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    (d / "metadata.yaml").unlink()
    assert check_set([d], replace(corpus, min_repetitions=1),
                     protocol_check=_ok)["runs"][0]["reasons"] == ["no_metadata"]


@needs_lf_bench
def test_a_record_of_another_corpus_id_fails(tmp_path, corpus):
    _archive_inputs(tmp_path)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    rec = json.loads((d / "fixed_input.json").read_text())
    (d / "fixed_input.json").write_text(json.dumps({**rec, "corpus_id": "sentry-fixed-input-v0"}))
    assert check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0][
        "reasons"] == ["corpus_id_mismatch:'sentry-fixed-input-v0'"]


@needs_lf_bench
@pytest.mark.parametrize("field,value,reason", [
    ("lf_timeout_ns", "2002500000", "malformed_record:lf_timeout_ns"),
    ("lf_timeout_ns", "abc", "malformed_record:lf_timeout_ns"),
    ("lf_timeout_ns", 2002500000.0, "malformed_record:lf_timeout_ns"),
    ("launcher", [0], "malformed_record:launcher"),
    ("input_sha256", ["executable"], "malformed_record:input_sha256"),
])
def test_a_malformed_record_field_is_a_rejection_not_a_crash(tmp_path, corpus, field, value, reason):
    _archive_inputs(tmp_path)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    rec = json.loads((d / "fixed_input.json").read_text())
    (d / "fixed_input.json").write_text(json.dumps({**rec, field: value}))
    reasons = check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0]["reasons"]
    assert reason in reasons


@needs_lf_bench
@pytest.mark.parametrize("body", ["[1, 2]", "\"x\"", "{not json", ""])
def test_a_record_that_is_not_a_json_object_is_rejected(tmp_path, corpus, body):
    _archive_inputs(tmp_path)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    (d / "fixed_input.json").write_text(body)
    rep = check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)
    assert "malformed_record" in rep["runs"][0]["reasons"]
    assert "repeat_index_missing_or_invalid" in rep["reasons"]


@needs_lf_bench
def test_a_bool_returncode_is_not_a_clean_exit(tmp_path, corpus):
    _archive_inputs(tmp_path)
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0),
                 launcher={"returncode": False, "signal": None, "stopped_by_driver": False})
    assert check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0][
        "reasons"] == ["not_self_terminated:rc=False,signal=None,driver=False"]


def _check_one(tmp_path, corpus, events) -> list[str]:
    _archive_inputs(tmp_path)
    d = _run_dir(tmp_path, corpus, "r0", events)
    return check_set([d], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0]["reasons"]


@needs_lf_bench
def test_no_run_begin_fails(tmp_path, corpus):
    events = [e for e in _simulate(corpus, seed=0) if e.name != "run_begin"]
    assert "no_run_begin" in _check_one(tmp_path, corpus, events)


@needs_lf_bench
def test_nonzero_identity_faults_fail(tmp_path, corpus):
    assert _check_one(tmp_path, corpus, _simulate(corpus, seed=0, identity_faults=3)) == ["identity_faults:3"]


def _with_fields(events, name, pred, **fields):
    out, hit = [], 0
    for e in events:
        if e.name == name and pred(e) and not hit:
            e, hit = replace(e, fields=dict(e.fields, **fields)), 1
        out.append(e)
    assert hit
    return out


@needs_lf_bench
@pytest.mark.parametrize("field", ["orphan_dispatches", "count_mismatch"])
def test_a_producer_edge_counter_fails(tmp_path, corpus, field):
    events = _with_fields(_simulate(corpus, seed=0), "run_end_edge",
                          lambda e: int(e.fields["edge"]) == 4, **{field: 1})
    assert _check_one(tmp_path, corpus, events) == [f"edge_{field}:edge4=1"]


@needs_lf_bench
def test_an_admitted_writer_left_untracked_fails(tmp_path, corpus):
    events = _simulate(corpus, seed=0)
    imu = next(e.gid() for e in events if e.name == "expectation" and int(e.fields["edge"]) == 4)
    events = _with_fields(events, "run_end_publisher", lambda e: e.gid() == imu, tracked=0)
    assert _check_one(tmp_path, corpus, events) == ["tracked_writers:6!=7"]


@needs_lf_bench
def test_a_tracked_writer_fault_counter_fails(tmp_path, corpus):
    events = _simulate(corpus, seed=0)
    imu = next(e.gid() for e in events if e.name == "expectation" and int(e.fields["edge"]) == 4)
    events = _with_fields(events, "run_end_publisher", lambda e: e.gid() == imu, refused_no_context=2)
    assert _check_one(tmp_path, corpus, events) == [
        "publisher_refused_no_context:ImuNode:/sentry/imu=2"]


# ── F4: each consumed key against its publication and the static edge ──────

IMU_EDGE = 4   # ImuNode.timer_callback /sentry/imu -> FusionNode.imu_callback, 1 ms


def _imu_dispatches(events):
    return sorted((e for e in events if e.name == "dispatch_begin" and int(e.fields["edge"]) == IMU_EDGE),
                  key=lambda e: int(e.fields["ordinal"]))


@needs_lf_bench
def test_swapped_consumed_ordinals_keep_the_old_checks_and_fail_the_tag_tie(tmp_path, corpus):
    events = _simulate(corpus, seed=0)
    a, b = _imu_dispatches(events)[:2]          # ordinals 1 (tag 1 ms) and 2 (tag 6 ms)
    swap = {a.fields["dispatch_id"]: b.fields["ordinal"], b.fields["dispatch_id"]: a.fields["ordinal"]}
    events = [replace(e, fields=dict(e.fields, ordinal=swap[e.fields["dispatch_id"]]))
              if e.name == "dispatch_begin" and e.fields["dispatch_id"] in swap else e for e in events]
    reasons = _check_one(tmp_path, corpus, events)
    # every key is still a published one and the (callback, tag) multiset is
    # unchanged: only the per-dispatch tie sees the swap
    assert not any(r.startswith(("orphan_dispatch", "oracle_", "corpus_")) for r in reasons)
    assert reasons == [
        "consumed_tag_mismatch:FusionNode.imu_callback@1000000:ImuNode:/sentry/imu#2:delta=-5000000",
        "consumed_tag_mismatch:FusionNode.imu_callback@6000000:ImuNode:/sentry/imu#1:delta=5000000",
    ]


@needs_lf_bench
def test_a_key_consumed_twice_fails(tmp_path, corpus):
    events = _simulate(corpus, seed=0)
    a, b = _imu_dispatches(events)[:2]
    events = _with_fields(events, "dispatch_begin",
                          lambda e: e.fields["dispatch_id"] == b.fields["dispatch_id"],
                          ordinal=a.fields["ordinal"])
    reasons = _check_one(tmp_path, corpus, events)
    assert "duplicate_consumption:edge4:ImuNode:/sentry/imu#1:x2" in reasons
    assert "consumed_tag_mismatch:FusionNode.imu_callback@6000000:ImuNode:/sentry/imu#1:delta=5000000" in reasons


@needs_lf_bench
def test_a_dispatch_on_another_reactions_edge_fails(tmp_path, corpus):
    events = _simulate(corpus, seed=0)
    a = _imu_dispatches(events)[0]
    # edge 6 is LidarNode /sentry/scan -> FusionNode.scan_callback, not imu_callback
    events = _with_fields(events, "dispatch_begin",
                          lambda e: e.fields["dispatch_id"] == a.fields["dispatch_id"], edge=6)
    reasons = _check_one(tmp_path, corpus, events)
    assert "edge_subscriber_mismatch:FusionNode.imu_callback@1000000:ImuNode:/sentry/imu#1:edge6" in reasons
    assert "edge_publisher_mismatch:FusionNode.imu_callback@1000000:ImuNode:/sentry/imu#1:ImuNode.timer_callback" \
        in reasons


@needs_lf_bench
def test_a_dispatch_on_an_unknown_edge_fails(tmp_path, corpus):
    events = _simulate(corpus, seed=0)
    a = _imu_dispatches(events)[0]
    events = _with_fields(events, "dispatch_begin",
                          lambda e: e.fields["dispatch_id"] == a.fields["dispatch_id"], edge=99)
    assert "unadmitted_edge:FusionNode.imu_callback@1000000:ImuNode:/sentry/imu#1:edge99" in \
        _check_one(tmp_path, corpus, events)


@needs_lf_bench
def test_protocol_exclusions_fail_the_run(tmp_path, corpus):
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    rep = check_set([d], replace(corpus, min_repetitions=1),
                    protocol_check=lambda _: ["trace_loss:rcllf"])
    assert rep["runs"][0]["reasons"] == ["protocol:trace_loss:rcllf"]


@needs_lf_bench
def test_cli_exit_status_and_outputs(tmp_path, corpus, monkeypatch, capsys):
    import lf_eval.runset as runset

    monkeypatch.setattr(runset, "check_run", lambda run_dir, window_total_s=None: [])
    dirs = [_run_dir(tmp_path, corpus, f"r{i}", _simulate(corpus, seed=i)) for i in range(5)]
    out = tmp_path / "check"
    # The CLI has no fixture bypass: runs without a CTF trace fail closed.
    rc = lf_eval_main(["fixed-input", "--corpus", str(CORPUS), *map(str, dirs)])
    assert rc == 1
    assert "ctf_identity_missing:no_tracing_dir" in capsys.readouterr().out
    monkeypatch.setattr(fixed_input, "check_set", check_set)
    rc = lf_eval_main(["fixed-input", "--corpus", str(CORPUS), *map(str, dirs), "--out", str(out)])
    assert rc == 0, capsys.readouterr().out
    rep = json.loads((out / "fixed_input_report.json").read_text())
    assert rep["verdict"] == "pass"
    assert len((out / "r0.logical.txt").read_text().splitlines()) == rep["runs"][0]["counts"]["lines"]
    rc = lf_eval_main(["fixed-input", "--corpus", str(CORPUS), *map(str, dirs[:2])])
    assert rc == 1
    assert "FAIL too_few_runs:2<5" in capsys.readouterr().out


@needs_lf_bench
def test_jsonl_roundtrip_keeps_the_digest(tmp_path, corpus):
    events = _simulate(corpus, seed=9)
    p = dump_jsonl(events, ReadStats(), tmp_path / "e.jsonl")
    back, _ = load_jsonl(p)
    assert canonical_trace(back, STATIC).digest == canonical_trace(events, STATIC).digest
    assert all(asdict(a) == asdict(b) for a, b in zip(events, back))


# --- trace identity: each run's own CTF trace -------------------------------------------

REAL_CTF = GOLDEN / "sentry_fixed_ctf" / "metadata"
REAL_STREAM = GOLDEN / "sentry_fixed_ctf" / "rcllf_17"
needs_real_ctf = requires_golden(GOLDEN / "sentry_fixed_ctf")
REAL_R0 = "20261004T033917Z_lfc_sentry-fixed-input-v1_r0"
CTF_HEADER = struct.Struct("<I16sIIIBBBBB")
CLOCK_UUID = "a2e55453-5a87-48d5-acbe-e39e2ef6c39d"


#: the LTTng-UST 2.13 declarations the stream check pins, as in the real metadata
LTTNG_DECLS = "".join(
    f"typealias integer {{ size = {bits}; align = 8; signed = false;{extra} }} := {name};\n"
    for name, bits, extra in (("uint8_t", 8, ""), ("uint32_t", 32, ""), ("uint64_t", 64, ""),
                              ("unsigned long", 64, ""),
                              ("uint64_clock_monotonic_t", 64, " map = clock.monotonic.value;"))
) + ("struct packet_context {\n\tuint64_clock_monotonic_t timestamp_begin;\n"
     "\tuint64_clock_monotonic_t timestamp_end;\n\tuint64_t content_size;\n\tuint64_t packet_size;\n"
     "\tuint64_t packet_seq_num;\n\tunsigned long events_discarded;\n\tuint32_t cpu_id;\n};\n"
     "stream {\n\tid = 0;\n\tpacket.context := struct packet_context;\n};\n")
STREAM_HEADER = struct.Struct("<I16sIQQQQQQQI")


def _tsdl(uuid: str, trace_name: str, *, trace_extra: str = "", env_extra: str = "",
          byte_order: str = "le", decls: str = LTTNG_DECLS) -> str:
    return (f'/* CTF 1.8 */\ntrace {{\n\tmajor = 1;\n\tminor = 8;\n\tuuid = "{uuid}";\n'
            f"\tbyte_order = {byte_order};\n\tpacket.header := struct {{\n\t\tuint32_t magic;\n"
            f"\t\tuint8_t  uuid[16];\n\t\tuint32_t stream_id;\n\t\tuint64_t stream_instance_id;\n"
            f"\t}};\n{trace_extra}}};\n\n"
            f'env {{\n\thostname = "h";\n\ttrace_name = "{trace_name}";\n{env_extra}}};\n\n'
            f'clock {{\n\tname = "monotonic";\n\tuuid = "{CLOCK_UUID}";\n}};\n{decls}')


def _stream(*uuids: str, packet: int = 256, content: int = 100, order: str = "<",
            magic: int = 0xC1FC1FC1) -> bytes:
    """An event-stream file as LTTng writes it: one zero-padded packet per uuid."""
    hdr = struct.Struct(order + STREAM_HEADER.format[1:])
    return b"".join(
        hdr.pack(magic, bytes.fromhex(u.replace("-", "")), 0, 0, 0, 0, content * 8, packet * 8, i, 0, 0)
        + b"\0" * (packet - hdr.size) for i, u in enumerate(uuids))


def _packetize(text: str, uuid: str, *, packet: int = 128) -> bytes:
    """Packetized CTF 1.8 metadata as LTTng writes it: fixed-size, zero-padded packets."""
    data, body, out = text.encode(), packet - CTF_HEADER.size, b""
    for i in range(0, len(data), body):
        chunk = data[i:i + body]
        out += CTF_HEADER.pack(0x75D11D57, bytes.fromhex(uuid.replace("-", "")), 0,
                               (CTF_HEADER.size + len(chunk)) * 8, packet * 8, 0, 0, 0, 1, 8)
        out += chunk + b"\0" * (body - len(chunk))
    return out


def _uuid(i: int) -> str:
    return f"{i:08x}-0000-4000-8000-{i:012x}"


def _give_trace(d: Path, metadata: bytes, streams: dict[str, bytes]) -> Path:
    """A per-uid LTTng trace: metadata, event streams and an ``index/`` that is never read."""
    m = d / "tracing" / "ust" / "uid" / "1000" / "64-bit" / "metadata"
    (m.parent / "index").mkdir(parents=True)
    m.write_bytes(metadata)
    for name, data in streams.items():
        (m.parent / name).write_bytes(data)
        (m.parent / "index" / f"{name}.idx").write_bytes(b"\xc1\xf1\xdc\xc1 not a stream")
    return m


@pytest.fixture
def ctf_runs(tmp_path, corpus, monkeypatch):
    """Five simulated runs, each with its own CTF trace; events still come from JSONL."""
    monkeypatch.setattr(fixed_input, "read_run", lambda d: load_jsonl(Path(d) / "semantic_events.jsonl"))
    dirs = [_run_dir(tmp_path, corpus, f"r{i}", _simulate(corpus, seed=i)) for i in range(5)]
    for i, d in enumerate(dirs):
        u = _uuid(i + 1)
        _give_trace(d, _packetize(_tsdl(u, f"ros2_trace_{d.name}"), u),
                    {"rcllf_0": _stream(u, u, u), "ros2_0": _stream(u)})
    return dirs


@needs_real_ctf
def test_the_real_metadata_names_its_own_trace_not_the_clock():
    text, header_uuid = ctf_metadata_text(REAL_CTF.read_bytes())
    assert ctf_identity_from_text(text) == ("47a19e4a-c238-4ff1-9e6d-622a341fb5ed", f"ros2_trace_{REAL_R0}")
    assert header_uuid.hex() == "47a19e4ac2384ff19e6d622a341fb5ed"
    assert CLOCK_UUID in text


@needs_lf_bench
def test_runs_with_their_own_ctf_traces_pass_without_the_bypass(ctf_runs, corpus):
    rep = fixed_input.check_set(ctf_runs, corpus, protocol_check=_ok)
    assert rep["verdict"] == "pass", rep["reasons"]
    assert rep["trace_identity"]["sources"] == ["ctf_metadata"]
    assert rep["trace_identity"]["distinct_ctf_uuids"] == 5


@needs_lf_bench
def test_a_trace_copied_into_otherwise_distinct_runs_fails_on_the_trace(ctf_runs, corpus):
    # Records (repeat_index, recorded_at), metadata.yaml, names and paths stay
    # distinct; only the trace is r0's. The old checker passed this set.
    for d in ctf_runs[1:]:
        shutil.rmtree(d / "tracing")
        shutil.copytree(ctf_runs[0] / "tracing", d / "tracing")
    rep = fixed_input.check_set(ctf_runs, corpus, protocol_check=_ok)
    assert rep["verdict"] == "fail"
    assert rep["reasons"] == [f"duplicate_ctf_trace_uuid:{_uuid(1)}:x5", "runs_failed:r1,r2,r3,r4"]
    assert [r["reasons"] for r in rep["runs"]] == [[]] + [["ctf_trace_name_mismatch:ros2_trace_r0"]] * 4


@needs_lf_bench
def test_a_copied_trace_renamed_to_the_run_still_shares_its_uuid(ctf_runs, corpus):
    shutil.rmtree(ctf_runs[1] / "tracing")
    shutil.copytree(ctf_runs[0] / "tracing", ctf_runs[1] / "tracing")
    renamed = _packetize(_tsdl(_uuid(1), "ros2_trace_r1"), _uuid(1))
    next((ctf_runs[1] / "tracing").rglob("metadata")).write_bytes(renamed)
    rep = fixed_input.check_set(ctf_runs, corpus, protocol_check=_ok)
    assert rep["reasons"] == [f"duplicate_ctf_trace_uuid:{_uuid(1)}:x2"]


@needs_real_ctf
@needs_lf_bench
def test_the_real_r0_trace_in_a_second_real_run_directory_fails(tmp_path, corpus, monkeypatch):
    monkeypatch.setattr(fixed_input, "read_run", lambda d: load_jsonl(Path(d) / "semantic_events.jsonl"))
    r1 = "20261004T033927Z_lfc_sentry-fixed-input-v1_r1"
    dirs = []
    for i, name in enumerate((REAL_R0, r1)):
        d = _run_dir(tmp_path, corpus, f"r{i}", _simulate(corpus, seed=i)).rename(tmp_path / name)
        _give_trace(d, REAL_CTF.read_bytes(), {"rcllf_17": REAL_STREAM.read_bytes()})
        dirs.append(d)
    rep = fixed_input.check_set(dirs, replace(corpus, min_repetitions=2), protocol_check=_ok)
    assert rep["reasons"] == ["duplicate_ctf_trace_uuid:47a19e4a-c238-4ff1-9e6d-622a341fb5ed:x2",
                              f"runs_failed:{r1}"]
    assert rep["runs"][1]["reasons"] == [f"ctf_trace_name_mismatch:ros2_trace_{REAL_R0}"]


@needs_lf_bench
def test_a_run_without_a_trace_fails_closed_without_the_bypass(tmp_path, corpus):
    d = _run_dir(tmp_path, corpus, "r0", _simulate(corpus, seed=0))
    one = replace(corpus, min_repetitions=1)
    assert fixed_input.check_set([d], one, protocol_check=_ok)["runs"][0]["reasons"] == [
        "ctf_identity_missing:no_tracing_dir"]
    row = check_set([d], one, protocol_check=_ok)["runs"][0]
    assert row["passed"] and row["trace_identity"] == {"source": "jsonl_fixture"}


@needs_lf_bench
def test_the_bypass_never_covers_a_run_that_has_a_tracing_dir(ctf_runs, corpus):
    m = next((ctf_runs[0] / "tracing").rglob("metadata"))
    m.unlink()
    assert check_set(ctf_runs[:1], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0][
        "reasons"] == ["ctf_identity_missing:no_metadata"]
    (m.parent.parent / "32-bit").mkdir()
    for p in (m, m.parent.parent / "32-bit" / "metadata"):
        p.write_bytes(_packetize(_tsdl(_uuid(1), "ros2_trace_r0"), _uuid(1)))
    assert check_set(ctf_runs[:1], replace(corpus, min_repetitions=1), protocol_check=_ok)["runs"][0][
        "reasons"] == ["ctf_identity_ambiguous:2_metadata_files"]


U = _uuid(7)


@needs_lf_bench
@pytest.mark.parametrize("metadata, reason", [
    (b"", "not_ctf_metadata"),
    (b"trace { uuid = \"x\"; };", "not_ctf_metadata"),
    (_packetize(_tsdl(U, "ros2_trace_r0"), U)[:-40], "bad_packet_size@"),
    (_packetize(_tsdl(U, "ros2_trace_r0"), U) + b"\x57\x1d", "truncated_packet_header@"),
    (_packetize(_tsdl(U, "ros2_trace_r0"), U)[:128] + b"\0" * 4 + _packetize(_tsdl(U, "x"), U)[132:256],
     "bad_packet_magic@128"),
    (_packetize(_tsdl(U, "ros2_trace_r0"), U)[:128] + _packetize(_tsdl(U, "ros2_trace_r0"), _uuid(8))[128:],
     "packet_uuids:2"),
    (_packetize(_tsdl(U, "ros2_trace_r0"), _uuid(8)), "packet_uuid_mismatch"),
    (_packetize(_tsdl(U, "ros2_trace_r0") + 'trace {\n\tuuid = "x";\n};\n', U), "trace_blocks:2"),
    (_packetize(_tsdl(U, "ros2_trace_r0", trace_extra=f'\tuuid = "{_uuid(9)}";\n'), U), "trace.uuid:2"),
    (_packetize(_tsdl(U, "ros2_trace_r0").replace("trace_name", "session_name"), U), "env.trace_name:0"),
    (_packetize(_tsdl("not-a-uuid", "ros2_trace_r0"), U), "trace.uuid:malformed"),
    (_packetize(_tsdl(U, "ros2_trace_r0")[:-3], U), "unterminated_statement"),
    (_packetize(_tsdl(U, "ros2_trace_r0") + "/* open", U), "unterminated_comment"),
    (_packetize(_tsdl(U, "ros2_trace_r0"), U)[:60] + b"\xff" + _packetize(_tsdl(U, "ros2_trace_r0"), U)[61:],
     "not_utf8"),
])
def test_malformed_or_ambiguous_ctf_identity_fails_closed(ctf_runs, corpus, metadata, reason):
    next((ctf_runs[0] / "tracing").rglob("metadata")).write_bytes(metadata)
    reasons = fixed_input.check_set(ctf_runs[:1], replace(corpus, min_repetitions=1),
                                    protocol_check=_ok)["runs"][0]["reasons"]
    assert len(reasons) == 1 and reasons[0].startswith("ctf_identity_malformed:" + reason), reasons


def test_plain_text_metadata_and_big_endian_packets_are_read():
    text = _tsdl(U, "ros2_trace_r0")
    assert ctf_metadata_text(text.encode()) == (text, None)
    big = struct.Struct(">I16sIIIBBBBB")
    data = text.encode()
    packet = big.pack(0x75D11D57, bytes.fromhex(U.replace("-", "")), 0, (big.size + len(data)) * 8,
                      (big.size + len(data)) * 8, 0, 0, 0, 1, 8) + data
    assert ctf_metadata_text(packet) == (text, bytes.fromhex(U.replace("-", "")))
    with pytest.raises(CtfIdentityError, match="trace_blocks:0"):
        ctf_identity_from_text('env { trace_name = "x"; };')


# --- trace identity: event-stream packets carry the run's own trace UUID -----------------

STREAM_DIR = Path("tracing/ust/uid/1000/64-bit")


def _stream_reasons(run: Path, corpus) -> list[str]:
    return fixed_input.check_set([run], replace(corpus, min_repetitions=1),
                                 protocol_check=_ok)["runs"][0]["reasons"]


@needs_real_ctf
def test_the_real_stream_packet_carries_the_real_metadata_uuid():
    text, header_uuid = ctf_metadata_text(REAL_CTF.read_bytes())
    order = fixed_input.ctf_stream_byte_order(text)
    assert order == "<"
    assert fixed_input.ctf_stream_packets(REAL_STREAM, header_uuid, order) == 1
    with pytest.raises(fixed_input.CtfStreamError, match="^uuid_mismatch@0$"):
        fixed_input.ctf_stream_packets(REAL_STREAM, bytes.fromhex(_uuid(1).replace("-", "")), order)


@needs_lf_bench
def test_runs_with_their_own_streams_report_what_was_walked(ctf_runs, corpus):
    rep = fixed_input.check_set(ctf_runs, corpus, protocol_check=_ok)
    assert rep["verdict"] == "pass", rep["reasons"]
    ident = rep["runs"][0]["trace_identity"]
    assert (ident["event_streams"], ident["stream_packets"]) == (2, 4)
    assert "every packet header of every event-stream file" in rep["trace_identity"]["scope"]


@needs_lf_bench
def test_streams_copied_under_each_runs_own_metadata_fail_on_the_stream_uuid(ctf_runs, corpus):
    # The audit counterexample: r1..r4 keep their own record, metadata.yaml and CTF
    # metadata (own UUID, own trace_name); only the event-stream files are r0's.
    # Every check before this one passed this set.
    for d in ctf_runs[1:]:
        for name in ("rcllf_0", "ros2_0"):
            shutil.copyfile(ctf_runs[0] / STREAM_DIR / name, d / STREAM_DIR / name)
    assert len({(d / STREAM_DIR / "metadata").read_bytes() for d in ctf_runs}) == 5
    assert len({(d / fixed_input.RUN_RECORD).read_bytes() for d in ctf_runs}) == 5
    rep = fixed_input.check_set(ctf_runs, corpus, protocol_check=_ok)
    assert rep["verdict"] == "fail"
    assert rep["reasons"] == ["runs_failed:r1,r2,r3,r4"]
    assert rep["trace_identity"]["distinct_ctf_uuids"] == 5
    assert [r["reasons"] for r in rep["runs"]] == [[]] + [[
        f"ctf_stream_uuid_mismatch:{STREAM_DIR}/rcllf_0@0",
        f"ctf_stream_uuid_mismatch:{STREAM_DIR}/ros2_0@0"]] * 4


@needs_lf_bench
def test_a_foreign_packet_after_the_first_is_found(ctf_runs, corpus):
    (ctf_runs[0] / STREAM_DIR / "rcllf_0").write_bytes(_stream(_uuid(1), _uuid(1), _uuid(2)))
    assert _stream_reasons(ctf_runs[0], corpus) == [f"ctf_stream_uuid_mismatch:{STREAM_DIR}/rcllf_0@512"]


@needs_lf_bench
def test_big_endian_streams_are_read_with_big_endian_metadata(ctf_runs, corpus):
    u = _uuid(1)
    (ctf_runs[0] / STREAM_DIR / "metadata").write_bytes(
        _packetize(_tsdl(u, "ros2_trace_r0", byte_order="be"), u))
    for name in ("rcllf_0", "ros2_0"):
        (ctf_runs[0] / STREAM_DIR / name).write_bytes(_stream(u, u, order=">"))
    assert _stream_reasons(ctf_runs[0], corpus) == []
    (ctf_runs[0] / STREAM_DIR / "ros2_0").write_bytes(_stream(u))
    assert _stream_reasons(ctf_runs[0], corpus) == [f"ctf_stream_malformed:{STREAM_DIR}/ros2_0:bad_packet_magic@0"]


U1 = _uuid(1)


@needs_lf_bench
@pytest.mark.parametrize("data, reason", [
    (b"", "empty"),
    (_stream(U1)[:60], "truncated_packet_header@0"),
    (_stream(U1) + _stream(U1)[:63], "truncated_packet_header@256"),
    (_stream(U1) + b"\0" * 256, "bad_packet_magic@256"),
    (_stream(U1, magic=0x75D11D57), "bad_packet_magic@0"),
    (_stream(U1)[:-1], "bad_packet_size@0"),
    (_stream(U1, U1)[:256 + 100], "bad_packet_size@256"),
    (_stream(U1, content=257), "bad_packet_size@0"),
    (_stream(U1, content=83), "bad_packet_size@0"),
    (_stream(U1, packet=80, content=80), "bad_packet_size@0"),
])
def test_a_malformed_stream_fails_closed(ctf_runs, corpus, data, reason):
    (ctf_runs[0] / STREAM_DIR / "rcllf_0").write_bytes(data)
    assert _stream_reasons(ctf_runs[0], corpus) == [f"ctf_stream_malformed:{STREAM_DIR}/rcllf_0:{reason}"]


@needs_lf_bench
def test_a_packet_size_that_is_not_whole_bytes_fails(ctf_runs, corpus):
    data = bytearray(_stream(U1))
    struct.pack_into("<Q", data, 56, 256 * 8 + 1)
    (ctf_runs[0] / STREAM_DIR / "rcllf_0").write_bytes(bytes(data))
    assert _stream_reasons(ctf_runs[0], corpus) == [f"ctf_stream_malformed:{STREAM_DIR}/rcllf_0:bad_packet_size@0"]


@needs_lf_bench
def test_no_event_stream_or_a_symlinked_one_fails(ctf_runs, corpus):
    d = ctf_runs[0] / STREAM_DIR
    (d / "ros2_0").unlink()
    (d / "rcllf_0").unlink()
    assert _stream_reasons(ctf_runs[0], corpus) == ["ctf_stream_missing:no_event_streams"]
    (d / "rcllf_0").symlink_to(ctf_runs[1] / STREAM_DIR / "rcllf_0")
    assert _stream_reasons(ctf_runs[0], corpus) == [
        f"ctf_stream_malformed:{STREAM_DIR}/rcllf_0:not_a_regular_file"]


@needs_lf_bench
def test_an_unreadable_stream_fails(ctf_runs, corpus, monkeypatch):
    real_open = open

    def deny(path, *a, **kw):
        if Path(path).name == "rcllf_0":
            raise PermissionError(path)
        return real_open(path, *a, **kw)
    monkeypatch.setattr("builtins.open", deny)
    assert _stream_reasons(ctf_runs[0], corpus) == [
        f"ctf_stream_malformed:{STREAM_DIR}/rcllf_0:PermissionError"]


@needs_lf_bench
@pytest.mark.parametrize("decls, reason", [
    (LTTNG_DECLS.replace("unsigned long;", "unsigned long;\ntypealias integer { size = 32; align = 8; "
                         "signed = false; } := unsigned long;"), "typealias:unsignedlong"),
    (LTTNG_DECLS.replace("size = 64; align = 8; signed = false; } := uint64_t",
                         "size = 64; align = 64; signed = false; } := uint64_t"), "typealias:uint64_t"),
    (LTTNG_DECLS.replace("\tuint64_t content_size;\n\tuint64_t packet_size;\n",
                         "\tuint64_t packet_size;\n\tuint64_t content_size;\n"), "packet_context"),
    (LTTNG_DECLS.replace(":= struct packet_context;", ":= struct { uint64_t packet_size; };"),
     "stream_packet_context"),
    (LTTNG_DECLS.split("stream {")[0], "stream_blocks:0"),
])
def test_an_unpinned_stream_layout_fails_closed(ctf_runs, corpus, decls, reason):
    (ctf_runs[0] / STREAM_DIR / "metadata").write_bytes(
        _packetize(_tsdl(U1, "ros2_trace_r0", decls=decls), U1))
    assert _stream_reasons(ctf_runs[0], corpus) == [f"ctf_stream_layout_unsupported:{reason}"]


def test_a_trace_packet_header_without_the_stream_fields_fails_closed():
    text = _tsdl(U1, "x").replace("\t\tuint32_t stream_id;\n", "")
    with pytest.raises(CtfIdentityError, match="^packet_header$"):
        fixed_input.ctf_stream_byte_order(text)
