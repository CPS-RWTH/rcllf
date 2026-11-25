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
"""Identity-aware semantic tracing on real generated systems (T10).

Every case runs a generated system inside a private LTTng session (channels
rcllf and ros2), reads the recorded CTF with bt2 and checks the claim statuses
the analyzer derives. Set RCLLF_SEMANTIC_GOLDEN_DIR to also copy each run's
JSONL dump and manifests there (scripts/record_semantic_golden.sh).
"""

from __future__ import annotations

import json
import os
import re
import shutil
import signal
from pathlib import Path

import pytest

from . import runtime_support as rts
from .semantic_session import SessionSpec, daemon, traced_launch

try:
    import bt2  # noqa: F401  the CTF reader needs it
except ImportError:  # pragma: no cover - dev image has it
    bt2 = None
try:
    from lf_eval.semantic.reader import dump_jsonl, load_jsonl, read_ctf
    from lf_eval.semantic.reconcile import analyze_run
except ImportError:  # pragma: no cover - lf_eval is installed in the dev image
    dump_jsonl = load_jsonl = read_ctf = analyze_run = None

# Opt-in: these runs start their own lttng-sessiond, which CI does not do yet
# (CI has no traced runs), and the pubsub/service smoke -k expressions
# would otherwise select them there. Markers, never a module-level skip: under
# pytest 7 (dev image) a module-level skip in this package drops every other
# runtime module from collection, so the whole smoke layer "passes" empty.
pytestmark = [
    pytest.mark.runtime,
    pytest.mark.xdist_group("semantic_trace"),
    pytest.mark.skipif(os.environ.get("RCLLF_RUN_SEMANTIC_TRACE") != "1",
                       reason="set RCLLF_RUN_SEMANTIC_TRACE=1 to run the traced checks"),
    pytest.mark.skipif(bt2 is None, reason="python3-bt2 not installed"),
    pytest.mark.skipif(analyze_run is None, reason="lf_eval not installed"),
    pytest.mark.skipif(shutil.which("lttng-sessiond") is None, reason="lttng-sessiond not installed"),
]



def _claims(block, kind="correspondence"):
    return {c["subject"].split(" ")[1]: c for c in block["claims"] if c["kind"] == kind}


def _run(tmp_path, name, case_key="corr", scenario="one", spec=SessionSpec(), env=None,
         kill=signal.SIGINT, until_tick=10, secs=1.0, case=None, **launch_kw):
    case = case or rts.require_case(case_key)
    env = dict(env or {})
    if case_key.startswith("corr"):
        env.setdefault("CORR_SCENARIO", scenario)
    until = (lambda log: f"corr_pub: tick={until_tick} " in log) if (
        case_key == "corr" and until_tick) else None
    with daemon() as d:
        run = traced_launch(d, case, tmp_path / name, secs=secs, spec=spec, env=env, until=until,
                            kill=kill, **launch_kw)
    events, stats = read_ctf(run.run_dir / "tracing")
    dump_jsonl(events, stats, run.run_dir / "semantic_events.jsonl")
    reloaded, restats = load_jsonl(run.run_dir / "semantic_events.jsonl")
    assert reloaded == events and restats == stats, "JSONL dump of the CTF must load identically (T6)"
    golden = os.environ.get("RCLLF_SEMANTIC_GOLDEN_DIR")
    if golden:
        dst = Path(golden) / name
        dst.mkdir(parents=True, exist_ok=True)
        for f in ("semantic_events.jsonl", "trace_manifest.json", "rcllf_trace_manifest.json"):
            if (run.run_dir / f).is_file():
                shutil.copy2(run.run_dir / f, dst / f)
    shutil.rmtree(run.run_dir / "tracing", ignore_errors=True) if golden else None
    return run, events, analyze_run(run.run_dir)


def test_many_reconstructs_every_chain_and_is_supported(tmp_path):
    run, events, block = _run(tmp_path, "corr_many", scenario="many")
    # T0 folded in: the executable-resident gate path reaches the provider,
    # seq is 1..N per category and the pre-begin publishes sit in a backlog.
    pubs = [e for e in events if e.name == "publication"]
    assert pubs, "rcllf provider not recorded on the gate path"
    assert sorted(e.seq for e in pubs) == list(range(1, len(pubs) + 1)), [e.seq for e in pubs]
    assert [e for e in events if e.name == "backlog"], "no backlog event at run_begin"
    claims = _claims(block)
    for eid in ("0", "1"):
        assert claims[eid]["overall"] == "supported", claims[eid]
    corr = block["metrics"]["semantic_correspondence"]["data"]
    assert corr["0"] == {"MATCHED": 30} and corr["1"] == {"MATCHED": 30}, corr
    chains = block["metrics"]["semantic_chain"]["data"]["chains"]
    assert len(chains) == 10, f"one timer dispatch per tick expected, got {len(chains)}"
    for ch in chains:
        # Three publications per tick at the timer's tag (t, 0), each fanned
        # out to both subscribers. Both edges are in-process, between two
        # nodes, d = 0 (corr_many_execution.yaml sets no after_delay) and
        # joined immediately, so lf_generator.corr_same_tag holds: since
        # every record due at a tag is delivered at that tag in
        # one drain firing, so all six hops sit at (t, 0), as HOP_MICRO = 0 in
        # test_runtime_correspondence.py. The generation-time
        # escape RCLLF_LOCAL_SPLIT_DRAIN=1 wakes the drain at (t, 1) and gives
        # [1] * 6; the earlier binding drained one record per microstep,
        # [1, 1, 2, 2, 3, 3].
        assert sorted(h["microstep"] for h in ch["hops"]) == [0] * 6, ch
        assert all(h["logical_delta_ns"] == 0 for h in ch["hops"]), ch
        assert all(h["outcome"] == "MATCHED" for h in ch["hops"]), ch
    assert not [c for c in block["epoch_integrity"].values() if c], block["epoch_integrity"]


def test_fanout_one_publication_two_expectations(tmp_path):
    _run_, events, block = _run(tmp_path, "corr_fanout", scenario="fanout")
    pubs = [e for e in events if e.name == "publication" and e.fields["origin"] == 0]
    exps = [e for e in events if e.name == "expectation"]
    assert len(exps) == 2 * len(pubs) == 20, (len(pubs), len(exps))
    assert all(c["overall"] == "supported" for k, c in _claims(block).items() if k in ("0", "1"))


def test_publication_event_disabled_is_uncovered(tmp_path):
    events_wo_pub = ["rcllf:run_begin", "rcllf:run_end", "rcllf:run_end_publisher", "rcllf:run_end_edge",
                     "rcllf:provider", "rcllf:backlog", "rcllf:expectation", "rcllf:terminal",
                     "rcllf:carrier", "rcllf:sample", "rcllf:dispatch_begin", "rcllf:dispatch_end",
                     "rcllf:cleanup"]
    _r, _e, block = _run(tmp_path, "corr_no_publication", spec=SessionSpec(rcllf_events=events_wo_pub))
    c = _claims(block)["0"]
    assert (c["coverage"], c["coverage_codes"]) == ("uncovered", ["provider_disabled:publication"]), c
    assert c["overall"] == "incomplete", c


def test_rmw_bypass_is_uncovered_and_exits_3(tmp_path):
    run, _e, block = _run(tmp_path, "corr_bypass", scenario="bypass")
    c = _claims(block)["0"]
    assert "path_bypass" in c["coverage_codes"], c
    assert c["reconciliation"] == "not_evaluated", c
    assert re.search(r"process has died \[pid \d+, exit code 3", run.log), run.log[-1500:]


# The W3 missing dispatch on the LTTng dispatch_end and in the trace_corr JSON
# of the same run. Expected values from the fixture source, as in
# test_runtime_correspondence: `one` publishes ordinal t at elapsed
# (t - 1) * 200 ms, so dropping ordinal 2 makes it missing at its tag
# (200 ms, 0). /corr/a is a local edge: writer and reader share the
# CorrBringup process, so it is decided when the publish returns and has no
# cutoff (docs/features/missing-messages.md "Edges inside one federate
# have no cutoff"). The dropped sample ends LOST at once, its T of 100 ms
# (corr_many_execution.yaml) only sizes the lag bound, and the cutoff a timed
# edge would get, 200 + 0 + 100 = 300 ms after the start tag, does not exist:
# dispatch_end.cutoff is the sentinel -1 and the JSON's cutoff_elapsed -1. With
# both hooks registered the deadline hook names the dispatch.
W3_DROP_ORDINAL = 2
W3_DROP_TAG_ELAPSED_NS = (W3_DROP_ORDINAL - 1) * 200_000_000
W3_T_NS = 100_000_000


def test_w3_missing_dispatch_end_carries_cutoff_and_missing_hook(tmp_path):
    from lf_eval.semantic import schema
    run, events, _block = _run(
        tmp_path, "corr_w3_missing", env={
            "RCLLF_TEST_CORR_DROP_SAMPLE": str(W3_DROP_ORDINAL), "ROS2_LF_TAKE_TIMEOUT_MS": "100",
            "CORR_DEADLINE_HOOK": "1", "CORR_MESSAGE_LOST_HOOK": "1", "RCLLF_TRACE_CALLBACKS": "1"})
    # The generator's side, independent of the runtime: the edge is local and
    # has no cutoff, its T is the declared 100 ms.
    manifest = json.loads((run.run_dir / "rcllf_trace_manifest.json").read_text())
    corr_a = [e for e in manifest["edges"] if e["topic"] == "/corr/a" and e["status"] == "admitted"]
    assert corr_a and all((e["bounds"].get("local_now"), e["bounds"].get("cutoff"), e["bounds"]["timeout_ns"])
                          == (True, "none", W3_T_NS) for e in corr_a), corr_a
    start = [e for e in events if e.name == "lf_start"]
    assert len(start) == 1, f"one lf_start expected: {start}"
    t0 = int(start[0].fields["lf_start_time_ns"])
    begins = {int(e.fields["dispatch_id"]): e for e in events if e.name == "dispatch_begin"}
    ends = [e for e in events if e.name == "dispatch_end"
            and int(begins[int(e.fields["dispatch_id"])].fields["entry_kind"])
            == schema.ENTRY_KINDS["CORR_SUB"]]
    assert ends, "no CORR_SUB dispatch_end recorded"
    missing = [e for e in ends if int(e.fields["outcome"]) != schema.OUTCOMES["MATCHED"]]
    # cutoff_elapsed, never cutoff - t0: -1 is no tag (the stale expectation
    # read it as one and failed with [(2, -<t0 + 1>, 3)]).
    got = [(int(begins[int(e.fields["dispatch_id"])].fields["ordinal"]),
            int(begins[int(e.fields["dispatch_id"])].fields["tag_time"]) - t0,
            int(e.fields["outcome"]), schema.cutoff_elapsed(e.fields["cutoff"], t0),
            int(e.fields["missing_hook"])) for e in missing]
    assert got == [(W3_DROP_ORDINAL, W3_DROP_TAG_ELAPSED_NS, schema.OUTCOMES["LOST"], None,
                    schema.MISSING_HOOKS["DEADLINE"])], got
    for e in ends:
        assert int(e.fields["cutoff"]) == schema.NO_CUTOFF, f"a local edge has a cutoff: {e}"
        if int(e.fields["outcome"]) == schema.OUTCOMES["MATCHED"]:
            assert int(e.fields["missing_hook"]) == schema.MISSING_HOOKS["NA"], e
    # The trace_corr JSON of the same run names the same cutoff and hook.
    js = [d for d in (json.loads(m) for m in re.findall(r'(\{"event":"corr".*\})', run.log))
          if d.get("kind") == "dispatch"]
    odd = [(d["ordinal"], d["elapsed"], d["outcome"], d["cutoff_elapsed"], d["missing_hook"])
           for d in js if d["outcome"] != "MATCHED"]
    assert odd == [(W3_DROP_ORDINAL, W3_DROP_TAG_ELAPSED_NS, "LOST", schema.NO_CUTOFF, "deadline")], odd
    assert js and {d["cutoff_elapsed"] for d in js} == {schema.NO_CUTOFF}, js


def test_dropped_publication_seq_is_a_gap(tmp_path):
    _r, _e, block = _run(tmp_path, "corr_drop_seq", env={"RCLLF_TEST_TRACE_DROP_SEQ": "publication:5"})
    c = _claims(block)["0"]
    assert c["integrity_codes"] == ["seq_gap:publication:5"], c
    assert c["overall"] == "incomplete", c


def test_skipped_run_end_is_no_end(tmp_path):
    _r, _e, block = _run(tmp_path, "corr_skip_run_end", env={"RCLLF_TEST_TRACE_SKIP_RUN_END": "1"})
    c = _claims(block)["0"]
    assert "no_end" in c["integrity_codes"], c
    assert c["overall"] == "incomplete", c


def test_sigkill_mid_run_is_no_end(tmp_path):
    _r, _e, block = _run(tmp_path, "corr_sigkill", kill=signal.SIGKILL, until_tick=5)
    c = _claims(block)["0"]
    assert "no_end" in c["integrity_codes"], c
    assert c["overall"] == "incomplete", c


def test_tiny_subbuffers_report_discards_and_a_gap(tmp_path):
    spec = SessionSpec(subbuf_size="4096", num_subbuf=2)
    _r, _e, block = _run(tmp_path, "corr_discard", scenario="many", spec=spec,
                         env={"RCLLF_TEST_TRACE_BURST": "20000"})
    c = _claims(block)["0"]
    assert "discarded:rcllf" in c["integrity_codes"], c
    assert any(code.startswith("seq_gap:") for code in c["integrity_codes"]), c


def test_process_without_admitted_edges_begins_and_ends(tmp_path):
    _r, events, block = _run(tmp_path, "service_no_edges", case_key="service", until_tick=0, secs=3.0)
    names = [e.name for e in events]
    assert names.count("run_begin") >= 1 and names.count("run_end") == names.count("run_begin"), names
    assert all(not codes for codes in block["epoch_integrity"].values()), block["epoch_integrity"]


def test_zero_delay_loop_is_supported(tmp_path):
    # corr_loop is a zero-delay loop without a causality loop (its /corr/pong
    # subscriber only logs), so both d = 0 edges are admitted.
    # The legacy reason codes are pinned by test_trace_manifest.py.
    _r, _e, block = _run(tmp_path, "corr_loop", case_key="corr_loop", until_tick=0, secs=3.0)
    corr = [c for c in block["claims"] if c["kind"] == "correspondence"]
    assert len(corr) == 2 and all(c["overall"] == "supported" for c in corr), block["claims"]


def test_pubsub_payload_and_counts(tmp_path):
    run, _e, block = _run(tmp_path, "pubsub", case_key="pubsub", until_tick=0, secs=4.0)
    published, heard = rts.parse_pubsub_log(run.log)
    assert published and not rts.find_pubsub_mismatches(published, heard), run.log[-1500:]
    c = [c for c in block["claims"] if c["kind"] == "correspondence"]
    assert c and all(x["overall"] == "supported" for x in c), c


def test_nested_publish_is_traced_as_nested_without_bypass(tmp_path):
    """A /corr/b echo published from the /corr/a on-new-message callback, which Fast DDS
    runs inside the tracked /corr/a publish: the gate sees a nested publish."""
    run, events, block = _run(tmp_path, "corr_nested", scenario="nested")
    nested = [e for e in events if e.name == "publication" and e.fields["origin"] == 5]
    assert len(nested) == 10, (len(nested), run.log[-1500:])
    others = [e for e in events if e.name == "publication" and e.seq in {n.seq for n in nested}
              and e.fields["origin"] != 5]
    assert not others, "a nested publish must emit exactly one publication(NESTED)"
    claims = [c for c in block["claims"] if c["kind"] == "correspondence"]
    assert not [c for c in claims if "path_bypass" in c["coverage_codes"]], claims
    assert all(c["overall"] == "supported" for c in claims), claims


def _stock_pubsub():
    share = rts._PACKAGES_ROOT / "install" / "pubsub_example" / "share" / "pubsub_example" / "launch"
    if not (share / "bringup.launch.py").is_file():
        pytest.skip("pubsub_example not installed")
    return rts.RuntimeCase(source_pkg="pubsub_example", launch_stem="bringup",
                           gen_pkg="pubsub_example", launch_file="bringup.launch.py")


def test_stock_pubsub_claims_are_unsupported(tmp_path):
    run, events, block = _run(tmp_path, "stock_pubsub", case=_stock_pubsub(), until_tick=0, secs=3.0,
                              copy_static=False, metadata={"mode": "stock"})
    shutil.copy2(rts._PACKAGES_ROOT / "lf-analysis" / "pubsub_example" / "bringup" / "system.json",
                 run.run_dir / "system.json")
    block = analyze_run(run.run_dir)
    assert not events, "a stock process loads no binding, so no rcllf events"
    claims = [c for c in block["claims"] if c["kind"] == "correspondence"]
    assert claims and all((c["overall"], c["coverage_codes"]) == ("unsupported", ["provider_absent"])
                          for c in claims), claims


def _federated_case():
    install = rts._PACKAGES_ROOT / "install"
    pkgs = sorted(p.name for p in install.glob("lf_generated_pubsub_example_bringup*_federated"))
    if not pkgs:
        pytest.skip("federated pubsub_example not built (FEDERATE=per-node docker/lf_build.sh)")
    launch = sorted((install / pkgs[0] / "share" / pkgs[0] / "launch").glob("*.launch.py"))
    return rts.RuntimeCase(source_pkg="pubsub_example", launch_stem="bringup", gen_pkg=pkgs[0],
                           launch_file=launch[0].name)


def test_federated_pubsub_every_federate_traced_without_manifest_mismatch(tmp_path):
    run, events, block = _run(tmp_path, "fed_pubsub", case=_federated_case(), until_tick=0, secs=8.0)
    begins = [e for e in events if e.name == "run_begin"]
    ends = [e for e in events if e.name == "run_end"]
    exes = sorted(str(e.fields["executable"]).rsplit("/", 1)[-1] for e in begins)
    assert len(exes) >= 2 and all(x.startswith("federate__") for x in exes), exes
    assert len(ends) == len(begins), (len(begins), len(ends))
    assert all(len(e.fields["static_manifest_hash"]) == 64 for e in begins), begins
    assert all(not codes for codes in block["epoch_integrity"].values()), block["epoch_integrity"]
    for c in block["claims"]:
        assert not any("manifest_mismatch" in x or "missing_producer" in x for x in c["integrity_codes"]), c
    # lf_start carries the RTI-negotiated start tag, the same in
    # every federate. The publisher's timer has offset 0, so the earliest
    # dispatch of the federation is at that tag, and no federate dispatches
    # before it.
    starts, firsts = {}, {}
    for pid in sorted({e.pid for e in begins}):
        mine = [e for e in events if e.pid == pid]
        ls = [int(e.fields["lf_start_time_ns"]) for e in mine if e.name == "lf_start"]
        tags = [int(e.fields["tag_time"]) for e in mine if e.name == "dispatch_begin"]
        assert len(ls) == 1 and tags, (pid, ls, len(tags))
        starts[pid], firsts[pid] = ls[0], min(tags)
        assert ls[0] <= min(tags), (pid, ls[0], min(tags))
    first = min(firsts.values())
    assert set(starts.values()) == {first}, (
        f"lf_start_time_ns per federate {starts} is not the first dispatch tag {first} "
        f"(off by {sorted(first - s for s in starts.values())} ns)")
