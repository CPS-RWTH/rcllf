# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""lf_eval.sync_oracle against recorded O1 streams and converted runs (S4).

Every fixture is a real output: ``*_o1_*.jsonl``, ``b8_o1ref_*.jsonl`` and ``f1_mut_*.jsonl`` come
from ``sync_oracle`` (header, and ref with one mutant flag); ``*_run.log`` are
converted runs (evidence/impl-90/s5_*) filtered to instance exact_q1 and the
player (RCLLF_SYNC, drain, dispatch and corr lines). The expected reports are
the hand derivations in the corpus files.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from lf_eval.sync_oracle import __main__ as cli
from lf_eval.sync_oracle.compare import compare, compare_config, compare_tuples, parse_jsonl
from lf_eval.sync_oracle.corpus import arrival_order, load_corpus
from lf_eval.sync_oracle.order import parse_run

FIX = Path(__file__).parent / "fixtures"
CORPORA = Path(__file__).resolve().parents[4] / "packages" / "src" / "sync_oracle_showcase" / "corpora"


def _o1(name: str) -> list[dict]:
    return parse_jsonl((FIX / name).read_text(encoding="utf-8"))


# F1's hand derivation (corpora/f1.yaml): the first event each mutant changes.
MUTANT_SEQ = {"first_wins": 1, "evict_oldest_arrival": 0, "no_clear_old": 3, "q_plus1": 0}


@pytest.mark.parametrize("mutant,seq", sorted(MUTANT_SEQ.items()))
def test_each_mutant_differs_at_the_hand_derived_seq(mutant, seq):
    report = compare_tuples(_o1("f1_o1_exact_q2.jsonl"), _o1(f"f1_mut_{mutant}.jsonl"))
    assert report is not None and report.startswith(f"tuple_mismatch at seq {seq}:"), report


def test_header_stream_is_the_expect_block():
    from lf_eval.sync_oracle.compare import short
    events = [short(e) for e in _o1("f1_o1_exact_q2.jsonl") if e["ev"] in ("signal", "drop")]
    assert events == load_corpus(CORPORA / "f1.yaml")["expect"]


def _config_case(**change):
    corpus = load_corpus(CORPORA / "f1.yaml")
    o1 = _o1("f1_o1_exact_q2.jsonl")
    node = [dict(r, node="exact_q2") for r in o1]
    node[0] = dict(node[0], impl="header", **change)
    player = [{"ev": "corpus", "corpus": corpus["digest"]}]
    return compare_config(o1, node, corpus, player)


def test_config_records_agree_on_the_pinned_versions():
    assert _config_case() is None


def test_unpinned_library_version_is_reported():
    assert _config_case(message_filters="4.12.0-1noble") == \
        "version_unpinned:message_filters 4.12.0-1noble (converted)"
    assert _config_case(rclcpp="") == "version_unpinned:rclcpp missing (converted)"


def test_config_field_mismatch_is_reported():
    assert _config_case(queue=3) == "config_mismatch:queue O1 2, converted 3"


def test_arrival_order_is_tick_then_input_then_publish_order():
    rows = [{"row": i, "tick": t, "input": n, "stamp_ns": 1, "payload": 0}
            for i, (t, n) in enumerate([(0, 1), (0, 0), (0, 1), (1, 0), (0, 0)])]
    assert [r["row"] for r in arrival_order(rows)] == [1, 4, 0, 2, 3]


def test_cli_prints_one_line_per_instance(tmp_path, capsys):
    o1 = tmp_path / "o1.jsonl"
    o1.write_text((FIX / "f1_o1_exact_q2.jsonl").read_text(encoding="utf-8"))
    log = tmp_path / "run.log"
    log.write_text("", encoding="utf-8")
    rc = cli.main(["--corpus", str(CORPORA / "f1.yaml"), "--log", str(log),
                   "--o1", f"exact_q2={o1}"])
    assert rc == 1
    assert capsys.readouterr().out == \
        "exact_q2: config_mismatch:missing (converted has no config record)\n"



def _run_report(run_name: str, corpus: str) -> str | None:
    """The comparator on a recorded converted run (instance exact_q1)."""
    run = parse_run((FIX / f"{run_name}_run.log").read_text(encoding="utf-8"))
    return compare(load_corpus(CORPORA / f"{corpus}.yaml"),
                   _o1(f"{corpus}_o1_exact_q1.jsonl"), run, "exact_q1")


@pytest.mark.parametrize("run_name,corpus", [("b8", "b8"), ("f1", "f1")])
def test_converted_run_is_compatible(run_name, corpus):
    assert _run_report(run_name, corpus) is None


def test_reversed_drains_are_an_arrival_order_mismatch():
    """Output verification 3: B8 generated with RCLLF_TEST_SYNC_REVERSE_DRAINS=1
    drains input 1 first, so the policy sees row 2 (input 1) before row 0, and
    its first event is B8's drop of input 1 stamp 2 instead of input 0 stamp 1."""
    assert _run_report("b8_rev", "b8") == (
        "arrival_order_mismatch at dispatch 0: expected input 0 row 0 at +0 ns "
        "microstep 0, converted input 1 row 2 at +0 ns microstep 0")
    rev = parse_run((FIX / "b8_rev_run.log").read_text(encoding="utf-8")).sync["exact_q1"]
    assert compare_tuples(_o1("b8_o1_exact_q1.jsonl"), rev) == \
        "tuple_mismatch at seq 0: O1 drop 1 -, converted drop - 2"


def test_dropped_sample_is_out_of_domain():
    """Output verification 3: RCLLF_TEST_CORR_DROP_SAMPLE=0:2 on P1 drops the
    second record of edge 0 (player -> exact_q1 on /sync/in0), which is row 2."""
    assert _run_report("p1_lost", "p1") == "out_of_domain: LOST on edge 0 ordinal 2 (row 2)"


def test_burst_above_the_bound_is_out_of_domain():
    """Output verification 5: N1 publishes 5 rows of input 0 in one tick, the
    edge bound is 4."""
    assert _run_report("n1", "n1") == \
        "out_of_domain: burst_exceeded edge 0: 5 records in one reaction > B = 4"


# The oracle must be the header, and the domain checks
# read only the two inputs of the instance, failing closed without their edges.

def _b8_log() -> str:
    return (FIX / "b8_run.log").read_text(encoding="utf-8")


def _b8_report(log: str, o1: str = "b8_o1_exact_q1.jsonl") -> str | None:
    return compare(load_corpus(CORPORA / "b8.yaml"), _o1(o1), parse_run(log), "exact_q1")


def test_reference_model_is_not_accepted_as_oracle():
    """``sync_oracle --impl ref`` restates ExactTime and gives B8's events
    exactly, but it is not the pinned library, so it cannot stand in for O1."""
    assert _b8_report(_b8_log(), "b8_o1ref_exact_q1.jsonl") == \
        "config_mismatch:impl O1 ran ref, oracle O1 is the header"


def test_run_without_admitted_input_edges_is_not_compatible():
    """The B8 run with its ``admitted edge`` lines removed: nothing establishes
    that the inputs were admitted edges, so the run is out of the domain."""
    log = "\n".join(line for line in _b8_log().splitlines()
                    if "rcllf corr: admitted edge" not in line)
    assert _b8_report(log) == \
        "out_of_domain: input 0 of exact_q1 has no admitted edge record"


def test_unrelated_legacy_edge_does_not_take_the_instance_out_of_domain():
    """A legacy edge elsewhere in the launch (count 1, into another node)."""
    log = _b8_log().replace("0 legacy edge(s)", "1 legacy edge(s)") + (
        "\n[Corpus-1] [INFO] [1.0] [ros2_lf_binding.corr]: rcllf corr: legacy edge "
        "/other CorpusPlayer -> Other__other: transient_local\n")
    assert _b8_report(log) is None


def test_legacy_edge_into_an_input_topic_is_out_of_domain():
    log = _b8_log().replace("0 legacy edge(s)", "1 legacy edge(s)") + (
        "\n[Corpus-1] [INFO] [1.0] [ros2_lf_binding.corr]: rcllf corr: legacy edge "
        "/sync/in0 Late -> ExactSyncNode__exact_q1: transient_local\n")
    assert _b8_report(log) == (
        "out_of_domain: legacy edge Late -> ExactSyncNode__exact_q1 on /sync/in0 "
        "(transient_local)")


def test_drain_of_another_subscription_is_not_a_sync_input():
    """A third subscription of the node drains at the same tag; it is not one
    of the synchronizer's inputs and is not part of its arrival order."""
    first = next(line for line in _b8_log().splitlines()
                 if '"callback_start"' in line and '"ExactSyncNode__exact_q1"' in line)
    other = (first.replace("sync_callback_via_sub_a", "other_callback")
             .replace('"rid":1,', '"rid":9,')
             .replace('"pub_gid":"010fa6a56e09b6300000000000003e03"',
                      '"pub_gid":"030000000000000000000000000000ff"'))
    assert other != first.replace("sync_callback_via_sub_a", "other_callback")
    lines = _b8_log().splitlines()
    log = "\n".join(lines[:lines.index(first)] + [other] + lines[lines.index(first):])
    assert _b8_report(log) is None


# An input is its topic's admitted
# edges into the node, not every edge that shares a writer with its drains.

def _edge_lines(edge: int, topic: str, outcome: str, gid: str = "",
                pub: str = "CorpusPlayer") -> list[str]:
    """An admitted edge from *pub* into exact_q1 on *topic* and one dispatch
    record on it, cut from B8's edge 0 lines: the player's /sync/in0 writer,
    or *gid*."""
    lines = _b8_log().splitlines()
    admitted = next(line for line in lines if "rcllf corr: admitted edge 0 " in line)
    dispatch = next(line for line in lines if '"kind":"dispatch","edge":0,' in line
                    and '"ordinal":1,' in line)
    return [admitted.replace("admitted edge 0 ", f"admitted edge {edge} ")
            .replace("'/sync/in0'", f"'{topic}'")
            .replace(" CorpusPlayer -> ", f" {pub} -> "),
            dispatch.replace('"edge":0,', f'"edge":{edge},')
            .replace('"outcome":"MATCHED"', f'"outcome":"{outcome}"')
            .replace('"gid":"010fa6a56e09b6300000000000003e03"', f'"gid":"{gid}"' if gid
                     else '"gid":"010fa6a56e09b6300000000000003e03"')]


def test_lost_samples_of_a_second_input_writer_are_out_of_domain():
    """A second writer node's admitted edge on /sync/in0 whose every sample is
    LOST has dispatch records and no drain; it is still one of input 0's edges.
    (A second edge from the player's own node would be a second subscription:
    the generator refuses a repeat edge of one writer node into one
    subscription as duplicate_subscription, lf_generator.py.)"""
    lost = _edge_lines(9, "/sync/in0", "LOST", gid="020000000000000000000000000000ff",
                       pub="Late")
    log = _b8_log() + "\n" + "\n".join(lost) + "\n"
    assert _b8_report(log) == "out_of_domain: LOST on edge 9 ordinal 1 (row 0)"


def test_edge_of_another_subscription_does_not_stand_in_for_an_input():
    """Input 0's admitted line is missing; an edge from the same writer into
    the node on another topic must not count as its record."""
    log = "\n".join(line for line in _b8_log().splitlines()
                    if "rcllf corr: admitted edge 0 " not in line)
    log += "\n" + "\n".join(_edge_lines(9, "/other", "MATCHED")) + "\n"
    assert _b8_report(log) == \
        "out_of_domain: input 0 of exact_q1 has no admitted edge record"


def test_lost_sample_on_another_subscription_does_not_reject_the_instance():
    log = _b8_log() + "\n" + "\n".join(_edge_lines(9, "/other", "LOST")) + "\n"
    assert _b8_report(log) is None


# A second subscription of the node on
# an input topic must not stand in for the synchronizer's own subscription.

def _twin_log(keep_input_edge: bool) -> str:
    """B8 plus a twin subscription of exact_q1 on /sync/in0 (rid 9,
    ``twin_callback``) fed by the same writer: its admitted edge 9 with a
    MATCHED dispatch, and one drain per input 0 drain, same writer and seq.
    Without *keep_input_edge* the synchronizer's own edge 0 line is gone."""
    lines = _b8_log().splitlines()
    if not keep_input_edge:
        lines = [line for line in lines if "rcllf corr: admitted edge 0 " not in line]
    twin = [line.replace('"rid":1,', '"rid":9,')
            .replace("sync_callback_via_sub_a", "twin_callback")
            for line in lines
            if '"callback_start"' in line and "sync_callback_via_sub_a" in line
            and '"ExactSyncNode__exact_q1"' in line]
    assert twin and all('"rid":9,' in t for t in twin)
    return "\n".join(lines + _edge_lines(9, "/sync/in0", "MATCHED") + twin) + "\n"


def test_twin_subscription_does_not_stand_in_for_the_input_edge():
    """Falsification (0411da8c reported ``compatible``): the twin's edge and
    MATCHED dispatches carried input 0's writer while the synchronizer's own
    subscription had no admitted edge record."""
    assert _b8_report(_twin_log(keep_input_edge=False)) == (
        "out_of_domain: second subscription on /sync/in0: twin_callback (rid 9) of "
        "exact_q1 drained input 0's writer 010fa6a56e09b6300000000000003e03")


def test_twin_subscription_edges_from_one_writer_are_out_of_domain():
    """Both subscriptions admitted: two edges from the player's node into the
    node on /sync/in0 are two subscriptions, found before any drain is read."""
    log = "\n".join(line for line in _twin_log(keep_input_edge=True).splitlines()
                    if "twin_callback" not in line)
    assert _b8_report(log) == (
        "out_of_domain: second subscription on /sync/in0: 2 admitted edges [0, 9] "
        "from CorpusPlayer into exact_q1")


def test_input_edge_in_another_line_form_still_counts():
    """The synchronizer's own edge 0 printed in the ingress form (the parser
    read only the local form before): the twin's local edge 9 is then no
    longer the input's only record."""
    log = _twin_log(keep_input_edge=False)
    admitted = next(line for line in _b8_log().splitlines()
                    if "rcllf corr: admitted edge 0 " in line)
    log += admitted.replace("admitted edge 0 ", "admitted ingress edge 0 ") + "\n"
    log = "\n".join(line for line in log.splitlines() if "twin_callback" not in line)
    assert _b8_report(log) == (
        "out_of_domain: ingress edge 0 into exact_q1 on input 0's topic /sync/in0")


def test_deferred_reader_line_is_an_admitted_edge_record():
    line = ("[X-1] [INFO] [1.0] [ros2_lf_binding.corr]: rcllf corr: admitted edge 3 "
            "CorpusPlayer -> ExactSyncNode__exact_q1 on '/sync/in0' with a deferred reader: "
            "the lifecycle node makes or resets the subscription in a hook")
    assert parse_run(line).edges == {3: {
        "pub": "CorpusPlayer", "sub": "ExactSyncNode__exact_q1", "topic": "/sync/in0",
        "kind": "deferred", "delay_ns": None}}


def test_input_without_drains_is_not_identified():
    """The synchronizer's edge 0 line gone, the twin's edge 9 the
    only record on /sync/in0, and no drain of input 0 at all (neither the
    synchronizer's nor the twin's): no rid ties edge 9 to the synchronizer."""
    log = "\n".join(line for line in _twin_log(keep_input_edge=False).splitlines()
                    if "twin_callback" not in line and "sync_callback_via_sub_a" not in line)
    assert _b8_report(log) == (
        "out_of_domain: input 0 of exact_q1 drained none of its 2 corpus rows, "
        "its subscription is not identified")


def test_input_without_corpus_rows_must_have_no_dispatch():
    """A corpus that gives input 1 no row: dispatch records on its edge are a
    writer the corpus does not have; without them the input passes."""
    from lf_eval.sync_oracle.compare import check_domain
    corpus = load_corpus(CORPORA / "b8.yaml")
    corpus = {**corpus, "rows": [r for r in corpus["rows"] if r["input"] == 0]}
    run = parse_run(_b8_log())
    run.drains = [d for d in run.drains if not d["callback"].endswith("_via_sub_b")]
    player = run.sync["corpus_player"]
    assert check_domain(run, "exact_q1", corpus, player) == (
        "out_of_domain: input 1 of exact_q1 has no corpus row but dispatch records on edge(s) [5]")
    run.dispatch = [r for r in run.dispatch if r["edge"] != 5]
    assert check_domain(run, "exact_q1", corpus, player) is None
