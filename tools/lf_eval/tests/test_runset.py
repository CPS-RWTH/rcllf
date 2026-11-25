# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Run-set inclusion check. The recorded fixture (fixtures/runset/, with its README) stays in the development repository; the tests skip where it is absent."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from lf_eval import runset

from .dev_only import requires_golden

FIX = Path(__file__).parent / "fixtures" / "runset" / "lfc_run"

#: the recorded run manifest carries the recording host's name and is not exported
pytestmark = requires_golden(FIX)


@pytest.fixture
def run(tmp_path, monkeypatch):
    d = tmp_path / "20260930T145657Z_lfc_default_r0"
    shutil.copytree(FIX, d)
    # semantic analysis needs the trace; stub it with an all-supported block
    monkeypatch.setattr(runset, "_semantic_block", lambda _d: {"claims": [
        {"kind": "correspondence", "integrity": "complete", "coverage": "covered", "overall": "supported"},
        {"kind": "correspondence", "integrity": "complete", "coverage": "unsupported", "overall": "unsupported"},
    ]})
    return d


def _manifest(d):
    p = d / "trace_manifest.json"
    return p, json.loads(p.read_text())


def test_recorded_run_is_included(run):
    assert runset.check_run(run) == []


def test_discarded_trace_events_exclude_the_run(run):
    p, m = _manifest(run)
    m["channels"]["ros2"]["discarded"] = 3
    p.write_text(json.dumps(m))
    assert runset.check_run(run) == ["trace_loss:ros2"]


def test_unknown_discard_count_excludes_the_run(run):
    p, m = _manifest(run)
    m["channels"]["rcllf"]["discarded"] = "unknown"
    p.write_text(json.dumps(m))
    assert runset.check_run(run) == ["trace_loss:rcllf"]


@pytest.mark.parametrize("manifest", [{"channels": {}}, {"channels": None}, {"schema": 1}])
def test_a_manifest_without_channels_vouches_for_nothing(run, manifest):
    p, m = _manifest(run)
    m.pop("channels")
    p.write_text(json.dumps({**m, **manifest}))
    assert runset.check_run(run) == ["trace_loss:no_channels"]


@pytest.mark.parametrize("text", ["{not json", "[1, 2]"])
def test_a_malformed_manifest_fails_closed(run, text):
    (run / "trace_manifest.json").write_text(text)
    assert runset.check_run(run) == ["trace_loss:manifest_malformed"]


@pytest.mark.parametrize("field,value", [("discarded", "lots"), ("discarded", -1), ("discarded", True),
                                         ("lost_packets", 2), ("lost_packets", "unknown"),
                                         ("lost_packets", None)])
def test_a_loss_count_that_is_not_zero_excludes_the_run(run, field, value):
    p, m = _manifest(run)
    m["channels"]["ros2"][field] = value
    p.write_text(json.dumps(m))
    assert runset.check_run(run) == ["trace_loss:ros2"]


def _set_md(run, **changes):
    import yaml

    md = yaml.safe_load((run / "metadata.yaml").read_text())
    for key, value in changes.items():
        if value is _DROP:
            md.pop(key)
        else:
            md[key] = value
    (run / "metadata.yaml").write_text(yaml.safe_dump(md))


@pytest.mark.parametrize("value", [4, "unknown"])
def test_metadata_events_discarded_counts_even_with_a_clean_manifest(run, value):
    _set_md(run, events_discarded=value)
    assert runset.check_run(run) == ["trace_loss:all"]


def test_old_run_without_manifest_is_judged_by_its_metadata(run):
    (run / "trace_manifest.json").unlink()
    assert runset.check_run(run) == []
    _set_md(run, events_discarded=_DROP)
    assert runset.check_run(run) == []
    _set_md(run, events_discarded=2)
    assert runset.check_run(run) == ["trace_loss:all"]


def test_a_failed_run_without_metadata_is_excluded(run):
    (run / "metadata.yaml").unlink()
    assert runset.check_run(run) == ["no_metadata"]


def test_run_shorter_than_the_window_is_excluded(run):
    # the recorded run lasted 16.2 s wall
    assert runset.check_run(run, window_total_s=10) == []
    assert runset.check_run(run, window_total_s=30) == ["shorter_than_window"]


def test_incomplete_lfc_semantic_claim_excludes_only_lfc(run, monkeypatch):
    monkeypatch.setattr(runset, "_semantic_block", lambda _d: {"claims": [
        {"kind": "correspondence", "integrity": "incomplete", "coverage": "covered", "overall": "incomplete"}]})
    assert runset.check_run(run) == ["semantic_incomplete"]


def test_check_runs_writes_a_table_with_every_reason(run, tmp_path):
    bad = tmp_path / "bad"
    bad.mkdir()
    rows = runset.check_runs([run, bad])
    assert [(r["run"], r["included"], r["reasons"]) for r in rows] == [
        (run.name, True, []), ("bad", False, ["no_metadata"])]


def test_cli_check_runs_prints_and_writes_the_table(run, tmp_path, capsys):
    from lf_eval.cli import main

    out = tmp_path / "run_set.json"
    assert main(["check-runs", str(run), str(tmp_path / "missing"), "--out", str(out),
                 "--warmup-s", "2", "--window-s", "10"]) == 0
    assert "1 included, 1 excluded" in capsys.readouterr().out
    doc = json.loads(out.read_text())
    assert doc["window"]["steady_s"] == 10.0 and [r["included"] for r in doc["runs"]] == [True, False]


def test_rslcpp_is_checked_against_its_simulated_horizon(run):
    md = run / "metadata.yaml"
    text = md.read_text().replace("mode: lfc", "mode: rslcpp")
    md.write_text(text + "simulated_horizon_s: 30\n")
    # wall duration 16.2 s, simulated 30 s: a 25 s protocol window is covered
    assert runset.check_run(run, window_total_s=25) == []
    assert runset.check_run(run, window_total_s=31) == ["shorter_than_window"]


# ── metadata integrity (opt-in) ────────────────────────────────

SHA = "2009fbe4d5434b7dcfb9bb1a2cfe14cad1b0629f"
FROZEN = runset.Expectations(git_sha=SHA, duration_s=30.0, scenario="default",
                             require_clean_tree=True, require_uniform_identity=True)


def _frozen_run(d, **overrides):
    """Give the fixture the candidate campaign's frozen metadata and workload files."""
    import yaml

    md = yaml.safe_load((d / "metadata.yaml").read_text())
    md.update(git_sha=SHA, git_dirty=False, duration=30.0)
    for key, value in overrides.items():
        if value is _DROP:
            md.pop(key)
        else:
            md[key] = value
    (d / "metadata.yaml").write_text(yaml.safe_dump(md))
    for name in runset.WORKLOAD_FILES:
        if not (d / name).exists():
            (d / name).write_text(f"{name} workload\n")
    return d


_DROP = object()


def _copy(run, tmp_path, name, **overrides):
    d = tmp_path / name
    shutil.copytree(run, d)
    return _frozen_run(d, **overrides)


def test_metadata_checks_are_off_by_default(run):
    # Semantics: the 000 SHA, dirty tree and 5 s duration of CANDIDATE_REPORT.md stay included
    _frozen_run(run, git_sha="0" * 40, git_dirty=True, duration=5.0)
    assert runset.check_run(run) == []
    assert runset.check_run(run, expect=runset.Expectations()) == []


def test_valid_frozen_run_is_included(run):
    _frozen_run(run)
    assert runset.check_run(run, expect=FROZEN) == []
    assert runset.check_runs([run], expect=FROZEN)[0]["included"] is True


@pytest.mark.parametrize("field,value,reason", [
    ("git_sha", "0" * 40, "metadata_mismatch:git_sha"),
    ("git_sha", SHA[:7], "metadata_invalid:git_sha"),
    ("git_sha", SHA.upper(), "metadata_invalid:git_sha"),
    ("git_sha", "unavailable", "metadata_invalid:git_sha"),
    ("git_sha", _DROP, "metadata_missing:git_sha"),
    ("git_sha", None, "metadata_missing:git_sha"),
    ("git_dirty", True, "metadata_mismatch:git_dirty"),
    ("git_dirty", "false", "metadata_invalid:git_dirty"),
    ("git_dirty", "unavailable", "metadata_invalid:git_dirty"),
    ("git_dirty", 0, "metadata_invalid:git_dirty"),
    ("git_dirty", _DROP, "metadata_missing:git_dirty"),
    ("duration", 5.0, "metadata_mismatch:duration"),
    ("duration", 30.000001, "metadata_mismatch:duration"),
    ("duration", "30.0", "metadata_invalid:duration"),
    ("duration", True, "metadata_invalid:duration"),
    ("duration", float("nan"), "metadata_invalid:duration"),
    ("duration", _DROP, "metadata_missing:duration"),
    ("scenario", "stress", "metadata_mismatch:scenario"),
    ("scenario", 7, "metadata_invalid:scenario"),
    ("scenario", _DROP, "metadata_missing:scenario"),
])
def test_each_mutated_field_excludes_the_run(run, field, value, reason):
    _frozen_run(run, **{field: value})
    assert runset.check_run(run, expect=FROZEN) == [reason]
    row, = runset.check_runs([run], expect=FROZEN)
    assert row["included"] is False and row["reasons"] == [reason]


def test_combined_mutation_reports_every_reason(run):
    # the exact tampering of the candidate's negtest/neg_mutated_metadata copy
    _frozen_run(run, git_sha="0" * 40, git_dirty=True, duration=5.0)
    assert runset.check_run(run, expect=FROZEN) == [
        "metadata_mismatch:git_sha", "metadata_mismatch:git_dirty", "metadata_mismatch:duration"]


def test_integer_duration_matches(run):
    _frozen_run(run, duration=30)
    assert runset.check_run(run, expect=FROZEN) == []


@pytest.mark.parametrize("text", ["git_sha: [unclosed\n", "- a list\n- not a mapping\n", "plain string\n"])
def test_malformed_metadata_fails_closed(run, text):
    (run / "metadata.yaml").write_text(text)
    for expect in (None, FROZEN):
        assert runset.check_run(run, expect=expect) == ["metadata_malformed"]
    row, = runset.check_runs([run], expect=FROZEN)
    assert row["included"] is False and row["mode"] == "unknown"


def test_unparseable_timestamps_fail_the_window_check(run):
    md = run / "metadata.yaml"
    md.write_text(md.read_text().replace("started_at: '2026", "started_at: 'garbage"))
    assert runset.check_run(run, window_total_s=10) == ["shorter_than_window"]


def test_uniform_identity_accepts_a_matching_set(run, tmp_path):
    _frozen_run(run)
    other = _copy(run, tmp_path, "r1")
    rows = runset.check_runs([run, other], expect=FROZEN)
    assert [r["included"] for r in rows] == [True, True]


def test_mixed_source_sha_excludes_every_run_without_a_majority_vote(run, tmp_path):
    _frozen_run(run)
    a = _copy(run, tmp_path, "a")
    b = _copy(run, tmp_path, "b", git_sha="1" * 40)
    only_uniform = runset.Expectations(require_uniform_identity=True)
    rows = runset.check_runs([run, a, b], expect=only_uniform)
    # two of three agree; still all three are excluded and stay visible
    assert [(r["run"], r["reasons"]) for r in rows] == [
        (run.name, ["identity_not_uniform:git_sha"]), ("a", ["identity_not_uniform:git_sha"]),
        ("b", ["identity_not_uniform:git_sha"])]


def test_changed_workload_file_excludes_every_run(run, tmp_path):
    _frozen_run(run)
    other = _copy(run, tmp_path, "r1")
    (other / "scenario_params.yaml").write_text("imu_node: {rate_hz: 400}\n")
    rows = runset.check_runs([run, other], expect=FROZEN)
    assert [r["reasons"] for r in rows] == [["identity_not_uniform:workload"]] * 2


def test_missing_workload_file_excludes_that_run(run, tmp_path):
    _frozen_run(run)
    other = _copy(run, tmp_path, "r1")
    (other / "system.json").unlink()
    rows = runset.check_runs([run, other], expect=FROZEN)
    assert [r["reasons"] for r in rows] == [[], ["identity_missing:system.json"]]


def test_cli_integrity_flags_exclude_and_record_expectations(run, tmp_path, capsys):
    from lf_eval.cli import main

    _frozen_run(run)
    bad = _copy(run, tmp_path, "bad", git_sha="0" * 40, git_dirty=True, duration=5.0)
    out = tmp_path / "run_set_integrity.json"
    assert main(["check-runs", str(run), str(bad), "--out", str(out),
                 "--expected-git-sha", SHA.upper(), "--expected-duration-s", "30",
                 "--expected-scenario", "default", "--require-clean-tree"]) == 0
    assert "1 included, 1 excluded" in capsys.readouterr().out
    doc = json.loads(out.read_text())
    assert [r["reasons"] for r in doc["runs"]] == [[], [
        "metadata_mismatch:git_sha", "metadata_mismatch:git_dirty", "metadata_mismatch:duration"]]
    assert doc["integrity"]["git_sha"] == SHA and doc["integrity"]["require_clean_tree"] is True
    assert doc["integrity"]["observed"]["git_sha"] == sorted([SHA, "0" * 40])


def test_cli_without_integrity_flags_writes_the_58_document(run, tmp_path):
    from lf_eval.cli import main

    out = tmp_path / "run_set.json"
    main(["check-runs", str(run), "--out", str(out)])
    assert set(json.loads(out.read_text())) == {"window", "runs"}


@pytest.mark.parametrize("flag,value", [("--expected-git-sha", "2009fbe"),
                                        ("--expected-git-sha", "g" * 40),
                                        ("--expected-duration-s", "nan"),
                                        ("--expected-duration-s", "abc")])
def test_cli_rejects_malformed_expectations(run, flag, value):
    from lf_eval.cli import main

    with pytest.raises(SystemExit) as exc:
        main(["check-runs", str(run), flag, value])
    assert exc.value.code == 2
