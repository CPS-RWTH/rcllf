# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the ``lf-eval report`` Markdown comparison-report layer.

Same conventions as ``test_cli.py``: the reader is monkeypatched with
synthetic events (no CTF binaries needed), and run directories are built by
hand with a real ``metadata.yaml`` — five-variant vocabulary, not three, per
the Phase 6 mode generalisation.
"""

from __future__ import annotations

import json

import pytest

from lf_eval.cli import main
from lf_eval.report import (
    _drops_section,
    _md_table,
    _run_metadata_table,
    _variant_summary_rows,
    generate_report,
    read_run_metadata_raw,
    resolve_run_system,
)

from tests.conftest import MS, PIPELINE_SYSTEM_JSON, EventBuilder


def _write_pipeline_trace(run_dir, mode, extra_metadata=""):
    (run_dir / "tracing").mkdir(parents=True)
    (run_dir / "metadata.yaml").write_text(
        f'timestamp: "20260810_143000"\nmode: "{mode}"\nduration: 2\n'
        f'label: "synthetic"\n{extra_metadata}'
    )
    b = EventBuilder()
    for node in ("NodeA", "NodeB", "NodeC"):
        b.add_node(node)
    b.add_timer("NodeA", 100.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")
    base = 1000 * MS
    for i in range(20):
        t0 = base + i * 100 * MS
        b.timer_fire("NodeA", t0, MS)
        b.publish("NodeA", "/a_out", t0 + MS)
        b.take("NodeB", "/a_out", t0 + 3 * MS)
        b.sub_fire("NodeB", "/a_out", t0 + 3 * MS, MS)
        b.publish("NodeB", "/b_out", t0 + 4 * MS)
        b.take("NodeC", "/b_out", t0 + 6 * MS)
        b.sub_fire("NodeC", "/b_out", t0 + 6 * MS, MS)
    return b.build()


@pytest.fixture
def two_run_dirs(tmp_path, monkeypatch):
    """Two run directories (stock-multi, lfc) sharing one synthetic trace."""
    run1 = tmp_path / "20260810T143000Z_stock-multi_default_r0"
    run2 = tmp_path / "20260810T143000Z_lfc_default_r0"
    events1 = _write_pipeline_trace(
        run1, "stock-multi", extra_metadata="scenario: default\nrepeat_index: 0\nevents_discarded: 0\n"
    )
    events2 = _write_pipeline_trace(
        run2, "lfc", extra_metadata="scenario: default\nrepeat_index: 0\nevents_discarded: 0\n"
    )
    events_by_dir = {run1: events1, run2: events2}

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        from lf_eval.reader.reader import find_tracing_dir

        tracing_dir = find_tracing_dir(path)
        run_dir = tracing_dir.parent
        for event in events_by_dir[run_dir]:
            if event_names is None or event.name in event_names:
                yield event

    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    monkeypatch.setattr("lf_eval.reader.available_backends", lambda: ["cli"])
    return run1, run2


@pytest.fixture
def system_json(tmp_path):
    path = tmp_path / "system.json"
    path.write_text(json.dumps(PIPELINE_SYSTEM_JSON))
    return path


# ── read_run_metadata_raw ───────────────────────────────────────────────


def test_read_run_metadata_raw_direct_yaml_parse(tmp_path):
    (tmp_path / "metadata.yaml").write_text(
        'mode: "stock-single"\nscenario: "stress"\nrepeat_index: 1\n'
        "events_discarded: 3\n"
    )
    data = read_run_metadata_raw(tmp_path)
    assert data["mode"] == "stock-single"
    assert data["scenario"] == "stress"
    assert data["repeat_index"] == 1
    assert data["events_discarded"] == 3


def test_read_run_metadata_raw_missing_file_returns_empty(tmp_path):
    assert read_run_metadata_raw(tmp_path / "nope") == {}


def test_read_run_metadata_raw_non_mapping_returns_empty(tmp_path):
    (tmp_path / "metadata.yaml").write_text("- a\n- list\n")
    assert read_run_metadata_raw(tmp_path) == {}


# ── resolve_run_system (R4) ─────────────────────────────────────────────


def test_resolve_run_system_explicit_wins(tmp_path, system_json):
    other = tmp_path / "run"
    other.mkdir()
    (other / "system.json").write_text(json.dumps({"name": "other-should-not-load"}))
    system = resolve_run_system(other, explicit=system_json)
    assert system is not None
    assert system.name == "pipeline"


def test_resolve_run_system_defaults_to_run_dirs_own_copy(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "system.json").write_text(json.dumps(PIPELINE_SYSTEM_JSON))
    system = resolve_run_system(run_dir, explicit=None)
    assert system is not None
    assert system.name == "pipeline"


def test_resolve_run_system_missing_copy_returns_none(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    assert resolve_run_system(run_dir, explicit=None) is None


# ── _md_table ─────────────────────────────────────────────────────────────


def test_md_table_empty():
    assert _md_table([]) == ""


def test_md_table_renders_header_and_rows():
    table = _md_table([{"a": 1, "b": "x"}, {"a": 2, "b": "y"}])
    lines = table.splitlines()
    assert lines[0] == "| a | b |"
    assert lines[1] == "| --- | --- |"
    assert lines[2] == "| 1 | x |"
    assert lines[3] == "| 2 | y |"


def test_md_table_none_becomes_empty_cell():
    table = _md_table([{"a": None}])
    assert "|  |" in table


# ── _run_metadata_table / direct-metadata-read contract (R7) ────────────


def test_run_metadata_table_reads_scenario_and_repeat(two_run_dirs):
    run1, run2 = two_run_dirs
    table = _run_metadata_table([run1, run2])
    assert "stock-multi" in table
    assert "lfc" in table
    assert "default" in table


# ── _variant_summary_rows ────────────────────────────────────────────────


def test_variant_summary_rows_one_row_per_mode(two_run_dirs, system_json):
    run1, run2 = two_run_dirs
    outdir_payload = _run_report_payload([run1, run2], system_json)
    rows = _variant_summary_rows([run1, run2], outdir_payload)
    modes = {r["mode"] for r in rows}
    assert any("stock-multi" in m or "MultiThreaded" in m for m in modes)
    assert any("lfc" in m for m in modes)
    assert len(rows) == 2


def _run_report_payload(run_dirs, system_json, tmp_path_out=None):
    """Run `lf-eval analyze` to get a real metrics_payload-shaped dict."""
    import tempfile
    from pathlib import Path

    outdir = Path(tempfile.mkdtemp()) / "analysis"
    code = main(
        [
            "analyze",
            *[str(d) for d in run_dirs],
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--no-plots",
        ]
    )
    assert code == 0
    return json.loads((outdir / "metrics.json").read_text())


# ── _drops_section / trace-loss authority ─────────────────────


def test_drops_section_flags_nonzero_events_discarded(tmp_path):
    run_dir = tmp_path / "run_with_loss"
    run_dir.mkdir()
    (run_dir / "metadata.yaml").write_text(
        'mode: "lfc"\nevents_discarded: 42\n'
    )
    payload = {"metrics": {"drops": {"available": True, "summary": []}}, "data_quality": {}}
    section = _drops_section([run_dir], payload)
    assert "events_discarded=42" in section
    assert "authoritative" in section


def test_drops_section_clean_run_has_no_warning(tmp_path):
    run_dir = tmp_path / "run_clean"
    run_dir.mkdir()
    (run_dir / "metadata.yaml").write_text('mode: "lfc"\nevents_discarded: 0\n')
    payload = {"metrics": {"drops": {"available": True, "summary": []}}, "data_quality": {}}
    section = _drops_section([run_dir], payload)
    assert "Data-quality warnings" not in section


def test_drops_section_flags_bt2_mismatch_against_authoritative_zero(tmp_path):
    run_dir = tmp_path / "run_mismatch"
    run_dir.mkdir()
    (run_dir / "metadata.yaml").write_text('mode: "lfc"\nevents_discarded: 0\n')
    payload = {
        "metrics": {"drops": {"available": True, "summary": []}},
        "data_quality": {
            run_dir.name: {"known": True, "discarded_events": 5, "discarded_packets": 0}
        },
    }
    section = _drops_section([run_dir], payload)
    assert "mismatch" in section
    assert "5 discarded event" in section


def test_drops_section_flags_disagreement_between_authoritative_and_bt2(tmp_path):
    run_dir = tmp_path / "run_disagree"
    run_dir.mkdir()
    (run_dir / "metadata.yaml").write_text('mode: "lfc"\nevents_discarded: 10\n')
    payload = {
        "metrics": {"drops": {"available": True, "summary": []}},
        "data_quality": {
            run_dir.name: {"known": True, "discarded_events": 3, "discarded_packets": 0}
        },
    }
    section = _drops_section([run_dir], payload)
    assert "events_discarded=10" in section
    assert "disagrees" in section


def test_drops_section_missing_events_discarded_is_silent(tmp_path):
    run_dir = tmp_path / "run_no_metadata_field"
    run_dir.mkdir()
    (run_dir / "metadata.yaml").write_text('mode: "lfc"\n')
    payload = {"metrics": {"drops": {"available": True, "summary": []}}, "data_quality": {}}
    section = _drops_section([run_dir], payload)
    assert "Data-quality warnings" not in section


# ── generate_report / end-to-end ─────────────────────────────────────────


def test_generate_report_writes_report_md(two_run_dirs, system_json, tmp_path):
    run1, run2 = two_run_dirs
    outdir = tmp_path / "out"
    payload = _run_report_payload([run1, run2], system_json)
    from lf_eval.model import load_system, resolve_chains

    system = load_system(system_json)
    chains = resolve_chains(system, None)
    report_path = generate_report(
        run_dirs=[run1, run2],
        metrics_payload=payload,
        chains=chains,
        out=outdir,
        title="Test comparison",
    )
    assert report_path.is_file()
    text = report_path.read_text()
    assert "# Test comparison" in text
    assert "## Runs" in text
    assert "## Comparison at a glance" in text
    assert "## End-to-end latency" in text
    assert "## Determinism" in text
    assert "## CPU utilisation" in text
    assert "## Drops" in text
    assert "## Reproducibility" in text


def test_generate_report_default_title(two_run_dirs, system_json, tmp_path):
    run1, run2 = two_run_dirs
    payload = _run_report_payload([run1, run2], system_json)
    report_path = generate_report(
        run_dirs=[run1, run2], metrics_payload=payload, chains=[], out=tmp_path / "out2"
    )
    assert "rcllf benchmark comparison report" in report_path.read_text()


# ── CLI wiring: `lf-eval report` ─────────────────────────────────────────


def test_cli_report_writes_metrics_and_report(two_run_dirs, system_json, tmp_path):
    run1, run2 = two_run_dirs
    outdir = tmp_path / "analysis"
    code = main(
        [
            "report",
            str(run1),
            str(run2),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--no-plots",
            "--title",
            "CLI report test",
        ]
    )
    assert code == 0
    assert (outdir / "metrics.json").is_file()
    report_text = (outdir / "report.md").read_text()
    assert "# CLI report test" in report_text


def test_cli_report_resolves_system_per_run_when_omitted(two_run_dirs, tmp_path):
    """Design R4: `report` without --system uses each run dir's own copy."""
    run1, run2 = two_run_dirs
    for run_dir in (run1, run2):
        (run_dir / "system.json").write_text(json.dumps(PIPELINE_SYSTEM_JSON))
    outdir = tmp_path / "analysis"
    code = main(
        ["report", str(run1), str(run2), "--out", str(outdir), "--no-plots"]
    )
    assert code == 0
    payload = json.loads((outdir / "metrics.json").read_text())
    # A resolved system means chains derive and e2e_latency has real data.
    assert payload["chains"], "expected a chain derived from the per-run system.json"
    assert payload["metrics"]["e2e_latency"]["available"] is True


def test_cli_report_without_any_system_json_skips_chain_metrics(tmp_path, monkeypatch):
    """No --system and no per-run copies: behaves like analyze without --system."""
    run_dir = tmp_path / "20260810T143000Z_lfc_default_r0"
    events = _write_pipeline_trace(run_dir, "lfc")

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        for event in events:
            if event_names is None or event.name in event_names:
                yield event

    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    monkeypatch.setattr("lf_eval.reader.available_backends", lambda: ["cli"])

    outdir = tmp_path / "analysis"
    code = main(["report", str(run_dir), "--out", str(outdir), "--no-plots"])
    assert code == 0
    payload = json.loads((outdir / "metrics.json").read_text())
    assert payload["chains"] == []
    assert payload["metrics"]["e2e_latency"]["available"] is False


def test_cli_report_help_mentions_optional_system(capsys):
    with pytest.raises(SystemExit):
        main(["report", "--help"])
    out = capsys.readouterr().out
    assert "--system" in out


def test_rslcpp_runs_get_the_operating_point_caveat(two_run_dirs, system_json, tmp_path):
    """RSLCPP's physical numbers are descriptive, never a matched comparison."""
    from lf_eval.model import load_system, resolve_chains

    run1, run2 = two_run_dirs
    chains = resolve_chains(load_system(system_json), None)
    payload = _run_report_payload([run1, run2], system_json)
    plain = generate_report(run_dirs=[run1, run2], metrics_payload=payload, chains=chains,
                            out=tmp_path / "a", title="t").read_text()
    assert "simulated time" not in plain
    md = run2 / "metadata.yaml"
    md.write_text(md.read_text().replace('mode: "lfc"', 'mode: "rslcpp"'))
    text = generate_report(run_dirs=[run1, run2], metrics_payload=payload, chains=chains,
                           out=tmp_path / "b", title="t").read_text()
    glance = text.split("## Comparison at a glance", 1)[1].split("\n## ", 1)[0]
    assert "RSLCPP runs on simulated time as fast as possible" in glance, glance
    assert "descriptive" in glance and "ordering" in glance, glance
