# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the lf-eval command line interface.

``analyze`` is exercised end to end without any CTF data by monkeypatching the
reader's event source with synthetic events — the whole point of isolating the
reader layer.
"""

from __future__ import annotations

import json

import pytest

from lf_eval.cli import _format_table, build_parser, main

from tests.conftest import PIPELINE_SYSTEM_JSON, MS, EventBuilder


@pytest.fixture
def fake_trace(tmp_path, monkeypatch):
    """A run directory whose trace reads back as the synthetic pipeline."""
    run_dir = tmp_path / "20260730_120000_lfc_synthetic"
    (run_dir / "tracing").mkdir(parents=True)
    (run_dir / "metadata.yaml").write_text(
        'timestamp: "20260730_120000"\nmode: "lfc"\nduration: 2\nlabel: "synthetic"\n'
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
    events = b.build()

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        for event in events:
            if event_names is None or event.name in event_names:
                yield event

    # load_run does `from lf_eval.reader import read_trace` at call time, and
    # cli imports available_backends the same way, so patching the package
    # attributes covers both lookups.
    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    monkeypatch.setattr("lf_eval.reader.available_backends", lambda: ["cli"])
    return run_dir


@pytest.fixture
def system_json(tmp_path):
    path = tmp_path / "system.json"
    path.write_text(json.dumps(PIPELINE_SYSTEM_JSON))
    return path


# ── parser ───────────────────────────────────────────────────────────────


def test_parser_requires_a_subcommand():
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_parser_metrics_splits_on_comma():
    args = build_parser().parse_args(
        ["analyze", "d", "--metrics", "cpu,drops"]
    )
    assert args.metrics == ["cpu", "drops"]


def test_parser_chain_is_repeatable():
    args = build_parser().parse_args(
        ["analyze", "d", "--chain", "A->B", "--chain", "B->C"]
    )
    assert args.chain == ["A->B", "B->C"]


def test_parser_accepts_multiple_run_dirs():
    args = build_parser().parse_args(["analyze", "d1", "d2", "d3"])
    assert args.run_dirs == ["d1", "d2", "d3"]


# ── list-metrics ─────────────────────────────────────────────────────────


def test_list_metrics_prints_all(capsys):
    assert main(["list-metrics"]) == 0
    out = capsys.readouterr().out
    for name in ("e2e_latency", "determinism", "cpu", "drops"):
        assert name in out
    # The extension recipe must be discoverable from the CLI.
    assert "@register" in out


# ── analyze ──────────────────────────────────────────────────────────────


def test_analyze_rejects_missing_directory(tmp_path, capsys):
    assert main(["analyze", str(tmp_path / "nope")]) == 2
    assert "not a directory" in capsys.readouterr().err


def test_analyze_rejects_unknown_metric(tmp_path, capsys):
    assert main(["analyze", str(tmp_path), "--metrics", "ghost"]) == 2
    assert "unknown metric" in capsys.readouterr().err


def test_analyze_rejects_missing_system_json(tmp_path, capsys):
    assert main(["analyze", str(tmp_path), "--system", str(tmp_path / "no.json")]) == 2
    assert "system.json not found" in capsys.readouterr().err


def test_analyze_writes_metrics_json_and_plots(fake_trace, system_json, tmp_path):
    outdir = tmp_path / "analysis"
    code = main(
        [
            "analyze",
            str(fake_trace),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
        ]
    )
    assert code == 0

    metrics_path = outdir / "metrics.json"
    assert metrics_path.is_file()
    payload = json.loads(metrics_path.read_text())

    assert payload["runs"][0]["mode"] == "lfc"
    assert payload["runs"][0]["nodes"] == ["NodeA", "NodeB", "NodeC"]
    assert payload["chains"][0]["nodes"] == ["NodeA", "NodeB", "NodeC"]

    # All four metrics must be present and available.
    for name in ("e2e_latency", "determinism", "cpu", "drops"):
        assert name in payload["metrics"], name
    assert payload["metrics"]["e2e_latency"]["available"] is True
    e2e_rows = payload["metrics"]["e2e_latency"]["summary"]
    assert e2e_rows[0]["mean_ms"] == pytest.approx(7.0)
    # the fake trace has no rmw identity: timestamp pairs give no PASS
    assert payload["metrics"]["determinism"]["summary"][0]["ordering"] == "INCOMPLETE"

    plots = list((outdir / "plots").glob("*.png"))
    assert plots, "expected at least one plot"
    assert all(p.stat().st_size > 0 for p in plots)


def test_analyze_records_plot_paths_relative_to_outdir(fake_trace, system_json, tmp_path):
    outdir = tmp_path / "analysis"
    main(["analyze", str(fake_trace), "--system", str(system_json), "--out", str(outdir)])
    payload = json.loads((outdir / "metrics.json").read_text())
    for plot in payload["metrics"]["e2e_latency"]["plots"]:
        assert plot.startswith("plots/")
        assert (outdir / plot).is_file()


def test_analyze_no_plots_skips_rendering(fake_trace, system_json, tmp_path):
    outdir = tmp_path / "analysis"
    main(
        [
            "analyze",
            str(fake_trace),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--no-plots",
        ]
    )
    assert not (outdir / "plots").exists()
    assert (outdir / "metrics.json").is_file()


def test_analyze_metric_subset(fake_trace, system_json, tmp_path):
    outdir = tmp_path / "analysis"
    main(
        [
            "analyze",
            str(fake_trace),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--metrics",
            "cpu",
        ]
    )
    payload = json.loads((outdir / "metrics.json").read_text())
    assert set(payload["metrics"]) == {"cpu"}


def test_analyze_explicit_chain_spec(fake_trace, system_json, tmp_path):
    outdir = tmp_path / "analysis"
    code = main(
        [
            "analyze",
            str(fake_trace),
            "--system",
            str(system_json),
            "--chain",
            "NodeA->NodeC",
            "--out",
            str(outdir),
        ]
    )
    assert code == 0
    payload = json.loads((outdir / "metrics.json").read_text())
    assert payload["chains"][0]["nodes"] == ["NodeA", "NodeC"]
    assert payload["chains"][0]["source"] == "spec"


def test_analyze_bad_chain_spec_is_rejected(fake_trace, system_json, capsys):
    code = main(
        ["analyze", str(fake_trace), "--system", str(system_json), "--chain", "NodeA->Ghost"]
    )
    assert code == 2
    assert "unknown node" in capsys.readouterr().err


def test_analyze_without_system_skips_chain_metrics(fake_trace, tmp_path, capsys):
    outdir = tmp_path / "analysis"
    code = main(["analyze", str(fake_trace), "--out", str(outdir)])
    assert code == 0
    payload = json.loads((outdir / "metrics.json").read_text())
    assert payload["chains"] == []
    # Latency needs a chain and must report itself as skipped, not crash.
    assert payload["metrics"]["e2e_latency"]["available"] is False
    # CPU and drops do not need a chain.
    assert payload["metrics"]["cpu"]["available"] is True
    assert "none resolved" in capsys.readouterr().out


def test_analyze_keep_raw_includes_samples(fake_trace, system_json, tmp_path):
    outdir = tmp_path / "analysis"
    main(
        [
            "analyze",
            str(fake_trace),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--keep-raw",
        ]
    )
    payload = json.loads((outdir / "metrics.json").read_text())
    chains = payload["metrics"]["e2e_latency"]["data"]["per_run"]
    run_data = next(iter(chains.values()))
    assert run_data["chains"]["primary"]["e2e"]["raw_us"]


def test_analyze_reports_backend_unavailable(fake_trace, monkeypatch, capsys):
    monkeypatch.setattr("lf_eval.reader.available_backends", lambda: [])
    assert main(["analyze", str(fake_trace)]) == 3
    assert "babeltrace" in capsys.readouterr().err


# ── --fail-on-missing-edge (topology soundness gate) ────────────────────


def test_parser_fail_on_missing_edge_defaults_false():
    args = build_parser().parse_args(["analyze", "d"])
    assert args.fail_on_missing_edge is False


def test_parser_fail_on_missing_edge_flag_on_analyze():
    args = build_parser().parse_args(["analyze", "d", "--fail-on-missing-edge"])
    assert args.fail_on_missing_edge is True


def test_parser_fail_on_missing_edge_flag_on_report():
    args = build_parser().parse_args(["report", "d", "--fail-on-missing-edge"])
    assert args.fail_on_missing_edge is True


# ── --include-infra-topics ──────────────────────────────────


def test_parser_include_infra_topics_defaults_false():
    args = build_parser().parse_args(["analyze", "d"])
    assert args.include_infra_topics is False


def test_parser_include_infra_topics_flag_on_analyze():
    args = build_parser().parse_args(["analyze", "d", "--include-infra-topics"])
    assert args.include_infra_topics is True


def test_parser_include_infra_topics_flag_on_report():
    args = build_parser().parse_args(["report", "d", "--include-infra-topics"])
    assert args.include_infra_topics is True


def test_analyze_fail_on_missing_edge_clean_trace_stays_zero(
    fake_trace, system_json, tmp_path
):
    """fake_trace's runtime edges are exactly system_json's declared edges."""
    outdir = tmp_path / "analysis"
    code = main(
        [
            "analyze",
            str(fake_trace),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--metrics",
            "topology",
            "--fail-on-missing-edge",
        ]
    )
    assert code == 0
    payload = json.loads((outdir / "metrics.json").read_text())
    assert payload["metrics"]["topology"]["data"].get("failed") is not True


def test_analyze_fail_on_missing_edge_exits_nonzero_on_missing_edge(
    system_json, tmp_path, monkeypatch
):
    """A runtime publisher not declared in system.json must fail the run."""
    from tests.conftest import EventBuilder

    run_dir = tmp_path / "20260805_120000_lfc_extra_edge"
    (run_dir / "tracing").mkdir(parents=True)
    (run_dir / "metadata.yaml").write_text(
        'timestamp: "20260805_120000"\nmode: "lfc"\nduration: 1\nlabel: "extra"\n'
    )

    b = EventBuilder()
    for node in ("NodeA", "NodeB", "NodeC"):
        b.add_node(node)
    b.add_timer("NodeA", 100.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")
    # Runtime edge the static model never declared — the dangerous class.
    b.add_publisher("NodeA", "/undeclared_out")
    events = b.build()

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        for event in events:
            if event_names is None or event.name in event_names:
                yield event

    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    monkeypatch.setattr("lf_eval.reader.available_backends", lambda: ["cli"])

    outdir = tmp_path / "analysis"
    code = main(
        [
            "analyze",
            str(run_dir),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--metrics",
            "topology",
            "--fail-on-missing-edge",
        ]
    )
    assert code == 1
    payload = json.loads((outdir / "metrics.json").read_text())
    assert payload["metrics"]["topology"]["data"]["failed"] is True


def test_analyze_missing_edge_without_flag_stays_zero(
    system_json, tmp_path, monkeypatch
):
    """Without --fail-on-missing-edge, topology stays advisory (exit 0)."""
    from tests.conftest import EventBuilder

    run_dir = tmp_path / "20260805_120000_lfc_extra_edge"
    (run_dir / "tracing").mkdir(parents=True)
    (run_dir / "metadata.yaml").write_text(
        'timestamp: "20260805_120000"\nmode: "lfc"\nduration: 1\nlabel: "extra"\n'
    )

    b = EventBuilder()
    for node in ("NodeA", "NodeB", "NodeC"):
        b.add_node(node)
    b.add_timer("NodeA", 100.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")
    b.add_publisher("NodeA", "/undeclared_out")
    events = b.build()

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        for event in events:
            if event_names is None or event.name in event_names:
                yield event

    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    monkeypatch.setattr("lf_eval.reader.available_backends", lambda: ["cli"])

    outdir = tmp_path / "analysis"
    code = main(
        [
            "analyze",
            str(run_dir),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--metrics",
            "topology",
        ]
    )
    assert code == 0
    payload = json.loads((outdir / "metrics.json").read_text())
    assert payload["metrics"]["topology"]["data"].get("failed") is not True
    assert payload["metrics"]["topology"]["data"]["per_run"]


def test_report_fail_on_missing_edge_exits_nonzero(
    system_json, tmp_path, monkeypatch
):
    from tests.conftest import EventBuilder

    run_dir = tmp_path / "20260805_120000_lfc_extra_edge"
    (run_dir / "tracing").mkdir(parents=True)
    (run_dir / "metadata.yaml").write_text(
        'timestamp: "20260805_120000"\nmode: "lfc"\nduration: 1\nlabel: "extra"\n'
    )
    (run_dir / "system.json").write_text(system_json.read_text())

    b = EventBuilder()
    for node in ("NodeA", "NodeB", "NodeC"):
        b.add_node(node)
    b.add_timer("NodeA", 100.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")
    b.add_publisher("NodeA", "/undeclared_out")
    events = b.build()

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        for event in events:
            if event_names is None or event.name in event_names:
                yield event

    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    monkeypatch.setattr("lf_eval.reader.available_backends", lambda: ["cli"])

    outdir = tmp_path / "analysis"
    code = main(
        [
            "report",
            str(run_dir),
            "--out",
            str(outdir),
            "--metrics",
            "topology",
            "--fail-on-missing-edge",
        ]
    )
    assert code == 1


# ── sample-cpu ───────────────────────────────────────────────────────────


def test_sample_cpu_list_pids(tmp_path, capsys):
    for pid, cmdline in ((7, "my_node --ros-args"), (8, "other")):
        directory = tmp_path / str(pid)
        directory.mkdir()
        (directory / "cmdline").write_bytes(cmdline.replace(" ", "\0").encode())
    code = main(
        ["sample-cpu", "--patterns", "my_node", "--proc-root", str(tmp_path), "--list-pids"]
    )
    assert code == 0
    assert capsys.readouterr().out.strip() == "7"


def test_sample_cpu_requires_out(capsys):
    assert main(["sample-cpu", "--patterns", "x"]) == 2
    assert "--out is required" in capsys.readouterr().err


def test_sample_cpu_rejects_empty_patterns(capsys):
    assert main(["sample-cpu", "--patterns", ",", "--out", "/tmp/x.json"]) == 2
    assert "at least one substring" in capsys.readouterr().err


def test_sample_cpu_reports_no_match(tmp_path, capsys):
    code = main(
        [
            "sample-cpu",
            "--patterns",
            "ghost",
            "--proc-root",
            str(tmp_path),
            "--out",
            str(tmp_path / "o.json"),
            "--warmup",
            "0",
            "--window",
            "0",
        ]
    )
    assert code == 1
    assert "no pids matched" in capsys.readouterr().err


# ── table formatting ─────────────────────────────────────────────────────


def test_format_table_aligns_columns():
    table = _format_table([{"a": 1, "b": "xx"}, {"a": 22, "b": "y"}])
    lines = table.splitlines()
    assert len(lines) == 4  # header, separator, two rows
    assert "a" in lines[0] and "b" in lines[0]


def test_format_table_handles_missing_keys_and_none():
    table = _format_table([{"a": 1}, {"b": None}])
    assert "-" in table


def test_format_table_empty():
    assert _format_table([]) == ""


def test_semantic_analyzer_error_is_an_incomplete_claim_not_a_silent_omission(
        fake_trace, system_json, tmp_path, monkeypatch):
    import lf_eval.semantic.reconcile as rec

    def boom(_run_dir):
        raise KeyError("totals")

    monkeypatch.setattr(rec, "analyze_run", boom)
    outdir = tmp_path / "analysis"
    assert main(["analyze", str(fake_trace), "--system", str(system_json), "--out", str(outdir),
                 "--no-plots"]) == 0
    sem = json.loads((outdir / "metrics.json").read_text()).get("semantic")
    assert sem, "run dropped from the semantic block: an analyzer crash must stay visible"
    (claim,) = sem["claims"]
    assert claim["overall"] == "incomplete", claim
    assert claim["integrity_codes"] == ["analyzer_error:KeyError"], claim


def test_analyze_window_is_recorded_and_a_too_short_run_is_refused(fake_trace, system_json, tmp_path, capsys):
    """The protocol window is per run in metrics.json, and never silently shortened."""
    out = tmp_path / "w"
    assert main(["analyze", str(fake_trace), "--system", str(system_json), "--out", str(out),
                 "--no-plots", "--warmup-s", "0"]) == 0
    run = json.loads((out / "metrics.json").read_text())["runs"][0]
    assert run["window"] == {"warmup_s": 0.0, "steady_s": None, "anchor": "first callback_start"}
    code = main(["analyze", str(fake_trace), "--system", str(system_json), "--out", str(tmp_path / "x"),
                 "--no-plots", "--window-s", "3600"])
    assert code == 2
    assert "ends after the last runtime event" in capsys.readouterr().err


def test_analyze_topology_decodes_the_trace_once(system_json, tmp_path, monkeypatch):
    """Timeline and topology share one CTF decode.

    Before the fix the topology metric re-read the trace after ``load_run``,
    so ``read_trace`` ran twice for one run.
    """
    from tests.conftest import EventBuilder

    run_dir = tmp_path / "20260805_120000_lfc_one_decode"
    (run_dir / "tracing").mkdir(parents=True)
    (run_dir / "metadata.yaml").write_text(
        'timestamp: "20260805_120000"\nmode: "lfc"\nduration: 1\nlabel: "one"\n'
    )

    b = EventBuilder()
    for node in ("NodeA", "NodeB", "NodeC"):
        b.add_node(node)
    b.add_timer("NodeA", 100.0)
    b.add_publisher("NodeA", "/a_out")
    b.add_subscription("NodeB", "/a_out")
    b.add_publisher("NodeB", "/b_out")
    b.add_subscription("NodeC", "/b_out")
    b.add_publisher("NodeA", "/undeclared_out")
    events = b.build()
    decodes = []

    def fake_read_trace(path, backend=None, event_names=None, discards=None):
        decodes.append(event_names)
        for event in events:
            if event_names is None or event.name in event_names:
                yield event

    monkeypatch.setattr("lf_eval.reader.read_trace", fake_read_trace)
    monkeypatch.setattr("lf_eval.reader.available_backends", lambda: ["cli"])

    outdir = tmp_path / "analysis"
    code = main(
        [
            "analyze",
            str(run_dir),
            "--system",
            str(system_json),
            "--out",
            str(outdir),
            "--metrics",
            "topology",
        ]
    )
    assert code == 0
    assert len(decodes) == 1, f"trace decoded {len(decodes)} times, expected 1"
    payload = json.loads((outdir / "metrics.json").read_text())
    missing = payload["metrics"]["topology"]["data"]["per_run"][run_dir.name][
        "missing_from_model"
    ]
    assert any(row["topic"] == "/undeclared_out" for row in missing), missing
