# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Scripted-stdin test for the --interactive prompt loop (minimal, per design)."""

from __future__ import annotations

import textwrap

import yaml

from colcon_lf.exec_config.interactive import run_interactive

from .conftest import make_graph, make_node


def test_interactive_writes_only_sparse_file(tmp_path):
    graph = make_graph({"n": make_node("n", subscriptions=["t"])})
    sparse_path = tmp_path / "execution.yaml"
    sparse_path.write_text(textwrap.dedent("""\
        schema_version: 1
        defaults: {}
        nodes: {}
        subscriptions: {}
        acknowledged_hazards: []
    """), encoding="utf-8")

    # Scripted stdin: set defaults.take_mode=block, skip take_timeout_ms,
    # then quit immediately (before nodes/subscriptions prompts).
    scripted_inputs = iter(["block", "", "q"])
    printed: list[str] = []

    def fake_input(prompt: str) -> str:
        return next(scripted_inputs)

    def fake_print(*args, **kwargs):
        printed.append(" ".join(str(a) for a in args))

    run_interactive(sparse_path, graph, input_fn=fake_input, print_fn=fake_print)

    result = yaml.safe_load(sparse_path.read_text())
    assert result["defaults"]["take_mode"] == "block"
    assert "take_timeout_ms" not in result["defaults"]
    assert "maxwait_ms" not in result["defaults"]


def test_interactive_invalid_value_is_skipped(tmp_path):
    graph = make_graph({"n": make_node("n")})
    sparse_path = tmp_path / "execution.yaml"
    sparse_path.write_text("schema_version: 1\n", encoding="utf-8")

    # take_mode gets an invalid value, then quit.
    scripted_inputs = iter(["not_a_valid_mode", "q"])

    def fake_input(prompt: str) -> str:
        return next(scripted_inputs)

    run_interactive(sparse_path, graph, input_fn=fake_input, print_fn=lambda *a, **k: None)

    result = yaml.safe_load(sparse_path.read_text())
    assert "take_mode" not in result["defaults"]


def test_interactive_full_run_completes_without_quit(tmp_path):
    graph = make_graph({"n": make_node("n", subscriptions=["t"])})
    sparse_path = tmp_path / "execution.yaml"
    sparse_path.write_text("schema_version: 1\n", encoding="utf-8")

    # Every prompt: leave unset (empty string). No 'q' needed — the loop
    # terminates naturally once all scopes are visited.
    def fake_input(prompt: str) -> str:
        return ""

    run_interactive(sparse_path, graph, input_fn=fake_input, print_fn=lambda *a, **k: None)

    result = yaml.safe_load(sparse_path.read_text())
    assert result["schema_version"] == 1
    assert result["defaults"] == {}
