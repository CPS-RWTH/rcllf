# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Integration test: colcon lf-analyze runs hazard detection and persists

diagnostics (facts only) into system.json at schema_version 3.

Mocks ``build_system_graph`` with a synthetic hazard-triggering graph so the
test doesn't need libclang/ROS headers — the analyze-verb wiring (not the
extraction itself) is what's under test here.
"""

from __future__ import annotations

import json
from unittest.mock import patch

import pytest

colcon_core = pytest.importorskip("colcon_core.verb", reason="colcon_core not available")

from tests.features.conftest import make_graph, make_node, make_sub  # noqa: E402


class TestAnalyzeVerbHazardWiring:
    def test_diagnostics_persisted_at_schema_v3(self, tmp_path):
        from colcon_lf.verb import analyze as analyze_mod

        graph = make_graph(
            {
                "/n": make_node(
                    "n",
                    subscribers=[
                        make_sub("t_unknown"),
                        make_sub("t_best_effort", qos_reliability="best_effort"),
                    ],
                )
            }
        )

        with patch(
            "colcon_lf.graph.hierarchical.build_system_graph", return_value=graph
        ):
            rc = analyze_mod._analyze_one(
                package="test_pkg",
                launch_file="bringup.launch.py",
                launch_path=tmp_path / "bringup.launch.py",
                workspace=tmp_path,
                output_dir=str(tmp_path / "out"),
                output_fmt="json",
            )

        assert rc == 0
        system_json = tmp_path / "out" / "test_pkg" / "bringup" / "system.json"
        assert system_json.is_file()
        data = json.loads(system_json.read_text())
        assert data["schema_version"] == 3
        assert "diagnostics" in data
        codes = {d["code"] for d in data["diagnostics"]}
        assert "RCLLF-H000" in codes
        assert "RCLLF-H001" in codes
        # Facts only — no severity key persisted.
        for diag in data["diagnostics"]:
            assert set(diag.keys()) == {"code", "node", "topic", "message"}

    def test_hazard_free_graph_persists_empty_diagnostics_list(self, tmp_path):
        from colcon_lf.verb import analyze as analyze_mod

        graph = make_graph({"/n": make_node("n", subscribers=[])})

        with patch(
            "colcon_lf.graph.hierarchical.build_system_graph", return_value=graph
        ):
            rc = analyze_mod._analyze_one(
                package="test_pkg",
                launch_file="bringup.launch.py",
                launch_path=tmp_path / "bringup.launch.py",
                workspace=tmp_path,
                output_dir=str(tmp_path / "out"),
                output_fmt="json",
            )

        assert rc == 0
        system_json = tmp_path / "out" / "test_pkg" / "bringup" / "system.json"
        data = json.loads(system_json.read_text())
        assert data["diagnostics"] == []
