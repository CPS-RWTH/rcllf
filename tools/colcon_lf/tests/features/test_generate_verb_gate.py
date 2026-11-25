# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Generate-verb hazard-gate tests (00_design.md section 3 pipeline wiring).

- schema_version < 3 or missing `diagnostics` field -> refuse.
- error-severity diagnostics (RCLLF-H003) -> refuse, unless acknowledged via
  execution.yaml's `acknowledged_hazards:`.
- warnings/info diagnostics never block.
"""

from __future__ import annotations

import json
import textwrap
from pathlib import Path

import pytest

colcon_core = pytest.importorskip("colcon_core.verb", reason="colcon_core not available")



@pytest.fixture(autouse=True)
def _no_lfc_package_emission(monkeypatch):
    """These tests exercise the exec-config and hazard gates, not package
    emission. Every lf-generate emits the lfc package, which
    stages the user's C++ sources from the fixture's /workspace paths; stub it
    so the gates stay testable on hosts without that tree."""
    import colcon_lf.verb.generate as gen

    monkeypatch.setattr(gen, "_emit_lfc_artifacts", lambda **kwargs: None)

def _minimal_system_json(*, schema_version=3, include_diagnostics=True, diagnostics=None) -> dict:
    """A minimal-but-complete system.json dict good enough for _generate_one."""
    data = {
        "schema_version": schema_version,
        "name": "Bringup",
        "launch_graph": {
            "root": {
                "file_path": "/workspace/packages/src/test_pkg/launch/bringup.launch.py",
                "package": "test_pkg",
                "arguments": [],
                "nodes": [],
                "includes": [],
                "groups": [],
                "errors": [],
                "context_states": [],
            }
        },
        "nodes": {
            "test_node": {
                "name": "test_node",
                "class_name": "TestNode",
                "package": "test_pkg",
                "source_file": "/workspace/packages/src/test_pkg/src/test_node.cpp",
                "header_file": "/workspace/packages/src/test_pkg/include/test_pkg/test_node.hpp",
                "is_component": False,
                "callbacks": [],
                "timers": [
                    {
                        "period_ms": 500.0,
                        "callback": "timer_callback",
                        "callback_group": "default",
                        "member_variable": "timer_",
                        "created_in": "constructor",
                        "destroyed_in": None,
                        "offset_ms": 0.0,
                    }
                ],
                "subscribers": [],
                "publishers": [],
                "callback_groups": [{"name": "default", "type": "MutuallyExclusive"}],
                "base_class": "rclcpp::Node",
                "service_servers": [],
                "action_servers": [],
            },
        },
        "communication": {"topics": [], "services": [], "actions": []},
    }
    if include_diagnostics:
        data["diagnostics"] = diagnostics if diagnostics is not None else []
    return data


def _write_system_json(tmp_path: Path, data: dict) -> Path:
    sj_path = tmp_path / "lf-analysis" / "test_pkg" / "bringup" / "system.json"
    sj_path.parent.mkdir(parents=True)
    sj_path.write_text(json.dumps(data))
    return sj_path


class TestSchemaVersionGate:
    def test_refuses_on_schema_version_2(self, tmp_path):
        from colcon_lf.verb.generate import _generate_one

        sj = _write_system_json(tmp_path, _minimal_system_json(schema_version=2))
        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 1

    def test_refuses_on_missing_schema_version(self, tmp_path):
        from colcon_lf.verb.generate import _generate_one

        data = _minimal_system_json()
        del data["schema_version"]
        sj = _write_system_json(tmp_path, data)
        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 1

    def test_refuses_on_missing_diagnostics_field_even_at_v3(self, tmp_path):
        """Missing field != empty list — must refuse even at schema_version 3."""
        from colcon_lf.verb.generate import _generate_one

        sj = _write_system_json(
            tmp_path, _minimal_system_json(schema_version=3, include_diagnostics=False)
        )
        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 1

    def test_proceeds_on_v3_with_empty_diagnostics(self, tmp_path):
        from colcon_lf.verb.generate import _generate_one

        sj = _write_system_json(tmp_path, _minimal_system_json(schema_version=3, diagnostics=[]))
        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 0


class TestHazardSeverityGate:
    def test_warning_diagnostics_never_block(self, tmp_path):
        from colcon_lf.verb.generate import _generate_one

        diags = [
            {
                "code": "RCLLF-H001",
                "node": "test_node",
                "topic": "t",
                "message": "best_effort input",
            }
        ]
        sj = _write_system_json(tmp_path, _minimal_system_json(diagnostics=diags))
        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 0

    def test_info_diagnostics_never_block(self, tmp_path):
        from colcon_lf.verb.generate import _generate_one

        diags = [
            {"code": "RCLLF-H000", "node": "test_node", "topic": "t", "message": "unknown qos"}
        ]
        sj = _write_system_json(tmp_path, _minimal_system_json(diagnostics=diags))
        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 0

    def test_error_diagnostic_h003_refuses_generation(self, tmp_path):
        from colcon_lf.verb.generate import _generate_one

        diags = [
            {
                "code": "RCLLF-H003",
                "node": "test_node",
                "topic": None,
                "message": "message_filters detected",
            }
        ]
        sj = _write_system_json(tmp_path, _minimal_system_json(diagnostics=diags))
        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 1

    def test_acknowledged_h003_proceeds_with_loud_log(self, tmp_path, capsys):
        from colcon_lf.verb.generate import _generate_one

        diags = [
            {
                "code": "RCLLF-H003",
                "node": "test_node",
                "topic": None,
                "message": "message_filters detected",
            }
        ]
        sj = _write_system_json(tmp_path, _minimal_system_json(diagnostics=diags))
        config_dir = sj.parent / "config"
        config_dir.mkdir()
        (config_dir / "execution.yaml").write_text(
            textwrap.dedent(
                """\
                schema_version: 1
                acknowledged_hazards:
                  - RCLLF-H003
                """
            ),
            encoding="utf-8",
        )

        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 0
        captured = capsys.readouterr()
        assert "RCLLF-H003" in captured.err
        assert "acknowledged" in captured.err.lower()

    def test_unacknowledged_other_error_code_still_refuses_when_h003_acknowledged(
        self, tmp_path
    ):
        """Acknowledging RCLLF-H003 must not blanket-acknowledge other error codes."""
        from colcon_lf.verb.generate import _generate_one

        diags = [
            {
                "code": "RCLLF-H003",
                "node": "test_node",
                "topic": None,
                "message": "message_filters detected",
            },
        ]
        sj = _write_system_json(tmp_path, _minimal_system_json(diagnostics=diags))
        config_dir = sj.parent / "config"
        config_dir.mkdir()
        # Acknowledge a code that is NOT present — H003 stays unacknowledged.
        (config_dir / "execution.yaml").write_text(
            "schema_version: 1\nacknowledged_hazards:\n  - RCLLF-SOME-OTHER-CODE\n",
            encoding="utf-8",
        )
        rc = _generate_one(
            system_json=sj, output_dir=None, after_delay="1 msec", generate_launch=False
        )
        assert rc == 1
