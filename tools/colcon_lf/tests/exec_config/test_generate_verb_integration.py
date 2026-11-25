# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Integration test: `colcon lf-generate --exec-config` stages exec_config.json.

Uses the real system_bringup fixture (skip-guarded if not on disk). Exercises
_generate_one end to end: default config/execution.yaml discovery, explicit
--exec-config override, and generate_launch_artifacts wiring the parameter
into the emitted launch file.
"""

from __future__ import annotations

import json
import shutil
import textwrap
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[4]
_SYSTEM_BRINGUP_JSON = (
    _REPO_ROOT / "packages" / "lf-analysis" / "system_bringup" / "system_bringup"
    / "system.json"
)



@pytest.fixture(autouse=True)
def _no_lfc_package_emission(monkeypatch):
    """These tests exercise the exec-config and hazard gates, not package
    emission. Every lf-generate emits the lfc package, which
    stages the user's C++ sources from the fixture's /workspace paths; stub it
    so the gates stay testable on hosts without that tree."""
    import colcon_lf.verb.generate as gen

    monkeypatch.setattr(gen, "_emit_lfc_artifacts", lambda **kwargs: None)

def _copy_fixture(tmp_path: Path) -> Path:
    work_dir = tmp_path / "system_bringup"
    work_dir.mkdir()
    system_json = work_dir / "system.json"
    shutil.copy(_SYSTEM_BRINGUP_JSON, system_json)
    return system_json


class TestGenerateOneDefaultExecConfig:
    def test_no_config_present_uses_builtin_defaults(self, tmp_path):
        if not _SYSTEM_BRINGUP_JSON.is_file():
            pytest.skip(f"system.json not found: {_SYSTEM_BRINGUP_JSON}")

        from colcon_lf.verb.generate import _generate_one

        system_json = _copy_fixture(tmp_path)
        rc = _generate_one(
            system_json=system_json,
            output_dir=None,
            after_delay="1 msec",
            generate_launch=True,
        )
        assert rc == 0

        exec_config_json = system_json.parent / "exec_config.json"
        assert exec_config_json.is_file()
        data = json.loads(exec_config_json.read_text())
        assert data["schema_version"] == 1
        assert data["defaults"] == {"take_mode": "skip", "take_timeout_ms": 100}

        # The lfc launch file (the only launch path) hands
        # this staged file to the runtime through ROS2_LF_EXEC_CONFIG; that is
        # checked by the runtime smoke tests against a real binary.

    def test_config_dir_execution_yaml_picked_up_by_default(self, tmp_path):
        if not _SYSTEM_BRINGUP_JSON.is_file():
            pytest.skip(f"system.json not found: {_SYSTEM_BRINGUP_JSON}")

        from colcon_lf.verb.generate import _generate_one

        system_json = _copy_fixture(tmp_path)
        config_dir = system_json.parent / "config"
        config_dir.mkdir()
        (config_dir / "execution.yaml").write_text(textwrap.dedent("""\
            schema_version: 1
            subscriptions:
              /robot_1/subscriber_node:
                sensor_data:
                  take_mode: block
        """), encoding="utf-8")

        rc = _generate_one(
            system_json=system_json,
            output_dir=None,
            after_delay="1 msec",
            generate_launch=False,
        )
        assert rc == 0

        exec_config_json = system_json.parent / "exec_config.json"
        data = json.loads(exec_config_json.read_text())
        rows = [
            row for row in data["subscriptions"]
            if row["node_class"] == "SubscriberNode" and row["topic"] == "sensor_data"
        ]
        assert len(rows) == 1
        assert rows[0]["take_mode"] == "block"

    def test_explicit_exec_config_flag_overrides_default(self, tmp_path):
        if not _SYSTEM_BRINGUP_JSON.is_file():
            pytest.skip(f"system.json not found: {_SYSTEM_BRINGUP_JSON}")

        from colcon_lf.verb.generate import _generate_one

        system_json = _copy_fixture(tmp_path)
        override_yaml = tmp_path / "override_execution.yaml"
        override_yaml.write_text(textwrap.dedent("""\
            schema_version: 1
            defaults:
              take_mode: block
        """), encoding="utf-8")

        rc = _generate_one(
            system_json=system_json,
            output_dir=None,
            after_delay="1 msec",
            generate_launch=False,
            exec_config_arg=str(override_yaml),
        )
        assert rc == 0
        data = json.loads((system_json.parent / "exec_config.json").read_text())
        assert data["defaults"]["take_mode"] == "block"

    def test_malformed_exec_config_is_hard_error(self, tmp_path):
        if not _SYSTEM_BRINGUP_JSON.is_file():
            pytest.skip(f"system.json not found: {_SYSTEM_BRINGUP_JSON}")

        from colcon_lf.verb.generate import _generate_one

        system_json = _copy_fixture(tmp_path)
        bad_yaml = tmp_path / "bad_execution.yaml"
        bad_yaml.write_text("schema_version: 1\ndefaults:\n  take_mode: bogus\n")

        rc = _generate_one(
            system_json=system_json,
            output_dir=None,
            after_delay="1 msec",
            generate_launch=False,
            exec_config_arg=str(bad_yaml),
        )
        assert rc == 1

    def test_missing_exec_config_file_is_hard_error(self, tmp_path):
        if not _SYSTEM_BRINGUP_JSON.is_file():
            pytest.skip(f"system.json not found: {_SYSTEM_BRINGUP_JSON}")

        from colcon_lf.verb.generate import _generate_one

        system_json = _copy_fixture(tmp_path)
        rc = _generate_one(
            system_json=system_json,
            output_dir=None,
            after_delay="1 msec",
            generate_launch=False,
            exec_config_arg=str(tmp_path / "does_not_exist.yaml"),
        )
        assert rc == 1
