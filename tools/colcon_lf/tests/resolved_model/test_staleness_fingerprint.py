# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""RCLLF-R003 fingerprint of system.json ignores generator-owned keys.

`colcon lf-generate` writes `component_instances` into system.json in place
after the resolved model is staged; before this fix every component-based
system (sentry_pipeline included) was reported stale by its own generate.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

from colcon_lf.resolved_model.resolve import GENERATOR_OWNED_SYSTEM_JSON_KEYS, input_sha256
from colcon_lf.verb.generate import COMPONENT_INSTANCES_KEY


FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "pubsub_system.json"


def _cli(*args):
    return subprocess.run(
        [sys.executable, "-m", "colcon_lf.resolved_model", *map(str, args)],
        capture_output=True, text=True,
    )


def _resolved(tmp_path):
    """Real analyzed system.json (recorded fixture) resolved by the real CLI."""
    sj = tmp_path / "system.json"
    shutil.copyfile(FIXTURE, sj)
    model = tmp_path / "resolved_model.json"
    res = _cli("resolve", sj, "-o", model, "--workspace", tmp_path)
    assert res.returncode == 0, res.stderr
    return sj, model


def test_generator_key_is_the_one_generate_writes():
    assert COMPONENT_INSTANCES_KEY in GENERATOR_OWNED_SYSTEM_JSON_KEYS


def test_component_instances_written_after_resolve_is_not_stale(tmp_path):
    sj, model = _resolved(tmp_path)
    raw = json.loads(sj.read_text(encoding="utf-8"))
    raw[COMPONENT_INSTANCES_KEY] = [{"class": "PublisherNode", "instance": "publisher_node"}]
    sj.write_text(json.dumps(raw, indent=2), encoding="utf-8")
    res = _cli("check", model, "--workspace", tmp_path)
    assert res.returncode == 0, res.stderr


def test_any_other_change_is_still_stale(tmp_path):
    sj, model = _resolved(tmp_path)
    raw = json.loads(sj.read_text(encoding="utf-8"))
    raw["name"] = raw.get("name", "") + "_changed"
    sj.write_text(json.dumps(raw, indent=2), encoding="utf-8")
    res = _cli("check", model, "--workspace", tmp_path)
    assert res.returncode != 0
    assert "RCLLF-R003 resolved model stale: system_json" in res.stderr
