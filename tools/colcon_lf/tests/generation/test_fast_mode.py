# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""LF fast mode and logical timeout for generated programs.

``colcon lf-generate --fast --timeout "20 sec"`` emits ``timeout: 20 sec``
into the target block of the one program and of the federated top-level file,
and bakes the reactor-c argument ``-f true`` into the generated launch files
(the one program's and every federate's). It does NOT emit ``fast: true``:
lfc refuses that target property in a program with physical actions, and every
rcllf node reactor has one. ``RCLLF_LF_ARGS`` passes reactor-c arguments (for
example ``-f true -o 20 sec``) to a built program through both launch files,
after the baked ones, so a program runs fast without regeneration and a
fast-generated program runs in real time with ``-f false``.
"""

from __future__ import annotations

import dataclasses
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from colcon_lf.federation.partition import FederationMap
from ..corr_declared import declared
from colcon_lf.generation.lf_generator import run_mode_launch_args, run_mode_target_options
from colcon_lf.generation.lf_time import format_lf_time_ns
from colcon_lf.verb.generate import _generate_federated, _generate_one

from .test_xfed_correspondence import FED, _graph, _pub, _sub, _tc

PKG = "lf_generated_test_pkg_bringup"


# ---------------------------------------------------------------------------
# The option values
# ---------------------------------------------------------------------------

def test_run_mode_options_are_empty_by_default():
    assert run_mode_target_options() == {}
    assert run_mode_target_options(fast=False, timeout=None) == {}


def test_run_mode_options_emit_a_canonical_timeout_and_never_fast():
    # lfc 0.12: "the fast target property is incompatible with physical
    # actions"; fast mode is a launch argument instead.
    assert run_mode_target_options(fast=True) == {}
    assert run_mode_launch_args(fast=False) == []
    assert run_mode_launch_args(fast=True) == ["-f", "true"]
    assert run_mode_target_options(timeout="20 sec") == {"timeout": "20 sec"}
    assert run_mode_target_options(fast=True, timeout="20000 msec") == {
        "timeout": "20 sec",
    }
    assert run_mode_target_options(timeout="1500ms") == {"timeout": "1500 msec"}


def test_run_mode_refuses_a_timeout_that_is_not_an_lf_time():
    with pytest.raises(ValueError, match="--timeout"):
        run_mode_target_options(timeout="twenty seconds")


@pytest.mark.parametrize("ns, text", [
    (0, "0 sec"), (20_000_000_000, "20 sec"), (1_500_000_000, "1500 msec"),
    (2_500, "2500 nsec"), (3_000, "3 usec"), (7, "7 nsec"),
])
def test_format_lf_time_is_the_shortest_exact_literal(ns, text):
    assert format_lf_time_ns(ns) == text


# ---------------------------------------------------------------------------
# The one program
# ---------------------------------------------------------------------------

def _one_program(tmp_path: Path, **run_mode) -> tuple[str, str]:
    """Generate a one-timer program with its launch; return (lf text, launch text)."""
    src = tmp_path / "packages/src/test_pkg/src"
    src.mkdir(parents=True)
    (src / "test_node.cpp").write_text("// stub\n")
    inc = tmp_path / "packages/src/test_pkg/include/test_pkg"
    inc.mkdir(parents=True)
    (inc / "test_node.hpp").write_text("// stub\n")
    data = {
        "schema_version": 3,
        "name": "Bringup",
        "launch_graph": {"root": {
            "file_path": str(tmp_path / "packages/src/test_pkg/launch/bringup.launch.py"),
            "package": "test_pkg", "arguments": [], "nodes": [], "includes": [],
            "groups": [], "errors": [], "context_states": [],
        }},
        "nodes": {"test_node": {
            "name": "test_node", "class_name": "TestNode", "package": "test_pkg",
            "source_file": str(src / "test_node.cpp"),
            "header_file": str(inc / "test_node.hpp"),
            "is_component": False, "callbacks": [],
            "timers": [{"period_ms": 500.0, "callback": "timer_callback",
                        "callback_group": "default", "member_variable": "timer_",
                        "created_in": "constructor", "destroyed_in": None,
                        "offset_ms": 0.0}],
            "subscribers": [], "publishers": [],
            "callback_groups": [{"name": "default", "type": "MutuallyExclusive"}],
            "base_class": "rclcpp::Node", "service_servers": [], "action_servers": [],
        }},
        "communication": {"topics": [], "services": [], "actions": []},
        "diagnostics": [],
    }
    sj = tmp_path / "packages/lf-analysis/test_pkg/bringup/system.json"
    sj.parent.mkdir(parents=True)
    sj.write_text(json.dumps(data))
    rc = _generate_one(system_json=sj, output_dir=None, after_delay="1 msec",
                       generate_launch=True, **run_mode)
    assert rc == 0
    pkg = tmp_path / "packages/lf-generated" / PKG
    return (pkg / "lf/Bringup.lf").read_text(), (pkg / "launch/lf_bringup.launch.py").read_text()


def _target_block(lf_text: str) -> str:
    start = lf_text.index("target CCpp {")
    return lf_text[start:lf_text.index("}", start) + 1]


def test_one_program_target_block_is_unchanged_without_the_options(tmp_path):
    lf_text, _ = _one_program(tmp_path)
    block = _target_block(lf_text)
    assert "fast" not in block
    assert "timeout" not in block
    assert "keepalive: true" in block


def test_one_program_emits_timeout_and_bakes_fast_into_the_launch(tmp_path):
    lf_text, launch = _one_program(tmp_path, fast=True, timeout="20000 msec")
    block = _target_block(lf_text)
    assert "fast" not in block  # lfc refuses it next to a physical action
    assert "timeout: 20 sec" in block
    assert "keepalive: true" in block
    assert "arguments=['-f', 'true']\n" in launch
    assert launch.index("['-f', 'true']") < launch.index(
        'shlex.split(os.environ.get("RCLLF_LF_ARGS"')


def test_one_program_launch_passes_rcllf_lf_args(tmp_path):
    _, launch = _one_program(tmp_path)
    assert 'arguments=[]\n' in launch
    assert '+ shlex.split(os.environ.get("RCLLF_LF_ARGS", ""))' in launch
    assert "import shlex" in launch
    compile(launch, "lf_bringup.launch.py", "exec")


# ---------------------------------------------------------------------------
# The federation
# ---------------------------------------------------------------------------

def _federation(tmp_path: Path, **run_mode) -> tuple[str, str]:
    """Generate a two-federate pub/sub with launches; return (top lf, federate launch)."""
    src = tmp_path / "packages/src/test_pkg/src"
    src.mkdir(parents=True, exist_ok=True)
    nodes = []
    for n in (_pub("pub", "PubNode"), _sub("sub", "SubNode")):
        f = src / f"{n.name}.cpp"
        f.write_text("// generated by the test\n")
        nodes.append(dataclasses.replace(n, source_file=str(f)))
    graph = _graph(nodes, [_tc("/data", ["pub"], ["sub"])])
    sj = tmp_path / "lf-analysis/test_pkg/bringup/system.json"
    sj.parent.mkdir(parents=True, exist_ok=True)
    sj.write_text(json.dumps({"name": "bringup", "nodes": {}, "communication": {"topics": []}}))
    fmap = FederationMap(node_to_federate={"pub": "pub", "sub": "sub"},
                         federate_names=["pub", "sub"], after_overrides=[],
                         coordination="centralized")
    rc = _generate_federated(system_json=sj, graph=graph, fmap=fmap, after_delay="0 msec",
                             generate_launch=True, corr_bounds=declared(graph, 1), **run_mode)
    assert rc == 0
    gen = tmp_path / "lf-generated"
    return ((gen / FED / "lf/Bringup_federated.lf").read_text(),
            (gen / f"{FED}_pub/launch/lf_pub.launch.py").read_text())


def test_federated_top_level_emits_timeout_and_federates_bake_fast(tmp_path):
    top, launch = _federation(tmp_path, fast=True, timeout="20 sec")
    block = _target_block(top)
    assert "fast" not in block
    assert "timeout: 20 sec" in block
    assert "args += ['-f', 'true']\n" in launch
    assert launch.index('args = ["-r"') < launch.index("['-f', 'true']") < launch.index(
        "RCLLF_LF_ARGS")
    compile(launch, "lf_pub.launch.py", "exec")


def test_federated_top_level_is_unchanged_without_the_options(tmp_path):
    top, _ = _federation(tmp_path)
    block = _target_block(top)
    assert "fast" not in block
    assert "timeout" not in block


def test_federate_launch_appends_rcllf_lf_args_after_the_rti_arguments(tmp_path):
    _, launch = _federation(tmp_path)
    assert "args += []\n" in launch
    assert 'args += shlex.split(os.environ.get("RCLLF_LF_ARGS", ""))' in launch
    assert launch.index('args = ["-r"') < launch.index("RCLLF_LF_ARGS")
    compile(launch, "lf_pub.launch.py", "exec")


# ---------------------------------------------------------------------------
# The verbs
# ---------------------------------------------------------------------------

def test_generate_verb_refuses_a_bad_timeout_before_generating(tmp_path, capsys, monkeypatch):
    import colcon_lf.verb.generate as gen

    monkeypatch.setattr(gen, "_generate_one", lambda **kw: pytest.fail("generated anyway"))
    args = SimpleNamespace(
        target_lfc=False, federate=None, system_json=str(tmp_path / "missing.json"),
        package=None, output_dir=None, workspace=str(tmp_path), after_delay=None,
        launch=True, emit_federation_template=False, force=False, exec_config=None,
        params_file=[], coordination=None, fast=True, timeout="soon",
    )
    rc = gen.GenerateVerb().main(context=SimpleNamespace(args=args))
    assert rc == 1
    assert "--timeout" in capsys.readouterr().err


def test_colcon_lf_forwards_fast_and_timeout(monkeypatch):
    import colcon_lf.verb.generate as gen
    from colcon_lf.verb.lf import LfVerb

    seen = {}

    def fake_main(self, *, context):
        seen["args"] = context.args
        return 0

    monkeypatch.setattr(gen.GenerateVerb, "main", fake_main)
    args = SimpleNamespace(package="pubsub_example", workspace=None, after_delay=None,
                           fast=True, timeout="20 sec")
    assert LfVerb()._run_generate(args) == 0
    assert seen["args"].fast is True
    assert seen["args"].timeout == "20 sec"


def test_rcllf_lf_args_splits_like_a_shell():
    """The value is split as the shell would; a quoted unit stays one argument."""
    import shlex

    assert shlex.split("-f true -o 20 sec") == ["-f", "true", "-o", "20", "sec"]
    assert shlex.split("") == []
    assert shlex.split(os.environ.get("RCLLF_LF_ARGS_UNSET_FOR_TEST", "")) == []
