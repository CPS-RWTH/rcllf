# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The execution config's ``workers`` key pins the LF worker count.

Unset, reactor-c sizes its worker pool as min(host cores, reaction graph
breadth) (``determine_number_of_workers`` in
``third_party/reactor-c/core/threaded/reactor_threaded.c``), so the same
program runs 8 workers on a 24-thread desk host and 2 on the 2-vCPU CI
runner. Setting ``workers`` removes that host dependence. These tests pin the
path from the YAML key to the ``workers:`` target property.
"""

from __future__ import annotations

import re

import pytest

from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.exec_config.policies import KEY_WORKERS, REGISTRY
from colcon_lf.exec_config.resolve import resolve_corr_bounds
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.launch_analysis.models import LaunchArgument, LaunchFileInfo

from .test_correspondence_emission import _graph, _pub, _sub, _tc


def _target_block(text: str) -> str:
    m = re.search(r"^target \w+ \{\n(.*?)^\}", text, re.S | re.M)
    assert m, f"no target block in:\n{text[:400]}"
    return m.group(1)


def _emit(defaults: dict) -> str:
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    raw = {"defaults": {"max_publishes_per_invocation": 1, **defaults}}
    bounds = resolve_corr_bounds(ExecConfig.from_dict(raw, graph, source="t"), graph)
    return emit_lf_system(generate_lf_system(graph, corr_bounds=bounds))


def test_workers_parses_positive_int_and_defaults_to_unset():
    spec = REGISTRY[KEY_WORKERS]
    assert spec.default is None and spec.runtime is False, spec
    assert spec.parse(1) == 1 and spec.parse(4) == 4
    for bad in (0, -1, True, "1", 1.5):
        with pytest.raises(ValueError, match="workers"):
            spec.parse(bad)


def test_workers_reaches_the_lf_target_block():
    target = _target_block(_emit({"workers": 1}))
    assert re.search(r"^\s*workers: 1,?$", target, re.M), (
        f"execution config workers: 1 is missing from the generated target block, so "
        f"reactor-c falls back to one worker per host core:\n{target}")


def test_unset_workers_leaves_the_reactor_c_default():
    target = _target_block(_emit({}))
    assert "workers" not in target, f"workers emitted without being configured:\n{target}"


def test_workers_is_defaults_only():
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    raw = {"defaults": {"max_publishes_per_invocation": 1},
           "nodes": {"sub": {"workers": 2}}}
    with pytest.raises(ValueError, match="workers"):
        ExecConfig.from_dict(raw, graph, source="t")


# ── the analyzed launch file's rcllf_workers argument ─────────────────

def _emit_with_launch_arg(value, defaults: dict | None = None) -> str:
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    graph.launch_graph.root.arguments.append(LaunchArgument(name="rcllf_workers",
                                                            default_value=value))
    raw = {"defaults": {"max_publishes_per_invocation": 1, **(defaults or {})}}
    bounds = resolve_corr_bounds(ExecConfig.from_dict(raw, graph, source="t"), graph)
    return emit_lf_system(generate_lf_system(graph, corr_bounds=bounds))


def test_launch_argument_reaches_the_lf_target_block():
    target = _target_block(_emit_with_launch_arg("4"))
    assert re.search(r"^\s*workers: 4,?$", target, re.M), (
        f"the launch file's rcllf_workers default 4 is missing from the generated target "
        f"block, so reactor-c falls back to one worker per host core:\n{target}")


def test_launch_argument_only_counts_in_the_root_launch_file():
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    child = LaunchFileInfo(file_path="/fake/child.launch.py", package="test_pkg",
                           arguments=[LaunchArgument(name="rcllf_workers", default_value="4")])
    graph.launch_graph.root.includes.append(child)
    target = _target_block(emit_lf_system(generate_lf_system(graph)))
    assert "workers" not in target, f"an included file's rcllf_workers was applied:\n{target}"


@pytest.mark.parametrize("bad", ["0", "-1", "four", "1.5", "", None])
def test_launch_argument_must_be_a_positive_integer(bad):
    with pytest.raises(ValueError, match="rcllf_workers"):
        _emit_with_launch_arg(bad)


def test_launch_argument_and_config_agreeing_is_fine_disagreeing_refused():
    target = _target_block(_emit_with_launch_arg("4", {"workers": 4}))
    assert re.search(r"^\s*workers: 4,?$", target, re.M), target
    with pytest.raises(ValueError, match="workers: 2.*rcllf_workers' 4"):
        _emit_with_launch_arg("4", {"workers": 2})
