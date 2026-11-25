# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Shared pytest fixtures for colcon_lf tests."""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest


# Root of the repository — two levels above tools/colcon_lf/
_REPO_ROOT = Path(__file__).resolve().parents[3]
_PACKAGES_SRC = _REPO_ROOT / "packages" / "src"
_PACKAGES_ROOT = _REPO_ROOT / "packages"  # colcon workspace root (contains src/ + install/)
# Packages only the tests load (packages/src/fixtures/README.md): the public tree ships
# them as test fixtures, not as examples.
_PACKAGES_FIXTURES = _REPO_ROOT / "packages" / "src" / "fixtures"


def find_package(name: str) -> Path:
    """The source folder of package ``name``: packages/src/ or packages/src/fixtures/."""
    for root in (_PACKAGES_SRC, _PACKAGES_FIXTURES):
        if (root / name).is_dir():
            return root / name
    raise AssertionError(
        f"package {name!r} is in neither {_PACKAGES_SRC} nor {_PACKAGES_FIXTURES}")


@pytest.fixture(scope="session")
def packages_src() -> Path:
    """Path to packages/src/ containing all example ROS2 packages."""
    assert _PACKAGES_SRC.is_dir(), f"packages/src not found at {_PACKAGES_SRC}"
    return _PACKAGES_SRC


@pytest.fixture(scope="session")
def package_dir():
    """Return :func:`find_package`: a package's source folder by name."""
    return find_package


@pytest.fixture(scope="session")
def packages_root() -> Path:
    """Colcon workspace root (packages/ — contains src/ and install/)."""
    return _PACKAGES_ROOT


@pytest.fixture(scope="session")
def cmake(packages_src: Path):
    """Return a helper that parses a CMakeLists.txt by package name."""
    from colcon_lf.node_analysis.cmake_parser import parse

    def _parse(package_name: str):
        path = find_package(package_name) / "CMakeLists.txt"
        assert path.exists(), f"CMakeLists.txt not found: {path}"
        return parse(path)

    return _parse


@pytest.fixture(scope="session")
def launch_parser(packages_src: Path, packages_root: Path):
    """Return a helper that parses a launch file by package and filename.

    Usage::

        graph = launch_parser("timer_example", "timer_node.launch.py")

    Requires a built + sourced ROS2 workspace (Docker).
    """
    from colcon_lf.launch_analysis.parser import parse_launch

    def _parse(package_name: str, launch_file: str):
        path = find_package(package_name) / "launch" / launch_file
        assert path.exists(), f"Launch file not found: {path}"
        return parse_launch(path, workspace_root=packages_root)

    return _parse


# ---------------------------------------------------------------------------
# Parallel runs (pytest-xdist)
# ---------------------------------------------------------------------------
# CI runs ``pytest -n logical --dist loadgroup``. Every test gets an xdist group:
#
# * files that read or write the real workspace (``packages/lf-analysis``,
#   ``packages/lf-generated``, staging trees) share ONE group, so they run on a
#   single worker in collection order and never race on the same system.json;
# * ``runtime/`` tests launch real binaries on one ROS domain and share ONE group;
# * playground cases are grouped by their example launch, so cases that share one
#   launch see the same memoized analysis run (test_playground_cases.py);
# * the resolved-model files share one analysis cache and one group;
# * files of independent snippet parses (each test writes its own C++ into tmp_path
#   and parses it with libclang, about 20 s each) are grouped per test function;
# * every other test is grouped by its class (or module, for plain functions),
#   so a class whose fixture parses one C++ file with libclang parses it on one
#   worker only, which is what makes the clang tests scale.
_SHARED_WORKSPACE_FILES = frozenset({
    "exec_config/test_configure_verb.py",
    "exec_config/test_generate_verb_integration.py",
    "features/test_generate_verb_gate.py",
    "generation/test_noncomponent_lfc.py",
    "generation/test_per_instance_rids.py",
    "generation/test_source_staging_fail_closed.py",
    "test_topology_svg.py",
    "test_verb_analyze.py",
    "tutorials/test_tutorial_commands.py",
})
# Files whose expensive fixture is a session cache shared ACROSS classes: grouped by
# module so the cached launch analysis is built once, not once per worker.
_MODULE_GROUP_FILES = frozenset({
    "test_hierarchical_graph.py",
})
# Files that share one session cache ACROSS modules: one group, so the cache is
# built once on one worker (resolved_model/conftest.py).
_NAMED_GROUP_FILES = {
    "resolved_model/test_gate.py": "resolved-model-analysis",
    "resolved_model/test_resolve.py": "resolved-model-analysis",
}
# Files whose tests share no fixture or cache beyond the test itself:
# one group per test function (parametrized cases of one function stay together,
# they share the parse_node cache).
_PER_TEST_FILES = frozenset({
    "test_main_only_out_of_line.py",
    "test_nav2_wrapper_topic_clang.py",
    "test_publish_bounds_clang.py",
    "test_qos_hardening_clang.py",
    "test_thread_container_spawn_clang.py",
    "test_topic_argument_literal_clang.py",
    "test_unresolved_include.py",
})
_TESTS_DIR = Path(__file__).resolve().parent


def _xdist_group(item) -> str:
    rel = Path(str(item.fspath)).resolve().relative_to(_TESTS_DIR).as_posix()
    if rel in _SHARED_WORKSPACE_FILES:
        return "shared-workspace"
    if rel.startswith("runtime/"):
        # Real launches on one ROS domain: two at once cross-talk on the
        # same topics (measured: pubsub smoke heard a second
        # Bringup's samples while test_runtime_ingress ran in parallel).
        return "runtime"
    if rel in _MODULE_GROUP_FILES:
        return rel
    if rel in _NAMED_GROUP_FILES:
        return _NAMED_GROUP_FILES[rel]
    if rel == "playground/test_playground_cases.py":
        callspec = getattr(item, "callspec", None)
        if callspec is not None and "module" in callspec.params:
            return f"playground:{callspec.params['module'].playground.example}"
    if rel in _PER_TEST_FILES:
        return item.nodeid.split("[", 1)[0]
    return item.nodeid.split("[", 1)[0].rsplit("::", 1)[0]


# ---------------------------------------------------------------------------
# CI shards
# ---------------------------------------------------------------------------
# ``RCLLF_TEST_SHARD=k/N`` keeps only the xdist groups assigned to shard k of N, so
# CI runs the suite as N parallel jobs. Whole groups move, never single tests, so
# the grouping guarantees above hold inside each shard. Groups are assigned longest
# first to the least loaded shard, weighted by the measured seconds per test in
# ci_test_durations.json (tests not listed weigh _DEFAULT_TEST_SECONDS). The
# assignment depends only on the collected items, so the xdist controller and every
# worker deselect the same tests. The union of all shards is the full suite; CI
# checks that in the lint job.
_DURATIONS_FILE = _TESTS_DIR / "ci_test_durations.json"
_DEFAULT_TEST_SECONDS = 0.5


def _shard_spec() -> tuple[int, int] | None:
    spec = os.environ.get("RCLLF_TEST_SHARD", "").strip()
    if not spec:
        return None
    k, _, n = spec.partition("/")
    shard, count = int(k), int(n)
    if not 1 <= shard <= count:
        raise pytest.UsageError(f"RCLLF_TEST_SHARD={spec!r}: need 1 <= k <= N")
    return shard, count


def assign_shards(group_weights: dict[str, float], count: int) -> dict[str, int]:
    """Map each group to a shard index in 1..count, longest group first."""
    loads = [0.0] * count
    assignment = {}
    for group in sorted(group_weights, key=lambda g: (-group_weights[g], g)):
        target = min(range(count), key=lambda i: (loads[i], i))
        loads[target] += group_weights[group]
        assignment[group] = target + 1
    return assignment


def _select_shard(config, items, groups, shard: int, count: int) -> None:
    durations = json.loads(_DURATIONS_FILE.read_text()) if _DURATIONS_FILE.is_file() else {}
    weights: dict[str, float] = {}
    for item, group in zip(items, groups):
        weights[group] = weights.get(group, 0.0) + durations.get(item.nodeid, _DEFAULT_TEST_SECONDS)
    assignment = assign_shards(weights, count)
    keep, drop = [], []
    for item, group in zip(items, groups):
        (keep if assignment[group] == shard else drop).append(item)
    if drop:
        config.hook.pytest_deselected(items=drop)
    items[:] = keep


# tryfirst: xdist's own collection hook reads the xdist_group markers, so they must
# be attached before it runs.
@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config, items):
    groups = [_xdist_group(item) for item in items]
    spec = _shard_spec()
    if spec is not None:
        _select_shard(config, items, groups, *spec)
        groups = [_xdist_group(item) for item in items]
    if not config.pluginmanager.hasplugin("xdist"):
        return
    for item, group in zip(items, groups):
        item.add_marker(pytest.mark.xdist_group(group))
