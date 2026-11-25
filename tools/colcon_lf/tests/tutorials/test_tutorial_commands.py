# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""CI-exercise the tutorial command sequences.

These tests pin the exact command sequences documented in
``docs/tutorials/porting-guide.md`` ("Verify it worked" blocks, one per
system part: timers, publishers/subscriptions, components, launch files)
and ``docs/tutorials/bring-your-own-code.md`` (Stage 1 — Analyze). **If a
test here fails after a CLI change, update BOTH the affected docs page(s)
and this file** — the two must never drift apart.

Unlike ``tests/playground/test_playground_cases.py`` (Phase 7), which calls
``colcon_lf.verb.analyze._analyze_one`` / ``colcon_lf.verb.generate.
_generate_one`` directly as Python functions, this suite shells the real
``colcon lf-analyze`` / ``colcon lf-configure`` / ``colcon lf-generate``
commands via ``subprocess`` from the ``packages/`` workspace root — the
same commands a tutorial reader would type, run through the actual
colcon verb/argparse layer. It reuses the Phase 7 suite's skip-on-bare-host
guard (needs ``/opt/ros/<distro>/include/rclcpp`` for libclang to resolve
node base classes) so it skips cleanly on a bare host and runs for real in
the dev container / CI image, picked up by the existing ``pytest tests/ -v``
step in ``.github/workflows/ci.yml`` (that step already ``cd``s into
``packages/`` with a built + sourced workspace before invoking pytest from
``tools/colcon_lf``, so ``tests/tutorials/`` is exercised with the same
environment the playground suite gets).

Deliberately NOT tested here (mirrors the roadmap's own "where feasible"
qualifier and Phase 7's precedent of deferring runtime-in-CI for the
soundness checker): ``colcon lf-build``, ``ros2 lf launch`` — both need the
``lfc``/reactor-c toolchain built, too heavy for per-commit CI; and
``lf-eval``/``lf-bench`` — the benchmark suite has its own, heavier CI story
(see ``docs/benchmark/index.md``). The "Run"/"Evaluate" stages remain
documented-but-not-CI-exercised, exactly as
``docs/tutorials/bring-your-own-code.md``'s own "What's actually verified
here" section discloses.

Note on ``lf-configure``/``lf-generate`` scoping: as both verbs
accept **either** ``--system-json <path>`` (one exact analysis) **or**
``--package <name>`` (every launch-stem analysis of that package) — mutually
exclusive. Historically neither had ``--package``, and an early draft of the
porting guide used the nonexistent flag; this suite caught it and the docs were
fixed the same session (Phase 8). Both spellings are exercised below so the
suite keeps pinning whichever one the docs pages show.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("colcon_core.verb", reason="colcon_core not available")

from ..ros_env import requires_colcon_lf_verbs, requires_ros_headers  # noqa: E402

# These tests shell the real colcon lf-analyze/lf-configure/lf-generate CLI on the
# packages/ workspace: they need the verbs registered, rclcpp headers for libclang
# and the built workspace's message headers.
pytestmark = [
    requires_ros_headers("the tutorial command tests"),
    requires_colcon_lf_verbs("the tutorial command tests"),
]

# Repo root, computed robustly from this file's location:
# tools/colcon_lf/tests/tutorials/test_tutorial_commands.py -> repo root is
# four parents up (tutorials -> tests -> colcon_lf -> tools -> <root>).
_REPO_ROOT = Path(__file__).resolve().parents[4]
_PACKAGES_ROOT = _REPO_ROOT / "packages"

# Generous but bounded per-command timeout: analyze needs a real libclang
# parse of rclcpp headers, which is slow but should never hang. This is a hang
# guard, not a performance check. CI runs the suite on 2 xdist workers on a
# 2-CPU runner; there `colcon lf-analyze --package system_bringup`
# took 173 s on one run and more than 300 s on another while the other worker
# parsed C++ with libclang, so the old 300 s bound failed a healthy run.
_TIMEOUT_S = 900


def _run(args: list[str], cwd: Path = _PACKAGES_ROOT) -> subprocess.CompletedProcess:
    """Shell a documented tutorial command from *cwd* (default: packages/ workspace root)."""
    return subprocess.run(
        args,
        cwd=cwd,
        capture_output=True,
        text=True,
        timeout=_TIMEOUT_S,
    )


def _assert_ok(result: subprocess.CompletedProcess, what: str) -> None:
    assert result.returncode == 0, (
        f"{what} failed (rc={result.returncode})\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )


class TestPortingGuideVerifyItWorked:
    """Shell the exact "Verify it worked" command blocks from porting-guide.md."""

    def test_timers(self):
        """docs/tutorials/porting-guide.md#timers "Verify it worked"."""
        analyze = _run(
            [
                "colcon",
                "lf-analyze",
                "--package",
                "timer_example",
                "--launch-file",
                "timer_node.launch.py",
                "--format",
                "json",
            ]
        )
        _assert_ok(analyze, "colcon lf-analyze --package timer_example")

        system_json = _PACKAGES_ROOT / "lf-analysis" / "timer_example" / "timer_node" / "system.json"
        assert system_json.is_file(), f"expected {system_json} to exist after analyze"

        data = json.loads(system_json.read_text())
        nodes = data.get("nodes", {})
        assert nodes, "timer_example system.json has an empty node set"

        timers = [t for node in nodes.values() for t in node.get("timers", [])]
        assert timers, f"timer_example system.json has no timer entries: {data}"
        assert any(t.get("period_ms") is not None for t in timers), (
            f"timer_example expects a timer with non-null period_ms (literal "
            f"1000ms chrono duration), got: {timers}"
        )

        configure = _run(
            [
                "colcon",
                "lf-configure",
                "--system-json",
                "lf-analysis/timer_example/timer_node/system.json",
            ]
        )
        _assert_ok(configure, "colcon lf-configure --system-json (timer_example)")

        generate = _run(
            [
                "colcon",
                "lf-generate",
                "--target-lfc",
                "--system-json",
                "lf-analysis/timer_example/timer_node/system.json",
            ]
        )
        _assert_ok(generate, "colcon lf-generate --target-lfc (timer_example)")

    def test_pubsub(self):
        """docs/tutorials/porting-guide.md#publishers-subscriptions "Verify it worked"."""
        analyze = _run(
            [
                "colcon",
                "lf-analyze",
                "--package",
                "pubsub_example",
                "--launch-file",
                "bringup.launch.py",
                "--format",
                "json",
            ]
        )
        _assert_ok(analyze, "colcon lf-analyze --package pubsub_example")

        system_json = _PACKAGES_ROOT / "lf-analysis" / "pubsub_example" / "bringup" / "system.json"
        assert system_json.is_file(), f"expected {system_json} to exist after analyze"

        data = json.loads(system_json.read_text())
        nodes = data.get("nodes", {})
        assert nodes, "pubsub_example system.json has an empty node set"
        assert len(nodes) == 2, f"pubsub_example expects 2 nodes, got {len(nodes)}: {nodes}"

        topic_connections = data.get("communication", {}).get("topics", [])
        assert len(topic_connections) >= 1, (
            f"pubsub_example expects at least 1 topic connection, got: {data}"
        )

        configure_show = _run(
            [
                "colcon",
                "lf-configure",
                "--system-json",
                "lf-analysis/pubsub_example/bringup/system.json",
                "--show",
            ]
        )
        _assert_ok(configure_show, "colcon lf-configure --system-json --show (pubsub_example)")

        generate = _run(
            [
                "colcon",
                "lf-generate",
                "--target-lfc",
                "--system-json",
                "lf-analysis/pubsub_example/bringup/system.json",
            ]
        )
        _assert_ok(generate, "colcon lf-generate --target-lfc (pubsub_example)")

        # ros2lf/api/discovery.py must recognize this exact
        # --target-lfc output (source launch file under lf-generated/,
        # not yet colcon-built) as a launchable-once-built "lfc" layout —
        # this is the same package/stem docs/getting-started/first-lf-run.md
        # walks through, and pre-fix, discovery only checked the stock-mode
        # paths this sequence never writes. No colcon build here (too heavy
        # for this suite, per its own docstring); a discovery-API-level
        # check against the real generated tree substitutes for one.
        ros2lf_root = _REPO_ROOT / "tools" / "ros2lf"
        if str(ros2lf_root) not in sys.path:
            sys.path.insert(0, str(ros2lf_root))
        from ros2lf.api.discovery import find_system

        entry = find_system(_PACKAGES_ROOT, "pubsub_example", "bringup")
        assert entry is not None, "ros2lf discovery did not find pubsub_example/bringup"
        assert entry.layout == "lfc", (
            f"expected lfc layout after --target-lfc generate, got {entry.layout}"
        )
        assert entry.has_launch, "ros2lf discovery did not see the --target-lfc launch file"

    def test_components(self):
        """docs/tutorials/porting-guide.md#components "Verify it worked"."""
        analyze = _run(
            [
                "colcon",
                "lf-analyze",
                "--package",
                "component_example",
                "--launch-file",
                "component_node.launch.py",
                "--format",
                "json",
            ]
        )
        _assert_ok(analyze, "colcon lf-analyze --package component_example")

        system_json = (
            _PACKAGES_ROOT / "lf-analysis" / "component_example" / "component_node" / "system.json"
        )
        assert system_json.is_file(), f"expected {system_json} to exist after analyze"

        data = json.loads(system_json.read_text())
        nodes = data.get("nodes", {})
        assert nodes, "component_example system.json has an empty node set"

        configure = _run(
            [
                "colcon",
                "lf-configure",
                "--system-json",
                "lf-analysis/component_example/component_node/system.json",
            ]
        )
        _assert_ok(configure, "colcon lf-configure --system-json (component_example)")

        generate = _run(
            [
                "colcon",
                "lf-generate",
                "--target-lfc",
                "--system-json",
                "lf-analysis/component_example/component_node/system.json",
            ]
        )
        _assert_ok(generate, "colcon lf-generate --target-lfc (component_example)")

    def test_launch_files(self):
        """docs/tutorials/porting-guide.md#launch-files "Verify it worked".

        Stops short of ``ros2 lf list`` (needs the built lfc binary — deferred
        per this module's docstring); asserts through lf-generate only.
        """
        analyze = _run(
            [
                "colcon",
                "lf-analyze",
                "--package",
                "system_bringup",
                "--launch-file",
                "system_bringup.launch.py",
                "--format",
                "json",
            ]
        )
        _assert_ok(analyze, "colcon lf-analyze --package system_bringup")

        system_json = (
            _PACKAGES_ROOT / "lf-analysis" / "system_bringup" / "system_bringup" / "system.json"
        )
        assert system_json.is_file(), f"expected {system_json} to exist after analyze"

        data = json.loads(system_json.read_text())
        nodes = data.get("nodes", {})
        assert nodes, "system_bringup system.json has an empty node set"

        configure_show = _run(
            [
                "colcon",
                "lf-configure",
                "--system-json",
                "lf-analysis/system_bringup/system_bringup/system.json",
                "--show",
            ]
        )
        _assert_ok(configure_show, "colcon lf-configure --system-json --show (system_bringup)")

        generate = _run(
            [
                "colcon",
                "lf-generate",
                "--target-lfc",
                "--system-json",
                "lf-analysis/system_bringup/system_bringup/system.json",
            ]
        )
        _assert_ok(generate, "colcon lf-generate --target-lfc (system_bringup)")


def test_bring_your_own_code_stage1_analyze_no_format_flag():
    """docs/tutorials/bring-your-own-code.md Stage 1 command, without --format.

    The bring-your-own-code guide's Stage 1 example
    (``colcon lf-analyze --package pubsub_example --launch-file
    bringup.launch.py``) omits ``--format`` entirely (default: ``all``,
    per the verb's argparse default) — pinned separately from the
    porting-guide's ``--format json`` invocation above. Re-analyzing the
    same package/launch-file pair here is intentionally idempotent.
    """
    result = _run(
        [
            "colcon",
            "lf-analyze",
            "--package",
            "pubsub_example",
            "--launch-file",
            "bringup.launch.py",
        ]
    )
    _assert_ok(result, "colcon lf-analyze --package pubsub_example (no --format)")


class TestPackageScoping:
    """`--package` on lf-configure / lf-generate.

    These are the commands the tutorials' early drafts assumed existed, now
    real. Each runs the same pipeline stage the ``--system-json`` cases above
    run, so a regression in the resolver shows up as a failing tutorial
    command, not just a failing unit test. ``lf-analyze --package
    pubsub_example`` has already run in this module (both classes above), so
    ``lf-analysis/pubsub_example/`` exists by the time these execute; each
    test re-analyzes anyway to be order-independent.
    """

    @staticmethod
    def _ensure_analyzed() -> None:
        _assert_ok(
            _run(
                [
                    "colcon",
                    "lf-analyze",
                    "--package",
                    "pubsub_example",
                    "--launch-file",
                    "bringup.launch.py",
                    "--format",
                    "json",
                ]
            ),
            "colcon lf-analyze --package pubsub_example (setup)",
        )

    def test_configure_package(self):
        self._ensure_analyzed()

        result = _run(["colcon", "lf-configure", "--package", "pubsub_example"])
        _assert_ok(result, "colcon lf-configure --package pubsub_example")
        # The fan-out notice must name the package and the launch stem it matched.
        assert "pubsub_example" in result.stdout, result.stdout
        assert "bringup" in result.stdout, result.stdout

    def test_generate_package(self):
        self._ensure_analyzed()

        result = _run(
            [
                "colcon",
                "lf-generate",
                "--target-lfc",
                "--package",
                "pubsub_example",
            ]
        )
        _assert_ok(result, "colcon lf-generate --target-lfc --package pubsub_example")
        assert "pubsub_example" in result.stdout, result.stdout

    def test_unknown_package_fails_with_actionable_message(self):
        """A typo'd package must fail loudly, never silently fan out to all analyses."""
        result = _run(
            ["colcon", "lf-configure", "--package", "no_such_package_xyz"]
        )
        assert result.returncode != 0, (
            f"expected failure for unknown package\nstdout:\n{result.stdout}"
        )
        combined = result.stdout + result.stderr
        assert "no_such_package_xyz" in combined, combined
        assert "lf-analyze" in combined, combined

    def test_package_and_system_json_are_mutually_exclusive(self):
        result = _run(
            [
                "colcon",
                "lf-generate",
                "--package",
                "pubsub_example",
                "--system-json",
                "lf-analysis/pubsub_example/bringup/system.json",
            ]
        )
        assert result.returncode != 0, (
            f"expected argparse rejection\nstdout:\n{result.stdout}"
        )
        assert "not allowed with" in (result.stdout + result.stderr).lower(), (
            result.stdout + result.stderr
        )
