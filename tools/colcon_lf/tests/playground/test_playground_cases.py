# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Playground CI harness.

End-to-end assertion, per registered ``FeatureModule.playground`` case, that:

1. the case's example launch file still exists under ``packages/src/`` or
   ``packages/src/fixtures/`` (so a
   registry entry can never silently rot once its fixture package is
   renamed/removed);
2. running the **real** analyzer (``colcon_lf.verb.analyze._analyze_one`` —
   deliberately not mocking ``build_system_graph``, unlike
   ``test_analyze_verb_integration.py``, because the whole point of the
   playground is a true end-to-end run against libclang/ROS2 headers) against
   that launch file produces the ``Outcome`` the module declares; and
3. the generate-verb hazard gate (``colcon_lf.verb.generate._generate_one``)
   agrees: PASS/WARN proceed (rc 0), REJECT refuses (rc 1) absent any
   acknowledged hazards.

Needs a built + sourced ROS2 workspace and libclang with real rclcpp headers
(same environment gap as ``tests/test_clang_parser.py`` /
``tests/test_qos_hardening_clang.py``) — skips cleanly on a bare host, runs
for real in the dev container / CI (CI should not be the first environment to execute these; run them once in the
dev container before pushing).
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

pytest.importorskip("colcon_core.verb", reason="colcon_core not available")

from ..ros_env import requires_ros_headers  # noqa: E402

# The cases analyze real example packages, whose message headers exist only in a
# built workspace.
pytestmark = requires_ros_headers("the playground cases")

# Repo root, computed robustly from this file's location:
# tools/colcon_lf/tests/playground/test_playground_cases.py -> repo root is
# four parents up (playground -> tests -> colcon_lf -> tools -> <root>).
_REPO_ROOT = Path(__file__).resolve().parents[4]
# Examples live in packages/src/, test-only packages in packages/src/fixtures/.
_PACKAGE_ROOTS = (_REPO_ROOT / "packages" / "src", _REPO_ROOT / "packages" / "src" / "fixtures")
_PACKAGES_ROOT = _REPO_ROOT / "packages"


def _cases():
    """Yield (feature_id, FeatureModule) for every registered module with a playground case."""
    from colcon_lf.features.registry import REGISTRY

    return [
        (feature_id, module)
        for feature_id, module in sorted(REGISTRY.items())
        if module.playground is not None
    ]


def _launch_path_for(example: str) -> Path:
    """Resolve a ``PlaygroundCase.example`` ("<package>/<launch-stem>") to its launch file."""
    package, _, launch_stem = example.partition("/")
    root = next((r for r in _PACKAGE_ROOTS if (r / package).is_dir()), _PACKAGE_ROOTS[0])
    return root / package / "launch" / f"{launch_stem}.launch.py"


_CASES = _cases()
_CASE_IDS = [feature_id for feature_id, _ in _CASES]

# Module-level memoization keyed by example string ("<package>/<launch-stem>").
# Several modules can share one launch file (co-firing hazards on
# ``qos_hazard_example/bringup``); analyzing it once per case is wasted
# libclang work — and, more importantly, each case must observe the *same*
# end-to-end run other co-firing modules see, not an independently re-run one.
_ANALYZE_CACHE: dict[str, tuple[int, Path]] = {}


def _analyze_example(example: str, cache: dict, tmp_root: Path) -> tuple[int, Path]:
    """Run ``_analyze_one`` for *example*, memoized in *cache*. Returns (rc, system_json_path)."""
    if example in cache:
        return cache[example]

    from colcon_lf.verb.analyze import _analyze_one

    package, _, launch_stem = example.partition("/")
    launch_path = _launch_path_for(example)
    out_dir = tmp_root / example.replace("/", "__")
    rc = _analyze_one(
        package=package,
        launch_file=f"{launch_stem}.launch.py",
        launch_path=launch_path,
        workspace=_PACKAGES_ROOT,
        output_dir=str(out_dir),
        output_fmt="json",
    )
    system_json = out_dir / package / launch_stem / "system.json"
    result = (rc, system_json)
    cache[example] = result
    return result


class TestPlaygroundCases:
    """One parametrized test per registered feature module's playground case."""

    @pytest.mark.parametrize("feature_id,module", _CASES, ids=_CASE_IDS)
    def test_playground_case(self, feature_id, module, tmp_path):
        from colcon_lf.features.diagnostics import Outcome, Severity
        from colcon_lf.features.registry import severity_for

        case = module.playground
        launch_path = _launch_path_for(case.example)
        assert launch_path.is_file(), (
            f"playground example for '{feature_id}' points at a launch file "
            f"that no longer exists: {launch_path} (example={case.example!r}) "
            "— registry examples must never rot"
        )

        # --- Run the real analyzer end-to-end (no graph mocking). ---
        rc, system_json = _analyze_example(case.example, _ANALYZE_CACHE, tmp_path)
        assert system_json.is_file(), (
            f"analyze did not produce system.json for '{case.example}' "
            f"(rc={rc}); expected at {system_json}"
        )

        data = json.loads(system_json.read_text())
        observed_diags = data.get("diagnostics", [])
        observed_codes = {d["code"] for d in observed_diags}
        severities = {severity_for(code) for code in observed_codes}

        has_error = Severity.ERROR in severities
        has_warning = Severity.WARNING in severities

        if case.expected_outcome is Outcome.PASS:
            assert rc == 0, f"analyze failed (rc={rc}) for PASS case '{feature_id}'"
            assert not has_warning and not has_error, (
                f"PASS case '{feature_id}' observed WARNING/ERROR-severity "
                f"codes: {observed_codes}"
            )
        elif case.expected_outcome is Outcome.WARN:
            assert rc == 0, f"analyze failed (rc={rc}) for WARN case '{feature_id}'"
            assert has_warning, (
                f"WARN case '{feature_id}' observed no WARNING-severity code "
                f"among {observed_codes}"
            )
            assert not has_error, (
                f"WARN case '{feature_id}' observed ERROR-severity code(s) "
                f"among {observed_codes} — co-firing module escalated this "
                "case to REJECT"
            )
        elif case.expected_outcome is Outcome.REJECT:
            assert rc == 0, f"analyze itself failed (rc={rc}) for REJECT case '{feature_id}'"
            assert has_error, (
                f"REJECT case '{feature_id}' observed no ERROR-severity code "
                f"among {observed_codes}"
            )
        else:  # pragma: no cover - exhaustive Outcome enum guard
            pytest.fail(f"unhandled Outcome: {case.expected_outcome!r}")

        assert set(case.expected_codes) <= observed_codes, (
            f"'{feature_id}' expected_codes {case.expected_codes} not a subset "
            f"of observed codes {observed_codes}"
        )

        # --- Generate-verb hazard gate, same launch/system.json, no acknowledgments. ---
        from colcon_lf.verb.generate import _generate_one

        gate_rc = _generate_one(
            system_json=system_json,
            output_dir=None,
            after_delay="1 msec",
            generate_launch=False,
        )
        if case.expected_outcome in (Outcome.PASS, Outcome.WARN):
            assert gate_rc == 0, (
                f"generate hazard gate refused (rc={gate_rc}) for "
                f"{case.expected_outcome.value.upper()} case '{feature_id}'"
            )
        else:
            assert gate_rc == 1, (
                f"generate hazard gate proceeded (rc={gate_rc}) for REJECT "
                f"case '{feature_id}' with no acknowledged hazards"
            )


def test_registry_has_playground_cases_to_run():
    """Sanity: the parametrization above isn't silently empty (all skipped/collected-away)."""
    assert len(_CASES) > 0, "no registered feature module has a playground case"
