# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the generated features-table doc.

Covers: deterministic output, every registered feature_id present, the
generated-header marker, and ``--check`` mode detecting a stale committed
file.
"""

from __future__ import annotations

from pathlib import Path

from colcon_lf.features import docs_table, evidence, limits
from colcon_lf.features.registry import REGISTRY


class TestRenderDeterminism:
    def test_two_renders_are_identical(self):
        """No timestamps or other non-deterministic content should leak in."""
        assert docs_table.render() == docs_table.render()

    def test_render_contains_header_marker(self):
        assert docs_table.HEADER_MARKER in docs_table.render()

    def test_header_marker_is_first_line(self):
        assert docs_table.render().splitlines()[0] == docs_table.HEADER_MARKER


class TestMiddlewareStatement:
    """The page states the one RMW the binding admits."""

    def test_page_names_fast_dds_as_required(self):
        rendered = docs_table.render()
        assert "rcllf requires Fast DDS (`rmw_fastrtps_cpp`" in rendered, (
            "support matrix has no RMW requirement statement"
        )

    def test_statement_matches_the_binding_gate(self):
        repo = Path(__file__).resolve().parents[4]
        src = repo / "packages/lf-frame/ros2_lf_binding/src/corr_runtime.cpp"
        text = src.read_text()
        assert 'std::strcmp(rmw_id, "rmw_fastrtps_cpp") != 0' in text, (
            "binding startup gate no longer refuses non-Fast-DDS RMWs; update the statement"
        )


class TestRenderCoverage:
    def test_contains_every_registered_feature_title(self):
        rendered = docs_table.render()
        for module in REGISTRY.values():
            assert module.title in rendered, f"{module.feature_id} title missing from table"

    def test_row_count_matches_registry_size(self):
        lines = docs_table.render().splitlines()
        # one '|' row per feature module, under the two header rows; the
        # stated-limits table after the blank line is not counted.
        start = next(i for i, line in enumerate(lines) if line.startswith("| Feature |"))
        end = lines.index("", start)
        assert end - start - 2 == len(REGISTRY)

    def test_contains_every_registered_diagnostic_code(self):
        rendered = docs_table.render()
        for module in REGISTRY.values():
            for code in module.codes:
                assert code in rendered, f"{code} missing from table"


class TestCheckMode:
    def test_check_passes_when_file_matches(self, tmp_path: Path):
        out = tmp_path / "support-matrix.md"
        assert docs_table.main(["--out", str(out)]) == 0
        assert docs_table.main(["--out", str(out), "--check"]) == 0

    def test_check_fails_on_missing_file(self, tmp_path: Path):
        out = tmp_path / "does-not-exist.md"
        assert docs_table.main(["--out", str(out), "--check"]) == 1

    def test_check_detects_staleness(self, tmp_path: Path, capsys):
        out = tmp_path / "support-matrix.md"
        out.write_text("stale content that does not match the generated table\n")
        rc = docs_table.main(["--out", str(out), "--check"])
        assert rc == 1
        captured = capsys.readouterr()
        assert "stale" in captured.out

    def test_write_mode_overwrites_stale_file(self, tmp_path: Path):
        out = tmp_path / "support-matrix.md"
        out.write_text("old content\n")
        assert docs_table.main(["--out", str(out)]) == 0
        assert out.read_text() == docs_table.render()


class TestCommittedFile:
    def test_committed_support_matrix_is_current(self):
        """Guards against the committed docs/features/support-matrix.md drifting
        from the registry — the same check CI's drift step performs."""
        repo_root = Path(__file__).resolve().parents[4]
        committed = repo_root / "docs" / "features" / "support-matrix.md"
        assert committed.exists(), f"{committed} not found — generate it first"
        assert committed.read_text() == docs_table.render()


class TestEvidenceClasses:
    """The evidence column and the stated limits cannot drift."""

    def test_every_feature_has_an_evidence_class(self):
        assert evidence.problems(set(REGISTRY)) == []

    def test_check_names_a_feature_without_evidence_class(self, monkeypatch, tmp_path, capsys):
        out = tmp_path / "support-matrix.md"
        docs_table.main(["--out", str(out)])
        monkeypatch.delitem(evidence.EVIDENCE, "core.actions")
        assert docs_table.main(["--out", str(out), "--check"]) == 1
        assert "feature 'core.actions' is in REGISTRY but has no evidence class" in capsys.readouterr().out

    def test_check_names_a_limit_slug_not_in_the_limits(self, monkeypatch, tmp_path, capsys):
        out = tmp_path / "support-matrix.md"
        monkeypatch.setitem(
            evidence.EVIDENCE, "core.actions", (evidence.Evidence.EXPERIMENTAL, ("no-such-limit",))
        )
        assert docs_table.main(["--out", str(out), "--check"]) == 1
        assert (
            "feature 'core.actions' cites limit 'no-such-limit', which is not in "
            "colcon_lf/features/limits.py"
        ) in capsys.readouterr().out

    def test_check_names_a_duplicate_limit_slug(self, monkeypatch, tmp_path, capsys):
        # Two rows for one slug: LIMIT_IDS keeps the later id, so the first row's
        # statement would render with no feature able to cite it.
        out = tmp_path / "support-matrix.md"
        docs_table.main(["--out", str(out)])
        monkeypatch.setattr(limits, "LIMIT_ROWS", (*limits.LIMIT_ROWS, limits.LIMIT_ROWS[0]))
        assert docs_table.main(["--out", str(out), "--check"]) == 1
        slug = limits.LIMIT_ROWS[0][0]
        assert f"error: limit slug '{slug}' appears twice in LIMIT_ROWS" in capsys.readouterr().out

    def test_rendered_limits_are_the_shipped_rows(self):
        page = docs_table.render()
        for index, (_, text) in enumerate(limits.LIMIT_ROWS, start=1):
            assert f'| <a id="l{index}"></a>L{index} | {text} |' in page
