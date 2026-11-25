# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the shipped limits source.

Covers: ids derived from the row order, every malformed row named by ``problems()``,
and that the shipped modules never need the evaluation docs: ``limits`` imports
nothing.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from colcon_lf.features import limits



def test_ids_follow_the_row_order():
    slugs = [slug for slug, _ in limits.LIMIT_ROWS]
    assert list(limits.limit_ids()) == slugs
    assert list(limits.limit_ids().values()) == [f"L{i}" for i in range(1, len(slugs) + 1)]
    assert list(limits.limits()) == list(limits.limit_ids().values())


def test_committed_rows_have_no_problem():
    assert limits.problems() == []


@pytest.mark.parametrize(
    "row, message",
    [
        (("dup", "X."), "limit slug 'dup' appears twice in LIMIT_ROWS (second at L2)"),
        (("blank", "  "), "limit 'blank' (L2) has an empty statement"),
        (("wrap", "a\nb"), "limit 'wrap' (L2) has a line break in its statement"),
        (("pipe", "a | b"), "limit 'pipe' (L2) has an unescaped '|' in its statement"),
    ],
)
def test_malformed_row_is_named(monkeypatch, row, message):
    first = ("dup", "X.") if row[0] == "dup" else ("first", "X.")
    monkeypatch.setattr(limits, "LIMIT_ROWS", (first, row))
    assert limits.problems() == [message]


def test_escaped_pipe_is_accepted(monkeypatch):
    monkeypatch.setattr(limits, "LIMIT_ROWS", (("pipe", r"a \| b"),))
    assert limits.problems() == []


def test_limits_imports_nothing():
    tree = ast.parse(Path(limits.__file__).read_text())
    imports = [
        node for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        and not (isinstance(node, ast.ImportFrom) and node.module == "__future__")
    ]
    assert imports == [], [ast.unparse(node) for node in imports]


