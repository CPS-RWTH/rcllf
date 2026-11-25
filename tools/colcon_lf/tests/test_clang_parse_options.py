# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""parse_node skips function bodies only inside the preamble."""

from __future__ import annotations

import pytest

cindex = pytest.importorskip("clang.cindex", reason="libclang not available")

from colcon_lf.node_analysis import clang_parser as cp  # noqa: E402

TU = cindex.TranslationUnit
SKIP = TU.PARSE_SKIP_FUNCTION_BODIES | cp._CXTU_LIMIT_SKIP_FUNCTION_BODIES_TO_PREAMBLE


def test_default_skips_bodies_only_in_the_preamble(monkeypatch):
    monkeypatch.delenv("RCLLF_CLANG_FULL_PARSE", raising=False)
    opts = cp._parse_options(cindex)
    assert opts & TU.PARSE_DETAILED_PROCESSING_RECORD
    assert opts & SKIP == SKIP, "header bodies must be skipped by default"
    assert opts & TU.PARSE_PRECOMPILED_PREAMBLE


def test_full_parse_escape_hatch(monkeypatch):
    monkeypatch.setenv("RCLLF_CLANG_FULL_PARSE", "1")
    opts = cp._parse_options(cindex)
    assert opts == TU.PARSE_DETAILED_PROCESSING_RECORD
