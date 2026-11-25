# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Guards for tests that read dev-only packages (release export).

The exported public tree does not ship ``tools/lf_bench``: its recorded fixtures,
corpora and Python package stay in the development repository. A test that reads
them is skipped there, keyed on that package being absent, and runs in the
development tree, where a missing fixture still fails it.
"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
LF_BENCH = REPO / "tools" / "lf_bench"


def requires_lf_bench(what: str):
    """Skip marker for a test that reads ``what`` from tools/lf_bench."""
    return pytest.mark.skipif(
        not LF_BENCH.is_dir(),
        reason=f"needs tools/lf_bench ({what}), a dev-only package the exported tree does not ship")
