# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Guards for tests that read dev-only packages (release export).

The exported public tree does not ship ``tools/lf_bench``: its recorded fixtures,
corpora and Python package stay in the development repository. A test that reads
them is skipped there, keyed on that package being absent, and runs in the
development tree, where a missing fixture still fails it.

The v0.1.0 export also leaves out the recorded semantic-trace goldens whose run
manifests carry the recording host's name (a release decision). A test
that reads one of them skips where the golden is absent and runs in the
development tree; the goldens return to the release once re-recorded.
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


#: The skip reason of every test that needs a recorded golden the export leaves out.
GOLDEN_NOT_SHIPPED = "recorded golden not shipped in this release"


def golden_absent_reason(path: Path) -> str:
    return (f"{GOLDEN_NOT_SHIPPED}: {path.relative_to(REPO)} stays in the development "
            "repository until it is re-recorded")


def requires_golden(path: Path):
    """Skip marker for a test that reads the recorded golden ``path``."""
    return pytest.mark.skipif(not path.exists(), reason=golden_absent_reason(path))
