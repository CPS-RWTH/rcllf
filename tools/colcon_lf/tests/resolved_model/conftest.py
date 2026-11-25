# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Shared launch analysis for the resolved-model tests.

test_resolve.py and test_gate.py analyze the same launch files (sentry_pipeline
alone takes about 100 s of libclang work on a CI runner). The graph of each launch
is built once per session and each module still writes it to its own tmp
directory, so no test sees another test's staged files. tests/conftest.py puts
both modules in one xdist group, so the cache is shared on one worker.
"""

from __future__ import annotations

import pytest


@pytest.fixture(scope="session")
def analyzed_graph(packages_root):
    """Return ``build(launch_path)``, the SystemGraph of *launch_path*, built once."""
    from colcon_lf.graph.hierarchical import build_system_graph

    cache = {}

    def build(launch_path):
        if launch_path not in cache:
            cache[launch_path] = build_system_graph(launch_path, workspace_root=packages_root)
        return cache[launch_path]

    return build
