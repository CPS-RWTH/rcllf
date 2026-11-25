# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Workspace-root resolution shared by every ``colcon lf-*`` verb."""

from __future__ import annotations

from pathlib import Path


def resolve_workspace(workspace_arg: str | None) -> Path | None:
    """Resolve the colcon workspace root.

    An explicit ``--workspace`` must be an existing directory. Without one, the
    first of ``./packages``, ``.`` and ``..`` that contains ``src/`` wins.
    """
    if workspace_arg is not None:
        p = Path(workspace_arg)
        if p.is_dir():
            return p
        return None

    cwd = Path.cwd()
    for candidate in [cwd / "packages", cwd, cwd.parent]:
        if (candidate / "src").is_dir():
            return candidate
    return None
