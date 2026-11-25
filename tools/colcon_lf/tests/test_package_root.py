# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The fallback include path finds a package's include/ at any source depth.

``_package_root`` walked five ancestors, so a source six directories below its
package root lost the package's own ``include/`` from the fallback flags (no
compile_commands.json), and every header there became an unresolved include.
"""

from __future__ import annotations

from pathlib import Path

from colcon_lf.node_analysis.clang_parser import _load_include_flags, _package_root


def _deep_package(tmp_path: Path) -> tuple[Path, Path, Path]:
    ws = tmp_path / "ws"
    pkg = ws / "src" / "deep_pkg"
    (pkg / "include" / "deep_pkg").mkdir(parents=True)
    (pkg / "package.xml").write_text("<package format=\"3\"><name>deep_pkg</name></package>\n")
    cpp = pkg / "src" / "a" / "b" / "c" / "d" / "e" / "deep_node.cpp"
    cpp.parent.mkdir(parents=True)
    cpp.write_text("int main() { return 0; }\n")
    return ws, pkg, cpp


def test_fallback_flags_hold_package_include_for_deep_source(tmp_path):
    ws, pkg, cpp = _deep_package(tmp_path)

    flags = _load_include_flags(cpp, ws)

    assert f"-I{pkg / 'include'}" in flags, (
        f"fallback include flags miss {pkg / 'include'} for {cpp}, 6 "
        f"directories below its package root: {flags}"
    )


def test_package_root_walk_stops_at_workspace(tmp_path):
    """A package.xml above the workspace is never taken for the source's package."""
    (tmp_path / "package.xml").write_text("<package format=\"3\"><name>outer</name></package>\n")
    ws = tmp_path / "ws"
    cpp = ws / "src" / "loose" / "node.cpp"
    cpp.parent.mkdir(parents=True)
    cpp.write_text("")

    assert _package_root(cpp, ws) is None
    assert _package_root(cpp) == tmp_path
