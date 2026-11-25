# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Read a user package's compile-time dependencies from its package.xml.

Shared by the lfc package emitters.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from colcon_lf.workspace import find_package_dirs


_SKIP_DEPS = frozenset({
    "ament_cmake", "ament_lint_auto", "ament_lint_common",
    "ament_cmake_auto", "ament_cmake_gtest", "ament_cmake_pytest",
    "rosidl_default_generators", "rosidl_default_runtime",
    "launch", "launch_ros", "launch_testing", "launch_testing_ament_cmake",
    "launch_testing_ros", "ros2launch", "python3-pytest",
    "builtin_interfaces",
})


def _read_package_deps(src_root: Path, package_name: str) -> list[str]:
    """Read <depend> and <build_depend> entries from a package's package.xml.

    The package is found by its ``<name>``, not its directory name.  Only
    returns C++ library dependencies (filters out Python/build tools).
    """
    pkg_dir = find_package_dirs(src_root).get(package_name)
    if pkg_dir is None:
        return []
    deps = []
    try:
        root = ET.parse(pkg_dir / "package.xml").getroot()
    except (ET.ParseError, OSError):
        return []
    # Only <depend> and <build_depend>: these are needed at compile time.
    for tag in ("depend", "build_depend"):
        for elem in root.findall(tag):
            dep = elem.text.strip() if elem.text else ""
            if dep and dep not in _SKIP_DEPS:
                deps.append(dep)
    return list(dict.fromkeys(deps))
