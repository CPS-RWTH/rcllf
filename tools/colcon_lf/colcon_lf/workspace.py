# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Workspace utilities shared across colcon_lf modules."""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree


def find_package_dirs(src_root: Path) -> dict[str, Path]:
    """Find all ROS2 packages under *src_root* by locating ``package.xml``.

    Returns ``{package_name: package_dir}``.  The name is the manifest's
    ``<name>``, as colcon and ``ros2 launch`` see it, so a package whose
    directory is named differently (ros2/examples keeps
    ``examples_rclcpp_minimal_publisher`` in ``minimal_publisher/``) is
    found by its package name.  An empty or unreadable manifest falls back
    to the directory name.  Handles nested mono-repos like Autoware
    (``src/launcher/autoware_launch/autoware_launch/``) and Nav2
    (``src/navigation2/nav2_controller/``).
    """
    packages: dict[str, Path] = {}
    for pkg_xml in sorted(src_root.rglob("package.xml")):
        pkg_dir = pkg_xml.parent
        rel = pkg_dir.relative_to(src_root)
        if any(part in ("build", "install", "log") for part in rel.parts):
            continue
        packages[manifest_package_name(pkg_xml) or pkg_dir.name] = pkg_dir
    return packages


def manifest_package_name(pkg_xml: Path) -> str | None:
    """Return the ``<name>`` of a ``package.xml``, or None if it has none."""
    try:
        name = ElementTree.parse(pkg_xml).getroot().find("name")
    except (ElementTree.ParseError, OSError):
        return None
    text = (name.text or "").strip() if name is not None else ""
    return text or None
