# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Unit tests for hierarchical._resolve_source — no Docker/launch_ros required."""

from __future__ import annotations

from colcon_lf.node_analysis.models import CMakeInfo, LibraryTarget


def _cmake(executables, libraries=None):
    return CMakeInfo(
        package_name="test_pkg",
        executables=executables,
        libraries={
            name: LibraryTarget(
                name=name,
                type=lib.get("type", "SHARED"),
                sources=lib.get("sources", []),
                component_plugins=lib.get("plugins", []),
            )
            for name, lib in (libraries or {}).items()
        },
    )


class TestResolveSource:
    """Regression: rclcpp_components_register_node EXECUTABLE stores the plugin
    class name, not a source path.  _resolve_source must follow through to the
    component-library lookup instead of returning the class name as a path."""

    def test_register_node_executable_resolves_to_cpp(self):
        from colcon_lf.graph.hierarchical import _resolve_source

        info = _cmake(
            executables={"front_lidar_driver": "FrontLidarDriver"},
            libraries={
                "autoware_reference_nodes": {
                    "sources": ["src/front_lidar_driver.cpp"],
                    "plugins": ["FrontLidarDriver"],
                }
            },
        )
        result = _resolve_source(info, "front_lidar_driver")
        assert result == "src/front_lidar_driver.cpp", f"got {result!r}"

    def test_register_node_executable_not_returned_as_class_name(self):
        from colcon_lf.graph.hierarchical import _resolve_source

        info = _cmake(
            executables={"front_lidar_driver": "FrontLidarDriver"},
            libraries={
                "autoware_reference_nodes": {
                    "sources": ["src/front_lidar_driver.cpp"],
                    "plugins": ["FrontLidarDriver"],
                }
            },
        )
        result = _resolve_source(info, "front_lidar_driver")
        # Must never return the plugin class name as a "path"
        assert result != "FrontLidarDriver", "returned plugin class name instead of source path"

    def test_multi_node_library_resolves_each_correctly(self):
        from colcon_lf.graph.hierarchical import _resolve_source

        info = _cmake(
            executables={
                "front_lidar_driver": "FrontLidarDriver",
                "behavior_planner": "BehaviorPlanner",
                "ndtlocalizer": "NDTLocalizer",
            },
            libraries={
                "autoware_reference_nodes": {
                    "sources": [
                        "src/front_lidar_driver.cpp",
                        "src/behavior_planner.cpp",
                        "src/ndtlocalizer.cpp",
                    ],
                    "plugins": ["FrontLidarDriver", "BehaviorPlanner", "NDTLocalizer"],
                }
            },
        )
        assert _resolve_source(info, "front_lidar_driver") == "src/front_lidar_driver.cpp"
        assert _resolve_source(info, "behavior_planner") == "src/behavior_planner.cpp"
        # "NDTLocalizer" snake-cases to "ndtlocalizer" which matches "ndtlocalizer.cpp"
        assert _resolve_source(info, "ndtlocalizer") == "src/ndtlocalizer.cpp"

    def test_add_executable_source_path_still_works(self):
        from colcon_lf.graph.hierarchical import _resolve_source

        info = _cmake(executables={"my_node": "src/my_node.cpp"})
        assert _resolve_source(info, "my_node") == "src/my_node.cpp"

    def test_unknown_executable_returns_none(self):
        from colcon_lf.graph.hierarchical import _resolve_source

        info = _cmake(executables={})
        assert _resolve_source(info, "nonexistent") is None
