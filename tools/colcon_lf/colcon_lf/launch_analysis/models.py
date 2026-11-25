# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Data models for the launch file analysis layer."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class LaunchArgument:
    """A declared launch argument (DeclareLaunchArgument)."""

    name: str
    default_value: str | None = None
    description: str | None = None
    choices: list[str] | None = None


@dataclass
class LaunchNodeAction:
    """A single node action resolved from a launch file.

    Corresponds to a ``launch_ros.actions.Node`` or a
    ``launch_ros.descriptions.ComposableNode`` loaded into a container.
    """

    package: str
    executable: str                         # or plugin class name for components
    name: str | None = None                 # Node name override
    namespace: str | None = None            # Resolved namespace (may include pushed ns)
    parameters: dict[str, Any] = field(default_factory=dict)   # Inline params
    parameter_files: list[Path] = field(default_factory=list)  # YAML config paths
    # ``parameters=[...]`` in launch order: ``{"file": "<abs path>"}``
    # or ``{"params": {...}}`` per entry. ``parameters`` and ``parameter_files``
    # above lose the interleaving and stay for their existing readers.
    parameter_sources: list[dict[str, Any]] = field(default_factory=list)
    remappings: list[tuple[str, str]] = field(default_factory=list)
    condition: str | None = None            # IfCondition/UnlessCondition expression
    # True when launch starts the node, None when a condition (its own or an
    # enclosing one) is undecidable; a node behind a false one is dropped.
    condition_result: bool | None = None
    is_component: bool = False              # Loaded via ComposableNodeContainer
    composable_container: str | None = None # Container node name if component
    environment: dict[str, str] = field(default_factory=dict)  # Env vars on this node
    output: str | None = None               # 'screen', 'log', 'both'
    respawn: bool = False                   # Respawn policy
    respawn_delay: float | None = None      # Seconds
    arguments: list[str] = field(default_factory=list)         # CLI args to executable
    # ``extra_arguments=[{'use_intra_process_comms': ...}]`` of a composable node:
    # True/False when launch sets it, None when it does not.
    use_intra_process_comms: bool | None = None


@dataclass
class LaunchProcessAction:
    """A non-ROS process launched via ExecuteProcess."""

    cmd: list[str]
    name: str | None = None
    cwd: str | None = None
    environment: dict[str, str] = field(default_factory=dict)
    condition: str | None = None
    output: str | None = None
    shell: bool = False


@dataclass
class LaunchGroup:
    """A GroupAction with an optional pushed namespace."""

    namespace: str | None
    nodes: list[LaunchNodeAction] = field(default_factory=list)
    includes: list[LaunchFileInfo] = field(default_factory=list)
    groups: list["LaunchGroup"] = field(default_factory=list)
    condition: str | None = None


@dataclass
class LaunchFileInfo:
    """Parsed content of a single launch file."""

    file_path: Path
    package: str                            # Owning package name
    arguments: list[LaunchArgument] = field(default_factory=list)
    nodes: list[LaunchNodeAction] = field(default_factory=list)
    includes: list[LaunchFileInfo] = field(default_factory=list)
    groups: list[LaunchGroup] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    set_parameters: list[tuple[str, Any]] = field(default_factory=list)
    set_environment: list[tuple[str, str]] = field(default_factory=list)
    set_configurations: list[tuple[str, str]] = field(default_factory=list)
    execute_processes: list[LaunchProcessAction] = field(default_factory=list)


@dataclass
class LaunchContextState:
    """Accumulated launch state from SetParameter, SetEnvironmentVariable, etc.

    Passed through the entity walk to track global state that affects
    OpaqueFunction execution and node configuration.
    """

    parameters: dict[str, Any] = field(default_factory=dict)
    environment: dict[str, str] = field(default_factory=dict)
    launch_configurations: dict[str, str] = field(default_factory=dict)


@dataclass
class LaunchGraph:
    """Fully resolved launch graph rooted at a single launch file."""

    root: LaunchFileInfo
    all_nodes: list[LaunchNodeAction] = field(default_factory=list)
    # Flattened list of every LaunchNodeAction with resolved namespaces
    context: LaunchContextState = field(default_factory=LaunchContextState)
    # Accumulated global state (parameters, env, configurations)
