# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Data models for the unified system graph."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.features.diagnostics import Diagnostic
from colcon_lf.launch_analysis.models import LaunchGraph
from colcon_lf.node_analysis.models import NodeInfo


SCHEMA_VERSION = 3
# v2 -> v3: added `diagnostics` (hazard-detection facts, features/ registry).
# Diagnostics persist code +
# facts only (no severity) — severity is resolved from the REGISTRY at read
# time.
# Additive within v3: `topic_dynamic` on publishers and subscribers (RCLLF-H004,
# Phase 7). It defaults to false, so readers must tolerate its absence.
# Additive within v3: `topic_param` on publishers and subscribers,
# default null.


@dataclass
class SystemGraph:
    """Unified model combining launch hierarchy, node internals, and communication."""

    name: str                                       # Derived from root launch file
    launch_graph: LaunchGraph
    nodes: dict[str, NodeInfo] = field(default_factory=dict)
                                                    # Keyed by fully qualified node name
    communication: SystemCommunication = field(default_factory=SystemCommunication)
    parameter_files: dict[str, dict[str, Any]] = field(default_factory=dict)
                                                    # Filename -> parsed YAML content
    schema_version: int = SCHEMA_VERSION            # Bumped when the schema changes
    diagnostics: list[Diagnostic] = field(default_factory=list)
                                                    # Hazard-detection facts (schema v3+);
                                                    # missing field (not just empty) means
                                                    # "analysis predates hazard detection".
