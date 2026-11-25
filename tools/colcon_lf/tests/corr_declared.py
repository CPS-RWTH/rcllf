# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Generation with a declared burst bound for every node.

Since W3 slice S2 a topic edge is admitted to correspondence only when the
burst bound ``k`` of its publisher is known. Until the analyzer proves it
(slice S3), tests that are about admitted edges declare ``k = 1`` for every
node, the exec-config ``max_publishes_per_invocation: 1``.
"""

from __future__ import annotations

from colcon_lf.exec_config.resolve import CorrBoundsConfig
from colcon_lf.generation.lf_generator import generate_lf_system


def declared(graph, k: int = 1, **kw) -> CorrBoundsConfig:
    """Bounds config declaring burst *k* for every node of *graph*."""
    return CorrBoundsConfig(burst={name: k for name in graph.nodes}, **kw)


def generate_declared(graph, **kw):
    """``generate_lf_system`` with ``declared(graph)`` unless *corr_bounds* is given."""
    kw.setdefault("corr_bounds", declared(graph))
    return generate_lf_system(graph, **kw)
