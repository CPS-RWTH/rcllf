# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Chain resolution — which node sequence a latency metric measures.

A *chain* is an ordered list of nodes data flows through. Resolution order:

1. an explicit ``--chain A->B->C`` CLI spec;
2. otherwise the longest path through ``system.json``'s topic DAG
   (:meth:`~lf_eval.model.system.SystemModel.longest_chain`).

Each consecutive pair becomes a *hop*. A hop is flagged ``gated`` when its
upstream node is timer-gated (owns a timer and has subscriptions, so it
re-times data on its own tick rather than forwarding immediately). Gated hops
are excluded from the causal-ordering check and reported separately in latency
plots, because their latency is dominated by the timer period, not by
scheduling — this generalises the predecessor script's hardcoded
``TIMER_GATED_NODES = {"BehaviorPlanner"}`` and its hardcoded
``BehaviorPlanner->MPCController`` plot exclusion.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from lf_eval.model.system import SystemModel


@dataclass(frozen=True)
class Hop:
    """One upstream -> downstream step of a chain.

    ``upstream``/``downstream`` are **canonical ids** (see
    :attr:`~lf_eval.model.system.NodeSpec.canonical_id`) so they key directly
    into a :class:`~lf_eval.model.timeline.Timeline`. Human-facing text uses
    :attr:`display_upstream` / :attr:`display_downstream`.
    """

    upstream: str
    downstream: str
    gated: bool = False
    display_upstream: str = ""
    display_downstream: str = ""

    def __post_init__(self) -> None:
        # Default the display names to the canonical ids.
        if not self.display_upstream:
            object.__setattr__(self, "display_upstream", self.upstream)
        if not self.display_downstream:
            object.__setattr__(self, "display_downstream", self.downstream)

    @property
    def label(self) -> str:
        """Human-facing hop label, e.g. ``FrontLidarDriver->NDTLocalizer``."""
        return f"{self.display_upstream}->{self.display_downstream}"


@dataclass(frozen=True)
class Chain:
    """A resolved node sequence with its hops.

    ``nodes`` holds canonical ids (timeline keys); ``display_nodes`` holds the
    labels to show users.
    """

    nodes: tuple[str, ...]
    hops: tuple[Hop, ...]
    name: str = "chain"
    source: str = "derived"  # "derived" | "spec"
    display_nodes: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.display_nodes:
            object.__setattr__(self, "display_nodes", self.nodes)

    @property
    def label(self) -> str:
        if not self.display_nodes:
            return self.name
        return f"{self.display_nodes[0]}->{self.display_nodes[-1]}"

    @property
    def ungated_hops(self) -> tuple[Hop, ...]:
        return tuple(h for h in self.hops if not h.gated)


def build_chain(
    nodes: list[str],
    system: Optional[SystemModel] = None,
    name: str = "chain",
    source: str = "derived",
) -> Chain:
    """Turn a sequence of canonical ids into a :class:`Chain`, marking gated hops."""
    gated = system.timer_gated_nodes() if system is not None else set()

    def display(node: str) -> str:
        return system.display_of(node) if system is not None else node

    displays = [display(node) for node in nodes]
    hops = tuple(
        Hop(
            upstream=nodes[i],
            downstream=nodes[i + 1],
            gated=nodes[i] in gated,
            display_upstream=displays[i],
            display_downstream=displays[i + 1],
        )
        for i in range(len(nodes) - 1)
    )
    return Chain(
        nodes=tuple(nodes),
        hops=hops,
        name=name,
        source=source,
        display_nodes=tuple(displays),
    )


def resolve_chains(
    system: Optional[SystemModel],
    specs: Optional[list[str]] = None,
) -> list[Chain]:
    """Resolve the chains to analyse.

    :param system: topology; required to derive a chain when no spec is given.
    :param specs: explicit ``A->B->C`` chain specs (repeatable ``--chain``).
    :returns: resolved chains; empty when nothing could be determined.
    """
    if specs:
        if system is None:
            raise ValueError("--chain requires --system system.json to resolve names")
        return [
            build_chain(
                system.chain_from_spec(spec),
                system=system,
                name=f"chain{i + 1}" if len(specs) > 1 else "chain",
                source="spec",
            )
            for i, spec in enumerate(specs)
        ]

    if system is None:
        return []
    nodes = system.longest_chain()
    if len(nodes) < 2:
        return []
    return [build_chain(nodes, system=system, name="primary", source="derived")]
