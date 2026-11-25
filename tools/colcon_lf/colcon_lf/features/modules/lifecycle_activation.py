# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""RCLLF-H005 — a lifecycle hook override that does not chain to the base.

`rclcpp_lifecycle::LifecycleNode::on_activate` is what activates
every managed publisher, via `impl_->on_activate()`
(`rclcpp_lifecycle/src/lifecycle_node.cpp`). A user override that returns
`SUCCESS` without calling it leaves every `LifecyclePublisher` deactivated for
the life of the process, so `publish()` silently drops. `on_deactivate` is the
mirror.

**Why this needs a diagnostic rather than a code review.** The runtime failure
is nearly invisible. `LifecyclePublisher::log_publisher_not_enabled` latches
`should_log_ = false` after emitting one warning, so a node that drops 100% of
its messages logs a single line at its first publish and then looks completely
healthy. Worse, the idiomatic node logs `"Publishing: ..."` immediately *before*
calling `publish()`, so the log positively suggests success. This shipped in
this repo's own flagship `lifecycle_example` and went unnoticed until a
byte-level A/B on a second example surfaced it.

It is also invisible to the rest of the toolchain. The extracted model records
a publisher on a topic and the generator wires an LF connection for it, so the
model claims an edge that carries nothing at runtime. That is the same class of
lie RCLLF-H004 exists for, one layer down.

**Scope of the check.** Fires only when all of these hold, which keeps it
quiet on code where not chaining is harmless:

1. the node has a managed lifecycle (`NodeInfo.is_lifecycle`);
2. it owns at least one lifecycle publisher, since a node with none loses
   nothing by skipping the base call;
3. it overrides `on_activate` or `on_deactivate`, the only two hooks whose base
   implementation does work (`models.LIFECYCLE_CHAINED_HOOKS`); and
4. that override contains no qualified call to the same hook name, as detected
   by `clang_parser._hook_chains_to_base`.

WARNING rather than ERROR, on the same reasoning as `dynamic_topic`. The
predicate is narrow and the detection fails open, but a false positive that
refused generation would be worse than a loud warning, and there are legitimate
shapes (a node that manages publisher activation by hand) where the author
genuinely means it.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Outcome,
    PlaygroundCase,
    Severity,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register
from colcon_lf.node_analysis.models import LIFECYCLE_CHAINED_HOOKS

CODE_H005 = "RCLLF-H005"


def _owns_lifecycle_publisher(node_info) -> bool:
    """Whether skipping the base call could actually cost this node anything.

    A lifecycle node with no publisher has nothing for `impl_->on_activate()`
    to activate, so an unchained override is inert and must not be reported.
    `is_lifecycle` on `PublisherInfo` is set by the parser for every publisher
    of a lifecycle node, so the `getattr` fallback only matters for a
    hand-built `NodeInfo` in a test.
    """
    return any(getattr(pub, "is_lifecycle", False) for pub in node_info.publishers)


def _message(hook: str, node_key: str, topics: list[str]) -> str:
    """Build the diagnostic text for one unchained hook."""
    where = ", ".join(f"'{t}'" for t in topics if t) or "(none named)"
    if hook == "on_activate":
        effect = (
            "its managed publishers are never activated, so every publish() "
            f"on {where} is silently dropped"
        )
    else:
        effect = (
            "its managed publishers are never deactivated, so publishing "
            f"on {where} continues after the node leaves the active state"
        )
    return (
        f"node '{node_key}' overrides {hook}() without calling "
        f"rclcpp_lifecycle::LifecycleNode::{hook}(), so {effect}. The base "
        "implementation is what activates managed publishers; an override "
        f"must chain to it. Add `return rclcpp_lifecycle::LifecycleNode::"
        f"{hook}(state);`, or capture its result first if the override needs "
        "to run code afterwards. Note that this failure is near-silent at "
        "runtime: LifecyclePublisher warns once and then stops warning."
    )


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Emit RCLLF-H005 for every lifecycle hook override that skips the base call."""
    diagnostics: list[Diagnostic] = []
    for node_key, node_info in ctx.graph.nodes.items():
        unchained = getattr(node_info, "lifecycle_hooks_not_chained", []) or []
        if not unchained:
            continue
        if not node_info.is_lifecycle:
            continue
        if not _owns_lifecycle_publisher(node_info):
            continue
        topics = [pub.topic for pub in node_info.publishers]
        # Filter to the hooks whose base implementation actually does work,
        # rather than trusting the field's contents. The parser applies the
        # same filter, so this is defence in depth: if a future parser change
        # started recording `on_configure`, a node that legitimately does not
        # chain that no-op hook would otherwise start warning for no reason,
        # and `_message` would describe a publisher consequence that does not
        # apply.
        for hook in [h for h in unchained if h in LIFECYCLE_CHAINED_HOOKS]:
            diagnostics.append(
                Diagnostic(
                    code=CODE_H005,
                    node=node_key,
                    topic=None,
                    message=_message(hook, node_key, topics),
                )
            )
    return diagnostics


MODULE = register(
    FeatureModule(
        feature_id="lifecycle.base_call_chaining",
        title="Lifecycle hook override skips the base call",
        support=Support.WARN,
        codes=(CODE_H005,),
        severity={CODE_H005: Severity.WARNING},
        detect=detect,
        conversion=(
            "warning: the node overrides on_activate()/on_deactivate() without "
            "chaining to rclcpp_lifecycle::LifecycleNode, so its managed "
            "publishers are never activated and every publish is dropped at "
            "runtime. The extracted model still records the publisher and the "
            "generator still wires its connection, so the LF system claims an "
            "edge that carries no data. Chain to the base implementation."
        ),
        playground=PlaygroundCase(
            # unchained_activate_node is purpose-built for this code: it is a
            # lifecycle component that creates a LifecyclePublisher in
            # on_configure and overrides on_activate WITHOUT the base call,
            # which is exactly the shipped lifecycle_example bug this code
            # exists to catch. Its sibling in the same package chains
            # correctly, so the package also proves the check stays quiet on
            # correct code.
            example="lifecycle_component_example/unchained_activate",
            expected_outcome=Outcome.WARN,
            expected_codes=(CODE_H005,),
        ),
        docs_entry="Lifecycle hook override skips the base call (warning)",
    )
)
