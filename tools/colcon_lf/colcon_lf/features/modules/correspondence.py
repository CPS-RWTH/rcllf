# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""core.correspondence, publication correspondence on topic edges.

An admitted unfederated topic edge dispatches the subscriber once per actual
publish with that publish's own sample, at t+d (d = the edge's after-delay).
A cross-federate edge is admitted too, under centralized coordination
and decentralized coordination (BNA inputs included),
with its records sent by the publishing reaction over
``after d``, so a relay that republishes in a subscription
reaction is admitted. The admitted edges into one
subscription form one fan-in group: several publishing nodes share
one drain and one delivery order, by release tag, then the rank of the
publisher's instance name, then publication order. A group is admitted whole or
stays legacy whole. Edges refused at generation keep the legacy path, whose stub
link counts each publish (one event per reaction only with
``RCLLF_COUNT_PUBLISHES=0`` at generation, Ruling Q1): a cross-federate ``after``
of 1 ns when the flush sender is in use (``xfed_flush_min``; d = 0 is admitted;
the flush sender, decentralized d > 0 or
``RCLLF_XFED_FLUSH=1``, carries a COMPLETE record),
``transient_local`` QoS, publishing callbacks that are not timer or subscription
reactions or that sit in different callback groups, and fan-in
groups whose members differ in delay or locality, or with a self, rebinding or
shared-publisher member. A legacy cross-federate link written at a third
microstep of one time exits. Each is listed with its reason in
the generated ``correspondence.json`` and at startup, so ``detect`` adds no
diagnostic of its own.
"""

from __future__ import annotations

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Outcome,
    PlaygroundCase,
    Support,
)
from colcon_lf.features.registry import FeatureModule, register


def detect(ctx: DetectionContext) -> list[Diagnostic]:
    """Admission is decided at generation and reported there, not as a diagnostic."""
    return []


MODULE = register(
    FeatureModule(
        feature_id="core.correspondence",
        title="Publication correspondence",
        support=Support.SUPPORTED,
        codes=(),
        severity={},
        detect=detect,
        conversion=(
            "An admitted topic edge carries one record per actual publish over a "
            "zero-delay LF connection into a subscriber-side carrier drained from t+d, "
            "each dispatch joined with the exact DDS sample of that publish."
        ),
        playground=PlaygroundCase(
            example="pubsub_example/bringup",
            expected_outcome=Outcome.PASS,
            expected_codes=(),
        ),
        docs_entry=(
            "Publication correspondence (supported for admitted topic edges, including "
            "cross-federate edges and relays on one clock, or on hosts with a "
            "declared clock offset bound below the edge timeout "
            "(ROS2_LF_XFED_CLOCK_BOUND_NS). Another host without a declared "
            "bound is refused. Lifecycle nodes are admitted. Several "
            "publishing nodes into one subscription are admitted as one fan-in group with "
            "one delivery order: release tag, then the rank of the publisher's instance "
            "name, then publication order. A group stays legacy as a whole "
            "when a member is refused, the members' delays differ, some members cross "
            "federates and others do not, or a member is a self edge, rebinds with a "
            "lifecycle node or shares its publisher with another member. Legacy for "
            "transient_local, services and actions)"
        ),
    )
)
