# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""An execution.yaml timer period reaches the binding's RID table.

``nodes.<node>.timers.<cb>.period`` sets the LF timer, and the RID entry of
that timer must carry the same period: ``ros2_lf_bind_node`` matches timer
handles to reactions by period and refuses to start when none fits ("RID 1
(FeatureComponent::timer_callback, timer, period 0 ns) has no unbound timer
handle ... that fits it").

To falsify: drop ``timer_period_overrides`` from ``collect_rid_table`` (or
build its period map from system.json alone) and these tests
fail on period 0. The end-to-end case on a real analysis is
``tests/resolved_model/test_gate.py::test_override_reaches_the_binding_rid_table``.
"""

from __future__ import annotations

import dataclasses

from colcon_lf.generation.binding_header_emitter import collect_rid_table
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from .test_correspondence_emission import _graph, _pub, _sub, _tc
from .test_xfed_correspondence import _federate


def _period_ns(table, binding_key: str, callback: str = "tick") -> int:
    (row,) = [r for r in table if r[1] == binding_key and r[2] == callback]
    return row[5]


def test_rid_period_is_the_overridden_period_per_instance():
    # Two instances of one class: "a" has a parameter-driven (null) period that
    # execution.yaml sets to 250 msec, "b" keeps its analyzed 100 msec.
    a = _pub("a", "PubNode", "data_a")
    a = dataclasses.replace(a, timers=[dataclasses.replace(a.timers[0], period_ms=None)])
    graph = _graph([a, _pub("b", "PubNode", "data_b")], [])
    overrides = {"a": {"tick": 250.0}}

    system = generate_lf_system(graph, target_lfc=True, timer_period_overrides=overrides)
    lf = emit_lf_system(system)
    assert "timer t_tick(0, 250 msec)" in lf and "timer t_tick(0, 100 msec)" in lf, lf

    table = collect_rid_table({r.name: r for r in system.reactors}, graph,
                              timer_period_overrides=overrides)
    keys = sorted({r[1] for r in table})
    periods = {k: _period_ns(table, k) for k in keys}
    assert sorted(periods.values()) == [100_000_000, 250_000_000], (
        f"RID timer periods {periods} do not match the LF timers (250 msec override, "
        f"100 msec analyzed), so the binding finds no handle of that period")


def test_federated_rid_init_carries_the_overridden_period(tmp_path):
    pub = _pub("pub", "PubNode")
    pub = dataclasses.replace(pub, timers=[dataclasses.replace(pub.timers[0], period_ms=None)])
    out = _federate(tmp_path, [pub, _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])],
                    {"pub": "pub", "sub": "sub"},
                    timer_period_overrides={"pub": {"tick": 250.0}})
    init = out["pub"]["init"]
    assert '"PubNode", "tick", "timer", "", 250000000LL,' in init, (
        f"the federate's RID entry for PubNode::tick does not carry the 250 msec "
        f"override:\n{init}")
