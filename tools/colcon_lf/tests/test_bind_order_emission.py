# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Bind-before-dispatch at tag 0.

The node reactor's bind startup reaction writes ``<group>.rcllf_bound`` into
every group sub-reactor, and every reaction in a sub-reactor that runs user
code lists ``rcllf_bound`` as a non-triggering source. LF's precedence graph
then orders bind before the first dispatch at tag 0. A reactor without group
sub-reactors keeps bind as its first declared reaction, which LF orders first
within the reactor.
"""

from __future__ import annotations

import re

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import BIND_PORT, generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackGroupInfo,
    CallbackInfo,
    NodeInfo,
    SubscriberInfo,
    TimerInfo,
)


def _graph(*nodes: NodeInfo) -> SystemGraph:
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={n.name: n for n in nodes},
        communication=SystemCommunication(),
    )


def _grouped_node() -> NodeInfo:
    """Four callback groups (default, two exclusive, one reentrant with two
    callbacks) plus a subscription, every timer at offset 0."""
    groups = [
        CallbackGroupInfo(name="ex_a_", type="MutuallyExclusive"),
        CallbackGroupInfo(name="ex_b_", type="MutuallyExclusive"),
        CallbackGroupInfo(name="re_", type="Reentrant"),
    ]
    timer_groups = [None, "ex_a_", "ex_b_", "re_", "re_"]
    callbacks, timers = [], []
    for i, grp in enumerate(timer_groups):
        cb = f"tick_{i}"
        callbacks.append(CallbackInfo(name=cb))
        timers.append(TimerInfo(period_ms=100.0 * (i + 1), callback=cb, callback_group=grp))
    callbacks.append(CallbackInfo(name="on_data"))
    return NodeInfo(
        name="grp",
        class_name="GrpNode",
        package="test_pkg",
        source_file="/fake/grp.cpp",
        callbacks=callbacks,
        timers=timers,
        subscribers=[SubscriberInfo(topic="data", message_type="std_msgs::msg::Int32",
                                    callback="on_data", callback_group="ex_a_")],
        callback_groups=groups,
    )


def _missing_bind_edges(system) -> list[str]:
    """Every user-code reaction without the bind edge, named by reactor and trigger."""
    by_name = {r.name: r for r in system.reactors}
    missing: list[str] = []
    for outer in system.reactors:
        if not outer.instances:
            continue
        for inst in outer.instances:
            sub = by_name[inst.reactor_class]
            if f"{inst.name}.{BIND_PORT}" not in outer.startup_effects:
                missing.append(f"{outer.name}: bind startup does not write {inst.name}.{BIND_PORT}")
            if not any(p.name == BIND_PORT and p.direction == "input" for p in sub.ports):
                missing.append(f"{sub.name}: no input {BIND_PORT}")
            for reaction in sub.reactions:
                if BIND_PORT not in reaction.sources:
                    missing.append(f"{sub.name}: reaction({', '.join(reaction.triggers)})")
            if sub.ingress_key is not None and not sub.ingress_bound:
                missing.append(f"{sub.name}: ingress reactions")
    return missing


def test_every_group_reaction_reads_the_bind_port():
    system = generate_lf_system(_graph(_grouped_node()), target_lfc=True)
    grouped = [r for r in system.reactors if r.instances]
    assert len(grouped) == 1 and len(grouped[0].instances) >= 4
    missing = _missing_bind_edges(system)
    assert not missing, "no bind-before-dispatch edge: " + "; ".join(missing)


def test_emitted_lf_carries_the_edge():
    text = emit_lf_system(generate_lf_system(_graph(_grouped_node()), target_lfc=True))
    startup = re.search(r"reaction\(startup\) -> ([^{]+)\{=\s*ros2_lf_bind_node", text)
    assert startup, "bind startup reaction has no effects:\n" + text
    written = {e.strip() for e in startup.group(1).split(",")}
    assert {"default_group.rcllf_bound", "ex_a.rcllf_bound", "ex_b.rcllf_bound",
            "tick_3.rcllf_bound", "tick_4.rcllf_bound"} <= written, written
    for inst in ("default_group", "ex_a", "ex_b", "tick_3", "tick_4"):
        assert f"lf_set({inst}.rcllf_bound, 1);" in text
    assert re.search(r"reaction\(t_tick_0\) rcllf_bound\b", text), text
    assert re.search(r"reaction\(data\) rcllf_bound\b", text), text
    assert "reaction(ingress_drain) rcllf_bound -> ingress_drain" in text
    assert text.count("input rcllf_bound: stub_t") == 5


def test_reactor_without_groups_binds_first_in_declaration_order():
    node = NodeInfo(name="bare", class_name="BareNode", package="test_pkg",
                    source_file="/fake/bare.cpp")
    text = emit_lf_system(generate_lf_system(_graph(node), target_lfc=True))
    body = text[text.index("reactor BareNode {"):]
    reactions = re.findall(r"reaction\(([^)]*)\)", body)
    assert reactions and reactions[0] == "startup", reactions
    assert 'ros2_lf_bind_node(self, "BareNode");' in body.split("reaction(", 2)[1]
