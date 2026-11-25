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
"""One-scheduler ingress emission (T7).

Every node gets exactly one drain host: the MutuallyExclusive default-group
sub-reactor, else (reentrant or empty default group) the outer node reactor
(Ruling R11). The drain host declares ``ingress_wake``/``ingress_drain`` and
the four ingress reactions; the preamble's wake thunk returns the
``lf_schedule`` handle so the binding can detect a refusal.
"""

from __future__ import annotations

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackGroupInfo,
    CallbackInfo,
    NodeInfo,
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


def _node(name: str, cls: str, groups=None, timer_groups=None) -> NodeInfo:
    """A node with one timer per entry of ``timer_groups`` (group name or None)."""
    timer_groups = timer_groups if timer_groups is not None else [None]
    callbacks, timers = [], []
    for i, grp in enumerate(timer_groups):
        cb = f"tick_{i}"
        callbacks.append(CallbackInfo(name=cb))
        timers.append(TimerInfo(period_ms=100.0 * (i + 1), callback=cb, callback_group=grp))
    return NodeInfo(
        name=name,
        class_name=cls,
        package="test_pkg",
        source_file=f"/fake/{name}.cpp",
        callbacks=callbacks,
        timers=timers,
        callback_groups=groups or [],
    )


def _hosts(system):
    return {r.name: r.ingress_key for r in system.reactors if r.ingress_key is not None}


def test_plain_node_drains_in_default_group_subreactor():
    system = generate_lf_system(_graph(_node("pub", "PubNode")), target_lfc=True)
    assert _hosts(system) == {"PubNode_DefaultGroup": "PubNode"}


def test_grouped_node_drains_in_default_group_not_other_group():
    node = _node(
        "grp", "GrpNode",
        groups=[CallbackGroupInfo(name="exclusive_group_", type="MutuallyExclusive")],
        timer_groups=[None, "exclusive_group_"],
    )
    system = generate_lf_system(_graph(node), target_lfc=True)
    assert _hosts(system) == {"GrpNode_DefaultGroup": "GrpNode"}


def test_empty_default_group_drains_in_outer_reactor():
    node = _node(
        "only_ex", "OnlyExNode",
        groups=[CallbackGroupInfo(name="exclusive_group_", type="MutuallyExclusive")],
        timer_groups=["exclusive_group_"],
    )
    system = generate_lf_system(_graph(node), target_lfc=True)
    assert _hosts(system) == {"OnlyExNode": "OnlyExNode"}


def test_reentrant_default_group_drains_in_outer_reactor():
    node = _node(
        "re", "ReNode",
        groups=[CallbackGroupInfo(name="default_group", type="Reentrant")],
        timer_groups=["default_group", "default_group"],
    )
    system = generate_lf_system(_graph(node), target_lfc=True)
    assert _hosts(system) == {"ReNode": "ReNode"}


def test_one_ingress_wake_per_node_and_all_four_reactions():
    system = generate_lf_system(
        _graph(_node("a", "ANode"), _node("b", "BNode")), target_lfc=True,
    )
    text = emit_lf_system(system)
    assert text.count("physical action ingress_wake") == 2
    assert text.count("logical action ingress_drain") == 2
    for key in ("ANode", "BNode"):
        assert text.count(
            f'ros2_lf_ingress_register(self, "{key}", rcllf_ingress_wake, ingress_wake);'
        ) == 1
        assert text.count(f'ros2_lf_ingress_move("{key}")') == 1
        assert text.count(f'ros2_lf_ingress_drain_one(self, "{key}")') == 1
        assert text.count(f'ros2_lf_ingress_close("{key}");') == 1
        assert text.count(f'ros2_lf_ingress_schedule_refused("{key}");') == 2
    # Group-hosted drains read the bind port, see test_bind_order_emission.
    assert text.count("reaction(ingress_wake) rcllf_bound -> ingress_drain") == 2
    assert text.count("reaction(ingress_drain) rcllf_bound -> ingress_drain") == 2


def test_wake_thunk_returns_the_schedule_handle():
    system = generate_lf_system(_graph(_node("a", "ANode")), target_lfc=True)
    assert "static inline int rcllf_ingress_wake(void * action) {" in system.preamble
    assert "return (int) lf_schedule(action, 0);" in system.preamble
    # Federated lfc output includes the preamble twice in one translation
    # unit (measured: 'redefinition of int rcllf_ingress_wake(void*)').
    assert system.preamble.index("#ifndef RCLLF_INGRESS_WAKE_DEFINED") < system.preamble.index(
        "static inline int rcllf_ingress_wake"
    )


def test_drain_host_reactions_follow_user_reactions():
    """Ingress reactions come last, so existing reaction priorities are unchanged."""
    system = generate_lf_system(_graph(_node("a", "ANode")), target_lfc=True)
    text = emit_lf_system(system)
    block = text[text.index("reactor ANode_DefaultGroup"):]
    assert block.index("ros2_lf_invoke_timer") < block.index("ros2_lf_ingress_register")


def test_ingress_capacity_is_set_before_register_only_when_not_default():
    """execution.yaml ``ingress_capacity`` reaches the drain host."""
    graph = _graph(_node("a", "ANode"), _node("b", "BNode"))
    text = emit_lf_system(
        generate_lf_system(graph, target_lfc=True, ingress_capacity={"a": 2, "b": 8})
    )
    set_a = 'ros2_lf_ingress_set_capacity("ANode", 2);'
    assert text.count(set_a) == 1
    assert text.index(set_a) < text.index('ros2_lf_ingress_register(self, "ANode"')
    # The default 8 emits nothing, so every existing generated file is unchanged.
    assert "ros2_lf_ingress_set_capacity(\"BNode\"" not in text
    assert emit_lf_system(generate_lf_system(graph, target_lfc=True)).count(
        "ros2_lf_ingress_set_capacity") == 0
