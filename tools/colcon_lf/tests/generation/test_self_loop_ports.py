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
"""A node that subscribes to the topic it publishes.

The fixture is the analyzer's real output for packages/src/self_loop_example,
captured with
``docker/lf_build.sh self_loop_example`` (then copied from
packages/lf-analysis/self_loop_example/bringup/system.json). Before the fix the
generator gave the node an output and an input both named ``self_count``, and
lfc failed with "Duplicate Variable 'self_count' in Reactor 'SelfLoopNode'".
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.output.serializer import deserialize_system_graph

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "self_loop_system.json"


def _graph():
    return deserialize_system_graph(FIXTURE.read_text())


def _reactor(system, name):
    return next(r for r in system.reactors if r.name == name)


def _assert_no_duplicate_variables(system):
    """The lfc check: one namespace per reactor for ports, timers, actions, instances."""
    for reactor in system.reactors:
        names = (
            [p.name for p in reactor.ports]
            + [t.name for t in reactor.timers]
            + [a.name for a in reactor.physical_actions]
            + list(reactor.logical_actions)
            + [i.name for i in reactor.instances]
        )
        seen = set()
        for name in names:
            assert name not in seen, (
                f"Duplicate Variable '{name}' in Reactor '{reactor.name}'"
            )
            seen.add(name)


@pytest.mark.filterwarnings("ignore::UserWarning")
class TestSelfLoopPorts:
    @pytest.fixture()
    def system(self):
        return generate_lf_system(_graph())

    def test_no_duplicate_variable(self, system):
        _assert_no_duplicate_variables(system)

    def test_one_output_and_one_input_with_distinct_names(self, system):
        for name in ("SelfLoopNode", "SelfLoopNode_DefaultGroup"):
            ports = {(p.name, p.direction) for p in _reactor(system, name).ports}
            assert ("self_count", "output") in ports, (name, ports)
            assert ("self_count_in", "input") in ports, (name, ports)
            assert ("self_count", "input") not in ports, (name, ports)

    def test_reactions_use_the_split_names(self, system):
        group = _reactor(system, "SelfLoopNode_DefaultGroup")
        by_kind = {r.kind: r for r in group.reactions}
        assert "self_count_in" in by_kind["sub"].triggers
        assert by_kind["timer"].effects == ["self_count"]

    def test_self_edge_is_a_legacy_link_with_the_topic_delay(self, system):
        # This fixture predates the analyzer's publish bounds, so the
        # self edge stays legacy as burst_unbounded and keeps the stub ports.
        # An analyzed self edge goes through correspondence
        # (test_self_loop_corr.py).
        (edge,) = system.corr_edges
        assert (edge.status, edge.reason) == ("legacy", "burst_unbounded")
        (conn,) = system.main_connections
        assert conn.source == "self_loop_node.self_count"
        assert conn.destination == "self_loop_node.self_count_in"
        assert conn.after_delay_ns == 0
        assert "self_loop_node.self_count -> self_loop_node.self_count_in after 0" in (
            emit_lf_system(system)
        )

    def test_declared_after_delay_applies_to_the_self_edge(self):
        system = generate_lf_system(_graph(), after_delay_ns=5_000_000)
        (conn,) = system.main_connections
        assert conn.after_delay_ns == 5_000_000
        assert "self_count -> self_loop_node.self_count_in after 5 msec" in (
            emit_lf_system(system)
        )
