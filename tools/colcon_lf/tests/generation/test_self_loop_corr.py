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
"""A self edge goes through publication correspondence.

Before the fix the self edge was refused with ``self_loop`` and stayed a stub
link, whose subscription reaction takes from DDS with ``take_mode: skip``.
Nothing bounded that take, so a value could be skipped if the take ran before
the sample reached the reader. That race is possible on the stub path but was
not observed in 80 rcllf and 80 stock runs over four conditions.

A ``corr_e`` LF link from a node to itself would be a zero-delay cycle, so
there is no link. The publishing reaction hands its records to the carrier
(``ros2_lf_corr_arrive``) and schedules the drain action, which keeps the
topic delay d, so the first dispatch stays at t+d. The fixture is the
analyzer output of packages/src/self_loop_example, launch file overload,
captured with ``LF_LAUNCH=overload docker/lf_build.sh self_loop_example``
(then copied from packages/lf-analysis/self_loop_example/overload/system.json).
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.output.serializer import deserialize_system_graph

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "self_loop_overload_system.json"


def _graph(split: bool = False):
    data = json.loads(FIXTURE.read_text())
    if split:
        # The subscription in its own MutuallyExclusive group: its drain sits
        # in another sub-reactor than the publishing timer reaction.
        node = data["nodes"]["self_loop_overload_node"]
        node["callback_groups"].append({"name": "sub_group", "type": "MutuallyExclusive"})
        node["subscribers"][0]["callback_group"] = "sub_group"
    return deserialize_system_graph(json.dumps(data))


def _reactor(system, name):
    return next(r for r in system.reactors if r.name == name)


def _by_kind(reactor):
    out = {}
    for r in reactor.reactions:
        out.setdefault(r.kind, []).append(r)
    return out


@pytest.mark.filterwarnings("ignore::UserWarning")
class TestSelfEdgeCorrespondence:
    @pytest.fixture()
    def system(self):
        return generate_lf_system(_graph(), after_delay_ns=1_000_000)

    def test_self_edge_is_admitted(self, system):
        (edge,) = system.corr_edges
        assert (edge.status, edge.reason) == ("admitted", None), (edge.status, edge.reason)
        assert edge.delay_ns == 1_000_000

    def test_no_link_and_no_corr_port(self, system):
        assert system.main_connections == []
        for name in ("SelfLoopOverloadNode", "SelfLoopOverloadNode_DefaultGroup"):
            ports = [p.name for p in _reactor(system, name).ports]
            assert not [p for p in ports if p.startswith("corr_e")], (name, ports)
            assert "self_overload" not in ports and "self_overload_in" not in ports, (name, ports)

    def test_self_edge_keeps_its_own_timeout(self, system):
        # T counts from the publication's commit, so no edge is chained
        # over its publisher's inputs. T stays the own 100 ms.
        (edge,) = system.corr_edges
        assert edge.timeout_ns == 100_000_000, edge.timeout_ns
        assert "timeout_chain_cycle" not in edge.bound_inputs, edge.bound_inputs
        assert "T_upstream_ns" not in edge.bound_inputs, edge.bound_inputs

    def test_timer_reaction_arrives_and_schedules_the_drain(self, system):
        group = _reactor(system, "SelfLoopOverloadNode_DefaultGroup")
        kinds = _by_kind(group)
        (timer,) = kinds["timer"]
        assert timer.effects == ["corr_drain_e0"]
        assert "ros2_lf_corr_emit(self, 0)" in timer.body
        assert "ros2_lf_corr_arrive(self, 0, n)" in timer.body
        assert "lf_schedule(corr_drain_e0, w)" in timer.body
        assert "corr_arrive" not in kinds
        (drain,) = kinds["sub"]
        assert drain.triggers == ["corr_drain_e0"]
        assert drain.corr_drain_edge == 0
        assert "corr_drain_e0" in group.logical_actions

    def test_emitted_program_has_no_self_connection(self, system):
        text = emit_lf_system(system)
        assert "self_loop_overload_node.self_overload" not in text
        assert "corr_e0" not in text.replace("corr_drain_e0", "")


@pytest.mark.filterwarnings("ignore::UserWarning")
class TestSplitSelfEdgeStaysLegacy:
    def test_split_self_edge_is_refused_and_keeps_the_stub_link(self):
        system = generate_lf_system(_graph(split=True), after_delay_ns=1_000_000)
        (edge,) = system.corr_edges
        assert (edge.status, edge.reason) == ("legacy", "self_loop_split")
        (conn,) = system.main_connections
        assert conn.source == "self_loop_overload_node.self_overload"
        assert conn.destination == "self_loop_overload_node.self_overload_in"
        assert conn.after_delay_ns == 1_000_000
