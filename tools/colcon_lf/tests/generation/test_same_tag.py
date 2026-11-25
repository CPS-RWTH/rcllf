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
"""Same-tag delivery of the first zero-delay record.

An admitted in-process edge with d = 0 runs the subscription callback of the
first record of a tag at the publication tag (t, m). The generator merges the
record port into the drain and tells the runtime, which keys the release at
m. Every other edge keeps the separate arrive and the (t, m + 1) key.
"""

from __future__ import annotations

import dataclasses

import pytest

from colcon_lf.exec_config.resolve import CorrBoundsConfig
from colcon_lf.generation.binding_header_emitter import collect_rid_table, emit_rid_init_cpp
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import corr_same_tag
from colcon_lf.generation.trace_manifest import _edge_row

from ..corr_declared import generate_declared
from .test_correspondence_emission import _edge, _graph, _pub, _reactor_block, _sub, _tc

SET = "ros2_lf_binding::set_corr_edge_same_tag(0);"


def _system(**gen):
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    return graph, generate_declared(graph, **gen)


def _cpp(graph, system):
    return emit_rid_init_cpp(collect_rid_table({r.name: r for r in system.reactors}, graph),
                             "pkg", corr_edges=system.corr_edges)


def test_zero_delay_in_process_edge_is_same_tag_in_lf_header_and_manifest():
    graph, system = _system(after_delay_ns=0)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.role, e.delay_ns, e.same_tag) == ("admitted", "local", 0, True), e
    assert SET in _cpp(graph, system), f"edge {e.label()}: runtime not told"
    assert _edge_row(e).get("same_tag") is True, _edge_row(e)
    group = _reactor_block(emit_lf_system(system), "SubNode_DefaultGroup")
    assert "reaction(corr_drain_e0, corr_e0)" in group, group


def test_positive_delay_keeps_the_split_drain_and_the_runtime_key():
    graph, system = _system(after_delay_ns=5_000_000)
    e = _edge(system, "pub", "sub")
    assert (e.status, e.delay_ns, e.same_tag) == ("admitted", 5_000_000, False), e
    assert SET not in _cpp(graph, system)
    assert "same_tag" not in _edge_row(e)
    group = _reactor_block(emit_lf_system(system), "SubNode_DefaultGroup")
    assert "reaction(corr_e0) rcllf_bound -> corr_drain_e0 {=" in group, group


def test_deferred_join_on_a_local_edge_is_refused_before_same_tag():
    # Keeps the drain action for deferred joins. Generation allows
    # 'join: deferred' on cross-federate edges only (a local edge is decided at
    # publish), and those are never same-tag, so a local deferred edge is a
    # generation error, not a split drain. The predicate test below pins that
    # a deferred edge is never same-tag.
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    bounds = CorrBoundsConfig(burst={n: 1 for n in graph.nodes},
                              deferred_join={("sub", "data")})
    with pytest.raises(ValueError, match=r"local edge .* is decided at publish"):
        generate_declared(graph, after_delay_ns=0, corr_bounds=bounds)


def test_split_escape_turns_same_tag_off(monkeypatch):
    monkeypatch.setenv("RCLLF_LOCAL_SPLIT_DRAIN", "1")
    graph, system = _system(after_delay_ns=0)
    e = _edge(system, "pub", "sub")
    assert not e.same_tag, e
    assert SET not in _cpp(graph, system)


def test_predicate_refuses_self_remote_and_unadmitted_edges():
    _, system = _system(after_delay_ns=0)
    e = _edge(system, "pub", "sub")
    assert corr_same_tag(e)
    for change in ({"sub_node": "pub"}, {"role": "egress"}, {"role": "ingress"},
                   {"edge_id": None}, {"delay_ns": 1}, {"deferred": True}):
        assert not corr_same_tag(dataclasses.replace(e, **change)), change
