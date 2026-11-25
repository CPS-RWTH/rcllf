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
"""A member helper that publishes does not make its edge legacy.

brake_fusion's radar, camera and wheel speed nodes publish from
``serve_pending``, the controller from ``try_grid_step`` (through
``grid_tick``). The analyzer lists each helper as a callback with the topic in
``publishers_used``, and before the fix ``_corr_refusal`` refused every such
edge as ``publisher_callback_kind``, since no timer or subscription names the
helper. Now the analyzer records the helper's callers and folds its topics
into theirs, and the generator leaves out a helper whose every caller is a
timer or subscription callback.

The fixture is the analyzer output of packages/src/brake_fusion, launch file
k3_assigned, recorded with ``colcon lf-analyze --package brake_fusion`` in the
dev image on the fix branch (then copied from
packages/lf-analysis/brake_fusion/k3_assigned/system.json).
It predates the change that moved the controller's grid mode
(``grid_tick``, ``try_grid_step``, ``grid_deadline``) out of ControllerNode into
GridControllerNode; the fixture still records the helper case it tests.
"""

from __future__ import annotations

import json
from pathlib import Path
from unittest import mock

import pytest

from colcon_lf.exec_config.resolve import CorrBoundsConfig
from colcon_lf.generation import lf_generator
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.output.serializer import deserialize_system_graph

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "brake_fusion_k3_assigned_system.json"

# The bounds the analyzer cannot prove, declared as an exec config would:
# the controller's fused_callback deadline handler is out of reach (k), and
# the subscription callbacks that call a helper are fed by legacy edges (rate).
BOUNDS = CorrBoundsConfig(
    burst={"controller": 3},
    min_input_period_ms={
        ("radar", "/sim/state"): 1.0,
        ("camera", "/sim/state"): 1.0,
        ("wheel_speed", "/sim/state"): 1.0,
        ("controller", "/fused/range"): 1.0,
        ("controller", "/wheel/speed"): 1.0,
    },
)
PERIODS = {"controller": {"grid_deadline": 10.0}}  # input_wait_ms, as config/k5 states

HELPER_EDGES = {
    "/radar/range": ["state_callback", "timer_callback"],
    "/camera/range": ["state_callback", "timer_callback"],
    "/brake_cmd": ["fused_callback", "wheel_callback", "control_callback", "grid_deadline"],
}


def _data():
    return json.loads(FIXTURE.read_text())


def _generate(data=None, bounds=BOUNDS):
    graph = deserialize_system_graph(json.dumps(data or _data()))
    return generate_lf_system(
        graph, after_delay_ns=1_000_000, corr_bounds=bounds, timer_period_overrides=PERIODS)


def _edges(data=None, bounds=BOUNDS):
    # The edge classification is what this file tests. A legacy edge into
    # ExactFusionNode's synchronizer then refuses generation (RCLLF-H003
    # sync_input_legacy, pinned below and in test_sync_input_admission.py), so
    # the classification is read with that later check switched off.
    with mock.patch.object(lf_generator, "_check_sync_inputs", lambda *_: None):
        system = _generate(data, bounds)
    return {e.topic: e for e in system.corr_edges if e.pub_node != "simulator"}


def _cb(data, node, name):
    return next(cb for cb in data["nodes"][node]["callbacks"] if cb["name"] == name)


def test_fixture_records_the_helpers_callers():
    data = _data()
    for node in ("radar", "camera", "wheel_speed"):
        assert _cb(data, node, "serve_pending")["callers"] == ["state_callback", "timer_callback"]
    assert _cb(data, "controller", "try_grid_step")["callers"] == [
        "fused_callback", "grid_deadline", "grid_tick", "wheel_callback"]
    assert _cb(data, "controller", "grid_tick")["callers"] == ["control_callback"]
    # Folded: the callers publish what the helpers publish.
    assert "/radar/range" in _cb(data, "radar", "state_callback")["publishers_used"]
    assert "/brake_cmd" in _cb(data, "controller", "fused_callback")["publishers_used"]


@pytest.mark.filterwarnings("ignore::UserWarning")
@pytest.mark.parametrize("topic", sorted(HELPER_EDGES))
def test_helper_edge_is_admitted(topic):
    edge = _edges()[topic]
    assert (edge.status, edge.reason) == ("admitted", None), (
        f"{topic} {edge.pub_node} -> {edge.sub_node}: {edge.status} {edge.reason}")
    assert edge.pub_callbacks == HELPER_EDGES[topic]


@pytest.mark.filterwarnings("ignore::UserWarning")
def test_without_declared_bounds_the_sizing_gate_decides():
    """Not the helper: each edge goes on to the bounds the analysis cannot prove."""
    reasons = {t: e.reason for t, e in _edges(bounds=None).items() if t in HELPER_EDGES}
    assert reasons == {
        "/radar/range": "rate_unbounded",
        "/camera/range": "rate_unbounded",
        "/brake_cmd": "burst_unbounded",
    }


@pytest.mark.filterwarnings("ignore::UserWarning")
def test_without_declared_bounds_the_fusion_sync_input_refuses():
    """The radar input of ExactFusionNode's ExactTime synchronizer is a
    legacy edge without the declared bounds, so generation refuses."""
    with pytest.raises(ValueError, match=r"RCLLF-H003 sync_input_legacy:rate_unbounded: "
                                         r"ExactTime input radar_sub_ "):
        _generate(bounds=None)


@pytest.mark.filterwarnings("ignore::UserWarning")
@pytest.mark.parametrize("callers", [None, [], ["timer_callback", "<constructor>"],
                                     ["timer_callback", "<used, not called, in <constructor>>"]])
def test_helper_with_an_unknown_or_outside_caller_stays_legacy(callers):
    data = _data()
    _cb(data, "radar", "serve_pending")["callers"] = callers
    edge = _edges(data)["/radar/range"]
    assert (edge.status, edge.reason) == ("legacy", "publisher_callback_kind")


@pytest.mark.filterwarnings("ignore::UserWarning")
def test_helper_reached_through_a_non_reaction_helper_stays_legacy():
    data = _data()
    _cb(data, "controller", "grid_tick")["callers"] = ["control_callback", "<constructor>"]
    edge = _edges(data)["/brake_cmd"]
    assert (edge.status, edge.reason) == ("legacy", "publisher_callback_kind")
