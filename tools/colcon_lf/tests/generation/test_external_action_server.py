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
"""An action server with no client inside the LF system.

Its requests come from a stock ROS client (``ros2 action send_goal``), so the
goal, result and cancel reactions are triggered by one physical action that
the binding schedules at arrival, not by request ports
nothing would ever write.
"""

from __future__ import annotations

from colcon_lf.comm_analysis.models import ActionConnection, SystemCommunication
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph

from ..corr_declared import generate_declared
from . import test_lf_generator as _gen

_PHYS = "ext_action_analyze"
_GOAL_RID = "RID_ActionServerNode_handle_goal"


def _graph(with_client: bool) -> SystemGraph:
    helper = _gen.TestActionAsyncChannelsCrossFederate()
    nodes = {"action_server": helper._make_action_server_node()}
    if with_client:
        nodes["action_client"] = helper._make_action_client_node()
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="action_example")
    return SystemGraph(
        name="action_bringup",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes=nodes,
        communication=SystemCommunication(actions=[ActionConnection(
            action="analyze",
            action_type="custom_interfaces::action::Analyze",
            server="action_server",
            clients=["action_client"] if with_client else [],
        )]),
    )


def _server_sub(system):
    return next(r for r in system.reactors if r.name == "ActionServerNode_DefaultGroup")


def test_server_reactions_are_triggered_by_the_physical_action():
    sub = _server_sub(generate_declared(_graph(with_client=False), target_lfc=True))
    by_kind = {r.kind: r for r in sub.reactions}
    for kind, entity in (
        ("action_goal_server", 0), ("action_result_server", 1), ("action_cancel_server", 2),
    ):
        r = by_kind[kind]
        assert r.triggers == [_PHYS], kind
        assert r.external_action == _PHYS
        assert r.external_register_rid == _GOAL_RID
        assert f"ros2_lf_action_external_pending(self, {_GOAL_RID}, {entity}));" in r.body
        assert "_req->value" not in r.body
    # The goal reaction still claims what its callbacks sent.
    assert by_kind["action_goal_server"].effects == [
        "analyze_goal_resp", "analyze_feedback", "analyze_result"]


def test_physical_action_is_no_port():
    sub = _server_sub(generate_declared(_graph(with_client=False), target_lfc=True))
    names = {p.name for p in sub.ports}
    assert _PHYS not in names
    assert "analyze_goal_req" not in names


def test_emitted_reactor_declares_and_registers_once():
    text = emit_lf_system(generate_declared(_graph(with_client=False), target_lfc=True))
    assert text.count(f"physical action {_PHYS}: int64_t") == 1
    assert text.count(
        f"ros2_lf_action_external_register(self, {_GOAL_RID}, "
        f"rcllf_external_wake, {_PHYS});") == 1
    assert f"reaction(startup) -> {_PHYS} {{=" in text


def test_server_with_an_lf_client_keeps_its_request_ports():
    system = generate_declared(_graph(with_client=True), target_lfc=True)
    sub = _server_sub(system)
    (goal,) = [r for r in sub.reactions if r.kind == "action_goal_server"]
    assert goal.triggers == ["analyze_goal_req"]
    assert goal.external_action is None
    assert _PHYS not in emit_lf_system(system)
