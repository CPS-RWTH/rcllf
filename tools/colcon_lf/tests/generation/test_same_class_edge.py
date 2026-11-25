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
"""A topic edge between two instances of one node class.

The analyzer listed each endpoint by the node name in the code, which both
instances share, and the generator mapped that name back to one node key. Both
ends of the edge landed on the same reactor instance and lfc rejected the
program with ``Not a port``. Endpoints are now listed by node key.
"""

from __future__ import annotations

import copy
import json
import re
import shutil
import subprocess

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    SubscriberInfo,
    TimerInfo,
)
from colcon_lf.output.serializer import deserialize_system_graph, serialize

from ..ros_env import requires_launch_ros

LAUNCH = "17_same_class_edge.launch.py"
MSG = "std_msgs::msg::String"

# Under ``PYTHONPATH=$PWD`` the ROS launch packages do not import,
# the analyzer reads the launch file as empty and the two analysis tests failed
# with ``KeyError: '/same_class_chat'``. They skip there now (fail under
# RCLLF_REQUIRE_ROS_TESTS=1); the hand-built-graph tests below still run.
needs_launch = requires_launch_ros("the same-class edge analysis tests")


def _connections(text: str) -> list[str]:
    return re.findall(r"^\s*(\w+\.\w+ -> \w+\.\w+)", text, flags=re.M)


def _lfc_errors(tmp_path, text: str) -> list[str] | None:
    """lfc's error lines for *text*, ``[]`` if it accepts it; skips without lfc."""
    lfc = shutil.which("lfc")
    if lfc is None:
        pytest.skip("lfc not on PATH (run in the dev container)")
    # lfc wants the file named after the main reactor.
    main = re.search(r"^main reactor (\w+)", text, flags=re.M)
    src = tmp_path / "src" / f"{main.group(1) if main else 'Bringup'}.lf"
    src.parent.mkdir(parents=True)
    src.write_text(text)
    proc = subprocess.run([lfc, "--no-compile", str(src)], capture_output=True, text=True,
                          timeout=300)
    errors = [ln for ln in (proc.stdout + proc.stderr).splitlines() if "error" in ln]
    return errors if proc.returncode != 0 else []


@pytest.fixture(scope="module")
def fixture_graph(package_dir, packages_root):
    pytest.importorskip("clang.cindex", reason="libclang needed (run in the dev container)")
    from colcon_lf.graph.hierarchical import build_system_graph
    launch = package_dir("launch_features_example") / "launch" / LAUNCH
    graph = build_system_graph(launch, workspace_root=packages_root)
    errors = graph.launch_graph.root.errors
    assert not errors, f"the analyzer could not read {LAUNCH}: {errors}"
    return graph


@needs_launch
def test_analyzer_lists_the_edge_by_node_instance(fixture_graph):
    topics = {t.topic: (t.publishers, t.subscribers) for t in fixture_graph.communication.topics}
    assert topics["/same_class_chat"] == (["relay_a"], ["/grp/relay_b"]), topics


def _with_timer_periods(graph: SystemGraph) -> SystemGraph:
    """A copy of *graph* with every unresolved timer period set.

    The fixture's timer period is a parameter the analyzer cannot resolve, and
    such a timer refuses generation instead of getting a
    1000 msec placeholder. This test is about the edge, so it sets the period
    the way execution.yaml would. The value only has to be a valid period.
    """
    graph = copy.deepcopy(graph)
    for node in graph.nodes.values():
        for timer in node.timers:
            if timer.period_ms is None:
                timer.period_ms = 100.0
    return graph


@needs_launch
def test_edge_connects_the_two_reactor_instances(fixture_graph, tmp_path):
    text = emit_lf_system(
        generate_lf_system(_with_timer_periods(fixture_graph), after_delay_ns=1_000_000))
    connections = _connections(text)
    # With a timer period set the edge is admitted as a correspondence edge
    # (corr_eN ports), so the instances are checked, not the port names.
    ends = [(c.split(" -> ")[0].split(".")[0], c.split(" -> ")[1].split(".")[0])
            for c in connections]
    assert ("relay_a", "grp_relay_b") in ends, connections
    assert all(src != dst for src, dst in ends), connections
    assert _lfc_errors(tmp_path, text) == [], text


def _relay(name: str) -> NodeInfo:
    """A node class that publishes ``out`` from a timer and subscribes ``in``."""
    return NodeInfo(
        name=name, class_name="Relay", package="test_pkg", source_file="/fake/relay.cpp",
        callbacks=[CallbackInfo(name="tick", publishers_used=["out"]),
                   CallbackInfo(name="on_in")],
        timers=[TimerInfo(period_ms=100.0, callback="tick")],
        publishers=[PublisherInfo(topic="out", message_type=MSG)],
        subscribers=[SubscriberInfo(topic="in", message_type=MSG, callback="on_in")],
    )


def _legacy_graph() -> SystemGraph:
    """An older graph (before endpoints carried the node instance key): the edge lists the shared node name on both ends."""
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={"a": _relay("relay"), "b": _relay("relay")},
        communication=SystemCommunication(topics=[TopicConnection(
            topic="/chat", message_type=MSG, publishers=["relay"], subscribers=["relay"])]),
    )


def test_legacy_file_with_a_shared_endpoint_name_is_refused():
    # Before the fix this generated "b.chat -> b.in", an edge from the
    # subscriber to itself, without a word.
    with pytest.raises(ValueError, match="older colcon lf-analyze"):
        generate_lf_system(_legacy_graph(), after_delay_ns=0)


def test_legacy_file_reads_without_the_endpoint_field():
    data = json.loads(serialize(_legacy_graph()))
    del data["communication"]["endpoint_ids"]
    graph = deserialize_system_graph(json.dumps(data))
    assert graph.communication.endpoint_ids == "node_name"
