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
"""Launch ``parameters=[...]`` keep their launch order in the lfc run.

For a ``Node`` ``launch_ros`` puts the list in order on the command line and
writes each inline dict to a parameter file, keyed by the node's full name
when the launch file gives a name and a namespace and by ``/**`` otherwise.
For a composable node it merges the list in order into the load request. The
tests parse a real launch file, so the order has to survive the analyzer, the
generator and the binding arguments. They also check that a relative parameter file still applies when
the generated system runs from another working directory.
"""

from __future__ import annotations

import pytest
import yaml

pytest.importorskip("launch_ros", reason="launch_ros not available, run inside Docker")

from colcon_lf.generation.node_identity import build_node_identities  # noqa: E402
from colcon_lf.graph.models import SystemGraph  # noqa: E402
from colcon_lf.node_analysis.models import NodeInfo  # noqa: E402
from colcon_lf.verb.generate import build_component_instances  # noqa: E402

NODE_LAUNCH = """\
import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([Node(
        package='test_pkg', executable='sub', name='sub', namespace='ns1',
        parameters={parameters},
    )])
"""

COMPONENT_LAUNCH = """\
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return launch.LaunchDescription([ComposableNodeContainer(
        name='box', namespace='', package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[ComposableNode(
            package='test_pkg', plugin='test_pkg::Sub', name='sub',
            namespace='ns1', parameters={parameters},
        )],
    )])
"""


def _parse(tmp_path, template: str, parameters: str):
    from colcon_lf.launch_analysis.parser import clear_parse_launch_cache, parse_launch

    pkg = tmp_path / "src" / "test_pkg"
    (pkg / "launch").mkdir(parents=True, exist_ok=True)
    (pkg / "package.xml").write_text("<package><name>test_pkg</name></package>")
    launch_file = pkg / "launch" / "order.launch.py"
    launch_file.write_text(template.format(parameters=parameters))
    clear_parse_launch_cache()
    return parse_launch(launch_file, workspace_root=tmp_path)


def _system(launch_graph, component: bool = False) -> SystemGraph:
    ni = NodeInfo(
        name="sub", class_name="Sub", package="test_pkg",
        source_file="/fake/sub.cpp", is_component=component,
    )
    return SystemGraph(name="order", launch_graph=launch_graph, nodes={"/ns1/sub": ni})


def _param_args(graph) -> list[str]:
    return list(build_node_identities(graph)["/ns1/sub"].param_args)


def _rate_rules(args: list[str]) -> list[tuple[int, str]]:
    """(position, key) of every rule in *args* that sets ``rate``.

    A ``-p rate:=v`` rule applies under ``/**``. An inline YAML payload names
    the node key it applies under.
    """
    out = []
    for i, arg in enumerate(args[:-1]):
        nxt = args[i + 1]
        if arg == "-p" and nxt.startswith("rate:="):
            out.append((i, "/**"))
        elif arg == "--rcllf-params-yaml":
            for key, body in yaml.safe_load(nxt).items():
                if "rate" in body.get("ros__parameters", {}):
                    out.append((i, key))
    return out


def _node_keyed_file(tmp_path):
    f = tmp_path / "sub.yaml"
    f.write_text("/ns1/sub:\n  ros__parameters:\n    rate: 7\n")
    return f


def _assert_inline_wins(args: list[str], f) -> None:
    file_pos = args.index("--params-file") if "--params-file" in args else -1
    assert file_pos >= 0 and args[file_pos + 1] == str(f), (
        f"parameter file {f} missing from the node's --ros-args {args}"
    )
    rules = _rate_rules(args)
    assert rules, f"inline rate:=8 missing from the node's --ros-args {args}"
    pos, key = rules[-1]
    assert pos > file_pos, (
        "inline rate:=8 is applied before the node-keyed file that sets rate: 7, "
        "so the lfc run gets 7 where launch_ros gets 8: "
        f"{args}"
    )
    assert key == "/ns1/sub", (
        f"inline rate:=8 applies under {key!r}, which loses to the node-keyed "
        f"file once an earlier inline value created that entry: {args}"
    )


def test_later_inline_value_beats_an_earlier_node_keyed_file(tmp_path):
    f = _node_keyed_file(tmp_path)
    graph = _parse(tmp_path, NODE_LAUNCH, f"[{{'robot': 'r1'}}, '{f}', {{'rate': 8}}]")
    _assert_inline_wins(_param_args(_system(graph)), f)


def _inline_payloads(args: list[str]) -> list[dict]:
    return [yaml.safe_load(args[i + 1]) for i, a in enumerate(args[:-1])
            if a == "--rcllf-params-yaml"]


def test_unnamespaced_node_files_inline_values_under_the_wildcard(tmp_path):
    # Without namespace= launch_ros keys an inline dict by /**, so the
    # node-keyed file beats even the later inline rate (ros2 launch gives 7).
    f = tmp_path / "sub.yaml"
    f.write_text("/sub:\n  ros__parameters:\n    rate: 7\n")
    graph = _parse(
        tmp_path, NODE_LAUNCH.replace(" namespace='ns1',", ""),
        f"[{{'robot': 'r1'}}, '{f}', {{'rate': 8}}]")
    system = _system(graph)
    system.nodes = {"sub": system.nodes["/ns1/sub"]}
    args = list(build_node_identities(system)["sub"].param_args)
    keys = [list(p) for p in _inline_payloads(args)]
    assert keys == [["/**"], ["/**"]], (
        "inline parameters of a node launched without namespace= must apply "
        f"under /** as launch_ros files them, got keys {keys} in {args}"
    )
    assert args.index("--params-file") == 2, args


def test_component_merges_the_sources_in_launch_order(tmp_path):
    # launch_ros merges a component's list in order into the LoadNode request,
    # so the later rate 8 beats the file's 7 and robot r1 survives the file.
    f = _node_keyed_file(tmp_path)
    graph = _parse(tmp_path, COMPONENT_LAUNCH, f"[{{'robot': 'r1'}}, '{f}', {{'rate': 8}}]")
    system = _system(graph, component=True)
    (inst,) = build_component_instances(
        system, {"/ns1/sub": "sub"}, build_node_identities(system))
    args = list(inst.get("param_args", []))
    expected = [{"/ns1/sub": {"ros__parameters": {"robot": "r1", "rate": 8}}}]
    assert _inline_payloads(args) == expected and "--params-file" not in args, (
        "component parameters are not merged in launch order, so the file's "
        f"rate: 7 can beat the later inline rate:=8: {args}"
    )


def test_relative_parameter_file_applies_from_another_directory(tmp_path, monkeypatch):
    analyze_dir = tmp_path / "analyze"
    analyze_dir.mkdir()
    (analyze_dir / "rel.yaml").write_text("/ns1/sub:\n  ros__parameters:\n    rate: 7\n")
    monkeypatch.chdir(analyze_dir)
    graph = _parse(tmp_path, NODE_LAUNCH, "['rel.yaml']")

    run_dir = tmp_path / "elsewhere"
    run_dir.mkdir()
    monkeypatch.chdir(run_dir)
    args = _param_args(_system(graph))
    expected = ["--params-file", str(analyze_dir / "rel.yaml")]
    assert args == expected, (
        "relative parameter file rel.yaml is lost once the generated system "
        f"runs from another directory: expected {expected}, got {args}"
    )
