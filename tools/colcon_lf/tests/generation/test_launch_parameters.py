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
"""Launch ``parameters=[...]`` reach the node in the lfc run.

The analyzer records each launch node's inline parameters and parameter files.
Before the fix no generator stage read them, so the node factory built every
node with name, namespace and remaps only, and the lfc run silently differed
from ``ros2 launch`` of the same file. The parameters now travel as
``--ros-args`` rules of the node's own ``NodeOptions``, the way ``launch_ros``
puts them on the node command line.
"""

from __future__ import annotations

from colcon_lf.generation.factory_emitter import emit_node_factories_cpp
from colcon_lf.generation.node_identity import build_node_identities
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph, LaunchNodeAction
from colcon_lf.node_analysis.models import NodeInfo
from colcon_lf.verb.generate import _factory_entries, build_component_instances

QOS_KEY = "qos_overrides./best_effort_topic.subscription.reliability"


def _graph(ni: NodeInfo, action: LaunchNodeAction) -> SystemGraph:
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return SystemGraph(
        name="bringup",
        launch_graph=LaunchGraph(root=root, all_nodes=[action]),
        nodes={ni.name: ni},
    )


def _node(component: bool = False) -> NodeInfo:
    return NodeInfo(
        name="sub", class_name="SubNode", package="test_pkg",
        source_file="/fake/sub.cpp", is_component=component,
    )


def _action(params_file, parameters=None) -> LaunchNodeAction:
    return LaunchNodeAction(
        package="test_pkg", executable="sub", name="sub",
        parameters=parameters if parameters is not None else {QOS_KEY: "best_effort"},
        parameter_files=[params_file],
    )


def _yaml(tmp_path):
    f = tmp_path / "sub.yaml"
    f.write_text("sub:\n  ros__parameters:\n    rate: 7\n")
    return f


def _dropped(expected: list[str], actual: list[str]) -> str:
    return (
        "launch parameters dropped from the node's --ros-args: "
        f"expected {expected} in {actual}"
    )


def _contains(seq: list[str], sub: list[str]) -> bool:
    return any(seq[i:i + len(sub)] == sub for i in range(len(seq) - len(sub) + 1))


def test_non_component_factory_passes_inline_parameters_then_files(tmp_path):
    f = _yaml(tmp_path)
    graph = _graph(_node(), _action(f))
    ids = build_node_identities(graph)
    (entry,) = _factory_entries(list(graph.nodes.items()), {"sub": "sub"}, ids)
    expected = ["-p", f"{QOS_KEY}:=best_effort", "--params-file", str(f)]
    cpp = emit_node_factories_cpp([entry])
    literal = ", ".join(f'"{a}"' for a in expected)
    assert literal in cpp, _dropped(expected, cpp)


def test_component_instance_carries_param_args(tmp_path):
    f = _yaml(tmp_path)
    graph = _graph(_node(component=True), _action(f))
    (inst,) = build_component_instances(graph, {"sub": "sub"}, build_node_identities(graph))
    expected = ["-p", f"{QOS_KEY}:=best_effort", "--params-file", str(f)]
    assert inst.get("param_args") == expected, _dropped(expected, inst.get("param_args", []))


def test_values_render_as_launch_ros_renders_them():
    from colcon_lf.generation.factory_emitter import param_args_for

    args = param_args_for(
        {"b": True, "i": 3, "f": 2.5, "s": "best_effort", "quoted": "true",
         "lst": ["a", "b"], "nested": {"inner": 1}},
        [],
    )
    rules = args[1::2]
    assert args[0::2] == ["-p"] * len(rules)
    assert rules == [
        "b:=true", "i:=3", "f:=2.5", "s:=best_effort", "quoted:='true'",
        "lst:=[a, b]", "nested.inner:=1",
    ]


def test_missing_parameter_file_is_dropped_with_a_warning(tmp_path, capsys):
    from colcon_lf.generation.factory_emitter import param_args_for

    missing = tmp_path / "absent.yaml"
    args = param_args_for({}, [missing], where="/sub")
    assert args == []
    err = capsys.readouterr().err
    assert "/sub" in err and str(missing) in err, err


def test_node_without_launch_parameters_keeps_its_arguments(tmp_path):
    graph = _graph(_node(), _action(_yaml(tmp_path), parameters={}))
    graph.launch_graph.all_nodes[0].parameter_files = []
    (entry,) = _factory_entries(list(graph.nodes.items()), {"sub": "sub"},
                                build_node_identities(graph))
    assert '"--ros-args", "-r", "__node:=sub"}' in emit_node_factories_cpp([entry])


def test_headerless_factory_materializes_inline_parameters(tmp_path):
    """A headerless node's inline dict reaches rclcpp as a params file.

    The ``--rcllf-params-yaml`` pair is the generator's own flag. Before the
    fix the headerless branch handed ``ros_args`` to ``NodeOptions`` as they
    were, so rclcpp aborted the node's construction.
    """
    f = _yaml(tmp_path)
    action = _action(f, parameters={})
    action.parameter_sources = [{"file": str(f)}, {"params": {"rate": 9}, "key": "/sub"}]
    graph = _graph(_node(), action)
    (entry,) = _factory_entries(list(graph.nodes.items()), {"sub": "sub"},
                                build_node_identities(graph))
    assert entry["header"] is None
    cpp = emit_node_factories_cpp([entry])
    assert '"--rcllf-params-yaml", "{/sub: {ros__parameters: {rate: 9}}}"' in cpp, cpp
    call = f'ros2_lf_make_node_{entry["idx"]}("sub", ros2_lf_node_options(ros_args));'
    assert call in cpp and "options.arguments(ros_args)" not in cpp, (
        "headerless node gets --rcllf-params-yaml unmaterialized; rclcpp would throw "
        "UnknownROSArgsError: found unknown ROS arguments: '--rcllf-params-yaml' "
        f":\n{cpp}"
    )
