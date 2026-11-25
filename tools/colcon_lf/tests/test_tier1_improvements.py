# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for Tier 1 improvements: condition evaluation, YAML overlay,
parameter default extraction, and argument forwarding.

All tests use in-memory objects only — no Docker, no libclang, no filesystem.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from colcon_lf.graph.hierarchical import (
    _apply_launch_parameters,
    _infer_param_type,
    launch_parameter_values,
)
from colcon_lf.launch_analysis.models import (
    LaunchArgument,
    LaunchNodeAction,
)
from colcon_lf.node_analysis.models import NodeInfo, ParameterInfo


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _node_info(name: str, parameters: list[ParameterInfo] | None = None) -> NodeInfo:
    return NodeInfo(
        name=name,
        class_name=name,
        package="test_pkg",
        source_file=Path("/dev/null"),
        parameters=parameters or [],
    )


def _launch_node(name: str, **kwargs: Any) -> LaunchNodeAction:
    return LaunchNodeAction(
        package="test_pkg",
        executable=name,
        name=name,
        **kwargs,
    )


# ===========================================================================
# _scope_condition on a node's condition (replaced _evaluate_condition)
# ===========================================================================

class TestNodeCondition:
    """A node's condition is evaluated by launch against the current scope."""

    @pytest.fixture(autouse=True)
    def _skip_if_no_launch(self):
        pytest.importorskip("launch", reason="launch package not available")

    @staticmethod
    def _evaluate(condition, scope):
        from types import SimpleNamespace
        from colcon_lf.launch_analysis.parser import _scope_condition
        return _scope_condition(SimpleNamespace(condition=condition), scope)[0]

    def test_no_condition_returns_true(self):
        assert self._evaluate(None, []) is True

    def test_if_and_unless(self):
        from launch.conditions import IfCondition, UnlessCondition
        from launch.substitutions import LaunchConfiguration
        use_sim = LaunchConfiguration("use_sim")
        assert self._evaluate(IfCondition(use_sim), [("use_sim", "true")]) is True
        assert self._evaluate(IfCondition(use_sim), [("use_sim", "false")]) is False
        assert self._evaluate(UnlessCondition(use_sim), [("use_sim", "true")]) is False
        assert self._evaluate(UnlessCondition(use_sim), [("use_sim", "false")]) is True

    def test_one_and_zero(self):
        from launch.conditions import IfCondition
        from launch.substitutions import LaunchConfiguration
        cond = IfCondition(LaunchConfiguration("flag"))
        assert self._evaluate(cond, [("flag", "1")]) is True
        assert self._evaluate(cond, [("flag", "0")]) is False

    def test_last_value_in_scope_wins(self):
        from launch.conditions import IfCondition
        from launch.substitutions import LaunchConfiguration
        cond = IfCondition(LaunchConfiguration("flag"))
        assert self._evaluate(cond, [("flag", "true"), ("flag", "false")]) is False

    def test_undecidable_returns_none(self):
        from launch.conditions import IfCondition
        from launch.substitutions import LaunchConfiguration
        # Unset, unknown, and values launch rejects ('yes' was accepted by the
        # legacy evaluator; launch raises on it).
        assert self._evaluate(IfCondition(LaunchConfiguration("x")), []) is None
        assert self._evaluate(IfCondition(LaunchConfiguration("x")), [("x", None)]) is None
        assert self._evaluate(IfCondition(LaunchConfiguration("x")), [("x", "yes")]) is None
        assert self._evaluate(IfCondition(LaunchConfiguration("x")),
                              [("x", "simulation")]) is None


# ===========================================================================
# _apply_launch_parameters tests (parameter files)
# ===========================================================================

def _files(contents: dict) -> dict:
    """The node's own parameter files as ``_apply_launch_parameters`` takes them."""
    return {key: (key, content) for key, content in contents.items()}


class TestOverlayYamlParameters:
    """Test YAML parameter value overlay onto NodeInfo."""

    def test_overlay_fills_missing_default(self):
        node = _node_info("my_node", [ParameterInfo(name="robot_name", type="string")])
        launch = _launch_node("my_node")
        yaml = {"params.yaml": {"my_node": {"ros__parameters": {"robot_name": "R2D2"}}}}
        _apply_launch_parameters([(launch, node)], [_files(yaml)])  # files passed to this node
        assert node.parameters[0].default_value == "R2D2"

    def test_overlay_overrides_cpp_default(self):
        # Maintainer decision P2: a params-file value must win
        # over a declared C++ default (ROS parameter precedence), not the
        # reverse. This test used to assert the inverted (buggy) behavior.
        node = _node_info("my_node", [
            ParameterInfo(name="robot_name", type="string", default_value="C3PO"),
        ])
        launch = _launch_node("my_node")
        yaml = {"params.yaml": {"my_node": {"ros__parameters": {"robot_name": "R2D2"}}}}
        _apply_launch_parameters([(launch, node)], [_files(yaml)])  # files passed to this node
        assert node.parameters[0].default_value == "R2D2"

    def test_overlay_wildcard_match(self):
        node = _node_info("any_node", [ParameterInfo(name="timeout", type="double")])
        launch = _launch_node("any_node")
        yaml = {"params.yaml": {"/**": {"ros__parameters": {"timeout": 5.0}}}}
        _apply_launch_parameters([(launch, node)], [_files(yaml)])  # files passed to this node
        assert node.parameters[0].default_value == 5.0

    def test_overlay_appends_yaml_only_param(self):
        node = _node_info("my_node", [])
        launch = _launch_node("my_node")
        yaml = {"params.yaml": {"my_node": {"ros__parameters": {"new_param": 42}}}}
        _apply_launch_parameters([(launch, node)], [_files(yaml)])  # files passed to this node
        assert len(node.parameters) == 1
        assert node.parameters[0].name == "new_param"
        assert node.parameters[0].default_value == 42
        assert node.parameters[0].type == "int"

    def test_overlay_empty_yaml(self):
        node = _node_info("my_node", [ParameterInfo(name="x", type="int")])
        launch = _launch_node("my_node")
        _apply_launch_parameters([(launch, node)], [{}])
        assert node.parameters[0].default_value is None

    def test_overlay_no_matching_node(self):
        node = _node_info("my_node", [ParameterInfo(name="x", type="int")])
        launch = _launch_node("my_node")
        yaml = {"params.yaml": {"other_node": {"ros__parameters": {"x": 99}}}}
        _apply_launch_parameters([(launch, node)], [_files(yaml)])  # files passed to this node
        assert node.parameters[0].default_value is None

    def test_overlay_ignores_class_name_key(self):
        # rcl matches a file's node key against the node's full name only;
        # a section named after the C++ class never reaches
        # /my_node in `ros2 launch`, so the analyzer must not apply it either.
        node = _node_info("my_node", [ParameterInfo(name="x", type="int")])
        node.class_name = "MyNodeClass"
        launch = _launch_node("my_node")
        yaml = {"params.yaml": {"MyNodeClass": {"ros__parameters": {"x": 7}}}}
        _apply_launch_parameters([(launch, node)], [_files(yaml)])  # files passed to this node
        assert node.parameters[0].default_value is None, (
            "a section keyed by the class name MyNodeClass was applied to /my_node"
        )


def _values(launch: LaunchNodeAction, node: NodeInfo, contents: dict) -> dict:
    """``launch_parameter_values`` over files given as ``{path: yaml content}``."""
    files = {path: (path, content) for path, content in contents.items()}
    return {k: v for k, (v, _role) in launch_parameter_values(launch, node, files).items()}


class TestLaunchParameterShapes:
    """YAML shapes and node names as rcl and launch_ros read them."""

    def test_nested_file_parameters_are_dotted(self):
        # rcl flattens nested maps under ros__parameters ({ctl: {gain: 1}} is
        # ctl.gain; checked with --params-file on a Jazzy node).
        launch = _launch_node("n", parameter_sources=[{"file": "/cfg/a.yaml"}])
        got = _values(launch, _node_info("n"), {"/cfg/a.yaml": {
            "/**": {"ros__parameters": {"ctl": {"gain": 1, "inner": {"k": 2}}}}}})
        assert got == {"ctl.gain": 1, "ctl.inner.k": 2}, (
            f"nested parameters kept as one dict-valued parameter: {got}"
        )

    def test_nested_inline_parameters_are_dotted(self):
        launch = _launch_node("n", parameter_sources=[
            {"params": {"ctl": {"gain": 3}}, "key": "/**"}])
        got = _values(launch, _node_info("n"), {})
        assert got == {"ctl.gain": 3}, f"nested inline dict not flattened: {got}"

    def test_namespace_nested_section_selects_the_node(self):
        # `ns2: {node_d: {ros__parameters: ...}}` is the section ns2/node_d
        # (checked: it sets /ns2/node_d under `ros2 run --params-file`).
        launch = _launch_node("node_d", namespace="/ns2",
                              parameter_sources=[{"file": "/cfg/a.yaml"}])
        got = _values(launch, _node_info("node_d"), {"/cfg/a.yaml": {
            "ns2": {"node_d": {"ros__parameters": {"robot_name": "nested_ns"}}}}})
        assert got == {"robot_name": "nested_ns"}, (
            f"namespace-nested section ns2/node_d not applied to /ns2/node_d: {got}"
        )

    def test_inline_yaml_string_is_not_a_file_source(self):
        # launch_ros loads the file entry once and then the dict; the dict's
        # string value naming a .yaml file is just a string parameter.
        launch = _launch_node("node_g", namespace="/fq", parameter_sources=[
            {"file": "/cfg/g.yaml"},
            {"params": {"config_file": "/cfg/g.yaml", "update_rate": 9}, "key": "/fq/node_g"}])
        got = _values(launch, _node_info("node_g"), {"/cfg/g.yaml": {
            "/fq/node_g": {"ros__parameters": {"robot_name": "from_file", "update_rate": 7}}}})
        assert got == {"robot_name": "from_file", "config_file": "/cfg/g.yaml",
                       "update_rate": 9}, (
            f"inline .yaml string replayed as a parameter file after the dict: {got}"
        )

    def test_slash_and_bare_key_are_separate_sections(self):
        # rcl keeps `node_d` and `/node_d` apart, each at its first
        # appearance; measured on parameter_node: robot_name f2_wild, update_rate 3.
        launch = _launch_node("node_d", parameter_sources=[
            {"file": "/cfg/f1.yaml"}, {"file": "/cfg/f2.yaml"}, {"file": "/cfg/f3.yaml"}])
        got = _values(launch, _node_info("node_d"), {
            "/cfg/f1.yaml": {"node_d": {"ros__parameters": {"robot_name": "f1_plain"}}},
            "/cfg/f2.yaml": {"/**": {"ros__parameters": {"robot_name": "f2_wild",
                                                        "update_rate": 2}}},
            "/cfg/f3.yaml": {"/node_d": {"ros__parameters": {"update_rate": 3}}},
        })
        assert got == {"robot_name": "f2_wild", "update_rate": 3}

    def test_node_remap_without_launch_name_renames_the_node(self):
        # launch_ros passes `remappings` as -r rules; with no name= the
        # `__node` rule is the first match, so rcl matches /renamed.
        launch = LaunchNodeAction(package="p", executable="exe", name=None,
                                  remappings=[("__node", "renamed")],
                                  parameter_sources=[{"file": "/cfg/a.yaml"}])
        got = _values(launch, _node_info("orig"), {"/cfg/a.yaml": {
            "orig": {"ros__parameters": {"x": 1}},
            "renamed": {"ros__parameters": {"y": 2}}}})
        assert got == {"y": 2}, f"sections matched against the code name, not /renamed: {got}"

    def test_node_remap_after_launch_name_is_ignored(self):
        # launch_ros puts `-r __node:=<name>` before the remappings, and rcl
        # takes the first match: name= wins.
        launch = _launch_node("given", remappings=[("__node", "renamed")],
                              parameter_sources=[{"file": "/cfg/a.yaml"}])
        got = _values(launch, _node_info("orig"), {"/cfg/a.yaml": {
            "given": {"ros__parameters": {"x": 1}},
            "renamed": {"ros__parameters": {"y": 2}}}})
        assert got == {"x": 1}

    def test_node_remap_in_arguments_beats_launch_name(self):
        # `arguments` come before launch_ros' own `-r __node:=<name>`.
        launch = _launch_node("given", arguments=["--ros-args", "-r", "__node:=renamed"],
                              parameter_sources=[{"file": "/cfg/a.yaml"}])
        got = _values(launch, _node_info("orig"), {"/cfg/a.yaml": {
            "given": {"ros__parameters": {"x": 1}},
            "renamed": {"ros__parameters": {"y": 2}}}})
        assert got == {"y": 2}

    def test_component_reads_nested_sections_like_launch_ros(self):
        launch = _launch_node("comp", namespace="/ns2", is_component=True,
                              parameter_sources=[{"file": "/cfg/a.yaml"}])
        got = _values(launch, _node_info("comp"), {"/cfg/a.yaml": {
            "ns2": {"comp": {"ros__parameters": {"ctl": {"gain": 4}}}}}})
        assert got == {"ctl.gain": 4}

    def test_unnamed_component_gets_no_file_sections(self):
        # load_composable_nodes passes request.node_name, "" without name=,
        # and to_parameters_list then matches no section.
        launch = LaunchNodeAction(package="p", executable="Comp", name=None,
                                  is_component=True,
                                  parameter_sources=[{"file": "/cfg/a.yaml"},
                                                     {"params": {"z": 5}}])
        got = _values(launch, _node_info("comp"), {"/cfg/a.yaml": {
            "/**": {"ros__parameters": {"x": 1}}}})
        assert got == {"z": 5}


class TestComponentParamArgsNeedsLaunchRos:
    def test_missing_launch_ros_raises_naming_the_import(self, monkeypatch):
        import sys

        from colcon_lf.generation.factory_emitter import component_param_args

        # Generation needs launch_ros; no silent fallback with another order.
        monkeypatch.setitem(sys.modules, "launch_ros.utilities", None)
        with pytest.raises(RuntimeError,
                           match=r"import of '(launch|launch_ros\.utilities)' failed.*Source ROS 2"):
            component_param_args([{"params": {"x": 1}}], "comp", "/ns", where="pkg/comp")


# ===========================================================================
# _infer_param_type tests
# ===========================================================================

class TestInferParamType:
    def test_bool(self):
        assert _infer_param_type(True) == "bool"

    def test_int(self):
        assert _infer_param_type(42) == "int"

    def test_float(self):
        assert _infer_param_type(3.14) == "double"

    def test_string(self):
        assert _infer_param_type("hello") == "string"

    def test_list(self):
        assert _infer_param_type([1, 2, 3]) == "array"

    def test_none(self):
        assert _infer_param_type(None) == ""


# ===========================================================================
# Serializer round-trip with condition_result
# ===========================================================================

class TestConditionResultSerialization:
    """Verify condition_result survives JSON round-trip."""

    def _roundtrip(self, node: LaunchNodeAction) -> LaunchNodeAction:
        import json
        from colcon_lf.output.serializer import serialize_to_dict, _from_dict
        data = serialize_to_dict(node)
        return _from_dict(data, LaunchNodeAction)

    def test_roundtrip_condition_result_true(self):
        node = LaunchNodeAction(
            package="pkg", executable="exe", name="n",
            condition="IfCondition(use_sim)", condition_result=True,
        )
        restored = self._roundtrip(node)
        assert restored.condition_result is True

    def test_roundtrip_condition_result_false(self):
        node = LaunchNodeAction(
            package="pkg", executable="exe", name="n",
            condition="IfCondition(use_sim)", condition_result=False,
        )
        restored = self._roundtrip(node)
        assert restored.condition_result is False

    def test_roundtrip_condition_result_none(self):
        node = LaunchNodeAction(
            package="pkg", executable="exe", name="n",
        )
        restored = self._roundtrip(node)
        assert restored.condition_result is None

    def test_backward_compat_missing_field(self):
        """Old JSON without condition_result should deserialize with None."""
        from colcon_lf.output.serializer import _from_dict
        data = {
            "package": "pkg", "executable": "exe", "name": "n",
            "parameters": {}, "parameter_files": [], "remappings": [],
            "condition": None, "is_component": False,
            "composable_container": None, "environment": {},
            "output": None, "respawn": False, "respawn_delay": None,
            "arguments": [],
        }
        restored = _from_dict(data, LaunchNodeAction)
        assert restored.condition_result is None
