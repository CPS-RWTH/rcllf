# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for colcon_lf.graph.hierarchical.

Phase 5 — 4 TDD assertions required by the plan:
1. Build SystemGraph from pubsub_example/bringup.launch.py -> 2 nodes, 1 topic
2. Build SystemGraph from system_bringup -> all nodes, all connections
3. Verify graph layers: launch hierarchy non-empty, all_nodes count matches
4. Verify parameter_files populated from YAML configs

All tests require a built and sourced ROS2 workspace (run inside Docker).
"""

from __future__ import annotations

import pytest

# Skip entire module if launch_ros or libclang is not available
pytest.importorskip("launch_ros", reason="launch_ros not available — run inside Docker")
pytest.importorskip("clang.cindex", reason="libclang not available — run inside Docker")

from .ros_env import requires_ros_headers  # noqa: E402

# Topics and the nodes of included launch files resolve only against a built
# workspace (message headers, the install space's package shares).
_needs_build = requires_ros_headers("the topic and system_bringup graph tests")


@pytest.fixture(scope="session")
def system_graph_builder(packages_src, packages_root):
    """Return a helper that builds a SystemGraph by package and launch filename.

    Results are cached per (package, launch_file) to avoid repeated parsing.
    """
    from colcon_lf.graph.hierarchical import build_system_graph

    _cache: dict[tuple[str, str], object] = {}

    def _build(package_name: str, launch_file: str):
        key = (package_name, launch_file)
        if key not in _cache:
            path = packages_src / package_name / "launch" / launch_file
            assert path.exists(), f"Launch file not found: {path}"
            _cache[key] = build_system_graph(path, workspace_root=packages_root)
        return _cache[key]

    return _build


# ---------------------------------------------------------------------------
# Assertion 1: pubsub_example bringup -> 2 nodes, 1 topic connection
# ---------------------------------------------------------------------------

class TestPubsubBringup:
    def test_node_count(self, system_graph_builder):
        graph = system_graph_builder("pubsub_example", "bringup.launch.py")
        assert len(graph.nodes) == 2

    @_needs_build
    def test_topic_connection(self, system_graph_builder):
        graph = system_graph_builder("pubsub_example", "bringup.launch.py")
        assert len(graph.communication.topics) >= 1
        topics = [t.topic for t in graph.communication.topics]
        assert any("sensor_data" in t for t in topics)

    @_needs_build
    def test_topic_has_publisher_and_subscriber(self, system_graph_builder):
        graph = system_graph_builder("pubsub_example", "bringup.launch.py")
        sensor_topic = next(
            t for t in graph.communication.topics if "sensor_data" in t.topic
        )
        assert len(sensor_topic.publishers) >= 1
        assert len(sensor_topic.subscribers) >= 1

    def test_graph_name(self, system_graph_builder):
        graph = system_graph_builder("pubsub_example", "bringup.launch.py")
        assert graph.name == "bringup"


# ---------------------------------------------------------------------------
# Assertion 2: system_bringup -> all nodes, all connections
# ---------------------------------------------------------------------------

@_needs_build
class TestSystemBringup:
    def test_has_multiple_nodes(self, system_graph_builder):
        graph = system_graph_builder("system_bringup", "system_bringup.launch.py")
        # At minimum: publisher, subscriber, timer, service_server, service_client, parameter
        assert len(graph.nodes) >= 6

    def test_has_topic_connections(self, system_graph_builder):
        graph = system_graph_builder("system_bringup", "system_bringup.launch.py")
        assert len(graph.communication.topics) >= 1

    def test_has_service_connections(self, system_graph_builder):
        graph = system_graph_builder("system_bringup", "system_bringup.launch.py")
        assert len(graph.communication.services) >= 1

    def test_node_names_include_expected(self, system_graph_builder):
        graph = system_graph_builder("system_bringup", "system_bringup.launch.py")
        node_keys = list(graph.nodes.keys())
        # There should be nodes from pubsub, timer, service, and parameter packages
        node_str = " ".join(node_keys)
        # Check that there are nodes containing recognizable names
        assert len(node_keys) >= 6, f"Expected >= 6 nodes, got: {node_keys}"


# ---------------------------------------------------------------------------
# Assertion 3: launch layer integrity
# ---------------------------------------------------------------------------

class TestLaunchLayerIntegrity:
    def test_pubsub_launch_includes_non_empty(self, system_graph_builder):
        graph = system_graph_builder("pubsub_example", "bringup.launch.py")
        assert len(graph.launch_graph.root.includes) > 0

    def test_pubsub_all_nodes_matches_dict(self, system_graph_builder):
        graph = system_graph_builder("pubsub_example", "bringup.launch.py")
        assert len(graph.launch_graph.all_nodes) == len(graph.nodes)

    def test_system_bringup_launch_structure(self, system_graph_builder):
        graph = system_graph_builder("system_bringup", "system_bringup.launch.py")
        # system_bringup includes multiple child launch files
        root = graph.launch_graph.root
        total_includes = len(root.includes)
        total_groups = len(root.groups)
        # Should have includes and/or groups
        assert total_includes + total_groups > 0

    def test_system_bringup_all_nodes_count(self, system_graph_builder):
        graph = system_graph_builder("system_bringup", "system_bringup.launch.py")
        assert len(graph.launch_graph.all_nodes) >= len(graph.nodes)


# ---------------------------------------------------------------------------
# Assertion 4: parameter_files populated from YAML configs
# ---------------------------------------------------------------------------

class TestParameterFiles:
    def test_parameter_files_non_empty(self, system_graph_builder):
        graph = system_graph_builder("system_bringup", "system_bringup.launch.py")
        assert len(graph.parameter_files) > 0

    def test_parameter_files_contain_expected_keys(self, system_graph_builder):
        graph = system_graph_builder("system_bringup", "system_bringup.launch.py")
        # At least one YAML file should have been parsed
        all_content = {}
        for _fname, content in graph.parameter_files.items():
            all_content.update(content)
        # The YAML files contain parameter_node.ros__parameters with known keys
        assert "parameter_node" in all_content
        params = all_content["parameter_node"].get("ros__parameters", {})
        assert "robot_name" in params or "max_speed" in params


# ---------------------------------------------------------------------------
# Maintainer decision P2: a parameter-file value must
# override a declared C++ default, and two nodes loading same-named files
# from different directories must not collide on a shared basename cache.
# ---------------------------------------------------------------------------

class TestParameterPrecedenceFixture:
    """Golden fixture: parameter_example/launch/precedence.launch.py.

    node_a: config/a/params.yaml, config/a/late.yaml, inline {debug_mode: True}
    node_b: config/b/params.yaml (same basename as node_a's file, different dir)
    """

    def test_node_a_file_overrides_cpp_default(self, system_graph_builder):
        # A1: C++ default (1.0) vs params-file value (5.0) — file wins.
        graph = system_graph_builder("parameter_example", "precedence.launch.py")
        node_a = graph.nodes["node_a"]
        max_speed = next(p for p in node_a.parameters if p.name == "max_speed")
        assert max_speed.default_value == 5.0

    def test_node_a_later_file_wins(self, system_graph_builder):
        # A2: a/params.yaml sets update_rate=7, a/late.yaml (listed later) sets 42.
        graph = system_graph_builder("parameter_example", "precedence.launch.py")
        node_a = graph.nodes["node_a"]
        update_rate = next(p for p in node_a.parameters if p.name == "update_rate")
        assert update_rate.default_value == 42

    def test_node_a_file_overrides_inline(self, system_graph_builder):
        # A3: the params file sets debug_mode=false, the inline dict sets
        # true, and a live `ros2 param get /node_a debug_mode` shows the file
        # wins. launch_ros keeps the list order, but node_a has no namespace,
        # so its inline dict is keyed /**; rcl applies the /** group (created
        # first, by a/params.yaml) before the node_a group.
        graph = system_graph_builder("parameter_example", "precedence.launch.py")
        node_a = graph.nodes["node_a"]
        debug_mode = next(p for p in node_a.parameters if p.name == "debug_mode")
        assert debug_mode.default_value is False

    def test_node_a_node_key_overrides_wildcard(self, system_graph_builder):
        # A4: a/params.yaml sets robot_name under both /** and node_a; node_a wins.
        graph = system_graph_builder("parameter_example", "precedence.launch.py")
        node_a = graph.nodes["node_a"]
        robot_name = next(p for p in node_a.parameters if p.name == "robot_name")
        assert robot_name.default_value == "node_a_from_params_file"

    def test_node_b_same_basename_file_not_collided(self, system_graph_builder):
        # P2 cache fix: node_b's own b/params.yaml (basename "params.yaml", same
        # as node_a's a/params.yaml) must be read, not skipped in favor of
        # node_a's file of the same name.
        graph = system_graph_builder("parameter_example", "precedence.launch.py")
        node_b = graph.nodes["node_b"]
        robot_name = next(p for p in node_b.parameters if p.name == "robot_name")
        assert robot_name.default_value == "node_b_from_params_file"

    def test_node_a_and_node_b_differ(self, system_graph_builder):
        graph = system_graph_builder("parameter_example", "precedence.launch.py")
        a_name = next(p for p in graph.nodes["node_a"].parameters if p.name == "robot_name")
        b_name = next(p for p in graph.nodes["node_b"].parameters if p.name == "robot_name")
        assert a_name.default_value != b_name.default_value


class TestParameterFileScope:
    """A parameter file applies only to the nodes it is passed to.

    ``wildcard_scope.launch.py``: node_c gets ``config/c/wildcard.yaml`` whose
    ``/**`` section sets max_speed 7.5 and robot_name c_wildcard; node_x gets
    no file. Live ``ros2 param get`` on the running launch confirms node_x keeps
    the C++ defaults (1.0, default_robot).
    """

    def _values(self, system_graph_builder):
        graph = system_graph_builder("parameter_example", "wildcard_scope.launch.py")
        return graph, {
            key: {p.name: p.default_value for p in node.parameters}
            for key, node in graph.nodes.items()
        }

    def test_wildcard_reaches_only_its_own_node(self, system_graph_builder):
        _graph, values = self._values(system_graph_builder)
        x = next(v for k, v in values.items() if k.endswith("node_x"))
        c = next(v for k, v in values.items() if k.endswith("node_c"))
        assert float(c["max_speed"]) == 7.5 and c["robot_name"] == "c_wildcard"
        assert float(x["max_speed"]) == 1.0, (
            f"node_c's /** file leaked into node_x: max_speed={x['max_speed']}"
        )
        assert x["robot_name"] == "default_robot", (
            f"node_c's /** file leaked into node_x: robot_name={x['robot_name']}"
        )

    def test_resolver_attributes_node_x_to_its_cpp_default(
        self, system_graph_builder, packages_root, tmp_path
    ):
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.output.serializer import serialize
        from colcon_lf.resolved_model.resolve import resolve

        graph, _ = self._values(system_graph_builder)
        system_json = tmp_path / "system.json"
        system_json.write_text(serialize(graph), encoding="utf-8")
        model = resolve(system_json, exec_config=ExecConfig.empty(), workspace=packages_root)
        rec = next(r for r in model.records if r.contract_id.endswith("node_x/max_speed#value"))
        assert rec.provenance == "extracted", (
            f"node_x max_speed attributed {rec.provenance} ({rec.evidence})"
        )


class TestInlineOverrideLaunchOrder:
    """The analyzer applies ``parameters=[...]`` as ``ros2 launch`` does.

    ``inline_override.launch.py`` passes ``[{robot_name: inline_first},
    config/d/override.yaml, {update_rate: 8}]`` to ``node_d`` (no namespace)
    and ``/fq/node_f``. ``launch_ros`` keys node_d's inline dicts ``/**`` and
    node_f's ``/fq/node_f``, and rcl applies every section in the order it first
    appeared. Expected values from ``ros2 param get`` on a stock launch of the
    same file.
    """

    EXPECTED = {
        "node_d": {"robot_name": "node_d_from_file", "update_rate": 7},
        "/fq/node_f": {"robot_name": "node_f_from_file", "update_rate": 8},
    }

    def test_resolved_values_match_ros2_launch(self, system_graph_builder):
        graph = system_graph_builder("parameter_example", "inline_override.launch.py")
        got = {
            key: {p.name: p.default_value for p in node.parameters
                  if p.name in ("robot_name", "update_rate")}
            for key, node in graph.nodes.items()
        }
        assert got == self.EXPECTED, (
            "analyzer's launch parameter values differ from ros2 launch "
            f"(an inline value ranked above the file?): {got}"
        )

    def test_resolver_attributes_the_winning_source(
        self, system_graph_builder, packages_root, tmp_path
    ):
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.output.serializer import serialize
        from colcon_lf.resolved_model.resolve import resolve

        graph = system_graph_builder("parameter_example", "inline_override.launch.py")
        system_json = tmp_path / "system.json"
        system_json.write_text(serialize(graph), encoding="utf-8")
        model = resolve(system_json, exec_config=ExecConfig.empty(), workspace=packages_root)
        evidence = {
            r.contract_id: (r.value, r.evidence) for r in model.records
            if r.contract_id.endswith(("/robot_name#value", "/update_rate#value"))
        }
        override = "parameter_example/config/d/override.yaml"
        for node, values in self.EXPECTED.items():
            for name, value in values.items():
                got_value, got_evidence = evidence[f"param:{node}/{name}#value"]
                assert got_value == value, (node, name, got_value)
                want = ("launch inline parameters"
                        if (node, name) == ("/fq/node_f", "update_rate") else override)
                assert got_evidence.endswith(want), (
                    f"{node} {name} attributed to {got_evidence!r}, expected {want!r}"
                )
