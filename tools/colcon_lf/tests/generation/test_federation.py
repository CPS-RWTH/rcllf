# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for federation support: partition resolution, template emission, emitter."""

from __future__ import annotations

import json
import re
import textwrap
from pathlib import Path

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection
from colcon_lf.federation.partition import (
    DEFAULT_CROSS_FEDERATE_AFTER_NS,
    DEFAULT_STP_OFFSET_NS,
    AfterOverride,
    FederationMap,
    _per_node,
    _per_process,
    resolve_partition,
)
from colcon_lf.federation.yaml_schema import MODE_PER_NODE, MODE_PER_PROCESS
from colcon_lf.generation.lf_emitter import emit_federated_top_level, emit_lf_system
from colcon_lf.generation.lf_model import LFConnection, LFSystem
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph, LaunchNodeAction
from colcon_lf.node_analysis.models import NodeInfo, TimerInfo, CallbackInfo, PublisherInfo, SubscriberInfo


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_node(
    name: str,
    class_name: str | None = None,
    src_dir=None,
) -> NodeInfo:
    """Build a NodeInfo for federation tests.

    ``src_dir`` materializes a real source file and points ``source_file`` at
    it.  Tests that drive the full generator (which stages node sources and
    fails closed on an unresolvable path) must pass it; tests that only
    exercise partitioning/emission can keep the non-existent placeholder path.
    """
    if src_dir is not None:
        src_dir.mkdir(parents=True, exist_ok=True)
        real_src = src_dir / f"{name}.cpp"
        real_src.write_text(f"// stand-in body for {name}\n", encoding="utf-8")
        source_file = str(real_src)
    else:
        source_file = f"/fake/{name}.cpp"
    return NodeInfo(
        name=name,
        class_name=class_name or name.replace("_", " ").title().replace(" ", ""),
        package="test_pkg",
        source_file=source_file,
    )


def _make_launch_graph(all_nodes: list | None = None) -> LaunchGraph:
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return LaunchGraph(root=root, all_nodes=all_nodes or [])


def _make_graph(
    node_names: list[str],
    launch_nodes: list | None = None,
    src_dir=None,
) -> SystemGraph:
    nodes = {n: _make_node(n, src_dir=src_dir) for n in node_names}
    comm = SystemCommunication()
    launch_graph = _make_launch_graph(launch_nodes)
    return SystemGraph(
        name="test_system",
        launch_graph=launch_graph,
        nodes=nodes,
        communication=comm,
    )


# ---------------------------------------------------------------------------
# FederationMap
# ---------------------------------------------------------------------------

class TestFederationMap:
    def test_federate_for(self):
        fmap = FederationMap(
            node_to_federate={"talker": "proc_a", "listener": "proc_b"},
            federate_names=["proc_a", "proc_b"],
        )
        assert fmap.federate_for("talker") == "proc_a"
        assert fmap.federate_for("listener") == "proc_b"
        assert fmap.federate_for("missing") is None

    def test_nodes_for(self):
        fmap = FederationMap(
            node_to_federate={"talker": "proc_a", "listener": "proc_b", "relay": "proc_a"},
            federate_names=["proc_a", "proc_b"],
        )
        assert sorted(fmap.nodes_for("proc_a")) == ["relay", "talker"]
        assert fmap.nodes_for("proc_b") == ["listener"]

    def test_after_ns_default(self):
        fmap = FederationMap()
        assert fmap.after_ns_for("a.out", "b.inp") == DEFAULT_CROSS_FEDERATE_AFTER_NS


# ---------------------------------------------------------------------------
# Per-node partitioning
# ---------------------------------------------------------------------------

class TestPerNodePartition:
    def test_each_node_gets_own_federate(self):
        graph = _make_graph(["talker", "listener"])
        fmap = _per_node(graph)
        assert len(fmap.federate_names) == 2
        assert fmap.federate_for("talker") != fmap.federate_for("listener")

    def test_federate_name_sanitized(self):
        graph = _make_graph(["my/node"])
        fmap = _per_node(graph)
        # sanitize_lf_identifier should strip slashes
        for name in fmap.federate_names:
            assert "/" not in name

    def test_three_nodes(self):
        graph = _make_graph(["a", "b", "c"])
        fmap = _per_node(graph)
        assert len(fmap.federate_names) == 3
        assert set(fmap.node_to_federate.keys()) == {"a", "b", "c"}


# ---------------------------------------------------------------------------
# Per-process partitioning
# ---------------------------------------------------------------------------

class TestPerProcessPartition:
    def test_standalone_nodes_each_get_own_federate(self):
        # Non-component nodes (no composable_container) → each gets its own federate
        graph = _make_graph(["talker", "listener"])
        fmap = _per_process(graph)
        assert len(fmap.federate_names) == 2
        assert fmap.federate_for("talker") != fmap.federate_for("listener")

    def test_component_nodes_share_container_federate(self):
        # Two composable nodes in the same container → one federate
        ln_a = LaunchNodeAction(
            package="test_pkg", executable="TestPkg::Talker",
            name="talker", is_component=True, composable_container="my_container"
        )
        ln_b = LaunchNodeAction(
            package="test_pkg", executable="TestPkg::Listener",
            name="listener", is_component=True, composable_container="my_container"
        )
        # Make nodes with matching class_name
        nodes_info = {
            "talker": NodeInfo(name="talker", class_name="Talker", package="test_pkg", source_file="/fake/talker.cpp"),
            "listener": NodeInfo(name="listener", class_name="Listener", package="test_pkg", source_file="/fake/listener.cpp"),
        }
        lg = _make_launch_graph([ln_a, ln_b])
        graph = SystemGraph(name="test", launch_graph=lg, nodes=nodes_info,
                            communication=SystemCommunication())
        fmap = _per_process(graph)
        # Both in same container → same federate
        assert fmap.federate_for("talker") == fmap.federate_for("listener")

    def test_falls_back_to_per_node_when_no_launch_graph(self):
        graph = _make_graph(["talker", "listener"])
        fmap = _per_process(graph)
        assert len(fmap.federate_names) == 2


# ---------------------------------------------------------------------------
# resolve_partition dispatch
# ---------------------------------------------------------------------------

class TestResolvePartition:
    def test_per_node_dispatch(self):
        graph = _make_graph(["a", "b"])
        fmap = resolve_partition(MODE_PER_NODE, graph)
        assert len(fmap.federate_names) == 2

    def test_per_process_dispatch(self):
        # Without composable containers, falls back to per-node behaviour
        graph = _make_graph(["a", "b"])
        fmap = resolve_partition(MODE_PER_PROCESS, graph)
        assert len(fmap.federate_names) == 2
        assert fmap.federate_for("a") != fmap.federate_for("b")

    def test_yaml_not_found_raises(self):
        graph = _make_graph(["a"])
        with pytest.raises(FileNotFoundError):
            resolve_partition("/nonexistent/path.yaml", graph)

    def test_yaml_valid(self, tmp_path):
        yaml_file = tmp_path / "fed.yaml"
        yaml_file.write_text(textwrap.dedent("""\
            system: test
            federates:
              perception:
                nodes:
                  - talker
              control:
                nodes:
                  - listener
            after_overrides: []
        """), encoding="utf-8")
        graph = _make_graph(["talker", "listener"])
        fmap = resolve_partition(str(yaml_file), graph)
        assert fmap.federate_for("talker") == "perception"
        assert fmap.federate_for("listener") == "control"

    def test_yaml_duplicate_node_raises(self, tmp_path):
        yaml_file = tmp_path / "fed.yaml"
        yaml_file.write_text(textwrap.dedent("""\
            system: test
            federates:
              a:
                nodes:
                  - talker
              b:
                nodes:
                  - talker
            after_overrides: []
        """), encoding="utf-8")
        graph = _make_graph(["talker"])
        with pytest.raises(ValueError, match="appears in both"):
            resolve_partition(str(yaml_file), graph)

    def test_yaml_missing_node_raises(self, tmp_path):
        yaml_file = tmp_path / "fed.yaml"
        yaml_file.write_text(textwrap.dedent("""\
            system: test
            federates:
              a:
                nodes:
                  - talker
            after_overrides: []
        """), encoding="utf-8")
        graph = _make_graph(["talker", "listener"])
        with pytest.raises(ValueError, match="not assigned to any federate"):
            resolve_partition(str(yaml_file), graph)

    def test_yaml_unknown_node_raises(self, tmp_path):
        yaml_file = tmp_path / "fed.yaml"
        yaml_file.write_text(textwrap.dedent("""\
            system: test
            federates:
              a:
                nodes:
                  - nonexistent_node
            after_overrides: []
        """), encoding="utf-8")
        graph = _make_graph(["talker"])
        with pytest.raises(ValueError, match="not in system.json"):
            resolve_partition(str(yaml_file), graph)

    def test_yaml_invalid_federate_name_raises(self, tmp_path):
        yaml_file = tmp_path / "fed.yaml"
        yaml_file.write_text(textwrap.dedent("""\
            system: test
            federates:
              "bad name":
                nodes:
                  - talker
            after_overrides: []
        """), encoding="utf-8")
        graph = _make_graph(["talker"])
        with pytest.raises(ValueError, match="not a valid identifier"):
            resolve_partition(str(yaml_file), graph)

    def test_yaml_after_overrides_parsed(self, tmp_path):
        yaml_file = tmp_path / "fed.yaml"
        yaml_file.write_text(textwrap.dedent("""\
            system: test
            federates:
              a:
                nodes:
                  - talker
              b:
                nodes:
                  - listener
            after_overrides:
              - from: talker.out
                to: listener.in
                after: 10ms
        """), encoding="utf-8")
        graph = _make_graph(["talker", "listener"])
        fmap = resolve_partition(str(yaml_file), graph)
        assert fmap.after_ns_for("talker.out", "listener.in") == 10_000_000

    # -- federation YAML shares the generator's duration grammar --

    @staticmethod
    def _write_override(tmp_path, after_literal):
        yaml_file = tmp_path / "fed.yaml"
        yaml_file.write_text(textwrap.dedent(f"""\
            system: test
            federates:
              a:
                nodes:
                  - talker
              b:
                nodes:
                  - listener
            after_overrides:
              - from: talker.out
                to: listener.in
                after: {after_literal}
        """), encoding="utf-8")
        return yaml_file

    @pytest.mark.parametrize("literal,expected", [
        # Terse spelling: still accepted.
        ("5ms", 5_000_000),
        ("100ns", 100),
        ("1us", 1_000),
        ("2s", 2_000_000_000),
        # LF spelling: once rejected here, even though --after-delay and
        # execution.yaml have always taken it.
        ("5 msec", 5_000_000),
        ("500 usec", 500_000),
        ("2 sec", 2_000_000_000),
        ("1 nsec", 1),
        # A bare zero needs no unit in either grammar.
        ("0", 0),
    ])
    def test_yaml_after_override_spellings(self, tmp_path, literal, expected):
        yaml_file = self._write_override(tmp_path, literal)
        graph = _make_graph(["talker", "listener"])
        fmap = resolve_partition(str(yaml_file), graph)
        assert fmap.after_ns_for("talker.out", "listener.in") == expected

    def test_yaml_after_override_bare_number_rejected(self, tmp_path):
        """A unitless non-zero value used to mean nanoseconds. It is now an error.

        Guessing a unit is the failure mode that made --after-delay a dead
        parameter, so federation YAML stops guessing too.
        """
        yaml_file = self._write_override(tmp_path, "500")
        graph = _make_graph(["talker", "listener"])
        with pytest.raises(ValueError, match="needs a unit"):
            resolve_partition(str(yaml_file), graph)

    def test_yaml_after_override_empty_rejected(self, tmp_path):
        yaml_file = self._write_override(tmp_path, '""')
        graph = _make_graph(["talker", "listener"])
        with pytest.raises(ValueError, match="must not be empty"):
            resolve_partition(str(yaml_file), graph)

    def test_yaml_after_override_unknown_unit_rejected(self, tmp_path):
        yaml_file = self._write_override(tmp_path, "1 fortnight")
        graph = _make_graph(["talker", "listener"])
        with pytest.raises(ValueError, match="unknown time unit"):
            resolve_partition(str(yaml_file), graph)

    def test_yaml_after_override_error_names_yaml_path(self, tmp_path):
        """The YAML path is the useful half of the message; delegation kept it."""
        yaml_file = self._write_override(tmp_path, "garbage")
        graph = _make_graph(["talker", "listener"])
        with pytest.raises(ValueError, match=re.escape(str(yaml_file))):
            resolve_partition(str(yaml_file), graph)

    def test_yaml_empty_federate_raises(self, tmp_path):
        yaml_file = tmp_path / "fed.yaml"
        yaml_file.write_text(textwrap.dedent("""\
            system: test
            federates:
              a:
                nodes: []
            after_overrides: []
        """), encoding="utf-8")
        graph = _make_graph(["talker"])
        with pytest.raises(ValueError, match="has no nodes"):
            resolve_partition(str(yaml_file), graph)


# ---------------------------------------------------------------------------
# Template emission
# ---------------------------------------------------------------------------

class TestFederationTemplate:
    def test_template_written(self, tmp_path):
        from colcon_lf.federation.template import emit_federation_template
        system_json = tmp_path / "bringup" / "system.json"
        system_json.parent.mkdir()
        system_json.write_text("{}")
        graph = _make_graph(["talker", "listener"])
        rc = emit_federation_template(graph, system_json)
        assert rc == 0
        out = tmp_path / "bringup" / "federation.yaml"
        assert out.exists()
        content = out.read_text()
        assert "talker" in content
        assert "listener" in content
        assert "federates:" in content
        assert "after_overrides: []" in content

    def test_template_refuses_overwrite_without_force(self, tmp_path):
        from colcon_lf.federation.template import emit_federation_template
        system_json = tmp_path / "bringup" / "system.json"
        system_json.parent.mkdir()
        system_json.write_text("{}")
        out = tmp_path / "bringup" / "federation.yaml"
        out.write_text("existing")
        graph = _make_graph(["talker"])
        rc = emit_federation_template(graph, system_json, force=False)
        assert rc == 1
        assert out.read_text() == "existing"

    def test_template_overwrites_with_force(self, tmp_path):
        from colcon_lf.federation.template import emit_federation_template
        system_json = tmp_path / "bringup" / "system.json"
        system_json.parent.mkdir()
        system_json.write_text("{}")
        out = tmp_path / "bringup" / "federation.yaml"
        out.write_text("existing")
        graph = _make_graph(["talker"])
        rc = emit_federation_template(graph, system_json, force=True)
        assert rc == 0
        assert out.read_text() != "existing"


# ---------------------------------------------------------------------------
# Federated top-level emitter
# ---------------------------------------------------------------------------

class TestEmitFederatedTopLevel:
    def test_federated_keyword(self):
        text = emit_federated_top_level(
            system_name="Bringup",
            federate_names=["talker", "listener"],
            cross_connections=[],
        )
        assert "federated reactor Bringup" in text

    def test_cross_connections_emitted(self):
        conn = LFConnection(
            source="talker.publisher_node.sensor_data",
            destination="listener.listener_node.sensor_data",
            is_cross_federate=True,
            federate_after_ns=5_000_000,
        )
        text = emit_federated_top_level(
            system_name="Bringup",
            federate_names=["talker", "listener"],
            cross_connections=[conn],
        )
        assert "talker.publisher_node.sensor_data -> listener.listener_node.sensor_data" in text
        assert "after 5 msec" in text

    def test_no_cross_connections_clean(self):
        text = emit_federated_top_level(
            system_name="Bringup",
            federate_names=["a", "b"],
            cross_connections=[],
        )
        assert "after" not in text


# ---------------------------------------------------------------------------
# LFConnection cross-federate after delay
# ---------------------------------------------------------------------------

class TestFederateLaunchPackageExecConfig:
    """Each federate must receive the WHOLE resolved exec_config.json table
    (00_design.md section 2.4 S6) — ROS2_LF_EXEC_CONFIG set in additional_env
    alongside SYSTEM_JSON, mirroring the existing system_json wiring exactly."""

    def _emit(self, tmp_path, *, with_exec_config: bool):
        from colcon_lf.verb.generate import _emit_federate_launch_package

        pkg_dir = tmp_path / "lf_generated_bringup_federated_talker"
        pkg_dir.mkdir()

        exec_config_json_path = None
        if with_exec_config:
            exec_config_json_path = pkg_dir / "exec_config.json"
            exec_config_json_path.write_text(
                textwrap.dedent("""\
                    {
                      "schema_version": 1,
                      "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
                      "subscriptions": []
                    }
                """),
                encoding="utf-8",
            )

        _emit_federate_launch_package(
            pkg_dir=pkg_dir,
            pkg_name="lf_generated_bringup_federated_talker",
            coord_pkg_name="lf_generated_bringup_federated",
            fed_name="talker",
            lf_stem="Talker",
            system_json=None,
            all_dep_packages=["rclcpp"],
            exec_config_json_path=exec_config_json_path,
        )
        launch_file = pkg_dir / "launch" / "lf_talker.launch.py"
        assert launch_file.is_file()
        return launch_file.read_text(), (pkg_dir / "CMakeLists.txt").read_text()

    def test_no_exec_config_env_falls_back_to_environ(self, tmp_path):
        launch_text, cmake_text = self._emit(tmp_path, with_exec_config=False)
        assert 'os.environ.get("ROS2_LF_EXEC_CONFIG", "")' in launch_text
        assert (
            'additional_env={"SYSTEM_JSON": system_json, '
            '"ROS2_LF_EXEC_CONFIG": exec_config,\n'
            '                        "RCLLF_EXIT_WITH_PARENT": str(os.getpid())}' in launch_text
        )
        assert "exec_config.json" not in cmake_text

    def test_exec_config_staged_and_wired_into_additional_env(self, tmp_path):
        launch_text, cmake_text = self._emit(tmp_path, with_exec_config=True)
        # Same get_package_share_directory idiom used for system_json.
        assert (
            "os.path.join(get_package_share_directory("
            "'lf_generated_bringup_federated_talker'), 'exec_config.json')"
            in launch_text
        )
        assert (
            'additional_env={"SYSTEM_JSON": system_json, '
            '"ROS2_LF_EXEC_CONFIG": exec_config,\n'
            '                        "RCLLF_EXIT_WITH_PARENT": str(os.getpid())}' in launch_text
        )
        # Installed via CMake alongside system.json, same guarded idiom.
        assert 'install(FILES exec_config.json' in cmake_text
        assert 'DESTINATION share/${PROJECT_NAME}' in cmake_text

    def test_exec_config_whole_table_staged_per_federate(self, tmp_path):
        """The per-federate exec_config.json is a byte-identical copy of the
        whole resolved table — never filtered to that federate's nodes,
        unlike system.json (S6: simpler; unmatched rows are just never
        looked up)."""
        from colcon_lf.verb.generate import _generate_federated
        from colcon_lf.federation.partition import FederationMap

        # Mirror the real {lf-analysis}/{package}/{stem}/system.json layout:
        # _generate_federated derives lf_generated_root as
        # system_json.parent.parent.parent / "lf-generated".
        analysis_dir = tmp_path / "lf-analysis" / "test_pkg" / "bringup"
        analysis_dir.mkdir(parents=True)
        system_json = analysis_dir / "system.json"
        system_json.write_text(json.dumps({
            "name": "bringup", "nodes": {}, "communication": {"topics": []},
        }))
        exec_config_json_path = analysis_dir / "exec_config.json"
        whole_table = {
            "schema_version": 1,
            "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
            "subscriptions": [
                {"node_class": "OtherFederateNode", "topic": "t",
                 "take_mode": "block", "take_timeout_ms": 250},
            ],
        }
        exec_config_json_path.write_text(json.dumps(whole_table))

        # Real source files: _generate_federated stages each node's .cpp into the
        # federate package and fails closed on an unresolvable path.
        graph = _make_graph(
            ["talker", "listener"],
            src_dir=tmp_path / "packages" / "src" / "test_pkg" / "src",
        )
        fmap = FederationMap(
            node_to_federate={"talker": "talker", "listener": "listener"},
            federate_names=["talker", "listener"],
        )

        rc = _generate_federated(
            system_json=system_json,
            graph=graph,
            fmap=fmap,
            after_delay="1 msec",
            generate_launch=True,
            exec_config_json_path=exec_config_json_path,
        )
        assert rc == 0

        lf_generated_root = tmp_path / "lf-generated"
        fed_exec_configs = sorted(lf_generated_root.rglob("exec_config.json"))
        # One copy staged per federate sub-package (not the top-level file).
        assert len(fed_exec_configs) == 2
        for p in fed_exec_configs:
            assert json.loads(p.read_text()) == whole_table

    def test_peer_check_runs_after_the_rti_start_time(self, tmp_path):
        """Each federate defers the peer check out of its
        pre-main wait and runs it in a wrapper of reactor-c's start time
        handshake, which its link wraps. A federate started seconds before its
        peer then no longer spends the peer check budget waiting for it."""
        from colcon_lf.verb.generate import _generate_federated
        from colcon_lf.federation.partition import FederationMap

        analysis_dir = tmp_path / "lf-analysis" / "test_pkg" / "bringup"
        analysis_dir.mkdir(parents=True)
        system_json = analysis_dir / "system.json"
        system_json.write_text(json.dumps({
            "name": "bringup", "nodes": {}, "communication": {"topics": []},
        }))
        graph = _make_graph(
            ["talker", "listener"],
            src_dir=tmp_path / "packages" / "src" / "test_pkg" / "src",
        )
        fmap = FederationMap(
            node_to_federate={"talker": "talker", "listener": "listener"},
            federate_names=["talker", "listener"],
        )
        assert _generate_federated(
            system_json=system_json, graph=graph, fmap=fmap,
            after_delay="1 msec", generate_launch=False,
        ) == 0
        root = tmp_path / "lf-generated"
        startups = sorted(root.rglob("ros2_lf_startup.cpp"))
        assert len(startups) == 2, startups
        for cpp in startups:
            text = cpp.read_text()
            body = text[text.index("ros2_lf_pre_main_init() {"):]
            assert body.index("ros2_lf_runtime_defer_peer_check();") < \
                body.index("ros2_lf_runtime_wait_ready();"), cpp
            wrap = re.search(
                r"void __wrap_lf_synchronize_with_other_federates\(void\) \{(.*?)\}",
                text, re.S)
            assert wrap, f"{cpp}: no wrapper of the start time handshake"
            assert wrap.group(1).index("__real_lf_synchronize_with_other_federates();") < \
                wrap.group(1).index("ros2_lf_runtime_peer_check();"), wrap.group(1)
            # The end counts go out before the
            # RTI resign, which a peer's resign can hold until the peer exits.
            term = re.search(
                r"void __wrap_lf_terminate_execution\(environment_t\* env\) \{"
                r"(.*?__real_lf_terminate_execution\(env\);)",
                text, re.S)
            assert term, f"{cpp}: no wrapper of the RTI resign"
            body = term.group(1)
            # A normal end only: after a SIGINT the RTI hears of the failure
            # first. The test hook delays the resign, after the counts went out.
            order = [body.find(s) for s in (
                "if (_lf_normal_termination)",
                "ros2_lf_runtime_shutdown_before_resign();",
                "ros2_lf_runtime_test_resign_delay();",
                "__real_lf_terminate_execution(env);",
            )]
            assert -1 not in order and order == sorted(order), (
                f"{cpp}: the binding shutdown does not run before the RTI resign "
                f"on a normal end, so a peer can wait for end counts this federate "
                f"sends only after its resign: {body}")
        dispatch = next(root.rglob("ros2_lf_bindings.cmake")).read_text()
        assert dispatch.count('"LINKER:--wrap=lf_synchronize_with_other_federates"') == 2, dispatch
        assert dispatch.count('"LINKER:--wrap=lf_terminate_execution"') == 2, dispatch

    def test_cross_federate_connection_uses_multi_segment_port_name(self, tmp_path):
        """Regression (federated path): ``_generate_federated`` builds

        the full connection set via a direct ``_build_connections`` call
        (verb/generate.py ~line 702) rather than through
        ``generate_lf_system``, to classify connections as intra- vs
        cross-federate before per-federate generation. That call site must
        also pass ``nodes=`` so a multi-segment node-local topic (e.g.
        basic_components' ``"basic_components/timer_out"``) resolves to the
        same port name the reactor declares — otherwise the boundary port
        name computed from ``conn.source``/``conn.destination`` (lines
        ~811-812) is wrong and the emitted top-level federated .lf references
        a port no federate exposes.
        """
        from colcon_lf.verb.generate import _generate_federated
        from colcon_lf.federation.partition import FederationMap

        analysis_dir = tmp_path / "lf-analysis" / "test_pkg" / "bringup"
        analysis_dir.mkdir(parents=True)
        system_json = analysis_dir / "system.json"
        system_json.write_text(json.dumps({
            "name": "bringup", "nodes": {}, "communication": {"topics": []},
        }))

        src_dir = tmp_path / "packages" / "src" / "test_pkg" / "src"
        pub = _make_node("timer_node", class_name="TimerNode", src_dir=src_dir)
        pub.callbacks = [CallbackInfo(name="timer_callback", publishers_used=["basic_components/timer_out"])]
        pub.timers = [TimerInfo(period_ms=500.0, callback="timer_callback")]
        pub.publishers = [PublisherInfo(
            topic="basic_components/timer_out",
            message_type="std_msgs::msg::String",
        )]
        sub = _make_node("relay_node", class_name="RelayNode", src_dir=src_dir)
        sub.callbacks = [CallbackInfo(name="on_message")]
        sub.subscribers = [SubscriberInfo(
            topic="basic_components/timer_out",
            message_type="std_msgs::msg::String",
            callback="on_message",
        )]

        graph = SystemGraph(
            name="combo_pipeline",
            launch_graph=_make_launch_graph(),
            nodes={"timer_node": pub, "relay_node": sub},
            communication=SystemCommunication(
                topics=[TopicConnection(
                    topic="/basic_components/timer_out",
                    message_type="std_msgs::msg::String",
                    publishers=["timer_node"],
                    subscribers=["relay_node"],
                )],
            ),
        )
        # Each node in its own federate so the topic connection is
        # cross-federate and goes through the boundary-port-naming path.
        fmap = FederationMap(
            node_to_federate={"timer_node": "timer_node", "relay_node": "relay_node"},
            federate_names=["timer_node", "relay_node"],
        )

        rc = _generate_federated(
            system_json=system_json,
            graph=graph,
            fmap=fmap,
            after_delay="1 msec",
            generate_launch=False,
        )
        assert rc == 0

        lf_generated_root = tmp_path / "lf-generated"
        top_level_lf = next(
            p for p in lf_generated_root.rglob("*_federated.lf")
            if p.parent == lf_generated_root / "lf_generated_test_pkg_combopipeline_federated" / "lf"
        )
        top_text = top_level_lf.read_text(encoding="utf-8")
        assert "basic_components_timer_out" in top_text
        # The earlier, last-segment-only port name must NOT appear as a
        # bare port reference (".timer_out" would be the truncated form the
        # bug produced; "basic_components_timer_out" legitimately contains
        # "timer_out" as a substring, so check for the truncated port form
        # specifically rather than plain substring absence).
        assert ".timer_out" not in top_text.replace("_timer_out", "")


class TestBoundaryPortNaming:
    """A federate's boundary port never takes the name of a
    reactor instance it contains. The reference system names every topic
    after its publishing node, so a decentralized link (refused
    correspondence admission, plain topic-named ports) declared output
    ``NDTLocalizer`` next to the instance ``NDTLocalizer`` and lfc's C code
    failed with a redeclaration of ``_lf_NDTLocalizer``."""

    @pytest.mark.parametrize("coordination", ["decentralized", "centralized"])
    def test_boundary_port_never_equals_contained_instance(self, tmp_path, coordination):
        from colcon_lf.verb.generate import _generate_federated
        from colcon_lf.federation.partition import FederationMap

        analysis_dir = tmp_path / "lf-analysis" / "test_pkg" / "bringup"
        analysis_dir.mkdir(parents=True)
        system_json = analysis_dir / "system.json"
        system_json.write_text(json.dumps({
            "name": "bringup", "nodes": {}, "communication": {"topics": []},
        }))

        # Each node publishes a topic named after itself, like the reference
        # system. Pub -> Sub crosses perception -> planning, Sub -> Pub
        # crosses back, so both federates hold an output and an input whose
        # topic name equals a contained instance. Sub2 also reads Pub, so the
        # output Pub fans out to two remote subscribers like NDTLocalizer.
        src_dir = tmp_path / "packages" / "src" / "test_pkg" / "src"
        nodes = {}
        for name, peer in (("Pub", "Sub"), ("Sub", "Pub"), ("Sub2", "Pub")):
            node = _make_node(name, class_name=f"{name}Node", src_dir=src_dir)
            node.callbacks = [
                CallbackInfo(name="timer_callback", publishers_used=[name]),
                CallbackInfo(name="on_message"),
            ]
            node.timers = [TimerInfo(period_ms=100.0, callback="timer_callback")]
            node.publishers = [PublisherInfo(topic=name, message_type="std_msgs::msg::String")]
            node.subscribers = [SubscriberInfo(
                topic=peer, message_type="std_msgs::msg::String", callback="on_message",
            )]
            nodes[name] = node
        graph = SystemGraph(
            name="collide",
            launch_graph=_make_launch_graph(),
            nodes=nodes,
            communication=SystemCommunication(topics=[
                TopicConnection(topic="/Pub", message_type="std_msgs::msg::String",
                                publishers=["Pub"], subscribers=["Sub", "Sub2"]),
                TopicConnection(topic="/Sub", message_type="std_msgs::msg::String",
                                publishers=["Sub"], subscribers=["Pub"]),
                TopicConnection(topic="/Sub2", message_type="std_msgs::msg::String",
                                publishers=["Sub2"], subscribers=["Pub"]),
            ]),
        )
        fmap = FederationMap(
            node_to_federate={"Pub": "perception", "Sub": "planning", "Sub2": "planning"},
            federate_names=["perception", "planning"],
            coordination=coordination,
        )

        rc = _generate_federated(
            system_json=system_json, graph=graph, fmap=fmap,
            after_delay="1 msec", generate_launch=False,
        )
        assert rc == 0

        fed_root = tmp_path / "lf-generated" / "lf_generated_test_pkg_collide_federated" / "lf"
        top_text = (fed_root / "Collide_federated.lf").read_text(encoding="utf-8")
        for fed in ("perception", "planning"):
            text = next((fed_root / fed).glob("*.lf")).read_text(encoding="utf-8")
            outer = text[text.index(f"reactor {next((fed_root / fed).glob('*.lf')).stem} {{"):]
            instances = set(re.findall(r"^\s+(\w+) = new \w+\(\)", outer, re.M))
            ports = set(re.findall(r"^\s+(?:input|output) (\w+):", outer, re.M))
            assert instances, f"no contained instances found in federate {fed}"
            assert ports, f"no boundary ports found in federate {fed}"
            clash = sorted(instances & ports)
            assert not clash, (
                f"federate {fed}: boundary port name equals a contained instance "
                f"{clash} (lfc: redeclaration of _lf_{clash[0]})"
            )
            # A fan-out output is forwarded once and triggers its legacy guard
            # once: lfc rejects a reaction naming one trigger twice
            # (redeclaration of NDTLocalizer::NDTLocalizer in the C code).
            forwards = re.findall(r"^\s+(\w+\.\w+ -> \w+|\w+ -> \w+\.\w+)$", outer, re.M)
            assert len(forwards) == len(set(forwards)), (
                f"federate {fed}: duplicate forwarding connection {sorted(forwards)}")
            for trig in re.findall(r"^\s+reaction\(([^)]*)\)", outer, re.M):
                names = [t.strip() for t in trig.split(",")]
                assert len(names) == len(set(names)), (
                    f"federate {fed}: reaction({trig}) names a trigger twice "
                    f"(lfc: redeclaration of {names[0].replace('.', '::')})")
            # Every top-level link endpoint on this federate names a declared port.
            for port in re.findall(rf"\b{fed}\.(\w+)", top_text):
                assert port in ports, f"top level references {fed}.{port}, not declared"


class TestRemappedIntraFederateLink:
    """A launch remap inside one federate names the declared
    ports. The federate sub-graph carried no ``launch_graph``, so the node
    identities fell back to bare names, the remapped endpoint did not
    resolve and the link took the topic's last segment as port name
    (``pub.t0 -> sub.t0``), which lfc rejects."""

    def test_remapped_link_inside_one_federate_names_declared_ports(self, tmp_path):
        from colcon_lf.comm_analysis.topic_mapper import build_communication
        from colcon_lf.federation.partition import FederationMap
        from colcon_lf.verb.generate import _generate_federated

        analysis_dir = tmp_path / "lf-analysis" / "test_pkg" / "bringup"
        analysis_dir.mkdir(parents=True)
        system_json = analysis_dir / "system.json"
        system_json.write_text(json.dumps({
            "name": "bringup", "nodes": {}, "communication": {"topics": []},
        }))

        # pub (out:=fan/t0) -> sub (in:=fan/t0) inside federate "main"; far
        # in federate "apart" reads fan/t0 unremapped, so the system is a
        # real two-federate federation.
        src_dir = tmp_path / "packages" / "src" / "test_pkg" / "src"
        msg = "std_msgs::msg::Int32"
        pub = _make_node("pub", class_name="PubNode", src_dir=src_dir)
        pub.callbacks = [CallbackInfo(name="tick", publishers_used=["out"])]
        pub.timers = [TimerInfo(period_ms=100.0, callback="tick")]
        pub.publishers = [PublisherInfo(topic="out", message_type=msg)]
        subs = {}
        for name, cls, topic in (("sub", "SubNode", "in"), ("far", "FarNode", "fan/t0")):
            node = _make_node(name, class_name=cls, src_dir=src_dir)
            node.callbacks = [CallbackInfo(name="on_msg")]
            node.subscribers = [SubscriberInfo(topic=topic, message_type=msg,
                                               callback="on_msg")]
            subs[name] = node
        entries = [(pub, [("out", "fan/t0")]), (subs["sub"], [("in", "fan/t0")]),
                   (subs["far"], [])]
        actions = [LaunchNodeAction(package="test_pkg", executable=ni.name, name=ni.name,
                                    remappings=list(remaps)) for ni, remaps in entries]
        graph = SystemGraph(
            name="bringup",
            launch_graph=_make_launch_graph(actions),
            nodes={ni.name: ni for ni, _ in entries},
            communication=build_communication(
                [(a, ni) for a, (ni, _) in zip(actions, entries)]),
        )
        assert [t.topic for t in graph.communication.topics] == ["/fan/t0"]
        fmap = FederationMap(
            node_to_federate={"pub": "main", "sub": "main", "far": "apart"},
            federate_names=["main", "apart"],
            coordination="decentralized",
        )

        rc = _generate_federated(
            system_json=system_json, graph=graph, fmap=fmap,
            after_delay="1 msec", generate_launch=False,
        )
        assert rc == 0

        fed_root = tmp_path / "lf-generated" / "lf_generated_test_pkg_bringup_federated" / "lf"
        text = next((fed_root / "main").glob("*.lf")).read_text(encoding="utf-8")
        links = re.findall(r"^\s*(pub\.\w+ -> sub\.\w+)", text, re.M)
        assert links == ["pub.out -> sub.in"], (
            f"intra-federate remapped link names {links}, expected the declared "
            f"ports ['pub.out -> sub.in']:\n{text}")


class TestLFConnectionCrossFederate:
    def test_federate_after_ns_used_in_emission(self):
        from colcon_lf.generation.lf_emitter import _emit_connection
        conn = LFConnection(
            source="a.out",
            destination="b.inp",
            after_delay_ns=1_000_000,
            is_cross_federate=True,
            federate_after_ns=8_000_000,
        )
        text = _emit_connection(conn)
        # federate_after_ns (8ms) should win over after_delay_ns (1ms)
        assert "8 msec" in text
        assert "1 msec" not in text

    def test_non_cross_uses_after_delay_ns(self):
        from colcon_lf.generation.lf_emitter import _emit_connection
        conn = LFConnection(
            source="a.out",
            destination="b.inp",
            after_delay_ns=3_000_000,
            is_cross_federate=False,
        )
        text = _emit_connection(conn)
        assert "3 msec" in text


# ---------------------------------------------------------------------------
# Decentralized coordination
# ---------------------------------------------------------------------------

class TestDecentralizedCoordination:
    def test_target_property_omitted_for_centralized(self):
        text = emit_federated_top_level(
            system_name="Bringup",
            federate_names=["talker", "listener"],
            cross_connections=[],
            target_options={"build-type": "RelWithDebInfo"},
            coordination="centralized",
        )
        assert "coordination:" not in text
        assert "Coordination: centralized" in text

    def test_target_property_emitted_for_decentralized(self):
        text = emit_federated_top_level(
            system_name="Bringup",
            federate_names=["talker", "listener"],
            cross_connections=[],
            target_options={"build-type": "RelWithDebInfo", "coordination": "decentralized"},
            coordination="decentralized",
        )
        assert "coordination: decentralized" in text
        assert "Coordination: decentralized" in text

    def test_stp_reaction_emitted_for_decentralized_input(self):
        """A cross-federate input with an stp offset gets a logging-only STP
        reaction alongside (not instead of) its plain forwarding connection.

        Falsified once: with the STP branch in lf_emitter._emit_main_reactor
        temporarily reverted to the old unconditional plain-connection code,
        this assertion failed with an AssertionError naming the missing 'STP('
        text, not a generic failure — confirming it tests the real behavior.
        """
        system = LFSystem(name="TalkerFederate", federate_name="talker")
        system.cross_federate_ports = [
            ("sensor_data", "input", "stub_t", "listener_node", "sensor_data", 15_000_000),
        ]
        text = emit_lf_system(system)
        assert "sensor_data -> listener_node.sensor_data" in text  # forwarding kept
        assert "reaction(sensor_data)" in text
        assert "STP(15 msec)" in text
        assert "lf_print_warning" in text
        assert "STP violation" in text

    def test_no_stp_reaction_for_centralized_input(self):
        system = LFSystem(name="TalkerFederate", federate_name="talker")
        system.cross_federate_ports = [
            ("sensor_data", "input", "stub_t", "listener_node", "sensor_data", None),
        ]
        text = emit_lf_system(system)
        assert "sensor_data -> listener_node.sensor_data" in text
        assert "STP(" not in text
        assert "reaction(sensor_data)" not in text

    def test_no_stp_reaction_for_output_port(self):
        """Output boundary ports never receive network data, so never get STP."""
        system = LFSystem(name="TalkerFederate", federate_name="talker")
        system.cross_federate_ports = [
            ("sensor_data", "output", "stub_t", "publisher_node", "sensor_data", 15_000_000),
        ]
        text = emit_lf_system(system)
        assert "publisher_node.sensor_data -> sensor_data" in text
        assert "STP(" not in text

    def test_yaml_coordination_parsed(self, tmp_path):
        yaml_path = tmp_path / "federation.yaml"
        yaml_path.write_text(
            "system: demo\n"
            "coordination: decentralized\n"
            "federates:\n"
            "  a:\n"
            "    nodes: [A]\n"
            "  b:\n"
            "    nodes: [B]\n"
        )
        graph = _make_graph(["A", "B"])
        fmap = resolve_partition(str(yaml_path), graph)
        assert fmap.coordination == "decentralized"

    def test_yaml_coordination_default_centralized(self, tmp_path):
        yaml_path = tmp_path / "federation.yaml"
        yaml_path.write_text(
            "system: demo\nfederates:\n  a:\n    nodes: [A]\n  b:\n    nodes: [B]\n"
        )
        graph = _make_graph(["A", "B"])
        fmap = resolve_partition(str(yaml_path), graph)
        assert fmap.coordination == "centralized"

    def test_yaml_coordination_invalid_rejected(self, tmp_path):
        yaml_path = tmp_path / "federation.yaml"
        yaml_path.write_text(
            "system: demo\ncoordination: quantum\nfederates:\n  a:\n    nodes: [A]\n  b:\n    nodes: [B]\n"
        )
        graph = _make_graph(["A", "B"])
        with pytest.raises(ValueError, match="coordination"):
            resolve_partition(str(yaml_path), graph)

    def test_yaml_stp_offset_parsed(self, tmp_path):
        yaml_path = tmp_path / "federation.yaml"
        yaml_path.write_text(
            "system: demo\n"
            "coordination: decentralized\n"
            "federates:\n"
            "  a:\n"
            "    nodes: [A]\n"
            "  b:\n"
            "    nodes: [B]\n"
            "after_overrides:\n"
            "  - from: A.out\n"
            "    to: B.inp\n"
            "    after: 5ms\n"
            "    stp_offset: 20ms\n"
        )
        graph = _make_graph(["A", "B"])
        fmap = resolve_partition(str(yaml_path), graph)
        assert fmap.after_overrides[0].stp_offset_ns == 20_000_000
        assert fmap.stp_ns_for("A.out", "B.inp") == 20_000_000

    def test_stp_ns_for_default_when_no_override(self):
        fmap = FederationMap(coordination="decentralized")
        assert fmap.stp_ns_for("A.out", "B.inp") == DEFAULT_STP_OFFSET_NS

    def test_stp_ns_for_override_wins(self):
        fmap = FederationMap(
            coordination="decentralized",
            after_overrides=[AfterOverride("A.out", "B.inp", 5_000_000, 30_000_000)],
        )
        assert fmap.stp_ns_for("A.out", "B.inp") == 30_000_000
