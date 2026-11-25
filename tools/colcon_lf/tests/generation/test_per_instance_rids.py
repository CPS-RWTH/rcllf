# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Per-instance reactor definitions / binding keys.

Acceptance tests:

* multi-instance class -> N reactor definitions, disjoint RID ranges,
  instance-keyed factory registrations;
* single-instance system -> byte-identical output to the per-class scheme
  (the regression guard that justifies keeping plain names at N=1);
* the N=1 -> N=2 key-flip transition;
* collision disambiguation for raw node keys that normalize identically;
* RID-macro set uniqueness assertion;
* the federation consistency check from section 6.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.generation.binding_header_emitter import (
    collect_rid_table,
    emit_reaction_ids_header,
    emit_rid_init_cpp,
)
from colcon_lf.generation.binding_key import build_binding_keys, normalize_node_key
from colcon_lf.generation.factory_emitter import emit_node_factories_cpp, ros_args_for
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph, LaunchNodeAction
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    SubscriberInfo,
    TimerInfo,
)

_REPO_ROOT = Path(__file__).resolve().parents[4]
_ANALYSIS = _REPO_ROOT / "packages" / "lf-analysis"
_ALL_FEATURES_JSON = _ANALYSIS / "launch_features_example" / "all_features" / "system.json"
_ALL_FEATURES_EXEC_CONFIG = (
    _REPO_ROOT / "packages" / "src" / "fixtures" / "launch_features_example" / "lf"
    / "all_features.execution.yaml")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _launch_graph(nodes: list[LaunchNodeAction] | None = None) -> LaunchGraph:
    root = LaunchFileInfo(file_path=Path("/fake/bringup.launch.py"), package="test_pkg")
    return LaunchGraph(root=root, all_nodes=nodes or [])


def _sensor_node(name: str) -> NodeInfo:
    return NodeInfo(
        name=name,
        class_name="SensorNode",
        package="test_pkg",
        source_file=Path("/fake/sensor.cpp"),
        header_file=Path("/fake/sensor.hpp"),
        callbacks=[
            CallbackInfo(name="timer_callback", publishers_used=["out"]),
            CallbackInfo(name="sensor_callback"),
        ],
        timers=[TimerInfo(period_ms=100.0, callback="timer_callback")],
        subscribers=[
            SubscriberInfo(
                topic="sensor_data",
                message_type="std_msgs::msg::String",
                callback="sensor_callback",
            )
        ],
        publishers=[PublisherInfo(topic="out", message_type="std_msgs::msg::Float64")],
    )


def _graph(node_keys: list[str], *, launch_nodes=None) -> SystemGraph:
    return SystemGraph(
        name="dup_system",
        launch_graph=_launch_graph(launch_nodes),
        nodes={key: _sensor_node(key.rsplit("/", 1)[-1]) for key in node_keys},
        communication=SystemCommunication(),
    )


def _artifacts(
    graph: SystemGraph,
    timer_period_overrides: dict[str, dict[str, float]] | None = None,
) -> tuple[str, list, str]:
    """Return (.lf text, rid_table, header text) for *graph*."""
    lf_system = generate_lf_system(
        graph, target_lfc=True, timer_period_overrides=timer_period_overrides)
    class_to_reactor = {r.name: r for r in lf_system.reactors}
    rid_table = collect_rid_table(
        class_to_reactor, graph, timer_period_overrides=timer_period_overrides)
    return emit_lf_system(lf_system), rid_table, emit_reaction_ids_header(rid_table)


def _execution_yaml_timer_periods(path: Path, graph: SystemGraph) -> dict[str, dict[str, float]]:
    """``{node_key: {callback: period_ms}}`` from *path*'s ``nodes.<node>.timers``.

    The same derivation as ``verb.generate._resolve_and_stage_resolved_model``.
    """
    from colcon_lf.exec_config.model import ExecConfig
    from colcon_lf.generation.lf_time import parse_lf_time_ns

    config = ExecConfig.load(path, graph)
    periods: dict[str, dict[str, float]] = {}
    for node_key, body in config.nodes.items():
        for callback, fields in (body.get("timers") or {}).items():
            if "period" in fields:
                periods.setdefault(node_key, {})[callback] = (
                    parse_lf_time_ns(fields["period"]) / 1_000_000)
    return periods


# ---------------------------------------------------------------------------
# binding_key helper
# ---------------------------------------------------------------------------


class TestBindingKeySynthesis:
    def test_single_instance_class_keeps_plain_name(self):
        graph = _graph(["/robot_1/sensor"])
        assert build_binding_keys(graph.nodes) == {"/robot_1/sensor": "SensorNode"}

    def test_multi_instance_class_is_suffixed(self):
        graph = _graph(["/robot_1/sensor", "sim_sensor"])
        assert build_binding_keys(graph.nodes) == {
            "/robot_1/sensor": "SensorNode__robot_1_sensor",
            "sim_sensor": "SensorNode__sim_sensor",
        }

    def test_normalize_strips_slashes_and_collapses_underscores(self):
        assert normalize_node_key("/robot_1/timer_node") == "robot_1_timer_node"
        assert normalize_node_key("//a//b") == "a_b"
        assert normalize_node_key("a-b.c") == "a_b_c"
        assert normalize_node_key("///") == "unnamed"

    def test_colliding_candidates_get_deterministic_hash_suffix(self):
        # /a/b_c and /a_b/c both normalize to "a_b_c".
        graph = _graph(["/a/b_c", "/a_b/c"])
        keys = build_binding_keys(graph.nodes)
        assert len(set(keys.values())) == 2
        for raw, key in keys.items():
            assert key.startswith("SensorNode__a_b_c_h"), key
        # Deterministic across runs for a fixed topology.
        assert build_binding_keys(graph.nodes) == keys

    def test_mixed_multiplicity_classes(self):
        graph = SystemGraph(
            name="mixed",
            launch_graph=_launch_graph(),
            nodes={
                "a": _sensor_node("a"),
                "b": _sensor_node("b"),
                "solo": NodeInfo(
                    name="solo", class_name="SoloNode", package="p",
                    source_file=Path("/fake/solo.cpp"),
                ),
            },
            communication=SystemCommunication(),
        )
        keys = build_binding_keys(graph.nodes)
        assert keys["solo"] == "SoloNode"
        assert keys["a"] == "SensorNode__a"
        assert keys["b"] == "SensorNode__b"

    def test_duplicate_keys_raise_naming_raw_keys(self, monkeypatch):
        graph = _graph(["/a/b_c", "/a_b/c"])
        # Break the disambiguator so both candidates survive identical.
        monkeypatch.setattr(
            "colcon_lf.generation.binding_key._hash_suffix", lambda raw: ""
        )
        with pytest.raises(ValueError) as exc:
            build_binding_keys(graph.nodes)
        assert "/a/b_c" in str(exc.value)
        assert "/a_b/c" in str(exc.value)


# ---------------------------------------------------------------------------
# Generator: N definitions, disjoint RID ranges, instance-keyed registration
# ---------------------------------------------------------------------------


class TestPerInstanceReactorDefinitions:
    def test_n_reactor_definitions_for_multi_instance_class(self):
        graph = _graph(["/robot_1/sensor", "sim_sensor"])
        lf_system = generate_lf_system(graph, target_lfc=True)
        outer = [
            r.name for r in lf_system.reactors if not r.name.endswith("_DefaultGroup")
        ]
        assert sorted(outer) == [
            "SensorNode__robot_1_sensor", "SensorNode__sim_sensor",
        ]
        assert len(lf_system.main_instances) == 2

    def test_startup_bodies_bind_per_instance_keys(self):
        graph = _graph(["/robot_1/sensor", "sim_sensor"])
        lf_system = generate_lf_system(graph, target_lfc=True)
        # First line binds; the rest write the groups' bind ports.
        bodies = {r.startup_body.splitlines()[0] for r in lf_system.reactors if r.startup_body}
        assert bodies == {
            'ros2_lf_bind_node(self, "SensorNode__robot_1_sensor");',
            'ros2_lf_bind_node(self, "SensorNode__sim_sensor");',
        }

    def test_disjoint_rid_ranges(self):
        graph = _graph(["/robot_1/sensor", "sim_sensor"])
        _lf, rid_table, _hdr = _artifacts(graph)
        rids_by_key: dict[str, set[int]] = {}
        for rid, key, *_rest in rid_table:
            rids_by_key.setdefault(key, set()).add(rid)
        assert set(rids_by_key) == {
            "SensorNode__robot_1_sensor", "SensorNode__sim_sensor",
        }
        a, b = rids_by_key.values()
        assert a and b
        assert not (a & b), "RID ranges must be disjoint per instance"
        # Every RID is used exactly once across the whole table.
        assert len(rid_table) == len({row[0] for row in rid_table})

    def test_rid_macros_are_instance_scoped(self):
        graph = _graph(["/robot_1/sensor", "sim_sensor"])
        _lf, rid_table, header = _artifacts(graph)
        defines = [l for l in header.splitlines() if l.startswith("#define RID_")]
        macros = [l.split()[1] for l in defines if not l.startswith("#define RID_COUNT")]
        assert len(macros) == len(set(macros)), "RID macro names must be unique"
        assert any(m.startswith("RID_SensorNode__robot_1_sensor_") for m in macros)
        assert any(m.startswith("RID_SensorNode__sim_sensor_") for m in macros)

    def test_callback_names_survive_the_macro_round_trip(self):
        """The synthesized key contains ``__``; cb-name extraction must still work."""
        graph = _graph(["/robot_1/sensor", "sim_sensor"])
        _lf, rid_table, _hdr = _artifacts(graph)
        callbacks = {row[2] for row in rid_table}
        assert callbacks == {"timer_callback", "sensor_callback"}

    def test_rid_init_cpp_registers_binding_keys(self):
        graph = _graph(["/robot_1/sensor", "sim_sensor"])
        _lf, rid_table, _hdr = _artifacts(graph)
        cpp = emit_rid_init_cpp(rid_table, "pkg")
        assert '"SensorNode__robot_1_sensor"' in cpp
        assert '"SensorNode__sim_sensor"' in cpp
        assert f"resize_handle_table({len(rid_table)})" in cpp


class TestRidMacroUniquenessAssertion:
    def test_duplicate_macro_names_raise(self):
        table = [
            (0, "SensorNode", "cb", "timer", "", 0),
            (1, "SensorNode", "cb", "sub", "t", 0),
        ]
        with pytest.raises(ValueError, match="RID macro name collision"):
            emit_reaction_ids_header(table)

    def test_distinct_macro_names_are_accepted(self):
        table = [
            (0, "SensorNode__a", "cb", "timer", "", 0),
            (1, "SensorNode__b", "cb", "timer", "", 0),
        ]
        header = emit_reaction_ids_header(table)
        assert "#define RID_SensorNode__a_cb 0" in header
        assert "#define RID_SensorNode__b_cb 1" in header


# ---------------------------------------------------------------------------
# Regression guard + N=1 -> N=2 transition
# ---------------------------------------------------------------------------


class TestSingleInstanceRegression:
    def test_single_instance_output_uses_plain_class_name(self):
        graph = _graph(["/robot_1/sensor"])
        lf_text, rid_table, header = _artifacts(graph)
        assert "SensorNode__" not in lf_text
        assert "SensorNode__" not in header
        assert {row[1] for row in rid_table} == {"SensorNode"}
        assert 'ros2_lf_bind_node(self, "SensorNode");' in lf_text

    @pytest.mark.skipif(
        not _ALL_FEATURES_JSON.is_file(), reason="lf-analysis fixtures not present"
    )
    def test_every_single_instance_fixture_is_unchanged_in_shape(self):
        """No in-tree single-instance system gains a synthesized key."""
        from colcon_lf.output.serializer import deserialize_system_graph

        checked = 0
        for system_json in sorted(_ANALYSIS.rglob("system.json")):
            import warnings

            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                graph = deserialize_system_graph(system_json.read_text())
            class_counts: dict[str, int] = {}
            for ni in graph.nodes.values():
                class_counts[ni.class_name] = class_counts.get(ni.class_name, 0) + 1
            if any(count > 1 for count in class_counts.values()):
                continue  # multi-instance systems are expected to change
            keys = set(build_binding_keys(graph.nodes).values())
            assert not any("__" in key for key in keys), system_json
            checked += 1
        assert checked > 10, "expected a meaningful number of single-instance fixtures"


class TestN1ToN2Transition:
    """Adding a second instance re-keys that class's generated rows."""

    def test_key_flip_and_row_count(self):
        one = _graph(["sensor"])
        _lf1, table1, _h1 = _artifacts(one)
        assert {row[1] for row in table1} == {"SensorNode"}

        two = _graph(["sensor", "sensor_2"])
        _lf2, table2, _h2 = _artifacts(two)
        assert {row[1] for row in table2} == {
            "SensorNode__sensor", "SensorNode__sensor_2",
        }
        # The original plain key is gone — this is the documented churn.
        assert "SensorNode" not in {row[1] for row in table2}
        assert len(table2) == 2 * len(table1)

    def test_yaml_authoring_keys_survive_the_flip(self):
        """execution.yaml keys are node names, which the flip does not touch."""
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.exec_config.resolve import resolve

        raw = {"nodes": {"sensor": {"take_mode": "block"}}}

        one = _graph(["sensor"])
        rows_one = resolve(ExecConfig.from_dict(raw, one, source="t"), one).subscriptions
        assert [r.node_class for r in rows_one] == ["SensorNode"]
        assert rows_one[0].values["take_mode"] == "block"

        two = _graph(["sensor", "sensor_2"])
        rows_two = resolve(ExecConfig.from_dict(raw, two, source="t"), two).subscriptions
        by_key = {r.node_class: r for r in rows_two}
        # Same YAML, still applies to the same node, under its new binding key.
        assert by_key["SensorNode__sensor"].values["take_mode"] == "block"
        assert by_key["SensorNode__sensor_2"].values["take_mode"] == "skip"


# ---------------------------------------------------------------------------
# Factory emission
# ---------------------------------------------------------------------------


class TestFactoryEmission:
    def _entries(self):
        return [
            {
                "binding_key": "SensorNode__robot_1_sensor",
                "class_name": "SensorNode",
                "package": "test_pkg",
                "header": "test_pkg/sensor.hpp",
                "idx": 0,
                "node_name": "sensor",
                "namespace": "/robot_1",
                "base_class": "rclcpp::Node",
            },
            {
                "binding_key": "SensorNode__sim_sensor",
                "class_name": "SensorNode",
                "package": "test_pkg",
                "header": "test_pkg/sensor.hpp",
                "idx": 1,
                "node_name": "sim_sensor",
                "namespace": "",
                "base_class": "rclcpp::Node",
            },
        ]

    def _lifecycle_entry(self):
        return [
            {
                "binding_key": "LifecycleExampleNode",
                "class_name": "LifecycleExampleNode",
                "package": "lifecycle_example",
                "header": "lifecycle_example/lifecycle_example_node.hpp",
                "idx": 0,
                "node_name": "lifecycle_example_node",
                "namespace": "",
                "base_class": "rclcpp_lifecycle::LifecycleNode",
            },
        ]

    def test_registrations_are_instance_keyed(self):
        cpp = emit_node_factories_cpp(self._entries())
        assert cpp.count("ros2_lf_register_node(") == 2
        assert '"SensorNode__robot_1_sensor"' in cpp
        assert '"SensorNode__sim_sensor"' in cpp
        # The class-keyed registration that used to clobber is gone.
        assert 'ros2_lf_register_node("SensorNode"' not in cpp
        # Each registration names its class, so the runtime's "not in
        # registry" check matches the class, not a key prefix.
        assert 'ros2_lf_register_node("SensorNode__robot_1_sensor", "SensorNode", ' in cpp
        assert 'ros2_lf_register_node("SensorNode__sim_sensor", "SensorNode", ' in cpp

    def test_each_construction_runs_in_its_binding_key_scope(self):
        # The subscription gate deepens the reader of an admitted
        # edge only while the subscribing node's binding key is in scope, so
        # the scope must open before the node is constructed in each block.
        cpp = emit_node_factories_cpp(self._entries())
        for key in ("SensorNode__robot_1_sensor", "SensorNode__sim_sensor"):
            scope = f'Ros2LfNodeConstructionScope ros2_lf_scope("{key}");'
            assert cpp.count(scope) == 1
            assert cpp.index(scope) < cpp.index(f'ros2_lf_register_node("{key}"')

    def test_node_options_remaps_carry_launch_identity(self):
        cpp = emit_node_factories_cpp(self._entries())
        assert '"--ros-args", "-r", "__node:=sensor", "-r", "__ns:=/robot_1"' in cpp
        assert '"--ros-args", "-r", "__node:=sim_sensor"' in cpp

    def test_if_constexpr_guard_and_warning(self):
        cpp = emit_node_factories_cpp(self._entries())
        assert (
            "if constexpr (std::is_constructible_v<C, const rclcpp::NodeOptions &>)"
        ) in cpp
        assert "RCLCPP_WARN" in cpp
        assert "does not accept NodeOptions" in cpp
        assert "std::call_once" in cpp

    def test_header_included_once_per_distinct_header(self):
        cpp = emit_node_factories_cpp(self._entries())
        assert cpp.count('#include "test_pkg/sensor.hpp"') == 1

    def test_ros_args_empty_without_identity(self):
        assert ros_args_for("", "") == []
        assert ros_args_for("n", "") == ["--ros-args", "-r", "__node:=n"]

    def test_construct_node_helper_does_not_hard_type_rclcpp_node(self):
        """The shared helper must be base-class-agnostic.

        Before the fix, ``ros2_lf_construct_node`` returned
        ``std::shared_ptr<rclcpp::Node>``, which cannot bind a
        ``LifecycleNode``-derived ``std::make_shared<C>(...)`` result. The
        return type must be auto-deducible (``std::shared_ptr<C>`` / a bare
        ``auto``) so the same template works for any node base.
        """
        cpp = emit_node_factories_cpp(self._entries())
        assert "std::shared_ptr<rclcpp::Node> ros2_lf_construct_node(" not in cpp

    def test_lifecycle_node_registration_does_not_declare_rclcpp_node_ptr(self):
        """A LifecycleNode entry must not produce a shared_ptr<rclcpp::Node> binding.

        ``rclcpp_lifecycle::LifecycleNode`` is a sibling of ``rclcpp::Node``,
        not a subclass, so any generated code that spells the constructed
        pointer's type as ``shared_ptr<rclcpp::Node>`` fails to compile.
        """
        cpp = emit_node_factories_cpp(self._lifecycle_entry())
        assert "LifecycleExampleNode" in cpp
        assert "ros2_lf_register_node" in cpp
        assert "std::shared_ptr<rclcpp::Node> ros2_lf_construct_node(" not in cpp
        # No local variable/binding narrows the constructed lifecycle node to
        # rclcpp::Node before registration.
        assert "std::shared_ptr<rclcpp::Node>" not in cpp

    def test_both_base_classes_share_the_same_construct_helper(self):
        """One helper instantiated for two unrelated bases — no per-base branch."""
        mixed = self._entries() + self._lifecycle_entry()
        cpp = emit_node_factories_cpp(mixed)
        assert cpp.count("template<typename C>") == 1
        assert cpp.count("ros2_lf_construct_node<") == 3

    def test_factory_entries_passes_base_class_through(self):
        """``_factory_entries`` must carry ``NodeInfo.base_class`` into the entry dict.

        The emitter needs it to know a node is a ``LifecycleNode``;
        if generate.py's staging step drops the field, the emitter can never
        see it regardless of what the emitter itself does with it.
        """
        from colcon_lf.node_analysis.models import NodeInfo
        from colcon_lf.verb.generate import _factory_entries

        lifecycle_ni = NodeInfo(
            name="lifecycle_example_node",
            class_name="LifecycleExampleNode",
            package="lifecycle_example",
            source_file=Path("/fake/lifecycle_example_node.cpp"),
            header_file=Path("/fake/lifecycle_example_node.hpp"),
            base_class="rclcpp_lifecycle::LifecycleNode",
        )
        plain_ni = self._sensor_node_for_entries()
        non_component_items = [("lifecycle_node", lifecycle_ni), ("sensor", plain_ni)]
        binding_keys = {"lifecycle_node": "LifecycleExampleNode", "sensor": "SensorNode"}
        entries = _factory_entries(non_component_items, binding_keys, {})
        by_key = {e["binding_key"]: e for e in entries}
        assert by_key["LifecycleExampleNode"]["base_class"] == "rclcpp_lifecycle::LifecycleNode"
        assert by_key["SensorNode"]["base_class"] == "rclcpp::Node"

    def _sensor_node_for_entries(self):
        return _sensor_node("sensor")


# ---------------------------------------------------------------------------
# Component-instance list
# ---------------------------------------------------------------------------


class TestComponentInstanceList:
    def _component_graph(self) -> SystemGraph:
        launch_nodes = [
            LaunchNodeAction(
                package="test_pkg", executable="Comp", name="comp_a",
                namespace="/ns_a", is_component=True,
            ),
            LaunchNodeAction(
                package="test_pkg", executable="Comp", name="comp_b",
                is_component=True,
            ),
        ]

        def comp(name: str) -> NodeInfo:
            return NodeInfo(
                name=name, class_name="CompNode", package="test_pkg",
                source_file=Path("/fake/comp.cpp"), is_component=True,
            )

        return SystemGraph(
            name="comps",
            launch_graph=_launch_graph(launch_nodes),
            nodes={"/ns_a/comp_a": comp("comp_a"), "comp_b": comp("comp_b")},
            communication=SystemCommunication(),
        )

    def test_one_entry_per_instance_with_exactly_five_fields(self):
        from colcon_lf.generation.node_identity import build_node_identities
        from colcon_lf.verb.generate import build_component_instances

        graph = self._component_graph()
        entries = build_component_instances(
            graph, build_binding_keys(graph.nodes), build_node_identities(graph),
        )
        assert len(entries) == 2
        for entry in entries:
            assert set(entry) == {
                "package", "plugin_class", "binding_key", "node_name", "namespace",
            }
            # plugin_class stays the real pluginlib class name.
            assert entry["plugin_class"] == "CompNode"
        by_key = {e["binding_key"]: e for e in entries}
        assert by_key["CompNode__ns_a_comp_a"]["namespace"] == "/ns_a"
        assert by_key["CompNode__ns_a_comp_a"]["node_name"] == "comp_a"
        assert by_key["CompNode__comp_b"]["namespace"] == ""

    def test_json_key_matches_the_runtime_parser(self):
        """The C++ side reads ``doc.at("component_instances")`` — keep them in sync."""
        from colcon_lf.verb.generate import COMPONENT_INSTANCES_KEY

        assert COMPONENT_INSTANCES_KEY == "component_instances"

    def test_staged_into_system_json_additively(self, tmp_path):
        from colcon_lf.generation.node_identity import build_node_identities
        from colcon_lf.verb.generate import (
            COMPONENT_INSTANCES_KEY,
            _stage_component_instances,
        )

        graph = self._component_graph()
        system_json = tmp_path / "system.json"
        system_json.write_text(json.dumps({"name": "comps", "nodes": {}}))
        _stage_component_instances(
            system_json=system_json,
            graph=graph,
            binding_keys=build_binding_keys(graph.nodes),
            identities=build_node_identities(graph),
        )
        raw = json.loads(system_json.read_text())
        assert raw["name"] == "comps"  # existing keys untouched
        assert len(raw[COMPONENT_INSTANCES_KEY]) == 2


# ---------------------------------------------------------------------------
# Blast-radius checks (design note section 6)
# ---------------------------------------------------------------------------


class TestFederationConsistency:
    """A two-instance class split across federates must agree on binding keys.

    ``exec_config.json`` is resolved once against the whole graph and shipped
    whole to every federate (00_design.md 2.4 S6), so per-federate synthesis
    would produce plain ``SensorNode`` in a federate holding one instance while
    the resolved table used ``SensorNode__<key>``, and every lookup would miss.
    """

    def test_per_federate_tables_use_system_wide_keys(self):
        full = _graph(["/robot_1/sensor", "sim_sensor"])
        system_keys = build_binding_keys(full.nodes)

        # Two federates, one instance each — the split that would re-key.
        for node_key in full.nodes:
            slice_graph = SystemGraph(
                name="fed",
                launch_graph=_launch_graph(),
                nodes={node_key: full.nodes[node_key]},
                communication=SystemCommunication(),
            )
            lf_system = generate_lf_system(
                slice_graph, target_lfc=True, binding_keys=system_keys,
            )
            table = collect_rid_table(
                {r.name: r for r in lf_system.reactors}, slice_graph,
                binding_keys=system_keys,
            )
            assert {row[1] for row in table} == {system_keys[node_key]}
            assert emit_lf_system(lf_system).count(
                f'ros2_lf_bind_node(self, "{system_keys[node_key]}");'
            ) == 1

    def test_whole_table_rule_one_row_per_model_subscription(self):
        """Each federate's rows are a subset of the whole system's rows."""
        full = _graph(["/robot_1/sensor", "sim_sensor"])
        system_keys = build_binding_keys(full.nodes)
        _lf, full_table, _h = _artifacts(full)
        full_rows = {(row[1], row[2], row[3], row[4]) for row in full_table}

        seen: set[tuple] = set()
        for node_key in full.nodes:
            slice_graph = SystemGraph(
                name="fed", launch_graph=_launch_graph(),
                nodes={node_key: full.nodes[node_key]},
                communication=SystemCommunication(),
            )
            lf_system = generate_lf_system(
                slice_graph, target_lfc=True, binding_keys=system_keys,
            )
            table = collect_rid_table(
                {r.name: r for r in lf_system.reactors}, slice_graph,
                binding_keys=system_keys,
            )
            rows = {(row[1], row[2], row[3], row[4]) for row in table}
            assert rows <= full_rows
            seen |= rows
        assert seen == full_rows


class TestVisualizationUnaffected:
    """Viz reads SystemGraph, not the LF model — one smoke assertion (section 6)."""

    def test_system_graph_is_not_mutated_by_generation(self):
        graph = _graph(["/robot_1/sensor", "sim_sensor"])
        before = {key: ni.class_name for key, ni in graph.nodes.items()}
        generate_lf_system(graph, target_lfc=True)
        after = {key: ni.class_name for key, ni in graph.nodes.items()}
        assert before == after == {
            "/robot_1/sensor": "SensorNode", "sim_sensor": "SensorNode",
        }


@pytest.mark.skipif(
    not _ALL_FEATURES_JSON.is_file(), reason="lf-analysis fixtures not present"
)
class TestGeneratedSizeGrowth:
    """Code-size growth is measured, not assumed (section 6, last bullet).

    ``all_features`` is the worst case in tree: ``FeatureNode`` x12
    non-component plus ``FeatureComponent`` x3 component (secondary_node is
    behind a false condition and not started).
    """

    def test_all_features_growth_is_bounded_by_instance_count(self):
        import warnings

        from colcon_lf.output.serializer import deserialize_system_graph

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            graph = deserialize_system_graph(_ALL_FEATURES_JSON.read_text())

        # Generation refuses a timer whose analyzed period is null
        # (RCLLF-R001), and every all_features timer reads its period from a
        # parameter the analyzer cannot see. lf-generate takes the periods from
        # the package's execution.yaml, so this test does the same.
        periods = _execution_yaml_timer_periods(_ALL_FEATURES_EXEC_CONFIG, graph)
        assert len(periods) == len(graph.nodes), sorted(periods)

        lf_text, rid_table, _header = _artifacts(graph, timer_period_overrides=periods)
        instances = len(graph.nodes)

        # One outer reactor definition per node instance (16), each with its own
        # RID range: 32 rows vs 4 under the per-class scheme (8x), 461 .lf lines
        # vs 83 (5.6x).  Both scale with instance count, not quadratically.
        # Added a per-node ingress drain host and physical action, which
        # took this system to 886 lines (measured 2026-09-24, ~55 per instance);
        # the bound stays linear in the instance count. This test skips in CI
        # (no lf-analysis output there), which is how the old 40/instance bound
        # went stale unnoticed. secondary_node was dropped (behind a
        # false condition), so 15 instances and 30 rows.
        assert instances == 15
        assert len(rid_table) == 30
        assert len({row[1] for row in rid_table}) == instances
        assert len(lf_text.splitlines()) < 60 * instances
