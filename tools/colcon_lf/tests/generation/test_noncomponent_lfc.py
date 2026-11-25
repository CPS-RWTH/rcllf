# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for non-component lfc package generation."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from colcon_lf.output.serializer import deserialize_system_graph
from colcon_lf.verb.generate import _emit_lfc_package

_NONCOMP_SYSTEM_JSON = {
    "schema_version": 3,
    "name": "Bringup",
    "launch_graph": {
        "root": {
            "file_path": "/workspace/packages/src/pubsub_example/launch/bringup.launch.py",
            "package": "pubsub_example",
            "arguments": [],
            "nodes": [],
            "includes": [],
            "groups": [],
            "errors": [],
            "context_states": [],
        }
    },
    "nodes": {
        "publisher_node": {
            "name": "publisher_node",
            "class_name": "PublisherNode",
            "package": "pubsub_example",
            "source_file": "/workspace/packages/src/pubsub_example/src/publisher_node.cpp",
            "header_file": "/workspace/packages/src/pubsub_example/include/pubsub_example/publisher_node.hpp",
            "is_component": False,
            "callbacks": [],
            "timers": [{"period_ms": 500.0, "callback": "timer_callback",
                        "callback_group": "default", "member_variable": "timer_",
                        "created_in": "constructor", "destroyed_in": None, "offset_ms": 0.0}],
            "subscribers": [],
            "publishers": [{"topic": "sensor_data",
                            "message_type": "custom_interfaces::msg::SensorData",
                            "qos_depth": 10, "member_variable": "publisher_",
                            "is_lifecycle": False, "created_in": "constructor",
                            "destroyed_in": None, "qos_reliability": None,
                            "qos_durability": None, "qos_history": None,
                            "qos_deadline_ms": None}],
            "callback_groups": [{"name": "default", "type": "MutuallyExclusive"}],
            "base_class": "rclcpp::Node",
            "service_servers": [], "action_servers": [],
        },
        "subscriber_node": {
            "name": "subscriber_node",
            "class_name": "SubscriberNode",
            "package": "pubsub_example",
            "source_file": "/workspace/packages/src/pubsub_example/src/subscriber_node.cpp",
            "header_file": "/workspace/packages/src/pubsub_example/include/pubsub_example/subscriber_node.hpp",
            "is_component": False,
            "callbacks": [],
            "timers": [],
            "subscribers": [{"topic": "sensor_data",
                             "message_type": "custom_interfaces::msg::SensorData",
                             "qos_depth": 10, "callback": "subscription_callback",
                             "callback_group": "default", "member_variable": "subscription_",
                             "is_lifecycle": False, "created_in": "constructor",
                             "destroyed_in": None, "qos_reliability": None,
                             "qos_durability": None, "qos_history": None,
                             "qos_deadline_ms": None}],
            "publishers": [],
            "callback_groups": [{"name": "default", "type": "MutuallyExclusive"}],
            "base_class": "rclcpp::Node",
            "service_servers": [], "action_servers": [],
        },
    },
    "communication": {"topics": [], "services": [], "actions": []},
}


@pytest.fixture
def gen_dir(tmp_path: Path):
    sj_path = tmp_path / "lf-analysis" / "pubsub_example" / "bringup" / "system.json"
    sj_path.parent.mkdir(parents=True)

    # Materialize the node sources the fixture references and point the model at
    # them.  The committed fixture hardcodes container paths
    # (/workspace/packages/src/...) which do not exist when tests run on the
    # host; staging then silently emitted placeholder .cpp files and
    # test_node_source_staged still passed because it only counted files.
    # Source staging now fails closed, so the paths must actually resolve.
    system_json = json.loads(json.dumps(_NONCOMP_SYSTEM_JSON))
    src_root = tmp_path / "packages" / "src" / "pubsub_example" / "src"
    src_root.mkdir(parents=True)
    for node in system_json["nodes"].values():
        real_src = src_root / Path(node["source_file"]).name
        real_src.write_text(
            f'#include "{node["header_file"]}"\n'
            f'// stand-in body for {node["class_name"]}\n',
            encoding="utf-8",
        )
        node["source_file"] = str(real_src)

    sj_path.write_text(json.dumps(system_json))

    raw = sj_path.read_text()
    graph = deserialize_system_graph(raw)

    from colcon_lf.generation.lf_generator import generate_lf_system
    from colcon_lf.generation.lf_emitter import emit_lf_system
    lf_system = generate_lf_system(graph, target_lfc=True)
    lf_text = emit_lf_system(lf_system)

    lf_dir = sj_path.parent / "lf"
    lf_dir.mkdir()
    lf_file = lf_dir / f"{lf_system.name}.lf"
    lf_file.write_text(lf_text)

    stem = lf_system.name.lower()
    pkg_name = f"lf_generated_{stem}"
    lf_generated_root = tmp_path / "lf-generated"
    lf_generated_root.mkdir()
    pkg_dir = lf_generated_root / pkg_name
    pkg_dir.mkdir()
    (pkg_dir / "lf").mkdir()
    (pkg_dir / "src").mkdir()
    import shutil
    shutil.copy2(lf_file, pkg_dir / "lf" / lf_file.name)

    _emit_lfc_package(
        pkg_dir=pkg_dir,
        pkg_name=pkg_name,
        lf_system=lf_system,
        graph=graph,
        lf_file=lf_file,
    )
    return pkg_dir


class TestNonComponentPackageEmission:
    def test_node_factories_cpp_exists(self, gen_dir: Path):
        assert (gen_dir / "src" / "node_factories.cpp").exists()

    def test_node_factories_registers_publisher(self, gen_dir: Path):
        text = (gen_dir / "src" / "node_factories.cpp").read_text()
        assert "PublisherNode" in text
        assert "ros2_lf_register_node" in text

    def test_node_factories_registers_subscriber(self, gen_dir: Path):
        text = (gen_dir / "src" / "node_factories.cpp").read_text()
        assert "SubscriberNode" in text

    def test_node_factories_no_linguafranca_executor(self, gen_dir: Path):
        text = (gen_dir / "src" / "node_factories.cpp").read_text()
        assert "linguafranca_executor" not in text

    def test_cmake_has_no_linguafranca_executor(self, gen_dir: Path):
        cmake = (gen_dir / "CMakeLists.txt").read_text()
        assert "linguafranca_executor" not in cmake

    def test_cmake_define_main_trick(self, gen_dir: Path):
        # The #define main lf_unused_main_N is prepended into the staged source files,
        # not into CMakeLists.txt (which uses target_sources to pull them in).
        staged = list((gen_dir / "src" / "nodes").rglob("*.cpp"))
        assert staged, "no staged node source files found"
        for f in staged:
            assert "lf_unused_main" in f.read_text(), f"{f.name} missing #define main lf_unused_main_N"
            # O2-16: the renamed main has no return statement; its warning is silenced.
            assert '#pragma GCC diagnostic ignored "-Wreturn-type"' in f.read_text(), f.name

    def test_cmake_links_node_registry(self, gen_dir: Path):
        cmake = (gen_dir / "CMakeLists.txt").read_text()
        assert "ros2_lf_binding" in cmake

    def test_cmake_dep_include_dirs(self, gen_dir: Path):
        cmake = (gen_dir / "CMakeLists.txt").read_text()
        assert "pubsub_example_INCLUDE_DIRS" in cmake

    def test_node_source_staged(self, gen_dir: Path):
        staged = list((gen_dir / "src" / "nodes").rglob("*.cpp"))
        assert len(staged) == 2
        # Counting files is not enough: the earlier placeholder path produced two
        # 3-line stubs that satisfied the count and then failed to link.  Assert
        # the real source body actually made it into the staged copy.
        for path in staged:
            text = path.read_text()
            assert "PLACEHOLDER" not in text, f"{path.name} is a placeholder stub"
            assert "stand-in body for" in text, f"{path.name} lacks the node body"

    def test_package_xml_no_linguafranca_executor(self, gen_dir: Path):
        xml = (gen_dir / "package.xml").read_text()
        assert "linguafranca_executor" not in xml
