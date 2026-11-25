# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""A main-only source file must name its namespaced node class.

``add_executable(x src/x_main.cpp)`` where ``x_main.cpp`` only constructs a
class declared in an included header inside a namespace.  The class has no
same-stem header, so the node takes the headerless path, whose appended
factory used to instantiate the bare class name, which is not in scope there.
"""
from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from colcon_lf.generation.factory_emitter import emit_headerless_factory_suffix
from colcon_lf.output.serializer import deserialize_system_graph
from colcon_lf.verb.generate import _emit_lfc_package

from .test_headerless_noncomponent_lfc import _system_json

_MAIN_ONLY_CPP = """\
#include "rclcpp/rclcpp.hpp"
#include "main_only_example/counter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<main_only_example::CounterNode>());
  rclcpp::shutdown();
  return 0;
}
"""


class TestFactorySuffix:
    def test_qualified_name_is_instantiated(self):
        suffix = emit_headerless_factory_suffix(
            0, "CounterNode", qualified_name="main_only_example::CounterNode")
        assert (
            'ros2_lf_construct_node_0<main_only_example::CounterNode>("CounterNode"'
        ) in suffix

    def test_empty_qualified_name_keeps_bare_class(self):
        suffix = emit_headerless_factory_suffix(0, "CounterNode")
        assert 'ros2_lf_construct_node_0<CounterNode>("CounterNode"' in suffix


@pytest.fixture
def gen_dir(tmp_path: Path):
    main_cpp = tmp_path / "counter_main.cpp"
    main_cpp.write_text(_MAIN_ONLY_CPP)

    data = _system_json(str(main_cpp))
    node = data["nodes"]["frontlidardriver"]
    node["class_name"] = "CounterNode"
    node["qualified_name"] = "main_only_example::CounterNode"
    node["package"] = "main_only_example"

    sj_path = tmp_path / "lf-analysis" / "main_only" / "counter" / "system.json"
    sj_path.parent.mkdir(parents=True)
    sj_path.write_text(json.dumps(data))
    graph = deserialize_system_graph(sj_path.read_text())

    from colcon_lf.generation.lf_emitter import emit_lf_system
    from colcon_lf.generation.lf_generator import generate_lf_system
    lf_system = generate_lf_system(graph, target_lfc=True)
    lf_file = sj_path.parent / "lf" / f"{lf_system.name}.lf"
    lf_file.parent.mkdir()
    lf_file.write_text(emit_lf_system(lf_system))

    pkg_name = f"lf_generated_{lf_system.name.lower()}"
    pkg_dir = tmp_path / "lf-generated" / pkg_name
    (pkg_dir / "lf").mkdir(parents=True)
    (pkg_dir / "src").mkdir()
    shutil.copy2(lf_file, pkg_dir / "lf" / lf_file.name)
    _emit_lfc_package(
        pkg_dir=pkg_dir, pkg_name=pkg_name, lf_system=lf_system,
        graph=graph, lf_file=lf_file,
    )
    return pkg_dir


def test_staged_main_only_source_uses_qualified_class(gen_dir: Path):
    staged = list((gen_dir / "src" / "nodes").rglob("counter_main.cpp"))
    assert len(staged) == 1
    text = staged[0].read_text()
    assert (
        'ros2_lf_construct_node_0<main_only_example::CounterNode>("CounterNode"'
    ) in text
    assert "ros2_lf_construct_node_0<CounterNode>" not in text
