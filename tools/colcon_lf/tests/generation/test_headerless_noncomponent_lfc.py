# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for lfc package generation of header-less non-component nodes.

Nodes whose class is defined directly in a .cpp (no header, global namespace)
— e.g. the autoware_reference_launch wrapper nodes — cannot be instantiated
from node_factories.cpp via a guessed "<package>/<className>.hpp" include.
Instead a factory function is appended to the staged source (where the class
is visible) and node_factories.cpp calls it through an extern declaration.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from colcon_lf.output.serializer import deserialize_system_graph
from colcon_lf.verb.generate import _emit_lfc_package

_NODE_CPP = """\
#include "rclcpp/rclcpp.hpp"

class FrontLidarDriver : public rclcpp::Node
{
public:
  FrontLidarDriver() : Node("FrontLidarDriver") {}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontLidarDriver>());
  rclcpp::shutdown();
  return 0;
}
"""


def _system_json(source_file: str, base_class: str = "rclcpp::Node") -> dict:
    return {
        "schema_version": 3,
        "name": "RefsysMini",
        "launch_graph": {
            "root": {
                "file_path": "/ws/src/autoware_reference_launch/launch/sys.launch.py",
                "package": "autoware_reference_launch",
                "arguments": [],
                "nodes": [],
                "includes": [],
                "groups": [],
                "errors": [],
                "context_states": [],
            }
        },
        "nodes": {
            "frontlidardriver": {
                "name": "frontlidardriver",
                "class_name": "FrontLidarDriver",
                "package": "autoware_reference_launch",
                "source_file": source_file,
                "header_file": None,
                "is_component": False,
                "callbacks": [],
                "timers": [{"period_ms": 100.0, "callback": "timer_callback",
                            "callback_group": "default", "member_variable": "timer_",
                            "created_in": "constructor", "destroyed_in": None,
                            "offset_ms": 0.0}],
                "subscribers": [],
                "publishers": [],
                "callback_groups": [{"name": "default", "type": "MutuallyExclusive"}],
                "base_class": base_class,
                "service_servers": [], "action_servers": [],
            },
        },
        "communication": {"topics": [], "services": [], "actions": []},
    }


@pytest.fixture
def gen_dir(tmp_path: Path):
    node_cpp = tmp_path / "front_lidar_driver.cpp"
    node_cpp.write_text(_NODE_CPP)

    sj_path = tmp_path / "lf-analysis" / "refsys" / "sys" / "system.json"
    sj_path.parent.mkdir(parents=True)
    sj_path.write_text(json.dumps(_system_json(str(node_cpp))))

    graph = deserialize_system_graph(sj_path.read_text())

    from colcon_lf.generation.lf_generator import generate_lf_system
    from colcon_lf.generation.lf_emitter import emit_lf_system
    lf_system = generate_lf_system(graph, target_lfc=True)
    lf_text = emit_lf_system(lf_system)

    lf_dir = sj_path.parent / "lf"
    lf_dir.mkdir()
    lf_file = lf_dir / f"{lf_system.name}.lf"
    lf_file.write_text(lf_text)

    pkg_name = f"lf_generated_{lf_system.name.lower()}"
    pkg_dir = tmp_path / "lf-generated" / pkg_name
    pkg_dir.mkdir(parents=True)
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


class TestHeaderlessNodeEmission:
    def test_no_guessed_header_include(self, gen_dir: Path):
        text = (gen_dir / "src" / "node_factories.cpp").read_text()
        assert "frontLidarDriver.hpp" not in text
        assert "autoware_reference_launch/" not in text

    def test_factory_extern_declared_and_called(self, gen_dir: Path):
        text = (gen_dir / "src" / "node_factories.cpp").read_text()
        # The factory now takes per-instance NodeOptions (design note section 4).
        # It registers itself and returns void: node_factories.cpp
        # never includes the headerless class's header, so it cannot name the
        # concrete type to receive a shared_ptr return value either.
        assert (
            "void ros2_lf_make_node_0("
            "const char * binding_key, const rclcpp::NodeOptions & options);"
        ) in text
        assert 'ros2_lf_make_node_0("FrontLidarDriver", ros2_lf_node_options(ros_args));' in text
        # Single instance of the class → plain class name as the binding key.
        assert "FrontLidarDriver__" not in text

    def test_no_namespace_qualified_make_shared(self, gen_dir: Path):
        text = (gen_dir / "src" / "node_factories.cpp").read_text()
        assert "autoware_reference_launch::FrontLidarDriver" not in text

    def test_staged_source_defines_factory(self, gen_dir: Path):
        staged = list((gen_dir / "src" / "nodes").rglob("*.cpp"))
        assert len(staged) == 1
        text = staged[0].read_text()
        assert text.startswith("#define main lf_unused_main_0")
        assert (
            "void ros2_lf_make_node_0("
            "const char * binding_key, const rclcpp::NodeOptions & options) {"
        ) in text
        # Identity realization: if constexpr over NodeOptions-constructibility,
        # inside a *template* so the untaken branch really is discarded.
        assert "template<typename C>" in text
        assert (
            "if constexpr (std::is_constructible_v<C, const rclcpp::NodeOptions &>)"
        ) in text
        assert 'ros2_lf_construct_node_0<FrontLidarDriver>("FrontLidarDriver"' in text
        # The factory registers itself (base-class-agnostic template) rather
        # than returning a value for node_factories.cpp to register.
        assert "ros2_lf_register_node(binding_key, " in text
        # Factory must come after the class definition
        assert text.index("class FrontLidarDriver") < text.index(
            "void ros2_lf_make_node_0(const char * binding_key"
        )


class TestHeaderlessLifecycleNodeEmission:
    """A headerless LifecycleNode-derived class cannot be spelled as
    ``shared_ptr<rclcpp::Node>`` in ``node_factories.cpp``'s extern declaration,
    because ``node_factories.cpp`` never includes the class's header (there is
    none) and so cannot name the concrete class either. The fix makes
    ``emit_headerless_factory_suffix`` base-class-agnostic: it never spells
    out a concrete node-pointer type on either side (the appended factory
    registers itself and returns void), so the same function works
    unmodified for a LifecycleNode-derived headerless class — verified here
    by calling it with a class name standing in for one.
    """

    def test_headerless_factory_never_declares_shared_ptr_rclcpp_node(self):
        from colcon_lf.generation.factory_emitter import emit_headerless_factory_suffix

        suffix = emit_headerless_factory_suffix(0, "LifecycleExampleNode")
        assert "std::shared_ptr<rclcpp::Node>" not in suffix
        assert "void ros2_lf_make_node_0(" in suffix
        assert "ros2_lf_register_node(binding_key, " in suffix
        assert 'ros2_lf_register_node(binding_key, "LifecycleExampleNode", ' in suffix
        assert "LifecycleExampleNode" in suffix


class TestTwoInstancesOfOneHeaderlessClass:
    """Probe P17g: two instances of one headerless class share one source file.

    Staging wrote the file once per instance, so the second write dropped the
    first instance's factory and the package failed to link with
    ``undefined reference to ros2_lf_make_node_0``. The file is now staged once
    with one appended factory per instance.
    """

    @pytest.fixture
    def two_gen_dir(self, tmp_path: Path):
        node_cpp = tmp_path / "front_lidar_driver.cpp"
        node_cpp.write_text(_NODE_CPP)
        sj = _system_json(str(node_cpp))
        second = json.loads(json.dumps(sj["nodes"]["frontlidardriver"]))
        second["name"] = "frontlidardriver_b"
        sj["nodes"]["frontlidardriver_b"] = second
        sj_path = tmp_path / "lf-analysis" / "refsys" / "sys" / "system.json"
        sj_path.parent.mkdir(parents=True)
        sj_path.write_text(json.dumps(sj))
        graph = deserialize_system_graph(sj_path.read_text())

        from colcon_lf.generation.lf_generator import generate_lf_system
        from colcon_lf.generation.lf_emitter import emit_lf_system
        lf_system = generate_lf_system(graph, target_lfc=True)
        lf_dir = sj_path.parent / "lf"
        lf_dir.mkdir()
        lf_file = lf_dir / f"{lf_system.name}.lf"
        lf_file.write_text(emit_lf_system(lf_system))
        pkg_name = f"lf_generated_{lf_system.name.lower()}"
        pkg_dir = tmp_path / "lf-generated" / pkg_name
        (pkg_dir / "lf").mkdir(parents=True)
        (pkg_dir / "src").mkdir()
        (pkg_dir / "lf" / lf_file.name).write_text(lf_file.read_text())
        _emit_lfc_package(
            pkg_dir=pkg_dir, pkg_name=pkg_name, lf_system=lf_system,
            graph=graph, lf_file=lf_file,
        )
        return pkg_dir

    def test_one_staged_file_defines_both_factories(self, two_gen_dir: Path):
        staged = list((two_gen_dir / "src" / "nodes").rglob("*.cpp"))
        assert len(staged) == 1
        text = staged[0].read_text()
        assert text.count("class FrontLidarDriver") == 1, "class staged twice"
        assert text.count("#define main ") == 1
        for idx in (0, 1):
            assert (
                f"void ros2_lf_make_node_{idx}("
                "const char * binding_key, const rclcpp::NodeOptions & options) {"
            ) in text, f"factory {idx} missing, link would fail on ros2_lf_make_node_{idx}"

    def test_node_factories_calls_both(self, two_gen_dir: Path):
        text = (two_gen_dir / "src" / "node_factories.cpp").read_text()
        assert "ros2_lf_make_node_0(" in text
        assert "ros2_lf_make_node_1(" in text


def _federate_staged(tmp_path: Path, nodes, topics, node_fed):
    """Run the real federated generator on *nodes* placed by *node_fed*."""
    from colcon_lf.federation.partition import FederationMap
    from colcon_lf.verb import generate as gen
    from ..corr_declared import declared
    from .test_correspondence_emission import _graph

    graph = _graph(nodes, topics)
    adir = tmp_path / "lf-analysis/test_pkg/bringup"
    adir.mkdir(parents=True, exist_ok=True)
    sj = adir / "system.json"
    sj.write_text(json.dumps({"name": "bringup", "nodes": {}, "communication": {"topics": []}}))
    fmap = FederationMap(node_to_federate=dict(node_fed),
                         federate_names=list(dict.fromkeys(node_fed.values())))
    rc = gen._generate_federated(system_json=sj, graph=graph, fmap=fmap, after_delay="0 msec",
                                 generate_launch=False, corr_bounds=declared(graph, 1))
    assert rc == 0


class TestTwoInstancesFederated:
    """The federated staging loop has the same two-instance rule.

    ``_generate_federated`` stages each federate's non-component sources in its
    own loop; before the fix the second instance's write dropped the first
    instance's factory there too.
    """

    def test_one_staged_file_defines_both_factories(self, tmp_path: Path):
        import dataclasses

        from .test_correspondence_emission import _pub, _sub, _tc
        from .test_xfed_correspondence import FED

        src = tmp_path / "packages/src/test_pkg/src"
        src.mkdir(parents=True)
        (src / "sub_node.cpp").write_text("// SubNode, two instances\n")
        (src / "pub.cpp").write_text("// PubNode\n")
        nodes = [dataclasses.replace(_pub("pub", "PubNode"), source_file=str(src / "pub.cpp"))]
        for name in ("s1", "s2"):
            nodes.append(dataclasses.replace(
                _sub(name, "SubNode"), source_file=str(src / "sub_node.cpp")))
        _federate_staged(tmp_path, nodes, [_tc("/data", ["pub"], ["s1", "s2"])],
                         {"pub": "pubs", "s1": "subs", "s2": "subs"})
        fed_src = tmp_path / "lf-generated" / f"{FED}_subs" / "src" / "nodes"
        staged = sorted(fed_src.rglob("*.cpp"))
        assert [p.name for p in staged] == ["sub_node.cpp"], staged
        text = staged[0].read_text()
        assert text.count("// SubNode, two instances") == 1, "source staged twice"
        assert text.count("#define main ") == 1
        for idx in (0, 1):
            assert f"void ros2_lf_make_node_{idx}(" in text, (
                f"factory {idx} missing, link would fail on ros2_lf_make_node_{idx}")
        factories = (fed_src.parent / "node_factories.cpp").read_text()
        assert "ros2_lf_make_node_0(" in factories and "ros2_lf_make_node_1(" in factories


class TestSameBasenameTwoSources:
    """Two different sources with one basename in one package.

    Before, the second source overwrote the first in the staged tree (link
    error); with the two-instance rule alone the second class's factory would
    land in the first class's file (compile error). Each source now gets its
    own staged file.
    """

    def test_each_source_staged_with_its_own_factory(self, tmp_path: Path):
        import dataclasses

        from colcon_lf.generation.lf_emitter import emit_lf_system
        from colcon_lf.generation.lf_generator import generate_lf_system
        from .test_correspondence_emission import _graph, _pub, _sub, _tc

        a = tmp_path / "talker" / "main.cpp"
        b = tmp_path / "listener" / "main.cpp"
        for f, cls in ((a, "PubNode"), (b, "SubNode")):
            f.parent.mkdir()
            f.write_text(f"// class {cls}\n")
        graph = _graph(
            [dataclasses.replace(_pub("pub", "PubNode"), source_file=str(a)),
             dataclasses.replace(_sub("sub", "SubNode"), source_file=str(b))],
            [_tc("/data", ["pub"], ["sub"])])
        lf_system = generate_lf_system(graph, target_lfc=True)
        pkg_dir = tmp_path / "lf-generated" / "pkg"
        (pkg_dir / "lf").mkdir(parents=True)
        lf_file = pkg_dir / "lf" / f"{lf_system.name}.lf"
        lf_file.write_text(emit_lf_system(lf_system))
        _emit_lfc_package(pkg_dir=pkg_dir, pkg_name="pkg", lf_system=lf_system,
                          graph=graph, lf_file=lf_file)
        staged = {p.name: p.read_text() for p in (pkg_dir / "src" / "nodes").rglob("*.cpp")}
        assert sorted(staged) == ["main.cpp", "main_1.cpp"], sorted(staged)
        assert "// class PubNode" in staged["main.cpp"]
        assert "void ros2_lf_make_node_0(" in staged["main.cpp"]
        assert "// class SubNode" in staged["main_1.cpp"]
        assert "void ros2_lf_make_node_1(" in staged["main_1.cpp"]
        assert "main_1.cpp" in (pkg_dir / "CMakeLists.txt").read_text()
