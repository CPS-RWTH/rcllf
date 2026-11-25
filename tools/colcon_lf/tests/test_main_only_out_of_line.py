# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The three-file layout (header, node .cpp, main-only .cpp).

The class is declared in a header, its members are defined out of line in a
second file, and the launched executable's own source file holds only main().
The analyzer must find the member definitions in another translation unit of
the CMake target (or of a library it links), by clang USR, and must refuse
with a precise error when there is none, never bind an empty node.
"""

from __future__ import annotations

import json
import shlex
from pathlib import Path

import pytest

pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the out-of-line main-only parser tests")

_HEADER = """\
#ifndef LAYOUT_PKG__WORKER_NODE_HPP_
#define LAYOUT_PKG__WORKER_NODE_HPP_
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
namespace layout_pkg
{
class WorkerNode : public rclcpp::Node
{
public:
  WorkerNode();

private:
  void tick();
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace layout_pkg
#endif  // LAYOUT_PKG__WORKER_NODE_HPP_
"""

_NODE_CPP = """\
#include "layout_pkg/worker_node.hpp"
using namespace std::chrono_literals;
namespace layout_pkg
{
WorkerNode::WorkerNode()
: Node("worker")
{
  pub_ = create_publisher<std_msgs::msg::Int32>("out", 10);
  timer_ = create_wall_timer(100ms, std::bind(&WorkerNode::tick, this));
}
void WorkerNode::tick() {pub_->publish(std_msgs::msg::Int32());}
}  // namespace layout_pkg
"""

_MAIN_CPP = """\
#include <memory>
#include "layout_pkg/worker_node.hpp"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<layout_pkg::WorkerNode>());
  rclcpp::shutdown();
  return 0;
}
"""

# A same-named class in another namespace, defined in a file of the same
# target. Matching must be by USR, so this must not count as a definition.
_DECOY_CPP = """\
#include "rclcpp/rclcpp.hpp"
namespace other_pkg
{
struct WorkerNode { WorkerNode() {} void tick() {} };
}  // namespace other_pkg
"""


def _real_flags(packages_root: Path) -> str:
    """Compiler flags of a real rclcpp translation unit in the built workspace."""
    for ccj in sorted((packages_root / "build").glob("*/compile_commands.json")):
        for entry in json.loads(ccj.read_text()):
            toks = shlex.split(entry.get("command", ""))
            out, i = [], 0
            while i < len(toks):
                t = toks[i]
                if t == "-isystem" and i + 1 < len(toks):
                    out.append(f"-isystem {toks[i + 1]}")
                    i += 1
                elif t.startswith(("-I", "-isystem", "-std=", "-D")):
                    out.append(t)
                i += 1
            if any("rclcpp" in f for f in out):
                return " ".join(out)
    pytest.skip("no built package with rclcpp flags in packages/build")


def _workspace(tmp_path: Path, packages_root: Path, *, exe_sources, lib_sources,
               link_lib: bool, decoy: bool = False, with_node_cpp: bool = True):
    """Build a throwaway workspace; returns (workspace_root, main_cpp_path)."""
    ws = tmp_path / "ws"
    pkg = ws / "src" / "layout_pkg"
    (pkg / "include" / "layout_pkg").mkdir(parents=True)
    (pkg / "src").mkdir()
    (pkg / "include" / "layout_pkg" / "worker_node.hpp").write_text(_HEADER)
    files = {"worker_main.cpp": _MAIN_CPP, "decoy.cpp": _DECOY_CPP}
    if with_node_cpp:
        files["worker_node.cpp"] = _NODE_CPP
    for name, text in files.items():
        (pkg / "src" / name).write_text(text)

    build = ws / "build" / "layout_pkg"
    build.mkdir(parents=True)
    flags = _real_flags(packages_root) + f" -I{pkg / 'include'}"
    entries = []
    for target, sources in (("worker_exe", exe_sources), ("worker_lib", lib_sources)):
        for src in sources:
            if src not in files:
                continue
            entries.append({
                "directory": str(build),
                "command": f"/usr/bin/c++ {flags} -o CMakeFiles/{target}.dir/src/{src}.o "
                           f"-c {pkg / 'src' / src}",
                "file": str(pkg / "src" / src),
            })
    (build / "compile_commands.json").write_text(json.dumps(entries))
    (build / "CMakeFiles" / "worker_exe.dir").mkdir(parents=True)
    (build / "CMakeFiles" / "worker_lib.dir").mkdir(parents=True)
    link = "/usr/bin/c++ CMakeFiles/worker_exe.dir/src/worker_main.cpp.o -o worker_exe"
    if link_lib:
        link += " libworker_lib.so"
    (build / "CMakeFiles" / "worker_exe.dir" / "link.txt").write_text(link + "\n")
    return ws, pkg / "src" / "worker_main.cpp"


@pytest.fixture(autouse=True)
def _fresh_cache():
    from colcon_lf.node_analysis.clang_parser import clear_parse_node_cache
    clear_parse_node_cache()
    yield
    clear_parse_node_cache()


def _check_resolved(node, main_cpp: Path):
    assert node.class_name == "WorkerNode"
    assert node.qualified_name == "layout_pkg::WorkerNode"
    # Analysed in the file that defines the members, not the main-only file.
    assert node.source_file.name == "worker_node.cpp"
    assert [t.callback for t in node.timers] == ["tick"]
    assert [p.topic for p in node.publishers] == ["out"]


def test_member_file_listed_in_the_executable(tmp_path, packages_root):
    from colcon_lf.node_analysis.clang_parser import parse_node
    ws, main_cpp = _workspace(
        tmp_path, packages_root,
        exe_sources=["worker_main.cpp", "worker_node.cpp", "decoy.cpp"],
        lib_sources=[], link_lib=False)
    _check_resolved(parse_node(main_cpp, ws), main_cpp)


def test_member_file_in_a_linked_library(tmp_path, packages_root):
    from colcon_lf.node_analysis.clang_parser import parse_node
    ws, main_cpp = _workspace(
        tmp_path, packages_root,
        exe_sources=["worker_main.cpp"], lib_sources=["worker_node.cpp", "decoy.cpp"],
        link_lib=True)
    _check_resolved(parse_node(main_cpp, ws), main_cpp)


def test_missing_member_definition_is_refused(tmp_path, packages_root):
    """Only the same-named decoy in another namespace is in the target."""
    from colcon_lf.node_analysis.clang_parser import NodeSourceError, parse_node
    ws, main_cpp = _workspace(
        tmp_path, packages_root,
        exe_sources=["worker_main.cpp", "decoy.cpp"], lib_sources=[],
        link_lib=False, with_node_cpp=False)
    with pytest.raises(NodeSourceError) as err:
        parse_node(main_cpp, ws)
    assert "layout_pkg::WorkerNode" in str(err.value)
    assert "no constructor definition" in str(err.value)


def _generic_workspace(tmp_path: Path, packages_root: Path, *, targets: dict,
                       extra_files: dict | None = None, link_txt: dict | None = None,
                       build_ninja: str | None = None):
    """Workspace with arbitrary targets.

    *targets* maps CMake target name to a list of source file names;
    *link_txt* maps a target to the text of its link.txt (absent: no file);
    *build_ninja* is the text of build.ninja (absent: none).
    """
    ws = tmp_path / "ws"
    pkg = ws / "src" / "layout_pkg"
    (pkg / "include" / "layout_pkg").mkdir(parents=True)
    (pkg / "src").mkdir()
    (pkg / "include" / "layout_pkg" / "worker_node.hpp").write_text(_HEADER)
    files = {"worker_main.cpp": _MAIN_CPP, "worker_node.cpp": _NODE_CPP,
             "decoy.cpp": _DECOY_CPP}
    files.update(extra_files or {})
    for name, text in files.items():
        (pkg / "src" / name).write_text(text)
    build = ws / "build" / "layout_pkg"
    build.mkdir(parents=True)
    flags = _real_flags(packages_root) + f" -I{pkg / 'include'}"
    entries = []
    for target, sources in targets.items():
        (build / "CMakeFiles" / f"{target}.dir").mkdir(parents=True)
        for src in sources:
            entries.append({
                "directory": str(build),
                "command": f"/usr/bin/c++ {flags} -o CMakeFiles/{target}.dir/src/{src}.o "
                           f"-c {pkg / 'src' / src}",
                "file": str(pkg / "src" / src),
            })
    (build / "compile_commands.json").write_text(json.dumps(entries))
    for target, text in (link_txt or {}).items():
        (build / "CMakeFiles" / f"{target}.dir" / "link.txt").write_text(text + "\n")
    if build_ninja is not None:
        (build / "build.ninja").write_text(build_ninja)
    return ws, pkg / "src" / "worker_main.cpp"


def test_member_file_in_an_object_library(tmp_path, packages_root):
    """The executable links the object files of an OBJECT library."""
    from colcon_lf.node_analysis.clang_parser import parse_node
    ws, main_cpp = _generic_workspace(
        tmp_path, packages_root,
        targets={"worker_exe": ["worker_main.cpp"],
                 "worker_obj": ["worker_node.cpp", "decoy.cpp"]},
        link_txt={"worker_exe": "/usr/bin/c++ CMakeFiles/worker_exe.dir/src/"
                                "worker_main.cpp.o CMakeFiles/worker_obj.dir/src/"
                                "worker_node.cpp.o -o worker_exe"})
    _check_resolved(parse_node(main_cpp, ws), main_cpp)


def test_ninja_build_without_link_txt(tmp_path, packages_root):
    """Ninja writes no link.txt; the link edge in build.ninja names the library."""
    from colcon_lf.node_analysis.clang_parser import parse_node
    ninja = (
        "build libworker_lib.so: CXX_SHARED_LIBRARY_LINKER__worker_lib_ "
        "CMakeFiles/worker_lib.dir/src/worker_node.cpp.o\n"
        "build worker_exe: CXX_EXECUTABLE_LINKER__worker_exe_ "
        "CMakeFiles/worker_exe.dir/src/worker_main.cpp.o | libworker_lib.so\n"
        "  LINK_LIBRARIES = -Wl,-rpath,. libworker_lib.so\n"
    )
    ws, main_cpp = _generic_workspace(
        tmp_path, packages_root,
        targets={"worker_exe": ["worker_main.cpp"],
                 "worker_lib": ["worker_node.cpp", "decoy.cpp"]},
        build_ninja=ninja)
    _check_resolved(parse_node(main_cpp, ws), main_cpp)


def test_package_wide_fallback_without_link_records(tmp_path, packages_root):
    """No link.txt and no build.ninja: the one defining TU of the package is used."""
    from colcon_lf.node_analysis.clang_parser import parse_node
    ws, main_cpp = _generic_workspace(
        tmp_path, packages_root,
        targets={"worker_exe": ["worker_main.cpp"],
                 "worker_lib": ["worker_node.cpp", "decoy.cpp"]})
    _check_resolved(parse_node(main_cpp, ws), main_cpp)


def test_main_compiled_into_two_targets_with_different_definitions(tmp_path, packages_root):
    """The same main .cpp is in two executables; each links a different definition."""
    from colcon_lf.node_analysis.clang_parser import NodeSourceError, parse_node
    ws, main_cpp = _generic_workspace(
        tmp_path, packages_root,
        targets={"exe_a": ["worker_main.cpp"], "lib_a": ["worker_node.cpp"],
                 "exe_b": ["worker_main.cpp"], "lib_b": ["worker_node_b.cpp"]},
        extra_files={"worker_node_b.cpp": _NODE_CPP},
        link_txt={"exe_a": "/usr/bin/c++ CMakeFiles/exe_a.dir/src/worker_main.cpp.o "
                           "-o exe_a liblib_a.so",
                  "exe_b": "/usr/bin/c++ CMakeFiles/exe_b.dir/src/worker_main.cpp.o "
                           "-o exe_b liblib_b.so"})
    with pytest.raises(NodeSourceError) as err:
        parse_node(main_cpp, ws)
    msg = str(err.value)
    assert "layout_pkg::WorkerNode" in msg
    assert "worker_node.cpp" in msg and "worker_node_b.cpp" in msg
