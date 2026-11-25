# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The tf2 scan's user/system split and anchor types (F91c).

On 17fd949f a header was system, and never read by the tf2 scan, when its path
began with /usr/ or /opt/. A workspace under /opt (``WORKSPACE`` is
configurable) hid a user helper header that makes a TransformListener, so
generation went ahead. Now a header is system only under the compiler's system
include roots or a ROS underlay prefix, and never inside the workspace.

On 17fd949f the anchor refusal (an include of transform_listener.h or buffer.h
with no scanned construction) was skipped by any Buffer or listener record. An
include of transform_listener.h plus an admitted Buffer plus a listener made in
a compiled library was accepted. Now each anchor needs a record of its type.
"""

from __future__ import annotations

import os
import shutil
import tempfile
from pathlib import Path

import pytest

from .ros_env import requires_ros_headers
from .test_tf2_scan_anchor import (
    HELPER_HEADER, HELPER_NODE_HEADER, PACKAGE_XML, WORKSPACE, _assert_refused, _package,
    _tf2_refusals)

OPT_HELPER = "/opt/f91c_ws/src/req_pkg/include/req_pkg/tf_holder.hpp"


def test_workspace_header_under_opt_is_user_code(monkeypatch):
    """No bare /opt prefix: a header outside every system root is user code."""
    from colcon_lf.node_analysis.clang_parser import _tf2_file_kind

    monkeypatch.setenv("AMENT_PREFIX_PATH", "/opt/ros/jazzy")
    kind = _tf2_file_kind(OPT_HELPER)
    assert kind == "user", (
        f"generation accepted: the tf2 scan classes {OPT_HELPER} as {kind!r} and never "
        "reads it, so a TransformListener made there takes /tf outside LF")


def test_system_roots_come_from_flags_underlays_and_workspace(monkeypatch):
    from colcon_lf.node_analysis.clang_parser import _tf2_file_kind, _tf2_roots

    monkeypatch.setenv("AMENT_PREFIX_PATH", "/opt/ros/jazzy:/opt/ws/install/my_pkg")
    roots = _tf2_roots(["-isystem/opt/sdk/include", "-I/opt/ws/src/req_pkg/include"],
                       Path("/opt/ws"))
    assert roots.user == ("/opt/ws",)
    assert "/opt/ros/jazzy" in roots.system and "/opt/sdk/include" in roots.system
    assert _tf2_file_kind("/opt/ros/jazzy/include/tf2_ros/tf2_ros/buffer.h", roots) == "system"
    assert _tf2_file_kind("/opt/sdk/include/sdk.h", roots) == "system"
    # The workspace wins over an underlay entry naming its own install space.
    assert _tf2_file_kind("/opt/ws/install/my_pkg/include/my_pkg/a.hpp", roots) == "user"
    assert _tf2_file_kind("/opt/ws/src/req_pkg/include/req_pkg/tf_holder.hpp", roots) == "user"
    # Outside every root: user, even under /usr or /opt.
    assert _tf2_file_kind("/opt/other/helper.hpp", roots) == "user"
    assert _tf2_file_kind("/usr/src/ws/helper.hpp", roots) == "user"
    # Split flags count like joined ones.
    split = _tf2_roots(["-isystem", "/opt/sdk2/include", "-I", "/opt/ws/src/a/include"], None)
    assert "/opt/sdk2/include" in split.system
    assert _tf2_file_kind("/opt/sdk2/include/sdk.h", split) == "system"
    # A "/" root (a workspace at /) never makes everything one kind.
    assert _tf2_roots(["-isystem/"], Path("/")).user == ()


def test_workspace_at_root_is_refused_not_dropped():
    """X91r2 (b): on 4e867225 norm() dropped a "/" workspace root silently, so
    the workspace's own headers under a system root were never read."""
    from colcon_lf.node_analysis.clang_parser import _tf2_roots

    refusal = getattr(_tf2_roots([], Path("/")), "refusal", "")
    assert "workspace root is '/'" in refusal, (
        "generation accepted: a workspace root of '/' is dropped silently, and every "
        "workspace header under a system root goes unread by the tf2 scan")


def _prefix(root: Path, *packages: str) -> Path:
    index = root / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    for name in packages:
        (index / name).write_text("")
    return root


def test_install_space_outside_the_workspace_is_user_code(tmp_path, monkeypatch):
    """X91r2 (a): colcon build --install-base /elsewhere copies a workspace
    package's headers out of the workspace; on 4e867225 its prefix counted as a
    system root (AMENT_PREFIX_PATH, -isystem), so the tf2 scan never read them."""
    from colcon_lf.node_analysis.clang_parser import _tf2_file_kind, _tf2_roots

    ws = tmp_path / "ws"
    (ws / "src" / "helper_pkg").mkdir(parents=True)
    (ws / "src" / "helper_pkg" / "package.xml").write_text(
        PACKAGE_XML.replace("req_pkg", "helper_pkg"))
    own = _prefix(tmp_path / "elsewhere" / "helper_pkg", "helper_pkg")
    header = own / "include" / "helper_pkg" / "helper_pkg" / "tf_holder.hpp"
    monkeypatch.setenv("AMENT_PREFIX_PATH", f"{own}:/opt/ros/jazzy")
    roots = _tf2_roots([f"-isystem{own}/include/helper_pkg"], ws)
    kind = _tf2_file_kind(str(header), roots)
    assert kind == "user", (
        f"generation accepted: the tf2 scan classes {header} as {kind!r} and never reads "
        "it, so a TransformListener made in the workspace's own installed header takes "
        "/tf outside LF")
    assert roots.refusal == ""
    # An underlay that registers none of the workspace's packages stays system.
    under = _prefix(tmp_path / "underlay", "other_pkg")
    monkeypatch.setenv("AMENT_PREFIX_PATH", f"{under}")
    assert _tf2_file_kind(str(under / "include" / "x.hpp"), _tf2_roots([], ws)) == "system"


def test_prefix_mixing_workspace_and_foreign_packages_is_refused(tmp_path, monkeypatch):
    """A prefix holding a workspace package next to foreign ones cannot be split."""
    from colcon_lf.node_analysis.clang_parser import _tf2_roots

    ws = tmp_path / "ws"
    (ws / "src" / "helper_pkg").mkdir(parents=True)
    (ws / "src" / "helper_pkg" / "package.xml").write_text(
        PACKAGE_XML.replace("req_pkg", "helper_pkg"))
    # A package under COLCON_IGNORE is not a workspace package.
    (ws / "src" / "ignored").mkdir()
    (ws / "src" / "ignored" / "COLCON_IGNORE").write_text("")
    (ws / "src" / "ignored" / "package.xml").write_text(
        PACKAGE_XML.replace("req_pkg", "ignored_pkg"))
    mixed = _prefix(tmp_path / "merged", "helper_pkg", "rclcpp")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(mixed))
    roots = _tf2_roots([], ws)
    assert "holds the workspace package(s) helper_pkg next to 1 package(s)" in roots.refusal
    assert str(mixed.resolve()) not in roots.user
    only_ignored = _prefix(tmp_path / "ign", "ignored_pkg")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(only_ignored))
    assert _tf2_roots([], ws).refusal == ""
    assert str(only_ignored.resolve()) not in _tf2_roots([], ws).user


@requires_ros_headers("the tf2 system-root tests")
def test_toolchain_and_ros_headers_stay_system(monkeypatch):
    """The compiler's implicit dirs and the -I fallback's /opt/ros prefix are system."""
    monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)  # the -I dirs alone must do it
    from colcon_lf.node_analysis.clang_parser import (
        _tf2_file_kind, _tf2_roots, _toolchain_include_dirs)

    assert _toolchain_include_dirs(), "the C++ toolchain reported no include dirs"
    distro = next(Path("/opt/ros").iterdir())
    roots = _tf2_roots([f"-I{distro}/include/rclcpp"], None)
    assert str(distro.resolve()) in roots.system
    # The prefix itself as -I (split form too) is its own ament prefix.
    assert str(distro.resolve()) in _tf2_roots(["-I", str(distro)], None).system
    vector = next(Path(d) / "vector" for d in _toolchain_include_dirs()
                  if (Path(d) / "vector").is_file())
    assert _tf2_file_kind(str(vector), roots) == "system"
    assert _tf2_file_kind(f"{distro}/include/tf2_ros/tf2_ros/buffer.h", roots) == "system"


# The helper header makes the listener; neither the node's .cpp nor its header
# includes tf2_ros directly, so only reading the helper header sees it.
HELPER_ONLY_CPP = """#include "req_pkg/req_node.hpp"
Req::Req() : Node("req") {
  tf_.start_tf(this);
}
"""


@requires_ros_headers("the tf2 /opt workspace test")
@pytest.mark.skipif(not os.access("/opt", os.W_OK), reason="/opt is not writable")
def test_helper_header_of_a_workspace_under_opt_is_read():
    root = Path(tempfile.mkdtemp(prefix="f91c_ws_", dir="/opt"))
    try:
        pkg = root / "src" / "req_pkg"
        (pkg / "include" / "req_pkg").mkdir(parents=True)
        (pkg / "src").mkdir()
        (pkg / "package.xml").write_text(PACKAGE_XML)
        (pkg / "include" / "req_pkg" / "req_node.hpp").write_text(HELPER_NODE_HEADER)
        (pkg / "include" / "req_pkg" / "tf_holder.hpp").write_text(HELPER_HEADER)
        cpp = pkg / "src" / "req_node.cpp"
        cpp.write_text(HELPER_ONLY_CPP)
        from colcon_lf.node_analysis.clang_parser import parse_node

        tf2 = parse_node(cpp, WORKSPACE).tf2
        # No tf2 result at all is the accepted outcome: nothing was scanned.
        refusals = [f"{r.subcode}: {r.detail}" for r in tf2.refusals] if tf2 else []
        _assert_refused(refusals, "TransformListener lst_ at tf_holder.hpp:11")
    finally:
        shutil.rmtree(root, ignore_errors=True)


BUFFER_HEADER = """#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
// Defined in a compiled library: makes a TransformListener on *buffer.
std::shared_ptr<void> make_tf_listener(rclcpp::Node * node, void * buffer);
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<void> tf_;
};
"""

# transform_listener.h is included, the Buffer is admitted, and the listener is
# made by make_tf_listener in a compiled library the scan cannot read.
BUFFER_EXTERN_LISTENER_CPP = """#include "req_pkg/req_node.hpp"
#include "tf2_ros/transform_listener.h"
Req::Req() : Node("req") {
  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_ = make_tf_listener(this, buf_.get());
}
"""


@requires_ros_headers("the tf2 anchor-type tests")
def test_listener_include_needs_a_listener_record_not_a_buffer(tmp_path):
    cpp = _package(tmp_path, {"req_node.hpp": BUFFER_HEADER}, BUFFER_EXTERN_LISTENER_CPP)
    refusals = _tf2_refusals(cpp)
    _assert_refused(refusals, "req_node.cpp:2 includes tf2_ros/transform_listener.h",
                    "no tf2_ros::TransformListener made")
    # The Buffer anchor (req_node.hpp:4 includes tf2_ros/buffer.h) has its record,
    # and the Buffer itself is admitted: the listener anchor is the only refusal.
    assert len(refusals) == 1, refusals


@requires_ros_headers("the tf2 anchor-type tests")
def test_buffer_include_with_an_admitted_buffer_has_no_anchor_refusal(tmp_path):
    cpp_text = BUFFER_EXTERN_LISTENER_CPP.replace(
        '#include "tf2_ros/transform_listener.h"\n', "").replace(
        "  tf_ = make_tf_listener(this, *buf_);\n", "")
    cpp = _package(tmp_path, {"req_node.hpp": BUFFER_HEADER}, cpp_text)
    assert not [r for r in _tf2_refusals(cpp) if " includes tf2_ros/" in r]


# The node includes its helper package's header, which the out-of-tree install
# space holds; only reading that installed copy sees the listener it makes.
OWN_INSTALL_NODE_HEADER = HELPER_NODE_HEADER.replace(
    '"req_pkg/tf_holder.hpp"', '"helper_pkg/tf_holder.hpp"')


@requires_ros_headers("the tf2 out-of-tree install test")
def test_listener_in_out_of_tree_install_header_is_refused(tmp_path, monkeypatch):
    """X91r2 (a), through parse_node: the compile command passes the workspace
    package's installed headers as -isystem (CMake does so for an imported
    target) from an install base outside the workspace."""
    import json

    from colcon_lf.node_analysis.clang_parser import parse_node

    ws = tmp_path / "ws"
    helper_src = ws / "src" / "helper_pkg"
    (helper_src / "include" / "helper_pkg").mkdir(parents=True)
    (helper_src / "package.xml").write_text(PACKAGE_XML.replace("req_pkg", "helper_pkg"))
    (helper_src / "include" / "helper_pkg" / "tf_holder.hpp").write_text(HELPER_HEADER)
    own = _prefix(tmp_path / "elsewhere" / "helper_pkg", "helper_pkg")
    installed = own / "include" / "helper_pkg" / "helper_pkg"
    installed.mkdir(parents=True)
    (installed / "tf_holder.hpp").write_text(HELPER_HEADER)
    pkg = ws / "src" / "req_pkg"
    (pkg / "include" / "req_pkg").mkdir(parents=True)
    (pkg / "src").mkdir()
    (pkg / "package.xml").write_text(PACKAGE_XML)
    (pkg / "include" / "req_pkg" / "req_node.hpp").write_text(OWN_INSTALL_NODE_HEADER)
    cpp = pkg / "src" / "req_node.cpp"
    cpp.write_text(HELPER_ONLY_CPP)
    distro = next(Path("/opt/ros").iterdir())
    ros = [f"-isystem {d}" for d in sorted((distro / "include").iterdir()) if d.is_dir()]
    (ws / "build" / "req_pkg").mkdir(parents=True)
    (ws / "build" / "req_pkg" / "compile_commands.json").write_text(json.dumps([{
        "directory": str(ws / "build" / "req_pkg"), "file": str(cpp),
        "command": " ".join(["c++", "-std=c++17", f"-I{pkg / 'include'}",
                             f"-isystem {own / 'include' / 'helper_pkg'}", *ros,
                             "-c", str(cpp)])}]))
    monkeypatch.setenv("AMENT_PREFIX_PATH", f"{own}:{distro}")
    tf2 = parse_node(cpp, ws).tf2
    refusals = [f"{r.subcode}: {r.detail}" for r in tf2.refusals] if tf2 else []
    _assert_refused(refusals, "TransformListener lst_ at tf_holder.hpp:")
