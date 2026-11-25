# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The tf2 scan cannot be bypassed by a macro or a header it never read (F91b).

On c1c3d2d7 the scan read the node's .cpp and ``include/**/<stem>.hpp`` as
text. A TransformListener spelled through a macro (``TF2_NS::TransformListener``)
or made by a helper in another header of the package produced no listener
record and no refusal, so generation went ahead and tf2_ros's private executor
took /tf outside LF. Now the scan reads every user header of the translation
unit, the compiler's own references to tf2_ros::Buffer and
tf2_ros::TransformListener must sit on lines the scan read, and a direct
include of the listener or buffer header with no records is refused.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from .ros_env import requires_ros_headers

REPO = Path(__file__).resolve().parents[3]
WORKSPACE = REPO / "packages"

pytestmark = requires_ros_headers("the tf2 anchor tests")

PACKAGE_XML = """<?xml version="1.0"?>
<package format="3"><name>req_pkg</name><version>0.0.0</version>
<description>F91b fixture</description><maintainer email="a@b.c">x</maintainer>
<license>Apache-2.0</license><depend>rclcpp</depend><depend>tf2_ros</depend></package>
"""

MACRO_HEADER = """#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#define TF2_NS tf2_ros
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<TF2_NS::TransformListener> lst_;
};
"""

MACRO_CPP = """#include "req_pkg/req_node.hpp"
Req::Req() : Node("req") {
  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  lst_ = std::make_shared<TF2_NS::TransformListener>(*buf_, this);
}
"""

# The helper header is not named after the .cpp, so the old scan never read it.
HELPER_HEADER = """#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
struct TfHolder {
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<tf2_ros::TransformListener> lst_;
  void start_tf(rclcpp::Node * node) {
    buf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_, node);
  }
};
"""

HELPER_NODE_HEADER = """#pragma once
#include "rclcpp/rclcpp.hpp"
#include "req_pkg/tf_holder.hpp"
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  TfHolder tf_;
};
"""

HELPER_CPP = """#include "req_pkg/req_node.hpp"
#include "tf2_ros/transform_listener.h"
Req::Req() : Node("req") {
  tf_.start_tf(this);
}
"""

# The listener is made in a library the package links (declared only here).
EXTERN_HEADER = """#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
std::shared_ptr<void> make_tf_listener(rclcpp::Node * node);
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  std::shared_ptr<void> tf_;
};
"""

EXTERN_CPP = """#include "req_pkg/req_node.hpp"
#include "tf2_ros/transform_listener.h"
Req::Req() : Node("req") {
  tf_ = make_tf_listener(this);
}
"""

# A macro listener on the same line as an admitted Buffer,
# stored type-erased, so no member declaration names it either.
SAME_LINE_HEADER = """#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#define TF2_NS tf2_ros
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<void> hidden_;
};
"""

SAME_LINE_CPP = """#include "req_pkg/req_node.hpp"
Req::Req() : Node("req") {
  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock()); hidden_ = std::make_shared<TF2_NS::TransformListener>(*buf_);
}
"""

ADMITTED_CPP = """#include "req_pkg/req_node.hpp"
Req::Req() : Node("req") {
  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this);
}
"""


def _package(tmp_path: Path, headers: dict[str, str], cpp: str) -> Path:
    pkg = tmp_path / "req_pkg"
    (pkg / "include" / "req_pkg").mkdir(parents=True)
    (pkg / "src").mkdir()
    (pkg / "package.xml").write_text(PACKAGE_XML)
    for name, text in headers.items():
        (pkg / "include" / "req_pkg" / name).write_text(text)
    cpp_path = pkg / "src" / "req_node.cpp"
    cpp_path.write_text(cpp)
    return cpp_path


def _tf2_refusals(cpp_path: Path) -> list[str]:
    from colcon_lf.node_analysis.clang_parser import parse_node

    info = parse_node(cpp_path, WORKSPACE)
    assert "tf2_ros" in info.detected_includes
    assert info.tf2 is not None, "no tf2 scan result"
    return [f"{r.subcode}: {r.detail}" for r in info.tf2.refusals]


def _assert_refused(refusals: list[str], *needles: str) -> None:
    assert refusals, (
        "generation accepted: the tf2 scan returned no refusal, so tf2_ros's private "
        "executor would take /tf outside LF")
    hits = [r for r in refusals if r.startswith("tf2_listener_unadmitted")
            and all(n in r for n in needles)]
    assert hits, refusals


def test_listener_spelled_through_a_macro_is_refused(tmp_path):
    cpp = _package(tmp_path, {"req_node.hpp": MACRO_HEADER}, MACRO_CPP)
    refusals = _tf2_refusals(cpp)
    _assert_refused(refusals, "tf2_ros::TransformListener at req_node.cpp:4", "macro")
    _assert_refused(refusals, "tf2_ros::TransformListener at req_node.hpp:12")


def test_macro_listener_on_the_line_of_an_admitted_buffer_is_refused(tmp_path):
    cpp = _package(tmp_path, {"req_node.hpp": SAME_LINE_HEADER}, SAME_LINE_CPP)
    refusals = _tf2_refusals(cpp)
    _assert_refused(refusals, "tf2_ros::TransformListener at req_node.cpp:3", "macro")


def test_listener_made_by_a_helper_in_an_unread_header_is_refused(tmp_path):
    cpp = _package(tmp_path, {"req_node.hpp": HELPER_NODE_HEADER,
                              "tf_holder.hpp": HELPER_HEADER}, HELPER_CPP)
    refusals = _tf2_refusals(cpp)
    # The helper header is read now, so its listener is seen and refused (it is
    # made outside Req's constructor and not given `this`).
    _assert_refused(refusals, "TransformListener lst_ at tf_holder.hpp:11")


def test_direct_listener_include_without_records_is_refused(tmp_path):
    cpp = _package(tmp_path, {"req_node.hpp": EXTERN_HEADER}, EXTERN_CPP)
    refusals = _tf2_refusals(cpp)
    _assert_refused(refusals, "req_node.cpp:2 includes tf2_ros/transform_listener.h",
                    "no tf2_ros::TransformListener made")


def test_admitted_form_in_a_fixture_package_stays_admitted(tmp_path):
    header = MACRO_HEADER.replace("#define TF2_NS tf2_ros\n", "").replace(
        "TF2_NS::", "tf2_ros::")
    cpp = _package(tmp_path, {"req_node.hpp": header}, ADMITTED_CPP)
    assert _tf2_refusals(cpp) == []


@pytest.mark.parametrize("rel, listeners, buffers", [
    ("tf2_acq_example/src/acq_sampler_node.cpp", ["listener_"], ["buffer_"]),
    ("tf2_acq_example/src/tf_source_node.cpp", [], []),
    ("fixtures/tf_broadcaster_example/src/dynamic_tf_broadcaster_node.cpp", [], []),
    ("fixtures/tf_broadcaster_example/src/static_tf_broadcaster_node.cpp", [], []),
])
def test_repo_tf2_nodes_keep_their_outcome(rel, listeners, buffers):
    """The repo's admitted tf2 nodes stay admitted (tf_listener_example stays
    refused with tf2_direct_lookup only, test_tf2_scan.py)."""
    from colcon_lf.node_analysis.clang_parser import parse_node

    info = parse_node(WORKSPACE / "src" / rel, WORKSPACE)
    assert info.tf2 is not None
    assert info.tf2.refusals == []
    assert [x.member for x in info.tf2.listeners] == listeners
    assert [x.member for x in info.tf2.buffers] == buffers
