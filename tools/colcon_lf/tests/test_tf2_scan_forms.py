# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""tf2 construction forms the scan does not admit are refused (F91-1).

The scan recognises a Buffer or TransformListener only through
``std::make_shared|make_unique<...>``. Before F91-1 a node that made its
listener another way got no listener record and no refusal, so generation went
ahead and tf2_ros's private executor ingested /tf outside LF. Every form here
must now be refused with ``tf2_listener_unadmitted`` naming file:line.
"""

from __future__ import annotations

import pytest

from colcon_lf.node_analysis.tf2_scan import scan

HEADER = """
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<tf2_ros::TransformListener> lst_;
};
"""

VALUE_HEADER = """
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  tf2_ros::Buffer buf_{get_clock()};
  tf2_ros::TransformListener lst_{buf_, this};
};
"""

BUF = "  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());\n"
LST = "  lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this);\n"


def _cpp(body: str) -> str:
    return '#include "req.hpp"\nReq::Req() : Node("req") {\n' + body + "}\n"


def _refusals(header: str, body: str) -> list[str]:
    info = scan([("req.hpp", header), ("req.cpp", _cpp(body))], "Req")
    assert info is not None
    return [f"{r.subcode}: {r.detail}" for r in info.refusals]


# (id, header, constructor body, a file:line every refusal list must name)
FORMS = [
    ("reset_new", HEADER,
     BUF + "  lst_.reset(new tf2_ros::TransformListener(*buf_, this));\n", "req.cpp:4"),
    ("value_member", VALUE_HEADER, "", "req.hpp:8"),
    ("shared_ptr_new", HEADER,
     BUF + "  lst_ = std::shared_ptr<tf2_ros::TransformListener>(\n"
     "    new tf2_ros::TransformListener(*buf_, this));\n", "req.cpp:4"),
    ("helper", HEADER, BUF + "  lst_ = make_tf_listener(*buf_, this);\n", "req.hpp:9"),
    ("using_namespace", HEADER,
     "  using namespace tf2_ros;\n" + BUF
     + "  lst_ = std::make_shared<TransformListener>(*buf_, this);\n", "req.cpp:3"),
    ("namespace_alias", HEADER,
     "  namespace tf = tf2_ros;\n" + BUF
     + "  lst_ = std::make_shared<tf::TransformListener>(*buf_, this);\n", "req.cpp:3"),
    ("buffer_reset_new", HEADER,
     "  buf_.reset(new tf2_ros::Buffer(get_clock()));\n" + LST, "req.cpp:3"),
    ("buffer_allocate_shared", HEADER,
     "  buf_ = std::allocate_shared<tf2_ros::Buffer>(alloc, get_clock());\n" + LST, "req.cpp:3"),
]


@pytest.mark.parametrize("header,body,where", [f[1:] for f in FORMS], ids=[f[0] for f in FORMS])
def test_unadmitted_construction_form_is_refused(header, body, where):
    got = _refusals(header, body)
    assert got, ("tf2 scan accepted a Buffer/TransformListener constructed outside "
                 "std::make_shared/make_unique: generation would go ahead with no LF ingress")
    assert all(g.startswith("tf2_listener_unadmitted: ") for g in got), got
    assert any(where in g for g in got), got


def test_admitted_form_and_its_member_declarations_stay_admitted():
    assert _refusals(HEADER, BUF + LST) == []
    # SharedPtr aliases and a nullptr initializer are declarations too.
    header = HEADER.replace("std::shared_ptr<tf2_ros::Buffer> buf_;",
                            "tf2_ros::Buffer::SharedPtr buf_{nullptr};")
    assert _refusals(header, BUF + LST) == []


def test_in_class_make_shared_initializer_of_the_buffer_stays_admitted():
    """An in-class `= std::make_shared<tf2_ros::Buffer>(...)`
    initializer is the admitted form, not an unadmitted mention."""
    header = HEADER.replace(
        "std::shared_ptr<tf2_ros::Buffer> buf_;",
        "std::shared_ptr<tf2_ros::Buffer> buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());")
    assert _refusals(header, LST) == []


def test_another_class_making_a_same_named_member_does_not_vouch_for_it():
    """`Other` makes its own `buf_`; `Req`'s `buf_` comes from
    a helper the scan cannot see. One make_* for two declarations is refused."""
    other = ("class Other {\n std::shared_ptr<tf2_ros::Buffer> buf_;\n"
             " void init() { buf_ = std::make_shared<tf2_ros::Buffer>(get_clock()); }\n};\n")
    info = scan([("req.hpp", HEADER), ("req.cpp", _cpp("  buf_ = make_tf_buffer();\n" + LST)),
                 ("other.hpp", other)], "Req")
    got = [f"{r.subcode}: {r.detail}" for r in info.refusals]
    assert any("member 'buf_' is declared 2 times but made 1 time(s)" in g for g in got), (
        "tf2 scan let another class's make_shared vouch for Req's buf_: " + repr(got))


def test_declared_member_the_scan_never_sees_made_names_the_member():
    got = _refusals(HEADER, BUF + "  lst_ = make_tf_listener(*buf_, this);\n")
    assert got == [
        "tf2_listener_unadmitted: TransformListener at req.hpp:9: member 'lst_' is never made "
        "with std::make_shared/make_unique<tf2_ros::TransformListener>, so the scan cannot see "
        "how it is constructed"]
