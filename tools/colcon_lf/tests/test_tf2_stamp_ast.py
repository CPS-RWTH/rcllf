# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Request stamp provenance on the compiler's AST.

On f0e516d2 the stamp of ``<adapter>->request(target, source, stamp, cb)``
was classified on the source text: names were matched, not resolved. Four
forms passed as admitted although the stamp comes from elsewhere:

* B3: ``auto alias = lk_; alias->request(..., foreign, cb)`` is no
  ``lk_->request`` call, so it was never scanned;
* B4: ``this->stamp_`` resolved to a same-named local's initializer;
* B5: ``msg->header.stamp`` took the subscription's provenance although a
  nested ``auto msg = external_;`` shadows the parameter;
* B6: an unqualified ``now()`` was the node clock even when ``now`` is a
  local lambda or a parameter.

Now every name is the declaration clang resolved it to. Each test of B3-B6
fails on f0e516d2 with "generation accepted".
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from .ros_env import requires_ros_headers
from .test_tf2_scan_anchor import WORKSPACE, _package

pytestmark = requires_ros_headers("the tf2 request stamp AST tests")

REPO = Path(__file__).resolve().parents[3]

HEADER = """#pragma once
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rcllf_tf2/acquisition_lookup.hpp"
class Req : public rclcpp::Node {
 public:
  Req();
 private:
  void on_tick();
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_ext(const rclcpp::Time & stamp);
  void on_fn(const std::function<rclcpp::Time()> & now);
  std::shared_ptr<tf2_ros::Buffer> buf_;
  std::shared_ptr<tf2_ros::TransformListener> lst_;
  std::shared_ptr<rcllf_tf2::AcquisitionLookup> lk_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub2_;
  sensor_msgs::msg::Imu::SharedPtr external_;
  std::function<void(sensor_msgs::msg::Imu::ConstSharedPtr)> handler_;
  rclcpp::Time stamp_;
  rclcpp::Time t_;
};
"""

# Line 9 is the first line of the constructor's extra body (@CTOR@).
CPP = """#include "req_pkg/req_node.hpp"
using namespace std::chrono_literals;
static const auto cb = [](const rcllf_tf2::Result &) {};
Req::Req() : Node("req") {
  buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  lst_ = std::make_shared<tf2_ros::TransformListener>(*buf_, this);
  lk_ = std::make_shared<rcllf_tf2::AcquisitionLookup>(*buf_, this, 60ms, rcllf_tf2::OnTimeout::reject());
  sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&Req::on_imu, this, std::placeholders::_1));
@CTOR@
}
void Req::on_tick() {
@TICK@
}
void Req::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  (void)msg;
@IMU@
}
void Req::on_ext(const rclcpp::Time & stamp) {
  (void)stamp;
@EXT@
}
void Req::on_fn(const std::function<rclcpp::Time()> & now) {
  (void)now;
@FN@
}
"""

ADMITTED = ("node_clock", "message")


def _tf2(tmp_path, ctor="", tick="", imu="", ext="", fn="", header=HEADER, tail=""):
    from colcon_lf.node_analysis.clang_parser import parse_node

    text = (CPP.replace("@CTOR@", ctor).replace("@TICK@", tick).replace("@IMU@", imu)
            .replace("@EXT@", ext).replace("@FN@", fn)) + tail
    cpp = _package(tmp_path, {"req_node.hpp": header}, text)
    info = parse_node(cpp, WORKSPACE)
    assert info.tf2 is not None, "no tf2 scan result"
    return info.tf2


def _line(marker: str, body: str, offset: int = 0) -> int:
    """The line of *body*'s first line, put at *marker* in ``CPP``."""
    head = CPP[:CPP.index(marker)]
    return head.count("\n") + 1 + offset


def _problems(tf2) -> list[str]:
    """What refuses generation: the scan's refusals and every request stamp
    the resolved model does not admit (resolve.py ``_tf2_stamp_record``)."""
    out = [f"{r.subcode}: {r.detail}" for r in tf2.refusals]
    for s in tf2.stamps or []:
        if s.kind not in ADMITTED or (s.kind == "message" and not (s.callback or s.topic)):
            out.append(f"{s.kind}: {s.adapter} at {s.file}:{s.line}: {s.detail}")
    return out


def _assert_refused(tf2, *needles: str) -> None:
    problems = _problems(tf2)
    assert problems, (
        "generation accepted: every request stamp classified admitted ("
        + "; ".join(f"{s.adapter} {s.kind} '{s.expr}' = {s.detail}" for s in tf2.stamps or [])
        + ")")
    assert any(all(n in p for n in needles) for p in problems), problems


# One translation unit for every form (a parse takes seconds): each request
# in a block of its own, so a local of one does not reach another.
FORMS = [
    ('lk_->request("a", "b", this->now(), cb);', "node_clock"),
    ('lk_->request("a", "b", now(), cb);', "node_clock"),
    ('lk_->request("a", "b", get_clock()->now() - 5ms, cb);', "node_clock"),
    ('auto t = rclcpp::Time(now()) + rclcpp::Duration(std::chrono::milliseconds(3)); '
     'lk_->request("a", "b", t, cb);', "node_clock"),
    ('lk_->request("a", "b", rclcpp::Clock(RCL_SYSTEM_TIME).now(), cb);', "foreign_clock"),
    ('lk_->request("a", "b", rclcpp::Time(0, 0, RCL_ROS_TIME), cb);', "unknown"),
    ('lk_->request("a", "b", stamp_, cb);', "unknown"),
]


def test_request_stamp_is_classified(tmp_path):
    tick = "\n".join(f"  {{ {form} }}" for form, _kind in FORMS)
    tf2 = _tf2(tmp_path, tick=tick)
    first = _line("@TICK@", tick)
    got = [(s.adapter, s.kind, s.callback, s.file, s.line) for s in tf2.stamps]
    assert got == [("lk_", kind, None, "req_node.cpp", first + i)
                   for i, (_form, kind) in enumerate(FORMS)], [s.detail for s in tf2.stamps]
    assert tf2.refusals == []


def test_message_stamp_in_a_member_callback_and_in_a_subscription_lambda(tmp_path):
    tf2 = _tf2(
        tmp_path,
        ctor=('  sub2_ = create_subscription<sensor_msgs::msg::Imu>("/imu2", 10,\n'
              '    [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {\n'
              '      lk_->request("a", "b", rclcpp::Time(m->header.stamp), cb);\n    });'),
        imu=('  const rclcpp::Time st = msg->header.stamp;\n'
             '  lk_->request("a", "b", st, cb);'))
    assert [(s.kind, s.callback, s.topic) for s in tf2.stamps] == [
        ("message", None, "/imu2"), ("message", "on_imu", None)], [s.detail for s in tf2.stamps]
    assert _problems(tf2) == []


def test_member_stamp_set_from_two_sources_is_unknown(tmp_path):
    tf2 = _tf2(tmp_path, ctor="  t_ = this->now();",
               imu='  t_ = msg->header.stamp;\n  lk_->request("a", "b", t_, cb);')
    [st] = tf2.stamps
    assert st.kind == "unknown" and st.detail.startswith("'t_' is set from different sources")


def test_tf2_acq_example_stamp_is_the_node_clock():
    from colcon_lf.node_analysis.clang_parser import parse_node

    info = parse_node(REPO / "packages" / "src" / "tf2_acq_example" / "src"
                      / "acq_sampler_node.cpp", WORKSPACE)
    assert [(s.adapter, s.kind, s.expr) for s in info.tf2.stamps] == [
        ("lookup_", "node_clock", "stamp")]
    assert info.tf2.stamps[0].detail == "'stamp' = 'start_' = the node's own clock (now())"
    assert info.tf2.refusals == []


def test_parameter_stamp_is_unknown(tmp_path):
    tf2 = _tf2(tmp_path, tick="  auto stamp = this->now();\n  (void)stamp;",
               ext='  lk_->request("a", "b", stamp, cb);')
    [st] = tf2.stamps
    assert st.kind == "unknown" and "parameter" in st.detail, st.detail


def test_local_stamp_is_resolved_in_its_own_function_only(tmp_path):
    tf2 = _tf2(tmp_path,
               tick=('  const rclcpp::Time stamp = this->now();\n'
                     '  lk_->request("a", "b", stamp, cb);'),
               imu=('  const rclcpp::Time stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();\n'
                    '  lk_->request("a", "b", stamp, cb);'))
    assert [s.kind for s in tf2.stamps] == ["node_clock", "foreign_clock"]


def test_lambda_outside_create_subscription_gets_no_topic(tmp_path):
    tf2 = _tf2(tmp_path, ctor=(
        '  handler_ = [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {\n'
        '    lk_->request("a", "b", m->header.stamp, cb);\n  };'))
    [st] = tf2.stamps
    assert (st.kind, st.topic, st.callback) == ("unknown", None, None), st.detail


def test_member_changed_through_a_non_const_reference_is_unknown(tmp_path):
    tf2 = _tf2(tmp_path, ctor="  t_ = this->now();",
               tick=('  rclcpp::Time & r = t_;\n  r = rclcpp::Clock(RCL_SYSTEM_TIME).now();\n'
                     '  lk_->request("a", "b", t_, cb);'))
    [st] = tf2.stamps
    assert st.kind == "unknown" and "non-const reference 'r'" in st.detail, st.detail


# --- B3-B6: fail-open forms of f0e516d2 -------------------------------------

def test_b3_request_through_an_alias_of_the_adapter_is_refused(tmp_path):
    tf2 = _tf2(tmp_path, tick=(
        "  auto alias = lk_;\n"
        '  lk_->request("a", "b", this->now(), cb);\n'
        '  alias->request("a", "b", rclcpp::Clock(RCL_SYSTEM_TIME).now(), cb);'))
    _assert_refused(tf2, "tf2_stamp_unknown", "'alias'", f"req_node.cpp:{_line('@TICK@', '') + 2}")


def test_b4_member_stamp_does_not_take_a_same_named_local(tmp_path):
    tf2 = _tf2(tmp_path, ctor="  stamp_ = rclcpp::Clock(RCL_SYSTEM_TIME).now();",
               tick=("  const rclcpp::Time stamp_ = this->now();\n  (void)stamp_;\n"
                     '  lk_->request("a", "b", this->stamp_, cb);'))
    _assert_refused(tf2, "foreign_clock", "'stamp_' = ")


def test_b5_header_stamp_of_a_local_shadowing_the_message_is_refused(tmp_path):
    tf2 = _tf2(tmp_path, imu=("  {\n    auto msg = external_;\n"
                              '    lk_->request("a", "b", msg->header.stamp, cb);\n  }'))
    _assert_refused(tf2, "unknown", "header of 'msg', a variable")


@pytest.mark.parametrize(("where", "body"), [
    ("tick", "  auto now = [this]() { return rclcpp::Clock(RCL_SYSTEM_TIME).now(); };\n"
             '  lk_->request("a", "b", now(), cb);'),
    ("fn", '  lk_->request("a", "b", now(), cb);'),
])
def test_b6_now_that_is_a_local_lambda_or_a_parameter_is_refused(tmp_path, where, body):
    tf2 = _tf2(tmp_path, **{where: body})
    _assert_refused(tf2, "unknown", "'now()'")


REF_HEADER = HEADER.replace("  rclcpp::Time t_;\n",
                            "  rclcpp::Time t_;\n  rclcpp::Time & ref() { return stamp_; }\n")
FOREIGN = "rclcpp::Clock(RCL_SYSTEM_TIME).now()"


@pytest.mark.parametrize("write", [
    f"static_cast<rclcpp::Time &>(stamp_) = {FOREIGN};",
    f"(rclcpp::Time &)stamp_ = {FOREIGN};",
    f"ref() = {FOREIGN};",
], ids=["static_cast", "c_cast", "reference_accessor"])
def test_luna_write_through_a_cast_or_a_reference_accessor_is_refused(tmp_path, write):
    """A use the scan does not know was
    taken as a read, so a write through a mutable cast or a non-const
    reference return left 'stamp_' = this->now() and admitted the request."""
    tf2 = _tf2(tmp_path, ctor="  stamp_ = this->now();", header=REF_HEADER,
               tick=f'  {write}\n  lk_->request("a", "b", stamp_, cb);')
    _assert_refused(tf2, "unknown", "'stamp_'")


def test_reads_through_a_const_cast_reference_and_value_return_stay_admitted(tmp_path):
    header = HEADER.replace("  rclcpp::Time t_;\n",
                            "  rclcpp::Time t_;\n  rclcpp::Time get() const { return stamp_; }\n")
    tf2 = _tf2(tmp_path, ctor="  stamp_ = this->now();", header=header, tick=(
        "  const rclcpp::Time & r = stamp_;\n"
        "  const auto c = static_cast<const rclcpp::Time &>(stamp_);\n"
        "  const auto v = static_cast<rclcpp::Time>(stamp_);\n"
        "  (void)r; (void)c; (void)v; (void)get();\n"
        '  lk_->request("a", "b", stamp_, cb);'))
    assert _problems(tf2) == []
    assert [s.kind for s in tf2.stamps] == ["node_clock"]


def test_adapter_the_compiler_does_not_resolve_is_refused_not_read_from_text(tmp_path):
    """No AST of the adapter (its header is not on the include path): the
    stamp is not classified at all, and the node is refused."""
    header = HEADER.replace('#include "rcllf_tf2/acquisition_lookup.hpp"',
                            '#include "rcllf_tf2_missing/acquisition_lookup.hpp"')
    tf2 = _tf2(tmp_path, header=header, tick='  lk_->request("a", "b", this->now(), cb);')
    _assert_refused(tf2, "tf2_stamp_unknown", "does not resolve AcquisitionLookup member(s) lk_")


# --- F91e: forms admitted on c2bc47c7 -------------------------------------
# Each refused form below fails on c2bc47c7 with "generation accepted".

RAW_HEADER = HEADER.replace(
    "  void on_tick();\n",
    "  void on_tick();\n  void on_raw(sensor_msgs::msg::Imu::SharedPtr msg);\n")
RAW_CPP = ("void Req::on_raw(sensor_msgs::msg::Imu::SharedPtr msg) {\n"
           "@RAW@\n}\n")
LAMBDA = ('  sub2_ = create_subscription<sensor_msgs::msg::Imu>("/imu2", 10,\n'
          '    [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {\n@BODY@\n    });')


@pytest.mark.parametrize(("where", "body", "needle"), [
    ("raw", '  msg = external_;\n  lk_->request("a", "b", msg->header.stamp, cb);',
     "'msg' is assigned"),
    ("raw", '  std::swap(msg, external_);\n  lk_->request("a", "b", msg->header.stamp, cb);',
     "'msg' is passed by non-const reference to swap"),
    ("imu", f'  msg->header.stamp = {FOREIGN};\n  lk_->request("a", "b", msg->header.stamp, cb);',
     "'msg' is assigned"),
    ("imu", f'  auto & h = msg->header;\n  h.stamp = {FOREIGN};\n'
            '  lk_->request("a", "b", msg->header.stamp, cb);',
     "'msg' is bound to the non-const reference 'h'"),
    ("imu", f'  auto keep = msg;\n  keep->header.stamp = {FOREIGN};\n'
            '  lk_->request("a", "b", msg->header.stamp, cb);',
     "'msg' is used other than through '->' or '*'"),
    ("lambda", '      m = external_;\n      lk_->request("a", "b", m->header.stamp, cb);',
     "'m' is assigned"),
], ids=["assign_pointer", "swap", "write_stamp", "mutable_header_ref", "pointer_alias",
        "lambda_assign"])
def test_sol1_written_message_parameter_loses_message_provenance(tmp_path, where, body,
                                                                 needle):
    """Any parameter of the callback was the subscription's
    message, even after the body wrote it or the message it points to."""
    if where == "lambda":
        tf2 = _tf2(tmp_path, ctor=LAMBDA.replace("@BODY@", body))
    elif where == "raw":
        tf2 = _tf2(tmp_path, header=RAW_HEADER, tail=RAW_CPP.replace("@RAW@", body))
    else:
        tf2 = _tf2(tmp_path, imu=body)
    _assert_refused(tf2, "unknown", needle, "may no longer hold the subscription's message")


def test_sol1_reads_of_the_message_keep_its_provenance(tmp_path):
    """Reads stay admitted: a const message through a copied pointer, a
    const reference to its header, and the real RCL_ROS_TIME clock argument."""
    tf2 = _tf2(tmp_path, ctor=LAMBDA.replace("@BODY@", (
        "      const auto keep = m;\n      const auto & h = m->header;\n"
        "      (void)keep; (void)h;\n"
        '      lk_->request("a", "b", rclcpp::Time(m->header.stamp, RCL_ROS_TIME), cb);')),
        imu='  const auto & h = msg->header;\n  (void)h;\n'
            '  lk_->request("a", "b", msg->header.stamp, cb);')
    assert [(s.kind, s.callback, s.topic) for s in tf2.stamps] == [
        ("message", None, "/imu2"), ("message", "on_imu", None)], [s.detail for s in tf2.stamps]
    assert _problems(tf2) == []


def test_sol2_local_constant_named_rcl_ros_time_is_refused(tmp_path):
    """The clock argument passed when its spelling ended in
    RCL_ROS_TIME; a local constant of that name holding another clock passed."""
    tf2 = _tf2(tmp_path, imu=(
        "  const rcl_clock_type_t RCL_ROS_TIME = RCL_SYSTEM_TIME;\n"
        '  lk_->request("a", "b", rclcpp::Time(msg->header.stamp, RCL_ROS_TIME), cb);'))
    _assert_refused(tf2, "unknown", "cannot resolve to rcl's RCL_ROS_TIME enumerator")


OP_HEADER = HEADER + (
    "inline rclcpp::Time operator+(const rclcpp::Time &, std::chrono::milliseconds) {\n"
    "  return rclcpp::Clock(RCL_SYSTEM_TIME).now();\n}\n")


def test_spelling_user_time_operator_is_not_rclcpp_arithmetic(tmp_path):
    """Audit (item 2's sweep): operator+/- was admitted by its spelling; a
    node's own exact-match overload returning another clock passed."""
    tf2 = _tf2(tmp_path, header=OP_HEADER, tick='  lk_->request("a", "b", now() + 5ms, cb);')
    _assert_refused(tf2, "unknown", "not rclcpp's own time arithmetic")


SUB_HEADER = HEADER.replace("  void on_tick();\n", (
    "  void on_tick();\n"
    "  template<class M, class F>\n"
    "  typename rclcpp::Subscription<M>::SharedPtr create_subscription(\n"
    "    const std::string &, const rclcpp::QoS &, F &&) { return nullptr; }\n"))


def test_spelling_own_create_subscription_gives_no_topic(tmp_path):
    """Audit: the lambda's topic came from any call spelled
    create_subscription; the node's own member of that name (which may call
    the lambda with any message) gave it the topic '/imu2'."""
    tf2 = _tf2(tmp_path, header=SUB_HEADER, ctor=LAMBDA.replace(
        "@BODY@", '      lk_->request("a", "b", m->header.stamp, cb);'))
    _assert_refused(tf2, "unknown", "no subscription callback or create_subscription lambda")


FAKE_HEADER = HEADER.replace('#include "rcllf_tf2/acquisition_lookup.hpp"', (
    "namespace fake { namespace rcllf_tf2 {\n"
    "struct Result {};\n"
    "struct OnTimeout { static OnTimeout reject() { return {}; } };\n"
    "struct AcquisitionLookup {\n"
    "  template<class... A> explicit AcquisitionLookup(A &&...) {}\n"
    "  template<class... A> void request(A &&...) {}\n};\n"
    "} }\n"
    "namespace rcllf_tf2 = fake::rcllf_tf2;"))


def test_spelling_adapter_type_is_the_resolved_class(tmp_path):
    """Audit: a member was an adapter when its type's spelling contained
    'rcllf_tf2::AcquisitionLookup'; fake::rcllf_tf2::AcquisitionLookup passed,
    and its requests were then skipped unscanned."""
    tf2 = _tf2(tmp_path, header=FAKE_HEADER,
               tick=f'  lk_->request("a", "b", {FOREIGN}, cb);')
    _assert_refused(tf2, "tf2_stamp_unknown", "does not resolve AcquisitionLookup member(s) lk_")


READ = '  lk_->request("a", "b", msg->header.stamp, cb);'


@pytest.mark.parametrize(("kw", "needle"), [
    (dict(tick="  on_imu(external_);", imu=READ),
     "on_imu() is used at line"),
    (dict(ctor="  handler_ = std::bind(&Req::on_imu, this, external_);", imu=READ),
     "on_imu() is used at line"),
    (dict(header=RAW_HEADER, tail=RAW_CPP.replace("@RAW@", READ)),
     "on_raw() is never passed to create_subscription"),
], ids=["direct_call", "bound_to_a_fixed_message", "never_subscribed"])
def test_luna_method_parameter_needs_a_proven_subscription(tmp_path, kw, needle):
    """A member function's parameter
    was the subscription's message although the node calls the subscribed
    method itself or binds it to a fixed message (resolve matches the
    callback by name, so both passed), or never subscribes it (resolve
    refuses that one; the scan now refuses it first)."""
    tf2 = _tf2(tmp_path, **kw)
    _assert_refused(tf2, "unknown", needle, "may be any message")


# F91f (on c2bc47c7..efc04b8f): the clock type argument was
# rcl's RCL_ROS_TIME when its enumerator lay outside the node's files in an
# enum *named* rcl_clock_type_e, and _peel stripped a written cast first. A
# dependency header's ``namespace fake { enum rcl_clock_type_e { RCL_ROS_TIME
# = 2 }; }`` cast to rcl_clock_type_t passed. The tests fail on efc04b8f with
# "generation accepted".
FAKE_ENUM = ("#pragma once\n"
             "namespace fake {\n"
             "enum rcl_clock_type_e { RCL_CLOCK_UNINITIALIZED = 0, RCL_ROS_TIME = 2 /* system */ };\n"
             "}\n")
FAKE_GLOBAL = ('#pragma once\nextern "C" {\n'
               "typedef enum rcl_clock_type_e { RCL_ROS_TIME = 2 } rcl_clock_type_t;\n}\n")


def _dependency(tmp_path, monkeypatch, name: str, text: str) -> Path:
    """*text* as a header of a dependency prefix outside the workspace: on
    AMENT_PREFIX_PATH, so the tf2 scan counts it as system code, not as the
    node's files."""
    prefix = tmp_path / "dep_prefix"
    path = prefix / "include" / "fake_dep" / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    monkeypatch.setenv("AMENT_PREFIX_PATH", os.pathsep.join(
        [str(prefix)] + [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]))
    return path


def test_sol_f91f_namespaced_enum_named_like_rcl_is_refused(tmp_path, monkeypatch):
    """Item a): fake::RCL_ROS_TIME (value 2, rcl's RCL_SYSTEM_TIME) in a
    namespace of a dependency header, cast to rcl_clock_type_t, put the
    stamp on the system clock and passed as the node's ROS time."""
    dep = _dependency(tmp_path, monkeypatch, "clock.hpp", FAKE_ENUM)
    tf2 = _tf2(tmp_path, header=HEADER + f'#include "{dep}"\n', imu=(
        '  lk_->request("a", "b", rclcpp::Time(msg->header.stamp, '
        "static_cast<rcl_clock_type_t>(fake::RCL_ROS_TIME)), cb);"))
    _assert_refused(tf2, "unknown", "cannot resolve to rcl's RCL_ROS_TIME enumerator")


@pytest.mark.parametrize("arg", [
    "static_cast<rcl_clock_type_t>(RCL_ROS_TIME)",
    "(rcl_clock_type_t)RCL_ROS_TIME",
    "rcl_clock_type_t(RCL_ROS_TIME)",
], ids=["static_cast", "c_style", "functional"])
def test_sol_f91f_written_cast_is_refused(tmp_path, arg):
    """Item b): a written cast is the one way another enum's constant or any
    integer reaches the rcl_clock_type_t parameter, so the rule cannot depend
    on what lies under it. rcl's own RCL_ROS_TIME under a no-op cast is the
    form that isolates b) from a) (the enum is rcl's, so only the cast rule
    can refuse it); dropping the cast admits it (sol1's test above)."""
    tf2 = _tf2(tmp_path, imu=(
        f'  lk_->request("a", "b", rclcpp::Time(msg->header.stamp, {arg}), cb);'))
    _assert_refused(tf2, "unknown", "cannot resolve to rcl's RCL_ROS_TIME enumerator")


def _clock_enums(tmp_path, monkeypatch, dep_name: str, dep_text: str, body: str,
                 rcl: bool = True) -> list:
    """clock_enum() of every argument of the calls in *body*, parsed against
    rcl's headers and one dependency header (without the cast rule in the
    way: these arguments are no written casts)."""
    import clang.cindex as cindex

    from colcon_lf.node_analysis.clang_parser import _gcc_builtin_flags
    from colcon_lf.node_analysis.tf2_scan import _StampAst

    dep = _dependency(tmp_path, monkeypatch, dep_name, dep_text)
    main = tmp_path / "probe.cpp"
    # rclcpp reaches rcl/time.h inside another extern "C" block (rcl.h's):
    # the enum's parent is then LINKAGE_SPEC -> LINKAGE_SPEC -> unit.
    main.write_text(('extern "C" {\n#include "rcl/time.h"\n}\n' if rcl else "")
                    + f'#include "{dep}"\n' + body)
    ros = Path("/opt/ros/jazzy/include")
    flags = ["-std=c++17", *(f"-I{d}" for d in sorted(ros.iterdir()) if d.is_dir()),
             *_gcc_builtin_flags()]
    tu = cindex.Index.create().parse(str(main), args=flags)
    errors = [d.spelling for d in tu.diagnostics if d.severity >= cindex.Diagnostic.Error]
    assert not errors, errors
    real = os.path.realpath(str(main))
    ast = _StampAst(tu, {real}, {real: main.read_bytes()})
    out, todo = [], [tu.cursor]
    while todo:
        cur = todo.pop()
        if cur.kind.name == "CALL_EXPR" and cur.location.file is not None \
                and os.path.realpath(cur.location.file.name) == real:
            out += [(cur.location.line, ast.clock_enum(a)) for a in cur.get_arguments()]
        todo.extend(cur.get_children())
    return [c for _line, c in sorted(out)]


def test_sol_f91f_clock_enum_is_rcls_own_enum(tmp_path, monkeypatch):
    """Item a) on its own: a namespaced enum named rcl_clock_type_e in a
    dependency header is no clock type, even reached without a cast; rcl's
    own enumerator, bare or in parentheses, still is."""
    got = _clock_enums(tmp_path, monkeypatch, "clock.hpp", FAKE_ENUM, (
        "void f(fake::rcl_clock_type_e);\nvoid h(rcl_clock_type_t);\n"
        "void g() {\n  f(fake::RCL_ROS_TIME);\n  h(RCL_ROS_TIME);\n  h((RCL_ROS_TIME));\n}\n"))
    assert got == [None, "RCL_ROS_TIME", "RCL_ROS_TIME"], (
        f"clock_enum gave {got}: fake::RCL_ROS_TIME resolved to rcl's RCL_ROS_TIME, "
        "so generation accepted a clock type of another enum")


def test_sol_f91f_clock_enum_needs_rcls_header(tmp_path, monkeypatch):
    """Item a), file half: a global extern "C" enum rcl_clock_type_e declared
    in a header that is not rcl/time.h (a translation unit without rcl's) is
    no clock type."""
    got = _clock_enums(tmp_path, monkeypatch, "time.h", FAKE_GLOBAL,
                       "void h(rcl_clock_type_t);\nvoid g() {\n  h(RCL_ROS_TIME);\n}\n",
                       rcl=False)
    assert got == [None], (
        f"clock_enum gave {got}: an enum outside rcl/time.h resolved to rcl's "
        "RCL_ROS_TIME, so generation accepted a clock type of another enum")
