# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Source scan for tf2_ros use.

``tf2_ros::TransformListener`` creates its ``/tf`` and ``/tf_static``
subscriptions inside the library, and ``TransformBroadcaster::sendTransform``
publishes through a publisher the library owns, so the libclang walk in
``clang_parser.py`` (which keys on ``create_subscription``/``create_publisher``
calls in user code) sees neither. Like ``message_filters_scan.py`` this scan
works on the comment-blanked source text of the node's .cpp and header, which
stays usable when the tf2 headers do not resolve.

Admitted form (anything else lands in ``Tf2Info.refusals`` with an RCLLF-R002
subcode, see ``SUBCODES``):

* ``buf_ = std::make_shared|make_unique<tf2_ros::Buffer>(get_clock())`` (also
  ``this->get_clock()``): the Buffer runs on the node's clock, which follows LF
  logical time under rcllf. Any other clock argument is ``tf2_clock_mismatch``.
* at most one ``lst_ = std::make_shared|make_unique<tf2_ros::TransformListener>(
  *buf_, this[, true])`` per node, in the constructor. Its two subscriptions
  become synthetic ``tf_ingest_via_<member stripped of _>`` subscriptions. Another form (no
  node argument, ``spin_thread = false``, custom QoS or options, a second
  listener, a listener made outside the constructor) is
  ``tf2_listener_unadmitted``.
* ``br_ = std::make_shared|make_unique<tf2_ros::TransformBroadcaster>(this)``
  and ``StaticTransformBroadcaster``: a publisher on ``/tf`` or ``/tf_static``;
  ``br_->sendTransform(...)`` counts as a publish of that topic.
* every ``<adapter>->request(target, source, stamp, cb)`` call records where
  ``stamp`` comes from (``Tf2Info.stamps``, see ``ast_stamps``: classified on
  the compiler's AST, not on this text); the resolved model turns it into
  ``tf2.sample_stamp.<member>`` (criterion 5).
* lookups only through ``rcllf_tf2::AcquisitionLookup``. A direct
  ``lookupTransform``/``canTransform`` with a non-zero timeout, or any
  ``waitForTransform``, is ``tf2_blocking_lookup``; every other direct call
  (``TimePointZero``, zero timeout) is ``tf2_direct_lookup``.
"""

from __future__ import annotations

import os
import re
from collections import Counter

from colcon_lf.node_analysis.message_filters_scan import (
    _balanced,
    _call_args_after,
    _split_args,
)
from colcon_lf.node_analysis.models import (
    Tf2AdapterInfo,
    Tf2BroadcasterInfo,
    Tf2BufferInfo,
    Tf2Info,
    Tf2ListenerInfo,
    Tf2LookupCallInfo,
    Tf2Refusal,
    Tf2StampInfo,
)

# The tf2_ros version the acquisition-time contract was checked against
# (S0 probe, container `ros-jazzy-tf2-ros`). Provenance: pinned dependency;
# lf_eval reports `version_unpinned` when a run's tf2_ros differs.
TF2_ROS_PIN = "0.36.23"

TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
TF_MESSAGE_TYPE = "tf2_msgs::msg::TFMessage"

# The QoS tf2_ros 0.36.23 gives its endpoints (tf2_ros/qos.hpp): the
# listener's DynamicListenerQoS and StaticListenerQoS are depth 100, the
# static one transient_local; the broadcasters' DynamicBroadcasterQoS is depth
# 100 and StaticBroadcasterQoS depth 1 transient_local. All reliable keep_last.
# Provenance: library default, read from the pinned header.
LISTENER_QOS = {"qos_depth": 100, "qos_reliability": "reliable",
                "qos_durability": "volatile", "qos_history": "keep_last"}
STATIC_LISTENER_QOS = dict(LISTENER_QOS, qos_durability="transient_local")
BROADCASTER_QOS = dict(LISTENER_QOS)
STATIC_BROADCASTER_QOS = dict(LISTENER_QOS, qos_depth=1, qos_durability="transient_local")

SUBCODES = (
    "tf2_listener_unadmitted",
    "tf2_clock_mismatch",
    "tf2_blocking_lookup",
    "tf2_direct_lookup",
    "tf2_stamp_mismatch",
    "tf2_stamp_unknown",
)

_LOOKUP_METHODS = ("lookupTransform", "canTransform", "waitForTransform")
_MAKE = r"std\s*::\s*make_(?:shared|unique)\s*"
_NODE_CLOCK = re.compile(r"^(?:this\s*->\s*)?get_clock\s*\(\s*\)$")
_DURATION_HINT = re.compile(
    r"chrono|Duration|duration|timeout|seconds|milliseconds|microseconds|nanoseconds"
    r"|\b\d+(?:\.\d+)?(?:ms|s|us|ns|min|h)\b")
_ZERO_DURATION = re.compile(
    r"^(?:std\s*::\s*)?(?:chrono\s*::\s*)?(?:\w+\s*\(\s*0+(?:\.0*)?\s*\)|0+(?:\.0*)?(?:ms|s|us|ns)?)$"
    r"|::zero\s*\(\s*\)$|^(?:tf2\s*::\s*)?durationFromSec\s*\(\s*0+(?:\.0*)?\s*\)$")
_CHRONO_LITERAL = re.compile(r"^(\d+)(ns|us|ms|s)$")
_CHRONO_CALL = re.compile(
    r"^(?:std\s*::\s*)?(?:chrono\s*::\s*)?(nanoseconds|microseconds|milliseconds|seconds)"
    r"\s*\(\s*(\d+)\s*\)$")
_UNIT_NS = {"ns": 1, "us": 1_000, "ms": 1_000_000, "s": 1_000_000_000,
            "nanoseconds": 1, "microseconds": 1_000, "milliseconds": 1_000_000,
            "seconds": 1_000_000_000}
# Mentions of the two types the scan must account for (F91-1).
_TYPE_MENTION = re.compile(r"\btf2_ros\s*::\s*(Buffer|TransformListener)\b")
# A declaration ends, or is initialized in class by the admitted make_* form
# (that make_* is a mention of its own, admitted or refused on its own merits).
_DECL_END = (r"\s*(?:;|=\s*nullptr\s*;|\{\s*(?:nullptr\s*)?\}\s*;"
             r"|(?==\s*std\s*::\s*make_(?:shared|unique)\s*<))")
# `tf2_ros::Buffer::SharedPtr buf_;`
_SMART_DECL_PTR = re.compile(
    r"tf2_ros\s*::\s*(?:Buffer|TransformListener)\s*::\s*(?:SharedPtr|UniquePtr)\s+"
    r"(?P<member>[A-Za-z_]\w*)" + _DECL_END)
# `std::shared_ptr<tf2_ros::Buffer> buf_;` (found around the type token)
_SMART_DECL_WRAPPED = re.compile(
    r"(?:std\s*::\s*)?(?:shared_ptr|unique_ptr)\s*<\s*(?P<type>tf2_ros\s*::\s*"
    r"(?:Buffer|TransformListener))\s*>\s*(?P<member>[A-Za-z_]\w*)" + _DECL_END)
_NS_ALIAS = re.compile(r"\busing\s+namespace\s+(?:::\s*)?tf2_ros\b"
                       r"|\bnamespace\s+[A-Za-z_]\w*\s*=\s*(?:::\s*)?tf2_ros\s*;")
_ON_TIMEOUT = re.compile(
    r"^(?:rcllf_tf2\s*::\s*)?OnTimeout\s*::\s*(reject|failure)\s*\(\s*\)$"
    r"|^(?:rcllf_tf2\s*::\s*)?OnTimeout\s*::\s*fallback\s*\(\s*\"([A-Za-z0-9_]+)\"\s*\)$")


def blank_comments(text: str) -> str:
    """Replace // and /* */ comments by spaces, keeping newlines and strings."""
    out: list[str] = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c in "\"'":
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(ch if ch == "\n" else " " for ch in text[i:j]))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def duration_ns(expr: str) -> int | None:
    """Nanoseconds of an integer chrono literal (``55ms``) or call (``milliseconds(55)``)."""
    e = re.sub(r"\s+", "", expr)
    m = _CHRONO_LITERAL.match(e)
    if m:
        return int(m.group(1)) * _UNIT_NS[m.group(2)]
    m = _CHRONO_CALL.match(e)
    if m:
        return int(m.group(2)) * _UNIT_NS[m.group(1)]
    return None


def _is_timeout_arg(arg: str) -> bool:
    """A duration-looking argument that is not a time point (stamps carry "Time")."""
    if not _DURATION_HINT.search(arg):
        return False
    return "Time" not in re.sub(r"Duration|durationFromSec", "", arg)


def _is_zero(arg: str) -> bool:
    return bool(_ZERO_DURATION.search(re.sub(r"\s+", "", arg)))


def _member_before(text: str, pos: int) -> str | None:
    """The identifier assigned at ``<member> = `` ending right before *pos*."""
    m = re.search(r"([A-Za-z_]\w*)\s*=\s*$", text[:pos])
    return m.group(1) if m else None


def _deref(arg: str) -> str | None:
    m = re.match(r"^\*\s*(?:this\s*->\s*)?([A-Za-z_]\w*)$", arg.strip())
    return m.group(1) if m else None


def _constructor_span(text: str, class_name: str) -> list[tuple[int, int]]:
    """Body spans of the class's constructors (out-of-line or in-class)."""
    spans: list[tuple[int, int]] = []
    if not class_name:
        return spans
    for m in re.finditer(rf"(?<![~\w]){re.escape(class_name)}\s*\(", text):
        got = _balanced(text, m.end() - 1, "(", ")")
        if got is None:
            continue
        j = got[1]
        # skip an init-list up to the body's brace; a ';' first means a declaration
        k = j
        while k < len(text) and text[k] not in "{;":
            if text[k] == "(":
                inner = _balanced(text, k, "(", ")")
                k = inner[1] if inner else k + 1
                continue
            k += 1
        if k >= len(text) or text[k] != "{":
            continue
        body = _balanced(text, k, "{", "}")
        if body is not None:
            spans.append((k, body[1]))
    return spans


def scan(sources: list[tuple[str, str]], class_name: str,
         accounted: set[tuple[str, int, int, str]] | None = None) -> Tf2Info | None:
    """Scan ``(path, text)`` sources of one node class. None without tf2_ros use.

    *accounted*, when given, receives ``(path, start, end, kind)`` character
    spans of the text the scan read as a tf2_ros::Buffer or
    tf2_ros::TransformListener (``kind``; ``"any"`` for a ``using namespace``):
    the type inside an admitted or refused make_*, a member declaration, any
    other mention of the two types. The parser compares them with the
    compiler's own references to the two types (F91b): a reference outside
    every span of its type is spelled in a form the scan cannot see (a macro,
    say), even when it shares a line with one the scan read.
    """
    info = Tf2Info()
    acc: set[tuple[str, int, int, str]] = accounted if accounted is not None else set()
    seen_any = False
    listeners_outside: list[Tf2ListenerInfo] = []
    mentions: list[tuple[str, str | None, str, int]] = []
    for path, raw in sources:
        text = blank_comments(raw)
        if "tf2_ros" not in text and "rcllf_tf2" not in text:
            continue
        seen_any = True
        fname = os.path.basename(path)
        ctor = _constructor_span(text, class_name)

        def line_of(pos: int, _text: str = text) -> int:
            return _text.count("\n", 0, pos) + 1

        def account(start: int, end: int, kind: str, _path: str = path) -> None:
            acc.add((_path, start, max(start, end), kind))

        admitted_at: set[int] = set()
        for m in re.finditer(_MAKE + r"<\s*tf2_ros\s*::\s*(Buffer|TransformListener|"
                             r"TransformBroadcaster|StaticTransformBroadcaster)\s*>", text):
            kind = m.group(1)
            admitted_at.add(text.index("tf2_ros", m.start()))
            member = _member_before(text, m.start()) or ""
            call = _call_args_after(text, m.end())
            args = call[0] if call else []
            line = line_of(m.start())
            account(m.start(), m.end(), kind)
            if kind == "Buffer":
                clock = args[0].strip() if args else ""
                info.buffers.append(Tf2BufferInfo(
                    member=member,
                    clock="node" if _NODE_CLOCK.match(re.sub(r"\s+", "", clock)) else clock,
                    file=fname, line=line))
            elif kind == "TransformListener":
                lst = Tf2ListenerInfo(member=member, buffer_member=_deref(args[0]) if args else None,
                                      file=fname, line=line)
                info.listeners.append(lst)
                why = None
                if len(args) < 2 or re.sub(r"\s+", "", args[1]) not in ("this", "this->shared_from_this()"):
                    why = ("it is not given the node itself (`this`), so it creates a node of its "
                           "own that rcllf does not convert")
                elif len(args) >= 3 and re.sub(r"\s+", "", args[2]) != "true":
                    why = f"spin_thread is {args[2].strip()!r}, not the default true"
                elif len(args) > 3:
                    why = "it is given its own QoS or subscription options"
                elif not member:
                    why = "it is not stored in a member"
                elif not any(a <= m.start() < b for a, b in ctor):
                    listeners_outside.append(lst)
                    why = "it is created outside the constructor"
                if why is not None:
                    info.refusals.append(Tf2Refusal(
                        "tf2_listener_unadmitted",
                        f"TransformListener {member or '<unnamed>'} at {fname}:{line}: {why}"))
            else:
                static = kind == "StaticTransformBroadcaster"
                info.broadcasters.append(Tf2BroadcasterInfo(
                    member=member, static=static, file=fname, line=line))

        # Every other mention of the two types (F91-1, fail closed): a member
        # declaration is admitted once its member is made in the admitted form
        # (checked after all sources), anything else is a construction or an
        # alias the scan does not follow.
        for m in _TYPE_MENTION.finditer(text):
            if m.start() in admitted_at:
                continue
            d = _SMART_DECL_PTR.match(text, m.start())
            if d is None:
                d = next((x for x in _SMART_DECL_WRAPPED.finditer(
                    text, max(0, m.start() - 40), m.end() + 80) if x.start("type") == m.start()),
                    None)
            mentions.append((m.group(1), d.group("member") if d else None, fname,
                             line_of(m.start())))
            account(min(m.start(), d.start()) if d else m.start(),
                    max(m.end(), d.end()) if d else m.end(), m.group(1))
        for m in _NS_ALIAS.finditer(text):
            mentions.append(("alias", None, fname, line_of(m.start())))
            account(m.start(), m.end(), "any")

        for m in re.finditer(_MAKE + r"<\s*rcllf_tf2\s*::\s*AcquisitionLookup\s*>", text):
            call = _call_args_after(text, m.end())
            args = call[0] if call else []
            on_timeout = None
            if len(args) >= 4:
                o = _ON_TIMEOUT.match(re.sub(r"\s+", "", args[3]).replace("rcllf_tf2::", ""))
                if o is not None:
                    on_timeout = o.group(1) or f"fallback:{o.group(2)}"
            info.adapters.append(Tf2AdapterInfo(
                member=_member_before(text, m.start()) or "",
                buffer_member=_deref(args[0]) if args else None,
                wait_bound_ns=duration_ns(args[2]) if len(args) >= 3 else None,
                on_timeout=on_timeout, file=fname, line=line_of(m.start())))

        for m in re.finditer(r"([A-Za-z_]\w*)\s*(?:->|\.)\s*(" + "|".join(_LOOKUP_METHODS)
                             + r")\s*\(", text):
            got = _balanced(text, m.end() - 1, "(", ")")
            args = _split_args(got[0]) if got else []
            method = m.group(2)
            timeout = method == "waitForTransform" or any(
                _is_timeout_arg(a) and not _is_zero(a) for a in args[2:])
            info.lookups.append(Tf2LookupCallInfo(
                receiver=m.group(1), method=method, timeout=timeout,
                file=fname, line=line_of(m.start())))

    if not seen_any:
        return None

    if len(info.listeners) > 1:
        info.refusals.append(Tf2Refusal(
            "tf2_listener_unadmitted",
            f"{len(info.listeners)} TransformListeners ("
            + ", ".join(f"{x.member} at {x.file}:{x.line}" for x in info.listeners)
            + "); one per node is admitted, a second would ingest /tf twice"))
    # Members are matched by name, not by class: two declarations of `buf_`
    # (two classes in the scanned files) need two admitted constructions, so
    # one class's make_* never vouches for another's member.
    made: dict[str, Counter] = {
        "Buffer": Counter(b.member for b in info.buffers if b.member),
        "TransformListener": Counter(x.member for x in info.listeners if x.member)}
    declared = Counter((k, mem) for k, mem, _f, _l in mentions if mem is not None)
    for kind, member, fname, line in mentions:
        if kind == "alias":
            why = ("a using-directive or namespace alias of tf2_ros hides which "
                   "Buffer/TransformListener the node constructs and how")
        elif member is not None and declared[(kind, member)] > made[kind][member] > 0:
            why = (f"member '{member}' is declared {declared[(kind, member)]} times but made "
                   f"{made[kind][member]} time(s) with std::make_shared/make_unique<tf2_ros::"
                   f"{kind}>, so the scan cannot tell which declaration a construction belongs to")
        elif member is not None and member in made[kind]:
            continue
        elif member is not None:
            why = (f"member '{member}' is never made with std::make_shared/make_unique<tf2_ros::"
                   f"{kind}>, so the scan cannot see how it is constructed")
        else:
            why = (f"tf2_ros::{kind} is used in a form other than std::make_shared/make_unique "
                   "or a smart-pointer member declaration (e.g. new, a value member, a helper "
                   "that returns one); its /tf ingress would bypass LF")
        info.refusals.append(Tf2Refusal(
            "tf2_listener_unadmitted", f"{kind if kind != 'alias' else 'tf2_ros'} at "
            f"{fname}:{line}: {why}"))
    for b in info.buffers:
        if b.clock != "node":
            info.refusals.append(Tf2Refusal(
                "tf2_clock_mismatch",
                f"Buffer {b.member} at {b.file}:{b.line} runs on clock {b.clock or '<none>'!r}, "
                "not the node's own get_clock(); under rcllf only the node clock follows LF "
                "logical time, so the Buffer's cache time would not be the tag"))
    for c in info.lookups:
        sub = "tf2_blocking_lookup" if c.timeout else "tf2_direct_lookup"
        why = ("waits on its timeout for a /tf message that can only arrive in a later "
               "reaction, so it blocks the LF worker" if c.timeout else
               "answers with what the Buffer holds at that instant (the latest transform for "
               "TimePointZero) and bypasses the bounded acquisition-time request")
        info.refusals.append(Tf2Refusal(
            sub, f"{c.receiver}->{c.method} at {c.file}:{c.line} {why}"))
    for a in info.adapters:
        if a.wait_bound_ns is None or a.wait_bound_ns <= 0 or a.on_timeout is None:
            info.refusals.append(Tf2Refusal(
                "tf2_listener_unadmitted",
                f"AcquisitionLookup {a.member or '<unnamed>'} at {a.file}:{a.line}: the wait bound "
                "must be a positive integer std::chrono literal and on_timeout one of "
                "OnTimeout::reject(), OnTimeout::failure(), OnTimeout::fallback(\"<name>\")"))
    listener_buffers = {lst.buffer_member for lst in info.listeners}
    for a in info.adapters:
        if info.listeners and a.buffer_member not in listener_buffers:
            # The binding feeds only the listener's Buffer from LF; any other
            # Buffer would never see /tf (or see it outside LF).
            info.refusals.append(Tf2Refusal(
                "tf2_listener_unadmitted",
                f"AcquisitionLookup {a.member or '<unnamed>'} at {a.file}:{a.line} reads Buffer "
                f"'{a.buffer_member}', but the node's TransformListener feeds "
                f"{', '.join(repr(b) for b in sorted(str(b) for b in listener_buffers))}"))
    if info.adapters and not info.listeners:
        info.refusals.append(Tf2Refusal(
            "tf2_listener_unadmitted",
            "an AcquisitionLookup needs the node's own TransformListener feeding its Buffer "
            "(none found)"))
    return info


# --- sample stamp provenance (acceptance criterion 5) -------------
#
# Classified on the compiler's AST of the node's translation unit:
# the regex classifier this replaces read names, so
# an alias of the adapter, a local shadowing a member or a message parameter,
# and a local lambda called `now` all passed as admitted forms. Here every
# name is the declaration the compiler resolved it to.

_STAMP_DEPTH = 8
_WRAPPERS = ("UNEXPOSED_EXPR", "PAREN_EXPR")
_CASTS = ("CXX_FUNCTIONAL_CAST_EXPR", "CXX_STATIC_CAST_EXPR", "CSTYLE_CAST_EXPR")
_NAME_REFS = ("TYPE_REF", "NAMESPACE_REF", "TEMPLATE_REF")
_ALL_CASTS = _CASTS + ("CXX_CONST_CAST_EXPR", "CXX_REINTERPRET_CAST_EXPR",
                       "CXX_DYNAMIC_CAST_EXPR")
_FUNCTIONS = ("CXX_METHOD", "FUNCTION_DECL", "FUNCTION_TEMPLATE", "CONSTRUCTOR",
              "DESTRUCTOR", "CONVERSION_FUNCTION", "LAMBDA_EXPR")
_STD_CLOCKS = ("system_clock", "steady_clock", "high_resolution_clock")
# Declarations admitted by their resolved, qualified name (never by spelling):
# rclcpp's own time arithmetic, the subscription calls of this node, the
# adapter class, and the enum of rcl's clock types.
_TIME_ARITH = ("rclcpp::Time::operator+", "rclcpp::Time::operator-", "rclcpp::operator+")
_SUBSCRIBE = ("rclcpp::Node::create_subscription",
              "rclcpp_lifecycle::LifecycleNode::create_subscription")
_ADAPTER = "rcllf_tf2::AcquisitionLookup"
_CLOCK_ENUMS = ("rcl_clock_type_e", "rcl_clock_type_t")
_CLOCK_HEADER = ("rcl", "time.h")  # where rcl declares them (rcl/rcl/time.h in Jazzy)
_SMART_PTRS = ("std::shared_ptr", "std::unique_ptr")
_THIS = "this"  # call_object's answer for `this`; compared by identity (Cursor.__eq__ needs a Cursor)


def _ckind(cur) -> str:
    try:
        return cur.kind.name
    except ValueError:  # a kind this libclang's python binding does not name
        return "UNKNOWN"


def _qualified(cur) -> str:
    """``ns::Class::name`` of a declaration cursor (its semantic parents)."""
    parts = []
    while cur is not None and _ckind(cur) != "TRANSLATION_UNIT":
        if cur.spelling:
            parts.append(cur.spelling)
        cur = cur.semantic_parent
    return "::".join(reversed(parts))


def _canon(cur) -> str:
    try:
        return cur.type.get_canonical().spelling
    except Exception:
        return ""


def _const_view(t) -> bool:
    """Whether a value of type *t* made from a variable cannot write it: a
    copy, or a reference or pointer to const."""
    if t.kind.name in ("LVALUEREFERENCE", "RVALUEREFERENCE", "POINTER"):
        return t.get_pointee().is_const_qualified()
    return True


def _pointee(t):
    """``(pointee, True)`` for a raw or smart pointer type *t* (through a
    reference; pointee ``None`` when unreadable), ``(t, False)`` otherwise."""
    t = t.get_canonical()
    if t.kind.name in ("LVALUEREFERENCE", "RVALUEREFERENCE"):
        t = t.get_pointee().get_canonical()
    if t.kind.name == "POINTER":
        return t.get_pointee(), True
    if t.kind.name == "RECORD" and _qualified(t.get_declaration()) in _SMART_PTRS:
        try:
            if t.get_num_template_arguments() > 0:
                return t.get_template_argument_type(0), True
        except Exception:
            pass
        return None, True
    return t, False


def _is_adapter_type(t) -> bool:
    """Whether *t* is (a pointer to) the class ``rcllf_tf2::AcquisitionLookup``
    clang resolved: its declaration, not a substring of the type's spelling."""
    pointee, _ptr = _pointee(t)
    if pointee is None:
        return False
    decl = pointee.get_canonical().get_declaration()
    return decl is not None and _ckind(decl) != "NO_DECL_FOUND" and _qualified(decl) == _ADAPTER


def _peel(cur):
    """*cur* without implicit casts, temporaries, parentheses and a written
    cast (``rclcpp::Time(x)``, ``static_cast<rclcpp::Time>(x)``: the
    conversion itself is the constructor call below it)."""
    while True:
        kind = _ckind(cur)
        kids = [k for k in cur.get_children() if _ckind(k) not in _NAME_REFS]
        if kind in _WRAPPERS + _CASTS and len(kids) == 1:
            cur = kids[0]
        else:
            return cur


class _StampAst:
    """One translation unit's request calls and the writes of every
    variable and member, keyed by the declarations clang resolved."""

    def __init__(self, tu, user_files: set[str], texts: dict[str, bytes]):
        self.texts = texts
        self.parent: dict = {}
        self.refs: dict = {}            # declaration -> its DECL_REF/MEMBER_REF uses
        self.ctor_inits: dict = {}      # field -> [initializer expressions]
        self.requests: list = []        # CALL_EXPRs named request
        self.adapter_fields: dict = {}  # field name -> FIELD_DECL of an adapter type
        stack = []
        for top in tu.cursor.get_children():
            loc = top.location
            if loc.file is None or os.path.realpath(loc.file.name) not in user_files:
                continue
            stack.append((top, None))
        while stack:
            cur, parent = stack.pop()
            if parent is not None:
                self.parent[cur] = parent
            kind = _ckind(cur)
            if kind in ("DECL_REF_EXPR", "MEMBER_REF_EXPR"):
                ref = cur.referenced
                if ref is not None:
                    self.refs.setdefault(ref, []).append(cur)
            elif kind == "CALL_EXPR" and cur.spelling == "request":
                self.requests.append(cur)
            elif kind == "FIELD_DECL" and _is_adapter_type(cur.type):
                self.adapter_fields[cur.spelling] = cur
            elif kind == "CONSTRUCTOR":
                kids = list(cur.get_children())
                for i, k in enumerate(kids[:-1]):
                    if _ckind(k) == "MEMBER_REF" and k.referenced is not None:
                        self.ctor_inits.setdefault(k.referenced, []).append(kids[i + 1])
            for child in cur.get_children():
                stack.append((child, cur))

    # -- helpers -----------------------------------------------------------

    def text(self, cur) -> str:
        ext = cur.extent
        if ext.start.file is None:
            return cur.spelling
        data = self.texts.get(os.path.realpath(ext.start.file.name))
        if data is None:
            return cur.spelling
        return " ".join(data[ext.start.offset:ext.end.offset].decode("utf-8", "replace").split())

    def outside(self, decl) -> bool:
        """Whether *decl* is declared outside the node's own files (a system
        or dependency header, which the node cannot redefine)."""
        f = decl.location.file
        return f is not None and os.path.realpath(f.name) not in self.texts

    def clock_enum(self, arg) -> str | None:
        """The enumerator of rcl's ``rcl_clock_type_t`` that *arg* names, as
        clang resolved it (``RCL_ROS_TIME``, ...); ``None`` for any other
        expression, a same-named variable or constant included.

        Fail closed (F91f): *arg* reaches the enumerator through implicit
        casts and parentheses only, so a written cast (``static_cast``,
        C-style, functional), the one way another enum's constant or any
        integer reaches the parameter, is ``None`` whatever it converts. The
        enum must be rcl's own: declared at global scope (directly or in
        nested ``extern "C"`` blocks) in ``rcl/time.h``
        (``/opt/ros/jazzy/include/rcl/rcl/time.h`` in Jazzy), not merely
        named ``rcl_clock_type_e`` in a namespace of a dependency header."""
        a = arg
        while _ckind(a) in _WRAPPERS:
            kids = [k for k in a.get_children() if _ckind(k) not in _NAME_REFS]
            if len(kids) != 1:
                return None
            a = kids[0]
        d = a.referenced if _ckind(a) == "DECL_REF_EXPR" else None
        if d is None or _ckind(d) != "ENUM_CONSTANT_DECL" or not self.outside(d):
            return None
        e = d.semantic_parent
        if e is None or _ckind(e) != "ENUM_DECL" or e.spelling not in _CLOCK_ENUMS:
            return None
        scope = e.semantic_parent
        while scope is not None and _ckind(scope) == "LINKAGE_SPEC":
            # extern "C" blocks nest (rcl.h includes time.h inside its own)
            # and open no scope of their own.
            scope = scope.semantic_parent
        if scope is None or _ckind(scope) != "TRANSLATION_UNIT":
            return None
        f = e.location.file
        if f is None or tuple(os.path.realpath(f.name).split(os.sep)[-2:]) != _CLOCK_HEADER:
            return None
        return d.spelling

    def up(self, cur):
        """The first ancestor of *cur* that is no wrapper, and the child of it
        on the path from *cur*."""
        child, p = cur, self.parent.get(cur)
        while p is not None and _ckind(p) in _WRAPPERS:
            child, p = p, self.parent.get(p)
        return p, child

    @staticmethod
    def call_args(call) -> list:
        try:
            return list(call.get_arguments())
        except Exception:
            return []

    @staticmethod
    def call_object(call):
        """The object expression of a member call ``obj.f(...)`` /
        ``obj->f(...)``, ``"this"`` for an implicit or explicit ``this``,
        ``None`` when *call* has no member callee."""
        kids = list(call.get_children())
        if not kids:
            return None
        callee = _peel(kids[0])
        if _ckind(callee) != "MEMBER_REF_EXPR":
            return None
        # Template arguments (``create_subscription<sensor_msgs::msg::Imu>``)
        # are name references among the callee's children, never its object.
        base = [k for k in callee.get_children() if _ckind(k) not in _NAME_REFS]
        if not base:
            return _THIS
        obj = _peel(base[0])
        return _THIS if _ckind(obj) == "CXX_THIS_EXPR" else obj

    def strip_deref(self, obj):
        """*obj* without a smart pointer's ``operator->``/``operator*``."""
        obj = _peel(obj)
        while (_ckind(obj) == "CALL_EXPR" and obj.spelling in ("operator->", "operator*")
               and self.call_args(obj)):
            obj = _peel(self.call_args(obj)[0])
        if _ckind(obj) == "UNARY_OPERATOR":   # *raw_ptr
            kids = list(obj.get_children())
            if len(kids) == 1:
                obj = _peel(kids[0])
        return obj

    @staticmethod
    def is_this_member(ref) -> bool:
        """A MEMBER_REF_EXPR on ``this`` (written or implicit)."""
        base = list(ref.get_children())
        return not base or _ckind(_peel(base[0])) == "CXX_THIS_EXPR"

    @staticmethod
    def literal(arg) -> str | None:
        """The string literal *arg* is (through its std::string conversion),
        ``None`` for any other expression."""
        lits = [c for c in arg.walk_preorder() if _ckind(c) == "STRING_LITERAL"]
        names = [c for c in arg.walk_preorder() if _ckind(c) in ("DECL_REF_EXPR",
                                                                 "MEMBER_REF_EXPR")]
        if len(lits) != 1 or names:
            return None
        return lits[0].spelling[1:-1] if lits[0].spelling.startswith('"') else None

    # -- the adapter of a request call ------------------------------------

    def adapter_of(self, call):
        """``(member, None)`` for ``member->request(...)`` on an adapter member
        of ``this``, else ``(None, why)``."""
        ref = call.referenced
        if ref is None or _qualified(ref) != "rcllf_tf2::AcquisitionLookup::request":
            return None, None
        obj = self.call_object(call)
        if obj is None or obj is _THIS:
            return None, "the call has no adapter object the scan can name"
        obj = self.strip_deref(obj)
        if (_ckind(obj) == "MEMBER_REF_EXPR" and obj.referenced is not None
                and _ckind(obj.referenced) == "FIELD_DECL" and self.is_this_member(obj)):
            return obj.referenced.spelling, None
        return None, (f"the request goes through '{self.text(obj)}', not through an "
                      "AcquisitionLookup member of the node (an alias or copy of the "
                      "adapter hides which adapter and stamp it is)")

    # -- stamp classification ----------------------------------------------

    def classify(self, expr, depth: int = 0, seen: frozenset = frozenset()):
        """``(kind, detail, callback, topic)`` of a stamp expression."""
        e = _peel(expr)
        kind = _ckind(e)
        shown = self.text(expr)
        if depth > _STAMP_DEPTH:
            return "unknown", f"'{shown}' is nested deeper than the scan follows", None, None
        if kind == "CALL_EXPR":
            return self.classify_call(e, shown, depth, seen)
        if kind == "MEMBER_REF_EXPR":
            got = self.message_stamp(e)
            if got is not None:
                return got
            ref = e.referenced
            if ref is not None and _ckind(ref) == "FIELD_DECL" and self.is_this_member(e):
                return self.follow(ref, depth, seen)
            return "unknown", f"the scan cannot follow '{shown}'", None, None
        if kind == "DECL_REF_EXPR":
            ref = e.referenced
            rk = _ckind(ref) if ref is not None else ""
            if rk == "VAR_DECL":
                return self.follow(ref, depth, seen)
            if rk == "PARM_DECL":
                return ("unknown", f"'{shown}' is a parameter of the function around it; its "
                        "caller sets the stamp, which the scan does not follow", None, None)
        return "unknown", f"the scan cannot follow '{shown}'", None, None

    def classify_call(self, e, shown, depth, seen):
        ref = e.referenced
        args = self.call_args(e)
        if ref is None:
            return "unknown", f"the compiler does not resolve '{shown}'", None, None
        q = _qualified(ref)
        rk = _ckind(ref)
        if rk == "CONSTRUCTOR" and q.startswith("rclcpp::Time::"):
            params = [_canon(x) for x in ref.get_arguments()]
            first = params[0] if params else ""
            if args and ("rclcpp::Time" in first or "builtin_interfaces::msg::Time" in first):
                # A clock type argument written in the node's files must
                # resolve to rcl's RCL_ROS_TIME enumerator (a same-named local
                # constant does not); an omitted one is the header's default
                # (a default argument lies outside the node's files).
                rest = [a for a in args[1:] if a.extent.start.file is not None
                        and os.path.realpath(a.extent.start.file.name) in self.texts]
                clocks = [self.clock_enum(a) for a in rest]
                if all(c == "RCL_ROS_TIME" for c in clocks):
                    return self.classify(args[0], depth + 1, seen)
                if any(c is None for c in clocks):
                    return ("unknown", f"'{shown}' gives a clock type the scan cannot resolve to "
                            "rcl's RCL_ROS_TIME enumerator", None, None)
                return ("foreign_clock", f"'{shown}' puts the stamp on a clock type that is "
                        "not the node's ROS time", None, None)
            if any(self.clock_enum(a) in ("RCL_SYSTEM_TIME", "RCL_STEADY_TIME")
                   for a in args):
                return ("foreign_clock", f"'{shown}' reads a clock that is not the node's",
                        None, None)
            return "unknown", f"'{shown}' is a constant, not an acquisition time", None, None
        if e.spelling in ("operator+", "operator-"):
            if q not in _TIME_ARITH or not self.outside(ref):
                return ("unknown", f"'{shown}' calls {q or e.spelling}, not rclcpp's own time "
                        "arithmetic", None, None)
            times = [a for a in args if "rclcpp::Time" in _canon(a)]
            spans = [a for a in args if "rclcpp::Duration" in _canon(a)
                     or "std::chrono::duration" in _canon(a)]
            if len(args) == 2 and len(times) == 1 and len(spans) == 1 and (
                    e.spelling == "operator+" or times[0] == args[0]):
                return self.classify(times[0], depth + 1, seen)
            return ("unknown", f"'{shown}' is not one time plus or minus durations",
                    None, None)
        if e.spelling == "now" and rk == "CXX_METHOD":
            obj = self.call_object(e)
            if q in ("rclcpp::Node::now", "rclcpp_lifecycle::LifecycleNode::now"):
                if obj is _THIS:
                    return "node_clock", "the node's own clock (now())", None, None
                return ("unknown", f"'{shown}' reads the clock of another node object",
                        None, None)
            if q == "rclcpp::Clock::now":
                src = self.strip_deref(obj) if obj is not None and obj is not _THIS else None
                if (src is not None and _ckind(src) == "CALL_EXPR"
                        and src.spelling == "get_clock" and src.referenced is not None
                        and _qualified(src.referenced) in (
                            "rclcpp::Node::get_clock", "rclcpp_lifecycle::LifecycleNode::get_clock")
                        and self.call_object(src) is _THIS):
                    return "node_clock", "the node's own clock (get_clock()->now())", None, None
                return ("foreign_clock", f"'{shown}' reads a clock that is not the node's",
                        None, None)
        if e.spelling == "now" and any(f"std::chrono::{c}::now" in q
                                       or f"std::chrono::_V2::{c}::now" in q
                                       for c in _STD_CLOCKS):
            return "foreign_clock", f"'{shown}' reads a clock that is not the node's", None, None
        return ("unknown", f"'{shown}' calls {q or e.spelling}, which is no admitted stamp "
                "source", None, None)

    def message_stamp(self, e):
        """``<param>->header.stamp`` of a subscription callback's message
        parameter; ``None`` when *e* is no ``header.stamp`` at all."""
        ref = e.referenced
        if (ref is None or e.spelling != "stamp"
                or not _qualified(ref).startswith("std_msgs::msg::Header_")):
            return None
        shown = self.text(e)
        base = list(e.get_children())
        header = _peel(base[0]) if base else None
        if header is None or _ckind(header) != "MEMBER_REF_EXPR" or header.spelling != "header":
            return "unknown", f"the scan cannot follow '{shown}'", None, None
        hb = list(header.get_children())
        holder = self.strip_deref(hb[0]) if hb else None
        if holder is None or _ckind(holder) != "DECL_REF_EXPR" or holder.referenced is None:
            return ("unknown", f"'{shown}' is the header of no named message", None, None)
        var = holder.referenced
        if _ckind(var) != "PARM_DECL":
            return ("unknown", f"'{shown}' reads the header of '{var.spelling}', a "
                    "variable, not the message parameter of a subscription callback "
                    "(it may hold any message)", None, None)
        owner = self.parent.get(var)
        detail = f"header.stamp of '{var.spelling}'"
        written = self.message_written(var)
        if written is not None:
            why, line = written
            return ("unknown", f"{detail}, but '{var.spelling}' {why} at line {line}, so it may "
                    "no longer hold the subscription's message", None, None)
        if owner is not None and _ckind(owner) == "CXX_METHOD":
            why = self.unsubscribed(owner, var)
            if why is not None:
                return ("unknown", f"{detail}, but the method {owner.spelling}() {why}, so "
                        f"'{var.spelling}' may be any message", None, None)
            return "message", detail, owner.spelling, None
        if owner is not None and _ckind(owner) == "LAMBDA_EXPR":
            call, _child = self.up(owner)
            if self.is_subscribe(call):
                args = self.call_args(call)
                if args and any(_peel(a) == owner for a in args):
                    topic = self.literal(args[0])
                    if topic is not None:
                        return "message", detail, None, topic
            return ("unknown", f"{detail}, but no subscription callback or create_subscription "
                    f"lambda around the call takes '{var.spelling}'", None, None)
        return ("unknown", f"{detail}, but '{var.spelling}' is a parameter of no member "
                "function or subscription lambda", None, None)

    def is_subscribe(self, call) -> bool:
        """Whether *call* is rclcpp's ``create_subscription`` on ``this``."""
        return (call is not None and _ckind(call) == "CALL_EXPR" and call.referenced is not None
                and _qualified(call.referenced) in _SUBSCRIBE
                and self.outside(call.referenced) and self.call_object(call) is _THIS)

    def unsubscribed(self, method, var) -> str | None:
        """``None`` when *var* is the first parameter of member function
        *method* and every use of *method* in the node's files is
        ``std::bind(&C::method, this, std::placeholders::_1)`` passed straight
        to ``create_subscription`` on ``this``; else why not (a direct call or
        another binding can hand it any message)."""
        params = list(method.get_arguments())
        if not params or params[0] != var:
            return "does not take the message as its first parameter"
        usr = method.get_usr()
        uses = [u for d, us in self.refs.items() if _ckind(d) == "CXX_METHOD"
                and d.get_usr() == usr for u in us]
        if not uses:
            return "is never passed to create_subscription in the node's files"
        for u in uses:
            where = f"is used at line {u.location.line}"
            amp, _c = self.up(u)
            bind, _c = self.up(amp) if amp is not None else (None, None)
            if (amp is None or _ckind(amp) != "UNARY_OPERATOR" or bind is None or _ckind(bind) != "CALL_EXPR"
                    or bind.referenced is None or _qualified(bind.referenced) != "std::bind"
                    or not self.outside(bind.referenced)):
                return f"{where} other than as std::bind(&...) of a subscription"
            args = self.call_args(bind)
            first = _peel(args[2]) if len(args) == 3 else None
            if (len(args) != 3 or _peel(args[0]) != amp
                    or _ckind(_peel(args[1])) != "CXX_THIS_EXPR"
                    or _ckind(first) != "DECL_REF_EXPR" or first.referenced is None
                    or _qualified(first.referenced) != "std::placeholders::_1"):
                return f"{where} in a std::bind other than (&method, this, _1)"
            sub, _c = self.up(bind)
            if not self.is_subscribe(sub) or not any(_peel(a) == bind
                                                     for a in self.call_args(sub)):
                return f"{where} in a std::bind not passed straight to create_subscription"
        return None

    def field_path(self, use):
        """The outermost field access ``use->a.b`` / ``use.a.b`` (through a
        smart pointer's ``operator->``/``operator*``) that starts at *use*;
        *use* itself when it is not dereferenced or accessed."""
        node = use
        p, _c = self.up(node)
        if (p is not None and _ckind(p) == "CALL_EXPR"
                and p.spelling in ("operator->", "operator*") and self.call_args(p)
                and _peel(self.call_args(p)[0]) == node):
            node = p
        while True:
            p, _c = self.up(node)
            if (p is None or _ckind(p) != "MEMBER_REF_EXPR" or p.referenced is None
                    or _ckind(p.referenced) != "FIELD_DECL"):
                return node
            node = p

    def message_written(self, var):
        """``(why, line)`` when the callback body may change the message
        parameter *var* (anywhere in it: no flow analysis), else ``None``.

        Every use must be a read on the ``use_of`` allow-list, taken at the
        outermost field it reaches (``msg->header.stamp = x`` writes). When
        *var* points to a mutable message, the pointer itself may only be
        dereferenced (or cast to void): a copy, alias or ``get()`` of it can
        write the message somewhere the scan does not look."""
        pointee, is_ptr = _pointee(var.type)
        aliasing = is_ptr and (pointee is None or not pointee.is_const_qualified())
        for use in self.refs.get(var, []):
            top = self.field_path(use)
            how = self.use_of(top)
            if how is not None:
                return (how if isinstance(how, str) else "is assigned"), use.location.line
            if aliasing and top == use:
                p, _c = self.up(use)
                if p is not None and _ckind(p) in _ALL_CASTS and p.type.kind.name == "VOID":
                    continue
                return ("is used other than through '->' or '*' while the message it points to "
                        "is mutable (a copy or alias of the pointer can write it)",
                        use.location.line)
        return None

    def follow(self, decl, depth, seen):
        """A local variable or a member of ``this``: every write must give one
        admitted source; any use that may write it otherwise is unknown."""
        name = decl.spelling
        if decl in seen:
            return "unknown", f"'{name}' is set from itself", None, None
        writes = []
        # The initializer (a local's, or a member's default member
        # initializer), a member's constructor initializers, then every
        # assignment. A member with neither keeps its default-constructed
        # value until the first assignment, which the scan does not order
        # against the requests (no flow analysis): it is not counted.
        kids = [k for k in decl.get_children() if _ckind(k) not in _NAME_REFS]
        writes.extend(kids[-1:])
        if _ckind(decl) == "FIELD_DECL":
            writes.extend(self.ctor_inits.get(decl, []))
        for use in self.refs.get(decl, []):
            how = self.use_of(use)
            if how is None:
                continue
            if isinstance(how, str):
                return ("unknown", f"'{name}' {how} at line {use.location.line}; the scan "
                        "cannot tell what it holds", None, None)
            writes.append(how)
        if not writes:
            return ("unknown", f"no initializer or assignment of '{name}' in the node's files",
                    None, None)
        got = {self.classify(w, depth + 1, seen | {decl}) for w in writes}
        kinds = {g[0] for g in got}
        if "foreign_clock" in kinds:
            k, d, cb, tp = next(g for g in got if g[0] == "foreign_clock")
            return k, f"'{name}' = {d}", cb, tp
        if len(got) == 1:
            k, d, cb, tp = next(iter(got))
            return k, f"'{name}' = {d}", cb, tp
        return ("unknown", f"'{name}' is set from different sources: "
                + "; ".join(sorted(g[1] for g in got)), None, None)

    def use_of(self, use):
        """``None`` for a read of the variable at *use*, the assigned
        expression for ``x = <expr>``, or why the use may change it."""
        p, child = self.up(use)
        pk = _ckind(p) if p is not None else ""
        if pk == "CALL_EXPR":
            args = self.call_args(p)
            ref = p.referenced
            if p.spelling == "operator=" and args and _peel(args[0]) == use:
                return args[1] if len(args) == 2 else "is assigned in a form the scan cannot read"
            if ref is None:
                return "is passed to a call the compiler does not resolve"
            if _ckind(ref) == "CXX_METHOD" and p.spelling.startswith("operator") and args \
                    and _peel(args[0]) == use:
                return None if ref.is_const_method() else f"is changed by {p.spelling}"
            params = list(ref.get_arguments()) if _ckind(ref) in (
                "FUNCTION_DECL", "CXX_METHOD", "CONSTRUCTOR", "FUNCTION_TEMPLATE") else []
            for i, a in enumerate(args):
                if _peel(a) == use or a == child:
                    if i >= len(params):
                        return "is passed to a call the scan cannot match to a parameter"
                    t = params[i].type
                    if t.kind.name in ("LVALUEREFERENCE", "RVALUEREFERENCE") \
                            and not t.get_pointee().is_const_qualified():
                        return f"is passed by non-const reference to {p.spelling}"
            return None
        if pk == "MEMBER_REF_EXPR":       # use.method(...)
            call, _c = self.up(p)
            if call is not None and _ckind(call) == "CALL_EXPR" and call.referenced is not None \
                    and _ckind(call.referenced) == "CXX_METHOD":
                return None if call.referenced.is_const_method() else \
                    f"is changed by its method {call.spelling}()"
            return "has a member used in a way the scan cannot read"
        if pk in ("UNARY_OPERATOR", "BINARY_OPERATOR", "COMPOUND_ASSIGNMENT_OPERATOR"):
            return "is used by a built-in operator (address-of or a dependent expression)"
        if pk == "VAR_DECL" and p != use:
            t = p.type
            if t.kind.name == "LVALUEREFERENCE" and not t.get_pointee().is_const_qualified():
                return f"is bound to the non-const reference '{p.spelling}'"
            if t.kind.name == "RVALUEREFERENCE":
                return f"is bound to the reference '{p.spelling}'"
            return None
        if pk == "LAMBDA_EXPR":
            return "is captured by a lambda"
        if pk in _ALL_CASTS:
            # A copy (value type) or a const view reads; a cast to a mutable
            # reference or pointer can write.
            return None if _const_view(p.type) else \
                "is cast to a mutable reference, through which it may be written"
        if pk == "RETURN_STMT":
            fn = self.parent.get(p)
            while fn is not None and _ckind(fn) not in _FUNCTIONS:
                fn = self.parent.get(fn)
            if fn is None or _ckind(fn) == "LAMBDA_EXPR":
                return "is returned from a lambda, whose return type the scan does not read"
            return None if _const_view(fn.result_type) else \
                f"is returned by mutable reference from {fn.spelling}(), which may write it"
        # Fail closed: any other use may write the variable.
        return f"is used in a form the scan cannot read ({pk or 'no parent'})"


def ast_stamps(tu, user_files: set[str], texts: dict[str, bytes], adapters: list[str]
               ) -> tuple[list[Tf2StampInfo], list[Tf2Refusal]]:
    """One ``Tf2StampInfo`` per ``<adapter>->request(...)`` call of the node's
    translation unit *tu* (only cursors in *user_files*), and a
    ``tf2_stamp_unknown`` refusal for a request the AST cannot attribute or
    resolve. *adapters* are the members the source scan read as adapters; one
    the AST does not resolve to ``rcllf_tf2::AcquisitionLookup`` means the
    compiler did not see the adapter (a missing include path), and no stamp
    is classified without it: the node is refused, never read by the regex.
    """
    ast = _StampAst(tu, user_files, texts)
    stamps: list[Tf2StampInfo] = []
    refusals: list[Tf2Refusal] = []
    blind = [a for a in adapters if a not in ast.adapter_fields]
    if blind:
        refusals.append(Tf2Refusal("tf2_stamp_unknown", (
            "the compiler does not resolve AcquisitionLookup member(s) "
            + ", ".join(blind) + " to rcllf_tf2::AcquisitionLookup (the header is not on the "
            "node's include path), so the stamps of their requests cannot be classified; "
            "build the workspace so compile_commands.json or the install tree has rcllf_tf2")))
    if not adapters and not ast.adapter_fields:
        return stamps, refusals
    for call in ast.requests:
        loc = call.location
        fname = os.path.basename(loc.file.name) if loc.file else ""
        where = f"request at {fname}:{loc.line}"
        if call.referenced is None:
            refusals.append(Tf2Refusal("tf2_stamp_unknown", (
                f"{where}: the compiler does not resolve this request call (a dependent or "
                "unresolved expression), so its adapter and stamp are unknown")))
            continue
        member, why = ast.adapter_of(call)
        if member is None:
            if why is not None:
                refusals.append(Tf2Refusal("tf2_stamp_unknown", f"{where}: {why}"))
            continue
        args = ast.call_args(call)
        if len(args) < 3:
            stamps.append(Tf2StampInfo(adapter=member, kind="unknown", expr="",
                                       detail="request call with fewer than three arguments",
                                       file=fname, line=loc.line))
            continue
        kind, detail, callback, topic = ast.classify(args[2])
        stamps.append(Tf2StampInfo(adapter=member, kind=kind, expr=ast.text(args[2]),
                                   detail=detail, callback=callback, topic=topic,
                                   file=fname, line=loc.line))
    stamps.sort(key=lambda s: (s.file, s.line))
    return stamps, refusals


def ingest_callback(member: str, topic: str = TF_TOPIC) -> str:
    """The synthetic callback name of a listener's ``/tf`` or ``/tf_static`` input.

    One name per topic: subscriptions sharing a callback share one reaction
    ID and the binding could not tell the two handles apart.
    """
    # The member's underscores go, as in message_filters' <cb>_via_<member>:
    # the generator sanitizes the callback of an input without a publisher in
    # the system into an LF identifier, which strips them, and the RID row
    # would then no longer find its subscription.
    stem = member.strip("_")
    return f"tf_static_ingest_via_{stem}" if topic == TF_STATIC_TOPIC else f"tf_ingest_via_{stem}"


__all__ = [
    "BROADCASTER_QOS", "LISTENER_QOS", "STATIC_BROADCASTER_QOS", "STATIC_LISTENER_QOS",
    "SUBCODES", "TF_MESSAGE_TYPE", "TF_STATIC_TOPIC", "TF_TOPIC",
    "ast_stamps", "blank_comments", "duration_ns", "ingest_callback", "scan",
]
