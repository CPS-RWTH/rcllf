# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""libclang snippet tests for Phase 5 QoS extraction hardening + H003 include scan.

Scope:

- defaults materialization: bare-int QoS materializes reliable/volatile/
  keep_last so ``None`` genuinely means "could not parse".
- named-profile recognition: ``SensorDataQoS``/``SystemDefaultsQoS``/
  ``rmw_qos_profile_sensor_data`` materialize attrs even with no chained
  method calls.
- the message_filters include scan (``_scan_detected_includes``) populates
  ``NodeInfo.detected_includes``.

These tests need real rclcpp headers to resolve ``_find_node_class`` (the
class must be recognized as inheriting from ``rclcpp::Node``), which this
host does not have (no ``/opt/ros`` install — the same environment gap the
existing ``test_clang_parser.py`` suite hits). Skip cleanly, don't fail,
when no ROS2 distro include tree is discoverable; full coverage comes from
the real ``qos_hazard_example`` fixture package + container CI.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the QoS-hardening snippet tests")


def _parse_snippet(tmp_path: Path, filename: str, source: str):
    """Write *source* to *tmp_path/filename* and parse it with parse_node.

    Uses a throwaway workspace root (tmp_path itself) with no build/ or
    install/ dir — include-flag discovery falls back to the /opt/ros
    auto-discovery path in ``_load_include_flags``.
    """
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp_path = tmp_path / filename
    cpp_path.write_text(source, encoding="utf-8")
    return parse_node(cpp_path, workspace_root=tmp_path)


_NODE_TEMPLATE = """\
#include "rclcpp/rclcpp.hpp"
{extra_includes}

class SnippetNode : public rclcpp::Node
{{
public:
  SnippetNode() : Node("snippet_node")
  {{
{body}
  }}

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
}};

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SnippetNode>());
  rclcpp::shutdown();
  return 0;
}}
"""


class TestQosDefaultsMaterialization:
    def test_bare_int_qos_materializes_defaults(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"',
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", 10, [](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "bare_int_qos.cpp", source)
        assert len(node.subscribers) == 1
        sub = node.subscribers[0]
        assert sub.qos_depth == 10
        assert sub.qos_reliability == "reliable"
        assert sub.qos_durability == "volatile"
        assert sub.qos_history == "keep_last"

    def test_explicit_best_effort_not_overridden_by_materialization(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"',
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", rclcpp::QoS(10).best_effort(), '
                '[](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "explicit_best_effort.cpp", source)
        assert len(node.subscribers) == 1
        assert node.subscribers[0].qos_reliability == "best_effort"


class TestChainedQosBuilders:
    """Chained ``rclcpp::QoS`` builder calls must resolve to the
    stated values instead of RCLLF-H000 ("undetermined")."""

    def test_reliable_and_durability_volatile_chain_resolved(self, tmp_path):
        """The exact chain from the issue: ``.reliable().durability_volatile()``."""
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"',
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", rclcpp::QoS(10).reliable().durability_volatile(), '
                '[](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "reliable_durability_volatile.cpp", source)
        assert len(node.subscribers) == 1
        sub = node.subscribers[0]
        assert sub.qos_reliability == "reliable"
        assert sub.qos_durability == "volatile"
        assert sub.qos_depth == 10

    def test_transient_local_resolved(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"',
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", rclcpp::QoS(10).transient_local(), '
                '[](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "transient_local.cpp", source)
        assert len(node.subscribers) == 1
        assert node.subscribers[0].qos_durability == "transient_local"

    def test_keep_all_resolved(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"',
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", rclcpp::QoS(10).keep_all(), '
                '[](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "keep_all.cpp", source)
        assert len(node.subscribers) == 1
        assert node.subscribers[0].qos_history == "keep_all"

    def test_keep_last_overrides_base_constructor_depth(self, tmp_path):
        """Falsification companion: before the fix, ``_extract_qos_depth``
        returned the *first* INTEGER_LITERAL found by a preorder walk over
        the whole QoS argument subtree — which is the base ``QoS(10))``'s
        own literal, visited before the ``.keep_last(3)`` argument because
        the member-call's callee/receiver subtree is walked first. So a
        chained ``.keep_last(3)`` silently lost to the base depth and this
        assertion (``qos_depth == 3``) failed with ``qos_depth == 10``
        instead — the exact wrong value the bug would produce, not just any
        failure.
        """
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"',
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", rclcpp::QoS(10).keep_last(3), '
                '[](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "keep_last_override.cpp", source)
        assert len(node.subscribers) == 1
        assert node.subscribers[0].qos_depth == 3


class TestNamedQosProfiles:
    def test_sensor_data_qos_materializes_best_effort_depth_5(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"',
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", rclcpp::SensorDataQoS(), '
                '[](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "sensor_data_qos.cpp", source)
        assert len(node.subscribers) == 1
        sub = node.subscribers[0]
        assert sub.qos_reliability == "best_effort"
        assert sub.qos_depth == 5

    def test_system_defaults_qos_materializes_reliable(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"',
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", rclcpp::SystemDefaultsQoS(), '
                '[](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "system_defaults_qos.cpp", source)
        assert len(node.subscribers) == 1
        assert node.subscribers[0].qos_reliability == "reliable"


class TestQosStringLiteralNotMatched:
    """A word inside a string literal in the QoS argument must not
    set the analyzed QoS. Before the fix this subscription was recorded as
    ``qos_reliability == 'best_effort'`` because the literal
    ``"best_effort_topic"`` contained the word, so RCLLF-H001 fired on a
    reliable subscription."""

    def test_literal_argument_of_qos_helper_leaves_reliability_unknown(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes=(
                '#include "std_msgs/msg/string.hpp"\n'
                '#include <cstdlib>\n'
                '#include <string>\n'
                'static rclcpp::QoS runtime_qos(const std::string & topic)\n'
                '{\n'
                '  rclcpp::QoS qos(10);\n'
                '  if (std::getenv("RCLLF_BE") != nullptr && topic.size() > 0) {qos.best_effort();}\n'
                '  return qos;\n'
                '}'
            ),
            body=(
                '    sub_ = this->create_subscription<std_msgs::msg::String>('
                '"topic", runtime_qos("best_effort_topic"), '
                '[](std_msgs::msg::String::SharedPtr) {});'
            ),
        )
        node = _parse_snippet(tmp_path, "literal_qos_arg.cpp", source)
        assert len(node.subscribers) == 1
        assert node.subscribers[0].qos_reliability is None


class TestMessageFiltersIncludeScan:
    def test_message_filters_include_detected(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "message_filters/subscriber.h"',
            body="",
        )
        node = _parse_snippet(tmp_path, "with_message_filters.cpp", source)
        assert "message_filters" in node.detected_includes

    def test_no_message_filters_include_not_detected(self, tmp_path):
        source = _NODE_TEMPLATE.format(extra_includes="", body="")
        node = _parse_snippet(tmp_path, "without_message_filters.cpp", source)
        assert "message_filters" not in node.detected_includes


class TestTf2IncludeScan:
    """RCLLF-R002 (T5): tf2_ros usage detected via the same
    allowlisted include scan as message_filters (`_DETECTED_INCLUDES_ALLOWLIST`)."""

    def test_tf2_ros_include_detected(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes=(
                '#include "tf2_ros/buffer.h"\n'
                '#include "tf2_ros/transform_listener.h"'
            ),
            body="",
        )
        node = _parse_snippet(tmp_path, "with_tf2.cpp", source)
        assert "tf2_ros" in node.detected_includes

    def test_no_tf2_include_not_detected(self, tmp_path):
        source = _NODE_TEMPLATE.format(extra_includes="", body="")
        node = _parse_snippet(tmp_path, "without_tf2.cpp", source)
        assert "tf2_ros" not in node.detected_includes


class TestTakeCallScan:
    """RCLLF-R002 (T5): a narrow member-call scan for explicit
    ``take``/``take_serialized``/``take_loaned`` (test-only fixture — no
    real package under packages/src calls these)."""

    def test_take_call_detected(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes="",
            body=(
                "    std_msgs::msg::String msg;\n"
                "    rclcpp::MessageInfo info;\n"
                "    sub_->take(msg, info);"
            ),
        )
        node = _parse_snippet(tmp_path, "with_take.cpp", source)
        assert "take" in node.detected_take_calls

    def test_take_serialized_call_detected(self, tmp_path):
        source = _NODE_TEMPLATE.format(
            extra_includes="",
            body="    sub_.take_serialized(msg, info);",
        )
        node = _parse_snippet(tmp_path, "with_take_serialized.cpp", source)
        assert "take_serialized" in node.detected_take_calls

    def test_no_take_call_not_detected(self, tmp_path):
        source = _NODE_TEMPLATE.format(extra_includes="", body="")
        node = _parse_snippet(tmp_path, "without_take.cpp", source)
        assert node.detected_take_calls == []

    def test_bare_take_identifier_not_a_call_not_detected(self, tmp_path):
        """Falsification companion: a bare ``take`` token that is not a
        member call (e.g. a local variable name) must not trigger detection
        — guards against widening the scan back to a plain token match."""
        source = _NODE_TEMPLATE.format(
            extra_includes="",
            body="    int take = 0;\n    (void)take;",
        )
        node = _parse_snippet(tmp_path, "bare_take.cpp", source)
        assert node.detected_take_calls == []


class TestLocalQosVariable:
    """A QoS profile held in a local variable (OWED analyzer-qos-local-variable).

    Before the fix the extractor read only the call's own QoS argument tokens,
    which are just ``qos``: depth and every attribute stayed None (the AEB
    nodes' 12 unresolved qos records).
    """

    def _sub(self, tmp_path, name, body):
        source = _NODE_TEMPLATE.format(
            extra_includes='#include "std_msgs/msg/string.hpp"', body=body)
        node = _parse_snippet(tmp_path, name, source)
        assert len(node.subscribers) == 1
        return node.subscribers[0]

    def test_local_variable_with_chain_resolves(self, tmp_path):
        sub = self._sub(tmp_path, "local_qos.cpp", (
            '    auto qos = rclcpp::QoS(rclcpp::KeepLast(7)).best_effort().durability_volatile();\n'
            '    sub_ = this->create_subscription<std_msgs::msg::String>('
            '"topic", qos, [](std_msgs::msg::String::SharedPtr) {});'))
        assert (sub.qos_depth, sub.qos_reliability, sub.qos_durability, sub.qos_history) == (
            7, "best_effort", "volatile", "keep_last")

    def test_copy_of_local_variable_resolves(self, tmp_path):
        sub = self._sub(tmp_path, "copy_qos.cpp", (
            '    auto qos = rclcpp::QoS(rclcpp::KeepLast(4)).reliable();\n'
            '    auto qos_b = qos;\n'
            '    sub_ = this->create_subscription<std_msgs::msg::String>('
            '"topic", qos_b, [](std_msgs::msg::String::SharedPtr) {});'))
        assert (sub.qos_depth, sub.qos_reliability) == (4, "reliable")

    def test_variable_changed_after_declaration_stays_unresolved(self, tmp_path):
        """A later ``qos.best_effort()`` statement may change the profile: the
        extractor must not report the declaration's reliable default."""
        sub = self._sub(tmp_path, "mutated_qos.cpp", (
            '    auto qos = rclcpp::QoS(5);\n'
            '    qos.best_effort();\n'
            '    sub_ = this->create_subscription<std_msgs::msg::String>('
            '"topic", qos, [](std_msgs::msg::String::SharedPtr) {});'))
        assert (sub.qos_depth, sub.qos_reliability) == (None, None)

    def test_reference_alias_mutation_stays_unresolved(self, tmp_path):
        """``auto & alias = qos; alias.best_effort();`` changes ``qos`` through
        the alias: the declaration's reliable default is stale."""
        sub = self._sub(tmp_path, "ref_alias_qos.cpp", (
            '    auto qos = rclcpp::QoS(5);\n'
            '    auto & alias = qos;\n'
            '    alias.best_effort();\n'
            '    sub_ = this->create_subscription<std_msgs::msg::String>('
            '"topic", qos, [](std_msgs::msg::String::SharedPtr) {});'))
        assert (sub.qos_depth, sub.qos_reliability) == (None, None), (
            f"stale initializer reported through a reference alias: "
            f"depth {sub.qos_depth}, reliability {sub.qos_reliability}")

    def test_pointer_alias_mutation_stays_unresolved(self, tmp_path):
        sub = self._sub(tmp_path, "ptr_alias_qos.cpp", (
            '    auto qos = rclcpp::QoS(5);\n'
            '    auto * alias = &qos;\n'
            '    alias->best_effort();\n'
            '    sub_ = this->create_subscription<std_msgs::msg::String>('
            '"topic", qos, [](std_msgs::msg::String::SharedPtr) {});'))
        assert (sub.qos_depth, sub.qos_reliability) == (None, None), (
            f"stale initializer reported through a pointer alias: "
            f"depth {sub.qos_depth}, reliability {sub.qos_reliability}")

    def test_long_copy_chain_resolves(self, tmp_path):
        """Six by-value copies: following must not stop after a fixed number of links."""
        copies = ''.join(f'    auto q{i} = q{i - 1};\n' for i in range(1, 7))
        sub = self._sub(tmp_path, "chain_qos.cpp", (
            '    auto q0 = rclcpp::QoS(rclcpp::KeepLast(6)).best_effort();\n' + copies +
            '    sub_ = this->create_subscription<std_msgs::msg::String>('
            '"topic", q6, [](std_msgs::msg::String::SharedPtr) {});'))
        assert (sub.qos_depth, sub.qos_reliability) == (6, "best_effort"), (
            f"copy chain of 6 links not followed to its initializer: "
            f"depth {sub.qos_depth}, reliability {sub.qos_reliability}")

    def test_self_initialized_variable_terminates_unresolved(self, tmp_path):
        """``rclcpp::QoS q = q;`` is a cycle: following stops, nothing is resolved."""
        sub = self._sub(tmp_path, "cycle_qos.cpp", (
            '    rclcpp::QoS q = q;\n'
            '    sub_ = this->create_subscription<std_msgs::msg::String>('
            '"topic", q, [](std_msgs::msg::String::SharedPtr) {});'))
        assert (sub.qos_depth, sub.qos_reliability) == (None, None)
