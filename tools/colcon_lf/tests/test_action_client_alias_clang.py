# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""libclang snippet tests for action client types named by an alias.

The generator refuses a lifecycle node's action client whose type is not a
qualified ``pkg::action::Name``. Nav2's ``WaypointFollower`` names its client
type with the class alias ``ClientT``, and the analysis kept ``ClientT`` even
with every header resolved. The analysis now follows ``using``/``typedef``
aliases in the source to the qualified type.

To falsify: make ``_resolve_action_type_aliases`` return at once and
``test_class_alias`` fails with ``['ClientT']``. Restore the lookup's old
fallback (the only alias of that name anywhere in the translation unit) and
``test_unrelated_alias_is_refused`` fails with
``['other_pkg::action::Wrong']``.
"""

from __future__ import annotations

from pathlib import Path

import pytest

clang = pytest.importorskip("clang.cindex", reason="libclang not available")

from .ros_env import requires_ros_headers  # noqa: E402

pytestmark = requires_ros_headers("the action client alias snippet tests")


_NODE_TEMPLATE = """\
#include <memory>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace demo
{{

// Declared, so libclang keeps the decoy aliases below.
namespace other_pkg::action
{{
struct Wrong {{}};
}}  // namespace other_pkg::action

{prelude}

class Decoy
{{
public:
  using ClientT = other_pkg::action::Wrong;
}};

class SnippetClient : public rclcpp_lifecycle::LifecycleNode
{{
public:
{aliases}
  SnippetClient() : LifecycleNode("snippet_client") {{}}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {{
    client_ = rclcpp_action::create_client<{client_type}>(this, "go");
    return CallbackReturn::SUCCESS;
  }}

private:
  rclcpp_action::Client<{client_type}>::SharedPtr client_;
}};

}}  // namespace demo

int main(int argc, char ** argv)
{{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<demo::SnippetClient>();
  rclcpp::shutdown();
  return 0;
}}
"""


def _client_types(
    tmp_path: Path, aliases: str, prelude: str = "", client_type: str = "ClientT",
) -> list[str]:
    from colcon_lf.node_analysis.clang_parser import parse_node

    cpp_path = tmp_path / "snippet_client.cpp"
    cpp_path.write_text(_NODE_TEMPLATE.format(
        aliases=aliases, prelude=prelude, client_type=client_type), encoding="utf-8")
    node = parse_node(cpp_path, workspace_root=tmp_path)
    assert node is not None, "snippet node class not found"
    assert [c.action_name for c in node.action_clients] == ["go"]
    return [c.action_type for c in node.action_clients]


class TestActionClientAlias:
    def test_class_alias(self, tmp_path):
        """Nav2 WaypointFollower's pattern. Before the fix the type was
        ``ClientT``, which the generator refuses for a lifecycle node. The
        class's own alias wins over Decoy's alias of the same name."""
        types = _client_types(
            tmp_path, "  using ClientT = example_interfaces::action::Fibonacci;")
        assert types == ["example_interfaces::action::Fibonacci"]

    def test_alias_chain(self, tmp_path):
        types = _client_types(
            tmp_path,
            "  using FibT = example_interfaces::action::Fibonacci;\n"
            "  using ClientT = FibT;")
        assert types == ["example_interfaces::action::Fibonacci"]

    def test_typedef(self, tmp_path):
        types = _client_types(
            tmp_path, "  typedef example_interfaces::action::Fibonacci ClientT;")
        assert types == ["example_interfaces::action::Fibonacci"]

    def test_unrelated_alias_is_refused(self, tmp_path):
        """Review, item 4. The client type ``Go`` is a class that
        comes in by a using-declaration, and the only alias named ``Go`` is a
        member of the unrelated class Other. Before the fix the lookup took that alias and
        named the wrong action. Now the type stays unresolved, so the
        generator refuses the client."""
        types = _client_types(
            tmp_path, "",
            prelude=(
                "namespace mine::action\n{\n"
                "struct Go : example_interfaces::action::Fibonacci {};\n}\n"
                "using mine::action::Go;\n"
                "class Other\n{\npublic:\n"
                "  using Go = other_pkg::action::Wrong;\n};"),
            client_type="Go")
        assert types == ["Go"]

    def test_qualified_alias(self, tmp_path):
        """``tools::ClientT`` resolves in namespace ``demo::tools``, not to
        Decoy's ``ClientT``."""
        types = _client_types(
            tmp_path, "",
            prelude=(
                "namespace tools\n{\n"
                "using ClientT = example_interfaces::action::Fibonacci;\n}"),
            client_type="tools::ClientT")
        assert types == ["example_interfaces::action::Fibonacci"]
