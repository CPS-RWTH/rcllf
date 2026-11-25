# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Emit ``node_factories.cpp`` — instance-keyed non-component node construction.

Non-component path:

* every node instance is registered under its **binding key**
  (``generation/binding_key.py``), so ``g_registry``'s class-keyed
  plain-assignment clobber cannot alias two instances of one class;
* each instance is constructed through a generated helper template that uses
  ``if constexpr (std::is_constructible_v<C, const rclcpp::NodeOptions&>)`` to
  pass per-instance ``rclcpp::NodeOptions`` carrying
  ``--ros-args -r __node:=<name> -r __ns:=<namespace>`` from the launch model
  — the only mechanism that overrides a constructor-set node name without
  requiring anything of user code;
* classes that do **not** accept ``NodeOptions`` fall back to
  default-construction plus a one-time ``RCLCPP_WARN`` naming the class and the
  unrealized identity (honest degradation, compile-time detected).

Headerless nodes reached through ``ros2_lf_make_node_{idx}()`` get the same
treatment: the appended factory in the staged source takes the options.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Any

import yaml

__all__ = [
    "HEADERLESS_FACTORY_SIGNATURE",
    "INLINE_PARAMS_FLAG",
    "UnguardedActionClientError",
    "client_cleanup_lines",
    "emit_headerless_factory_suffix",
    "emit_node_factories_cpp",
    "emit_empty_node_factories_cpp",
    "component_param_args",
    "param_args_for",
    "ros_args_for",
]

# Signature of the generated per-node factory appended to a headerless node's
# staged source.  Takes options so the same identity realization applies.
# Registers the node itself (returns void) rather than returning a
# shared_ptr: node_factories.cpp never includes the headerless class's
# header, so it cannot name the concrete type in an extern declaration —
# `shared_ptr<rclcpp::Node>` only worked for rclcpp::Node bases and is wrong
# for rclcpp_lifecycle::LifecycleNode siblings. Registering
# inside the appended factory, where the concrete class is visible, avoids
# needing to spell the type on the node_factories.cpp side at all.
HEADERLESS_FACTORY_SIGNATURE = (
    "void ros2_lf_make_node_{idx}("
    "const char * binding_key, const rclcpp::NodeOptions & options)"
)

# Construct a node with per-instance identity when the class accepts
# rclcpp::NodeOptions; otherwise default-construct and warn once.
# `-r __node:=` / `-r __ns:=` in NodeOptions::arguments() override a
# constructor-set node name, which is why the remap lives here and not in the
# launch file (the launch file emits one Node(...) per wrapper process).
#
# Returns std::shared_ptr<C> (auto-deduced), not std::shared_ptr<rclcpp::Node>:
# rclcpp_lifecycle::LifecycleNode is a sibling of rclcpp::Node, not a
# subclass, so hard-typing the return as rclcpp::Node made every lifecycle
# node's construction fail to compile. The caller passes the
# result straight to ros2_lf_register_node(), whose header-only template
# overload accepts any NodeT and type-erases to NodeBaseInterface via
# get_node_base_interface(), so no per-base branching is needed here.
#
# ros2_lf_node_options() is the one place a node's --ros-args become
# NodeOptions, for header and headerless nodes alike: the inline launch
# parameters (INLINE_PARAMS_FLAG pairs) must become temp params files before
# rclcpp sees them, or it aborts with UnknownROSArgsError.
_HELPER_TEMPLATE = """
namespace {
rclcpp::NodeOptions ros2_lf_node_options(const std::vector<std::string> & ros_args)
{
  rclcpp::NodeOptions options;
  if (!ros_args.empty()) {
    // Inline launch parameters become temp params files here.
    options.arguments(ros2_lf_materialize_param_args(ros_args));
  }
  return options;
}

template<typename C>
auto ros2_lf_construct_node(
  const char * class_name, const std::vector<std::string> & ros_args)
{
  if constexpr (std::is_constructible_v<C, const rclcpp::NodeOptions &>) {
    return std::make_shared<C>(ros2_lf_node_options(ros_args));
  } else {
    static std::once_flag warned;
    std::call_once(
      warned, [class_name]() {
        RCLCPP_WARN(
          rclcpp::get_logger("ros2_lf_binding"),
          "Node class '%s' does not accept NodeOptions; instance identity not "
          "realized (instances share the constructor-defined node name and "
          "namespace). Add a `const rclcpp::NodeOptions &` constructor "
          "parameter to fix.", class_name);
      });
    return std::make_shared<C>();
  }
}
}  // namespace
"""


def _yaml_value(value: Any) -> str:
    """One parameter value as the YAML text of a ``-p name:=value`` rule.

    ``launch_ros`` renders a ``Parameter`` rule with ``yaml.dump(value)``, and
    rcl parses the text back as YAML, so a string that would read as another
    type (``'true'``, ``'1'``) stays quoted and stays a string.
    """
    text = yaml.safe_dump(value, default_flow_style=True, width=float("inf")).strip()
    if text.endswith("\n..."):
        text = text[: -len("\n...")]
    elif text.endswith("..."):
        text = text[: -len("...")].rstrip()
    return text


def _flatten(parameters: dict, prefix: str = "") -> list[tuple[str, Any]]:
    """Nested dicts become dotted names, as ``launch_ros`` flattens them."""
    out: list[tuple[str, Any]] = []
    for name, value in parameters.items():
        full = f"{prefix}{name}"
        if isinstance(value, dict):
            out += _flatten(value, full + ".")
        else:
            out.append((full, value))
    return out


# (node, path) pairs already warned about: the identities are built more than
# once per generate run, the warning should still appear once.
_WARNED_MISSING_FILES: set[tuple[str, str]] = set()


# Pseudo-flag pairing a node's inline launch parameters with their YAML text.
# The binding (ros2_lf_materialize_param_args) writes the text to a temp file
# and passes it as ``--params-file``, as launch_ros does.
INLINE_PARAMS_FLAG = "--rcllf-params-yaml"


def _inline_params_yaml(parameters: dict, key: str) -> str:
    """One inline ``parameters=`` dict as the YAML text of its params file."""
    body = dict(_flatten(dict(parameters)))
    return yaml.safe_dump(
        {key: {"ros__parameters": body}}, default_flow_style=True, width=float("inf"),
    ).strip()


def _file_args(pf, where: str) -> list[str]:
    path = Path(str(pf))
    if not path.is_file():
        if (where, str(path)) not in _WARNED_MISSING_FILES:
            _WARNED_MISSING_FILES.add((where, str(path)))
            print(
                f"  Warning: parameter file {path} of launch node {where or '<unnamed>'} "
                "does not exist here, so the lfc run will not load it.",
                file=sys.stderr,
            )
        return []
    return ["--params-file", str(path)]


def param_args_for(parameters, parameter_files, *, where: str = "", sources=None) -> list[str]:
    """The ``--ros-args`` rules that give a node its launch ``parameters=``.

    *sources* is the analyzer's ordered ``parameter_sources`` list and is
    emitted in launch order, as ``launch_ros`` puts it on a node's command
    line: a file becomes ``--params-file <path>``, an inline dict becomes
    ``INLINE_PARAMS_FLAG`` plus its YAML text filed under the source's
    ``key``. That key is the node's full name when the launch file gives a
    name and a namespace, and ``/**`` otherwise, as ``launch_ros`` chooses
    it. rcl then resolves the rules exactly as for ``ros2 launch``. Without
    *sources* (a launch model from before the field) inline values become
    ``-p name:=<yaml>`` rules ahead of the files, the precedence the analyzer
    and the resolved model assume. A file missing at generate time
    is dropped with a warning, as ``launch_ros`` drops it ("Parameter file path
    is not a file").
    """
    args: list[str] = []
    if sources:
        for source in sources:
            if "file" in source:
                args += _file_args(source["file"], where)
            elif source.get("params"):
                args += [
                    INLINE_PARAMS_FLAG,
                    _inline_params_yaml(source["params"], source.get("key", "/**")),
                ]
        return args
    for name, value in _flatten(dict(parameters or {})):
        args += ["-p", f"{name}:={_yaml_value(value)}"]
    for pf in parameter_files or ():
        args += _file_args(pf, where)
    return args


def _plain(value: Any) -> Any:
    return [_plain(v) for v in value] if isinstance(value, (list, tuple)) else value


def component_param_args(sources, node_name, namespace, *, where: str = "") -> list[str]:
    """The parameter rules of a composable node's launch ``parameters=``.

    ``launch_ros`` does not pass a component's parameters as files. It merges
    the list in order into the ``LoadNode`` request (``to_parameters_list``):
    a file contributes the sections that match the node, an inline dict its
    values, and a later value wins. The same function merges the sources here
    and the result becomes one inline YAML filed under the node's full name,
    the only parameter source of the component's ``NodeOptions``. Files are
    read at generate time, so the system must be regenerated after a
    parameter file changes. Generation runs with ROS sourced; without
    ``launch_ros`` this raises instead of guessing a different precedence.
    """
    try:
        from launch import LaunchContext
        from launch_ros.utilities import to_parameters_list
    except ImportError as exc:
        raise RuntimeError(
            f"{where or 'composable node'}: cannot merge the component's launch parameters: "
            f"import of '{exc.name}' failed ({exc}). Source ROS 2 (launch_ros) before "
            "generating; a component's parameter order is launch_ros' to_parameters_list."
        ) from exc
    evaluated: list[Any] = []
    for source in sources:
        if "file" in source:
            if _file_args(source["file"], where):
                evaluated.append(Path(source["file"]))
        elif source.get("params"):
            evaluated.append(dict(_flatten(dict(source["params"]))))
    merged: dict[str, Any] = {}
    for param in to_parameters_list(LaunchContext(), node_name or "", namespace or "", evaluated):
        merged[param.name] = _plain(param.value)
    if not merged:
        return []
    key = "/" + "/".join(p for p in ((namespace or "").strip("/"), node_name or "") if p)
    if not node_name:
        key = "/**"
    return [INLINE_PARAMS_FLAG, _inline_params_yaml(merged, key)]


def ros_args_for(
    node_name: str, namespace: str, remappings=(), param_args=(),
) -> list[str]:
    """Build the ``--ros-args`` remap vector realizing one instance's identity.

    *remappings* are the launch ``remappings=`` pairs. They are passed as
    ``-r src:=dst`` rules exactly as ``launch_ros`` does, so the node's topics
    carry the names the analyzer resolved its edges to.
    *param_args* are the launch parameter rules from ``param_args_for``,
    appended after the remaps.
    Returns ``[]`` when the launch model gives no name, namespace or remap,
    so single-node systems keep constructing with default options.
    """
    args: list[str] = []
    if node_name:
        args += ["-r", f"__node:={node_name}"]
    if namespace:
        args += ["-r", f"__ns:={namespace}"]
    for src, dst in remappings:
        args += ["-r", f"{src}:={dst}"]
    args += list(param_args)
    if args:
        args = ["--ros-args"] + args
    return args


def _auto_transition_arg(auto_transition: bool) -> str:
    """The third ``ros2_lf_register_node`` argument, or nothing when defaulted.

    Emitted only for a node that opts out, so generated output for every other
    node stays byte-identical.
    """
    return "" if auto_transition else ", /*auto_transition=*/false"


def _cpp_string_vector(values: list[str]) -> str:
    if not values:
        return "{}"
    inner = ", ".join('"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"' for v in values)
    return "{" + inner + "}"


def emit_headerless_factory_suffix(
    idx: int, class_name: str, *, auto_transition: bool = True,
    qualified_name: str = "",
) -> str:
    """Text appended to a headerless node's staged source: its factory function.

    The class is only visible inside that translation unit, so the identity
    realization has to be inlined here rather than reached through the shared
    template in ``node_factories.cpp``.  ``if constexpr`` only discards the
    untaken branch inside a **template**, so the helper is a function template
    over the node class and the factory just instantiates it.

    The appended factory registers the constructed node itself (via the
    header-only ``ros2_lf_register_node`` template) and returns ``void``,
    rather than returning ``std::shared_ptr<rclcpp::Node>`` for
    ``node_factories.cpp`` to register: that file never includes this class's
    header (there is none — it is headerless), so it cannot name the concrete
    type in its extern declaration either. Registering here, where the
    concrete class is visible, sidesteps needing to spell the type on the
    other side, and makes this path base-class-agnostic (rclcpp::Node or
    rclcpp_lifecycle::LifecycleNode alike) with no per-base branching.

    ``auto_transition`` is baked into the emitted call rather than threaded
    through ``HEADERLESS_FACTORY_SIGNATURE``: it is a generation-time decision
    and this suffix is already generated per node, so widening the signature
    would buy nothing.

    ``qualified_name`` is the AST-derived ``ns::Cls`` of the class.  It is
    needed when the staged source only holds ``main()`` and the class is
    declared in an included header inside a namespace, where the bare name is
    not in scope at the end of the file.  Empty falls back to
    ``class_name`` (class visible unqualified, e.g. global namespace).
    """
    cls_ref = qualified_name or class_name
    signature = HEADERLESS_FACTORY_SIGNATURE.format(idx=idx)
    helper = f"ros2_lf_construct_node_{idx}"
    return (
        "\n// Auto-appended by colcon lf-generate: factory for header-less node.\n"
        "namespace {\n"
        "template<typename C>\n"
        f"auto {helper}(\n"
        "  const char * class_name, const rclcpp::NodeOptions & options)\n"
        "{\n"
        "  if constexpr (std::is_constructible_v<C, const rclcpp::NodeOptions &>) {\n"
        "    return std::make_shared<C>(options);\n"
        "  } else {\n"
        "    (void)options;\n"
        "    static std::once_flag ros2_lf_warned;\n"
        "    std::call_once(\n"
        "      ros2_lf_warned, [class_name]() {\n"
        "        RCLCPP_WARN(\n"
        '          rclcpp::get_logger("ros2_lf_binding"),\n'
        '          "Node class \'%s\' does not accept NodeOptions; instance identity "\n'
        '          "not realized (instances share the constructor-defined node name "\n'
        '          "and namespace).", class_name);\n'
        "      });\n"
        "    return std::make_shared<C>();\n"
        "  }\n"
        "}\n"
        "}  // namespace\n"
        f"{signature} {{\n"
        f'  ros2_lf_register_node(binding_key, "{class_name}", '
        f'{helper}<{cls_ref}>("{class_name}", options){_auto_transition_arg(auto_transition)});\n'
        "}\n"
    )


# A service or action type the generated code can name: pkg::srv::Name or
# pkg::action::Name. An alias the analysis could not resolve ("Fibonacci") is
# not one.
_QUALIFIED_INTERFACE = re.compile(r"^(\w+)::(srv|action)::([A-Za-z]\w*)$")


def _interface_header(type_name: str) -> str:
    """rosidl header of a qualified interface type, ``pkg/srv/add_two_ints.hpp``."""
    pkg, kind, name = _QUALIFIED_INTERFACE.match(type_name).groups()
    snake = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    snake = re.sub("([a-z0-9])([A-Z])", r"\1_\2", snake).lower()
    return f"{pkg}/{kind}/{snake}.hpp"


class UnguardedActionClientError(ValueError):
    """A lifecycle node's action client whose type is not qualified.

    While the node is inactive the binding executes the client's goal, result
    and cancel responses with their user callbacks suppressed, so rclcpp
    erases its pending entries. To cancel a goal the server accepted after
    deactivate, and to stop the callbacks of a goal at deactivate, it reads
    the goal id and the goal response's ``accepted`` through operations
    generated per action type. Without the type there are none, and the goal
    would run on at the server with nothing to cancel it, so generation stops.
    """


def client_cleanup_lines(entries: list[dict]) -> tuple[list[str], list[str]]:
    """Includes and registrations of the typed client operations.

    When a lifecycle node leaves the active state, the binding prunes the
    pending requests of its service clients, stops the callbacks of its action
    clients' goals and cancels them. While the node is inactive it reads goal
    ids and goal responses of its action clients. Each rclcpp call needs the
    client's type, which only the generated code knows, so one functor per
    service type and one set of operations per action type of the lifecycle
    nodes is registered here. An unqualified service type is skipped with a
    comment, and the binding warns at runtime for its client. An unqualified
    action type raises ``UnguardedActionClientError``.
    """
    services: list[str] = []
    actions: list[str] = []
    skipped: list[str] = []
    for entry in entries:
        for type_name in entry.get("action_client_types", ()):
            m = _QUALIFIED_INTERFACE.match(type_name or "")
            if m is None or m.group(2) != "action":
                raise UnguardedActionClientError(
                    f"lifecycle node '{entry.get('binding_key', entry.get('class_name'))}' "
                    f"has an action client of type '{type_name}', which is not a "
                    "qualified 'pkg::action::Name'. The binding cannot read its goal "
                    "ids or goal responses, so a goal accepted while the node is "
                    "inactive could not be cancelled. Spell the client's "
                    "type as rclcpp_action::Client<pkg::action::Name> in the node source."
                )
            if type_name not in actions:
                actions.append(type_name)
        for type_name in entry.get("service_client_types", ()):
            m = _QUALIFIED_INTERFACE.match(type_name or "")
            if m is None or m.group(2) != "srv":
                if type_name not in skipped:
                    skipped.append(type_name)
            elif type_name not in services:
                services.append(type_name)
    if not (services or actions or skipped):
        return [], []
    includes = [f'#include "{_interface_header(t)}"' for t in services + actions]
    if actions:
        includes.insert(0, '#include "rclcpp_action/rclcpp_action.hpp"')
    body: list[str] = []
    for type_name in skipped:
        body.append(
            f"  // No typed cleanup for unqualified client type "
            f"'{type_name}'."
        )
    for type_name in services:
        body += [
            "  ros2_lf_register_client_cleanup(",
            "    [](rclcpp::ClientBase & client) -> int64_t {",
            f"      auto * typed = dynamic_cast<rclcpp::Client<{type_name}> *>(&client);",
            "      return typed == nullptr ? -1 : "
            "static_cast<int64_t>(typed->prune_pending_requests());",
            "    });",
        ]
    for type_name in actions:
        client = f"rclcpp_action::Client<{type_name}>"
        impl = f"{type_name}::Impl"
        body += [
            "  {",
            "    Ros2LfActionClientOps ops;",
            "    ops.goal_id = [](rclcpp::Waitable & client, const void * request, bool result,",
            "      Ros2LfGoalId & id) -> bool {",
            f"        if (dynamic_cast<{client} *>(&client) == nullptr) {{return false;}}",
            "        id = result ?",
            f"          static_cast<const {impl}::GetResultService::Request *>(request)"
            "->goal_id.uuid :",
            f"          static_cast<const {impl}::SendGoalService::Request *>(request)"
            "->goal_id.uuid;",
            "        return true;",
            "      };",
            "    ops.goal_accepted = [](rclcpp::Waitable & client, const void * response,",
            "      bool & accepted) -> bool {",
            f"        if (dynamic_cast<{client} *>(&client) == nullptr) {{return false;}}",
            f"        accepted = static_cast<const {impl}::SendGoalService::Response *>"
            "(response)->accepted;",
            "        return true;",
            "      };",
            "    ops.deactivate = [](rclcpp::Waitable & client,",
            "      const std::vector<Ros2LfGoalId> & ids) -> bool {",
            f"        auto * typed = dynamic_cast<{client} *>(&client);",
            "        if (typed == nullptr) {return false;}",
            "        for (const auto & id : ids) {typed->stop_callbacks(id);}",
            "        typed->async_cancel_all_goals();",
            "        return true;",
            "      };",
            "    ros2_lf_register_action_client_ops(std::move(ops));",
            "  }",
        ]
    return includes, body


def emit_node_factories_cpp(entries: list[dict]) -> str:
    """Emit ``node_factories.cpp`` for the given non-component node instances.

    Parameters
    ----------
    entries:
        One dict per node instance, in emission order, with keys:
        ``binding_key``, ``class_name``, ``package``, ``header`` (``"pkg/foo.hpp"``
        or ``None`` for a headerless node), ``idx`` (staging index, used for the
        headerless factory name), ``node_name``, ``namespace``, optionally
        ``remappings`` and ``param_args`` (launch parameter rules), and optionally
        ``auto_transition`` (defaults to True; see ``_auto_transition_arg``),
        ``service_client_types`` and ``action_client_types`` (a lifecycle
        node's client types, see ``client_cleanup_lines``).
    """
    cleanup_includes, cleanup_body = client_cleanup_lines(entries)
    lines: list[str] = [
        "// Auto-generated by colcon lf-generate. DO NOT EDIT.",
        "// Copyright 2026 rcllf Contributors",
        "// SPDX-License-Identifier: Apache-2.0",
        "#include <memory>",
        "#include <mutex>",
        "#include <string>",
        "#include <type_traits>",
        "#include <vector>",
        '#include "rclcpp/rclcpp.hpp"',
        '#include "ros2_lf_binding/node_registry.h"',
    ]

    seen_hdrs: set[str] = set()
    for entry in entries:
        hdr = entry.get("header")
        if hdr and hdr not in seen_hdrs:
            lines.append(f'#include "{hdr}"')
            seen_hdrs.add(hdr)
    lines += cleanup_includes

    lines.append(_HELPER_TEMPLATE.rstrip("\n"))

    extern_lines = [
        HEADERLESS_FACTORY_SIGNATURE.format(idx=entry["idx"]) + ";"
        for entry in entries
        if not entry.get("header")
    ]
    if extern_lines:
        lines += [""] + extern_lines

    lines += ["", "void ros2_lf_register_all_nodes() {"]
    lines += cleanup_body
    for entry in entries:
        binding_key = entry["binding_key"].replace('"', '\\"')
        args_literal = _cpp_string_vector(
            ros_args_for(
                entry.get("node_name", ""), entry.get("namespace", ""),
                entry.get("remappings", ()), entry.get("param_args", ()),
            )
        )
        lines.append(f"  {{")
        lines.append(
            f"    const std::vector<std::string> ros_args = {args_literal};"
        )
        # The subscription gate deepens the reader of an admitted
        # edge created while this node's binding key is in scope.
        lines.append(f'    Ros2LfNodeConstructionScope ros2_lf_scope("{binding_key}");')
        if entry.get("header"):
            pkg = entry.get("package") or ""
            cls = entry["class_name"]
            fqcls = entry.get("qualified_name") or (f"{pkg}::{cls}" if pkg else cls)
            lines.append(
                f'    ros2_lf_register_node("{binding_key}", "{cls}", '
                f'ros2_lf_construct_node<{fqcls}>("{cls}", ros_args)'
                f"{_auto_transition_arg(entry.get('auto_transition', True))});"
            )
        else:
            # Headerless: the factory registers itself (see
            # emit_headerless_factory_suffix) because node_factories.cpp
            # cannot name the concrete class here to receive a return value.
            lines.append(
                f'    ros2_lf_make_node_{entry["idx"]}("{binding_key}", '
                "ros2_lf_node_options(ros_args));"
            )
        lines.append("  }")
    lines += ["}", ""]
    return "\n".join(lines) + "\n"


def emit_empty_node_factories_cpp(note: str = "no non-component nodes in this system.") -> str:
    """Emit the stub used when a system has no non-component nodes."""
    return (
        f"// Auto-generated — {note}\n"
        "void ros2_lf_register_all_nodes() {}\n"
    )
