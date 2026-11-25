# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Name sanitization utilities for ROS2 → LF conversion.

Handles conversion of ROS2 topic names, message types, callback names,
and launch file stems into valid LF identifiers.
"""

from __future__ import annotations

import re

# LF identifiers: start with letter or underscore, then alnum or underscore
_INVALID_IDENT_CHARS = re.compile(r"[^a-zA-Z0-9_]")
_VALID_IDENT = re.compile(r"[A-Za-z][A-Za-z0-9_]*")

# Prefix for a main reactor whose launch stem starts with a digit.
DIGIT_STEM_PREFIX = "Launch_"

# C++ reserved words that lfc rejects as LF identifier names.
_CPP_RESERVED = frozenset({
    "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand", "bitor",
    "bool", "break", "case", "catch", "char", "char8_t", "char16_t", "char32_t",
    "class", "compl", "concept", "const", "consteval", "constexpr", "constinit",
    "const_cast", "continue", "co_await", "co_return", "co_yield", "decltype",
    "default", "delete", "do", "double", "dynamic_cast", "else", "enum",
    "explicit", "export", "extern", "false", "float", "for", "friend", "goto",
    "if", "inline", "int", "long", "mutable", "namespace", "new", "noexcept",
    "not", "not_eq", "nullptr", "operator", "or", "or_eq", "private",
    "protected", "public", "register", "reinterpret_cast", "requires", "return",
    "short", "signed", "sizeof", "static", "static_assert", "static_cast",
    "struct", "switch", "template", "this", "thread_local", "throw", "true",
    "try", "typedef", "typeid", "typename", "union", "unsigned", "using",
    "virtual", "void", "volatile", "wchar_t", "while", "xor", "xor_eq",
})


def sanitize_lf_identifier(name: str) -> str:
    """Ensure *name* is a valid LF identifier.

    Replaces invalid characters with underscores, strips leading/trailing
    underscores (except one leading if needed), prepends ``_`` if the
    result starts with a digit, and appends ``_id`` if the result is a
    C++ reserved word (which lfc disallows).

    KEEP IN SYNC with tools/ros2lf/ros2lf/api/discovery.py's
    ``_sanitize_lf_identifier`` — ros2lf depends only on ros2cli and
    cannot import colcon_lf, so it re-implements this transform (used
    together with ``launch_stem_to_reactor_name`` below to recognize
    ``--target-lfc``'s ``lf_generated_<name>`` artifact layout).
    """
    result = _INVALID_IDENT_CHARS.sub("_", name)
    result = result.strip("_") or "unnamed"
    if result[0].isdigit():
        result = f"_{result}"
    if result in _CPP_RESERVED:
        result = f"{result}_id"
    return result


def topic_to_port_name(topic: str) -> str:
    """Convert a ROS2 topic like ``/ns/sensor_data`` to an LF port name.

    Strips leading ``/``, replaces ``/`` with ``_``.
    """
    name = topic.lstrip("/")
    name = name.replace("/", "_")
    return sanitize_lf_identifier(name)


def message_type_to_lf_type(msg_type: str) -> str:
    """Convert a C++ message type to a simplified LF type name.

    ``custom_interfaces::msg::SensorData`` → ``SensorData``
    ``std_msgs::msg::String`` → ``String``
    ``reference_interfaces::msg::Message4kb`` → ``Message4kb``

    Also strips template allocator noise from extracted types.
    """
    # Strip allocator template suffix: SensorData_<std::allocator<void>> → SensorData
    clean = re.sub(r"_<[^>]*>", "", msg_type)
    # Take last component after ::
    parts = clean.split("::")
    return sanitize_lf_identifier(parts[-1])


def callback_to_timer_name(callback_name: str) -> str:
    """Convert a callback name to an LF timer name: ``timer_callback`` → ``t_timer_callback``."""
    return f"t_{sanitize_lf_identifier(callback_name)}"


def launch_stem_to_reactor_name(stem: str) -> str:
    """Convert a launch file stem to a PascalCase main reactor name.

    ``pubsub_example_bringup`` → ``PubsubExampleBringup``
    ``bringup`` → ``Bringup``

    KEEP IN SYNC with tools/ros2lf/ros2lf/api/discovery.py's
    ``_launch_stem_to_reactor_name`` — see the note on
    ``sanitize_lf_identifier`` above.
    A stem starting with a digit (``11_load_composable_nodes``) gets the
    fixed prefix ``Launch_``, because lfc rejects LF file and reactor names
    that start with a number. Joined names never contain an
    underscore, so the prefix is unambiguous and ``generated_package_stem``
    drops it again. A stem that still gives no valid identifier is refused.
    """
    parts = stem.split("_")
    name = "".join(p.capitalize() for p in parts if p) or "Main"
    if name[0].isdigit():
        name = f"{DIGIT_STEM_PREFIX}{name}"
    if not _VALID_IDENT.fullmatch(name):
        raise ValueError(
            f"launch file stem {stem!r} gives main reactor name {name!r}, which is "
            "not a valid LF identifier (letters, digits and underscores only); "
            "rename the launch file"
        )
    return name


def generated_package_stem(reactor_name: str) -> str:
    """The lower-cased ``<name>`` half of ``lf_generated_<package>_<name>``.

    It is the main reactor name lowered, without the ``Launch_`` prefix that
    ``launch_stem_to_reactor_name`` adds to a digit-leading stem, so the
    package and its ``lf_<name>.launch.py`` keep the bare stem
    (``lf_generated_launch_features_example_11loadcomposablenodes``).
    Every emitter that derives a package name from the reactor name or the
    ``.lf`` file stem goes through here.

    KEEP IN SYNC with tools/ros2lf/ros2lf/api/discovery.py's
    ``_generated_package_stem``.
    """
    if reactor_name.startswith(DIGIT_STEM_PREFIX) and reactor_name[
        len(DIGIT_STEM_PREFIX):len(DIGIT_STEM_PREFIX) + 1
    ].isdigit():
        reactor_name = reactor_name[len(DIGIT_STEM_PREFIX):]
    return reactor_name.lower()


def generated_lfc_package_name(source_package: str, reactor_stem_lower: str) -> str:
    """The ``lf_generated_<package>_<stem>`` colcon package name.

    Before the package prefix, the name depended only on the launch stem
    (``reactor_stem_lower``, e.g. ``lf_system.name.lower()``), so two source
    packages analyzing a same-named launch file (``pubsub_example`` and
    ``service_example`` both have ``bringup.launch.py``) produced the same
    ``lf_generated_bringup`` package and silently overwrote each other when
    built into the same workspace.

    The package prefix keeps the source ROS2 package name (underscores and
    all) while the stem half drops its underscores, so the mapping is NOT
    injective: ``mt_main`` and ``mtmain`` of one package, or package ``a_b``
    with stem ``c`` and package ``a`` with stem ``b_c``, give the same name.
    ``colcon lf-analyze`` refuses such a pair before it writes anything
    (``verb/analyze.py`` ``_refuse_generated_name_collisions``).

    KEEP IN SYNC with tools/ros2lf/ros2lf/api/discovery.py's
    ``generated_package_name`` and the benchmark harness's
    ``_generated_package_name`` — see the note on ``sanitize_lf_identifier``
    above for why those are reimplemented rather than imported.
    """
    pkg = sanitize_lf_identifier(source_package).lower()
    return f"lf_generated_{pkg}_{reactor_stem_lower}"


def class_name_to_reactor_name(class_name: str) -> str:
    """Ensure a C++ class name is a valid LF reactor name (PascalCase).

    Most ROS2 class names are already PascalCase. This just sanitizes
    any invalid characters.
    """
    return sanitize_lf_identifier(class_name)


def service_to_request_port_name(service_name: str) -> str:
    """Convert a ROS2 service name like ``/process_data`` to an LF input port name.

    Strips leading ``/``, replaces ``/`` with ``_``, appends ``_req``.
    """
    name = service_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_req"


def service_to_response_port_name(service_name: str) -> str:
    """Convert a ROS2 service name like ``/process_data`` to an LF input port name.

    Strips leading ``/``, replaces ``/`` with ``_``, appends ``_resp``.
    """
    name = service_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_resp"


def action_to_goal_req_port_name(action_name: str) -> str:
    """Convert a ROS2 action name like ``analyze`` to an LF input port name."""
    name = action_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_goal_req"


def action_to_goal_resp_port_name(action_name: str) -> str:
    """Convert a ROS2 action name like ``analyze`` to an LF input/output port name."""
    name = action_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_goal_resp"


def action_to_feedback_port_name(action_name: str) -> str:
    """Convert a ROS2 action name like ``analyze`` to an LF input port name."""
    name = action_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_feedback"


def action_to_result_port_name(action_name: str) -> str:
    """Convert a ROS2 action name like ``analyze`` to an LF input port name."""
    name = action_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_result"


def action_to_result_req_port_name(action_name: str) -> str:
    """Convert a ROS2 action name like ``analyze`` to an LF result-request port name."""
    name = action_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_result_req"


def action_to_cancel_req_port_name(action_name: str) -> str:
    """Convert a ROS2 action name like ``analyze`` to an LF cancel-request port name."""
    name = action_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_cancel_req"


def action_to_cancel_resp_port_name(action_name: str) -> str:
    """Convert a ROS2 action name like ``analyze`` to an LF cancel-response port name."""
    name = action_name.lstrip("/").replace("/", "_")
    return sanitize_lf_identifier(name) + "_cancel_resp"


def deduplicate_names(names: list[str]) -> list[str]:
    """Append ``_2``, ``_3``, etc. to duplicate names in *names*.

    >>> deduplicate_names(["a", "b", "a", "a"])
    ['a', 'b', 'a_2', 'a_3']
    """
    seen: dict[str, int] = {}
    result: list[str] = []
    for name in names:
        count = seen.get(name, 0)
        if count == 0:
            result.append(name)
        else:
            result.append(f"{name}_{count + 1}")
        seen[name] = count + 1
    return result
