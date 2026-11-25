# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Parser for ROS2 interface definition files (.msg, .srv, .action).

Extracts field definitions from:
- .msg  files: flat list of fields
- .srv  files: request fields / response fields (separated by ``---``)
- .action files: goal / result / feedback fields (two ``---`` separators)
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class InterfaceField:
    type: str   # e.g. "float64", "float64[]", "std_msgs/Header", "string"
    name: str   # e.g. "temperature"


@dataclass
class MessageDef:
    fields: list[InterfaceField]


@dataclass
class ServiceDef:
    request: list[InterfaceField]
    response: list[InterfaceField]


@dataclass
class ActionDef:
    goal: list[InterfaceField]
    result: list[InterfaceField]
    feedback: list[InterfaceField]


def _parse_idl_fields(text: str) -> list[InterfaceField]:
    """Parse fields from an IDL ``struct { ... }`` block.

    Handles lines like ``uint64 size;``, ``char node_name[48];``,
    and nested type arrays like ``reference_interfaces::msg::Foo bar[10];``.
    Skips constant definitions (``const ...``).
    """
    import re
    fields: list[InterfaceField] = []
    # Find the struct body (first struct block)
    m = re.search(r"struct\s+\w+\s*\{([^}]+)\}", text, re.DOTALL)
    if not m:
        return fields
    body = m.group(1)
    for line in body.splitlines():
        line = line.split("//")[0].strip().rstrip(";").strip()
        if not line or line.startswith("const "):
            continue
        # Match: type name  OR  type name[N]
        match = re.match(r"^(.+?)\s+(\w+)(\[\d+\])?$", line)
        if match:
            field_type = match.group(1).strip()
            field_name = match.group(2)
            array_suffix = match.group(3) or ""
            # Convert IDL :: to ROS2 / separator
            field_type = field_type.replace("::", "/")
            fields.append(InterfaceField(
                type=f"{field_type}{array_suffix}", name=field_name,
            ))
    return fields


def parse_idl_msg(path: Path) -> MessageDef:
    """Parse an IDL message definition file."""
    text = path.read_text(encoding="utf-8")
    return MessageDef(fields=_parse_idl_fields(text))


def _parse_fields(lines: list[str]) -> list[InterfaceField]:
    """Parse a block of lines into a list of InterfaceField objects.

    Each non-comment, non-blank line must be ``type name`` or ``type name = default``.
    Constants (``TYPE NAME = value``) are skipped.
    """
    fields: list[InterfaceField] = []
    for line in lines:
        line = line.split("#", 1)[0].strip()  # strip inline comments
        if not line or line.startswith("---"):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        field_type, field_name = parts[0], parts[1]
        # Skip constant definitions (ALL_CAPS name with = value)
        if "=" in line and field_name.isupper():
            continue
        fields.append(InterfaceField(type=field_type, name=field_name))
    return fields


def _split_sections(path: Path) -> list[list[str]]:
    """Read file and split on ``---`` separator lines into sections."""
    text = path.read_text(encoding="utf-8")
    sections: list[list[str]] = [[]]
    for line in text.splitlines():
        if line.strip() == "---":
            sections.append([])
        else:
            sections[-1].append(line)
    return sections


def parse_msg(path: Path) -> MessageDef:
    """Parse a .msg definition file."""
    sections = _split_sections(path)
    return MessageDef(fields=_parse_fields(sections[0]))


def parse_srv(path: Path) -> ServiceDef:
    """Parse a .srv definition file."""
    sections = _split_sections(path)
    request = _parse_fields(sections[0]) if len(sections) > 0 else []
    response = _parse_fields(sections[1]) if len(sections) > 1 else []
    return ServiceDef(request=request, response=response)


def parse_action(path: Path) -> ActionDef:
    """Parse a .action definition file."""
    sections = _split_sections(path)
    goal = _parse_fields(sections[0]) if len(sections) > 0 else []
    result = _parse_fields(sections[1]) if len(sections) > 1 else []
    feedback = _parse_fields(sections[2]) if len(sections) > 2 else []
    return ActionDef(goal=goal, result=result, feedback=feedback)


def _search_workspace_packages(
    name: str, workspace_root: Path, extensions: list[str],
) -> Path | None:
    """Search all packages under ``workspace_root/src/`` for a short interface name.

    Tries every ``src/<pkg>/<kind>/<name><ext>`` combination.  Returns the first
    match, or *None* if nothing is found.
    """
    src_dir = workspace_root / "src"
    if not src_dir.is_dir():
        return None
    for ext in extensions:
        kind = ext.lstrip(".")  # ".msg" -> "msg"
        for pkg_dir in src_dir.iterdir():
            if not pkg_dir.is_dir():
                continue
            candidate = pkg_dir / kind / f"{name}{ext}"
            if candidate.exists():
                return candidate
            # Also search nested mono-repo layouts (e.g. navigation2/nav2_msgs)
            for sub_dir in pkg_dir.iterdir():
                if not sub_dir.is_dir():
                    continue
                candidate = sub_dir / kind / f"{name}{ext}"
                if candidate.exists():
                    return candidate
    return None


def _search_system_packages(
    pkg: str, kind: str, name: str, ext: str,
) -> Path | None:
    """Search system-installed ROS2 packages for an interface file.

    Uses ``ament_index_python`` when available, then falls back to common
    install prefixes (``/opt/ros/*/share/``).
    """
    # Try ament_index_python first (works inside ROS2 environment)
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory(pkg))
        candidate = share_dir / kind / f"{name}{ext}"
        if candidate.exists():
            return candidate
    except Exception:
        pass

    # Fallback: scan common ROS2 install prefixes
    ros_root = Path("/opt/ros")
    if ros_root.is_dir():
        for distro_dir in ros_root.iterdir():
            candidate = distro_dir / "share" / pkg / kind / f"{name}{ext}"
            if candidate.exists():
                return candidate

    return None


def resolve_interface_file(type_str: str, workspace_root: Path) -> Path | None:
    """Resolve a C++ type string to the source .msg/.srv/.action file path.

    Handles fully-qualified types like:
    - ``custom_interfaces::msg::SensorData`` -> ``src/custom_interfaces/msg/SensorData.msg``
    - ``custom_interfaces::srv::ProcessData`` -> ``src/custom_interfaces/srv/ProcessData.srv``
    - ``custom_interfaces::action::Analyze``  -> ``src/custom_interfaces/action/Analyze.action``

    Also handles short names (e.g. ``Analyze``) by searching all workspace
    packages, and falls back to system-installed ROS2 packages for standard
    message types like ``std_msgs::msg::String``.

    Returns None if the type cannot be resolved or the file does not exist.
    """
    # Strip trailing _<suffix> from generated type names (e.g. "Analyze_Goal_")
    # and shared_ptr wrappers ("SensorData::SharedPtr" -> "SensorData")
    type_str = type_str.split("::SharedPtr")[0].split("::ConstSharedPtr")[0]

    parts = type_str.split("::")

    # Short name (e.g. "Analyze") — search all workspace packages
    if len(parts) < 3:
        name = parts[-1]
        return _search_workspace_packages(
            name, workspace_root, [".action", ".srv", ".msg", ".idl"],
        )

    pkg = parts[0]
    kind = parts[1].lower()   # "msg", "srv", or "action"
    name = parts[2]

    ext_map = {"msg": ".msg", "srv": ".srv", "action": ".action"}
    ext = ext_map.get(kind)
    if ext is None:
        return None

    # Try workspace source tree first (.msg/.srv/.action, then .idl fallback)
    for check_ext in (ext, ".idl"):
        candidate = workspace_root / "src" / pkg / kind / f"{name}{check_ext}"
        if candidate.exists():
            return candidate

    # Try nested mono-repo layouts (e.g. src/reference-system/reference_interfaces/)
    src_dir = workspace_root / "src"
    if src_dir.is_dir():
        for parent_dir in src_dir.iterdir():
            if not parent_dir.is_dir():
                continue
            for check_ext in (ext, ".idl"):
                candidate = parent_dir / pkg / kind / f"{name}{check_ext}"
                if candidate.exists():
                    return candidate

    # Fallback: system-installed ROS2 packages (e.g. std_msgs, sensor_msgs)
    return _search_system_packages(pkg, kind, name, ext)


def parse_interface(type_str: str, workspace_root: Path) -> MessageDef | ServiceDef | ActionDef | None:
    """Resolve and parse a ROS2 interface type string.

    Returns the parsed definition (MessageDef, ServiceDef, or ActionDef),
    or None if the file cannot be found or parsed.
    """
    path = resolve_interface_file(type_str, workspace_root)
    if path is None:
        return None
    try:
        if path.suffix == ".msg":
            return parse_msg(path)
        if path.suffix == ".srv":
            return parse_srv(path)
        if path.suffix == ".action":
            return parse_action(path)
        if path.suffix == ".idl":
            return parse_idl_msg(path)
    except Exception:
        pass
    return None


def fields_to_dicts(fields: list[InterfaceField]) -> list[dict[str, Any]]:
    """Convert a list of InterfaceField to a JSON-serializable list of dicts."""
    return [{"type": f.type, "name": f.name} for f in fields]
