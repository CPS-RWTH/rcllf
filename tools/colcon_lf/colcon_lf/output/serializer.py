# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""JSON serialization and deserialization for all colcon-lf data models.

Handles:
- Path objects <-> strings
- Nested dataclasses <-> nested dicts
- tuple fields (remappings) <-> JSON arrays
- Any-typed fields (parameter values) passed through as-is
- Schema version: files missing schema_version are treated as v1 with a warning
"""

from __future__ import annotations

import json
import warnings
from dataclasses import asdict, fields, is_dataclass
from pathlib import Path, PurePosixPath
from typing import Any, get_args, get_origin, get_type_hints

from colcon_lf.graph.models import SCHEMA_VERSION

from colcon_lf.comm_analysis.models import (
    ActionConnection,
    ServiceConnection,
    SystemCommunication,
    TopicConnection,
)
from colcon_lf.features.diagnostics import Diagnostic
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import (
    LaunchArgument,
    LaunchContextState,
    LaunchFileInfo,
    LaunchGraph,
    LaunchGroup,
    LaunchNodeAction,
    LaunchProcessAction,
)
from colcon_lf.node_analysis.models import (
    ActionClientInfo,
    ActionServerInfo,
    CallbackGroupInfo,
    CallbackInfo,
    CallbackParameter,
    CMakeInfo,
    LibraryTarget,
    NodeInfo,
    ParameterInfo,
    PublisherInfo,
    ServiceClientInfo,
    ServiceServerInfo,
    SubscriberInfo,
    TimerInfo,
)


# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------

class _Encoder(json.JSONEncoder):
    """JSON encoder that converts Path objects to strings."""

    def default(self, o: Any) -> Any:
        if isinstance(o, (Path, PurePosixPath)):
            return str(o)
        # Handle ROS2 launch substitution objects (TextSubstitution, etc.)
        # that leak into parameter dicts without being resolved.
        if hasattr(o, "text"):
            return str(o.text)
        if hasattr(o, "perform"):
            try:
                return str(o.perform(None))
            except Exception:
                return str(o)
        # Last resort: stringify unknown objects
        try:
            return str(o)
        except Exception:
            return repr(o)


def serialize(obj: Any) -> str:
    """Serialize a dataclass instance to a JSON string."""
    data = _sanitize_keys(asdict(obj))
    return json.dumps(data, cls=_Encoder, indent=2)


def _sanitize_keys(obj: Any) -> Any:
    """Recursively convert non-string dict keys to strings.

    ROS2 launch files can produce parameter dicts with tuple keys
    (e.g. from substitution objects). JSON requires string keys.
    """
    if isinstance(obj, dict):
        return {str(k): _sanitize_keys(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize_keys(item) for item in obj]
    return obj


def serialize_to_dict(obj: Any) -> dict:
    """Serialize a dataclass instance to a dict (with Paths as strings)."""
    return json.loads(serialize(obj))


# ---------------------------------------------------------------------------
# Deserialization
# ---------------------------------------------------------------------------

# Registry mapping dataclass types to themselves, used for type-aware
# reconstruction from dicts.
_DATACLASS_REGISTRY: dict[str, type] = {
    cls.__name__: cls
    for cls in [
        # Node layer
        CallbackParameter,
        CallbackInfo,
        CallbackGroupInfo,
        TimerInfo,
        SubscriberInfo,
        ServiceServerInfo,
        ActionServerInfo,
        PublisherInfo,
        ServiceClientInfo,
        ActionClientInfo,
        ParameterInfo,
        NodeInfo,
        CMakeInfo,
        LibraryTarget,
        # Launch layer
        LaunchArgument,
        LaunchNodeAction,
        LaunchProcessAction,
        LaunchGroup,
        LaunchFileInfo,
        LaunchContextState,
        LaunchGraph,
        # Comm layer
        TopicConnection,
        ServiceConnection,
        ActionConnection,
        SystemCommunication,
        # Features layer (hazard-detection diagnostics, schema v3+)
        Diagnostic,
        # Graph layer
        SystemGraph,
    ]
}


def _from_dict(data: dict[str, Any], cls: type) -> Any:
    """Recursively reconstruct a dataclass instance from a dict.

    Handles:
    - Path fields: str -> Path
    - Nested dataclasses: dict -> dataclass
    - list[tuple[str,str]]: list[list] -> list[tuple]
    - Optional fields: None passed through
    """
    if not is_dataclass(cls):
        return data

    hints = get_type_hints(cls)
    kwargs: dict[str, Any] = {}

    for f in fields(cls):
        if f.name not in data:
            continue
        value = data[f.name]
        hint = hints.get(f.name, Any)
        kwargs[f.name] = _convert_value(value, hint)

    return cls(**kwargs)


def _convert_value(value: Any, hint: Any) -> Any:
    """Convert a JSON-parsed value to the expected Python type."""
    if value is None:
        return None

    origin = get_origin(hint)
    args = get_args(hint)

    # Handle Path
    if hint is Path:
        return Path(value) if value is not None else None

    # Handle X | None (Union with None)
    if _is_optional(hint):
        inner = _unwrap_optional(hint)
        return _convert_value(value, inner)

    # Handle list[X]
    if origin is list and args:
        inner = args[0]
        # list[tuple[str, str]] -> convert inner lists to tuples
        if get_origin(inner) is tuple:
            return [tuple(item) for item in value]
        # list[SomeDataclass]
        if is_dataclass(inner):
            return [_from_dict(item, inner) for item in value]
        # list[Path]
        if inner is Path:
            return [Path(item) for item in value]
        return value

    # Handle dict[str, X]
    if origin is dict and args:
        key_type, val_type = args
        if is_dataclass(val_type):
            return {k: _from_dict(v, val_type) for k, v in value.items()}
        if val_type is Path:
            return {k: Path(v) for k, v in value.items()}
        return value

    # Handle nested dataclass
    if is_dataclass(hint) and isinstance(value, dict):
        return _from_dict(value, hint)

    return value


def _is_optional(hint: Any) -> bool:
    """Check if a type hint is Optional (X | None)."""
    origin = get_origin(hint)
    if origin is type(None):
        return True
    # Python 3.10+ union: X | None
    import types
    if isinstance(hint, types.UnionType):
        return type(None) in get_args(hint)
    return False


def _unwrap_optional(hint: Any) -> Any:
    """Extract the inner type from Optional[X] / X | None."""
    args = get_args(hint)
    non_none = [a for a in args if a is not type(None)]
    return non_none[0] if len(non_none) == 1 else hint


def deserialize_node_info(json_str: str) -> NodeInfo:
    """Deserialize a JSON string to a NodeInfo."""
    data = json.loads(json_str)
    return _from_dict(data, NodeInfo)


def deserialize_system_graph(json_str: str) -> SystemGraph:
    """Deserialize a JSON string to a SystemGraph."""
    data = json.loads(json_str)
    v = data.get("schema_version")
    if v is None:
        warnings.warn(
            "system.json has no schema_version — treating as v1. "
            "Re-run 'colcon lf-analyze' to regenerate with schema_version=2.",
            stacklevel=2,
        )
    elif v < SCHEMA_VERSION:
        warnings.warn(
            f"system.json has schema_version={v}, current is {SCHEMA_VERSION}. "
            "Re-run 'colcon lf-analyze' to pick up new fields.",
            stacklevel=2,
        )
    elif v > SCHEMA_VERSION:
        warnings.warn(
            f"system.json has schema_version={v}, newer than this tool's "
            f"current schema_version={SCHEMA_VERSION}. Some fields may be "
            "ignored — upgrade colcon_lf to match.",
            stacklevel=2,
        )
    return _from_dict(data, SystemGraph)


def deserialize_launch_graph(json_str: str) -> LaunchGraph:
    """Deserialize a JSON string to a LaunchGraph."""
    data = json.loads(json_str)
    return _from_dict(data, LaunchGraph)
