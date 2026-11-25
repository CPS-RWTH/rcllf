# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Launch file parser (Python, XML, and YAML).

All formats are loaded via ROS2's ``AnyLaunchDescriptionSource``, which
auto-detects the format from the file extension (``.py``, ``.xml``,
``.yaml``) and produces the same ``LaunchDescription.entities`` tree.
The tree is walked by ``_walk_entities()`` to build a :class:`LaunchGraph`.

Path resolution strategy for ``FindPackageShare``:
  1. ``<workspace>/install/<pkg>/share/<pkg>/``  (install space)
  2. ``<workspace>/src/<pkg>/``                  (source tree fallback)

The workspace root is the ``packages/`` directory (parent of both
``src/`` and ``install/``).
"""

from __future__ import annotations

import contextlib
import json
import re
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterator

from colcon_lf.launch_analysis.models import (
    LaunchArgument,
    LaunchContextState,
    LaunchFileInfo,
    LaunchGraph,
    LaunchGroup,
    LaunchNodeAction,
    LaunchProcessAction,
)
from colcon_lf.workspace import manifest_package_name


# ---------------------------------------------------------------------------
# Module-level parse result cache (Phase 7.1)
# ---------------------------------------------------------------------------

_parse_launch_cache: dict[tuple[Path, tuple[tuple[str, str], ...]], LaunchGraph] = {}

# ``name:=value`` launch arguments the analysis runs with, as the user gives
# them to ``colcon lf --launch-arg``. Set around a whole analysis by
# :func:`launch_argument_overrides` so the several parse_launch callers (the
# system graph, the launch-only path) see the same values.
_active_launch_args: dict[str, str] = {}


class LaunchArgumentError(ValueError):
    """A ``--launch-arg`` the launch file does not declare."""


@contextlib.contextmanager
def launch_argument_overrides(overrides: dict[str, str] | None) -> Iterator[None]:
    """Run an analysis with *overrides* as the command-line launch arguments."""
    global _active_launch_args
    previous = _active_launch_args
    _active_launch_args = dict(overrides or {})
    try:
        yield
    finally:
        _active_launch_args = previous


def parse_launch_arg(text: str) -> tuple[str, str]:
    """Split a ``NAME:=VALUE`` command-line argument."""
    name, sep, value = text.partition(":=")
    if not sep or not name:
        raise LaunchArgumentError(
            f"launch argument {text!r} is not of the form NAME:=VALUE")
    return name, value

# Serialises the sys.dont_write_bytecode save and restore in _parse_file. Two
# threads interleaving it can restore each other's True and leave bytecode
# writing off for the whole process. Reentrant, because an included launch
# file may be loaded while an outer load still holds it.
_BYTECODE_LOCK = threading.RLock()


def clear_parse_launch_cache() -> None:
    """Clear the module-level ``parse_launch()`` result cache."""
    _parse_launch_cache.clear()


def _join_path_parts(parts: list[str]) -> str:
    """Join path parts, appending parts that start with '/' instead of replacing.

    ``Path("/a/b", "/c/d")`` discards ``/a/b`` because the second arg is
    absolute.  This function always concatenates: ``"/a/b" + "/c/d"`` →
    ``"/a/b/c/d"``.  Relative parts are joined with ``/`` as usual.
    """
    if not parts:
        return ""
    result = parts[0]
    for p in parts[1:]:
        if p.startswith("/"):
            result = result.rstrip("/") + p
        else:
            result = result.rstrip("/") + "/" + p
    return result


# ---------------------------------------------------------------------------
# Path resolver
# ---------------------------------------------------------------------------

class _PackageResolver:
    """Resolves ``FindPackageShare`` to a filesystem path."""

    def __init__(self, workspace_root: Path) -> None:
        self.workspace_root = workspace_root
        self._install = workspace_root / "install"
        self._src = workspace_root / "src"
        # Eagerly scan source tree for nested packages (handles monorepos)
        if self._src.is_dir():
            from colcon_lf.workspace import find_package_dirs
            self._src_packages = find_package_dirs(self._src)
        else:
            self._src_packages: dict[str, Path] = {}

    def find_package_share(self, package: str) -> Path | None:
        """Return the share directory for *package*, or None."""
        # Priority 1: install space (most accurate)
        install_share = self._install / package / "share" / package
        if install_share.is_dir():
            return install_share
        # Priority 2: flat source path
        src_pkg = self._src / package
        if src_pkg.is_dir():
            return src_pkg
        # Priority 3: recursive source scan (handles nested monorepos)
        nested = self._src_packages.get(package)
        if nested is not None and nested.is_dir():
            return nested
        return None

    def resolve_path_substitution(self, subst: Any) -> Path | None:
        """Statically resolve a launch substitution to a Path."""
        try:
            from launch.substitutions import (
                PathJoinSubstitution,
                TextSubstitution,
            )
            from launch_ros.substitutions import FindPackageShare as FPS
        except ImportError:
            return None

        # In Jazzy, FPS is a subclass of PathJoinSubstitution.
        # Handle FPS *before* PathJoinSubstitution to avoid infinite recursion
        # (FPS.substitutions[0] returns a list wrapping FPS itself).
        # The package name is stored in the private `_FindPackage__package` attr
        # as a list containing a TextSubstitution.
        if isinstance(subst, FPS):
            raw_pkg = getattr(subst, "_FindPackage__package", None)
            pkg = self._get_text_from_list(raw_pkg)
            if pkg:
                return self.find_package_share(pkg)
            return None

        # PathJoinSubstitution.substitutions returns a list-of-lists in Jazzy:
        #   [[FPS(...)], [TextSubstitution('launch')], [TextSubstitution('file')]]
        # Each inner list has exactly one element — unwrap it before recursing.
        if isinstance(subst, PathJoinSubstitution):
            parts: list[str] = []
            for s in subst.substitutions:
                item = s[0] if (isinstance(s, list) and len(s) == 1) else s
                resolved = self.resolve_path_substitution(item)
                if resolved is None:
                    return None
                parts.append(str(resolved))
            return Path(_join_path_parts(parts)) if parts else None

        if isinstance(subst, TextSubstitution):
            return Path(subst.text)

        if isinstance(subst, str):
            return Path(subst)

        if isinstance(subst, Path):
            return subst

        # Bare list (e.g. the __location attribute is [PathJoinSubstitution(...)]):
        # if single-element resolve to that element; otherwise join as path parts.
        if isinstance(subst, list):
            if len(subst) == 1:
                return self.resolve_path_substitution(subst[0])
            parts = []
            for s in subst:
                r = self.resolve_path_substitution(s)
                if r is None:
                    return None
                parts.append(str(r))
            return Path(_join_path_parts(parts)) if parts else None

        return None

    def _get_text_from_list(self, subst: Any) -> str | None:
        """Extract a plain string from a TextSubstitution or a list thereof."""
        try:
            from launch.substitutions import TextSubstitution
        except ImportError:
            return None
        if isinstance(subst, str):
            return subst
        if isinstance(subst, TextSubstitution):
            return subst.text
        if isinstance(subst, list) and subst:
            return self._get_text_from_list(subst[0])
        return None

    def _get_text(self, subst: Any) -> str | None:
        try:
            from launch.substitutions import TextSubstitution
        except ImportError:
            return None
        if isinstance(subst, TextSubstitution):
            return subst.text
        if isinstance(subst, str):
            return subst
        return None


# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------

def _condition_var_name(cond: Any) -> str | None:
    """Extract the LaunchConfiguration variable name from a condition object.

    Returns the variable name string, or *None* if it cannot be determined.
    """
    try:
        from launch.substitutions import LaunchConfiguration, TextSubstitution
    except ImportError:
        return None
    pred_expr = getattr(cond, "_IfCondition__predicate_expression", None)
    if isinstance(pred_expr, list) and pred_expr:
        first = pred_expr[0]
        if isinstance(first, LaunchConfiguration):
            vname = first.variable_name
            if isinstance(vname, list) and vname:
                if isinstance(vname[0], TextSubstitution):
                    return vname[0].text
            elif isinstance(vname, str):
                return vname
    return None


def _resolve_condition(entity: Any) -> str | None:
    """Return a string description of the condition on an entity."""
    try:
        from launch.conditions import IfCondition, UnlessCondition
    except ImportError:
        return None

    cond = getattr(entity, "condition", None)
    if cond is None:
        return None

    var = _condition_var_name(cond) or "..."

    # Check UnlessCondition FIRST — it's a subclass of IfCondition in some distros
    if isinstance(cond, UnlessCondition):
        return f"UnlessCondition({var})"
    if isinstance(cond, IfCondition):
        return f"IfCondition({var})"
    return str(cond)


def _subst_to_str(subst: Any, _depth: int = 0) -> str | None:
    """Best-effort conversion of a launch substitution to a string.

    The *_depth* guard prevents infinite recursion on self-referential
    substitution objects (e.g. FindPackageShare.substitutions contains
    itself in both Jazzy and Humble).
    """
    if _depth > 20 or subst is None:
        return None
    if isinstance(subst, str):
        return subst
    # List of substitutions (e.g. ComposableNode.node_plugin)
    if isinstance(subst, list):
        parts = [_subst_to_str(s, _depth + 1) for s in subst]
        if all(p is not None for p in parts):
            return "".join(parts)  # type: ignore[arg-type]
        return None
    try:
        from launch.substitutions import LaunchConfiguration, TextSubstitution
        if isinstance(subst, TextSubstitution):
            return subst.text
        if isinstance(subst, LaunchConfiguration):
            # variable_name may be a list[TextSubstitution] in Jazzy
            vname = subst.variable_name
            if isinstance(vname, list):
                text = "".join(
                    v.text if isinstance(v, TextSubstitution) else str(v)
                    for v in vname
                )
            else:
                text = str(vname)
            default = getattr(subst, "_LaunchConfiguration__default", None)
            default_text = _subst_to_str(default, _depth + 1)
            if default_text is not None and not set("')") & set(default_text):
                # launch's frontend form ``$(var NAME DEFAULT)``: the default
                # applies while NAME is unset (launch_configuration.py:87-102).
                if not default_text or any(c.isspace() for c in default_text):
                    default_text = f"'{default_text}'"
                return f"$(var {text} {default_text})"
            return f"$(var {text})"
    except ImportError:
        pass
    if hasattr(subst, "substitutions"):
        parts = [_subst_to_str(s, _depth + 1) for s in subst.substitutions]
        if all(p is not None for p in parts):
            return "".join(parts)  # type: ignore[arg-type]
    return None


def _join_namespaces(parts: list[str]) -> str:
    """Join namespace parts into a /a/b/c string."""
    cleaned = [p.strip("/") for p in parts if p and p.strip("/")]
    return ("/" + "/".join(cleaned)) if cleaned else ""


# ---------------------------------------------------------------------------
# Parameter key/value resolution helpers
# ---------------------------------------------------------------------------

def _resolve_param_key(key: Any) -> str:
    """Resolve a parameter dict key to a plain string.

    Nav2 launch files store inline parameter keys as tuples of
    TextSubstitution objects, e.g. ``(TextSubstitution(text='use_sim_time'),)``.
    This function converts any such key to its string representation.
    """
    if isinstance(key, str):
        return key
    if isinstance(key, tuple):
        parts = []
        for item in key:
            s = _subst_to_str(item)
            parts.append(s if s is not None else str(item))
        return "".join(parts)
    s = _subst_to_str(key)
    return s if s is not None else str(key)


def _evaluate_parameter_value(
    value: Any,
    config_scope: list[tuple[str, str | None]],
    errors: list[str] | None,
    where: str,
) -> Any:
    """Evaluate a launch_ros ``ParameterValue`` as ``ParameterValue.evaluate`` does.

    The substitution is performed with the launch configurations in effect
    and coerced to ``value_type`` (YAML rules when there is none), which is
    what ``<param value=...>`` in XML and ``ParameterValue(..., value_type=)``
    in Python produce (launch_ros/parameter_descriptions.py:80-90).
    """
    from launch.utilities.type_utils import perform_typed_substitution
    try:
        return perform_typed_substitution(
            _launch_context(config_scope), value.value, value.value_type)
    except Exception as exc:  # SubstitutionFailure, TypeError, ValueError
        _add_error(errors, f"{where}: parameter value not evaluated: {exc}")
        return f"<unresolved {type(value).__name__}>"


def _resolve_param_value(
    value: Any,
    config_scope: list[tuple[str, str | None]] | None = None,
    errors: list[str] | None = None,
    where: str = "",
) -> Any:
    """Best-effort resolution of a parameter value.

    Lists whose entries are themselves tuples/substitutions (Nav2 node_names
    lists) are recursively resolved to plain strings.  A ``ParameterValue``
    is evaluated with *config_scope*.  Other values are returned as-is.
    """
    try:
        from launch_ros.parameter_descriptions import ParameterValue
    except ImportError:
        ParameterValue = None  # type: ignore[assignment,misc]
    if ParameterValue is not None and isinstance(value, ParameterValue):
        return _evaluate_parameter_value(value, config_scope or [], errors, where)
    if isinstance(value, list):
        resolved = []
        for item in value:
            if isinstance(item, (list, tuple)):
                inner = [_resolve_param_key(x) for x in item]
                resolved.append("".join(inner) if len(inner) > 1 else (inner[0] if inner else ""))
            else:
                s = _subst_to_str(item)
                resolved.append(s if s is not None else str(item))
        return resolved
    if isinstance(value, tuple):
        # In Jazzy, normalize_parameter_dict encodes plain string values via
        # yaml.dump() into TextSubstitution tuples.  Decode them back so that
        # e.g. (TextSubstitution("opaque\n...\n"),) → "opaque".
        raw = _resolve_param_key(value)
        if isinstance(raw, str):
            try:
                import yaml as _yaml
                return _yaml.safe_load(raw)
            except Exception:
                return raw
        return raw
    s = _subst_to_str(value)
    if s is not None:
        return s
    return value


# One pass over ``$(var NAME)`` or ``$(var NAME DEFAULT)``: a substituted
# value is not scanned again.
_VAR_RE = re.compile(r"\$\(var ([^)\s]+)(?: ('[^']*'|[^)\s']+))?\)")


def _perform_config(subst: Any, config_scope: list[tuple[str, str | None]]) -> str | None:
    """Evaluate *subst* as launch's ``perform_substitutions`` does at this point.

    A ``LaunchConfiguration`` takes its current value from *config_scope*
    (the last entry wins); an unset one takes its ``default`` when it has one
    (launch_configuration.py:87-102) and otherwise stays ``$(var NAME)``.
    Other substitutions go through :func:`_subst_to_str`, so literal text that
    looks like ``$(var NAME)`` stays text, as in launch.
    """
    try:
        from launch.substitutions import LaunchConfiguration
    except ImportError:
        return _subst_to_str(subst)
    if isinstance(subst, (list, tuple)):
        parts = [_perform_config(s, config_scope) for s in subst]
        if all(p is not None for p in parts):
            return "".join(parts)  # type: ignore[arg-type]
        return None
    if isinstance(subst, LaunchConfiguration):
        name = _perform_config(subst.variable_name, config_scope)
        if name is None:
            return None
        values = dict(config_scope)
        if name in values:
            return values[name]
        default = getattr(subst, "_LaunchConfiguration__default", None)
        if default is not None:
            return _perform_config(default, config_scope)
        return f"$(var {name})"
    return _subst_to_str(subst)


def _perform_value(subst: Any, config_scope: list[tuple[str, str | None]]) -> str | None:
    """*subst* performed against *config_scope*, or None when not exactly known.

    Only text and launch configurations with a value are performed exactly;
    :func:`_subst_to_str` renders other substitutions (``$(env X)``, joined
    path parts) as text launch would not produce, and an unset configuration
    without a default makes launch fail.
    """
    try:
        from launch.substitutions import LaunchConfiguration, TextSubstitution
    except ImportError:
        return None
    parts = list(subst) if isinstance(subst, (list, tuple)) else [subst]
    for part in parts:
        if isinstance(part, str) or type(part) is TextSubstitution:
            continue
        if type(part) is not LaunchConfiguration:
            return None
        name = _perform_value(part.variable_name, config_scope)
        default = getattr(part, "_LaunchConfiguration__default", None)
        if name is None or (name not in dict(config_scope) and (
                default is None or _perform_value(default, config_scope) is None)):
            return None
    return _perform_config(subst, config_scope)


# launch_ros keeps SetRemap rules and SetParameter values in the launch
# configurations ``ros_remaps`` and ``global_params`` (set_remap.py:91-93,
# set_parameter.py:90-93), so they are scoped as every other configuration is
# (GroupAction copy, forwarding=False reset, TimerAction copy). The walk
# carries them in ``config_scope`` under these names, as JSON text; the NUL
# prefix keeps them out of reach of ``$(var NAME)``.
_REMAP_SCOPE = "\0ros_remaps"
_PARAM_SCOPE = "\0global_params"


def _scope_items(config_scope: list[tuple[str, str | None]], key: str) -> list[Any]:
    """The JSON items of a pseudo configuration, in launch order."""
    return [json.loads(v) for k, v in config_scope if k == key and v is not None]


def _add_error(errors: list[str] | None, message: str) -> None:
    if errors is not None and message not in errors:
        errors.append(message)


def _launch_context(config_scope: list[tuple[str, str | None]]) -> Any:
    """A ``LaunchContext`` holding the configurations known in *config_scope*."""
    from launch import LaunchContext
    context = LaunchContext()
    for name, value in config_scope:
        if value is not None and not name.startswith("\0"):
            context.launch_configurations[name] = value
    return context


def _resolve_text(
    raw: Any,
    config_scope: list[tuple[str, str | None]],
    errors: list[str] | None,
    where: str,
) -> str | None:
    """Text of a node name, namespace or remap side, as launch performs it.

    The launch configurations in effect at the node are applied. What cannot
    be evaluated stays visible and is recorded in *errors* with the
    argument's name; the generated system must never receive ``$(var X)``.
    """
    if raw is None:
        return None
    text = _perform_value(raw, config_scope)
    if text is None:
        text = _subst_to_str(raw) if not isinstance(raw, tuple) else _resolve_param_key(raw)
    if text is not None and "$(" in text:
        match = _VAR_RE.search(text)
        if match:
            _add_error(errors, f"{where} '{text}': launch argument '{match.group(1)}' "
                               f"has no value here")
        else:
            _add_error(errors, f"{where} '{text}': substitution not evaluated")
    return text


def _scope_remaps(config_scope: list[tuple[str, str | None]]) -> list[tuple[str, str]]:
    """SetRemap rules in effect, in launch order (they precede a node's own)."""
    return [(src, dst) for src, dst in _scope_items(config_scope, _REMAP_SCOPE)]


# Substitutions whose result depends only on their operands, so a condition
# built from them evaluates the same at analysis time as at launch time.
# EnvironmentVariable, Command, FindPackageShare, ... depend on the launch
# host and are not evaluated.
_PURE_SUBSTITUTIONS = (
    "TextSubstitution", "LaunchConfiguration", "PythonExpression",
    "EqualsSubstitution", "NotEqualsSubstitution", "NotSubstitution",
    "AndSubstitution", "OrSubstitution", "AnySubstitution", "AllSubstitution",
    "IfElseSubstitution",
)


def _python_expression_is_pure(
    subst: Any, config_scope: list[tuple[str, str | None]],
) -> bool:
    """Whether PythonExpression *subst* evaluates without reading the host.

    launch ``eval``s the performed expression (python_expression.py), so
    ``__import__('os').getenv('X')`` is a valid one. Only literals,
    comparisons (including ``in``), ``and``/``or``/``not``, and names that
    are exactly a value substituted into the expression are accepted.
    """
    import ast
    parts = list(getattr(subst, "expression", None) or ())
    texts = [_perform_config(part, config_scope) for part in parts]
    if any(text is None for text in texts):
        return False
    substituted = {text for part, text in zip(parts, texts)
                   if type(part).__name__ != "TextSubstitution"}
    try:
        tree = ast.parse("".join(texts).strip(), mode="eval")  # type: ignore[arg-type]
    except SyntaxError:
        return False
    allowed = (ast.Expression, ast.Constant, ast.Compare, ast.BoolOp, ast.And, ast.Or,
               ast.Eq, ast.NotEq, ast.Lt, ast.LtE, ast.Gt, ast.GtE, ast.In, ast.NotIn,
               ast.Is, ast.IsNot, ast.Tuple, ast.List, ast.Load)
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            if node.id not in substituted:
                return False
        elif isinstance(node, ast.UnaryOp):
            if not isinstance(node.op, (ast.Not, ast.USub, ast.UAdd)):
                return False
        elif isinstance(node, (ast.Not, ast.USub, ast.UAdd)):
            continue
        elif not isinstance(node, allowed):
            return False
    return True


def _describe_condition(cond: Any) -> str:
    """*cond* as launch code reads, e.g. ``IfCondition($(var use_x))``."""
    kind = type(cond).__name__
    expression = getattr(cond, "_IfCondition__predicate_expression", None)
    if expression is not None:
        return f"{kind}({_subst_to_str(expression) or '...'})"
    name = getattr(cond, "_LaunchConfigurationEquals__launch_configuration_name", None)
    if name is not None:
        expected = getattr(cond, "_LaunchConfigurationEquals__expected_value", None)
        shown = "None" if expected is None else repr(_subst_to_str(expected) or "...")
        return f"{kind}({name!r}, {shown})"
    return kind


def _scope_condition(
    entity: Any, config_scope: list[tuple[str, str | None]],
) -> tuple[bool | None, str]:
    """Evaluate *entity*'s condition against *config_scope* as launch would.

    launch runs an action only when its condition is None or evaluates true
    (``launch/action.py:105-109``). IfCondition and UnlessCondition expand
    their expression and accept ``true``/``1``/``false``/``0`` after
    ``strip().lower()``, raising otherwise
    (``conditions/evaluate_condition_expression_impl.py``);
    LaunchConfigurationEquals compares the configuration with the expanded
    value, an unset configuration matching only ``None``
    (``conditions/launch_configuration_equals.py``). The condition is
    evaluated by launch itself, in a context holding *config_scope*.

    Returns ``(result, "")``, or ``(None, reason)`` when the result depends on
    something the analyzer does not know: a configuration without a value, an
    unknown one (``None`` in *config_scope*), a substitution outside
    ``_PURE_SUBSTITUTIONS``, or a condition type launch does not ship.
    """
    cond = getattr(entity, "condition", None)
    if cond is None:
        return True, ""
    try:
        from launch import LaunchContext
        from launch.conditions import IfCondition, LaunchConfigurationEquals
        from launch.substitution import Substitution
        from launch.substitutions import LaunchConfiguration
    except ImportError as exc:
        return None, f"launch not available: {exc}"
    if type(cond).__module__.split(".")[0] != "launch" or \
            not isinstance(cond, (IfCondition, LaunchConfigurationEquals)):
        return None, f"condition type {type(cond).__name__} is not evaluated"

    values = dict(config_scope)
    # A value that still holds $(var ...) was not resolved when it was set.
    known = {k: v for k, v in values.items()
             if v is not None and "$(var " not in v}
    names: list[str] = []
    name = getattr(cond, "_LaunchConfigurationEquals__launch_configuration_name", None)
    if name is not None:
        names.append(str(name))
    expressions: list[Any] = []
    stack: list[Any] = [vars(cond)]
    seen: set[int] = set()
    while stack:
        item = stack.pop()
        if id(item) in seen:
            continue
        seen.add(id(item))
        if isinstance(item, dict):
            stack.extend(item.values())
        elif isinstance(item, (list, tuple)):
            stack.extend(item)
        elif isinstance(item, Substitution):
            if type(item).__name__ not in _PURE_SUBSTITUTIONS or \
                    not type(item).__module__.startswith("launch."):
                return None, f"it uses {type(item).__name__}, which is not evaluated"
            if type(item).__name__ == "PythonExpression":
                expressions.append(item)
            if isinstance(item, LaunchConfiguration):
                var = _perform_config(item.variable_name, config_scope)
                if var is None:
                    return None, "a launch configuration name is not evaluated"
                names.append(var)
            stack.append(vars(item))
    for var in names:
        if var in values and var not in known:
            return None, f"launch configuration '{var}' has no known value here"
    # launch eval()s a PythonExpression, which can read the host.
    if not all(_python_expression_is_pure(e, config_scope) for e in expressions):
        return None, ("its PythonExpression is not only literals, comparisons "
                      "and boolean operators")

    context = LaunchContext()
    context.launch_configurations.update(known)
    try:
        return bool(cond.evaluate(context)), ""
    except Exception as exc:  # SubstitutionFailure, InvalidConditionExpressionError
        return None, str(exc)


def _apply_condition(
    entity: Any,
    config_scope: list[tuple[str, str | None]],
    file_info: LaunchFileInfo,
    action: str,
    names: list[str],
) -> bool | None:
    """Evaluate *entity*'s condition for a change of the configurations *names*.

    An undecidable condition is not guessed: it is recorded in
    ``file_info.errors`` and each of *names* becomes unknown (``None``) in
    *config_scope*, so a parameter bound to it is reported, not filled.
    """
    result, reason = _scope_condition(entity, config_scope)
    if result is None:
        message = (f"{action}: condition {_describe_condition(entity.condition)} "
                   f"not evaluated ({reason})")
        if names:
            listed = ", ".join(f"'{n}'" for n in names)
            message += f"; {listed} {'is' if len(names) == 1 else 'are'} unknown after it"
        if message not in file_info.errors:
            file_info.errors.append(message)
        config_scope.extend((n, None) for n in names)
    return result


def _undecided_inside(
    entity: Any,
    kind: str,
    result: bool | None,
    reason: str,
    undecided: str | None,
) -> str | None:
    """The undecidable condition the actions inside *entity* run under.

    *undecided* is the one of an enclosing action, if any; otherwise an
    undecidable condition of *entity* itself (``result`` None).
    """
    if undecided is not None or result is not None:
        return undecided
    return (f"enclosing {kind} condition {_describe_condition(entity.condition)} "
            f"not evaluated ({reason})")


def _starts(
    entity: Any,
    config_scope: list[tuple[str, str | None]],
    file_info: LaunchFileInfo,
    what: str,
    undecided: str | None,
) -> bool | None:
    """Whether launch starts the node *entity*.

    launch skips an action whose condition is false (``launch/action.py:105-109``),
    so a false one drops the node. An undecidable condition, the node's own or
    the *undecided* one of an enclosing GroupAction, IncludeLaunchDescription
    or TimerAction, is not guessed: the node is kept, its ``condition_result``
    is None and an error names the node and the condition.
    """
    result, reason = _scope_condition(entity, config_scope)
    if result is False:
        return False
    if result is None:
        cause = (f"condition {_describe_condition(entity.condition)} "
                 f"not evaluated ({reason})")
    elif undecided is not None:
        cause = undecided
    else:
        return True
    message = f"{what}: {cause}; the generated system assumes it starts"
    if message not in file_info.errors:
        file_info.errors.append(message)
    return None


def _names_set(before: list[tuple[str, str | None]],
               after: list[tuple[str, str | None]]) -> list[str]:
    """Names *after* (a copy of *before*, then walked) set beyond *before*."""
    return list(dict.fromkeys(
        name for name, _ in after[len(before):] if not name.startswith("\0")))


def _launch_ros_sequence_ok(sequence: list[Any]) -> bool:
    """Mirror launch_ros ``check_sequence_type_is_allowed``.

    Non-empty, first item bool/int/float/str, every item an instance of the
    first item's type (launch_ros/utilities/evaluate_parameters.py).
    """
    if not sequence:
        return False
    subtype = type(sequence[0])
    if subtype not in (bool, int, float, str):
        return False
    return all(isinstance(item, subtype) for item in sequence)


def _bind_value(
    value: Any,
    where: str,
    config_scope: list[tuple[str, str | None]],
    errors: list[str] | None,
) -> Any:
    """Bind ``$(var X)`` in one parameter value as launch_ros types the result.

    A value launch_ros rejects, or an argument without any value, stays text
    and is recorded in *errors* under *where*.
    """
    import yaml

    values = dict(config_scope)

    def error(message: str) -> None:
        _add_error(errors, message)

    if not isinstance(value, str) or "$(var " not in value:
        return value
    missing: list[str] = []
    unknown: list[str] = []

    def substitute(match: re.Match[str]) -> str:
        name, default = match.group(1), match.group(2)
        if name in values:
            if values[name] is None:
                unknown.append(name)
                return match.group(0)
            return str(values[name])
        if default is not None:
            # LaunchConfiguration(default=...) while NAME is unset.
            return default[1:-1] if default.startswith("'") else default
        missing.append(name)
        return match.group(0)

    text = _VAR_RE.sub(substitute, value)
    if unknown:
        error(f"{where}: launch argument '{unknown[0]}' has no known value here")
        return text
    if missing:
        error(f"{where}: launch argument '{missing[0]}' has no value")
        return text
    try:
        # launch_ros parses an empty result as '' rather than None.
        loaded = yaml.safe_load(text if text else "''")
    except yaml.YAMLError:
        error(f"{where}: value {text!r} is not valid YAML; launch_ros rejects it")
        return text
    if type(loaded) in (bool, int, float, str):
        return loaded
    if isinstance(loaded, list) and _launch_ros_sequence_ok(loaded):
        return loaded
    error(f"{where}: value {text!r} parses as {type(loaded).__name__}; "
          f"launch_ros rejects it")
    return text


def _is_primitive(value: Any) -> bool:
    if isinstance(value, (bool, int, float, str, bytes)):
        return True
    if isinstance(value, (list, tuple)):
        return all(_is_primitive(v) for v in value)
    if isinstance(value, dict):
        return all(_is_primitive(v) for v in value.values())
    return False


def _check_primitive(name: str, value: Any, where: str, errors: list[str] | None) -> None:
    """A parameter reaching a node must be bool, int, float, str or a list of them.

    Anything else is a launch object or substitution the analysis did not
    evaluate; its repr would reach the node as a value it rejects.
    """
    if not _is_primitive(value):
        _add_error(errors, f"{where}: value of type {type(value).__name__} was not "
                           f"evaluated to a bool, int, float, str or list of them")


def _bind_launch_args(
    node_action: LaunchNodeAction,
    file_info: LaunchFileInfo,
    config_scope: list[tuple[str, str | None]],
) -> None:
    """Replace ``$(var X)`` in *node_action*'s inline parameters by X's value.

    ``launch_ros`` performs a substitution-valued parameter and YAML-parses
    the text, so ``{'timer_period_ms': LaunchConfiguration('timer_period_ms')}``
    with default ``'500'`` reaches the node as the integer 500. Left as
    ``$(var ...)`` the generated system passes a string and the node's
    ``declare_parameter<int64_t>`` throws.

    X's value is the launch configuration in effect at the node: the last
    entry for X in *config_scope*, which the walk keeps like launch's
    ``context.launch_configurations`` (declared defaults, include arguments,
    SetLaunchConfiguration, GroupAction scoping; see :func:`_walk_entities`).
    The analyzer has no command-line input, so declared defaults stand for
    launch-time values. The result is typed as launch_ros
    ``evaluate_parameter_dict`` does; a value launch_ros rejects, or an
    argument without any value, stays text and is recorded in
    ``file_info.errors``. The same configuration evaluates the parameter files
    the static path resolver left open (``_evaluate_deferred_files``).
    """
    values = dict(config_scope)
    node_label = node_action.name or node_action.executable

    def error(message: str) -> None:
        if message not in file_info.errors:
            file_info.errors.append(message)

    _evaluate_deferred_files(
        node_action, {k: v for k, v in values.items() if v is not None}, node_label, error)

    def bind(param: str, value: Any) -> Any:
        where = f"node '{node_label}' parameter '{param}'"
        bound = _bind_value(value, where, config_scope, file_info.errors)
        _check_primitive(param, bound, where, file_info.errors)
        return bound

    node_action.parameters = {
        k: bind(k, v) for k, v in node_action.parameters.items()}
    for source in node_action.parameter_sources:
        if "params" in source:
            source["params"] = {k: bind(k, v) for k, v in source["params"].items()}


# A ``parameter_sources`` entry the static path resolver could not resolve,
# held as ``(ParameterFile, resolver)`` until ``_bind_launch_args`` evaluates it
# with the launch configuration in effect.
_DEFERRED_FILE = "_deferred_file"


def _evaluate_deferred_files(
    node_action: LaunchNodeAction,
    values: dict[str, Any],
    node_label: str,
    error: Any,
) -> None:
    """Replace each deferred parameter file of *node_action* by its evaluation.

    The file keeps its place in launch order in ``parameter_sources`` and
    ``parameter_files``. A file launch cannot evaluate is dropped and recorded
    through *error*; ``ros2 launch`` would fail on it.
    """
    sources: list[dict[str, Any]] = []
    files = list(node_action.parameter_files)
    for source in node_action.parameter_sources:
        if _DEFERRED_FILE not in source:
            sources.append(source)
            continue
        entry, resolver = source[_DEFERRED_FILE]
        try:
            path = _evaluate_param_file(entry, values, resolver.workspace_root)
        except Exception as exc:  # launch raises SubstitutionFailure, OSError, ...
            error(f"node '{node_label}': parameter file "
                  f"{_describe_param_file(entry)} not evaluated, so its "
                  f"parameters are missing from the analysis: {exc}")
            continue
        files.insert(sum(1 for s in sources if "file" in s), path)
        sources.append({"file": _absolute_param_file(path)})
    node_action.parameter_sources = sources
    node_action.parameter_files = files


def _describe_param_file(entry: Any) -> str:
    """*entry* as launch code reads, e.g. ``ParameterFile(RewrittenYaml(...))``.

    ``RewrittenYaml.describe`` returns ``''``, so a substitution without a
    description is named by its class and its ``name`` (the source file).
    """
    def describe(value: Any) -> str:
        if isinstance(value, (list, tuple)):
            return " + ".join(describe(v) for v in value) or "''"
        if not hasattr(value, "perform"):
            return repr(str(value))
        text = value.describe() if hasattr(value, "describe") else ""
        if text:
            return text
        source = getattr(value, "name", None)
        inner = f"source_file={describe(source)}" if source is not None else ""
        return f"{type(value).__name__}({inner})"

    param_file = getattr(entry, "_ParameterFile__param_file", entry)
    allow_substs = getattr(entry, "_ParameterFile__allow_substs", False)
    substs = "" if allow_substs is False else f", allow_substs={describe(allow_substs)}"
    return f"ParameterFile({describe(param_file)}{substs})"


def _evaluate_param_file(entry: Any, values: dict[str, Any], workspace_root: Path) -> Path:
    """The parameter file ``launch_ros`` passes for *entry*.

    *entry* is a ``ParameterFile`` whose path is a substitution, such as Nav2's
    ``ParameterFile(RewrittenYaml(source_file=..., root_key=...,
    param_rewrites=..., convert_types=True), allow_substs=True)``. It is
    evaluated as ``ParameterFile.evaluate`` does, in a launch context holding
    *values* as launch configurations, so launch's own substitutions (and
    ``RewrittenYaml`` itself) compute the file. ``evaluate`` is not called: it
    caches its result on the object, which Nav2 shares between nodes and
    includes, and deletes its temporary file when the object is collected.

    A file the substitutions wrote (``RewrittenYaml`` and ``ReplaceString``
    write temporary files) or rewrote (``allow_substs``) is stored by
    ``_store_launch_params``; a plain path is returned as is.
    """
    import tempfile

    import yaml
    from launch import LaunchContext
    from launch.frontend.parse_substitution import parse_substitution
    from launch.utilities import perform_substitutions
    from launch.utilities.type_utils import perform_typed_substitution

    param_file = getattr(entry, "_ParameterFile__param_file", entry)
    allow_substs = getattr(entry, "_ParameterFile__allow_substs", False)
    context = LaunchContext()
    context.launch_configurations.update({k: str(v) for k, v in values.items()})

    if isinstance(param_file, list):
        path = Path(perform_substitutions(context, param_file))
        # A substitution outside launch/launch_ros that returns a file in the
        # temporary directory wrote that file (RewrittenYaml, ReplaceString).
        written = (
            any(not type(s).__module__.startswith(("launch.", "launch_ros."))
                for s in param_file)
            and path.parent == Path(tempfile.gettempdir())
        )
    else:
        path, written = Path(param_file), False
    substitute = perform_typed_substitution(context, allow_substs, data_type=bool)
    if not (written or substitute):
        return path

    text = path.read_text()
    if written:
        path.unlink(missing_ok=True)
    if substitute:
        text = perform_substitutions(context, parse_substitution(text))
        yaml.safe_load(text)  # launch_ros rejects a file that is not YAML
    return _store_launch_params(text, workspace_root)


def _store_launch_params(text: str, workspace_root: Path) -> Path:
    """Write an evaluated parameter file where the generated system finds it.

    ``ros2 launch`` keeps it in a temporary file for the run; the analyzer
    keeps it under ``<workspace>/lf-analysis/launch-params/``, named by its
    content so a re-analysis reuses the same path, and falls back to the
    temporary directory when the workspace is not writable.
    """
    import hashlib
    import tempfile

    name = f"launch_params_{hashlib.sha256(text.encode()).hexdigest()[:16]}.yaml"
    last: OSError | None = None
    for directory in (Path(workspace_root) / "lf-analysis" / "launch-params",
                      Path(tempfile.gettempdir()) / "rcllf-launch-params"):
        target = directory / name
        try:
            directory.mkdir(parents=True, exist_ok=True)
            if not target.is_file() or target.read_text() != text:
                target.write_text(text)
            return target
        except OSError as exc:
            last = exc
    raise OSError(f"cannot store evaluated parameter file {name}: {last}")


# ---------------------------------------------------------------------------
# Node extractor
# ---------------------------------------------------------------------------

def _absolute_param_file(path: Path) -> str:
    """A launch parameter file as an absolute path.

    ``launch_ros`` opens a relative path against the working directory of
    ``ros2 launch``, not against the launch file. The analyzer runs from that
    directory, which is also where it loads the file itself, so the path is
    anchored there and the generated system finds the file from any directory.
    """
    return str(path if path.is_absolute() else Path.cwd() / path)


def _global_parameter_sources(
    config_scope: list[tuple[str, str | None]],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """SetParameter values in effect, as launch_ros passes them to a node.

    launch_ros expands the global parameters before the node's own, so its
    own parameters override them (node.py:419-436). Each is a ``-p
    name:=value`` rule, which rcl files under the ``/**`` key. Returns the
    merged values and the ``parameter_sources`` entries to put in front.
    """
    merged: dict[str, Any] = {}
    sources: list[dict[str, Any]] = []
    for name, value in _scope_items(config_scope, _PARAM_SCOPE):
        merged[name] = value
        sources.append({"params": {name: value}, "key": "/**"})
    return merged, sources


def _extract_node(
    entity: Any,
    resolver: _PackageResolver,
    ns_stack: list[str],
    config_scope: list[tuple[str, str | None]] | None = None,
    errors: list[str] | None = None,
) -> LaunchNodeAction | None:
    """Extract a LaunchNodeAction from a launch_ros Node entity.

    Name, namespace, remappings and parameters are performed with
    *config_scope*, the launch configurations in effect at the node;
    SetRemap rules and SetParameter values of the enclosing scope apply
    (see ``_REMAP_SCOPE``). Problems go to *errors*.
    """
    if config_scope is None:
        config_scope = []
    try:
        from launch_ros.actions import ComposableNodeContainer, Node
    except ImportError:
        return None
    # ComposableNodeContainer is a subclass of Node — skip it here so the
    # dedicated ComposableNodeContainer handler can process it instead.
    if isinstance(entity, ComposableNodeContainer):
        return None
    if not isinstance(entity, Node):
        return None

    # Use private (name-mangled) attributes — the public properties
    # raise RuntimeError before the action is executed (Jazzy behaviour).
    package = _subst_to_str(
        getattr(entity, "_Node__package", None)
    ) or ""
    executable = _subst_to_str(
        getattr(entity, "_Node__node_executable", None)
    ) or ""
    node_label = _subst_to_str(getattr(entity, "_Node__node_name", None)) or executable
    name = _resolve_text(getattr(entity, "_Node__node_name", None), config_scope,
                         errors, f"node '{node_label}' name")
    raw_ns = getattr(entity, "_Node__node_namespace", None)
    node_ns = _resolve_text(raw_ns, config_scope, errors,
                            f"node '{node_label}' namespace") or ""

    resolved_ns = _join_namespaces(
        ns_stack + ([node_ns] if node_ns else [])
    )
    # launch_ros files an inline parameters dict under the node's full name
    # only when the name and a namespace (own or pushed) are both given, and
    # under /** otherwise (Node.is_node_name_fully_specified).
    if name and (raw_ns is not None or any(ns_stack)):
        inline_key = f"{resolved_ns}/{name}"
    else:
        inline_key = "/**"

    # Scope rules (SetRemap) come before the node's own (node.py:465-474).
    remappings: list[tuple[str, str]] = _scope_remaps(config_scope)
    raw_remappings = getattr(entity, "_Node__remappings", None) or []
    for pair in raw_remappings:
        src = _resolve_text(pair[0], config_scope, errors,
                            f"node '{node_label}' remap source") or ""
        dst = _resolve_text(pair[1], config_scope, errors,
                            f"node '{node_label}' remap target") or ""
        remappings.append((src, dst))

    inline_params, param_sources = _global_parameter_sources(config_scope)
    param_files: list[Path] = []
    raw_params = getattr(entity, "_Node__parameters", None) or []
    for p in raw_params:
        if isinstance(p, dict):
            # Keys may be TextSubstitution tuples or plain strings — resolve them.
            entry: dict[str, Any] = {}
            for k, v in p.items():
                resolved_k = _resolve_param_key(k)
                resolved_v = _resolve_param_value(
                    v, config_scope, errors, f"node '{node_label}' parameter '{resolved_k}'")
                inline_params[resolved_k] = resolved_v
                entry[resolved_k] = resolved_v
            param_sources.append({"params": entry, "key": inline_key})
        else:
            # In Jazzy, parameters=[PathJoinSubstitution(...)] stores entries
            # as ParameterFile objects; the actual path substitution is in
            # `.param_file`.  Fall back to resolving the object directly if
            # it is not a ParameterFile.
            subst = getattr(p, "param_file", p)
            resolved = resolver.resolve_path_substitution(subst)
            if resolved is not None:
                param_files.append(resolved)
                param_sources.append({"file": _absolute_param_file(resolved)})
            else:
                param_sources.append({_DEFERRED_FILE: (p, resolver)})

    # Output, respawn, arguments — Node inherits from ExecuteLocal in Humble
    output_raw = (getattr(entity, "_ExecuteLocal__output", None) or
                  getattr(entity, "_Node__output", None))
    output = _subst_to_str(output_raw)
    respawn = bool(getattr(entity, "_ExecuteLocal__respawn", None) or
                   getattr(entity, "_Node__respawn", False))
    respawn_delay_raw = (getattr(entity, "_ExecuteLocal__respawn_delay", None) or
                         getattr(entity, "_Node__respawn_delay", None))
    respawn_delay = float(respawn_delay_raw) if respawn_delay_raw is not None else None
    raw_arguments = list(getattr(entity, "_Node__arguments", None) or [])
    # ros_arguments=[...] is arguments with a leading --ros-args.
    raw_ros_arguments = getattr(entity, "_Node__ros_arguments", None)
    if raw_ros_arguments:
        raw_arguments += ["--ros-args", *raw_ros_arguments]
    arguments = []
    for a in raw_arguments:
        s = _subst_to_str(a)
        # An unresolved entry stays visible, so generation refuses it.
        arguments.append(s if s is not None else "<unresolved substitution>")

    return LaunchNodeAction(
        package=package,
        executable=executable,
        name=name,
        namespace=resolved_ns or None,
        parameters=inline_params,
        parameter_files=param_files,
        parameter_sources=param_sources,
        remappings=remappings,
        condition=_resolve_condition(entity),
        output=output,
        respawn=respawn,
        respawn_delay=respawn_delay,
        arguments=arguments,
    )


# ---------------------------------------------------------------------------
# Recursive entity walker
# ---------------------------------------------------------------------------

_MAX_INCLUDE_DEPTH = 50

# Launch actions that start nothing and change nothing the analysis models,
# so ignoring them cannot make the generated system differ from the launched
# one. Every other class the walk does not handle is refused.
_HARMLESS_ACTIONS = frozenset({
    "DeclareLaunchArgument",
    "LogInfo",
    "ExecuteProcess",
    "SetEnvironmentVariable",
    "AppendEnvironmentVariable",
    "SetLaunchConfiguration",
    "SetParameter",
    "SetRemap",
    "PushRosNamespace",
    "PushROSNamespace",
})


def _extract_composable_node(
    comp: Any,
    resolver: _PackageResolver,
    ns_stack: list[str],
    container_name: str | None = None,
    config_scope: list[tuple[str, str | None]] | None = None,
    errors: list[str] | None = None,
) -> LaunchNodeAction | None:
    """Extract a LaunchNodeAction from a ComposableNode description.

    As :func:`_extract_node`: the launch configurations, SetRemap rules and
    SetParameter values in effect apply (load_composable_nodes.py:298-318).
    """
    if config_scope is None:
        config_scope = []
    try:
        from launch_ros.descriptions import ComposableNode
    except ImportError:
        return None
    if not isinstance(comp, ComposableNode):
        return None

    pkg = _subst_to_str(comp.package) or ""
    plugin = _subst_to_str(comp.node_plugin) or ""
    label = _subst_to_str(comp.node_name) or plugin
    cname = _resolve_text(comp.node_name, config_scope, errors,
                          f"composable node '{label}' name")
    comp_ns = _resolve_text(getattr(comp, "_ComposableNode__node_namespace", None) or
                            getattr(comp, "node_namespace", None), config_scope, errors,
                            f"composable node '{label}' namespace") or ""
    resolved_ns = _join_namespaces(ns_stack + ([comp_ns] if comp_ns else [])) or None

    # Extract parameters
    inline_params, param_sources = _global_parameter_sources(config_scope)
    param_files: list[Path] = []
    raw_params = getattr(comp, "_ComposableNode__parameters", None)
    if raw_params is None:
        raw_params = getattr(comp, "parameters", None)
    if raw_params is None:
        raw_params = []
    for p in raw_params:
        if isinstance(p, dict):
            entry: dict[str, Any] = {}
            for k, v in p.items():
                rk = _resolve_param_key(k)
                entry[rk] = _resolve_param_value(
                    v, config_scope, errors, f"composable node '{label}' parameter '{rk}'")
            inline_params.update(entry)
            param_sources.append({"params": entry})
        else:
            subst = getattr(p, "param_file", p)
            resolved = resolver.resolve_path_substitution(subst)
            if resolved is not None:
                param_files.append(resolved)
                param_sources.append({"file": _absolute_param_file(resolved)})
            else:
                param_sources.append({_DEFERRED_FILE: (p, resolver)})

    # Extract remappings
    remappings: list[tuple[str, str]] = _scope_remaps(config_scope)
    raw_remappings = getattr(comp, "_ComposableNode__remappings", None)
    if raw_remappings is None:
        raw_remappings = getattr(comp, "remappings", None)
    if raw_remappings is None:
        raw_remappings = []
    for pair in raw_remappings:
        if isinstance(pair, (list, tuple)) and len(pair) == 2:
            src = _resolve_text(pair[0], config_scope, errors,
                                f"composable node '{label}' remap source") or ""
            dst = _resolve_text(pair[1], config_scope, errors,
                                f"composable node '{label}' remap target") or ""
            remappings.append((src, dst))

    use_ipc: bool | None = None
    raw_extra = getattr(comp, "_ComposableNode__extra_arguments", None)
    if raw_extra is None:
        raw_extra = getattr(comp, "extra_arguments", None)
    for extra in raw_extra or []:
        if not isinstance(extra, dict):
            continue
        for key, value in extra.items():
            if _resolve_param_key(key) != "use_intra_process_comms":
                continue
            where = f"composable node '{label}' extra_arguments use_intra_process_comms"
            typed = _resolve_param_value(value, config_scope, errors, where)
            typed = _bind_value(typed, where, config_scope, errors)
            use_ipc = typed if isinstance(typed, bool) else None

    return LaunchNodeAction(
        package=pkg,
        executable=plugin,
        name=cname,
        namespace=resolved_ns,
        parameters=inline_params,
        parameter_files=param_files,
        parameter_sources=param_sources,
        remappings=remappings,
        is_component=True,
        composable_container=container_name,
        use_intra_process_comms=use_ipc,
    )


def _walk_entities(
    entities: list[Any],
    resolver: _PackageResolver,
    ns_stack: list[str],
    file_info: LaunchFileInfo,
    all_nodes: list[LaunchNodeAction],
    visited: set[Path],
    depth: int = 0,
    config_scope: list[tuple[str, str | None]] | None = None,
    undecided: str | None = None,
) -> None:
    """Walk launch entities, populating *file_info* and *all_nodes*.

    *config_scope* models launch's ``context.launch_configurations`` as
    (name, value) pairs in launch order, the last pair for a name winning; it
    binds ``$(var X)`` in node parameters. Values are evaluated when they are
    set, as launch does. Scoping follows launch in the Jazzy image
    (``launch/actions/``): a declared default applies only to an unset name
    (``declare_launch_argument.py:205-215``); an include adds its arguments
    to the current scope and does not push one, so sets inside the included
    file stay in effect afterwards (``include_launch_description.py:238-243``);
    a scoped GroupAction works on a copy, or with ``forwarding=False`` on
    only its evaluated ``launch_configurations``, and an unscoped one on the
    parent's list (``group_action.py:114-141``,
    ``reset_launch_configurations.py:84-109``). Unlike
    ``file_info.set_configurations``, which collects every set for reporting.
    ``forwarding=False`` also resets the namespace stack, since launch_ros
    keeps the namespace in the ``ros_namespace`` configuration.
    A conditional SetLaunchConfiguration or DeclareLaunchArgument changes the
    scope only when its condition holds (``_scope_condition``); an
    undecidable one makes the name unknown (``None``) and records an error.
    A conditional GroupAction or IncludeLaunchDescription changes the scope
    only when its condition holds; an undecidable one makes every name it
    would set unknown.
    A node runs only when its own condition and every enclosing one hold, as
    in launch: a false one drops it, together with everything
    inside a false GroupAction, IncludeLaunchDescription, TimerAction or
    container. An undecidable one keeps the node with ``condition_result``
    None and an error (``_starts``); *undecided* names the undecidable
    condition of an enclosing action.
    """
    if config_scope is None:
        config_scope = []
    try:
        from launch.actions import (
            DeclareLaunchArgument,
            ExecuteProcess,
            GroupAction,
            IncludeLaunchDescription,
            OpaqueFunction,
            SetEnvironmentVariable,
            TimerAction,
        )
        from launch_ros.actions import (
            ComposableNodeContainer,
            LoadComposableNodes,
            PushRosNamespace,
        )
        from launch_ros.descriptions import ComposableNode
    except ImportError as exc:
        raise RuntimeError(
            "launch / launch_ros not available — "
            "run inside Docker with ROS2 sourced"
        ) from exc

    # Optional imports — not available in all distros
    try:
        from launch.actions import SetLaunchConfiguration
    except ImportError:
        SetLaunchConfiguration = None  # type: ignore[assignment,misc]
    try:
        from launch_ros.actions import SetRemap
    except ImportError:
        SetRemap = None  # type: ignore[assignment,misc]
    try:
        from launch_ros.actions import SetParameter
    except ImportError:
        try:
            from launch.actions import SetParameter  # type: ignore[attr-defined]
        except ImportError:
            SetParameter = None  # type: ignore[assignment,misc]
    try:
        from launch.actions import AppendEnvironmentVariable
    except ImportError:
        AppendEnvironmentVariable = None  # type: ignore[assignment,misc]

    for entity in entities:

        # ---- DeclareLaunchArgument ----------------------------------------
        if isinstance(entity, DeclareLaunchArgument):
            name = _subst_to_str(entity.name) or str(entity.name)
            # launch sets the default only when the configuration is unset
            # (declare_launch_argument.py:205-215): an including file's
            # value, or one set earlier, wins over the declared default.
            # A false condition skips the action (launch/action.py:105-109).
            # launch evaluates the condition whenever the action is visited,
            # so an invalid one fails even when the default would not apply.
            sets_default = entity.default_value is not None and \
                name not in dict(config_scope)
            applies = _apply_condition(
                entity, config_scope, file_info, f"DeclareLaunchArgument '{name}'",
                [name] if sets_default else [])
            if sets_default:
                scope_default = _perform_config(entity.default_value, config_scope)
                if scope_default is None:
                    resolved = resolver.resolve_path_substitution(entity.default_value)
                    scope_default = str(resolved) if resolved is not None else None
                if applies and scope_default is not None:
                    config_scope.append((name, scope_default))
            # Check if this argument was already pre-seeded by a parent
            # IncludeLaunchDescription — parent overrides take precedence.
            existing = {a.name for a in file_info.arguments}
            if name in existing:
                # Update description if the child provides one
                for a in file_info.arguments:
                    if a.name == name and a.description is None:
                        a.description = entity.description
                continue
            default = None
            if entity.default_value is not None:
                # Try single element (simple string/TextSubstitution); an
                # empty list is launch's '' (declare_launch_argument.py:214-215)
                default = _subst_to_str(entity.default_value[0]) \
                    if entity.default_value else ""
                # Try full list as concatenated string
                if default is None:
                    default = _subst_to_str(entity.default_value)
                # Fallback: resolve path-based defaults (FindPackageShare + "/config/...")
                if default is None:
                    resolved = resolver.resolve_path_substitution(entity.default_value)
                    if resolved is not None:
                        default = str(resolved)
            choices: list[str] | None = None
            if entity.choices:
                resolved_choices = [
                    _subst_to_str(c) for c in entity.choices
                ]
                # Keep only successfully resolved choices
                choices = [c for c in resolved_choices if c is not None] or None
            file_info.arguments.append(
                LaunchArgument(
                    name=name,
                    default_value=default,
                    description=entity.description,
                    choices=choices,
                )
            )
            continue

        # ---- Node -----------------------------------------------------------
        node_action = _extract_node(entity, resolver, ns_stack, config_scope, file_info.errors)
        if node_action is not None:
            starts = _starts(entity, config_scope, file_info,
                             f"Node '{node_action.name or node_action.executable}'",
                             undecided)
            if starts is False:
                continue
            node_action.condition_result = starts
            _bind_launch_args(node_action, file_info, config_scope)
            file_info.nodes.append(node_action)
            all_nodes.append(node_action)
            continue

        # ---- ComposableNodeContainer ----------------------------------------
        if isinstance(entity, ComposableNodeContainer):
            container_name = (
                _subst_to_str(
                    getattr(entity, "_Node__node_name", None)
                ) or "component_container"
            )
            raw_descs = getattr(
                entity,
                "_ComposableNodeContainer__composable_node_descriptions",
                None,
            )
            if raw_descs is None:
                raw_descs = getattr(entity, "composable_node_descriptions", None) or []
            # The container (or LoadComposableNodes) action and each
            # description's own condition decide (composable_node_container.py:85,
            # load_composable_nodes.py:279-282).
            result, reason = _scope_condition(entity, config_scope)
            if result is False:
                continue
            inside = _undecided_inside(entity, type(entity).__name__, result, reason,
                                       undecided)
            for comp in raw_descs:
                na = _extract_composable_node(comp, resolver, ns_stack, container_name,
                                              config_scope, file_info.errors)
                if na is not None:
                    starts = _starts(SimpleNamespace(condition=comp.condition()),
                                     config_scope, file_info,
                                     f"ComposableNode '{na.name or na.executable}'", inside)
                    if starts is False:
                        continue
                    na.condition_result = starts
                    _bind_launch_args(na, file_info, config_scope)
                    file_info.nodes.append(na)
                    all_nodes.append(na)
            continue

        # ---- LoadComposableNodes -------------------------------------------
        if isinstance(entity, LoadComposableNodes):
            container_name = _subst_to_str(
                getattr(entity, "_LoadComposableNodes__target_container", None)
                or getattr(entity, "target_container", None)
            )
            raw_descs = getattr(
                entity,
                "_LoadComposableNodes__composable_node_descriptions",
                None,
            )
            if raw_descs is None:
                raw_descs = getattr(entity, "composable_node_descriptions", None) or []
            # The container (or LoadComposableNodes) action and each
            # description's own condition decide (composable_node_container.py:85,
            # load_composable_nodes.py:279-282).
            result, reason = _scope_condition(entity, config_scope)
            if result is False:
                continue
            inside = _undecided_inside(entity, type(entity).__name__, result, reason,
                                       undecided)
            for comp in raw_descs:
                na = _extract_composable_node(comp, resolver, ns_stack, container_name,
                                              config_scope, file_info.errors)
                if na is not None:
                    starts = _starts(SimpleNamespace(condition=comp.condition()),
                                     config_scope, file_info,
                                     f"ComposableNode '{na.name or na.executable}'", inside)
                    if starts is False:
                        continue
                    na.condition_result = starts
                    _bind_launch_args(na, file_info, config_scope)
                    file_info.nodes.append(na)
                    all_nodes.append(na)
            continue

        # ---- IncludeLaunchDescription ---------------------------------------
        if isinstance(entity, IncludeLaunchDescription):
            _parent_defaults = {
                a.name: a.default_value
                for a in file_info.arguments
                if a.default_value is not None
            }
            # The condition is evaluated before the include adds its
            # arguments (launch/action.py:105-109). A false include is
            # skipped, as launch does. An undecidable one is parsed on a copy
            # of the scope and leaves every name it would set unknown.
            result, reason = _scope_condition(entity, config_scope)
            if result is False:
                continue
            include_scope = config_scope if result else list(config_scope)
            child = _resolve_include(
                entity, resolver, ns_stack, all_nodes, visited, depth,
                parent_arg_defaults=_parent_defaults,
                config_scope=include_scope,
                errors=file_info.errors,
                undecided=_undecided_inside(entity, "IncludeLaunchDescription",
                                            result, reason, undecided),
            )
            if child is not None:
                file_info.includes.append(child)
            if result is None:
                _apply_condition(entity, config_scope, file_info, "IncludeLaunchDescription",
                                 _names_set(config_scope, include_scope))
            continue

        # ---- GroupAction ----------------------------------------------------
        if isinstance(entity, GroupAction):
            # Jazzy stores actions in the private `_GroupAction__actions` attr;
            # fall back to `get_sub_entities()` if the private attr is absent.
            raw_actions = getattr(entity, "_GroupAction__actions", None)
            if raw_actions is None:
                raw_actions = getattr(entity, "actions", None)
            sub_entities = list(raw_actions) if raw_actions is not None else []
            pushed_ns: str | None = None
            remaining: list[Any] = []
            # Build a quick lookup of argument defaults already declared
            arg_defaults = {
                a.name: a.default_value
                for a in file_info.arguments
                if a.default_value is not None
            }

            for sub in sub_entities:
                if isinstance(sub, PushRosNamespace):
                    # Performed with the launch configurations in effect, as
                    # push_ros_namespace.py:67-82 does when it runs.
                    raw = _resolve_text(sub.namespace, config_scope, file_info.errors,
                                        "PushRosNamespace") or ""
                    pushed_ns = raw or None
                else:
                    remaining.append(sub)

            # forwarding=False also clears ``ros_namespace``, the configuration
            # PushRosNamespace and Node read (push_ros_namespace.py:67-82,
            # node.py:397), unless the group keeps it.
            unforwarded = getattr(entity, "_GroupAction__scoped", True) and \
                not getattr(entity, "_GroupAction__forwarding", True)
            # The condition is evaluated before the group changes any scope
            # (launch/action.py:105-109). A false group is skipped, as launch
            # does. An undecidable one is walked on a copy of the scope and
            # leaves every name it would set in the parent unknown.
            parent_scope = config_scope
            result, reason = _scope_condition(entity, parent_scope)
            if result is False:
                continue
            config_scope = parent_scope if result else list(parent_scope)
            base_stack = ns_stack
            if unforwarded:
                kept_ns = None
                for k, v in (getattr(
                        entity, "_GroupAction__launch_configurations", None) or {}).items():
                    if _perform_config(k, config_scope) == "ros_namespace":
                        kept_ns = _perform_config(v, config_scope)
                base_stack = [kept_ns] if kept_ns else []
            new_stack = base_stack + ([pushed_ns] if pushed_ns else [])
            group = LaunchGroup(namespace=pushed_ns, condition=_resolve_condition(entity))
            file_info.groups.append(group)

            # Walk sub-entities into a temporary file_info so that nodes and
            # includes are owned by the group, not flattened to the file level.
            from colcon_lf.launch_analysis.models import LaunchFileInfo as _LFI
            group_file_info = _LFI(
                file_path=file_info.file_path,
                package=file_info.package,
            )
            # Pre-seed with parent's declared arguments so conditions resolve.
            group_file_info.arguments = list(file_info.arguments)
            # launch's GroupAction (group_action.py:114-141): scoped=True (the
            # default) pushes the launch configurations, sets its
            # launch_configurations and pops after its actions, so sets before
            # the group reach nodes inside and sets inside do not reach nodes
            # after it. With forwarding=False the push is followed by a reset
            # to the launch_configurations, evaluated before the clear
            # (reset_launch_configurations.py:84-109). scoped=False sets them
            # in the parent's configurations.
            group_configs = list((getattr(
                entity, "_GroupAction__launch_configurations", None) or {}).items())
            scoped = getattr(entity, "_GroupAction__scoped", True)
            if unforwarded:
                group_scope = [
                    (_perform_config(k, config_scope) or str(k),
                     _perform_value(v, config_scope))
                    for k, v in group_configs
                ]
            else:
                group_scope = list(config_scope) if scoped else config_scope
                for k, v in group_configs:
                    group_scope.append((_perform_config(k, group_scope) or str(k),
                                        _perform_value(v, group_scope)))
            for k, v in group_scope[len(group_scope) - len(group_configs):]:
                if v is None:
                    group_file_info.errors.append(
                        f"GroupAction: launch configuration '{k}' not evaluated; "
                        f"'{k}' is unknown in the group")
            _walk_entities(
                remaining, resolver, new_stack, group_file_info, all_nodes,
                visited, depth, config_scope=group_scope,
                undecided=_undecided_inside(entity, "GroupAction", result, reason,
                                            undecided),
            )
            if result is None:
                _apply_condition(entity, parent_scope, file_info, "GroupAction",
                                 _names_set(parent_scope, config_scope))
            config_scope = parent_scope
            # Assign group-owned structure.
            group.nodes = group_file_info.nodes
            group.includes = group_file_info.includes
            group.groups = group_file_info.groups
            # Propagate side-effects to parent.
            file_info.set_environment.extend(group_file_info.set_environment)
            file_info.set_configurations.extend(group_file_info.set_configurations)
            file_info.set_parameters.extend(group_file_info.set_parameters)
            file_info.execute_processes.extend(group_file_info.execute_processes)
            file_info.errors.extend(group_file_info.errors)
            continue

        # ---- OpaqueFunction ------------------------------------------------
        if isinstance(entity, OpaqueFunction):
            try:
                from launch import LaunchContext
                context = LaunchContext()
                # Seed context with declared argument defaults
                for arg in file_info.arguments:
                    if arg.default_value is not None:
                        context.launch_configurations[arg.name] = arg.default_value
                # Seed with SetLaunchConfiguration values
                for cfg_name, cfg_val in file_info.set_configurations:
                    context.launch_configurations[cfg_name] = cfg_val
                # Command-line and include arguments and sets in effect.
                for cfg_name, cfg_val in config_scope:
                    if cfg_val is not None and not cfg_name.startswith("\0"):
                        context.launch_configurations[cfg_name] = cfg_val
                result = entity.execute(context)
                if result:
                    _walk_entities(
                        list(result), resolver, ns_stack, file_info,
                        all_nodes, visited, depth, config_scope=config_scope,
                        undecided=undecided,
                    )
            except Exception as exc:
                file_info.errors.append(f"OpaqueFunction skipped: {exc}")
            continue

        # ---- SetEnvironmentVariable ----------------------------------------
        if isinstance(entity, SetEnvironmentVariable):
            env_name = _subst_to_str(getattr(entity, "name", None)) or ""
            env_value = _subst_to_str(getattr(entity, "value", None)) or ""
            file_info.set_environment.append((env_name, env_value))
            continue

        # ---- AppendEnvironmentVariable -------------------------------------
        if AppendEnvironmentVariable is not None and isinstance(
            entity, AppendEnvironmentVariable
        ):
            env_name = _subst_to_str(getattr(entity, "name", None)) or ""
            env_value = _subst_to_str(getattr(entity, "value", None)) or ""
            file_info.set_environment.append((env_name, env_value))
            continue

        # ---- SetLaunchConfiguration ----------------------------------------
        if SetLaunchConfiguration is not None and isinstance(entity, SetLaunchConfiguration):
            # Name and value are performed when the action runs
            # (set_launch_configuration.py:71-74).
            raw_name = getattr(entity, "name", None)
            cfg_name = _perform_config(raw_name, config_scope) \
                or _subst_to_str(raw_name) or ""
            cfg_value = _subst_to_str(getattr(entity, "value", None)) or ""
            # A false condition skips the action (launch/action.py:105-109).
            applies = _apply_condition(
                entity, config_scope, file_info, f"SetLaunchConfiguration '{cfg_name}'",
                [cfg_name])
            if applies is False:
                continue
            file_info.set_configurations.append((cfg_name, cfg_value))
            if applies:
                # A value the analyzer cannot evaluate is unknown (None); its
                # text stays in set_configurations only.
                scope_value = _perform_value(getattr(entity, "value", None), config_scope)
                if scope_value is None:
                    file_info.errors.append(
                        f"SetLaunchConfiguration '{cfg_name}': value {cfg_value!r} not "
                        f"evaluated; '{cfg_name}' is unknown after it")
                config_scope.append((cfg_name, scope_value))
            continue

        # ---- SetParameter --------------------------------------------------
        if SetParameter is not None and isinstance(entity, SetParameter):
            # In Humble, name/value are tuples of TextSubstitution
            raw_name = getattr(entity, "name", None)
            raw_value = getattr(entity, "value", None)
            param_name = _resolve_param_key(raw_name) if raw_name is not None else ""
            param_value = _resolve_param_key(raw_value) if raw_value is not None else ""
            file_info.set_parameters.append((param_name, param_value))
            # Evaluated when the action runs; applies to every node and
            # composable node of the enclosing scope started after it, below
            # the node's own parameters (set_parameter.py:88-93).
            where = f"SetParameter '{param_name}'"
            applies = _apply_condition(entity, config_scope, file_info, where, [])
            if applies:
                typed = _resolve_param_value(raw_value, config_scope, file_info.errors, where)
                typed = _bind_value(typed, where, config_scope, file_info.errors)
                _check_primitive(param_name, typed, where, file_info.errors)
                config_scope.append((_PARAM_SCOPE, json.dumps([param_name, typed])))
            continue

        # ---- ExecuteProcess ------------------------------------------------
        if isinstance(entity, ExecuteProcess):
            raw_cmd = getattr(entity, "_ExecuteProcess__cmd", None) or \
                      getattr(entity, "cmd", [])
            cmd_parts = []
            for part in (raw_cmd if raw_cmd else []):
                s = _subst_to_str(part)
                if s is not None:
                    cmd_parts.append(s)
            proc_name = _subst_to_str(getattr(entity, "_ExecuteProcess__name", None) or
                                      getattr(entity, "name", None))
            proc_output = _subst_to_str(
                getattr(entity, "_ExecuteLocal__output", None)
                or getattr(entity, "_ExecuteProcess__output", None)
                or getattr(entity, "output", None)
            )
            proc = LaunchProcessAction(
                cmd=cmd_parts,
                name=proc_name,
                condition=_resolve_condition(entity),
                output=proc_output,
            )
            file_info.execute_processes.append(proc)
            continue

        # ---- TimerAction ---------------------------------------------------
        if isinstance(entity, TimerAction):
            raw_actions = getattr(entity, "_TimerAction__actions", None) or \
                          getattr(entity, "actions", None) or []
            result, reason = _scope_condition(entity, config_scope)
            if result is False:
                continue
            # The actions run when the timer fires, on a copy of the
            # configurations as they were when it started, and their changes
            # do not persist (timer_action.py handle()), so a set inside never
            # reaches the siblings after the timer.
            _walk_entities(
                list(raw_actions), resolver, ns_stack, file_info,
                all_nodes, visited, depth, config_scope=list(config_scope),
                undecided=_undecided_inside(entity, "TimerAction", result, reason,
                                            undecided),
            )
            continue

        # ---- PushRosNamespace (standalone, outside GroupAction) -------------
        if isinstance(entity, PushRosNamespace):
            raw = _resolve_text(getattr(entity, "namespace", None), config_scope,
                                file_info.errors, "PushRosNamespace") or ""
            if raw:
                ns_stack = ns_stack + [raw]
            continue

        # ---- SetRemap -------------------------------------------------------
        if SetRemap is not None and isinstance(entity, SetRemap):
            # Performed when the action runs; the rule reaches every node and
            # composable node of the enclosing scope started after it
            # (set_remap.py:88-93, node.py:465-474).
            src = _resolve_text(entity.src, config_scope, file_info.errors,
                                "SetRemap source")
            dst = _resolve_text(entity.dst, config_scope, file_info.errors,
                                "SetRemap target")
            applies = _apply_condition(entity, config_scope, file_info, "SetRemap", [])
            if applies and src is not None and dst is not None:
                config_scope.append((_REMAP_SCOPE, json.dumps([src, dst])))
            continue

        # ---- Actions whose effect the analysis cannot reproduce -------------
        # Silently ignoring one would give a system that differs from what
        # ``ros2 launch`` starts, so analysis refuses it.
        if type(entity).__name__ not in _HARMLESS_ACTIONS:
            file_info.errors.append(
                f"launch action {type(entity).__name__} is not supported by rcllf "
                f"analysis, which cannot derive its effect on the system; remove it "
                f"or replace it with an action rcllf analyzes (Node, "
                f"ComposableNodeContainer, LoadComposableNodes, GroupAction, "
                f"PushRosNamespace, SetRemap, SetParameter, SetLaunchConfiguration, "
                f"DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, "
                f"OpaqueFunction)")


def _describe_unresolved_path(raw_location: Any) -> str:
    """Return a human-readable stub path for an unresolvable include location.

    Tries to extract the package name and relative path from PathJoinSubstitution
    or FindPackageShare objects, producing strings like ``<pkg>/launch/file.py``
    instead of the raw Python repr.
    """
    if raw_location is None:
        return "unresolved"
    if isinstance(raw_location, (str, Path)):
        return str(raw_location)

    def _hint(subst: Any, depth: int = 0) -> str | None:
        if depth > 10 or subst is None:
            return None
        if isinstance(subst, str):
            return subst
        if isinstance(subst, Path):
            return str(subst)
        if isinstance(subst, list):
            parts = [_hint(s, depth + 1) for s in subst]
            non_none = [p for p in parts if p is not None]
            return "/".join(non_none) if non_none else None
        try:
            from launch_ros.substitutions import FindPackageShare as _FPS
            if isinstance(subst, _FPS):
                raw_pkg = getattr(subst, "_FindPackage__package", None)
                if isinstance(raw_pkg, list) and raw_pkg:
                    from launch.substitutions import TextSubstitution as _TS
                    if isinstance(raw_pkg[0], _TS):
                        return f"<{raw_pkg[0].text}>"
                return "<pkg>"
        except ImportError:
            pass
        try:
            from launch.substitutions import TextSubstitution as _TS
            if isinstance(subst, _TS):
                return subst.text
        except ImportError:
            pass
        try:
            from launch.substitutions import PathJoinSubstitution as _PJS
            if isinstance(subst, _PJS):
                parts = []
                for s in subst.substitutions:
                    item = s[0] if (isinstance(s, list) and len(s) == 1) else s
                    h = _hint(item, depth + 1)
                    parts.append(h or "?")
                return "/".join(parts) if parts else None
        except ImportError:
            pass
        return None

    result = _hint(raw_location)
    return result if result else "unresolved-include"


def _resolve_include(
    entity: Any,
    resolver: _PackageResolver,
    ns_stack: list[str],
    all_nodes: list[LaunchNodeAction],
    visited: set[Path],
    depth: int = 0,
    parent_arg_defaults: dict[str, str] | None = None,
    config_scope: list[tuple[str, str | None]] | None = None,
    errors: list[str] | None = None,
    undecided: str | None = None,
) -> LaunchFileInfo | None:
    """Resolve an IncludeLaunchDescription and recursively parse it.

    A launch argument the analyzer cannot evaluate is recorded in *errors*.
    Returns a :class:`LaunchFileInfo` on success, a stub with an error
    message if the included file cannot be loaded, or *None* if the
    include path cannot be resolved at all.
    """
    try:
        # LaunchDescriptionSource lives in the singular module in Jazzy
        from launch.launch_description_source import LaunchDescriptionSource
    except ImportError:
        return None

    source = entity.launch_description_source
    if not isinstance(source, LaunchDescriptionSource):
        return None

    # `.location` returns a string repr of the substitution (e.g.
    # "PathJoinSubstitution(...)").  The real substitution object is stored
    # in the private `__location` attribute — use that for static resolution.
    raw_location = getattr(
        source, "_LaunchDescriptionSource__location", None
    ) or source.location
    # For XML sources, location may be a plain string path.
    if isinstance(raw_location, str):
        raw_location = Path(raw_location)
    resolved_path = resolver.resolve_path_substitution(raw_location)
    if resolved_path is None or not resolved_path.exists():
        # Cannot resolve path — create a stub with an error
        stub_path = _describe_unresolved_path(raw_location)
        return LaunchFileInfo(
            file_path=Path(stub_path),
            package="?",
            errors=[f"Unresolved include: {stub_path}"],
        )

    resolved_path = resolved_path.resolve()

    raw_args = getattr(entity, "_IncludeLaunchDescription__launch_arguments", None)
    if raw_args is None:
        raw_args = getattr(entity, "launch_arguments", None)
    # launch sets each launch argument as a configuration in the current
    # scope, in order, and then yields the included description without
    # pushing a scope (include_launch_description.py:238-243). The arguments
    # and every set inside the included file stay in effect for the parent's
    # later actions, so the child walks the parent's own list.
    if config_scope is None:
        config_scope = []
    for pair in raw_args or ():
        if isinstance(pair, (list, tuple)) and len(pair) >= 2:
            key = _perform_config(pair[0], config_scope)
            val = _perform_value(pair[1], config_scope)
            if val is None:
                # A package path, as for a declared default.
                resolved = resolver.resolve_path_substitution(pair[1])
                val = str(resolved) if resolved is not None else None
            if key:
                # A value the analyzer cannot evaluate is unknown (None).
                config_scope.append((key, val))
                if val is None and errors is not None:
                    message = (f"IncludeLaunchDescription: launch argument '{key}' not "
                               f"evaluated; '{key}' is unknown after it")
                    if message not in errors:
                        errors.append(message)

    if resolved_path in visited:
        return None

    if depth >= _MAX_INCLUDE_DEPTH:
        return LaunchFileInfo(
            file_path=resolved_path,
            package=_guess_package(resolved_path),
            errors=[f"Max include depth ({_MAX_INCLUDE_DEPTH}) exceeded"],
        )

    # Extract forwarded launch arguments from IncludeLaunchDescription.
    # These override DeclareLaunchArgument defaults in the included file.
    arg_overrides: dict[str, str] = {}
    if raw_args:
        for pair in raw_args:
            if isinstance(pair, (list, tuple)) and len(pair) >= 2:
                key = _subst_to_str(pair[0])
                val = _subst_to_str(pair[1])
                if key and val is not None:
                    # If the value is a $(var VAR) reference, resolve it
                    # to the parent's declared argument default.
                    if (
                        val.startswith("$(var ") and val.endswith(")")
                        and parent_arg_defaults
                    ):
                        var_name = val[6:-1]
                        val = parent_arg_defaults.get(var_name, val)
                    arg_overrides[key] = val

    try:
        return _parse_file(
            resolved_path, resolver, ns_stack, all_nodes, visited,
            depth + 1, arg_overrides=arg_overrides,
            config_scope=config_scope, undecided=undecided,
        )
    except Exception as exc:
        import logging
        logging.getLogger(__name__).warning(
            "Failed to parse included launch file %s: %s", resolved_path, exc
        )
        return LaunchFileInfo(
            file_path=resolved_path,
            package=_guess_package(resolved_path),
            errors=[f"Import failed: {exc}"],
        )


# ---------------------------------------------------------------------------
# File loader
# ---------------------------------------------------------------------------

def _parse_file(
    launch_path: Path,
    resolver: _PackageResolver,
    ns_stack: list[str],
    all_nodes: list[LaunchNodeAction],
    visited: set[Path],
    depth: int = 0,
    arg_overrides: dict[str, str] | None = None,
    config_scope: list[tuple[str, str | None]] | None = None,
    undecided: str | None = None,
) -> LaunchFileInfo:
    """Load and parse a single launch file (Python, XML, or YAML).

    Uses ROS2's ``AnyLaunchDescriptionSource`` which auto-detects format
    from the file extension (``.py``, ``.xml``, ``.yaml``).  All formats
    produce the same ``LaunchDescription`` entity tree.

    Parameters
    ----------
    arg_overrides:
        Launch argument overrides forwarded from a parent
        ``IncludeLaunchDescription(launch_arguments={...})``.
        These take precedence over ``DeclareLaunchArgument`` defaults.
    config_scope:
        Launch configurations in effect at the include, already holding the
        include's arguments (see :func:`_resolve_include`). The included file
        walks this list itself: an include does not push a scope.
    undecided:
        The undecidable condition of an enclosing action (``_walk_entities``).
    """
    visited.add(launch_path)
    package = _guess_package(launch_path)

    try:
        from launch.launch_description_sources import (
            AnyLaunchDescriptionSource,
        )
    except ImportError as exc:
        return LaunchFileInfo(
            file_path=launch_path,
            package=package,
            errors=[f"launch package not available: {exc}"],
        )

    # Loading a Python launch file imports it.  Without this guard the import
    # writes a __pycache__ directory into the analysed package's source tree,
    # and a later `colcon build --symlink-install` of an ament_python package
    # whose setup.py globs launch/* fails on it (nav2_simple_commander).
    with _BYTECODE_LOCK:
        dont_write_bytecode = sys.dont_write_bytecode
        sys.dont_write_bytecode = True
        try:
            source = AnyLaunchDescriptionSource(str(launch_path))
            ld = source.try_get_launch_description_without_context()
        except Exception as exc:
            return LaunchFileInfo(
                file_path=launch_path,
                package=package,
                errors=[f"Failed to load launch file: {exc}"],
            )
        finally:
            sys.dont_write_bytecode = dont_write_bytecode

    if ld is None:
        return LaunchFileInfo(
            file_path=launch_path,
            package=package,
            errors=[f"Could not load launch description from {launch_path}"],
        )

    file_info = LaunchFileInfo(file_path=launch_path, package=package)

    # Pre-seed arguments from parent's IncludeLaunchDescription overrides.
    # These are treated as if the child file had declared the argument with
    # this value as its default — they take precedence over the child's own
    # DeclareLaunchArgument defaults during OpaqueFunction context seeding.
    if arg_overrides:
        for arg_name, arg_val in arg_overrides.items():
            file_info.arguments.append(
                LaunchArgument(name=arg_name, default_value=arg_val)
            )

    _walk_entities(
        list(ld.entities), resolver, ns_stack, file_info, all_nodes, visited,
        depth, config_scope=config_scope if config_scope is not None else [],
        undecided=undecided,
    )
    return file_info


def _guess_package(launch_path: Path) -> str:
    """Infer the package name by walking up for a package.xml."""
    path = launch_path.parent
    for _ in range(4):
        if (path / "package.xml").exists():
            return manifest_package_name(path / "package.xml") or path.name
        path = path.parent
    return launch_path.parent.parent.name


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def _declared_arguments(file_info: LaunchFileInfo) -> set[str]:
    """Names of the launch arguments *file_info* and what it includes declare."""
    names = {a.name for a in file_info.arguments}
    for child in file_info.includes:
        names |= _declared_arguments(child)
    pending = list(file_info.groups)
    while pending:
        group = pending.pop()
        pending.extend(group.groups)
        for child in group.includes:
            names |= _declared_arguments(child)
    return names


def parse_launch(
    launch_file: str | Path,
    workspace_root: str | Path | None = None,
    launch_args: dict[str, str] | None = None,
) -> LaunchGraph:
    """Parse a launch file and return a :class:`LaunchGraph`.

    Results are cached by resolved *launch_file* path — repeated calls
    for the same file return the cached result without re-importing.

    Parameters
    ----------
    launch_file:
        Absolute path to a launch file (``.launch.py``, ``_launch.py``,
        ``.launch.xml``, or ``.launch.yaml``).
    workspace_root:
        Colcon workspace root (contains ``src/`` and ``install/``).
        Inferred automatically if *None*.
    launch_args:
        ``name:=value`` command-line launch arguments (default: those of the
        enclosing :func:`launch_argument_overrides`). They are set before the
        file runs, as ``ros2 launch`` sets them, and are part of the cache
        key. An argument the file and its includes do not declare raises
        :class:`LaunchArgumentError`, as ``ros2 launch`` rejects it.
    """
    launch_file = Path(launch_file).resolve()
    if launch_args is None:
        launch_args = _active_launch_args
    cache_key = (launch_file, tuple(sorted(launch_args.items())))

    if cache_key in _parse_launch_cache:
        return _parse_launch_cache[cache_key]

    if workspace_root is None:
        candidate = launch_file.parent
        for _ in range(8):
            if (candidate / "src").is_dir():
                workspace_root = candidate
                break
            candidate = candidate.parent
        if workspace_root is None:
            workspace_root = launch_file.parent

    resolver = _PackageResolver(Path(workspace_root))
    all_nodes: list[LaunchNodeAction] = []
    visited: set[Path] = set()
    root = _parse_file(
        launch_file, resolver, [], all_nodes, visited,
        config_scope=list(launch_args.items()))
    undeclared = sorted(set(launch_args) - _declared_arguments(root))
    if undeclared:
        declared = sorted(_declared_arguments(root))
        raise LaunchArgumentError(
            f"{launch_file.name}: launch argument(s) {', '.join(undeclared)} are not "
            f"declared by the launch file; it declares: "
            f"{', '.join(declared) if declared else '(none)'}")
    result = LaunchGraph(root=root, all_nodes=all_nodes)
    _parse_launch_cache[cache_key] = result
    return result
