# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""libclang-based C++ node analyser for ROS2 packages.

Parses a single ``.cpp`` source file with libclang, walking the AST to
extract:

- Node class name and base class (``rclcpp::Node`` / ``rclcpp_lifecycle::LifecycleNode``)
- Publishers, subscribers, service servers/clients, action servers/clients
- Timers (with period in ms)
- Callback groups (``MutuallyExclusive`` / ``Reentrant``)
- Declared parameters (name + type)
- Detection of ``add_on_set_parameters_callback``
- Lifecycle callback methods (on_configure, on_activate, …)
- Component registration (``RCLCPP_COMPONENTS_REGISTER_NODE``)
- Method signatures for named callbacks (parameter names + types)

Include paths are loaded from ``compile_commands.json`` in the colcon build
directory (``<workspace_root>/build/<package>/compile_commands.json``).  If
no entry is found, include paths are auto-discovered from the install tree.

Public API
----------
``parse_node(cpp_path, workspace_root=None) -> NodeInfo``
"""

from __future__ import annotations

import dataclasses
import functools
import json
import os
import re
import shlex
import shutil
import subprocess
from collections import Counter
from fractions import Fraction
from pathlib import Path
from typing import Any, Callable

from colcon_lf.node_analysis.models import (
    ActionClientInfo,
    ActionServerInfo,
    CallbackGroupInfo,
    CallbackInfo,
    CallbackParameter,
    NodeInfo,
    ParameterInfo,
    PublisherInfo,
    ServiceClientInfo,
    ServiceServerInfo,
    LIFECYCLE_BASE_SUBSTRINGS,
    LIFECYCLE_CHAINED_HOOKS,
    SubscriberInfo,
    TimerInfo,
    is_lifecycle_base,
)
from colcon_lf.workspace import manifest_package_name


# ---------------------------------------------------------------------------
# Module-level parse result cache (Phase 7.1)
# ---------------------------------------------------------------------------

# Keyed by (source path, -D flags): one file compiled with different
# definitions per executable (``#ifdef`` mains) parses to different nodes.
_parse_node_cache: dict[tuple[Path, tuple[str, ...]], NodeInfo] = {}


def clear_parse_node_cache() -> None:
    """Clear the module-level ``parse_node()`` result cache."""
    _parse_node_cache.clear()


# ---------------------------------------------------------------------------
# Chrono literal helpers
# ---------------------------------------------------------------------------

# Exact milliseconds per unit. The binding matches a timer by its
# exact period in nanoseconds, and the generator floors the exact decimal of
# period_ms (policies.period_ms_to_ns). A float product such as 5 * 1e-6 is
# 4.9999999999999996e-06 ms, which floors to 4 ns for a 5 ns timer, so the
# literal is scaled exactly and rounded to the nearest float once, whose
# shortest repr is the literal's own decimal.
_CHRONO_SUFFIXES: dict[str, Fraction] = {
    "ns": Fraction(1, 1_000_000),
    "us": Fraction(1, 1_000),
    "ms": Fraction(1),
    "s": Fraction(1_000),
    "min": Fraction(60_000),
    "h": Fraction(3_600_000),
}


def _chrono_ms(value: str, multiplier: Fraction, exact: bool = True) -> float | None:
    """Milliseconds of the literal *value* in a unit of *multiplier* ms.

    With *exact*, None when no float carries the value exactly. A float holds
    about 16 significant digits, so a literal such as 10000000000000001ns
    (10000000000.000001 ms) rounds to 10000000000.000002 ms, one ns longer
    than the timer, and no handle would fit its RID. None leaves the period
    unresolved, which the R001 gate refuses by name instead.
    """
    try:
        ms = Fraction(value) * multiplier
    except (ValueError, ZeroDivisionError):
        return None
    out = float(ms)
    if exact and Fraction(repr(out)) != ms:
        return None
    return out


def _parse_chrono_ms(tokens: list[str]) -> float | None:
    """Return milliseconds from a token sequence like ``['500', 'ms']``."""
    if len(tokens) < 2:
        return None
    multiplier = _CHRONO_SUFFIXES.get(tokens[1])
    if multiplier is None:
        return None
    return _chrono_ms(tokens[0], multiplier)


# ---------------------------------------------------------------------------
# Include path resolution
# ---------------------------------------------------------------------------

def _gcc_builtin_flags() -> list[str]:
    """Return -isystem flags for GCC built-in headers (stddef.h, etc.).

    The pip ``libclang`` package bundles its own libclang.so but not the GCC
    compiler built-in headers.  Without these, template-heavy code (like ROS2
    nodes) silently drops AST nodes when ``stddef.h`` is unresolvable.
    """
    candidates = [
        # GCC 13 on Ubuntu 24.04
        "/usr/lib/gcc/x86_64-linux-gnu/13/include",
        "/usr/lib/gcc/x86_64-linux-gnu/12/include",
        # aarch64
        "/usr/lib/gcc/aarch64-linux-gnu/13/include",
        "/usr/lib/gcc/aarch64-linux-gnu/12/include",
    ]
    result: list[str] = []
    for p in candidates:
        if Path(p).is_dir():
            result.append(f"-isystem{p}")
    return result


def _match_compile_entry(entry_file: str, cpp_path: Path, workspace_root: Path) -> bool:
    """Check if a compile_commands.json entry matches *cpp_path*.

    Handles path prefix mismatches when the workspace was built inside a
    Docker container (e.g. ``/workspace/...``) but analysis runs on the
    host filesystem (e.g. ``~/ws/src/<pkg>/...``).
    """
    entry_path = Path(entry_file)
    # Direct match (same filesystem)
    try:
        if entry_path.resolve() == cpp_path.resolve():
            return True
    except OSError:
        pass
    # Relative path match: strip workspace prefix from both sides
    # Common container prefixes used by Docker volumes
    container_prefixes = [Path("/workspace"), Path("/ws"), Path("/colcon_ws")]
    for prefix in [workspace_root] + container_prefixes:
        try:
            entry_rel = entry_path.relative_to(prefix)
        except ValueError:
            continue
        try:
            cpp_rel = cpp_path.relative_to(workspace_root)
        except ValueError:
            continue
        if entry_rel == cpp_rel:
            return True
    return False


def _rewrite_container_path(flag: str, workspace_root: Path) -> str:
    """Rewrite ``-I`` / ``-isystem`` paths from container to host filesystem.

    Only rewrites if the original path doesn't exist on disk (i.e. running
    outside the build container).
    """
    for opt in ("-I", "-isystem"):
        if flag.startswith(opt):
            path_part = flag[len(opt):]
            if Path(path_part).is_dir():
                return flag  # Path exists as-is, no rewrite needed
            for prefix in ("/workspace/", "/ws/", "/colcon_ws/"):
                if path_part.startswith(prefix):
                    rewritten = str(workspace_root / path_part[len(prefix):])
                    if Path(rewritten).is_dir():
                        return f"{opt}{rewritten}"
    return flag


_TARGET_DIR_RE = re.compile(r"CMakeFiles/([^/\\]+)\.dir/")


def _entry_tokens(entry: dict) -> list[str]:
    """The compiler words of a compile_commands.json entry.

    CMake writes ``command``; other generators write ``arguments``.
    """
    args = entry.get("arguments")
    if isinstance(args, list) and args:
        return [str(a) for a in args]
    return shlex.split(entry.get("command", ""))


def _entry_target(entry: dict) -> str | None:
    """The CMake target an entry compiles for, from its output object path."""
    out = entry.get("output", "")
    if not out:
        toks = _entry_tokens(entry)
        for i, tok in enumerate(toks):
            if tok == "-o" and i + 1 < len(toks):
                out = toks[i + 1]
                break
    m = _TARGET_DIR_RE.search(out)
    return m.group(1) if m else None


def _matching_compile_entries(cpp_path: Path, workspace_root: Path) -> list[dict]:
    """Every compile_commands.json entry of one package build that compiles *cpp_path*."""
    build_root = workspace_root / "build"
    if not build_root.is_dir():
        return []
    for pkg_build in build_root.iterdir():
        ccj = pkg_build / "compile_commands.json"
        if not ccj.exists():
            continue
        try:
            db = json.loads(ccj.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        hits = [
            e for e in db
            if _match_compile_entry(e.get("file", ""), cpp_path, workspace_root)
        ]
        if hits:
            return hits
    return []


def _entry_defines(entry: dict) -> tuple[str, ...]:
    return tuple(sorted(t for t in _entry_tokens(entry) if t.startswith("-D")))


def _compile_variants(cpp_path: Path, workspace_root: Path) -> dict[str | None, tuple[str, ...]]:
    """``{target: -D flags}`` for every build target compiling *cpp_path*."""
    return {
        _entry_target(e): _entry_defines(e)
        for e in _matching_compile_entries(cpp_path, workspace_root)
    }


def _load_include_flags(
    cpp_path: Path,
    workspace_root: Path,
    target: str | None = None,
    extra_defines: list[str] | None = None,
) -> list[str]:
    """Return compiler ``-I``/``-D`` flags for *cpp_path*.

    Tries compile_commands.json in each package build directory first;
    falls back to auto-discovery from the install tree. A file several
    executables compile (``add_executable(a x.cpp)``,
    ``add_executable(b x.cpp)`` with ``target_compile_definitions(b ...)``)
    has one entry per target: the one of *target* is used, the first otherwise.
    *extra_defines* (``-DNAME`` words from the CMakeLists of *target*) are
    added when no compile_commands.json entry carries the target's own flags.
    """
    entries = _matching_compile_entries(cpp_path, workspace_root)
    if entries:
        entry = next(
            (e for e in entries if target is not None and _entry_target(e) == target),
            entries[0],
        )
        flags = _entry_tokens(entry)
        result: list[str] = []
        i = 0
        while i < len(flags):
            f = flags[i]
            if f.startswith("-I") or f.startswith("-D") or f.startswith("-std="):
                result.append(_rewrite_container_path(f, workspace_root))
            elif f == "-isystem":
                # -isystem <path> is two tokens; convert to -isystem<path>
                # libclang accepts -isystem as a combined flag
                if i + 1 < len(flags):
                    combined = f"-isystem{flags[i + 1]}"
                    result.append(_rewrite_container_path(combined, workspace_root))
                    i += 1
            elif f.startswith("-isystem"):
                result.append(_rewrite_container_path(f, workspace_root))
            i += 1
        # libclang (pip package) doesn't auto-discover GCC built-in
        # headers (stddef.h, etc.).  Add them so template-heavy ROS2
        # code parses correctly.
        result.extend(_gcc_builtin_flags())
        if extra_defines and not (
            target is not None and any(_entry_target(e) == target for e in entries)
        ):
            result.extend(d for d in extra_defines if d not in result)
        return result

    # Fallback: the package's own include/ first (what its CMakeLists passes
    # via include_directories), then all include dirs from the install tree.
    flags: list[str] = ["-std=c++17"]
    pkg_root = _package_root(cpp_path, workspace_root)
    if pkg_root is not None and (pkg_root / "include").is_dir():
        flags.append(f"-I{pkg_root / 'include'}")
    install_root = workspace_root / "install"
    if install_root.is_dir():
        for inc in install_root.glob("*/include"):
            flags.append(f"-I{inc}")
            for sub in inc.iterdir():
                if sub.is_dir():
                    flags.append(f"-I{sub}")
    # Try all known ROS2 distro include paths
    for distro in ("jazzy", "humble", "iron", "rolling"):
        ros_inc = Path(f"/opt/ros/{distro}/include")
        if ros_inc.is_dir():
            flags.append(f"-I{ros_inc}")
            for sub in ros_inc.iterdir():
                if sub.is_dir():
                    flags.append(f"-I{sub}")
            break  # Use the first available distro
    # Last, include dirs a package build produced but did not install, such
    # as a vendored SDK unpacked into the build tree (the MuJoCo headers).
    # Only build/<pkg>/include and build/<pkg>/*/include: a
    # bounded glob, and searched after the install tree so it can only fill
    # a gap, never shadow an installed header.
    if pkg_root is not None:
        pkg_build = workspace_root / "build" / _package_name(pkg_root)
        if pkg_build.is_dir():
            for inc in [pkg_build / "include", *sorted(pkg_build.glob("*/include"))]:
                if inc.is_dir():
                    flags.append(f"-I{inc}")
    # Same GCC built-in header gap as the compile_commands.json branch above:
    # the pip `libclang` package doesn't auto-discover stddef.h etc., which
    # silently drops template-heavy ROS2 AST nodes (e.g. create_subscription<T>)
    # without these.
    flags.extend(_gcc_builtin_flags())
    if extra_defines:
        flags.extend(d for d in extra_defines if d not in flags)
    return flags


def _package_root(cpp_path: Path, workspace_root: Path | None = None) -> Path | None:
    """Return the nearest ancestor of *cpp_path* holding a ``package.xml``.

    The walk has no depth limit. It stops at *workspace_root*, so a source
    outside any package never picks up a package.xml above the workspace.
    """
    stop = Path(workspace_root).absolute() if workspace_root is not None else None
    for path in Path(cpp_path).absolute().parents:
        if (path / "package.xml").is_file():
            return path
        if path == stop:
            break
    return None


def _package_name(pkg_root: Path) -> str:
    """Return the ``<name>`` from *pkg_root*'s package.xml (colcon's build dir name)."""
    try:
        m = re.search(r"<name>\s*([^<\s]+)\s*</name>", (pkg_root / "package.xml").read_text())
    except OSError:
        m = None
    return m.group(1) if m else pkg_root.name


# ---------------------------------------------------------------------------
# Workspace root inference
# ---------------------------------------------------------------------------

def _infer_workspace_root(cpp_path: Path) -> Path:
    """Walk up from *cpp_path* to find the colcon workspace root (has src/)."""
    candidate = cpp_path.parent
    for _ in range(10):
        if (candidate / "src").is_dir() and (candidate / "build").is_dir():
            return candidate
        if (candidate / "src").is_dir():
            return candidate
        candidate = candidate.parent
    return cpp_path.parent


# ---------------------------------------------------------------------------
# Token helpers (libclang)
# ---------------------------------------------------------------------------

def _cursor_tokens(cursor: Any) -> list[str]:
    """Return the raw spelling of all tokens in a cursor's extent."""
    try:
        return [t.spelling for t in cursor.get_tokens()]
    except Exception:
        return []


def _first_string_literal(cursor: Any) -> str | None:
    """Return the first STRING_LITERAL descendant value of a CALL_EXPR cursor.

    Uses walk_preorder so it handles arbitrarily nested implicit casts and
    UNEXPOSED_EXPR wrappers that libclang inserts around string arguments.
    Stops at the first non-MEMBER_REF_EXPR child to avoid reading the method
    name itself.

    Also handles ``constexpr`` / ``const char[]`` variables: if no inline
    string literal is found, looks for a DECL_REF_EXPR pointing to a VAR_DECL
    and extracts the string from the variable's initializer.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None
    # Skip the first child (MEMBER_REF_EXPR = the method reference "create_publisher")
    # and search remaining subtree for the first string literal.
    children = list(cursor.get_children())
    search_roots = children[1:] if len(children) > 1 else children
    for root in search_roots:
        for desc in root.walk_preorder():
            if desc.kind == CursorKind.STRING_LITERAL:
                tokens = _cursor_tokens(desc)
                if tokens:
                    raw = tokens[0]
                    if raw.startswith('"') and raw.endswith('"'):
                        return raw[1:-1]
                    return raw

    # Fallback: look for a DECL_REF_EXPR pointing to a constexpr/const variable
    first_var_name: str | None = None
    for root in search_roots:
        for desc in root.walk_preorder():
            if desc.kind == CursorKind.DECL_REF_EXPR:
                ref = desc.referenced
                if ref is not None and ref.kind == CursorKind.VAR_DECL:
                    result = _string_from_var_decl(ref)
                    if result is not None:
                        return result
                    # Remember the variable name for the placeholder fallback
                    if first_var_name is None:
                        first_var_name = desc.spelling or None
    # Last resort: use the variable name as a placeholder so the topic is
    # visible in output even when it's determined at runtime via parameters.
    if first_var_name:
        return f"${{{first_var_name}}}"
    return None


# Cursor kinds that may legitimately appear in the subtree of a *lone* string
# literal argument: libclang wraps string arguments in implicit casts and
# unexposed expressions, and a ``std::string`` parameter adds a constructor
# call around the literal. Anything outside this set means the topic argument
# expression does more than name one literal string.
_LITERAL_WRAPPER_KINDS = frozenset({
    "STRING_LITERAL",
    "UNEXPOSED_EXPR",
    "PAREN_EXPR",
    "CSTYLE_CAST_EXPR",
    "CXX_STATIC_CAST_EXPR",
    "CXX_CONST_CAST_EXPR",
    "CXX_FUNCTIONAL_CAST_EXPR",
    # std::string / rclcpp::Node's const std::string & parameter
    "CALL_EXPR",
    "CXX_CONSTRUCT_EXPR",
    "CXX_TEMPORARY_OBJECT_EXPR",
    "CXX_BIND_TEMPORARY_EXPR",
    "MATERIALIZE_TEMPORARY_EXPR",
    "TYPE_REF",
    "NAMESPACE_REF",
    "TEMPLATE_REF",
    "DECL_REF_EXPR",  # names std::string / operator overloads, see below
})

# Kinds that unambiguously prove the expression computes the name rather than
# naming a literal: string concatenation, ternaries, member/array access,
# ``std::to_string`` results, lambdas, etc.
_DYNAMIC_MARKER_KINDS = frozenset({
    "BINARY_OPERATOR",
    "COMPOUND_ASSIGNMENT_OPERATOR",
    "CONDITIONAL_OPERATOR",
    "MEMBER_REF_EXPR",
    "ARRAY_SUBSCRIPT_EXPR",
    "LAMBDA_EXPR",
    "UNARY_OPERATOR",
    "INTEGER_LITERAL",
    "FLOATING_LITERAL",
})


def classify_topic_expression(
    kinds: list[str],
    *,
    literal_count: int,
    referenced_var_names: list[str],
    resolved_const_count: int = 0,
) -> tuple[bool, str]:
    """Decide whether a topic-argument expression resolves to one static name.

    Pure function over already-extracted facts so it is unit-testable without
    libclang (the caller does the AST walking).

    Parameters
    ----------
    kinds:
        ``CursorKind.name`` for every cursor in the topic argument's subtree,
        in pre-order.
    literal_count:
        How many ``STRING_LITERAL`` cursors the subtree contains. Note that a
        C++ adjacent-literal concatenation (``"a" "b"``) is a *single*
        STRING_LITERAL cursor, so >1 means genuinely separate operands.
    referenced_var_names:
        Names of ``DECL_REF_EXPR`` cursors referencing a variable/field whose
        value could **not** be resolved statically.
    resolved_const_count:
        How many ``DECL_REF_EXPR`` cursors reference a ``constexpr`` /
        ``const char[]`` variable whose initializer *was* resolved (by
        ``_string_from_var_decl``). Such a reference is as good as an inline
        literal: ``_first_string_literal`` returns the real name for it, so it
        must not be reported as dynamic even though ``literal_count`` is 0 —
        the literal lives in the *declaration's* subtree, not the call's.

    Returns
    -------
    (is_dynamic, reason)
        ``is_dynamic`` is True when the expression is *not* a lone resolvable
        string literal — the RCLLF-H004 condition. ``reason`` is a short
        human-readable classification used in warning/diagnostic text
        (``""`` when not dynamic).
    """
    # A resolved const reference counts as a literal for every check below.
    effective_literals = literal_count + resolved_const_count
    # Structural markers first: they name the *shape* of the computation
    # (BINARY_OPERATOR, CONDITIONAL_OPERATOR, ...), which is a more useful
    # reason string than a bare literal count.
    for kind in kinds:
        if kind in _DYNAMIC_MARKER_KINDS:
            return True, f"computed expression ({kind})"
    if effective_literals > 1:
        return True, "multiple string literals (concatenation)"
    if effective_literals == 0:
        if referenced_var_names:
            return True, f"variable reference ('{referenced_var_names[0]}')"
        return True, "no string literal in the argument expression"
    # Exactly one literal and no dynamic markers — but an *unresolved* variable
    # reference alongside it still means the name is composed at runtime
    # (e.g. a helper call `make_name(prefix_, "suffix")`).
    if referenced_var_names:
        return True, f"string literal combined with variable '{referenced_var_names[0]}'"
    unknown = [k for k in kinds if k not in _LITERAL_WRAPPER_KINDS]
    if unknown:
        return True, f"unexpected node in topic expression ({unknown[0]})"
    return False, ""


def _topic_argument_roots(cursor: Any) -> list[Any]:
    """Return the subtree roots ``_first_string_literal`` searches for *cursor*.

    Mirrors ``_first_string_literal``'s "skip the first child (the
    MEMBER_REF_EXPR naming the method)" rule so the classifier sees exactly the
    same cursors the extractor does.
    """
    children = list(cursor.get_children())
    return children[1:] if len(children) > 1 else children


def _classify_topic_argument(cursor: Any, arg_index: int = 0) -> tuple[bool, str]:
    """Classify a pub/sub CALL_EXPR's topic argument via :func:`classify_topic_expression`.

    ``arg_index`` is the zero-based position of the topic among the call's
    arguments: 0 for ``create_publisher(topic, ...)``, 1 for the Nav2 wrapper
    constructors ``make_unique<TwistPublisher>(node, topic, ...)``.

    Returns ``(is_dynamic, reason)``. Fails *safe* (not dynamic) when libclang
    is unavailable, so behaviour outside a parsing environment is unchanged.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return False, ""

    kinds: list[str] = []
    literal_count = 0
    resolved_const_count = 0
    referenced_var_names: list[str] = []
    # Only the topic argument subtree names the topic; the node, QoS and
    # callback arguments around it are irrelevant (and full of dynamic-looking
    # nodes).
    roots = _topic_argument_roots(cursor)
    if len(roots) <= arg_index:
        return True, "no arguments"
    for desc in roots[arg_index].walk_preorder():
        name = desc.kind.name
        kinds.append(name)
        if desc.kind == CursorKind.STRING_LITERAL:
            literal_count += 1
        elif desc.kind == CursorKind.DECL_REF_EXPR:
            ref = desc.referenced
            if ref is not None and ref.kind == CursorKind.VAR_DECL:
                # A resolvable constexpr/const char[] initializer is as good as
                # an inline literal — that is exactly what _first_string_literal
                # resolves, so it must not be reported as dynamic.
                if _string_from_var_decl(ref) is None:
                    referenced_var_names.append(desc.spelling or "<unnamed>")
                else:
                    resolved_const_count += 1
            elif ref is not None and ref.kind == CursorKind.FIELD_DECL:
                referenced_var_names.append(desc.spelling or "<unnamed>")
    return classify_topic_expression(
        kinds,
        literal_count=literal_count,
        referenced_var_names=referenced_var_names,
        resolved_const_count=resolved_const_count,
    )


def _topic_argument_literal(cursor: Any, arg_index: int = 0) -> str | None:
    """Return the name the topic argument at *arg_index* spells, or None.

    Same rules as ``_first_string_literal`` (inline literal first, then a
    resolvable constant, then the ``${var}`` placeholder), but searched only in
    the argument :func:`_classify_topic_argument` classifies. Searching the
    whole call took ``"base"`` from ``get_node("base")`` in a Nav2 wrapper call
    and a log string from a subscription callback.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None
    roots = _topic_argument_roots(cursor)
    if len(roots) <= arg_index:
        return None
    root = roots[arg_index]
    for desc in root.walk_preorder():
        if desc.kind == CursorKind.STRING_LITERAL:
            tokens = _cursor_tokens(desc)
            if tokens:
                raw = tokens[0]
                if raw.startswith('"') and raw.endswith('"'):
                    return raw[1:-1]
                return raw
    first_var_name: str | None = None
    for desc in root.walk_preorder():
        if desc.kind == CursorKind.DECL_REF_EXPR:
            ref = desc.referenced
            if ref is not None and ref.kind == CursorKind.VAR_DECL:
                result = _string_from_var_decl(ref)
                if result is not None:
                    return result
                if first_var_name is None:
                    first_var_name = desc.spelling or None
    if first_var_name:
        return f"${{{first_var_name}}}"
    return None


def _unwrap_expr(cursor: Any) -> Any:
    """Strip implicit casts, parentheses and copy/move constructions around an expression."""
    from clang.cindex import CursorKind
    while True:
        children = list(cursor.get_children())
        if len(children) != 1:
            return cursor
        if cursor.kind in (CursorKind.UNEXPOSED_EXPR, CursorKind.PAREN_EXPR):
            cursor = children[0]
            continue
        ref = cursor.referenced if cursor.kind == CursorKind.CALL_EXPR else None
        if ref is not None and ref.kind == CursorKind.CONSTRUCTOR:
            cursor = children[0]
            continue
        return cursor


def _topic_argument_field(cursor: Any, arg_index: int = 0) -> Any | None:
    """The FIELD_DECL when the topic argument is exactly one member of ``this``, else None.

    ``create_subscription(twin_topic_, ...)`` names a member, so the
    topic is the member's value, which ``_param_backed_fields`` may tie to a
    declared parameter.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None
    roots = _topic_argument_roots(cursor)
    if len(roots) <= arg_index:
        return None
    expr = _unwrap_expr(roots[arg_index])
    ref = expr.referenced if expr.kind == CursorKind.MEMBER_REF_EXPR else None
    if ref is None or ref.kind != CursorKind.FIELD_DECL:
        return None
    base = [_unwrap_expr(c) for c in expr.get_children()]
    if any(b.kind != CursorKind.CXX_THIS_EXPR for b in base):
        return None
    return ref


# Operators on a std::string that leave their left operand unchanged.
_READONLY_OPERATORS = frozenset({
    "operator==", "operator!=", "operator<", "operator>", "operator<=", "operator>=",
    "operator+", "operator<<",
})


def _binds_mutably(callee: Any, index: int) -> bool:
    """Whether argument *index* of a call to *callee* may be written through.

    True for a non-const lvalue or an rvalue reference parameter (a move), and
    whenever the callee or its parameter is unknown.
    """
    from clang.cindex import TypeKind
    if callee is None:
        return True
    params = list(callee.get_arguments())
    if index >= len(params):
        return False  # a C variadic argument is passed by value
    t = params[index].type
    if t.kind == TypeKind.RVALUEREFERENCE:
        return True
    return t.kind == TypeKind.LVALUEREFERENCE and not t.get_pointee().is_const_qualified()


def _field_use(stack: list[Any], ref_cursor: Any) -> tuple[bool, Any | None]:
    """Classify one MEMBER_REF_EXPR of a field: ``(is_write, assigned_expression)``.

    *stack* holds the ancestors of *ref_cursor*, innermost last. Only a use
    proven to leave the member unchanged is a read; every other use, including
    one this function does not recognise (a cast, a conditional operator), is
    a write. Only a plain ``=`` or a member initializer names the assigned
    expression.
    """
    from clang.cindex import CursorKind, TypeKind
    child, i = ref_cursor, len(stack) - 1
    while i >= 0 and stack[i].kind in (CursorKind.UNEXPOSED_EXPR, CursorKind.PAREN_EXPR):
        child, i = stack[i], i - 1
    if i < 0:
        return True, None
    parent = stack[i]
    if parent.kind == CursorKind.CONSTRUCTOR:
        # Member initializer: the MEMBER_REF is followed by its initializer.
        siblings = list(parent.get_children())
        pos = next((n for n, s in enumerate(siblings) if s == child), None)
        if pos is not None and pos + 1 < len(siblings):
            return True, siblings[pos + 1]
        return True, None
    if parent.kind == CursorKind.MEMBER_REF_EXPR:
        method = parent.referenced
        is_const = method is not None and method.kind == CursorKind.CXX_METHOD \
            and method.is_const_method()
        return (not is_const), None
    if parent.kind == CursorKind.CALL_EXPR:
        args = list(parent.get_arguments())
        index = next((n for n, a in enumerate(args) if a == child), None)
        if index is None:
            return True, None
        op = parent.spelling
        if op.startswith("operator"):
            if index == 0:
                if op == "operator=" and len(args) == 2:
                    return True, args[1]
                return op not in _READONLY_OPERATORS, None
            callee = parent.referenced
            if callee is not None and callee.kind == CursorKind.CXX_METHOD:
                index -= 1  # the object is the member operator's implicit argument
            return _binds_mutably(callee, index), None
        return _binds_mutably(parent.referenced, index), None
    by_value = (TypeKind.LVALUEREFERENCE, TypeKind.RVALUEREFERENCE, TypeKind.POINTER)
    if parent.kind == CursorKind.VAR_DECL:
        # A copy is a read; any reference may later be written through (a const_cast).
        return parent.type.kind in by_value, None
    if parent.kind == CursorKind.RETURN_STMT:
        # Returned by value from the method itself (not from a lambda in it): a copy.
        returns_copy = (stack and stack[0].kind == CursorKind.CXX_METHOD
                        and stack[0].result_type.kind not in by_value
                        and not any(c.kind == CursorKind.LAMBDA_EXPR for c in stack))
        return not returns_copy, None
    return True, None


def _declared_string_parameter(expr: Any) -> str | None:
    """The name of ``declare_parameter<std::string>("name", ...)`` when *expr* is exactly that call."""
    from clang.cindex import CursorKind
    call = _unwrap_expr(expr)
    if call.kind != CursorKind.CALL_EXPR or call.spelling != "declare_parameter":
        return None
    name = _first_string_literal(call)
    if not name:
        return None
    type_arg = _get_template_arg_type(call) or ""
    if "string" not in type_arg:
        children = list(call.get_children())
        default = _extract_literal_value(children[2]) if len(children) >= 3 else None
        if not isinstance(default, str):
            return None
    return name


def _cursor_flag(cursor: Any, name: str) -> bool | None:
    """``cursor.<name>()``, or None when this libclang binding lacks the query."""
    query = getattr(cursor, name, None)
    return None if query is None else bool(query())


def _is_std_string_value(field: Any) -> bool:
    """Whether *field* is a non-mutable ``std::string`` held by value.

    The canonical type must be ``std::basic_string<char>`` itself: a reference,
    a pointer, a ``string_view`` or a ``wstring`` is not proven to hold the
    parameter's value. Unknown mutability counts as mutable.
    """
    from clang.cindex import CursorKind, TypeKind
    if _cursor_flag(field, "is_mutable_field") is not False:
        return False
    t = field.type.get_canonical()
    if t.kind != TypeKind.RECORD:
        return False
    decl = t.get_declaration()
    if decl.spelling != "basic_string":
        return False
    scope = decl.semantic_parent
    while scope is not None and scope.kind == CursorKind.NAMESPACE and scope.spelling != "std":
        scope = scope.semantic_parent  # an inline namespace such as __cxx11
    if scope is None or scope.kind != CursorKind.NAMESPACE or scope.spelling != "std":
        return False
    if scope.semantic_parent is not None and \
            scope.semantic_parent.kind != CursorKind.TRANSLATION_UNIT:
        return False
    return t.get_template_argument_type(0).kind in (TypeKind.CHAR_S, TypeKind.CHAR_U)


def _has_unseen_mutator(class_cursor: Any) -> bool:
    """Whether a member function that may write a member has no definition in the TU.

    Const, deleted and pure virtual functions are exempt; any other method,
    constructor, destructor or member template whose body is not visible could
    write the member, so the proof fails.
    """
    from clang.cindex import CursorKind
    kinds = (CursorKind.CXX_METHOD, CursorKind.CONSTRUCTOR, CursorKind.DESTRUCTOR,
             CursorKind.FUNCTION_TEMPLATE, CursorKind.CONVERSION_FUNCTION)
    for c in class_cursor.get_children():
        if c.kind not in kinds:
            continue
        if _cursor_flag(c, "is_deleted_method") or _cursor_flag(c, "is_pure_virtual_method"):
            continue
        if c.kind == CursorKind.CXX_METHOD and _cursor_flag(c, "is_const_method"):
            continue
        if _cursor_flag(c, "is_default_method"):
            continue  # a defaulted function's effect is the language's, checked below for ctors
        if c.get_definition() is None:
            return True
    return False


def _write_runs_on_every_construction(
    class_cursor: Any, ctor: Any, write: Any, stack: list[Any],
) -> bool:
    """Whether the one write in *ctor* runs whenever the class is constructed.

    Every other user-declared constructor must be deleted, or a defaulted copy
    or move constructor (which copies a member that was itself so written); a
    constructor template is not proven. The write must be a member initializer
    or a top-level statement of the body with no ``return`` or ``goto`` before
    it: inside an ``if``, a loop, a ``switch``, a ``try`` or a lambda it may not run.
    """
    from clang.cindex import CursorKind
    ctor_def = ctor.get_definition() or ctor
    for c in class_cursor.get_children():
        if c.kind == CursorKind.FUNCTION_TEMPLATE and c.spelling == class_cursor.spelling:
            return False
        if c.kind != CursorKind.CONSTRUCTOR:
            continue
        if (c.get_definition() or c) == ctor_def:
            continue
        if _cursor_flag(c, "is_deleted_method"):
            continue
        if _cursor_flag(c, "is_default_method") and (
                _cursor_flag(c, "is_copy_constructor") or _cursor_flag(c, "is_move_constructor")):
            continue
        return False
    if stack and stack[-1].kind == CursorKind.CONSTRUCTOR:
        return True  # a member initializer
    # ``member = value;`` directly in the body: ctor, body, [cleanups...], operator=.
    if len(stack) < 3 or stack[1].kind != CursorKind.COMPOUND_STMT:
        return False
    j = 2
    while j < len(stack) and stack[j].kind == CursorKind.UNEXPOSED_EXPR:
        j += 1
    if j >= len(stack) or stack[j].kind != CursorKind.CALL_EXPR or stack[j].spelling != "operator=":
        return False
    if any(c.kind not in (CursorKind.UNEXPOSED_EXPR, CursorKind.PAREN_EXPR)
           for c in stack[j + 1:]):
        return False
    first = stack[2].extent.start.offset
    for c in stack[1].walk_preorder():
        if c.kind in (CursorKind.RETURN_STMT, CursorKind.GOTO_STMT, CursorKind.INDIRECT_GOTO_STMT) \
                and c.extent.start.offset < first:
            return False
    return True


def _param_backed_fields(
    class_cursor: Any, tu_full: Any, fields_by_usr: dict[str, Any],
) -> dict[str, tuple[str, Any, Any]]:
    """The members whose value is a declared string parameter's launch value.

    Maps a field USR to ``(parameter name, write cursor, constructor)`` when the private
    ``std::string`` member has exactly one write in the class's methods, in a
    constructor, and that write assigns ``declare_parameter<std::string>(...)``
    to it. The value is then fixed when the constructor returns: a later
    ``ros2 param set`` changes the parameter, not the member. Any other write
    (a parameter callback, a hook re-reading ``get_parameter``, a non-const
    reference handed out) leaves the member, and so the topic, unresolved.
    """
    try:
        from clang.cindex import AccessSpecifier, CursorKind
    except ImportError:
        return {}
    wanted = {
        usr: f for usr, f in fields_by_usr.items()
        if f.access_specifier == AccessSpecifier.PRIVATE and _is_std_string_value(f)
    }
    if not wanted:
        return {}
    # A friend or a nested class may write the member outside the methods scanned below.
    if any(c.kind in (CursorKind.FRIEND_DECL, CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL,
                      CursorKind.CLASS_TEMPLATE)
           for c in class_cursor.get_children()):
        return {}
    if _has_unseen_mutator(class_cursor):
        return {}
    writes: dict[str, list[tuple[Any, Any, Any, list[Any]]]] = {usr: [] for usr in wanted}
    method_kinds = (CursorKind.CXX_METHOD, CursorKind.CONSTRUCTOR, CursorKind.DESTRUCTOR,
                    CursorKind.FUNCTION_TEMPLATE, CursorKind.CONVERSION_FUNCTION)
    seen: set[tuple[str, int]] = set()
    for method in tu_full.cursor.walk_preorder():
        if method.kind not in method_kinds or not method.is_definition():
            continue
        if method.semantic_parent != class_cursor:
            continue
        key = (str(method.location.file), method.extent.start.offset)
        if key in seen:
            continue
        seen.add(key)
        stack: list[Any] = []

        def visit(c: Any) -> None:
            # A member initializer names the field with a MEMBER_REF under the constructor.
            is_ref = c.kind == CursorKind.MEMBER_REF_EXPR or (
                c.kind == CursorKind.MEMBER_REF and stack and stack[-1].kind == CursorKind.CONSTRUCTOR)
            if is_ref and c.referenced is not None:
                usr = c.referenced.get_usr()
                if usr in writes:
                    is_write, value = _field_use(stack, c)
                    if is_write:
                        writes[usr].append((method, c, value, list(stack)))
            stack.append(c)
            for ch in c.get_children():
                visit(ch)
            stack.pop()

        visit(method)
    result: dict[str, tuple[str, Any, Any]] = {}
    for usr, sites in writes.items():
        if len(sites) != 1:
            continue
        method, write, value, stack = sites[0]
        if method.kind != CursorKind.CONSTRUCTOR or value is None:
            continue
        if not _write_runs_on_every_construction(class_cursor, method, write, stack):
            continue
        name = _declared_string_parameter(value)
        if name:
            result[usr] = (name, write, method)
    return result


def _note_topic_field(endpoint: Any, call_cursor: Any) -> None:
    """Remember the member an unresolved topic argument names, for ``_bind_param_backed_topics``."""
    if endpoint.topic_dynamic and not endpoint.topic:
        field = _topic_argument_field(call_cursor)
        if field is not None:
            endpoint._rcllf_topic_field = (field, call_cursor)


def _bind_param_backed_topics(endpoints: list[Any], class_cursor: Any, tu_full: Any) -> None:
    """Set ``topic_param`` on endpoints whose topic member holds a declared parameter.

    ``graph.hierarchical._resolve_topic_placeholders`` then takes the topic
    from the parameter's launch value. An endpoint made in the constructor
    counts only when it is made after the write, in the same constructor.
    """
    pending = [ep for ep in endpoints if getattr(ep, "_rcllf_topic_field", None) is not None]
    if not pending:
        return
    fields_by_usr = {ep._rcllf_topic_field[0].get_usr(): ep._rcllf_topic_field[0] for ep in pending}
    backed = _param_backed_fields(class_cursor, tu_full, fields_by_usr)
    for ep in pending:
        field, site = ep._rcllf_topic_field
        del ep._rcllf_topic_field
        hit = backed.get(field.get_usr())
        if hit is None:
            continue
        name, write, ctor = hit
        if ep.created_in == "constructor":
            # Made in the constructor: only after the write, in that constructor's body.
            same_file = str(site.location.file) == str(ctor.location.file)
            start, end = ctor.extent.start.offset, ctor.extent.end.offset
            if not same_file or not (
                write.extent.start.offset < site.extent.start.offset <= end
                and start <= site.extent.start.offset
            ):
                continue
        elif ep.created_in is None:
            continue
        ep.topic_param = name


def _string_from_var_decl(var_cursor: Any) -> str | None:
    """Extract a string literal from a VAR_DECL's initializer."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None
    for desc in var_cursor.walk_preorder():
        if desc.kind == CursorKind.STRING_LITERAL:
            tokens = _cursor_tokens(desc)
            if tokens:
                raw = tokens[0]
                if raw.startswith('"') and raw.endswith('"'):
                    return raw[1:-1]
                return raw
    return None


def _extract_literal_value(cursor: Any) -> Any | None:
    """Extract a literal value (string, int, float, bool) from a cursor subtree.

    Walks the subtree looking for STRING_LITERAL, INTEGER_LITERAL,
    FLOATING_LITERAL, or CXX_BOOL_LITERAL_EXPR nodes. A unary ``-`` or ``+``
    on a numeric literal is applied. Any other operator
    expression (binary, conditional, ``~``, ``!``, ``-kConst``) cannot be
    evaluated here and returns None, never the first literal inside it.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None
    for desc in cursor.walk_preorder():
        if desc.kind == CursorKind.UNARY_OPERATOR:
            tokens = _cursor_tokens(desc)
            operand = next(desc.get_children(), None)
            value = _extract_literal_value(operand) if operand is not None else None
            if (
                tokens
                and tokens[0] in ("-", "+")
                and isinstance(value, (int, float))
                and not isinstance(value, bool)
            ):
                return -value if tokens[0] == "-" else value
            return None
        if desc.kind in (CursorKind.BINARY_OPERATOR, CursorKind.CONDITIONAL_OPERATOR):
            return None
        if desc.kind == CursorKind.STRING_LITERAL:
            tokens = _cursor_tokens(desc)
            if tokens:
                raw = tokens[0]
                if raw.startswith('"') and raw.endswith('"'):
                    return raw[1:-1]
                return raw
        if desc.kind == CursorKind.INTEGER_LITERAL:
            tokens = _cursor_tokens(desc)
            if tokens:
                try:
                    return int(tokens[0])
                except ValueError:
                    return tokens[0]
        if desc.kind == CursorKind.FLOATING_LITERAL:
            tokens = _cursor_tokens(desc)
            if tokens:
                try:
                    return float(tokens[0])
                except ValueError:
                    return tokens[0]
        if desc.kind == CursorKind.CXX_BOOL_LITERAL_EXPR:
            tokens = _cursor_tokens(desc)
            if tokens:
                return tokens[0] == "true"
    return None


def _extract_callback_name_from_bind(cursor: Any) -> str | None:
    """Find the method name inside a ``std::bind(&Class::method, ...)`` call."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None

    def _walk(c: Any, depth: int = 0) -> str | None:
        if depth > 12:
            return None
        if c.kind == CursorKind.DECL_REF_EXPR:
            ref = c.referenced
            if ref is not None and ref.kind == CursorKind.CXX_METHOD:
                return ref.spelling
        for child in c.get_children():
            result = _walk(child, depth + 1)
            if result:
                return result
        return None

    return _walk(cursor)


def _callback_form(arg: Any | None) -> tuple[str | None, Any]:
    """Classify a timer or subscription callback argument.

    Returns ``("lambda", cursor)`` for an inline lambda, ``("method", name)``
    for ``std::bind(&Class::method, ...)`` or a bare ``&Class::method``, and
    ``(None, None)`` for anything else. Only the argument itself counts, seen
    through implicit conversions and parentheses: a lambda or a bind nested in
    a conditional, a call or any other expression names a body the analyzer
    cannot pick, so the caller quotes the argument for RCLLF-H007.
    """
    if arg is None:
        return None, None
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None, None

    def _extent(c: Any) -> tuple:
        return (c.extent.start.offset, c.extent.end.offset)

    def _strip(c: Any) -> Any:
        # Implicit conversions, temporaries, copy/move constructions and
        # parentheses wrap the written expression without changing it.
        while True:
            children = list(c.get_children())
            if c.kind in (CursorKind.UNEXPOSED_EXPR, CursorKind.PAREN_EXPR) and len(children) == 1:
                c = children[0]
            elif (c.kind == CursorKind.CALL_EXPR and len(children) == 1
                  and _extent(children[0]) == _extent(c)):
                c = children[0]
            else:
                return c

    def _member_pointer(c: Any) -> str | None:
        c = _strip(c)
        if c.kind == CursorKind.UNARY_OPERATOR:
            children = list(c.get_children())
            if len(children) != 1:
                return None
            c = _strip(children[0])
        if c.kind == CursorKind.DECL_REF_EXPR:
            ref = c.referenced
            if ref is not None and ref.kind == CursorKind.CXX_METHOD:
                return ref.spelling
        return None

    expr = _strip(arg)
    if expr.kind == CursorKind.LAMBDA_EXPR:
        return "lambda", expr
    if expr.kind == CursorKind.CALL_EXPR and expr.spelling == "bind":
        children = list(expr.get_children())
        method = _member_pointer(children[1]) if len(children) >= 2 else None
        return ("method", method) if method else (None, None)
    method = _member_pointer(expr)
    return ("method", method) if method else (None, None)


def _callback_source(arg: Any | None) -> str:
    """Source text of a callback argument the analyzer cannot model (RCLLF-H007)."""
    if arg is None:
        return "(no callback argument found)"
    text = " ".join(_cursor_tokens(arg))
    for spaced, tight in ((" :: ", "::"), (":: ", "::"), (" ::", "::"), ("( ", "("),
                          (" )", ")"), (" ,", ","), ("& ", "&"), (" < ", "<"), (" >", ">")):
        text = text.replace(spaced, tight)
    if not text:
        return "(callback argument without source tokens)"
    return text if len(text) <= 80 else text[:77] + "..."


def _is_member_call(cursor: Any) -> bool:
    """True when the CALL_EXPR's first child is the MEMBER_REF_EXPR of a method."""
    from clang.cindex import CursorKind

    first = next(iter(cursor.get_children()), None)
    return first is not None and first.kind == CursorKind.MEMBER_REF_EXPR


# The node argument of a free rclcpp::create_timer that names the node being
# analyzed: ``this``, ``shared_from_this()`` or its base/timers interface
# (optionally ``.get()`` for the raw-pointer overload), tokens joined.
_OWN_NODE_ARG = re.compile(
    r"this|(?:this->)?(?:shared_from_this|get_node_base_interface|get_node_timers_interface)"
    r"\(\)(?:\.get\(\))?")


def _enclosing_scopes(decl: Any) -> list[str]:
    """Names of the scopes around *decl*, outermost first (``["rclcpp"]``).

    A class or function scope is named ``<kind>:<name>`` so it never equals a
    namespace; an anonymous namespace is ``(anonymous)``.
    """
    scopes: list[str] = []
    parent = decl.semantic_parent
    while parent is not None and parent.kind.name != "TRANSLATION_UNIT":
        name = parent.spelling or "(anonymous)"
        scopes.append(name if parent.kind.name == "NAMESPACE" else f"{parent.kind.name}:{name}")
        parent = parent.semantic_parent
    return scopes[::-1]


def _free_timer_arguments(cursor: Any) -> tuple[dict[str, Any] | None, str]:
    """Arguments of a free ``rclcpp::create_timer`` call by parameter name.

    rclcpp (Jazzy) has three overloads: ``(node, clock, period, callback,
    group, autostart)``, ``(node_base, node_timers, clock, period, callback,
    group, autostart)`` and ``(clock, period, callback, group, node_base,
    node_timers, autostart)``. The callee's parameter names say which argument
    is which, so no position is assumed. Returns ``(args, "")`` with the
    ``period``, ``callback`` and optional ``group`` argument cursors, or
    ``(None, reason)`` when the call cannot be read as a timer of this node.
    """
    callee = cursor.referenced
    params = list(callee.get_arguments()) if callee is not None else []
    args = list(cursor.get_arguments())
    if not params or len(args) > len(params):
        return None, "the callee and its parameter names are not resolved"
    scope = _enclosing_scopes(callee)
    if scope != ["rclcpp"]:
        # A project function that is also spelled create_timer is not rclcpp's
        # timer factory; its arguments say nothing about a timer of this node.
        return None, ("the callee is " + "::".join(scope + ["create_timer"])
                      + ", not rclcpp::create_timer")
    by_name = {p.spelling: a for p, a in zip(params, args)}
    if "period" not in by_name or "callback" not in by_name:
        return None, "the call has no period or callback parameter"
    owners = [by_name[n] for n in ("node", "node_base", "node_timers") if n in by_name]
    if not owners:
        return None, "the call names no node"
    for owner in owners:
        if not _OWN_NODE_ARG.fullmatch("".join(_cursor_tokens(owner))):
            return None, ("its node argument '" + " ".join(_cursor_tokens(owner))
                          + "' is not this node")
    return by_name, ""


def _extract_timer_period(cursor: Any) -> float | None:
    """Extract timer period in ms from a ``create_wall_timer`` CALL_EXPR.

    Handles both combined tokens (``500ms``) and split tokens (``500`` + ``ms``).
    The first argument is the second child of the CALL_EXPR (index 1, after the
    MEMBER_REF_EXPR method reference).
    """
    # Check second child — it's the chrono-duration argument.
    children = list(cursor.get_children())
    if len(children) < 2:
        return None
    return _duration_ms(children[1])


# rclcpp::Duration factories whose one literal argument is in the named unit
# (the free rclcpp::create_timer takes an rclcpp::Duration).
_DURATION_FACTORIES: dict[str, Fraction] = {
    "from_seconds": Fraction(1_000),
    "from_nanoseconds": Fraction(1, 1_000_000),
}
# std::chrono duration types whose constructor takes one integer count in the
# named unit: std::chrono::milliseconds(33) (tf_broadcaster_example's timer).
_CHRONO_TYPES: dict[str, Fraction] = {
    "nanoseconds": _CHRONO_SUFFIXES["ns"],
    "microseconds": _CHRONO_SUFFIXES["us"],
    "milliseconds": _CHRONO_SUFFIXES["ms"],
    "seconds": _CHRONO_SUFFIXES["s"],
    "minutes": _CHRONO_SUFFIXES["min"],
    "hours": _CHRONO_SUFFIXES["h"],
}
_INT_LITERAL = re.compile(r"(\d+)[uUlL]*")
# A decimal integer or floating literal (0.25, 1e-1, .5f, 50000000LL); group 1 is
# the value without its suffix.
_NUMBER_LITERAL = re.compile(r"((?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)[fFlLuU]*")


def _duration_ms(duration_child: Any) -> float | None:
    """Milliseconds of one duration argument: a chrono literal or an rclcpp::Duration of one."""
    tokens = _cursor_tokens(duration_child)
    if not tokens:
        return None
    # rclcpp::Duration::from_seconds(0.05) / from_nanoseconds(50000000): exactly
    # one numeric literal between the parentheses, nothing else.
    for i, tok in enumerate(tokens):
        mult = _DURATION_FACTORIES.get(tok)
        if mult is None:
            continue
        inner = tokens[i + 1:]
        lit = _NUMBER_LITERAL.fullmatch(inner[1]) if len(inner) == 3 else None
        if lit is not None and inner[0] == "(" and inner[2] == ")":
            return _chrono_ms(lit.group(1), mult)
        return None
    # std::chrono::milliseconds(33): the whole argument is one duration
    # constructor of one integer literal.
    if (len(tokens) >= 4 and tokens[-4] in _CHRONO_TYPES and tokens[-3] == "("
            and tokens[-1] == ")"):
        prefix = tokens[:-4]
        lit = _INT_LITERAL.fullmatch(tokens[-2])
        if lit is not None and prefix in ([], ["std", "::", "chrono", "::"],
                                          ["chrono", "::"]):
            return _chrono_ms(lit.group(1), _CHRONO_TYPES[tokens[-4]])
    # Combined token like "500ms" or "1s"
    for tok in tokens:
        for suffix, mult in _CHRONO_SUFFIXES.items():
            if tok.endswith(suffix) and tok[:-len(suffix)].replace(".", "", 1).isdigit():
                ms = _chrono_ms(tok[:-len(suffix)], mult)
                if ms is not None:
                    return ms
    # Separate tokens: ["500", "ms"]
    return _parse_chrono_ms(tokens[:2])


def _get_template_arg_type(cursor: Any, index: int = 0) -> str | None:
    """Return the spelling of template argument *index* on a CALL_EXPR cursor.

    Prefers the canonical (typedef-resolved, fully-qualified) spelling when it
    contains namespace separators (``::``), which gives e.g.
    ``custom_interfaces::action::Analyze`` instead of a short alias ``Analyze``.
    """
    try:
        n = cursor.get_num_template_arguments()
        if n is not None and n > index:
            t = cursor.get_template_argument_type(index)
            if t is not None:
                spelling = t.spelling
                if spelling and spelling != "void":
                    # If already fully-qualified, return as-is.
                    if "::" in spelling:
                        return spelling
                    # Otherwise try canonical type to resolve using-aliases
                    # (e.g. ``using Analyze = ...::action::Analyze``).
                    try:
                        canonical = t.get_canonical().spelling
                        if canonical and "::" in canonical:
                            # Strip allocator noise
                            # (e.g. Message4kb_<std::allocator<void>>)
                            clean = re.sub(
                                r"_<std::allocator<void>>", "", canonical,
                            )
                            return clean
                    except Exception:
                        pass
                    return spelling
    except Exception:
        pass
    # Fallback: scan token stream for '<type>' pattern
    tokens = _cursor_tokens(cursor)
    try:
        lt = tokens.index("<")
        gt = next(
            i for i in range(lt + 1, len(tokens)) if tokens[i] == ">"
        )
        inner = "".join(tokens[lt + 1: gt]).strip()
        if inner:
            return inner
    except (ValueError, StopIteration):
        pass
    return None


# Named QoS profiles recognized by ``_extract_qos_attrs`` (Phase 5 hardening).
# Each entry is
# the materialized attrs a bare use of that profile name implies. Depth for
# ``rclcpp::SensorDataQoS()`` / ``rmw_qos_profile_sensor_data`` is 5 (the
# actual rclcpp/rmw default; verify against the
# installed rclcpp when running inside the dev container, not just this
# table).
_NAMED_QOS_PROFILES: dict[str, dict] = {
    "SensorDataQoS": {
        "reliability": "best_effort",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 5,
    },
    "rmw_qos_profile_sensor_data": {
        "reliability": "best_effort",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 5,
    },
    "SystemDefaultsQoS": {
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 10,
    },
}


def _named_qos_profile(tokens: list[str]) -> dict | None:
    """Return the materialized attrs for a recognized named QoS profile, if any.

    Matches whole identifier tokens only, so neither a string
    literal nor a longer identifier that contains a profile name counts.
    """
    for name, attrs in _NAMED_QOS_PROFILES.items():
        if name in tokens:
            return attrs
    return None


# QoS keywords recognized by ``_extract_qos_attrs``. A value is
# set only by a chained builder call (``.best_effort()``, ``->reliable()``) or
# by a whole enum token. String literals and identifiers that merely contain
# one of these words never count.
_QOS_CHAINED_CALLS: dict[str, tuple[str, str]] = {
    "best_effort": ("reliability", "best_effort"),
    "reliable": ("reliability", "reliable"),
    "transient_local": ("durability", "transient_local"),
    "durability_volatile": ("durability", "volatile"),
    "keep_all": ("history", "keep_all"),
    "keep_last": ("history", "keep_last"),
}
_QOS_ENUM_TOKENS: dict[str, tuple[str, str]] = {
    "BEST_EFFORT": ("reliability", "best_effort"),
    "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT": ("reliability", "best_effort"),
    "RELIABLE": ("reliability", "reliable"),
    "RMW_QOS_POLICY_RELIABILITY_RELIABLE": ("reliability", "reliable"),
    "TRANSIENT_LOCAL": ("durability", "transient_local"),
    "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL": ("durability", "transient_local"),
    "VOLATILE": ("durability", "volatile"),
    "RMW_QOS_POLICY_DURABILITY_VOLATILE": ("durability", "volatile"),
    "KEEP_ALL": ("history", "keep_all"),
    "RMW_QOS_POLICY_HISTORY_KEEP_ALL": ("history", "keep_all"),
    "KEEP_LAST": ("history", "keep_last"),
    "RMW_QOS_POLICY_HISTORY_KEEP_LAST": ("history", "keep_last"),
}
# Precedence when both values of a field appear (kept from the substring
# matcher): best_effort over reliable, transient_local over volatile,
# keep_all over keep_last.
_QOS_PRECEDENCE: dict[str, tuple[str, str]] = {
    "reliability": ("best_effort", "reliable"),
    "durability": ("transient_local", "volatile"),
    "history": ("keep_all", "keep_last"),
}
_STRING_LITERAL_RE = re.compile(r"""^(?:u8|u|U|L)?R?["']""")


def _qos_keyword_values(tokens: list[str]) -> dict[str, set[str]]:
    """Collect the QoS values each field is set to by chained calls or enum tokens."""
    found: dict[str, set[str]] = {}
    for i, tok in enumerate(tokens):
        hit = _QOS_ENUM_TOKENS.get(tok)
        if hit is None and i > 0 and tokens[i - 1] in (".", "->"):
            hit = _QOS_CHAINED_CALLS.get(tok)
        if hit is not None:
            found.setdefault(hit[0], set()).add(hit[1])
    return found


# Calls that only read a QoS object passed to them (``const rclcpp::QoS &``).
# ``subscribe`` is ``message_filters::Subscriber::subscribe``, which takes its
# QoS by value or by const reference; a ``subscribe`` declared
# outside namespace ``message_filters`` may take ``rmw_qos_profile_t &`` and
# change it, so only the message_filters one counts (``_is_reading_call``).
_QOS_READING_CALLS = frozenset({"create_publisher", "create_subscription", "subscribe"})
_QOS_NAMESPACED_READS = {"subscribe": "message_filters"}


def _is_reading_call(call: Any) -> bool:
    """True iff the CALL_EXPR *call* is a QoS-reading call (``_QOS_READING_CALLS``).

    A name in ``_QOS_NAMESPACED_READS`` counts only when the called declaration
    sits inside that namespace; an unresolved callee does not count.
    """
    if call.spelling not in _QOS_READING_CALLS:
        return False
    ns = _QOS_NAMESPACED_READS.get(call.spelling)
    if ns is None:
        return True
    from clang.cindex import CursorKind
    decl = call.referenced
    while decl is not None and decl.kind != CursorKind.TRANSLATION_UNIT:
        if decl.kind == CursorKind.NAMESPACE and decl.spelling == ns:
            return True
        decl = decl.semantic_parent
    return False


# ``rclcpp::QoS::get_rmw_qos_profile()`` hands out the profile the QoS object
# holds. Its non-const overload returns a reference, so the call
# is a read only when its result is itself read: the ancestor above the call
# decides, as for a plain reference to the variable.
_QOS_PROFILE_GETTERS = frozenset({"get_rmw_qos_profile"})
# One-argument constructor calls that copy a QoS: ``rclcpp::QoS(qos)`` and the
# implicit ``rmw_qos_profile_t`` copy of ``qos.get_rmw_qos_profile()`` passed
# by value. The copy's own use decides.
_QOS_COPY_TYPES = frozenset({"QoS", "rmw_qos_profile_s"})


def _qos_ref_target(expr: Any) -> Any | None:
    """The local ``VarDecl`` a QoS argument names, or None.

    Walks down single-child wrappers (implicit casts, the copy constructor
    ``QoS(qos)``) to a ``DECL_REF_EXPR`` and returns its referenced
    declaration when that is a variable declared inside a function body.
    """
    from clang.cindex import CursorKind
    node = expr
    for _ in range(8):
        if node.kind == CursorKind.DECL_REF_EXPR:
            ref = node.referenced
            if ref is None or ref.kind != CursorKind.VAR_DECL:
                return None
            parent = ref.semantic_parent
            if parent is None or parent.kind not in (
                    CursorKind.CONSTRUCTOR, CursorKind.CXX_METHOD, CursorKind.FUNCTION_DECL,
                    CursorKind.FUNCTION_TEMPLATE, CursorKind.LAMBDA_EXPR):
                return None
            return ref
        children = list(node.get_children())
        if len(children) != 1:
            return None
        node = children[0]
    return None


def _var_initializer(var: Any) -> Any | None:
    """The initializer expression of a ``VarDecl`` (its last expression child)."""
    exprs = [c for c in var.get_children() if c.kind.is_expression()]
    return exprs[-1] if exprs else None


def _uses_are_reads(var: Any) -> bool:
    """True iff every reference to the local *var* in its function only reads it.

    A read is an argument of a QoS-reading call (``create_publisher`` /
    ``create_subscription``) or the initializer of another variable held by
    value (a copy). Anything else (a chained ``qos.reliable()`` statement, an
    assignment, a pass to an unknown function, a reference or pointer alias
    such as ``auto & alias = qos;``) may change the profile after its
    declaration, so the variable is not followed.
    """
    from clang.cindex import CursorKind, TypeKind
    aliases = (TypeKind.LVALUEREFERENCE, TypeKind.RVALUEREFERENCE, TypeKind.POINTER)
    func = var.semantic_parent
    ok = True

    def visit(node: Any, chain: list) -> None:
        nonlocal ok
        if not ok:
            return
        if node.kind == CursorKind.DECL_REF_EXPR and node.referenced is not None \
                and node.referenced == var:
            # The nearest ancestor that is not a single-child wrapper decides.
            for anc in reversed(chain):
                if anc.kind in (CursorKind.UNEXPOSED_EXPR, CursorKind.PAREN_EXPR):
                    continue
                if anc.kind == CursorKind.CALL_EXPR and anc.spelling in _QOS_COPY_TYPES \
                        and len(list(anc.get_children())) == 1:
                    continue  # copy constructor
                if anc.kind in (CursorKind.MEMBER_REF_EXPR, CursorKind.CALL_EXPR) \
                        and anc.spelling in _QOS_PROFILE_GETTERS \
                        and len(list(anc.get_children())) == 1:
                    continue  # qos.get_rmw_qos_profile(): its use decides
                if anc.kind == CursorKind.CALL_EXPR and _is_reading_call(anc):
                    return
                if anc.kind == CursorKind.VAR_DECL and anc.type.kind not in aliases:
                    return
                ok = False
                return
            ok = False
            return
        chain.append(node)
        for child in node.get_children():
            visit(child, chain)
        chain.pop()

    visit(func, [])
    return ok


def _qos_argument(cursor: Any, index: int = 2) -> Any | None:
    """The QoS argument of a create_publisher/create_subscription call (children[2]).

    *index* is the argument's child index: 3 for a message_filters
    ``member.subscribe(this, "topic", qos, ...)`` call.

    A local variable (``auto qos = rclcpp::QoS(...).reliable(); ...
    create_publisher<T>("t", qos)``) resolves to its initializer, following
    copies (``auto qos_a = qos;``) to the first initializer that names no
    variable, but only when no statement in the function can change any link
    after its declaration (``_uses_are_reads``). Otherwise, and on a cycle
    (``rclcpp::QoS q = q;``), the current expression is returned, which stays
    unresolved (RCLLF-H000).
    """
    children = list(cursor.get_children())
    if len(children) <= index:
        return None
    expr = children[index]
    seen: set = set()
    while True:
        try:
            var = _qos_ref_target(expr)
            if var is None or var.hash in seen or not _uses_are_reads(var):
                return expr
            seen.add(var.hash)
            init = _var_initializer(var)
        except (AttributeError, ValueError):
            # Following is best effort: a cursor it cannot walk keeps the
            # expression reached so far, so an inline QoS is never lost.
            return expr
        if init is None:
            return expr
        expr = init


def _mf_subscribe_call(tu: Any, class_cursor: Any, files: set[str], member: str,
                       topic: str, qos_args: str) -> Any | None:
    """The AST call ``member.subscribe(this, "topic", qos, ...)`` of a message_filters input.

    The source scan (``message_filters_scan``) finds the input,
    this finds the same call in the AST so its QoS argument (child 3, after
    the callee, ``this`` and the topic) resolves like a create_subscription
    QoS, local variables included. Only a call inside a method or constructor
    of *class_cursor* whose QoS argument has the scanned spelling counts, and
    only when exactly one does. Otherwise (the AST degrades without the
    message_filters headers) None, and the caller keeps the scan's reading.
    Only top-level declarations from *files* (the node's source and header)
    are walked, not the included rclcpp headers.
    """
    from clang.cindex import CursorKind
    from colcon_lf.node_analysis.message_filters_scan import _norm
    want = _norm(qos_args)
    found = []
    # libclang gives an expression no semantic parent, so the walk starts at
    # the class's own constructors and methods (their definitions).
    funcs = [f for top in tu.cursor.get_children()
             if top.location.file is not None
             and os.path.realpath(str(top.location.file)) in files
             for f in top.walk_preorder()
             if f.kind in (CursorKind.CONSTRUCTOR, CursorKind.CXX_METHOD)
             and f.is_definition() and f.semantic_parent == class_cursor]
    calls = (cur for f in funcs for cur in f.walk_preorder())
    for cur in calls:
        if cur.kind != CursorKind.CALL_EXPR or cur.spelling != "subscribe":
            continue
        children = list(cur.get_children())
        if len(children) < 4 or children[0].kind != CursorKind.MEMBER_REF_EXPR:
            continue
        base = list(children[0].get_children())
        if len(base) != 1 or base[0].spelling != member:
            continue
        if f'"{topic}"' not in _cursor_tokens(children[2]):
            continue
        if _norm("".join(_cursor_tokens(children[3]))) != want:
            continue
        found.append(cur)
    return found[0] if len(found) == 1 else None


def _mf_input_qos(tu: Any, class_cursor: Any, files: set[str], inp: Any) -> dict:
    """QoS fields of one message_filters input.

    The scan's text reading (``qos_from_args``) covers an absent argument
    (``rmw_qos_profile_default``) and the literal forms. A QoS argument it
    cannot read, such as a local ``rclcpp::QoS`` variable passed as
    ``qos.get_rmw_qos_profile()``, resolves through the create_subscription
    helpers on the AST call (``_extract_qos_depth`` / ``_extract_qos_attrs``,
    argument index 3). A field neither resolves stays None: unknown, never a
    silent default (RCLLF-H000/H001).
    """
    from colcon_lf.node_analysis import message_filters_scan
    text = message_filters_scan.qos_from_args(inp.qos_args)
    fields = {**text, "qos_deadline_ms": None}
    if inp.qos_args is None:
        return fields
    try:
        call = _mf_subscribe_call(tu, class_cursor, files, inp.member, inp.topic,
                                  inp.qos_args)
    except Exception:
        call = None
    if call is None:
        return fields
    depth = _extract_qos_depth(call, 3)
    attrs = _materialize_qos_defaults(depth, _extract_qos_attrs(call, 3))
    fields["qos_deadline_ms"] = attrs["deadline_ms"]
    if depth is not None or text["qos_depth"] is None:
        fields.update(qos_depth=depth, qos_reliability=attrs["reliability"],
                      qos_durability=attrs["durability"], qos_history=attrs["history"])
    return fields


def _extract_keep_last_depth(qos_child: Any) -> int | None:
    """Return the argument of a chained ``.keep_last(n)`` call, if present.

    ``.keep_last(n)`` is the most specific depth statement in a QoS builder
    chain — it must win over the base ``QoS(depth)`` /
    ``KeepLast(m)`` constructor's own literal. Only the call's own argument
    subtree is searched (``get_children()[1:]``, skipping the
    ``MemberRefExpr`` callee at index 0) so the base constructor's literal
    nested inside the callee's receiver expression is never picked up by
    mistake.
    """
    from clang.cindex import CursorKind
    for desc in qos_child.walk_preorder():
        if desc.kind != CursorKind.CALL_EXPR or desc.spelling != "keep_last":
            continue
        call_children = list(desc.get_children())
        arg_roots = call_children[1:] if len(call_children) > 1 else call_children
        for root in arg_roots:
            for sub in root.walk_preorder():
                if sub.kind == CursorKind.INTEGER_LITERAL:
                    toks = _cursor_tokens(sub)
                    if toks:
                        try:
                            return int(toks[0])
                        except ValueError:
                            pass
        # A `.keep_last(...)` call was found but its argument literal could
        # not be read (e.g. a non-literal expression) — stop looking at
        # further `.keep_last()` calls but let the caller fall back to the
        # base-constructor search below rather than reporting no depth.
        break
    return None


def _extract_qos_depth(cursor: Any, index: int = 2) -> int | None:
    """Extract queue depth from the QoS argument of create_publisher/create_subscription.

    The QoS argument is children[2] (after the implicit self and topic), a
    local QoS variable resolved to its initializer (``_qos_argument``).
    Handles the bare-integer case: ``create_publisher<T>("topic", 10)`` and
    named-profile recognition (``rclcpp::SensorDataQoS()`` etc., Phase 5
    hardening) — a named profile with no literal integer in its token stream
    still materializes a depth from ``_NAMED_QOS_PROFILES``. A chained
    ``.keep_last(n)`` call overrides the base constructor's
    depth, matching real ``rclcpp::QoS`` semantics.
    """
    try:
        from clang.cindex import CursorKind
        qos_child = _qos_argument(cursor, index)
        if qos_child is None:
            return None

        keep_last_depth = _extract_keep_last_depth(qos_child)
        if keep_last_depth is not None:
            return keep_last_depth

        for desc in qos_child.walk_preorder():
            if desc.kind == CursorKind.INTEGER_LITERAL:
                toks = _cursor_tokens(desc)
                if toks:
                    try:
                        return int(toks[0])
                    except ValueError:
                        pass
        # No integer literal (e.g. a bare named-profile expression) — fall
        # back to the named-profile table.
        tokens = [t for t in _cursor_tokens(qos_child) if not _STRING_LITERAL_RE.match(t)]
        profile = _named_qos_profile(tokens)
        if profile is not None:
            return profile.get("depth")
    except Exception:
        pass
    return None


def _extract_qos_attrs(cursor: Any, index: int = 2) -> dict:
    """Extract QoS reliability/durability/history/deadline from a create_publisher/subscription call.

    Scans the token stream of the QoS argument (children[2]) for known method names
    chained on a ``rclcpp::QoS`` object, and recognizes named QoS profiles
    (``rclcpp::SensorDataQoS()``, ``SystemDefaultsQoS()``,
    ``rmw_qos_profile_sensor_data``, Phase 5 hardening) so a
    profile expression materializes attrs even with no chained method calls.
    Returns a dict with keys: ``reliability``, ``durability``, ``history``,
    ``deadline_ms`` (all may be None if genuinely unparseable).
    """
    result: dict = {"reliability": None, "durability": None, "history": None, "deadline_ms": None}
    try:
        qos_child = _qos_argument(cursor, index)
        if qos_child is None:
            return result
        # String literals never carry a QoS setting: a topic
        # name like "best_effort_topic" passed to a QoS helper is not one.
        tokens = [t for t in _cursor_tokens(qos_child) if not _STRING_LITERAL_RE.match(t)]
        token_str = " ".join(tokens)

        # Named-profile recognition first: a bare `rclcpp::SensorDataQoS()`
        # (no chained .best_effort()/.keep_last() calls) still materializes
        # attrs from the table. Chained method calls below can still override
        # individual fields (e.g. `rclcpp::SensorDataQoS().reliable()`).
        profile = _named_qos_profile(tokens)
        if profile is not None:
            result["reliability"] = profile.get("reliability")
            result["durability"] = profile.get("durability")
            result["history"] = profile.get("history")

        # Reliability, durability, history from chained calls and enum tokens
        found = _qos_keyword_values(tokens)
        for field, (first, second) in _QOS_PRECEDENCE.items():
            values = found.get(field, set())
            if first in values:
                result[field] = first
            elif second in values:
                result[field] = second

        # Deadline: look for .deadline(std::chrono::...) argument
        dl_match = re.search(r"deadline\s*\(\s*(\d+)\s*(ms|us|ns|s)\b", token_str)
        if dl_match:
            # A QoS deadline is not matched by its exact ns, nearest float is fine.
            result["deadline_ms"] = _chrono_ms(
                dl_match.group(1), _CHRONO_SUFFIXES[dl_match.group(2)], exact=False)
    except Exception:
        pass
    return result


def _materialize_qos_defaults(depth: int | None, attrs: dict) -> dict:
    """Materialize ROS2 defaults for a bare-int (or otherwise-unqualified) QoS argument.

    Phase 5 hardening: ``create_subscription<T>("t", 10, cb)``'s
    QoS argument token stream is just ``10`` — no chained method calls and no
    named profile — so *every* field in ``attrs`` stays ``None`` even though
    ``rclcpp::QoS(10)`` unambiguously means the ROS2 default profile:
    reliable / volatile / keep_last. Without this step ``None`` conflates
    "materialized reliable" with "could not parse" — the exact tri-state bug
    RCLLF-H000 exists to make visible. Only fills fields that are still
    ``None`` after ``_extract_qos_attrs`` — an explicit ``.best_effort()`` (or
    a named profile) is never overridden.

    Only applies when *depth* was resolved (i.e. the QoS argument was
    recognized as *some* QoS expression, not just entirely unparseable) —
    an unresolved depth alongside all-None attrs means the QoS argument
    itself could not be understood at all, which should stay ``None``
    (genuinely unknown, RCLLF-H000).
    """
    if depth is None:
        return attrs
    materialized = dict(attrs)
    if materialized.get("reliability") is None:
        materialized["reliability"] = "reliable"
    if materialized.get("durability") is None:
        materialized["durability"] = "volatile"
    if materialized.get("history") is None:
        materialized["history"] = "keep_last"
    return materialized


def _extract_timer_offset(cursor: Any) -> float:
    """Start offset of a timer CALL_EXPR in ms: always 0.0.

    rclcpp has no offset argument. ``create_wall_timer(period, callback, group,
    autostart)``, ``Node::create_timer(period, callback, group)`` and the free
    ``rclcpp::create_timer(node, clock, period, callback, group, autostart)``
    all start the timer at creation, so no source states an offset. A start
    offset is set in ``execution.yaml`` instead,
    ``nodes.<node>.timers.<timer>.offset``.
    """
    return 0.0


# A wait for the service response after async_send_request in the same method.
# Executor spins run the response callback in place, a
# member wait()/wait_for()/wait_until() is a condition variable or a future,
# and a sleep inside a loop polls a flag the response callback sets.
_SPIN_WAIT_CALLS = {"spin_some", "spin_once", "spin_all", "spin_until_future_complete"}
_MEMBER_WAIT_CALLS = {"wait", "wait_for", "wait_until"}
_SLEEP_CALLS = {"sleep_for", "sleep_until"}
_LOOP_KEYWORDS = {"while", "for", "do"}


def _waits_after_send(tokens: list[str]) -> bool:
    """True when a method waits for a response after its ``async_send_request`` call.

    Only the tokens after the call's closing parenthesis count, so a response
    callback passed as an argument (which may call ``future.get()``) is skipped.
    """
    try:
        start = tokens.index("async_send_request")
    except ValueError:
        return False
    depth = 0
    end = len(tokens)
    for i in range(start + 1, len(tokens)):
        if tokens[i] == "(":
            depth += 1
        elif tokens[i] == ")":
            depth -= 1
            if depth == 0:
                end = i
                break
    tail = tokens[end + 1:]
    called = {
        (tail[j - 1] if j > 0 else "", t)
        for j, t in enumerate(tail[:-1]) if tail[j + 1] == "("
    }
    names = {name for _, name in called}
    if names & _SPIN_WAIT_CALLS:
        return True
    if any(prev in (".", "->") and name in _MEMBER_WAIT_CALLS for prev, name in called):
        return True
    return bool(names & _SLEEP_CALLS and set(tail) & _LOOP_KEYWORDS)


def _classify_service_client_call(method_cursor: Any) -> str | None:
    """Classify the call style of a service client by scanning the enclosing method body.

    Scans the full token stream of the method cursor for patterns following
    ``async_send_request``:
    - ``spin_until_future_complete`` → "blocking_spin"
    - A lambda or function pointer as second arg to async_send_request → "async_callback",
      unless the method then waits for the response → "blocking_wait"
    - ``.wait (`` or ``.wait_for (`` or ``.get (`` on a future variable → "blocking_future"
    - Without a callback argument, a spin, member wait or sleep loop after the call
      → "blocking_wait"
    - Otherwise → None (undetermined)
    """
    try:
        tokens = _cursor_tokens(method_cursor)
        token_str = " ".join(tokens)

        if "spin_until_future_complete" in token_str:
            return "blocking_spin"

        # Check async_callback first: async_send_request with a lambda/bind as second arg.
        # Must precede the blocking_future check because a lambda body may contain .get()
        # on the SharedFuture arg, which would otherwise trigger a false blocking_future.
        async_match = re.search(r"async_send_request\s*\(", token_str)
        if async_match:
            rest = token_str[async_match.end():]
            if re.search(r",\s*(?:\[|\bstd\s*::\s*bind\b|\bstd\s*::\s*function\b)", rest):
                return "blocking_wait" if _waits_after_send(tokens) else "async_callback"

        # Heuristic: look for future variable followed by .wait( / .wait_for( / .get(
        if re.search(r"\.\s*(?:wait|wait_for|get)\s*\(", token_str):
            return "blocking_future"

        if async_match and _waits_after_send(tokens):
            return "blocking_wait"

    except Exception:
        pass
    return None


def _extract_trailing_callback_group(
    cursor: Any,
    cbg_var_names: dict[str, str],
) -> str | None:
    """Return the resolved callback-group name if any token in *cursor* matches
    a known group variable.

    Uses the same token-scan strategy as the create_wall_timer branch: walk
    all tokens in the call expression and return the first cbg_var_names match.
    Returns None when no group variable is present (meaning the node's default
    callback group applies — the same convention used for TimerInfo).
    """
    if not cbg_var_names:
        return None
    for tok in _cursor_tokens(cursor):
        if tok in cbg_var_names:
            return cbg_var_names[tok]
    return None


def _extract_enum_arg(cursor: Any) -> str | None:
    """Extract a trailing enum identifier from a call like ``create_callback_group(Type::Value)``."""
    tokens = _cursor_tokens(cursor)
    # Tokens look like: create_callback_group ( rclcpp :: CallbackGroupType :: MutuallyExclusive )
    # We want the last identifier before the closing paren.
    try:
        paren_close = len(tokens) - 1 - tokens[::-1].index(")")
        for i in range(paren_close - 1, -1, -1):
            t = tokens[i]
            if t.isidentifier() and t not in ("rclcpp", "CallbackGroupType"):
                return t
    except ValueError:
        pass
    return None


def _build_options_map(constructor_cursor: Any) -> dict[str, str]:
    """Scan the constructor body for SubscriptionOptions/PublisherOptions assignments.

    Detects the pattern::

        sub_options.callback_group = exclusive_group_;

    Returns a mapping ``{options_var_name: cbg_member_spelling}``, e.g.
    ``{"sub_options": "exclusive_group_"}``.
    """
    result: dict[str, str] = {}
    for cur in constructor_cursor.walk_preorder():
        tokens = [t.spelling for t in cur.get_tokens()]
        if "callback_group" not in tokens or "=" not in tokens:
            continue
        try:
            eq_idx = tokens.index("=")
        except ValueError:
            continue
        # Expect layout:  options_var  .  callback_group  =  group_member
        if (
            eq_idx >= 3
            and tokens[eq_idx - 1] == "callback_group"
            and tokens[eq_idx - 2] in (".", "->")
            and eq_idx + 1 < len(tokens)
        ):
            options_var = tokens[eq_idx - 3]
            group_member = tokens[eq_idx + 1].rstrip(";")
            result[options_var] = group_member
    return result


# ---------------------------------------------------------------------------
# Include-directive scan (Phase 5: RCLLF-H003 message_filters detection)
# ---------------------------------------------------------------------------

# v1 allowlist for the include scan (00_design.md section 3.1).
# Extend this set, not the scan mechanism, when a new source-level hazard
# needs an include-based signal. "tf2_ros" (T5) covers
# tf2_ros::Buffer/TransformListener (both spelled with the tf2_ros
# namespace token, matching either as an #include spelling or a bare
# token/namespace-qualified reference) and, transitively, lookupTransform/
# canTransform calls made through such a buffer.
_DETECTED_INCLUDES_ALLOWLIST = ("message_filters", "tf2_ros")

# Member calls that publish through a publisher member: rclcpp's publish and a
# tf2_ros broadcaster's sendTransform, whose member the tf2 scan
# maps to /tf or /tf_static in publisher_var_map.
_PUBLISH_CALL_NAMES = ("publish", "sendTransform")


def _scan_detected_includes(tu: Any) -> list[str]:
    """Scan include directives + raw tokens for allowlisted spellings.

    Deliberately an include-directive + token/spelling scan, *not* AST
    instantiation matching: ``message_filters::Subscriber<M>``
    is a VAR_DECL/constructor call the constructor walker never visits (it
    dispatches on ``create_subscription`` spellings only), and — more
    importantly — missing third-party headers (the common case; message_filters
    is rarely on this machine's include path) leave the type unresolved, so
    AST-based matching degrades to nothing exactly when it's needed most.
    ``CursorKind.INCLUSION_DIRECTIVE`` cursors are available because
    ``parse_node`` parses with ``PARSE_DETAILED_PROCESSING_RECORD``.

    Returns the sorted, deduplicated subset of ``_DETECTED_INCLUDES_ALLOWLIST``
    found either as an ``#include`` spelling or as a bare token/spelling
    anywhere in the translation unit (covers both
    ``#include "message_filters/subscriber.h"`` and any stray textual
    reference, e.g. a fully-qualified ``message_filters::Subscriber`` even if
    the header itself failed to resolve).
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []

    found: set[str] = set()

    try:
        for cursor in tu.cursor.walk_preorder():
            if cursor.kind != CursorKind.INCLUSION_DIRECTIVE:
                continue
            spelling = (cursor.spelling or "").lower()
            for entry in _DETECTED_INCLUDES_ALLOWLIST:
                if entry in spelling:
                    found.add(entry)
    except Exception:
        pass

    # Fallback / supplement: raw token scan across the whole file. This is
    # robust to unresolved headers (the include directive may still be
    # present in the token stream even if libclang can't attach it as an
    # INCLUSION_DIRECTIVE cursor in every configuration) and to
    # namespace-qualified spellings appearing in code even without a
    # resolvable include.
    try:
        for tok in tu.cursor.get_tokens():
            spelling = tok.spelling
            for entry in _DETECTED_INCLUDES_ALLOWLIST:
                if entry in spelling:
                    found.add(entry)
    except Exception:
        pass

    return sorted(found)


# ---------------------------------------------------------------------------
# Explicit take() call scan (T5: RCLLF-R002 detection)
# ---------------------------------------------------------------------------

# Subscription-side pull methods (`rclcpp::SubscriptionBase`): explicit take
# bypasses the callback-delivered path LF's runtime binding assumes.
_TAKE_CALL_NAMES = ("take", "take_serialized", "take_loaned")


def _scan_take_calls(tu: Any) -> list[str]:
    """Narrow member-call token scan for explicit ``take``/``take_serialized``/``take_loaned``.

    Same rationale as ``_scan_detected_includes`` (a token scan, not AST
    instantiation matching) but *narrower*: unlike an include
    spelling, "take" alone is a common identifier (a local variable, a
    parameter name) and a bare-token scan would false-positive constantly.
    This requires the member-call shape ``. take (`` / ``-> take (`` in the
    raw token stream, which is specific to a call on a subscription handle
    (``sub_->take(...)``) without needing the subscription's static type to
    resolve (robust to the same missing-header cases the include scan
    guards against).
    """
    found: set[str] = set()
    try:
        tokens = [t.spelling for t in tu.cursor.get_tokens()]
    except Exception:
        return []

    n = len(tokens)
    for i, tok in enumerate(tokens):
        if tok not in _TAKE_CALL_NAMES:
            continue
        if i < 1 or i + 1 >= n:
            continue
        if tokens[i - 1] not in (".", "->"):
            continue
        if tokens[i + 1] != "(":
            continue
        found.add(tok)

    return sorted(found)


# ---------------------------------------------------------------------------
# ROS-time sleep scan (RCLLF-H008 detection)
# ---------------------------------------------------------------------------

_SLEEP_CALL_NAMES = ("sleep_for", "sleep_until")


def _scan_ros_sleep_calls(tu: Any) -> list[str]:
    """Token scan for ``get_clock ( ) -> sleep_for|sleep_until (``.

    The node's own clock (``get_clock()``) follows LF logical time under
    rcllf, so a sleep on it inside a reaction never returns. The shape is
    specific to that clock: a sleep on a member ``rclcpp::Clock`` (a steady
    clock, say) does not match, and the runtime clock gate still refuses a
    ROS-time sleep the scan misses. Returns ``"<call>:<line>"`` entries.
    """
    found: set[str] = set()
    try:
        tokens = list(tu.cursor.get_tokens())
    except Exception:
        return []

    spell = [t.spelling for t in tokens]
    n = len(spell)
    for i, tok in enumerate(spell):
        if tok not in _SLEEP_CALL_NAMES or i < 4 or i + 1 >= n:
            continue
        if spell[i - 4 : i] != ["get_clock", "(", ")", "->"] or spell[i + 1] != "(":
            continue
        found.add(f"{tok}:{tokens[i].location.line}")

    return sorted(found, key=lambda s: (int(s.split(":")[1]), s))


# ---------------------------------------------------------------------------
# Component macro detection
# ---------------------------------------------------------------------------

def _extract_component_class(tu: Any) -> str | None:
    """Return the class name from ``RCLCPP_COMPONENTS_REGISTER_NODE(ClassName)``."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None

    for cursor in tu.cursor.walk_preorder():
        if cursor.kind == CursorKind.MACRO_INSTANTIATION:
            if cursor.spelling == "RCLCPP_COMPONENTS_REGISTER_NODE":
                tokens = _cursor_tokens(cursor)
                # Tokens: RCLCPP_COMPONENTS_REGISTER_NODE ( ns :: ClassName )
                try:
                    paren_open = tokens.index("(")
                    paren_close = tokens.index(")", paren_open)
                    inner = "".join(tokens[paren_open + 1: paren_close])
                    return inner.strip()
                except ValueError:
                    pass
    return None


# ---------------------------------------------------------------------------
# Class cursor finder
# ---------------------------------------------------------------------------

_ROS_BASE_CLASSES = {
    "rclcpp::Node",
    "rclcpp_lifecycle::LifecycleNode",
    # Simplified spellings that may appear after template stripping
    "Node",
    "LifecycleNode",
}

# Lifecycle base spellings live in models.py, so the parser and generation
# share one definition (generation must ask the same question
# the parser asks, and a drift between the two would be silent).
_LIFECYCLE_BASE_SUBSTRINGS = LIFECYCLE_BASE_SUBSTRINGS
# Substrings that indicate any ROS node base for unresolved forward declarations
_ROS_BASE_SUBSTRINGS = ("::Node", "LifecycleNode")

_LIFECYCLE_METHODS = {
    "on_configure", "on_activate", "on_deactivate",
    "on_cleanup", "on_shutdown", "on_error",
}

# Nav2 wrapper class names that wrap create_publisher / create_subscription.
# Constructor signature: (node_ptr, topic_string, qos[, callbacks...])
_NAV2_PUBLISHER_WRAPPERS = {"TwistPublisher"}

# Statements whose body runs on some paths only.
# A lambda counts too: its body runs only when it is called.
_CONDITIONAL_KIND_NAMES = (
    "IF_STMT", "SWITCH_STMT", "FOR_STMT", "WHILE_STMT", "DO_STMT",
    "CXX_FOR_RANGE_STMT", "CONDITIONAL_OPERATOR", "LAMBDA_EXPR",
    "CXX_TRY_STMT", "CXX_CATCH_STMT",
)
# Statements after which the rest of the walked body may not run.
_EXIT_KIND_NAMES = ("RETURN_STMT", "CXX_THROW_EXPR", "GOTO_STMT", "INDIRECT_GOTO_STMT")


def _kinds(names: tuple[str, ...]) -> frozenset:
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return frozenset()
    return frozenset(getattr(CursorKind, n) for n in names if hasattr(CursorKind, n))


_CONDITIONAL_KINDS = _kinds(_CONDITIONAL_KIND_NAMES)
_EXIT_KINDS = _kinds(_EXIT_KIND_NAMES)


_SHORT_CIRCUIT = ("&&", "||", "and", "or")
# Spellings of the binary operators that always evaluate both operands.
_PLAIN_BINARY_OPS = frozenset((
    "*", "/", "%", "+", "-", "<<", ">>", "<=>", "<", ">", "<=", ">=", "==", "!=",
    "&", "^", "|", "=", ",", ".*", "->*", "bitand", "bitor", "xor", "not_eq",
))


_BINOP_FNS: list = []  # [kind_fn, spelling_fn] once loaded, [None] when unavailable


def _binop_functions() -> tuple[Any, Any] | None:
    """libclang's binary-operator-kind functions on a private ctypes handle.

    A separate ``CDLL`` keeps the argtypes set here off the function objects
    the clang binding itself registers on ``conf.lib``.
    """
    if not _BINOP_FNS:
        try:
            import ctypes
            from clang.cindex import conf, Cursor, _CXString
            lib = ctypes.CDLL(conf.lib._name)
            kind_fn = lib.clang_getCursorBinaryOperatorKind
            kind_fn.argtypes = [Cursor]
            kind_fn.restype = ctypes.c_int
            spell_fn = lib.clang_getBinaryOperatorKindSpelling
            spell_fn.argtypes = [ctypes.c_int]
            spell_fn.restype = _CXString
            _BINOP_FNS.extend([kind_fn, spell_fn])
        except Exception:
            _BINOP_FNS.append(None)
    return None if _BINOP_FNS[0] is None else (_BINOP_FNS[0], _BINOP_FNS[1])


def _ast_binary_operator(cursor: Any) -> str | None:
    """Spelling of a BINARY_OPERATOR's opcode from the AST, or None when libclang cannot say.

    ``clang_getCursorBinaryOperatorKind`` (libclang 17+) reads the opcode the
    compiler resolved, so a macro such as ``#define AND &&`` does not hide it.
    The Python binding of libclang 18 does not wrap it yet.
    """
    fns = _binop_functions()
    if fns is None:
        return None
    try:
        from clang.cindex import _CXString
        kind = fns[0](cursor)
        if kind <= 0:  # CXBinaryOperator_Invalid
            return None
        op = _CXString.from_result(fns[1](kind))
        if isinstance(op, bytes):
            op = op.decode()
        return op or None
    except Exception:
        return None


def _token_binary_operator(cursor: Any) -> str | None:
    """Spelling of the one token between a BINARY_OPERATOR's operands, or None.

    None when the gap holds no token or more than one, which a macro that
    spells the operator (or the whole expression) produces.
    """
    children = list(cursor.get_children())
    if len(children) != 2:
        return None
    lhs_end = children[0].extent.end.offset
    rhs_start = children[1].extent.start.offset
    if lhs_end is None or rhs_start is None or lhs_end > rhs_start:
        return None
    gap = [t.spelling for t in cursor.get_tokens()
           if lhs_end <= t.extent.start.offset and t.extent.end.offset <= rhs_start]
    return gap[0] if len(gap) == 1 else None


def _binary_operator_condition(cursor: Any) -> bool | None:
    """Whether a BINARY_OPERATOR short-circuits: True, False, or None when its kind is unread.

    The AST opcode is authoritative. Without it, the single token between the
    operands decides, and only when it spells a known operator; anything
    else (a macro such as ``AND``, an operator inside a macro expansion) is
    None, which the caller counts as not exact.
    """
    op = _ast_binary_operator(cursor)
    if op is None:
        op = _token_binary_operator(cursor)
    if op in _SHORT_CIRCUIT:
        return True
    if op in _PLAIN_BINARY_OPS:
        return False
    return None


def _opens_condition(cursor: Any) -> bool | None:
    """Whether the children of *cursor* run on some paths only.

    A short-circuit ``&&`` / ``||`` counts: its right operand may not run.
    None when *cursor* is a binary operator whose kind cannot be read:
    its operands may or may not all run.
    """
    if cursor.kind in _CONDITIONAL_KINDS:
        return True
    if cursor.kind.name == "BINARY_OPERATOR":
        return _binary_operator_condition(cursor)
    return False
_GOTO_KINDS = _kinds(("GOTO_STMT", "INDIRECT_GOTO_STMT"))


def _undecide_after_goto(body: Any, publishers: list) -> None:
    """Any goto in a walked body leaves its publishers undecided.

    A backward goto can run a create call that comes before it again, which
    the walk's straight-line reading (``exited`` covers only the calls after a
    goto) cannot see: an undercount. A goto inside a lambda counts too; that
    errs on the safe side.
    """
    if any(c.kind in _GOTO_KINDS for c in body.walk_preorder()):
        for p in publishers:
            if p.unconditional is True:
                p.unconditional = None


def _extent_key(cursor: Any) -> tuple:
    ext = cursor.extent
    f = ext.start.file
    return (f.name if f is not None else None, ext.start.offset, ext.end.offset)


def _strip_wrappers(cursor: Any) -> Any:
    """*cursor* seen through implicit conversions, temporaries, parentheses and brace lists."""
    while True:
        children = list(cursor.get_children())
        if len(children) != 1:
            return cursor
        name = cursor.kind.name
        if name in ("UNEXPOSED_EXPR", "PAREN_EXPR", "INIT_LIST_EXPR"):
            cursor = children[0]
        elif name == "CALL_EXPR" and _extent_key(children[0]) == _extent_key(cursor):
            cursor = children[0]  # a converting or copying construction
        else:
            return cursor


def _own_field(cursor: Any) -> Any | None:
    """The FIELD_DECL that *cursor* names on this object (``f_`` / ``this->f_``), else None."""
    c = _strip_wrappers(cursor)
    if c.kind.name != "MEMBER_REF_EXPR":
        return None
    ref = c.referenced
    if ref is None or ref.kind.name != "FIELD_DECL":
        return None
    base = list(c.get_children())
    if not base or (len(base) == 1 and _strip_wrappers(base[0]).kind.name == "CXX_THIS_EXPR"):
        return ref
    return None


def _owns_shared(field: Any) -> bool:
    """True when the FIELD_DECL *field* is a ``std::shared_ptr`` (it keeps what it holds).

    A ``std::weak_ptr`` or raw pointer member lets the publisher die with the
    temporary that create_publisher returned.
    """
    try:
        return field.type.get_canonical().spelling.startswith("std::shared_ptr<")
    except Exception:
        return False


def _note_member_store(cursor: Any, stores: dict) -> None:
    """Record ``f_ = create_publisher(...)`` and the ``f_(create_publisher(...))`` initializer.

    *stores* maps the create call's extent to ``(field USR, extent of the
    member reference on the left)`` (lifetime).
    """
    pairs: list[tuple[Any, Any]] = []
    name = cursor.kind.name
    children = list(cursor.get_children())
    if name == "BINARY_OPERATOR" and len(children) == 2:
        op = _ast_binary_operator(cursor) or _token_binary_operator(cursor)
        if op == "=":
            pairs.append((children[0], children[1]))
    elif name == "CALL_EXPR" and cursor.spelling == "operator=" and len(children) in (2, 3):
        pairs.append((children[0], children[-1]))
    elif name == "CONSTRUCTOR":
        # Member initializers: a MEMBER_REF to the field, then its initializer.
        for lhs, rhs in zip(children, children[1:]):
            if lhs.kind.name == "MEMBER_REF" and lhs.referenced is not None \
                    and lhs.referenced.kind.name == "FIELD_DECL":
                pairs.append((lhs, rhs))
    for lhs, rhs in pairs:
        if lhs.kind.name == "MEMBER_REF":
            field = lhs.referenced
        else:
            field = _own_field(lhs)
        if field is None or not _owns_shared(field):
            continue
        call = _strip_wrappers(rhs)
        if call.kind.name == "CALL_EXPR" and call.spelling == "create_publisher":
            stores[_extent_key(call)] = (field.get_usr(), _extent_key(lhs))


# After a reference to a kept publisher member, the text that only reads it:
# a call through it, or .get().
_MEMBER_READ = re.compile(rb"\s*(?:->|\.\s*get\s*\(\s*\))")
# Overloaded operators whose object is only read: ``*f_`` and ``f_->`` (the
# standard library's only; see _read_by_use).
_READING_OPERATORS = {"operator*", "operator->"}
# Overloaded comparisons that only read both operands: ``f_ == nullptr``, ``f_ != other``.
_READING_COMPARISONS = {"operator==", "operator!="}


def _read_by_use(ref: Any, parents: list) -> bool:
    """True when the use of member reference *ref* (under *parents*) only reads it.

    The nearest ancestor past implicit conversions and parentheses decides: a
    dereference (``*f_``, ``f_->``), a copy construction (``auto p = f_;``, a
    by-value argument), a const member call on it (the boolean conversion of
    ``if (f_)``, ``!f_`` and ``static_cast<bool>(f_)``, ``use_count()``) or a
    comparison (``f_ == nullptr``, ``f_ != nullptr``) leaves the member as it
    was. ``std::move(f_)`` reaches the move constructor through the ``move``
    call, so it is not a read.
    """
    for p in reversed(parents):
        kind = p.kind.name
        if kind in ("UNEXPOSED_EXPR", "PAREN_EXPR"):
            continue
        if kind == "MEMBER_REF_EXPR":
            # f_.method(): the member is the object; a const method leaves it.
            method = p.referenced
            try:
                return (method is not None
                        and method.kind.name in ("CXX_METHOD", "CONVERSION_FUNCTION")
                        and method.is_const_method())
            except Exception:
                return False
        if kind != "CALL_EXPR":
            return False
        n_children = len(list(p.get_children()))
        if (p.spelling in _READING_OPERATORS and n_children == 2) \
                or (p.spelling in _READING_COMPARISONS and n_children == 3):
            # Only the standard library's operator; a project overload may
            # take the member by non-const reference and reset it
            # (``Pub & operator*(Pub::SharedPtr & p)``).
            op = p.referenced
            return op is not None and _enclosing_scopes(op)[:1] == ["std"]
        target = p.referenced
        try:
            return (n_children == 1 and target is not None
                    and target.kind.name == "CONSTRUCTOR" and target.is_copy_constructor())
        except Exception:
            return False
    return False


_CLASS_KINDS = ("CLASS_DECL", "STRUCT_DECL", "UNION_DECL", "CLASS_TEMPLATE")
_FUNCTION_KINDS = ("CXX_METHOD", "CONSTRUCTOR", "DESTRUCTOR", "CONVERSION_FUNCTION",
                   "FUNCTION_TEMPLATE", "FUNCTION_DECL")


def _access(cursor: Any) -> str | None:
    """``PUBLIC`` / ``PROTECTED`` / ``PRIVATE`` of a member, or None when unknown."""
    try:
        return cursor.access_specifier.name
    except Exception:
        return None


def _unseen_member_code(cls: Any, own: set[Path], never_runs: Any) -> str | None:
    """A member function or friend of *cls* whose body is not in the files *own*.

    Returns its name, or None when all of the class's code (and that of the
    classes nested in it) is defined in *own* (lifetime). A deleted
    method has no body anywhere, and a private non-virtual method for which
    *never_runs* is true is called by no code (code that could call it is
    itself checked); both are skipped. A friend class counts as unseen.

    Removed three exemptions, each of which let unseen code reset a
    member: a pure virtual method can still have a body in another .cpp and be
    called qualified (``DataNode::clear()``); a const method can reset any
    member through ``const_cast<DataNode *>(this)``; and the node's destructor,
    though it runs after its own node is gone, can reset another, live
    instance's member (``peer_->pub_.reset()``). A destructor whose body is
    seen is checked by _fields_touched_elsewhere, which skips only its uses
    through ``this``.
    """
    definition = cls.get_definition() or cls

    def _defined_here(decl: Any) -> bool:
        target = decl.get_definition()
        loc = target.location.file if target is not None else None
        return loc is not None and Path(loc.name).resolve() in own

    for c in definition.get_children():
        kind = c.kind.name
        if kind in _CLASS_KINDS and c.is_definition():
            nested = _unseen_member_code(c, own, never_runs)
            if nested is not None:
                return nested
        elif kind == "FRIEND_DECL":
            for f in c.get_children():
                if f.kind.name not in _FUNCTION_KINDS:
                    return f"friend {f.spelling or f.kind.name}"
                if not _defined_here(f):
                    return f"friend {f.spelling}"
        elif kind in _FUNCTION_KINDS:
            if _cursor_flag(c, "is_deleted_method"):
                continue
            if not _cursor_flag(c, "is_virtual_method") and never_runs(c):
                continue
            if not _defined_here(c):
                return c.spelling
    return None


def _through_this(ref: Any) -> bool:
    """True when member reference *ref* names a member of ``this`` (``f_``, ``this->f_``)."""
    children = list(ref.get_children())
    if not children:
        return True  # implicit this
    if len(children) != 1:
        return False
    base = children[0]
    while base.kind.name in ("UNEXPOSED_EXPR", "PAREN_EXPR"):
        inner = list(base.get_children())
        if len(inner) != 1:
            return False
        base = inner[0]
    return base.kind.name == "CXX_THIS_EXPR"


def _fields_touched_elsewhere(body: Any, fields: set[str], store_lhs: set[tuple]) -> set[str]:
    """Fields in *fields* that the node's own code may reset, reassign, move or hand out.

    Every reference to the field in the source file and the class's own file,
    outside the recorded create stores and the node's own destructor's uses
    through ``this``, must only read it:
    ``->``, ``.get()``, ``*f_`` or a copy (``_read_by_use``). Any other use
    (``reset()``, ``=``, ``std::move``, a reference binding, a truth test)
    counts as a possible write, which errs on the safe side. A private,
    non-virtual method that no code here refers to never runs and is skipped.
    Code in other translation units is not seen, so when the class (or a class
    nested in it) declares a method, constructor, destructor or friend whose
    definition is not in these files, every field counts as touched: that code
    may reset the member (``void stop() { pub_.reset(); }`` in another .cpp;
    this includes const methods, pure virtual methods and the destructor, see
    _unseen_member_code).
    A public member, or one of a base class, counts as touched as well: code
    outside the class may reset it. A protected member is reset only by a
    subclass, which is another node class whose analysis sees it as a base
    member.
    """
    tu = body.translation_unit
    own: set[Path] = set()
    for name in (tu.spelling, getattr(body.semantic_parent.location.file, "name", None)
                 if body.semantic_parent is not None else None):
        if name:
            own.add(Path(name).resolve())
    sources: dict[str, bytes] = {}
    tops = [top for top in tu.cursor.get_children()
            if top.location.file is not None and Path(top.location.file.name).resolve() in own]
    referenced: set[str] = set()
    for top in tops:
        for c in top.walk_preorder():
            if c.kind.is_reference() or c.kind.is_expression():
                ref = c.referenced
                if ref is not None and ref != c:
                    referenced.add(ref.get_usr())

    def _dead(c: Any) -> bool:
        if c.kind.name != "CXX_METHOD" or not c.is_definition() or c.is_virtual_method():
            return False
        return _unreferenced_private(c)

    def _unreferenced_private(c: Any) -> bool:
        try:
            private = c.access_specifier.name == "PRIVATE"
        except Exception:
            return False
        return private and c.get_usr() not in referenced

    node_class = body.semantic_parent.get_usr() if body.semantic_parent is not None else None
    node = body.semantic_parent
    if node is None:
        return set(fields)
    node_fields = {c.get_usr(): c for c in (node.get_definition() or node).get_children()
                   if c.kind.name == "FIELD_DECL"}
    if _unseen_member_code(node, own, _unreferenced_private):
        return set(fields)
    # A member of a base class is reset by the base's code, which is not scanned;
    # a public member may be reset by any code that holds the node.
    touched: set[str] = {f for f in fields if f not in node_fields
                         or _access(node_fields[f]) not in ("PRIVATE", "PROTECTED")}

    def _node_destructor(c: Any) -> bool:
        # Only the node's own destructor runs after the node is gone; another
        # class's (a scope guard in the constructor) may reset the member early.
        parent = c.semantic_parent
        return (c.kind.name == "DESTRUCTOR" and node_class is not None
                and parent is not None and parent.get_usr() == node_class)

    def _visit(c: Any, parents: list, in_destructor: bool = False) -> None:
        if _dead(c):
            return
        in_destructor = in_destructor or _node_destructor(c)
        if c.kind.name == "MEMBER_REF_EXPR" and c.referenced is not None:
            usr = c.referenced.get_usr()
            # In the node's destructor its own members go with it; a member of
            # another instance (peer_->pub_) may still be live.
            if usr in fields and usr not in touched and _extent_key(c) not in store_lhs \
                    and not (in_destructor and _through_this(c)):
                fname, _start, end = _extent_key(c)
                try:
                    raw = sources.setdefault(fname, Path(fname).read_bytes())
                except (OSError, TypeError):
                    raw = b""
                if (end is None or not _MEMBER_READ.match(raw, end)) \
                        and not _read_by_use(c, parents):
                    touched.add(usr)
        parents.append(c)
        for ch in c.get_children():
            _visit(ch, parents, in_destructor)
        parents.pop()

    for top in tops:
        _visit(top, [])
    return touched


def _undecide_unkept_publishers(body: Any, publishers: list) -> None:
    """Publishers the node may not keep alive are undecided (lifetime).

    A publisher counts as made on every path only when its create call is
    assigned straight to a member of this node (``f_ = create_publisher(...)``
    or a member initializer), no other create call is stored in that member,
    and the node's own code only reads the member. A publisher in a local
    variable, a container or a discarded result, or in a member set twice or
    reset, may be gone while the reader still expects it: an overcount.
    """
    kept = [getattr(p, "_kept_in", None) for p in publishers]
    stores_per_field = Counter(k[0] for k in kept if k is not None)
    candidates = []
    for p, k in zip(publishers, kept):
        if p.unconditional is not True:
            continue
        if k is None or stores_per_field[k[0]] > 1:
            p.unconditional = None
        else:
            candidates.append((p, k[0]))
    if not candidates:
        return
    touched = _fields_touched_elsewhere(
        body, {f for _p, f in candidates}, {k[1] for k in kept if k is not None})
    for p, field in candidates:
        if field in touched:
            p.unconditional = None


_NAV2_SUBSCRIBER_WRAPPERS = {"TwistSubscriber", "OdomSubscriber"}
# Nav2 action server wrappers (make_unique<SimpleActionServer<T>>(...))
_NAV2_ACTION_SERVER_WRAPPERS = {"SimpleActionServer", "ActionServer"}
# Nav2 declare_parameter helpers (treated identically to declare_parameter)
_NAV2_DECLARE_PARAM_HELPERS = {
    "declare_parameter_if_not_declared",
    "nav2_util::declare_parameter_if_not_declared",
}

# Gap 5: Known message types for Nav2 wrapper classes.
_WRAPPER_MESSAGE_TYPES: dict[str, str] = {
    "TwistPublisher": "geometry_msgs::msg::Twist",
    "TwistSubscriber": "geometry_msgs::msg::Twist",
    "OdomSubscriber": "nav_msgs::msg::Odometry",
}


def _inherits_from_ros_node(
    cursor: Any,
    cache: dict[str, bool],
    depth: int = 0,
) -> bool:
    """Check if *cursor* (a CLASS_DECL) transitively inherits from a ROS2 node base.

    Walks up the inheritance chain via ``base.type.get_declaration()``.
    Results are cached by cursor USR (Unified Symbol Resolution) to avoid
    repeated walks.  Recursion is limited to 10 levels.
    """
    if depth > 10:
        return False
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return False

    usr = cursor.get_usr() if cursor.get_usr else ""
    if usr and usr in cache:
        return cache[usr]

    result = False
    for child in cursor.get_children():
        if child.kind != CursorKind.CXX_BASE_SPECIFIER:
            continue
        base_name = child.type.spelling if child.type else ""
        base_clean = re.sub(r"<.*>", "", base_name).strip()
        if base_clean in _ROS_BASE_CLASSES:
            result = True
            break
        # Check for aliased base names (e.g. lc::LifecycleNode for rclcpp_lifecycle)
        if any(s in base_clean for s in _ROS_BASE_SUBSTRINGS):
            result = True
            break
        # Resolve the base class declaration and check recursively
        base_decl = child.type.get_declaration() if child.type else None
        # ``Base<T>`` is an implicit specialization that carries no bases of
        # its own; the template the specifier names does.
        tmpl = next(
            (c for c in child.get_children() if c.kind == CursorKind.TEMPLATE_REF),
            None,
        )
        if tmpl is not None and tmpl.referenced is not None:
            base_decl = tmpl.referenced
        if base_decl is not None and base_decl.kind in (
            CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL,
            CursorKind.CLASS_TEMPLATE,
        ):
            if base_decl.is_definition():
                if _inherits_from_ros_node(base_decl, cache, depth + 1):
                    result = True
                    break

    if usr:
        cache[usr] = result
    return result


_SYSTEM_FILE_PREFIXES = ("/opt/", "/usr/")


def _is_workspace_cursor(cursor: Any) -> bool:
    """Whether *cursor* is declared in a file outside the installed system trees."""
    loc = cursor.location.file
    return loc is not None and not loc.name.startswith(_SYSTEM_FILE_PREFIXES)


def _base_specs(cursor: Any) -> list[tuple[Any, bool]]:
    """``(base declaration, is_template)`` for each direct base of a class cursor.

    A base written ``EchoBase<Msg>`` is an implicit specialization whose
    declaration libclang reports without its own bases; the base specifier's
    TEMPLATE_REF child names the template, which is what the walk needs.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    out: list[tuple[Any, bool]] = []
    for child in cursor.get_children():
        if child.kind != CursorKind.CXX_BASE_SPECIFIER:
            continue
        tmpl = next(
            (c for c in child.get_children() if c.kind == CursorKind.TEMPLATE_REF),
            None,
        )
        decl = None
        if tmpl is not None:
            decl = tmpl.referenced
        elif child.type is not None:
            decl = child.type.get_declaration()
        if decl is None or decl.kind not in (
            CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL, CursorKind.CLASS_TEMPLATE,
        ):
            continue
        is_template = tmpl is not None or decl.kind == CursorKind.CLASS_TEMPLATE
        out.append((decl, is_template))
    return out


def _class_family(
    class_cursor: Any, depth: int = 0,
) -> tuple[list[Any], list[Any]]:
    """The workspace bases of a node class, most-base first, and its template bases.

    Bases declared in the installed ROS trees (``rclcpp::Node`` and friends)
    are neither: their code is the runtime's. A template base is returned
    apart, because its constructor cannot be walked (RCLLF-H015); its own
    bases are still part of the family.
    """
    bases: list[Any] = []
    templates: list[Any] = []
    if depth > 10:
        return bases, templates
    for decl, is_template in _base_specs(class_cursor):
        if not _is_workspace_cursor(decl):
            continue
        definition = decl if decl.is_definition() else decl.get_definition()
        if definition is None:
            continue
        sub_bases, sub_templates = _class_family(definition, depth + 1)
        for b in sub_bases:
            if b not in bases:
                bases.append(b)
        for t in sub_templates:
            if t not in templates:
                templates.append(t)
        if is_template:
            if definition not in templates:
                templates.append(definition)
        elif definition not in bases:
            bases.append(definition)
    return bases, templates


_family_cache: dict[int, list[Any]] = {}


def _in_family(parent: Any, class_cursor: Any) -> bool:
    """Whether *parent* is the node class or one of its workspace base classes."""
    if parent is None:
        return False
    if parent == class_cursor:
        return True
    fam = _family_cache.get(class_cursor.hash)
    if fam is None:
        fam = _class_family(class_cursor)[0]
        _family_cache[class_cursor.hash] = fam
    return any(parent == b for b in fam)


_ENDPOINT_CREATE_CALLS = frozenset({
    "create_publisher", "create_subscription", "create_service", "create_client",
    "create_generic_publisher", "create_generic_subscription",
    "create_wall_timer", "create_timer",
})


def _template_endpoint_calls(template: Any) -> list[str]:
    """The ``create_*`` calls in a template base's constructors.

    A token scan: libclang does not resolve a call on a dependent ``this``.
    Methods the constructor names are scanned too (one level), so a
    ``setup()`` helper counts.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    ctor_tokens: list[str] = []
    methods: dict[str, list[str]] = {}
    for child in template.get_children():
        if child.kind == CursorKind.CONSTRUCTOR:
            ctor_tokens += _cursor_tokens(child)
        elif child.kind in (CursorKind.CXX_METHOD, CursorKind.FUNCTION_TEMPLATE):
            methods[child.spelling] = _cursor_tokens(child)
    scanned = list(ctor_tokens)
    for name, toks in methods.items():
        if name in ctor_tokens:
            scanned += toks
    found: list[str] = []
    for tok in scanned:
        if tok in _ENDPOINT_CREATE_CALLS and tok not in found:
            found.append(tok)
    return found


def _main_node_class_names(tu: Any) -> list[str]:
    """Class names ``main()`` of the primary file builds with ``make_shared``/``make_unique``.

    In order of appearance. These name the node class an executable runs,
    ahead of the scoring heuristic of ``_find_node_class``.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    main_file = tu.spelling
    names: list[str] = []
    for top in tu.cursor.get_children():
        if top.kind != CursorKind.FUNCTION_DECL or top.spelling != "main":
            continue
        if not top.is_definition():
            continue
        if top.location.file is None or top.location.file.name != main_file:
            continue
        for call in top.walk_preorder():
            if call.kind != CursorKind.CALL_EXPR:
                continue
            if call.spelling not in ("make_shared", "make_unique"):
                continue
            for ref in call.walk_preorder():
                if ref.kind == CursorKind.TYPE_REF:
                    name = re.sub(r"^(class|struct)\s+", "", ref.spelling).strip()
                    if name and name not in names:
                        names.append(name)
    return names


def _find_node_class(tu: Any) -> Any | None:
    """Return the most-derived CLASS_DECL that inherits from a ROS2 node base.

    Collects all matching classes in the translation unit. The class
    ``main()`` builds with ``make_shared`` or the component registration names
    wins when it is a candidate; otherwise the most-derived one (last
    definition in source order, which is typically the concrete node class
    rather than a utility base wrapper) is picked. A class template is never
    the answer while a concrete class exists.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None

    cache: dict[str, bool] = {}
    candidates: list[Any] = []

    for cursor in tu.cursor.walk_preorder():
        if cursor.kind not in (
            CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL,
            CursorKind.CLASS_TEMPLATE,
        ):
            continue
        if not cursor.is_definition():
            continue
        if _inherits_from_ros_node(cursor, cache):
            candidates.append(cursor)

    if not candidates:
        return None

    concrete = [c for c in candidates if c.kind != CursorKind.CLASS_TEMPLATE]
    if concrete:
        candidates = concrete
    hints = _main_node_class_names(tu)
    component = _extract_component_class(tu)
    if component:
        hints.append(component)
    for hint in hints:
        short = hint.rsplit("::", 1)[-1]
        named = [c for c in candidates if c.spelling == short]
        if len(named) > 1:
            named = [c for c in named if _qualified_class_name(c).endswith(hint)] or named
        if named:
            return named[-1]

    # Score candidates to pick the most-concrete node class.
    # Prefer classes defined in the main source file over those pulled in via
    # headers, and penalise names that look like internal/iterator types.
    main_file = tu.spelling  # path of the primary source file
    _INTERNAL_NAMES = ("iterator", "value", "detail", "impl", "helper", "base")
    _SYSTEM_PREFIXES = ("/opt/", "/usr/")

    def _score(c: Any) -> int:
        score = 0
        loc = c.location.file
        # A class from the main file or a workspace header beats one from an
        # installed ROS header: rclcpp_lifecycle::LifecycleNode itself is a
        # candidate whenever its definition is visible (it won
        # over helper_method_example's HelperNode).
        if loc and (loc.name == main_file or not loc.name.startswith(_SYSTEM_PREFIXES)):
            score += 2
        try:
            from clang.cindex import CursorKind
            for child in c.get_children():
                if child.kind == CursorKind.CONSTRUCTOR and child.is_definition():
                    score += 1
                    break
        except Exception:
            pass
        name_lower = c.spelling.lower()
        if not name_lower.endswith("node") and any(s in name_lower for s in _INTERNAL_NAMES):
            score -= 2
        return score

    # Among candidates with the highest score, pick the last in source order.
    best_score = max(_score(c) for c in candidates)
    best = [c for c in candidates if _score(c) == best_score]
    return best[-1]


def _find_ros_base(class_cursor: Any, depth: int = 0) -> str:
    """Walk up the inheritance chain to find the ultimate ROS2 base class name.

    Returns the canonical name (e.g. ``"rclcpp_lifecycle::LifecycleNode"``) when
    found, or the closest resolvable approximation when headers are missing.
    """
    if depth > 10:
        return ""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return ""
    for child in class_cursor.get_children():
        if child.kind != CursorKind.CXX_BASE_SPECIFIER:
            continue
        base_name = child.type.spelling if child.type else ""
        base_clean = re.sub(r"<.*>", "", base_name).strip()
        if base_clean in _ROS_BASE_CLASSES:
            return base_clean
        # Check for aliased base names (e.g. lc::LifecycleNode)
        if any(s in base_clean for s in _LIFECYCLE_BASE_SUBSTRINGS):
            return "rclcpp_lifecycle::LifecycleNode"
        if "::Node" in base_clean or base_clean == "Node":
            return "rclcpp::Node"
        base_decl = child.type.get_declaration() if child.type else None
        tmpl = next(
            (c for c in child.get_children() if c.kind == CursorKind.TEMPLATE_REF),
            None,
        )
        if tmpl is not None and tmpl.referenced is not None:
            base_decl = tmpl.referenced
        if base_decl is not None and base_decl.kind in (
            CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL,
            CursorKind.CLASS_TEMPLATE,
        ):
            if base_decl.is_definition():
                result = _find_ros_base(base_decl, depth + 1)
                if result:
                    return result
    return ""


def _base_class_spelling(class_cursor: Any) -> str:
    """Return the ROS2 base class spelling, walking up the inheritance chain.

    Falls back to a direct-base-name heuristic when ``_find_ros_base`` cannot
    resolve the full chain (e.g. missing nav2_util headers).
    """
    result = _find_ros_base(class_cursor)
    if result:
        return result
    # Heuristic fallback: inspect the direct base class name
    try:
        from clang.cindex import CursorKind
        for child in class_cursor.get_children():
            if child.kind != CursorKind.CXX_BASE_SPECIFIER:
                continue
            base_name = child.type.spelling if child.type else ""
            base_clean = re.sub(r"<.*>", "", base_name).strip()
            if any(s in base_clean for s in _LIFECYCLE_BASE_SUBSTRINGS):
                return "rclcpp_lifecycle::LifecycleNode"
            if "::Node" in base_clean or base_clean in ("Node", "rclcpp::Node"):
                return "rclcpp::Node"
    except Exception:
        pass
    return ""


def _ros_node_name_from_constructor(constructor_cursor: Any) -> str | None:
    """Return the ROS node name from the ``Node(...)`` base initializer.

    A node's runtime identity is the string literal it passes to its
    ``rclcpp::Node`` / ``rclcpp_lifecycle::LifecycleNode`` base constructor
    (``: Node("action_client")``), which is what shows up as
    ``rcl_node_init.node_name`` in a trace. Deriving it from the class name
    instead is only ever a guess, and a wrong guess silently breaks
    trace-to-model resolution in lf_eval.

    Only the *base initializer* is inspected — not the constructor body — so
    unrelated string literals (topic names, parameter names, log messages)
    cannot be mistaken for the node name. Returns ``None`` when no literal is
    present, which is the legitimate case for a node that takes its name as a
    constructor argument or from ``NodeOptions``; the caller then falls back to
    the class-derived guess.
    """
    if constructor_cursor is None:
        return None
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None

    try:
        for child in constructor_cursor.get_children():
            # Skip the constructor body outright — a literal in there is a
            # topic/parameter/log string, never the node name. Base and member
            # initializers are the cursor's non-COMPOUND_STMT expression
            # children; libclang wraps each in an UNEXPOSED_EXPR, so the
            # CALL_EXPR naming the base class sits *inside* the child rather
            # than being the child itself.
            if child.kind == CursorKind.COMPOUND_STMT:
                continue
            if child.kind not in (CursorKind.UNEXPOSED_EXPR, CursorKind.CALL_EXPR):
                continue

            for node_call in child.walk_preorder():
                if node_call.kind != CursorKind.CALL_EXPR:
                    continue
                spelling = (node_call.spelling or "").strip()
                base_clean = re.sub(r"<.*>", "", spelling).strip().split("::")[-1]
                # Member initializers (count_(0)) and the implicit
                # std::basic_string conversion around the literal are not the
                # base-class call we want.
                if base_clean not in ("Node", "LifecycleNode"):
                    continue
                for desc in node_call.walk_preorder():
                    if desc.kind != CursorKind.STRING_LITERAL:
                        continue
                    tokens = _cursor_tokens(desc)
                    if not tokens:
                        continue
                    raw = tokens[0]
                    if raw.startswith('"') and raw.endswith('"'):
                        raw = raw[1:-1]
                    raw = raw.strip()
                    # The first literal in the base initializer is the node
                    # name; anything after it (namespace, options) is not.
                    return raw or None
    except Exception:
        return None
    return None


# ---------------------------------------------------------------------------
# Method signature extractor
# ---------------------------------------------------------------------------

def _extract_method_signatures(
    class_cursor: Any,
    tu: Any,
    cpp_path: Path,
) -> dict[str, CallbackInfo]:
    """Return a dict mapping method name -> CallbackInfo with parameters."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return {}

    infos: dict[str, CallbackInfo] = {}

    # Collect methods defined in the same file (avoids parsing included headers)
    cpp_resolved = cpp_path.resolve()
    own_files = _own_definition_files(cpp_resolved, class_cursor)

    for cursor in tu.cursor.walk_preorder():
        if cursor.kind != CursorKind.CXX_METHOD:
            continue
        # Only methods belonging to our node class
        if not _in_family(cursor.semantic_parent, class_cursor):
            continue
        # Only definitions (not just declarations)
        if not cursor.is_definition():
            continue
        # Only from the primary source file
        if cursor.location.file is None:
            continue
        if Path(cursor.location.file.name).resolve() not in own_files:
            continue

        method_name = cursor.spelling
        params: list[CallbackParameter] = []
        for arg in cursor.get_arguments():
            pname = arg.spelling or ""
            ptype = arg.type.spelling if arg.type else ""
            # Use canonical form to resolve typedefs (e.g. SharedPtr)
            try:
                canonical = arg.type.get_canonical().spelling
                if canonical and canonical != ptype:
                    ptype = canonical
            except Exception:
                pass
            params.append(CallbackParameter(name=pname, type=ptype))

        infos[method_name] = CallbackInfo(name=method_name, parameters=params)

    return infos


def _is_action_create_client(cursor: Any) -> bool:
    """Return True if this create_client call is rclcpp_action::create_client.

    Service clients are member calls (first child is MEMBER_REF_EXPR).
    Action clients are free-function calls — first child is UNEXPOSED_EXPR
    or DECL_REF_EXPR (the namespace-qualified function reference).
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return False
    children = list(cursor.get_children())
    if not children:
        return False
    first = children[0]
    # Member call: first child is MEMBER_REF_EXPR (this->create_client)
    if first.kind == CursorKind.MEMBER_REF_EXPR:
        return False
    # Free function: look for NAMESPACE_REF to rclcpp_action in first child
    for desc in first.walk_preorder():
        if desc.kind == CursorKind.NAMESPACE_REF and desc.spelling == "rclcpp_action":
            return True
    return True  # No MEMBER_REF_EXPR = free function = action client


def _extract_all_method_refs(cursor: Any) -> list[str]:
    """Walk *cursor* subtree and return all CXX_METHOD spellings from
    DECL_REF_EXPR nodes, in document order, skipping the first child
    (which is the function reference itself).
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    methods: list[str] = []
    seen: set[str] = set()
    children = list(cursor.get_children())
    # Skip first child (the function/method reference)
    for child in children[1:]:
        for desc in child.walk_preorder():
            if desc.kind == CursorKind.DECL_REF_EXPR and desc.referenced:
                if desc.referenced.kind == CursorKind.CXX_METHOD:
                    name = desc.referenced.spelling
                    if name and name not in seen:
                        seen.add(name)
                        methods.append(name)
    return methods


# Container members that construct or take an element: on a container of
# threads the inserted element is a new thread running the callable argument.
_THREAD_CONTAINER_INSERTS = frozenset({"emplace_back", "push_back", "emplace"})


def _is_thread_container_insert(call: Any) -> bool:
    """Whether *call* inserts into a container whose elements are std threads.

    ``workers_.emplace_back(&Class::execute, this, gh)`` starts ``execute`` on
    a new thread just as ``std::thread(&Class::execute, this, gh)`` does. The
    container is recognised by the clang type of its first template argument
    (``std::thread`` or ``std::jthread``), never by the member's name.
    """
    try:
        from clang.cindex import CursorKind, TypeKind
    except ImportError:
        return False
    if call.spelling not in _THREAD_CONTAINER_INSERTS:
        return False
    children = list(call.get_children())
    if not children or children[0].kind != CursorKind.MEMBER_REF_EXPR:
        return False
    obj = next(iter(children[0].get_children()), None)
    if obj is None:
        return False
    container = obj.type.get_canonical()
    if container.kind in (TypeKind.POINTER, TypeKind.LVALUEREFERENCE, TypeKind.RVALUEREFERENCE):
        container = container.get_pointee().get_canonical()
    if container.get_num_template_arguments() < 1:
        return False
    decl = container.get_template_argument_type(0).get_canonical().get_declaration()
    return (decl is not None and decl.spelling in ("thread", "jthread")
            and _outer_namespace(decl) == "std")


def _extract_svc_response_callback(
    call_cursor: Any,
    svc_var_to_client: dict[str, Any],
    svc_client_var_map: dict[str, str],
    callbacks: dict[str, CallbackInfo],
    member_var_set: set[str],
    resource_vars: set[str],
    publisher_var_map: dict[str, str],
    act_client_var_map: dict[str, str],
) -> None:
    """Extract the response callback from an async_send_request() call.

    Handles two forms:
    - std::bind(&Class::method, ...) — links existing CallbackInfo
    - inline lambda — synthesizes a CallbackInfo with body analysis

    Mutates *svc_var_to_client* entries (sets response_callback) and may
    add new entries to *callbacks*.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return

    children = list(call_cursor.get_children())
    # Children: [MEMBER_REF_EXPR, arg0_request, arg1_callback]
    if len(children) < 3:
        return

    # Identify which service client by scanning tokens for a known member var
    tokens = _cursor_tokens(call_cursor)
    matched_cli = None
    matched_svc_name = None
    for tok in tokens:
        if tok in svc_var_to_client:
            matched_cli = svc_var_to_client[tok]
            matched_svc_name = svc_client_var_map.get(tok)
            break
    if matched_cli is None:
        # Fallback: if only one service client, use it
        if len(svc_var_to_client) == 1:
            var = next(iter(svc_var_to_client))
            matched_cli = svc_var_to_client[var]
            matched_svc_name = svc_client_var_map.get(var)
        else:
            return

    # Already resolved — skip
    if matched_cli.response_callback:
        return

    callback_arg = children[2]

    # --- Form 1: std::bind(&Class::method, ...) ---
    method_refs = _extract_all_method_refs(call_cursor)
    for ref in method_refs:
        if ref in callbacks:
            matched_cli.response_callback = ref
            return

    # --- Form 2: inline lambda ---
    lambda_cursor = None
    for desc in callback_arg.walk_preorder():
        if desc.kind == CursorKind.LAMBDA_EXPR:
            lambda_cursor = desc
            break
    if lambda_cursor is None:
        return

    # Synthesize a callback name
    svc_name = matched_svc_name or "unknown"
    synth_name = f"{svc_name}_response_callback"
    # Avoid name collisions
    if synth_name in callbacks:
        idx = 2
        while f"{synth_name}_{idx}" in callbacks:
            idx += 1
        synth_name = f"{synth_name}_{idx}"

    # Create CallbackInfo
    cb_info = CallbackInfo(name=synth_name, parameters=_lambda_parameters(lambda_cursor))
    callbacks[synth_name] = cb_info
    matched_cli.response_callback = synth_name

    _analyze_lambda_body(
        lambda_cursor, cb_info, callbacks, member_var_set, resource_vars,
        publisher_var_map, svc_client_var_map, act_client_var_map,
    )


def _lambda_parameters(lambda_cursor: Any) -> list[CallbackParameter]:
    """Parameters of a lambda, canonical type spelling preferred."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    params: list[CallbackParameter] = []
    for child in lambda_cursor.get_children():
        if child.kind == CursorKind.PARM_DECL:
            pname = child.spelling or ""
            ptype = child.type.spelling if child.type else ""
            try:
                canonical = child.type.get_canonical().spelling
                if canonical and canonical != ptype:
                    ptype = canonical
            except Exception:
                pass
            params.append(CallbackParameter(name=pname, type=ptype))
    return params


def _analyze_lambda_body(
    lambda_cursor: Any,
    cb_info: CallbackInfo,
    callbacks: dict[str, CallbackInfo],
    member_var_set: set[str],
    resource_vars: set[str],
    publisher_var_map: dict[str, str],
    svc_client_var_map: dict[str, str],
    act_client_var_map: dict[str, str],
) -> None:
    """Fill *cb_info*'s effects from a lambda body.

    Reads publish / async_send_request / async_send_goal calls and member
    variable reads and writes, like ``_analyze_callback_bodies`` does for a
    method. A call to a member function of the node merges that function's
    already analyzed effects (one level), so ``[this](m) { handle(m); }``
    declares what ``handle`` publishes.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return

    pubs_used: set[str] = set()
    svcs_used: set[str] = set()
    acts_used: set[str] = set()
    vars_read: set[str] = set()
    vars_written: set[str] = set()

    for desc in lambda_cursor.walk_preorder():
        if desc.kind == CursorKind.MEMBER_REF_EXPR:
            member_name = desc.spelling
            tokens = _cursor_tokens(desc)

            if member_name in _PUBLISH_CALL_NAMES:
                for tok in tokens:
                    if tok in publisher_var_map:
                        pubs_used.add(publisher_var_map[tok])
                        break
            elif member_name == "async_send_request":
                for tok in tokens:
                    if tok in svc_client_var_map:
                        svcs_used.add(svc_client_var_map[tok])
                        break
            elif member_name in ("async_send_goal", "async_cancel_goal"):
                for tok in tokens:
                    if tok in act_client_var_map:
                        acts_used.add(act_client_var_map[tok])
                        break
            elif member_name in member_var_set and member_name not in resource_vars:
                vars_read.add(member_name)

        if desc.kind == CursorKind.CALL_EXPR:
            ref = desc.referenced
            if (
                ref is not None and ref.kind == CursorKind.CXX_METHOD
                and ref.spelling in callbacks and ref.spelling != cb_info.name
            ):
                callee = callbacks[ref.spelling]
                pubs_used.update(callee.publishers_used)
                svcs_used.update(callee.service_clients_used)
                acts_used.update(callee.action_clients_used)
                vars_read.update(callee.variables_read)
                vars_written.update(callee.variables_written)

        if desc.kind == CursorKind.BINARY_OPERATOR:
            tokens = _cursor_tokens(desc)
            if "=" in tokens:
                eq_idx = tokens.index("=")
                for i in range(eq_idx):
                    if tokens[i] in member_var_set:
                        vars_written.add(tokens[i])

        if desc.kind == CursorKind.COMPOUND_ASSIGNMENT_OPERATOR:
            for tok in _cursor_tokens(desc):
                if tok in member_var_set:
                    vars_written.add(tok)
                    break

    body = " ".join(_cursor_tokens(lambda_cursor))
    for var in member_var_set:
        if any(p in body for p in (f"{var} ++", f"++ {var}", f"{var} --", f"-- {var}")):
            vars_written.add(var)

    cb_info.publishers_used = sorted(pubs_used)
    cb_info.service_clients_used = sorted(svcs_used)
    cb_info.action_clients_used = sorted(acts_used)
    cb_info.variables_written = sorted(vars_written)
    cb_info.variables_read = sorted(vars_read - vars_written)


def _synthesize_lambda_callbacks(
    subscribers: list[SubscriberInfo],
    timers: list[TimerInfo],
    callbacks: dict[str, CallbackInfo],
    member_variables: list[str],
    publisher_var_map: dict[str, str],
    svc_client_var_map: dict[str, str],
    act_client_var_map: dict[str, str],
) -> dict[str, Any]:
    """Give every lambda-callback subscription and timer its own callback.

    The name comes from provenance, not from the source: the entity's position
    in the node's creation order, ``lambda_subscription_<i>`` or
    ``lambda_timer_<i>``, the same index ``collect_rid_table`` emits as the
    RID's creation index, which ``ros2_lf_bind_node`` matches handles by. Runs
    after ``_analyze_callback_bodies`` so a member function the lambda calls
    already has its effects. Returns the lambda cursor per synthesized
    callback name, for the publish count.
    """
    bodies: dict[str, Any] = {}
    member_var_set = set(member_variables)
    resource_vars = (
        set(publisher_var_map) | set(svc_client_var_map) | set(act_client_var_map)
    )
    for family, entities in (("subscription", subscribers), ("timer", timers)):
        for index, entity in enumerate(entities):
            lambda_cursor = entity.__dict__.pop("_lambda_cursor", None)
            if lambda_cursor is None:
                continue
            name = f"lambda_{family}_{index}"
            suffix = 2
            while name in callbacks:
                name = f"lambda_{family}_{index}_{suffix}"
                suffix += 1
            cb_info = CallbackInfo(name=name, parameters=_lambda_parameters(lambda_cursor))
            callbacks[name] = cb_info
            entity.callback = name
            bodies[name] = lambda_cursor
            _analyze_lambda_body(
                lambda_cursor, cb_info, callbacks, member_var_set, resource_vars,
                publisher_var_map, svc_client_var_map, act_client_var_map,
            )
    return bodies


# ---------------------------------------------------------------------------
# Helper method body collector (Gap 1)
# ---------------------------------------------------------------------------

def _collect_class_method_bodies(
    class_cursor: Any,
    tu: Any,
    cpp_path: Path | None = None,
) -> dict[str, Any]:
    """Return a mapping of method name -> CXX_METHOD cursor for the class.

    Only includes methods defined in the primary source file (when *cpp_path*
    is given) to avoid pulling in header-only utility methods.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return {}

    cpp_resolved = cpp_path.resolve() if cpp_path else None
    own_files = _own_definition_files(cpp_resolved, class_cursor)
    methods: dict[str, Any] = {}

    for cursor in tu.cursor.walk_preorder():
        if cursor.kind != CursorKind.CXX_METHOD:
            continue
        if not _in_family(cursor.semantic_parent, class_cursor):
            continue
        if not cursor.is_definition():
            continue
        if cpp_resolved and cursor.location.file is not None:
            if Path(cursor.location.file.name).resolve() not in own_files:
                continue
        name = cursor.spelling
        if name and name not in methods:
            methods[name] = cursor
    return methods


# ---------------------------------------------------------------------------
# Gap 3: Typedef alias resolution
# ---------------------------------------------------------------------------

def _resolve_typedef_simple_type(cursor: Any, type_str: str) -> str | None:
    """Attempt to resolve a typedef/using alias to a known wrapper class name.

    When ``make_unique<DockingActionServer>(...)`` is used and
    ``DockingActionServer`` is a typedef for ``SimpleActionServer<T>``,
    this function returns ``"SimpleActionServer"``.

    Strategy:
    1. Use libclang's canonical type on the template argument.
    2. Fall back to scanning the translation unit for ``using`` aliases.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None

    simple = type_str.rsplit("::", 1)[-1]

    # Strategy 1: libclang canonical type
    try:
        n = cursor.get_num_template_arguments()
        if n is not None and n > 0:
            t = cursor.get_template_argument_type(0)
            if t is not None:
                canonical = t.get_canonical().spelling
                if canonical:
                    # Check if canonical contains a known wrapper name
                    for wrapper in (
                        _NAV2_ACTION_SERVER_WRAPPERS
                        | _NAV2_PUBLISHER_WRAPPERS
                        | _NAV2_SUBSCRIBER_WRAPPERS
                    ):
                        if wrapper in canonical:
                            return wrapper
    except Exception:
        pass

    # Strategy 2: scan TU for TYPE_ALIAS_DECL / TYPEDEF_DECL matching simple
    try:
        tu = cursor.translation_unit
        for desc in tu.cursor.walk_preorder():
            if desc.kind not in (CursorKind.TYPE_ALIAS_DECL, CursorKind.TYPEDEF_DECL):
                continue
            if desc.spelling != simple:
                continue
            # Get the underlying type spelling
            underlying = desc.underlying_typedef_type.spelling if desc.underlying_typedef_type else ""
            for wrapper in (
                _NAV2_ACTION_SERVER_WRAPPERS
                | _NAV2_PUBLISHER_WRAPPERS
                | _NAV2_SUBSCRIBER_WRAPPERS
            ):
                if wrapper in underlying:
                    return wrapper
    except Exception:
        pass

    return None


# ---------------------------------------------------------------------------
# Constructor body walker
# ---------------------------------------------------------------------------

def _walk_constructor(
    constructor_cursor: Any,
    callbacks: dict[str, CallbackInfo],
    class_method_bodies: dict[str, Any] | None = None,
    _helper_depth: int = 0,
    warnings_out: list[str] | None = None,
) -> tuple[
    list[PublisherInfo],
    list[SubscriberInfo],
    list[TimerInfo],
    list[ServiceServerInfo],
    list[ServiceClientInfo],
    list[ActionServerInfo],
    list[ActionClientInfo],
    list[CallbackGroupInfo],
    list[ParameterInfo],
    bool,          # has_param_callback
    str | None,    # param_callback_name
    dict[str, str],  # publisher_var_map: member_var -> topic
    dict[str, str],  # svc_client_var_map: member_var -> service_name
    dict[str, str],  # act_client_var_map: member_var -> action_name
]:
    """Walk the constructor AST and extract all ROS2 API calls."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return [], [], [], [], [], [], [], [], [], False, {}, {}, {}

    publishers: list[PublisherInfo] = []
    publisher_var_map: dict[str, str] = {}  # member_var_name -> topic
    svc_client_var_map: dict[str, str] = {}  # member_var_name -> service_name
    act_client_var_map: dict[str, str] = {}  # member_var_name -> action_name
    subscribers: list[SubscriberInfo] = []
    timers: list[TimerInfo] = []
    service_servers: list[ServiceServerInfo] = []
    service_clients: list[ServiceClientInfo] = []
    action_servers: list[ActionServerInfo] = []
    action_clients: list[ActionClientInfo] = []
    callback_groups: list[CallbackGroupInfo] = []
    parameters: list[ParameterInfo] = []
    has_param_callback = False
    param_callback_name: str | None = None

    # Map variable name -> callback group name (for direct 3rd-arg style)
    # e.g. exclusive_group_ is assigned to timer as 3rd argument
    cbg_var_names: dict[str, str] = {}  # var_spelling -> cbg_name

    # Phase 3.1: map options-struct variable -> callback group member name
    # e.g. {"sub_options": "exclusive_group_"} from:
    #   sub_options.callback_group = exclusive_group_;
    options_map: dict[str, str] = _build_options_map(constructor_cursor)

    def _note_nonliteral_topic(cursor: Any, api: str) -> None:
        # A topic name built from a variable, parameter, or expression cannot
        # be resolved statically; surface it so the user knows the extracted
        # graph may be missing this edge instead of failing silently.
        if warnings_out is None:
            return
        loc = getattr(cursor, "location", None)
        where = (
            f"{loc.file.name}:{loc.line}"
            if loc is not None and loc.file is not None
            else "unknown location"
        )
        msg = (
            f"{api} with non-literal topic name at {where}; "
            "the connection may be missing from the extracted graph"
        )
        if msg not in warnings_out:
            warnings_out.append(msg)

    def _note_unread_timer(cursor: Any, why: str) -> None:
        # A free rclcpp::create_timer the analyzer cannot read as
        # a timer of this node; the node then has no timer for it.
        if warnings_out is None:
            return
        loc = getattr(cursor, "location", None)
        where = (
            f"{loc.file.name}:{loc.line}"
            if loc is not None and loc.file is not None
            else "unknown location"
        )
        msg = f"rclcpp::create_timer at {where} not read as a timer of this node: {why}"
        if msg not in warnings_out:
            warnings_out.append(msg)

    def _resolve_topic_argument(cursor: Any, api: str, arg_index: int = 0) -> tuple[str, bool]:
        """Resolve a pub/sub topic-name argument. Returns ``(topic, is_dynamic)``.

        Two silent-failure modes are
        closed here:

        1. ``_first_string_literal`` returns the *first* literal in the whole
           argument subtree, so ``"prefix_" + suffix`` used to record
           ``"prefix_"`` as a complete topic name with **no** warning at all.
           The classifier now detects the extra operands, the best-effort
           literal is discarded (``topic = ""``), and the entry is flagged.
        2. The already-warned non-literal case only produced a stderr
           ``parse_warning``; nothing structured reached ``system.json``. The
           returned ``is_dynamic`` flag lands on the emitted
           ``PublisherInfo``/``SubscriberInfo`` as ``topic_dynamic`` and is
           surfaced as RCLLF-H004 by ``features/modules/dynamic_topic.py``.

        The human-readable ``parse_warning`` is kept unchanged for every
        dynamic case (analyze stderr behaviour is a superset of before).
        """
        literal = _topic_argument_literal(cursor, arg_index) or ""
        is_dynamic, _reason = _classify_topic_argument(cursor, arg_index)
        if not is_dynamic and literal:
            return literal, False
        if not is_dynamic:
            # Classified static but no name came out (an empty constant, or a
            # literal libclang gives no tokens for). An empty static topic would
            # match nothing and raise no H004, so report it as dynamic.
            _note_nonliteral_topic(cursor, api)
            return "", True
        _note_nonliteral_topic(cursor, api)
        # Keep a best-effort value only when it is the *whole* argument (the
        # `${var}` runtime-parameter placeholder from _first_string_literal's
        # last-resort branch); a partial literal from a concatenation is worse
        # than no name at all because it silently matches nothing downstream
        # while looking authoritative.
        if literal.startswith("${") and literal.endswith("}"):
            return literal, True
        return "", True

    # Fifth review: a publisher counts as created
    # unconditionally only when no branch, loop, lambda, try block or
    # short-circuit operand encloses its create call and no return, throw or
    # goto of the walked body comes before it. ``exited`` records the latter.
    # ``unsure``: a binary operator whose kind cannot be read
    # encloses the call, so the analysis does not decide (None).
    exited = False

    def _here_unconditional(cond: bool, unsure: bool = False) -> bool | None:
        if cond or exited:
            return False
        return None if unsure else True

    # Create calls whose result is assigned straight
    # to a member of this node, keyed by the call's extent.
    member_stores: dict[tuple, tuple[str, tuple]] = {}

    def _walk(cursor: Any, depth: int = 0, cond: bool = False,
              in_lambda: bool = False, unsure: bool = False) -> None:
        nonlocal has_param_callback, param_callback_name, exited

        if depth > 20:
            return

        if not in_lambda and cursor.kind in _EXIT_KINDS:
            exited = True
        if cursor.kind == CursorKind.LAMBDA_EXPR:
            in_lambda = True
        opens = _opens_condition(cursor)
        child_cond = cond or opens is True
        child_unsure = unsure or opens is None
        _note_member_store(cursor, member_stores)

        if cursor.kind == CursorKind.CALL_EXPR:
            name = cursor.spelling

            # ---- create_publisher ----------------------------------------
            if name == "create_publisher":
                topic, topic_dynamic = _resolve_topic_argument(cursor, "create_publisher")
                msg_type = _get_template_arg_type(cursor) or ""
                var_name = _infer_lhs_varname(cursor)
                qos_depth = _extract_qos_depth(cursor)
                qos_attrs = _materialize_qos_defaults(qos_depth, _extract_qos_attrs(cursor))
                publishers.append(PublisherInfo(
                    topic=topic, message_type=msg_type,
                    qos_depth=qos_depth,
                    member_variable=var_name,
                    qos_reliability=qos_attrs["reliability"],
                    qos_durability=qos_attrs["durability"],
                    qos_history=qos_attrs["history"],
                    qos_deadline_ms=qos_attrs["deadline_ms"],
                    topic_dynamic=topic_dynamic,
                    unconditional=_here_unconditional(cond, unsure),
                ))
                publishers[-1]._kept_in = member_stores.get(_extent_key(cursor))
                _note_topic_field(publishers[-1], cursor)
                if var_name and topic:
                    publisher_var_map[var_name] = topic
                return

            # ---- create_subscription -------------------------------------
            if name == "create_subscription":
                topic, topic_dynamic = _resolve_topic_argument(cursor, "create_subscription")
                msg_type = _get_template_arg_type(cursor) or ""
                # Phase 3.1: resolve callback_group via SubscriptionOptions
                # child layout: self + topic + qos + callback + options (optional)
                cbg_name: str | None = None
                sub_children = list(cursor.get_children())
                # An inline lambda gets a callback named by creation order after
                # the walk (_synthesize_lambda_callbacks). Only an
                # argument that is exactly one lambda or one bound member counts
                # (_callback_form); anything else is quoted for RCLLF-H007.
                cb_arg = sub_children[3] if len(sub_children) >= 4 else None
                form, found = _callback_form(cb_arg)
                lambda_cursor = found if form == "lambda" else None
                cb_name = found if form == "method" else None
                if len(sub_children) >= 5:
                    opts_child = sub_children[4]
                    for desc in opts_child.walk_preorder():
                        if desc.kind == CursorKind.DECL_REF_EXPR:
                            member_spelling = options_map.get(desc.spelling)
                            if member_spelling:
                                cbg_name = cbg_var_names.get(member_spelling)
                            break
                qos_depth_sub = _extract_qos_depth(cursor)
                qos_attrs_sub = _materialize_qos_defaults(
                    qos_depth_sub, _extract_qos_attrs(cursor)
                )
                sub_info = SubscriberInfo(
                    topic=topic,
                    message_type=msg_type,
                    callback=cb_name or "",
                    qos_depth=qos_depth_sub,
                    callback_group=cbg_name,
                    member_variable=_infer_lhs_varname(cursor),
                    qos_reliability=qos_attrs_sub["reliability"],
                    qos_durability=qos_attrs_sub["durability"],
                    qos_history=qos_attrs_sub["history"],
                    qos_deadline_ms=qos_attrs_sub["deadline_ms"],
                    topic_dynamic=topic_dynamic,
                    callback_expr=(
                        _callback_source(cb_arg)
                        if not cb_name and lambda_cursor is None else None
                    ),
                )
                if lambda_cursor is not None:
                    sub_info._lambda_cursor = lambda_cursor
                _note_topic_field(sub_info, cursor)
                subscribers.append(sub_info)
                return

            # ---- create_wall_timer / Node::create_timer ------------------
            # Node::create_timer(period, callback[, group]) has the same child
            # layout on the node's ROS clock. The free function
            # rclcpp::create_timer(node, clock, period, callback, ...) is read
            # by parameter name; a call for another node is
            # refused with a warning instead of being given to this node.
            if name == "create_wall_timer" or name == "create_timer":
                if name == "create_wall_timer" or _is_member_call(cursor):
                    # Child layout: self + period + callback + group (optional).
                    timer_children = list(cursor.get_children())
                    period_ms = _extract_timer_period(cursor)
                    cb_arg = timer_children[2] if len(timer_children) >= 3 else None
                    group_scope = cursor
                else:
                    free_args, why = _free_timer_arguments(cursor)
                    if free_args is None:
                        _note_unread_timer(cursor, why)
                        return
                    period_ms = _duration_ms(free_args["period"])
                    cb_arg = free_args["callback"]
                    group_scope = free_args.get("group")
                form, found = _callback_form(cb_arg)
                lambda_cursor = found if form == "lambda" else None
                cb_name = found if form == "method" else None
                # The callback group is an optional last argument. Detect it by
                # scanning the call's tokens for any known cbg variable name.
                # This is more robust than relying on a fixed child index.
                cbg_name: str | None = None
                if cbg_var_names and group_scope is not None:
                    timer_tokens = _cursor_tokens(group_scope)
                    for tok in timer_tokens:
                        if tok in cbg_var_names:
                            cbg_name = cbg_var_names[tok]
                            break
                timer_info = TimerInfo(
                    period_ms=period_ms,
                    callback=cb_name or "",
                    callback_group=cbg_name,
                    member_variable=_infer_lhs_varname(cursor),
                    offset_ms=_extract_timer_offset(cursor),
                    callback_expr=(
                        _callback_source(cb_arg)
                        if not cb_name and lambda_cursor is None else None
                    ),
                )
                if lambda_cursor is not None:
                    timer_info._lambda_cursor = lambda_cursor
                timers.append(timer_info)
                return

            # ---- create_service ------------------------------------------
            if name == "create_service":
                svc_name = _first_string_literal(cursor) or ""
                svc_type = _get_template_arg_type(cursor) or ""
                cb_name = _extract_callback_name_from_bind(cursor)
                service_servers.append(
                    ServiceServerInfo(
                        service_name=svc_name,
                        service_type=svc_type,
                        callback=cb_name or "",
                        callback_group=_extract_trailing_callback_group(cursor, cbg_var_names),
                        member_variable=_infer_lhs_varname(cursor),
                    )
                )
                return

            # ---- create_client (service OR action) -----------------------
            if name == "create_client":
                # Distinguish action client (rclcpp_action::create_client, a free
                # function) from service client (this->create_client, a member).
                # Heuristic: if the first child is a NAMESPACE_REF to rclcpp_action
                # or DECL_REF_EXPR (free function), it's an action client.
                is_action = _is_action_create_client(cursor)
                if is_action:
                    action_name = _first_string_literal(cursor) or ""
                    action_type = _get_template_arg_type(cursor) or ""
                    var_name = _infer_lhs_varname(cursor)
                    action_clients.append(
                        ActionClientInfo(
                            action_name=action_name,
                            action_type=action_type,
                            member_variable=var_name,
                            callback_group=_extract_trailing_callback_group(cursor, cbg_var_names),
                        )
                    )
                    if var_name and action_name:
                        act_client_var_map[var_name] = action_name
                else:
                    svc_name = _first_string_literal(cursor) or ""
                    svc_type = _get_template_arg_type(cursor) or ""
                    var_name = _infer_lhs_varname(cursor)
                    service_clients.append(
                        ServiceClientInfo(
                            service_name=svc_name,
                            service_type=svc_type,
                            member_variable=var_name,
                            callback_group=_extract_trailing_callback_group(cursor, cbg_var_names),
                        )
                    )
                    if var_name and svc_name:
                        svc_client_var_map[var_name] = svc_name
                return

            # ---- rclcpp_action::create_server ----------------------------
            if name == "create_server":
                action_name = _first_string_literal(cursor) or ""
                action_type = _get_template_arg_type(cursor) or ""
                # Extract callbacks: walk the full subtree for CXX_METHOD refs
                # (bind calls are wrapped in UNEXPOSED_EXPR layers).
                bind_cbs = _extract_all_method_refs(cursor)
                goal_cb = bind_cbs[0] if len(bind_cbs) > 0 else None
                cancel_cb = bind_cbs[1] if len(bind_cbs) > 1 else None
                accepted_cb = bind_cbs[2] if len(bind_cbs) > 2 else None
                action_servers.append(
                    ActionServerInfo(
                        action_name=action_name,
                        action_type=action_type,
                        goal_callback=goal_cb,
                        cancel_callback=cancel_cb,
                        accepted_callback=accepted_cb,
                        callback_group=_extract_trailing_callback_group(cursor, cbg_var_names),
                        member_variable=_infer_lhs_varname(cursor),
                    )
                )
                return

            # ---- create_callback_group -----------------------------------
            if name == "create_callback_group":
                cbg_type = _extract_enum_arg(cursor) or "Unknown"
                # Infer variable name: the assignment target is the parent
                # BINARY_OPERATOR or DECL_STMT / VAR_DECL.
                cbg_varname = _infer_lhs_varname(cursor)
                cbg_display = cbg_varname or f"cbg_{len(callback_groups)}"
                info = CallbackGroupInfo(name=cbg_display, type=cbg_type)
                callback_groups.append(info)
                if cbg_varname:
                    cbg_var_names[cbg_varname] = cbg_display
                return

            # ---- declare_parameter ---------------------------------------
            if name == "declare_parameter":
                param_name = _first_string_literal(cursor) or ""
                param_type = _get_template_arg_type(cursor) or ""
                # Simplify std:: prefix from type
                param_type = re.sub(r"^std::", "", param_type)
                # Extract default value from the second argument.
                # AST children: [0]=implicit self, [1]=param name, [2]=default
                default_value: Any | None = None
                children = list(cursor.get_children())
                if len(children) >= 3:
                    default_value = _extract_literal_value(children[2])
                if param_name:
                    parameters.append(
                        ParameterInfo(
                            name=param_name,
                            type=param_type,
                            default_value=default_value,
                        )
                    )
                return

            # ---- Gap 4: add_parameter (Nav2 pattern) --------------------
            if name == "add_parameter":
                param_name = _first_string_literal(cursor) or ""
                if param_name:
                    parameters.append(ParameterInfo(name=param_name, type=""))
                return

            # ---- add_on_set_parameters_callback --------------------------
            if name == "add_on_set_parameters_callback":
                has_param_callback = True
                param_callback_name = _extract_callback_name_from_bind(cursor) or None
                return

            # ---- Nav2 declare_parameter helpers --------------------------
            if name in _NAV2_DECLARE_PARAM_HELPERS or name == "declare_parameter_if_not_declared":
                param_name = _first_string_literal(cursor) or ""
                if param_name:
                    parameters.append(ParameterInfo(name=param_name, type=""))
                return

            # ---- Nav2 wrapper publisher/subscriber (make_unique<T>) ------
            # Handles: make_unique<TwistPublisher>(node, "topic", qos)
            #          make_unique<TwistSubscriber>(node, "topic", qos, cb)
            if name in ("make_unique", "make_shared"):
                type_str = _get_template_arg_type(cursor) or ""
                # Strip namespace qualifiers to get the simple class name
                simple_type = type_str.rsplit("::", 1)[-1]

                # Gap 3: Resolve typedef aliases — if the simple type is not
                # recognized, attempt to resolve via libclang canonical type
                # or token-based typedef scanning.
                if simple_type not in (
                    _NAV2_PUBLISHER_WRAPPERS | _NAV2_SUBSCRIBER_WRAPPERS
                    | _NAV2_ACTION_SERVER_WRAPPERS
                ):
                    resolved = _resolve_typedef_simple_type(cursor, type_str)
                    if resolved:
                        simple_type = resolved

                if simple_type in _NAV2_PUBLISHER_WRAPPERS:
                    # Nav2 wrappers take (node, topic, ...), so the topic is
                    # argument index 1.
                    topic, topic_dynamic = _resolve_topic_argument(
                        cursor, simple_type, arg_index=1)
                    var_name = _infer_lhs_varname(cursor)
                    # Gap 5: use known wrapper message type
                    msg_type = _WRAPPER_MESSAGE_TYPES.get(simple_type, "")
                    publishers.append(PublisherInfo(
                        topic=topic, message_type=msg_type,
                        member_variable=var_name,
                        topic_dynamic=topic_dynamic,
                        unconditional=_here_unconditional(cond, unsure)))
                    publishers[-1]._kept_in = member_stores.get(_extent_key(cursor))
                    if var_name and topic:
                        publisher_var_map[var_name] = topic
                    return
                if simple_type in _NAV2_SUBSCRIBER_WRAPPERS:
                    # See the publisher-wrapper note above: topic is index 1.
                    topic, topic_dynamic = _resolve_topic_argument(
                        cursor, simple_type, arg_index=1)
                    cb_name = _extract_callback_name_from_bind(cursor)
                    var_name = _infer_lhs_varname(cursor)
                    # Gap 5: use known wrapper message type
                    msg_type = _WRAPPER_MESSAGE_TYPES.get(simple_type, "")
                    subscribers.append(SubscriberInfo(
                        topic=topic, message_type=msg_type,
                        callback=cb_name or "",
                        member_variable=var_name,
                        topic_dynamic=topic_dynamic))
                    return
                # Nav2 SimpleActionServer wrapper:
                # make_unique<SimpleActionServer<ActionType>>(node, action_name, cb, ...)
                if simple_type in _NAV2_ACTION_SERVER_WRAPPERS:
                    action_type = _get_template_arg_type(cursor) or ""
                    action_name = _first_string_literal(cursor) or ""
                    execute_cb = _extract_callback_name_from_bind(cursor)
                    var_name = _infer_lhs_varname(cursor)
                    action_servers.append(
                        ActionServerInfo(
                            action_name=action_name,
                            action_type=action_type,
                            accepted_callback=execute_cb,
                            member_variable=var_name,
                        )
                    )
                    return

            # ---- Gap 1: Helper method traversal -------------------------
            # If this is a call to a same-class method (this->initPubSub()
            # or just initPubSub()), recursively walk that method body.
            if class_method_bodies and _helper_depth < 3:
                callee_name = cursor.spelling
                if callee_name in class_method_bodies and callee_name not in _LIFECYCLE_METHODS:
                    helper_cursor = class_method_bodies[callee_name]
                    (
                        h_pubs, h_subs, h_timers,
                        h_srv_servers, h_srv_clients,
                        h_act_servers, h_act_clients,
                        h_cbg, h_params, h_has_pcb, h_pcb_name,
                        h_pub_map, h_svc_map, h_act_map,
                    ) = _walk_constructor(
                        helper_cursor, callbacks,
                        class_method_bodies=class_method_bodies,
                        _helper_depth=_helper_depth + 1,
                    )
                    here = _here_unconditional(cond, unsure)
                    for p in h_pubs:
                        if here is False:
                            # The helper itself runs only on some paths.
                            p.unconditional = False
                        elif here is None and p.unconditional is True:
                            p.unconditional = None
                    publishers.extend(h_pubs)
                    subscribers.extend(h_subs)
                    timers.extend(h_timers)
                    service_servers.extend(h_srv_servers)
                    service_clients.extend(h_srv_clients)
                    action_servers.extend(h_act_servers)
                    action_clients.extend(h_act_clients)
                    callback_groups.extend(h_cbg)
                    parameters.extend(h_params)
                    if h_has_pcb:
                        has_param_callback = True
                    if h_pcb_name and not param_callback_name:
                        param_callback_name = h_pcb_name
                    publisher_var_map.update(h_pub_map)
                    svc_client_var_map.update(h_svc_map)
                    act_client_var_map.update(h_act_map)
                    # Don't return — still recurse children for nested calls

        # Recurse
        for child in cursor.get_children():
            _walk(child, depth + 1, child_cond, in_lambda, child_unsure)

    _walk(constructor_cursor)
    _undecide_after_goto(constructor_cursor, publishers)
    if _helper_depth == 0:
        _undecide_unkept_publishers(constructor_cursor, publishers)

    # --- Gap 2: Resolve ${variable} topic/service/action names from params ---
    _resolve_placeholders_from_params(
        publishers, subscribers, service_servers, service_clients,
        action_servers, action_clients, parameters,
        publisher_var_map, svc_client_var_map, act_client_var_map,
    )

    return (
        publishers, subscribers, timers,
        service_servers, service_clients,
        action_servers, action_clients,
        callback_groups, parameters, has_param_callback, param_callback_name,
        publisher_var_map, svc_client_var_map, act_client_var_map,
    )


def _resolve_placeholders_from_params(
    publishers: list[PublisherInfo],
    subscribers: list[SubscriberInfo],
    service_servers: list[ServiceServerInfo],
    service_clients: list[ServiceClientInfo],
    action_servers: list[ActionServerInfo],
    action_clients: list[ActionClientInfo],
    parameters: list[ParameterInfo],
    publisher_var_map: dict[str, str],
    svc_client_var_map: dict[str, str],
    act_client_var_map: dict[str, str],
) -> None:
    """Resolve ``${variable}`` placeholders in topic/service/action names.

    When a topic name is ``${speed_limit_topic}`` and there is a declared
    parameter named ``speed_limit_topic`` with a string default value, replace
    the placeholder with the parameter's default value.
    """
    # Build a lookup: param_name -> default string value
    param_defaults: dict[str, str] = {}
    for p in parameters:
        if p.default_value is not None and isinstance(p.default_value, str):
            param_defaults[p.name] = p.default_value

    if not param_defaults:
        return

    placeholder_re = re.compile(r"^\$\{(\w+)\}$")

    def _try_resolve(name: str) -> str:
        m = placeholder_re.match(name)
        if m:
            var = m.group(1)
            # Match parameter name directly or with common suffixes stripped
            if var in param_defaults:
                return param_defaults[var]
            # Try matching: variable "cmd_vel_in_topic" -> param "cmd_vel_in_topic"
            for pname, pval in param_defaults.items():
                if pname == var or pname.replace("_topic", "") == var.replace("_topic", ""):
                    return pval
        return name

    for pub in publishers:
        old = pub.topic
        pub.topic = _try_resolve(pub.topic)
        if old != pub.topic:
            # The name resolved after all — clear the RCLLF-H004 flag so the
            # diagnostic does not fire on an endpoint whose topic *is* known.
            pub.topic_dynamic = False
        if pub.member_variable and old != pub.topic:
            publisher_var_map[pub.member_variable] = pub.topic

    for sub in subscribers:
        old_sub_topic = sub.topic
        sub.topic = _try_resolve(sub.topic)
        if old_sub_topic != sub.topic:
            sub.topic_dynamic = False

    for srv in service_servers:
        srv.service_name = _try_resolve(srv.service_name)

    for cli in service_clients:
        old = cli.service_name
        cli.service_name = _try_resolve(cli.service_name)
        if cli.member_variable and old != cli.service_name:
            svc_client_var_map[cli.member_variable] = cli.service_name

    for srv in action_servers:
        srv.action_name = _try_resolve(srv.action_name)

    for cli in action_clients:
        old = cli.action_name
        cli.action_name = _try_resolve(cli.action_name)
        if cli.member_variable and old != cli.action_name:
            act_client_var_map[cli.member_variable] = cli.action_name


def _infer_lhs_varname(call_cursor: Any) -> str | None:
    """Try to infer the variable name that the result of *call_cursor* is assigned to.

    Walks up via the parent's token stream to find an assignment like
    ``exclusive_group_ = create_callback_group(...)`` and returns
    ``exclusive_group_``.
    """
    # Heuristic: read the source text on the call's own line, before the call,
    # and look for a trailing assignment.
    #
    # Sliced as BYTES, not as a str. `extent.start.offset` is a libclang byte
    # offset, so indexing a decoded `str` with it silently skews by
    # (bytes - characters) for every multi-byte character earlier in the file.
    # One em dash in a comment (3 bytes, 1 character) shifts every later offset
    # by 2, which turned `  timer_ = ` into `  timer_ = th` and made this
    # return None for every call after it in the file. That is a silent
    # extraction failure, not a crash: the resource is still recorded, just
    # with `member_variable=None`, so `_detect_resets` can no longer match it
    # and `destroyed_in` stays unset.
    start_offset = call_cursor.extent.start.offset
    if start_offset is None:
        return None
    try:
        raw = Path(call_cursor.translation_unit.spelling).read_bytes()
        # Look back to the start of the current *statement* (the nearest
        # preceding ';', '{' or '}'), not just the current line: an
        # assignment like ``cancel_timer_ =\n    this->create_wall_timer(...)``
        # (action_client_node.cpp) splits the variable name onto
        # the previous line, which a same-line-only prefix never sees. Safe
        # to widen unconditionally — the regex below is anchored to the end
        # of the prefix, so any unrelated code further back before the
        # statement boundary cannot change the match.
        stmt_start = max(
            raw.rfind(b";", 0, start_offset),
            raw.rfind(b"{", 0, start_offset),
            raw.rfind(b"}", 0, start_offset),
        ) + 1
        prefix = raw[stmt_start:start_offset].decode("utf-8", errors="replace")
        # Find assignment: "  exclusive_group_ = "
        m = re.search(r"(\w+)\s*=\s*$", prefix.strip())
        if m:
            return m.group(1)
    except Exception:
        pass
    return None


# ---------------------------------------------------------------------------
# Action client detection (separate pass — not in constructor)
# ---------------------------------------------------------------------------

_QUALIFIED_ACTION = re.compile(r"^(?:::)?(\w+::action::[A-Za-z]\w*)$")
_ACTION_CLIENT_WRAPPER = re.compile(r"^(?:::)?rclcpp_action::Client<(.+)>$")


def _alias_declarations(tu: Any) -> dict[str, list[tuple[Any, str]]]:
    """Map each ``using``/``typedef`` alias name to (declaring scope, aliased text).

    The aliased text is read from the declaration's tokens, so it is the
    spelling in the source even when libclang could not resolve the type.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return {}
    aliases: dict[str, list[tuple[Any, str]]] = {}
    for desc in tu.cursor.walk_preorder():
        if desc.kind not in (CursorKind.TYPE_ALIAS_DECL, CursorKind.TYPEDEF_DECL):
            continue
        tokens = _cursor_tokens(desc)
        if tokens and tokens[-1] == ";":
            tokens = tokens[:-1]
        if desc.kind == CursorKind.TYPE_ALIAS_DECL:
            if "=" not in tokens:
                continue
            text = "".join(tokens[tokens.index("=") + 1:])
        else:
            if len(tokens) < 3 or tokens[0] != "typedef" or tokens[-1] != desc.spelling:
                continue
            text = "".join(tokens[1:-1])
        aliases.setdefault(desc.spelling, []).append((desc.semantic_parent, text))
    return aliases


def _scope_name(cursor: Any) -> str:
    """The qualified name of a class or namespace cursor, ``""`` for the TU."""
    parts: list[str] = []
    while cursor is not None and cursor.semantic_parent is not None:
        parts.append(cursor.spelling)
        cursor = cursor.semantic_parent
    return "::".join(reversed(parts))


def _resolve_action_type_aliases(
    clients: list[ActionClientInfo], class_cursor: Any, tu: Any,
) -> None:
    """Replace an action client's alias type with the qualified type it names.

    A name is looked up the way C++ looks it up, from the node's
    class outward through its enclosing namespaces, and a qualified name
    ``a::Name`` only in the scope ``a`` names from there. A name found in
    an alias is looked up again from that alias's scope. Aliases chain, and
    ``rclcpp_action::Client<T>`` resolves to ``T``. A type with no alias in
    those scopes (one declared in a base class, a using-declaration) is left
    unchanged, so the generator refuses it rather than take a same-named
    alias from an unrelated scope.
    """
    if not any(c.action_type and not _QUALIFIED_ACTION.match(c.action_type)
               for c in clients):
        return
    aliases = _alias_declarations(tu)

    def enclosing(scope: Any) -> list[Any]:
        chain: list[Any] = []
        while scope is not None and len(chain) < 32:
            chain.append(scope)
            scope = scope.semantic_parent
        return chain

    def lookup(name: str, start: Any) -> tuple[str, Any] | None:
        global_only = name.startswith("::")
        *qualifier, leaf = name.lstrip(":").split("::")
        found = aliases.get(leaf, [])
        qual = "::".join(qualifier)
        for scope in [tu.cursor] if global_only else enclosing(start):
            if qual:
                outer = _scope_name(scope)
                want = f"{outer}::{qual}" if outer else qual
                hit = [(p, t) for p, t in found if _scope_name(p) == want]
            else:
                hit = [(p, t) for p, t in found if p == scope]
            if hit:
                return hit[0][1], hit[0][0]
        return None

    for cli in clients:
        current = cli.action_type or ""
        start = class_cursor
        for _ in range(8):
            if _QUALIFIED_ACTION.match(current):
                break
            wrapper = _ACTION_CLIENT_WRAPPER.match(current)
            if wrapper:
                current = wrapper.group(1)
                continue
            target = lookup(current, start)
            if target is None:
                break
            current, start = target
        m = _QUALIFIED_ACTION.match(current)
        if m:
            cli.action_type = m.group(1)


def _find_action_clients(tu: Any, class_cursor: Any) -> list[ActionClientInfo]:
    """Scan all methods of the class for rclcpp_action::create_client calls."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []

    clients: list[ActionClientInfo] = []

    for cursor in tu.cursor.walk_preorder():
        if cursor.kind != CursorKind.CALL_EXPR:
            continue
        if cursor.spelling != "create_client":
            continue
        # Verify it is inside our node class
        parent = cursor.semantic_parent
        while parent is not None:
            if parent == class_cursor:
                break
            parent = parent.semantic_parent
        else:
            continue
        action_name = _first_string_literal(cursor) or ""
        action_type = _get_template_arg_type(cursor) or ""
        clients.append(
            ActionClientInfo(action_name=action_name, action_type=action_type)
        )

    return clients


# ---------------------------------------------------------------------------
# Timers created outside the constructor/lifecycle hooks
# ---------------------------------------------------------------------------

def _find_main_outside_callbacks(
    tu: Any, class_cursor: Any,
) -> list[dict[str, str]]:
    """Node work ``main()`` does itself instead of in an executor callback.

    A call to a method the node class (or a workspace base) defines, or a
    ``publish`` on a publisher, made in ``main()`` outside any lambda. The
    stock ``spin``/``spin_some``/``add_node``/``ok``/``shutdown``/``init``,
    executor construction and parameter reads are rclcpp's own functions and
    never count. rcllf replaces ``main()``, so this work never runs
    (RCLLF-H014).
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    main_file = tu.spelling
    found: list[dict[str, str]] = []

    def visit(cursor: Any) -> None:
        for child in cursor.get_children():
            if child.kind == CursorKind.LAMBDA_EXPR:
                continue
            if child.kind == CursorKind.CALL_EXPR:
                ref = child.referenced
                if ref is not None and ref.kind == CursorKind.CXX_METHOD:
                    parent = ref.semantic_parent
                    name = None
                    if _in_family(parent, class_cursor):
                        name = ref.spelling
                    elif ref.spelling == "publish" and parent is not None and \
                            parent.spelling in ("Publisher", "LifecyclePublisher"):
                        name = "publish"
                    if name is not None:
                        entry = {"source": Path(main_file).name, "call": name}
                        if entry not in found:
                            found.append(entry)
            visit(child)

    for top in tu.cursor.get_children():
        if top.kind != CursorKind.FUNCTION_DECL or top.spelling != "main":
            continue
        if not top.is_definition():
            continue
        if top.location.file is None or top.location.file.name != main_file:
            continue
        visit(top)
    return found


def _called_method_names(cursor: Any) -> set[str]:
    """Names of the member functions called anywhere under *cursor*."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return set()
    names: set[str] = set()
    for c in cursor.walk_preorder():
        if c.kind == CursorKind.CALL_EXPR and c.spelling:
            names.add(c.spelling)
    return names


def _setup_scope_methods(
    roots: list[Any], methods: dict[str, Any], max_depth: int = 3,
) -> set[str]:
    """Methods the constructor and lifecycle hooks run, helpers included.

    ``_walk_constructor`` follows a helper the constructor calls (up to three
    levels), so an endpoint created there is a constructor endpoint.
    """
    scope: set[str] = set()
    frontier = [r for r in roots if r is not None]
    for _ in range(max_depth):
        nxt: list[Any] = []
        for root in frontier:
            for name in _called_method_names(root):
                if name in methods and name not in scope:
                    scope.add(name)
                    nxt.append(methods[name])
        frontier = nxt
    return scope


def _lambda_description(call: Any) -> str | None:
    """``lambda at file.cpp:12`` for a create_* call whose callback is a lambda."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return None
    for c in call.walk_preorder():
        if c.kind == CursorKind.LAMBDA_EXPR and c.location.file is not None:
            return f"lambda at {Path(c.location.file.name).name}:{c.location.line}"
    return None


def _out_of_scope_creations(
    tu: Any,
    class_cursor: Any,
    lifecycle_callbacks: list[str],
    scope_roots: list[Any],
    methods: dict[str, Any],
) -> list[tuple[Any, str]]:
    """``(create_* call, method name)`` for each creation outside the setup scope.

    Enumerates the family's own ``CXX_METHOD`` definitions top-down and walks
    each one's subtree for the call, rather than climbing back up via
    ``semantic_parent`` — a CALL_EXPR's ``semantic_parent`` is unreliable.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    in_scope = _setup_scope_methods(scope_roots, methods)
    found: list[tuple[Any, str]] = []
    for method_cursor in tu.cursor.walk_preorder():
        if method_cursor.kind != CursorKind.CXX_METHOD:
            continue
        if not _in_family(method_cursor.semantic_parent, class_cursor):
            continue
        if not method_cursor.is_definition():
            continue
        method_name = method_cursor.spelling
        if method_name in lifecycle_callbacks or method_name in in_scope:
            continue  # walked by the constructor or lifecycle-hook pass
        for cursor in method_cursor.walk_preorder():
            if cursor.kind == CursorKind.CALL_EXPR and cursor.spelling in _ENDPOINT_CREATE_CALLS:
                found.append((cursor, method_name))
    return found


def _find_out_of_scope_timers(
    tu: Any,
    class_cursor: Any,
    lifecycle_callbacks: list[str],
    cbg_var_names: dict[str, str] | None = None,
    scope_roots: list[Any] | None = None,
    methods: dict[str, Any] | None = None,
) -> list["TimerInfo"]:
    """``create_wall_timer`` calls in a method that is neither the
    constructor nor a recognized lifecycle hook.

    Those two places (and the helpers they call) are the only ones the
    parser walks for resource creation and, not coincidentally, the only ones
    ``ros2_lf_bind_node`` can ever observe: the runtime drives a lifecycle
    node's configure/activate transitions *before* its single, one-shot
    handle-table collection pass (``node_binding.cpp``). A timer created
    inside an arbitrary member function (e.g. a goal-accepted callback) is
    created later, at runtime, so it is never in scope for that pass.
    """
    out_of_scope: list[TimerInfo] = []
    for cursor, method_name in _out_of_scope_creations(
            tu, class_cursor, lifecycle_callbacks, scope_roots or [], methods or {}):
        if cursor.spelling != "create_wall_timer":
            continue
        period_ms = _extract_timer_period(cursor)
        cb_name = _lambda_description(cursor) or _extract_callback_name_from_bind(cursor)
        cbg_name: str | None = None
        if cbg_var_names:
            for tok in _cursor_tokens(cursor):
                if tok in cbg_var_names:
                    cbg_name = cbg_var_names[tok]
                    break
        out_of_scope.append(
            TimerInfo(
                period_ms=period_ms,
                callback=cb_name or "",
                callback_group=cbg_name,
                member_variable=_infer_lhs_varname(cursor),
                created_in=method_name,
                offset_ms=_extract_timer_offset(cursor),
            )
        )
    return out_of_scope


_ENDPOINT_KIND = {
    "create_publisher": "publisher",
    "create_subscription": "subscription",
    "create_service": "service",
    "create_client": "client",
    "create_generic_publisher": "generic publisher",
    "create_generic_subscription": "generic subscription",
}


def _find_unbound_endpoints(
    tu: Any,
    class_cursor: Any,
    lifecycle_callbacks: list[str],
    scope_roots: list[Any],
    methods: dict[str, Any],
) -> list[dict[str, str]]:
    """Publisher, subscription, service and client creations outside the setup scope.

    The runtime binds the endpoints that exist after the constructor, so one
    created later is never part of the generated system (RCLLF-H013).
    """
    out: list[dict[str, str]] = []
    for cursor, method_name in _out_of_scope_creations(
            tu, class_cursor, lifecycle_callbacks, scope_roots, methods):
        kind = _ENDPOINT_KIND.get(cursor.spelling)
        if kind is None:
            continue
        out.append({
            "kind": kind,
            "topic": _first_string_literal(cursor) or "(dynamic name)",
            "method": method_name,
            "callback": (
                _lambda_description(cursor)
                or _extract_callback_name_from_bind(cursor) or ""
            ),
        })
    return out


# ---------------------------------------------------------------------------
# Member variable extraction
# ---------------------------------------------------------------------------

def _extract_member_variables(class_cursor: Any) -> list[str]:
    """Return the names of member (field) variables declared in the node class."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    result: list[str] = []
    for cls in [*_class_family(class_cursor)[0], class_cursor]:
        for child in cls.get_children():
            if child.kind == CursorKind.FIELD_DECL:
                name = child.spelling
                if name and name not in result:
                    result.append(name)
    return result


# ---------------------------------------------------------------------------
# Lifecycle base-call chaining detection (RCLLF-H005)
# ---------------------------------------------------------------------------

def _hook_chains_to_base(method_cursor: Any, hook_name: str) -> bool:
    """Whether a lifecycle hook override calls the base implementation.

    Token-based rather than AST-based, deliberately. The same unresolved-header
    conditions that force ``is_lifecycle_base`` to use a broad substring check
    apply here: with a missing third-party base header, libclang cannot resolve
    the callee of ``nav2_util::LifecycleNode::on_activate(state)`` to a
    declaration, so an AST search for a resolved call would report "no base
    call" for code that has one. A qualified call is recognisable purely
    syntactically as ``::`` immediately followed by the hook name, which
    matches ``LifecycleNode::on_activate``,
    ``rclcpp_lifecycle::LifecycleNode::on_activate`` and
    ``nav2_util::LifecycleNode::on_activate`` alike.

    Only the **body** is scanned. An out-of-line definition's own declarator
    (``HelperNode::on_activate``) is itself ``::`` followed by the hook name,
    so scanning the whole cursor extent would call every such definition
    chained and the check would never fire.

    Fails open: any trouble locating or tokenizing the body returns True, so a
    parse problem cannot manufacture a diagnostic.
    """
    try:
        from clang.cindex import CursorKind

        body = None
        for child in method_cursor.get_children():
            if child.kind == CursorKind.COMPOUND_STMT:
                body = child
                break
        if body is None:
            return True

        tokens = [t.spelling for t in body.get_tokens()]
        for i in range(1, len(tokens)):
            if tokens[i] == hook_name and tokens[i - 1] == "::":
                return True
        return False
    except Exception:
        return True


# ---------------------------------------------------------------------------
# Lifecycle reset detection
# ---------------------------------------------------------------------------

def _detect_resets(method_cursor: Any, method_name: str, resources: list) -> None:
    """Walk a lifecycle method body and tag resources whose .reset() is called.

    When ``timer_.reset()`` is found, the resource whose *member_variable*
    matches ``timer_`` gets ``destroyed_in = method_name``.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return

    # Build a map from member variable name to resource object
    var_to_resource = {
        r.member_variable: r
        for r in resources
        if getattr(r, "member_variable", None)
    }
    if not var_to_resource:
        return

    for desc in method_cursor.walk_preorder():
        if desc.kind != CursorKind.MEMBER_REF_EXPR:
            continue
        if desc.spelling != "reset":
            continue
        # Tokens of a ".reset()" MEMBER_REF_EXPR include the receiver tokens too
        tokens = _cursor_tokens(desc)
        for tok in tokens:
            if tok in var_to_resource:
                r = var_to_resource[tok]
                if r.destroyed_in is None:
                    r.destroyed_in = method_name
                break


# ---------------------------------------------------------------------------
# Callback body analysis
# ---------------------------------------------------------------------------

def _analyze_callback_bodies(
    class_cursor: Any,
    tu: Any,
    cpp_path: Path,
    publisher_var_map: dict[str, str],
    svc_client_var_map: dict[str, str],
    act_client_var_map: dict[str, str],
    member_variables: list[str],
    callbacks: dict[str, CallbackInfo],
    action_servers: list | None = None,
    action_clients: list | None = None,
    service_clients: list | None = None,
) -> None:
    """Analyze callback method bodies to populate publishers_used, service/action clients, and variables.

    Mutates the CallbackInfo objects in *callbacks* in-place.
    Also populates spawns_callbacks, feedback_published, result_published on CallbackInfo,
    goal_response_callback/feedback_callback/result_callback on ActionClientInfo,
    and response_callback on ServiceClientInfo.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return

    member_var_set = set(member_variables)
    # Publisher/client member variables accessed via -> are not data reads
    _resource_vars = (
        set(publisher_var_map.keys())
        | set(svc_client_var_map.keys())
        | set(act_client_var_map.keys())
    )

    # Build reverse map: accepted/execute callback name -> action name
    # Used to resolve publish_feedback/succeed/canceled to an action name.
    _cb_to_action: dict[str, str] = {}
    for srv in (action_servers or []):
        act = srv.action_name
        for cb_field in (srv.goal_callback, srv.cancel_callback, srv.accepted_callback):
            if cb_field:
                _cb_to_action[cb_field] = act

    # SendGoalOptions field names -> ActionClientInfo field to populate
    _SEND_GOAL_OPTION_FIELDS = {
        "goal_response_callback": "goal_response_callback",
        "feedback_callback":      "feedback_callback",
        "result_callback":        "result_callback",
    }
    # Build map: action_client member variable name -> ActionClientInfo object
    _act_var_to_client: dict[str, Any] = {}
    for cli in (action_clients or []):
        if cli.member_variable:
            _act_var_to_client[cli.member_variable] = cli

    # Build map: service_client member variable name -> ServiceClientInfo object
    _svc_var_to_client: dict[str, Any] = {}
    for cli in (service_clients or []):
        if cli.member_variable:
            _svc_var_to_client[cli.member_variable] = cli

    for cursor in tu.cursor.walk_preorder():
        if cursor.kind != CursorKind.CXX_METHOD:
            continue
        if not _in_family(cursor.semantic_parent, class_cursor):
            continue
        if not cursor.is_definition():
            continue
        cb_name = cursor.spelling
        if cb_name not in callbacks:
            continue

        cb_info = callbacks[cb_name]
        pubs_used: set[str] = set()
        svcs_used: set[str] = set()
        acts_used: set[str] = set()
        vars_read: set[str] = set()
        vars_written: set[str] = set()

        # Walk the method body
        for desc in cursor.walk_preorder():
            if desc.kind == CursorKind.MEMBER_REF_EXPR:
                member_name = desc.spelling
                tokens = _cursor_tokens(desc)

                # Detect publisher_->publish() calls (and a tf2 broadcaster's
                # sendTransform)
                if member_name in _PUBLISH_CALL_NAMES:
                    for tok in tokens:
                        if tok in publisher_var_map:
                            pubs_used.add(publisher_var_map[tok])
                            break

                # Detect client_->async_send_request() calls
                elif member_name == "async_send_request":
                    for tok in tokens:
                        if tok in svc_client_var_map:
                            svcs_used.add(svc_client_var_map[tok])
                            break

                # Detect client_->async_send_goal() / async_cancel_goal() calls
                elif member_name in ("async_send_goal", "async_cancel_goal"):
                    for tok in tokens:
                        if tok in act_client_var_map:
                            acts_used.add(act_client_var_map[tok])
                            break

                # Detect goal_handle->publish_feedback() — action server feedback
                elif member_name == "publish_feedback":
                    action_name = _cb_to_action.get(cb_name, "?")
                    cb_info.feedback_published.append(action_name)

                # Detect goal_handle->succeed() / goal_handle->canceled() / succeeded_current()
                elif member_name in ("succeed", "canceled", "succeeded_current",
                                     "terminate_current", "abort"):
                    action_name = _cb_to_action.get(cb_name, "?")
                    if action_name not in cb_info.result_published:
                        cb_info.result_published.append(action_name)

                # Detect SendGoalOptions field assignments:
                # send_goal_options.goal_response_callback = std::bind(...)
                # Defer to post-loop pass (needs method_tokens for act_var resolution)

                elif member_name == "reset":
                    # timer_.reset() / publisher_.reset() — treat as write
                    for tok in tokens:
                        if tok in member_var_set:
                            vars_written.add(tok)
                            break

                elif member_name in member_var_set and member_name not in _resource_vars:
                    token_str = " ".join(tokens)
                    if (
                        f"{member_name} ++" in token_str
                        or f"{member_name} --" in token_str
                        or f"++ {member_name}" in token_str
                        or f"-- {member_name}" in token_str
                    ):
                        vars_written.add(member_name)
                    else:
                        vars_read.add(member_name)

            if desc.kind == CursorKind.BINARY_OPERATOR:
                tokens = _cursor_tokens(desc)
                if "=" in tokens:
                    eq_idx = tokens.index("=")
                    for i in range(eq_idx):
                        if tokens[i] in member_var_set:
                            vars_written.add(tokens[i])

            if desc.kind == CursorKind.COMPOUND_ASSIGNMENT_OPERATOR:
                tokens = _cursor_tokens(desc)
                for tok in tokens:
                    if tok in member_var_set:
                        vars_written.add(tok)
                        break

            # Detect std::thread{std::bind(&Class::execute, ...)} spawning, and a
            # thread built in place: workers_.emplace_back(&Class::execute, this, gh)
            if desc.kind == CursorKind.CALL_EXPR and (
                    desc.spelling in ("thread", "jthread", "async")
                    or _is_thread_container_insert(desc)):
                spawned = _extract_all_method_refs(desc)
                for s in spawned:
                    if s in callbacks and s not in cb_info.spawns_callbacks:
                        cb_info.spawns_callbacks.append(s)

            # Detect client_->async_send_request(request, callback) — response callback
            if (desc.kind == CursorKind.CALL_EXPR
                    and desc.spelling == "async_send_request"
                    and _svc_var_to_client):
                _extract_svc_response_callback(
                    desc, _svc_var_to_client, svc_client_var_map, callbacks,
                    member_var_set, _resource_vars, publisher_var_map,
                    act_client_var_map,
                )

        # Robust increment/decrement/assignment detection via full method tokens
        method_tokens = _cursor_tokens(cursor)
        method_token_str = " ".join(method_tokens)
        for var in member_var_set:
            if f"{var} ++" in method_token_str or f"++ {var}" in method_token_str:
                vars_written.add(var)
            if f"{var} --" in method_token_str or f"-- {var}" in method_token_str:
                vars_written.add(var)
            # Detect assignment: var followed by "=" but not "==" (e.g. timer_ = create_wall_timer(...))
            for idx, tok in enumerate(method_tokens):
                if tok == var and idx + 1 < len(method_tokens):
                    next_tok = method_tokens[idx + 1]
                    if next_tok == "=" and (idx + 2 >= len(method_tokens) or method_tokens[idx + 2] != "="):
                        vars_written.add(var)
                        break

        cb_info.publishers_used = sorted(pubs_used)
        cb_info.service_clients_used = sorted(svcs_used)
        cb_info.action_clients_used = sorted(acts_used)
        cb_info.variables_written = sorted(vars_written)
        cb_info.variables_read = sorted(vars_read - vars_written)

        # --- SendGoalOptions binding pass (token-based) ---
        # Detect: send_goal_options.goal_response_callback = std::bind(&Class::cb, ...)
        # Pattern in token stream: <field_name> = ... <method_name> ...
        # We scan for each known SendGoalOptions field name in the token stream;
        # after the "=" we look for the first token that is a known callback name.
        if _act_var_to_client:
            for field_name, target_attr in _SEND_GOAL_OPTION_FIELDS.items():
                # Find all positions of this field name in method tokens
                for tok_idx, tok in enumerate(method_tokens):
                    if tok != field_name:
                        continue
                    # Confirm next token is "=" (assignment, not comparison)
                    if tok_idx + 1 >= len(method_tokens):
                        continue
                    if method_tokens[tok_idx + 1] != "=":
                        continue
                    if tok_idx + 2 < len(method_tokens) and method_tokens[tok_idx + 2] == "=":
                        continue  # skip "=="
                    # Scan after "=" for a known callback name
                    bound_cb: str | None = None
                    for rhs_tok in method_tokens[tok_idx + 2:]:
                        if rhs_tok in callbacks:
                            bound_cb = rhs_tok
                            break
                        if rhs_tok == ";":
                            break
                    if not bound_cb:
                        continue
                    # Assign to the matching action client
                    matched = False
                    for act_var, cli_obj in _act_var_to_client.items():
                        if act_var in method_tokens:
                            if not getattr(cli_obj, target_attr, None):
                                setattr(cli_obj, target_attr, bound_cb)
                            matched = True
                            break
                    if not matched and len(_act_var_to_client) == 1:
                        cli_obj = next(iter(_act_var_to_client.values()))
                        if not getattr(cli_obj, target_attr, None):
                            setattr(cli_obj, target_attr, bound_cb)

    # --- Fix up "?" action names by following spawns_callbacks transitively ---
    # _cb_to_action only includes direct action server callbacks (goal/cancel/accepted).
    # Spawned callbacks (e.g. execute via std::thread) are not in the map yet.
    for cb_name_key, act_name in list(_cb_to_action.items()):
        if cb_name_key in callbacks:
            for spawned in callbacks[cb_name_key].spawns_callbacks:
                if spawned not in _cb_to_action:
                    _cb_to_action[spawned] = act_name

    # Second pass: replace any "?" in feedback_published/result_published. A
    # callback no server callback reaches (a step timer) publishes
    # on the node's only action server when it has one, and stays "?" when it
    # has several (the generator then claims every server of the node).
    only_server = (
        action_servers[0].action_name if action_servers and len(action_servers) == 1 else "?")
    for cb_info in callbacks.values():
        cb_info.feedback_published = list(dict.fromkeys(
            _cb_to_action.get(cb_info.name, only_server) if a == "?" else a
            for a in cb_info.feedback_published
        ))
        cb_info.result_published = list(dict.fromkeys(
            _cb_to_action.get(cb_info.name, only_server) if a == "?" else a
            for a in cb_info.result_published
        ))


# ---------------------------------------------------------------------------
# Action handles used from a thread of the node's own (RCLLF-H011)
# ---------------------------------------------------------------------------

# Goal handle and SimpleActionServer members that send feedback or a result.
_ACTION_SEND_MEMBERS = frozenset({
    "publish_feedback", "succeed", "canceled", "abort",
    "succeeded_current", "terminate_current",
})


def _is_thread_start(desc: Any) -> bool:
    """Whether *desc* starts a thread: std::thread, std::jthread, std::async or a container insert.

    libclang reports a constructor call as CALL_EXPR, so a named
    ``std::thread t([this] { ... })`` and a temporary ``std::thread(...)``
    both match.
    """
    from clang.cindex import CursorKind
    return desc.kind == CursorKind.CALL_EXPR and (
        desc.spelling in ("thread", "jthread", "async") or _is_thread_container_insert(desc))


def _sends_on_action_handle(
    cursor: Any, method_bodies: dict[str, Any], depth: int = 0,
    seen: set[str] | None = None,
) -> bool:
    """Whether the code under *cursor*, or a method of the class it calls, sends
    action feedback or a result. Calls are followed three levels deep."""
    from clang.cindex import CursorKind
    seen = set() if seen is None else seen
    for desc in cursor.walk_preorder():
        if desc.kind == CursorKind.MEMBER_REF_EXPR and desc.spelling in _ACTION_SEND_MEMBERS:
            return True
        if depth >= 3 or desc.kind != CursorKind.CALL_EXPR:
            continue
        callee = desc.referenced
        name = callee.spelling if callee is not None else ""
        if (callee is not None and callee.kind == CursorKind.CXX_METHOD
                and name in method_bodies and name not in seen):
            seen.add(name)
            if _sends_on_action_handle(method_bodies[name], method_bodies, depth + 1, seen):
                return True
    return False


def _find_thread_action_senders(
    constructor_cursor: Any,
    method_bodies: dict[str, Any],
    action_servers: list,
) -> list[dict[str, str]]:
    """Threads the node starts itself that send action feedback or results.

    Every method of the class and the constructor is searched for a thread
    start. Its target (a method of the class or a lambda) is searched for a
    feedback publish or a result call. Such a send has no LF origin, so under
    lfc it is refused at run time and RCLLF-H011 refuses generation. One entry
    per thread target: ``{"thread", "started_in", "action"}``. The action is
    the node's only action server, or "?" when the node has several.
    """
    if not action_servers:
        return []
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return []
    action = action_servers[0].action_name if len(action_servers) == 1 else "?"
    starters = list(method_bodies.items())
    if constructor_cursor is not None:
        starters.insert(0, ("constructor", constructor_cursor))
    found: list[dict[str, str]] = []
    keys: set[tuple[str, str]] = set()
    for started_in, body in starters:
        try:
            for desc in body.walk_preorder():
                if not _is_thread_start(desc):
                    continue
                targets: list[tuple[str, Any]] = [
                    (m, method_bodies[m]) for m in _extract_all_method_refs(desc)
                    if m in method_bodies]
                # A lambda anywhere in the start expression is the thread body
                # (a constructor call has no callee child to skip).
                for child in desc.get_children():
                    for sub in child.walk_preorder():
                        if sub.kind == CursorKind.LAMBDA_EXPR:
                            targets.append((f"lambda in {started_in}", sub))
                for thread, target in targets:
                    if (thread, started_in) in keys:
                        continue
                    if _sends_on_action_handle(target, method_bodies):
                        keys.add((thread, started_in))
                        found.append(
                            {"thread": thread, "started_in": started_in, "action": action})
        except Exception:
            continue
    return found


# ---------------------------------------------------------------------------
# Publish bounds per callback (W3 slice S3)
# ---------------------------------------------------------------------------

_LOOP_KIND_NAMES = ("FOR_STMT", "WHILE_STMT", "DO_STMT", "CXX_FOR_RANGE_STMT")
_GOTO_KIND_NAMES = ("GOTO_STMT", "INDIRECT_GOTO_STMT")


class _PublishCount:
    """Publish call sites reachable from one function body, per topic.

    ``counts`` holds topics whose sites all run at most once per call,
    ``unbounded`` topics with a site in a loop or a nested lambda (reason per
    topic). ``everything`` is set when no count of the body can be trusted
    (goto, recursion, a publish on a publisher the analyzer cannot name, a
    member helper without a visible body, an escape). ``escape`` is set when
    a call the analyzer does not follow receives ``this``, a publisher or a
    publishing callable: that callee can publish from any callback, so the
    whole class loses its bounds. ``serialized_unnamed`` is set by a
    serialized publish on a publisher the analyzer cannot name.
    """

    def __init__(self) -> None:
        self.counts: dict[str, int] = {}
        self.unbounded: dict[str, str] = {}
        self.serialized: set[str] = set()
        self.everything: str | None = None
        self.escape: str | None = None
        self.serialized_unnamed: str | None = None

    def anything(self) -> bool:
        """True when the body may publish at all."""
        return bool(self.counts or self.unbounded or self.everything or self.escape
                    or self.serialized_unnamed)

    def add(self, topic: str, n: int, looped: str | None) -> None:
        if looped is not None:
            self.unbounded.setdefault(topic, looped)
        else:
            self.counts[topic] = self.counts.get(topic, 0) + n

    def merge(self, other: "_PublishCount", looped: str | None) -> None:
        for topic, n in other.counts.items():
            self.add(topic, n, looped)
        for topic, why in other.unbounded.items():
            self.unbounded.setdefault(topic, why)
        self.serialized |= other.serialized
        if other.everything and not self.everything:
            self.everything = other.everything
        self.escape = self.escape or other.escape
        self.serialized_unnamed = self.serialized_unnamed or other.serialized_unnamed

    def bounds(self, topics: list[str]) -> tuple[dict[str, int | None], dict[str, str]]:
        """``(publish_bounds, publish_unbounded)`` over the counted and the *topics*."""
        bounds: dict[str, int | None] = {}
        why: dict[str, str] = {}
        for topic in sorted(set(self.counts) | set(self.unbounded) | set(topics)):
            if self.everything:
                bounds[topic], why[topic] = None, self.everything
            elif topic in self.unbounded:
                bounds[topic], why[topic] = None, self.unbounded[topic]
            elif topic in self.counts:
                bounds[topic] = self.counts[topic]
            else:
                # Used (publishers_used) but no counted call site: unknown.
                bounds[topic], why[topic] = None, "publish call site not counted"
        return bounds, why


def _publish_call_topic(call: Any, publisher_var_map: dict[str, str]) -> tuple[bool, str | None]:
    """``(is_publish, topic)`` for a CALL_EXPR. Same token rule as publishers_used."""
    from clang.cindex import CursorKind

    if call.spelling not in _PUBLISH_CALL_NAMES:
        return False, None
    callee = next(iter(call.get_children()), None)
    if callee is None or callee.kind != CursorKind.MEMBER_REF_EXPR:
        return False, None
    for tok in _cursor_tokens(callee):
        if tok in publisher_var_map:
            return True, publisher_var_map[tok]
    ref = call.referenced
    parent = getattr(ref, "semantic_parent", None) if ref is not None else None
    if parent is not None and "Publisher" in (parent.spelling or ""):
        return True, None
    return False, None


def _is_serialized_publish(call: Any) -> bool:
    """True when the called ``publish`` overload takes a serialized message."""
    ref = call.referenced
    if ref is None:
        return False
    try:
        arg_types = [a.type.spelling for a in ref.get_arguments()]
    except Exception:
        return False
    return any("SerializedMessage" in t or "serialized_message" in t for t in arg_types)


# Callees in these namespaces never publish on the node's publishers behind
# the analyzer's back (rclcpp's own publish calls are counted as sites).
_TRUSTED_NAMESPACES = frozenset({"rclcpp", "rclcpp_lifecycle", "rcl", "rmw", "rcutils"})
# Calls that only wrap or forward a value: the wrapped value escapes where the
# result goes, not into the wrapper itself.
_PASS_THROUGH_CALLS = frozenset({
    "bind", "bind_front", "bind_back", "ref", "cref", "move", "forward", "mem_fn",
})
# Thread spawns: the callable runs on a new thread, never inside a callback
# dispatch, and the binding keys a publish's origin by the dispatch on the
# publishing thread (corr_runtime.cpp), so it adds nothing to any burst.
_THREAD_SPAWN_CALLS = frozenset({"thread", "jthread", "async"})
# message_filters calls of the admitted synchronizer form (see check_call).
_MESSAGE_FILTERS_TRUSTED_CALLS = frozenset({"subscribe", "registerCallback", "registerDropCallback"})
_PUBLISHER_TYPE_RE = re.compile(r"\brclcpp(?:_lifecycle)?::\w*Publisher(?:Base)?\b")
_RCL_NODE_TYPE_RE = re.compile(r"\brclcpp(?:_lifecycle)?::(?:Node|LifecycleNode)\b")
_SYSTEM_PREFIXES = ("/usr/", "/opt/")


def _outer_namespace(ref: Any) -> str | None:
    """The outermost namespace that encloses *ref*, or None at global scope."""
    from clang.cindex import CursorKind

    ns = None
    parent = ref.semantic_parent
    while parent is not None and parent.kind != CursorKind.TRANSLATION_UNIT:
        if parent.kind == CursorKind.NAMESPACE:
            ns = parent.spelling
        parent = parent.semantic_parent
    return ns


def _in_system_header(cursor: Any) -> bool:
    loc = cursor.location.file
    return loc is None or loc.name.startswith(_SYSTEM_PREFIXES)


def _type_reaches(t: Any, node_re: re.Pattern, depth: int = 0) -> str | None:
    """What of the node a value of type *t* can reach: a publisher, the node, or None.

    Pointers, references, template arguments (through the canonical spelling)
    and the fields of project structs (two levels deep) are seen through.
    """
    from clang.cindex import CursorKind, TypeKind

    canon = t.get_canonical()
    spelling = canon.spelling
    if _PUBLISHER_TYPE_RE.search(spelling):
        return "a publisher"
    if node_re.search(spelling) or _RCL_NODE_TYPE_RE.search(spelling):
        return "the node"
    if depth >= 2:
        return None
    while canon.kind in (TypeKind.POINTER, TypeKind.LVALUEREFERENCE, TypeKind.RVALUEREFERENCE):
        canon = canon.get_pointee().get_canonical()
    decl = canon.get_declaration()
    if decl is None or decl.kind not in (CursorKind.STRUCT_DECL, CursorKind.CLASS_DECL):
        return None
    if _in_system_header(decl):
        return None
    for field_cursor in decl.get_children():
        if field_cursor.kind == CursorKind.FIELD_DECL:
            why = _type_reaches(field_cursor.type, node_re, depth + 1)
            if why:
                return f"{why} (field {decl.spelling}::{field_cursor.spelling})"
    return None


_FUNCTION_KINDS = ("CXX_METHOD", "FUNCTION_DECL", "CONSTRUCTOR", "DESTRUCTOR",
                   "FUNCTION_TEMPLATE", "CONVERSION_FUNCTION")


def _strip_expr(c: Any) -> Any:
    """*c* without the implicit casts and parentheses around it."""
    while c.kind.name in ("UNEXPOSED_EXPR", "PAREN_EXPR"):
        kids = list(c.get_children())
        if len(kids) != 1:
            break
        c = kids[0]
    return c


def _is_plain_assignment(c: Any) -> bool:
    """Whether the BINARY_OPERATOR *c* is ``=`` (not ``==``, ``+`` and so on)."""
    kids = list(c.get_children())
    if len(kids) != 2:
        return False
    end = kids[0].extent.end.offset
    for tok in c.get_tokens():
        if tok.extent.start.offset >= end:
            return tok.spelling == "="
    return False


def _outside_storage(target: Any) -> str | None:
    """Where storing into the expression *target* keeps a value beyond the frame.

    None for a member of the node reached through ``this`` (the node keeps
    it) and for a plain local or by-value parameter (gone with the frame).
    A global, a static, a reference, a pointer target or anything the
    analyzer does not recognise keeps it outside the node, where any code
    can publish through it.
    """
    from clang.cindex import CursorKind, StorageClass, TypeKind

    c = _strip_expr(target)
    if c.kind == CursorKind.MEMBER_REF_EXPR:
        kids = list(c.get_children())
        if not kids:
            return None  # implicit this
        base = _strip_expr(kids[0])
        if base.kind == CursorKind.CXX_THIS_EXPR:
            return None
        if base.type.get_canonical().kind == TypeKind.POINTER:
            return f"the target of {base.spelling or 'a pointer'}"
        return _outside_storage(base)
    ref = c.referenced if c.kind == CursorKind.DECL_REF_EXPR else (
        c if c.kind == CursorKind.VAR_DECL else None)
    if ref is not None:
        if ref.type.get_canonical().kind in (TypeKind.LVALUEREFERENCE, TypeKind.RVALUEREFERENCE):
            return f"the reference {ref.spelling}"
        if ref.kind == CursorKind.PARM_DECL:
            return None
        if ref.kind == CursorKind.VAR_DECL:
            if ref.storage_class == StorageClass.STATIC or (
                    ref.semantic_parent is None
                    or ref.semantic_parent.kind.name not in _FUNCTION_KINDS):
                return f"the global or static {ref.spelling}"
            return None
    return f"the storage at line {c.location.line}"


def _count_publish_sites(
    body: Any,
    class_cursor: Any,
    publisher_var_map: dict[str, str],
    _stack: tuple[str, ...] = (),
) -> _PublishCount:
    """Count the publish call sites of *body* (a method or lambda cursor).

    A site in a loop or in a lambda nested in *body* has no bound. A call to a
    member function of the node class is followed, its sites count once per
    call site of the helper. A call to any other function with a visible body
    outside the system headers is followed too, without the publisher names:
    the node's publishers have other names there, so any publish in it gives
    no bound. Every call outside the class is also safe only when nothing it
    receives reaches the node, since the callee may keep it and publish from
    another callback: ``this``, a publisher, the node through a pointer or a
    struct field, a member function pointer or a lambda that publishes make
    it an escape, and the class loses every bound. rclcpp, rcl, rmw and
    rcutils callees are trusted, and so are the values wrapped in the
    arguments of such a call.
    """
    from clang.cindex import CursorKind

    result = _PublishCount()
    usr = body.get_usr() or f"{body.spelling}@{body.location.line}"
    stack = (*_stack, usr)
    class_name = class_cursor.type.get_canonical().spelling
    node_re = re.compile(rf"(?<![\w:]){re.escape(class_name)}\b")

    def escape_of(arg: Any) -> str | None:
        """What of the node the argument expression *arg* hands to its callee."""
        kind = arg.kind
        if kind == CursorKind.CXX_THIS_EXPR:
            return "this"
        if kind == CursorKind.LAMBDA_EXPR:
            inner = _count_publish_sites(arg, class_cursor, publisher_var_map, stack)
            if inner.anything():
                return f"a lambda that publishes (line {arg.location.line})"
            return None
        if kind in (CursorKind.DECL_REF_EXPR, CursorKind.MEMBER_REF_EXPR):
            ref = arg.referenced
            if (kind == CursorKind.DECL_REF_EXPR and ref is not None
                    and ref.kind == CursorKind.CXX_METHOD and _in_family(ref.semantic_parent, class_cursor)):
                defn = ref.get_definition()
                if ref.get_usr() in stack or defn is None or _count_publish_sites(
                        defn, class_cursor, publisher_var_map, stack).anything():
                    return f"member function {ref.spelling}"
                return None
            why = _type_reaches(arg.type, node_re)
            if why:
                return why
            if ref is not None and ref.kind == CursorKind.VAR_DECL and "lambda at" in ref.type.spelling:
                for d in ref.get_children():
                    if d.kind == CursorKind.LAMBDA_EXPR:
                        return escape_of(d)
            return None
        if kind == CursorKind.CALL_EXPR:
            ref = arg.referenced
            if ref is not None and _outer_namespace(ref) in _TRUSTED_NAMESPACES:
                # An rclcpp handle (a timer holding a bound callback): rclcpp
                # owns the callable, and its calls are trusted.
                return None
            if ref is not None and ref.spelling in _THREAD_SPAWN_CALLS and _outer_namespace(ref) == "std":
                return None  # a thread or a future: its callable runs off any dispatch
            why = _type_reaches(arg.type, node_re)
            if why:
                return why
            if ref is None or not (ref.spelling in _PASS_THROUGH_CALLS
                                   or ref.kind == CursorKind.CONSTRUCTOR):
                # Another call: its own arguments are checked where it is walked.
                return None
        for child in arg.get_children():
            why = escape_of(child)
            if why:
                return why
        return None

    def check_call(c: Any, ref: Any, line: int) -> None:
        name = ref.spelling if ref is not None else (c.spelling or "an unnamed callee")
        if ref is not None and ref.spelling in _PASS_THROUGH_CALLS:
            return
        if (ref is not None and ref.spelling in _MESSAGE_FILTERS_TRUSTED_CALLS
                and _outer_namespace(ref) == "message_filters"):
            # The admitted synchronizer form
            # (message_filters_scan.py) hands the node to Subscriber::subscribe
            # only to create the input subscription, and registerCallback
            # stores a callable the library calls synchronously inside that
            # input's rclcpp callback, which the analysis models as the
            # <cb>_via_<member> subscription callbacks. registerDropCallback
            # stores the ExactTime drop callable, also run inside that input
            # callback. A drop callback that publishes is still counted as a
            # publishing callback of the node, which has no reaction of its
            # own, so its edge stays refused (publisher_callback_kind). Every other form is
            # refused by RCLLF-H003, so neither call can publish from
            # anywhere else.
            return
        if (ref is not None and ref.spelling in _THREAD_SPAWN_CALLS
                and _outer_namespace(ref) == "std"):
            return
        if _is_thread_container_insert(c):
            return  # a thread built in place: its callable runs off any dispatch
        if _PUBLISHER_TYPE_RE.search(c.type.get_canonical().spelling):
            return  # a publisher copy, alias or move: tracked where it is passed on
        try:
            args = list(c.get_arguments()) if ref is not None else list(c.get_children())
        except Exception:
            args = list(c.get_children())
        if name == "operator=" and args and any(
                d.kind == CursorKind.MEMBER_REF_EXPR for d in args[0].walk_preorder()) and next(
                d for d in args[0].walk_preorder()
                if d.kind == CursorKind.MEMBER_REF_EXPR).spelling == "deadline_callback":
            return  # a deadline handler: folded into its subscription callback
        for arg in args:
            why = escape_of(arg)
            if why:
                reason = f"call to {name} at line {line} receives {why}"
                result.escape = result.escape or reason
                result.everything = result.everything or reason
                return

    def check_store(target: Any, value: Any, line: int) -> None:
        """An assignment or a static initializer: the stored value may escape."""
        why = escape_of(value)
        where = why and _outside_storage(target)
        if where:
            reason = f"assignment at line {line} stores {why} in {where}"
            result.escape = result.escape or reason
            result.everything = result.everything or reason

    def walk(c: Any, looped: str | None, trusted_arg: bool) -> None:
        kind = c.kind.name
        line = c.location.line
        if c.kind == CursorKind.BINARY_OPERATOR and _is_plain_assignment(c):
            lhs, rhs = c.get_children()
            check_store(lhs, rhs, line)
        elif (c.kind == CursorKind.VAR_DECL and c.storage_class.name == "STATIC"):
            init = [d for d in c.get_children() if d.kind.is_expression()]
            if init:
                check_store(c, init[-1], line)
        elif (c.kind == CursorKind.CALL_EXPR and c.spelling == "operator="
              and c.referenced is not None):
            args = list(c.get_arguments())
            if len(args) == 2:
                check_store(args[0], args[1], line)
        child_trusted = trusted_arg and kind in ("UNEXPOSED_EXPR", "PAREN_EXPR")
        if kind in _LOOP_KIND_NAMES:
            looped = looped or f"loop at line {line}"
        elif kind in _GOTO_KIND_NAMES and not result.everything:
            result.everything = f"goto at line {line}"
        elif c.kind == CursorKind.LAMBDA_EXPR:
            looped = looped or f"lambda at line {line}"
        elif c.kind == CursorKind.CALL_EXPR:
            is_publish, topic = _publish_call_topic(c, publisher_var_map)
            if is_publish:
                if topic is None:
                    result.everything = result.everything or (
                        f"publish on a publisher the analyzer cannot name at line {line}")
                    if _is_serialized_publish(c):
                        result.serialized_unnamed = result.serialized_unnamed or (
                            f"serialized publish on a publisher the analyzer cannot name "
                            f"at line {line}")
                else:
                    result.add(topic, 1, looped)
                    if _is_serialized_publish(c):
                        result.serialized.add(topic)
            else:
                ref = c.referenced
                callable_ref = ref is not None and ref.kind in (
                    CursorKind.CXX_METHOD, CursorKind.FUNCTION_DECL, CursorKind.CONSTRUCTOR,
                    CursorKind.FUNCTION_TEMPLATE, CursorKind.CONVERSION_FUNCTION)
                defn = ref.get_definition() if callable_ref else None
                if callable_ref and ref.kind == CursorKind.CXX_METHOD and _in_family(ref.semantic_parent, class_cursor):
                    if ref.get_usr() in stack:
                        result.everything = result.everything or (
                            f"recursive call of {ref.spelling} at line {line}")
                    elif defn is None:
                        result.everything = result.everything or (
                            f"member {ref.spelling} called at line {line} has no visible body")
                    else:
                        result.merge(
                            _count_publish_sites(defn, class_cursor, publisher_var_map, stack),
                            looped and f"{looped} (through {ref.spelling})",
                        )
                elif callable_ref and _outer_namespace(ref) in _TRUSTED_NAMESPACES:
                    child_trusted = True
                elif (callable_ref and ref.kind == CursorKind.CXX_METHOD
                      and "lambda at" in ref.semantic_parent.type.spelling):
                    pass  # calls a lambda: its body is walked where it is written
                elif trusted_arg and (ref is None or ref.kind == CursorKind.CONSTRUCTOR
                                      or ref.spelling in _PASS_THROUGH_CALLS):
                    child_trusted = True  # wraps a value handed to a trusted callee
                else:
                    if defn is not None and not _in_system_header(defn):
                        if defn.get_usr() in stack:
                            result.everything = result.everything or (
                                f"recursive call of {ref.spelling} at line {line}")
                        else:
                            result.merge(
                                _count_publish_sites(defn, class_cursor, {}, stack),
                                looped and f"{looped} (through {ref.spelling})",
                            )
                    # Followed or not, the callee may keep what it receives
                    # and publish through it from another callback.
                    check_call(c, ref, line)
        for child in c.get_children():
            walk(child, looped, child_trusted)

    for child in body.get_children():
        walk(child, None, False)
    return result


def _deadline_handler_bodies(
    sub_call: Any,
    owner: Any,
    class_cursor: Any,
    members: list[Any],
    definition_of: Callable[[Any], Any | None] | None = None,
) -> tuple[list[Any], str | None]:
    """Bodies of the requested-deadline-missed handler of one create_subscription.

    Returns ``(bodies, unresolved)``. The options argument is followed when it
    names a local variable of *owner* (the creating method), a field of the
    node class (searched in every member, *members*) or calls a function
    whose definition is visible. *definition_of* resolves a callee to its
    definition; the default is the cursor's own ``get_definition()``, and
    ``parse_node`` passes a resolver that also finds the bodies the
    preamble-skipping parse dropped (a helper defined in a header).
    There, an assignment to
    ``event_callbacks.deadline_callback`` of a lambda or of
    ``std::bind(&Class::method, ...)`` gives the handler. Options with no such
    assignment give no handler. Anything the analyzer cannot follow (a
    parameter, a global, an opaque initializer, options handed to another
    call, a handler that is neither form) sets *unresolved*: the handler may
    publish, so the callback's bounds become unknown.
    """
    from clang.cindex import CursorKind

    if definition_of is None:
        def definition_of(ref: Any) -> Any | None:
            return ref.get_definition()

    children = list(sub_call.get_children())
    if len(children) < 5:
        return [], None
    opts = children[4]
    line = sub_call.location.line

    def strip(c: Any) -> Any:
        while c.kind in (CursorKind.UNEXPOSED_EXPR, CursorKind.PAREN_EXPR,
                         CursorKind.UNARY_OPERATOR):
            kids = list(c.get_children())
            if len(kids) != 1:
                break
            c = kids[0]
        return c

    def init_scope(expr: Any, what: str) -> tuple[Any | None, str | None]:
        """``(scope, unresolved)`` for the initializer *expr* of the options *what*."""
        expr = strip(expr)
        ref = expr.referenced if expr.kind == CursorKind.CALL_EXPR else None
        if ref is not None and ref.kind == CursorKind.CONSTRUCTOR:
            args = list(expr.get_arguments())
            if not args:
                return None, None  # default constructed: no handler yet
            if len(args) == 1:
                return init_scope(args[0], what)  # a copy or a move
        if ref is not None and ref.kind in (CursorKind.FUNCTION_DECL, CursorKind.CXX_METHOD):
            defn = definition_of(ref)
            if defn is not None:
                return defn, None
        return None, (f"{what} is initialized at line {expr.location.line} from an expression "
                      "the analyzer cannot follow")

    def passed_on(scope: Any, var: Any) -> str | None:
        """A call in *scope* other than create_subscription that receives *var* itself."""
        for c in scope.walk_preorder():
            if c.kind != CursorKind.CALL_EXPR or c.spelling == "create_subscription":
                continue
            try:
                args = list(c.get_arguments())
            except Exception:
                continue
            for arg in args:
                a = strip(arg)
                if a.kind == CursorKind.CALL_EXPR and a.referenced is not None and (
                        a.referenced.kind == CursorKind.CONSTRUCTOR):
                    kids = list(a.get_arguments())
                    a = strip(kids[0]) if len(kids) == 1 else a
                if a.kind in (CursorKind.DECL_REF_EXPR, CursorKind.MEMBER_REF_EXPR) and a.referenced == var:
                    return (f"subscription options {var.spelling} are passed to {c.spelling} "
                            f"at line {c.location.line}")
        return None

    scopes: list[tuple[Any, Any]] = []  # (function body to search, variable or field, or None)
    for desc in opts.walk_preorder():
        kind = desc.kind
        if kind not in (CursorKind.DECL_REF_EXPR, CursorKind.MEMBER_REF_EXPR, CursorKind.CALL_EXPR):
            continue
        ref = desc.referenced
        if ref is None:
            return [], f"options of create_subscription at line {line} cannot be resolved"
        if kind == CursorKind.CALL_EXPR and ref.kind == CursorKind.CONSTRUCTOR:
            if not list(desc.get_arguments()):
                return [], None  # rclcpp::SubscriptionOptions(): no handler
            continue  # a copy or conversion: the copied options are further down
        if kind == CursorKind.DECL_REF_EXPR and ref.kind == CursorKind.VAR_DECL and ref.semantic_parent == owner:
            scopes.append((owner, ref))
            for init in ref.get_children():
                if init.kind.is_expression():
                    scope, why = init_scope(init, f"options variable {ref.spelling}")
                    if why:
                        return [], why
                    if scope is not None:
                        scopes.append((scope, None))
            why = passed_on(owner, ref)
            if why:
                return [], why
            break
        if (kind == CursorKind.MEMBER_REF_EXPR and ref.kind == CursorKind.FIELD_DECL
                and _in_family(ref.semantic_parent, class_cursor)):
            for init in ref.get_children():
                if init.kind.is_expression():
                    scope, why = init_scope(init, f"options field {ref.spelling}")
                    if why:
                        return [], why
                    if scope is not None:
                        scopes.append((scope, None))
            for method in members:
                scopes.append((method, ref))
                why = passed_on(method, ref)
                if why:
                    return [], why
                if method.kind != CursorKind.CONSTRUCTOR:
                    continue
                kids = list(method.get_children())
                for i, kid in enumerate(kids[:-1]):  # member initializer list
                    if kid.kind == CursorKind.MEMBER_REF and kid.referenced == ref:
                        scope, why = init_scope(kids[i + 1], f"options field {ref.spelling}")
                        if why:
                            return [], why
                        if scope is not None:
                            scopes.append((scope, None))
            break
        if kind == CursorKind.CALL_EXPR and ref.kind in (CursorKind.FUNCTION_DECL, CursorKind.CXX_METHOD):
            defn = definition_of(ref)
            if defn is None:
                return [], (f"options of create_subscription at line {line} come from "
                            f"{ref.spelling}, which has no visible body")
            scopes.append((defn, None))
            break
        return [], (f"options of create_subscription at line {line} name {ref.spelling}, "
                    "which is not a local variable or a field of the node class")

    bodies: list[Any] = []
    for scope, var in scopes:
        for c in scope.walk_preorder():
            if c.kind not in (CursorKind.CALL_EXPR, CursorKind.BINARY_OPERATOR):
                continue
            kids = list(c.get_children())
            if len(kids) < 2:
                continue
            lhs = kids[0]
            lhs_refs = [d for d in lhs.walk_preorder() if d.kind == CursorKind.MEMBER_REF_EXPR]
            if not lhs_refs or lhs_refs[0].spelling != "deadline_callback":
                continue
            if var is not None and not any(
                    d.kind in (CursorKind.DECL_REF_EXPR, CursorKind.MEMBER_REF_EXPR)
                    and d.referenced == var
                    for d in lhs.walk_preorder()):
                continue
            found = False
            for rhs in kids[1:]:
                for d in rhs.walk_preorder():
                    if d.kind == CursorKind.LAMBDA_EXPR:
                        bodies.append(d)
                        found = True
                        break
                    if (d.kind in (CursorKind.DECL_REF_EXPR, CursorKind.MEMBER_REF_EXPR)
                            and d.referenced is not None
                            and d.referenced.kind == CursorKind.CXX_METHOD
                            and _in_family(d.referenced.semantic_parent, class_cursor)
                            and d.referenced.get_definition() is not None):
                        bodies.append(d.referenced.get_definition())
                        found = True
                        break
                if found:
                    break
            if not found:
                return [], (f"deadline handler assigned at line {c.location.line} is not a "
                            "lambda or a member function of the node class")
    return bodies, None


def _analyze_callers(
    class_cursor: Any,
    tu: Any,
    callbacks: dict[str, CallbackInfo],
    lambda_bodies: dict[str, Any] | None = None,
) -> None:
    """Fill ``callers`` and fold a member helper's publishes into its callers.

    A callback that publishes through a member helper publishes
    from its own invocation, so the helper's topics join the callback's
    ``publishers_used`` (transitively, as a lambda callback already folds the
    member functions it calls). ``callers`` lets the generator attribute a
    helper's publishes to the reactions that call it. It is set only where
    the caller set is closed: a private, non-virtual method of a class with
    no friend and a visible definition of every method it declares. A use of
    the method that is not a direct call (``&Node::helper`` bound or passed
    on) is recorded as a ``<...>`` caller, so the helper is never taken to
    run only inside its callers.
    """
    try:
        from clang.cindex import AccessSpecifier, CursorKind
    except ImportError:
        return
    lambda_bodies = lambda_bodies or {}

    def extent(c: Any) -> tuple:
        return (c.extent.start.offset, c.extent.end.offset)

    lambda_names = {extent(c): name for name, c in lambda_bodies.items()}
    own_kinds = (CursorKind.CXX_METHOD, CursorKind.CONSTRUCTOR, CursorKind.DESTRUCTOR)

    def own_method(ref: Any) -> bool:
        return (ref is not None and ref.kind == CursorKind.CXX_METHOD
                and _in_family(ref.semantic_parent, class_cursor))

    calls: dict[str, set[str]] = {}  # caller -> member functions it calls
    callers: dict[str, set[str]] = {}  # member function -> its callers
    called: Counter = Counter()  # (caller, callee) -> direct calls
    used: Counter = Counter()  # (caller, callee) -> references

    def visit(cursor: Any, owner: str) -> None:
        for child in cursor.get_children():
            inner = owner
            if child.kind == CursorKind.LAMBDA_EXPR:
                inner = lambda_names.get(extent(child)) or f"<lambda in {owner}>"
            elif child.kind == CursorKind.CALL_EXPR and own_method(child.referenced):
                name = child.referenced.spelling
                calls.setdefault(owner, set()).add(name)
                callers.setdefault(name, set()).add(owner)
                called[(owner, name)] += 1
            elif child.kind in (CursorKind.MEMBER_REF_EXPR, CursorKind.DECL_REF_EXPR) and own_method(
                    child.referenced):
                used[(owner, child.referenced.spelling)] += 1
            visit(child, inner)

    complete = True
    for decl in class_cursor.get_children():
        if decl.kind == CursorKind.FRIEND_DECL:
            complete = False
        elif decl.kind in (*own_kinds, CursorKind.FUNCTION_TEMPLATE):
            if decl.get_definition() is None and not (
                decl.is_pure_virtual_method() or decl.is_default_method()
                or getattr(decl, "is_deleted_method", lambda: False)()
            ):
                complete = False
    for method in tu.cursor.walk_preorder():
        if (method.kind not in own_kinds or not _in_family(method.semantic_parent, class_cursor)
                or not method.is_definition()):
            continue
        owner = method.spelling if method.kind == CursorKind.CXX_METHOD else f"<{method.kind.name.lower()}>"
        visit(method, owner)
    for (owner, name), n in used.items():
        if n > called[(owner, name)]:
            callers.setdefault(name, set()).add(f"<used, not called, in {owner}>")

    for name, cb_info in callbacks.items():
        ref = next((m for m in class_cursor.get_children()
                    if m.kind == CursorKind.CXX_METHOD and m.spelling == name), None)
        closed = (complete and ref is not None and ref.access_specifier == AccessSpecifier.PRIVATE
                  and not ref.is_virtual_method())
        cb_info.callers = sorted(callers.get(name, set())) if closed else None

    changed = True
    while changed:
        changed = False
        for owner, callees in calls.items():
            caller = callbacks.get(owner)
            if caller is None:
                continue
            topics = set(caller.publishers_used)
            for callee in callees:
                if callee in callbacks:
                    topics |= set(callbacks[callee].publishers_used)
            if topics != set(caller.publishers_used):
                caller.publishers_used = sorted(topics)
                changed = True


def _analyze_publish_bounds(
    class_cursor: Any,
    tu: Any,
    publisher_var_map: dict[str, str],
    publishers: list[PublisherInfo],
    callbacks: dict[str, CallbackInfo],
    lambda_bodies: dict[str, Any] | None = None,
    definition_of: Callable[[Any], Any | None] | None = None,
) -> None:
    """Fill ``publish_bounds``, ``missing_handler_publishers`` and ``serialized_publish``.

    W3 slice S3: the burst bound k of a correspondence edge is
    proven here when every publish call site of the edge's publisher in the
    callback runs at most once per invocation. A deadline handler's
    publishes are folded into its subscription callback: the binding runs the
    handler instead of the callback in the drain reaction, so the bound is the
    larger of the two counts, never their sum. *lambda_bodies* maps the
    synthesized name of a lambda callback to its lambda cursor.
    *definition_of* is handed to :func:`_deadline_handler_bodies`.
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return
    lambda_bodies = lambda_bodies or {}

    def extent(c: Any) -> tuple:
        return (c.extent.start.offset, c.extent.end.offset)

    lambda_names = {extent(c): name for name, c in lambda_bodies.items()}

    members = [
        c for c in tu.cursor.walk_preorder()
        if c.kind in (CursorKind.CXX_METHOD, CursorKind.CONSTRUCTOR)
        and _in_family(c.semantic_parent, class_cursor) and c.is_definition()
    ]
    serialized: set[str] = set()
    escape: str | None = None
    serialized_unnamed: str | None = None
    for method in members:
        count = _count_publish_sites(method, class_cursor, publisher_var_map)
        serialized |= count.serialized
        escape = escape or count.escape
        serialized_unnamed = serialized_unnamed or count.serialized_unnamed
        cb_info = callbacks.get(method.spelling) if method.kind == CursorKind.CXX_METHOD else None
        if cb_info is not None:
            cb_info.publish_bounds, cb_info.publish_unbounded = count.bounds(cb_info.publishers_used)
    for name, body in lambda_bodies.items():
        count = _count_publish_sites(body, class_cursor, publisher_var_map)
        serialized |= count.serialized
        escape = escape or count.escape
        serialized_unnamed = serialized_unnamed or count.serialized_unnamed
        cb_info = callbacks[name]
        cb_info.publish_bounds, cb_info.publish_unbounded = count.bounds(cb_info.publishers_used)

    def unknown(cb_info: CallbackInfo, why: str) -> None:
        topics = set(cb_info.publishers_used) | set(cb_info.publish_bounds)
        cb_info.publish_bounds = {t: None for t in sorted(topics)}
        cb_info.publish_unbounded = {t: why for t in sorted(topics)}

    for method in members:
        for call in method.walk_preorder():
            if call.kind != CursorKind.CALL_EXPR or call.spelling != "create_subscription":
                continue
            children = list(call.get_children())
            form, found = _callback_form(children[3] if len(children) >= 4 else None)
            cb_name = found if form == "method" else (
                lambda_names.get(extent(found)) if form == "lambda" else None)
            cb_info = callbacks.get(cb_name) if cb_name else None
            if cb_info is None:
                continue
            handlers, unresolved = _deadline_handler_bodies(
                call, method, class_cursor, members, definition_of)
            if unresolved:
                unknown(cb_info, f"deadline handler: {unresolved}")
                continue
            for handler in handlers:
                count = _count_publish_sites(handler, class_cursor, publisher_var_map)
                serialized |= count.serialized
                if count.everything:
                    unknown(cb_info, f"deadline handler: {count.everything}")
                h_bounds, h_why = count.bounds([])
                for topic, n in h_bounds.items():
                    if topic not in cb_info.missing_handler_publishers:
                        cb_info.missing_handler_publishers.append(topic)
                    if topic not in cb_info.publishers_used:
                        cb_info.publishers_used = sorted({*cb_info.publishers_used, topic})
                    have = cb_info.publish_bounds.get(topic, 0)
                    if topic in cb_info.publish_bounds and have is None:
                        continue
                    if n is None:
                        cb_info.publish_bounds[topic] = None
                        cb_info.publish_unbounded[topic] = f"deadline handler: {h_why[topic]}"
                    else:
                        cb_info.publish_bounds[topic] = max(have, n)
                cb_info.missing_handler_publishers.sort()

    if escape:
        # The callee that received the node can publish from any callback.
        for cb_info in callbacks.values():
            unknown(cb_info, f"the node escapes: {escape}")
    for pub in publishers:
        # A serialized publish on a publisher the analyzer cannot name may be
        # on any of the node's publishers.
        if pub.topic in serialized or serialized_unnamed:
            pub.serialized_publish = True


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

# libclang TranslationUnit flags not exposed as named constants by the Python
# bindings (clang-c/Index.h, CXTranslationUnit_Flags).
_CXTU_CREATE_PREAMBLE_ON_FIRST_PARSE = 0x100
_CXTU_LIMIT_SKIP_FUNCTION_BODIES_TO_PREAMBLE = 0x800


_UNRESOLVED_INCLUDE_RE = re.compile(r"^'(?P<header>[^']+)' file not found$")


class UnresolvedIncludeError(Exception):
    """The node class is missing from the parse because of unresolved includes.

    Deliberately not a :class:`RuntimeError`. The graph builder skips a source
    that raises RuntimeError with a log warning, and a launched node hidden by
    a missing header must fail the analysis, not vanish from it.
    """

    def __init__(
        self,
        source_file: str | Path,
        unresolved_includes: list[dict[str, Any]],
        node: str | None = None,
    ) -> None:
        self.source_file = Path(source_file)
        self.unresolved_includes = unresolved_includes
        self.node = node
        who = f"node {node!r}" if node else "node"
        headers = ", ".join(
            f"'{inc['header']}' not found, included from "
            f"{inc['included_from']}:{inc['line']}"
            for inc in unresolved_includes
        )
        super().__init__(
            f"{who} (source {self.source_file}): no ROS2 node class found, "
            f"because of unresolved include(s): {headers}. Make the headers "
            f"reachable (build the package with "
            f"-DCMAKE_EXPORT_COMPILE_COMMANDS=ON, or install them under "
            f"install/<pkg>/include) and re-run colcon lf-analyze."
        )

    def for_node(self, node: str) -> UnresolvedIncludeError:
        """Return the same error naming the launched *node*."""
        return UnresolvedIncludeError(
            self.source_file, self.unresolved_includes, node=node
        )


def _unresolved_includes(tu: Any, cpp_path: Path) -> list[dict[str, Any]]:
    """Return the ``#include`` directives libclang could not resolve.

    Each entry names the header, the file holding the directive (the node's
    own header when that is where the include sits) and its line.
    """
    result: list[dict[str, Any]] = []
    for diag in tu.diagnostics:
        if diag.severity < 3:
            continue
        m = _UNRESOLVED_INCLUDE_RE.match(diag.spelling.strip())
        if m is None:
            continue
        loc = diag.location
        included_from = Path(loc.file.name).resolve() if loc.file else cpp_path
        entry = {
            "header": m.group("header"),
            "included_from": str(included_from),
            "line": loc.line,
        }
        if entry not in result:
            result.append(entry)
    return result


def _parse_options(cindex) -> int:
    """TranslationUnit options for ``parse_node``.

    Function bodies are skipped only inside the preamble (the included
    headers, above all rclcpp); the node's own file is parsed in full, and
    extraction only walks that file. Measured over the 56 example sources: the
    same 27 node results and the same 29 non-node errors as a full parse, in
    about 0.56x the time. ``RCLLF_CLANG_FULL_PARSE=1`` restores the full parse.
    """
    opts = cindex.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD
    if os.environ.get("RCLLF_CLANG_FULL_PARSE") != "1":
        opts |= (
            cindex.TranslationUnit.PARSE_PRECOMPILED_PREAMBLE
            | _CXTU_CREATE_PREAMBLE_ON_FIRST_PARSE
            | cindex.TranslationUnit.PARSE_SKIP_FUNCTION_BODIES
            | _CXTU_LIMIT_SKIP_FUNCTION_BODIES_TO_PREAMBLE
        )
    return opts


def _preamble_definition_resolver(
    index: Any, cindex: Any, cpp_path: Path, include_flags: list[str],
) -> Callable[[Any], Any | None]:
    """A resolver for the function bodies the preamble-skipping parse dropped.

    :func:`_parse_options` skips every function body in the
    included headers, so a helper defined ``inline`` in a project header (one
    that builds a subscription's options, say) has no visible definition in
    the node's translation unit and ``get_definition()`` returns ``None``.
    The returned callable answers ``get_definition()`` where it is not
    ``None``; otherwise, for a function or method declared in a file, it
    parses the same translation unit once more with every body kept (once
    per ``parse_node`` call, on first use only) and returns the definition
    with the same USR, looked for in the file that declares the function
    first and then in every other non-system file of the unit (a prototype
    in one project header, the ``inline`` body in another), or ``None``.
    System headers are never searched. Cursors
    it returns live in the second translation unit: they can be walked and
    counted, and comparisons with cursors of the first one are false, which
    keeps every use of them conservative (a member of the node class met
    there counts as a foreign function).
    """
    state: dict[str, Any] = {}

    def resolve(ref: Any) -> Any | None:
        from clang.cindex import CursorKind

        defn = ref.get_definition()
        if defn is not None:
            return defn
        if ref.kind not in (CursorKind.FUNCTION_DECL, CursorKind.CXX_METHOD):
            return None
        where = ref.location.file
        usr = ref.get_usr()
        if where is None or not usr:
            return None
        if "tu" not in state:
            try:
                state["tu"] = index.parse(str(cpp_path), args=include_flags, options=0)
            except Exception:
                state["tu"] = None
        tu = state["tu"]
        if tu is None:
            return None
        wanted = where.name
        scopes = (
            CursorKind.NAMESPACE, CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL,
            CursorKind.CLASS_TEMPLATE, CursorKind.LINKAGE_SPEC, CursorKind.UNEXPOSED_DECL,
        )

        def find(cursor: Any, in_file: Callable[[str], bool]) -> Any | None:
            for child in cursor.get_children():
                f = child.location.file
                if f is None or not in_file(f.name):
                    continue
                if (child.kind in (CursorKind.FUNCTION_DECL, CursorKind.CXX_METHOD)
                        and child.is_definition() and child.get_usr() == usr):
                    return child
                if child.kind in scopes:
                    found = find(child, in_file)
                    if found is not None:
                        return found
            return None

        found = find(tu.cursor, lambda name: name == wanted)
        if found is None:
            found = find(tu.cursor, lambda name: not name.startswith(_SYSTEM_PREFIXES))
        return found

    return resolve


def _constructor_definition(class_cursor: Any, tu: Any) -> Any | None:
    """The first constructor of *class_cursor* that has a body, or None."""
    try:
        from clang.cindex import CursorKind
        for cursor in class_cursor.get_children():
            if cursor.kind == CursorKind.CONSTRUCTOR and cursor.is_definition():
                return cursor
        # Not a direct child (defined out of line): walk deeper
        for cursor in tu.cursor.walk_preorder():
            if cursor.kind == CursorKind.CONSTRUCTOR:
                if cursor.semantic_parent == class_cursor and cursor.is_definition():
                    return cursor
    except Exception:
        pass
    return None


def _merge_constructor_walks(walks: list[tuple]) -> tuple:
    """Merge the ``_walk_constructor`` results of a base chain and the node itself, in order."""
    if not walks:
        return ([], [], [], [], [], [], [], [], [], False, None, {}, {}, {})
    lists = [[] for _ in range(9)]
    has_cb = False
    param_cb = None
    maps: list[dict] = [{}, {}, {}]
    for w in walks:
        for i in range(9):
            lists[i].extend(w[i])
        has_cb = has_cb or w[9]
        if w[10] and not param_cb:
            param_cb = w[10]
        for i in range(3):
            maps[i].update(w[11 + i])
    return (*lists, has_cb, param_cb, *maps)


def _compile_variant_issues(
    cpp_path: Path,
    workspace_root: Path,
    target: str | None,
    extra_defines: list[str] | None,
) -> list[dict[str, str]]:
    """RCLLF-H016 facts: executables of one file whose definitions differ and cannot be told apart.

    Empty when the file is compiled one way, or when the executable's own
    compile flags are known (its compile_commands.json entry or the
    ``target_compile_definitions`` of its CMakeLists). Otherwise the file is
    parsed once per distinct set of definitions: ``fatal`` is "1" when those
    parses name different node classes, so a wrong pick would run the wrong
    node.
    """
    variants = _compile_variants(cpp_path, workspace_root)
    if len(set(variants.values())) <= 1:
        return []
    if target is not None and target in variants:
        return []
    if extra_defines is not None:
        return []
    try:
        import clang.cindex as cindex
    except ImportError:
        return []
    classes: set[str | None] = set()
    for tgt in variants:
        flags = _load_include_flags(cpp_path, workspace_root, target=tgt)
        tu = cindex.Index.create().parse(
            str(cpp_path), args=flags, options=_parse_options(cindex))
        found = _find_node_class(tu)
        classes.add(_qualified_class_name(found) if found is not None else None)
    reason = (
        f"{cpp_path.name} is compiled by {len(variants)} executables with different "
        f"definitions ({', '.join(sorted(str(t) for t in variants))}) and the executable "
        "could not be matched to one of them"
    )
    return [{
        "source": cpp_path.name,
        "reason": reason,
        "fatal": "1" if len(classes) > 1 else "0",
    }]


def parse_node(
    cpp_path: str | Path,
    workspace_root: str | Path | None = None,
    target: str | None = None,
    extra_defines: list[str] | None = None,
) -> NodeInfo:
    """Parse a ROS2 C++ node source file and return a :class:`NodeInfo`.

    Results are cached by resolved *cpp_path* — repeated calls for the same
    file return the cached result without re-parsing.

    Parameters
    ----------
    cpp_path:
        Absolute path to the ``.cpp`` source file.
    workspace_root:
        Colcon workspace root (contains ``src/`` and ``build/``).
        Inferred automatically if *None*.
    target:
        The CMake target (executable) the node is analysed for. A source file
        several targets compile is read with that target's compile flags.
    extra_defines:
        ``-DNAME`` words from the target's CMakeLists, used when no
        compile_commands.json entry exists for the target.
    """
    cpp_path = Path(cpp_path).resolve()
    _family_cache.clear()

    try:
        import clang.cindex as cindex
    except ImportError as exc:
        raise RuntimeError(
            "libclang Python bindings not available — "
            "install `libclang` package and ensure libclang.so is on LD_LIBRARY_PATH"
        ) from exc

    if workspace_root is None:
        workspace_root = _infer_workspace_root(cpp_path)
    workspace_root = Path(workspace_root)

    include_flags = _load_include_flags(
        cpp_path, workspace_root, target=target, extra_defines=extra_defines)
    cache_key = (cpp_path, tuple(f for f in include_flags if f.startswith("-D")))
    if cache_key in _parse_node_cache:
        return _parse_node_cache[cache_key]
    compile_variant_issues = _compile_variant_issues(
        cpp_path, workspace_root, target, extra_defines)

    index = cindex.Index.create()
    tu_full = index.parse(
        str(cpp_path),
        args=include_flags,
        options=_parse_options(cindex),
    )

    # Collect fatal/error diagnostics — missing headers cause incomplete AST
    # and lead to empty publisher/subscriber extraction.  We surface these as
    # actionable warnings so the user knows what system package to install.
    parse_warnings: list[str] = []
    for diag in tu_full.diagnostics:
        # Severity: 0=ignored,1=note,2=warning,3=error,4=fatal
        if diag.severity >= 3:
            msg = diag.spelling.strip()
            if msg and msg not in parse_warnings:
                parse_warnings.append(msg)
    unresolved_includes = _unresolved_includes(tu_full, cpp_path)

    class_cursor = _find_node_class(tu_full)
    if class_cursor is None and unresolved_includes:
        raise UnresolvedIncludeError(cpp_path, unresolved_includes)
    if class_cursor is None:
        raise RuntimeError(
            f"No ROS2 node class found in {cpp_path}. "
            "Make sure the file includes rclcpp/rclcpp.hpp and the class "
            "inherits from rclcpp::Node or rclcpp_lifecycle::LifecycleNode."
        )

    # A node whose source only holds main() defines its class in a
    # header, whose function bodies the preamble-skipping parse above dropped.
    # Re-parse in full so the constructor and callbacks can be read.
    if _class_body_lives_elsewhere(class_cursor, tu_full, cpp_path):
        # Members may be defined out of line in another translation unit of
        # the same CMake target (or of a library it links). Resolve them by
        # the class USR; a single defining TU is analysed in place of this one.
        tiers = _target_translation_units(cpp_path, workspace_root)
        defining: list[Path] = []
        checked: list[Path] = []
        for tier in tiers or []:
            fresh = [t for t in tier if t not in checked]
            defining = _tus_defining_members(
                index, cindex, class_cursor, fresh, workspace_root, cpp_path,
            )
            checked += fresh
            if defining:
                break
        other_tus = checked if tiers is not None else None
        if len(defining) == 1:
            result = parse_node(
                defining[0], workspace_root, target=target, extra_defines=extra_defines)
            _parse_node_cache[cache_key] = result
            return result
        if len(defining) > 1:
            raise NodeSourceError(
                f"{_qualified_class_name(class_cursor)} (declared via "
                f"{cpp_path}) has member definitions in {len(defining)} "
                f"translation units ({', '.join(str(d) for d in defining)}). "
                "Staging one node from several source files is not supported; "
                "define the constructor and callbacks in one .cpp file."
            )
        tu_full = index.parse(
            str(cpp_path),
            args=include_flags,
            options=cindex.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD,
        )
        reparsed = _find_node_class(tu_full)
        if reparsed is not None:
            class_cursor = reparsed
        if not _has_constructor_definition(class_cursor, tu_full):
            raise NodeSourceError(
                f"{_qualified_class_name(class_cursor)} is only declared via "
                f"{cpp_path} and no constructor definition was found in the "
                "class header or in any translation unit of its CMake target "
                f"({_describe_tus(other_tus)}). Generating it would bind no "
                "timers, subscriptions or publishers. Add the .cpp that "
                "defines the class to the executable (or link the library "
                "that holds it), and make sure the package is built with "
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON."
            )

    base_class = _base_class_spelling(class_cursor)
    class_name = class_cursor.spelling

    # ---- Component detection (macro pass — needs preprocessing record) --
    component_class = _extract_component_class(tu_full)
    is_component = component_class is not None

    # ---- Include-directive scan (Phase 5: RCLLF-H003) -------------------
    detected_includes = _scan_detected_includes(tu_full)

    # ---- Explicit take() call scan (T5: RCLLF-R002) ----------
    detected_take_calls = _scan_take_calls(tu_full)

    # ---- ROS-time sleep scan (RCLLF-H008) --------------------
    detected_ros_sleep_calls = _scan_ros_sleep_calls(tu_full)

    # ---- Lifecycle callback detection ----------------------------------
    lifecycle_callbacks: list[str] = []
    try:
        from clang.cindex import CursorKind
        for cursor in tu_full.cursor.walk_preorder():
            if cursor.kind != CursorKind.CXX_METHOD:
                continue
            if cursor.semantic_parent != class_cursor:
                continue
            if cursor.spelling in _LIFECYCLE_METHODS and cursor.is_definition():
                lifecycle_callbacks.append(cursor.spelling)
    except Exception:
        pass

    # ---- Method signatures ---------------------------------------------
    callbacks = _extract_method_signatures(class_cursor, tu_full, cpp_path)

    # ---- Walk constructor body -----------------------------------------
    # A node whose endpoints live in a base class: the workspace bases'
    # constructors run first (most-base first), then the node's own.
    family, template_bases = _class_family(class_cursor)
    constructor_cursor = _constructor_definition(class_cursor, tu_full)
    base_ctors = [
        (b, _constructor_definition(b, tu_full)) for b in family
    ]
    base_ctors = [(b, c) for b, c in base_ctors if c is not None]

    # Gap 1: Collect all class method bodies for helper traversal
    class_method_bodies = _collect_class_method_bodies(class_cursor, tu_full, cpp_path)

    walks = []
    for _base, base_ctor in base_ctors:
        walks.append(_walk_constructor(
            base_ctor, callbacks, class_method_bodies=class_method_bodies))
    if constructor_cursor is not None:
        walks.append(_walk_constructor(
            constructor_cursor, callbacks, class_method_bodies=class_method_bodies))
    (
        publishers, subscribers, timers,
        service_servers, service_clients,
        action_servers, action_clients,
        callback_groups, parameters, has_param_callback, param_callback,
        publisher_var_map, svc_client_var_map, act_client_var_map,
    ) = _merge_constructor_walks(walks)
    # Tag constructor-created resources
    for r in publishers + subscribers + timers + service_servers + service_clients + action_servers + action_clients:
        r.created_in = "constructor"

    # Scan lifecycle methods for resource creation.
    # Lifecycle nodes create publishers/timers in on_configure/on_activate.
    # The gate uses a broad check so it fires even when the header chain can't
    # be fully resolved (e.g. nav2_util::LifecycleNode with missing headers).
    _is_lifecycle = is_lifecycle_base(base_class, lifecycle_callbacks)
    lifecycle_hooks_not_chained: list[str] = []
    if _is_lifecycle and lifecycle_callbacks:
        try:
            from clang.cindex import CursorKind
            for cursor in tu_full.cursor.walk_preorder():
                if cursor.kind != CursorKind.CXX_METHOD:
                    continue
                if cursor.semantic_parent != class_cursor:
                    continue
                if not cursor.is_definition():
                    continue
                if cursor.spelling not in lifecycle_callbacks:
                    continue
                lc_method = cursor.spelling
                # Does this override chain to the base? Only meaningful for the
                # hooks whose base implementation does work.
                if (
                    lc_method in LIFECYCLE_CHAINED_HOOKS
                    and lc_method not in lifecycle_hooks_not_chained
                    and not _hook_chains_to_base(cursor, lc_method)
                ):
                    lifecycle_hooks_not_chained.append(lc_method)
                # Walk this lifecycle method for resource creation
                (
                    lc_pubs, lc_subs, lc_timers,
                    lc_srv_servers, lc_srv_clients,
                    lc_act_servers, lc_act_clients,
                    lc_cbg, lc_params, lc_has_param_cb, lc_param_cb,
                    lc_pub_map, lc_svc_map, lc_act_map,
                ) = _walk_constructor(cursor, callbacks,
                                      class_method_bodies=class_method_bodies)
                # Tag lifecycle-created resources with the method name
                for r in lc_pubs + lc_subs + lc_timers + lc_srv_servers + lc_srv_clients + lc_act_servers + lc_act_clients:
                    r.created_in = lc_method
                publishers.extend(lc_pubs)
                subscribers.extend(lc_subs)
                timers.extend(lc_timers)
                service_servers.extend(lc_srv_servers)
                service_clients.extend(lc_srv_clients)
                action_servers.extend(lc_act_servers)
                action_clients.extend(lc_act_clients)
                callback_groups.extend(lc_cbg)
                parameters.extend(lc_params)
                has_param_callback = has_param_callback or lc_has_param_cb
                if lc_param_cb and not param_callback:
                    param_callback = lc_param_cb
                publisher_var_map.update(lc_pub_map)
                svc_client_var_map.update(lc_svc_map)
                act_client_var_map.update(lc_act_map)
                # Detect .reset() calls — tag matching resources with destroyed_in
                _detect_resets(cursor, lc_method,
                               publishers + subscribers + timers +
                               service_servers + service_clients +
                               action_servers + action_clients)
        except Exception:
            pass

    # A topic held in a member set once from a declared parameter.
    _bind_param_backed_topics(publishers + subscribers, class_cursor, tu_full)

    # The generator needs the qualified type of a lifecycle node's
    # action client, which libclang leaves as the alias when it cannot resolve
    # the interface header.
    _resolve_action_type_aliases(action_clients, class_cursor, tu_full)

    # Tag publishers as lifecycle publishers when the node is a lifecycle node
    if _is_lifecycle:
        for pub in publishers:
            pub.is_lifecycle = True

    # Timers created in a method that is neither the constructor nor a
    # recognized lifecycle hook — facts-only, never generated.
    setup_roots = [constructor_cursor, *(c for _b, c in base_ctors)]
    setup_roots += [
        cur for cur in tu_full.cursor.walk_preorder()
        if cur.kind == cindex.CursorKind.CXX_METHOD and cur.is_definition()
        and cur.spelling in lifecycle_callbacks
        and cur.semantic_parent == class_cursor
    ]
    unbound_timers = _find_out_of_scope_timers(
        tu_full, class_cursor, lifecycle_callbacks,
        scope_roots=setup_roots, methods=class_method_bodies)
    unbound_endpoints = _find_unbound_endpoints(
        tu_full, class_cursor, lifecycle_callbacks, setup_roots, class_method_bodies)
    template_base_endpoints = [
        {"base": t.spelling, "calls": ", ".join(calls)}
        for t in template_bases
        for calls in [_template_endpoint_calls(t)]
        if calls
    ]
    main_outside_callbacks = _find_main_outside_callbacks(tu_full, class_cursor)

    # tf2_ros use: the library owns the /tf publishers and the
    # listener's subscriptions, so no create_* call is visible. A source scan
    # recognises them; each broadcaster member becomes a publisher on /tf or
    # /tf_static whose sendTransform calls count as publishes, before the
    # callback bodies are analysed.
    tf2_info = None
    if "tf2_ros" in detected_includes:
        from colcon_lf.node_analysis import tf2_scan
        tf2_info = _scan_tf2(index, cindex, cpp_path, include_flags, class_name,
                             workspace_root)
        for br in (tf2_info.broadcasters if tf2_info else []):
            if not br.member or br.member in publisher_var_map:
                continue
            topic = tf2_scan.TF_STATIC_TOPIC if br.static else tf2_scan.TF_TOPIC
            publishers.append(PublisherInfo(
                topic=topic, message_type=tf2_scan.TF_MESSAGE_TYPE,
                member_variable=br.member, created_in="constructor", unconditional=True,
                **(tf2_scan.STATIC_BROADCASTER_QOS if br.static else tf2_scan.BROADCASTER_QOS),
            ))
            publisher_var_map[br.member] = topic

    # Extract member variables and analyze callback bodies
    member_vars = _extract_member_variables(class_cursor)
    _analyze_callback_bodies(
        class_cursor, tu_full, cpp_path,
        publisher_var_map, svc_client_var_map, act_client_var_map,
        member_vars, callbacks,
        action_servers=action_servers,
        action_clients=action_clients,
        service_clients=service_clients,
    )
    lambda_bodies = _synthesize_lambda_callbacks(
        subscribers, timers, callbacks, member_vars,
        publisher_var_map, svc_client_var_map, act_client_var_map,
    )
    _analyze_callers(class_cursor, tu_full, callbacks, lambda_bodies)
    _analyze_publish_bounds(
        class_cursor, tu_full, publisher_var_map, publishers, callbacks, lambda_bodies,
        _preamble_definition_resolver(index, cindex, cpp_path, include_flags))
    thread_action_senders = _find_thread_action_senders(
        constructor_cursor, class_method_bodies, action_servers)

    # Classify service client call styles by scanning callback method bodies.
    if service_clients:
        try:
            from clang.cindex import CursorKind
            for method_cursor in tu_full.cursor.walk_preorder():
                if method_cursor.kind != CursorKind.CXX_METHOD:
                    continue
                if not _in_family(method_cursor.semantic_parent, class_cursor):
                    continue
                if not method_cursor.is_definition():
                    continue
                style = _classify_service_client_call(method_cursor)
                if style is None:
                    continue
                # Apply to all service clients whose member variable appears in this method
                method_toks = set(_cursor_tokens(method_cursor))
                for cli in service_clients:
                    if cli.call_style is None and cli.member_variable and cli.member_variable in method_toks:
                        cli.call_style = style
        except Exception:
            pass

    # message_filters inputs: the library creates the rclcpp
    # subscription, so no create_subscription call is visible. A source scan
    # recognises two-input ExactTime/ApproximateTime synchronizers and adds
    # one subscription per input with a synthetic callback that carries the
    # synchronizer callback's effects. Unsupported uses land in
    # message_filters.unsupported and are refused by RCLLF-H003.
    message_filters_info = None
    if "message_filters" in detected_includes:
        from colcon_lf.node_analysis import message_filters_scan
        mf_sources = [cpp_path.read_text(errors="replace")]
        mf_files = {os.path.realpath(cpp_path)}
        mf_header = _find_header(cpp_path)
        if mf_header is not None:
            mf_sources.append(mf_header.read_text(errors="replace"))
            mf_files.add(os.path.realpath(mf_header))
        mf = message_filters_scan.scan(mf_sources, class_name)
        message_filters_info = mf.info
        for inp in mf.inputs:
            sync_cb = callbacks.get(inp.sync_callback)
            if sync_cb is None:
                message_filters_info.unsupported.append(
                    f"synchronizer callback {inp.sync_callback} is not a "
                    f"method of {class_name}"
                )
                continue
            merged = dataclasses.replace(sync_cb, name=inp.callback)
            if inp.observer is not None:
                # The observer runs in the same rclcpp callback
                # before the synchronizer (registration order); its reads and
                # writes join the input's reaction, effects are refused.
                obs_cb = callbacks.get(inp.observer)
                why = (f"observer {inp.observer} is not a method of {class_name}"
                       if obs_cb is None else message_filters_scan.observer_refusal(obs_cb))
                if why is not None:
                    message_filters_info.unsupported.append(why)
                    continue
                merged = dataclasses.replace(
                    merged,
                    variables_read=list(dict.fromkeys(
                        obs_cb.variables_read + merged.variables_read)),
                    variables_written=list(dict.fromkeys(
                        obs_cb.variables_written + merged.variables_written)))
            callbacks[inp.callback] = merged
            subscribers.append(SubscriberInfo(
                topic=inp.topic,
                message_type=inp.message_type,
                callback=inp.callback,
                member_variable=inp.member,
                created_in="constructor",
                sync_callback=inp.sync_callback,
                **_mf_input_qos(tu_full, class_cursor, mf_files, inp),
            ))

    # tf2 listener inputs: a TransformListener subscribes /tf and
    # /tf_static in a callback group of its own, spun by tf2's thread. The
    # binding claims both handles (kGroupTf2Listener) and runs the listener's
    # own callback, which feeds the Buffer, inside the input's LF reaction.
    # One synthetic callback with no effects serves both inputs; it lands in
    # the node's default group so it is serialised with the node's callbacks.
    if tf2_info is not None and not tf2_info.refusals:
        from colcon_lf.node_analysis import tf2_scan
        for lst in tf2_info.listeners:
            # tf2_ros creates /tf first, then /tf_static (TransformListener::init).
            for topic, qos in ((tf2_scan.TF_TOPIC, tf2_scan.LISTENER_QOS),
                               (tf2_scan.TF_STATIC_TOPIC, tf2_scan.STATIC_LISTENER_QOS)):
                name = tf2_scan.ingest_callback(lst.member, topic)
                callbacks[name] = CallbackInfo(name=name)
                subscribers.append(SubscriberInfo(
                    topic=topic, message_type=tf2_scan.TF_MESSAGE_TYPE, callback=name,
                    member_variable=lst.member, created_in="constructor",
                    tf2_listener=lst.member, **qos,
                ))

    # Synthesize default MutuallyExclusive callback group for any entity
    # (trigger OR client waitable) that has no explicit group. This matches
    # rclcpp's runtime behavior: every entity created without a callback_group
    # argument joins the node's single default MutuallyExclusive group
    # (node_base_->get_default_callback_group()).
    #
    # Name is "default_group" (not "default") so lf_naming.sanitize_lf_identifier
    # passes it through unchanged — "default" is a C++ reserved word, which the
    # sanitizer rewrites to "default_id", yielding an awkward _DefaultId sub-reactor.
    default_group_members = (
        list(timers) + list(subscribers)
        + list(service_servers) + list(action_servers)
        + list(service_clients) + list(action_clients)
    )
    if any(m.callback_group is None for m in default_group_members):
        callback_groups.append(
            CallbackGroupInfo(name="default_group", type="MutuallyExclusive")
        )
        for m in default_group_members:
            if m.callback_group is None:
                m.callback_group = "default_group"

    # Prefer the ROS name the node actually registers with (the base
    # initializer literal); fall back to the class-derived guess only when the
    # constructor supplies no literal (name passed in as an argument / via
    # NodeOptions). See _ros_node_name_from_constructor.
    _ros_name = _ros_node_name_from_constructor(constructor_cursor)

    result = NodeInfo(
        name=_ros_name
        or class_name.lower().replace("node", "_node").strip("_")
        or class_name,
        class_name=class_name,
        qualified_name=_qualified_class_name(class_cursor),
        package=_guess_package(cpp_path),
        source_file=cpp_path,
        header_file=_find_header(cpp_path),
        base_class=base_class,
        is_component=is_component,
        callbacks=list(callbacks.values()),
        callback_groups=callback_groups,
        timers=timers,
        subscribers=subscribers,
        service_servers=service_servers,
        action_servers=action_servers,
        publishers=publishers,
        service_clients=service_clients,
        action_clients=action_clients,
        parameters=parameters,
        member_variables=member_vars,
        lifecycle_callbacks=lifecycle_callbacks,
        lifecycle_hooks_not_chained=lifecycle_hooks_not_chained,
        has_param_callback=has_param_callback,
        param_callback=param_callback,
        parse_warnings=parse_warnings,
        unresolved_includes=unresolved_includes,
        detected_includes=detected_includes,
        detected_take_calls=detected_take_calls,
        detected_ros_sleep_calls=detected_ros_sleep_calls,
        unbound_timers=unbound_timers,
        message_filters=message_filters_info,
        tf2=tf2_info,
        thread_action_senders=thread_action_senders,
        unbound_endpoints=unbound_endpoints,
        compile_defines=list(extra_defines or []),
        template_base_endpoints=template_base_endpoints,
        main_outside_callbacks=main_outside_callbacks,
        compile_variant_issues=compile_variant_issues,
        base_classes_merged=[b.spelling for b in family],
    )
    _parse_node_cache[cache_key] = result
    return result


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _guess_package(cpp_path: Path) -> str:
    """Walk up from the source file to find the package name (package.xml)."""
    path = cpp_path.parent
    for _ in range(5):
        if (path / "package.xml").exists():
            return manifest_package_name(path / "package.xml") or path.name
        path = path.parent
    return cpp_path.parent.parent.name


class NodeSourceError(Exception):
    """The source defining a node class could not be found.

    Deliberately not a ``RuntimeError``: graph construction skips a node on
    ``RuntimeError`` with a warning, which would yield a silent empty node.
    This error must reach the user and fail analysis.
    """


def _describe_tus(tus: list[Path] | None) -> str:
    if tus is None:
        return "target translation units unknown: no compile_commands.json entry"
    return "checked " + (", ".join(str(t) for t in tus) or "no other file")


def _entry_object_target(entry: dict) -> str | None:
    """CMake target an compile_commands.json entry belongs to.

    Read from the object path (``-o CMakeFiles/<target>.dir/...``) that the
    build system itself assigned.
    """
    try:
        toks = shlex.split(entry.get("command", ""))
    except ValueError:
        return None
    for i, t in enumerate(toks[:-1]):
        if t == "-o":
            for part in Path(toks[i + 1]).parts:
                if part.endswith(".dir"):
                    return part[: -len(".dir")]
    return None


def _entry_source(entry: dict, workspace_root: Path) -> Path:
    path = Path(entry.get("file", ""))
    if path.exists():
        return path.resolve()
    for prefix in ("/workspace/", "/ws/", "/colcon_ws/"):
        if str(path).startswith(prefix):
            alt = workspace_root / str(path)[len(prefix):]
            if alt.exists():
                return alt.resolve()
    return path


def _linked_target_names(pkg_build: Path, target: str) -> set[str]:
    """Targets of the same package that *target* links, read from the build files.

    Works for both generators: ``CMakeFiles/<target>.dir/link.txt`` (Makefiles)
    or the link edge of ``build.ninja``. A link line names other targets either
    as libraries (``lib<name>.so`` / ``.a``) or as object-library objects
    (``CMakeFiles/<name>.dir/...o``). Only names with a ``CMakeFiles/<name>.dir``
    directory are accepted, so system libraries are ignored.
    """
    tokens: list[str] = []
    link_txt = pkg_build / "CMakeFiles" / f"{target}.dir" / "link.txt"
    ninja = pkg_build / "build.ninja"
    try:
        if link_txt.exists():
            tokens = shlex.split(link_txt.read_text())
        elif ninja.exists():
            own = f"CMakeFiles/{target}.dir/"
            # Group build edges with their indented variables (LINK_LIBRARIES).
            edges: list[list[str]] = []
            for line in ninja.read_text().replace("$\n", " ").splitlines():
                if line.startswith("build "):
                    edges.append([line])
                elif line.startswith(" ") and edges:
                    edges[-1].append(line)
            for e in edges:
                head = e[0]
                if "LINKER__" in head and own in head:
                    tokens = shlex.split(" ".join(e).replace("$:", ":"))
                    break
    except (ValueError, OSError):
        return set()
    names: set[str] = set()
    for tok in tokens:
        path = Path(tok)
        for part in path.parts:
            if part.endswith(".dir") and tok.endswith(".o"):
                names.add(part[: -len(".dir")])
        base = path.name
        if base.startswith("lib") and base.endswith((".so", ".a")):
            names.add(base[3:].rsplit(".", 1)[0])
    return {n for n in names if (pkg_build / "CMakeFiles" / f"{n}.dir").is_dir()}


def _target_translation_units(
    cpp_path: Path, workspace_root: Path,
) -> list[list[Path]] | None:
    """Candidate translation units for the members of a class declared via *cpp_path*.

    Returns search tiers, narrowest first, each without *cpp_path* itself:
    1. sources of every CMake target that compiles *cpp_path* plus the
       same-package targets they link (libraries and object libraries);
    2. every translation unit of the same package's compile_commands.json.
    ``None`` when no compile_commands.json entry matches *cpp_path*.
    """
    build_root = workspace_root / "build"
    if not build_root.is_dir():
        return None
    me = cpp_path.resolve()
    for pkg_build in sorted(build_root.iterdir()):
        ccj = pkg_build / "compile_commands.json"
        if not ccj.exists():
            continue
        try:
            db = json.loads(ccj.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        mine = [e for e in db
                if _match_compile_entry(e.get("file", ""), cpp_path, workspace_root)]
        if not mine:
            continue
        targets: set[str] = set()
        for e in mine:
            t = _entry_object_target(e)
            if t is not None:
                targets.add(t)
                targets |= _linked_target_names(pkg_build, t)
        narrow: list[Path] = []
        package: list[Path] = []
        for e in db:
            src = _entry_source(e, workspace_root)
            if src == me:
                continue
            if src not in package:
                package.append(src)
            if _entry_object_target(e) in targets and src not in narrow:
                narrow.append(src)
        return [narrow, package]
    return None


def _tus_defining_members(
    index: Any, cindex: Any, class_cursor: Any, tus: list[Path] | None,
    workspace_root: Path, cpp_path: Path,
) -> list[Path]:
    """TUs among *tus* that define a constructor or method of the class.

    The class is identified by clang USR, so a same-named class in another
    namespace is not matched.
    """
    if not tus:
        return []
    from clang.cindex import CursorKind
    usr = class_cursor.get_usr()
    found: list[Path] = []
    for tu_path in tus:
        if not tu_path.exists():
            continue
        tu = index.parse(
            str(tu_path),
            args=_load_include_flags(tu_path, workspace_root),
            options=_parse_options(cindex),
        )
        for c in tu.cursor.walk_preorder():
            if c.kind not in (CursorKind.CXX_METHOD, CursorKind.CONSTRUCTOR):
                continue
            if not c.is_definition():
                continue
            parent = c.semantic_parent
            if parent is None or parent.get_usr() != usr:
                continue
            f = c.location.file
            if f is not None and Path(f.name).resolve() == tu_path:
                found.append(tu_path)
                break
    return found


def _has_constructor_definition(class_cursor: Any, tu: Any) -> bool:
    from clang.cindex import CursorKind
    usr = class_cursor.get_usr()
    for c in tu.cursor.walk_preorder():
        if (c.kind == CursorKind.CONSTRUCTOR and c.is_definition()
                and c.semantic_parent is not None
                and c.semantic_parent.get_usr() == usr):
            return True
    return False


def _class_body_lives_elsewhere(class_cursor: Any, tu: Any, cpp_path: Path) -> bool:
    """True if the class is defined in another file and the primary file
    defines none of its members (the main-only source layout)."""
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return False
    loc = class_cursor.location.file
    cpp_resolved = cpp_path.resolve()
    if loc is None or Path(loc.name).resolve() == cpp_resolved:
        return False
    for cursor in tu.cursor.walk_preorder():
        if cursor.kind not in (CursorKind.CXX_METHOD, CursorKind.CONSTRUCTOR):
            continue
        if cursor.semantic_parent != class_cursor or not cursor.is_definition():
            continue
        f = cursor.location.file
        if f is not None and Path(f.name).resolve() == cpp_resolved:
            return False
    return True


def _own_definition_files(cpp_resolved: Path | None, class_cursor: Any) -> set[Path]:
    """Files whose method definitions belong to the node being analysed.

    The primary source file, plus the file holding the class definition when
    that is another file (a header-only node whose source only has ``main()``).
    Methods defined in the class body of a header are then
    visible; anything else pulled in by includes stays excluded.
    """
    files: set[Path] = set()
    if cpp_resolved:
        files.add(cpp_resolved)
    loc = class_cursor.location.file
    if loc is not None:
        files.add(Path(loc.name).resolve())
    for base in _class_family(class_cursor)[0]:
        base_loc = base.location.file
        if base_loc is not None:
            files.add(Path(base_loc.name).resolve())
    return files


def _qualified_class_name(cursor: Any) -> str:
    """Return ``ns::Outer::Cls`` for a class cursor from its semantic parents.

    Read from the AST, never guessed from the package name.  Anonymous
    namespaces are skipped (the name is not spellable from another scope).
    """
    try:
        from clang.cindex import CursorKind
    except ImportError:
        return cursor.spelling
    parts = [cursor.spelling]
    parent = cursor.semantic_parent
    while parent is not None and parent.kind != CursorKind.TRANSLATION_UNIT:
        if parent.spelling:
            parts.append(parent.spelling)
        parent = parent.semantic_parent
    return "::".join(reversed(parts))


# tf2 use the scan must see (F91b). A user file (the node's .cpp, its package's
# headers, any header outside the system include roots) that includes one of
# these headers, or a tf2_ros header that pulls one in, anchors the node: its
# scan must yield a record of the anchored type (F91c), else it is refused.
_TF2_ANCHOR_HEADERS = {"tf2_ros/buffer.h": "Buffer", "tf2_ros/buffer.hpp": "Buffer",
                       "tf2_ros/transform_listener.h": "TransformListener",
                       "tf2_ros/transform_listener.hpp": "TransformListener"}
# Headers of rcllf's own tf2 adapter (packages/lf-frame/rcllf_tf2), which take
# a tf2_ros::Buffer by reference; they are part of rcllf, not user code.
_TF2_TRUSTED_INCLUDE_DIR = ("include", "rcllf_tf2")
_TF2_TYPE_RE = re.compile(r"\btf2_ros::(Buffer|TransformListener)\b")


@dataclasses.dataclass(frozen=True)
class _Tf2Roots:
    """Where user code ends for the tf2 scan of one translation unit (F91c).

    ``user``: the workspace (source tree, build and install space).
    ``system``: the compiler's system include roots (the ``-isystem`` dirs and
    the toolchain's implicit dirs) and the ROS underlay prefixes. A header is
    system only under a system root and outside the workspace, never by a bare
    ``/usr`` or ``/opt`` prefix: a workspace under /opt is still user code.
    """

    user: tuple[str, ...]
    system: tuple[str, ...]
    # Why the split cannot be told for this translation unit (X91r2): a
    # workspace root of "/", or an ament prefix outside the workspace that
    # holds a workspace package next to packages of other origin. Non-empty
    # refuses the node's tf2 use (``tf2_listener_unadmitted``).
    refusal: str = ""


def _under(path: str, roots: tuple[str, ...]) -> bool:
    return any(path == r or path.startswith(r + os.sep) for r in roots)


@functools.lru_cache(maxsize=None)
def _toolchain_include_dirs() -> tuple[str, ...]:
    """The implicit ``#include <...>`` dirs the C++ toolchain reports (``-E -v``)."""
    for cxx in ("c++", "g++", "clang++"):
        exe = shutil.which(cxx)
        if exe is None:
            continue
        try:
            err = subprocess.run([exe, "-xc++", "-E", "-v", "-"], input="", text=True,
                                 capture_output=True, timeout=60).stderr
        except (OSError, subprocess.SubprocessError):
            continue
        dirs: list[str] = []
        inside = False
        for line in err.splitlines():
            if line.startswith("#include <...> search starts here:"):
                inside = True
            elif line.startswith("End of search list."):
                break
            elif inside and line.strip():
                dirs.append(line.strip().split(" (framework directory)")[0])
        if dirs:
            return tuple(dirs)
    return ()


def _ament_prefix_of(include_dir: str) -> str | None:
    """The ament install prefix holding *include_dir* (``/opt/ros/jazzy`` for
    ``-I/opt/ros/jazzy/include/rclcpp``), or None."""
    path = Path(include_dir)
    for parent in (path, *path.parents):
        if (parent / "share" / "ament_index" / "resource_index").is_dir():
            return str(parent)
    return None


_PACKAGE_NAME_RE = re.compile(r"<name>\s*([^<\s]+)\s*</name>")


@functools.lru_cache(maxsize=None)
def _workspace_packages(root: str) -> frozenset[str]:
    """Names of the packages whose sources live in the workspace *root* (X91r2).

    The colcon discovery shape: a directory with a package.xml is one package
    and is not searched further; ``COLCON_IGNORE``/``AMENT_IGNORE`` and hidden
    directories are skipped, and so are the workspace's own build, install and
    log spaces.
    """
    names: set[str] = set()
    for dirpath, dirnames, filenames in os.walk(root):
        if "COLCON_IGNORE" in filenames or "AMENT_IGNORE" in filenames:
            dirnames[:] = []
            continue
        if "package.xml" in filenames:
            dirnames[:] = []
            try:
                m = _PACKAGE_NAME_RE.search(Path(dirpath, "package.xml").read_text(errors="replace"))
            except OSError:
                m = None
            if m:
                names.add(m.group(1))
            continue
        dirnames[:] = [d for d in dirnames if not d.startswith(".")
                       and not (dirpath == root and d in ("build", "install", "log"))]
    return frozenset(names)


def _prefix_packages(prefix: str) -> frozenset[str]:
    """The packages an ament prefix registers in its resource index."""
    try:
        return frozenset(os.listdir(Path(prefix, "share", "ament_index", "resource_index",
                                         "packages")))
    except OSError:
        return frozenset()


def _tf2_roots(include_flags: list[str], workspace_root: Path | None) -> _Tf2Roots:
    """The user and system roots of one translation unit (F91c, X91r2).

    System: every ``-isystem`` dir of *include_flags*, the toolchain's implicit
    dirs, the sourced underlays (``AMENT_PREFIX_PATH``) and the ament prefix of
    any ``-I`` dir (the install-tree fallback passes ``/opt/ros/<distro>`` as
    ``-I``). User: *workspace_root*, which wins over every system root, and the
    workspace's own install space wherever it lives (X91r2): an ament prefix
    whose resource index registers only packages whose sources are in the
    workspace (``colcon build --install-base /elsewhere`` copies their headers
    there, and CMake passes them as ``-isystem``). A prefix that registers a
    workspace package next to packages from elsewhere cannot be told apart,
    and neither can a workspace at ``/``: both set ``refusal``.
    """
    system = list(_toolchain_include_dirs())
    system += os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    owners: list[str] = []
    for i, flag in enumerate(include_flags):
        # Joined (-isystemDIR, -IDIR) or split (-isystem DIR, -I DIR).
        nxt = include_flags[i + 1] if i + 1 < len(include_flags) else ""
        if flag.startswith("-isystem"):
            path = flag[len("-isystem"):] or nxt
            system.append(path)
            owners.append(_ament_prefix_of(path) or "")
        elif flag.startswith("-I"):
            system.append(_ament_prefix_of(flag[2:] or nxt) or "")

    def norm(paths: list[str]) -> tuple[str, ...]:
        return tuple(sorted({os.path.realpath(p) for p in paths if p} - {os.sep}))

    refusal = ""
    user: list[str] = []
    if workspace_root is not None:
        ws = os.path.realpath(str(workspace_root))
        if ws == os.sep:
            refusal = (
                "the workspace root is '/', so every header on the machine would be user "
                "code and the tf2 scan cannot tell the node's files from ROS and the "
                "toolchain; build the workspace in its own directory")
        else:
            user.append(ws)
            mine = _workspace_packages(ws)
            for prefix in norm(system + owners):
                if _under(prefix, (ws,)) or not mine:
                    continue
                registered = _prefix_packages(prefix)
                ours = sorted(registered & mine)
                if not ours:
                    continue
                if registered <= mine:
                    user.append(prefix)  # the workspace's own install space
                elif not refusal:
                    refusal = (
                        f"the ament prefix {prefix} holds the workspace package(s) "
                        f"{', '.join(ours[:3])} next to {len(registered - mine)} package(s) "
                        "whose sources are not in the workspace, so the tf2 scan cannot tell "
                        "which of its headers are user code; install the workspace into its "
                        "own prefix (colcon build --install-base <dir> of this workspace only)")
    return _Tf2Roots(norm(user), norm(system), refusal)


def _tf2_file_kind(path: str, roots: _Tf2Roots | None = None) -> str:
    """``system``, ``trusted`` (rcllf_tf2's headers) or ``user``.

    Without *roots*, the system roots are the toolchain's and the sourced
    underlays' and there is no workspace.
    """
    if roots is None:
        roots = _tf2_roots([], None)
    resolved = os.path.realpath(path)
    parts = Path(resolved).parts
    for i in range(len(parts) - 1):
        if parts[i:i + 2] == _TF2_TRUSTED_INCLUDE_DIR:
            return "trusted"
    if _under(resolved, roots.user):
        return "user"
    if _under(resolved, roots.system):
        return "system"
    return "user"


def _tf2_includes(tu: Any, cpp_path: Path,
                  roots: _Tf2Roots | None = None) -> tuple[list[str], list[tuple[str, str]]]:
    """User files of *tu* and the includes that anchor it to tf2 (F91b, F91c).

    Both come from the compiler's own include graph (``get_includes``), so a
    header is found however it is named or nested. An anchor is an include,
    written in a user file, of a tf2_ros header that is or pulls in
    ``tf2_ros/buffer.h`` or ``tf2_ros/transform_listener.h``. Each is returned
    as ``("<file>:<line> includes <header>", type)``, type ``Buffer`` or
    ``TransformListener`` (the anchor header's class; a header that pulls one
    in carries the type of each anchor header it reaches), first anchor per
    type. An unresolved tf2_ros include other than the broadcasters' anchors as
    well (fail closed: what it pulls in is unknown), type ``any`` unless its
    spelling names an anchor header.
    """
    if roots is None:
        roots = _tf2_roots([], None)
    main = os.path.realpath(str(cpp_path))
    edges: list[tuple[str, int, str]] = []
    for inc in tu.get_includes():
        if inc.source is None or inc.include is None:
            continue
        edges.append((os.path.realpath(inc.source.name), inc.location.line,
                      os.path.realpath(inc.include.name)))
    users = [main] + sorted({dst for _s, _l, dst in edges
                             if _tf2_file_kind(dst, roots) == "user"} - {main})
    children: dict[str, set[str]] = {}
    for src, _line, dst in edges:
        children.setdefault(src, set()).add(dst)

    def anchor_type(path: str) -> str | None:
        norm = path.replace(os.sep, "/")
        return next((k for h, k in _TF2_ANCHOR_HEADERS.items() if norm.endswith("/" + h)), None)

    def anchor_types(path: str) -> set[str]:
        """The anchor types *path* is or pulls in; an anchor header's own
        includes do not widen its type."""
        seen: set[str] = set()
        todo, found = [path], set()
        while todo:
            cur = todo.pop()
            if cur in seen:
                continue
            seen.add(cur)
            kind = anchor_type(cur)
            if kind is not None:
                found.add(kind)
                continue
            todo.extend(children.get(cur, ()))
        return found

    user_set = set(users)
    anchors: dict[str, str] = {}
    for src, line, dst in sorted(edges):
        if src in user_set and "/tf2_ros/" in dst.replace(os.sep, "/"):
            # Jazzy installs include/tf2_ros/tf2_ros/<header>: the last component.
            tail = dst.replace(os.sep, "/").rsplit("/tf2_ros/", 1)[1]
            for kind in sorted(anchor_types(dst)):
                anchors.setdefault(kind, f"{os.path.basename(src)}:{line} includes tf2_ros/{tail}")
    try:
        from clang.cindex import CursorKind
        for cur in tu.cursor.get_children():
            if cur.kind != CursorKind.INCLUSION_DIRECTIVE or cur.location.file is None:
                continue
            spelling = cur.spelling or ""
            if (spelling.startswith("tf2_ros/") and cur.get_included_file() is None
                    and "broadcaster" not in spelling and spelling != "tf2_ros/qos.hpp"
                    and os.path.realpath(cur.location.file.name) in user_set):
                anchors.setdefault(_TF2_ANCHOR_HEADERS.get(spelling, "any"), (
                    f"{os.path.basename(cur.location.file.name)}:"
                    f"{cur.location.line} includes {spelling} (unresolved)"))
    except Exception:
        pass
    return users, [(desc, kind) for kind, desc in sorted(anchors.items())]


def _tf2_compiler_refs(tu: Any, user_files: set[str]) -> list[tuple[str, int, int, str]]:
    """``(file, line, byte column, type)`` of every reference the compiler resolves to
    tf2_ros::Buffer or tf2_ros::TransformListener in a user file (F91b).

    Counted: a written type name (TYPE_REF, also through a typedef or alias),
    a direct constructor call, and a variable or member whose type contains
    either class. A macro is expanded by then, so ``TF2_NS::TransformListener``
    counts like the spelled-out name, at the place it is used.
    """
    from clang.cindex import CursorKind

    def tf2_class(cur: Any) -> str | None:
        if cur is None:
            return None
        if cur.kind in (CursorKind.CLASS_DECL, CursorKind.STRUCT_DECL):
            parent = cur.semantic_parent
            if (cur.spelling in ("Buffer", "TransformListener") and parent is not None
                    and parent.spelling == "tf2_ros"):
                return cur.spelling
            return None
        if cur.kind in (CursorKind.TYPEDEF_DECL, CursorKind.TYPE_ALIAS_DECL):
            m = _TF2_TYPE_RE.search(cur.underlying_typedef_type.get_canonical().spelling)
            return m.group(1) if m else None
        return None

    refs: set[tuple[str, int, int, str]] = set()
    for top in tu.cursor.get_children():
        if top.location.file is None or os.path.realpath(top.location.file.name) not in user_files:
            continue
        for cur in top.walk_preorder():
            loc = cur.location
            if loc.file is None:
                continue
            path = os.path.realpath(loc.file.name)
            if path not in user_files:
                continue
            kind = None
            try:
                if cur.kind == CursorKind.TYPE_REF:
                    kind = tf2_class(cur.referenced)
                elif cur.kind == CursorKind.CALL_EXPR and cur.referenced is not None \
                        and cur.referenced.kind == CursorKind.CONSTRUCTOR:
                    kind = tf2_class(cur.referenced.semantic_parent)
                elif cur.kind in (CursorKind.VAR_DECL, CursorKind.FIELD_DECL):
                    m = _TF2_TYPE_RE.search(cur.type.get_canonical().spelling)
                    kind = m.group(1) if m else None
            except Exception:
                kind = None
            if kind is not None:
                refs.add((path, loc.line, loc.column, kind))
    return sorted(refs)


def _scan_tf2(index: Any, cindex: Any, cpp_path: Path, include_flags: list[str],
              class_name: str, workspace_root: Path | None = None) -> Any:
    """The tf2 scan of one node, closed against forms the regex cannot see (F91b).

    Sources are the node's .cpp and every user header of its translation unit
    (the compiler's include graph; before F91b only ``include/**/<stem>.hpp``).
    A full parse (function bodies of headers kept) then lists every reference
    the compiler resolves to tf2_ros::Buffer or tf2_ros::TransformListener in
    those files; one outside every span the scan read as that type is
    refused (a span, not a line: a macro listener next to an admitted Buffer
    on one line hides nothing), and so is a
    node anchored to tf2 by a direct include whose scan yields no Buffer and
    no TransformListener.
    """
    from colcon_lf.node_analysis import tf2_scan
    from colcon_lf.node_analysis.models import Tf2Info, Tf2Refusal

    tu = index.parse(str(cpp_path), args=include_flags,
                     options=cindex.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD)
    roots = _tf2_roots(include_flags, workspace_root)
    users, anchors = _tf2_includes(tu, cpp_path, roots)
    paths = list(users)
    header = _find_header(cpp_path)
    if header is not None and os.path.realpath(str(header)) not in paths:
        paths.append(os.path.realpath(str(header)))
    sources = []
    for path in paths:
        try:
            sources.append((path, Path(path).read_text(errors="replace")))
        except OSError:
            continue
    texts = dict(sources)
    accounted: set[tuple[str, int, int, str]] = set()
    info = tf2_scan.scan(sources, class_name, accounted)
    spans: dict[str, list[tuple[int, int, str]]] = {}
    for path, start, end, kind in accounted:
        spans.setdefault(path, []).append((start, end, kind))

    def char_offset(path: str, line: int, column: int) -> int | None:
        """The text offset of clang's 1-based line and byte column."""
        lines = texts.get(path, "").split("\n")
        if not 1 <= line <= len(lines):
            return None
        head = lines[line - 1].encode("utf-8", "replace")[:max(column - 1, 0)]
        return sum(len(x) + 1 for x in lines[:line - 1]) + len(head.decode("utf-8", "replace"))

    refusals: list[Tf2Refusal] = []
    for path, line, column, kind in _tf2_compiler_refs(tu, set(paths)):
        pos = char_offset(path, line, column)
        if pos is not None and any(a <= pos < b and k in (kind, "any")
                                   for a, b, k in spans.get(path, ())):
            continue
        refusals.append(Tf2Refusal(
            "tf2_listener_unadmitted",
            f"tf2_ros::{kind} at {os.path.basename(path)}:{line}: the compiler resolves this to "
            f"tf2_ros::{kind}, but the source scan reads no admitted or refused form of it "
            "there (a macro, an alias or a deduced type hides the spelling); write "
            f"std::make_shared<tf2_ros::{kind}>(...) in the constructor"))
    # Each anchor needs a record of its own type (F91c): a Buffer record does not
    # account for an include of transform_listener.h, whose listener may be
    # made by a helper in a compiled library.
    have = {"Buffer": bool(info and info.buffers),
            "TransformListener": bool(info and info.listeners)}
    for anchor, kind in anchors:
        if (have["Buffer"] or have["TransformListener"]) if kind == "any" else have[kind]:
            continue
        what = "tf2_ros::Buffer or TransformListener" if kind == "any" else f"tf2_ros::{kind}"
        refusals.append(Tf2Refusal(
            "tf2_listener_unadmitted",
            f"{anchor}, but the source scan finds no {what} "
            "made with std::make_shared/make_unique in the node's files; one made "
            "elsewhere (a helper in another library, a macro) would take /tf outside LF. "
            "Make it in the constructor, or drop the include"))
    if roots.refusal:
        refusals.append(Tf2Refusal(
            "tf2_listener_unadmitted",
            f"{os.path.basename(str(cpp_path))}: {roots.refusal}"))
    # Request stamps come from this translation unit's AST, never from the
    # source text.
    raw: dict[str, bytes] = {}
    for path in paths:
        try:
            raw[path] = Path(path).read_bytes()
        except OSError:
            continue
    stamps, stamp_refusals = tf2_scan.ast_stamps(
        tu, set(paths), raw,
        sorted({a.member for a in (info.adapters if info else []) if a.member}))
    if info is not None:
        info.stamps = stamps
    elif stamps or stamp_refusals:
        info = Tf2Info(stamps=stamps)
    refusals.extend(stamp_refusals)
    if refusals:
        info = info if info is not None else Tf2Info()
        info.refusals.extend(refusals)
    return info


def _find_header(cpp_path: Path) -> Path | None:
    """Attempt to find the corresponding header file."""
    stem = cpp_path.stem
    # Look in sibling include/ directories
    pkg_root = cpp_path.parent.parent
    for inc_dir in pkg_root.rglob("include"):
        for header in inc_dir.rglob(f"{stem}.hpp"):
            return header
    return None
