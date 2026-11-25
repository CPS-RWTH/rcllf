# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Workspace detection and LF system discovery."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

# --- launch stem -> generated lfc package/reactor name transform ---
#
# KEEP IN SYNC with colcon_lf/generation/lf_naming.py's
# launch_stem_to_reactor_name() + sanitize_lf_identifier(). ros2lf depends
# only on ros2cli (see tools/ros2lf/pyproject.toml) and cannot import
# colcon_lf, so this is a small, faithful re-implementation rather than a
# shared import. If lf_naming.py's transform changes, update this too.
_INVALID_IDENT_CHARS = re.compile(r"[^a-zA-Z0-9_]")

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


def _sanitize_lf_identifier(name: str) -> str:
    """Mirror lf_naming.sanitize_lf_identifier."""
    result = _INVALID_IDENT_CHARS.sub("_", name)
    result = result.strip("_") or "unnamed"
    if result[0].isdigit():
        result = f"_{result}"
    if result in _CPP_RESERVED:
        result = f"{result}_id"
    return result


# Mirror lf_naming.DIGIT_STEM_PREFIX.
_DIGIT_STEM_PREFIX = "Launch_"


def _launch_stem_to_reactor_name(stem: str) -> str:
    """Mirror lf_naming.launch_stem_to_reactor_name (PascalCase, unsanitized).

    Unlike the generator it does not refuse an invalid name: discovery only
    guesses package names and must not raise on a stray launch file.
    """
    parts = stem.split("_")
    name = "".join(p.capitalize() for p in parts if p) or "Main"
    if name[0].isdigit():
        name = f"{_DIGIT_STEM_PREFIX}{name}"
    return name


def _generated_package_stem(reactor_name: str) -> str:
    """Mirror lf_naming.generated_package_stem: lowered, digit prefix dropped."""
    rest = reactor_name[len(_DIGIT_STEM_PREFIX):]
    if reactor_name.startswith(_DIGIT_STEM_PREFIX) and rest[:1].isdigit():
        reactor_name = rest
    return reactor_name.lower()


def _lfc_generated_name(stem: str) -> str:
    """The <name> lf-generate uses for lf_generated_<name> and lf_<name>.launch.py.

    generate.py: ``stem = lf_system.name.lower()`` where ``lf_system.name`` is
    built from ``launch_stem_to_reactor_name(original_stem)`` (PascalCase,
    e.g. ``lifecycle_node`` -> ``LifecycleNode``), then lowered here to
    ``lifecyclenode``. sanitize_lf_identifier is applied to the stem's parts
    only indirectly (capitalize() doesn't touch invalid chars); mirrored here
    for parity with lf_naming.py even though launch stems are typically
    already valid identifiers.
    """
    return _generated_package_stem(
        _sanitize_lf_identifier(_launch_stem_to_reactor_name(stem))
    )


def _generated_package_name_for(package: str, name_lower: str) -> str:
    """The ``lf_generated_<package>_<name>`` colcon package name.

    Package-qualified so two source packages sharing a launch stem (e.g.
    pubsub_example and service_example both analyzing bringup.launch.py)
    never collide on the same generated package name.

    KEEP IN SYNC with colcon_lf/generation/lf_naming.py's
    ``generated_lfc_package_name`` — see the module note above.
    """
    return f"lf_generated_{_sanitize_lf_identifier(package).lower()}_{name_lower}"


def generated_package_name(package: str, stem: str) -> str:
    """The ``lf_generated_<package>_<name>`` colcon package name for a
    source *package* and launch *stem* (best-effort guess before generation
    has run; the real name may differ if the generator appended "System" to
    de-conflict with a node class name — see ``_load_entry``, which prefers
    reading the actual generated .lf filename when it exists).
    """
    return _generated_package_name_for(package, _lfc_generated_name(stem))


def generated_launch_filename(stem: str) -> str:
    """The ``lf_<name>.launch.py`` filename lf-generate installs for a stem."""
    return f"lf_{_lfc_generated_name(stem)}.launch.py"


@dataclass
class SystemEntry:
    """An LF-analyzed system discovered in the workspace."""

    package: str
    stem: str
    path: Path  # Path to system.json
    node_count: int
    has_components: bool
    has_non_components: bool
    has_launch: bool  # lf_generated_<name>/launch/lf_<name>.launch.py exists
    wrapper_built: bool  # lf_generated_<name> is colcon-installed (name kept for callers)
    layout: str | None = None  # "lfc" when generated, else None (stock layout removed)
    generated_package: str = ""  # lf_generated_<name>
    launch_filename: str = ""  # lf_<name>.launch.py
    federated: bool = False  # last lf-generate wrote a federated artifact
    generate_status: str = ""  # "generated" | "refused" | "failed" | "" (no record)
    refusal_code: str = ""  # RCLLF-H... of a refused system


def find_workspace(workspace_arg: str | None = None) -> Path | None:
    """Resolve the colcon workspace root."""
    if workspace_arg is not None:
        p = Path(workspace_arg)
        if p.is_dir():
            return p
        return None

    cwd = Path.cwd()
    for candidate in [cwd / "packages", cwd, cwd.parent]:
        if (candidate / "src").is_dir():
            return candidate
    return None


def discover_systems(
    workspace: Path,
    package_filter: str | None = None,
) -> list[SystemEntry]:
    """Scan lf-analysis/ for all analyzed systems."""
    base = workspace / "lf-analysis"
    if not base.is_dir():
        return []

    entries = []
    for sj_path in sorted(base.rglob("system.json")):
        try:
            rel = sj_path.parent.relative_to(base)
            parts = rel.parts
            if len(parts) < 2:
                continue
            pkg = parts[0]
            stem = parts[1]
        except (ValueError, IndexError):
            continue

        if package_filter and pkg != package_filter:
            continue

        entry = _load_entry(pkg, stem, sj_path)
        if entry:
            entries.append(entry)

    return entries


def stale_wrapper_artifacts(
    workspace: Path, package: str | None = None, stem: str | None = None,
) -> list[Path]:
    """Artifacts the removed wrapper path left in lf-analysis/.

    The old ``colcon lf`` wrote ``lf-analysis/<pkg>/<stem>/launch/``
    (``lf_<stem>.launch.py`` for the never-built ``lf_wrapper_*`` package),
    ``lf-analysis/<pkg>/<stem>/wrapper/`` and ``lf-analysis/COLCON_IGNORE``.
    Discovery ignores them, so they cannot shadow the lf_generated_* package,
    and ``ros2 lf`` names them so a user can delete them. COLCON_IGNORE is
    not listed: it keeps colcon out of lf-analysis/, and ``colcon lf-analyze``
    and lf-bench write it again, because lf-bench's lfc copy under
    ``lf-analysis/lf-generated/`` has the name of the ``lf-generated/`` package.
    """
    base = workspace / "lf-analysis"
    if not base.is_dir():
        return []
    found: list[Path] = []
    for system_dir in sorted(p.parent for p in base.glob("*/*/system.json")):
        if package is not None and system_dir.parent.name != package:
            continue
        if stem is not None and system_dir.name != stem:
            continue
        found += sorted((system_dir / "launch").glob("lf_*.launch.py"))
        if (system_dir / "wrapper").is_dir():
            found.append(system_dir / "wrapper")
    return found


def find_system(
    workspace: Path, package: str, stem: str
) -> SystemEntry | None:
    """Find a specific system by package and stem."""
    sj_path = workspace / "lf-analysis" / package / stem / "system.json"
    if not sj_path.is_file():
        return None
    return _load_entry(package, stem, sj_path)


def _load_entry(pkg: str, stem: str, sj_path: Path) -> SystemEntry | None:
    """Load a SystemEntry from a system.json file."""
    try:
        with open(sj_path) as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError):
        return None

    nodes = data.get("nodes", {})
    all_launch_nodes = data.get("launch_graph", {}).get("all_nodes", [])

    has_components = any(n.get("is_component") for n in all_launch_nodes)
    has_non_components = any(
        not n.get("is_component") for n in all_launch_nodes
    )

    # The lfc layout is the only one: lf-generate writes
    # <workspace>/lf-generated/lf_generated_<name>/ and "built" means colcon
    # installed its share/launch/ tree. <name> is the lower-cased main reactor
    # name; take it from the generated .lf next to system.json when present
    # (the generator appends "System" when a stem equals a node class name,
    # e.g. parameter_node -> ParameterNodeSystem), else derive it from the stem.
    workspace_root = sj_path.parent.parent.parent.parent
    lf_files = sorted((sj_path.parent / "lf").glob("*.lf"))
    if lf_files:
        name = _generated_package_stem(lf_files[0].stem)
        pkg_name = _generated_package_name_for(pkg, name)
        lfc_launch_filename = f"lf_{name}.launch.py"
    else:
        pkg_name = generated_package_name(pkg, stem)
        lfc_launch_filename = generated_launch_filename(stem)
    # The record lf-generate leaves next to system.json names the package it
    # wrote (a federated run writes lf_generated_<pkg>_<name>_federated with an
    # lf_<name>_federated.launch.py, not the single-process names guessed above).
    federated = False
    recorded = _recorded_package(sj_path.parent)
    if recorded is not None:
        base, federated = recorded
        launch_dir = workspace_root / "lf-generated" / base / "launch"
        found = sorted(launch_dir.glob("lf_*.launch.py")) if launch_dir.is_dir() else []
        if found:
            pkg_name, lfc_launch_filename = base, found[0].name
    has_launch = (
        workspace_root / "lf-generated" / pkg_name / "launch" / lfc_launch_filename
    ).is_file()
    wrapper_built = (
        workspace_root / "install" / pkg_name / "share" / pkg_name / "launch"
        / lfc_launch_filename
    ).is_file()
    layout = "lfc" if (has_launch or wrapper_built) else None
    status, code = _generate_status(sj_path.parent)

    return SystemEntry(
        package=pkg,
        stem=stem,
        path=sj_path,
        node_count=len(nodes),
        has_components=has_components,
        has_non_components=has_non_components,
        has_launch=has_launch,
        wrapper_built=wrapper_built,
        layout=layout,
        generated_package=pkg_name,
        launch_filename=lfc_launch_filename,
        federated=federated,
        generate_status=status,
        refusal_code=code,
    )


def _recorded_package(system_dir: Path) -> tuple[str, bool] | None:
    """``(package, federated)`` from ``lfc_packages.txt`` next to system.json."""
    try:
        parts = (system_dir / "lfc_packages.txt").read_text(encoding="utf-8").split()
    except OSError:
        return None
    if not parts:
        return None
    return parts[0], len(parts) > 1 and parts[1] == "federated"


def _generate_status(system_dir: Path) -> tuple[str, str]:
    """``(status, hazard code)`` from ``generate_status.json`` (``("", "")`` if absent)."""
    try:
        data = json.loads((system_dir / "generate_status.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return "", ""
    return str(data.get("status", "")), str(data.get("code") or "")
