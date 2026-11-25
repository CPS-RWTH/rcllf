# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""CMakeLists.txt parser for ROS2 packages.

Extracts:
- add_executable / ament_auto_add_executable(name src/file.cpp ...)
- add_library / ament_auto_add_library(name [SHARED|STATIC|MODULE] src/file.cpp ...)
- ament_target_dependencies(target dep1 dep2 ...)  (inline or multiline)
- rclcpp_components_register_nodes(target "pkg::Class")
- rclcpp_components_register_node(target PLUGIN "pkg::Class" EXECUTABLE exec_name)
- rosidl_generate_interfaces(... "msg/X.msg" ... DEPENDENCIES ...)
"""

from __future__ import annotations

import re
from pathlib import Path

from colcon_lf.node_analysis.models import CMakeInfo, LibraryTarget


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _strip_comments(text: str) -> str:
    """Remove CMake line comments (# ...) without touching strings."""
    return re.sub(r"#[^\n]*", "", text)


def _extract_call(text: str, func: str) -> list[str]:
    """Return a list of all argument strings inside calls to *func*.

    Handles multi-line calls by matching balanced parentheses.
    Returns one string per call, containing the raw argument text.
    """
    results = []
    pattern = re.compile(r"\b" + re.escape(func) + r"\s*\(", re.IGNORECASE)
    for m in pattern.finditer(text):
        start = m.end()  # position just after '('
        depth = 1
        i = start
        while i < len(text) and depth > 0:
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
            i += 1
        results.append(text[start : i - 1])
    return results


def _tokenize(args_text: str) -> list[str]:
    """Split CMake argument text into tokens.

    Handles quoted strings (strips quotes) and unquoted tokens.
    Ignores generator expressions like $<...>.
    """
    tokens: list[str] = []
    i = 0
    s = args_text.strip()
    while i < len(s):
        if s[i] in (" ", "\t", "\n", "\r"):
            i += 1
        elif s[i] == '"':
            # Quoted string
            end = s.find('"', i + 1)
            if end == -1:
                tokens.append(s[i + 1 :])
                break
            tokens.append(s[i + 1 : end])
            i = end + 1
        elif s[i] == "$":
            # Generator expression or variable — skip to matching '>'
            if i + 1 < len(s) and s[i + 1] == "<":
                depth = 1
                j = i + 2
                while j < len(s) and depth > 0:
                    if s[j] == "<":
                        depth += 1
                    elif s[j] == ">":
                        depth -= 1
                    j += 1
                i = j  # skip the whole generator expression
            else:
                # CMake variable like ${VAR} — include as token
                end = i
                while end < len(s) and s[end] not in (" ", "\t", "\n", "\r"):
                    end += 1
                tokens.append(s[i:end])
                i = end
        else:
            end = i
            while end < len(s) and s[end] not in (" ", "\t", "\n", "\r", ")"):
                end += 1
            token = s[i:end]
            if token:
                tokens.append(token)
            i = end
    return tokens


def _resolve_project_name(tokens: list[str], project_name: str) -> list[str]:
    """Replace ${PROJECT_NAME} with the actual project name."""
    return [t.replace("${PROJECT_NAME}", project_name) for t in tokens]


def _extract_variables(text: str) -> dict[str, list[str]]:
    """Extract ``set(VAR value ...)`` definitions into a variables dict.

    Stores all value tokens as a list so that multi-file ``set(SOURCES a b c)``
    variables expand correctly when used in ``add_library``.
    Resolves chained references like ``set(B ${A}_suffix)``.
    """
    variables: dict[str, list[str]] = {}
    for args_text in _extract_call(text, "set"):
        tokens = _tokenize(args_text)
        if len(tokens) >= 2:
            var_name = tokens[0]
            values = tokens[1:]
            # Skip CMake keywords that indicate special set() forms
            if values[0].upper() in ("CACHE", "PARENT_SCOPE", "FORCE"):
                continue
            # Strip trailing CACHE / PARENT_SCOPE sentinels
            clean_values = [v for v in values if v.upper() not in ("CACHE", "PARENT_SCOPE", "FORCE")]
            variables[var_name] = clean_values if clean_values else values[:1]

    # Resolve chained references (iterate until stable)
    for _ in range(5):
        changed = False
        for var, vals in variables.items():
            resolved = _resolve_var_refs_list(vals, variables)
            if resolved != vals:
                variables[var] = resolved
                changed = True
        if not changed:
            break

    return variables


def _resolve_var_refs(text: str, variables: dict[str, list[str]]) -> str:
    """Replace ``${VAR}`` references in *text* using *variables* dict.

    When a variable expands to multiple values, joins them with a space
    (suitable for single-token context; use ``_resolve_variables`` for lists).
    """
    def _repl(m: re.Match) -> str:
        vals = variables.get(m.group(1))
        if vals is None:
            return m.group(0)
        return " ".join(vals)
    return re.sub(r"\$\{(\w+)\}", _repl, text)


def _resolve_var_refs_list(tokens: list[str], variables: dict[str, list[str]]) -> list[str]:
    """Resolve ``${VAR}`` in a list of tokens, expanding list-valued variables."""
    result: list[str] = []
    _var_re = re.compile(r"\$\{(\w+)\}")
    for tok in tokens:
        m = _var_re.fullmatch(tok)
        if m:
            # Whole token is a variable reference — expand to potentially many tokens
            vals = variables.get(m.group(1))
            if vals is not None:
                result.extend(vals)
            else:
                result.append(tok)
        elif _var_re.search(tok):
            # Variable embedded in a larger string — join expansion inline
            result.append(_resolve_var_refs(tok, variables))
        else:
            result.append(tok)
    return result


def _resolve_variables(tokens: list[str], variables: dict[str, list[str]]) -> list[str]:
    """Resolve ``${VAR}`` references in a list of tokens."""
    return _resolve_var_refs_list(tokens, variables)


# ---------------------------------------------------------------------------
# Main parser
# ---------------------------------------------------------------------------

def parse(cmake_path: str | Path) -> CMakeInfo:
    """Parse a CMakeLists.txt file and return a :class:`CMakeInfo`.

    Parameters
    ----------
    cmake_path:
        Path to the CMakeLists.txt file.
    """
    cmake_path = Path(cmake_path)
    text = cmake_path.read_text(encoding="utf-8")
    text = _strip_comments(text)

    # Extract project name
    project_calls = _extract_call(text, "project")
    project_name = _tokenize(project_calls[0])[0] if project_calls else cmake_path.parent.name

    # Extract set() variables and add PROJECT_NAME
    variables = _extract_variables(text)
    variables["PROJECT_NAME"] = [project_name]

    info = CMakeInfo(package_name=project_name)

    # ------------------------------------------------------------------
    # add_executable / ament_auto_add_executable(name src1 [src2 ...])
    # ------------------------------------------------------------------
    for func in ("add_executable", "ament_auto_add_executable"):
        for args_text in _extract_call(text, func):
            tokens = _resolve_variables(_tokenize(args_text), variables)
            if len(tokens) >= 2:
                name = tokens[0]
                src = tokens[1]
                info.executables[name] = src

    # ------------------------------------------------------------------
    # add_library / ament_auto_add_library(name [SHARED|STATIC|MODULE] src1 [src2 ...])
    # ------------------------------------------------------------------
    lib_types = {"SHARED", "STATIC", "MODULE", "OBJECT", "INTERFACE"}
    for func in ("add_library", "ament_auto_add_library"):
        for args_text in _extract_call(text, func):
            tokens = _resolve_variables(_tokenize(args_text), variables)
            if not tokens:
                continue
            name = tokens[0]
            lib_type = ""
            sources: list[str] = []
            for t in tokens[1:]:
                if t.upper() in lib_types:
                    lib_type = t.upper()
                else:
                    sources.append(t)
            info.libraries[name] = LibraryTarget(name=name, type=lib_type, sources=sources)

    # ------------------------------------------------------------------
    # ament_target_dependencies(target dep1 dep2 ...)
    # ------------------------------------------------------------------
    for args_text in _extract_call(text, "ament_target_dependencies"):
        tokens = _resolve_variables(_tokenize(args_text), variables)
        if len(tokens) >= 2:
            target = tokens[0]
            deps = tokens[1:]
            existing = info.dependencies.get(target, [])
            info.dependencies[target] = existing + deps

    # ------------------------------------------------------------------
    # target_compile_definitions(target [PRIVATE|PUBLIC|INTERFACE] X Y=1 ...)
    # ------------------------------------------------------------------
    scopes = {"PRIVATE", "PUBLIC", "INTERFACE"}
    for args_text in _extract_call(text, "target_compile_definitions"):
        tokens = _resolve_variables(_tokenize(args_text), variables)
        if len(tokens) >= 2:
            defs = [f"-D{t}" for t in tokens[1:] if t not in scopes]
            info.target_definitions.setdefault(tokens[0], []).extend(defs)

    # ------------------------------------------------------------------
    # rclcpp_components_register_nodes(target "pkg::Class" ...)
    # ------------------------------------------------------------------
    for args_text in _extract_call(text, "rclcpp_components_register_nodes"):
        tokens = _resolve_variables(_tokenize(args_text), variables)
        if len(tokens) >= 2:
            target = tokens[0]
            plugins = tokens[1:]
            if target in info.libraries:
                info.libraries[target].component_plugins.extend(plugins)

    # ------------------------------------------------------------------
    # rclcpp_components_register_node(target PLUGIN "pkg::Class"
    #                                 EXECUTABLE exec_name [EXECUTOR ...])
    # ------------------------------------------------------------------
    for args_text in _extract_call(text, "rclcpp_components_register_node"):
        tokens = _resolve_variables(_tokenize(args_text), variables)
        if len(tokens) < 2:
            continue
        target = tokens[0]
        plugin = ""
        executable = ""
        keyword_args = {"PLUGIN", "EXECUTABLE", "EXECUTOR"}
        i = 1
        while i < len(tokens):
            if tokens[i] == "PLUGIN" and i + 1 < len(tokens):
                plugin = tokens[i + 1]
                i += 2
            elif tokens[i] == "EXECUTABLE" and i + 1 < len(tokens):
                executable = tokens[i + 1]
                i += 2
            elif tokens[i] in keyword_args:
                i += 2  # skip keyword + value
            else:
                i += 1
        if plugin and target in info.libraries:
            info.libraries[target].component_plugins.append(plugin)
        if executable and plugin:
            # The EXECUTABLE keyword creates a thin wrapper that delegates
            # to the component.  Register it so node resolution can find it.
            info.executables[executable] = plugin

    # ------------------------------------------------------------------
    # rosidl_generate_interfaces(${PROJECT_NAME} "msg/X.msg" ... DEPENDENCIES ...)
    # ------------------------------------------------------------------
    for args_text in _extract_call(text, "rosidl_generate_interfaces"):
        tokens = _resolve_variables(_tokenize(args_text), variables)
        interfaces: list[str] = []
        skip_rest = False
        for t in tokens[1:]:
            if t == "DEPENDENCIES":
                skip_rest = True
            if skip_rest:
                continue
            if any(t.endswith(ext) for ext in (".msg", ".srv", ".action")):
                interfaces.append(t)
        info.interfaces = interfaces

    return info
