# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Hierarchical graph assembly — orchestrate all extraction layers into a SystemGraph.

Pipeline:
    launch file  ->  parse_launch()   ->  LaunchGraph
    per node     ->  cmake parse()    ->  CMakeInfo (executable -> source mapping)
                 ->  parse_node()     ->  NodeInfo
    all nodes    ->  build_communication()  ->  SystemCommunication
    YAML configs ->  yaml.safe_load() ->  parameter_files dict

    => SystemGraph
"""

from __future__ import annotations

import copy
import logging
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml

from colcon_lf.comm_analysis.topic_mapper import build_communication
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchGraph, LaunchNodeAction
from colcon_lf.launch_analysis.parser import parse_launch
from colcon_lf.launch_analysis.ros_arguments import launch_identity
from colcon_lf.node_analysis.cmake_parser import parse as parse_cmake
from colcon_lf.node_analysis.clang_parser import UnresolvedIncludeError, parse_node
from colcon_lf.node_analysis.models import CMakeInfo, LibraryTarget, NodeInfo
from colcon_lf.workspace import find_package_dirs

_LOG = logging.getLogger(__name__)


# package.xml build types whose sources the analyzer parses from CMakeLists.txt.
_CMAKE_BUILD_TYPES = frozenset({"ament_cmake", "cmake"})


def _build_type(pkg_dir: Path) -> str | None:
    """Return ``<export><build_type>`` of *pkg_dir*'s package.xml, or None."""
    try:
        root = ET.parse(pkg_dir / "package.xml").getroot()
    except (OSError, ET.ParseError):
        return None
    node = root.find("export/build_type")
    if node is None or not (node.text or "").strip():
        return None
    return node.text.strip()


class SkippedNodesError(Exception):
    """Launched nodes the analyzer could not turn into a NodeInfo.

    Not a RuntimeError, so no caller mistakes it for a skippable parse error.
    ``reasons`` holds one message per node, naming the node and its source.
    """

    def __init__(self, reasons: list[str]) -> None:
        self.reasons = reasons
        super().__init__(
            f"{len(reasons)} launched node(s) could not be analyzed and would "
            f"be missing from system.json: " + "; ".join(reasons)
        )


def build_system_graph(
    launch_file: str | Path,
    workspace_root: str | Path | None = None,
    launch_graph_json: str | Path | None = None,
    not_analyzed: list[str] | None = None,
) -> SystemGraph:
    """Build a unified :class:`SystemGraph` from a root launch file.

    Parameters
    ----------
    launch_file:
        Absolute path to the root ``.launch.py`` file.
    workspace_root:
        Colcon workspace root (contains ``src/``, ``build/``, ``install/``).
        Inferred automatically if *None*.
    not_analyzed:
        If given, receives one message per launched node that is outside the
        analysis by design (its package is not in ``src/``, or has no
        CMakeLists.txt), so the caller can report it.  Such a node is left out
        of the graph without failing it.  A node of a C++ package in ``src/``
        that cannot be resolved or parsed raises :class:`SkippedNodesError`.
    """
    launch_file = Path(launch_file).resolve()

    if workspace_root is None:
        workspace_root = _infer_workspace_root(launch_file)
    workspace_root = Path(workspace_root)

    src_root = workspace_root / "src"

    # --- Step 0: Build package directory map ---
    pkg_dirs = find_package_dirs(src_root)

    # --- Step 1: Parse launch hierarchy (or load from pre-computed JSON) ---
    if launch_graph_json is not None and Path(launch_graph_json).exists():
        from colcon_lf.output.serializer import deserialize_launch_graph
        launch_graph = deserialize_launch_graph(Path(launch_graph_json).read_text())
    else:
        launch_graph = parse_launch(launch_file, workspace_root)

    # --- Step 2: Resolve each node to C++ source and parse ---
    cmake_cache: dict[str, CMakeInfo] = {}
    nodes: dict[str, NodeInfo] = {}
    node_pairs: list[tuple[LaunchNodeAction | None, NodeInfo]] = []
    node_file_keys: list[list[str]] = []  # parallel to node_pairs
    parameter_files: dict[str, dict[str, Any]] = {}

    # Launched nodes the analyzer cannot resolve or parse. Each one would be
    # missing from system.json, so they fail the analysis together instead
    # of being skipped with a log line nobody sees.
    skipped: list[str] = []

    for launch_node in launch_graph.all_nodes:
        pkg = launch_node.package
        executable = launch_node.executable
        who = f"node {launch_node.name or executable!r}"

        # A node of a prebuilt package (rviz2, robot_state_publisher, a
        # component container) or of a Python package has no C++ source in
        # the workspace to analyze. It is reported, not failed, so that a
        # workspace such as Nav2 whose launch files also start these nodes
        # still analyzes.
        if pkg not in pkg_dirs:
            if not_analyzed is not None:
                not_analyzed.append(
                    f"{who} (package {pkg!r}, executable {executable!r}) not "
                    f"analyzed: package {pkg!r} is not under {src_root}"
                )
            continue
        cmakelists = pkg_dirs[pkg] / "CMakeLists.txt"
        if not cmakelists.exists():
            # The package.xml build type decides, not the missing file: a
            # CMake package without its CMakeLists.txt is broken, and its
            # node must not pass as a Python one.
            build_type = _build_type(pkg_dirs[pkg])
            if build_type in _CMAKE_BUILD_TYPES:
                skipped.append(
                    f"{who} (package {pkg!r}, executable {executable!r}): "
                    f"package {pkg!r} has build type {build_type!r} but no "
                    f"{cmakelists}, so the executable cannot be resolved to "
                    f"a source file"
                )
            elif not_analyzed is not None:
                kind = (
                    f"has build type {build_type!r}" if build_type
                    else "declares no build type"
                )
                not_analyzed.append(
                    f"{who} (package {pkg!r}, executable {executable!r}) not "
                    f"analyzed: package {pkg!r} {kind}, so it is not a C++ "
                    f"package"
                )
            continue

        # Parse CMake for this package (cached)
        cmake_info = _get_cmake_info(pkg, pkg_dirs, cmake_cache)

        # Resolve executable to source file
        cpp_rel = _resolve_source(cmake_info, executable)
        if cpp_rel is None:
            skipped.append(
                f"{who} (package {pkg!r}, executable {executable!r}): the "
                f"executable does not resolve to a source file in the "
                f"package's CMakeLists.txt"
            )
            continue

        # Use actual package directory (handles nested mono-repos like Nav2)
        pkg_dir = pkg_dirs.get(pkg, src_root / pkg)
        cpp_path = pkg_dir / cpp_rel

        if not cpp_path.exists():
            skipped.append(f"{who} (source {cpp_path}): source file not found")
            continue

        # Parse C++ node. Only these two error types are expected here, any
        # other exception (e.g. NodeSourceError) propagates.
        try:
            node_info = parse_node(
                cpp_path, workspace_root, target=executable,
                extra_defines=cmake_info.target_definitions.get(executable, []),
            )
        except UnresolvedIncludeError as exc:
            # A missing header hid the node class.
            skipped.append(str(exc.for_node(launch_node.name or executable)))
            continue
        except RuntimeError as exc:
            skipped.append(f"{who} (source {cpp_path}): {exc}")
            continue

        # parse_node() is cached by source path (`_parse_node_cache`), so two
        # launch actions of the same executable/class get back the *same*
        # NodeInfo object. The per-instance overlay below mutates
        # ParameterInfo.default_value in place, so without a copy here, two
        # instances of one node class would silently share (and clobber)
        # each other's resolved parameter values (found instancing
        # two `parameter_node`s under the P2 fix: last-processed instance's
        # values leaked into the other's).
        node_info = copy.deepcopy(node_info)

        # Use a qualified node name: namespace/node_name or just executable
        node_key = _node_key(launch_node, node_info)
        nodes[node_key] = node_info
        node_pairs.append((launch_node, node_info))

        # Collect YAML parameter files — from explicit parameter_files list
        # and from inline parameters dict (Nav2 passes files via parameters=).
        # Key by resolved path, workspace-relative when possible (not
        # basename): two nodes can each pass a same-named params.yaml from
        # different directories, and each must keep its own content. An
        # absolute key is unsafe: `output/writer.py`'s `config_dir / filename` silently
        # discards `config_dir` for an absolute `filename`, so `colcon
        # lf-analyze` overwrote the real source params.yaml files in place.
        # The node remembers ITS OWN keys, in launch order: a file is applied
        # only to the nodes it was passed to (a `/**` section
        # in node C's file must not reach node A).
        own_keys = node_parameter_file_keys(launch_node, workspace_root)
        node_file_keys.append(own_keys)
        own_paths = _node_parameter_file_paths(launch_node) if launch_node is not None else []
        for pf_key, pf_path in zip(own_keys, own_paths):
            if pf_key not in parameter_files:
                try:
                    with open(pf_path) as f:
                        parameter_files[pf_key] = yaml.safe_load(f) or {}
                except Exception:
                    _LOG.warning("Failed to parse YAML parameter file: %s", pf_path)

    if skipped:
        raise SkippedNodesError(skipped)

    # --- Step 2b: Apply launch `parameters=[...]` (inline dicts and files) ---
    # In launch order with launch_ros' keying and rcl's section order,
    # so system.json predicts what `ros2 launch` gives the node.
    _apply_launch_parameters(node_pairs, [
        _own_file_contents(launch_node, keys, parameter_files)
        for (launch_node, _), keys in zip(node_pairs, node_file_keys)
    ])

    # --- Step 2c: Resolve ${param} placeholders in topic/service names ---
    _resolve_topic_placeholders(node_pairs)

    # --- Step 3: Build communication map ---
    # Endpoints by node key, so two instances of one class stay apart.
    communication = build_communication(
        node_pairs, workspace_root=workspace_root,
        node_keys=[_node_key(ln, ni) for ln, ni in node_pairs],
    )

    # --- Step 4: Assemble SystemGraph ---
    name = launch_file.stem.replace(".launch", "")

    return SystemGraph(
        name=name,
        launch_graph=launch_graph,
        nodes=nodes,
        communication=communication,
        parameter_files=parameter_files,
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parameter_file_key(pf_path: Path, workspace_root: Path) -> str:
    """Collision-free ``SystemGraph.parameter_files`` key for *pf_path*.

    Workspace-relative POSIX path when *pf_path* is inside *workspace_root*
    (the common case, and the only one exercised by any in-tree example);
    the resolved absolute path otherwise (maintainer decision P2,
    corrected after the writer overwrite bug — see the call site). Two
    different files that happen to share a basename still get distinct keys
    as long as their directories differ, which is the actual collision this
    replaces (a plain basename) fixes.
    """
    resolved = pf_path.resolve()
    try:
        return resolved.relative_to(workspace_root.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def _node_parameter_file_paths(launch_node) -> list[Path]:
    """Existing parameter files passed to *launch_node*, in launch order, deduplicated.

    Only file entries of ``parameters=``; an inline string value that names a
    .yaml file is a string parameter, which launch_ros never loads.
    """
    paths: list[Path] = [Path(pf) for pf in launch_node.parameter_files]
    out: list[Path] = []
    seen: set[Path] = set()
    for pf in paths:
        if pf.exists() and pf.resolve() not in seen:
            seen.add(pf.resolve())
            out.append(pf)
    return out


def node_parameter_file_keys(launch_node, workspace_root) -> list[str]:
    """``SystemGraph.parameter_files`` keys of the files passed to *launch_node*.

    Shared by the analyzer overlay and the resolver so both apply
    a file only to the nodes it was given to.
    """
    if launch_node is None:
        return []
    root = Path(workspace_root) if workspace_root is not None else Path.cwd()
    return [_parameter_file_key(pf, root) for pf in _node_parameter_file_paths(launch_node)]


INLINE_PARAMETERS_ROLE = "launch inline parameters"


def _own_file_contents(launch_node, own_keys, parameter_files) -> dict[str, tuple[str, Any]]:
    """``{resolved path: (parameter_files key, content)}`` of *launch_node*'s own files."""
    if launch_node is None:
        return {}
    return {
        pf.resolve().as_posix(): (key, parameter_files.get(key))
        for pf, key in zip(_node_parameter_file_paths(launch_node), own_keys)
    }


def _node_key_matches(section: str, fqn: str) -> bool:
    """Does the parameter-file node key *section* select the node *fqn*?

    rclcpp and launch_ros both turn ``/*`` into one name token and ``/**``
    into any number of them, then require a full match. A key without a
    leading slash is relative to the root, as rcl reads it.
    """
    if not section.startswith("/"):
        section = "/" + section
    pattern, i = [], 0
    while i < len(section):
        if section.startswith("/*", i):
            pattern.append(r"(/\w+)")
            i += 2
        elif section[i] == "*":
            pattern.append("*")
            i += 1
        else:
            pattern.append(re.escape(section[i]))
            i += 1
    try:
        return re.fullmatch("".join(pattern), fqn) is not None
    except re.error:
        return False


def _flatten_parameters(params: dict, prefix: str = "") -> dict[str, Any]:
    """Nested parameter dicts as dotted names (``{a: {b: 1}}`` -> ``a.b``), as rcl
    and launch_ros read them."""
    out: dict[str, Any] = {}
    for key, value in params.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            out.update(_flatten_parameters(value, name + "."))
        else:
            out[name] = value
    return out


def _file_sections(content: Any, strip: bool) -> list[tuple[str, dict[str, Any]]]:
    """``(node key, flat values)`` per ``ros__parameters`` block, in file order.

    Keys nested above ``ros__parameters`` are joined with ``/``
    (``ns: {node: ...}`` is ``ns/node``). rcl keeps each key as written;
    launch_ros (*strip*) drops every leading ``/``.
    """
    out: list[tuple[str, dict[str, Any]]] = []

    def walk(body: dict, keys: list[str]) -> None:
        for key, value in body.items():
            if key == "ros__parameters":
                if isinstance(value, dict) and keys:
                    out.append(("/".join(keys), _flatten_parameters(value)))
                return
            if isinstance(key, str) and isinstance(value, dict):
                walk(value, keys + [key.lstrip("/") if strip or keys else key])

    if isinstance(content, dict):
        walk(content, [])
    return out


def _effective_fqn(launch_node: LaunchNodeAction, node_info: NodeInfo) -> str:
    """The full name rcl matches parameter sections against (``__node`` remaps)."""
    identity = launch_identity(launch_node, node_info.name)
    ns = (identity.namespace or "").strip("/")
    return f"/{ns}/{identity.name}" if ns else f"/{identity.name}"


def launch_parameter_values(
    launch_node: LaunchNodeAction, node_info: NodeInfo, files: dict[str, tuple[str, Any]],
) -> dict[str, tuple[Any, str]]:
    """The launch ``parameters=`` values *launch_node* starts with.

    *files* maps each of the node's own parameter files, by resolved path, to
    ``(role, content)``. Returns ``{name: (value, role)}``, where role is the
    file's role or ``INLINE_PARAMETERS_ROLE``; a later source wins.

    The sources are walked in launch order (``parameter_sources``), as
    ``launch_ros`` passes them:

    - A ``Node`` gets every source as a ``--params-file``, an inline dict
      keyed by its full name when the launch file gives a name and a
      namespace and by ``/**`` otherwise (the key the launch parser records).
      rcl groups all values by node key in the order a key first appears,
      and applies the matching groups in that order. A node-keyed file
      section therefore beats a ``/**`` inline dict created before it, even
      when the dict comes later in the list.
    - A composable node's sources are merged in list order into its load
      request (``to_parameters_list``): a file contributes its sections that
      match the node, an inline dict its values.

    Nested parameter dicts count as dotted names and nested node keys as
    namespaces, in files and inline dicts alike. A ``Node`` matches the
    sections against its effective full name, after ``__node``/``__ns``
    remaps; a composable node against its launch name, as launch_ros does.

    Without ``parameter_sources`` (a launch model from an older analyzer)
    inline values come first and the files after them.
    """
    component = launch_node.is_component
    if component:
        ns = (launch_node.namespace or "").strip("/")
        name = launch_node.name or ""
        # launch_ros applies no file section to a component without a name.
        fqn = (f"/{ns}/{name}" if ns else f"/{name}") if name else None
    else:
        fqn = _effective_fqn(launch_node, node_info)
    sections: dict[str, dict[str, tuple[Any, str]]] = {}
    merged: dict[str, tuple[Any, str]] = {}

    def add_file(role: str, content: Any) -> None:
        if not component:
            # rcl keeps a section per key as written ("node" and "/node" are
            # two), in the order each key first appears.
            for key, values in _file_sections(content, strip=False):
                sections.setdefault(key, {}).update({k: (v, role) for k, v in values.items()})
            return
        if fqn is None:
            return
        found = _file_sections(content, strip=True)
        for key, values in found:
            if _node_key_matches(key, fqn):
                merged.update({k: (v, role) for k, v in values.items()})
        # launch_ros applies a component's exact node section once more,
        # after the file's wildcard sections (to_parameters_list).
        for key, values in found:
            if "/" + key == fqn:
                merged.update({k: (v, role) for k, v in values.items()})

    def add_inline(key: str, params: dict) -> None:
        values = {k: (v, INLINE_PARAMETERS_ROLE) for k, v in _flatten_parameters(params).items()}
        if component:
            merged.update(values)
        else:
            sections.setdefault(key, {}).update(values)

    def add_path(path: Any) -> None:
        entry = files.get(Path(str(path)).resolve().as_posix())
        if entry is not None:
            add_file(*entry)

    if not component:
        # -p rules in arguments= come before parameters=.
        arg_params = dict(launch_identity(launch_node, node_info.name).params)
        if arg_params:
            add_inline("/**", arg_params)
    sources = launch_node.parameter_sources
    if not sources:
        if launch_node.parameters:
            add_inline("/**", launch_node.parameters)
        for role, content in files.values():
            add_file(role, content)
    for source in sources:
        if "file" in source:
            add_path(source["file"])
        elif source.get("params"):
            # A string value ending in .yaml is a string parameter, not
            # another file source: launch_ros never loads it.
            add_inline(source.get("key", "/**"), source["params"])
    if component:
        return merged
    out: dict[str, tuple[Any, str]] = {}
    for section, values in sections.items():
        if _node_key_matches(section, fqn):
            out.update(values)
    return out


def _apply_launch_parameters(
    node_pairs: list[tuple[LaunchNodeAction | None, NodeInfo]],
    node_files: list[dict[str, tuple[str, Any]]],
) -> None:
    """Overlay each node's launch ``parameters=`` values onto its NodeInfo in place.

    A launch-time value overrides the declared C++ default (ROS parameter
    precedence). A parameter found only at launch time is appended
    with its type inferred from the value.
    """
    from colcon_lf.node_analysis.models import ParameterInfo

    for (launch_node, node_info), files in zip(node_pairs, node_files):
        if launch_node is None:
            continue
        param_index = {p.name: p for p in node_info.parameters}
        for key, (value, _role) in launch_parameter_values(launch_node, node_info, files).items():
            if key in param_index:
                param_index[key].default_value = value
            else:
                node_info.parameters.append(
                    ParameterInfo(name=key, type=_infer_param_type(value), default_value=value)
                )


def _resolve_topic_placeholders(
    node_pairs: list[tuple[LaunchNodeAction | None, NodeInfo]],
) -> None:
    """Resolve remaining ``${param}`` placeholders in topic/service/action names.

    After YAML parameter overlay, node parameters may have values that can
    fill in unresolved ``${variable}`` placeholders in resource names.
    """
    placeholder_re = re.compile(r"\$\{(\w+)\}")

    for _launch_node, node_info in node_pairs:
        # Build param lookup from node's parameters
        param_map: dict[str, str] = {}
        for p in node_info.parameters:
            if p.default_value is not None and isinstance(p.default_value, str):
                param_map[p.name] = p.default_value

        # A topic held in a member the constructor set from a
        # declared parameter is that parameter's value after the launch
        # overlay, the same value the node reads when it is constructed.
        for ep in [*node_info.publishers, *node_info.subscribers]:
            value = param_map.get(ep.topic_param or "")
            if ep.topic_param and not ep.topic and value and "${" not in value:
                ep.topic = value
                ep.topic_dynamic = False

        if not param_map:
            continue

        def _resolve(name: str) -> str:
            m = placeholder_re.match(name)
            if m and m.group(0) == name:  # Full match only
                var = m.group(1)
                if var in param_map:
                    return param_map[var]
            return name

        # A resolved name is a known topic, so drop the parser's topic_dynamic
        # flag too (RCLLF-H004 and the resolved model read it).
        for ep in [*node_info.publishers, *node_info.subscribers]:
            resolved = _resolve(ep.topic)
            if resolved != ep.topic:
                ep.topic = resolved
                # Only a non-empty, placeholder-free value is a known topic.
                if resolved and "${" not in resolved:
                    ep.topic_dynamic = False
        for srv in node_info.service_servers:
            srv.service_name = _resolve(srv.service_name)
        for cli in node_info.service_clients:
            cli.service_name = _resolve(cli.service_name)
        for srv in node_info.action_servers:
            srv.action_name = _resolve(srv.action_name)
        for cli in node_info.action_clients:
            cli.action_name = _resolve(cli.action_name)


def _infer_param_type(value: Any) -> str:
    """Infer a ROS2 parameter type string from a Python value."""
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "double"
    if isinstance(value, str):
        return "string"
    if isinstance(value, list):
        return "array"
    return ""


def _infer_workspace_root(path: Path) -> Path:
    """Walk up from *path* looking for a directory that contains ``src/``."""
    candidate = path.parent
    for _ in range(10):
        if (candidate / "src").is_dir():
            return candidate
        candidate = candidate.parent
    return path.parent




def _get_cmake_info(
    package: str,
    pkg_dirs: dict[str, Path],
    cache: dict[str, CMakeInfo],
) -> CMakeInfo | None:
    """Return CMakeInfo for *package*, caching results."""
    if package in cache:
        return cache[package]

    pkg_dir = pkg_dirs.get(package)
    if pkg_dir is None:
        cache[package] = None  # type: ignore[assignment]
        return None

    cmake_path = pkg_dir / "CMakeLists.txt"
    if not cmake_path.exists():
        cache[package] = None  # type: ignore[assignment]
        return None

    info = parse_cmake(cmake_path)
    cache[package] = info
    return info


def _resolve_source(cmake_info: CMakeInfo, executable: str) -> str | None:
    """Map an executable (or component plugin) to a relative source file path.

    Checks executables first, then component libraries.  For Nav2-style
    packages where the executable is a thin ``main.cpp`` wrapper and the
    actual node implementation lives in a companion ``_core`` library,
    the library source is preferred.
    """
    # Direct executable match
    if executable in cmake_info.executables:
        value = cmake_info.executables[executable]
        # Check for a companion library with the actual implementation
        companion = _find_companion_library(cmake_info, executable)
        if companion and companion.sources:
            return companion.sources[0]
        # rclcpp_components_register_node(...EXECUTABLE exec) stores the plugin
        # class name (e.g. "FrontLidarDriver"), not a source path.  Fall through
        # to the component-library search so the class→snake_case→source lookup
        # below can handle it.
        if value.endswith((".cpp", ".cc", ".cxx")):
            return value

    # Component: executable might be the plugin class name (e.g. "component_example::ComponentNode")
    # or the library target name.  Check libraries for matching component_plugins.
    # Also handles the rclcpp_components_register_node EXECUTABLE case where
    # cmake_info.executables[exec] stores the plugin class name as `value`.
    _exec_alias = cmake_info.executables.get(executable, "")
    for lib in cmake_info.libraries.values():
        for plugin in lib.component_plugins:
            if (
                executable == plugin
                or executable.endswith("::" + plugin.split("::")[-1])
                or _exec_alias == plugin
                or (_exec_alias and _exec_alias.endswith("::" + plugin.split("::")[-1]))
            ):
                # Try to match source file by: (1) executable name, (2) class→snake_case
                class_name = plugin.split("::")[-1]
                snake = _camel_to_snake(class_name)
                for src in lib.sources:
                    if executable in src or snake in src:
                        return src
                # Fallback: return first source
                if lib.sources:
                    return lib.sources[0]

    # Fallback: check if executable matches a library target name
    if executable in cmake_info.libraries:
        lib = cmake_info.libraries[executable]
        if lib.sources:
            return lib.sources[0]

    return None


def _find_companion_library(
    cmake_info: CMakeInfo, executable: str,
) -> LibraryTarget | None:
    """Find a companion library containing the node implementation.

    Nav2-style packages use a thin ``main.cpp`` executable linked to a
    ``{executable}_core`` library with the actual node code.
    """
    # {executable}_core pattern (Nav2 convention)
    core_name = f"{executable}_core"
    if core_name in cmake_info.libraries:
        return cmake_info.libraries[core_name]

    # Any library whose name starts with the executable name
    for name, lib in cmake_info.libraries.items():
        if name.startswith(executable) and lib.sources:
            return lib

    return None


def _camel_to_snake(name: str) -> str:
    """Convert CamelCase to snake_case (e.g. TimerPublisherNode -> timer_publisher_node)."""
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def _node_key(launch_node: LaunchNodeAction, node_info: NodeInfo) -> str:
    """Build a qualified key for a node in the nodes dict.

    Uses the name and namespace the node runs with (``launch_identity``: an
    ``arguments=`` remap, else the launch override, else the name in the C++
    code), so the key is the node's name in ``ros2 node list``.
    """
    identity = launch_identity(launch_node, node_info.name)
    name, ns = identity.name, identity.namespace
    if ns:
        return f"{ns}/{name}" if not ns.endswith("/") else f"{ns}{name}"
    return name
