# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Colcon verb: ``colcon lf-generate`` — generate LF code from system.json.

Usage::

    # Generate from a specific system.json
    colcon lf-generate --system-json lf-analysis/pubsub_example/bringup/system.json

    # Generate from every analysis of one package (all launch stems)
    colcon lf-generate --package pubsub_example

    # Generate from all system.json files in the workspace analysis output
    colcon lf-generate

    # Custom output directory
    colcon lf-generate --system-json path/to/system.json --output-dir /tmp/lf
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint

from colcon_lf.comm_analysis.endpoints import AmbiguousEndpointError, endpoint_key_map
from colcon_lf.generation.factory_emitter import (
    emit_empty_node_factories_cpp,
    emit_headerless_factory_suffix,
    emit_node_factories_cpp,
)
from colcon_lf.node_analysis.models import is_lifecycle_base
from colcon_lf.verb.workspace import resolve_workspace as _resolve_workspace

# Extra includes prepended to a headerless node's staged source so the
# `if constexpr` factory appended at its end compiles. node_registry.h is
# needed because the appended factory now registers the node itself
# (node_factories.cpp cannot name a headerless class to register it).
_HEADERLESS_FACTORY_INCLUDES = (
    "#include <memory>\n"
    "#include <mutex>\n"
    "#include <type_traits>\n"
    '#include "ros2_lf_binding/node_registry.h"\n'
)


class SourceStagingError(RuntimeError):
    """A non-component node's source file could not be staged.

    Raised instead of writing a placeholder .cpp.  The staged copy is what
    supplies the node class's constructor to the generated wrapper, so a
    placeholder yields a package that *compiles* (the class is declared by the
    exported header) but fails to *link*, with an ``undefined reference to
    <Class>::<Class>(...)`` error in a different package minutes later.  Failing
    here keeps the diagnosis at the point where the missing input is known.
    """


def _stage_node_source_or_fail(src_file: str, ni) -> str:
    """Return the node's source text, or raise SourceStagingError.

    The usual cause is running generation where the extracted absolute paths do
    not resolve — e.g. a ``system.json`` produced inside the dev container
    (``/workspace/...``) but generated from the host.
    """
    if src_file and Path(src_file).is_file():
        return Path(src_file).read_text(encoding="utf-8")
    raise SourceStagingError(
        f"Cannot stage source for node class {ni.class_name!r} "
        f"(package {ni.package or 'unknown'!r}): "
        f"source file {src_file or '<none recorded>'!r} does not exist.\n"
        "The generated package would compile but fail to link with "
        f"'undefined reference to {ni.class_name}::{ni.class_name}(...)'.\n"
        "If system.json was produced inside the dev container, its paths are "
        "container paths — re-run extraction and generation in the same "
        "environment (e.g. 'docker compose run --rm dev', then 'colcon lf')."
    )


def _staged_define_lines(ni) -> str:
    """``#define`` lines for the executable target's ``-D`` definitions.

    The staged copy of a node source is compiled by the generated package, not
    by the user's target, so the user's ``target_compile_definitions`` (the
    ``#ifdef`` main selector) must be restated in the copy itself.
    """
    lines = []
    for item in getattr(ni, "compile_defines", None) or []:
        body = item[2:] if item.startswith("-D") else item
        name, _, value = body.partition("=")
        if name:
            lines.append(f"#define {name} {value or '1'}\n")
    return "".join(lines)


def _staged_node_dest(pkg_nodes_dir: Path, src_file: str, ni, staged_from: dict) -> Path:
    """Staged path of a non-component node's source in *pkg_nodes_dir*.

    Instances of one class (one source file) share one staged file, which then
    carries one factory per instance (probe P17g). Two different sources with
    one basename in one package (e.g. two ``main.cpp``) get distinct names, so
    neither overwrites the other. *staged_from* maps each staged path to the
    source it holds and is updated here.
    """
    origin = src_file or ni.class_name
    basename = Path(src_file).name if src_file else f"{ni.class_name.lower()}.cpp"
    dest = pkg_nodes_dir / basename
    n = 1
    while staged_from.get(dest, origin) != origin:
        dest = pkg_nodes_dir / f"{Path(basename).stem}_{n}{Path(basename).suffix}"
        n += 1
    staged_from[dest] = origin
    return dest


def _factory_entries(
    non_component_items,
    binding_keys: dict,
    identities: dict,
    auto_transition: dict | None = None,
) -> list[dict]:
    """Build the ``emit_node_factories_cpp`` entry list for a node set.

    ``non_component_items`` is ``[(node_key, NodeInfo), ...]`` in staging order
    (the index is the ``ros2_lf_make_node_{idx}`` suffix used for headerless
    nodes, so it must match the staging loop's enumeration).

    ``auto_transition`` maps node key to the resolved lifecycle auto-transition
    flag (``_resolve_auto_transition``); a missing key means the default True,
    so an absent map leaves every node on the behaviour from before this setting existed.
    """
    auto_transition = auto_transition or {}
    entries: list[dict] = []
    for idx, (node_key, ni) in enumerate(non_component_items):
        header = None
        if ni.package and ni.class_name and ni.header_file:
            header = f"{ni.package}/{Path(str(ni.header_file)).name}"
        identity = identities.get(node_key)
        entries.append({
            "binding_key": binding_keys[node_key],
            "class_name": ni.class_name,
            "qualified_name": getattr(ni, "qualified_name", "") or "",
            "package": ni.package or "",
            "header": header,
            "idx": idx,
            "node_name": identity.node_name if identity else ni.name,
            "namespace": identity.namespace if identity else "",
            "remappings": [list(r) for r in identity.remappings] if identity else [],
            "param_args": list(identity.param_args) if identity else [],
            "base_class": ni.base_class,
            "auto_transition": auto_transition.get(node_key, True),
            # Typed cleanup on deactivate, lifecycle nodes only.
            "service_client_types": [
                c.service_type for c in ni.service_clients
            ] if ni.is_lifecycle else [],
            "action_client_types": [
                c.action_type for c in ni.action_clients
            ] if ni.is_lifecycle else [],
        })
    return entries


# Top-level system.json key carrying the per-instance component list.
# Additive on the existing schema (v3 is already gated fail-closed), consumed by
# the runtime's parse_component_list (runtime.cpp reads doc.at("component_instances")).
# Design note section 4 (component path) — keep this string in sync with the C++ side.
COMPONENT_INSTANCES_KEY = "component_instances"

# Top-level key of a federate's system.json: every topic with
# endpoints both in this federate and outside it, with its system-wide
# publisher and subscriber node counts. Read by match_warmup.cpp.
CROSS_FEDERATE_TOPICS_KEY = "cross_federate_topics"


def _endpoints_before_tag0(node: dict | None, kind: str, topic: str, listed: int) -> int:
    """How many of a node's *listed* *kind* endpoints on *topic* exist before tag 0.

    *listed* is how often the node's key appears in the topic's endpoint list
    of system.json, which holds one entry per endpoint (topic_mapper.py
    build_communication appends once per publisher or subscriber), so a node
    with two readers on one topic is listed twice. A lifecycle node's endpoint
    made in a hook has no DDS entity before tag 0, so the peer check skips it
     and the warm-up must not wait for it. When the node's own endpoint
    infos for *topic* do not account for every listed entry (a remap the
    analysis resolved differently), every listed entry counts as made before
    tag 0: the warm-up then waits for an endpoint that may come late and
    refuses at its bound, which is loud, rather than lifting early.
    """
    if not node or not is_lifecycle_base(
        node.get("base_class", ""), node.get("lifecycle_callbacks"),
    ):
        return listed
    infos = [
        i for i in node.get(kind, [])
        if i.get("topic") and (
            i["topic"] == topic or topic.endswith("/" + i["topic"].lstrip("~/"))
        )
    ]
    if len(infos) != listed:
        return listed
    return sum(i.get("created_in", "constructor") == "constructor" for i in infos)


def cross_federate_topics(
    topics: list, fed_node_set, runtime_to_key: dict, nodes: dict | None = None,
) -> list[dict]:
    """Topics of the whole system.json that cross this federate's boundary.

    A topic crosses when this federate holds at least one of its endpoints and
    another federate holds at least one more. The counts are system-wide, so
    a local writer waits for every reader of the topic, local or remote. They
    count endpoints, not nodes: the endpoint lists hold one entry per endpoint,
    so a node with two readers on one topic counts twice, as DDS counts two
    matched readers. An endpoint a lifecycle node makes in a hook
    is left out (*nodes*, the whole system's node dicts by key).
    """
    nodes = nodes or {}
    out = []
    for t in topics:
        pubs = [runtime_to_key.get(p, p) for p in t.get("publishers", [])]
        subs = [runtime_to_key.get(s, s) for s in t.get("subscribers", [])]
        here = sum(k in fed_node_set for k in pubs + subs)
        if 0 < here < len(pubs) + len(subs):
            out.append({
                "topic": t["topic"],
                "publishers": sum(
                    _endpoints_before_tag0(nodes.get(k), "publishers", t["topic"], pubs.count(k))
                    for k in dict.fromkeys(pubs)),
                "subscribers": sum(
                    _endpoints_before_tag0(nodes.get(k), "subscribers", t["topic"], subs.count(k))
                    for k in dict.fromkeys(subs)),
            })
    return out


def _report_lifecycle_component_reroute(graph) -> None:
    """Announce every managed lifecycle node taken off the component path.

    A lifecycle node registered with
    ``rclcpp_components_register_nodes`` cannot be brought up on the component
    path, because ``LFComponentManager`` never registers it in the node
    registry and so nothing drives ``configure()``/``activate()``. It would be
    bound while still ``unconfigured``, owning none of the entities its
    ``on_configure``/``on_activate`` create, and the only symptom would be a
    ``0 timer(s), 0 subscription(s)`` line at the end of binding.

    ``NodeInfo.takes_component_path`` routes such a node to the non-component
    path instead, which constructs it directly and registers it. That is a
    real change in how the node is built, so it is said out loud rather than
    left for someone to infer from the generated tree.
    """
    # Derived from `takes_component_path`, not from an independent
    # `is_component and is_lifecycle` test, so this cannot announce a re-route
    # that the routing sites did not actually perform. Falsifying the property
    # must break this message too, otherwise the message becomes the thing that
    # hides the regression.
    rerouted = [
        ni for ni in graph.nodes.values() if ni.is_component and not ni.takes_component_path
    ]
    if not rerouted:
        return
    print(
        f"  {len(rerouted)} managed lifecycle node(s) registered as components "
        "will be built as non-component nodes, so their lifecycle can be "
        "driven before binding:"
    )
    for ni in rerouted:
        print(f"    {ni.package or '<unknown package>'}::{ni.class_name}")


def build_component_instances(graph, binding_keys: dict, identities: dict) -> list[dict]:
    """One entry per component node *instance*, in graph order.

    The five fields the design note freezes (``package``, ``plugin_class``,
    ``binding_key``, ``node_name``, ``namespace``). An instance with launch
    remaps also carries ``remap_rules``, the remaps as ``src:=dst`` strings.
    An instance with launch parameters carries ``param_args``,
    the ``-p`` and ``--params-file`` rules.
    ``plugin_class`` stays the real pluginlib class name (``load_component`` must
    receive it verbatim); ``binding_key`` is the generator's instance key that
    ``class_to_id_`` is keyed by, so N instances no longer collapse to one.
    """
    entries: list[dict] = []
    for node_key, ni in graph.nodes.items():
        if not ni.takes_component_path:
            continue
        identity = identities.get(node_key)
        entries.append({
            "package": ni.package or "",
            "plugin_class": ni.class_name,
            "binding_key": binding_keys[node_key],
            "node_name": identity.node_name if identity else ni.name,
            "namespace": identity.namespace if identity else "",
        })
        if identity and identity.remappings:
            # LoadNode remap_rules, the launch remaps as src:=dst.
            entries[-1]["remap_rules"] = [f"{src}:={dst}" for src, dst in identity.remappings]
        if identity and identity.param_args:
            # Launch parameters as --ros-args rules, appended to
            # the LoadNode node options by LFComponentManager.
            entries[-1]["param_args"] = list(identity.param_args)
    return entries


def _stage_component_instances(
    *, system_json: Path, graph, binding_keys: dict, identities: dict,
) -> None:
    """Write the per-instance component list into *system_json* in place.

    The runtime reads the component list straight out of ``system.json``
    (``runtime.cpp``'s ``parse_component_list``), so that is where the list has
    to land.  Additive: no existing key is touched, and the write is a no-op
    when the system has no component nodes and the file has no stale list.
    """
    entries = build_component_instances(graph, binding_keys, identities)
    try:
        raw = json.loads(system_json.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - unreadable system.json
        print(
            f"  Warning: cannot stage component instances into {system_json}: {exc}",
            file=sys.stderr,
        )
        return
    if not entries and COMPONENT_INSTANCES_KEY not in raw:
        return
    raw[COMPONENT_INSTANCES_KEY] = entries
    system_json.write_text(json.dumps(raw, indent=2), encoding="utf-8")


class GenerateVerb(VerbExtensionPoint):
    """Generate Lingua Franca code from extracted ROS2 system models."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            VerbExtensionPoint.EXTENSION_POINT_VERSION, "^1.0"
        )

    def add_arguments(self, *, parser):  # noqa: D102
        from colcon_lf.verb.configure import add_package_argument

        add_package_argument(parser)
        parser.add_argument(
            "--output-dir",
            default=None,
            help=(
                "Output directory for generated .lf files. "
                "Default: lf/ subdirectory next to system.json."
            ),
        )
        parser.add_argument(
            "--workspace",
            default=None,
            help="Colcon workspace root (default: auto-detect).",
        )
        parser.add_argument(
            "--after-delay",
            default=None,
            help=(
                "Default 'after' delay (declared logical-time offset, LET-style) "
                "for inter-node topic connections, e.g. '5 msec'. Overrides "
                "execution.yaml's defaults.after_delay; a per-link after_delay_ns "
                "in system.json still wins. Default: '0' (see the execution-config "
                "docs for what d = 0 means per edge kind)."
            ),
        )
        parser.add_argument(
            "--launch",
            action="store_true",
            default=True,
            dest="launch",
            help="Generate LF launch files (default: on).",
        )
        parser.add_argument(
            "--no-launch",
            action="store_false",
            dest="launch",
            help="Skip launch file and wrapper generation.",
        )
        parser.add_argument(
            "--federate",
            default=None,
            metavar="{per-process,per-node,<path>.yaml}",
            help=(
                "Generate a federated artifact instead of single-process. "
                "Modes: 'per-process' (one federate per launch process_group), "
                "'per-node' (one federate per ROS2 node), "
                "or a path to a federation.yaml file for explicit partition. "
                "Coordination mode defaults to centralized (RTI-mediated, never "
                "tardy); see --coordination for LF's decentralized mode."
            ),
        )
        parser.add_argument(
            "--coordination",
            default=None,
            choices=["centralized", "decentralized"],
            help=(
                "Coordination mode for --federate output. 'centralized' (default): "
                "the RTI mediates every cross-federate tag advance, never tardy. "
                "'decentralized': peer-to-peer, STP-offset based, tardy messages "
                "are possible (LF regime 3, docs/design/time-and-coordination.md). "
                "Only meaningful with --federate. When omitted, a custom "
                "federation.yaml's own 'coordination:' key is used if present, "
                "else centralized."
            ),
        )
        parser.add_argument(
            "--emit-federation-template",
            action="store_true",
            default=False,
            help=(
                "Write a starter federation.yaml next to each system.json "
                "(one federate per node by default). Does not generate code. "
                "Refuses to overwrite an existing file unless --force is also given."
            ),
        )
        parser.add_argument(
            "--force",
            action="store_true",
            default=False,
            help="Allow --emit-federation-template to overwrite an existing federation.yaml.",
        )
        parser.add_argument(
            "--target-lfc",
            action="store_true",
            default=False,
            help=(
                "Deprecated no-op, accepted for one release: "
                "lf-generate always emits the lfc-compilable .lf and the "
                "lf_generated_<name> colcon package."
            ),
        )
        parser.add_argument(
            "--exec-config",
            default=None,
            metavar="<path>",
            help=(
                "Path to an execution.yaml (sparse execution-policy config). "
                "Default: the analysis dir's config/execution.yaml when "
                "present; absent = built-in defaults (today's behavior, zero "
                "migration burden). Malformed file is a hard error, never a "
                "silent fall-through."
            ),
        )
        parser.add_argument(
            "--fast",
            action="store_true",
            default=False,
            help=(
                "Fast mode: the generated launch files start the "
                "program with the reactor-c argument '-f true', so logical time "
                "advances as fast as the reactions allow instead of waiting for "
                "physical time. Outputs stay those of the real-time run; see "
                "docs/features/fast-mode.md. (Not the 'fast' target property: "
                "lfc refuses it in programs with physical actions, which every "
                "rcllf program has.)"
            ),
        )
        parser.add_argument(
            "--timeout",
            default=None,
            metavar="<LF time>",
            help=(
                "Emit 'timeout: <LF time>' (e.g. '20 sec'): the generated program "
                "ends at that logical time. Without it a keepalive program runs "
                "until it is stopped. RCLLF_LF_ARGS='-f true -o 20 sec' at launch "
                "sets both without regeneration."
            ),
        )
        parser.add_argument(
            "--params-file",
            action="append",
            default=[],
            metavar="<path>",
            help=(
                "A ROS2 parameter YAML file to apply at generate time, "
                "postdating analysis — repeatable; later files "
                "take precedence, standing for --ros-args --params-file "
                "appended at run time. Feeds the resolved-model report and "
                "its RCLLF-R001/R002/R003 fail-closed gate."
            ),
        )

    def main(self, *, context) -> int:  # noqa: D102
        args = context.args

        if getattr(args, "target_lfc", False):
            print(
                "Note: --target-lfc is deprecated and has no effect: the lfc "
                "path is the only generation path. It will be "
                "removed in the next release.",
                file=sys.stderr,
            )

        # Validate the run-mode options once, before any output.
        fast = bool(getattr(args, "fast", False))
        timeout = getattr(args, "timeout", None)
        try:
            from colcon_lf.generation.lf_generator import run_mode_target_options
            run_mode_target_options(fast=fast, timeout=timeout)
        except ValueError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1

        if getattr(args, "coordination", None) and not args.federate:
            print(
                "Error: --coordination requires --federate (coordination mode "
                "only applies to federated output).",
                file=sys.stderr,
            )
            return 1

        from colcon_lf.verb.configure import (
            package_scope_error,
            report_package_scope,
        )

        system_jsons = _find_system_jsons(
            args.system_json, args.workspace, args.package
        )
        if not system_jsons:
            if args.package is not None:
                print(
                    package_scope_error(
                        args.package, _resolve_workspace(args.workspace)
                    ),
                    file=sys.stderr,
                )
            else:
                print("Error: No system.json files found.", file=sys.stderr)
            return 1

        report_package_scope(args.package, system_jsons)

        # Template-only mode — no code generation
        if args.emit_federation_template:
            print(f"==> Emitting federation templates for {len(system_jsons)} system(s)\n")
            failures = 0
            for sj in system_jsons:
                print(f"--- {sj} ---")
                try:
                    from colcon_lf.output.serializer import deserialize_system_graph
                    graph = deserialize_system_graph(sj.read_text(encoding="utf-8"))
                    from colcon_lf.federation.template import emit_federation_template
                    rc = emit_federation_template(
                        graph=graph,
                        system_json=sj,
                        force=args.force,
                    )
                except Exception as exc:
                    print(f"  Error: {exc}", file=sys.stderr)
                    rc = 1
                if rc != 0:
                    failures += 1
            if failures:
                print(f"\n==> {failures}/{len(system_jsons)} template(s) failed", file=sys.stderr)
                return 1
            print(f"\n==> All {len(system_jsons)} template(s) written successfully")
            return 0

        workspace = _resolve_workspace(args.workspace)

        # Every generated program compiles against the reactor-c
        # inside lfc's jar. Refuse an unpatched runtime here, before a program
        # whose every run would end "read-back invalid" is generated.
        from colcon_lf.generation.lfc_runtime import check_lfc_runtime, lfc_runtime_error
        runtime = check_lfc_runtime(workspace)
        if runtime["status"] == "failed":
            print(lfc_runtime_error(runtime), file=sys.stderr)
            return 1
        if runtime["status"] == "skipped":
            print(f"Note: lfc runtime not checked: {runtime['reason']}", file=sys.stderr)

        print(f"==> Found {len(system_jsons)} system(s) to generate\n")

        failures = 0
        for sj in system_jsons:
            rc = _generate_recorded(
                system_json=sj,
                output_dir=args.output_dir,
                after_delay=args.after_delay,
                generate_launch=args.launch,
                target_lfc=True,
                federate=args.federate,
                exec_config_arg=args.exec_config,
                params_files=args.params_file,
                workspace=workspace,
                coordination=args.coordination,
                fast=fast,
                timeout=timeout,
            )
            if rc != 0:
                failures += 1

        if failures:
            print(
                f"\n==> {failures}/{len(system_jsons)} system(s) failed",
                file=sys.stderr,
            )
            return 1

        print(f"\n==> All {len(system_jsons)} system(s) generated successfully")
        return 0


# A user's main() may fall off its end (legal for main, not for the renamed
# lf_unused_main_N): silence -Wreturn-type in the staged user sources only.
_STAGED_MAIN_PRAGMA = '#pragma GCC diagnostic ignored "-Wreturn-type"\n'

GENERATE_STATUS_FILE = "generate_status.json"


class _Tee:
    """Write to the real stream and keep a copy of the text."""

    def __init__(self, stream, sink: list[str]):
        self._stream = stream
        self._sink = sink

    def write(self, text):
        self._sink.append(text)
        return self._stream.write(text)

    def flush(self):
        self._stream.flush()

    def __getattr__(self, name):
        return getattr(self._stream, name)


def _generate_recorded(**kwargs) -> int:
    """Run :func:`_generate_one` and record its outcome next to system.json.

    Writes ``generate_status.json`` (``generated``, ``refused`` with the
    ``RCLLF-H...`` hazard code from the output, or ``failed``) so that
    ``colcon lf`` and ``ros2 lf list`` can report each system.
    """
    import re

    system_json = kwargs["system_json"]
    seen: list[str] = []
    out, err = sys.stdout, sys.stderr
    sys.stdout, sys.stderr = _Tee(out, seen), _Tee(err, seen)
    try:
        rc = _generate_one(**kwargs)
    finally:
        sys.stdout, sys.stderr = out, err
    status = {"status": "generated"}
    if rc != 0:
        text = "".join(seen)
        code = re.search(r"RCLLF-H\d+", text)
        errors = [ln.strip() for ln in text.splitlines()
                  if "rror" in ln or (code and code.group(0) in ln)]
        if code:  # the line that names the hazard is the reason
            errors = [ln for ln in errors if code.group(0) in ln] or errors
        status = {
            "status": "refused" if code else "failed",
            "code": code.group(0) if code else None,
            "reason": (errors[-1] if errors else "")[:300],
        }
    try:
        (Path(system_json).parent / GENERATE_STATUS_FILE).write_text(
            json.dumps(status) + "\n", encoding="utf-8")
    except OSError:
        pass
    return rc


def _generate_one(
    *,
    system_json: Path,
    output_dir: str | None,
    after_delay: str | None,
    generate_launch: bool = True,
    target_lfc: bool = True,  # deprecated no-op, lfc is the only path
    federate: str | None = None,
    exec_config_arg: str | None = None,
    params_files: list[str] | None = None,
    workspace: Path | None = None,
    coordination: str | None = None,
    fast: bool = False,
    timeout: str | None = None,
) -> int:
    """Generate LF code from one system.json. Returns 0 on success."""
    print(f"--- {system_json} ---")

    try:
        with open(system_json) as f:
            raw = f.read()

        from colcon_lf.output.serializer import deserialize_system_graph

        raw_data = json.loads(raw)
        graph = deserialize_system_graph(raw)
    except Exception as exc:
        print(f"  Error loading: {exc}", file=sys.stderr)
        return 1

    n_nodes = len(graph.nodes)
    n_topics = len(graph.communication.topics)
    print(f"  {n_nodes} nodes, {n_topics} topics")

    # An older system.json (before endpoints carried the node key) whose endpoint names several node instances is refused
    # before anything is staged.
    try:
        endpoint_key_map(graph)
    except AmbiguousEndpointError as exc:
        print(f"  Error: {exc}", file=sys.stderr)
        return 1

    _report_lifecycle_component_reroute(graph)

    # --- Hazard-detection gate (schema v3 + diagnostics required) ---
    # A v2-or-older
    # system.json, or one missing the `diagnostics` field entirely, predates
    # hazard detection and must not be silently treated as "no hazards
    # found" (fail-open). Missing field != empty list, so this checks the
    # *raw* parsed JSON, not the deserialized graph (whose `diagnostics`
    # field always exists, defaulted to `[]`, once loaded as a dataclass).
    schema_version = raw_data.get("schema_version")
    has_diagnostics_field = "diagnostics" in raw_data
    if schema_version is None or schema_version < 3 or not has_diagnostics_field:
        print(
            "  Error: analysis predates hazard detection; re-run "
            "'colcon lf-analyze' "
            f"(schema_version={schema_version!r}, "
            f"diagnostics field present={has_diagnostics_field}).",
            file=sys.stderr,
        )
        return 1

    try:
        rc = _check_hazard_gate(graph=graph, system_json=system_json, exec_config_arg=exec_config_arg)
    except Exception as exc:
        print(f"  Error checking hazard diagnostics: {exc}", file=sys.stderr)
        return 1
    if rc != 0:
        return rc

    # --- External topics: every subscribed topic without an
    # in-system publisher must be declared in execution.yaml's
    # external_topics, else its callback would never be dispatched.
    try:
        rc, external_topics, external_anchors = _check_external_topics(
            graph=graph, system_json=system_json, exec_config_arg=exec_config_arg,
        )
    except Exception as exc:
        print(f"  Error checking external topics: {exc}", file=sys.stderr)
        return 1
    if rc != 0:
        return rc
    # A stamp-anchored topic inserts events at an absolute tag,
    # which fast mode would run past before the message exists.
    stamped = sorted(t for t, a in external_anchors.items() if a.get("anchor") == "stamp")
    if fast and stamped:
        print(
            f"  Error: --fast with 'anchor: stamp' external topic(s) {stamped}: fast mode "
            f"advances logical time without waiting for physical time, so a tag computed "
            f"from header.stamp would already be in the past when the message arrives "
            f"and every message would be refused. Run this system without --fast, or "
            f"anchor the topic(s) at arrival.", file=sys.stderr,
        )
        return 1

    # --- Resolve execution-policy config, stage exec_config.json ---
    try:
        exec_config_json_path = _resolve_and_stage_exec_config(
            system_json=system_json,
            graph=graph,
            exec_config_arg=exec_config_arg,
        )
    except Exception as exc:
        print(f"  Error resolving execution config: {exc}", file=sys.stderr)
        return 1

    # --- Resolved model: resolve, stage resolved_model.json,
    # fail-closed gate RCLLF-R001/R002. Right after exec-config resolution
    # (needs the parsed ExecConfig for the P3 timer-period override and
    # acknowledged_hazards), before any LF is emitted.
    try:
        (resolved_model, timer_period_overrides, timer_offset_overrides,
         ack_hazards) = _resolve_and_stage_resolved_model(
            system_json=system_json,
            graph=graph,
            exec_config_arg=exec_config_arg,
            params_files=params_files,
            workspace=workspace,
            after_delay=after_delay,
        )
    except Exception as exc:
        print(f"  Error resolving the resolved model: {exc}", file=sys.stderr)
        return 1

    rc = _check_resolved_model_gate(resolved_model, ack_hazards)
    if rc != 0:
        return rc
    # Timer start offsets go onto the graph's TimerInfo, so every
    # later emission path (single program and federates) reads the same value.
    from colcon_lf.generation.lf_generator import apply_timer_offset_overrides
    graph = apply_timer_offset_overrides(graph, timer_offset_overrides)
    from colcon_lf.generation.lf_generator import apply_sync_queue_params
    graph = apply_sync_queue_params(graph, resolved_model)
    from colcon_lf.generation.binding_header_emitter import collect_foreign_executor_acks
    executor_acks = collect_foreign_executor_acks(resolved_model, ack_hazards, graph)
    from colcon_lf.generation.binding_header_emitter import collect_tf2_configs
    tf2_configs = collect_tf2_configs(resolved_model, graph)

    # --- Resolve the effective after-delay (flag > execution.yaml > default) ---
    try:
        after_delay = _resolve_after_delay(
            after_delay_flag=after_delay,
            system_json=system_json,
            graph=graph,
            exec_config_arg=exec_config_arg,
        )
    except Exception as exc:
        print(f"  Error resolving after-delay: {exc}", file=sys.stderr)
        return 1

    # --- Resolve the per-node lifecycle auto-transition flag ---
    auto_transition = _resolve_auto_transition(
        system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
    )
    # --- Resolve the per-node ingress STORED capacity ---
    ingress_capacity = _resolve_ingress_capacity(
        system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
    )
    # --- Resolve the per-node reaction order at one tag ---
    reaction_order = _resolve_reaction_order(
        system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
    )
    # --- Timers whose offset is set at program start ---
    runtime_offsets = _resolve_runtime_offsets(
        system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
    )

    # Federated mode: resolve partition then generate N+1 packages
    if federate:
        try:
            from colcon_lf.federation.partition import resolve_partition
            fmap = resolve_partition(federate, graph, system_json)
        except Exception as exc:
            print(f"  Error resolving federation partition: {exc}", file=sys.stderr)
            return 1
        # Coordination precedence: --coordination flag > federation YAML's own
        # 'coordination:' key (only the custom-YAML mode can set that) > centralized.
        effective_coordination = coordination or fmap.coordination
        fmap.coordination = effective_coordination
        n_fed = len(fmap.federate_names)
        print(
            f"  Federation: {n_fed} federates [{', '.join(fmap.federate_names)}] "
            f"(coordination: {effective_coordination})"
        )
        cross = sum(
            1 for n1, f1 in fmap.node_to_federate.items()
            for n2, f2 in fmap.node_to_federate.items()
            if f1 != f2
        )
        try:
            from colcon_lf.exec_config.resolve import resolve_corr_bounds

            # The federation is admitted and sized as one system,
            # from the same bounds and timer periods as the unfederated path.
            from colcon_lf.exec_config.connections import link_delays

            fed_exec_config = _load_exec_config(
                system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
            )
            fed_corr_bounds = resolve_corr_bounds(fed_exec_config, graph)
            fed_link_delays = link_delays(fed_exec_config)
            rc = _generate_federated(
                system_json=system_json,
                graph=graph,
                fmap=fmap,
                corr_bounds=fed_corr_bounds,
                timer_period_overrides=timer_period_overrides,
                after_delay=after_delay,
                generate_launch=generate_launch,
                exec_config_json_path=exec_config_json_path,
                auto_transition=auto_transition,
                foreign_executor_acks=executor_acks,
                tf2_configs=tf2_configs,
                external_topics=external_topics,
                external_anchors=external_anchors,
                ingress_capacity=ingress_capacity,
                link_delays=fed_link_delays,
                reaction_order=reaction_order,
                runtime_offsets=runtime_offsets,
                output_deadlines=output_deadlines_of(fed_exec_config),
                fast=fast,
                timeout=timeout,
            )
            if rc == 0:
                from colcon_lf.generation.lf_generator import generate_lf_system
                from colcon_lf.generation.lf_naming import (
                    generated_lfc_package_name, generated_package_stem,
                )

                # The name needs the whole system, built with the same timer
                # periods: without them a timer whose period comes from
                # execution.yaml has none and is refused. It is
                # a federated pass (every edge refused as "federated"): as one
                # process every edge would be local, and a cross-federate
                # 'join: deferred' would fail the local-edge check after
                # the federation was already written.
                name = generated_package_stem(generate_lf_system(
                    graph, after_delay=after_delay, corr_bounds=fed_corr_bounds,
                    timer_period_overrides=timer_period_overrides, federated=True,
                ).name)
                source_package = system_json.parent.parent.name
                fed_pkg_name = (
                    f"{generated_lfc_package_name(source_package, name)}_federated"
                )
                record_generated_packages(
                    system_json, fed_pkg_name, federated=True,
                )
            return rc
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"  Error generating federated artifacts: {exc}", file=sys.stderr)
            return 1

    # Generate .lf files
    try:
        from colcon_lf.exec_config.resolve import resolve_corr_bounds
        from colcon_lf.generation.lf_generator import generate_lf_system
        from colcon_lf.generation.lf_emitter import emit_lf_system

        # Declared burst bounds, take timeouts, input periods and
        # the worker lag bound size (or refuse) every correspondence edge.
        from colcon_lf.exec_config.connections import link_delays

        exec_config = _load_exec_config(
            system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
        )
        corr_bounds = resolve_corr_bounds(exec_config, graph)
        lf_system = generate_lf_system(
            graph,
            after_delay=after_delay,
            target_lfc=True,
            timer_period_overrides=timer_period_overrides,
            external_topics=external_topics,
            corr_bounds=corr_bounds,
            ingress_capacity=ingress_capacity,
            # Per-connection delays from execution.yaml's connections:.
            link_delays=link_delays(exec_config),
            reaction_order=reaction_order,
            runtime_offsets=runtime_offsets,
            # Per-publisher actuation lateness bounds.
            output_deadlines=output_deadlines_of(exec_config),
        )
        # Logical timeout of the one program (fast mode is a
        # launch argument, see run_mode_launch_args).
        from colcon_lf.generation.lf_generator import (
            run_mode_launch_args,
            run_mode_target_options,
        )
        lf_system.target_options.update(run_mode_target_options(fast=fast, timeout=timeout))
        lf_text = emit_lf_system(lf_system)
    except Exception as exc:
        print(f"  Error generating LF: {exc}", file=sys.stderr)
        return 1

    # RCLLF-H001 again, now per edge status.
    from colcon_lf.features.modules.best_effort_input import CODE_H001, h001_by_edge_status

    for severity, message in h001_by_edge_status(graph.diagnostics, lf_system.corr_edges,
                                                 graph=graph):
        print(f"    ! [{severity.value.upper()}] {CODE_H001} {message}", file=sys.stderr)

    if output_dir is not None:
        out_dir = Path(output_dir)
    else:
        out_dir = system_json.parent / "lf"

    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / f"{lf_system.name}.lf"
    out_file.write_text(lf_text, encoding="utf-8")
    print(f"  -> {out_file}")

    # Emit the reaction-ID header and the lf_generated_<package>_<name> package
    # (package-qualified so two source packages sharing a launch
    # stem, e.g. pubsub_example and service_example both analyzing
    # bringup.launch.py, never collide on the same generated package name).
    try:
        pkg_name = _emit_lfc_artifacts(
            system_json=system_json,
            out_dir=out_dir,
            lf_system=lf_system,
            graph=graph,
            auto_transition=auto_transition,
            foreign_executor_acks=executor_acks,
            tf2_configs=tf2_configs,
            external_topics=external_topics,
            external_anchors=external_anchors,
            timer_period_overrides=timer_period_overrides,
            launch_lf_args=run_mode_launch_args(fast=fast),
        )
    except Exception as exc:
        print(f"  Error generating lfc artifacts: {exc}", file=sys.stderr)
        return 1

    record_generated_packages(system_json, pkg_name)
    return 0


GENERATED_MARKER = "lfc_packages.txt"


def record_generated_packages(system_json: Path, base: str, *, federated: bool = False) -> None:
    """Record which lf_generated_* packages the last lf-generate wrote for a system.

    ``colcon lf-build`` reads this file next to ``system.json``. One line:
    ``<base> single`` or ``<base> federated`` (the federated coordinator; its
    per-federate packages are ``<base>_<federate>``). Overwritten on every
    generate, so a switch between single-process and federated builds only
    what was generated last.
    """
    marker = system_json.parent / GENERATED_MARKER
    marker.write_text(f"{base} {'federated' if federated else 'single'}\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Hazard-detection gate helpers
# ---------------------------------------------------------------------------

def _load_exec_config(
    *,
    system_json: Path,
    graph,
    exec_config_arg: str | None,
):
    """Load the ``ExecConfig`` for a system.json, shared by the hazard gate
    and ``_resolve_and_stage_exec_config``.

    The user layer is the explicit ``--exec-config`` file, which must exist
    (hard error, never a silent fall-through — frozen contract, 00_design.md
    section 2.4 item 4), else the analysis dir's ``config/execution.yaml``.
    It is layered value by value over the package-owned
    ``lf/<launch stem>.execution.yaml`` (``ExecConfig.load_layered``),
    so an explicit file never drops the package's declarations. Neither file
    present means built-in defaults via ``ExecConfig.empty()``.

    Both call sites load it here, once.
    """
    from colcon_lf.exec_config.model import ExecConfig, package_exec_config_path

    if exec_config_arg is not None:
        exec_config_path = Path(exec_config_arg)
        if not exec_config_path.is_file():
            raise ValueError(f"--exec-config file not found: {exec_config_path}")
        return ExecConfig.load_layered(
            exec_config_path, package_exec_config_path(graph), graph,
            user_label="--exec-config",
        )

    default_path = system_json.parent / "config" / "execution.yaml"
    return ExecConfig.load_layered(
        default_path if default_path.is_file() else None,
        package_exec_config_path(graph),
        graph,
    )


# Documented fallback when neither the flag nor execution.yaml sets a value.
# "0" is ``after 0``: the subscriber runs one microstep after the
# publisher, at (t, m + 1), the smallest delay that still orders it after the
# publisher, so the CLI adds no logical latency of its own. Any
# d > 0 is an application choice (a LET budget) and belongs in
# execution.yaml or system.json. Same value as the library's
# DEFAULT_AFTER_DELAY_NS, stated here because the CLI documents it.
DEFAULT_AFTER_DELAY = "0"


def _resolve_after_delay(
    *,
    after_delay_flag: str | None,
    system_json: Path,
    graph,
    exec_config_arg: str | None,
) -> str:
    """Resolve the effective default after-delay string for one system.

    Precedence, highest first (a per-link ``after_delay_ns`` in system.json
    beats all of these, and is applied later in ``_build_connections``):

    1. explicit ``--after-delay`` flag
    2. ``execution.yaml`` → ``defaults.after_delay``
    3. ``DEFAULT_AFTER_DELAY`` ("0")
    """
    if after_delay_flag is not None:
        return after_delay_flag

    try:
        config = _load_exec_config(
            system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
        )
    except Exception:
        # Config problems are reported by the hazard gate / staging step with
        # far better context; never let this lookup be the thing that fails.
        return DEFAULT_AFTER_DELAY

    from colcon_lf.exec_config.policies import KEY_AFTER_DELAY

    configured = config.defaults.get(KEY_AFTER_DELAY)
    if configured:
        return str(configured)
    return DEFAULT_AFTER_DELAY


def _resolve_auto_transition(
    *,
    system_json: Path,
    graph,
    exec_config_arg: str | None,
) -> dict:
    """Resolve the per-node lifecycle auto-transition flag.

    Returns ``{node_key: bool}`` for every node in *graph*. Precedence is the
    registry default (True), then ``execution.yaml``'s ``defaults``, then its
    per-node ``nodes:`` entry — the same per-key precedence the runtime policy
    keys use, minus the subscription scope the key does not have.

    Generation-time only, like ``after_delay``: the flag decides whether the
    generated factory builds a transition thunk at all, so it never reaches
    ``exec_config.json`` and cannot change that file's frozen shape.
    """
    from colcon_lf.exec_config.policies import KEY_AUTO_TRANSITION, REGISTRY

    fallback = bool(REGISTRY[KEY_AUTO_TRANSITION].default)
    try:
        config = _load_exec_config(
            system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
        )
    except Exception:
        # Same reasoning as _resolve_after_delay: the hazard gate and the
        # staging step report config problems with far better context, and
        # both run before this. Never let this lookup be the thing that fails.
        return {}

    system_default = config.defaults.get(KEY_AUTO_TRANSITION, fallback)
    resolved: dict = {}
    for node_key in graph.nodes:
        node_value = config.nodes.get(node_key, {}).get(KEY_AUTO_TRANSITION)
        resolved[node_key] = bool(system_default if node_value is None else node_value)
    _warn_on_lifecycle_opt_out(graph, resolved)
    return resolved


def _resolve_ingress_capacity(
    *,
    system_json: Path,
    graph,
    exec_config_arg: str | None,
) -> dict:
    """Resolve the per-node ingress STORED capacity.

    Returns ``{node_key: int}`` for every node in *graph*, with the same
    precedence as ``_resolve_auto_transition``: registry default (8), then
    ``defaults``, then the node's ``nodes:`` entry. Generation-time only: the
    value is baked into the drain host's startup reaction.
    """
    from colcon_lf.exec_config.policies import KEY_INGRESS_CAPACITY, REGISTRY

    fallback = int(REGISTRY[KEY_INGRESS_CAPACITY].default)
    try:
        config = _load_exec_config(
            system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
        )
    except Exception:
        # The hazard gate and the staging step report config problems first.
        return {}
    system_default = config.defaults.get(KEY_INGRESS_CAPACITY, fallback)
    resolved: dict = {}
    for node_key in graph.nodes:
        node_value = config.nodes.get(node_key, {}).get(KEY_INGRESS_CAPACITY)
        resolved[node_key] = int(system_default if node_value is None else node_value)
    return resolved


def output_deadlines_of(config) -> dict[str, dict[str, int]]:
    """``{node_key: {topic: output_deadline_ns}}`` of ``publishers:``."""
    from colcon_lf.exec_config.yaml_schema import KEY_PUB_OUTPUT_DEADLINE_NS

    return {
        node: {topic: int(body[KEY_PUB_OUTPUT_DEADLINE_NS]) for topic, body in topics.items()}
        for node, topics in (getattr(config, "publishers", None) or {}).items()
    }


def _resolve_reaction_order(
    *,
    system_json: Path,
    graph,
    exec_config_arg: str | None,
) -> dict:
    """Resolve the per-node reaction order at one tag.

    Returns ``{node_key: order}`` for every node in *graph*, with the same
    precedence as ``_resolve_ingress_capacity``: registry default
    (``timers_first``), then ``defaults``, then the node's ``nodes:``
    entry. Generation-time only: it sets the reaction declaration order.
    """
    from colcon_lf.exec_config.policies import KEY_REACTION_ORDER, REGISTRY

    fallback = REGISTRY[KEY_REACTION_ORDER].default
    try:
        config = _load_exec_config(
            system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
        )
    except Exception:
        # The hazard gate and the staging step report config problems first.
        return {}
    system_default = config.defaults.get(KEY_REACTION_ORDER, fallback)
    return {
        node_key: config.nodes.get(node_key, {}).get(KEY_REACTION_ORDER, system_default)
        for node_key in graph.nodes
    }


def _resolve_runtime_offsets(
    *,
    system_json: Path,
    graph,
    exec_config_arg: str | None,
) -> dict[str, set[str]]:
    """``{node_key: {timer callback}}`` with ``offset_runtime: true``.

    Generation-time: each such timer reads its offset from a program option.
    """
    from colcon_lf.exec_config.yaml_schema import KEY_TIMER_OFFSET_RUNTIME, KEY_TIMERS

    try:
        config = _load_exec_config(
            system_json=system_json, graph=graph, exec_config_arg=exec_config_arg,
        )
    except Exception:
        # The hazard gate and the staging step report config problems first.
        return {}
    out: dict[str, set[str]] = {}
    for node_key, body in config.nodes.items():
        for callback, fields in (body.get(KEY_TIMERS) or {}).items():
            if fields.get(KEY_TIMER_OFFSET_RUNTIME):
                out.setdefault(node_key, set()).add(callback)
    return out


def _timer_offsets_launch(lf_system, table: list | None = None, own: bool = False) -> tuple:
    """Launch-file pieces for run-time timer offsets.

    Returns ``(statement, argument expression)``. The statement loads
    ``rcllf_timer_offsets.py`` from beside the launch file and turns
    ``RCLLF_TIMER_OFFSETS`` into program options. Every launch file carries it,
    also with no run-time timer, so a name set for a build without that timer
    is refused, never ignored. *table* is the whole federation's
    ``(node, callback, parameter)`` list; *own* restricts the options to
    *lf_system*'s parameters (a federate).
    """
    rows = lf_system.runtime_offsets if table is None else table
    names = {f"{node.lstrip('/')}/{cb}": param for node, cb, param in rows}
    own_expr = repr({p for _, _, p in lf_system.runtime_offsets}) if own else "None"
    statement = (
        "# RCLLF_TIMER_OFFSETS -> --<timer>_offset options, checked\n"
        "            # against the run-time timers of this build. Unset: no options.\n"
        "            offset_args = runpy.run_path(os.path.join(\n"
        "                os.path.dirname(os.path.realpath(__file__)), \"rcllf_timer_offsets.py\"),\n"
        f"            )[\"timer_offset_args\"]({names!r}, {own_expr}) if os.environ.get(\n"
        "                \"RCLLF_TIMER_OFFSETS\", \"\").strip() else []"
    )
    return statement, "offset_args"


def _copy_timer_offsets_helper(launch_dir: Path) -> None:
    """Install ``rcllf_timer_offsets.py`` beside a generated launch file."""
    import shutil

    from colcon_lf.generation import rcllf_timer_offsets

    shutil.copyfile(rcllf_timer_offsets.__file__, launch_dir / "rcllf_timer_offsets.py")


def _warn_on_lifecycle_opt_out(graph, resolved: dict) -> None:
    """Warn for each lifecycle node that opted out of automatic bring-up.

    ``auto_transition: false`` hands activation to "a supervisor, state machine
    or operator". An operator can transition the
    node (the binding's ingress role serves ``change_state`` on an LF worker),
    and the binding follows every transition: an
    entity binds at its first dispatch after the node becomes active, and
    nothing of the node runs while it is not active.

    So a node nobody activates stays idle for the whole run, and its sole
    symptom is a ``0 timer(s), 0 subscription(s)`` line. A warning rather than
    an error, because a supervisor or an operator is a real and legitimate use.
    """
    opted_out = [
        ni for key, ni in graph.nodes.items()
        if ni.is_lifecycle and not resolved.get(key, True)
    ]
    if not opted_out:
        return
    print(
        f"  WARNING: {len(opted_out)} managed lifecycle node(s) set "
        "'auto_transition: false', so nothing in the generated program drives "
        "them to 'active':",
        file=sys.stderr,
    )
    for ni in opted_out:
        print(f"    {ni.package or '<unknown package>'}::{ni.class_name}", file=sys.stderr)
    print(
        "  Their callbacks stay idle until an in-process supervisor or "
        "'ros2 lifecycle set' activates them, and are bound then. "
        "Remove 'auto_transition: false' unless something owns "
        "activation.",
        file=sys.stderr,
    )


def _check_hazard_gate(
    *,
    graph,
    system_json: Path,
    exec_config_arg: str | None,
) -> int:
    """Refuse generation on unacknowledged error-severity diagnostics.

    Error-severity codes (resolved from the features REGISTRY at *read*
    time, never from a persisted severity) refuse generation
    unless listed in execution.yaml's ``acknowledged_hazards:``. Warnings and
    info diagnostics print but never block. Acknowledgements are logged
    loudly, per the design.

    Loads the ExecConfig via the shared ``_load_exec_config`` helper so the
    gate runs before, and does not depend on the side effects of,
    ``_resolve_and_stage_exec_config``'s staging of ``exec_config.json`` —
    malformed exec-config files are still a hard error here, consistent with
    the frozen contract's "never a silent fall-through".
    """
    from colcon_lf.features.diagnostics import Severity
    from colcon_lf.features.registry import severity_for

    config = _load_exec_config(
        system_json=system_json, graph=graph, exec_config_arg=exec_config_arg
    )

    acknowledged = set(config.acknowledged_hazards)

    error_diags = [
        diag for diag in graph.diagnostics if severity_for(diag.code) == Severity.ERROR
    ]
    if not error_diags:
        return 0

    by_code: dict[str, list] = {}
    for diag in error_diags:
        by_code.setdefault(diag.code, []).append(diag)

    unacknowledged_codes = sorted(set(by_code) - acknowledged)
    acknowledged_codes_present = sorted(set(by_code) & acknowledged)

    for code in acknowledged_codes_present:
        print(
            f"  Warning: acknowledged hazard {code} present "
            f"({len(by_code[code])} occurrence(s)) — proceeding because it is "
            f"listed in execution.yaml's acknowledged_hazards.",
            file=sys.stderr,
        )

    if unacknowledged_codes:
        print(
            "  Error: generation refused — error-severity hazard diagnostics "
            "present:",
            file=sys.stderr,
        )
        for code in unacknowledged_codes:
            print(f"    ! {code} ({len(by_code[code])} occurrence(s))", file=sys.stderr)
            for diag in by_code[code][:3]:
                print(f"      {diag.message}", file=sys.stderr)
        print(
            "  To proceed anyway, add the code(s) to execution.yaml's "
            "'acknowledged_hazards:' list.",
            file=sys.stderr,
        )
        return 1

    return 0


def _check_tf2_external_ingress(
    graph, runtime_to_key: dict, acknowledged: set[str], anchors: dict | None = None,
) -> int:
    """N1: an admitted TransformListener whose /tf or /tf_static
    no node of this system publishes, and which execution.yaml does not
    declare as a stamp-anchored external topic, refuses with RCLLF-R002
    tf2_clock_mismatch.

    Undeclared, such transforms would enter from outside at arrival time (or
    not at all), so the Buffer's content at a tag, and with it every lookup
    outcome, would depend on physical timing. Declared ``anchor: stamp``,
    the header stamps are their provenance: the binding admits each
    TFMessage at the one header.stamp all its transforms carry and ends a
    message with differing stamps unadmitted as ``tf_mixed_stamps`` (plan
    S2). Acknowledging RCLLF-R002 proceeds as unsupported (the
    external-topic gate still applies). Returns 1 when it printed a refusal.
    """
    if "RCLLF-R002" in acknowledged:
        return 0
    from colcon_lf.exec_config.yaml_schema import EXT_ANCHOR_STAMP, KEY_EXT_ANCHOR

    stamped = {t for t, a in (anchors or {}).items()
               if isinstance(a, dict) and a.get(KEY_EXT_ANCHOR) == EXT_ANCHOR_STAMP}
    published = {tc.topic for tc in graph.communication.topics if tc.publishers}
    rc = 0
    for key, node in graph.nodes.items():
        for sub in getattr(node, "subscribers", None) or []:
            if (not getattr(sub, "tf2_listener", None) or sub.topic in published
                    or sub.topic in stamped):
                continue
            print(
                f"  Error: generation refused, RCLLF-R002 tf2_clock_mismatch: {key} "
                f"TransformListener {sub.tf2_listener} subscribes '{sub.topic}', which no node "
                f"in this system publishes; transforms from outside have no declared stamp "
                f"provenance, so the Buffer's content at a tag would depend on arrival time; "
                f"broadcast them from a node of this system, or declare '{sub.topic}' in "
                f"execution.yaml external_topics with 'anchor: stamp' so each TFMessage enters "
                f"at its header stamp (or list RCLLF-R002 in acknowledged_hazards "
                f"to proceed as unsupported)",
                file=sys.stderr,
            )
            rc = 1
    return rc


def _check_external_topics(
    *,
    graph,
    system_json: Path,
    exec_config_arg: str | None,
) -> tuple[int, set[str]]:
    """Refuse a subscription whose topic has no in-system publisher.

    Such a topic is fed only from outside the generated system. Unless
    execution.yaml lists it under ``external_topics:``, its LF input port
    would stay unconnected and every message would be dropped silently.
    Returns ``(rc, declared external topics, external anchors)``; the anchors
    are ``config.external_anchors`` (stamp-anchored topics).
    """
    config = _load_exec_config(
        system_json=system_json, graph=graph, exec_config_arg=exec_config_arg
    )
    declared = set(config.external_topics)
    anchors = dict(config.external_anchors)
    runtime_to_key = endpoint_key_map(graph)
    if _check_tf2_external_ingress(graph, runtime_to_key, set(config.acknowledged_hazards),
                                   anchors):
        return 1, declared, anchors
    missing = [
        tc for tc in graph.communication.topics
        if tc.subscribers and not tc.publishers and tc.topic not in declared
    ]
    if not missing:
        rc = _check_external_take_mode(config=config, graph=graph, declared=declared)
        return rc, declared, anchors

    from colcon_lf.exec_config.model import package_exec_config_path

    yaml_path = (
        Path(exec_config_arg) if exec_config_arg is not None
        else package_exec_config_path(graph)
        or system_json.parent / "config" / "execution.yaml"
    )
    # One error per (node instance, topic): a node can be listed more than
    # once for a topic, e.g. once per subscription it creates on it.
    reported: set[tuple[str, str]] = set()
    for tc in missing:
        for sub in tc.subscribers:
            key = runtime_to_key.get(sub, sub)
            if (key, tc.topic) in reported:
                continue
            reported.add((key, tc.topic))
            node = graph.nodes.get(key)
            cls = f" (class {node.class_name})" if node is not None else ""
            print(
                f"  Error: node '{key}'{cls} subscribes to '{tc.topic}', which no "
                f"node in this system publishes. Under rcllf its callback would "
                f"never be dispatched.",
                file=sys.stderr,
            )
    print(
        "  Refusing to generate instead of dropping their messages silently. "
        "If the messages come from outside the system (a driver, "
        "'ros2 topic pub', a rosbag), declare the topic(s) external in "
        f"{yaml_path}:",
        file=sys.stderr,
    )
    print("    external_topics:", file=sys.stderr)
    for tc in missing:
        print(f"      - {tc.topic}", file=sys.stderr)
    return 1, declared, anchors


def _check_external_take_mode(*, config, graph, declared: set[str]) -> int:
    """Refuse ``take_mode: block`` on an external subscription.

    An internal subscription takes its message when its reaction runs, so
    ``block`` can poll for a late one. An external message enters as an LF
    physical action: the ingress thread has already taken it when the action
    is scheduled, and the reaction runs on that arrival. There is nothing left
    to wait for, so ``block`` cannot apply, and ``skip`` (the default) is what
    every external subscription does.
    """
    from colcon_lf.exec_config.resolve import resolve

    names = {t.lstrip("/") for t in declared}
    rows = [
        row for row in resolve(config, graph).subscriptions
        if row.topic.lstrip("/") in names and row.values.get("take_mode") == "block"
    ]
    for row in rows:
        node, topic = row.node_keys[0], "/" + row.topic.lstrip("/")
        print(
            f"Error: take_mode 'block' on node '{node}' (binding key "
            f"{row.node_class}) for external topic '{topic}', set by "
            f"{row.origins.get('take_mode', 'built-in')}. An external_topics "
            f"message enters as an LF physical action that the binding has "
            f"already taken when the reaction runs, so there is nothing to wait "
            f"for and 'block' cannot apply. Set it to skip for this subscription:",
            file=sys.stderr,
        )
        print(
            f"    subscriptions:\n      {node}:\n        {topic}:\n"
            f"          take_mode: skip",
            file=sys.stderr,
        )
    return 1 if rows else 0


def _declare_external_topics(
    startup_text: str, external_topics, external_anchors: dict | None = None,
) -> str:
    """Tell the binding which topics are external before it starts.

    ``ros2_lf_runtime_wait_ready`` waits for every subscription to match a
    publisher. A declared external topic has no publisher in the system, so
    without this its subscription made every start wait the full timeout.
    ``ros2_lf_runtime_init`` reports the ``ROS2_LF_TAKE_MODE`` override, which
    leaves external subscriptions out, so the declarations come before it.

    A topic with ``anchor: stamp`` also gets its constants
    declared, so the binding tags its messages from ``header.stamp``.
    """
    if not external_topics:
        return startup_text
    anchors = external_anchors or {}
    lines = []
    for t in sorted(external_topics):
        lines.append(f'    ros2_lf_runtime_declare_external_topic("{t}");\n')
        a = anchors.get(t)
        if a is not None and a.get("anchor") == "stamp":
            lines.append(
                f'    ros2_lf_runtime_declare_external_stamp_anchor("{t}", '
                f'{int(a["delay_ns"])}LL, {int(a["ingress_reserve_ns"])}LL, '
                f'{int(a["grid_ns"])}LL);\n'
            )
    decls = "".join(lines)
    marker = "    ros2_lf_runtime_init(argc, argv);\n"
    assert startup_text.count(marker) == 1, "startup template changed"
    return startup_text.replace(marker, decls + marker)


# ---------------------------------------------------------------------------
# Federated generation helpers
# ---------------------------------------------------------------------------

def _boundary_port_name(
    port: str, inst: str, instances: set[str], opposite: dict, owned: dict | None = None,
) -> str:
    """Boundary port name for inner *inst*.*port* on a federate reactor.

    The port name itself unless it equals a contained instance or a port of
    the other direction, then ``<inst>_<port>``, then a numbered suffix.
    *owned* maps the boundary outputs to their inner endpoints: an output
    serves one inner port, so a name another inner port forwards into is
    taken. lfc takes one connection into a port, and one output
    fed by two publisher nodes would carry only one of their write counts.
    """
    def taken(name: str) -> bool:
        if name in instances or name in opposite:
            return True
        inners = (owned or {}).get(name)
        return bool(inners) and any((i, p) != (inst, port) for i, p, _ in inners)

    name = port
    if taken(name):
        name = f"{inst}_{port}"
    base, n = name, 1
    while taken(name):
        name = f"{base}_{n}"
        n += 1
    return name


def _generate_federated(
    *,
    system_json: Path,
    graph,
    fmap,
    after_delay: str,
    generate_launch: bool,
    exec_config_json_path: Path | None = None,
    auto_transition: dict | None = None,
    foreign_executor_acks: list[tuple[str, str]] | None = None,
    tf2_configs: list[tuple[str, int, str]] | None = None,
    external_topics: set[str] | None = None,
    external_anchors: dict | None = None,
    corr_bounds=None,
    timer_period_overrides: dict[str, dict[str, float]] | None = None,
    ingress_capacity: dict | None = None,
    link_delays=None,
    reaction_order: dict | None = None,
    runtime_offsets: dict[str, set[str]] | None = None,
    output_deadlines: dict[str, dict[str, int]] | None = None,
    fast: bool = False,
    timeout: str | None = None,
) -> int:
    """Generate N+1 federated colcon packages from a FederationMap. Returns 0 on success."""
    from colcon_lf.generation.lf_generator import (
        ACTION_PORT_TYPE, DEFAULT_AFTER_DELAY_NS, XCORR_PORT_TYPE, XTAG_PORT_TYPE, XfedContext,
        _build_connections, _count_publishes, corr_port_name, generate_lf_system,
        xtag_publisher_count,
        plan_federated_corr, route_fan_in, topic_fan_in, xcorr_outer_port_name, xcorr_port_name,
        run_mode_launch_args, run_mode_target_options, xfed_flush,
    )
    from colcon_lf.generation.lf_time import parse_lf_time_ns
    from colcon_lf.generation.lf_emitter import emit_lf_system, emit_federated_top_level
    from colcon_lf.generation.lf_naming import (
        generated_lfc_package_name, generated_package_stem, launch_stem_to_reactor_name,
        sanitize_lf_identifier,
    )
    from colcon_lf.generation.binding_header_emitter import (
        collect_publisher_qos, collect_rid_table, emit_reaction_ids_header, emit_rid_init_cpp,
    )
    from colcon_lf.generation.binding_key import build_binding_keys
    from colcon_lf.generation.node_identity import build_node_identities
    from colcon_lf.graph.models import SystemGraph

    # Binding keys are synthesized ONCE from the whole system graph and reused
    # for every federate: exec_config.json is resolved against the whole graph
    # and shipped whole to each federate (00_design.md 2.4 S6), so per-federate
    # synthesis would disagree with it for any class whose instances are split.
    system_binding_keys = build_binding_keys(graph.nodes)
    system_identities = build_node_identities(graph)

    lf_analysis_root = system_json.parent.parent.parent
    lf_generated_root = lf_analysis_root.parent / "lf-generated"
    lf_generated_root.mkdir(parents=True, exist_ok=True)

    # Derive system stem (same as single-process)
    full_system = generate_lf_system(
        graph, after_delay=after_delay, target_lfc=True, federated=True,
        external_topics=external_topics, corr_bounds=corr_bounds,
        timer_period_overrides=timer_period_overrides, link_delays=link_delays,
        runtime_offsets=runtime_offsets,
    )
    system_stem = full_system.name  # PascalCase system name
    # Package-qualified: source_package comes from the standard
    # lf-analysis/<package>/<stem>/system.json layout (build_system_graph and
    # _output_path both write to it; _emit_lfc_artifacts derives it the same way).
    source_package = system_json.parent.parent.name
    fed_pkg_name = (
        f"{generated_lfc_package_name(source_package, generated_package_stem(system_stem))}"
        "_federated"
    )
    fed_pkg_dir = lf_generated_root / fed_pkg_name
    fed_pkg_dir.mkdir(parents=True, exist_ok=True)

    # Compute runtime_to_key and key_to_instance (mirrors generate_lf_system internals)
    runtime_to_key = endpoint_key_map(graph)
    key_to_instance: dict[str, str] = {key: sanitize_lf_identifier(key) for key in graph.nodes}

    # Publication correspondence is admitted once over the
    # whole system with the federation map. Each federate gets its rows, and
    # an admitted cross-federate edge becomes an xcorr_g<G> link.
    xfed = XfedContext(
        node_federate=dict(fmap.node_to_federate),
        link_after_ns=fmap.after_ns_for,
        decentralized=fmap.coordination == "decentralized",
        nodes=graph.nodes,
        key_to_instance=key_to_instance,
    )
    peer_fqn = {
        key: f"{ident.namespace.rstrip('/')}/{ident.node_name}"
        for key, ident in system_identities.items()
    }
    default_after_ns = (
        parse_lf_time_ns(after_delay, source="--after-delay") if after_delay else None)
    system_corr_edges, fed_corr_plan = plan_federated_corr(
        graph,
        federate_names=fmap.federate_names,
        xfed=xfed,
        after_delay_ns=default_after_ns,
        binding_keys=system_binding_keys,
        peer_fqn=peer_fqn,
        bounds=corr_bounds,
        timer_period_overrides=timer_period_overrides,
        link_delays=link_delays,
        reaction_order=reaction_order,
    )
    # global id -> (edge, {federate: local id})
    xcorr_by_global = {
        e.edge_id: (e, {
            row.federate: row.edge_id
            for rows in fed_corr_plan.values() for row in rows if row.global_id == e.edge_id
        })
        for e in system_corr_edges if e.edge_id is not None and e.role == "cross"
    }

    # Build ALL connections (same as single-process). The intra-federate links
    # carry the delays each federate's generate_lf_system gives them, so the
    # fan-in check below sees execution.yaml's per-link delays.
    all_connections = _build_connections(
        graph.communication,
        set(graph.nodes.keys()),
        after_delay_ns=default_after_ns,
        link_delays=link_delays,
        runtime_to_key=runtime_to_key,
        key_to_instance=key_to_instance,
        nodes=graph.nodes,
        topic_edges=system_corr_edges,
        scopes=system_identities,
    )
    # Each publisher node of a merged stub input gets its own
    # input port, numbered over the whole system so every federate agrees.
    stub_fan_in = topic_fan_in(all_connections)
    route_fan_in(all_connections, stub_fan_in)

    # Classify connections as intra-federate vs cross-federate
    # instance_name → federate_name lookup
    instance_to_federate: dict[str, str] = {}
    for node_key, inst_name in key_to_instance.items():
        fed_name = fmap.federate_for(node_key)
        if fed_name:
            instance_to_federate[inst_name] = fed_name

    intra_connections: dict[str, list] = {f: [] for f in fmap.federate_names}
    cross_connections: list = []

    for conn in all_connections:
        src_inst = conn.source.split(".")[0]
        dst_inst = conn.destination.split(".")[0]
        src_fed = instance_to_federate.get(src_inst)
        dst_fed = instance_to_federate.get(dst_inst)
        if src_fed and dst_fed and src_fed == dst_fed:
            intra_connections[src_fed].append(conn)
        else:
            # Cross-federate: determine after delay from fmap override or default.
            # route_fan_in already moved a k-th publisher's link to port
            # <port>_fanin_k; the override names the subscriber's own port,
            # which route_fan_in recorded (probe P17h).
            dst_key = conn.user_destination or conn.destination
            # Without an override the link keeps its in-process delay.
            after_ns = fmap.after_ns_for(conn.source, dst_key, conn.after_delay_ns)
            conn.is_cross_federate = True
            conn.federate_after_ns = after_ns
            # STP offset only applies under decentralized coordination; None
            # under centralized means "no STP reaction is emitted" (lf_emitter).
            if fmap.coordination == "decentralized":
                conn.federate_stp_ns = fmap.stp_ns_for(conn.source, dst_key)
            cross_connections.append(conn)

    # An override that matched no cross-federate link is an error.
    from colcon_lf.federation.partition import unmatched_override_errors
    override_errors = unmatched_override_errors(fmap)
    if override_errors:
        for msg in override_errors:
            print(f"  ERROR: {msg}", file=sys.stderr)
        return 1

    # Probe P17h: a stub fan-in is deterministic only if its writers land on one tag.
    from colcon_lf.generation.lf_generator import fan_in_tag_errors
    split_errors = fan_in_tag_errors(all_connections, lambda i: instance_to_federate.get(i, ""))
    # A cross-federate tag link needs one publisher node per topic.
    count_publishes = _count_publishes()
    if count_publishes and not split_errors:
        from colcon_lf.generation.lf_generator import xtag_publisher_errors
        split_errors = xtag_publisher_errors(
            all_connections, lambda i: instance_to_federate.get(i, ""))
    if split_errors:
        for msg in split_errors:
            print(f"  ERROR: {msg}", file=sys.stderr)
        return 1

    # Item5 piece D (probes P27 to P27c): the budget check sees every link the
    # generator emits, the admitted xcorr edges' link ends included, at the
    # generator's own W. A fan-in link is keyed by the subscriber's own port
    # (probe P17h, the CONS3 fix), as the override lookups above.
    from colcon_lf.federation.partition import budget_errors
    xcorr_ports = {corr_port_name(g) for g in xcorr_by_global}
    budget_problems = budget_errors(
        fmap,
        generated_links=[
            (c.source, c.user_destination or c.destination) for c in cross_connections
            if c.source.split(".", 1)[-1] not in xcorr_ports
        ] + [e.link_ports for e, _ in xcorr_by_global.values() if e.link_ports],
    )
    if budget_problems:
        for msg in budget_problems:
            print(f"  ERROR: budget check refused: {msg}", file=sys.stderr)
        return 1

    # Print summary
    n_intra = sum(len(v) for v in intra_connections.values())
    n_cross = len(cross_connections)
    print(f"  Cross-federate connections: {n_cross}, intra-federate: {n_intra}")

    from colcon_lf.comm_analysis.models import SystemCommunication, TopicConnection, ServiceConnection, ActionConnection
    from colcon_lf.generation.lf_model import LFConnection as LFC

    class _FedGraph:
        def __init__(self, nodes, communication, name, launch_graph=None):
            self.nodes = nodes
            self.communication = communication
            self.name = name
            # The whole system's launch model: build_node_identities looks up
            # only this federate's nodes in it, so a launch remap inside one
            # federate names the declared ports.
            self.launch_graph = launch_graph
        process_groups = {}

    # The federated reactor name must match the top-level .lf filename stem.
    # We use "{system_stem}_federated" so file = Bringup_federated.lf, reactor = Bringup_federated.
    federated_reactor_name = f"{system_stem}_federated"

    # Build a mapping: (src_inst, src_port) -> list of (dst_inst, dst_port, after_ns)
    # for all cross-federate connections.  Used to determine which ports each federate
    # must expose on its outer reactor.
    #
    # cross_connections[i].source  = "inner_inst.port"
    # cross_connections[i].destination = "inner_inst.port"
    # instance_to_federate maps inner_inst -> fed_name

    # For each federate, collect the exposed ports it needs.
    # A boundary port can serve multiple inner instances:
    #   - input port → multiple inner subscribers (fan-out inside the federate)
    #   - output port → one inner publisher port; two publisher nodes on one
    #     topic leave on two outputs
    # so values are LISTS of (inner_inst, inner_port).  The boundary port itself
    # is declared once on the outer reactor; the emitter writes one forwarding
    # connection per inner endpoint.
    # Each entry additionally carries the connection's stp_offset_ns (None
    # under centralized coordination, or for output ports which never receive
    # network data so can never be tardy).
    fed_out_ports: dict[str, dict[str, list[tuple[str, str, int | None]]]] = {
        f: {} for f in fmap.federate_names
    }
    fed_in_ports: dict[str, dict[str, list[tuple[str, str, int | None]]]] = {
        f: {} for f in fmap.federate_names
    }

    # Top-level connections are deduped by (src_fed, src_port, dst_fed, dst_port).
    # Two inner-subscriber edges that share the same publishing-side port collapse
    # into a single boundary connection — LF rejects duplicate destinations.
    seen_top_conns: set[tuple[str, str, str, str]] = set()
    top_cross_conns: list = []
    legacy_guarded: dict[str, set[str]] = {f: set() for f in fmap.federate_names}
    # Boundary ports of legacy stub topic links. With publish
    # counting they carry the writer's tag log events (port -> topic, and the
    # edges an input port receives); without it the receiver warns at startup
    # that the edge keeps the count rule.
    xtag_ports: dict[str, dict[str, tuple[str, list[str]]]] = {
        f: {} for f in fmap.federate_names
    }
    count_rule_edges: dict[str, list[tuple[str, str, str]]] = {f: [] for f in fmap.federate_names}
    # Per receiving federate, topic -> publisher instance -> its
    # gated publishers of the topic, and the
    # federates writing the topic there (one: xtag_publisher_errors refuses more).
    xtag_writers: dict[str, dict[str, dict[str, int]]] = {f: {} for f in fmap.federate_names}
    xtag_writer_feds: dict[str, dict[str, set[str]]] = {f: {} for f in fmap.federate_names}
    instance_to_key = {inst: key for key, inst in key_to_instance.items()}

    # Fourth review: a topic's writers' gated publisher
    # count is used only where the analysis proves it exact; otherwise every
    # link of the topic keeps the count rule, warned at startup with the reason.
    xtag_count: dict[tuple[str, str], int] = {}
    xtag_inexact: dict[str, str] = {}
    for conn in cross_connections if count_publishes else []:
        src_inst = conn.source.split(".", 1)[0]
        if conn.topic is None or (src_inst, conn.topic) in xtag_count:
            continue
        key = instance_to_key.get(src_inst, "")
        n, why = xtag_publisher_count(
            graph.nodes.get(key), system_identities.get(key), conn.topic)
        xtag_count[(src_inst, conn.topic)] = n
        if why is not None:
            xtag_inexact.setdefault(conn.topic, f"publisher count not exact: {why}")

    # A boundary port named like a contained instance makes lfc
    # declare _lf_<name> twice. Reference system topics carry the name of
    # their publishing node, so the plain topic-named ports hit this.
    fed_instances: dict[str, set[str]] = {f: set() for f in fmap.federate_names}
    for inst_name, fed_name in instance_to_federate.items():
        fed_instances.setdefault(fed_name, set()).add(inst_name)

    xcorr_global = {corr_port_name(g): g for g in xcorr_by_global}
    for conn in cross_connections:
        src_inst, src_port = conn.source.split(".", 1)
        dst_inst, dst_port = conn.destination.split(".", 1)
        src_fed = instance_to_federate.get(src_inst, "")
        dst_fed = instance_to_federate.get(dst_inst, "")
        if not src_fed or not dst_fed:
            continue

        if src_port in xcorr_global:
            # The records of admitted edge G cross on their own
            # link (D1). They leave at t + 1 ns, so the link is
            # after d - 1 ns and they arrive at t + d. With d = 0
            # (centralized) the link is a plain zero-delay connection, so they
            # arrive at the publishing tag (t, m) itself.
            edge, local_ids = xcorr_by_global[xcorr_global[src_port]]
            outer = xcorr_outer_port_name(edge.edge_id)
            fed_out_ports[src_fed].setdefault(outer, []).append(
                (src_inst, xcorr_port_name(local_ids[src_fed]), None))
            # Under decentralized coordination
            # the ingress waits the link's stp_offset for its records, else it
            # waits 0 and every record is tardy. The xcorr ports name no yaml
            # entry, so the offset is looked up by the user's edge, like its after.
            xstp = (fmap.stp_ns_for(*edge.link_ports)
                    if fmap.coordination == "decentralized" and edge.link_ports else None)
            xin = fed_in_ports[dst_fed].setdefault(outer, [])
            xentry = (dst_inst, xcorr_port_name(local_ids[dst_fed]), xstp)
            if xentry not in xin:
                xin.append(xentry)
            top_cross_conns.append(LFC(
                source=f"{src_fed}.{outer}",
                destination=f"{dst_fed}.{outer}",
                is_cross_federate=True,
                # Records are sent at the publishing tag over the link's own
                # d. A flush edge sends at t + 1 ns instead, so its
                # link is d - 1 ns and the records still land at t + d.
                federate_after_ns=_xcorr_link_after_ns(edge.delay_ns, fmap.coordination),
            ))
            continue

        # Exposed output port name on the source federate reactor.
        # Use the port name directly; prefix with instance name if it would
        # collide with an existing input or a contained instance on the same
        # federate reactor.
        out_port_name = _boundary_port_name(
            src_port, src_inst, fed_instances[src_fed], fed_in_ports[src_fed],
            fed_out_ports[src_fed])
        # One publisher fanning out to several remote subscribers is one
        # inner endpoint: forward it once, so the legacy guard below names
        # it once (a repeated trigger is a C redeclaration).
        out_inners = fed_out_ports[src_fed].setdefault(out_port_name, [])
        if (src_inst, src_port, None) not in out_inners:
            out_inners.append((src_inst, src_port, None))

        # Exposed input port name on the destination federate reactor.
        in_port_name = _boundary_port_name(
            dst_port, dst_inst, fed_instances[dst_fed], fed_out_ports[dst_fed])
        in_inners = fed_in_ports[dst_fed].setdefault(in_port_name, [])
        if (dst_inst, dst_port, conn.federate_stp_ns) not in in_inners:
            in_inners.append((dst_inst, dst_port, conn.federate_stp_ns))
        if conn.topic is not None:
            edge = f"{conn.source} -> {conn.user_destination or conn.destination}"
            reason = xtag_inexact.get(conn.topic) if count_publishes else (
                "generated with RCLLF_COUNT_PUBLISHES=0")
            if reason is None:
                xtag_ports[src_fed].setdefault(out_port_name, (conn.topic, []))
                edges = xtag_ports[dst_fed].setdefault(in_port_name, (conn.topic, []))[1]
                if edge not in edges:
                    edges.append(edge)
                src_inst = conn.source.split(".", 1)[0]
                xtag_writers[dst_fed].setdefault(conn.topic, {})[src_inst] = \
                    xtag_count[(src_inst, conn.topic)]
                xtag_writer_feds[dst_fed].setdefault(conn.topic, set()).add(src_fed)
            elif (edge, conn.topic, reason) not in count_rule_edges[dst_fed]:
                count_rule_edges[dst_fed].append((edge, conn.topic, reason))

        # Top-level connection: dedupe so a single publisher-side port maps to
        # exactly one inbound port per receiving federate.  Multiple inner
        # subscribers share the same boundary port and fan out internally.
        key = (src_fed, out_port_name, dst_fed, in_port_name)
        if key in seen_top_conns:
            continue
        seen_top_conns.add(key)

        top_conn = LFC(
            source=f"{src_fed}.{out_port_name}",
            destination=f"{dst_fed}.{in_port_name}",
            is_cross_federate=True,
            federate_after_ns=conn.federate_after_ns,
        )
        top_cross_conns.append(top_conn)
        if (conn.federate_after_ns or 0) > 0:
            # Every microstep of one time lands on one tag.
            legacy_guarded[src_fed].add(out_port_name)

    # The delay each link across federates runs, for the
    # federated resolved model, and the carriers each federate registers so
    # the read-back can name the network delay it reads.
    cross_links, network_links = _federated_link_facts(
        cross_connections, xcorr_global, xcorr_by_global, instance_to_federate,
        key_to_instance, fmap)
    _stage_federated_resolved_model(system_json, cross_links)

    # Generate per-federate packages
    # federate_launch_stems[fed_name] = PascalCase LF system class name
    # federate_parameters[fed_name] = its run-time timer offsets
    federate_parameters: dict[str, list[tuple[str, str, str]]] = {}
    federate_launch_stems: dict[str, str] = {}
    # per-federate dep info needed by coordinator CMakeLists
    federate_dep_info: dict[str, dict] = {}

    fed_trace_parts: list = []
    for fed_name in fmap.federate_names:
        fed_node_keys = fmap.nodes_for(fed_name)
        if not fed_node_keys:
            continue
        fed_node_set = set(fed_node_keys)
        fed_nodes = {k: v for k, v in graph.nodes.items() if k in fed_node_set}

        # Filter communication to intra-federate only
        fed_topics = []
        for tc in graph.communication.topics:
            pubs_in = [p for p in tc.publishers if runtime_to_key.get(p, p) in fed_node_set]
            subs_in = [s for s in tc.subscribers if runtime_to_key.get(s, s) in fed_node_set]
            if pubs_in and subs_in:
                fed_topics.append(TopicConnection(
                    topic=tc.topic,
                    message_type=getattr(tc, "message_type", ""),
                    publishers=pubs_in,
                    subscribers=subs_in,
                    after_delay_ns=tc.after_delay_ns,
                ))

        fed_services = []
        for sc in graph.communication.services:
            if runtime_to_key.get(sc.server, sc.server) in fed_node_set:
                clients_in = [c for c in sc.clients if runtime_to_key.get(c, c) in fed_node_set]
                if clients_in:
                    fed_services.append(ServiceConnection(
                        service=sc.service,
                        service_type=getattr(sc, "service_type", ""),
                        server=sc.server,
                        clients=clients_in,
                    ))

        fed_actions = []
        for ac in graph.communication.actions:
            if not ac.server:
                continue
            if runtime_to_key.get(ac.server, ac.server) in fed_node_set:
                clients_in = [c for c in ac.clients if runtime_to_key.get(c, c) in fed_node_set]
                # An action with no client anywhere stays: its server takes
                # external requests through a physical action.
                if clients_in or not ac.clients:
                    fed_actions.append(ActionConnection(
                        action=ac.action,
                        action_type=getattr(ac, "action_type", ""),
                        server=ac.server,
                        clients=clients_in,
                    ))

        fed_comm = SystemCommunication(
            topics=fed_topics, services=fed_services, actions=fed_actions,
            endpoint_ids=graph.communication.endpoint_ids,
        )
        fed_graph = _FedGraph(nodes=fed_nodes, communication=fed_comm, name=fed_name,
                              launch_graph=getattr(graph, "launch_graph", None))

        # Generate LFSystem for this federate
        fed_lf = generate_lf_system(
            fed_graph, after_delay=after_delay, target_lfc=True, federated=True,
            binding_keys=system_binding_keys, external_topics=external_topics,
            timer_period_overrides=timer_period_overrides,
            corr_plan=fed_corr_plan.get(fed_name, []),
            corr_bounds=corr_bounds,
            ingress_capacity=ingress_capacity,
            stub_fan_in=stub_fan_in,
            link_delays=link_delays,
            reaction_order=reaction_order,
            runtime_offsets=runtime_offsets,
            output_deadlines=output_deadlines,
        )
        fed_lf.federate_name = fed_name
        stray = set(fed_lf.runtime_offsets) - set(full_system.runtime_offsets)
        if stray:
            raise ValueError(f"federate {fed_name}: run-time timer offsets {sorted(stray)} "
                             "are named differently in the whole system")
        fed_lf.federation = fmap
        fed_lf.network_links = network_links.get(fed_name, [])
        federate_launch_stems[fed_name] = fed_lf.name  # PascalCase class name

        # Attach cross-federate ports so lf_emitter adds them to the outer reactor
        # and emits forwarding connections.  One tuple per (boundary_port, inner)
        # pair: the emitter dedupes the port DECLARATION but writes one forwarding
        # connection per tuple, allowing fan-out (one input → many subscribers).
        # Format: (port_name, direction, lf_type, inner_instance, inner_port, stp_offset_ns)
        cross_ports: list[tuple[str, str, str, str, str, int | None]] = []
        # An action channel crosses with its count.
        counted_ports = {
            p.name for r in fed_lf.reactors for p in r.ports if p.type == ACTION_PORT_TYPE
        }

        def _port_type(name: str, inner_port: str) -> str:
            if name.startswith("xcorr_g"):
                return XCORR_PORT_TYPE
            if name in xtag_ports[fed_name]:
                return XTAG_PORT_TYPE
            return ACTION_PORT_TYPE if inner_port in counted_ports else "stub_t"

        for out_port_name, inners in fed_out_ports[fed_name].items():
            for inner_inst, inner_port, _stp_ns in inners:
                cross_ports.append(
                    (out_port_name, "output", _port_type(out_port_name, inner_port), inner_inst,
                     inner_port, None)
                )
        for in_port_name, inners in fed_in_ports[fed_name].items():
            for inner_inst, inner_port, stp_ns in inners:
                cross_ports.append(
                    (in_port_name, "input", _port_type(in_port_name, inner_port), inner_inst,
                     inner_port, stp_ns)
                )
        fed_lf.cross_federate_ports = cross_ports
        fed_lf.legacy_guard_ports = sorted(legacy_guarded[fed_name])
        fed_lf.legacy_guard_max = 1 if fmap.coordination == "decentralized" else 2
        fed_lf.xtag_ports = xtag_ports[fed_name]
        for topic, feds in xtag_writer_feds[fed_name].items():
            # One writer federate per crossing topic: the binding keeps one
            # writer session per topic.
            assert len(feds) == 1, (topic, sorted(feds))
        fed_lf.xtag_expect = {t: sum(w.values()) for t, w in xtag_writers[fed_name].items()}
        fed_lf.count_rule_edges = count_rule_edges[fed_name]

        # Emit .lf into coordinator's lf/{fed_name}/ (lfc resolves imports relative to top-level)
        top_lf_dir = fed_pkg_dir / "lf"
        top_lf_dir.mkdir(parents=True, exist_ok=True)
        fed_lf_subdir = top_lf_dir / fed_name
        fed_lf_subdir.mkdir(parents=True, exist_ok=True)
        lf_file = fed_lf_subdir / f"{fed_lf.name}.lf"
        lf_file.write_text(emit_lf_system(fed_lf), encoding="utf-8")

        # RID table and headers — go into coordinator's lf/{fed_name}/include/
        class_to_reactor = {r.name: r for r in fed_lf.reactors}
        rid_table = collect_rid_table(
            class_to_reactor, fed_graph, binding_keys=system_binding_keys,
            timer_period_overrides=timer_period_overrides,
        )
        header_text = emit_reaction_ids_header(rid_table)
        include_dir = fed_lf_subdir / "include"
        include_dir.mkdir(parents=True, exist_ok=True)
        (include_dir / "ros2_lf_reaction_ids.h").write_text(header_text, encoding="utf-8")

        # RID init cpp and binding sources go into per-federate package src/
        fed_sub_pkg_name = f"{fed_pkg_name}_{fed_name}"
        fed_sub_pkg_dir = lf_generated_root / fed_sub_pkg_name
        fed_sub_pkg_dir.mkdir(parents=True, exist_ok=True)
        src_dir = fed_sub_pkg_dir / "src"
        src_dir.mkdir(parents=True, exist_ok=True)
        # Acknowledgements carry binding keys, like the RID table.
        fed_keys = {system_binding_keys.get(n, ni.class_name) for n, ni in fed_graph.nodes.items()}
        init_cpp = emit_rid_init_cpp(
            rid_table, fed_sub_pkg_name, corr_edges=fed_lf.corr_edges,
            legacy_ingress=fed_lf.corr_legacy_ingress,
            publisher_qos=collect_publisher_qos(fed_graph, binding_keys=system_binding_keys),
            foreign_executor_acks=[
                a for a in foreign_executor_acks or [] if a[0] in fed_keys
            ],
            tf2_configs=[c for c in tf2_configs or [] if c[0] in fed_keys],
            lf_system=fed_lf,
        )
        (src_dir / "ros2_lf_reaction_ids_init.cpp").write_text(init_cpp, encoding="utf-8")
        _write_correspondence_json(fed_sub_pkg_dir, fed_lf.corr_edges)
        # Collected for the one static trace manifest of the federation.
        fed_trace_parts.append((fed_name, fed_sub_pkg_dir, fed_lf.corr_edges, rid_table))

        # node_factories.cpp for non-component nodes — one registration per
        # node *instance*, keyed by the system-wide binding key.
        component_nodes = [ni for ni in fed_graph.nodes.values() if ni.takes_component_path]
        non_component_items = [
            (key, ni) for key, ni in fed_graph.nodes.items() if not ni.takes_component_path
        ]
        non_component_nodes = [ni for _, ni in non_component_items]
        if non_component_nodes:
            nodes_dir = src_dir / "nodes"
            nodes_dir.mkdir(parents=True, exist_ok=True)
            staged_sources = []
            staged_text: dict = {}
            staged_from: dict = {}
            for idx, (_node_key, ni) in enumerate(non_component_items):
                src_file = str(ni.source_file) if ni.source_file else ""
                pkg_nodes_dir = nodes_dir / (ni.package or "unknown")
                pkg_nodes_dir.mkdir(exist_ok=True)
                dest = _staged_node_dest(pkg_nodes_dir, src_file, ni, staged_from)
                define_line = (
                    _staged_define_lines(ni)
                    + f"#define main lf_unused_main_{idx}\n"
                    + _STAGED_MAIN_PRAGMA
                )
                # Header-less nodes: append a factory next to the class
                # definition (same scheme as the non-federated emitter).
                factory_suffix = ""
                if not ni.header_file:
                    define_line += _HEADERLESS_FACTORY_INCLUDES
                    factory_suffix = emit_headerless_factory_suffix(
                        idx, ni.class_name,
                        qualified_name=getattr(ni, "qualified_name", "") or "",
                        auto_transition=(auto_transition or {}).get(_node_key, True),
                    )
                # Two instances of one class share one source file: stage it
                # once and append each instance's factory, else the second write
                # drops the first instance's ros2_lf_make_node_{idx}.
                if dest in staged_text:
                    staged_text[dest] += factory_suffix
                    dest.write_text(staged_text[dest], encoding="utf-8")
                    continue
                staged_text[dest] = (
                    define_line + _stage_node_source_or_fail(src_file, ni) + factory_suffix)
                dest.write_text(staged_text[dest], encoding="utf-8")
                staged_sources.append((ni.class_name, ni.package or "", f"src/nodes/{ni.package or 'unknown'}/{dest.name}", idx))
        else:
            staged_sources = []

        import textwrap as _tw
        if non_component_nodes:
            (src_dir / "node_factories.cpp").write_text(
                emit_node_factories_cpp(
                    _factory_entries(
                        non_component_items, system_binding_keys, system_identities,
                        auto_transition,
                    )
                )
            )
        else:
            (src_dir / "node_factories.cpp").write_text(
                emit_empty_node_factories_cpp("no non-component nodes.")
            )

        # ros2_lf_startup.cpp
        startup_text = _tw.dedent(f"""\
            // Auto-generated by colcon lf-generate. DO NOT EDIT.
            // Copyright 2024 rcllf Contributors
            // SPDX-License-Identifier: Apache-2.0
            #include <cstdlib>
            #include "ros2_lf_binding/ros2_lf_binding.h"

            void ros2_lf_reaction_ids_init();
            void ros2_lf_register_all_nodes();

            static void ros2_lf_shutdown_atexit() {{
                ros2_lf_runtime_shutdown();
            }}

            // The link wraps reactor-c's start time handshake
            // (-Wl,--wrap in ros2_lf_bindings.cmake). The RTI answers once
            // every federate has connected, so the peer check runs then,
            // before tag 0, and its timeout no longer spans the launch.
            extern "C" void __real_lf_synchronize_with_other_federates(void);
            extern "C" void __wrap_lf_synchronize_with_other_federates(void) {{
                __real_lf_synchronize_with_other_federates();
                ros2_lf_runtime_peer_check();
            }}

            // Known limitation: reactor-c's termination() runs before
            // the atexit shutdown below and resigns to the RTI. The RTI can hold
            // that resign behind a peer's until the peer exits, while the peer
            // waits for this federate's end counts. Publish them first: the
            // binding shutdown runs here, before the resign; the atexit call
            // then only ends the process.
            // Only a normal end: after a SIGINT the MSG_TYPE_FAILED to the RTI
            // goes out first and the atexit shutdown runs as before.
            struct environment_t;
            extern "C" bool _lf_normal_termination;
            extern "C" void __real_lf_terminate_execution(environment_t* env);
            extern "C" void __wrap_lf_terminate_execution(environment_t* env) {{
                if (_lf_normal_termination) {{
                    ros2_lf_runtime_shutdown_before_resign();
                    ros2_lf_runtime_test_resign_delay();
                }}
                __real_lf_terminate_execution(env);
            }}

            __attribute__((constructor))
            static void ros2_lf_pre_main_init() {{
                ros2_lf_reaction_ids_init();
                static const char* argv0[] = {{"lf_bringup", nullptr}};
                int argc = 1;
                char** argv = const_cast<char**>(argv0);
                ros2_lf_runtime_init(argc, argv);
                ros2_lf_register_all_nodes();
                ros2_lf_runtime_load_nodes();
                ros2_lf_runtime_defer_peer_check();
                // Anchor LF logical time zero to the instant DDS discovery
                // settles, so the first tags are not released in a startup burst.
                ros2_lf_runtime_wait_ready();
                std::atexit(ros2_lf_shutdown_atexit);
            }}
        """)
        (src_dir / "ros2_lf_startup.cpp").write_text(
            _declare_external_topics(startup_text, external_topics, external_anchors),
            encoding="utf-8",
        )

        # Per-federate system.json (filtered to this federate's nodes)
        fed_system_json: Path | None = None
        if system_json is not None:
            import json as _json
            raw = _json.loads(system_json.read_text(encoding="utf-8"))
            all_nodes = raw.get("nodes", {})
            if "nodes" in raw:
                raw["nodes"] = {k: v for k, v in raw["nodes"].items() if k in fed_node_set}
            if "communication" in raw:
                comm = raw["communication"]
                # The topics filtered out below cross this
                # federate's boundary. Their system-wide endpoint counts let the
                # runtime wait for DDS to match them before the first tag.
                raw[CROSS_FEDERATE_TOPICS_KEY] = cross_federate_topics(
                    comm.get("topics", []), fed_node_set, runtime_to_key, all_nodes,
                )
                if "topics" in comm:
                    comm["topics"] = [
                        t for t in comm["topics"]
                        if (all(runtime_to_key.get(p, p) in fed_node_set for p in t.get("publishers", []))
                            and all(runtime_to_key.get(s, s) in fed_node_set for s in t.get("subscribers", [])))
                    ]
                if "services" in comm:
                    comm["services"] = [
                        s for s in comm["services"]
                        if runtime_to_key.get(s.get("server", ""), s.get("server", "")) in fed_node_set
                    ]
                if "actions" in comm:
                    comm["actions"] = [
                        a for a in comm["actions"]
                        if runtime_to_key.get(a.get("server", ""), a.get("server", "")) in fed_node_set
                    ]
            # Per-instance component list, filtered to this federate's nodes.
            # Keys come from the system-wide map so every federate agrees with
            # the whole exec_config.json table it receives.
            raw[COMPONENT_INSTANCES_KEY] = [
                entry
                for key, entry in zip(
                    [k for k, ni in graph.nodes.items() if ni.takes_component_path],
                    build_component_instances(
                        graph, system_binding_keys, system_identities,
                    ),
                )
                if key in fed_node_set
            ]
            fed_system_json = fed_sub_pkg_dir / "system.json"
            fed_system_json.write_text(_json.dumps(raw, indent=2), encoding="utf-8")

        # Per-federate exec_config.json — the WHOLE resolved table, unfiltered
        # (00_design.md section 2.4 S6: simpler; rows for other federates are
        # simply never looked up). Staged into the federate's own package dir
        # so it installs and is discoverable via get_package_share_directory,
        # mirroring fed_system_json above.
        fed_exec_config_json: Path | None = None
        if exec_config_json_path is not None and exec_config_json_path.is_file():
            fed_exec_config_json = fed_sub_pkg_dir / "exec_config.json"
            fed_exec_config_json.write_text(
                exec_config_json_path.read_text(encoding="utf-8"), encoding="utf-8"
            )

        # Collect dep info for coordinator CMakeLists
        component_packages = sorted({ni.package for ni in component_nodes if ni.package})
        non_component_packages = sorted({ni.package for ni in non_component_nodes if ni.package})
        all_nodes_list = list(fed_graph.nodes.values())
        extra_packages: set[str] = set()
        if any(getattr(ni, "action_servers", []) or getattr(ni, "action_clients", []) for ni in all_nodes_list):
            extra_packages.add("rclcpp_action")
        direct_packages = set(component_packages) | set(non_component_packages) | extra_packages
        transitive: set[str] = set()
        if system_json is not None and non_component_packages:
            from colcon_lf.generation.package_deps import _read_package_deps
            workspace_src = system_json.parent
            for _ in range(6):
                workspace_src = workspace_src.parent
                if (workspace_src / "src").is_dir():
                    break
            for pkg in non_component_packages:
                for dep in _read_package_deps(workspace_src / "src", pkg):
                    if dep not in direct_packages:
                        transitive.add(dep)
        _hardcoded = {"rclcpp", "rclcpp_components", "ros2_lf_binding"}
        all_dep_packages = sorted(direct_packages | transitive)
        federate_dep_info[fed_name] = {
            "all_dep_packages": all_dep_packages,
            "staged_sources": staged_sources,
            "non_component_nodes": non_component_nodes,
            "system_json": fed_system_json,
        }

        # Per-federate package: launch + system.json + package.xml only.
        # The binary is built by the coordinator CMakeLists and installed into
        # the coordinator's lib/<coord_pkg_name>/. The per-federate launch file
        # references it from the coordinator package.
        _emit_federate_launch_package(
            pkg_dir=fed_sub_pkg_dir,
            pkg_name=fed_sub_pkg_name,
            coord_pkg_name=fed_pkg_name,
            fed_name=fed_name,
            lf_stem=fed_lf.name,
            system_json=fed_system_json,
            all_dep_packages=all_dep_packages,
            exec_config_json_path=fed_exec_config_json,
            fed_lf=fed_lf,
            runtime_offsets_table=full_system.runtime_offsets,
            launch_lf_args=run_mode_launch_args(fast=fast),
        )
        print(f"  -> {fed_sub_pkg_dir}/ (federate: {fed_name})")
        federate_parameters[fed_name] = list(fed_lf.parameters)

    # Emit top-level federated .lf
    # The federated reactor name must match the filename stem.
    top_target_options = {
        "build-type": "RelWithDebInfo",
        "keepalive": True,
        # Path is relative to the staged top-level .lf at lf-src/<reactor>.lf;
        # the dispatch file lives at lf-src/include/ros2_lf_bindings.cmake.
        "cmake-include": '"include/ros2_lf_bindings.cmake"',
    }
    # Only add the target property for decentralized: centralized is LF's own
    # default, so omitting it there keeps existing (centralized) generated
    # output byte-identical to before this feature.
    if fmap.coordination == "decentralized":
        top_target_options["coordination"] = "decentralized"
    # The logical timeout goes on the top-level file, the one lfc
    # reads the target block from; the per-federate files are imports. Fast
    # mode is a launch argument of every federate (run_mode_launch_args).
    top_target_options.update(run_mode_target_options(fast=fast, timeout=timeout))
    top_lf_text = emit_federated_top_level(
        system_name=federated_reactor_name,
        federate_names=fmap.federate_names,
        cross_connections=top_cross_conns,
        target="CCpp",
        target_options=top_target_options,
        federate_lf_stems=federate_launch_stems,
        coordination=fmap.coordination,
        federate_parameters=federate_parameters,
    )
    top_lf_dir = fed_pkg_dir / "lf"
    top_lf_dir.mkdir(parents=True, exist_ok=True)
    top_lf_file = top_lf_dir / f"{federated_reactor_name}.lf"
    top_lf_file.write_text(top_lf_text, encoding="utf-8")
    print(f"  -> {top_lf_file} (top-level federated reactor)")

    # Coordinator package: runs lfc on the federated .lf, builds all per-federate binaries
    _emit_coordinator_package(
        pkg_dir=fed_pkg_dir,
        pkg_name=fed_pkg_name,
        system_stem=system_stem,
        federated_reactor_name=federated_reactor_name,
        federate_names=fmap.federate_names,
        federate_launch_stems=federate_launch_stems,
        federate_dep_info=federate_dep_info,
        lf_file=top_lf_file,
        coordination=fmap.coordination,
    )
    print(f"  -> {fed_pkg_dir}/ (federation coordinator package)")

    _emit_federated_trace_manifest(system_json=system_json, parts=fed_trace_parts)
    return 0


FEDERATED_RESOLVED_MODEL = "resolved_model_federated.json"


def _xcorr_link_after_ns(delay_ns: int, coordination: str) -> int | None:
    """The `after` of an admitted edge's record link across federates.

    Records are sent at the publishing tag over the link's own d. A
    flush edge sends at t + 1 ns, so its link is d - 1 ns and the records
    still land at t + d. A
    centralized d = 0 link is a plain zero-delay connection (``None``, no
    ``after``), so the records arrive at their publication tag (t, m) as in
    one process; the binding's ingress files them there (``set_zero_link``).
    A decentralized d = 0 link keeps ``after 0`` and (t, m + 1).
    """
    from colcon_lf.generation.lf_generator import xfed_flush
    if xfed_flush(delay_ns, coordination == "decentralized"):
        return delay_ns - 1
    if delay_ns == 0 and coordination != "decentralized":
        return None
    return delay_ns


def _federated_link_facts(cross_connections, xcorr_global, xcorr_by_global,
                          instance_to_federate, key_to_instance, fmap):
    """``(cross_links, network_links)`` of a federation.

    ``cross_links`` has one row per directed channel across federates, in the
    form :func:`colcon_lf.resolved_model.resolve.federate_link_records` takes.
    The delay d comes from ``FederationMap.after_ns_for``: an admitted edge
    holds it as ``delay_ns`` and runs ``after d - 1 ns``, a legacy
    link runs ``federate_after_ns``. ``network_links[federate]`` are the rows
    the federate registers (``readback_links.link_rows``): carrier
    ``network`` where the link arrives, ``network-out`` where it leaves, with
    the local correspondence edge id of an admitted link. An override is
    cited only when it names the exact ports the delay was looked up by."""
    from colcon_lf.federation.partition import DEFAULT_CROSS_FEDERATE_AFTER_NS
    from colcon_lf.generation.lf_generator import xcorr_port_name, xfed_flush
    from colcon_lf.resolved_model.model import (
        PROVENANCE_EXPLICITLY_CONFIGURED, PROVENANCE_EXTRACTED,
    )

    inst_to_key = {inst: key for key, inst in key_to_instance.items()}

    def source(ports, delay_ns: int) -> tuple[str, str]:
        # The override after_ns_for applied: the one naming exactly these ports.
        for ov in fmap.after_overrides:
            if (ov.from_port, ov.to_port) == ports:
                where = f"{fmap.source}: " if fmap.source else ""
                return (PROVENANCE_EXPLICITLY_CONFIGURED,
                        f"{where}after_overrides {ov.from_port} -> {ov.to_port}")
        if delay_ns != DEFAULT_CROSS_FEDERATE_AFTER_NS:
            # No override: the link kept its configured in-process delay.
            return (PROVENANCE_EXPLICITLY_CONFIGURED,
                    "the link's in-process delay (--after-delay, execution.yaml "
                    "defaults.after_delay or system.json after_delay_ns)")
        return PROVENANCE_EXTRACTED, "federation.partition.DEFAULT_CROSS_FEDERATE_AFTER_NS"

    cross_links: list[dict] = []
    network_links: dict[str, list[dict]] = {}
    seen_edges: set[int] = set()
    for conn in cross_connections:
        src_inst, src_port = conn.source.split(".", 1)
        dst_inst, dst_port = conn.destination.split(".", 1)
        src_fed = instance_to_federate.get(src_inst, "")
        dst_fed = instance_to_federate.get(dst_inst, "")
        if not src_fed or not dst_fed:
            continue
        if src_port in xcorr_global:
            edge, local_ids = xcorr_by_global[xcorr_global[src_port]]
            if edge.edge_id in seen_edges:
                continue
            seen_edges.add(edge.edge_id)
            link = ("topic", edge.pub_node, edge.sub_node, edge.topic)
            after_ns = network_ns = edge.delay_ns
            prov, evidence = source(edge.link_ports, edge.delay_ns)
            network_evidence = evidence  # sent at the publishing tag
            if xfed_flush(edge.delay_ns, fmap.coordination == "decentralized"):
                network_ns = edge.delay_ns - 1
                network_evidence = (f"{evidence}, minus 1 ns: a flush edge's records "
                                    "leave at t + 1 ns")
            in_port, in_edge = xcorr_port_name(local_ids[dst_fed]), local_ids[dst_fed]
            out_edge = local_ids[src_fed]
        elif conn.link is not None:
            link = conn.link
            after_ns = network_ns = conn.federate_after_ns
            if after_ns is None:
                after_ns = network_ns = DEFAULT_CROSS_FEDERATE_AFTER_NS
            # A fan-in link cites the override of the subscriber's own port (P17h).
            user_dst = getattr(conn, "user_destination", None) or conn.destination
            prov, evidence = source((conn.source, user_dst), after_ns)
            network_evidence = evidence
            in_port, in_edge, out_edge = dst_port, -1, -1
        else:
            continue
        kind, src, dst, name = link
        src, dst = inst_to_key.get(src_inst, src), inst_to_key.get(dst_inst, dst)
        cross_links.append({
            "kind": kind, "src": src, "dst": dst, "name": name, "after_ns": after_ns,
            "network_ns": network_ns, "provenance": prov, "evidence": evidence,
            "network_evidence": network_evidence, "admitted": in_edge >= 0,
        })
        row = {"kind": kind, "src": src, "dst": dst, "name": name,
               "src_instance": src_inst, "dst_instance": dst_inst}
        network_links.setdefault(dst_fed, []).append(
            {**row, "carrier": "network", "dst_port": in_port, "edge": in_edge})
        network_links.setdefault(src_fed, []).append(
            {**row, "carrier": "network-out", "dst_port": "", "edge": out_edge})
    return cross_links, network_links


def _stage_federated_resolved_model(system_json: Path, cross_links: list[dict]) -> None:
    """``resolved_model_federated.json``, the resolved model with the
    link delays the federated program runs. The unfederated program of the same
    analysis keeps ``resolved_model.json``."""
    from colcon_lf.resolved_model.model import ResolvedModel, finalize, from_json, to_json
    from colcon_lf.resolved_model.resolve import federate_link_records

    base = system_json.parent / "resolved_model.json"
    target = system_json.parent / FEDERATED_RESOLVED_MODEL
    if not base.is_file():
        target.unlink(missing_ok=True)
        return
    model = from_json(base.read_text(encoding="utf-8"))
    fed = finalize(ResolvedModel(
        system=model.system, claim_status=model.claim_status,
        records=federate_link_records(model.records, cross_links),
        mutation_channels=model.mutation_channels, inputs=model.inputs,
        schema=model.schema, schema_version=model.schema_version,
    ))
    target.write_text(to_json(fed), encoding="utf-8")
    print(f"  -> {target} ({len(cross_links)} link(s) across federates)")


def _emit_federated_trace_manifest(*, system_json: Path, parts) -> str:
    """One static trace manifest for the whole federation (every
    federate binary an expected producer, each edge row naming its federate),
    and one ``rcllf_trace_build_info`` source per federate sub-package, all
    carrying the same manifest hash. Returns the manifest hash."""
    import hashlib

    from colcon_lf.generation.trace_manifest import (
        BUILD_INFO_CPP,
        FEDERATED_MANIFEST_NAME,
        build_static_manifest,
        emit_build_info_cpp,
        find_provider_marker,
        generated_source_hash,
        write_static_manifest,
    )

    model_hash = None
    resolved_model_path = system_json.parent / FEDERATED_RESOLVED_MODEL
    if resolved_model_path.is_file():
        from colcon_lf.resolved_model.model import from_json

        model_hash = from_json(resolved_model_path.read_text(encoding="utf-8")).model_hash
    source_hashes = {name: generated_source_hash(pkg_dir) for name, pkg_dir, _e, _r in parts}
    combined = hashlib.sha256(
        "".join(f"{n}:{h}\n" for n, h in sorted(source_hashes.items())).encode("utf-8")
    ).hexdigest()
    doc = build_static_manifest(
        executables=[f"federate__{name}" for name, _p, _e, _r in parts],
        corr_edges=[], rid_table=[], model_hash=model_hash,
        generated_source_hash=combined, provider_marker=find_provider_marker(),
    )
    doc["federated"] = True
    for name, _pkg_dir, edges, rid_table in parts:
        part = build_static_manifest(
            executables=[], corr_edges=edges, rid_table=rid_table, model_hash=None,
            generated_source_hash="", provider_marker=None,
        )
        doc["edges"] += [dict(e, federate=name) for e in part["edges"]]
        doc["rids"] += [dict(r, federate=name) for r in part["rids"]]
    manifest_path = system_json.parent / FEDERATED_MANIFEST_NAME
    manifest_hash = write_static_manifest(doc, manifest_path)
    for name, pkg_dir, _e, _r in parts:
        (pkg_dir / "src" / BUILD_INFO_CPP).write_text(
            emit_build_info_cpp(manifest_hash, source_hashes[name], model_hash), encoding="utf-8",
        )
    print(f"  -> {manifest_path} (federated, sha256 {manifest_hash[:12]})")
    return manifest_hash


def _emit_federate_launch_package(
    *,
    pkg_dir: Path,
    pkg_name: str,
    coord_pkg_name: str,
    fed_name: str,
    lf_stem: str,
    system_json: Path | None,
    all_dep_packages: list,
    exec_config_json_path: Path | None = None,
    fed_lf=None,
    runtime_offsets_table: list | None = None,
    launch_lf_args: list[str] | None = None,
) -> None:
    """Emit a minimal per-federate package: package.xml + launch file only.

    The binary is built and installed by the coordinator CMakeLists.txt.
    The per-federate launch file references the binary from the coordinator
    package. ``exec_config_json_path``, when given, is the per-federate copy
    of the WHOLE resolved exec_config.json (00_design.md section 2.4 S6) —
    installed and wired into ``additional_env`` alongside SYSTEM_JSON, same
    idiom, new key ``ROS2_LF_EXEC_CONFIG``.
    """
    import textwrap as _tw

    _hardcoded = {"rclcpp", "rclcpp_components", "ros2_lf_binding"}
    dep_xml = "\n".join(f"  <depend>{p}</depend>" for p in all_dep_packages if p not in _hardcoded)
    pkg_xml = (
        '<?xml version="1.0"?>\n'
        '<?xml-model href="http://download.ros.org/schema/package_format3.xsd"\n'
        '    schematypens="http://www.w3.org/2001/XMLSchema"?>\n'
        f'<package format="3">\n'
        f'  <name>{pkg_name}</name>\n'
        f'  <version>0.1.0</version>\n'
        f'  <description>Federation federate {fed_name} — launch and system.json only.</description>\n'
        f'  <maintainer email="harun.teper@cps.rwth-aachen.de">colcon-lf</maintainer>\n'
        f'  <license>Apache-2.0</license>\n'
        f'\n'
        f'  <buildtool_depend>ament_cmake</buildtool_depend>\n'
        f'  <depend>rclcpp</depend>\n'
        f'  <depend>rclcpp_components</depend>\n'
        f'  <depend>ros2_lf_binding</depend>\n'
        f'{dep_xml}\n'
        f'\n'
        f'  <export>\n'
        f'    <build_type>ament_cmake</build_type>\n'
        f'  </export>\n'
        f'</package>\n'
    )
    (pkg_dir / "package.xml").write_text(pkg_xml, encoding="utf-8")

    # Minimal CMakeLists — just install launch/ and system.json (binary comes from coordinator)
    cmake_lines = [
        f"cmake_minimum_required(VERSION 3.19)",
        f"project({pkg_name})",
        f"",
        f"find_package(ament_cmake REQUIRED)",
        f"",
        f"install(DIRECTORY launch/",
        f"    DESTINATION share/${{PROJECT_NAME}}/launch",
        f")",
    ]
    if system_json is not None:
        cmake_lines += [
            f"if(EXISTS \"${{CMAKE_CURRENT_SOURCE_DIR}}/system.json\")",
            f"    install(FILES system.json",
            f"        DESTINATION share/${{PROJECT_NAME}}",
            f"    )",
            f"endif()",
        ]
    if exec_config_json_path is not None:
        cmake_lines += [
            f"if(EXISTS \"${{CMAKE_CURRENT_SOURCE_DIR}}/exec_config.json\")",
            f"    install(FILES exec_config.json",
            f"        DESTINATION share/${{PROJECT_NAME}}",
            f"    )",
            f"endif()",
        ]
    cmake_lines += ["", "ament_package()"]
    (pkg_dir / "CMakeLists.txt").write_text("\n".join(cmake_lines) + "\n", encoding="utf-8")

    launch_dir = pkg_dir / "launch"
    launch_dir.mkdir(parents=True, exist_ok=True)

    # The binary federate__<fed_name> is installed by the coordinator package.
    # lfc names federates as "federate__<instance_name>" where instance_name = fed_name.
    binary_name = f"federate__{fed_name}"

    needs_share_dir_import = system_json is not None or exec_config_json_path is not None

    if system_json is not None:
        system_json_expr = (
            f"os.path.join(get_package_share_directory('{pkg_name}'), 'system.json')"
        )
    else:
        system_json_expr = 'os.environ.get("SYSTEM_JSON", "")'

    if exec_config_json_path is not None:
        exec_config_expr = (
            f"os.path.join(get_package_share_directory('{pkg_name}'), 'exec_config.json')"
        )
    else:
        exec_config_expr = 'os.environ.get("ROS2_LF_EXEC_CONFIG", "")'

    extra_import = (
        "from ament_index_python.packages import get_package_share_directory\n"
        if needs_share_dir_import else ""
    )

    if fed_lf is None:
        from colcon_lf.generation.lf_model import LFSystem

        fed_lf = LFSystem(name=lf_stem)
    offsets_stmt, offsets_expr = _timer_offsets_launch(
        fed_lf, runtime_offsets_table or [], own=True)
    launch_text = _tw.dedent(f"""\
        # Auto-generated by colcon lf-generate --federate. DO NOT EDIT.
        # Standalone launch for federate {fed_name}. Coordination mode (centralized
        # or decentralized) is fixed at generation time by --coordination; see the
        # federated .lf target property for which one this build used.
        # Reads RTI_HOST/RTI_PORT from env so this federate can be started
        # independently against an externally-managed RTI (e.g. the `rti`
        # compose service or another running RTI).
        import os
        import runpy
        import shlex
        from launch import LaunchDescription
        from launch.actions import RegisterEventHandler
        from launch.event_handlers import OnProcessExit
        from launch_ros.actions import Node
        {extra_import}

        def _exit_fails_the_launch(event, context):
            # A federate that exits nonzero (a failed
            # reconcile, a startup refusal, the stop bound) fails this launch,
            # so `ros2 launch` exits 1 and a run check cannot miss it. Without
            # it the launch exits 0 whatever its processes returned.
            if event.returncode != 0:
                raise RuntimeError(
                    f"rcllf: federate {fed_name} exited {{event.returncode}}, the run failed")


        def generate_launch_description():
            system_json = {system_json_expr}
            exec_config = {exec_config_expr}
            {offsets_stmt}
            rti_host = os.environ.get("RTI_HOST", "localhost")
            rti_port = os.environ.get("RTI_PORT", "15045")
            federation_id = os.environ.get("FEDERATION_ID", "")

            args = ["-r", f"{{rti_host}}:{{rti_port}}"]
            if federation_id:
                args += ["-i", federation_id]
            args += {offsets_expr}
            # Reactor-c arguments. The generated default
            # ({list(launch_lf_args or [])!r} for --fast), then RCLLF_LF_ARGS
            # at launch time, e.g. "-f true -o 20 sec" (fast mode, logical
            # timeout); reactor-c takes the last -f, so the env var wins.
            args += {list(launch_lf_args or [])!r}
            args += shlex.split(os.environ.get("RCLLF_LF_ARGS", ""))

            federate = Node(
                package="{coord_pkg_name}",
                executable="{binary_name}",
                output="screen",
                arguments=args,
                # SIGTERM when ros2 launch dies, which it does
                # on a SIGTERM without stopping its processes. The pid is
                # this launch process's, evaluated here, not in the child.
                additional_env={{"SYSTEM_JSON": system_json, "ROS2_LF_EXEC_CONFIG": exec_config,
                                "RCLLF_EXIT_WITH_PARENT": str(os.getpid())}},
            )
            return LaunchDescription([
                federate,
                RegisterEventHandler(OnProcessExit(
                    target_action=federate, on_exit=_exit_fails_the_launch)),
            ])
    """)
    (launch_dir / f"lf_{lf_stem.lower()}.launch.py").write_text(launch_text, encoding="utf-8")
    _copy_timer_offsets_helper(launch_dir)


def _emit_coordinator_package(
    *,
    pkg_dir: Path,
    pkg_name: str,
    system_stem: str,
    federated_reactor_name: str,
    federate_names: list,
    federate_launch_stems: dict,
    federate_dep_info: dict,
    lf_file: Path,
    coordination: str,
) -> None:
    """Emit coordinator package: runs lfc on the federated .lf and builds all per-federate binaries.

    ``coordination`` is required and must be one of VALID_COORDINATION_MODES:
    with a default, a caller that forgot the mode got
    the centralized header for a decentralized build, the symptom.
    """
    import textwrap

    from colcon_lf.federation.yaml_schema import (
        COORDINATION_CENTRALIZED, COORDINATION_DECENTRALIZED, VALID_COORDINATION_MODES)

    if coordination not in VALID_COORDINATION_MODES:
        raise ValueError(
            f"coordinator package: unknown coordination {coordination!r} "
            f"(expected one of {', '.join(VALID_COORDINATION_MODES)})")
    # The launch header names the mode this build was generated for.
    if coordination == COORDINATION_DECENTRALIZED:
        rti_role = (
            "# is started for LF decentralized coordination: the federates talk peer\n"
            "        # to peer and advance their tags on STP offsets, the RTI coordinates\n"
            "        # startup and stop (generated with --coordination decentralized)."
        )
    else:
        assert coordination == COORDINATION_CENTRALIZED
        rti_role = (
            "# is the time authority for LF centralized coordination: it grants every\n"
            "        # cross-federate tag advance (generated with --coordination centralized)."
        )

    # Collect union of all deps across all federates
    all_packages: set[str] = set()
    for info in federate_dep_info.values():
        all_packages.update(info["all_dep_packages"])
    _hardcoded = {"rclcpp", "rclcpp_components", "ros2_lf_binding"}

    dep_find = "\n".join(
        f"find_package({p} REQUIRED)"
        for p in sorted(all_packages) if p not in _hardcoded
    )
    dep_include_dirs = "\n            ".join(
        f"${{{p}_INCLUDE_DIRS}}"
        for p in sorted(all_packages) if p not in _hardcoded
    )
    dep_libraries = "\n            ".join(
        f"${{{p}_LIBRARIES}}"
        for p in sorted(all_packages) if p not in _hardcoded
    )

    dep_xml = "\n".join(
        f"  <depend>{p}</depend>"
        for p in sorted(all_packages)
        if p not in _hardcoded
    )
    pkg_xml = (
        '<?xml version="1.0"?>\n'
        '<?xml-model href="http://download.ros.org/schema/package_format3.xsd"\n'
        '    schematypens="http://www.w3.org/2001/XMLSchema"?>\n'
        f'<package format="3">\n'
        f'  <name>{pkg_name}</name>\n'
        f'  <version>0.1.0</version>\n'
        f'  <description>Federation coordinator for {system_stem} — runs lfc and builds all federate binaries.</description>\n'
        f'  <maintainer email="harun.teper@cps.rwth-aachen.de">colcon-lf</maintainer>\n'
        f'  <license>Apache-2.0</license>\n'
        f'\n'
        f'  <buildtool_depend>ament_cmake</buildtool_depend>\n'
        f'  <depend>rclcpp</depend>\n'
        f'  <depend>rclcpp_components</depend>\n'
        f'  <depend>ros2_lf_binding</depend>\n'
        f'{dep_xml}\n'
        f'\n'
        f'  <export>\n'
        f'    <build_type>ament_cmake</build_type>\n'
        f'  </export>\n'
        f'</package>\n'
    )
    (pkg_dir / "package.xml").write_text(pkg_xml, encoding="utf-8")

    # ----- cmake-include dispatch file -----
    # The federated .lf's target CCpp { cmake-include: ... } propagates to each
    # per-federate .lf that lfc generates internally. We write ONE dispatch file
    # that branches on ${LF_MAIN_TARGET} (e.g. "federate__publisher_node") to
    # apply the right binding sources / includes / libraries to each federate.
    # Note: lfc owns the file `<binary>_extension.cmake` — that name is reserved.
    # We use a different name (`ros2_lf_bindings.cmake`) to avoid the collision.
    lf_include_dir = pkg_dir / "lf" / "include"
    lf_include_dir.mkdir(parents=True, exist_ok=True)

    dispatch_lines = [
        "# Auto-generated by colcon lf-generate --federate. DO NOT EDIT.",
        "# lfc applies this file (via cmake-include in the top-level federated .lf)",
        "# to every per-federate target. We dispatch on ${LF_MAIN_TARGET} to apply",
        "# the binding sources / includes / link libraries for each federate.",
        "",
    ]

    # Hardcoded packages we always need (not per-federate); avoid duplicates.
    _hardcoded_fed = {"rclcpp", "rclcpp_components", "ros2_lf_binding"}

    # Collect all unique find_package calls — these are safe to emit unconditionally.
    all_fed_deps: set[str] = set()
    for info in federate_dep_info.values():
        all_fed_deps.update(info.get("all_dep_packages", []))
    dispatch_lines.append("# Common dependency find_package calls")
    dispatch_lines.append("find_package(rclcpp REQUIRED)")
    dispatch_lines.append("find_package(rclcpp_components REQUIRED)")
    dispatch_lines.append("find_package(ros2_lf_binding REQUIRED)")
    for p in sorted(all_fed_deps):
        if p not in _hardcoded_fed:
            dispatch_lines.append(f"find_package({p} REQUIRED)")
    dispatch_lines.append("")
    dispatch_lines.append("# Enable C++ for the lfc target so .cpp binding sources compile.")
    dispatch_lines.append("enable_language(CXX)")
    dispatch_lines.append("set_target_properties(${LF_MAIN_TARGET} PROPERTIES")
    dispatch_lines.append("    LINKER_LANGUAGE CXX")
    dispatch_lines.append("    CXX_STANDARD 17")
    dispatch_lines.append(")")
    dispatch_lines.append("")

    # Per-federate branches
    for idx, fed_name in enumerate(federate_names):
        info = federate_dep_info.get(fed_name, {})
        staged_sources = info.get("staged_sources", [])
        sub_pkg_name = f"{pkg_name}_{fed_name}"
        sub_pkg_src_abs = str((lf_file.parent.parent.parent / sub_pkg_name).resolve())
        binary = f"federate__{fed_name}"
        # Absolute path to per-federate include dir (RID header lives here).
        fed_include_dir = str((pkg_dir / "lf" / fed_name / "include").resolve())
        # Drop hardcoded packages — those are find_package'd unconditionally above.
        fed_deps = sorted(p for p in info.get("all_dep_packages", []) if p not in _hardcoded_fed)

        nc_src_lines = "\n        ".join(
            f"{sub_pkg_src_abs}/{rel_path}"
            for _, _, rel_path, _ in staged_sources
        )
        nc_src_block = f"\n        {nc_src_lines}" if nc_src_lines else ""
        dep_incl_lines = "\n        ".join(f"${{{p}_INCLUDE_DIRS}}" for p in fed_deps)
        dep_libs_lines = "\n        ".join(f"${{{p}_LIBRARIES}}" for p in fed_deps)

        keyword = "if" if idx == 0 else "elseif"
        dispatch_lines.append(f'{keyword}(LF_MAIN_TARGET STREQUAL "{binary}")')
        dispatch_lines.append(f"    target_sources(${{LF_MAIN_TARGET}} PRIVATE")
        dispatch_lines.append(f"        {sub_pkg_src_abs}/src/ros2_lf_startup.cpp")
        dispatch_lines.append(f"        {sub_pkg_src_abs}/src/ros2_lf_reaction_ids_init.cpp")
        dispatch_lines.append(f"        {sub_pkg_src_abs}/src/node_factories.cpp")
        # The federate's build facts for its rcllf run_begin.
        dispatch_lines.append(f"        {sub_pkg_src_abs}/src/rcllf_trace_build_info.cpp{nc_src_block}")
        # The rcl-level publication gate lives in the executable.
        dispatch_lines.append("        ${ros2_lf_binding_DIR}/../src/publication_gate.cpp")
        # The rclcpp clock gate lives in the executable too.
        dispatch_lines.append("        ${ros2_lf_binding_DIR}/../src/clock_gate.cpp")
        # So does the rcl_action send and take gate.
        dispatch_lines.append("        ${ros2_lf_binding_DIR}/../src/action_gate.cpp")
        dispatch_lines.append(f"    )")
        dispatch_lines.append(f"    target_include_directories(${{LF_MAIN_TARGET}} PRIVATE")
        dispatch_lines.append(f"        {fed_include_dir}")
        dispatch_lines.append(f"    )")
        dispatch_lines.append(f"    target_include_directories(${{LF_MAIN_TARGET}} SYSTEM PRIVATE")
        dispatch_lines.append(f"        ${{rclcpp_INCLUDE_DIRS}}")
        dispatch_lines.append(f"        ${{rclcpp_components_INCLUDE_DIRS}}")
        dispatch_lines.append(f"        ${{ros2_lf_binding_INCLUDE_DIRS}}")
        if dep_incl_lines:
            dispatch_lines.append(f"        {dep_incl_lines}")
        dispatch_lines.append(f"    )")
        # ros2_lf_startup.cpp runs the peer check in the wrapper.
        dispatch_lines.append("    target_link_options(${LF_MAIN_TARGET} PRIVATE")
        dispatch_lines.append('        "LINKER:--wrap=lf_synchronize_with_other_federates"')
        dispatch_lines.append('        "LINKER:--wrap=lf_terminate_execution"')
        dispatch_lines.append("    )")
        dispatch_lines.append(f"    target_link_libraries(${{LF_MAIN_TARGET}} PRIVATE")
        dispatch_lines.append(f"        ${{rclcpp_LIBRARIES}}")
        dispatch_lines.append(f"        ${{rclcpp_components_LIBRARIES}}")
        dispatch_lines.append(f"        ${{ros2_lf_binding_LIBRARIES}}")
        if dep_libs_lines:
            dispatch_lines.append(f"        {dep_libs_lines}")
        dispatch_lines.append(f"    )")

    if federate_names:
        dispatch_lines.append("endif()")

    dispatch_file = lf_include_dir / "ros2_lf_bindings.cmake"
    dispatch_file.write_text("\n".join(dispatch_lines) + "\n", encoding="utf-8")

    # Coordinator CMakeLists.txt — small: invoke lfc (full compile), install the binaries.
    install_lines = "\n".join(
        f'install(PROGRAMS "${{LF_GEN_DIR}}/fed-gen/{federated_reactor_name}/bin/federate__{name}"\n'
        f'    DESTINATION lib/${{PROJECT_NAME}}\n)'
        for name in federate_names
    )

    from colcon_lf.generation.lfc_runtime import cmake_runtime_check, patch_checker
    # The emitted CMake re-checks the lfc runtime at configure time.
    lfc_runtime_check = textwrap.indent(
        cmake_runtime_check(patch_checker(pkg_dir)), " " * 8).lstrip()
    cmake_text = textwrap.dedent(f"""\
        cmake_minimum_required(VERSION 3.19)
        project({pkg_name})

        find_package(ament_cmake REQUIRED)
        # find_package each dep here too so they propagate via ament_export_dependencies
        # (the per-federate extension.cmake files also call find_package, but we need
        # the runtime ROS2 env to be set up by ament when the binaries run).
        find_package(rclcpp REQUIRED)
        find_package(rclcpp_components REQUIRED)
        find_package(ros2_lf_binding REQUIRED)
        {dep_find}

        # ---------------------------------------------------------------------------
        # Run lfc on the top-level federated .lf with full compile.
        # lfc decomposes the federated reactor into per-federate sources, applies our
        # *_extension.cmake files (one per federate) which add binding sources, then
        # builds each federate as an isolated standalone CMake project.
        # ---------------------------------------------------------------------------
        set(LFC_BIN "$ENV{{LFC}}")
        if("${{LFC_BIN}}" STREQUAL "")  # a string test: LFC=OFF names a program
            set(LFC_BIN "/opt/lfc/bin/lfc")
        endif()
        {lfc_runtime_check}

        # Stage lf/ into the build dir so lfc resolves imports relative to the top-level file.
        # lfc writes fed-gen/ relative to its WORKING_DIRECTORY.
        set(LF_GEN_DIR "${{CMAKE_CURRENT_BINARY_DIR}}/lf-src")
        execute_process(COMMAND ${{CMAKE_COMMAND}} -E make_directory ${{LF_GEN_DIR}})
        execute_process(
            COMMAND ${{CMAKE_COMMAND}} -E copy_directory
                "${{CMAKE_CURRENT_SOURCE_DIR}}/lf"
                "${{LF_GEN_DIR}}"
        )

        set(LF_SRC_STAGED "${{LF_GEN_DIR}}/{federated_reactor_name}.lf")
        # LFC_BIN is the file the runtime check judged, quoted: an unquoted
        # expansion splits a path at a semicolon and runs another program.
        # Full compile (no --no-compile): lfc builds each federate as a standalone
        # CMake project. Our cmake-include dispatch file (lf/include/ros2_lf_bindings.cmake)
        # is applied to each federate via the federated reactor's target options.
        #
        # lfc's sub-cmake invocations need CMAKE_PREFIX_PATH set so find_package(rclcpp etc.)
        # resolves. CMAKE_PREFIX_PATH is a cmake variable (a ;-separated list at this scope)
        # — convert it to ;-separated for env propagation. Also include AMENT_PREFIX_PATH
        # (colon-separated) as a fallback for ROS2 packages.
        string(REPLACE ";" ":" _LF_CMAKE_PREFIX_PATH "${{CMAKE_PREFIX_PATH}}")
        set(_LF_AMENT_PATH "$ENV{{AMENT_PREFIX_PATH}}")
        if(_LF_AMENT_PATH)
            set(_LF_FULL_PREFIX "${{_LF_CMAKE_PREFIX_PATH}}:${{_LF_AMENT_PATH}}")
        else()
            set(_LF_FULL_PREFIX "${{_LF_CMAKE_PREFIX_PATH}}")
        endif()
        message(STATUS "[lf] CMAKE_PREFIX_PATH for lfc: ${{_LF_FULL_PREFIX}}")
        execute_process(
            COMMAND ${{CMAKE_COMMAND}} -E env
                "CMAKE_PREFIX_PATH=${{_LF_FULL_PREFIX}}"
                "AMENT_PREFIX_PATH=${{_LF_FULL_PREFIX}}"
                "${{LFC_BIN}}" ${{LF_SRC_STAGED}}
            WORKING_DIRECTORY ${{LF_GEN_DIR}}
            RESULT_VARIABLE LFC_RESULT
            OUTPUT_VARIABLE LFC_OUTPUT
            ERROR_VARIABLE LFC_OUTPUT
        )
        if(NOT LFC_RESULT EQUAL 0)
            message(FATAL_ERROR "lfc failed:\\n${{LFC_OUTPUT}}")
        endif()

        # Install the per-federate binaries lfc produced.
        {install_lines}

        install(DIRECTORY launch/
            DESTINATION share/${{PROJECT_NAME}}/launch
        )
        install(DIRECTORY lf/
            DESTINATION share/${{PROJECT_NAME}}/lf
        )

        ament_package()
    """)
    (pkg_dir / "CMakeLists.txt").write_text(cmake_text, encoding="utf-8")

    # Orchestrator launch file
    launch_dir = pkg_dir / "launch"
    launch_dir.mkdir(parents=True, exist_ok=True)

    fed_node_launches = "\n".join(
        f"        IncludeLaunchDescription(\n"
        f"            PythonLaunchDescriptionSource([\n"
        f"                FindPackageShare('{pkg_name}_{f}'),\n"
        f"                '/launch/lf_{federate_launch_stems[f].lower()}.launch.py'\n"
        f"            ])\n"
        f"        ),"
        for f in federate_names
    )

    launch_text = textwrap.dedent(f"""\
        # Auto-generated by colcon lf-generate --federate. DO NOT EDIT.
        # Orchestrator: starts RTI daemon, then launches each federate. The RTI
        {rti_role}
        # The RTI runs under rti_guard.py: a Ctrl-C stops the
        # federates through their LF stop request and the RTI then ends with
        # code 0 instead of exiting 2 on the SIGINT.
        import os
        import sys
        from launch import LaunchDescription
        from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
        from launch.event_handlers import OnProcessExit
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch_ros.substitutions import FindPackageShare


        def _rti_exit_fails_the_launch(event, context):
            # The guard exits nonzero when the RTI lost a
            # federate (4) or failed; the launch fails with it. Each included
            # federate launch fails on its own federate's exit the same way.
            if event.returncode != 0:
                raise RuntimeError(f"rcllf: the RTI exited {{event.returncode}}, the run failed")


        def generate_launch_description():
            rti_host = os.environ.get("RTI_HOST", "")
            rti_port = os.environ.get("RTI_PORT", "15045")

            actions = []
            if not rti_host:
                guard = os.path.join(os.path.dirname(os.path.realpath(__file__)), "rti_guard.py")
                rti = ExecuteProcess(
                    cmd=[sys.executable, guard, "RTI", "-n", "{len(federate_names)}", "-p", rti_port],
                    name="RTI",
                    output="screen",
                    # Room for the federates' LF shutdown before SIGTERM.
                    sigterm_timeout="10",
                    # The guard dies with this launch process.
                    additional_env={{"RCLLF_EXIT_WITH_PARENT": str(os.getpid())}},
                )
                actions.append(rti)
                actions.append(RegisterEventHandler(OnProcessExit(
                    target_action=rti, on_exit=_rti_exit_fails_the_launch)))

            actions += [
{fed_node_launches}
            ]
            return LaunchDescription(actions)
    """)
    (launch_dir / f"lf_{system_stem.lower()}_federated.launch.py").write_text(
        launch_text, encoding="utf-8"
    )
    import shutil
    from colcon_lf.federation import rti_guard
    shutil.copyfile(rti_guard.__file__, launch_dir / "rti_guard.py")


# ---------------------------------------------------------------------------
# lfc artifact helpers
# ---------------------------------------------------------------------------

def _write_correspondence_json(pkg_dir: Path, corr_edges) -> None:
    """Write the topic edge table next to the package's package.xml."""
    import json

    pkg_dir.mkdir(parents=True, exist_ok=True)
    rows = [e.to_json() for e in corr_edges]
    (pkg_dir / "correspondence.json").write_text(
        json.dumps(rows, indent=2) + "\n", encoding="utf-8",
    )


def _emit_lfc_artifacts(
    *,
    auto_transition: dict | None = None,
    system_json: Path,
    out_dir: Path,
    lf_system,
    graph,
    foreign_executor_acks: list[tuple[str, str]] | None = None,
    tf2_configs: list[tuple[str, int, str]] | None = None,
    external_topics: set[str] | None = None,
    external_anchors: dict | None = None,
    timer_period_overrides: dict[str, dict[str, float]] | None = None,
    launch_lf_args: list[str] | None = None,
) -> str:
    """Emit the reaction-ID header, init cpp, and lfc-specific package skeleton.

    Returns the ``lf_generated_<package>_<name>`` package name actually used,
    so callers record the same name lf-build/ros2-lf discover later.
    ``timer_period_overrides`` is the one *lf_system* was generated with.
    """
    from colcon_lf.generation.binding_header_emitter import (
        collect_publisher_qos,
        collect_rid_table,
        emit_reaction_ids_header,
        emit_rid_init_cpp,
    )
    from colcon_lf.generation.binding_key import build_binding_keys
    from colcon_lf.generation.node_identity import build_node_identities

    # Per-instance component list into system.json — the runtime reads it there
    # and loads one component per entry, keyed by binding_key.
    _stage_component_instances(
        system_json=system_json,
        graph=graph,
        binding_keys=build_binding_keys(graph.nodes),
        identities=build_node_identities(graph),
    )

    # Build class_to_reactor map from the already-generated lf_system
    class_to_reactor = {r.name: r for r in lf_system.reactors}

    rid_table = collect_rid_table(
        class_to_reactor, graph, timer_period_overrides=timer_period_overrides,
    )
    header_text = emit_reaction_ids_header(rid_table)

    include_dir = out_dir / "include"
    include_dir.mkdir(parents=True, exist_ok=True)
    header_file = include_dir / "ros2_lf_reaction_ids.h"
    header_file.write_text(header_text, encoding="utf-8")
    print(f"  -> {header_file}")

    # Generated colcon package skeleton: lf_generated_<package>_{stem}/
    # Emit to packages/lf-generated/ (sibling of lf-analysis/) rather than
    # inside lf-analysis/ so it never touches the user's src/ directory.
    # Package-qualified: source_package comes from the standard
    # lf-analysis/<package>/<stem>/system.json layout (see _output_path in
    # verb/analyze.py) — system_json.parent is the stem dir, .parent.parent
    # the package dir.
    from colcon_lf.generation.lf_naming import (
        generated_lfc_package_name, generated_package_stem,
    )

    stem = generated_package_stem(lf_system.name)
    source_package = system_json.parent.parent.name
    pkg_name = generated_lfc_package_name(source_package, stem)
    lf_analysis_root = system_json.parent.parent.parent  # e.g. packages/lf-analysis/
    lf_generated_root = lf_analysis_root.parent / "lf-generated"
    lf_generated_root.mkdir(parents=True, exist_ok=True)
    pkg_dir = lf_generated_root / pkg_name

    # Emit ros2_lf_reaction_ids_init.cpp into the package src/ dir
    src_dir = pkg_dir / "src"
    src_dir.mkdir(parents=True, exist_ok=True)
    init_cpp_text = emit_rid_init_cpp(
        rid_table, pkg_name, corr_edges=getattr(lf_system, "corr_edges", None),
        foreign_executor_acks=foreign_executor_acks,
        tf2_configs=tf2_configs,
        publisher_qos=collect_publisher_qos(graph),
        lf_system=lf_system,
    )
    init_cpp_file = src_dir / "ros2_lf_reaction_ids_init.cpp"
    init_cpp_file.write_text(init_cpp_text, encoding="utf-8")
    print(f"  -> {init_cpp_file}")
    _write_correspondence_json(pkg_dir, getattr(lf_system, "corr_edges", None) or [])

    # Bundle the .lf file and RID header inside the package so it's self-contained.
    import shutil
    lf_file_src = out_dir / f"{lf_system.name}.lf"
    pkg_lf_dir = pkg_dir / "lf"
    pkg_lf_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(lf_file_src, pkg_lf_dir / lf_file_src.name)
    pkg_lf_include_dir = pkg_lf_dir / "include"
    pkg_lf_include_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(header_file, pkg_lf_include_dir / "ros2_lf_reaction_ids.h")
    # Install system.json and exec_config.json with the package, so the
    # launch file finds them through the share directory from any cwd. A
    # relative lf-analysis/ path read from the wrong cwd once loaded 0
    # components. The copies match the RID table built above.
    shutil.copy2(system_json, pkg_dir / "system.json")
    staged_exec_config = system_json.parent / "exec_config.json"
    if staged_exec_config.is_file():
        shutil.copy2(staged_exec_config, pkg_dir / "exec_config.json")

    _emit_lfc_package(
        auto_transition=auto_transition,
        pkg_dir=pkg_dir,
        pkg_name=pkg_name,
        lf_system=lf_system,
        graph=graph,
        lf_file=out_dir / f"{lf_system.name}.lf",
        system_json=system_json,
        external_topics=external_topics,
        external_anchors=external_anchors,
        launch_lf_args=launch_lf_args,
    )
    _emit_trace_manifest(
        system_json=system_json, pkg_dir=pkg_dir, lf_system=lf_system, rid_table=rid_table,
    )
    print(f"  -> {pkg_dir}/ (lfc package)")
    return pkg_name


def _emit_trace_manifest(*, system_json: Path, pkg_dir: Path, lf_system, rid_table) -> str:
    """Static trace manifest next to resolved_model.json, and the
    executable's ``rcllf_trace_build_info`` source. Returns the manifest hash."""
    from colcon_lf.generation.trace_manifest import (
        BUILD_INFO_CPP,
        STATIC_MANIFEST_NAME,
        build_static_manifest,
        emit_build_info_cpp,
        find_provider_marker,
        generated_source_hash,
        write_static_manifest,
    )

    model_hash = None
    resolved_model_path = system_json.parent / "resolved_model.json"
    if resolved_model_path.is_file():
        from colcon_lf.resolved_model.model import from_json

        model_hash = from_json(resolved_model_path.read_text(encoding="utf-8")).model_hash
    source_hash = generated_source_hash(pkg_dir)
    doc = build_static_manifest(
        executables=[lf_system.name],
        corr_edges=getattr(lf_system, "corr_edges", None) or [],
        rid_table=rid_table,
        model_hash=model_hash,
        generated_source_hash=source_hash,
        provider_marker=find_provider_marker(),
    )
    manifest_path = system_json.parent / STATIC_MANIFEST_NAME
    manifest_hash = write_static_manifest(doc, manifest_path)
    (pkg_dir / "src" / BUILD_INFO_CPP).write_text(
        emit_build_info_cpp(manifest_hash, source_hash, model_hash), encoding="utf-8",
    )
    print(f"  -> {manifest_path} (sha256 {manifest_hash[:12]})")
    return manifest_hash


def _emit_lfc_package(
    *,
    auto_transition: dict | None = None,
    pkg_dir: Path,
    pkg_name: str,
    lf_system,
    graph,
    lf_file: Path,
    system_json: Path | None = None,
    external_topics: set[str] | None = None,
    external_anchors: dict | None = None,
    launch_lf_args: list[str] | None = None,
) -> None:
    """Write the generated colcon package skeleton for lfc compilation.
    ``launch_lf_args`` are reactor-c arguments the launch file bakes in
    (``-f true`` for ``--fast``)."""
    import textwrap

    from colcon_lf.generation.lf_naming import generated_package_stem

    stem = lf_system.name

    import shutil as _shutil

    from colcon_lf.generation.binding_key import build_binding_keys
    from colcon_lf.generation.node_identity import build_node_identities

    binding_keys = build_binding_keys(graph.nodes)
    identities = build_node_identities(graph)

    component_nodes = [ni for ni in graph.nodes.values() if ni.takes_component_path]
    # (node_key, NodeInfo) so the binding key / launch identity join stays exact.
    non_component_items = [
        (key, ni) for key, ni in graph.nodes.items() if not ni.takes_component_path
    ]
    non_component_nodes = [ni for _, ni in non_component_items]

    component_packages: list[str] = sorted({ni.package for ni in component_nodes if ni.package})
    non_component_packages: list[str] = sorted({ni.package for ni in non_component_nodes if ni.package})
    extra_packages: set[str] = set()
    all_nodes = list(graph.nodes.values())
    if any(getattr(ni, "action_servers", []) or getattr(ni, "action_clients", []) for ni in all_nodes):
        extra_packages.add("rclcpp_action")
    direct_packages = set(component_packages) | set(non_component_packages) | extra_packages
    # Add transitive deps from each non-component node's package.xml so headers
    # like std_msgs/msg/string.hpp are available at compile time.
    transitive: set[str] = set()
    if system_json is not None and non_component_packages:
        from colcon_lf.generation.package_deps import _read_package_deps
        workspace_src = system_json.parent
        for _ in range(6):
            workspace_src = workspace_src.parent
            if (workspace_src / "src").is_dir():
                break
        for pkg in non_component_packages:
            for dep in _read_package_deps(workspace_src / "src", pkg):
                if dep not in direct_packages:
                    transitive.add(dep)
    all_dep_packages: list[str] = sorted(direct_packages | transitive)

    # Directories
    src_dir = pkg_dir / "src"
    launch_dir = pkg_dir / "launch"
    src_dir.mkdir(parents=True, exist_ok=True)
    launch_dir.mkdir(parents=True, exist_ok=True)

    # Stage non-component source files as pkg_dir/src/nodes/<pkg>/<basename>
    # Prepend "#define main lf_unused_main_N" directly into the staged copy so
    # the node's main() doesn't collide with lfc's generated main().
    staged_sources: list[tuple[str, str, str, int]] = []  # (class_name, pkg, rel_path, idx)

    if non_component_nodes:
        nodes_dir = src_dir / "nodes"
        nodes_dir.mkdir(parents=True, exist_ok=True)

        # Iterate the (node_key, NodeInfo) pairs, not the bare NodeInfos: the
        # key is what the per-node auto_transition map is keyed by.
        # Enumeration order is identical — non_component_nodes is built from
        # non_component_items — so staging indices are unchanged.
        staged_text: dict = {}
        staged_from: dict = {}
        for idx, (node_key, ni) in enumerate(non_component_items):
            src_file = str(ni.source_file) if ni.source_file else ""
            pkg_nodes_dir = nodes_dir / (ni.package or "unknown")
            pkg_nodes_dir.mkdir(exist_ok=True)
            dest = _staged_node_dest(pkg_nodes_dir, src_file, ni, staged_from)
            define_line = (
                _staged_define_lines(ni)
                + f"#define main lf_unused_main_{idx}\n"
                + _STAGED_MAIN_PRAGMA
            )
            # Header-less nodes (class defined directly in the .cpp, e.g. the
            # autoware_reference_launch wrappers) cannot be instantiated from
            # node_factories.cpp — there is no header to include and the class
            # may live in the global namespace. Append a factory function to
            # the staged copy, where the class definition is visible;
            # node_factories.cpp calls it through an extern declaration.
            factory_suffix = ""
            if not ni.header_file:
                define_line += _HEADERLESS_FACTORY_INCLUDES
                factory_suffix = emit_headerless_factory_suffix(
                    idx, ni.class_name,
                    qualified_name=getattr(ni, "qualified_name", "") or "",
                    auto_transition=(auto_transition or {}).get(node_key, True),
                )
            # Two instances of one class share one source file: stage it once
            # and append each instance's factory (see the federated loop).
            if dest in staged_text:
                staged_text[dest] += factory_suffix
                dest.write_text(staged_text[dest], encoding="utf-8")
                continue
            original = _stage_node_source_or_fail(src_file, ni)
            staged_text[dest] = define_line + original + factory_suffix
            dest.write_text(staged_text[dest], encoding="utf-8")
            rel_path = f"src/nodes/{ni.package or 'unknown'}/{dest.name}"
            staged_sources.append((ni.class_name, ni.package or "", rel_path, idx))

    # --- src/node_factories.cpp ---
    # Uses a named function (not __attribute__((constructor))) because nodes
    # require rclcpp::init() before construction. ros2_lf_startup.cpp calls
    # ros2_lf_register_all_nodes() after rclcpp::init() inside its constructor.
    # Registration is keyed by binding key (one entry per node *instance*), and
    # each instance is constructed with launch-model name/namespace remaps.
    if non_component_nodes:
        entries = _factory_entries(
            non_component_items, binding_keys, identities, auto_transition,
        )
        (src_dir / "node_factories.cpp").write_text(emit_node_factories_cpp(entries))
    else:
        # Emit an empty stub so CMakeLists.txt can always reference it
        (src_dir / "node_factories.cpp").write_text(emit_empty_node_factories_cpp())

    # --- CMakeLists.txt ---
    dep_find = "\n".join(
        f"find_package({p} REQUIRED)" for p in all_dep_packages
    )
    dep_ament = " ".join(all_dep_packages) if all_dep_packages else ""
    dep_include_dirs = "\n            ".join(
        f"${{{p}_INCLUDE_DIRS}}" for p in all_dep_packages
    )
    dep_libraries = "\n            ".join(
        f"${{{p}_LIBRARIES}}" for p in all_dep_packages
    )

    # Build CMake snippet for non-component node sources.
    # The #define main lf_unused_main_N is already prepended into each staged
    # source file, so no set_source_files_properties trick is needed here.
    nc_cmake_snippet = ""
    if staged_sources:
        src_lines = "\n".join(
            f"    ${{CMAKE_CURRENT_SOURCE_DIR}}/{rel_path}"
            for _, _, rel_path, _ in staged_sources
        )
        nc_cmake_snippet = (
            f"\n# Non-component node sources (#define main already prepended in staged copy)\n"
            f"target_sources({stem} PRIVATE\n{src_lines}\n)\n"
        )
    from colcon_lf.generation.lfc_runtime import cmake_runtime_check, patch_checker
    # The emitted CMake re-checks the lfc runtime at configure time.
    lfc_runtime_check = textwrap.indent(
        cmake_runtime_check(patch_checker(pkg_dir)), " " * 8).lstrip()
    cmake_text = textwrap.dedent(f"""\
        cmake_minimum_required(VERSION 3.19)
        project({pkg_name})

        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(rclcpp_components REQUIRED)
        find_package(ros2_lf_binding REQUIRED)
        {dep_find}

        # ---------------------------------------------------------------------------
        # Step 1: Run lfc to generate reactor-c sources (code-gen only, no compile)
        # ---------------------------------------------------------------------------
        set(LFC_BIN "$ENV{{LFC}}")
        if("${{LFC_BIN}}" STREQUAL "")  # a string test: LFC=OFF names a program
            set(LFC_BIN "/opt/lfc/bin/lfc")
        endif()
        {lfc_runtime_check}

        # Copy the .lf file into the build dir before invoking lfc.
        # lfc resolves its output path relative to the source file location,
        # so if the source is outside the build tree the generated path contains
        # ".." components that lfc cannot write through.  Staging the file here
        # keeps all paths inside CMAKE_CURRENT_BINARY_DIR.
        set(LF_GEN_DIR "${{CMAKE_CURRENT_BINARY_DIR}}/lf-src")
        set(LF_SRC_STAGED "${{LF_GEN_DIR}}/{stem}.lf")
        execute_process(
            COMMAND ${{CMAKE_COMMAND}} -E make_directory ${{LF_GEN_DIR}}
        )
        execute_process(
            COMMAND ${{CMAKE_COMMAND}} -E copy
                "${{CMAKE_CURRENT_SOURCE_DIR}}/lf/{stem}.lf"
                "${{LF_SRC_STAGED}}"
        )
        # LFC_BIN is the file the runtime check judged, quoted: an unquoted
        # expansion splits a path at a semicolon and runs another program.
        execute_process(
            COMMAND "${{LFC_BIN}}" --no-compile -o ${{LF_GEN_DIR}} ${{LF_SRC_STAGED}}
            RESULT_VARIABLE LFC_RESULT
        )
        if(NOT LFC_RESULT EQUAL 0)
            message(FATAL_ERROR "lfc failed with exit code ${{LFC_RESULT}}")
        endif()

        # Locate lfc's generated CMakeLists.txt — the path under src-gen/ depends
        # on how lfc resolves the .lf file location, so we glob for it.
        file(GLOB_RECURSE LF_GEN_CMAKES
            "${{LF_GEN_DIR}}/src-gen/*/CMakeLists.txt"
        )
        # Filter to the top-level one for {stem} (not lib/ or platform/ subdirs)
        set(LF_GEN_SRCS_DIR "")
        foreach(_cmake_file IN LISTS LF_GEN_CMAKES)
            get_filename_component(_dir "${{_cmake_file}}" DIRECTORY)
            get_filename_component(_name "${{_dir}}" NAME)
            if(_name STREQUAL "{stem}")
                set(LF_GEN_SRCS_DIR "${{_dir}}")
                break()
            endif()
        endforeach()
        if(NOT LF_GEN_SRCS_DIR)
            message(FATAL_ERROR "Could not find lfc-generated CMakeLists.txt for {stem} under ${{LF_GEN_DIR}}/src-gen/")
        endif()
        message(STATUS "lfc generated sources at: ${{LF_GEN_SRCS_DIR}}")

        # ---------------------------------------------------------------------------
        # Step 2: Build using lfc's own generated CMakeLists.txt (handles includes)
        #         then add our binding sources to the lfc target.
        # ---------------------------------------------------------------------------
        add_subdirectory(${{LF_GEN_SRCS_DIR}} lf-build)

        # lfc names its target after the main reactor ('{stem}')
        set(LF_TARGET {stem})

        # Add our generated init/startup files to the lfc-created target.
        # No separate main.cpp — lfc already provides main() in {stem}.cpp.
        target_sources(${{LF_TARGET}} PRIVATE
            src/ros2_lf_startup.cpp
            src/ros2_lf_reaction_ids_init.cpp
            src/node_factories.cpp
            src/rcllf_trace_build_info.cpp
        )
        # The rcl-level publication gate must be in the executable
        # (installed by ros2_lf_binding under share/ros2_lf_binding/src).
        target_sources(${{LF_TARGET}} PRIVATE ${{ros2_lf_binding_DIR}}/../src/publication_gate.cpp)
        # So does the rclcpp clock gate (ROS-time sleeps,
        # jump handlers and clock reads rcllf cannot drive from LF time).
        target_sources(${{LF_TARGET}} PRIVATE ${{ros2_lf_binding_DIR}}/../src/clock_gate.cpp)
        # And the rcl_action gate (counted action channels).
        target_sources(${{LF_TARGET}} PRIVATE ${{ros2_lf_binding_DIR}}/../src/action_gate.cpp)
        {nc_cmake_snippet}
        target_include_directories(${{LF_TARGET}} PRIVATE
            ${{CMAKE_CURRENT_SOURCE_DIR}}/lf/include
        )

        # Use plain target_link_libraries (not ament_target_dependencies) because
        # lfc's generated CMakeLists.txt uses the old-style (plain) signature and
        # CMake forbids mixing keyword and plain style on the same target.
        target_include_directories(${{LF_TARGET}} SYSTEM PRIVATE
            ${{rclcpp_INCLUDE_DIRS}}
            ${{rclcpp_components_INCLUDE_DIRS}}
            ${{ros2_lf_binding_INCLUDE_DIRS}}
            {dep_include_dirs}
        )
        target_link_libraries(${{LF_TARGET}} PRIVATE
            ${{rclcpp_LIBRARIES}}
            ${{rclcpp_components_LIBRARIES}}
            ${{ros2_lf_binding_LIBRARIES}}
            {dep_libraries}
        )

        install(TARGETS ${{LF_TARGET}}
            DESTINATION lib/${{PROJECT_NAME}}
        )
        install(DIRECTORY launch/
            DESTINATION share/${{PROJECT_NAME}}/launch
        )
        if(EXISTS "${{CMAKE_CURRENT_SOURCE_DIR}}/system.json")
            install(FILES system.json
                DESTINATION share/${{PROJECT_NAME}}
            )
        endif()
        if(EXISTS "${{CMAKE_CURRENT_SOURCE_DIR}}/exec_config.json")
            install(FILES exec_config.json
                DESTINATION share/${{PROJECT_NAME}}
            )
        endif()

        ament_package()
    """)
    (pkg_dir / "CMakeLists.txt").write_text(cmake_text, encoding="utf-8")

    # --- package.xml ---
    _hardcoded_deps = {"rclcpp", "rclcpp_components", "ros2_lf_binding"}
    dep_xml_lines = "\n".join(
        f"  <depend>{p}</depend>"
        for p in all_dep_packages
        if p not in _hardcoded_deps
    )
    pkg_xml_text = (
        '<?xml version="1.0"?>\n'
        '<?xml-model href="http://download.ros.org/schema/package_format3.xsd"\n'
        '    schematypens="http://www.w3.org/2001/XMLSchema"?>\n'
        f'<package format="3">\n'
        f'  <name>{pkg_name}</name>\n'
        f'  <version>0.1.0</version>\n'
        f'  <description>Generated lfc package for {stem}</description>\n'
        f'  <maintainer email="harun.teper@cps.rwth-aachen.de">colcon-lf</maintainer>\n'
        f'  <license>Apache-2.0</license>\n'
        f'\n'
        f'  <buildtool_depend>ament_cmake</buildtool_depend>\n'
        f'\n'
        f'  <depend>rclcpp</depend>\n'
        f'  <depend>rclcpp_components</depend>\n'
        f'  <depend>ros2_lf_binding</depend>\n'
        f'{dep_xml_lines}\n'
        f'\n'
        f'  <export>\n'
        f'    <build_type>ament_cmake</build_type>\n'
        f'  </export>\n'
        f'</package>\n'
    )
    (pkg_dir / "package.xml").write_text(pkg_xml_text, encoding="utf-8")

    # --- src/ros2_lf_startup.cpp ---
    # lfc generates its own main() so we must NOT define another main().
    # We use __attribute__((constructor)) to run before main(), calling
    # rclcpp::init (via ros2_lf_runtime_init), loading all components via
    # ros2_lf_runtime_load_nodes (so they exist before startup reactions fire
    # and call ros2_lf_bind_node), then constructing non-component nodes.
    main_cpp_text = textwrap.dedent(f"""\
        // Auto-generated by colcon lf-generate. DO NOT EDIT.
        // Copyright 2024 rcllf Contributors
        // SPDX-License-Identifier: Apache-2.0
        #include <cstdlib>
        #include "ros2_lf_binding/ros2_lf_binding.h"

        void ros2_lf_reaction_ids_init();
        void ros2_lf_register_all_nodes();

        static void ros2_lf_shutdown_atexit() {{
            ros2_lf_runtime_shutdown();
        }}

        __attribute__((constructor))
        static void ros2_lf_pre_main_init() {{
            ros2_lf_reaction_ids_init();
            // argc/argv not available in constructor; pass 0/nullptr and let
            // ros2_lf_runtime_init read SYSTEM_JSON from the environment.
            static const char* argv0[] = {{"lf_bringup", nullptr}};
            int argc = 1;
            char** argv = const_cast<char**>(argv0);
            ros2_lf_runtime_init(argc, argv);
            // Register non-component nodes before ros2_lf_runtime_load_nodes()
            // queries the registry. Component nodes don't need pre-registration.
            ros2_lf_register_all_nodes();
            // Load nodes (components from class-loader + non-components from registry)
            // before the LF scheduler fires startup reactions that call ros2_lf_bind_node.
            ros2_lf_runtime_load_nodes();
            // Anchor LF logical time zero to the instant DDS discovery settles,
            // so the first tags are not released in a startup burst.
            ros2_lf_runtime_wait_ready();
            std::atexit(ros2_lf_shutdown_atexit);
        }}
    """)
    (src_dir / "ros2_lf_startup.cpp").write_text(
        _declare_external_topics(main_cpp_text, external_topics, external_anchors),
        encoding="utf-8",
    )

    # --- launch/lf_{stem}.launch.py ---
    # No 'name=' argument: lfc binaries do not accept --ros-args remapping,
    # which launch_ros injects when a node name is specified.
    # SYSTEM_JSON: prefer the env var (allows override), fall back to the
    # system.json installed in this package's share directory (federated path)
    # or the one found via ament_index at runtime.
    has_pkg_system_json = (pkg_dir / "system.json").is_file() or (
        system_json is not None and system_json.name == "system.json"
        and system_json.parent.name == pkg_name
    )
    if has_pkg_system_json:
        system_json_expr = (
            f'os.environ.get("SYSTEM_JSON", '
            f"os.path.join(get_package_share_directory('{pkg_name}'), 'system.json'))"
        )
        extra_import = "from ament_index_python.packages import get_package_share_directory\n"
    else:
        system_json_baked = str(system_json) if system_json is not None else ""
        system_json_expr = f'os.environ.get("SYSTEM_JSON", "{system_json_baked}")'
        extra_import = ""
    offsets_stmt, offsets_expr = _timer_offsets_launch(lf_system)
    launch_text = textwrap.dedent(f"""\
        # Auto-generated by colcon lf-generate. DO NOT EDIT.
        import os
        import runpy
        import shlex
        from launch import LaunchDescription
        from launch.actions import RegisterEventHandler
        from launch.event_handlers import OnProcessExit
        from launch_ros.actions import Node
        {extra_import}

        def _exit_fails_the_launch(event, context):
            # A program that exits nonzero (a failed
            # reconcile, a startup refusal, the stop bound) fails this launch,
            # so `ros2 launch` exits 1 and a run check cannot miss it. Without
            # it the launch exits 0 whatever its processes returned.
            if event.returncode != 0:
                raise RuntimeError(
                    f"rcllf: {stem} exited {{event.returncode}}, the run failed")


        def generate_launch_description():
            system_json = {system_json_expr}
            {offsets_stmt}
            # lf-generate stages the resolved execution policy next to
            # system.json; the runtime reads ROS2_LF_EXEC_CONFIG (absent file
            # -> built-in defaults). The stock launch passed it as a node
            # parameter; this is the lfc equivalent.
            exec_config = os.environ.get(
                "ROS2_LF_EXEC_CONFIG",
                os.path.join(os.path.dirname(system_json), "exec_config.json"),
            )
            program = Node(
                    package="{pkg_name}",
                    executable="{stem}",
                    output="screen",
                    # Reactor-c arguments. The generated default
                    # (["-f", "true"] for --fast), the run-time timer offsets
                    #, then RCLLF_LF_ARGS at launch time, e.g.
                    # "-f true -o 20 sec" (fast mode, timeout); reactor-c
                    # takes the last -f, so the env var wins.
                    arguments={list(launch_lf_args or [])!r}
                    + {offsets_expr}
                    + shlex.split(os.environ.get("RCLLF_LF_ARGS", "")),
                    # As for federates: SIGTERM when ros2
                    # launch dies, which it does on a SIGTERM without
                    # stopping its processes. The pid is this launch
                    # process's, evaluated here, not in the child.
                    additional_env={{
                        "SYSTEM_JSON": system_json,
                        "ROS2_LF_EXEC_CONFIG": exec_config,
                        "RCLLF_EXIT_WITH_PARENT": str(os.getpid()),
                    }},
            )
            return LaunchDescription([
                program,
                RegisterEventHandler(OnProcessExit(
                    target_action=program, on_exit=_exit_fails_the_launch)),
            ])
    """)
    (launch_dir / f"lf_{generated_package_stem(stem)}.launch.py").write_text(
        launch_text, encoding="utf-8"
    )
    _copy_timer_offsets_helper(launch_dir)


def _resolve_and_stage_exec_config(
    *,
    system_json: Path,
    graph,
    exec_config_arg: str | None,
) -> Path | None:
    """Resolve the execution-policy config and stage exec_config.json.

    Default (no --exec-config): the analysis dir's ``config/execution.yaml``
    when present; absent = built-in defaults (today's behavior, zero
    migration burden). Malformed file is a hard error (raises), never a
    silent fall-through (frozen contract, 00_design.md section 2.4 item 4).

    Stages ``exec_config.json`` next to *system_json* and returns its path,
    or None if execution config is not applicable (should not happen; a
    built-in-defaults ExecConfig is always resolvable).
    """
    from colcon_lf.exec_config.resolve import print_warnings, resolve

    config = _load_exec_config(
        system_json=system_json, graph=graph, exec_config_arg=exec_config_arg
    )

    resolved = resolve(config, graph)
    print_warnings(resolved.warnings)

    exec_config_json_path = system_json.parent / "exec_config.json"
    exec_config_json_path.write_text(
        json.dumps(resolved.to_json_dict(), indent=2), encoding="utf-8"
    )
    print(f"  -> {exec_config_json_path}")
    return exec_config_json_path


def _resolve_and_stage_resolved_model(
    *,
    system_json: Path,
    graph,
    exec_config_arg: str | None,
    params_files: list[str] | None,
    workspace: Path | None,
    after_delay: str | None = None,
):
    """Build the resolved model (T3), write ``resolved_model.json``,
    and derive the per-node timer-period overrides for the generator.

    Returns ``(ResolvedModel, timer_period_overrides, timer_offset_overrides,
    acknowledged_hazards)`` where ``timer_period_overrides`` is
    ``{node_key: {callback: period_ms}}`` from ``execution.yaml``'s P3 override,
    parsed into milliseconds for ``generation.lf_generator.generate_lf_system``,
    ``timer_offset_overrides`` is the same shape for the start offsets of
    and
    ``acknowledged_hazards`` is the same ``ExecConfig``'s acknowledged set,
    for ``_check_resolved_model_gate`` (per-code, not inferred from
    ``claim_status``, which is global across all gated codes).
    """
    from colcon_lf.generation.lf_time import parse_lf_time_ns
    from colcon_lf.resolved_model.model import to_json
    from colcon_lf.resolved_model.resolve import resolve as resolve_model

    exec_config = _load_exec_config(
        system_json=system_json, graph=graph, exec_config_arg=exec_config_arg
    )

    resolved_workspace = workspace if workspace is not None else system_json.parent
    model = resolve_model(
        system_json,
        params_files=[Path(p) for p in (params_files or [])],
        exec_config=exec_config,
        workspace=resolved_workspace,
        after_delay=after_delay,
    )

    resolved_model_path = system_json.parent / "resolved_model.json"
    resolved_model_path.write_text(to_json(model), encoding="utf-8")
    print(f"  -> {resolved_model_path} (claim_status={model.claim_status})")

    timer_overrides: dict[str, dict[str, dict[str, float]]] = {"period": {}, "offset": {}}
    for node_key, node_cfg in exec_config.nodes.items():
        for callback, fields in (node_cfg.get("timers") or {}).items():
            for name, literal in fields.items():
                if name not in timer_overrides:
                    continue  # offset_runtime: _resolve_runtime_offsets
                timer_overrides[name].setdefault(node_key, {})[callback] = (
                    parse_lf_time_ns(literal) / 1_000_000
                )

    return (model, timer_overrides["period"], timer_overrides["offset"],
            set(exec_config.acknowledged_hazards))


# The conversion each tf2 R002 subcode points to.
_TF2_SUBCODE_HINTS = {
    "tf2_direct_lookup": (
        "request the transform through rcllf_tf2::AcquisitionLookup, which "
        "answers for the stamp with a bounded wait "),
    "tf2_blocking_lookup": (
        "a waiting lookup cannot see /tf messages from later reactions; use "
        "rcllf_tf2::AcquisitionLookup with its wait bound instead "),
    "tf2_listener_unadmitted": (
        "the admitted form is one member std::make_shared<tf2_ros::"
        "TransformListener>(*buffer, this) created in the constructor "),
    "tf2_clock_mismatch": (
        "construct the Buffer with the node's get_clock() and declare the "
        "provenance of external /tf in execution.yaml "),
    "tf2_stamp_mismatch": (
        "request at a stamp on the node clock: the node's now() or the "
        "header.stamp of a message published inside the system or declared "
        "in execution.yaml external_topics with 'anchor: stamp' "),
    "tf2_stamp_unknown": (
        "pass the request a stamp the scan can follow (now(), msg->header.stamp "
        "in the subscription callback, or a variable set from one of them) "),
}


def _check_resolved_model_gate(resolved_model, acknowledged_hazards: set[str]) -> int:
    """RCLLF-R001/R002 fail-closed gate (T6/T7).

    Reads records, not persisted diagnostics (Ruling): a patched or
    generate-time-overridden system.json can resolve a record the analyzer
    itself saw as unresolved. RCLLF-H003 (message_filters) stays on the
    existing hazard-diagnostic gate (`_check_hazard_gate`, already run
    before this) and is not re-checked here. Acknowledgement is per-code
    (`acknowledged_hazards` from `execution.yaml`, the same set `resolve()`
    used to compute `claim_status`) — not inferred from `claim_status`
    itself, which is a single system-wide value and would conflate two
    different gated codes if only one of them were acknowledged.
    """
    from colcon_lf.resolved_model.model import PROVENANCE_UNRESOLVED

    for r in resolved_model.records:
        if r.provenance != PROVENANCE_UNRESOLVED:
            continue
        if r.reason == "RCLLF-R001":
            # An acknowledgement no longer proceeds. The binding
            # matches a timer handle by its exact period in nanoseconds, so a
            # timer with no known period fits no handle and the build would
            # refuse at bind anyway, and the old 1000 ms placeholder put a
            # period nobody measured into the correspondence bounds. The only
            # way past is a real period, from the analysis or execution.yaml.
            fqn, cb = r.subject["node"], r.subject["callback"]
            acked = (
                " (listing RCLLF-R001 in acknowledged_hazards no longer proceeds)" if "RCLLF-R001" in acknowledged_hazards else ""
            )
            print(
                f"  Error: generation refused, RCLLF-R001 timer period "
                f"unresolved for {fqn}/{cb} (period_ms is null in "
                "system.json; supply it explicitly or set "
                f"nodes.{fqn}.timers.{cb}.period in execution.yaml){acked}",
                file=sys.stderr,
            )
            return 1
        if r.reason == "RCLLF-R002" and r.subject.get("kind") == "topic":
            # topic:/tf#provenance unresolved. The N1 check in
            # _check_external_topics refuses it first with the listener's
            # name; this branch covers the acknowledged case and callers that
            # skip that check.
            topic = r.subject.get("topic")
            if "RCLLF-R002" in acknowledged_hazards:
                print(
                    f"  Warning: acknowledged hazard RCLLF-R002 present "
                    f"({topic}) — proceeding because it is listed in "
                    "execution.yaml's acknowledged_hazards.",
                    file=sys.stderr,
                )
                continue
            print(
                f"  Error: generation refused, RCLLF-R002 tf2_clock_mismatch: "
                f"'{topic}' reaches a TransformListener from outside the system "
                "with no declared stamp provenance; declare it in execution.yaml "
                "external_topics with 'anchor: stamp' (or list RCLLF-R002 in "
                "acknowledged_hazards to proceed as unsupported)",
                file=sys.stderr,
            )
            return 1
        if r.reason == "RCLLF-R002":
            fqn = r.subject["node"]
            mechanism = r.property.split(".", 1)[1] if "." in r.property else r.property
            if "RCLLF-R002" in acknowledged_hazards:
                print(
                    f"  Warning: acknowledged hazard RCLLF-R002 present "
                    f"({fqn}) — proceeding because it is listed in "
                    "execution.yaml's acknowledged_hazards.",
                    file=sys.stderr,
                )
                continue
            refusals = (r.value.get("refusals")
                        if isinstance(r.value, dict) else None)
            if refusals:
                # The tf2 scan names why the node is outside
                # the admitted tf2 form, one line per refusal.
                for ref in refusals:
                    print(
                        f"  Error: generation refused, RCLLF-R002 "
                        f"{ref['subcode']}: {fqn} {ref['detail']}; "
                        f"{_TF2_SUBCODE_HINTS.get(ref['subcode'], '')}"
                        "(or list RCLLF-R002 in acknowledged_hazards to "
                        "proceed as unsupported)",
                        file=sys.stderr,
                    )
                return 1
            print(
                f"  Error: generation refused, RCLLF-R002 {fqn} depends on "
                f"unsupported {mechanism} (no rcllf contract for it; list "
                "RCLLF-R002 in acknowledged_hazards to proceed as "
                "unsupported)",
                file=sys.stderr,
            )
            return 1
    return 0


def _find_system_jsons(
    system_json_arg: str | None,
    workspace_arg: str | None,
    package_arg: str | None = None,
) -> list[Path]:
    """Locate system.json files to process.

    Thin alias for ``verb/configure.py``'s resolver so ``--package``
    has exactly one implementation.  Kept as a module-level name here because
    tests and ``verb/lf.py`` import it from both modules.

    Workspace resolution lives in ``verb/workspace.py`` for every verb.
    """
    from colcon_lf.verb.configure import (
        _find_system_jsons as _shared_find_system_jsons,
    )

    return _shared_find_system_jsons(system_json_arg, workspace_arg, package_arg)
