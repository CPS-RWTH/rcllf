# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``resolve()`` — build a :class:`ResolvedModel` from a real ``system.json``.

Properties resolved:
node identity, pub/sub topic + QoS, timer period (including the P3
``execution.yaml`` override and the benchmark harness's patch marker), parameter
values (analyze-time overlay plus any generate-time ``--params-file``
layered on top), the ``message_filters`` mechanism flag, and the
exec-config-resolved subscription policy keys.

Ruling: the resolver owns precedence and does not trust
``SystemGraph.parameter_files`` (the analyze-time cache) or the overlaid
``ParameterInfo.default_value`` blindly — it independently determines
*provenance* (was this value touched by any launch-time or generate-time
override, or is it the bare C++ declared default) by replaying the same
node-name/FQN/``/**`` matching analyze time used, against the *same*
already-loaded YAML content plus any ``--params-file`` inputs analyze never
saw. It does not need to re-read files from disk or re-run clang (the
`graph/hierarchical.py` P2 fix already made ``default_value`` itself
correct; the resolver does not repeat interpreting the YAML, only asks
"did anything touch this key").
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import yaml

from colcon_lf.comm_analysis.topic_mapper import resolve_name
from colcon_lf.graph.hierarchical import (
    _effective_fqn,
    _file_sections,
    _node_key,
    _node_key_matches,
    launch_parameter_values,
)
from colcon_lf.launch_analysis.ros_arguments import launch_identity
from colcon_lf.output.serializer import deserialize_system_graph
from colcon_lf.resolved_model.model import (
    CLAIM_PARTIAL,
    CLAIM_SUPPORTED,
    CLAIM_UNSUPPORTED,
    COMPAT_UNSUPPORTED,
    PROVENANCE_EXPLICITLY_CONFIGURED,
    PROVENANCE_MEASURED_PROFILED,
    PROVENANCE_EXTRACTED,
    PROVENANCE_UNRESOLVED,
    Record,
    ResolvedModel,
)

if TYPE_CHECKING:
    from colcon_lf.exec_config.model import ExecConfig
    from colcon_lf.graph.models import SystemGraph
    from colcon_lf.launch_analysis.models import LaunchNodeAction
    from colcon_lf.node_analysis.models import NodeInfo

MUTATION_CHANNELS = {
    "launch_overrides": "resolved",
    "ros_parameter_events": "observable_not_monitored",
    "environment": "not_detectable",
    "direct_field_mutation": "not_detectable",
}

_MESSAGE_FILTERS_INCLUDE = "message_filters"
_TF2_INCLUDE = "tf2_ros"


def _rel(path: Path, workspace: Path) -> str:
    """POSIX-relative-to-workspace path, absolute only if truly outside it."""
    resolved = path.resolve()
    try:
        return resolved.relative_to(workspace).as_posix()
    except ValueError:
        return resolved.as_posix()


def _launch_node_for(
    node_key: str, node_info: "NodeInfo", all_nodes: list
) -> "LaunchNodeAction | None":
    for launch_node in all_nodes:
        if _node_key(launch_node, node_info) == node_key:
            return launch_node
    return None


def _yaml_touches(
    param_name: str, fqn: str, yaml_files: dict[str, dict[str, Any]]
) -> tuple[bool, str | None, Any]:
    """Does any loaded YAML file set *param_name* for the node *fqn*?

    Returns ``(touched, role, value)``. Sections are read and matched as rcl
    does (nested names dotted, nested keys as namespaces, wildcards; see
    ``graph/hierarchical.py``). When more than one file in *yaml_files*
    (iterated in dict order, i.e. CLI order for the caller's
    ``--params-file`` case) matches, the last one wins.
    """
    touched = False
    role: str | None = None
    value: Any = None
    for r, content in yaml_files.items():
        for key, values in _file_sections(content, strip=False):
            if _node_key_matches(key, fqn) and param_name in values:
                touched, role, value = True, r, values[param_name]
    return touched, role, value


def _own_parameter_files(
    launch_node, graph, workspace: Path,
) -> dict[str, tuple[str, dict[str, Any]]]:
    """The loaded parameter files passed to *launch_node*, in launch order.

    Returned as ``{resolved path: (parameter_files key, content)}``, the
    *files* argument of ``launch_parameter_values``.

    Never the whole-system cache: a file applies only to the
    nodes it was given to. Files are matched to ``graph.parameter_files`` keys
    by path, independent of the workspace root either side used: a key matches
    if it is the file's resolved absolute path or a path-component suffix of
    it, and the longest match wins (keys share one root, so a shorter suffix
    can only be a different file). Fails closed if a file the launch action
    names was never loaded, instead of silently dropping it.
    """
    from colcon_lf.graph.hierarchical import _node_parameter_file_paths

    if launch_node is None:
        return {}
    out: dict[str, dict[str, Any]] = {}
    for pf in _node_parameter_file_paths(launch_node):
        abs_path = pf.resolve().as_posix()
        matches = [k for k in graph.parameter_files if k == abs_path or abs_path.endswith("/" + k)]
        if not matches:
            raise ValueError(
                f"parameter file {abs_path} passed to node {getattr(launch_node, 'name', '?')} "
                "was not loaded by lf-analyze; re-run colcon lf-analyze"
            )
        key = max(matches, key=len)
        out[abs_path] = (key, graph.parameter_files[key])
    return out


@dataclass
class _ParamsFileInput:
    role: str
    content: dict[str, Any]


def resolve(
    system_json: Path,
    *,
    params_files: list[Path] | None = None,
    exec_config: "ExecConfig",
    workspace: Path,
    after_delay: str | None = None,
) -> ResolvedModel:
    """Build a :class:`ResolvedModel` for the analysis at *system_json*.

    *after_delay* is the ``--after-delay`` flag of ``lf-generate``, the
    highest-precedence default of the link after-delays."""
    system_json = Path(system_json)
    workspace = Path(workspace).resolve()
    raw_text = system_json.read_text(encoding="utf-8")
    graph: "SystemGraph" = deserialize_system_graph(raw_text)

    # Optional input from an external benchmark tool: a top-level
    # `lf_bench_patch` key (`patched_fields`, a map of field path to the value
    # the tool patched in) that the deserializer above silently drops (unknown
    # to SystemGraph). It is read directly off the raw JSON so a patched timer
    # period attributes as `explicitly_configured` (evidence
    # `lf_bench_scenario#<path>`), not `extracted`. The key name is a stable
    # file-format name kept for tools that already write it; a system.json
    # without it resolves as before.
    lf_bench_patched_fields: dict[str, dict[str, Any]] = (
        (json.loads(raw_text).get("lf_bench_patch") or {}).get("patched_fields", {})
    )

    params_files = params_files or []
    cli_inputs: list[_ParamsFileInput] = []
    for i, pf in enumerate(params_files):
        with open(pf) as f:
            content = yaml.safe_load(f) or {}
        cli_inputs.append(_ParamsFileInput(role=f"params_file[{i}]", content=content))

    records: list[Record] = []
    sync_nodes: list[tuple[str, Any, Any]] = []  # sync.message_filters

    for node_key, node_info in graph.nodes.items():
        launch_node = _launch_node_for(node_key, node_info, graph.launch_graph.all_nodes)
        # The identity the node runs with, arguments= remaps included.
        identity = launch_identity(launch_node, node_info.name) if launch_node else None
        namespace = identity.namespace if identity else None
        remappings = list(identity.remappings) if identity else []
        ros_name = identity.name if identity else node_info.name
        param_fqn = (
            _effective_fqn(launch_node, node_info) if launch_node else f"/{node_info.name}"
        )

        # --- node:<fqn>#fqn ---
        records.append(Record(
            contract_id=f"node:{node_key}#fqn",
            property="fqn",
            subject={"kind": "node", "node": node_key},
            value=node_key,
            provenance=PROVENANCE_EXTRACTED,
            evidence="launch namespace + name (or ctor name)",
        ))

        # --- node:<fqn>#reaction_order ---
        records.append(_reaction_order_record(exec_config, node_key))

        # --- pub/sub:<fqn>/<source name>#topic (+ qos) ---
        for kind, endpoints in (("sub", node_info.subscribers), ("pub", node_info.publishers)):
            for ep in endpoints:
                source_name = ep.topic
                dynamic = getattr(ep, "topic_dynamic", False)
                subject = {"kind": kind, "node": node_key, "source": source_name}
                base_id = f"{kind}:{node_key}/{source_name}"
                if dynamic or not source_name:
                    records.append(Record(
                        contract_id=f"{base_id}#topic",
                        property="topic",
                        subject=subject,
                        value=source_name,
                        provenance=PROVENANCE_UNRESOLVED,
                        evidence="topic name argument not a resolvable string literal",
                        reason="RCLLF-H004",
                    ))
                else:
                    resolved_topic = resolve_name(
                        source_name, namespace, remappings, ros_name)
                    records.append(Record(
                        contract_id=f"{base_id}#topic",
                        property="topic",
                        subject=subject,
                        value=resolved_topic,
                        provenance=PROVENANCE_EXTRACTED,
                        evidence="comm_analysis.topic_mapper.resolve_name",
                    ))
                for prop, val in (
                    ("qos_depth", ep.qos_depth), ("qos_reliability", ep.qos_reliability),
                ):
                    if val is not None:
                        records.append(Record(
                            contract_id=f"{base_id}#{prop}",
                            property=prop,
                            subject=subject,
                            value=val,
                            provenance=PROVENANCE_EXTRACTED,
                            evidence="node_analysis clang extraction",
                        ))
                    else:
                        records.append(Record(
                            contract_id=f"{base_id}#{prop}",
                            property=prop,
                            subject=subject,
                            value=None,
                            provenance=PROVENANCE_UNRESOLVED,
                            evidence="not resolvable from source",
                        ))

        # --- timer:<fqn>/<cb>#period ---
        timer_overrides = exec_config.nodes.get(node_key, {}).get("timers", {})
        for timer_idx, timer in enumerate(node_info.timers):
            subject = {"kind": "timer", "node": node_key, "callback": timer.callback}
            base_id = f"timer:{node_key}/{timer.callback}"
            timer_cfg = timer_overrides.get(timer.callback) or {}
            override = timer_cfg.get("period")
            patch_entry = lf_bench_patched_fields.get(
                f"nodes.{node_key}.timers[{timer_idx}].period_ms"
            )
            if override is not None:
                from colcon_lf.generation.lf_time import parse_lf_time_ns
                period_ms = parse_lf_time_ns(override) / 1_000_000
                records.append(Record(
                    contract_id=f"{base_id}#period", property="period", subject=subject,
                    value=period_ms, unit="ms", domain="(0,inf)",
                    provenance=PROVENANCE_EXPLICITLY_CONFIGURED,
                    evidence=f"execution.yaml:nodes.{node_key}.timers.{timer.callback}.period",
                ))
            elif patch_entry is not None and timer.period_ms is not None:
                # The benchmark harness's
                # patch marker (patch.py::patch_system_json) names exactly
                # which fields it filled in from the scenario — a genuinely
                # provided value, not a source-level literal.
                records.append(Record(
                    contract_id=f"{base_id}#period", property="period", subject=subject,
                    value=timer.period_ms, unit="ms", domain="(0,inf)",
                    provenance=PROVENANCE_EXPLICITLY_CONFIGURED,
                    evidence=(
                        f"lf_bench_scenario#{patch_entry['scenario_key']}"
                        f"={patch_entry['value']}"
                    ),
                ))
            elif timer.period_ms is not None:
                records.append(Record(
                    contract_id=f"{base_id}#period", property="period", subject=subject,
                    value=timer.period_ms, unit="ms", domain="(0,inf)",
                    provenance=PROVENANCE_EXTRACTED,
                    evidence="literal period_ms in system.json",
                ))
            else:
                # compatibility_mode stays the Record default (preserved):
                # per the property table, only the mechanism rows
                # (message_filters/tf2/take) get `unsupported` — a missing
                # timer period is absent data, not an unsupported mechanism.
                # Whether generation proceeds anyway (acknowledged) is the
                # generate-verb gate's concern (verb/generate.py), read off
                # `exec_config.acknowledged_hazards` directly, not encoded
                # here.
                records.append(Record(
                    contract_id=f"{base_id}#period", property="period", subject=subject,
                    value=None, unit="ms", domain="(0,inf)",
                    provenance=PROVENANCE_UNRESOLVED,
                    evidence="period_ms is null in system.json (no override configured)",
                    reason="RCLLF-R001",
                ))
            # --- timer:<fqn>/<cb>#offset ---
            offset_override = timer_cfg.get("offset")
            if offset_override is not None:
                from colcon_lf.generation.lf_time import parse_lf_time_ns
                records.append(Record(
                    contract_id=f"{base_id}#offset", property="offset", subject=subject,
                    value=parse_lf_time_ns(offset_override) / 1_000_000, unit="ms",
                    domain="[0,inf)", provenance=PROVENANCE_EXPLICITLY_CONFIGURED,
                    evidence=f"execution.yaml:nodes.{node_key}.timers.{timer.callback}.offset",
                ))
            else:
                records.append(Record(
                    contract_id=f"{base_id}#offset", property="offset", subject=subject,
                    value=timer.offset_ms, unit="ms", domain="[0,inf)",
                    provenance=PROVENANCE_EXTRACTED,
                    evidence="offset_ms in system.json (rclcpp timers start at creation)",
                ))

        # --- node:<fqn>#mechanism.message_filters ---
        # Two-input ExactTime/ApproximateTime synchronizers are admitted:
        # their inputs are ordinary LF-dispatched
        # subscriptions, so the record is extracted. Any unsupported use (or
        # an older system.json without a scan result) stays unresolved.
        detected = getattr(node_info, "detected_includes", None) or []
        if _MESSAGE_FILTERS_INCLUDE in detected:
            mf = getattr(node_info, "message_filters", None)
            if mf is not None and not mf.unsupported:
                records.append(Record(
                    contract_id=f"node:{node_key}#mechanism.message_filters",
                    property="mechanism.message_filters",
                    subject={"kind": "node", "node": node_key},
                    value=[f"{s.policy}({s.queue_size}) {'+'.join(s.inputs)} -> {s.callback}"
                           for s in mf.synchronizers],
                    provenance=PROVENANCE_EXTRACTED,
                    evidence="message_filters scan admitted every synchronizer "
                    "(two-input ExactTime/ApproximateTime/BoundedNearestToAnchor)",
                ))
                records += _bna_records(node_key, node_info, mf, exec_config)
                sync_nodes.append((node_key, node_info, mf))
            else:
                records.append(Record(
                    contract_id=f"node:{node_key}#mechanism.message_filters",
                    property="mechanism.message_filters",
                    subject={"kind": "node", "node": node_key},
                    value=True,
                    provenance=PROVENANCE_UNRESOLVED,
                    evidence="unsupported message_filters use (RCLLF-H003): "
                    + ("; ".join(mf.unsupported) if mf is not None
                       else "no scan result in system.json"),
                    compatibility_mode=COMPAT_UNSUPPORTED,
                    reason="RCLLF-H003",
                ))
        # --- node:<fqn>#mechanism.tf2 ---
        # The tf2 scan (node_analysis/tf2_scan.py) splits tf2 use into the
        # admitted form (a Buffer on the node clock, one TransformListener
        # whose /tf and /tf_static inputs LF owns, broadcasters as ordinary
        # publishers, lookups only through rcllf_tf2::AcquisitionLookup) and
        # refusals, each with an RCLLF-R002 subcode. A system.json analysed
        # before the scan existed has no tf2 field and keeps the old blanket
        # refusal.
        if _TF2_INCLUDE in detected:
            records += _tf2_records(node_key, getattr(node_info, "tf2", None),
                                    exec_config.nodes.get(node_key, {}).get("tf2"),
                                    node_info=node_info, graph=graph,
                                    exec_config=exec_config)

        # --- node:<fqn>#mechanism.take ---
        take_calls = getattr(node_info, "detected_take_calls", None) or []
        if take_calls:
            records.append(Record(
                contract_id=f"node:{node_key}#mechanism.take",
                property="mechanism.take",
                subject={"kind": "node", "node": node_key},
                value=True,
                provenance=PROVENANCE_UNRESOLVED,
                evidence=f"explicit take call detected: {', '.join(take_calls)}",
                compatibility_mode=COMPAT_UNSUPPORTED,
                reason="RCLLF-R002",
            ))

        # --- param:<fqn>/<name>#value ---
        launch_values = (
            launch_parameter_values(
                launch_node, node_info, _own_parameter_files(launch_node, graph, workspace))
            if launch_node else {}
        )
        for param in node_info.parameters:
            subject = {"kind": "param", "node": node_key, "name": param.name}
            base_id = f"param:{node_key}/{param.name}"
            value = param.default_value
            provenance = PROVENANCE_EXTRACTED
            evidence = "C++ declared default"
            # Attribution comes from the same launch-order walk that set the
            # analyzer's value (graph/hierarchical.py): the source
            # that wins in `ros2 launch` is the one named as evidence.
            launched = launch_values.get(param.name)
            if launched is not None:
                provenance = PROVENANCE_EXPLICITLY_CONFIGURED
                evidence = launched[1]
            # CLI --params-file postdates analysis; later files win, same as
            # the analyze-time overlay's own dict-insertion-order semantics.
            for cli in cli_inputs:
                cli_touched, _, cli_value = _yaml_touches(
                    param.name, param_fqn, {cli.role: cli.content}
                )
                if cli_touched:
                    value = cli_value
                    provenance = PROVENANCE_EXPLICITLY_CONFIGURED
                    evidence = cli.role
            records.append(Record(
                contract_id=f"{base_id}#value", property="value", subject=subject,
                value=value, provenance=provenance, evidence=evidence,
            ))

    # --- sub:<fqn>/<topic>#<policy key> from the resolved exec-config ---
    for row in exec_config_rows(exec_config, graph):
        records.append(row)

    # --- link:<src>-><dst>/<name>#after_delay ---
    link_rows = link_delay_rows(graph, exec_config, after_delay)
    records += link_rows

    # --- node:<fqn>#sync.message_filters ---
    for node_key, node_info, mf in sync_nodes:
        records += _sync_records(node_key, node_info, mf, link_rows)

    # --- topic:/tf#provenance, topic:/tf_static#provenance ---
    records += _tf2_topic_records(graph, exec_config)

    # claim_status (design "Artifact and schema"): supported when no record
    # is unresolved; partial when only non-gated records are unresolved;
    # unsupported when a gated refusal's code (RCLLF-R001/R002/H003) was
    # acknowledged. Acknowledging a gated code without the record actually
    # being unresolved is a no-op here (nothing to downgrade).
    gated_codes = {"RCLLF-R001", "RCLLF-R002", "RCLLF-H003"}
    unresolved_reasons = {r.reason for r in records if r.provenance == PROVENANCE_UNRESOLVED}
    acknowledged_gated = unresolved_reasons & gated_codes & set(exec_config.acknowledged_hazards)
    if acknowledged_gated:
        claim_status = CLAIM_UNSUPPORTED
    elif unresolved_reasons:
        claim_status = CLAIM_PARTIAL
    else:
        claim_status = CLAIM_SUPPORTED

    inputs = [
        {"role": "system_json", "path": _rel(system_json, workspace),
         "sha256": input_sha256(system_json, role="system_json")},
    ]
    for cli in cli_inputs:
        pf_path = params_files[int(cli.role.split("[")[1].rstrip("]"))]
        inputs.append({"role": cli.role, "path": _rel(Path(pf_path), workspace), "sha256": _sha256(Path(pf_path))})

    model = ResolvedModel(
        system={"name": graph.name, "system_json_schema_version": graph.schema_version},
        claim_status=claim_status,
        records=records,
        mutation_channels=dict(MUTATION_CHANNELS),
        inputs=inputs,
    )
    from colcon_lf.resolved_model.model import finalize
    return finalize(model)


def _bna_records(
    node_key: str, node_info: Any, mf: Any, exec_config: "ExecConfig",
) -> list[Record]:
    """``node:<fqn>#sync.bna`` and ``sub:<fqn>/<topic>#stamp_provenance``.

    The provenance is per subscription, as execution.yaml declares it. A
    missing declaration is unresolved here and refused by generation.
    """
    from colcon_lf.exec_config.policies import KEY_STAMP_PROVENANCE
    from colcon_lf.generation.lf_generator import bna_input_callback

    out: list[Record] = []
    for sync in mf.synchronizers:
        if sync.bna_params is None:
            continue
        out.append(Record(
            contract_id=f"node:{node_key}#sync.bna",
            property="sync.bna",
            subject={"kind": "node", "node": node_key},
            value={"member": sync.member_variable, **sync.bna_params},
            unit="ns",
            provenance=PROVENANCE_EXTRACTED,
            evidence="literal BoundedNearestToAnchor constructor arguments",
        ))
        callbacks = {bna_input_callback(sync.callback, m) for m in sync.inputs}
        for sub in node_info.subscribers:
            if sub.callback not in callbacks:
                continue
            declared = exec_config.subscriptions.get(node_key, {}).get(
                sub.topic, {}).get(KEY_STAMP_PROVENANCE)
            out.append(Record(
                contract_id=f"sub:{node_key}/{sub.topic}#{KEY_STAMP_PROVENANCE}",
                property=KEY_STAMP_PROVENANCE,
                subject={"kind": "sub", "node": node_key, "topic": sub.topic},
                value=declared,
                provenance=(PROVENANCE_EXPLICITLY_CONFIGURED if declared is not None
                            else PROVENANCE_UNRESOLVED),
                evidence=(
                    f"execution.yaml:subscriptions.{node_key}.{sub.topic}."
                    f"{KEY_STAMP_PROVENANCE}" if declared is not None
                    else "no stamp_provenance declared, generation refuses"
                ),
            ))
    return out


def _tf2_records(node_key: str, tf2: Any, configured: dict | None = None, *,
                 node_info: Any = None, graph: Any = None,
                 exec_config: Any = None) -> list[Record]:
    """``mechanism.tf2``, one ``tf2.lookup`` and one ``tf2.sample_stamp`` per
    adapter.

    *configured* is ``nodes.<node>.tf2`` of execution.yaml
    (``{wait_bound_ns?, on_timeout?}``). It wins over the constructor
    literals for every adapter of the node, the record is then
    ``explicitly_configured`` and keeps the literals under ``literal``.
    *node_info*, *graph* and *exec_config* resolve a message stamp to the
    topic and its producer (``_tf2_stamp_record``).
    """
    subject = {"kind": "node", "node": node_key}
    if tf2 is None:
        return [Record(
            contract_id=f"node:{node_key}#mechanism.tf2",
            property="mechanism.tf2", subject=subject, value=True,
            provenance=PROVENANCE_UNRESOLVED,
            evidence="detected_includes contains 'tf2_ros' but system.json has no tf2 scan "
            "result (analysed by an older colcon lf-analyze); re-run colcon lf-analyze",
            compatibility_mode=COMPAT_UNSUPPORTED, reason="RCLLF-R002",
        )]
    if tf2.refusals:
        return [Record(
            contract_id=f"node:{node_key}#mechanism.tf2",
            property="mechanism.tf2", subject=subject,
            value={"refusals": [{"subcode": r.subcode, "detail": r.detail}
                                for r in tf2.refusals]},
            provenance=PROVENANCE_UNRESOLVED,
            evidence="; ".join(f"{r.subcode}: {r.detail}" for r in tf2.refusals),
            compatibility_mode=COMPAT_UNSUPPORTED, reason="RCLLF-R002",
        )]
    records = [Record(
        contract_id=f"node:{node_key}#mechanism.tf2",
        property="mechanism.tf2", subject=subject,
        value={
            "listeners": [x.member for x in tf2.listeners],
            "broadcasters": [
                {"member": b.member, "topic": "/tf_static" if b.static else "/tf"}
                for b in tf2.broadcasters
            ],
            "buffer_clock": "node" if tf2.buffers else None,
            "ingress": "lf_owned" if tf2.listeners else None,
        },
        provenance=PROVENANCE_EXTRACTED,
        evidence="node_analysis/tf2_scan.py: "
        + ", ".join(
            f"{kind} {x.member} at {x.file}:{x.line}"
            for kind, xs in (("Buffer", tf2.buffers), ("TransformListener", tf2.listeners),
                             ("broadcaster", tf2.broadcasters))
            for x in xs
        ),
    )]
    from colcon_lf.node_analysis.tf2_scan import TF2_ROS_PIN
    for a in tf2.adapters:
        value = {
            "wait_bound_ns": a.wait_bound_ns,
            "on_timeout": a.on_timeout,
            "buffer": a.buffer_member,
            "buffer_clock": "node",
            "tf2_ros_version": TF2_ROS_PIN,
        }
        provenance = PROVENANCE_EXTRACTED
        evidence = f"literal AcquisitionLookup arguments at {a.file}:{a.line}"
        if configured:
            value["literal"] = {"wait_bound_ns": a.wait_bound_ns, "on_timeout": a.on_timeout}
            keys = sorted(k for k in configured if k in ("wait_bound_ns", "on_timeout"))
            value.update({k: configured[k] for k in keys})
            value["configured"] = keys
            provenance = PROVENANCE_EXPLICITLY_CONFIGURED
            evidence = (f"execution.yaml nodes.{node_key}.tf2 "
                        f"({', '.join(sorted(configured))}) over the literal "
                        f"AcquisitionLookup arguments at {a.file}:{a.line}")
        records.append(Record(
            contract_id=f"node:{node_key}#tf2.lookup.{a.member}",
            property=f"tf2.lookup.{a.member}", subject=dict(subject, adapter=a.member),
            value=value,
            unit="ns", domain="(0,inf)",
            provenance=provenance,
            evidence=evidence,
        ))
        records.append(_tf2_stamp_record(node_key, a.member, tf2.stamps, node_info,
                                         graph, exec_config))
    return records


def _norm_topic(topic: str) -> str:
    return "/" + topic.lstrip("/")


def _tf2_stamp_record(node_key: str, member: str, stamps: Any, node_info: Any,
                      graph: Any, exec_config: Any) -> Record:
    """``node:<fqn>#tf2.sample_stamp.<member>``: where the stamp of every
    request on adapter *member* comes from (criterion 5).

    The adapter asks the Buffer, which runs on the node clock (LF logical
    time under rcllf), for the transform at the stamp; the stamp must be a
    time on that clock. Admitted sources, per request call:

    * ``node_clock``: the node's own ``now()`` (``extracted``);
    * ``message``: the ``header.stamp`` of a message of a subscription of
      this node, whose topic is published inside the system (a converted
      publisher, ``extracted``) or declared ``external_topics`` with
      ``anchor: stamp`` (``explicitly_configured``).

    Refused with RCLLF-R002: a stamp read from another clock, or a message
    from outside the system without a stamp anchor (``tf2_stamp_mismatch``),
    and a stamp whose source the scan cannot follow, an adapter with no
    request the scan sees, or a system.json from before the stamp scan
    (``tf2_stamp_unknown``). Not scanned: the expression the producer
    stamps its messages with.
    """
    from colcon_lf.exec_config.yaml_schema import EXT_ANCHOR_STAMP, KEY_EXT_ANCHOR

    subject = {"kind": "node", "node": node_key, "adapter": member}
    cid, prop = f"node:{node_key}#tf2.sample_stamp.{member}", f"tf2.sample_stamp.{member}"
    refusals: list[dict] = []
    requests: list[dict] = []
    configured_any = False
    mine = None if stamps is None else [s for s in stamps if s.adapter == member]
    if mine is None:
        refusals.append({"subcode": "tf2_stamp_unknown", "detail": (
            f"AcquisitionLookup {member}: system.json has no stamp scan (analysed before "
            "the sample stamp record); re-run colcon lf-analyze")})
    elif not mine:
        refusals.append({"subcode": "tf2_stamp_unknown", "detail": (
            f"AcquisitionLookup {member}: the scan finds no {member}->request(...) call, so "
            "the stamp its requests carry is unknown")})
    subs = getattr(node_info, "subscribers", None) or []
    publishers: dict[str, list[str]] = {}
    if graph is not None:
        for tc in graph.communication.topics:
            if tc.publishers:
                publishers.setdefault(_norm_topic(tc.topic), []).extend(tc.publishers)
    anchors = {_norm_topic(k): v for k, v in
               (getattr(exec_config, "external_anchors", None) or {}).items()}
    for st in mine or []:
        where = f"{member}->request at {st.file}:{st.line}"
        req = {"expr": st.expr, "kind": st.kind, "file": st.file, "line": st.line}
        if st.kind == "node_clock":
            req["source"] = "node_clock"
        elif st.kind == "foreign_clock":
            refusals.append({"subcode": "tf2_stamp_mismatch", "detail": (
                f"{where} passes stamp '{st.expr}': {st.detail}, but the Buffer runs on "
                "the node clock")})
            continue
        elif st.kind == "message":
            topics = ([_norm_topic(st.topic)] if st.topic else sorted(
                {_norm_topic(s.topic) for s in subs if s.callback == st.callback}))
            if not topics:
                refusals.append({"subcode": "tf2_stamp_unknown", "detail": (
                    f"{where} passes stamp '{st.expr}' ({st.detail}) in "
                    f"'{st.callback}', which is no subscription callback of the node")})
                continue
            req["topics"] = topics
            sources = []
            for t in topics:
                anchor = anchors.get(t)
                if t in publishers:
                    sources.append({"topic": t, "source": "in_system",
                                    "publishers": sorted(set(publishers[t]))})
                elif isinstance(anchor, dict) and anchor.get(KEY_EXT_ANCHOR) == EXT_ANCHOR_STAMP:
                    configured_any = True
                    sources.append({"topic": t, "source": "external_stamp",
                                    "basis": anchor.get("basis")})
                else:
                    refusals.append({"subcode": "tf2_stamp_mismatch", "detail": (
                        f"{where} passes stamp '{st.expr}' ({st.detail}) of '{t}', which "
                        "comes from outside the system without 'anchor: stamp', so the "
                        "stamp is on a clock the Buffer does not run on")})
            req["sources"] = sources
        else:
            refusals.append({"subcode": "tf2_stamp_unknown", "detail": (
                f"{where} passes stamp '{st.expr}': {st.detail}")})
            continue
        requests.append(req)
    if refusals:
        return Record(
            contract_id=cid, property=prop, subject=subject,
            value={"refusals": refusals},
            provenance=PROVENANCE_UNRESOLVED,
            evidence="; ".join(f"{r['subcode']}: {r['detail']}" for r in refusals),
            compatibility_mode=COMPAT_UNSUPPORTED, reason="RCLLF-R002",
        )
    return Record(
        contract_id=cid, property=prop, subject=subject,
        value={"buffer_clock": "node", "requests": requests},
        provenance=PROVENANCE_EXPLICITLY_CONFIGURED if configured_any else PROVENANCE_EXTRACTED,
        evidence="node_analysis/tf2_scan.py: " + "; ".join(
            f"{r['expr']} at {r['file']}:{r['line']} -> {r['kind']}" for r in requests)
        + ("; execution.yaml external_topics anchor: stamp" if configured_any else ""),
    )


def _tf2_topic_records(graph: "SystemGraph", exec_config: "ExecConfig") -> list[Record]:
    """``topic:<tf topic>#provenance``: where the stamps of the transforms an
    admitted TransformListener ingests come from (plan S4).

    One record per /tf or /tf_static that a listener of the system takes.
    Published inside the system: the in-system broadcasters (``extracted``),
    whose messages ride correspondence edges at their publishing tag.
    Declared in ``external_topics`` with ``anchor: stamp``: the declared
    basis, delay and grid (``explicitly_configured``); each TFMessage enters
    at the one header.stamp its transforms carry, and one with differing
    stamps is ended as ``tf_mixed_stamps``. Otherwise ``unresolved`` with
    RCLLF-R002, the N1 refusal (``_check_tf2_external_ingress``).
    """
    from colcon_lf.exec_config.yaml_schema import EXT_ANCHOR_STAMP, KEY_EXT_ANCHOR

    listeners: dict[str, list[str]] = {}
    for node_key, node in graph.nodes.items():
        for sub in getattr(node, "subscribers", None) or []:
            if getattr(sub, "tf2_listener", None):
                listeners.setdefault(sub.topic, []).append(f"{node_key}.{sub.tf2_listener}")
    publishers = {tc.topic: sorted(tc.publishers) for tc in graph.communication.topics
                  if tc.publishers}
    anchors = getattr(exec_config, "external_anchors", None) or {}
    out: list[Record] = []
    for topic in sorted(listeners):
        subject = {"kind": "topic", "topic": topic}
        takers = sorted(set(listeners[topic]))
        anchor = anchors.get(topic)
        if topic in publishers:
            out.append(Record(
                contract_id=f"topic:{topic}#provenance", property="provenance",
                subject=subject,
                value={"source": "in_system", "publishers": publishers[topic],
                       "listeners": takers},
                provenance=PROVENANCE_EXTRACTED,
                evidence=f"system.json: {topic} published by {', '.join(publishers[topic])}; "
                "its transforms reach the Buffer at the publishing tag over a "
                "correspondence edge",
            ))
        elif isinstance(anchor, dict) and anchor.get(KEY_EXT_ANCHOR) == EXT_ANCHOR_STAMP:
            out.append(Record(
                contract_id=f"topic:{topic}#provenance", property="provenance",
                subject=subject,
                value={"source": "external", "listeners": takers,
                       **{k: v for k, v in anchor.items()},
                       "tf_rule": "common_stamp_or_tf_mixed_stamps"},
                provenance=PROVENANCE_EXPLICITLY_CONFIGURED,
                evidence=f"execution.yaml external_topics: {topic} anchor: stamp; each "
                "TFMessage enters at the one header.stamp all its transforms carry, one "
                "with differing stamps or none is not admitted (tf_mixed_stamps)",
            ))
        else:
            out.append(Record(
                contract_id=f"topic:{topic}#provenance", property="provenance",
                subject=subject,
                value={"source": "external_undeclared", "listeners": takers},
                provenance=PROVENANCE_UNRESOLVED,
                evidence=f"no node of the system publishes {topic} and execution.yaml does "
                "not declare it external_topics with anchor: stamp",
                compatibility_mode=COMPAT_UNSUPPORTED, reason="RCLLF-R002",
            ))
    return out


def _sync_records(node_key: str, node_info: Any, mf: Any, link_rows: list[Record]) -> list[Record]:
    """``node:<fqn>#sync.message_filters``: the mf-sync/1 parameters.

    Policy, queue size, the inputs in input order with their topic and the
    link delay d of the link into this node (``None`` when no link of the
    system feeds it, which generation refuses), and the library versions the
    domain was validated against. One record per ExactTime/ApproximateTime
    synchronizer; a BNA synchronizer has ``sync.bna`` instead.
    """
    from colcon_lf.features.modules.message_filters import DOMAIN_ID, LIBRARY_PINS
    from colcon_lf.generation.lf_generator import bna_input_callback

    into = {}
    for row in link_rows:
        subj = row.subject
        if row.property == "after_delay" and subj.get("dst") == node_key and subj.get("link") == "topic":
            into.setdefault(subj["name"].lstrip("/"), set()).add(row.value)
    out: list[Record] = []
    for sync in mf.synchronizers:
        if sync.bna_params is not None:
            continue
        inputs = []
        for member in sync.inputs:
            cb = bna_input_callback(sync.callback, member)
            topic = next((s.topic for s in node_info.subscribers if s.callback == cb), None)
            delays = sorted(into.get((topic or "").lstrip("/"), ()))
            inputs.append({"member": member, "topic": topic,
                           "delay_ns": delays[0] if len(delays) == 1 else (delays or None)})
        out.append(Record(
            contract_id=f"node:{node_key}#sync.message_filters",
            property="sync.message_filters",
            subject={"kind": "node", "node": node_key},
            value={"domain": DOMAIN_ID, "member": sync.member_variable, "policy": sync.policy,
                   "queue_size": sync.queue_size, "callback": sync.callback,
                   "inputs": inputs, "library_pins": dict(LIBRARY_PINS)},
            provenance=PROVENANCE_EXTRACTED,
            evidence="message_filters scan (policy, queue, inputs) and the link after-delay records",
        ))
    return out


def _reaction_order_record(exec_config: "ExecConfig", node_key: str) -> Record:
    """``node:<fqn>#reaction_order``: the declaration order of the node's timer
    and subscription reactions. An unset key reads back as the
    registry default, the convention of ``exec_config_rows``."""
    from colcon_lf.exec_config.policies import KEY_REACTION_ORDER, REACTION_ORDERS, REGISTRY

    if KEY_REACTION_ORDER in exec_config.nodes.get(node_key, {}):
        value = exec_config.nodes[node_key][KEY_REACTION_ORDER]
        evidence = f"execution.yaml:nodes.{node_key}.{KEY_REACTION_ORDER}"
    elif KEY_REACTION_ORDER in exec_config.defaults:
        value = exec_config.defaults[KEY_REACTION_ORDER]
        evidence = f"execution.yaml:defaults.{KEY_REACTION_ORDER}"
    else:
        value = REGISTRY[KEY_REACTION_ORDER].default
        evidence = f"registry-default:{KEY_REACTION_ORDER}"
    return Record(
        contract_id=f"node:{node_key}#{KEY_REACTION_ORDER}",
        property=KEY_REACTION_ORDER,
        subject={"kind": "node", "node": node_key},
        value=value,
        domain="{" + ",".join(REACTION_ORDERS) + "}",
        provenance=PROVENANCE_EXPLICITLY_CONFIGURED,
        evidence=evidence,
    )


def exec_config_rows(exec_config: "ExecConfig", graph: "SystemGraph") -> list[Record]:
    """``sub:<fqn>/<topic>#<policy key>`` records from the resolved exec-config."""
    from colcon_lf.exec_config.policies import RUNTIME_REGISTRY
    from colcon_lf.exec_config.resolve import resolve as resolve_exec_config

    resolved = resolve_exec_config(exec_config, graph)
    out: list[Record] = []
    for row in resolved.subscriptions:
        for node_key in row.node_keys:
            for key in RUNTIME_REGISTRY:
                if key not in row.values:
                    continue
                is_default = row.values[key] == RUNTIME_REGISTRY[key].default and (
                    key not in exec_config.defaults
                    and key not in exec_config.nodes.get(node_key, {})
                    and key not in exec_config.subscriptions.get(node_key, {}).get(row.topic, {})
                )
                out.append(Record(
                    contract_id=f"sub:{node_key}/{row.topic}#{key}",
                    property=key,
                    subject={"kind": "sub", "node": node_key, "topic": row.topic},
                    value=row.values[key],
                    provenance=PROVENANCE_EXPLICITLY_CONFIGURED,
                    evidence=(
                        f"registry-default:{key}" if is_default
                        else f"execution.yaml:{_scope_for(exec_config, node_key, row.topic, key)}"
                    ),
                ))
    return out


#: ``lf-generate``'s default after-delay when neither the flag nor
#: ``execution.yaml`` sets one (``verb.generate.DEFAULT_AFTER_DELAY``).
DEFAULT_LINK_AFTER_DELAY = "0"


def link_delay_rows(
    graph: "SystemGraph", exec_config: "ExecConfig", after_delay: str | None = None,
) -> list[Record]:
    """``link:<src>-><dst>/<name>#after_delay`` records, in ns.

    The after-delay of every link between two nodes of the system, resolved
    with ``_build_connections``' precedence: a per-link ``after_delay_ns`` in
    system.json, then the ``--after-delay`` flag, then ``execution.yaml``
    ``defaults.after_delay``, then 0 (one microstep). A topic link runs with it as an LF
    ``after`` or, when admitted, as the correspondence delay d. A service link
    carries it both ways; an action link carries it on goal request and
    response and 1 msec (``ACTION_ASYNC_AFTER_DELAY_NS``) on its other
    channels (``#after_delay.async``). A federation replaces the delay of a
    link across federates (:func:`federate_link_records`). A ``connections:``
    entry of ``execution.yaml`` beats all of these; its record
    names the entry and the stated reason, and a measured value has
    provenance ``measured_profiled``."""
    from colcon_lf.comm_analysis.endpoints import endpoint_key_map
    from colcon_lf.exec_config.connections import REASON_MEASURED, link_delays
    from colcon_lf.exec_config.policies import KEY_AFTER_DELAY
    from colcon_lf.generation.lf_generator import ACTION_ASYNC_AFTER_DELAY_NS
    from colcon_lf.generation.lf_time import parse_lf_time_ns

    configured = exec_config.defaults.get(KEY_AFTER_DELAY)
    if after_delay is not None:
        literal, prov, evidence = after_delay, PROVENANCE_EXPLICITLY_CONFIGURED, "lf-generate --after-delay"
    elif configured:
        literal, prov, evidence = (str(configured), PROVENANCE_EXPLICITLY_CONFIGURED,
                                   f"execution.yaml:defaults.{KEY_AFTER_DELAY}")
    else:
        literal, prov, evidence = DEFAULT_LINK_AFTER_DELAY, PROVENANCE_EXTRACTED, "lf-generate default"
    default_ns = parse_lf_time_ns(literal, source="after_delay")
    nodes = set(graph.nodes)
    r2k = endpoint_key_map(graph)
    configured_links = link_delays(exec_config)
    out: dict[str, Record] = {}

    def add(src: str, dst: str, name: str, prop: str, kind: str, ns: int, prov_: str,
            evidence_: str) -> None:
        src, dst = r2k.get(src, src), r2k.get(dst, dst)
        if src not in nodes or dst not in nodes:
            return
        cfg = configured_links.get(name, src, dst) if prop == "after_delay" else None
        if cfg is not None:
            ns = cfg.ns
            prov_ = (PROVENANCE_MEASURED_PROFILED if cfg.reason == REASON_MEASURED
                     else PROVENANCE_EXPLICITLY_CONFIGURED)
            evidence_ = f"{cfg.evidence} ({cfg.reason}: {cfg.source})" if cfg.source else (
                f"{cfg.evidence} ({cfg.reason})")
        cid = f"link:{src}->{dst}/{name.lstrip('/')}#{prop}"
        out[cid] = Record(
            contract_id=cid, property=prop,
            subject={"kind": "link", "link": kind, "src": src, "dst": dst, "name": name},
            value=int(ns), unit="ns", domain="[0,inf)", provenance=prov_, evidence=evidence_,
        )

    comm = graph.communication
    for tc in comm.topics if comm else []:
        if tc.after_delay_ns is not None:
            ns, p, ev = tc.after_delay_ns, PROVENANCE_EXTRACTED, f"system.json:topics.{tc.topic}.after_delay_ns"
        else:
            ns, p, ev = default_ns, prov, evidence
        for pub in tc.publishers:
            for sub in tc.subscribers:
                add(pub, sub, tc.topic, "after_delay", "topic", ns, p, ev)
    for sc in comm.services if comm else []:
        for client in sc.clients:
            add(client, sc.server, sc.service, "after_delay", "service", default_ns, prov, evidence)
    for ac in comm.actions if comm else []:
        for client in ac.clients:
            add(client, ac.server, ac.action, "after_delay", "action", default_ns, prov, evidence)
            add(client, ac.server, ac.action, "after_delay.async", "action",
                ACTION_ASYNC_AFTER_DELAY_NS, PROVENANCE_EXTRACTED,
                "lf_generator.ACTION_ASYNC_AFTER_DELAY_NS")
    return [out[k] for k in sorted(out)]


#: lf_generator's action channels the client sends; the others the server sends.
_ACTION_CLIENT_SENDS = ("goal_req", "result_req", "cancel_req")
#: The action channels that carry the link's after-delay, the rest ``#after_delay.async``.
_ACTION_SYNC_CHANNELS = ("goal_req", "goal_resp")


def _link_record_for(kind: str, src: str, dst: str, name: str) -> str:
    """The ``link:`` record a directed LF channel's delay belongs to."""
    prop = "after_delay"
    a, b = src, dst
    if kind == "service response":
        a, b = dst, src
    elif kind.startswith("action "):
        channel = kind.split(" ", 1)[1]
        prop = "after_delay" if channel in _ACTION_SYNC_CHANNELS else "after_delay.async"
        if channel not in _ACTION_CLIENT_SENDS:
            a, b = dst, src
    return f"link:{a}->{b}/{name.lstrip('/')}#{prop}"


def federate_link_records(records: list[Record], cross_links: list[dict[str, Any]]) -> list[Record]:
    """The link records of a federated program.

    A link across federates does not run the after-delay the unfederated
    program runs: lf-generate gives it the federation's (``after_overrides``,
    else ``DEFAULT_CROSS_FEDERATE_AFTER_NS``). ``cross_links`` has one row per
    directed channel across federates: ``kind`` (``topic``, ``service
    request``, ``action goal_req``, ...), ``src``, ``dst``, ``name``,
    ``after_ns`` (the delay d a record arrives after), ``network_ns`` (the
    ``after`` of lfc's network connection), ``provenance``, ``evidence`` and
    ``network_evidence`` and ``admitted``. An admitted correspondence edge
    runs a network link of d - 1 ns, because its records leave at t + 1 ns;
    a legacy link runs d itself. The network record's subject
    names the carrier, ``correspondence`` or ``legacy``. The channel's ``link:`` record takes d, and a
    ``link:<src>-><dst>/<name>[<kind>]#after_delay.network`` record holds the
    network ``after``. Channels of one record with different delays leave it
    unresolved."""
    wanted: dict[str, list[dict[str, Any]]] = {}
    out: list[Record] = []
    for row in cross_links:
        name = row["name"]
        wanted.setdefault(_link_record_for(row["kind"], row["src"], row["dst"], name), []).append(row)
        cid = f"link:{row['src']}->{row['dst']}/{name.lstrip('/')}[{row['kind']}]#after_delay.network"
        out.append(Record(
            contract_id=cid, property="after_delay.network",
            subject={"kind": "link", "link": row["kind"], "src": row["src"], "dst": row["dst"],
                     "name": name,
                     "carrier": "correspondence" if row.get("admitted") else "legacy"},
            value=int(row["network_ns"]), unit="ns", domain="[0,inf)",
            provenance=row["provenance"], evidence=row["network_evidence"],
        ))
    for record in records:
        rows = wanted.pop(record.contract_id, None)
        if rows is None:
            out.append(record)
            continue
        values = sorted({int(r["after_ns"]) for r in rows})
        if len(values) == 1:
            first = rows[0]
            out.append(Record(
                contract_id=record.contract_id, property=record.property,
                subject=dict(record.subject), value=values[0], unit="ns", domain=record.domain,
                provenance=first["provenance"], evidence=first["evidence"],
            ))
        else:
            out.append(Record(
                contract_id=record.contract_id, property=record.property,
                subject=dict(record.subject), value=None, unit="ns", domain=record.domain,
                provenance=PROVENANCE_UNRESOLVED, evidence="federation after_overrides",
                reason="its channels across federates run different after-delays: "
                + ", ".join(f"{r['kind']} {r['after_ns']} ns" for r in rows),
            ))
    if wanted:
        raise ValueError("cross-federate links without a link record in the resolved model: "
                         + ", ".join(sorted(wanted)))
    return out


def _scope_for(exec_config: "ExecConfig", node_key: str, topic: str, key: str) -> str:
    if key in exec_config.subscriptions.get(node_key, {}).get(topic, {}):
        return f"subscriptions.{node_key}.{topic}.{key}"
    if key in exec_config.nodes.get(node_key, {}):
        return f"nodes.{node_key}.{key}"
    return f"defaults.{key}"


# Keys that ``colcon lf-generate`` itself writes into system.json in place
# (``verb/generate.py`` ``_stage_component_instances``). They are generation
# output, not configuration input, so the RCLLF-R003 fingerprint ignores them;
# otherwise every component-based system went stale during its own generate.
GENERATOR_OWNED_SYSTEM_JSON_KEYS = ("component_instances",)


def input_sha256(path: Path, role: str) -> str:
    """Fingerprint one recorded input (RCLLF-R003).

    ``system_json`` is hashed as canonical JSON without the generator-owned
    keys; every other input is hashed byte for byte.
    """
    import hashlib
    import json
    if role == "system_json":
        raw = json.loads(path.read_text(encoding="utf-8"))
        for key in GENERATOR_OWNED_SYSTEM_JSON_KEYS:
            raw.pop(key, None)
        data = json.dumps(raw, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(data).hexdigest()
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _sha256(path: Path) -> str:
    return input_sha256(path, role="file")
