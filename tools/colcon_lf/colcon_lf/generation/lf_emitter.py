# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Render an LFSystem model to ``.lf`` source text.

Entry point: ``emit_lf_system(system) -> str``
"""

from __future__ import annotations

from colcon_lf.generation.lf_model import (
    LFConnection,
    LFPhysicalAction,
    LFPort,
    LFReaction,
    LFReactor,
    LFReactorInstance,
    LFSystem,
    LFTimer,
)

INDENT = "    "
# The binding's built-in ingress STORED capacity (kIngressCap, ingress.hpp).
INGRESS_DEFAULT_CAPACITY = 8

# Nanosecond thresholds for unit selection (largest unit that divides evenly)
_US = 1_000
_MS = 1_000_000
_S = 1_000_000_000


def _ns_to_lf_time(ns: int) -> str:
    """Format an integer nanosecond delay as a readable LF time literal."""
    if ns == 0:
        return "0"
    if ns % _S == 0:
        return f"{ns // _S} sec"
    if ns % _MS == 0:
        return f"{ns // _MS} msec"
    if ns % _US == 0:
        return f"{ns // _US} usec"
    return f"{ns} nsec"


def emit_lf_system(system: LFSystem) -> str:
    """Emit a complete ``.lf`` file from an LFSystem."""
    parts: list[str] = []

    # Target declaration with optional options block
    if system.target_options:
        opts_lines = [f"target {system.target} {{"]
        for k, v in system.target_options.items():
            if isinstance(v, bool):
                v_str = "true" if v else "false"
            elif isinstance(v, str):
                v_str = v
            else:
                v_str = str(v)
            opts_lines.append(f"{INDENT}{k}: {v_str},")
        opts_lines.append("}")
        parts.append("\n".join(opts_lines))
    else:
        parts.append(f"target {system.target}")

    # Preamble block (shared includes / typedefs)
    if system.preamble:
        parts.append("")
        preamble_lines = [f"preamble {{="]
        for line in system.preamble.splitlines():
            preamble_lines.append(f"{INDENT}{line}" if line.strip() else "")
        preamble_lines.append("=}")
        parts.append("\n".join(preamble_lines))

    # Reactor definitions (non-main)
    for reactor in system.reactors:
        parts.append("")
        parts.append(_emit_reactor(reactor, is_main=False))

    # Main reactor
    parts.append("")
    parts.append(_emit_main_reactor(system))

    return "\n".join(parts) + "\n"


def _external_action_inputs(reactor: LFReactor) -> list[tuple[str, str]]:
    """``(physical action, goal RID macro)`` of the external action servers
    whose reactions live in *reactor*, once each, in reaction order."""
    out: list[tuple[str, str]] = []
    for r in reactor.reactions:
        if r.external_action is None or r.external_register_rid is None:
            continue
        if all(a != r.external_action for a, _ in out):
            out.append((r.external_action, r.external_register_rid))
    return out


def _emit_reactor(reactor: LFReactor, *, is_main: bool) -> str:
    """Emit a single reactor definition block."""
    prefix = "main " if is_main else ""
    if reactor.name.lower().startswith("_lf"):
        # lfc names its generated classes `_lf_...` (the delay
        # reactor of a connection `after d` is `_lf_GenDelay_<hash>`), and the
        # read-back identifies those by that name.
        raise ValueError(f"reactor class {reactor.name!r}: names starting with '_lf' are lfc's")
    lines: list[str] = [
        f"{prefix}reactor {reactor.name}{_emit_parameters(reactor.parameters)} {{"]

    # Ports (outputs first, then inputs — conventional ordering)
    outputs = [p for p in reactor.ports if p.direction == "output"]
    inputs = [p for p in reactor.ports if p.direction == "input"]
    for port in outputs:
        lines.append(f"{INDENT}{_emit_port(port)}")
    for port in inputs:
        lines.append(f"{INDENT}{_emit_port(port)}")

    # Timers
    for timer in reactor.timers:
        lines.append(f"{INDENT}{_emit_timer(timer)}")

    # Physical actions
    for pa in reactor.physical_actions:
        lines.append(f"{INDENT}{_emit_physical_action(pa)}")

    for la in reactor.logical_actions:
        lines.append(f"{INDENT}logical action {la}")
    for action, _ in reactor.external_inputs:
        lines.append(f"{INDENT}physical action {action}: int64_t")
    for action, _ in _external_action_inputs(reactor):
        lines.append(f"{INDENT}physical action {action}: int64_t")

    # Ingress actions: physical wake from the ingress thread,
    # logical drain that runs one request per microstep on an LF worker.
    if reactor.ingress_key is not None:
        lines.append(f"{INDENT}physical action ingress_wake")
        lines.append(f"{INDENT}logical action ingress_drain")

    # Instances (nested reactors)
    for inst in reactor.instances:
        lines.append(f"{INDENT}{_emit_instance(inst)}")

    # Internal connections (port forwarding)
    for conn in reactor.connections:
        lines.append(f"{INDENT}{_emit_connection(conn)}")

    # Startup reaction (emitted before user reactions)
    if reactor.startup_body is not None:
        lines.append("")
        lines.append(_emit_startup_reaction(reactor.startup_body, reactor.startup_effects))

    # User reactions. LF runs one reactor's reactions of a tag in
    # declaration order, so the corr edge close goes last. At the stop tag it
    # then follows every reaction here that can publish (timer, subscription,
    # ingress drain) and the reconcile counts their records.
    closes = [r for r in reactor.reactions if r.kind == "corr_close"]
    for reaction in reactor.reactions:
        if reaction.kind == "corr_close":
            continue
        lines.append("")
        lines.append(_emit_reaction(reaction))

    # External input: register each subscription's physical
    # action with the binding, which schedules it at message arrival.
    for action, rid in reactor.external_inputs:
        lines.append("")
        lines.append(
            f"{INDENT}reaction(startup) -> {action} {{=\n"
            f"{INDENT}{INDENT}ros2_lf_external_register(self, {rid}, "
            f"rcllf_external_wake, {action});\n{INDENT}=}}"
        )

    # External action ingress: the binding schedules the
    # physical action when a request from a client outside LF arrives.
    for action, rid in _external_action_inputs(reactor):
        lines.append("")
        lines.append(
            f"{INDENT}reaction(startup) -> {action} {{=\n"
            f"{INDENT}{INDENT}ros2_lf_action_external_register(self, {rid}, "
            f"rcllf_external_wake, {action});\n{INDENT}=}}"
        )

    if reactor.ingress_key is not None:
        for block in _ingress_reactions(
            reactor.ingress_key, bound=reactor.ingress_bound,
            capacity=reactor.ingress_capacity,
        ):
            lines.append("")
            lines.append(block)

    for reaction in closes:
        lines.append("")
        lines.append(_emit_reaction(reaction))

    lines.append("}")
    return "\n".join(lines)


def _ingress_reactions(
    key: str, *, bound: bool = False, capacity: int | None = None,
) -> list[str]:
    """The drain host's ingress reactions.

    startup registers the node with the binding's ingress role and hands it
    the wake thunk plus the physical action; ingress_wake moves stored request
    ids into the carrier; ingress_drain runs exactly one request and
    reschedules itself while work remains; shutdown closes the mailbox. A
    schedule handle <= 0 is a refusal and closes the node's mailbox.

    A *capacity* other than the binding default is set before
    the registration, so no request is ever stored against the default.
    """
    k = key.replace("\\", "\\\\").replace('"', '\\"')
    # In a group sub-reactor the drain runs user code (parameter
    # and lifecycle callbacks), so it reads the bind port like every reaction.
    uses = " rcllf_bound" if bound else ""
    resched = (
        f"{INDENT}{INDENT}{INDENT}if (lf_schedule(ingress_drain, 0) <= 0) {{\n"
        f"{INDENT}{INDENT}{INDENT}{INDENT}ros2_lf_ingress_schedule_refused(\"{k}\");\n"
        f"{INDENT}{INDENT}{INDENT}}}\n"
    )
    set_cap = ""
    if capacity is not None and capacity != INGRESS_DEFAULT_CAPACITY:
        set_cap = f"{INDENT}{INDENT}ros2_lf_ingress_set_capacity(\"{k}\", {int(capacity)});\n"
    return [
        f"{INDENT}reaction(startup) -> ingress_wake {{=\n{set_cap}"
        f"{INDENT}{INDENT}ros2_lf_ingress_register(self, \"{k}\", rcllf_ingress_wake, "
        f"ingress_wake);\n{INDENT}=}}",
        f"{INDENT}reaction(ingress_wake){uses} -> ingress_drain {{=\n"
        f"{INDENT}{INDENT}if (ros2_lf_ingress_move(\"{k}\")) {{\n{resched}"
        f"{INDENT}{INDENT}}}\n{INDENT}=}}",
        f"{INDENT}reaction(ingress_drain){uses} -> ingress_drain {{=\n"
        f"{INDENT}{INDENT}if (ros2_lf_ingress_drain_one(self, \"{k}\")) {{\n{resched}"
        f"{INDENT}{INDENT}}}\n{INDENT}=}}",
        f"{INDENT}reaction(shutdown) {{=\n"
        f"{INDENT}{INDENT}ros2_lf_ingress_close(\"{k}\");\n{INDENT}=}}",
    ]


def _emit_main_reactor(system: LFSystem) -> str:
    """Emit the main (or federate-level) reactor with instances and inter-node connections."""
    # When this system is a federate inside a federated reactor, emit a plain
    # reactor (not main reactor) with exposed cross-federate ports.
    is_federate = system.federate_name is not None
    prefix = "" if is_federate else "main "
    lines: list[str] = [
        f"{prefix}reactor {system.name}{_emit_parameters(system.parameters)} {{"]

    # Cross-federate boundary ports (output/input forwarded to/from inner instances).
    # cross_federate_ports may contain MULTIPLE entries per port_name (one per
    # inner endpoint) so a single boundary port can fan out to many subscribers
    # or fan in from many publishers.  Declare each unique port only once.
    cross_ports = getattr(system, "cross_federate_ports", [])
    declared_ports: set[str] = set()
    for entry in cross_ports:
        port_name, direction, lf_type = entry[0], entry[1], entry[2]
        if port_name in declared_ports:
            continue
        declared_ports.add(port_name)
        lines.append(f"{INDENT}{direction} {port_name}: {lf_type}")

    for inst in system.main_instances:
        lines.append(f"{INDENT}{_emit_instance(inst)}")

    if (cross_ports or system.main_instances) and system.main_connections:
        lines.append("")

    for conn in system.main_connections:
        if conn.comment:
            lines.append(f"{INDENT}// {conn.comment}")
        lines.append(f"{INDENT}{_emit_connection(conn)}")

    # Forwarding connections for cross-federate ports, plus (decentralized
    # coordination, input ports only) a logging-only STP reaction on the same
    # boundary trigger. The STP reaction never replaces or touches the plain
    # forwarding connection below it — reading a port via both a connection and
    # a reaction in the same reactor is valid LF, and this keeps centralized
    # output byte-identical to before.
    xtag_ports = getattr(system, "xtag_ports", {})
    if cross_ports:
        lines.append("")
        for port_name, direction, _lf_type, inner_inst, inner_port, stp_ns in cross_ports:
            if port_name in xtag_ports:
                continue  # packed and imported below
            if direction == "output":
                # Forward inner output → outer output
                lines.append(f"{INDENT}{inner_inst}.{inner_port} -> {port_name}")
                continue
            # Forward outer input → inner input
            lines.append(f"{INDENT}{port_name} -> {inner_inst}.{inner_port}")
            if stp_ns is not None:
                offset_str = _ns_to_lf_time(stp_ns)
                lines.append(f"{INDENT}reaction({port_name}) {{=")
                lines.append(f"{INDENT}=}} STP({offset_str}) {{=")
                lines.append(
                    f'{INDENT}{INDENT}lf_print_warning("STP violation: tardy message on '
                    f'federate boundary port \'{port_name}\' (-> {inner_inst}.{inner_port}) '
                    f'at tag (%lld, %u)", lf_time_logical_elapsed(), lf_tag().microstep);'
                )
                lines.append(f"{INDENT}=}}")

    xtag_expect = getattr(system, "xtag_expect", {})
    if xtag_expect:
        # Declared before the imports, so it runs first at the start
        # tag; n is the one writer federate's publishers of the topic.
        lines.append(f"{INDENT}reaction(startup) {{=")
        for topic, n in sorted(xtag_expect.items()):
            lines.append(f'{INDENT}{INDENT}ros2_lf_xtag_expect("{_c_str(topic)}", {n});')
        lines.append(f"{INDENT}=}}")
    lines.extend(_emit_xtag_reactions(cross_ports, xtag_ports))
    for edge, topic, reason in getattr(system, "count_rule_edges", []):
        # This edge carries only a count, so its take keeps the count rule.
        lines.append(f"{INDENT}reaction(startup) {{=")
        lines.append(
            f'{INDENT}{INDENT}lf_print_warning("cross-federate take link {_c_str(edge)} on topic '
            f"'{_c_str(topic)}' carries only a count ({_c_str(reason)}), so the count rule "
            'applies and a late sample may be delivered one tag late");')
        lines.append(f"{INDENT}=}}")

    # `after d` with d > 0 maps every microstep of one time to one
    # tag. reactor-c moves a second message of one tag to the next microstep
    # and merges a third into it, so the third write of a time exits. Under
    # decentralized coordination the receiver judges that second message tardy
    # and moves or drops it, so there the second write exits.
    guard_max = getattr(system, "legacy_guard_max", 2)
    for port_name in getattr(system, "legacy_guard_ports", []):
        inners = [
            f"{inst}.{port}" for name, direction, _t, inst, port, _s in cross_ports
            if name == port_name and direction == "output"
        ]
        lines.append(f"{INDENT}reaction({', '.join(inners)}) {{=")
        lines.append(f'{INDENT}{INDENT}ros2_lf_xfed_legacy_sent("{port_name}", {guard_max});')
        lines.append(f"{INDENT}=}}")

    lines.append("}")
    return "\n".join(lines)


def _c_str(text: str) -> str:
    """*text* escaped for a C string literal."""
    return text.replace("\\", "\\\\").replace('"', '\\"')


def _emit_xtag_reactions(cross_ports, xtag_ports) -> list[str]:
    """Pack and import reactions of the rcllf_xtag_t boundary ports.

    An output packs the write count of its inner publisher port with the
    writer's tag log events of the topic since that port's last pack (two
    publisher nodes of the federate leave on two ports); an input imports
    the events into this federate's tag log and writes the count to every inner
    subscriber port, so the take drops a late sample by tag. Under decentralized
    coordination the import also runs, with a warning, when the message is tardy.
    """
    lines: list[str] = []
    done: set[str] = set()
    # Every import before every pack. LF orders the reactions of
    # one reactor by declaration, so a pack declared before an import whose
    # inner node feeds the packed output closes a zero-delay cycle inside the
    # federate reactor (pack -> import -> node -> output -> pack) that no
    # cross-federate `after` can break. Imports first keeps the declaration
    # order the same as the data flow: input -> node -> output.
    ordered = ([r for r in cross_ports if r[1] == "input"]
               + [r for r in cross_ports if r[1] == "output"])
    for port_name, direction, _t, inner_inst, inner_port, _s in ordered:
        if port_name not in xtag_ports or port_name in done:
            continue
        done.add(port_name)
        topic, edges = xtag_ports[port_name]
        rows = [r for r in cross_ports if r[0] == port_name]
        if direction == "output":
            src = f"{inner_inst}.{inner_port}"
            lines.append(f"{INDENT}reaction({src}) -> {port_name} {{=")
            lines.append(f"{INDENT}{INDENT}rcllf_xtag_t rcllf_x;")
            lines.append(
                f'{INDENT}{INDENT}ros2_lf_xtag_pack("{_c_str(topic)}", "{port_name}", {src}->value, '
                "&rcllf_x);")
            lines.append(f"{INDENT}{INDENT}lf_set({port_name}, rcllf_x);")
            lines.append(f"{INDENT}=}}")
            continue
        inners = [f"{r[3]}.{r[4]}" for r in rows]
        edge = ", ".join(edges)
        body = [
            f'int32_t rcllf_n = ros2_lf_xtag_import("{_c_str(topic)}", "{_c_str(edge)}", '
            f"&{port_name}->value);",
            "if (rcllf_n > 0) {",
        ] + [f"{INDENT}lf_set({i}, rcllf_n);" for i in inners] + ["}"]
        lines.append(f"{INDENT}reaction({port_name}) -> {', '.join(inners)} {{=")
        lines.extend(f"{INDENT}{INDENT}{b}" for b in body)
        stps = [r[5] for r in rows if r[5] is not None]
        if stps:
            lines.append(f"{INDENT}=}} STP({_ns_to_lf_time(max(stps))}) {{=")
            lines.append(
                f'{INDENT}{INDENT}lf_print_warning("STP violation: tardy message on '
                f"federate boundary port '{port_name}' (-> {', '.join(inners)}) "
                f'at tag (%lld, %u)", lf_time_logical_elapsed(), lf_tag().microstep);')
            lines.extend(f"{INDENT}{INDENT}{b}" for b in body)
        lines.append(f"{INDENT}=}}")
    return lines


def _emit_port(port: LFPort) -> str:
    """Emit ``input name: Type`` or ``output name: Type``."""
    return f"{port.direction} {port.name}: {port.type}"


def _emit_timer(timer: LFTimer) -> str:
    """Emit ``timer name(offset, period)``."""
    return f"timer {timer.name}({timer.offset}, {timer.period})"


def _emit_physical_action(pa: LFPhysicalAction) -> str:
    """Emit ``physical action name(min_delay)``."""
    delay_str = _ns_to_lf_time(pa.min_delay_ns)
    return f"physical action {pa.name}({delay_str}): stub_t"


def _emit_instance(instance: LFReactorInstance) -> str:
    """Emit ``name = new Class()`` or ``name = new Class(p = v, ...)``."""
    args = ", ".join(f"{p} = {v}" for p, v in instance.args)
    return f"{instance.name} = new {instance.reactor_class}({args})"


def _emit_parameters(parameters: list[tuple[str, str, str]]) -> str:
    """Emit ``(p: type = default, ...)`` after a reactor name, or nothing
    when there are none, so a reactor without parameters keeps today's text.
    """
    if not parameters:
        return ""
    return "(" + ", ".join(f"{n}: {t} = {d}" for n, t, d in parameters) + ")"


def _emit_connection(conn: LFConnection) -> str:
    """Emit ``source -> destination`` or ``source -> destination after N msec``."""
    base = f"{conn.source} -> {conn.destination}"
    # Cross-federate connections use federate_after_ns when set
    if conn.is_cross_federate and conn.federate_after_ns is not None:
        return f"{base} after {_ns_to_lf_time(conn.federate_after_ns)}"
    if conn.after_delay_ns is not None:
        return f"{base} after {_ns_to_lf_time(conn.after_delay_ns)}"
    return base


def emit_federated_top_level(
    system_name: str,
    federate_names: list[str],
    cross_connections: list[LFConnection],
    target: str = "CCpp",
    target_options: dict | None = None,
    federate_lf_stems: dict | None = None,
    coordination: str = "centralized",
    federate_parameters: dict[str, list[tuple[str, str, str]]] | None = None,
) -> str:
    """Emit the top-level federated .lf file declaring all federates and cross-federate connections.

    This file is compiled by lfc with --federated to produce per-federate binaries.
    federate_lf_stems maps federate_name -> lf_system_name (the reactor class / .lf filename stem).
    The per-federate .lf files must be co-located under {fed_name}/{lf_stem}.lf relative to this file.

    colcon_lf can emit either of LF's two coordination modes: centralized (the
    RTI mediates every cross-federate tag advance; default, never tardy) or
    decentralized (peer-to-peer, STP-offset based, tardy messages possible).
    ``coordination`` selects which; the actual ``coordination: decentralized``
    target property, when applicable, is expected in ``target_options`` (the
    caller adds it there). See docs/design/time-and-coordination.md.

    ``federate_parameters`` maps a federate to the run-time timer offset
    parameters of its reactor. They become parameters of the
    federated reactor, which lfc turns into command-line options of the
    federate that uses them, and are passed down to the federate.
    """
    parts: list[str] = []

    # Target declaration
    if target_options:
        opts_lines = [f"target {target} {{"]
        for k, v in (target_options or {}).items():
            if isinstance(v, bool):
                v_str = "true" if v else "false"
            elif isinstance(v, str):
                v_str = v
            else:
                v_str = str(v)
            opts_lines.append(f"{INDENT}{k}: {v_str},")
        opts_lines.append("}")
        parts.append("\n".join(opts_lines))
    else:
        parts.append(f"target {target}")

    # Coordination mode note, mirroring the actual target property (or its
    # LF-default absence) so a reader never has to cross-reference the two.
    if coordination == "decentralized":
        parts.append(
            "// Coordination: decentralized (peer-to-peer, STP-offset/tardiness-based, "
            "RTI for startup/shutdown only). See docs/design/time-and-coordination.md#regime-3-federated-decentralized."
        )
    else:
        parts.append(
            "// Coordination: centralized (RTI-mediated, never tardy). Pass "
            "--coordination decentralized to colcon lf-generate for regime 3."
        )

    parts.append("")

    # Import each federate reactor from its own .lf file.
    # Per-federate .lf files are placed at {fed_name}/{lf_stem}.lf relative to this top-level file.
    # lf_stem is the LF system class name (PascalCase, e.g. "PublisherNodeSystem").
    for fed_name in federate_names:
        lf_stem = (federate_lf_stems or {}).get(fed_name) or fed_name
        # lf_stem is already the reactor class name (PascalCase)
        reactor_class = lf_stem
        parts.append(f'import {reactor_class} from "{fed_name}/{lf_stem}.lf"')

    parts.append("")

    # Top-level federated reactor
    fed_params = federate_parameters or {}
    top_params = [p for f in federate_names for p in fed_params.get(f, [])]
    lines = [f"federated reactor {system_name}{_emit_parameters(top_params)} {{"]
    for fed_name in federate_names:
        lf_stem_inst = (federate_lf_stems or {}).get(fed_name) or fed_name
        lines.append(f"{INDENT}" + _emit_instance(LFReactorInstance(
            name=fed_name, reactor_class=lf_stem_inst,
            args=[(n, n) for n, _, _ in fed_params.get(fed_name, [])])))

    if cross_connections:
        lines.append("")
        for conn in cross_connections:
            lines.append(f"{INDENT}{_emit_connection(conn)}")

    lines.append("}")
    parts.append("\n".join(lines))

    return "\n".join(parts) + "\n"


def _emit_startup_reaction(body: str, effects: list[str] | None = None) -> str:
    """Emit a ``reaction(startup)`` block, with ``-> effects`` when given."""
    header = f"{INDENT}reaction(startup)"
    if effects:
        header += " -> " + ", ".join(effects)
    lines = [header + " {="]
    for line in body.splitlines():
        lines.append(f"{INDENT}{INDENT}{line}" if line.strip() else "")
    lines.append(f"{INDENT}=}}")
    return "\n".join(lines)


def _emit_reaction(reaction: LFReaction) -> str:
    """Emit a full reaction block with {= =} body."""
    trigger_str = ", ".join(reaction.triggers)
    parts = [f"{INDENT}reaction({trigger_str})"]
    if reaction.sources:
        parts[0] += " " + ", ".join(reaction.sources)

    if reaction.effects:
        effects_str = ", ".join(reaction.effects)
        parts[0] += f" -> {effects_str}"

    parts[0] += " {="
    for line in reaction.body.splitlines():
        parts.append(f"{INDENT}{INDENT}{line}" if line.strip() else "")
    parts.append(f"{INDENT}=}}")

    return "\n".join(parts)
