# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Link identities the read-back names delays with.

The program registers, per link of the system, the carrier that realises it:
the LF connection into an input port (``carrier="lf"``) or the admitted
correspondence edge (``carrier="corr"``), or, in a federate, lfc's network
connection of a link across federates (``carrier="network"`` where it
arrives, ``"network-out"`` where it leaves). It also registers the offset of
every reaction-triggering input port in lfc's self struct, taken with
``offsetof`` on lfc's own header inside lfc's target, so that the binding can
name the port an lfc delay reactor writes. The delay is read from the running
program; these rows only say which contract a delay belongs to.
"""

from __future__ import annotations

import re
from collections import Counter
from typing import Any

from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_model import LFSystem

_CLASS_LINE = re.compile(r"^(main )?reactor (\w+)")
_REACTION_LINE = re.compile(r"^    reaction\s*\(")


def lfc_class_name(reactor_name: str) -> str:
    """lfc 0.12's C name of a reactor class: ``_`` + the lowercased name
    (``ServiceServerNode_DefaultGroup`` -> ``_serviceservernode_defaultgroup``,
    header ``_serviceservernode_defaultgroup.h``, struct ``..._self_t``)."""
    return "_" + reactor_name.lower()


def link_rows(lf_system: LFSystem) -> list[dict[str, Any]]:
    """One row per carrier of a system link in this program."""
    rows: list[dict[str, Any]] = []
    for conn in lf_system.main_connections:
        if conn.link is None or conn.is_cross_federate:
            continue
        kind, src, dst, name = conn.link
        src_inst = conn.source.split(".", 1)[0]
        dst_inst, dst_port = conn.destination.split(".", 1)
        rows.append({"kind": kind, "src": src, "dst": dst, "name": name, "carrier": "lf",
                     "src_instance": src_inst, "dst_instance": dst_inst, "dst_port": dst_port,
                     "edge": -1})
    for e in lf_system.corr_edges:
        if e.edge_id is None or e.role != "local":
            continue
        rows.append({"kind": "topic", "src": e.pub_node, "dst": e.sub_node, "name": e.topic,
                     "carrier": "corr", "src_instance": "", "dst_instance": "", "dst_port": "",
                     "edge": e.edge_id})
    # A link across federates runs lfc's network connection; the
    # federate it arrives in reads that connection's `after` back.
    rows += [dict(r) for r in lf_system.network_links]
    return rows


def port_rows(lf_system: LFSystem) -> list[tuple[str, str]]:
    """``(reactor class, input port)`` for every input port that triggers a
    reaction of an instantiated class: lfc gives exactly those a
    ``trigger_t _lf__<port>`` field."""
    by_name = {r.name: r for r in lf_system.reactors}
    used: list[str] = []
    todo = [i.reactor_class for i in lf_system.main_instances]
    while todo:
        cls = todo.pop()
        if cls in used or cls not in by_name:
            continue
        used.append(cls)
        todo += [i.reactor_class for i in by_name[cls].instances]
    rows: list[tuple[str, str]] = []
    for cls in sorted(used):
        reactor = by_name[cls]
        inputs = {p.name for p in reactor.ports if p.direction == "input"}
        triggers = {t for r in reactor.reactions for t in r.triggers}
        rows += [(cls, port) for port in sorted(inputs & triggers)]
    return rows


def class_rows(lf_system: LFSystem) -> list[tuple[str, int, int]]:
    """``(lfc C class, reactions per instance, instances)`` for every class
    of the program with a reaction. The reactions are counted in the emitted
    LF text (a reaction is declared at the reactor's indentation), the
    instances along the main reactor's instance tree. The main reactor itself
    is one instance; lfc's C name of a ``main reactor X`` is ``_x_main``."""
    reactions: dict[str, int] = {}
    main = None
    current = None
    for line in emit_lf_system(lf_system).splitlines():
        m = _CLASS_LINE.match(line)
        if m:
            current = m.group(2)
            reactions[current] = 0
            if m.group(1):
                main = current
        elif current is not None and _REACTION_LINE.match(line):
            reactions[current] += 1
    by_name = {r.name: r for r in lf_system.reactors}
    instances: Counter[str] = Counter({lf_system.name: 1})
    todo = [i.reactor_class for i in lf_system.main_instances]
    while todo:
        cls = todo.pop()
        instances[cls] += 1
        todo += [i.reactor_class for i in by_name[cls].instances] if cls in by_name else []
    rows = []
    for cls, n in sorted(reactions.items()):
        if n and instances[cls]:
            c_name = lfc_class_name(cls) + ("_main" if cls == main else "")
            rows.append((c_name, n, instances[cls]))
    return rows


def emit_cpp(lf_system: LFSystem | None) -> tuple[list[str], list[str]]:
    """``(includes, body lines)`` for ros2_lf_reaction_ids_init.cpp."""
    if lf_system is None:
        return [], []

    def q(text: str) -> str:
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'

    ports = port_rows(lf_system)
    # lfc's own class .cpp includes schedule.h first: the class header carries
    # the top-level preamble, whose wake thunks call lf_schedule.
    includes = ['#include <cstddef>', '#include "ros2_lf_binding/readback_api.h"',
                'extern "C" {', '#include "include/api/schedule.h"', '}']
    includes += [f'#include "{lfc_class_name(c)}.h"' for c in sorted({c for c, _ in ports})]
    body = [f"  ros2_lf_binding::register_lf_class({q(c)}, {n}, {k});"
            for c, n, k in class_rows(lf_system)]
    for cls, port in ports:
        c = lfc_class_name(cls)
        body.append(f"  ros2_lf_binding::register_lf_port({q(c)}, "
                    f"offsetof({c}_self_t, _lf__{port}), {q(port)});")
    for row in link_rows(lf_system):
        body.append(
            "  ros2_lf_binding::register_lf_link({"
            + ", ".join(q(row[k]) for k in ("kind", "src", "dst", "name", "carrier",
                                            "src_instance", "dst_instance", "dst_port"))
            + f", {row['edge']}}});")
    return includes, body
