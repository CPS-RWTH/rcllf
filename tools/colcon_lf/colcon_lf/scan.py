# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""``ros2 lf scan``: what a workspace holds and whether each launch file converts.

Read-only: every launch file is
analyzed in-process with the same graph builder, incomplete-AST check and
feature registry as ``colcon lf-analyze``, but nothing is written under
``lf-analysis/``. The result is one JSON document (``rcllf.scan/1``), printed as
a table, as JSON, or as a static self-contained HTML page.

Convertible verdict per launch file:

- ``yes``: the graph was built, no node's AST is incomplete, no diagnostic of
  severity error or warning.
- ``warn``: as ``yes`` but with warning diagnostics, launch-analysis errors or
  launched nodes outside the analysis (``colcon lf-generate`` proceeds).
- ``no``: the analyzer refused the launch file (a node it could not analyze,
  an incomplete AST) or an error-severity diagnostic (the generate hazard gate
  refuses it unless acknowledged in ``execution.yaml``).
"""

from __future__ import annotations

import argparse
import html
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional, Sequence

SCAN_SCHEMA = "rcllf.scan/1"


def _rel(path: Path, root: Path) -> str:
    try:
        return Path(path).resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return str(path)


def _launch_nodes(file_info) -> list[dict[str, Any]]:
    """Every node the launch description starts, includes and groups inlined.

    Depth first: a scope's own nodes, then each include, then each group. Within each
    list the order is the launch file's, but a node written after an include is listed
    before it.
    """
    out: list[dict[str, Any]] = []

    def walk(scope) -> None:
        for n in scope.nodes:
            out.append({"package": n.package, "executable": n.executable,
                        "name": n.name, "namespace": n.namespace,
                        "is_component": bool(n.is_component),
                        "condition_result": n.condition_result})
        for child in scope.includes:
            walk(child)
        for grp in scope.groups:
            walk(grp)

    walk(file_info)
    return out


def scan_launch(package: str, launch_file: str, launch_path: Path,
                workspace: Path) -> dict[str, Any]:
    """Analyze one launch file in memory and describe it (no files written)."""
    from colcon_lf.comm_analysis.endpoints import endpoint_key_map
    from colcon_lf.graph.hierarchical import SkippedNodesError, build_system_graph
    from colcon_lf.verb.analyze import _launch_errors, incomplete_reasons

    entry: dict[str, Any] = {
        "package": package, "launch_file": launch_file,
        "path": _rel(launch_path, workspace),
        "convertible": "no", "reasons": [], "nodes": [], "launched": [],
        "topics": [], "services": [], "actions": [], "not_analyzed": [],
        "launch_errors": [], "diagnostics": [],
    }
    not_analyzed: list[str] = []
    try:
        graph = build_system_graph(launch_path, workspace_root=workspace,
                                   not_analyzed=not_analyzed)
    except SkippedNodesError as exc:
        entry["not_analyzed"] = list(not_analyzed)
        entry["reasons"] = list(exc.reasons)
        return entry
    except Exception as exc:  # the analyzer's own failure is a scan finding
        entry["not_analyzed"] = list(not_analyzed)
        entry["reasons"] = [f"analysis failed: {type(exc).__name__}: {exc}"]
        return entry

    entry["not_analyzed"] = list(not_analyzed)
    entry["launch_errors"] = [{"file": _rel(Path(f), workspace), "error": e}
                              for f, e in _launch_errors(graph.launch_graph.root)]
    entry["launched"] = _launch_nodes(graph.launch_graph.root)

    comm = graph.communication
    keymap = endpoint_key_map(graph)

    def keys(names: Sequence[str]) -> list[str]:
        return sorted(keymap.get(n, n) for n in names if n)

    topics = sorted(comm.topics, key=lambda t: t.topic)
    entry["topics"] = [{"topic": t.topic, "message_type": t.message_type,
                        "publishers": keys(t.publishers), "subscribers": keys(t.subscribers)}
                       for t in topics]
    entry["services"] = [{"service": s.service, "service_type": s.service_type,
                          "server": keymap.get(s.server, s.server), "clients": keys(s.clients)}
                         for s in sorted(comm.services, key=lambda s: s.service)]
    entry["actions"] = [{"action": a.action, "action_type": a.action_type,
                         "server": keymap.get(a.server, a.server), "clients": keys(a.clients)}
                        for a in sorted(comm.actions, key=lambda a: a.action)]
    for key, ni in sorted(graph.nodes.items()):
        entry["nodes"].append({
            "key": key, "package": ni.package, "class_name": ni.class_name,
            "is_component": bool(ni.is_component),
            "publishes": sorted(t["topic"] for t in entry["topics"] if key in t["publishers"]),
            "subscribes": sorted(t["topic"] for t in entry["topics"] if key in t["subscribers"]),
            "timers": len(ni.timers),
        })

    incomplete = incomplete_reasons(graph)
    from colcon_lf.features.diagnostics import DetectionContext
    from colcon_lf.features.registry import run_detection, severity_for

    for d in run_detection(DetectionContext(graph=graph)):
        try:
            sev = severity_for(d.code).value
        except KeyError:
            sev = "error"  # an unowned code is not waved through
        entry["diagnostics"].append(dict(d.to_dict(), severity=sev))
    errors = [d for d in entry["diagnostics"] if d["severity"] == "error"]
    warnings = [d for d in entry["diagnostics"] if d["severity"] == "warning"]
    if incomplete or errors:
        entry["reasons"] = incomplete + [f"{d['code']} (error): {d['message']}" for d in errors]
        return entry
    soft = ([f"{d['code']} (warning): {d['message']}" for d in warnings]
            + [f"launch error in {e['file']}: {e['error']}" for e in entry["launch_errors"]]
            + [f"not analyzed: {r}" for r in not_analyzed])
    entry["convertible"] = "warn" if soft else "yes"
    entry["reasons"] = soft
    return entry


def scan_workspace(workspace: Path, package: Optional[str] = None,
                   launch_file: Optional[str] = None) -> dict[str, Any]:
    """``rcllf.scan/1`` for *workspace* (the colcon root holding ``src/``)."""
    from colcon_lf.verb.analyze import (
        _collision_message, _discover_targets, _find_packages, _output_collisions)

    workspace = Path(workspace)
    src = workspace / "src"
    pkgs = _find_packages(src)
    if package is not None and package not in pkgs:
        raise ValueError(f"package {package!r} not found under {src}")
    targets = _discover_targets(src, package, launch_file)
    if launch_file is not None and not targets:
        raise ValueError(f"launch file {launch_file!r} not found in package {package!r}")
    by_pkg: dict[str, list[dict[str, Any]]] = {
        name: [] for name in sorted(pkgs) if package is None or name == package}
    for pkg, lf, path in targets:
        by_pkg[pkg].append(scan_launch(pkg, lf, path, workspace))
    # Two launch files with one output directory: lf-analyze of the whole
    # package refuses them, one at a time with --launch-file works.
    for pkg, stem, names in _output_collisions(targets):
        reason = (_collision_message(pkg, stem, names)
                  + ", so lf-analyze --package refuses them; select one with --launch-file")
        for entry in by_pkg[pkg]:
            if entry["launch_file"] in names:
                entry["reasons"].append(reason)
                if entry["convertible"] == "yes":
                    entry["convertible"] = "warn"
    launches = [e for v in by_pkg.values() for e in v]
    return {
        "schema": SCAN_SCHEMA,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "workspace": str(workspace),
        "packages": [{"name": name, "path": _rel(pkgs[name], workspace), "launch_files": v}
                     for name, v in by_pkg.items()],
        "summary": {
            "packages": len(by_pkg),
            "launch_files": len(launches),
            "convertible": {k: sum(1 for e in launches if e["convertible"] == k)
                            for k in ("yes", "warn", "no")},
        },
    }


# ────────────────────────────────────────────────────────────────── renderers


def render_text(doc: dict[str, Any]) -> str:
    lines = [f"workspace {doc['workspace']}: {doc['summary']['packages']} packages, "
             f"{doc['summary']['launch_files']} launch files "
             + ", ".join(f"{k}={v}" for k, v in doc["summary"]["convertible"].items()), ""]
    for p in doc["packages"]:
        if not p["launch_files"]:
            lines.append(f"{p['name']}: no launch files")
            continue
        for e in p["launch_files"]:
            lines.append(f"{p['name']}/{e['launch_file']}: convertible={e['convertible']}, "
                         f"{len(e['nodes'])} nodes, {len(e['topics'])} topics")
            for n in e["launched"]:
                lines.append(f"  launched {n['package']}/{n['executable']} "
                             f"name={n['name']} namespace={n['namespace']}")
            for n in e["nodes"]:
                lines.append(f"  node {n['key']} ({n['class_name']}) "
                             f"pub {n['publishes']} sub {n['subscribes']} timers {n['timers']}")
            for t in e["topics"]:
                lines.append(f"  topic {t['topic']} [{t['message_type']}] "
                             f"{t['publishers']} -> {t['subscribers']}")
            for kind, typ in (("services", "service"), ("actions", "action")):
                for x in e[kind]:
                    lines.append(f"  {typ} {x[typ]} [{x[typ + '_type']}] "
                                 f"server {x['server']} clients {x['clients']}")
            for n in e["not_analyzed"]:
                lines.append(f"  not analysed: {n}")
            for r in e["reasons"]:
                lines.append(f"  ! {r}")
    return "\n".join(lines) + "\n"


_CSS = """body{font-family:system-ui,sans-serif;margin:2em;max-width:72em;color:#1b1b1b}
table{border-collapse:collapse;margin:.5em 0 1em}th,td{border:1px solid #ccc;padding:.2em .5em;
text-align:left;vertical-align:top}th{background:#f2f2f2}code{font-size:.95em}
.yes{color:#1a7f37}.warn{color:#9a6700}.no{color:#cf222e}h3{margin-top:1.5em}"""


def _e(v: Any) -> str:
    return html.escape("" if v is None else str(v), quote=True)


def _list(items: Sequence[Any]) -> str:
    return ", ".join(f"<code>{_e(i)}</code>" for i in items) or "&ndash;"


def render_html(doc: dict[str, Any]) -> str:
    """Static, self-contained page: inline CSS, no script, no external URL."""
    s = doc["summary"]
    out = ["<!DOCTYPE html>", '<html lang="en"><head><meta charset="utf-8">',
           "<title>ros2 lf scan</title>", f"<style>{_CSS}</style></head><body>",
           "<h1>ros2 lf scan</h1>",
           f"<p>Workspace <code>{_e(doc['workspace'])}</code>, generated {_e(doc['generated_at'])}"
           f" ({_e(doc['schema'])}).</p>",
           "<table><tr><th>packages</th><th>launch files</th><th>convertible yes</th>"
           "<th>warn</th><th>no</th></tr>",
           f"<tr><td>{_e(s['packages'])}</td><td>{_e(s['launch_files'])}</td>"
           + "".join(f"<td>{_e(s['convertible'][k])}</td>" for k in ("yes", "warn", "no"))
           + "</tr></table>",
           "<h2>Launch files</h2>",
           "<table><tr><th>package</th><th>launch file</th><th>convertible</th>"
           "<th>nodes</th><th>topics</th></tr>"]
    for p in doc["packages"]:
        if not p["launch_files"]:
            out.append(f"<tr><td>{_e(p['name'])}</td><td colspan=\"4\">no launch files</td></tr>")
        for e in p["launch_files"]:
            c = e["convertible"]
            out.append(f"<tr><td>{_e(p['name'])}</td><td><code>{_e(e['launch_file'])}</code></td>"
                       f'<td class="{_e(c)}">{_e(c)}</td><td>{len(e["nodes"])}</td>'
                       f"<td>{len(e['topics'])}</td></tr>")
    out.append("</table>")
    for p in doc["packages"]:
        for e in p["launch_files"]:
            c = e["convertible"]
            out.append(f"<h3>{_e(p['name'])} / {_e(e['launch_file'])}: "
                       f'<span class="{_e(c)}">{_e(c)}</span></h3>')
            if e["reasons"]:
                out.append("<ul>" + "".join(f"<li>{_e(r)}</li>" for r in e["reasons"]) + "</ul>")
            if e["launched"]:
                out.append("<table><tr><th>launched</th><th>package</th><th>executable</th>"
                           "<th>namespace</th></tr>")
                out += [f"<tr><td><code>{_e(n['name'])}</code></td><td>{_e(n['package'])}</td>"
                        f"<td>{_e(n['executable'])}</td><td>{_e(n['namespace'])}</td></tr>"
                        for n in e["launched"]]
                out.append("</table>")
            if e["not_analyzed"]:
                out.append("<p>Not analysed: " + _list(e["not_analyzed"]) + "</p>")
            if e["nodes"]:
                out.append("<table><tr><th>node</th><th>package</th><th>class</th>"
                           "<th>publishes</th><th>subscribes</th><th>timers</th></tr>")
                out += [f"<tr><td><code>{_e(n['key'])}</code></td><td>{_e(n['package'])}</td>"
                        f"<td>{_e(n['class_name'])}</td><td>{_list(n['publishes'])}</td>"
                        f"<td>{_list(n['subscribes'])}</td><td>{_e(n['timers'])}</td></tr>"
                        for n in e["nodes"]]
                out.append("</table>")
            if e["topics"]:
                out.append("<table><tr><th>topic</th><th>type</th><th>publishers</th>"
                           "<th>subscribers</th></tr>")
                out += [f"<tr><td><code>{_e(t['topic'])}</code></td><td>{_e(t['message_type'])}</td>"
                        f"<td>{_list(t['publishers'])}</td><td>{_list(t['subscribers'])}</td></tr>"
                        for t in e["topics"]]
                out.append("</table>")
            for kind, typ in (("services", "service"), ("actions", "action")):
                if e[kind]:
                    out.append(f"<table><tr><th>{typ}</th><th>type</th><th>server</th>"
                               "<th>clients</th></tr>")
                    out += [f"<tr><td><code>{_e(x[typ])}</code></td>"
                            f"<td>{_e(x[typ + '_type'])}</td><td>{_list([x['server']])}</td>"
                            f"<td>{_list(x['clients'])}</td></tr>" for x in e[kind]]
                    out.append("</table>")
            if e["diagnostics"]:
                out.append("<table><tr><th>code</th><th>severity</th><th>node</th>"
                           "<th>topic</th><th>message</th></tr>")
                out += [f"<tr><td>{_e(d['code'])}</td><td>{_e(d['severity'])}</td>"
                        f"<td>{_e(d['node'])}</td><td>{_e(d['topic'])}</td>"
                        f"<td>{_e(d['message'])}</td></tr>" for d in e["diagnostics"]]
                out.append("</table>")
    out.append("</body></html>")
    return "\n".join(out) + "\n"


# ──────────────────────────────────────────────────────────────────────── CLI


def add_scan_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--workspace", default=None,
                        help="colcon workspace root holding src/ (default: ./packages, . or ..)")
    parser.add_argument("--package", default=None, help="scan only this package")
    parser.add_argument("--launch-file", default=None,
                        help="scan only this launch file of --package")
    parser.add_argument("--json", action="store_true", help="print the rcllf.scan/1 JSON")
    parser.add_argument("--html", type=Path, metavar="FILE",
                        help="also write a static self-contained HTML page")
    parser.add_argument("-o", "--out", type=Path, metavar="FILE",
                        help="also write the rcllf.scan/1 JSON to FILE")


def scan_command(args: argparse.Namespace) -> int:
    from colcon_lf.verb.workspace import resolve_workspace

    if args.launch_file and not args.package:
        print("error: --launch-file needs --package", file=sys.stderr)
        return 2
    workspace = resolve_workspace(args.workspace)
    if workspace is None or not (workspace / "src").is_dir():
        print("error: no colcon workspace with src/ found; use --workspace", file=sys.stderr)
        return 2
    try:
        doc = scan_workspace(workspace, args.package, args.launch_file)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    text = json.dumps(doc, indent=2, sort_keys=True) + "\n"
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
    if args.html:
        Path(args.html).write_text(render_html(doc), encoding="utf-8")
    sys.stdout.write(text if args.json else render_text(doc))
    return 0
