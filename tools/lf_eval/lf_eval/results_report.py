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
"""``ros2 lf report``: Markdown or static HTML from an eval dir (R7).

``ros2 lf report <results...> [--html]``: static HTML first. Input is a results folder (its
newest ``eval/<UTC>/``) or an eval dir written by ``ros2 lf eval``.

Before anything is rendered the eval dir is checked against its own
``SHA256SUMS``: every listed file (including the line pinning the raw
``../../SHA256SUMS``) must hash as recorded, and no unlisted file may sit in the
eval dir. A failure names each file and exits 4, as step 0 of the eval does.

The HTML page is self-contained: inline CSS, metric plots as ``data:`` URIs,
comparison plots as inline SVG, no script and no external URL. Every string
taken from the data is HTML-escaped. Every value taken from ``metrics.json``
sits in an element whose ``data-path`` attribute is its JSON pointer (RFC 6901)
into ``metrics.json`` and whose text is the value written in full precision
(``repr`` for floats), so a reader (and the test) can check each number
against the file it came from. The page shows no number that is not such a
value (no counts or totals computed at render time).
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import html
import json
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath
from typing import Any, Optional, Sequence

from lf_eval.results_eval import (EXIT_REJECTED, EXIT_USAGE, SUMS, EvalError,
                                  render_report, sha256_file)

METRICS_JSON = "metrics.json"
#: leaves of one metric's ``data`` shown in the HTML; the rest stay in metrics.json
DATA_LEAF_CAP = 400


# ───────────────────────────────────────────────────────────── locate + verify


def find_eval_dir(path: Path) -> Path:
    """*path* itself if it is an eval dir, else the newest ``path/eval/<UTC>``.

    *path* is an eval dir when it holds ``metrics.json`` or sits directly under an
    ``eval/`` folder; the latter lets a deleted ``metrics.json`` be named by the
    check against ``SHA256SUMS`` instead of turning into "no eval dir".

    No component of *path* as supplied may be a symlink (``eval-link/run`` with
    ``eval-link`` a symlink is refused like a symlinked leaf).
    """
    path = Path(path)
    for i in range(1, len(path.parts) + 1):
        part = Path(*path.parts[:i])
        if part.is_symlink():
            raise EvalError(f"{part} is a symlink; refusing to read through it",
                            EXIT_REJECTED)
    if (path / METRICS_JSON).is_file() or (path.parent.name == "eval" and path.is_dir()):
        return path
    ev = path / "eval"
    if ev.is_symlink():
        raise EvalError(f"{ev} is a symlink; refusing to read through it", EXIT_REJECTED)
    dirs = sorted(d for d in ev.iterdir() if d.is_dir() and (d / METRICS_JSON).is_file()) \
        if ev.is_dir() else []
    if not dirs:
        raise EvalError(f"{path}: no eval dir (no {METRICS_JSON} here or under eval/*/); "
                        "run `ros2 lf eval` first")
    if dirs[-1].is_symlink():
        raise EvalError(f"{dirs[-1]} is a symlink; refusing to read through it", EXIT_REJECTED)
    return dirs[-1]


@dataclass
class Sealed:
    """An eval dir that verified: the bytes that were hashed, kept for rendering."""

    evdir: Path
    doc: dict[str, Any]
    blobs: dict[str, bytes] = field(default_factory=dict)
    #: results folders whose raw SHA256SUMS the eval dir pins
    roots: list[Path] = field(default_factory=list)


def _inside(root: Path, p: Path) -> bool:
    """True when *p* resolves to a path below *root*. A path that cannot be
    resolved (a symlink loop raises RuntimeError on Python < 3.13, OSError
    after) counts as outside."""
    try:
        return root in p.resolve().parents
    except (OSError, RuntimeError):
        return False


def _check(evdir: Path) -> tuple[list[str], dict[str, bytes], list[Path]]:
    sums = evdir / SUMS
    if sums.is_symlink() or not sums.is_file():
        return [f"missing: {SUMS}"], {}, []
    problems: list[str] = []
    listed: set[str] = set()
    blobs: dict[str, bytes] = {}
    # digest of a pinned raw SHA256SUMS -> every folder whose manifest hashes so
    raw: dict[str, list[Path]] = {}
    root = evdir.resolve()
    for n, line in enumerate(sums.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        digest, sep, rel = line.partition("  ")
        if not sep or len(digest) != 64:
            problems.append(f"malformed {SUMS} line {n}")
            continue
        p = evdir / rel
        # inside by resolved containment: plots/../../x.png is outside
        inside = _inside(root, p)
        key = os.path.normpath(rel) if inside else rel
        if inside:
            listed.add(PurePosixPath(key).as_posix())
        if p.is_symlink():
            problems.append(f"symlink: {rel}")
        elif not p.is_file():
            problems.append(f"missing: {rel}")
        elif inside:
            # hash the very bytes that will be rendered: nothing is reopened later
            b = p.read_bytes()
            if hashlib.sha256(b).hexdigest() != digest:
                problems.append(f"changed: {rel}")
            else:
                blobs[PurePosixPath(key).as_posix()] = b
        elif sha256_file(p) != digest:
            problems.append(f"changed: {rel}")
        elif PurePosixPath(rel).name == SUMS:
            raw.setdefault(digest, []).append(p.parent.resolve())
    for p in sorted(evdir.rglob("*")):
        rel = p.relative_to(evdir).as_posix()
        if p.is_symlink() and rel not in listed:
            problems.append(f"symlink: {rel}")
        elif p.is_file() and rel != SUMS and rel not in listed:
            problems.append(f"unlisted: {rel}")
    if not problems:
        # every input metrics.json names must have its raw SHA256SUMS pinned here,
        # by the folder of that name: (folder, digest) must match one pinned line.
        # The name, not the full path, binds: an eval dir moved with its results
        # folder keeps the folder string recorded at eval time.
        try:
            inputs = json.loads(blobs[METRICS_JSON].decode("utf-8")).get("inputs") or []
        except (KeyError, ValueError, AttributeError):
            inputs = None
        if not inputs:
            problems.append(f"{METRICS_JSON} names no verified input")
        for inp in inputs or []:
            if not isinstance(inp, dict) or not any(
                    PurePosixPath(str(inp.get("folder"))).name == f.name
                    for f in raw.get(inp.get("sha256sums_sha256"), [])):
                folder = inp.get("folder") if isinstance(inp, dict) else inp
                problems.append(f"unpinned: no {SUMS} line pins the raw {SUMS} of input "
                                f"{folder}")
    return problems, blobs, sorted({f for fs in raw.values() for f in fs})


def verify_eval_dir(evdir: Path) -> list[str]:
    """Problems of *evdir* against its own SHA256SUMS ([] when it verifies)."""
    return _check(evdir)[0]


def load_verified(path: Path) -> Sealed:
    evdir = find_eval_dir(path)
    problems, blobs, roots = _check(evdir)
    if problems:
        raise EvalError(f"{evdir}: eval dir does not match its {SUMS}, nothing rendered:\n"
                        + "\n".join(f"  {p}" for p in problems), EXIT_REJECTED)
    return Sealed(evdir, json.loads(blobs[METRICS_JSON].decode("utf-8")), blobs, roots)


# ──────────────────────────────────────────────────────────────────── HTML


def ptr(*parts: Any) -> str:
    """RFC 6901 JSON pointer for *parts*."""
    return "".join("/" + str(p).replace("~", "~0").replace("/", "~1") for p in parts)


def value_text(v: Any) -> str:
    """The text a value from metrics.json is shown as (full precision)."""
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(v)
    return str(v)


def _e(s: Any) -> str:
    return html.escape("" if s is None else str(s), quote=True)


def _val(v: Any, path: str, tag: str = "span") -> str:
    """One value from metrics.json, tagged with its pointer."""
    return f'<{tag} class="v" data-path="{_e(path)}">{_e(value_text(v))}</{tag}>'


def _is_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _table(rows: Sequence[dict[str, Any]], base: str) -> str:
    if not rows:
        return ""
    cols = list(dict.fromkeys(k for r in rows for k in r))
    out = ["<table><tr>" + "".join(f"<th>{_e(c)}</th>" for c in cols) + "</tr>"]
    for i, r in enumerate(rows):
        cells = []
        for c in cols:
            if c in r:
                cls = ' class="num"' if _is_num(r[c]) else ""
                cells.append(f"<td{cls}>{_val(r[c], base + ptr(i, c))}</td>")
            else:
                cells.append("<td></td>")
        out.append("<tr>" + "".join(cells) + "</tr>")
    out.append("</table>")
    return "\n".join(out)


def _leaves(v: Any, path: str, out: list[tuple[str, Any]]) -> None:
    if isinstance(v, dict):
        for k in v:
            _leaves(v[k], path + ptr(k), out)
    elif isinstance(v, list):
        for i, x in enumerate(v):
            _leaves(x, path + ptr(i), out)
    else:
        out.append((path, v))


def _data_block(data: Any, base: str) -> str:
    if data in (None, {}, []):
        return ""
    leaves: list[tuple[str, Any]] = []
    _leaves(data, base, leaves)
    shown = leaves[:DATA_LEAF_CAP]
    rows = "".join(f'<tr><td><code>{_e(p[len(base):])}</code></td>'
                   f'<td class="num">{_val(v, p)}</td></tr>' for p, v in shown)
    more = ("<p>Further values are omitted here; see metrics.json.</p>"
            if len(leaves) > len(shown) else "")
    return ("<details><summary>data</summary><table><tr><th>pointer</th><th>value</th></tr>"
            f"{rows}</table>{more}</details>")


def _img(evdir: Path, rel: str, blobs: Optional[dict[str, bytes]] = None) -> str:
    """A plot as a data URI. With *blobs* (a verified eval dir) only the hashed
    bytes are used and the file is not reopened."""
    p = (evdir / rel)
    inside = _inside(evdir.resolve(), p)
    if not inside:
        data = None
    elif blobs is not None:
        data = blobs.get(PurePosixPath(os.path.normpath(str(rel))).as_posix())
    elif p.is_file():
        data = p.read_bytes()
    else:
        data = None
    if data is None:
        return f"<p>plot {_e(rel)} not found in the eval dir</p>"
    mime = {".svg": "image/svg+xml", ".png": "image/png"}.get(p.suffix.lower())
    if mime is None:
        return f"<p>plot {_e(rel)}: unsupported type</p>"
    b64 = base64.b64encode(data).decode("ascii")
    return f'<figure><img alt="{_e(rel)}" src="data:{mime};base64,{b64}"></figure>'


def _label_col(rows: Sequence[dict[str, Any]]) -> Optional[str]:
    for c in dict.fromkeys(k for r in rows for k in r):
        if all(not _is_num(r.get(c)) for r in rows):
            return c
    return None


def compare_svg(rows: Sequence[dict[str, Any]], base: str) -> str:
    """Inline SVG bar chart of each row's ``delta`` (candidate - baseline).

    Labels and values are text elements tagged with their pointer, like every
    other value on the page. Rows whose delta is not a finite number are left
    out of the chart (they are still in the table).
    """
    label = _label_col(rows)
    pts = [(i, r) for i, r in enumerate(rows)
           if _is_num(r.get("delta")) and math.isfinite(r["delta"])]
    if not pts or label is None:
        return ""
    mx = max(abs(r["delta"]) for _, r in pts) or 1.0
    row_h, left, half, pad = 22, 200, 160, 70
    width = left + 2 * half + pad
    height = row_h * len(pts) + 10
    zero = left + half
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" class="cmp" width="{width}" '
           f'height="{height}" role="img"><title>delta = candidate - baseline</title>',
           f'<line x1="{zero}" y1="0" x2="{zero}" y2="{height}" stroke="#555"/>']
    for k, (i, r) in enumerate(pts):
        y = 5 + k * row_h
        d = r["delta"]
        w = abs(d) / mx * half
        x = zero if d >= 0 else zero - w
        fill = "#2f6fb0" if d >= 0 else "#c0504d"
        out.append(f'<text x="{left - 8}" y="{y + 14}" text-anchor="end" class="v" '
                   f'data-path="{_e(base + ptr(i, label))}">{_e(value_text(r.get(label)))}</text>')
        out.append(f'<rect x="{x:.2f}" y="{y + 3}" width="{max(w, 1):.2f}" height="{row_h - 8}" '
                   f'fill="{fill}"/>')
        tx = zero + half + 6
        out.append(f'<text x="{tx}" y="{y + 14}" class="v" '
                   f'data-path="{_e(base + ptr(i, "delta"))}">{_e(value_text(d))}</text>')
    out.append("</svg>")
    return "\n".join(out)


_CSS = """body{font-family:system-ui,sans-serif;margin:2em;max-width:80em;color:#1b1b1b}
table{border-collapse:collapse;margin:.5em 0 1em}th,td{border:1px solid #ccc;padding:.2em .5em;
text-align:left;vertical-align:top}th{background:#f2f2f2}td.num{text-align:right;
font-variant-numeric:tabular-nums}code{font-size:.95em}.st-ok{color:#1a7f37}
.st-unavailable,.st-not_applicable{color:#57606a}.st-error,.st-refused{color:#cf222e}
figure{margin:.5em 0}img{max-width:100%}svg.cmp text{font:12px system-ui,sans-serif}"""


def _status(s: Any, path: str) -> str:
    return f'<span class="st-{_e(s)}">{_val(s, path)}</span>'


def _metric(name: str, m: dict[str, Any], base: str, evdir: Path,
            blobs: Optional[dict[str, bytes]] = None) -> list[str]:
    out = [f"<h4>{_e(name)}: {_status(m.get('status'), base + ptr('status'))}</h4>"]
    if m.get("note"):
        out.append(f"<p>{_val(m['note'], base + ptr('note'))}</p>")
    if m.get("summary"):
        out.append(_table(m["summary"], base + ptr("summary")))
    out.append(_data_block(m.get("data"), base + ptr("data")))
    for rel in m.get("plots") or []:
        out.append(_img(evdir, rel, blobs))
    return out


def render_html(doc: dict[str, Any], evdir: Path,
                blobs: Optional[dict[str, bytes]] = None) -> str:
    """The static report page for one eval dir's ``metrics.json``."""
    return _page([_section(doc, evdir, blobs)])


def render_html_many(items: Sequence[Sealed]) -> str:
    return _page([_section(it.doc, it.evdir, it.blobs) for it in items])


def _page(sections: Sequence[str]) -> str:
    return "\n".join(["<!DOCTYPE html>", '<html lang="en"><head><meta charset="utf-8">',
                      "<title>ros2 lf report</title>", f"<style>{_CSS}</style></head><body>",
                      "<h1>ros2 lf report</h1>", *sections, "</body></html>"]) + "\n"


def _section(doc: dict[str, Any], evdir: Path,
             blobs: Optional[dict[str, bytes]] = None) -> str:
    out = [f"<section><h2>Eval dir <code>{_e(evdir.name)}</code></h2>",
           f"<p>Schema {_val(doc.get('schema'), ptr('schema'))}, lf_eval "
           f"{_val(doc.get('lf_eval_version'), ptr('lf_eval_version'))}, generated "
           f"{_val(doc.get('generated_at'), ptr('generated_at'))}.</p>",
           "<h3>Verified inputs</h3><table><tr><th>results folder</th>"
           "<th>raw SHA256SUMS sha256</th><th>missing files waived</th></tr>"]
    for i, inp in enumerate(doc.get("inputs") or []):
        waived = inp.get("waived_missing") or []
        w = ("<details><summary>waived (--allow-missing "
             + ", ".join(_val(g, ptr("inputs", i, "allow_missing", j))
                         for j, g in enumerate(inp.get("allow_missing") or []))
             + ")</summary><ul>"
             + "".join(f"<li>{_val(x, ptr('inputs', i, 'waived_missing', j))}</li>"
                       for j, x in enumerate(waived))
             + "</ul></details>") if waived else "none"
        out.append(f"<tr><td><code>{_val(inp.get('folder'), ptr('inputs', i, 'folder'))}</code></td>"
                   f"<td><code>{_val(inp.get('sha256sums_sha256'), ptr('inputs', i, 'sha256sums_sha256'))}"
                   f"</code></td><td>{w}</td></tr>")
    out.append("</table>")

    for key, g in (doc.get("groups") or {}).items():
        gb = ptr("groups", key)
        out.append(f"<h3>Group <code>{_e(key)}</code></h3>")
        out.append("<table><tr><th>kind</th><th>variant</th><th>point</th><th>build</th>"
                   "<th>runs used</th><th>runs excluded</th></tr><tr>"
                   + "".join(f"<td>{_val(g.get(k), gb + ptr(k))}</td>"
                             for k in ("kind", "variant", "point", "build_ref"))
                   + "<td>" + "<br>".join(_val(r, gb + ptr("runs_used", j))
                                          for j, r in enumerate(g.get("runs_used") or []))
                   + "</td><td>"
                   + "<br>".join(_val(r.get("run"), gb + ptr("runs_excluded", j, "run"))
                                 for j, r in enumerate(g.get("runs_excluded") or []))
                   + "</td></tr></table>")
        for name, m in (g.get("metrics") or {}).items():
            out += _metric(name, m, gb + ptr("metrics", name), evdir, blobs)
    if doc.get("cross_group"):
        out.append("<h3>Cross-group metrics</h3>")
        for name, m in doc["cross_group"].items():
            out += _metric(name, m, ptr("cross_group", name), evdir, blobs)
    if doc.get("compare") is not None:
        out.append(f"<h3>Comparison against baseline {_val(doc.get('baseline'), ptr('baseline'))}"
                   "</h3><p>delta = candidate - baseline</p>")
        for i, c in enumerate(doc["compare"]):
            cb = ptr("compare", i)
            out.append(f"<h4>{_val(c.get('metric'), cb + ptr('metric'))}: "
                       f"{_val(c.get('candidate'), cb + ptr('candidate'))} vs "
                       f"{_val(c.get('baseline'), cb + ptr('baseline'))} "
                       f"({_status(c.get('status'), cb + ptr('status'))})</h4>")
            if c.get("status") == "ok":
                out.append(compare_svg(c.get("rows") or [], cb + ptr("rows")))
                out.append(_table(c.get("rows") or [], cb + ptr("rows")))
            elif c.get("reason"):
                out.append(f"<p>{_val(c['reason'], cb + ptr('reason'))}</p>")
    out.append("</section>")
    return "\n".join(x for x in out if x)


# ──────────────────────────────────────────────────────────────────────── CLI


def add_report_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("results", nargs="+", type=Path, metavar="RESULTS",
                        help="results folder (its newest eval/<UTC>/) or an eval dir")
    parser.add_argument("--html", action="store_true",
                        help="static self-contained HTML instead of Markdown")
    parser.add_argument("-o", "--out", type=Path, metavar="FILE",
                        help="write to FILE instead of stdout")


def report_command(args: argparse.Namespace) -> int:
    try:
        items = [load_verified(p) for p in args.results]
    except EvalError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return exc.code
    except (OSError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_USAGE
    if args.html:
        text = render_html_many(items)
    else:
        text = "\n".join(render_report(it.doc) for it in items)
    if args.out:
        out = Path(args.out).resolve()
        for it in items:
            # the eval dir and every results folder it pins are sealed by a SHA256SUMS
            for sealed, what in [(it.evdir, "eval dir")] + [(r, "results folder")
                                                           for r in it.roots]:
                if sealed.resolve() in out.parents:
                    print(f"error: --out {args.out} lies inside {what} {sealed}, which its "
                          f"{SUMS} seals; write the report elsewhere", file=sys.stderr)
                    return EXIT_USAGE
        Path(args.out).write_text(text, encoding="utf-8")
        print(f"Written: {args.out}", file=sys.stderr)
    else:
        sys.stdout.write(text)
    return 0


__all__ = ["add_report_arguments", "compare_svg", "find_eval_dir", "load_verified",
           "ptr", "render_html", "render_html_many", "report_command", "Sealed",
           "value_text", "verify_eval_dir"]
