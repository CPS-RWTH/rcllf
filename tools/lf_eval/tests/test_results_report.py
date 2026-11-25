# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``ros2 lf report [--html]`` over eval dirs (run spec slice R7).

The number check parses the HTML with the standard library's ``html.parser``
and resolves each value's ``data-path`` pointer in ``metrics.json`` with its
own pointer walk (written here, not imported from the renderer). A number is
compared as a number: ``json.loads`` of the shown text must equal the
``metrics.json`` value. Text outside the tagged elements must hold no number.

Inputs: the eval dir recorded in the dev container over the R3 fixture
(``tools/lf_bench/tests/fixtures/runspec_r3/.../eval/20261111T224727Z``, R6).
In the development repository, one more test compares a fresh render with the HTML
``ros2 lf report --html`` wrote from that eval dir in the dev container. Both inputs
are development fixtures; the tests that need them skip without them.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
from html.parser import HTMLParser
from pathlib import Path

import pytest

from lf_eval.results_eval import EXIT_REJECTED, evaluate
from lf_eval.results_report import (add_report_arguments, compare_svg, render_html,
                                    report_command)

from .dev_only import requires_lf_bench

needs_lf_bench = requires_lf_bench("the recorded R3 results folder, tests/fixtures/runspec_r3")

REPO = Path(__file__).resolve().parents[3]
R3 = REPO / "tools/lf_bench/tests/fixtures/runspec_r3/20261110T152946Z_sentry-r3"
EVAL = R3 / "eval/20261111T224727Z"
TRIMMED = ("runs/*/tracing/*", "build/*/compile_commands.json")
NUMBER = re.compile(r"^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$")


# ── an independent reader of the page ──────────────────────────────────────


class _Page(HTMLParser):
    """Collects (data-path, text) of tagged values, the text outside them,
    and every URL-bearing attribute."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.values: list[tuple[str, str]] = []
        self.loose: list[str] = []
        self.urls: list[tuple[str, str, str]] = []
        self.tags: set[str] = set()
        self._stack: list[tuple[str, str | None]] = []
        self._buf: list[str] = []
        self._style = False

    def handle_starttag(self, tag, attrs):
        self.tags.add(tag)
        a = dict(attrs)
        for k in ("src", "href", "xlink:href", "action", "srcset"):
            if k in a:
                self.urls.append((tag, k, a[k] or ""))
        if tag == "style":
            self._style = True
        path = a.get("data-path")
        if path is not None:
            self._buf = []
        self._stack.append((tag, path))

    def handle_endtag(self, tag):
        if tag == "style":
            self._style = False
        while self._stack:
            t, path = self._stack.pop()
            if path is not None:
                self.values.append((path, "".join(self._buf)))
                self._buf = []
            if t == tag:
                break

    def handle_data(self, data):
        if self._style:
            return
        if any(p is not None for _, p in self._stack):
            self._buf.append(data)
        else:
            self.loose.append(data)


def _resolve(doc, pointer: str):
    cur = doc
    for raw in pointer.split("/")[1:]:
        key = raw.replace("~1", "/").replace("~0", "~")
        cur = cur[int(key)] if isinstance(cur, list) else cur[key]
    return cur


def html_mismatches(page_html: str, doc: dict) -> list[str]:
    """Every shown value that differs from metrics.json, and every loose number."""
    p = _Page()
    p.feed(page_html)
    bad = []
    for path, text in p.values:
        try:
            want = _resolve(doc, path)
        except (KeyError, IndexError, ValueError):
            bad.append(f"{path}: not in metrics.json")
            continue
        if isinstance(want, str):
            if text != want:
                bad.append(f"{path}: HTML shows {text!r}, metrics.json has {want!r}")
            continue
        try:
            shown = json.loads(text)
        except ValueError:
            bad.append(f"{path}: HTML shows {text!r}, metrics.json has {want!r}")
            continue
        if shown != want or type(shown) is not type(want):
            bad.append(f"{path}: HTML shows {text!r}, metrics.json has {want!r}")
    for chunk in p.loose:
        for tok in chunk.split():
            tok = tok.strip("().,:;")
            if NUMBER.match(tok):
                bad.append(f"untagged number {tok!r} in text {chunk.strip()!r}")
    return bad


def _leaf_pointers(v, path=""):
    if isinstance(v, dict):
        for k, x in v.items():
            yield from _leaf_pointers(x, path + "/" + k.replace("~", "~0").replace("/", "~1"))
    elif isinstance(v, list):
        for i, x in enumerate(v):
            yield from _leaf_pointers(x, f"{path}/{i}")
    else:
        yield path


def _shown_paths(page_html: str) -> set[str]:
    p = _Page()
    p.feed(page_html)
    return {path for path, _ in p.values}


# ── the proof: every number in the HTML equals metrics.json ───────────────


def _run_report(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    add_report_arguments(parser)
    return report_command(parser.parse_args(argv))


@needs_lf_bench
def test_report_html_numbers_equal_metrics_json(tmp_path: Path) -> None:
    out = tmp_path / "r.html"
    assert _run_report([str(R3), "--html", "-o", str(out)]) == 0
    page = out.read_text()
    doc = json.loads((EVAL / "metrics.json").read_text())
    assert html_mismatches(page, doc) == []
    shown = _shown_paths(page)
    # every summary cell, every compare row cell and every data leaf is on the page
    want = set()
    for key, g in doc["groups"].items():
        for name, m in g["metrics"].items():
            base = f"/groups/{key}/metrics/{name}"
            want |= set(_leaf_pointers(m.get("summary") or [], base + "/summary"))
            want |= set(_leaf_pointers(m.get("data") or {}, base + "/data"))
    for i, c in enumerate(doc["compare"]):
        want |= set(_leaf_pointers(c["rows"], f"/compare/{i}/rows"))
    assert want - shown == set()
    # hand values from the R6 journal: lfc-delays|p0 vs lfc|p0 /sentry/state delta -2.0
    rows = doc["compare"][0]["rows"]
    i = next(j for j, r in enumerate(rows) if r["topic"] == "/sentry/state")
    assert ("/compare/0/rows/%d/delta" % i) in shown
    assert rows[i]["delta"] == -2.0
    assert '>-2.0</span>' in page




@needs_lf_bench
def test_perturbed_number_is_caught() -> None:
    """Falsification kept in the suite: one changed number on the page is named."""
    doc = json.loads((EVAL / "metrics.json").read_text())
    page = render_html(doc, EVAL)
    assert html_mismatches(page, doc) == []
    bad_page = page.replace(">-2.0</span>", ">-3.0</span>", 1)
    assert bad_page != page
    found = html_mismatches(bad_page, doc)
    assert len(found) == 1 and "HTML shows '-3.0', metrics.json has -2.0" in found[0], found
    # a number added outside a tagged element is caught too
    found = html_mismatches(page.replace("<h1>ros2 lf report</h1>",
                                         "<h1>ros2 lf report</h1><p>total 1144</p>"), doc)
    assert found == ["untagged number '1144' in text 'total 1144'"], found


@needs_lf_bench
def test_markdown_report_equals_eval_report_md(tmp_path: Path, capsys) -> None:
    """``ros2 lf report`` (Markdown) over a fresh eval dir prints its report.md."""
    folder = tmp_path / R3.name
    shutil.copytree(R3, folder, symlinks=True, ignore=shutil.ignore_patterns("eval"))
    out = evaluate([folder], echo=lambda *_: None, allow_missing=TRIMMED, baseline="lfc",
                   no_plots=True)
    capsys.readouterr()
    assert _run_report([str(out)]) == 0
    assert capsys.readouterr().out == (out / "report.md").read_text()


# ── self-contained and escaped ────────────────────────────────────────────


def _hostile_doc() -> dict:
    evil = '<script>alert("x")</script>&"\'<img src=x onerror=alert(1)>'
    return {
        "schema": "rcllf.metrics/1", "generated_at": evil, "lf_eval_version": "0.1.0",
        "inputs": [{"folder": evil, "sha256sums_sha256": "0" * 64,
                    "allow_missing": [evil], "waived_missing": [evil]}],
        "groups": {evil: {"kind": evil, "variant": evil, "point": "p0", "build_ref": evil,
                          "runs_used": [evil], "runs_excluded": [{"run": evil, "reasons": []}],
                          "metrics": {evil: {"status": evil, "note": evil, "available": True,
                                             "summary": [{evil: evil, "n": 1.5}],
                                             "data": {evil: [1, 2.25, None, True]},
                                             "plots": ["../../etc/passwd"]}}}},
        "cross_group": {}, "baseline": evil,
        "compare": [{"metric": evil, "candidate": evil, "baseline": evil, "status": "ok",
                     "rows": [{"topic": evil, "delta": -1.5, "baseline": 2, "candidate": 0.5}]},
                    {"metric": evil, "candidate": evil, "baseline": evil, "status": "refused",
                     "reason": evil}],
    }


def test_every_data_string_is_escaped(tmp_path: Path) -> None:
    doc = _hostile_doc()
    page = render_html(doc, tmp_path)
    p = _Page()
    p.feed(page)
    assert "script" not in p.tags and "img" not in p.tags, p.tags
    assert "<script" not in page and "onerror=alert" not in page.replace("onerror=alert(1)&gt;", "")
    assert html_mismatches(page, doc) == []
    # the evil string comes back verbatim from the parsed text, so it was escaped, not dropped
    evil = doc["generated_at"]
    assert sum(1 for _, t in p.values if t == evil) > 10


@needs_lf_bench
def test_page_is_self_contained(tmp_path: Path) -> None:
    """No script, no stylesheet link, no URL except data: images and the SVG namespace."""
    out = tmp_path / "r.html"
    assert _run_report([str(R3), "--html", "-o", str(out)]) == 0
    for page in (out.read_text(), render_html(_hostile_doc(), tmp_path)):
        p = _Page()
        p.feed(page)
        assert not {"script", "link", "iframe", "object", "embed"} & p.tags
        assert all(url.startswith("data:") for _, _, url in p.urls), p.urls
        assert "@import" not in page and "url(" not in page
        # plot paths outside the eval dir are never read
        assert "passwd" not in page or "not found in the eval dir" in page


def test_compare_plot_bars_follow_delta_sign() -> None:
    rows = [{"topic": "/a", "delta": -2.0}, {"topic": "/b", "delta": 4.0},
            {"topic": "/c", "delta": 0.0}]
    svg = compare_svg(rows, "/compare/0/rows")
    rects = re.findall(r'<rect x="([\d.]+)" y="[\d.]+" width="([\d.]+)"[^>]*fill="(#\w+)"', svg)
    assert len(rects) == 3
    (xa, wa, fa), (xb, wb, fb), _ = [(float(x), float(w), f) for x, w, f in rects]
    zero = 200 + 160
    assert xa + wa == pytest.approx(zero) and fa == "#c0504d"   # negative: left of zero
    assert xb == pytest.approx(zero) and fb == "#2f6fb0"        # positive: right of zero
    assert wb == pytest.approx(2 * wa)                          # |4| = 2 * |-2|


# ── the eval dir is checked before rendering ──────────────────────────────


def _place(tmp_path: Path) -> Path:
    """A copy of the recorded eval dir at <tmp>/<R3 name>/eval/<stamp>, raw sums beside it.

    The folder keeps R3's name: the report ties each input of metrics.json to the
    pinned raw SHA256SUMS of the folder of that name."""
    folder = tmp_path / R3.name
    (folder / "eval").mkdir(parents=True)
    shutil.copy(R3 / "SHA256SUMS", folder / "SHA256SUMS")
    ev = folder / "eval" / EVAL.name
    shutil.copytree(EVAL, ev)
    return ev


@needs_lf_bench
def test_changed_metrics_json_is_refused(tmp_path: Path, capsys) -> None:
    ev = _place(tmp_path)
    assert _run_report([str(ev), "--html"]) == 0
    capsys.readouterr()
    p = ev / "metrics.json"
    p.write_bytes(p.read_bytes().replace(b'"delta": -2.0', b'"delta": -3.0', 1))
    assert _run_report([str(ev), "--html"]) == EXIT_REJECTED
    err = capsys.readouterr().err
    assert "changed: metrics.json" in err and "nothing rendered" in err


@needs_lf_bench
def test_unlisted_file_and_changed_raw_sums_are_refused(tmp_path: Path, capsys) -> None:
    ev = _place(tmp_path)
    (ev / "extra.svg").write_text("<svg/>")
    (ev.parents[1] / "SHA256SUMS").write_text("tampered\n")
    assert _run_report([str(ev.parents[1])]) == EXIT_REJECTED
    err = capsys.readouterr().err
    assert "unlisted: extra.svg" in err and "changed: ../../SHA256SUMS" in err


@needs_lf_bench
def test_out_inside_the_eval_dir_is_refused(tmp_path: Path, capsys) -> None:
    ev = _place(tmp_path)
    assert _run_report([str(ev), "--html", "-o", str(ev / "x.html")]) == 2
    assert not (ev / "x.html").exists()


@needs_lf_bench
def test_eval_writes_report_html_sealed_and_equal_to_report(tmp_path: Path) -> None:
    """``ros2 lf eval`` writes report.html into its SHA256SUMS, and ``ros2 lf
    report --html`` over that eval dir reproduces it byte for byte, plots
    embedded as data: URIs."""
    folder = tmp_path / R3.name
    shutil.copytree(R3, folder, symlinks=True, ignore=shutil.ignore_patterns("eval"))
    out = evaluate([folder], echo=lambda *_: None, allow_missing=TRIMMED, baseline="lfc")
    names = {ln.partition("  ")[2] for ln in (out / "SHA256SUMS").read_text().splitlines()}
    assert "report.html" in names
    digest = hashlib.sha256((out / "report.html").read_bytes()).hexdigest()
    assert f"{digest}  report.html" in (out / "SHA256SUMS").read_text()
    again = tmp_path / "again.html"
    assert _run_report([str(folder), "--html", "-o", str(again)]) == 0
    assert again.read_text() == (out / "report.html").read_text()
    doc = json.loads((out / "metrics.json").read_text())
    page = again.read_text()
    assert html_mismatches(page, doc) == []
    assert "data:image" not in page  # this fixture's metrics write no plot


def test_metric_plots_are_embedded_as_data_uris(tmp_path: Path) -> None:
    """A plot file of the eval dir is inlined byte for byte; a path outside the
    eval dir is never read."""
    import base64

    png = b"\x89PNG\r\n\x1a\n" + bytes(range(64))
    (tmp_path / "plots" / "g").mkdir(parents=True)
    (tmp_path / "plots" / "g" / "a.png").write_bytes(png)
    (tmp_path.parent / "outside.png").write_bytes(png)
    doc = _hostile_doc()
    m = next(iter(next(iter(doc["groups"].values()))["metrics"].values()))
    m["plots"] = ["plots/g/a.png", "../outside.png"]
    page = render_html(doc, tmp_path)
    uris = re.findall(r'src="data:image/png;base64,([A-Za-z0-9+/=]+)"', page)
    assert [base64.b64decode(u) for u in uris] == [png]
    assert "plot ../outside.png not found in the eval dir" in page


# ── review findings ───────────────────────────


@needs_lf_bench
def test_dropped_raw_sums_line_is_refused(tmp_path: Path, capsys) -> None:
    """Without the line pinning ../../SHA256SUMS a changed raw manifest would
    pass unseen; the check names the unpinned input instead."""
    ev = _place(tmp_path)
    sums = ev / "SHA256SUMS"
    sums.write_text("".join(ln for ln in sums.read_text().splitlines(keepends=True)
                            if not ln.rstrip().endswith("../../SHA256SUMS")))
    (ev.parents[1] / "SHA256SUMS").write_text("tampered\n")
    assert _run_report([str(ev), "--html"]) == EXIT_REJECTED
    err = capsys.readouterr().err
    assert "unpinned: no SHA256SUMS line pins the raw SHA256SUMS of input" in err, err


@needs_lf_bench
def test_out_inside_the_results_folder_is_refused(tmp_path: Path, capsys) -> None:
    ev = _place(tmp_path)
    raw = ev.parents[1] / "SHA256SUMS"
    before = raw.read_bytes()
    assert _run_report([str(ev), "--html", "-o", str(raw)]) == 2
    assert raw.read_bytes() == before
    assert "inside results folder" in capsys.readouterr().err


@needs_lf_bench
def test_symlinked_eval_dir_is_refused(tmp_path: Path, capsys) -> None:
    ev = _place(tmp_path)
    link = ev.parent / "29991231T000000Z"
    link.symlink_to(ev, target_is_directory=True)
    assert _run_report([str(link)]) == EXIT_REJECTED
    assert _run_report([str(ev.parents[1])]) == EXIT_REJECTED  # newest is the symlink
    assert "is a symlink; refusing to read through it" in capsys.readouterr().err


@needs_lf_bench
def test_deleted_metrics_json_is_named(tmp_path: Path, capsys) -> None:
    ev = _place(tmp_path)
    (ev / "metrics.json").unlink()
    assert _run_report([str(ev)]) == EXIT_REJECTED
    assert "missing: metrics.json" in capsys.readouterr().err


@needs_lf_bench
def test_render_uses_the_hashed_bytes_not_a_reread(tmp_path: Path) -> None:
    """A plot or metrics.json changed after verification does not reach the page."""
    import base64

    from lf_eval.results_report import load_verified, render_html_many

    ev = _place(tmp_path)
    png = b"\x89PNG\r\n\x1a\n" + bytes(range(64))
    (ev / "plots" / "g").mkdir(parents=True)
    (ev / "plots" / "g" / "a.png").write_bytes(png)
    doc = json.loads((ev / "metrics.json").read_text())
    m = next(iter(next(iter(doc["groups"].values()))["metrics"].values()))
    m["plots"] = ["plots/g/a.png"]
    (ev / "metrics.json").write_text(json.dumps(doc))
    # reseal the eval dir's own files; keep the line pinning ../../SHA256SUMS
    raw = [ln for ln in (ev / "SHA256SUMS").read_text().splitlines() if "  ../" in ln]
    own = [f"{hashlib.sha256((ev / r).read_bytes()).hexdigest()}  {r}"
           for r in ("metrics.json", "plots/g/a.png", "report.md")]
    (ev / "SHA256SUMS").write_text("\n".join(own + raw) + "\n")
    sealed = load_verified(ev)
    page = render_html_many([sealed])
    (ev / "plots" / "g" / "a.png").write_bytes(png[::-1])
    p = ev / "metrics.json"
    p.write_text(p.read_text().replace('"lfc|p0"', '"XXX|p0"', 1))
    again = render_html_many([sealed])
    uris = re.findall(r'src="data:image/png;base64,([A-Za-z0-9+/=]+)"', again)
    assert [base64.b64decode(u) for u in uris] == [png], "page shows bytes that were never hashed"
    assert again == page
    with pytest.raises(Exception, match="changed: metrics.json"):
        load_verified(ev)


# ── review findings, round 2 ─────────────────


def _reseal(ev: Path, doc: dict, raw_lines: list[str]) -> None:
    """Write *doc* as metrics.json and reseal the eval dir's own files plus *raw_lines*."""
    (ev / "metrics.json").write_text(json.dumps(doc))
    own = sorted(p.relative_to(ev).as_posix() for p in ev.rglob("*")
                 if p.is_file() and p.name != "SHA256SUMS")
    lines = [f"{hashlib.sha256((ev / r).read_bytes()).hexdigest()}  {r}" for r in own]
    (ev / "SHA256SUMS").write_text("\n".join(lines + raw_lines) + "\n")


def _twin(ev: Path, name: str) -> tuple[Path, str]:
    """A second results folder *name* beside the first, byte-identical raw manifest."""
    a = ev.parents[1]
    b = a.parent / name
    b.mkdir()
    shutil.copy(a / "SHA256SUMS", b / "SHA256SUMS")
    return b, hashlib.sha256((a / "SHA256SUMS").read_bytes()).hexdigest()


@needs_lf_bench
def test_input_folder_must_be_the_pinned_folder(tmp_path: Path, capsys) -> None:
    """metrics.json naming folder B while only folder A's raw SHA256SUMS is pinned is
    refused, even when both manifests hash the same."""
    ev = _place(tmp_path)
    b, digest = _twin(ev, "20261111T000000Z_other")
    doc = json.loads((ev / "metrics.json").read_text())
    doc["inputs"][0]["folder"] = str(b)
    _reseal(ev, doc, [f"{digest}  ../../SHA256SUMS"])
    rc = _run_report([str(ev), "--html"])
    err = capsys.readouterr().err
    assert rc == EXIT_REJECTED, f"eval names folder {b.name} but pins only {ev.parents[1].name}; " \
        f"report rendered (exit {rc})"
    assert f"unpinned: no SHA256SUMS line pins the raw SHA256SUMS of input {b}" in err, err


@needs_lf_bench
def test_out_is_guarded_against_every_pinned_folder(tmp_path: Path, capsys) -> None:
    """Two pinned results folders with byte-identical manifests: --out into either
    is refused."""
    ev = _place(tmp_path)
    a = ev.parents[1]
    b, digest = _twin(ev, "20261111T000000Z_other")
    doc = json.loads((ev / "metrics.json").read_text())
    doc["inputs"] = [doc["inputs"][0], dict(doc["inputs"][0], folder=str(b))]
    _reseal(ev, doc, [f"{digest}  ../../SHA256SUMS", f"{digest}  ../../../{b.name}/SHA256SUMS"])
    for folder in (a, b):
        out = folder / "x.html"
        rc = _run_report([str(ev), "--html", "-o", str(out)])
        assert rc == 2 and not out.exists(), \
            f"--out {out} inside pinned results folder {folder.name} was written (exit {rc})"
        assert "inside results folder" in capsys.readouterr().err


@needs_lf_bench
def test_symlinked_ancestor_is_refused(tmp_path: Path, capsys) -> None:
    """A path whose ancestor is a symlink (eval-link -> the real eval/ dir) is refused,
    not only a symlinked leaf."""
    ev = _place(tmp_path)
    link = tmp_path / "eval-link"
    link.symlink_to(ev.parent, target_is_directory=True)
    rc = _run_report([str(link / ev.name)])
    err = capsys.readouterr().err
    assert rc == EXIT_REJECTED, f"read through symlinked ancestor {link} (exit {rc})"
    assert f"{link} is a symlink; refusing to read through it" in err, err


@needs_lf_bench
def test_listed_plot_outside_the_eval_dir_is_not_embedded(tmp_path: Path) -> None:
    """plots/../../outside.png is listed in SHA256SUMS and named as a plot; it lies
    outside the eval dir, so it is hashed but never embedded."""
    import base64

    from lf_eval.results_report import load_verified, render_html_many

    ev = _place(tmp_path)
    png = b"\x89PNG\r\n\x1a\n" + bytes(range(64))
    (ev / "plots").mkdir()
    (ev.parent / "outside.png").write_bytes(png)
    rel = "plots/../../outside.png"
    doc = json.loads((ev / "metrics.json").read_text())
    m = next(iter(next(iter(doc["groups"].values()))["metrics"].values()))
    m["plots"] = [rel]
    raw = [ln for ln in (ev / "SHA256SUMS").read_text().splitlines() if "  ../" in ln]
    _reseal(ev, doc, raw + [f"{hashlib.sha256(png).hexdigest()}  {rel}"])
    sealed = load_verified(ev)
    assert rel not in sealed.blobs, f"{rel} lies outside the eval dir but was kept as a blob"
    page = render_html_many([sealed])
    uris = re.findall(r'src="data:image/png;base64,([A-Za-z0-9+/=]+)"', page)
    assert [base64.b64decode(u) for u in uris] == [], \
        f"page embeds {rel}, a file outside the eval dir"
    assert f"plot {rel} not found in the eval dir" in page


# ── a looping symlink must not crash the report ─────────────────

def test_looping_symlink_plot_is_a_placeholder(tmp_path: Path) -> None:
    """A plot path that is a symlink loop (loop.png -> loop.png) cannot be resolved;
    it is treated as outside the eval dir, not raised as a traceback."""
    (tmp_path / "plots").mkdir()
    (tmp_path / "plots" / "loop.png").symlink_to("loop.png")
    doc = _hostile_doc()
    m = next(iter(next(iter(doc["groups"].values()))["metrics"].values()))
    m["plots"] = ["plots/loop.png"]
    page = render_html(doc, tmp_path)
    assert "plot plots/loop.png not found in the eval dir" in page


@needs_lf_bench
def test_listed_looping_symlink_is_refused_by_name(tmp_path: Path, capsys) -> None:
    """A SHA256SUMS line naming a symlink loop is refused like any other symlink,
    with exit EXIT_REJECTED and the path named, not a traceback."""
    ev = _place(tmp_path)
    (ev / "plots").mkdir()
    (ev / "plots" / "loop.png").symlink_to("loop.png")
    rel = "plots/loop.png"
    doc = json.loads((ev / "metrics.json").read_text())
    m = next(iter(next(iter(doc["groups"].values()))["metrics"].values()))
    m["plots"] = [rel]
    raw = [ln for ln in (ev / "SHA256SUMS").read_text().splitlines() if "  ../" in ln]
    _reseal(ev, doc, raw + [f"{'0' * 64}  {rel}"])
    assert _run_report([str(ev), "--html"]) == EXIT_REJECTED
    err = capsys.readouterr().err
    assert f"symlink: {rel}" in err and "nothing rendered" in err, err
