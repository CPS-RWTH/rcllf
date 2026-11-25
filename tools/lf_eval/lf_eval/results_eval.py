# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``ros2 lf eval``: the metric registry over ``rcllf.result/1`` results folders.

The steps, in order:

step 0  verify. Every results folder is checked against its ``SHA256SUMS``
        before anything else is read: a listed file that is missing, a file
        whose hash changed, a regular file the manifest does not list, or a
        symlink outside ``runs/*/logs/`` rejects the folder, and the message
        names every offending path. A rejected folder is never partially
        evaluated, and one rejected folder stops the whole eval.
        ``--allow-missing GLOB`` waives *missing* files matching the glob only
        (for fixtures whose traces were trimmed after recording); every waived
        path is recorded in ``metrics.json``. Changed and unlisted files are
        never waivable, and neither is a missing ``result.json``. ``eval/`` is
        outside the raw scope only when it is a real directory: an ``eval``
        symlink (or file) rejects the folder, since the eval dir is written there.
        Every later read is bound to the bytes step 0 hashed: ``result.json``,
        ``run.json``, ``system.json`` and the ``*.jsonl`` inputs are read once,
        their sha256 checked against ``SHA256SUMS``, and parsed from those
        bytes. Trace runs are read by path (CTF via babeltrace), so every listed
        file of the run dir is re-hashed right before and right after the trace
        load; the remaining window is a file swapped and swapped back between
        those two checks.
step 1  load. ``result.json`` lists the runs; they are grouped by
        (variant, sweep point). Excluded runs are listed but not passed to a
        metric unless ``--include-excluded``.
step 2  compute per group: ``Metric.compute(runs, system, chains)`` with the
        group's own ``build/<variant>-<point>/system.json``. A metric whose
        declared ``inputs`` a run or group lacks is ``unavailable`` (or
        ``not_applicable`` for ``lf_model`` on a stock group). Options reach
        only metrics that declare them (``options_schema``); an option for a
        metric without the declaration is an eval error. ``cross_group``
        metrics are called once over all groups, without a graph model: a
        ``cross_group`` metric that declares ``graph_model`` or ``lf_model`` is
        ``not_applicable``. Group keys (``[folder:]variant|point``) and their
        plot dir names must be distinct; a collision is an eval error.
step 3  compare. With ``--baseline VARIANT`` every other variant's group is
        compared with the baseline group of the same sweep point, through each
        metric's ``compare()``; ``delta = candidate - baseline``. A comparison
        of two variant kinds (rcllf vs stock) is refused unless both groups
        declare a horizon.

Output ``<first results folder>/eval/<UTC>/`` (or ``--out``): ``metrics.json``
(``rcllf.metrics/1``), ``report.md``, ``report.html`` (static, see
:mod:`lf_eval.results_report`), ``plots/<group>/`` and a ``SHA256SUMS``
over every file of the eval dir except itself, plus one line per input folder
with the hash of the raw ``SHA256SUMS`` that step 0 verified (path relative to
the eval dir, so ``sha256sum -c SHA256SUMS`` inside the eval dir checks it).
"""

from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import logging
import os
import stat
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, Optional, Sequence

from lf_eval import __version__

logger = logging.getLogger(__name__)

METRICS_SCHEMA = "rcllf.metrics/1"
SUMS = "SHA256SUMS"
RESULT_JSON = "result.json"
#: Symlinks are tolerated only here (ROS's ``logs/ros/latest``); no metric
#: reads under ``logs/``. A symlink anywhere else could point a reader at
#: bytes step 0 never hashed, so it rejects the folder.
SYMLINK_OK = "runs/*/logs/*"

EXIT_USAGE = 2
EXIT_REJECTED = 4


class EvalError(Exception):
    """An eval that must stop, with its CLI exit code."""

    def __init__(self, message: str, code: int = EXIT_USAGE) -> None:
        super().__init__(message)
        self.code = code


# ─────────────────────────────────────────────────────────── step 0: verify


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


@dataclass
class Verified:
    """Step 0 outcome of one folder."""

    root: Path
    problems: list[str]
    sums_sha256: Optional[str] = None
    waived_missing: list[str] = field(default_factory=list)
    listed: dict[str, str] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.problems

    def present(self, rel: str) -> bool:
        """``rel`` is listed and was not waived as missing (step 0's view)."""
        return rel in self.listed and rel not in self.waived_missing

    def present_under(self, rel: str) -> bool:
        pre = rel.rstrip("/") + "/"
        return any(n.startswith(pre) and self.present(n) for n in self.listed)

    def _changed(self, rel: str, why: str) -> EvalError:
        return EvalError(f"{self.root}: {rel} changed after step 0 verification ({why}); "
                         "nothing written", EXIT_REJECTED)

    def read_bytes(self, rel: str) -> bytes:
        """The bytes of ``rel``, only if they hash to what step 0 verified.

        Read once and parsed from the returned bytes by the caller, so a file
        replaced after step 0 can never be attributed to the verified hash.
        """
        if not self.present(rel):
            raise EvalError(f"{self.root}: {rel} was not verified by step 0 "
                            "(unlisted or waived as missing)", EXIT_REJECTED)
        try:
            data = (self.root / rel).read_bytes()
        except OSError as exc:
            raise self._changed(rel, f"unreadable: {exc.strerror}") from None
        if hashlib.sha256(data).hexdigest() != self.listed[rel]:
            raise self._changed(rel, "sha256 differs from SHA256SUMS")
        return data

    def read_json(self, rel: str) -> Any:
        return json.loads(self.read_bytes(rel))

    def recheck_tree(self, rel: str) -> None:
        """Re-run step 0 on everything under ``rel/`` (for readers that take a path)."""
        pre = rel.rstrip("/") + "/"
        for name, digest in sorted(self.listed.items()):
            if not name.startswith(pre) or not self.present(name):
                continue
            p = self.root / name
            if p.is_symlink() or not p.is_file():
                raise self._changed(name, "no longer a regular file")
            if sha256_file(p) != digest:
                raise self._changed(name, "sha256 differs from SHA256SUMS")
        files, links, special = _walk_raw(self.root / rel)
        # a file waived as missing that is back now was never hashed either
        extra = sorted(f"{pre}{n}" for n in files if not self.present(f"{pre}{n}"))
        extra += sorted(f"{pre}{n}" for n in links | special
                        if not fnmatch.fnmatch(f"{pre}{n}", SYMLINK_OK))
        if extra:
            raise self._changed(extra[0], "appeared after step 0; "
                                f"{len(extra)} unverified path(s)")


def _walk_raw(root: Path, skip_eval: bool = False) -> tuple[set[str], set[str], set[str]]:
    """(regular files, symlinks, other non-regular) under the raw scope.

    The raw scope is everything but ``eval/`` and ``SHA256SUMS`` itself.
    ``os.walk`` does not descend into a symlinked directory; it is reported
    as a symlink like a file symlink. Only a *real* ``eval/`` directory is
    left out (``skip_eval``): an ``eval`` symlink is reported like any other
    symlink, because the eval dir is written through it.
    """
    files, links, special = set(), set(), set()
    for dirpath, dirnames, filenames in os.walk(root):
        rel = Path(dirpath).relative_to(root)
        if rel == Path(".") and skip_eval:
            dirnames[:] = [d for d in dirnames
                           if d != "eval" or os.path.islink(Path(dirpath) / d)]
        for name in list(dirnames) + filenames:
            p = (rel / name).as_posix()
            try:
                mode = os.lstat(Path(dirpath) / name).st_mode
            except OSError:
                continue
            if stat.S_ISLNK(mode):
                links.add(p)
            elif stat.S_ISDIR(mode):
                continue
            elif stat.S_ISREG(mode):
                if p != SUMS or not skip_eval:
                    files.add(p)
            else:
                special.add(p)  # FIFO, device, socket: never part of a results folder
    return files, links, special


def verify_folder(root: Path, allow_missing: Sequence[str] = ()) -> Verified:
    """Step 0: check every raw file of ``root`` against its ``SHA256SUMS``."""
    root = Path(root)
    sums = root / SUMS
    if not sums.is_file() or sums.is_symlink():
        return Verified(root, [f"missing: {SUMS}"])
    listed: dict[str, str] = {}
    problems: list[str] = []
    for n, line in enumerate(sums.read_text().splitlines(), 1):
        digest, sep, name = line.partition("  ")
        bad = (not sep or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest)
               or not name or name.startswith("/") or ".." in Path(name).parts
               or name in listed)
        if bad:
            problems.append(f"malformed: {SUMS} line {n}: {line[:120]!r}")
            continue
        listed[name] = digest
    files, links, special = _walk_raw(root, skip_eval=True)
    waived = []
    if RESULT_JSON not in listed and RESULT_JSON not in files:
        problems.append(f"missing: {RESULT_JSON} (required; never waivable)")
    for name, digest in sorted(listed.items()):
        if name not in files:
            if name in links or name in special:
                problems.append(f"not a regular file: {name}")
            elif name == RESULT_JSON:
                problems.append(f"missing: {RESULT_JSON} (required; never waivable)")
            elif any(fnmatch.fnmatch(name, g) for g in allow_missing):
                waived.append(name)
            else:
                problems.append(f"missing: {name}")
        elif sha256_file(root / name) != digest:
            problems.append(f"changed: {name}")
    problems += [f"unlisted: {n}" for n in sorted(files - set(listed))]
    problems += [f"symlink: {n}" for n in sorted(links - set(listed))
                 if not fnmatch.fnmatch(n, SYMLINK_OK)]
    problems += [f"not a regular file: {n}" for n in sorted(special - set(listed))]
    return Verified(root, problems, sha256_file(sums), waived, listed)


# ────────────────────────────────────────────────────────────── step 1: load


@dataclass
class ResultRun:
    """One run of a results folder, as the results-folder metrics see it."""

    name: str
    path: Path
    variant: str
    point: str
    kind: str
    included: bool
    run_json: dict[str, Any]
    reasons: list[str] = field(default_factory=list)
    #: step 0 outcome of the folder and this run dir relative to it; every
    #: read goes through ``verified`` so it sees only the hashed bytes
    verified: Optional[Verified] = None
    rel: str = ""

    def _jsonl(self, filename: str) -> Optional[list[dict[str, Any]]]:
        rel = f"{self.rel}/{filename}"
        if not self.verified.present(rel):
            return None
        text = self.verified.read_bytes(rel).decode()
        return [json.loads(x) for x in text.splitlines() if x.strip()]

    @property
    def outputs(self) -> Optional[list[dict[str, Any]]]:
        return self._jsonl("outputs.jsonl")

    @property
    def sources(self) -> Optional[list[dict[str, Any]]]:
        return self._jsonl("sources.jsonl")

    @property
    def arrivals(self) -> Optional[list[dict[str, Any]]]:
        return self._jsonl("arrivals.jsonl")

    def has(self, kind: str) -> bool:
        """Whether step 0 verified the input (listed and not waived as missing)."""
        v, r = self.verified, self.rel
        return {
            "trace": lambda: v.present_under(f"{r}/tracing"),
            "outputs": lambda: v.present(f"{r}/outputs.jsonl"),
            "arrivals": lambda: v.present(f"{r}/arrivals.jsonl"),
            "cpu": lambda: v.present(f"{r}/cpu_proc.json"),
            "bag": lambda: v.present_under(f"{r}/bag"),
        }[kind]()


@dataclass
class Group:
    key: str
    folder: Path
    variant: str
    point: str
    kind: str
    build_ref: Optional[str]
    horizon: Optional[dict[str, Any]]
    runs: list[ResultRun]
    verified: Optional[Verified] = None
    #: injective identity (folder prefix, variant, point); ``key`` is its
    #: readable rendering and is checked for collisions before step 2
    ident: tuple[str, str, str] = ("", "", "")

    @property
    def build_dir(self) -> Optional[Path]:
        return self.folder / self.build_ref if self.build_ref else None

    def build_file(self, name: str) -> Optional[str]:
        """``<build_ref>/<name>`` relative to the folder, as SHA256SUMS lists it."""
        return PurePosixPath(self.build_ref, name).as_posix() if self.build_ref else None

    def included(self, include_excluded: bool) -> list[ResultRun]:
        return [r for r in self.runs if r.included or include_excluded]


def _horizon_ns(horizon: Optional[dict[str, Any]]) -> Optional[int]:
    if not horizon:
        return None
    if horizon.get("ns") is not None:
        return int(horizon["ns"])
    if horizon.get("wall_s") is not None:
        return int(round(float(horizon["wall_s"]) * 1e9))
    return None


def _inside(folder: Path, sub: str, ref: str, what: str) -> Path:
    """``folder/<ref>`` when it stays under ``folder/<sub>``, else an EvalError.

    result.json and run.json name the run and build dirs a metric reads; a
    ``../`` or absolute reference would point a metric at bytes step 0 never
    verified.
    """
    path = folder / ref
    if Path(ref).is_absolute() or ".." in Path(ref).parts:
        raise EvalError(f"{folder}: {what} {ref!r} leaves the results folder", EXIT_REJECTED)
    try:
        path.resolve().relative_to((folder / sub).resolve())
    except ValueError:
        raise EvalError(f"{folder}: {what} {ref!r} is not under {sub}/", EXIT_REJECTED) from None
    return path


def load_groups(verified: Verified, prefix: str = "") -> list[Group]:
    """Step 1 for one verified folder; every file is read through ``verified``."""
    folder = verified.root
    doc = verified.read_json(RESULT_JSON)
    runs_by_group: dict[tuple[str, str], list[ResultRun]] = {}
    for entry in doc.get("runs", []):
        rid = str(entry["run_id"])
        if Path(rid).is_absolute() or ".." in Path(rid).parts:
            raise EvalError(f"{folder}: run_id {rid!r} leaves the results folder", EXIT_REJECTED)
        if Path(rid).name != rid or rid in ("", "."):
            raise EvalError(f"{folder}: run_id {rid!r} is not one directory name", EXIT_REJECTED)
        run_dir = _inside(folder, "runs", f"runs/{rid}", "run_id")
        rel = f"runs/{rid}"
        rj = verified.read_json(f"{rel}/run.json") if verified.present(f"{rel}/run.json") else {}
        run = ResultRun(
            name=entry["run_id"], path=run_dir, variant=entry["variant"],
            point=entry["point"], kind=rj.get("kind", "unknown"),
            included=bool(entry.get("included")), run_json=rj,
            reasons=list(entry.get("reasons", [])), verified=verified, rel=rel)
        runs_by_group.setdefault((run.variant, run.point), []).append(run)
    groups = []
    for (variant, point), runs in sorted(runs_by_group.items()):
        kinds = {r.kind for r in runs}
        refs = {r.run_json.get("build_ref") for r in runs}
        horizons = {json.dumps(r.run_json.get("horizon"), sort_keys=True) for r in runs}
        key = f"{prefix}{variant}|{point}"
        for what, vals in (("kind", kinds), ("build_ref", refs), ("horizon", horizons)):
            if len(vals) != 1:
                raise EvalError(f"{folder}: group {key}: runs disagree on {what}: {sorted(map(str, vals))}")
        build_ref = refs.pop()
        if build_ref:
            _inside(folder, "build", build_ref, f"group {key}: build_ref")
        if build_ref:
            build_ref = PurePosixPath(build_ref).as_posix()
        groups.append(Group(key=key, folder=folder, variant=variant, point=point,
                            kind=kinds.pop(), build_ref=build_ref,
                            horizon=json.loads(horizons.pop()), runs=runs,
                            verified=verified, ident=(prefix, variant, point)))
    return groups


# ─────────────────────────────────────────────────────────── step 2: compute


def parse_options(specs: Sequence[str], metrics: Sequence[Any]) -> dict[str, dict[str, Any]]:
    """``--option metric.key=value`` against each metric's ``options_schema``."""
    by_name = {m.name: m for m in metrics}
    out: dict[str, dict[str, Any]] = {}
    for spec in specs:
        head, eq, raw = spec.partition("=")
        mname, dot, key = head.partition(".")
        if not eq or not dot or not mname or not key:
            raise EvalError(f"--option {spec!r}: expected METRIC.KEY=VALUE")
        if mname not in by_name:
            raise EvalError(f"--option {spec!r}: metric {mname!r} is not selected")
        schema = by_name[mname].options_schema
        if schema is None:
            raise EvalError(f"--option {spec!r}: metric {mname!r} declares no options; "
                            f"passing it options is an eval error")
        if key not in schema:
            raise EvalError(f"--option {spec!r}: metric {mname!r} declares only "
                            f"{sorted(schema)}")
        typ = schema[key]
        try:
            if typ is bool:
                if raw.lower() not in ("true", "false"):
                    raise ValueError(raw)
                val: Any = raw.lower() == "true"
            else:
                val = typ(raw)
        except ValueError:
            raise EvalError(f"--option {spec!r}: {raw!r} is not a {typ.__name__}") from None
        out.setdefault(mname, {})[key] = val
    return out


def _missing_inputs(metric: Any, group: Group, runs: Sequence[ResultRun]) -> tuple[str, str]:
    """``(status, reason)``; status ``""`` when every declared input is there."""
    for kind in metric.inputs:
        if kind == "lf_model":
            if group.kind == "stock":
                return "not_applicable", "declares lf_model; a stock group has no LF model"
            rel = group.build_file("resolved_model.json")
            if not (rel and group.verified.present(rel)):
                return "unavailable", f"no resolved_model.json in {group.build_ref}"
        elif kind == "graph_model":
            rel = group.build_file("system.json")
            if not (rel and group.verified.present(rel)):
                return "unavailable", f"no system.json in {group.build_ref}"
        else:
            lacking = [r.name for r in runs if not r.has(kind)]
            if lacking:
                return "unavailable", f"input {kind!r} not captured in {', '.join(lacking)}"
    return "", ""


def _call(metric: Any, runs: Sequence[Any], system: Any, chains: Sequence[Any],
          options: dict[str, Any]) -> Any:
    if metric.options_schema is None:
        if options:  # defensive: parse_options already refuses this
            raise EvalError(f"metric {metric.name!r} declares no options, got {sorted(options)}")
        return metric.compute(runs, system, chains)
    return metric.compute(runs, system, chains, **options)


def _trace_runs(runs: Sequence[ResultRun], system: Any, backend: Optional[str]) -> list[Any]:
    from lf_eval.model import load_run

    loaded = []
    for rr in runs:
        # load_run reads the run dir by path (babeltrace for CTF): re-run step 0
        # on the run dir right before and right after, so a swap is caught
        # unless it is undone between the two checks (documented window).
        rr.verified.recheck_tree(rr.rel)
        run = load_run(rr.path, system=system, backend=backend)
        rr.verified.recheck_tree(rr.rel)
        # plan 3c: runs carry .outputs, .sources and .arrivals
        run.outputs, run.sources, run.arrivals = rr.outputs, rr.sources, rr.arrivals
        loaded.append(run)
    return loaded


def compute_group(group: Group, metrics: Sequence[Any], options: dict[str, dict[str, Any]],
                  *, include_excluded: bool, chain_specs: Sequence[str],
                  backend: Optional[str]) -> tuple[dict[str, Any], dict[str, Any]]:
    """Step 2 for one group: ``({metric: entry}, {metric: MetricResult})``."""
    from lf_eval.model import resolve_chains
    from lf_eval.model.system import parse_system

    runs = group.included(include_excluded)
    system = None
    sys_rel = group.build_file("system.json")
    if sys_rel is not None and group.verified.present(sys_rel):
        system = parse_system(group.verified.read_json(sys_rel),
                              source_path=group.folder / sys_rel)
    chains = resolve_chains(system, list(chain_specs) or None) if system is not None else []
    entries: dict[str, Any] = {}
    results: dict[str, Any] = {}
    trace_cache: Optional[list[Any]] = None
    for metric in metrics:
        if metric.scope != "per_group":
            continue
        if not runs:
            entries[metric.name] = {"status": "unavailable", "available": False,
                                    "note": "no included runs in this group"}
            continue
        status, reason = _missing_inputs(metric, group, runs)
        if status:
            entries[metric.name] = {"status": status, "available": False, "note": reason}
            continue
        opts = dict(options.get(metric.name, {}))
        if metric.options_schema and "horizon_ns" in metric.options_schema:
            opts.setdefault("horizon_ns", _horizon_ns(group.horizon))
            if opts["horizon_ns"] is None:
                del opts["horizon_ns"]
        try:
            if "trace" in metric.inputs:
                if trace_cache is None:
                    trace_cache = _trace_runs(runs, system, backend)
                arg_runs: Sequence[Any] = trace_cache
            else:
                arg_runs = runs
            result = _call(metric, arg_runs, system, chains, opts)
        except EvalError:
            raise
        except Exception as exc:  # one bad metric must not kill the eval
            logger.exception("metric %s failed on %s", metric.name, group.key)
            entries[metric.name] = {"status": "error", "available": False,
                                    "note": f"error: {type(exc).__name__}: {exc}"}
            continue
        doc = result.to_dict()
        doc["status"] = "ok" if result.available else "unavailable"
        if opts:
            doc["options"] = opts
        entries[metric.name] = doc
        results[metric.name] = result
    return entries, results


# ─────────────────────────────────────────────────────────── step 3: compare


def compare_groups(groups: Sequence[Group], results: dict[str, dict[str, Any]],
                   metrics: Sequence[Any], baseline: str) -> list[dict[str, Any]]:
    from lf_eval.metrics import has_compare

    bases = [g for g in groups if g.variant == baseline]
    if not bases:
        known = sorted({g.variant for g in groups})
        raise EvalError(f"--baseline {baseline!r}: no such variant; variants: {known}")

    def base_for(g: Group) -> tuple[Optional[Group], Optional[str]]:
        # The candidate's own folder first; another folder only when exactly one
        # folder has the baseline at this point (otherwise the pairing is a guess).
        same = [b for b in bases if b.point == g.point and b.folder == g.folder]
        if same:
            return same[0], None
        other = [b for b in bases if b.point == g.point]
        if len(other) == 1:
            return other[0], None
        if other:
            return None, (f"{baseline} at point {g.point} is in {len(other)} other folders "
                          f"({', '.join(b.key for b in other)}) and none in the candidate's; "
                          "ambiguous baseline")
        return None, f"no {baseline} group at point {g.point}"

    out = []
    for g in groups:
        if g.variant == baseline:
            continue
        b, why = base_for(g)
        for metric in metrics:
            if not has_compare(metric) or metric.scope != "per_group":
                continue
            row: dict[str, Any] = {
                "metric": metric.name, "baseline": b.key if b else None, "candidate": g.key,
                "point": g.point, "convention": "delta = candidate - baseline",
                "horizon": {"baseline": b.horizon if b else None, "candidate": g.horizon},
            }
            if b is None:
                row.update(status="refused", reason=why)
            elif b.kind != g.kind and (not _horizon_ns(b.horizon) or not _horizon_ns(g.horizon)):
                row.update(status="refused",
                           reason=f"mixed-kind comparison ({b.kind} vs {g.kind}) needs a "
                                  "declared horizon on both sides")
            else:
                rb, rc = results[b.key].get(metric.name), results[g.key].get(metric.name)
                if rb is None or rc is None or not rb.available or not rc.available:
                    row.update(status="unavailable",
                               reason="metric unavailable on "
                                      + ("baseline" if rb is None or not rb.available else "candidate"))
                else:
                    row.update(status="ok", rows=metric.compare(rb, rc))
            out.append(row)
    return out


# ─────────────────────────────────────────────────────────────────── output


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.6g}"
    return "" if v is None else str(v)


def _table(rows: Sequence[dict[str, Any]]) -> list[str]:
    if not rows:
        return []
    cols = list(dict.fromkeys(k for r in rows for k in r))
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    lines += ["| " + " | ".join(_fmt(r.get(c)).replace("|", "\\|") for c in cols) + " |"
              for r in rows]
    return lines


def render_report(doc: dict[str, Any]) -> str:
    lines = ["# ros2 lf eval", "",
             f"lf_eval {doc['lf_eval_version']}, {doc['generated_at']}", "", "## Step 0", ""]
    for inp in doc["inputs"]:
        waived = len(inp["waived_missing"])
        lines.append(f"- `{inp['folder']}`: verified, raw SHA256SUMS sha256 "
                     f"`{inp['sha256sums_sha256']}`"
                     + (f", {waived} missing files waived by --allow-missing "
                        f"{inp['allow_missing']}" if waived else ""))
    for key, g in doc["groups"].items():
        lines += ["", f"## {key}", "",
                  f"kind {g['kind']}, build `{g['build_ref']}`, runs "
                  f"{len(g['runs_used'])} used, {len(g['runs_excluded'])} excluded"]
        for name, m in g["metrics"].items():
            lines += ["", f"### {name}: {m['status']}"]
            if m.get("note"):
                lines.append(f"\n{m['note']}")
            if m.get("summary"):
                lines += [""] + _table(m["summary"])
    if doc.get("cross_group"):
        lines += ["", "## cross-group metrics"]
        for name, m in doc["cross_group"].items():
            lines += ["", f"### {name}: {m['status']}"] + ([""] + _table(m.get("summary") or []))
    if doc.get("compare") is not None:
        lines += ["", f"## Comparison against baseline `{doc['baseline']}`",
                  "", "delta = candidate - baseline"]
        for c in doc["compare"]:
            lines += ["", f"### {c['metric']}: {c['candidate']} vs {c['baseline']} ({c['status']})"]
            if c["status"] == "ok":
                lines += [""] + _table(c["rows"])
            else:
                lines.append(f"\n{c['reason']}")
    return "\n".join(lines) + "\n"


def _write_eval_sums(outdir: Path, verified: Sequence[Verified]) -> Path:
    files = sorted(p for p in outdir.rglob("*") if p.is_file() and p != outdir / SUMS)
    lines = [f"{sha256_file(p)}  {p.relative_to(outdir).as_posix()}\n" for p in files]
    for v in verified:
        rel = os.path.relpath(v.root / SUMS, outdir)
        lines.append(f"{v.sums_sha256}  {Path(rel).as_posix()}\n")
    out = outdir / SUMS
    out.write_text("".join(lines))
    return out


def _slug(key: str) -> str:
    return "".join(c if c.isalnum() or c in "-_." else "_" for c in key)


def _refuse_key_collisions(groups: Sequence[Group]) -> None:
    """Group keys and plot dir names must identify one (folder, variant, point).

    ``f"{variant}|{point}"`` is not injective (``("a|b", "p0")`` and
    ``("a", "b|p0")`` both read ``a|b|p0``), and neither is ``_slug``; a
    collision would silently merge two groups in ``metrics.json`` or ``plots/``.
    """
    for what, render in (("group key", lambda g: g.key), ("plot dir", lambda g: _slug(g.key))):
        seen: dict[str, Group] = {}
        for g in groups:
            k = render(g)
            if k in seen and seen[k].ident != g.ident:
                a, b = seen[k].ident, g.ident
                raise EvalError(f"{what} {k!r} would collide: (folder, variant, point) "
                                f"{list(a)} and {list(b)}; rename a variant or sweep point")
            seen[k] = g


def evaluate(folders: Sequence[Path], *, metric_names: Optional[Sequence[str]] = None,
             baseline: Optional[str] = None, option_specs: Sequence[str] = (),
             include_excluded: bool = False, allow_missing: Sequence[str] = (),
             chain_specs: Sequence[str] = (), backend: Optional[str] = None,
             out: Optional[Path] = None, no_plots: bool = False,
             echo: Any = print) -> Path:
    """Run steps 0 to 3 and write the eval dir; returns its path."""
    from lf_eval.metrics import resolve_metrics

    folders = [Path(f) for f in folders]
    if not folders:
        raise EvalError("no results folder given")
    for f in folders:
        # A folder with SHA256SUMS is judged by step 0 (a deleted result.json is
        # a rejection, exit 4); one with neither file is not a results folder.
        if not (f / RESULT_JSON).is_file() and not (f / SUMS).is_file():
            raise EvalError(f"{f}: not a results folder (no {RESULT_JSON}); "
                            "single run dirs go to `lf-eval analyze`")
    # 2. distinct group-key prefixes
    if len(folders) > 1:
        names = [f.name for f in folders]
        dup = sorted({n for n in names if names.count(n) > 1})
        if dup:
            raise EvalError(f"results folders share a name {dup}; group keys are prefixed "
                            "with the folder name, so each folder needs a distinct one")
    # 4. an --out inside a folder's raw scope would break that folder's next step 0
    if out is not None:
        o = Path(out).resolve()
        for f in folders:
            fr = f.resolve()
            if (o == fr or fr in o.parents) and not (fr / "eval") in o.parents:
                raise EvalError(f"--out {out} lies inside {f} outside its eval/; "
                                "use a path under eval/ or outside the folder")
    try:
        metrics = resolve_metrics(metric_names)
    except KeyError as exc:
        raise EvalError(str(exc.args[0])) from None
    options = parse_options(option_specs, metrics)

    # step 0: every folder, before anything is read
    verified = [verify_folder(f, allow_missing) for f in folders]
    rejected = [v for v in verified if not v.ok]
    if rejected:
        msg = []
        for v in rejected:
            msg.append(f"{v.root}: rejected by step 0 verification ({len(v.problems)} "
                       f"problem{'s' if len(v.problems) != 1 else ''}), nothing evaluated:")
            msg += [f"  {p}" for p in v.problems]
        raise EvalError("\n".join(msg), EXIT_REJECTED)
    for v in verified:
        echo(f"step 0: {v.root}: verified against {SUMS}"
             + (f" ({len(v.waived_missing)} missing files waived by --allow-missing)"
                if v.waived_missing else ""))

    # step 1
    multi = len(folders) > 1
    groups: list[Group] = []
    for v in verified:
        groups += load_groups(v, prefix=f"{v.root.name}:" if multi else "")
    _refuse_key_collisions(groups)

    # step 2
    group_docs: dict[str, Any] = {}
    group_results: dict[str, dict[str, Any]] = {}
    for g in groups:
        echo(f"group {g.key}: {len(g.included(include_excluded))} runs")
        entries, results = compute_group(g, metrics, options, include_excluded=include_excluded,
                                         chain_specs=chain_specs, backend=backend)
        group_results[g.key] = results
        group_docs[g.key] = {
            "folder": str(g.folder), "variant": g.variant, "point": g.point, "kind": g.kind,
            "build_ref": g.build_ref, "horizon": g.horizon,
            "runs_used": [r.name for r in g.included(include_excluded)],
            "runs_excluded": [{"run": r.name, "reasons": r.reasons}
                              for r in g.runs if not r.included],
            "metrics": entries,
        }
    cross: dict[str, Any] = {}
    for metric in metrics:
        if metric.scope != "cross_group":
            continue
        models = [k for k in metric.inputs if k in ("graph_model", "lf_model")]
        if models:
            # One call over every group has no single graph/LF model to pass.
            cross[metric.name] = {
                "status": "not_applicable", "available": False,
                "note": f"declares {', '.join(models)}; a cross-group metric is called once "
                        "over every group and gets no graph or LF model (system=None)"}
            continue
        runs = [r for g in groups for r in g.included(include_excluded)]
        lacking = [k for k in metric.inputs if k in ("trace", "outputs", "arrivals", "cpu", "bag")
                   and any(not r.has(k) for r in runs)]
        if lacking or not runs:
            cross[metric.name] = {"status": "unavailable", "available": False,
                                  "note": f"input {lacking} not captured in every run"
                                  if lacking else "no included runs"}
            continue
        try:
            arg_runs = _trace_runs(runs, None, backend) if "trace" in metric.inputs else runs
            result = _call(metric, arg_runs, None, (), dict(options.get(metric.name, {})))
        except EvalError:
            raise
        except Exception as exc:  # one bad metric must not kill the eval
            logger.exception("cross-group metric %s failed", metric.name)
            cross[metric.name] = {"status": "error", "available": False,
                                  "note": f"error: {type(exc).__name__}: {exc}"}
            continue
        cross[metric.name] = dict(result.to_dict(), status="ok" if result.available else "unavailable")

    # step 3
    compare = compare_groups(groups, group_results, metrics, baseline) if baseline else None

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    outdir = Path(out) if out else folders[0] / "eval" / stamp
    n = 1
    while outdir.exists():
        if out:
            raise EvalError(f"--out {outdir} exists; an eval dir is written once")
        outdir = folders[0] / "eval" / f"{stamp}-{n}"
        n += 1
    if not out:
        ev = folders[0] / "eval"
        if ev.is_symlink() or (ev.exists() and not ev.is_dir()):  # step 0 already rejects it
            raise EvalError(f"{ev} is not a real directory; refusing to write through it",
                            EXIT_REJECTED)
    outdir.mkdir(parents=True)

    if not no_plots:
        for g in groups:
            for mname, result in group_results[g.key].items():
                if not result.available:
                    continue
                metric = next(m for m in metrics if m.name == mname)
                try:
                    written = metric.plot(result, outdir / "plots" / _slug(g.key))
                except Exception as exc:  # plots never fail the eval
                    logger.warning("plot %s/%s failed: %s", g.key, mname, exc)
                    continue
                group_docs[g.key]["metrics"][mname]["plots"] = [
                    Path(p).relative_to(outdir).as_posix() for p in written]

    doc = {
        "schema": METRICS_SCHEMA,
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "lf_eval_version": __version__,
        "inputs": [{"folder": str(v.root), "sha256sums_sha256": v.sums_sha256,
                    "allow_missing": list(allow_missing), "waived_missing": v.waived_missing}
                   for v in verified],
        "metrics_selected": [m.name for m in metrics],
        "options": options,
        "include_excluded": include_excluded,
        "groups": group_docs,
        "cross_group": cross,
        "baseline": baseline,
        "compare": compare,
    }
    text = json.dumps(doc, indent=2, sort_keys=True, default=str) + "\n"
    (outdir / "metrics.json").write_text(text)
    # Both reports are rendered from the written bytes, so `ros2 lf report
    # [--html]` over this eval dir reproduces them exactly (key order,
    # default=str values).
    from lf_eval.results_report import render_html  # imports this module
    written = json.loads(text)
    (outdir / "report.md").write_text(render_report(written))
    (outdir / "report.html").write_text(render_html(written, outdir), encoding="utf-8")
    _write_eval_sums(outdir, verified)
    echo(f"Written: {outdir}")
    return outdir


# ────────────────────────────────────────────────────────────────────── CLI


def add_eval_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("results", nargs="+", type=Path, metavar="RESULTS",
                        help="rcllf.result/1 results folder(s), as the benchmark "
                             "harness's `ros2 lf run` writes them")
    parser.add_argument("--baseline", metavar="VARIANT",
                        help="compare every other variant with this one, per sweep point "
                             "(delta = candidate - baseline)")
    parser.add_argument("--metrics", type=lambda s: [p.strip() for p in s.split(",") if p.strip()],
                        help="comma-separated metric names (default: all)")
    parser.add_argument("--option", action="append", default=[], metavar="METRIC.KEY=VALUE",
                        help="option for a metric that declares it (repeatable)")
    parser.add_argument("--include-excluded", action="store_true",
                        help="also pass runs that run_set.json excluded to the metrics")
    parser.add_argument("--allow-missing", action="append", default=[], metavar="GLOB",
                        help="step 0: waive MISSING files matching GLOB (e.g. trimmed "
                             "'runs/*/tracing/*'); changed or unlisted files still reject")
    parser.add_argument("--chain", action="append", default=[], metavar="A->B->C",
                        help="explicit chain spec (repeatable); default derives from the group's system.json")
    parser.add_argument("--backend", choices=["bt2", "cli"], default=None,
                        help="force a trace reader backend")
    parser.add_argument("-o", "--out", type=Path, metavar="DIR",
                        help="eval dir (default <first results>/eval/<UTC>/)")
    parser.add_argument("--no-plots", action="store_true", help="skip plots")


def eval_command(args: argparse.Namespace) -> int:
    try:
        evaluate(args.results, metric_names=args.metrics, baseline=args.baseline,
                 option_specs=args.option, include_excluded=args.include_excluded,
                 allow_missing=args.allow_missing, chain_specs=args.chain,
                 backend=args.backend, out=args.out, no_plots=args.no_plots)
    except EvalError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return exc.code
    return 0
