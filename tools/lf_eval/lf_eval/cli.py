# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``lf-eval`` command-line interface.

Synopsis::

    lf-eval analyze <run-dir>... [--system system.json] [--chain A->B->C]
                                 [--metrics e2e_latency,cpu] [--out DIR]
                                 [--backend {bt2,cli}] [--keep-raw] [--no-plots]
                                 [--fail-on-missing-edge] [--include-infra-topics]
    lf-eval report <run-dir>... [--system system.json] [--chain A->B->C]
                                [--metrics ...] [--out DIR] [--title STR]
                                [--fail-on-missing-edge] [--include-infra-topics]
    lf-eval list-metrics
    lf-eval sample-cpu --patterns P[,P...] --out cpu_proc.json
                       [--warmup S] [--window S] [--cores N] [--list-pids]

``analyze`` writes ``<out>/metrics.json`` plus ``<out>/plots/*.png`` and prints a
summary table per metric. ``report`` runs the same load/compute pipeline and
additionally renders ``<out>/report.md`` — a Markdown comparison document with
per-metric sections, a per-variant summary table, and a per-run data-quality
section (see :mod:`lf_eval.report`). ``report``'s ``--system`` is optional:
when omitted, each run directory's own ``system.json`` copy is resolved
independently rather than requiring one shared path.

``--fail-on-missing-edge`` makes the ``topology`` soundness metric
(:mod:`lf_eval.metrics.topology`) CI-usable without changing the
advisory-by-default contract every other metric keeps: it passes
``fail_on_missing_edge=True`` to the metric (which sets
``data["failed"] = True`` on a ``missing_from_model`` finding) and the CLI
turns that in-band flag into a real non-zero exit code.

``--include-infra-topics`` opts back into reporting the ROS infrastructure
topics the metric excludes by default (``/rosout``, ``/parameter_events``,
``/tf``, ``/tf_static``). Note this is distinct from the action/lifecycle
edges the model *derives* (see
:func:`lf_eval.model.system.derive_infrastructure_edges`): those are compared
as declared edges rather than suppressed, so this flag does not affect them.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Optional, Sequence

from lf_eval import __version__

logger = logging.getLogger("lf_eval")


# ── helpers ───────────────────────────────────────────────────────────────


def _format_table(rows: Sequence[dict[str, Any]], indent: str = "  ") -> str:
    """Render dict rows as a fixed-width table (keys of the first row = columns)."""
    if not rows:
        return ""
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    widths = {
        col: max(len(col), *(len(_cell(row.get(col))) for row in rows))
        for col in columns
    }
    lines = [indent + "  ".join(col.ljust(widths[col]) for col in columns)]
    lines.append(indent + "  ".join("-" * widths[col] for col in columns))
    for row in rows:
        lines.append(
            indent + "  ".join(_cell(row.get(col)).ljust(widths[col]) for col in columns)
        )
    return "\n".join(lines)


def _cell(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:g}"
    return str(value)


def _add_window_args(parser: argparse.ArgumentParser) -> None:
    """Measurement window (benchmark protocol)."""
    parser.add_argument(
        "--warmup-s", type=float, default=None, metavar="S",
        help="skip S seconds after the run's first callback_start (timing and CPU metrics)",
    )
    parser.add_argument(
        "--window-s", type=float, default=None, metavar="S",
        help="then measure exactly S seconds; a run that ends earlier is refused",
    )


def _git_sha(value: str) -> str:
    sha = value.strip().lower()
    if len(sha) != 40 or any(c not in "0123456789abcdef" for c in sha):
        raise argparse.ArgumentTypeError(f"not a full 40-hex git SHA: {value!r}")
    return sha


def _finite_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError:
        number = math.nan
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError(f"not a finite number: {value!r}")
    return number


def _window(args: argparse.Namespace):
    """The Window for ``args``, or None when no window flag was given."""
    from lf_eval.model.run import Window

    warmup = getattr(args, "warmup_s", None)
    steady = getattr(args, "window_s", None)
    if warmup is None and steady is None:
        return None
    return Window(warmup_s=warmup or 0.0, steady_s=steady)


def _default_outdir() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("output/experiments") / f"analysis_{stamp}"


def _resolve_system_from_run_dirs(
    run_dirs: Sequence[Path],
) -> tuple[Optional[Any], list[Path]]:
    """Resolve a shared ``SystemModel`` from run directories' own ``system.json``.

    Per R4 (design §D/§E): every run directory of a given scenario carries a
    byte-identical, scenario-patched ``system.json`` copy (frozen by F4), so
    the first run directory with a readable copy is authoritative for the
    whole batch's chain/gating derivation. Every other run directory's copy is
    still read and compared (a cheap JSON-equality check) so a genuine
    cross-scenario mismatch is reported rather than silently picking whichever
    run happened to be listed first.

    :returns: ``(system_model_or_None, mismatched_paths)``.
    """
    import json

    from lf_eval.model import load_system

    first_data: Optional[dict] = None
    system = None
    mismatched: list[Path] = []
    for run_dir in run_dirs:
        candidate = Path(run_dir) / "system.json"
        if not candidate.is_file():
            continue
        try:
            with candidate.open() as handle:
                data = json.load(handle)
        except (OSError, ValueError):
            continue
        if system is None:
            system = load_system(candidate)
            first_data = data
        elif data != first_data:
            mismatched.append(candidate)
    return system, mismatched


# ── analyze ───────────────────────────────────────────────────────────────


class _ComputeError(Exception):
    """Raised by :func:`_compute_all` to unwind with a CLI exit code."""

    def __init__(self, code: int) -> None:
        super().__init__(f"exit {code}")
        self.code = code


def _compute_all(args: argparse.Namespace) -> dict[str, Any]:
    """Shared load/compute pipeline for both ``analyze`` and ``report``.

    Reads run dirs, resolves the system/chains, loads every run's trace, and
    computes every requested metric — exactly what ``analyze`` did inline
    before this refactor (design §E: "analyze's existing body becomes a
    shared ``_compute_all(...)`` helper both subcommands call"). Printing
    progress to stdout/stderr and writing ``metrics.json`` stay the caller's
    job, so ``report`` can reuse the computation without ``analyze``'s CLI
    table output.

    :raises _ComputeError: carrying the CLI exit code on any failure, so
        both callers translate it the same way.
    """
    from lf_eval.metrics import resolve_metrics
    from lf_eval.model import load_run, load_system, resolve_chains
    from lf_eval.model.run import WindowError
    from lf_eval.reader import BackendUnavailableError, available_backends

    run_dirs = [Path(p) for p in args.run_dirs]
    missing = [p for p in run_dirs if not p.is_dir()]
    if missing:
        for path in missing:
            print(f"error: not a directory: {path}", file=sys.stderr)
        raise _ComputeError(2)

    try:
        metrics = resolve_metrics(args.metrics)
    except KeyError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise _ComputeError(2) from exc

    system = None
    if args.system:
        system_path = Path(args.system)
        if not system_path.is_file():
            print(f"error: system.json not found: {system_path}", file=sys.stderr)
            raise _ComputeError(2)
        system = load_system(system_path)
        print(f"System: {system.name} ({len(system.nodes)} nodes) [{system_path}]")
    elif getattr(args, "resolve_system_per_run", False):
        # `report`-only default (R4): no shared --system was given, so resolve
        # from each run directory's own system.json copy rather than leaving
        # `system` unset (which would silently drop every timer and produce
        # zero gated hops, R4's own finding). Per-scenario runs are expected
        # (F4) to carry byte-identical copies; the first readable one is used
        # for the batch's chain/gating derivation, and a mismatch across the
        # other run dirs is surfaced as a warning rather than silently
        # ignored or arbitrarily overridden per run.
        system, mismatch = _resolve_system_from_run_dirs(run_dirs)
        if system is not None:
            print(
                f"System: {system.name} ({len(system.nodes)} nodes) "
                f"[{system.source_path}, resolved per-run]"
            )
        if mismatch:
            print(
                "warning: run directories disagree on system.json content "
                f"(differs from {run_dirs[0]}): {', '.join(str(p) for p in mismatch)}",
                file=sys.stderr,
            )

    try:
        chains = resolve_chains(system, args.chain)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise _ComputeError(2) from exc
    for chain in chains:
        print(
            f"Chain ({chain.source}): {' -> '.join(chain.display_nodes)}"
            + (
                f"  [gated hops: {', '.join(h.label for h in chain.hops if h.gated)}]"
                if any(h.gated for h in chain.hops)
                else ""
            )
        )
    if not chains:
        print(
            "Chain: none resolved (latency/ordering metrics will be skipped; "
            "pass --system or --chain)"
        )

    if not available_backends():
        print(f"error: {BackendUnavailableError.MESSAGE}", file=sys.stderr)
        raise _ComputeError(3)

    runs = []
    for run_dir in run_dirs:
        print(f"Reading {run_dir} ...", flush=True)
        try:
            run = load_run(run_dir, system=system, backend=args.backend, window=_window(args))
        except WindowError as exc:
            # A run that does not cover the protocol window is excluded before
            # analysis, never measured over a shorter span.
            print(f"error: {run_dir}: {exc}", file=sys.stderr)
            raise _ComputeError(2) from exc
        except BackendUnavailableError as exc:
            print(f"error: {exc}", file=sys.stderr)
            raise _ComputeError(3) from exc
        except (OSError, RuntimeError) as exc:
            print(f"error: failed to read {run_dir}: {exc}", file=sys.stderr)
            continue
        print(
            f"  mode={run.mode} nodes={len(run.timeline.invocations)} "
            f"callbacks={run.timeline.total_callback_starts} "
            f"span={run.timeline.duration_ns / 1e9:.1f}s"
        )
        runs.append(run)

    if not runs:
        print("error: no runs could be read", file=sys.stderr)
        raise _ComputeError(1)

    outdir = Path(args.out) if args.out else _default_outdir()
    plots_dir = outdir / "plots"
    outdir.mkdir(parents=True, exist_ok=True)

    payload: dict[str, Any] = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "lf_eval_version": __version__,
        "runs": [
            {
                "name": r.name,
                "path": str(r.path),
                "mode": r.mode,
                "label": r.metadata.label,
                "system": r.metadata.system,
                "duration_s": r.metadata.duration_s,
                "nodes": r.timeline.nodes,
                "callback_starts": r.timeline.total_callback_starts,
                "trace_span_s": round(r.timeline.duration_ns / 1e9, 3),
                "window": r.window.to_dict() if r.window is not None else None,
            }
            for r in runs
        ],
        "chains": [
            {
                "name": c.name,
                "nodes": list(c.display_nodes),
                "canonical_nodes": list(c.nodes),
                "source": c.source,
            }
            for c in chains
        ],
        "metrics": {},
        # bt2/CTF-side discard cross-check (M7) — independent of, and never a
        # substitute for, each run's `metadata.yaml`'s authoritative
        # `events_discarded` (an `lttng list <session>`-derived sum the
        # orchestrator writes). `lf-eval report` reads `metadata.yaml`
        # directly for the authoritative figure and cross-references this
        # block only to flag a mismatch (see lf_eval.report).
        "data_quality": {
            r.name: r.discard_stats.to_dict()
            for r in runs
            if r.discard_stats is not None
        },
    }

    options = {"include_raw": args.keep_raw}
    if args.max_gap_ms:
        options["max_gap_ns"] = int(args.max_gap_ms * 1_000_000)
    if getattr(args, "fail_on_missing_edge", False):
        options["fail_on_missing_edge"] = True
    if getattr(args, "include_infra_topics", False):
        options["include_infra_topics"] = True

    print()
    for metric in metrics:
        print(f"== {metric.name} ==")
        try:
            result = metric.compute(runs, system=system, chains=chains, **options)
        except Exception as exc:  # keep one bad metric from killing the run
            logger.exception("metric %s failed", metric.name)
            print(f"  ERROR: {exc}", file=sys.stderr)
            payload["metrics"][metric.name] = {
                "available": False,
                "note": f"error: {exc}",
            }
            continue

        if not args.no_plots:
            try:
                written = metric.plot(result, plots_dir)
                result.plots = [str(p.relative_to(outdir)) for p in written]
            except Exception as exc:
                logger.exception("plotting %s failed", metric.name)
                print(f"  WARNING: plotting failed: {exc}", file=sys.stderr)

        if not result.available:
            print(f"  skipped: {result.note or 'no data'}")
        else:
            if result.note:
                print(f"  note: {result.note}")
            if result.summary_rows:
                print(_format_table(result.summary_rows))
            for plot in result.plots:
                print(f"  plot: {plot}")
        payload["metrics"][metric.name] = result.to_dict()
        print()

    # Identity claims from the rcllf provider, per run. A run
    # without a run manifest or rcllf events still gets its claims, with the
    # codes that say why they are not supported.
    semantic_runs = {}
    for run_dir in run_dirs:
        from lf_eval.semantic.reconcile import analyze_run

        try:
            semantic_runs[Path(run_dir).name] = analyze_run(Path(run_dir))
        except Exception as exc:  # the legacy metrics must not depend on it
            print(f"  semantic: {Path(run_dir).name}: {type(exc).__name__}: {exc}", file=sys.stderr)
            # ...but the run must not vanish from the claims: fail closed.
            from lf_eval.semantic.claims import Claim, claims_block

            semantic_runs[Path(run_dir).name] = claims_block(
                [Claim(subject="semantic analysis", kind="analysis", integrity="incomplete",
                       integrity_codes=[f"analyzer_error:{type(exc).__name__}"])],
                {"error": f"{type(exc).__name__}: {exc}"})
    if semantic_runs:
        from lf_eval.semantic.claims import REPORT_SCHEMA

        payload["semantic"] = {
            "schema": REPORT_SCHEMA,
            "claims": [dict(c, subject=f"{name}: {c['subject']}")
                       for name, block in semantic_runs.items() for c in block["claims"]],
            "runs": semantic_runs,
        }

    return {
        "run_dirs": run_dirs,
        "system": system,
        "chains": chains,
        "payload": payload,
        "outdir": outdir,
        "plots_dir": plots_dir,
    }


def _topology_failed(payload: dict[str, Any]) -> bool:
    """``True`` when the ``topology`` metric set ``data["failed"]``.

    ``--fail-on-missing-edge`` asks the topology metric to flag a
    ``missing_from_model`` finding (see ``lf_eval/metrics/topology.py``); the
    metric itself has no CLI exit-code channel (every metric is
    advisory/descriptive), so the CLI is what turns that in-band flag into a
    real non-zero exit.
    """
    topology = payload.get("metrics", {}).get("topology")
    if not isinstance(topology, dict):
        return False
    return bool(topology.get("data", {}).get("failed"))


def cmd_analyze(args: argparse.Namespace) -> int:
    try:
        computed = _compute_all(args)
    except _ComputeError as exc:
        return exc.code

    outdir = computed["outdir"]
    plots_dir = computed["plots_dir"]
    payload = computed["payload"]

    metrics_path = outdir / "metrics.json"
    with metrics_path.open("w") as handle:
        json.dump(payload, handle, indent=2, default=str)

    print(f"Written: {metrics_path}")
    if not args.no_plots and plots_dir.is_dir():
        print(f"Plots:   {plots_dir}/")

    if getattr(args, "fail_on_missing_edge", False) and _topology_failed(payload):
        print(
            "error: topology metric found a runtime edge missing from the "
            "static model (--fail-on-missing-edge)",
            file=sys.stderr,
        )
        return 1
    return 0


# ── report ────────────────────────────────────────────────────────────────


def cmd_report(args: argparse.Namespace) -> int:
    from lf_eval.report import generate_report

    try:
        computed = _compute_all(args)
    except _ComputeError as exc:
        return exc.code

    outdir = computed["outdir"]
    payload = computed["payload"]

    metrics_path = outdir / "metrics.json"
    with metrics_path.open("w") as handle:
        json.dump(payload, handle, indent=2, default=str)

    report_path = generate_report(
        run_dirs=computed["run_dirs"],
        metrics_payload=payload,
        chains=computed["chains"],
        out=outdir,
        title=args.title or "rcllf benchmark comparison report",
    )

    print(f"Written: {metrics_path}")
    print(f"Report:  {report_path}")

    if getattr(args, "fail_on_missing_edge", False) and _topology_failed(payload):
        print(
            "error: topology metric found a runtime edge missing from the "
            "static model (--fail-on-missing-edge)",
            file=sys.stderr,
        )
        return 1
    return 0


# ── list-metrics ──────────────────────────────────────────────────────────


def cmd_check_runs(args: argparse.Namespace) -> int:
    """Protocol run-set check: included or excluded, with reasons."""
    from lf_eval.runset import Expectations, check_runs, run_identity

    window = _window(args)
    total = None if window is None or window.steady_s is None else window.warmup_s + window.steady_s
    expect = Expectations(git_sha=args.expected_git_sha, duration_s=args.expected_duration_s,
                          scenario=args.expected_scenario,
                          require_clean_tree=args.require_clean_tree,
                          require_uniform_identity=args.require_uniform_identity)
    run_dirs = [Path(d) for d in args.run_dirs]
    rows = check_runs(run_dirs, window_total_s=total, expect=expect)
    print(_format_table([dict(r, reasons=",".join(r["reasons"]) or "-") for r in rows]))
    excluded = sum(1 for r in rows if not r["included"])
    print(f"\n{len(rows) - excluded} included, {excluded} excluded")
    if args.out:
        doc: dict[str, Any] = {"window": None if window is None else window.to_dict(), "runs": rows}
        if expect.requested():
            # only with integrity flags, so an older run_set.json stays byte-identical
            idents = [run_identity(d) for d in run_dirs]
            doc["integrity"] = dict(expect.to_dict(), observed={
                k: sorted({i[k] for i in idents if i[k] is not None}) for k in ("git_sha", "workload")})
        Path(args.out).write_text(json.dumps(doc, indent=2) + "\n")
    return 0


def cmd_fixed_input(args: argparse.Namespace) -> int:
    """Fixed admitted-input semantic check: exit 0 only on ``pass``."""
    from lf_eval.semantic.fixed_input import CorpusError, check_set, load_corpus

    try:
        corpus = load_corpus(Path(args.corpus))
    except (OSError, CorpusError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    out = Path(args.out) if args.out else None
    report = check_set([Path(d) for d in args.run_dirs], corpus, out_dir=out)
    print(_format_table([{"run": r["run"], "passed": r["passed"], "digest": r["digest"][:16],
                          "inputs": r["counts"]["inputs"], "dispatches": r["counts"]["dispatches"],
                          "reasons": ",".join(r["reasons"]) or "-"} for r in report["runs"]]))
    print(f"\ncorpus {report['corpus']['id']} sha256 {corpus.sha256[:16]}: "
          f"{report['verdict'].upper()} {','.join(report['reasons']) or ''}")
    unchecked = report["input_hashes"]["recorded_only"]
    print(f"input hashes not independently checked (no archived copy): {','.join(unchecked) or 'none'}")
    ident = report["trace_identity"]
    print(f"trace identity: {','.join(ident['sources'])}, {ident['distinct_ctf_uuids']} distinct CTF trace UUIDs")
    if out is not None:
        (out / "fixed_input_report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return 0 if report["verdict"] == "pass" else 1


def cmd_output_check(args: argparse.Namespace) -> int:
    """Per-run output check of paced runs: exit 0 only if every run passes."""
    from lf_eval.output_check import check_run
    from lf_eval.semantic.fixed_input import CorpusError, load_corpus

    try:
        publishes = load_corpus(Path(args.corpus)).publishes
    except (OSError, CorpusError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    rows = [check_run(Path(d), args.checks, publishes, args.window_ns) for d in args.run_dirs]
    print(_format_table([{"run": r["run"], "mode": r["mode"], "passed": r["passed"],
                          "reasons": ",".join(x for c in r["checks"] for x in c["reasons"]) or "-"}
                         for r in rows]))
    if args.out:
        Path(args.out).write_text(json.dumps(rows, indent=2, sort_keys=True) + "\n")
    return 0 if all(r["passed"] for r in rows) else 1


def cmd_chain_latency(args: argparse.Namespace) -> int:
    """Message-exact end-to-end latency per declared chain of one run."""
    from lf_eval.chain_latency import chain_latency, parse_chain

    try:
        chains = [parse_chain(c) for c in args.chain]
        result = chain_latency(Path(args.run_dir), chains, args.mode)
    except (OSError, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    rows = []
    for name, c in result["chains"].items():
        row = {"chain": name, "n": c["physical"]["n"], "sinks": c["sinks"],
               "p50_ms": c["physical"]["p50_ns"], "p99_ms": c["physical"]["p99_ns"],
               "max_ms": c["physical"]["max_ns"], "unresolved": sum(c["unresolved"].values())}
        if c.get("not_traceable"):
            # a hop without message identity in the trace, never a lost sample
            row["not_traceable"] = sum(c["not_traceable"].values())
        for k in ("p50_ms", "p99_ms", "max_ms"):
            row[k] = None if row[k] is None else round(row[k] / 1e6, 3)
        if "bound" in c:
            row["outside_bound"] = c["samples_outside_bound"]
        rows.append(row)
    print(_format_table(rows))
    if args.out:
        Path(args.out).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    for reason in result["failures"]:
        print(f"FAIL {reason}", file=sys.stderr)
    return 1 if result["failures"] else 0


def cmd_list_metrics(_args: argparse.Namespace) -> int:
    from lf_eval.metrics import all_metrics

    metrics = all_metrics()
    width = max((len(m.name) for m in metrics), default=0)
    print("Available metrics:\n")
    for metric in metrics:
        flag = " (cross-run)" if metric.cross_run else ""
        print(f"  {metric.name.ljust(width)}  {metric.summary}{flag}")
    print(
        "\nAdd your own: subclass lf_eval.metrics.registry.Metric, decorate with "
        '@register("name"),\nand import it from lf_eval/metrics/__init__.py.'
    )
    return 0


# ── sample-cpu ────────────────────────────────────────────────────────────


def cmd_sample_cpu(args: argparse.Namespace) -> int:
    from lf_eval.metrics.sample_cpu import (
        discover_pids,
        format_pid_list,
        sample,
        write_report,
    )

    patterns = [p for p in args.patterns.split(",") if p]
    if not patterns:
        print("error: --patterns must list at least one substring", file=sys.stderr)
        return 2

    if args.list_pids:
        pids = discover_pids(patterns, proc_root=args.proc_root)
        print(format_pid_list(pids.keys()))
        return 0

    if not args.out:
        print("error: --out is required unless --list-pids", file=sys.stderr)
        return 2

    try:
        report = sample(
            patterns,
            warmup_s=args.warmup,
            window_s=args.window,
            proc_root=args.proc_root,
            cores=args.cores,
        )
    except LookupError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    write_report(report, args.out)
    print(
        f"sample-cpu: aggregate {report['aggregate_cores']} cores "
        f"({report['aggregate_pct_machine']}% of a "
        f"{report['n_cores_machine']}-core machine) -> {args.out}"
    )
    return 0


# ── parser ────────────────────────────────────────────────────────────────


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lf-eval",
        description="LTTng trace analysis for rcllf: trace -> metrics -> plots.",
    )
    parser.add_argument("--version", action="version", version=f"lf-eval {__version__}")
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="enable debug logging"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    analyze = sub.add_parser(
        "analyze",
        help="analyze one or more experiment run directories",
        description=(
            "Analyze LTTng traces from experiment run directories and write "
            "metrics.json + plots."
        ),
        epilog=(
            "examples:\n"
            "  lf-eval analyze output/experiments/latest --system "
            "packages/lf-analysis/pubsub_example/bringup/system.json\n"
            "  lf-eval analyze output/experiments/2026*_lfc_* "
            "output/experiments/2026*_stock_* --metrics e2e_latency,determinism"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    analyze.add_argument(
        "run_dirs", nargs="+", metavar="RUN-DIR", help="experiment run directories"
    )
    analyze.add_argument(
        "--system", metavar="PATH", help="system.json from `colcon lf-analyze`"
    )
    analyze.add_argument(
        "--chain",
        action="append",
        metavar="A->B->C",
        help="explicit chain spec (repeatable); default derives one from --system",
    )
    analyze.add_argument(
        "--metrics",
        type=lambda s: [p.strip() for p in s.split(",") if p.strip()],
        help="comma-separated metric names (default: all)",
    )
    analyze.add_argument("-o", "--out", metavar="DIR", help="output directory")
    analyze.add_argument(
        "--backend",
        choices=("bt2", "cli"),
        help="force a CTF backend (default: bt2 if importable, else the CLI)",
    )
    analyze.add_argument(
        "--max-gap-ms",
        type=float,
        default=None,
        help=("drop timestamp-paired latency samples whose gap exceeds N ms; "
              "default none, message-identity pairs ignore it"),
    )
    analyze.add_argument(
        "--keep-raw",
        action="store_true",
        help="keep raw per-sample latencies in metrics.json (large)",
    )
    analyze.add_argument("--no-plots", action="store_true", help="skip plot generation")
    _add_window_args(analyze)
    analyze.add_argument(
        "--fail-on-missing-edge",
        action="store_true",
        help=(
            "exit non-zero if the topology metric finds a runtime pub/sub "
            "edge missing from --system's static model (soundness gate; "
            "requires --metrics to include topology, the default)"
        ),
    )
    analyze.add_argument(
        "--include-infra-topics",
        action="store_true",
        help=(
            "do not exclude ROS infrastructure topics (/rosout, "
            "/parameter_events, /tf, /tf_static) from topology "
            "missing_from_model findings"
        ),
    )
    analyze.set_defaults(func=cmd_analyze)

    report = sub.add_parser(
        "report",
        help="render a Markdown comparison report across run directories",
        description=(
            "Run the same load/compute pipeline as `analyze` and render a "
            "Markdown comparison report (report.md) plus metrics.json + "
            "plots. --system is optional: when omitted, each run directory's "
            "own system.json copy is resolved independently (per-run, never "
            "one shared/last-patched path)."
        ),
        epilog=(
            "examples:\n"
            "  lf-eval report output/experiments/2026*_stock-single_default_r* "
            "output/experiments/2026*_lfc_default_r* --out output/analysis/default\n"
            "  lf-eval report output/experiments/run1 --system "
            "packages/lf-analysis/sentry_pipeline/bringup/system.json"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    report.add_argument(
        "run_dirs", nargs="+", metavar="RUN-DIR", help="experiment run directories"
    )
    report.add_argument(
        "--system",
        metavar="PATH",
        help=(
            "system.json to use for every run (default: each run directory's "
            "own <run-dir>/system.json copy, resolved independently per run)"
        ),
    )
    report.add_argument(
        "--chain",
        action="append",
        metavar="A->B->C",
        help="explicit chain spec (repeatable); default derives one from --system",
    )
    report.add_argument(
        "--metrics",
        type=lambda s: [p.strip() for p in s.split(",") if p.strip()],
        help="comma-separated metric names (default: all)",
    )
    report.add_argument("-o", "--out", metavar="DIR", help="output directory")
    report.add_argument(
        "--title", metavar="STR", help="report heading (default: a generic title)"
    )
    report.add_argument(
        "--backend",
        choices=("bt2", "cli"),
        help="force a CTF backend (default: bt2 if importable, else the CLI)",
    )
    report.add_argument(
        "--max-gap-ms",
        type=float,
        default=None,
        help=("drop timestamp-paired latency samples whose gap exceeds N ms; "
              "default none, message-identity pairs ignore it"),
    )
    report.add_argument(
        "--keep-raw",
        action="store_true",
        help="keep raw per-sample latencies in metrics.json (large)",
    )
    report.add_argument("--no-plots", action="store_true", help="skip plot generation")
    _add_window_args(report)
    report.add_argument(
        "--fail-on-missing-edge",
        action="store_true",
        help=(
            "exit non-zero if the topology metric finds a runtime pub/sub "
            "edge missing from the static model (soundness gate; requires "
            "--metrics to include topology, the default)"
        ),
    )
    report.add_argument(
        "--include-infra-topics",
        action="store_true",
        help=(
            "do not exclude ROS infrastructure topics (/rosout, "
            "/parameter_events, /tf, /tf_static) from topology "
            "missing_from_model findings"
        ),
    )
    report.set_defaults(func=cmd_report, resolve_system_per_run=True)

    check = sub.add_parser(
        "check-runs", help="protocol run-set check: included or excluded per run, with reasons")
    check.add_argument("run_dirs", nargs="+", metavar="RUN_DIR")
    check.add_argument("-o", "--out", metavar="FILE", help="also write the table as JSON")
    _add_window_args(check)
    integrity = check.add_argument_group(
        "metadata integrity (added after the protocol v1 freeze, opt-in; a missing or "
        "malformed field excludes the run). Without these flags the base check is unchanged on "
        "well-formed, loss-free runs, but now fails closed: malformed metadata, an invalid "
        "timestamp, a malformed or channel-less trace manifest, or a non-zero or unreadable "
        "loss count excludes the run instead of crashing or passing")
    integrity.add_argument("--expected-git-sha", type=_git_sha, metavar="SHA",
                           help="exclude runs whose metadata git_sha is not this full 40-hex SHA")
    integrity.add_argument("--expected-duration-s", type=_finite_float, metavar="S",
                           help="exclude runs whose metadata duration is not S seconds")
    integrity.add_argument("--expected-scenario", metavar="NAME",
                           help="exclude runs whose metadata scenario is not NAME")
    integrity.add_argument("--require-clean-tree", action="store_true",
                           help="exclude runs whose metadata git_dirty is not false")
    integrity.add_argument(
        "--require-uniform-identity", action="store_true",
        help="require one git_sha and one workload digest (scenario_params.yaml, "
             "scenario_overlay.env, system.json) across all given runs; on any disagreement "
             "every run carrying a value is excluded (no majority vote)")
    check.set_defaults(func=cmd_check_runs)

    fixed = sub.add_parser(
        "fixed-input",
        help="fixed admitted-input semantic check: identity, corpus, oracle, reproduction")
    fixed.add_argument("run_dirs", nargs="+", metavar="RUN_DIR")
    fixed.add_argument("--corpus", required=True, help="corpus manifest (rcllf.fixed_input_corpus/1)")
    fixed.add_argument("-o", "--out", metavar="DIR",
                       help="write fixed_input_report.json and one <run>.logical.txt per run")
    fixed.set_defaults(func=cmd_fixed_input)

    oc = sub.add_parser(
        "output-check",
        help="per-run output check of paced runs against the declared configuration")
    oc.add_argument("run_dirs", nargs="+", metavar="RUN_DIR")
    oc.add_argument("--checks", required=True,
                    type=lambda s: [p.strip() for p in s.split(",") if p.strip()],
                    help="comma-separated: logical-oracle, publish-accounting")
    oc.add_argument("--corpus", required=True,
                    help="corpus file whose oracle.publishes declares the topics per callback")
    oc.add_argument("--window-ns", default=None, metavar="START:END",
                    type=lambda s: tuple(int(x) for x in s.split(":", 1)),
                    help="accounting window as ns offsets from each run's first traced event "
                         "(default: the run's own, latest reader creation to last publish)")
    oc.add_argument("-o", "--out", metavar="FILE", help="write the rows as JSON")
    oc.set_defaults(func=cmd_output_check)

    cl = sub.add_parser(
        "chain-latency",
        help="message-exact end-to-end latency per chain of one run, with the lfc bound check")
    cl.add_argument("run_dir", metavar="RUN_DIR")
    cl.add_argument("--chain", action="append", required=True, metavar="A->B->C",
                    help="system.json node keys from source to sink (repeatable)")
    cl.add_argument("--mode", choices=("stock", "lfc"), required=True,
                    help="stock: ros2 source timestamps; lfc: rcllf publication identity")
    cl.add_argument("-o", "--out", metavar="FILE", help="write the samples and summary as JSON")
    cl.set_defaults(func=cmd_chain_latency)

    from lf_eval.results_eval import add_eval_arguments, eval_command

    ev = sub.add_parser(
        "eval",
        help="evaluate rcllf.result/1 results folders (same as `ros2 lf eval`)",
        description="Step 0 verification against SHA256SUMS, metrics per "
                    "(variant, sweep point) group, optional --baseline comparison; "
                    "writes <results>/eval/<UTC>/.",
    )
    add_eval_arguments(ev)
    ev.set_defaults(func=lambda a: eval_command(a))

    listing = sub.add_parser("list-metrics", help="list registered metrics")
    listing.set_defaults(func=cmd_list_metrics)

    sampler = sub.add_parser(
        "sample-cpu",
        help="sample /proc CPU usage of running nodes (run during an experiment)",
    )
    sampler.add_argument(
        "--patterns",
        required=True,
        help="comma-separated cmdline substrings identifying the target processes",
    )
    sampler.add_argument("--out", help="output JSON path (cpu_proc.json)")
    sampler.add_argument(
        "--warmup", type=float, default=10.0, help="seconds to skip before sampling"
    )
    sampler.add_argument(
        "--window", type=float, default=50.0, help="steady-state window length (s)"
    )
    sampler.add_argument(
        "--cores", type=int, default=None, help="machine core count (default: detected)"
    )
    sampler.add_argument("--proc-root", default="/proc", help=argparse.SUPPRESS)
    sampler.add_argument(
        "--list-pids",
        action="store_true",
        help="print matching pids and exit (bare ints, for taskset)",
    )
    sampler.set_defaults(func=cmd_sample_cpu)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    return args.func(args)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
