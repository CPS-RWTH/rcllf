# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``lf-eval report`` — the Markdown comparison-report layer (design §E).

Renders a human-readable comparison report above the same
compute pipeline ``lf-eval analyze`` already runs (:mod:`lf_eval.cli`'s
``_compute_all`` helper). ``lf-eval analyze`` stays the machine-readable
``metrics.json`` + plots entry point; ``report`` adds the "comparison at a
glance" document the project's release criteria ask for.

**Direct-metadata-read contract (M4/R7, frozen).** Per-run facts this module
needs — ``scenario``, ``repeat_index``, ``events_discarded``, ``started_at``,
``ended_at`` — are read by re-parsing each run directory's own
``metadata.yaml`` with ``yaml.safe_load`` (:func:`read_run_metadata_raw`),
**never** via ``RunMetadata.extra[...]`` as if it were a stable, typed field:
``extra`` is an untyped grab-bag by construction (see
:func:`lf_eval.model.run.load_metadata`), and treating a specific key within
it as guaranteed-present would silently couple this module to
``RunMetadata``'s implementation rather than to the YAML schema itself, which
is the actual frozen inter-slice contract. This also makes the report
robust to whatever subset ``cli.py``'s own ``metrics.json`` payload does or
does not carry — belt and suspenders per the session lead's directive.

**``events_discarded`` authority (frozen).** A run's
``metadata.yaml`` ``events_discarded`` (an ``lttng list <session>``-derived
sum, written by the orchestrator after ``lttng stop`` and before
``lttng destroy``) is authoritative for that run's trace-loss status. The
bt2/CTF-side cross-check this package's reader also computes
(:class:`lf_eval.reader.events.DiscardStats`, surfaced in ``metrics.json``'s
``data_quality`` block) is a separate, independent number — never a second
writer of the same field. When the two disagree, that disagreement is itself
surfaced as a warning, not silently reconciled one way or the other.

**``--system`` resolution (R4, frozen).** When no explicit ``--system`` is
given, each run directory's own ``<run-dir>/system.json`` copy is loaded
independently (the design's per-scenario/per-run copy, §D) rather than
requiring one shared path — a stale or missing copy for one run must not
silently degrade every other run's chain/gating derivation.
"""

from __future__ import annotations

import io
from pathlib import Path
from typing import Any, Optional, Sequence

from lf_eval.model.chain import Chain
from lf_eval.model.run import METADATA_FILENAME, Run
from lf_eval.model.system import SystemModel, load_system
from lf_eval.plots import style


def read_run_metadata_raw(run_dir: Path) -> dict[str, Any]:
    """Direct ``yaml.safe_load`` of a run directory's ``metadata.yaml``.

    Returns ``{}`` when the file is missing, empty, or not a mapping — the
    same tolerant shape :func:`lf_eval.model.run.load_metadata` uses, but
    without going through :class:`~lf_eval.model.run.RunMetadata` at all, per
    this module's frozen direct-read contract (see the module docstring).
    """
    import yaml

    path = Path(run_dir) / METADATA_FILENAME
    if not path.is_file():
        return {}
    with path.open() as handle:
        data = yaml.safe_load(handle) or {}
    return data if isinstance(data, dict) else {}


def resolve_run_system(run_dir: Path, explicit: Optional[Path]) -> Optional[SystemModel]:
    """Resolve the ``system.json`` a single run should be analysed against.

    :param run_dir: the run directory.
    :param explicit: an explicit ``--system`` override; when given, it wins
        for *every* run (e.g. a refsys report pointed at a different
        package's ``system.json``, design §G). When omitted, each run's own
        ``<run-dir>/system.json`` copy is used (R4) — never a single shared
        path silently defaulted from one run or from whichever scenario ran
        last.
    """
    if explicit is not None:
        return load_system(explicit)
    candidate = Path(run_dir) / "system.json"
    if candidate.is_file():
        return load_system(candidate)
    return None


def _metric_title(name: str) -> str:
    titles = {
        "e2e_latency": "End-to-end latency",
        "determinism": "Determinism",
        "cpu": "CPU utilisation",
        "drops": "Drops",
        "reproducibility": "Reproducibility (per-node callback order across runs)",
    }
    return titles.get(name, name.replace("_", " ").capitalize())


def _md_table(rows: Sequence[dict[str, Any]]) -> str:
    """Render dict rows as a GitHub-flavoured Markdown table."""
    if not rows:
        return ""
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        cells = []
        for col in columns:
            value = row.get(col)
            cells.append("" if value is None else str(value))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def _run_metadata_table(run_dirs: Sequence[Path]) -> str:
    """The metadata summary table at the top of the report (M4)."""
    rows = []
    for run_dir in run_dirs:
        raw = read_run_metadata_raw(run_dir)
        rows.append(
            {
                "run": run_dir.name,
                "mode": raw.get("mode", "unknown"),
                "scenario": raw.get("scenario", ""),
                "repeat": raw.get("repeat_index", ""),
                "duration_s": raw.get("duration", ""),
                "label": raw.get("label", ""),
                "started_at": raw.get("started_at", ""),
                "events_discarded": raw.get("events_discarded", ""),
            }
        )
    return _md_table(rows)


def _variant_summary_rows(
    run_dirs: Sequence[Path],
    metrics_payload: dict[str, Any],
) -> list[dict[str, Any]]:
    """One row per ``mode`` — the "comparison at a glance" table (§E).

    Pulls ``mode`` from each run's own ``metadata.yaml`` (never from
    ``metrics.json``'s subset, per M4) and per-metric summary numbers from
    the already-computed ``metrics_payload`` (this module's caller passes
    the same payload ``lf-eval analyze`` would write, so no metric is
    recomputed here).
    """
    mode_by_run: dict[str, str] = {
        run_dir.name: str(read_run_metadata_raw(run_dir).get("mode", "unknown"))
        for run_dir in run_dirs
    }
    modes = style.present_modes(mode_by_run.values())

    e2e_by_run = _rows_by_run(metrics_payload, "e2e_latency", "mean_ms")
    ordering_by_run = _rows_by_run(metrics_payload, "determinism", "ordering")
    cpu_by_run = _rows_by_run(metrics_payload, "cpu", "trace_cores")
    drops_by_run = _rows_by_run(metrics_payload, "drops", "drop_pct")

    rows: list[dict[str, Any]] = []
    for mode in modes:
        member_runs = [name for name, m in mode_by_run.items() if m == mode]
        rows.append(
            {
                "mode": style.mode_label(mode),
                "runs": len(member_runs),
                "mean_e2e_ms": _mean_of(e2e_by_run, member_runs),
                "ordering": _first_of(ordering_by_run, member_runs),
                "cpu_cores": _mean_of(cpu_by_run, member_runs),
                "drop_pct": _mean_of(drops_by_run, member_runs),
            }
        )
    return rows


def _rows_by_run(
    metrics_payload: dict[str, Any], metric_name: str, field: str
) -> dict[str, Any]:
    """Extract ``{run_name: field_value}`` from a metric's ``summary`` rows."""
    metric = (metrics_payload.get("metrics") or {}).get(metric_name) or {}
    out: dict[str, Any] = {}
    for row in metric.get("summary") or []:
        name = row.get("run")
        # a None value (a chain row with no sample) never hides another
        # chain's value of the same run
        if name is not None and row.get(field) is not None:
            out[name] = row[field]
    return out


def _mean_of(values: dict[str, Any], names: Sequence[str]) -> Optional[float]:
    numeric = [values[n] for n in names if isinstance(values.get(n), (int, float))]
    if not numeric:
        return None
    return round(sum(numeric) / len(numeric), 4)


def _first_of(values: dict[str, Any], names: Sequence[str]) -> Optional[Any]:
    for n in names:
        if n in values:
            return values[n]
    return None


def _drops_section(
    run_dirs: Sequence[Path],
    metrics_payload: dict[str, Any],
) -> str:
    """Drops section body, including per-run trace-loss / cross-check warnings.

    Authority split (frozen): ``events_discarded`` in a run's own
    ``metadata.yaml`` is authoritative for that run's trace-loss status. The
    bt2-derived count in ``metrics.json``'s ``data_quality`` block is an
    independent cross-check; disagreement between the two is surfaced as an
    additional warning, never silently reconciled.
    """
    drops_metric = (metrics_payload.get("metrics") or {}).get("drops") or {}
    lines = []
    if drops_metric.get("available") is False:
        lines.append(f"_{drops_metric.get('note') or 'not available'}_\n")
    elif drops_metric.get("summary"):
        lines.append(_md_table(drops_metric["summary"]))
        lines.append("")

    data_quality = metrics_payload.get("data_quality") or {}
    warnings: list[str] = []
    for run_dir in run_dirs:
        raw = read_run_metadata_raw(run_dir)
        authoritative = raw.get("events_discarded")
        if not isinstance(authoritative, int):
            continue
        if authoritative > 0:
            warning = (
                f"- **{run_dir.name}**: `events_discarded={authoritative}` "
                "(authoritative, from `metadata.yaml` / `lttng list`) — this "
                "run's drop numbers may reflect trace loss, not genuine "
                "message drops. Treat with caution."
            )
            bt2 = data_quality.get(run_dir.name) or {}
            bt2_total = bt2.get("discarded_events")
            if bt2.get("known") and isinstance(bt2_total, int) and bt2_total != authoritative:
                warning += (
                    f" (bt2 cross-check reports {bt2_total}, which disagrees "
                    "with the authoritative count — worth investigating.)"
                )
            warnings.append(warning)
        else:
            # events_discarded == 0 (clean per the authority), but the bt2
            # cross-check independently thinks otherwise — surfaced too, per
            # "if they ever disagree, that disagreement is itself diagnostic".
            bt2 = data_quality.get(run_dir.name) or {}
            bt2_total = bt2.get("discarded_events")
            if bt2.get("known") and isinstance(bt2_total, int) and bt2_total > 0:
                warnings.append(
                    f"- **{run_dir.name}**: `metadata.yaml` reports "
                    "`events_discarded=0`, but the bt2 cross-check counted "
                    f"{bt2_total} discarded event(s) in the trace — a "
                    "mismatch worth investigating, not resolved automatically."
                )

    if warnings:
        lines.append("**Data-quality warnings:**\n")
        lines.extend(warnings)
        lines.append("")

    if not lines:
        lines.append("_no publish/take data collected_\n")

    return "\n".join(lines)


def generate_report(
    run_dirs: Sequence[Path],
    metrics_payload: dict[str, Any],
    chains: Sequence[Chain],
    out: Path,
    title: str = "rcllf benchmark comparison report",
) -> Path:
    """Render ``<out>/report.md`` from an already-computed ``metrics.json`` payload.

    :param run_dirs: the run directories the report covers, in the order
        given on the command line — each one's own ``metadata.yaml`` is read
        directly (see the module docstring's frozen contract).
    :param metrics_payload: the same payload shape ``lf-eval analyze`` writes
        to ``metrics.json`` (``{"metrics": {...}, "data_quality": {...}}``).
    :param chains: the resolved chains, for the title block only.
    :param out: output directory; ``report.md`` is written directly under it
        (plots referenced by metrics are assumed already written to
        ``out/plots/`` by the same ``analyze`` pass that produced
        ``metrics_payload``).
    :param title: report heading.
    """
    run_dirs = [Path(p) for p in run_dirs]
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)

    buf = io.StringIO()
    buf.write(f"# {title}\n\n")

    if chains:
        buf.write("## Chains\n\n")
        for chain in chains:
            gated = ", ".join(h.label for h in chain.hops if h.gated)
            buf.write(f"- **{chain.name}** ({chain.source}): {' -> '.join(chain.display_nodes)}")
            if gated:
                buf.write(f"  — gated hops: {gated}")
            buf.write("\n")
        buf.write("\n")

    buf.write("## Runs\n\n")
    table = _run_metadata_table(run_dirs)
    buf.write(table if table else "_no runs_\n")
    buf.write("\n\n")

    buf.write("## Comparison at a glance\n\n")
    summary_rows = _variant_summary_rows(run_dirs, metrics_payload)
    table = _md_table(summary_rows)
    buf.write(table if table else "_no variants to compare_\n")
    buf.write("\n\n")
    if any(str(read_run_metadata_raw(d).get("mode")) == "rslcpp" for d in run_dirs):
        # A different operating point, never a matched physical comparison.
        buf.write(
            "> RSLCPP runs on simulated time as fast as possible: its clock jumps to the next "
            "scheduled event, so idle time between periods collapses. Its latency, CPU and "
            "throughput rows are descriptive only, not a matched comparison with the "
            "wall-clock-paced variants. The comparable axis is same-run ordering and "
            "reproducibility.\n\n")

    # Identity claims first, with their three statuses and codes.
    semantic = metrics_payload.get("semantic")
    if semantic is not None:
        from lf_eval.semantic.claims import render_claims_table

        buf.write("## Claims\n\n")
        buf.write(render_claims_table(semantic))
        buf.write("\n")

    metrics = metrics_payload.get("metrics") or {}
    profiles_open = False
    for name in ("reproducibility", "e2e_latency", "determinism", "cpu", "drops"):
        if name not in metrics:
            continue
        metric = metrics[name]
        if metric.get("evidence") == "heuristic_timestamp_pairing":
            # Timestamp-paired results are physical profiles, never claims.
            if not profiles_open:
                buf.write("## Physical profiles (heuristic pairing)\n\n")
                profiles_open = True
            buf.write(f"### {_metric_title(name)}\n\n")
            clock = metric.get("clock") or {}
            label = ", ".join(f"{k}: {v}" for k, v in clock.items())
            buf.write(f"_evidence: heuristic_timestamp_pairing{', ' + label if label else ''}_\n\n")
        else:
            buf.write(f"## {_metric_title(name)}\n\n")

        if name == "drops":
            buf.write(_drops_section(run_dirs, metrics_payload))
            buf.write("\n")
        else:
            if metric.get("available") is False:
                buf.write(f"_{metric.get('note') or 'not available'}_\n\n")
                # rows of an unavailable metric carry the counters that say
                # why (e.g. every activation unresolved), so they are shown
                if metric.get("summary"):
                    buf.write(_md_table(metric["summary"]))
                    buf.write("\n\n")
            else:
                if metric.get("note"):
                    buf.write(f"> {metric['note']}\n\n")
                summary = metric.get("summary") or []
                buf.write(_md_table(summary) if summary else "_no summary rows_\n")
                buf.write("\n\n")

        for plot in metric.get("plots") or []:
            buf.write(f"![{name}]({plot})\n\n")

    text = buf.getvalue()
    report_path = out / "report.md"
    report_path.write_text(text)
    return report_path
