# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``ros2 lf eval`` over results folders (run spec slice R6).

Reads the recorded R3 results folder
``tools/lf_bench/tests/fixtures/runspec_r3/20261110T152946Z_sentry-r3`` (what
``ros2 lf run`` wrote in the dev container; its CTF traces and compile
commands were trimmed after recording, so its SHA256SUMS still lists them and
every eval over it waives exactly those as missing with ``--allow-missing``).

The delta checks recompute the expected value by hand from the fixture's raw
``outputs.jsonl`` rows (plain ``json`` line parsing, run dirs picked by name),
never through ``lf_eval``'s own counting code.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import pytest

from lf_eval.metrics import registry
from lf_eval.metrics.registry import Metric, MetricResult
from lf_eval.results_eval import (
    EXIT_REJECTED,
    EvalError,
    Group,
    compare_groups,
    evaluate,
    verify_folder,
)

from .dev_only import requires_lf_bench

needs_lf_bench = requires_lf_bench("the recorded R3 results folder, tests/fixtures/runspec_r3")

REPO = Path(__file__).resolve().parents[3]
R3 = REPO / "tools/lf_bench/tests/fixtures/runspec_r3/20261110T152946Z_sentry-r3"
TRIMMED = ("runs/*/tracing/*", "build/*/compile_commands.json")
HORIZON_NS = 2_000_000_000  # how.horizon.rcllf.logical = "2 sec" in sentry-r3.runspec.yaml
RUN = "20261110T153532Z_lfc-p0_default_r0"


@pytest.fixture
def folder(tmp_path: Path) -> Path:
    """A private copy of the R3 results folder without its recorded eval/ dirs."""
    dst = tmp_path / R3.name
    shutil.copytree(R3, dst, symlinks=True, ignore=shutil.ignore_patterns("eval"))
    return dst


def _eval(folder: Path, **kw) -> Path:
    kw.setdefault("allow_missing", TRIMMED)
    kw.setdefault("no_plots", True)
    return evaluate([folder], echo=lambda *_: None, **kw)


def _metrics(outdir: Path) -> dict:
    return json.loads((outdir / "metrics.json").read_text())


def _resign(folder: Path) -> None:
    """Rewrite SHA256SUMS after a deliberate edit (the test plays the run writer)."""
    lines = []
    old = {ln.partition("  ")[2] for ln in (folder / "SHA256SUMS").read_text().splitlines()}
    for name in sorted(old):
        p = folder / name
        digest = hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else "0" * 64
        lines.append(f"{digest}  {name}\n")
    (folder / "SHA256SUMS").write_text("".join(lines))


# ── hand derivation ───────────────────────────────────────────────────────


def _hand_counts(group_marker: str, topic: str, horizon_ns: int = HORIZON_NS) -> list[int]:
    """Rows of ``topic`` with tag_ns <= horizon in each run dir whose name has ``group_marker``."""
    counts = []
    for run_dir in sorted((R3 / "runs").iterdir()):
        if group_marker not in run_dir.name:
            continue
        n = 0
        with open(run_dir / "outputs.jsonl") as fh:
            for line in fh:
                row = json.loads(line)
                if row["topic"] == topic and row["tag_ns"] <= horizon_ns:
                    n += 1
        counts.append(n)
    return counts


@needs_lf_bench
def test_hand_delta_state_topic(folder: Path) -> None:
    """lfc-delays vs lfc at p0, /sentry/state: delta = candidate mean - baseline mean."""
    base = _hand_counts("_lfc-p0_", "/sentry/state")
    cand = _hand_counts("_lfc-delays-p0_", "/sentry/state")
    assert len(base) == 2 and len(cand) == 2
    hand = sum(cand) / len(cand) - sum(base) / len(base)
    assert hand == -2.0  # 420 vs 422 per run: the after-delay variant loses its last two

    doc = _metrics(_eval(folder, baseline="lfc"))
    row = next(c for c in doc["compare"]
               if c["metric"] == "output_count" and c["candidate"] == "lfc-delays|p0")
    assert row["status"] == "ok" and row["baseline"] == "lfc|p0"
    got = next(r for r in row["rows"] if r["topic"] == "/sentry/state")
    assert got["delta"] == hand, f"delta {got['delta']:+} != hand {hand:+}"
    assert (got["baseline"], got["candidate"]) == (422.0, 420.0)


@needs_lf_bench
def test_hand_delta_every_topic_both_points(folder: Path) -> None:
    doc = _metrics(_eval(folder, baseline="lfc"))
    rows = {c["candidate"]: c for c in doc["compare"] if c["metric"] == "output_count"}
    assert sorted(rows) == ["lfc-delays|p0", "lfc-delays|p1"]
    for point in ("p0", "p1"):
        for r in rows[f"lfc-delays|{point}"]["rows"]:
            if r["topic"] == "(total)":
                continue
            base = _hand_counts(f"_lfc-{point}_", r["topic"])
            cand = _hand_counts(f"_lfc-delays-{point}_", r["topic"])
            assert r["delta"] == sum(cand) / len(cand) - sum(base) / len(base), (point, r)


@needs_lf_bench
def test_option_opt_in_horizon_by_hand(folder: Path) -> None:
    doc = _metrics(_eval(folder, metric_names=["output_count"],
                         option_specs=["output_count.horizon_ns=1000000000"]))
    per_topic = doc["groups"]["lfc|p0"]["metrics"]["output_count"]["data"]["per_topic"]
    hand = _hand_counts("_lfc-p0_", "/sentry/imu", horizon_ns=1_000_000_000)
    assert per_topic["/sentry/imu"]["mean"] == sum(hand) / len(hand) == 201.0
    assert doc["groups"]["lfc|p0"]["metrics"]["output_count"]["options"] == {
        "horizon_ns": 1_000_000_000}


@needs_lf_bench
def test_default_horizon_comes_from_run_json(folder: Path) -> None:
    doc = _metrics(_eval(folder, metric_names=["output_count"]))
    assert doc["groups"]["lfc|p1"]["metrics"]["output_count"]["options"] == {
        "horizon_ns": HORIZON_NS}


@needs_lf_bench
@pytest.mark.parametrize("spec, msg", [
    ("e2e_latency.include_raw=true", "metric 'e2e_latency' declares no options"),
    ("output_count.topics=x", "declares only ['horizon_ns']"),
    ("output_count.horizon_ns=soon", "'soon' is not a int"),
    ("cpu.x=1", "metric 'cpu' is not selected"),
])
def test_option_errors(folder: Path, spec: str, msg: str) -> None:
    names = ["output_count", "e2e_latency"]
    with pytest.raises(EvalError) as exc:
        _eval(folder, metric_names=names, option_specs=[spec])
    assert msg in str(exc.value)
    assert not (folder / "eval").exists()


# ── step 0 ────────────────────────────────────────────────────────────────


@needs_lf_bench
def test_fixture_verifies_with_only_trimmed_files_waived(folder: Path) -> None:
    v = verify_folder(folder, TRIMMED)
    assert v.ok, v.problems[:5]
    assert v.waived_missing and all("/tracing/" in w or w.endswith("compile_commands.json")
                                    for w in v.waived_missing)
    assert v.sums_sha256 == hashlib.sha256((folder / "SHA256SUMS").read_bytes()).hexdigest()


@needs_lf_bench
def test_trimmed_fixture_rejected_without_waiver(folder: Path) -> None:
    with pytest.raises(EvalError) as exc:
        evaluate([folder], echo=lambda *_: None)
    assert exc.value.code == EXIT_REJECTED
    assert "missing: build/lfc-p0/compile_commands.json" in str(exc.value)


@needs_lf_bench
def test_one_byte_changed_is_rejected(folder: Path) -> None:
    target = folder / "runs" / RUN / "outputs.jsonl"
    data = bytearray(target.read_bytes())
    data[100] ^= 0x01
    target.write_bytes(bytes(data))
    with pytest.raises(EvalError) as exc:
        _eval(folder)
    assert exc.value.code == EXIT_REJECTED
    assert f"changed: runs/{RUN}/outputs.jsonl" in str(exc.value)
    assert "nothing evaluated" in str(exc.value)
    assert not (folder / "eval").exists()


@needs_lf_bench
def test_extra_file_is_rejected(folder: Path) -> None:
    (folder / "runs" / RUN / "arrivals.jsonl").write_text("{}\n")
    with pytest.raises(EvalError) as exc:
        _eval(folder)
    assert exc.value.code == EXIT_REJECTED
    assert f"unlisted: runs/{RUN}/arrivals.jsonl" in str(exc.value)
    assert not (folder / "eval").exists()


@needs_lf_bench
def test_symlink_outside_logs_is_rejected(folder: Path) -> None:
    (folder / "runs" / RUN / "bag").symlink_to("/tmp")
    with pytest.raises(EvalError) as exc:
        _eval(folder)
    assert f"symlink: runs/{RUN}/bag" in str(exc.value)


@needs_lf_bench
def test_waiver_never_covers_a_changed_file(folder: Path) -> None:
    target = folder / "runs" / RUN / "metadata.yaml"
    target.write_text(target.read_text() + "\n")
    with pytest.raises(EvalError) as exc:
        _eval(folder, allow_missing=TRIMMED + ("*",))
    assert f"changed: runs/{RUN}/metadata.yaml" in str(exc.value)


@needs_lf_bench
def test_one_rejected_folder_stops_the_eval(folder: Path, tmp_path: Path) -> None:
    other = tmp_path / "other" / "second-set"
    shutil.copytree(folder, other, symlinks=True)
    (other / "spec.yaml").write_text("tampered\n")
    with pytest.raises(EvalError) as exc:
        evaluate([folder, other], allow_missing=TRIMMED, echo=lambda *_: None)
    assert "changed: spec.yaml" in str(exc.value)
    assert not (folder / "eval").exists()


# ── step 1 / 2 ────────────────────────────────────────────────────────────


@needs_lf_bench
def test_groups_inputs_and_eval_dir(folder: Path) -> None:
    out = _eval(folder, baseline="lfc")
    doc = _metrics(out)
    assert doc["schema"] == "rcllf.metrics/1"
    assert sorted(doc["groups"]) == ["lfc-delays|p0", "lfc-delays|p1", "lfc|p0", "lfc|p1"]
    g = doc["groups"]["lfc|p0"]
    assert g["build_ref"] == "build/lfc-p0" and g["kind"] == "rcllf"
    assert len(g["runs_used"]) == 2 and g["runs_excluded"] == []
    # trace metrics: declared input missing in the trimmed fixture -> unavailable, not an error
    assert g["metrics"]["e2e_latency"]["status"] == "unavailable"
    assert "input 'trace' not captured" in g["metrics"]["e2e_latency"]["note"]
    assert g["metrics"]["output_count"]["status"] == "ok"
    # the eval records the raw manifest it verified, and its own manifest
    raw = hashlib.sha256((folder / "SHA256SUMS").read_bytes()).hexdigest()
    assert doc["inputs"][0]["sha256sums_sha256"] == raw
    assert out.parent == folder / "eval"
    lines = (out / "SHA256SUMS").read_text().splitlines()
    names = {ln.partition("  ")[2]: ln.partition("  ")[0] for ln in lines}
    assert names["../../SHA256SUMS"] == raw
    assert set(names) == {"metrics.json", "report.md", "report.html", "../../SHA256SUMS"}
    for name, digest in names.items():
        assert hashlib.sha256((out / name).read_bytes()).hexdigest() == digest
    # the eval dir is outside the raw scope: the folder still verifies
    assert verify_folder(folder, TRIMMED).ok
    assert "delta = candidate - baseline" in (out / "report.md").read_text()


@needs_lf_bench
def test_excluded_run_listed_not_computed(folder: Path) -> None:
    doc = json.loads((folder / "result.json").read_text())
    for r in doc["runs"]:
        if r["run_id"] == RUN:
            r["included"] = False
            r["reasons"] = ["test: excluded"]
    (folder / "result.json").write_text(json.dumps(doc))
    _resign(folder)
    got = _metrics(_eval(folder, metric_names=["output_count"]))["groups"]["lfc|p0"]
    assert got["runs_excluded"] == [{"run": RUN, "reasons": ["test: excluded"]}]
    assert got["metrics"]["output_count"]["data"]["per_topic"]["(total)"]["runs"] == 1
    got = _metrics(_eval(folder, metric_names=["output_count"], include_excluded=True))
    assert got["groups"]["lfc|p0"]["metrics"]["output_count"]["data"]["per_topic"][
        "(total)"]["runs"] == 2


@needs_lf_bench
def test_unknown_baseline(folder: Path) -> None:
    with pytest.raises(EvalError, match="no such variant"):
        _eval(folder, baseline="stock")


def test_not_a_results_folder(tmp_path: Path) -> None:
    with pytest.raises(EvalError, match="not a results folder"):
        evaluate([tmp_path], echo=lambda *_: None)


class _Probe(Metric):
    """Records how it was called."""

    calls: list = []
    summary = "test probe"

    def compute(self, runs, system=None, chains=(), *args, **options):
        type(self).calls.append({"runs": sorted(r.name for r in runs), "args": args,
                                 "options": options, "system": system is not None})
        return MetricResult(name=self.name, data={})


def _register(monkeypatch, name: str, **attrs) -> type:
    cls = type(f"Probe_{name}", (_Probe,), dict(attrs, calls=[], name=name))
    monkeypatch.setitem(registry._REGISTRY, name, cls)
    return cls


@needs_lf_bench
def test_metric_without_options_called_with_three_args(folder: Path, monkeypatch) -> None:
    cls = _register(monkeypatch, "probe_plain", inputs=("outputs",))
    _eval(folder, metric_names=["probe_plain"])
    assert len(cls.calls) == 4
    assert all(c["options"] == {} and c["args"] == () and c["system"] for c in cls.calls)
    assert all(len(c["runs"]) == 2 for c in cls.calls)


@needs_lf_bench
def test_cross_group_scope_sees_every_group(folder: Path, monkeypatch) -> None:
    cls = _register(monkeypatch, "probe_cross", inputs=("outputs",), scope="cross_group")
    doc = _metrics(_eval(folder, metric_names=["probe_cross"]))
    assert len(cls.calls) == 1 and len(cls.calls[0]["runs"]) == 8
    assert doc["cross_group"]["probe_cross"]["status"] == "ok"
    assert "probe_cross" not in doc["groups"]["lfc|p0"]["metrics"]


@needs_lf_bench
def test_lf_model_metric_on_rcllf_group_runs(folder: Path, monkeypatch) -> None:
    cls = _register(monkeypatch, "probe_lf", inputs=("outputs", "lf_model"))
    doc = _metrics(_eval(folder, metric_names=["probe_lf"]))
    assert len(cls.calls) == 4
    assert doc["groups"]["lfc|p0"]["metrics"]["probe_lf"]["status"] == "ok"


@needs_lf_bench
def test_lf_model_not_applicable_on_stock_and_arrivals_unavailable(folder: Path,
                                                                   monkeypatch) -> None:
    for run_dir in (folder / "runs").iterdir():
        if "_lfc-delays-" in run_dir.name:
            rj = json.loads((run_dir / "run.json").read_text())
            rj["kind"] = "stock"
            (run_dir / "run.json").write_text(json.dumps(rj))
    _resign(folder)
    _register(monkeypatch, "probe_lf", inputs=("lf_model",))
    _register(monkeypatch, "probe_arr", inputs=("arrivals",))
    doc = _metrics(_eval(folder, metric_names=["probe_lf", "probe_arr"]))
    stock = doc["groups"]["lfc-delays|p0"]["metrics"]
    assert stock["probe_lf"]["status"] == "not_applicable"
    assert stock["probe_arr"]["status"] == "unavailable"
    assert "input 'arrivals' not captured" in stock["probe_arr"]["note"]
    assert doc["groups"]["lfc|p0"]["metrics"]["probe_lf"]["status"] == "ok"


def _group(variant: str, kind: str, horizon) -> Group:
    return Group(key=f"{variant}|p0", folder=Path("."), variant=variant, point="p0",
                 kind=kind, build_ref=None, horizon=horizon, runs=[])


def test_mixed_kind_compare_needs_a_horizon_on_both_sides() -> None:
    from lf_eval.metrics import get_metric

    metric = get_metric("output_count")
    res = MetricResult(name="output_count",
                       data={"per_topic": {"(total)": {"mean": 3.0}}})
    groups = [_group("lfc", "rcllf", {"ns": HORIZON_NS}), _group("stock", "stock", None)]
    results = {g.key: {"output_count": res} for g in groups}
    rows = compare_groups(groups, results, [metric], "lfc")
    assert rows[0]["status"] == "refused"
    assert "mixed-kind comparison (rcllf vs stock)" in rows[0]["reason"]
    groups[1] = _group("stock", "stock", {"wall_s": 2.0})
    rows = compare_groups(groups, results, [metric], "lfc")
    assert rows[0]["status"] == "ok" and rows[0]["rows"][-1]["delta"] == 0.0


# ── the recorded eval dir ─────────────────────────────────────────────────


@needs_lf_bench
def test_recorded_eval_dir_matches_a_fresh_eval(folder: Path) -> None:
    """The eval dir recorded in the dev container over this fixture (checked in
    under its eval/) verifies and carries the numbers a fresh eval computes."""
    recorded = sorted((R3 / "eval").iterdir())
    assert len(recorded) == 1, recorded
    rec = recorded[0]
    for line in (rec / "SHA256SUMS").read_text().splitlines():
        digest, _, name = line.partition("  ")
        assert hashlib.sha256((rec / name).read_bytes()).hexdigest() == digest, name
    old = _metrics(rec)
    new = _metrics(_eval(folder, baseline="lfc"))
    assert old["inputs"][0]["sha256sums_sha256"] == new["inputs"][0]["sha256sums_sha256"]
    for key, g in new["groups"].items():
        assert old["groups"][key]["metrics"]["output_count"]["data"] == \
            g["metrics"]["output_count"]["data"]
    assert old["compare"] == new["compare"]


# ── review fixes ──────────────────────────────────


def _set_result_json(folder: Path, edit) -> None:
    doc = json.loads((folder / "result.json").read_text())
    edit(doc)
    (folder / "result.json").write_text(json.dumps(doc))
    _resign(folder)


@needs_lf_bench
def test_run_id_outside_runs_is_rejected(folder: Path, tmp_path: Path) -> None:
    """A signed result.json must not point a metric at bytes step 0 never read."""
    outside = tmp_path / "outside"
    shutil.copytree(folder / "runs" / RUN, outside, symlinks=True)

    def edit(doc):
        next(r for r in doc["runs"] if r["run_id"] == RUN)["run_id"] = "../../outside"
    _set_result_json(folder, edit)
    with pytest.raises(EvalError, match=r"run_id '\.\./\.\./outside' leaves the results folder") as exc:
        _eval(folder)
    assert exc.value.code == EXIT_REJECTED


@needs_lf_bench
def test_build_ref_outside_build_is_rejected(folder: Path) -> None:
    for run_dir in (folder / "runs").iterdir():
        if "_lfc-p0_" in run_dir.name:
            rj = json.loads((run_dir / "run.json").read_text())
            rj["build_ref"] = "runs"
            (run_dir / "run.json").write_text(json.dumps(rj))
    _resign(folder)
    with pytest.raises(EvalError, match=r"build_ref 'runs' is not under build/"):
        _eval(folder)


@needs_lf_bench
def test_folders_with_the_same_name_are_refused(folder: Path, tmp_path: Path) -> None:
    other = tmp_path / "b" / folder.name
    shutil.copytree(folder, other, symlinks=True)
    with pytest.raises(EvalError, match="share a name"):
        evaluate([folder, other], allow_missing=TRIMMED, no_plots=True, echo=lambda *_: None)


@needs_lf_bench
def test_baseline_comes_from_the_candidates_folder(folder: Path, tmp_path: Path) -> None:
    other = tmp_path / "second-set"
    shutil.copytree(folder, other, symlinks=True)
    doc = _metrics(evaluate([folder, other], baseline="lfc", metric_names=["output_count"],
                            allow_missing=TRIMMED, no_plots=True, echo=lambda *_: None))
    pairs = {(r["candidate"], r["baseline"]) for r in doc["compare"]}
    a, b = folder.name, other.name
    assert (f"{a}:lfc-delays|p0", f"{a}:lfc|p0") in pairs, f"wrong baseline folder: {sorted(pairs)}"
    assert (f"{b}:lfc-delays|p0", f"{b}:lfc|p0") in pairs, f"wrong baseline folder: {sorted(pairs)}"


@needs_lf_bench
def test_out_inside_the_raw_scope_is_refused(folder: Path) -> None:
    with pytest.raises(EvalError, match="outside its eval/"):
        _eval(folder, out=folder / "report")
    assert not (folder / "report").exists()
    _eval(folder, out=folder / "eval" / "mine")  # under eval/ is outside the raw scope
    verify = evaluate([folder], allow_missing=TRIMMED, no_plots=True, echo=lambda *_: None)
    assert verify.is_dir()


@needs_lf_bench
def test_cross_group_trace_metric_gets_loaded_runs_and_errors_are_contained(
        folder: Path, monkeypatch) -> None:
    import lf_eval.results_eval as re_mod

    class Loaded:
        def __init__(self, rr):
            self.name = "loaded:" + rr.name

    monkeypatch.setattr(re_mod.ResultRun, "has", lambda self, kind: True)
    monkeypatch.setattr(re_mod, "_trace_runs", lambda runs, system, backend: [Loaded(r) for r in runs])
    cls = _register(monkeypatch, "probe_xtrace", inputs=("trace",), scope="cross_group")

    class Boom(_Probe):
        def compute(self, runs, system=None, chains=()):
            raise RuntimeError("boom")
    monkeypatch.setitem(registry._REGISTRY, "probe_boom",
                        type("Probe_boom", (Boom,), dict(name="probe_boom", scope="cross_group",
                                                         inputs=("outputs",), calls=[])))
    doc = _metrics(_eval(folder, metric_names=["probe_xtrace", "probe_boom"]))
    assert all(n.startswith("loaded:") for n in cls.calls[0]["runs"]), cls.calls[0]["runs"]
    assert doc["cross_group"]["probe_boom"]["status"] == "error"
    assert "boom" in doc["cross_group"]["probe_boom"]["note"]


@needs_lf_bench
def test_deleted_result_json_is_a_step0_rejection(folder: Path) -> None:
    (folder / "result.json").unlink()
    with pytest.raises(EvalError, match="missing: result.json") as exc:
        _eval(folder)
    assert exc.value.code == EXIT_REJECTED


# ── round 2 ──────────────────────────────
# Each test fails on 955145eb with the defect's own symptom (recorded in the
# R6 round-2 journal entry), not merely with "DID NOT RAISE".


@needs_lf_bench
def test_eval_symlink_is_rejected_by_step0(folder: Path) -> None:
    """An ``eval`` symlink must not pass step 0 and redirect the eval dir."""
    (folder / "eval").symlink_to("build")
    before = {p.relative_to(folder).as_posix() for p in (folder / "build").rglob("*")}
    try:
        _eval(folder)
    except EvalError as exc:
        assert exc.code == EXIT_REJECTED, exc
        assert "symlink: eval" in str(exc)
        return
    written = sorted({p.relative_to(folder).as_posix() for p in (folder / "build").rglob("*")}
                     - before)
    pytest.fail(f"eval symlink passed step 0; the eval wrote unlisted files into the raw "
                f"tree: {written}")


@needs_lf_bench
def test_eval_that_is_a_regular_file_is_rejected(folder: Path) -> None:
    (folder / "eval").write_text("x")
    with pytest.raises(EvalError, match="unlisted: eval") as exc:
        _eval(folder)
    assert exc.value.code == EXIT_REJECTED


def _swap_after_step0(monkeypatch, rel: str, change) -> None:
    """Change ``<folder>/<rel>`` right after step 0 verified it."""
    import lf_eval.results_eval as re_mod

    real = re_mod.verify_folder

    def verify_then_swap(root, allow_missing=()):
        v = real(root, allow_missing)
        p = Path(root) / rel
        p.write_bytes(change(p.read_bytes()))
        return v
    monkeypatch.setattr(re_mod, "verify_folder", verify_then_swap)


def _drop_last_line(data: bytes) -> bytes:
    lines = data.splitlines(keepends=True)
    return b"".join(lines[:-1])


@needs_lf_bench
def test_outputs_replaced_after_step0_is_rejected(folder: Path, monkeypatch) -> None:
    rel = f"runs/{RUN}/outputs.jsonl"
    verified_rows = len((folder / rel).read_text().splitlines())
    seen: dict = {}

    class Rows(_Probe):
        def compute(self, runs, system=None, chains=()):
            for r in runs:
                seen[r.name] = len(r.outputs)
            return MetricResult(name=self.name, data={})
    monkeypatch.setitem(registry._REGISTRY, "probe_rows",
                        type("Probe_rows", (Rows,), dict(name="probe_rows", inputs=("outputs",),
                                                         calls=[])))
    _swap_after_step0(monkeypatch, rel, _drop_last_line)
    try:
        _eval(folder, metric_names=["probe_rows"])
    except EvalError as exc:
        assert exc.code == EXIT_REJECTED, exc
        assert f"{rel} changed after step 0" in str(exc)
        return
    pytest.fail(f"metric read {seen.get(RUN)} rows from {rel} replaced after step 0 "
                f"(the verified file has {verified_rows}); metrics.json attributes them to "
                "the verified SHA256SUMS")


@needs_lf_bench
@pytest.mark.parametrize("rel", ["result.json", f"runs/{RUN}/run.json", "build/lfc-p0/system.json"])
def test_json_inputs_replaced_after_step0_are_rejected(folder: Path, monkeypatch, rel: str) -> None:
    _swap_after_step0(monkeypatch, rel, lambda b: b + b"\n")
    try:
        _eval(folder, metric_names=["output_count"])
    except EvalError as exc:
        assert exc.code == EXIT_REJECTED, exc
        assert f"{rel} changed after step 0" in str(exc)
        return
    pytest.fail(f"eval completed over {rel} replaced after step 0")


@needs_lf_bench
def test_trace_run_files_rechecked_right_before_the_trace_load(folder: Path,
                                                               monkeypatch) -> None:
    import lf_eval.model as model_mod
    import lf_eval.results_eval as re_mod

    loaded: list = []

    class Loaded:
        def __init__(self, path):
            self.name, self.path = Path(path).name, path

    def fake_load_run(path, system=None, backend=None, window=None):
        loaded.append(Path(path).name)
        return Loaded(path)
    monkeypatch.setattr(model_mod, "load_run", fake_load_run)
    monkeypatch.setattr(re_mod.ResultRun, "has", lambda self, kind: True)
    _register(monkeypatch, "probe_trace", inputs=("trace",))
    _swap_after_step0(monkeypatch, f"runs/{RUN}/metadata.yaml", lambda b: b + b"\n")
    try:
        _eval(folder, metric_names=["probe_trace"])
    except EvalError as exc:
        assert exc.code == EXIT_REJECTED, exc
        assert f"runs/{RUN}/metadata.yaml changed after step 0" in str(exc)
        assert RUN not in loaded
        return
    pytest.fail(f"load_run read {RUN} after its metadata.yaml changed post step 0")


@needs_lf_bench
def test_group_keys_are_injective(folder: Path) -> None:
    """("x|y", "p0") and ("x", "y|p0") must not share one metrics.json entry."""
    def edit(doc):
        for r in doc["runs"]:
            if (r["variant"], r["point"]) == ("lfc", "p0"):
                r["variant"] = "x|y"
            elif (r["variant"], r["point"]) == ("lfc", "p1"):
                r["variant"], r["point"] = "x", "y|p0"
    _set_result_json(folder, edit)
    try:
        doc = _metrics(_eval(folder, metric_names=["output_count"]))
    except EvalError as exc:
        assert "x|y|p0" in str(exc) and "collide" in str(exc), exc
        assert exc.code == 2
        return
    pytest.fail(f"(x|y, p0) and (x, y|p0) share group key 'x|y|p0': metrics.json has "
                f"{len(doc['groups'])} groups for 4 (variant, point) pairs: {sorted(doc['groups'])}")


@needs_lf_bench
@pytest.mark.parametrize("model", ["graph_model", "lf_model"])
def test_cross_group_metric_declaring_a_model_is_refused(folder: Path, monkeypatch,
                                                         model: str) -> None:
    cls = _register(monkeypatch, "probe_xmodel", inputs=("outputs", model), scope="cross_group")
    doc = _metrics(_eval(folder, metric_names=["probe_xmodel"]))
    entry = doc["cross_group"]["probe_xmodel"]
    assert not cls.calls, (f"cross-group metric declaring {model} was called with "
                           f"system={'set' if cls.calls[0]['system'] else 'None'} "
                           f"(status {entry['status']})")
    assert entry["status"] == "not_applicable"
    assert model in entry["note"]


@needs_lf_bench
def test_result_json_absence_is_never_waivable(folder: Path) -> None:
    (folder / "result.json").unlink()
    with pytest.raises(EvalError) as exc:
        _eval(folder, allow_missing=("*",))
    assert exc.value.code == EXIT_REJECTED, (
        f"exit {exc.value.code} (not {EXIT_REJECTED}) for a waived result.json: {exc.value}")
    assert "missing: result.json" in str(exc.value)


@needs_lf_bench
def test_waived_trace_file_that_reappears_is_rejected_before_the_trace_load(
        folder: Path, monkeypatch) -> None:
    import lf_eval.model as model_mod
    import lf_eval.results_eval as re_mod

    waived = next(n for n in verify_folder(folder, TRIMMED).waived_missing
                  if n.startswith(f"runs/{RUN}/tracing/"))
    monkeypatch.setattr(model_mod, "load_run", lambda *a, **k: pytest.fail("load_run reached"))
    monkeypatch.setattr(re_mod.ResultRun, "has", lambda self, kind: True)
    _register(monkeypatch, "probe_trace", inputs=("trace",))
    real = re_mod.verify_folder

    def verify_then_restore(root, allow_missing=()):
        v = real(root, allow_missing)
        (Path(root) / waived).parent.mkdir(parents=True, exist_ok=True)
        (Path(root) / waived).write_bytes(b"not the recorded trace")
        return v
    monkeypatch.setattr(re_mod, "verify_folder", verify_then_restore)
    with pytest.raises(EvalError, match="appeared after step 0") as exc:
        _eval(folder, metric_names=["probe_trace"])
    assert exc.value.code == EXIT_REJECTED
